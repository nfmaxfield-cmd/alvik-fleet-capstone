"""The Live lab's first piece (docs/build-plans/live-lab-outline.md, 2026-10-22): a Kalman filter running live
on the moving robot during an E02 session, fed by its wheels and gyro (RAW rows, as they arrive) and the overhead
camera, so the live view can show what the filter is doing:
  - how much it trusts each sensor right now: for every reading, the share of the surprise (the reading minus
    what the filter expected) it took on board (0 = ignored, 1 = believed outright), averaged per sensor;
  - its uncertainty (an ellipse on the picture) shrinking at each camera fix and growing between them;
  - one plain sentence about the latest camera fix ("the camera disagreed by 2.3 cm: trusted it 64%").
Plus, from the runner's own records: the self-calibration settling run by run (gyro scale, drift, wheel scale per
robot), and each condition's end errors as they come in.

Nothing here steers anything: it only listens (the runner's camera samples and the robots' RAW rows).
The filter is tools/fusion/ekf.py, the same one the offline analysis uses.
"""
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (os.path.join(TOOLS, "fusion"), TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import ekf as EKF  # noqa: E402

D2R = math.pi / 180.0
SENSORS = ("wheels", "gyro", "camera")


def _num(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


class TrustEKF(EKF.EKF):
    """The fusion EKF, also keeping how much of each reading's surprise it took on board (trace(H K) / rows)."""

    def _update(self, z, h, H, R, angle_rows=(), freeze=(), estimate_cons=False):
        P = self.P
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        self.last_trust = float(np.trace(H @ K) / H.shape[0])
        y = np.atleast_1d(z - h)
        self.last_innov = float(np.linalg.norm(y))
        return super()._update(z, h, H, R, angle_rows, freeze, estimate_cons)


class LiveLab:
    """One filter for the robot that is moving (E02 moves one at a time). Thread-safe."""

    def __init__(self, keep_s=6.0):
        self.lock = threading.Lock()
        self.f = None
        self.robot = None
        self.run = None
        self.keep = keep_s
        self.events = deque(maxlen=600)          # (t, sensor, trust 0..1, innovation)
        self.last_cam = None                     # dict(t, d_cm, trust)
        self.track = deque(maxlen=400)           # (t, x, y, sx, sy, sxy) filter estimates
        self.n_cam = 0
        self.gw = 0.0                            # running sum of gyro x wheel turn rate: which way the gyro counts
        self.resets = 0
        self.last_reset = None

    # --- inputs -----------------------------------------------------------------------------------------
    def on_raw(self, robot, rec, t):
        """A RAW row from a robot (dict by column name), received at laptop time t."""
        v, w, gz = _num(rec.get("v_cms")), _num(rec.get("w_dps")), _num(rec.get("gz"))
        x, y, th = _num(rec.get("x_cm")), _num(rec.get("y_cm")), _num(rec.get("th_deg"))
        run = rec.get("run")
        with self.lock:
            if self.f is None or robot != self.robot or run != self.run:
                if x is None or y is None or th is None:
                    return
                # wheels trusted less than the offline default: E02 finds push against boards (the wheels turn,
                # the robot doesn't), and uncalibrated runs are the point of the experiment
                self.f = TrustEKF(x, y, th * D2R, t, noise=EKF.Noise(v_rel=0.08, w_rel=0.12, q_pos=0.4, cam_xy=1.0),
                                  sigma_xy=1.0, sigma_th=3.0 * D2R)
                self.robot, self.run = robot, run
                self.events.clear()
                self.track.clear()
            f = self.f
            f.predict(t)
            if v is not None:
                f.wheel_v(v)
                self.events.append((t, "wheels", f.last_trust, f.last_innov))
            if w is not None:
                f.wheel_w(w * D2R)
            if gz is not None and w is not None:
                self.gw += gz * w
            if gz is not None:
                sign = -1.0 if self.gw < 0 else 1.0     # the gyro's sign agrees with the wheels' turning
                f.gyro(sign * gz * D2R)
                self.events.append((t, "gyro", f.last_trust, f.last_innov))
            self._keep(t)

    def on_camera(self, robot, t, x, y):
        with self.lock:
            f = self.f
            if f is None or robot != self.robot or t < f.t - 1.0:
                return
            f.predict(max(t, f.t))
            before = f.s[:2].copy()
            if f.cam_d2((x, y)) > 60.0:
                # the camera and the filter are far apart (a slip it could not see coming): start again from
                # the camera, and say so
                f.reset_position(x, y, sigma_xy=1.0)
                self.resets += 1
                self.last_reset = t
            f.cam_xy((x, y))
            d = float(math.hypot(x - before[0], y - before[1]))
            self.events.append((t, "camera", f.last_trust, d))
            self.last_cam = dict(t=t, d_cm=d, trust=f.last_trust)
            self.n_cam += 1
            self._keep(t)

    def _keep(self, t):
        f = self.f
        P = f.P
        self.track.append((t, float(f.s[0]), float(f.s[1]), float(P[0, 0]), float(P[1, 1]), float(P[0, 1])))

    # --- what the view shows ------------------------------------------------------------------------------
    def state(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            if self.f is None:
                return dict(robot=None)
            ev = [e for e in self.events if e[0] >= now - self.keep]
            trust = {}
            for s in SENSORS:
                vals = [e[2] for e in ev if e[1] == s]
                trust[s] = round(float(np.mean(vals)), 3) if vals else None
            # "weights" as Nick asked for them (outline answer 9): where it is, and which way it turns
            last1 = [e[2] for e in self.events if e[1] == "camera" and e[0] >= now - 1.0]
            cam_w = 1.0 - float(np.prod([1.0 - k for k in last1])) if last1 else 0.0
            n = self.f.n
            w_now = abs(float(self.f.s[4]))
            rg = (n.g_base + n.g_rel * w_now) ** 2
            rw = (n.w_base + n.w_rel * w_now) ** 2
            gyro_share = (1 / rg) / (1 / rg + 1 / rw)
            weights = dict(position=dict(camera=round(cam_w, 3), motion=round(1 - cam_w, 3)),
                           turning=dict(gyro=round(gyro_share, 3), wheels=round(1 - gyro_share, 3)))
            t, x, y, sxx, syy, sxy = self.track[-1] if self.track else (now, float(self.f.s[0]), float(self.f.s[1]),
                                                                       1.0, 1.0, 0.0)
            vals, vecs = np.linalg.eigh(np.array([[sxx, sxy], [sxy, syy]]))
            vals = np.clip(vals, 1e-6, None)
            ang = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
            words = ""
            if self.last_reset and now - self.last_reset < 3.0:
                words = "The wheels and the camera parted ways (a wheel slip?): the filter started again from the camera."
            elif self.last_cam and now - self.last_cam["t"] < 3.0:
                c = self.last_cam
                words = ("Latest camera picture: %.1f cm from where the filter expected the robot; it took %.0f%% of "
                         "that on board (about 25 pictures a second add up)." % (c["d_cm"], 100 * c["trust"]))
            elif self.last_cam:
                words = "No camera fix for %.0f s: the filter runs on the wheels and gyro, so it grows less sure." % (
                    now - self.last_cam["t"])
            return dict(robot=self.robot, run=self.run, x=x, y=y,
                        sd_major_cm=round(2 * math.sqrt(vals[1]), 2), sd_minor_cm=round(2 * math.sqrt(vals[0]), 2),
                        angle_deg=round(ang, 1), trust=trust, weights=weights, words=words,
                        sigma_xy_cm=round(math.sqrt(0.5 * (sxx + syy)), 2), n_camera=self.n_cam)


def calibration_series(st):
    """The self-calibration per robot as it settled: [{robot, order, gyro_scale, gyro_drift_dps, wheel_scale,
    se_*}] from the runner's fits history (accepted fits only)."""
    out = []
    for robot, f in (st.get("fits") or {}).items():
        for h in f.get("history", []):
            v, fi = h.get("values") or {}, h.get("fitted") or {}
            if not h.get("accepted") or not v:
                continue
            out.append(dict(robot=robot, order=h.get("order"), gyro_scale=v.get("gyro_scale"),
                            gyro_drift_dps=v.get("gyro_drift_dps"), wheel_scale=v.get("wheel_scale"),
                            se_gyro_scale=fi.get("se_gyro_scale"), se_wheel_scale=fi.get("se_wheel_scale")))
    return out


def results_series(st):
    """Each finished run's end and far-point error (camera), by condition."""
    out = []
    for r in st.get("schedule") or []:
        if r.get("status") == "done" and r.get("end_err_cm") is not None:
            out.append(dict(cond=r["cond"], robot=r["robot"], route=r["route"], order=r["order"],
                            end_err_cm=r["end_err_cm"], far_err_cm=r.get("far_err_cm")))
    return out
