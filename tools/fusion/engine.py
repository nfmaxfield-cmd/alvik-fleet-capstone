"""Runs one filter per robot over recorded data, with sensors switched on and
off as a configuration says, and the tracking rules that keep camera
sightings tied to the right robot.

Tracking rules
  1. Possible area: a camera sighting can only belong to a robot if it lies
     where that robot could be: inside its uncertainty (a Mahalanobis gate)
     AND no further than its top speed allows since the camera last saw it.
  2. Look-alikes: sightings that fit no robot are tracked separately as
     unknown objects. They are never handed to a robot, so a look-alike that
     appears somewhere else cannot pull a robot's position over to itself.
  3. New things prove themselves: an unknown object counts as real only after
     several sightings in a short time (5 within 0.5 s by default).
  4. Robots don't vanish: when the camera loses a robot, its wheels and gyro
     carry it on; its possible area grows until it is seen again.
  5. Picked up ("kidnapped robot"): when the robot reports being lifted, its
     position is marked unknown. Once set down, it takes over a confirmed
     unknown object that moves the way its own wheels say it moves.

Late camera readings: a camera fix that arrives late (cam_latency_s) is put
in at the moment its photo was taken: the filter rewinds to just before that
moment, applies the fix, and replays the wheel and gyro readings since.
"""
import bisect
import math
import random
from collections import deque

import numpy as np

from ekf import EKF, Noise, D2R, wrap

DEFAULT = dict(
    name="wheels+gyro+camera",
    sensors=["wheel_v", "wheel_w", "gyro", "camera"],
    sensor_off={},            # sensor -> [[t0, t1], ...] seconds from run start
    cam_drop_p=0.0,           # drop each camera frame with this probability
    cam_off=[],               # [[t0, t1], ...] seconds from run start: camera blocked
    cam_off_mid_s=0.0,        # camera blocked for this long around the middle of each run
    cam_blind=[],             # [[x0, y0, x1, y1], ...] floor cm: sightings there are blocked
    cam_rate_hz=0,            # 0 = every frame; otherwise thin out to this rate
    cam_latency_s=0.0,        # camera fixes reach the filter this late
    cam_heading=True,         # use the detector's heading when it gives one
    gate_chi2=13.8,           # possible-area size (chi-square, 2 dof, 99.9%)
    vmax=30.0,                # cm/s, fastest an Alvik can move
    confirm_hits=5,           # an unknown object is real after this many sightings...
    confirm_window_s=0.5,     # ...within this long
    start_from="tape",        # tape: start at the measured start pose; camera: find it
    noise={},                 # overrides for ekf.Noise
    seed=1,
)


def config(**kw):
    c = {k: (list(v) if isinstance(v, list) else (dict(v) if isinstance(v, dict) else v)) for k, v in DEFAULT.items()}
    for k, v in kw.items():
        if k not in c:
            raise KeyError(f"unknown setting {k}")
        c[k] = v
    return c


def fnum(x, default=0.0):
    try:
        v = float(x)
        return default if math.isnan(v) else v
    except (TypeError, ValueError):
        return default


# ---------------------------------------------------------------- gyro units
def wheel_turn_signs(runs):
    """robot -> +1 or -1 so that w_dps * sign is counter-clockwise positive.
    Checked against the wheels themselves (right wheel faster = turning
    left), since the library's sign convention isn't documented."""
    acc = {}
    for run in runs:
        a = acc.setdefault(run.robot, 0.0)
        for r in run.raw:
            w, d = fnum(r.get("w_dps")), fnum(r.get("wr_rpm")) - fnum(r.get("wl_rpm"))
            if abs(w) > 20 and not math.isnan(d):
                a += w * d
        acc[run.robot] = a
    return {robot: (-1.0 if a < 0 else 1.0) for robot, a in acc.items()}


def gyro_factors(runs, wsign=None):
    """robot -> factor turning RAW gz into rad/s with the floor's sign
    (counter-clockwise positive). Only the SIGN and UNIT (deg/s or rad/s) are
    taken from comparing with the wheels; the scale stays the gyro's own, so
    the gyro remains an independent sensor."""
    acc = {}
    for run in runs:
        a = acc.setdefault(run.robot, [0.0, 0.0])
        ws = (wsign or {}).get(run.robot, 1.0)
        for r in run.raw:
            w, g = fnum(r.get("w_dps")) * ws, fnum(r.get("gz"))
            if abs(w) > 20:
                a[0] += g * w
                a[1] += w * w
    out = {}
    for robot, (gw, ww) in acc.items():
        if ww == 0:
            out[robot] = (D2R, "no turns seen; assuming deg/s")
            continue
        k = gw / ww
        if 0.3 < abs(k) < 3:
            out[robot] = (math.copysign(D2R, k), "deg/s")
        elif 0.004 < abs(k) < 0.06:
            out[robot] = (math.copysign(1.0, k), "rad/s")
        else:
            out[robot] = (math.copysign(D2R, k), f"unclear (gyro/wheel ratio {k:.3g}); assuming deg/s")
    return out


# ---------------------------------------------------------------- one robot
class Tracked:
    """One robot's filter plus its event history, so late readings can be
    slotted in at the right moment."""

    def __init__(self, run, cfg, gyro_factor):
        self.run = run
        self.cfg = cfg
        self.gf = gyro_factor
        self.t0 = run.raw[0]["t"]
        self.t1 = run.raw[-1]["t"]
        self.noise = Noise(**cfg["noise"])
        self.lost_state = False
        self.was_lifted = False
        self.events = []      # (t, order, kind, data), applied in this order
        self.snaps = []       # filter snapshot taken BEFORE each event
        if cfg["start_from"] == "tape" and run.tape_start:
            x, y, h = run.tape_start
            self.ekf = EKF(x, y, h * D2R, self.t0, self.noise)
        else:
            self.ekf = EKF(0.0, 0.0, 0.0, self.t0, self.noise)
            self.ekf.lost()
            self.lost_state = True
        self.cam_used = 0
        self.cam_blocked = 0
        self.live = []        # (t, x, y, th): what the robot would have known at each moment
        self.kidnaps = []

    def off(self, sensor, t):
        return any(a <= t - self.t0 <= b for a, b in self.cfg["sensor_off"].get(sensor, []))

    def _apply(self, ev):
        t, _, kind, d = ev
        k = self.ekf
        k.predict(t)
        if kind == "raw":
            s = self.cfg["sensors"]
            if d["lifted"]:
                return
            v, w_wheel, g = d["v"], d["w"], d["g"]
            still = d["still"]
            if "wheel_v" in s and not self.off("wheel_v", t):
                if still:
                    k.zupt()
                else:
                    k.wheel_v(v)
            if "wheel_w" in s and not self.off("wheel_w", t) and not still:
                k.wheel_w(w_wheel)
            if "gyro" in s and not self.off("gyro", t):
                k.gyro(g)
        elif kind == "cam":
            k.cam_xy((d["x"], d["y"]))
            if d.get("heading") is not None and self.cfg["cam_heading"]:
                k.cam_th(d["heading"] * D2R)
        elif kind == "lost":
            k.lost()
        elif kind == "reset":
            k.reset_position(d["x"], d["y"], 1.0)

    def add(self, ev):
        """Apply an event; if it is older than ones already applied, rewind and replay."""
        i = bisect.bisect_right([(e[0], e[1]) for e in self.events], (ev[0], ev[1])) if (
            self.events and (ev[0], ev[1]) < (self.events[-1][0], self.events[-1][1])) else len(self.events)
        if i == len(self.events):
            self.snaps.append(self.ekf.snapshot())
            self.events.append(ev)
            self._apply(ev)
            return
        self.ekf.restore(self.snaps[i])
        self.events.insert(i, ev)
        del self.snaps[i:]
        for e in self.events[i:]:
            self.snaps.append(self.ekf.snapshot())
            self._apply(e)

    def state_at(self, t):
        """Predicted state and covariance at time t (t may be in the past),
        without changing the filter."""
        i = bisect.bisect_right([e[0] for e in self.events], t)
        if i >= len(self.events):
            return self.ekf.predicted(t)
        keep = self.ekf.snapshot()
        self.ekf.restore(self.snaps[i])   # state after every event up to time t
        out = self.ekf.predicted(t)
        self.ekf.restore(keep)
        return out

    def possible(self, t, xy):
        """Rule 1: is this sighting where the robot could be? Returns d2 or None."""
        s, P = self.state_at(t)
        S = P[:2, :2] + np.eye(2) * self.noise.cam_xy ** 2
        d = np.asarray(xy) - s[:2]
        d2 = float(d @ np.linalg.solve(S, d))
        if d2 > self.cfg["gate_chi2"]:
            return None
        since = max(0.0, t - self.ekf.last_cam)
        sigma = math.sqrt(max(P[0, 0], P[1, 1]))
        if math.hypot(*d) > self.cfg["vmax"] * since + 3 * sigma + 2.0:
            return None
        return d2

    def trajectory(self):
        """Final estimate at every wheel reading, with all information applied."""
        out = []
        for ev, snap in zip(self.events, self.snaps[1:] + [self.ekf.snapshot()]):
            if ev[2] == "raw":
                s, P = snap[0], snap[1]
                out.append((ev[0], s[0], s[1], s[2], math.sqrt(P[0, 0]), math.sqrt(P[1, 1])))
        return out


# ---------------------------------------------------------------- unknown objects
class Unknowns:
    """Rules 2 and 3: sightings no robot claims, tracked separately."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.tracks = []   # dict(id, x, y, t, hits deque, confirmed, vx, vy)
        self.next_id = 1
        self.confirmed_ever = []

    def add(self, t, x, y):
        best, bd = None, None
        for tr in self.tracks:
            dt = t - tr["t"]
            d = math.hypot(x - tr["x"], y - tr["y"])
            if d <= 4.0 + self.cfg["vmax"] * dt and (bd is None or d < bd):
                best, bd = tr, d
        if best is None:
            best = dict(id=self.next_id, x=x, y=y, t=t, hits=deque(), confirmed=False, vx=0.0, vy=0.0, first=t)
            self.next_id += 1
            self.tracks.append(best)
        else:
            dt = max(t - best["t"], 1e-3)
            best["vx"] = 0.7 * best["vx"] + 0.3 * (x - best["x"]) / dt
            best["vy"] = 0.7 * best["vy"] + 0.3 * (y - best["y"]) / dt
            best["x"], best["y"], best["t"] = x, y, t
        best["hits"].append(t)
        while best["hits"] and t - best["hits"][0] > self.cfg["confirm_window_s"]:
            best["hits"].popleft()
        if not best["confirmed"] and len(best["hits"]) >= self.cfg["confirm_hits"]:
            best["confirmed"] = True
            self.confirmed_ever.append(dict(id=best["id"], t=t, x=x, y=y))
        return best

    def prune(self, t):
        self.tracks = [tr for tr in self.tracks if t - tr["t"] < 1.5]


# ---------------------------------------------------------------- engine
def run_engine(runs, detections, cfg, gyro_f=None):
    """Process all runs (and camera sightings) together, in the order the
    information would have reached the robots. Returns robot run_id -> Tracked,
    and the unknown-object tracker."""
    rng = random.Random(cfg["seed"])
    wsign = wheel_turn_signs(runs)
    gyro_f = gyro_f or gyro_factors(runs, wsign)
    use_cam = "camera" in cfg["sensors"] and detections
    events = []   # (arrival, order, payload)
    for ri, run in enumerate(runs):
        gf = gyro_f.get(run.robot, (D2R, ""))[0]
        ws = wsign.get(run.robot, 1.0)
        for r in run.raw:
            v = fnum(r.get("v_cms"))
            w = fnum(r.get("w_dps")) * D2R * ws
            # still = wheels not turning at all (a turn in place moves the wheels too)
            still = (abs(fnum(r.get("wl_rpm"))) < 0.2 and abs(fnum(r.get("wr_rpm"))) < 0.2
                     and abs(v) < 0.1 and abs(fnum(r.get("w_dps"))) < 0.5)
            d = dict(v=v, w=w, g=fnum(r.get("gz")) * gf, still=still, lifted=fnum(r.get("lifted")) > 0.5)
            events.append((r["t"], 0, ("raw", ri, d)))
    if use_cam:
        frames = {}
        for det in detections:
            frames.setdefault((det["frame"], round(det["t"], 3)), []).append(det)
        last_kept = -1e9
        for (_, t), dets in sorted(frames.items(), key=lambda kv: kv[0][1]):
            if cfg["cam_rate_hz"] and t - last_kept < 1.0 / cfg["cam_rate_hz"] - 1e-3:
                continue
            if cfg["cam_drop_p"] and rng.random() < cfg["cam_drop_p"]:
                continue
            last_kept = t
            dets = [d for d in dets if not any(x0 <= d["x"] <= x1 and y0 <= d["y"] <= y1
                                               for x0, y0, x1, y1 in cfg["cam_blind"])]
            events.append((t + cfg["cam_latency_s"], 1, ("cam", t, dets)))
    events.sort(key=lambda e: (e[0], e[1]))

    tracked = {}
    unknown = Unknowns(cfg)
    order = 0
    for arrival, _, payload in events:
        order += 1
        if payload[0] == "raw":
            _, ri, d = payload
            run = runs[ri]
            tr = tracked.get(run.run_id)
            if tr is None:
                tr = tracked[run.run_id] = Tracked(run, cfg, gyro_f.get(run.robot, (D2R, ""))[0])
            if d["lifted"] and not tr.was_lifted:          # rule 5: picked up
                tr.was_lifted = True
                tr.lost_state = True
                tr.kidnaps.append(arrival)
                tr.add((arrival, order, "lost", None))
            elif not d["lifted"]:
                tr.was_lifted = False
            tr.add((arrival, order, "raw", d))
            s = tr.ekf.s
            tr.live.append((arrival, s[0], s[1], s[2]))
            continue
        _, t_cap, dets = payload
        active = [tr for tr in tracked.values() if tr.t0 - 0.2 <= t_cap <= tr.t1 + 0.3]
        pairs = []
        for di, det in enumerate(dets):
            for tr in active:
                if tr.lost_state:
                    continue
                d2 = tr.possible(t_cap, (det["x"], det["y"]))
                if d2 is not None:
                    pairs.append((d2, di, tr))
        pairs.sort(key=lambda p: p[0])
        used_d, used_t = set(), set()
        for d2, di, tr in pairs:                           # closest pairs first, one each
            if di in used_d or id(tr) in used_t:
                continue
            used_d.add(di)
            used_t.add(id(tr))
            mid = (tr.t1 - tr.t0) / 2
            if any(a <= t_cap - tr.t0 <= b for a, b in cfg["cam_off"]) or (
                    cfg["cam_off_mid_s"] and abs(t_cap - tr.t0 - mid) <= cfg["cam_off_mid_s"] / 2):
                tr.cam_blocked += 1
                continue
            tr.add((t_cap, order, "cam", dets[di]))
            tr.cam_used += 1
        for di, det in enumerate(dets):                     # rules 2 and 3
            if di in used_d:
                continue
            ut = unknown.add(t_cap, det["x"], det["y"])
            for tr in active:                               # rule 5: re-find a set-down robot
                if tr.lost_state and not tr.was_lifted and ut["confirmed"]:
                    robot_speed = abs(tr.ekf.s[3])
                    track_speed = math.hypot(ut["vx"], ut["vy"])
                    if abs(track_speed - robot_speed) < max(3.0, 0.5 * robot_speed):
                        tr.add((t_cap, order, "reset", dict(x=det["x"], y=det["y"])))
                        tr.lost_state = False
                        unknown.tracks.remove(ut)
                        break
        unknown.prune(t_cap)
    return tracked, unknown
