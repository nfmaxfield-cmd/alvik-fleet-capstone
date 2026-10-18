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

Identity (only when the setup uses "blink" or motion_identity; otherwise the
rules above run exactly as before)
  Every sighting gets a probability for each hypothesis: each robot whose run
  is going, "elsewhere" (a robot name that isn't running now) and "none" (not
  a robot). Two clues, combined by naive Bayes (treated as independent, flat
  prior, multiplied and renormalised):
    blink   blink.py's per-track name probabilities. A detector track takes
            them from a blink light 5 cm or closer in the same frame and
            keeps them between flashes with fading confidence (mixed with
            "no idea" by exp(-time since that light / name_carry_s)); the name
            is dropped when the track comes within 6 cm of another object.
    motion  each camera track's displacement over motion_window_s compared
            with each robot's own wheel+gyro displacement over the same time
            (turned to the floor by the robot's heading estimate): a 2-D
            Gaussian likelihood; "none" gets a broad one (motion_none_cm).
            Windows multiply (with forgetting, motion_decay); a window in
            which nothing moved more than motion_min_cm says nothing.
  If blink alone gives one robot blink_name_p (0.9) or more, the sighting may
  only go to that robot, and only if it passes that robot's possible area;
  otherwise it goes to the unknown-object tracker carrying the name (so rule
  5 can't hand it to another robot either). If only blink x motion reaches
  0.9, the position must agree too: with each robot's position likelihood
  multiplied in (outside its possible area = impossible; "none" = 1 /
  clutter_cm2) that robot must still have 0.9. Below that, the normal rules
  apply, except that robots the clues (x position) put under id_exclude_p are
  not candidates. A camera track's motion record is forgotten whenever another
  object comes within 6 cm of it (tracks can swap there).
  Each fix records where its identity came from: blink, motion, both (each
  clue alone was enough), combined (only together) or position (normal rules).

Heading from motion (heading_from_motion): when a robot drives forward (both
wheels faster than hfm_wheel_min_cms, turning under hfm_turn_max_dps, for the
whole hfm_window_s) and its camera fixes moved faster than hfm_track_min_cms,
the direction they moved is a heading fix, noise sqrt(2) x position noise /
distance moved (so it shrinks with speed), plus hfm_floor_deg.
Gain measured in simulation: none when camera position fixes exist (the
filter already turns them into heading as the robot drives); it may help
when only a detector without heading is available.
Calibration rolls / start nudges (calibration_results.csv) are heading fixes
at their times, in setups that opt in (calib_heading).
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
    vmax="auto",              # cm/s, fastest a robot can move; auto = 1.5x the fastest wheel speed in the data + 5
    confirm_hits=5,           # an unknown object is real after this many sightings...
    confirm_window_s=0.5,     # ...within this long
    start_from="tape",        # tape: start at the measured start pose; camera: find it
    noise={},                 # overrides for ekf.Noise
    seed=1,
    # --- blink lights and identity (sensor "blink"; none of these change setups without it
    #     unless motion_identity / heading_from_motion are switched on)
    blink_name_p=0.9,         # a sighting goes to a robot BY NAME only when P(that robot) >= this
    blink_identity=True,      # use blink.py's name probabilities; False = blink lights are anonymous positions
    led_offset_cm="auto",     # light (forward, left) of the robot centre, cm; auto = calibration_results.csv,
                              # else runs.LED_OFFSET_CM (2.75, 0)
    motion_identity=False,    # identity from comparing camera-track motion with each robot's wheels + gyro
    motion_window_s=0.5,      # window for that comparison
    motion_min_cm=1.5,        # a window where nothing moved this far says nothing about identity
    motion_none_cm=10.0,      # spread of a non-robot object's displacement per window (cm)
    motion_decay=0.7,         # older windows count this much less per window (a track can switch objects)
    id_exclude_p=0.05,        # robots below this probability are not candidates under the normal rules
    det_blink_link_cm=5.0,    # a detector sighting this close to a blink light shares its name probabilities
    name_carry_s=5.0,         # ...and its track keeps them, fading with this time constant (0.6 s -> still 0.9)
    clutter_cm2=19200.0,      # position likelihood of "not this robot" = 1 / this area (the camera's view, cm^2)
    heading_from_motion=False,
    hfm_window_s=1.0,         # direction of travel over this long
    hfm_wheel_min_cms=3.0,    # both wheels at least this fast, forward
    hfm_turn_max_dps=10.0,    # and turning less than this (gyro), for the whole window
    hfm_track_min_cms=3.0,    # and the camera fixes moved at least this fast
    hfm_floor_deg=1.0,        # added to the heading noise (rolling straight is assumed: no side slip)
    hfm_gate_chi2=16.0,       # drop a heading fix this far (squared sigmas) from the estimate
    wheel_radius_cm=1.7,      # turns RAW wl_rpm / wr_rpm into cm/s (nominal; to be measured)
    calib_heading=False,      # opt in: calibration_results.csv start-nudge headings become heading fixes
                              # (off by default so the older setups stay comparable when the file is given)
    calib_heading_sigma_deg=0.8,  # calib.py vs simulator truth: mean 0.45-0.62 deg, max 1.33 deg
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


def fastest_speed(runs):
    """Top speed (cm/s) for the "possible area" rule, from the robots' own
    wheel speeds, with room to spare for slips and pushes."""
    top = 0.0
    for run in runs:
        for r in run.raw:
            top = max(top, abs(fnum(r.get("v_cms"))))
    return max(15.0, 1.5 * top + 5.0)


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
        self.ev_keys = []     # (t, order) of each event, for bisect
        self.ev_t = []
        self.odo = []         # (t, X, Y, TH, vl_cms, vr_cms, turn_rad_s, lifted): own wheels+gyro only
        self.fixes = []       # dict(t, frame, kind, x, y, source) for every camera fix used
        self.id_sources = {}  # source -> count
        self.hdg_used = {"motion": 0, "calib": 0}
        self.hdg_rejected = 0
        self.last_hfm = -1e9
        self.fix_hist = {"det": deque(), "blink": deque()}   # (t, x, y) of assigned fixes, for heading from motion
        self.led = (0.0, 0.0)
        self._odo_v, self._odo_lifted = 0.0, False

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
        elif kind == "blink":
            k.cam_light((d["x"], d["y"]), d["off"])
        elif kind == "hdg":
            k.heading(d["th"], d["sigma"])
        elif kind == "lost":
            k.lost()
        elif kind == "reset":
            k.reset_position(d["x"], d["y"], d.get("sigma", 1.0))

    def add(self, ev):
        """Apply an event; if it is older than ones already applied, rewind and replay."""
        key = (ev[0], ev[1])
        i = bisect.bisect_right(self.ev_keys, key) if (self.events and key < self.ev_keys[-1]) else len(self.events)
        if i == len(self.events):
            self.snaps.append(self.ekf.snapshot())
            self.events.append(ev)
            self.ev_keys.append(key)
            self.ev_t.append(ev[0])
            self._apply(ev)
            return
        self.ekf.restore(self.snaps[i])
        self.events.insert(i, ev)
        self.ev_keys.insert(i, key)
        self.ev_t.insert(i, ev[0])
        del self.snaps[i:]
        for e in self.events[i:]:
            self.snaps.append(self.ekf.snapshot())
            self._apply(e)

    def state_at(self, t):
        """Predicted state and covariance at time t (t may be in the past),
        without changing the filter."""
        i = bisect.bisect_right(self.ev_t, t)
        if i >= len(self.events):
            return self.ekf.predicted(t)
        keep = self.ekf.snapshot()
        self.ekf.restore(self.snaps[i])   # state after every event up to time t
        out = self.ekf.predicted(t)
        self.ekf.restore(keep)
        return out

    def possible(self, t, xy, off=None):
        """Rule 1: is this sighting where the robot could be? Returns d2 or None.
        With `off` (forward, left) the sighting is the robot's light, which
        should be at position + R(heading) off."""
        g = self.gate(t, xy, off)
        return None if g is None else g[0]

    def gate(self, t, xy, off=None):
        """possible(), also returning the log of the sighting's position
        likelihood under this robot (2-D Gaussian), or None."""
        s, P = self.state_at(t)
        if off is None:
            S = P[:2, :2] + np.eye(2) * self.noise.cam_xy ** 2
            d = np.asarray(xy) - s[:2]
        else:
            h, H = self.ekf.light_h(off, s)
            S = H @ P @ H.T + np.eye(2) * self.noise.blink_xy ** 2
            d = np.asarray(xy) - h
        d2 = float(d @ np.linalg.solve(S, d))
        if d2 > self.cfg["gate_chi2"]:
            return None
        since = max(0.0, t - self.ekf.last_cam)
        sigma = math.sqrt(max(P[0, 0], P[1, 1]))
        if math.hypot(*d) > self.cfg["vmax"] * since + 3 * sigma + 2.0:
            return None
        return d2, -d2 / 2 - 0.5 * math.log(4 * math.pi * math.pi * max(np.linalg.det(S), 1e-12))

    # ---- own motion (wheels + gyro only), for identity and heading from motion
    def odo_step(self, t, d, use_gyro):
        """Dead-reckon the robot's own motion (no camera) in its own frame."""
        lifted = d["lifted"]
        if not self.odo:
            self.odo.append((t, 0.0, 0.0, 0.0, d["vl"], d["vr"], 0.0, lifted))
            return
        tp, X, Y, TH, vl, vr, turn, _ = self.odo[-1]
        dt = max(0.0, t - tp)
        pv = self._odo_v
        if not self._odo_lifted:
            X += pv * math.cos(TH + turn * dt / 2) * dt
            Y += pv * math.sin(TH + turn * dt / 2) * dt
            TH += turn * dt
        nturn = (d["g"] - self.ekf.s[5]) if use_gyro else d["w"]
        self.odo.append((t, X, Y, TH, d["vl"], d["vr"], nturn, lifted))
        self._odo_v, self._odo_lifted = d["v"], lifted

    def odo_at(self, t):
        """(X, Y, TH) of the own-motion log at time t (linear), or None outside it."""
        o = self.odo
        if not o or t < o[0][0] - 1e-6 or t > o[-1][0] + 0.05:
            return None
        i = bisect.bisect_right(self._odo_t(), t)
        if i >= len(o):
            return o[-1][1:4]
        if i == 0:
            return o[0][1:4]
        a, b = o[i - 1], o[i]
        f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return tuple(a[j] + f * (b[j] - a[j]) for j in (1, 2, 3))

    def _odo_t(self):
        tc = self.__dict__.setdefault("_odo_tc", [])
        tc.extend(o[0] for o in self.odo[len(tc):])
        return tc

    def odo_rows(self, ta, tb):
        ts = self._odo_t()
        return self.odo[bisect.bisect_left(ts, ta):bisect.bisect_right(ts, tb)]

    def live_heading(self, t):
        if not self.live:
            return self.ekf.s[2]
        tc = self.__dict__.setdefault("_live_tc", [])
        tc.extend(p[0] for p in self.live[len(tc):])
        i = min(max(bisect.bisect_right(tc, t) - 1, 0), len(self.live) - 1)
        return self.live[i][3]

    def own_displacement(self, ta, tb, off=(0.0, 0.0)):
        """Floor-frame displacement over [ta, tb] by the robot's own wheels and
        gyro, turned to the floor by its heading estimate at ta; for a light at
        `off`, plus the light's swing as the robot turns. None if unknown
        (outside the log, or lifted in between)."""
        a, b = self.odo_at(ta), self.odo_at(tb)
        if a is None or b is None or any(r[7] for r in self.odo_rows(ta, tb)):
            return None
        tha = self.live_heading(ta)
        phi = tha - a[2]
        dx, dy = b[0] - a[0], b[1] - a[1]
        c, s = math.cos(phi), math.sin(phi)
        Dx, Dy = c * dx - s * dy, s * dx + c * dy
        if off[0] or off[1]:
            thb = tha + (b[2] - a[2])
            Dx += rot(off, thb)[0] - rot(off, tha)[0]
            Dy += rot(off, thb)[1] - rot(off, tha)[1]
        return Dx, Dy

    def trajectory(self):
        """Final estimate at every wheel reading, with all information applied."""
        out = []
        for ev, snap in zip(self.events, self.snaps[1:] + [self.ekf.snapshot()]):
            if ev[2] == "raw":
                s, P = snap[0], snap[1]
                out.append((ev[0], s[0], s[1], s[2], math.sqrt(P[0, 0]), math.sqrt(P[1, 1])))
        return out


def rot(off, th):
    c, s = math.cos(th), math.sin(th)
    return c * off[0] - s * off[1], s * off[0] + c * off[1]


# ---------------------------------------------------------------- heading from motion
def hfm_gate(rows, cfg):
    """Was the robot driving forward, not turning much, for all these own-motion
    rows (t, X, Y, TH, vl_cms, vr_cms, turn_rad_s, lifted)?"""
    if len(rows) < 2:
        return False
    return all(r[4] > cfg["hfm_wheel_min_cms"] and r[5] > cfg["hfm_wheel_min_cms"] and not r[7]
               and abs(r[6]) < cfg["hfm_turn_max_dps"] * D2R for r in rows)


def hfm_heading(pa, pb, ta, tb, turn_since_mid, sigma_xy, cfg):
    """Heading (rad, at tb) from the direction the robot's centre moved from pa
    (time ta) to pb (tb), or None if it moved too slowly. turn_since_mid: how
    much the robot turned from the middle of the window to tb (own gyro), since
    a chord points along the heading at its middle. Noise: sqrt(2) sigma_xy /
    distance (two noisy end points), plus hfm_floor_deg."""
    dx, dy = pb[0] - pa[0], pb[1] - pa[1]
    dist = math.hypot(dx, dy)
    if tb <= ta or dist / (tb - ta) < cfg["hfm_track_min_cms"]:
        return None
    return wrap(math.atan2(dy, dx) + turn_since_mid), math.sqrt(2) * sigma_xy / dist + cfg["hfm_floor_deg"] * D2R


# ---------------------------------------------------------------- identity
def blink_dist(hyps, probs, p_none):
    """blink.py's name probabilities spread over the hypotheses: each running
    robot, "elsewhere" (names not running now) and "none". A robot without its
    own column takes p_other. Floor of 1e-4 (blink.py rounds to 3 decimals)."""
    out, used = {}, set()
    for h in hyps:
        if h in ("none", "elsewhere"):
            continue
        n = h if h in probs else "other"
        out[h] = probs.get(n, 0.0)
        used.add(n)
    if "none" in hyps:
        out["none"] = p_none
    if "elsewhere" in hyps:
        out["elsewhere"] = sum(p for n, p in probs.items() if n not in used)
    return normalise({h: max(p, 1e-4) for h, p in out.items()})


def normalise(d):
    tot = sum(d.values())
    return {k: v / tot for k, v in d.items()} if tot > 0 else {k: 1.0 / len(d) for k in d}


def combine_identity(p_blink, p_motion, tau):
    """Naive Bayes: P(h | blink, motion) proportional to P(h | blink) P(h | motion)
    (each found with a flat prior, the clues taken as independent; dividing by
    the flat prior once changes nothing after renormalising). Either may be None
    (no clue). Returns (posterior, best, source) where source says which clue
    named `best` at >= tau: both / blink / motion / combined, or None if the
    posterior is under tau."""
    hyps = list((p_blink or p_motion or {}).keys())
    if not hyps:
        return {}, None, None
    post = normalise({h: (p_blink[h] if p_blink else 1.0) * (p_motion[h] if p_motion else 1.0) for h in hyps})
    best = max(post, key=post.get)
    if post[best] < tau:
        return post, best, None
    b = bool(p_blink) and p_blink[best] >= tau
    m = bool(p_motion) and p_motion[best] >= tau
    return post, best, ("both" if b and m else "blink" if b else "motion" if m else "combined")


def motion_loglik(d, D, sigma):
    """log N(d; D, sigma^2 I) in 2-D."""
    e2 = (d[0] - D[0]) ** 2 + (d[1] - D[1]) ** 2
    return -e2 / (2 * sigma * sigma) - math.log(2 * math.pi * sigma * sigma)


class CamTracks:
    """Camera tracks for identity by motion. Blink sightings come with
    blink.py's track numbers; detector sightings are chained here frame to
    frame (nearest first, within 4 cm + top speed x time)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.tracks = {}
        self.next_det = 1

    def get(self, key):
        tr = self.tracks.get(key)
        if tr is None:
            tr = self.tracks[key] = dict(key=key, hist=deque(), ll={}, ll_none=0.0, n_win=0, last_win=-1e9,
                                         t=-1e9, probs=None, p_none=0.0, link=None, link_t=None)
        return tr

    def det_keys(self, t, pts):
        cands = [tr for k, tr in self.tracks.items() if k[0] == "d" and t - tr["t"] < 1.0]
        pairs = []
        for i, (x, y) in enumerate(pts):
            for tr in cands:
                lx, ly = tr["hist"][-1][1:]
                dd = math.hypot(x - lx, y - ly)
                if dd <= 4.0 + self.cfg["vmax"] * (t - tr["t"]):
                    pairs.append((dd, i, tr["key"], dd < 6.0 and t - tr["t"] < 0.2))
        pairs.sort()
        out, used = [None] * len(pts), set()
        n_pt, n_tr = {}, {}
        for dd, i, k, close in pairs:
            if close:
                n_pt[i] = n_pt.get(i, 0) + 1
                n_tr[k] = n_tr.get(k, 0) + 1
        for dd, i, k, _ in pairs:
            if out[i] is None and k not in used:
                out[i] = k
                used.add(k)
                if n_pt.get(i, 0) > 1 or n_tr.get(k, 0) > 1:
                    self.forget(self.tracks[k])     # two objects within 6 cm: it may swap, so forget its motion
        for i in range(len(pts)):
            if out[i] is None:
                out[i] = ("d", self.next_det)
                self.next_det += 1
        return out

    @staticmethod
    def forget(tr):
        tr["ll"], tr["ll_none"], tr["n_win"] = {}, 0.0, 0
        if tr["key"][0] == "d":              # a detector track also drops the name it carried
            tr["link"], tr["link_t"] = None, None

    def push(self, tr, t, x, y):
        tr["hist"].append((t, x, y))
        tr["t"] = t
        while tr["hist"] and t - tr["hist"][0][0] > 3 * self.cfg["motion_window_s"] + 0.5:
            tr["hist"].popleft()

    def prune(self, t):
        for k in [k for k, tr in self.tracks.items() if t - tr["t"] > 5.0]:
            del self.tracks[k]


def pos_at(hist, t, tol):
    """The sample in hist [(t, x, y)] nearest time t, if within tol."""
    best = None
    for h in hist:
        if abs(h[0] - t) <= tol and (best is None or abs(h[0] - t) < abs(best[0] - t)):
            best = h
    return best


# ---------------------------------------------------------------- unknown objects
class Unknowns:
    """Rules 2 and 3: sightings no robot claims, tracked separately."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.tracks = []   # dict(id, x, y, t, hits deque, confirmed, vx, vy)
        self.next_id = 1
        self.confirmed_ever = []

    def add(self, t, x, y, name=None, kind="det"):
        """kind: lights (blink) and robot centres (detector) are tracked apart,
        since a robot's light sits 2.75 cm from its centre."""
        best, bd = None, None
        for tr in self.tracks:
            if tr["kind"] != kind:
                continue
            dt = t - tr["t"]
            d = math.hypot(x - tr["x"], y - tr["y"])
            if d <= 4.0 + self.cfg["vmax"] * dt and (bd is None or d < bd):
                best, bd = tr, d
        if best is None:
            best = dict(id=self.next_id, x=x, y=y, t=t, hits=deque(), confirmed=False, vx=0.0, vy=0.0, first=t,
                        name=None, kind=kind)
            self.next_id += 1
            self.tracks.append(best)
        else:
            dt = max(t - best["t"], 1e-3)
            best["vx"] = 0.7 * best["vx"] + 0.3 * (x - best["x"]) / dt
            best["vy"] = 0.7 * best["vy"] + 0.3 * (y - best["y"]) / dt
            best["x"], best["y"], best["t"] = x, y, t
        if name is not None:
            best["name"] = name
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
def wheel_fwd_signs(runs):
    """robot -> +1 or -1 so that wl_rpm / wr_rpm * sign is forward-positive
    (checked against v_cms)."""
    acc = {}
    for run in runs:
        a = acc.setdefault(run.robot, 0.0)
        for r in run.raw:
            a += fnum(r.get("v_cms")) * (fnum(r.get("wl_rpm")) + fnum(r.get("wr_rpm")))
        acc[run.robot] = a
    return {robot: (-1.0 if a < 0 else 1.0) for robot, a in acc.items()}


def run_engine(runs, detections, cfg, gyro_f=None, blink=None, calib_rows=None, led_offsets=None, signs=None):
    """Process all runs (and camera sightings) together, in the order the
    information would have reached the robots. Returns robot run_id -> Tracked,
    and the unknown-object tracker.

    blink: runs.load_blink rows (used if "blink" is in the sensors);
    calib_rows: runs.load_calib_results rows (start-nudge headings);
    led_offsets: {robot or None: (forward, left) cm} (runs.led_offsets);
    signs: (wheel_turn_signs, wheel_fwd_signs) worked out from a larger set of
    runs, so a subset of runs is processed exactly as within the whole set
    (replay.py with one calibration per run); default: from `runs`."""
    rng = random.Random(cfg["seed"])
    if cfg["vmax"] == "auto":
        cfg = dict(cfg, vmax=fastest_speed(runs))
    wsign = signs[0] if signs else wheel_turn_signs(runs)
    gyro_f = gyro_f or gyro_factors(runs, wsign)
    fsign = signs[1] if signs else wheel_fwd_signs(runs)
    rpm2cms = 2 * math.pi * cfg["wheel_radius_cm"] / 60
    use_cam = "camera" in cfg["sensors"] and detections
    use_blink = bool("blink" in cfg["sensors"] and blink)
    ident = use_blink or cfg["motion_identity"]
    if cfg["led_offset_cm"] == "auto":
        led = dict(led_offsets) if led_offsets else {None: (2.75, 0.0)}
    else:
        led = {None: tuple(cfg["led_offset_cm"])}
    led.setdefault(None, (2.75, 0.0))
    events = []   # (arrival, order, payload)
    for ri, run in enumerate(runs):
        gf = gyro_f.get(run.robot, (D2R, ""))[0]
        ws = wsign.get(run.robot, 1.0)
        fs = fsign.get(run.robot, 1.0)
        for r in run.raw:
            v = fnum(r.get("v_cms"))
            w = fnum(r.get("w_dps")) * D2R * ws
            # still = wheels not turning at all (a turn in place moves the wheels too)
            still = (abs(fnum(r.get("wl_rpm"))) < 0.2 and abs(fnum(r.get("wr_rpm"))) < 0.2
                     and abs(v) < 0.1 and abs(fnum(r.get("w_dps"))) < 0.5)
            d = dict(v=v, w=w, g=fnum(r.get("gz")) * gf, still=still, lifted=fnum(r.get("lifted")) > 0.5,
                     vl=fnum(r.get("wl_rpm")) * fs * rpm2cms, vr=fnum(r.get("wr_rpm")) * fs * rpm2cms)
            events.append((r["t"], 0, ("raw", ri, d)))
    if calib_rows and cfg["calib_heading"]:
        idx = {run.run_id: ri for ri, run in enumerate(runs)}
        for c in calib_rows:
            if c["run"] in idx and c["t"] is not None and c["heading"] is not None:
                events.append((c["t"], 2, ("hdg", idx[c["run"]], dict(
                    th=wrap(c["heading"] * D2R), sigma=cfg["calib_heading_sigma_deg"] * D2R, src="calib"))))
    sights = list(detections) if use_cam else []
    if use_blink:
        sights += [dict(b, kind="blink") for b in blink]
    if sights:
        frames = {}
        for det in sights:
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
    ctracks = CamTracks(cfg)
    use_gyro = "gyro" in cfg["sensors"]
    order = 0
    for arrival, _, payload in events:
        order += 1
        if payload[0] == "raw":
            _, ri, d = payload
            run = runs[ri]
            tr = tracked.get(run.run_id)
            if tr is None:
                tr = tracked[run.run_id] = Tracked(run, cfg, gyro_f.get(run.robot, (D2R, ""))[0])
                tr.led = tuple(led.get(run.robot, led[None]))
            if d["lifted"] and not tr.was_lifted:          # rule 5: picked up
                tr.was_lifted = True
                tr.lost_state = True
                tr.kidnaps.append(arrival)
                tr.add((arrival, order, "lost", None))
            elif not d["lifted"]:
                tr.was_lifted = False
            tr.add((arrival, order, "raw", d))
            tr.odo_step(arrival, d, use_gyro)
            s = tr.ekf.s
            tr.live.append((arrival, s[0], s[1], s[2]))
            continue
        if payload[0] == "hdg":
            _, ri, d = payload
            tr = tracked.get(runs[ri].run_id)
            if tr is not None:
                tr.add((arrival, order, "hdg", d))
                tr.hdg_used["calib"] += 1
            continue
        _, t_cap, dets = payload
        active = [tr for tr in tracked.values() if tr.t0 - 0.2 <= t_cap <= tr.t1 + 0.3]
        if ident:
            _identity_frame(t_cap, order, dets, active, cfg, unknown, ctracks, use_blink)
            continue
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
            if _blocked(tr, t_cap, cfg):
                tr.cam_blocked += 1
                continue
            _use_fix(tr, t_cap, order, dets[di], "det", "position", cfg)
        for di, det in enumerate(dets):                     # rules 2 and 3
            if di in used_d:
                continue
            ut = unknown.add(t_cap, det["x"], det["y"])
            _refind(ut, active, t_cap, order, det, unknown)
        unknown.prune(t_cap)
    return tracked, unknown


def _blocked(tr, t_cap, cfg):
    mid = (tr.t1 - tr.t0) / 2
    return any(a <= t_cap - tr.t0 <= b for a, b in cfg["cam_off"]) or (
        cfg["cam_off_mid_s"] and abs(t_cap - tr.t0 - mid) <= cfg["cam_off_mid_s"] / 2)


def _refind(ut, active, t_cap, order, det, unknown):
    """Rule 5: re-find a set-down robot (never one the sighting's name rules out)."""
    for tr in active:
        if tr.lost_state and not tr.was_lifted and ut["confirmed"] and ut.get("name") in (None, tr.run.robot):
            robot_speed = abs(tr.ekf.s[3])
            track_speed = math.hypot(ut["vx"], ut["vy"])
            if abs(track_speed - robot_speed) < max(3.0, 0.5 * robot_speed):
                x, y, sig = det["x"], det["y"], 1.0
                if det.get("kind") == "blink":       # light -> centre needs the heading, which is lost: the
                    sig += math.hypot(*tr.led)       # centre is somewhere within the offset of the light
                tr.add((t_cap, order, "reset", dict(x=x, y=y, sigma=sig)))
                tr.lost_state = False
                unknown.tracks.remove(ut)
                break


def _use_fix(tr, t_cap, order, det, kind, source, cfg):
    """Apply one camera fix to a robot, record where its identity came from,
    then try heading from motion."""
    if kind == "blink":
        tr.add((t_cap, order, "blink", dict(x=det["x"], y=det["y"], off=tr.led)))
    else:
        tr.add((t_cap, order, "cam", det))
    tr.cam_used += 1
    tr.fixes.append(dict(t=t_cap, frame=det.get("frame", -1), kind=kind, x=det["x"], y=det["y"], source=source))
    tr.id_sources[source] = tr.id_sources.get(source, 0) + 1
    hist = tr.fix_hist[kind]
    hist.append((t_cap, det["x"], det["y"]))
    while hist and t_cap - hist[0][0] > 2 * cfg["hfm_window_s"] + 0.5:
        hist.popleft()
    if cfg["heading_from_motion"]:
        _heading_from_motion(tr, t_cap, order, kind, cfg)


def _heading_from_motion(tr, t, order, kind, cfg):
    W = cfg["hfm_window_s"]
    if t - tr.last_hfm < W or tr.lost_state:
        return
    a = pos_at(tr.fix_hist[kind], t - W, 0.2 * W)
    if a is None or a[0] >= t:
        return
    ta = a[0]
    if not hfm_gate(tr.odo_rows(ta, t), cfg):
        return
    oa, ob, om = tr.odo_at(ta), tr.odo_at(t), tr.odo_at((ta + t) / 2)
    if oa is None or ob is None or om is None:
        return
    pa, pb = (a[1], a[2]), tr.fix_hist[kind][-1][1:]
    sig = tr.noise.cam_xy
    if kind == "blink":                       # light -> centre: take off the light's swing as the robot turns
        sig = tr.noise.blink_xy
        tha = tr.live_heading(ta)
        thb = tha + (ob[2] - oa[2])
        pa = (pa[0] - rot(tr.led, tha)[0], pa[1] - rot(tr.led, tha)[1])
        pb = (pb[0] - rot(tr.led, thb)[0], pb[1] - rot(tr.led, thb)[1])
    m = hfm_heading(pa, pb, ta, t, ob[2] - om[2], sig, cfg)
    if m is None:
        return
    th, sigma = m
    y, p22 = tr.ekf.heading_d2(th)
    tr.last_hfm = t
    if y * y / (p22 + sigma * sigma) > cfg["hfm_gate_chi2"]:
        tr.hdg_rejected += 1
        return
    tr.add((t, order, "hdg", dict(th=th, sigma=sigma, src="motion")))
    tr.hdg_used["motion"] += 1


def _motion_update(ct, kind, active, t, cfg):
    """One identity-by-motion window for camera track ct ending at time t."""
    W = cfg["motion_window_s"]
    if t - ct["last_win"] < W:
        return
    a = pos_at(ct["hist"], t - W, 0.3 * W)
    if a is None:
        return
    b = ct["hist"][-1]
    dt = b[0] - a[0]
    if dt < 0.5 * W:
        return
    d = (b[1] - a[1], b[2] - a[2])
    sig_m = active[0].noise.blink_xy if (kind == "blink" and active) else (active[0].noise.cam_xy if active else 0.8)
    preds = {}
    for tr in active:
        if tr.lost_state:
            continue
        D = tr.own_displacement(a[0], b[0], tr.led if kind == "blink" else (0.0, 0.0))
        if D is not None:
            preds[tr.run.robot] = (tr, D)
    moved = max([math.hypot(*d)] + [math.hypot(*D) for _, D in preds.values()])
    ct["last_win"] = t
    if moved < cfg["motion_min_cm"]:
        return
    k = cfg["motion_decay"]
    sn = cfg["motion_none_cm"]
    ln = -(d[0] ** 2 + d[1] ** 2) / (2 * sn * sn) - math.log(2 * math.pi * sn * sn)
    for tr in active:
        r = tr.run.robot
        if r not in ct["ll"]:
            ct["ll"][r] = ct["ll_none"]
        if r in preds:
            D = preds[r][1]
            sth = max(math.sqrt(max(tr.ekf.P[2, 2], 0.0)), 5 * D2R)
            mag = math.hypot(*D)
            sigma = math.sqrt(2 * sig_m ** 2 + (0.05 * mag) ** 2 + (sth * mag) ** 2 + 0.3 ** 2)
            lr = motion_loglik(d, D, sigma)
        else:
            lr = ln
        ct["ll"][r] = k * ct["ll"][r] + lr
    ct["ll_none"] = k * ct["ll_none"] + ln
    for r in ct["ll"]:                          # keep within +-30 of "none" (no runaway certainty)
        ct["ll"][r] = min(max(ct["ll"][r], ct["ll_none"] - 30), ct["ll_none"] + 30)
    ct["n_win"] += 1


def _motion_dist(ct, hyps):
    if not ct or ct["n_win"] == 0:
        return None
    ll = {h: ct["ll"].get(h, ct["ll_none"]) if h not in ("none", "elsewhere") else ct["ll_none"] for h in hyps}
    m = max(ll.values())
    return normalise({h: math.exp(v - m) for h, v in ll.items()})


def _decide(pb, pm, gates, hyps, cfg):
    """Who may take this sighting. pb / pm: blink / motion distributions over
    hyps (or None); gates: robot -> (d2, log position likelihood, Tracked) for
    the robots whose possible area it is in.
    Returns (named, candidates {robot: gate}, source, name):
      - blink alone names h at >= blink_name_p: only h may take it (if it's
        in h's area); a name that isn't a running robot sends it to the
        unknown objects with that name.
      - else blink x motion names a robot at >= blink_name_p AND, with the
        position likelihoods put in too (outside a robot's area = impossible,
        "none" = 1 / clutter_cm2), that robot still has >= blink_name_p:
        only that robot ("motion" / "combined").
      - otherwise the normal rules, among the robots in whose area it is and
        that (blink x motion x position) doesn't put under id_exclude_p."""
    tau = cfg["blink_name_p"]
    if pb:
        b = max(pb, key=pb.get)
        if pb[b] >= tau:
            src = "both" if (pm and pm[b] >= tau) else "blink"
            if b in gates:
                return True, {b: gates[b]}, src, b
            return True, {}, src, b
    post, best, source = combine_identity(pb, pm, tau)
    if not post:
        return False, gates, None, None
    lz = -math.log(cfg["clutter_cm2"])
    ll = {h: (gates[h][1] if h in gates else (lz if h in ("none", "elsewhere") else None)) for h in hyps}
    m = max(v for v in ll.values() if v is not None)
    pp = normalise({h: post[h] * math.exp(v - m) for h, v in ll.items() if v is not None})
    if source is not None and pp.get(best, 0.0) >= tau:
        if best in gates:
            return True, {best: gates[best]}, source, best
        if best in ("none", "elsewhere"):
            return True, {}, source, best
    return False, {r: g for r, g in gates.items() if pp.get(r, 0.0) >= cfg["id_exclude_p"]}, None, None


def _identity_frame(t_cap, order, dets, active, cfg, unknown, ctracks, use_blink):
    """One camera frame with the identity rules (module docstring)."""
    tau = cfg["blink_name_p"]
    running = [tr.run.robot for tr in active]
    blinks = [i for i, d in enumerate(dets) if d.get("kind") == "blink"]
    plain = [i for i, d in enumerate(dets) if d.get("kind") != "blink"]
    keys = [None] * len(dets)
    for i in blinks:
        keys[i] = ("b", dets[i]["track"])
    for i, k in zip(plain, ctracks.det_keys(t_cap, [(dets[i]["x"], dets[i]["y"]) for i in plain])):
        keys[i] = k
    info = []
    for i, det in enumerate(dets):
        kind = "blink" if det.get("kind") == "blink" else "det"
        ct = ctracks.get(keys[i])
        ctracks.push(ct, t_cap, det["x"], det["y"])
        if kind == "blink":
            ct["probs"], ct["p_none"] = det["probs"], det["p_none"]
            if any(j != i and math.hypot(det["x"] - dets[j]["x"], det["y"] - dets[j]["y"]) < 6.0 for j in blinks):
                CamTracks.forget(ct)                       # two lights this close: blink.py's tracks may swap
        elif blinks and cfg["det_blink_link_cm"] > 0:      # a detector sighting at a blink light shares its name
            ds = sorted((math.hypot(det["x"] - dets[j]["x"], det["y"] - dets[j]["y"]), j) for j in blinks)
            if ds[0][0] <= cfg["det_blink_link_cm"] and (len(ds) == 1 or ds[1][0] > 1.5 * ds[0][0]):
                ct["link"], ct["link_t"] = keys[ds[0][1]], t_cap
        if cfg["motion_identity"]:
            _motion_update(ct, kind, active, t_cap, cfg)
        src_ct = ct if kind == "blink" else ctracks.tracks.get(ct["link"]) if ct["link"] else None
        has_names = bool(use_blink and cfg["blink_identity"] and src_ct and src_ct["probs"])
        hyps = running + ["none"] + (["elsewhere"] if has_names else [])
        pb = blink_dist(hyps, src_ct["probs"], src_ct["p_none"]) if has_names else None
        if pb and kind == "det":                           # carried name: confidence fades since the last light
            w = math.exp(-max(0.0, t_cap - ct["link_t"]) / cfg["name_carry_s"])
            pb = {h: w * p + (1 - w) / len(pb) for h, p in pb.items()}
        pm = _motion_dist(ct, hyps) if cfg["motion_identity"] else None
        gates = {}
        for tr in active:
            if not tr.lost_state:
                g = tr.gate(t_cap, (det["x"], det["y"]), tr.led if kind == "blink" else None)
                if g is not None:
                    gates[tr.run.robot] = (g[0], g[1], tr)
        info.append(_decide(pb, pm, gates, hyps, cfg) + (kind,))

    named, anon = [], []
    for i, det in enumerate(dets):
        to, cands, source, name, kind = info[i]
        for r, (d2, _, tr) in cands.items():
            (named if to else anon).append((d2, i, tr))
    used_d, used_t = set(), set()
    for group in (named, anon):                               # named first, then closest pairs, one each per kind
        group.sort(key=lambda p: p[0])
        for d2, i, tr in group:
            kt = (id(tr), info[i][4])
            if i in used_d or kt in used_t:
                continue
            used_d.add(i)
            used_t.add(kt)
            if _blocked(tr, t_cap, cfg):
                tr.cam_blocked += 1
                continue
            _use_fix(tr, t_cap, order, dets[i], info[i][4], info[i][2] or "position", cfg)
    for i, det in enumerate(dets):                            # rules 2 and 3, names kept
        if i in used_d:
            continue
        ut = unknown.add(t_cap, det["x"], det["y"], info[i][3], info[i][4])
        _refind(ut, active, t_cap, order, det, unknown)
    unknown.prune(t_cap)
    ctracks.prune(t_cap)
