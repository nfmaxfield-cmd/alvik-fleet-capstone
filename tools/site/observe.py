"""From survey logs to what the site solver uses: still poses ("stations") with
the camera's view of the robot and every front-sensor zone's average reading
there, the wheel/gyro motion between stations, and the edge guard's trips.

Input, the same for the real robots and the simulator:
  runs       tools/fusion/runs.py Run objects (RAW rows with "t" on the laptop
             clock), each with .edge = its EDGE records (dicts with "t")
  sightings  per robot, the camera's positions [(t, x, y, heading_deg or None)]
             on the laptop clock (floor cm)

A station is a stretch where the robot stands still (wheel speed under 0.3
cm/s and turn rate under 2 deg/s) for at least 0.8 s; its first 0.25 s are
dropped (the robot settles and the sensor's frame catches up). Readings: each
zone's distinct frames (a RAW row repeats the last frame until a new one), 2 to
120 cm; their median and how many.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "fusion")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import landmarks as LM  # noqa: E402
import raw_units as RU  # noqa: E402  (RAW v_cms is mm/s; v_cms, w_dps read 1.08 high)

STILL_V, STILL_W, STILL_MIN_S, SETTLE_S = 0.3, 2.0, 0.8, 0.25
STILL_MEDIAN_N = 5     # still/moving flag is median-filtered over this many RAW rows (~0.1 s): a lone wheel-speed
                       # noise sample past STILL_W no longer splits a stand (it used to cut the stand before a
                       # touch creep short, so the touch found no station); real motion edges are kept in place


def fnum(v, default=float("nan")):
    try:
        x = float(v)
        return x
    except (TypeError, ValueError):
        return default


def speed_cms(r):
    """True forward speed (cm/s) of a RAW row (tools/raw_units.py: the column is mm/s, 1.08 high); 0 if missing."""
    return RU.speed_cms(r.get("v_cms")) if fnum(r.get("v_cms"), None) is not None else 0.0


def turn_dps(r):
    """True wheel turn rate (deg/s) of a RAW row (the column is 1.08 high); 0 if missing."""
    return RU.turn_dps(r.get("w_dps")) if fnum(r.get("w_dps"), None) is not None else 0.0


class Station:
    def __init__(self, robot, run, t0, t1):
        self.robot, self.run, self.t0, self.t1 = robot, run, t0, t1
        self.cam = None            # (x, y) median camera position
        self.cam_th = None         # mean camera heading (deg) or None
        self.n_cam = 0
        self.zones = [None] * 5    # per zone (median cm, n distinct frames, robust sd cm) or None
        self.odo = None            # library odometry (x, y, th_deg) at the station
        self.yaw = None            # gyro yaw (deg, unwrapped) at the station
        self.edge = None           # dict(side?, back_cm) if the edge guard tripped just before this station
        self.idx = -1

    def __repr__(self):
        return f"Station({self.robot} {self.run} {self.t0:.1f}-{self.t1:.1f} cam={self.cam})"


def still_spans(raw):
    """[(i0, i1)] index ranges of RAW rows where the robot stands still long enough."""
    flag = [abs(speed_cms(r)) < STILL_V and abs(turn_dps(r)) < STILL_W
            and fnum(r.get("lifted"), 0.0) < 0.5 for r in raw]
    h = STILL_MEDIAN_N // 2
    out, start = [], None
    for i, r in enumerate(raw):
        win = flag[max(0, i - h):i + h + 1]
        still = 2 * sum(win) > len(win) if len(win) > 1 else flag[i]   # median of the window (a tie: moving)
        if still and start is None:
            start = i
        if (not still or i == len(raw) - 1) and start is not None:
            end = i if still else i - 1
            if raw[end]["t"] - raw[start]["t"] >= STILL_MIN_S:
                out.append((start, end))
            start = None
    return out


def zone_stats(rows):
    out = []
    for c in LM.ZONE_COLS:
        vals, prev = [], None
        for r in rows:
            v = fnum(r.get(c))
            if v != v:
                continue
            if v != prev:
                vals.append(v)
            prev = v
        vals = [v for v in vals if 2.0 <= v <= 120.0]
        if len(vals) < 3:
            out.append(None)
            continue
        a = np.array(vals)
        med = float(np.median(a))
        mad = 1.4826 * float(np.median(np.abs(a - med)))
        keep = np.abs(a - med) <= max(3 * mad, 1.0)           # a hand passing, another robot: dropped
        a = a[keep]
        out.append((float(np.median(a)) if len(a) % 2 else float(np.mean(np.sort(a)[len(a) // 2 - 1:len(a) // 2 + 1])),
                    int(len(a)), max(float(np.std(a)), 0.05)))
    return out


def camera_at(sight, t0, t1, cam_heading=True):
    s = [v for v in sight if t0 <= v[0] <= t1]
    if len(s) < 3:
        return None, None, 0
    a = np.array([[v[1], v[2]] for v in s])
    med = np.median(a, 0)
    th = None
    hs = [v[3] for v in s if v[3] is not None and cam_heading]
    if len(hs) >= 3:
        hr = np.radians(hs)
        th = math.degrees(math.atan2(np.mean(np.sin(hr)), np.mean(np.cos(hr))))
    return (float(med[0]), float(med[1])), th, len(s)


def yaw_to_floor(raw, gyro_th, sight, win_s=0.6):
    """(sign, offset_deg) so that floor heading = sign x gyro yaw + offset, from the stretches where the robot
    drives straight forward (the camera's direction of travel is its heading then). None if no such stretch."""
    if not sight:
        return None
    S = np.array([(v[0], v[1], v[2]) for v in sight])
    T = np.array([r["t"] for r in raw])
    pairs = []
    i = 0
    while i < len(raw):
        r = raw[i]
        if speed_cms(r) > 3.0 and abs(turn_dps(r)) < 3.0:
            t0 = r["t"]
            j = int(np.searchsorted(T, t0 + win_s))
            if j < len(raw) and all(speed_cms(q) > 2.0 for q in raw[i:j + 1]):
                a = S[np.abs(S[:, 0] - t0) < 0.15]
                b = S[np.abs(S[:, 0] - (t0 + win_s)) < 0.15]
                if len(a) and len(b):
                    d = b[:, 1:].mean(0) - a[:, 1:].mean(0)
                    if np.hypot(*d) > 1.5:
                        pairs.append((math.atan2(d[1], d[0]), math.radians(0.5 * (gyro_th[i] + gyro_th[j]))))
                i = j
        i += 1
    if len(pairs) < 2:
        return None
    best = None
    for sgn in (1.0, -1.0):
        off = np.array([(t - sgn * y + math.pi) % (2 * math.pi) - math.pi for t, y in pairs])
        c, s_ = np.mean(np.cos(off)), np.mean(np.sin(off))
        rr = math.hypot(c, s_)
        if best is None or rr > best[0]:
            best = (rr, sgn, math.degrees(math.atan2(s_, c)))
    return best[1], best[2]


def stations(runs, sightings, min_cam=5):
    """All stations of all runs, in time order per run, numbered. Each station also gets heading_guess (deg):
    the camera's heading if it gives one, else the gyro's yaw turned to the floor (yaw_to_floor)."""
    out = []
    for run in runs:
        raw = sorted(run.raw, key=lambda r: r["t"])
        if len(raw) < 10:
            continue
        gyro_th = _gyro_yaw(raw)
        y2f = yaw_to_floor(raw, gyro_th, [v for v in sightings.get(run.robot, [])
                                          if raw[0]["t"] - 1 <= v[0] <= raw[-1]["t"] + 1])
        trips = [e for e in getattr(run, "edge", []) if (e.get("event") or "").strip() == "trip"]
        for i0, i1 in still_spans(raw):
            t0 = raw[i0]["t"] + SETTLE_S
            t1 = raw[i1]["t"]
            rows = [r for r in raw[i0:i1 + 1] if r["t"] >= t0]
            if len(rows) < 5:
                continue
            st = Station(run.robot, run.run_id, t0, t1)
            st.cam, st.cam_th, st.n_cam = camera_at(sightings.get(run.robot, []), t0, t1)
            if st.cam is None or st.n_cam < min_cam:
                continue
            st.zones = zone_stats(rows)
            mid = rows[len(rows) // 2]
            st.odo = (fnum(mid.get("x_cm"), 0.0), fnum(mid.get("y_cm"), 0.0), fnum(mid.get("th_deg"), 0.0))
            st.yaw = float(np.interp(mid["t"], [r["t"] for r in raw], gyro_th))
            if st.cam_th is not None:
                st.heading_guess = st.cam_th
            elif y2f is not None:
                st.heading_guess = (y2f[0] * st.yaw + y2f[1] + 180.0) % 360.0 - 180.0
            else:
                st.heading_guess = 0.0
            # an edge trip shortly before this still stretch: how far the robot rolled after it (odometry)
            for e in trips:
                if raw[max(i0 - 1, 0)]["t"] - 3.0 <= e["t"] <= t0:
                    j = int(np.searchsorted([r["t"] for r in raw], e["t"]))
                    j = min(max(j, 0), len(raw) - 1)
                    xa, ya = fnum(raw[j].get("x_cm"), 0.0), fnum(raw[j].get("y_cm"), 0.0)
                    back = math.hypot(st.odo[0] - xa, st.odo[1] - ya)
                    st.edge = dict(back_cm=back, t=e["t"], cue=(e.get("info") or "").strip())
            out.append(st)
    for k, st in enumerate(out):
        st.idx = k
    return out


def _gyro_yaw(raw):
    """Heading change from the gyro (deg, unwrapped, starts at 0): the firmware's fused yaw."""
    y = np.array([fnum(r.get("yaw_deg"), 0.0) for r in raw])
    return np.degrees(np.unwrap(np.radians(y)))


def odometry_links(stations):
    """Wheel/gyro motion between consecutive stations of the same run: list of (i, j, d_along, d_left, d_th_deg,
    path_cm, turned_deg): the move from station i to j in i's own frame (library odometry for distance,
    gyro for turning). The gyro's sign against the floor is checked by the solver."""
    out = []
    by = {}
    for st in stations:
        by.setdefault((st.robot, st.run), []).append(st)
    for lst in by.values():
        for a, b in zip(lst, lst[1:]):
            dx, dy = b.odo[0] - a.odo[0], b.odo[1] - a.odo[1]
            th = math.radians(a.odo[2])
            along = math.cos(th) * dx + math.sin(th) * dy
            left = -math.sin(th) * dx + math.cos(th) * dy
            dth_odo = b.odo[2] - a.odo[2]
            dth_gyro = b.yaw - a.yaw
            out.append((a.idx, b.idx, along, left, dth_gyro, math.hypot(dx, dy), abs(dth_odo)))
    return out
