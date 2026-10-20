"""Carry labels between two cameras that watch the same floor (see CROSSCAM.md).

A label confirmed in camera A becomes a ready-made guess in camera B's frame
taken at (nearly) the same moment; confirmed labels of both cameras are
compared on the floor and the disagreements listed.

Times: give each camera's frame times on ONE clock. camrec.py writes t_wall
(the Pi's clock, shared by every process on that Pi) and t_mono (seconds since
THAT process started: two camrec processes have different zeros), so
load_frame_times() reads t_wall by default. If B's times are on another clock,
pass offset_s (A-clock time = B time + offset_s).

Object dicts are the labelling tool's (tools/label/app.py Session.frames[f]
["objects"]): obj, cls, x, y (px), heading_deg, heading_known, status, score.
A by-frame mapping may hold either such lists or Session.frames-style dicts
with an "objects" list.
"""
import csv
import math

import numpy as np

try:
    from .cameras import CameraModel, transfer  # noqa: F401
except ImportError:          # run with tools/recognition on sys.path
    from cameras import CameraModel, transfer  # noqa: F401

CONFIRMED = ("confirmed", "corrected")
LABELLED = ("confirmed", "corrected", "auto")     # what disagreements() compares
SPEED_CM_S = 20.0          # assumed top speed of a labelled object (robot drives ~ up to this); scoring only
CALIB_CM_DEFAULT = 0.5     # floor error assumed for a camera model without a left-one-out figure
HEIGHT_SD_KNOWN_CM = 0.5   # height uncertainty of a class listed in class_heights
HEIGHT_SD_DEFAULT_CM = 2.0  # ... and of one that falls back to default_height_cm
SCORE_REF_CM = 3.0         # uncertainty at which the score drops to exp(-1/2) = 0.61


# ---------------------------------------------------------------- times
def load_frame_times(path, column="t_wall"):
    """frames.csv (tools/pi/camrec.py) -> {frame: time s}. Default t_wall: shared by both
    camrec processes on one Pi (t_mono restarts at 0 in each process)."""
    out = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r.get(column, "") != "":
                out[int(r["frame"])] = float(r[column])
    return out


def _times(t):
    """{frame: t} | sequence (index = frame) | (n, 2) array of frame, t -> (frames, times) sorted by time."""
    if t is None:
        return np.zeros(0, int), np.zeros(0)
    if isinstance(t, dict):
        fr = np.fromiter(t.keys(), int, len(t))
        tt = np.fromiter((float(v) for v in t.values()), float, len(t))
    else:
        a = np.asarray(t, float)
        if a.ndim == 2 and a.shape[1] == 2:
            fr, tt = a[:, 0].astype(int), a[:, 1]
        else:
            fr, tt = np.arange(len(a)), a.ravel()
    ok = np.isfinite(tt)
    fr, tt = fr[ok], tt[ok]
    o = np.argsort(tt, kind="stable")
    return fr[o], tt[o]


def _interval(tt):
    return float(np.median(np.diff(tt))) if len(tt) > 1 else 0.0


def match_frames(times_a, times_b, max_dt=None, offset_s=0.0, return_dt=False):
    """{frame_b: frame_a} with frame_a the A frame nearest in time to each B frame, kept only
    within max_dt seconds (default: half the larger median frame interval). B times are moved
    onto A's clock with offset_s first. return_dt=True gives {frame_b: (frame_a, t_a - t_b)}."""
    fa, ta = _times(times_a)
    fb, tb = _times(times_b)
    if not len(fa) or not len(fb):
        return {}
    tb = tb + offset_s
    if max_dt is None:
        max_dt = 0.5 * max(_interval(ta), _interval(tb))
    i = np.clip(np.searchsorted(ta, tb), 1, max(len(ta) - 1, 1))
    lo = np.clip(i - 1, 0, len(ta) - 1)
    hi = np.clip(i, 0, len(ta) - 1)
    pick = np.where(np.abs(ta[lo] - tb) <= np.abs(ta[hi] - tb), lo, hi)
    d = ta[pick] - tb
    out = {}
    for b, j, dd in zip(fb, pick, d):
        if abs(dd) <= max_dt + 1e-9:
            out[int(b)] = (int(fa[j]), float(dd)) if return_dt else int(fa[j])
    return out


# ---------------------------------------------------------------- guesses for camera B
def _objects(v):
    if isinstance(v, dict):
        return v.get("objects") or []
    return v or []


def _height(cls, class_heights, default):
    if class_heights and cls in class_heights:
        return float(class_heights[cls]), HEIGHT_SD_KNOWN_CM
    return float(default), HEIGHT_SD_DEFAULT_CM


def _calib_cm(cam):
    v = (cam.fit or {}).get("loo_rms_cm")
    return float(v) if v is not None else CALIB_CM_DEFAULT


def cross_guesses(objects_a_by_frame, cam_a, cam_b, times_a, times_b, size_b=None, class_heights=None,
                  default_height_cm=4.0, score_scale=1.0, min_score=None, max_dt=None, offset_s=0.0,
                  speed_cm_s=SPEED_CM_S, margin_px=0.0, lead_cm=3.0):
    """Camera A's confirmed labels -> guesses in camera B's frames.

    Returns {frame_b: [dict(cls, x, y, heading_deg, score, source='camera', from_frame, from_obj,
    heading_known, dt_s, sigma_cm)]}. Uses objects with status confirmed/corrected; with
    min_score set, also unconfirmed (pending/auto) ones whose own score >= min_score (their
    score then multiplies in). Rejected objects never.

    Score = score_scale * exp(-0.5 * (sigma / SCORE_REF_CM)^2), sigma (cm on the floor) = the
    root-sum-square of
      - timing: speed_cm_s * |time gap between the two frames| (how far it could have moved),
      - calibration: both cameras' left-one-out floor errors (CALIB_CM_DEFAULT if unknown),
      - height: how far B's pixel moves if the object's height is off by its sd (0.5 cm for a
        class in class_heights, 2 cm for the default height), in B's cm at that spot.
    So a guess from a well-calibrated pair at the same instant scores ~0.95; one from a frame
    100 ms away while a robot may be driving at 20 cm/s scores ~0.8."""
    size_b = tuple(size_b) if size_b is not None else cam_b.image_size
    if size_b != tuple(cam_b.image_size):
        cam_b = CameraModel(cam_b.K, cam_b.dist, cam_b.rvec, cam_b.tvec, size_b, fit=cam_b.fit, name=cam_b.name)
    pairs = match_frames(times_a, times_b, max_dt, offset_s, return_dt=True)
    calib = math.hypot(_calib_cm(cam_a), _calib_cm(cam_b))
    out = {}
    for fb, (fa, dts) in sorted(pairs.items()):
        if fa not in objects_a_by_frame:
            continue
        guesses = []
        for o in _objects(objects_a_by_frame[fa]):
            st, cls = o.get("status"), o.get("cls")
            if cls in (None, "none") or st == "rejected":
                continue
            src_score = 1.0
            if st not in CONFIRMED:
                if min_score is None or o.get("score") is None or float(o["score"]) < min_score:
                    continue
                src_score = float(o["score"])
            h, h_sd = _height(cls, class_heights, default_height_cm)
            known = bool(o.get("heading_known")) and o.get("heading_deg") is not None
            ub, vb, hb, vis = transfer(cam_a, cam_b, float(o["x"]), float(o["y"]), h,
                                       float(o["heading_deg"]) if known else None, lead_cm, margin_px)
            if not vis:
                continue
            x, y = cam_a.to_floor(float(o["x"]), float(o["y"]), h)
            scale = cam_b.px_per_cm(x, y, h)
            u1, v1, _, _ = transfer(cam_a, cam_b, float(o["x"]), float(o["y"]), h + h_sd)
            u2, v2, _, _ = transfer(cam_a, cam_b, float(o["x"]), float(o["y"]), max(h - h_sd, 0.0))
            h_cm = 0.5 * math.hypot(u1 - u2, v1 - v2) / scale if scale > 0 else 0.0
            if not np.isfinite(h_cm):
                h_cm = 0.0
            sigma = math.sqrt((speed_cm_s * dts) ** 2 + calib ** 2 + h_cm ** 2)
            score = score_scale * src_score * math.exp(-0.5 * (sigma / SCORE_REF_CM) ** 2)
            guesses.append(dict(cls=cls, x=round(ub, 2), y=round(vb, 2),
                                heading_deg=None if hb is None else round(hb, 1), heading_known=hb is not None,
                                score=round(score, 3), source="camera", from_frame=int(fa), from_obj=o.get("obj"),
                                dt_s=round(dts, 4), sigma_cm=round(sigma, 2)))
        if guesses:
            out[fb] = guesses
    return out


# ---------------------------------------------------------------- disagreements
def _reviewed(v, statuses):
    """Labelled objects of a frame, or None if the frame is not fully reviewed (has pending objects)."""
    objs = _objects(v)
    if isinstance(v, dict) and v.get("state") not in (None, "reviewed", "auto", "edited"):
        return None
    if any(o.get("status") == "pending" for o in objs):
        return None
    return [o for o in objs if o.get("status") in statuses and o.get("cls") not in (None, "none")]


def _greedy(da, db, cost):
    """Pairs (i, j) by increasing cost (inf = not allowed), each index used once."""
    cand = sorted((cost[i][j], i, j) for i in range(len(da)) for j in range(len(db)) if np.isfinite(cost[i][j]))
    ua, ub, out = set(), set(), []
    for c, i, j in cand:
        if i not in ua and j not in ub:
            ua.add(i)
            ub.add(j)
            out.append((i, j, c))
    return out


def disagreements(labels_a_by_frame, labels_b_by_frame, cam_a, cam_b, times_a, times_b, class_heights=None,
                  tol_cm=2.0, default_height_cm=4.0, match_cm=15.0, max_dt=None, offset_s=0.0, margin_px=10.0,
                  statuses=LABELLED):
    """Both cameras' labels compared on the floor, for every B frame with an A frame near in time
    where both frames are reviewed (no pending objects). Returns a list of dicts
    {frame_a, frame_b, obj_a, obj_b, cls, cls_b, dist_cm, dt_s, floor_a, floor_b, kind}:
      'moved'        same class matched (nearest, within match_cm) but > tol_cm apart on the floor;
      'class'        left over after same-class matching, an object of another class within match_cm;
      'missing_in_b' an A object with no partner, whose spot is inside B's image (margin_px in);
      'missing_in_a' likewise for a B object.
    Each label is placed on the plane at its class height (class_heights, else default_height_cm).
    Compares confirmed / corrected / auto labels by default (statuses=...), so auto-accepted
    labels that the other camera contradicts show up too."""
    pairs = match_frames(times_a, times_b, max_dt, offset_s, return_dt=True)
    out = []

    def place(cam, objs):
        res = []
        for o in objs:
            h, _ = _height(o["cls"], class_heights, default_height_cm)
            x, y = cam.to_floor(float(o["x"]), float(o["y"]), h)
            res.append((np.array([x, y]), h))
        return res

    for fb, (fa, dts) in sorted(pairs.items()):
        if fa not in labels_a_by_frame or fb not in labels_b_by_frame:
            continue
        la = _reviewed(labels_a_by_frame[fa], statuses)
        lb = _reviewed(labels_b_by_frame[fb], statuses)
        if la is None or lb is None:
            continue
        pa, pb = place(cam_a, la), place(cam_b, lb)

        def dist(i, j):
            d = float(np.linalg.norm(pa[i][0] - pb[j][0]))
            return d if np.isfinite(d) and d <= match_cm else np.inf

        same = [[dist(i, j) if la[i]["cls"] == lb[j]["cls"] else np.inf for j in range(len(lb))]
                for i in range(len(la))]
        base = dict(frame_a=int(fa), frame_b=int(fb), dt_s=round(dts, 4))

        def row(kind, i, j, d):
            oa, ob = (la[i] if i is not None else None), (lb[j] if j is not None else None)
            return dict(base, obj_a=oa and oa.get("obj"), obj_b=ob and ob.get("obj"),
                        cls=(oa or ob)["cls"], cls_b=ob["cls"] if ob else None,
                        dist_cm=None if d is None else round(d, 2),
                        floor_a=None if i is None else [round(float(a), 2) for a in pa[i][0]],
                        floor_b=None if j is None else [round(float(a), 2) for a in pb[j][0]], kind=kind)

        m1 = _greedy(la, lb, same)
        ua, ub = {i for i, _, _ in m1}, {j for _, j, _ in m1}
        for i, j, d in m1:
            if d > tol_cm:
                out.append(row("moved", i, j, d))
        ra, rb = [i for i in range(len(la)) if i not in ua], [j for j in range(len(lb)) if j not in ub]
        cross = [[dist(i, j) if j in rb else np.inf for j in range(len(lb))] if i in ra else [np.inf] * len(lb)
                 for i in range(len(la))]
        for i, j, d in _greedy(la, lb, cross):
            out.append(row("class", i, j, d))
            ua.add(i)
            ub.add(j)
        for i in range(len(la)):
            if i in ua:
                continue
            x, y = pa[i][0]
            if np.isfinite(x):
                u, v = cam_b.project((x, y, pa[i][1]))
                if cam_b.in_image(u, v, margin_px):
                    out.append(row("missing_in_b", i, None, None))
        for j in range(len(lb)):
            if j in ub:
                continue
            x, y = pb[j][0]
            if np.isfinite(x):
                u, v = cam_a.project((x, y, pb[j][1]))
                if cam_a.in_image(u, v, margin_px):
                    out.append(row("missing_in_a", None, j, None))
    return out
