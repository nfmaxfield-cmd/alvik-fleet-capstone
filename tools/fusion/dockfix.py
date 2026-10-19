"""Home dock and touch probe records (firmware dock.h, 2026-10-19a+) as fusion measurements.

DOCK (result "docked"): the robot sits flush in a home dock, so its whole pose is known: the dock's back wall
  (map: home_docks, or engine setting dock_poses) minus the bumper geometry (the run's own PARAM bump_ahead_cm):
      x = wall_x - bump_ahead cos h,  y = wall_y - bump_ahead sin h,  heading = h
  applied at ms_both (both switches clicked) as one 3-row update with dock_sigma_cm / dock_sigma_deg (about
  0.5 mm and 0.3 deg: the pocket's play and the seating; E00e measures them). Which dock: the one nearest the
  filter's estimate (a robot docks at most once at a time; a dock further than dock_max_cm is not used).
  Also a check, not a measurement: the floor stripe. The centre line sensor's travel from the stripe to the
  docking is fixed by the dock (stripe_check_cm: measured minus expected; a wheel-size or stripe-reading check).

TOUCH (result "touched"): one nose switch clicked against a mapped face. The lever's point on the bumper face
  (bump_ahead_cm ahead of the tracked point, bump_half_cm to that side; "LR": the middle) is sw_gap_cm short of
  the face straight ahead: a range measurement h(x, y, heading) = distance along the heading from that point to
  the nearest landmark face, z = sw_gap_cm, sigma touch_sigma_cm. Its Jacobian is taken numerically.

Nothing here runs unless the session has DOCK / TOUCH rows, so older data replays exactly as before.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import check_session as cs  # noqa: E402
import landmarks as lm  # noqa: E402

D2R = math.pi / 180.0


def fnum(x, default=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def attach(runs_by_id, session, fits):
    """Read dock.csv / touch.csv of a console session into run.dock / run.touch (dicts with 't', laptop s)."""
    for kind, ms_col in (("dock", "ms_both"), ("touch", "ms_click")):
        for r in cs.rows(session, kind):
            run = runs_by_id.get(r.get("run"))
            if run is None:
                continue
            fit = cs.fit_for(fits, run.robot, run.run_id)
            ms = fnum(r.get(ms_col))
            if not fit or not ms or math.isnan(ms):
                continue
            d = dict(r)
            d["t"] = cs.to_laptop(fit, ms)
            ms0 = fnum(r.get("ms_start"))
            d["t_start"] = cs.to_laptop(fit, ms0) if ms0 == ms0 and ms0 else None
            getattr(run, kind).append(d)


def geometry(run, cfg):
    """(bump_ahead, bump_half, sw_gap) cm: the run's own PARAM values, else the engine settings."""
    p = run.params or {}
    g = lambda k: p.get(k) if p.get(k) is not None and not math.isnan(p.get(k)) else cfg[k]
    return g("bump_ahead_cm"), g("bump_half_cm"), g("sw_gap_cm")


def dock_pose(wall, bump_ahead):
    """Robot pose (x, y, heading rad) when flush in a dock whose back wall middle is wall = (x, y, h deg)."""
    h = wall[2] * D2R
    return wall[0] - bump_ahead * math.cos(h), wall[1] - bump_ahead * math.sin(h), h


def stripe_check(rec, run, cfg):
    """Measured minus expected travel from the stripe (centre line sensor) to the docking, cm (nan if none)."""
    a, _, _ = geometry(run, cfg)
    s, t = fnum(rec.get("stripe_c_cm")), fnum(rec.get("travel_cm"))
    if math.isnan(s) or s < 0 or math.isnan(t):
        return float("nan")
    expect = -a - (cfg["dock_stripe_x0_cm"] - cfg["line_forward_cm"])
    return (t - s) - expect


def events(runs, cfg):
    """[(t, order, (kind, run index, data))] for run_engine."""
    out = []
    if not cfg["dock_fix"]:
        return out
    for ri, run in enumerate(runs):
        a, half, gap = geometry(run, cfg)
        for rec in getattr(run, "dock", []):
            if rec.get("result") != "docked":
                continue
            out.append((rec["t"], 2, ("dock", ri, dict(bump_ahead=a, stripe=stripe_check(rec, run, cfg),
                                                       rec=rec))))
        for rec in getattr(run, "touch", []):
            if rec.get("result") != "touched":
                continue
            side = rec.get("side", "")
            lat = 0.0 if side == "LR" else half if side == "L" else -half if side == "R" else None
            if lat is None:
                continue
            out.append((rec["t"], 2, ("touch", ri, dict(fwd=a, lat=lat, gap=gap, rec=rec))))
    return out


def apply_dock(tr, d, cfg):
    """One DOCK fix on a Tracked robot's filter. Returns the dock used (or None)."""
    k = tr.ekf
    walls = cfg["dock_poses"]
    if not walls:
        return None
    best = None
    for w in walls:
        x, y, h = dock_pose(w, d["bump_ahead"])
        dist = math.hypot(x - k.s[0], y - k.s[1])
        if best is None or dist < best[0]:
            best = (dist, (x, y, h), w)
    lost = math.sqrt(max(k.P[0, 0], k.P[1, 1])) > cfg["dock_max_cm"]
    if best[0] > cfg["dock_max_cm"] and not lost:
        tr.dock_rejected = getattr(tr, "dock_rejected", 0) + 1
        return None
    x, y, h = best[1]
    sx, sh = cfg["dock_sigma_cm"], cfg["dock_sigma_deg"] * D2R
    if lost:
        k.reset_position(x, y, sx, th=h, sigma_th=sh)
        tr.lost_state = False
    else:
        H = np.zeros((3, k.N))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        k._update(np.array([x, y, h]), k.s[:3].copy(), H, np.diag([sx * sx, sx * sx, sh * sh]), angle_rows=(2,))
    k.last_cam = k.t
    tr.dock_fixes = getattr(tr, "dock_fixes", []) + [dict(t=k.t, x=x, y=y, th=h, wall=best[2], stripe=d["stripe"])]
    return best[2]


def touch_range(s, fwd, lat, faces, back=0.5):
    """Distance straight ahead (along heading s[2]) from the lever point to the nearest face; None if none."""
    c, sn = math.cos(s[2]), math.sin(s[2])
    q = (s[0] + fwd * c - lat * sn - back * c, s[1] + fwd * sn + lat * c - back * sn)
    hit = lm.nearest(q, (c, sn), faces)
    return None if hit is None else hit[0] - back


def apply_touch(tr, d, cfg):
    """One TOUCH range on a Tracked robot's filter. Returns the squared Mahalanobis distance, or None."""
    faces = cfg["touch_faces"] or tr.faces
    if not faces:
        return None
    k = tr.ekf
    s = k.s[:3].copy()
    h = touch_range(s, d["fwd"], d["lat"], faces)
    if h is None or h > cfg["touch_max_cm"]:
        return None
    J = np.zeros(3)
    for j, e in enumerate((0.01, 0.01, 0.001)):
        sp = s.copy()
        sp[j] += e
        hp = touch_range(sp, d["fwd"], d["lat"], faces)
        if hp is None:
            return None
        J[j] = (hp - h) / e
    sig = cfg["touch_sigma_cm"]
    d2 = k.range_d2(d["gap"], h, J, sig)
    if d2 > cfg["touch_gate_chi2"] and math.sqrt(max(k.P[0, 0], k.P[1, 1])) < cfg["touch_max_cm"]:
        tr.touch_rejected = getattr(tr, "touch_rejected", 0) + 1
        return None
    k.landmark_range(d["gap"], h, J, sig)
    tr.touch_used = getattr(tr, "touch_used", 0) + 1
    return d2
