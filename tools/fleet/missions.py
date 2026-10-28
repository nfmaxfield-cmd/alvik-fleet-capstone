#!/usr/bin/env python3
"""Run modes on the planner (school demo, docs/build-plans/mission-control.md item 6, 2026-10-27). Each mode turns
into ordinary planner tasks (map.json "tasks": goto / spot / wait_s / until_s items), so the planner keeps every pair
of robots apart, re-plans round a dropped brick, and run_plan.py drives it:

  instructions   "go to A · wait 5 s · go to C · repeat 3x" -> tasks (named spots)
  mow            one full pass of lanes (MOW_LANE_CM apart, MOW_INSET_CM in from every side, or the planner's least
                 room from an open side if that is more); lanes carry their line, so a lane end inside a new obstacle
                 slides along its lane and the route round the obstacle is planned (planner.py, "lane" items)
  chicken        two robots drive at each other and swerve when their fronts are chicken_swerve_cm apart; the plan's
                 closest approach is checked against chicken_min_cm
  follow         a leader drives a loop; the follower takes the same points one behind
  roam           random spots for about the given time
  show           the opening dance: formations on beat targets ("until_s"), all three
  monday         Show -> Center mows (right of way) while Left and Right do Chicken x2, Follow the leader 60 s, Roam
                 60 s (more roaming if Center is not done yet) -> all three park in a row

    python3 tools/fleet/missions.py monday --map table.json --out monday_map.json
    python3 tools/fleet/run_plan.py --map monday_map.json --dry-run ...

Times here are estimates (est_s: drives at cruise speed, turns at the turn rate); the planner's plan has the real
timing and the summary (missions.summary) compares it with the beat targets.
"""
import argparse
import copy
import json
import math
import os
import random
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM   # noqa: E402
import planner as P     # noqa: E402

# named settings (every one editable from any view: Nick, 2026-10-27 00:08)
SETTINGS = {
    "mow_lane_cm": 12.0,          # Nick (brief): lanes 12 cm apart
    "mow_inset_cm": 10.0,         # Nick (brief): 10 cm in from every edge (raised to the planner's least room from an
                                  # open side, 11.2 cm, where that is more: said in the summary)
    "chicken_swerve_cm": 15.0,    # Nick (2026-10-27 00:08): swerve when the fronts are 15 cm apart
    "chicken_min_cm": 8.0,        # Nick: never under 8 cm (two half-widths + ~3 cm camera error, rounded up; DOE on
                                  # the board: closest approach measured by the camera over 10 chickens)
    "chicken_run_cm": 45.0,       # chosen: each robot starts this far from the middle (a run of ~25 cm at cruise
                                  # speed before the swerve: long enough to look like a dare). No DOE (looks)
    "chicken_aside_cm": 20.0,     # chosen: the swerve drives this far sideways (clears the other robot's body
                                  # circle with room). No DOE (geometry)
    "chicken_rounds": 2,          # Nick (brief): twice
    "follow_s": 60.0,             # Nick (brief): 60 s
    "roam_s": 60.0,               # Nick (brief): 60 s
    "roam_hop_cm": 30.0,          # chosen: each random spot at least this far from the last (so it looks like
                                  # roaming, not shuffling). No DOE (looks)
    "show_bpm": 120.0,            # the dance composer's tempo (e02_dance), no music on Monday (Nick)
    "park_gap_cm": 24.0,          # the final row's centres this far apart: two turn circles (2 x 6.72 cm) plus the
                                  # 8 cm separation, rounded up (20 cm failed to plan: neighbours turning on their
                                  # spots came 6.6 cm apart). Geometry, no DOE
    "separation_cm": 8.0,         # the plan's closest two bodies for these missions: chicken_min_cm, for every pair
}

ROLE_OF = {"Left": "A", "Center": "B", "Right": "C"}


def settings(over=None):
    s = dict(SETTINGS)
    for k, v in (over or {}).items():
        if k not in SETTINGS:
            raise ValueError("unknown mission setting %r" % k)
        s[k] = type(SETTINGS[k])(v)
    return s


# ---------------------------------------------------------------------------- estimates and helpers

_PL = {}


def plan_s(m, visits, start):
    """Time (s) the planner's own steps take for these visits from start (x, y, heading deg), without waits for
    other robots (exact for the steps; est_s is the quick rough figure)."""
    key = id(m)
    if key not in _PL:
        _PL.clear()
        _PL[key] = P.Planner(m)
    pl = _PL[key]
    job = P.Job("x", [start[0], start[1], math.radians(start[2] if len(start) > 2 else 0.0)], list(visits), False,
                [start[0], start[1], 0.0])
    pr = P.build_prims(job, list(range(len(visits))), pl.grid, pl.world, pl.docks, pl.s, pl.body, [])
    return sum(p_["dur"] for p_ in pr) * pl.s.tick_s


def est_s(points, start, s=None):
    """Rough time (s) to visit points [(x, y)] from start (x, y, heading deg): cruise speed and turn rate."""
    s = s or P.settings()
    t, (x, y, h) = 0.0, (start[0], start[1], start[2] if len(start) > 2 else 0.0)
    for p in points:
        L = math.dist((x, y), p[:2])
        if L > 0.5:
            hd = math.degrees(math.atan2(p[1] - y, p[0] - x))
            dt = abs((hd - h + 180) % 360 - 180)
            t += dt / s.turn_dps + s.turn_settle_s + L / s.cruise_cms + s.drive_settle_s
            h = hd
        x, y = p[0], p[1]
    return t


def least_open_room(m):
    """The least room the planner leaves between a robot's centre and an open side (turnable there)."""
    pl = P.Planner(m)
    return pl.grid.turn_thr + float(m.get("open_margin_cm", FM.OPEN_MARGIN_CM))


def _centre(m):
    f = m["floor"]
    return (f["x_min"] + f["x_max"]) / 2.0, (f["y_min"] + f["y_max"]) / 2.0


def _g(x, y, h=None, **kw):
    d = {"goto": [round(x, 2), round(y, 2)] + ([round(h, 1)] if h is not None else [])}
    d.update(kw)
    return d


# ---------------------------------------------------------------------------- go to spots + instruction lists

_STEP = re.compile(r"^\s*(go to|goto|go)\s+([A-Za-z]\w*)\s*$|^\s*wait\s+([\d.]+)\s*s?\s*$|^\s*repeat\s+(\d+)\s*[x×]?\s*$",
                   re.I)


def instructions(text, spots):
    """'go to A · wait 5 s · go to C · repeat 3x' -> visits. Steps are split by '·', ';', ',' or new lines; 'repeat
    N x' repeats everything before it (since the last repeat) N times in all."""
    parts = [p for p in re.split(r"[·;,\n]", text) if p.strip()]
    out, block = [], []
    for p in parts:
        mm = _STEP.match(p)
        if not mm:
            raise ValueError("I can't read %r (use: go to A, wait 5 s, repeat 3x)" % p.strip())
        if mm.group(2):
            nm = mm.group(2).upper()
            if nm not in spots:
                raise ValueError("there is no spot %s (spots: %s)" % (nm, ", ".join(sorted(spots)) or "none"))
            block.append({"spot": nm})
        elif mm.group(3):
            block.append({"wait_s": float(mm.group(3))})
        else:
            n = int(mm.group(4))
            if n < 1:
                raise ValueError("repeat needs 1 or more")
            out += copy.deepcopy(block) * n
            block = []
    return out + block


# ---------------------------------------------------------------------------- lawnmower

def mow(m, s=None, robot_start=None, along="long"):
    """One full pass: lanes MOW lane spacing apart, alternating direction, along the table's long side (along="long")
    or its short side (along="short": the mower then sweeps across the table's length, which leaves the far part free
    for the others: Monday). Starts from the lane end nearest robot_start. Returns (visits, notes)."""
    s = settings(s)
    f = m["floor"]
    inset = s["mow_inset_cm"]
    notes = []
    need = least_open_room(m)
    if m.get("sides") and need > inset:
        notes.append("lanes start %.1f cm in from the open sides, not %.0f: the planner keeps a robot's centre %.1f cm "
                     "from an open edge (turn circle + safety + open margin), plus 1 cm" % (need + 1.0, inset, need))
        inset = need + 1.0             # 1 cm inside the planner's limit: a robot stopping a little past a lane end
                                       # can still turn there (M1: 0.12 cm past it boxed Center in)
    W, H = f["x_max"] - f["x_min"], f["y_max"] - f["y_min"]
    along_x = (W >= H) if along == "long" else (W < H)
    lo, hi = (f["y_min"] + inset, f["y_max"] - inset) if along_x else (f["x_min"] + inset, f["x_max"] - inset)
    a0, a1 = (f["x_min"] + inset, f["x_max"] - inset) if along_x else (f["y_min"] + inset, f["y_max"] - inset)
    lanes, v = [], lo
    while v <= hi + 1e-6:
        lanes.append(v)
        v += s["mow_lane_cm"]
    if hi - lanes[-1] > s["mow_lane_cm"] / 2:
        lanes.append(hi)
    # start from the lane end nearest the robot
    flip = False
    if robot_start is not None:
        def pt(c, e):
            return (e, c) if along_x else (c, e)
        ends = {(0, 0): pt(lanes[0], a0), (0, 1): pt(lanes[0], a1), (1, 0): pt(lanes[-1], a0), (1, 1): pt(lanes[-1], a1)}
        best = min(ends, key=lambda k_: math.dist(robot_start[:2], ends[k_]))
        if best[0] == 1:
            lanes = lanes[::-1]
        flip = best[1] == 1
    out = []
    for k, c in enumerate(lanes):
        e0, e1 = (a0, a1) if (k % 2 == 0) != flip else (a1, a0)
        line = [e0, c, e1, c] if along_x else [c, e0, c, e1]
        for j, e in enumerate((e0, e1)):
            x, y = (e, c) if along_x else (c, e)
            out.append(_g(x, y, label="mow lane %d %s" % (k + 1, "start" if j == 0 else "end"), lane=line, end=j))
    notes.append("%d lanes %.0f cm apart, %.0f cm long" % (len(lanes), s["mow_lane_cm"], a1 - a0))
    return out, notes


# ---------------------------------------------------------------------------- chicken

def chicken(m, a="Left", b="Right", s=None, t0=0.0, centre=None):
    """Two robots drive at each other along the table's middle line and swerve (each to its own left) when their
    fronts are chicken_swerve_cm apart; then back to their ends for the next round. Returns {robot: visits}, and the
    swerve points (for the closest-approach check)."""
    s = settings(s)
    cx, cy = centre or _centre(m)
    body = FM.body_of(FM.normalize(m))
    half = body.front
    meet = s["chicken_swerve_cm"] / 2.0 + half       # each centre this far from the middle at the swerve
    R = s["chicken_run_cm"]
    out = {a: [], b: []}
    # each swerves to its own left (they pass on opposite sides), unless a side has no room or another robot stands
    # there: then both swerve to the side with room
    others = [r["start"][:2] for r in FM.normalize(m)["robots"] if r["name"] not in (a, b)]
    pl = P.Planner(m)
    asd = s["chicken_aside_cm"]

    def ok(x, y):
        return pl.grid.turnable([x, y]) and all(math.dist((x, y), o) > 2 * 6.8 + s["chicken_min_cm"] for o in others)
    sa, sb = 1.0, -1.0
    if not (ok(cx - meet, cy + sa * asd) and ok(cx + meet, cy + sb * asd)):
        sa = sb = 1.0 if ok(cx - meet, cy + asd) and ok(cx + meet, cy + asd) else -1.0
    for k in range(int(s["chicken_rounds"])):
        lab = "chicken %d" % (k + 1)
        out[a] += [_g(cx - R, cy, 0.0, label=lab + " ready"), {"wait_s": 0.5, "label": lab + " ready"},
                   _g(cx - meet, cy, label=lab + " swerve"), _g(cx - meet, cy + sa * asd, label=lab + " aside")]
        out[b] += [_g(cx + R, cy, 180.0, label=lab + " ready"), {"wait_s": 0.5, "label": lab + " ready"},
                   _g(cx + meet, cy, label=lab + " swerve"), _g(cx + meet, cy + sb * asd, label=lab + " aside")]
    return out


# ---------------------------------------------------------------------------- follow the leader, roam

def follow(m, leader="Left", follower="Right", s=None, start=None, x_range=None):
    """The leader drives a winding path (rows across the region, never back over itself) for about follow_s; the
    follower takes the same points one behind. Points are FOLLOW_STEP_CM apart, so the two keep the separation even
    while both turn on their spots. (A closed loop was tried first: on a small table the leader came round again to
    where the follower was waiting, and no plan could be made.) Returns {robot: visits}."""
    s = settings(s)
    f = m["floor"]
    room = least_open_room(m) + 2.0
    x0, x1 = x_range if x_range else (f["x_min"] + room, f["x_max"] - room)
    y0, y1 = f["y_min"] + room, f["y_max"] - room
    step = FOLLOW_STEP_CM
    nx = max(1, int((x1 - x0) // step) + 1)
    ny = max(1, int((y1 - y0) // step) + 1)
    xs = [x0 + (x1 - x0) * (i / (nx - 1) if nx > 1 else 0.5) for i in range(nx)]
    ys = [y0 + (y1 - y0) * (j / (ny - 1) if ny > 1 else 0.5) for j in range(ny)]
    # serpentine along the region's longer side
    pts = []
    if (x1 - x0) >= (y1 - y0):
        for j, y in enumerate(ys):
            pts += [(x, y) for x in (xs if j % 2 == 0 else xs[::-1])]
    else:
        for i, x in enumerate(xs):
            pts += [(x, y) for y in (ys if i % 2 == 0 else ys[::-1])]
    mm_ = FM.normalize(m)
    st = start or (pts[0][0], pts[0][1], 90.0)
    while len(pts) > 3 and plan_s(mm_, [_g(x, y) for x, y in pts[1:-1]], st) >= s["follow_s"]:
        pts.pop()
    lab = "follow the leader"
    lead = [_g(x, y, label=lab) for x, y in pts[1:]]
    foll = [_g(x, y, label=lab) for x, y in pts[:-1]]
    return {leader: lead, follower: foll}


def roam(m, robot, s=None, seed=1, seconds=None, start=None, keep_out=(), x_range_at=None, y_range=None):
    """Random spots (the planner's turnable cells) for about `seconds` (roam_s). keep_out: [(x, y, r)] to avoid.
    x_range_at(t): the x range allowed for a spot reached t s into the roam (Monday: away from the mower)."""
    s = settings(s)
    seconds = s["roam_s"] if seconds is None else seconds
    pl = P.Planner(m)
    g = pl.grid
    free = g.centres[g.free]
    rng = random.Random(seed)
    st = start or (free[0][0], free[0][1], 0.0)
    pts, cur = [], st[:2]
    tries = 0
    mm_ = FM.normalize(m)
    while (plan_s(mm_, [_g(x, y) for x, y in pts], st) if pts else 0.0) < seconds and tries < 5000:
        tries += 1
        p = free[rng.randrange(len(free))]
        if math.dist(p, cur) < s["roam_hop_cm"] or any(math.dist(p, (kx, ky)) < kr for kx, ky, kr in keep_out):
            continue
        if y_range is not None and not y_range[0] <= p[1] <= y_range[1]:
            continue
        if x_range_at is not None:
            lo, hi = x_range_at(plan_s(mm_, [_g(x, y) for x, y in pts + [(float(p[0]), float(p[1]))]], st))
            if not lo <= p[0] <= hi:
                continue
        pts.append((float(p[0]), float(p[1])))
        cur = p
    return [_g(x, y, label="roam") for x, y in pts]


# ---------------------------------------------------------------------------- the show

def show(m, names=("Left", "Center", "Right"), s=None):
    """The opening show (about 55 s, all three, no music): figures on beat targets. Each robot dances in its own third
    of the table, so no figure needs one robot to wait for another and the beats hold (the planner still checks
    every pair). until_s holds each robot until its beat. Returns ({robot: visits}, end time s, [figure dict(name,
    words, t)])."""
    s = settings(s)
    spb = 60.0 / s["show_bpm"]
    cx, cy = _centre(m)
    f = m["floor"]
    room = least_open_room(m) + 1.0
    W = f["x_max"] - f["x_min"]
    dx = min(40.0, (W - 2 * room) / 3.0)
    sq = min(8.0, ((f["y_max"] - f["y_min"]) / 2 - room) / 2, dx / 2 - 8.0)
    dy = min(16.0, (f["y_max"] - f["y_min"]) / 2 - room)
    xs = {n: cx + k * dx for n, k in zip(names, (-1, 0, 1))}
    figs, out = [], {n: [] for n in names}
    beat = [0.0]

    def fig(name, words, beats_later):
        beat[0] += beats_later
        figs.append({"name": name, "words": words, "t": round(beat[0] * spb, 2), "beat": beat[0]})
        return beat[0] * spb

    def at(n, t, x, y, h=None):
        out[n] += [{"until_s": round(t, 2), "label": figs[-1]["name"]}, _g(x, y, h, label=figs[-1]["name"])]

    t = fig("Line up", "All three line up across the middle, facing the audience.", 0)
    for n in names:
        at(n, t, xs[n], cy, 90.0)
    t = fig("Ripple", "A quarter turn and back runs along the line, one robot after the other.", 16)
    for k, n in enumerate(names):
        at(n, t + 2 * k * spb, xs[n], cy, 0.0)
        at(n, t + (2 * k + 4) * spb, xs[n], cy, 90.0)
    t = fig("Wave", "Each robot steps forward and back in turn, like a wave.", 14)
    for k, n in enumerate(names):
        at(n, t + 3 * k * spb, xs[n], cy + dy, 90.0)
        at(n, t + (3 * k + 6) * spb, xs[n], cy, 90.0)
    t = fig("Squares", "All three drive a small square together, the two ends turning the opposite way to the middle.",
            26)
    for k, n in enumerate(names):
        sgn = 1.0 if k == 1 else -1.0
        corners = [(xs[n] + sgn * sq, cy), (xs[n] + sgn * sq, cy + 2 * sq), (xs[n] - sgn * sq, cy + 2 * sq),
                   (xs[n] - sgn * sq, cy), (xs[n], cy)]
        for j, (x, y) in enumerate(corners):
            at(n, t + 8 * j * spb, x, y)
    t = fig("Finale", "Facing the audience; the ends turn out and back as a bow.", 40)
    for n in names:
        at(n, t, xs[n], cy, 90.0)
    at(names[0], t + 2 * spb, xs[names[0]], cy, 180.0)
    at(names[2], t + 2 * spb, xs[names[2]], cy, 0.0)
    at(names[0], t + 7 * spb, xs[names[0]], cy, 90.0)
    at(names[2], t + 7 * spb, xs[names[2]], cy, 90.0)
    end = t + 12 * spb
    figs[0]["line"] = {n: [round(xs[n], 2), round(cy, 2), 90.0] for n in names}     # the line-up spots
    return out, end, figs


# ---------------------------------------------------------------------------- Monday's routine

MOWER_CLEAR_CM = 30.0    # chosen: Left and Right play at least this far (in x) from where the mower's centre will be:
                         # two turn circles (13.4 cm) + the 8 cm separation + ~8 cm for the estimate's timing error.
                         # DOE note: 20 / 30 / 40 cm against the planned waits for the mower (dry runs)


FOLLOW_STEP_CM = 25.0       # follow the leader: points this far apart (two turn circles 13.4 cm + the 8 cm separation +
                            # 3.6 cm). Geometry, no DOE
CHICKEN_MIN_RUN_CM = 10.0   # chosen: the shortest run before the swerve that still reads as "driving at each other"
                            # (1 s at cruise speed); with 15 cm the second round found no room beside the mower on a
                            # 152 cm table. No DOE (looks); Nick may prefer it longer on a longer table
LR_SLACK_S = 30.0        # chosen: Left and Right's routine is planned to end this long before Center is done (their
                         # waits for the mower and each other came to 8-30 s in the plans). DOE note: 15 / 30 / 45 s
                         # against the plan's Left / Right lateness and idle time at the end (planner only, no robots)


BAND_PAD_S = 5.0         # chosen: the mower's band is taken over its window widened this much each way (Left and Right
                         # run a few seconds behind their own step times: waits for each other; 15 s left no room
                         # for Chicken on a 152 cm table). DOE note: 0 / 5 / 15 s against Monday plans found and
                         # their waits (planner only)


def mower_band(plan_c, name, t0, t1, step=0.5):
    """(min x, max x) of the mower's centre between mission times t0 - BAND_PAD_S and t1 + BAND_PAD_S (its plan made
    alone)."""
    t0, t1 = max(0.0, t0 - BAND_PAD_S), t1 + BAND_PAD_S
    xs = [P.pose_at(plan_c, name, t)[0] for t in [t0 + k * step for k in range(int((t1 - t0) / step) + 1)] + [t1]]
    return min(xs), max(xs)


def free_x(m, band, need=0.0):
    """The larger x range of the table away from the mower's band (MOWER_CLEAR_CM), or None if less than need."""
    f = m["floor"]
    room = least_open_room(m)
    lo_side = (f["x_min"] + room, band[0] - MOWER_CLEAR_CM)
    hi_side = (band[1] + MOWER_CLEAR_CM, f["x_max"] - room)
    best = max((lo_side, hi_side), key=lambda r: r[1] - r[0])
    return best if best[1] - best[0] >= need else None


def monday(m, s=None, seed=1, tries=6):
    """Monday's routine (see _monday), with Roam and Follow shortened until Left and Right's own steps end
    LR_SLACK_S before Center is done (their steps timed by the planner, without waits)."""
    k = 1.0
    for _ in range(tries):
        out = _monday(m, s, seed, k)
        mm = FM.normalize(out)
        pl = P.Planner(mm)
        ends = []
        for j in P.jobs_from_map(mm):
            if j.name in ("Left", "Right"):
                vis = [v for v in j.visits if v.get("label") != "park"][:-1]     # up to the wait above the park spot
                pr = P.build_prims(P.Job(j.name, j.start, vis, False, j.start), list(range(len(vis))), pl.grid,
                                   pl.world, pl.docks, pl.s, pl.body, [])
                ends.append(sum(p_["dur"] for p_ in pr) * pl.s.tick_s)
        t_c = out["mission"]["center_done_s"]
        out["mission"]["left_right_end_s"] = round(max(ends), 1)
        if max(ends) <= t_c - LR_SLACK_S:
            return out
        k *= max(0.5, (t_c - LR_SLACK_S - out["mission"]["show_end_s"]) /
                 max(1.0, max(ends) - out["mission"]["show_end_s"]))
    out["mission"]["notes"].append("Left and Right's routine still ends %.0f s after Center (the planner makes them "
                                   "wait; check the plan)" % (max(ends) - t_c))
    return out


def _monday(m, s=None, seed=1, k_user=1.0):
    """Show -> split -> park. Center mows one full pass with right of way: lanes across the table's short side, swept
    from one end of the table to the other, so at any moment it is in one strip of the table. Left and Right play in
    the part of the table it is not in (its timing comes from its own plan, made first): Chicken x2 and Roam where
    it has not reached yet, then Follow the leader where it has already been, then more roaming until it is done;
    they park once it is done, in a row along the audience's side (the map's y_min side)."""
    s = settings(s)
    m = FM.normalize(m)
    names = [r["name"] for r in m["robots"]]
    if set(names) != {"Left", "Center", "Right"}:
        raise ValueError("Monday's routine needs Left, Center and Right (map has %s)" % ", ".join(names))
    f = m["floor"]
    cx, cy = _centre(m)
    # the show's line is Center, Left, Right from the x_min end: Center finishes the show at that end and starts
    # mowing there, with nobody between it and its first lane (with Center in the middle of the line, its drive to
    # either end passed within 3 cm of Left or Right standing in the line, and no plan could be made)
    sh, t_show, figs = show(m, ("Center", "Left", "Right"), s)
    split = {"until_s": round(t_show, 2), "label": "split"}
    # the robots start on their line-up spots (Mission Control shows them; a positioning plan brings them there):
    # from wherever they stand, Left and Center would have to cross on the way to the line, and the planner, with
    # Center first, found no order (dry runs 2026-10-27)
    for r in m["robots"]:
        r["start"] = list(figs[0]["line"][r["name"]])
    mow_v, notes = mow(m, s, robot_start=(f["x_min"], f["y_max"]), along="short")
    room = least_open_room(m) + 2.0
    row_y = f["y_min"] + room
    # the row: Center at the end it comes from (it finishes mowing at the x_max end), Left and Right beside it
    # towards the end they wait at (the x_min end, mowed first), so nobody crosses anybody on the way in
    for r in m["robots"]:
        k = {"Left": -1, "Right": 0, "Center": 1}[r["name"]]
        r["park"] = [round(cx + k * s["park_gap_cm"], 2), round(row_y, 2), 90.0]
    mc = copy.deepcopy(m)
    mc["robots"] = [r for r in m["robots"] if r["name"] == "Center"]
    mc["tasks"] = {"Center": {"visit": sh["Center"] + [split] + mow_v, "park": True}}
    mc["mission"] = {"planner": {"separation_cm": s["separation_cm"]}}
    plan_c = P.make_plan(mc)
    t_center = plan_c["robots"]["Center"]["finish_s"]
    ch_min = 2 * (s["chicken_swerve_cm"] / 2.0 + 4.75 + CHICKEN_MIN_RUN_CM) + 2 * 6.72

    seqs = {"Left": [], "Right": []}
    # the band is judged from when the mower reaches its first lane (its drive there crosses half the table)
    first = next((st["t_end"] for st in plan_c["robots"]["Center"]["steps"]
                  if st.get("label") == "mow lane 1 start"), t_show)
    t = max(t_show, first)
    # 1. Chicken, round by round, where the mower is not
    for k in range(int(s["chicken_rounds"])):
        s1 = dict(s, chicken_rounds=1)
        rough = 25.0
        rng_x = free_x(m, mower_band(plan_c, "Center", t, t + rough), need=ch_min)
        if rng_x is not None:
            # the run fits the room away from the mower (chicken_run_cm at most)
            s1["chicken_run_cm"] = min(s["chicken_run_cm"], (rng_x[1] - rng_x[0] - 2 * 6.72) / 2.0)
        if rng_x is None:
            notes.append("chicken round %d: no room away from the mower at %.0f s: done where it is, the planner "
                         "makes them wait for it" % (k + 1, t))
            rng_x = (f["x_min"] + room, f["x_max"] - room)
        c = ((rng_x[0] + rng_x[1]) / 2.0, cy)
        ch = chicken(m, s=s1, centre=c)
        for n in seqs:
            for v in ch[n]:
                if v.get("label"):
                    v["label"] = v["label"].replace("chicken 1", "chicken %d" % (k + 1))
            seqs[n] += ch[n]
        t += plan_s(m, ch["Left"], (c[0] - s1["chicken_run_cm"], cy, 0.0))
    # Roam and Follow fit in what is left of Center's pass (LR_SLACK_S early): shortened together if need be
    avail = t_center - LR_SLACK_S - t
    k_fit = min(1.0, max(0.3, avail / (s["roam_s"] + s["follow_s"]))) * k_user
    if k_fit < 1.0:
        notes.append("Roam and Follow the leader shortened to %.0f s and %.0f s so Left and Right finish with Center "
                     "(its pass takes about %.0f s after the show)" % (s["roam_s"] * k_fit, s["follow_s"] * k_fit,
                                                                     t_center - t_show))
    s = dict(s, roam_s=s["roam_s"] * k_fit, follow_s=s["follow_s"] * k_fit)
    # 2. Roam where the mower is not, spot by spot
    t_roam0 = t

    def x_at(dt):
        r_ = free_x(m, mower_band(plan_c, "Center", t_roam0 + dt - 3.0, t_roam0 + dt + 6.0), need=20.0)
        return r_ or (f["x_min"] + room, f["x_max"] - room)
    # each roams in its own band (top / bottom half), the bands two turn circles + the separation apart: two robots
    # roaming at random over the same ground with fixed spot lists cornered each other (no plan, dry run 2026-10-27)
    gap = 2 * 6.72 + s["separation_cm"]
    bands = {"Left": (cy + gap / 2.0, f["y_max"]), "Right": (f["y_min"], cy - gap / 2.0)}
    roams = {}
    for k, n in enumerate(("Left", "Right")):
        roams[n] = roam(m, n, s, seed=seed + k, seconds=s["roam_s"], start=(cx, cy, 90.0), x_range_at=x_at,
                        y_range=bands[n])
        seqs[n] += roams[n]
    t += s["roam_s"]
    # 3. Follow the leader (60 s) where the mower has been (or is not)
    band = mower_band(plan_c, "Center", t, t + s["follow_s"])
    rng_x = free_x(m, band, need=0.0)          # the larger side away from the mower, however narrow: a small loop
    fo = follow(m, s=s, x_range=rng_x)
    for n in seqs:
        seqs[n] += fo[n]
    t += s["follow_s"]
    # 4. then they wait above their park spots until Center is done (no extra roaming: the planner's waits for the
    #    mower made any filler run past Center's end)
    tasks = {"Center": {"visit": sh["Center"] + [split] + mow_v, "park": True}}
    # until Center is done, Left and Right wait straight above their park spots, then drive straight down into them
    # (a waiting spot elsewhere made their last drives cross: no plan, dry run 2026-10-27)
    for n in ("Left", "Right"):
        pk = next(r["park"] for r in m["robots"] if r["name"] == n)
        tasks[n] = {"visit": sh[n] + [split] + seqs[n] + [_g(pk[0], pk[1] + 22.0, 270.0, label="wait for Center"),
                                                          {"until_s": round(t_center, 1), "label": "park"}],
                    "park": True}
    out = dict(m)
    out["tasks"] = tasks
    out["mission"] = {"name": "Monday", "settings": s, "show_end_s": round(t_show, 2), "figures": figs,
                      "center_done_s": round(t_center, 1),
                      "notes": notes + ["Center has right of way (planned first); it sweeps the table end to end",
                                        "Left and Right: Chicken x2, Roam %.0f s, Follow the leader %.0f s, then "
                                        "wait above their park spots until Center is done (about %.0f s), then park" % (s["roam_s"],
                                                                                            s["follow_s"], t_center)],
                      "planner": {"separation_cm": s["separation_cm"], "priority_first": "Center"}}
    return out


# ============================================================================ blocks (one system, 2026-10-28)
# Nick (2026-10-27 18:36): "the software used to craft runs": a mission is a list of blocks, each for one or more
# robots, editable on Mission Control's Mission screen. Blocks: mow-region (short lanes, stop-and-turn), go-to-spot,
# meet, circle (a true arc), chicken, spread-out (park), dance, wait. Places are given as fractions of the table
# (0..1 along x and y), so the same mission fits any table (Monday: 61 x 122 to 76 x 137 cm; exact size Thursday).
# A block applies to its robots in turn; build() turns the list into planner tasks per robot (visit items), keeping
# track of where each robot ends, so the next block starts from there.

BLOCKS = ("mow-region", "go-to-spot", "meet", "circle", "chicken", "spread-out", "dance", "wait")

CIRCLE_R_CM = 15.0       # chosen: the circle's radius: two robots 180 deg apart on it are 30 cm centre to centre (bodies
                         # ~20 cm), and pieces two apart (60 deg) stay clear, so both drive it together without waits.
                         # 12 cm made neighbouring pieces meet (the planner held one robot back). DOE note: 12 / 15 / 18 cm
                         # on the board: the camera's closest approach and radial error against how it looks
MEET_RUN_IN_CM = 12.0    # chosen: the straight run-in to a circle's entry point (camera heading before the circle,
                         # Nick's "confirm before anything risky"). No DOE beyond the run-in's own (confirm_cms row)
MEET_SLACK_S = 3.0       # chosen: the meet time is planned this long after the later robot's estimated arrival (the
                         # estimate leaves out waits for the other robot). DOE note: 1 / 3 / 6 s against the plan's
                         # waits at the meet (planner only)
CHICKEN_RUN_IN_CM = 12.0  # chosen: Chicken's ready spot is reached on a straight run-in along the chicken line (the
                          # heading confirmed before driving at the other robot). No DOE (as MEET_RUN_IN_CM)


def _frac(m, fx, fy):
    f = m["floor"]
    return f["x_min"] + fx * (f["x_max"] - f["x_min"]), f["y_min"] + fy * (f["y_max"] - f["y_min"])


def lanes_x(m, s):
    """Lane positions across the table's length (lanes run along the short side, x = const for a long-x table), as
    mow(along="short") makes them, with the planner's room from open sides."""
    s = settings(s)
    f = m["floor"]
    inset = s["mow_inset_cm"]
    need = least_open_room(m)
    if m.get("sides") and need > inset:
        inset = need + 1.0
    W, H = f["x_max"] - f["x_min"], f["y_max"] - f["y_min"]
    long_x = W >= H
    lo, hi = ((f["x_min"], f["x_max"]) if long_x else (f["y_min"], f["y_max"]))
    lo, hi = lo + inset, hi - inset
    out, v = [], lo
    while v <= hi + 1e-6:
        out.append(round(v, 2))
        v += s["mow_lane_cm"]
    if hi - out[-1] > s["mow_lane_cm"] / 2:
        out.append(round(hi, 2))
    a0, a1 = ((f["y_min"] + inset, f["y_max"] - inset) if long_x else (f["x_min"] + inset, f["x_max"] - inset))
    return out, (a0, a1), long_x, inset


def mow_region(m, s, fx0, fx1, direction, start_xy):
    """The lanes whose position lies in [fx0, fx1) of the table's length (fx1 = 1 includes the last), visited in
    increasing (direction +1) or decreasing (-1) position, each lane driven end to end, alternating, starting from the
    end nearest start_xy: stop-and-turn (drive the lane, 90 deg, the lane gap over, 90 deg, back). Returns visits."""
    lanes, (a0, a1), long_x, _ = lanes_x(m, s)
    f = m["floor"]
    L0, L1 = (f["x_min"], f["x_max"]) if long_x else (f["y_min"], f["y_max"])
    p0, p1 = L0 + fx0 * (L1 - L0), L0 + fx1 * (L1 - L0)
    mine = [v for v in lanes if p0 - 1e-6 <= v < p1 - 1e-6 or (fx1 >= 1.0 and v >= p0 - 1e-6)]
    mine = sorted(mine, reverse=direction < 0)
    if not mine:
        return []
    out = []
    cur = start_xy
    for k, c in enumerate(mine):
        def pt(e):
            return (c, e) if long_x else (e, c)
        ends = (a0, a1) if math.dist(cur, pt(a0)) <= math.dist(cur, pt(a1)) else (a1, a0)
        line = [*pt(ends[0]), *pt(ends[1])]
        for j, e in enumerate(ends):
            x, y = pt(e)
            out.append(_g(x, y, label="mow lane %d %s" % (lanes.index(c) + 1, "start" if j == 0 else "end"), lane=line,
                          end=j))
        cur = pt(ends[1])
    return out


def _last_xy(visits, default):
    for v in reversed(visits):
        if "goto" in v:
            return tuple(v["goto"][:2])
        if "arc" in v:
            a = v["arc"]
            ang = math.radians(a["a0_deg"] + a["sweep_deg"])
            return a["c"][0] + a["r"] * math.cos(ang), a["c"][1] + a["r"] * math.sin(ang)
    return default


def monday_program(robots=("Left", "Right")):
    """Monday's default (Nick, 2026-10-27 18:26-18:36): 2 robots, a short presentation.
    1. Each starts at its own end of the table and mows towards the centre line in lanes along the SHORT side
       (stop-and-turn), working towards the centre line.
    2. They meet at the centre line mid-mow and circle each other 1.5 times on a true curve, which leaves them on
       swapped sides; then each carries on mowing in its direction (no backing off).
    3. At the end of their mowing they turn towards each other and play Chicken (swerve at 15 cm, never under 8 cm).
    4. They spread out, equally spaced on the table, and dance until switched off.
    Claude's reading of "mid-mow" (an assumption, flagged in the build report): each robot mows the outer quarter at its
    end, they meet and circle, and each then mows the inner quarter on the other side, going on in its own direction,
    so every lane is mowed once and they end facing each other across the middle for Chicken."""
    a, b = robots
    return {"name": "Monday, 2 robots", "robots": [a, b],
            "starts": {a: {"at": [0.0, 0.0], "face": "+y"}, b: {"at": [1.0, 0.0], "face": "+y"}},
            "blocks": [
                {"block": "mow-region", "robot": a, "from": 0.0, "to": 0.25, "direction": "+"},
                {"block": "mow-region", "robot": b, "from": 0.75, "to": 1.0, "direction": "-"},
                {"block": "meet", "robots": [a, b], "at": [0.5, 0.5], "r_cm": CIRCLE_R_CM},
                {"block": "circle", "robots": [a, b], "turns": 1.5, "way": "anticlockwise"},
                {"block": "mow-region", "robot": a, "from": 0.5, "to": 0.75, "direction": "+"},
                {"block": "mow-region", "robot": b, "from": 0.25, "to": 0.5, "direction": "-"},
                {"block": "chicken", "robots": [a, b], "swerve_cm": 15.0, "min_cm": 8.0, "rounds": 1},
                {"block": "spread-out", "robots": [a, b]},
                {"block": "dance", "robots": [a, b], "seconds": 0},
            ]}


def _robots_of(bl):
    if "robots" in bl:
        return list(bl["robots"])
    return [bl["robot"]] if bl.get("robot") else []


def build(m, program, s=None):
    """A mission program (blocks) -> a mission map: every robot's start (the program's "starts": fractions of the table,
    snapped to the first lane end near it), its tasks (visit items) and the mission's notes. Robots not in the program
    are left out of the map (Monday: 2 of the 3)."""
    s = settings(s)
    m = FM.normalize(copy.deepcopy(m))
    names = list(program.get("robots") or [r["name"] for r in m["robots"]])
    known = {r["name"]: r for r in m["robots"]}
    for n in names:
        if n not in known:
            raise ValueError("the program uses %s, which the map does not have (robots: %s)" % (n, ", ".join(known)))
    f = m["floor"]
    room = least_open_room(m) + 1.0
    visits = {n: [] for n in names}
    starts = {}
    notes = []
    lanes, (a0, a1), long_x, inset = lanes_x(m, s)
    # starts: the program's "at" (fractions), moved in to the first lane end there; facing along the short side
    for n in names:
        st = (program.get("starts") or {}).get(n)
        if st:
            x, y = _frac(m, *st["at"])
            x = min(max(x, f["x_min"] + inset), f["x_max"] - inset)
            y = min(max(y, f["y_min"] + inset), f["y_max"] - inset)
            h = {"+y": 90.0, "-y": 270.0, "+x": 0.0, "-x": 180.0}.get(st.get("face", "+y"), 90.0)
            starts[n] = [round(x, 2), round(y, 2), h]
        else:
            starts[n] = list(known[n]["start"])
    t_est = {n: 0.0 for n in names}           # each robot's own time so far (no waits), for meet times
    mm_ = FM.normalize(dict(m, robots=[dict(known[n], start=starts[n], park=starts[n]) for n in names], tasks={}))

    def advance(n, items):
        """add items to robot n, keeping its time estimate."""
        start = starts[n] if not visits[n] else None
        here = _pose_after(n)
        visits[n].extend(items)
        moving = [v for v in items if "goto" in v or "arc" in v]
        if moving:
            try:
                t_est[n] += plan_s(mm_, moving, here)
            except P.PlanError:
                t_est[n] += est_s([v["goto"][:2] for v in moving if "goto" in v], here, P.settings())
        for v in items:
            if "wait_s" in v:
                t_est[n] += v["wait_s"]
            if "until_s" in v:
                t_est[n] = max(t_est[n], v["until_s"])
        return start

    def _pose_after(n):
        xy = _last_xy(visits[n], tuple(starts[n][:2]))
        h = starts[n][2]
        for v in reversed(visits[n]):
            if "goto" in v and len(v["goto"]) > 2:
                h = v["goto"][2]
                break
            if "arc" in v:
                a = v["arc"]
                h = a["a0_deg"] + a["sweep_deg"] + (90.0 if a["sweep_deg"] >= 0 else -90.0)
                break
            if "goto" in v:
                break
        return (xy[0], xy[1], h)

    circle = {}
    for k, bl in enumerate(program["blocks"]):
        kind = bl.get("block")
        who = _robots_of(bl)
        for n in who:
            if n not in names:
                raise ValueError("block %d (%s) names %s, who is not in this mission" % (k + 1, kind, n))
        if kind == "mow-region":
            n = who[0]
            items = mow_region(m, s, float(bl.get("from", 0.0)), float(bl.get("to", 1.0)),
                               -1 if str(bl.get("direction", "+")).startswith("-") else 1, _pose_after(n)[:2])
            if not items:
                notes.append("block %d: no lane of the table lies in %s's region %.2f-%.2f" % (k + 1, n, bl.get("from", 0),
                                                                                             bl.get("to", 1)))
            advance(n, items)
        elif kind == "go-to-spot":
            n = who[0]
            if bl.get("spot"):
                sp = (m.get("spots") or {}).get(str(bl["spot"]).upper())
                if not sp:
                    raise ValueError("block %d: there is no spot %s" % (k + 1, bl["spot"]))
                x, y = sp[:2]
            else:
                x, y = _frac(m, *bl["at"])
            g = _g(x, y, bl.get("heading"), label=bl.get("label") or "go to spot")
            advance(n, [g])
        elif kind == "wait":
            for n in who:
                advance(n, [{"wait_s": float(bl.get("seconds", 1.0)), "label": "wait"}])
        elif kind == "meet":
            if len(who) != 2:
                raise ValueError("block %d: meet is for 2 robots" % (k + 1))
            c = _frac(m, *bl.get("at", [0.5, 0.5]))
            r = float(bl.get("r_cm", CIRCLE_R_CM))
            # each robot goes to the circle's point on its own side (along the table's length), facing round it
            pos = {n: _pose_after(n) for n in who}
            left, right = sorted(who, key=lambda n: pos[n][0] if long_x else pos[n][1])
            way = 1.0       # anticlockwise; the circle block may turn the other way: then the entries face the other way
            nxt = next((b2 for b2 in program["blocks"][k + 1:] if b2.get("block") == "circle"), None)
            if nxt and str(nxt.get("way", "anticlockwise")).startswith("clock"):
                way = -1.0
            ent = {left: 180.0 if long_x else 270.0, right: 0.0 if long_x else 90.0}
            circle = {"c": [round(c[0], 2), round(c[1], 2)], "r": r, "entry": ent, "way": way, "sync": "meet%d" % k}
            for n in who:
                ang = math.radians(ent[n])
                ex, ey = c[0] + r * math.cos(ang), c[1] + r * math.sin(ang)
                hd = (ent[n] + 90.0 * way) % 360.0
                advance(n, [_g(ex, ey, hd, label="meet", run_in_cm=MEET_RUN_IN_CM)])
            t_meet = max(t_est[n] for n in who) + MEET_SLACK_S
            for n in who:
                advance(n, [{"until_s": round(t_meet, 1), "label": "meet"}, {"sync": circle["sync"], "label": "meet"}])
            notes.append("meet at (%.0f, %.0f), about %.0f s in" % (c[0], c[1], t_meet))
        elif kind == "circle":
            if not circle:
                raise ValueError("block %d: circle needs a meet block before it" % (k + 1))
            turns = float(bl.get("turns", 1.5))
            for n in who:
                advance(n, [{"arc": {"c": circle["c"], "r": circle["r"], "a0_deg": circle["entry"][n],
                                     "sweep_deg": round(360.0 * turns * circle["way"], 3)}, "label": "circle"}])
            notes.append("circle %.1f times, r %.0f cm (%s)" % (turns, circle["r"],
                                                                "anticlockwise" if circle["way"] > 0 else "clockwise"))
            circle = {}
        elif kind == "chicken":
            if len(who) != 2:
                raise ValueError("block %d: chicken is for 2 robots" % (k + 1))
            pos = {n: _pose_after(n) for n in who}
            a, b = sorted(who, key=lambda n: pos[n][0])          # a comes from the low-x side
            s1 = dict(s, chicken_swerve_cm=float(bl.get("swerve_cm", s["chicken_swerve_cm"])),
                      chicken_min_cm=float(bl.get("min_cm", s["chicken_min_cm"])),
                      chicken_rounds=int(bl.get("rounds", 1)))
            W = f["x_max"] - f["x_min"]
            # the ready spots leave room behind them for the straight run-in (its start must be turnable)
            s1["chicken_run_cm"] = min(s["chicken_run_cm"], W / 2.0 - room - CHICKEN_RUN_IN_CM)
            Hh = (f["y_max"] - f["y_min"]) / 2.0
            s1["chicken_aside_cm"] = min(s["chicken_aside_cm"], Hh - room)
            ch = chicken(m, a=a, b=b, s=s1, centre=_frac(m, *bl.get("at", [0.5, 0.5])))
            for n in who:
                for v in ch[n]:
                    if "goto" in v and v.get("label", "").endswith("ready"):
                        v["run_in_cm"] = CHICKEN_RUN_IN_CM
                advance(n, ch[n])
            notes.append("Chicken: swerve at %.0f cm, never under %.0f cm (run %.0f cm, aside %.0f cm)"
                         % (s1["chicken_swerve_cm"], s1["chicken_min_cm"], s1["chicken_run_cm"], s1["chicken_aside_cm"]))
        elif kind == "spread-out":
            pos = {n: _pose_after(n) for n in who}
            order = sorted(who, key=lambda n: pos[n][0] if long_x else pos[n][1])
            N = len(order)
            fy = float(bl.get("y", 0.5))
            for i, n in enumerate(order):
                fx = (i + 1) / (N + 1)
                x, y = _frac(m, fx, fy) if long_x else _frac(m, fy, fx)
                advance(n, [_g(x, y, float(bl.get("heading", 90.0)), label="spread out")])
        elif kind == "dance":
            for n in who:
                advance(n, [{"dance": float(bl.get("seconds", 0)), "label": "dance"}])
        else:
            raise ValueError("block %d: unknown block %r (blocks: %s)" % (k + 1, kind, ", ".join(BLOCKS)))
    out = copy.deepcopy(m)
    out["robots"] = [dict(known[n], start=starts[n], park=list(_pose_after(n))) for n in names]
    out["tasks"] = {n: {"visit": visits[n], "park": False} for n in names}
    out["mission"] = {"name": program.get("name", "mission"), "program": program, "settings": s,
                      "notes": notes, "estimate_s": {n: round(t, 1) for n, t in t_est.items()},
                      "planner": {"separation_cm": s["separation_cm"]}}
    return FM.normalize(out)


def planner_over(mm):
    """Planner settings a mission map asks for."""
    pm = (mm.get("mission") or {}).get("planner") or {}
    return {k: v for k, v in pm.items() if k in P.DEFAULT_SETTINGS}


def summary(mm, plan):
    """Plain numbers: finish time, closest pair overall and Left-Right during chicken, show timing against beats."""
    out = {"makespan_s": plan["summary"]["makespan_s"], "closest_cm": plan["summary"]["min_separation_cm"]}
    late, n_beats = [], 0
    t0 = float(plan.get("mission_t0_s", 0.0) or 0.0)
    for n, r in plan["robots"].items():
        for st in r["steps"]:
            if st.get("beat_s") is not None:
                n_beats += 1
                late.append(round(t0 + st["t_start"] - st["beat_s"], 2))
    out["beats"] = n_beats
    out["beats_late_over_0_5s"] = sum(1 for x in late if x > 0.5)
    out["beat_late_median_s"] = sorted(late)[len(late) // 2] if late else 0.0
    out["beat_late_worst_s"] = max(late) if late else 0.0
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("mode", choices=["monday", "mow", "chicken", "follow", "roam", "show", "instructions"])
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--robot", default="Center")
    ap.add_argument("--text", default="", help="instructions mode: e.g. 'go to A, wait 5 s, go to C, repeat 3x'")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="a mission setting (repeatable)")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(argv)
    m = FM.load(a.map)
    s = settings(dict(kv.split("=", 1) for kv in a.set))
    if a.mode == "monday":
        mm = monday(m, s, a.seed)
    else:
        mm = dict(m)
        start = {r["name"]: r["start"] for r in m["robots"]}
        if a.mode == "mow":
            v, notes = mow(m, s, robot_start=start.get(a.robot))
            mm["tasks"] = {a.robot: {"visit": v, "park": True}}
        elif a.mode == "chicken":
            mm["tasks"] = {k: {"visit": v, "park": True} for k, v in chicken(m, s=s).items()}
        elif a.mode == "follow":
            mm["tasks"] = {k: {"visit": v, "park": True} for k, v in follow(m, s=s).items()}
        elif a.mode == "roam":
            mm["tasks"] = {a.robot: {"visit": roam(m, a.robot, s, seed=a.seed, start=start.get(a.robot)), "park": True}}
        elif a.mode == "show":
            sh, end, figs = show(m, s=s)
            mm["tasks"] = {k: {"visit": v, "park": True} for k, v in sh.items()}
        else:
            mm["tasks"] = {a.robot: {"visit": instructions(a.text, m.get("spots") or {}), "park": True}}
        mm["mission"] = {"name": a.mode, "settings": s, "planner": {"separation_cm": s["separation_cm"]}}
    with open(a.out, "w") as f:
        json.dump(mm, f, indent=1)
    print("%s mission -> %s" % (a.mode, a.out))


if __name__ == "__main__":
    main()
