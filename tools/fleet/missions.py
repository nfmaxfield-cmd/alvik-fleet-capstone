#!/usr/bin/env python3
"""Run modes on the planner (school demo, docs/build-plans/mission-control.md item 6, 2026-10-27). Each mode turns
into ordinary planner tasks (map.json "tasks": goto / spot / wait_s / until_s items), so the planner keeps every pair
of robots apart, re-plans round a dropped brick, and run_plan.py drives it:

  instructions   "go to A · wait 5 s · go to C · repeat 3x" -> tasks (named spots)
  mow            one full pass of lanes (MOW_LANE_CM apart, MOW_INSET_CM in from every side, or the planner's least
                 room from an open side if that is more); lanes carry their line, so a lane end inside a new obstacle
                 slides along its lane and the route round the obstacle is planned (planner.py, "lane" items)
  chicken        two robots drive at each other, swerve (an angled jog) when their fronts are chicken_swerve_cm apart
                 and drive on past each other, bodies chicken_pass_gap_cm apart, to swapped ends; the plan's closest
                 approach is checked against chicken_min_cm
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
    "mow_inset_cm": 17.0,         # Nick (2026-10-28 21:27): lanes moved in from the edges (was 10, his brief, which the
                                  # planner's least room from an open side raised to 12.2 on a blank table). One value
                                  # sets both the lane line (the first and last lanes' distance from the short ends)
                                  # and the turn-around point (each lane end's distance from the long sides). Target:
                                  # the body's nearest corner at least 7 cm from any open edge, including the turn on
                                  # the spot at each lane end, where the 9.5 cm square's corners swing 6.72 cm from its
                                  # middle: 6.72 + 7 + 3 cm (the robots' real tracking, within 2-3 cm of the line 95 %
                                  # of the time, 28 Oct) = 16.7, rounded up. 13 cm would leave 6.3 cm at the turn even
                                  # with perfect tracking. Chosen, not derived from a test: DOE note: a board DOE of
                                  # lane margin (e.g. 14 / 17 / 20 cm) against the camera's closest body-to-edge at the
                                  # turn-arounds would set it (and show whether the lane ends can sit nearer than the
                                  # lane line: along a lane the stop error is smaller than across it in the simulator)
    "edge_room_cm": 17.0,         # Nick (2026-10-28 21:27, "everywhere in the mission"): the same target as
                                  # mow_inset_cm for the spots the blocks pick themselves: Chicken's run-in starts (the
                                  # robot turns on the spot there to face along the Chicken line) and the ends its pass
                                  # drives on to. Before: the planner's least room + 1 cm (12.2 cm), which left the body
                                  # 5.5 cm from the short ends at the turn by plan. Shortens Chicken's run on 61 x 122
                                  # from 36.8 to 32 cm (swerve 15, pass gap 10 unchanged). Chosen from 2-3 cm real p95
                                  # plus the turn-around swing (6.72 + 7 + 3, rounded up): DOE note: a board DOE of
                                  # lane margin vs closest body-to-edge (as mow_inset_cm) would set both together
    "chicken_swerve_cm": 15.0,    # Nick (2026-10-27 00:08): swerve when the fronts are 15 cm apart
    "chicken_min_cm": 8.0,        # Nick: never under 8 cm (two half-widths + ~3 cm camera error, rounded up; DOE on
                                  # the board: closest approach measured by the camera over 10 chickens)
    "chicken_run_cm": 45.0,       # chosen: each robot starts this far from the middle (a run of ~25 cm at cruise
                                  # speed before the swerve: long enough to look like a dare). No DOE (looks)
    "chicken_pass_gap_cm": 10.0,  # Nick (2026-10-28 03:5x): the bodies pass about 10 cm apart side to side; each robot
                                  # sidesteps (pass gap + body width) / 2 = 9.75 cm, each to its own left, so they pass
                                  # on opposite sides. DOE note: worth one on the board (8 / 10 / 12 cm against the
                                  # camera's measured gap and how close it looks to the audience), but under min_cm + 1
                                  # the planner makes one robot wait for the other to pass (no longer a game of Chicken)
    "chicken_jog_deg": 70.0,      # chosen (Claude, 2026-10-28): the swerve is an angled jog: turn this far towards its
                                  # own side on the spot, drive the diagonal (10.4 cm) to the pass line, turn back,
                                  # drive on. With the fronts 15 cm apart the planner's 9 cm (8 cm + 1 cm between its
                                  # samples, steps judged against the other robot's same and neighbouring steps) holds
                                  # only from about 60 deg: 45 deg comes to 5.9 cm, 55 to 8.4, 60 to 9.1, 65 and 70 to
                                  # 10.0 (the pass gap itself; chicken_shape). 70 rather than 65: 65 falls to 9.5 cm if
                                  # the first pass piece grows by 1 cm, 70 keeps 10. A true arc (planner/firmware arc
                                  # move) does not fit: an S-curve came to 0-5 cm, a jog with an arc back to 5-8 cm,
                                  # and arcs tighter than about 9 cm radius ask the outer wheel for more than the
                                  # motor's 70 rpm. A steeper jog is used by itself when min_cm needs it. DOE note: not
                                  # worth one for looks (the geometry sets it); the lever for a shallower, smoother
                                  # swerve is swerve_cm: at 20 cm 50 deg clears 9.4 cm, at 25 cm 45 deg clears 10 cm
                                  # (a DOE on swerve_cm 15 / 20 / 25 would be a looks judgement by Nick, not a measure)
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
    # Mirroring (Nick, 2026-10-29 20:06-20:08): Left and Right mirror each other across the table's middle line (x =
    # the centre) on every mirrored leg (mowing, the walk to the meet, spreading out); run_plan's mirror sync re-times
    # them when they drift out of step (the leader slows, the late one speeds up, within mirror_min / max_cms; if speed
    # cannot close the gap the leader waits at its next point). The circle and Chicken stay synced moments (they are
    # point-symmetric, not mirrored: each robot circles / swerves the same way round)
    "mirror_cruise_cms": 8.0,     # chosen: the mirrored legs' planned speed. It must sit inside Nick's 5-10 cm/s band
                                  # (accuracy worsens above 10: p95 about 3.5 cm at 13) with room both ways: at 10 (the
                                  # planner's cruise) a late robot could not speed up at all. 8 gives the late one +25 %
                                  # and the leader -37 %. Costs about 6-9 s on Monday's mowing. DOE note: worth one in
                                  # the simulator then the board: 7 / 8 / 9 cm/s against the lane-end mirror error after
                                  # a 3 s pause and the mission's length
    "mirror_min_cms": 5.0,        # Nick (2026-10-29 20:06): the slowest a re-timed leg is sent at. No DOE: his band
    "mirror_max_cms": 10.0,       # Nick: the fastest (accuracy worsens above 10 cm/s). No DOE: his band (a board DOE of
                                  # 10 / 11.5 / 13 cm/s against the line check p95 would show whether 11 is still fine)
    "mirror_tol_s": 0.5,          # Nick (agreed 2026-10-29): the robots reach matching points within 0.5 s of each other
    "mirror_tol_cm": 3.0,         # ... and within 3 cm of each other's mirrored position (the report's target)
    "mirror_wait_max_s": 8.0,     # chosen: the longest the leader waits at one point for the late robot when speed alone
                                  # cannot close the gap; past it the leader carries on (mirroring is given up until
                                  # the next synced moment, said in the event log). Covers Monday's injected cases (3 s
                                  # pause, 5 s late start) with room. DOE note: not worth one (it only bounds a wait);
                                  # the simulator's late-start runs show how long the waits are
}

ROLE_OF = {"Left": "A", "Center": "B", "Right": "C"}


RETIRED_SETTINGS = {"chicken_aside_cm"}   # 2026-10-28: Chicken passes now (chicken_pass_gap_cm); an old saved value
                                          # (Mission Control's settings.json) is skipped, not refused


SUPERSEDED_DEFAULTS = {"mow_inset_cm": (10.0,)}   # 2026-10-28 21:27: lanes moved in (10 -> 17). Mission Control's
                                                   # "Save mission settings" writes every value on the screen, so a
                                                   # settings.json saved before today holds the old default 10; that
                                                   # value counts as not set (the new default applies). Any other
                                                   # value someone typed in is kept


def settings(over=None):
    s = dict(SETTINGS)
    for k, v in (over or {}).items():
        if k in RETIRED_SETTINGS:
            continue
        if k in SUPERSEDED_DEFAULTS and float(v) in SUPERSEDED_DEFAULTS[k]:
            continue
        if k not in SETTINGS:
            raise ValueError("unknown mission setting %r" % k)
        s[k] = type(SETTINGS[k])(v)
    gap_lock_settings(s)
    return s


# ---------------------------------------------------------------------------- the 8 cm lock (Nick, 2026-10-29 07:3x)
# No 2-robot mission runs with a robot gap under the 8 cm safety net (engine_checks.MIN_GAP_CM) or the mission's own
# chicken_min_cm, from any screen or saved file: Mission Control's "Save planner settings" used to write the planner's
# own default 5 cm over the mission's 8. A lower value anywhere (mission settings, planner settings, a saved
# settings.json, mission_run --setting, run_plan --min-gap-cm) is raised, with a message saying so. The one way under it
# is the simulator's safety-net DOE (mission_run --dry --below-net-doe, run_plan --dry-run --below-net-doe).
GAP_KEYS_MISSION = ("separation_cm", "chicken_min_cm")
_BELOW_NET_DOE = [False]


def below_net_doe(on=True):
    """The simulator's safety-net DOE only (5 / 6.5 / 8 cm): lets a gap under the net through in this process."""
    _BELOW_NET_DOE[0] = bool(on)


def gap_floor(s=None):
    """The least robot gap any 2-robot mission may plan or run with: the net, or chicken_min_cm if that is more."""
    import engine_checks as EC
    cm = float((s or {}).get("chicken_min_cm", SETTINGS["chicken_min_cm"]))
    return max(EC.MIN_GAP_CM, cm)


def gap_lock_settings(s, where="mission settings"):
    """Raise separation_cm / chicken_min_cm in a mission settings dict to the net, in place; the messages."""
    if _BELOW_NET_DOE[0]:
        return []
    import engine_checks as EC
    out = []
    for k in GAP_KEYS_MISSION:
        if k in s and float(s[k]) < EC.MIN_GAP_CM:
            out.append("%s: %s %.1f cm raised to %.1f cm (no 2-robot mission runs under the %.0f cm safety net)"
                       % (where, k, float(s[k]), EC.MIN_GAP_CM, EC.MIN_GAP_CM))
            s[k] = type(SETTINGS[k])(EC.MIN_GAP_CM)
    if "separation_cm" in s and float(s["separation_cm"]) < float(s.get("chicken_min_cm", 0.0)):
        out.append("%s: separation_cm %.1f cm raised to chicken_min_cm, %.1f cm" % (where, float(s["separation_cm"]),
                                                                                   float(s["chicken_min_cm"])))
        s["separation_cm"] = float(s["chicken_min_cm"])
    return out


def gap_lock_planner(over, floor=None, n_robots=2, where="planner settings"):
    """Raise a planner settings dict's separation_cm to the floor (2 or more robots), in place; the messages. A
    missing separation_cm is set too (the planner's own default is 5 cm)."""
    if n_robots < 2 or _BELOW_NET_DOE[0]:
        return []
    floor = gap_floor() if floor is None else max(floor, gap_floor())
    v = over.get("separation_cm")
    if v is None:
        over["separation_cm"] = floor
        return []
    if float(v) < floor - 1e-9:
        over["separation_cm"] = floor
        return ["%s: separation_cm %.1f cm raised to %.1f cm (no 2-robot mission runs under the %.0f cm safety net)"
                % (where, float(v), floor, floor)]
    return []


def mission_robots(mm):
    """How many robots a mission map moves (robots with tasks)."""
    t = mm.get("tasks") or {}
    return sum(1 for n, v in t.items() if (v or {}).get("visit")) or len(mm.get("robots") or [])


# ---------------------------------------------------------------------------- estimates and helpers

_PL = {}


def plan_s(m, visits, start):
    """Time (s) the planner's own steps take for these visits from start (x, y, heading deg), without waits for
    other robots (exact for the steps; est_s is the quick rough figure)."""
    # the cache holds the map itself: keyed on id(m) alone, a later build's map could get a freed map's id and its
    # times came from the earlier table's planner (found 2026-10-28 21:45: Monday on 76 x 137 planned or not
    # depending on which table was built before it in the same process)
    if _PL.get("m") is not m:
        _PL.clear()
        _PL.update(m=m, pl=P.Planner(m))
    pl = _PL["pl"]
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

CHICKEN_JOG_MAX_DEG = 90.0   # the steepest jog chicken_shape tries (90: a straight sidestep). No DOE: geometry
CHICKEN_PULL_OUT_CM = 3.0    # chosen: after turning back onto its pass line each robot drives this short piece before
                             # the rest of the pass. The planner keeps apart steps that only touch in time, and the other
                             # robot's turn back (a full turn circle on its own pass line) against a first piece of
                             # 12-15 cm came to 8.0 cm; 2-4 cm keep 10 cm at 70 deg. Costs one 0.2 s settle. No DOE:
                             # geometry (planner only)
CHICKEN_CLEAR_MARGIN_CM = 0.5  # chosen: the jog must clear the planner's floor by this much (goto points are rounded to
                               # 0.01 cm and steps to 0.1 s ticks). No DOE: numerical


def chicken_shape(m, s=None):
    """The swerve's shape for these settings: each robot's sidestep (its centre line moves this far to its own side,
    so the bodies pass chicken_pass_gap_cm apart), the jog angle, and the closest two bodies come during the swerve
    when both jog together (each move against the other robot's move at the same time, as the planner judges them:
    whole swept areas). The jog starts at chicken_jog_deg and is made steeper, 5 deg at a time, until that clears
    the floor (the larger of chicken_min_cm and the planner's separation_cm, plus the planner's 1 cm between samples);
    below that the planner would make one robot wait and they would not swerve together. Returns a dict with
    sidestep_cm, jog_deg, swerve_clear_cm, pass_clear_cm (the bodies' gap side by side), floor_cm and notes."""
    import numpy as np
    import geometry as G
    s = settings(s)
    mm = FM.normalize(m)
    body = FM.body_of(mm)
    pl = P.Planner(mm)
    floor = max(s["chicken_min_cm"], s["separation_cm"]) + P._slack(body, pl.s)
    side = (s["chicken_pass_gap_cm"] + body.width) / 2.0
    meet = s["chicken_swerve_cm"] / 2.0 + body.front

    def mirror(Q):
        R_ = Q.copy()
        R_[:, :2] *= -1.0
        R_[:, 2] += math.pi
        return R_

    f = np.linspace(0.0, 1.0, 25)

    def line(p, q, h):
        return np.column_stack([p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]), np.full(len(f), h)])

    def clear(th):
        # one robot's swerve, as the planner cuts it into steps (centre of the chicken line at 0, 0, driving +x); the
        # other robot does the same mirrored. The planner keeps apart any two steps whose times overlap or touch, so
        # each step is judged against the other robot's same step and the ones either side of it
        S = (-meet, 0.0)
        E = (-meet + (side / math.tan(th) if th < math.radians(89.9) else 0.0), side)
        steps = [line((S[0] - pl.s.chunk_cm, 0.0), S, 0.0),                           # the run at each other
                 np.column_stack([np.full(len(f), S[0]), np.full(len(f), S[1]), f * th]),  # turn out
                 line(S, E, th),                                                         # the diagonal
                 np.column_stack([np.full(len(f), E[0]), np.full(len(f), E[1]), (1 - f) * th]),  # turn back
                 line(E, (E[0] + CHICKEN_PULL_OUT_CM, side), 0.0),                       # the short first piece
                 line((E[0] + CHICKEN_PULL_OUT_CM, side), (E[0] + CHICKEN_PULL_OUT_CM + pl.s.chunk_cm, side), 0.0)]
        out = math.inf
        for i, A in enumerate(steps):
            for j in range(max(0, i - 1), min(len(steps), i + 2)):
                Bm = mirror(steps[j])
                out = min(out, float(np.min(G.body_gaps(body, np.repeat(A, len(Bm), 0), np.tile(Bm, (len(A), 1))))))
        return out
    notes = []
    th = s["chicken_jog_deg"]
    c = clear(math.radians(th))
    while c < floor + CHICKEN_CLEAR_MARGIN_CM and th < CHICKEN_JOG_MAX_DEG:
        th = min(CHICKEN_JOG_MAX_DEG, th + 5.0)
        c = clear(math.radians(th))
    if th != s["chicken_jog_deg"]:
        notes.append("Chicken: the jog is %.0f deg, not %.0f: at %.0f deg the two robots would come under %.1f cm "
                     "swerving together" % (th, s["chicken_jog_deg"], s["chicken_jog_deg"],
                                            floor + CHICKEN_CLEAR_MARGIN_CM))
    pass_clear = 2 * side - body.width
    if c < floor or pass_clear < floor - 1e-6:
        notes.append("Chicken: with swerve %.0f cm, pass gap %.0f cm and never under %.0f cm, the robots cannot swerve "
                     "and pass at the same time; the planner makes one wait for the other (raise the pass gap or the "
                     "swerve distance)" % (s["chicken_swerve_cm"], s["chicken_pass_gap_cm"], s["chicken_min_cm"]))
    return {"sidestep_cm": side, "jog_deg": th, "swerve_clear_cm": round(c, 2), "pass_clear_cm": round(pass_clear, 2),
            "floor_cm": round(floor, 2), "notes": notes}


CHICKEN_OLD_ASIDE_CM = 20.0   # the old routine's swerve (passing=False): drive this far sideways and stop, as before
                              # 2026-10-28 (it was chicken_aside_cm). No DOE (geometry)


def chicken(m, a="Left", b="Right", s=None, t0=0.0, centre=None, swap=False, passing=True):
    """Two robots drive at each other along a line through `centre` (the table's middle by default) and, when their
    fronts are chicken_swerve_cm apart, each swerves to its own left (an angled jog, chicken_shape) onto its own
    pass line, so they pass on opposite sides with the bodies chicken_pass_gap_cm apart, then drives on straight to
    about where the other one started (they swap ends). The next round starts from the swapped ends (swap=True: the
    first round does too, a starting at the high-x end). Returns {robot: visits}; each round's items are labelled
    "chicken k ready" (ready spot, then a short wait), "chicken k swerve" (the run at each other, ending at the swerve
    point), "chicken k sidestep" (the jog: a turn on the spot and the diagonal) and "chicken k pass" (turn back, drive
    on past). passing=False: the swerve before 2026-10-28 (the old 3-robot routine, missions.monday): each goes
    CHICKEN_OLD_ASIDE_CM sideways and stops, and the next round starts from the same ends."""
    s = settings(s)
    cx, cy = centre or _centre(m)
    body = FM.body_of(FM.normalize(m))
    meet = s["chicken_swerve_cm"] / 2.0 + body.front      # each centre this far from the middle at the swerve
    R = s["chicken_run_cm"]
    if not passing:
        return _chicken_aside(m, a, b, s, cx, cy, meet, R)
    sh = chicken_shape(m, s)
    side, th = sh["sidestep_cm"], math.radians(sh["jog_deg"])
    fwd = side / math.tan(th) if th < math.radians(89.9) else 0.0
    # the pass lines sit either side of the chicken line; if another robot stands near one (the old 3-robot routine),
    # or a line has no room, the chicken line moves across (2 cm at a time, up to the sidestep) until both fit
    others = [r["start"][:2] for r in FM.normalize(m)["robots"] if r["name"] not in (a, b)]
    pl = P.Planner(m)

    def ok(y):
        pts = [(cx + u * R, y + u * side) for u in (-1, 1)] + [(cx - u * (meet - fwd), y + u * side) for u in (-1, 1)]
        return all(pl.grid.turnable([x, yy]) and all(math.dist((x, yy), o) > 2 * 6.8 + s["chicken_min_cm"]
                                                        for o in others) for x, yy in pts)
    for dy in [0.0] + [k * 2.0 * sg for k in range(1, int(side // 2) + 2) for sg in (1, -1)]:
        if ok(cy + dy):
            cy += dy
            break
    out = {a: [], b: []}
    for k in range(int(s["chicken_rounds"])):
        lab = "chicken %d" % (k + 1)
        for n, u0 in ((a, 1.0), (b, -1.0)):
            u = -u0 if (k % 2 == 1) != swap else u0      # +1: drives towards +x (starts at the low-x end)
            h = 0.0 if u > 0 else 180.0
            y_pass = cy + u * side                       # its own left: +y facing +x, -y facing -x
            out[n] += [_g(cx - u * R, cy, h, label=lab + " ready"), {"wait_s": 0.5, "label": lab + " ready"},
                       _g(cx - u * meet, cy, label=lab + " swerve"),
                       _g(cx - u * (meet - fwd), y_pass, label=lab + " sidestep"),
                       _g(cx - u * (meet - fwd - CHICKEN_PULL_OUT_CM), y_pass, h, label=lab + " pass"),
                       _g(cx + u * R, y_pass, h, label=lab + " pass")]
    return out


def chicken_mission(m, a="Left", b="Right", s=None):
    """Chicken on its own (Mission Control's Chicken mode, `missions.py chicken`): chicken()'s rounds with the same
    wait and sync mark at the ready spots as Monday's block, so both robots set off together (passing needs that:
    plan it with plan_synced). Returns {robot: visits}."""
    s = settings(s)
    ch = chicken(m, a, b, s=s)
    start = {r["name"]: r["start"] for r in FM.normalize(m)["robots"]}
    rounds = {n: {} for n in (a, b)}
    for n in (a, b):
        for v in ch[n]:
            rounds[n].setdefault(int(re.match(r"chicken (\d+)", v["label"]).group(1)), []).append(v)
    out = {a: [], b: []}
    t_est = {a: 0.0, b: 0.0}
    pose = {n: tuple(start[n][:3]) for n in (a, b)}
    for rd in sorted(rounds[a]):
        for n in (a, b):
            rdy = rounds[n][rd][0]
            try:
                t_est[n] += plan_s(m, [rdy], pose[n])
            except P.PlanError:
                t_est[n] += est_s([rdy["goto"][:2]], pose[n])
            pose[n] = tuple(rdy["goto"][:3])
        t_ready = max(t_est.values()) + MEET_SLACK_S
        for n in (a, b):
            rest = rounds[n][rd][1:]
            lab = rounds[n][rd][0]["label"]
            out[n] += [rounds[n][rd][0], {"until_s": round(t_ready, 1), "label": lab},
                       {"sync": "chicken_%d" % rd, "label": lab}] + rest
            moving = [v for v in rest if "goto" in v]
            try:
                t_est[n] = t_ready + plan_s(m, moving, pose[n])
            except P.PlanError:
                t_est[n] = t_ready + est_s([v["goto"][:2] for v in moving], pose[n])
            last = moving[-1]["goto"]
            pose[n] = (last[0], last[1], last[2] if len(last) > 2 else pose[n][2])
    return out


def _chicken_aside(m, a, b, s, cx, cy, meet, R):
    """The swerve before 2026-10-28, kept for the old 3-robot routine: each swerves to its own left, unless a side has
    no room or another robot stands there (then both to the side with room), drives CHICKEN_OLD_ASIDE_CM sideways and
    stops."""
    others = [r["start"][:2] for r in FM.normalize(m)["robots"] if r["name"] not in (a, b)]
    pl = P.Planner(m)
    asd = CHICKEN_OLD_ASIDE_CM

    def ok(x, y):
        return pl.grid.turnable([x, y]) and all(math.dist((x, y), o) > 2 * 6.8 + s["chicken_min_cm"] for o in others)
    sa, sb = 1.0, -1.0
    if not (ok(cx - meet, cy + sa * asd) and ok(cx + meet, cy + sb * asd)):
        sa = sb = 1.0 if ok(cx - meet, cy + asd) and ok(cx + meet, cy + asd) else -1.0
    out = {a: [], b: []}
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
        # the old routine keeps the old swerve (aside and stop): passing needs both robots to set off together, and
        # this routine has no sync waits (Center mows with right of way); Monday's blocks pass (missions.build)
        ch = chicken(m, s=s1, centre=c, passing=False)
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


MIRROR_LANE_STRETCH_CM = 1.0   # mirrored lanes: each half's lanes are spread evenly, at most mow_lane_cm + this apart
                               # (Monday: 10.1-12.3 cm; 8 lanes on 61 x 122 as before). No DOE: looks (coverage)


MIRROR_TURN_R_CM = 6.72   # the 9.5 cm square's corner radius about its middle: two robots turning on the spot at the
                          # innermost lanes at the same moment (mirrored, they do) swing their corners this far out


def mirror_centre_gap(m):
    """Mirrored lanes: the innermost pair (one each side of the middle line) this far apart, centre to centre, so both
    robots mow them at once and even while both turn on the spot at their lane ends the bodies stay STEP_AWAY_GAP_CM
    apart by plan (the step-away's 13 cm, about 11 measured). With lanes 12 cm apart the first inner lanes were 2-3 cm
    apart and one robot had to wait (the step-away), which no mirror can have. A first version kept 13 cm only with
    the robots standing square: turning together their corners came 9.0-9.4 cm apart in the simulator (29 Oct).
    The strip between them is where the robots circle."""
    body = float((m.get("robot") or {}).get("width_cm", 9.5))
    r = max(MIRROR_TURN_R_CM, body / math.sqrt(2.0))
    return 2.0 * r + STEP_AWAY_GAP_CM


def lanes_x(m, s, mirror=False):
    """Lane positions across the table's length (lanes run along the short side, x = const for a long-x table), as
    mow(along="short") makes them, with the planner's room from open sides. mirror=True (Nick, 2026-10-29: Left and Right
    mirror each other): the lanes are symmetric about the middle line: each half's lanes spread evenly from the first
    lane (inset from the end) to the innermost one (mirror_centre_gap / 2 from the middle)."""
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
    a0, a1 = ((f["y_min"] + inset, f["y_max"] - inset) if long_x else (f["x_min"] + inset, f["x_max"] - inset))
    if mirror:
        c = (lo + hi) / 2.0
        inner = c - mirror_centre_gap(m) / 2.0
        span = inner - lo
        if span <= 0:
            half = [lo] if lo < c else []
        else:
            n = int(math.ceil(span / (s["mow_lane_cm"] + MIRROR_LANE_STRETCH_CM) - 1e-9)) + 1
            half = [lo + span * k / (n - 1) for k in range(n)]
        out = [round(v, 2) for v in half] + [round(2 * c - v, 2) for v in reversed(half)]
        return out, (a0, a1), long_x, inset
    out, v = [], lo
    while v <= hi + 1e-6:
        out.append(round(v, 2))
        v += s["mow_lane_cm"]
    if hi - out[-1] > s["mow_lane_cm"] / 2:
        out.append(round(hi, 2))
    return out, (a0, a1), long_x, inset


def mow_region(m, s, fx0, fx1, direction, start_xy, mirror=False):
    """The lanes whose position lies in [fx0, fx1) of the table's length (fx1 = 1 includes the last), visited in
    increasing (direction +1) or decreasing (-1) position, each lane driven end to end, alternating, starting from the
    end nearest start_xy: stop-and-turn (drive the lane, 90 deg, the lane gap over, 90 deg, back). Returns visits.
    mirror=True: the symmetric lanes (lanes_x mirror), and a region in one half of the table takes exactly the mirror
    images of the lanes its mirrored region takes in the other half (a lane on a region's edge cannot fall to one side
    only)."""
    lanes, (a0, a1), long_x, _ = lanes_x(m, s, mirror=mirror)
    f = m["floor"]
    L0, L1 = (f["x_min"], f["x_max"]) if long_x else (f["y_min"], f["y_max"])
    p0, p1 = L0 + fx0 * (L1 - L0), L0 + fx1 * (L1 - L0)
    mine = [v for v in lanes if p0 - 1e-6 <= v < p1 - 1e-6 or (fx1 >= 1.0 and v >= p0 - 1e-6)]
    if mirror and (fx1 <= 0.5 + 1e-9 or fx0 >= 0.5 - 1e-9):
        c = (L0 + L1) / 2.0
        hi_side = fx0 >= 0.5 - 1e-9
        q0, q1 = (1.0 - fx1, 1.0 - fx0) if hi_side else (fx0, fx1)      # the region as seen in the low half
        P0, P1 = L0 + q0 * (L1 - L0), L0 + q1 * (L1 - L0)
        low = [v for v in lanes if v < c and (P0 - 1e-6 <= v < P1 - 1e-6 or (q1 >= 0.5 - 1e-9 and v >= P0 - 1e-6))]
        mine = [round(2 * c - v, 2) for v in low] if hi_side else low
    mine = sorted(mine, reverse=direction < 0)
    if not mine:
        return []
    out = []
    cur = start_xy
    for k, c in enumerate(mine):
        def pt(e):
            return (c, e) if long_x else (e, c)
        # rounded: two mirrored robots equally far from both ends (Monday's circle exits sit on the middle of the
        # table's width) must break the tie the same way (float noise sent one up and the other down, 29 Oct)
        ends = (a0, a1) if round(math.dist(cur, pt(a0)), 3) <= round(math.dist(cur, pt(a1)), 3) else (a1, a0)
        line = [*pt(ends[0]), *pt(ends[1])]
        for j, e in enumerate(ends):
            x, y = pt(e)
            k_lane = min(range(len(lanes)), key=lambda q: abs(lanes[q] - c))
            out.append(_g(x, y, label="mow lane %d %s" % (k_lane + 1, "start" if j == 0 else "end"), lane=line,
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
    3. At the end of their mowing they turn towards each other and play Chicken (swerve at 15 cm, never under 8 cm):
       each swerves a little to its own side and they drive on past each other, bodies about 10 cm apart, to swapped
       ends (Nick, 2026-10-28 03:52).
    4. They spread out, equally spaced on the table, and dance until switched off.
    Claude's reading of "mid-mow" (an assumption, flagged in the build report): each robot mows the outer quarter at its
    end, they meet and circle, and each then mows the inner quarter on the other side, going on in its own direction,
    so every lane is mowed once and they end facing each other across the middle for Chicken."""
    a, b = robots
    return {"name": "Monday, 2 robots", "robots": [a, b], "mirror": [a, b],
            "starts": {a: {"at": [0.0, 0.0], "face": "+y"}, b: {"at": [1.0, 0.0], "face": "+y"}},
            "blocks": [
                {"block": "mow-region", "robot": a, "from": 0.0, "to": 0.25, "direction": "+"},
                {"block": "mow-region", "robot": b, "from": 0.75, "to": 1.0, "direction": "-"},
                {"block": "meet", "robots": [a, b], "at": [0.5, 0.5], "r_cm": CIRCLE_R_CM},
                {"block": "circle", "robots": [a, b], "turns": 1.5, "way": "anticlockwise"},
                {"block": "mow-region", "robot": a, "from": 0.5, "to": 0.75, "direction": "+"},
                {"block": "mow-region", "robot": b, "from": 0.25, "to": 0.5, "direction": "-"},
                {"block": "chicken", "robots": [a, b], "swerve_cm": 15.0, "min_cm": 8.0, "pass_gap_cm": 10.0,
                 "rounds": 1},
                {"block": "spread-out", "robots": [a, b]},
                {"block": "dance", "robots": [a, b], "seconds": 0},
            ]}


STEP_AWAY_GAP_CM = 13.0  # chosen (Nick, 2026-10-29: "about 11 cm or more"): after the circle, the robot that waits for
                         # the other's first lane stands at least this far (body to body, by plan) from that lane. The
                         # simulator drifted 1.1-1.6 cm under the plan there (7.9 cm against 9.0-9.5, 28 Oct), so 13
                         # keeps about 11. DOE note: worth one on the board: 11 / 13 / 15 cm against the camera's
                         # closest approach in that stretch (the extra step's time is free: it is inside the wait)
STEP_AWAY_MIN_CM = 3.0   # chosen: a step shorter than this is lengthened to it (a drive of 1-2 cm is mostly the
                         # robot's stop error). No DOE: below the 2-3 cm real tracking error it would not show


def _step_away(m, s, rest, who, exits, long_x, mirror=False):
    """After the circle (Monday: the robots leave it on swapped sides and each goes on mowing): if their first lanes
    are too close for both to mow at once, the planner keeps one standing where it left the circle while the other
    mows its first lane. That spot was 9.0-9.5 cm (body to body) from the lane by plan on 61 x 122 and 76 x 137, and
    7.9 cm in 2 of 6 simulator runs (28 Oct 21:27). The robot standing in the way (its circle exit nearest the other's
    first lane) steps away from that lane first, to STEP_AWAY_GAP_CM, and waits there. Returns dict(robot, goto, note)
    or None (no conflict, or already far enough)."""
    if len(who) != 2:
        return None
    first, nlanes = {}, {}
    for n in who:
        bl = next((b for b in rest if b.get("block") == "mow-region" and b.get("robot") == n), None)
        if bl is None:
            return None
        items = mow_region(m, s, float(bl.get("from", 0.0)), float(bl.get("to", 1.0)),
                           -1 if str(bl.get("direction", "+")).startswith("-") else 1, exits[n], mirror=mirror)
        if not items:
            return None
        first[n] = items[0]["goto"][0] if long_x else items[0]["goto"][1]
        nlanes[n] = len(items) // 2
    body = float((m.get("robot") or {}).get("width_cm", 9.5))
    sep = max(s["separation_cm"], s["chicken_min_cm"])
    a, b = who
    if abs(first[a] - first[b]) >= body + sep + 1.0:
        return None                                  # both can mow at once: nobody waits
    ax = 0 if long_x else 1

    def off(n):            # n's exit from the other's first lane, along the table's length
        o = [x for x in who if x != n][0]
        return exits[n][ax] - first[o]
    w = min(who, key=lambda n: abs(off(n)))
    need = STEP_AWAY_GAP_CM + body - abs(off(w))
    if need <= 0:
        return None
    step = max(need, STEP_AWAY_MIN_CM)
    sign = 1.0 if off(w) >= 0 else -1.0
    f = m["floor"]
    lo, hi = (f["x_min"], f["x_max"]) if long_x else (f["y_min"], f["y_max"])
    v = min(max(exits[w][ax] + sign * step, lo + s["mow_inset_cm"]), hi - s["mow_inset_cm"])
    xy = [v, exits[w][1]] if long_x else [exits[w][0], v]
    gap = abs(v - first[[x for x in who if x != w][0]]) - body
    return {"robot": w, "goto": _g(round(xy[0], 2), round(xy[1], 2), label="step away"),
            "note": "after the circle %s steps %.1f cm away from %s's first lane and waits there (bodies %.1f cm apart "
                    "by plan, was %.1f)" % (w, abs(v - exits[w][ax]), [x for x in who if x != w][0], gap,
                                             abs(off(w)) - body)}


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
    # the least room from a robot's centre to an open side at the spots the blocks pick themselves (Chicken's run-in
    # starts and the ends its pass drives on to): edge_room_cm, or the planner's least room + 1 cm if that is more
    room = max(least_open_room(m) + 1.0, s["edge_room_cm"])
    visits = {n: [] for n in names}
    starts = {}
    notes = []
    # mirroring (Nick, 2026-10-29 20:06): the program's "mirror": [a, b] makes those two mirror each other across the
    # table's middle line: symmetric lanes, and every point they drive to on a mirrored block (mow-region, meet,
    # spread-out) carries a key it shares with its mirror image (mirror_key), at mirror_cruise_cms
    mir = program.get("mirror")
    if mir is None and len(names) == 2 and program.get("blocks") == monday_program(tuple(names))["blocks"]:
        # Monday's default saved before mirroring (Mission Control's program.json from before 2026-10-29 20:06 has no
        # "mirror"): the unedited default mirrors too. An edited program mirrors only if it says so
        mir = list(names)
    mpair = [n for n in (mir or []) if n in names]
    mpair = mpair if len(mpair) == 2 else []
    lanes, (a0, a1), long_x, inset = lanes_x(m, s, mirror=bool(mpair))
    cmid = ((f["x_min"] + f["x_max"]) / 2.0) if long_x else ((f["y_min"] + f["y_max"]) / 2.0)
    mcount = {n: {} for n in names}

    def mtag(n, items):
        """the mirror key and speed on robot n's goto items (n one of the mirror pair)"""
        if n not in mpair:
            return items
        for v in items:
            if "goto" not in v:
                continue
            x, y = v["goto"][0], v["goto"][1]
            cx_, cy_ = (min(x, 2 * cmid - x), y) if long_x else (x, min(y, 2 * cmid - y))
            base = "%.1f,%.1f" % (cx_, cy_)
            k_ = mcount[n][base] = mcount[n].get(base, 0) + 1
            v["mirror"] = "%s#%d" % (base, k_)
            v["cruise_cms"] = s["mirror_cruise_cms"]
        return items
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
                               -1 if str(bl.get("direction", "+")).startswith("-") else 1, _pose_after(n)[:2],
                               mirror=n in mpair)
            mtag(n, items)
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
                advance(n, mtag(n, [_g(ex, ey, hd, label="meet", run_in_cm=MEET_RUN_IN_CM)]))
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
            sa = _step_away(m, s, program["blocks"][k + 1:], who, {n: _pose_after(n)[:2] for n in who}, long_x,
                            mirror=bool(mpair) and set(who) == set(mpair))
            if sa:
                advance(sa["robot"], [sa["goto"]])
                notes.append(sa["note"])
        elif kind == "chicken":
            if len(who) != 2:
                raise ValueError("block %d: chicken is for 2 robots" % (k + 1))
            pos = {n: _pose_after(n) for n in who}
            a, b = sorted(who, key=lambda n: pos[n][0])          # a comes from the low-x side
            s1 = dict(s, chicken_swerve_cm=float(bl.get("swerve_cm", s["chicken_swerve_cm"])),
                      chicken_min_cm=float(bl.get("min_cm", s["chicken_min_cm"])),
                      chicken_pass_gap_cm=float(bl.get("pass_gap_cm", s["chicken_pass_gap_cm"])),
                      chicken_jog_deg=float(bl.get("jog_deg", s["chicken_jog_deg"])),
                      chicken_rounds=int(bl.get("rounds", 1)))
            W = f["x_max"] - f["x_min"]
            # the ready spots leave room behind them for the straight run-in (its start must be turnable); the pass
            # ends at about the other robot's ready spot, so the same room holds there
            s1["chicken_run_cm"] = min(s["chicken_run_cm"], W / 2.0 - room - CHICKEN_RUN_IN_CM)
            shape = chicken_shape(m, s1)
            ctr = _frac(m, *bl.get("at", [0.5, 0.5]))
            if min(ctr[1] - f["y_min"], f["y_max"] - ctr[1]) - room < shape["sidestep_cm"] - 1e-6:
                raise ValueError("block %d: Chicken's pass lines (%.1f cm either side of its line, for a %.0f cm pass "
                                 "gap) do not fit the table's width here" % (k + 1, shape["sidestep_cm"],
                                                                            s1["chicken_pass_gap_cm"]))
            # only this mission's robots: a robot the program leaves out (Monday: Center) is not on the table, but its
            # start spot in the map moved the chicken line 6 cm off the middle (76 x 137, 2026-10-28 21:55), and with
            # the lanes 17 cm in that line found no plan
            ch = chicken(dict(m, robots=[r for r in m["robots"] if r["name"] in names]), a=a, b=b, s=s1, centre=ctr)
            for n in who:
                for v in ch[n]:
                    if "goto" in v and v.get("label", "").endswith("ready"):
                        v["run_in_cm"] = CHICKEN_RUN_IN_CM
            # each round: both reach their ready spots, wait for each other there (as the meet block does), then run
            # at each other together. Without this each robot played its half when it got there (Nick 2026-10-28:
            # Chicken could not be seen; on 61 x 122 Left swerved at 82 s, Right at 103 s)
            rounds = {}
            for n in who:
                for v in ch[n]:
                    rounds.setdefault(n, {}).setdefault(int(re.match(r"chicken (\d+)", v["label"]).group(1)), []).append(v)
            for rd in sorted(rounds[a]):
                for n in who:
                    advance(n, [rounds[n][rd][0]])                 # the ready spot
                t_ready = max(t_est[n] for n in who) + MEET_SLACK_S
                for n in who:
                    lab = rounds[n][rd][0]["label"]
                    advance(n, [{"until_s": round(t_ready, 1), "label": lab},
                                {"sync": "chicken%d_%d" % (k, rd - 1), "label": lab}] + rounds[n][rd][1:])
                notes.append("Chicken round %d: both ready about %.0f s in" % (rd, t_ready))
            notes += shape["notes"]
            notes.append("Chicken: swerve at %.0f cm, never under %.0f cm; each jogs %.0f deg to its own side, %.1f cm "
                         "over, and they pass with the bodies %.0f cm apart to swapped ends (run %.0f cm)"
                         % (s1["chicken_swerve_cm"], s1["chicken_min_cm"], shape["jog_deg"], shape["sidestep_cm"],
                            s1["chicken_pass_gap_cm"], s1["chicken_run_cm"]))
        elif kind == "spread-out":
            pos = {n: _pose_after(n) for n in who}
            order = sorted(who, key=lambda n: pos[n][0] if long_x else pos[n][1])
            N = len(order)
            fy = float(bl.get("y", 0.5))
            for i, n in enumerate(order):
                fx = (i + 1) / (N + 1)
                x, y = _frac(m, fx, fy) if long_x else _frac(m, fy, fx)
                advance(n, mtag(n, [_g(x, y, float(bl.get("heading", 90.0)), label="spread out")]))
        elif kind == "dance":
            for n in who:
                advance(n, [{"dance": float(bl.get("seconds", 0)), "label": "dance"}])
        else:
            raise ValueError("block %d: unknown block %r (blocks: %s)" % (k + 1, kind, ", ".join(BLOCKS)))
    out = copy.deepcopy(m)
    out["robots"] = [dict(known[n], start=starts[n], park=list(_pose_after(n))) for n in names]
    out["tasks"] = {n: {"visit": visits[n], "park": False} for n in names}
    if mpair:
        notes.append("%s and %s mirror each other across the middle line (x = %.1f cm): symmetric lanes, the innermost "
                     "pair %.1f cm apart; mirrored legs at %.0f cm/s, re-timed between %.0f and %.0f cm/s"
                     % (mpair[0], mpair[1], cmid, mirror_centre_gap(m), s["mirror_cruise_cms"], s["mirror_min_cms"],
                        s["mirror_max_cms"]))
    out["mission"] = {"name": program.get("name", "mission"), "program": program, "settings": s,
                      "notes": notes, "estimate_s": {n: round(t, 1) for n, t in t_est.items()},
                      "planner": {"separation_cm": s["separation_cm"]}}
    if mpair:
        out["mission"]["mirror"] = {"robots": mpair, "axis": "x" if long_x else "y", "at_cm": round(cmid, 3),
                                    "min_cms": s["mirror_min_cms"], "max_cms": s["mirror_max_cms"],
                                    "cruise_cms": s["mirror_cruise_cms"], "tol_s": s["mirror_tol_s"],
                                    "tol_cm": s["mirror_tol_cm"], "wait_max_s": s["mirror_wait_max_s"]}
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
            mm["tasks"] = {k: {"visit": v, "park": True} for k, v in chicken_mission(m, s=s).items()}
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


SYNC_TOL_S = 0.6        # the robots of a meet / Chicken round set off within this of each other in the plan (the plan's
                        # 0.1 s ticks and a 0.5 s wait round it). No DOE: a display and timing tolerance
SYNC_ROUNDS = 5         # plans at most (each lengthens the earlier robots' waits by how early they set off)


def plan_synced(mm, over=None, t_limit_s=60.0, log=None):
    """Plan a mission whose robots must set off together ({"until_s"} then {"sync"} items: the meet before the
    circle, each Chicken round), then check the plan. An until_s wait is measured on the robot's own timeline
    without the waits the planner adds for the other robots (planner.build_prims), so a robot that yielded on the
    way (13 s for Left on 70 x 130 before Chicken) sets off that much later than the other, and each played its part
    alone. Each robot of a late group gets its own wait lengthened by how far it set off before the group's last
    robot, and the mission is planned again (SYNC_ROUNDS at most). Returns the plan; mm's until_s values change.

    Since Chicken passes (2026-10-28) the whole mission only plans when its robots set off together: the planner
    places one robot first, and if the other is late (it gave way on the way) the first one's pass ends beside the
    late one's ready spot and no plan is found (13 s of searching on 61 x 122). So each group is lined up first on the
    mission cut just after its sync mark (meet, then each Chicken round, in order), and only then is the whole
    mission planned."""
    if over is None:
        # the mission's own planner settings (Monday: separation 8 cm), as mission_run.py and Mission Control pass
        # them; without this a caller with no over planned at the planner's default 5 cm, which with the lanes and
        # Chicken 17 cm in found no plan on 76 x 137 (2026-10-28 21:40) although the real paths plan it
        over = planner_over(mm)
    # the 8 cm lock: whatever planner settings came with the call (Mission Control's saved planner screen held the
    # planner's own 5 cm), a mission with 2 or more robots plans at the net or the mission's chicken_min_cm
    over = dict(over)
    ms = (mm.get("mission") or {}).get("settings") or {}
    for w in gap_lock_planner(over, gap_floor(ms), mission_robots(mm)):
        (log or (lambda x: print(x, file=sys.stderr)))(w)
    _presync(mm, over, t_limit_s, log)
    return _sync_loop(mm, over, t_limit_s, log)


def _sync_ids(mm):
    """Sync marks in the order the robots reach them."""
    out = []
    for t in (mm.get("tasks") or {}).values():
        for v in t.get("visit", []):
            if isinstance(v, dict) and v.get("sync") and v["sync"] not in out:
                out.append(v["sync"])
    return out


def _presync(mm, over, t_limit_s, log):
    """Line up each sync group's waits on the mission cut just after its sync mark (see plan_synced). A cut that does
    not plan is left alone (the whole mission is then planned as it is)."""
    for sid in _sync_ids(mm):
        cut = copy.deepcopy(mm)
        for t in cut["tasks"].values():
            vs = t.get("visit", [])
            i = next((k for k, v in enumerate(vs) if isinstance(v, dict) and v.get("sync") == sid), None)
            if i is not None:
                t["visit"] = vs[:i + 1]
                t["park"] = False
        try:
            _sync_loop(cut, over, t_limit_s, log)
        except P.PlanError:
            if log:
                log("%s: the mission cut there did not plan; planning the whole mission as it is" % sid)
            return
        for n, t in mm["tasks"].items():
            for a, b in zip(t.get("visit", []), cut["tasks"][n].get("visit", [])):
                if isinstance(a, dict) and "until_s" in a and "until_s" in b:
                    a["until_s"] = b["until_s"]


def _sync_loop(mm, over=None, t_limit_s=60.0, log=None):
    plan = None
    for _ in range(SYNC_ROUNDS):
        plan = P.make_plan(mm, over=over, t_limit_s=t_limit_s)
        late = {}
        for sid, who in _sync_steps(plan).items():
            go = {n: t for n, (t, _) in who.items()}
            if len(go) >= 2 and max(go.values()) - min(go.values()) > SYNC_TOL_S:
                late[sid] = {n: max(go.values()) - t for n, t in go.items()}
        if not late:
            return _no_sync_beats(plan)
        for sid, add in late.items():
            for n, d in add.items():
                vs = mm["tasks"][n]["visit"]
                for i, v in enumerate(vs):
                    if v.get("sync") == sid and i > 0 and "until_s" in vs[i - 1]:
                        vs[i - 1]["until_s"] = round(vs[i - 1]["until_s"] + d, 1)
            if log:
                log("%s: planned to set off %.1f s apart; the earlier robot's wait lengthened, planning again"
                    % (sid, max(add.values())))
    return _no_sync_beats(plan)


def _no_sync_beats(plan):
    """A wait to set off together (meet, Chicken) is not a show's beat: the robots go when both are there, so its
    step is not scored as late against the until_s the builder guessed (summary's beats_late_over_0_5s)."""
    for r in plan["robots"].values():
        for st in r["steps"]:
            if st.get("sync"):
                st.pop("beat_s", None)
    return plan


def _sync_steps(plan):
    """{sync id: {robot: (time its next step starts, time it arrived: end of its last move before the waits)}}"""
    out = {}
    for n, r in plan["robots"].items():
        steps = r["steps"]
        for i, st in enumerate(steps):
            if not st.get("sync"):
                continue
            j = i - 1
            while j >= 0 and steps[j]["type"] == "wait":
                j -= 1
            arrived = steps[j]["t_end"] if j >= 0 else 0.0
            nxt = next((x for x in steps[i + 1:] if x["type"] != "wait"), steps[i])
            out.setdefault(st["sync"], {})[n] = (nxt["t_start"], arrived)
    return out
