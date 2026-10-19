"""The site survey route: where a robot stands to range each landmark with its
front distance sensor, and how it approaches each open plywood edge.

Landmark views. For every face of a landmark long enough to range (>= 6 cm) and
on a side where the robot fits, candidate spots put the sensor 14, 24 or 38 cm
from the face, square on or 18 deg to either side, aimed at a point 30 / 70 %
(long faces) or 50 % along it. A spot is kept only if the robot can turn there
(the fleet planner's free-space grid) and the centre zone, from that spot, would
hit that face cleanly by the fusion's own rules (landmarks.predict_zone: not two
faces close in angle, not near the face's ends, within 20 deg of square on). Up
to `per_face` spots per face, the most different first. At each spot the robot
stands still facing the aim point (dwell), turns `sweep_deg` left and right and
stands still again: three still poses per spot, every zone seeing the face at a
different angle (that is what pins down each zone's direction and the board's
turn).

Corner sweeps. For each end of each long face (>= 15 cm), from 16 and 30 cm in
front of it (or up to 8 cm in from the end), facing the end: the robot turns in
2 deg steps over +-14 deg, standing still 1.2 s at each step (corner_steps).

Edge approaches. For every open side, up to 5 spots spread along it (from 10 %
to 90 %, where the robot fits): drive to a spot 16 cm inside the edge, face
straight out, and creep out at 2 cm/s with the edge guard on (line sensors).
The line sensors pass the edge, the guard stops the robot and ends the run:
that moment is a position reference for the edge. The drive itself ends with
the robot centre 3 cm inside the clicked edge (never closer): the line sensors,
3.5 cm ahead of the axle, are then 0.5 cm past the clicked edge, so the guard
trips on its own before the planned stop whenever the real edge is no more than
0.5 cm further out than the click; otherwise the approach ends without a
reading (sim_eval.py counts them). The guard must have passed its E0c check
(tools/edge_check.py). In follow mode (firmware 2026-10-18c) a trip stops only
the step: the robot holds and run_plan.py maps the edge as a no-go strip. The
next approach backs the robot 14 cm in first, in a run of its own, when it ended
within 10 cm of the edge (the guard learns the plywood at the start of each
run, so a run that starts with the line sensors over the cement has no guard).

Routes between spots come from the fleet planner (tools/fleet/planner.py) with
its safety margins, so the plans run on tools/fleet/run_plan.py and the
firmware follower unchanged.
"""
import copy
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "fusion"), os.path.join(TOOLS, "fleet")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import engine as ENG  # noqa: E402
import fleetmap as FM  # noqa: E402
import geometry as G  # noqa: E402
import landmarks as LM  # noqa: E402
import planner as P  # noqa: E402

DEFAULTS = dict(distances=(24.0, 14.0, 38.0), angles=(0.0, -18.0, 18.0), per_face=4, min_face_cm=6.0,
                dwell_s=2.0, sweep_deg=12.0, sweep_dwell_s=1.5, edge_fracs=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9), edge_in_cm=16.0,
                edge_stop_cm=3.0, edge_cms=2.0, back_cm=14.0, corner_dists_cm=(16.0, 30.0), corner_sweep_deg=14.0,
                corner_step_deg=2.0, corner_dwell_s=1.2, max_edge_points=5,
                touch=False, touch_gap_cm=3.0, touch_bump_ahead_cm=6.09, touch_min_face_cm=10.0, touch_dwell_s=2.0)
LINE_FWD_CM = 3.5      # line sensors ahead of the turning centre (grid_center_cm)


def tof_cfg(**over):
    return ENG.config(**over)


def survey_map(m, robot, others_parked=True):
    """A copy of the map with one robot (start = park = its start spot) and the other robots as round
    obstacles where they are parked (their start spots)."""
    m = FM.normalize(copy.deepcopy(m))
    me = [r for r in m["robots"] if r["name"] == robot]
    if not me:
        raise ValueError(f"robot {robot} is not in the map")
    if others_parked:
        for r in m["robots"]:
            if r["name"] != robot:
                m["obstacles"].append(dict(id=f"parked {r['name']}", shape="circle", center=r["start"][:2],
                                           radius=7.0, source="user", time=FM.now_iso(), note="robot parked"))
    m["robots"] = me
    m["tasks"] = {}
    return m


def _grid(m, s=None):
    s = s or P.settings()
    body = FM.body_of(m)
    return P.Grid(FM.World(m), body, s), body


def landmark_viewpoints(m, faces, cfg=None, opts=None, view_poly=None, grid=None):
    """Spots to range the landmark faces from. faces: list of (landmark name, Face) at the map's
    (prior) positions. Returns a list of dict(landmark, face, pose [x, y, deg], dist_cm, angle_deg)."""
    o = dict(DEFAULTS, **(opts or {}))
    cfg = cfg or tof_cfg()
    grid = grid or _grid(m)[0]
    allf = [f for _, f in faces]
    fwd = cfg["tof_forward_cm"] - cfg["tof_recess_cm"]
    Pm = np.diag([0.09, 0.09, (1.0 * math.pi / 180) ** 2])
    out = []
    for name, f in faces:
        if f.length < o["min_face_cm"]:
            continue
        fracs = (0.3, 0.7) if f.length >= 25 else (0.5,)
        cands = []
        for side in (1.0, -1.0):
            n = side * f.n
            for di, d in enumerate(o["distances"]):
                for ai, a in enumerate(o["angles"]):
                    for fr in fracs:
                        q = f.a + fr * f.length * f.u
                        dirv = G.wrap(math.atan2(n[1], n[0]) + math.radians(a))
                        sp = q + d * np.array([math.cos(dirv), math.sin(dirv)])
                        h = G.wrap(dirv + math.pi)
                        c = sp - fwd * np.array([math.cos(h), math.sin(h)])
                        if not grid.turnable(c):
                            continue
                        if view_poly is not None and not G.points_in_polygon(np.array([c]), np.asarray(view_poly))[0]:
                            continue
                        why, info = LM.predict_zone(np.array([c[0], c[1], h]), Pm, 0.0, allf, cfg)
                        if why or allf[info["face"]] is not f:
                            continue
                        rank = (ai > 0) * 10 + di + 0.1 * abs(fr - 0.5)
                        cands.append((rank, dict(landmark=name, face=f.name, pose=[float(c[0]), float(c[1]),
                                                 math.degrees(h)], dist_cm=d, angle_deg=a, frac=fr)))
        cands.sort(key=lambda t: t[0])
        picked = []
        for _, v in cands:
            if len(picked) >= o["per_face"]:
                break
            # the most different first: skip a spot within 6 cm of one already picked
            if any(math.dist(v["pose"][:2], p["pose"][:2]) < 6.0 for p in picked):
                continue
            picked.append(v)
        out += picked
    return out


def order_greedy(start, spots):
    """Nearest-next order from the start pose."""
    left = list(spots)
    out, p = [], np.asarray(start[:2], float)
    while left:
        k = min(range(len(left)), key=lambda i: math.dist(p, left[i]["pose"][:2]))
        out.append(left.pop(k))
        p = np.asarray(out[-1]["pose"][:2])
    return out


def _step(kind, frm, to, phase, speed=None, reverse=False, dur_s=0.0, front_min=None, task=None, board=None):
    st = {"i": 0, "type": kind, "phase": phase, "from": [round(float(v), 2) for v in frm],
          "to": [round(float(v), 2) for v in to], "t_start": 0.0, "t_end": round(dur_s, 2), "move_s": round(dur_s, 3),
          "board": board, "task": task, "after": [], "speed": speed}
    if kind in ("drive", "approach"):
        st["reverse"] = bool(reverse)
    if kind in ("turn", "park"):
        st["turn_deg"] = round(G.wrap(math.radians(to[2] - frm[2])) * 180 / math.pi, 2)
    if front_min is not None:
        st["front_min_cm"] = front_min
    return st


def retime(steps, s=None):
    """Renumber and re-time a single robot's steps one after another (no other robots: empty after lists)."""
    s = s or P.settings()
    t = 0.0
    for i, st in enumerate(steps):
        st["i"] = i
        if st["type"] == "wait":
            dur = st["t_end"] - st["t_start"]
        elif st["type"] in ("turn", "park"):
            dur = abs(st.get("turn_deg", 0.0)) / s.turn_dps + s.turn_settle_s
        elif st["type"] in ("touch", "home"):
            dur = st["t_end"] - st["t_start"]
        else:
            dur = math.dist(st["from"][:2], st["to"][:2]) / max(st["speed"] or s.cruise_cms, 0.1) + s.drive_settle_s
        st["t_start"], st["t_end"] = round(t, 2), round(t + dur, 2)
        t += dur
    return steps


def view_steps(pose, front_ray, o, task):
    """Dwell facing the spot's heading, sweep left, dwell, sweep right, dwell (a list of plan steps)."""
    out = [_step("wait", pose, pose, "dwell", dur_s=o["dwell_s"], task=task)]
    cur = list(pose)
    for dh in (o["sweep_deg"], -2 * o["sweep_deg"]):
        nxt = [cur[0], cur[1], cur[2] + dh]
        k = max(1, int(abs(dh) // 5))
        seen = [front_ray([cur[0], cur[1], math.radians(cur[2] + dh * j / k)]) for j in range(k + 1)]
        seen = [v for v in seen if v is not None]
        out.append(_step("turn", cur, nxt, "sweep", speed=60.0, front_min=min(seen) if seen else None, task=task))
        out.append(_step("wait", nxt, nxt, "dwell", dur_s=o["sweep_dwell_s"], task=task))
        cur = nxt
    return out


def touch_viewpoints(m, landmarks, opts=None, grid=None):
    """The optional touch probe survey step (nose switches, firmware dock.h): spots square to each long face
    of every board, with the bumper touch_gap_cm from it: a quarter and three quarters along it (two points
    fix the face's turn too), or its middle if it is shorter than 20 cm. Returns a list of dict(landmark, face,
    pose [x, y, deg], kind "touch")."""
    o = dict(DEFAULTS, **(opts or {}))
    grid = grid or _grid(m)[0]
    out = []
    for l in landmarks:
        if l.kind != "board":
            continue
        for f in l.faces:
            if f.length < o["touch_min_face_cm"]:
                continue
            for fr in ((0.25, 0.75) if f.length >= 20.0 else (0.5,)):
                q = f.a + fr * f.length * f.u
                for side in (1.0, -1.0):
                    n = side * f.n
                    c = q + (o["touch_bump_ahead_cm"] + o["touch_gap_cm"]) * n
                    if not grid.turnable(c):
                        continue
                    h = math.degrees(math.atan2(-n[1], -n[0]))
                    out.append(dict(landmark=l.name, face=f.name, pose=[float(c[0]), float(c[1]), h], kind="touch",
                                    frac=fr))
                    break
    return out


def touch_steps(pose, o, task):
    """Dwell (a camera station), then creep until a nose switch clicks and back off to the spot (follow step P
    with its own back-off), then dwell again."""
    st = _step("touch", pose, pose, "touch", dur_s=8.0, task=task)
    st["max_travel_cm"] = round(o["touch_gap_cm"] + 2.0, 2)
    st["back_cm"] = round(o["touch_gap_cm"], 2)
    return [_step("wait", pose, pose, "dwell", dur_s=o["touch_dwell_s"], task=task), st,
            _step("wait", pose, pose, "dwell", dur_s=1.0, task=task)]


def plan_views(m, robot, start, spots, opts=None, t_limit_s=30.0, home=None):
    """One plan for `robot` visiting the spots in order from `start` (x, y, deg) and back to `home` (default:
    start), with the dwell/sweep steps at each spot (corner spots: the fine sweep). Returns (plan dict, the
    spots actually in it, the survey map)."""
    o = dict(DEFAULTS, **(opts or {}))
    sm = survey_map(m, robot, others_parked=False)
    sm["robots"][0]["start"] = list(start)
    sm["robots"][0]["park"] = list(home or start)
    sm["tasks"] = {robot: {"visit": [{"goto": sp["pose"]} for sp in spots], "any_order": False, "park": True}}
    plan = P.make_plan(sm, over={"skip_unreachable": True}, t_limit_s=t_limit_s)
    steps = plan["robots"][robot]["steps"]
    body = FM.body_of(FM.normalize(sm))
    fr = P._front_ray_fn(FM.normalize(sm), body)
    last = {}
    for k, st in enumerate(steps):
        if st.get("task") is not None and st["task"] < len(spots):
            last[st["task"]] = k
    out, kept = [], []
    for k, st in enumerate(steps):
        out.append(st)
        for task, kk in last.items():
            if kk == k:
                if spots[task].get("kind") == "corner":
                    out += corner_steps(st["to"], fr, o, task)
                elif spots[task].get("kind") == "touch":
                    out += touch_steps(st["to"], o, task)
                else:
                    out += view_steps(st["to"], fr, o, task)
                kept.append(dict(spots[task], task=task))
    for k, st in enumerate(out):
        st["spot"] = st.get("task") if st.get("task") is not None and st["task"] < len(spots) else None
    plan["robots"][robot]["steps"] = retime(out)
    plan["robots"][robot]["finish_s"] = out[-1]["t_end"] if out else 0.0
    plan["summary"]["makespan_s"] = plan["robots"][robot]["finish_s"]
    plan["site_spots"] = kept
    plan["map_hash"] = FM.map_hash(sm)
    plan["plan_id"] = P.plan_id(plan)
    return plan, kept, sm


def corner_steps(pose, front_ray, o, task):
    """The fine sweep at a corner spot: still at headings -corner_sweep_deg .. +corner_sweep_deg in
    corner_step_deg steps, then back to the middle. Each still pose is a station; the reading of a zone flips
    between the board and what is behind it between two neighbouring stations: that brackets the corner's
    direction (tools/site/solve.py, corner bearings)."""
    out = []
    cur = list(pose)
    span, stp = o["corner_sweep_deg"], o["corner_step_deg"]
    hs = [pose[2] - span + k * stp for k in range(int(round(2 * span / stp)) + 1)]
    for h in hs + [pose[2]]:
        nxt = [cur[0], cur[1], h]
        seen = front_ray([cur[0], cur[1], math.radians(h)])
        out.append(_step("turn", cur, nxt, "sweep", speed=60.0, front_min=seen, task=task))
        out.append(_step("wait", nxt, nxt, "dwell", dur_s=o["corner_dwell_s"], task=task))
        cur = nxt
    return out


def corner_viewpoints(m, landmarks, cfg=None, opts=None, view_poly=None, grid=None):
    """Spots for the fine sweep at each end of each landmark's long faces: the sensor 16 and 30 cm from the
    face (two distances: a zone's edge ray is d x tan(width / 3) to the side of its centre line, so two
    distances tell the corner's position from the zone's width), up to 8 cm in from the end along it,
    facing the end (so it is in the middle of the sweep). For each distance the first spot where the robot
    can turn is kept."""
    o = dict(DEFAULTS, **(opts or {}))
    cfg = cfg or tof_cfg()
    grid = grid or _grid(m)[0]
    fwd = cfg["tof_forward_cm"] - cfg["tof_recess_cm"]
    out = []
    for l in landmarks:
        long = [f for f in l.faces if f.length >= 15.0]
        for f in long:
            for end, q, inward in (("a", f.a, f.u), ("b", f.b, -f.u)):
                for side in (1.0, -1.0):
                    n = side * f.n
                    # the face must be on the robot's side: the normal points away from the landmark's centre
                    if np.dot(q - l.center, n) <= 0:
                        continue
                    found = None
                    for d in o["corner_dists_cm"]:
                        for off in (0.0, 4.0, 8.0):
                            sp = q + d * n + off * inward
                            v = q - sp
                            h = math.atan2(v[1], v[0])
                            c = sp - fwd * np.array([math.cos(h), math.sin(h)])
                            if not grid.turnable(c):
                                continue
                            if view_poly is not None and not G.points_in_polygon(np.array([c]), np.asarray(view_poly))[0]:
                                continue
                            found = dict(landmark=l.name, face=f.name, kind="corner", end=end,
                                         pose=[float(c[0]), float(c[1]), math.degrees(h)], dist_cm=d, angle_deg=0.0,
                                         inward_cm=off)
                            out.append(found)
                            break
    return out


def split_spots(spots, max_per_segment):
    """Spots in segments of at most max_per_segment (each segment is its own run, from and back to the
    start spot: without live camera fixes a robot's own pose drifts, so runs are kept short)."""
    if not max_per_segment or max_per_segment <= 0:
        return [spots]
    return [spots[i:i + max_per_segment] for i in range(0, len(spots), max_per_segment)]


def edge_targets(m, edges, opts=None, grid=None):
    """For every open edge and fraction: dict(side, frac, pre (x, y, deg), end (x, y, deg)) where the robot can
    stand 16 cm inside the edge facing out, with nothing (board, obstacle) in its way out."""
    o = dict(DEFAULTS, **(opts or {}))
    grid = grid or _grid(m)[0]
    world = FM.World(m)
    out = []
    for e in edges:
        if not e.open:
            continue
        for fr in o["edge_fracs"]:
            q = e.a + fr * (e.b - e.a)
            h = math.degrees(math.atan2(e.n[1], e.n[0]))
            pre = q - o["edge_in_cm"] * e.n
            end = q - o["edge_stop_cm"] * e.n
            if not grid.turnable(pre):
                continue
            # the drive out sweeps the body to the edge: no board or obstacle in that strip (floor edge aside)
            body = FM.body_of(m)
            sweep = body.drive_sweep(pre, end, math.radians(h))
            clear = min([G.polygon_polygon_dist(sweep, poly) for _, poly in world.polys] or [99.0])
            if clear < 1.5:
                continue
            out.append(dict(side=e.side, frac=fr, pre=[float(pre[0]), float(pre[1]), h], end=[float(end[0]), float(end[1]), h]))
    # at most max_edge_points per edge, spread along it
    res = []
    for side in {t["side"] for t in out}:
        ts = [t for t in out if t["side"] == side]
        k = o["max_edge_points"]
        if len(ts) > k:
            idx = np.unique(np.round(np.linspace(0, len(ts) - 1, k)).astype(int))
            ts = [ts[i] for i in idx]
        res += ts
    return sorted(res, key=lambda t: (t["side"], t["frac"]))


def plan_edge(m, robot, start, target, opts=None, back_first=False):
    """Steps for one edge approach from `start`: (back 14 cm straight in, if starting at an edge), the planned
    route to the target's pre-edge spot, turn out, creep out with the edge guard on. A list of plan steps."""
    o = dict(DEFAULTS, **(opts or {}))
    steps = []
    cur = list(start)
    if back_first:
        h = math.radians(cur[2])
        back = [cur[0] - o["back_cm"] * math.cos(h), cur[1] - o["back_cm"] * math.sin(h), cur[2]]
        steps.append(_step("drive", cur, back, "edge_back", speed=5.0, reverse=True))
        cur = back
    sm = survey_map(m, robot)
    sm["robots"][0]["start"] = list(cur)
    sm["robots"][0]["park"] = list(cur)
    sm["tasks"] = {robot: {"visit": [{"goto": target["pre"]}], "any_order": False, "park": False}}
    plan = P.make_plan(sm, t_limit_s=20.0)
    steps += plan["robots"][robot]["steps"]
    pre = steps[-1]["to"] if steps else cur
    steps.append(_step("drive", pre, [target["end"][0], target["end"][1], pre[2]], "edge_out", speed=o["edge_cms"]))
    return retime(steps)


def near_open_edge(pose, edges, within_cm=10.0):
    """Is the robot centre within `within_cm` of an open edge (so a run starting here could begin with the line
    sensors over the cement)?"""
    p = np.asarray(pose[:2], float)
    return any(e.open and float(np.dot(e.mid - p, e.n)) < within_cm for e in edges)


def plan_back(start, opts=None):
    """After the last edge: back 14 cm straight in."""
    o = dict(DEFAULTS, **(opts or {}))
    h = math.radians(start[2])
    back = [start[0] - o["back_cm"] * math.cos(h), start[1] - o["back_cm"] * math.sin(h), start[2]]
    return retime([_step("drive", start, back, "edge_back", speed=5.0, reverse=True)])


def plan_return(m, robot, start, home):
    """Planned route from `start` back to `home` (the robot's start spot)."""
    sm = survey_map(m, robot)
    sm["robots"][0]["start"] = list(start)
    sm["robots"][0]["park"] = list(home)
    sm["tasks"] = {robot: {"visit": [], "any_order": False, "park": True}}
    plan = P.make_plan(sm, t_limit_s=20.0)
    return retime(plan["robots"][robot]["steps"])


def plan_file(m_survey, robot, steps, start):
    """A plan.json run_plan.py accepts, for one robot and given steps."""
    plan = {"format": P.PLAN_FORMAT, "version": P.PLAN_VERSION, "created": FM.now_iso(),
            "map_hash": FM.map_hash(m_survey), "t0_s": 0.0, "replan_of": None, "settings": dict(P.settings()),
            "priority_order": [robot], "docks": {}, "events": [], "warnings": [], "stranded": [],
            "robots": {robot: {"start": [round(float(v), 2) for v in start], "task_offset": 0, "pending": None,
                               "visit_order": [], "finish_s": steps[-1]["t_end"] if steps else 0.0, "steps": steps}}}
    plan["summary"] = {"makespan_s": plan["robots"][robot]["finish_s"]}
    plan["plan_id"] = P.plan_id(plan)
    return plan
