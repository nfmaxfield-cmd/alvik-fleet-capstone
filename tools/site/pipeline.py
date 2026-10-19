"""The site setup's survey, end to end on the simulator (setup.py --sim and
sim_eval.py), and the pieces both share with the real-robot path: what the
user's clicks give, the per-robot priors, and running the solver.
"""
import copy
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "fusion"), os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fleetmap as FM  # noqa: E402
import sitemap as SM  # noqa: E402
import survey as SV  # noqa: E402
import solve as SO  # noqa: E402

EDGE_SETTINGS = ("set safe_edge 1",)          # line sensors (edge_dir / edge_thr: E0c, tools/edge_check.py)
EDGE_OFF = ("set safe_edge 0",)


def clicked_map(true_map, sigma_cm, seed, edge_sigma_cm=None):
    """What clicking on the camera picture gives: every board end and block corner off by a normal error of
    sigma_cm in x and in y (independently), and the floor's sides off by edge_sigma_cm."""
    rng = np.random.default_rng(seed)
    m = copy.deepcopy(FM.normalize(true_map))
    for b in m["boards"]:
        b["p1"] = list(np.asarray(b["p1"]) + rng.normal(0, sigma_cm, 2))
        b["p2"] = list(np.asarray(b["p2"]) + rng.normal(0, sigma_cm, 2))
    for o in m["obstacles"]:
        if o["shape"] == "polygon" and o.get("source") == "user":
            o["points"] = [list(np.asarray(p) + rng.normal(0, sigma_cm, 2)) for p in o["points"]]
    es = sigma_cm if edge_sigma_cm is None else edge_sigma_cm
    f = m["floor"]
    for k in f:
        f[k] = float(f[k] + rng.normal(0, es))
    return m


def views_for(m, robot, landmarks, opts=None, view_poly=None):
    sm = SV.survey_map(m, robot, others_parked=False)
    faces = [(l.name, f) for l in landmarks for f in l.faces]
    return SV.landmark_viewpoints(sm, faces, opts=opts, view_poly=view_poly)


def share_views(m, robots, landmarks, opts=None, view_poly=None):
    """Spots for each robot: every robot ranges every landmark it can reach (each robot's own sensor
    errors are then measured on every landmark, and the landmarks are seen by three sensors)."""
    return {r: views_for(m, r, landmarks, opts, view_poly) for r in robots}


def run_sim_survey(site, user_map, robots, edges, opts=None, log=print, max_spots=None, segment_spots=0,
                   do_edges=True, edge_robots=None, only_landmarks=None):
    """The survey on the simulator: for each robot in turn (the others off the board), its view spots
    (planned on the user's map: that is all we know before the survey) in segments, then the edge
    approaches. Returns (spots done per robot, edge trips per robot, robot seconds)."""
    o = dict(SV.DEFAULTS, **(opts or {}))
    m = FM.normalize(user_map)
    L = SM.landmarks_from_map(m, include=only_landmarks)
    starts = {r["name"]: r["start"] for r in m["robots"]}
    done, trips = {}, {}
    t0 = site.robot_time_s
    for rn in robots:
        start = starts[rn]
        spots = views_for(m, rn, L, o)
        if o.get("corners", True):
            spots += SV.corner_viewpoints(SV.survey_map(m, rn, others_parked=False), L, opts=o)
        if o.get("touch"):
            spots += SV.touch_viewpoints(SV.survey_map(m, rn, others_parked=False), L, o)
        spots = SV.order_greedy(start, spots)
        if max_spots:
            spots = spots[:max_spots]
        done[rn] = []
        for seg in SV.split_spots(spots, segment_spots):
            todo, here, tries = list(seg), list(start), 0
            while todo and tries < 6:
                tries += 1
                plan, kept, _ = SV.plan_views(m, rn, here, todo, o, home=start)
                steps = plan["robots"][rn]["steps"]
                run, failed, ok = site.follow(rn, here, steps)
                ok = set(ok)
                fin = [sp for sp in kept if all(i in ok for i, st in enumerate(steps) if st.get("spot") == sp["task"])]
                done[rn] += fin
                if not failed:
                    break
                # a step stopped (something closer than the map said): drop the spot it was on or heading to,
                # plan the rest again from where the robot is (run_plan.py re-plans the same way)
                bad = next((st.get("spot") for i, st in enumerate(steps) if i not in ok and st.get("spot") is not None), None)
                left = [sp for sp in kept if sp not in fin and sp["task"] != bad]
                log(f"  {rn}: a step stopped ({failed[1][:60]}); {len(left)} spots planned again from here")
                todo = [{k: v for k, v in sp.items() if k != "task"} for sp in left]
                here = list(site.seen_pose(rn))
            log(f"  {rn}: {len(done[rn])} spots done")
        trips[rn] = []
        if not do_edges or (edge_robots is not None and rn not in edge_robots):
            continue
        targets = SV.edge_targets(SV.survey_map(m, rn, others_parked=False), edges, o)
        at_edge = False
        for tg in targets:
            here = site.seen_pose(rn)
            if at_edge and SV.near_open_edge(here, edges):
                # back off the edge in a run of its own: the edge guard learns the plywood at the start of each
                # run, so the next approach must start with the line sensors on the plywood
                site.follow(rn, here, SV.plan_back(here, o))
                here = site.seen_pose(rn)
                at_edge = False
            try:
                steps = SV.plan_edge(m, rn, here, tg, o, back_first=False)
            except Exception as e:                       # noqa: BLE001  (planner: no route)
                log(f"  {rn}: edge {tg['side']} {int(tg['frac'] * 100)}% skipped ({e})")
                continue
            run, failed, _ = site.follow(rn, here, steps, settings=EDGE_SETTINGS)
            res = run["result"] if run else "?"
            # follow mode (firmware 2026-10-18c): the guard stops the step ("plywood edge at ..."), the robot
            # holds; older firmware ended the run ("plywood edge ahead" / "limited:plywood edge")
            tripped = "plywood edge" in res or bool(failed and str(failed[1]).startswith("plywood edge"))
            trips[rn].append(dict(side=tg["side"], frac=tg["frac"], tripped=tripped, result=res))
            at_edge = True
        if at_edge:
            here = site.seen_pose(rn)
            site.follow(rn, here, SV.plan_back(here, o) if SV.near_open_edge(here, edges) else [], settings=EDGE_OFF)
            here = site.seen_pose(rn)
            try:
                site.follow(rn, here, SV.plan_return(m, rn, here, start))
            except Exception as e:                       # noqa: BLE001
                log(f"  {rn}: no route back to the start spot ({e})")
        n_t = sum(t["tripped"] for t in trips[rn])
        log(f"  {rn}: {len(trips[rn])} edge approaches, {n_t} stopped by the edge guard")
    return done, trips, site.robot_time_s - t0


def robot_priors_from_cal(cal, bench=None):
    """Per-robot priors for the solver: from a tof_calibration.json (zone directions and their sd), and/or a
    bench file {robot: {bias_cm, bias_sd_cm, scale, scale_sd}} (E0c C9: the robot square to a board at
    tape-measured distances)."""
    out = {}
    for rn, r in ((cal or {}).get("robots") or {}).items():
        out[rn] = dict(zone_offset_deg=r.get("zone_offset_deg", [0.0] * 5),
                       zone_sd_deg=[max(float(r.get("yaw_sigma_deg", 1.0)), 0.5)] * 5)
    for rn, b in (bench or {}).items():
        d = out.setdefault(rn, {})
        for k in ("bias_cm", "bias_sd_cm", "scale", "scale_sd"):
            if k in b:
                d[k] = float(b[k])
    return out


def solve_site(user_map, open_sides, runs, sightings, robots, opts=None, robot_priors=None, tape=None,
               click_sigma_cm=1.0, edge_sigma_cm=1.0, cam_sys_cm=None, verbose=False, prior_sigma_by_landmark=None):
    """prior_sigma_by_landmark: {name: cm} instead of click_sigma_cm for those landmarks (a refresh: the
    landmarks not surveyed again keep their locked uncertainty as their prior)."""
    m = FM.normalize(user_map)
    L = SM.landmarks_from_map(m, prior_sigma_cm=click_sigma_cm)
    for l in L:
        if prior_sigma_by_landmark and l.name in prior_sigma_by_landmark:
            l.prior_sigma_cm = float(prior_sigma_by_landmark[l.name])
    E = SM.edges_from_map(m, open_sides, prior_sigma_cm=edge_sigma_cm)
    prob = SO.build(L, E, runs, sightings, robots, opts, robot_priors, tape)
    return SO.solve(prob, cam_sys_cm=cam_sys_cm, verbose=verbose)


def map_errors(est, true_map):
    """Per landmark: the largest distance (cm) between an estimated tape point and the true one; per open
    edge: the error along its normal at 25/50/75 %."""
    tm = FM.normalize(true_map)
    truth = {l.name: l for l in SM.landmarks_from_map(tm)}
    out = {}
    for name, l in est.landmarks.items():
        t = truth[name]
        errs = [float(np.hypot(*(est.anchor(name, a) - t.anchors[a]))) for a in l.anchors]
        out[name] = max(errs)
    for side, e in est.edges.items():
        if not e.open:
            continue
        te = SM.Edge(side, tm["floor"], True)
        errs = []
        for fr in (0.25, 0.5, 0.75):
            p = e.point(fr, est.e[side])
            q = te.point(fr, np.zeros(2))
            errs.append(abs(float(np.dot(p - q, te.n))))
        out[f"edge {side}"] = max(errs)
    return out


def estimate_from_map(user_map, open_sides, click_sigma_cm=1.0, cam_sys_cm=0.15):
    """The clicks alone as an estimate (case a)."""
    m = FM.normalize(user_map)
    L = SM.landmarks_from_map(m, prior_sigma_cm=click_sigma_cm)
    E = SM.edges_from_map(m, open_sides, prior_sigma_cm=click_sigma_cm)
    return SM.Estimate(L, E, cam_sys_cm=0.0)
