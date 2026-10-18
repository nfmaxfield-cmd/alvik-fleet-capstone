#!/usr/bin/env python3
"""Simulation evaluation of the site setup (docs/experiments/E00d-site-setup).

    python tools/site/sim_eval.py --out results/site_eval [--seeds 1 2 3] [--config tools/sim/configs/x.json]

For each seed: the true world (tools/site/examples/sim_site_world.json: five
boards / a cinderblock, open sides x0 and y1), the user's clicks (every board end
off by a normal error of --click-sd cm in x and y), the robots' own errors
(--config: per-robot front-sensor zone directions and range errors, wheel sizes;
default: the fusion's r7/r11 set), a camera with 0.3 cm / 3 deg noise per
sighting and a fixed calibration error field (--cam-sys rms cm). Then:

  (a) clicks only: the map as clicked
  (b) clicks + per-robot calibration runs: each robot's calibration run on its
      start spot, then tools/fusion/tof_calib.py's fit (as it is used today: the
      spins' readings of the clicked boards; the robots' average range bias held
      at 0) refines the boards
  (c) site setup: the survey (every robot: view spots, corner sweeps, edge
      approaches), the joint solve with (b)'s sensor directions and the bench
      range priors (E0c C9), then the tape audit (2 largest-uncertainty points +
      1 random); a failed audit re-surveys the failing landmarks and merges the
      tape readings; once passed, the audit readings are merged too (c+tape)
  (c0) as (c) without the bench range priors

Map error per landmark: the largest distance between an estimated and the true
tape point (both ends of both long faces); edges: along the normal at 25, 50,
75 %. Also: user effort (clicks, confirmations, tape points, estimated minutes)
and robot time.

Writes <out>/site_eval.json and site_eval.csv, and for --fusion-files the
locked map and the site calibration (tof_calibration.json format) of the first
seed, made with no camera calibration error (the fusion's simulated sessions
have their own camera frame), for tools/fusion/replay.py.
"""
import argparse
import copy
import csv
import json
import math
import os
import statistics as stats
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "fusion"), os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import config as SIMCFG  # noqa: E402
import engine as ENG  # noqa: E402
import fleetmap as FM  # noqa: E402
import landmarks as LM  # noqa: E402
import pipeline as PL  # noqa: E402
import simrun as SR  # noqa: E402
import sitemap as SM  # noqa: E402
import tof_calib as TC  # noqa: E402

WORLD = os.path.join(HERE, "examples", "sim_site_world.json")
OPEN = ["x0", "y1"]
R_CONFIG = {
    "robot": {"radius_scale_err_pct_by_robot": {"Left": -3.0, "Center": 3.0, "Right": -3.0}},
    "tof": {"zone_angle_err_deg_by_robot": {"Left": [3.3, 2.0, 2.8, 1.5, 3.1], "Center": [-2.6, -1.4, -2.0, -2.9, -1.2],
                                             "Right": [1.0, 2.2, 1.5, 0.4, 2.3]},
            "range_err_by_robot": {"Left": [0.4, 1.0], "Center": [-0.3, 0.0], "Right": [0.2, -1.0]}}}
# user effort (minutes): estimates, see the E00d record
EFFORT_S = dict(click=10, confirm=5, open_edge_answer=10, tape_point=90, place_robot=30, length_tape=0)


def sim_config(path):
    """The simulator's settings: --config, else the r7/r11 sensor errors. No tape lines on the floor (the real
    board has none: tools/sim/configs/real_oct18.json; tape next to an open edge would trip the edge guard
    early unless edge_max is set from E0c's edge_tape row)."""
    cfg = SIMCFG.load_config()
    if path:
        cfg = SIMCFG.load_config(path)
    else:
        SIMCFG._merge(cfg, copy.deepcopy(R_CONFIG))
    cfg.session.tape_lines = []
    return cfg


def bench_priors(cfg, robots, seed, sd_bias=0.15, sd_scale=0.003):
    """What E0c's C9 rows give (robot square to a board at tape distances 10..100 cm): each robot's range
    bias and scale, measured with an error of sd_bias / sd_scale."""
    rng = np.random.default_rng(seed + 99)
    out = {}
    for r in robots:
        b, s = cfg.tof.range_err_by_robot.get(r, [0.0, 0.0])
        out[r] = dict(bias_cm=float(b + rng.normal(0, sd_bias)), bias_sd_cm=max(sd_bias, 0.1) * 1.3,
                      scale=float(1 + s / 100 + rng.normal(0, sd_scale)), scale_sd=sd_scale * 1.3)
    return out


# ---------------------------------------------------------------- (b): calibration runs + tof_calib
def spin_refs(runs, sightings):
    """Reference path of each calibration run: position from the camera (median of 0.3 s), heading from the
    gyro tied to the camera's average heading."""
    refs, steps = {}, {}
    for run in runs:
        if run.experiment != "calib" or len(run.raw) < 50:
            continue
        S = np.array([s for s in sightings[run.robot] if run.raw[0]["t"] - 1 <= s[0] <= run.raw[-1]["t"] + 1])
        if len(S) < 20:
            continue
        T = np.array([r["t"] for r in run.raw])
        yaw = np.unwrap(np.radians([ENG.fnum(r.get("yaw_deg")) for r in run.raw]))
        cam_th = np.radians(np.interp(T, S[:, 0], np.degrees(np.unwrap(np.radians(S[:, 3])))))
        # gyro sign against the camera, then the offset
        sgn = 1.0 if np.corrcoef(np.diff(yaw), np.diff(cam_th))[0, 1] >= 0 else -1.0
        th = sgn * yaw
        off = math.atan2(np.mean(np.sin(cam_th - th)), np.mean(np.cos(cam_th - th)))
        th = th + off
        out = []
        for t, h in zip(T, th):
            m = np.abs(S[:, 0] - t) <= 0.3
            if m.sum() >= 3:
                out.append((t, float(np.median(S[m, 1])), float(np.median(S[m, 2])), h))
        if len(out) > 10:
            refs[run.run_id] = np.array(out)
            steps[run.run_id] = TC.spin_steps(run)
    return refs, steps


def case_b(user_map, runs, sightings, board_cm=2.0):
    """tof_calib's fit on the calibration spins: (refined Estimate, its calibration dict)."""
    m = FM.normalize(user_map)
    faces = LM.faces_from_map(m)
    refs, steps = spin_refs(runs, sightings)
    cfg = ENG.config(tof_max_incidence_deg=45.0)
    data, robots = TC.collect([r for r in runs if r.run_id in refs], refs, faces, cfg, steps=steps)
    L = SM.landmarks_from_map(m)
    if data is None:
        return SM.Estimate(L, SM.edges_from_map(m, OPEN), cam_sys_cm=0.0), None
    model = TC.Model(data, robots, faces, cfg, board_cm=board_cm)
    p = model.x0(0.0, 0.0)
    noise = (0.5, 0.03)
    for _ in range(2):
        sigma = noise[0] + noise[1] * np.abs(data["z"])
        p = TC.fit(model, sigma, p0=p)
        noise = TC.noise_model(model, p)
    cal = TC.cal_dict(model, p, noise, (0.0, 0.0), sorted(set(data["run"])), "clicked", None, 0.0)
    g = {}
    for l in L:
        b = cal["boards"].get(l.name)
        g[l.name] = np.array([b["dx_cm"], b["dy_cm"], b["turn_deg"], 0.0]) if b else np.zeros(4)
    return SM.Estimate(L, SM.edges_from_map(m, OPEN), g=g, cam_sys_cm=0.0), cal


# ---------------------------------------------------------------- one seed
def one_seed(seed, args, log=print):
    true = json.load(open(WORLD))
    tm = FM.normalize(true)
    robots = [r["name"] for r in tm["robots"]]
    cfg = sim_config(args.config)
    user = PL.clicked_map(true, args.click_sd, seed=seed)
    cam = SR.CameraModel(seed, sys_cm=args.cam_sys)
    site = SR.SimSite(tm, cfg, seed=seed, camera=cam, board_cm=[0, 0, 122, 122], open_edges=OPEN)
    starts = {r["name"]: r["start"] for r in tm["robots"]}
    out = dict(seed=seed)
    # (a)
    est_a = PL.estimate_from_map(user, OPEN, click_sigma_cm=args.click_sd)
    out["a"] = PL.map_errors(est_a, true)
    # (b): one calibration run per robot (on its start spot, as after power-on)
    t0 = site.robot_time_s
    for r in robots:
        site.calibrate(r, starts[r])
    t_calib = site.robot_time_s - t0
    cal_runs = [run for r in robots for run in site.runs(r) if run.experiment == "calib"]
    est_b, cal_b = case_b(user, cal_runs, site.sightings)
    out["b"] = PL.map_errors(est_b, true)
    out["b_robots"] = (cal_b or {}).get("robots")
    # (c): the survey
    E = SM.edges_from_map(FM.normalize(user), OPEN, prior_sigma_cm=args.click_sd)
    t1 = time.time()
    done, trips, t_survey = PL.run_sim_survey(site, user, robots, E, log=log)
    log(f"  survey: {t_survey / 60:.1f} robot-minutes ({time.time() - t1:.0f} s to simulate)")
    runs = [run for r in robots for run in site.runs(r) if run.experiment == "follow"]
    if getattr(args, "keep", False):
        import pickle
        with open(os.path.join(args.out, f"seed{seed}_logs.pkl"), "wb") as f:
            pickle.dump(dict(runs=runs, sightings=site.sightings, user=user, cal_b=cal_b,
                             bench=bench_priors(cfg, robots, seed),
                             truth={r: [(rb.t_at(ms), x, y, th) for ms, x, y, th, _ in rb.traj]
                                    for r, rb in site.robots.items()}), f)
    pri = PL.robot_priors_from_cal(cal_b, bench_priors(cfg, robots, seed))
    pri0 = PL.robot_priors_from_cal(cal_b, None)
    opts = dict(cam_sys_cm=args.cam_sys if args.cam_sys > 0 else 0.0)
    res0 = PL.solve_site(user, OPEN, runs, site.sightings, robots, opts=opts, robot_priors=pri0,
                         click_sigma_cm=args.click_sd)
    out["c0"] = PL.map_errors(res0.est, true)
    res = PL.solve_site(user, OPEN, runs, site.sightings, robots, opts=opts, robot_priors=pri,
                        click_sigma_cm=args.click_sd)
    out["c"] = PL.map_errors(res.est, true)
    out["c_sigma"] = {k: round(res.est.landmark_sigma(k), 3) for k in res.est.landmarks}
    out["c_summary"] = res.summary()
    out["c_robots"] = res.robots_out
    # the audit loop
    audit = run_audit(seed, res, user, runs, site, robots, pri, true, args, log)
    out["audit"] = audit["rounds"]
    out["fill_in"] = audit["fill"]
    out["c_audited"] = PL.map_errors(audit["audited"].est, true)       # the map the audit checked
    out["c_tape"] = PL.map_errors(audit["final"].est, true)
    out["c_after_audit_sigma"] = {k: round(audit["final"].est.landmark_sigma(k), 3) for k in audit["final"].est.landmarks}
    # effort
    n_clicks = 2 * len(tm["boards"]) + 2 * len(OPEN)
    n_tape = sum(len(r["points"]) for r in audit["rounds"]) + len(audit["fill"])
    effort = dict(clicks=n_clicks, confirmations=len(tm["boards"]) + len(OPEN), open_edge_answers=1,
                  tape_points=n_tape, robot_placements=len(robots), audit_rounds=len(audit["rounds"]))
    effort["user_minutes"] = round((effort["clicks"] * EFFORT_S["click"] + effort["confirmations"] * EFFORT_S["confirm"]
                                    + EFFORT_S["open_edge_answer"] + n_tape * EFFORT_S["tape_point"]
                                    + effort["robot_placements"] * EFFORT_S["place_robot"]) / 60.0, 1)
    effort["robot_minutes_calibration"] = round(t_calib / 60, 1)
    effort["robot_minutes_survey"] = round((t_survey + audit["robot_s"]) / 60, 1)
    effort["spots"] = {r: len(v) for r, v in done.items()}
    effort["edge_approaches"] = {r: len(v) for r, v in trips.items()}
    effort["edge_trips"] = {r: sum(t["tripped"] for t in v) for r, v in trips.items()}
    out["effort"] = effort
    out["_res"] = audit["final"]
    out["_runs"] = runs
    out["_user"] = user
    return out


def run_audit(seed, res, user, runs, site, robots, pri, true, args, log, max_rounds=3):
    """Tape audit: pick, measure (truth + tape error), compare; on a failure re-survey the failing landmarks
    and merge the readings as data; once passed, merge the audit readings too (the lock records the audit
    as it was before merging)."""
    rng = np.random.default_rng(seed + 7)
    tm = FM.normalize(true)
    truth = {l.name: l for l in SM.landmarks_from_map(tm)}
    tape, used, rounds = [], set(), []
    cur = res
    t0 = site.robot_time_s

    def measure(p):
        if p["kind"] == "edge":
            side = p["landmark"].split()[1]
            q = SM.Edge(side, tm["floor"], True).point(p["anchor"], np.zeros(2))
        else:
            q = truth[p["landmark"]].anchors[p["anchor"]]
        return q + rng.normal(0, SM.AUDIT_TAPE_SD_CM, 2)
    # fill-in: what the robots could not pin down is tape-measured and merged first (repeat: one point per
    # landmark at a time, the worst; another may remain above the target once that one is fixed)
    fill = []
    for _ in range(3):
        need = SM.pick_fill_in(cur.est.points())
        if not need:
            break
        for p in need:
            tape.append((p["landmark"], p["anchor"], measure(p)))
            used.add(p["key"])
            fill.append(dict(point=p["key"], sigma_cm=round(p["sigma"], 3)))
        cur = PL.solve_site(user, OPEN, runs, site.sightings, robots, opts=dict(cam_sys_cm=args.cam_sys),
                            robot_priors=pri, tape=tape, click_sigma_cm=args.click_sd)
    if fill:
        log(f"  fill-in by tape (sd above {SM.TARGET_SD_CM} cm): " + ", ".join(f"{f['point']} ({f['sigma_cm']:.2f})" for f in fill))
    for k in range(max_rounds):
        pts = cur.est.points()
        pick = SM.pick_audit(pts, seed=seed * 100 + k, exclude=used)
        rec = []
        for p in pick:
            meas = measure(p)
            c = SM.audit_compare(p, meas)
            rec.append(dict(point=p["key"], why=p["why"], est=[round(float(v), 2) for v in p["xy"]],
                            sigma_cm=round(p["sigma"], 3), tape=[round(float(v), 2) for v in meas],
                            error_cm=round(c["error_cm"], 3), allowed_cm=round(c["allowed_cm"], 3), passed=c["passed"]))
            tape.append((p["landmark"], p["anchor"], meas))
            used.add(p["key"])
        ok = all(r["passed"] for r in rec)
        rounds.append(dict(round=k + 1, points=rec, passed=ok))
        log(f"  audit round {k + 1}: " + ", ".join(f"{r['point']} {r['error_cm']:.2f}/{r['allowed_cm']:.2f} cm"
                                                     for r in rec) + (" PASS" if ok else " FAIL"))
        if ok:
            break
        # re-survey the failing landmarks (every robot, their spots only), then solve again with the tape merged
        bad = {p["landmark"] for p, r in zip(pick, rec) if not r["passed"] and p["kind"] != "edge"}
        if bad:
            sub = copy.deepcopy(FM.normalize(user))
            PL.run_sim_survey(site, sub, robots, [], log=log, do_edges=False,
                              only_landmarks=bad)
        runs = [run for r in robots for run in site.runs(r) if run.experiment == "follow"]
        cur = PL.solve_site(user, OPEN, runs, site.sightings, robots, opts=dict(cam_sys_cm=args.cam_sys),
                            robot_priors=pri, tape=tape, click_sigma_cm=args.click_sd)
    final = PL.solve_site(user, OPEN, runs, site.sightings, robots, opts=dict(cam_sys_cm=args.cam_sys),
                          robot_priors=pri, tape=tape, click_sigma_cm=args.click_sd)
    return dict(rounds=rounds, final=final, robot_s=site.robot_time_s - t0, fill=fill, audited=cur)


def summarize(results):
    rows = []
    keys = ["a", "b", "c0", "c", "c_audited", "c_tape"]
    names = sorted({k for r in results for k in r["a"]})
    for k in keys:
        for n in names:
            v = [r[k][n] for r in results if n in r[k]]
            rows.append(dict(case=k, item=n, mean_cm=round(stats.mean(v), 3), max_cm=round(max(v), 3), n=len(v)))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--config", help="simulator config (default: the fusion's r7/r11 sensor errors)")
    ap.add_argument("--click-sd", type=float, default=1.0, help="click error per board end, cm (x and y each)")
    ap.add_argument("--cam-sys", type=float, default=0.15, help="camera calibration error field, rms cm per axis")
    ap.add_argument("--keep", action="store_true", help="save each seed's logs (pickle) in --out")
    ap.add_argument("--fusion-files", action="store_true",
                    help="also write the first seed's locked map and site calibration made with no camera "
                         "calibration error (for tools/fusion/replay.py on the r7/r11 sessions)")
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    results = []
    for s in args.seeds:
        print(f"seed {s}")
        r = one_seed(s, args)
        for k in ("a", "b", "c0", "c", "c_audited", "c_tape"):
            print(f"  {k:9s} " + "  ".join(f"{n} {v:.2f}" for n, v in r[k].items()))
        results.append(r)
    clean = [{k: v for k, v in r.items() if not k.startswith("_")} for r in results]
    rows = summarize(clean)
    with open(os.path.join(args.out, "site_eval.json"), "w") as f:
        json.dump(dict(results=clean, summary=rows, args=vars(args)), f, indent=1, default=float)
    with open(os.path.join(args.out, "site_eval.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print("\nmean over seeds and landmarks (cm): " + ", ".join(
        f"{k} {stats.mean([r['mean_cm'] for r in rows if r['case'] == k and not r['item'].startswith('edge')]):.2f}"
        for k in ("a", "b", "c0", "c", "c_audited", "c_tape")))
    if args.fusion_files:
        a2 = argparse.Namespace(**vars(args))
        a2.cam_sys = 0.0
        r = one_seed(args.seeds[0], a2, log=lambda *_: None)
        res = r["_res"]
        folder = os.path.join(args.out, "fusion_map")
        m = SM.locked_map(FM.normalize(r["_user"]), res.est, 1, r["audit"], SM.camera_info(None),
                          dict(simulated=True, seed=args.seeds[0]))
        SM.write_lock(folder, m, res.est)
        cal = SM.tof_calibration_from(res.est, res.robots_out, res.noise, [run.run_id for run in r["_runs"]],
                                      map_given=os.path.join(folder, "map.json"))
        with open(os.path.join(folder, "tof_calibration.json"), "w") as f:
            json.dump(cal, f, indent=1)
        print(f"fusion files in {folder}: map errors " + ", ".join(f"{k} {v:.2f}" for k, v in r["c_tape"].items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
