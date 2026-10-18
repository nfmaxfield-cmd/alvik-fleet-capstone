#!/usr/bin/env python3
"""Site setup: learn where everything on the floor is (boards, blocks, the open
plywood edges) by merging the robots' sensors, the camera and your input, check
it with a tape measure, and lock the map.

Run it when the floor is new or something on it changed. (The per-robot
calibration run is different: it runs once after each power-on and calibrates
the robot itself.)

    python tools/site/setup.py --site site --map map.json --calib <recording>/floor_calibration.json
    python tools/site/setup.py --site site --refresh B        # one landmark moved: survey just that one
    python tools/site/setup.py --site site_sim --sim --auto   # the whole thing on the simulator, no questions

Steps (each says what it needs from you, in plain words; tools/site/README.md
has the same steps with pictures in words):

  1. camera check      is the camera calibration there and good enough?
  2. landmarks         which boards and blocks are on the floor (you click them on the
                       camera page, or accept what the map / camera found) and which
                       sides of the plywood have no border board
  3. survey plan       where each robot will stand to measure each landmark from
                       several distances and angles, and where it creeps up to the
                       open edges with the edge guard on
  4. survey            each robot drives its plan (tools/fleet/run_plan.py), the
                       others off the board; the camera records
  5. solve             everything measured, merged in one fit: each landmark's
                       position and how sure it is
  6. tape              points the robots could not pin down: you measure them
  7. audit             2-3 points chosen by the program: you measure them; pass
                       or fail (and why)
  8. lock              map.json + landmarks.csv (+ the fusion's calibration file)

State is kept in <site>/site_state.json, so you can stop and start again.
"""
import argparse
import csv
import json
import os
import random
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "fusion"), os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fleetmap as FM  # noqa: E402
import pipeline as PL  # noqa: E402
import sitemap as SM  # noqa: E402
import survey as SV  # noqa: E402

CAMERA_LOO_OK_CM = 0.35        # floor calibration's left-one-out error that is good enough (B1/B2 checks)


# ---------------------------------------------------------------- talking to the user
class Ask:
    """Questions in plain words. auto: take the default (simulation)."""

    def __init__(self, auto=False):
        self.auto = auto

    def say(self, text=""):
        print(text, flush=True)

    def step(self, n, title):
        print(f"\n=== Step {n}: {title} ===", flush=True)

    def yes(self, q, default=True):
        if self.auto:
            print(f"{q} [{'Y/n' if default else 'y/N'}] (auto: {'yes' if default else 'no'})")
            return default
        a = input(f"{q} [{'Y/n' if default else 'y/N'}] ").strip().lower()
        return default if not a else a.startswith("y")

    def text(self, q, default=""):
        if self.auto:
            print(f"{q} (auto: {default})")
            return default
        a = input(f"{q} " + (f"[{default}] " if default else "")).strip()
        return a or default

    def enter(self, q):
        if self.auto:
            print(f"{q} (auto)")
            return
        input(f"{q} Press Enter when done. ")

    def tape(self, point, hint, simulate=None):
        """A tape measurement of a floor point, as x, y in inches (or cm)."""
        if self.auto and simulate is not None:
            xy = simulate()
            print(f"  tape at {point}: {SM.to_in(xy[0])}, {SM.to_in(xy[1])} (simulated)")
            return xy
        import landmark_pick as LP
        while True:
            a = input(f"  {point}\n    {hint}\n    Type x, y in inches (e.g. 36 3/32, 12 1/2) or add cm at the end: ").strip()
            try:
                return np.array(LP.parse_tape(a), float)
            except ValueError as e:
                print(f"    {e}. Try again.")


# ---------------------------------------------------------------- state
def load_state(site):
    p = os.path.join(site, "site_state.json")
    if os.path.exists(p):
        with open(p) as f:
            return json.load(f)
    return {}


def save_state(site, state):
    p = os.path.join(site, "site_state.json")
    with open(p + ".tmp", "w") as f:
        json.dump(state, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    os.replace(p + ".tmp", p)


# ---------------------------------------------------------------- step 1: camera
def camera_check(ask, calib, site):
    ask.step(1, "camera check")
    info = SM.camera_info(calib)
    if not info.get("path"):
        ask.say("No camera calibration given (--calib). The map will be only as good as the robots' own wheels "
                "between camera sightings, which is not good enough. Make one first:\n"
                "  python tools/calibrate_floor.py <recording folder>")
        return info, False
    loo = info.get("loo_rms_cm") or info.get("rms_cm")
    ok = loo is not None and loo <= CAMERA_LOO_OK_CM
    ask.say(f"Camera calibration: {info['path']}\n  fit error {loo if loo is not None else '?'} cm "
            f"(good enough: {CAMERA_LOO_OK_CM} cm or less) -> {'OK' if ok else 'NOT GOOD ENOUGH'}")
    _, old = SM.read_lock(site) if os.path.exists(os.path.join(site, "map.json")) else (None, None)
    if old and (old.get("camera_calibration") or {}).get("hash") not in (None, info.get("hash")):
        ask.say("  The camera calibration changed since the map was last locked: every landmark is measured again.")
    if not ok:
        ask.say("  Redo the camera calibration (python tools/calibrate_floor.py <recording folder>) and run this "
                "again; or carry on, and the map's uncertainty will say how much that costs.")
    return info, ok


# ---------------------------------------------------------------- step 2: landmarks and open edges
def landmarks_step(ask, m, state, recording=None, calib=None, detections=None):
    ask.step(2, "landmarks and open edges")
    m = FM.normalize(m)
    if recording and not ask.auto:
        ask.say("The camera page opens next. In its Landmarks mode, click both ends of every board (where it meets "
                "the floor) and the corners of every block; fix or delete anything that is wrong; press "
                "\"Save landmarks\".")
        cmd = [sys.executable, os.path.join(TOOLS, "calibrate_floor.py"), recording, "--map", state["map_path"]]
        if ask.yes("Open the camera page now?"):
            proc = subprocess.Popen(cmd)
            ask.enter("Click the landmarks on the page and press Save landmarks.")
            proc.terminate()
            m = FM.load(state["map_path"])
    if detections and calib:
        cands = detector_candidates(detections, calib)
        if cands:
            ask.say(f"The camera's detector found {len(cands)} possible board(s) or block(s) not in the map:")
            for c in cands:
                ask.say(f"  near x {c[0]:.0f} cm, y {c[1]:.0f} cm ({SM.to_in(c[0])}, {SM.to_in(c[1])})")
            ask.say("  If any is a real board, add it on the camera page (step above) and run this step again.")
    names = [b["name"] for b in m["boards"]] + [o["id"] for o in m["obstacles"]
                                                 if o["shape"] == "polygon" and o.get("source") == "user"]
    if not names:
        raise SystemExit("No boards or blocks in the map: add them on the camera page first.")
    keep = []
    for n in names:
        if ask.yes(f"  Landmark {n}: is it on the floor now, where the page shows it?"):
            keep.append(n)
    ask.say("Which sides of the plywood have NO border board (open: the cement is right there)?")
    for s in SM.SIDES:
        ask.say(f"  {s}: {SM.SIDE_WORDS[s]}")
    default = ",".join(state.get("open_edges") or [s for s in SM.SIDES if s in (m.get("site_open_edges") or [])])
    raw = ask.text("Type the open sides, separated by commas (e.g. x0,y1), or none:", default or "none")
    open_edges = [s.strip() for s in raw.split(",") if s.strip() in SM.SIDES]
    return m, keep, open_edges


def detector_candidates(det_csv, calib, cls="machine", min_score=0.5):
    """Floor positions of detector 'machine' sightings (median per track), as candidates to accept."""
    import runs as RN
    _, Hinv = RN.load_homography(calib)
    pts = {}
    with open(det_csv, newline="") as f:
        for r in csv.DictReader(f):
            if (r.get("cls") or r.get("class") or "") != cls:
                continue
            try:
                if float(r.get("score") or 1) < min_score:
                    continue
                x, y = RN.apply_h(Hinv, float(r["x_px"]), float(r["y_px"]))
            except (KeyError, ValueError):
                continue
            pts.setdefault(r.get("obj") or r.get("track") or "0", []).append((x, y))
    return [tuple(np.median(np.array(v), 0)) for v in pts.values() if len(v) >= 5]


# ---------------------------------------------------------------- step 3/4: survey
def plan_survey(ask, m, robots, open_edges, opts, site, only=None):
    ask.step(3, "survey plan")
    L = SM.landmarks_from_map(m, include=only)
    E = SM.edges_from_map(m, open_edges)
    plans = {}
    total = 0.0
    for rn in robots:
        start = next(r["start"] for r in m["robots"] if r["name"] == rn)
        spots = PL.views_for(m, rn, L, opts)
        spots += SV.corner_viewpoints(SV.survey_map(m, rn, others_parked=False), L, opts=opts)
        spots = SV.order_greedy(start, spots)
        edges = SV.edge_targets(SV.survey_map(m, rn, others_parked=False), E, opts) if not only else []
        plans[rn] = dict(start=start, spots=spots, edges=edges)
        est_s = 14.0 * sum(1 for s in spots if s.get("kind") != "corner") + 40.0 * sum(
            1 for s in spots if s.get("kind") == "corner") + 45.0 * len(edges)
        total += est_s
        ask.say(f"  {rn}: {len(spots)} spots ({sum(1 for s in spots if s.get('kind') == 'corner')} corner sweeps), "
                f"{len(edges)} edge approaches, about {est_s / 60:.0f} minutes")
    ask.say(f"Robot time about {total / 60:.0f} minutes in all (one robot at a time).")
    with open(os.path.join(site, "survey_plan.json"), "w") as f:
        json.dump(plans, f, indent=1)
    return plans


def run_real_survey(ask, m, plans, args, site):
    """Each robot's plan on the real robots through tools/fleet/run_plan.py. Returns the session folders."""
    ask.step(4, "survey (real robots)")
    ask.say("Start the camera recording on the Pi now (python3 camrec.py --label site) and leave it running "
            "until the survey is done.")
    ask.enter("Recording started?")
    sessions = []
    for rn, pl in plans.items():
        ask.say(f"\nRobot {rn}: take the other robots off the board. Put {rn} on its start spot "
                f"(x {pl['start'][0]:.1f} cm, y {pl['start'][1]:.1f} cm, facing {pl['start'][2]:.0f} deg).")
        ask.enter(f"{rn} on its spot, the others off the board?")
        for k, seg in enumerate(SV.split_spots(pl["spots"], args.segment_spots)):
            plan, kept, sm = SV.plan_views(m, rn, pl["start"], seg, PL_OPTS(args))
            sessions.append(_run_plan(sm, plan, site, f"{rn}_views{k}", args))
            if args.segment_spots and k < len(SV.split_spots(pl["spots"], args.segment_spots)) - 1 and args.fixes == "none":
                ask.enter(f"{rn} is back at its start spot. If it is more than a finger's width off the mark, "
                          "put it back exactly on it.")
        if pl["edges"]:
            ask.say(f"{rn}: the open edges next. It creeps up to each edge with the edge guard on and stops there.")
            here = pl["start"]
            # each approach ends where the guard stopped it: the next run starts from the robot's own pose there
            # (its last FOLLOW record), backs off in a run of its own (the guard learns the plywood at the start
            # of each run), then plans the next approach from there
            for k, tg in enumerate(pl["edges"]):
                if k:
                    back = SV.plan_back(here)
                    sm = SV.survey_map(m, rn, others_parked=False)
                    sm["robots"][0]["start"] = list(here)
                    f = _run_plan(sm, SV.plan_file(sm, rn, back, here), site, f"{rn}_back{k}", args, allow_off=True)
                    sessions.append(f)
                    here = last_pose(f, rn, back[-1]["to"])
                steps = SV.plan_edge(m, rn, here, tg)
                sm = SV.survey_map(m, rn, others_parked=False)
                sm["robots"][0]["start"] = list(here)
                f = _run_plan(sm, SV.plan_file(sm, rn, steps, here), site, f"{rn}_edge{k}", args,
                              send=["set safe_edge 1"], allow_off=True)
                sessions.append(f)
                here = last_pose(f, rn, steps[-1]["to"])
            back = SV.plan_back(here)
            sm = SV.survey_map(m, rn, others_parked=False)
            sm["robots"][0]["start"] = list(here)
            sessions.append(_run_plan(sm, SV.plan_file(sm, rn, back, here), site, f"{rn}_back_last", args,
                                      send=["set safe_edge 0"], allow_off=True))
    ask.enter("Survey done. Stop the camera recording.")
    return sessions


def PL_OPTS(args):
    """Survey options (tools/site/survey.py DEFAULTS); none are exposed on the command line yet."""
    return {}


def last_pose(folder, robot, default):
    """The robot's own pose at the end of a run_plan.py session (its last FOLLOW record), or default."""
    p = os.path.join(folder, "follow.csv")
    last = None
    if os.path.exists(p):
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                if r.get("robot") == robot and (r.get("state") or "") != "nopose":
                    last = r
    if not last:
        return list(default)
    try:
        return [float(last["x_cm"]), float(last["y_cm"]), float(last["th_deg"])]
    except (KeyError, ValueError):
        return list(default)


def _run_plan(sm, plan, site, label, args, send=(), allow_off=False):
    folder = os.path.join(site, "sessions", label)
    os.makedirs(folder, exist_ok=True)
    mp, pp = os.path.join(folder, "survey_map.json"), os.path.join(folder, "plan.json")
    with open(mp, "w") as f:
        json.dump(FM.normalize(sm), f, indent=1)
    plan["map_hash"] = FM.map_hash(sm)
    with open(pp, "w") as f:
        json.dump(plan, f, indent=1)
    cmd = [sys.executable, os.path.join(TOOLS, "fleet", "run_plan.py"), "--map", mp, "--plan", pp, "--session", folder,
           "--fixes", args.fixes, "--yes"]
    for c in send:
        cmd += ["--send", c]
    if args.dry_run:
        cmd += ["--dry-run"]
    print("  " + " ".join(os.path.relpath(c, REPO) if os.path.isabs(c) else c for c in cmd[1:]), flush=True)
    r = subprocess.run(cmd)
    if r.returncode and not allow_off:
        print(f"  (run_plan.py ended with code {r.returncode}: see {folder})")
    return folder


def collect_real(sessions, detections, calib, recording, robots, video_offset=None):
    """Runs and camera sightings of a real survey (step 5's input)."""
    import check_session as CS
    import runs as RN
    runs, fits0 = [], None
    for s in sessions:
        rr, fits = RN.load_runs(s)
        fits0 = fits0 or fits
        edge = CS.rows(s, "edge")
        for run in rr:
            if run.experiment != "follow":
                continue
            fit = CS.fit_for(fits, run.robot, run.run_id)
            run.edge = []
            for e in edge:
                if e.get("run") == run.run_id:
                    e = dict(e)
                    e["t"] = CS.to_laptop(fit, float(e["ms"])) if fit else float(e.get("t_recv") or 0)
                    run.edge.append(e)
            runs.append(run)
    vc = RN.video_clock(sessions[0], recording, fits0, video_offset)
    if vc is None:
        raise SystemExit("Could not line up the video and robot clocks: give --video-offset or a recording with "
                         "video_clock.json (python tools/check_session.py <session> --video <recording>).")
    clock, how = vc
    print(how)
    _, Hinv = RN.load_homography(calib)
    dets = RN.load_detections(detections, Hinv, clock, parallax=RN.load_parallax(calib), dist=RN.load_distortion(calib))
    # one robot on the board at a time: a sighting belongs to the robot whose run is going then
    spans = [(r.t_start, r.t_end, r.robot) for r in runs if r.raw]
    sight = {r: [] for r in robots}
    for d in dets:
        for a, b, rn in spans:
            if a - 1 <= d["t"] <= b + 2:
                sight.setdefault(rn, []).append((d["t"], d["x"], d["y"], d.get("heading")))
                break
    return runs, sight


# ---------------------------------------------------------------- steps 5-8
def show_estimate(ask, est):
    ask.say("\nWhere everything is (estimate and how sure, 1 standard deviation):")
    for name, l in est.landmarks.items():
        s = est.landmark_sigma(name)
        c = l.move(l.center, est.g[name])
        ask.say(f"  {l.kind} {name}: centre x {c[0]:.2f}, y {c[1]:.2f} cm; sure to {SM.cm_and_in(s)}")
    for side, e in est.edges.items():
        if e.open:
            p, _ = e.line(est.e[side])
            v = p[0] if side[0] == "x" else p[1]
            ask.say(f"  open edge {side}: at {side[0]} = {v:.2f} cm; sure to {SM.cm_and_in(est.edge_point_sigma(side, 0.5))}")


def point_hint(p):
    if p["kind"] == "edge":
        side = p["landmark"].split()[1]
        axis = "x" if side[0] == "x" else "y"
        other = "y" if axis == "x" else "x"
        return (f"the plywood edge {side} at {other} = {p['xy'][1 if axis == 'x' else 0]:.1f} cm "
                f"({SM.to_in(p['xy'][1 if axis == 'x' else 0])}): measure its {axis} (type both numbers)")
    return (f"the floor corner {p['anchor']} of {p['landmark']}, near x {p['xy'][0]:.1f}, y {p['xy'][1]:.1f} cm "
            f"({SM.to_in(p['xy'][0])}, {SM.to_in(p['xy'][1])}): measure from the axes' origin")


def tape_and_audit(ask, solve_fn, res, simulate_tape=None, max_rounds=3, resurvey=None, seed=None, n_worst=2):
    """Steps 6 and 7. Returns (final result, audit records, fill-in records, tape list)."""
    ask.step(6, "tape measurements the robots could not do")
    tape, used, fill = [], set(), []
    cur = res
    for _ in range(3):
        need = SM.pick_fill_in(cur.est.points())
        if not need:
            break
        for p in need:
            ask.say(f"  {p['key']}: the robots left it sure only to {SM.cm_and_in(p['sigma'])}.")
            xy = ask.tape(p["key"], point_hint(p), (lambda p=p: simulate_tape(p)) if simulate_tape else None)
            tape.append((p["landmark"], p["anchor"], xy))
            used.add(p["key"])
            fill.append(dict(point=p["key"], sigma_cm=round(p["sigma"], 3), tape=[round(float(v), 2) for v in xy]))
        cur = solve_fn(tape)
    if not fill:
        ask.say("  None needed: every point is sure to 0.5 cm or better.")
    ask.step(7, "audit")
    ask.say(f"Pass rule: every audited point within max({SM.AUDIT_FLOOR_CM} cm, {SM.AUDIT_K:.0f} x its standard "
            "deviation, map and tape together) of where the map puts it.")
    rounds = []
    for k in range(max_rounds):
        pick = SM.pick_audit(cur.est.points(), n_worst=n_worst,
                             seed=seed if seed is not None else random.randrange(1 << 30), exclude=used)
        rec = []
        for p in pick:
            ask.say(f"  {p['key']} ({p['why']}): map says x {p['xy'][0]:.2f}, y {p['xy'][1]:.2f} cm, "
                    f"sure to {SM.cm_and_in(p['sigma'])}")
            xy = ask.tape(p["key"], point_hint(p), (lambda p=p: simulate_tape(p)) if simulate_tape else None)
            c = SM.audit_compare(p, xy)
            rec.append(dict(point=p["key"], why=p["why"], est=[round(float(v), 3) for v in p["xy"]],
                            sigma_cm=round(p["sigma"], 3), tape=[round(float(v), 3) for v in xy],
                            error_cm=round(c["error_cm"], 3), allowed_cm=round(c["allowed_cm"], 3), passed=c["passed"]))
            ask.say(f"    off by {SM.cm_and_in(c['error_cm'])}, allowed {SM.cm_and_in(c['allowed_cm'])}: "
                    f"{'PASS' if c['passed'] else 'FAIL'}")
            tape.append((p["landmark"], p["anchor"], xy))
            used.add(p["key"])
        ok = all(r["passed"] for r in rec)
        rounds.append(dict(round=k + 1, points=rec, passed=ok))
        if ok:
            ask.say("Audit passed.")
            break
        bad = sorted({p["landmark"] for p, r in zip(pick, rec) if not r["passed"] and p["kind"] != "edge"})
        ask.say("Audit failed for " + ", ".join(r["point"] for r in rec if not r["passed"])
                + ". Those landmarks are measured again by the robots, with your tape readings added.")
        if resurvey and bad:
            resurvey(bad)
        cur = solve_fn(tape)
    final = solve_fn(tape)
    return final, rounds, fill, tape, cur


def lock(ask, site, base_map, res, rounds, fill, camera, survey_info, runs):
    ask.step(8, "lock")
    if not rounds or not rounds[-1]["passed"]:
        ask.say("The audit did not pass: the map is NOT locked. Check the failing landmarks (moved during the survey? "
                "a hand-measurement slip?) and run the setup again.")
        return None
    v = SM.next_version(site)
    m = SM.locked_map(FM.normalize(base_map), res.est, v, rounds, camera, survey_info, robots=res.robots_out,
                      sensor=dict(zone_width_deg=res.noise.get("width", 0.0)), fill_in=fill)
    path = SM.write_lock(site, m, res.est)
    cal = SM.tof_calibration_from(res.est, res.robots_out, res.noise, [r.run_id for r in runs],
                                  map_given=os.path.abspath(path))
    with open(os.path.join(site, "tof_calibration.json"), "w") as f:
        json.dump(cal, f, indent=1)
    with open(os.path.join(site, "audit.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["round", "point", "why", "map_x_cm", "map_y_cm", "sigma_cm", "tape_x_cm", "tape_y_cm", "error_cm",
                    "allowed_cm", "passed"])
        for r in rounds:
            for p in r["points"]:
                w.writerow([r["round"], p["point"], p["why"], *p["est"], p["sigma_cm"], *p["tape"], p["error_cm"],
                            p["allowed_cm"], int(p["passed"])])
    ask.say(f"Locked: version {v}, {path}\n  landmarks.csv, tof_calibration.json (for tools/fusion/replay.py "
            f"--tof-cal) and audit.csv beside it.\n  Use it: run_plan.py --map {path} --site-map {path}")
    return path


# ---------------------------------------------------------------- the whole thing
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", required=True, help="folder for this site's files (state, sessions, the locked map)")
    ap.add_argument("--map", help="fleet map.json to start from (default: <site>/map.json, else the simulator world "
                                  "with --sim)")
    ap.add_argument("--calib", help="floor_calibration.json (the camera calibration)")
    ap.add_argument("--recording", help="camera recording folder of the survey (detections, video clock)")
    ap.add_argument("--detections", help="camera detections CSV of the survey recording (tools/detector)")
    ap.add_argument("--video-offset", type=float, help="video clock minus laptop clock, s")
    ap.add_argument("--robots", help="robots to survey with, comma-separated (default: every robot in the map)")
    ap.add_argument("--tof-cal", help="tof_calibration.json from the per-robot calibration runs (zone directions)")
    ap.add_argument("--bench", help="JSON {robot: {bias_cm, bias_sd_cm, scale, scale_sd}} from E0c's C9 rows")
    ap.add_argument("--click-sd", type=float, default=1.0, help="how far a click may be off, cm (default 1)")
    ap.add_argument("--fixes", default="none", help="run_plan.py position source: none (default) or udp[:PORT]")
    ap.add_argument("--segment-spots", type=int, default=0,
                    help="spots per run; without live camera fixes keep runs short (e.g. 6) and put the robot back "
                         "on its spot between them")
    ap.add_argument("--audit-every-landmark", action="store_true",
                    help="audit the least-sure point of EVERY landmark and edge (plus one random), not just the 2 "
                         "least sure: about 1.5 minutes of tape work per landmark, catches a single bad landmark")
    ap.add_argument("--refresh", help="landmark name(s), comma-separated: survey only these again")
    ap.add_argument("--dry-run", action="store_true", help="real-robot steps against simulated robots in real time "
                                                          "(run_plan.py --dry-run)")
    ap.add_argument("--sim", action="store_true", help="the whole setup on the simulator (fast, not real time)")
    ap.add_argument("--auto", action="store_true", help="take the default answer to every question (with --sim: "
                                                       "simulated clicks and tape)")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args(argv)
    ask = Ask(args.auto)
    os.makedirs(args.site, exist_ok=True)
    state = load_state(args.site)
    if args.sim:
        return run_sim(ask, args, state)
    map_path = args.map or os.path.join(args.site, "map.json")
    if not os.path.exists(map_path):
        raise SystemExit(f"No map at {map_path}: draw the floor in tools/fleet/app.py first (or give --map).")
    state["map_path"] = os.path.abspath(map_path)
    camera, _ = camera_check(ask, args.calib, args.site)
    m, keep, open_edges = landmarks_step(ask, json.load(open(map_path)), state, args.recording, args.calib,
                                         args.detections)
    state.update(open_edges=open_edges, landmarks=keep)
    save_state(args.site, state)
    with open(os.path.join(args.site, "map_clicked.json"), "w") as f:    # the map before the survey (E0d scores it)
        json.dump(FM.normalize(m), f, indent=1)
    robots = args.robots.split(",") if args.robots else [r["name"] for r in m["robots"]]
    only = args.refresh.split(",") if args.refresh else None
    if state.get("sessions") and args.detections and ask.yes(
            f"A survey was already driven ({len(state['sessions'])} runs saved in {args.site}/sessions). "
            "Use it (no new driving)?"):
        pass
    else:
        plans = plan_survey(ask, m, robots, open_edges, {}, args.site, only=only)
        sessions = run_real_survey(ask, m, plans, args, args.site)
        state["sessions"] = state.get("sessions", []) + sessions
        save_state(args.site, state)
    if not (args.detections and args.calib):
        ask.say("\nNext: run the detector on the survey recording, then run this again with --detections and "
                "--calib (and --recording) to solve, check and lock the map.")
        return 0
    ask.step(5, "solve")
    runs, sight = collect_real(state["sessions"], args.detections, args.calib, args.recording, robots,
                               args.video_offset)
    cal = json.load(open(args.tof_cal)) if args.tof_cal else None
    bench = json.load(open(args.bench)) if args.bench else None
    pri = PL.robot_priors_from_cal(cal, bench)
    cam_sys = (camera.get("loo_rms_cm") or camera.get("rms_cm") or 0.15)

    def solve_fn(tape):
        return PL.solve_site(m, open_edges, runs, sight, robots, opts=dict(cam_sys_cm=cam_sys), robot_priors=pri,
                             tape=tape, click_sigma_cm=args.click_sd, verbose=True)
    res = solve_fn([])
    show_estimate(ask, res.est)
    final, rounds, fill, tape, _ = tape_and_audit(ask, solve_fn, res, n_worst=99 if args.audit_every_landmark else 2)
    lock(ask, args.site, m, final, rounds, fill, camera, dict(sessions=state["sessions"], robots=robots), runs)
    return 0


def run_sim(ask, args, state):
    """The whole setup on the simulator: the true world is tools/site/examples/sim_site_world.json, the clicks
    and tape readings are simulated from it."""
    import sim_eval as SE
    import simrun as SR
    world = json.load(open(args.map or SE.WORLD))
    tm = FM.normalize(world)
    camera = dict(path=None, hash=None, note="simulated camera (0.3 cm, 3 deg per sighting; 0.15 cm calibration error)")
    ask.step(1, "camera check (simulated)")
    ask.say("Simulated camera: calibration error 0.15 cm rms (an assumption: the real one comes from its fit).")
    user = PL.clicked_map(world, args.click_sd, seed=args.seed)
    with open(os.path.join(args.site, "map_clicked.json"), "w") as f:
        json.dump(user, f, indent=1)
    ask.step(2, "landmarks and open edges (simulated clicks, 1 cm per board end)")
    open_edges = world.get("site_open_edges") or ["x0", "y1"]
    names = [b["name"] for b in tm["boards"]]
    ask.say("Landmarks: " + ", ".join(names) + "; open sides: " + ", ".join(open_edges))
    robots = args.robots.split(",") if args.robots else [r["name"] for r in tm["robots"]]
    cfg = SE.sim_config(None)
    site = SR.SimSite(tm, cfg, seed=args.seed, camera=SR.CameraModel(args.seed), board_cm=[0, 0, 122, 122],
                      open_edges=open_edges)
    starts = {r["name"]: r["start"] for r in tm["robots"]}
    for r in robots:                     # the per-robot calibration run after power-on (not part of the setup)
        site.calibrate(r, starts[r])
    cal_runs = [run for r in robots for run in site.runs(r) if run.experiment == "calib"]
    _, cal_b = SE.case_b(user, cal_runs, site.sightings)
    plan_survey(ask, FM.normalize(user), robots, open_edges, {}, args.site)
    ask.step(4, "survey (simulated robots)")
    E = SM.edges_from_map(FM.normalize(user), open_edges, prior_sigma_cm=args.click_sd)
    PL.run_sim_survey(site, user, robots, E, log=ask.say)
    runs = [run for r in robots for run in site.runs(r) if run.experiment == "follow"]
    pri = PL.robot_priors_from_cal(cal_b, SE.bench_priors(cfg, robots, args.seed))
    ask.step(5, "solve")

    def solve_fn(tape):
        return PL.solve_site(user, open_edges, runs, site.sightings, robots, opts=dict(cam_sys_cm=0.15),
                             robot_priors=pri, tape=tape, click_sigma_cm=args.click_sd, verbose=True)
    res = solve_fn([])
    show_estimate(ask, res.est)
    truth = {l.name: l for l in SM.landmarks_from_map(tm)}
    rng = np.random.default_rng(args.seed + 7)

    def sim_tape(p):
        if p["kind"] == "edge":
            q = SM.Edge(p["landmark"].split()[1], tm["floor"], True).point(p["anchor"], np.zeros(2))
        else:
            q = truth[p["landmark"]].anchors[p["anchor"]]
        return q + rng.normal(0, SM.AUDIT_TAPE_SD_CM, 2)

    def resurvey(bad):
        PL.run_sim_survey(site, user, robots, [], log=ask.say, do_edges=False, only_landmarks=bad)
        runs[:] = [run for r in robots for run in site.runs(r) if run.experiment == "follow"]
    final, rounds, fill, tape, _ = tape_and_audit(ask, solve_fn, res, simulate_tape=sim_tape, resurvey=resurvey,
                                                  seed=args.seed)
    errs = PL.map_errors(final.est, world)
    ask.say("\n(Simulation only) true error of the locked map: " + ", ".join(f"{k} {v:.2f} cm" for k, v in errs.items()))
    lock(ask, args.site, user, final, rounds, fill, camera, dict(simulated=True, seed=args.seed, robots=robots), runs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
