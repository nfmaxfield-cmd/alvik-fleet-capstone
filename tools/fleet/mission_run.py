#!/usr/bin/env python3
"""Run a mission from one command, with or without Mission Control (one system, 2026-10-28; Nick 18:36: "also runnable
from one command without the page", the safety net for Monday). Mission Control's Run screen starts this too, so the
page and the command are the same run.

    python3 tools/fleet/mission_run.py --table 61x122                        # Monday's default on the robots (Pi)
    python3 tools/fleet/mission_run.py --map logs/mc/map.json --program logs/mc/program.json
    python3 tools/fleet/mission_run.py --table 70x130 --dry                  # the simulator, both simulated cameras
    python3 tools/fleet/mission_run.py --table 61x122 --dry --washout --drop-pct 5 --dead Right@60

What it does, in order (each step logged in the session folder, logs/<date-time>-mission[-dry]/):
  1. the workspace: --map (Mission Control's, with the warm-up's sides and zones) or a blank table of --table size
     (every side "not felt yet": kept away from like an open edge);
  2. the mission: --program (blocks) or Monday's default for --robots (missions.monday_program), built for this table
     and planned (refused if the plan's closest approach is under 8 cm);
  3. the cameras: the camera bridge (cam_bridge.py; --farm: each camera tracked in its own process) on the recorder's
     pictures (real) or the simulator's two cameras (dry);
  4. the run: run_plan.py with every engine check (battery, restarts, settings read back, silent robots, the 8 cm net),
     re-planned from where the camera has the robots at the start;
  5. the record (item 6): run_plan's logs (both cameras' positions, every move and its timing, battery, messages),
     the Pi's temperature and throttling (cpu.csv, perf.csv: the heat tracker), the pictures at the tracking rate and
     a clip 5 s either side of every stop (frames/), the line check, mission_summary.json; --upload: a release with
     a tag of its own (upload_session.py --unique-tag).
Ctrl+C stops every robot (run_plan's own stop).
"""
import argparse
import datetime as dt
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import fleetmap as FM        # noqa: E402
import missions as MS        # noqa: E402
import planner as P          # noqa: E402
import workspace as WS       # noqa: E402
import engine_checks as EC   # noqa: E402

PY = sys.executable
EDGE_GUARD = ["set safe_edge 3", "set edge_dir 0", "set edge_thr 150", "set edge_n 3", "set edge_bot_cm 11",
              "set bumper_cm 0"]    # the robots' edge guard, as run-fleet-pi.sh / ladder.SAFE (lost at every power-off)
PLAN_LIMIT_S = 60.0     # planning time allowed for the mission (Monday plans in 4-5 s here). No DOE: a limit
BRIDGE_START_S = 5.0    # the camera bridge learns the empty floor round the robots before run_plan starts (as
                        # run-fleet-pi.sh). No DOE: the bridge needs 5 pictures


def table_map(size, robots):
    """A blank table: 'WxL' or 'LxW' cm (the longer side along x)."""
    a, b = (float(v) for v in size.lower().split("x"))
    L, W = max(a, b), min(a, b)
    return WS.blank_map((0, 0, L, W), [(n, [20 + 30 * k, 15, 90]) for k, n in enumerate(robots)])


def say(s):
    print(time.strftime("%H:%M:%S ") + s, flush=True)


def build(args, out):
    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    if args.map:
        m = FM.load(args.map)
        m["tasks"] = {}
    else:
        m = table_map(args.table, ["Left", "Center", "Right"])
    if args.program:
        with open(args.program) as f:
            prog = json.load(f)
    else:
        prog = MS.monday_program(tuple(robots))
    mm = MS.build(m, prog)
    over = {"separation_cm": max(float(((mm.get("mission") or {}).get("planner") or {}).get("separation_cm", 8.0)),
                                 args.min_gap_cm)}
    t0 = time.time()
    plan = P.make_plan(mm, over=over, t_limit_s=PLAN_LIMIT_S)
    gap = EC.plan_gap(plan, mm, gate_cm=args.min_gap_cm)
    bad = EC.gate_words(gap, args.min_gap_cm)
    mp, pp = os.path.join(out, "mission.json"), os.path.join(out, "plan.json")
    with open(mp, "w") as f:
        json.dump(mm, f, indent=1)
    P.save_plan(plan, pp)
    with open(os.path.join(out, "program.json"), "w") as f:
        json.dump(prog, f, indent=1)
    f_ = mm["floor"]
    say("mission %s on a %.0f x %.0f cm table, robots %s: planned in %.1f s, done in %.0f s, closest %.1f cm"
        % (prog.get("name"), f_["x_max"] - f_["x_min"], f_["y_max"] - f_["y_min"], ", ".join(plan["robots"]),
           time.time() - t0, plan["summary"]["makespan_s"], gap["closest_cm"]))
    for n in mm["mission"].get("notes", []):
        say("  " + n)
    if bad:
        raise SystemExit("Plan refused: %s" % bad)
    return mm, plan, mp, pp


def start_bridge(args, out, mp, cal, scal, stream, side_stream):
    cmd = ["nice", "-n", "5", PY, os.path.join(HERE, "cam_bridge.py"), "--map", mp, "--calib", cal, "--stream", stream,
           "--out", os.path.join(out, "bridge"), "--wait-calib-s", "40"]
    if scal:
        cmd += ["--side-calib", scal, "--side-stream", side_stream]
    if args.farm:
        cmd += ["--farm"]
        if args.rings:
            cmd += ["--overhead-ring", "mirlt-overhead", "--side-ring", "mirlt-side"]
        if args.half:
            cmd += ["--half"]
    f = open(os.path.join(out, "bridge.txt"), "w")
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=REPO), f


def run(args):
    tag = "mission" + ("-dry" if args.dry else "")
    out = os.path.abspath(args.session or os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") +
                                                       "-" + tag))
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "mission_cmd.json"), "w") as f:
        json.dump({"argv": sys.argv, "args": vars(args)}, f, indent=1)
    if args.mission:
        # a mission made elsewhere (Mission Control's warm-up): its map and plan as they are
        mp, pp = os.path.abspath(args.mission), os.path.abspath(args.plan)
    else:
        mm, plan, mp, pp = build(args, out)
    if args.plan_only:
        return 0
    sess = os.path.join(out, "run")
    os.makedirs(sess, exist_ok=True)
    rp = [os.path.join(HERE, "run_plan.py"), "--map", mp, "--plan", pp, "--session", sess, "--fixes", "udp",
          "--poses-to", "127.0.0.1:5011", "--replan-from-camera", "--timeout-s", str(args.timeout_s),
          "--border-cm", str(args.border_cm), "--min-gap-cm", str(args.min_gap_cm),
          "--place-tol-cm", "4"]
    for c in EDGE_GUARD:
        rp += ["--send", c]
    for ip in args.robot:
        rp += ["--robot", ip]
    cond = {}
    if args.dry:
        cal = os.path.join(sess, "sim", "table_camera.json")
        scal = os.path.join(sess, "sim", "side_camera.json") if not args.no_side else ""
        stream, side_stream = "127.0.0.1:8765", "127.0.0.1:8766"
        rp += ["--dry-run", "--yes", "--seed", str(args.seed), "--sim-arg=--fix-hz=0", "--sim-arg=--serve-live=8765",
               "--sim-arg=--live-calib-out=%s" % cal]
        if scal:
            rp += ["--sim-arg=--live-side=8766", "--sim-arg=--live-side-calib-out=%s" % scal]
        if args.washout:
            rp += ["--sim-arg=--live-washout"]
        if args.drop_pct:
            cond["drop_pct"] = args.drop_pct
        if args.dead:
            n, _, t = args.dead.partition("@")
            cond["robot_out"] = {"name": n, "t": float(t or 60), "mode": "dead"}
        if cond:
            cp = os.path.join(out, "sim_cond.json")
            with open(cp, "w") as f:
                json.dump(cond, f)
            rp += ["--sim-arg=--sweep-cond=%s" % cp]
        launcher = [PY, os.path.join(HERE, "sweeps", "rp_shim.py")]     # sim_shim's conditions (drops, a dead robot)
    else:
        import setup_conf as SCF
        cal = args.calib or SCF.overhead_calib(relative=False)
        scal = "" if args.no_side else (args.side_calib or SCF.side_calib() or "")
        stream, side_stream = args.stream, args.side_stream
        launcher = [PY]
    # the record: cpu.csv, frames (tracked pictures and clips)
    cpu = frames = tail = None
    try:
        import e02_cpulog as CL
        cpu = CL.CpuLog(out, procs={"mission": os.getpid()},
                        find={"recorder": "camrec.py", "bridge": "cam_bridge.py", "run_plan": "run_plan.py"})
        cpu.start()
    except Exception as e:     # noqa: BLE001
        say("(cpu log not started: %s)" % e)
    br, brf = start_bridge(args, out, mp, cal, scal, stream, side_stream)
    say("camera bridge starting (it learns the empty floor round the robots: keep hands out of the picture)")
    if not args.no_frames:
        try:
            import doe_loop as DL
            streams = [("overhead", stream, 12.0)] + ([("side", side_stream, 10.0)] if scal else [])
            if args.dry:
                time.sleep(0.1)
            frames = DL.Frames(out, streams)
            tail = DL.Tail(os.path.join(sess, "coordinator.jsonl"), frames)
            tail.start()
            frames.active(True, "mission")
        except Exception as e:     # noqa: BLE001
            say("(frame saving not started: %s)" % e)
    time.sleep(BRIDGE_START_S)
    say("the robots start now. To stop them at any moment: Ctrl+C")
    code = 1
    # (rp_shim.py stands in for run_plan.py on a dry run and takes run_plan's arguments)
    child = subprocess.Popen(launcher + (rp[1:] if args.dry else rp), cwd=REPO)
    try:
        code = child.wait()
    except KeyboardInterrupt:
        say("stopping every robot")
        child.send_signal(signal.SIGINT)
        try:
            code = child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            child.kill()
    finally:
        for p, f in ((br, brf),):
            try:
                p.send_signal(signal.SIGINT)
                p.wait(timeout=10)
            except (subprocess.TimeoutExpired, OSError):
                p.kill()
            f.close()
        if tail is not None:
            tail.stop()
        if frames is not None:
            frames.active(False)
            frames.close()
        if cpu is not None:
            try:
                cpu.stop()
            except Exception:     # noqa: BLE001
                pass
    summ = summarise(out, sess, frames, cond)
    if args.upload and not args.dry:
        up = [PY, os.path.join(TOOLS, "pi", "upload_session.py"), "--recordings", args.recordings, "--session", out,
              "--tag", "mission-" + os.path.basename(out), "--unique-tag", "--logs-only"]
        say("uploading: " + " ".join(up[1:]))
        subprocess.call(up, cwd=REPO)
    say("result: %s; saved in %s" % (summ.get("result"), out))
    return code


def summarise(out, sess, frames, cond):
    """mission_summary.json: what the run did against the plan, the simulator's truth (dry), the line check."""
    s = {"folder": out, "conditions": cond}
    try:
        with open(os.path.join(sess, "run_plan_summary.json")) as f:
            rs = json.load(f)
    except (OSError, ValueError):
        rs = {}
    s.update(result=rs.get("result"), duration_s=rs.get("duration_s"), planned_s=rs.get("planned_makespan_s"),
             replans=len([r for r in rs.get("replans", []) if r.get("source") not in ("improve",)]),
             robots_out=rs.get("robots_out"), plans_refused=rs.get("plans_refused"),
             sensor_stops=len(rs.get("sensor_stops") or []), confirm_checks=rs.get("confirm_checks"),
             battery=rs.get("battery"), start_checks=rs.get("start_checks"))
    sim = rs.get("simulator") or {}
    if sim:
        s["sim"] = {k: sim.get(k) for k in ("min_body_gap_cm", "min_body_gap_at", "collision", "min_open_edge_cm",
                                            "min_open_edge_at", "hung")}
    try:
        import line_check as LC
        tru = os.path.join(sess, "sim", "truth.csv")
        lc = LC.check(sess, os.path.join(out, "bridge"), truth=tru if os.path.exists(tru) else None,
                      png=os.path.join(out, "line_check.png"))
        s["line_check"] = {n: dict(p95_cm=r["camera"].get("p95_cm"), within_pct=r["camera"].get("within_gate_pct"),
                                   misses=r["camera"].get("misses"), sure_share_pct=r["sure_share_pct"],
                                   camera_vs_truth=r.get("camera_vs_truth"))
                           for n, r in lc["robots"].items()}
        s["closest_by_camera"] = lc.get("closest_pair_camera")
    except Exception as e:     # noqa: BLE001
        s["line_check"] = "not made: %s" % e
    try:
        with open(os.path.join(out, "bridge", "bridge_summary.json")) as f:
            bs = json.load(f)
        s["camera"] = {"frames": bs.get("frames"), "robots": bs.get("robots"), "farm": bs.get("farm")}
    except (OSError, ValueError):
        pass
    if frames is not None:
        s["frames"] = frames.words()
    # labels the robots reached (blocks done), from the step reports
    done = {}
    try:
        with open(os.path.join(sess, "coordinator.jsonl")) as f:
            plan_steps = {}
            for ln in f:
                e = json.loads(ln)
                if e.get("event") == "labelled_step" and e.get("state") == "done":
                    lab = str(e.get("label", "")).split(" lane")[0]
                    done.setdefault(e["robot"], [])
                    if lab not in done[e["robot"]]:
                        done[e["robot"]].append(lab)
        s["reached"] = done
    except (OSError, ValueError):
        pass
    with open(os.path.join(out, "mission_summary.json"), "w") as f:
        json.dump(s, f, indent=1, default=str)
    return s


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--table", default="61x122", help="the table's size in cm, e.g. 61x122 (without --map)")
    ap.add_argument("--map", help="Mission Control's workspace map (sides, zones from the warm-up)")
    ap.add_argument("--program", help="a block program (Mission Control's program.json); default: Monday's")
    ap.add_argument("--robots", default="Left,Right", help="Monday's two robots (default Left,Right)")
    ap.add_argument("--mission", help="a mission map made elsewhere (Mission Control's warm-up), with --plan")
    ap.add_argument("--plan", help="with --mission: its plan")
    ap.add_argument("--session", help="the session folder (default logs/<date-time>-mission)")
    ap.add_argument("--plan-only", action="store_true", help="build and plan, then stop (nothing moves)")
    ap.add_argument("--min-gap-cm", type=float, default=EC.MIN_GAP_CM)
    ap.add_argument("--border-cm", type=float, default=6.0)
    ap.add_argument("--timeout-s", type=float, default=600.0)
    ap.add_argument("--robot", action="append", default=[], help="a robot's IP (default: found on the WiFi)")
    ap.add_argument("--calib", help="the overhead's floor calibration (default: what the setup page saved)")
    ap.add_argument("--side-calib", help="the side camera's (default: what the setup page saved)")
    ap.add_argument("--no-side", action="store_true", help="the overhead camera alone")
    ap.add_argument("--stream", default="127.0.0.1:8000")
    ap.add_argument("--side-stream", default="127.0.0.1:8001")
    ap.add_argument("--farm", action="store_true", help="each camera tracked in its own process (trackfarm.py)")
    ap.add_argument("--rings", action="store_true", help="--farm: the recorder shares its pictures (camrec.py --shm)")
    ap.add_argument("--half", action="store_true", help="--farm: track at half size")
    ap.add_argument("--no-frames", action="store_true", help="no tracked pictures or clips")
    ap.add_argument("--upload", action="store_true", help="upload the session (a release with a tag of its own)")
    ap.add_argument("--recordings", default=os.path.expanduser("~/recordings"))
    g = ap.add_argument_group("simulator (dry run)")
    g.add_argument("--dry", action="store_true")
    g.add_argument("--washout", action="store_true", help="the overhead's board washed out (27 Oct 11:20)")
    g.add_argument("--drop-pct", type=float, default=0.0, help="lost packets, %% each way")
    g.add_argument("--dead", help="NAME@T: that robot dies T s into the run (silent, still)")
    g.add_argument("--seed", type=int, default=1)
    args = ap.parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
