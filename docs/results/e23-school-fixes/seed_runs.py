#!/usr/bin/env python3
"""(e23, simulator only) The acceptance runs: school site, two robots, Visit points with 20 + 20 points in a fixed
order, the command Mission Control's practice mode builds (tag yaw, a dark patch, a 0.7 s camera clock skew), run
straight through mission_run.py --dry so each run has its own seed and its own breaks:
  restart  NAME@T      that robot restarts T s into the plan (new boot, settings lost), as a robot waking would
  late     EVERY,STALL the cameras freeze STALL s every EVERY s; the next picture arrives that old (school 18 Nov)
  slow     X           every re-plan takes X times as long as on this machine (the school Pi planned 5-10 x slower)
  skew     S           the cameras' clock is S s ahead of the Pi's (Mission Control's practice mode: 0.7, with which the
                       simulated robots refuse EVERY fix as "from the future"; 0: one clock, as on the school Pi)
    seed_runs.py REPO OUT_DIR CASE [CASE ...]      CASE = name:seed[:restart=Alpha@70][:late=20,1.5][:slow=6][:speed=10][:skew=0]
Each case's folder gets the run, and OUT_DIR/<name>.json the numbers (analyse_run.py)."""
import json, os, subprocess, sys, time
REPO, OUT = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
os.makedirs(OUT, exist_ok=True)
ALPHA = ",".join("P%02d" % k for k in range(1, 21))
BRAVO = ",".join("P%02d" % k for k in range(21, 41))
for case in sys.argv[3:]:
    parts = case.split(":")
    name, seed = parts[0], int(parts[1])
    o = dict(p.split("=", 1) for p in parts[2:])
    folder = os.path.join(OUT, name)
    if os.path.exists(os.path.join(folder, "mission_summary.json")):
        print(name, "already run"); continue
    os.makedirs(folder, exist_ok=True)
    home = os.path.join(OUT, name + "-home")
    os.makedirs(os.path.join(home, "conf"), exist_ok=True)
    cond = {}
    if o.get("restart"):
        n, _, t = o["restart"].partition("@")
        cond["restart"] = {"name": n, "t": float(t), "boot_s": 3.0}
    cp = os.path.join(home, "cond.json")
    json.dump(cond, open(cp, "w"))
    cmd = [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_two_cameras.py"), REPO,
           "--session", folder, "--timeout-s", "900.0",
           "--border-cm", "6.0", "--tracking", "tags", "--site", "school-conference", "--feel-table", "skip",
           "--robots", "auto", "--split", "auto", "--motion", "stopturn",
           "--visit-order", "Alpha:%s;Bravo:%s" % (ALPHA, BRAVO), "--visit-keep-unreadable", "--sim-robots", "2",
           "--sim-bind", "--sim-tag-yaw", "Alpha=180,Bravo=90", "--sim-dark", "150,0,200,50",
           "--sim-clock-skew", o.get("skew", "0.7"),
           "--sim-cond", cp, "--practice-real", "--readmap-store", os.path.join(home, "readmap"),
           "--visit-speed-cms", o.get("speed", "10"), "--visit-dwell-s", o.get("dwell", "5"), "--visit-chunk-cm", "0",
           "--on-problem", "keep_going", "--dry", "--seed", str(seed), "--no-frames",
           "--sim-arg=--max-s=1500"]       # the simulator's own life (600 s by default) must outlast a slowed run
    if o.get("late"):
        cmd += ["--sim-late", o["late"]]
    env = dict(os.environ, HOME=home, MIRLT_CONF_DIR=os.path.join(home, "conf"),
               MIRLT_HEARD_FILE=os.path.join(home, "conf", "heard_robots.json"))
    if o.get("slow"):
        env["MIRLT_SIM_REPLAN_SLOW"] = o["slow"]
        hook = os.environ.get("E23_OLD_CODE_SLOW_HOOK")     # the code before e23 has no such switch: the same wait is
        if hook:                                             # put round its planner from outside (slowplan/)
            env["PYTHONPATH"] = hook + os.pathsep + env.get("PYTHONPATH", "")
    for kv in (o.get("env") or "").split(","):
        if "=" in kv:
            env[kv.split("=")[0]] = kv.split("=", 1)[1]
    t0 = time.time()
    with open(os.path.join(folder, "mission_stdout.txt"), "w") as f:
        p = subprocess.Popen(cmd, cwd=REPO, env=env, stdout=f, stderr=subprocess.STDOUT)
        try:
            rc = p.wait(timeout=float(o.get("wall", 1500)))
        except subprocess.TimeoutExpired:
            # past the wall-clock cap (25 min): ended the way a person would (Ctrl+C: every robot stopped, the
            # summary written) and counted as not finished
            import signal
            p.send_signal(signal.SIGINT)
            try:
                rc = p.wait(timeout=90)
            except subprocess.TimeoutExpired:
                p.kill()
                rc = -9
            open(os.path.join(folder, "CAPPED.txt"), "w").write("stopped by the acceptance runner after %s s of wall clock\n" % o.get("wall", 1500))
    json.dump({"case": case, "options": o, "seed": seed, "exit": rc, "wall_s": round(time.time() - t0, 1), "cmd": cmd[1:]},
              open(os.path.join(folder, "case.json"), "w"), indent=1)
    r = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyse_run.py"), folder,
                        os.path.join(OUT, name + ".json")], capture_output=True, text=True)
    print(name, "exit", rc, "%.0f s" % (time.time() - t0), r.stdout.strip()[-900:], r.stderr.strip()[-300:], flush=True)
