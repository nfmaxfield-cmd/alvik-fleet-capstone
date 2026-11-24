#!/usr/bin/env python3
"""(e24, simulator only) One acceptance run per CASE: the school site, two robots, Visit points, as
docs/results/e23-school-fixes/seed_runs.py (whose command it builds on), with the cases e24 adds.

    run_case.py REPO OUT_DIR CASE [CASE ...]     CASE = name:seed[:key=value]...

  pts=N            N points per robot (Alpha P01.., Bravo P21..); default 20
  lists=crossed    Alpha gets P21.., Bravo gets P01.. (each robot's list in the other's half: school runs 9-11)
  order=A:P01,P02;B:P35   the lists as typed (A = Alpha, B = Bravo)
  restart=Bravo@120       that robot restarts 120 s into the plan
  yaw=Alpha@5@25          a +25 deg gyro yaw jump at Alpha's first turn at or after 5 s (school 23 Nov, N1)
  out=Bravo@0             that robot is dead from T s on (one robot missing)
  robots=1                only one simulated robot answers (Bravo's list stays in the order)
  battery=Bravo@22        that robot reports 22 % battery (the school floor is 25 %)
  late=20,1.5             e23's stand-in: both cameras freeze 1.5 s every 20 s
  standin=school23        e24's stand-in preset (tools/fleet/sim_standin.py): losses drawn from the 23 Nov measurements,
                          more on camera 1's half, re-plans 5.0 s + 0.31 s a point
  slow=6                  e23's planner slow-down (not with standin=)
  wall=600                the runner ends the run (Ctrl+C) after this many seconds of wall clock
  env=K=V,K=V             extra environment (Safety switches)
  ahead=N                 MIRLT_PLAN_AHEAD_K=N (the plan-ahead DOE)
Changelog
  2026-11-24  first version (e24).
"""
import json, os, signal, subprocess, sys, time
VERSION = "e24-run-case-2026-11-24"
REPO, OUT = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
E23 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "e23-school-fixes")
os.makedirs(OUT, exist_ok=True)
for case in sys.argv[3:]:
    parts = case.split(":")
    name, seed = parts[0], int(parts[1])
    o = {}
    for p in parts[2:]:
        k, _, v = p.partition("=")
        o[k] = v
    # "order=A:P01,P02;B:P35" contains colons: rebuilt from the raw case text
    if "order=" in case:
        raw = case.split("order=", 1)[1]
        cut = len(raw)
        for k in ("pts", "lists", "restart", "yaw", "out", "robots", "battery", "late", "standin", "slow", "wall", "env", "ahead", "skew", "speed"):
            j = raw.find(":" + k + "=")
            if j >= 0:
                cut = min(cut, j)
        o["order"] = raw[:cut]
    npts = int(o.get("pts", 20))
    A = ",".join("P%02d" % k for k in range(1, 1 + npts))
    B = ",".join("P%02d" % k for k in range(21, 21 + npts))
    if o.get("lists") == "crossed":
        A, B = B, A
    order = "Alpha:%s;Bravo:%s" % (A, B)
    if o.get("order"):
        order = o["order"].replace("A:", "Alpha:").replace("B:", "Bravo:")
    folder = os.path.join(OUT, name)
    if os.path.exists(os.path.join(folder, "case.json")):
        print(name, "already run"); continue
    os.makedirs(folder, exist_ok=True)
    home = os.path.join(OUT, name + "-home")
    os.makedirs(os.path.join(home, "conf"), exist_ok=True)
    cond = {}
    if o.get("restart"):
        n, _, t = o["restart"].partition("@")
        cond["restart"] = {"name": n, "t": float(t), "boot_s": 3.0}
    if o.get("yaw"):
        n, t, d = (o["yaw"].split("@") + ["5", "25"])[:3]
        cond["yaw_jump"] = [{"name": n, "t": float(t), "deg": float(d)}]
    if o.get("out"):
        n, _, t = o["out"].partition("@")
        cond["robot_out"] = {"name": n, "t": float(t or 0), "mode": "dead"}
    if o.get("battery"):
        n, _, pct = o["battery"].partition("@")
        cond["battery"] = {n: float(pct)}
    cp = os.path.join(home, "cond.json")
    json.dump(cond, open(cp, "w"))
    cmd = [sys.executable, os.path.join(E23, "run_two_cameras.py"), REPO,
           "--session", folder, "--timeout-s", "900.0",
           "--border-cm", "6.0", "--tracking", "tags", "--site", "school-conference", "--feel-table", "skip",
           "--robots", "auto", "--split", "auto", "--motion", "stopturn",
           "--visit-order", order, "--visit-keep-unreadable", "--sim-robots", o.get("robots", "2"),
           "--sim-bind", "--sim-tag-yaw", "Alpha=180,Bravo=90", "--sim-dark", "150,0,200,50",
           "--sim-clock-skew", o.get("skew", "0"),
           "--sim-cond", cp, "--practice-real", "--readmap-store", os.path.join(home, "readmap"),
           "--visit-speed-cms", o.get("speed", "10"), "--visit-dwell-s", o.get("dwell", "5"), "--visit-chunk-cm", "0",
           "--on-problem", "keep_going", "--dry", "--seed", str(seed), "--no-frames",
           "--sim-arg=--max-s=1500"]
    if o.get("late"):
        cmd += ["--sim-late", o["late"]]
    if o.get("standin"):
        cmd += ["--sim-standin", o["standin"]]
    env = dict(os.environ, HOME=home, MIRLT_CONF_DIR=os.path.join(home, "conf"),
               MIRLT_HEARD_FILE=os.path.join(home, "conf", "heard_robots.json"))
    if o.get("slow"):
        env["MIRLT_SIM_REPLAN_SLOW"] = o["slow"]
    if o.get("ahead"):
        env["MIRLT_PLAN_AHEAD_K"] = o["ahead"]
    for kv in (o.get("env") or "").split(","):
        if "=" in kv:
            env[kv.split("=")[0]] = kv.split("=", 1)[1]
    t0 = time.time()
    with open(os.path.join(folder, "mission_stdout.txt"), "w") as f:
        p = subprocess.Popen(cmd, cwd=REPO, env=env, stdout=f, stderr=subprocess.STDOUT)
        try:
            rc = p.wait(timeout=float(o.get("wall", 1140)))
        except subprocess.TimeoutExpired:
            p.send_signal(signal.SIGINT)
            try:
                rc = p.wait(timeout=90)
            except subprocess.TimeoutExpired:
                p.kill()
                rc = -9
            open(os.path.join(folder, "CAPPED.txt"), "w").write("stopped by the acceptance runner after %s s of wall clock\n" % o.get("wall", 1140))
    json.dump({"case": case, "options": o, "seed": seed, "exit": rc, "wall_s": round(time.time() - t0, 1), "cmd": cmd[1:],
               "order": order, "version": VERSION},
              open(os.path.join(folder, "case.json"), "w"), indent=1)
    r = subprocess.run([sys.executable, os.path.join(E23, "analyse_run.py"), folder,
                        os.path.join(OUT, name + ".json")], capture_output=True, text=True)
    print(name, "exit", rc, "%.0f s" % (time.time() - t0), r.stdout.strip()[-600:], r.stderr.strip()[-300:], flush=True)
