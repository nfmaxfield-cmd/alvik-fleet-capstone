#!/usr/bin/env python3
"""Monday's 2-robot mission under the school's possible conditions, through the one-system stack (2026-10-28):
`mission_run.py --dry` (blocks -> plan -> camera bridge on both simulated cameras -> run_plan with every engine
check), one network namespace per run (ns_mission.py), conditions from sim_shim.py. Answers open question 31 for the
new mission, and (with --levels) the safety-net DOE: the minimum allowed closest approach (mission separation, Chicken's
min_cm and run_plan --min-gap-cm together) at 5 / 6.5 / 8 cm.

    nice -n 10 python3 tools/fleet/sweeps/monday2_sweep.py run --out DIR [--only COND ...] [--sizes ...]
                                                              [--seeds 1,2,3] [--levels 8] [--jobs 2]
    python3 tools/fleet/sweeps/monday2_sweep.py collect --out DIR        # results.json from every run's metrics

Each run's metrics go to <run>/metrics.json and DIR/results.jsonl; a finished run's large files (truth, raw records,
tracks) are deleted after that (disk), except the first seed of each condition on 61 x 122 at 8 cm (pictures).
"""
import argparse
import csv
import json
import math
import os
import random
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(FLEET))
if FLEET not in sys.path:
    sys.path.insert(0, FLEET)

import numpy as np          # noqa: E402

import fleetmap as FM       # noqa: E402
import geometry as G        # noqa: E402
import planner as P         # noqa: E402

NS = os.path.join(HERE, "ns_mission.py")
SIZES = ["61x122", "66x127", "70x130", "76x137"]     # Nick 16:43: a bit over 2 x 4 ft, at most 2.5 x 4.5 ft
STILL_WIN_S, STILL_CM, STILL_DEG = 0.5, 0.25, 2.0   # the Monday sweep's standstill rule (plan and run alike; no DOE)
OUT_T = (10.0, 110.0)     # chosen: a robot dies / restarts at a time drawn from 10-110 s (Monday's plan: 115-137 s).
                          # No DOE: the draw spreads it over every block
BOOT_S = (2.0, 6.0)       # chosen: a restarting robot is silent for 2-6 s (drawn), either side of SILENT_S (4 s), so
                          # both the "silent" and the "new boot ID" checks get exercised. Measure on the robots (no DOE)
WHEEL_PCT = (3.0, 5.0)    # Nick's brief: wheels 3-5 % off (each robot its own size and sign, drawn)
GYRO_PCT = (6.5, 8.5)     # chosen round Nick's 7.5 %: each robot's real error drawn (27 Oct L2: 7.0-8.4 %), the stored
                          # correction is the fleet's 1.075 for all (engine_checks PRIOR), so "corrected" leaves the
                          # real residual of up to 1 %
STORED_GYRO = 1.075
LIVE_STALL_EVERY_S = 20.0  # as the Monday sweep's CAM_STALL_EVERY_S (DOE note there)
TIMEOUT_S = 330.0         # a run is stopped after this (plans are 115-137 s): machine time only, no DOE
KEEP_SIZE = "61x122"


def code_version():
    """the commit and whether missions.py / mission_run.py differ from it (a run records what it ran)"""
    try:
        head = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=REPO, text=True).strip()
        dirty = subprocess.check_output(["git", "status", "--porcelain", "tools/fleet/missions.py",
                                         "tools/fleet/mission_run.py", "tools/fleet/run_plan.py",
                                         "tools/fleet/planner.py"], cwd=REPO, text=True).split()
        return head + ("+changes:" + ",".join(x for x in dirty if x != "M") if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return None


CODE = None


def draw(name, size, seed):
    key = "gyro" if name.startswith("gyro ") else name    # corrected / uncorrected: the same robots (paired)
    return random.Random("%s|%s|%d" % (key, size, seed))


def cond_args(name, size, seed):
    """-> (sim_shim conditions, extra mission_run arguments, words)."""
    r = draw(name, size, seed)
    c, a, w = {}, [], ""
    if name == "base":
        pass
    elif name.startswith("lost "):
        pct = float(name.split()[1].rstrip("%"))
        c["drop_pct"] = pct
    elif name == "washed out":
        a += ["--washout"]
    elif name == "side camera only":
        c["side_only"] = True
    elif name.startswith("stall ") or name.startswith("stale "):
        mode, s = name.split()[0], float(name.split()[1])
        c.update(live_stall_s=s, live_stall_mode=mode, live_stall_every_s=LIVE_STALL_EVERY_S)
    elif name == "wheels 3-5% off":
        v = {n: round(r.choice((-1, 1)) * r.uniform(*WHEEL_PCT), 2) for n in ("Left", "Center", "Right")}
        c["_wheel"] = v
        w = "wheels " + ", ".join("%s %+.1f %%" % (n, v[n]) for n in ("Left", "Right"))
    elif name.startswith("gyro "):
        v = {n: round(r.uniform(*GYRO_PCT), 2) for n in ("Left", "Center", "Right")}
        for n in ("Left", "Right"):
            a += ["--sim-arg=--turn-err=%s=%s" % (n, v[n])]
        if "corrected" in name and "uncorrected" not in name:
            a += ["--rp-arg=--send", "--rp-arg=set gyro_scale %.4f" % STORED_GYRO]
        w = "turns " + ", ".join("%s +%.1f %%" % (n, v[n]) for n in ("Left", "Right"))
    elif name == "robot dies":
        n, t = r.choice(("Left", "Right")), round(r.uniform(*OUT_T), 1)
        c["robot_out"] = {"name": n, "t": t, "mode": "dead"}
        w = "%s dies at %.0f s" % (n, t)
    elif name == "restart mid-run":
        n, t, b = r.choice(("Left", "Right")), round(r.uniform(*OUT_T), 1), round(r.uniform(*BOOT_S), 1)
        c["restart"] = {"name": n, "t": t, "boot_s": b}
        w = "%s restarts at %.0f s (silent %.1f s)" % (n, t, b)
    else:
        raise ValueError(name)
    return c, a, w


CONDS = ["base", "lost 5%", "lost 15%", "washed out", "side camera only", "stall 1 s", "stall 2.5 s",
         "stale 2.5 s", "wheels 3-5% off", "gyro 7.5% off, uncorrected", "gyro 7.5% off, corrected",
         "robot dies", "restart mid-run"]
# the safety-net DOE's noisy conditions (the ones that move the camera's spots or the robots off their lines)
DOE_CONDS = ["base", "washed out", "side camera only", "stale 2.5 s", "gyro 7.5% off, uncorrected"]


def slug(s):
    return s.replace(" ", "_").replace("%", "pct").replace(",", "").replace(".", "p")


def folder_of(root, name, size, seed, level):
    return os.path.join(root, "runs", "gap%s" % slug("%g" % level), slug(name), "%s-s%d" % (size, seed))


def run_cmd(root, name, size, seed, level, retry=0):
    d = folder_of(root, name, size, seed, level)
    os.makedirs(d, exist_ok=True)
    c, a, w = cond_args(name, size, seed)
    if "_wheel" in c:
        cfg = os.path.join(d, "sim_config.json")
        with open(cfg, "w") as f:
            json.dump({"robot": {"radius_scale_err_pct_by_robot": c.pop("_wheel")}}, f)
        a += ["--sim-arg=--config=%s" % cfg]
    if retry and "drop_pct" in c:
        c["retry"] = retry           # pressed again: the WiFi's losses are not the same packets the second time
    if c:
        with open(os.path.join(d, "cond.json"), "w") as f:
            json.dump(c, f)
        a += ["--sim-cond", os.path.join(d, "cond.json")]
    if abs(level - 8.0) > 1e-9:
        a += ["--min-gap-cm", "%g" % level, "--setting", "separation_cm=%g" % level,
              "--setting", "chicken_min_cm=%g" % level]
    cmd = ["nice", "-n", "10", "unshare", "-n", sys.executable, NS, "--table", size, "--dry", "--seed", str(seed),
           "--session", d, "--no-frames", "--timeout-s", "%.0f" % TIMEOUT_S] + a
    meta = dict(condition=name, size=size, seed=seed, level=level, words=w, cmd=cmd, code=CODE)
    return d, cmd, meta


# ---------------------------------------------------------------------------- what a run shows

def _still(xs, ys, ths, k):
    n = len(xs)
    j = np.minimum(np.arange(n) + k, n - 1)
    d = np.hypot(xs[j] - xs, ys[j] - ys)
    a = np.abs((ths[j] - ths + 180.0) % 360.0 - 180.0)
    return (d < STILL_CM) & (a < STILL_DEG)


def _longest(flags):
    best = cur = 0
    for f in flags:
        cur = cur + 1 if f else 0
        best = max(best, cur)
    return best


def load_events(path):
    ev = []
    if os.path.exists(path):
        with open(path) as f:
            for ln in f:
                try:
                    ev.append(json.loads(ln))
                except ValueError:
                    pass
    return ev


def label_at(ev, robot, t):
    lab = None
    for e in ev:
        if e.get("event") == "labelled_step" and e.get("robot") == robot and e.get("state") == "started":
            if e["t"] <= t:
                lab = e.get("label")
            else:
                break
    return lab


def metrics(d):
    out = {}
    meta = json.load(open(os.path.join(d, "sweep_meta.json")))
    out.update({k: meta.get(k) for k in ("condition", "size", "seed", "level", "words")})
    try:
        ms = json.load(open(os.path.join(d, "mission_summary.json")))
    except (OSError, ValueError):
        ms = {}
    rs = {}
    try:
        rs = json.load(open(os.path.join(d, "run", "run_plan_summary.json")))
    except (OSError, ValueError):
        pass
    mm = json.load(open(os.path.join(d, "mission.json")))
    plan = json.load(open(os.path.join(d, "plan.json")))
    out.update(result=rs.get("result") or ms.get("result"), finished=(rs.get("result") == "done"),
               took_s=rs.get("duration_s"), planned_s=plan["summary"]["makespan_s"],
               plan_closest_cm=plan["summary"]["min_separation_cm"],
               replans=ms.get("replans"), robots_out=rs.get("robots_out"), plans_refused=rs.get("plans_refused"),
               sensor_stops=ms.get("sensor_stops"), reached=ms.get("reached"))
    if out["took_s"] and out["planned_s"]:
        out["time_ratio"] = round(out["took_s"] / out["planned_s"], 3)
    lc = ms.get("line_check")
    if isinstance(lc, dict):
        out["line_p95_cm"] = {n: r.get("p95_cm") for n, r in lc.items()}
        out["cam_truth_p95_cm"] = {n: (r.get("camera_vs_truth") or {}).get("p95_cm") for n, r in lc.items()}
        out["line_misses"] = {n: r.get("misses") for n, r in lc.items()}
        out["sure_share_pct"] = {n: r.get("sure_share_pct") for n, r in lc.items()}
    ev = load_events(os.path.join(d, "run", "coordinator.jsonl"))
    sent = [e["t"] for e in ev if e.get("event") == "step_sent"]
    tp = os.path.join(d, "run", "sim", "truth.csv")
    if not sent or not os.path.exists(tp):
        out["no_truth"] = True
        return out
    t0 = min(sent)
    t1 = t0 + float(rs.get("duration_s") or 0.0) if rs.get("duration_s") else max(e["t"] for e in ev)
    by_t = {}
    with open(tp) as f:
        for r in csv.DictReader(f):
            t = float(r["t"])
            if t0 <= t <= t1:
                by_t.setdefault(r["t"], {})[r["robot"]] = (t, float(r["x_cm"]), float(r["y_cm"]),
                                                           float(r["heading_deg"]))
    names = sorted(plan["robots"])
    rows = [v for _, v in sorted(by_t.items(), key=lambda kv: float(kv[0])) if all(n in v for n in names)]
    if len(rows) < 20:
        out["no_truth"] = True
        return out
    T = np.array([r[names[0]][0] for r in rows])
    A = {n: np.array([[r[n][1], r[n][2], math.radians(r[n][3])] for r in rows]) for n in names}
    mn = FM.normalize(mm)
    body = FM.body_of(mn)
    f = mn["floor"]
    # closest two bodies
    gaps = np.array(G.body_gaps(body, A[names[0]], A[names[1]]), float)
    i = int(np.argmin(gaps))
    out["closest_cm"] = round(float(gaps[i]), 2)
    out["closest_t"] = round(float(T[i] - t0), 1)
    out["closest_during"] = {n: label_at(ev, n, T[i]) for n in names}
    out["contact"] = bool(gaps[i] <= 0)
    # closest body corner to each side (all four open at the school): the long sides are the lanes' ends
    edge = {}
    for n in names:
        c = np.asarray(body.corners(A[n]))           # (N, 4, 2)
        dd = {"y_min": c[:, :, 1].min(1) - f["y_min"], "y_max": f["y_max"] - c[:, :, 1].max(1),
              "x_min": c[:, :, 0].min(1) - f["x_min"], "x_max": f["x_max"] - c[:, :, 0].max(1)}
        for sd, v in dd.items():
            k = int(np.argmin(v))
            if sd not in edge or v[k] < edge[sd][0]:
                edge[sd] = (float(v[k]), n, float(T[k]))
    long_ = min((edge[s] for s in ("y_min", "y_max")), key=lambda e: e[0])
    short = min((edge[s] for s in ("x_min", "x_max")), key=lambda e: e[0])
    out["edge_long_cm"] = round(long_[0], 2)
    out["edge_long_during"] = label_at(ev, long_[1], long_[2])
    out["edge_short_cm"] = round(short[0], 2)
    out["edge_short_during"] = label_at(ev, short[1], short[2])
    out["edge_by_side"] = {sd: dict(cm=round(v[0], 2), robot=v[1], t=round(v[2] - t0, 1), during=label_at(ev, v[1], v[2]))
                           for sd, v in edge.items()}     # body corners (truth), not the centre
    out["edge_cm"] = round(min(long_[0], short[0]), 2)
    # standing still: both robots at once, run against the plan
    k = max(1, int(round(STILL_WIN_S / 0.05)))
    st_run = np.ones(len(T), bool)
    still_s = {}
    for n in names:
        fl = _still(A[n][:, 0], A[n][:, 1], np.degrees(A[n][:, 2]), k)
        still_s[n] = round(float(fl.sum()) * 0.05, 1)
        st_run &= fl
    ts = np.arange(0.0, out["planned_s"], 0.05)
    st_plan = np.ones(len(ts), bool)
    for n in names:
        po = np.array([P.pose_at(plan, n, t) for t in ts])
        st_plan &= _still(po[:, 0], po[:, 1], np.degrees(po[:, 2]), k)
    out["both_still_s"] = round(float(st_run.sum()) * 0.05, 1)
    out["both_still_plan_s"] = round(float(st_plan.sum()) * 0.05, 1)
    out["both_still_extra_s"] = round(out["both_still_s"] - out["both_still_plan_s"], 1)
    out["longest_both_still_s"] = round(_longest(st_run) * 0.05, 1)
    out["longest_both_still_plan_s"] = round(_longest(st_plan) * 0.05, 1)
    # the circle: true distance from its centre against its radius, while each robot is on its arcs
    circ = {}
    for n in names:
        ce = [e for e in ev if e.get("event") == "labelled_step" and e.get("robot") == n and e.get("label") == "circle"]
        if not ce:
            continue
        ta = min(e["t"] for e in ce if e["state"] == "started")
        done = [e["t"] for e in ce if e["state"] == "done"]
        tb = max(done) if done else ta
        e0 = ce[0]
        try:
            pl = json.load(open(os.path.join(d, "run", "plan_%d.json" % e0["plan"])))
            st = pl["robots"][n]["steps"][e0["i"]]
            cc, rr = st["arc"]["c"], st["arc"]["r"]
        except (OSError, KeyError, IndexError, TypeError, ValueError):
            continue
        m_ = (T >= ta) & (T <= tb)
        if m_.sum() < 3:
            continue
        err = np.abs(np.hypot(A[n][m_, 0] - cc[0], A[n][m_, 1] - cc[1]) - rr)
        circ[n] = dict(p95_cm=round(float(np.percentile(err, 95)), 2), max_cm=round(float(err.max()), 2),
                       s=round(tb - ta, 1), complete=bool(done) and len(done) >= len([x for x in ce if x["state"] ==
                                                                                       "started"]))
    out["circle"] = circ
    # what happened, in the run's own words
    out["events"] = sorted({e.get("event") for e in ev if e.get("event") in (
        "robot_out", "retired", "boot_changed", "plan_refused", "replan", "stranded", "camera_guard", "border_stop",
        "edge_boundary", "tracker_lost", "silent")})
    try:
        sr = json.load(open(os.path.join(d, "run", "sim", "sim_report.json")))
        out["sim_events"] = [e for e in sr.get("events", []) if e.get("kind") in ("robot_out", "robot_restart",
                                                                                 "hung")]
        out["hung"] = sr.get("hung")
    except (OSError, ValueError):
        pass
    return out


def slim(d):
    """a finished run: keep the small files (summaries, plans, pictures), drop the big ones."""
    for dp, _, fs in os.walk(d):
        for fn in fs:
            p = os.path.join(dp, fn)
            if os.path.getsize(p) > 200_000:
                os.remove(p)


# ---------------------------------------------------------------------------- the runs

# Priority (the machine is shared): every condition on the smallest and largest table first, then the safety-net
# DOE, then the two middle sizes. Conditions run on 2 sizes only (FULL_SIZES lists the 4-size ones)
FULL_SIZES = {"base", "lost 15%", "washed out", "side camera only", "stale 2.5 s", "robot dies", "restart mid-run"}


def jobs_for(a):
    seeds = [int(x) for x in a.seeds.split(",")]
    sizes = a.sizes.split(",")
    levels = [float(x) for x in a.levels.split(",")]
    ends = [s for s in sizes if s in ("61x122", "76x137")] or sizes[:1]
    mids = [s for s in sizes if s not in ends]
    out = []

    def add(name, size, seed, lvl):
        if (name, size, seed, lvl) not in out:
            out.append((name, size, seed, lvl))
    for lvl in [x for x in levels if abs(x - 8.0) < 1e-9]:
        for seed in seeds:
            for name in a.only or CONDS:
                for size in ends:
                    add(name, size, seed, lvl)
    for seed in seeds:                                      # the DOE's other levels, a seed at a time
        for name in a.only or DOE_CONDS:
            for size in ends:
                for lvl in [x for x in levels if abs(x - 8.0) > 1e-9]:
                    add(name, size, seed, lvl)
    for lvl in levels:
        if abs(lvl - 8.0) > 1e-9:
            continue
        conds = a.only or [c for c in CONDS if c in FULL_SIZES]
        for seed in seeds:
            for name in conds:
                for size in mids:
                    add(name, size, seed, lvl)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=["run", "collect", "list", "metrics"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--only", action="append", default=[])
    ap.add_argument("--sizes", default=",".join(SIZES))
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--levels", default="8")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--max-load", type=float, default=1.3,
                    help="start a second run only under this 1-minute load (one run of ours is ~0.7-1.0: the machine "
                         "otherwise idle)")
    ap.add_argument("--keep-all", action="store_true")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    logf = open(os.path.join(a.out, "sweep_log.txt"), "a")

    def log(m):
        line = "%s %s" % (time.strftime("%H:%M:%S"), m)
        print(line, flush=True)
        logf.write(line + "\n")
        logf.flush()
    if a.cmd == "list":
        for j in jobs_for(a):
            print(j, cond_args(j[0], j[1], j[2])[2])
        return 0
    if a.cmd == "metrics":
        for j in jobs_for(a):
            d = folder_of(a.out, *j)
            if os.path.exists(os.path.join(d, "sweep_meta.json")):
                m = metrics(d)
                json.dump(m, open(os.path.join(d, "metrics.json"), "w"), indent=1)
        return 0
    if a.cmd == "collect":
        rows = []
        for dp, _, fs in os.walk(os.path.join(a.out, "runs")):
            if "metrics.json" in fs:
                rows.append(json.load(open(os.path.join(dp, "metrics.json"))))
        rows.sort(key=lambda r: (r.get("level"), CONDS.index(r["condition"]) if r.get("condition") in CONDS else 99,
                                 SIZES.index(r["size"]) if r.get("size") in SIZES else 9, r.get("seed")))
        json.dump(rows, open(os.path.join(a.out, "results.json"), "w"), indent=1)
        log("collected %d runs" % len(rows))
        return 0
    global CODE
    CODE = code_version()
    log("code %s" % CODE)
    todo = [j for j in jobs_for(a) if not os.path.exists(os.path.join(folder_of(a.out, *j), "metrics.json"))]
    log("%d runs to do" % len(todo))
    running = []      # (proc, folder, meta, t0, tries, fh)
    first_try = {}

    def start(j, tries=0):
        d, cmd, meta = run_cmd(a.out, *j, retry=tries)
        if tries:
            for fn in os.listdir(d):
                if fn not in ("cond.json", "sim_config.json"):
                    p = os.path.join(d, fn)
                    shutil.rmtree(p) if os.path.isdir(p) else os.remove(p)
        meta["load_at_start"] = os.getloadavg()
        meta["tries"] = tries + 1
        json.dump(meta, open(os.path.join(d, "sweep_meta.json"), "w"), indent=1)
        fh = open(os.path.join(d, "mission_stdout.txt"), "w")
        p = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, cwd=REPO, env=dict(os.environ,
                                                                                         PYTHONHASHSEED="0"))
        running.append((p, d, meta, time.time(), tries, fh, j))
        log("start %s %s seed %d gap %g %s" % (j[0], j[1], j[2], j[3], meta["words"]))
    while todo or running:
        while todo and len(running) < a.jobs and (not running or os.getloadavg()[0] < a.max_load):
            start(todo.pop(0))
            time.sleep(8)          # stagger the start-ups (planning + imports are the heavy part)
        time.sleep(2)
        for item in list(running):
            p, d, meta, t0, tries, fh, j = item
            if p.poll() is None:
                continue
            running.remove(item)
            fh.close()
            try:
                m = metrics(d)
            except Exception as e:          # noqa: BLE001
                m = dict(meta, error=repr(e)[:300])
            m["wall_s"] = round(time.time() - t0, 1)
            m["code"] = meta.get("code")
            m["tries"] = tries + 1
            m["load_at_start"] = meta.get("load_at_start")
            if str(m.get("result", "")).startswith("start-up failed") and tries == 0:
                # a refused start is a result (kept as first_try), then the run is started again, as Nick would
                why = ""
                try:
                    txt = open(os.path.join(d, "mission_stdout.txt")).read()
                    why = next((ln.strip() for ln in txt.splitlines() if "Nothing was moved" in ln or
                                "No camera fix" in ln or "Not heard" in ln), "")
                except OSError:
                    pass
                log("  start-up failed (%s): once more" % why)
                shutil.copytree(d, d + "-startfail", dirs_exist_ok=True)
                slim(d + "-startfail")
                first_try[j] = why or str(m.get("result"))
                start(j, tries=1)
                continue
            if j in first_try:
                m["first_try_refused"] = first_try[j]
            json.dump(m, open(os.path.join(d, "metrics.json"), "w"), indent=1)
            with open(os.path.join(a.out, "results.jsonl"), "a") as f:
                f.write(json.dumps(m) + "\n")
            log("  done %s %s s%d gap %g: %s, took %s/%s s, closest %s cm, edge %s cm, still+ %s s, line p95 %s" % (
                j[0], j[1], j[2], j[3], m.get("result"), m.get("took_s"), m.get("planned_s"), m.get("closest_cm"),
                m.get("edge_cm"), m.get("both_still_extra_s"), m.get("line_p95_cm")))
            keep = a.keep_all or (j[1] == KEEP_SIZE and j[2] == 1 and abs(j[3] - 8.0) < 1e-9) or not m.get("finished")
            if not keep:
                slim(d)
    log("all done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
