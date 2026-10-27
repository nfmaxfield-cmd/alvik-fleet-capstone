#!/usr/bin/env python3
"""Monday's school demo under the school's possible conditions (simulator only, 2026-10-27): the warm-up on a blank
table of each size, its answers applied, Monday's routine planned from that map, then dry runs of the routine with a
brick drop through the simulated camera (fleet_sim --brick-camera), each under one condition (sim_shim.py):

  table size      120x60, 152x76 (the assumed one), 180x90, 122x122 cm (all sides open: no boards at the school)
  lost packets    5 / 15 % each way
  camera          picture age 0.35 / 0.7 s; whole-camera stalls of 1.0 / 2.5 s every 20 s
  camera angle    on a stand at the audience side, 35 deg from 180 cm (height-corrected, or not) or 50 deg from 150 cm
  brick           middle (the build's M runs), on Center's next mowing lane, on Chicken's line, on the park row,
                  3 s ahead of a moving robot (fleet_sim auto)
  robot out       Right dead (stops, silent) or lifted off; Center (the mower) dead
  wheel scale     +3 / +5 / -5 % on every robot (a new surface); +5 % with 2.5 s camera stalls

    nice -n 19 python3 tools/fleet/sweeps/monday_sweep.py planfit --out DIR      # which table sizes Monday plans on
    nice -n 19 python3 tools/fleet/sweeps/monday_sweep.py prep --out DIR          # warm-ups, maps, Monday plans
    nice -n 19 python3 tools/fleet/sweeps/monday_sweep.py run --out DIR [--only NAME ...] [--seeds 1,2]
    python3 tools/fleet/sweeps/monday_sweep.py collect --out DIR --csv results.csv

One simulation at a time; each dry run in its own network namespace (unshare -n, rp_shim.py), so its UDP ports never
meet another dry run on the machine. Runs already done (run_plan_summary.json present) are skipped.
"""
import argparse
import copy
import csv
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
if FLEET not in sys.path:
    sys.path.insert(0, FLEET)

import numpy as np          # noqa: E402

import fleetmap as FM       # noqa: E402
import geometry as G        # noqa: E402
import missions as MS       # noqa: E402
import planner as P         # noqa: E402
import warmup as WU         # noqa: E402
import workspace as WS      # noqa: E402

RP_SHIM = os.path.join(HERE, "rp_shim.py")
# the edge guard and run settings as Mission Control sends them (mission_control.py RUN_DEFAULTS)
EDGE_GUARD = ["set safe_edge 3", "set edge_dir 0", "set edge_thr 150", "set edge_n 3", "set edge_bot_cm 11"]
SIZES = {"152x76": (152, 76), "120x60": (120, 60), "180x90": (180, 90), "122x122": (122, 122)}
BRICK_T_FRAC = 118.0 / 230.0     # the build's M runs: brick at 118 s of a 230 s plan. Kept for comparison, no DOE
BRICK_MID = (100.0 / 152.0, 40.0 / 76.0, 90.0)   # ... at (100, 40), across: as a fraction of the table. No DOE
BRICK_CLEAR_CM = 15.0     # chosen: a dropped brick lands at least this far from every robot's planned centre (Nick
                          # would not drop it on a robot); the drop moves 2 s later until it does. No DOE (a person
                          # chooses the spot on the day)
TIMEOUT_K, TIMEOUT_ADD_S = 1.6, 30.0   # chosen: a run is stopped at 1.6 x the plan + 30 s (the build's runs took up to
                          # 1.03 x). Only bounds machine time for runs that hang; no DOE
STILL_WIN_S, STILL_CM, STILL_DEG = 0.5, 0.25, 2.0   # chosen: a robot "stands still" when it moved under 0.25 cm and
                          # turned under 2 deg over 0.5 s (cruise 10 cm/s, turns 60 deg/s: far above). No DOE: the same
                          # rule is applied to the plan and the run, so only the difference is reported
CAM_STALL_EVERY_S = 20.0  # chosen: a stalling camera stalls once every 20 s (the Pi's heat-throttled tracker and the
                          # school WiFi are unmeasured). DOE note: stall length x spacing (1 / 2.5 s x 10 / 20 / 60 s)
                          # against standstill and closest approach, simulator, docs/doe-backlog.md
OUT_T_S = 100.0           # chosen: the robot taken out at 100 s (mid-mowing, after Chicken). No DOE: one moment of
                          # many; the result is the same in kind at any moment after the show (see the report)

# the conditions: name -> dict(group, size, cond (sim_shim), brick, warm (which warm-up's map), wheel, seeds)
C = {}


def cond(name, group, size="152x76", seeds=(1,), brick="middle", warm="nominal", wheel=None, **sim):
    C[name] = dict(name=name, group=group, size=size, seeds=list(seeds), brick=brick, warm=warm, wheel=wheel, sim=sim)


cond("base 152x76", "table size", seeds=(1, 2, 3))
cond("180x90", "table size", size="180x90", seeds=(1, 2))
cond("122x122", "table size", size="122x122", seeds=(1, 2))
cond("120x60", "table size", size="120x60", seeds=(1,))
cond("lost 5%", "lost packets", seeds=(1, 2), drop_pct=5.0)
cond("lost 15%", "lost packets", seeds=(1, 2), drop_pct=15.0)
cond("age 0.35 s", "camera", seeds=(1,), cam_age_s=0.35)
cond("age 0.7 s", "camera", seeds=(1, 2), cam_age_s=0.7)
cond("stall 1 s", "camera", seeds=(1, 2), cam_stall_s=1.0, cam_stall_every_s=CAM_STALL_EVERY_S)
cond("stall 2.5 s", "camera", seeds=(1, 2), cam_stall_s=2.5, cam_stall_every_s=CAM_STALL_EVERY_S)
cond("tilt 35 corrected", "camera angle", seeds=(1,),
     cam_tilt=dict(tilt_deg=35.0, height_cm=180.0, led_cm=4.0, corrected=True, height_err_pct=10.0, nadir_err_cm=10.0))
cond("tilt 35 uncorrected", "camera angle", seeds=(1, 2), warm="tilt35u",
     cam_tilt=dict(tilt_deg=35.0, height_cm=180.0, led_cm=4.0, corrected=False))
cond("tilt 50 uncorrected", "camera angle", seeds=(1, 2), cam_tilt=dict(tilt_deg=50.0, height_cm=150.0, led_cm=4.0,
                                                                        corrected=False))
cond("brick lane", "brick", seeds=(1, 2), brick="lane")
cond("brick chicken", "brick", seeds=(1, 2), brick="chicken")
cond("brick park row", "brick", seeds=(1,), brick="park")
cond("brick ahead", "brick", seeds=(1, 2), brick="ahead")
cond("Right dead", "robot out", seeds=(1,), robot_out=dict(name="Right", t=OUT_T_S, mode="dead"))
cond("Right lifted", "robot out", seeds=(1,), robot_out=dict(name="Right", t=OUT_T_S, mode="lifted"))
cond("Center dead", "robot out", seeds=(1,), robot_out=dict(name="Center", t=OUT_T_S, mode="dead"))
cond("wheel +3%", "wheel scale", seeds=(1,), wheel=3.0)
cond("wheel +5%", "wheel scale", seeds=(1, 2), wheel=5.0, warm="wheel+5")
cond("wheel -5%", "wheel scale", seeds=(1, 2), wheel=-5.0)
cond("wheel +5% + stall 2.5 s", "wheel scale", seeds=(1,), wheel=5.0, cam_stall_s=2.5,
     cam_stall_every_s=CAM_STALL_EVERY_S)

WARMS = {"nominal": {}, "tilt35u": C["tilt 35 uncorrected"]["sim"], "wheel+5": {}}
WARM_WHEEL = {"wheel+5": 5.0}


def starts(W, H):
    """Where the robots stand for the warm-up: Mission Control's default spots (30 / 76 / 122, 18 on 152 x 76),
    scaled along the table."""
    return [("Left", [round(0.197 * W, 1), 18.0, 90.0]), ("Center", [round(0.5 * W, 1), 18.0, 90.0]),
            ("Right", [round(0.803 * W, 1), 18.0, 90.0])]


def blank(size):
    W, H = SIZES[size]
    return WS.blank_map((0, 0, W, H), starts(W, H))


def wheel_config(path, pct):
    with open(path, "w") as f:
        json.dump({"robot": {"radius_scale_err_pct_by_robot": {n: pct for n in ("Left", "Center", "Right")}}}, f)
    return path


def dry_run(map_path, plan_path, folder, seed, sim=None, wheel=None, brick=None, border_cm=6.0, timeout_s=900.0):
    """run_plan.py --dry-run through rp_shim (own network namespace), as Mission Control starts it. Returns the code."""
    os.makedirs(folder, exist_ok=True)
    cmd = ["nice", "-n", "19", "unshare", "-n", sys.executable, RP_SHIM, "--map", map_path, "--plan", plan_path,
           "--yes", "--session", folder, "--timeout-s", "%.0f" % timeout_s, "--border-cm", str(border_cm),
           "--improve-budget-s", "6.0", "--dry-run", "--seed", str(seed)]
    for c in EDGE_GUARD:
        cmd += ["--send", c]
    if sim:
        cp = os.path.join(folder, "sweep_cond.json")
        with open(cp, "w") as f:
            json.dump(sim, f, indent=1)
        cmd += ["--sim-arg=--sweep-cond=%s" % cp]
    if wheel:
        cmd += ["--sim-arg=--config=%s" % wheel_config(os.path.join(folder, "sim_config.json"), wheel)]
    if brick:
        cmd += ["--brick", brick, "--sim-arg=--brick-camera"]
    env = dict(os.environ, PYTHONHASHSEED="0")
    t0 = time.time()
    with open(os.path.join(folder, "run_plan_stdout.txt"), "w") as out:
        code = subprocess.call(cmd, stdout=out, stderr=subprocess.STDOUT, env=env)
    with open(os.path.join(folder, "sweep_cmd.json"), "w") as f:
        json.dump({"cmd": cmd, "code": code, "wall_s": round(time.time() - t0, 1),
                   "load": os.getloadavg()}, f, indent=1)
    return code


# ---------------------------------------------------------------------------- prep: warm-up, map, Monday's plan

def prep_one(root, size, warm="nominal", log=print):
    d = os.path.join(root, "prep", "%s-%s" % (size, warm))
    os.makedirs(d, exist_ok=True)
    done = os.path.join(d, "prep.json")
    if os.path.exists(done):
        with open(done) as f:
            return json.load(f)
    rec = {"size": size, "warm": warm}
    m = blank(size)
    FM.save(m, os.path.join(d, "table.json"))
    wm = WU.plan_tasks(m)
    with open(os.path.join(d, "warm_map.json"), "w") as f:
        json.dump(wm, f, indent=1)
    wp = P.make_plan(wm)
    P.save_plan(wp, os.path.join(d, "warm_plan.json"))
    rec["warm_planned_s"] = wp["summary"]["makespan_s"]
    log("%s %s: warm-up planned %.0f s; running it" % (size, warm, wp["summary"]["makespan_s"]))
    wf = os.path.join(d, "warm-run")
    code = dry_run(os.path.join(d, "warm_map.json"), os.path.join(d, "warm_plan.json"), wf, 1, sim=WARMS[warm] or None,
                   wheel=WARM_WHEEL.get(warm), border_cm=0.0,
                   timeout_s=TIMEOUT_K * wp["summary"]["makespan_s"] + TIMEOUT_ADD_S)
    rec["warm_code"] = code
    res = WU.analyze(wm, wf)
    with open(os.path.join(d, "warmup_result.json"), "w") as f:
        json.dump(res, f, indent=1)
    rec["warm_words"] = WU.words(res)
    rec["warm_sides"] = {k: v["kind"] for k, v in res["sides"].items()}
    rec["warm_where"] = res.get("where")
    rec["warm_zones"] = len(res["zones"])
    rec["warm_scales"] = res.get("scales")
    m2 = WU.apply(m, res)                     # Nick keeps what it found (zones stay "proposed": kept out)
    FM.save(m2, os.path.join(d, "map.json"))
    t0 = time.time()
    try:
        mm = MS.monday(m2)
        over = dict(mm["mission"]["planner"])
        plan = P.make_plan(mm, over=over, t_limit_s=60)
        with open(os.path.join(d, "mission.json"), "w") as f:
            json.dump(mm, f, indent=1)
        P.save_plan(plan, os.path.join(d, "plan.json"))
        rec.update(plan_ok=True, plan_s=round(time.time() - t0, 1), makespan_s=plan["summary"]["makespan_s"],
                   plan_closest_cm=plan["summary"]["min_separation_cm"], notes=mm["mission"]["notes"])
    except Exception as e:                    # noqa: BLE001 (a table Monday cannot be planned on is a result)
        rec.update(plan_ok=False, plan_s=round(time.time() - t0, 1), plan_error=repr(e)[:400])
    with open(done, "w") as f:
        json.dump(rec, f, indent=1)
    log("%s %s: %s" % (size, warm, "planned %.0f s (%.0f s to plan)" % (rec["makespan_s"], rec["plan_s"])
                       if rec["plan_ok"] else "NO PLAN: " + rec["plan_error"]))
    return rec


def planfit(root, log=print):
    """Planner only: does Monday plan on a table of this size (all sides open, as before the warm-up)?"""
    out = os.path.join(root, "planfit.json")
    have = json.load(open(out)) if os.path.exists(out) else []
    seen = {(r["W"], r["H"]) for r in have}
    for W in (120, 152, 180):
        for H in (60, 64, 68, 76, 90, 122):
            if (W, H) in seen or (W == 180 and H == 122):
                continue
            m = WS.blank_map((0, 0, W, H), starts(W, H))
            t0 = time.time()
            r = {"W": W, "H": H}
            try:
                mm = MS.monday(m)
                plan = P.make_plan(mm, over=dict(mm["mission"]["planner"]), t_limit_s=60)
                r.update(ok=True, makespan_s=plan["summary"]["makespan_s"],
                         closest_cm=plan["summary"]["min_separation_cm"],
                         shortened=[n for n in mm["mission"]["notes"] if "shortened" in n or "no room" in n])
            except Exception as e:            # noqa: BLE001
                r.update(ok=False, error=repr(e)[:300])
            r["plan_s"] = round(time.time() - t0, 1)
            have.append(r)
            log("planfit %dx%d: %s" % (W, H, ("%.0f s" % r["makespan_s"]) if r["ok"] else r["error"][:120]))
            with open(out, "w") as f:
                json.dump(have, f, indent=1)
    return have


# ---------------------------------------------------------------------------- the runs

def brick_spec(kind, mm, plan):
    """--brick for the simulator: (spec, where-words)."""
    f = mm["floor"]
    W, H = f["x_max"] - f["x_min"], f["y_max"] - f["y_min"]
    T = BRICK_T_FRAC * plan["summary"]["makespan_s"]
    if kind == "ahead":
        return "auto@%.0f" % T, "3 s ahead of a moving robot"
    if kind == "middle":
        x, y, a = f["x_min"] + BRICK_MID[0] * W, f["y_min"] + BRICK_MID[1] * H, BRICK_MID[2]
        what = "middle (as the build's M runs)"
    elif kind == "lane":
        lanes = [st for st in plan["robots"]["Center"]["steps"] if "mow lane" in (st.get("label") or "")
                 and st["label"].endswith("start") and st["t_start"] > T + 8.0]
        st = lanes[0]
        nxt = next(s for s in plan["robots"]["Center"]["steps"] if s.get("label") == st["label"].replace("start", "end"))
        x, y = (st["to"][0] + nxt["to"][0]) / 2.0, (st["to"][1] + nxt["to"][1]) / 2.0
        a = 0.0 if abs(st["to"][0] - nxt["to"][0]) < abs(st["to"][1] - nxt["to"][1]) else 90.0   # across the lane
        what = "across Center's %s (mowed %.0f s later)" % (st["label"].replace(" start", ""), st["t_start"] - T)
    elif kind == "chicken":
        L = [s for s in plan["robots"]["Left"]["steps"] if s.get("label") == "chicken 2 ready"]
        R = [s for s in plan["robots"]["Right"]["steps"] if s.get("label") == "chicken 2 ready"]
        x = (L[-1]["to"][0] + R[-1]["to"][0]) / 2.0
        y = (L[-1]["to"][1] + R[-1]["to"][1]) / 2.0
        a = 90.0                             # across their line
        T = L[-1]["t_start"] - 2.0
        what = "on Chicken's line, between them, before round 2"
    elif kind == "park":
        pk = {r["name"]: r["park"] for r in mm["robots"]}
        x, y, a = pk["Right"][0], pk["Right"][1], 0.0
        T = 0.8 * plan["summary"]["makespan_s"]
        what = "on the park row (Right's spot)"
    else:
        raise ValueError(kind)
    # not on a robot: later until every robot's planned centre is BRICK_CLEAR_CM away
    poly = None
    for _ in range(40):
        c, s_ = math.cos(math.radians(a)), math.sin(math.radians(a))
        poly = np.array([[x + c * u - s_ * v, y + s_ * u + c * v] for u, v in ((10, 4.5), (-10, 4.5), (-10, -4.5), (10, -4.5))])
        near = min(float(G.points_to_polygon(np.array([P.pose_at(plan, n, T)[:2]]), poly)[0]) for n in plan["robots"])
        if near >= BRICK_CLEAR_CM:
            break
        T += 2.0
    return "%.1f,%.1f,%.0f@%.0f" % (x, y, a, T), what


def run_one(root, c, seed, log=print):
    pr = prep_one(root, c["size"], c["warm"], log)
    d = os.path.join(root, "prep", "%s-%s" % (c["size"], c["warm"]))
    folder = os.path.join(root, "runs", "%s__s%d" % (c["name"].replace(" ", "_").replace("%", "pct").replace("+", "p"),
                                                     seed))
    if not pr.get("plan_ok"):
        return None
    if os.path.exists(os.path.join(folder, "run_plan_summary.json")):
        return folder
    mm = json.load(open(os.path.join(d, "mission.json")))
    plan = json.load(open(os.path.join(d, "plan.json")))
    spec, what = brick_spec(c["brick"], mm, plan)
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "sweep_meta.json"), "w") as f:
        json.dump({"condition": c, "seed": seed, "brick": spec, "brick_where": what, "prep": d}, f, indent=1)
    log("run %s seed %d (brick %s: %s)" % (c["name"], seed, spec, what))
    t0 = time.time()
    code = dry_run(os.path.join(d, "mission.json"), os.path.join(d, "plan.json"), folder, seed, sim=c["sim"] or None,
                   wheel=c["wheel"], brick=spec,
                   timeout_s=TIMEOUT_K * plan["summary"]["makespan_s"] + TIMEOUT_ADD_S)
    try:
        mt = metrics(folder)
        log("  -> %s in %.0f s wall: %s" % ("code %d" % code, time.time() - t0,
                                             {k: mt[k] for k in ("result", "took_s", "closest_cm", "edge_cm", "replans",
                                                                 "lanes", "extra_still_s")}))
    except Exception as e:                    # noqa: BLE001
        log("  -> code %d, metrics failed: %r" % (code, e))
    return folder


# ---------------------------------------------------------------------------- what a run shows

def _still(xs, ys, ths, dt):
    """Per-sample still flags (moved < STILL_CM and turned < STILL_DEG over STILL_WIN_S)."""
    k = max(1, int(round(STILL_WIN_S / dt)))
    n = len(xs)
    out = np.zeros(n, bool)
    for i in range(n):
        j = min(n - 1, i + k)
        if math.hypot(xs[j] - xs[i], ys[j] - ys[i]) < STILL_CM and \
                abs((ths[j] - ths[i] + 180) % 360 - 180) < STILL_DEG:
            out[i] = True
    return out


def metrics(folder):
    s = json.load(open(os.path.join(folder, "run_plan_summary.json")))
    meta = json.load(open(os.path.join(folder, "sweep_meta.json"))) if os.path.exists(
        os.path.join(folder, "sweep_meta.json")) else {}
    plan = json.load(open(os.path.join(folder, "plan_given.json")))
    sim = s.get("simulator") or {}
    ev = []
    cj = os.path.join(folder, "coordinator.jsonl")
    if os.path.exists(cj):
        for ln in open(cj):
            try:
                ev.append(json.loads(ln))
            except ValueError:
                pass
    lab = [e for e in ev if e.get("event") == "labelled_step"]
    t_run0 = min([e["t"] - e["t_run"] for e in lab if e.get("t_run") is not None] or [None]) if lab else None
    lanes_planned = len({st["label"].split(" ")[2] for st in plan["robots"]["Center"]["steps"]
                         if "mow lane" in (st.get("label") or "")})
    lanes_done = len({e["label"].split(" ")[2] for e in lab if e["robot"] == "Center" and
                      "mow lane" in e["label"] and e["label"].endswith("end") and e["state"] == "done"})
    acts = {}
    for key in ("chicken 1 swerve", "chicken 2 swerve", "roam", "follow the leader"):
        acts[key] = sorted({e["robot"] for e in lab if e["label"] == key and e["state"] == "done"})
    parked = []
    rr = s.get("replans", [])
    replans = [r for r in rr if r.get("source") not in ("improve",)]
    # standstill from the simulator's truth against the first plan, same rule for both
    extra, fleet_extra, still_run, still_plan = None, None, {}, {}
    tp = os.path.join(folder, "sim", "truth.csv")
    if os.path.exists(tp) and t_run0 is not None:
        rows = {}
        with open(tp) as f:
            for r in csv.DictReader(f):
                rows.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"]),
                                                        float(r["heading_deg"])))
        t_end = t_run0 + s.get("duration_s", 0.0)
        dt = 0.05
        flags = {}
        for n, v in rows.items():
            v = [p for p in v if t_run0 <= p[0] <= t_end]
            if len(v) < 10:
                continue
            a = np.array(v)
            fl = _still(a[:, 1], a[:, 2], a[:, 3], dt)
            flags[n] = (a[:, 0], fl)
            still_run[n] = round(float(fl.sum()) * dt, 1)
        mk = plan["summary"]["makespan_s"]
        ts = np.arange(0.0, mk, dt)
        pflags = {}
        for n in plan["robots"]:
            poses = np.array([P.pose_at(plan, n, t) for t in ts])
            fl = _still(poses[:, 0], poses[:, 1], np.degrees(poses[:, 2]), dt)
            pflags[n] = fl
            still_plan[n] = round(float(fl.sum()) * dt, 1)
        if flags:
            extra = round(sum(still_run.get(n, 0) - still_plan.get(n, 0) for n in still_plan), 1)
            # all three standing at once
            grid = np.arange(t_run0, t_end, dt)
            allr = np.ones(len(grid), bool)
            for n, (tt, fl) in flags.items():
                allr &= np.interp(grid, tt, fl.astype(float), left=1, right=1) > 0.5
            allp = np.ones(len(ts), bool)
            for n, fl in pflags.items():
                allp &= fl
            fleet_extra = round(float(allr.sum()) * dt - float(allp.sum()) * dt, 1)
    # where each robot ended against its park spot (the truth)
    mm = json.load(open(os.path.join(meta["prep"], "mission.json"))) if meta.get("prep") else None
    if mm and os.path.exists(tp):
        last = {}
        with open(tp) as f:
            for r in csv.DictReader(f):
                last[r["robot"]] = (float(r["x_cm"]), float(r["y_cm"]))
        for r in mm["robots"]:
            if r["name"] in last and math.dist(last[r["name"]], r["park"][:2]) < 3.0:
                parked.append(r["name"])
    # re-plans where the code before this sweep's planner fix would have dropped a robot's visits (its next step's
    # task before its plan's own visit list: seq_visits[negative:]). Approximate: the step under way at the event
    old_bug = 0
    for e in ev:
        if e.get("event") != "replan" or e.get("old_plan") is None:
            continue
        pp = os.path.join(folder, "plan_%d.json" % e["old_plan"])
        if not os.path.exists(pp):
            continue
        op = json.load(open(pp))
        for n, st in (e.get("states") or {}).items():
            r = op["robots"].get(n) or {}
            steps = r.get("steps") or []
            nxt = next((x for x in steps[st.get("step", 0):] if x["phase"] not in ("wait", "hold")), None)
            if nxt is not None and nxt.get("task") is not None and nxt["task"] < r.get("task_offset", 0):
                old_bug += 1
    edge = sim.get("min_open_edge_cm")
    cond_ = meta.get("condition", {})
    brick_ev = [e for e in sim.get("events", []) if e.get("kind") in ("brick", "camera_reported_brick")]
    tb = next((e["t"] for e in brick_ev if e["kind"] == "brick"), None)
    tr = next((e["t"] for e in brick_ev if e["kind"] == "camera_reported_brick"), None)
    return {
        "condition": cond_.get("name"), "group": cond_.get("group"), "seed": meta.get("seed"),
        "brick": meta.get("brick"), "brick_where": meta.get("brick_where"),
        "result": s.get("result"), "finished": s.get("result") == "done",
        "took_s": s.get("duration_s"), "planned_s": s.get("planned_makespan_s"),
        "late_s": round((s.get("duration_s") or 0) - (s.get("planned_makespan_s") or 0), 1),
        "closest_cm": sim.get("min_body_gap_cm"), "closest_at": sim.get("min_body_gap_at"),
        "brick_cm": sim.get("min_static_clearance_cm"), "contact": sim.get("collision"),
        "edge_cm": edge, "edge_crossed": edge is not None and edge < 0, "edge_at": sim.get("min_open_edge_at"),
        "hung": sim.get("hung") or [],
        "replans": len(replans), "replan_cpu_max_s": round(max([r["cpu_s"] for r in replans] or [0.0]), 2),
        "replan_cpu_sum_s": round(sum(r["cpu_s"] for r in replans), 1),
        "improves_kept": sum(1 for r in rr if r.get("source") == "improve" and r.get("kept")),
        "stranded_events": sum(1 for r in rr if r.get("stranded")),
        "replan_sources": sorted({r.get("source") for r in replans}),
        "sensor_obstacles": s.get("sensor_obstacles"), "border_stops": sum(1 for e in ev if e.get("event") == "border_stop"),
        "edge_stops": sum(1 for e in ev if e.get("event") == "edge_boundary"),
        "tracker_lost": sum(1 for e in ev if e.get("event") == "tracker_lost"),
        "lanes": "%d/%d" % (lanes_done, lanes_planned), "lanes_done": lanes_done, "lanes_planned": lanes_planned,
        "activities": acts, "parked": parked,
        "still_run_s": still_run, "still_plan_s": still_plan, "extra_still_s": extra, "fleet_still_extra_s": fleet_extra,
        "brick_t": tb, "brick_seen_t": tr, "brick_seen_after_s": round(tr - tb, 2) if (tr is not None and tb is not None) else None,
        "old_code_drop_risk": old_bug,
        "resends": sum((s.get("resends") or {}).values()),
        "packets_lost": sim.get("packets_lost"),
        "folder": os.path.basename(folder),
    }


def collect(root):
    out = []
    rd = os.path.join(root, "runs")
    for name in sorted(os.listdir(rd)) if os.path.isdir(rd) else []:
        f = os.path.join(rd, name)
        if os.path.exists(os.path.join(f, "run_plan_summary.json")):
            try:
                out.append(metrics(f))
            except Exception as e:          # noqa: BLE001
                out.append({"folder": name, "error": repr(e)})
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=["planfit", "prep", "run", "collect", "list"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--only", action="append", default=[], help="condition name (repeatable)")
    ap.add_argument("--seeds", help="override the seeds, e.g. 1,2")
    ap.add_argument("--max-seed", type=int, default=99, help="only seeds up to this (first pass: 1)")
    ap.add_argument("--json", help="collect: write the rows here")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    logf = open(os.path.join(a.out, "sweep_log.txt"), "a")

    def log(msg):
        line = "%s %s" % (time.strftime("%H:%M:%S"), msg)
        print(line, flush=True)
        logf.write(line + "\n")
        logf.flush()
    if a.cmd == "list":
        for c in C.values():
            print("%-28s %-14s %-8s seeds %s" % (c["name"], c["group"], c["size"], c["seeds"]))
    elif a.cmd == "planfit":
        planfit(a.out, log)
    elif a.cmd == "prep":
        for c in C.values():
            if not a.only or c["name"] in a.only:
                prep_one(a.out, c["size"], c["warm"], log)
    elif a.cmd == "run":
        names = a.only or list(C)
        for n in names:
            c = C[n]
            seeds = [int(x) for x in a.seeds.split(",")] if a.seeds else c["seeds"]
            for sd in seeds:
                if sd <= a.max_seed:
                    run_one(a.out, c, sd, log)
    else:
        rows = collect(a.out)
        with open(a.json or os.path.join(a.out, "results.json"), "w") as f:
            json.dump(rows, f, indent=1)
        for r in rows:
            print(json.dumps({k: r.get(k) for k in ("condition", "seed", "result", "took_s", "closest_cm", "edge_cm",
                                                    "replans", "lanes", "extra_still_s")}))


if __name__ == "__main__":
    main()
