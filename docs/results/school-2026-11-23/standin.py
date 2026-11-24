#!/usr/bin/env python3
"""Numbers for the simulator's school-Pi stand-in, from the 23 Nov runs. Read-only.
Usage: python3 -I standin.py <jayruns23 folder> [out.json]
(1) per-robot camera losses while the plan ran ('tracker_back' lost_s): how many a minute, how long, time between;
(2) re-plan time against points planned (least-squares line over every 'replan' with points_planned);
(3) first-plan times; (4) the run's stop tolerance as it moved (reach_tol_cm); (5) sources.csv 'other' (must be 0: no
    source beyond tags, lock, motion, lights, ml fed a robot's position).
Changelog: 2026-11-24 first version."""
import csv, json, os, re, sys, collections, statistics as S
ROOT = sys.argv[1]
def q(v, p): v = sorted(v); return round(v[int(p * (len(v) - 1))], 2) if v else None
allst = {1: [], 2: []}; betw = {1: [], 2: []}; robmin = {1: 0.0, 2: 0.0}; rp = []; first = []; tol = []; other = 0; per_run = []
for d in sorted(os.listdir(ROOT)):
    cj = os.path.join(ROOT, d, "run", "coordinator.jsonl")
    if not os.path.exists(cj): continue
    L = []
    for l in open(cj, errors="replace"):
        try: L.append(json.loads(l))
        except ValueError: pass
    t0 = next((e["t"] for e in L if e.get("event") == "message" and "plan plan-" in e.get("text", "") and "started" in e["text"]), None)
    if t0 is None: continue
    t1 = L[-1]["t"]; nrob = len(set(e["robot"] for e in L if e.get("event") == "step_sent"))
    by = collections.defaultdict(list)
    for e in L:
        if e.get("event") == "tracker_back" and e["t"] - e["lost_s"] >= t0: by[e["robot"]].append((e["t"] - e["lost_s"], e["lost_s"]))
        if e.get("event") == "replan" and e.get("points_planned") is not None: rp.append((e["points_planned"], e["cpu_s"], nrob, d[9:13]))
        if e.get("event") in ("point_confidence", "point_not_reached"): tol.append((e.get("reach_tol_cm"), e.get("reach_tol_with_pm_cm")))
    n = 0
    for rb, v in by.items():
        v.sort(); allst[nrob] += [x[1] for x in v]; betw[nrob] += [b[0] - a[0] for a, b in zip(v, v[1:])]; n += len(v)
    robmin[nrob] += nrob * (t1 - t0) / 60.0
    so = open(os.path.join(ROOT, d, "run_plan_stdout.txt"), errors="replace").read()
    m = re.search(r"planned in ([0-9.]+) s", so); m2 = re.search(r"started: \d robots, (\d+) steps", so)
    pts = sum(len(b.split(" -> ")) for a, b in re.findall(r"VISIT ORDER (\w+): (.*)", so)) or 40
    if m: first.append((d[9:13], nrob, pts, float(m.group(1))))
    f = os.path.join(ROOT, d, "run", "sources.csv")
    if os.path.exists(f):
        for r in csv.DictReader(open(f)):
            try: other += int(r["other"])
            except (ValueError, KeyError): pass
    per_run.append((d[9:13], nrob, round((t1 - t0) / 60, 1), n, round(n / (nrob * (t1 - t0) / 60.0), 1)))
res = {}
for k in (2, 1):
    v = allst[k]
    res["losses_%d_robot_runs" % k] = {"n": len(v), "per_robot_minute": round(len(v) / robmin[k], 1) if robmin[k] else None, "len_median": q(v, 0.5), "len_p75": q(v, 0.75), "len_p90": q(v, 0.9),
                                        "len_p99": q(v, 0.99), "len_max": q(v, 1.0), "between_median_s": q(betw[k], 0.5), "between_p25": q(betw[k], 0.25), "between_p75": q(betw[k], 0.75),
                                        "share_over_1.5_s": round(sum(x > 1.5 for x in v) / len(v), 3) if v else None, "share_over_2.5_s": round(sum(x > 2.5 for x in v) / len(v), 3) if v else None}
def fit(pts):
    n = len(pts); sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts); sxx = sum(p[0] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
    b = (n * sxy - sx * sy) / (n * sxx - sx * sx); a = (sy - b * sx) / n
    resid = [p[1] - a - b * p[0] for p in pts]
    return round(a, 2), round(b, 3), round(S.pstdev(resid), 2), n
res["replan_fit_all"] = dict(zip(("fixed_s", "s_per_point", "resid_sd_s", "n"), fit(rp)))
res["replan_fit_two_robots"] = dict(zip(("fixed_s", "s_per_point", "resid_sd_s", "n"), fit([p for p in rp if p[2] == 2])))
res["replan_fit_one_robot"] = dict(zip(("fixed_s", "s_per_point", "resid_sd_s", "n"), fit([p for p in rp if p[2] == 1])))
small = [p[1] for p in rp if p[0] <= 4 and p[0] > 0]; big = [p[1] for p in rp if p[0] >= 20]
res["replan_small_2to4_points"] = {"n": len(small), "median": q(small, 0.5), "min": q(small, 0), "max": q(small, 1)}
res["replan_20plus_points"] = {"n": len(big), "median": q(big, 0.5), "min": q(big, 0), "max": q(big, 1)}
res["replans_planning_few_points_share"] = round(sum(1 for p in rp if 0 < p[0] <= 4) / len(rp), 2)
res["first_plans"] = first
tl = [t[0] for t in tol if t[0] is not None]
res["reach_tol_cm"] = {"min": min(tl), "max": max(tl), "with_pm_min": min(t[1] for t in tol if t[1]), "with_pm_max": max(t[1] for t in tol if t[1]), "stops": len(tl)}
res["loss_lens_two_robot_runs"] = [round(x, 2) for x in allst[2]]; res["loss_lens_one_robot_runs"] = [round(x, 2) for x in allst[1]]
res["replans"] = [(p[0], round(p[1], 2), p[2], p[3]) for p in rp]
res["sources_other_total"] = other; res["per_run"] = per_run
print(json.dumps({k: v for k, v in res.items() if k not in ("loss_lens_two_robot_runs", "loss_lens_one_robot_runs", "replans")}, indent=1))
if len(sys.argv) > 2: json.dump(res, open(sys.argv[2], "w"), indent=1)
