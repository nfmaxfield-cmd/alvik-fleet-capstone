"""How far from its point was every stop the school runs recorded (16 and 18 Nov)? Read-only on the run folders.
run/points.csv: the stop's position (the camera's median during the stop, else the robot's own pose) and its stated +-;
the point's typed position from the site's sticker file. tol = where drives end (2.9 cm) + the stop's own +- (e23's
reach_tol_cm). python3 -I real_stop_miss.py <jayruns> <repo> [out.json]
Changelog: 2026-11-19 first version; 2026-11-19e23c seed_miss, seed_miss_plus_pm, median_pm_cm (the audit's fix)."""
import csv, glob, json, math, os, sys
root, repo = sys.argv[1], sys.argv[2]
sys.path.insert(0, os.path.join(repo, "tools", "fleet"))
import sticker_points as SP, sites as SI
pts = {k: v[:2] for k, v in SP.points_cm(SI.load("school-conference"))[0].items()}
rows = []
for run in sorted(glob.glob(os.path.join(root, "*"))):
    f = os.path.join(run, "run", "points.csv")
    if not os.path.exists(f):
        continue
    for r in csv.DictReader(open(f)):
        lab = r["point"].replace("point ", "")
        if lab not in pts or not r["x"]:
            continue
        miss = math.hypot(float(r["x"]) - pts[lab][0], float(r["y"]) - pts[lab][1])
        pm = float(r["plus_minus_cm"] or 0.0)
        rows.append({"run": os.path.basename(run), "robot": r["robot"], "point": lab, "miss_cm": round(miss, 1), "pm_cm": pm,
                     "tol_cm": round(2.94 + pm, 1), "source": r["source"], "refused": miss > 2.94 + pm})
m = sorted(q["miss_cm"] for q in rows)
ok = [q for q in rows if not q["refused"]]
bad = [q for q in rows if q["refused"]]
out = {"stops": len(rows), "median_miss_cm": m[len(m) // 2], "p90_miss_cm": m[int(0.9 * len(m))], "refused": len(bad),
       "refused_list": sorted(bad, key=lambda q: -q["miss_cm"]),
       "kept_max_miss_cm": max(q["miss_cm"] for q in ok) if ok else None,
       "share_under_3cm": round(sum(v <= 3.0 for v in m) / len(m), 3)}
# (e23c, audit of e23) the two seeds run_plan uses: median and median absolute deviation of the miss, and of the miss
# plus the stop's own +- (MISS_SEED_CM, MISS_PM_SEED_CM)
import statistics


def _seed(v):
    med = statistics.median(v)
    return [round(med, 2), round(statistics.median([abs(x - med) for x in v]), 2)]


_raw = []
for run in sorted(glob.glob(os.path.join(root, "*"))):
    f = os.path.join(run, "run", "points.csv")
    if os.path.exists(f):
        for r in csv.DictReader(open(f)):
            lab = r["point"].replace("point ", "")
            if lab in pts and r["x"]:
                _raw.append((math.hypot(float(r["x"]) - pts[lab][0], float(r["y"]) - pts[lab][1]),
                             float(r["plus_minus_cm"] or 0.0)))
out["seed_miss"] = _seed([a for a, _ in _raw])
out["seed_miss_plus_pm"] = _seed([a + b for a, b in _raw])
out["median_pm_cm"] = round(statistics.median([b for _, b in _raw]), 2)
print(json.dumps({k: v for k, v in out.items() if k != "refused_list"}))
for q in out["refused_list"]:
    print("  ", q["run"][9:15], q["robot"], q["point"], q["miss_cm"], "tol", q["tol_cm"], q["source"])
if len(sys.argv) > 3:
    json.dump(out, open(sys.argv[3], "w"), indent=1)
