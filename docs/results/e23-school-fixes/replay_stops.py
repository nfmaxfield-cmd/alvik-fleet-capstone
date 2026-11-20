"""The stops the school runs recorded (16 and 18 Nov), judged again by run_plan's own record_point: as e23 was built
(MIRLT_REACH_TOL=e23) and after the audit's fix (bounded, the default). Read-only on the run folders.

    python3 -I replay_stops.py <jayruns> <repo> [out.json] [fixture.json]

out.json: every stop with both verdicts and tolerances; fixture.json: the stops alone (run, robot, point, miss, +-,
source), which tools/fleet/tests/data/e23_real_stops.json is a copy of.
Changelog: 2026-11-19e23c first version (from the audit's replay_stops.py, which copied the rule instead of running it)."""
import csv
import glob
import json
import math
import os
import sys

root, repo = sys.argv[1], sys.argv[2]
sys.path.insert(0, os.path.join(repo, "tools", "fleet"))
import run_plan as RP           # noqa: E402
import sites as SI              # noqa: E402
import sticker_points as SP     # noqa: E402

pts = {k: v[:2] for k, v in SP.points_cm(SI.load("school-conference"))[0].items()}
stops = []
for run in sorted(glob.glob(os.path.join(root, "*"))):
    f = os.path.join(run, "run", "points.csv")
    if not os.path.exists(f):
        continue
    for r in csv.DictReader(open(f)):
        lab = r["point"].replace("point ", "")
        if lab not in pts or not r["x"]:
            continue
        stops.append({"run": os.path.basename(run), "robot": r["robot"], "point": lab,
                      "miss_cm": round(math.hypot(float(r["x"]) - pts[lab][0], float(r["y"]) - pts[lab][1]), 1),
                      "pm_cm": float(r["plus_minus_cm"] or 0.0), "source": r["source"]})


def coord():
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.log = lambda ev, **k: None
    c.say = lambda *a, **k: None
    c.t_run, c.outdir = RP.now() - 100.0, None
    c.point_csv = lambda rec: None
    c.cam_info = lambda: {"moved": {}}
    c.cam_unc = lambda f: f["unc"]
    return c


def replay(bounded):
    RP.REACH_TOL_BOUNDED = bounded
    out, cur, c = [], None, None
    for s in stops:
        if s["run"] != cur:
            cur, c = s["run"], coord()
        b = RP.Bot(s["robot"])
        t1 = RP.now()
        b.pose = (s["miss_cm"], 0.0, 0.0)
        b.fix_log = [dict(t=t1 - 1.0, x=s["miss_cm"], y=0.0, unc=s["pm_cm"], cams=["overhead"])]
        ok = RP.Coordinator.record_point(c, b, "point " + s["point"], t1 - 2.0, t1, target=[0.0, 0.0])
        rec = c.points[-1] if ok else c.not_reached[-1]
        out.append((bool(ok), rec["reach_tol_cm"], rec.get("reach_tol_with_pm_cm")))
    return out


old, new = replay(False), replay(True)
rows = [dict(s, e23_credited=o[0], e23_tol_cm=o[1], credited=n[0], tol_cm=n[1], tol_with_pm_cm=n[2])
        for s, o, n in zip(stops, old, new)]
print("%d stops; refused by e23 as built: %d; refused now: %d" % (
    len(rows), sum(not r["e23_credited"] for r in rows), sum(not r["credited"] for r in rows)))
print("largest tolerance: e23 %.1f cm, now %.1f cm" % (max(r["e23_tol_cm"] for r in rows), max(r["tol_cm"] for r in rows)))
for r in rows:
    if not r["credited"] or not r["e23_credited"]:
        print("  %s %-5s %s  %5.1f cm off, +-%4.1f | e23: tol %5.1f %-8s | now: tol %4.1f / %4.1f %s" % (
            r["run"][9:15], r["robot"], r["point"], r["miss_cm"], r["pm_cm"], r["e23_tol_cm"],
            "credited" if r["e23_credited"] else "refused", r["tol_cm"], r["tol_with_pm_cm"],
            "credited" if r["credited"] else "refused"))
if len(sys.argv) > 3:
    json.dump({"stops": rows}, open(sys.argv[3], "w"), indent=1)
if len(sys.argv) > 4:
    json.dump({"what": "the 131 stops of the school runs of 16 and 18 Nov: distance of the stop's recorded position "
                       "from its point's typed position, and the stop's stated +- (replay_stops.py)",
               "stops": stops}, open(sys.argv[4], "w"), indent=0)
