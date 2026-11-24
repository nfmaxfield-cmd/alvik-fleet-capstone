#!/usr/bin/env python3
"""Per robot (by WiFi name) and run: (a) camera distance / wheel distance over each drive step the run checked late
('move_check_late': camera_cm, wheel_cm; steps of 15 cm or more), a direct check of wheel_scale; (b) the error found
when a camera saw the robot again after driving unseen ('blind_end': err_cm after dist_cm, drives of 5 cm or more);
(c) the settings the robot reported (run/param.csv). Read-only.
Usage: python3 -I track_err.py <jayruns23 folder> [out.json]   Changelog: 2026-11-24 first version."""
import csv, json, os, re, sys, collections, statistics as S
ROOT = sys.argv[1]; out = []
for d in sorted(os.listdir(ROOT)):
    cj = os.path.join(ROOT, d, "run", "coordinator.jsonl")
    if not os.path.exists(cj): continue
    so = open(os.path.join(ROOT, d, "run_plan_stdout.txt"), errors="replace").read()
    names = dict((b, a) for a, b in re.findall(r"(AL-\w+) is (Alpha|Bravo)", so))
    par = {}
    for r in csv.DictReader(open(os.path.join(ROOT, d, "run", "param.csv"), newline="")):
        if r["name"] in ("wheel_scale", "gyro_scale", "gyro_drift_dps"): par[(r["robot"], r["name"])] = r["value"]
    ratio = collections.defaultdict(list); blind = collections.defaultdict(list); lead = {}
    m = re.search(r"STOP LEAD[^:]*: (.*)", so)
    for l in open(cj, errors="replace"):
        try: e = json.loads(l)
        except ValueError: continue
        if e.get("event") == "move_check_late" and e.get("wheel_cm", 0) >= 15 and e.get("camera_cm") is not None:
            ratio[e["robot"]].append(e["camera_cm"] / e["wheel_cm"])
        if e.get("event") == "blind_end" and e.get("dist_cm", 0) >= 5:
            blind[e["robot"]].append((e["err_cm"], e["dist_cm"]))
    for rb in sorted(names):
        r = ratio[rb]; b = blind[rb]
        o = {"run": d[9:13], "slot": rb, "robot": names[rb], "wheel_scale": par.get((rb, "wheel_scale")), "gyro_scale": par.get((rb, "gyro_scale")),
             "gyro_drift_dps": par.get((rb, "gyro_drift_dps")), "steps": len(r), "cam_over_wheel_median": round(S.median(r), 4) if r else None,
             "cam_over_wheel_iqr": [round(x, 3) for x in S.quantiles(r, n=4)[::2]] if len(r) >= 4 else None,
             "blind_n": len(b), "blind_err_median_cm": round(S.median(x[0] for x in b), 2) if b else None,
             "blind_err_per_10cm": round(10 * sum(x[0] for x in b) / sum(x[1] for x in b), 2) if b else None,
             "stop_lead": (m.group(1) if m else "")}
        out.append(o); print(o)
for nm in sorted(set(o["robot"] for o in out)):
    for label, sel in (("12:10-15:19", lambda o: o["run"] < "1600"), ("16:49 on", lambda o: o["run"] >= "1600")):
        v = [o for o in out if o["robot"] == nm and sel(o) and o["cam_over_wheel_median"]]
        if v: print(nm, label, "runs %d: camera/wheel median of medians %.4f; unseen-drive error per 10 cm %.2f" % (
            len(v), S.median(o["cam_over_wheel_median"] for o in v), S.median(o["blind_err_per_10cm"] for o in v if o["blind_err_per_10cm"] is not None)))
if len(sys.argv) > 2: json.dump(out, open(sys.argv[2], "w"), indent=1)
