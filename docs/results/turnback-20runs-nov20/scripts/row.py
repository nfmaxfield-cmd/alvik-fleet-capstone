#!/usr/bin/env python3
"""One results.csv row for a finished run folder. row.py CONFIG SEED FOLDER -> appends to results.csv"""
import collections, csv, json, os, sys
T = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(T, "wt_ccb3", "docs", "results", "e23-school-fixes"))
sys.path.insert(0, os.path.join(T, "scripts"))
import acc_table as AT, first_stall as FS
cfg, seed, folder = sys.argv[1], int(sys.argv[2]), sys.argv[3]
r = AT.one(folder)
ev = collections.Counter()
for ln in open(os.path.join(folder, "run", "coordinator.jsonl")):
    try: ev[json.loads(ln).get("event")] += 1
    except ValueError: pass
out = open(os.path.join(folder, "mission_stdout.txt"), errors="replace").read()
capped = os.path.exists(os.path.join(folder, "CAPPED.txt"))
case = json.load(open(os.path.join(folder, "case.json")))
fs = FS.analyse(folder)
row = {"config": cfg, "seed": seed, "finished": int(not capped and str(r.get("result", "")).startswith("done")),
       "result": r.get("result"), "exit": case.get("exit"), "truly": r.get("truly_reached"), "asked": r.get("asked"),
       "false_credits": len(r.get("false_credits") or []), "replans": r.get("replans"), "turnbacks": ev.get("veer", 0),
       "veer_not_at_point": ev.get("veer_not_at_point", 0),
       "tb_first_moving_stall": int(fs["veer_at_first"]), "first_moving_stall_t_run": fs["first_moving_stall_t_run"],
       "first_moving_robot": ",".join(fs["moving"]) if fs["moving"] else "",
       "run_s": r.get("duration_s"), "wall_s": case.get("wall_s"),
       "closest_two_bodies_cm": r.get("closest_two_bodies_cm"), "closest_body_to_open_edge_cm": r.get("closest_body_to_open_edge_cm"),
       "contact": r.get("contact"), "hung_over_edge": len(r.get("hung_over_edge") or []),
       "crash": int(bool(r.get("crashed")) or "Traceback" in out), "crash_why": (r.get("crashed") or "")[:80],
       "old_fix_not_carried": ev.get("old_fix_not_carried", 0), "folder": os.path.basename(folder)}
p = os.path.join(T, "results.csv")
new = not os.path.exists(p)
with open(p, "a", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(row))
    if new: w.writeheader()
    w.writerow(row)
print(json.dumps(row))
