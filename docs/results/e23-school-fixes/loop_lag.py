"""How far behind was the run program when it read each robot's pose row? Read-only on Jay-Sun's run folders.
run/follow.csv: t_recv (the Pi's clock when run_plan READ the row) and ms (the robot's clock when it SENT it).
t_recv - ms/1000 has a floor (the two clocks' offset plus the quickest delivery); what a row is above that floor is how
late it was read: WiFi delay plus the time it waited for the run program's loop. python3 -I loop_lag.py <jayruns> [out]"""
import csv, glob, json, os, sys
root = sys.argv[1]
out = {}
def q(v, p):
    v = sorted(v)
    return round(v[min(len(v) - 1, int(p * len(v)))], 3)
for run in sorted(glob.glob(os.path.join(root, "*"))):
    f = os.path.join(run, "run", "follow.csv")
    if not os.path.exists(f):
        continue
    by = {}
    for r in csv.DictReader(open(f)):
        try:
            by.setdefault((r["robot"], r["run"].split("-")[0]), []).append((float(r["t_recv"]), float(r["t_recv"]) - float(r["ms"]) / 1000.0, r["state"]))
        except (ValueError, KeyError):
            pass
    late, late_drive = [], []
    for k, v in by.items():
        if len(v) < 50:
            continue
        floor = min(o for _, o, _ in v)
        late += [o - floor for _, o, _ in v]
        late_drive += [o - floor for _, o, s in v if s in ("drive", "turn", "arc")]
    if len(late) >= 100:
        out[os.path.basename(run)] = {"rows": len(late), "late_median_s": q(late, 0.5), "late_p90_s": q(late, 0.9),
                                      "late_p99_s": q(late, 0.99), "late_max_s": q(late, 1.0),
                                      "over_0.5_s_pct": round(100.0 * sum(a > 0.5 for a in late) / len(late), 1),
                                      "over_1.0_s_pct": round(100.0 * sum(a > 1.0 for a in late) / len(late), 1),
                                      "while_moving_p90_s": q(late_drive, 0.9) if late_drive else None}
for k, v in out.items():
    print(k[9:15], v)
if len(sys.argv) > 2:
    json.dump(out, open(sys.argv[2], "w"), indent=1)
