"""How long did a robot's step report wait before the run program's loop handled it? Read-only on the run folders.
run/step.csv t_recv: when the link's listener thread heard the row. run/coordinator.jsonl "step_report" t: when the main
loop handled it. The difference is the loop's lag (it also forwards camera fixes, so the same lag delays every fix).
python3 -I drain_lag.py <jayruns> [out.json]"""
import csv, glob, json, os, sys
root = sys.argv[1]
out = {}
def q(v, p):
    v = sorted(v)
    return round(v[min(len(v) - 1, int(p * len(v)))], 3)
for run in sorted(glob.glob(os.path.join(root, "*"))):
    fs, fc = os.path.join(run, "run", "step.csv"), os.path.join(run, "run", "coordinator.jsonl")
    if not (os.path.exists(fs) and os.path.exists(fc)):
        continue
    heard = {}
    for r in csv.DictReader(open(fs)):
        try:
            heard.setdefault((r["robot"], int(r["id"]), r["state"]), []).append(float(r["t_recv"]))
        except (ValueError, KeyError):
            pass
    lag, plans = [], []
    for ln in open(fc):
        try:
            e = json.loads(ln)
        except ValueError:
            continue
        if e.get("event") == "replan":
            plans.append((e["t"] - e["cpu_s"], e["t"]))
        if e.get("event") != "step_report":
            continue
        ts = heard.get((e.get("robot"), e.get("id"), e.get("state")))
        if not ts:
            continue
        d = min((e["t"] - t for t in ts if e["t"] - t >= -0.01), default=None)
        if d is not None and d < 30:
            lag.append((e["t"], d))
    if len(lag) < 30:
        continue
    inp = [d for t, d in lag if any(a <= t <= b for a, b in plans)]
    outp = [d for t, d in lag if not any(a <= t <= b for a, b in plans)]
    v = [d for _, d in lag]
    out[os.path.basename(run)] = {"reports": len(v), "lag_median_s": q(v, 0.5), "lag_p90_s": q(v, 0.9), "lag_max_s": q(v, 1.0),
                                  "over_0.5_s_pct": round(100.0 * sum(a > 0.5 for a in v) / len(v), 1),
                                  "while_planning": {"n": len(inp), "median_s": q(inp, 0.5), "p90_s": q(inp, 0.9), "max_s": q(inp, 1.0)} if inp else None,
                                  "not_planning": {"n": len(outp), "median_s": q(outp, 0.5), "p90_s": q(outp, 0.9), "max_s": q(outp, 1.0)} if outp else None}
for k, v in out.items():
    print(k[9:15], v)
if len(sys.argv) > 2:
    json.dump(out, open(sys.argv[2], "w"), indent=1)
