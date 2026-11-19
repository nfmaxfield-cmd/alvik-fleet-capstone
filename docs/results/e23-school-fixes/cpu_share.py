"""Where the Pi's processor time went in each run: cpu.csv (each program's share of one core, and each core's load),
read-only on the run folders. python3 -I cpu_share.py <jayruns> [out.json]"""
import csv, glob, json, os, statistics as st, sys
root = sys.argv[1]
out = {}
for run in sorted(glob.glob(os.path.join(root, "20261118-*"))):
    f = os.path.join(run, "cpu.csv")
    if not os.path.exists(f):
        continue
    rows = list(csv.DictReader(open(f)))
    if len(rows) < 30:
        continue
    rec = {}
    for k in rows[0].keys():
        if k in ("t", "time"):
            continue
        v = []
        for r in rows:
            try:
                v.append(float(r[k]))
            except (ValueError, TypeError):
                pass
        if len(v) >= 20:
            v.sort()
            rec[k] = {"median": round(st.median(v), 1), "p90": round(v[int(0.9 * len(v))], 1)}
    out[os.path.basename(run)] = rec
for k, v in out.items():
    print(k[9:15], {a: b["median"] for a, b in v.items()})
if len(sys.argv) > 2:
    json.dump(out, open(sys.argv[2], "w"), indent=1)
