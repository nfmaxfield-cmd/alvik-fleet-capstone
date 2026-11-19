"""Camera-probe open times in Jay-Sun's 25 run logs: the gap between successive failed-open warnings in each start
(OpenCV's own clock in the line), read-only. python3 -I open_times.py <jayruns folder>"""
import glob, os, re, sys, json, statistics as st
root = sys.argv[1]
gaps, per = [], {}
for f in sorted(glob.glob(os.path.join(root, "*", "run_plan_stdout.txt"))):
    ts = [float(m.group(1)) for m in re.finditer(r"\[ WARN:0@([\d.]+)\] global cap_gstreamer.cpp:2839", open(f, errors="replace").read())]
    g = [round(b - a, 3) for a, b in zip(ts, ts[1:]) if b > a]
    per[os.path.basename(os.path.dirname(f))] = {"probes": len(ts), "first_s": ts[0] if ts else None, "gaps": g}
    gaps += g
gaps.sort()
q = lambda p: gaps[min(len(gaps) - 1, int(p * len(gaps)))]
out = {"n": len(gaps), "min": gaps[0], "median": st.median(gaps), "p95": q(0.95), "p99": q(0.99), "max": gaps[-1],
       "mean": round(st.mean(gaps), 3), "sd": round(st.pstdev(gaps), 3), "per_run": per}
print(json.dumps({k: v for k, v in out.items() if k != "per_run"}))
for k, v in per.items():
    print(k, v["probes"], v["first_s"], max(v["gaps"]) if v["gaps"] else None)
json.dump(out, open(sys.argv[2], "w"), indent=1) if len(sys.argv) > 2 else None
