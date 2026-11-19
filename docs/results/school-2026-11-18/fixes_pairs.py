#!/usr/bin/env python3
"""Per run: (a) how often each robot got a camera fix while the plan ran (bridge/fixes.csv: median and 90th-percentile
gap between fixes, share of the time with no fix for over 1 s), by camera; (b) the closest the two robots' middles came,
from the robots' own fused pose (run/follow.csv, both interpolated to 0.2 s) and from the camera (same-picture pairs).
Usage: python3 -I fixes_pairs.py <jayruns folder>"""
import csv, json, math, os, sys, bisect, collections, statistics as S
ROOT = sys.argv[1]
def q(v, p):
    v = sorted(v); return v[min(len(v) - 1, int(p * len(v)))] if v else float("nan")
for d in sorted(os.listdir(ROOT)):
    cj = os.path.join(ROOT, d, "run", "coordinator.jsonl")
    if not os.path.exists(cj): continue
    L = [json.loads(l) for l in open(cj) if l.startswith("{")]
    t0 = next((r["t"] for r in L if r.get("event") == "message" and "plan plan-" in r.get("text", "") and "started" in r["text"]), None)
    if t0 is None: continue
    t1 = L[-1]["t"]
    out = [d[:15]]
    fx = os.path.join(ROOT, d, "bridge", "fixes.csv")
    by = collections.defaultdict(list); cams = collections.defaultdict(collections.Counter)
    if os.path.exists(fx):
        for r in csv.DictReader(open(fx)):
            t = float(r["t"])
            if t0 <= t <= t1: by[r["robot"]].append(t); cams[r["robot"]][r["cams"]] += 1
    for rb in sorted(by):
        ts = sorted(by[rb]); g = [b - a for a, b in zip(ts, ts[1:])]
        if not g: continue
        dark = sum(x for x in g if x > 1.0)
        out.append("%s: %d fixes, gap median %.2f s, 90%% %.2f s, max %.1f s, %.0f %% of the time over 1 s since the last fix; by camera %s" % (
            rb, len(ts), S.median(g), q(g, 0.9), max(g), 100 * dark / (t1 - t0), dict(cams[rb])))
    fo = os.path.join(ROOT, d, "run", "follow.csv")
    tr = collections.defaultdict(list); used = {}
    if os.path.exists(fo):
        for r in csv.DictReader(open(fo)):
            try:
                t = float(r["t_recv"])
                if r["state"] == "nopose" or t < t0: continue
                tr[r["robot"]].append((t, float(r["x_cm"]), float(r["y_cm"])))
                used[(r["robot"], r["run"])] = (int(r["fixes_used"]), int(r["fixes_rejected"]))
            except (ValueError, KeyError): pass
    u = collections.Counter(); rj = collections.Counter()
    for (rb, run), (a, b) in used.items(): u[rb] += a; rj[rb] += b
    out.append("fixes used / refused on the robots: %s" % {rb: (u[rb], rj[rb]) for rb in sorted(u)})
    names = sorted(tr)
    if len(names) == 2:
        def interp(seq, t):
            ts = [s[0] for s in seq]; i = bisect.bisect_left(ts, t)
            if i <= 0 or i >= len(seq): return None
            a, b = seq[i - 1], seq[i]
            if b[0] - a[0] > 2.0: return None
            f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0
            return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]))
        best = None; t = t0
        while t < t1:
            p, r2 = interp(tr[names[0]], t), interp(tr[names[1]], t)
            if p and r2:
                dd = math.hypot(p[0] - r2[0], p[1] - r2[1])
                if best is None or dd < best[0]: best = (dd, t - t0, p, r2)
            t += 0.2
        if best: out.append("closest middles by their own poses: %.1f cm at %.0f s (%s at %.0f,%.0f; %s at %.0f,%.0f)" % (
            best[0], best[1], names[0], best[2][0], best[2][1], names[1], best[3][0], best[3][1]))
    print("\n   ".join(out))
