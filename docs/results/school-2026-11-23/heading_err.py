#!/usr/bin/env python3
"""Own heading against the camera's while a robot stands (so picture delay does not matter).
For every camera read of a robot (run/camera_track.csv, else bridge/fixes.csv) where the robot's own heading
(run/follow.csv th_deg) changed under 1 deg in the 1.2 s before and after: difference = camera heading - own heading.
Prints per run and robot the median and largest difference and every stretch where it stayed over 10 deg.
Usage: python3 -I heading_err.py <jayruns23 folder> [out.json]   Read-only. Changelog: 2026-11-24 first version."""
import csv, json, os, sys, bisect, collections, statistics as S
ROOT = sys.argv[1]; res = {}
def wrap(a): return ((a + 180) % 360) - 180
for d in sorted(os.listdir(ROOT)):
    fo = os.path.join(ROOT, d, "run", "follow.csv")
    if not os.path.exists(fo): continue
    cj = os.path.join(ROOT, d, "run", "coordinator.jsonl")
    t0 = None
    for l in open(cj):
        if '"message"' in l and "plan plan-" in l and "started" in l: t0 = json.loads(l)["t"]; break
    if t0 is None: continue
    own = collections.defaultdict(list)
    for r in csv.DictReader(open(fo, newline="")):
        try:
            if r["state"] != "nopose": own[r["robot"]].append((float(r["t_recv"]), float(r["th_deg"])))
        except ValueError: pass
    cam = collections.defaultdict(list)
    f = os.path.join(ROOT, d, "run", "camera_track.csv"); hk = "heading_deg"
    if not os.path.exists(f): f = os.path.join(ROOT, d, "bridge", "fixes.csv"); hk = "heading"
    for r in csv.DictReader(open(f, newline="")):
        try: cam[r["robot"]].append((float(r["t"]), float(r[hk])))
        except ValueError: pass
    for rb in sorted(own):
        o = own[rb]; ts = [x[0] for x in o]; diffs = []
        for t, h in cam[rb]:
            if t < t0: continue
            i0 = bisect.bisect_left(ts, t - 1.2); i1 = bisect.bisect_right(ts, t + 1.2)
            w = o[i0:i1]
            if len(w) < 8 or w[0][0] > t - 0.8 or w[-1][0] < t + 0.8: continue
            if max(abs(wrap(x[1] - w[0][1])) for x in w) > 1.0: continue
            diffs.append((round(t - t0, 1), round(wrap(h - w[len(w) // 2][1]), 1)))
        if not diffs: continue
        a = [abs(x[1]) for x in diffs]
        ep = []; cur = None
        for t, df in diffs:
            if abs(df) > 10:
                if cur and t - cur[1] < 20: cur[1] = t; cur[2] = max(cur[2], abs(df), key=abs); cur[3] += 1
                else:
                    cur = [t, t, abs(df), 1]; ep.append(cur)
        res["%s %s" % (d[:15], rb)] = {"n": len(a), "median": S.median(a), "p95": sorted(a)[int(0.95 * (len(a) - 1))], "max": max(a),
                                         "over10": sum(1 for x in a if x > 10), "episodes": ep}
        print(d[:15], rb, "standing reads %d: |camera - own heading| median %.1f, 95%% %.1f, max %.1f deg; over 10 deg %d; stretches (from s, to s, worst deg, reads): %s" % (
            len(a), S.median(a), sorted(a)[int(0.95 * (len(a) - 1))], max(a), sum(1 for x in a if x > 10), ep))
if len(sys.argv) > 2: json.dump(res, open(sys.argv[2], "w"), indent=1)
