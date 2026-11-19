#!/usr/bin/env python3
"""Every recorded stop (run/stops.csv): how far the camera's position during the dwell was from the point's sticker
coordinates (the site's points file, with the site file's overrides for P09 and P10).
Usage: python3 -I stop_error.py <jayruns folder> <school-conference-points.csv>"""
import csv, math, os, sys, statistics as S
pts = {r["id"]: (float(r["x_cm"]), float(r["y_cm"])) for r in csv.DictReader(open(sys.argv[2]))}
pts["P09"] = (17 * 2.54, 57.5 * 2.54); pts["P10"] = (33 * 2.54, 37.5 * 2.54)
allv = []
for d in sorted(os.listdir(sys.argv[1])):
    f = os.path.join(sys.argv[1], d, "run", "stops.csv")
    if not os.path.exists(f): continue
    v = []
    for r in csv.DictReader(open(f)):
        p = pts[r["point"]]
        e = math.hypot(float(r["cam_x_cm"]) - p[0], float(r["cam_y_cm"]) - p[1])
        v.append((e, r["robot"], r["point"], r["source"], r["plus_minus_cm"]))
    if not v: continue
    allv += [(x[0], d[:15]) + x[1:] for x in v]
    by = {}
    for x in v: by.setdefault("side" if "side" in x[3] else "overhead" if "overhead" in x[3] else "odometry", []).append(x[0])
    print(d[:15], "stops %d;" % len(v), "; ".join("%s: median %.1f cm, worst %.1f cm (n %d)" % (k, S.median(w), max(w), len(w)) for k, w in sorted(by.items())))
    for x in sorted(v, reverse=True):
        if x[0] > 8: print("     %.1f cm from %s (%s, %s, said +-%s cm)" % (x[0], x[2], x[1], x[3], x[4]))
