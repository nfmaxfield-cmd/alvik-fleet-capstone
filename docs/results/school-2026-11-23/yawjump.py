#!/usr/bin/env python3
"""Jumps in the robots' gyro yaw (run/raw.csv, yaw_deg) that the gyro rate (gz) and the wheels do not explain.
For every pair of raw rows less than 0.3 s apart: jump = change in yaw_deg minus gz x dt. A jump over 5 deg is listed with
how long the robot had stood still before it (wheel angles unchanged) and the mean gz while it stood (the gyro's bias),
so 'bias x standing time' can be compared with the jump. Read-only.
Usage: python3 -I yawjump.py <jayruns23 folder> [out.json]
Changelog: 2026-11-24 first version."""
import csv, json, os, sys, collections
ROOT = sys.argv[1]; out = []
for d in sorted(os.listdir(ROOT)):
    f = os.path.join(ROOT, d, "run", "raw.csv")
    if not os.path.exists(f): continue
    prev = {}; still = {}; last = {}; n = collections.Counter(); rows_n = collections.Counter()
    for r in csv.DictReader(open(f, newline="")):
        try:
            rb = r["robot"]; t = float(r["t_recv"]); ms = float(r["ms"]); yaw = float(r["yaw_deg"]); gz = float(r["gz"])
            wl = float(r["wl_deg"]); wr = float(r["wr_deg"]); th = float(r["th_deg"])
        except (ValueError, KeyError, TypeError): continue
        rows_n[rb] += 1
        p = prev.get(rb)
        if p and 0 < ms - p[1] < 300:
            dt = (ms - p[1]) / 1000.0
            dy = ((yaw - p[2] + 180) % 360) - 180
            jump = dy - 0.5 * (gz + p[3]) * dt
            s = still.get(rb)
            if abs(jump) > 5.0:
                l = last.get(rb)
                if (s is None or s[2] < 5) and l and p[1] - l[3] < 1000: s = l      # the standing stretch that just ended
                out.append({"run": d[:15], "robot": rb, "t": round(t, 2), "jump_deg": round(jump, 1),
                            "dyaw": round(dy, 1), "gz": gz, "dt": round(dt, 3),
                            "stood_s": round((s[3] - s[0]) / 1000.0, 1) if s else 0.0,
                            "bias_dps": round(s[1] / s[2], 3) if s and s[2] else None,
                            "bias_x_stood": round(s[1] / s[2] * (s[3] - s[0]) / 1000.0, 1) if s and s[2] else None})
                n[rb] += 1
            if wl == p[4] and wr == p[5] and abs(gz) < 3.0:
                if s is None or s is last.get(rb): s = still[rb] = [p[1], 0.0, 0, ms]
                s[1] += gz; s[2] += 1; s[3] = ms
            else:
                if still.get(rb) is not None and still[rb][2] >= 5: last[rb] = still[rb]
                still[rb] = None
        else:
            still[rb] = None
        prev[rb] = (t, ms, yaw, gz, wl, wr)
    print(d[:15], "raw rows", dict(rows_n), "yaw jumps over 5 deg:", dict(n))
for o in out: print(o)
if len(sys.argv) > 2: json.dump(out, open(sys.argv[2], "w"), indent=1)
