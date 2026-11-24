#!/usr/bin/env python3
"""Picture of the plate contact in the 12:19 run of 23 Nov: Alpha's first 30 s, where the camera saw it, where its own
pose said it was, the leg it was sent on, and the zone 1 outlet plate. Read-only.
Usage: python3 -I plate_run2.py <run folder 20261123-121948-visit-robots> <out.png>   Changelog: 2026-11-24 first version."""
import csv, json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
RUN, OUT = sys.argv[1], sys.argv[2]
IN = 2.54; INK, MUTED, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"; BLUE, ORANGE = "#2a78d6", "#eb6834"
t0 = None
for l in open(os.path.join(RUN, "run", "coordinator.jsonl")):
    if '"message"' in l and "plan plan-" in l and "started" in l: t0 = json.loads(l)["t"]; break
cam = [(float(r["t"]) - t0, float(r["x_cm"]), float(r["y_cm"])) for r in csv.DictReader(open(os.path.join(RUN, "run", "camera_track.csv"))) if r["robot"] == "Alpha" and 0 <= float(r["t"]) - t0 <= 20]
own = [(float(r["t_recv"]) - t0, float(r["x_cm"]), float(r["y_cm"])) for r in csv.DictReader(open(os.path.join(RUN, "run", "follow.csv"))) if r["robot"] == "Alpha" and r["state"] != "nopose" and 0 <= float(r["t_recv"]) - t0 <= 16.7]
fig, ax = plt.subplots(figsize=(9.5, 6.2), dpi=120); fig.patch.set_facecolor(SURF); ax.set_facecolor(SURF)
c, s = (25.22, 33.0), (2.44, 6.62)
ax.add_patch(Rectangle(((c[0] - s[0] / 2) * IN, (c[1] - s[1] / 2) * IN), s[0] * IN, s[1] * IN, facecolor="#d9d8d3", edgecolor=MUTED, zorder=1))
ax.text(c[0] * IN, (c[1] + s[1] / 2) * IN + 2, "zone 1 outlet plate\n(keep-out)", ha="center", fontsize=8, color=INK)
ax.plot([116.92, 34.92], [102.77, 31.86], color=MUTED, lw=1.5, ls="--", zorder=2, label="the leg it was sent on (to P01)")
ax.plot([p[1] for p in own], [p[2] for p in own], color=ORANGE, lw=2, zorder=3, label="where its own pose said it was")
ax.plot([p[1] for p in cam], [p[2] for p in cam], "-o", color=BLUE, lw=2, ms=5, zorder=4, label="where the camera saw it")
ax.add_patch(Rectangle((57.75 - 4.75, 86.61 - 4.75), 9.5, 9.5, angle=0, fill=False, edgecolor=BLUE, lw=1.2, zorder=4))
ax.text(46, 96, "it stopped here: tilt stop,\n3.4 deg for 2 s (19.1 s)", fontsize=8, color=INK, ha="right")
ax.text(118, 106, "start", fontsize=8, color=INK); ax.text(30, 27, "P01", fontsize=8, color=INK)
ax.set_aspect("equal"); ax.set_xlim(20, 130); ax.set_ylim(22, 114); ax.set_xlabel("x along the table, cm"); ax.set_ylabel("y, cm")
for sp in ("top", "right"): ax.spines[sp].set_visible(False)
ax.grid(color=GRID, lw=0.8); ax.set_axisbelow(True); ax.legend(frameon=False, fontsize=8, loc="lower right")
ax.set_title("23 Nov 12:19, Alpha (AL-E7DC), first 20 s: its heading was 27 deg off after a gyro jump, and it drove onto the plate", loc="left", fontsize=9.5, color=INK)
fig.text(0.01, 0.01, "Measured: run/camera_track.csv (roof tag read by camera 1), run/follow.csv (the robot's own pose), the step sent at 9.2 s (coordinator.jsonl), the site file's keep-out. "
         "The square is the 9.5 cm body where it stopped. Source: run folder 20261123-121948-visit-robots.", fontsize=7, color=MUTED, wrap=True)
fig.tight_layout(rect=(0, 0.04, 1, 1)); fig.savefig(OUT)
