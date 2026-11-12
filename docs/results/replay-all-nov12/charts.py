#!/usr/bin/env python3
"""The report's pictures. Logs only.

    python3 charts.py RUNS_ROOT RESULTS_DIR OUT_DIR

RESULTS_DIR: this folder (robots_w1.csv, drift_long.csv, arc_k_scale_in_use.json, elim_0514.json); RUNS_ROOT: the
unpacked runs (for the 5 Nov 14:55 scene). Writes the PNGs named in report.md.

Changelog
  2026-11-12 v1  first version (replay-all-nov12)
"""
import csv
import json
import os
import statistics as st
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402

VERSION = "charts-2026-11-12v1"
ROOT, RES, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
OFF, ON, INK, MUTED = "#eb6834", "#2a78d6", "#0b0b0b", "#52514e"
ROBOT = {"Left": "#2952CC", "Center": "#C2410C", "Right": "#7A3EC8"}     # tools/fleet/sites/home.json
plt.rcParams.update({"font.size": 8, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
B = 1.7944e9


def label(run):
    mon = " Oct" if run[4:6] == "10" else " Nov"
    return "%s%s %s:%s" % (run[6:8], mon, run[9:11], run[11:13])


def foot(fig, text):
    fig.text(0.01, 0.01, text, fontsize=6.3, color=MUTED, wrap=True)


rob = list(csv.DictReader(open(os.path.join(RES, "robots_w1.csv"))))

# 1. longest unnamed while the camera saw it, off vs on, every run and robot with a spell
pairs = {}
for r in rob:
    pairs.setdefault((r["run"], r["robot"]), {})[r["mode"]] = float(r["unnamed_longest_s"])
rows = [(k, v) for k, v in sorted(pairs.items()) if max(v.values()) >= 3.0]
fig, ax = plt.subplots(figsize=(8, 0.28 * len(rows) + 1.4))
for i, ((run, n), v) in enumerate(rows):
    a, b = max(0.5, v["off"]), max(0.5, v["on"])
    ax.plot([a, b], [i, i], color="#c9c7c1", lw=1.5, zorder=1)
    ax.scatter([a], [i], color=OFF, s=26, zorder=2, label="e17 off (as the runs had it)" if i == 0 else None)
    ax.scatter([b], [i], color=ON, s=26, zorder=3, label="e17 on" if i == 0 else None)
ax.set_yticks(range(len(rows)))
ax.set_yticklabels(["%s  %s" % (label(run), n) for (run, n), _ in rows])
ax.invert_yaxis()
ax.set_xscale("log")
ax.set_xlabel("longest time the tracking layer named nothing while the bridge saw the robot (s, log scale)")
ax.legend(frameon=False, loc="lower right")
ax.set_title("Every uploaded home free roam, replayed through the tracking layer: e17 off vs on", loc="left", color=INK)
foot(fig, "Measured: each run's camera sightings (bridge/track.csv) and own poses (follow.csv), fed once a picture as "
          "the Pi did, through tracking_layer.py (3f37317); replay_one.py. Rows: run and robot with a spell of 3 s or more.")
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(os.path.join(OUT, "replay-all-feature-longest-unnamed-off-vs-on-chart.png"), dpi=150)
plt.close(fig)

# 2. own vs camera gap, per run and robot (as the runs had it: e17 cannot change it in a replay)
offr = [r for r in rob if r["mode"] == "off" and r["gap_p50"]]
fig, ax = plt.subplots(figsize=(8, 0.26 * len(offr) + 1.5))
for i, r in enumerate(offr):
    p50, p90, p99, mx = (float(r[k]) for k in ("gap_p50", "gap_p90", "gap_p99", "gap_max"))
    c = ROBOT.get(r["robot"], INK)
    ax.plot([p50, p99], [i, i], color=c, lw=1.2, alpha=0.6)
    ax.plot([p90, p90], [i - 0.25, i + 0.25], color=c, lw=1.5)
    ax.scatter([p50], [i], color=c, s=18, zorder=3)
    ax.scatter([mx], [i], color=c, s=18, marker="x", zorder=3)
for x, txt in ((6.0, "robot's own fix gate (about 6 cm)"), (25.0, "old gate ceiling (25 cm)")):
    ax.axvline(x, color=MUTED, lw=0.8, ls="--")
    ax.text(x, -1.0, txt, fontsize=6.5, color=MUTED, ha="center")
ax.set_yticks(range(len(offr)))
ax.set_yticklabels(["%s  %s" % (label(r["run"]), r["robot"]) for r in offr])
ax.invert_yaxis()
ax.set_xscale("log")
ax.set_xlabel("own pose to the camera's sighting (cm, log scale): dot = median, tick = 90th percentile, "
              "line end = 99th, x = largest")
ax.set_title("Own pose against the camera, every sighting of every run", loc="left", color=INK)
foot(fig, "Measured: every bridge sighting (bridge/track.csv) against the robot's own pose at that picture's time "
          "(follow.csv, looked up as the run did). The runs' own values: a replay cannot change the robots' poses.")
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(os.path.join(OUT, "replay-all-problem-own-vs-camera-gap-per-run-chart.png"), dpi=150)
plt.close(fig)

# 3. the drift record: three panels, robot by robot across runs and dates
dl = list(csv.DictReader(open(os.path.join(RES, "drift_long.csv"))))
scale = json.load(open(os.path.join(RES, "arc_k_scale_in_use.json")))
fig, axs = plt.subplots(1, 3, figsize=(10.5, 3.9))
offs = {"Left": -0.22, "Center": 0.0, "Right": 0.22}


def panel(ax, measure, fn, runs, title, ylab, minn=5):
    seen = set()
    for n in ("Left", "Center", "Right"):
        for i, run in enumerate(runs):
            v = sorted(x for x in (fn(r) for r in dl if r["measure"] == measure and r["robot"] == n and
                                   r["run"] == run) if x is not None)
            if len(v) < minn:
                continue
            q1, med, q3 = v[len(v) // 4], st.median(v), v[(3 * len(v)) // 4]
            ax.plot([i + offs[n]] * 2, [q1, q3], color=ROBOT[n], lw=1.5, alpha=0.6)
            ax.scatter([i + offs[n]], [med], color=ROBOT[n], s=22, zorder=3, label=None if n in seen else n)
            seen.add(n)
    ax.set_xticks(range(len(runs)))
    ax.set_xticklabels([label(r) for r in runs], rotation=50, ha="right", fontsize=6.5)
    ax.set_title(title, loc="left", color=INK, fontsize=8)
    ax.set_ylabel(ylab)


runs_s = sorted({r["run"] for r in dl if r["measure"] == "stretch_err_per_cm"})
runs_s = [r for r in runs_s if sum(1 for q in dl if q["run"] == r and q["measure"] == "stretch_err_per_cm" and
                                   float(q["own_cm_or_deg"]) >= 20) >= 5]
panel(axs[0], "stretch_err_per_cm", lambda r: float(r["value"]) if float(r["own_cm_or_deg"]) >= 20 else None, runs_s,
      "Position: blind stretches of 20-100 cm", "cm of error per cm driven, no camera fix")
runs_c = sorted({r["run"] for r in dl if r["measure"] == "curve_k"})
panel(axs[1], "curve_k", lambda r: float(r["value"]) / scale.get(r["run"], {}).get(r["robot"], 1.0), runs_c,
      "Curves: curvature made / asked", "ratio (its saved arc_k_scale taken out)")
axs[1].axhline(1.0, color=MUTED, lw=0.6, ls=":")
runs_t = sorted({r["run"] for r in dl if r["measure"] == "turn90_err_deg"})
panel(axs[2], "turn90_err_deg", lambda r: abs(float(r["value"])), runs_t,
      "Turns on the spot (DOE turn checks)", "deg of error per 90 deg turn", minn=4)
axs[0].legend(frameon=False, fontsize=7)
fig.suptitle("Each home robot's odometry error, run by run (dot = median, line = middle half)", x=0.01, ha="left",
             color=INK, fontsize=9)
foot(fig, "Measured: follow.csv against bridge/track.csv (left), arcfit.csv (middle), the DOE loops' turns.csv "
          "(right; 27 Oct - 2 Nov). drift.py. Runs with fewer than 5 values (4 for turns) are left out.")
fig.tight_layout(rect=(0, 0.06, 1, 0.94))
fig.savefig(os.path.join(OUT, "replay-all-feature-drift-record-per-robot-across-runs-chart.png"), dpi=150)
plt.close(fig)

# 4. the 5 Nov 14:55 scene: e17's wrong elimination
run = os.path.join(ROOT, "20261105-143553-freeroam")
T0, T1 = B - 491420.0, B - 491396.0
own, cam = {}, {}
for r in csv.DictReader(open(os.path.join(run, "follow.csv"))):
    t = float(r["t_recv"])
    if T0 <= t <= T1 and r["state"] != "nopose":
        own.setdefault(r["robot"], []).append((float(r["x_cm"]), float(r["y_cm"])))
for r in csv.DictReader(open(os.path.join(run, "bridge", "track.csv"))):
    t = float(r["t"])
    if T0 <= t <= T1 and r["status"] == "seen":
        cam.setdefault(r["robot"], []).append((float(r["x"]), float(r["y"])))
fig, ax = plt.subplots(figsize=(5.8, 5.6))
ax.add_patch(plt.Rectangle((0, 0), 122, 122, fill=False, ec=MUTED, lw=1))
for n, c in ROBOT.items():
    if own.get(n):
        xs, ys = zip(*own[n])
        ax.plot(xs, ys, color=c, lw=2, alpha=0.8, label="%s's own pose" % n)
        ax.scatter([xs[-1]], [ys[-1]], color=c, s=30, marker="o")
    if cam.get(n):
        xs, ys = zip(*cam[n])
        ax.scatter(xs, ys, color=c, s=7, alpha=0.4, marker="s", label="camera: the bridge says %s" % n)
el = json.load(open(os.path.join(RES, "elim_0514.json")))
ax.annotate("", xy=tuple(el["spot"]), xytext=tuple(el["right_own"]), arrowprops=dict(arrowstyle="->", color=ON, lw=1.5))
ax.scatter([el["spot"][0]], [el["spot"][1]], s=140, facecolors="none", edgecolors=ON, lw=1.5,
           label="e17 names this spot Right (%.0f cm away)" % el["d_cm"])
ax.set_xlim(-5, 127)
ax.set_ylim(-5, 127)
ax.set_aspect("equal")
ax.set_xlabel("x on the board (cm)")
ax.set_ylabel("y on the board (cm)")
ax.legend(frameon=False, fontsize=6, loc="lower left")
ax.set_title("Home 5 Nov 14:55: the 24 s before e17's wrong correction", loc="left", color=INK)
foot(fig, "Measured: follow.csv and bridge/track.csv of 20261105-143553-freeroam; the elimination from replay_one.py "
          "(e17 on, lag 0).")
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(os.path.join(OUT, "replay-all-problem-wrong-elimination-5nov-1455-chart.png"), dpi=150)
plt.close(fig)
print("charts written to", OUT)
