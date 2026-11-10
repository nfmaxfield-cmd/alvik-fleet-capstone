import csv, sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
r = list(csv.DictReader(open("out/stops_all.csv"))); O = sys.argv[1]
plt.rcParams.update({"font.size": 10})
# 1 stop offsets
fig, ax = plt.subplots(figsize=(6, 5.6))
for z, c, lab in ((1, "#1b6e7a", "zone 1 (P01-P10)"), (2, "#d9822b", "zone 2 (P11-P20)")):
    xs = [float(x["ex"]) for x in r if int(x["zone"]) == z]; ys = [float(x["ey"]) for x in r if int(x["zone"]) == z]
    ax.scatter(xs, ys, s=16, alpha=.7, c=c, label=lab)
for rad in (1, 2, 5):
    ax.add_patch(plt.Circle((0, 0), rad, fill=False, ls=":", color="#888")); ax.text(rad*.71, rad*.71, "%d cm" % rad, color="#666", fontsize=8)
ax.set_aspect("equal"); ax.set_xlim(-8, 8); ax.set_ylim(-8, 8); ax.axhline(0, c="#ccc", lw=.6); ax.axvline(0, c="#ccc", lw=.6)
ax.set_xlabel("x offset from the point (cm)"); ax.set_ylabel("y offset from the point (cm)")
ax.set_title("Where the robot stopped, relative to each point\n140 stops, 7 tours, AL-1DC0, school table, 9 Nov", fontsize=10)
ax.legend(loc="lower right", fontsize=8)
fig.text(.01, .01, "Source: stops.csv (camera median during each 8 s stop) vs the run's point coordinates (mission.json)", fontsize=7, color="#555")
fig.tight_layout(rect=(0, .03, 1, 1)); fig.savefig(O + "/01-stop-offsets.png", dpi=150)
# 2 odometry drift
fig, ax = plt.subplots(figsize=(7, 4.2))
for run in sorted(set(x["run"] for x in r)):
    rr = [x for x in r if x["run"] == run]
    ax.plot([float(x["odo_dist"]) / 100 for x in rr], [float(x["odo_err"]) for x in rr], marker="o", ms=3, lw=1.2, label=run[9:11] + ":" + run[11:13])
ax.set_xlabel("distance driven since the tour started (m)"); ax.set_ylabel("wheels+gyro position vs camera (cm)")
ax.set_title("How far the robot's own dead reckoning drifts with no camera help", fontsize=10)
ax.legend(title="tour start", fontsize=8, ncol=2)
fig.text(.01, .01, "Source: odometry_raw.csv (never camera-corrected) vs the camera at each stop; gyro_scale 1.075, wheel_scale 0.979 on the robot", fontsize=7, color="#555")
fig.tight_layout(rect=(0, .03, 1, 1)); fig.savefig(O + "/02-odometry-drift.png", dpi=150)
# 3 settle
st = np.sort([float(x["settle"]) for x in r if x["settle"] != "nan"])
fig, ax = plt.subplots(figsize=(6.5, 3.8))
ax.step(st, np.arange(1, len(st) + 1) / len(st) * 100, where="post", c="#1b6e7a")
for v in (0.5, 1.0): ax.axvline(v, ls=":", c="#888"); ax.text(v + .03, 8, "%.1f s: %.0f %%" % (v, (st <= v).mean() * 100), fontsize=8)
ax.set_xlabel("seconds after the stop starts until the camera position holds within 0.5 cm"); ax.set_ylabel("% of stops settled")
ax.set_title("How long the robot takes to settle at a point (%d stops)" % len(st), fontsize=10)
fig.text(.01, .01, "Source: camera_track.csv within each stop window from stops.csv", fontsize=7, color="#555")
fig.tight_layout(rect=(0, .03, 1, 1)); fig.savefig(O + "/03-settle-time.png", dpi=150)
