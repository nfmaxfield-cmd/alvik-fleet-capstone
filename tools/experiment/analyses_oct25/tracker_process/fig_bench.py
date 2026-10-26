"""The before/after chart of the tracker process report: pictures handled a second against the runner's own work,
from tools/pi/tracker_proc_bench.py's CSVs (bench_load0/1/2.csv beside this file)."""
import csv, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
rows = {}
for L in (0, 1, 2):
    for r in csv.DictReader(open(os.path.join(HERE, "bench_load%d.csv" % L))):
        rows.setdefault(r["mode"], []).append((L, float(r["fps_median"]), float(r["fps_p10"])))
names = {"before": "Before (tracker thread in the runner, old matcher)", "thread": "Faster matcher, still a runner thread",
         "process": "Faster matcher, its own process (this build)"}
cols = {"before": "#b0b0b0", "thread": "#7fa7d4", "process": "#1f5aa6"}
fig, ax = plt.subplots(figsize=(7.2, 4.0), dpi=130)
for m in ("before", "thread", "process"):
    v = sorted(rows[m])
    ax.plot([q[0] for q in v], [q[1] for q in v], "o-", color=cols[m], lw=2, label=names[m])
    ax.fill_between([q[0] for q in v], [q[2] for q in v], [q[1] for q in v], color=cols[m], alpha=0.15, lw=0)
ax.axhline(25, color="#444", lw=0.8, ls=":")
ax.text(2.02, 25.3, "camera: 25 a second", fontsize=8, color="#444", ha="right", va="bottom")
ax.set_xticks([0, 1, 2]); ax.set_xticklabels(["none", "1 busy thread", "2 busy threads"])
ax.set_xlabel("Other work in the runner's process (stand-in: Python threads busy half the time)")
ax.set_ylabel("Pictures handled a second\n(median; shaded down to the slowest 10 %)")
ax.set_ylim(0, 27); ax.grid(axis="y", alpha=0.3); ax.legend(fontsize=8, loc="lower left", frameon=False)
ax.set_title("Tracker keeping up with the camera, 15:45 footage replayed at 25 a second", fontsize=10)
fig.text(0.01, 0.005, "Measured: 40 s per point of the 15:45 overhead video (16:31:34), Center followed as the runner did; "
         "this 2-core build machine, not the Pi.", fontsize=6.5, color="#555")
fig.tight_layout(rect=(0, 0.03, 1, 1))
fig.savefig(os.path.join(HERE, "../../../../docs/build-reports/e02-oct25-tracker-process.png"))
