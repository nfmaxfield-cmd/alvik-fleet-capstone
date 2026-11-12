"""e19: the e17 comparison re-run (docs/results/e18-nov12/stress/run1.py, unchanged), e18 against e19, one dot a run.
    python3 chart_stress.py   -> docs/screenshots/2026-11-12/e19-feature-e17-comparison-rerun-chart.png and a table"""
import glob, json, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.dirname(HERE)
DOCS = os.path.dirname(os.path.dirname(RES))
SHOT = os.path.join(DOCS, "screenshots", "2026-11-12")
B, O, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e6e5e0"
runs = {}
for tag, d in (("e18", os.path.join(DOCS, "results", "e18-nov12", "stress")), ("e19", os.path.join(RES, "stress"))):
    for f in glob.glob(os.path.join(d, "%s-*.json" % tag)):
        _, seed, delay = os.path.basename(f)[:-5].split("-")
        runs[(tag, int(seed), delay)] = json.load(open(f))
rows = ["| 6 runs each | 0.45 s late: e18 | e19 | 1.0 s late: e18 | e19 |", "|---|---|---|---|---|"]
def col(tag, dl, fn):
    v = [fn(r) for (t, s, d), r in sorted(runs.items()) if t == tag and d == dl]
    return v
meas = [("Runs with contacts", lambda r: int(bool(r["contacts"])), sum),
        ("Runs with a robot off the board", lambda r: int(bool(r["off_board"]) or bool(r.get("falls"))), sum),
        ("Closest the bodies came, median (cm)", lambda r: r["min_gap_cm"], lambda v: sorted(v)[len(v) // 2] if v else None),
        ("Name swaps", lambda r: r["swaps"], sum),
        ("Longest stand, max (s)", lambda r: max(r["longest_still"].values()), lambda v: max(v) if v else None)]
for name, fn, agg in meas:
    cells = []
    for dl in ("0.45", "1.0"):
        for tag in ("e18", "e19"):
            v = col(tag, dl, fn)
            a = agg(v) if v else None
            cells.append("%s (%d runs)" % (round(a, 1) if isinstance(a, float) else a, len(v)) if v else "-")
    rows.append("| %s | %s |" % (name, " | ".join(cells)))
open(os.path.join(RES, "stress", "table.md"), "w").write("\n".join(rows) + "\n")
print("\n".join(rows))
fig, axes = plt.subplots(1, 2, figsize=(12, 4))
for ax, (title, fn) in zip(axes, (("Closest two bodies came (cm, edge to edge)", lambda r: r["min_gap_cm"]),
                                  ("Longest a robot stood still (s)", lambda r: max(r["longest_still"].values())))):
    for i, dl in enumerate(("0.45", "1.0")):
        for j, (tag, c) in enumerate((("e18", O), ("e19", B))):
            v = col(tag, dl, fn)
            xs = [i * 3 + j + 0.06 * (k - 2.5) for k in range(len(v))]
            ax.scatter(xs, v, s=36, color=c, edgecolor="#fcfcfb", linewidth=1.2, zorder=3,
                       label=tag if i == 0 else None)
    ax.set_xticks([0, 1, 3, 4]); ax.set_xticklabels(["e18\n0.45 s late", "e19\n0.45 s late", "e18\n1.0 s late", "e19\n1.0 s late"], fontsize=8)
    ax.set_title(title, fontsize=10, color=INK, loc="left")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.yaxis.grid(True, color=GRID); ax.set_axisbelow(True); ax.tick_params(colors=MUTED, labelsize=8)
axes[1].set_yscale("log")
from matplotlib.ticker import FixedLocator, ScalarFormatter
axes[1].yaxis.set_major_locator(FixedLocator([4, 6, 10, 15, 20, 30]))
axes[1].yaxis.set_major_formatter(ScalarFormatter())
axes[1].yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
axes[0].legend(frameon=False, fontsize=8)
fig.suptitle("The e17 comparison re-run: fast simulator, 6 seeds x 30 min a cell, camera fixes 0.45 / 1.0 s late "
             "(docs/results/e18-nov12/stress/run1.py, unchanged)", fontsize=9, color=MUTED, x=0.01, ha="left")
fig.tight_layout(rect=(0, 0, 1, 0.93))
fig.savefig(os.path.join(SHOT, "e19-feature-e17-comparison-rerun-chart.png"), dpi=120, facecolor="#fcfcfb")
