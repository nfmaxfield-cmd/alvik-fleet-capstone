#!/usr/bin/env python3
"""(e23c, the audit's fixes) Charts and terminal pictures for the audit section of the e23 report.
usage: charts_e23c.py REPO OUT_DIR [OLD_REPO] [ACCEPTANCE_JSON]
  1. the stop tolerance through the real 16:11 and 18:06 runs, e23 as built against now (stops_replayed.json)
  2. a burst of four old pictures: own pose minus truth after each, e23 as built against now (the test's own replica)
  3. (OLD_REPO given) the Stop that no robot answered, as text: e23 as built and now (stop_noanswer.py)
  4. (ACCEPTANCE_JSON given) points truly reached in every acceptance run, by seed (acceptance/seeds_e23c.json)
Version 1 (2026-11-19)."""
import json, os, subprocess, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
REPO, OUT = os.path.abspath(sys.argv[1]), sys.argv[2]
OLD = sys.argv[3] if len(sys.argv) > 3 and sys.argv[3] != "-" else None
ACC = sys.argv[4] if len(sys.argv) > 4 else None
HERE = os.path.dirname(os.path.abspath(__file__))
os.makedirs(OUT, exist_ok=True)
BLUE, RED, GREY, GREEN = "#2952CC", "#C2410C", "#8A8F98", "#2E7D32"

# ---- 1. the tolerance through two real runs --------------------------------------------------------------------------
rows = json.load(open(os.path.join(HERE, "stops_replayed.json")))["stops"]
fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), sharey=True)
for ax, run, title in zip(axes, ("20261118-161142-visit-robots", "20261118-180601-visit-robots"),
                          ("18 Nov 16:11 (four stops at P29, 24 to 29 cm off)", "18 Nov 18:06 (20 + 20 points)")):
    r = [q for q in rows if q["run"] == run]
    x = list(range(1, len(r) + 1))
    ax.step(x, [q["e23_tol_cm"] for q in r], where="mid", color=RED, lw=1.8, label="tolerance, e23 as built")
    ax.step(x, [q["tol_cm"] for q in r], where="mid", color=BLUE, lw=1.8, label="tolerance now")
    for k, q in zip(x, r):
        ax.scatter(k, q["miss_cm"], s=55, marker="o" if q["credited"] else "X", zorder=3,
                   facecolor=GREEN if q["credited"] else "black", edgecolor="black", linewidth=0.6)
    ax.set_title(title, fontsize=9)
    ax.set_xlabel("Stop number in the run")
    ax.set_xticks(x)
    ax.grid(axis="y", color="#E5E7EB", lw=0.6)
axes[0].set_ylabel("Distance from the point (cm)")
import matplotlib.lines as ml
h, _ = axes[0].get_legend_handles_labels()
axes[1].legend(handles=h + [ml.Line2D([], [], marker="o", ls="", mfc=GREEN, mec="black", label="a stop, counted now"),
                            ml.Line2D([], [], marker="X", ls="", mfc="black", mec="black", label="a stop, refused now")],
               fontsize=8, frameon=False, loc="upper left")
fig.suptitle("How far a stop may be from its point and still count: the tolerance through two real school runs", fontsize=10)
fig.text(0.01, 0.01, "Measured: each recorded stop's position against its point's typed position (run/points.csv of Jay-Sun's "
         "18 Nov runs), judged by run_plan.record_point with MIRLT_REACH_TOL=e23 and with the default\n"
         "(docs/results/e23-school-fixes/replay_stops.py, stops_replayed.json).", fontsize=7, color=GREY)
fig.tight_layout(rect=(0, 0.07, 1, 0.95))
fig.savefig(os.path.join(OUT, "e23c-problem-stop-tolerance-climbed-with-misses.png"), dpi=130)
plt.close(fig)

# ---- 2. the burst of old pictures ------------------------------------------------------------------------------------
os.environ.setdefault("HOME", "/nonexistent")
sys.path.insert(0, os.path.join(REPO, "tools", "fleet", "tests"))
import test_e23c_audit_fixes as T
fig, ax = plt.subplots(figsize=(8.6, 4.6))
for e, ls in ((3.0, "-"), (8.0, "--")):
    for flag, col, name in ((False, RED, "e23 as built"), (True, BLUE, "now")):
        T.RP.OLD_FIX_MOTION_ONLY = flag
        y = [-e] + T._burst(e)
        ax.plot(range(5), y, ls, color=col, marker="o", lw=1.8, label="%s, starting %.0f cm behind" % (name, e))
T.RP.OLD_FIX_MOTION_ONLY = True
ax.axhline(0.0, color=GREY, lw=1)
ax.text(4.05, 0.15, "the truth", fontsize=8, color=GREY)
ax.set_xticks(range(5))
ax.set_xticklabels(["before", "after picture 1\n(1.6 s old)", "after picture 2\n(1.4 s old)", "after picture 3\n(1.2 s old)",
                    "after picture 4\n(1.05 s old)"], fontsize=8)
ax.set_ylabel("The robot's own position minus its true position (cm)")
ax.set_title("Four old camera pictures in a row: where the robot's own position ends up", fontsize=10)
ax.legend(fontsize=8, frameon=False, loc="upper left")
ax.grid(axis="y", color="#E5E7EB", lw=0.6)
fig.text(0.01, 0.01, "Measured: a 1-D copy of the firmware's fix step (pose.h) fed by the real run_plan.project_old_fix; the robot drives "
         "at 10 cm/s, pictures arrive 0.2 s apart\n(tools/fleet/tests/test_e23c_audit_fixes.py, _burst; from the audit's "
         "burst_test.py). Computed, not a robot run.", fontsize=7, color=GREY)
fig.tight_layout(rect=(0, 0.08, 1, 1))
fig.savefig(os.path.join(OUT, "e23c-problem-burst-of-old-pictures-over-corrected.png"), dpi=130)
plt.close(fig)


def term(lines, out, title):
    import textwrap
    lines = [w for ln in lines for w in (textwrap.wrap(ln, 150, subsequent_indent=" " * 44) or [""])]
    hh = 0.19 * len(lines) + 0.7
    fig = plt.figure(figsize=(14.5, hh))
    fig.patch.set_facecolor("#1E1E1E")
    fig.text(0.01, 1 - 0.25 / hh, title, color="#9CDCFE", family="monospace", fontsize=9, va="top")
    for i, ln in enumerate(lines):
        col = "#F48771" if ("every robot confirmed" in ln or ln.startswith("==== e23 as built")) else \
            "#B5CEA8" if ("no robot answered" in ln or ln.startswith("==== now")) else "#D4D4D4"
        fig.text(0.01, 1 - (0.5 + 0.19 * i) / hh, ln, color=col, family="monospace", fontsize=8, va="top")
    fig.savefig(out, dpi=110, facecolor=fig.get_facecolor())
    plt.close(fig)


# ---- 3. the Stop that no robot answered ------------------------------------------------------------------------------
if OLD:
    lines = []
    for name, repo in (("e23 as built (b202ea5)", OLD), ("now", REPO)):
        r = subprocess.run([sys.executable, os.path.join(HERE, "stop_noanswer.py"), repo], capture_output=True, text=True)
        lines += ["==== %s" % name] + r.stdout.rstrip().splitlines() + [""]
    term(lines, os.path.join(OUT, "e23c-problem-stop-nobody-answered-said-confirmed.png"),
         "docs/results/e23-school-fixes/stop_noanswer.py: Stop before the robots are named, and no robot answers "
         "(stubs; nothing sent to a robot)")

# ---- 4. acceptance by seed ---------------------------------------------------------------------------------------------
if ACC:
    acc = json.load(open(ACC))
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.4))
    for ax, case, asked in zip(axes, ("school20", "school4"), (40, 8)):
        rs = [q for q in acc["rows"] if q["case"] == case]
        seeds = sorted({q["seed"] for q in rs})
        for q in rs:
            ax.scatter(seeds.index(q["seed"]), q["truly"], s=80, color=BLUE if q["ended"] != "capped" else RED,
                       edgecolor="black", linewidth=0.6, zorder=3, marker="o" if q["ended"] != "capped" else "s")
        # e23 as built (b202ea5) on the same seeds, where it was run: hollow diamonds beside the dots
        for q in [q for q in acc["rows"] if q["case"] == "b202" + case and q["seed"] in seeds]:
            ax.scatter(seeds.index(q["seed"]) + 0.22, q["truly"], s=70, facecolor="white", edgecolor="black",
                       linewidth=1.0, marker="D", zorder=3)
        ax.axhline(asked, color=GREY, ls="--", lw=1)
        ax.text(len(seeds) - 0.55, asked, " asked: %d" % asked, fontsize=8, color=GREY, va="bottom", ha="right")
        ax.set_xticks(range(len(seeds)))
        ax.set_xticklabels(["seed %d" % s for s in seeds])
        ax.set_xlim(-0.5, len(seeds) - 0.5)
        ax.set_ylim(0, asked * 1.12)
        ax.set_ylabel("Points the robots truly stood at")
        ax.set_title("%d + %d points" % (asked // 2, asked // 2), fontsize=9)
        ax.grid(axis="y", color="#E5E7EB", lw=0.6)
    axes[1].legend(handles=[ml.Line2D([], [], marker="o", ls="", mfc=BLUE, mec="black", label="final code: run ended by itself"),
                            ml.Line2D([], [], marker="D", ls="", mfc="white", mec="black",
                                      label="e23 as built, same seed (4 + 4 only)"),
                            ml.Line2D([], [], marker="s", ls="", mfc=RED, mec="black",
                                      label="final code: not finished at 19 min")],
                   fontsize=8, frameon=False, loc="lower left")
    fig.suptitle("School Pi stand-in in the simulator, final code: points truly reached in every run, by seed", fontsize=10)
    fig.text(0.01, 0.01, "Measured: the simulator's true robot positions during each recorded stop against the point's place "
             "(acc_table.py), one mark per run; late pictures 1.5 s every 20 s, planning 6 x slower\n"
             "(docs/results/e23-school-fixes/acceptance/seeds_e23c.json). The same seed does not give the same run: "
             "outcomes depend on wall-clock timing.", fontsize=7, color=GREY)
    fig.tight_layout(rect=(0, 0.08, 1, 0.94))
    fig.savefig(os.path.join(OUT, "e23c-problem-acceptance-by-seed.png"), dpi=130)
    plt.close(fig)
print("charts written to", OUT)
