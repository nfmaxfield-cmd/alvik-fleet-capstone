#!/usr/bin/env python3
"""(e23) The report's charts, from the numbers beside this file. usage: charts.py REPO OUT_DIR [ACCEPTANCE_DIR]"""
import glob, json, math, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
REPO, OUT = sys.argv[1], sys.argv[2]
ACC = sys.argv[3] if len(sys.argv) > 3 else None
HERE = os.path.dirname(os.path.abspath(__file__))
os.makedirs(OUT, exist_ok=True)
BLUE, RED, GREY, GREEN = "#2952CC", "#C2410C", "#8A8F98", "#2E7D32"

# ---- 1. the move check on the real verdicts of 18 Nov --------------------------------------------------------------
sys.path.insert(0, os.path.join(REPO, "tools", "fleet", "tests"))
import test_e23_school_fixes as T
rows = []
for c in T.CASES:
    rows.append((T._short_cm(c), c["took_s"], c["verdict"], T._judge(c)[0]))
fig, ax = plt.subplots(figsize=(9.5, 4.6))
for short, took, old, new_fail in rows:
    ax.scatter(max(short, -6), took, s=70, marker="o" if old == "not_moved" else "s",
               facecolor=RED if new_fail else GREEN, edgecolor="black", linewidth=0.6, zorder=3)
ax.axvline(3.0, color=GREY, ls="--", lw=1)
ax.text(3.2, ax.get_ylim()[1] * 0.93, "3 cm short or more:\nreally not done", fontsize=8, color=GREY, va="top")
ax.set_xlabel("How far short of the step's target the robot's own pose ended, along the step's line (cm; below 0: already past it)")
ax.set_ylabel("Time the step took (s)")
ax.set_title("Every 'did NOT move' (circle) and 'not physically possible' (square) verdict of 18 Nov, judged again by e23", fontsize=10)
import matplotlib.lines as ml
ax.legend(handles=[ml.Line2D([], [], marker="o", ls="", mfc=GREEN, mec="black", label="e23: moved (25)"),
                   ml.Line2D([], [], marker="o", ls="", mfc=RED, mec="black", label="e23: still not done (4)")],
          loc="upper right", bbox_to_anchor=(0.86, 0.98), fontsize=8, frameon=False)
fig.text(0.01, 0.01, "Measured: each step's planned line, the robot's own pose when it started and ended the step, and the time "
         "between (29 verdicts in run/coordinator.jsonl of Jay-Sun's 18 Nov school runs;\ntools/fleet/tests/data/e23_move_checks.json).",
         fontsize=7, color=GREY)
fig.tight_layout(rect=(0, 0.07, 1, 1))
fig.savefig(os.path.join(OUT, "e23-feature-move-check-on-the-real-verdicts.png"), dpi=130)
plt.close(fig)

# ---- 2. the Pi through 18 Nov: pictures a second per camera, and where the cores were ------------------------------
age = json.load(open(os.path.join(HERE, "picture_age.json")))
cpu = json.load(open(os.path.join(HERE, "cpu_share.json")))
runs = [r for r in sorted(age) if r.startswith("20261118")]
lab = [r[9:11] + ":" + r[11:13] for r in runs]
fig, (a1, a2) = plt.subplots(2, 1, figsize=(9.5, 6.4), sharex=True)
x = range(len(runs))
a1.plot(x, [age[r].get("overhead", {}).get("per_s") for r in runs], "-o", color=BLUE, label="camera 1 (overhead)")
a1.plot(x, [age[r].get("side", {}).get("per_s") if "side" in age[r] else float("nan") for r in runs], "-s", color=RED,
        label="camera 2 (side)")
a1.set_ylabel("Pictures read per second\nby the tag bridge")
a1.legend(fontsize=8, frameon=False)
a1.set_ylim(0, 10)
a1.set_title("The school Pi through 18 Nov: pictures read per camera, and how busy each processor core was", fontsize=10)
for k, c in zip(("core0", "core1", "core2", "core3"), (RED, BLUE, GREEN, GREY)):
    a2.plot(x, [cpu.get(r, {}).get(k, {}).get("median") for r in runs], "-o", color=c, ms=4,
            label={"core0": "core 0 (the recorder, both cameras)", "core1": "core 1 (the run program and its planner)",
                   "core2": "core 2 (tag bridge)", "core3": "core 3 (tag bridge)"}[k])
a2.set_ylabel("Core busy, median of the run (%)")
a2.set_ylim(0, 105)
a2.legend(fontsize=7, frameon=False, ncol=2, loc="lower right")
a2.set_xticks(list(x))
a2.set_xticklabels(lab, rotation=45, fontsize=8)
a2.set_xlabel("Run start on 18 Nov (camera 2 joined the runs from 15:03; the Pi was throttled in most health rows from 16:30)")
fig.text(0.01, 0.005, "Measured: bridge/detect_times.csv (one row per picture read) and cpu.csv (each core's load every 2 s) of each run folder; "
         "docs/results/e23-school-fixes/picture_age.json, cpu_share.json.", fontsize=7, color=GREY)
fig.tight_layout(rect=(0, 0.03, 1, 1))
fig.savefig(os.path.join(OUT, "e23-problem-pi-load-two-cameras.png"), dpi=130)
plt.close(fig)

# ---- 3. planning time per re-plan: the school Pi's run, the code before e23 and e23 in the simulator -----------------
if ACC:
    series = []
    pi = json.load(open(os.path.join(HERE, "pi_replans_run24.json")))
    series.append(("School Pi, 18 Nov 18:06\n(20 + 20 points, before e23)", pi["cpu_s"], GREY))
    for f in sorted(glob.glob(os.path.join(ACC, "*", "run", "run_plan_summary.json"))):
        name = os.path.basename(os.path.dirname(os.path.dirname(f)))
        try:
            rs = json.load(open(f))
        except ValueError:
            continue
        v = [r["cpu_s"] for r in rs.get("replans") or [] if r.get("cpu_s") is not None]
        if v:
            series.append((name.replace("-", "\n", 1), v, RED if name.startswith("old") else BLUE))
    fig, ax = plt.subplots(figsize=(max(9.5, 1.1 * len(series)), 4.8))
    import random
    rng = random.Random(1)
    for k, (name, v, col) in enumerate(series):
        ax.scatter([k + rng.uniform(-0.18, 0.18) for _ in v], v, s=18, color=col, alpha=0.7, zorder=3)
        sv = sorted(v)
        ax.plot([k - 0.28, k + 0.28], [sv[len(sv) // 2]] * 2, color="black", lw=1.6, zorder=4)
    ax.axhline(2.0, color=GREY, ls="--", lw=1)
    ax.text(len(series) - 0.5, 2.1, "the re-plan's budget, 2 s", fontsize=8, color=GREY, ha="right")
    ax.set_xticks(range(len(series)))
    ax.set_xticklabels([s[0] for s in series], fontsize=7)
    ax.set_ylabel("Time to work out one re-plan (s)")
    ax.set_yscale("log")
    ax.set_title("Planning time of every re-plan (a dot each; the bar is the median): the Pi's real run, then the simulator runs", fontsize=10)
    fig.text(0.01, 0.005, "Measured: cpu_s of every re-plan in each run's run_plan_summary.json. Simulator runs named 'slow' plan 6 times slower "
             "than this machine, like the Pi (docs/results/e23-school-fixes/acceptance).", fontsize=7, color=GREY)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(OUT, "e23-feature-planning-time-per-replan.png"), dpi=130)
    plt.close(fig)
print("charts written to", OUT)
