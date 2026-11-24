#!/usr/bin/env python3
"""(e24) The report's two charts, from acceptance/runs.jsonl and acceptance/doe_runs.jsonl.
    charts.py RESULTS_DIR SCREENSHOT_DIR
Changelog: 2026-11-24 first version.
"""
import json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
R, S = sys.argv[1], sys.argv[2]
rows = [json.loads(l) for l in open(os.path.join(R, "acceptance", "runs.jsonl"))]
ORDER = ["four", "twenty", "crossed", "restart", "yaw", "batt", "miss"]
NAME = {"four": "4 + 4", "twenty": "20 + 20", "crossed": "20 + 20\ncrossed", "restart": "8 + 8\nrestart", "yaw": "4 + 4\ngyro jump",
        "batt": "6 + 6\nlow battery", "miss": "4 + 4\none robot\nmissing"}
fig, ax = plt.subplots(2, 1, figsize=(11, 7.2), sharex=True)
x, ticks, lab = 0, [], []
for c in ORDER:
    g = [r for r in rows if r["case"] == c]
    if not g:
        continue
    x0 = x
    for code, col in (("before", "#9aa3ad"), ("after", "#2f5fd0")):
        for r in sorted([r for r in g if r["code"] == code], key=lambda r: r["seed"]):
            share = 100.0 * (r["reached"] or 0) / max(1, r["asked"] or 1)
            ax[0].bar(x, share, color=col, width=0.8)
            ax[0].text(x, share + 1.5, "%d/%d" % (r["reached"] or 0, r["asked"] or 0), ha="center", fontsize=7)
            ax[1].bar(x, r["duration_s"] or 0, color=col, width=0.8)
            if r.get("capped") or r["result"] != "done":
                ax[1].text(x, (r["duration_s"] or 0) + 10, "x", ha="center", fontsize=9, color="#b3261e")
            x += 1
        x += 0.4
    ticks.append((x0 + x - 1.4) / 2); lab.append(NAME[c]); x += 1.2
ax[0].set_ylabel("points reached, % of asked"); ax[0].set_ylim(0, 115)
ax[1].set_ylabel("run time, s"); ax[1].set_xticks(ticks); ax[1].set_xticklabels(lab, fontsize=9)
ax[0].bar(0, 0, color="#9aa3ad", label="before (39df4cf)"); ax[0].bar(0, 0, color="#2f5fd0", label="after (e24)")
ax[0].legend(loc="lower right", fontsize=8)
ax[0].set_title("Simulator acceptance runs on the 23 Nov school stand-in: one bar per run (seed), before and after e24", fontsize=10)
fig.text(0.01, 0.01, "Measured: points a robot truly stopped at (simulator's true positions) and the run's own time; a red x marks a run that did not end 'done'. "
         "Source: docs/results/e24-school-usability/acceptance/runs.jsonl", fontsize=7)
fig.tight_layout(rect=(0, 0.03, 1, 1))
fig.savefig(os.path.join(R, "acceptance_runs.png"), dpi=110)
fig.savefig(os.path.join(S, "e24-feature-acceptance-before-after-by-run.png"), dpi=110)
dp = os.path.join(R, "acceptance", "doe_runs.jsonl")
if os.path.exists(dp):
    d = [json.loads(l) for l in open(dp)]
    lv = ["1", "2", "3", "all"]
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.8))
    for i, (k, yl) in enumerate((("duration_s", "run time, s"), ("replan_cpu_s", "time spent planning, s"), ("replans", "re-plans"))):
        for j, a in enumerate(lv):
            g = sorted([r for r in d if r["ahead"] == a], key=lambda r: r["seed"])
            ys = [r.get(k) or 0 for r in g]
            ax[i].scatter([j + (n - len(ys) / 2) * 0.05 for n in range(len(ys))], ys, s=22,
                          c=["#2f5fd0" if r["reached"] == r["asked"] and r["result"] == "done" else "#b3261e" for r in g])
            if ys:
                ys2 = sorted(ys); ax[i].plot([j - 0.3, j + 0.3], [ys2[len(ys2) // 2]] * 2, color="#333", lw=1.2)
        ax[i].set_xticks(range(len(lv))); ax[i].set_xticklabels(["1", "2", "3", "whole list"]); ax[i].set_ylabel(yl)
        ax[i].set_xlabel("points planned ahead per robot")
    fig.suptitle("Plan-ahead experiment: 10 simulator runs per setting (10 + 10 crossed lists, school stand-in); line = middle run; red = not every point reached", fontsize=9)
    fig.text(0.01, 0.01, "Measured: each run's own time, planning time and re-plan count. Source: docs/results/e24-school-usability/acceptance/doe_runs.jsonl", fontsize=7)
    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    fig.savefig(os.path.join(R, "plan_ahead_doe.png"), dpi=110)
    fig.savefig(os.path.join(S, "e24-feature-plan-ahead-experiment.png"), dpi=110)
