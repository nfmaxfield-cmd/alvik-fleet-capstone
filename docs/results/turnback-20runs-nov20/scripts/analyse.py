#!/usr/bin/env python3
"""Summary, tests and chart from results.csv."""
import csv, json, os, statistics as st
import numpy as np
from scipy import stats
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
T = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
rows = list(csv.DictReader(open(os.path.join(T, "results.csv"))))
NAMES = {"a": "a: e23 as built (b202ea5)", "b": "b: final code (ccb37ca)", "c": "c: final + OLD_FIX_JUMPS=count"}
F = ["finished", "truly", "false_credits", "replans", "turnbacks", "tb_first_moving_stall", "run_s", "closest_two_bodies_cm",
     "closest_body_to_open_edge_cm", "crash", "hung_over_edge", "first_moving_stall_t_run"]
def col(c, k): return [float(r[k]) for r in rows if r["config"] == c and r[k] not in ("", "None")]
out = {}
for c in "abc":
    n = sum(r["config"] == c for r in rows)
    out[c] = {"n": n}
    for k in F:
        v = col(c, k)
        if v: out[c][k] = {"median": st.median(v), "min": min(v), "max": max(v), "sum": sum(v)}
    out[c]["contact"] = sum(r["contact"] not in ("False", "", "None") for r in rows if r["config"] == c)
tests = {}
for x, y in (("a", "b"), ("b", "c"), ("a", "c")):
    t = {}
    for k in ("turnbacks", "replans", "run_s", "truly"):
        u = stats.mannwhitneyu(col(x, k), col(y, k), alternative="two-sided")
        t[k] = {"U": float(u.statistic), "p": float(u.pvalue)}
    fx, fy = col(x, "tb_first_moving_stall"), col(y, "tb_first_moving_stall")
    tab = [[int(sum(fx)), len(fx) - int(sum(fx))], [int(sum(fy)), len(fy) - int(sum(fy))]]
    fr = stats.fisher_exact(tab)
    t["tb_first_moving_stall"] = {"table": tab, "odds_ratio": float(fr.statistic), "p": float(fr.pvalue)}
    tests[x + "_vs_" + y] = t
# does a turn-back at the first moving stall go with more turn-backs over the run? (all configs pooled)
yes = [float(r["turnbacks"]) for r in rows if r["tb_first_moving_stall"] == "1"]
no = [float(r["turnbacks"]) for r in rows if r["tb_first_moving_stall"] == "0"]
if yes and no:
    u = stats.mannwhitneyu(yes, no, alternative="two-sided")
    tests["turnbacks_by_first_stall"] = {"yes_median": st.median(yes), "no_median": st.median(no), "n_yes": len(yes), "n_no": len(no), "p": float(u.pvalue)}
json.dump({"summary": out, "tests": tests}, open(os.path.join(T, "analysis.json"), "w"), indent=1)
print(json.dumps({"summary": out, "tests": tests}, indent=1))
# chart
fig, ax = plt.subplots(figsize=(7.5, 4.6), dpi=130)
rng = np.random.default_rng(0)
cols = {"a": "#4C78A8", "b": "#E45756", "c": "#54A24B"}
for i, c in enumerate("abc"):
    v = col(c, "turnbacks"); fs = [r["tb_first_moving_stall"] == "1" for r in rows if r["config"] == c]
    ax.boxplot([v], positions=[i], widths=0.5, showfliers=False, medianprops={"color": "black"},
               boxprops={"color": "#888"}, whiskerprops={"color": "#888"}, capprops={"color": "#888"})
    jx = i + rng.uniform(-0.17, 0.17, len(v))
    for xx, yy, f in zip(jx, v, fs):
        ax.scatter(xx, yy, s=26, color=cols[c], edgecolor="black" if f else cols[c], linewidth=1.0,
                   marker="o" if f else "^", alpha=0.85, zorder=3)
    labs = labs if 'labs' in dir() else []
    labs.append("%s\nturn-back at first moving stall: %d of %d" % (NAMES[c], int(sum(fs)), len(fs)))
ax.set_xticks(range(3)); ax.set_xticklabels(labs, fontsize=7.5)
ax.set_ylabel("turn-backs (veer events) per run")
ax.set_ylim(bottom=-0.5)
ax.set_title("Turn-backs per 4 + 4 run, by code version\n(simulator stand-in for the school Pi, seeds 1-20 each)", fontsize=10)
ax.scatter([], [], marker="o", color="#999", edgecolor="black", label="run with a turn-back at the first moving stall")
ax.scatter([], [], marker="^", color="#999", label="run without one")
ax.legend(fontsize=7.5, loc="upper left", frameon=False)
fig.text(0.5, 0.075, "All three code versions turn back about as often (median 3-4 per run); the runs with many turn-backs are the ones that turned back at the first stall, in every version.",
         fontsize=7.8, ha="center", wrap=True)
fig.text(0.01, 0.01, "Data: coordinator.jsonl 'veer' events, 60 dry runs of seed_runs.py (both cameras freeze 1.5 s every 20 s, planning 6x slower, camera 2 on).",
         fontsize=6.5, color="#555")
fig.subplots_adjust(bottom=0.25)
fig.savefig(os.path.join(T, "turnbacks.png"))
