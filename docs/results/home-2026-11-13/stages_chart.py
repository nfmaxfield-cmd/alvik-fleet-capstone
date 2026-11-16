#!/usr/bin/env python3
# Home free roam 13 Nov (e21): each robot's stage over the run, and the planner's pass time. usage: stages_chart.py RUN_DIR OUT.png
import csv, sys, datetime
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
D, OUT = sys.argv[1], sys.argv[2]
t0 = 1794577504.99; tend = 1794591935.5
def h(t): return 8.75 + (float(t) - t0) / 3600.0
col = {"run": "#12805C", "veer": "#E3A008", "found": "#2952CC", "pause": "#B42318"}
rows = list(csv.DictReader(open(D + "/stages.csv")))
fig, (a, b) = plt.subplots(2, 1, figsize=(10, 4.6), sharex=True, gridspec_kw={"height_ratios": [2, 1.3], "hspace": 0.45})
for i, n in enumerate(["Left", "Center", "Right"]):
    l = sorted((float(r["t"]), r["stage"]) for r in rows if r["robot"] == n)
    for (s, st), (e, _) in zip(l, l[1:] + [(tend, None)]):
        a.barh(i, h(e) - h(s), left=h(s), color=col.get(st, "#999"), height=0.6)
a.set_yticks([0, 1, 2], ["Left", "Center", "Right"]); a.invert_yaxis()
for st, c in col.items(): a.barh(-5, 0, color=c, label={"pause": "pause (held, searched for)"}.get(st, st))
a.set_ylim(2.6, -0.6); a.set_xlim(8.7, 12.8); a.legend(ncol=4, fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.02), frameon=False)
a.set_title("Each robot's stage, home free roam 13 Nov 08:45-12:45 (e21 code)", fontsize=10)
pm = list(csv.DictReader(open(D + "/pi_monitor.csv")))
k = "loop_work_ms_p95"
tk = "t"
xs = [h(r[tk]) for r in pm if r[k] not in ("", None)]; ys = [float(r[k]) / 1000 for r in pm if r[k] not in ("", None)]
b.plot(xs, ys, color="#475467", lw=0.6); b.axhline(0.05, color="#B42318", ls=":", lw=1)
b.set_yscale("log"); b.set_xlim(8.7, 12.8); b.set_ylabel("planner pass (s)", fontsize=8); b.set_xlabel("time of day (h)", fontsize=8)
b.text(xs[0], 0.06, "aim: one pass every 0.05 s", fontsize=7, color="#B42318")
fig.text(0.01, 0.005, "Measured: stages.csv (the run's own stage changes) and pi_monitor.csv (planner loop, 95 % of passes, every 2 s), run 20261113-084020-freeroam.", fontsize=7, color="#5B6474")
fig.subplots_adjust(left=0.08, right=0.98, top=0.92, bottom=0.14); fig.savefig(OUT, dpi=110)
