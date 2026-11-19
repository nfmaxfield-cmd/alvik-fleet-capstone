#!/usr/bin/env python3
"""Chart: points reached against points asked, per run, in time order (from digest.json made by digest.py).
Usage: python3 -I chart.py digest.json runs_progress.png"""
import json, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
R = json.load(open(sys.argv[1]))
END = {
 "20261116-173641": "stopped by hand at 55 s (old code, one robot, no list)",
 "20261116-174306": "stopped by hand while the cameras were being opened",
 "20261116-174647": "not started: leftover Stop kept stopping the robot",
 "20261116-174754": "not started: robots' port held (leftover Stop)",
 "20261116-174837": "start-up failed: leftover Stop kept stopping the robot",
 "20261118-123004": "not started: Pi card full (0.35 GB free)",
 "20261118-125433": "not started: robots asleep, none answered in 3 min",
 "20261118-130132": "done (one robot)",
 "20261118-132919": "done, both lists",
 "20261118-134340": "done, both lists",
 "20261118-142714": "not started: AL-E7DC's tag outside camera 1's view",
 "20261118-143047": "stopped by hand; Bravo sent blind to the far half, held",
 "20261118-150347": "done, both lists",
 "20261118-161142": "done, both lists (Bravo drove onto an outlet plate)",
 "20261118-163101": "start hung 4 min opening cameras, killed",
 "20261118-164246": "done, both lists",
 "20261118-165350": "start hung 5 min opening cameras, killed",
 "20261118-170021": "not started: leftover Stop cut the compass check",
 "20261118-170126": "not started: leftover Stop cut the compass check",
 "20261118-170445": "not started: robots' port held (leftover Stop)",
 "20261118-170939": "program crashed at 292 s (KeyError), shown as 'done'",
 "20261118-173209": "one robot, 40 points: held again and again, out at 1007 s",
 "20261118-175439": "one robot: overshot P33 to the table edge, held, out",
 "20261118-180601": "stopped by hand at 460 s: planner busy 87 % of the run",
 "20261118-183130": "program crashed at 36 s (KeyError), shown as 'done'",
}
INK, MUTED, REACHED, ASKED = "#1f2328", "#6b7280", "#2f6fb0", "#d9dde3"
fig, ax = plt.subplots(figsize=(13, 9.2), dpi=130)
y = 0; ticks = []; labs = []
for r in R:
    key = r["run"][:15]
    rows = [(rb, v[0], v[1]) for rb, v in (r.get("progress") or {}).items()]
    if not rows:
        got = sum(len(v) for v in (r.get("got") or {}).values())
        rows = [("", got, 17 if got else 0)]
    rows.sort()
    for k, (rb, n, tot) in enumerate(rows):
        ax.barh(y, tot, height=0.62, color=ASKED, zorder=1)
        ax.barh(y, n, height=0.62, color=REACHED, zorder=2)
        ax.text(-0.6, y, rb, ha="right", va="center", fontsize=7.5, color=MUTED)
        ax.text(max(tot, 0) + 0.6, y, ("%d of %d" % (n, tot)) if tot else "0", va="center", fontsize=8, color=INK)
        if k == 0:
            ax.text(46, y + (len(rows) - 1) / 2.0, END.get(key, ""), va="center", fontsize=8.2, color=INK)
            ax.text(-5.5, y + (len(rows) - 1) / 2.0, "%s Nov %s:%s" % (key[6:8], key[9:11], key[11:13]), ha="right", va="center", fontsize=8.2, color=INK)
        y += 1
    y += 0.55
ax.set_ylim(y, -1); ax.set_xlim(-14, 100); ax.set_yticks([])
ax.set_xticks([0, 10, 20, 30, 40]); ax.tick_params(axis="x", labelsize=8, colors=MUTED)
for s in ("top", "right", "left"): ax.spines[s].set_visible(False)
ax.spines["bottom"].set_bounds(0, 40); ax.spines["bottom"].set_color(MUTED)
ax.set_xlabel("points on the robot's list (grey) and points it stopped at (blue)", fontsize=9, color=INK, loc="left")
ax.set_title("Visit points runs at the school table, 16 and 18 Nov 2026: points reached against points asked, per robot",
             fontsize=11, color=INK, loc="left")
fig.text(0.01, 0.012, "Measured: stops recorded in each run's run/stops.csv (or the run's POINT lines) against its visit_order.json; how each run ended from run_plan_stdout.txt. "
         "Source: the 25 run folders 20261116-* and 20261118-*-visit-robots from the school Pi.", fontsize=7.5, color=MUTED)
fig.tight_layout(rect=(0, 0.025, 1, 1))
fig.savefig(sys.argv[2])
