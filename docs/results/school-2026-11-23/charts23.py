#!/usr/bin/env python3
"""The four charts of the 23 Nov school report, from the JSON files the other scripts write.
Usage: python3 -I charts23.py <folder with digest.json pi23.json veers23.json standin.json> <turnback results.csv> <out folder>
Colours: the dataviz reference palette's first three categorical slots (blue, orange, aqua), checked with its validator;
grey is context only. One y-scale per panel. Changelog: 2026-11-24 first version."""
import csv, json, os, sys, statistics as S
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
IN, TB, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
J = lambda n: json.load(open(os.path.join(IN, n)))
INK, MUTED, GRID, CTX = "#0b0b0b", "#52514e", "#e4e3df", "#d9d8d3"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
SURF = "#fcfcfb"
plt.rcParams.update({"font.size": 8.5, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
                     "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF})
def clean(ax, grid="y"):
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    ax.grid(axis=grid, color=GRID, linewidth=0.8, zorder=0); ax.set_axisbelow(True)
def foot(fig, text): fig.text(0.01, 0.008, text, fontsize=7, color=MUTED, wrap=True)

# ---------------------------------------------------------------- 1: points reached against asked, 18 Nov beside 23 Nov
D = J("digest.json")
N18 = [  # from docs/results/school-2026-11-18/report.md, section 2.2 (runs that had a list; "really" figures)
 ("13:01", 3, 3, "done (one robot)"), ("13:29", 8, 8, "done"), ("13:43", 9, 9, "done"), ("14:30", 4, 3, "stopped by hand (far half unseen)"),
 ("15:03", 8, 8, "done"), ("16:11", 8, 7, "done on paper; plate contact"), ("16:42", 8, 8, "done"),
 ("17:09", 40, 33, "crashed at 292 s, shown as done"), ("17:32", 40, 26, "one robot; every robot out at 1007 s"),
 ("17:54", 7, 3, "one robot; held at the edge, out"), ("18:06", 40, 12, "stopped by hand: planner busy 87 %"), ("18:31", 40, 0, "crashed at 36 s, shown as done")]
END23 = {"1210": "done (15 cm/s); 2 points refused", "1219": "done; plate contact at 16 s, 4 points refused", "1233": "not started: naming read the wrong tag",
         "1237": "done, table share, no re-plan", "1247": "every robot out at 470 s", "1405": "done in 672 s (planned 316)", "1433": "done in 1327 s (planned 404); one robot out",
         "1508": "done", "1519": "stopped by hand at 409 s", "1649": "stopped by hand at 438 s", "1729": "stopped by hand at 299 s",
         "1754": "done, one robot", "1813": "one robot; ended by the card guard at 92 s", "1823": "not started: card full", "1825": "not started: card full", "1835": "done, one robot"}
rows23 = []
for r in D:
    k = r["run"][9:13]; pr = r.get("progress") or {}
    asked = sum(v["asked"] for v in pr.values()); got = sum(v["credited"] for v in pr.values())
    rows23.append(("%s:%s" % (k[:2], k[2:]), asked, got, END23.get(k, "")))
fig, axs = plt.subplots(1, 2, figsize=(13.5, 6.2), dpi=120, sharex=True)
for ax, rows, title in ((axs[0], N18, "18 Nov 2026 (code 015e311, before e23)"), (axs[1], rows23, "23 Nov 2026 (e23c + Jay-Sun's local changes)")):
    for i, (lab, a, g, note) in enumerate(rows):
        ax.barh(i, a, height=0.62, color=CTX, zorder=2, label="asked" if i == 0 else None)
        ax.barh(i, g, height=0.34, color=BLUE, zorder=3, label="stopped at (camera-checked)" if i == 0 else None)
        ax.text(max(a, 1) + 1, i, ("%d of %d   " % (g, a) if a else "0   ") + note, va="center", fontsize=7.6, color=INK)
    ax.set_yticks(range(len(rows))); ax.set_yticklabels([r[0] for r in rows]); ax.set_ylim(len(rows) - 0.4, -0.6)
    ax.set_xlim(0, 98); ax.set_xticks([0, 10, 20, 30, 40]); clean(ax, "x"); ax.set_title(title, loc="left", fontsize=9.5, color=INK)
    ax.set_xlabel("points, both robots together", loc="left")
axs[0].legend(loc="upper right", frameon=False, fontsize=8)
fig.suptitle("Visit points at the school table: points asked and points truly stopped at, per run that had a list", x=0.01, ha="left", fontsize=11, color=INK)
foot(fig, "Measured. 23 Nov: stops the run credited (coordinator.jsonl point_confidence), each checked against the camera's median position during the dwell; lists from each run's VISIT ORDER lines or split.json. "
          "18 Nov: the 18 Nov report's run table. Source: the school Pi's run folders 20261118-* and 20261123-*-visit-robots.")
fig.tight_layout(rect=(0, 0.035, 1, 0.95)); fig.savefig(os.path.join(OUT, "points_18_vs_23.png")); plt.close(fig)

# ---------------------------------------------------------------- 2: the Pi per run, 15 fps then 10 fps
P = J("pi23.json")
x = list(range(len(P))); labs = ["%s\n%d robot%s" % (p["clock"], p["robots"], "s" if p["robots"] > 1 else "") for p in P]
first10 = next(i for i, p in enumerate(P) if p["clock"] >= "15:00")
fig, axs = plt.subplots(4, 1, figsize=(11.5, 9.6), dpi=120, sharex=True)
def band(ax):
    ax.axvspan(first10 - 0.5, len(P) - 0.5, color="#f0efec", zorder=0)
ax = axs[0]; band(ax)
ax.plot(x, [p["cams"]["overhead"]["per_s"] for p in P], "-o", color=BLUE, lw=2, ms=6, label="camera 1 (overhead)", zorder=3)
ax.plot(x, [p["cams"]["side"]["per_s"] for p in P], "-s", color=ORANGE, lw=2, ms=6, label="camera 2 (side)", zorder=3)
ax.set_ylabel("pictures read\na second"); ax.set_ylim(0, 10); ax.legend(frameon=False, ncol=2, loc="upper left"); clean(ax)
ax.text(first10 - 0.45, 9.2, "site file says 10 pictures a second from here (15 before)", fontsize=7.8, color=MUTED)
ax = axs[1]; band(ax)
worst = [max(v["over1_time_pct"] for v in p["fixes"].values()) for p in P]; best = [min(v["over1_time_pct"] for v in p["fixes"].values()) for p in P]
ax.plot(x, worst, "-o", color=BLUE, lw=2, ms=6, label="the robot seen less", zorder=3)
ax.plot(x, best, "-s", color=ORANGE, lw=2, ms=6, label="the robot seen more", zorder=3)
ax.set_ylabel("% of the run more than\n1 s since its last fix"); ax.set_ylim(0, 50); ax.legend(frameon=False, ncol=2, loc="upper left"); clean(ax)
ax = axs[2]; band(ax)
for k, c, m, l in (("core0", BLUE, "o", "core 0 (recorder)"), ("core1", ORANGE, "s", "core 1 (planner)"), ("core2", AQUA, "^", "core 2 (tag bridge; core 3 is the same)")):
    ax.plot(x, [p[k] for p in P], "-" + m, color=c, lw=2, ms=6, label=l, zorder=3)
ax.set_ylabel("core busy %\n(median)"); ax.set_ylim(0, 112); ax.legend(frameon=False, ncol=3, loc="lower left"); clean(ax)
ax = axs[3]; band(ax)
ax.plot(x, [100.0 * p["throttled_now_rows"] / max(p["rows"], 1) for p in P], "-o", color=BLUE, lw=2, ms=6, zorder=3)
for i, p in enumerate(P): ax.text(i, 100.0 * p["throttled_now_rows"] / max(p["rows"], 1) + 5, "%.0f C" % p["temp_c"], ha="center", fontsize=7.5, color=INK)
ax.set_ylabel("% of readings under the\nsoft temperature limit"); ax.set_ylim(0, 118); clean(ax)
ax.set_xticks(x); ax.set_xticklabels(labs, fontsize=7.8)
fig.suptitle("The school Pi while each plan ran, 23 Nov 2026: pictures read, time without a fix, core load and heat (median temperature above each dot)", x=0.01, ha="left", fontsize=10.5, color=INK)
foot(fig, "Measured, from the plan's start to its last event: bridge/detect_times.csv (pictures the tag bridge finished), bridge/fixes.csv (fixes per robot), cpu.csv (cores), pi_monitor.csv (throttle flags, temperature). "
          "The change to 10 pictures a second is placed from the site file copied into the 15:08 run's export (dated 15:04); the logs do not record the capture rate itself. Source: the 13 run folders that drove.")
fig.tight_layout(rect=(0, 0.035, 1, 0.96)); fig.savefig(os.path.join(OUT, "pi_before_after_10fps.png")); plt.close(fig)

# ---------------------------------------------------------------- 3: turn-backs, real against the simulator's 20 runs
V = J("veers23.json")["runs"]
sim = list(csv.DictReader(open(TB)))
groups = [("simulator stand-in\ne23 as built (20 runs)", [60 * int(r["turnbacks"]) / float(r["run_s"]) for r in sim if r["config"] == "a"], CTX),
          ("simulator stand-in\nfinal code (20 runs)", [60 * int(r["turnbacks"]) / float(r["run_s"]) for r in sim if r["config"] == "b"], CTX),
          ("school table 23 Nov\ntwo robots (10 runs)", [60 * v["veers"] / v["plan_s"] for v in V if v["clock"] < "17:40"], BLUE),
          ("school table 23 Nov\none robot (3 runs)", [60 * v["veers"] / v["plan_s"] for v in V if v["clock"] >= "17:40"], BLUE)]
fig, axs = plt.subplots(1, 2, figsize=(12.5, 5.2), dpi=120, gridspec_kw={"width_ratios": [1.25, 1]})
ax = axs[0]
import random
random.seed(1)
for i, (lab, v, c) in enumerate(groups):
    ax.scatter([i + random.uniform(-0.16, 0.16) for _ in v], v, s=42, color=c if c != CTX else "#8d8c86", edgecolor=SURF, linewidth=1.2, zorder=3)
    ax.plot([i - 0.28, i + 0.28], [S.median(v)] * 2, color=INK, lw=2, zorder=4)
    ax.text(i + 0.31, S.median(v), "median %.1f" % S.median(v), va="center", fontsize=7.8, color=INK)
ax.set_xticks(range(len(groups))); ax.set_xticklabels([g[0] for g in groups], fontsize=8); ax.set_ylabel("turn-backs (veers) per minute of run"); clean(ax)
ax.set_xlim(-0.5, len(groups) - 0.3); ax.set_title("Turn-backs per minute; each dot is one run", loc="left", fontsize=9.5, color=INK)
ax = axs[1]
tot = J("veers23.json")["total"]
bars = [("sim, e23\nas built", sum(int(r["tb_first_moving_stall"]) for r in sim if r["config"] == "a"), 20, "#8d8c86"),
        ("sim, final\ncode", sum(int(r["tb_first_moving_stall"]) for r in sim if r["config"] == "b"), 20, "#8d8c86"),
        ("sim, old-fix\nswitch back", sum(int(r["tb_first_moving_stall"]) for r in sim if r["config"] == "c"), 20, "#8d8c86"),
        ("school table\n23 Nov", tot["first_yes"], tot["first_n"], BLUE)]
for i, (lab, k, n, c) in enumerate(bars):
    ax.bar(i, 100.0 * k / n, width=0.55, color=c, zorder=3)
    ax.text(i, 100.0 * k / n + 2, "%d of %d" % (k, n), ha="center", fontsize=8.5, color=INK)
ax.set_xticks(range(len(bars))); ax.set_xticklabels([b[0] for b in bars], fontsize=8); ax.set_ylim(0, 70); ax.set_ylabel("% of runs"); clean(ax)
ax.set_title("Runs with a turn-back at the first camera loss of a moving robot", loc="left", fontsize=9.5, color=INK)
fig.suptitle("Turn-backs: the school table on 23 Nov against the simulator's school-Pi stand-in (4 + 4 points, 20 runs a version)", x=0.01, ha="left", fontsize=10.5, color=INK)
foot(fig, "Measured: 'veer' events in each run's coordinator.jsonl per minute of plan time; moving = over 3 cm/s (the robot's own pose at the school, true position in the simulator). "
          "Sources: the 13 school run folders of 23 Nov that drove; docs/results/turnback-20runs-nov20/results.csv.")
fig.tight_layout(rect=(0, 0.04, 1, 0.94)); fig.savefig(os.path.join(OUT, "turnbacks_real_vs_sim.png")); plt.close(fig)

# ---------------------------------------------------------------- 4: what the stand-in should be: camera losses and re-plan cost
T = J("standin.json")
fig, axs = plt.subplots(1, 2, figsize=(12.5, 5.0), dpi=120)
ax = axs[0]
v = [min(x_, 5.0) for x_ in T["loss_lens_two_robot_runs"]]
ax.hist(v, bins=[1.0 + 0.1 * i for i in range(41)], color=BLUE, zorder=3, edgecolor=SURF, linewidth=0.6)
ax.axvline(1.5, color=INK, lw=1.5, ls="--", zorder=4); ax.text(1.53, ax.get_ylim()[1] * 0.93, "stand-in today: every loss 1.5 s", fontsize=8, color=INK)
ax.set_xlabel("length of one camera loss, s (losses over 5 s drawn at 5)"); ax.set_ylabel("camera losses"); clean(ax)
L2 = T["losses_2_robot_runs"]
ax.set_title("Two-robot runs: %d camera losses, %.1f a minute per robot (stand-in: 3 a minute)" % (L2["n"], L2["per_robot_minute"]), loc="left", fontsize=9.5, color=INK)
ax = axs[1]
rp = T["replans"]
for nrob, c, m, lab in ((2, BLUE, "o", "two robots"), (1, ORANGE, "s", "one robot")):
    ax.scatter([p[0] for p in rp if p[2] == nrob], [p[1] for p in rp if p[2] == nrob], s=30, color=c, marker=m, edgecolor=SURF, linewidth=0.8, zorder=3, label=lab)
f = T["replan_fit_all"]; xs = [0, 45]
ax.plot(xs, [f["fixed_s"] + f["s_per_point"] * q for q in xs], color=INK, lw=1.8, zorder=4, label="fitted: %.1f s + %.2f s a point" % (f["fixed_s"], f["s_per_point"]))
ax.plot(xs, [0.4 + 0.36 * q for q in xs], color=MUTED, lw=1.5, ls="--", zorder=4, label="the code's first guess: 0.4 s + 0.36 s a point")
ax.axhline(2.0, color=MUTED, lw=1, ls=":", zorder=2); ax.text(45, 2.2, "2 s budget", ha="right", fontsize=7.8, color=MUTED)
ax.set_xlabel("points in the re-plan"); ax.set_ylabel("time to compute, s"); ax.set_ylim(0, 35); ax.legend(frameon=False, fontsize=8, loc="upper left"); clean(ax)
ax.set_title("Every re-plan of 23 Nov (%d): time against points planned" % len(rp), loc="left", fontsize=9.5, color=INK)
fig.suptitle("What the Pi really did on 23 Nov, for setting the simulator's stand-in", x=0.01, ha="left", fontsize=10.5, color=INK)
foot(fig, "Measured: 'tracker_back' (lost_s) and 'replan' (cpu_s, points_planned) events in each run's coordinator.jsonl, plan time only. Source: the 13 school run folders of 23 Nov that drove.")
fig.tight_layout(rect=(0, 0.04, 1, 0.94)); fig.savefig(os.path.join(OUT, "standin_from_real.png")); plt.close(fig)
print("charts written to", OUT)
