#!/usr/bin/env python3
"""e21 report charts (docs/screenshots/2026-11-12/e21-*.png) from the results in this folder.

    python3 docs/results/e21-nov12/charts.py
Every chart states what is measured and where the data comes from (title and source line).

Changelog: e21-2026-11-12a first version. e21-2026-11-13b the stands chart shows e21 with both camera verdicts; the
stage and gap charts read the final default (x25-e21-whole).
"""
import glob
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402
from matplotlib.patches import Ellipse, Circle   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
OUT = os.path.join(REPO, "docs", "screenshots", "2026-11-12")
sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
import error_shape as ES          # noqa: E402
import odo_trust as OT            # noqa: E402

BLUE, ORANGE, GREEN, GREY, RED, PURPLE = "#2952CC", "#D9822B", "#12805C", "#8A8F99", "#B42318", "#7A4FBF"
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})


def source(fig, text):
    fig.text(0.01, 0.01, text, fontsize=8, color="#5B6474", ha="left", va="bottom")


def save(fig, name):
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=130)
    plt.close(fig)
    print(p)


def cone_chart():
    d = json.load(open(os.path.join(HERE, "cone_fit.json")))
    st = [r for r in d["stretches"] if not r["events"]]
    a = d["fit"]["along_sides"]
    q = d["fit"]["quiet_q95"]
    fig, axs = plt.subplots(1, 2, figsize=(10.5, 4.3), sharey=False)
    ax = axs[0]
    ax.scatter([r["s"] for r in st], [r["along"] for r in st], s=6, color=BLUE, alpha=0.35)
    xs = [15, 60]
    ax.plot(xs, [a["b_fwd"] + a["a_fwd"] * x for x in xs], color=ORANGE, lw=1.8, label="ahead, 97.5 %")
    ax.plot(xs, [-min(x, a["b_back"]) for x in xs], color=RED, lw=1.8, label="back along its track, 97.5 %")
    ax.axhline(0, color=GREY, lw=0.8)
    ax.set_xlabel("cm driven without a camera fix")
    ax.set_ylabel("error ALONG its own track, cm (+ ahead)")
    ax.set_ylim(-36, 12)
    ax.legend(frameon=False, fontsize=8, loc="lower left")
    ax.set_title("Along the track: one-sided")
    ax = axs[1]
    ax.scatter([r["s"] for r in st], [abs(r["across"]) for r in st], s=6, color=BLUE, alpha=0.35)
    ax.plot(xs, [q["b_across"] + q["a_across"] * x for x in xs], color=ORANGE, lw=1.8, label="95 %")
    ax.set_xlabel("cm driven without a camera fix")
    ax.set_ylabel("error ACROSS its track (sideways), cm")
    ax.legend(frameon=False, fontsize=8)
    ax.set_title("Sideways: the cone widens")
    fig.suptitle("Where a robot really is after driving blind, against its own pose (%d stretches)" % len(st))
    source(fig, "Measured: home free roams 5, 7, 11 Nov (4 runs), stretches of 15-60 cm with no camera fix taken and no "
                "reported event; own pose from follow.csv, truth from the camera (bridge/track.csv).\n"
                "Source: docs/results/e21-nov12/cone_fit.py, cone_fit.json")
    fig.tight_layout(rect=(0, 0.09, 1, 0.95))
    save(fig, "e21-feature-measured-cone-along-and-across-chart.png")


def shape_chart():
    fig, ax = plt.subplots(figsize=(8.6, 5.0))
    tr = OT.Trust("x")
    sh = ES.Shape("x", tr)
    sh.anchor(0.0, 20.0, 50.0, 0.0, 2.45, "fix")
    pts = []
    x, y, h = 20.0, 50.0, 0.0
    for k in range(70):
        h += 1.2 if k > 20 else 0.0
        x += math.cos(math.radians(h))
        y += math.sin(math.radians(h))
        sh.drive(x, y, h, 1.0, 1.2 if k > 20 else 0.0, False)
        pts.append((x, y))
        if k in (19, 44, 69):
            i, a, b, ang, off, _gx, _gy = sh.ellipse()
            cx = x + off * math.cos(math.radians(ang))
            cy = y + off * math.sin(math.radians(ang))
            ax.add_patch(Ellipse((cx, cy), 2 * (a + i), 2 * (b + i), angle=ang, fc=BLUE, alpha=0.15, ec=BLUE))
            old = 2.0 + 0.07 * sh.s
            ax.add_patch(Circle((x, y), old, fc="none", ec=ORANGE, lw=1.6, ls="--"))
            ax.text(x + 1, y - 3, "%.0f cm" % sh.s, fontsize=8, color="#333")
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#333", lw=1.2, label="its own dead-reckoned track")
    ax.plot([], [], color=BLUE, lw=6, alpha=0.3, label="e21: its error shape (cone along the track + disc)")
    ax.plot([], [], color=ORANGE, lw=1.6, ls="--", label="e20: the circle bubble (2 cm + 0.07 cm per cm)")
    ax.set_aspect("equal")
    ax.set_xlim(10, 95)
    ax.set_ylim(25, 100)
    ax.set_xlabel("cm")
    ax.set_ylabel("cm")
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    ax.set_title("A lost robot's possible position after 20, 45 and 70 cm blind (the fleet prior)")
    source(fig, "Computed: tools/fleet/error_shape.py with the measured prior (cone_fit.json), a straight 20 cm then a "
                "curve; the e20 circle for comparison.\nSource: docs/results/e21-nov12/charts.py")
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    save(fig, "e21-feature-error-shape-against-the-e20-circle-chart.png")


def replay_chart():
    d = json.load(open(os.path.join(HERE, "replay_escape.json")))
    cats = sorted(d["by_cat"])
    fig, ax = plt.subplots(figsize=(9.6, 4.8))
    for i, c in enumerate(cats):
        rows = [r for r in d["rows"] if r["cat"] == c]
        fired = [r for r in rows if r["fired"]]
        ys = [i] * len(fired)
        ax.scatter([r["sooner_s"] for r in fired], ys, s=14, color=RED if c.startswith("C") else BLUE, alpha=0.6)
        ax.text(1.05e3, i, "%d of %d" % (len(fired), len(rows)), va="center", fontsize=8)
    ax.set_yticks(range(len(cats)))
    short = {"A": "pushed or bumped", "B": "wheels turned, robot did not move", "C": "camera confused with another robot",
             "D": "own pose jumped (a re-seed landed wrong)", "E": "drifted, every fix refused by its gate",
             "F": "drifted, fixes reaching it", "G": "standing or short, no event"}
    ax.set_yticklabels([short.get(c[0], c[2:40]) for c in cats], fontsize=8)
    ax.set_xscale("log")
    ax.set_xlim(0.3, 3e3)
    ax.set_xlabel("seconds sooner the new rule would have corrected its own pose (log scale)")
    ax.set_title("The 147 divergence episodes of 8 home runs, replayed through the consistency escape")
    source(fig, "Replay of real logs (5-11 Nov, the audit's onsets.json): camera sightings (bridge/track.csv) against each "
                "robot's own pose and filter sd (follow.csv).\nRed: camera mix-ups, where a firing could be a wrong "
                "correction. Source: docs/results/e21-nov12/replay_escape.py, replay_escape.json")
    fig.tight_layout(rect=(0, 0.09, 0.92, 1))
    save(fig, "e21-feature-replay-escape-corrects-sooner-chart.png")


def _runs(d):
    out = {}
    for f in glob.glob(os.path.join(d, "*.json")):
        out[os.path.basename(f)[:-5]] = json.load(open(f))
    return out


def gap_chart(acc):
    real = json.load(open(os.path.join(HERE, "real_gap_1111.json")))
    ps = ("p50", "p75", "p90", "p95", "p99")
    fig, ax = plt.subplots(figsize=(8.6, 4.6))
    xs = [50, 75, 90, 95, 99]
    for k, v in real.items():
        ax.plot(xs, [v[p] for p in ps], color="#111", lw=2.2, marker="o",
                label="real home run %s:%s" % (k[9:11], k[11:13]))
    sets = [("e20 code, the old simulator (no firmware filter)",
             os.path.join(REPO, "docs", "results", "e20-nov12", "extra", "e20"), GREY),
            ("e20 code, the e21 simulator (the firmware's filter: the ratchet)", os.path.join(acc, "x25-e20"), ORANGE),
            ("e21 code, the e21 simulator", os.path.join(acc, "x25-e21-whole"), BLUE)]
    for lab, d, col in sets:
        R = {k: v for k, v in _runs(d).items() if k.startswith("measured-") and k.count("-") == 1}
        if not R:
            continue
        med = []
        for p in ps:
            vals = sorted(v["measures"]["own_cam_gap_cm"][p] for v in R.values())
            med.append(vals[len(vals) // 2])
        ax.plot(xs, med, color=col, lw=1.6, marker="s", label="%s (median of %d runs)" % (lab, len(R)))
    ax.set_yscale("log")
    ax.set_xlabel("percentile of the time")
    ax.set_ylabel("own pose to camera, cm (log scale)")
    ax.set_title("How far each robot's own pose was from the camera")
    ax.legend(frameon=False, fontsize=7.5, loc="upper left")
    source(fig, "Real: 11 Nov 08:26 and 08:58 home runs (follow.csv against bridge/track.csv). Simulated: hard simulator, "
                "measured profile, seeds 1-8, 0.5 h each.\nSource: docs/results/e21-nov12/real_gap_1111.json, "
                "docs/results/e21-nov12/x25-*/, docs/results/e20-nov12/extra/e20/")
    fig.tight_layout(rect=(0, 0.09, 1, 1))
    save(fig, "e21-problem-simulator-ratchet-own-pose-gap-real-vs-sim-chart.png")


def stands_chart(acc):
    a, b = _runs(os.path.join(acc, "x25-e20")), _runs(os.path.join(acc, "x25-e21-whole"))
    pc = _runs(os.path.join(acc, "x25-e21"))
    keys = sorted(set(a) & set(b), key=lambda k: (k.split("-")[0] != "measured" or "_" in k, k))
    if not keys:
        return
    fig, ax = plt.subplots(figsize=(8.6, 7.4))
    for i, k in enumerate(keys):
        va, vb = a[k]["broke"].get("longest_free_stand_s") or 0, b[k]["broke"].get("longest_free_stand_s") or 0
        ax.plot([va, vb], [i, i], color="#D0D5DD", lw=1.5, zorder=1)
        ax.scatter([va], [i], color=ORANGE, s=28, zorder=2)
        ax.scatter([vb], [i], color=BLUE, s=28, zorder=3)
        if k in pc:
            vc = pc[k]["broke"].get("longest_free_stand_s") or 0
            ax.scatter([vc], [i], facecolors="none", edgecolors=BLUE, s=28, zorder=3, lw=1)
            brc = pc[k]["broke"]
            if brc.get("falls") or brc.get("contacts"):
                ax.text(vc + 8, i - 0.35, "%s%s" % ("F%d " % brc["falls"] if brc.get("falls") else "",
                                                    "C%d" % brc["contacts"] if brc.get("contacts") else ""),
                        color="#7A8CC9", fontsize=6, va="center")
        for v, col, br in ((va, ORANGE, a[k]["broke"]), (vb, BLUE, b[k]["broke"])):
            if br.get("falls") or br.get("contacts"):
                ax.text(v + 8, i, "%s%s" % ("F%d " % br["falls"] if br.get("falls") else "",
                                           "C%d" % br["contacts"] if br.get("contacts") else ""), color=col, fontsize=7,
                        va="center")
    ax.scatter([], [], color=ORANGE, label="e20 code (the e21 simulator)")
    ax.scatter([], [], color=BLUE, label="e21 code (camera verdict whole: the default)")
    ax.scatter([], [], facecolors="none", edgecolors=BLUE, label="e21 code, the per-camera verdict")
    ax.axvline(60, color=GREY, ls="--", lw=1)
    ax.set_yticks(range(len(keys)))
    ax.set_yticklabels(keys, fontsize=8)
    ax.set_xlabel("longest stand by a robot free to move, s  (F = falls, C = runs with contact)")
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    ax.set_title("25 hard-simulator runs of 0.5 h, the same seeds: e20 against e21")
    source(fig, "Simulated: hard simulator (with the firmware's filter, bumps and slips), the 25 seeds and faults of e20's "
                "comparison.\nSource: docs/results/e21-nov12/x25-e20/, x25-e21-whole/, x25-e21/ (hardsim_batch.py)")
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    save(fig, "e21-problem-25-runs-stands-falls-contacts-e20-vs-e21-chart.png")


def stage_chart(acc):
    b = _runs(os.path.join(acc, "x25-e21-whole"))
    groups = {"measured": [], "harsh": [], "camera faults": []}
    for k, v in b.items():
        g = "harsh" if k.startswith("harsh") else ("measured" if k.count("-") == 1 else "camera faults")
        groups[g].append(v.get("stage_s") or {})
    st = ("run", "veer", "found", "pause")
    fig, ax = plt.subplots(figsize=(8.6, 4.4))
    w = 0.25
    for j, (g, rows) in enumerate(groups.items()):
        if not rows:
            continue
        tot = {s: sum(r.get(s, 0.0) for r in rows) for s in st}
        T = sum(tot.values()) or 1.0
        ax.bar([i + (j - 1) * w for i in range(4)], [100.0 * tot[s] / T for s in st], w,
               label="%s (%d runs)" % (g, len(rows)), color=(GREEN, RED, PURPLE)[j])
        for i, s in enumerate(st):
            ax.text(i + (j - 1) * w, 100.0 * tot[s] / T + 1, "%.0f" % (100.0 * tot[s] / T), ha="center", fontsize=7)
    ax.set_xticks(range(4))
    ax.set_xticklabels(["run", "veer", "found\n(its own pose being corrected)", "pause and search"])
    ax.set_ylabel("share of robot time, %")
    ax.legend(frameon=False, fontsize=8)
    ax.set_title("Time in each of Nick's four stages, e21, hard simulator")
    source(fig, "Simulated: the 25 e21 runs (0.5 h each), the controller's stage clock (freeroam_core.stage_watch).\n"
                "Source: docs/results/e21-nov12/x25-e21-whole/ (stage_s)")
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    save(fig, "e21-feature-time-in-each-stage-chart.png")


def cost_chart():
    p = os.path.join(HERE, "plan_cost.json")
    if not os.path.exists(p):
        return
    d = json.load(open(p))
    fig, axs = plt.subplots(1, 2, figsize=(10.0, 4.2))
    ax = axs[0]
    acc = d["accuracy"]
    ks = ("circle", "capsule", "ellipse")
    ax.bar(range(3), [acc[k]["lost_median_cm"] for k in ks], 0.4, color=GREY, label="median")
    ax.bar([i + 0.4 for i in range(3)], [acc[k]["lost_p90_cm"] for k in ks], 0.4, color=BLUE, label="1 in 10")
    ax.set_xticks([i + 0.2 for i in range(3)])
    ax.set_xticklabels(["circle of its\nlargest reach", "capsule", "ellipse along the\nline between them (e21)"])
    ax.set_ylabel("room given away, cm (0 = exact)")
    ax.set_title("How much room each bound wastes")
    ax.legend(frameon=False, fontsize=8)
    for i, k in enumerate(ks):
        ax.text(i + 0.2, max(acc[k]["lost_p90_cm"], acc[k]["lost_median_cm"]) + 0.3, "unsafe: %d" % acc[k]["unsafe"],
                ha="center", fontsize=7)
    ax = axs[1]
    labs, vals, cols = [], [], []
    for k, lab, col in (("tick_e20", "e20", ORANGE), ("tick_e21", "e21", BLUE)):
        t = d.get(k) or {}
        if "mean_ms" in t:
            for q, nm in (("mean_ms", "mean"), ("p95_ms", "95 %"), ("p99_ms", "99 %")):
                labs.append("%s\n%s" % (lab, nm))
                vals.append(5.0 * t[q])
                cols.append(col)
    ax.bar(range(len(vals)), vals, color=cols)
    ax.axhline(50, color=GREY, ls="--", lw=1)
    ax.text(0, 51, "the 50 ms loop", fontsize=7, color="#5B6474")
    ax.set_xticks(range(len(vals)))
    ax.set_xticklabels(labs, fontsize=8)
    ax.set_ylabel("planner time per tick on the Pi, ms (x5 this machine)")
    ax.set_title("The whole planner's tick, scaled to the Pi")
    source(fig, "Left: 2,000 random shape pairs against the exact gap (error_shape.true_gap). Right: the free-roam tick in the "
                "fast simulator, measured profile seed 1, 0.1 h,\non this 2-core machine x 5 (the Pi 4's measured ratio). "
                "Source: docs/results/e21-nov12/plan_cost.py, plan_cost.json")
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    save(fig, "e21-feature-shape-bound-accuracy-and-planner-cost-chart.png")


if __name__ == "__main__":
    acc = sys.argv[1] if len(sys.argv) > 1 else HERE
    cone_chart()
    shape_chart()
    replay_chart()
    gap_chart(acc)
    stands_chart(acc)
    stage_chart(acc)
    cost_chart()
