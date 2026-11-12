#!/usr/bin/env python3
"""e20 report charts (docs/screenshots/2026-11-12/e20-*.png) from the results in this folder.

    python3 docs/results/e20-nov12/charts.py
Every chart states what is measured and where the data comes from (title and source line)."""
import glob
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
OUT = os.path.join(REPO, "docs", "screenshots", "2026-11-12")
sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
import camera_trust as CT          # noqa: E402

BLUE, ORANGE, GREEN, GREY, RED = "#2952CC", "#D9822B", "#12805C", "#8A8F99", "#B42318"
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})


def source(fig, text):
    fig.text(0.01, 0.01, text, fontsize=8, color="#5B6474", ha="left", va="bottom")


def replay_chart():
    d = json.load(open(os.path.join(HERE, "replay_trust_home_1111.json")))
    fig, ax = plt.subplots(figsize=(8.2, 4.2))
    for cam, col in (("overhead", BLUE), ("side", ORANGE)):
        b = CT.TrustBook("x")
        lo, hi, xs = [], [], []
        for k, r in enumerate([r for r in d["rows"] if r["cam"] == cam and r["sure"]]):
            b.observe(cam, "ml", "heading", None, None, r["err_own"] <= CT.HEAD_K * math.hypot(15.0, 2.0))
            s = b.score(cam, "ml", "heading", area=None)
            xs.append(k + 1)
            lo.append(s["lower"])
            hi.append(s["upper"])
        ax.fill_between(xs, lo, hi, color=col, alpha=0.25, lw=0)
        ax.plot(xs, lo, color=col, lw=1.6, label="%s camera (lower and upper bound)" % cam)
        ax.plot(xs, hi, color=col, lw=0.8)
    ax.axhline(CT.R_STAR, color=GREY, ls="--", lw=1)
    ax.text(140, 0.55, "dashed line 0.90: trusted once the lower bound is above it,\nnot trusted once the upper bound "
            "is below it", fontsize=8, color="#5B6474")
    ax.set_xlabel("detector heading checks so far (each against the robot's own confirmed heading)")
    ax.set_ylabel("share of checks that agreed")
    ax.set_ylim(0, 1.02)
    ax.set_title("Learned heading trust of each camera's detector, 11 Nov home pictures (replay)")
    ax.legend(loc="lower right", frameon=False, fontsize=9)
    source(fig, "Measured: detector-v1 headings on all 363 pictures of the 11 Nov 08:58 home run, against each robot's own "
                "heading where its travel confirmed it.\nSource: tools/analysis/tagfree_replay.py --trust, "
                "docs/results/e20-nov12/replay_trust_home_1111.json")
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    p = os.path.join(OUT, "e20-feature-replay-learned-heading-trust-overhead-vs-side-chart.png")
    fig.savefig(p, dpi=130)
    print(p)


def stands_chart(e19_dir, e20_dir):
    def rows(ds):
        out = {}
        for d in ds:
            for f in glob.glob(os.path.join(d, "*.json")) + glob.glob(os.path.join(d, "*", "*.json")):
                r = json.load(open(f))
                if "broke" in r:
                    out[os.path.basename(f)[:-5]] = r["broke"].get("longest_free_stand_s") or 0.0
        return out
    a = rows([e19_dir, os.path.join(HERE, "extra", "e19")])
    b = rows([e20_dir, os.path.join(HERE, "extra", "e20")])
    keys = sorted(set(a) & set(b), key=lambda k: (k.split("-")[0] != "measured" or "_" in k, k))
    fig, ax = plt.subplots(figsize=(8.6, 7.4))
    y = list(range(len(keys)))
    for i, k in enumerate(keys):
        ax.plot([a[k], b[k]], [i, i], color="#D0D5DD", lw=1.5, zorder=1)
    ax.scatter([a[k] for k in keys], y, color=ORANGE, label="e19", zorder=2, s=28)
    ax.scatter([b[k] for k in keys], y, color=BLUE, label="e20", zorder=3, s=28)
    ax.axvline(60, color=GREY, ls="--", lw=1)
    ax.text(62, -0.9, "60 s: the measured-profile limit", fontsize=8, color="#5B6474")
    ax.set_yticks(y)
    ax.set_yticklabels(keys, fontsize=8.5)
    ax.set_xscale("symlog", linthresh=10)
    ax.set_xlabel("longest stand by a robot free to move (s, log scale past 10 s; harsh rows include robots hung by the fault)")
    ax.set_title("Never still: the longest stand by a robot free to move,\nhard simulator, e19 against e20 (same seeds)")
    ax.set_xlim(0, 1000)
    ax.legend(frameon=False, loc="upper left", bbox_to_anchor=(0.0, 1.0))
    source(fig, "Measured: 25 hard-simulator runs of 0.5 h (the long checks, harsh and measured seeds 1-8, extra camera-fault "
                "seeds), same seeds.\nSource: docs/results/e19-nov12/hardsim-long, docs/results/e20-nov12/hardsim-long "
                "and docs/results/e20-nov12/extra")
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    p = os.path.join(OUT, "e20-feature-hardsim-free-stands-e19-vs-e20-chart.png")
    fig.savefig(p, dpi=130)
    print(p)


def scenario_chart(name, title, marks):
    d = json.load(open(os.path.join(HERE, "scenario_%s_1.json" % name)))
    tl = d["timeline"]
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.8), sharey=True)
    for ax, key, lab in ((axs[0], "pos_agree", "motion against each robot's own wheels"),
                         (axs[1], "frame_agree", "position against the other cameras")):
        for cam, col in (("cam-a", BLUE), ("cam-b", ORANGE), ("cam-c", GREEN)):
            xs = [r["t"] / 60.0 for r in tl if r[cam].get(key) is not None]
            ys = [r[cam][key] for r in tl if r[cam].get(key) is not None]
            ax.plot(xs, ys, color=col, lw=1.6, label=cam)
        for t0, t1, w in marks:
            ax.axvspan(t0 / 60.0, (t1 if t1 else tl[-1]["t"]) / 60.0, color="#F2F4F7", zorder=0)
        ax.set_title(lab, fontsize=10)
        ax.set_xlabel("minutes into the run")
        ax.set_ylim(0, 1.02)
    axs[0].set_ylabel("share of recent checks that agreed")
    axs[0].legend(frameon=False, loc="lower left", fontsize=9)
    fig.suptitle(title, fontsize=11)
    source(fig, "Measured: the hard simulator (11 Nov profile), three simulated cameras, seed 1, 15 min; grey band: %s.\n"
                "Source: docs/results/e20-nov12/cam_scenarios.py, scenario_*_1.json" % marks[0][2])
    fig.tight_layout(rect=(0, 0.08, 1, 0.95))
    p = os.path.join(OUT, ("e20-%s-scenario-%s-trust-timeline-chart.png" % ("problem" if name == "swap" else "feature", name)))
    fig.savefig(p, dpi=130)
    print(p)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    replay_chart()
    stands_chart(os.path.join(REPO, "docs", "results", "e19-nov12", "hardsim-long"),
                 os.path.join(HERE, "hardsim-long"))
    T = 15 * 60.0
    scenario_chart("bump", "A camera bumped 6 deg mid-run (cam-a): which checks notice", [(0.4 * T, None, "cam-a bumped")])
    scenario_chart("swap", "Two cameras' names swapped for a spell (cam-b and cam-c)", [(0.3 * T, 0.5 * T,
                                                                                          "names swapped")])
