#!/usr/bin/env python3
"""e22 report and weekly-summary charts (docs/screenshots/2026-11-13/e22-*.png) from the results in this folder.

    python3 docs/results/e22-nov13/charts.py
Every chart states what is measured and where the data come from (its title and source line); the report puts one
plain sentence under each.

Changelog: e22-2026-11-13a first version. e22c: source lines wrapped to the figure.
"""
import glob
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402
from matplotlib.patches import Ellipse, Rectangle   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
OUT = os.path.join(REPO, "docs", "screenshots", "2026-11-13")
BLUE, ORANGE, GREEN, GREY, RED, PURPLE = "#2952CC", "#D9822B", "#12805C", "#8A8F99", "#B42318", "#7A4FBF"
CODES = (("e20", "x25-e20", ORANGE), ("e21", "x25-e21", PURPLE), ("e22", "x25-e22", BLUE))
ROBOT_COL = {"Left": BLUE, "Center": ORANGE, "Right": PURPLE}
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})


def source(fig, text):
    import textwrap
    w = max(60, int(fig.get_size_inches()[0] * 15))      # (e22c) wrapped to the figure's width
    text = "\n".join(textwrap.fill(t, w) for t in text.split("\n"))
    fig.text(0.01, 0.01, text, fontsize=8, color="#5B6474", ha="left", va="bottom")


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=130)
    plt.close(fig)
    print(p)


def runs(d):
    out = {}
    for f in sorted(glob.glob(os.path.join(HERE, d, "*.json"))):
        out[os.path.basename(f)[:-5]] = json.load(open(f))
    return out


ORDER = ["measured-%d" % k for k in range(1, 9)] + ["harsh-%d" % k for k in range(1, 9)] + \
    ["measured-missing_cam-%d" % k for k in (1, 2, 3)] + ["measured-wrong_names-%d" % k for k in (1, 2, 3, 8)] + \
    ["measured-restart-1", "measured-skew-2"]


def label(k):
    return k.replace("measured-", "measured ").replace("harsh-", "harsh ").replace("missing_cam", "missing camera") \
        .replace("wrong_names", "wrong names").replace("-", " ")


def group(k):
    return "measured" if k.startswith("measured-") and k.count("-") == 1 else "harsh" if k.startswith("harsh") else \
        "camera and robot faults"


# 1. the error shapes on the map --------------------------------------------------------------------------------------
def shapes_chart():
    snaps = json.load(open(os.path.join(HERE, "shape_snaps.json")))
    pick = [s for s in snaps if s["t"] in (120.0, 200.0)] or snaps[:2]
    fig, axs = plt.subplots(1, len(pick), figsize=(5.2 * len(pick), 5.6))
    for ax, sn in zip(axs if len(pick) > 1 else [axs], pick):
        ax.add_patch(Rectangle((0, 0), 122, 122, facecolor="#EFE9DD", edgecolor="#8A7A5C", lw=1.5))
        for r in sn["robots"]:
            d, c = r["shape_draw"], ROBOT_COL.get(r["name"], GREY)
            ca, sa = math.cos(math.radians(d["ang"])), math.sin(math.radians(d["ang"]))
            ex, ey = d["x"] + d["off"] * ca, d["y"] + d["off"] * sa
            ax.add_patch(Ellipse((ex, ey), 2 * (d["a"] + d["iso"] + 4.7), 2 * (d["b"] + d["iso"] + 4.7), angle=d["ang"],
                                 facecolor=c, alpha=0.15, edgecolor=c, lw=1.8, ls="--" if r["lost"] else "-"))
            ax.plot([d["x"]], [d["y"]], "s", color=c, ms=9)
            ax.plot([r["truth"][0]], [r["truth"][1]], "x", color="black", ms=9, mew=2)
            A_, B_ = d["a"] + d["iso"] + 4.7, d["b"] + d["iso"] + 4.7
            top = ey + math.sqrt((A_ * sa) ** 2 + (B_ * ca) ** 2)
            ax.text(ex, min(126.0, top + 2.0), "%s (%s)" % (r["name"], r["stage"]), color=c, ha="center",
                    fontsize=9, fontweight="bold")
        ax.set_xlim(-8, 130)
        ax.set_ylim(-8, 132)
        ax.set_aspect("equal")
        ax.set_title("%.0f s into the run" % sn["t"])
        ax.set_xlabel("cm")
    fig.suptitle("Each robot's error shape on the board (square: where it thinks it is; x: where it really is)",
                 fontsize=11)
    source(fig, "Measured: the controller's error shape (body + 95 % error; dashed: no camera fix now) and the simulator's "
                "truth. Source: hard simulator, harsh profile seed 4, e22 code (shape_snaps.json)")
    fig.tight_layout(rect=(0, 0.06, 1, 0.95))
    save(fig, "e22-feature-error-shapes-on-the-map-chart.png")


# 2. the four stages --------------------------------------------------------------------------------------------------
def stages_chart():
    rows = []
    for code, d, col in CODES[1:]:
        R = runs(d)
        for g in ("measured", "harsh", "camera and robot faults"):
            st = {}
            for k, v in R.items():
                if group(k) != g:
                    continue
                for s, x in (v.get("stage_s") or {}).items():
                    st[s] = st.get(s, 0.0) + x
            T = sum(st.values()) or 1.0
            rows.append(("%s, %s" % (code, g), {s: 100.0 * st.get(s, 0.0) / T for s in ("run", "veer", "found",
                                                                                      "pause")}))
    fig, ax = plt.subplots(figsize=(10, 4.4))
    cols = {"run": GREEN, "veer": ORANGE, "found": BLUE, "pause": RED}
    ys = list(range(len(rows)))[::-1]
    for y, (lab, sh) in zip(ys, rows):
        x0 = 0.0
        for s in ("run", "veer", "found", "pause"):
            ax.barh(y, sh[s], left=x0, color=cols[s], height=0.6, label=s if y == ys[0] else None)
            if sh[s] >= 6:
                ax.text(x0 + sh[s] / 2, y, "%.0f %%" % sh[s], ha="center", va="center", color="white", fontsize=8)
            x0 += sh[s]
    ax.set_yticks(ys)
    ax.set_yticklabels([r[0] for r in rows])
    ax.set_xlim(0, 100)
    ax.set_xlabel("share of the robots' time in each of Nick's four stages, %")
    ax.legend(ncol=4, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.13))
    source(fig, "Measured: seconds each robot spent in run / veer / found / pause (the controller's stage, stage_s). "
                "Source: 25 hard-simulator runs each (e22 simulator), docs/results/e22-nov13/x25-e21/, x25-e22/")
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    save(fig, "e22-feature-four-stages-e21-vs-e22-chart.png")


# 3. the ratchet before / after -----------------------------------------------------------------------------------------
def ratchet_chart():
    P = ("p50", "p75", "p90", "p95", "p99")
    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    try:
        real = json.load(open(os.path.join(REPO, "docs", "results", "e21-nov12", "real_gap_1111.json")))
        for nm, key, ls in (("08:26", "20261111-082651-freeroam", "-"), ("08:58", "20261111-085837-freeroam", "--")):
            q = real.get(key)
            if isinstance(q, dict) and all(p in q for p in P):
                ax.plot(range(len(P)), [q[p] for p in P], ls, color="black", lw=1.5, label="real home run 11 Nov %s" % nm)
    except (OSError, ValueError):
        pass
    for code, d, col in (CODES[0], CODES[2]):
        R = runs(d)
        med = []
        for p in P:
            v = sorted(r["measures"]["own_cam_gap_cm"][p] for r in R.values() if r["measures"].get("own_cam_gap_cm"))
            med.append(v[len(v) // 2])
        ax.plot(range(len(P)), med, "o-", color=col, lw=2, label="%s code, simulator (median of 25 runs)" % code)
    ax.set_xticks(range(len(P)))
    ax.set_xticklabels(["half the time", "3 in 4", "9 in 10", "19 in 20", "99 in 100"])
    ax.set_xlabel("share of the time the gap stays under the value")
    ax.set_ylabel("own pose to camera, cm")
    ax.set_title("The ratchet: how far each robot's own pose drifts from where the camera sees it")
    ax.legend(frameon=False, fontsize=8)
    source(fig, "Measured: distance from each robot's own pose to the camera's spot, sampled every second. Source: "
                "x25-e20/ and x25-e22/ (hard simulator with the firmware's filter), real_gap_1111.json (11 Nov logs)")
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    save(fig, "e22-feature-ratchet-before-after-e20-vs-e22-chart.png")


# 4. standing time ----------------------------------------------------------------------------------------------------
def stands_chart():
    fig, ax = plt.subplots(figsize=(9.5, 7.6))
    ys = list(range(len(ORDER)))[::-1]
    for j, (code, d, col) in enumerate(CODES):
        R = runs(d)
        xs = [max(1.0, (R.get(k) or {}).get("broke", {}).get("longest_free_stand_s") or 1.0) for k in ORDER]
        ax.scatter(xs, [y + (j - 1) * 0.22 for y in ys], color=col, s=26, label=code, zorder=3)
    ax.axvline(60, color=RED, lw=1, ls="--")
    ax.text(62, ys[0] + 0.7, "60 s", color=RED, fontsize=8)
    ax.set_xscale("log")
    ax.set_yticks(ys)
    ax.set_yticklabels([label(k) for k in ORDER], fontsize=8)
    ax.set_xlabel("longest stand by a robot free to move, seconds (log scale)")
    ax.set_title("Standing: the longest stand in each run, e20 / e21 / e22")
    ax.legend(frameon=False, loc="lower right")
    source(fig, "Measured: the simulator's truth (a body neither moving nor turning), robots hung at an edge left out. "
                "Source: 25 runs each, the same seeds and faults, e22 simulator (x25-e20/, x25-e21/, x25-e22/)")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    save(fig, "e22-feature-standing-time-25-runs-e20-e21-e22-chart.png")


# 5. falls by seed --------------------------------------------------------------------------------------------------------
def falls_chart():
    fig, ax = plt.subplots(figsize=(6.4, 8.2))
    ax.set_xlim(-0.5, 2.5)
    ax.set_ylim(-0.5, len(ORDER) - 0.5)
    for j, (code, d, col) in enumerate(CODES):
        R = runs(d)
        for i, k in enumerate(ORDER):
            b = (R.get(k) or {}).get("broke", {})
            f, c = b.get("falls") or 0, 1 if b.get("contacts") else 0
            y = len(ORDER) - 1 - i
            if f or c:
                ax.add_patch(Rectangle((j - 0.45, y - 0.45), 0.9, 0.9, color=RED if f else ORANGE, alpha=0.25 + 0.25 * f))
            txt = ("%d fall%s" % (f, "s" if f > 1 else "") if f else "") + (" + contact" if c else "")
            ax.text(j, y, txt or "-", ha="center", va="center", fontsize=8, color="black" if (f or c) else GREY)
    ax.set_xticks([0, 1, 2])
    ax.set_xticklabels([c[0] for c in CODES])
    ax.set_yticks(range(len(ORDER)))
    ax.set_yticklabels([label(k) for k in ORDER[::-1]], fontsize=8)
    ax.set_title("Falls and body contact in each run")
    for s in ("left", "bottom"):
        ax.spines[s].set_visible(False)
    tot = []
    for code, d, col in CODES:
        R = runs(d)
        tot.append("%s %d falls, %d runs with contact" % (code, sum((v["broke"].get("falls") or 0) for v in R.values()),
                                                         sum(1 for v in R.values() if v["broke"].get("contacts"))))
    source(fig, "Measured: the simulator's truth (a middle over the edge; two bodies touching). Totals: " +
           "; ".join(tot) + ".\nSource: 25 runs each, the same seeds and faults, e22 simulator (x25-*/)")
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    save(fig, "e22-feature-falls-and-contacts-by-seed-chart.png")


# 6. the Pi's speed ---------------------------------------------------------------------------------------------------------
def timing_chart():
    d = json.load(open(os.path.join(HERE, "tick_timing.json")))
    fig, ax = plt.subplots(figsize=(8.5, 4.4))
    P = (("p50_ms", "half"), ("p95_ms", "19 in 20"), ("p99_ms", "99 in 100"), ("p999_ms", "999 in 1000"))
    for case in d["cases"]:
        col = BLUE if case["way"] == "e22" else ORANGE
        ls = "-" if case["profile"] == "measured" else "--"
        ax.plot(range(len(P)), [case[p] * d.get("pi_factor", 1.0) for p, _ in P], "o" + ls, color=col, lw=2,
                label=("planning paced (e22), %s profile" if case["way"] == "e22" else
                       "planning every tick in full (e21's way, same code), %s profile") % case["profile"])
    ax.axhline(50, color=RED, lw=1, ls="--")
    ax.text(0.02, 52, "the 50 ms loop", color=RED, fontsize=8)
    ax.set_yscale("log")
    ax.set_xticks(range(len(P)))
    ax.set_xticklabels([q for _, q in P])
    ax.set_xlabel("share of the ticks that take less than the value")
    ax.set_ylabel("controller tick, ms (estimated for the Pi 4)")
    ax.set_title("The controller's tick: planning every tick in full against planning paced (e22)")
    ax.legend(frameon=False, fontsize=8)
    source(fig, "Measured: every tick's time on the build machine x %.0f (the Pi 4 is about 5 x slower; earlier reports). "
                "Source: tools/pi/plan_timing_bench.py's runs, tick_timing.json" % d.get("pi_factor", 1.0))
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    save(fig, "e22-feature-pi-tick-timing-e21-vs-e22-chart.png")


if __name__ == "__main__":
    which = sys.argv[1:] or ["shapes", "stages", "ratchet", "stands", "falls", "timing"]
    for w in which:
        globals()[w + "_chart"]()
