#!/usr/bin/env python3
"""Charts for docs/results/monday-2robot-sweep.md and docs/results/safety-gap-doe.md, from monday2_sweep.py's
results.json.

    python3 tools/fleet/sweeps/monday2_charts.py --rows DIR/results.json --out docs/screenshots/2026-10-28
"""
import argparse
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

INK, MUTED, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
BLUE, ORANGE, RED, GREEN = "#2a78d6", "#eb6834", "#e34948", "#1f9e6a"
SIZE_COL = {"61x122": "#2a78d6", "66x127": "#1f9e6a", "70x130": "#c98a00", "76x137": "#7a3ec8"}
SRC = ("Source: simulator dry runs through the one-system stack (mission_run.py --dry: both simulated cameras, camera "
       "bridge, run_plan), simulator truth; sweep 2026-10-28")


def style(ax):
    ax.grid(axis="x", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=7.5)


def strip(ax, rows, conds, val, xlabel, ref=None, ref_label=None):
    ys = {c: i for i, c in enumerate(conds)}
    sizes = sorted({r["size"] for r in rows})
    off = {s: (k - (len(sizes) - 1) / 2) * 0.14 for k, s in enumerate(sizes)}
    for r in rows:
        v = val(r)
        if v is None or r["condition"] not in ys:
            continue
        fin = r.get("finished")
        c = SIZE_COL.get(r["size"], BLUE)
        ax.scatter([v], [ys[r["condition"]] + off[r["size"]]], s=22, zorder=3, color=c if fin else SURF,
                   edgecolors=c if fin else RED, linewidths=1.4 if not fin else 0.4)
    if ref is not None:
        ax.axvline(ref, color=ORANGE, lw=1.3, ls="--", zorder=2)
        if ref_label:
            ax.text(ref, len(conds) - 0.4, " " + ref_label, color=MUTED, fontsize=7, va="bottom")
    ax.set_yticks(range(len(conds)))
    ax.set_ylim(len(conds) - 0.5, -0.7)
    ax.set_xlabel(xlabel, fontsize=8, color=INK)
    style(ax)


def robustness(rows, conds, out):
    rows = [r for r in rows if abs(r.get("level", 8) - 8.0) < 1e-9]
    fig, axs = plt.subplots(1, 4, figsize=(13.5, 0.36 * len(conds) + 1.9), dpi=130, sharey=True)
    strip(axs[0], rows, conds, lambda r: r.get("closest_cm"), "closest two bodies, cm (truth)", 8.0, "8 cm net")
    strip(axs[1], rows, conds, lambda r: r.get("edge_cm"), "closest body to an open edge, cm (truth)", 0.0, "edge")
    strip(axs[2], rows, conds, lambda r: r.get("time_ratio"), "took / planned", 1.0, "plan")
    strip(axs[3], rows, conds, lambda r: max([v for v in (r.get("line_p95_cm") or {}).values() if v is not None] or
                                            [None]) if r.get("line_p95_cm") else None,
          "line check p95, worse robot, cm (camera)", 3.0, "3 cm gate")
    axs[0].set_yticklabels(conds, fontsize=7.5)
    hs = [plt.Line2D([], [], marker="o", ls="", color=c, label=s.replace("x", " x ")) for s, c in SIZE_COL.items()
          if any(r["size"] == s for r in rows)]
    hs.append(plt.Line2D([], [], marker="o", ls="", markerfacecolor=SURF, markeredgecolor=RED, label="did not finish"))
    fig.legend(handles=hs, loc="upper right", ncol=len(hs), fontsize=7.5, frameon=False)
    fig.text(0.01, 0.005, "Each dot is one dry run of Monday's 2-robot mission. " + SRC, fontsize=6.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    p = os.path.join(out, "monday2-sweep-margins-time-line-check.png")
    fig.savefig(p)
    plt.close(fig)
    return p


def stillness(rows, conds, out):
    rows = [r for r in rows if abs(r.get("level", 8) - 8.0) < 1e-9]
    fig, axs = plt.subplots(1, 2, figsize=(9.5, 0.36 * len(conds) + 1.9), dpi=130, sharey=True)
    strip(axs[0], rows, conds, lambda r: r.get("both_still_extra_s"), "both robots still at once, s beyond the plan",
          0.0, "as planned")
    strip(axs[1], rows, conds, lambda r: max([v.get("p95_cm") for v in (r.get("circle") or {}).values()] or [None]),
          "circle: distance from the 15 cm radius, p95, cm (truth)")
    axs[0].set_yticklabels(conds, fontsize=7.5)
    fig.text(0.01, 0.005, "Each dot is one dry run. " + SRC, fontsize=6.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    p = os.path.join(out, "monday2-sweep-standstill-circle.png")
    fig.savefig(p)
    plt.close(fig)
    return p


def doe(rows, out):
    levels = sorted({r["level"] for r in rows})
    conds = []
    for r in rows:
        if r["level"] != 8.0 and r["condition"] not in conds:
            conds.append(r["condition"])
    rows = [r for r in rows if r["condition"] in conds and r.get("closest_cm") is not None
            and r["size"] in ("61x122", "76x137")]                # the DOE's tables (the 8 cm sweep also ran others)
    ccol = dict(zip(conds, ["#2a78d6", "#eb6834", "#7a3ec8", "#1f9e6a", "#c98a00", "#d6409f", "#52514e"]))
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.3), dpi=130)
    ax = axs[0]
    for k, lv in enumerate(levels):
        rr = [r for r in rows if r["level"] == lv]
        for j, c in enumerate(conds):
            v = [r["closest_cm"] for r in rr if r["condition"] == c]
            x = [k + (j - (len(conds) - 1) / 2) * 0.09] * len(v)
            ax.scatter(x, v, s=18, color=ccol[c], zorder=3, label=c if k == 0 else None, linewidths=0)
        pl = [r["plan_closest_cm"] for r in rr]
        if pl:
            ax.hlines(sum(pl) / len(pl), k - 0.4, k + 0.4, color=INK, lw=1.2, zorder=2)
        ax.hlines(lv, k - 0.4, k + 0.4, color=RED, lw=1.2, ls="--", zorder=2)
    ax.set_xticks(range(len(levels)))
    ax.set_xticklabels(["net %g cm" % lv for lv in levels], fontsize=8)
    ax.set_ylabel("closest two bodies came, cm (simulator truth)", fontsize=8, color=INK)
    ax.set_ylim(0, None)
    ax.grid(axis="y", color=GRID, lw=0.8)
    style(ax)
    ax.grid(axis="x", visible=False)
    ax.set_title("Closest approach by safety net (black: planned, red dashed: the net)", fontsize=8.5, color=INK,
                 loc="left")
    ax = axs[1]
    for k, lv in enumerate(levels):
        rr = [r for r in rows if r["level"] == lv]
        for j, c in enumerate(conds):
            v = [r["plan_closest_cm"] - r["closest_cm"] for r in rr if r["condition"] == c]
            x = [k + (j - (len(conds) - 1) / 2) * 0.09] * len(v)
            ax.scatter(x, v, s=18, color=ccol[c], zorder=3, linewidths=0)
    ax.axhline(0, color=MUTED, lw=0.8)
    ax.set_xticks(range(len(levels)))
    ax.set_xticklabels(["net %g cm" % lv for lv in levels], fontsize=8)
    ax.set_ylabel("planned closest minus real closest, cm", fontsize=8, color=INK)
    ax.grid(axis="y", color=GRID, lw=0.8)
    style(ax)
    ax.grid(axis="x", visible=False)
    ax.set_title("How much closer than planned they came (above 0: closer)", fontsize=8.5, color=INK, loc="left")
    fig.legend(loc="lower center", ncol=4, fontsize=7, frameon=False)
    fig.text(0.01, 0.005, "Each dot is one dry run (61 x 122 and 76 x 137 tables, seeds 1-3). " + SRC, fontsize=6.5,
             color=MUTED)
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    p = os.path.join(out, "safety-gap-doe-closest-by-net.png")
    fig.savefig(p)
    plt.close(fig)
    return p


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--rows", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    rows = json.load(open(a.rows))
    import monday2_sweep as M2
    conds = [c for c in M2.CONDS if any(r["condition"] == c for r in rows)]
    os.makedirs(a.out, exist_ok=True)
    print(robustness(rows, conds, a.out))
    print(stillness(rows, conds, a.out))
    if any(r.get("level") != 8.0 for r in rows):
        print(doe(rows, a.out))


if __name__ == "__main__":
    main()
