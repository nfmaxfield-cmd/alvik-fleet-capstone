#!/usr/bin/env python3
"""Charts for docs/results/monday-mission-sweep.md from the sweep's rows (monday_sweep.py collect -> results.json) and
its table-size planner check (planfit.json).

    python3 tools/fleet/sweeps/sweep_charts.py --rows docs/results/data/monday-sweep/results.json \
        --planfit docs/results/data/monday-sweep/planfit.json --out docs/screenshots/2026-10-27
"""
import argparse
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

INK, MUTED, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
BLUE, ORANGE, RED = "#2a78d6", "#eb6834", "#e34948"
SOURCE = "Source: simulator dry runs (run_plan.py --dry-run, fleet_sim truth), Monday sweep 2026-10-27"


def order(rows):
    groups, seen = [], []
    for r in rows:
        if r["condition"] not in seen:
            seen.append(r["condition"])
            groups.append((r["group"], r["condition"]))
    return groups


def strip(ax, rows, conds, key, xlabel, ref=None, ref_label=None, clip=None):
    ys = {c: i for i, (_, c) in enumerate(conds)}
    for r in rows:
        v = r.get(key)
        if v is None:
            continue
        if clip is not None:
            v = max(clip[0], min(clip[1], v))
        fin = r.get("finished")
        ax.scatter([v], [ys[r["condition"]]], s=46, zorder=3, color=BLUE if fin else SURF,
                   edgecolors=BLUE if fin else RED, linewidths=1.8 if not fin else 0.8)
    if ref is not None:
        ax.axvline(ref, color=ORANGE, lw=1.5, ls="--", zorder=2)
        if ref_label:
            ax.text(ref, len(conds) - 0.4, " " + ref_label, color=MUTED, fontsize=8, va="bottom")
    ax.set_xlabel(xlabel, color=INK, fontsize=9)
    ax.grid(axis="x", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)


def group_bands(ax, conds):
    prev = None
    for i, (g, _) in enumerate(conds):
        if g != prev and i > 0:
            ax.axhline(i - 0.5, color=GRID, lw=0.8)
        prev = g


def label_rows(ax, conds):
    ax.set_yticks(range(len(conds)))
    ax.set_yticklabels(["%s  ·  %s" % (g, c) if True else c for g, c in conds], fontsize=8, color=INK)
    ax.set_ylim(len(conds) - 0.5, -0.5)


def safety(rows, out):
    conds = order(rows)
    fig, axs = plt.subplots(1, 3, figsize=(12, 0.32 * len(conds) + 1.9), sharey=True)
    strip(axs[0], rows, conds, "closest_cm", "Closest two robot bodies came (cm)", ref=8.0, ref_label="plan's floor 8 cm")
    strip(axs[1], rows, conds, "edge_cm", "Closest a body came to an open edge (cm; < 0 = over it)", ref=0.0,
          ref_label="edge")
    strip(axs[2], rows, conds, "brick_cm", "Closest a body came to the brick (cm)", ref=0.0, clip=(-1, 30))
    label_rows(axs[0], conds)
    for ax in axs:
        group_bands(ax, conds)
    fig.suptitle("Monday's routine under school conditions: safety margins per run (filled = finished, red ring = "
                 "did not finish)", fontsize=10, color=INK, x=0.01, ha="left")
    fig.text(0.01, 0.005, SOURCE + "; body gaps measured on the simulator's true poses.", fontsize=7, color=MUTED)
    fig.tight_layout(rect=(0, 0.02, 1, 0.97))
    fig.savefig(out, dpi=140, facecolor="white")
    plt.close(fig)


def timing(rows, out):
    conds = order(rows)
    fig, axs = plt.subplots(1, 3, figsize=(12, 0.32 * len(conds) + 1.9), sharey=True)
    strip(axs[0], rows, conds, "late_s", "Took minus planned (s)", ref=0.0)
    strip(axs[1], rows, conds, "replans", "Re-plans (not counting improvement tries)")
    strip(axs[2], rows, conds, "fleet_still_extra_s", "All three standing still, beyond the plan (s)", ref=0.0)
    label_rows(axs[0], conds)
    for ax in axs:
        group_bands(ax, conds)
    fig.suptitle("Monday's routine under school conditions: time, re-plans and standstill per run (filled = finished, "
                 "red ring = did not finish)", fontsize=10, color=INK, x=0.01, ha="left")
    fig.text(0.01, 0.005, SOURCE + "; standstill from the simulator's true poses against the first plan, same rule.",
             fontsize=7, color=MUTED)
    fig.tight_layout(rect=(0, 0.02, 1, 0.97))
    fig.savefig(out, dpi=140, facecolor="white")
    plt.close(fig)


def planfit(rows, out):
    Ws = sorted({r["W"] for r in rows})
    Hs = sorted({r["H"] for r in rows})
    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    for r in rows:
        x, y = Ws.index(r["W"]), Hs.index(r["H"])
        if r["ok"]:
            ax.scatter([x], [y], s=900, marker="s", color=BLUE, zorder=2)
            ax.text(x, y, "%.0f s" % r["makespan_s"], ha="center", va="center", color="white", fontsize=8, zorder=3)
        else:
            ax.scatter([x], [y], s=900, marker="s", color=SURF, edgecolors=RED, linewidths=1.8, zorder=2)
            ax.text(x, y, "no plan", ha="center", va="center", color=INK, fontsize=7, zorder=3)
    ax.set_xticks(range(len(Ws)))
    ax.set_xticklabels(["%d cm" % w for w in Ws], fontsize=8)
    ax.set_yticks(range(len(Hs)))
    ax.set_yticklabels(["%d cm" % h for h in Hs], fontsize=8)
    ax.set_xlabel("Table length (x)", fontsize=9)
    ax.set_ylabel("Table depth (y)", fontsize=9)
    ax.set_xlim(-0.6, len(Ws) - 0.4)
    ax.set_ylim(-0.6, len(Hs) - 0.4)
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.set_title("Does Monday's routine plan on this table? Planned finish time, or no plan", fontsize=10, loc="left")
    fig.text(0.01, 0.01, "Source: missions.monday + planner.make_plan (planner only, all four sides open, as before the "
             "warm-up), Monday sweep 2026-10-27", fontsize=7, color=MUTED)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out, dpi=140, facecolor="white")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rows", required=True)
    ap.add_argument("--planfit")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rows = [r for r in json.load(open(a.rows)) if "error" not in r]
    os.makedirs(a.out, exist_ok=True)
    safety(rows, os.path.join(a.out, "monday-sweep-safety-margins.png"))
    timing(rows, os.path.join(a.out, "monday-sweep-time-replans-standstill.png"))
    if a.planfit:
        planfit(json.load(open(a.planfit)), os.path.join(a.out, "monday-sweep-table-size-fit.png"))


if __name__ == "__main__":
    main()
