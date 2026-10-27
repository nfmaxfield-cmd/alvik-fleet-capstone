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
    """(group, condition) in first-seen order, conditions kept together under their group."""
    groups, seen, gorder = [], [], []
    for r in rows:
        if r["group"] not in gorder:
            gorder.append(r["group"])
        if r["condition"] not in seen:
            seen.append(r["condition"])
            groups.append((r["group"], r["condition"]))
    return sorted(groups, key=lambda gc: gorder.index(gc[0]))


def strip(ax, rows, conds, key, xlabel, ref=None, ref_label=None, clip=None):
    ys = {c: i for i, (_, c) in enumerate(conds)}
    for r in rows:
        v = r.get(key)
        if v is None or v != v or v in (float("inf"), float("-inf")):
            continue
        if clip is not None:
            v = max(clip[0], min(clip[1], v))
        fin = r.get("finished")
        ax.scatter([v], [ys[r["condition"]]], s=46, zorder=3, color=BLUE if fin else SURF,
                   edgecolors=BLUE if fin else RED, linewidths=1.8 if not fin else 0.8)
    if ref is not None:
        ax.axvline(ref, color=ORANGE, lw=1.5, ls="--", zorder=2)
        if ref_label:
            ax.text(ref, -0.6, " " + ref_label, color=MUTED, fontsize=8, va="bottom")
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
    ax.set_ylim(len(conds) - 0.5, -1.1)


def safety(rows, out):
    conds = order(rows)
    fig, axs = plt.subplots(1, 3, figsize=(12, 0.32 * len(conds) + 1.9), sharey=True)
    strip(axs[0], rows, conds, "closest_cm", "Closest two robot bodies came (cm)", ref=8.0, ref_label="plan's floor 8 cm")
    strip(axs[1], rows, conds, "edge_cm", "Closest a body came to an open edge\n(cm; below 0 = over it)", ref=0.0,
          ref_label="edge")
    strip(axs[2], rows, conds, "brick_cm", "Closest a body came to the brick\n(cm; 0 = touching)", ref=0.0, clip=(-1, 30))
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
    strip(axs[1], rows, conds, "replans", "Re-plans\n(not counting improvement tries)")
    strip(axs[2], rows, conds, "fleet_still_extra_s", "All three standing still at once,\nbeyond the plan (s)", ref=0.0)
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


ROBOT = {"Left": "#2952CC", "Center": "#C2410C", "Right": "#7A3EC8"}     # the robots' own colours (mission-control brief)


def tracks(folder, out, title, t_max=None):
    """Where each robot really drove (simulator truth) in one dry run, with the table, the brick and every obstacle the
    run_plan re-plans added (camera reports, edge-guard strips), and where each robot stood at the end."""
    import csv
    meta = json.load(open(os.path.join(folder, "sweep_meta.json")))
    mm = json.load(open(os.path.join(meta["prep"], "mission.json")))
    f = mm["floor"]
    pts = {}
    t0 = None
    for r in csv.DictReader(open(os.path.join(folder, "sim", "truth.csv"))):
        t = float(r["t"])
        t0 = t if t0 is None else t0
        if t_max is None or t - t0 <= t_max:
            pts.setdefault(r["robot"], []).append((float(r["x_cm"]), float(r["y_cm"])))
    obs = []
    for ln in open(os.path.join(folder, "coordinator.jsonl")):
        e = json.loads(ln)
        if e.get("event") == "replan" and e.get("obstacle"):
            o = e["obstacle"]
            if o.get("points"):
                obs.append((o.get("source"), o["points"]))
    rep = os.path.join(folder, "sim", "sim_report.json")
    brick = None
    if os.path.exists(rep):
        for e in json.load(open(rep)).get("events", []):
            if e.get("kind") == "brick":
                brick = e.get("points")
    W, H = f["x_max"] - f["x_min"], f["y_max"] - f["y_min"]
    fig, ax = plt.subplots(figsize=(min(11, 2.0 + W / 17.0), 1.6 + H / 17.0))
    ax.add_patch(plt.Rectangle((f["x_min"], f["y_min"]), W, H, fill=True, color="#f1f0ec", zorder=0))
    ax.add_patch(plt.Rectangle((f["x_min"], f["y_min"]), W, H, fill=False, ec=MUTED, lw=1.2, zorder=1))
    seen = set()
    for src, poly in obs:
        lab = {"camera": "camera-reported object", "edge_guard": "edge-guard no-go strip",
               "distance_sensor": "object the front sensor found"}.get(src, src)
        ax.add_patch(plt.Polygon(poly, closed=True, fc="#e3494833" if src == "camera" else "#eda10033",
                                 ec=RED if src == "camera" else "#c98500", lw=1.0, zorder=2,
                                 label=None if lab in seen else lab))
        seen.add(lab)
    if brick:
        ax.add_patch(plt.Polygon(brick, closed=True, fc="#52514e", ec=INK, lw=1.0, zorder=3, label="the brick (truth)"))
    for n, p in pts.items():
        xs, ys = zip(*p)
        ax.plot(xs, ys, color=ROBOT.get(n, BLUE), lw=1.0, alpha=0.85, zorder=4, label=n)
        ax.scatter([xs[-1]], [ys[-1]], s=60, color=ROBOT.get(n, BLUE), edgecolors="white", linewidths=1.5, zorder=5)
    ax.set_xlim(f["x_min"] - 8, f["x_max"] + 8)
    ax.set_ylim(f["y_min"] - 8, f["y_max"] + 8)
    ax.set_aspect("equal")
    ax.set_xlabel("x (cm)", fontsize=8)
    ax.set_ylabel("y (cm), audience at the bottom", fontsize=8)
    ax.tick_params(labelsize=7, colors=MUTED)
    ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False)
    ax.set_title(title, fontsize=9, loc="left")
    fig.text(0.01, 0.01, "Source: simulator truth (fleet_sim truth.csv) and run_plan's re-plan records, run %s; "
             "dots = where each robot ended" % os.path.basename(folder), fontsize=6.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out, dpi=140, facecolor="white")
    plt.close(fig)


def table(rows):
    """One markdown row per condition: runs finished, worst margins, time against plan, re-plans, lanes."""
    out = ["| Group | Condition | Finished | Closest robots, cm (worst) | Closest to open edge, cm (worst) | "
           "Took / planned, s | Re-plans | Longest re-plan, s | Lanes mowed (worst) | All 3 still beyond plan, s |",
           "|---|---|---|---|---|---|---|---|---|---|"]
    for g, c in order(rows):
        rs = [r for r in rows if r["condition"] == c]
        fin = sum(1 for r in rs if r["finished"])

        def worst(k, f=min):
            v = [r[k] for r in rs if r.get(k) is not None]
            return f(v) if v else None
        tk = "; ".join("%.0f / %.0f" % (r["took_s"] or 0, r["planned_s"] or 0) for r in rs)
        rp = ", ".join(str(r["replans"]) for r in rs)
        lw = min(rs, key=lambda r: r["lanes_done"])["lanes"]
        fs = ", ".join("%+.0f" % r["fleet_still_extra_s"] if r.get("fleet_still_extra_s") is not None else "-" for r in rs)
        cl, ed = worst("closest_cm"), worst("edge_cm")
        out.append("| %s | %s | %d of %d | %s | %s | %s | %s | %s | %s | %s |" % (
            g, c, fin, len(rs), "-" if cl is None else "%.1f" % cl, "-" if ed is None else "%.1f" % ed, tk, rp,
            "%.1f" % worst("replan_cpu_max_s", max), lw, fs))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rows")
    ap.add_argument("--planfit")
    ap.add_argument("--out", required=True)
    ap.add_argument("--table", action="store_true", help="print the markdown table of the rows instead")
    ap.add_argument("--track", nargs=3, action="append", default=[], metavar=("RUN_FOLDER", "PNG", "TITLE"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for folder, png, title in a.track:
        tracks(folder, os.path.join(a.out, png), title)
    if not a.rows:
        return
    if a.table:
        print(table([r for r in json.load(open(a.rows)) if "error" not in r]))
        return
    rows = [r for r in json.load(open(a.rows)) if "error" not in r]
    os.makedirs(a.out, exist_ok=True)
    safety(rows, os.path.join(a.out, "monday-sweep-safety-margins.png"))
    timing(rows, os.path.join(a.out, "monday-sweep-time-replans-standstill.png"))
    if a.planfit:
        planfit(json.load(open(a.planfit)), os.path.join(a.out, "monday-sweep-table-size-fit.png"))


if __name__ == "__main__":
    main()
