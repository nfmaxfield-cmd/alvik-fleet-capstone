"""Tables and charts for docs/build-reports/e02-oct25-real-fusion.md, from real_fusion.py's and live_restart_doe.py's
CSVs (out/). Logs only. Usage: python3 real_fusion_report.py  (seconds)

Headline filter for gaps: runs whose start heading checked ok, gap ends 15 cm or more from both blocked boards
(near a board the camera's outline merges with the board and jumps several cm: see the report)."""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
DOCS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(HERE))), "docs", "build-reports")
NEAR_BOARD_CM = 15.0     # chosen: the camera jumps seen at the boards started 10-15 cm out
SOURCES = (("fused", "wheels + gyro + camera (fused)"), ("wheels_gyro", "wheels + gyro (fusion, no camera)"),
           ("own_pose", "robot's own pose"), ("cam_cv", "camera alone, moving on at its last speed"),
           ("cam_hold", "camera alone, last position held"))
COLORS = {"fused": "#2a78d6", "wheels_gyro": "#eb6834", "own_pose": "#1baf7a", "cam_cv": "#eda100",
          "cam_hold": "#e87ba4"}
MARKERS = {"fused": "o", "wheels_gyro": "s", "own_pose": "^", "cam_cv": "D", "cam_hold": "v"}


def rows(fname):
    p = os.path.join(OUT, fname)
    if not os.path.exists(p):
        return []
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def fl(r, k):
    v = r.get(k, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def away(r, key="board_cm"):
    b = fl(r, key)
    return b is None or b >= NEAR_BOARD_CM


def q(v, p):
    return float(np.percentile(v, p)) if len(v) else float("nan")


def gap_table(gaps, pr, sel_fn, title, Ls=None):
    Ls = Ls or sorted({int(g["L"]) for g in gaps})
    pr("\n" + title)
    pr("| gap s | gaps | " + " | ".join(lbl for _, lbl in SOURCES) + " | fused beat the best single source |")
    pr("|---|---|" + "---|" * len(SOURCES) + "---|")
    out = {}
    for L in Ls:
        sel = [g for g in gaps if int(g["L"]) == L and sel_fn(g) and fl(g, "fused") is not None
               and fl(g, "wheels_gyro") is not None]
        if not sel:
            continue
        cells = []
        for k, _ in SOURCES:
            v = [fl(g, k) for g in sel if fl(g, k) is not None]
            out.setdefault(k, {})[L] = (np.median(v) if v else np.nan, q(v, 25), q(v, 75), q(v, 90), len(v))
            cells.append("%.1f (%.1f)" % (np.median(v), q(v, 90)) if v else "-")
        beat = [fl(g, "fused") < min(fl(g, k) for k in ("wheels_gyro", "cam_cv", "cam_hold", "own_pose")
                                     if fl(g, k) is not None) for g in sel]
        pr("| %d | %d | %s | %.0f%% |" % (L, len(sel), " | ".join(cells), 100 * np.mean(beat)))
    pr("<sub>Each cell: median (90th percentile) error at the gap's end against the camera, cm.</sub>")
    return out


def chart_gaps(stats, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8.2, 4.6), dpi=130)
    for k, lbl in SOURCES:
        if k not in stats:
            continue
        Ls = sorted(stats[k])
        med = [stats[k][L][0] for L in Ls]
        lo = [stats[k][L][1] for L in Ls]
        hi = [stats[k][L][2] for L in Ls]
        ax.fill_between(Ls, lo, hi, color=COLORS[k], alpha=0.12, linewidth=0)
        ax.plot(Ls, med, color=COLORS[k], lw=2, marker=MARKERS[k], ms=7, label=lbl)
        ax.annotate(lbl.split(" (")[0].replace(", last position held", ", held").replace(
            ", moving on at its last speed", ", moving on"), (Ls[-1], med[-1]), xytext=(6, 0),
            textcoords="offset points", va="center", fontsize=8, color="#52514e")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xticks([2, 5, 10, 20, 40])
    ax.set_xticklabels(["2", "5", "10", "20", "40"])
    ax.set_xlabel("camera gap length, s (camera withheld from the fusion)")
    ax.set_ylabel("error at the gap's end vs camera, cm")
    ax.set_title("Position error after a camera gap, real E02 sessions (finds + routes)", fontsize=10)
    ax.grid(True, which="major", color="#e6e5e1", lw=0.6)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.legend(fontsize=8, frameon=False, loc="upper left")
    ax.set_xlim(1.7, 90)
    fig.text(0.01, 0.01, "Data: track.csv (camera) and robot-logs RAW of 6 real sessions, 2026-10-22 to 10-24; "
             "line = median, band = middle half; ok-heading runs, gap ends >= 15 cm from the boards.",
             fontsize=7, color="#52514e")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def chart_routes(routes, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    conds = ["U", "C", "K", "KC"]
    srcs = (("own_end_abs", "robot's own pose", "own_pose"), ("A_end_abs", "wheels + gyro (fusion, no camera)",
                                                              "wheels_gyro"),
            ("Bc_end", "fused, camera for the first 10 s only", "fused"))
    fig, ax = plt.subplots(figsize=(8.2, 4.4), dpi=130)
    rng = np.random.default_rng(1)
    for ci, c in enumerate(conds):
        for si, (k, lbl, col) in enumerate(srcs):
            v = [fl(r, k) for r in routes if r["cond"] == c and r["hdg"] == "ok" and fl(r, k) is not None]
            if not v:
                continue
            x = ci + (si - 1) * 0.25 + rng.uniform(-0.06, 0.06, len(v))
            ax.scatter(x, v, s=14, color=COLORS[col], alpha=0.55, marker=MARKERS[col], linewidths=0,
                       label=lbl if ci == 0 else None)
            ax.plot([ci + (si - 1) * 0.25 - 0.1, ci + (si - 1) * 0.25 + 0.1], [np.median(v)] * 2, color="#0b0b0b",
                    lw=2)
    ax.set_xticks(range(len(conds)))
    ax.set_xticklabels(["U (uncalibrated)", "C (calibrated)", "K (laptop steers)", "KC (camera fixes)"])
    ax.set_yscale("log")
    ax.set_ylabel("end-of-route position error vs camera, cm")
    ax.set_xlabel("E02 condition")
    ax.set_title("Where each estimate thought the robot ended a route, against the camera", fontsize=10)
    ax.grid(True, axis="y", color="#e6e5e1", lw=0.6)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.legend(fontsize=8, frameon=False, loc="upper right")
    fig.text(0.01, 0.01, "Data: done E02 routes (e02.csv, runs.csv), RAW and track.csv, ok-heading routes; dot = one "
             "route, bar = median. All start from the robot's own start belief.", fontsize=7, color="#52514e")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def chart_lab(lab, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    Ts = ["15", "30", "60", "120", "240", "never"]
    lbl = {"today": "as today", "position": "per-robot filter, starts at the camera",
           "camera": "... + heading from the camera's travel", "floor": "... + heading from the runner's floor pose"}
    col = {"today": "#e87ba4", "position": "#eda100", "camera": "#2a78d6", "floor": "#1baf7a"}
    mk = {"today": "v", "position": "D", "camera": "o", "floor": "^"}
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 4.0), dpi=130)
    for H in ("today", "position", "camera", "floor"):
        rr = [pooled(lab, H, T) for T in Ts]
        x = range(len(Ts))
        axs[0].plot(x, [r[0] for r in rr], color=col[H], marker=mk[H], lw=2, ms=7, label=lbl[H])
        axs[1].plot(x, [100 * r[3] for r in rr], color=col[H], marker=mk[H], lw=2, ms=7, label=lbl[H])
    axs[0].set_ylabel("restarts per hour of lab time")
    axs[1].set_ylabel("time the lab was > 5 cm from the camera, %")
    for a in axs:
        a.set_xticks(range(len(Ts)))
        a.set_xticklabels(Ts)
        a.set_xlabel("restart threshold (squared distance)")
        a.grid(True, color="#e6e5e1", lw=0.6)
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
    axs[0].set_yscale("symlog", linthresh=10)
    axs[0].legend(fontsize=7.5, frameon=False)
    fig.suptitle("Live lab replayed on every real session: restarts and error by threshold", fontsize=10)
    fig.text(0.01, 0.01, "Data: robot-logs RAW (arrival times) and every track.csv camera sample, all real sessions; "
             "replayed through tools/experiment/e02_live.py's filter.", fontsize=7, color="#52514e")
    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    fig.savefig(path)
    plt.close(fig)


def pooled(lab, H, T):
    sel = [r for r in lab if r["start"] == H and r["T"] == T]
    h = sum(float(r["lab_h"]) for r in sel)
    n = sum(int(r["restarts"]) for r in sel)
    sel_g = [r for r in sel if float(r["n_gaps"]) > 0]
    gw = np.array([float(r["n_gaps"]) for r in sel_g])
    gmed = float(np.average([float(r["gap_err_med"]) for r in sel_g], weights=gw)) if gw.sum() else float("nan")
    sel_c = [r for r in sel if float(r["n_cam"]) > 0 and r["err_p95"] not in ("nan", "")]
    wc = np.array([float(r["n_cam"]) for r in sel_c])
    p95 = float(np.average([float(r["err_p95"]) for r in sel_c], weights=wc)) if wc.sum() else float("nan")
    dv = sum(float(r["diverged_share"] or 0) * float(r["lab_h"]) for r in sel) / h if h else float("nan")
    g90 = float(np.average([float(r["gap_err_p90"]) for r in sel_g], weights=gw)) if gw.sum() else float("nan")
    return n / h if h else float("nan"), gmed, p95, dv, n, g90


def main():
    lines = []
    pr = lambda s="": (print(s), lines.append(s))   # noqa: E731
    gaps, runs_, routes = rows("real_fusion_gaps.csv"), rows("real_fusion_runs.csv"), rows("real_fusion_routes.csv")
    pr("Runs scored: %d (%d routes, %d finds); sessions: %s" % (
        len(runs_), len(routes), len(runs_) - len(routes), ", ".join(sorted({r["session"] for r in runs_}))))
    hd = {}
    for r in runs_:
        hd[(r["kind"], r["hdg"])] = hd.get((r["kind"], r["hdg"]), 0) + 1
    pr("Start heading check: " + ", ".join("%s %s %d" % (k[0], k[1], v) for k, v in sorted(hd.items())))
    wrong = [r for r in runs_ if r["hdg"] == "wrong"]
    for r in wrong:
        pr("  wrong start heading: %s %s %s %s %s err %s deg" % (r["session"], r["robot"], r["run"], r["kind"],
                                                                 r["cond"], r["hdg_err0"]))
    st = gap_table(gaps, pr, lambda g: g["hdg"] == "ok" and away(g),
                   "### Gaps, all driving runs (ok heading, gap end >= 15 cm from the boards)")
    gap_table(gaps, pr, lambda g: g["hdg"] == "ok" and away(g) and g["kind"] == "route",
              "### Gaps, routes only")
    gap_table(gaps, pr, lambda g: g["hdg"] == "ok" and away(g) and g["kind"] == "find", "### Gaps, finds only")
    gap_table(gaps, pr, lambda g: g["hdg"] == "ok" and not away(g), "### Gaps ending NEAR a board (< 15 cm)")
    gap_table(gaps, pr, lambda g: g["hdg"] != "ok", "### Gaps in runs whose start heading was wrong or unmeasured")
    for robot in ("Left", "Center", "Right"):
        gap_table(gaps, pr, lambda g, rb=robot: g["hdg"] == "ok" and away(g) and g["robot"] == rb,
                  "### %s" % robot, Ls=[5, 10, 20])
    # fused following the camera
    v = [fl(r, "bf_res_med") for r in runs_ if fl(r, "bf_res_med") is not None]
    v95 = [fl(r, "bf_res_p95") for r in runs_ if fl(r, "bf_res_p95") is not None]
    pr("\nFused with the camera always on, distance from the camera: median of run medians %.2f cm, median of run "
       "95th percentiles %.2f cm" % (np.median(v), np.median(v95)))
    # find-end board reference
    pr("\n### Find ends: each estimate against the find's own board-touch pose")
    pr("| estimate | finds | median cm | 90th pct cm |")
    pr("|---|---|---|---|")
    for k, lbl in (("find_vs_cam", "camera"), ("find_vs_fused_seen", "fused, camera on"),
                   ("find_vs_Bc", "fused, camera for the first 10 s only"), ("find_vs_A", "wheels + gyro only")):
        v = [fl(r, k) for r in runs_ if r["hdg"] == "ok" and fl(r, k) is not None]
        if v:
            pr("| %s | %d | %.2f | %.2f |" % (lbl, len(v), np.median(v), q(v, 90)))
    # routes
    pr("\n### Routes: end error against the camera (cm, median; ok-heading routes)")
    pr("| cond | robot | n | robot's own pose (firmware) | own, start aligned | wheels + gyro (fusion) | "
       "wheels + gyro, start aligned | fused, camera first 10 s | runner end_err (ended from start) | "
       "runner robot_thought_end |")
    pr("|---|---|---|---|---|---|---|---|---|---|")
    for c in ("U", "C", "K", "KC"):
        for robot in ("Left", "Center", "Right", "all"):
            sel = [r for r in routes if r["cond"] == c and r["hdg"] == "ok" and (robot == "all" or r["robot"] == robot)]
            if not sel:
                continue

            def m(k):
                v = [fl(r, k) for r in sel if fl(r, k) is not None]
                return "%.1f" % np.median(v) if v else "-"
            pr("| %s | %s | %d | %s | %s | %s | %s | %s | %s | %s |" % (
                c, robot, len(sel), m("own_end_abs"), m("own_end_aligned"), m("A_end_abs"), m("A_end_aligned"),
                m("Bc_end"), m("runner_end_err"), m("runner_thought_end")))
    # drift
    pr("\n### Drift of wheels + gyro alone (start aligned), per robot")
    pr("| robot | kind | runs | end error median cm | per minute median | per metre driven median |")
    pr("|---|---|---|---|---|---|")
    for robot in ("Left", "Center", "Right"):
        for kind in ("route", "find"):
            sel = [r for r in runs_ if r["robot"] == robot and r["kind"] == kind and r["hdg"] == "ok"
                   and fl(r, "A_end_aligned") is not None and fl(r, "moved_cm")]
            if not sel:
                continue
            e = [fl(r, "A_end_aligned") for r in sel]
            pm = [fl(r, "A_end_aligned") / (fl(r, "dur_s") / 60) for r in sel]
            pm_ = [fl(r, "A_end_aligned") / (fl(r, "moved_cm") / 100) for r in sel if fl(r, "moved_cm") > 20]
            pr("| %s | %s | %d | %.1f | %.1f | %.1f |" % (robot, kind, len(sel), np.median(e), np.median(pm),
                                                          np.median(pm_) if pm_ else float("nan")))
    for k in ("own_end_aligned",):
        pr("\n### Drift of the robot's own pose (start aligned), routes")
        pr("| robot | cond | routes | end error median cm | per metre driven median |")
        pr("|---|---|---|---|---|")
        for robot in ("Left", "Center", "Right"):
            for c in ("U", "C", "K"):
                sel = [r for r in routes if r["robot"] == robot and r["cond"] == c and r["hdg"] == "ok"
                       and fl(r, k) is not None]
                if sel:
                    e = [fl(r, k) for r in sel]
                    pm = [fl(r, k) / (fl(r, "moved_cm") / 100) for r in sel if fl(r, "moved_cm") > 20]
                    pr("| %s | %s | %d | %.1f | %.1f |" % (robot, c, len(sel), np.median(e), np.median(pm)))
    # route blind after 10 s: each source from the camera's view at 10 s
    pr("\n### Routes driven blind after the first 10 s: error at the end, each source anchored at the camera's "
       "position at 10 s (cm, median / 90th pct; ok heading)")
    for k, lbl in (("Bc_end", "fused (its own state at 10 s, then blind)"), ("Bc_A", "wheels + gyro"),
                   ("Bc_own", "robot's own pose"), ("Bc_cam_hold", "camera held at 10 s")):
        v = [fl(r, k) for r in routes if r["hdg"] == "ok" and fl(r, k) is not None and away(r, "end_board_cm")]
        if v:
            pr("- %s: %.1f / %.1f (n %d)" % (lbl, np.median(v), q(v, 90), len(v)))
    # live lab
    lab = rows("live_restart_doe.csv") or (rows("live_restart_doe_a.csv") + rows("live_restart_doe_b.csv"))
    if lab:
        pr("\n### Live lab DOE (pooled over sessions)")
        pr("| heading | T | restarts | per hour | prediction error p95 cm | time diverged (> 5 cm) | "
           "after a 2 s gap: median / 90th pct cm |")
        pr("|---|---|---|---|---|---|---|")
        for H in ("today", "position", "camera", "floor"):
            for T in ("15", "30", "60", "120", "240", "never"):
                ph, gm, p95, dv, n, g90 = pooled(lab, H, T)
                gtxt = "%.2f / %.2f" % (gm, g90) if gm == gm else "-"
                pr("| %s | %s | %d | %.0f | %.2f | %.1f%% | %s |" % (H, T, n, ph, p95, 100 * dv, gtxt))
        chart_lab(lab, os.path.join(DOCS, "e02-oct25-real-fusion-lab.png"))
    if st:
        chart_gaps(st, os.path.join(DOCS, "e02-oct25-real-fusion-gaps.png"))
    if routes:
        chart_routes(routes, os.path.join(DOCS, "e02-oct25-real-fusion-routes.png"))
    with open(os.path.join(OUT, "real_fusion_report.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
