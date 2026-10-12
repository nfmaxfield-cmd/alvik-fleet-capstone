#!/usr/bin/env python3
"""Turn a filled-in tape measurement sheet into the numbers and chart for an
experiment record.

    python tools/tape_results.py docs/experiments/E01-odometry-precision/b-straight
    python tools/tape_results.py docs/experiments/E01-odometry-precision/c-l-path --logs logs/20261015-*

Reads <folder>/data/measurements.csv (made by make_run_order.py) and writes:
  <folder>/data/results.csv   one row per trial: position, heading and errors
  <folder>/chart.svg          every trial as a dot, mean and 95% interval per level
and prints a summary table to paste into the README.

With --logs, each trial's run_id is matched to the robot's own log, and the
robot's odometry estimate is compared with the tape in pairs (paired t-test):
does odometry see the real error?

Position = midpoint of the pointer ruler's two ends. Heading = direction from
the rear end to the front end. x is along the taped x axis, y to the left.
Standard library only.
"""
import argparse
import csv
import glob
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from summarize import mean_sd, t_quantile, t_cdf, welch, fmt, table  # noqa: E402


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def interval(vals):
    """mean, sd, n, half-width of the 95% interval."""
    m, sd, n = mean_sd(vals)
    half = t_quantile(0.975, n - 1) * sd / math.sqrt(n) if n > 1 else float("nan")
    return m, sd, n, half


def paired(a, b):
    """Mean of a-b, 95% half-width, p (paired t-test)."""
    d = [x - y for x, y in zip(a, b)]
    m, sd, n, half = interval(d)
    if n < 2 or sd == 0 or math.isnan(sd):
        return m, half, float("nan"), n
    t = m / (sd / math.sqrt(n))
    return m, half, 2 * (1 - t_cdf(abs(t), n - 1)), n


def level_key(lv):
    v = num(lv)
    return (0, v, "") if v is not None else (1, 0, lv)


# ---------------------------------------------------------------- trials
def load_trials(folder):
    path = os.path.join(folder, "data", "measurements.csv")
    if not os.path.exists(path):
        sys.exit(f"no sheet at {path}. Make one with tools/make_run_order.py")
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            xf, yf, xr, yr = (num(r.get(k)) for k in ("xf_cm", "yf_cm", "xr_cm", "yr_cm"))
            if None in (xf, yf, xr, yr):
                continue                       # not measured yet
            if (r.get("excluded") or "").strip() in ("1", "yes", "y", "x"):
                continue
            t = dict(r)
            t["x"], t["y"] = (xf + xr) / 2, (yf + yr) / 2
            t["heading"] = math.degrees(math.atan2(yf - yr, xf - xr))
            tx, ty, th = (num(r.get(k)) for k in ("target_x_cm", "target_y_cm", "target_heading_deg"))
            t["heading_err"] = wrap180(t["heading"] - th) if th is not None else None
            t["abs_heading_err"] = abs(t["heading_err"]) if th is not None else None
            if tx is not None and ty is not None:
                t["along_err"], t["cross_err"] = t["x"] - tx, t["y"] - ty
                t["pos_err"] = math.hypot(t["along_err"], t["cross_err"])
            else:
                t["along_err"] = t["cross_err"] = t["pos_err"] = None
            out.append(t)
    return out


# ---------------------------------------------------------------- robot logs
def load_odometry(log_globs):
    """run_id -> dict of what the robot itself believed."""
    odo = {}
    folders = [f for g in log_globs for f in glob.glob(g) if os.path.isdir(f)]
    for folder in folders:
        for path in glob.glob(os.path.join(folder, "*.csv")):
            base = os.path.basename(path).lower()
            with open(path, newline="") as f:
                rows = list(csv.DictReader(f))
            for r in rows:
                rid = r.get("run")
                if not rid:
                    continue
                if base.startswith("straight"):
                    cmd, x, y = num(r.get("cmd_cm")), num(r.get("odo_x_cm")), num(r.get("odo_y_cm"))
                    if None not in (cmd, x, y):
                        odo[rid] = dict(along_err=x - cmd, cross_err=y, pos_err=math.hypot(x - cmd, y),
                                        heading_err=num(r.get("imu_dyaw_deg")))
                elif base.startswith("demo_end"):
                    odo[rid] = dict(pos_err=num(r.get("return_err_cm")), heading_err=num(r.get("heading_err_deg")))
                elif base == "turn.csv" or (base.startswith("turn_") and base[5:6].isdigit()):
                    cmd, imu = num(r.get("cmd_deg")), num(r.get("dyaw_deg"))
                    if cmd is not None and imu is not None:
                        odo[rid] = dict(heading_err=imu - cmd)   # the IMU's own view of the turn
    return odo


# ---------------------------------------------------------------- chart
def svg_chart(groups, metric, unit, title, path):
    """Dot plot: every trial, plus mean and 95% interval, per level."""
    levels = sorted(groups, key=level_key)
    vals = [v for lv in levels for v in groups[lv]]
    if not vals:
        return
    stats = {lv: interval(groups[lv]) for lv in levels}
    lo = min(vals + [m - h for m, _, n, h in stats.values() if n > 1] + [0])
    hi = max(vals + [m + h for m, _, n, h in stats.values() if n > 1] + [0])
    pad = (hi - lo) * 0.1 or 1
    lo, hi = lo - pad, hi + pad
    W, H, L, R, T, B = 640, 370, 70, 20, 64, 50
    pw, ph = W - L - R, H - T - B
    Y = lambda v: T + ph * (hi - v) / (hi - lo)
    step = pw / len(levels)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" font-family="sans-serif" font-size="12">',
             f'<rect width="{W}" height="{H}" fill="white"/>',
             f'<text x="{L}" y="24" font-size="15" font-weight="600">{title}</text>']
    # y ticks
    span = hi - lo
    raw = span / 5
    mag = 10 ** math.floor(math.log10(raw))
    tick = min((m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw), default=mag)
    v = math.ceil(lo / tick) * tick
    while v <= hi:
        y = Y(v)
        parts.append(f'<line x1="{L}" x2="{W - R}" y1="{y:.1f}" y2="{y:.1f}" stroke="#ddd"/>')
        parts.append(f'<text x="{L - 8}" y="{y + 4:.1f}" text-anchor="end" fill="#555">{v:g}</text>')
        v += tick
    if lo < 0 < hi:
        parts.append(f'<line x1="{L}" x2="{W - R}" y1="{Y(0):.1f}" y2="{Y(0):.1f}" stroke="#888"/>')
    parts.append(f'<text x="16" y="{T + ph / 2:.1f}" transform="rotate(-90 16 {T + ph / 2:.1f})" '
                 f'text-anchor="middle" fill="#333">{metric} ({unit})</text>')
    for i, lv in enumerate(levels):
        cx = L + step * (i + 0.5)
        for j, v in enumerate(groups[lv]):
            jit = ((j * 37) % 11 - 5) * step * 0.03     # fixed spread so dots don't overlap
            parts.append(f'<circle cx="{cx - step * 0.15 + jit:.1f}" cy="{Y(v):.1f}" r="3.5" fill="#4a7ab8" fill-opacity="0.6"/>')
        m, sd, n, h = stats[lv]
        if n > 1:
            x = cx + step * 0.12
            parts.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{Y(m - h):.1f}" y2="{Y(m + h):.1f}" stroke="#222" stroke-width="1.5"/>')
            for e in (m - h, m + h):
                parts.append(f'<line x1="{x - 6:.1f}" x2="{x + 6:.1f}" y1="{Y(e):.1f}" y2="{Y(e):.1f}" stroke="#222" stroke-width="1.5"/>')
        parts.append(f'<line x1="{cx + step * 0.12 - 10:.1f}" x2="{cx + step * 0.12 + 10:.1f}" y1="{Y(m):.1f}" y2="{Y(m):.1f}" stroke="#c0392b" stroke-width="2.5"/>')
        parts.append(f'<text x="{cx:.1f}" y="{H - B + 20}" text-anchor="middle">{lv}</text>')
        parts.append(f'<text x="{cx:.1f}" y="{H - B + 36}" text-anchor="middle" fill="#777">n = {n}</text>')
    parts.append(f'<text x="{L}" y="44" fill="#777" font-size="11">dots: trials · red line: mean · black bar: 95% interval</text>')
    parts.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(parts))


# ---------------------------------------------------------------- E0
def repeatability(trials):
    by = {}
    for t in trials:
        by.setdefault((t["robot"], t["level"]), []).append(t)
    dx, dy, dh = [], [], []
    for ts in by.values():
        if len(ts) >= 2:
            a, b = ts[0], ts[1]
            dx.append(a["x"] - b["x"]); dy.append(a["y"] - b["y"]); dh.append(wrap180(a["heading"] - b["heading"]))
    if len(dx) < 2:
        print("Need at least 2 spots measured twice.")
        return
    # Each difference holds two measurement errors, so one reading's SD = SD(diff) / sqrt 2.
    sx, sy, sh = (st.stdev(d) / math.sqrt(2) for d in (dx, dy, dh))
    print(f"\nMEASUREMENT CHECK (E0): {len(dx)} spots measured twice")
    table(["", "repeatability SD", "pass if"], [["x (cm)", fmt(sx, 2), "<= 0.5"], ["y (cm)", fmt(sy, 2), "<= 0.5"],
                                                ["heading (deg)", fmt(sh, 2), "<= 0.5"]])
    ok = sx <= 0.5 and sy <= 0.5 and sh <= 0.5
    print("  PASS: the rig is precise enough." if ok else "  FAIL: improve the rig before running other experiments.")


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", help="experiment folder, e.g. docs/experiments/E01-odometry-precision/b-straight")
    ap.add_argument("--logs", nargs="*", default=[], help="console log folders, to compare with odometry")
    args = ap.parse_args()

    trials = load_trials(args.folder)
    if not trials:
        print("No measured trials yet (fill in xf_cm, yf_cm, xr_cm, yr_cm).")
        return 1
    name = os.path.basename(os.path.normpath(args.folder))
    parent = os.path.basename(os.path.dirname(os.path.normpath(args.folder)))
    if parent.startswith("E0") or parent.startswith("E1"):
        name = f"{parent.split('-')[0]} {name}"

    if "E00" in os.path.abspath(args.folder):
        repeatability(trials)
        return 0

    has_pos = any(t["pos_err"] is not None for t in trials)
    metrics = [("pos_err", "position error", "cm"), ("along_err", "along-track error", "cm"),
               ("cross_err", "cross-track error", "cm"), ("heading_err", "heading error", "deg")] if has_pos \
        else [("heading_err", "heading error", "deg"), ("abs_heading_err", "absolute heading error", "deg")]
    main_metric = metrics[0] if has_pos else metrics[1]

    # per-trial results
    odo = load_odometry(args.logs) if args.logs else {}
    res_path = os.path.join(args.folder, "data", "results.csv")
    cols = ["order", "robot", "level", "run_id", "x", "y", "heading"] + [m for m, _, _ in metrics]
    if odo:
        cols += ["odo_pos_err", "odo_along_err", "odo_heading_err"]
    with open(res_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for t in trials:
            o = odo.get(t.get("run_id", "").strip(), {})
            row = [t["order"], t["robot"], t["level"], t.get("run_id", "")] + \
                  [f"{t[k]:.2f}" if t.get(k) is not None else "" for k in ["x", "y", "heading"] + [m for m, _, _ in metrics]]
            if odo:
                row += [f"{o[k]:.2f}" if o.get(k) is not None else "" for k in ("pos_err", "along_err", "heading_err")]
            w.writerow(row)

    # summary per level (robots pooled; per robot when there are several)
    robots = sorted({t["robot"] for t in trials})
    print(f"\n{name}: {len(trials)} measured trials, robots: {', '.join(robots)}")
    for key, label, unit in metrics:
        rows = []
        groups = {}
        for t in trials:
            if t.get(key) is not None:
                groups.setdefault(t["level"], []).append(t[key])
        for lv in sorted(groups, key=level_key):
            m, sd, n, h = interval(groups[lv])
            rows.append([lv, n, fmt(m, 2), fmt(sd, 2), f"{fmt(m - h, 2)} to {fmt(m + h, 2)}"])
        print(f"\n{label} ({unit})")
        table(["level", "n", "mean", "sd", "95% interval"], rows)
    if len(robots) > 1:
        key = main_metric[0]
        rows = []
        for rb in robots:
            for lv in sorted({t["level"] for t in trials}, key=level_key):
                v = [t[key] for t in trials if t["robot"] == rb and t["level"] == lv and t.get(key) is not None]
                m, sd, n, h = interval(v)
                rows.append([rb, lv, n, fmt(m, 2), fmt(sd, 2)])
        print(f"\n{main_metric[1]} by robot ({main_metric[2]})")
        table(["robot", "level", "n", "mean", "sd"], rows)

    # two levels: Welch's test on the main metric
    key, label, unit = main_metric
    groups = {}
    for t in trials:
        if t.get(key) is not None:
            groups.setdefault(t["level"], []).append(t[key])
    if len(groups) == 2:
        a, b = sorted(groups, key=level_key)
        r = welch(groups[a], groups[b])
        if r:
            d, half, df, p = r
            print(f"\n{label}: {a} minus {b} = {d:.2f} {unit}, 95% interval {d - half:.2f} to {d + half:.2f}, "
                  f"p = {p:.3g} (Welch's t-test)")
    elif len(groups) > 2 and has_pos:
        # error growth with distance: least-squares slope of error vs level
        pts = [(num(t["level"]), t[key]) for t in trials if num(t["level"]) is not None and t.get(key) is not None]
        if len(pts) > 2:
            xs, ys = zip(*pts)
            mx, my = st.mean(xs), st.mean(ys)
            sxx = sum((x - mx) ** 2 for x in xs)
            slope = sum((x - mx) * (y - my) for x, y in pts) / sxx
            resid = [y - (my + slope * (x - mx)) for x, y in pts]
            se = math.sqrt(sum(e * e for e in resid) / (len(pts) - 2) / sxx)
            h = t_quantile(0.975, len(pts) - 2) * se
            print(f"\n{label} grows {slope * 100:.2f} cm per metre driven "
                  f"(95% interval {(slope - h) * 100:.2f} to {(slope + h) * 100:.2f})")

    # tape vs the robot's own estimate
    if odo:
        okey = "pos_err" if has_pos else "heading_err"
        tkey = "pos_err" if has_pos else "heading_err"
        pairs = [(t[tkey], odo[t["run_id"].strip()][okey]) for t in trials
                 if t.get("run_id", "").strip() in odo and odo[t["run_id"].strip()].get(okey) is not None
                 and t.get(tkey) is not None]
        if len(pairs) >= 2:
            tape, rob = zip(*pairs)
            m, half, p, n = paired(tape, rob)
            what = "position error" if has_pos else "heading error"
            print(f"\nTape vs robot's own estimate, {n} paired trials: tape {what} minus robot's = {m:.2f}, "
                  f"95% interval {m - half:.2f} to {m + half:.2f}, p = {p:.3g} (paired t-test)")
        else:
            print("\n(no trials matched to the logs: is run_id filled in?)")

    chart_groups = {}
    for t in trials:
        if t.get(key) is not None:
            chart_groups.setdefault(t["level"], []).append(t[key])
    chart = os.path.join(args.folder, "chart.svg")
    svg_chart(chart_groups, label, unit, f"{name}: {label} by level (tape)", chart)
    print(f"\nwrote {os.path.relpath(res_path)} and {os.path.relpath(chart)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
