#!/usr/bin/env python3
"""Summarize home_lab results saved by alvik_console.py.

    python tools/summarize.py logs/20261004-140000
    python tools/summarize.py logs/*          # several sessions together

Prints one table per experiment:
  turn  - accuracy and time per run (method + settings)
  lap   - lap time and steering error per run (speed + gains)
  grid  - run time by mode, and the stop-vs-drive-through difference
Standard library only.
"""
import csv
import glob
import math
import os
import statistics as st
import sys
from collections import defaultdict


def read_rows(folders, prefix):
    rows = []
    for folder in folders:
        for path in sorted(glob.glob(os.path.join(folder, f"{prefix}*.csv"))):
            # turn.csv must not swallow trace.csv etc.; match name exactly
            base = os.path.basename(path)
            if not (base == f"{prefix}.csv" or base.startswith(f"{prefix}_")):
                continue
            if prefix == "grid" and not base.startswith("grid_"):
                continue
            with open(path, newline="") as f:
                for r in csv.DictReader(f):
                    r["_session"] = os.path.basename(os.path.normpath(folder))
                    rows.append(r)
    return rows


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def mean_sd(vals):
    vals = [v for v in vals if not math.isnan(v)]
    if not vals:
        return float("nan"), float("nan"), 0
    sd = st.stdev(vals) if len(vals) > 1 else float("nan")
    return st.mean(vals), sd, len(vals)


# --- Student t quantile without scipy (for 95% confidence intervals) -------
def _betacf(a, b, x):
    qab, qap, qam = a + b, a + 1, a - 1
    c, d = 1.0, 1 - qab * x / qap
    d = 1 / (d if abs(d) > 1e-30 else 1e-30)
    h = d
    for m in range(1, 200):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1 + aa * d
        d = 1 / (d if abs(d) > 1e-30 else 1e-30)
        c = 1 + aa / c if abs(1 + aa / c) > 1e-30 else 1e-30
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1 + aa * d
        d = 1 / (d if abs(d) > 1e-30 else 1e-30)
        c = 1 + aa / c if abs(1 + aa / c) > 1e-30 else 1e-30
        de = d * c
        h *= de
        if abs(de - 1) < 3e-12:
            break
    return h


def _betainc(a, b, x):
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    lbt = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1 - x)
    if x < (a + 1) / (a + b + 2):
        return math.exp(lbt) * _betacf(a, b, x) / a
    return 1 - math.exp(lbt) * _betacf(b, a, 1 - x) / b


def t_cdf(t, df):
    x = df / (df + t * t)
    p = 0.5 * _betainc(df / 2, 0.5, x)
    return 1 - p if t > 0 else p


def t_quantile(p, df):
    lo, hi = 0.0, 1000.0
    for _ in range(200):
        mid = (lo + hi) / 2
        if t_cdf(mid, df) < p:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def welch(a, b):
    """Difference of means a-b with a 95% interval (Welch's t)."""
    ma, sa, na = mean_sd(a)
    mb, sb, nb = mean_sd(b)
    if na < 2 or nb < 2:
        return None
    va, vb = sa * sa / na, sb * sb / nb
    se = math.sqrt(va + vb)
    if se == 0:
        return ma - mb, 0.0, float("inf"), 0
    df = (va + vb) ** 2 / (va * va / (na - 1) + vb * vb / (nb - 1))
    tq = t_quantile(0.975, df)
    t = (ma - mb) / se
    p = 2 * (1 - t_cdf(abs(t), df))
    return ma - mb, tq * se, df, p


def fmt(x, nd=1):
    return "-" if math.isnan(x) else f"{x:.{nd}f}"


def table(headers, rows):
    widths = [max(len(str(h)), *(len(str(r[i])) for r in rows)) for i, h in enumerate(headers)]
    print("  ".join(str(h).rjust(w) for h, w in zip(headers, widths)))
    for r in rows:
        print("  ".join(str(c).rjust(w) for c, w in zip(r, widths)))


def summarize_turn(folders):
    rows = read_rows(folders, "turn")
    if not rows:
        return
    print("\nTURN TEST - per run")
    failed = sum(1 for r in rows if r.get("ok", "1") == "0")
    rows = [r for r in rows if r.get("ok", "1") != "0"]
    groups = defaultdict(list)
    for r in rows:
        groups[(r["_session"], r.get("robot", "-"), r["run"])].append(r)
    out = []
    for (sess, robot, run), rs in groups.items():
        rel = [num(r["err_rel_deg"]) for r in rs]
        odo = [num(r.get("odo_dyaw_deg")) - num(r["cmd_deg"]) for r in rs]
        m_rel, sd_rel, n = mean_sd(rel)
        m_abs_rel = st.mean(abs(v) for v in rel)
        t_done, _, _ = mean_sd([num(r["t_done_ms"]) for r in rs])
        drift = num(rs[-1]["err_abs_deg"])
        r0 = rs[0]
        m_odo, _, _ = mean_sd(odo)
        out.append([sess, robot, run, r0["method"], r0["max_rpm"], r0["kp"], r0["tol_deg"], n,
                    fmt(m_rel, 2), fmt(sd_rel, 2), fmt(m_abs_rel, 2), fmt(m_odo, 2), fmt(t_done, 0), fmt(drift, 2)])
    table(["session", "robot", "run", "method", "max_rpm", "kp", "tol", "n",
           "mean_err", "sd_err", "mean_|err|", "odo_err", "time_ms", "final_drift"], out)
    print("  mean_err: average signed miss per turn by the IMU (bias); sd_err: spread; odo_err: the same\n"
          "  miss judged by wheel odometry instead (a check on the IMU); final_drift: IMU heading error\n"
          "  after the last turn. Library time_ms includes ~300 ms the library itself waits per turn.")
    if failed:
        print(f"  ({failed} turns that timed out are left out)")


def summarize_lap(folders):
    rows = read_rows(folders, "lap")
    if not rows:
        return
    print("\nLAP TEST - per run")
    groups = defaultdict(list)
    for r in rows:
        groups[(r["_session"], r.get("robot", "-"), r["run"])].append(r)
    out = []
    for (sess, robot, run), rs in groups.items():
        lap, sd, n = mean_sd([num(r["lap_ms"]) for r in rs])
        err, _, _ = mean_sd([num(r["mean_abs_err"]) for r in rs])
        r0 = rs[0]
        out.append([sess, robot, run, r0["base_rpm"], r0["kp"], r0["kd"], n,
                    fmt(lap / 1000, 2), fmt(sd / 1000, 2), fmt(err, 3)])
    table(["session", "robot", "run", "base_rpm", "kp", "kd", "laps", "lap_s", "sd_s", "mean_|err|"], out)


def summarize_grid(folders):
    rows = [r for r in read_rows(folders, "grid") if "total_ms" in r]
    if not rows:
        return
    print("\nGRID TEST - completed runs by route and mode")
    # Compare like with like: same route, speed and stop pause.
    by = defaultdict(list)
    for r in rows:
        if r["result"] == "done":
            key = (r.get("robot", "-"), r["route"], r["base_rpm"], r["stop_ms"])
            by[key + (r["mode"],)].append(num(r["total_ms"]) / 1000)
    skipped = sum(1 for r in rows if r["result"] != "done")
    out = []
    for (robot, route, rpm, stop_ms, mode), vals in sorted(by.items()):
        m, sd, n = mean_sd(vals)
        out.append([robot, route, rpm, stop_ms, "stop" if mode == "0" else "drive-through", n, fmt(m, 2), fmt(sd, 2)])
    table(["robot", "route", "rpm", "stop_ms", "mode", "runs", "mean_s", "sd_s"], out)
    if skipped:
        print(f"  ({skipped} unfinished runs left out)")
    for key in sorted({k[:4] for k in by}):
        robot, route, rpm, stop_ms = key
        a, b = by.get(key + ("0",), []), by.get(key + ("1",), [])
        label = f"{robot} {route} at {rpm} rpm"
        res = welch(a, b)
        if res is None:
            print(f"  {label}: need at least 2 finished runs in each mode to compare.")
            continue
        diff, half, df, p = res
        pct = 100 * diff / st.mean(a)
        print(f"  {label}: drive-through saves {diff:.2f} s per run ({pct:.0f}%), "
              f"95% interval {diff - half:.2f} to {diff + half:.2f} s, p = {p:.3g}")
    print("  The interval and p come from Welch's t-test, which compares two averages without assuming\n"
          "  equal spread; an interval that stays above 0 means the saving is unlikely to be luck.")


def main():
    folders = [f for a in sys.argv[1:] for f in glob.glob(a) if os.path.isdir(f)]
    if not folders:
        print(__doc__)
        return 1
    if not any(glob.glob(os.path.join(f, "*.csv")) for f in folders):
        print("No CSV files there. Point at a session folder, e.g. logs/20261004-140000, or logs/*")
        return 1
    summarize_turn(folders)
    summarize_lap(folders)
    summarize_grid(folders)
    return 0


if __name__ == "__main__":
    sys.exit(main())
