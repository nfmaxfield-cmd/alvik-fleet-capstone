"""E0b nudge check: is a 5 cm start roll better than 2 cm?

    python tools/nudge_check.py docs/experiments/E00b-nudge-check

Each run is the start roll alone (forward and back), with tape marks before
and after. Per roll size this prints:

  - how far the roll left the robot off its marks (position and heading),
  - the tape's own heading repeatability: the heading change between the two
    sets of marks (an upper bound: any real turn during the roll is in it too),
  - if heading_check.py has been run with these runs' calibration results
    (data/heading_check.csv), how closely the camera's heading from the roll
    matches the tape, per roll size.

Decision rule (written before the runs, in the README): use 5 cm for T1 if
its camera heading spread is smaller than at 2 cm AND the 5 cm roll leaves
the robot off its marks by no more than 0.2 cm and 0.3 deg on average.
"""
import argparse
import csv
import math
import os
import statistics as st
import sys


def num(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def pose(xf, yf, xr, yr):
    return (xf + xr) / 2, (yf + yr) / 2, math.degrees(math.atan2(yf - yr, xf - xr))


def welch(a, b):
    """Welch's t and approximate two-sided p (normal approximation for p when df is large enough)."""
    if len(a) < 2 or len(b) < 2:
        return None, None
    va, vb = st.variance(a) / len(a), st.variance(b) / len(b)
    if va + vb == 0:
        return None, None
    t = (st.mean(a) - st.mean(b)) / math.sqrt(va + vb)
    df = (va + vb) ** 2 / (va ** 2 / (len(a) - 1) + vb ** 2 / (len(b) - 1))
    try:
        from scipy import stats
        p = 2 * stats.t.sf(abs(t), df)
    except ImportError:
        p = math.erfc(abs(t) / math.sqrt(2))
    return t, p


def summary(vals):
    if not vals:
        return "  -"
    m = st.mean(vals)
    sd = st.stdev(vals) if len(vals) > 1 else float("nan")
    return f"{m:6.2f} (SD {sd:.2f}, n {len(vals)})"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("experiment", help="experiment folder with data/measurements.csv")
    a = ap.parse_args()
    data = os.path.join(a.experiment, "data")
    with open(os.path.join(data, "measurements.csv"), newline="", encoding="utf-8-sig") as f:
        rows = [r for r in csv.DictReader(f)
                if str(r.get("excluded", "")).strip().lower() not in ("1", "y", "yes", "true", "x")]
    by = {}
    out = []
    for r in rows:
        v = [num(r.get(k)) for k in ("xf0_cm", "yf0_cm", "xr0_cm", "yr0_cm", "xf_cm", "yf_cm", "xr_cm", "yr_cm")]
        if None in v:
            continue
        x0, y0, h0 = pose(*v[:4])
        x1, y1, h1 = pose(*v[4:])
        c, s = math.cos(math.radians(h0)), math.sin(math.radians(h0))
        dx, dy = x1 - x0, y1 - y0
        o = dict(run=r.get("run_id", ""), robot=r.get("robot", ""), level=r.get("level", ""),
                 shift_cm=math.hypot(dx, dy), along_cm=c * dx + s * dy, left_cm=-s * dx + c * dy,
                 dheading_deg=wrap180(h1 - h0))
        out.append(o)
        by.setdefault(o["level"], []).append(o)
    if not out:
        sys.exit("No runs with both start and end marks yet.")

    print("How far the roll left the robot off its marks (end marks minus start marks)")
    print(f"{'roll':8s} {'off by, cm':26s} {'along, cm':26s} {'heading change, deg':26s}")
    for lv in sorted(by):
        g = by[lv]
        print(f"{lv:8s} {summary([o['shift_cm'] for o in g]):26s} {summary([o['along_cm'] for o in g]):26s} "
              f"{summary([o['dheading_deg'] for o in g]):26s}")
    levels = sorted(by)
    if len(levels) == 2:
        t, p = welch([o["shift_cm"] for o in by[levels[1]]], [o["shift_cm"] for o in by[levels[0]]])
        if t is not None:
            print(f"Off-by distance, {levels[1]} vs {levels[0]}: Welch t = {t:.2f}, p = {p:.3f}")
    allh = [o["dheading_deg"] for o in out]
    if len(allh) > 1:
        print(f"\nTape heading repeatability (all runs): SD {st.stdev(allh):.2f} deg between the two sets of marks "
              f"(upper bound: includes any real turn during the roll).")

    hc = os.path.join(data, "heading_check.csv")
    cam = {}
    if os.path.isfile(hc):
        with open(hc, newline="", encoding="utf-8-sig") as f:
            for r in csv.DictReader(f):
                d = num(r.get("diff_deg"))
                if d is not None:
                    cam.setdefault(r.get("level", ""), []).append(d)
        print("\nCamera heading (from the roll) minus tape heading, per roll size")
        for lv in sorted(cam):
            print(f"{lv:8s} {summary(cam[lv])}")
    else:
        print(f"\n(No {hc} yet: run tools/heading_check.py on this folder with the session's "
              f"calibration_results.csv to compare camera and tape heading per roll size.)")

    if len(levels) == 2 and all(lv in cam and len(cam[lv]) > 1 for lv in levels):
        small, big = levels   # 'nudge2' sorts before 'nudge5'
        sd_s, sd_b = st.stdev(cam[small]), st.stdev(cam[big])
        off = st.mean([o["shift_cm"] for o in by[big]])
        turn = st.mean([abs(o["dheading_deg"]) for o in by[big]])
        ok = sd_b < sd_s and off <= 0.2 and turn <= 0.3
        print(f"\nDecision rule: camera spread {sd_b:.2f} vs {sd_s:.2f} deg; {big} leaves the robot "
              f"{off:.2f} cm and {turn:.2f} deg off on average -> "
              + (f"use {big[5:]} cm for T1." if ok else f"keep {small[5:]} cm."))
    path = os.path.join(data, "nudge_check.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        for o in out:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in o.items()})
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
