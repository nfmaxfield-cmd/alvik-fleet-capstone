"""Check the tape's start headings against the camera's.

Every turn, demo and straight run starts with a 2 cm roll forward and back
(the start nudge). The camera sees which way the robot rolled
(tools/blink/calib.py writes it to calibration_results.csv). Comparing that
with the heading from the tape start marks shows how precisely the tape
method gives heading, which T0's single pass couldn't show.

    python tools/heading_check.py docs/experiments/E01-baseline-dataset \\
        --calib-results <recording>/calibration_results.csv

Prints, per run, tape heading, camera heading and the difference, then the
mean difference (a constant offset = the camera calibration's rotation, not
tape error) and the spread of the differences (tape + camera heading error
together; the tape's part is at most this). Writes heading_check.csv beside
the sheet.
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
        return None if math.isnan(v) else v
    except (TypeError, ValueError):
        return None


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def tape_heading(r, suffix="0"):
    xf, yf, xr, yr = (num(r.get(f"{k}{suffix}_cm")) for k in ("xf", "yf", "xr", "yr"))
    if None in (xf, yf, xr, yr):
        return None
    return math.degrees(math.atan2(yf - yr, xf - xr))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("experiment", help="experiment folder with data/measurements.csv")
    ap.add_argument("--calib-results", nargs="+", required=True,
                    help="calibration_results.csv file(s) from tools/blink/calib.py")
    a = ap.parse_args()

    sheet = os.path.join(a.experiment, "data", "measurements.csv")
    with open(sheet, newline="", encoding="utf-8-sig") as f:
        rows = {r["run_id"]: r for r in csv.DictReader(f) if (r.get("run_id") or "").strip()}
    cam = {}
    for path in a.calib_results:
        with open(path, newline="", encoding="utf-8-sig") as f:
            for r in csv.DictReader(f):
                if r.get("event") == "nudge" and num(r.get("heading_floor_deg")) is not None:
                    cam[r["run"]] = r

    out = []
    for run, r in rows.items():
        th = tape_heading(r)
        c = cam.get(run)
        if th is None or c is None:
            continue
        ch = num(c["heading_floor_deg"])
        out.append(dict(run=run, robot=r.get("robot", ""), level=r.get("level", ""), tape_heading_deg=round(th, 2),
                        camera_heading_deg=round(ch, 2), diff_deg=round(wrap180(ch - th), 2),
                        identity=c.get("identity", "")))
    if not out:
        sys.exit("No runs with both tape start marks and a camera nudge heading. Check run ids match.")
    print(f"{'run':10s} {'robot':7s} {'route':12s} {'tape':>8s} {'camera':>8s} {'diff':>7s}")
    for o in out:
        print(f"{o['run']:10s} {o['robot']:7s} {o['level']:12s} {o['tape_heading_deg']:8.2f} "
              f"{o['camera_heading_deg']:8.2f} {o['diff_deg']:7.2f}")
    d = [o["diff_deg"] for o in out]
    mean = st.mean(d)
    sd = st.stdev(d) if len(d) > 1 else float("nan")
    print(f"\n{len(d)} runs: mean difference {mean:+.2f} deg (camera calibration's rotation), "
          f"spread (SD) {sd:.2f} deg")
    print(f"Tape heading error is at most {sd:.2f} deg per run (the camera's own error is in this too).")
    print("SD: standard deviation, the typical distance of a value from the mean.")
    path = os.path.join(a.experiment, "data", "heading_check.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
