"""Check the tape's start headings against the camera's.

Every turn, demo and straight run starts with a 2 cm roll forward and back
(the start nudge). The camera sees which way the robot rolled
(tools/blink/calib.py writes it to calibration_results.csv). Comparing that
with the heading from the tape start marks shows how precisely the tape
method gives heading, which T0's single pass couldn't show.

    python tools/heading_check.py docs/experiments/E01-baseline-dataset \\
        --calib-results <recording>/calibration_results.csv

Prints, per run, tape heading, camera heading and the difference, then the
mean difference and the spread of the differences. A constant offset is
either the camera calibration's rotation or a habit in where the marks go
(the two look the same here). The spread is tape and camera heading error
together.

The camera's own share: the 2 cm nudge is only ~7 px on camera, so a 1 px
error in the shift is ~8 deg. Where the robot detector also gave a heading
(det_heading_floor_deg), the two camera methods are compared; half the
variance of their difference estimates one camera method's variance
(assumes the two errors are independent and similar), and that is taken out
of the spread to estimate the tape's own heading error. Without detector
headings only the upper bound is given.

Skipped: excluded runs, and nudges whose blink code disagreed with the run's
robot. Writes heading_check.csv beside the sheet.
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
        rows = {r["run_id"]: r for r in csv.DictReader(f) if (r.get("run_id") or "").strip()
                and str(r.get("excluded", "")).strip().lower() not in ("1", "y", "yes", "true", "x")}
    cam = {}
    for path in a.calib_results:
        with open(path, newline="", encoding="utf-8-sig") as f:
            for r in csv.DictReader(f):
                if (r.get("event") == "nudge" and num(r.get("heading_floor_deg")) is not None
                        and "DISAGREE" not in (r.get("identity") or "")):
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
                        identity=c.get("identity", ""), moved_px=c.get("moved_px", ""),
                        det_heading_deg=c.get("det_heading_floor_deg", "")))
    if not out:
        sys.exit("No runs with both tape start marks and a camera nudge heading. Check run ids match.")
    print(f"{'run':10s} {'robot':7s} {'route':12s} {'tape':>8s} {'camera':>8s} {'diff':>7s}")
    for o in out:
        print(f"{o['run']:10s} {o['robot']:7s} {o['level']:12s} {o['tape_heading_deg']:8.2f} "
              f"{o['camera_heading_deg']:8.2f} {o['diff_deg']:7.2f}")
    d = [o["diff_deg"] for o in out]
    mean = st.mean(d)
    sd = st.stdev(d) if len(d) > 1 else float("nan")
    print(f"\n{len(d)} runs: mean difference {mean:+.2f} deg (camera rotation or a marking habit), "
          f"spread (SD) {sd:.2f} deg")
    print(f"Tape heading error is at most {sd:.2f} deg per run (the camera's own error is in this too).")
    pairs = [wrap180(num(o["det_heading_deg"]) - o["camera_heading_deg"]) for o in out
             if num(o["det_heading_deg"]) is not None]
    if len(pairs) >= 3 and len(d) > 1:
        cam_var = st.variance(pairs) / 2
        tape_var = sd ** 2 - cam_var
        print(f"Camera heading noise (from {len(pairs)} runs where the detector also gave a heading): "
              f"about {math.sqrt(cam_var):.2f} deg.")
        if tape_var > 0:
            print(f"Tape heading error estimate: about {math.sqrt(tape_var):.2f} deg per run.")
        else:
            print("The camera noise accounts for all of the spread: the tape's heading error is too small "
                  "to see with a 2 cm nudge.")
    else:
        print("No detector headings to compare, so the camera's share can't be taken out: "
              "only the upper bound above applies.")
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
