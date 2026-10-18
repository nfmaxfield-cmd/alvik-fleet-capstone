#!/usr/bin/env python3
"""Make a randomized run order that doubles as the tape-measurement sheet.

    python tools/make_run_order.py E1 --robots Left --reps 8 --seed 21
    python tools/make_run_order.py E0 --seed 5
    python tools/make_run_order.py E1B --robots Right --reps 4 --seed 22

Writes docs/experiments/<folder>/data/measurements.csv (refuses to overwrite
a sheet that already has measurements, unless --force). Print it or open it
in Excel, fill in the blank columns as you go, and save it as CSV.

Columns you fill in for each trial:
  run_id         from the console line "run 3F2A-7: ... starting"
  xf0_cm..yr0_cm the robot's front and rear marks BEFORE the run (where it really started)
  xf_cm, yf_cm   front mark after the run, from the taped axes
  xr_cm, yr_cm   rear mark after the run
  battery_pct    from `status` (optional)
  excluded       1 if the trial is thrown out, with the reason in note

Each robot runs every level (blocking by robot); the order is shuffled
within each robot so battery drain and floor wear don't line up with
one level. --seed makes the shuffle repeatable; note the seed in the
experiment's README.
"""
import argparse
import csv
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# experiment -> (folder, levels, commands(level), target(level))
# target = (x_cm ahead, y_cm to the left, heading_deg turned left) relative to
# where the robot starts; None = not measured for that experiment.
EXPERIMENTS = {
    "E0": ("E00-measurement-check", [f"P{i:02d}" for i in range(1, 11)],
           lambda lv: f"place the robot by hand at spot {lv}", lambda lv: (None, None, None)),
    # E1 baseline data set: every run logs all sensors, is filmed, and gets
    # tape marks at its start and its end.
    "E1": ("E01-baseline-dataset", ["straight120", "L", "square3"],
           lambda lv: {"straight120": "set straight_rpm 0 / set straight_cm 120 / run straight",
                       "L": "set demo_route 1 / set demo_a_cm 30 / set demo_b_cm 30 / run demo",
                       "square3": "set demo_route 3 / set square_cm 50 / set square_laps 3 / run demo"}[lv],
           lambda lv: (120.0, 0.0, 0.0) if lv == "straight120" else (0.0, 0.0, 0.0)),
}
# E1b: the same runs as E1 at low battery, for a paired comparison with the
# same robot's E1 runs at full battery.
EXPERIMENTS["E1B"] = ("E01b-low-battery",) + EXPERIMENTS["E1"][1:]

COLS = ["order", "robot", "level", "commands", "run_id", "target_x_cm", "target_y_cm", "target_heading_deg",
        "xf0_cm", "yf0_cm", "xr0_cm", "yr0_cm", "xf_cm", "yf_cm", "xr_cm", "yr_cm", "battery_pct", "excluded", "note"]


def blank(v):
    return "" if v is None else f"{v:g}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("exp", choices=sorted(EXPERIMENTS))
    ap.add_argument("--robots", default="Left", help="comma-separated, e.g. Left,Center,Right")
    ap.add_argument("--reps", type=int, help="trials per level per robot (E0: repeats per spot, default 2)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--out", help="output CSV (default: the experiment's data/measurements.csv)")
    ap.add_argument("--force", action="store_true", help="overwrite a sheet that already has measurements")
    args = ap.parse_args()

    folder, levels, how, target = EXPERIMENTS[args.exp]
    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    rng = random.Random(args.seed)
    rows = []

    if args.exp == "E0":
        reps = args.reps or 2
        robot = robots[0]
        for rep in range(1, reps + 1):
            spots = levels[:]
            rng.shuffle(spots)   # a new order each pass, so the second reading isn't from memory
            for lv in spots:
                rows.append((robot, lv, how(lv), f"pass {rep}"))
    else:
        if not args.reps:
            ap.error("--reps is required (trials per level per robot)")
        for robot in robots:
            trials = [lv for lv in levels for _ in range(args.reps)]
            rng.shuffle(trials)
            for lv in trials:
                rows.append((robot, lv, how(lv), ""))

    out = args.out or os.path.join(ROOT, "docs", "experiments", folder, "data", "measurements.csv")
    if os.path.exists(out) and not args.force:
        with open(out, newline="", encoding="utf-8-sig") as f:   # -sig: Excel's "CSV UTF-8" starts with a BOM
            filled = [r for r in csv.DictReader(f) if r.get("xf_cm", "").strip()]
        if filled:
            print(f"{out} already has {len(filled)} measured trials. Use --force to replace it.")
            return 1
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(COLS)
        for i, (robot, lv, cmd, note) in enumerate(rows, 1):
            tx, ty, th = target(lv)
            w.writerow([i, robot, lv, cmd, "", blank(tx), blank(ty), blank(th)] + [""] * 8 + ["", "", note])
    print(f"wrote {len(rows)} trials to {(os.path.relpath(out, ROOT) if os.path.splitdrive(os.path.abspath(out))[0] == os.path.splitdrive(ROOT)[0] else out)}" + (f" (seed {args.seed})" if args.seed is not None else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
