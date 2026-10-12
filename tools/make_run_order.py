#!/usr/bin/env python3
"""Make a randomized run order that doubles as the tape-measurement sheet.

    python tools/make_run_order.py E1a --robots Left --reps 16
    python tools/make_run_order.py E1b --robots Left,Center,Right --reps 3

Writes docs/experiments/<folder>/data/measurements.csv (refuses to overwrite
a sheet that already has measurements, unless --force). Print it or open it
in Excel, fill in the blank columns as you go, and save it as CSV.

Columns you fill in after each trial:
  run_id         from the console line "run 3F2A-7: ... starting"
  xf_cm, yf_cm   front end of the pointer ruler, from the taped axes
  xr_cm, yr_cm   rear end of the pointer ruler
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
# target = (x_cm, y_cm, heading_deg); None = not measured for that experiment.
# Folders follow the course numbering: E1 = Alvik precision, odometry only.
E1 = "E01-odometry-precision/"
EXPERIMENTS = {
    "E0": ("E00-measurement-check", [f"P{i:02d}" for i in range(1, 11)],
           lambda lv: f"place the robot by hand at spot {lv}", lambda lv: (None, None, None)),
    "E1a": (E1 + "a-turns", ["library", "imu"],
            lambda lv: f"set turn_n 1 / set turn_method {0 if lv == 'library' else 1} / run turn",
            lambda lv: (None, None, 90.0)),
    "E1b": (E1 + "b-straight", ["30", "60", "120"],
            lambda lv: f"set straight_rpm 0 / set straight_cm {lv} / run straight",
            lambda lv: (float(lv), 0.0, 0.0)),
    "E1c": (E1 + "c-l-path", ["L"],
            lambda lv: "set demo_route 1 / run demo", lambda lv: (0.0, 0.0, 0.0)),
    "E1d": (E1 + "d-square", ["1lap", "3laps"],
            lambda lv: f"set demo_route 3 / set square_cm 50 / set square_laps {lv[0]} / run demo",
            lambda lv: (0.0, 0.0, 0.0)),
    # E1e: speed is shuffled; battery can't be, so it runs as two blocks (high first, then low).
    "E1e": (E1 + "e-speed-battery", ["20rpm", "50rpm"],
            lambda lv: f"set straight_cm 60 / set straight_rpm {lv[:2]} / run straight",
            lambda lv: (60.0, 0.0, 0.0)),
}

COLS = ["order", "robot", "level", "commands", "run_id", "target_x_cm", "target_y_cm", "target_heading_deg",
        "xf_cm", "yf_cm", "xr_cm", "yr_cm", "battery_pct", "excluded", "note"]


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
    elif args.exp == "E1e":
        if not args.reps:
            ap.error("--reps is required (trials per speed per battery block per robot)")
        for robot in robots:
            for block in ("high", "low"):
                trials = [lv for lv in levels for _ in range(args.reps)]
                rng.shuffle(trials)
                for lv in trials:
                    note = "battery above 80%" if block == "high" else "battery below 30%"
                    rows.append((robot, f"{lv}-{block}", how(lv), note))
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
        with open(out, newline="") as f:
            filled = [r for r in csv.DictReader(f) if r.get("xf_cm", "").strip()]
        if filled:
            print(f"{out} already has {len(filled)} measured trials. Use --force to replace it.")
            return 1
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLS)
        for i, (robot, lv, cmd, note) in enumerate(rows, 1):
            tx, ty, th = target(lv.split("-")[0])
            w.writerow([i, robot, lv, cmd, "", blank(tx), blank(ty), blank(th), "", "", "", "", "", "", note])
    print(f"wrote {len(rows)} trials to {os.path.relpath(out, ROOT)}" + (f" (seed {args.seed})" if args.seed is not None else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
