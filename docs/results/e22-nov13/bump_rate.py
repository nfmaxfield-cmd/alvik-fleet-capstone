#!/usr/bin/env python3
"""e22 (Nick 13 Nov: "simulator bumps and slips at the measured rates: yes, if it matches reality or helps prepare for
it"): the rate and size of BUMPS (the body moved by something other than its own wheels: another robot's touch) in real
home free roams, for the hard simulator's measured profile. Logs only; nothing touches a robot.

    python3 docs/results/e22-nov13/bump_rate.py <run folder> [...] --jumps docs/results/e21-nov12/cone_fit.json \
        --out docs/results/e22-nov13/bump_rate.json

Rate: the robots' own "felt a robot" events (the accelerometer's jolt from a touch, e16's contact watch), counted per
robot-hour of each run (its events' first to last time x the robots in it). Size: the measured jump of the same event
(cone_fit.json "felt_robot": the own-vs-camera gap 2-3 s after it less the gap before; its quantiles). Slips are already
measured into the profile (realism_measure: slips_per_robot_h, slip_len_s); this script only reports them alongside.

Changelog: e22-2026-11-13a first version.
"""
import argparse
import csv
import json
import os

VERSION = "bump-rate-e22-2026-11-13a"


def run_rate(d):
    rows = list(csv.DictReader(open(os.path.join(d, "events.csv"))))
    ts = [float(r["t"]) for r in rows if r.get("t")]
    robots = sorted({r["robot"] for r in rows if r.get("robot") and r["robot"] not in ("-", "")})
    hours = (max(ts) - min(ts)) / 3600.0 if ts else 0.0
    felt = sum(1 for r in rows if r.get("kind") == "felt a robot")
    return dict(run=os.path.basename(d.rstrip("/")), hours=round(hours, 3), robots=robots, felt=felt,
                robot_h=round(hours * len(robots), 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--jumps", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rs = [run_rate(d) for d in a.runs]
    rh = sum(r["robot_h"] for r in rs)
    n = sum(r["felt"] for r in rs)
    j = json.load(open(a.jumps))["event_jumps"]["felt_robot"]
    out = dict(version=VERSION, runs=rs, robot_h=round(rh, 2), felt=n,
               bumps_per_robot_h=round(n / rh, 3) if rh else 0.0,
               bump_cm={k: round(float(v), 2) for k, v in j.items() if k.startswith("p")}, bump_cm_n=j.get("n"),
               note="rate: 'felt a robot' events per robot-hour; size: their measured jump quantiles (cone_fit.json)")
    json.dump(out, open(a.out, "w"), indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
