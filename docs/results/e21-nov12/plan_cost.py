#!/usr/bin/env python3
"""e21 (Nick 12 Nov, decision 3: "no overlapping shapes ... as long as it doesn't overload the Pi"): what keeping the
error shapes apart costs the planner, and how accurate each bound is. Simulator only.

    python3 docs/results/e21-nov12/plan_cost.py --e20 <e20 tools/fleet> --out docs/results/e21-nov12/plan_cost.json

1. Accuracy (random shape pairs): the gap each bound reports against the true gap between the two shapes (sampled
   separating axes): a circle of the shape's largest reach, the ellipse's reach along the line between the centres (the
   representation e21 uses), and the capsule bound. A bound is SAFE when it never reports more gap than there is.
2. Cost, the micro: one clearance() between two reservations of a typical move (both with shapes / neither).
3. Cost, the whole planner: the free-roam controller's tick (everything it does each 50 ms), timed in the fast simulator
   (the measured profile, seed 1, 0.1 h) for the e20 code and this code, the same seed. Scaled to the Pi 4 by the
   measured ratio (about 5 times slower than this build machine: docs/build-reports/e02-nov02-conference-table.md).

Changelog: e21-2026-11-12a first version.
"""
import argparse
import json
import math
import os
import random
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.join(HERE, "..", "..", "..", "tools", "fleet")
PI_SLOWER = 5.0
TICK_S = 0.05

TICK_CODE = r'''
import sys, time, json, os
sys.path.insert(0, sys.argv[1])
import freeroam_fastsim as F, fastsim_realism as R, freeroam_core as FC
ts = []
orig = FC.Controller.tick
def tick(self, t):
    a = time.perf_counter(); orig(self, t); ts.append(time.perf_counter() - a)
FC.Controller.tick = tick
prof = R.load_profile(sys.argv[2]); Cls = R.make_class(F)
s = Cls(int(sys.argv[3]), float(sys.argv[4]), profile=sys.argv[2], prof=prof, camera="detections", sensors=True,
        edge_physics=True)
s.run()
ts.sort()
q = lambda p: ts[int(p * (len(ts) - 1))]
print(json.dumps(dict(n=len(ts), mean_ms=1000 * sum(ts) / len(ts), p50_ms=1000 * q(.5), p95_ms=1000 * q(.95),
                      p99_ms=1000 * q(.99), max_ms=1000 * ts[-1])))
'''


def accuracy(n=2000, seed=1):
    sys.path.insert(0, FLEET)
    import error_shape as ES
    rng = random.Random(seed)
    rows = dict(circle=[], ellipse=[], capsule=[])
    unsafe = dict(circle=0, ellipse=0, capsule=0)
    for _ in range(n):
        sa = (rng.uniform(0, 4), rng.uniform(0, 15), rng.uniform(0, 15), rng.uniform(-180, 180), rng.uniform(-8, 0))
        sb = (rng.uniform(0, 4), rng.uniform(0, 15), rng.uniform(0, 15), rng.uniform(-180, 180), rng.uniform(-8, 0))
        p, q = (0.0, 0.0), (rng.uniform(-60, 60), rng.uniform(-60, 60))
        tg = ES.true_gap(p, sa, q, sb, 360)
        if tg <= 0:
            continue
        ra = sa[0] + max(sa[1] + abs(sa[4]), sa[2])
        rb = sb[0] + max(sb[1] + abs(sb[4]), sb[2])
        g = dict(circle=math.hypot(q[0], q[1]) - ra - rb, ellipse=ES.pair_gap(p, sa, q, sb),
                 capsule=ES.capsule_gap(p, sa, q, sb))
        for k, v in g.items():
            rows[k].append(tg - v)                  # the gap given up (cm): 0 is exact, > 0 cautious, < 0 unsafe
            if v > tg + 1e-6:
                unsafe[k] += 1
    out = {}
    for k, v in rows.items():
        v.sort()
        out[k] = dict(n=len(v), lost_median_cm=round(v[len(v) // 2], 2), lost_p90_cm=round(v[int(0.9 * (len(v) - 1))], 2),
                      unsafe=unsafe[k])
    return out


def micro(reps=3000):
    sys.path.insert(0, FLEET)
    import freeroam_core as FC
    A = FC.Traj.move(0.0, (20.0, 20.0), 0.0, 30.0, 40.0, 7.5)
    B = FC.Traj.move(0.0, (60.0, 40.0), 180.0, -30.0, 40.0, 7.5)
    t0 = time.perf_counter()
    for _ in range(reps):
        FC.clearance(A, B, now=0.0)
    plain = (time.perf_counter() - t0) / reps
    A.set_shape([(0.0, 10.0, 5.0, 30.0)] * len(A.x))
    B.set_shape([(0.0, 8.0, 4.0, 150.0)] * len(B.x))
    t0 = time.perf_counter()
    for _ in range(reps):
        FC.clearance(A, B, now=0.0)
    shaped = (time.perf_counter() - t0) / reps
    return dict(samples=(len(A.x), len(B.x)), plain_us=round(1e6 * plain, 1), shaped_us=round(1e6 * shaped, 1),
                ratio=round(shaped / plain, 2))


def ticks(code, profile="measured", seed=1, hours=0.1):
    r = subprocess.run([sys.executable, "-c", TICK_CODE, os.path.abspath(code), profile, str(seed), str(hours)],
                       capture_output=True, text=True, env=dict(os.environ, HOME=os.environ.get("TMPHOME", "/tmp")))
    try:
        return json.loads(r.stdout.strip().splitlines()[-1])
    except Exception:  # noqa: BLE001
        return dict(error=r.stderr[-800:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--e20", required=True)
    ap.add_argument("--out")
    ap.add_argument("--profile", default="measured")
    a = ap.parse_args()
    res = dict(accuracy=accuracy(), micro=micro())
    res["tick_e20"] = ticks(a.e20, a.profile)
    res["tick_e21"] = ticks(FLEET, a.profile)
    for k in ("tick_e20", "tick_e21"):
        d = res[k]
        if "mean_ms" in d:
            d["pi_mean_ms"] = round(PI_SLOWER * d["mean_ms"], 2)
            d["pi_p95_ms"] = round(PI_SLOWER * d["p95_ms"], 2)
            d["pi_share_of_tick_p95"] = round(PI_SLOWER * d["p95_ms"] / (1000 * TICK_S), 3)
    print(json.dumps(res, indent=1))
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
