#!/usr/bin/env python3
"""The free-roam controller's tick timed ON THE PI (e22, Nick 13 Nov: "the slowest 1 in 100 planner ticks was estimated at
~51 ms against the 50 ms loop": this measures it on the Pi itself). No robots, no cameras: the hard simulator's robots
(tools/fleet/fastsim_realism.py) run the real controller (tools/fleet/freeroam_core.py) as fast as the Pi can, and every
tick's own time is measured with the Pi's clock. Nothing moves and nothing is sent anywhere.

    python3 tools/pi/plan_timing_bench.py                 # e22's way and e21's way, two profiles (about 5-8 min)
    python3 tools/pi/plan_timing_bench.py --quick         # e22's way only, one profile (about 2 min)
    ./run-fleet-pi.sh timing                              # the same as the first line

It prints, for each case, the slowest 1 in 100 ticks (the 99th percentile) against the 50 ms loop, and writes
logs/timing/plan-timing-<date>.json (every number, the Pi's model and load) for the report.

Fine print: a percentile is the value that share of the ticks stays under; the 99th is the slowest 1 in 100.

Changelog: plan-timing-bench-e22-2026-11-13a first version.
"""
import argparse
import json
import os
import platform
import subprocess
import sys
import time

VERSION = "plan-timing-bench-e22-2026-11-13a"
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
FLEET = os.path.join(REPO, "tools", "fleet")
TICK_MS = 50.0

ONE = r'''
import sys, time, json, os
sys.path.insert(0, sys.argv[1])
import freeroam_fastsim as F, fastsim_realism as R, freeroam_core as FC
ts = []
clock = time.thread_time if os.environ.get("BENCH_CPU") == "1" else time.perf_counter
orig = FC.Controller.tick
def tick(self, t):
    a = clock(); orig(self, t); ts.append(clock() - a)
FC.Controller.tick = tick
prof = R.load_profile(sys.argv[2]); Cls = R.make_class(F)
s = Cls(int(sys.argv[3]), float(sys.argv[4]), profile=sys.argv[2], prof=prof, camera="detections", sensors=True,
        edge_physics=True)
for t_ in s.ctrl.trust.values():
    t_.async_fit = True                        # as the Pi's runner sets it (freeroam.py): refits off the loop
w0 = time.time()
s.run()
ts.sort()
n = len(ts)
q = lambda p: 1000.0 * ts[int(p * (n - 1))]
print(json.dumps(dict(ticks=n, mean_ms=round(1000.0 * sum(ts) / n, 3), p50_ms=round(q(.5), 3), p95_ms=round(q(.95), 3),
                      p99_ms=round(q(.99), 3), p999_ms=round(q(.999), 3), max_ms=round(1000.0 * ts[-1], 2),
                      over_50ms=sum(1 for x in ts if x > 0.05), wall_s=round(time.time() - w0, 1),
                      deferred=s.ctrl.counts.get("plans_deferred", 0) + s.ctrl.counts.get("picks_deferred", 0))))
'''


def run_case(profile, seed, hours, way, cpu=False):
    env = dict(os.environ, MIRLT_PLAN_BUDGET="derived" if way == "e22" else "every_tick", MIRLT_PLAN_SLOWDOWN="1",
               BENCH_CPU="1" if cpu else "0")
    env.setdefault("HOME", "/tmp")
    t0 = time.time()
    r = subprocess.run([sys.executable, "-c", ONE, FLEET, profile, str(seed), str(hours)], capture_output=True,
                       text=True, env=env)
    try:
        d = json.loads(r.stdout.strip().splitlines()[-1])
    except Exception:  # noqa: BLE001
        d = dict(error=(r.stderr or r.stdout)[-600:])
    d.update(profile=profile, seed=seed, hours=hours, way=way, secs=round(time.time() - t0, 1))
    return d


def loadavg():
    try:
        return os.getloadavg()
    except OSError:
        return None


def model():
    try:
        return open("/proc/device-tree/model").read().strip("\x00").strip()
    except OSError:
        return platform.machine()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quick", action="store_true", help="e22's way only, the measured profile only")
    ap.add_argument("--hours", type=float, default=0.1, help="simulated hours per case (0.1 = about 6,700 ticks)")
    ap.add_argument("--out", default=os.path.join(REPO, "logs", "timing"))
    ap.add_argument("--cpu", action="store_true", help="time each tick by the CPU time it used (a machine running "
                    "other work); default: the wall clock, as the loop lives it")
    ap.add_argument("--seeds", default=None, help="profile:seed,... (default measured:1,harsh:4)")
    a = ap.parse_args(argv)
    cases = [("measured", 1)] if a.quick else [("measured", 1), ("harsh", 4)]
    if a.seeds:
        cases = [(q.split(":")[0], int(q.split(":")[1])) for q in a.seeds.split(",")]
    ways = ["e22"] if a.quick else ["e22", "e21"]
    print("Timing the free-roam controller on %s (load %s). Nothing moves. Each case takes 1-3 minutes." % (
        model(), loadavg()))
    res = []
    for prof, seed in cases:
        for way in ways:
            print("  %s seed %d, %s's way ..." % (prof, seed, way), flush=True)
            d = run_case(prof, seed, a.hours, way, a.cpu)
            res.append(d)
            if "error" in d:
                print("    FAILED: %s" % d["error"])
                continue
            print("    slowest 1 in 100 ticks: %.1f ms of the %.0f ms loop (%s); typical %.1f ms; worst %.0f ms; "
                  "%d of %d ticks over 50 ms" % (d["p99_ms"], TICK_MS, "fits" if d["p99_ms"] <= TICK_MS else "OVER",
                                                 d["p50_ms"], d["max_ms"], d["over_50ms"], d["ticks"]), flush=True)
    os.makedirs(a.out, exist_ok=True)
    fn = os.path.join(a.out, "plan-timing-%s.json" % time.strftime("%Y%m%d-%H%M%S"))
    with open(fn, "w") as f:
        json.dump(dict(version=VERSION, model=model(), load=loadavg(), python=sys.version.split()[0],
                       clock="thread CPU time" if a.cpu else "wall clock",
                       when=time.strftime("%Y-%m-%d %H:%M:%S"), cases=res), f, indent=1)
    ok = [d for d in res if d.get("way") == "e22" and "p99_ms" in d]
    worst = max([d["p99_ms"] for d in ok] or [float("nan")])
    print("")
    print("RESULT: e22's slowest 1 in 100 ticks: %.1f ms (the loop is %.0f ms): %s" % (
        worst, TICK_MS, "FITS" if worst <= TICK_MS else "OVER THE LOOP"))
    print("Saved: %s" % fn)
    return 0


if __name__ == "__main__":
    sys.exit(main())
