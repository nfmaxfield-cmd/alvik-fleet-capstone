#!/usr/bin/env python3
"""The hard simulator's LONG regression checks (a separate command, not collected by pytest: about 20-40 minutes on
two cores). Fast checks are in test_hard_simulator.py.

    python3 tools/fleet/tests/long_hardsim.py                 # measured + harsh, 4 seeds x 0.5 h each, then the checks
    python3 tools/fleet/tests/long_hardsim.py --seeds 1-8 --out logs/hardsim-long
    python3 tools/fleet/tests/long_hardsim.py --check-only --out logs/hardsim-long   # re-check results already there

Checks (each printed PASS / FAIL with its numbers; exit 1 on any FAIL):
  realism (timing the code cannot change): the simulated fix delay's median within 25 % of the real run's, quiet
          spells over 1 s within a factor 1.5 of the real rate, pictures a second within 30 %
  measured profile: no body contact; bodies never closer than 1 cm (CHOSEN margin); no robot off the board; no name swaps; never-still: no stand over 60 s by a robot
          free to move (not hung at an edge) that the run still counts in (Nick 12 Nov: movement never settles at 0)
  harsh profile: no body contact in more than a quarter of the runs; every run ends with every robot accounted for or
          held with a reason (never silently lost); no stand over 120 s by a robot free to move (not hung)
  measured: robots hung with a wheel over an edge 120 s at most in all, a run (never give up)
  cases: four single-fault runs the 12 Nov batches broke on (CASES below), each with the check that failed
The limits are the build's reading of Nick's rules (Claude's choice, flagged in the report); the numbers each run gives
are kept so a DOE can move them.

Version / changelog:
  long-hardsim-2026-11-12a  first version (hard simulator build).
  long-hardsim-2026-11-12e19  only actual contact fails; a pass under 1 cm is a WARNING (the coordinator, e19).
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
sys.path.insert(0, FLEET)
sys.path.insert(0, os.path.join(os.path.dirname(FLEET), "analysis"))
VERSION = "long-hardsim-2026-11-12e19"


# single-fault cases the 12 Nov batches broke on (fault, seed, what must hold): each a 0.5 h run on the measured profile
CASES = (("skew", 2, "free_stand"),          # e18: Right stood 311 s by a corner after a tilt recovery
         ("missing_cam", 1, "free_stand"),   # e18: Right stood 133 s with the overhead camera gone
         ("wrong_names", 2, "no_fall"),      # e17 / e18: robots fell with the camera's sightings mirrored
         ("restart", 1, "no_long_hang"))     # e18: a robot hung with a wheel over the edge for 21 min after a restart


def run(out, seeds, hours, jobs):
    B = os.path.join(FLEET, "hardsim_batch.py")
    for prof in ("measured", "harsh"):
        subprocess.check_call([sys.executable, B, "--profile", prof, "--seeds", seeds, "--hours", str(hours), "--jobs",
                               str(jobs), "--out", os.path.join(out, prof)])
    for fault, seed, _ in CASES:
        subprocess.check_call([sys.executable, B, "--profile", "measured", "--faults", fault, "--seeds", str(seed),
                               "--hours", "0.5", "--jobs", "1", "--out", os.path.join(out, "single")])


def load(d):
    return [json.load(open(f)) for f in sorted(glob.glob(os.path.join(d, "*.json")))]


def check(out):
    import fastsim_realism as R
    prof = R.load_profile("measured")
    real_delay = sorted(d for _, d in prof["timeline"]["pics"])
    real_d50 = real_delay[len(real_delay) // 2]
    real_stalls = prof["stall_pi_per_h"] + prof["stall_wifi_per_robot_h"]
    real_pics = prof["pictures_s"]["p50"]
    fails = []

    warns = []

    def say(ok, what, val, warn=False):
        """(e19, the coordinator: only actual contact is a failure; a pass under 1 cm is a WARNING)"""
        print("%s  %s: %s" % ("PASS" if ok else ("WARN" if warn else "FAIL"), what, val))
        if not ok:
            (warns if warn else fails).append(what)
    M, H = load(os.path.join(out, "measured")), load(os.path.join(out, "harsh"))
    if not M:
        print("no measured runs in", out)
        return 1
    d50 = sorted(r["measures"]["fix_delay_s"]["p50"] for r in M)[len(M) // 2]
    say(abs(d50 - real_d50) <= 0.25 * real_d50, "fix delay median (real %.2f s)" % real_d50, d50)
    st = sorted(r["measures"]["stalls"]["per_robot_h"] for r in M)[len(M) // 2]
    say(real_stalls / 1.5 <= st <= real_stalls * 1.5, "quiet spells over 1 s per robot-hour (real %.0f)" % real_stalls,
        st)
    pc = sorted(r["measures"]["pictures_s"]["p50"] for r in M)[len(M) // 2]
    say(abs(pc - real_pics) <= 0.3 * real_pics, "pictures a second (real %.1f)" % real_pics, pc)
    say(all(r["broke"]["contacts"] == 0 for r in M), "measured: no body contact",
        [r["broke"]["contacts"] for r in M])
    say(all(r["broke"]["min_gap_cm"] >= 1.0 for r in M), "measured: bodies never closer than 1 cm (a warning: no "
        "contact)", [r["broke"]["min_gap_cm"] for r in M], warn=True)
    say(all(r["broke"]["falls"] == 0 for r in M), "measured: no robot off the board", [r["broke"]["falls"] for r in M])
    say(all(r["broke"]["swaps"] == 0 for r in M), "measured: no name swaps", [r["broke"]["swaps"] for r in M])
    say(all(r["broke"].get("longest_free_stand_s", r["broke"]["longest_stand_s"]) <= 60 for r in M),
        "measured: never still (no stand over 60 s by a robot free to move)",
        [r["broke"].get("longest_free_stand_s") for r in M])
    if H:
        touched = sum(1 for r in H if r["broke"]["contacts"])
        say(touched <= len(H) / 4.0, "harsh: body contact in at most a quarter of the runs", "%d of %d" % (touched,
                                                                                                       len(H)))

        def long_stands(r):
            hu = r["broke"].get("hung_at")
            out = []
            for st in r["broke"]["stands"]:
                t, n, dur = st[0], st[1], st[2]
                if (hu and n == hu[0] and t <= hu[1] + hu[2] and t + dur >= hu[1]) or (len(st) > 5 and st[5]):
                    continue                    # hung (the fault, or a wheel over an edge): the body cannot move
                if dur > 120:
                    out.append((n, dur))
            return out
        say(all(not long_stands(r) for r in H), "harsh: no stand over 120 s by a robot free to move",
            [long_stands(r) for r in H])
    say(all(r["broke"]["hung_s"] <= 120 for r in M), "measured: no run with robots hung at an edge over 120 s in all",
        [r["broke"]["hung_s"] for r in M])
    for fault, seed, what in CASES:
        f = os.path.join(out, "single", "measured-%s-%d.json" % (fault, seed))
        if not os.path.exists(f):
            print("SKIP  case %s seed %d: not run" % (fault, seed))
            continue
        b = json.load(open(f))["broke"]
        if what == "free_stand":
            say(b.get("longest_free_stand_s", 0) <= 60, "case %s seed %d: no stand over 60 s by a robot free to move"
                % (fault, seed), b.get("longest_free_stand_s"))
        elif what == "no_fall":
            say(b["falls"] == 0, "case %s seed %d: no robot off the board" % (fault, seed), b["falls"])
        elif what == "no_long_hang":
            say(b["hung_s"] <= 120, "case %s seed %d: robots hung at an edge 120 s at most in all" % (fault, seed),
                b["hung_s"])
    if warns:
        print("%d warning(s): %s" % (len(warns), "; ".join(warns)))
    print("%d check(s) failed" % len(fails) if fails else "all checks passed")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seeds", default="1-4")
    ap.add_argument("--hours", type=float, default=0.5)
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out")
    ap.add_argument("--check-only", action="store_true")
    a = ap.parse_args(argv)
    out = a.out or tempfile.mkdtemp(prefix="hardsim-long-")
    if not a.check_only:
        run(out, a.seeds, a.hours, a.jobs)
    sys.exit(check(out))


if __name__ == "__main__":
    main()
