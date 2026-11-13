#!/usr/bin/env python3
"""Hard-simulator batches: many seeds of the free-roam controller on a realism profile (fastsim_realism.py), one JSON
per run with the realism measures and what broke. Simulator only: nothing touches a robot or a Pi.

    python3 tools/fleet/hardsim_batch.py --profile measured --seeds 1-8 --hours 1 --jobs 2 --out logs/hardsim/measured
    python3 tools/fleet/hardsim_batch.py --profile measured --faults hung --seeds 1-4 --hours 0.5 --out ...
    python3 tools/fleet/hardsim_batch.py --profile measured --code /path/to/old/tools/fleet ...   # another code version
    python3 tools/fleet/hardsim_batch.py summarize logs/hardsim/measured/*.json

Version / changelog:
  hardsim-batch-2026-11-12a  first version (hard simulator build).
  hardsim-batch-2026-11-12e21  each run's JSON also keeps the controller's seconds in each of Nick's stages (stage_s),
                 a few e21 counts (escapes, searches, veers, heading flips caught) and the simulated firmware's refusals.
"""
import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
VERSION = "hardsim-batch-2026-11-12e21"   # e21: stage_s, e21 counts and firmware refusals kept. Before: "hardsim-batch-2026-11-12a"


def run_one(seed, hours, profile, out, code=None, faults=None, profile_path=None):
    """One run in THIS process (the batch runs each in its own process)."""
    if code:
        sys.path.insert(0, code)          # the controller / simulator code under test
    if HERE not in sys.path:
        sys.path.insert(1 if code else 0, HERE)
    import freeroam_fastsim as F
    import fastsim_realism as R
    prof = R.load_profile(profile, profile_path)
    Cls = R.make_class(F)
    fl = {k: True for k in (faults or [])}
    t0 = time.time()
    s = Cls(seed, hours, profile=profile, prof=prof, faults=fl,
            hdg_fault={"deg": 25, "near_cm": 14, "every_s": 60} if profile == "easy" else None,
            camera="detections", sensors=True, edge_physics=True)
    summ = s.run()
    res = dict(version=VERSION, realism=R.VERSION, fastsim=getattr(F, "VERSION", None), code=code or HERE,
               seed=seed, hours=hours, profile=profile, faults=sorted(fl), secs=round(time.time() - t0, 1),
               env={k: v for k, v in os.environ.items() if k.startswith("MIRLT_")},
               measures=s.measure_sim(), broke=s.breakage(summ),
               summary={k: summ.get(k) for k in ("contacts", "min_gap_cm", "off_board", "watchdog_stops",
                                                  "longest_still_s", "lost_episodes", "moving_pct", "falls",
                                                  "hangs", "edge_min_body_cm")},
               stage_s={k: round(v, 1) for k, v in (getattr(s.ctrl, "stage_s", None) or {}).items()},
               e21={k: v for k, v in s.ctrl.counts.items() if k in (
                   "consistent_escapes", "searches", "search_holds", "veers", "veer_steps", "heading_flips_caught",
                   "head_unknown_confirmed", "shape_events", "creep_edge_refused", "camera_corrections", "creeps")},
               fw_refusals={k: v for k, v in getattr(s, "stats", {}).items() if str(k).startswith("fw_")})
    with open(out, "w") as f:
        json.dump(res, f, default=str)
    return res


def seeds_of(spec):
    out = []
    for part in spec.split(","):
        if "-" in part:
            a, b = part.split("-")
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def batch(a):
    os.makedirs(a.out, exist_ok=True)
    todo = []
    for sd in seeds_of(a.seeds):
        tag = "%s%s-%d" % (a.profile, ("-" + "+".join(a.faults)) if a.faults else "", sd)
        out = os.path.join(a.out, tag + ".json")
        if os.path.exists(out) and not a.force:
            continue
        cmd = [sys.executable, os.path.abspath(__file__), "one", str(sd), str(a.hours), a.profile, out]
        if a.code:
            cmd += ["--code", a.code]
        if a.faults:
            cmd += ["--faults", ",".join(a.faults)]
        if a.profile_path:
            cmd += ["--profile-path", a.profile_path]
        todo.append((tag, cmd))
    running = []
    while todo or running:
        while todo and len(running) < a.jobs:
            tag, cmd = todo.pop(0)
            log = open(os.path.join(a.out, tag + ".log"), "w")
            running.append((tag, subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT), log, time.time()))
        time.sleep(1.0)
        for item in list(running):
            tag, p, log, t0 = item
            if p.poll() is not None:
                log.close()
                running.remove(item)
                print("%s %s in %.0f s" % (tag, "ok" if p.returncode == 0 else "FAILED %d" % p.returncode,
                                           time.time() - t0), flush=True)


def summarize(files):
    rows = [json.load(open(f)) for f in files if f.endswith(".json")]
    for r in rows:
        b = r["broke"]
        print("%-28s contacts %d min %.1f falls %d swaps %d hands %d stands>5s %d (longest %.0f s) breaches %d/%d wd %d"
              % (os.path.basename(r.get("profile", "")) + "-" + "+".join(r.get("faults", [])) + "-%d" % r["seed"],
                 b["contacts"], b["min_gap_cm"], b["falls"], b["swaps"], b["needs_hand"], b["stands_over"],
                 b["longest_stand_s"], b["blind_breaches"], b["blind_samples"], b["watchdog_stops"]))


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "one":
        ap = argparse.ArgumentParser()
        ap.add_argument("cmd")
        ap.add_argument("seed", type=int)
        ap.add_argument("hours", type=float)
        ap.add_argument("profile")
        ap.add_argument("out")
        ap.add_argument("--code")
        ap.add_argument("--faults", default="")
        ap.add_argument("--profile-path")
        a = ap.parse_args(argv)
        run_one(a.seed, a.hours, a.profile, a.out, a.code, [f for f in a.faults.split(",") if f], a.profile_path)
        return
    if argv and argv[0] == "summarize":
        summarize(argv[1:])
        return
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--profile", default="measured", choices=("easy", "measured", "harsh"))
    ap.add_argument("--faults", default="", help="harsh faults on top of the profile, comma separated")
    ap.add_argument("--seeds", default="1-4")
    ap.add_argument("--hours", type=float, default=0.5)
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out", required=True)
    ap.add_argument("--code", help="another tools/fleet folder to test (default: this one)")
    ap.add_argument("--profile-path")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args(argv)
    a.faults = [f for f in a.faults.split(",") if f]
    batch(a)


if __name__ == "__main__":
    main()
