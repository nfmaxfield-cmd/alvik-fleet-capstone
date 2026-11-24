#!/usr/bin/env python3
"""(e24, Nick 24 Nov D5) The recorder's capture rate, decided from the Pi's own measured load instead of typed.

    python3 tools/pi/capture_rate.py show                 # the rate in force at this site and why
    python3 tools/pi/capture_rate.py decide <run folder>   # what this one run's numbers call for (nothing saved)
    python3 tools/pi/capture_rate.py note <run folder>     # the same, added to the site's record (a real run's end does it)

Why: on 23 Nov Jay-Sun lowered the school's rate from 15 to 10 pictures a second. The runs' own numbers showed no
measured difference: the recorder filled core 0 either way, and the tag bridge read camera 1 only 1.2-1.7 times a second
and camera 2 2.6-4.1 times with two robots on the table (docs/results/school-2026-11-23/report.md, 2.6). Pictures the
bridge never reads only cost the recorder's core. So the rate follows what the bridge can READ:

    wanted rate = the lowest of the site's choices that is at least NEED_X times the bridge's busiest camera's read rate
                  (every read then finds a new picture, with one to spare); the highest choice when none is enough.

The site's file gives the choices and the rate to start from (capture.fps_auto: {"choices": [10, 15], "start": 10});
the record lives beside the site's camera names (<conf dir>/capture_rate.json), never in the repo. The rate in force is
the LOWEST the last KEEP_N real runs called for (the busiest run governs: two robots load the Pi more than one). A rate
set by hand on this Pi (`./run-fleet-pi.sh local set capture.fps 15`) wins over all of this. A change takes effect when
the recorder next starts.

Evidence read per run: bridge/detect_times.csv (each picture the bridge read, per camera) and cpu.csv (core 0, the
recorder's share): the cores are recorded with the decision but do not change it (on 23 Nov core 0 read 100 % at both
rates, so it cannot tell them apart).

Changelog
  2026-11-24  first version (e24).
"""
import csv
import json
import os
import statistics
import sys
import time

VERSION = "capture-rate-2026-11-24"
NEED_X = 2.0     # [chosen] pictures captured per picture read: one new picture for every read and one to spare (a read
#                  that just misses a picture gets the next). DOE: yes, on the Pi's timing bench (./run-fleet-pi.sh
#                  bench): 10 against 15 per core layout, with one and two robots in view
KEEP_N = 5       # [chosen] the last real runs whose wanted rates count. DOE: no (a memory length; one odd run cannot
#                  set the rate for long, and a changed set-up is followed within a day's runs)
MIN_PICTURES = 50   # a run with fewer pictures read by a camera says nothing (a refused start). No DOE: a floor
HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.join(os.path.dirname(HERE), "fleet")


def read_rates(folder):
    """{camera: pictures the bridge read a second} from the run's bridge/detect_times.csv ({} when there is none)."""
    p = os.path.join(folder, "bridge", "detect_times.csv")
    by = {}
    try:
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    by.setdefault(r.get("cam") or "?", []).append(float(r.get("t_done") or r.get("t")))
                except (TypeError, ValueError):
                    continue
    except OSError:
        return {}
    out = {}
    for cam, v in by.items():
        if len(v) < MIN_PICTURES:
            continue
        v.sort()
        dur = v[-1] - v[0]
        if dur > 1.0:
            out[cam] = round((len(v) - 1) / dur, 2)
    return out


def cores(folder):
    """Median core 0 busy % and the recorder's CPU % over the run (cpu.csv), for the record only."""
    c0, rec = [], []
    try:
        with open(os.path.join(folder, "cpu.csv"), newline="") as f:
            for r in csv.DictReader(f):
                for k, lst in (("core0", c0), ("recorder_cpu", rec)):
                    try:
                        lst.append(float(r[k]))
                    except (KeyError, TypeError, ValueError):
                        pass
    except OSError:
        pass
    return {"core0_pct": round(statistics.median(c0), 1) if c0 else None,
            "recorder_cpu_pct": round(statistics.median(rec), 1) if rec else None}


def wanted(rates, choices):
    """The lowest choice that is at least NEED_X x the busiest camera's read rate (the highest when none is)."""
    choices = sorted(float(c) for c in choices)
    if not rates or not choices:
        return None
    need = NEED_X * max(rates.values())
    for c in choices:
        if c >= need:
            return c
    return choices[-1]


def decide(folder, choices):
    rates = read_rates(folder)
    w = wanted(rates, choices)
    if w is None:
        return {"fps": None, "why": "this run has no record of the pictures the bridge read (bridge/detect_times.csv)",
                "rates": rates}
    cam = max(rates, key=rates.get)
    return {"fps": w, "rates": rates, "cores": cores(folder), "run": os.path.basename(folder.rstrip("/")),
            "why": "the bridge read at most %.1f pictures a second (the %s camera); %g a second is the lowest choice "
                   "that gives every read a new picture %g times over" % (rates[cam], cam, w, NEED_X)}


def record_path(site):
    return os.path.join(site["conf_dir"], "capture_rate.json")


def load_record(site):
    try:
        with open(record_path(site)) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def auto_cfg(site):
    a = (site.get("capture") or {}).get("fps_auto")
    if not isinstance(a, dict) or not a.get("choices"):
        return None
    return {"choices": [float(c) for c in a["choices"]], "start": float(a.get("start") or min(a["choices"]))}


def in_force(site):
    """(fps, words): the capture rate the recorder is started with at this site, and where it comes from."""
    cap = site.get("capture") or {}
    local = (site.get("_local") or {}).get("capture.fps")
    if local is not None:
        return float(local), "set by hand on this Pi (site_local.json)"
    a = auto_cfg(site)
    if a is None:
        return float(cap.get("fps", 25)), "the site file's (no fps_auto there)"
    runs = [r for r in (load_record(site).get("runs") or []) if r.get("fps")][-KEEP_N:]
    if not runs:
        return a["start"], "the site's starting rate (no real run measured here yet)"
    low = min(runs, key=lambda r: r["fps"])
    return float(low["fps"]), "the lowest the last %d run%s called for: %s (%s)" % (
        len(runs), "" if len(runs) == 1 else "s", low.get("why", ""), low.get("run", ""))


def note(folder, site, say=print):
    """A real run's end: its wanted rate goes into the site's record. Returns the decision (dict) or None."""
    a = auto_cfg(site)
    if a is None:
        return None
    before = in_force(site)[0]
    d = decide(folder, a["choices"])
    if d.get("fps") is None:
        return None
    rec = load_record(site)
    runs = [r for r in (rec.get("runs") or []) if r.get("run") != d.get("run")]
    runs.append(dict(d, t=round(time.time(), 1)))
    rec.update(version=VERSION, runs=runs[-20:])
    os.makedirs(site["conf_dir"], exist_ok=True)
    with open(record_path(site), "w") as f:
        json.dump(rec, f, indent=1)
    after, words = in_force(site)
    say("CAPTURE RATE: this run calls for %g pictures a second (%s). In force at this site: %g (%s)%s" % (
        d["fps"], d["why"], after, words,
        "" if after == before else ": CHANGED from %g; it applies when the recorder next starts "
                                   "(./run-fleet-pi.sh mc-stop, then ./run-fleet-pi.sh mc)" % before))
    return d


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    sys.path.insert(0, FLEET)
    import sites as SI
    site = SI.load()
    cmd = argv[0] if argv else "show"
    if cmd == "show":
        fps, words = in_force(site)
        print("Capture rate at %s: %g pictures a second: %s." % (site["name"], fps, words))
        for r in (load_record(site).get("runs") or [])[-KEEP_N:]:
            print("  %s: wanted %g (read a second: %s; core 0 %s %%, recorder %s %%)" % (
                r.get("run"), r.get("fps"), ", ".join("%s %.1f" % kv for kv in sorted((r.get("rates") or {}).items())),
                (r.get("cores") or {}).get("core0_pct"), (r.get("cores") or {}).get("recorder_cpu_pct")))
        return 0
    if cmd in ("decide", "note") and len(argv) > 1:
        a = auto_cfg(site) or {"choices": [10.0, 15.0]}
        if cmd == "note":
            d = note(argv[1], site)
            if d is None:
                print("Nothing noted (no fps_auto at this site, or the run has no bridge record).")
        else:
            print(json.dumps(decide(argv[1], a["choices"]), indent=1))
        return 0
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())
