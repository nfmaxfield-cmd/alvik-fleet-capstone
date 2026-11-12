#!/usr/bin/env python3
"""How big a share of the planner's core may the trained detector take? A short bench on the home Pi that answers it
from data (Mission Control build, Nick 12 Nov; how-to: docs/how-to/pi-bench-core-share.md).

    python3 tools/pi/detector_share_bench.py run [--minutes 5] [--shares 10,25,50]
    python3 tools/pi/detector_share_bench.py apply 25          # the chosen share for every run (share.json)
    python3 tools/pi/detector_share_bench.py report DIR        # the table and the choice again

Robots stay PARKED (nothing moves): the detector needs only to see them. The recorder must be running (the real
cameras). For each share, in turn, for --minutes each, with a share of 0 first (the detector off: the baseline):
  * a normal run's load: a practice free roam (freeroam.py run --dry, simulated robots) pinned like a real one (the
    planner's core 1), and the recorder as it is;
  * the detector on the planner's core 1, capped at the share (tagfree.Budget, as in a run), on the recorder's newest
    overhead picture, asked for one parked robot at a time as if it were lost;
  * the Pi efficiency monitor (pi_monitor.py) beside it: picture lag per camera, the planner's loop time, each core,
    temperature, throttling.
The detector's benefit is the TIME TO FIND a lost (standing) robot: from the ask until a detection lands within FOUND_CM
of where that robot is parked (its spot learnt in the first minute of the first share over 0, from the detector's own
detections).

The choice: the baseline's 95th percentile is the run's normal spread. A share passes when its median picture lag and
its median planner loop (95 % of passes) stay at or under the baseline's 95th percentiles, and no throttle flag appears
that the baseline did not have. The largest share that passes is chosen. If none passes, the smallest share tried is
said, with why.

Changelog
  2026-11-12a  first version (Mission Control build)
"""
import argparse
import csv
import glob
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
import threading
import time

VERSION = "share-bench-2026-11-12a"
HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for _p in (HERE, os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "detector"), os.path.join(TOOLS, "experiment")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

ASK_EVERY_S = 2.0        # chosen: a lost robot is asked for this often (tracking_layer's on-demand rate for one robot is
                         # ML_ROBOT_HZ = 2 a second at most; one ask in flight at a time here). DOE: no (the bench's pace)
FOUND_CM = 5.0           # chosen: a detection this close to a parked spot has found that robot (the detector's centre
                         # error is 3.7 cm at 95 %, detector-v1 report). DOE: no
GIVE_S = 20.0            # chosen: an ask not answered in this long is counted as "not found in 20 s" (kept in the table,
                         # never dropped). DOE: no
DEFAULT_SHARES = (10, 25, 50)


def pct(xs, q):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    return xs[min(len(xs) - 1, int(q * (len(xs) - 1) + 0.5))]


def med(xs):
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else None


# ------------------------------------------------------------------------------------------------ the detector loop
class Finder:
    """Asks a DetectorSource for one parked robot at a time, as the tracking layer asks for a lost one; measures the
    time to find it. spots: [(x, y)] parked robots (learnt when None)."""

    def __init__(self, source, spots=None, clock=time.time):
        self.src, self.spots, self.clock = source, spots, clock
        self.seen = []                  # every detection (x, y) while learning
        self.finds = []                 # seconds from ask to found (GIVE_S + when not found)
        self.k = 0
        self.ask_t = None
        self.target = None

    def learn(self, dets):
        self.seen += [(d.x, d.y) for d in dets]

    def spots_from_seen(self, min_n=3):
        """Cluster the learning detections (greedy, FOUND_CM): each cluster seen min_n times or more is a parked robot."""
        cl = []
        for x, y in self.seen:
            for c in cl:
                if math.hypot(x - c[0] / c[2], y - c[1] / c[2]) <= FOUND_CM:
                    c[0] += x
                    c[1] += y
                    c[2] += 1
                    break
            else:
                cl.append([x, y, 1])
        return [(c[0] / c[2], c[1] / c[2]) for c in cl if c[2] >= min_n]

    def step(self):
        now = self.clock()
        if not self.spots:
            self.learn(self.src("bench", now, "learn", None))
            return
        if self.target is None or now - self.ask_t >= GIVE_S:
            if self.target is not None:
                self.finds.append(GIVE_S)
            self.target = self.spots[self.k % len(self.spots)]
            self.k += 1
            self.ask_t = now
        tx, ty = self.target
        dets = self.src("bench", now, "lost", (tx, ty, 30.0))
        if any(math.hypot(d.x - tx, d.y - ty) <= FOUND_CM for d in dets):
            self.finds.append(now - self.ask_t)
            self.target = None


def make_source(share, model_path=None, calib=None, grab=None):
    """The real thing: detector-v1 on the recorder's newest overhead picture, through the overhead calibration, capped
    at share (0-1), as tagfree.for_runner builds it for a run."""
    import tagfree as TF
    import infer as INF
    import e02_camera as CAM
    import mission_control as MC
    model = INF.load(model_path or TF.find_model(), threads=1)
    oc = CAM.load_camera(calib)
    grab = grab or (lambda: (time.time(), MC.grab_jpeg("127.0.0.1", 8000)))

    def pic():
        t, j = grab()
        return (t, j) if j else None
    cam = TF.Camera("overhead", pic, TF.camera_model_to_floor(oc, TF.Z_OVERHEAD_CM), True, TF.SIGMA_OVERHEAD_CM)
    return TF.DetectorSource(model, [cam], sync=True, budget=TF.Budget(duty=share))


# ------------------------------------------------------------------------------------------------ one share
def run_share(out, share_pct, minutes, finder_spots=None, source_factory=make_source, practice=True, dry=False):
    """One share for `minutes`: the practice load, the monitor, the detector loop pinned to the planner's core."""
    import pi_monitor as PM
    d = os.path.join(out, "share-%02d" % share_pct)
    os.makedirs(d, exist_ok=True)
    secs = minutes * 60.0
    procs = []
    if practice and not dry:
        cmd = [sys.executable, os.path.join(TOOLS, "fleet", "freeroam.py"), "run", "--dry", "--hours",
               "%.3f" % (minutes / 60.0 + 0.05), "--robots", "Left,Center,Right", "--sim-minutes", "%.1f" % minutes,
               "--dir", os.path.join(d, "practice")]
        pin = planner_pin()
        procs.append(subprocess.Popen(pin + cmd, cwd=REPO, stdout=open(os.path.join(d, "practice.txt"), "w"),
                                      stderr=subprocess.STDOUT, start_new_session=True,
                                      env=dict(os.environ, MIRLT_PI_MONITOR="0")))
    stop = threading.Event()
    mon = threading.Thread(target=lambda: PM.watch(d, pid=os.getpid(), every_s=PM.EVERY_S, max_s=secs,
                                                   sampler=Sampler2(d)), daemon=True)
    mon.start()
    finder = None
    if share_pct > 0:
        try:
            pin_self(planner_pin())
            finder = Finder(source_factory(share_pct / 100.0), spots=finder_spots)
        except Exception as e:  # noqa: BLE001
            with open(os.path.join(d, "detector_error.txt"), "w") as f:
                f.write(str(e))
    t0 = time.time()
    while time.time() - t0 < secs:
        if finder is not None:
            finder.step()
            if not finder.spots and time.time() - t0 > 60.0:
                finder.spots = finder.spots_from_seen()
                with open(os.path.join(out, "spots.json"), "w") as f:
                    json.dump(finder.spots, f)
        time.sleep(1.0 / 20 if finder is not None else 0.5)
    stop.set()
    for p in procs:
        try:
            os.killpg(p.pid, 2)
            p.wait(timeout=30)
        except Exception:  # noqa: BLE001
            p.kill()
    mon.join(timeout=10)
    res = {"share_pct": share_pct, "minutes": minutes, "finds_s": finder.finds if finder else [],
           "spots": finder.spots if finder else None,
           "detector": finder.src.summary() if finder is not None and hasattr(finder.src, "summary") else None}
    with open(os.path.join(d, "finds.json"), "w") as f:
        json.dump(res, f, indent=1, default=str)
    return res


class Sampler2:
    """pi_monitor's Sampler, with the practice free roam's feed (the planner's loop) in <share>/practice."""

    def __init__(self, d):
        import pi_monitor as PM
        self.s = PM.Sampler(os.path.join(d, "practice"))
        self.d = d

    def row(self):
        os.makedirs(os.path.join(self.d, "practice"), exist_ok=True)
        return self.s.row()


def planner_pin():
    try:
        import core_layout as CL
        c = CL.cores("planner")
    except Exception:  # noqa: BLE001
        c = ""
    return ["taskset", "-c", c] if c and shutil.which("taskset") else []


def pin_self(pin):
    if pin:
        try:
            os.sched_setaffinity(0, {int(x) for x in pin[2].split(",")})
        except (AttributeError, OSError, ValueError):
            pass


# ------------------------------------------------------------------------------------------------ the choice
def share_stats(d):
    import pi_monitor as PM
    rows = PM.rows_of(os.path.join(d, PM.NAME))
    f = PM._f
    try:
        with open(os.path.join(d, "finds.json")) as fh:
            fd = json.load(fh)
    except (OSError, ValueError):
        fd = {}
    finds = fd.get("finds_s") or []
    lag = [f(r.get("lag_overhead_s")) for r in rows]
    loop = [f(r.get("loop_work_ms_p95")) for r in rows]
    return {"rows": len(rows), "lag_med": med(lag), "lag_p95": pct(lag, 0.95), "loop_med": med(loop),
            "loop_p95": pct(loop, 0.95), "core1_med": med([f(r.get("core1")) for r in rows]),
            "temp_max": max([x for x in (f(r.get("temp_c")) for r in rows) if x is not None], default=None),
            "throttle": sorted({r.get("throttle_now") for r in rows if r.get("throttle_now")}),
            "find_med": med(finds), "find_p95": pct(finds, 0.95), "asks": len(finds),
            "not_found": sum(1 for x in finds if x >= GIVE_S),
            "det_ms_med": ((fd.get("detector") or {}).get("load") or {}).get("ms_median")}


def choose(stats):
    """stats: {share_pct: share_stats}. The baseline (0) gives the normal spread. Returns (share or None, why, table)."""
    base = stats.get(0)
    if not base or base["lag_p95"] is None and base["loop_p95"] is None:
        return None, "no baseline (share 0) readings: the bench could not judge", {}
    verdict = {}
    for s, st in sorted(stats.items()):
        if s == 0:
            continue
        why = []
        if base["lag_p95"] is not None and (st["lag_med"] is None or st["lag_med"] > base["lag_p95"]):
            why.append("picture lag %s s over the baseline's 95 %% (%.2f s)" % (_g(st["lag_med"]), base["lag_p95"]))
        if base["loop_p95"] is not None and (st["loop_med"] is None or st["loop_med"] > base["loop_p95"]):
            why.append("planner loop %s ms over the baseline's 95 %% (%.1f ms)" % (_g(st["loop_med"]),
                                                                                base["loop_p95"]))
        new_thr = [t for t in st["throttle"] if t not in base["throttle"]]
        if new_thr:
            why.append("throttling: " + ", ".join(new_thr))
        verdict[s] = (not why, "; ".join(why) or "inside the normal spread")
    ok = [s for s, (p, _) in verdict.items() if p]
    if ok:
        return max(ok), "the largest share whose picture lag and loop time stay inside the baseline's spread", verdict
    return None, "no share stayed inside the baseline's spread: keep the smallest (%d %%) and look at why" % min(
        verdict) if verdict else "no shares tried", verdict


def _g(v):
    return "-" if v is None else ("%.2f" % v if abs(v) < 10 else "%.0f" % v)


def report(out):
    stats = {}
    for d in sorted(glob.glob(os.path.join(out, "share-*"))):
        stats[int(os.path.basename(d).split("-")[1])] = share_stats(d)
    pick, why, verdict = choose(stats)
    L = ["# Detector core share bench (%s)" % os.path.basename(out.rstrip("/")), "",
         "| share of core 1 | picture lag median / 95 %% (s) | planner loop median / 95 %% (ms) | core 1 busy median %% | "
         "hottest C | throttling | time to find median / 95 %% (s) | not found in %.0f s | detector ms a call | verdict |"
         % GIVE_S, "|---|---|---|---|---|---|---|---|---|---|"]
    for s, st in sorted(stats.items()):
        v = "baseline (detector off)" if s == 0 else ("PASS" if verdict.get(s, (False,))[0] else "fail: " +
                                                      verdict.get(s, (False, "?"))[1])
        L.append("| %d %% | %s / %s | %s / %s | %s | %s | %s | %s / %s | %d of %d | %s | %s |" % (
            s, _g(st["lag_med"]), _g(st["lag_p95"]), _g(st["loop_med"]), _g(st["loop_p95"]), _g(st["core1_med"]),
            _g(st["temp_max"]), ", ".join(st["throttle"]) or "none", _g(st["find_med"]), _g(st["find_p95"]),
            st["not_found"], st["asks"], _g(st["det_ms_med"]), v))
    L += ["", "**Chosen: %s.** %s." % ("%d %%" % pick if pick is not None else "none", why), ""]
    if pick is not None:
        L += ["To use it for every run: `python3 tools/pi/detector_share_bench.py apply %d`" % pick, ""]
    L += ["<sub>Measured on this Pi by tools/pi/detector_share_bench.py (pi_monitor.csv and finds.json in each share's "
          "folder). 95 %: 95 of 100 readings were at or under it. The baseline is the detector off under the same "
          "load.</sub>", ""]
    p = os.path.join(out, "bench.md")
    with open(p, "w") as f:
        f.write("\n".join(L))
    return p, pick


def apply(share_pct, path=None):
    import tagfree as TF
    p = os.path.expanduser(path or TF.SHARE_FILE)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, "w") as f:
        json.dump({"duty": round(float(share_pct) / 100.0, 4), "set": time.strftime("%Y-%m-%d %H:%M"),
                   "by": "detector_share_bench.py apply"}, f, indent=1)
    return p


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--minutes", type=float, default=5.0)
    r.add_argument("--shares", default=",".join(str(s) for s in DEFAULT_SHARES))
    r.add_argument("--out", default="")
    a_ = sub.add_parser("apply")
    a_.add_argument("share", type=float)
    rp = sub.add_parser("report")
    rp.add_argument("dir")
    a = ap.parse_args(argv)
    if a.cmd == "apply":
        print("Saved: every run now caps the detector at %g %% of one core (%s)." % (a.share, apply(a.share)))
        return 0
    if a.cmd == "report":
        p, _ = report(a.dir)
        print(open(p).read())
        return 0
    import mission_control as MC
    if MC.grab_jpeg("127.0.0.1", 8000) is None:
        print("The recorder is not running (no picture on port 8000). Start it first: ./run-fleet-pi.sh rec")
        return 1
    out = a.out or os.path.join(REPO, "logs", time.strftime("%Y%m%d-%H%M%S") + "-share-bench")
    os.makedirs(out, exist_ok=True)
    shares = [0] + [int(float(x)) for x in a.shares.split(",") if x.strip()]
    total = a.minutes * len(shares)
    print("Detector share bench: shares %s, %.0f min each (about %.0f min). Robots stay parked. Results: %s" % (
        ", ".join("%d %%" % s for s in shares), a.minutes, total, out), flush=True)
    spots = None
    for s in shares:
        print("  %s  share %d %% ..." % (time.strftime("%H:%M"), s), flush=True)
        res = run_share(out, s, a.minutes, finder_spots=spots)
        spots = spots or res.get("spots")
    p, pick = report(out)
    print(open(p).read())
    print("To send the results: copy the table above into the chat (the full numbers are in %s)." % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
