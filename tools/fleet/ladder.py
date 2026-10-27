#!/usr/bin/env python3
"""The remote ladder (MIRLT, 2026-10-27): Phase 0 run from the laptop over SSH, with nobody at the board.

Nick leaves the robots switched on, spread out on the plywood (no boards, every side an open edge), at least 30 cm
from the edges and 40 cm apart, where and facing whichever way they happen to be. Nobody can move them. His
instruction: "start cautious and grow increasingly more bold. I'll be available at my laptop to review as needed."

The steps, each run only after the one before it passed, each ending with PASS or FAIL and what to look at:

  L0  status    firmware and battery of every robot; the edge guard switched on; the camera's pictures arrive; each
                robot found by its lights (nothing moves). A robot within 25 cm of an edge is kept out.
  L1  calibrate each robot alone (the planner needs a calibration since power-on), after the camera checked it has
                room (30 cm to every edge, 35 cm to every other robot).
  L2  turns     the turn check (replaces the ruler): the camera measures four `turn 90`s. Headings come from short
                out-and-back crawls the camera watches (one still picture cannot tell a square robot's heading).
                A robot whose turns are off is kept out of every later step (it stays parked: the others go round it).
  L3  straight  each robot that passed L2: the planner drives it 25 cm straight out and back, slowly, with camera
                fixes (the first time run_plan drives it); the line check gate.
  L4  square    test one (b1): one robot is planned from where it stands to the square's start spot (the others
                parked), driven there with camera fixes, then drives the square twice: first slowly, then normally.
  L5  cross     test cross (b2): two robots driven to their starts, then cross four times: first slowly with wide
                spacing, then at normal speed and spacing.
  L6  brick     (attended only: someone at the board) test cross with the brick watch on.

    python3 tools/fleet/ladder.py run [--from L2] [--pause]       the ladder from the first step not yet passed
    python3 tools/fleet/ladder.py step L2 [--robots Left]         one step (its predecessor must have passed)
    python3 tools/fleet/ladder.py turncheck Left                  the camera-measured turn check alone (bench a2)
    python3 tools/fleet/ladder.py status                          what has passed so far, where the robots are
    python3 tools/fleet/ladder.py stop                            (dry runs) stop the simulated board

run-fleet-pi.sh calls it (`./run-fleet-pi.sh ladder`, `./run-fleet-pi.sh L2`, ...). Ctrl+C (or a dropped SSH link)
stops every robot. Everything is kept in logs/<date>-ladder/ (state.json: what passed, where each robot is; one
folder per step). --dry runs it all against the simulator (tools/sim/fleet_sim.py --persist --serve-live: the
simulated overhead camera, robots at random spots and headings; --turn-err for a robot whose turns are off).
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "live"),
          os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.append(p)

import numpy as np   # noqa: E402

PY = sys.executable

# ---------------------------------------------------------------------------------------------- chosen numbers
EDGE_REFUSE_CM = 25.0   # Nick (2026-10-27): the turn check refuses a robot the camera puts within 25 cm of an edge;
                        # used for every step: a robot nearer than this is never moved today. No DOE: a safety rule
CALIB_EDGE_CM = 30.0    # calibration room: the session plan's "30 cm in from every edge" (its roll is ~35 cm from
CALIB_GAP_CM = 35.0     # its back, the edge guard stops it at a plywood edge). No DOE: the firmware's own needs
CRAWL_CM = 10.0         # chosen: a heading crawl's length, forward then back to where it was (e02_back_max_cm is
                        # 10 cm: backing is blind to the edge guard). DOE note: 6 / 10 cm (camera heading spread on a
                        # still-standing robot's repeated crawls against the time it takes); docs/doe-backlog.md
CRAWL_CMS = 5.0         # chosen: crawl speed (e02_cms; half the route speed): more pictures per cm. No DOE: the
                        # line fit uses every picture, the speed barely matters to it
TURN_TOL_DEG = 6.0      # chosen: each of the four camera-measured quarter turns within 6 deg of 90 (the ruler's
                        # "about 5 deg" gate plus the camera's heading spread, ~1 deg in the dry runs). DOE note: on
                        # the board, 4 / 6 / 8 deg against repeated checks of a healthy robot (false FAILs) and the
                        # turn error that later breaks the line check (L3/L4 p95); docs/doe-backlog.md
TOTAL_TOL_DEG = 8.0     # chosen: after the four turns it faces within 8 deg of where it started. DOE: as above
SLOW_CMS = 5.0          # chosen: the cautious runs' speed (L3, L4 slow, L5 slow, drives to a start spot): half the
                        # planner's 10 cm/s. DOE note: 5 / 7.5 / 10 cm/s against the line check's p95 and the camera
                        # guard's stops (the 10 cm/s runs are the ladder's last rung anyway); docs/doe-backlog.md
NORMAL_CMS = 10.0       # the planner's own cruise_cms (planner.py DEFAULT_SETTINGS)
OUTBACK_CM = 25.0       # chosen: L3's straight out-and-back (one 15 cm step and one 10 cm step each way, as the
                        # planner chunks it). No DOE: long enough for a travel heading from the camera (5 cm)
WIDE_SEP_CM = 10.0      # chosen: L5 slow: the planner keeps the two bodies at least this far apart, twice its default
                        # separation_cm of 5 (20 was tried: the smaller crosses' start spots are only ~15 cm apart, so
                        # no plan exists). DOE note: 5 / 10 / 15 cm (who-is-who unsure moments against the closest
                        # the camera saw them); docs/doe-backlog.md
LINE_CLEAR_CM = 20.0    # chosen: a test's planned lines and spots keep a parked robot's centre this far away
                        # (bodies about 10 cm apart). No DOE: a safety room, any size that fits works
LAYOUT_MARGINS = (30.0, 25.0)   # test spots and lines this far in from every edge (30 as the Phase 0 maps; 25 if
                                # nothing fits at 30). No DOE: the camera spot rides onto the nose within ~15 cm
SQUARE_SIDES = (62.0, 52.0, 42.0, 32.0)      # test one's square: 62 cm as the Phase 0 map, smaller if the parked
CROSS_SIZES = ((62.0, 52.0), (52.0, 44.0), (42.0, 36.0))   # robots leave no room. Cross: along x across
LEAD_IN_CM = 15.0       # chosen: a test starts this far along its first line from the corner, and the drive there
                        # visits the corner first, so its last leg is straight along the start heading and it barely
                        # turns at the end (dry run 2026-10-27: a final turn of 163 deg on the spot swung the robot's
                        # centre 4 cm round its axle and the start check refused it). One planner step (chunk_cm).
                        # No DOE: any length from ~10 cm (a camera heading) to the side's length works
START_TOL_CM = 4.0      # the start-spot check after a drive (the launcher's --place-tol-cm for a taped spot)
START_TOL_DEG = 6.0     # chosen: run_plan's 5 deg plus the heading's spread from one 15 cm leg (dry runs: ~1 deg).
                        # The test's first straight drive corrects the rest (a 10 deg start passed). No DOE: a check
GATE_CM = 3.0           # line check: each robot's 95th percentile within 3 cm of its planned lines (Phase 0's gate)
MIN_BATTERY = 25        # chosen: below this a robot is kept out (a run at 20 % sagged on 2026-10-25). No DOE
MIN_FIRMWARE = "2026-10-18b"
OBSTACLE_R_CM = 10.0    # chosen: a parked robot on the planner's map: a circle round its corners (half diagonal 6.7 cm)
                        # plus 3 cm (dry run 2026-10-27 with 7 cm: a drive passed the parked Center 6.8 cm body to
                        # body). No DOE: a safety room
SAFE = ["set safe_edge 3", "set edge_dir 0", "set edge_thr 150", "set edge_n 3", "set edge_bot_cm 11",
        "set bumper_cm 0"]   # the robots' edge guard, as run-fleet-pi.sh / e02.py (not kept over a power-off)
# The stored driving corrections (2026-10-27 13:58, real L2: every robot turned +5 to +10 deg too far on each quarter
# turn, about 7.5 %, because the ladder left them on factory values; the e02 runner has always sent its prior,
# e02_fit.PRIOR: the robots' gyros under-read by ~7.5 % and their wheels over-read ~2 %). Sent with the edge guard on
# every connect (not kept over a power-off). A per-robot file ~/.config/mirlt/corrections.json
# {"Left": {"gyro_scale": .., "wheel_scale": .., "gyro_drift_dps": ..}} overrides it when present.
CORR_FILE = os.path.expanduser(os.environ.get("MIRLT_CONF_DIR", "~/.config/mirlt") + "/corrections.json")


def corrections_for(name):
    try:
        sys.path.insert(0, os.path.join(TOOLS, "experiment"))
        import e02_fit as FIT
        c = dict(FIT.PRIOR)
    except Exception:
        c = dict(gyro_scale=1.075, gyro_drift_dps=0.0, wheel_scale=0.979)
    try:
        with open(CORR_FILE) as f:
            c.update((json.load(f) or {}).get(name) or {})
    except (OSError, ValueError):
        pass
    return c


def corrections_cmds(name):
    c = corrections_for(name)
    return ["set gyro_scale %.4f" % c["gyro_scale"], "set gyro_drift_dps %.4f" % c["gyro_drift_dps"],
            "set wheel_scale %.4f" % c["wheel_scale"]]
STEPS = ["L0", "L1", "L2", "L3", "L4", "L5", "L6"]
TITLES = {"L0": "status: firmware, battery, edge guard, camera, every robot found by its lights",
          "L1": "calibrate each robot alone (the camera checks its room first)",
          "L2": "turn check: four quarter turns measured by the camera",
          "L3": "one robot at a time: 25 cm straight out and back, slowly, planned and with camera fixes",
          "L4": "test one (b1): drive to the square's start, the square twice, slow then normal speed",
          "L5": "test cross (b2): both to their starts, cross 4 times, slow with wide spacing then normal",
          "L6": "brick (b3): test cross with the brick watch on (attended only)"}
PREFER = ["Left", "Right", "Center"]


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def say(s=""):
    print(s, flush=True)


class LadderStop(Exception):
    """A step that cannot go on: the words say why and what to decide."""


# ---------------------------------------------------------------------------------------------------- the camera
class Eye:
    """The overhead camera's pictures (the recorder's /live) in a thread, the side camera's too when it is calibrated
    (two-camera tracking, 2026-10-27: e02_twocam.TwoCamTracker), a tracker on every robot, lights finding.

    Reading and tracking are apart (real run 2026-10-27 13:01: the reading thread also tracked every picture, ~23 a
    second from both cameras, and on the Pi it fell seconds behind, so `still` never found 4 recent sightings and all
    three robots were reported unseen): one reading thread per camera keeps only the NEWEST picture; one tracking
    thread per camera takes the newest picture when it is due (OVERHEAD_HZ / e02_twocam.SIDE_HZ) and drops the ones it
    had no time for. Every tracked picture's cost and how far behind it was go to tracking_times.csv in the step
    folder (log_to)."""
    OVERHEAD_HZ = 12.0      # chosen: overhead pictures tracked a second (the camera sends ~21-25). At 10 cm/s, the
                            # ladder's fastest leg, that is under 1 cm between pictures; Pi budget: 12 x ~45 ms = about
                            # half of one core. DOE note: 8 / 12 / 16 a second against the Pi's measured cost and lag
                            # (tracking_times.csv) and the line check's p95
    BEHIND_S = 0.5          # tracking more than this behind the newest picture is reported in plain words

    def __init__(self, stream, calib, rect, side_stream=None, side_calib=None, lights=True):
        import e02_camera as CAM
        import sources as S
        try:
            import scipy.optimize  # noqa: F401  (loaded now, not inside the first tracked picture: ~0.6 s here, more on the Pi)
        except Exception:
            pass
        self.CAM = CAM
        self.cam = CAM.load_camera(calib)
        self.rect = rect
        self.frames = deque(maxlen=90)
        self.trk = None
        self.lock = threading.Lock()
        self.cv = threading.Condition()
        self.closed = False
        self.n = 0
        self.lights = lights
        self.side_cam, self.side_lag, self.side_bias = None, None, (0.0, 0.0)
        self.side_frames = deque(maxlen=30)
        self.side_n = 0
        self.newest = {"overhead": None, "side": None}       # (img, t, t_arrived, n)
        self.done = {"overhead": None, "side": None}         # (t, t_arrived) of the last picture tracked
        self.cost = {"overhead": deque(maxlen=300), "side": deque(maxlen=300)}   # (cost ms, behind s, dropped)
        self.tlog = None
        self.tlog_lock = threading.Lock()
        self.side_words = "no side camera calibration: the overhead camera alone (and the robots' lights)"
        if side_calib and side_stream:
            import e02_twocam as TW
            self.side_cam, self.side_lag, self.side_bias = TW.load_side(side_calib)
            self.side_words = "side camera %s (lag %.3f s), calibration %s" % (side_stream, self.side_lag, side_calib)
            self.side_src = S.StreamSource(side_stream, log=lambda s: None)
            threading.Thread(target=self._run_side, daemon=True).start()
            threading.Thread(target=self._track, args=("side",), daemon=True).start()
        self.src = S.StreamSource(stream, log=lambda s: None)
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()
        threading.Thread(target=self._track, args=("overhead",), daemon=True).start()

    def _run(self):
        try:
            for fr in self.src:
                if self.closed:
                    break
                t = fr.t_pi - 0.04
                with self.lock:
                    self.frames.append((fr.img, t))
                    self.n += 1
                with self.cv:
                    self.newest["overhead"] = (fr.img, t, fr.t_arrived, self.n)
                    self.cv.notify_all()
        except Exception as e:      # the stream ended
            self.error = str(e)

    def _run_side(self):
        try:
            for fr in self.side_src:
                if self.closed:
                    break
                t = fr.t_pi - 0.04
                with self.lock:
                    self.side_frames.append((fr.img, t))
                    self.side_n += 1
                with self.cv:
                    self.newest["side"] = (fr.img, t, fr.t_arrived, self.side_n)
                    self.cv.notify_all()
        except Exception as e:
            self.side_error = str(e)

    def _track(self, cam):
        """Tracks the newest picture of camera cam when one is due; the ones in between are dropped."""
        import e02_twocam as TW
        every = 1.0 / (self.OVERHEAD_HZ if cam == "overhead" else TW.SIDE_HZ)
        last_t, last_n, seen_t = None, None, None
        while not self.closed:
            with self.cv:
                self.cv.wait_for(lambda: self.closed or (self.newest[cam] is not None
                                                         and self.newest[cam][1] not in (last_t, seen_t)), timeout=0.5)
                item = self.newest[cam]
            trk = self.trk
            if self.closed or item is None or item[1] in (last_t, seen_t):
                continue
            img, t, t_arr, n = item
            if trk is None:
                last_t, last_n = t, n
                continue
            if last_t is not None and t - last_t < 0.8 * every:
                # not due yet: wait for the next picture (0.8: a camera sending 25 a second gives every 2nd picture at
                # the 12-a-second cap, 12.5 a second, not every 3rd)
                seen_t = t
                continue
            c0 = time.perf_counter()
            try:
                if cam == "overhead":
                    trk.push(img, t)
                else:
                    trk.side_t = None                             # the rate is kept here (push_side's own gate off)
                    trk.push_side(img, t)
            except Exception as e:                                # a tracking error must not kill the thread
                self.track_error = "%s: %s" % (cam, e)
            cost = (time.perf_counter() - c0) * 1000.0
            behind = time.time() - t_arr                          # this machine's clock only: waited + tracked
            dropped = 0 if last_n is None else max(0, n - last_n - 1)
            last_t, last_n = t, n
            self.done[cam] = (t, t_arr)
            self.cost[cam].append((cost, behind, dropped))
            with self.tlog_lock:
                if self.tlog is not None:
                    try:
                        self.tlog.write("%s,%.3f,%.3f,%.1f,%.3f,%d\n" % (cam, t, time.time(), cost, behind, dropped))
                    except (OSError, ValueError):
                        self.tlog = None

    def log_to(self, folder):
        """Every tracked picture from now on: camera, picture time, done time, cost ms, behind s, pictures dropped."""
        with self.tlog_lock:
            if self.tlog is not None:
                try:
                    self.tlog.close()
                except OSError:
                    pass
            self.tlog = None
            try:
                new = not os.path.exists(os.path.join(folder, "tracking_times.csv"))
                self.tlog = open(os.path.join(folder, "tracking_times.csv"), "a", buffering=1)
                if new:
                    self.tlog.write("camera,t_picture,t_done,cost_ms,behind_s,dropped\n")
            except OSError:
                self.tlog = None

    def tracking_words(self):
        """One line per camera: pictures tracked a second, cost a picture (median / 90th percentile), how far behind."""
        out = []
        for cam in ("overhead", "side"):
            c = list(self.cost[cam])[-120:]
            if len(c) < 3:
                continue
            ms = np.array([q[0] for q in c])
            bh = np.array([q[1] for q in c])
            dr = sum(q[2] for q in c)
            out.append("%s camera tracking: %.0f ms a picture (90%%: %.0f ms), %.2f s behind at most, %d pictures "
                       "dropped of %d" % (cam, np.median(ms), np.percentile(ms, 90), float(bh.max()), dr, dr + len(c)))
        return out

    def behind(self):
        """Seconds the overhead tracking is behind the newest picture it was given (0 when caught up)."""
        nw, d = self.newest["overhead"], self.done["overhead"]
        if nw is None:
            return 0.0
        if d is None:
            return time.time() - nw[2] if self.trk is not None else 0.0
        return max(0.0, nw[1] - d[0])

    def trk_time(self):
        """The time of the newest picture the tracker has worked through (either camera), or None."""
        ts = [d[0] for d in self.done.values() if d is not None]
        return max(ts) if ts else None

    def close(self):
        self.closed = True
        with self.cv:
            self.cv.notify_all()
        with self.tlog_lock:
            if self.tlog is not None:
                try:
                    self.tlog.close()
                except OSError:
                    pass
                self.tlog = None
        self.src.closed = True
        if getattr(self, "side_src", None) is not None:
            self.side_src.closed = True

    def rate(self, s=2.0):
        n0, m0 = self.n, self.side_n
        time.sleep(s)
        self.side_rate = (self.side_n - m0) / s
        return (self.n - n0) / s

    def collect(self, s):
        t0 = time.time()
        time.sleep(s)
        with self.lock:
            return [im for im, t in self.frames if t >= t0 - 0.05]

    def latest(self, k=5):
        with self.lock:
            return [im for im, _ in list(self.frames)[-k:]]

    def latest_side(self, k=5):
        with self.lock:
            return [im for im, _ in list(self.side_frames)[-k:]]

    def find_lights(self, bench, name, others=(), tries=2, keeper=None):
        """Its lights (`led 1 1 1 2500`, steady white): the one spot that got brighter. (x, y) of the lights (near
        its front) or None. keeper: the ladder's lights keeper, paused meanwhile (its steady lights would leave nothing
        to change), and this robot's lights put out first."""
        if keeper is not None:
            keeper.pause(True)
        try:
            for _ in range(tries):
                if keeper is not None:
                    bench.send(name, "led 0 0 0 3000")      # dark before (the keeper had them on)
                    time.sleep(0.4)
                before = self.collect(0.8)
                bench.send(name, "led 1 1 1 2500")
                time.sleep(0.5)
                during = self.collect(1.0)
                time.sleep(1.3)
                if len(before) < 2 or len(during) < 2:
                    continue
                f = self.CAM.locate_light(self.cam, before, during, board_rect=self.rect, margin_cm=8.0)
                if f and all(math.hypot(f["x"] - o[0], f["y"] - o[1]) > 12.0 for o in others):
                    return f["x"], f["y"]
            return None
        finally:
            if keeper is not None:
                keeper.pause(False)

    def start_tracking(self, spots):
        """Floor memory from the last pictures (each camera's) with every robot's floor filled in from around it; a
        track on each."""
        import e02_twocam as TW
        cx, cy = (self.rect[0] + self.rect[2]) / 2.0, (self.rect[1] + self.rect[3]) / 2.0
        side_ok = self.side_cam is not None and len(self.side_frames) >= 3
        trk = TW.TwoCamTracker(self.cam, board_center=(cx, cy), board_rect=self.rect, margin_cm=8.0,
                               side_cam=self.side_cam if side_ok else None, side_lag_s=self.side_lag or TW.SIDE_LAG_S,
                               lights=self.lights, side_bias=self.side_bias)
        fr = self.latest(5)
        trk.set_background(fr, robots=[(x, y) for x, y in spots.values()])
        if side_ok:
            trk.set_side_background(self.latest_side(5), robots=[(x, y) for x, y in spots.values()])
        for n, (x, y) in spots.items():
            trk.add(n, x, y)
        self.done = {"overhead": None, "side": None}
        self.trk = trk
        time.sleep(0.8)

    def still(self, name, s=0.8, timeout=6.0, spread_cm=1.2, body_only=False):
        """Where the camera sees it standing still: the median of its sure spots over the last s, or None. body_only:
        only spots where a camera saw its body (not its lights alone): L0's check."""
        t_end = time.time() + timeout
        self.still_why = ""
        while time.time() < t_end:
            ref = self.trk_time()           # the tracker's own newest picture: a slow tracker is not "unseen"
            h = self.trk.history(name, ref - s - 0.1, ref, body_only=body_only) if self.trk and ref else []
            if len(h) >= 4 and h[-1][0] - h[0][0] >= 0.6 * s:
                xs, ys = np.array([q[1] for q in h]), np.array([q[2] for q in h])
                mx, my = float(np.median(xs)), float(np.median(ys))
                if float(np.max(np.hypot(xs - mx, ys - my))) <= spread_cm:
                    return mx, my
            time.sleep(0.15)
        b = self.behind()
        if b > self.BEHIND_S:
            self.still_why = "camera tracking is %.1f s behind the camera" % b
        return None

    def seen_by(self, name, s=1.5):
        """Which camera saw it over the last s: 'both', 'overhead', 'side', 'lights' (the most frequent), or None."""
        if not self.trk:
            return None
        with self.trk.lock:
            tr = self.trk.tracks.get(name)
            ref = self.trk_time() or time.time()
            hs = [q[4] for q in tr.hist if q[0] >= ref - s and len(q) > 4 and q[4]] if tr else []
        return max(set(hs), key=hs.count) if hs else None


class LightsKeeper(threading.Thread):
    """Keeps every robot's lights on (`led 1 1 1 20000`, renewed every RENEW_S) while the ladder watches between its
    steps, so the lights can stand in for a body neither camera sees (e02_twocam). The robots ignore `led` while a
    run is going (their blink codes then do the same job)."""
    RENEW_S = 12.0          # chosen: well inside the firmware's 20 s cap on one `led`. No DOE: a renewal period

    def __init__(self, bench, names):
        super().__init__(daemon=True)
        self.bench, self.names = bench, list(names)
        self.paused = threading.Event()
        self.stopping = False
        self.next_t = 0.0

    def pause(self, on):
        if on:
            self.paused.set()
        else:
            self.paused.clear()
            self.next_t = 0.0

    def run(self):
        while not self.stopping:
            if not self.paused.is_set() and time.time() >= self.next_t:
                for n in self.names:
                    try:
                        self.bench.send(n, "led 1 1 1 20000")
                    except Exception:
                        pass
                self.next_t = time.time() + self.RENEW_S
            time.sleep(0.2)

    def stop(self):
        self.stopping = True


# ------------------------------------------------------------------------------------------------- the robots
class Bench:
    """The WiFi link (tools/alvik_console.py's Link): commands, and the robots' answers as they come."""

    def __init__(self, ips, outdir):
        import alvik_console as AC
        import re
        self.re_fw = re.compile(r"status (\S+) \(firmware ([^)]+)\).*battery (\d+)%")
        self.link = AC.Link(outdir, ips)
        self.link.session.quiet = True
        self.lock = threading.Lock()
        self.fw = {}
        self.legs = {}          # name -> [(t, kind, value, result, x, y, th, info)]
        self.runs = {}          # name -> {run_id: exp}
        self.runend = {}        # name -> (exp, result, t)
        self.calib_look = {}
        self.msgs = deque(maxlen=400)
        self.link.session.listeners.append(self._listen)

    def _listen(self, name, kind, rtype, vals):
        t = time.time()
        with self.lock:
            if kind == "M":
                text = vals if isinstance(vals, str) else ",".join(vals)
                self.msgs.append((t, name, text))
                m = self.re_fw.search(text)
                if m:
                    self.fw[m.group(1)] = (m.group(2), int(m.group(3)))
                return
            if rtype == "LEG" and len(vals) >= 10:
                try:
                    self.legs.setdefault(name, []).append((t, vals[2], float(vals[3]), vals[4], float(vals[6]),
                                                           float(vals[7]), float(vals[8]), ",".join(vals[9:])))
                except ValueError:
                    pass
            elif rtype == "RUNINFO" and len(vals) >= 2:
                self.runs.setdefault(name, {})[vals[0]] = vals[1]
            elif rtype == "RUNEND" and len(vals) >= 2:
                self.runend[name] = (self.runs.get(name, {}).get(vals[0]), vals[1], t)
            elif rtype == "CALIB_LOOK" and len(vals) >= 10:
                lk = dict(result=vals[9])
                if len(vals) >= 15:
                    lk.update(spin=vals[12], roll=vals[13], reason=vals[14])
                self.calib_look[name] = lk

    def robots(self):
        return self.link.robots()

    def send(self, name, cmd):
        return self.link.send_name(name, cmd)

    def wait(self, cond, s):
        t_end = time.time() + s
        while time.time() < t_end:
            if cond():
                return True
            time.sleep(0.1)
        return cond()

    def leg(self, name, text, timeout):
        """An idle leg (`go <cm>` / `turn <deg>`): sends it, waits for its end. (result, x, y, th, info)."""
        kind = text.split()[0]
        t0 = time.time()
        n0 = len(self.legs.get(name, []))
        self.send(name, text)

        def recs():
            with self.lock:
                return [r for r in self.legs.get(name, [])[n0:] if r[1] == kind]
        if not self.wait(lambda: recs(), 1.5):
            self.send(name, text)           # nothing heard: lost on the way (an answer lost would show `started`)
        if not self.wait(lambda: any(r[3] in ("done", "failed", "refused", "dup") for r in recs()), timeout):
            return ("timeout", None, None, None, "no end of the leg in %.0f s" % (time.time() - t0))
        r = [q for q in recs() if q[3] in ("done", "failed", "refused", "dup")][-1]
        return (r[3], r[4], r[5], r[6], r[7])

    def close(self):
        self.link.close()       # sends stop to every robot


# -------------------------------------------------------------------------------------------------- geometry
def seg_dist(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    f = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - ax - f * dx, p[1] - ay - f * dy)


def edge_dist(rect, x, y):
    return min(x - rect[0], rect[2] - x, y - rect[1], rect[3] - y)


def edge_words(rect, x, y):
    d = {"x = %.0f" % rect[0]: x - rect[0], "x = %.0f" % rect[2]: rect[2] - x,
         "y = %.0f" % rect[1]: y - rect[1], "y = %.0f" % rect[3]: rect[3] - y}
    k = min(d, key=d.get)
    return "%.0f cm from the %s edge" % (d[k], k)


def line_fit_heading(pts, p_from, p_to):
    """Direction of the straight line through the camera's spots (principal axis), pointing from p_from to p_to."""
    a = np.array([[q[1], q[2]] for q in pts], float)
    if len(a) < 5:
        return None
    c = a - a.mean(0)
    _, _, vt = np.linalg.svd(c, full_matrices=False)
    d = vt[0]
    if d[0] * (p_to[0] - p_from[0]) + d[1] * (p_to[1] - p_from[1]) < 0:
        d = -d
    return math.degrees(math.atan2(d[1], d[0]))


def segs_clear(segs, pts, need):
    """The least distance of the points from the segments (inf when none)."""
    best = float("inf")
    for p in pts:
        for a, b in segs:
            best = min(best, seg_dist(p, a, b))
    return best


def square_layouts(rect, parked, mover_at):
    """Test one's square, the largest that fits: corners LAYOUT_MARGINS in, every side and corner LINE_CLEAR_CM from
    every parked robot. Start: the corner nearest the robot, facing along a side. Returns a list, best first."""
    out = []
    for s in SQUARE_SIDES:
        for margin in LAYOUT_MARGINS:
            xs = np.arange(rect[0] + margin, rect[2] - margin - s + 0.01, 2.0)
            ys = np.arange(rect[1] + margin, rect[3] - margin - s + 0.01, 2.0)
            for x0 in xs:
                for y0 in ys:
                    c = [(x0, y0), (x0, y0 + s), (x0 + s, y0 + s), (x0 + s, y0)]
                    segs = [(c[k], c[(k + 1) % 4]) for k in range(4)]
                    if segs_clear(segs, parked, LINE_CLEAR_CM) < LINE_CLEAR_CM:
                        continue
                    mid = (rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0
                    off = math.hypot(x0 + s / 2 - mid[0], y0 + s / 2 - mid[1])
                    k = min(range(4), key=lambda i: math.hypot(c[i][0] - mover_at[0], c[i][1] - mover_at[1]))
                    out.append((-s, margin != LAYOUT_MARGINS[0], off, dict(side=s, margin=margin, corners=c, start=k)))
        if out:
            break                       # the biggest square that fits, at whichever margin
    out.sort(key=lambda q: q[:3])
    return [q[3] for q in out]


def square_segs(lay):
    c = lay["corners"]
    return [(c[k], c[(k + 1) % 4]) for k in range(4)]


def holding_spots(rect, segs, blockers, keep_clear):
    """Where to drive robots that are in a test's way (healthy ones only): LINE_CLEAR_CM from the test's lines,
    LAYOUT_MARGINS[0] in from the edges, 30 cm from every other robot (keep_clear) and each other; nearest first.
    {name: (x, y)} or None when one has nowhere to go."""
    out, taken = {}, list(keep_clear)
    xs = np.arange(rect[0] + LAYOUT_MARGINS[0], rect[2] - LAYOUT_MARGINS[0] + 0.01, 2.0)
    ys = np.arange(rect[1] + LAYOUT_MARGINS[0], rect[3] - LAYOUT_MARGINS[0] + 0.01, 2.0)
    cand = [(float(x), float(y)) for x in xs for y in ys
            if segs_clear(segs, [(x, y)], 0) >= LINE_CLEAR_CM]
    for n, p in blockers.items():
        ok = [q for q in cand if all(math.dist(q, t) >= 30.0 for t in taken)]
        if not ok:
            return None
        q = min(ok, key=lambda c: math.dist(c, p))
        out[n] = q
        taken.append(q)
    return out


def square_map(rect, lay, name, parked_obs, cms):
    c, k = lay["corners"], lay["start"]
    order = [c[(k + i) % 4] for i in range(4)]          # counter-clockwise from the start corner
    a, b = order[0], order[1]
    h = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    st = (a[0] + LEAD_IN_CM * math.cos(math.radians(h)), a[1] + LEAD_IN_CM * math.sin(math.radians(h)))
    names = "ABCD"
    m = base_map(rect, "Remote ladder L4 (test one, b1): a %.0f cm square twice, %.0f cm/s" % (lay["side"], cms),
                 parked_obs, cms)
    m["spots"] = {names[i]: [round(order[i][0], 1), round(order[i][1], 1)] for i in range(4)}
    m["robots"] = [{"name": name, "start": [round(st[0], 1), round(st[1], 1), round(h, 1)]}]
    m["lead_in"] = {name: m["spots"]["A"]}      # the drive to the start comes in through A (ladder.drive_to)
    m["tasks"] = {name: {"visit": [{"spot": q} for q in "BCDABCDA"]}}
    return m


def cross_layouts(rect, parked, movers):
    """Test cross: a rectangle L (the way they travel) x W, the two start spots on one short side, each robot to the
    far corner across and back twice. Both diagonals and all four corners LINE_CLEAR_CM from every parked robot."""
    out = []
    mid = (rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0
    for L, W in CROSS_SIZES:
        for margin in LAYOUT_MARGINS:
            for rot in (0, 90, 180, 270):
                ux, uy = math.cos(math.radians(rot)), math.sin(math.radians(rot))
                vx, vy = -uy, ux
                ext_x = abs(ux) * L + abs(vx) * W
                ext_y = abs(uy) * L + abs(vy) * W
                for cx in np.arange(rect[0] + margin + ext_x / 2, rect[2] - margin - ext_x / 2 + 0.01, 2.0):
                    for cy in np.arange(rect[1] + margin + ext_y / 2, rect[3] - margin - ext_y / 2 + 0.01, 2.0):
                        def P(u, v):
                            return (cx + (u - L / 2) * ux + (v - W / 2) * vx, cy + (u - L / 2) * uy + (v - W / 2) * vy)
                        A, B, C, D = P(0, 0), P(0, W), P(L, 0), P(L, W)
                        segs = [(A, D), (B, C), (A, A), (B, B), (C, C), (D, D)]
                        if segs_clear(segs, parked, LINE_CLEAR_CM) < LINE_CLEAR_CM:
                            continue
                        n1, n2 = movers[0][0], movers[1][0]
                        p1, p2 = movers[0][1], movers[1][1]
                        d_ab = math.dist(p1, A) + math.dist(p2, B)
                        d_ba = math.dist(p1, B) + math.dist(p2, A)
                        asg = {n1: "A", n2: "B"} if d_ab <= d_ba else {n1: "B", n2: "A"}
                        off = math.hypot(cx - mid[0], cy - mid[1])
                        out.append((-L * W, margin != LAYOUT_MARGINS[0], off + 0.2 * min(d_ab, d_ba),
                                    dict(size=[L, W], rot=rot, margin=margin, A=A, B=B, C=C, D=D, heading=rot,
                                         assign=asg)))
        if out:
            break                       # the biggest cross that fits
    out.sort(key=lambda q: q[:3])
    return [q[3] for q in out]


def cross_map(rect, lay, parked_obs, cms, sep):
    m = base_map(rect, "Remote ladder L5 (test cross, b2): %.0f x %.0f cm, %.0f cm/s, bodies kept %.0f cm apart"
                 % (lay["size"][0], lay["size"][1], cms, sep), parked_obs, cms, sep)
    m["spots"] = {k: [round(lay[k][0], 1), round(lay[k][1], 1)] for k in "ABCD"}
    m["robots"], m["tasks"] = [], {}
    far = {"A": "D", "B": "C"}
    m["lead_in"] = {}
    for n, s in sorted(lay["assign"].items(), key=lambda q: q[1]):
        p, q = lay[s], lay[far[s]]
        h = math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))      # facing along its own diagonal
        st = (p[0] + LEAD_IN_CM * math.cos(math.radians(h)), p[1] + LEAD_IN_CM * math.sin(math.radians(h)))
        m["robots"].append({"name": n, "start": [round(st[0], 1), round(st[1], 1), round(h, 1)]})
        m["tasks"][n] = {"visit": [{"spot": q} for q in (far[s], s, far[s], s)]}
        m["lead_in"][n] = [round(p[0], 1), round(p[1], 1)]
    return m


def base_map(rect, notes, parked_obs, cms, sep=None):
    pl = {"cruise_cms": float(cms)}
    if sep is not None:
        pl["separation_cm"] = float(sep)
    return {"format": "alvik-fleet-map", "version": 1, "notes": notes,
            "floor": {"x_min": rect[0], "y_min": rect[1], "x_max": rect[2], "y_max": rect[3]},
            "sides": {"x_min": "open", "x_max": "open", "y_min": "open", "y_max": "open"},
            "robot": {"length_cm": 9.5, "width_cm": 9.5, "front_offset_cm": 4.75},
            "obstacles": [{"id": "parked-%s" % n, "shape": "circle", "center": [round(x, 1), round(y, 1)],
                           "radius": OBSTACLE_R_CM, "source": "camera", "note": "%s, parked" % n}
                          for n, (x, y) in parked_obs.items()],
            "mission": {"name": "ladder", "planner": pl}}


# ---------------------------------------------------------------------------------------------- the ladder
class Ladder:
    def __init__(self, a):
        self.a = a
        self.day = dt.date.today().strftime("%Y%m%d")
        self.dir = a.dir or os.path.join(REPO, "logs", "%s-ladder%s" % (self.day, "-dryrun" if a.dry else ""))
        os.makedirs(self.dir, exist_ok=True)
        self.state_path = os.path.join(self.dir, "state.json")
        self.st = self.load()
        self.eye = None
        self.bench = None
        self.sim = None
        self.keeper = None
        self.side_stream, self.side_calib = None, None
        if a.dry and getattr(a, "what", "run") not in ("status", "stop", "reset"):
            self.dry_board()
        elif a.dry:
            self.ips, self.stream, self.calib = [], "", ""
        else:
            self.ips = a.robot
            self.stream = a.stream
            self.calib = a.calib
            if a.side_calib:
                self.side_stream, self.side_calib = a.side_stream, a.side_calib
        self.rect = tuple(self.st.get("rect") or (0.0, 0.0, 122.0, 122.0))

    # -- state ----------------------------------------------------------------------------------------
    def load(self):
        if os.path.exists(self.state_path):
            with open(self.state_path) as f:
                return json.load(f)
        return {"passed": {}, "results": {}, "robots": {}, "kept_out": {}, "rect": [0.0, 0.0, 122.0, 122.0]}

    def save(self):
        tmp = self.state_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.st, f, indent=1)
        os.replace(tmp, self.state_path)

    def pos(self, n):
        r = self.st["robots"].get(n) or {}
        return (r["x"], r["y"]) if "x" in r else None

    def heading(self, n):
        r = self.st["robots"].get(n) or {}
        return r.get("h")

    def set_pose(self, n, x, y, h=None, how=None):
        r = self.st["robots"].setdefault(n, {})
        r.update(x=round(x, 2), y=round(y, 2), t=round(time.time(), 1))
        if h is not None:
            r.update(h=round(wrap180(h), 2), h_how=how, h_t=round(time.time(), 1))
        self.save()

    def keep_out(self, n, why):
        self.st["kept_out"][n] = why
        self.save()

    def active(self, need=None):
        names = sorted(self.st["robots"], key=lambda n: PREFER.index(n) if n in PREFER else 9)
        out = [n for n in names if n not in self.st["kept_out"]]
        if need:
            out = [n for n in out if self.st["passed"].get("%s:%s" % (need, n))]
        return out

    # -- the dry board ----------------------------------------------------------------------------------
    def dry_board(self):
        """The simulator as the board: three robots at random spots and headings (30 cm in, 40 cm apart), Center's
        turns --turn-err off; it keeps running between steps (fleet_sim --persist) until `ladder.py stop`."""
        sd = os.path.join(self.dir, "sim")
        os.makedirs(sd, exist_ok=True)
        self.ips = ["127.0.0.2", "127.0.0.3", "127.0.0.4"]
        self.stream = "127.0.0.1:8765"
        self.calib = os.path.join(sd, "table_camera.json")
        self.side_calib_dry = os.path.join(sd, "side_camera.json")
        if not self.a.sim_no_side:
            self.side_stream, self.side_calib = "127.0.0.1:8766", self.side_calib_dry
        pidf = os.path.join(sd, "pid")
        if os.path.exists(pidf):
            try:
                os.kill(int(open(pidf).read()), 0)
                return
            except (OSError, ValueError):
                pass
        rng = random.Random(self.a.seed)
        names = ["Left", "Center", "Right"]
        while True:
            pts = [(rng.uniform(30, 92), rng.uniform(30, 92)) for _ in names]
            if all(math.dist(pts[i], pts[j]) >= 40 for i in range(3) for j in range(i + 1, 3)):
                break
        starts = {n: [round(p[0], 1), round(p[1], 1), round(rng.uniform(-180, 180), 1)] for n, p in zip(names, pts)}
        m = base_map((0, 0, 122, 122), "remote ladder dry board", {}, NORMAL_CMS)
        m["robots"] = [{"name": n, "start": starts[n]} for n in names]
        mp, sp = os.path.join(sd, "board_map.json"), os.path.join(sd, "starts.json")
        json.dump(m, open(mp, "w"), indent=1)
        json.dump(starts, open(sp, "w"), indent=1)
        if os.path.exists(self.calib):
            os.remove(self.calib)
        cmd = ["nice", "-n", "10", PY, os.path.join(TOOLS, "sim", "fleet_sim.py"), "--map", mp, "--starts", sp,
               "--out", sd, "--seed", str(self.a.seed), "--fix-hz", "0", "--serve-live", "8765",
               "--live-calib-out", self.calib, "--persist", "--max-s", "5400", "--quiet"]
        if not self.a.sim_calibrated:
            cmd.append("--uncalibrated")           # as switched on: L1 must calibrate them
        if not self.a.sim_no_side:
            cmd += ["--live-side", "8766", "--live-side-calib-out", self.side_calib_dry]
        if self.a.sim_washout:
            cmd.append("--live-washout")
        for te in self.a.turn_err:
            cmd += ["--turn-err", te]
        p = subprocess.Popen(cmd, stdout=open(os.path.join(sd, "sim_stdout.txt"), "a"), stderr=subprocess.STDOUT,
                             start_new_session=True)
        open(pidf, "w").write(str(p.pid))
        self.st["dry"] = dict(starts=starts, turn_err=self.a.turn_err, seed=self.a.seed)
        self.save()
        say("Dry run: the simulated board is up (robots at %s; turns off: %s)" % (
            ", ".join("%s (%.0f, %.0f, %.0f deg)" % (n, *s) for n, s in starts.items()), ", ".join(self.a.turn_err) or "none"))
        t0 = time.time()
        while not os.path.exists(self.calib) and time.time() - t0 < 30:
            time.sleep(0.3)
        time.sleep(1.0)

    def stop_dry(self):
        pidf = os.path.join(self.dir, "sim", "pid")
        if os.path.exists(pidf):
            try:
                os.killpg(int(open(pidf).read()), signal.SIGTERM)
            except (OSError, ValueError):
                pass
            os.remove(pidf)
            time.sleep(2.0)

    # -- links -----------------------------------------------------------------------------------------
    def need_bench(self, sub):
        if self.bench is None:
            self.bench = Bench(self.ips, sub)
            self.bench.wait(lambda: len(self.bench.robots()) >= len(self.ips), 10)
            for n in self.bench.robots():           # the edge guard: not kept over a power-off, so every time
                # (the simulator's robots have no real gyro under-read: dry runs keep factory values)
                for c in SAFE + ([] if getattr(self.a, "dry", False) else corrections_cmds(n)):
                    self.bench.send(n, c)
                    time.sleep(0.02)
            time.sleep(0.3)
        return self.bench

    def drop_bench(self):
        if self.keeper is not None:
            self.keeper.stop()
            self.keeper = None
        if self.bench is not None:
            self.bench.close()
            self.bench = None
            time.sleep(0.5)

    def need_eye(self):
        if self.eye is None:
            self.eye = Eye(self.stream, self.calib, self.rect, side_stream=self.side_stream,
                           side_calib=self.side_calib, lights=not self.a.no_lights)
            t0 = time.time()
            while (self.eye.n < 5 or (self.eye.side_cam is not None and self.eye.side_n < 5)) and time.time() - t0 < 15:
                time.sleep(0.2)
            if self.eye.n < 5:
                raise LadderStop("No pictures from the overhead camera (%s). Is the recorder running? "
                                 "(./run-fleet-pi.sh rec)" % self.stream)
            if self.eye.side_cam is not None and self.eye.side_n < 5:
                say("  NOTE: no pictures from the side camera (%s): following with the overhead camera alone."
                    % self.side_stream)
            if not self.a.no_lights and self.bench is not None and self.keeper is None:
                self.keeper = LightsKeeper(self.bench, self.bench.robots())
                self.keeper.start()
        return self.eye

    def drop_eye(self):
        if self.keeper is not None:
            self.keeper.stop()
            self.keeper = None
        if self.eye is not None:
            self.eye.close()
            self.eye = None

    def sub(self, step):
        d = os.path.join(self.dir, time.strftime("%H%M%S") + "-" + step)
        os.makedirs(d, exist_ok=True)
        return d

    # -- where is everyone ------------------------------------------------------------------------------
    def find_all(self, sub, names=None):
        """Every robot by its lights (nothing moves), then the tracker on all of them. Returns {name: (x, y)}."""
        bench, eye = self.need_bench(sub), self.need_eye()
        names = names or sorted(bench.robots(), key=lambda n: PREFER.index(n) if n in PREFER else 9)
        found, lights = {}, {}
        for n in names:
            q = eye.find_lights(bench, n, others=list(lights.values()), keeper=self.keeper)
            if q is None:
                say("  %s: its lights were not seen anywhere on the board." % n)
                continue
            lights[n] = q
        eye.log_to(sub)
        eye.start_tracking(lights)
        if eye.trk.side is not None and eye.trk.side_gain > 1.0:
            say("  side camera picture is dark (mean %.0f of 255): brightened %.1fx for tracking"
                % (eye.trk.side_mean, eye.trk.side_gain))
        for n, q in lights.items():
            p = eye.still(n, timeout=4.0, body_only=True)
            if p is None and eye.still_why:
                say("  %s: %s" % (n, eye.still_why))
            if p is None:
                # NEITHER camera sees its body (2026-10-27 11:20: the overhead alone could not, on the washed-out board;
                # the side camera now looks too): it cannot be followed, so it never moves today, but it is on every
                # map where its lights are, so the others go round it (OBSTACLE_R_CM covers the ~3 cm from its lights
                # to its middle)
                cams = "neither the overhead camera nor the side camera can" if eye.trk.side is not None else \
                    "the overhead camera cannot (the side camera is not in use: %s)" % eye.side_words
                say("  %s: its lights are at (%.1f, %.1f), but %s see its body there (too little contrast with the "
                    "board): kept out, parked there; the others go round it." % (n, q[0], q[1], cams))
                r = self.st["robots"].setdefault(n, {})
                r.update(x=round(q[0], 2), y=round(q[1], 2), t=round(time.time(), 1), body_unseen=True)
                r.pop("h", None)
                self.st["kept_out"].setdefault(n, "no camera can see its body (only its lights)")
                self.save()
                continue
            found[n] = p
            say("  %s: body seen by %s" % (n, {"both": "both cameras", "overhead": "the overhead camera",
                                               "side": "the side camera only"}.get(eye.seen_by(n), "a camera")))
            h = None
            old = self.st["robots"].get(n)
            if old and "h" in old and math.hypot(old["x"] - p[0], old["y"] - p[1]) < 2.5:
                h = old["h"]                    # it has not moved since its heading was measured
            led = math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))
            r = self.st["robots"].setdefault(n, {})
            r.update(x=round(p[0], 2), y=round(p[1], 2), t=round(time.time(), 1), lights_dir=round(led, 1))
            if h is None:
                r.pop("h", None)
            self.save()
        for w in eye.tracking_words():
            say("  " + w)
        return found

    def others_of(self, n):
        return {m: self.pos(m) for m in self.st["robots"] if m != n and self.pos(m)}

    # ===================================================================================== the steps
    def L0(self, sub):
        bench = self.need_bench(sub)
        say("Robots heard: %s" % (", ".join(bench.robots()) or "none"))
        for n in bench.robots():
            bench.send(n, "stop")
        time.sleep(0.8)
        for n in bench.robots():
            for c in SAFE + ["set e02_cms %.1f" % NORMAL_CMS]:
                bench.send(n, c)
                time.sleep(0.03)
            bench.send(n, "status")
        bench.wait(lambda: all(n in bench.fw for n in bench.robots()), 5)
        good = []
        for n in sorted(bench.robots(), key=lambda q: PREFER.index(q) if q in PREFER else 9):
            fw = bench.fw.get(n)
            if fw is None:
                say("  %s: did not say its firmware (status lost?)" % n)
                continue
            ok = fw[0] >= MIN_FIRMWARE and fw[1] >= MIN_BATTERY
            say("  %s: firmware %s, battery %d%%%s" % (n, fw[0], fw[1], "" if ok else (
                "  -> kept out: firmware older than %s" % MIN_FIRMWARE if fw[0] < MIN_FIRMWARE else
                "  -> kept out: battery under %d%%" % MIN_BATTERY)))
            if ok:
                good.append(n)
            else:
                self.keep_out(n, "firmware %s, battery %d%%" % fw)
        eye = self.need_eye()
        fps = eye.rate(2.0)
        say("Overhead camera: %.0f pictures a second" % fps)
        if eye.side_cam is not None:
            say("Side camera: %.0f pictures a second (%s)" % (getattr(eye, "side_rate", 0.0), eye.side_words))
        else:
            say("Side camera: not used (%s)" % eye.side_words)
        if fps < 5:
            raise LadderStop("The camera sends only %.0f pictures a second (it needs 5+). Is the Pi too hot or busy? "
                             "Look at the recorder window (tmux attach -t fleet)." % fps)
        say("Finding each robot by its lights (nothing moves) ...")
        heard = sorted(bench.robots(), key=lambda q: PREFER.index(q) if q in PREFER else 9)
        found = self.find_all(sub, heard)        # every robot heard, kept out or not: the planner must know where
        ignore = [q.strip() for q in (self.a.ignore or "").split(",") if q.strip()]
        lost = [n for n in heard if self.pos(n) is None and n not in ignore]
        for n in ignore:
            self.keep_out(n, "ignored: Nick says it is not on the board")
        if lost:
            for n in lost:
                self.keep_out(n, "not found by its lights")
            self.result("L0", False, "not found anywhere on the board: %s" % ", ".join(lost),
                        "the live picture (http://100.122.222.75:8000): is it on the board, are its lights on?",
                        "The planner would not know where %s is, so nothing moves. Run L0 again (./run-fleet-pi.sh L0). "
                        "If it is still not found and you are SURE it is not on the board: ./run-fleet-pi.sh L0 ignore "
                        "%s" % (" and ".join(lost), ",".join(lost)))
            return False
        res = {}
        for n in good:
            p = found.get(n)
            if p is None:
                res[n] = "kept out"          # lights only (body not seen) or ignored: kept out above
                continue
            ed = edge_dist(self.rect, *p)
            near = [(m, math.dist(p, q)) for m, q in found.items() if m != n and math.dist(p, q) < 30]
            why = ""
            if ed < EDGE_REFUSE_CM:
                why = "%s (it must be %.0f cm in)" % (edge_words(self.rect, *p), EDGE_REFUSE_CM)
                self.keep_out(n, why)
            say("  %s at (%.1f, %.1f), %s%s" % (n, p[0], p[1], edge_words(self.rect, *p),
                                                 "  -> kept out: too near the edge" if why else ""))
            for m, d in near:
                say("  note: %s and %s are only %.0f cm apart (centres)" % (n, m, d))
            res[n] = "ok" if not why else "kept out"
            if not why:
                self.st["passed"]["L0:%s" % n] = round(time.time())
        self.save()
        ok = [n for n in res if res[n] == "ok"]
        self.result("L0", bool(ok), "found and fine: %s" % (", ".join(ok) or "none"),
                    "the robots' firmware lines above; the live picture (http://100.122.222.75:8000)",
                    "" if ok else "No robot can go on. Check they are switched on and on the board, then run L0 again.")
        return bool(ok)

    def L1(self, sub, names):
        bench, eye = self.need_bench(sub), self.need_eye()
        self.find_all(sub, names)
        done = []
        for n in names:
            p = self.pos(n)
            others = self.others_of(n)
            ed = edge_dist(self.rect, *p)
            close = [(m, math.dist(p, q)) for m, q in others.items() if math.dist(p, q) < CALIB_GAP_CM]
            if ed < CALIB_EDGE_CM or close:
                why = edge_words(self.rect, *p) if ed < CALIB_EDGE_CM else "%s is %.0f cm away" % close[0]
                say("  %s: no room to calibrate (%s; it needs %.0f cm to every edge and %.0f cm to every robot): kept "
                    "out" % (n, why, CALIB_EDGE_CM, CALIB_GAP_CM))
                self.keep_out(n, "no room to calibrate: " + why)
                continue
            say("  %s: calibrating alone (about 65 s) at (%.1f, %.1f) ..." % (n, p[0], p[1]))
            t0 = time.time()
            bench.runend.pop(n, None)
            bench.calib_look.pop(n, None)
            bench.send(n, "calibrate")
            fin = bench.wait(lambda: n in bench.runend and bench.runend[n][0] == "calib", 120)
            import session_checks as SC
            le = bench.runend.get(n)
            oc = SC.calib_outcome(le[1] if fin else None, bench.calib_look.get(n))
            q = eye.still(n, timeout=6.0)
            if q is None:
                lq = eye.find_lights(bench, n, others=list(others.values()))
                if lq is not None:
                    eye.trk.add(n, *lq)
                    q = eye.still(n, timeout=4.0)
            if q is not None:
                self.set_pose(n, *q)
                self.st["robots"][n].pop("h", None)
            moved = math.dist(p, q) if q else None
            txt = SC.calib_outcome_text(le[1], bench.calib_look.get(n)) if fin else "no end record in 120 s"
            say("    %s: %s in %.0f s; the camera has it %s" % (
                n, txt, time.time() - t0, "at (%.1f, %.1f), %.1f cm from where it started" % (q[0], q[1], moved)
                if q else "nowhere (lost)"))
            if oc == SC.CALIB_FAILED or q is None or edge_dist(self.rect, *q) < EDGE_REFUSE_CM:
                self.keep_out(n, "calibration: %s" % (txt if oc == SC.CALIB_FAILED else "camera lost it or it is "
                                                      "now too near an edge"))
                continue
            self.st["passed"]["L1:%s" % n] = round(time.time())
            done.append(n)
            self.save()
        self.result("L1", bool(done), "calibrated: %s" % (", ".join(done) or "none"),
                    "the CALIB lines in %s; the live picture" % sub,
                    "" if done else "No robot calibrated. Look at the reasons above: a robot without room cannot be "
                                    "moved to where it has room today.")
        return bool(done)

    # -- L2: the turn check --------------------------------------------------------------------------
    def crawl(self, n, p0, h_known):
        """Out and back CRAWL_CM, the camera watching: (heading deg, details) or raises LadderStop."""
        bench, eye = self.bench, self.eye
        sign = 1
        if h_known is not None:
            tip = (p0[0] + CRAWL_CM * math.cos(math.radians(h_known)), p0[1] + CRAWL_CM * math.sin(math.radians(h_known)))
            others = self.others_of(n).values()
            if edge_dist(self.rect, *tip) < EDGE_REFUSE_CM - 5 or any(math.dist(tip, o) < 22 for o in others):
                sign = -1                         # forward would go towards an edge or a robot: back first
        t0 = time.time()
        r1 = bench.leg(n, "go %.1f" % (sign * CRAWL_CM), 20)
        p1 = eye.still(n)
        r2 = bench.leg(n, "go %.1f" % (-sign * CRAWL_CM), 20)
        p2 = eye.still(n)
        t1 = time.time()
        if r1[0] != "done" or r2[0] != "done":
            bad = r1 if r1[0] != "done" else r2
            raise LadderStop("%s's crawl did not finish: %s (%s)" % (n, bad[0], bad[4]))
        if p1 is None or p2 is None:
            raise LadderStop("the camera lost %s during its crawl" % n)
        pts = eye.trk.history(n, t0, t1)
        h = line_fit_heading(pts, p0, p1)
        moved = math.dist(p0, p1)
        if h is None or not (0.6 * CRAWL_CM <= moved <= 1.4 * CRAWL_CM):
            raise LadderStop("the camera saw %s crawl %.1f cm (it was sent %.0f cm): the camera or its wheels are "
                             "wrong" % (n, moved, CRAWL_CM))
        if sign < 0:
            h = wrap180(h + 180.0)
        # the camera's pose as a fix (firmware e02.h `fix`, idle: replaces its pose): its own heading starts from the
        # camera's for the next turn, and its blind-backing allowance (e02_back_max_cm since the last fix) refills
        bench.send(n, "fix %.2f %.2f %.2f %.3f" % (p2[0], p2[1], h, time.time()))
        time.sleep(0.3)
        return h, dict(crawl_cm=round(moved, 2), back_off_cm=round(math.dist(p0, p2), 2), pictures=len(pts),
                       end=[round(p2[0], 2), round(p2[1], 2)], own_th_before_fix=r2[3])

    def turncheck(self, n, sub):
        bench, eye = self.need_bench(sub), self.need_eye()
        p = self.pos(n)
        if p is None or not eye.trk or eye.trk.state(n) is None:
            self.find_all(sub)
            p = self.pos(n)
        if p is None:
            raise LadderStop("%s is not on the camera" % n)
        p = eye.still(n) or p
        if edge_dist(self.rect, *p) < EDGE_REFUSE_CM:
            raise LadderStop("%s is %s: the turn check needs it %.0f cm in (it turns and crawls %.0f cm). Not "
                             "checked." % (n, edge_words(self.rect, *p), EDGE_REFUSE_CM, CRAWL_CM))
        bench.send(n, "set e02_cms %.1f" % CRAWL_CMS)
        time.sleep(0.3)
        say("  %s at (%.1f, %.1f): heading crawl (%.0f cm out and back) ..." % (n, p[0], p[1], CRAWL_CM))
        rows = []
        try:
            h0, d0 = self.crawl(n, p, None)
            heads = [h0]
            say("    start heading %.1f deg (camera, %d pictures)" % (h0, d0["pictures"]))
            for k in range(4):
                r = bench.leg(n, "turn 90", 20)
                if r[0] != "done":
                    raise LadderStop("%s's turn %d did not finish: %s (%s)" % (n, k + 1, r[0], r[4]))
                own_turn = wrap180(r[3] - heads[-1]) if r[3] is not None else None   # its gyro, from the fix
                pk = eye.still(n)
                if pk is None:
                    raise LadderStop("the camera lost %s after turn %d" % (n, k + 1))
                hk, dk = self.crawl(n, pk, wrap180(heads[-1] + (own_turn if own_turn is not None else 90.0)))
                cam_turn = wrap180(hk - heads[-1])
                heads.append(hk)
                err = abs(cam_turn) - 90.0
                rows.append(dict(turn=k + 1, camera_deg=round(cam_turn, 1), own_deg=None if own_turn is None
                                 else round(own_turn, 1), error_deg=round(err, 1), crawl=dk))
                say("    turn %d: camera %+.1f deg, the robot's gyro %s -> error %+.1f deg  %s" % (
                    k + 1, cam_turn, "%+.1f" % own_turn if own_turn is not None else "?", err,
                    "ok" if abs(err) <= TURN_TOL_DEG else "OFF"))
        finally:
            bench.send(n, "set e02_cms %.1f" % NORMAL_CMS)
        total = wrap180(heads[-1] - heads[0])
        signs = {1 if r["camera_deg"] > 0 else -1 for r in rows}
        ok = all(abs(r["error_deg"]) <= TURN_TOL_DEG for r in rows) and abs(total) <= TOTAL_TOL_DEG and len(signs) == 1
        q = eye.still(n) or p
        self.set_pose(n, q[0], q[1], heads[-1], "turn check crawl")
        res = dict(robot=n, turns=rows, total_error_deg=round(total, 1), start_heading=round(h0, 1),
                   pass_=ok, tol_turn_deg=TURN_TOL_DEG, tol_total_deg=TOTAL_TOL_DEG)
        with open(os.path.join(sub, "turncheck-%s.json" % n), "w") as f:
            json.dump(res, f, indent=1)
        say("    after 4 turns it faces %+.1f deg from where it started (allowed %.0f); each turn allowed %.0f deg "
            "-> %s" % (total, TOTAL_TOL_DEG, TURN_TOL_DEG, "PASS" if ok else "FAIL"))
        return ok, res

    def L2(self, sub, names):
        self.find_all(sub)
        ok = []
        for n in names:
            try:
                good, res = self.turncheck(n, sub)
            except LadderStop as e:
                say("    %s: %s" % (n, e))
                good, res = False, dict(error=str(e))
            if good:
                self.st["passed"]["L2:%s" % n] = round(time.time())
                ok.append(n)
            else:
                why = res.get("error") or "turns off (%s; total %+.1f deg)" % (
                    ", ".join("%+.1f" % r["error_deg"] for r in res["turns"]), res["total_error_deg"])
                self.keep_out(n, "turn check: " + why)
                say("  %s is KEPT OUT of every later step: it stays parked where it is, and the others go round it."
                    % n)
            self.save()
        out = [n for n in names if n not in ok]
        self.result("L2", bool(ok), "turns true: %s%s" % (", ".join(ok) or "none", "; kept out: %s" % ", ".join(out)
                                                         if out else ""),
                    "turncheck-<robot>.json in %s; the live picture" % sub,
                    "" if ok else "No robot turns true. Decide: calibrate again (L1) and repeat L2, or end today's "
                                  "remote test here.")
        return bool(ok)

    # -- runs through the planner ------------------------------------------------------------------------
    def run_planned(self, tag, m, at, sub, gate=True, bridge_extra=()):
        """cam_bridge + run_plan on map m; at: {name: (x, y, heading)} where the camera has each moving robot now.
        Returns dict(code, summary, check, sess)."""
        self.drop_bench()
        self.drop_eye()
        sess = os.path.join(sub, tag)
        os.makedirs(sess, exist_ok=True)
        mp = os.path.join(sess, "map.json")
        with open(mp, "w") as f:
            json.dump(m, f, indent=1)
        bcmd = ["nice", "-n", "5", PY, os.path.join(HERE, "cam_bridge.py"), "--map", mp, "--calib", self.calib,
                "--stream", self.stream, "--out", os.path.join(sess, "bridge")]
        for n, (x, y, h) in at.items():
            bcmd += ["--at", "%s=%.2f,%.2f,%.2f" % (n, x, y, h)]
        cms = float(((m.get("mission") or {}).get("planner") or {}).get("cruise_cms", NORMAL_CMS))
        if cms < 7.5:
            # cam_bridge's travel heading needs HEADING_MIN_CM (5 cm) inside its window: 1 s at 5 cm/s never has it
            # (dry run 2026-10-27: no heading sent in a whole slow drive, a 19 deg error stayed); 2 s does
            bcmd += ["--heading-window-s", "%.1f" % max(1.0, 2.0 * 5.0 / cms)]
        if self.side_calib:
            bcmd += ["--side-stream", self.side_stream, "--side-calib", self.side_calib]
        if self.a.no_lights:
            bcmd += ["--no-lights"]
        bcmd += list(bridge_extra)
        bf = open(os.path.join(sess, "bridge.txt"), "w")
        br = subprocess.Popen(bcmd, stdout=bf, stderr=subprocess.STDOUT)
        time.sleep(5.0)
        rcmd = [PY, os.path.join(HERE, "run_plan.py"), "--map", mp, "--session", sess, "--fixes", "udp",
                "--poses-to", "127.0.0.1:5011", "--place-tol-cm", str(START_TOL_CM), "--place-tol-deg",
                str(START_TOL_DEG)]
        for ip in self.ips:
            rcmd += ["--robot", ip]
        for c in SAFE:
            rcmd += ["--send", c]
        if self.a.dry:
            rcmd = ["nice", "-n", "10"] + rcmd
        say("    run_plan: %s" % m["notes"])
        code = subprocess.call(rcmd)
        try:
            br.send_signal(signal.SIGINT)
            br.wait(timeout=8)
        except subprocess.TimeoutExpired:
            br.kill()
        bf.close()
        summ = {}
        sp = os.path.join(sess, "run_plan_summary.json")
        if os.path.exists(sp):
            summ = json.load(open(sp))
        chk = None
        if os.path.exists(os.path.join(sess, "coordinator.jsonl")) and os.path.exists(os.path.join(sess, "bridge",
                                                                                                    "track.csv")):
            import line_check as LC
            tru = os.path.join(self.dir, "sim", "truth.csv") if self.a.dry else None
            try:
                chk = LC.check(sess, os.path.join(sess, "bridge"), truth=tru, gate=GATE_CM,
                               png=os.path.join(sess, "line_check.png"))
            except Exception as e:      # no step was sent: nothing to check
                say("    (line check: %s)" % e)
        return dict(code=code, summary=summ, check=chk, sess=sess)

    def end_poses(self, sess, names):
        """Where each robot ended: the camera's spot (median of its last second, standing still) and its heading:
        the camera's line through its LAST straight drive (one or more drive steps on one heading) plus what its own
        gyro turned after that leg."""
        track = {}
        with open(os.path.join(sess, "bridge", "track.csv")) as f:
            for r in csv.DictReader(f):
                if r["status"] == "seen":
                    track.setdefault(r["robot"], []).append((float(r["t"]), float(r["x"]), float(r["y"])))
        sent, done = {}, {}
        with open(os.path.join(sess, "coordinator.jsonl")) as f:
            for ln in f:
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                if e.get("event") == "step_sent":
                    sent[(e["robot"], e["id"])] = e
                elif e.get("event") == "step_report" and e.get("state") in ("started", "done"):
                    done.setdefault((e["robot"], e["id"]), {})[e["state"]] = e
        own = {}
        with open(os.path.join(sess, "follow.csv")) as f:
            for r in csv.DictReader(f):
                try:
                    own.setdefault(r["robot"], []).append((float(r["t_recv"]), float(r["th_deg"])))
                except (ValueError, KeyError):
                    pass
        out = {}
        for n in names:
            tr = track.get(n, [])
            if not tr:
                out[n] = None
                continue
            t_last = tr[-1][0]
            last = [q for q in tr if q[0] >= t_last - 1.0]
            x, y = float(np.median([q[1] for q in last])), float(np.median([q[2] for q in last]))
            drives = []
            for (rn, sid), e in sorted(sent.items(), key=lambda q: q[1]["t"]):
                parts = e["text"].split()
                if rn != n or len(parts) < 8 or parts[2] != "D" or "done" not in done.get((rn, sid), {}):
                    continue
                ts = done[(rn, sid)].get("started", e)["t"]
                drives.append((ts, done[(rn, sid)]["done"]["t"], float(parts[7]),
                               (float(parts[3]), float(parts[4])), (float(parts[5]), float(parts[6]))))
            h, how = None, "none"
            if drives:
                leg = [drives[-1]]
                for d in reversed(drives[:-1]):
                    if abs(wrap180(d[2] - leg[0][2])) < 1.0 and math.dist(d[4], leg[0][3]) < 1.0:
                        leg.insert(0, d)
                    else:
                        break
                ta, tb = leg[0][0], leg[-1][1]
                pts = [q for q in tr if ta <= q[0] <= tb]
                hc = line_fit_heading(pts, leg[0][3], leg[-1][4]) if len(pts) >= 8 else None
                length = math.dist(leg[0][3], leg[-1][4])
                if hc is not None:
                    mv = math.degrees(math.atan2(leg[-1][4][1] - leg[0][3][1], leg[-1][4][0] - leg[0][3][0]))
                    if abs(wrap180(mv - leg[0][2])) > 90:      # a backwards leg: it faces the other way
                        hc = wrap180(hc + 180.0)
                    # its own heading where that leg ended and at its very end, from the steps' `done` reports
                    # (follow.csv's records stop mid-way through the last turn: dry run, 2026-10-27, read 20 deg short)
                    dn = sorted((d["done"]["t"], d["done"].get("th")) for (rn, _), d in done.items()
                                if rn == n and "done" in d and d["done"].get("th") is not None)
                    th_leg = [q[1] for q in dn if q[0] <= tb + 1e-3]
                    turn_after = wrap180(dn[-1][1] - th_leg[-1]) if dn and th_leg else 0.0
                    h = wrap180(hc + turn_after)
                    how = "camera line through its last %.0f cm straight (%d pictures) %+.1f deg its gyro turned " \
                          "after" % (length, len(pts), turn_after)
            out[n] = dict(x=x, y=y, h=h, how=how)
        return out

    def drive_to(self, sub, tag, targets, cms=SLOW_CMS, via=None):
        """Plan each robot in targets {name: (x, y, heading)} from where the camera has it to its target pose (the
        other robots parked, on the map as obstacles), drive it there with camera fixes. Returns {name: end pose}."""
        at = {n: (*self.pos(n), self.heading(n)) for n in targets}
        for n, (x, y, h) in at.items():
            if h is None:
                raise LadderStop("%s's heading is not known (it moved since its last heading crawl)" % n)
        need = {n: (x, y, h) for n, (x, y, h) in targets.items()
                if math.dist((x, y), at[n][:2]) > 1.5 or abs(wrap180(h - at[n][2])) > 3.0}
        if not need:
            return {n: dict(x=at[n][0], y=at[n][1], h=at[n][2], how="already there") for n in targets}
        parked = {m: self.pos(m) for m in self.st["robots"] if m not in need and self.pos(m)}
        m = base_map(self.rect, "Remote ladder: drive %s to %s" % (", ".join(need), tag), parked, cms)
        m["robots"] = [{"name": n, "start": [round(v, 2) for v in at[n]],
                        "park": [round(v, 2) for v in need[n]]} for n in need]
        m["tasks"] = {n: {"visit": [], "park": True} for n in need}
        m["spots"] = {}
        for n, q in (via or {}).items():           # in through the corner: the last leg runs along the start heading
            if n in need:
                m["spots"]["in-" + n] = list(q)
                m["tasks"][n]["visit"] = [{"spot": "in-" + n}]
        r = self.run_planned("drive-" + tag, m, {n: at[n] for n in need}, sub, gate=False)
        if r["summary"].get("result") != "done":
            raise LadderStop("the drive to %s's start did not finish (%s). Look at %s/bridge.txt and the live picture"
                             % (tag, r["summary"].get("result"), r["sess"]))
        ends = self.end_poses(r["sess"], list(need))
        for n, e in ends.items():
            if e is None:
                raise LadderStop("the camera did not see %s at the end of its drive" % n)
            self.set_pose(n, e["x"], e["y"], e["h"], e["how"])
            say("    %s ended at (%.1f, %.1f), heading %s: %s" % (n, e["x"], e["y"],
                                                                   "?" if e["h"] is None else "%.1f" % e["h"], e["how"]))
        return {**ends, **{n: dict(x=at[n][0], y=at[n][1], h=at[n][2], how="already there")
                           for n in targets if n not in need}}

    def start_check(self, n, end, target):
        d = math.dist((end["x"], end["y"]), target[:2])
        dh = None if end["h"] is None else abs(wrap180(end["h"] - target[2]))
        ok = d <= START_TOL_CM and dh is not None and dh <= START_TOL_DEG
        say("    start spot check, %s: %.1f cm and %s deg from the test's start (allowed %.0f cm, %.0f deg) -> %s" % (
            n, d, "?" if dh is None else "%.1f" % dh, START_TOL_CM, START_TOL_DEG, "ok" if ok else "REFUSED"))
        if not ok:
            raise LadderStop("%s is not on the test's start spot after its drive (%.1f cm, %s deg off; allowed %.0f cm, "
                             "%.0f deg), so the test is not run. The drive's picture: line_check.png in its folder"
                             % (n, d, "?" if dh is None else "%.1f" % dh, START_TOL_CM, START_TOL_DEG))

    def gate_words(self, r, names):
        chk = r["check"] or {}
        res = r["summary"].get("result")
        lines, ok = [], res == "done" and bool(chk.get("pass"))
        for n in names:
            c = (chk.get("robots") or {}).get(n, {}).get("camera", {})
            lines.append("%s p95 %s cm (median %s, max %s)" % (n, c.get("p95_cm"), c.get("median_cm"), c.get("max_cm")))
        cp = chk.get("closest_pair_camera")
        if cp:
            lines.append("closest bodies %.1f cm (camera)" % cp["bodies_cm"])
        return ok, "run %s; %s" % (res, "; ".join(lines))

    def L3(self, sub, names):
        self.find_all(sub)
        ok = []
        for n in names:
            p, h = self.pos(n), self.heading(n)
            if h is None:
                say("  %s: heading not known (it moved since L2): heading crawl first" % n)
                self.need_bench(sub)
                self.need_eye()
                h, _ = self.crawl(n, self.eye.still(n) or p, None)
                q = self.eye.still(n) or p
                self.set_pose(n, q[0], q[1], h, "crawl")
                p = q
            others = self.others_of(n)
            best = None
            for turn in (0, 90, -90, 180):
                d = wrap180(h + turn)
                o = (p[0] + OUTBACK_CM * math.cos(math.radians(d)), p[1] + OUTBACK_CM * math.sin(math.radians(d)))
                if edge_dist(self.rect, *o) < LAYOUT_MARGINS[0]:
                    continue
                if segs_clear([(p, o)], list(others.values()), LINE_CLEAR_CM) < LINE_CLEAR_CM:
                    continue
                best = o
                break
            if best is None:
                say("  %s: no straight 25 cm way out that stays %.0f cm in and %.0f cm from the others: not run"
                    % (n, LAYOUT_MARGINS[0], LINE_CLEAR_CM))
                continue
            m = base_map(self.rect, "Remote ladder L3: %s 25 cm straight out and back at %.0f cm/s" % (n, SLOW_CMS),
                         others, SLOW_CMS)
            m["spots"] = {"O": [round(best[0], 1), round(best[1], 1)]}
            m["robots"] = [{"name": n, "start": [round(p[0], 2), round(p[1], 2), round(h, 2)]}]
            m["tasks"] = {n: {"visit": [{"spot": "O"}], "park": True}}
            r = self.run_planned("L3-" + n, m, {n: (p[0], p[1], h)}, sub)
            good, words = self.gate_words(r, [n])
            say("  %s: %s -> %s" % (n, words, "PASS" if good else "FAIL"))
            if r["check"] is not None:
                e = self.end_poses(r["sess"], [n])[n]
                if e:
                    self.set_pose(n, e["x"], e["y"], e["h"], e["how"])
            if good:
                ok.append(n)
                self.st["passed"]["L3:%s" % n] = round(time.time())
            else:
                self.keep_out(n, "L3 straight: " + words)
            self.st["results"]["L3:%s" % n] = dict(words=words, sess=r["sess"])
            self.save()
            self.find_all(sub)
        self.result("L3", bool(ok), "straight and back within the gate: %s" % (", ".join(ok) or "none"),
                    "line_check.png in each L3-<robot> folder of %s" % sub,
                    "" if ok else "No robot followed a straight line within %.0f cm. Decide: look at the "
                                  "line_check.png and the bridge's 'unsure' lines before going further." % GATE_CM)
        return bool(ok)

    def L4(self, sub, names):
        self.find_all(sub)
        n = names[0]
        parked = {m: q for m, q in self.others_of(n).items()}
        # the robots that cannot move (kept out, or not through L3) fix where the square can go; a healthy robot in
        # its way is driven to a holding spot first (2026-10-27 dry runs: with two robots parked where they were
        # left, a square of the plan's size fitted in 0 of 60 random layouts, only 42 cm ones in 26)
        fixed = {m: q for m, q in parked.items() if not self.st["passed"].get("L3:%s" % m)}
        movable = {m: q for m, q in parked.items() if m not in fixed}
        lays = square_layouts(self.rect, list(fixed.values()), self.pos(n))
        moves = None
        if lays:
            segs = square_segs(lays[0])
            block = {m: q for m, q in movable.items() if segs_clear(segs, [q], 0) < LINE_CLEAR_CM}
            if block:
                keep = [q for m, q in parked.items() if m not in block] + [self.pos(n)]
                hs = holding_spots(self.rect, segs, block, keep)
                if hs:
                    moves = {m: (hs[m][0], hs[m][1], self.heading(m)) for m in block}
                else:
                    lays = None
        if not lays:
            lays = square_layouts(self.rect, list(parked.values()), self.pos(n))
            moves = None
        if moves:
            say("  making room: %s" % "; ".join("%s from (%.0f, %.0f) to (%.0f, %.0f)" % (
                m, *parked[m], v[0], v[1]) for m, v in moves.items()))
            if any(v[2] is None for v in moves.values()):
                raise LadderStop("a robot in the square's way has no known heading: run L3 again")
            self.drive_to(sub, "L4-room", moves)
            parked = {m: q for m, q in self.others_of(n).items()}
        if not lays:
            raise LadderStop("No square of %s cm fits between the parked robots (%s) with %.0f cm room, even with the "
                             "healthy ones driven out of the way. Decide: skip L4 and try L5 (`./run-fleet-pi.sh L5`: "
                             "the cross needs less room), or end here." % (
                                 "/".join("%.0f" % s for s in SQUARE_SIDES),
                                 ", ".join("%s (%.0f, %.0f)" % (m, *q) for m, q in parked.items()), LINE_CLEAR_CM))
        lay = lays[0]
        self.st["results"]["L4_layout"] = lay
        say("  %s's square: %.0f cm, corners %s, %.0f cm in; start at corner %s" % (
            n, lay["side"], ", ".join("(%.0f, %.0f)" % c for c in lay["corners"]), lay["margin"], "ABCD"[0]))
        allok, words = True, []
        for label, cms in (("slow", SLOW_CMS), ("normal", NORMAL_CMS)):
            m = square_map(self.rect, lay, n, parked, cms)
            st = m["robots"][0]["start"]
            say("  L4 %s (%.0f cm/s): driving %s to the square's start (%.0f, %.0f) facing %.0f deg ..."
                % (label, cms, n, *st))
            end = self.drive_to(sub, "L4%s-%s" % (label[0], n), {n: tuple(st)}, via=m["lead_in"])[n]
            self.start_check(n, end, st)
            r = self.run_planned("L4-%s-%s" % (label, n), m, {n: (end["x"], end["y"], end["h"])}, sub)
            good, w = self.gate_words(r, [n])
            say("  L4 %s: %s -> %s" % (label, w, "PASS" if good else "FAIL"))
            words.append("%s: %s" % (label, w))
            if r["check"] is not None:
                e = self.end_poses(r["sess"], [n])[n]
                if e:
                    self.set_pose(n, e["x"], e["y"], e["h"], e["how"])
            if not good:
                allok = False
                break
            self.find_all(sub)
        self.st["results"]["L4"] = words
        if allok:
            self.st["passed"]["L4:%s" % n] = round(time.time())
        self.save()
        self.result("L4", allok, "; ".join(words), "line_check.png in the L4-slow-%s and L4-normal-%s folders of %s"
                    % (n, n, sub), "" if allok else "The square left the 3 cm gate. Decide: look at its "
                    "line_check.png; if the dots slant away after a corner, it is the turns (L2's numbers); if they "
                    "jump, it is the camera (bridge.txt 'unsure').")
        return allok

    def L5(self, sub, names):
        if len(names) < 2:
            raise LadderStop("L5 needs two robots that passed L3; only %s did. The ladder ends here." %
                             (", ".join(names) or "none"))
        self.find_all(sub)
        movers = names[:2]
        parked = {m: q for m, q in self.st["robots"].items() if m not in movers and "x" in q}
        parked = {m: (q["x"], q["y"]) for m, q in parked.items()}
        lays = cross_layouts(self.rect, list(parked.values()), [(n, self.pos(n)) for n in movers])
        if not lays:
            raise LadderStop("No crossing layout fits round the parked robots (%s). Decide: run L5 another day, or "
                             "accept a smaller cross (CROSS_SIZES in ladder.py)." %
                             ", ".join("%s (%.0f, %.0f)" % (m, *q) for m, q in parked.items()))
        lay = lays[0]
        self.st["results"]["L5_layout"] = {k: v for k, v in lay.items()}
        say("  cross: %.0f x %.0f cm, travel heading %d deg, %s on A (%.0f, %.0f), %s on B (%.0f, %.0f)" % (
            lay["size"][0], lay["size"][1], lay["rot"], [k for k, v in lay["assign"].items() if v == "A"][0],
            *lay["A"], [k for k, v in lay["assign"].items() if v == "B"][0], *lay["B"]))
        allok, words = True, []
        for label, cms, sep in (("slow, wide", SLOW_CMS, WIDE_SEP_CM), ("normal", NORMAL_CMS, 5.0)):
            m = cross_map(self.rect, lay, parked, cms, sep)
            starts = {r["name"]: tuple(r["start"]) for r in m["robots"]}
            say("  L5 %s: driving %s to their starts ..." % (label, " and ".join(starts)))
            ends = self.drive_to(sub, "L5-%s" % label.split(",")[0], starts, via=m["lead_in"])
            for n in starts:
                self.start_check(n, ends[n], starts[n])
            r = self.run_planned("L5-%s" % label.replace(", ", "-"), m,
                                 {n: (ends[n]["x"], ends[n]["y"], ends[n]["h"]) for n in starts}, sub)
            good, w = self.gate_words(r, list(starts))
            say("  L5 %s: %s -> %s" % (label, w, "PASS" if good else "FAIL"))
            words.append("%s: %s" % (label, w))
            if r["check"] is not None:
                for n, e in self.end_poses(r["sess"], list(starts)).items():
                    if e:
                        self.set_pose(n, e["x"], e["y"], e["h"], e["how"])
            if not good:
                allok = False
                break
            self.find_all(sub)
        self.st["results"]["L5"] = words
        if allok:
            self.st["passed"]["L5"] = round(time.time())
        self.save()
        self.result("L5", allok, "; ".join(words), "line_check.png in the L5 folders of %s; bridge.txt 'unsure' "
                    "lines when they passed each other" % sub, "" if allok else "Decide: if the camera lost who is "
                    "who as they passed (bridge.txt 'unsure'/'merged'), the normal spacing is too close for the "
                    "camera; if one stopped for the camera guard, look at its turns.")
        return allok

    def L6(self, sub, names):
        if not self.a.attended:
            raise LadderStop("L6 (the brick) needs someone at the board to put the brick down. Run it with "
                             "`./run-fleet-pi.sh L6 attended` when you are there.")
        lay = self.st["results"].get("L5_layout")
        if not lay:
            raise LadderStop("L6 needs L5's layout: run L5 first")
        self.find_all(sub)
        movers = list(lay["assign"])
        parked = {m: self.pos(m) for m in self.st["robots"] if m not in movers and self.pos(m)}
        m = cross_map(self.rect, {**lay, "A": tuple(lay["A"]), "B": tuple(lay["B"]), "C": tuple(lay["C"]),
                                  "D": tuple(lay["D"])}, parked, NORMAL_CMS, 5.0)
        starts = {r["name"]: tuple(r["start"]) for r in m["robots"]}
        ends = self.drive_to(sub, "L6", starts, via=m["lead_in"])
        for n in starts:
            self.start_check(n, ends[n], starts[n])
        say("  About 15 s after they start, put the brick down where they cross and take your hand away.")
        r = self.run_planned("L6-brick", m, {n: (ends[n]["x"], ends[n]["y"], ends[n]["h"]) for n in starts}, sub,
                             bridge_extra=["--brick-watch"])
        good, w = self.gate_words(r, list(starts))
        self.result("L6", good, w, "bridge.txt 'brick watch' lines in %s" % r["sess"], "")
        return good

    # -- running the ladder -------------------------------------------------------------------------------------
    def result(self, step, ok, what, look, decide):
        self.st["results"][step] = dict(ok=ok, what=what, t=round(time.time()))
        if ok:
            self.st["passed"][step] = round(time.time())
        self.save()
        say("")
        say("%s %s: %s" % (step, "PASS" if ok else "FAIL", what))
        say("    Look at: %s" % look)
        if self.st["kept_out"]:
            say("    Kept out (parked where they are): %s" % "; ".join("%s (%s)" % kv for kv in self.st["kept_out"].items()))
        if decide:
            say("    For you to decide: %s" % decide)

    def run_step(self, step, in_order=False):
        """in_order (the ladder): the step before must have passed. A single step (./run-fleet-pi.sh L5) needs only
        the step it builds on: L5 needs L3 (the cross can still run when no square fitted for L4), L6 needs L5."""
        need = {"L1": "L0", "L2": "L1", "L3": "L2", "L4": "L3", "L5": "L3", "L6": "L5"}.get(step)
        prev = (STEPS[STEPS.index(step) - 1] if step != "L0" else None) if in_order else need
        if prev and not self.st["passed"].get(prev):
            raise LadderStop("%s comes after %s, which has not passed yet." % (step, prev))
        need = "L3" if step == "L6" else need
        names = self.active(need)
        if self.a.robots:
            names = [n for n in names if n in self.a.robots.split(",")]
        if step != "L0" and not names:
            raise LadderStop("no robot is left for %s" % step)
        say("")
        say("=" * 100)
        say("%s  %s%s" % (step, TITLES[step], ("  [%s]" % ", ".join(names)) if step != "L0" else ""))
        say("=" * 100)
        sub = self.sub(step)
        fn = getattr(self, step)
        return fn(sub) if step == "L0" else fn(sub, names)

    def run(self, start=None, pause=False):
        steps = STEPS[:-1]
        i = steps.index(start) if start else next((k for k, s in enumerate(steps) if not self.st["passed"].get(s)),
                                                  len(steps))
        for s in steps[i:]:
            if pause and s != steps[i]:
                input("\nPress Enter to start %s (%s), or Ctrl+C to stop here: " % (s, TITLES[s]))
            if not self.run_step(s, in_order=True):
                say("\nThe ladder stopped at %s." % s)
                return False
        say("\nThe ladder is done: every step passed (L6, the brick, needs you at the board: "
            "./run-fleet-pi.sh L6 attended).")
        return True

    def status(self):
        say("Ladder folder: %s" % self.dir)
        for s in STEPS:
            r = self.st["results"].get(s)
            say("  %s %-6s %s" % (s, "PASS" if self.st["passed"].get(s) else ("FAIL" if r else "-"),
                                  (r or {}).get("what", "")))
        for n, r in self.st["robots"].items():
            say("  %s at (%s, %s), heading %s%s" % (n, r.get("x"), r.get("y"), r.get("h", "?"),
                                                    "  KEPT OUT: " + self.st["kept_out"][n] if n in self.st["kept_out"] else ""))

    def close(self):
        self.drop_eye()
        self.drop_bench()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["run", "step", "turncheck", "status", "stop", "reset"])
    ap.add_argument("arg", nargs="?", help="step: L0..L6; turncheck: the robot")
    ap.add_argument("--from", dest="start", help="run: start at this step")
    ap.add_argument("--pause", action="store_true", help="run: wait for Enter before each step")
    ap.add_argument("--robots", default="", help="only these robots, e.g. Left,Right")
    ap.add_argument("--attended", action="store_true", help="someone is at the board (L6)")
    ap.add_argument("--ignore", default="", help="L0: robots heard on the WiFi but NOT on the board (Nick's word)")
    ap.add_argument("--robot", action="append", default=[], help="robot IP (repeatable)")
    ap.add_argument("--stream", default="127.0.0.1:8000", help="the recorder's /live")
    ap.add_argument("--calib", default=os.path.join(REPO, "tools", "experiment", "data",
                                                    "overhead_floorcal_roam1.json"))
    ap.add_argument("--side-stream", default="127.0.0.1:8001", help="the recorder's side camera /live (camera 1)")
    ap.add_argument("--side-calib", default="", help="the side camera's floor calibration (setup page); none: the "
                                                      "overhead camera alone")
    ap.add_argument("--no-lights", action="store_true", help="no lights-as-markers fallback (and no steady lights)")
    ap.add_argument("--sim-washout", action="store_true", help="--dry: the simulated board washed out under the lamp "
                                                               "for the overhead camera (11:20), the side camera on")
    ap.add_argument("--sim-no-side", action="store_true", help="--dry: no simulated side camera")
    ap.add_argument("--dir", help="the ladder's folder (default logs/<date>-ladder)")
    ap.add_argument("--dry", action="store_true", help="the simulator as the board")
    ap.add_argument("--seed", type=int, default=7, help="--dry: where the simulated robots start")
    ap.add_argument("--turn-err", action="append", default=[], metavar="NAME=PCT", help="--dry: a robot's turns off")
    ap.add_argument("--sim-calibrated", action="store_true", help="--dry: the robots start calibrated (quick tests)")
    a = ap.parse_args(argv)

    def hup(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGHUP, hup)          # the SSH link dropped: stop every robot as Ctrl+C does
    signal.signal(signal.SIGTERM, hup)
    L = None
    try:
        L = Ladder(a)
        if a.what == "status":
            L.status()
            return 0
        if a.what == "stop":
            L.stop_dry()
            say("Stopped the simulated board." if a.dry else "Nothing to stop (the robots stop when a step ends).")
            return 0
        if a.what == "reset":
            L.stop_dry()
            os.remove(L.state_path) if os.path.exists(L.state_path) else None
            say("Ladder state cleared: the next run starts at L0.")
            return 0
        if a.what == "turncheck":
            n = a.arg or "Center"
            sub = L.sub("turncheck-" + n)
            L.find_all(sub)
            ok, _ = L.turncheck(n, sub)
            say("TURN CHECK %s: %s" % (n, "PASS" if ok else "FAIL"))
            return 0 if ok else 1
        if a.what == "step":
            ok = L.run_step((a.arg or "L0").upper())
            return 0 if ok else 1
        ok = L.run(a.start.upper() if a.start else None, a.pause)
        return 0 if ok else 1
    except LadderStop as e:
        say("\nSTOPPED: %s" % e)
        return 1
    except KeyboardInterrupt:
        say("\nStopped (Ctrl+C or the connection dropped): every robot was sent stop.")
        return 130
    finally:
        if L is not None:
            L.close()


if __name__ == "__main__":
    sys.exit(main())
