#!/usr/bin/env python3
"""E02 accuracy experiment: the laptop runner. Double-click run-e02.bat, or:

    python tools/experiment/e02.py --pi st1maxfini.local            # a new session (about 2 hours)
    python tools/experiment/e02.py --resume                          # carry on the last session after a crash
    python tools/experiment/e02.py --dry-run --blocks 1 --routes 1,2 # the whole thing against the simulator

Plan: docs/build-plans/accuracy-experiment-E02.md. What it does, in order:
  1. Setup checks, each in plain words: the Pi recorder (both cameras answering, notes heard), the floor
     calibration (the studio's cameras/<camera>.json, or one made here from the board's corners: it
     offers), the two blocked boards found in the picture and their floor lines measured, every route fits,
     the robots connected (battery), each robot found by the camera (a short drive), then parked.
  2. A find check: one `run find` per robot, so a firmware problem shows before 2 hours start, and the
     camera measures where each robot really touched each board (a second measure of the board lines).
  3. The 2-minute naming check: every robot stands still blinking its code (`run gyro` for 120 s, which
     also measures each gyro's drift) while tools/live's blink tracker names them. Recorded only.
  4. The schedule: balanced, shuffled blocks of U / C / K / KC x the three routes x the robots, block 0
     all U. One robot moves at a time. Per run: corrections for the condition, `run find`, `run e02`
     (K: the laptop steers it leg by leg; KC: camera fixes 3 a second), the camera measures start, far
     point and end, marks go into both camera videos, then it is parked again by camera. After each U run
     the corrections are re-learned (gyro scale, drift, wheels) and the latest are used for C and KC.
  5. Every robot is parked again by camera after its run. A run that NEEDED RECOVERY (the camera lost it, it
     left the board, a safety stop cut its route short, or it ended more than 40 cm from its start) is noted
     as such (runs.csv `recovery`), never as an outlier: outliers are the analysis's statistical call within a
     condition. A camera watchdog stops any robot heading for an open edge or a parked robot. Nick is called
     only when a robot is stuck or off the board, with exact steps; with nobody answering for --help-wait-min
     that robot is put aside and the others carry on.
Everything is saved after every step (e02_state.json), so after a crash `--resume` carries on: an
interrupted run is redone.

Files in the session folder (logs/e02-<date-time>/): e02_state.json, runs.csv (one row per run: camera
start / far / end, errors, flags, corrections used), track.csv (the moving robot's camera positions),
fits.csv (every self-calibration), e02_log.txt (everything the console said), naming_check.json,
setup.json, robot logs in robot-logs-<n>/ (the console's CSVs: raw, bump, find, e02, leg ...).

PROTOCOL: firmware 2026-10-22a (docs/data-formats.md "E02", firmware/home_lab/e02.h); the runner refuses other
firmware. What this runner sends:
  set safe_edge 1, edge_dir 0, edge_thr 150, edge_n 3, bumper_cm 0, e02_face_cm 0  (SAFETY: at setup, checked by
      `get`, and with every find / route, checked in its PARAM records; the firmware refuses to drive without
      the edge guard)
  calibrate (once after power-on; calib_spin_deg 0, calib_roll_cm 0: only the compass wiggle)
  set gyro_scale / gyro_drift_dps / wheel_scale  (finds, transits and moves: the latest learned, else the priors;
      K: 1, 0, 1) and e02_route_gyro_scale / _gyro_drift_dps / _wheel_scale (U: 1, 0, 1; C, KC: learned): the
      firmware switches to these at the route's start, so only the route runs on the condition's corrections. `set`, not `corr reset`: same values, no rewrite of the saved corrections
  set e02_xboard_side / e02_yboard_side (-1 | +1), e02_xboard_cm / e02_yboard_cm (camera-measured faces),
      e02_start_x / e02_start_y (route start spot), e02_route_deg (straight: along the diagonal; L, square:
      away from the y-board), e02_l_turn (towards the middle), e02_leg_pause_ms 700 (all mode-0 runs: KC
      fixes are only used while still), e02_transit_max_cm 140,
      e02_pause_ms 1000, e02_route, e02_mode
  fix x y th|- t_wall   before every find, every set of idle legs and every backwards leg (replaces the pose; it
      also resets the firmware's blind-backing allowance); KC: up to 3 a second while it stands still, from
      pictures taken after it stopped, heading only when the camera just saw a straight stretch and RAW yaw
      says no turn since
  run find; run e02; go d id / turn deg id (idle legs, and mode 1 legs in K); mark start|far|end id (K)
  Records read by column name: RUNINFO, RUNEND, PARAM, RAW (ms, x_cm, y_cm, yaw_deg, th_deg), FIND (result,
  after_th), BUMP (board, ms_contact), E02 (start_x/y, far_goal_x/y, end_x/y, ms_start, ms_far,
  fixes_used), LEG (id, run, result or state), GYRO (raw_drift_dps).
"""
import argparse
import csv
import datetime as dt
import glob
import json
import math
import os
import subprocess
import re
import sys
import threading
import time
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

import e02_plan as PL  # noqa: E402
import e02_steer as ST  # noqa: E402
import e02_fit as FIT  # noqa: E402
import e02_camera as CAM  # noqa: E402
import e02_map as MAP  # noqa: E402

PAUSE_MS = 1000
FIX_HZ = 3.0
DRIVE_CMS_GUESS = 8.0
LEG_PAUSE_MS = 700        # e02_leg_pause_ms in every mode-0 run: still long enough for a camera fix (KC)
TRANSIT_MAX_CM = 140      # e02_transit_max_cm: park spots are up to ~110 cm from the find's staging point
BACK_MAX_CM = 9.5         # firmware e02_back_max_cm is 10: backwards legs are split into pieces this long
TURN_DPS_GUESS = 45.0
ID_TURN_DEG = 40.0        # identify: turn this much on the spot and back (2026-10-22)
HOLD_CM = 6.0             # between legs the followed robot stands still: the camera only accepts it this near
CHECK_CM, CHECK_FRAC = 5.0, 0.35   # camera vs wheels: a go leg's distances may differ by this much...
ID_EXTRA_CM = 5.0                  # ...plus this on the first leg after a turn on the spot (its centre is rougher)
PARK_LEG_CM = 60.0        # parking: long legs on the robot's own wheels and gyro, the camera checks in between
STALE_S = 1.5             # a camera position older than this is not used to steer
ASIDE_PARK_CM = 15.0      # a robot put aside further than this from its park spot is driven there (own pose)
BLIND_CLEAR_CM = 35.0     # ... leaving the corner: first to this far from both boards
BLIND_PIECE_CM = 15.0     # driving blind (own wheels and gyro): drives no longer than this
BLIND_EDGE_CM = 15.0      # ... never to a point nearer an open edge than this
BLIND_RETRACE_LEGS = 6    # ... retracing: its last this many drives
BLOCKS_FIND_CM = 35.0     # a robot put aside this near the find's staging point blocks every find
RESCUE_EDGE_CM = 15.0     # a lost robot finds itself against the boards only if last seen this far in from the open edges (park spots: 17+)
SWEEP_STEP_DEG = 30.0     # a finder robot turns in these steps, reading its front sensor after each
SWEEP_RANGE_CM = 70.0     # ... a reading nearer than this
SWEEP_GAP_CM = 8.0        # ... and this much shorter than the map says is something not on the map
SWEEP_NEAR_CM = 25.0      # the camera then looks for the lost robot's turn within this of it
FRONT_OFFSET_CM = 0.0     # front sensor reading minus the map's distance (data before the session; set it from the
                          # next real session's front_checks.csv: none yet, the simulator agrees within 1 cm)
FRONT_TOL_CM = 8.0        # ... and how far a reading may be from that before it counts as a disagreement
FRONT_STEER_FACTOR = 1.5  # a disagreement this many tolerances, with a measured heading, makes the runner find it again
FRONT_LEARN_MIN = 8       # --front-learn: readings before the learned offset and tolerance are used
FRONT_LEARN_MIN_TOL = 3.0
FRONT_MIN_COS = 0.7       # the front-sensor check only counts boards met within 45 deg of head-on
FRONT_CHECK_RANGE_CM = 50.0  # the front-sensor check only counts things nearer than this
FRONT_CHECK_CM = 6.0      # ... and says so when the sensor and the map differ by more
EXPECT_TURN_CM = 7.0       # a robot turning on its spot stays within this of where it stood (camera's view)
EXPECT_GO_EXTRA_CM = 8.0   # ... a drive of d cm within d + this
AUTO_TRIES = 3            # a lost or stuck robot tries this many times on its own before Nick is called
BACK_OFF_CM = 8.0         # a leg stopped by its front sensor backs this far before turning away (2026-10-22 loop)
EDGE_TURN_CM = 9.0        # no turn on the spot closer than this to an open edge (the wheels reach it first)
FIND_TIMEOUT_S = 150
E02_TIMEOUT_S = {0: 150, 1: 400}
BATTERY_WARN, BATTERY_STOP = 40, 15
EXPECTED_FW = "2026-10-22b"
# The edge guard (line sensors) and the bare body. safe_edge is not saved on the robot: set after every power-on.
# edge_thr 150 is CHOSEN, not measured at a real open edge: plywood reads 37-84 (roam1 RAW, ~70 000 rows), so a
# change of 150 is far outside its noise; what the sensors read past the edge has not been logged yet. The camera
# watchdog is the second, independent guard.
SAFETY_SETTINGS = (("safe_edge", 1.0), ("edge_dir", 0.0), ("edge_thr", 150.0), ("edge_n", 3.0), ("bumper_cm", 0.0),
                   ("e02_face_cm", 0.0))
SAFETY_CMDS = ["set %s %g" % kv for kv in SAFETY_SETTINGS]
DEFAULT_CALIB = os.path.join(HERE, "data", "overhead_floorcal_roam1.json")   # roam1 board corners, camera unmoved


def num(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def hms(s):
    s = max(0, int(s))
    return "%d:%02d" % (s // 3600, (s % 3600) // 60)


# =============================================================================== robot side listener
class Listener:
    """Everything the robots send that the runner needs, read by column name (the firmware lane may add
    columns or change their order)."""

    def __init__(self, session):
        self.s = session
        self.cv = threading.Condition()
        self.raw = defaultdict(list)          # run id -> [dict(t_recv, ms, x_cm, y_cm, th_deg, yaw_deg)]
        self.recs = defaultdict(list)         # type -> [(t_recv, robot, dict)]
        self.runinfo = {}                     # run id -> dict
        self.battery = {}                     # robot -> (t, %)
        self.firmware = {}
        self.msgs = []                        # (t, robot, text)
        self.lab = None                       # e02_live.LiveLab, fed every RAW row

    def __call__(self, name, kind, rtype, vals):
        t = time.time()
        with self.cv:
            if kind == "M":
                text = vals if isinstance(vals, str) else ",".join(vals)
                self.msgs.append((t, name, text))
                if len(self.msgs) > 2000:
                    del self.msgs[:1000]
                low = text.lower()
                if "battery" in low:
                    k = low.find("battery")
                    digits = "".join(ch for ch in low[k + 7:k + 13] if ch.isdigit())
                    if digits:
                        self.battery[name] = (t, int(digits))
                if "(firmware " in text:
                    k = text.find("(firmware ")
                    self.firmware[name] = text[k + 10:text.find(")", k)]
            elif kind == "D":
                cols = self.s.headers.get(rtype)
                rec = dict(zip(cols, vals)) if cols and len(cols) == len(vals) else {}
                if rtype == "RAW" and rec and self.lab is not None:
                    try:
                        self.lab.on_raw(name, rec, t)
                    except Exception:                 # the live lab is a display: never break the listener
                        pass
                if rtype == "RAW" and rec:
                    run = rec.get("run")
                    lst = self.raw[run]
                    if len(lst) < 40000:
                        lst.append(dict(t_recv=t, ms=num(rec.get("ms")), x_cm=num(rec.get("x_cm")),
                                        y_cm=num(rec.get("y_cm")), th_deg=num(rec.get("th_deg")),
                                        yaw_deg=num(rec.get("yaw_deg"))))
                elif rtype == "RUNINFO" and rec:
                    self.runinfo[rec.get("run")] = rec
                    b = num(rec.get("battery"))
                    if b is not None:
                        self.battery[name] = (t, int(b))
                elif rtype == "RUNEND" and rec:
                    b = num(rec.get("battery"))
                    if b is not None:
                        self.battery[name] = (t, int(b))
                elif rtype in ("BUMP", "FIND", "E02", "LEG", "GYRO", "CORR") and rec:
                    self.recs[rtype].append((t, name, rec))
            self.cv.notify_all()

    def latest(self, rtype, robot, after=0.0, run=None):
        with self.cv:
            for t, n, rec in reversed(self.recs[rtype]):
                if n == robot and t >= after and (run is None or rec.get("run") == run):
                    return t, rec
        return None

    def wait_rec(self, rtype, robot, after, timeout, run=None, poll=None):
        end = time.time() + timeout
        while time.time() < end:
            r = self.latest(rtype, robot, after, run)
            if r:
                return r
            with self.cv:
                self.cv.wait(0.1)
            if poll and poll():
                return None
        return None

    def leg_final(self, robot, run, lid):
        """The final LEG record (done / failed / refused / dup) of leg id `lid` in `run`, or None."""
        with self.cv:
            for t, n, rec in reversed(self.recs["LEG"]):
                state = rec.get("state") or rec.get("result")
                if n == robot and rec.get("id") == str(lid) and rec.get("run") in (run, "-") and \
                        state in ("done", "failed", "refused", "dup"):
                    return dict(rec, state=state)
        return None

    def wait_leg(self, robot, run, lid, after, timeout, stop=None):
        end = time.time() + timeout
        while time.time() < end:
            r = self.leg_final(robot, run, lid)
            if r:
                return r
            with self.cv:
                self.cv.wait(0.1)
            if stop and stop():
                return self.leg_final(robot, run, lid)
        return None

    def got_params(self, robot, since):
        """{name: value} from `get` answers ("M,name = value ...") since a time."""
        out = {}
        with self.cv:
            for t, n, m in self.msgs:
                if n != robot or t < since or " = " not in m:
                    continue
                k, _, rest = m.partition(" = ")
                k = k.strip().split(",")[-1].strip()
                try:
                    out[k] = float(rest.split()[0])
                except (ValueError, IndexError):
                    pass
        return out

    def said(self, robot, since, words):
        with self.cv:
            return [m for t, n, m in self.msgs if n == robot and t >= since and any(w in m.lower() for w in words)]


# =============================================================================== the runner
class Runner:
    def __init__(self, a, state, session_dir):
        self.a = a
        self.st = state
        self.dir = session_dir
        self.logf = open(os.path.join(session_dir, "e02_log.txt"), "a", encoding="utf-8")
        self.t_launch = time.time()
        self.phase = "setup"
        self.order = ""
        self.active = None
        self.trackf = open(os.path.join(session_dir, "track.csv"), "a", newline="")
        self.trackw = csv.writer(self.trackf)
        if os.path.getsize(os.path.join(session_dir, "track.csv")) == 0:
            self.trackw.writerow(["t", "robot", "x_cm", "y_cm", "order", "phase"])
        self.heads = {}               # robot -> [heading deg or None, measured bool]
        self.unavailable = set()
        self.last_result = ""
        self.kc_stop = threading.Event()
        self.marks_unconfirmed = 0
        self.watchdog_fired = False
        self.watch_stop = None
        self.watch_quit = threading.Event()
        self.moves = {}               # robot -> dict(run ("-" = idle legs), why): it is taking legs now
        self.leg_ids = {}
        self.last_lid = {}            # robot -> id of the last leg sent
        self.main_lid = {}            # robot -> id of the last leg() asked for (not a refind's turns)
        self.watch_count = defaultdict(int)  # robot -> watchdog stops in the current drive
        self.disagree = defaultdict(int)  # robot -> camera / wheels disagreements in a row
        self.fresh_id = {}            # robot -> True: its position came from identify() and it has not driven since
        self.watch_edge_cm = None     # the watchdog's open-edge distance (None: PL.WATCH_EDGE_CM; mapping: 0)
        self.bricks_view = None       # the brick demo's drawing on the live view (bricks, path, tracks), or None
        self.bricks_guard = None      # the brick demo's free_space while objects are on the board (camera watchdog)
        self.link = self.listener = self.watch = self.pi = None
        self.feed = self.tracker = self.history = self.cam = self.board = None
        self.front_seen = {}                 # "all" -> [front reading - map distance]
        self.front_bad = {}                  # robot -> words: the last leg's front reading disagreed with the map
        self.lost_place = set()              # robots that moved unseen with no trusted pose since (a failed own find)

    # --- talking ----------------------------------------------------------------------------------
    def say(self, msg, quiet=False):
        line = "[%s] %s" % (dt.datetime.now().strftime("%H:%M:%S"), msg)
        if not quiet:
            print(line, flush=True)
            if getattr(self, "view", None) is not None:
                self.view.log(line)
        self.logf.write(line + "\n")
        self.logf.flush()

    def planned_schedule(self):
        """The schedule this session will run (the same one before setup, to show, and when it starts)."""
        a = self.a
        nb = a.blocks or PL.blocks_for_time(a.hours, a.run_s, a.find_s, PL.SETUP_S, len(self.st["robots"]))
        sched = PL.make_schedule(self.st["robots"], nb, self.st["seed"])
        if a.routes != "1,2,3":
            keep = {int(v) for v in a.routes.split(",")}
            sched = [r for r in sched if r["route"] in keep]
            for i, r in enumerate(sched):
                r["order"] = i + 1
        return sched

    def recorder_ok(self):
        """The Pi recorder at a block start (or after 3 unconfirmed marks): the overhead stream and the notes link
        must answer. If not: pause, tell Nick, check again every 30 s until both answer (no runs without the
        video that measures them)."""
        a = self.a
        while True:
            ok1, w1, _, _ = CAM.check_stream(a.pi, a.overhead_port)
            notes = CAM.notes_answer(a.pi)
            if ok1 and notes:
                self.marks_unconfirmed = 0
                return True
            self.say("PAUSED: the Pi recorder is not answering (overhead camera: %s; notes: %s). Check the Pi's SSH "
                     "window: if camrec.py stopped, start it again with the same command as at the start. This "
                     "checks again every 30 s and carries on by itself." % ("OK" if ok1 else w1,
                                                                         "OK" if notes else "no answer"))
            self.st.setdefault("recorder_pauses", []).append([round(time.time(), 1), ok1, notes])
            self.st.save()
            print("\a", end="", flush=True)
            time.sleep(30.0)

    def mark(self, text):
        """A note in both camera videos (camrec.py writes it to each camera's events.csv)."""
        ok = self.pi.send(text) if self.pi else False
        self.marks_unconfirmed = 0 if ok else self.marks_unconfirmed + 1
        self.say("  [video mark] %s%s" % (text, "" if ok else "  (the Pi didn't confirm it)"), quiet=ok)
        self.st.setdefault("notes", []).append([round(time.time(), 3), text, bool(ok)])

    def ask(self, prompt, default="y"):
        if self.a.auto:
            self.say(prompt + " -> " + default + " (automatic)")
            return default
        try:
            return input(prompt).strip()
        except EOFError:
            return default

    def ask_timeout(self, prompt, timeout_s):
        """input() that gives up after timeout_s (None): an unattended session must never wait for ever."""
        if self.a.auto:
            return None
        box = []
        th = threading.Thread(target=lambda: box.append(self._input(prompt)), daemon=True)
        th.start()
        th.join(timeout_s)
        return box[0] if box else None

    @staticmethod
    def _input(prompt):
        try:
            return input(prompt).strip()
        except EOFError:
            return ""

    def send(self, robot, cmd):
        self.link.send_name(robot, cmd)
        self.say("    -> %s: %s" % (robot, cmd), quiet=True)

    # --- camera -----------------------------------------------------------------------------------
    def on_sample(self, s):
        if getattr(self, "lab", None) is not None and self.tracker is not None and self.tracker.active:
            try:
                self.lab.on_camera(self.tracker.active, s[0], s[1], s[2])
            except Exception:
                pass
        if self.tracker is not None and self.tracker.active:
            self.trackw.writerow(["%.3f" % s[0], self.tracker.active, "%.2f" % s[1], "%.2f" % s[2], self.order, self.phase])

    def pos(self, robot=None, fresh_s=1.5):
        p = self.tracker.where()
        if p is None or (robot and self.tracker.active != robot):
            return None
        if time.time() - p[0] > fresh_s:
            return None
        return p

    def settle(self, t_from=None, min_s=0.4, timeout=3.0):
        """The active robot's position once it stands still (median of min_s), or its latest one if the camera saw
        it in the last STALE_S. None if the camera has not seen it lately (the caller finds it again)."""
        self.camera_alive()
        end = time.time() + timeout
        while time.time() < end:
            sp = CAM.still_position(self.history.since(max(t_from or 0, time.time() - 2.0)), min_s=min_s)
            if sp:
                return sp
            time.sleep(0.1)
        p = self.tracker.where()
        if p is None or self.tracker.last_seen < time.time() - STALE_S:
            return None
        return (p[1], p[2])

    def camera_alive(self):
        """The camera feed thread must be running: if it died (an error in a frame), start a new one (review
        2026-10-22: a dead feed froze every position without anyone noticing)."""
        f = self.feed
        if f is None or self.a is None or getattr(self.a, "dry_run_feed_ok", False):
            return True
        if f.is_alive():
            return True
        self.say("  (the camera feed stopped: %s; starting it again)" % (f.err or "no reason given"))
        nf = CAM.Feed(stream_url(self.a), self.tracker, self.history,
                      log=lambda m: self.say("  (camera: %s)" % m))
        nf.on_sample = f.on_sample
        nf.hooks = f.hooks
        nf.start()
        self.feed = nf
        nf.wait_frames(3, timeout=8)
        return False

    # --- runs and camera-steered moves -------------------------------------------------------------
    def start_run(self, robot, cmd="run e02", retry=True):
        """Send a `run ...` and wait for its RUNINFO. A robot that says it is not calibrated (switched off and
        on) gets its calibration first, then one more try. Returns the run id or None."""
        before = self.watch.started.get(robot)
        t0 = time.time()
        if self.tracker is not None and self.tracker.active == robot:
            self.tracker.clear_expect()          # a run drives it: the camera must follow it anywhere
        self.send(robot, cmd)
        run = self.watch.wait_start(robot, before, 8)
        if run:
            return run
        said = self.listener.said(robot, t0, ("not calibrated", "busy", "unknown", "run what", "refuse"))
        if retry and any("not calibrated" in m.lower() for m in said):
            self.say("  %s was switched off and on since its calibration: calibrating it first." % robot)
            self.calibrate([robot])
            return self.start_run(robot, cmd, retry=False)
        if retry and any("busy" in m.lower() for m in said):
            self.send(robot, "stop")
            time.sleep(1.5)
            return self.start_run(robot, cmd, retry=False)
        self.say("  %s did not start `%s`%s" % (robot, cmd, (": " + said[-1]) if said else ""))
        return None

    def calibrate(self, robots):
        """The compass check after power-on (`calibrate`, needed before find / e02 / gyro runs): spin and roll
        switched off, so it only wiggles in place on its park spot. All at once (each stays on its spot)."""
        robots = [r for r in robots if r not in self.unavailable]
        if not robots:
            return
        self.mark("e02 calibrate %s" % ",".join(robots))
        for r in robots:
            self.end_moves(r)
            for c in ["set calib_spin_deg 0", "set calib_roll_cm 0", "set calib_roll_min_cm 99"] + SAFETY_CMDS:
                self.send(r, c)
                time.sleep(0.05)
        runs = {}
        for r in robots:
            before = self.watch.started.get(r)
            self.send(r, "calibrate")
            runs[r] = (before, time.time())
        for r, (before, t0) in runs.items():
            run = self.watch.wait_start(r, before, 10)
            end = self.watch.wait_end(run, 90) if run else None
            self.say("  %s calibration (compass check): %s" % (r, end["result"] if end else "no answer"))
            self.st.setdefault("calibrations", []).append([round(t0, 1), r, run, end["result"] if end else None])
            if r in self.heads and self.heads[r][0] is not None:
                self.heads[r][1] = False             # the wiggle may have turned it a little
        self.st.save()

    def fix_now(self, robot, heading=None):
        """Tell the robot where the camera sees it (`fix x y th t_wall`; while idle it replaces its pose)."""
        if self.tracker is None or self.tracker.active != robot:
            return None                  # the camera follows another robot (or none): no position for this one
        p = self.settle(min_s=0.3)
        if p is None:
            return None
        h = heading if heading is not None else self.heads.get(robot, [None])[0]
        self.send(robot, "fix %.2f %.2f %s %.3f" % (p[0], p[1], "-" if h is None else "%.1f" % h, time.time()))
        time.sleep(0.15)
        return p

    def begin_moves(self, robot, why="moving"):
        """Camera-steered moves that are not measurements (parking, finding a robot, putting it back) are idle
        legs: `go` / `turn` with no run going (firmware 2026-10-21b: a board close in front no longer fails them,
        so the 2026-10-21a workaround of carrying them inside a camera-steered `run e02` is gone; idle legs are
        the safer of the two: each has its own travel cap of the leg + 10 cm, against the route + 140 cm + 30 cm
        of a run, and they leave no fake e02 runs in the logs). A camera fix first, so its own pose is sane."""
        if robot in self.moves:
            return True
        self.set_corrections(robot, self.move_corrections(robot))
        h = self.heads.get(robot, [None])[0]
        self.fix_now(robot, h if h is not None else self.board.face_xboard)
        self.moves[robot] = dict(run="-", why=why)
        return True

    def wait_mode1(self, robot, t0, timeout=8.0):
        """A camera-steered run takes legs only once it says so (after its settle pause and start flash)."""
        end = time.time() + timeout
        while time.time() < end:
            if self.listener.said(robot, t0, ("waiting for go",)):
                return True
            time.sleep(0.1)
        return False

    def end_moves(self, robot):
        m = self.moves.pop(robot, None)
        if not m or m["run"] == "-":
            return
        self.send(robot, "mark end %d" % self.next_id(robot))
        if self.watch.wait_end(m["run"], 6) is None:
            self.send(robot, "stop")
            self.watch.wait_end(m["run"], 3)

    def next_id(self, robot):
        """Leg ids only ever go up (the firmware answers `dup` to an id it has seen: a resend is safe)."""
        self.leg_ids[robot] = self.leg_ids.get(robot, int(time.time()) % 100000 * 10) + 1
        return self.leg_ids[robot]

    def send_leg(self, robot, text, timeout):
        """One leg or mark with the next id, resent once (same id) if no answer. Returns (state, info)."""
        m = self.moves.get(robot)
        if m is None:
            return "refused", "not moving"
        lid = self.next_id(robot)
        self.last_lid[robot] = lid
        following = self.tracker is not None and self.tracker.active == robot
        if following:
            self.tracker.clear_expect()          # it may move now
            self.expect_leg(text, timeout)
        for attempt in range(2):
            t_s = time.time()
            self.send(robot, "%s %d" % (text, lid))
            stop = (lambda: self.watch.ended.get(m["run"]) is not None) if m["run"] != "-" else None
            res = self.listener.wait_leg(robot, m["run"], lid, t_s - 0.05, timeout, stop=stop)
            if res and following and res.get("state") in ("done", "failed", "dup"):
                time.sleep(0.3)                  # it has stopped: from now on it can't be anywhere else
                self.tracker.hold(HOLD_CM)
            if res:
                if res.get("state") == "refused" and "running" in (res.get("info") or "") and attempt == 0:
                    time.sleep(1.0)              # an idle leg still finishing: once more
                    continue
                return res.get("state", "?"), res.get("info", "")
            if m["run"] != "-" and self.watch.ended.get(m["run"]) is not None:
                self.moves.pop(robot, None)
                return "refused", "its camera-steered run ended (%s)" % self.watch.ended[m["run"]]["result"]
        return "lost", "no answer"

    def front_check(self, robot, p, words, kind, val):
        """Nick, 2026-10-22: the front sensor as a free check of the position. After every finished leg the firmware
        (2026-10-22b) reports the front sensor's middle zone ("front=<cm>"); here it is compared with what the camera
        position, the heading and the map say is ahead (a board line, or another robot), and written to
        front_checks.csv. Only recorded for now (the numbers from a session decide the limits before it steers
        anything); a large disagreement is said quietly."""
        m = re.search(r"front=([0-9.]+)", words or "")
        h = self.heads.get(robot, [None, False])
        if not m or p is None or h[0] is None or self.board is None:
            return None
        front = float(m.group(1))
        if front <= 0:
            return None                           # no reading (nothing in range)
        hd = ST.wrap180(h[0] + val) if kind == "turn" else h[0]      # heads[] holds the heading before the leg
        ahead = self.ahead_cm(robot, p, hd)
        row = [round(time.time(), 2), robot, kind, round(val, 1), round(p[0], 1), round(p[1], 1), round(hd, 1),
               int(bool(h[1])), front, "" if ahead is None else round(ahead[0], 1), "" if ahead is None else ahead[1]]
        path = os.path.join(self.dir, "front_checks.csv")
        new = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["t", "robot", "leg", "value", "x_cm", "y_cm", "heading_deg", "heading_measured",
                            "front_cm", "expected_cm", "what"])
            w.writerow(row)
        if ahead is not None and ahead[0] <= FRONT_CHECK_RANGE_CM and self.square_on(ahead):
            off, tol = self.front_limits()
            miss = front - off - ahead[0]
            if h[1]:
                self.front_seen.setdefault("all", []).append(front - ahead[0])
            if abs(miss) > tol:
                self.say("    %s: front sensor %.0f cm, the camera and map say %s at %.0f cm" % (
                    robot, front, ahead[1], ahead[0]), quiet=not (h[1] and abs(miss) > FRONT_STEER_FACTOR * tol))
                if h[1] and abs(miss) > FRONT_STEER_FACTOR * tol:
                    # a measured heading, and the sensor far from the map: the camera position is suspect
                    self.front_bad[robot] = "the front sensor reads %.0f cm, the map says %s at %.0f cm" % (
                        front, ahead[1], ahead[0] + off)
        return front, ahead

    def front_limits(self):
        """(offset, tolerance) cm for the front sensor against the map. Nick, 2026-10-22: for now from data gathered
        before the session (FRONT_OFFSET_CM / FRONT_TOL_CM, to be set from the front_checks.csv of the next real
        session); --front-learn (off by default) learns them in this session instead, from scratch, once
        FRONT_LEARN_MIN readings are in: the median of reading - map, and 3 x its spread (MAD x 1.48), at least
        FRONT_LEARN_MIN_TOL. NOTE (Nick): switch to --front-learn once the from-scratch setup can stand on it."""
        v = self.front_seen.get("all") or []
        if getattr(self.a, "front_learn", False) and len(v) >= FRONT_LEARN_MIN:
            med = float(np.median(v))
            mad = float(np.median([abs(x - med) for x in v])) * 1.4826
            return med, max(FRONT_LEARN_MIN_TOL, 3.0 * mad)
        return FRONT_OFFSET_CM, FRONT_TOL_CM

    def ahead_cm(self, robot, p, heading):
        """Distance from the robot's front face to the first thing straight ahead, by the map: (cm, what) or None.
        Boards are lines; other robots are discs of PL.ROBOT_HALF_CM."""
        b = self.board
        ux, uy = math.cos(math.radians(heading)), math.sin(math.radians(heading))
        best = None
        for what, axis, v in (("the y-board", "x", b.yboard_cm), ("the x-board", "y", b.xboard_cm)):
            d = (v - p[0]) / ux if axis == "x" and abs(ux) > 1e-6 else ((v - p[1]) / uy if axis == "y" and abs(uy) > 1e-6
                                                                      else -1)
            if d > 0 and (best is None or d < best[0]):
                best = (d, what, abs(ux) if axis == "x" else abs(uy))     # cos of the angle to the board's normal
        for n, q in self.st["positions"].items():
            if n == robot or not q:
                continue
            ox, oy = q[0] - p[0], q[1] - p[1]
            t = ox * ux + oy * uy
            off = abs(-ox * uy + oy * ux)
            if t > 0 and off < PL.ROBOT_HALF_CM:
                d = t - math.sqrt(max(0.0, PL.ROBOT_HALF_CM ** 2 - off ** 2))
                if best is None or d < best[0]:
                    best = (d, n, 1.0)
        if best is None:
            return None
        return best[0] - PL.ROBOT_HALF_CM, best[1], best[2]

    @staticmethod
    def square_on(ahead):
        """Is the thing ahead met steeply enough for the front sensor to read it? (dry run 15:35: at 19 deg to a
        board the sensor read 92 cm where the board was 42 cm away; its zones see past a board met that flat)"""
        return ahead is not None and (len(ahead) < 3 or ahead[2] >= FRONT_MIN_COS)

    def expect_leg(self, text, timeout):
        """Where the followed robot can be during this leg (2026-10-22: during a turn on its spot the tracker was
        free to take another robot 34 cm away): a turn keeps it within EXPECT_TURN_CM of where it stands; a drive
        of d cm within d + EXPECT_GO_EXTRA_CM. A camera-steered run (no tracker.where()) is left free."""
        p = self.tracker.where()
        if p is None:
            return
        parts = text.split()
        try:
            verb, val = parts[0], abs(float(parts[1]))
        except (IndexError, ValueError):
            return
        if verb == "turn":
            r = EXPECT_TURN_CM
        elif verb in ("go", "probe"):
            r = val + EXPECT_GO_EXTRA_CM
        else:
            return
        self.tracker.expect(p[1], p[2], r, for_s=timeout + 3.0)

    def room_behind(self, robot, p, back_cm):
        """Backing up is blind (no edge guard, no front sensor): is there room behind for back_cm, by the camera?
        Returns plain words of what is in the way, or ''. Unknown heading: assume the worst way it could face."""
        h = self.heads.get(robot, [None])[0]
        if p is None:
            return "the camera can't see it"
        heads = [h] if h is not None else list(range(0, 360, 30))
        parked = {n: tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q}
        for hh in heads:
            ux, uy = math.cos(math.radians(hh)), math.sin(math.radians(hh))
            for f in (0.5, 1.0):
                cx, cy = p[0] - ux * back_cm * f, p[1] - uy * back_cm * f
                rx, ry = cx - ux * PL.ROBOT_HALF_CM, cy - uy * PL.ROBOT_HALF_CM
                if not self.board.on_board(rx, ry, margin=2.0):
                    return "an open edge behind it"
                for n, q in parked.items():
                    if math.hypot(cx - q[0], cy - q[1]) < PL.WATCH_ROBOT_CM:
                        return "%s parked behind it" % n
        return ""

    def leg(self, robot, cmd, run=None):
        """One go / turn leg. A backwards go goes in pieces of at most BACK_MAX_CM; before each the camera checks
        the room behind it and a camera fix is sent (the firmware allows e02_back_max_cm of blind backing per
        fix). Returns (p0, p1, words)."""
        kind, val = ("go" if cmd[0] == "probe" else cmd[0]), float(cmd[1])
        p0 = self.settle(min_s=0.3, timeout=1.5)
        pieces = [val]
        if kind == "go" and val < -BACK_MAX_CM:
            n = int(math.ceil(-val / BACK_MAX_CM))
            pieces = [val / n] * n
        words = ""
        self.watch_stop = None
        for v in pieces:
            if kind == "go" and v < 0:
                pb = self.settle(min_s=0.3, timeout=1.5)
                bad = self.room_behind(robot, pb, -v)
                if bad:
                    words = "refused (no room to back up: %s)" % bad
                    break
                self.fix_now(robot)
            for attempt in range(2):
                if robot not in self.moves and not self.begin_moves(robot):
                    return p0, p0, "could not start camera-steered moves"
                est = abs(v) / DRIVE_CMS_GUESS + 1.0 if kind == "go" else abs(v) / TURN_DPS_GUESS + 1.0
                state, info = self.send_leg(robot, "%s %.2f" % (kind, v), 2.5 * est + 6.0)
                words = state + ((" (" + info + ")") if info else "")
                if state == "refused" and robot not in self.moves and attempt == 0:
                    continue                      # its run had ended (quiet too long): start another, try again
                if state == "refused" and "busy" in info and attempt == 0:
                    time.sleep(1.5)               # its run is still starting (settle pause, start flash)
                    continue
                if state == "refused" and "backing blind" in info and attempt == 0:
                    self.fix_now(robot)           # the fix did not arrive: once more
                    continue
                break
            if self.watch_stop:
                words = "watchdog (%s)" % self.watch_stop
                break
            if state not in ("done", "dup"):
                break
        time.sleep(0.25)
        self.main_lid[robot] = self.last_lid.get(robot)
        p1 = self.settle(t_from=time.time() - 0.6, min_s=0.3, timeout=2.0)
        self.front_check(robot, p1, words, kind, val)
        if kind == "go" and p0 is not None and len(pieces) == 1:
            bad = self.cross_check(robot, p0, p1)
            if bad:
                words = "camera and wheels disagree (%s)" % bad
                p1 = self.refind(robot, bad)
        return p0, p1, words

    def wheels_moved(self, robot, lid):
        """How far the robot's own wheels and gyro say it moved during leg lid (its LEG records' poses from
        'started' to the final one), or None."""
        a = b = None
        with self.listener.cv:
            for t, n, rec in self.listener.recs["LEG"]:
                if n != robot or rec.get("id") != str(lid):
                    continue
                st_ = rec.get("state") or rec.get("result")
                xy = (num(rec.get("x_cm")), num(rec.get("y_cm")))
                if None in xy:
                    continue
                if st_ == "started":
                    a = xy
                elif st_ in ("done", "failed"):
                    b = xy
        if a is None or b is None:
            return None
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def cross_check(self, robot, p0, p1):
        """The camera against the wheels (Nick, 2026-10-21): after a go leg, did the camera see it move about as
        far as its wheels say? Returns plain words of the disagreement, or ''. Agreement resets the count."""
        w = self.wheels_moved(robot, self.last_lid.get(robot))
        if w is None:
            return ""
        if p1 is None:
            return "the camera lost it while its wheels say it moved %.0f cm" % w
        c = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        extra = ID_EXTRA_CM if self.fresh_id.pop(robot, False) else 0.0   # p0 came from a turn on the spot
        if abs(c - w) > max(CHECK_CM, CHECK_FRAC * w) + extra:
            return "the camera saw it move %.0f cm, its wheels say %.0f cm" % (c, w)
        self.disagree[robot] = 0
        return ""

    def refind(self, robot, why):
        """The camera and the wheels disagree: stop, and find it again by a turn on its spot (nothing else turns
        just then). Returns its position, or None (then the caller gives up and, twice in a row, Nick is called)."""
        self.disagree[robot] += 1
        self.say("    %s: %s: finding it again by a short turn on its spot" % (robot, why))
        self.st.setdefault("disagreements", []).append([round(time.time(), 1), robot, why, self.order])
        self.send(robot, "stop")
        time.sleep(0.5)
        last = self.tracker.where()
        near = (last[1], last[2], 60.0) if last else None
        if last and self.board.edge_room(last[1], last[2])[0] < EDGE_TURN_CM:
            # a turn on the spot this near an open edge could put a wheel over it: back away first, if it is
            # known which way it faces; otherwise Nick
            h = self.heads.get(robot, [None])[0]
            if h is None:
                return None
            self.leg(robot, ("go", -6.0))
        p = self.identify(robot, near=near) or self.identify(robot)
        if p is None:
            return None
        self.fresh_id[robot] = True
        self.tracker.robot_at(p[0], p[1])
        self.tracker.follow(robot, p[0], p[1])
        self.history.clear()
        self.tracker.hold(HOLD_CM)
        self.heads[robot] = [self.heads.get(robot, [None])[0], False]
        return self.settle(min_s=0.3)

    def mark_leg(self, robot, which, timeout=8.0):
        state, info = self.send_leg(robot, "mark %s" % which, timeout + PAUSE_MS / 1000.0)
        return state

    def safe_path(self, robot, p, target, depth=0):
        """Waypoints from p to target that keep clear of every other robot the runner knows of (parked, or found
        and not parked yet: 2026-10-21 Center was steered straight into Right, not found yet). Detour points
        beside the robots in the way, on the side with room on the board; every piece checked; up to 2 detours.
        None if there is no clear way."""
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q]
        clear_cm = PL.PARK_CLEAR_CM + 4.0

        def first_block(a, b):
            dx, dy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(dx, dy)
            if L < 1e-6:
                return None
            ux, uy = dx / L, dy / L
            worst = None
            for o in others:
                # a robot already near the start or the end: only a piece that comes nearer to it than that is
                # blocked (leaving a neighbour, or arriving at a spot beside one, is allowed)
                near = min(math.hypot(a[0] - o[0], a[1] - o[1]), math.hypot(b[0] - o[0], b[1] - o[1])) - 0.5
                lim = min(clear_cm, near)
                t = max(0.0, min(L, (o[0] - a[0]) * ux + (o[1] - a[1]) * uy))
                d = math.hypot(a[0] + ux * t - o[0], a[1] + uy * t - o[1])
                if d < lim and (worst is None or t < worst[0]):
                    worst = (t, o, ux, uy)
            return worst

        blk = first_block(p, target)
        if blk is None:
            return [target]
        if depth >= 2:
            return None
        _, o, ux, uy = blk
        best = None
        for side in (1.0, -1.0):
            for off in (30.0, 40.0):
                q = (o[0] - uy * side * off, o[1] + ux * side * off)
                if not self.board.on_board(q[0], q[1], margin=15.0):
                    continue
                if min([math.hypot(q[0] - r[0], q[1] - r[1]) for r in others] or [99.0]) < clear_cm:
                    continue
                if first_block(p, q) is not None:
                    continue
                rest = self.safe_path(robot, q, target, depth + 1)
                if rest is None:
                    continue
                cost = math.hypot(q[0] - p[0], q[1] - p[1]) + math.hypot(target[0] - q[0], target[1] - q[1])
                if best is None or cost < best[0]:
                    best = (cost, [q] + rest)
        return best[1] if best else None

    def drive_to(self, robot, target, final_heading=None, run=None, max_legs=40, arrive_cm=ST.ARRIVE_CM,
                 max_leg=ST.MAX_LEG_CM, avoid=True):
        """To target (x, y) by single legs, each driven on the robot's own wheels and gyro, the camera picking the
        next leg and checked against the wheels after each; then optionally turned to final_heading. avoid: go
        round the other robots the runner knows of. Returns (ok, words)."""
        h = self.heads.get(robot, [None, False])
        self.watch_count[robot] = 0
        p = self.settle(min_s=0.3, timeout=2.0)
        wps = self.safe_path(robot, p, target) if (avoid and p is not None) else [target]
        if wps is None:
            return False, "no clear way round the other robots"
        if len(wps) > 1:
            self.say("    %s: going round another robot (via %.0f, %.0f)" % (robot, wps[0][0], wps[0][1]), quiet=True)
        steer = ST.Steer(wps, heading=h[0], measured=h[1], final_heading=final_heading, max_legs=max_legs,
                         arrive_cm=arrive_cm, max_leg=max_leg)
        stuck = 0
        blocked = 0
        self.front_bad.pop(robot, None)
        while True:
            p = self.settle(min_s=0.3, timeout=2.0)
            if p is None:
                p = self.refind(robot, "the camera can't see it")
                if p is None:
                    return False, "the camera can't see it, and a turn on its spot did not show it"
                steer.heading, steer.measured = None, False
            if not self.board.on_board(p[0], p[1]):
                return False, "the camera sees it off the board at (%.0f, %.0f) cm" % p
            cmd = steer.next(p)
            if cmd[0] == "done":
                self.heads[robot] = [steer.heading, steer.measured]
                return True, ""
            if cmd[0] == "gave_up":
                self.heads[robot] = [steer.heading, steer.measured]
                return False, "it did not get there in %d legs" % max_legs
            if cmd[0] == "turn" and abs(cmd[1]) < ST.MIN_TURN_DEG:
                self.heads[robot] = [steer.heading, steer.measured]
                return True, ""
            if steer.heading is not None:
                self.heads[robot] = [steer.heading, steer.measured]     # the front-sensor check reads it
            self.front_bad.pop(robot, None)
            p0, p1, res = self.leg(robot, cmd, run=run)
            fb = self.front_bad.pop(robot, None)
            if fb and p1 is not None and not res.startswith("camera and wheels"):
                p1 = self.refind(robot, fb)
                if p1 is None:
                    return False, fb
                steer.heading, steer.measured = None, False
                steer.legs += 1
                continue
            if res.startswith("camera and wheels"):
                if p1 is None or self.disagree[robot] >= 2:
                    return False, res
                steer.heading, steer.measured = None, False   # found again by a turn: heading unknown
                steer.legs += 1
                continue
            if p0 is None or p1 is None:
                return False, "the camera lost it"
            if res.startswith("watchdog"):
                self.heads[robot] = [steer.heading, False]
                self.watch_count[robot] += 1
                if self.watch_count[robot] >= 2:
                    self.watch_count[robot] = 0
                    return False, "the camera watchdog stopped it twice: " + res
                continue
            if res.startswith("refused") or res.startswith("could not"):
                return False, "it refused the leg: " + res
            steer.after(cmd, p0, p1)
            if "obstacle" in res and cmd[0] in ("go", "probe") and cmd[1] > 0:
                # its front stop saw something (a board, another robot): the heading guess was wrong or the way
                # is blocked. Back off first (2026-10-22: turning on the spot against a board, then planning straight
                # back into it, looped 13 times), then a quarter turn towards the board's middle, and plan again.
                # Counted on its own: the turn legs in between used to reset the count, so it never gave up.
                blocked += 1
                self.say("    %s: something in front (%s): backing off and turning away (%d)" % (robot, res, blocked))
                if blocked >= 3:
                    self.heads[robot] = [steer.heading, steer.measured]
                    return False, "something keeps blocking its way"
                self.heads[robot] = [steer.heading, False]   # the heading guess may be what was wrong
                pb0, pb1, _ = self.leg(robot, ("go", -BACK_OFF_CM), run=run)
                if pb1 is not None:
                    steer.after(("go", -BACK_OFF_CM), pb0 if pb0 is not None else p1, pb1)
                    p1 = pb1
                ang = self.away_turn(p1, steer.heading)
                self.leg(robot, ("turn", ang))
                steer.after(("turn", ang), p1, p1)
                self.front_bad.pop(robot, None)
                continue
            if ST.leg_stuck(cmd, p0, p1):
                stuck += 1
                self.say("    %s hardly moved on '%s %.1f' (%s)" % (robot, cmd[0], cmd[1], res))
                if stuck >= 3:
                    self.heads[robot] = [steer.heading, steer.measured]
                    return False, "it is stuck (it did not move when told to, 3 times)"
                if stuck == 2:   # try backing out
                    self.leg(robot, ("go", -5.0), run=run)
            else:
                stuck = 0

    def away_turn(self, p, heading):
        """A quarter turn, to whichever side points more towards the board's middle (a fixed +90 could turn it
        along a board or towards an open edge). +90 when the heading or position is not known."""
        if p is None or heading is None:
            return 90.0
        b = self.board
        cx, cy = (b.x0 + b.x1) / 2.0, (b.y0 + b.y1) / 2.0
        want = math.degrees(math.atan2(cy - p[1], cx - p[0]))
        best = None
        for a in (90.0, -90.0):
            e = abs(ST.wrap180(want - (heading + a)))
            if best is None or e < best[0]:
                best = (e, a)
        return best[1]

    # --- finding a robot with the camera ------------------------------------------------------------
    def board_rect(self):
        b = self.board or PL.Board()
        return (b.x0, b.y0, b.x1, b.y1)

    def identify(self, robot, near=None, angle=ID_TURN_DEG):
        """Where is `robot`? It turns a little on its spot and back (2026-10-21: a short DRIVE left its old spot
        behind, which the camera could take for the robot; a turn leaves nothing behind) and the camera looks for
        the one robot-sized change. Only this robot moves meanwhile. near = (x, y, radius cm) to look only there.
        Returns (x, y) or None. Its heading is not known after this (a body seen from above looks alike turned)."""
        self.mark("e02 identify %s" % robot)
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q]
        if self.tracker is not None and self.tracker.active == robot:
            self.tracker.idle()
        if robot not in self.moves:          # inside a camera-steered run (K) its legs stay in that run
            self.moves[robot] = dict(run="-", why="identify")
        for ang in (angle, -angle):
            before = self.feed.wait_frames(8, timeout=5)
            # lights on while it turns (2026-10-22, Nick): two changes in one place, the body's corners and the
            # lights; in a shadow, where the corners barely show, the lights carry it (firmware 2026-10-22a `led`)
            self.send(robot, "led 1 1 1 %d" % int((abs(ang) / TURN_DPS_GUESS * 2.5 + 4.0) * 1000))
            state, info = self.send_leg(robot, "turn %.1f" % ang, abs(ang) / TURN_DPS_GUESS * 2.5 + 6.0)
            time.sleep(0.6)
            after = self.feed.wait_frames(8, timeout=5)
            self.send_leg(robot, "turn %.1f" % -ang, abs(ang) / TURN_DPS_GUESS * 2.5 + 6.0)
            time.sleep(0.4)
            if state not in ("done", "dup") or len(before) < 3 or len(after) < 3:
                self.say("    (%s did not turn: %s %s)" % (robot, state, info), quiet=True)
                continue
            cands = CAM.locate_change(self.cam, before, after, board_rect=self.board_rect(), near=near)
            cands = [c for c in cands if all(math.hypot(c["x"] - o[0], c["y"] - o[1]) > 6.0 for o in others)]
            self.st.setdefault("identify", []).append([round(time.time(), 1), robot, ang,
                                                       [[round(c["x"], 1), round(c["y"], 1), c["area"],
                                                         round(c["strength"], 1)] for c in cands[:4]]])
            if not cands:
                continue
            if len(cands) > 1 and cands[1]["area"] * cands[1]["strength"] > 0.5 * cands[0]["area"] * cands[0]["strength"]:
                self.say("    (two places changed when %s turned: looking again)" % robot, quiet=True)
                continue
            return (cands[0]["x"], cands[0]["y"])
        # third try: its lights, steady (firmware 2026-10-22a `led`): pictures before and while they are on; the one
        # spot that got brighter. Works at the ~2 pictures a second the laptop gets (the 2 Hz flash below does not).
        if self.feed is not None and hasattr(self.feed, "collect"):
            before, _ = self.feed.collect(1.2)
            self.send(robot, "led 1 1 1 2500")
            time.sleep(0.5)
            during, _ = self.feed.collect(1.6)
            if len(before) >= 1 and len(during) >= 1:
                f = CAM.locate_light(self.cam, before, during, board_rect=self.board_rect(), near=near)
                self.st.setdefault("identify", []).append([round(time.time(), 1), robot, "light",
                                                           [round(f["x"], 1), round(f["y"], 1), round(f["strength"])]
                                                           if f else None, len(before), len(during)])
                if f and all(math.hypot(f["x"] - o[0], f["y"] - o[1]) > 6.0 for o in others):
                    self.say("    (found %s by its lights)" % robot)
                    return (f["x"], f["y"])
        # fourth try: the 2 Hz flash (`identify`), for a fast picture stream
        if self.feed is not None and hasattr(self.feed, "collect"):
            self.send(robot, "identify")
            imgs, ts = self.feed.collect(3.6)
            f = CAM.locate_flash(self.cam, imgs, ts, board_rect=self.board_rect(), near=near)
            self.st.setdefault("identify", []).append([round(time.time(), 1), robot, "flash",
                                                       [round(f["x"], 1), round(f["y"], 1), f["strength"]] if f else None])
            if f and all(math.hypot(f["x"] - o[0], f["y"] - o[1]) > 6.0 for o in others):
                self.say("    (found %s by its lights)" % robot)
                return (f["x"], f["y"])
        return None

    def acquire(self, robot, step_cm=10.0, near=None):
        """Find robot in the picture: identify it by a turn on its spot, then a short drive (forward; blocked:
        a quarter turn and again) gives its heading. Returns (x, y) or None."""
        p = self.identify(robot, near=near)
        self.moves.pop(robot, None)
        if p is None:
            return None
        self.st["positions"][robot] = [p[0], p[1], None]
        return self.acquire_heading(robot, p)

    def activate(self, robot):
        """Follow `robot` from where it was last parked (the tracker keeps every parked robot visible)."""
        self.active = robot
        sp = self.st["positions"].get(robot)
        p = self.tracker.where()
        if self.tracker.active == robot and p and (sp is None or math.hypot(p[1] - sp[0], p[2] - sp[1]) < 5):
            return True
        if sp is None:
            return False
        self.tracker.follow(robot, sp[0], sp[1])
        self.history.clear()
        self.tracker.hold(HOLD_CM)
        if robot not in self.heads:
            self.heads[robot] = [sp[2], False]
        time.sleep(0.3)
        return self.settle(min_s=0.3) is not None

    # --- parking ----------------------------------------------------------------------------------
    def park_index(self, robot):
        return self.st["robots"].index(robot)

    def park(self, robot, run=None):
        """Camera-steered to its park spot: to the staging point, turn to face the x-board, then back into the
        spot (the backwards leg measures the heading it ends with)."""
        i = self.park_index(robot)
        px, py, ph = self.board.park(i)
        sx, sy = self.board.stage(i)
        p = self.settle(min_s=0.3)
        if p is None:
            return False, "the camera can't see it"
        self.phase = "parking"
        esc = self.board.escape_point(p[0], p[1])
        if esc is not None:
            # near an open edge (a route that ended off course): first straight in from it, then on as usual
            self.say("    %s is %.0f cm from an open edge: moving it in first" % (robot, self.board.edge_room(*p)[0]))
            ok, why = self.drive_to(robot, esc, arrive_cm=4.0, max_leg=PARK_LEG_CM)
            if not ok:
                return ok, why
        if math.hypot(p[0] - px, p[1] - py) > 4.0:
            if math.hypot(p[0] - sx, p[1] - sy) > 3.0:
                ok, why = self.drive_to(robot, (sx, sy), arrive_cm=2.0, max_leg=PARK_LEG_CM)
                if not ok:
                    return ok, why
            h = self.heads.get(robot, [None, False])
            if h[0] is not None:
                e = ST.wrap180(ph - h[0])
                if abs(e) > ST.MIN_TURN_DEG:
                    self.leg(robot, ("turn", e))
                self.heads[robot] = [ph, False]
            p = self.settle(min_s=0.3)
            if p is None:
                return False, "the camera lost it"
            ux, uy = math.cos(math.radians(ph)), math.sin(math.radians(ph))
            along = (px - p[0]) * ux + (py - p[1]) * uy           # negative: the spot is behind it
            if abs(along) > 2.0:
                p0, p1, _ = self.leg(robot, ("go", along))
                if p0 and p1:
                    st_ = ST.Steer([], heading=self.heads[robot][0])
                    st_.after(("go", along), p0, p1)
                    self.heads[robot] = [st_.heading, st_.measured]
                    if ST.leg_stuck(("go", along), p0, p1):
                        return False, "it did not move back into its spot"
        ok, why = self.drive_to(robot, (px, py), final_heading=ph, arrive_cm=3.0, avoid=False)
        self.end_moves(robot)
        if ok:
            p = self.settle(min_s=0.3)
            self.st["positions"][robot] = [p[0], p[1], self.heads.get(robot, [None])[0]]
            self.tracker.park(robot, p[0], p[1])
            self.st.save()
        return ok, why

    def retry_find(self, robot, park=True):
        """One try at getting a robot back with nobody's hands: a fresh picture of the floor, a turn on its spot,
        which way it faces, then parking. Returns (ok, words)."""
        last = self.tracker.where() if (self.tracker is not None and self.tracker.active == robot) else None
        q = (last[1], last[2]) if last else tuple((self.st["positions"].get(robot) or [None, None])[:2])
        if q[0] is not None and (not self.board.on_board(q[0], q[1]) or
                                 self.board.edge_room(q[0], q[1])[0] < EDGE_TURN_CM + 3):
            # review 2026-10-22: a turn on the spot this near an open edge could put a wheel over it
            return False, "atedge"
        time.sleep(1.0)
        old_pos = self.st["positions"].pop(robot, None)
        self.disagree[robot] = 0
        self.fresh_floor()
        p = self.identify(robot)
        self.moves.pop(robot, None)
        if p is None:
            if old_pos:
                self.st["positions"][robot] = old_pos
            return False, "noturn"
        self.st["positions"][robot] = [p[0], p[1], None]
        self.say("  Found %s at (%.1f, %.1f) cm." % (robot, p[0], p[1]))
        if self.tracker is not None:
            self.tracker.hand_moved.discard(robot)
        self.lost_place.discard(robot)
        if not park:
            return True, ""
        if not self.board.on_board(p[0], p[1]) or self.board.edge_room(p[0], p[1])[0] < EDGE_TURN_CM + 3:
            # 2026-10-22 10:23: Center found 3.5 cm off the board, then its heading drive and "moving it in first"
            # took it over the edge (it fell): its edge guard had learned the plywood level with a sensor already
            # over the edge. Its heading is unknown here, so no drive at all: Nick.
            return False, "atedge"
        if self.acquire_heading(robot, p) is None:
            return False, "noheading"
        self.set_corrections(robot, self.move_corrections(robot))
        ok, why = self.park(robot)
        return ok, why

    def sweep_for(self, lost):
        """Nick, 2026-10-22: a robot that knows where it is looks for the lost one with its front sensor. It turns
        on its spot in SWEEP_STEP_DEG steps (each turn leg reports the front sensor: firmware 2026-10-22b); a reading
        well short of what the map says is ahead (a board, or a robot we know of) is something not on the map: the
        lost robot. The camera then looks for its turn there only. Returns (x, y) or None."""
        for finder in self.st["robots"]:
            if finder == lost or finder in self.unavailable:
                continue
            q = self.st["positions"].get(finder)
            h = self.heads.get(finder, [None])[0]
            if h is None and q and len(q) > 2:
                h = q[2]
            if not q or h is None or self.board.edge_room(q[0], q[1])[0] < EDGE_TURN_CM + 3:
                continue
            self.say("  %s looks round for %s with its front sensor ..." % (finder, lost))
            self.mark("e02 sweep %s for %s" % (finder, lost))
            if not self.begin_moves(finder, "looking for " + lost):
                continue
            hits = []
            heading = h
            for k in range(int(round(360.0 / SWEEP_STEP_DEG))):
                state, info = self.send_leg(finder, "turn %.1f" % SWEEP_STEP_DEG, SWEEP_STEP_DEG / TURN_DPS_GUESS * 2.5 + 6.0)
                heading = ST.wrap180(heading + SWEEP_STEP_DEG)
                m = re.search(r"front=([0-9.]+)", info or "")
                if state not in ("done", "dup") or not m:
                    continue
                front = float(m.group(1))
                if front <= 0:
                    continue                      # no reading (nothing in range)
                saved = self.st["positions"].pop(lost, None)
                ahead = self.ahead_cm(finder, q[:2], heading)
                if saved is not None:
                    self.st["positions"][lost] = saved
                if front < SWEEP_RANGE_CM and (ahead is None or front < ahead[0] - SWEEP_GAP_CM):
                    d = front + 2 * PL.ROBOT_HALF_CM
                    hits.append((front, q[0] + d * math.cos(math.radians(heading)),
                                 q[1] + d * math.sin(math.radians(heading))))
            self.heads[finder] = [heading, self.heads.get(finder, [None, False])[1]]
            self.moves.pop(finder, None)
            for _, x, y in sorted(hits):
                if not self.board.on_board(x, y):
                    continue
                self.say("  %s sees something not on the map at about (%.0f, %.0f) cm: is it %s?" % (finder, x, y, lost))
                p = self.identify(lost, near=(x, y, SWEEP_NEAR_CM))
                if p is not None:
                    return p
        return None

    def rescue_find(self, robot, need_camera=True):
        """Nick, 2026-10-22: a robot the camera can't find finds itself, against the two boards. Its own pose (the
        last camera fix plus the legs since, wheels and gyro) is good enough to drive to the find's staging point;
        the find's own checks keep it safe (the front stop; a bump much sooner than the pose says fails it; the
        edge guard). No camera fix is sent (the camera can't see it). After the find its position is exact: the
        camera then looks for its turn there. Only when its last known place was well in from the open edges and
        nothing says it was moved by hand. Returns (x, y) or None."""
        q = self.st["positions"].get(robot)
        if not q or self.board.edge_room(q[0], q[1])[0] < RESCUE_EDGE_CM:
            return None
        if self.tracker is not None and robot in self.tracker.hand_moved:
            return None                    # lifted or pushed while held: its own pose is wrong
        self.say("  %s finds itself against the boards (the camera can't see it) ..." % robot)
        for c in self.board_params():
            self.send(robot, c)
            time.sleep(0.05)
        if self.tracker is not None and self.tracker.active == robot:
            self.tracker.idle()            # do_find must not send a fix from a lost track
        for attempt in range(2):
            out = self.do_find(robot, "rescue")
            if out.get("ok"):
                self.lost_place.discard(robot)
                break
            self.lost_place.add(robot)       # it drove somewhere the camera did not see: its place is unknown now
            self.say("  %s: %s" % (robot, out.get("words")))
            if "stop" in (out.get("words") or ""):
                return None                # stopped (watchdog or a stop): don't push on blind
            # (a find that failed squaring up, or found no board in reach, is worth one more try: dry run 13:40)
        else:
            return None
        fr = self.listener.latest("FIND", robot, out["t0"])
        ax, ay = (num(fr[1].get("after_x")), num(fr[1].get("after_y"))) if fr else (None, None)
        if ax is None or ay is None:
            return None
        self.say("  %s found itself at (%.1f, %.1f) cm; the camera looks there ..." % (robot, ax, ay))
        self.st["positions"][robot] = [ax, ay, None]      # where it is, even if the camera can't see it there
        self.fresh_floor()
        p = self.identify(robot, near=(ax, ay, SWEEP_NEAR_CM))
        if p is None and not need_camera:
            return (ax, ay)                # its find's end pose is exact: good enough to drive it out from
        return p

    def help_from_nick(self, robot, problem, park=True):
        """Stuck, lost or off the board: first AUTO_TRIES tries on its own (2026-10-22: pressing Enter with no
        hands, i.e. just finding it again, worked nearly every time); then exact steps for Nick, then the runner
        finds it again by a turn on its spot and parks it itself. Returns True when it is back (and parked, unless
        park=False: then only found)."""
        if park and "off the board" not in problem:
            # (park=False: the caller goes on from where the robot stands, e.g. jammed on a brick: Nick moves it)
            for k in range(AUTO_TRIES):
                self.say("  %s: %s. Trying again on its own (%d of %d) ..." % (robot, problem, k + 1, AUTO_TRIES))
                self.mark("e02 auto-retry %s %d: %s" % (robot, k + 1, problem))
                self.st.setdefault("auto_retries", []).append([round(time.time(), 1), robot, problem])
                self.send(robot, "stop")
                ok, why = self.retry_find(robot, park)
                if ok:
                    self.say("  %s is back without help." % robot)
                    return True
                if why == "atedge":
                    problem = "it is at (or over) an open edge, and which way it faces is not known"
                    break
                if why == "noturn" and k == AUTO_TRIES - 1 and self.tracker is not None:
                    self.heads[robot] = [None, False]        # a rescue find sets it (exact); a sweep does not
                    p = self.sweep_for(robot) or self.rescue_find(robot)
                    if p is not None:
                        self.st["positions"][robot] = [p[0], p[1], None]
                        self.say("  Found %s again at (%.1f, %.1f) cm." % (robot, p[0], p[1]))
                        if not park:
                            return True
                        h = self.heads.get(robot, [None, False])
                        if h[0] is not None and h[1]:
                            # just found itself against the boards: its heading is exact, no drive needed
                            self.tracker.robot_at(p[0], p[1])
                            self.tracker.follow(robot, p[0], p[1])
                            self.history.clear()
                            self.tracker.hold(HOLD_CM)
                            headed = True
                        else:
                            headed = not (self.board.edge_room(p[0], p[1])[0] < EDGE_TURN_CM + 3) and \
                                self.acquire_heading(robot, p) is not None
                        if headed:
                            self.set_corrections(robot, self.move_corrections(robot))
                            ok, why = self.park(robot)
                            if ok:
                                return True
                        # found but not parked (dry run 13:15: lost during its first turn in the corner): it is
                        # in view now, so one more ordinary try before Nick
                        self.say("  %s: one more try now that it is in view ..." % robot)
                        ok, why2 = self.retry_find(robot, park)
                        if ok:
                            self.say("  %s is back without help." % robot)
                            return True
                if why not in ("noturn", "noheading"):
                    problem = why
        where = ("anywhere on the open part of the board, at least 30 cm (1 ft) from the other robots, the boards "
                 "and the edges")
        facing = "open board"
        self.mark("e02 help %s: %s" % (robot, problem))
        self.st.setdefault("help_calls", []).append([round(time.time(), 1), robot, problem])
        self.st.save()
        print("\n" + "!" * 78)
        for line in ST.nick_instructions(robot, problem, where, facing):
            self.say(line)
        self.say("   (You don't need to put it on any particular spot: it finds its own way there.)")
        print("!" * 78)
        self.send(robot, "stop")
        self.send(robot, "identify")
        if self.a.auto:
            pass
        wait_s = self.a.help_wait_min * 60.0
        self.say("(If nobody answers within %.0f minutes, %s is put aside and the others carry on; --resume later "
                 "runs what it missed.)" % (self.a.help_wait_min, robot))
        while True:
            print("\a", end="", flush=True)
            ans = self.ask_timeout("Press Enter when it is back on the board (f = flash it again, skip = put it "
                                   "aside): ", wait_s)
            if ans is None or ans.lower() == "skip":
                self.mark("e02 help %s %s" % ("timeout" if ans is None else "skip", robot))
                self.unavailable.add(robot)
                if ans is None:
                    self.say("Nobody answered in %.0f minutes: %s's remaining runs are put aside." % (
                        self.a.help_wait_min, robot))
                    self.keep_on_map(robot)
                else:                          # Nick took it off the board
                    self.st["positions"].pop(robot, None)
                    if self.tracker is not None:
                        self.tracker.forget(robot)
                    self.st.save()
                return False
            if ans.lower() == "f":
                self.send(robot, "identify")
                continue
            self.say("Taking a fresh picture of the board (keep hands out of it), then %s turns a little on its "
                     "spot ..." % robot)
            ok, why = self.retry_find(robot, park)
            if ok:
                return True
            if why == "noturn":
                self.say("The camera did not see %s turn. Check it is on the board and switched on (its side light "
                         "on), then press Enter again." % robot)
            elif why == "atedge":
                self.say("The camera sees it at (or over) an open edge. Move it well in from the edge, then press Enter.")
            elif why == "noheading":
                self.say("It turned, but the camera lost it when it drove. Press Enter to try again.")
            else:
                self.say("It moved, but parking failed: %s. Put it somewhere open again and press Enter." % why)

    # --- moving a robot the camera can't see, on its own wheels and gyro (Nick, 2026-10-22) -------------------
    def own_pose(self, robot, since=0.0):
        """The robot's own position (x, y, heading) from its last finished leg: wheels and gyro since the last
        camera fix or find, in the board's frame. None if it has not reported one."""
        r = self.listener.latest("LEG", robot, since)
        f = self.listener.latest("FIND", robot, since)
        if f and (not r or f[0] >= r[0]):
            # a find after its last leg: a finished find's end pose is exact; a failed one's is not to be trusted
            # (it may have started from no pose at all; review 2026-10-22: a dry run kept Left at (99, 162))
            if (f[1].get("result") or "") != "ok":
                return None
            x, y, th = num(f[1].get("after_x")), num(f[1].get("after_y")), num(f[1].get("after_th"))
        elif r:
            x, y, th = num(r[1].get("x_cm")), num(r[1].get("y_cm")), num(r[1].get("th_deg"))
        else:
            return None
        if None in (x, y, th) or not self.board.on_board(x, y, margin=-5.0):
            return None                     # off the board: not a pose it can have (unknown pose, power cycle)
        return (x, y, th)

    def own_path(self, robot, until, n=BLIND_RETRACE_LEGS):
        """Where the robot's own pose was at the end of each of its last n finished drives before `until` (the start
        of the blind drive: not the blind pieces themselves) and after its last find (newest last)."""
        out = []
        f = self.listener.latest("FIND", robot)
        after = f[0] if f else 0.0
        with self.listener.cv:
            for t, nm, rec in self.listener.recs["LEG"]:
                st_ = rec.get("state") or rec.get("result")
                if nm == robot and st_ == "done" and rec.get("kind") == "go" and after <= t < until:
                    x, y = num(rec.get("x_cm")), num(rec.get("y_cm"))
                    if x is not None and y is not None:
                        out.append((x, y))
        return out[-n:]

    def drive_blind(self, robot, waypoints):
        """Drive through waypoints on the robot's own pose: turn to face each, then short drives
        (BLIND_PIECE_CM), the front stop and the edge guard on. After every drive the front sensor is checked
        against what the map says should be ahead (boards, robots we know of): landmarks confirm the pose. Returns
        (ok, words)."""
        if not self.begin_moves(robot, "driving blind"):
            return False, "could not start its moves"
        for wp in waypoints:
            if not self.board.on_board(wp[0], wp[1]) or self.board.edge_room(wp[0], wp[1])[0] < BLIND_EDGE_CM:
                return False, "the way out runs near an open edge"
            reached = False
            for _ in range(20):
                pose = self.own_pose(robot)
                if pose is None:
                    return False, "it has not said where it thinks it is"
                d = math.hypot(wp[0] - pose[0], wp[1] - pose[1])
                if d < 3.0:
                    reached = True
                    break
                turn = ST.wrap180(math.degrees(math.atan2(wp[1] - pose[1], wp[0] - pose[0])) - pose[2])
                if abs(turn) > ST.MIN_TURN_DEG:
                    state, info = self.send_leg(robot, "turn %.1f" % turn, abs(turn) / TURN_DPS_GUESS * 2.5 + 6.0)
                    if state not in ("done", "dup"):
                        return False, "a turn did not finish (%s %s)" % (state, info)
                go = min(d, BLIND_PIECE_CM)
                state, info = self.send_leg(robot, "go %.1f" % go, go / DRIVE_CMS_GUESS * 2.5 + 6.0)
                if state not in ("done", "dup"):
                    return False, "a drive did not finish (%s %s)" % (state, info)
                # landmarks: what is ahead by its own pose against what its front sensor reads
                pose = self.own_pose(robot)
                m = re.search(r"front=([0-9.]+)", info or "")
                if pose and m and float(m.group(1)) > 0:
                    ahead = self.ahead_cm(robot, pose[:2], pose[2])
                    off, tol = self.front_limits()
                    if ahead and ahead[0] <= FRONT_CHECK_RANGE_CM and self.square_on(ahead) and \
                            abs(float(m.group(1)) - off - ahead[0]) > FRONT_STEER_FACTOR * tol:
                        return False, "its front sensor reads %.0f cm where its own pose puts %s at %.0f cm" % (
                            float(m.group(1)), ahead[1], ahead[0])
            if not reached:
                return False, "it did not reach (%.0f, %.0f) cm" % tuple(wp)
        return True, ""

    def drive_out(self, robot):
        """A robot the camera can't see, or can't park, stuck where it blocks the others (the find's corner): drive
        it to its park spot on its own pose, round the robots we know of; else retrace its last drives backwards
        (that way was clear a moment ago); then the camera looks for it where it should be. Returns (x, y) or
        None."""
        pose = self.own_pose(robot)
        if pose is None:
            return None
        if self.tracker is not None and robot in self.tracker.hand_moved:
            return None                    # moved by hand: its own pose is wrong
        self.say("  %s drives out on its own wheels and gyro (the camera can't follow it there) ..." % robot)
        self.mark("e02 drive out %s" % robot)
        t_start = time.time()
        try:
            px, py, _ = self.board.park(self.park_index(robot))     # its own park spot: out of every route's way
            # first straight away from both boards (dry run 14:51: driving along the board from the corner, its
            # front stop saw the board and stopped it), then round the robots to its spot
            out = self.board.uv(BLIND_CLEAR_CM, BLIND_CLEAR_CM)
            first = [out] if (self.board.edge_room(pose[0], pose[1])[1] < BLIND_CLEAR_CM and
                              math.hypot(out[0] - pose[0], out[1] - pose[1]) > 3.0) else []
            rest = self.safe_path(robot, first[0] if first else pose[:2], (px, py))
            wps = first + rest if rest is not None else None
            ok, words = (self.drive_blind(robot, wps) if wps is not None else
                         (False, "no clear way round the other robots"))     # never straight through them
            if not ok:
                self.say("  %s: %s; retracing its last drives instead ..." % (robot, words))
                back = list(reversed(self.own_path(robot, until=t_start)))
                pose2 = self.own_pose(robot)
                back = [q for q in back if pose2 is None or math.hypot(q[0] - pose2[0], q[1] - pose2[1]) > 3.0]
                if not back:
                    return None
                ok, words = self.drive_blind(robot, back)
                if not ok:
                    self.say("  %s: %s" % (robot, words))
                    return None
            pose = self.own_pose(robot)
            self.fresh_floor()
            p = self.identify(robot, near=(pose[0], pose[1], SWEEP_NEAR_CM)) if pose else None
        finally:
            self.moves.pop(robot, None)
        if p is not None:
            self.say("  %s is in view again at (%.1f, %.1f) cm." % (robot, p[0], p[1]))
            return p
        if pose is not None:
            # it drove there without trouble, but the camera can't see it there either: go by its own pose (the
            # others are planned round it; better than leaving it where every find drives)
            self.say("  The camera can't see %s at (%.0f, %.0f) cm, where it drove; going by its own position." % (
                robot, pose[0], pose[1]))
            return (pose[0], pose[1])
        return None

    def keep_on_map(self, robot):
        """A robot put aside stays on the board as an obstacle the others go round: try once more to find where it
        is (a turn on its spot, no Nick); if that fails, keep the last place it was seen. During setup nobody knows
        where it is at all: the session stops rather than drive the others blind (review 2026-10-22)."""
        last = self.tracker.where() if (self.tracker and self.tracker.active == robot) else None
        q = self.st["positions"].get(robot)
        guess = (last[1], last[2]) if last else (tuple(q[:2]) if q else None)
        p = self.identify(robot, near=(guess[0], guess[1], 60.0) if guess else None)
        self.moves.pop(robot, None)
        if p is None:
            # the camera can't see it: its own pose from its last leg, if that is newer than the camera's last
            # sighting (dry run 14:31: it had found itself in the find corner, the runner still had its park spot)
            own = None
            if self.tracker is None or robot not in self.tracker.hand_moved:
                own = self.own_pose(robot, since=(last[0] if last else 0.0))
            p = (own[0], own[1]) if own else (None if robot in self.lost_place else guess)
        if p is None:
            # (also: it moved unseen after its last known place, e.g. a failed find of its own: dry run 15:15, the
            # runner kept it at its park spot while it stood in Center's way)
            raise SystemExit("%s is somewhere on the board but the camera can't find it, so the other robots could "
                             "drive into it. Take %s off the board (or put it anywhere open), then start again." % (
                                 robot, robot))
        self.st["positions"][robot] = [p[0], p[1], None]
        if self.tracker is not None:
            if self.tracker.active == robot:
                self.tracker.idle()
            self.tracker.park(robot, p[0], p[1])
        self.say("  %s stays where it is, at about (%.0f, %.0f) cm; the others go round it." % (robot, p[0], p[1]))
        self.st.save()
        fx, fy = self.board.find_stage()
        kx, ky, _ = self.board.park(self.park_index(robot))
        in_corner = math.hypot(p[0] - fx, p[1] - fy) < BLOCKS_FIND_CM
        if not in_corner and math.hypot(p[0] - kx, p[1] - ky) > ASIDE_PARK_CM:
            # put aside out on the board (dry run 15:01: at (73, 62), where the routes run; the next run's watchdog
            # stopped it): drive it to its own spot on its own pose; if that fails it stays where it is
            self.say("  %s is put aside out on the board, where the routes run: moving it to its spot ..." % robot)
            q = self.drive_out(robot)
            if q is not None:
                self.st["positions"][robot] = [q[0], q[1], None]
                if self.tracker is not None:
                    self.tracker.robot_at(q[0], q[1])
                    self.tracker.park(robot, q[0], q[1])
                self.say("  %s stays at about (%.0f, %.0f) cm; the others go round it." % (robot, q[0], q[1]))
                self.st.save()
            return
        if in_corner:
            # dry run 13:15: a robot put aside by the blocked corner, where every find drives: every run after it
            # failed its find. Nick (2026-10-22): drive it out blind first (its own pose, landmarks, retracing),
            # then the finder sweep and its own find; stop only if all of that fails.
            self.say("  %s is in the corner every find drives to: moving it out ..." % robot)
            q2 = None
            for step in (None, self.sweep_for, lambda r: self.rescue_find(r, need_camera=False)):
                # each step that only finds it (the sweep) or puts it in the corner (its own find, which leaves an
                # exact pose) is followed by a drive out (review 2026-10-22)
                if step is not None:
                    found = step(robot)
                    if found is None:
                        continue
                    if math.hypot(found[0] - fx, found[1] - fy) >= BLOCKS_FIND_CM:
                        q2 = found
                        break
                    if step == self.sweep_for:      # (bound methods: equal, never identical)
                        # its position is known now (heading not): tell it, then drive out on its gyro's heading
                        self.send(robot, "fix %.2f %.2f - %.3f" % (found[0], found[1], time.time()))
                        time.sleep(0.2)
                q = self.drive_out(robot)
                if q is not None and math.hypot(q[0] - fx, q[1] - fy) >= BLOCKS_FIND_CM:
                    q2 = q
                    break
            if q2 is not None:
                self.st["positions"][robot] = [q2[0], q2[1], None]
                if self.tracker is not None:
                    self.tracker.robot_at(q2[0], q2[1])
                    self.tracker.park(robot, q2[0], q2[1])
                self.say("  %s stays at about (%.0f, %.0f) cm, out of the way; the others go round it." % (
                    robot, q2[0], q2[1]))
                self.st.save()
                return
            raise SystemExit("%s is put aside at about (%.0f, %.0f) cm, in the corner every robot's find drives to, "
                             "so no run can start. Take %s off the board, then start again with --resume." % (
                                 robot, p[0], p[1], robot))

    # --- checks -----------------------------------------------------------------------------------
    def wait_robots(self, names, timeout=40):
        t0 = time.time()
        while time.time() - t0 < timeout and not set(names) <= set(self.link.robots()):
            time.sleep(0.5)
        return [n for n in names if n not in self.link.robots()]

    def batteries(self, names):
        t = time.time()
        for n in names:
            self.send(n, "status")
        time.sleep(1.5)
        out = {}
        for n in names:
            b = self.listener.battery.get(n)
            out[n] = b[1] if b and b[0] >= t - 0.5 else None
        return out

    def setup(self):
        a, st = self.a, self.st
        setup = st["setup"]
        self.say("=== Setup checks ===")
        # 1. Pi recorder
        ok1, w1, size, fps = CAM.check_stream(a.pi, a.overhead_port)
        if a.no_side:
            ok2, w2 = True, "not used this session (--no-side)"
        else:
            ok2, w2, _, _ = CAM.check_stream(a.pi, a.side_port)
        notes = CAM.notes_answer(a.pi)
        self.say("Overhead camera (%s:%d): %s" % (a.pi, a.overhead_port, "OK, " + w1 if ok1 else "PROBLEM: " + w1))
        self.say("Side camera (%s:%d): %s" % (a.pi, a.side_port, ("OK, " if not a.no_side else "") + w2 if ok2
                                                 else "PROBLEM: " + w2))
        self.say("Pi recorder hears notes: %s" % ("OK" if notes else "PROBLEM: no answer on port 5007"))
        setup["pi"] = dict(overhead=ok1, side=ok2, notes=notes, fps=round(fps, 1))
        if not ok1:
            raise SystemExit("The overhead camera is not sending pictures. On the Pi (SSH window), start the "
                             "recorder with both cameras:\n  python3 tools/pi/camrec.py --device 0,2 --names "
                             "overhead,side --fps 25 --label E02\nthen start this again.")
        if not ok2 or not notes:
            if self.ask("The side camera or the notes are not answering. Carry on anyway? (y/n) ", "y").lower() != "y":
                raise SystemExit("Stopped. Start the Pi recorder with both cameras, then start this again.")
        # 2. calibration
        frames = self.feed.wait_frames(10, timeout=10)
        if not frames:
            raise SystemExit("No pictures from the overhead camera.")
        img = frames[-1]
        cal = st.get("calibration")
        if not cal or not os.path.exists(cal):
            cal = None
            if not a.make_calibration:
                cal = CAM.find_calibration(a.calib, a.project, self.dir)
                if cal is None and os.path.exists(DEFAULT_CALIB):
                    self.say("No floor calibration in the studio project: using the one made from roam1's board corners "
                             "(%s; the overhead camera has not moved since)." % DEFAULT_CALIB)
                    cal = DEFAULT_CALIB
            if cal and not self.confirm_calibration(cal, img):
                self.say("The board outline does not line up with this calibration (%s)." % cal)
                cal = None
                a.make_calibration = True
        if cal:
            self.say("Floor calibration: %s" % cal)
        else:
            self.say("No floor calibration found (looked in the studio project %s\\cameras and this session)."
                     % (a.project or CAM.default_project()))
            if self.ask("Make one now from the board's 4 corners in the camera picture? (y/n) ", "y").lower() != "y":
                raise SystemExit("Make one in the studio (More -> Floor calibration -> rectangle: the board's 4 "
                                 "corners, 122.15 x 121.9 cm), then start this again.")
            corners, why = CAM.find_board_corners(img)
            if corners is None:
                raise SystemExit("Could not find the board's corners by myself (%s). Make the calibration in the "
                                 "studio instead, then start this again." % why)
            cal = os.path.join(self.dir, "cameras", "overhead.json")
            res = CAM.make_calibration(corners, (img.shape[1], img.shape[0]), cal, height_cm=a.lens_height_cm)
            cam = CAM.load_camera(cal)
            pic = CAM.draw_check(img, cam, os.path.join(self.dir, "check_calibration.png"),
                                 PL.Board(), corners)
            self.say("Made %s: %s" % (cal, res["text"]))
            self.say("Open %s: the green outline must sit on the board's edges, 0,0 in the bottom-right corner." % pic)
            if self.ask("Does the green outline sit on the board's edges? (y/n) ", "y").lower() != "y":
                os.remove(cal)
                raise SystemExit("Not used. Make the calibration in the studio instead, then start this again.")
            setup["calibration_made_here"] = dict(corners=corners, err_cm=res["err_cm"], text=res["text"])
        st["calibration"] = cal
        self.cam = CAM.load_camera(cal)
        self.make_tracker()
        st.save()
        # 3. the blocked boards
        if not st.get("board"):
            self.find_boards(img)
        self.board = PL.Board.from_dict(st["board"])
        self.tracker_area()
        sx, sy, sh = self.board.nominal_start()
        self.say("Route start spot: (%.1f, %.1f) cm, near the blocked corner" % (sx, sy))
        for r in PL.ROUTES:
            ok, open_room, board_room = self.board.route_fits(r)
            self.say("Route %s, heading %.0f deg: %s (front at least %.0f cm from the open edges, body %.1f cm from "
                     "the boards)" % (PL.ROUTE_NAMES[r], self.board.route_heading(r), "fits" if ok else "TOO TIGHT",
                                      open_room, board_room))
        if self.board.turn_sign < 0:
            self.say("WARNING: with the boards on these two edges the square route (the firmware turns it left) would "
                     "head into the x-board. The square runs are left out of this session's plan.")
            st["setup"]["square_left_out"] = True
        st.save()

    def confirm_calibration(self, cal, img):
        """The plain check: the board's outline (from the calibration) drawn over the live picture, on screen;
        Nick says whether it lines up (y/n). Unattended (--auto) or no screen: the picture is saved and taken as
        right. Returns True to use it."""
        cam = CAM.load_camera(cal)
        path = os.path.join(self.dir, "check_calibration.png")
        CAM.draw_check(img, cam, path, PL.Board())
        self.st["setup"]["calibration_check"] = path
        if self.a.auto:
            self.say("Calibration check picture: %s (automatic mode: taken as lining up)" % path)
            return True
        shown = CAM.show_picture(path, "E02: does the green outline sit on the board's edges?")
        self.say("%s the camera picture with the board's outline in green from the floor calibration%s." % (
            "On screen now:" if shown else "Open", "" if shown else " (" + path + ")"))
        ans = self.ask("Does the green outline sit on the plywood board's 4 edges (within about a finger's width)? "
                       "Type y or n and press Enter: ", "y").lower()
        CAM.close_picture()
        ok = ans.startswith("y")
        self.st["setup"]["calibration_confirmed"] = dict(file=cal, answer=ans)
        return ok

    def tracker_area(self):
        """The tracker's board area and walls (the board faces, robot-centre side: a centre is half a body in from
        the face), from the board found in the picture."""
        if self.tracker is None or self.board is None:
            return
        b = self.board
        self.tracker.set_area((b.x0, b.y0, b.x1, b.y1),
                              walls=[("y", b.xboard_cm + b.uy * PL.ROBOT_HALF_CM),
                                     ("x", b.yboard_cm + b.ux * PL.ROBOT_HALF_CM)])

    def make_tracker(self):
        if self.tracker is None:
            b = PL.Board()
            self.tracker = CAM.MotionTracker(self.cam, ((b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2))
            self.feed.tracker = self.tracker

    def find_boards(self, img):
        """Which two edges are blocked (asked, in picture words; or --board-sides), then their floor lines
        measured in the picture."""
        b0 = PL.Board()
        mid_x, mid_y = (b0.x0 + b0.x1) / 2, (b0.y0 + b0.y1) / 2
        xs_words = {side: CAM.picture_side(self.cam, mid_x, b0.y0 if side == 0 else b0.y1) for side in (0, 1)}
        ys_words = {side: CAM.picture_side(self.cam, b0.x0 if side == 0 else b0.x1, mid_y) for side in (0, 1)}
        if self.a.board_sides:
            xs, ys = (int(v) for v in self.a.board_sides.split(","))
        else:
            CAM.draw_check(img, self.cam, os.path.join(self.dir, "check_board_edges.png"), b0)
            self.say("Look at the camera picture (also saved as %s)." % os.path.join(self.dir, "check_board_edges.png"))

            def pick(question, words):
                while True:
                    ans = self.ask("%s Type %s or %s and press Enter: " % (question, words[0], words[1]), words[0]).lower()
                    for side, w in words.items():
                        if ans == w:
                            return side
                    self.say("  Please type %s or %s." % (words[0], words[1]))
            xs = pick("One blocked board runs across the picture along its %s or %s edge (the x-board). Which?"
                      % (xs_words[0], xs_words[1]), xs_words)
            ys = pick("The other blocked board runs along the %s or %s edge (the y-board). Which?"
                      % (ys_words[0], ys_words[1]), ys_words)
        bd = PL.Board(xboard_side=xs, yboard_side=ys)
        self.say("Blocked boards: the x-board along the %s of the picture, the y-board along the %s."
                 % (xs_words[xs], ys_words[ys]))
        found = {}
        for which in ("x", "y"):
            m = CAM.measure_board_line(img, self.cam, bd, which)
            if m is None:
                edge = (bd.y0 if xs == 0 else bd.y1) if which == "x" else (bd.x0 if ys == 0 else bd.x1)
                self.say("  Could not see the %s-board's line clearly in the picture: using the board's edge (%.1f cm) "
                         "for now; the find check measures it again from where the robots touch it." % (which, edge))
                m = dict(value=edge, spread=None, n=0, values=[], from_edge=True)
            else:
                self.say("  %s-board's floor line at %s = %.2f cm (%d of 17 places, they agree within %.2f cm)"
                         % (which, "y" if which == "x" else "x", m["value"], m["n"], m["spread"]))
                if m["spread"] > 1.0:
                    self.say("  WARNING: the %s-board's line is uneven (%.1f cm): is it straight and flat against the "
                             "floor?" % (which, m["spread"]))
            found[which] = m
        bd.xboard_cm, bd.yboard_cm = found["x"]["value"], found["y"]["value"]
        self.st["board"] = bd.to_dict()
        self.st["setup"]["board_lines_image"] = found
        CAM.draw_check(img, self.cam, os.path.join(self.dir, "check_boards.png"), bd)

    def boards_still(self):
        """Re-measure the board lines (block start): they must not have moved."""
        if self.feed.last_img is None:
            return
        img = self.feed.last_img
        for which in ("x", "y"):
            m = CAM.measure_board_line(img, self.cam, self.board, which)
            ref0 = self.st["setup"]["board_lines_image"][which]
            if ref0.get("from_edge"):
                continue
            ref = ref0["value"]
            if m is None:
                self.say("  (could not re-measure the %s-board this time: a robot or a shadow in the way?)" % which)
                continue
            moved = m["value"] - ref
            self.st.setdefault("board_checks", []).append([round(time.time(), 1), which, m["value"], round(moved, 2)])
            if abs(moved) > 1.0:
                self.say("WARNING: the %s-board looks %.1f cm away from where it was at the start. Were the boards "
                         "bumped? (the runs carry on; this is in the data)" % (which, moved))
                self.mark("e02 board %s moved %.1f cm" % (which, moved))

    def robots_ready(self):
        names = self.st["robots"]
        missing = self.wait_robots(names)
        if missing:
            raise SystemExit("Not heard from: %s. Switch them on (same WiFi as this laptop), then start again."
                             % ", ".join(missing))
        bats = self.batteries(names)
        for n in names:
            b = bats.get(n)
            fw = self.listener.firmware.get(n, "?")
            self.say("%s: connected (firmware %s), battery %s" % (n, fw, "%d%%" % b if b is not None else "not reported"))
            if b is not None and b < BATTERY_WARN:
                self.say("  WARNING: %s's battery is low for a 2-hour session: charge it first if you can." % n)
            if b is not None and b < BATTERY_STOP:
                self.unavailable.add(n)
                self.say("  %s is too flat to run: its runs are put aside." % n)
        self.st["setup"]["batteries"] = bats
        old = [n for n in names if self.listener.firmware.get(n) not in (EXPECTED_FW, None)]
        if old:
            raise SystemExit("%s run%s firmware %s; this session needs %s (it refuses to drive without the edge guard "
                             "and limits blind backing). Upload firmware/home_lab to each robot (Arduino IDE: open "
                             "home_lab.ino, pick the robot's port, press Upload), then start this again." % (
                                 ", ".join(old), "s" if len(old) == 1 else "", "/".join(sorted(
                                     {self.listener.firmware.get(n) for n in old})), EXPECTED_FW))
        self.safety_settings([n for n in names if n not in self.unavailable])
        self.say("Compass check (`calibrate`, needed once after switching on; spin and roll off: it only wiggles) ...")
        self.calibrate([n for n in names if n not in self.unavailable])
        # every robot found first by a short turn on its spot (nothing leaves its spot, so nothing is mistaken for
        # a robot), THEN parked, each going round the others (2026-10-22, after the first real session)
        self.find_all([n for n in names if n not in self.unavailable])
        self.park_all([n for n in names if n not in self.unavailable])
        self.active = None
        CAM.draw_check(self.feed.last_img, self.cam, os.path.join(self.dir, "check_robots.png"), self.board,
                       robots={n: tuple(self.st["positions"][n][:2]) for n in names if n in self.st["positions"]})

    def find_all(self, names):
        """Identify every robot (a turn on its spot), then a fresh floor memory with all of them known."""
        self.phase = "finding"
        self.say("Finding the robots: each turns a little on its spot and back (about 6 s each) ...")
        for n in names:
            self.st["positions"].pop(n, None)
        for n in names:
            p = self.identify(n)
            self.moves.pop(n, None)
            if p is None:
                self.say("  The camera did not see %s turn." % n)
                self.help_from_nick(n, "the camera did not see it turn when asked to", park=False)
                continue
            self.st["positions"][n] = [p[0], p[1], None]
            self.say("  %s is at (%.1f, %.1f) cm" % (n, p[0], p[1]))
        self.st.save()
        self.fresh_floor()

    def fresh_floor(self):
        """A new floor memory from a few still pictures; every robot the runner knows of is marked on it."""
        frames = self.feed.wait_frames(10, timeout=8)
        known = {n: tuple(q[:2]) for n, q in self.st["positions"].items() if q}
        self.tracker.idle()
        self.tracker.clear_protect()
        self.tracker.set_background(frames, robots=list(known.values()))
        for n, q in known.items():
            self.tracker.park(n, q[0], q[1])

    def park_all(self, names):
        """Heading and parking, robot by robot, in an order where nobody's park spot is taken by another robot.
        A robot standing on someone else's spot when all spots are taken goes to the middle of the board first."""
        todo = [n for n in names if n in self.st["positions"] and n not in self.unavailable]
        deferred = defaultdict(int)
        passes = 0
        while todo:
            passes += 1
            if passes > 3 * len(names) + 2:
                raise SystemExit("Parking went round in circles (%s still not parked). Put the robots anywhere open on "
                                 "the board, 30 cm apart, and start again." % ", ".join(todo))
            def blocked(n):
                i = self.park_index(n)
                px_, py_ = self.board.park(i)[:2]
                sx_, sy_ = self.board.stage(i)
                # another robot on its spot, or right on the point it backs in from (2026-10-22: a looser test for
                # the backing-in point; 24 cm round both dead-locked robots put down 30 cm from the walls)
                return [m for m in todo if m != n and (
                    math.hypot(self.st["positions"][m][0] - px_, self.st["positions"][m][1] - py_) < PL.PARK_CLEAR_CM + 6
                    or math.hypot(self.st["positions"][m][0] - sx_, self.st["positions"][m][1] - sy_) < PL.PARK_CLEAR_CM)]
            def path_ok(n):
                q0 = self.st["positions"][n]
                return self.safe_path(n, (q0[0], q0[1]), self.board.stage(self.park_index(n))) is not None

            def obstructs(m):
                # how many other robots' ways (to their backing-in point, and their spots) m stands in
                qm = self.st["positions"][m]
                k = 0
                for n2 in todo:
                    if n2 == m:
                        continue
                    a = self.st["positions"][n2]
                    for b in (self.board.stage(self.park_index(n2)), self.board.park(self.park_index(n2))[:2]):
                        dx, dy = b[0] - a[0], b[1] - a[1]
                        L = math.hypot(dx, dy) or 1e-6
                        t = max(0.0, min(L, ((qm[0] - a[0]) * dx + (qm[1] - a[1]) * dy) / L))
                        if math.hypot(a[0] + dx / L * t - qm[0], a[1] + dy / L * t - qm[1]) < PL.PARK_CLEAR_CM + 4:
                            k += 1
                return k
            # park first a robot whose spot is free AND whose way there is clear (2026-10-22: checking only the
            # spot dead-locked three robots put down in the corners); none: the one most in the others' way moves
            # to a free holding spot first
            free = [n for n in todo if not blocked(n) and path_ok(n)]
            if not free:
                todo.sort(key=lambda m: -obstructs(m))
            n = free[0] if free else todo[0]
            self.active = n
            self.phase = "finding"
            q = self.st["positions"][n]
            p = self.acquire_heading(n, (q[0], q[1]))
            if p is None:
                if not self.help_from_nick(n, "the camera lost it while measuring which way it faces"):
                    todo.remove(n)
                    continue
            elif not free:
                mid = self.holding_spot(n)
                self.say("  Every park spot has another robot near it: moving %s out of the way first, to (%.0f, %.0f) cm."
                         % (n, mid[0], mid[1]))
                ok, why = self.drive_to(n, mid, max_leg=PARK_LEG_CM)
                self.end_moves(n)
                if not ok:
                    if not self.help_from_nick(n, why):
                        todo.remove(n)
                    continue
                p = self.settle(min_s=0.3)
                if p:
                    self.st["positions"][n] = [p[0], p[1], self.heads.get(n, [None])[0]]
                    self.tracker.park(n, p[0], p[1])
                continue
            self.set_corrections(n, self.move_corrections(n))
            ok, why = self.park(n)
            if not ok and "no clear way" in why and deferred[n] < 2 and len(todo) > 1:
                # another robot not parked yet is in the way: park the others first, then try again
                deferred[n] += 1
                self.end_moves(n)
                p = self.settle(min_s=0.3)
                if p:
                    self.st["positions"][n] = [p[0], p[1], self.heads.get(n, [None])[0]]
                    self.tracker.park(n, p[0], p[1])
                self.say("  %s waits: another robot is in its way; parking the others first." % n)
                todo.remove(n)
                todo.append(n)
                continue
            if not ok and not self.help_from_nick(n, why):
                todo.remove(n)
                continue
            p = self.settle()
            if p:
                self.say("  %s parked at (%.1f, %.1f) cm" % (n, p[0], p[1]))
            todo.remove(n)
        self.st.save()

    def holding_spot(self, robot):
        """Where a robot can wait out of everyone's way while the others park (2026-10-22: robots put down in three
        corners dead-locked; the middle was used without checking it was free): the point on the board furthest
        from every other robot, every park spot and every backing-in point, and well in from the edges."""
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q]
        spots = []
        for i in range(len(self.st["robots"])):
            spots.append(tuple(self.board.park(i)[:2]))
            spots.append(tuple(self.board.stage(i)))
        best = None
        for x in np.arange(self.board.x0 + 25, self.board.x1 - 24, 4.0):
            for y in np.arange(self.board.y0 + 25, self.board.y1 - 24, 4.0):
                d_r = min([math.hypot(x - o[0], y - o[1]) for o in others] or [99.0])
                d_s = min([math.hypot(x - q[0], y - q[1]) for q in spots] or [99.0])
                score = min(d_r, 1.5 * d_s)
                if best is None or score > best[0]:
                    best = (score, (float(x), float(y)))
        return best[1]

    def acquire_heading(self, robot, p):
        """A robot found by identify(): follow it from p and measure its heading by a short drive (forward;
        blocked: a quarter turn and again). Returns its position or None."""
        self.fresh_id[robot] = True
        self.tracker.robot_at(p[0], p[1])
        self.tracker.follow(robot, p[0], p[1])
        self.history.clear()
        self.tracker.hold(HOLD_CM)
        self.heads[robot] = [None, False]
        self.moves.pop(robot, None)
        if not self.begin_moves(robot, "finding its heading"):
            return None
        for attempt in range(3):
            p0, p1, words = self.leg(robot, ("go", 10.0))
            if p0 and p1 and math.hypot(p1[0] - p0[0], p1[1] - p0[1]) >= ST.MIN_HEADING_CM:
                self.heads[robot] = [ST.bearing(p0, p1), True]
                self.say("  %s faces %.0f deg" % (robot, self.heads[robot][0]))
                return p1
            if "disagree" in words and self.disagree[robot] >= 2:
                return None
            self.leg(robot, ("turn", 90.0))
        return None

    def map_board(self, robot, n_rays=16):
        """Robot perimeter mapping (e02_map.py): from the middle of the board, rays out in n_rays directions; each
        ends where the robot's own sensors stop it (front stop at a board, edge guard at an open edge); then
        straight back. The camera only watches (its watchdog lets it reach the edge itself: that is what is
        measured; it still stops a robot whose centre would cross the edge). Returns the sides found, or None."""
        st = self.st
        if not self.activate(robot):
            self.say("Mapping: the camera can't see %s at its spot; no mapping this time." % robot)
            return None
        self.say("=== Mapping: %s feels out the board (%d rays from the middle; about %d minutes) ===" % (
            robot, n_rays, max(1, round(n_rays * 22 / 60))))
        self.mark("e02 map start %s" % robot)
        self.phase = "mapping"
        self.order = "map"
        self.set_corrections(robot, self.move_corrections(robot))
        mid = ((self.board.x0 + self.board.x1) / 2, (self.board.y0 + self.board.y1) / 2)
        ok, why = self.drive_to(robot, mid, max_leg=PARK_LEG_CM, arrive_cm=4.0)
        if not ok:
            self.say("Mapping: could not get %s to the middle (%s); no mapping this time." % (robot, why))
            self.park(robot)
            return None
        rays = []
        self.watch_edge_cm = 0.0
        try:
            others = [(n, tuple(q[:2])) for n, q in st["positions"].items() if n != robot and q]
            for k, h in enumerate(MAP.ray_headings(n_rays)):
                blocker = MAP.ray_blocked(mid, h, others)
                if blocker:
                    self.say("  ray %2d, %4.0f deg: skipped (%s is parked that way)" % (k + 1, h, blocker))
                    rays.append(dict(k=k, heading=round(h, 1), skipped=blocker, kind=None))
                    continue
                p = self.settle(min_s=0.3)
                if p is None:
                    break
                if math.hypot(p[0] - mid[0], p[1] - mid[1]) > 8.0:
                    ok, why = self.drive_to(robot, mid, max_leg=PARK_LEG_CM, arrive_cm=4.0)
                    if not ok:
                        self.say("  could not get back to the middle (%s): mapping ends here" % why)
                        break
                hd = self.heads.get(robot, [None])[0]
                if hd is None:
                    self.leg(robot, ("go", 5.0))
                    hd = self.heads.get(robot, [None])[0]
                    if hd is None:
                        break
                turn = ST.wrap180(h - hd)
                if abs(turn) >= ST.MIN_TURN_DEG:
                    self.leg(robot, ("turn", turn))
                self.heads[robot] = [h, False]
                self.fix_now(robot, h)
                p0, p1, words = self.leg(robot, ("go", MAP.RAY_CM))
                lid = self.main_lid.get(robot)
                fin = self.listener.leg_final(robot, "-", lid) or {}
                kind = MAP.stop_kind(fin.get("state"), fin.get("info"))
                rx, ry, rth = num(fin.get("x_cm")), num(fin.get("y_cm")), num(fin.get("th_deg"))
                ray = dict(k=k, heading=round(h, 1), state=fin.get("state"), info=fin.get("info"), kind=kind,
                           words=words, cam_start=p0, cam_end=p1,
                           robot_end=[rx, ry, rth] if rx is not None else None)
                if p0 and p1 and math.hypot(p1[0] - p0[0], p1[1] - p0[1]) >= ST.MIN_HEADING_CM:
                    self.heads[robot] = [ST.bearing(p0, p1), True]
                if kind and rx is not None:
                    ray["point"] = MAP.contact_point(rx, ry, rth, kind)
                    if p1:
                        hh = self.heads[robot][0]
                        ray["point_cam"] = MAP.contact_point(p1[0], p1[1], rth if hh is None else hh, kind)
                rays.append(ray)
                self.say("  ray %2d, %4.0f deg: %s%s" % (k + 1, h, {"edge": "open edge", "board": "board"}.get(kind, "nothing met (%s)" % (words or fin.get("state"))),
                                                       (" at (%.1f, %.1f) cm" % tuple(ray["point"])) if ray.get("point") else ""))
                # straight back: off the edge first (a turn on the spot puts a wheel over an edge it is this close
                # to: firmware edge.h), then turn round and forward (the guards look ahead; backing up is blind)
                back = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if (p0 and p1) else None
                q = p1 or p0
                if kind == "edge" or (q and self.board.edge_room(q[0], q[1])[0] < EDGE_TURN_CM + 3):
                    self.leg(robot, ("go", -7.0))
                    back = (back - 7.0) if back else None
                self.leg(robot, ("turn", 180.0))
                if self.heads.get(robot, [None])[0] is not None:
                    self.heads[robot] = [ST.wrap180(self.heads[robot][0] + 180.0), False]
                if back and back > 2.0:
                    self.leg(robot, ("go", back))
        finally:
            self.watch_edge_cm = None
        others = [tuple(q[:2]) for n, q in st["positions"].items() if n != robot and q]
        pts = [(r["point"][0], r["point"][1], r["kind"]) for r in rays if r.get("point")]
        sides = MAP.fit_sides(pts, self.board, others=others)
        for line in MAP.summary_words(sides, self.board):
            self.say(line)
        # the blocked boards: felt vs the picture
        felt = {}
        for key, sd in sides.items():
            if sd["kind"] != "board":
                continue
            if sd["axis"] == "y":
                felt["x"] = sd["value"]          # the x-board is the line y = const
            else:
                felt["y"] = sd["value"]
        img_lines = st["setup"].get("board_lines_image", {})
        for which, v in felt.items():
            pic = img_lines.get(which, {}).get("value")
            if pic is not None:
                self.say("  %s-board: felt at %.1f cm, the picture says %.1f cm (%+.1f cm)" % (which, v, pic, v - pic))
        st["setup"]["map"] = dict(robot=robot, rays=rays, sides=sides, felt_boards=felt, t=round(time.time(), 1))
        st.save()
        MAP.save(os.path.join(self.dir, "map.json"), robot, rays, sides, self.board)
        try:
            self.draw_map(rays, sides)
        except Exception as e:
            self.say("  (map picture failed: %s)" % e, quiet=True)
        self.mark("e02 map end %s" % robot)
        ok, why = self.park(robot)
        if not ok:
            self.help_from_nick(robot, why)
        return sides

    def draw_map(self, rays, sides):
        import cv2
        img = self.feed.last_img
        if img is None:
            return
        v = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for r in rays:
            p0, pt = r.get("cam_start"), r.get("point")
            if p0 and pt:
                a = self.cam.project((p0[0], p0[1], 0.0))
                b = self.cam.project((pt[0], pt[1], 0.0))
                col = (0, 0, 255) if r["kind"] == "edge" else (255, 160, 0)
                cv2.line(v, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), col, 1)
                cv2.circle(v, (int(b[0]), int(b[1])), 3, col, -1)
        path = os.path.join(self.dir, "check_map.png")
        cv2.imwrite(path, v)
        self.say("  The map, drawn on the camera picture: %s (red: open edges, blue: boards)" % path)

    # --- the brick demo (Nick, 2026-10-22; e02_bricks.py) ---------------------------------------------
    def brick_demo(self):
        """Nick puts bricks (or any objects) on the board; the camera finds what changed, plans a path round them,
        and one robot crosses three times on the same path (the ladder, BR.LADDER): (1) factory settings, wheels
        and gyro only; (2) wheels and gyro with the learned corrections, the camera only watching; (3) the same
        path with camera fixes between legs. --tight-gap: the room kept round the objects comes from the robot's
        measured precision this session, and the path goes straight through the narrowest gap it can safely make.
        Then the objects come off and it parks. The edge guard and front stop stay on (and the camera watchdog
        guards the objects): a crossing they stop is recorded as failed, and the robot is backed off before the
        next. Returns the result dict, or None."""
        import e02_bricks as BR
        st = self.st
        names = [n for n in st["robots"] if n not in self.unavailable and st["positions"].get(n)]
        if not names:
            self.say("Brick demo: no robot is parked on the board, so there is nothing to show.")
            return None
        tight = bool(getattr(self.a, "tight_gap", False))
        k = float(getattr(self.a, "gap_k", BR.GAP_K) or BR.GAP_K)
        precs = {n: {m: BR.gap_precision(st.get("schedule") or [], n, m, k=k) for m in BR.LADDER} for n in names}
        print("")
        self.say("=== Brick demo: a robot finds its way %s ===" % (
            "through a tight gap between objects you put down" if tight else "round objects you put down"))
        self.phase, self.order, self.active = "bricks", "bricks", None
        need = min(precs[n]["camera"]["gap_cm"] for n in names)
        if tight:
            self.say("How precise each robot is this session (its camera-measured errors), and so the narrowest gap "
                     "it can safely drive through:")
            for n in names:
                for line in BR.precision_words(n, precs[n]):
                    self.say(line)
        self.say("Taking a picture of the board as it is now (keep your hands out of the picture) ...")
        self.fresh_floor()
        time.sleep(0.5)
        floor = BR.floor_snapshot(self.tracker)
        if floor is None:
            self.say("Brick demo: the camera has no picture of the floor yet; no demo this time.")
            return None
        flag = os.path.join(self.dir, "sim", "BRICKS_ON")
        if self.a.dry_run:
            open(flag, "w").close()                 # the simulator puts the bricks down now
            time.sleep(1.0)
        self.mark("e02 bricks place")
        ch = found = None
        while True:
            print("\n" + "-" * 78)
            self.say("Now put the objects down (bricks, or anything to hand: a shoe, a mug, a pen, a cable):")
            if tight:
                self.say("1. Put two objects on the open part of the board with a gap of about %.0f cm (%.0f in) "
                         "between them: at least 30 cm (1 ft) from every robot and at least 20 cm (8 in) from the "
                         "open edges, with clear board on both sides of the gap." % (need + 4.0, (need + 4.0) / 2.54))
                self.say("   (The robot needs %.0f cm as the camera sees it; the camera sees each object about 2 cm "
                         "bigger than it is, hence the extra. Anything wider than %.0f cm is still fine, just less "
                         "impressive.)" % (math.ceil(need), need + 10.0))
            else:
                self.say("1. Put them anywhere on the open part of the board: at least 30 cm (1 ft) from every robot "
                         "and at least 20 cm (8 in) from the open edges.")
                self.say("   Leave at least one gap about 25 cm (10 in) wide between them, so a robot can get through.")
            self.say("2. Don't touch or move the robots.")
            self.say("3. Step back so your hands and your shadow are out of the camera picture.")
            print("-" * 78)
            self.ask("4. Then press Enter here: ", "")
            frames = self.still_frames()
            robots = {n: tuple(st["positions"][n][:2]) for n in names}
            found = BR.find_bricks(self.cam, frames, floor, self.board_rect(), robots=list(robots.values()))
            for line in BR.brick_words(found["bricks"], found):
                self.say(line)
            if not found["bricks"] and not self.a.auto:
                ans = self.ask("Check the objects are inside the board's outline on the live view, then press Enter "
                               "to look again (or type go and press Enter to cross with no objects): ", "")
                if ans.lower() != "go" or tight:
                    continue
            if tight:
                ch = self._choose_tight_gap(found, robots, precs, need)
                if ch is None:
                    if self.a.auto:
                        self._bricks_off(flag)
                        return None
                    self.say("  Move the objects so the gap is about %.0f cm (%.0f in), step back, and press Enter."
                             % (need + 4.0, (need + 4.0) / 2.54))
                    continue
            else:
                ch = BR.choose_crossing(found["occ"], found["grid"], self.board, robots)
            if ch is None:
                self.say("  There is no way across: the objects leave no gap a robot can get through (it needs about "
                         "25 cm, 10 in), or they are too close to the robots.")
                if self.a.auto:
                    self._bricks_off(flag)
                    return None
                self.say("  Move one object to open a gap, step back, and press Enter.")
                continue
            for line in BR.plan_words(ch):
                self.say(line)
            self.bricks_view = dict(bricks=found["bricks"], room_cm=round(ch["space"]["r_cm"], 1),
                                    path=list(ch["path"]["points"]), start=ch["start"], goal=ch["goal"], tracks={})
            pic = self.save_bricks_picture()
            self.say("  The plan is drawn on the live view%s: red outlines are the objects, the blue line is the path, "
                     "the white dot the start and the white ring the goal%s." % (
                         " (http://localhost:%d)" % self.a.view_port if self.a.view_port else "",
                         (" (also saved as %s)" % pic) if pic else ""))
            ans = self.ask("Press Enter to start the three crossings (or type r and press Enter to move the objects "
                           "and look again): ", "")
            if ans.lower().startswith("r"):
                self.bricks_view = None
                continue
            break
        self.mark("e02 bricks found %d" % len(found["bricks"]))
        self.fresh_floor()                 # the bricks are part of the floor now: the tracker follows only the robot
        robot = ch["robot"]
        corr = self.move_corrections(robot)
        source = "learned from the camera in this session's calibration runs" if self.fitted(robot) else \
            "the starting guesses (this session has no calibration runs yet%s)" % (
                "; the gyro creep measured in the naming check" if ((st.get("naming_check") or {}).get(
                    "gyro_raw_drift_dps") or {}).get(robot) is not None else "")
        self.say("  %s crosses. Its corrections: %s (%s)." % (robot, FIT.describe(corr), source))
        out = dict(t=round(time.time(), 1), robot=robot, bricks=found["bricks"], start=ch["start"], goal=ch["goal"],
                   path=ch["path"], approach=ch["approach"]["points"], words=ch["words"],
                   straight_blocked_cm=ch["straight_blocked_cm"], corrections=corr, corrections_source=source,
                   factory=dict(gyro_scale=1.0, gyro_drift_dps=0.0, wheel_scale=1.0),
                   room_cm=round(ch["space"]["r_cm"], 1), settings=dict(margin_cm=BR.MARGIN_CM,
                                                                        open_edge_cm=BR.OPEN_EDGE_CLEAR_CM,
                                                                        board_cm=BR.BOARD_CLEAR_CM,
                                                                        robot_cm=BR.ROBOT_CLEAR_CM),
                   left_out=dict(specks=len(found.get("specks") or []), rim=found.get("rim", 0)),
                   tight_gap=bool(tight), precision=precs.get(robot), crossings=[])
        if ch.get("gap"):
            out.update(gap=ch["gap"], clearance_cm=ch["clearance_cm"], gap_k=k)
        others = [tuple(q[:2]) for n, q in st["positions"].items() if n != robot and q]
        space = BR.free_space(found["occ"], found["grid"], self.board, others=others)
        if ch.get("gap"):              # to and from the start: also back through the gap along its line
            space = BR.corridor_space(space, BR.gap_space(found["occ"], found["grid"], self.board, others,
                                                          ch["clearance_cm"]), ch["path"]["points"])
        self.bricks_guard = space          # the camera watchdog guards the objects from now until they are off
        try:
            if not self.activate(robot):
                self.say("  The camera can't see %s at its spot: no crossings this time." % robot)
                return self._bricks_end(robot, out, flag, space)
            self.set_corrections(robot, corr)
            first = ST.bearing(ch["path"]["points"][0], ch["path"]["points"][1])
            if ch.get("gap"):
                # a straight run through the gap would test only the wheels; starting side-on puts a 90 deg turn
                # first, where an uncorrected gyro shows (about 7 deg off at factory settings: 7 cm sideways at 60 cm)
                first = ST.wrap180(first + 90.0)
                out["start_turn_deg"] = -90.0
            for n, method in enumerate(BR.LADDER, 1):
                self.set_corrections(robot, corr)            # getting to the start: always its learned corrections
                self.say("  Getting %s to the start (the camera steers this part; it is not measured) ..." % robot)
                ok, why = self.bricks_to(robot, space, ch["start"], final_heading=first)
                if not ok:
                    self.say("  %s could not get to the start: %s." % (robot, why))
                    if not self.help_from_nick(robot, "it could not get to the brick demo's start: " + why, park=False):
                        break
                    self.activate(robot)
                    ok, why = self.bricks_to(robot, space, ch["start"], final_heading=first)
                    if not ok:
                        self.say("  Still no way to the start (%s): the demo stops here." % why)
                        break
                if method == "factory":
                    self.begin_moves(robot)                  # (already moving: no change) then factory values
                    self.set_corrections(robot, None)        # the U condition's values: gyro 1, drift 0, wheels 1
                    time.sleep(0.3)
                c = self.brick_crossing(robot, ch, space, n, method)
                self.set_corrections(robot, corr)
                c["corrections"] = None if method == "factory" else corr
                out["crossings"].append(c)
                for line in BR.crossing_words(c):
                    self.say(line)
                BR.save(os.path.join(self.dir, "bricks.json"), out)
                if c.get("safety"):
                    if not self.bricks_back_off(robot, space, c):
                        break
                    moved = self.objects_moved(floor, found, names, robot)
                    if moved:
                        c["objects_moved_cm2"] = moved
                        self.say("  An object was pushed (about %.0f cm2 of the board looks different from the plan)."
                                 % moved)
                        if not self.a.auto:
                            self.ask("  Put it back on its red outline on the live view, step back, and press Enter: ",
                                     "")
                            self.fresh_floor()
        finally:
            self.end_moves(robot)
        return self._bricks_end(robot, out, flag, space)

    def _choose_tight_gap(self, found, robots, precs, need):
        """Tight-gap mode: the gaps between the objects, which one to cross (the narrowest a robot can safely make,
        or the one Nick types), and the crossing straight through it. Returns a crossing dict or None."""
        import e02_bricks as BR
        gaps = BR.find_gaps(found["occ"], found["grid"], found["bricks"])
        best = min(robots, key=lambda n: precs[n]["camera"]["gap_cm"])
        for line in BR.gap_words(gaps, need, best):
            self.say(line)
        ok = [i for i, g in enumerate(gaps) if g["width_cm"] + 1e-6 >= need]
        if not ok:
            self.say("  No gap is wide enough: %s needs %.0f cm (as the camera sees it)." % (best, math.ceil(need)))
            return None
        pick = ok[0]                                    # the narrowest safe gap: the most impressive
        if not self.a.auto and len(gaps) > 1:
            ans = self.ask("Press Enter to cross gap %d (the narrowest it can safely make), or type another gap's "
                           "number and press Enter: " % (pick + 1), "")
            if ans.strip().isdigit() and 1 <= int(ans) <= len(gaps):
                pick = int(ans) - 1
                if pick not in ok:
                    self.say("  Gap %d is too narrow to cross safely; crossing gap %d." % (pick + 1, ok[0] + 1))
                    pick = ok[0]
        clear = {n: precs[n]["camera"]["clearance_cm"] for n in robots}
        for i in [pick] + [j for j in ok if j != pick]:
            ch = BR.choose_gap_crossing(found["occ"], found["grid"], self.board, robots, gaps[i], clear)
            if ch is not None:
                if i != pick:
                    self.say("  Gap %d has no clear board on both sides (or no robot can get there): crossing gap %d."
                             % (pick + 1, i + 1))
                self.say("  %s crosses gap %d: %.1f cm wide as the camera sees it; it needs %.1f cm." % (
                    ch["robot"], i + 1, gaps[i]["width_cm"], 2.0 * ch["clearance_cm"]))
                return ch
        self.say("  No safe gap has clear board on both sides for a straight run through it (about 25 cm each side), "
                 "or no robot can get to it.")
        return None

    def bricks_back_off(self, robot, space, c):
        """After a crossing the safety stopped (front stop, edge guard, watchdog): back the robot straight up, away
        from the objects, so the next crossing starts clean. Returns True when it is free to go on."""
        import e02_bricks as BR
        p = self.settle(min_s=0.4)
        h = c.get("end_heading")
        if p is None or h is None:
            return self.help_from_nick(robot, "the safety stopped it next to an object and the camera can't tell which "
                                              "way it faces", park=False)
        self.heads[robot] = [h, True]
        d = BR.back_off_cm(space, p, h, want_cm=12.0)
        if d < 2.0:
            self.say("  %s is already clear of the objects behind it; it goes on from here." % robot)
            return True
        self.say("  Backing %s up %.0f cm, straight away from what stopped it, before the next crossing ..." % (robot, d))
        self.watch_stop = None
        p0, p1, words = self.leg(robot, ("go", -round(d, 1)))
        c["backed_off_cm"] = round(d, 1)
        if not (words or "").lower().startswith(("done", "dup")):
            self.say("  It could not back up (%s)." % words)
            return self.help_from_nick(robot, "it could not back away from an object: " + words, park=False)
        return True

    def objects_moved(self, floor, found, names, robot):
        """After a stopped crossing: did the robot push an object? The objects again, against the same floor
        picture; returns the board area (cm2) that differs from the plan's objects (0 when under 25 cm2)."""
        import e02_bricks as BR
        try:
            frames = self.still_frames(tries=3)
            pos = []
            for n in names:
                q = self.settle(min_s=0.3) if n == robot else self.st["positions"].get(n)
                if q is not None:
                    pos.append(tuple(q[:2]))
            now = BR.find_bricks(self.cam, frames, floor, self.board_rect(), robots=pos)
            g = found["grid"]
            X, Y = g.centres()
            hidden = np.zeros(g.shape, bool)              # round a robot the camera sees nothing: not compared
            for q in pos + [tuple(self.st["positions"][n][:2]) for n in names if self.st["positions"].get(n)]:
                hidden |= np.hypot(X - q[0], Y - q[1]) < 1.5 * BR.ROBOT_CM + 1.0
            # cells more than 2 cm from the other picture's objects: something new, or something gone
            new = now["occ"] & (BR.brick_distance(found["occ"], g) > 2.0) & ~hidden
            gone = found["occ"] & (BR.brick_distance(now["occ"], g) > 2.0) & ~hidden
            diff = float(new.sum() + gone.sum()) * g.cell ** 2
            return round(diff, 0) if diff >= 25.0 else 0
        except Exception as e:           # a check must never stop the demo
            self.say("  (could not check the objects: %s)" % e, quiet=True)
            return 0

    def still_frames(self, tries=6):
        """Pictures of the board once nothing moves in it any more (Nick's hands gone)."""
        import e02_bricks as BR
        region = None
        a = b = []
        for k in range(tries):
            time.sleep(1.0)
            a = self.feed.wait_frames(6, timeout=5)
            time.sleep(0.8)
            b = self.feed.wait_frames(6, timeout=5)
            if len(a) < 3 or len(b) < 3:
                continue
            if region is None:
                region = BR.board_mask(self.cam, a[0].shape, self.board_rect()) > 0
            share = BR.moving_share(a, b, region=region)
            if share < 0.004:
                return a + b
            self.say("  Something is still moving in the picture (a hand or a shadow?): please step back from the "
                     "board. Looking again ...")
        return a + b

    def save_bricks_picture(self):
        try:
            import cv2
            import e02_view
            img = self.feed.last_img
            if img is None or not getattr(self, "bricks_view", None):
                return None
            v = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
            e02_view.draw_bricks(v, self.cam, self.bricks_view)
            path = os.path.join(self.dir, "check_bricks.png")
            cv2.imwrite(path, v)
            return path
        except Exception as e:                # a picture must never stop the demo
            self.say("  (brick picture not saved: %s)" % e, quiet=True)
            return None

    def bricks_to(self, robot, space, target, final_heading=None):
        """Camera-steered (not measured) to target along a path round the bricks and the other robots. Returns
        (ok, words)."""
        import e02_bricks as BR
        p = self.settle(min_s=0.3)
        if p is None:
            p = self.refind(robot, "the camera can't see it")
            if p is None:
                return False, "the camera can't see it"
        plan = BR.plan_path(space, p, target, escape_cm=25.0)
        if not plan["ok"]:
            return False, plan["why"]
        pts = plan["points"][1:]
        for k, w in enumerate(pts):
            last = k == len(pts) - 1
            ok, why = self.drive_to(robot, w, final_heading=final_heading if last else None,
                                    arrive_cm=2.0 if last else 3.0, max_leg=40.0, avoid=False)
            if not ok:
                return False, why
        return True, ""

    @staticmethod
    def crossing_stop(words, p1):
        """Plain words of what ended a crossing leg early, or '' (it ran)."""
        low = (words or "").lower()
        if low.startswith("done") or low.startswith("dup"):
            return ""
        if "obstacle" in low:
            return "its front stop saw something just ahead and stopped it (%s)" % words
        if "edge" in low:
            return "its edge guard stopped it at an open edge (%s)" % words
        if low.startswith("watchdog") or "stop command" in low:   # the watchdog's stop can beat its own note
            return "the camera watchdog stopped it (%s)" % words
        if low.startswith("camera and wheels"):
            return "" if p1 is not None else "the camera lost it (%s)" % words
        return "a leg did not run as asked (%s)" % words

    def brick_crossing(self, robot, ch, space, n, method="calibrated"):
        """One crossing (rung n of the ladder) from where the robot stands now (at the start, facing along the path)
        to the goal. method "factory" / "calibrated": the legs are worked out once, from the camera's start position
        and heading, and driven on the robot's own wheels and gyro (with whatever corrections it was given: none for
        factory); the camera only watches. "camera": after every leg the camera picks the next one from where it
        sees the robot and tells the robot where it is (a fix). c["safety"]: the front stop, edge guard or camera
        watchdog ended it (counted as failed); c["end_heading"]: which way it faced at the end, from the camera."""
        import e02_bricks as BR
        how = BR.METHOD_WORDS[method]
        c = dict(n=n, method=method, how=how, failed="", safety=False, legs_done=0, legs=[])
        turned = 0.0                  # turns sent since the last go leg the camera saw move (for end_heading)
        last_go = None
        p = self.settle(min_s=0.5)
        h, measured = self.heads.get(robot, [None, False])
        if p is None or h is None:
            c["failed"] = "the camera did not see where it starts, or which way it faces"
            return c
        self.fix_now(robot, h)
        goal = tuple(ch["goal"])
        pts = [tuple(p)] + [tuple(q) for q in ch["path"]["points"][1:]]
        c.update(start_cam=[round(p[0], 2), round(p[1], 2)], start_heading=round(h, 1), heading_measured=bool(measured))
        self.say("  --- Crossing %d: %s ---" % (n, how))
        self.say("  %s" % ({"factory": "Factory settings: no corrections at all (gyro scale 1, drift 0, wheel scale 1). "
                                       "The camera only watches; the front stop is there if it drifts into an object.",
                            "calibrated": "The camera only watches and measures now: the robot drives the planned legs "
                                          "on its own wheels and gyro, with the corrections it learned."}.get(
            method, "After every leg the camera says where the robot really is, and the next leg is worked out from "
                    "there.")))
        self.mark("e02 bricks crossing %d start %s" % (n, robot))
        t0 = t_last = time.time()
        track = []
        self.bricks_view.setdefault("tracks", {})[n] = track

        def keep_track():
            nonlocal t_last
            now = time.time()
            track.extend((round(q[1], 2), round(q[2], 2)) for q in self.history.since(t_last, now))
            t_last = now

        def say_leg(k, total, cmd):
            if cmd[0] == "turn":
                w = "turn %.0f deg %s" % (abs(cmd[1]), "left" if cmd[1] > 0 else "right")
            else:
                w = "drive %.0f cm %s" % (abs(cmd[1]), "forward" if cmd[1] >= 0 else "back")
            self.say("    leg %d%s: %s" % (k, (" of %d" % total) if total else "", w))
        def seen(cmd, p0, p1):
            nonlocal turned, last_go
            if cmd[0] == "turn":
                turned += float(cmd[1])
            elif cmd[0] == "go" and p0 is not None and p1 is not None and math.hypot(p1[0] - p0[0], p1[1] - p0[1]) >= 3.0:
                last_go = ST.bearing(p0, p1) if cmd[1] > 0 else ST.bearing(p1, p0)
                turned = 0.0
        if method != "camera":
            legs = BR.legs_from(p, h, pts)
            c["planned_legs"] = [lg["cmd"] for lg in legs]
            for k, lg in enumerate(legs, 1):
                say_leg(k, len(legs), lg["cmd"])
                if method == "factory" and robot not in self.moves:
                    self.begin_moves(robot)                  # (re)starting moves sets its learned corrections:
                    self.set_corrections(robot, None)        # factory values again for this crossing
                p0, p1, words = self.leg(robot, lg["cmd"])
                keep_track()
                seen(lg["cmd"], p0, p1)
                c["legs"].append(dict(cmd=lg["cmd"], result=words, cam_end=p1))
                why = self.crossing_stop(words, p1)
                if why:
                    c["failed"] = why
                    break
                c["legs_done"] += 1
        else:
            steer = ST.Steer(pts[1:], heading=h, measured=bool(measured), arrive_cm=2.0, max_leg=30.0, max_legs=40)
            k = 0
            while True:
                q = self.settle(min_s=0.3, timeout=2.0)
                if q is None:
                    c["failed"] = "the camera lost it"
                    break
                cmd = steer.next(q)
                if cmd[0] == "done" or (cmd[0] == "turn" and abs(cmd[1]) < ST.MIN_TURN_DEG):
                    break
                if cmd[0] == "gave_up":
                    c["failed"] = "it did not reach the goal in %d legs" % steer.max_legs
                    break
                k += 1
                say_leg(k, 0, cmd)
                p0, p1, words = self.leg(robot, cmd)
                keep_track()
                seen(cmd, p0, p1)
                c["legs"].append(dict(cmd=cmd, result=words, cam_end=p1))
                why = self.crossing_stop(words, p1)
                if why:
                    c["failed"] = why
                    break
                c["legs_done"] += 1
                if words.lower().startswith("camera and wheels"):
                    steer.heading, steer.measured = None, False       # found again by a turn: heading unknown
                    steer.legs += 1
                    continue
                if p0 is None or p1 is None:
                    c["failed"] = "the camera lost it"
                    break
                steer.after(cmd, p0, p1)
                self.heads[robot] = [steer.heading, steer.measured]
                self.fix_now(robot, steer.heading if steer.measured else None)    # the camera fix
        time.sleep(0.3)
        endp = self.settle(t_from=time.time() - 0.8, min_s=0.5)
        keep_track()
        c["safety"] = bool(c["failed"]) and any(w in c["failed"] for w in ("front stop", "edge guard", "watchdog"))
        if c["safety"]:
            c["failed"] = "a safety stop: " + c["failed"]
        if last_go is not None:
            c["end_heading"] = round(ST.wrap180(last_go + turned), 1)
        elif method == "camera" and self.heads.get(robot, [None, False])[1]:
            c["end_heading"] = self.heads[robot][0]
        c["seconds"] = round(time.time() - t0, 1)
        self.mark("e02 bricks crossing %d end %s" % (n, "failed" if c["failed"] else "done"))
        if endp is not None:
            c["cam_end"] = [round(endp[0], 2), round(endp[1], 2)]
            c["end_err_cm"] = round(math.hypot(endp[0] - goal[0], endp[1] - goal[1]), 2)
        fin = self.listener.leg_final(robot, "-", self.main_lid.get(robot)) or {}
        bx, by = num(fin.get("x_cm")), num(fin.get("y_cm"))
        if bx is not None and by is not None:
            c["robot_end"] = [round(bx, 2), round(by, 2)]
            c["robot_thought_cm"] = round(math.hypot(bx - goal[0], by - goal[1]), 2)
        c["closest"] = BR.closest_approach(space, track)
        c["track"] = track
        return c

    def _bricks_off(self, flag):
        if self.a.dry_run and os.path.exists(flag):
            os.remove(flag)                          # the simulator takes the bricks off
            time.sleep(1.0)

    def _bricks_end(self, robot, out, flag, space):
        """Bricks off (Nick), a fresh floor memory, the robot parked; bricks.json and the plain summary."""
        import e02_bricks as BR
        p = self.settle(min_s=0.3)
        if p is not None:
            self.st["positions"][robot] = [p[0], p[1], self.heads.get(robot, [None])[0]]
        print("\n" + "-" * 78)
        self.say("The crossings are over. Take the objects off the board now (don't move the robots), step back so "
                 "your hands are out of the picture, and press Enter: %s then parks itself." % robot)
        print("-" * 78)
        if self.a.auto:
            ans = ""
            self._bricks_off(flag)
        else:
            ans = self.ask_timeout("Press Enter when the objects are off (or type k and press Enter to leave them: it "
                                   "then goes round them to its spot): ", self.a.help_wait_min * 60.0)
        kept = ans is None or ans.lower().startswith("k")
        self.mark("e02 bricks %s" % ("kept" if kept else "off"))
        self.bricks_view = None
        if not kept:
            self.bricks_guard = None                 # nothing left on the board to guard
            time.sleep(1.0)
            self.fresh_floor()
        if self.activate(robot):
            if kept:
                i = self.park_index(robot)
                ok, why = self.bricks_to(robot, space, self.board.stage(i))
                if not ok:
                    self.say("  It could not find a way round the objects to its spot (%s)." % why)
            self.set_corrections(robot, self.move_corrections(robot))
            ok, why = self.park(robot)
        else:
            ok, why = False, "the camera can't see it"
        if not ok:
            self.help_from_nick(robot, why)
        self.bricks_guard = None
        out["bricks_taken_off"] = not kept
        summary = []
        for c in out["crossings"]:
            summary.append(dict(n=c["n"], method=c.get("method"), how=c["how"], end_err_cm=c.get("end_err_cm"),
                                robot_thought_cm=c.get("robot_thought_cm"), failed=c.get("failed") or "",
                                safety_stop=bool(c.get("safety")),
                                closest_centre_cm=(c.get("closest") or {}).get("centre_cm"),
                                closest_side_cm=(c.get("closest") or {}).get("body_cm"), legs=c.get("legs_done"),
                                seconds=c.get("seconds")))
        out["summary"] = summary
        BR.save(os.path.join(self.dir, "bricks.json"), out)
        self.st["bricks_demo"] = dict(t=out["t"], robot=robot, n_bricks=len(out["bricks"]), crossings=summary,
                                      tight_gap=out.get("tight_gap", False),
                                      gap_cm=(out.get("gap") or {}).get("width_cm"),
                                      narrowest_gap_cm=((out.get("precision") or {}).get("camera") or {}).get("gap_cm"))
        self.st.save()
        print("")
        self.say("=== Brick demo: what happened ===")
        for line in BR.brick_words(out["bricks"]):
            self.say(line)
        if out.get("precision"):
            for line in BR.precision_words(robot, out["precision"]):
                self.say(line)
        if out.get("gap"):
            self.say("  It was asked to cross a %.1f cm gap (as the camera sees it)." % out["gap"]["width_cm"])
        for c in out["crossings"]:
            for line in BR.crossing_words(c):
                self.say(line)
        if summary:
            self.say("  The three crossings side by side (same path; distance from the goal by the camera; closest "
                     "approach = the gap between its side and the nearest object):")
            for line in BR.ladder_words(out["crossings"]):
                self.say(line)
        self.say("  Saved: %s" % os.path.join(self.dir, "bricks.json"))
        return out

    def safety_settings(self, names):
        """The edge guard on and the bare body (no nose bracket fitted), sent and read back with `get`. A robot
        that does not confirm them gets them again; still not: the session refuses to start."""
        for attempt in range(3):
            todo = []
            for n in names:
                t0 = time.time()
                for c in SAFETY_CMDS:
                    self.send(n, c)
                    time.sleep(0.05)
                for k, _ in SAFETY_SETTINGS:
                    self.send(n, "get %s" % k)
                    time.sleep(0.05)
                time.sleep(1.0)
                got = self.listener.got_params(n, t0)
                bad = [k for k, v in SAFETY_SETTINGS if got.get(k) is None or abs(got[k] - v) > 1e-3]
                if bad:
                    todo.append((n, bad))
            if not todo:
                self.say("Edge guard on and no nose bracket set on %s (read back)." % ", ".join(names))
                self.st["setup"]["safety_settings"] = dict(SAFETY_SETTINGS)
                return
            names = [n for n, _ in todo]
        raise SystemExit("These robots did not confirm the edge guard settings: %s. Check they are switched on and "
                         "on the WiFi, then start this again." % "; ".join("%s (%s)" % (n, ", ".join(b)) for n, b in todo))

    # --- the find check (setup) --------------------------------------------------------------------
    def board_params(self):
        """The board settings every find and route needs (firmware sides are -1 / +1), the route start spot
        (as near the y-board as still lets it turn: the most room for the 100 cm straight), the L's turn
        towards the board's middle, and the same stand-still after every leg for every condition (camera
        fixes are only taken while still: KC needs it, the others get it too so all four drive the same way)."""
        b = self.board
        xs, ys = b.firmware_sides()
        sx, sy, _ = b.nominal_start()
        return ["set e02_xboard_side %d" % xs, "set e02_yboard_side %d" % ys,
                "set e02_xboard_cm %.2f" % b.xboard_cm, "set e02_yboard_cm %.2f" % b.yboard_cm,
                "set e02_pause_ms %d" % PAUSE_MS, "set e02_start_x %.2f" % sx, "set e02_start_y %.2f" % sy,
                "set e02_l_turn %d" % (1 if b.turn_sign > 0 else -1), "set e02_leg_pause_ms %d" % LEG_PAUSE_MS,
                "set e02_transit_max_cm %d" % TRANSIT_MAX_CM] + SAFETY_CMDS

    def find_check(self):
        """One `run find` per robot before the session: does it work, and where does the camera see each
        robot touching each board (a second measure of the board lines)."""
        res = {}
        for n in self.st["robots"]:
            if n in self.unavailable:
                continue
            self.say("Find check: %s finds itself against both boards ..." % n)
            if not self.activate(n):
                self.say("  The camera can't see %s at its spot." % n)
                continue
            self.set_corrections(n, self.move_corrections(n))
            for c in self.board_params():
                self.send(n, c)
                time.sleep(0.05)
            out = self.do_find(n, "setup")
            res[n] = out
            self.say("  %s: %s" % (n, out["words"]))
            ok, why = self.park(n)
            if not ok and not self.help_from_nick(n, why):
                continue
        self.active = None
        lines = defaultdict(list)
        for n, o in res.items():
            for which, v in (o.get("board_lines") or {}).items():
                lines[which].append(v)
        img_lines = self.st["setup"]["board_lines_image"]
        summary = {}
        for which in ("x", "y"):
            v = lines.get(which) or []
            if not v:
                continue
            med = float(np.median(v))
            spread = float(np.ptp(v)) if len(v) > 1 else 0.0
            diff = med - img_lines[which]["value"]
            summary[which] = dict(touch=round(med, 2), spread=round(spread, 2), picture=img_lines[which]["value"],
                                  difference=round(diff, 2), n=len(v))
            self.say("  %s-board: where the robots touched it %.2f cm (%d robots, within %.1f cm); in the picture "
                     "%.2f cm" % (which, med, len(v), spread, img_lines[which]["value"]))
            if abs(diff) > 1.5:
                self.say("  WARNING: the two measures of the %s-board differ by %.1f cm (in the data; the picture's "
                         "line is kept)." % (which, diff))
            if img_lines[which].get("from_edge") and len(v) >= 2 and spread <= 1.0:
                self.say("  -> using where the robots touched it (%.2f cm): the picture gave no clear line." % med)
                if which == "x":
                    self.board.xboard_cm = med
                else:
                    self.board.yboard_cm = med
        self.st["board"] = self.board.to_dict()
        self.tracker_area()
        self.st["setup"]["find_check"] = dict(robots=res, lines=summary)
        self.st.save()
        self.correct_map(summary)

    def correct_map(self, lines):
        """Mapping's board points assume where the front stop halts a robot (MAP.TOF_AHEAD_CM + SAFE_FRONT_CM ahead
        of the axle); the find check's bumps measure the boards directly. Their difference is the real stopping
        distance: the map's board points are moved by it and fitted again (2026-10-22)."""
        m = self.st["setup"].get("map")
        if not m or not lines:
            return
        offs = [m["felt_boards"][w] - lines[w]["touch"] for w in ("x", "y")
                if w in (m.get("felt_boards") or {}) and w in lines]
        if not offs:
            return
        off = float(np.mean(offs))
        # felt beyond the boards means the robot stops nearer the board than assumed: move the points back
        for r in m["rays"]:
            if r.get("kind") == "board" and r.get("point") and r.get("robot_end"):
                th = math.radians(r["robot_end"][2])
                r["point"] = [r["point"][0] - off * math.cos(th), r["point"][1] - off * math.sin(th)]
        pts = [(r["point"][0], r["point"][1], r["kind"]) for r in m["rays"] if r.get("point")]
        others = [tuple(q[:2]) for n, q in self.st["positions"].items() if n != m["robot"] and q]
        m["sides"] = MAP.fit_sides(pts, self.board, others=others)
        m["board_offset_cm"] = round(off, 2)
        m["felt_boards_corrected"] = {}
        for sd in m["sides"].values():
            if sd["kind"] == "board":
                m["felt_boards_corrected"]["x" if sd["axis"] == "y" else "y"] = sd["value"]
        self.say("  Mapping vs the bumps: the front stop halts a robot %.1f cm %s a board than assumed; the map's "
                 "board points are moved by that (%s)." % (abs(off), "nearer" if off > 0 else "farther from",
                                                          ", ".join("%s-board %.1f cm" % kv for kv in
                                                                    sorted(m["felt_boards_corrected"].items()))))
        self.st.save()
        MAP.save(os.path.join(self.dir, "map.json"), m["robot"], m["rays"], m["sides"], self.board)

    def do_find(self, robot, order):
        """A camera fix (the firmware wants one before every find), then `run find`, and the camera's view of
        it. Returns dict(ok, words, run, board_lines, t0, t1)."""
        self.end_moves(robot)
        t0 = time.time()
        self.phase = "find"
        self.fix_now(robot)
        self.watchdog_fired = False
        self.watch_stop = None
        self.mark("e02 %s %s find start" % (order, robot))
        run = self.start_run(robot, "run find")
        if not run:
            return dict(ok=False, words="did not start `run find`", run=None, t0=t0, t1=time.time())
        end = self.watch.wait_end(run, FIND_TIMEOUT_S, poll=lambda: self.watchdog(robot))
        t1 = time.time()
        if end is None:
            self.send(robot, "stop")
            end = self.watch.wait_end(run, 5) or dict(result="no end record")
        fr = self.listener.latest("FIND", robot, t0)
        # find ends squared up to the boards, backed off: its own heading is then as good as the camera's
        ath = num(fr[1].get("after_th")) if fr else None
        self.heads[robot] = [ath, True] if ath is not None else [ST.wrap180(self.board.start_heading + 180.0), False]
        result = fr[1].get("result", "?") if fr else end["result"]
        ok = end["result"] == "done" and (fr is None or result == "ok")
        out = dict(ok=ok, run=run, t0=t0, t1=t1, result=result, runend=end["result"],
                   words="found itself" if ok else "find did not finish (%s, %s)" % (result, end["result"]))
        # where the camera saw it touch each board
        lines = {}
        raw = self.listener.raw.get(run) or []
        off = FIT.clock_offset(raw)
        with self.listener.cv:
            bumps = [(t, rec) for t, n, rec in self.listener.recs["BUMP"] if n == robot and rec.get("run") == run]
        prm = self.watch.params.get(run, {})
        face = PL.ROBOT_HALF_CM + (prm.get("bumper_cm") or 0.0)      # body centre (the camera's point) to its face
        if prm.get("bumper_cm") and prm.get("bump_ahead_cm"):
            self.board.face_cm = prm["bump_ahead_cm"]
        for _, rec in bumps:
            ms = num(rec.get("ms_contact")) or num(rec.get("ms_detect")) or num(rec.get("ms"))
            which = (rec.get("board") or "").strip().lower()[:1]
            if ms is None or off is None or which not in ("x", "y"):
                continue
            tb = ms / 1000.0 + off
            smp = self.history.since(tb - 0.2, tb + 2.5)        # the bump, the push to square, the back-off
            sp = None
            if len(smp) >= 5:
                # where it pressed against the board: the samples closest to it
                key = (lambda q: q[2] * self.board.uy) if which == "x" else (lambda q: q[1] * self.board.ux)
                near = sorted(smp, key=key)[:max(3, len(smp) // 5)]
                sp = (float(np.median([q[1] for q in near])), float(np.median([q[2] for q in near])))
            if sp:
                if which == "x":
                    lines["x"] = sp[1] - self.board.uy * face
                else:
                    lines["y"] = sp[0] - self.board.ux * face
        out["board_lines"] = {k: round(v, 2) for k, v in lines.items()}
        out["bumps"] = len(bumps)
        out["bump_results"] = [rec.get("result") for _, rec in bumps]
        self.mark("e02 %s %s find end %s" % (order, robot, end["result"]))
        return out

    def watchdog(self, robot=None):
        """For wait_end polls: has the camera watchdog stopped the robot in this run?"""
        return self.watchdog_fired

    def watchdog_loop(self):
        """The camera watchdog, all session, 10 times a second, for whichever robot the camera follows (finds,
        routes, K legs, parking, idle legs alike): a robot whose centre is, or within 0.3 s will be, closer than
        WATCH_EDGE_CM to an open edge or WATCH_ROBOT_CM to a parked robot, while moving towards it, gets `stop`
        (three times, 0.2 s apart: a lost WiFi packet must not matter). Moving away is always allowed, so it can be
        brought back. The firmware's edge guard and front stop are the first line; this is the second."""
        last = {}
        while not self.watch_quit.is_set():
            time.sleep(0.1)
            try:
                robot = self.tracker.active if self.tracker is not None else None
                if not robot or self.board is None:
                    continue
                smp = self.history.since(time.time() - 0.6)
                if len(smp) < 3 or time.time() - smp[-1][0] > 1.0:
                    continue
                parked = {n: tuple(q[:2]) for n, q in self.st["positions"].items() if n != robot and q}
                why = ST.hazard(smp, self.board, parked, edge_cm=self.watch_edge_cm)
                if not why and getattr(self, "bricks_guard", None) is not None:      # the brick demo's objects
                    import e02_bricks as BR
                    why = BR.object_hazard(smp, self.bricks_guard)
                if not why or time.time() - last.get(robot, 0) < 2.0:
                    continue
                last[robot] = time.time()
                for _ in range(3):
                    self.send(robot, "stop")
                    time.sleep(0.2)
                self.watchdog_fired = True
                self.watch_stop = why
                self.st.setdefault("watchdog_stops", []).append([round(time.time(), 1), robot, why, self.order])
                self.say("  STOPPED %s (camera watchdog): %s" % (robot, why))
                self.mark("e02 watchdog stop %s" % robot)
            except Exception as e:          # the watchdog must never die
                self.say("  (watchdog: %s)" % e, quiet=True)

    # --- the naming check ---------------------------------------------------------------------------
    def naming_check(self):
        """2 minutes, everything parked, all blinking their codes (`run gyro` 120 s, which also measures each
        gyro's drift), the blink tracker of tools/live naming them. Recorded only."""
        names = [n for n in self.st["robots"] if n not in self.unavailable]
        secs = self.a.naming_s
        self.say("=== Naming check: %d s, the robots stand still blinking their codes ===" % secs)
        live = None
        try:
            live = NamingWatch(self.cam, self.dir)
            self.feed.hooks.append(live.push)
        except Exception as e:      # tools/live missing a piece: the check is data only
            self.say("  (the blink tracker could not start: %s; recording the check without it)" % e)
        self.mark("e02 naming check start")
        t0 = time.time()
        for n in names:
            self.send(n, "set gyro_check_s %d" % secs)
        time.sleep(0.3)
        for n in names:
            self.send(n, "run gyro")
        time.sleep(secs + 6)
        self.mark("e02 naming check end")
        if live:
            self.feed.hooks.remove(live.push)
        drift = {}
        for n in names:
            r = self.listener.latest("GYRO", n, t0)
            if r:
                drift[n] = num(r[1].get("raw_drift_dps"))
        known = {n: self.st["positions"].get(n) for n in names}
        result = dict(t0=t0, seconds=secs, gyro_raw_drift_dps=drift,
                      blink=live.summary(known) if live else None)
        self.st["naming_check"] = result
        with open(os.path.join(self.dir, "naming_check.json"), "w") as f:
            json.dump(result, f, indent=1)
        if live:
            for n, r in result["blink"]["robots"].items():
                self.say("  %s: %s" % (n, r["words"]))
        for n, d in drift.items():
            if d is not None:
                self.say("  %s's gyro creeps %+.3f deg/s standing still (starting point for its drift correction)" % (n, d))
        self.st.save()

    # --- corrections ------------------------------------------------------------------------------
    def prior(self, robot):
        p = dict(FIT.PRIOR)
        d = ((self.st.get("naming_check") or {}).get("gyro_raw_drift_dps") or {}).get(robot)
        if d is not None:
            p["gyro_drift_dps"] = round(p["gyro_scale"] * d, 4)
        return p

    def fitted(self, robot):
        return (self.st["fits"].get(robot) or {}).get("values")

    def move_corrections(self, robot):
        """For moves between runs (parking): the latest learned values, else the priors."""
        return self.fitted(robot) or self.prior(robot)

    def set_corrections(self, robot, values):
        """U / K: factory values (1, 0, 1); C / KC / moves: the given ones. Sent as `set` (verified against the
        run's PARAM records); `corr reset` would do the same for U but also rewrite the robot's saved
        corrections, and the simulator does not have `corr`."""
        if values is None:
            values = dict(gyro_scale=1.0, gyro_drift_dps=0.0, wheel_scale=1.0)
        cmds = ["set gyro_scale %.4f" % values["gyro_scale"], "set gyro_drift_dps %.4f" % values["gyro_drift_dps"],
                "set wheel_scale %.4f" % values["wheel_scale"]]
        for c in cmds:
            self.send(robot, c)
            time.sleep(0.05)
        return cmds

    def learn(self, robot, run_row, t0, t1, runs):
        """After a U run: pair the camera's straight stretches with the robot's raw counting, refit."""
        cam = [(s[0], s[1], s[2]) for s in self.history.since(t0, t1)]
        stretches = FIT.camera_stretches(cam)
        obs = []
        for run in runs:
            raw = self.listener.raw.get(run) or []
            info = self.listener.runinfo.get(run) or {}
            ys = num(info.get("yaw_sign")) or 1
            ser = FIT.raw_series(raw, yaw_sign=1 if ys >= 0 else -1)
            if ser is None:
                continue
            k = (ser["t"] >= ser["t"][0] - 0.1) & (ser["t"] <= ser["t"][-1] + 0.1)
            mine = [s for s in stretches if s["t0"] >= ser["t"][k][0] and s["t1"] <= ser["t"][k][-1]]
            obs += FIT.pair(mine, ser, run=run)
        data = self.st["fit_data"].setdefault(robot, [])
        data += obs
        prev = self.fitted(robot)
        measured_drift = ((self.st.get("naming_check") or {}).get("gyro_raw_drift_dps") or {}).get(robot) is not None
        res = FIT.fit_robot(data, prev, prior=self.prior(robot),
                            prior_sd=dict(gyro_drift_dps=0.1) if measured_drift else None)
        entry = dict(t=round(time.time(), 1), order=run_row["order"], n_new=len(obs), n_total=len(data),
                     n_stretches=len(stretches), accepted=res["accepted"], fitted=res["fitted"], reasons=res["reasons"],
                     values=res["values"], gyro_ok=res["gyro_ok"], wheel_ok=res["wheel_ok"])
        f = self.st["fits"].setdefault(robot, dict(values=None, history=[]))
        f["history"].append(entry)
        if res["values"] and res["accepted"]:
            f["values"] = res["values"]
        run_row["fit_obs"] = len(obs)
        self.write_fits()
        words = FIT.describe(f["values"])
        self.say("  Self-calibration %s (%d stretches from this run, %d in all): %s%s" % (
            robot, len(obs), len(data), words, ("; " + "; ".join(res["reasons"])) if res["reasons"] else ""))

    # --- one run ----------------------------------------------------------------------------------
    def do_run(self, r):
        robot, cond, route = r["robot"], r["cond"], r["route"]
        st = self.st
        done, total = st.counts()
        el = time.time() - st["t_session_start"] + st.get("elapsed_before", 0)
        left = (total - done) * st.mean_run_s()
        print("")
        self.say("=== Run %d of %d (block %d): %s, %s (%s), route %s | %s in, about %s left ===" % (
            done + 1, total, r["block"], robot, cond, PL.COND_WORDS[cond], PL.ROUTE_NAMES[route], hms(el), hms(left)))
        if self.last_result:
            self.say("    last result: " + self.last_result)
        r["status"] = "running"
        r["t_start"] = round(time.time(), 2)
        st.save()
        self.order = str(r["order"])
        if not self.activate(robot):
            ok = self.recover(robot, "the camera can't see it at its spot before its run")
            if not ok:
                r["status"] = "pending"
                st.save()
                return
        mode = 1 if cond == "K" else 0
        tag = "%d/%d %s %s r%d" % (done + 1, total, robot, cond, route)
        # 1. find: every find with the same corrections (the latest learned, else the priors), so the routes
        # all start from an equally good find; the condition's corrections are set after it (the firmware keeps
        # the heading when they change)
        find_corr = self.move_corrections(robot)
        self.end_moves(robot)
        fcmds = self.set_corrections(robot, find_corr)
        rdeg = self.board.route_heading(route)
        params = self.board_params() + ["set e02_route %d" % route, "set e02_mode %d" % mode,
                                        "set e02_route_deg %.1f" % rdeg]
        for c in params:
            self.send(robot, c)
            time.sleep(0.05)
        t_run0 = time.time()
        fnd = self.do_find(robot, tag)
        r["find_run"], r["find_result"] = fnd["run"], fnd.get("result")
        r["find_board_lines"] = fnd.get("board_lines")
        r["find_corr"] = find_corr
        if fnd["run"]:
            bad, unseen = check_params(self.watch, fnd["run"], list(params) + fcmds)
            if bad:
                self.say("  %s did not get every setting (a WiFi packet was lost): %s. Redoing this run." % (robot, "; ".join(bad)))
                r["redos"] = r.get("redos", 0) + 1
                r["status"] = "pending" if r["redos"] < 3 else "failed"
                r.setdefault("notes", []).append("settings lost: " + "; ".join(bad))
                self.park(robot)
                st.save()
                return
        if not fnd["ok"]:
            self.say("  Find failed: %s" % fnd["words"])
            r.setdefault("notes", []).append("find failed: " + fnd["words"])
            if r.get("find_redos", 0) < 1:
                # a missed bump or a nudged robot: park it and do this run again later in its block (once)
                r["find_redos"] = r.get("find_redos", 0) + 1
                r["status"] = "pending"
                st.save()
                self.say("  This run is done again later in its block.")
                self.set_corrections(robot, self.move_corrections(robot))
                if not self.recover(robot, "find failed"):
                    self.say("  %s could not be parked; its remaining runs are put aside." % robot)
                self.active = None
                return
            r.setdefault("flags", []).append("find_failed")
            self.finish_failed(r, robot, "find failed: " + fnd["words"])
            return
        # 2. the condition's corrections. Mode 0 (U, C, KC): the transit to the start spot and the start turn run on
        # the find's corrections; the firmware switches to the condition's at the route's start (e02_route_*), so
        # the route alone is what is measured. K: factory values, set now (the laptop steers it all the way).
        source = "factory"
        if cond in ("C", "KC"):
            vals = self.fitted(robot)
            if vals is None:
                vals = self.prior(robot)
                source = "prior (no learned values yet)"
                self.say("  No learned corrections for %s yet: this %s run uses the priors (flagged)." % (robot, cond))
                r.setdefault("flags", []).append("prior_corrections")
            else:
                source = "learned"
        else:
            vals = dict(gyro_scale=1.0, gyro_drift_dps=0.0, wheel_scale=1.0)
        r["corr"] = vals
        if mode == 0:
            cmds = ["set e02_route_gyro_scale %.4f" % vals["gyro_scale"],
                    "set e02_route_gyro_drift_dps %.4f" % vals["gyro_drift_dps"],
                    "set e02_route_wheel_scale %.4f" % vals["wheel_scale"]]
            for c in cmds:
                self.send(robot, c)
                time.sleep(0.05)
            cmds = cmds + fcmds
        else:
            cmds = self.set_corrections(robot, vals) + ["set e02_route_gyro_scale -999",
                                                         "set e02_route_gyro_drift_dps -999",
                                                         "set e02_route_wheel_scale -999"]
            for c in cmds[3:]:
                self.send(robot, c)
                time.sleep(0.05)
        r["corr_source"] = source
        time.sleep(0.3)
        # 3. the route
        self.phase = "route"
        pre = self.settle(min_s=0.5)
        nominal = rdeg
        t_e0 = time.time()
        self.mark("e02 %s route start" % tag)
        run = self.start_run(robot, "run e02")
        if not run:
            r.setdefault("flags", []).append("e02_no_start")
            self.finish_failed(r, robot, "did not start `run e02`")
            return
        r["e02_run"] = run
        bad, _ = check_params(self.watch, run, cmds + ["set e02_route %d" % route, "set e02_mode %d" % mode])
        if bad:
            self.send(robot, "stop")
            self.watch.wait_end(run, 5)
            self.say("  %s did not get every setting (a WiFi packet was lost): %s. Redoing this run." % (robot, "; ".join(bad)))
            r["redos"] = r.get("redos", 0) + 1
            r["status"] = "pending" if r["redos"] < 3 else "failed"
            r.setdefault("notes", []).append("settings lost: " + "; ".join(bad))
            self.park(robot)
            st.save()
            return
        prm = self.watch.params.get(run, {})
        k_info = None
        self.watchdog_fired = False
        self.watch_stop = None
        if cond == "KC":
            self.kc_stop.clear()
            th = threading.Thread(target=self.kc_fixes, args=(robot, r, run), daemon=True)
            th.start()
        if cond == "K":
            k_info = self.k_drive(robot, run, route, prm, nominal)
            end = self.watch.wait_end(run, 15) or dict(result="no end record")
        else:
            end = self.watch.wait_end(run, E02_TIMEOUT_S[0], poll=lambda: self.watchdog(robot))
            if end is None:
                self.send(robot, "stop")
                end = self.watch.wait_end(run, 5) or dict(result="no end record")
        self.kc_stop.set()
        self.moves.pop(robot, None)
        e02_end = self.listener.latest("E02", robot, t_e0, run=run)
        eth = num(e02_end[1].get("end_th")) if e02_end else None
        # where it faces now: its own belief at the end (a stopped route may face anywhere); the next leg
        # measures it again
        self.heads[robot] = [eth if eth is not None else nominal, False]
        t_e1 = time.time()
        time.sleep(0.5)
        endp = self.settle(t_from=time.time() - 0.8, min_s=0.5)
        self.mark("e02 %s route end %s" % (tag, end["result"]))
        r["e02_result"] = end["result"]
        r["battery"] = end.get("battery")
        # 4. what the camera measured: start, far point and end, at the moments the robot logged them
        e02 = self.listener.latest("E02", robot, t_e0, run=run)
        rec = e02[1] if e02 else {}
        raw = self.listener.raw.get(run) or []
        off = FIT.clock_offset(raw)

        def lap(ms_key):
            ms = num(rec.get(ms_key))
            return None if ms is None or off is None else ms / 1000.0 + off
        r["t_find_l"], r["t_e02_l"], r["t_end_l"] = round(t_run0, 3), round(t_e0, 3), round(t_e1, 3)
        for k in ("ms_start", "ms_far", "ms_end"):
            if lap(k) is not None:
                r["t_%s_l" % k[3:]] = round(lap(k), 3)
        if k_info:
            for k in ("t_start", "t_far"):
                if k_info.get(k):
                    r[k + "_l"] = round(k_info[k], 3)

        def cam_at(ms_key, span):
            t = lap(ms_key)
            if t is None:
                return None
            smp = self.history.since(t + 0.05, t + 0.05 + span)
            if not smp:
                return None
            return (float(np.median([q[1] for q in smp])), float(np.median([q[2] for q in smp])))
        start = (k_info or {}).get("start") or cam_at("ms_start", 0.4) or pre
        far = (k_info or {}).get("far") or cam_at("ms_far", 0.8 * PAUSE_MS / 1000.0)
        bs = [num(rec.get(k)) for k in ("start_x", "start_y", "far_goal_x", "far_goal_y")]
        if k_info and k_info.get("far_goal"):
            goal = k_info["far_goal"]
        elif None not in bs:
            goal = (start[0] + bs[2] - bs[0], start[1] + bs[3] - bs[1])     # the robot's own route, from the camera start
        else:
            out_w, _ = self.board.route_floor(route, start, nominal)
            goal = out_w[-1]
            r.setdefault("flags", []).append("goal_from_nominal_route")
        if far is None:
            far = self.longest_still(t_e0, t_e1, goal)
            if far:
                r.setdefault("flags", []).append("far_from_stillness")
        r["cam_start"] = [round(start[0], 2), round(start[1], 2)] if start else None
        r["intended_far"] = [round(goal[0], 2), round(goal[1], 2)]
        r["cam_far"] = [round(far[0], 2), round(far[1], 2)] if far else None
        r["cam_end"] = [round(endp[0], 2), round(endp[1], 2)] if endp else None
        far_err = math.hypot(far[0] - goal[0], far[1] - goal[1]) if far else None
        end_err = math.hypot(endp[0] - start[0], endp[1] - start[1]) if (endp and start) else None
        r["far_err_cm"] = round(far_err, 2) if far_err is not None else None
        r["end_err_cm"] = round(end_err, 2) if end_err is not None else None
        bel = None
        sx, sy, ex, ey = (num(rec.get(k)) for k in ("start_x", "start_y", "end_x", "end_y"))
        if None not in (sx, sy, ex, ey):
            bel = math.hypot(ex - sx, ey - sy)
        r["fixes_used"] = num(rec.get("fixes_used"))
        r["robot_thought_end_cm"] = round(bel, 2) if bel is not None else None
        if k_info:
            r["legs"] = k_info["legs"]
        on_board = endp is not None and self.board.on_board(endp[0], endp[1], margin=-1.0)
        verdict, why = ST.judge_run(far_err, end_err, on_board=on_board, seen=endp is not None,
                                    recover_cm=self.a.recover_cm,
                                    runend="watchdog stop" if self.watchdog_fired else end["result"])
        r["recovery"] = verdict
        r["verdict"] = verdict or "ok"
        if end["result"] != "done":
            r.setdefault("flags", []).append("runend:" + end["result"])
        self.last_result = "%s %s %s: far point %s, back at start %s%s" % (
            robot, cond, PL.ROUTE_SHORT[route], "%.1f cm off" % far_err if far_err is not None else "not seen",
            "%.1f cm off" % end_err if end_err is not None else "not seen",
            " (robot thought %.1f cm)" % bel if bel is not None else "")
        self.say("  Result: " + self.last_result)
        if verdict:
            r.setdefault("flags", []).append("recovery:" + verdict)
            self.say("  Needed recovery: %s (noted; kept in the data, not an outlier)." % why)
            self.mark("e02 %s recovery %s" % (tag, verdict))
        # 4. learn (U runs)
        if cond == "U":           # any U run, finished or stopped: the raw gyro and wheel counts are still true
            try:
                self.learn(robot, r, t_run0, t_e1 + 0.5, [x for x in (fnd["run"], run) if x])
            except Exception as e:      # a fit problem must never stop the session
                self.say("  (self-calibration skipped this run: %s)" % e)
        # 5. park (this also drives a lost robot back)
        for c in ("set e02_route_gyro_scale -999", "set e02_route_gyro_drift_dps -999", "set e02_route_wheel_scale -999"):
            self.send(robot, c)
        self.set_corrections(robot, self.move_corrections(robot))
        r["status"] = "done"
        r["t_end"] = round(time.time(), 2)
        st.d.setdefault("durations", []).append(round(r["t_end"] - r["t_start"], 1))
        bat = num(r.get("battery"))
        if bat is not None and bat < BATTERY_STOP:
            self.say("  %s's battery is at %d%%: its remaining runs are put aside (charge it, then --resume)." % (robot, bat))
            self.unavailable.add(robot)
        st.save()
        self.write_runs()
        if not self.recover(robot, why if verdict else None):
            self.say("  %s could not be parked; its remaining runs are put aside." % robot)
        self.active = None

    def longest_still(self, t0, t1, near):
        """The longest still period between t0 and t1 (ties: nearest to `near`): fallback far point."""
        smp = self.history.since(t0, t1)
        best = None
        i = 0
        while i < len(smp):
            j = i
            while j + 1 < len(smp) and math.hypot(smp[j + 1][1] - smp[i][1], smp[j + 1][2] - smp[i][2]) < 0.6:
                j += 1
            dur = smp[j][0] - smp[i][0]
            if dur >= 0.6:
                x = float(np.median([s[1] for s in smp[i:j + 1]]))
                y = float(np.median([s[2] for s in smp[i:j + 1]]))
                key = (round(dur, 1), -math.hypot(x - near[0], y - near[1]))
                if best is None or key > best[0]:
                    best = (key, (x, y))
            i = j + 1
        return best[1] if best else None

    def recover(self, robot, why=None):
        """Park the robot by camera (after a lost run: 'driven back'). Nick only if that fails."""
        if why:
            self.say("  Driving %s back by camera (%s) ..." % (robot, why))
        ok, words = self.park(robot)
        if ok:
            return True
        self.say("  Parking %s failed: %s" % (robot, words))
        p = self.pos(robot)
        if p is None:
            # look for it by a turn on its spot, near where it was last seen
            self.say("  Looking for %s by a short turn on its spot ..." % robot)
            last = self.tracker.where() or (None, *(self.st["positions"].get(robot) or [None, None])[:2])
            near = (last[1], last[2], 60.0) if last and last[1] is not None else None
            q = self.acquire(robot, step_cm=8.0, near=near)
            if q:
                ok, words = self.park(robot)
                if ok:
                    return True
        return self.help_from_nick(robot, words)

    def finish_failed(self, r, robot, words):
        r["status"] = "failed"
        r.setdefault("notes", []).append(words)
        r["t_end"] = round(time.time(), 2)
        self.last_result = "%s %s %s: %s" % (robot, r["cond"], PL.ROUTE_SHORT[r["route"]], words)
        self.st.save()
        self.write_runs()
        self.send(robot, "stop")
        time.sleep(0.5)
        self.set_corrections(robot, self.move_corrections(robot))
        if not self.recover(robot, words):
            self.say("  %s could not be parked; its remaining runs are put aside." % robot)

    # --- K and KC -----------------------------------------------------------------------------------
    def route_points(self, route, prm, start, heading):
        """The firmware's own route (fw_e02 / e02.h e_waypoints) from its PARAM values: (waypoints on the floor
        from `start` along `heading`, index of the far point)."""
        if route == 1:
            L = prm.get("e02_straight_cm") or PL.STRAIGHT_CM
            w, far = [(L, 0.0), (0.0, 0.0)], 0
        elif route == 2:
            a = prm.get("e02_l_cm") or PL.L_CM
            t = 1.0 if (prm.get("e02_l_turn") if prm.get("e02_l_turn") is not None else 1) >= 0 else -1.0
            w, far = [(a, 0.0), (a, t * a), (a, 0.0), (0.0, 0.0)], 1
        else:
            q = prm.get("e02_square_cm") or PL.SQUARE_CM
            w, far = [(q, 0.0), (q, q), (0.0, q), (0.0, 0.0)], 1
        return PL.rot(w, start[0], start[1], heading), far

    def k_drive(self, robot, run, route, prm, heading):
        """Condition K: the laptop steers it by camera, leg by leg (the robot's own pose is not used): to the
        route's start spot (transit), `mark start`, out to the far point, `mark far` (it stands still there),
        back along the route to the start, turned to the start heading, `mark end`. The same route and start
        spot as the robot drives itself in U / C / KC."""
        info = dict(legs=0, far=None, start=None, far_goal=None)
        self.wait_mode1(robot, time.time() - 10.0)
        self.moves[robot] = dict(run=run, why="K")
        self.heads[robot] = [heading if self.heads.get(robot, [None])[0] is None else self.heads[robot][0], False]
        sx, sy, _ = self.board.nominal_start()
        ok, why = self.drive_to(robot, (sx, sy), final_heading=heading)
        if not ok:
            self.say("  K drive: %s" % why)
            self.mark_leg(robot, "end")
            return info
        start = self.settle(min_s=0.5)
        info["start"] = start
        info["t_start"] = time.time()
        self.mark_leg(robot, "start")
        pts, far_i = self.route_points(route, prm, start, heading)
        info["far_goal"] = pts[far_i]
        for k, w in enumerate(pts):
            last = k == len(pts) - 1
            ok, why = self.drive_to(robot, w, final_heading=heading if last else None, run=run,
                                    arrive_cm=ST.ARRIVE_CM if (last or k == far_i) else 2.0)
            if not ok:
                self.say("  K drive: %s" % why)
                break
            if k == far_i:
                t = time.time()
                info["t_far"] = t
                self.mark_leg(robot, "far")                 # it stands still e02_pause_ms, then answers
                smp = self.history.since(t, time.time())
                if smp:
                    info["far"] = (float(np.median([q[1] for q in smp])), float(np.median([q[2] for q in smp])))
        self.mark_leg(robot, "end")
        self.moves.pop(robot, None)
        with self.listener.cv:
            info["legs"] = sum(1 for t, n, rec in self.listener.recs["LEG"]
                               if n == robot and rec.get("run") == run and rec.get("state") == "done")
        return info

    def kc_fixes(self, robot, r, run):
        """Condition KC: camera fixes to the robot, --fix-hz a second, while its run lasts. The firmware takes a
        fix only while its wheels have stood still e02_fix_still_ms (it stands e02_leg_pause_ms after every leg).
        The heading goes with a fix only when the camera saw a straight stretch end just before and the gyro
        (RAW yaw) says it has not turned since; otherwise position only ("-")."""
        n = nh = 0
        period = 1.0 / self.a.fix_hz
        while not self.kc_stop.wait(period):
            recent = self.history.since(time.time() - 3.0)
            fx = kc_fix_from(recent)
            if fx is None:
                continue                          # moving, or not still long enough: no picture of where it is now
            x, y, t_pic, t_stop, seg = fx
            th = "-"
            h = ST.heading_from_track(seg, min_cm=3.0, max_rms_cm=0.4) if seg else None
            turned = self.gyro_turn_since(run, t_stop) if h is not None else None
            if h is not None and turned is not None and abs(turned) < 2.0:
                th = "%.1f" % ST.wrap180(h + turned)
                nh += 1
            self.send(robot, "fix %.2f %.2f %s %.3f" % (x, y, th, t_pic))
            n += 1
        r["fixes_sent"] = n
        r["fixes_with_heading"] = nh

    def gyro_turn_since(self, run, t):
        """How far the robot turned (deg, left +, raw gyro) since laptop time t, from its RAW rows, or None."""
        raw = self.listener.raw.get(run) or []
        if len(raw) < 3:
            return None
        off = FIT.clock_offset(raw[-400:])
        info = self.listener.runinfo.get(run) or {}
        sign = -1.0 if (num(info.get("yaw_sign")) or 1) < 0 else 1.0
        rows = [q for q in raw[-400:] if q["ms"] is not None and q["yaw_deg"] is not None]
        before = [q for q in rows if q["ms"] / 1000.0 + off <= t]
        if not before or not rows:
            return None
        d = ST.wrap180(rows[-1]["yaw_deg"] - before[-1]["yaw_deg"])
        return sign * d

    # --- the dance (Nick, 2026-10-22; e02_dance.py) ------------------------------------------------------
    def dance(self):
        """The 3-robot dance set to a beat: start poses, a count-in, every move on its beat with camera fixes
        between moves and a supervisor that stops all robots if two get too close or one is lost; then parking.
        Returns the result dict (dance_result.json), or None."""
        import e02_dance as DN
        return DN.DanceRun(self, bpm=getattr(self.a, "dance_bpm", 120.0) or 120.0).run()

    # --- files ------------------------------------------------------------------------------------
    RUN_COLS = ["order", "block", "robot", "cond", "route", "status", "recovery", "flags", "find_run", "e02_run",
                "find_result", "e02_result", "t_start", "t_end", "cam_start_x", "cam_start_y", "cam_far_x", "cam_far_y",
                "cam_end_x", "cam_end_y", "intended_far_x", "intended_far_y", "far_err_cm", "end_err_cm",
                "robot_thought_end_cm", "corr_source", "gyro_scale", "gyro_drift_dps", "wheel_scale", "battery",
                "fixes_sent", "fixes_used", "legs", "fit_obs", "notes"]

    def write_runs(self):
        rows = []
        for r in self.st["schedule"]:
            c = r.get("corr") or {}

            def xy(k, i):
                v = r.get(k)
                return v[i] if v else ""
            rows.append(dict(order=r["order"], block=r["block"], robot=r["robot"], cond=r["cond"], route=r["route"],
                             status=r["status"], recovery=r.get("recovery", ""), flags=" ".join(r.get("flags", [])),
                             find_run=r.get("find_run") or "", e02_run=r.get("e02_run") or "",
                             find_result=r.get("find_result") or "", e02_result=r.get("e02_result") or "",
                             t_start=r.get("t_start", ""), t_end=r.get("t_end", ""),
                             cam_start_x=xy("cam_start", 0), cam_start_y=xy("cam_start", 1),
                             cam_far_x=xy("cam_far", 0), cam_far_y=xy("cam_far", 1),
                             cam_end_x=xy("cam_end", 0), cam_end_y=xy("cam_end", 1),
                             intended_far_x=xy("intended_far", 0), intended_far_y=xy("intended_far", 1),
                             far_err_cm=r.get("far_err_cm", ""), end_err_cm=r.get("end_err_cm", ""),
                             robot_thought_end_cm=r.get("robot_thought_end_cm", ""), corr_source=r.get("corr_source", ""),
                             gyro_scale=c.get("gyro_scale", ""), gyro_drift_dps=c.get("gyro_drift_dps", ""),
                             wheel_scale=c.get("wheel_scale", ""), battery=r.get("battery", ""),
                             fixes_sent=r.get("fixes_sent", ""), fixes_used=r.get("fixes_used", ""),
                             legs=r.get("legs", ""), fit_obs=r.get("fit_obs", ""), notes=" | ".join(r.get("notes", []))))
        write_csv(os.path.join(self.dir, "runs.csv"), self.RUN_COLS, rows)
        try:
            self.write_analysis_runs()
        except Exception as e:      # never stop a session over a side file
            self.say("  (e02_runs.csv not written: %s)" % e)

    def write_analysis_runs(self):
        """e02_runs.csv for tools/experiment/e02_analysis.py (lane A's --runs): its column names, times on the
        camera's (Pi's) clock: Pi time = laptop time - (laptop - Pi clock offset) + capture delay."""
        off = self.feed.clock.offset if self.feed is not None and self.feed.clock.offset is not None else None
        if off is not None:
            self.st["setup"]["pi_offset_s"] = round(off - self.feed.cap, 4)
        po = self.st["setup"].get("pi_offset_s")

        def pi(t):
            return "" if (t is None or po is None) else "%.3f" % (t - po)
        rows = []
        for r in self.st["schedule"]:
            if r["status"] not in ("done", "failed"):
                continue
            rows.append(dict(run_id=r.get("e02_run") or "", robot=r["robot"], condition=r["cond"], route=r["route"],
                             block=r["block"], robot_run=r.get("e02_run") or "", find_run=r.get("find_run") or "",
                             order=r["order"], recovery=r.get("recovery") or "",
                             t_find=pi(r.get("t_find_l")), t_start=pi(r.get("t_start_l") or r.get("t_e02_l")),
                             t_far=pi(r.get("t_far_l")), t_end=pi(r.get("t_end_l")), status=r["status"],
                             flags=" ".join(r.get("flags", []))))
        write_csv(os.path.join(self.dir, "e02_runs.csv"), ["run_id", "robot", "condition", "route", "block", "robot_run",
                                                          "find_run", "order", "recovery", "t_find", "t_start", "t_far",
                                                          "t_end", "status", "flags"], rows)

    def write_fits(self):
        cols = ["robot", "t", "order", "accepted", "gyro_ok", "wheel_ok", "n_new", "n_total", "fit_gyro_scale",
                "fit_gyro_drift_dps", "fit_wheel_scale", "se_gyro_scale", "se_gyro_drift_dps", "se_wheel_scale",
                "rms_deg", "rms_cm", "turns", "used_gyro_scale", "used_gyro_drift_dps", "used_wheel_scale", "reasons"]
        rows = []
        for robot, f in self.st["fits"].items():
            for h in f.get("history", []):
                fi, v = h.get("fitted") or {}, h.get("values") or {}
                rows.append(dict(robot=robot, t=h["t"], order=h["order"], accepted=h["accepted"], gyro_ok=h["gyro_ok"],
                                 wheel_ok=h["wheel_ok"], n_new=h["n_new"], n_total=h["n_total"],
                                 fit_gyro_scale=_r(fi.get("gyro_scale")), fit_gyro_drift_dps=_r(fi.get("gyro_drift_dps")),
                                 fit_wheel_scale=_r(fi.get("wheel_scale")), se_gyro_scale=_r(fi.get("se_gyro_scale")),
                                 se_gyro_drift_dps=_r(fi.get("se_gyro_drift_dps")),
                                 se_wheel_scale=_r(fi.get("se_wheel_scale")), rms_deg=_r(fi.get("rms_deg")),
                                 rms_cm=_r(fi.get("rms_cm")), turns=fi.get("turns", ""),
                                 used_gyro_scale=v.get("gyro_scale", ""), used_gyro_drift_dps=v.get("gyro_drift_dps", ""),
                                 used_wheel_scale=v.get("wheel_scale", ""), reasons=" | ".join(h.get("reasons", []))))
        write_csv(os.path.join(self.dir, "fits.csv"), cols, rows)

    # --- the session --------------------------------------------------------------------------------
    def session(self):
        st = self.st
        if not st["schedule"]:
            sched = self.planned_schedule()
            if st["setup"].get("square_left_out"):
                sched = [r for r in sched if r["route"] != 3]
                for i, r in enumerate(sched):
                    r["order"] = i + 1
            st["schedule"] = sched
            st.save()
            for line in PL.plan_words(sched, self.a.run_s, self.a.find_s, 0):
                self.say(line)
        self.write_runs()
        st["t_session_start"] = time.time()
        self.mark("e02 session start")
        n_this = 0
        cur_block = None
        while True:
            r = PL.next_run(st["schedule"], lambda rb: self.fitted(rb) is not None, self.unavailable)
            if r is None:
                break
            if self.a.max_runs and n_this >= self.a.max_runs:
                self.say("Stopping after %d runs (--max-runs)." % n_this)
                break
            if r["block"] != cur_block:
                if cur_block is not None:
                    el = time.time() - st["t_session_start"] + st.get("elapsed_before", 0)
                    need = st.mean_run_s() * sum(1 for x in st["schedule"] if x["block"] == r["block"])
                    if not self.a.dry_run and el + need > self.a.hours * 3600 + 15 * 60:
                        self.say("Time is up for whole blocks (%s in; the next block needs about %s). The remaining "
                                 "blocks stay in the plan: --resume runs them later." % (hms(el), hms(need)))
                        break
                cur_block = r["block"]
                self.say("--- Block %d ---" % cur_block)
                self.recorder_ok()
                self.boards_still()
            elif self.marks_unconfirmed >= 3:
                self.recorder_ok()
            self.do_run(r)
            n_this += 1
        done = [r for r in st["schedule"] if r["status"] == "done"]
        self.mark("e02 session end")
        self.say("=== Session: %d runs done, %d failed, %d still to do ===" % (
            len(done), sum(1 for r in st["schedule"] if r["status"] == "failed"),
            sum(1 for r in st["schedule"] if r["status"] == "pending")))
        for cond in PL.CONDITIONS:
            e = [r["end_err_cm"] for r in done if r["cond"] == cond and r.get("end_err_cm") is not None]
            f = [r["far_err_cm"] for r in done if r["cond"] == cond and r.get("far_err_cm") is not None]
            if e:
                self.say("  %-2s (%s): %d runs, end error mean %.1f cm, far point mean %.1f cm (camera; the analysis "
                         "does the statistics)" % (cond, PL.COND_WORDS[cond], len(e), float(np.mean(e)),
                                                  float(np.mean(f)) if f else float("nan")))
        for robot in st["robots"]:
            self.say("  %s: %s" % (robot, FIT.describe(self.fitted(robot))))
        st["elapsed_before"] = st.get("elapsed_before", 0) + time.time() - st["t_session_start"]
        st.save()


def stream_url(a):
    """The overhead camera's live stream; --stream-q lowers its JPEG quality (fewer bytes over WiFi, more pictures a
    second: 2026-10-22, the laptop got only ~2 a second at the default quality 95)."""
    q = getattr(a, "stream_q", 0)
    return "%s:%d/live%s" % (a.pi, a.overhead_port, ("?q=%d" % q) if q else "")


def kc_fix_from(samples, after_stop_s=0.1, min_still_s=0.3, moved_cm=0.8):
    """A KC camera fix only from pictures taken after the robot stopped: (x, y, t of the newest picture, t_stop,
    the stretch before the stop) or None. t_stop = the last picture that was more than moved_cm from where it
    stands now; the position = the median of the pictures from t_stop + after_stop_s on, at least min_still_s of
    them (so the firmware's own still check, e02_fix_still_ms, and the picture agree)."""
    if not samples:
        return None
    sp = CAM.still_position(samples, min_s=min_still_s)
    if sp is None:
        return None
    moving = [q for q in samples if math.hypot(q[1] - sp[0], q[2] - sp[1]) > moved_cm]
    t_stop = moving[-1][0] if moving else samples[0][0]
    if not moving and samples[-1][0] - samples[0][0] < 2.5:
        return None                               # it may have stopped just before this window: not sure when
    still = [q for q in samples if q[0] >= t_stop + after_stop_s]
    if len(still) < 3 or still[-1][0] - still[0][0] < min_still_s:
        return None
    x = float(np.median([q[1] for q in still]))
    y = float(np.median([q[2] for q in still]))
    seg = [q for q in samples if t_stop - 1.0 <= q[0] <= t_stop] if moving else []
    return x, y, still[-1][0], t_stop, seg


def _r(v):
    return "" if v is None else round(v, 5)


def write_csv(path, cols, rows):
    tmp = path + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    try:
        os.replace(tmp, path)
    except OSError:     # open in Excel: keep the copy next to it
        pass


def check_params(watch, run, cmds, wait_s=2.0):
    """run_sheet.check_params, with `corr reset` understood."""
    import run_sheet
    return run_sheet.check_params(watch, run, cmds, wait_s)


class NamingWatch:
    """tools/live's blink tracker on the overhead frames during the naming check (record only)."""

    def __init__(self, cam, folder):
        for p in (os.path.join(TOOLS, "live"), os.path.join(TOOLS, "blink"), os.path.join(TOOLS, "fusion")):
            if p not in sys.path:
                sys.path.insert(0, p)
        import track_live as TL
        calib = CAM.write_floor_calibration(cam, os.path.join(folder, "floor_calibration_for_blink.json"))
        ns = argparse.Namespace(led_height_cm=None, names=None, vmax_cms=40.0, exposure_ms=None, replay=None,
                                name_p=0.9, no_fast=False, calib_results=None, led_offset_cm=None)
        import floormap
        ns.led_height_cm = floormap.LED_HEIGHT_CM
        self.tracker, self.fmap = TL.make_tracker(ns, calib)
        self.fixes = defaultdict(list)
        self.lock = threading.Lock()
        self.out = open(os.path.join(folder, "naming_check_fixes.jsonl"), "w")

    def push(self, img, t):
        try:
            msgs = self.tracker.push(img, t)
        except Exception:
            return
        with self.lock:
            for m in msgs:
                if m.get("type") == "fix":
                    self.fixes[m["robot"]].append((m["x"], m["y"]))
                    self.out.write(json.dumps(m) + "\n")

    def summary(self, known):
        out = {}
        with self.lock:
            for n, kp in known.items():
                fx = self.fixes.get(n, [])
                near = [f for f in fx if kp and math.hypot(f[0] - kp[0], f[1] - kp[1]) <= 8.0]
                wrong = len(fx) - len(near) if kp else None
                words = ("named %d times, %d of them at its spot" % (len(fx), len(near))) if fx else "never named"
                out[n] = dict(fixes=len(fx), at_its_spot=len(near), elsewhere=wrong, words=words)
            others = {k: len(v) for k, v in self.fixes.items() if k not in known}
        self.out.close()
        return dict(robots=out, other_names=others)


# =============================================================================== main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pi", default="st1maxfini.local", help="the Pi recorder's address")
    ap.add_argument("--overhead-port", type=int, default=8000)
    ap.add_argument("--side-port", type=int, default=8001)
    ap.add_argument("--stream-q", type=int, default=0, help="JPEG quality of the live camera stream (0 = the Pi's "
                                                              "default, 95; e.g. 70 for more pictures a second)")
    ap.add_argument("--front-learn", action="store_true",
                    help="learn the front sensor's offset and spread against the map in this session (from scratch); "
                         "off: the values from data (FRONT_OFFSET_CM, FRONT_TOL_CM)")
    ap.add_argument("--no-side", action="store_true", help="no side camera this session: don't check or ask about it")
    ap.add_argument("--map", action="store_true", help="after setup, one robot maps the board by touch (about 6 min)")
    ap.add_argument("--map-only", action="store_true", help="setup and the mapping only, then stop")
    ap.add_argument("--bricks", action="store_true", help="after the session: the brick demo (you put bricks or any "
                                                         "objects down, a robot crosses round them three times: factory "
                                                         "settings, then its calibration, then with camera fixes)")
    ap.add_argument("--bricks-only", action="store_true", help="setup (and the naming check: the gyro creep), then the "
                                                              "brick demo only")
    ap.add_argument("--tight-gap", action="store_true", help="the brick demo (after the session, or with "
                                                             "--bricks-only) goes straight through the narrowest gap "
                                                             "between two objects the robot can safely make, from its "
                                                             "measured precision this session")
    ap.add_argument("--gap-k", type=float, default=None, help="tight gap: room = half the robot + k x the 95th "
                                                              "percentile of its measured error (default 1.0)")
    ap.add_argument("--dry-bricks", default="", help="dry run: the bricks the simulator puts down when asked, "
                                                     "'x,y,length,width[,deg];...' cm")
    ap.add_argument("--dance", action="store_true", help="after the session: the 3-robot dance to a beat (about 1.5 "
                                                        "min; watch and listen on http://localhost:8090/dance)")
    ap.add_argument("--dance-only", action="store_true", help="setup (and the naming check), then the dance only")
    ap.add_argument("--dance-bpm", type=float, default=120.0, help="the dance's tempo (120: the lights flash on "
                                                                   "every beat)")
    ap.add_argument("--view-port", type=int, default=8090, help="live view in a browser: http://localhost:<port> (0 = off)")
    ap.add_argument("--map-rays", type=int, default=16)
    ap.add_argument("--map-robot", default="", help="which robot maps (default: the first)")
    ap.add_argument("--dry-start", default="parks", choices=["parks", "swapped", "scatter", "corners"],
                    help="dry run: where the simulated robots stand at the start")
    ap.add_argument("--calib", help="floor calibration file (default: the studio project's cameras/overhead.json)")
    ap.add_argument("--project", help="the studio project folder (default Documents\\recognition-project)")
    ap.add_argument("--make-calibration", action="store_true", help="make one from the board corners even if one exists")
    ap.add_argument("--lens-height-cm", type=float, help="camera lens height above the floor, if measured")
    ap.add_argument("--board-sides", help="which edges are blocked, as x-board side,y-board side (0 = low, 1 = high "
                                          "end of the other axis; e.g. 0,0 = boards along y = 0 and x = 0). Default: "
                                          "asked, with the camera picture")
    ap.add_argument("--robots", default="Left,Center,Right")
    ap.add_argument("--robot", action="append", default=[], help="robot IP, if robots aren't found automatically")
    ap.add_argument("--hours", type=float, default=2.0)
    ap.add_argument("--run-s", type=float, default=PL.RUN_S, help="time a route run with its parking takes, for the "
                                                                   "plan (s; measured ~70)")
    ap.add_argument("--find-s", type=float, default=PL.FIND_S, help="time a find takes, for the plan (s)")
    ap.add_argument("--help-wait-min", type=float, default=10.0, help="how long to wait for Nick before putting a "
                                                                       "robot aside (minutes)")
    ap.add_argument("--blocks", type=int, help="regular blocks after the all-U block (default: what fits in --hours)")
    ap.add_argument("--routes", default="1,2,3", help="only these routes (testing)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--fix-hz", type=float, default=FIX_HZ)
    ap.add_argument("--recover-cm", "--lost-cm", dest="recover_cm", type=float, default=ST.RECOVER_CM,
                    help="a run that ends this far from its start spot needed recovery (noted, not an outlier)")
    ap.add_argument("--naming-s", type=int, default=120)
    ap.add_argument("--skip-naming", action="store_true")
    ap.add_argument("--skip-find-check", action="store_true")
    ap.add_argument("--max-runs", type=int, default=0, help="stop after this many runs (0 = all)")
    ap.add_argument("--resume", nargs="?", const="latest", help="carry on a session (folder, or the latest)")
    ap.add_argument("--auto", action="store_true", help="answer every question with its default (no person)")
    ap.add_argument("--dry-run", action="store_true", help="against the simulator (tools/experiment/e02_sim.py)")
    ap.add_argument("--dry-true-calib", action="store_true", help="dry run: use the simulator's true camera "
                                                                  "instead of making one from the corners")
    a = ap.parse_args(argv)

    sim = None
    if not a.resume and not a.dry_run and not a.auto:
        # an unfinished real session on this laptop: offer to carry it on (the launcher double-click path)
        for c in sorted(glob.glob(os.path.join(REPO, "logs", "e02-*", "e02_state.json")), reverse=True):
            if "-dryrun" in c:
                continue
            try:
                left = sum(1 for r in json.load(open(c)).get("schedule", []) if r.get("status") in ("pending", "running"))
            except (OSError, ValueError):
                continue
            if left:
                ans = input("An unfinished E02 session is on this laptop (%s, %d runs to go). Carry it on? "
                            "Type y and press Enter to carry on, or n for a new session: " % (os.path.dirname(c), left))
                if ans.strip().lower().startswith("y"):
                    a.resume = os.path.dirname(c)
            break
    if a.resume:
        if a.resume == "latest":
            cands = sorted(c for c in glob.glob(os.path.join(REPO, "logs", "e02-*", "e02_state.json"))
                           if a.dry_run == ("-dryrun" in os.path.basename(os.path.dirname(c))))
            if not cands:
                sys.exit("No E02 session to resume in logs/.")
            sdir = os.path.dirname(cands[-1])
        else:
            sdir = a.resume
        state = PL.State.load(sdir)
        print("Resuming %s" % sdir)
        a.dry_run = a.dry_run or state["setup"].get("dry_run", False)
    else:
        sdir = os.path.join(REPO, "logs", "e02-" + dt.datetime.now().strftime("%Y%m%d-%H%M%S")
                            + ("-dryrun" if a.dry_run else ""))
        os.makedirs(sdir, exist_ok=True)
        state = PL.State(sdir)
        state["robots"] = [n.strip() for n in a.robots.split(",") if n.strip()]
        state["seed"] = a.seed if a.seed is not None else int(time.time()) % 100000
        state["setup"]["dry_run"] = a.dry_run
        state["setup"]["args"] = vars(a)
        state.save()
    if a.dry_run:
        a.auto = True
        a.pi = "127.0.0.1"
        a.robot = a.robot or ["127.0.0.%d" % (2 + k) for k in range(len(state["robots"]))]
        if a.board_sides is None:
            a.board_sides = "1,1"          # Nick's layout: boards along x = max and y = max (far corner)
        sim = start_sim(state, sdir)
        if a.dry_true_calib and not state.get("calibration"):
            a.calib = os.path.join(sdir, "sim", "camera_model.json")
        if not a.calib and not state.get("calibration"):
            a.make_calibration = True

    try:
        return launch(a, state, sdir, sim)
    finally:
        stop_sim(sim, sdir)      # never leave the simulator running


def launch(a, state, sdir, sim):
    state["launches"] = state.get("launches", 0) + 1
    logdir = os.path.join(sdir, "robot-logs-%d" % state["launches"])
    state.setdefault("log_dirs", []).append(logdir)
    redo = state.interrupted()
    state.save()

    import alvik_console as AC
    import run_sheet
    run = Runner(a, state, sdir)
    run.say("E02 session folder: %s (launch %d)" % (sdir, state["launches"]))
    for r in redo:
        run.say("Run %d was interrupted last time: it will be redone." % r["order"])
    link = AC.Link(logdir, a.robot)
    link.session.quiet = True
    run.link = link
    run.listener = Listener(link.session)
    run.watch = run_sheet.RunWatcher()
    link.session.listeners += [run.watch, run.listener]
    run.pi = run_sheet.PiNotes(a.pi)
    run.view = None
    try:
        import e02_live
        run.lab = e02_live.LiveLab()
        run.listener.lab = run.lab
    except Exception as e:
        run.lab = None
        run.say("(live lab not started: %s)" % e, quiet=True)
    if a.view_port:
        try:
            import e02_view
            run.view = e02_view.LiveView(run, a.view_port)
            run.say(run.view.start())
        except Exception as e:          # the view is a convenience: never stop a session over it
            run.say("(live view not started: %s)" % e)
            run.view = None
    hist = CAM.History()
    run.history = hist
    if not state["schedule"] and not getattr(a, "bricks_only", False) and not getattr(a, "dance_only", False):
        for line in PL.plan_words(run.planned_schedule(), a.run_s, a.find_s):
            run.say(line)
    threading.Thread(target=run.watchdog_loop, daemon=True).start()
    code = 0
    feed = None
    try:
        feed = CAM.Feed(stream_url(a), None, hist, log=lambda m: run.say("  (camera: %s)" % m))
        feed.on_sample = run.on_sample
        feed.start()
        run.feed = feed
        run.setup()
        run.robots_ready()
        if (a.map or a.map_only) and not state["setup"].get("map"):
            names = [n for n in state["robots"] if n not in run.unavailable]
            if names:
                run.map_board(a.map_robot if a.map_robot in names else names[0], n_rays=a.map_rays)
        if a.map_only:
            raise SystemExit("Mapping done (--map-only): no runs this time.")
        if getattr(a, "dance_only", False):
            if not a.skip_naming and not state.get("naming_check"):
                run.naming_check()
            run.dance()
        elif getattr(a, "bricks_only", False):
            if not a.skip_naming and not state.get("naming_check"):
                run.naming_check()
            run.brick_demo()
        else:
            if not a.skip_find_check and not state["setup"].get("find_check"):
                run.find_check()
            if not a.skip_naming and not state.get("naming_check"):
                run.naming_check()
            run.session()
            if getattr(a, "bricks", False) or getattr(a, "tight_gap", False):
                run.brick_demo()
            if getattr(a, "dance", False):
                run.dance()
    except KeyboardInterrupt:
        run.say("Stopped by Ctrl+C: every robot is stopped. Start again with --resume to carry on.")
        code = 1
    except SystemExit as e:
        run.say(str(e))
        code = 1
    finally:
        run.watch_quit.set()
        try:
            link.close()
        except Exception:
            pass
        if feed:
            feed.close()
        state.save()
        run.trackf.close()
        if sim is not None:
            stop_sim(sim, sdir)
            try:
                rep = dry_run_report(sdir)
                run.say("Dry-run check against the simulator's truth: " + json.dumps(rep["summary"]))
            except Exception as e:
                run.say("(dry-run report failed: %s)" % e)
        run.say("Saved in %s" % sdir)
    return code


def start_sim(state, sdir):
    simdir = os.path.join(sdir, "sim")
    os.makedirs(simdir, exist_ok=True)
    for f in ("STOP", "sim_info.json", "BRICKS_ON"):
        if os.path.exists(os.path.join(simdir, f)):
            os.remove(os.path.join(simdir, f))
    sides = (state["setup"].get("args") or {}).get("board_sides") or "1,1"
    sim = subprocess.Popen([sys.executable, os.path.join(HERE, "e02_sim.py"), "--out", simdir,
                            "--xboard-side", sides.split(",")[0], "--yboard-side", sides.split(",")[1],
                            "--robots", ",".join(state["robots"]), "--seed", str(state["seed"] % 1000 + 1),
                            "--start", (state["setup"].get("args") or {}).get("dry_start") or "parks"]
                           + (["--bricks", (state["setup"].get("args") or {}).get("dry_bricks")]
                              if (state["setup"].get("args") or {}).get("dry_bricks") else []),
                           stdout=open(os.path.join(simdir, "sim_stdout.txt"), "a"), stderr=subprocess.STDOUT)
    t0 = time.time()
    while time.time() - t0 < 40 and not os.path.exists(os.path.join(simdir, "sim_info.json")):
        if sim.poll() is not None:
            raise SystemExit("The simulator stopped at once: see %s" % os.path.join(simdir, "sim_stdout.txt"))
        time.sleep(0.3)
    time.sleep(1.0)
    return sim


def stop_sim(sim, sdir):
    if sim is None:
        return
    open(os.path.join(sdir, "sim", "STOP"), "w").close()
    try:
        sim.wait(timeout=10)
    except subprocess.TimeoutExpired:
        sim.kill()


def dry_run_report(sdir):
    """Score a dry run against the simulator's truth: the camera tracker's error, the learned corrections
    against the robots' true errors, and how the runs went."""
    sim = os.path.join(sdir, "sim")
    truth = defaultdict(list)
    with open(os.path.join(sim, "truth.csv")) as f:
        for r in csv.DictReader(f):
            truth[r["robot"]].append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"])))
    errs = []
    with open(os.path.join(sdir, "track.csv")) as f:
        rows = list(csv.DictReader(f))
    for robot, tr in truth.items():
        ts = np.array([t[0] for t in tr])
        xs = np.array([t[1] for t in tr])
        ys = np.array([t[2] for t in tr])
        for r in rows:
            if r["robot"] != robot:
                continue
            t = float(r["t"])
            if t < ts[0] or t > ts[-1]:
                continue
            errs.append(math.hypot(float(r["x_cm"]) - np.interp(t, ts, xs), float(r["y_cm"]) - np.interp(t, ts, ys)))
    info = json.load(open(os.path.join(sim, "sim_info.json")))
    state = json.load(open(os.path.join(sdir, "e02_state.json")))
    fits = {}
    for robot, tr in info["robots"].items():
        v = (state["fits"].get(robot) or {}).get("values")
        fits[robot] = dict(true=dict(gyro_scale=round(tr["g_true"], 4), gyro_drift_dps=round(tr["gyro_drift_dps"], 4),
                                     wheel_scale=round(tr["w_true"], 4)), learned=v)
    sched = state["schedule"]
    by = defaultdict(list)
    for r in sched:
        if r["status"] == "done" and r.get("end_err_cm") is not None:
            by[r["cond"]].append((r["end_err_cm"], r.get("far_err_cm")))
    e = np.array(errs) if errs else np.array([np.nan])
    summ = dict(track_samples=len(errs), track_err_median_cm=round(float(np.median(e)), 2),
                track_err_p95_cm=round(float(np.percentile(e, 95)), 2) if errs else None,
                runs_done=sum(1 for r in sched if r["status"] == "done"),
                runs_failed=sum(1 for r in sched if r["status"] == "failed"),
                runs_per_condition={c: len(by[c]) for c in by},
                end_err_mean_cm={c: round(float(np.mean([v[0] for v in by[c]])), 2) for c in by},
                end_err_sd_cm={c: round(float(np.std([v[0] for v in by[c]], ddof=1)), 2) if len(by[c]) > 1 else None
                               for c in by},
                recoveries={k: sum(1 for r in sched if r.get("recovery") == k)
                            for k in {r.get("recovery") for r in sched if r.get("recovery")}},
                watchdog_stops=len(state.get("watchdog_stops") or []),
                help_calls=len(state.get("help_calls") or []),
                find_failures=sum(1 for r in sched if "find_failed" in (r.get("flags") or [])),
                far_err_mean_cm={c: round(float(np.mean([v[1] for v in by[c] if v[1] is not None])), 2) for c in by},
                board_true=info["board"], board_used=state.get("board"))
    rep = dict(summary=summ, fits=fits)
    with open(os.path.join(sdir, "dry_run_report.json"), "w") as f:
        json.dump(rep, f, indent=1)
    return rep


if __name__ == "__main__":
    sys.exit(main())
