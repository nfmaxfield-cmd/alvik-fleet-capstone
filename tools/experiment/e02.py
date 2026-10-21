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
  5. A robot more than 15 cm off: flagged, driven back by camera, the session carries on. Nick is called
     only when a robot is stuck or off the board, with exact steps.
Everything is saved after every step (e02_state.json), so after a crash `--resume` carries on: an
interrupted run is redone.

Files in the session folder (logs/e02-<date-time>/): e02_state.json, runs.csv (one row per run: camera
start / far / end, errors, flags, corrections used), track.csv (the moving robot's camera positions),
fits.csv (every self-calibration), e02_log.txt (everything the console said), naming_check.json,
setup.json, robot logs in robot-logs-<n>/ (the console's CSVs: raw, bump, find, e02, leg ...).

PROTOCOL (contract in the build plan; assumptions marked * are not in the contract and the firmware lane
must match them or tell us; check docs/data-formats.md "E02"):
  corr reset | set gyro_scale v | set gyro_drift_dps v | set wheel_scale v       (before `run find`)
  set e02_xboard_side 0|1, e02_yboard_side 0|1, e02_xboard_cm v, e02_yboard_cm v, e02_route 1|2|3,
      e02_mode 0|1, e02_pause_ms 1000; run find; run e02
  fix x y th t_wall    th may be "-" (*: heading unknown); t_wall = laptop Unix time of the picture (*)
  go d_cm | turn deg   *also accepted when idle (parking, finding a robot); *each ends with a LEG record
                       (columns read by name: kind, value, result); without LEG records the camera and a
                       timer decide when a leg is over
  *e02 far | e02 end   camera-steered mode: the laptop marks the far point and ends the run
  *the robot faces the x-board when `run find` starts (the laptop parks it that way), and turns towards
   the y-board itself; *routes are driven forwards only, turns towards the board's middle; *the square
   is 50 cm, 1 lap, far point = the corner diagonally opposite the start
  records read by column name: RUNINFO, RUNEND, PARAM, RAW (ms, x_cm, y_cm, yaw_deg, th_deg), FIND
  (result), E02 (ms_far), BUMP (ms, board), LEG, GYRO (raw_drift_dps)
"""
import argparse
import csv
import datetime as dt
import glob
import json
import math
import os
import subprocess
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

PAUSE_MS = 1000
FIX_HZ = 3.0
DRIVE_CMS_GUESS = 8.0
LEG_PAUSE_MS = 700        # e02_leg_pause_ms in every mode-0 run: still long enough for a camera fix (KC)
TRANSIT_MAX_CM = 140      # e02_transit_max_cm: park spots are up to ~110 cm from the find's staging point
BACK_MAX_CM = 9.5         # firmware e02_back_max_cm is 10: backwards legs are split into pieces this long
TURN_DPS_GUESS = 45.0
FIND_TIMEOUT_S = 150
E02_TIMEOUT_S = {0: 150, 1: 400}
BATTERY_WARN, BATTERY_STOP = 40, 15


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
                if n == robot and rec.get("id") == str(lid) and rec.get("run") in (run, "-") and \
                        rec.get("state") in ("done", "failed", "refused", "dup"):
                    return rec
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
        self.moves = {}               # robot -> dict(run, id, why): its camera-steered run taking legs now
        self.link = self.listener = self.watch = self.pi = None
        self.feed = self.tracker = self.history = self.cam = self.board = None

    # --- talking ----------------------------------------------------------------------------------
    def say(self, msg, quiet=False):
        line = "[%s] %s" % (dt.datetime.now().strftime("%H:%M:%S"), msg)
        if not quiet:
            print(line, flush=True)
        self.logf.write(line + "\n")
        self.logf.flush()

    def mark(self, text):
        """A note in both camera videos (camrec.py writes it to each camera's events.csv)."""
        ok = self.pi.send(text) if self.pi else False
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

    def send(self, robot, cmd):
        self.link.send_name(robot, cmd)
        self.say("    -> %s: %s" % (robot, cmd), quiet=True)

    # --- camera -----------------------------------------------------------------------------------
    def on_sample(self, s):
        if self.active:
            self.trackw.writerow(["%.3f" % s[0], self.active, "%.2f" % s[1], "%.2f" % s[2], self.order, self.phase])

    def pos(self, robot=None, fresh_s=1.5):
        p = self.tracker.where()
        if p is None or (robot and self.tracker.active != robot):
            return None
        if time.time() - p[0] > fresh_s:
            return None
        return p

    def settle(self, t_from=None, min_s=0.4, timeout=3.0):
        """The active robot's position once it stands still (median of min_s), or its latest one."""
        end = time.time() + timeout
        while time.time() < end:
            sp = CAM.still_position(self.history.since(max(t_from or 0, time.time() - 2.0)), min_s=min_s)
            if sp:
                return sp
            time.sleep(0.1)
        p = self.tracker.where()
        return (p[1], p[2]) if p else None

    # --- runs and camera-steered moves -------------------------------------------------------------
    def start_run(self, robot, cmd="run e02", retry=True):
        """Send a `run ...` and wait for its RUNINFO. A robot that says it is not calibrated (switched off and
        on) gets its calibration first, then one more try. Returns the run id or None."""
        before = self.watch.started.get(robot)
        t0 = time.time()
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
            for c in ("set calib_spin_deg 0", "set calib_roll_cm 0", "set calib_roll_min_cm 99"):
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
        p = self.settle(min_s=0.3)
        if p is None:
            return None
        h = heading if heading is not None else self.heads.get(robot, [None])[0]
        self.send(robot, "fix %.2f %.2f %s %.3f" % (p[0], p[1], "-" if h is None else "%.1f" % h, time.time()))
        time.sleep(0.15)
        return p

    def begin_moves(self, robot, why="transit"):
        """Legs (go / turn) are only taken inside a camera-steered `run e02` (e02_mode 1). For moves that are not
        measurements (parking, finding a robot, putting it back) one is started here; they are listed in
        transit_runs.csv so the analysis leaves them out."""
        if robot in self.moves:
            return True
        self.set_corrections(robot, self.move_corrections(robot))
        h = self.heads.get(robot, [None])[0]
        self.fix_now(robot, h if h is not None else self.board.face_xboard)   # the first fix needs a heading
        for c in ("set e02_mode 1", "set e02_route 1"):
            self.send(robot, c)
            time.sleep(0.05)
        run = self.start_run(robot, "run e02")
        if not run:
            return False
        self.moves[robot] = dict(run=run, id=0, why=why)
        self.st.setdefault("transit_runs", []).append([run, robot, why, round(time.time(), 1), self.order])
        self.write_transit()
        return True

    def end_moves(self, robot):
        m = self.moves.pop(robot, None)
        if not m:
            return
        m["id"] += 1
        self.send(robot, "mark end %d" % m["id"])
        if self.watch.wait_end(m["run"], 6) is None:
            self.send(robot, "stop")
            self.watch.wait_end(m["run"], 3)

    def send_leg(self, robot, text, timeout):
        """One leg or mark with the next id, resent once if no answer. Returns (state, info)."""
        m = self.moves.get(robot)
        if m is None:
            return "refused", "no camera-steered run"
        for attempt in range(2):
            if attempt == 0:
                m["id"] += 1
            t_s = time.time()
            self.send(robot, "%s %d" % (text, m["id"]))
            res = self.listener.wait_leg(robot, m["run"], m["id"], t_s - 0.05, timeout,
                                         stop=lambda: self.watch.ended.get(m["run"]) is not None)
            if res:
                return res.get("state", "?"), res.get("info", "")
            if self.watch.ended.get(m["run"]) is not None:
                break
        if self.watch.ended.get(m["run"]) is not None:
            self.moves.pop(robot, None)
            return "refused", "its camera-steered run ended (%s)" % self.watch.ended[m["run"]]["result"]
        return "lost", "no answer"

    def leg(self, robot, cmd, run=None):
        """One go / turn leg (a backwards go longer than e02_back_max_cm goes in pieces). Returns (p0, p1, words)."""
        kind, val = ("go" if cmd[0] == "probe" else cmd[0]), float(cmd[1])
        p0 = self.settle(min_s=0.3, timeout=1.5)
        pieces = [val]
        if kind == "go" and val < -BACK_MAX_CM:
            n = int(math.ceil(-val / BACK_MAX_CM))
            pieces = [val / n] * n
        words = ""
        for v in pieces:
            for attempt in range(2):
                if robot not in self.moves and not self.begin_moves(robot):
                    return p0, p0, "could not start a camera-steered run"
                est = abs(v) / DRIVE_CMS_GUESS + 1.0 if kind == "go" else abs(v) / TURN_DPS_GUESS + 1.0
                state, info = self.send_leg(robot, "%s %.2f" % (kind, v), 2.5 * est + 6.0)
                words = state + ((" (" + info + ")") if info else "")
                if state == "refused" and robot not in self.moves and attempt == 0:
                    continue                      # its run had ended (quiet too long): start another, try again
                break
            if state not in ("done", "dup"):
                break
        time.sleep(0.25)
        p1 = self.settle(t_from=time.time() - 0.6, min_s=0.3, timeout=2.0)
        return p0, p1, words

    def mark_leg(self, robot, which, timeout=8.0):
        state, info = self.send_leg(robot, "mark %s" % which, timeout + PAUSE_MS / 1000.0)
        return state

    def drive_to(self, robot, target, final_heading=None, run=None, max_legs=40, arrive_cm=ST.ARRIVE_CM):
        """Camera-steered to target (x, y) by single legs, then optionally turned to final_heading.
        Returns (ok, words)."""
        h = self.heads.get(robot, [None, False])
        steer = ST.Steer([target], heading=h[0], measured=h[1], final_heading=final_heading, max_legs=max_legs,
                         arrive_cm=arrive_cm)
        stuck = 0
        while True:
            p = self.settle(min_s=0.3, timeout=2.0)
            if p is None:
                return False, "the camera can't see it"
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
            p0, p1, res = self.leg(robot, cmd, run=run)
            if p0 is None or p1 is None:
                return False, "the camera lost it"
            if res.startswith("refused") or res.startswith("could not"):
                return False, "it refused the leg: " + res
            steer.after(cmd, p0, p1)
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

    # --- finding a robot with the camera ------------------------------------------------------------
    def acquire(self, robot, step_cm=12.0):
        """Find robot in the picture by a short drive (see e02_camera's docstring). Returns (x, y) or None."""
        self.tracker.forget(robot)
        before = self.tracker.blobs()
        if not self.begin_moves(robot, "finding it"):
            return None

        def new(bl):
            return [b for b in bl if all(math.hypot(b[3] - o[3], b[4] - o[4]) > 3.0 for o in before)]
        for sign in (1.0, -1.0):
            first = None
            m = self.moves.get(robot)
            if m is None:
                return None
            dist = sign * (step_cm if sign > 0 else BACK_MAX_CM)
            m["id"] += 1
            t_s = time.time()
            self.send(robot, "go %.1f %d" % (dist, m["id"]))
            end = t_s + abs(dist) / DRIVE_CMS_GUESS * 2.5 + 4.0
            while time.time() < end:
                nb = new(self.tracker.blobs())
                if nb and first is None:
                    first = (sum(b[3] for b in nb) / len(nb), sum(b[4] for b in nb) / len(nb))
                if self.listener.leg_final(robot, m["run"], m["id"]):
                    break
                time.sleep(0.07)
            time.sleep(0.8)
            nb = new(self.tracker.blobs())
            if first is None or not nb:
                continue
            if len(nb) == 1:
                # moved too little to leave its own spot, or merged: go a bit further the same way
                m["id"] += 1
                self.send(robot, "go %.1f %d" % (sign * 8.0, m["id"]))
                self.listener.wait_leg(robot, m["run"], m["id"], time.time() - 0.05, 8.0 / DRIVE_CMS_GUESS + 4.0)
                time.sleep(0.8)
                nb = new(self.tracker.blobs())
                if len(nb) < 2:
                    continue
            nb.sort(key=lambda b: math.hypot(b[3] - first[0], b[4] - first[1]))
            ghost, rob = nb[0], nb[-1]
            self.tracker.refresh_spot(ghost[3], ghost[4])
            self.tracker.follow(robot, rob[3], rob[4])
            hd = ST.bearing((ghost[3], ghost[4]), (rob[3], rob[4]))
            self.heads[robot] = [hd if sign > 0 else ST.wrap180(hd + 180), True]
            time.sleep(0.5)
            p = self.settle(min_s=0.3)
            return p
        return None

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
        if math.hypot(p[0] - px, p[1] - py) > 4.0:
            if math.hypot(p[0] - sx, p[1] - sy) > 3.0:
                ok, why = self.drive_to(robot, (sx, sy), arrive_cm=2.0)
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
        ok, why = self.drive_to(robot, (px, py), final_heading=ph, arrive_cm=3.0)
        self.end_moves(robot)
        if ok:
            p = self.settle(min_s=0.3)
            self.st["positions"][robot] = [p[0], p[1], self.heads.get(robot, [None])[0]]
            self.tracker.park(robot, p[0], p[1])
            self.st.save()
        return ok, why

    def help_from_nick(self, robot, problem):
        """Stuck or off the board: exact steps, then find it again. Returns True when it is back."""
        i = self.park_index(robot)
        px, py, _ = self.board.park(i)
        side = CAM.picture_side(self.cam, self.board.corner[0] + self.board.ux * 60, self.board.xboard_cm)
        where = "about %.0f cm (%.0f in) in from the open edge that is opposite the board on the %s of the camera " \
                "picture, roughly where it was parked before" % (PL.PARK_EDGE_CM, PL.PARK_EDGE_CM / 2.54, side)
        facing = "the blocked board with the cement blocks on the %s of the camera picture" % side
        self.mark("e02 help %s: %s" % (robot, problem))
        self.st.setdefault("help_calls", []).append([round(time.time(), 1), robot, problem])
        self.st.save()
        print("\n" + "!" * 78)
        for line in ST.nick_instructions(robot, problem, where, facing):
            self.say(line)
        print("!" * 78)
        self.send(robot, "stop")
        self.send(robot, "identify")
        if self.a.auto:
            self.say("(automatic mode: nobody to help; %s's remaining runs are put aside)" % robot)
            self.unavailable.add(robot)
            return False
        while True:
            ans = self.ask("Press Enter when it is back on the board (f = flash it again, skip = put it aside): ", "")
            if ans.lower() == "f":
                self.send(robot, "identify")
                continue
            if ans.lower() == "skip":
                self.unavailable.add(robot)
                return False
            time.sleep(1.0)
            p = self.acquire(robot)
            if p:
                ok, why = self.park(robot)
                if ok:
                    return True
                self.say("It moved, but parking failed: %s" % why)
            else:
                self.say("The camera could not find %s after a short drive. Check it is on the board, switched "
                         "on, with its front facing that board, then press Enter again." % robot)

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
        ok2, w2, _, _ = CAM.check_stream(a.pi, a.side_port)
        notes = CAM.notes_answer(a.pi)
        self.say("Overhead camera (%s:%d): %s" % (a.pi, a.overhead_port, "OK, " + w1 if ok1 else "PROBLEM: " + w1))
        self.say("Side camera (%s:%d): %s" % (a.pi, a.side_port, "OK, " + w2 if ok2 else "PROBLEM: " + w2))
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
            cal = CAM.find_calibration(a.calib, a.project, self.dir) if not a.make_calibration else None
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
        sx, sy, sh = self.board.nominal_start()
        self.say("Route start spot: (%.1f, %.1f) cm, heading %.0f deg (away from the y-board)" % (sx, sy, sh))
        for r in PL.ROUTES:
            ok, room = self.board.route_fits(r)
            self.say("Route %s from there: %s" % (PL.ROUTE_NAMES[r], "fits (closest %.1f cm from an open edge or "
                     "board)" % room if ok else "TIGHT: only %.1f cm of room" % room))
        if self.board.turn_sign < 0:
            self.say("WARNING: with the boards on these two edges the square route (the firmware turns it left) would "
                     "head into the x-board. The square runs are left out of this session's plan.")
            st["setup"]["square_left_out"] = True
        st.save()

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
        # one fresh background, then each robot found by a short drive and parked
        self.say("Taking a picture of the board with nothing moving ...")
        frames = self.feed.wait_frames(10, timeout=8)
        self.tracker.set_background(frames)
        for n in names:
            if n in self.unavailable:
                continue
            self.active = n
            self.phase = "finding"
            self.say("Finding %s in the picture (it drives about 12 cm forward) ..." % n)
            p = self.acquire(n)
            if p is None:
                self.say("The camera did not see %s move." % n)
                if not self.help_from_nick(n, "the camera did not see it move when asked to"):
                    continue
                p = self.settle()
            self.say("  %s seen at (%.1f, %.1f) cm, facing %.0f deg" % (n, p[0], p[1], self.heads[n][0]))
            self.set_corrections(n, self.move_corrections(n))
            ok, why = self.park(n)
            if not ok:
                if not self.help_from_nick(n, why):
                    continue
            p = self.settle()
            self.say("  %s parked at (%.1f, %.1f) cm" % (n, p[0], p[1]))
        self.active = None
        self.say("Compass check (`calibrate`, needed once after switching on; spin and roll off: it only wiggles) ...")
        self.calibrate([n for n in names if n not in self.unavailable])
        CAM.draw_check(self.feed.last_img, self.cam, os.path.join(self.dir, "check_robots.png"), self.board,
                       robots={n: tuple(self.st["positions"][n][:2]) for n in names if n in self.st["positions"]})

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
                "set e02_transit_max_cm %d" % TRANSIT_MAX_CM]

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
        self.st["setup"]["find_check"] = dict(robots=res, lines=summary)
        self.st.save()

    def do_find(self, robot, order):
        """A camera fix (the firmware wants one before every find), then `run find`, and the camera's view of
        it. Returns dict(ok, words, run, board_lines, t0, t1)."""
        self.end_moves(robot)
        t0 = time.time()
        self.phase = "find"
        self.fix_now(robot)
        self.mark("e02 %s %s find start" % (order, robot))
        run = self.start_run(robot, "run find")
        if not run:
            return dict(ok=False, words="did not start `run find`", run=None, t0=t0, t1=time.time())
        end = self.watch.wait_end(run, FIND_TIMEOUT_S, poll=lambda: self.watchdog(robot))
        t1 = time.time()
        if end is None:
            self.send(robot, "stop")
            end = self.watch.wait_end(run, 5) or dict(result="no end record")
        self.heads[robot] = [self.board.start_heading, False]    # find ends facing along the route (not measured)
        fr = self.listener.latest("FIND", robot, t0)
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
            smp = self.history.since(tb + 0.3, tb + 1.2)
            sp = CAM.still_position(smp, min_s=0.3) if smp else None
            if sp is None and smp:
                sp = (float(np.median([s[1] for s in smp])), float(np.median([s[2] for s in smp])))
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

    def watchdog(self, robot):
        """While a robot drives on its own: stop it if the camera sees it at an open edge or off the board."""
        p = self.pos(robot, fresh_s=2.0)
        if p is None:
            return False
        open_d, _ = self.board.edge_room(p[1], p[2])
        if open_d < PL.ROBOT_HALF_CM - 1.0 or not self.board.on_board(p[1], p[2], margin=-2.0):
            self.send(robot, "stop")
            self.say("  STOPPED %s: the camera sees it at the board's open edge (%.1f, %.1f)" % (robot, p[1], p[2]))
            self.mark("e02 watchdog stop %s" % robot)
            return True
        return False

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
        if values is None:
            self.send(robot, "corr reset")
            return ["corr reset"]
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
        params = self.board_params() + ["set e02_route %d" % route, "set e02_mode %d" % mode]
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
                r["status"] = "pending"
                r.setdefault("notes", []).append("settings lost: " + "; ".join(bad))
                self.park(robot)
                st.save()
                return
        if not fnd["ok"]:
            self.say("  Find failed: %s" % fnd["words"])
            r.setdefault("flags", []).append("find_failed")
            self.finish_failed(r, robot, "find failed: " + fnd["words"])
            return
        # 2. the condition's corrections
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
            cmds = self.set_corrections(robot, vals)
            r["corr"] = vals
        else:
            cmds = self.set_corrections(robot, None)
            cmds = ["set gyro_scale 1", "set gyro_drift_dps 0", "set wheel_scale 1"]
            r["corr"] = dict(gyro_scale=1.0, gyro_drift_dps=0.0, wheel_scale=1.0)
        r["corr_source"] = source
        time.sleep(0.3)
        # 3. the route
        self.phase = "route"
        pre = self.settle(min_s=0.5)
        nominal = self.board.start_heading
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
            r["status"] = "pending"
            r.setdefault("notes", []).append("settings lost: " + "; ".join(bad))
            self.park(robot)
            st.save()
            return
        prm = self.watch.params.get(run, {})
        k_info = None
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
        self.heads[robot] = [nominal, False]        # the route ends facing its start heading
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

        def cam_at(ms_key, span):
            ms = num(rec.get(ms_key))
            if ms is None or off is None:
                return None
            t = ms / 1000.0 + off
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
        verdict, why = ST.judge_run(far_err, end_err, on_board=on_board, seen=endp is not None, lost_cm=self.a.lost_cm)
        r["verdict"] = verdict
        if end["result"] != "done":
            r.setdefault("flags", []).append("runend:" + end["result"])
        self.last_result = "%s %s %s: far point %s, back at start %s%s" % (
            robot, cond, PL.ROUTE_SHORT[route], "%.1f cm off" % far_err if far_err is not None else "not seen",
            "%.1f cm off" % end_err if end_err is not None else "not seen",
            " (robot thought %.1f cm)" % bel if bel is not None else "")
        self.say("  Result: " + self.last_result)
        if verdict != "ok":
            r.setdefault("flags", []).append("outlier:" + verdict)
            self.say("  Flagged as an outlier: %s." % why)
            self.mark("e02 %s outlier %s" % (tag, verdict))
        # 4. learn (U runs)
        if cond == "U" and end["result"] == "done":
            try:
                self.learn(robot, r, t_run0, t_e1 + 0.5, [x for x in (fnd["run"], run) if x])
            except Exception as e:      # a fit problem must never stop the session
                self.say("  (self-calibration skipped this run: %s)" % e)
        # 5. park (this also drives a lost robot back)
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
        if not self.recover(robot, why if verdict != "ok" else None):
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
            # look for it with a short drive near where it was
            self.say("  Looking for %s with a short drive ..." % robot)
            q = self.acquire(robot, step_cm=8.0)
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
        self.moves[robot] = dict(run=run, id=0, why="K")
        self.heads[robot] = [heading if self.heads.get(robot, [None])[0] is None else self.heads[robot][0], False]
        sx, sy, _ = self.board.nominal_start()
        ok, why = self.drive_to(robot, (sx, sy), final_heading=heading)
        if not ok:
            self.say("  K drive: %s" % why)
            self.mark_leg(robot, "end")
            return info
        start = self.settle(min_s=0.5)
        info["start"] = start
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
            p = self.pos(robot, fresh_s=0.5)
            if p is None:
                continue
            th = "-"
            recent = self.history.since(time.time() - 3.0)
            sp = CAM.still_position(recent, min_s=0.3) if recent else None
            if sp:
                # the last straight stretch before standing still
                moving = [q for q in recent if math.hypot(q[1] - sp[0], q[2] - sp[1]) > 0.8]
                if moving:
                    t_stop = moving[-1][0]
                    seg = [q for q in recent if t_stop - 1.0 <= q[0] <= t_stop]
                    h = ST.heading_from_track(seg, min_cm=3.0, max_rms_cm=0.4)
                    turned = self.gyro_turn_since(run, t_stop)
                    if h is not None and turned is not None and abs(turned) < 2.0:
                        th = "%.1f" % ST.wrap180(h + turned)
                        nh += 1
            self.send(robot, "fix %.2f %.2f %s %.3f" % (p[1], p[2], th, p[0]))
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

    # --- files ------------------------------------------------------------------------------------
    RUN_COLS = ["order", "block", "robot", "cond", "route", "status", "verdict", "flags", "find_run", "e02_run",
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
                             status=r["status"], verdict=r.get("verdict", ""), flags=" ".join(r.get("flags", [])),
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

    def write_transit(self):
        write_csv(os.path.join(self.dir, "transit_runs.csv"), ["run", "robot", "why", "t", "during_order"],
                  [dict(zip(["run", "robot", "why", "t", "during_order"], r)) for r in self.st.get("transit_runs", [])])

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
            nb = self.a.blocks or PL.blocks_for(self.a.hours, self.a.run_s, len(st["robots"]))
            sched = PL.make_schedule(st["robots"], nb, st["seed"])
            if self.a.routes != "1,2,3":
                keep = {int(v) for v in self.a.routes.split(",")}
                sched = [r for r in sched if r["route"] in keep]
                for i, r in enumerate(sched):
                    r["order"] = i + 1
            if st["setup"].get("square_left_out"):
                sched = [r for r in sched if r["route"] != 3]
                for i, r in enumerate(sched):
                    r["order"] = i + 1
            st["schedule"] = sched
            st.save()
            self.say("Schedule: %d runs (block 0: %d U runs, then %d blocks), about %s at %.0f s a run." % (
                len(sched), sum(1 for r in sched if r["block"] == 0), nb, hms(len(sched) * self.a.run_s), self.a.run_s))
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
                self.boards_still()
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
    ap.add_argument("--run-s", type=float, default=110.0, help="time a run takes, for the plan (s)")
    ap.add_argument("--blocks", type=int, help="regular blocks after the all-U block (default: what fits in --hours)")
    ap.add_argument("--routes", default="1,2,3", help="only these routes (testing)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--fix-hz", type=float, default=FIX_HZ)
    ap.add_argument("--lost-cm", type=float, default=ST.LOST_CM)
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
    if a.resume:
        if a.resume == "latest":
            cands = sorted(glob.glob(os.path.join(REPO, "logs", "e02-*", "e02_state.json")))
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
    hist = CAM.History()
    run.history = hist
    code = 0
    feed = None
    try:
        feed = CAM.Feed("%s:%d" % (a.pi, a.overhead_port), None, hist, log=lambda m: run.say("  (camera: %s)" % m))
        feed.on_sample = run.on_sample
        feed.start()
        run.feed = feed
        run.setup()
        run.robots_ready()
        if not a.skip_find_check and not state["setup"].get("find_check"):
            run.find_check()
        if not a.skip_naming and not state.get("naming_check"):
            run.naming_check()
        run.session()
    except KeyboardInterrupt:
        run.say("Stopped by Ctrl+C: every robot is stopped. Start again with --resume to carry on.")
        code = 1
    except SystemExit as e:
        run.say(str(e))
        code = 1
    finally:
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
    for f in ("STOP", "sim_info.json"):
        if os.path.exists(os.path.join(simdir, f)):
            os.remove(os.path.join(simdir, f))
    sides = (state["setup"].get("args") or {}).get("board_sides") or "1,1"
    sim = subprocess.Popen([sys.executable, os.path.join(HERE, "e02_sim.py"), "--out", simdir,
                            "--xboard-side", sides.split(",")[0], "--yboard-side", sides.split(",")[1],
                            "--robots", ",".join(state["robots"]), "--seed", str(state["seed"] % 1000 + 1)],
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
                end_err_mean_cm={c: round(float(np.mean([v[0] for v in by[c]])), 2) for c in by},
                far_err_mean_cm={c: round(float(np.mean([v[1] for v in by[c] if v[1] is not None])), 2) for c in by},
                board_true=info["board"], board_used=state.get("board"))
    rep = dict(summary=summ, fits=fits)
    with open(os.path.join(sdir, "dry_run_report.json"), "w") as f:
        json.dump(rep, f, indent=1)
    return rep


if __name__ == "__main__":
    sys.exit(main())
