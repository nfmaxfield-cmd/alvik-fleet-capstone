#!/usr/bin/env python3
"""Mark the robot (MIRLT): hand-checked robot positions from recorded footage, marked on a laptop or phone instead
of measured on the floor (Nick, 2026-10-25: "you give me frames with your guesses, I move the green ring to mark
the robot, then you record its measurement").

It picks one picture a second from a stretch of a recording, guesses where the robot is with the live tracker
(e02_camera.MotionTracker, replayed on the footage), and shows each picture with the guess as a green ring the size
of a robot. You slide the ring onto the robot and press Confirm. Each mark is saved in pixels and in floor cm (the
floor calibration), with the guess and how far off it was. When you correct a guess, the tracker starts again from
your mark, so the next pictures' guesses follow the robot you marked and most need only Confirm.

The marks are the hand-checked truth the tracker-replay experiments need (docs/experiments/DOE-tracking.md DOE 1,
docs/doe-backlog.md "Camera and tracking": a position checked about once a second).

HOW TO USE IT (Nick, on the laptop)
-----------------------------------
The easy way: in the project folder (alvik-fleet-capstone), double-click  mark-the-robot.bat
A black window opens and says "Open http://localhost:8770 ...", and your browser opens the page by itself.
The page lists the three stretches of the 2026-10-22 evening (about 6 minutes of footage in all). Click one: the
page downloads just that stretch (about 100 MB, a few minutes; the black window shows the progress), then shows the
first picture with a green ring.

The PowerShell way (the same thing): open PowerShell in the project folder, type this and press Enter:

    py -3 tools\\label\\mark_robot.py

On each picture:
  - The green ring is the guess. If it sits on the robot, press Enter (or the green Confirm button).
  - If not: drag anywhere on the picture to slide the ring (it moves with your finger or mouse, finely), or tap
    or click where the robot is to put the ring there. Then Confirm. Arrow keys nudge it by one pixel.
  - Robot hidden or out of the picture: "Not visible" (N). Can't tell which robot is the one (several close
    together): put the ring on your best guess and press "Not sure which" (W). Don't want to judge it: "Skip" (S).
  - "Play last second" (P) shows the second before this picture, to see which robot moved.
  - Undo (Z) takes back the last mark. You can close the window any time: the next start carries on where you
    stopped. A few pictures come back at the end with the ring put off on purpose: they measure how exactly
    marks repeat.
When it says "All marked", the marks are in tools/experiment/data/marks/<stretch>/ (marks.csv and summary.json).
Send those two files (drag them into the chat).

Any other session that was uploaded to GitHub (troubleshooting: where did the runner lose the robot?). Only the
minutes you ask for are downloaded, plus the session's logs, so each picture also shows where the runner thought
the robot was (red cross), and the marks save how far that was from your mark:

    py -3 tools\\label\\mark_robot.py --release e02-session-20261024-193526 --robot Right --from 19:40:00 --to 19:42:00

Your own recording folder on the laptop (video.avi + frames.csv, copied from the Pi; or a folder of pictures),
with the runner's session folder for the red crosses:

    py -3 tools\\label\\mark_robot.py "C:\\Users\\Nick\\Desktop\\recordings\\20261025-101500_E02" --robot Left --from 10:20:05 --to 10:22:00 --session logs\\e02-20261025-101530

Times are the clock time on the recording's day (or +M:SS after the recording's start). With a session folder the
tool uses that session's own calibration (a from-scratch session's too), its legs (when the robot was driving) and
its laptop-to-Pi clock offset. Without any calibration (--no-calib) the marks are saved in pixels only;
    py -3 tools\\label\\mark_robot.py --recompute-cm <marks folder> --calib <calibration.json>
fills in the cm later.

Other options: --every 0.5 (pictures this many seconds apart), --at x,y (where the robot is at --from, floor cm),
--port, --phone (also reachable from a phone on the home WiFi: the black window prints the address), --no-browser,
--checks 0 (no repeat checks), --get-only (download, don't open the page).

For the replay experiments: load_truth(<marks folder>) gives the sure marks [dict(t, x, y, ...)];
score_track(samples, truth) scores a replayed track [(t, x, y or None)] against them (more than 5 cm off = wrong).
"""
import argparse
import csv
import datetime as dt
import glob
import io
import json
import math
import os
import random
import re
import socket
import sys
import threading
import time
import urllib.request
import zipfile
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
EXP = os.path.join(REPO, "tools", "experiment")
for p in (HERE, EXP):
    if p not in sys.path:
        sys.path.insert(0, p)

import mark_footage as FT  # noqa: E402

PAGE = os.path.join(HERE, "mark_robot.html")
DEFAULT_CALIB = os.path.join(EXP, "data", "overhead_floorcal_roam1.json")
MARKS_ROOT = os.path.join(EXP, "data", "marks")
CLIPS_JSON = os.path.join(EXP, "tests", "data", "realclips")
PORT = 8770

ROBOT_TOP_CM = 3.7          # the camera sees the robot's top (e02_camera.ROBOT_TOP_CM); marks are on that plane
ROBOT_CM = 9.5
RESEED_CM = 3.0             # chosen: a mark this far from the tracker's own guess starts the tracker again there
                            # (about the tracker's agreement scale; a DOE is not worth it: it only changes the help)
BIAS_N = 8                  # the guess is shifted by the median of the last this-many small corrections ...
BIAS_MIN_N = 3              # ... once there are this many
CHECK_OFFSET_CM = 6.0       # a repeat check's ring starts this far off the first mark (more than the 5 cm limit)
CHECK_SHARE = 0.05          # chosen: share of marked pictures shown again as repeat checks (at least 3)
MARK_STATUSES = ("confirmed", "corrected", "unsure", "not_visible", "skipped")
TRUTH_STATUSES = ("confirmed", "corrected")

# The stretches the tracker-replay DOE needs (DOE-tracking.md DOE 1: the three evening losses of 2026-10-22).
# Each reads its times, start spot and legs from the regression test's json (tests/data/realclips).
PRESETS = {
    "oct22eve_left": dict(robot="Left", json="oct22eve_left.json", release="e02-session-oct22-evening",
                          title="Left, 22 Oct 19:30, beside the x-board", board_sides=(1, 1)),
    "oct22eve_center": dict(robot="Center", json="oct22eve_center.json", release="e02-session-oct22-evening",
                            title="Center, 22 Oct 20:09, the open corner", board_sides=(1, 1)),
    "oct22eve_right": dict(robot="Right", json="oct22eve_right.json", release="e02-session-oct22-evening",
                           title="Right, 22 Oct 20:26, the bright corner", board_sides=(1, 1)),
}

COLUMNS = ["item", "kind", "pick", "frame", "t_wall", "clock", "status", "u_px", "v_px", "x_cm", "y_cm",
           "guess_u_px", "guess_v_px", "guess_x_cm", "guess_y_cm", "guess_source", "guess_diff_px", "guess_diff_cm",
           "tracker_u_px", "tracker_v_px", "tracker_x_cm", "tracker_y_cm", "tracker_diff_cm", "runner_x_cm", "runner_y_cm", "runner_diff_cm",
           "repeat_of_u_px", "repeat_of_v_px", "repeat_diff_cm", "moved_px", "seconds", "reseed", "seq",
           "marked_at"]


# ============================================================================ small helpers
def default_footage_root():
    docs = os.path.join(os.path.expanduser("~"), "Documents")
    return os.path.join(docs if os.path.isdir(docs) else os.path.expanduser("~"), "mirlt-footage")


def clock(t):
    return dt.datetime.fromtimestamp(t).strftime("%H:%M:%S")


def parse_time(s, t_first):
    """A time given on the command line: Unix seconds, '+M:SS' / '+S' after the recording's first picture, or a
    clock time HH:MM[:SS] on the recording's day (this computer's time zone)."""
    s = str(s).strip()
    if s.startswith("+"):
        parts = [float(p) for p in s[1:].split(":")]
        sec = 0.0
        for p in parts:
            sec = sec * 60 + p
        return t_first + sec
    try:
        v = float(s)
        if v > 1e8:
            return v
    except ValueError:
        pass
    parts = s.split(":")
    if len(parts) not in (2, 3):
        raise SystemExit("Time %r: give it as HH:MM:SS (clock time), +M:SS (after the start) or Unix seconds" % s)
    h, m = int(parts[0]), int(parts[1])
    sec = float(parts[2]) if len(parts) == 3 else 0.0
    day = dt.datetime.fromtimestamp(t_first)
    return dt.datetime(day.year, day.month, day.day, h, m, int(sec), int(round((sec % 1) * 1e6))).timestamp()


def _f(v, nd=2):
    return "" if v is None else ("%.*f" % (nd, v))


def _num(s):
    try:
        return float(s) if s not in ("", None) else None
    except ValueError:
        return None


class Calib:
    """Pixels <-> floor cm on the plane of the robot's top, from a floor calibration (or none: pixels only)."""

    def __init__(self, path=None, robot_px=None):
        self.path = path
        self.cam = None
        if path:
            import e02_camera as CAM
            self.cam = CAM.load_camera(path)
        self.robot_px_fixed = robot_px

    @property
    def ok(self):
        return self.cam is not None

    def to_cm(self, u, v):
        if self.cam is None:
            return None
        x, y = self.cam.to_floor(float(u), float(v), ROBOT_TOP_CM)[:2]
        return (float(x), float(y)) if np.isfinite(x) and np.isfinite(y) else None

    def to_px(self, x, y):
        if self.cam is None:
            return None
        u, v = self.cam.project((float(x), float(y), ROBOT_TOP_CM))[:2]
        return (float(u), float(v)) if np.isfinite(u) and np.isfinite(v) else None

    def ring_px(self, u=None, v=None):
        """The robot's width in pixels where (u, v) is (the board's middle when not given)."""
        if self.robot_px_fixed:
            return float(self.robot_px_fixed)
        if self.cam is None:
            return 23.0
        p = self.to_cm(u, v) if u is not None else None
        x, y = p if p else (61.0, 61.0)
        s = self.cam.px_per_cm(x, y, ROBOT_TOP_CM)
        if not np.isfinite(s) or s <= 0:
            s = self.cam.px_per_cm(61.0, 61.0, ROBOT_TOP_CM)
        return float(ROBOT_CM * s)

    def grid(self, w, h, step=16):
        """Floor cm at every step-th pixel (the page interpolates it to show cm while you drag)."""
        if self.cam is None:
            return None
        us = list(range(0, w + step, step))
        vs = list(range(0, h + step, step))
        g = []
        for v in vs:
            row = []
            for u in us:
                p = self.to_cm(u, v)
                row.append([round(p[0], 2), round(p[1], 2)] if p else None)
            g.append(row)
        return dict(step=step, nu=len(us), nv=len(vs), xy=g)


# ============================================================================ the tracker's guesses
class Guesser(threading.Thread):
    """Runs the live tracker over the stretch from a seed (a start spot, or your latest correction) and keeps its
    position at every picked picture. A new seed stops the run and starts it again from there."""

    def __init__(self, sess):
        super().__init__(daemon=True)
        self.s = sess
        self.cv = threading.Condition()
        self.want = None            # (pick index, x_cm, y_cm) to run from
        self.running = None
        self.guesses = {}           # pick index -> (x_cm, y_cm, source) from the current run
        self.reached = -1           # highest pick index the current run has passed
        self.error = None
        self.stop_flag = False
        self.frames_done = 0
        self.done_run = False

    def request(self, seed):
        with self.cv:
            if seed != self.want:
                self.want = seed
                for k in [k for k in self.guesses if k >= seed[0]]:
                    del self.guesses[k]         # the old run's guesses past the new seed are stale now
                self.cv.notify()

    def stop(self):
        with self.cv:
            self.stop_flag = True
            self.cv.notify()

    def run(self):
        while True:
            with self.cv:
                while not self.stop_flag and (self.want is None or self.want == self.running):
                    self.cv.wait()
                if self.stop_flag:
                    return
                seed = self.want
                self.running = seed
                self.done_run = False
                self.reached = seed[0] - 1
                for k in [k for k in self.guesses if k >= seed[0]]:
                    del self.guesses[k]
            try:
                self._run(seed)
            except Exception as e:      # the page still works without guesses (they follow your marks)
                self.error = "%s: %s" % (type(e).__name__, e)
                with self.cv:
                    self.done_run = True

    def _changed(self, seed):
        with self.cv:
            return self.stop_flag or self.want != seed

    def _run(self, seed):
        import e02_camera as CAM
        import e02_plan as PL
        s = self.s
        k0, x0, y0 = seed[:3]
        f0 = s.picks[k0]
        cam = s.calib.cam
        bd = s.board
        tr = CAM.MotionTracker(cam, ((bd.x0 + bd.x1) / 2, (bd.y0 + bd.y1) / 2), board_rect=(bd.x0, bd.y0, bd.x1, bd.y1))
        for k, v in (s.tracker_settings or {}).items():
            setattr(tr, k, v)
        if s.walls:
            tr.set_area((bd.x0, bd.y0, bd.x1, bd.y1), walls=[("y", bd.xboard_cm + bd.uy * PL.ROBOT_HALF_CM),
                                                            ("x", bd.yboard_cm + bd.ux * PL.ROBOT_HALF_CM)])
        floor = [s.store.gray(f) for f in s.floor_frames]
        floor = [g for g in floor if g is not None]
        warm = [f for f in s.store.neighbours(f0, before=12) if f < f0]
        if not floor:
            floor = [s.store.gray(f) for f in warm] or [s.store.gray(f0)]
            tr.set_background(floor, robots=[(x0, y0)] + list(s.others))
        else:
            tr.set_background(floor, robots=list(s.others))
        for j, o in enumerate(s.others):
            tr.park("other%d" % j, o[0], o[1])
        tr.follow(s.robot, x0, y0, t=s.store.t(f0))
        for f in warm:
            tr.push(s.store.gray(f), s.store.t(f))
        tr.follow(s.robot, x0, y0, t=s.store.t(f0))
        legs = s.legs
        held = False
        if legs:
            tr.hold(6.0)
            held = True
        pick_at = {f: k for k, f in enumerate(s.picks) if k >= k0}
        last = s.picks[-1]
        i = s.store.frames.index(f0)
        while i < len(s.store.frames):
            f = s.store.frames[i]
            if f > last:
                break
            if self._changed(seed):
                return
            t = s.store.t(f)
            if legs:
                cur = [g for g in legs if g[0] - 0.1 <= t <= g[1] + 0.3]
                if cur and held:
                    p = tr.where()
                    tr.clear_expect()
                    if p:
                        tr.expect(p[1], p[2], 7.0 if cur[0][2] == "turn" else 18.0, kind=cur[0][2])
                        tr.hint = tr.hint[:3] + (t + 30,)        # replay time, not the wall clock
                    held = False
                elif not cur and not held:
                    tr.hold(6.0)
                    held = True
            g = s.store.gray(f)
            out = tr.push(g, t) if g is not None else None
            self.frames_done += 1
            if f in pick_at:
                k = pick_at[f]
                if k == k0:
                    val = (x0, y0, "your mark" if seed[3:] == ("mark",) else "start spot")
                elif out is not None:
                    val = (float(out[1]), float(out[2]), "tracker")
                else:
                    w = tr.where()
                    val = (float(w[1]), float(w[2]), "tracker, last seen") if w else None
                with self.cv:
                    if self.want != seed:
                        return
                    if val:
                        self.guesses[k] = val
                    self.reached = k
            i += 1
        with self.cv:
            if self.want == seed:
                self.reached = len(s.picks) - 1
                self.done_run = True


# ============================================================================ a marking session
class Session:
    """One stretch being marked: the picked pictures, the guesses, your marks (saved after every change)."""

    def __init__(self, store, calib, robot, t0, t1, out_dir, every=1.0, seed=None, legs=(), others=(), board=None,
                 walls=False, floor_frames=(), runner=None, checks=CHECK_SHARE, name="", title="",
                 tracker_settings=None, guesses=True, source=None):
        import e02_plan as PL
        self.store, self.calib, self.robot = store, calib, robot
        self.t0, self.t1, self.every = float(t0), float(t1), float(every)
        self.out = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.legs = [tuple(g) for g in (legs or [])]
        self.others = [tuple(o) for o in (others or [])]
        self.board = board or PL.Board()
        self.walls = walls
        self.floor_frames = list(floor_frames or [])
        self.runner = runner or []          # [(t, x, y)] the runner's own positions (troubleshooting)
        self.check_share = float(checks)
        self.name, self.title = name, title or name
        self.tracker_settings = tracker_settings
        self.source = source or {}
        self.lock = threading.RLock()
        self.picks = self._pick_frames()
        self.items = [dict(kind="pick", pick=k, frame=f) for k, f in enumerate(self.picks)]
        self.marks = {}                     # item index -> row (dict of COLUMNS)
        self.undo = []                      # [(item, previous row or None)]
        self.seq = 0
        self.started = time.time()
        self.seed0 = (0, float(seed[0]), float(seed[1])) if seed else None
        self._load()
        self.guesser = None
        if guesses and calib.ok:
            self.guesser = Guesser(self)
            self.guesser.start()
            self._update_seed()

    # --- picking ---------------------------------------------------------------------------------
    def _pick_frames(self):
        pj = os.path.join(self.out, "picks.json")
        if os.path.exists(pj):
            with open(pj) as f:
                d = json.load(f)
            fr = [int(x) for x in d["frames"] if self.store.has(int(x))]
            if fr:
                return fr
        out, t = [], self.t0
        while t <= self.t1 + 1e-6:
            f = self.store.nearest(t)
            if abs(self.store.t(f) - t) <= max(0.5, self.every / 2) and (not out or f != out[-1]):
                out.append(f)
            t += self.every
        if not out:
            raise SystemExit("No pictures between %s and %s in this footage (it runs %s to %s)."
                             % (clock(self.t0), clock(self.t1), clock(self.store.t(self.store.frames[0])),
                                clock(self.store.t(self.store.frames[-1]))))
        with open(pj, "w") as f:
            json.dump(dict(frames=out, every_s=self.every, t0=self.t0, t1=self.t1, robot=self.robot), f)
        return out

    # --- saving and loading ------------------------------------------------------------------------
    def _load(self):
        p = os.path.join(self.out, "marks.csv")
        ip = os.path.join(self.out, "items.json")
        if os.path.exists(ip):
            with open(ip) as f:
                self.items = json.load(f)["items"]
        if os.path.exists(p):
            with open(p, newline="") as f:
                for r in csv.DictReader(f):
                    k = int(r["item"])
                    if k < len(self.items):
                        self.marks[k] = {c: r.get(c, "") for c in COLUMNS}
                        self.seq = max(self.seq, int(r.get("seq") or 0))
        sp = os.path.join(self.out, "state.json")
        if os.path.exists(sp):
            try:
                with open(sp) as f:
                    st = json.load(f)
                self.undo = [(u[0], u[1]) for u in st.get("undo", [])]
                self.spent_before = float(st.get("seconds_spent", 0.0))
            except (ValueError, KeyError):
                self.undo = []
        if not hasattr(self, "spent_before"):
            self.spent_before = 0.0

    def _save(self):
        p = os.path.join(self.out, "marks.csv")
        with open(p + ".tmp", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=COLUMNS)
            w.writeheader()
            for k in sorted(self.marks):
                w.writerow(self.marks[k])
        os.replace(p + ".tmp", p)
        with open(os.path.join(self.out, "items.json"), "w") as f:
            json.dump(dict(items=self.items), f)
        with open(os.path.join(self.out, "state.json"), "w") as f:
            json.dump(dict(undo=self.undo[-200:], seconds_spent=round(self.spent(), 1)), f)
        with open(os.path.join(self.out, "summary.json"), "w") as f:
            json.dump(self.summary(), f, indent=1)

    def spent(self):
        return self.spent_before + (time.time() - self.started)

    # --- guesses -----------------------------------------------------------------------------------
    def _frontier(self):
        for k, it in enumerate(self.items):
            if k not in self.marks:
                return k
        return len(self.items)

    def _resolve_pending(self):
        """Marks made before the tracker reached their picture: once it has, fill in its guess and restart it from
        the mark only if it disagrees (so confirming quickly never makes it start over and over)."""
        if self.guesser is None:
            return
        changed = False
        with self.guesser.cv:
            if self.guesser.running != self.guesser.want:
                return                              # a new run is starting: wait for it
            gs, run, reached, done = dict(self.guesser.guesses), self.guesser.running, self.guesser.reached, \
                self.guesser.done_run
        for k, r in self.marks.items():
            if r["reseed"] != "pending":
                continue
            pk = int(r["pick"])
            g = gs.get(pk)
            if g is not None and r["x_cm"] != "":
                px = self.calib.to_px(g[0], g[1])
                r["tracker_x_cm"], r["tracker_y_cm"] = _f(g[0]), _f(g[1])
                if px:
                    r["tracker_u_px"], r["tracker_v_px"] = _f(px[0]), _f(px[1])
                d = math.hypot(float(r["x_cm"]) - g[0], float(r["y_cm"]) - g[1])
                r["tracker_diff_cm"] = _f(d)
                r["reseed"] = "1" if d > RESEED_CM else "0"
                changed = True
            elif run is not None and (run[0] > pk or ((reached >= pk or done) and run[0] <= pk)):
                r["reseed"] = "1" if run[0] <= pk else "0"     # it passed without seeing the robot / never will
                changed = True
        if changed:
            self._save()
            self._update_seed()

    def _update_seed(self):
        """Run the tracker from your latest correction before the first unmarked picture (or the start spot)."""
        if self.guesser is None:
            return
        front = self._frontier()
        best = None
        for k, r in self.marks.items():
            if r["kind"] != "pick" or r["reseed"] != "1" or r["x_cm"] == "":
                continue
            pk = int(r["pick"])
            if pk < front and (best is None or pk > best[0]):
                best = (pk, float(r["x_cm"]), float(r["y_cm"]), "mark")
        seed = best or self.seed0
        if seed is not None:
            self.guesser.request(seed)

    def _last_mark_px(self, k_item):
        """Your nearest earlier mark with a position (the ring follows it when there is no tracker guess)."""
        it = self.items[k_item]
        best = None
        for k, r in self.marks.items():
            if r["kind"] == "pick" and r["u_px"] != "" and int(r["pick"]) < it["pick"]:
                if best is None or int(r["pick"]) > best[0]:
                    best = (int(r["pick"]), float(r["u_px"]), float(r["v_px"]))
        return best[1:] if best else None

    def bias_px(self):
        """Median of your last few small corrections of the tracker (where you put the centre against where it
        does), in pixels; (0, 0) until there are enough."""
        rows = [r for k, r in sorted(self.marks.items(), key=lambda kv: int(kv[1]["seq"] or 0))
                if r["kind"] == "pick" and r["status"] in TRUTH_STATUSES and r["tracker_u_px"] != ""
                and r["u_px"] != "" and r["reseed"] != "1"]
        rows = rows[-BIAS_N:]
        if len(rows) < BIAS_MIN_N:
            return 0.0, 0.0
        bu = float(np.median([float(r["u_px"]) - float(r["tracker_u_px"]) for r in rows]))
        bv = float(np.median([float(r["v_px"]) - float(r["tracker_v_px"]) for r in rows]))
        return (bu, bv) if math.hypot(bu, bv) >= 1.0 else (0.0, 0.0)

    def guess(self, k_item):
        """What the page shows for an item: the ring's start (u, v), where it came from, and whether the tracker
        has got there yet."""
        it = self.items[k_item]
        if it["kind"] == "check":
            return dict(u=it["start_u"], v=it["start_v"], source="repeat check", ready=True, tracker=None)
        k = it["pick"]
        g = None
        ready = True
        if self.guesser is not None:
            with self.guesser.cv:
                g = self.guesser.guesses.get(k)
                ready = self.guesser.running is not None and self.guesser.running == self.guesser.want and (
                    self.guesser.reached >= k or self.guesser.done_run)
                if self.guesser.want is None:
                    ready = True
        if g:
            px = self.calib.to_px(g[0], g[1])
            if px:
                bu, bv = self.bias_px() if g[2].startswith("tracker") else (0.0, 0.0)
                return dict(u=px[0] + bu, v=px[1] + bv, source=g[2] + (" (+ your usual offset)" if (bu or bv) else ""),
                            ready=True, tracker=(g[0], g[1]), tracker_px=px)
        lm = self._last_mark_px(k_item)
        if lm:
            return dict(u=lm[0], v=lm[1], source="your last mark", ready=ready, tracker=None)
        if self.seed0 and k == 0:
            px = self.calib.to_px(self.seed0[1], self.seed0[2])
            if px:
                return dict(u=px[0], v=px[1], source="start spot", ready=True, tracker=None)
        img = self.store.gray(it["frame"])
        h, w = img.shape[:2] if img is not None else (480, 640)
        return dict(u=w / 2.0, v=h / 2.0, source="none: put the ring on %s" % self.robot, ready=ready, tracker=None)

    def runner_at(self, t, within=0.5):
        best = None
        for tt, x, y in self.runner:
            if abs(tt - t) <= within and (best is None or abs(tt - t) < best[0]):
                best = (abs(tt - t), x, y)
        return best[1:] if best else None

    # --- the page's questions ----------------------------------------------------------------------
    def item_info(self, k):
        with self.lock:
            self._resolve_pending()
            it = self.items[k]
            f = it["frame"]
            t = self.store.t(f)
            g = self.guess(k)
            prev = self._last_mark_px(k) if it["kind"] == "pick" else None
            run = self.runner_at(t)
            run_px = self.calib.to_px(*run) if run else None
            r = self.marks.get(k)
            mark = None
            if r:
                mark = dict(status=r["status"], u=_num(r["u_px"]), v=_num(r["v_px"]), x=_num(r["x_cm"]),
                            y=_num(r["y_cm"]), diff_cm=_num(r["guess_diff_cm"]), diff_px=_num(r["guess_diff_px"]))
            ring_u, ring_v = (mark["u"], mark["v"]) if mark and mark["u"] is not None else (g["u"], g["v"])
            n_checks = sum(1 for i in self.items if i["kind"] == "check")
            return dict(k=k, kind=it["kind"], pick=it["pick"], n_picks=len(self.picks), n_items=len(self.items),
                        check_no=(k - len(self.picks) + 1) if it["kind"] == "check" else None, n_checks=n_checks,
                        frame=f, t=t, clock=clock(t), rel="+%d:%04.1f" % divmod(t - self.t0, 60),
                        guess=dict(u=g["u"], v=g["v"], source=g["source"], ready=g["ready"]),
                        ring_r=self.calib.ring_px(ring_u, ring_v) / 2.0,
                        prev=prev, runner=list(run_px) if run_px else None, mark=mark,
                        prev_frame=self.picks[it["pick"] - 1] if it["pick"] > 0 else None)

    def state(self):
        with self.lock:
            counts = {s: 0 for s in MARK_STATUSES}
            for r in self.marks.values():
                if r["kind"] == "pick":
                    counts[r["status"]] = counts.get(r["status"], 0) + 1
            self._resolve_pending()
            n_marked_picks = sum(1 for r in self.marks.values() if r["kind"] == "pick")
            if not hasattr(self, "_wh"):
                img = self.store.gray(self.picks[0])
                self._wh = img.shape[1::-1] if img is not None else (640, 480)
            w, h = self._wh
            gs = self.guesser
            return dict(name=self.name, title=self.title, robot=self.robot, n_picks=len(self.picks),
                        n_items=len(self.items), marked=len(self.marks), marked_picks=n_marked_picks,
                        counts=counts, next=self._frontier(), done=self._frontier() >= len(self.items),
                        width=w, height=h, has_cm=self.calib.ok, out=self.out,
                        calib=os.path.basename(self.calib.path) if self.calib.path else None,
                        every=self.every, t0=clock(self.t0), t1=clock(self.t1), can_undo=bool(self.undo),
                        guesses=gs is not None, guess_error=gs.error if gs else None,
                        summary=self.summary_short())

    # --- marking -----------------------------------------------------------------------------------
    def mark(self, k, status, u=None, v=None, moved_px=None, seconds=None):
        if status not in MARK_STATUSES:
            raise ValueError("status must be one of %s" % ", ".join(MARK_STATUSES))
        with self.lock:
            it = self.items[k]
            f = it["frame"]
            t = self.store.t(f)
            g = self.guess(k)
            row = {c: "" for c in COLUMNS}
            row.update(item=k, kind=it["kind"], pick=it["pick"], frame=f, t_wall="%.4f" % t, clock=clock(t),
                       status=status, guess_u_px=_f(g["u"]), guess_v_px=_f(g["v"]), guess_source=g["source"],
                       moved_px=_f(moved_px, 1), seconds=_f(seconds, 1), marked_at="%.1f" % time.time())
            gcm = self.calib.to_cm(g["u"], g["v"])
            if gcm:
                row["guess_x_cm"], row["guess_y_cm"] = _f(gcm[0]), _f(gcm[1])
            if g.get("tracker"):
                row["tracker_x_cm"], row["tracker_y_cm"] = _f(g["tracker"][0]), _f(g["tracker"][1])
                row["tracker_u_px"], row["tracker_v_px"] = _f(g["tracker_px"][0]), _f(g["tracker_px"][1])
            run = self.runner_at(t)
            if run:
                row["runner_x_cm"], row["runner_y_cm"] = _f(run[0]), _f(run[1])
            has_pos = status in ("confirmed", "corrected", "unsure") and u is not None and v is not None
            if has_pos:
                u, v = float(u), float(v)
                row["u_px"], row["v_px"] = _f(u), _f(v)
                d_px = math.hypot(u - g["u"], v - g["v"])
                row["guess_diff_px"] = _f(d_px)
                cm = self.calib.to_cm(u, v)
                if cm:
                    row["x_cm"], row["y_cm"] = _f(cm[0]), _f(cm[1])
                    if gcm:
                        row["guess_diff_cm"] = _f(math.hypot(cm[0] - gcm[0], cm[1] - gcm[1]))
                    if g.get("tracker"):
                        row["tracker_diff_cm"] = _f(math.hypot(cm[0] - g["tracker"][0], cm[1] - g["tracker"][1]))
                    if run:
                        row["runner_diff_cm"] = _f(math.hypot(cm[0] - run[0], cm[1] - run[1]))
                if status == "confirmed" and d_px > 0.5:
                    row["status"] = status = "corrected"
                tr_d = _num(row["tracker_diff_cm"])
                if it["kind"] != "pick":
                    row["reseed"] = "0"
                elif tr_d is None and self.guesser is not None and not g["ready"]:
                    row["reseed"] = "pending"      # the tracker hasn't got here yet: decide when it has
                else:
                    row["reseed"] = "1" if tr_d is None or tr_d > RESEED_CM else "0"
                if it["kind"] == "check":
                    row["repeat_of_u_px"], row["repeat_of_v_px"] = _f(it["orig_u"]), _f(it["orig_v"])
                    ocm = self.calib.to_cm(it["orig_u"], it["orig_v"])
                    if cm and ocm:
                        row["repeat_diff_cm"] = _f(math.hypot(cm[0] - ocm[0], cm[1] - ocm[1]))
            self.seq += 1
            row["seq"] = self.seq
            self.undo.append((k, self.marks.get(k)))
            self.marks[k] = row
            self._maybe_add_checks()
            self._save()
            self._update_seed()
            return self._next_after(k)

    def _next_after(self, k):
        for j in list(range(k + 1, len(self.items))) + list(range(0, k + 1)):
            if j not in self.marks:
                return j
        return min(k + 1, len(self.items) - 1)

    def undo_last(self):
        with self.lock:
            if not self.undo:
                return None
            k, prev = self.undo.pop()
            if prev is None:
                self.marks.pop(k, None)
            else:
                self.marks[k] = prev
            self._save()
            self._update_seed()
            return k

    def _maybe_add_checks(self):
        """Once every picture is marked: show a few again with the ring put off on purpose (how exactly do marks
        repeat?)."""
        if self.check_share <= 0 or any(i["kind"] == "check" for i in self.items):
            return
        if any(k not in self.marks for k in range(len(self.picks))):
            return
        cands = [k for k in range(len(self.picks)) if self.marks[k]["status"] in TRUTH_STATUSES]
        if len(cands) < 10:
            return
        n = max(3, int(round(self.check_share * len(cands))))
        rnd = random.Random("%s-%d" % (self.name or self.robot, len(self.picks)))
        chosen = sorted(rnd.sample(cands, min(n, len(cands))))
        for k in chosen:
            r = self.marks[k]
            u, v = float(r["u_px"]), float(r["v_px"])
            off = CHECK_OFFSET_CM * self.calib.ring_px(u, v) / ROBOT_CM
            a = rnd.uniform(0, 2 * math.pi)
            self.items.append(dict(kind="check", pick=k, frame=self.picks[k], orig_u=u, orig_v=v,
                                   start_u=u + off * math.cos(a), start_v=v + off * math.sin(a)))

    # --- results -----------------------------------------------------------------------------------
    def summary_short(self):
        rows = [r for r in self.marks.values() if r["kind"] == "pick" and r["status"] in TRUTH_STATUSES]
        n = len(rows)
        asis = sum(1 for r in rows if r["status"] == "confirmed")
        d = [float(r["guess_diff_cm"]) for r in rows if r["guess_diff_cm"] != ""]
        return dict(n=n, as_is=asis, med_cm=round(float(np.median(d)), 1) if d else None,
                    minutes=round(self.spent() / 60.0, 1))

    def summary(self):
        picks = [r for r in self.marks.values() if r["kind"] == "pick"]
        truth = [r for r in picks if r["status"] in TRUTH_STATUSES]
        counts = {s: sum(1 for r in picks if r["status"] == s) for s in MARK_STATUSES}

        def stats(vals, limit=5.0):
            if not vals:
                return None
            a = np.array(vals)
            return dict(n=len(vals), median=round(float(np.median(a)), 2), p90=round(float(np.percentile(a, 90)), 2),
                        max=round(float(a.max()), 2), share_over_5cm=round(float(np.mean(a > limit)), 3))

        checks = [r for r in self.marks.values() if r["kind"] == "check" and r["repeat_diff_cm"] != ""]
        rep = [float(r["repeat_diff_cm"]) for r in checks]
        return dict(
            what="Hand-checked robot positions (tools/label/mark_robot.py): one picture every %g s; marks on the "
                 "plane of the robot's top (%.1f cm), floor cm from the calibration below" % (self.every, ROBOT_TOP_CM),
            stretch=self.name, title=self.title, robot=self.robot, t0=self.t0, t1=self.t1,
            clock_from=clock(self.t0), clock_to=clock(self.t1), source=self.source,
            calibration=self.calib.path, n_picks=len(self.picks), marked=len(picks),
            done=len(picks) >= len(self.picks), counts=counts,
            guess_help=dict(
                note="how good the help was: the ring's start against your mark (the tracker was restarted from "
                     "your corrections, so this is NOT the tracker's own error)",
                confirmed_as_is=counts["confirmed"], corrected=counts["corrected"],
                diff_cm=stats([float(r["guess_diff_cm"]) for r in truth if r["guess_diff_cm"] != ""])),
            runner=stats([float(r["runner_diff_cm"]) for r in truth if r["runner_diff_cm"] != ""]),
            repeat_checks=dict(note="the same picture marked twice (the second time the ring started %.0f cm off): "
                                    "how exactly marks repeat" % CHECK_OFFSET_CM,
                               **(stats(rep) or dict(n=0))),
            minutes_spent=round(self.spent() / 60.0, 1),
            files=dict(marks="marks.csv (one row per picture: kind pick = the truth, kind check = a repeat)",
                       read_with="tools/label/mark_robot.py load_truth() and score_track()"),
            written=dt.datetime.now().isoformat(timespec="seconds"))

    def close(self):
        if self.guesser:
            self.guesser.stop()


# ============================================================================ for the replay experiments
def load_truth(path, statuses=TRUTH_STATUSES):
    """The hand-checked positions of a marks folder (or its marks.csv): [dict(t, x, y, u, v, status, frame)], in
    time order. Only pictures marked with a position you were sure of (confirmed / corrected) by default."""
    p = os.path.join(path, "marks.csv") if os.path.isdir(path) else path
    out = []
    with open(p, newline="") as f:
        for r in csv.DictReader(f):
            if r["kind"] != "pick" or r["status"] not in statuses:
                continue
            out.append(dict(t=float(r["t_wall"]), x=_num(r["x_cm"]), y=_num(r["y_cm"]), u=_num(r["u_px"]),
                            v=_num(r["v_px"]), status=r["status"], frame=int(r["frame"])))
    out.sort(key=lambda d: d["t"])
    return out


def not_visible_times(path):
    p = os.path.join(path, "marks.csv") if os.path.isdir(path) else path
    with open(p, newline="") as f:
        return [float(r["t_wall"]) for r in csv.DictReader(f) if r["kind"] == "pick" and r["status"] == "not_visible"]


def score_track(samples, truth, limit_cm=5.0, max_dt=0.03):
    """A replayed track [(t, x, y or None)] against the hand-checked truth (load_truth): at each truth picture the
    track's position at that picture (nearest sample within max_dt s). Returns n, wrong (more than limit_cm off),
    missing (no position), and their shares: DOE-tracking.md's "wrong-position time" response."""
    ts = [s[0] for s in samples]
    import bisect
    n = wrong = missing = 0
    errs = []
    for d in truth:
        if d["x"] is None:
            continue
        j = bisect.bisect_left(ts, d["t"])
        cands = [i for i in (j - 1, j) if 0 <= i < len(ts) and abs(ts[i] - d["t"]) <= max_dt]
        if not cands:
            continue
        i = min(cands, key=lambda i: abs(ts[i] - d["t"]))
        n += 1
        if samples[i][1] is None:
            missing += 1
            continue
        e = math.hypot(samples[i][1] - d["x"], samples[i][2] - d["y"])
        errs.append(e)
        wrong += e > limit_cm
    return dict(n=n, wrong=wrong, missing=missing, share_wrong=wrong / n if n else None,
                share_missing=missing / n if n else None,
                median_cm=float(np.median(errs)) if errs else None)


def recompute_cm(folder, calib_path):
    """Fill in (or redo) the floor cm of a marks folder from its pixels with another calibration."""
    cal = Calib(calib_path)
    p = os.path.join(folder, "marks.csv")
    with open(p, newline="") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for pre in ("", "guess_"):
            u, v = _num(r[pre + "u_px"]), _num(r[pre + "v_px"])
            c = cal.to_cm(u, v) if u is not None and v is not None else None
            r[pre + "x_cm"], r[pre + "y_cm"] = (_f(c[0]), _f(c[1])) if c else ("", "")
        a, b = (_num(r["x_cm"]), _num(r["y_cm"])), (_num(r["guess_x_cm"]), _num(r["guess_y_cm"]))
        r["guess_diff_cm"] = _f(math.hypot(a[0] - b[0], a[1] - b[1])) if None not in a + b else ""
        if r["repeat_of_u_px"]:
            o = cal.to_cm(float(r["repeat_of_u_px"]), float(r["repeat_of_v_px"]))
            r["repeat_diff_cm"] = _f(math.hypot(a[0] - o[0], a[1] - o[1])) if o and None not in a else ""
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    return len(rows)


# ============================================================================ presets (the DOE's stretches)
def preset_info(name):
    p = PRESETS[name]
    with open(os.path.join(CLIPS_JSON, p["json"])) as f:
        m = json.load(f)
    return p, m


def preset_folder(name, root=None):
    return os.path.join(root or default_footage_root(), name)


def fetch_preset(name, root=None, log=print):
    p, m = preset_info(name)
    folder = preset_folder(name, root)
    if os.path.exists(os.path.join(folder, "stretch_index.csv")):
        return folder
    url = FT.RELEASE_URL % (p["release"], m["video"])
    furl = FT.RELEASE_URL % (p["release"], m["frames"])
    fl = float(m.get("floor_t_after_start", 30.0))
    meta = dict(name=name, title=p["title"], robot=p["robot"], start_t=m["start_t"], end_t=m["end_t"], at=m["at"],
                others=m.get("others", []), legs=m.get("legs", []), board_sides=list(p["board_sides"]),
                floor_after_start_s=fl, release=p["release"], video=m["video"], frames=m["frames"],
                note=m.get("note", ""))
    FT.fetch(folder, [url], furl, m["start_t"], m["end_t"], extra_ranges=[("start", fl, fl + 0.6)], meta=meta,
             log=log)
    return folder


REPO_SLUG = "nfmaxfield-cmd/alvik-fleet-capstone"


def release_assets(tag, opener=None):
    """{name: download url} of a release's files (public repository: no token needed)."""
    op = opener or (lambda req: urllib.request.urlopen(req, timeout=60))
    req = urllib.request.Request("https://api.github.com/repos/%s/releases/tags/%s" % (REPO_SLUG, tag),
                                 headers={"Accept": "application/vnd.github+json", "User-Agent": "mirlt-mark-robot"})
    with op(req) as r:
        d = json.loads(r.read())
    return {x["name"]: x["browser_download_url"] for x in d.get("assets", [])}


def pick_release_files(names, video=None, frames=None):
    """From a release's file names: the overhead video (whole, or its parts in order), where frames.csv is
    (a file, or inside the recording's small zip), and the logs zip. Session releases name them video.avi.part-aa
    ... (Pi upload) or video.part-aa (laptop); frames.csv sits in rec-<...>-small.zip."""
    vids = video.split(",") if video else sorted(n for n in names if re.fullmatch(r"video(\.avi)?(\.part-[a-z]{2})?", n))
    if not vids:
        raise SystemExit("No video in this release (files: %s). Give --video." % ", ".join(sorted(names)))
    if len(vids) > 1 and "video.avi" in vids:
        vids = ["video.avi"]
    fr = frames or ("frames.csv" if "frames.csv" in names else None)
    zipped = None
    if fr is None:
        zs = sorted(n for n in names if n.startswith("rec-") and n.endswith("-small.zip"))
        zipped = zs[0] if zs else None
        if zipped is None:
            raise SystemExit("No frames.csv in this release. Give --frames-name.")
    logs = sorted(n for n in names if n.startswith("logs") and n.endswith(".zip"))
    return vids, fr, zipped, (logs[0] if logs else None)


def fetch_release(tag, a, root=None, log=print, opener=None):
    """Download --from to --to of a session release's video into a stretch folder (plus its logs, so the runner's
    own positions and the legs can be shown). Returns the folder."""
    op = opener or (lambda req: urllib.request.urlopen(req, timeout=120))
    assets = release_assets(tag, opener)
    vids, fr, zipped, logs = pick_release_files(set(assets), getattr(a, "video", None),
                                                getattr(a, "frames_name", None))
    root = root or default_footage_root()
    tmp = os.path.join(root, tag + "_partial")
    os.makedirs(tmp, exist_ok=True)
    fpath = os.path.join(tmp, "frames.csv")
    if not os.path.exists(fpath):
        log("Getting the recording's frames.csv ...")
        with op(urllib.request.Request(assets[zipped or fr])) as r:
            blob = r.read()
        if zipped:
            z = zipfile.ZipFile(io.BytesIO(blob))
            cands = sorted((n for n in z.namelist() if n.endswith("/frames.csv") or n == "frames.csv"),
                           key=lambda n: n.count("/"))
            if not cands:
                raise SystemExit("No frames.csv inside %s." % zipped)
            blob = z.read(cands[0])
        with open(fpath, "wb") as f:
            f.write(blob)
    times = FT.read_frame_times(fpath)
    first = min(times.values())
    t0, t1 = parse_time(a.t_from, first), parse_time(a.t_to, first)
    robot = getattr(a, "robot", None) or "robot"
    name = "%s_%s_%s-%s" % (tag, robot.replace(" ", ""), clock(t0).replace(":", ""), clock(t1).replace(":", ""))
    folder = os.path.join(root, name)
    if os.path.exists(os.path.join(folder, "stretch_index.csv")):
        return folder
    os.makedirs(folder, exist_ok=True)
    os.replace(fpath, os.path.join(folder, "frames.csv"))
    meta = dict(name=name, title="%s, %s %s-%s" % (robot, tag, clock(t0), clock(t1)), robot=robot, start_t=t0,
                end_t=t1, release=tag, video=vids, frames=zipped or fr)
    if getattr(a, "at", None):
        meta["at"] = [float(v) for v in a.at.split(",")]
    if logs:
        log("Getting the session's logs (%s) ..." % logs)
        with op(urllib.request.Request(assets[logs])) as r:
            z = zipfile.ZipFile(io.BytesIO(r.read()))
        sd = os.path.join(folder, "session")
        for n in z.namelist():
            parts = n.split("/")
            if n.endswith("/") or ".." in parts:
                continue
            if parts[-1] in ("track.csv", "leg.csv", "e02_state.json") or n.endswith(".json") and "cameras" in parts:
                dst = os.path.join(sd, *parts[1:]) if len(parts) > 1 else os.path.join(sd, parts[0])
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                with open(dst, "wb") as f:
                    f.write(z.read(n))
        meta["session_dir"] = "session"
    FT.fetch(folder, [assets[v] for v in vids], None, t0, t1, meta=meta, log=log, opener=opener)
    try:
        os.rmdir(tmp)
    except OSError:
        pass
    return folder


def session_for_folder(folder, a=None, out=None, guesses=True):
    """A Session for a fetched stretch folder (stretch.json says what it is) or anything open_store reads."""
    import e02_plan as PL
    a = a or argparse.Namespace()
    store = FT.open_store(folder, getattr(a, "frames", None), getattr(a, "fps", None), getattr(a, "first_frame", None))
    meta = getattr(store, "meta", {}) or {}
    first_t = float(meta.get("recording_start_t", store.t(store.frames[0])))   # the recording's first picture
    t0 = parse_time(a.t_from, first_t) if getattr(a, "t_from", None) else meta.get("start_t", first_t)
    t1 = parse_time(a.t_to, first_t) if getattr(a, "t_to", None) else meta.get("end_t", store.t(store.frames[-1]))
    robot = getattr(a, "robot", None) or meta.get("robot") or "the robot"
    calib_path = getattr(a, "calib", None)
    session_dir = getattr(a, "session", None) or meta.get("session_dir")
    if session_dir and not os.path.isabs(session_dir) and os.path.isdir(os.path.join(folder, session_dir)):
        session_dir = os.path.join(folder, session_dir)
    sess_st = {}
    if session_dir and os.path.exists(os.path.join(session_dir, "e02_state.json")):
        with open(os.path.join(session_dir, "e02_state.json")) as f:
            sess_st = json.load(f)
    if not calib_path and session_dir:
        import e02_camera as CAM
        c = sess_st.get("calibration")                 # what the runner used (a from-scratch session: its own)
        for cand in ([c, os.path.join(REPO, c), os.path.join(session_dir, c)] if c else []):
            if cand and os.path.exists(cand):
                calib_path = cand
                break
        calib_path = calib_path or CAM.find_calibration(None, None, session_dir)
    if not calib_path and not getattr(a, "no_calib", False) and os.path.exists(DEFAULT_CALIB):
        calib_path = DEFAULT_CALIB
    calib = Calib(calib_path, getattr(a, "robot_px", None))
    seed = None
    if getattr(a, "at", None):
        seed = tuple(float(v) for v in a.at.split(","))
    elif meta.get("at") and abs(meta.get("start_t", -1) - t0) < 0.6:
        seed = tuple(meta["at"])
    others = [tuple(float(v) for v in p.split(",")) for p in (getattr(a, "others", "") or "").split(";") if p.strip()] \
        or [tuple(o) for o in meta.get("others", [])]
    legs = list(meta.get("legs", []))
    off = getattr(a, "clock_offset", None)
    if off is None:                                    # Pi time = laptop time - pi_offset_s (e02.py)
        po = (sess_st.get("setup") or {}).get("pi_offset_s")
        off = -float(po) if po is not None else 0.0
    off = float(off)
    leg_files = [a.legs] if getattr(a, "legs", None) else (
        sorted(glob.glob(os.path.join(session_dir, "robot-logs-*", "leg.csv"))) if session_dir else [])
    if leg_files:
        legs = sorted(g for p in leg_files for g in read_legs(p, robot, off))
    runner = []
    track = getattr(a, "runner_track", None) or (os.path.join(session_dir, "track.csv") if session_dir else None)
    if track and os.path.exists(track):
        runner = [(float(r["t"]) + off, float(r["x_cm"]), float(r["y_cm"])) for r in csv.DictReader(open(track))
                  if r.get("robot") == robot and r.get("x_cm")]
    if seed is None and runner:                        # troubleshooting: start from where the runner thought it was
        near = min(runner, key=lambda r: abs(r[0] - t0))
        if abs(near[0] - t0) <= 1.0:
            seed = (near[1], near[2])
    sides = meta.get("board_sides")
    board = PL.Board(xboard_side=sides[0], yboard_side=sides[1]) if sides else PL.Board()
    floor = []
    if meta.get("floor_after_start_s") is not None:
        ft = first_t + float(meta["floor_after_start_s"])
        floor = [f for f in store.frames if ft <= store.t(f) <= ft + 0.6][:12]
    name = meta.get("name") or "%s_%s-%s" % (robot.replace(" ", ""), clock(t0).replace(":", ""),
                                              clock(t1).replace(":", ""))
    if out is None:
        out = getattr(a, "out", None) or (os.path.join(MARKS_ROOT, name) if meta.get("name") else
                                          os.path.join(folder if os.path.isdir(folder) else os.path.dirname(folder),
                                                       "marks", name))
    return Session(store, calib, robot, t0, t1, out, every=getattr(a, "every", 1.0) or 1.0, seed=seed, legs=legs,
                   others=others, board=board, walls=bool(sides), floor_frames=floor, runner=runner,
                   checks=getattr(a, "checks", CHECK_SHARE), name=name, title=meta.get("title", name),
                   guesses=guesses, source=dict(footage=os.path.abspath(folder), **{k: meta[k] for k in
                                                ("release", "video", "frames") if k in meta}))


def read_legs(path, robot, offset=0.0):
    """[start, end, kind] of a robot's legs from a leg.csv: the runner's (robot, t_recv, id, result, kind) as
    e02_replay.legs_moving reads it. offset: added to the times (laptop clock -> video clock)."""
    spans, open_ = [], {}
    for r in csv.DictReader(open(path)):
        if r.get("robot") != robot:
            continue
        t, lid, res = float(r["t_recv"]) + offset, r["id"], r["result"]
        if res in ("queued", "started"):
            open_.setdefault(lid, (t, r.get("kind") or "go"))
        elif res in ("done", "failed", "refused", "dup") and lid in open_:
            t_a, kind = open_.pop(lid)
            spans.append([t_a, t, kind if kind in ("turn", "go") else "go"])
    return spans


# ============================================================================ the web page
class App:
    """The server's state: the open session, or the home list of stretches while none is open."""

    def __init__(self, session=None, root=None):
        self.session = session
        self.root = root
        self.lock = threading.RLock()
        self.job = None             # dict(name, log, error, done) while a stretch downloads

    def state(self):
        """The open session's state, or the home list (decided under the lock: never the list of a job that has
        just finished while its session is being attached)."""
        with self.lock:
            if self.session is not None:
                return self.session.state()
            return self.home()

    def home(self):
        out = []
        for name, p in PRESETS.items():
            folder = preset_folder(name, self.root)
            have = os.path.exists(os.path.join(folder, "stretch_index.csv"))
            marks = os.path.join(MARKS_ROOT, name, "summary.json")
            st = None
            if os.path.exists(marks):
                with open(marks) as f:
                    sm = json.load(f)
                st = dict(marked=sm.get("marked", 0), n=sm.get("n_picks", 0), done=sm.get("done", False))
            _, m = preset_info(name)
            out.append(dict(name=name, title=p["title"], robot=p["robot"], seconds=round(m["end_t"] - m["start_t"]),
                            downloaded=have, marks=st))
        job = dict(self.job) if self.job else None
        return dict(home=True, stretches=out, job=job, folder=self.root or default_footage_root())

    def open_preset(self, name):
        if name not in PRESETS:
            raise ValueError("no stretch %r" % name)
        with self.lock:
            if self.job and not self.job.get("done"):
                return
            self.job = dict(name=name, log=[], error=None, done=False)

        def work():
            job = self.job

            def log(s):
                print(s, flush=True)
                job["log"] = (job["log"] + [s])[-6:]
            try:
                folder = fetch_preset(name, self.root, log)
                log("Working out the first guesses ...")
                sess = session_for_folder(folder)
                with self.lock:
                    if self.session:
                        self.session.close()
                    self.session = sess
                    job["done"] = True
            except BaseException as e:      # noqa: BLE001 (SystemExit from fetch too)
                job["error"] = "%s" % (e,)
                job["done"] = True
        threading.Thread(target=work, daemon=True).start()

    def go_home(self):
        with self.lock:
            if self.session:
                self.session.close()
            self.session = None
            self.job = None


def make_handler(app):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json", cache=False):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "max-age=3600" if cache else "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            s = app.session
            try:
                if u.path in ("/", "/index.html"):
                    with open(PAGE, "rb") as f:
                        return self._send(200, f.read(), "text/html; charset=utf-8")
                if u.path == "/api/state":
                    return self._send(200, app.state())
                if s is None:
                    return self._send(404, dict(error="no stretch open"))
                if u.path == "/api/item":
                    k = int(q.get("k", ["0"])[0])
                    return self._send(200, s.item_info(max(0, min(k, len(s.items) - 1))))
                if u.path.startswith("/img/"):
                    f = int(u.path[5:].split(".")[0])
                    b = s.store.jpeg(f)
                    return self._send(200 if b else 404, b or b"", "image/jpeg", cache=True)
                if u.path == "/api/between":
                    a, b = int(q["a"][0]), int(q["b"][0])
                    fr = [f for f in s.store.frames if a < f <= b]
                    step = max(1, len(fr) // 12)
                    return self._send(200, dict(frames=fr[::step][-12:]))
                if u.path == "/api/calgrid":
                    st = s.state()
                    return self._send(200, s.calib.grid(st["width"], st["height"]) or {}, cache=True)
                if u.path == "/api/summary":
                    return self._send(200, s.summary())
                return self._send(404, dict(error="not found"))
            except Exception as e:      # noqa: BLE001
                return self._send(500, dict(error="%s: %s" % (type(e).__name__, e)))

        def do_POST(self):
            u = urlparse(self.path)
            n = int(self.headers.get("Content-Length") or 0)
            try:
                d = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                d = {}
            s = app.session
            try:
                if u.path == "/api/open":
                    app.open_preset(d.get("name"))
                    return self._send(200, dict(ok=True))
                if u.path == "/api/home":
                    app.go_home()
                    return self._send(200, dict(ok=True))
                if s is None:
                    return self._send(404, dict(error="no stretch open"))
                if u.path == "/api/mark":
                    nxt = s.mark(int(d["k"]), d["status"], d.get("u"), d.get("v"), d.get("moved_px"),
                                 d.get("seconds"))
                    return self._send(200, dict(next=nxt, state=s.state()))
                if u.path == "/api/undo":
                    k = s.undo_last()
                    return self._send(200, dict(k=k, state=s.state()))
                return self._send(404, dict(error="not found"))
            except Exception as e:      # noqa: BLE001
                return self._send(400, dict(error="%s: %s" % (type(e).__name__, e)))
    return H


def lan_address():
    try:
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.connect(("10.255.255.255", 1))
        ip = sk.getsockname()[0]
        sk.close()
        return ip
    except OSError:
        return None


def serve(app, port=PORT, phone=False, browser=True):
    host = "0.0.0.0" if phone else "127.0.0.1"
    for p in range(port, port + 10):
        try:
            srv = ThreadingHTTPServer((host, p), make_handler(app))
            break
        except OSError:
            continue
    else:
        raise SystemExit("Ports %d-%d are all busy: close the other window, or give --port" % (port, port + 9))
    url = "http://localhost:%d" % p
    print("\nMark the robot is running. Open %s in the browser (it opens by itself)." % url)
    if phone:
        ip = lan_address()
        if ip:
            print("On a phone on the same WiFi: http://%s:%d" % (ip, p))
    print("Leave this window open while you mark. To stop: close this window (or press Ctrl+C).\n", flush=True)
    if browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if app.session:
            app.session.close()
        srv.server_close()


# ============================================================================ command line
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", nargs="?", help="a stretch name (%s), a recording folder, a fetched stretch folder, a "
                                            "folder of pictures or a video; nothing: the list of stretches"
                    % ", ".join(PRESETS))
    ap.add_argument("--from", dest="t_from", help="start: HH:MM:SS clock time, +M:SS after the recording's start, "
                                                  "or Unix seconds")
    ap.add_argument("--to", dest="t_to", help="end (same forms)")
    ap.add_argument("--robot", help="the robot's name (shown on the page and saved)")
    ap.add_argument("--at", help="where the robot is at --from, floor cm x,y (otherwise your first mark starts the "
                                 "tracker)")
    ap.add_argument("--others", default="", help="robots standing still: x,y;x,y (floor cm)")
    ap.add_argument("--every", type=float, default=1.0, help="seconds between pictures (default 1)")
    ap.add_argument("--frames", help="frames.csv, when it is not next to the video")
    ap.add_argument("--fps", type=float, help="pictures per second, when there is no frames.csv")
    ap.add_argument("--first-frame", type=int, help="for a piece of a split video: its first picture's frame number")
    ap.add_argument("--calib", help="floor calibration (default: the session's own, else %s)"
                    % os.path.relpath(DEFAULT_CALIB, REPO))
    ap.add_argument("--no-calib", action="store_true", help="pixels only (fill in cm later with --recompute-cm)")
    ap.add_argument("--robot-px", type=float, help="a robot's width in pixels (only without a calibration)")
    ap.add_argument("--session", help="the runner's session folder (troubleshooting): its track.csv shows where the "
                                      "runner thought the robot was; its calibration is used")
    ap.add_argument("--runner-track", help="a track.csv to compare with (default: the --session's)")
    ap.add_argument("--legs", help="leg.csv (robot, t_recv, id, result, kind): when the robot drove")
    ap.add_argument("--clock-offset", type=float, default=None,
                    help="seconds to add to the session's (laptop) times to get the video's (Pi) clock (default: "
                         "from the session's e02_state.json, else 0)")
    ap.add_argument("--release", help="a GitHub release of a session (e.g. e02-session-20261024-193526): download "
                                      "only --from to --to of its video (and its logs, for troubleshooting)")
    ap.add_argument("--video", help="with --release: the video's file name(s), comma-separated parts (default: "
                                    "found by itself)")
    ap.add_argument("--frames-name", help="with --release: the frames.csv's name in the release (default: found)")
    ap.add_argument("--out", help="where to save the marks")
    ap.add_argument("--checks", type=float, default=CHECK_SHARE, help="share of pictures shown again as repeat "
                                                                      "checks (0 = none)")
    ap.add_argument("--footage", help="where downloaded stretches go (default: Documents\\mirlt-footage)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--phone", action="store_true", help="also reachable from a phone on the home WiFi")
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--get-only", action="store_true", help="only download the stretch, then stop")
    ap.add_argument("--recompute-cm", metavar="MARKS_FOLDER", help="fill in the cm of a marks folder from --calib")
    a = ap.parse_args(argv)
    if a.recompute_cm:
        if not a.calib:
            raise SystemExit("Give the calibration with --calib")
        print("Recomputed %d rows." % recompute_cm(a.recompute_cm, a.calib))
        return
    app = App(root=a.footage)
    if a.release:
        if not (a.t_from and a.t_to):
            raise SystemExit("With --release, give --from and --to (the stretch to download).")
        folder = fetch_release(a.release, a, a.footage)
        if a.get_only:
            return
        app.session = session_for_folder(folder, a)
    elif a.what in PRESETS:
        folder = fetch_preset(a.what, a.footage)
        if a.get_only:
            return
        app.session = session_for_folder(folder, a)
    elif a.what:
        if not os.path.exists(a.what):
            raise SystemExit("Not found: %s\nGive a stretch name (%s) or a folder / video." % (a.what,
                                                                                         ", ".join(PRESETS)))
        app.session = session_for_folder(a.what, a)
    serve(app, a.port, a.phone, not a.no_browser)


if __name__ == "__main__":
    main()
