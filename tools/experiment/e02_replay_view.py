#!/usr/bin/env python3
"""REPLAY mode for the E02 live view (live-lab outline, modes: "plays a recorded session through the same screen:
the fallback if the live link fails on the day"; 2026-10-22). Plays a runner session folder and its overhead
recording through the same browser pages as a live session (http://localhost:<port>/ and /lab), in real time
(or 2x / 4x), with a pause / speed / jump bar and a red "REPLAY of <date time>" badge on every page and picture.

    python tools/experiment/e02_replay_view.py --session logs/e02-20261022-052003-dryrun \
        --recording logs/e02-20261022-052003-dryrun/sim/recording [--from 05:25:00] [--speed 2] [--port 8090]

--recording: a folder with the video (video.avi or any .avi; also looked for in cam0/) and frames.csv (t_wall per
frame), or the video file itself. A video cut from a longer recording (setup.avi) starts later than frames.csv:
--first-frame is the recording's frame number of the video's first frame (default: the cut is taken to run to
the end of the recording, i.e. rows of frames.csv minus frames in the video).

What the page shows, from the session's own records, revealed only up to the replay time:
  - the picture: the recording's frame at the replay time;
  - the followed robot (green ring), phase and run: track.csv (the robot the camera followed, and its phase);
  - the runner's lines: e02_log.txt (the lines the console printed; the robot commands it logged quietly are left
    out, as live);
  - results and self-calibration: e02_state.json with each run's results shown from its t_end, fits from each
    history entry's t, the robot map from its t; robot labels: each robot's latest camera position (track.csv,
    or its identify fix), the saved positions once the session is over;
  - the Live lab's filter: fed the robots' RAW rows (robot-logs-<n>/raw.csv) and the camera samples (track.csv)
    at their times.
Not recorded, so not shown: the hold circle and the yellow / grey change dots (the tracker's working state).
"""
import argparse
import bisect
import csv
import datetime as dt
import glob
import json
import ntpath
import os
import re
import sys
import threading
import time
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import e02_camera as CAM  # noqa: E402
import e02_live as LIVE  # noqa: E402
import e02_plan as PL  # noqa: E402
import e02_view as VIEW  # noqa: E402

PLAN_KEYS = ("cond", "route", "robot", "order", "block")
# lines the runner logged with say(quiet=True): in e02_log.txt but never on the console or the live view
QUIET = re.compile(r"^(    -> |    \(\S+ did not turn: |    \(two places changed when |    \S+: going round another robot|"
                   r"  \(map picture failed|  \(watchdog: |\(live lab not started)")
LOG_LINE = re.compile(r"^\[(\d\d):(\d\d):(\d\d)\] ?(.*)$")
TIMED_LISTS = ("notes", "calibrations", "identify", "board_checks", "help_calls", "watchdog_stops")
RAW_KEYS = ("run", "v_cms", "w_dps", "gz", "x_cm", "y_cm", "th_deg")


# =============================================================================== session times
def session_start(session_dir):
    """(epoch, tzinfo) of the session's start: the robot logs' session.json (it carries the laptop's UTC offset),
    else the folder name in this computer's time zone."""
    for p in sorted(glob.glob(os.path.join(session_dir, "robot-logs-*", "session.json"))):
        try:
            d = json.load(open(p))
            start = dt.datetime.fromisoformat(d["start"])
            if start.tzinfo is not None:
                return float(d.get("start_unix", start.timestamp())), start.tzinfo
        except (OSError, ValueError, KeyError):
            pass
    m = re.search(r"(\d{8})-(\d{6})", os.path.basename(os.path.normpath(session_dir)))
    tz = dt.datetime.now().astimezone().tzinfo
    if m:
        t = dt.datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S").replace(tzinfo=tz)
        return t.timestamp(), tz
    return os.path.getmtime(os.path.join(session_dir, "e02_state.json")), tz


def parse_log(lines, start_epoch, tz):
    """e02_log.txt lines -> [(t, line, quiet)]. Each line's [HH:MM:SS] is taken on the session's day, moving on a
    day when the clock goes back past midnight; a line without a time gets the one before."""
    day = dt.datetime.fromtimestamp(start_epoch, tz).date()
    out, last = [], start_epoch
    for raw in lines:
        line = raw.rstrip("\n").rstrip("\r")
        if not line.strip():
            continue
        m = LOG_LINE.match(line)
        if m:
            h, mi, s, msg = int(m.group(1)), int(m.group(2)), int(m.group(3)), m.group(4)
            t = dt.datetime(day.year, day.month, day.day, h, mi, s, tzinfo=tz).timestamp()
            while t < last - 12 * 3600:
                day += dt.timedelta(days=1)
                t = dt.datetime(day.year, day.month, day.day, h, mi, s, tzinfo=tz).timestamp()
            last = t
            out.append((t, line, bool(QUIET.match(msg))))
        else:
            out.append((last, line, False))
    return out


def say_lines(log, t, keep=14, times=None):
    """What the live view's log box showed at time t: the last `keep` console lines up to t."""
    k = bisect.bisect_right(times if times is not None else [e[0] for e in log], t)
    shown = []
    while k > 0 and len(shown) < keep:
        k -= 1
        if not log[k][2]:
            shown.append(log[k][1])
    return shown[::-1]


# =============================================================================== the state, as it was at time t
def gate_state(st, t, track_last=None, session_over=False):
    """The runner's state file as it stood at time t (approximately: from the times it records).
    track_last: robot -> (t, x, y) of its latest camera position up to t (for the robot labels)."""
    out = {k: v for k, v in st.items() if k not in ("schedule", "fits", "fit_data", "positions", "setup") + TIMED_LISTS}
    sched = []
    for r in st.get("schedule") or []:
        ts, te = r.get("t_start"), r.get("t_end")
        if ts is None or t < ts:
            q = {k: r[k] for k in PLAN_KEYS if k in r}
            q["status"] = "pending"
        elif te is None or t < te:
            q = {k: r[k] for k in PLAN_KEYS if k in r}
            q.update(status="running", t_start=ts)
        else:
            q = dict(r)
        sched.append(q)
    out["schedule"] = sched
    fits = {}
    for robot, f in (st.get("fits") or {}).items():
        hist = [h for h in f.get("history", []) if h.get("t") is not None and h["t"] <= t]
        if not hist:
            continue
        g = {k: v for k, v in f.items() if k not in ("history", "values")}
        g["history"] = hist
        acc = [h for h in hist if h.get("accepted") and h.get("values")]
        if acc:
            g["values"] = acc[-1]["values"]
        fits[robot] = g
    out["fits"] = fits
    out["fit_data"] = {robot: [o for o in obs if o.get("tm") is not None and o["tm"] <= t]
                       for robot, obs in (st.get("fit_data") or {}).items()}
    for k in TIMED_LISTS:
        if k in st:
            out[k] = [e for e in st[k] or [] if isinstance(e, (list, tuple)) and e and _num(e[0]) is not None
                      and e[0] <= t]
    if "durations" in st:
        out["durations"] = list(st["durations"] or [])[:sum(1 for r in sched if r["status"] == "done")]
    setup = dict(st.get("setup") or {})
    m = setup.get("map")
    if m and (_num(m.get("t")) is None or float(m["t"]) > t):
        setup.pop("map")
    out["setup"] = setup
    if session_over:
        out["positions"] = dict(st.get("positions") or {})
    else:
        pos = {}
        for e in out.get("identify", []):              # [t, robot, radius, [[x, y, ...]]]
            if len(e) >= 4 and e[3]:
                pos[e[1]] = (e[0], e[3][0][0], e[3][0][1])
        for robot, q in (track_last or {}).items():
            if robot not in pos or q[0] >= pos[robot][0]:
                pos[robot] = q
        out["positions"] = {robot: [q[1], q[2]] for robot, q in pos.items()}
    return out


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


# =============================================================================== the replay clock
class Clock:
    """Replay time: runs from t0 to t1 at `speed` x real time; pause, speed and jump."""

    def __init__(self, t0, t1, start=None, speed=1.0, wall=time.monotonic):
        self.t0, self.t1 = t0, t1
        self.wall = wall
        self.speed = float(speed)
        self.paused = False
        self.lock = threading.Lock()
        self._base_t = min(max(start if start is not None else t0, t0), t1)
        self._base_w = wall()

    def now(self):
        with self.lock:
            if self.paused:
                return self._base_t
            t = self._base_t + self.speed * (self.wall() - self._base_w)
            if t >= self.t1:                       # the end: stop there
                self._base_t, self._base_w, self.paused = self.t1, self.wall(), True
                return self.t1
            return t

    def set(self, speed=None, paused=None, jump=None):
        now = self.now()
        with self.lock:
            self._base_t = now if jump is None else min(max(float(jump), self.t0), self.t1)
            self._base_w = self.wall()
            if speed is not None:
                self.speed = float(speed)
            if paused is not None:
                self.paused = bool(paused)
                if not paused and self._base_t >= self.t1:
                    self._base_t = self.t0             # play again from the start
        return self.now()


# =============================================================================== stand-ins for the runner's parts
class Feed:
    def __init__(self):
        self.last_img = None


class Tracker:
    """What the live view reads from the runner's MotionTracker, from track.csv."""

    def __init__(self, rpx):
        self.rpx = rpx
        self.active = None
        self.hint = None
        self.last_cands = []
        self.pos = None

    def where(self):
        return self.pos


class ReplayLab(LIVE.LiveLab):
    """The Live lab filter, asked about the replay time rather than the wall clock."""

    def __init__(self, clock, **kw):
        super().__init__(**kw)
        self.clock = clock

    def state(self, now=None):
        return super().state(self.clock.now() if now is None else now)


# =============================================================================== the video
class Video:
    """Frames of the overhead recording by wall time (frames.csv t_wall); first: frames.csv row of video frame 0."""

    def __init__(self, video, frames_csv, first_frame=None):
        self.cap = cv2.VideoCapture(video)
        if not self.cap.isOpened():
            raise SystemExit("Can't open the video %s" % video)
        self.ts = [float(r["t_wall"]) for r in csv.DictReader(open(frames_csv))]
        self.n = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if first_frame is None:
            first_frame = max(0, len(self.ts) - self.n)
        self.first = first_frame
        self.n = min(self.n, len(self.ts) - self.first)
        self.folder = os.path.dirname(os.path.abspath(frames_csv))
        self.pos = None                     # index of the next frame cap.read() returns
        self.cur, self.img = None, None
        self.set_offset(0.0, "none")

    def set_offset(self, off, source):
        """off: camera (Pi) clock minus laptop clock, s. t_first / t_last are on the laptop's clock."""
        self.offset, self.offset_source = float(off), source
        self.t_first = self.ts[self.first] - self.offset
        self.t_last = self.ts[self.first + self.n - 1] - self.offset

    def index(self, t):
        """Video frame shown at laptop time t (the latest one taken at or before t), or None outside the recording."""
        if t < self.t_first - 0.5 or t > self.t_last + 0.5:
            return None
        return max(0, min(self.n - 1, bisect.bisect_right(self.ts, t + self.offset) - 1 - self.first))

    def frame(self, t):
        i = self.index(t)
        if i is None:
            return None
        if i == self.cur:
            return self.img
        if self.pos is not None and 0 <= i - self.pos <= 60:
            for _ in range(i - self.pos):
                self.cap.grab()
        else:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, f = self.cap.read()
        self.pos = i + 1 if ok else None
        if not ok:
            return self.img
        self.cur, self.img = i, (cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) if f.ndim == 3 else f)
        return self.img


def clock_offset(st, rec_folder):
    """(camera clock minus laptop clock in s, where it came from): the recording's video_clock.json (offset_s), else
    the runner's setup.pi_offset_s (Pi time = laptop time - pi_offset_s), else the video marks: each note the runner
    sent (e02_state.json notes, laptop time) against the time the Pi heard it (events.csv), the median; else 0."""
    p = os.path.join(rec_folder, "video_clock.json")
    if os.path.exists(p):
        try:
            return float(json.load(open(p))["offset_s"]), "video_clock.json"
        except (OSError, ValueError, KeyError):
            pass
    po = _num((st.get("setup") or {}).get("pi_offset_s"))
    if po is not None:
        return -po, "e02_state.json setup.pi_offset_s"
    p = os.path.join(rec_folder, "events.csv")
    if os.path.exists(p) and st.get("notes"):
        ev = {}
        for r in csv.DictReader(open(p, newline="")):
            t = _num(r.get("t_wall"))
            if t is not None:
                ev.setdefault(r.get("text", ""), []).append(t)
        d = []
        for e in st["notes"]:
            if len(e) >= 2 and _num(e[0]) is not None and e[1] in ev:
                best = min((te - e[0] for te in ev[e[1]]), key=abs)
                if abs(best) < 120:
                    d.append(best)
        if d:
            return float(np.median(d)), "video marks (%d notes matched in events.csv)" % len(d)
    return 0.0, "none found: the camera's clock is taken to agree with the laptop's"


def find_recording(path, frames=None):
    """(video, frames.csv) from a recording folder (or its cam0/) or a video file."""
    if os.path.isfile(path):
        video = path
        folder = os.path.dirname(path)
    else:
        folder = path
        if not glob.glob(os.path.join(folder, "*.avi")) and os.path.isdir(os.path.join(folder, "cam0")):
            folder = os.path.join(folder, "cam0")
        vids = sorted(glob.glob(os.path.join(folder, "*.avi")))
        if not vids:
            raise SystemExit("No .avi video in %s" % path)
        video = os.path.join(folder, "video.avi") if os.path.join(folder, "video.avi") in vids else vids[0]
    frames = frames or os.path.join(folder, "frames.csv")
    if not os.path.exists(frames):
        raise SystemExit("No frames.csv next to %s (give --frames)" % video)
    return video, frames


# =============================================================================== the session
def load_track(session_dir):
    rows = []
    p = os.path.join(session_dir, "track.csv")
    if os.path.exists(p):
        for r in csv.DictReader(open(p, newline="")):
            t, x, y = _num(r.get("t")), _num(r.get("x_cm")), _num(r.get("y_cm"))
            if t is not None and x is not None and y is not None:
                rows.append((t, r.get("robot", ""), x, y, r.get("order", ""), r.get("phase", "")))
    rows.sort(key=lambda q: q[0])
    return rows


def load_raw(session_dir):
    rows = []
    for p in sorted(glob.glob(os.path.join(session_dir, "robot-logs-*", "raw.csv"))):
        for r in csv.DictReader(open(p, newline="")):
            t = _num(r.get("t_recv"))
            if t is not None:
                rows.append((t, r.get("robot", ""), {k: r.get(k) for k in RAW_KEYS}))
    rows.sort(key=lambda q: q[0])
    return rows


def find_calibration(st, session_dir, override=None):
    cands = [override] if override else []
    c = st.get("calibration")
    if c:
        base = ntpath.basename(c)                 # a Windows path from Nick's laptop
        cands += [c, os.path.join(session_dir, base), os.path.join(session_dir, "sim", base),
                  os.path.join(HERE, "data", base)]
    cands.append(os.path.join(HERE, "data", "overhead_floorcal_roam1.json"))
    for p in cands:
        if p and os.path.exists(p):
            return p
    return None


class ReplayRunner:
    """Looks like an e02.Runner to e02_view.LiveView (feed.last_img, tracker, st, cam, board, phase, order, lab),
    with everything as it stood at the replay clock's time. step() moves it to the clock's time."""

    def __init__(self, session_dir, video=None, start=None, speed=1.0, calib=None, wall=time.monotonic):
        self.dir = session_dir
        self.st_full = json.load(open(os.path.join(session_dir, "e02_state.json")))
        self.t_start, self.tz = session_start(session_dir)
        with open(os.path.join(session_dir, "e02_log.txt"), encoding="utf-8", errors="replace") as f:
            self.log = parse_log(f.readlines(), self.t_start, self.tz)
        self.log_t = [e[0] for e in self.log]
        self.track = load_track(session_dir)
        self.track_t = [q[0] for q in self.track]
        self.by_robot = {}
        for q in self.track:
            self.by_robot.setdefault(q[1], ([], []))
            self.by_robot[q[1]][0].append(q[0])
            self.by_robot[q[1]][1].append(q)
        self.raw = load_raw(session_dir)
        self.raw_t = [q[0] for q in self.raw]
        self.video = video
        if video is not None:
            video.set_offset(*clock_offset(self.st_full, video.folder))
        ends =[self.log[-1][0] + 1.0 if self.log else self.t_start]
        ends += [self.track_t[-1]] if self.track else []
        ends += [self.raw_t[-1]] if self.raw else []
        self.t_end = max(ends)
        t0 = min([self.t_start] + ([self.log[0][0]] if self.log else []))
        self.t_first = t0
        if start is None:
            start = max(t0, video.t_first) if video is not None else t0
        self.clock = Clock(t0, self.t_end, start=start, speed=speed, wall=wall)
        cal = find_calibration(self.st_full, session_dir, calib)
        self.cam = CAM.load_camera(cal) if cal else None
        self.calib_path = cal
        try:
            self.board = PL.Board.from_dict(self.st_full["board"]) if self.st_full.get("board") else None
        except Exception:
            self.board = None
        bc = self.board or PL.Board()
        rpx = (CAM.ROBOT_CM * self.cam.px_per_cm((bc.x0 + bc.x1) / 2, (bc.y0 + bc.y1) / 2, CAM.ROBOT_TOP_CM)
               if self.cam is not None else 30.0)
        self.feed = Feed()
        self.tracker = Tracker(rpx)
        self.lab = ReplayLab(self.clock)
        self.st = gate_state(self.st_full, t0)
        self.phase, self.order = "setup", ""
        self.lines = []
        self.t = None                      # the time everything was last moved to
        self.lock = threading.Lock()
        self.label = dt.datetime.fromtimestamp(self.t_start, self.tz).strftime("%Y-%m-%d %H:%M:%S")

    # --- time ---------------------------------------------------------------------------------------
    def hms(self, t):
        return dt.datetime.fromtimestamp(t, self.tz).strftime("%H:%M:%S")

    def parse_time(self, s):
        """HH:MM:SS (the session's day and clock; after midnight if the session ran past it) or epoch seconds."""
        s = str(s).strip()
        try:
            v = float(s)
            if v > 1e8:
                return v
        except ValueError:
            pass
        parts = [float(p) for p in s.split(":")]
        while len(parts) < 3:
            parts.append(0.0)
        day = dt.datetime.fromtimestamp(self.t_first, self.tz).date()
        for k in range(2):
            d = day + dt.timedelta(days=k)
            t = dt.datetime(d.year, d.month, d.day, int(parts[0]), int(parts[1]), int(parts[2]), tzinfo=self.tz).timestamp()
            t += parts[2] % 1
            if t >= self.t_first - 12 * 3600:
                return t
        return t

    # --- moving to the clock's time --------------------------------------------------------------------
    def step(self):
        t = self.clock.now()
        with self.lock:
            prev = self.t
            if prev is None or t < prev or t - prev > 30.0:
                # a jump (or the start): a fresh filter, fed the last few seconds before t
                self.lab = ReplayLab(self.clock)
                prev = t - 10.0
            self._feed_lab(prev, t)
            self.t = t
            k = bisect.bisect_right(self.track_t, t)
            last = {}
            for robot, (ts, rows) in self.by_robot.items():
                i = bisect.bisect_right(ts, t)
                if i:
                    last[robot] = (rows[i - 1][0], rows[i - 1][2], rows[i - 1][3])
            cur = self.track[k - 1] if k else None
            if cur is not None and t - cur[0] <= 2.0:
                self.tracker.active = cur[1]
            else:
                self.tracker.active = None
            self.tracker.pos = (cur[0], cur[2], cur[3]) if cur is not None and t - cur[0] <= 1.5 else None
            over = t >= self.t_end - 1.0
            self.st = gate_state(self.st_full, t, track_last=last, session_over=over)
            running = [r for r in self.st["schedule"] if r["status"] == "running"]
            if over:
                self.phase, self.order = "session over", ""
            elif cur is not None:
                self.phase = cur[5] or self.phase
                self.order = cur[4] or (str(running[0]["order"]) if running else "")
            if running and not over:
                self.order = str(running[0]["order"])
            self.lines = say_lines(self.log, t, times=self.log_t)
            if self.video is not None:
                self.feed.last_img = self.video.frame(t)
        return t

    def _feed_lab(self, t_from, t_to):
        """RAW rows and camera samples with t_from < t <= t_to, in time order."""
        i0, i1 = bisect.bisect_right(self.raw_t, t_from), bisect.bisect_right(self.raw_t, t_to)
        j0, j1 = bisect.bisect_right(self.track_t, t_from), bisect.bisect_right(self.track_t, t_to)
        ev = [(q[0], 0, q) for q in self.raw[i0:i1]] + [(q[0], 1, q) for q in self.track[j0:j1]]
        ev.sort(key=lambda e: (e[0], e[1]))
        for t, kind, q in ev:
            try:
                if kind == 0:
                    self.lab.on_raw(q[1], q[2], t)
                else:
                    self.lab.on_camera(q[1], t, q[2], q[3])
            except Exception:           # a display: a bad row never stops the replay
                pass

    def run_forever(self, stop, period=0.04):
        while not stop.is_set():
            try:
                self.step()
            except Exception as e:
                print("replay step: %s" % e, flush=True)
            stop.wait(period)


# =============================================================================== the pages
BADGE = """<div id="rp-badge">REPLAY of the session of %(label)s &middot; a recorded session, not live</div>"""
BAR = """<div id="rp-bar"><button id="rp-pause">Pause</button>
<span class="rp-sp"><button data-s="1">1x</button><button data-s="2">2x</button><button data-s="4">4x</button></span>
<b id="rp-now">--:--:--</b><input id="rp-slider" type="range" min="0" max="1000" value="0">
<span id="rp-end" class="rp-dim">--:--:--</span>
<form id="rp-jump"><input id="rp-at" placeholder="HH:MM:SS" size="8"><button>Jump</button></form></div>
<style>
#rp-badge{position:sticky;top:0;z-index:10;background:#c62828;color:#fff;font:600 15px system-ui,sans-serif;
 padding:7px 16px;text-align:center;letter-spacing:.02em}
#rp-bar{position:fixed;left:0;right:0;bottom:0;z-index:10;display:flex;flex-wrap:wrap;align-items:center;gap:10px;
 padding:8px 16px;background:#1d222b;border-top:2px solid #c62828;font:14px system-ui,sans-serif;color:#e8eaed}
#rp-bar button{background:#2c3340;color:#e8eaed;border:1px solid #3a4250;border-radius:5px;padding:4px 10px;cursor:pointer}
#rp-bar button.on{background:#c62828;border-color:#c62828}
#rp-slider{flex:1;min-width:140px}#rp-jump{display:flex;gap:4px;margin:0}
#rp-at{background:#11151b;color:#e8eaed;border:1px solid #3a4250;border-radius:5px;padding:4px 6px}
.rp-dim{color:#9aa0a6}body{padding-bottom:64px}
</style>
<script>
(function(){let st=null,drag=false;
function ctl(q){return fetch('/replay/control?'+q).then(r=>r.json()).then(show).catch(()=>{})}
function show(s){st=s;document.getElementById('rp-now').textContent=s.now_hms+(s.paused?' (paused)':'');
 document.getElementById('rp-end').textContent=s.end_hms;
 document.getElementById('rp-pause').textContent=s.paused?'Play':'Pause';
 document.querySelectorAll('#rp-bar [data-s]').forEach(b=>b.classList.toggle('on',+b.dataset.s==s.speed));
 if(!drag)document.getElementById('rp-slider').value=Math.round(1000*(s.t-s.t0)/Math.max(1,s.t1-s.t0));}
function poll(){fetch('/replay.json').then(r=>r.json()).then(show).catch(()=>{})}
document.getElementById('rp-pause').onclick=()=>ctl('paused='+(st&&st.paused?0:1));
document.querySelectorAll('#rp-bar [data-s]').forEach(b=>b.onclick=()=>ctl('speed='+b.dataset.s));
const sl=document.getElementById('rp-slider');sl.oninput=()=>{drag=true};
sl.onchange=()=>{drag=false;if(st)ctl('jump='+(st.t0+(st.t1-st.t0)*sl.value/1000))};
document.getElementById('rp-jump').onsubmit=e=>{e.preventDefault();
 ctl('jump='+encodeURIComponent(document.getElementById('rp-at').value))};
setInterval(poll,500);poll();})();
</script>"""


class ReplayView(VIEW.LiveView):
    """The live view's pages, served from a ReplayRunner, with the replay bar and badge."""

    def page(self, name):
        html = super().page(name)
        r = self.r
        html = html.replace("<title>E02 live view</title>", "<title>E02 REPLAY</title>")
        html = html.replace("<title>Live lab</title>", "<title>Live lab REPLAY</title>")
        html = html.replace(">E02 live view <", ">E02 view: REPLAY <")
        html = html.replace("<h1>Live lab</h1>", "<h1>Live lab (replay)</h1>")
        html = html.replace("<body>", "<body>" + BADGE % dict(label=r.label), 1)
        return html.replace("</body>", BAR + "</body>")

    def status(self):
        r = self.r
        d = super().status()
        d["log"] = list(r.lines)
        d["replay"] = self.replay_state()
        return d

    def replay_state(self):
        r, c = self.r, self.r.clock
        t = c.now()
        return dict(t=t, t0=c.t0, t1=c.t1, now_hms=r.hms(t), end_hms=r.hms(c.t1), start_hms=r.hms(c.t0),
                    speed=c.speed, paused=c.paused, label=r.label,
                    video=bool(r.video is not None and r.video.index(t) is not None))

    def extra_get(self, path):
        u = urlparse(path)
        if u.path == "/replay.json":
            return json.dumps(self.replay_state()).encode(), "application/json"
        if u.path == "/replay/control":
            q = {k: v[-1] for k, v in parse_qs(u.query).items()}
            kw = {}
            if "speed" in q and _num(q["speed"]) and 0 < float(q["speed"]) <= 16:
                kw["speed"] = float(q["speed"])
            if "paused" in q:
                kw["paused"] = q["paused"] in ("1", "true", "yes")
            if q.get("jump"):
                try:
                    kw["jump"] = self.r.parse_time(q["jump"])
                except ValueError:
                    pass
            self.r.clock.set(**kw)
            self.r.step()
            return json.dumps(self.replay_state()).encode(), "application/json"
        return None

    def frame_bgr(self):
        r = self.r
        with r.lock:
            v = super().frame_bgr()
        t = r.t if r.t is not None else r.clock.now()
        if r.feed.last_img is None:
            cv2.putText(v, "no recording at this time", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        txt = "REPLAY %s  %s" % (r.hms(t), "paused" if r.clock.paused else "%gx" % r.clock.speed)
        (w, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(v, (6, 6), (18 + w, 16 + h), (40, 40, 198), -1)
        cv2.putText(v, txt, (12, 12 + h), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return v


# =============================================================================== command line
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True, help="the runner's session folder logs/e02-<date-time>")
    ap.add_argument("--recording", help="the overhead recording: a folder with the .avi and frames.csv, or the .avi")
    ap.add_argument("--frames", help="frames.csv, if not next to the video")
    ap.add_argument("--first-frame", type=int, default=None,
                    help="frames.csv row of the video's first frame (default: the video runs to the recording's end)")
    ap.add_argument("--from", dest="start", help="start at this time (HH:MM:SS, the session's clock)")
    ap.add_argument("--speed", type=float, default=1.0, choices=(1.0, 2.0, 4.0))
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--calib", help="the floor calibration, if the session's one is not on this computer")
    a = ap.parse_args(argv)
    video = None
    if a.recording:
        vpath, fpath = find_recording(a.recording, a.frames)
        video = Video(vpath, fpath, a.first_frame)
    run = ReplayRunner(a.session, video=video, speed=a.speed, calib=a.calib)
    if a.start:
        run.clock.set(jump=run.parse_time(a.start))
    run.step()
    view = ReplayView(run, a.port)
    msg = view.start()
    print("REPLAY of the session started %s (%s to %s)" % (run.label, run.hms(run.clock.t0), run.hms(run.clock.t1)))
    if video is not None:
        print("  recording: %s to %s; camera clock %+.2f s from the laptop's (%s)" % (
            run.hms(video.t_first), run.hms(video.t_last), video.offset, video.offset_source))
    else:
        print("  no recording given: the picture stays black")
    print("  starting at %s, %gx; floor calibration: %s" % (run.hms(run.clock.now()), run.clock.speed, run.calib_path))
    print("  " + msg + "  (camera view: /, Live lab: /lab).  Ctrl+C to stop.", flush=True)
    if view.srv is None:
        return 1
    stop = threading.Event()
    try:
        run.run_forever(stop)
    except KeyboardInterrupt:
        pass
    stop.set()
    view.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
