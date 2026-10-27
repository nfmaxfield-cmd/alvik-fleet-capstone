#!/usr/bin/env python3
"""Camera setup page (2026-10-27): after the cameras have been moved, make the overhead camera's floor calibration
from the board's corners again, look at the side camera, and pick the overhead exposure that makes the robots'
lights stand out. Runs ON THE PI; open http://<the Pi>:8096 from the laptop.

    ./run-fleet-pi.sh setup                            (the same as the line below)
    python3 tools/pi/camera_setup.py                   # cameras found by name (tools/pi/camnames.py)
    python3 tools/pi/camera_setup.py --fake            # no cameras, no robots: a simulated board (how it was tested)

The page owns the cameras while it runs: stop the preview / recorder first (camrec locks the exposure at its start,
so the two cannot share the camera). Stop the page with Ctrl+C; it puts the overhead camera back on automatic
exposure (the saved value, if any, is applied by camrec at the recorder's next start).

1. Overhead corners: the board's 4 corners found in a picture (e02_camera.find_board_corners, the same order and
   floor frame as e02.py's classic_calibration: corner 1 = (0, 0) at the picture's bottom-right, 2 = (122.15, 0),
   3 = (122.15, 121.9), 4 = (0, 121.9)); drag any corner or click 4 in order; Check draws a 10 cm grid from the
   fitted calibration; Save writes ~/.config/mirlt/calib/overhead_floorcal_<date-time>.json and points
   ~/.config/mirlt/overhead_calib at it (tools/pi/setup_conf.py; run-fleet-pi.sh, session_plan.sh and e02.py read it).
2. Side camera: its picture and frame rate. Its calibration is refitted from the robots
   (docs/build-plans/two-camera-geometry.md); optional corner clicks are stored separately, not used yet.
3. Flash check (a small DOE: one factor, exposure, at 4 levels): all robots light their LEDs (`led 1 1 1 1500`)
   while the overhead camera takes pictures at its locked automatic exposure and at 1/2, 1/4, 1/8 of it; for each:
   the lights' contrast (LED on minus LED off at each robot), light against the board, and the robots' tops against
   the board (what the tracker needs). Save the recommended (or any) one as the overhead exposure. If the side camera
   is open too, the moment each camera first sees the lights gives the side camera's lag.
"""
import argparse
import json
import os
import shutil
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
for p in (HERE, os.path.join(REPO, "tools", "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import e02_camera as CAM  # noqa: E402
import setup_conf as SC  # noqa: E402

# chosen: next to mission control's 8095; nothing else on the Pi uses it (launcher 8091, live page 8090, camrec 8000/8001)
PORT = 8096
ROBOT_IPS = ("192.168.1.250", "192.168.1.252", "192.168.1.253")
CMD_PORT = 5006                    # tools/alvik_console.py CMD_PORT (laptop/Pi -> robot)
GRID_CM = 10.0                     # Nick's ask: a 10 cm grid
LENS_HEIGHT_CM = 262.0             # roam1's measured lens height (its studio spec); change it on the page if moved
# DOE levels (chosen: halving steps cover 8x in 4 runs; a finer second round near the winner would be the next DOE)
DOE_FRACTIONS = (1.0, 0.5, 0.25, 0.125)
LED_MS = 1500                      # chosen: long enough for 0.3 s settle + ~0.9 s of steady "on" pictures at 25 fps
SETTLE_S = 0.8                     # chosen: UVC webcams take a few frames to apply a new exposure (no DOE: a wait)
# Recommendation rule: a robot's top must differ from the board by at least twice the tracker's own threshold
# (e02_camera.MotionTracker thr=16 grey levels; the factor 2 is chosen, not derived: a DOE on the tracker's lock-on
# rate at each exposure, on recorded footage, would replace it; see the report). Then the strongest weakest light.
TRACKER_THR = 16.0
TOP_MIN = 2 * TRACKER_THR


# ======================================================================================= maths (tested)
def px_per_cm(corners):
    c = np.asarray(corners, float)
    sides = [np.hypot(*(c[(i + 1) % 4] - c[i])) for i in range(4)]
    return float(np.mean(sides)) / ((CAM.BOARD_W_CM + CAM.BOARD_L_CM) / 2)


def fit(corners, image_size, path, height_cm=LENS_HEIGHT_CM):
    """Fit and write a calibration (e02_camera.make_calibration, as e02.py classic_calibration). Returns
    (CameraModel, floorcal result)."""
    res = CAM.make_calibration([[float(u), float(v)] for u, v in corners], tuple(image_size), path,
                               height_cm=height_cm or None)
    return CAM.load_camera(path), res


def board_outline_px(cam, n=24, z=0.0):
    W, L = CAM.BOARD_W_CM, CAM.BOARD_L_CM
    pts = [(W * k / n, 0) for k in range(n)] + [(W, L * k / n) for k in range(n)]
    pts += [(W - W * k / n, L) for k in range(n)] + [(0, L - L * k / n) for k in range(n + 1)]
    return np.array([cam.project((x, y, z)) for x, y in pts], float)


def draw_grid(img, cam, corners=None, old_cam=None, step_cm=GRID_CM):
    """The check picture: a 10 cm grid over the board from cam (thin), the board outline (green), the corners used
    (red, numbered), yesterday's outline from old_cam (orange) for comparison. BGR uint8."""
    out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
    W, L = CAM.BOARD_W_CM, CAM.BOARD_L_CM

    def line(pts, col, th):
        uv = np.array([cam.project((x, y, 0.0)) for x, y in pts], float)
        ok = np.all(np.isfinite(uv), axis=1)
        cv2.polylines(out, [np.round(uv[ok]).astype(np.int32)], False, col, th, cv2.LINE_AA)
    xs = list(np.arange(0, W, step_cm)) + [W]
    ys = list(np.arange(0, L, step_cm)) + [L]
    for x in xs[1:-1]:
        line([(x, y) for y in np.linspace(0, L, 13)], (255, 220, 0) if round(x) % 50 else (255, 120, 255), 1)
    for y in ys[1:-1]:
        line([(x, y) for x in np.linspace(0, W, 13)], (255, 220, 0) if round(y) % 50 else (255, 120, 255), 1)
    if old_cam is not None:
        uv = board_outline_px(old_cam)
        cv2.polylines(out, [np.round(uv).astype(np.int32)], True, (0, 140, 255), 1, cv2.LINE_AA)
        u, v = old_cam.project((0.0, 0.0, 0.0))
        if np.isfinite(u):
            cv2.circle(out, (int(round(u)), int(round(v))), 9, (0, 140, 255), 2, cv2.LINE_AA)
            cv2.putText(out, "old 0,0", (int(u) - 70, int(v) + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 255), 1)
    cv2.polylines(out, [np.round(board_outline_px(cam)).astype(np.int32)], True, (0, 220, 0), 2, cv2.LINE_AA)
    for i, (u, v) in enumerate(corners or []):
        cv2.circle(out, (int(round(u)), int(round(v))), 6, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(out, str(i + 1), (int(u) + 7, int(v) - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    for (x, y, t) in ((4, 4, "0,0"), (W - 14, 4, "x"), (4, L - 8, "y")):
        u, v = cam.project((x, y, 0.0))
        if np.isfinite(u):
            cv2.putText(out, t, (int(u), int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 2)
    return out


def board_mask(shape, corners, grow_px=0):
    m = np.zeros(shape[:2], np.uint8)
    cv2.fillPoly(m, [np.round(np.asarray(corners, float)).astype(np.int32)], 255)
    if grow_px > 0:
        m = cv2.dilate(m, np.ones((2 * grow_px + 1,) * 2, np.uint8))
    elif grow_px < 0:
        m = cv2.erode(m, np.ones((-2 * grow_px + 1,) * 2, np.uint8))
    return m > 0


def analyse_flash(off, on, corners, max_robots=3):
    """One exposure setting: off, on = median grey pictures with the LEDs off / on. Finds the lights from on - off
    inside the board, groups the two LEDs of a robot (lights within 8 cm), and measures per robot:
      light: LED on minus LED off at the light (95th percentile of its pixels), grey levels
      light_vs_board: the lit LED minus the board's median brightness
      top: |board round the robot minus the robot's top| in the off picture (a 2.5 cm disc at the lights' centre
           against a 9-14 cm ring; the LEDs sit on the top near its front, so the disc is on the top)
    and for the board: its median brightness and the share of it at 250+ (burnt out). Returns a dict."""
    off = off.astype(np.int16)
    on = on.astype(np.int16)
    ppc = px_per_cm(corners)
    inside = board_mask(off.shape, corners, grow_px=int(3 * ppc))
    board = board_mask(off.shape, corners, grow_px=-int(2 * ppc))
    board_med = float(np.median(off[board])) if board.any() else float("nan")
    sat = float(np.mean(off[board] >= 250)) if board.any() else float("nan")
    diff = np.where(inside, on - off, 0)
    peak = int(diff.max())
    out = dict(board=round(board_med, 1), burnt_pct=round(100 * sat, 2), peak_diff=peak, robots=[])
    if peak < 25:            # chosen: below this, sensor noise and auto white balance, not a light (no DOE: a floor)
        out["why"] = "no lights seen (largest change %d grey levels): are the robots on, idle and on this WiFi?" % peak
        return out
    lit = (diff >= max(20, 0.35 * peak)).astype(np.uint8)
    grow = max(1, int(4 * ppc))
    groups = cv2.dilate(lit, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * grow + 1,) * 2))
    n, lab, stats, cents = cv2.connectedComponentsWithStats(groups)
    found = []
    for k in range(1, n):
        sel = (lab == k) & (lit > 0)
        if sel.sum() < 2:
            continue
        found.append((float(diff[sel].sum()), k, sel))
    found.sort(reverse=True)
    yy, xx = np.mgrid[0:off.shape[0], 0:off.shape[1]]
    for _, k, sel in found[:max_robots]:
        vs, us = np.nonzero(sel)
        cu, cv_ = float(us.mean()), float(vs.mean())
        on_v = float(np.percentile(on[sel], 95))
        off_v = float(np.percentile(off[sel], 95))
        r2 = (xx - cu) ** 2 + (yy - cv_) ** 2
        disc = r2 <= (2.5 * ppc) ** 2
        ring = (r2 >= (9 * ppc) ** 2) & (r2 <= (14 * ppc) ** 2) & board
        top = abs(float(np.median(off[ring])) - float(np.median(off[disc]))) if ring.any() else float("nan")
        out["robots"].append(dict(u=round(cu, 1), v=round(cv_, 1), light=round(on_v - off_v, 1),
                                  light_vs_board=round(on_v - board_med, 1), top=round(top, 1)))
    out["robots"].sort(key=lambda r: r["u"])
    if out["robots"]:
        out["weakest_light"] = min(min(r["light"], r["light_vs_board"]) for r in out["robots"])
        out["weakest_top"] = min(r["top"] for r in out["robots"])
    return out


def recommend(rows):
    """rows: [dict(exposure, fraction, result=analyse_flash(...))], the first the automatic one. The setting with the
    strongest weakest light among those whose weakest robot top stands >= TOP_MIN from the board.
    Returns (index or None, words)."""
    have = [i for i, r in enumerate(rows) if r["result"].get("robots")]
    if not have:
        return None, "No lights were seen at any setting, so nothing to recommend: the camera's automatic stays."
    need = TOP_MIN
    ok = [i for i in have if rows[i]["result"]["weakest_top"] >= need]
    if not ok:
        return None, ("At no setting did every robot's top stand %.0f grey levels from the board (twice the tracker's "
                      "threshold), so nothing is recommended: keep the camera's automatic, and tell Claude the "
                      "numbers below." % need)
    best = max(ok, key=lambda i: rows[i]["result"]["weakest_light"])
    r = rows[best]
    words = ("Recommended: %g (%s of the automatic): its weakest light stands %.0f grey levels out, and its weakest "
             "robot top %.0f from the board (the rule needs at least %.0f, twice the tracker's threshold)." % (
                 r["exposure"], "all" if r["fraction"] == 1 else "1/%d" % round(1 / r["fraction"]),
                 r["result"]["weakest_light"], r["result"]["weakest_top"], need))
    return best, words


def _kth_largest(a, k):
    a = a.ravel()
    k = min(k, a.size)
    return float(np.partition(a, a.size - k)[a.size - k])


def onset(times, frames, ref, t_send):
    """When the lights first show: per picture, the 20th-largest brightening against ref anywhere in the picture
    (the lights are a few dozen pixels; the 20th largest ignores a few hot pixels)
    (no calibration needed, so it works for the side camera too); the first picture after t_send reaching half the
    largest rise. Returns (time, rise) or (None, rise)."""
    if not frames:
        return None, 0.0
    s = [_kth_largest(cv2.subtract(f, ref), 20) for f in frames]
    top = max(s) - float(np.median(s[:max(1, len(s) // 4)]))
    if top < 20:            # chosen: the same "is it a light" floor as above, a little lower for small far lights
        return None, top
    thr = float(np.median(s[:max(1, len(s) // 4)])) + 0.5 * top
    for t, v in zip(times, s):
        if t >= t_send and v >= thr:
            return t, top
    return None, top


# ======================================================================================= cameras
class Grabber(threading.Thread):
    """Reads one camera all the time: the newest picture, its read time, the real frame rate, and a recording
    window for the flash check."""

    def __init__(self, name, src):
        super().__init__(daemon=True)
        self.name, self.src = name, src
        self.lock = threading.Lock()
        self.frame, self.t = None, 0.0
        self.times = []
        self.rec = None
        self.paused = threading.Event()
        self.idle = threading.Event()
        self.stopping = False
        self.fails = 0

    def run(self):
        while not self.stopping:
            if self.paused.is_set():
                self.idle.set()
                time.sleep(0.02)
                continue
            self.idle.clear()
            ok, f = self.src.read()
            t = time.monotonic()
            if not ok or f is None:
                self.fails += 1
                time.sleep(0.05)
                continue
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) if f.ndim == 3 else f
            with self.lock:
                self.frame, self.t = g, t
                self.times.append(t)
                self.times = [x for x in self.times if x > t - 3.0]
                if self.rec is not None:
                    self.rec.append((t, g))

    def latest(self):
        with self.lock:
            return (None if self.frame is None else self.frame.copy()), self.t

    def fps(self):
        with self.lock:
            ts = list(self.times)
        if len(ts) < 3 or time.monotonic() - ts[-1] > 1.0:
            return 0.0
        return (len(ts) - 1) / (ts[-1] - ts[0])

    def record(self, on):
        with self.lock:
            if on:
                self.rec = []
                return None
            r, self.rec = self.rec, None
            return r or []

    def pause(self, on):
        if on:
            self.paused.set()
            self.idle.wait(2.0)
        else:
            self.paused.clear()


class LiveSource:
    """A real camera through camrec's opening and control code."""

    def __init__(self, dev, fps=25):
        import camrec
        self.CR = camrec
        self.dev, self.fps = dev, fps
        self.cap = camrec.open_camera(str(dev), 640, 480, fps, 4, "MJPG",
                                      fail=lambda m: (_ for _ in ()).throw(RuntimeError(m)))
        self.ctl = camrec.Controls(self.cap, "/dev/video%s" % dev, log=lambda s: print("  " + s))

    def read(self):
        return self.cap.read()

    def lock_exposure(self):
        """Let automatic exposure settle, freeze it (camrec.apply_controls 'lock'), return the value."""
        self.ctl.set("auto_exposure", self.CR.AE_AUTO)
        rep = self.CR.apply_controls(self.cap, str(self.dev), exposure="lock", focus=None, wb=None, fps=self.fps,
                                     warmup_s=2.0, fps_check_s=0)
        v = (rep.get("after") or {}).get("exposure") or self.cap.get(cv2.CAP_PROP_EXPOSURE)
        return float(v) if v and v > 0 else None

    def exposure_min(self):
        rng = (self.ctl.v4l2 or {}).get(self.ctl._v4l2_name("exposure") or "", {})
        return float(rng.get("min", 1) or 1)

    def set_exposure(self, v):
        self.ctl.set("auto_exposure", self.CR.AE_MANUAL, alt=self.CR.AE_MANUAL_OLD)
        return self.ctl.set("exposure", float(v))

    def restore_auto(self):
        return self.ctl.set("auto_exposure", self.CR.AE_AUTO)

    def release(self):
        self.cap.release()


class UdpRobots:
    def __init__(self, ips=ROBOT_IPS):
        self.ips = list(ips)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, cmd):
        for ip in self.ips:
            try:
                self.sock.sendto(cmd.encode(), (ip, CMD_PORT))
            except OSError as e:
                print("  could not send %r to %s: %s" % (cmd, ip, e))


# ---------------------------------------------------------------------------------- the simulated board (--fake)
class FakeWorld:
    """Three robots on the board, LEDs that obey `led`, and two cameras: the overhead from roam1's model moved a
    little (as if the camera had been moved), the side from side_floorcal_roam1.json. Brightness scales with
    exposure; the LEDs are bright enough to burn out at the automatic exposure."""
    ROBOTS = ((30.0, 40.0, 0.3), (75.0, 90.0, 2.0), (95.0, 25.0, -1.2))

    def __init__(self, side_lag_s=0.04):
        self.led_until = 0.0
        self.side_lag_s = side_lag_s
        self.lock = threading.Lock()

    def send(self, cmd):
        p = cmd.split()
        if p and p[0] == "led":
            ms = float(p[4]) if len(p) > 4 else 2000.0
            on = any(x != "0" for x in p[1:4])
            with self.lock:
                self.led_until = time.monotonic() + ms / 1000.0 if on else 0.0

    def lit(self, t):
        with self.lock:
            return t < self.led_until


def _moved_overhead():
    cam = CAM.load_camera(SC.ROAM1)
    d = cam.to_dict()
    d["rvec"] = (np.asarray(d["rvec"], float) + [0.01, -0.01, 0.05]).tolist()
    d["tvec"] = (np.asarray(d["tvec"], float) + [3.0, 4.0, 0.0]).tolist()
    return CAM.CameraModel.from_dict(d)


class FakeSource:
    def __init__(self, world, which="overhead", fps=25.0, seed=1):
        self.world, self.which, self.fps = world, which, fps
        if which == "overhead":
            self.cam = _moved_overhead()
        else:
            self.cam = CAM.load_camera(os.path.join(REPO, "tools", "experiment", "data", "side_floorcal_roam1.json"))
        W, H = self.cam.image_size
        self.rng = np.random.default_rng(seed)
        base = np.full((H, W), 45.0, np.float32)
        yy, xx = np.mgrid[0:H, 0:W]
        board = np.zeros((H, W), np.uint8)
        cv2.fillPoly(board, [np.round(board_outline_px(self.cam)).astype(np.int32)], 1)
        base[board > 0] = (165 + 10 * np.sin(xx / 23.0) * np.cos(yy / 31.0))[board > 0]
        leds = np.zeros((H, W), np.float32)
        for x, y, th in FakeWorld.ROBOTS:
            ring = [(x + 4.75 * np.cos(a), y + 4.75 * np.sin(a), CAM.ROBOT_TOP_CM) for a in np.linspace(0, 2 * np.pi, 30)]
            uv = np.array([self.cam.project(p) for p in ring])
            cv2.fillPoly(base, [np.round(uv).astype(np.int32)], 55.0)
            for side in (-1, 1):                          # two LEDs near the front of the top
                lx = x + 3.0 * np.cos(th) - side * 2.5 * np.sin(th)
                ly = y + 3.0 * np.sin(th) + side * 2.5 * np.cos(th)
                u, v = self.cam.project((lx, ly, CAM.ROBOT_TOP_CM))
                cv2.circle(leds, (int(round(u)), int(round(v))), 2, 900.0, -1)
        self.base = cv2.GaussianBlur(base, (3, 3), 0)
        self.leds = cv2.GaussianBlur(leds, (7, 7), 1.6)
        self.auto = 120.0 if which == "overhead" else 160.0
        self.exposure, self.manual = self.auto, False
        self.next_t = time.monotonic()
        self.image_size = (W, H)

    def read(self):
        now = time.monotonic()
        if self.next_t > now:
            time.sleep(self.next_t - now)
        self.next_t = max(self.next_t + 1.0 / self.fps, time.monotonic())
        t = time.monotonic() - (self.world.side_lag_s if self.which == "side" else 0.0)
        g = self.exposure / self.auto
        img = self.base * g + (self.leds * g if self.world.lit(t) else 0.0)
        img = img + self.rng.normal(0, 2.0, img.shape).astype(np.float32)
        return True, np.clip(img, 0, 255).astype(np.uint8)

    def lock_exposure(self):
        self.exposure, self.manual = self.auto, True
        return self.auto

    def exposure_min(self):
        return 1.0

    def set_exposure(self, v):
        self.exposure, self.manual = float(v), True
        return True

    def restore_auto(self):
        self.exposure, self.manual = self.auto, False
        return True

    def release(self):
        pass


# ======================================================================================= the page's state
class Setup:
    def __init__(self, over, side, robots, out_dir, fake=False):
        self.over, self.side, self.robots, self.out, self.fake = over, side, robots, out_dir, fake
        os.makedirs(out_dir, exist_ok=True)
        self.lock = threading.Lock()
        self.still = None              # the overhead picture the corners are on
        self.corners, self.found_words = None, ""
        self.check = None              # dict(err_cm, max_err_cm, text, jpg, path)
        self.saved = None
        self.side_corners = None
        self.doe = dict(state="idle", progress="", rows=[], rec=None, words="", lag=None)
        old, words = SC.overhead_calib(relative=False)
        self.old_path, self.old_words = old, words
        try:
            self.old_cam = CAM.load_camera(old)
        except Exception:
            self.old_cam = None
        self.jpgs = {}

    # ---- corners
    def grab(self):
        f = None
        for _ in range(40):
            f, t = self.over.latest()
            if f is not None:
                break
            time.sleep(0.1)
        if f is None:
            raise RuntimeError("the overhead camera sends no pictures (is the preview or recorder still running?)")
        time.sleep(0.3)
        f, _ = self.over.latest()
        corners, why = CAM.find_board_corners(f)
        with self.lock:
            self.still = f
            cv2.imwrite(os.path.join(self.out, "overhead.png"), f)
            if corners is None:
                # start from yesterday's corners so there is something to drag
                self.corners = (np.round(board_corners_px(self.old_cam), 1).tolist() if self.old_cam is not None
                                else [[480, 400], [480, 80], [160, 80], [160, 400]])
                self.found_words = ("Could not find the board's corners by myself (%s). The red points start at "
                                    "yesterday's corners: drag each onto its board corner." % why)
            else:
                self.corners = corners
                self.found_words = "Found the board's 4 corners by myself. Check them, drag any that are off."
            self.check = None
        return self.state()

    def fit_check(self, corners, height_cm, save=False):
        if self.still is None:
            raise RuntimeError("no overhead picture yet")
        corners = [[float(u), float(v)] for u, v in corners]
        if len(corners) != 4:
            raise ValueError("4 corners are needed")
        path = SC.new_calib_path() if save else os.path.join(self.out, "candidate_overhead_floorcal.json")
        cam, res = fit(corners, (self.still.shape[1], self.still.shape[0]), path, height_cm)
        pic = draw_grid(self.still, cam, corners, self.old_cam)
        cv2.imwrite(os.path.join(self.out, "check_saved.png" if save else "check.png"), pic)
        moved = None
        if self.old_cam is not None:
            moved = float(np.max(np.hypot(*(np.asarray(corners) - board_corners_px(self.old_cam)).T)))
        with self.lock:
            self.corners = corners
            self.jpgs["check"] = _jpg(pic)
            self.check = dict(err_cm=res["err_cm"], max_err_cm=res["max_err_cm"], text=res["text"],
                              height_cm=height_cm, moved_px=None if moved is None else round(moved, 1),
                              mirrored=bool(res.get("mirrored")))
        if save:
            SC.save_calib_pointer(path)
            shutil.copy(path, os.path.join(self.out, os.path.basename(path)))
            with open(path) as f:
                d = json.load(f)
            d["camera_setup"] = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), corners_px=corners, height_cm=height_cm,
                                     page="tools/pi/camera_setup.py", fake=self.fake, logs=self.out)
            with open(path, "w") as f:
                json.dump(d, f, indent=1)
            with self.lock:
                self.saved = dict(path=path, pointer=os.path.join(SC.conf_dir(), "overhead_calib"))
        return self.state()

    def save_side(self, corners):
        f, _ = self.side.latest() if self.side else (None, 0)
        path = os.path.join(SC.conf_dir(), "side_corners_%s.json" % time.strftime("%Y%m%d-%H%M%S"))
        os.makedirs(SC.conf_dir(), exist_ok=True)
        d = dict(corners_px=corners, order="as clicked: (0,0), (122.15,0), (122.15,121.9), (0,121.9) like the overhead",
                 image_size=None if f is None else [f.shape[1], f.shape[0]], note="not used yet: the side calibration "
                 "is refitted from the robots (docs/build-plans/two-camera-geometry.md)")
        with open(path, "w") as fh:
            json.dump(d, fh, indent=1)
        if f is not None:
            cv2.imwrite(os.path.join(self.out, "side.png"), f)
        shutil.copy(path, self.out)
        with self.lock:
            self.side_corners = dict(path=path, corners=corners)
        return self.state()

    # ---- flash check (DOE)
    def start_flash(self):
        with self.lock:
            if self.doe["state"] == "running":
                return self.state()
            if self.corners is None:
                raise RuntimeError("find the board's corners first (the check measures inside the board)")
            self.doe = dict(state="running", progress="starting", rows=[], rec=None, words="", lag=None)
        threading.Thread(target=self._flash, daemon=True).start()
        return self.state()

    def _say(self, s):
        print("  flash check: " + s, flush=True)
        with self.lock:
            self.doe["progress"] = s

    def _flash(self):
        src = self.over.src
        rows, lags = [], []
        try:
            self._say("letting the overhead camera's automatic exposure settle, then locking it (about 3 s)")
            self.over.pause(True)
            try:
                base = src.lock_exposure()
            finally:
                self.over.pause(False)
            if not base:
                raise RuntimeError("the overhead camera does not report its exposure, so it cannot be set here")
            lo = src.exposure_min()
            levels = []
            for fr in DOE_FRACTIONS:
                v = max(lo, round(base * fr))
                if v not in [x for x, _ in levels]:
                    levels.append((v, fr))
            corners = list(self.corners)
            for i, (v, fr) in enumerate(levels):
                self._say("setting %d of %d: exposure %g (%s of the automatic)" % (
                    i + 1, len(levels), v, "all" if fr == 1 else "1/%d" % round(1 / fr)))
                if i:
                    src.set_exposure(v)
                time.sleep(SETTLE_S)
                self.over.record(True)
                if self.side:
                    self.side.record(True)
                time.sleep(0.6)
                t_send = time.monotonic()
                self.robots.send("led 1 1 1 %d" % LED_MS)
                time.sleep(LED_MS / 1000.0 - 0.2)
                ov = self.over.record(False)
                sd = self.side.record(False) if self.side else []
                time.sleep(0.9)            # the lights go off before the next setting
                offs = [g for t, g in ov if t < t_send - 0.05]
                ons = [g for t, g in ov if t_send + 0.35 <= t]
                if len(offs) < 2 or len(ons) < 2:
                    raise RuntimeError("too few pictures from the overhead camera (%d off, %d on)" % (len(offs), len(ons)))
                off = np.median(np.stack(offs), axis=0).astype(np.uint8)
                on = np.median(np.stack(ons), axis=0).astype(np.uint8)
                res = analyse_flash(off, on, corners)
                to, _ = onset([t for t, _ in ov], [g for _, g in ov], off, t_send)
                ts = None
                if sd:
                    soffs = [g for t, g in sd if t < t_send - 0.05]
                    if soffs:
                        sref = np.median(np.stack(soffs), axis=0).astype(np.uint8)
                        ts, _ = onset([t for t, _ in sd], [g for _, g in sd], sref, t_send)
                if to is not None and ts is not None:
                    lags.append(ts - to)
                key = "doe%d" % i
                pic = flash_picture(off, on, res)
                cv2.imwrite(os.path.join(self.out, "flash_%02d_exposure_%g.png" % (i, v)), pic)
                with self.lock:
                    self.jpgs[key] = _jpg(pic)
                rows.append(dict(exposure=v, fraction=fr, result=res, jpg=key, n_off=len(offs), n_on=len(ons),
                                 seen_at_s=None if to is None else round(to - t_send, 3)))
                with self.lock:
                    self.doe["rows"] = list(rows)
            rec, words = recommend(rows)
            lag = None
            if lags:
                lag = dict(values=[round(x, 3) for x in lags], median=round(float(np.median(lags)), 3),
                           over_fps=round(self.over.fps(), 1), side_fps=round(self.side.fps(), 1) if self.side else None)
            with self.lock:
                self.doe.update(state="done", rec=rec, words=words, lag=lag, base=base,
                                progress="done: the camera is back on automatic exposure")
            with open(os.path.join(self.out, "flash_check.json"), "w") as f:
                json.dump(dict(rows=rows, recommended=rec, words=words, lag=lag, base=base,
                               fractions=DOE_FRACTIONS, led_ms=LED_MS, fake=self.fake), f, indent=1, default=str)
        except Exception as e:
            with self.lock:
                self.doe.update(state="failed", progress="stopped: %s" % e)
            print("  flash check failed: %s" % e, flush=True)
        finally:
            try:
                src.restore_auto()
            except Exception:
                pass

    def save_exposure(self, value):
        info = {}
        with self.lock:
            if value is not None:
                row = next((r for r in self.doe.get("rows", []) if abs(r["exposure"] - float(value)) < 0.5), None)
                if row:
                    info = dict(fraction_of_auto=row["fraction"], auto=self.doe.get("base"),
                                weakest_light=row["result"].get("weakest_light"),
                                weakest_top=row["result"].get("weakest_top"), logs=self.out)
        SC.save_exposure(value, why="camera setup page flash check (a 4-level DOE)", **info)
        return self.state()

    def state(self):
        with self.lock:
            v, d = SC.overhead_exposure()
            return dict(
                fake=self.fake, out=self.out, corners=self.corners, found=self.found_words,
                image_size=None if self.still is None else [self.still.shape[1], self.still.shape[0]],
                check=self.check, saved=self.saved, old=dict(path=self.old_path, words=self.old_words),
                height_cm=LENS_HEIGHT_CM, grid_cm=GRID_CM,
                fps=dict(overhead=round(self.over.fps(), 1), side=None if not self.side else round(self.side.fps(), 1)),
                side=bool(self.side), side_corners=self.side_corners,
                doe=dict(state=self.doe["state"], progress=self.doe["progress"], words=self.doe.get("words", ""),
                         rec=self.doe.get("rec"), lag=self.doe.get("lag"), base=self.doe.get("base"),
                         rows=[dict(exposure=r["exposure"], fraction=r["fraction"], jpg=r["jpg"], res=r["result"],
                                    seen_at_s=r["seen_at_s"]) for r in self.doe.get("rows", [])]),
                exposure=dict(value=v, saved=d.get("saved")), still_t=0 if self.still is None else id(self.still))


def board_corners_px(cam):
    W, L = CAM.BOARD_W_CM, CAM.BOARD_L_CM
    return np.array([cam.project((x, y, 0.0)) for x, y in ((0, 0), (W, 0), (W, L), (0, L))], float)


def flash_picture(off, on, res, crop_cm=14):
    """The LEDs-on picture with each robot ringed, and under it each robot close up: off | on (2x)."""
    pic = cv2.cvtColor(on, cv2.COLOR_GRAY2BGR)
    for k, r in enumerate(res.get("robots", [])):
        cv2.circle(pic, (int(r["u"]), int(r["v"])), 16, (0, 200, 255), 2, cv2.LINE_AA)
        cv2.putText(pic, str(k + 1), (int(r["u"]) + 18, int(r["v"])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
    crops = []
    h = 40
    for r in res.get("robots", []):
        u, v = int(r["u"]), int(r["v"])
        for src in (off, on):
            c = np.zeros((2 * h, 2 * h), np.uint8)
            y0, x0 = max(0, v - h), max(0, u - h)
            part = src[y0:v + h, x0:u + h]
            c[:part.shape[0], :part.shape[1]] = part
            crops.append(cv2.resize(c, (2 * h * 2, 2 * h * 2), interpolation=cv2.INTER_NEAREST))
        crops.append(np.full((4 * h, 6), 255, np.uint8))
    if crops:
        strip = np.hstack(crops)
        W = pic.shape[1]
        if strip.shape[1] > W:
            strip = cv2.resize(strip, (W, int(strip.shape[0] * W / strip.shape[1])))
        pad = np.zeros((strip.shape[0], W), np.uint8)
        pad[:, :strip.shape[1]] = strip
        pic = np.vstack([pic, cv2.cvtColor(pad, cv2.COLOR_GRAY2BGR)])
    return pic


def _jpg(img, q=85):
    ok, b = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return b.tobytes() if ok else b""


# ======================================================================================= web
PAGE = os.path.join(HERE, "camera_setup.html")


def make_handler(S):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json"):
            if isinstance(body, (dict, list)):
                body = json.dumps(body, default=str).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/":
                with open(PAGE, encoding="utf-8") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if p == "/api/state":
                return self._send(200, S.state())
            if p == "/img/still.jpg":
                with S.lock:
                    st = S.still
                return self._send(200, _jpg(st), "image/jpeg") if st is not None else self._send(404, b"", "text/plain")
            if p in ("/img/side.jpg", "/img/overhead_live.jpg"):
                g = S.side if p == "/img/side.jpg" else S.over
                f, _ = g.latest() if g else (None, 0)
                return self._send(200, _jpg(f), "image/jpeg") if f is not None else self._send(404, b"", "text/plain")
            if p.startswith("/img/") and p.endswith(".jpg"):
                with S.lock:
                    b = S.jpgs.get(p[5:-4])
                return self._send(200, b, "image/jpeg") if b else self._send(404, b"", "text/plain")
            return self._send(404, {"error": "not found"})

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                body = {}
            p = self.path
            try:
                if p == "/api/grab":
                    return self._send(200, S.grab())
                if p in ("/api/check", "/api/save"):
                    h = body.get("height_cm")
                    h = float(h) if h not in (None, "") else None
                    return self._send(200, S.fit_check(body["corners"], h, save=p == "/api/save"))
                if p == "/api/side_corners":
                    return self._send(200, S.save_side(body["corners"]))
                if p == "/api/flash":
                    return self._send(200, S.start_flash())
                if p == "/api/exposure":
                    v = body.get("value")
                    return self._send(200, S.save_exposure(None if v in (None, "") else float(v)))
            except Exception as e:
                return self._send(400, {"error": str(e)})
            return self._send(404, {"error": "not found"})
    return H


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--fake", action="store_true", help="simulated board and robots (no cameras, nothing sent)")
    ap.add_argument("--no-side", action="store_true", help="leave the side camera alone")
    ap.add_argument("--out", help="logs folder (default logs/<date-time>-p0-camera-setup, uploaded by run-fleet-pi.sh stop)")
    a = ap.parse_args(argv)
    out = a.out or os.path.join(REPO, "logs", time.strftime("%Y%m%d-%H%M%S") + "-p0-camera-setup" + ("-dryrun" if a.fake else ""))
    srcs = []
    if a.fake:
        world = FakeWorld()
        over_src = FakeSource(world, "overhead", 25.0)
        side_src = None if a.no_side else FakeSource(world, "side", 15.0, seed=2)
        robots = world
    else:
        import camnames
        od, words = camnames.resolve("overhead")
        print(words)
        if od is None:
            print("Stopped: no overhead camera found by its name.")
            return 1
        try:
            over_src = LiveSource(od)
        except RuntimeError as e:
            print("Stopped: %s\nIf the preview or recorder is still running, stop it first (Ctrl+C in its window, or "
                  "tmux kill-window -t fleet:rec), then start this again." % e)
            return 1
        side_src = None
        if not a.no_side:
            sd, words = camnames.resolve("side")
            print(words)
            if sd is not None and sd != od:
                try:
                    side_src = LiveSource(sd)
                except RuntimeError as e:
                    print("Side camera not opened (%s): the page goes on without it." % e)
        robots = UdpRobots()
    srcs = [s for s in (over_src, side_src) if s]
    over = Grabber("overhead", over_src)
    side = Grabber("side", side_src) if side_src else None
    for g in (over, side):
        if g:
            g.start()
    S = Setup(over, side, robots, out, fake=a.fake)
    print(S.old_words)
    try:
        S.grab()
        print(S.found_words)
    except RuntimeError as e:
        print("Stopped: %s" % e)
        return 1
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(S))
    print("Camera setup page: http://100.122.222.75:%d (Tailscale) or http://<the Pi>:%d. Logs: %s" % (a.port, a.port, out))
    print("Stop it with Ctrl+C (the overhead camera goes back to automatic exposure).", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        srv.server_close()
        for g in (over, side):
            if g:
                g.stopping = True
        time.sleep(0.2)
        try:
            over_src.restore_auto()
        except Exception:
            pass
        for s in srcs:
            s.release()
        print(SC.overhead_calib()[1])
        v, _ = SC.overhead_exposure()
        print("Overhead exposure for the next recorder start: %s" % ("%g" % v if v is not None else "the camera's own (automatic)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
