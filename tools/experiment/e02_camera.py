"""E02 camera side: the floor calibration (load the studio's, or make one from the board's corners),
the blocked boards' floor lines, a motion tracker for the one robot that moves, and the Pi stream.

Why a motion tracker and not the blink names (tools/live): in E02 only one robot moves at a time and
the laptop always knows which one, so "the thing that moves" is that robot. The real-footage session
(docs/session-notes/2026-10-21-real-footage.md) found moving robots 93-100% of the time by motion,
while blink naming on real footage named Left and Center too rarely to give positions. Blink naming is
still recorded once, in the 2-minute naming check, as data.

How the tracker keeps parked robots visible: the background (empty floor) is updated slowly everywhere
EXCEPT around every robot's last known position, so a parked robot never fades into the background, and
the floor under a robot is the floor seen before it arrived. A robot that was already standing there when
the background was first taken is "acquired" by a short drive (acquire_robot in e02.py): once it has
moved off its spot, that spot's floor is copied into the background.

Floor positions are the turning centre's, taken as the centre of the robot's outline seen from above at
the top's height (3.7 cm); its heading comes from how it moves (e02_steer.heading_from_track).
"""
import json
import math
import os
import socket
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (TOOLS, os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

from cameras import CameraModel  # noqa: E402
import floorcal  # noqa: E402

ROBOT_TOP_CM = 3.7          # the camera sees the robot's top (tools/fusion/runs.py ALVIK_TOP_CM)
ROBOT_CM = 9.5
BOARD_W_CM, BOARD_L_CM = 122.15, 121.9   # x edges, y edges (2026-10-21 real-footage calibration)
CAMERA_NAMES = ("overhead", "cam0", "default")


# =============================================================================== calibration
def default_project():
    docs = os.path.join(os.path.expanduser("~"), "Documents")
    return os.path.join(docs if os.path.isdir(docs) else os.path.expanduser("~"), "recognition-project")


def find_calibration(given=None, project=None, session_dir=None):
    """The floor calibration to use: given path, else the session's own cameras/overhead.json (made here
    earlier), else the studio project's cameras/<overhead|cam0|default>.json. Returns a path or None."""
    if given:
        return given if os.path.exists(given) else None
    cands = []
    if session_dir:
        cands.append(os.path.join(session_dir, "cameras", "overhead.json"))
    proj = project or default_project()
    cands += [os.path.join(proj, "cameras", n + ".json") for n in CAMERA_NAMES]
    for c in cands:
        if os.path.exists(c):
            return c
    return None


def load_camera(path):
    """CameraModel from a studio camera file (cameras/<name>.json) or a floor_calibration.json."""
    with open(path) as f:
        d = json.load(f)
    if "K" in d and "rvec" in d:
        return CameraModel.from_dict(d)
    return CameraModel.from_floor_calibration(path)


def write_floor_calibration(cam, path):
    """A floor_calibration.json (homography form, as tools/live/floormap.py reads) from a CameraModel, for
    the blink naming check."""
    R = cam.R
    H = cam.K @ np.column_stack([R[:, 0], R[:, 1], cam.tvec])
    if cam.mirror_y:
        H = H @ np.diag([1.0, -1.0, 1.0])
    H = H / H[2, 2]
    C = cam.center
    d = dict(note="made by tools/experiment/e02_camera.py from %s" % (cam.name or "a camera model"),
             H_floor_to_px=H.tolist(), image_size=list(cam.image_size),
             k1=float(cam.dist[0]), f_px=float(cam.f), cx=float(cam.K[0, 2]), cy=float(cam.K[1, 2]),
             camera=dict(height_cm=float(C[2]), nadir_floor_cm=[float(C[0]), float(C[1])]))
    with open(path, "w") as f:
        json.dump(d, f, indent=1)
    return path


def find_board_corners(img):
    """The plywood board's 4 corners in a grey picture, ordered for floorcal's rectangle: corner 1 = the
    floor origin at the bottom-right of the picture, 2 = top-right (along x, up the picture), 3 = top-left,
    4 = bottom-left (along y, to the left), as in Nick's frame (real-footage notes). Returns
    (corners [[u, v] x 4], quality text) or (None, why)."""
    g = cv2.GaussianBlur(img, (5, 5), 0)
    _, th = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    H, W = img.shape[:2]
    best = None
    for mask in (th, 255 - th):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for c in cnts:
            a = cv2.contourArea(c)
            if a < 0.15 * W * H or a > 0.97 * W * H:
                continue
            if cv2.pointPolygonTest(c, (W / 2, H / 2), False) < 0:
                continue
            hull = cv2.convexHull(c)
            approx = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True)
            if len(approx) == 4 and (best is None or a > best[0]):
                best = (a, approx.reshape(4, 2).astype(float), c.reshape(-1, 2).astype(float))
    if best is None:
        return None, "could not find a 4-sided board in the picture"
    _, q, pts = best
    # refine each side: fit a line to the outline points near it, intersect neighbours
    lines = []
    for i in range(4):
        a, b = q[i], q[(i + 1) % 4]
        d = b - a
        L = np.hypot(*d)
        n = np.array([-d[1], d[0]]) / L
        s = (pts - a) @ (d / L)
        off = np.abs((pts - a) @ n)
        sel = pts[(off < 4) & (s > 0.1 * L) & (s < 0.9 * L)]
        if len(sel) < 10:
            lines.append((a, d / L))
            continue
        vx, vy, x0, y0 = cv2.fitLine(sel.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        lines.append((np.array([x0, y0]), np.array([vx, vy])))
    cor = []
    for i in range(4):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        A = np.array([d1, -d2]).T
        try:
            s = np.linalg.solve(A, p2 - p1)
            cor.append(p1 + s[0] * d1)
        except np.linalg.LinAlgError:
            cor.append(q[i])
    cor = np.array(cor)
    sm, df = cor.sum(1), cor[:, 0] - cor[:, 1]
    br, tl = cor[np.argmax(sm)], cor[np.argmin(sm)]
    tr, bl = cor[np.argmax(df)], cor[np.argmin(df)]
    out = [br, tr, tl, bl]
    if len({tuple(np.round(c, 1)) for c in out}) < 4:
        return None, "the corners found are not clearly separate"
    return [[round(float(u), 1), round(float(v), 1)] for u, v in out], "found"


def make_calibration(corners, image_size, path, width_cm=BOARD_W_CM, length_cm=BOARD_L_CM, height_cm=None):
    """Fit and save a camera model from the board corners (floorcal's rectangle method). Returns the
    floorcal result dict (ok, err_cm, text, model ...)."""
    spec = dict(mode="rectangle", corners_px=corners, width_cm=width_cm, length_cm=length_cm)
    if height_cm:
        spec["height_cm"] = height_cm
    res = floorcal.fit(spec, image_size)
    d = res["model"].to_dict()
    d["name"] = "overhead"
    d["studio_calibration"] = dict(spec=res["spec"], mode="rectangle", err_cm=res["err_cm"],
                                   max_err_cm=res["max_err_cm"], mirrored=res["mirrored"], points=res["points"],
                                   note="made by tools/experiment/e02.py from the board corners found in the picture")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(d, f, indent=1)
    return res


def draw_check(img, cam, path, board=None, corners=None, robots=None):
    """A picture for Nick to check by eye: the board outline from the calibration (green), the found
    corners (red), the blocked boards' measured lines (yellow), robots (cyan). Saved as PNG."""
    out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()

    def poly(pts, col, th=1):
        uv = [cam.project((x, y, 0.0)) for x, y in pts]
        uv = [(int(round(u)), int(round(v))) for u, v in uv if np.isfinite(u) and np.isfinite(v)]
        for a, b in zip(uv[:-1], uv[1:]):
            cv2.line(out, a, b, col, th, cv2.LINE_AA)
    if board is not None:
        x0, y0, x1, y1 = board.x0, board.y0, board.x1, board.y1
        n = 20
        edge = [(x0 + (x1 - x0) * k / n, y0) for k in range(n + 1)] + [(x1, y0 + (y1 - y0) * k / n) for k in range(n + 1)]
        edge += [(x1 - (x1 - x0) * k / n, y1) for k in range(n + 1)] + [(x0, y1 - (y1 - y0) * k / n) for k in range(n + 1)]
        poly(edge, (0, 200, 0))
        poly([(x0 + (x1 - x0) * k / n, board.xboard_cm) for k in range(n + 1)], (0, 220, 255), 2)
        poly([(board.yboard_cm, y0 + (y1 - y0) * k / n) for k in range(n + 1)], (0, 220, 255), 2)
        u, v = cam.project((x0 + 3, y0 + 3, 0.0))
        cv2.putText(out, "0,0", (int(u), int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)
    for c in corners or []:
        cv2.circle(out, (int(round(c[0])), int(round(c[1]))), 4, (0, 0, 255), 1)
    for name, (x, y) in (robots or {}).items():
        u, v = cam.project((x, y, ROBOT_TOP_CM))
        cv2.circle(out, (int(u), int(v)), 8, (255, 255, 0), 1)
        cv2.putText(out, name, (int(u) + 9, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    cv2.imwrite(path, out)
    return path


_SHOWN = {"win": None}


def show_picture(path, title):
    """Put a picture on screen for Nick, without blocking: Windows opens it in the Photos viewer; elsewhere an
    OpenCV window. Returns False if there is no screen to show it on (the caller then names the file)."""
    try:
        if sys.platform.startswith("win"):
            os.startfile(path)            # noqa: the default picture viewer
            return True
        img = cv2.imread(path)
        if img is None or not (os.environ.get("DISPLAY") or sys.platform == "darwin"):
            return False
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.imshow(title, img)
        cv2.waitKey(300)
        _SHOWN["win"] = title
        return True
    except Exception:
        return False


def close_picture():
    if _SHOWN["win"]:
        try:
            cv2.destroyWindow(_SHOWN["win"])
            cv2.waitKey(1)
        except Exception:
            pass
        _SHOWN["win"] = None


def picture_side(cam, x, y):
    """Which side of the picture a floor point is nearest: 'top', 'bottom', 'left', 'right'."""
    u, v = cam.project((x, y, 0.0))
    W, H = cam.image_size
    d = dict(left=u, right=W - u, top=v, bottom=H - v)
    return min(d, key=d.get)


# =============================================================================== board lines
def _sample(img, u, v):
    return cv2.getRectSubPix(img, (1, 1), (float(u), float(v)))[0, 0]


def measure_board_line(img, cam, board, which, n_profiles=17, inside_cm=12.0, outside_cm=4.0, step_cm=0.25):
    """Where the blocked board's inner face meets the floor, cm along the other axis: the first strong
    brightness edge met when going from the board's middle outwards, along n_profiles lines across the
    edge. which: 'x' (the x-board, a line y = value) or 'y' (the y-board, x = value).
    Returns dict(value, spread, n, values) or None when too few profiles found an edge."""
    f = img.astype(np.float32)
    f = cv2.GaussianBlur(f, (3, 3), 0)
    if which == "x":
        nominal = board.y0 if board.xboard_side == 0 else board.y1
        inward = board.uy
        lo, hi = sorted((board.x0, board.x1))
    else:
        nominal = board.x0 if board.yboard_side == 0 else board.x1
        inward = board.ux
        lo, hi = sorted((board.y0, board.y1))
    along = np.linspace(lo + 15.0, hi - 15.0, n_profiles)
    offs = np.arange(inside_cm, -outside_cm - 1e-9, -step_cm)      # from inside outwards
    found = []
    for a in along:
        vals = []
        for o in offs:
            c = nominal + inward * o
            x, y = (a, c) if which == "x" else (c, a)
            u, v = cam.project((x, y, 0.0))
            vals.append(_sample(f, u, v) if (0 <= u < f.shape[1] - 1 and 0 <= v < f.shape[0] - 1) else np.nan)
        vals = np.array(vals)
        if np.isnan(vals).any():
            continue
        gr = np.abs(np.gradient(vals))
        floor_sd = float(np.std(vals[: max(4, int(4.0 / step_cm))]))
        thr = max(6.0, 4.0 * floor_sd, 0.4 * float(gr.max()))
        k = next((i for i in range(2, len(gr) - 1) if gr[i] >= thr and gr[i] >= gr[i - 1] and gr[i] >= gr[i + 1]), None)
        if k is None:
            continue
        # sub-sample peak
        den = gr[k - 1] - 2 * gr[k] + gr[k + 1]
        dk = 0.5 * (gr[k - 1] - gr[k + 1]) / den if abs(den) > 1e-9 else 0.0
        o = offs[k] - step_cm * dk
        found.append(nominal + inward * o)
    if len(found) < max(5, n_profiles // 2):
        return None
    vals = np.array(found)
    med = float(np.median(vals))
    spread = float(1.4826 * np.median(np.abs(vals - med)))
    return dict(value=round(med, 2), spread=round(spread, 2), n=len(found), values=[round(v, 2) for v in vals])


# =============================================================================== the tracker
class MotionTracker:
    """Finds and follows the one moving robot in grey frames. Thread-safe (one lock)."""

    def __init__(self, cam, board_center=(61.0, 61.0), thr=22.0, alpha=0.03, z_cm=ROBOT_TOP_CM, max_speed_cms=40.0):
        self.cam = cam
        self.z = z_cm
        self.rpx = ROBOT_CM * cam.px_per_cm(board_center[0], board_center[1], z_cm)
        self.area = self.rpx ** 2
        self.thr = thr
        self.alpha = alpha
        self.vmax = max_speed_cms
        self.bg = None
        self.protect = {}           # robot -> (x, y) floor cm: never faded into the background
        self.active = None          # robot being followed
        self.pos = None             # (t, x, y) its latest camera position
        self.last_seen = 0.0
        self.misses = 0
        self.lock = threading.Lock()
        self.frozen = False         # True: the background is not updated (while finding a robot by a short drive)
        self.frame = None           # latest blurred frame (float32)
        self.frame_t = None
        self.k_open = np.ones((3, 3), np.uint8)
        self.k_close = np.ones((5, 5), np.uint8)

    def _px(self, x, y):
        return self.cam.project((x, y, self.z))

    def _floor(self, u, v):
        return self.cam.to_floor(u, v, self.z)

    def _protect_mask(self, shape):
        m = np.zeros(shape, np.uint8)
        r = int(round(0.95 * self.rpx))
        for x, y in self.protect.values():
            u, v = self._px(x, y)
            if np.isfinite(u) and np.isfinite(v):
                cv2.circle(m, (int(round(u)), int(round(v))), r, 255, -1)
        return m

    def blobs(self, g=None):
        """Robot-sized things that differ from the background: [(u, v, area, x, y)]."""
        g = self.frame if g is None else g
        if g is None or self.bg is None:
            return []
        diff = cv2.absdiff(g, self.bg)
        mask = (diff > self.thr).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.k_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.k_close)
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        out = []
        for i in range(1, n):
            a = stats[i, cv2.CC_STAT_AREA]
            if 0.3 * self.area <= a <= 3.5 * self.area:
                u, v = cents[i]
                x, y = self._floor(u, v)
                out.append((float(u), float(v), int(a), float(x), float(y)))
        return out

    def push(self, img, t):
        """One frame (grey uint8) taken at laptop time t. Returns the active robot's (t, x, y) or None."""
        g = cv2.GaussianBlur(img, (3, 3), 0).astype(np.float32)
        with self.lock:
            self.frame, self.frame_t = g, t
            if self.bg is None:
                self.bg = g.copy()
                return None
            out = None
            if self.active is not None and self.pos is not None:
                _, px, py = self.pos
                dt_ = max(0.0, t - self.pos[0])
                gate_cm = ROBOT_CM * 0.9 + self.vmax * min(dt_, 1.0)
                best = None
                for u, v, a, x, y in self.blobs(g):
                    d = math.hypot(x - px, y - py)
                    if d <= gate_cm and (best is None or d < best[0]):
                        best = (d, x, y)
                if best is not None:
                    self.pos = (t, best[1], best[2])
                    self.protect[self.active] = (best[1], best[2])
                    self.last_seen = t
                    self.misses = 0
                    out = self.pos
                else:
                    self.misses += 1
            if self.frozen:
                return out
            keep = self._protect_mask(g.shape)
            cv2.accumulateWeighted(g, self.bg, self.alpha, mask=cv2.bitwise_not(keep))
            return out

    # --- control ------------------------------------------------------------------------------
    def set_background(self, frames):
        """A fresh background from a few frames (median), while nothing moves."""
        fs = [cv2.GaussianBlur(f, (3, 3), 0).astype(np.float32) for f in frames]
        with self.lock:
            self.bg = np.median(np.stack(fs), axis=0).astype(np.float32)

    def refresh_spot(self, x, y, scale=1.0):
        """Copy the current frame into the background around a floor spot (a robot just left it)."""
        with self.lock:
            if self.frame is None or self.bg is None:
                return
            m = np.zeros(self.bg.shape, np.uint8)
            u, v = self._px(x, y)
            cv2.circle(m, (int(round(u)), int(round(v))), int(round(0.85 * self.rpx * scale)), 255, -1)
            self.bg[m > 0] = self.frame[m > 0]

    def follow(self, robot, x, y, t=None):
        """Follow `robot`, now at (x, y)."""
        with self.lock:
            self.active = robot
            self.pos = (time.time() if t is None else t, x, y)
            self.protect[robot] = (x, y)
            self.last_seen = self.pos[0]
            self.misses = 0

    def park(self, robot, x, y):
        """A robot stays put at (x, y) (kept out of the background)."""
        with self.lock:
            self.protect[robot] = (x, y)

    def idle(self):
        """Follow nobody for now (parked robots stay protected)."""
        with self.lock:
            self.active, self.pos = None, None

    def forget(self, robot):
        with self.lock:
            self.protect.pop(robot, None)
            if self.active == robot:
                self.active, self.pos = None, None

    def where(self):
        with self.lock:
            return self.pos


class History:
    """The active robot's camera samples, for headings, stillness and the run's track file."""

    def __init__(self, keep_s=600.0):
        self.q = deque()
        self.keep = keep_s
        self.lock = threading.Lock()

    def add(self, s):
        with self.lock:
            self.q.append(s)
            while self.q and self.q[0][0] < s[0] - self.keep:
                self.q.popleft()

    def since(self, t0, t1=None):
        with self.lock:
            return [s for s in self.q if s[0] >= t0 and (t1 is None or s[0] <= t1)]

    def clear(self):
        with self.lock:
            self.q.clear()


def still_position(samples, min_s=0.4, max_spread_cm=0.4):
    """Median position of the last min_s of samples if the robot stood still over it, else None."""
    if not samples:
        return None
    t1 = samples[-1][0]
    win = [s for s in samples if s[0] >= t1 - min_s]
    if len(win) < 3 or t1 - win[0][0] < 0.6 * min_s:
        return None
    xs = np.array([s[1] for s in win])
    ys = np.array([s[2] for s in win])
    if max(np.ptp(xs), np.ptp(ys)) > max_spread_cm * 2.5:
        return None
    return float(np.median(xs)), float(np.median(ys))


# =============================================================================== the Pi stream
class Feed(threading.Thread):
    """Reads the overhead camera's /live stream (tools/live/sources.StreamSource), turns the Pi's time into
    laptop time (ClockEnvelope, minus the capture delay) and pushes every frame to the tracker."""

    def __init__(self, url, tracker, history, capture_delay_s=0.04, log=print, source=None):
        super().__init__(daemon=True)
        import sources as S
        self.S = S
        self.src = source or S.StreamSource(url, log=log)
        self.tracker = tracker
        self.history = history
        self.clock = S.ClockEnvelope()
        self.cap = capture_delay_s
        self.frames = 0
        self.last_t = None
        self.last_img = None
        self.recent = deque(maxlen=30)
        self.cond = threading.Condition()
        self.stopped = False
        self.err = None
        self.hooks = []              # functions(img, t) called for every frame (the naming check)
        self.on_sample = None        # function((t, x, y)) for every position of the followed robot

    def run(self):
        try:
            for fr in self.src:
                if self.stopped:
                    break
                self.clock.add(fr.t_arrived, fr.t_pi)
                t = fr.t_pi + self.clock.offset - self.cap
                for h in list(self.hooks):
                    h(fr.img, t)
                s = self.tracker.push(fr.img, t) if self.tracker is not None else None
                if s is not None:
                    self.history.add(s)
                    if self.on_sample:
                        self.on_sample(s)
                with self.cond:
                    self.frames += 1
                    self.last_t = t
                    self.last_img = fr.img
                    self.recent.append(fr.img)
                    self.cond.notify_all()
        except Exception as e:      # keep the reason for the console
            self.err = e

    def wait_frames(self, n, timeout=10.0):
        """Wait for n new frames; returns them (grey images) or fewer on timeout."""
        out = []
        end = time.time() + timeout
        with self.cond:
            seen = self.frames
            while len(out) < n and time.time() < end:
                self.cond.wait(0.2)
                if self.frames > seen:
                    out.append(self.last_img)
                    seen = self.frames
        return out

    def close(self):
        self.stopped = True
        try:
            self.src.close()
        except Exception:
            pass


def check_stream(host, port, timeout=6.0, n=5):
    """Is a camera at http://host:port/live sending frames? Returns (ok, words, (W, H) or None, fps)."""
    import sources as S
    src = S.StreamSource("%s:%d" % (host, port), timeout=timeout, log=lambda *_: None)
    got, t0, size = 0, time.time(), None
    result = [None]

    def reader():
        nonlocal got, size
        try:
            for fr in src:
                got += 1
                size = (fr.img.shape[1], fr.img.shape[0])
                if got >= n:
                    break
        except Exception as e:
            result[0] = e
    th = threading.Thread(target=reader, daemon=True)
    th.start()
    th.join(timeout)
    src.close()
    if got == 0:
        return False, "no pictures from %s:%d (%s)" % (host, port, result[0] or "nothing within %.0f s" % timeout), None, 0.0
    el = time.time() - t0
    return True, "%d pictures in %.1f s" % (got, el), size, got / el if el > 0 else 0.0


def notes_answer(host, port=5007, text="e02 check", timeout=0.5):
    """Does the Pi recorder answer a note (it replies 'ok ...')?"""
    try:
        addr = (socket.gethostbyname(host), port)
    except OSError:
        return False
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        for _ in range(3):
            try:
                s.sendto(text.encode(), addr)
                data, _ = s.recvfrom(512)
                if data.startswith(b"ok "):
                    return True
            except OSError:
                pass
    finally:
        s.close()
    return False
