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
    """Finds and follows the one moving robot in grey frames, against a FLOOR MEMORY (Nick, 2026-10-21): what
    the floor looks like at every spot, kept fresh wherever the camera sees it with no robot on it. Thread-safe.

    Why (the first real session, 2026-10-21): the old tracker compared every frame with one picture taken at the
    start, patched only where a robot drove away; robots put down by hand, a hand in the picture or the spot a
    robot had just left all looked like robots, and it followed them. Now:
      - floor memory: pixels no known robot covers and that have stood still for STABLE_FRAMES are copied into
        the floor (new floor at once, small drift slowly); under a known robot the floor is UNKNOWN until it
        leaves: filled in from the floor around it (inpaint) and only trusted with a higher threshold, so the
        spot a robot leaves never looks like a robot, and the robot never vanishes into the floor;
      - a robot-shape check: size and squareness of the outline (a hand, a long smear, a lighting patch fail);
      - the board area: nothing further than margin_cm outside the board is a robot (the levels, the shelf);
      - motion: the frame-to-frame change says which outline is moving now;
      - a whole-picture brightness shift (the camera's auto exposure) is taken out before comparing;
      - gating: the followed robot's next position must be near its last one (speed limit) and near where the
        runner expects it (expect(): from the leg the robot was told to drive), so it can't jump to something else.
    Positions are the outline's centre on the plane of the robot's top (ROBOT_TOP_CM)."""

    STABLE_FRAMES = 12          # a pixel this many frames without change (~0.5 s) can be learned as floor
    MOTION_THR = 9.0            # grey levels of frame-to-frame change that count as motion
    UNKNOWN_EXTRA = 14.0        # extra threshold on floor that was filled in, not seen
    ABSORB_S = 25.0             # something new standing still this long, where no robot is, becomes floor

    def __init__(self, cam, board_center=(61.0, 61.0), thr=16.0, alpha=0.02, z_cm=ROBOT_TOP_CM, max_speed_cms=40.0,
                 board_rect=None, margin_cm=8.0):
        self.cam = cam
        self.z = z_cm
        self.rpx = ROBOT_CM * cam.px_per_cm(board_center[0], board_center[1], z_cm)
        self.area = self.rpx ** 2
        self.thr_min = thr
        self.thr = thr
        self.alpha = alpha
        self.vmax = max_speed_cms
        self.board_rect = board_rect            # (x0, y0, x1, y1) cm, or None: anywhere
        self.margin = margin_cm
        self.bg = None
        self.known = None                       # uint8: 1 = floor seen, 0 = filled in (under a robot at the start)
        self.stable = None                      # int16: frames without change
        self.still_new = None                   # float32: seconds a pixel has differed from the floor while still
        self.prev = None
        self.protect = {}                       # robot -> (x, y) floor cm: covered, never learned as floor
        self.active = None
        self.pos = None
        self.hint = None                        # (x, y, radius cm, until t): where the runner expects the robot
        self.last_seen = 0.0
        self.misses = 0
        self.lock = threading.Lock()
        self.frozen = False
        self.frame = None
        self.frame_t = None
        self.offset = 0.0
        self.k_open = np.ones((3, 3), np.uint8)
        self.k_close = np.ones((7, 7), np.uint8)
        self.last_cands = []
        self.n = 0

    # --- geometry --------------------------------------------------------------------------------
    def _px(self, x, y):
        return self.cam.project((x, y, self.z))

    def _floor(self, u, v):
        return self.cam.to_floor(u, v, self.z)

    def on_area(self, x, y):
        if self.board_rect is None:
            return True
        x0, y0, x1, y1 = self.board_rect
        m = self.margin
        return min(x0, x1) - m <= x <= max(x0, x1) + m and min(y0, y1) - m <= y <= max(y0, y1) + m

    def _disk_mask(self, shape, pts, r_scale):
        m = np.zeros(shape, np.uint8)
        r = int(round(r_scale * self.rpx))
        for x, y in pts:
            u, v = self._px(x, y)
            if np.isfinite(u) and np.isfinite(v):
                cv2.circle(m, (int(round(u)), int(round(v))), r, 1, -1)
        return m

    def _covered(self, shape):
        pts = [q for r, q in self.protect.items() if r != self.active]
        m = self._disk_mask(shape, pts, 1.25)
        if self.active is not None and self.pos is not None:
            # the followed robot: just its body (and halo), so the floor it has just uncovered is learned at once
            m |= self._disk_mask(shape, [(self.pos[1], self.pos[2])], 1.0)
        if self.active is not None and self.pos is not None and self.frame_t is not None and \
                self.frame_t - self.last_seen > 0.3:
            # lost for a moment: don't learn anything near where it may be now
            m |= self._disk_mask(shape, [(self.pos[1], self.pos[2])], 1.25 + self.vmax * min(3.0, self.frame_t - self.last_seen) / ROBOT_CM)
        return m

    # --- what differs from the floor --------------------------------------------------------------
    def _diff(self, g):
        d = np.abs(g - self.offset - self.bg)
        lim = np.where(self.known > 0, self.thr, self.thr + self.UNKNOWN_EXTRA).astype(np.float32)
        return d, (d > lim).astype(np.uint8) * 255

    def _shape_ok(self, area, w, h):
        a, r = self.area, self.rpx
        lo, hi = sorted((w, h))
        return (0.3 * a <= area <= 2.6 * a and hi <= 2.0 * r and lo >= 0.4 * r and hi / max(lo, 1.0) <= 2.3
                and area / max(w * h, 1.0) >= 0.3)

    def candidates(self, g=None, motion=None):
        """Every outline that differs from the floor: dicts u, v, area, x, y, shape (robot-shaped and on the
        board area), moving (share of it moving now)."""
        g = self.frame if g is None else g
        if g is None or self.bg is None:
            return []
        _, mask = self._diff(g)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.k_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.k_close)
        n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        out = []
        for i in range(1, n):
            a = int(stats[i, cv2.CC_STAT_AREA])
            if a < 0.15 * self.area:
                continue
            w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
            u, v = cents[i]
            x, y = self._floor(u, v)
            if not (np.isfinite(x) and np.isfinite(y)):
                continue
            ys, xs = np.nonzero(lab[stats[i, 1]:stats[i, 1] + h, stats[i, 0]:stats[i, 0] + w] == i)
            if len(xs) >= 5:
                (_, _), (rw, rh), _ = cv2.minAreaRect(np.column_stack([xs, ys]).astype(np.float32))
            else:
                rw, rh = w, h
            mv = 0.0
            if motion is not None:
                sub = motion[stats[i, 1]:stats[i, 1] + h, stats[i, 0]:stats[i, 0] + w]
                mv = float((sub[ys, xs] > 0).mean()) if len(xs) else 0.0
            out.append(dict(u=float(u), v=float(v), area=a, x=float(x), y=float(y), moving=mv,
                            shape=bool(self._shape_ok(a, rw, rh) and self.on_area(x, y))))
        return out

    def blobs(self, g=None):
        """Robot-shaped things on the board area that differ from the floor: [(u, v, area, x, y)]."""
        with self.lock:
            return [(c["u"], c["v"], c["area"], c["x"], c["y"]) for c in self.candidates(g) if c["shape"]]

    # --- every frame -------------------------------------------------------------------------------
    def push(self, img, t):
        """One frame (grey uint8) taken at laptop time t. Returns the active robot's (t, x, y) or None."""
        g = cv2.GaussianBlur(img, (3, 3), 0).astype(np.float32)
        with self.lock:
            self.n += 1
            dt_frame = (t - self.frame_t) if self.frame_t is not None else 0.04
            self.frame, self.frame_t = g, t
            if self.bg is None:
                self._init_floor([g])
                self.prev = g
                return None
            if self.prev is None or self.prev.shape != g.shape:
                self.prev = g
            motion = (np.abs(g - self.prev) > self.MOTION_THR).astype(np.uint8)
            self.prev = g
            self.stable = np.where(motion > 0, 0, np.minimum(self.stable + 1, 30000)).astype(np.int16)
            if self.n % 5 == 0:
                self._update_offset(g)
            out = None
            if self.active is not None and self.pos is not None:
                cands = self.candidates(g, motion)
                self.last_cands = cands
                _, px, py = self.pos
                dt_ = max(0.0, t - self.pos[0])
                gate_cm = ROBOT_CM * 0.9 + self.vmax * min(dt_, 1.0)
                hint = self.hint if (self.hint and t <= self.hint[3]) else None
                best = None
                others = [q for r, q in self.protect.items() if r != self.active]
                for c in cands:
                    if not c["shape"]:
                        continue
                    if any(math.hypot(c["x"] - q[0], c["y"] - q[1]) < 0.8 * ROBOT_CM for q in others):
                        continue                 # that is another robot standing still
                    d = math.hypot(c["x"] - px, c["y"] - py)
                    if d > gate_cm:
                        continue
                    if hint and math.hypot(c["x"] - hint[0], c["y"] - hint[1]) > hint[2]:
                        continue
                    score = d - 3.0 * c["moving"]
                    if best is None or score < best[0]:
                        best = (score, c["x"], c["y"])
                if best is not None:
                    self.pos = (t, best[1], best[2])
                    self.protect[self.active] = (best[1], best[2])
                    self.last_seen = t
                    self.misses = 0
                    out = self.pos
                else:
                    self.misses += 1
            if not self.frozen:
                self._learn(g, dt_frame)
            return out

    def _init_floor(self, gs):
        self.bg = np.median(np.stack(gs), axis=0).astype(np.float32) if len(gs) > 1 else gs[0].copy()
        self.known = np.ones(self.bg.shape, np.uint8)
        self.stable = np.zeros(self.bg.shape, np.int16)
        self.still_new = np.zeros(self.bg.shape, np.float32)
        self.offset = 0.0

    def _update_offset(self, g):
        """The camera's auto exposure shifts the whole picture: the median change over the seen floor."""
        k = (self.known[::4, ::4] > 0)
        if k.sum() < 100:
            return
        d = (g[::4, ::4] - self.bg[::4, ::4])[k]
        self.offset = float(np.median(d))
        mad = float(np.median(np.abs(d - self.offset)))
        self.thr = max(self.thr_min, 5.0 * 1.4826 * mad)

    def _learn(self, g, dt_frame):
        cov = self._covered(g.shape)
        ok = (cov == 0) & (self.stable >= self.STABLE_FRAMES)
        gc = g - self.offset
        d = np.abs(gc - self.bg)
        new_floor = ok & (self.known == 0)                       # a robot has left: its floor, seen at last
        self.bg[new_floor] = gc[new_floor]
        self.known[new_floor] = 1
        same = ok & (self.known > 0) & (d <= self.thr)
        if same.any():
            self.bg[same] += self.alpha * (gc[same] - self.bg[same])
        changed = ok & (self.known > 0) & (d > self.thr)
        self.still_new[changed] += dt_frame
        self.still_new[~changed] = 0.0
        absorb = self.still_new > self.ABSORB_S
        if absorb.any():
            self.bg[absorb] = gc[absorb]
            self.still_new[absorb] = 0.0

    # --- control ------------------------------------------------------------------------------
    def set_background(self, frames, robots=None):
        """A fresh floor memory from a few frames (median) while nothing moves. robots: floor positions of
        robots standing in them (their floor is filled in from around them until they leave)."""
        fs = [cv2.GaussianBlur(f, (3, 3), 0).astype(np.float32) for f in frames]
        with self.lock:
            self._init_floor(fs)
            for x, y in (robots or []):
                self._unknown_at(x, y)

    def _unknown_at(self, x, y, scale=1.2):
        m = self._disk_mask(self.bg.shape, [(x, y)], scale)
        self.known[m > 0] = 0
        src = np.clip(self.bg, 0, 255).astype(np.uint8)
        filled = cv2.inpaint(src, (self.known == 0).astype(np.uint8) * 255, 5, cv2.INPAINT_TELEA)
        self.bg[self.known == 0] = filled[self.known == 0].astype(np.float32)

    def robot_at(self, x, y):
        """A robot stands at (x, y) on what the floor memory thinks is floor (it was there when the picture was
        taken, or put down since): its floor becomes unknown until it leaves."""
        with self.lock:
            if self.bg is not None:
                self._unknown_at(x, y)

    def refresh_spot(self, x, y, scale=1.0):
        """Copy the current frame into the floor around a spot (nothing stands there now)."""
        with self.lock:
            if self.frame is None or self.bg is None:
                return
            m = self._disk_mask(self.bg.shape, [(x, y)], 0.85 * scale) > 0
            self.bg[m] = self.frame[m] - self.offset
            self.known[m] = 1

    def expect(self, x, y, radius_cm, for_s=10.0):
        """The runner expects the followed robot within radius_cm of (x, y) for the next for_s seconds."""
        with self.lock:
            self.hint = (x, y, radius_cm, time.time() + for_s)

    def clear_expect(self):
        with self.lock:
            self.hint = None

    def hold(self, radius_cm=6.0):
        """The followed robot has been told to stand still: only accept it within radius_cm of where it is now
        (a hand reaching in next to it, or a robot bumped into it, can't pull the tracker away). Only when the
        camera saw it just now: a hold round an old position could never find it again."""
        with self.lock:
            if self.pos is not None and self.frame_t is not None and self.frame_t - self.last_seen < 0.5:
                self.hint = (self.pos[1], self.pos[2], radius_cm, float("inf"))
            else:
                self.hint = None

    def clear_protect(self):
        with self.lock:
            self.protect.clear()

    def follow(self, robot, x, y, t=None):
        with self.lock:
            self.active = robot
            self.pos = (time.time() if t is None else t, x, y)
            self.protect[robot] = (x, y)
            self.last_seen = self.pos[0]
            self.misses = 0
            self.hint = None

    def park(self, robot, x, y):
        with self.lock:
            self.protect[robot] = (x, y)

    def idle(self):
        with self.lock:
            self.active, self.pos, self.hint = None, None, None

    def forget(self, robot):
        with self.lock:
            self.protect.pop(robot, None)
            if self.active == robot:
                self.active, self.pos, self.hint = None, None, None

    def where(self):
        with self.lock:
            return self.pos


def locate_change(cam, before, after, board_rect=None, margin_cm=8.0, near=None, z_cm=ROBOT_TOP_CM, min_frac=0.4):
    """Where did something robot-sized change between two sets of frames (grey, nothing else moving)? For finding
    a robot by a short turn on its spot: the corners of its body move, its spot stays put, so the change sits on
    the robot and nowhere else (no empty spot left behind, unlike a drive). Medians of each set (noise), the whole
    picture's brightness shift taken out. near: (x, y, radius cm) to look only there. Returns a list of dicts
    (x, y, area, strength) best first; [] if nothing robot-sized changed."""
    b = np.median(np.stack([cv2.GaussianBlur(f, (3, 3), 0).astype(np.float32) for f in before]), axis=0)
    a = np.median(np.stack([cv2.GaussianBlur(f, (3, 3), 0).astype(np.float32) for f in after]), axis=0)
    d = a - b
    d -= float(np.median(d[::4, ::4]))
    ad = np.abs(d)
    mad = float(np.median(ad[::4, ::4]))
    thr = max(12.0, 6.0 * 1.4826 * mad)
    mask = (ad > thr).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cx = cy = 61.0
    if board_rect is not None:
        cx, cy = (board_rect[0] + board_rect[2]) / 2, (board_rect[1] + board_rect[3]) / 2
    rpx = ROBOT_CM * cam.px_per_cm(cx, cy, z_cm)
    area0 = rpx ** 2
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, n):
        ar = int(stats[i, cv2.CC_STAT_AREA])
        w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        if ar < 0.1 * area0 or ar > 6.0 * area0 or max(w, h) > 3.6 * rpx:
            continue
        u, v = cents[i]
        x, y = cam.to_floor(u, v, z_cm)
        if not (np.isfinite(x) and np.isfinite(y)):
            continue
        if board_rect is not None:
            x0, y0, x1, y1 = board_rect
            if not (min(x0, x1) - margin_cm <= x <= max(x0, x1) + margin_cm and
                    min(y0, y1) - margin_cm <= y <= max(y0, y1) + margin_cm):
                continue
        if near is not None and math.hypot(x - near[0], y - near[1]) > near[2]:
            continue
        out.append(dict(x=float(x), y=float(y), area=ar, strength=float(ad[lab == i].mean()), u=float(u), v=float(v)))
    # the corners of one turning body can show as separate pieces: pieces closer than a robot's size are one
    merged = []
    for c in sorted(out, key=lambda c: -c["area"] * c["strength"]):
        for m in merged:
            if math.hypot(c["x"] - m["x"], c["y"] - m["y"]) < 1.3 * ROBOT_CM:
                tot = m["area"] + c["area"]
                for k in ("x", "y", "u", "v"):
                    m[k] = (m[k] * m["area"] + c[k] * c["area"]) / tot
                m["strength"] = (m["strength"] * m["area"] + c["strength"] * c["area"]) / tot
                m["area"] = tot
                break
        else:
            merged.append(dict(c))
    merged = [m for m in merged if m["area"] >= min_frac * area0]
    merged.sort(key=lambda c: -c["area"] * c["strength"])
    return merged


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
