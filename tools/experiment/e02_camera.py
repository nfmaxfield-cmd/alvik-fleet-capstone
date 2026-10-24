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
    HELD_MEDIAN_N = 15          # a held robot's position: the mean of this many of its latest real positions
    HOLD_TRUST_S = 4.0          # a held robot counts as seen at its spot for at most this long without its outline
    WALL_CM = 3.0               # a robot centre may be this far past its wall limit (the picture's board lines are 1-2 cm short)
    MOVING_MIN = 0.15           # share of an outline changing frame to frame for it to count as moving
    COVER_SCALE = 1.35          # the followed robot's floor is never learned within this many robot widths of it
    COVER_MOVING = 1.25          # ... and while it moves
    GATE_GROW_S = 0.6           # a lost robot's search circle grows with time for at most this long
    OTHER_CM = 1.1 * ROBOT_CM   # an outline this near another robot's last place is that robot (two touching: 9.5-13)
    MERGE_CM = 0.8 * ROBOT_CM   # robot-shaped outlines this near each other are one robot
    USE_LOCK = True             # follow with the lock-on tracker, this detector checking it
    CHECK_S = 2.0               # chosen (Nick: start at 2 s; a DOE is written up to set it): detector checks the lock
    AGREE_CM = 6.0              # chosen: the detector's outline within this of the lock agrees (a DOE could set it)
    DISAGREE_N = 2              # checks in a row the detector sees it elsewhere, standing still: lock on there again
    LOCK_SURE = 0.7             # chosen: a lock match this good is trusted over the detector's outline (DOE: confidence)
    NUDGE_CM = 3.0              # ... below it, a detector outline further than this from the lock moves the lock
    RELOCK_STILL_N = 12         # chosen: a detector outline still for this many pictures (~0.5 s) can be locked on
    RELOCK_STILL_CM = 1.0       # ... within this
    START_NEAR_CM = 12.0        # the lock starts on the detector's outline within this of the place handed over
    START_WAIT_S = 1.0          # ... waiting at most this long for it, then on the place handed over
    PIN_CM = 3.0                # during a turn on its spot the lock may not move further than this

    def __init__(self, cam, board_center=(61.0, 61.0), thr=16.0, alpha=0.02, z_cm=ROBOT_TOP_CM, max_speed_cms=30.0,
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
        self.hold_ref = None                    # (mask, picture there) when the robot was told to stand still
        self.disturbed = False                  # something moved over a robot that should stand still
        self.moved_by_hand = 0                  # times a held robot was found moved (it is then "lost")
        self.hand_moved = set()                 # robots found moved while held (the runner clears one when it is back)
        self.last_real = 0.0                    # when the followed robot's outline was last really seen
        self.held_hist = deque(maxlen=self.HELD_MEDIAN_N)   # its real positions since it was told to stand still
        # the lock-on tracker (e02_lock.py, Nick 2026-10-22/23): handed the followed robot by this detector, it
        # follows that robot's own look; this detector checks it every CHECK_S and at every stop
        self.use_lock = self.USE_LOCK
        self.lk = None
        self.lock_start = None                  # (x, y): lock on here at the next picture
        self.lock_retake = False                # a turn on its spot just ended: take its look again
        self.lock_off = (0.0, 0.0)              # the detector's outline centre minus the lock's (learned)
        self.lock_start_t = None
        self.det_hist = deque(maxlen=50)        # the detector's own outline, picture by picture (None: not seen)
        self.leg_kind = None                    # "turn" / "go" while the runner drives a leg
        self.check_t = 0.0
        self.check_hold = False                 # a check is due at the next picture (it just stopped)
        self.disagree_n = 0
        self.source = None                      # where the last position came from: lock / detector / held
        self.stats = dict(both=0, lock=0, detector=0, held=0, none=0, checks=0, agree=0, disagree=0, relocks=0)
        self.last_check = None                  # (t, distance cm or None)

    # --- geometry --------------------------------------------------------------------------------
    def _px(self, x, y):
        u, v = self.cam.project((x, y, self.z))
        du, dv = getattr(self, "px_shift", (0.0, 0.0))
        return u + du, v + dv

    def _floor(self, u, v):
        # the camera's picture creeps (2026-10-24: up to 2 px over a session, the mount): CameraShiftWatch measures
        # it against the session's first picture, and pixels are taken back before they become floor positions
        du, dv = getattr(self, "px_shift", (0.0, 0.0))
        return self.cam.to_floor(u - du, v - dv, self.z)

    def _area_lims(self):
        """(xlo, xhi, ylo, yhi) floor cm a robot's centre can be in: the board plus margin_cm at open edges, and
        only up to WALL_CM past a wall's face (set_area)."""
        x0, y0, x1, y1 = self.board_rect
        m = self.margin
        lims = [min(x0, x1) - m, max(x0, x1) + m, min(y0, y1) - m, max(y0, y1) + m]
        for axis, v in getattr(self, "walls", None) or []:
            i = 0 if axis == "x" else 2
            if abs(v - lims[i]) < abs(v - lims[i + 1]):
                lims[i] = max(lims[i], v - self.WALL_CM)
            else:
                lims[i + 1] = min(lims[i + 1], v + self.WALL_CM)
        return lims

    def set_area(self, board_rect, walls=()):
        """The board (x0, y0, x1, y1) cm and its walls [("x", x of the face) | ("y", y of the face)] (2026-10-22:
        the live runner's tracker had no board area at all, so it followed Nick's hands 47 cm off the board; and a
        robot pressed against a board joined the board's edge in one outline). Pixels beyond a wall's face, or far
        off the board, are left out before outlines are made."""
        with self.lock:
            self.board_rect = tuple(board_rect)
            self.walls = list(walls)
            self.cut = None

    def _cut_mask(self, shape):
        if self.board_rect is None:
            return None
        if getattr(self, "cut", None) is None or self.cut.shape != shape:
            xlo, xhi, ylo, yhi = self._area_lims()
            # the outline of a robot's top must fit: half a body more than its centre's limits
            hb = 0.5 * ROBOT_CM
            pts = []
            n = 12
            for k in range(n):                               # sample the edges (the lens bends straight lines)
                pts.append((xlo - hb + (xhi - xlo + 2 * hb) * k / n, ylo - hb))
            for k in range(n):
                pts.append((xhi + hb, ylo - hb + (yhi - ylo + 2 * hb) * k / n))
            for k in range(n):
                pts.append((xhi + hb - (xhi - xlo + 2 * hb) * k / n, yhi + hb))
            for k in range(n):
                pts.append((xlo - hb, yhi + hb - (yhi - ylo + 2 * hb) * k / n))
            px = [self._px(x, y) for x, y in pts]
            m = np.zeros(shape, np.uint8)
            if all(np.isfinite(u) and np.isfinite(v) for u, v in px):
                cv2.fillPoly(m, [np.array([[int(round(u)), int(round(v))] for u, v in px], np.int32)], 255)
            else:
                m[:] = 255
            self.cut = m
        return self.cut

    def on_area(self, x, y):
        if self.board_rect is None:
            return True
        xlo, xhi, ylo, yhi = self._area_lims()
        return xlo <= x <= xhi and ylo <= y <= yhi

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
            # the followed robot: its body, its halo and the error of its position (2026-10-22: with 1.0 robot
            # width, a position a few cm off let a standing robot's edge be learned as floor, bit by bit; its
            # outline then broke into pieces and it was lost)
            # (only while it is held still: moving, its pixels never stand still long enough to be learned, and
            # the floor it uncovers must be learned at once; real clip hand_near_center)
            held = self.hint is not None and self.hint[3] == float("inf")
            m |= self._disk_mask(shape, [(self.pos[1], self.pos[2])], self.COVER_SCALE if held else self.COVER_MOVING)
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
        cut = self._cut_mask(mask.shape) if getattr(self, "walls", None) else None
        if cut is not None:
            mask &= cut
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
                            rw=float(rw), rh=float(rh),
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
            if self.hold_ref is not None and self.active is not None and self.pos is not None:
                self._check_held(g, motion)
            if self.active is not None and self.pos is not None:
                cands = self.candidates(g, motion)
                self.last_cands = cands
                _, px, py = self.pos
                dt_ = max(0.0, t - self.pos[0])
                # 2026-10-22 session: lost for a moment during a turn, the gate grew to 50 cm and the tracker took
                # Right, 34 cm away, for Left (then "the camera saw it move 0 cm"). The gate now grows for at
                # most GATE_GROW_S, and another robot is anything within OTHER_CM of where it was last placed.
                gate_cm = ROBOT_CM * 0.9 + self.vmax * min(dt_, self.GATE_GROW_S)
                hint = self.hint if (self.hint and t <= self.hint[3]) else None
                best = None
                det_moving = False
                others = [q for r, q in self.protect.items() if r != self.active]
                ok = [c for c in cands if c["shape"]
                      and not any(math.hypot(c["x"] - q[0], c["y"] - q[1]) < self.OTHER_CM for q in others)]
                merged = self._merge(ok)
                if hint and hint[3] != float("inf") and any(c["moving"] >= self.MOVING_MIN for c in merged):
                    # it is being driven: something that moves beats something that stands still (2026-10-22
                    # evening: a ghost of Left left in the floor memory where it had stood, at distance 0, kept
                    # winning over the real Left driving away, 10 cm off: "moved 0 cm", "lost")
                    merged = [c for c in merged if c["moving"] >= self.MOVING_MIN]
                for c in merged:
                    d = math.hypot(c["x"] - px, c["y"] - py)
                    if d > gate_cm:
                        continue
                    if hint and math.hypot(c["x"] - hint[0], c["y"] - hint[1]) > hint[2]:
                        continue
                    score = d - 3.0 * c["moving"]
                    if best is None or score < best[0]:
                        best = (score, c["x"], c["y"])
                        det_moving = c["moving"] >= self.MOVING_MIN
                held_now = self.hint is not None and self.hint[3] == float("inf")
                det = (best[1], best[2]) if best is not None else None
                if self.use_lock:
                    lk_out = self._lock_step(img, t, det, held_now, hint, det_moving)
                    if lk_out is not None:
                        best = (0.0, lk_out[0], lk_out[1])
                if best is not None:
                    self.last_real = t
                if best is None and self.hold_ref is not None and not self.disturbed and \
                        self.hint is not None and self.hint[3] == float("inf") and \
                        t - self.last_real <= self.HOLD_TRUST_S:
                    # (only for HOLD_TRUST_S after its outline was last really seen: a hold set on a wrong spot
                    # must go stale so the runner finds the robot again; review 2026-10-22)
                    # held still and the picture where it stands has not changed since it stopped: it is still
                    # there, even when its outline against the floor memory is not robot-shaped just now (by a
                    # board's edge, or with a ghost of itself in the floor: 2026-10-22 footage 10:40:11-16)
                    best = (0.0, self.pos[1], self.pos[2], "held")
                elif best is not None and held_now:
                    # held still: report the mean of its recent real positions (two outline centres alternating: a
                    # median would still jump between them) (2026-10-22 evening, live: a held
                    # robot's position flipped 2-4 cm frame to frame between the held spot and its outline's
                    # centre, so the runner's still check, which wants 1 cm, never saw it stand still)
                    self.held_hist.append((best[1], best[2]))
                    best = (best[0], float(np.mean([q[0] for q in self.held_hist])),
                            float(np.mean([q[1] for q in self.held_hist])))
                if best is not None:
                    if not self.use_lock or self.source not in ("lock", "both"):
                        self.source = "held" if len(best) > 3 else "detector"
                    self.stats[self.source] += 1
                    self.pos = (t, best[1], best[2])
                    self.protect[self.active] = (best[1], best[2])
                    self.last_seen = t
                    self.misses = 0
                    out = self.pos
                else:
                    self.misses += 1
                    self.source = None
                    self.stats["none"] += 1
            if not self.frozen:
                self._learn(g, dt_frame)
            return out

    def _lock_step(self, img, t, det, held_now, hint, det_moving=False):
        """The lock-on tracker's picture (called under self.lock from push). det: the detector's own outline of the
        followed robot this picture, or None. Returns the lock's (x, y), or None (then the detector's is used).
        Sets self.source to "lock" when the lock's position is used."""
        import e02_lock as LK
        self.source = None
        if self.lk is None:
            self.lk = LK.LockTracker(self.cam, self.rpx, self.z)
        if self.lock_start is not None:
            if self.lock_start_t is None:
                self.lock_start_t = t
            x, y = self.lock_start
            if det is not None and math.hypot(det[0] - x, det[1] - y) < self.START_NEAR_CM:
                x, y = det                       # where the detector sees its outline, not just the place handed
            elif t - self.lock_start_t < self.START_WAIT_S:
                return None                      # (2026-10-21 footage: handed 7 cm off, the lock took the floor)
            if self.bg is not None:
                self.lk.set_floor(np.clip(self.bg, 0, 255).astype(np.uint8))
            self.lock_start, self.lock_start_t = None, None
            self.lock_off = (0.0, 0.0)
            if not self.lk.start(img, x, y, t):
                self.lk.stop()
            self.check_t = t
        self.det_hist.append(det)
        if not self.lk.locked:
            # lost: the detector carries on; once it sees the robot stand still, lock on to it there again. Still:
            # the runner says so (a hold), or the detector's own outline has not moved for RELOCK_STILL_N pictures
            # (2026-10-23 replay, a find run (no hold during the robot's own runs): after the lock let go it never
            # came back, and the robot went unseen whenever it stood still)
            recent = list(self.det_hist)[-self.RELOCK_STILL_N:]
            still = len(recent) == self.RELOCK_STILL_N and all(q is not None for q in recent) and \
                max(math.hypot(q[0] - recent[-1][0], q[1] - recent[-1][1]) for q in recent) <= self.RELOCK_STILL_CM
            if det is not None and (held_now or still):
                self.lock_start = det
                self.stats["relocks"] += 1
            return None
        if self.lock_retake:
            self.lock_retake = False
            self.lk.retake(img, t)
        driving = hint is not None and not held_now
        turning = driving and self.leg_kind == "turn"
        pred = (self.lk.pos[1], self.lk.pos[2]) if turning else None
        s = self.lk.step(img, t, predict=pred, still=held_now, in_leg=driving,
                         pin_cm=self.PIN_CM if turning else None)
        # the detector checks the lock every CHECK_S and when the robot has just stopped
        if (t - self.check_t >= self.CHECK_S or self.check_hold) and self.lk.pos is not None:
            self.check_t, self.check_hold = t, False
            self.stats["checks"] += 1
            if det is None:
                self.last_check = (t, None)
            else:
                d = math.hypot(det[0] - self.lk.pos[1], det[1] - self.lk.pos[2])
                self.last_check = (t, round(d, 1))
                if d <= self.AGREE_CM:
                    self.stats["agree"] += 1
                    self.disagree_n = 0
                else:
                    self.stats["disagree"] += 1
                    self.disagree_n += 1
                    if self.disagree_n >= self.DISAGREE_N and held_now:
                        # the detector keeps seeing it elsewhere while it stands still: lock on there
                        self.disagree_n = 0
                        self.lock_start = det
                        self.stats["relocks"] += 1
                        return det
            if self.bg is not None:
                self.lk.set_floor(np.clip(self.bg, 0, 255).astype(np.uint8))
        if s is None:
            return None
        if det is not None:
            gap = math.hypot(det[0] - s[1], det[1] - s[2])
            if gap <= self.AGREE_CM:
                # both see it: the detector's outline centre is the body's centre; the lock's centre is wherever its
                # look was centred, a few cm off (its lights' glare): learn that offset, report the detector's
                a = 0.1
                self.lock_off = ((1 - a) * self.lock_off[0] + a * (det[0] - s[1]),
                                 (1 - a) * self.lock_off[1] + a * (det[1] - s[2]))
                self.source = "both"
                return None
            if self.lk.score < self.LOCK_SURE or det_moving:
                # an unsure lock falling behind a robot the detector sees clearly (2026-10-21 footage, Center
                # driven with no leg told: the lock lagged 8 cm): the detector's place, and the lock searches there
                self.lk.nudge(det[0] - self.lock_off[0], det[1] - self.lock_off[1], t)
                return None
        # the detector does not see it (or sees something standing still elsewhere while the lock is sure): the
        # lock's place, if it is on the board's area (inside the walls)
        x, y = s[1] + self.lock_off[0], s[2] + self.lock_off[1]
        if not self.on_area(x, y):
            return None
        self.source = "lock"
        return (x, y)

    def confidence(self):
        """Witness summary for the runner: where the position comes from, the lock's match score, and the
        detector's last check (seconds ago, distance cm or None = it did not see it)."""
        with self.lock:
            lc = self.last_check
            return dict(source=self.source, score=round(self.lk.score, 2) if self.lk is not None else None,
                        locked=bool(self.lk is not None and self.lk.locked),
                        check_age_s=None if lc is None or self.frame_t is None else round(self.frame_t - lc[0], 1),
                        check_cm=None if lc is None else lc[1], stats=dict(self.stats))

    def _merge(self, cands):
        """Pieces of one robot (its body and the glare of its lights, or two halves split by a shadow line) as
        one: robot-shaped outlines nearer each other than MERGE_CM become their area-weighted centre. 2026-10-22:
        the followed robot's position flipped 1.5-4 cm between two such pieces, frame after frame."""
        out = []
        for c in sorted(cands, key=lambda c: -c["area"]):
            for o in out:
                if math.hypot(c["x"] - o["x"], c["y"] - o["y"]) < self.MERGE_CM:
                    a = o["area"] + c["area"]
                    o["x"] = (o["x"] * o["area"] + c["x"] * c["area"]) / a
                    o["y"] = (o["y"] * o["area"] + c["y"] * c["area"]) / a
                    o["moving"] = max(o["moving"], c["moving"])
                    o["area"] = a
                    break
            else:
                out.append(dict(c))
        return out

    def _check_held(self, g, motion):
        """A robot told to stand still: is the picture where it stands still the same as when it stopped? If it
        stays different (and nothing is moving there any more) for STABLE_FRAMES, it was picked up or pushed: it is
        lost (the runner finds it again by a turn or its lights) instead of the tracker holding on to the empty
        spot (2026-10-22, real footage 22:44:43: Nick lifted Center). The picture's overall brightness shift is
        taken out first."""
        m, ref, off0 = self.hold_ref
        diff = float(np.mean(np.abs((g[m] - self.offset) - (ref - off0))))
        if diff > 12.0 and float(motion[m].mean()) < 0.08:
            self.disturbed = (self.disturbed or 0) + 1
        else:
            self.disturbed = 0
        if self.disturbed >= self.STABLE_FRAMES:
            self.moved_by_hand += 1
            self.hand_moved.add(self.active)
            self.protect.pop(self.active, None)
            self.pos, self.hint, self.hold_ref = None, None, None
            self.disturbed = 0

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

    def robot_at(self, x, y, scale=1.2):
        """A robot stands at (x, y) on what the floor memory thinks is floor (it was there when the picture was
        taken, or put down since): its floor becomes unknown until it leaves. scale: radius in robot widths (larger
        when (x, y) came from a turn on its spot, a few cm uncertain: else its edge stays in the floor as a ghost)."""
        with self.lock:
            if self.bg is not None:
                self._unknown_at(x, y, scale)

    def refresh_spot(self, x, y, scale=1.0):
        """Copy the current frame into the floor around a spot (nothing stands there now)."""
        with self.lock:
            if self.frame is None or self.bg is None:
                return
            m = self._disk_mask(self.bg.shape, [(x, y)], 0.85 * scale) > 0
            self.bg[m] = self.frame[m] - self.offset
            self.known[m] = 1

    def expect(self, x, y, radius_cm, for_s=10.0, kind=None):
        """The runner expects the followed robot within radius_cm of (x, y) for the next for_s seconds (kind: the
        leg, "turn" or "go")."""
        with self.lock:
            self.hint = (x, y, radius_cm, time.time() + for_s)
            self.leg_kind = kind

    def clear_expect(self):
        with self.lock:
            self.hint = None
            self.hold_ref = None

    def hold(self, radius_cm=6.0):
        """The followed robot has been told to stand still: only accept it within radius_cm of where it is now
        (a hand reaching in next to it, or a robot bumped into it, can't pull the tracker away). Only when the
        camera saw it just now: a hold round an old position could never find it again."""
        with self.lock:
            self.held_hist.clear()
            if self.leg_kind == "turn" and self.lk is not None and self.lk.locked:
                self.lock_retake = True          # a turn on its spot ended: its look is taken again where it is
            self.leg_kind = None
            self.check_hold = True
            if self.pos is not None and self.frame_t is not None and self.frame_t - self.last_seen < 0.5:
                self.hint = (self.pos[1], self.pos[2], radius_cm, float("inf"))
                m = self._disk_mask(self.frame.shape, [(self.pos[1], self.pos[2])], 1.1) > 0
                self.hold_ref = (m, self.frame[m].copy(), self.offset)
                self.disturbed = 0
            else:
                self.hint = None
                self.hold_ref = None

    def clear_protect(self):
        with self.lock:
            self.protect.clear()

    def follow(self, robot, x, y, t=None):
        with self.lock:
            if self.lk is not None:
                self.lk.stop()
            self.lock_start = (x, y)            # the detector (or the runner's find) hands it over: lock on
            self.lock_start_t = None
            self.lock_retake, self.leg_kind, self.disagree_n = False, None, 0
            self.active = robot
            self.pos = (time.time() if t is None else t, x, y)
            self.last_real = self.pos[0]
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
            self.lock_start = None
            if self.lk is not None:
                self.lk.stop()

    def forget(self, robot):
        with self.lock:
            self.protect.pop(robot, None)
            if self.active == robot:
                self.active, self.pos, self.hint = None, None, None
                self.lock_start = None
                if self.lk is not None:
                    self.lk.stop()

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

    def last_t(self):
        """The newest sample's time, or None."""
        with self.lock:
            return self.q[-1][0] if self.q else None

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
class QualityMap:
    """How well the camera sees a robot, place by place, built while the robots drive (2026-10-24 analysis of the
    morning footage: the detail the camera sees on a robot predicts whether the lock-on tracker holds it, r = 0.73
    across 20 cm cells; near the top board the bright floor flattens the robot and the lock-on held 8-36% of the time
    against 90%+ near the open edges). Per cell: the robot's detail (brightness spread within DETAIL_R_PX of its
    centre), the floor's brightness round it, and the tracker's source (both / lock / detector). poor(x, y): a cell
    with enough sightings whose lock-on share is under POOR_LOCK or detail under POOR_DETAIL (both chosen from that
    analysis; DOE: KC with and without the poor-cell weighting)."""

    CELL_CM = 20.0
    DETAIL_R_PX = 9
    RING_PX = (13, 20)
    MIN_N = 15
    POOR_LOCK = 0.4
    POOR_DETAIL = 25.0
    PERIOD_S = 0.5

    def __init__(self):
        self.cells = {}
        self.t_last = 0.0
        yy, xx = np.mgrid[-24:25, -24:25]
        rr = np.hypot(xx, yy)
        self.inner = rr <= self.DETAIL_R_PX
        self.ring = (rr >= self.RING_PX[0]) & (rr <= self.RING_PX[1])

    def add(self, img, u, v, x, y, source, t):
        if t - self.t_last < self.PERIOD_S or img is None:
            return None
        self.t_last = t
        u, v = int(round(u)), int(round(v))
        h, w = img.shape[:2]
        if not (24 <= u < w - 24 and 24 <= v < h - 24):
            return None
        g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        patch = g[v - 24:v + 25, u - 24:u + 25].astype(np.float32)
        detail, floor = float(patch[self.inner].std()), float(patch[self.ring].mean())
        k = (int(math.floor(x / self.CELL_CM)), int(math.floor(y / self.CELL_CM)))
        c = self.cells.setdefault(k, dict(n=0, detail=0.0, floor=0.0, lock=0))
        c["n"] += 1
        c["detail"] += detail
        c["floor"] += floor
        c["lock"] += 1 if source in ("both", "lock") else 0
        return detail

    def cell(self, x, y):
        return self.cells.get((int(math.floor(x / self.CELL_CM)), int(math.floor(y / self.CELL_CM))))

    def poor(self, x, y):
        c = self.cell(x, y)
        if not c or c["n"] < self.MIN_N:
            return False
        return c["lock"] / c["n"] < self.POOR_LOCK or c["detail"] / c["n"] < self.POOR_DETAIL

    def rows(self):
        out = []
        for (i, j), c in sorted(self.cells.items()):
            n = c["n"]
            out.append(dict(x0_cm=i * self.CELL_CM, y0_cm=j * self.CELL_CM, n=n, detail=round(c["detail"] / n, 1),
                            floor=round(c["floor"] / n, 1), lock_share=round(c["lock"] / n, 3),
                            poor=int(n >= self.MIN_N and (c["lock"] / n < self.POOR_LOCK or
                                                         c["detail"] / n < self.POOR_DETAIL))))
        return out


class CameraShiftWatch:
    """Has the camera moved since the session started? (2026-10-24: the pictures' background, the walls and the
    shelf round the plywood, crept up to 2 px during sessions, about 1 cm on the board near the top board; the
    top board's line walked 1 cm though the board never moved.) Every PERIOD_S a picture is aligned to the
    session's first one (OpenCV ECC, a shift only, at half size), using only what lies outside the plywood (robots
    never go there). The shift goes to the tracker (px_shift), so positions are taken back to the first picture's
    camera; on_shift(dict) is told each measurement. Chosen: PERIOD_S, WARN_PX (DOE: the shift's effect on the
    board-line checks, with and without the correction)."""

    PERIOD_S = 5.0
    WARN_PX = 1.0
    MARGIN_PX = 12

    def __init__(self, cam, board_rect, tracker, on_shift=None):
        self.cam, self.rect, self.tracker, self.on_shift = cam, board_rect, tracker, on_shift
        self.ref = None
        self.mask = None
        self.t_last = None
        self.shift = (0.0, 0.0)
        self.cc = None
        self.n = 0
        self.max_px = 0.0

    def _make_mask(self, shape):
        x0, y0, x1, y1 = self.rect
        pts = [self.cam.project((x, y, 0.0)) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
        if not all(np.isfinite(p).all() for p in map(np.asarray, pts)):
            return None
        m = np.full(shape, 255, np.uint8)
        poly = np.array([[int(round(u)), int(round(v))] for u, v in pts], np.int32)
        cv2.fillPoly(m, [poly], 0)
        m = cv2.erode(m, np.ones((2 * self.MARGIN_PX + 1, 2 * self.MARGIN_PX + 1), np.uint8))
        return m[::2, ::2].copy()

    def __call__(self, img, t):
        if self.t_last is not None and t - self.t_last < self.PERIOD_S:
            return
        self.t_last = t
        g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        small = cv2.GaussianBlur(g[::2, ::2].astype(np.float32), (5, 5), 0)
        if self.ref is None:
            self.mask = self._make_mask(g.shape)
            if self.mask is None or self.mask.sum() < 255 * 2000:
                return
            self.ref = small
            return
        W = np.eye(2, 3, dtype=np.float32)
        try:
            cc, W = cv2.findTransformECC(self.ref, small, W, cv2.MOTION_TRANSLATION,
                                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 60, 1e-4), self.mask, 3)
        except cv2.error:
            return
        if cc < 0.8:
            return                                   # the background itself changed (a person, a light): not a shift
        self.shift = (2.0 * float(W[0, 2]), 2.0 * float(W[1, 2]))
        self.cc = float(cc)
        self.n += 1
        mag = math.hypot(*self.shift)
        self.max_px = max(self.max_px, mag)
        if self.tracker is not None:
            self.tracker.px_shift = self.shift
        if self.on_shift:
            self.on_shift(dict(t=round(t, 1), du=round(self.shift[0], 2), dv=round(self.shift[1], 2), cc=round(cc, 3),
                               warn=mag > self.WARN_PX))


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
        self.newest_only = source is None   # live: always the newest frame (a replay or test source: every one)
        self.pending = None
        self.reader_done = False
        self.skipped = 0             # frames dropped because the tracker was busy (newest_only)
        self.lag = 0.0
        self.on_sample = None        # function((t, x, y)) for every position of the followed robot
        self.on_perf = None          # function(dict) once a second: is the laptop keeping up with the camera?
        self.keep_s = 0.0            # > 0: keep this many seconds of pictures (JPEG) for frame_at (measured frames)
        self.ring = deque()          # (t, jpeg bytes)
        self._perf = dict(t0=None, n=0, push_s=0.0, push_max=0.0, age_max=0.0, age_sum=0.0, skipped0=0)

    def _read(self):
        """Reader thread: takes every frame off the stream as it arrives (so nothing piles up in the network
        buffer) and keeps only the newest for the tracker. 2026-10-22 evening: the tracker on the laptop could not
        keep up with 25 frames a second, so frames queued and the runner was looking at pictures seconds old: a
        robot that had just driven 10 cm "had not moved" (old pictures) or was "lost" (no fresh ones), while the
        live view showed it plainly."""
        try:
            for fr in self.src:
                if self.stopped:
                    break
                self.clock.add(fr.t_arrived, fr.t_pi)
                with self.cond:
                    if self.pending is not None:
                        self.skipped += 1
                    self.pending = fr
                    self.cond.notify_all()
        except Exception as e:
            self.err = e
        finally:
            with self.cond:
                self.reader_done = True
                self.cond.notify_all()

    def _frames(self):
        """The newest frame each time the tracker is ready for one."""
        reader = threading.Thread(target=self._read, daemon=True)
        reader.start()
        while not self.stopped:
            with self.cond:
                while self.pending is None and not self.reader_done and not self.stopped:
                    self.cond.wait(0.5)
                if self.pending is None:
                    if self.reader_done:
                        if self.err:
                            raise self.err
                        return
                    continue
                fr, self.pending = self.pending, None
            self.lag = max(0.0, time.time() - fr.t_arrived)
            yield fr

    def run(self):
        try:
            for fr in (self._frames() if self.newest_only else self.src):
                if self.stopped:
                    break
                if not self.newest_only:
                    self.clock.add(fr.t_arrived, fr.t_pi)
                t = fr.t_pi + self.clock.offset - self.cap
                for h in list(self.hooks):
                    h(fr.img, t)
                a = time.time()
                s = self.tracker.push(fr.img, t) if self.tracker is not None else None
                self._perf_add(a, time.time(), t)
                if s is not None:
                    self.history.add(s)
                    if self.on_sample:
                        self.on_sample(s)
                if self.keep_s > 0:
                    ok_, jpg = cv2.imencode(".jpg", fr.img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ok_:
                        with self.cond:
                            self.ring.append((t, jpg))
                            while self.ring and t - self.ring[0][0] > self.keep_s:
                                self.ring.popleft()
                with self.cond:
                    self.frames += 1
                    self.last_t = t
                    self.last_img = fr.img
                    self.recent.append(fr.img)
                    self.cond.notify_all()
        except Exception as e:      # keep the reason for the console
            self.err = e

    def frame_at(self, t, within=0.3):
        """The kept picture nearest laptop time t (decoded), or None (keep_s off, or nothing within `within` s)."""
        with self.cond:
            best = min(self.ring, key=lambda q: abs(q[0] - t)) if self.ring else None
        if best is None or abs(best[0] - t) > within:
            return None
        return cv2.imdecode(best[1], cv2.IMREAD_UNCHANGED)

    def _perf_add(self, a, b, t):
        """Once a second (to on_perf): pictures handled, pictures dropped, the tracker's time per picture, and each
        picture's age when its position was ready (laptop clock minus the picture's time: capture + WiFi + waiting +
        processing). Nick, 2026-10-23: "make sure the program doesn't fall behind reality"."""
        p = self._perf
        if p["t0"] is None:
            p["t0"], p["skipped0"] = a, self.skipped
        p["n"] += 1
        p["push_s"] += b - a
        p["push_max"] = max(p["push_max"], b - a)
        age = b - t
        p["age_sum"] += age
        p["age_max"] = max(p["age_max"], age)
        if b - p["t0"] >= 1.0:
            dt = b - p["t0"]
            row = dict(t=round(b, 3), fps_handled=round(p["n"] / dt, 1),
                       dropped=self.skipped - p["skipped0"], ms_mean=round(1000 * p["push_s"] / p["n"], 1),
                       ms_max=round(1000 * p["push_max"], 1), age_mean_s=round(p["age_sum"] / p["n"], 3),
                       age_max_s=round(p["age_max"], 3), wait_s=round(self.lag, 3),
                       source=getattr(self.tracker, "source", None) or "")
            self._perf = dict(t0=b, n=0, push_s=0.0, push_max=0.0, age_max=0.0, age_sum=0.0, skipped0=self.skipped)
            if self.on_perf:
                try:
                    self.on_perf(row)
                except Exception:
                    pass

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

    def collect(self, seconds):
        """Every frame for the next `seconds`: (grey pictures, their laptop times)."""
        got = []
        hook = lambda img, t: got.append((img, t))      # noqa: E731
        self.hooks.append(hook)
        try:
            time.sleep(seconds)
        finally:
            self.hooks.remove(hook)
        return [g[0] for g in got], [g[1] for g in got]

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


def locate_flash(cam, frames, times, board_rect=None, margin_cm=8.0, near=None, z_cm=ROBOT_TOP_CM,
                 freqs=(1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3)):
    """Where is a robot flashing its lights (`identify`: both LEDs white, 250 ms on / 250 ms off for 3 s, i.e.
    2 Hz)? For a third try when a turn on the spot showed nothing (2026-10-22, Nick: "does it flash its lights
    to find it?"). Per pixel, how strongly its brightness swings at about 2 Hz; the auto exposure and the
    picture's noise don't swing at that rate, an LED does. Found on the real footage at both help calls where the
    robot was in view (22:38:05 and 22:48:13). frames: grey pictures over >= 2.5 s, times: their times (s).
    Returns dict(x, y, strength, ratio) or None."""
    if len(frames) < 16:
        return None
    ts = np.asarray(times, float)
    ts = ts - ts[0]
    if ts[-1] < 2.0:
        return None
    fr = np.stack([f.astype(np.float32) for f in frames])
    fr -= fr.mean(axis=0)
    best = None
    for f in freqs:
        c = np.cos(2 * np.pi * f * ts)[:, None, None]
        s = np.sin(2 * np.pi * f * ts)[:, None, None]
        p = np.hypot((fr * c).mean(axis=0), (fr * s).mean(axis=0))
        best = p if best is None else np.maximum(best, p)
    ratio = best / (fr.std(axis=0) + 1e-3)
    score = best * (ratio > 0.3)
    h, w = score.shape
    # only on the board area (or near the point given)
    vv, uu = np.mgrid[0:h:4, 0:w:4]
    mask = np.zeros_like(score, dtype=bool)
    xs, ys = cam.to_floor(uu.ravel().astype(float), vv.ravel().astype(float), z_cm)
    ok = np.isfinite(xs) & np.isfinite(ys)
    if board_rect is not None:
        x0, y0, x1, y1 = board_rect
        ok &= (xs >= min(x0, x1) - margin_cm) & (xs <= max(x0, x1) + margin_cm)
        ok &= (ys >= min(y0, y1) - margin_cm) & (ys <= max(y0, y1) + margin_cm)
    if near is not None:
        ok &= np.hypot(xs - near[0], ys - near[1]) <= near[2]
    sub = np.zeros(vv.shape, bool).ravel()
    sub[ok] = True
    mask[0:h:4, 0:w:4] = sub.reshape(vv.shape)
    mask = cv2.dilate(mask.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    sc = np.where(mask, score, 0.0)
    peak = float(sc.max())
    if peak < 18.0:            # real LEDs: 25-31 grey levels of 2 Hz swing; other things on the footage: up to 15
        return None
    vals = sc[mask]
    if peak < 2.5 * float(np.percentile(vals, 99.5)):
        return None                       # many pixels swing alike: not one robot's lights
    v0, u0 = np.unravel_index(int(np.argmax(sc)), sc.shape)
    if ratio[v0, u0] < 0.42:   # real LEDs: over half its swing at 2 Hz; other things: about a third
        return None
    rpx = ROBOT_CM * cam.px_per_cm(*cam.to_floor(float(u0), float(v0), z_cm), z_cm)
    yy, xx = np.nonzero(sc > 0.5 * peak)
    keep = np.hypot(xx - u0, yy - v0) <= 1.3 * rpx
    u, v = float(np.average(xx[keep], weights=sc[yy[keep], xx[keep]])), float(np.average(yy[keep], weights=sc[yy[keep], xx[keep]]))
    x, y = cam.to_floor(u, v, z_cm)
    return dict(x=float(x), y=float(y), strength=round(peak, 1), ratio=round(float(ratio[v0, u0]), 2), u=u, v=v)


def locate_light(cam, before, during, board_rect=None, margin_cm=8.0, near=None, z_cm=ROBOT_TOP_CM, stat="median"):
    """Where did a robot's lights come on (firmware 2026-10-22a `led 1 1 1 <ms>`: both LEDs steady white)? Works at
    any frame rate, even the ~2 pictures a second the laptop gets (2026-10-22: the 2 Hz `identify` flash can't be
    seen at that rate). Small bright spots that got brighter between the two sets of pictures (medians; stat="max"
    for footage of a blinking light), grouped within a robot's size; the strongest group, if it clearly beats the
    next. Returns dict(x, y, strength, n_spots) or None."""
    b = np.median(np.stack([f.astype(np.float32) for f in before]), axis=0)
    st = np.stack([f.astype(np.float32) for f in during])
    a = st.max(axis=0) if stat == "max" else np.median(st, axis=0)
    d = a - b
    d -= float(np.median(d[::4, ::4]))
    mad = float(np.median(np.abs(d[::4, ::4])))
    thr = max(30.0, 8.0 * 1.4826 * mad)
    mask = (d > thr).astype(np.uint8)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    cx = cy = 61.0
    if board_rect is not None:
        cx, cy = (board_rect[0] + board_rect[2]) / 2, (board_rect[1] + board_rect[3]) / 2
    rpx = ROBOT_CM * cam.px_per_cm(cx, cy, z_cm)
    spots = []
    for i in range(1, n):
        ar = int(stats[i, cv2.CC_STAT_AREA])
        if ar < 2 or ar > 0.6 * rpx * rpx:
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
        spots.append([float(x), float(y), float(d[lab == i].sum())])
    groups = []
    for x, y, s in sorted(spots, key=lambda q: -q[2]):
        for g in groups:
            if math.hypot(x - g["x"], y - g["y"]) < 1.3 * ROBOT_CM:
                tot = g["strength"] + s
                g["x"] = (g["x"] * g["strength"] + x * s) / tot
                g["y"] = (g["y"] * g["strength"] + y * s) / tot
                g["strength"], g["n_spots"] = tot, g["n_spots"] + 1
                break
        else:
            groups.append(dict(x=x, y=y, strength=s, n_spots=1))
    groups.sort(key=lambda g: -g["strength"])
    if not groups or (len(groups) > 1 and groups[1]["strength"] > 0.4 * groups[0]["strength"]):
        return None
    return groups[0]
