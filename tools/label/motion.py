#!/usr/bin/env python3
"""Motion-based robot detector for the overhead camera (no learning).

This is two things at once:
  - the labelling program's first guesses, before any model is trained;
  - the project's simple, non-learning comparison detector.

How it works, in plain words:
  1. Background. A picture of the empty floor is made by taking the median of
     frames sampled over about a minute. A robot that stops for a few seconds
     is only in a few of those samples, so the median still shows the floor
     under it and the robot keeps being detected while it is stopped.
  2. Each frame is compared with the background (background subtraction) and
     with the frame two steps earlier (frame differencing). Overall brightness
     is matched first, so lighting flicker does not light up the whole floor.
  3. Changed pixels are grouped into blobs. A blob about one robot big becomes
     an 'alvik' guess; a blob two or three robots big is split; a large blob
     reaching in from the picture edge becomes a 'hand' guess; specks are
     dropped. A "ghost" (the floor showing again where a long-parked robot
     used to be) is recognised because its outline is in the background
     picture, not in the current frame, and is dropped.
  4. Each guess gets the blob centre and a score 0..1. The heading comes from
     the direction the blob moved over the last ~0.3 s. When the robot turns on
     the spot, or seems to drive backwards, the heading is only a guess.

Use frame by frame (for example live):

    state = MotionDetector(fps=30)
    for frame in frames:
        guesses = detect(frame, state)   # list of dict(cls, x, y, heading_deg, score)

Use on a whole recording (better background; the labelling app does this):

    dets, info = detect_recording("recording/video.avi")

It also has the detector interface from docs/data-formats.md:
    load(path) -> model         (path: None, or a JSON file of settings)
    predict(model, frame_bgr)   (frames must be given in time order)

Image conventions: x right, y down, origin top-left. Headings are degrees,
0 = image +x (right), counter-clockwise as seen on screen is positive.

Run it on a recording to see what it finds:
    python tools/label/motion.py <recording folder> [--show]
"""
import argparse
import collections
import copy
import json
import math
import os
import sys

import numpy as np
import cv2


CLASSES = ("alvik", "hand", "unknown", "machine")


class Params:
    """All tunable numbers in one place. Any of them can be passed as a keyword."""

    def __init__(self, **kw):
        # Robot size in pixels = square root of the area it covers seen from above
        # (about the side length of the square robot). None = estimate it from the video.
        self.robot_px = None
        self.default_robot_px = 30.0   # used until (or if) it can be estimated
        self.fps = 30.0
        # Background: median of frames sampled every bg_sample_s seconds over a
        # window of +-bg_half_window_s, rebuilt every bg_update_s seconds.
        self.bg_half_window_s = 30.0
        self.bg_sample_s = 2.0
        self.bg_update_s = 5.0
        self.diff_gap = 2              # frame differencing: compare with the frame this many steps back
        self.min_threshold = 18        # smallest change (0-255) that counts as foreground ...
        self.noise_factor = 4.0        # ... or this many times the measured picture noise, if bigger
        self.min_area_frac = 0.2       # blobs smaller than this fraction of a robot are noise
        # Heading from motion
        self.heading_window_s = 0.3    # direction = where the robot is now vs this long ago
        self.move_speed = 0.5          # robot sizes per second; slower counts as "not moving"
        self.turn_deg = 20.0           # body rotation while stopped above this = turning on the spot
        self.reverse_max_s = 3.0       # "backwards" motion lasting longer than this is taken as forwards
        for k, v in kw.items():
            if not hasattr(self, k):
                raise TypeError("unknown setting: %s" % k)
            setattr(self, k, v)

    def as_dict(self):
        return dict(self.__dict__)


# ----------------------------------------------------------------------------
# Small angle helpers
# ----------------------------------------------------------------------------

def wrap180(a):
    """Angle in degrees wrapped to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def wrap360(a):
    """Angle in degrees wrapped to [0, 360)."""
    return a % 360.0


# ----------------------------------------------------------------------------
# Heading from motion
# ----------------------------------------------------------------------------

class HeadingEstimator:
    """Works out which way one robot faces from how its centre moves.

    Rules, in plain words:
      - Driving: the robot faces the way it is moving.
      - The first time it moves we assume it is driving forwards.
      - If it suddenly moves the opposite way to where it faced, without the
        body having rotated, it is probably reversing: facing is kept, and the
        heading is marked as a guess. If that lasts longer than reverse_max_s
        we decide it had really turned round, and take the motion direction.
      - Stopped: the heading carries over from when it last moved.
      - Stopped but the body outline rotates (robots are square, so the outline
        angle is known modulo 90 degrees): it is turning on the spot. The
        heading is last heading + rotation seen, marked as a guess.

    update() returns (heading_deg or None, status) where status is
    'known', 'guess' or None (no idea yet).
    """

    def __init__(self, robot_px, params=None):
        self.p = params or Params()
        self.robot_px = float(robot_px)
        self.hist = collections.deque()   # (t, x, y, body angle) over the last ~1 s
        self.facing = None                # which way we believe it faces, degrees
        self.known = False                # facing is solid (not a guess)
        self.angle = None                 # body outline angle, unwrapped (keeps counting past 90)
        self.last_angle = None            # last raw outline angle (mod 90)
        self.angle_at_facing = None       # body angle when facing was last set
        self.t_angle = None               # when the outline angle was last seen
        self.moving = False
        self.reverse_since = None         # time reversing started
        self.first_motion = False         # facing came from the very first motion

    def set_facing(self, heading_deg, known=True):
        """Tell it the facing direction (from the user or from a model)."""
        if heading_deg is None:
            return
        self.facing = wrap360(float(heading_deg))
        self.known = bool(known)
        self.angle_at_facing = self.angle
        self.reverse_since = None
        self.first_motion = False

    def clone(self):
        """Independent copy (settings shared)."""
        c = copy.copy(self)
        c.hist = collections.deque(self.hist)
        return c

    def reset_history(self, t, x, y):
        """Forget the recent path (after the user moved the robot's position)."""
        self.hist.clear()
        self.hist.append((t, x, y, self.angle))
        self.moving = False

    def _rotation_since(self, base):
        if self.angle is None or base is None:
            return 0.0
        return self.angle - base

    def update(self, t, x, y, angle90=None, blind=False):
        """One new position. angle90: body outline angle mod 90 if seen.
        blind=True: the robot is visible but its outline is not (touching
        another robot), so a turn on the spot could go unseen."""
        p = self.p
        # 1. Body rotation from the outline angle (only meaningful modulo 90,
        #    so each step is taken as the smallest turn that explains it).
        if angle90 is not None:
            if self.angle is None or t - self.t_angle > 0.2:
                # First sight, or seen again after a gap: we cannot know how far
                # it turned meanwhile, so start counting afresh.
                if self.angle is not None:
                    self.hist = collections.deque((h[0], h[1], h[2], None) for h in self.hist)
                    self.known = False
                self.angle = float(angle90)
                self.angle_at_facing = self.angle
            else:
                self.angle += (angle90 - self.last_angle + 45.0) % 90.0 - 45.0
            self.last_angle = angle90
            self.t_angle = t

        # 2. Position history over the last second.
        self.hist.append((t, x, y, self.angle))
        while self.hist and t - self.hist[0][0] > 1.0:
            self.hist.popleft()

        # 3. Is it moving? Compare with the position heading_window_s ago.
        old = None
        for h in self.hist:
            if t - h[0] <= p.heading_window_s + 1e-6:
                old = h
                break
        speed, direction, mid = 0.0, None, None
        if old is not None and t - old[0] >= 0.6 * p.heading_window_s:
            dx, dy = x - old[1], y - old[2]
            dist = math.hypot(dx, dy)
            speed = dist / (t - old[0])
            if dist >= 2.0:
                direction = math.degrees(math.atan2(-dy, dx))   # minus: image y points down
                t_mid = 0.5 * (old[0] + t)
                mid = min(self.hist, key=lambda h: abs(h[0] - t_mid))
        v_on = p.move_speed * self.robot_px
        if self.moving:
            self.moving = direction is not None and speed > 0.6 * v_on
        else:
            self.moving = direction is not None and speed > v_on

        # 4. Decide the heading.
        if self.moving:
            # The motion direction describes the middle of the window; add the
            # body rotation since then (corrects the lag on curves, and catches
            # a turn on the spot that starts right after driving).
            turned = self._rotation_since(mid[3])
            now = direction + turned
            spinning = abs(turned) > p.turn_deg
            if self.facing is None:
                # First motion: assume forwards.
                self.facing, self.known, self.first_motion = wrap360(now), True, True
                self.angle_at_facing, self.reverse_since = self.angle, None
                return self.facing, ("guess" if spinning else "known")
            expected = self.facing + self._rotation_since(self.angle_at_facing)
            diff = abs(wrap180(now - expected))
            if diff <= 100.0:
                # Driving forwards (straight or on a curve).
                self.facing, self.known = wrap360(now), True
                self.reverse_since = None
                if diff < 45.0:
                    self.first_motion = False
                status = "known"
            elif diff >= 135.0:
                # Moving the opposite way to where it faced: probably reversing.
                if self.reverse_since is None:
                    self.reverse_since = t
                if t - self.reverse_since > p.reverse_max_s:
                    # Too long to be a reverse: it must have turned round unseen.
                    self.facing, self.known = wrap360(now), True
                    self.reverse_since = None
                    status = "known"
                else:
                    self.facing, self.known = wrap360(now + 180.0), False
                    status = "guess"
            else:
                # Sideways-ish motion: can't tell, take the motion direction as a guess.
                self.facing, self.known = wrap360(now), False
                status = "guess"
            self.angle_at_facing = self.angle
            return self.facing, ("guess" if spinning else status)

        # Not moving.
        if self.facing is None:
            return None, None
        rot = self._rotation_since(self.angle_at_facing)
        if abs(rot) > p.turn_deg:
            return wrap360(self.facing + rot), "guess"   # turning on the spot
        if blind:
            return self.facing, "guess"                   # could be turning unseen
        return self.facing, ("known" if self.known else "guess")


# ----------------------------------------------------------------------------
# Picture comparison and blobs
# ----------------------------------------------------------------------------

def _gain(frame, ref):
    """Per-colour brightness ratio frame/ref, measured on a sparse grid of pixels.

    Robots cover a small part of the picture, so the median ratio is the
    lighting change (flicker, auto-exposure)."""
    fs = frame[::16, ::16].reshape(-1, 3)
    rs = ref[::16, ::16].reshape(-1, 3)
    return np.median(fs, axis=0) / np.maximum(np.median(rs, axis=0), 1.0)


def diff_image(frame, ref):
    """Largest per-colour difference between frame and ref (0-255), after
    scaling ref to the frame's overall brightness."""
    g = _gain(frame, ref).astype(np.float32)
    ref_scaled = cv2.transform(ref, np.diag(g))
    d = cv2.absdiff(frame, ref_scaled)
    c = cv2.split(d)
    return cv2.max(cv2.max(c[0], c[1]), c[2])


def _threshold(diff, p):
    """Foreground threshold: min_threshold, or noise_factor x the picture noise."""
    sample = diff[::8, ::8]
    sigma = 1.4826 * float(np.median(sample))   # robust noise estimate (most pixels are floor)
    return max(float(p.min_threshold), p.noise_factor * sigma)


_K3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
_K5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def _grad(gray):
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    return cv2.magnitude(gx, gy)


def _looks_like_ghost(frame, bg, mask, ax0, ay0, ax1, ay1):
    """True if the outline of `mask` (a region of the box ax0..ax1, ay0..ay1)
    is clearly stronger in the background picture than in the frame: the
    floor is showing again where something used to be."""
    band = cv2.dilate(mask, _K3, iterations=2) - cv2.erode(mask, _K3, iterations=1)
    sel = band > 0
    if not sel.any():
        return False
    gf = float(_grad(cv2.cvtColor(frame[ay0:ay1, ax0:ax1], cv2.COLOR_BGR2GRAY))[sel].mean())
    gb = float(_grad(cv2.cvtColor(bg[ay0:ay1, ax0:ax1], cv2.COLOR_BGR2GRAY))[sel].mean())
    return gb > 1.3 * gf + 3.0


def find_blobs(frame, bg, prev, robot_px, p):
    """Find robot-like blobs in one frame.

    frame, bg: BGR images (bg = empty floor). prev: an earlier frame or None.
    Returns (detections, raw_blobs). Each detection is a dict with
    cls, x, y, heading_deg (None here), score, plus extras: angle90 (outline
    angle mod 90, on-screen counter-clockwise), area_ratio, moving.
    raw_blobs is a list of (area, solidity, aspect, touches_border) used to
    estimate the robot size.
    """
    h, w = frame.shape[:2]
    robot_area = robot_px * robot_px
    d_bg = diff_image(frame, bg)
    thr = _threshold(d_bg, p)
    fg = (d_bg > thr).astype(np.uint8)
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, _K3)
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, _K5)

    moving = None
    if prev is not None:
        d_prev = diff_image(frame, prev)
        moving = (d_prev > max(thr, _threshold(d_prev, p))).astype(np.uint8)

    n, labels, stats, cents = cv2.connectedComponentsWithStats(fg, connectivity=8)
    dets, raw, fragments = [], [], []
    pad = 4
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < p.min_area_frac * robot_area:
            continue
        x0, y0 = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
        bw, bh = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        ax0, ay0 = max(0, x0 - pad), max(0, y0 - pad)
        ax1, ay1 = min(w, x0 + bw + pad), min(h, y0 + bh + pad)
        mask = (labels[ay0:ay1, ax0:ax1] == i).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            continue
        cnt = max(contours, key=cv2.contourArea)
        hull_area = max(cv2.contourArea(cv2.convexHull(cnt)), 1.0)
        solidity = min(1.0, area / hull_area)
        (rcx, rcy), (rw, rh), rang = cv2.minAreaRect(cnt)
        aspect = max(rw, rh) / max(min(rw, rh), 1.0)
        border = x0 <= 1 or y0 <= 1 or x0 + bw >= w - 1 or y0 + bh >= h - 1
        raw.append((area, solidity, aspect, border))

        # How much of the blob is moving right now (frame differencing)?
        if moving is not None:
            mv = moving[ay0:ay1, ax0:ax1]
            motion_frac = float((mv & mask).sum()) / area
        else:
            motion_frac = 0.0

        # Ghost test for still blobs: a real object has its outline in this
        # frame; a ghost has it only in the background picture.
        if motion_frac < 0.15 and _looks_like_ghost(frame, bg, mask, ax0, ay0, ax1, ay1):
            continue

        cx, cy = float(cents[i][0]), float(cents[i][1])
        contrast = float(d_bg[ay0:ay1, ax0:ax1][mask > 0].mean()) / thr
        evidence = min(1.0, contrast / 2.5)
        ratio = area / robot_area
        # On-screen counter-clockwise outline angle, modulo 90 (image y points down).
        angle90 = (-rang) % 90.0

        if border and (ratio > 1.8 or (aspect > 2.2 and ratio > 1.0)):
            # Big, or long and bigger than a robot, reaching in from the edge: an arm.
            dets.append(dict(cls="hand", x=cx, y=cy, heading_deg=None,
                             score=round(0.35 + 0.25 * evidence, 3),
                             area_ratio=ratio, moving=motion_frac > 0.05))
        elif 0.45 <= ratio <= 1.7 and solidity > 0.7 and (aspect < 1.8 or (border and aspect < 3.0)):
            # One robot (a robot cut off by the picture edge looks long and thin).
            size_fit = math.exp(-(math.log(ratio) / 0.35) ** 2)
            shape = min(1.0, max(0.0, (solidity - 0.7) / 0.25)) * min(1.0, max(0.0, (1.8 - aspect) / 0.5))
            score = 0.2 + 0.8 * size_fit * (0.5 + 0.5 * shape) * (0.6 + 0.4 * evidence)
            if border:
                score *= 0.7
            dets.append(dict(cls="alvik", x=cx, y=cy, heading_deg=None, score=round(score, 3),
                             angle90=angle90 if aspect < 1.35 else None,
                             area_ratio=ratio, moving=motion_frac > 0.05))
        elif 1.7 < ratio <= 3.6 and not border:
            # Probably 2 or 3 robots touching: split the pixels into k groups.
            k = int(min(3, max(2, round(ratio))))
            ys, xs = np.nonzero(mask)
            pts = np.column_stack([xs + ax0, ys + ay0]).astype(np.float32)
            crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.2)
            _, lab, centres = cv2.kmeans(pts, k, None, crit, 3, cv2.KMEANS_PP_CENTERS)
            lab = lab.ravel()
            for j in range(k):
                share = float((lab == j).sum()) / area
                ok = 0.5 * (1.0 / k) < share < 1.6 * (1.0 / k)
                # A robot leaving the spot where it was parked touches its own
                # ghost: drop a part that is still and has its outline only in
                # the background picture.
                part = np.zeros_like(mask)
                part[ys[lab == j], xs[lab == j]] = 1
                if moving is not None:
                    part_motion = float((moving[ay0:ay1, ax0:ax1] & part).sum()) / max(1, part.sum())
                else:
                    part_motion = 0.0
                if part_motion < 0.15 and _looks_like_ghost(frame, bg, part, ax0, ay0, ax1, ay1):
                    continue
                dets.append(dict(cls="alvik", x=float(centres[j][0]), y=float(centres[j][1]),
                                 heading_deg=None, score=0.45 if ok else 0.3, angle90=None,
                                 area_ratio=ratio / k, moving=motion_frac > 0.05, split=True))
        elif ratio < 0.45:
            fragments.append((cx, cy, area, motion_frac))
        else:
            dets.append(dict(cls="unknown", x=cx, y=cy, heading_deg=None,
                             score=round(0.25 + 0.15 * evidence, 3),
                             area_ratio=ratio, moving=motion_frac > 0.05))

    # Pieces of one robot (for example a dark top and a pale base) can come out
    # as separate small blobs: join close pieces whose total is robot-sized.
    used = [False] * len(fragments)
    for a in range(len(fragments)):
        if used[a]:
            continue
        ax_, ay_, aa, am = fragments[a]
        best = None
        for b in range(a + 1, len(fragments)):
            if used[b]:
                continue
            bx_, by_, ba, bm = fragments[b]
            if math.hypot(ax_ - bx_, ay_ - by_) < 0.8 * robot_px and 0.45 <= (aa + ba) / robot_area <= 1.6:
                best = b
                break
        if best is not None:
            bx_, by_, ba, bm = fragments[best]
            used[a] = used[best] = True
            tot = aa + ba
            dets.append(dict(cls="alvik", x=(ax_ * aa + bx_ * ba) / tot, y=(ay_ * aa + by_ * ba) / tot,
                             heading_deg=None, score=0.35, angle90=None,
                             area_ratio=tot / robot_area, moving=max(am, bm) > 0.05))
        elif aa / robot_area >= 0.3:
            # A lone large piece (robot half hidden or half blended in): weak guess.
            dets.append(dict(cls="alvik", x=ax_, y=ay_, heading_deg=None, score=0.25, angle90=None,
                             area_ratio=aa / robot_area, moving=am > 0.05))
    return dets, raw


def estimate_robot_px(raw_blobs, default=None):
    """Robot size from blob measurements: the median size of compact blobs
    (solid, square-ish, not touching the picture edge). Returns None if there
    are too few to trust."""
    areas = [a for (a, sol, asp, border) in raw_blobs if sol > 0.85 and asp < 1.5 and not border]
    if len(areas) < 15:
        return default
    return float(math.sqrt(np.median(areas)))


# ----------------------------------------------------------------------------
# Frame-by-frame detector (keeps its own state)
# ----------------------------------------------------------------------------

class MotionDetector:
    """Stateful motion detector: feed it frames in time order.

    Without a given background it builds one as it goes from the past minute
    (so a robot that is parked from the very start is only found once it
    moves). detect_recording() below gives a better background by also
    looking ahead in the recording.
    """

    def __init__(self, robot_px=None, fps=30.0, **settings):
        self.p = Params(robot_px=robot_px, fps=fps, **settings)
        self.robot_px = float(robot_px) if robot_px else self.p.default_robot_px
        self.size_fixed = bool(robot_px)
        self.frame_no = 0
        self.recent = collections.deque(maxlen=self.p.diff_gap)   # last few frames, for differencing
        self.samples = collections.deque(maxlen=max(3, int(2 * self.p.bg_half_window_s / self.p.bg_sample_s)))
        self.bg = None
        self.raw = []                  # blob measurements for estimating the robot size
        self.memory = []               # short-term memory for headings: dicts x, y, est, last

    def _own_background(self, frame):
        p = self.p
        step = max(1, int(round(p.bg_sample_s * p.fps)))
        if self.frame_no % step == 0:
            self.samples.append(frame.copy())
        update = max(1, int(round(p.bg_update_s * p.fps)))
        if self.bg is None or (self.frame_no % update == 0 and len(self.samples) >= 3):
            self.bg = np.median(np.stack(self.samples), axis=0).astype(np.uint8) if len(self.samples) >= 3 \
                else self.samples[0].copy()
        return self.bg

    def detect(self, frame, background=None, t=None):
        """Guesses for one frame. background: optional empty-floor picture.
        t: time in seconds (default frame number / fps)."""
        bg = background if background is not None else self._own_background(frame)
        prev = self.recent[0] if len(self.recent) == self.recent.maxlen else None
        dets, raw = find_blobs(frame, bg, prev, self.robot_px, self.p)
        self.recent.append(frame)
        if not self.size_fixed:
            self.raw.extend(raw)
            est = estimate_robot_px(self.raw)
            if est is not None and len(self.raw) >= 60:
                self.robot_px, self.size_fixed = est, True
                self.raw = []
        t = self.frame_no / self.p.fps if t is None else t
        self._headings(dets, t)
        self.frame_no += 1
        return dets

    def _headings(self, dets, t):
        """Give each robot guess a heading from its recent motion, using a
        simple nearest-neighbour memory of where robots were."""
        gate = 1.0 * self.robot_px
        free = list(range(len(self.memory)))
        for d in dets:
            if d["cls"] != "alvik":
                continue
            best, best_dist = None, None
            for j in free:
                m = self.memory[j]
                dist = math.hypot(d["x"] - m["x"], d["y"] - m["y"])
                allow = gate + 3.0 * self.robot_px * (t - m["last"])
                if dist <= allow and (best is None or dist < best_dist):
                    best, best_dist = j, dist
            if best is None:
                m = dict(x=d["x"], y=d["y"], last=t, est=HeadingEstimator(self.robot_px, self.p))
                self.memory.append(m)
            else:
                free.remove(best)
                m = self.memory[best]
            m["x"], m["y"], m["last"] = d["x"], d["y"], t
            heading, status = m["est"].update(t, d["x"], d["y"], d.get("angle90"),
                                              blind="angle90" in d and d["angle90"] is None)
            if heading is not None and status == "known":
                d["heading_deg"] = round(heading, 1)
            elif heading is not None:
                d["heading_guess"] = round(heading, 1)   # ambiguous: offered as a guess only
                d["score"] = round(d["score"] * 0.75, 3)
        self.memory = [m for m in self.memory if t - m["last"] < 2.0]


def detect(frame_bgr, state):
    """detect(frame_bgr, state) -> list of dict(cls, x, y, heading_deg or None, score).
    state is a MotionDetector; give frames in time order."""
    return state.detect(frame_bgr)


# Detector interface from docs/data-formats.md ---------------------------------

def load(path=None):
    """load(path) -> model. path may be None/'' (defaults) or a JSON file of settings."""
    settings = {}
    if path:
        with open(path) as f:
            settings = json.load(f)
    return MotionDetector(**settings)


def predict(model, frame_bgr):
    """predict(model, frame_bgr) -> list of dict(cls, x, y, heading_deg or None, score)."""
    out = []
    for d in model.detect(frame_bgr):
        out.append(dict(cls=d["cls"], x=round(d["x"], 1), y=round(d["y"], 1),
                        heading_deg=d.get("heading_deg"), score=d["score"]))
    return out


# ----------------------------------------------------------------------------
# Whole recording (background looks both back and ahead)
# ----------------------------------------------------------------------------

def count_frames(video_path):
    cap = cv2.VideoCapture(video_path)
    n = 0
    while cap.grab():
        n += 1
    cap.release()
    return n


def read_frames_at(video_path, wanted, progress=None):
    """Decode only the wanted frame numbers (in one pass). Returns {frame: image}.
    progress(done, total): optional, frames gone through so far (about every 30)."""
    wanted = set(int(f) for f in wanted)
    out = {}
    cap = cv2.VideoCapture(video_path)
    f, last = 0, max(wanted) if wanted else -1
    while f <= last and cap.grab():
        if f in wanted:
            ok, img = cap.retrieve()
            if ok:
                out[f] = img
        f += 1
        if progress and f % 30 == 0:
            progress(f, last + 1)
    cap.release()
    if progress:
        progress(last + 1, last + 1)
    return out


SIZE_METHOD = 2             # 2: measure_object_sizes (Oct 22); 1: compact blobs only (failed on real footage)
SIZE_STRONG_FRAC = 0.45      # size: a blob's pixels count when they differ by this x its strong (90th pct) change
SIZE_MAX_ASPECT = 1.5        # size: a robot's outline is about square (motion blur stretches it a little)
SIZE_MIN_FILL = 0.25         # size: at least this share of the outline's inside differs (else it is speckle)
SIZE_MIN_PX = 8.0            # size: smaller outlines are specks, not objects
SIZE_MIN_PLACES = 4          # size: needs objects seen at this many different places


def measure_object_sizes(frame, bg, p):
    """Outline sizes of the objects in one frame against an empty-floor picture, for measuring the
    object size. Returns dicts x, y, side (px: the short side of the smallest rectangle around the
    object), size (sqrt of the outline's area), fill, aspect.

    Made for real footage, where find_blobs' area is not the object's size: a grey robot on a pale
    board differs from the floor mostly along its dark edges, so its blob is a ring or a U with the
    middle missing (area too small, solidity too low); and the board around a robot is often a little
    lighter than in the background (a light halo, mostly on one side), which joins the blob and makes
    it bigger. So, per blob (pieces of one broken outline grouped): keep only the pixels that differ
    strongly for THAT blob, and only on the side (darker or lighter than the floor) that carries most of
    the change (a halo is weak, and on the other side), take the outline around all of them (it closes
    the ring), and measure its short side (blur on one side lengthens the long side only)."""
    h, w = frame.shape[:2]
    d_bg = diff_image(frame, bg)
    g = _gain(frame, bg).astype(np.float32)
    darker = (cv2.transform(bg, np.diag(g)).astype(np.float32) - frame.astype(np.float32)).mean(axis=2) > 0
    thr = _threshold(d_bg, p)
    fg = (d_bg > thr).astype(np.uint8)
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, _K3)
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, _K5)
    # pieces of one object's broken outline are measured together (the grouping margin itself is not
    # measured: only pixels that differ count)
    group = cv2.dilate(fg, _K5, iterations=2)
    contours, _ = cv2.findContours(group, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = []
    for c in contours:
        x, y, bw, bh = cv2.boundingRect(c)
        if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
            continue                      # cut off by the picture's edge: not its whole size
        x0, y0, x1, y1 = x, y, x + bw, y + bh
        region = np.zeros((y1 - y0, x1 - x0), np.uint8)
        cv2.drawContours(region, [c - [x0, y0]], -1, 1, -1)
        inside = (region > 0) & (fg[y0:y1, x0:x1] > 0)
        if inside.sum() < 0.5 * SIZE_MIN_PX * SIZE_MIN_PX:
            continue
        d = d_bg[y0:y1, x0:x1]
        strong = max(thr, SIZE_STRONG_FRAC * float(np.percentile(d[inside], 90)))
        m = (d > strong) & inside
        # the outline is the side of the change that carries most of it (darker than the floor for a dark
        # robot; a halo is the other side, lighter)
        dk = darker[y0:y1, x0:x1]
        m &= dk if float(d[m & dk].sum()) >= float(d[m & ~dk].sum()) else ~dk
        m = m.astype(np.uint8)
        n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        keep = np.isin(lab, [k for k in range(1, n) if st[k, cv2.CC_STAT_AREA] >= 6])
        ys, xs = np.nonzero(keep)
        if len(xs) < 20:
            continue
        hull = cv2.convexHull(np.column_stack([xs, ys]).astype(np.int32))
        hull_area = cv2.contourArea(hull) + 0.5 * cv2.arcLength(hull, True) + 1.0   # in whole pixels
        (cx, cy), (rw, rh), _ = cv2.minAreaRect(hull)
        side, long_ = min(rw, rh) + 1.0, max(rw, rh) + 1.0
        out.append(dict(x=float(cx + x0), y=float(cy + y0), side=float(side), size=math.sqrt(hull_area),
                        fill=len(xs) / hull_area, aspect=long_ / side))
    return out


def object_size_from_measurements(meas, min_places=SIZE_MIN_PLACES):
    """The object size (px) from measure_object_sizes() of frames spread over a recording, or None.

    Each place counts once (a robot parked for minutes, or a thing that is always there, is one vote,
    not one per frame), as the median side measured there; the answer is the median over places, after
    leaving out places far from a first median (specks, two robots touching)."""
    ok = [m for m in meas if m["aspect"] < SIZE_MAX_ASPECT and m["fill"] > SIZE_MIN_FILL and m["side"] >= SIZE_MIN_PX]
    places = []
    for m in ok:
        for pl in places:
            if math.hypot(pl["x"] - m["x"], pl["y"] - m["y"]) < 0.5 * max(m["side"], pl["side0"]):
                pl["v"].append(m["side"])
                break
        else:
            places.append(dict(x=m["x"], y=m["y"], side0=m["side"], v=[m["side"]]))
    if len(places) < min_places:
        return None
    sides = [float(np.median(pl["v"])) for pl in places]
    first = float(np.median(sides))
    near = [s for s in sides if 0.65 * first <= s <= 1.5 * first]
    if len(near) < min_places:
        return None
    return float(np.median(near))


def estimate_robot_size(video_path, n_frames=None, params=None, progress=None):
    """Robot size in pixels from ~40 frames spread over the recording,
    compared with an overall background (measure_object_sizes: works on grey, low-contrast real
    footage too). None if too few objects were seen. progress(done, total): optional, frames read so far."""
    p = params or Params()
    n = n_frames or count_frames(video_path)
    bg_idx = np.linspace(0, n - 1, 31).astype(int)
    look_idx = np.linspace(0, n - 1, 41).astype(int)
    imgs = read_frames_at(video_path, set(bg_idx) | set(look_idx), progress=progress)
    stack = [imgs[i] for i in bg_idx if i in imgs]
    if len(stack) < 3:
        return None
    bg_all = np.median(np.stack(stack), axis=0).astype(np.uint8)
    meas, raw = [], []
    for i in look_idx:
        if i in imgs:
            meas.extend(measure_object_sizes(imgs[i], bg_all, p))
            raw.extend(find_blobs(imgs[i], bg_all, None, p.default_robot_px, p)[1])
    est = object_size_from_measurements(meas)
    return est if est is not None else estimate_robot_px(raw)


def detect_recording(video_path, fps=None, robot_px=None, times=None, progress=None, n_frames=None,
                     size_progress=None, **settings):
    """Run the motion detector over a whole recording.

    The background for each frame is the median of frames sampled from about
    30 s before to 30 s after it (the window is shifted at the start and end so
    it is always about a minute long). Two readers go through the video: one
    ahead, collecting background samples, one detecting.

    times: optional list of frame times in seconds (from frames.csv) used for
    headings; default frame/fps. progress(done, total): optional callback (about every
    30 frames). size_progress(done, total): optional, while the object size is measured.
    n_frames: number of frames if known (saves a counting pass).
    Returns (detections by frame: list of lists, info dict).
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError("could not open " + video_path)
    fps = fps or cap.get(cv2.CAP_PROP_FPS) or 30.0
    cap.release()
    n = n_frames or count_frames(video_path)
    if n == 0:
        raise IOError("no frames in " + video_path)
    p = Params(robot_px=robot_px, fps=fps, **settings)
    step = max(1, int(round(p.bg_sample_s * fps)))
    half = int(round(p.bg_half_window_s * fps))
    update = max(1, int(round(p.bg_update_s * fps)))

    if robot_px:
        size_source = "given"
    else:
        est = estimate_robot_size(video_path, n, p, progress=size_progress)
        robot_px = est if est else p.default_robot_px
        size_source = "estimated" if est else "default"
    rest = p.as_dict()
    rest.pop("robot_px")
    rest.pop("fps")
    det = MotionDetector(robot_px=float(robot_px), fps=fps, **rest)

    ahead = cv2.VideoCapture(video_path)
    reader = cv2.VideoCapture(video_path)
    samples = collections.deque()       # (frame number, image)
    next_ahead = 0
    bg, bg_built_at = None, -10 ** 9
    out = []
    for f in range(n):
        # Window of frames for this frame's background, about a minute long.
        lo, hi = f - half, f + half
        if lo < 0:
            lo, hi = 0, min(n - 1, 2 * half)
        if hi > n - 1:
            lo, hi = max(0, n - 1 - 2 * half), n - 1
        while next_ahead <= hi:
            if not ahead.grab():
                next_ahead = hi + 1
                break
            if next_ahead % step == 0:
                ok, img = ahead.retrieve()
                if ok:
                    samples.append((next_ahead, img))
            next_ahead += 1
        while samples and samples[0][0] < lo:
            samples.popleft()
        if bg is None or f - bg_built_at >= update:
            imgs = [img for (_, img) in samples]
            bg = np.median(np.stack(imgs), axis=0).astype(np.uint8) if len(imgs) >= 3 else imgs[0]
            bg_built_at = f
        ok, frame = reader.read()
        if not ok:
            break
        t = times[f] if times is not None and f < len(times) else f / fps
        out.append(det.detect(frame, background=bg, t=t))
        if progress and (f % 30 == 0 or f == n - 1):
            progress(f + 1, n)
    ahead.release()
    reader.release()
    info = dict(n_frames=len(out), fps=fps, robot_px=round(float(robot_px), 2), robot_px_source=size_source,
                settings=p.as_dict())
    return out, info


def main():
    ap = argparse.ArgumentParser(description="Run the motion detector on a recording and print what it finds.")
    ap.add_argument("recording", help="recording folder (with video.avi) or a video file")
    ap.add_argument("--robot-px", type=float, default=None, help="robot size in pixels (default: estimate)")
    ap.add_argument("--show", action="store_true", help="show the video with the guesses drawn (q quits)")
    args = ap.parse_args()
    video = os.path.join(args.recording, "video.avi") if os.path.isdir(args.recording) else args.recording
    dets, info = detect_recording(video, robot_px=args.robot_px,
                                  progress=lambda d, n: print("  %d / %d frames" % (d, n), end="\r"))
    print()
    print("Robot size: %.1f px (%s)" % (info["robot_px"], info["robot_px_source"]))
    counts = collections.Counter(d["cls"] for frame in dets for d in frame)
    print("Frames: %d, guesses: %s" % (len(dets), dict(counts)))
    if args.show:
        cap = cv2.VideoCapture(video)
        for f, frame_dets in enumerate(dets):
            ok, img = cap.read()
            if not ok:
                break
            for d in frame_dets:
                c = (0, 200, 0) if d["cls"] == "alvik" else (0, 140, 255)
                cv2.circle(img, (int(d["x"]), int(d["y"])), 4, c, -1)
                h = d.get("heading_deg")
                if h is not None:
                    e = (int(d["x"] + 20 * math.cos(math.radians(h))), int(d["y"] - 20 * math.sin(math.radians(h))))
                    cv2.arrowedLine(img, (int(d["x"]), int(d["y"])), e, c, 2)
            cv2.imshow("motion", img)
            if cv2.waitKey(int(1000 / info["fps"])) & 0xFF == ord("q"):
                break
        cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
