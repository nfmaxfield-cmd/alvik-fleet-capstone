#!/usr/bin/env python3
"""Who is who, which way each robot faces, and where its lights sit on it,
from the calibration runs (and the start nudges) seen by the overhead camera.

    python tools/blink/calib.py --session logs/20261018-101500 \\
        --recording recordings/20261018-101455_E1
    python tools/blink/calib.py --session ... --recording ... \\
        --detections recordings/.../detections.csv      # robot centres: gives the LED offset

Inputs:
  --session     console log folder (runinfo, raw, sync, clock_sync, param)
  --recording   camrec.py folder (video.avi, frames.csv); floor_calibration.json
                is taken from there unless --calib is given
  --blink       blink_detections.csv (default <recording>/blink_detections.csv;
                made with blink.py if missing)
  --detections  optional robot-centre detections: frame,t_wall,x_px,y_px[,score]
                (detector output, the simulator's camera_detections.csv, ...)
  --video-offset  video clock minus laptop clock, s (default: from the sync flashes)

For every calib run (the forward roll: RAW step 9 from firmware 2026-10-18c, which
looks around first and rolls up to calib_roll_cm where it finds room, the length in its
CALIB_LOOK record; step 2 before, 15 cm in 2026-10-18b, 3 cm earlier) and every start
nudge (RAW step -1 = roll forward 2 cm and back) it reports:
  - who: the robot named by the log (the robot whose RAW shows the roll at
    that moment) and the robot named by the blink code of the light that
    moved at that moment, and whether they agree
  - heading: the direction of the forward roll, measured on the video by
    comparing the picture around the robot just before the roll with the
    picture during the pause at its far end (phase correlation: the shift
    that best lines the two up; the floor does not move, the robot does)
  - LED offset: where the light (midpoint of the two LEDs) sits on the robot,
    forward and left of the robot's centre, in px and cm (needs --detections)
and for a calib run of firmware 2026-10-18b or later (steps: docs/data-formats.md; STEPS_* below):
  - heading from the forward AND back roll, each corrected for the roll's
    curve (half the heading change the gyro saw during it), combined
  - wheel distance scale: camera distance / wheel odometry distance
  - gyro scale: the camera's rotation over each spin (the robot's picture in
    the stills before and after, aligned by rotation) / the gyro's
  - light offset from the turning centre: a circle through the light's
    flashes during the spins, and where on it the light sits vs the heading
Each with a quality column (ok / check: why).

Output: calibration_results.csv (columns in README.md), plus a printed summary.
"""
import argparse
import csv
import math
import os
import statistics as st
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
sys.path.insert(0, TOOLS)
sys.path.insert(0, os.path.join(TOOLS, "fusion"))
sys.path.insert(0, HERE)
import check_session as cs  # noqa: E402
import session_checks as sc  # noqa: E402
import runs as fr  # noqa: E402
import blink  # noqa: E402

OUT_COLS = ["run", "robot_log", "experiment", "event", "t_video", "robot_blink", "blink_code", "blink_score",
            "blink_p_log", "identity", "others_moving", "lights_moved", "onset_diff_ms", "heading_img_deg", "heading_floor_deg",
            "heading_method", "moved_px", "moved_cm", "commanded_cm", "motion_quality", "det_heading_floor_deg",
            "light_x_px", "light_y_px", "centre_x_px", "centre_y_px", "led_fwd_px", "led_left_px", "led_fwd_cm",
            "led_left_cm",
            # calib runs from firmware 2026-10-18b on (long roll back and forth, spins); blank otherwise
            "heading_chord_deg", "heading_gyro_corr_deg", "heading_back_deg", "heading_spread_deg", "heading_quality",
            "moved_back_cm", "back_quality", "wheel_cm", "wheel_back_cm", "wheel_scale", "wheel_scale_spread",
            "wheel_scale_quality", "gyro_bias_dps", "spin1_gyro_deg", "spin1_cam_deg", "spin2_gyro_deg", "spin2_cam_deg",
            "spin_match", "gyro_scale", "gyro_scale_spread", "gyro_scale_trace", "gyro_scale_quality", "spin_centre_x_px", "spin_centre_y_px",
            "led_points", "led_radius_cm", "led_fit_rms_cm", "led_angle_sd_deg", "spin_led_fwd_cm", "spin_led_left_cm",
            "led_offset_quality",
            # calib runs from firmware 2026-10-18c on: what the look-around chose (CALIB_LOOK)
            "look_result", "look_heading_deg", "look_room_cm", "look_spin", "look_roll", "look_reason",
            # calib runs: done / limited (no room) / failed (session_checks.calib_outcome; blank for nudges)
            "outcome", "note"]

SKIPPED_NO_ROOM = "skipped: no room"     # quality of a value the robot was not allowed to measure (not a check)

# RAW step numbers of a calib run, by firmware (docs/data-formats.md "calibration run")
STEPS_LOOK = dict(blink=1, wiggle=2, pre_spin=3, spin1=4, mid=5, spin2=6, post=7, pre_roll=8, fwd=9, back=10)  # 2026-10-18c+
STEPS_18B = dict(blink=1, fwd=2, back=3, wiggle=4, pre_spin=5, spin1=6, mid=7, spin2=8, post=9)  # 2026-10-18b
STEPS_OLD = dict(blink=1, fwd=2, back=3, wiggle=4, post=5)                                        # before


def fnum(x, default=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


# ------------------------------------------------------------------ video access
class Frames:
    """Frames of video.avi by index, with their t_wall (video clock)."""

    def __init__(self, recording, video=None):
        import cv2
        self.cv2 = cv2
        self.t = blink.read_frame_times(recording)
        self.cap = cv2.VideoCapture(video or os.path.join(recording, "video.avi"))
        self.pos = 0
        self.cache = {}

    def get(self, i):
        if i in self.cache:
            return self.cache[i]
        if i != self.pos:
            self.cap.set(self.cv2.CAP_PROP_POS_FRAMES, i)
            self.pos = i
        ok, img = self.cap.read()
        self.pos += 1
        if not ok:
            return None
        g = self.cv2.cvtColor(img, self.cv2.COLOR_BGR2GRAY).astype(np.float32)
        w = np.min(img, axis=2).astype(np.float32)
        if len(self.cache) > 200:
            self.cache.clear()
        self.cache[i] = (g, w)
        return self.cache[i]

    def indices(self, t0, t1):
        i0 = int(np.searchsorted(self.t, t0, side="left"))
        i1 = int(np.searchsorted(self.t, t1, side="right"))
        return [i for i in range(max(0, i0), min(len(self.t), i1))]

    def median(self, t0, t1, kind=0, max_n=9):
        idx = self.indices(t0, t1)
        if not idx:
            return None, 0
        if len(idx) > max_n:
            idx = [idx[int(round(k))] for k in np.linspace(0, len(idx) - 1, max_n)]
        ims = [self.get(i) for i in idx]
        ims = [m[kind] for m in ims if m is not None]
        if not ims:
            return None, 0
        return np.median(np.stack(ims), axis=0).astype(np.float32), len(ims)


def crop(img, cx, cy, half):
    H, W = img.shape
    x0, y0 = int(round(cx)) - half, int(round(cy)) - half
    x1, y1 = x0 + 2 * half, y0 + 2 * half
    if x0 < 0 or y0 < 0 or x1 > W or y1 > H:
        # shift the window inside the image (the robot may sit near an edge)
        dx = -min(0, x0) - max(0, x1 - W)
        dy = -min(0, y0) - max(0, y1 - H)
        x0, x1, y0, y1 = x0 + dx, x1 + dx, y0 + dy, y1 + dy
        if x0 < 0 or y0 < 0 or x1 > W or y1 > H:
            return None, 0, 0
    return img[y0:y1, x0:x1], x0, y0


def phase_shift(A, B, rmin, rmax):
    """Shift d (px) with B(x) ~ A(x - d), searched for |d| in [rmin, rmax]
    (so the still floor, at d = 0, is left out). Returns (dx, dy, quality),
    quality = peak height in noise widths of the correlation surface."""
    import cv2
    n = A.shape[0]
    win = cv2.createHanningWindow((A.shape[1], A.shape[0]), cv2.CV_32F)

    def prep(M):
        M = M - cv2.GaussianBlur(M, (0, 0), 5)
        return M * win
    FA = np.fft.fft2(prep(A))
    FB = np.fft.fft2(prep(B))
    R = FB * np.conj(FA)
    R /= np.abs(R) + 1e-9
    r = np.fft.fftshift(np.real(np.fft.ifft2(R)))
    c = n // 2
    yy, xx = np.mgrid[0:r.shape[0], 0:r.shape[1]]
    rr = np.hypot(xx - r.shape[1] // 2, yy - c)
    m = (rr >= rmin) & (rr <= rmax)
    if not m.any():
        return None
    rm = np.where(m, r, -np.inf)
    iy, ix = np.unravel_index(int(np.argmax(rm)), r.shape)
    pk = float(r[iy, ix])

    def sub(a, b, c_):
        d = a - 2 * b + c_
        return 0.0 if d >= 0 else float(np.clip(0.5 * (a - c_) / d, -0.5, 0.5))
    fx = ix + (sub(r[iy, ix - 1], pk, r[iy, ix + 1]) if 0 < ix < r.shape[1] - 1 else 0.0)
    fy = iy + (sub(r[iy - 1, ix], pk, r[iy + 1, ix]) if 0 < iy < r.shape[0] - 1 else 0.0)
    # refine: evaluate the correlation surface itself on a fine grid around the
    # peak (a parabola through three samples leans towards whole pixels)
    hgt, wid = R.shape
    ky = np.fft.fftfreq(hgt)
    kx = np.fft.fftfreq(wid)
    for step in (0.1, 0.02):
        offs = np.arange(-6, 7) * step
        sx = fx - wid // 2 + offs                      # shifts to try (fftshifted coordinates)
        sy = fy - hgt // 2 + offs
        Ex = np.exp(2j * np.pi * np.outer(sx, kx))     # (n, wid)
        Ey = np.exp(2j * np.pi * np.outer(sy, ky))     # (n, hgt)
        val = np.real(Ey @ R @ Ex.T) / (hgt * wid)      # val[i, j] = surface at (sy[i], sx[j])
        i, j = np.unravel_index(int(np.argmax(val)), val.shape)
        fx, fy = sx[j] + wid // 2, sy[i] + hgt // 2
    ring = r[m & (np.hypot(xx - ix, yy - iy) > 3)]
    if ring.size < 8:
        ring = r.ravel()
    med = float(np.median(ring))
    mad = 1.4826 * float(np.median(np.abs(ring - med))) + 1e-9
    return fx - r.shape[1] // 2, fy - c, (pk - med) / mad


def measure_shift(ref, paus, x, y, half, e, half1=None):
    """How far the robot near (x, y) moved between the still picture and the
    pause picture: (dx, dy, quality) or None. e = expected distance, px.
    First pass: phase correlation of the same window (half-size half1, big
    enough to hold the robot at both ends of a long roll) in both pictures.
    Second pass: windows moved by half the shift each way, so the robot sits in
    the same place in both windows (a window that cuts the robot differently in
    the two pictures pulls the answer by up to ~0.7 deg); the remaining small
    shift is measured near zero."""
    half1 = max(half, half1 or half)
    A, ax, ay = crop(ref, x, y, half1)
    B, bx, by = crop(paus, x, y, half1)
    if A is None or B is None or (ax, ay) != (bx, by):
        return None
    res = phase_shift(A, B, max(2.0, 0.4 * e), 1.8 * e + 2)
    if res is None:
        return None
    dx, dy, q = res
    sa = (int(round(-dx / 2)), int(round(-dy / 2)))
    sb = (int(round(dx)) + sa[0], int(round(dy)) + sa[1])
    A2, ax2, ay2 = crop(ref, x + sa[0], y + sa[1], half)
    B2, bx2, by2 = crop(paus, x + sb[0], y + sb[1], half)
    if A2 is None or B2 is None or (bx2 - ax2, by2 - ay2) != (sb[0] - sa[0], sb[1] - sa[1]):
        return dx, dy, q
    r2 = phase_shift(A2, B2, 0.0, 2.0)
    if r2 is None:
        return dx, dy, q
    return sb[0] - sa[0] + r2[0], sb[1] - sa[1] + r2[1], q


def flash_spot(frames, t_on, t_off, pair_max_px):
    """Where a sync flash lit up: the midpoint of the brightest pair of spots
    that brightened from just before the flash to during it. Returns
    (x, y, strength) or None."""
    import cv2
    lit, n1 = frames.median(t_on + 0.06, t_off - 0.06, kind=1, max_n=5)
    # dark: the last frames before the flash (the 150 ms dark lead-in; if a late
    # or dropped frame left none there, the frames just before it)
    before = frames.indices(t_on - 0.45, t_on - 0.02)[-3:]
    dark = None
    if before:
        dark, n0 = frames.median(frames.t[before[0]] - 1e-6, frames.t[before[-1]] + 1e-6, kind=1, max_n=3)
    if lit is None or dark is None:
        return None
    d = cv2.GaussianBlur(lit - dark, (0, 0), 1.0)
    pk = []
    dd = d.copy()
    for _ in range(6):
        iy, ix = np.unravel_index(int(np.argmax(dd)), dd.shape)
        v = float(dd[iy, ix])
        if v < 30:
            break
        pk.append((float(ix), float(iy), v))
        cv2.circle(dd, (ix, iy), 5, -1e9, -1)
    if not pk:
        return None
    x0, y0, v0 = pk[0]
    mates = [q for q in pk[1:] if math.hypot(q[0] - x0, q[1] - y0) <= pair_max_px and q[2] >= 0.4 * v0]
    if mates:
        x1, y1, v1 = mates[0]
        return (x0 + x1) / 2, (y0 + y1) / 2, v0
    return x0, y0, v0


# ------------------------------------------------------------------ geometry
class Geo:
    def __init__(self, calib, led_height_cm):
        self.H, self.Hinv = fr.load_homography(calib)
        self.par = fr.load_parallax(calib, led_height_cm)

    def floor(self, x, y):
        return fr.floor_from_top(*fr.apply_h(self.Hinv, x, y), self.par)

    def px_per_cm(self, x, y):
        fx, fy = fr.apply_h(self.Hinv, x, y)
        u1, v1 = fr.apply_h(self.H, fx + 1, fy)
        u2, v2 = fr.apply_h(self.H, fx, fy + 1)
        return 0.5 * (math.hypot(u1 - x, v1 - y) + math.hypot(u2 - x, v2 - y))


# ------------------------------------------------------------------ log side
def motion_event(run, step):
    """From RAW: (t_step0, t_onset, t_pause0, t_pause1, t_step1, peak_deg) on the laptop
    clock for the forward roll of `step`, or None. Uses the wheel angles: the roll
    starts when their mean leaves its resting value, the pause is where it
    stays at its forward extreme."""
    rows = [r for r in run.raw if r.get("step") not in (None, "") and int(fnum(r["step"], 99)) == step]
    if not rows:
        return None
    t0, t1 = rows[0]["t"], rows[-1]["t"]
    span = [r for r in run.raw if t0 - 0.5 <= r["t"] <= t1]
    m = np.array([(fnum(r.get("wl_deg"), 0) + fnum(r.get("wr_deg"), 0)) / 2 for r in span])
    ts = np.array([r["t"] for r in span])
    if not len(m) or np.all(np.isnan(m)):
        return None
    before = m[ts < t0 + 0.02]
    base = float(np.median(before[-10:])) if len(before) else float(m[0])
    rel = m - base
    inside = ts >= t0 - 0.02
    if not inside.any():
        return None
    k = int(np.argmax(np.where(inside, rel, -np.inf)))
    peak = float(rel[k])
    if peak < 5:
        return dict(t0=t0, t1=t1, onset=None, p0=None, p1=None, peak=peak)
    on = next((ts[i] for i in range(len(ts)) if inside[i] and rel[i] > 3), None)
    i0 = k
    while i0 > 0 and rel[i0 - 1] >= peak - 2:
        i0 -= 1
    i1 = k
    while i1 < len(rel) - 1 and rel[i1 + 1] >= peak - 2:
        i1 += 1
    return dict(t0=t0, t1=t1, onset=float(on), p0=float(ts[i0]), p1=float(ts[i1]), peak=peak)


def moving_between(run, ta, tb):
    """Did this run's wheels turn between laptop times ta and tb?"""
    vals = [(fnum(r.get("wl_deg"), 0) + fnum(r.get("wr_deg"), 0)) / 2 for r in run.raw if ta <= r["t"] <= tb]
    return len(vals) >= 2 and (max(vals) - min(vals)) > 3


class Series:
    """One run's RAW log as arrays on the laptop clock: wheel angles, own
    odometry, the gyro's yaw unwrapped (raw sign) and the step number."""

    def __init__(self, run):
        rows = [r for r in run.raw if r.get("step") not in (None, "")]
        g = lambda k: np.array([fnum(r.get(k)) for r in rows], float)
        self.t = np.array([r["t"] for r in rows], float)
        self.step = np.array([int(fnum(r["step"], 99)) for r in rows])
        self.wl, self.wr = g("wl_deg"), g("wr_deg")
        self.x, self.y = g("x_cm"), g("y_cm")
        yaw = g("yaw_deg")
        ok = ~np.isnan(yaw)
        self.yaw = np.full(len(yaw), np.nan)
        if ok.any():
            self.yaw[ok] = np.degrees(np.unwrap(np.radians(yaw[ok])))
        self.sign = self._yaw_sign()

    def _yaw_sign(self):
        """+1 / -1 so that sign * yaw grows on a left turn, from the log itself:
        the yaw's changes against the wheels' (right minus left) changes. None
        if the wheels hardly turned against each other."""
        if len(self.t) < 3:
            return None
        dw = np.diff(self.wr - self.wl)
        dy = np.diff(self.yaw)
        m = ~np.isnan(dw) & ~np.isnan(dy) & (np.abs(dw) > 0.3) & (np.diff(self.t) < 0.2)
        if np.abs(dw[m]).sum() < 60:          # under ~60 wheel degrees of turning: too little to tell
            return None
        c = float(np.sum(dw[m] * dy[m]))
        return 1 if c > 0 else -1

    def still(self, step, trim=0.1, min_s=0.15, tol=1.0):
        """(t0, t1): the last stretch of RAW step `step` in which neither wheel moves by more
        than tol degrees (the robot stands still), trim s cut from its start; None if too short."""
        idx = np.nonzero(self.step == step)[0]
        if not len(idx):
            return None
        last = idx[-1]
        i = len(idx) - 1
        while i > 0 and abs(self.wl[idx[i - 1]] - self.wl[last]) <= tol and abs(self.wr[idx[i - 1]] - self.wr[last]) <= tol:
            i -= 1
        t0, t1 = self.t[idx[i]] + trim, self.t[last]
        return (t0, t1) if t1 - t0 >= min_s else None

    def span(self, step):
        idx = np.nonzero(self.step == step)[0]
        return (self.t[idx[0]], self.t[idx[-1]]) if len(idx) else None

    def at(self, arr, t0, t1):
        m = (self.t >= t0) & (self.t <= t1) & ~np.isnan(arr)
        return float(np.median(arr[m])) if m.any() else float("nan")

    def heading_change(self, a, b):
        """Gyro heading change (left positive, deg) from still span a to still span b."""
        if self.sign is None or a is None or b is None:
            return float("nan")
        return self.sign * (self.at(self.yaw, *b) - self.at(self.yaw, *a))

    def turned(self, a, b, bias):
        """heading_change() with the gyro's still-standing drift (bias, deg/s) taken out."""
        if a is None or b is None or math.isnan(bias):
            return float("nan")
        return self.heading_change(a, b) - bias * (0.5 * (b[0] + b[1]) - 0.5 * (a[0] + a[1]))

    def odo_dist(self, a, b):
        """Wheel odometry distance (the robot's own x_cm, y_cm) between still spans a and b."""
        if a is None or b is None:
            return float("nan")
        d = math.hypot(self.at(self.x, *b) - self.at(self.x, *a), self.at(self.y, *b) - self.at(self.y, *a))
        if d > 0.1:
            return d
        # no odometry in the log: mean wheel angle with the library's nominal 1.7 cm wheel radius
        w = lambda s: (self.at(self.wl, *s) + self.at(self.wr, *s)) / 2
        return abs(w(b) - w(a)) * math.pi / 180 * 1.7

    def bias(self, spans):
        """Gyro drift while standing still (deg/s, left positive): line fit over each still span
        of at least 0.5 s, combined weighted by length. (value, total seconds used)."""
        num = den = 0.0
        if self.sign is None:
            return float("nan"), 0.0
        for s in spans:
            if s is None or s[1] - s[0] < 0.5:
                continue
            m = (self.t >= s[0]) & (self.t <= s[1]) & ~np.isnan(self.yaw)
            if m.sum() < 10:
                continue
            k = np.polyfit(self.t[m] - self.t[m].mean(), self.yaw[m], 1)[0]
            w = s[1] - s[0]
            num += self.sign * k * w
            den += w
        return (num / den, den) if den else (float("nan"), 0.0)

    def yaw_at(self, t):
        m = ~np.isnan(self.yaw)
        return self.sign * np.interp(t, self.t[m], self.yaw[m]) if self.sign is not None and m.any() else np.nan


def fit_circle(P):
    """Least-squares circle through points P (n x 2): (cx, cy, r, rms residual)."""
    x, y = P[:, 0], P[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x * x + y * y
    (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = math.sqrt(max(c + cx * cx + cy * cy, 0.0))
    for _ in range(10):                      # geometric refinement (Gauss-Newton)
        d = np.hypot(x - cx, y - cy)
        d = np.where(d < 1e-9, 1e-9, d)
        J = np.c_[-(x - cx) / d, -(y - cy) / d, -np.ones(len(x))]
        res = d - r
        step, *_ = np.linalg.lstsq(J, -res, rcond=None)
        cx, cy, r = cx + step[0], cy + step[1], r + step[2]
        if np.abs(step).max() < 1e-7:
            break
    res = np.hypot(x - cx, y - cy) - r
    return float(cx), float(cy), float(abs(r)), float(np.sqrt(np.mean(res * res)))


def angular_span(angles_deg):
    """How much of the circle the angles cover, deg (360 minus the largest gap)."""
    a = np.sort(np.mod(np.asarray(angles_deg, float), 360.0))
    if len(a) < 2:
        return 0.0
    gaps = np.diff(np.r_[a, a[0] + 360.0])
    return float(360.0 - gaps.max())


def robot_mask(A, E, pc, ppc, max_cm=8.0, thr=12.0, grow=0):
    """The robot's outline in picture A (uint8 0/255): where A differs from the empty floor E
    (the same spot with the robot away), inside max_cm of pc, cleaned up. None if too small."""
    import cv2
    d = cv2.GaussianBlur(np.abs(A - E), (0, 0), 1.5)
    m = (d > thr).astype(np.uint8) * 255
    disc = np.zeros(A.shape, np.uint8)
    cv2.circle(disc, (int(round(pc[0])), int(round(pc[1]))), int(round(max_cm * ppc)), 255, -1)
    m = cv2.bitwise_and(m, disc)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    m = np.where(lab == big, 255, 0).astype(np.uint8)
    if grow > 0:      # take in a band of floor around the outline: the edges carry most of the rotation
        m = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * grow + 1, 2 * grow + 1)))
    elif grow < 0:
        m = cv2.erode(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (-2 * grow + 1, -2 * grow + 1)))
    return m if m.sum() / 255 >= (5 * ppc) ** 2 else None


def rotation_between(A_img, B_img, cx, cy, ppc, init_deg, geo, mask_cm=4.6, half_cm=9.0, empty=None, grow_cm=1.0):
    """Rotation of the robot (floor degrees, counter-clockwise positive, within +-180) from
    picture A to picture B, both of it standing still, turning centre near image point
    (cx, cy): ECC alignment (rotation + shift) of the robot, started from init_deg (and
    +-4 deg). With `empty` (a picture of the same spot without the robot) the alignment
    uses the robot's outline grown by grow_cm, so its edges are in (in the simulator:
    0.20 deg rms per spin, against 0.31 for the outline alone and 0.27 for a 4.6 cm disc);
    without, a disc of mask_cm (the inside of the top, which has few features).
    Returns (deg, correlation 0..1, mask kind) or None."""
    import cv2
    half = int(round(half_cm * ppc))
    A, ax, ay = crop(A_img, cx, cy, half)
    B, bx, by = crop(B_img, cx, cy, half)
    if A is None or B is None or (ax, ay) != (bx, by):
        return None
    pc = np.array([cx - ax, cy - ay])
    mask, kind = None, "outline"
    if empty is not None:
        E, ex, ey = crop(empty, cx, cy, half)
        if E is not None and (ex, ey) == (ax, ay):
            mask = robot_mask(A, E, pc, ppc, grow=int(round(grow_cm * ppc)))
    if mask is None:
        kind = "disc"
        mask = np.zeros(A.shape, np.uint8)
        cv2.circle(mask, (int(round(pc[0])), int(round(pc[1]))), int(round(mask_cm * ppc)), 255, -1)
    A = cv2.GaussianBlur(A, (0, 0), 1.0)
    B = cv2.GaussianBlur(B, (0, 0), 1.0)
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 300, 1e-7)
    best = None
    for d0 in (init_deg, init_deg - 4.0, init_deg + 4.0):
        a = math.radians(d0)
        R = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])   # floor CCW = image CW in (x, y-down)
        W = np.zeros((2, 3), np.float32)
        W[:, :2] = R
        W[:, 2] = pc - R @ pc
        try:
            cc, W = cv2.findTransformECC(A, B, W, cv2.MOTION_EUCLIDEAN, crit, mask, 3)
        except cv2.error:
            continue
        if best is None or cc > best[0]:
            best = (float(cc), W.copy())
    if best is None:
        return None
    cc, W = best
    # floor angle of a short line through the centre, before and after (handles the camera's
    # small tilt and the image's flipped y axis)
    p1 = np.array([pc[0] - 15, pc[1]])
    p2 = np.array([pc[0] + 15, pc[1]])
    q1, q2 = W[:, :2] @ p1 + W[:, 2], W[:, :2] @ p2 + W[:, 2]
    f = lambda p: geo.floor(p[0] + ax, p[1] + ay)
    a0 = math.atan2(f(p2)[1] - f(p1)[1], f(p2)[0] - f(p1)[0])
    a1 = math.atan2(f(q2)[1] - f(q1)[1], f(q2)[0] - f(q1)[0])
    return wrap(math.degrees(a1 - a0)), cc, kind


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def circ_mean(deg):
    a = np.radians(np.asarray(deg, float))
    return math.degrees(math.atan2(np.mean(np.sin(a)), np.mean(np.cos(a))))


# ------------------------------------------------------------------ main work
def blink_tracks(rows):
    by = defaultdict(list)
    for r in rows:
        if r.get("lit", 1):
            by[r["track"]].append(r)
    out = {}
    for k, v in by.items():
        v.sort(key=lambda r: r["t"])
        out[k] = dict(track=k, robot=v[0]["robot"], code=v[0]["code"], score=v[0]["score"], probs=v[0].get("probs", {}),
                      t=np.array([r["t"] for r in v]), x=np.array([r["x"] for r in v]), y=np.array([r["y"] for r in v]),
                      n=np.array([r.get("n_spots", 2) for r in v]))
    return out


def track_position(T, t0, t1, around=None):
    """Median lit position in [t0, t1]; else (if around) the 10 rows nearest that time within 2 s."""
    m = (T["t"] >= t0) & (T["t"] <= t1)
    if m.sum() >= 2:
        return float(np.median(T["x"][m])), float(np.median(T["y"][m])), int(m.sum())
    if around is not None:
        d = np.abs(T["t"] - around)
        idx = np.argsort(d)[:10]
        idx = idx[d[idx] <= 2.0]
        if len(idx):
            return float(np.median(T["x"][idx])), float(np.median(T["y"][idx])), 0
    return None


def read_detections(path):
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            cls = r.get("cls") or "alvik"
            if cls not in ("alvik", ""):
                continue
            if "status" in r and r["status"] not in ("confirmed", "corrected", "auto"):
                continue
            out.append((fnum(r["t_wall"]), fnum(r["x_px"]), fnum(r["y_px"])))
    a = np.array(out, float) if out else np.zeros((0, 3))
    return a[np.argsort(a[:, 0])] if len(a) else a


def centre_near(dets, t0, t1, x, y, rad):
    if dets is None or not len(dets):
        return None
    m = (dets[:, 0] >= t0) & (dets[:, 0] <= t1) & (np.hypot(dets[:, 1] - x, dets[:, 2] - y) <= rad)
    if m.sum() < 2:
        return None
    return float(np.median(dets[m, 1])), float(np.median(dets[m, 2])), int(m.sum())


def analyse_event(run, ev, kind, cmd_cm, off, frames, tracks, dets, geo, all_runs, names, S=None, still_ms=None,
                  steps=None, look=None):
    """One forward roll (and, in a calib run from firmware 2026-10-18b on, the roll back and
    the spins): returns an output row (dict). steps: the run's RAW step numbers (STEPS_*);
    look: its CALIB_LOOK record (2026-10-18c+)."""
    row = dict(run=run.run_id, robot_log=run.robot, experiment=run.experiment, event=kind, commanded_cm=cmd_cm)
    steps = steps or STEPS_OLD
    if look:
        row.update(look_result=look.get("result", ""), look_heading_deg=look.get("heading_deg", ""),
                   look_room_cm=look.get("room_cm", ""))
    notes = []
    S = S or Series(run)
    nmed = 15 if (still_ms or 0) >= 600 else 9      # long stills: median of more frames
    if kind == "calib" and look and fnum(look.get("roll_cm"), 1.0) <= 0:
        # 2026-10-18c+: it found no room to roll; the spins can still be measured
        return analyse_spin_only(run, row, S, steps, off, frames, tracks, dets, geo, names, nmed)
    if ev is None or ev.get("onset") is None:
        row["note"] = "no forward roll in the wheel log" if ev else \
            f"no RAW step {steps['fwd'] if kind == 'calib' else -1}"
        return row
    v = lambda t: t + off            # laptop -> video clock
    row["t_video"] = round(v(ev["onset"]), 3)
    # still before the roll
    if kind == "calib" and "pre_roll" in steps and S.still(steps["pre_roll"]) is not None:
        s8 = S.still(steps["pre_roll"])      # 2026-10-18c+: the still after turning to the roll heading
        r0 = (s8[0], min(s8[1], ev["onset"] - 0.08))
    elif kind == "calib":
        s1 = [r["t"] for r in run.raw if fnum(r.get("step"), 99) == 1]
        r0 = (s1[0] + 0.3, ev["onset"] - 0.08) if s1 else (ev["onset"] - 1.0, ev["onset"] - 0.08)
    else:
        sy = run.sync.get("start")
        r0 = (sy[1] + 0.03, ev["onset"] - 0.08) if sy else (ev["onset"] - 0.6, ev["onset"] - 0.08)
        if r0[1] - r0[0] < 0.1:            # hardly any still time after the flash: use the flash itself
            r0 = (sy[0] + 0.05, ev["onset"] - 0.08) if sy else r0
    p0, p1 = ev["p0"] + 0.03, ev["p1"] - 0.03
    if p1 - p0 < 0.04:
        p0, p1 = ev["p0"], ev["p1"]
    # candidate lights near this moment
    cands = []
    for T in tracks.values():
        pos = track_position(T, v(r0[0]), v(r0[1]), around=v(ev["onset"]))
        if pos is None:
            continue
        if abs(T["t"][np.argmin(np.abs(T["t"] - v(ev["onset"])))] - v(ev["onset"])) > 2.5:
            continue
        cands.append(dict(src="blink", T=T, x=pos[0], y=pos[1], n_still=pos[2]))
    pair_max = 8.0 * geo.px_per_cm(320, 240)
    fs = None
    sy = run.sync.get("start")
    if sy:
        fs = flash_spot(frames, v(sy[0]), v(sy[1]), pair_max)
    if fs:
        near = [c for c in cands if math.hypot(c["x"] - fs[0], c["y"] - fs[1]) <= 6 * geo.px_per_cm(fs[0], fs[1])]
        if near:
            for c in near:
                c["flash"] = fs
        else:
            cands.append(dict(src="flash", T=None, x=fs[0], y=fs[1], n_still=0, flash=fs))
    if not cands:
        row["note"] = "no blinking light or start flash found near this moment"
        return row
    ref, nref = frames.median(v(r0[0]), v(r0[1]), kind=0, max_n=nmed)
    paus, npause = frames.median(v(p0), v(p1), kind=0, max_n=nmed)
    if ref is None or paus is None:
        row["note"] = "video frames missing for the still or the pause"
        return row
    moved = []
    for c in cands:
        ppc = geo.px_per_cm(c["x"], c["y"])
        half = int(round(14 * ppc))
        e = cmd_cm * ppc
        res = measure_shift(ref, paus, c["x"], c["y"], half, e, half1=int(round(max(14.0, 12 + 0.8 * cmd_cm) * ppc)))
        if res is None:
            continue
        dx, dy, q = res
        c.update(dx=dx, dy=dy, q=q, ppc=ppc, mag=math.hypot(dx, dy))
        if q >= 8 and 0.45 * e <= c["mag"] <= 1.7 * e:
            c["onset_v"] = motion_onset(frames, c, ref, v(ev["onset"]), v(p0))
            moved.append(c)
    row["lights_moved"] = len(moved)
    others = [o.robot for o in all_runs if o.robot != run.robot and o.raw and o.raw[0]["t"] <= ev["p1"]
              and o.raw[-1]["t"] >= ev["onset"] - 0.3 and moving_between(o, ev["onset"] - 0.3, ev["p1"])]
    row["others_moving"] = ";".join(sorted(set(others)))
    if not moved:
        row["note"] = "no light near this moment moved by the commanded distance"
        return row
    for c in moved:
        c["dt"] = (c["onset_v"] - v(ev["onset"])) if c.get("onset_v") is not None else 9.9
    moved.sort(key=lambda c: abs(c["dt"]))
    c = moved[0]
    if len(moved) > 1:
        notes.append(f"{len(moved)} lights moved; picked the one whose start matches the wheel log best "
                     f"({1000 * c['dt']:+.0f} ms vs {1000 * moved[1]['dt']:+.0f} ms)")
        if abs(abs(moved[1]["dt"]) - abs(c["dt"])) < 0.05:
            notes.append("AMBIGUOUS: two lights started moving within 50 ms of each other")
    if c.get("onset_v") is not None:
        row["onset_diff_ms"] = round(1000 * c["dt"])
    # who
    if c["src"] == "blink":
        T = c["T"]
        row.update(robot_blink=T["robot"], blink_code=T["code"], blink_score=round(T["score"], 3))
        expect = run.robot if run.robot in names.values() else names.get(4, "other")
        if expect in T["probs"]:
            row["blink_p_log"] = round(T["probs"][expect], 3)
        if not T["robot"]:
            row["identity"] = "code not readable"
        else:
            row["identity"] = "agree" if T["robot"] == expect else "DISAGREE"
    else:
        row["identity"] = "motion only (no blink code seen)"
    if others:
        row["identity"] += " (other robots moving too)"
    # heading
    dx, dy = c["dx"], c["dy"]
    hi = math.degrees(math.atan2(-dy, dx))
    lx, ly = c["x"], c["y"]
    f0 = geo.floor(lx, ly)
    f1 = geo.floor(lx + dx, ly + dy)
    hf = math.degrees(math.atan2(f1[1] - f0[1], f1[0] - f0[0]))
    moved_cm = math.hypot(f1[0] - f0[0], f1[1] - f0[1])
    row.update(heading_img_deg=round(hi, 2), heading_chord_deg=round(hf, 2), heading_method="video shift",
               moved_px=round(c["mag"], 2), moved_cm=round(moved_cm, 2), motion_quality=round(c["q"], 1))
    # The roll's direction is the robot's MEAN heading while rolling: a slightly curved roll
    # (left and right wheels a little different) turns it by d_fwd, so the heading it stood
    # at is the chord minus half of that turn, which the gyro measures.
    still0, pause = (r0[0], r0[1]), (ev["p0"], ev["p1"])
    # gyro drift while standing: from the stills of a calib run (the 3 s blink still, and from
    # 2026-10-18b the stills after the rolls and around the spins); not for start nudges
    bias = float("nan")
    if kind == "calib":
        bias, b_s = S.bias([still0, pause] + [S.still(steps[k]) for k in ("back", "pre_spin", "mid", "post")
                                               if k in steps])
        if b_s < 1.5:
            bias = float("nan")
    d_fwd = S.turned(still0, pause, bias)
    h_fwd = wrap(hf - d_fwd / 2) if not math.isnan(d_fwd) else hf
    if not math.isnan(d_fwd):
        row["heading_gyro_corr_deg"] = round(-d_fwd / 2, 3)
        row["heading_method"] = "video shift + gyro"
    est = [h_fwd]
    odo_f = S.odo_dist(still0, pause)
    if odo_f > 0.3:
        row["wheel_cm"] = round(odo_f, 3)
    # the roll back (calib runs, firmware 2026-10-18b+: step 3 ends with a still of calib_still_ms)
    back = S.still(steps["back"]) if kind == "calib" and still_ms else None
    if back is not None:
        after, n_after = frames.median(v(back[0]), v(back[1]), kind=0, max_n=nmed)
        res = None
        if after is not None:
            res = measure_shift(paus, after, c["x"] + dx, c["y"] + dy, int(round(14 * c["ppc"])), cmd_cm * c["ppc"],
                                half1=int(round(max(14.0, 12 + 0.8 * cmd_cm) * c["ppc"])))
        if res is not None and res[2] >= 8 and (res[0] * dx + res[1] * dy) < 0:
            bx, by, bq = res
            g0, g1 = geo.floor(lx + dx, ly + dy), geo.floor(lx + dx + bx, ly + dy + by)
            hb = math.degrees(math.atan2(g1[1] - g0[1], g1[0] - g0[0]))
            back_cm = math.hypot(g1[0] - g0[0], g1[1] - g0[1])
            d_back = S.turned(pause, back, bias)
            if not math.isnan(d_back) and not math.isnan(d_fwd):
                h_back = wrap(hb - 180.0 - d_back / 2 - d_fwd)
            else:
                h_back = wrap(hb - 180.0)
            row.update(heading_back_deg=round(h_back, 2), moved_back_cm=round(back_cm, 2),
                       back_quality=round(bq, 1))
            est.append(h_back)
            odo_b = S.odo_dist(pause, back)
            if odo_b > 0.3:
                row["wheel_back_cm"] = round(odo_b, 3)
            if odo_f > 0.3 and odo_b > 0.3:
                ws_f, ws_b = moved_cm / odo_f, back_cm / odo_b
                row["wheel_scale"] = round((moved_cm + back_cm) / (odo_f + odo_b), 4)
                row["wheel_scale_spread"] = round(ws_f - ws_b, 4)
                okw = abs(ws_f - ws_b) <= 0.01 and 0.9 <= row["wheel_scale"] <= 1.1
                row["wheel_scale_quality"] = "ok" if okw else (
                    "check: forward and back differ by %.1f%%" % (100 * abs(ws_f - ws_b)) if abs(ws_f - ws_b) > 0.01
                    else "check: far from 1")
        else:
            notes.append("roll back not measured on the video")
    h0 = circ_mean(est)
    row["heading_floor_deg"] = round(h0, 2)
    if len(est) == 2:
        sp = wrap(est[0] - est[1])
        row["heading_spread_deg"] = round(sp, 2)
        row["heading_quality"] = "ok" if abs(sp) <= 1.0 and c["q"] >= 8 else "check: forward and back rolls differ by %.1f deg" % sp
    elif kind == "calib" and still_ms:
        row["heading_quality"] = "check: forward roll only"
    else:
        row["heading_quality"] = "single roll" if cmd_cm < 10 else "ok"
    # light position while still: blink lights during the still, else the start flash
    L0 = None
    if c["src"] == "blink" and c["n_still"] >= 2:
        L0 = (c["x"], c["y"])
    elif c.get("flash"):
        L0 = c["flash"][:2]
        notes.append("light position from the start flash")
    elif c["src"] == "blink":
        L0 = (c["x"], c["y"])
        notes.append("light position from pulses after the roll (robot may have moved since)")
    if L0:
        row.update(light_x_px=round(L0[0], 2), light_y_px=round(L0[1], 2))
    # robot centre (detections): heading cross-check and LED offset
    if dets is not None and L0:
        rad = 10 * c["ppc"]
        C0 = centre_near(dets, v(r0[0]), v(r0[1]), L0[0], L0[1], rad)
        if C0 is None and kind == "nudge" and sy:
            C0 = centre_near(dets, v(sy[0]) - 0.5, v(r0[1]), L0[0], L0[1], rad)
        C1 = centre_near(dets, v(p0), v(p1), L0[0] + dx, L0[1] + dy, rad)
        if C0:
            row.update(centre_x_px=round(C0[0], 2), centre_y_px=round(C0[1], 2))
            if C1:
                g0, g1 = geo.floor(*C0[:2]), geo.floor(*C1[:2])
                if math.hypot(g1[0] - g0[0], g1[1] - g0[1]) > 0.4 * cmd_cm:
                    row["det_heading_floor_deg"] = round(math.degrees(math.atan2(g1[1] - g0[1], g1[0] - g0[0])), 2)
            a = math.radians(hi)
            ex, ey = L0[0] - C0[0], L0[1] - C0[1]
            row["led_fwd_px"] = round(ex * math.cos(a) - ey * math.sin(a), 2)
            row["led_left_px"] = round(-ex * math.sin(a) - ey * math.cos(a), 2)
            gl, gc = geo.floor(*L0), geo.floor(*C0[:2])
            h = math.radians(hf)
            ex, ey = gl[0] - gc[0], gl[1] - gc[1]
            row["led_fwd_cm"] = round(ex * math.cos(h) + ey * math.sin(h), 2)
            row["led_left_cm"] = round(-ex * math.sin(h) + ey * math.cos(h), 2)
        else:
            notes.append("no robot-centre detections near the light while still")
    if kind == "calib" and still_ms and L0 and "spin1" in steps and S.span(steps["spin1"]) is not None:
        # the pause at the far end of the roll is a picture of the floor where the robot
        # stood (and spins), with the robot 15 cm away: the spin pictures' background
        empty = paus if cmd_cm >= 12 else None
        analyse_spin(run, S, row, c, L0, h0, still0, bias, off, frames, tracks, dets, geo, nmed, notes, empty=empty,
                     steps=steps)
    row["note"] = "; ".join(notes)
    return row


def analyse_spin(run, S, row, c, L0, h0, still0, b, off, frames, tracks, dets, geo, nmed, notes, empty=None,
                 steps=STEPS_18B):
    """The spins of a calib run (2026-10-18b+): still, spin left, still, spin back right, still
    (steps 5-9 in 2026-10-18b, 3-7 from 2026-10-18c). h0 None (no roll): no heading, so the
    light's offset is reported as a radius only.
    Gyro scale: the camera's rotation (whole-robot picture alignment between the stills)
    against the gyro's (bias from the stills taken out). Light offset: the lights circle
    the turning centre; the circle's centre and radius, and where on it the light sits
    relative to the heading, give the light's (forward, left) offset from the turning centre."""
    v = lambda t: t + off
    s5, s7, s9 = S.still(steps["pre_spin"]), S.still(steps["mid"]), S.still(steps["post"])
    sp6, sp8 = S.span(steps["spin1"]), S.span(steps["spin2"])
    if None in (s5, s7, s9, sp6, sp8):
        notes.append("spin steps incomplete in the log")
        return
    if S.sign is None:
        notes.append("gyro direction not found in the log: spin not analysed")
        return
    if math.isnan(b):
        b = 0.0
    row["gyro_bias_dps"] = round(b, 4)
    mid = lambda s: 0.5 * (s[0] + s[1])
    g1 = S.heading_change(s5, s7) - b * (mid(s7) - mid(s5))
    g2 = S.heading_change(s7, s9) - b * (mid(s9) - mid(s7))
    row.update(spin1_gyro_deg=round(g1, 3), spin2_gyro_deg=round(g2, 3))
    # ---- the lights' circle (blink flashes during the spins and the stills around them)
    ppc = c["ppc"]
    rest = None
    pts = []
    for T in tracks.values():
        m = (T["t"] >= v(s5[0])) & (T["t"] <= v(s5[1]))
        # (8 cm: from 2026-10-18c L0 is the light after the turn to the roll heading, which
        # moves it around the turning centre by up to twice its ~3 cm radius)
        if m.any() and math.hypot(np.median(T["x"][m]) - L0[0], np.median(T["y"][m]) - L0[1]) <= 8 * ppc:
            rest = (float(np.median(T["x"][m])), float(np.median(T["y"][m])))
    near = rest or L0
    for T in tracks.values():
        m = (T["t"] >= v(sp6[0])) & (T["t"] <= v(sp8[1])) & (np.hypot(T["x"] - near[0], T["y"] - near[1]) <= 8 * ppc)
        for k in np.nonzero(m)[0]:
            pts.append((T["t"][k], T["x"][k], T["y"][k], T["n"][k]))
    pts.sort()
    centre_px = None
    if len(pts) >= 8:
        P = np.array([[p[1], p[2]] for p in pts])
        F = np.array([geo.floor(p[1], p[2]) for p in pts])
        cxp, cyp, rp, _ = fit_circle(P)
        centre_px = (cxp, cyp)
        tl = np.array([p[0] for p in pts]) - off
        spin_of = np.where(tl <= sp6[1] + 0.3, 1, np.where(tl >= sp8[0] - 0.3, 2, 0))
        fits = []
        for k in (1, 2):
            m = spin_of == k
            if m.sum() >= 6:
                cx, cy, r, rms = fit_circle(F[m])
                ang = np.degrees(np.arctan2(F[m, 1] - cy, F[m, 0] - cx))
                fits.append(dict(k=k, m=m, cx=cx, cy=cy, r=r, rms=rms, span=angular_span(ang)))
        fits = [f for f in fits if f["span"] >= 180]
        if fits:
            row["spin_centre_x_px"], row["spin_centre_y_px"] = round(cxp, 2), round(cyp, 2)
            row["led_points"] = int(sum(f["m"].sum() for f in fits))
            row["led_radius_cm"] = round(float(np.mean([f["r"] for f in fits])), 3)
            row["led_fit_rms_cm"] = round(float(np.sqrt(np.mean([f["rms"] ** 2 for f in fits]))), 3)
            row["_fits"] = (fits, F, tl)
    else:
        notes.append(f"only {len(pts)} light flashes seen during the spins")
    # ---- camera rotation: the robot's picture in the stills before, between and after the spins
    if centre_px is None:
        C0 = centre_near(dets, v(s5[0]), v(s5[1]), near[0], near[1], 10 * ppc) if dets is not None else None
        if C0 is not None:
            centre_px = C0[:2]
            notes.append("spin centre from the robot-centre detections")
        else:
            if row.get("heading_img_deg") in (None, ""):
                notes.append("spin centre not found (no light circle, no roll heading)")
                return
            a = math.radians(row.get("heading_img_deg", 0.0))
            centre_px = (near[0] - 2.75 * ppc * math.cos(a), near[1] + 2.75 * ppc * math.sin(a))
            notes.append("spin centre ASSUMED 2.75 cm behind the light")
    imgs = [frames.median(v(s[0]), v(s[1]), kind=0, max_n=nmed)[0] for s in (s5, s7, s9)]
    cams, ccs = [], []
    for (A, B), g in zip(((imgs[0], imgs[1]), (imgs[1], imgs[2])), (g1, g2)):
        if A is None or B is None:
            cams.append(float("nan"))
            continue
        res = rotation_between(A, B, centre_px[0], centre_px[1], ppc, wrap(g), geo, empty=empty)
        if res is None:
            cams.append(float("nan"))
            continue
        d, cc, mk = res
        if mk != "outline" and "disc" not in notes:
            notes.append("spin pictures aligned inside a disc (no empty-floor picture of the spot): less precise")
        cams.append(d + 360.0 * round((g - d) / 360.0))     # whole turns from the gyro
        ccs.append(cc)
    cam1, cam2 = cams
    if not math.isnan(cam1):
        row["spin1_cam_deg"] = round(cam1, 3)
    if not math.isnan(cam2):
        row["spin2_cam_deg"] = round(cam2, 3)
    if ccs:
        row["spin_match"] = round(min(ccs), 3)
    gs = float("nan")
    if not (math.isnan(cam1) or math.isnan(cam2)) and abs(g1 - g2) > 90:
        gs = (cam1 - cam2) / (g1 - g2)
        s1, s2 = cam1 / g1, cam2 / g2
        row["gyro_scale"] = round(gs, 5)
        row["gyro_scale_spread"] = round(s1 - s2, 5)
        okg = abs(s1 - s2) <= 0.005 and 0.95 <= gs <= 1.05 and min(ccs) >= 0.8
        row["gyro_scale_quality"] = "ok" if okg else "check: " + ", ".join(
            q for q, bad in (("the two spins differ by %.2f%%" % (100 * abs(s1 - s2)), abs(s1 - s2) > 0.005),
                             ("far from 1", not 0.95 <= gs <= 1.05),
                             ("weak picture match %.2f" % min(ccs), min(ccs) < 0.8)) if bad)
    elif not math.isnan(cam1) and abs(g1) > 90:
        gs = cam1 / g1
        row["gyro_scale"] = round(gs, 5)
        row["gyro_scale_quality"] = "check: first spin only"
    # ---- light offset from the turning centre, in the robot's frame
    if "_fits" in row and h0 is None:
        row.pop("_fits")
        row["led_offset_quality"] = SKIPPED_NO_ROOM + " (no roll, so no heading: radius only)"
    if "_fits" in row:
        fits, F, tl = row.pop("_fits")
        k_scale = gs if not math.isnan(gs) else 1.0
        # heading at the still before the spin: the roll's heading plus what the gyro turned since
        h5 = h0 + k_scale * (S.heading_change(still0, s5) - b * (mid(s5) - mid(still0)))
        y5 = S.yaw_at(mid(s5))
        phis = []
        for f in fits:
            idx = np.nonzero(f["m"])[0]
            hd = h5 + k_scale * (S.yaw_at(tl[idx]) - y5 - b * (tl[idx] - mid(s5)))
            ang = np.degrees(np.arctan2(F[idx, 1] - f["cy"], F[idx, 0] - f["cx"]))
            phis.append(wrap(ang - hd))
        # cross-check of the gyro scale: the light's angle around the centre against the
        # gyro's, over each spin (slope of a line fit; a constant clock offset only shifts it)
        slopes = []
        for f in fits:
            idx = np.nonzero(f["m"])[0]
            if len(idx) >= 10:
                th = np.degrees(np.unwrap(np.radians(np.degrees(np.arctan2(F[idx, 1] - f["cy"], F[idx, 0] - f["cx"])))))
                ps = S.yaw_at(tl[idx]) - b * (tl[idx] - tl[idx][0])
                if np.ptp(ps) > 180:
                    slopes.append(float(np.polyfit(ps, th, 1)[0]))
        if slopes:
            gt = float(np.mean(slopes))
            row["gyro_scale_trace"] = round(gt, 5)
            if not math.isnan(gs) and abs(gt - gs) > 0.005 and row.get("gyro_scale_quality") == "ok":
                row["gyro_scale_quality"] = "check: the light's trace says %.4f" % gt
        allp = np.concatenate(phis)
        phi = circ_mean(allp)
        r = row["led_radius_cm"]
        row["led_angle_sd_deg"] = round(float(np.sqrt(np.mean(np.array([wrap(p - phi) for p in allp]) ** 2))), 2)
        row["spin_led_fwd_cm"] = round(r * math.cos(math.radians(phi)), 3)
        row["spin_led_left_cm"] = round(r * math.sin(math.radians(phi)), 3)
        okl = row["led_fit_rms_cm"] <= 0.3 and row["led_points"] >= 12 and 0.5 <= r <= 8
        row["led_offset_quality"] = "ok" if okl else "check: " + ", ".join(
            q for q, bad in (("circle fit rms %.2f cm" % row["led_fit_rms_cm"], row["led_fit_rms_cm"] > 0.3),
                             ("only %d flashes" % row["led_points"], row["led_points"] < 12),
                             ("radius %.1f cm" % r, not 0.5 <= r <= 8)) if bad)


def look_roll_cm(run, S, look):
    """The forward roll's commanded length in a 2026-10-18c+ calib run: its CALIB_LOOK roll_cm;
    without that record (lost over WiFi), the wheels' own distance over RAW step 9 (the library
    stops the wheels at the commanded distance by their encoders), else calib_roll_cm."""
    v = fnum((look or {}).get("roll_cm"))
    if v == v:
        return v
    d = S.odo_dist(S.still(STEPS_LOOK["pre_roll"]), S.still(STEPS_LOOK["fwd"]))
    if d == d and d > 0.3:
        return round(d, 1)
    return run.params.get("calib_roll_cm", 15.0)


def analyse_spin_only(run, row, S, steps, off, frames, tracks, dets, geo, names, nmed):
    """A 2026-10-18c+ calib run that found no room to roll: who (blink code only) and the spins
    (gyro scale; the light's circle radius, but no offset direction: no roll, no heading)."""
    v = lambda t: t + off
    notes = ["no roll (the robot found no room): no heading or wheel scale"]
    row["heading_quality"] = row["wheel_scale_quality"] = SKIPPED_NO_ROOM     # not measured: blank values
    s3 = S.still(steps["pre_spin"])
    if s3 is None:
        row["note"] = "; ".join(notes + ["no still before the spin in the log"])
        return row
    expect = run.robot if run.robot in names.values() else names.get(4, "other")
    best = None
    for T in tracks.values():
        pos = track_position(T, v(s3[0]), v(s3[1]))
        if pos is None:
            continue
        p = T["probs"].get(expect, 1.0 if T["robot"] == expect else 0.0)
        if best is None or p > best[0]:
            best = (p, T, pos)
    if best is None or best[0] < 0.5:
        row["note"] = "; ".join(notes + ["no blinking light with this robot's code seen before the spin"])
        return row
    p, T, pos = best
    row.update(robot_blink=T["robot"], blink_code=T["code"], blink_score=round(T["score"], 3), blink_p_log=round(p, 3),
               identity="blink code only (no roll)", light_x_px=round(pos[0], 2), light_y_px=round(pos[1], 2))
    c = dict(ppc=geo.px_per_cm(pos[0], pos[1]))
    if "spin1" in steps and S.span(steps["spin1"]) is not None:
        bias, b_s = S.bias([S.still(steps[k]) for k in ("pre_spin", "mid", "post", "pre_roll")])
        if b_s < 1.5:
            bias = float("nan")
        analyse_spin(run, S, row, c, pos[:2], None, s3, bias, off, frames, tracks, dets, geo, nmed, notes,
                     empty=None, steps=steps)
    else:
        notes.append("no spin either")
    row["note"] = "; ".join(notes)
    return row


def mark_outcome(row, run, look):
    """A 2026-10-18c+ calib run's row: its outcome (done / limited (no room) / failed) and, for what
    the robot skipped for lack of room, quality SKIPPED_NO_ROOM with the values left blank (not
    measured): neither ok nor a check, since nothing went wrong."""
    look = look or {}
    row["look_spin"], row["look_roll"], row["look_reason"] = look.get("spin", ""), look.get("roll", ""), look.get("reason", "")
    row["outcome"] = sc.calib_outcome(run.result, look)
    if row["outcome"] != sc.CALIB_LIMITED:
        return row
    if look.get("roll") == "skipped" or ("roll" not in look and fnum(look.get("roll_cm"), 1.0) <= 0):
        for k in ("heading_floor_deg", "heading_img_deg", "wheel_scale"):
            row[k] = ""
        row["heading_quality"] = row["wheel_scale_quality"] = SKIPPED_NO_ROOM
    if look.get("spin") == "skipped":
        for k in ("gyro_scale", "spin_led_fwd_cm", "spin_led_left_cm"):
            row[k] = ""
        row["gyro_scale_quality"] = row["led_offset_quality"] = SKIPPED_NO_ROOM
    return row


def motion_onset(frames, c, ref, t_log_v, t_pause_v):
    """Video time when the picture around this light started to change:
    first frame whose difference from the still picture reaches half of the
    difference during the pause."""
    half = int(round(10 * c["ppc"]))
    A, ax, ay = crop(ref, c["x"], c["y"], half)
    if A is None:
        return None
    idx = frames.indices(t_log_v - 0.5, t_pause_v + 0.02)
    if len(idx) < 3:
        return None
    vals = []
    for i in idx:
        g = frames.get(i)
        if g is None:
            continue
        B, bx, by = crop(g[0], c["x"], c["y"], half)
        if B is None:
            continue
        vals.append((frames.t[i], float(np.mean(np.abs(B - A)))))
    if len(vals) < 3:
        return None
    base = np.median([q[1] for q in vals[:5]])
    top = vals[-1][1]
    if top - base < 2:
        return None
    lvl = base + 0.5 * (top - base)
    for (ta, va), (tb, vb) in zip(vals, vals[1:]):
        if va < lvl <= vb:
            return ta + (tb - ta) * (lvl - va) / max(vb - va, 1e-9)
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True)
    ap.add_argument("--recording", required=True)
    ap.add_argument("--blink")
    ap.add_argument("--detections")
    ap.add_argument("--calib")
    ap.add_argument("--video", help="video file, if not <recording>/video.avi")
    ap.add_argument("--video-offset", type=float, help="video clock minus laptop clock, s")
    ap.add_argument("--led-height-cm", type=float, default=fr.ALVIK_TOP_CM)
    ap.add_argument("--names", help="code to name, e.g. 1=Left,2=Center,3=Right,4=other (default: from the "
                                    "session's PARAM blink_code and robot names, as blink.py --session)")
    ap.add_argument("--out", help="default <recording>/calibration_results.csv")
    args = ap.parse_args(argv)
    names = blink.parse_names(args.names) if args.names else blink.names_from_session(args.session)
    calib = args.calib or os.path.join(args.recording, "floor_calibration.json")
    if not os.path.exists(calib):
        sys.exit(f"{calib} not found: give --calib floor_calibration.json")
    geo = Geo(calib, args.led_height_cm)
    if geo.par is None:
        print("note: floor_calibration.json has no camera height/nadir: light positions not corrected for height")
    runs, fits = fr.load_runs(args.session)
    looks = {r["run"]: r for r in cs.rows(args.session, "calib_look")}     # 2026-10-18c+
    off = args.video_offset
    if off is None:
        vo = fr.video_offset(args.session, args.recording, fits)
        if vo is None:
            sys.exit("could not line up the video with the robot logs from the sync flashes: give --video-offset")
        off, spread, n = vo
        print(f"video clock = laptop clock {1000 * off:+.0f} ms (from {n} flashes, spread {1000 * spread:.0f} ms)")
    bpath = args.blink or os.path.join(args.recording, "blink_detections.csv")
    if not os.path.exists(bpath):
        print(f"{bpath} not found: running blink.py first")
        blink.main([args.recording, "--out", bpath] + (["--names", args.names] if args.names else
                                                       ["--session", args.session])
                   + (["--video", args.video] if args.video else []))
    tracks = blink_tracks(blink.read_blink_csv(bpath))
    dets = read_detections(args.detections) if args.detections else None
    frames = Frames(args.recording, args.video)
    out = []
    for run in sorted(runs, key=lambda r: r.t_start):
        cmd = run.params.get("start_nudge_cm", 2.0)
        cmd = 2.0 if cmd != cmd else cmd
        if run.experiment == "calib":
            S = Series(run)
            roll = run.params.get("calib_roll_cm", float("nan"))
            still = run.params.get("calib_still_ms", 1000.0)
            still = still if still == still else 1000.0
            if "calib_roll_min_cm" in run.params:     # 2026-10-18c+: spins first, then a roll where there is room
                look = looks.get(run.run_id)
                ev = motion_event(run, STEPS_LOOK["fwd"])
                out.append(mark_outcome(analyse_event(run, ev, "calib", look_roll_cm(run, S, look), off, frames, tracks,
                                                      dets, geo, runs, names, S=S, still_ms=still, steps=STEPS_LOOK,
                                                      look=look or {}), run, look))
            elif roll == roll:          # firmware 2026-10-18b: the long roll, stills, spins
                ev = motion_event(run, 2)
                out.append(analyse_event(run, ev, "calib", max(3.0, roll), off, frames, tracks, dets, geo, runs, names,
                                         S=S, still_ms=still, steps=STEPS_18B))
            else:                     # older: a 3 cm roll (max(3, start_nudge_cm)), no spin
                ev = motion_event(run, 2)
                out.append(analyse_event(run, ev, "calib", max(3.0, cmd), off, frames, tracks, dets, geo, runs, names,
                                         S=S))
        if any(fnum(r.get("step"), 99) == -1 for r in run.raw):
            ev = motion_event(run, -1)
            out.append(analyse_event(run, ev, "nudge", cmd, off, frames, tracks, dets, geo, runs, names))
    path = args.out or os.path.join(args.recording, "calibration_results.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=OUT_COLS, extrasaction="ignore")
        w.writeheader()
        for r in out:
            w.writerow(r)
    print_summary(out)
    print(f"wrote {len(out)} rows to {path}")
    return 0


def print_summary(out):
    hdr = ["run", "robot_log", "event", "robot_blink", "identity", "heading_floor_deg", "moved_cm", "led_fwd_cm",
           "led_left_cm", "note"]
    tab = [[str(r.get(k, "")) for k in hdr] for r in out]
    w = [max(len(h), *(len(t[i]) for t in tab)) if tab else len(h) for i, h in enumerate(hdr)]
    w[-1] = len(hdr[-1])
    print("  " + "  ".join(h.ljust(w[i]) for i, h in enumerate(hdr)))
    for t in tab:
        print("  " + "  ".join(c.ljust(w[i]) for i, c in enumerate(t)))
    cal = [r for r in out if r["event"] == "calib"]
    ok = sum(1 for r in cal if str(r.get("identity", "")).startswith("agree"))
    bad = sum(1 for r in cal if "DISAGREE" in str(r.get("identity", "")))
    print(f"calibration runs: {len(cal)}, blink code and wheel log agree on {ok}, disagree on {bad}")
    outc = [r.get("outcome") for r in cal if r.get("outcome")]
    if outc:
        print("calibration outcomes: " + ", ".join(f"{outc.count(k)} {k}" for k in
                                                  (sc.CALIB_DONE, sc.CALIB_LIMITED, sc.CALIB_FAILED))
              + " (limited = it skipped the spin or roll for lack of room: not a failure)")
    offs = [(r["led_fwd_cm"], r["led_left_cm"]) for r in out if r.get("led_fwd_cm") not in (None, "")]
    if offs:
        print(f"light offset from robot centre (median of {len(offs)}): forward "
              f"{st.median(o[0] for o in offs):.2f} cm, left {st.median(o[1] for o in offs):.2f} cm")


if __name__ == "__main__":
    sys.exit(main())
