#!/usr/bin/env python3
"""Find the robots' blinking light codes in an overhead recording and say
which robot each light is.

    python tools/blink/blink.py recordings/20261018-101455_E1
    python tools/blink/blink.py <recording> --out blink_detections.csv --interpolate

Reads <recording>/video.avi and frames.csv (camrec.py format; timing comes
from frames.csv t_wall, never from the frame number). If
<recording>/floor_calibration.json exists it is used only for the pixel
scale (px per cm); --px-per-cm overrides it.

Firmware 2026-10-18a+ code (rate code): after 300 ms dark, one 100 ms
flash every (n + 2) x 100 ms: Left 0.3 s, Center 0.4 s, Right 0.5 s, other
names 0.6 s, unless a robot's PARAM blink_code (1..4) chose its code: give
--session <console log folder> to read that from the runs (or --names). A flash can be stretched to
~215 ms, cut short (the lights go dark before long blocking calls), and its
start can now and then be delayed by up to ~115 ms.

How it works, in four stages:
  1. spots: in every frame, small spots brighter than the brightest point of
     the ring around them (white light: the darkest colour channel is used).
  2. flashes: a spot followed from frame to frame while it stays lit. It
     counts as a flash only if it was clearly darker just before AND just
     after (it switched on, then off) and was lit for at most 250 ms.
     250-420 ms is a sync flash (left out); longer is a steady light. A
     robot's two LEDs switch together, so two segments that start and end
     together close to each other become one "light" (their midpoint).
  3. tracks: lights are chained into one track per robot, using where the
     last flash was (robots move at most --vmax-cms and can't jump), whether
     it showed two LEDs, and, once a track's rate is clear, its beat (a new
     flash comes a whole number of periods after the last one).
  4. decode: the flash-to-flash intervals give, for each code, how likely
     the track is that robot (each interval: one period, or two or three if
     flashes were missed, plus timing noise, now and then shifted by a
     delayed start), and how likely it is no robot code at all. The period
     is the median of the one-period intervals. Every track with 3+ flashes
     is reported with these probabilities; it gets a name only if the best
     code is likely enough (--min-name-p) and its spot isn't shared with
     other flashing (a flickering lamp split into several "tracks").

Output CSV (one row per frame in which a track's light was lit; with
--interpolate also rows between flashes, marked lit=0):
  frame,t_wall,robot,x_px,y_px,code,score,track,lit,n_spots,period_s,shared,
  p_Left,p_Center,p_Right,p_other,p_none
"""
import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
except ImportError:   # scipy is optional: greedy matching instead
    linear_sum_assignment = None

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)

DEFAULT_NAMES = {1: "Left", 2: "Center", 3: "Right", 4: "other"}
BASE_COLS = ["frame", "t_wall", "robot", "x_px", "y_px", "code", "score", "track", "lit", "n_spots", "period_s",
             "shared"]


def out_cols(names):
    return BASE_COLS + [f"p_{names[k]}" for k in sorted(names)] + ["p_none"]


@dataclass
class Params:
    # the code (firmware blink.h, 2026-10-18a): one 100 ms flash every (n + 2) x 100 ms
    slot_s: float = 0.1
    gap_slots: int = 2
    on_s: float = 0.1
    max_code: int = 4
    names: dict = field(default_factory=lambda: dict(DEFAULT_NAMES))
    # spots
    px_per_cm: float = 4.0
    ring_r0: int = 4              # ring around a spot, px (outside an LED's glow)
    ring_r1: int = 6
    t_min: float = 10.0           # a spot must be this much brighter than the brightest point of its ring
    max_spots: int = 300          # per frame (strongest kept)
    # on-segments
    link_gap_s: float = 0.085     # longest gap bridged inside one on-segment (one missed frame)
    vmax_cms: float = 40.0
    amax_cms2: float = 100.0      # fastest speed change a robot makes (cm/s per s; generous: the sim uses 20)
    swing_frac: float = 0.5       # just before/after a segment the spot must be darker by this much of its contrast...
    swing_min: float = 25.0       # ...and by at least this many grey levels
    max_pulse_s: float = 0.25     # a flash is 100 ms, stretched at most to ~215 ms
    max_sync_s: float = 0.42
    pair_max_cm: float = 8.0      # the robot's two LEDs are at most this far apart
    pair_dt_s: float = 0.045
    # tracks
    track_gap_s: float = 2.0      # no flash for this long ends a track
    tol_s: float = 0.045          # a flash start is on its beat within this...
    delay_s: float = 0.115        # ...or delayed by up to this (robot code blocked)
    # rate decoding: flash starts on a beat of the code's period, + timing noise, now and then late
    sigma_s: float = 0.012        # flash start timing noise (each start is known to about +-17 ms), s
    eps_delay: float = 0.15       # share of flash starts that may be delayed
    eps_out: float = 0.02         # share of flash starts that may be anywhere
    prior_none: float = 0.2       # prior that a flashing light is not a robot code at all
    max_interval_s: float = 1.3   # longer flash-to-flash intervals are left out of the decoding
    min_name_p: float = 0.6       # name a track only if its best code has at least this probability...
    min_name_intervals: int = 5   # ...and it has at least this many flash-to-flash intervals...
    min_rhythm: float = 0.75      # ...and at least this share of its flashes start right on its beat
    min_pulses: int = 3           # tracks with fewer flashes are not reported
    max_foreign: float = 0.25     # other flashes on a track's own spot, per flash of the track
    # camera: how long each frame's exposure lasts, s. 0 = not given: analyse() reads exposure_ms from the
    # recording's meta.json (camrec.py writes it when exposure is locked), else EXPOSURE_FALLBACK_S
    exposure_s: float = 0.0

    def period(self, n):
        return (n + self.gap_slots) * self.slot_s

    @property
    def vmax_px(self):
        return self.vmax_cms * self.px_per_cm


EXPOSURE_FALLBACK_S = 0.012   # used when meta.json doesn't give exposure_ms (exposure not locked, older recording)


def exposure_from_meta(recording, fallback=EXPOSURE_FALLBACK_S):
    """(exposure_s, source): exposure_ms from <recording>/meta.json if present and
    positive, else the fallback."""
    try:
        with open(os.path.join(recording, "meta.json")) as f:
            e = float(json.load(f).get("exposure_ms") or 0)
        if e > 0:
            return e / 1000.0, "meta.json"
    except (OSError, ValueError, TypeError, AttributeError):
        pass
    return fallback, "assumed"


def parse_names(s):
    out = dict(DEFAULT_NAMES)
    if s:
        for part in s.split(","):
            k, v = part.split("=")
            out[int(k)] = v.strip()
    return out


def names_from_session(session, log=print):
    """{code: name} from a console log folder: each robot's PARAM blink_code (0 =
    from its name, exactly as firmware/home_lab/blink.h codeFor(): Left 1,
    Center 2, Right 3, any other name 4, case-sensitive). Warns (via log) when
    two robots share a code, a robot's code changes between runs, a name is not
    exactly Left / Center / Right, or blink_code was set."""
    if TOOLS not in sys.path:
        sys.path.insert(0, TOOLS)
    import check_session as cs
    codes, notes = cs.session_blink_codes(session)
    if not codes:
        log(f"warning: no PARAM records with blinking on in {session}: default names "
            + ", ".join(f"{k}={v}" for k, v in DEFAULT_NAMES.items()))
        return dict(DEFAULT_NAMES)
    for n in notes:
        log("warning: " + n)
    names = cs.blink_names(codes)
    log("blink codes from " + session + ": " + ", ".join(
        f"{k}={v}{'' if any(k in c for c in codes.values()) else ' (no robot)'}" for k, v in sorted(names.items())))
    return names


def read_frame_times(folder):
    ts = []
    with open(os.path.join(folder, "frames.csv"), newline="") as f:
        for r in csv.DictReader(f):
            try:
                ts.append(float(r["t_wall"]))
            except (TypeError, ValueError, KeyError):
                ts.append(float("nan"))
    return np.array(ts, float)


def repair_late_frames(t):
    """A frame that reached the Pi late is followed by one that arrives almost
    at once. Frames are never early, so pull such a late frame's time back to
    one frame interval before the next frame. Used for timing only; output
    keeps the original t_wall."""
    t = np.array(t, float)
    ok = np.isfinite(t)
    if ok.sum() < 3:
        return t
    T = float(np.median(np.diff(t[ok])))
    for i in range(len(t) - 2, 0, -1):
        if not (np.isfinite(t[i - 1]) and np.isfinite(t[i]) and np.isfinite(t[i + 1])):
            continue
        if t[i + 1] - t[i] < 0.5 * T:      # (backwards, so a queue of late frames unwinds)
            t[i] = min(t[i], t[i + 1] - T)
    return t


def px_per_cm_from_calib(path):
    """Pixel scale at the image centre from a floor_calibration.json."""
    sys.path.insert(0, os.path.join(TOOLS, "fusion"))
    import runs as fr
    H, Hinv = fr.load_homography(path)
    try:
        with open(path) as f:
            size = json.load(f).get("image_size") or [640, 480]
    except (OSError, ValueError):
        size = [640, 480]
    u, v = size[0] / 2, size[1] / 2
    x0, y0 = fr.apply_h(Hinv, u, v)
    u1, v1 = fr.apply_h(H, x0 + 1, y0)
    u2, v2 = fr.apply_h(H, x0, y0 + 1)
    return 0.5 * (math.hypot(u1 - u, v1 - v) + math.hypot(u2 - u, v2 - v))


# ------------------------------------------------------------------ stage 1: spots
class SpotFinder:
    """A lit LED is a small spot brighter than EVERYTHING in a ring around it
    (the ring's brightest point, not its average: the corner of a large bright
    area, such as the robot's light-grey top, is as bright as the ring and so
    does not count). Works on the darkest colour channel, so only white-ish
    light counts."""

    def __init__(self, p):
        import cv2
        self.cv2 = cv2
        self.p = p
        r1 = p.ring_r1
        yy, xx = np.mgrid[-r1:r1 + 1, -r1:r1 + 1]
        rr = np.hypot(xx, yy)
        self.ring = ((rr >= p.ring_r0) & (rr <= r1)).astype(np.uint8)
        self.kern = np.ones((5, 5), np.uint8)

    def maps(self, img):
        """(core brightness, spot response) maps."""
        cv2 = self.cv2
        if img.ndim == 2:            # grayscale (the live stream): already the darkest channel
            w = img
        else:
            b, g, r = cv2.split(img)
            w = cv2.min(cv2.min(b, g), r)
        core = cv2.blur(w.astype(np.float32), (3, 3))
        ring = cv2.dilate(core, self.ring, borderType=cv2.BORDER_REPLICATE)
        return core, core - ring

    def spots(self, R):
        cv2, p = self.cv2, self.p
        thr = p.t_min
        mx = cv2.dilate(R, self.kern)
        ys, xs = np.nonzero((R >= thr) & (R >= mx))
        if len(xs) > p.max_spots:
            keep = np.argpartition(R[ys, xs], -p.max_spots)[-p.max_spots:]
            ys, xs = ys[keep], xs[keep]
        out = []
        Hh, Ww = R.shape
        for x, y in zip(xs, ys):
            v = float(R[y, x])
            fx, fy = float(x), float(y)
            if 0 < x < Ww - 1:   # parabola through the peak and its neighbours: sub-pixel position
                l, rr = float(R[y, x - 1]), float(R[y, x + 1])
                d = l - 2 * v + rr
                if d < 0:
                    fx += float(np.clip(0.5 * (l - rr) / d, -0.5, 0.5))
            if 0 < y < Hh - 1:
                u, dd = float(R[y - 1, x]), float(R[y + 1, x])
                d = u - 2 * v + dd
                if d < 0:
                    fy += float(np.clip(0.5 * (u - dd) / d, -0.5, 0.5))
            out.append((fx, fy, v))
        return out, thr


def sample(M, x, y, r=1):
    """Brightest value within r px of (x, y), or None outside the image."""
    Hh, Ww = M.shape
    xi, yi = int(round(x)), int(round(y))
    x0, x1 = max(0, xi - r), min(Ww, xi + r + 1)
    y0, y1 = max(0, yi - r), min(Hh, yi + r + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    return float(M[y0:y1, x0:x1].max())


# ------------------------------------------------------------------ stage 2: on-segments
class Segment:
    __slots__ = ("fi", "t", "x", "y", "v", "c", "before", "after", "on", "off", "est", "kind", "id", "early", "u", "w",
                 "done_t")

    def __init__(self, fi, t, x, y, v, c):
        self.fi, self.t, self.x, self.y, self.v, self.c = [fi], [t], [x], [y], [v], [c]
        self.before, self.after = [], []   # core brightness where the spot was just before / just after
        self.early = {}                    # frame -> core map crop, for the frames before the start
        self.on = self.off = self.est = 0.0
        self.u = self.w = None             # estimated switch-on and switch-off times
        self.kind = ""

    @property
    def a(self):
        return self.t[0] if self.u is None else self.u

    @property
    def b(self):
        return self.t[-1] if self.w is None else self.w

    def vel(self):
        if len(self.t) >= 2 and self.t[-1] > self.t[0]:
            dt = self.t[-1] - self.t[0]
            return (self.x[-1] - self.x[0]) / dt, (self.y[-1] - self.y[0]) / dt
        return 0.0, 0.0


def link(prev_xy, cur_xy, gates):
    """Match current spots to previous ones (min total distance within each gate)."""
    if not prev_xy or not cur_xy:
        return []
    P = np.array(prev_xy)
    C = np.array(cur_xy)
    D = np.hypot(P[:, None, 0] - C[None, :, 0], P[:, None, 1] - C[None, :, 1])
    g = np.array(gates)[:, None]
    big = 1e6
    Dm = np.where(D <= g, D, big)
    pairs = []
    if linear_sum_assignment is not None:
        ri, ci = linear_sum_assignment(Dm)
        pairs = [(i, j) for i, j in zip(ri, ci) if Dm[i, j] < big]
    else:
        used_i, used_j = set(), set()
        for k in np.argsort(Dm, axis=None):
            i, j = divmod(int(k), Dm.shape[1])
            if Dm[i, j] >= big:
                break
            if i not in used_i and j not in used_j:
                used_i.add(i)
                used_j.add(j)
                pairs.append((i, j))
    return pairs


CROP = 12   # half-size of the core-map crops kept for the frames before a segment


def finish_segment(s, cores, p, frame_t):
    """Was the spot clearly darker just before and just after (it switched
    on, then off)? How long was it lit? Sets s.kind."""
    vx, vy = s.vel()
    n = len(frame_t)
    s.before, s.after = [], []
    for k in (s.fi[0] - 1, s.fi[0] - 2):
        if k in s.early and 0 <= k < n:
            dt = frame_t[k] - s.t[0]
            x, y = s.x[0] + vx * dt, s.y[0] + vy * dt
            crop, ox, oy = s.early[k]
            s.before.append(sample(crop, x - ox, y - oy))
    for k in (s.fi[-1] + 1, s.fi[-1] + 2):
        if k in cores and k < n:
            dt = frame_t[k] - s.t[-1]
            s.after.append(sample(cores[k], s.x[-1] + vx * dt, s.y[-1] + vy * dt))
    s.early = {}
    on = float(np.percentile(s.c, 75))
    contrast = float(np.percentile(s.v, 75))
    offs_b = [v for v in s.before if v is not None]
    offs_a = [v for v in s.after if v is not None]
    need = max(p.swing_min, p.swing_frac * contrast)
    ok_b = (not offs_b) or (on - min(offs_b) >= need)
    ok_a = (not offs_a) or (on - min(offs_a) >= need)
    off = min(offs_b + offs_a) if (offs_b or offs_a) else on - contrast
    s.on, s.off = on, off
    span = max(on - off, 1e-6)

    def weight(fi):
        lo, hi = frame_t[max(0, fi - 1)], frame_t[min(n - 1, fi + 1)]
        return (hi - lo) / 2 if hi > lo else 1 / 30

    est = 0.0
    for fi, c in zip(s.fi, s.c):
        est += float(np.clip((c - off) / span, 0, 1)) * weight(fi)
    # the frames just outside: a light switching on or off partway through an exposure
    for v, fi in ((s.before[0] if s.before else None, s.fi[0] - 1), (s.after[0] if s.after else None, s.fi[-1] + 1)):
        if v is not None and 0 <= fi < n:
            est += float(np.clip((v - off) / span, 0, 1)) * weight(fi)
    s.est = est
    # when did it switch on and off? A partly lit frame just outside means it
    # switched during that frame's exposure; otherwise it switched somewhere
    # between the last dark frame and the first lit one (take the middle).
    E = p.exposure_s or EXPOSURE_FALLBACK_S     # exposure, s (from meta.json via analyse())
    f0, f1 = s.fi[0], s.fi[-1]
    s.u, s.w = s.t[0], s.t[-1]
    if f0 - 1 >= 0:
        fb = float(np.clip((s.before[0] - off) / span, 0, 1)) if (s.before and s.before[0] is not None) else 0.0
        tp = frame_t[f0 - 1]
        s.u = tp - fb * E if fb > 0.15 else (tp + s.t[0]) / 2
    if f1 + 1 < n:
        fa = float(np.clip((s.after[0] - off) / span, 0, 1)) if (s.after and s.after[0] is not None) else 0.0
        tn = frame_t[f1 + 1]
        s.w = tn - (1 - fa) * E if fa > 0.15 else (s.t[-1] + tn) / 2
    if not (ok_b and ok_a):
        s.kind = "steady"            # never clearly switched on/off: a lamp, a bright speck
    elif est <= p.max_pulse_s:
        s.kind = "pulse"
    elif est <= p.max_sync_s:
        s.kind = "sync"
    else:
        s.kind = "long"


class SegmentFinder:
    """Stages 1-2 one frame at a time (find_segments over a whole video, and
    tools/live while frames arrive). frame_t: the frame times so far, indexed
    by frame number (a list that push() appends to when live=True, or the
    whole recording's array)."""

    def __init__(self, p, frame_t=None):
        self.p = p
        self.sf = SpotFinder(p)
        self.live = frame_t is None
        self.frame_t = [] if frame_t is None else frame_t
        self.cores = {}          # frame index -> core brightness map (last few frames)
        self.active, self.waiting = [], []
        self.spots = []          # the last frame's spots (x, y, response)
        self.core = None         # the last frame's core map

    def push(self, fi, img, t=None):
        """Add frame fi (its time t when live). Returns the segments that ended
        and were finished (kind set) with this frame."""
        p = self.p
        if self.live:
            assert fi == len(self.frame_t), "frames must come in order, numbered from 0"
            self.frame_t.append(float(t))
        t = float(self.frame_t[fi])
        frame_t = self.frame_t
        core, R = self.sf.maps(img)
        self.core = core
        cores = self.cores
        cores[fi] = core
        for k in [k for k in cores if k < fi - 4]:
            del cores[k]
        done = []
        if not np.isfinite(t):
            self.spots = []
            return done
        # segments that ended: finish once the two frames after them are in
        still = []
        for s in self.waiting:
            if fi >= s.fi[-1] + 2:
                finish_segment(s, cores, p, frame_t)
                done.append(s)
            else:
                still.append(s)
        self.waiting = still
        keep = []
        for s in self.active:
            if t - s.t[-1] > p.link_gap_s:
                if fi >= s.fi[-1] + 2:
                    finish_segment(s, cores, p, frame_t)
                    done.append(s)
                else:
                    self.waiting.append(s)
            else:
                keep.append(s)
        active = self.active = keep
        spots, _ = self.sf.spots(R)
        self.spots = spots
        # where each segment's spot should be now: a new one anywhere within
        # reach; one seen twice or more, straight on from its last step (a
        # spot that jumps sideways is something else lighting up next to it,
        # e.g. an LED beside a bright corner of the robot's top)
        preds, gates = [], []
        for s in active:
            dt = max(t - s.t[-1], 1 / 60)
            if len(s.t) >= 2 and s.t[-1] > s.t[-2]:
                d0 = s.t[-1] - s.t[-2]
                preds.append((s.x[-1] + (s.x[-1] - s.x[-2]) / d0 * dt, s.y[-1] + (s.y[-1] - s.y[-2]) / d0 * dt))
                gates.append(2.5 + 20.0 * dt)
            else:
                preds.append((s.x[-1], s.y[-1]))
                gates.append(3.0 + p.vmax_px * dt)
        pairs = link(preds, [(x, y) for x, y, _ in spots], gates)
        matched = set()
        for i, j in pairs:
            s = active[i]
            x, y, v = spots[j]
            s.fi.append(fi)
            s.t.append(t)
            s.x.append(x)
            s.y.append(y)
            s.v.append(v)
            s.c.append(float(core[int(round(y)), int(round(x))]))
            matched.add(j)
        for j, (x, y, v) in enumerate(spots):
            if j in matched:
                continue
            s = Segment(fi, t, x, y, v, float(core[int(round(y)), int(round(x))]))
            xi, yi = int(round(x)), int(round(y))
            x0, y0 = max(0, xi - CROP), max(0, yi - CROP)
            for k in (fi - 1, fi - 2):
                if k in cores:
                    s.early[k] = (cores[k][y0:yi + CROP + 1, x0:xi + CROP + 1].copy(), x0, y0)
            active.append(s)
        return done

    def flush(self):
        """Finish every segment still open (end of the video)."""
        out = []
        for s in self.active + self.waiting:
            finish_segment(s, self.cores, self.p, self.frame_t)
            out.append(s)
        self.active, self.waiting = [], []
        return out


def find_segments(video, frame_t, p, progress=None, max_frames=None):
    """Stages 1-2 over the whole video. Returns (segments, n_frames_read)."""
    import cv2
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {video}")
    finder = SegmentFinder(p, frame_t)
    done = []
    fi = -1
    n_t = len(frame_t)
    while True:
        ok, img = cap.read()
        if not ok:
            break
        fi += 1
        if fi >= n_t or (max_frames and fi >= max_frames):
            fi -= 1
            break
        done += finder.push(fi, img)
        if progress and fi % 1500 == 0:
            progress(fi, n_t)
    cap.release()
    done += finder.flush()
    done.sort(key=lambda s: s.a)
    for i, s in enumerate(done):
        s.id = i
    return done, fi + 1


# ------------------------------------------------------------------ lights (LED pairs)
class Light:
    """One pulse (or flash) of one robot: one or two LED segments merged."""

    def __init__(self, segs, kind):
        self.segs = segs
        self.kind = kind
        frames = sorted(set(f for s in segs for f in s.fi))
        pos = {}
        if len(segs) == 2:
            A = {f: (x, y) for f, x, y in zip(segs[0].fi, segs[0].x, segs[0].y)}
            B = {f: (x, y) for f, x, y in zip(segs[1].fi, segs[1].x, segs[1].y)}
            common = [f for f in frames if f in A and f in B]
            half = np.median([[(B[f][0] - A[f][0]) / 2, (B[f][1] - A[f][1]) / 2] for f in common], axis=0)
            self.half = (float(half[0]), float(half[1]))     # from LED A to the midpoint
            for f in frames:
                if f in A and f in B:
                    pos[f] = ((A[f][0] + B[f][0]) / 2, (A[f][1] + B[f][1]) / 2, 2)
                elif f in A:
                    pos[f] = (A[f][0] + half[0], A[f][1] + half[1], 1)
                else:
                    pos[f] = (B[f][0] - half[0], B[f][1] - half[1], 1)
        else:
            self.half = None
            s = segs[0]
            for f, x, y in zip(s.fi, s.x, s.y):
                pos[f] = (x, y, 1)
        self.frames = frames
        self.pos = pos
        self.ts = {}
        for s in segs:
            for f, t in zip(s.fi, s.t):
                self.ts[f] = t
        self.a = min(s.a for s in segs)
        self.b = max(s.b for s in segs)
        self.est = max(s.est for s in segs)
        self.paired = len(segs) == 2

    @property
    def start_xy(self):
        x, y, _ = self.pos[self.frames[0]]
        return x, y

    @property
    def end_xy(self):
        x, y, _ = self.pos[self.frames[-1]]
        return x, y

    def velocity(self):
        if len(self.frames) < 2:
            return None
        f0, f1 = self.frames[0], self.frames[-1]
        dt = self.ts[f1] - self.ts[f0]
        if dt <= 0:
            return None
        return ((self.pos[f1][0] - self.pos[f0][0]) / dt, (self.pos[f1][1] - self.pos[f0][1]) / dt)


def pair_lights(segs, p, kinds=("pulse",)):
    """Merge the two LEDs of one robot: segments of the same kind that start and
    end together (within pair_dt_s) and lie within pair_max_cm of each other."""
    segs = [s for s in segs if s.kind in kinds]
    maxd = p.pair_max_cm * p.px_per_cm
    cand = []
    for i, s in enumerate(segs):
        for j in range(i + 1, len(segs)):
            q = segs[j]
            if q.a - s.a > p.pair_dt_s:
                break
            if s.kind != q.kind or abs(q.b - s.b) > p.pair_dt_s:
                continue
            fs = {f: (x, y) for f, x, y in zip(s.fi, s.x, s.y)}
            d = [math.hypot(fs[f][0] - x, fs[f][1] - y) for f, x, y in zip(q.fi, q.x, q.y) if f in fs]
            if not d:
                continue
            dm = float(np.median(d))
            if 1.0 < dm <= maxd:
                cand.append((abs(q.a - s.a) + abs(q.b - s.b) + 0.0005 * dm, i, j))
    cand.sort()
    used = set()
    lights = []
    for _, i, j in cand:
        if i in used or j in used:
            continue
        used.update((i, j))
        lights.append(Light([segs[i], segs[j]], segs[i].kind))
    for i, s in enumerate(segs):
        if i not in used:
            lights.append(Light([s], s.kind))
    lights.sort(key=lambda L: L.a)
    return lights


# ------------------------------------------------------------------ stage 4: rate decoding
def start_density(r, T, p):
    """Density of a flash start's offset r (s) from its robot's beat (period
    T): on the beat (frame timing noise), now and then delayed (code
    blocked), or, rarely, anywhere (a mis-timed or foreign flash: so that one
    odd flash can't veto a code)."""
    g = np.exp(-0.5 * (r / p.sigma_s) ** 2) / (p.sigma_s * math.sqrt(2 * math.pi))
    u = np.where((r >= -p.tol_s) & (r <= p.delay_s + p.tol_s), 1.0 / (p.delay_s + 2 * p.tol_s), 0.0)
    return (1 - p.eps_delay - p.eps_out) * g + p.eps_delay * u + p.eps_out / T


def decode_rate(starts, p):
    """Which code flashes at this rhythm? starts: flash start times (s).

    For each code, the flash starts should sit on a beat of that code's period
    (whole numbers of periods apart: missed flashes don't matter), each within
    frame-timing noise, now and then late. Compared with flashes at random
    moments, that gives each code a likelihood; with a prior, probabilities
    for each code and for "no robot code" (none). The long-run rate (total
    time over the number of periods spanned) must also match the code's
    period, so a lamp flickering at, say, 0.25 s is not taken for 0.3 s.
    Returns dict(probs={code: P}, p_none, code (most likely), period_s
    (median of the flash-to-flash intervals that are one period), n
    (intervals), rhythm (share of flashes on the best code's beat))."""
    a = np.sort(np.asarray(starts, float))
    d = np.diff(a)
    codes = list(range(1, p.max_code + 1))
    dd = d[(d > 0.12) & (d <= p.max_interval_s)]
    if len(a) < 2 or len(dd) == 0:
        pr = {n: (1 - p.prior_none) / len(codes) for n in codes}
        return dict(probs=pr, p_none=p.prior_none, code=0, period_s=float("nan"), n=0, rhythm=0.0)
    ll, onb = {}, {}
    for n in codes:
        T = p.period(n)
        psi = np.mod(a, T)
        best, best_r = -1e18, None
        for c in psi:                                      # beat through each flash in turn
            r = np.mod(psi - c + T / 2, T) - T / 2
            r = r - np.median(r[np.abs(r) <= p.tol_s])     # centre it on the on-beat flashes
            v = float(np.sum(np.log(start_density(r, T, p) * T)))
            if v > best:
                best, best_r = v, r
        M = float(np.sum(np.maximum(1, np.round(dd / T))))
        rate = float(np.sum(dd)) / M
        s_rate = 0.03 / M + 0.002
        ll[n] = math.log((1 - p.prior_none) / len(codes)) + best - 0.5 * ((rate - T) / s_rate) ** 2
        onb[n] = float(np.mean(np.abs(best_r) <= 2.5 * p.sigma_s))
    ll[0] = math.log(p.prior_none)
    mx = max(ll.values())
    w = {k: math.exp(v - mx) for k, v in ll.items()}
    tot = sum(w.values())
    probs = {n: w[n] / tot for n in codes}
    bestc = max(codes, key=lambda n: probs[n])
    base = float(np.percentile(dd, 25))
    one = dd[dd <= 1.4 * base]
    return dict(probs=probs, p_none=w[0] / tot, code=bestc, period_s=float(np.median(one)), n=len(dd),
                rhythm=onb[bestc])


# ------------------------------------------------------------------ stage 3: tracks
class Track:
    def __init__(self, tid):
        self.id = tid
        self.lights = []
        self.dec = None
        self.quality = 0.0
        self.beat = None         # period (s) once the track's code is clear

    @property
    def last(self):
        return self.lights[-1]

    def add(self, L, p):
        self.lights.append(L)
        if len(self.lights) >= 3:
            self.dec = decode_rate([x.a for x in self.lights[-12:]], p)
            best = max(self.dec["probs"].values())
            self.quality = best
            self.beat = p.period(self.dec["code"]) if best >= 0.8 else None

    def velocity(self):
        L = self.last
        v = L.velocity()
        if len(self.lights) >= 2:
            P = self.lights[-2]
            dt = L.a - P.b
            if 0 < dt < 2.0:
                (x0, y0), (x1, y1) = P.end_xy, L.start_xy
                v2 = ((x1 - x0) / dt, (y1 - y0) / dt)
                v = v2 if v is None else ((v[0] + v2[0]) / 2, (v[1] + v2[1]) / 2)
        return v

    def position_at(self, t, max_gap=1.0):
        """Light position at time t, interpolated between this track's flashes
        (None if no flash within max_gap of t)."""
        key = len(self.lights)
        if getattr(self, "_pts_key", None) != key:      # cache until the track changes
            pts = []
            for L in self.lights:
                for f in L.frames:
                    pts.append((L.ts[f], L.pos[f][0], L.pos[f][1]))
            pts.sort()
            self._pts, self._pts_t, self._pts_key = pts, np.array([q[0] for q in pts]), key
        pts, ts = self._pts, self._pts_t
        if not pts:
            return None
        i = int(np.searchsorted(ts, t))
        near = [j for j in (i - 1, i) if 0 <= j < len(ts)]
        if not near or min(abs(ts[j] - t) for j in near) > max_gap:
            return None
        if len(near) == 1 or ts[near[1]] == ts[near[0]]:
            j = near[0]
            return pts[j][1], pts[j][2]
        j0, j1 = near
        w = (t - ts[j0]) / (ts[j1] - ts[j0])
        return pts[j0][1] + w * (pts[j1][1] - pts[j0][1]), pts[j0][2] + w * (pts[j1][2] - pts[j0][2])


def beat_cost(L, T, p):
    """Timing cost of adding light L to track T: once a track's rate is clear,
    its flashes start a whole number of periods after its last one (a start
    can be delayed, which also shifts the next interval). None = doesn't fit."""
    if T.beat is None:
        return 0.0
    d = L.a - T.last.a
    m = max(1, int(round(d / T.beat)))
    e = d - m * T.beat
    if abs(e) <= p.tol_s:
        return abs(e) / 0.025
    if abs(e) <= p.delay_s + p.tol_s:
        return 1.5
    return None


NEW_TRACK_COST = 3.0


def join_cost(T, L, p):
    """Cost of adding light L to track T (lower = better), or None if it can't be
    that track's: too far for its speed, off its predicted path, off its beat."""
    dt = L.a - T.last.b
    if dt < 0.05:
        return None
    (x0, y0) = T.last.end_xy
    (x1, y1) = L.start_xy
    d = math.hypot(x1 - x0, y1 - y0)
    if d > 6.0 + p.vmax_px * dt:
        return None
    v = T.velocity() if dt < 2.0 else None
    if v is not None:
        sp = math.hypot(*v)
        if sp > p.vmax_px:
            v = (v[0] * p.vmax_px / sp, v[1] * p.vmax_px / sp)
        dp = min(math.hypot(x1 - (x0 + v[0] * dt), y1 - (y0 + v[1] * dt)), d)
        # a robot can't jump: off its predicted path by at most what
        # braking/accelerating (amax_cms2) or stopping could explain
        if dp > 6.0 + 0.5 * p.amax_cms2 * p.px_per_cm * dt * dt + 0.6 * math.hypot(*v) * dt:
            return None
        c = dp / (6.0 + 0.35 * p.vmax_px * dt)
    else:
        c = d / (6.0 + 0.5 * p.vmax_px * dt)
    g = beat_cost(L, T, p)
    if g is None:
        return None
    c += g
    # a robot shows two spots (its LEDs) and a lamp one: a light that
    # doesn't match what the track has mostly shown is less likely its
    recent = T.lights[-3:]
    n_pair = sum(x.paired for x in recent)
    if (not L.paired and 2 * n_pair > len(recent)) or (L.paired and 2 * n_pair < len(recent)):
        c += 1.5
    # prefer tracks that already flash like a robot's code
    c += 1.0 * (1.0 - T.quality)
    return c


def assign_batch(tracks, batch, p, new_track):
    """Add a batch of lights that started together (within 60 ms) to the open
    tracks, or start new tracks (new_track() makes one and is told nothing
    else). Returns [(light, track, new)] in batch order."""
    NEW = NEW_TRACK_COST
    cost = np.full((len(batch), len(tracks) + len(batch)), 1e6)
    for bi, L in enumerate(batch):
        cost[bi, len(tracks) + bi] = NEW
        for ti, T in enumerate(tracks):
            c = join_cost(T, L, p)
            if c is not None:
                cost[bi, ti] = c
    if linear_sum_assignment is not None:
        ri, ci = linear_sum_assignment(cost)
    else:
        ri, ci, used = [], [], set()
        for bi in range(len(batch)):
            for cj in np.argsort(cost[bi]):
                if cj not in used:
                    used.add(cj)
                    ri.append(bi)
                    ci.append(cj)
                    break
    out = []
    n_old = len(tracks)
    for bi, cj in zip(ri, ci):
        L = batch[bi]
        if cj < n_old and cost[bi, cj] < 1e5:
            tracks[cj].add(L, p)
            out.append((L, tracks[cj], False))
        else:
            T = new_track()
            T.add(L, p)
            tracks.append(T)
            out.append((L, T, True))
    return out


def build_tracks(lights, p):
    tracks, closed = [], []
    lights = [L for L in lights if L.kind == "pulse"]
    i = 0
    while i < len(lights):
        batch = [lights[i]]
        j = i + 1
        while j < len(lights) and lights[j].a - lights[i].a <= 0.06:
            batch.append(lights[j])
            j += 1
        i = j
        t_now = batch[0].a
        alive = []
        for T in tracks:
            (closed if t_now - T.last.b > p.track_gap_s else alive).append(T)
        tracks = alive
        assign_batch(tracks, batch, p, lambda: Track(len(tracks) + len(closed) + 1000))
    allt = closed + tracks
    allt.sort(key=lambda T: T.lights[0].a)
    for n, T in enumerate(allt, 1):
        T.id = n
    return allt


def foreign_pulses(T, segs, p, radius_cm=3.0):
    """Lights that switched on where T's light was at that moment but are not
    part of T (flashes, flash-length or longer lit spells), per LED. A robot's
    own spot shows nothing but its own flashes; a flickering lamp split into
    several 'tracks' shows the others' flashes."""
    mine = set(id(s) for L in T.lights for s in L.segs)
    r = radius_cm * p.px_per_cm
    t0, t1 = T.lights[0].a, T.lights[-1].b
    n = 0
    for s in segs:
        if s.a > t1 + 1:
            break                                    # segments are in time order
        if s.kind not in ("pulse", "sync", "long") or id(s) in mine or s.a < t0 or s.a > t1:
            continue
        q = T.position_at(s.a)
        if q is None:
            continue
        if math.hypot(s.x[0] - q[0], s.y[0] - q[1]) <= r:
            n += 1
    return n / max(1, max(len(L.segs) for L in T.lights))   # LED pairs: count each event once


# ------------------------------------------------------------------ whole pipeline
def decode_tracks(tracks, p, segs=None):
    """Decode every track. A track is reported (T.accepted) if it has at least
    min_pulses flashes; it is NAMED only if its best code is probable enough
    and its spot isn't shared with other flashing (then name '' and code 0,
    but the probabilities are still given)."""
    for T in tracks:
        dec = decode_rate([L.a for L in T.lights], p)
        T.dec = dec
        T.foreign = foreign_pulses(T, segs, p) if segs is not None else 0.0
        T.shared = T.foreign / max(1, len(T.lights))
        best = dec["code"]
        T.score = dec["probs"].get(best, 0.0)
        named = (best and T.score >= p.min_name_p and T.shared <= p.max_foreign and dec["n"] >= p.min_name_intervals
                 and dec["rhythm"] >= p.min_rhythm)
        T.code = best if named else 0
        T.name = p.names.get(best, f"code{best}") if named else ""
        T.accepted = len(T.lights) >= p.min_pulses
    return tracks


def single_led_fix(T):
    """Lights where only one LED was seen: move to the pair midpoint using the
    nearest (in time) light of the same track where both were seen."""
    paired = [L for L in T.lights if L.paired]
    if not paired:
        return
    for L in T.lights:
        if L.paired:
            continue
        near = min(paired, key=lambda q: abs(q.a - L.a))
        if abs(near.a - L.a) > 2.0:
            continue
        hx, hy = near.half
        s = L.segs[0]
        # which of the two LEDs is this? the one whose position + its half vector
        # lands closer to where the pair's midpoint was
        mx, my = near.start_xy
        x0, y0 = s.x[0], s.y[0]
        sign = 1 if math.hypot(x0 + hx - mx, y0 + hy - my) <= math.hypot(x0 - hx - mx, y0 - hy - my) else -1
        for f in list(L.pos):
            x, y, ns = L.pos[f]
            L.pos[f] = (x + sign * hx, y + sign * hy, ns)


def analyse(recording, p=None, video=None, progress=None, max_frames=None):
    """Run every stage on a recording folder. Returns (tracks, segments, lights, frame_t)."""
    p = p or Params()
    if not p.exposure_s:
        from dataclasses import replace
        p = replace(p, exposure_s=exposure_from_meta(recording)[0])
    frame_t = read_frame_times(recording)
    segs, n = find_segments(video or os.path.join(recording, "video.avi"), repair_late_frames(frame_t), p, progress,
                            max_frames)
    lights = pair_lights(segs, p, kinds=("pulse", "sync"))
    tracks = build_tracks(lights, p)
    tracks = decode_tracks(tracks, p, segs)
    for T in tracks:
        single_led_fix(T)
    return tracks, segs, lights, frame_t[:n]


def rows_for(tracks, frame_t, p, interpolate=False):
    """Output rows: BASE_COLS values then one probability per code, then p_none."""
    rows = []
    for T in tracks:
        if not T.accepted:
            continue
        probs = [T.dec["probs"].get(k, 0.0) for k in sorted(p.names)] + [T.dec["p_none"]]
        tail = [T.dec["period_s"], T.shared] + probs
        score = T.score if T.code else max(T.dec["probs"].values())
        prev = None
        for L in T.lights:
            if interpolate and prev is not None:
                f0, f1 = prev.frames[-1], L.frames[0]
                (x0, y0), (x1, y1) = prev.end_xy, L.start_xy
                t0, t1 = prev.ts[f0], L.ts[f1]
                if 0 < t1 - t0 <= 1.0:
                    for f in range(f0 + 1, f1):
                        t = float(frame_t[f])
                        if not np.isfinite(t):
                            continue
                        w = (t - t0) / (t1 - t0)
                        rows.append([f, t, T.name, x0 + w * (x1 - x0), y0 + w * (y1 - y0), T.code, score,
                                     T.id, 0, 0] + tail)
            for f in L.frames:
                x, y, ns = L.pos[f]
                rows.append([f, float(frame_t[f]), T.name, x, y, T.code, score, T.id, 1, ns] + tail)
            prev = L
    rows.sort(key=lambda r: (r[0], r[7]))
    return rows


def write_rows(path, rows, names=None):
    names = names or DEFAULT_NAMES
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(out_cols(names))
        for r in rows:
            w.writerow([r[0], f"{r[1]:.4f}", r[2], f"{r[3]:.2f}", f"{r[4]:.2f}", r[5], f"{r[6]:.3f}", r[7], r[8],
                        r[9], f"{r[10]:.3f}", f"{r[11]:.2f}"] + [f"{q:.3f}" for q in r[12:]])


def read_blink_csv(path):
    """blink_detections.csv -> list of dicts with numbers converted; 'probs'
    maps each robot name (and 'none') to its probability."""
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            probs = {k[2:]: float(v) for k, v in r.items() if k.startswith("p_") and v not in ("", None)}
            out.append(dict(frame=int(r["frame"]), t=float(r["t_wall"]), robot=r["robot"], x=float(r["x_px"]),
                            y=float(r["y_px"]), code=int(r["code"]), score=float(r["score"]),
                            track=int(r.get("track") or 0), lit=int(r.get("lit") or 1),
                            n_spots=int(r.get("n_spots") or 0), period=float(r.get("period_s") or "nan"),
                            probs=probs))
    return out


def summary(tracks, segs, lights, p, t0=0.0):
    kinds = {}
    for s in segs:
        kinds[s.kind] = kinds.get(s.kind, 0) + 1
    lines = [f"on-segments: {len(segs)} (" + ", ".join(f"{v} {k}" for k, v in sorted(kinds.items())) + ")",
             f"flashes: {sum(1 for L in lights if L.kind == 'pulse')} "
             f"({sum(1 for L in lights if L.kind == 'pulse' and L.paired)} with both LEDs), "
             f"{sum(1 for L in lights if L.kind == 'sync')} sync-like flashes (left out)"]
    acc = [T for T in tracks if T.accepted]
    lines.append(f"tracks with {p.min_pulses}+ flashes: {len(acc)}, named {sum(1 for T in acc if T.code)}")
    hdr = "  ".join(f"{p.names[k]:>6s}" for k in sorted(p.names))
    lines.append(f"  track  name     from..to (s)       flashes  period  {hdr}    none  shared")
    for T in sorted(acc, key=lambda T: (T.code == 0, T.lights[0].a))[:40]:
        d = T.dec
        pr = "  ".join(f"{d['probs'][k]:6.2f}" for k in sorted(p.names))
        lines.append(f"  {T.id:5d}  {T.name or '?':8s} {T.lights[0].a - t0:7.1f}..{T.lights[-1].b - t0:7.1f}  "
                     f"{len(T.lights):7d}  {d['period_s']:6.3f}  {pr}  {d['p_none']:6.2f}  {T.shared:6.2f}")
    if len(acc) > 40:
        lines.append(f"  ... and {len(acc) - 40} more unnamed tracks")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", help="camrec.py recording folder (video.avi + frames.csv)")
    ap.add_argument("--out", help="output CSV (default <recording>/blink_detections.csv)")
    ap.add_argument("--video", help="video file, if not <recording>/video.avi")
    ap.add_argument("--calib", help="floor_calibration.json, for the pixel scale (default: beside the recording)")
    ap.add_argument("--px-per-cm", type=float, help="pixel scale (default from --calib, else 4)")
    ap.add_argument("--names", help="code to name, e.g. 1=Left,2=Center,3=Right,4=other (default: from --session, "
                                    "else that)")
    ap.add_argument("--session", help="console log folder of the same runs: code -> name from each robot's PARAM "
                                      "blink_code and name, as the firmware picks it (warns about clashes)")
    ap.add_argument("--interpolate", action="store_true", help="also write rows between flashes (lit=0)")
    ap.add_argument("--vmax-cms", type=float, default=40.0, help="fastest robot speed to allow (cm/s)")
    ap.add_argument("--min-name-p", type=float, default=0.6, help="name a track only above this probability")
    ap.add_argument("--max-frames", type=int, help="only the first N frames (quick look)")
    args = ap.parse_args(argv)
    names = parse_names(args.names)
    if args.session:
        sess_names = names_from_session(args.session)
        if not args.names:
            names = sess_names
        elif sess_names != names:
            print(f"warning: --names {names} differs from what the session's robots blink {sess_names}")
    elif not args.names:
        print("note: no --session or --names: assuming Left=1, Center=2, Right=3, other=4 (PARAM blink_code 0 "
              "everywhere; give --session to check)")
    p = Params(names=names, vmax_cms=args.vmax_cms, min_name_p=args.min_name_p)
    calib = args.calib or os.path.join(args.recording, "floor_calibration.json")
    if args.px_per_cm:
        p.px_per_cm = args.px_per_cm
    elif os.path.exists(calib):
        try:
            p.px_per_cm = px_per_cm_from_calib(calib)
        except (ValueError, OSError, KeyError) as e:
            print(f"note: {calib}: {e}; using {p.px_per_cm} px/cm")
    print(f"pixel scale {p.px_per_cm:.2f} px/cm")
    p.exposure_s, src = exposure_from_meta(args.recording)
    print(f"exposure {p.exposure_s * 1000:.1f} ms ({src})")

    def prog(i, n):
        print(f"  frame {i} of {n}")
    tracks, segs, lights, frame_t = analyse(args.recording, p, args.video, prog, args.max_frames)
    rows = rows_for(tracks, frame_t, p, args.interpolate)
    out = args.out or os.path.join(args.recording, "blink_detections.csv")
    write_rows(out, rows, p.names)
    print(summary(tracks, segs, lights, p, float(frame_t[0]) if len(frame_t) else 0.0))
    print("  (times in s from the first frame; shared = other flashing on the track's spot, per flash)")
    print(f"wrote {len(rows)} rows to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
