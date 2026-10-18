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

For every calib run (RAW step 2 = roll forward 3 cm) and every start nudge
(RAW step -1 = roll forward 2 cm and back) it reports:
  - who: the robot named by the log (the robot whose RAW shows the roll at
    that moment) and the robot named by the blink code of the light that
    moved at that moment, and whether they agree
  - heading: the direction of the forward roll, measured on the video by
    comparing the picture around the robot just before the roll with the
    picture during the pause at its far end (phase correlation: the shift
    that best lines the two up; the floor does not move, the robot does)
  - LED offset: where the light (midpoint of the two LEDs) sits on the robot,
    forward and left of the robot's centre, in px and cm (needs --detections)

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
import runs as fr  # noqa: E402
import blink  # noqa: E402

OUT_COLS = ["run", "robot_log", "experiment", "event", "t_video", "robot_blink", "blink_code", "blink_score",
            "blink_p_log", "identity", "others_moving", "lights_moved", "onset_diff_ms", "heading_img_deg", "heading_floor_deg",
            "heading_method", "moved_px", "moved_cm", "commanded_cm", "motion_quality", "det_heading_floor_deg",
            "light_x_px", "light_y_px", "centre_x_px", "centre_y_px", "led_fwd_px", "led_left_px", "led_fwd_cm",
            "led_left_cm", "note"]


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


def measure_shift(ref, paus, x, y, half, e):
    """How far the robot near (x, y) moved between the still picture and the
    pause picture: (dx, dy, quality) or None. e = expected distance, px.
    First pass: phase correlation of the same window in both pictures. Second
    pass: windows moved by half the shift each way, so the robot sits in the
    same place in both windows (a window that cuts the robot differently in the
    two pictures pulls the answer by up to ~0.7 deg); the remaining small shift
    is measured near zero."""
    A, ax, ay = crop(ref, x, y, half)
    B, bx, by = crop(paus, x, y, half)
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
                      t=np.array([r["t"] for r in v]), x=np.array([r["x"] for r in v]), y=np.array([r["y"] for r in v]))
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


def analyse_event(run, ev, kind, cmd_cm, off, frames, tracks, dets, geo, all_runs, names):
    """One forward roll: returns an output row (dict)."""
    row = dict(run=run.run_id, robot_log=run.robot, experiment=run.experiment, event=kind, commanded_cm=cmd_cm)
    notes = []
    if ev is None or ev.get("onset") is None:
        row["note"] = "no forward roll in the wheel log" if ev else f"no RAW step {'2' if kind == 'calib' else '-1'}"
        return row
    v = lambda t: t + off            # laptop -> video clock
    row["t_video"] = round(v(ev["onset"]), 3)
    # still before the roll
    if kind == "calib":
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
    ref, nref = frames.median(v(r0[0]), v(r0[1]), kind=0)
    paus, npause = frames.median(v(p0), v(p1), kind=0)
    if ref is None or paus is None:
        row["note"] = "video frames missing for the still or the pause"
        return row
    moved = []
    for c in cands:
        ppc = geo.px_per_cm(c["x"], c["y"])
        half = int(round(14 * ppc))
        e = cmd_cm * ppc
        res = measure_shift(ref, paus, c["x"], c["y"], half, e)
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
    row.update(heading_img_deg=round(hi, 2), heading_floor_deg=round(hf, 2), heading_method="video shift",
               moved_px=round(c["mag"], 2), moved_cm=round(math.hypot(f1[0] - f0[0], f1[1] - f0[1]), 2),
               motion_quality=round(c["q"], 1))
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
    row["note"] = "; ".join(notes)
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
    ap.add_argument("--names", help="code to name, e.g. 1=Left,2=Center,3=Right,4=other")
    ap.add_argument("--out", help="default <recording>/calibration_results.csv")
    args = ap.parse_args(argv)
    names = blink.parse_names(args.names)
    calib = args.calib or os.path.join(args.recording, "floor_calibration.json")
    if not os.path.exists(calib):
        sys.exit(f"{calib} not found: give --calib floor_calibration.json")
    geo = Geo(calib, args.led_height_cm)
    if geo.par is None:
        print("note: floor_calibration.json has no camera height/nadir: light positions not corrected for height")
    runs, fits = fr.load_runs(args.session)
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
        blink.main([args.recording, "--out", bpath] + (["--names", args.names] if args.names else [])
                   + (["--video", args.video] if args.video else []))
    tracks = blink_tracks(blink.read_blink_csv(bpath))
    dets = read_detections(args.detections) if args.detections else None
    frames = Frames(args.recording, args.video)
    out = []
    for run in sorted(runs, key=lambda r: r.t_start):
        cmd = run.params.get("start_nudge_cm", 2.0)
        cmd = 2.0 if cmd != cmd else cmd
        if run.experiment == "calib":
            ev = motion_event(run, 2)
            out.append(analyse_event(run, ev, "calib", max(3.0, cmd), off, frames, tracks, dets, geo, runs, names))
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
    offs = [(r["led_fwd_cm"], r["led_left_cm"]) for r in out if r.get("led_fwd_cm") not in (None, "")]
    if offs:
        print(f"light offset from robot centre (median of {len(offs)}): forward "
              f"{st.median(o[0] for o in offs):.2f} cm, left {st.median(o[1] for o in offs):.2f} cm")


if __name__ == "__main__":
    sys.exit(main())
