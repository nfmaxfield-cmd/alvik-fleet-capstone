#!/usr/bin/env python3
"""Camera discovery inside robot finding (build e13; Nick's design, 10 Nov 2026).

Which camera sees which robot is not typed: it is found. Before a run:

  1. CLEAR SPACE  the person confirms every robot has clear space round it: a radius DERIVED from what the discovery
                  motion needs (clearance(): half the body's diagonal, the stroke, the stop at the stroke's speed, the
                  robot's own drift over the stroke before any camera has it, and the motion detector's margin to keep
                  two neighbours apart), said in cm and inches with its derivation. Not a typed 12 in.
  2. EVERY CAMERA every camera the Pi can find is opened (find_cameras(): the recorder's streams, then /dev/video
                  cameras the recorder does not hold), not only the named ones.
  3. STROKES      each robot in turn makes a short straight stroke forward and back (OB.NUDGE_CM, the heading check's
                  own length, at DISC_SPEED): a straight stroke never swings a corner into the unknown, unlike a turn.
                  Only one robot moves at a time, so whatever moves in a picture during its stroke is that robot.
  4. WHO, WHERE   in every camera's pictures the motion (the picture against its still background, as the labelling
                  motion detector does: tools/label/motion.py's thresholds) and, where present, roof tags decide
                  which cameras see which robot and where: a COVERAGE MAP (camera -> robots seen, pixel and table
                  positions). A camera that sees no robot is reported and left out of the run.
  5. MEASURED     per robot and camera: the start heading (the direction the stroke moved, through the camera's model
                  when it has one), the roof tag's turn on the robot (tag heading - motion heading) when there is a tag,
                  the robot's size in pixels (a readability proxy), the light (whole picture and round the robot), the
                  camera's tilt (its model; else a hint from the stroke's pixels against the body's), and more:
                  the stroke's length in the picture (px per cm), the motion's contrast against the picture's noise,
                  neighbours too close for the detector to separate, frames used.

Typed camera names (the site's camera_names / cameras.json) are a first guess only (observe, then trust): kept
behind MIRLT_CAMERA_NAMES=typed (the old way: the typed names decide), compared with what discovery found and said.

    python3 tools/fleet/camera_discovery.py clearance [--cm-per-px 0.24]     the derived clear space, with its derivation
    python3 tools/fleet/camera_discovery.py list                             every camera this Pi can open
    python3 tools/fleet/camera_discovery.py sim [--out DIR]                  the simulator rig (an extra blind camera, a
                                                                             renamed and a missing camera, two robots close)
Writes coverage.json (and coverage.md, words) where asked.
"""
import argparse
import glob
import json
import math
import os
import socket
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "pi"),
           os.path.join(TOOLS, "label"), os.path.join(TOOLS, "live")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

VERSION = "camera-discovery-2026-11-10e14"   # e14 (10 Nov): a wiggle on the spot where a stroke is not safe (the
#   robot's own footprint only: analyse_wiggle gives its position, no heading); each stroke's END position (end_cm), so a
#   drive step can be followed by motion (motion_find.py); robots found by motion at the start of every run.
#   e13: the first version (coverage, strokes, typed names a first guess)

try:
    import cv2
except ImportError:     # pragma: no cover
    cv2 = None

# ---------------------------------------------------------------------------------------------- numbers (and why)
HALF_DIAG_CM = 6.72      # the 9.5 cm body's half diagonal (safety_rules.HALF_DIAG_CM). Not chosen
BODY_CM = 9.5            # the body's side. Not chosen
DISC_SPEED = 5.0         # cm/s: the slowest speed in the free-roam menu (freeroam_core BACK_V). Not chosen here
BLIND_BASE_CM = 2.0      # a robot nobody sees drifts p95 2.0 cm + 0.07 cm per cm driven: MEASURED (29 Oct logs, 22,000
BLIND_PER_CM = 0.07      # stretches; freeroam_core BLIND_BASE / BLIND_PER_CM, run_plan BLIND_ERR_*). Not chosen here
DETECT_CLOSE_PX = 5      # the motion detector joins changed pixels this close (tools/label/motion.py: a 5 x 5 closing),
DETECT_PAD_PX = 4        # and pads each blob by 4 px: a neighbour within these pixels joins the mover's blob. Not chosen
CM_PER_PX_UNKNOWN = 0.5  # CHOSEN: with no camera model, a coarse picture is assumed (640 px over 3.2 m); DOE: no (a
                         # bound: any real camera here is finer, which only shrinks the margin)
MIN_T = 18               # the motion detector's smallest change that counts (0-255) and its noise factor
NOISE_K = 4.0            # (tools/label/motion.py min_threshold 18, noise_factor 4). Not chosen here
MIN_AREA_FRAC = 0.2      # a change smaller than this share of a robot is noise (motion.py min_area_frac). Not chosen
DEFAULT_ROBOT_PX = 30.0  # the robot's size in pixels until measured (motion.py default_robot_px). Not chosen here
BG_FRAMES = 5            # pictures of the still table per camera before each robot's stroke (their median). Not tuned
WIGGLE_DEG = 30.0        # CHOSEN (e14): the find-me wiggle on the spot (+- this, then back). A square roof turned 30 deg
                         # changes a band round its four corners (about a third of its area at the robot's own size),
                         # well over MIN_AREA_FRAC; 90 deg would look the same as the start (a square). DOE row (e14)
SEEN_K = 3.0             # CHOSEN: a camera sees the robot when the changed area during its stroke is this many times
                         # the largest change in the same camera while every robot stood still (and at least
                         # MIN_AREA_FRAC of a robot). DOE row (docs/doe-backlog.md, e13)


def nudge_cm():
    try:
        import observe as OB
        return float(OB.NUDGE_CM)
    except Exception:     # noqa: BLE001
        return 6.0


def stop_cm(v=DISC_SPEED):
    try:
        import safety_rules as SR
        return float(SR.stop_cm(v))
    except Exception:     # noqa: BLE001
        return 1.4


# ---------------------------------------------------------------------------------------------- 1. clear space
def clearance(cm_per_px=None, stroke_cm=None, speed=DISC_SPEED):
    """The clear space each robot needs for the discovery stroke, DERIVED (Nick: "do not use my arbitrary 12 in").
    Returns dict(radius_cm, radius_in, centres_cm, centres_in, gap_cm, terms [(what, cm, why)], words)."""
    L = nudge_cm() if stroke_cm is None else float(stroke_cm)
    cpp = CM_PER_PX_UNKNOWN if cm_per_px is None else float(cm_per_px)
    sep_px = DETECT_CLOSE_PX + DETECT_PAD_PX
    terms = [
        ("half the body's diagonal", HALF_DIAG_CM, "the body's corners swing within this whatever way it faces"),
        ("the stroke", L, "it drives %.0f cm and back (the heading check's own length); its heading is not known yet, "
                          "so the stroke may point any way: a full circle" % L),
        ("its stop at %.0f cm/s" % speed, stop_cm(speed), "the measured 95th-percentile overrun at that speed"),
        ("its own drift before a camera has it", BLIND_BASE_CM + BLIND_PER_CM * L,
         "p95 %.1f + %.2f cm per cm driven unseen (29 Oct logs): the position doubt before it is found" % (
             BLIND_BASE_CM, BLIND_PER_CM)),
        ("the motion detector's margin", sep_px * cpp,
         "%d px (its %d px joining + %d px pad) at %.2f cm per px%s: closer, a neighbour joins the mover's blob" % (
             sep_px, DETECT_CLOSE_PX, DETECT_PAD_PX, cpp, "" if cm_per_px is not None else
             " (no camera model: a coarse picture assumed)")),
    ]
    R = sum(v for _, v, _ in terms)
    centres = R + HALF_DIAG_CM            # a neighbour's nearest corner must stay outside R
    gap = centres - BODY_CM               # side to side, two robots squared up
    words = ("Each robot needs nothing within %.0f cm (%.1f in) of its middle: %s. Two robots: middles at least %.0f "
             "cm (%.1f in) apart, about %.0f cm (%.1f in) between their sides. That is more than a hand's width from "
             "the table's edge and from anything else." % (
                 R, R / 2.54, " + ".join("%s %.1f" % (w, v) for w, v, _ in terms), centres, centres / 2.54, gap,
                 gap / 2.54))
    return dict(radius_cm=round(R, 1), radius_in=round(R / 2.54, 1), centres_cm=round(centres, 1),
                centres_in=round(centres / 2.54, 1), gap_cm=round(gap, 1), terms=[(w, round(v, 2), y) for w, v, y in terms],
                stroke_cm=L, speed=speed, words=words)


# ---------------------------------------------------------------------------------------------- 2. every camera
def recorder_streams(ports=range(8000, 8008), host="127.0.0.1", timeout=0.3):
    """The recorder's streams that answer (camrec.py: camera i on port 8000 + i). [(name, "host:port")]."""
    out = []
    for p in ports:
        s = socket.socket()
        s.settimeout(timeout)
        try:
            s.connect((host, p))
            out.append(("stream-%d" % p, "%s:%d" % (host, p)))
        except OSError:
            pass
        finally:
            s.close()
    return out


def v4l_cameras():
    """/dev/video devices with their names (camrec's way: v4l2 names from sysfs). [(name, "/dev/videoN")]; only each
    USB camera's first node (the one that sends pictures)."""
    out = []
    for d in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            idx = int(open(os.path.join(d, "index")).read().strip())
        except (OSError, ValueError):
            idx = 0
        if idx != 0:
            continue
        try:
            nm = open(os.path.join(d, "name")).read().strip()
        except OSError:
            nm = os.path.basename(d)
        out.append((nm, "/dev/" + os.path.basename(d)))
    return out


def find_cameras():
    """Every camera this Pi can open: the recorder's streams (it holds the devices while it records), then devices
    it does not hold. [dict(name, src, kind)]."""
    cams = [dict(name=n, src=s, kind="stream") for n, s in recorder_streams()]
    held = len(cams)
    for n, dev in v4l_cameras():
        if held and cv2 is not None:
            cap = cv2.VideoCapture(dev)
            ok = cap.isOpened() and cap.read()[0]
            cap.release()
            if not ok:
                continue                # held by the recorder: its stream above is the same camera
        cams.append(dict(name=n, src=dev, kind="v4l"))
    return cams


class Grabber:
    """The newest picture from a stream ("host:port", the recorder's /live) or a /dev/video camera, kept by a thread;
    grab() waits (up to timeout) for a picture newer than the last one returned (grey, or None)."""

    def __init__(self, src, timeout=2.0):
        import threading
        self.src, self.timeout = src, timeout
        self.latest, self.n, self.taken, self.stop_ = None, 0, 0, False
        self.cond = threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def _put(self, g):
        with self.cond:
            self.latest, self.n = g, self.n + 1
            self.cond.notify_all()

    def _run(self):
        if str(self.src).startswith("/dev/"):
            cap = cv2.VideoCapture(self.src)
            while not self.stop_:
                ok, f = cap.read()
                if ok:
                    self._put(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
                else:
                    time.sleep(0.2)
            cap.release()
            return
        while not self.stop_:
            try:
                import sources as S
                st = S.StreamSource(self.src, log=lambda *_: None)
                st._connect()
                for _h, body, _arr in st.parts():
                    if self.stop_:
                        return
                    g = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_GRAYSCALE)
                    if g is not None:
                        self._put(g)
            except Exception:  # noqa: BLE001  (a camera that drops is tried again)
                time.sleep(1.0)

    def grab(self):
        with self.cond:
            end = time.time() + self.timeout
            while self.n <= self.taken and time.time() < end:
                self.cond.wait(max(0.01, end - time.time()))
            if self.n <= self.taken:
                return None                # no new picture: this camera is stalled
            self.taken = self.n
            return self.latest

    def close(self):
        self.stop_ = True


def leg_mover(robots):
    """The real robots' stroke: an idle leg `go +-cm` (calib_run.Robot.leg; the firmware's edge guard stays on)."""
    def mover(robot, cm, on_tick, n):
        rb = robots[robot]
        import threading
        done = threading.Event()
        res = {}

        def go():
            res["r"] = rb.leg("go", cm, timeout=20.0)
            done.set()
        threading.Thread(target=go, daemon=True).start()
        k = 0
        while not done.wait(abs(cm) / DISC_SPEED / max(1, n)):
            k += 1
            on_tick(k)
        on_tick(k + 1)
        return res.get("r")
    return mover


def leg_turner(robots, dps=60.0):
    """(e14) The real robots' wiggle on the spot: an idle leg `turn +-deg` (calib_run.Robot.leg; the calibration's turn
    speed, 60 deg/s), pictures taken as it turns."""
    def turner(robot, deg, on_tick, n):
        rb = robots[robot]
        import threading
        done = threading.Event()
        res = {}

        def go():
            res["r"] = rb.leg("turn", deg, timeout=20.0)
            done.set()
        threading.Thread(target=go, daemon=True).start()
        k = 0
        while not done.wait(abs(deg) / dps / max(1, n)):
            k += 1
            on_tick(k)
        on_tick(k + 1)
        return res.get("r")
    return turner


def run_real(site, folder, ips=(), confirm=None, log=print):
    """On the Pi: every camera found, every robot on the WiFi, the clear-space question, the strokes, coverage.json.
    confirm(words) -> True when the person says every robot has that clear space (default: asks in the terminal)."""
    import calib_run as CR
    from cameras import CameraModel
    tc = os.path.join(site.get("conf_dir") or os.path.expanduser("~/.config/mirlt"), "tagcal")
    model = CameraModel.load(os.path.join(tc, "overhead.json")) if os.path.exists(os.path.join(tc, "overhead.json")) \
        else None
    cpp = None
    if model is not None:
        tb = site.get("table") or {}
        cpp = 1.0 / float(model.px_per_cm(float(tb.get("length_cm", 200)) / 2, float(tb.get("width_cm", 100)) / 2, 4.0))
    cl = clearance(cpp)
    log("CLEAR SPACE: " + cl["words"])
    ok = confirm(cl["words"]) if confirm else input("Has every robot that much clear space round it? (y/n) ").strip(
        ).lower().startswith("y")
    if not ok:
        raise CR.RunEnd("the clear space was not confirmed: nothing moved")
    found = find_cameras()
    if not found:
        raise CR.RunEnd("no camera found on this Pi (no recorder stream, no /dev/video camera): nothing moved")
    typed = dict(site.get("camera_names") or {})
    grabs = [Grabber(c["src"]) for c in found]
    cams = [CamInfo(c["name"], g.grab, model if k == 0 else None) for k, (c, g) in enumerate(zip(found, grabs))]
    link = CR.Link(folder)
    try:
        bots = CR.discover(link, list(ips), 2.0, False)
        robots = {n: CR.Robot(link, n, ip) for n, ip in bots.items()}
        for rb in robots.values():
            ok_, words = CR.arm_safety(rb, 3, log=log)
            if not ok_:
                raise CR.RunEnd("%s: its safety layers did not read back (%s): nothing moved" % (rb.name, words))
        det = None
        try:
            import tagcore as TC
            det = TC.TagDetector(((site.get("tags") or {}).get("family")) or "tag25h9")
        except Exception:  # noqa: BLE001
            pass
        rec = discover(cams, sorted(robots), leg_mover(robots), tag_det=det, typed=typed, log=log)
        rec["clearance"] = cl
        save(rec, folder)
        return rec
    finally:
        link.close()
        for g in grabs:
            g.close()


# ---------------------------------------------------------------------------------------------- 3-5. the analysis
def _noise_sigma(a, b):
    d = a.astype(np.float32) - b.astype(np.float32)
    return float(1.4826 * np.median(np.abs(d - np.median(d)))) / math.sqrt(2.0)


def _mask(frame, bg, thr):
    d = cv2.absdiff(frame, bg)
    m = (d > thr).astype(np.uint8)
    k3 = np.ones((3, 3), np.uint8)
    k5 = np.ones((DETECT_CLOSE_PX, DETECT_CLOSE_PX), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k3)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k5)


def _largest(m, near_px=None):
    """The largest changed area, joined with the other sizeable ones within near_px of it (a plain roof changes only
    at its leading and trailing strips: two pieces of one robot)."""
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, connectivity=8)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    keep = [i]
    if near_px:
        for j in range(1, n):
            if j != i and st[j, cv2.CC_STAT_AREA] >= 0.2 * st[i, cv2.CC_STAT_AREA] and \
                    math.dist(cen[j], cen[i]) <= near_px:
                keep.append(j)
    mask = np.isin(lab, keep)
    ys, xs = np.nonzero(mask)
    return dict(area=int(sum(int(st[k, cv2.CC_STAT_AREA]) for k in keep)), c=(float(xs.mean()), float(ys.mean())),
                mask=mask)


class CamInfo:
    """One camera during discovery: its name, its model (or None), the still table's pictures."""

    def __init__(self, name, grab, model=None, typed_role=None):
        self.name, self.grab, self.model, self.typed_role = name, grab, model, typed_role
        self.bg = None
        self.thr = MIN_T
        self.quiet_area = 0
        self.light = None
        self.frames = 0


def _robot_px(cam, xy=None):
    if cam.model is not None and xy is not None:
        try:
            return float(cam.model.px_per_cm(xy[0], xy[1], 4.0)) * BODY_CM
        except Exception:     # noqa: BLE001
            pass
    return DEFAULT_ROBOT_PX


def _robot_px_at(cam, blobs, default):
    """(e14) The robot's size in pixels where the largest change is (through the camera's model): a far robot in a
    low camera is a third of DEFAULT_ROBOT_PX, and the noise floor's "need" must shrink with it."""
    if cam.model is None or not blobs:
        return default
    b = max(blobs, key=lambda q: q["area"])
    xy = _table_pt(cam, b["c"])
    return _robot_px(cam, xy) if xy is not None else default


def still(cams, n=BG_FRAMES):
    """Every camera's still table: the median of n pictures, the threshold from its noise (motion.py's rule), and the
    largest change between still pictures (the quiet level a robot's stroke must stand out from)."""
    for c in cams:
        fr = []
        for _ in range(n):
            g = c.grab()
            if g is not None:
                fr.append(g)
        if not fr:
            c.bg = None
            continue
        c.bg = np.median(np.stack(fr), axis=0).astype(np.uint8)
        sig = _noise_sigma(fr[0], fr[-1]) if len(fr) > 1 else 3.0
        c.thr = max(MIN_T, NOISE_K * sig)
        c.quiet_area = max([int(_mask(f, c.bg, c.thr).sum()) for f in fr] + [0])
        c.light = round(float(c.bg.mean()), 1)


def stroke_record(cams, mover, robot, stroke_cm=None, frames_per_leg=6):
    """Robot `robot` strokes forward and back; every camera's pictures are kept: {cam: [(phase, k, grey)]}."""
    L = nudge_cm() if stroke_cm is None else float(stroke_cm)
    rec = {c.name: [] for c in cams}

    def snap(phase, k):
        for c in cams:
            g = c.grab()
            if g is not None:
                rec[c.name].append((phase, k, g))
                c.frames += 1
    snap("pre", 0)
    r1 = mover(robot, +L, lambda k: snap("fwd", k), frames_per_leg)
    snap("fwd_end", 0)
    r2 = mover(robot, -L, lambda k: snap("back", k), frames_per_leg)
    snap("end", 0)
    rec["_moves"] = [r1, r2]          # (e14) what the robot said about its stroke (a refusal is said, not guessed)
    return rec


def wiggle_record(cams, turner, robot, deg=WIGGLE_DEG, frames_per_leg=6):
    """(e14) Robot `robot` turns +deg on the spot and back (where a stroke is not safe: it needs only its own
    footprint); every camera's pictures kept as stroke_record's ("pre", "fwd" while turning out, "fwd_end", "back",
    "end")."""
    rec = {c.name: [] for c in cams}

    def snap(phase, k):
        for c in cams:
            g = c.grab()
            if g is not None:
                rec[c.name].append((phase, k, g))
                c.frames += 1
    snap("pre", 0)
    r1 = turner(robot, +deg, lambda k: snap("fwd", k), frames_per_leg)
    snap("fwd_end", 0)
    r2 = turner(robot, -deg, lambda k: snap("back", k), frames_per_leg)
    snap("end", 0)
    rec["_moves"] = [r1, r2]
    return rec


def analyse_wiggle(cam, frames):
    """(e14) One camera, one robot's wiggle on the spot -> where it stands (the middle of the area that changed while
    it turned: a square roof turning on its middle changes symmetrically round it). No heading (a square looks the
    same every quarter turn); seen False when nothing stood out."""
    if cam.bg is None or not frames:
        return dict(seen=False, why="no pictures")
    turn = [g for ph, _k, g in frames if ph in ("fwd", "fwd_end")]
    if not turn:
        return dict(seen=False, why="no pictures during the wiggle")
    rpx = _robot_px(cam)
    blobs = [_largest(_mask(g, cam.bg, cam.thr), near_px=1.6 * rpx) for g in turn]
    blobs = [b for b in blobs if b is not None]
    rpx = _robot_px_at(cam, blobs, rpx)
    need = max(MIN_AREA_FRAC * rpx * rpx * 0.5, SEEN_K * max(1, cam.quiet_area))
    best = max([b["area"] for b in blobs] + [0])
    contrast = round(best / max(1.0, float(cam.quiet_area)), 1)
    if not blobs or best < need:
        return dict(seen=False, why="no change stood out (largest %d px, needed %d)" % (best, need), contrast=contrast)
    b = max(blobs, key=lambda q: q["area"])
    ys, xs = np.nonzero(b["mask"])
    mid = (0.5 * (float(np.percentile(xs, 98)) + float(np.percentile(xs, 2))),
           0.5 * (float(np.percentile(ys, 98)) + float(np.percentile(ys, 2))))
    body_px = float(max(np.percentile(xs, 98) - np.percentile(xs, 2), np.percentile(ys, 98) - np.percentile(ys, 2)))
    out = dict(seen=True, how="wiggle", contrast=contrast, where_px=[round(v, 1) for v in mid],
               body_px=round(body_px, 1), stroke_px=0.0, heading_img_deg=None, frames=len(frames))
    xy = _table_pt(cam, mid)
    if xy is not None:
        out["where_cm"] = [round(xy[0], 1), round(xy[1], 1)]
        try:
            out["px_per_cm_model"] = round(float(cam.model.px_per_cm(xy[0], xy[1], 4.0)), 2)
        except Exception:     # noqa: BLE001
            pass
    g = turn[-1]
    hx, hy = int(round(mid[0])), int(round(mid[1]))
    w = int(max(10, 1.5 * body_px))
    roi = g[max(0, hy - w):hy + w, max(0, hx - w):hx + w]
    out["light_near"] = round(float(roi.mean()), 1) if roi.size else None
    return out


def _match_shift(a, b, uv, body_px, stroke_px):
    """Where the patch of picture a round uv (the robot's body) went in picture b: (du, dv) or None."""
    h = int(max(4, 0.75 * body_px))       # the whole body and its outline (a plain roof has no texture of its own)
    u, v = int(round(uv[0])), int(round(uv[1]))
    if v - h < 0 or u - h < 0 or v + h >= a.shape[0] or u + h >= a.shape[1]:
        return None
    tpl = a[v - h:v + h + 1, u - h:u + h + 1]
    r = int(max(stroke_px * 1.6, 0.6 * body_px)) + 2
    y0, x0 = max(0, v - h - r), max(0, u - h - r)
    win = b[y0:min(b.shape[0], v + h + r + 1), x0:min(b.shape[1], u + h + r + 1)]
    if win.shape[0] < tpl.shape[0] or win.shape[1] < tpl.shape[1] or float(tpl.std()) < 2.0:
        return None
    res = cv2.matchTemplate(win, tpl, cv2.TM_CCOEFF_NORMED)
    _mn, mx, _l, loc = cv2.minMaxLoc(res)
    if mx < 0.5:
        return None
    px, py = float(loc[0]), float(loc[1])
    if 0 < loc[0] < res.shape[1] - 1:                  # sub-pixel: a parabola through the peak and its neighbours
        l_, c_, r_ = res[loc[1], loc[0] - 1], res[loc[1], loc[0]], res[loc[1], loc[0] + 1]
        den = l_ - 2 * c_ + r_
        if abs(den) > 1e-9:
            px += 0.5 * (l_ - r_) / den
    if 0 < loc[1] < res.shape[0] - 1:
        l_, c_, r_ = res[loc[1] - 1, loc[0]], res[loc[1], loc[0]], res[loc[1] + 1, loc[0]]
        den = l_ - 2 * c_ + r_
        if abs(den) > 1e-9:
            py += 0.5 * (l_ - r_) / den
    return (x0 + px - (u - h), y0 + py - (v - h))


def _tags_in(grey, det):
    if det is None:
        return {}
    try:
        return {d.id: d for d in det.detect(grey)}
    except Exception:     # noqa: BLE001
        return {}


def _img_heading(p0, p1):
    return math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))


def _table_pt(cam, uv, z=4.0):
    if cam.model is None:
        return None
    try:
        x, y = cam.model.to_floor(float(uv[0]), float(uv[1]), z)
        return (x, y) if np.isfinite(x) and np.isfinite(y) else None
    except Exception:     # noqa: BLE001
        return None


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def analyse(cam, frames, stroke_cm, tag_det=None, tag_z=6.0):
    """One camera, one robot's stroke -> what this camera saw of it (seen False when nothing stood out)."""
    if cam.bg is None or not frames:
        return dict(seen=False, why="no pictures")
    fwd = [g for ph, _k, g in frames if ph in ("fwd", "fwd_end")]
    pre = [g for ph, _k, g in frames if ph == "pre"]
    if not fwd:
        return dict(seen=False, why="no pictures during the stroke")
    rpx = _robot_px(cam)
    blobs = [_largest(_mask(g, cam.bg, cam.thr), near_px=1.6 * rpx) for g in fwd]
    blobs = [b for b in blobs if b is not None]
    rpx = _robot_px_at(cam, blobs, rpx)
    need = max(MIN_AREA_FRAC * rpx * rpx, SEEN_K * max(1, cam.quiet_area))
    best = max([b["area"] for b in blobs] + [0])
    contrast = round(best / max(1.0, float(cam.quiet_area)), 1)
    if not blobs or best < need:
        return dict(seen=False, why="no change stood out (largest %d px, needed %d)" % (best, need),
                    contrast=contrast)
    big = [b for b in blobs if b["area"] >= need]
    c0, c1 = big[0]["c"], big[-1]["c"]
    end = big[-1]
    ys, xs = np.nonzero(end["mask"])
    h_img = _img_heading(c0, c1) if math.dist(c0, c1) > 0.5 else None
    # the changed area at the forward end covers the body and the stroke: across the stroke it is the body's width
    if h_img is not None:
        a = math.radians(h_img)
        along = xs * math.cos(a) + ys * math.sin(a)
        across = -xs * math.sin(a) + ys * math.cos(a)
        body_px = float(np.percentile(across, 98) - np.percentile(across, 2))
        span_px = float(np.percentile(along, 98) - np.percentile(along, 2))
        stroke_px = max(0.0, span_px - body_px)
    else:
        body_px = float(math.sqrt(end["area"]))
        stroke_px = 0.0
    # the robot's middle at the start: the changed area's middle (half-way between its far ends along and across the
    # stroke: a plain roof changes only at its leading and trailing strips) less half the stroke along its direction
    if h_img is not None:
        ma = 0.5 * (np.percentile(along, 98) + np.percentile(along, 2))
        mc = 0.5 * (np.percentile(across, 98) + np.percentile(across, 2))
        mid = (float(ma * math.cos(a) - mc * math.sin(a)), float(ma * math.sin(a) + mc * math.cos(a)))
    else:
        mid = (float(xs.mean()), float(ys.mean()))
    start_uv = (mid[0] - 0.5 * stroke_px * math.cos(math.radians(h_img)), mid[1] - 0.5 * stroke_px *
                math.sin(math.radians(h_img))) if h_img is not None else mid
    # refined: the robot's own picture before the stroke found again in the picture at its forward end (template
    # matching): the displacement is the stroke itself, to a fraction of a pixel, not the changed area's drift
    refined = False
    if pre and h_img is not None:
        disp = _match_shift(pre[0], fwd[-1], start_uv, body_px, stroke_px)
        if disp is not None and math.hypot(*disp) > 0.5 * max(1.0, stroke_px) and \
                abs(_wrap(math.degrees(math.atan2(disp[1], disp[0])) - h_img)) < 60:
            h_img = math.degrees(math.atan2(disp[1], disp[0]))
            stroke_px = float(math.hypot(*disp))
            refined = True
            c0 = start_uv
            c1 = (start_uv[0] + disp[0], start_uv[1] + disp[1])
    out = dict(seen=True, contrast=contrast, heading_sure=refined,       # (e14) the stroke's own picture found again
               where_px=[round(v, 1) for v in start_uv], body_px=round(body_px, 1),
               stroke_px=round(stroke_px, 1), heading_img_deg=None if h_img is None else round(h_img, 1),
               frames=len(frames))
    xy = _table_pt(cam, start_uv)
    if xy is not None:
        out["where_cm"] = [round(xy[0], 1), round(xy[1], 1)]
        p0, p1 = _table_pt(cam, c0), _table_pt(cam, c1)
        if p0 and p1 and math.dist(p0, p1) > 0.3:
            out["heading_deg"] = round(_img_heading(p0, p1), 1)
            out["end_cm"] = [round(p1[0], 1), round(p1[1], 1)]      # (e14) where the stroke's forward leg ended
        try:
            out["px_per_cm_model"] = round(float(cam.model.px_per_cm(xy[0], xy[1], 4.0)), 2)
        except Exception:     # noqa: BLE001
            pass
    if stroke_px > 0 and stroke_cm:
        out["px_per_cm_stroke"] = round(stroke_px / stroke_cm, 2)
        out["px_per_cm_body"] = round(body_px / BODY_CM, 2)
        r = out["px_per_cm_stroke"] / max(1e-6, out["px_per_cm_body"])
        out["tilt_hint_deg"] = round(math.degrees(math.acos(min(1.0, r))), 0) if r < 1.0 else 0.0
    g = fwd[-1]
    hx, hy = int(round(start_uv[0])), int(round(start_uv[1]))
    w = int(max(10, 1.5 * body_px))
    roi = g[max(0, hy - w):hy + w, max(0, hx - w):hx + w]
    out["light_near"] = round(float(roi.mean()), 1) if roi.size else None
    # roof tags: the tag that moved with the stroke is this robot's; its heading against the motion's = its turn
    if tag_det is not None and pre:
        t0, t1 = _tags_in(pre[0], tag_det), _tags_in(fwd[-1], tag_det)
        best_t = None
        for tid, d1 in t1.items():
            d0 = t0.get(tid)
            if d0 is None:
                continue
            mv = np.asarray(d1.center) - np.asarray(d0.center)
            if np.hypot(*mv) < max(1.0, 0.4 * stroke_px) or h_img is None:
                continue
            if abs(_wrap(math.degrees(math.atan2(mv[1], mv[0])) - h_img)) > 45:
                continue
            best_t = (tid, d0)
        if best_t is not None:
            tid, d0 = best_t
            out["tag_id"] = int(tid)
            cc = np.asarray(d0.corners)
            top = 0.5 * (cc[0] + cc[1])
            ctr = np.asarray(d0.center)
            th_img = _img_heading(ctr, top)
            # the printed top of the tag (corner 0-1 edge) against the nose (the motion): its turn on the robot
            if cam.model is not None and out.get("heading_deg") is not None:
                a, b = _table_pt(cam, ctr, tag_z), _table_pt(cam, top, tag_z)
                if a and b:
                    out["tag_heading_deg"] = round(_img_heading(a, b), 1)
                    out["tag_turn_deg"] = round(_wrap(out["tag_heading_deg"] - out["heading_deg"]), 1)
            else:
                out["tag_turn_img_deg"] = round(_wrap(th_img - h_img), 1)
            out["tag_px"] = round(float(d0.size_px), 1)
    return out


def camera_tilt(model):
    if model is None:
        return None
    try:
        return round(float(model.pose_summary()["tilt_from_vertical_deg"]), 1)
    except Exception:     # noqa: BLE001
        return None


def discover(cams, robots, mover, stroke_cm=None, tag_det=None, typed=None, log=print, frames_per_leg=6, modes=None,
             turner=None, on_moved=None):
    """The whole discovery: still pictures, each robot's stroke, the analysis, the coverage map. typed: {role: name}
    (the site's camera names: a first guess, compared). (e14) modes: {robot: "stroke" | "wiggle"} (default stroke; a
    wiggle needs turner(robot, deg, on_tick, n)); on_moved(robot, mode, t0, t1): called after each robot's move (the
    run's own tag reads decide which roof tag moved with it). Returns the record (coverage.json's content)."""
    L = nudge_cm() if stroke_cm is None else float(stroke_cm)
    rec = dict(version=VERSION, time=time.strftime("%Y-%m-%dT%H:%M:%S"), stroke_cm=L, speed_cms=DISC_SPEED,
               cameras={}, robots={}, blind=[], notes=[], modes={})
    per = {}
    for rb in robots:
        still(cams)                       # the table as it is now (the robot before moved back)
        mode = (modes or {}).get(rb, "stroke")
        if mode == "wiggle" and turner is None:
            mode = "stroke"
        rec["modes"][rb] = mode
        t0 = time.time()
        if mode == "wiggle":
            fr = wiggle_record(cams, turner, rb, WIGGLE_DEG, frames_per_leg)
            per[rb] = {c.name: analyse_wiggle(c, fr[c.name]) for c in cams}
        else:
            fr = stroke_record(cams, mover, rb, L, frames_per_leg)
            per[rb] = {c.name: analyse(c, fr[c.name], L, tag_det) for c in cams}
        refused = [m for m in (fr.get("_moves") or []) if isinstance(m, dict) and m.get("result") not in (None, "done")]
        if refused:
            rec.setdefault("refused", {})[rb] = "%s %s" % (refused[0].get("result"), refused[0].get("info") or "")
        if on_moved is not None:
            t_ = on_moved(rb, mode, t0, time.time())
            if t_ is not None:
                rec.setdefault("tag_moved", {})[rb] = int(t_)     # the run's own tag reads moved with it
    for c in cams:
        sees = [rb for rb in robots if per[rb][c.name].get("seen")]
        rec["cameras"][c.name] = dict(sees=sees, typed_role=c.typed_role, model=c.model is not None,
                                      no_pictures=c.frames == 0 and c.bg is None,
                                      tilt_deg=camera_tilt(c.model), light=c.light, frames=c.frames,
                                      threshold=round(float(c.thr), 1),
                                      where_px={rb: per[rb][c.name]["where_px"] for rb in sees},
                                      where_cm={rb: per[rb][c.name].get("where_cm") for rb in sees})
        if not sees:
            rec["blind"].append(c.name)
    for rb in robots:
        seen_by = [c.name for c in cams if per[rb][c.name].get("seen")]
        # (e14) the camera with the robot biggest in its picture AND a model (a position in cm) first
        best = max(seen_by, key=lambda n: (per[rb][n].get("where_cm") is not None, per[rb][n].get("body_px", 0))) \
            if seen_by else None
        b = per[rb][best] if best else {}
        rec["robots"][rb] = dict(cameras=seen_by, best_camera=best, heading_deg=b.get("heading_deg"),
                                 how=rec["modes"].get(rb, "stroke"), end_cm=b.get("end_cm"),
                                 heading_img_deg=b.get("heading_img_deg"), where_cm=b.get("where_cm"),
                                 tag_id=(rec.get("tag_moved") or {}).get(rb, next(
                                     (per[rb][n]["tag_id"] for n in seen_by if "tag_id" in per[rb][n]), None)),
                                 tag_turn_deg=next((per[rb][n]["tag_turn_deg"] for n in seen_by
                                                    if "tag_turn_deg" in per[rb][n]), None),
                                 per_camera=per[rb])
        if not seen_by and (rec.get("refused") or {}).get(rb):
            rec["notes"].append("%s: it did not make its move (the robot said: %s)" % (rb, rec["refused"][rb]))
        elif not seen_by:
            rec["notes"].append("%s: no camera saw it move (switched off, not on the table, or out of every view)" % rb)
    # two robots too close for the motion detector: a mover's changed area reaching a neighbour's start
    for c in cams:
        pts = {rb: per[rb][c.name] for rb in robots if per[rb][c.name].get("seen")}
        for a, pa in pts.items():
            for b_, pb in pts.items():
                if a >= b_:
                    continue
                d = math.dist(pa["where_px"], pb["where_px"])
                lim = 0.5 * (pa["body_px"] + pb["body_px"]) + pa.get("stroke_px", 0) + DETECT_CLOSE_PX + DETECT_PAD_PX
                if d < lim:
                    rec["notes"].append("%s and %s stand %.0f px apart in camera %s (the detector needs %.0f): their "
                                        "blobs may join; each was named by its own stroke (only one moved at a time)"
                                        % (a, b_, d, c.name, lim))
    # typed names: a first guess
    if typed:
        names = {c.name for c in cams}
        for role, nm in typed.items():
            if not nm:
                continue
            hit = [c.name for c in cams if nm.lower() in c.name.lower()]
            if not hit:
                rec["notes"].append("typed %s camera '%s' was not found (cameras found: %s)" % (
                    role, nm, ", ".join(sorted(names)) or "none"))
            elif all(h in rec["blind"] for h in hit):
                rec["notes"].append("typed %s camera '%s' is plugged in but saw no robot" % (role, nm))
        roles = {}
        live = [c for c in cams if c.name not in rec["blind"]]
        if live:
            best = max(live, key=lambda c: (len(rec["cameras"][c.name]["sees"]),
                                            np.mean([per[r][c.name].get("body_px", 0) for r in rec["cameras"][c.name]
                                                     ["sees"]] or [0])))
            roles["overhead"] = best.name
            rest = [c.name for c in live if c is not best]
            if rest:
                roles["side"] = rest[0]
        rec["roles_found"] = roles
        rec["roles_typed"] = dict(typed)
    rec["names_rule"] = os.environ.get("MIRLT_CAMERA_NAMES", "discovered")
    for line in words(rec):
        log(line)
    return rec


def words(rec):
    out = ["CAMERA DISCOVERY (each robot a %.0f cm stroke forward and back, one at a time):" % rec["stroke_cm"]]
    for n, c in rec["cameras"].items():
        if c.get("no_pictures"):
            import camera_feed as CF
            out.append("  " + CF.words_for(n) + " Left out of this run.")
            continue
        out.append("  camera %s: %s%s" % (n, ("sees " + ", ".join(c["sees"])) if c["sees"] else
                                          "SEES NO ROBOT: left out of this run",
                                          ("; tilt %.0f deg" % c["tilt_deg"]) if c.get("tilt_deg") is not None else ""))
    for n, r in rec["robots"].items():
        out.append("  %s: seen by %s%s%s%s" % (
            n, ", ".join(r["cameras"]) or "NO camera",
            ("; starts facing %.0f deg" % r["heading_deg"]) if r.get("heading_deg") is not None else "",
            ("; roof tag %d" % r["tag_id"]) if r.get("tag_id") is not None else "; no roof tag moved with it",
            ("; tag turned %+.0f deg on it" % r["tag_turn_deg"]) if r.get("tag_turn_deg") is not None else ""))
    for x in rec.get("notes") or []:
        out.append("  ! " + x)
    if rec.get("roles_found"):
        out.append("  roles found: %s (typed: %s; %s)" % (
            ", ".join("%s = %s" % kv for kv in rec["roles_found"].items()),
            ", ".join("%s = %s" % kv for kv in (rec.get("roles_typed") or {}).items()) or "none",
            "the TYPED names are used (MIRLT_CAMERA_NAMES=typed)" if rec.get("names_rule") == "typed" else
            "the found ones are used"))
    return out


def save(rec, folder):
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "coverage.json"), "w") as f:
        json.dump(rec, f, indent=1, default=str)
    with open(os.path.join(folder, "coverage.md"), "w") as f:
        f.write("\n".join(words(rec)) + "\n")
    return os.path.join(folder, "coverage.json")


# ---------------------------------------------------------------------------------------------- the simulator rig
class SimRig:
    """Simulated cameras (tools/sim/tag_cam.TagCamera, the real drawing and blur) over a table with robots, a mover that
    drives them, and the guesses broken on purpose: an extra camera looking at the wall (sees nothing), a camera whose
    name differs from the typed one, a typed camera that is not plugged in, two robots close together, robots with or
    without roof tags (a plain white roof)."""

    def __init__(self, table=(396.0, 168.0), robots=None, tags=True, extra_blind=True, renamed=True, seed=3,
                 size=(640, 480)):
        import tag_cam as TCAM
        L, W = table
        self.table = (0.0, 0.0, L, W)
        self.robots = robots or {"Alpha": [120.0, 80.0, 30.0], "Bravo": [141.0, 84.0, 200.0],
                                 "Charlie": [300.0, 60.0, -90.0]}
        self.tags = tags
        cams = [TCAM.TagCamera((L * 0.3, W / 2, 200.0), (L * 0.3 + 0.01, W / 2, 0.0), size=size, hfov_deg=74,
                               name="USB 2.0 Camera" if renamed else "C920e", table=self.table, seed=seed),
                TCAM.TagCamera((L * 0.78, W / 2, 200.0), (L * 0.78 + 0.01, W / 2 + 3, 0.0), size=size, hfov_deg=60,
                               name="EMEET C950", table=self.table, seed=seed + 1, roll_deg=8)]
        if extra_blind:
            cams.append(TCAM.TagCamera((L / 2, -80.0, 60.0), (L / 2, -400.0, 80.0), size=size, hfov_deg=60,
                                       name="laptop webcam", table=self.table, seed=seed + 2))
        self.cams = cams
        self.t = 0.0

    def draw(self):
        out = []
        for k, (n, p) in enumerate(sorted(self.robots.items())):
            out.append(dict(x=p[0], y=p[1], th=math.radians(p[2]), z=6.0, family="tag25h9", id=k, tag_cm=8.0,
                            roof_cm=9.5))
        return out

    def grabber(self, cam):
        def grab():
            cov = () if self.tags else tuple(range(len(self.robots)))
            return cam.render(self.draw(), t=self.t, covered=cov)
        return grab

    def mover(self, robot, cm, on_tick, n):
        p = self.robots[robot]
        h = math.radians(p[2])
        for k in range(1, n + 1):
            p[0] += cm / n * math.cos(h)
            p[1] += cm / n * math.sin(h)
            self.t += abs(cm) / n / DISC_SPEED
            on_tick(k)

    def turner(self, robot, deg, on_tick, n):
        """(e14) The wiggle on the spot."""
        p = self.robots[robot]
        for k in range(1, n + 1):
            p[2] += deg / n
            self.t += abs(math.radians(deg / n)) * 4.5 / DISC_SPEED
            on_tick(k)

    def caminfos(self, models=True, typed_roles=None):
        tr = typed_roles or {}
        return [CamInfo(c.name, self.grabber(c), c.true_model() if (models and c.name != "laptop webcam") else None,
                        next((r for r, nm in tr.items() if nm == c.name), None)) for c in self.cams]

    def tag_detector(self):
        if not self.tags:
            return None
        import tagcore as TC
        return TC.TagDetector("tag25h9")


def run_sim(out=None, tags=True, log=print):
    rig = SimRig(tags=tags)
    typed = {"overhead": "C920e", "side": "Logitech BRIO"}     # one renamed, one missing
    cams = rig.caminfos(typed_roles=typed)
    truth = {n: list(p) for n, p in rig.robots.items()}
    rec = discover(cams, sorted(rig.robots), rig.mover, tag_det=rig.tag_detector(), typed=typed, log=log)
    rec["sim_truth"] = truth
    if out:
        save(rec, out)
    return rec


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd")
    c = sp.add_parser("clearance")
    c.add_argument("--cm-per-px", type=float, default=None)
    sp.add_parser("list")
    r = sp.add_parser("run", help="on the Pi, with the robots: every camera, every robot's stroke, coverage.json")
    r.add_argument("--out", default="")
    r.add_argument("--robot", action="append", default=[])
    s = sp.add_parser("sim")
    s.add_argument("--out", default="")
    s.add_argument("--no-tags", action="store_true")
    a = ap.parse_args(argv)
    if a.cmd == "clearance":
        d = clearance(a.cm_per_px)
        print(d["words"])
        for w, v, y in d["terms"]:
            print("  %-40s %5.2f cm  (%s)" % (w, v, y))
        return 0
    if a.cmd == "list":
        for cm in find_cameras():
            print("%-30s %-24s %s" % (cm["name"], cm["src"], cm["kind"]))
        return 0
    if a.cmd == "run":
        import sites as SI
        out = a.out or os.path.join(os.path.dirname(TOOLS), "logs", time.strftime("%Y%m%d-%H%M%S") + "-cameras")
        run_real(SI.load(), out, ips=a.robot)
        print("coverage: %s" % os.path.join(out, "coverage.json"))
        return 0
    if a.cmd == "sim":
        run_sim(a.out or None, tags=not a.no_tags)
        return 0
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
