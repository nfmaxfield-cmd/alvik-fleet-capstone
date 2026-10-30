#!/usr/bin/env python3
"""The facing check (Nick, 2026-10-29): which way each robot faces, from its lights, before anything moves.

Why: on 28 Oct (19:17 DOE loop) Center's heading was 190 deg wrong and it drove the wrong way three times; the camera
checked each robot's SPOT, never which way it faced. Monday's start (and Mission Control's) took every heading as placed.

The robot's own two lights (Alvik left_led and right_led, on its top): measured on 28 Oct's find-step pictures, they sit
ACROSS the robot, one on its left and one on its right, 6.5-7.5 cm apart and about level with its middle (0.4-1.5 cm
ahead of it), NOT one at the front and one at the back. The line through them is square to the heading, so:

    heading = the direction from the LEFT light to the RIGHT light, turned 90 deg clockwise

and which light is the left one says which end is the front. Firmware 2026-10-29a lights one side alone
(`led 1 1 1 <ms> L` / `... R`); older firmware lights both, and then only the robot's idle state can name the left one
(after a `led` ends, its LEFT light shows the selected experiment's colour and its right light is dark; but a robot
that has run `follow` since power-on shows nothing, and then front and back cannot be told apart).

What it does, all robots dark first (`led 0 0 0`), then one robot at a time (nothing moves; 6-8 s for two robots):
  1. its LEFT light alone, pictures from both cameras;  2. its RIGHT light alone (old firmware: its idle state);
  3. in each camera, the spot that came on in 1 and in 2 (against the dark pictures), on the floor at the lights' height;
  4. per camera a heading from left to right light; the cameras together (side camera weighted more: it measured
     within 0.4-3.7 deg of the drives' camera lines on 28 Oct, the washed-out overhead within about 10);
  5. against the heading the run assumes: more than TOL_DEG off -> nothing moves, in plain words which way it faces.

Used by run_plan.py (--facing-check: Monday's start, Mission Control's Run and Setup, through mission_run.py) and by
the ladder's find step (L0 and every DOE loop: ladder.Eye.find_lights).

    python3 tools/fleet/facing.py --selftest          # the measurement on drawn pictures (no cameras, no robots)
"""
import math
import os
import sys
import threading
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, TOOLS, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np    # noqa: E402

TOL_DEG = 15.0          # chosen from the simulator (Monday's mission, 61 x 122, both cameras): robots put down 9, 12 and 15
                        # deg off ran the whole mission; 18 deg (Left) and 28 deg (Right) stopped it after 1 s ("camera
                        # and Left's own pose disagree by 7.9 cm"). Over it, nothing moves. Measured-vs-true error: side
                        # camera 0.4-3.7 deg (28 Oct, night), the simulator 1-3. DOE note: Thursday's board, starts
                        # placed 0 / 10 / 15 / 20 deg off: first-leg stops and line-check p95 against the angle, and the
                        # false-stop rate of well-placed robots (the measure's error). Starting from the MEASURED heading
                        # instead would need run_plan to plan again, and Monday's timed mission refuses that (planner:
                        # "no priority order gave a plan", simulator 18 deg)
SIDEWAYS_DEG = 45.0     # words only: over this it is "sideways", over 135 "the wrong way". No DOE: definitions
SETTLE_S = 0.35         # a command's answer -> the light seen in pictures (camera exposure, the stream's delay). DOE
                        # note: from the stills round each command, the time until the spot is full (25 a second)
COLLECT_S = 0.45        # pictures averaged (median) per state: ~11 overhead, ~13 side at the Pi's rates. DOE note:
                        # 0.3 / 0.45 / 0.6 s against the heading's spread between repeats
SPOT_THR = 30.0         # grey levels a light must rise over the dark pictures (28 Oct: 40-90 overhead, 60-160 side,
                        # board noise under 10). DOE note: 20 / 30 / 40 against spots missed vs false spots (hands, glare)
PAIR_CM = (4.0, 9.5)    # the two lights' distance on the floor (measured 6.5-6.6 side, 6.7-7.5 overhead, 28 Oct). No
                        # DOE: a sanity window round the measured spacing (another robot's light is 14+ cm away)
NEAR_CM = 15.0          # a light this far from the other light's camera spot or the expected spot is not this robot's.
                        # No DOE: a robot's size and a half
CAM_SIGMA = {"side": 4.0, "overhead": 10.0}   # deg, each camera's weight in the mean (28 Oct find steps against the
                        # drives' camera lines). DOE note: refit from Thursday's starts
AGREE_DEG = 30.0        # the two cameras further apart than this: the measure is not sure. No DOE: under TOL_DEG
IDLE_MIN = 0.25         # old firmware: the idle light's spot must be at least this share of the same light's white spot
                        # (28 Oct: 0.56 overhead, 0.75 side for a real one; under 0.1 for noise). DOE note: from the
                        # find steps of a session on old firmware, real idle lights against robots selected `follow`
LED_Z_CM = 3.7          # the lights' height = the robot's top (e02_camera.ROBOT_TOP_CM)
MEASURE_RETRIES = 2     # a robot whose lights were not read is measured again this many times. No DOE: retries
REPEAT_DARK_MS = 15000  # every robot's lights held dark this long while the others are measured (renewed per robot)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------------------------------------- the measurement
def spots(dark, lit, cam, rect=None, margin_cm=10.0, thr=SPOT_THR, z=LED_Z_CM):
    """Spots that came on between two sets of pictures (lists of grey images, medians), on the floor at height z:
    [(strength, x, y, u, v)], strongest first. rect: the board (x0, y0, x1, y1); spots further out are ignored."""
    import cv2
    b = np.median(np.stack([f.astype(np.float32) for f in dark]), axis=0)
    a = np.median(np.stack([f.astype(np.float32) for f in lit]), axis=0)
    d = a - b
    d -= float(np.median(d[::4, ::4]))
    m = (d > thr).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    out = []
    for i in range(1, n):
        area = int(st[i, cv2.CC_STAT_AREA])
        if area < 2 or area > 400:
            continue
        x0, y0, w, h = st[i, 0], st[i, 1], st[i, 2], st[i, 3]
        sub = lab[y0:y0 + h, x0:x0 + w] == i
        ww = d[y0:y0 + h, x0:x0 + w] * sub
        vv, uu = np.nonzero(sub)
        wv = ww[vv, uu]
        s = float(wv.sum())
        u, v = float((uu * wv).sum() / s) + x0, float((vv * wv).sum() / s) + y0
        x, y = cam.to_floor(u, v, z)
        if not (np.isfinite(x) and np.isfinite(y)):
            continue
        if rect is not None and not (min(rect[0], rect[2]) - margin_cm <= x <= max(rect[0], rect[2]) + margin_cm and
                                     min(rect[1], rect[3]) - margin_cm <= y <= max(rect[1], rect[3]) + margin_cm):
            continue
        out.append((s, float(x), float(y), u, v))
    out.sort(key=lambda q: -q[0])
    return out


def one_camera(cam, dark, first, second, both_firmware, rect=None, near=None, exclude=()):
    """One camera's measure for one robot. first: pictures with its LEFT light alone (old firmware: both lights);
    second: its RIGHT light alone (old firmware: its idle state, the left light in its experiment's colour or nothing).
    near: (x, y) where it should be, or None (anywhere on the board). Returns dict(ok, heading, left, right, sign, why)."""
    if len(dark) < 2 or len(first) < 2 or len(second) < 2:
        return dict(ok=False, why="too few pictures (%d dark, %d, %d)" % (len(dark), len(first), len(second)))
    s1 = spots(dark, first, cam, rect)
    s2 = spots(dark, second, cam, rect)
    if near is not None:
        s1 = [q for q in s1 if math.hypot(q[1] - near[0], q[2] - near[1]) <= NEAR_CM]
        s2 = [q for q in s2 if math.hypot(q[1] - near[0], q[2] - near[1]) <= NEAR_CM]
    for e in exclude:                                     # another robot's lights (measured before this one)
        s1 = [q for q in s1 if math.hypot(q[1] - e[0], q[2] - e[1]) > 7.0]
        s2 = [q for q in s2 if math.hypot(q[1] - e[0], q[2] - e[1]) > 7.0]
    if not s1:
        return dict(ok=False, why="its lights did not show")
    if not both_firmware:
        # left alone, then right alone: one spot each (a second, much weaker spot is glare: kept out by strength)
        if not s2:
            return dict(ok=False, why="its right light did not show")
        L, R = s1[0], s2[0]
        if math.hypot(L[1] - R[1], L[2] - R[2]) < PAIR_CM[0] and len(s1) >= 2 and s1[1][0] > 0.4 * s1[0][0]:
            # the "left alone" picture shows two lights: the firmware lit both (older than 2026-10-29a)
            return one_camera(cam, dark, first, second, True, rect, near, exclude)
        sign = "left/right lights"
    else:
        if len(s1) < 2:
            return dict(ok=False, why="only one of its two lights showed")
        A, B = s1[0], s1[1]
        if not s2:
            return dict(ok=True, sign=None, heading=None, pair=_pair(A, B),
                        why="front and back not told apart (older firmware, and its idle light is off)")
        ea = sum(q[0] for q in s2 if math.hypot(q[1] - A[1], q[2] - A[2]) < 2.5)
        eb = sum(q[0] for q in s2 if math.hypot(q[1] - B[1], q[2] - B[2]) < 2.5)
        # a real idle light is bright (28 Oct, Left green: 0.56-0.75 of the white light's spot); a speck of noise
        # near one light (Center 19:34, selected follow, its idle light off) is under a tenth
        if max(ea, eb) < 0.8 * (ea + eb) or max(ea, eb) < IDLE_MIN * (A[0] if ea > eb else B[0]):
            return dict(ok=True, sign=None, heading=None, pair=_pair(A, B),
                        why="front and back not told apart (older firmware: its idle light did not pick one side)")
        L, R = (A, B) if ea > eb else (B, A)
        sign = "idle light (left)"
    sep = math.hypot(L[1] - R[1], L[2] - R[2])
    if not (PAIR_CM[0] <= sep <= PAIR_CM[1]):
        return dict(ok=False, why="its two lights %.1f cm apart (they are 6.5-7.5)" % sep)
    h = wrap180(math.degrees(math.atan2(L[2] - R[2], L[1] - R[1])) - 90.0)
    return dict(ok=True, heading=round(h, 1), left=(round(L[1], 2), round(L[2], 2)),
                right=(round(R[1], 2), round(R[2], 2)), mid=(round((L[1] + R[1]) / 2, 2), round((L[2] + R[2]) / 2, 2)),
                sep=round(sep, 2), sign=sign, pair=_pair(L, R))


def _pair(a, b):
    """The line through the two lights, as a heading modulo 180 (deg in -90..90) and their middle."""
    h = math.degrees(math.atan2(a[2] - b[2], a[1] - b[1])) - 90.0
    h = (h + 90.0) % 180.0 - 90.0
    return dict(axis=round(h, 1), mid=(round((a[1] + b[1]) / 2, 2), round((a[2] + b[2]) / 2, 2)))


def combine(per_cam):
    """{camera: one_camera result} -> dict(heading or None, sure, how, spread). Side and overhead weighted by
    CAM_SIGMA; two cameras more than AGREE_DEG apart: not sure."""
    got = [(c, r) for c, r in per_cam.items() if r.get("ok") and r.get("heading") is not None]
    if not got:
        axes = [(c, r) for c, r in per_cam.items() if r.get("ok") and r.get("pair")]
        why = "; ".join("%s camera: %s" % (c, r.get("why")) for c, r in per_cam.items() if r.get("why")) or "no camera"
        if axes:
            return dict(heading=None, sure=False, axis=axes[0][1]["pair"]["axis"], how=why,
                        mid=axes[0][1]["pair"]["mid"])
        return dict(heading=None, sure=False, how=why)
    if len(got) == 2:
        (c1, r1), (c2, r2) = got
        d = abs(wrap180(r1["heading"] - r2["heading"]))
        w1, w2 = 1.0 / CAM_SIGMA.get(c1, 8.0) ** 2, 1.0 / CAM_SIGMA.get(c2, 8.0) ** 2
        h = wrap180(r1["heading"] + w2 / (w1 + w2) * wrap180(r2["heading"] - r1["heading"]))
        mid = tuple(round((w1 * a + w2 * b) / (w1 + w2), 2) for a, b in zip(r1["mid"], r2["mid"]))
        return dict(heading=round(h, 1), sure=d <= AGREE_DEG, spread=round(d, 1), mid=mid,
                    how="both cameras (%s %.0f, %s %.0f)" % (c1, r1["heading"], c2, r2["heading"]))
    c, r = got[0]
    return dict(heading=r["heading"], sure=True, mid=r["mid"], how="the %s camera alone" % c)


# ------------------------------------------------------------------------------------------------ plain words
def compass(h):
    """A heading in board words: +x is 'towards x = end', etc."""
    return "%.0f deg" % h


def verdict(name, meas, assumed, tol=TOL_DEG, front="its front (the distance sensor)", target=None):
    """Plain words and ok for one robot. assumed: the heading the run assumes (deg) or None. target: words for where
    its front should point (e.g. 'across the table, towards the far long side'). Returns dict(ok, words, off)."""
    h = meas.get("heading")
    if h is None:
        if meas.get("axis") is not None and assumed is not None:
            off = abs(wrap180(meas["axis"] - assumed))
            off = min(off, 180.0 - off)
            if off > tol:
                return dict(ok=False, off=None, words="%s is turned sideways (its lights' line is about %.0f deg off "
                                                      "its start direction): turn it so %s faces %s." % (
                                                          name, off, front, target or "its start direction"))
            return dict(ok=True, off=None, words="%s: not sideways, but its front and back could not be told apart "
                                                 "(%s). Check by eye that %s faces %s." % (
                                                     name, meas.get("how"), front, target or "its start direction"))
        return dict(ok=False, off=None, words="%s: could not tell which way it faces (%s). Is it on the board with its "
                                              "lights on top? Nothing moved." % (name, meas.get("how")))
    if assumed is None:
        return dict(ok=True, off=None, words="%s faces %.0f deg (%s)." % (name, h, meas.get("how")))
    off = wrap180(h - assumed)
    a = abs(off)
    side = "to its left" if off > 0 else "to its right"
    if not meas.get("sure"):
        return dict(ok=False, off=round(off, 1), words="%s: the two cameras disagree on which way it faces (%s). "
                                                       "Nothing moved: look at it, then start again." % (name, meas.get("how")))
    if a > 135:
        words = ("%s is facing the wrong way (about %.0f deg off): turn it round so %s faces %s."
                 % (name, a, front, target or "its start direction"))
        return dict(ok=False, off=round(off, 1), words=words)
    if a > max(tol, SIDEWAYS_DEG):
        words = ("%s is turned about %.0f deg %s (sideways): turn it so %s faces %s."
                 % (name, a, side, front, target or "its start direction"))
        return dict(ok=False, off=round(off, 1), words=words)
    if a > tol:
        return dict(ok=False, off=round(off, 1),
                    words="%s is turned about %.0f deg %s (its start allows %.0f): turn it so %s faces %s."
                          % (name, a, side, tol, front, target or "its start direction"))
    return dict(ok=True, off=round(off, 1), words="%s faces its start direction (%.0f deg off, %s)." % (name, a, meas.get("how")))


# ------------------------------------------------------------------------------------------------ the cameras
class Grab:
    """The newest pictures of each camera (the recorder's /live streams, or the simulator's), with their Pi times."""

    def __init__(self, streams, keep=60):
        import sources as S
        self.buf = {}
        self.err = {}
        self.closed = False
        self.srcs = {}
        for name, url in streams.items():
            if not url:
                continue
            self.buf[name] = deque(maxlen=keep)
            src = S.StreamSource(url, log=lambda s: None)
            self.srcs[name] = src
            threading.Thread(target=self._run, args=(name, src), daemon=True).start()

    def _run(self, name, src):
        try:
            for fr in src:
                if self.closed:
                    break
                self.buf[name].append((fr.t_pi - 0.04, fr.img, time.time()))
        except Exception as e:     # noqa: BLE001
            self.err[name] = str(e)

    def wait_first(self, timeout=6.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if all(len(b) for b in self.buf.values()):
                return True
            time.sleep(0.05)
        return False

    def between(self, name, t0, t1, wait_s=1.5):
        """Pictures of camera name taken between t0 and t1 (this machine's clock), waiting for them to arrive."""
        end = time.time() + wait_s
        while time.time() < end:
            b = list(self.buf.get(name, []))
            if b and b[-1][0] >= t1:
                break
            time.sleep(0.03)
        return [im for t, im, _ in list(self.buf.get(name, [])) if t0 <= t <= t1]

    def close(self):
        self.closed = True
        for s in self.srcs.values():
            try:
                s.close()
            except Exception:     # noqa: BLE001
                pass


# ------------------------------------------------------------------------------------------------ the check
def new_firmware(fw):
    return bool(fw) and str(fw) >= "2026-10-29a"


def check(names, send, acked, grab, cams, assumed=None, near=None, firmware=None, rect=None, tol=TOL_DEG,
          say=print, targets=None, log=None):
    """Measure every robot in names, one at a time, nothing moving. send(name, cmd); acked(name, t_sent, words) ->
    time of the robot's answer containing words, or None (waited for); grab: Grab; cams: {"overhead": cam,
    "side": cam}; assumed: {name: heading the run assumes}; near: {name: (x, y)} or None; firmware: {name: version}.
    Returns dict(ok, robots={name: dict(meas, verdict)}, seconds)."""
    t_start = time.time()
    assumed, near, firmware, targets = assumed or {}, near or {}, firmware or {}, targets or {}
    res = {}

    def cmd(n, c, words):
        for _ in range(2):                                # one resend: a `led` lost on the WiFi (28 Oct: all three lost some)
            t = time.time()
            send(n, c)
            ta = acked(n, t, words)
            if ta is not None:
                return ta
        return None

    dark_ok = []
    for n in names:                                       # everyone dark first (one resend each if not answered)
        if cmd(n, "led 0 0 0 %d" % REPEAT_DARK_MS, "led 0 0 0") is None:
            res[n] = dict(meas=dict(heading=None, sure=False, how="it did not answer `led`"), verdict=None)
        else:
            dark_ok.append(n)
    td = time.time()
    time.sleep(SETTLE_S + COLLECT_S)
    dark = {c: grab.between(c, td + SETTLE_S - 0.05, td + SETTLE_S + COLLECT_S) for c in cams}
    done_spots = []                                       # lights of robots measured already: never another's

    def measure(n):
        new = new_firmware(firmware.get(n))
        t1 = cmd(n, "led 1 1 1 1500 L", "led 1 1 1")
        if t1 is None:
            return None
        time.sleep(SETTLE_S + COLLECT_S)
        first = {c: grab.between(c, t1 + SETTLE_S, t1 + SETTLE_S + COLLECT_S) for c in cams}
        if new:
            t2 = cmd(n, "led 1 1 1 1500 R", "led 1 1 1")
        else:
            t2 = cmd(n, "led 0 0 0 60", "led 0 0 0")       # its idle state: the left light in its experiment's colour
            if t2 is not None:
                t2 += 0.06
        time.sleep(SETTLE_S + COLLECT_S)
        second = {c: grab.between(c, (t2 or 0) + SETTLE_S, (t2 or 0) + SETTLE_S + COLLECT_S) for c in cams}
        cmd(n, "led 0 0 0 %d" % REPEAT_DARK_MS, "led 0 0 0")   # dark again (answered) while the others are measured
        per = {}
        for c, cam in cams.items():
            per[c] = one_camera(cam, dark[c], first[c], second[c], not new, rect, near.get(n), exclude=done_spots)
        meas = combine(per)
        meas["cameras"] = {c: {k: v for k, v in r.items() if k != "pair"} for c, r in per.items()}
        return meas

    for n in dark_ok:
        meas = measure(n)
        for k in range(MEASURE_RETRIES):
            # its lights not seen (too few pictures in the window: 30 Oct 10:02, Center "0" lit pictures; or a lost
            # `led`): measured again, a little longer each time, before anything gives up (Nick 10:08: no stops)
            if meas is not None and (meas.get("heading") is not None or meas.get("axis") is not None):
                break
            say("  %s: its lights were not read (%s): measuring it again (%d of %d)" % (
                n, (meas or {}).get("how", "no answer to `led`"), k + 1, MEASURE_RETRIES))
            time.sleep(0.3 * (k + 1))
            meas = measure(n)
        if meas is not None and meas.get("heading") is not None and not meas.get("sure"):
            again = measure(n)                            # the cameras disagreed: once more
            if again is not None and again.get("sure"):
                meas = again
            if meas is not None:
                meas["repeated"] = True
        if meas is None:
            res[n] = dict(meas=dict(heading=None, sure=False, how="it did not answer `led`"), verdict=None)
            continue
        if meas.get("mid"):
            done_spots.append(meas["mid"])
        v = verdict(n, meas, assumed.get(n), tol=tol.get(n, TOL_DEG) if isinstance(tol, dict) else tol,
                    target=targets.get(n))
        res[n] = dict(meas=meas, verdict=v)
        if log:
            log(n, meas, v)
    for n in names:
        send(n, "led 0 0 0 60")                            # back to normal
    ok = True
    for n in names:
        v = res[n]["verdict"]
        if v is None:
            ok = False
            say("  %s: %s. Nothing moved." % (n, res[n]["meas"]["how"]))
            continue
        say("  " + v["words"])
        ok = ok and v["ok"]
    return dict(ok=ok, robots=res, seconds=round(time.time() - t_start, 1))


def identity_words(res, starts, tol_cm=10.0):
    """A robot's lights nearer another robot's start spot than its own (and over tol_cm from its own): swapped?"""
    out = []
    for n, r in res.items():
        mid = (r.get("meas") or {}).get("mid")
        if mid is None or n not in starts:
            continue
        own = math.hypot(mid[0] - starts[n][0], mid[1] - starts[n][1])
        if own <= tol_cm:
            continue
        for o, s in starts.items():
            if o != n and math.hypot(mid[0] - s[0], mid[1] - s[1]) < own:
                out.append("%s's lights are on %s's start spot (%.0f cm from its own): are %s and %s swapped?"
                           % (n, o, own, n, o))
                break
    return out


def towards_words(h, rect):
    """Where a heading points on the board, in words: 'towards the y = 61 side (90 deg)'."""
    c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
    if rect is None:
        return "%.0f deg" % h
    x0, y0, x1, y1 = rect
    if abs(c) >= abs(s):
        return "towards the x = %g %s (%.0f deg)" % (round(x1 if c > 0 else x0, 1), "end" if abs(x1 - x0) >= abs(y1 - y0) else "side", h)
    return "towards the y = %g %s (%.0f deg)" % (round(y1 if s > 0 else y0, 1), "side" if abs(x1 - x0) >= abs(y1 - y0) else "end", h)


def run_plan_check(coord, link, args, names, say=print, straighten=False, tag="facing"):
    """run_plan.py's start (Monday, Mission Control's Run and Setup): every robot's facing against its start heading in
    the plan, before its follow run starts (`led` is refused during one). Raises SystemExit (nothing has moved) when a
    robot faces the wrong way, sideways, or cannot be measured. Writes facing.json in the session folder.
    straighten (run_plan --straighten, 2026-10-29): a robot that faces the wrong way is not a stop: its measured heading
    is kept (res["headings"]) for the start's correction; only a robot that cannot be measured (or whose two cameras
    disagree twice) stops the start. tag: the file name (facing.json, facing-recheck-1.json ...)."""
    import json
    import e02_camera as CAM
    t0 = time.time()
    cal, scal = args.facing_calib, args.facing_side_calib or ""
    while not (cal and os.path.exists(cal)) and time.time() - t0 < 25:    # the simulator writes its models at start
        time.sleep(0.2)
    if not (cal and os.path.exists(cal)):
        raise SystemExit("Facing check: no overhead camera calibration (%s). Nothing moved. (--no-facing-check "
                         "skips the check)" % cal)
    cams = {"overhead": CAM.load_camera(cal)}
    streams = {"overhead": args.facing_stream}
    if scal and args.facing_side_stream:
        while not os.path.exists(scal) and time.time() - t0 < 25:
            time.sleep(0.2)
        if os.path.exists(scal):
            cams["side"] = CAM.load_camera(scal)
            streams["side"] = args.facing_side_stream
    grab = Grab(streams)
    try:
        if not grab.wait_first(8.0):
            missing = [c for c, b in grab.buf.items() if not b]
            for c in missing:
                cams.pop(c, None)
            if not cams:
                raise SystemExit("Facing check: no pictures from the cameras (%s). Nothing moved." % ", ".join(missing))
            say("  facing check: no pictures from the %s camera; the other alone" % ", ".join(missing))
        m = coord.m
        try:
            f = m["floor"]
            rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
        except (KeyError, TypeError):
            rect = None
        starts = {n: coord.plan["robots"][n]["start"] for n in names}
        assumed = {n: float(starts[n][2]) for n in names}
        targets = {n: towards_words(assumed[n], rect) for n in names}
        fw = {n: coord.bots[n].firmware for n in names}

        def send(n, c):
            link.send_name(n, c)

        def acked(n, t_sent, words, timeout=0.6):
            end = time.time() + timeout
            while time.time() < end:
                coord.drain()
                for t, v in list(coord.bots[n].said)[-6:]:
                    if t >= t_sent - 0.01 and words in v and "led" in v:
                        return t
                time.sleep(0.02)
            return None

        say("Facing check: which way each robot faces, from its lights (nothing moves; about %.0f s) ..."
            % (2.0 + 3.0 * len(names)))
        tols = {n: float(args.facing_tol_deg) for n in names}
        res = check(names, send, acked, grab, cams, assumed=assumed, firmware=fw, rect=rect, tol=tols,
                    say=(lambda w: None) if straighten else say, targets=targets)
        if straighten:
            res["headings"] = {}
            ok = True
            for n in names:
                r = res["robots"][n]
                meas = r["meas"]
                h = meas.get("heading")
                if h is None and meas.get("axis") is not None:
                    # old firmware, front and back not told apart: the end of its lights' line nearer its start heading
                    a = float(meas["axis"])
                    h = a if abs(wrap180(a - assumed[n])) <= 90 else wrap180(a + 180.0)
                    say("  %s: its lights' line is at %.0f deg, but front and back could not be told apart (%s): taken "
                        "as %.0f deg, the end nearer its start heading, checked by a short watched crawl"
                        % (n, a, meas.get("how"), h))
                    res.setdefault("unsure", []).append(n)
                elif h is None:
                    say("  WARNING: %s: which way it faces is not known from its lights (%s): it is found by a short "
                        "watched crawl instead" % (n, meas.get("how")))
                    res.setdefault("unknown", []).append(n)
                    continue
                elif not meas.get("sure"):
                    # the cameras disagree twice: the weighted heading (the side camera counts more) is kept for now
                    # and confirmed by a short watched crawl before anything else moves it
                    say("  WARNING: %s: the two cameras disagree on which way it faces, twice (%s): taken as %.0f deg "
                        "for now, checked by a short watched crawl" % (n, meas.get("how"), h))
                    res.setdefault("unsure", []).append(n)
                res["headings"][n] = float(h)
                off = wrap180(float(h) - assumed[n])
                say("  %s faces %.0f deg (%s): %.0f deg off its start heading%s" % (
                    n, h, meas.get("how"), abs(off), "" if abs(off) <= 3.0 else
                    " (backwards)" if abs(off) > 135 else ""))
            res["ok"] = ok
        for w in identity_words(res["robots"], {n: starts[n][:2] for n in names}):
            if straighten:
                # robots may be put down anywhere (Nick, 30 Oct: "the user only clicks corners 1-4; the robots
                # self-correct"): who is who comes from which robot lit its light, so a robot nearer another's start
                # spot is driven to its own (straighten.py keeps their ways 8 cm apart, or refuses), not a stop
                say("  (note) " + w.split(": are ")[0] + ": it drives to its own start spot")
                continue
            say("  " + w)
            res["ok"] = False
        old = [n for n in names if not new_firmware(fw.get(n))]
        if old:
            say("  (%s: firmware %s; 2026-10-29a lights one side at a time, so front and back are told apart every "
                "time: flash it)" % (", ".join(old), ", ".join(sorted(set(str(fw.get(n)) for n in old)))))
        try:
            with open(os.path.join(getattr(coord, "outdir", None) or ".", tag + ".json"), "w") as f:
                json.dump(dict(res, assumed=assumed, allowed_deg=tols), f, indent=1, default=str)
        except OSError:
            pass
        coord.log("facing_check", ok=res["ok"], seconds=res["seconds"],
                  robots={n: dict(heading=r["meas"].get("heading"), off=(r["verdict"] or {}).get("off"),
                                  how=r["meas"].get("how")) for n, r in res["robots"].items()})
        coord.start_checks[tag] = {n: dict(heading=r["meas"].get("heading"), off=(r["verdict"] or {}).get("off"),
                                                ok=(r["verdict"] or {}).get("ok", False)) for n, r in res["robots"].items()}
        say("  facing check done in %.1f s" % res["seconds"])
        if not res["ok"] and straighten:
            raise SystemExit("Facing check: a robot could not be measured (above), so it cannot be straightened. "
                             "Nothing moved%s." % ("" if tag == "facing" else " since its correction"))
        if not res["ok"]:
            raise SystemExit("Facing check: not every robot faces its start direction (above). Nothing moved. Put "
                             "it right and start again.")
        return res
    finally:
        grab.close()


# ------------------------------------------------------------------------------------------------ self-test
def _selftest():
    """Drawn pictures: a camera looking straight down (1 px = 0.5 cm), a robot's two lights 6.8 cm apart."""
    import cv2

    class Cam:
        def to_floor(self, u, v, z):
            return u * 0.5, v * 0.5

    cam = Cam()
    rng = np.random.default_rng(1)
    base = (150 + rng.normal(0, 3, (260, 260))).astype(np.float32)

    def pic(lights):
        im = base.copy() + rng.normal(0, 2, base.shape)
        for x, y in lights:
            cv2.circle(im, (int(round(x / 0.5)), int(round(y / 0.5))), 2, 255, -1)
        return np.clip(im, 0, 255).astype(np.uint8)

    worst = 0.0
    for h in range(-180, 180, 15):
        r = math.radians(h)
        cx, cy = 60.0, 70.0
        L = (cx - 3.4 * math.sin(r), cy + 3.4 * math.cos(r))
        R = (cx + 3.4 * math.sin(r), cy - 3.4 * math.cos(r))
        dark = [pic([]) for _ in range(5)]
        res = one_camera(cam, dark, [pic([L]) for _ in range(5)], [pic([R]) for _ in range(5)], False)
        assert res["ok"], res
        worst = max(worst, abs(wrap180(res["heading"] - h)))
        old = one_camera(cam, dark, [pic([L, R]) for _ in range(5)], [pic([L]) for _ in range(5)], True)
        assert old["ok"] and abs(wrap180(old["heading"] - h)) < 8, (h, old)
    print("selftest: 24 headings, worst error %.1f deg" % worst)


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
