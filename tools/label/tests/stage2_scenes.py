#!/usr/bin/env python3
"""Synthetic scenes for the stage-2 tracking tests (appearance chain, splits, rejected spots).

A scene is a textured floor with objects that follow keyframes (time s, x, y); between two
keyframes an object moves in a straight line, and two equal keyframes make it stand still.
Pictures are drawn on demand (render(f)), so a 24,000-frame recording costs no memory.

Alongside the pictures, dets(f) gives what a motion finder would report: a robot is seen only
while it moves and for absorb_s after it stops (then the background swallows it); two robots
closer than merge_r robot sizes are one blob in the middle; a hand is always seen; a stain is
reported on the frames listed in its "flicker" (a lamp switched on, say).

    sc = Scene(seconds=80, fps=25)
    sc.robot("A", [(0, 100, 240), (4, 220, 240), (74, 220, 240), (78, 340, 240)])
    sc.hand("H", [(20, 320, -40), (21, 320, 240), (22, 320, -40)])
    sc.stain("S", 400, 300, flicker=range(100, 140))
    frames = sc.frames()           # appearance.ArrayFrames
    dets = sc.all_dets()           # list per frame
    sc.truth("A", f)               # (x, y) or None

    python tools/label/tests/stage2_scenes.py bench [--frames 24000] [--video]   # speed check
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
LABEL = os.path.dirname(HERE)
if LABEL not in sys.path:
    sys.path.insert(0, LABEL)

import appearance  # noqa: E402

W, H, R = 640, 480, 30.0


def floor_picture(seed=3):
    """A grey, gently textured floor with two tape lines (so patches of floor are not all alike)."""
    rng = np.random.default_rng(seed)
    low = cv2.resize(rng.normal(0, 1, (H // 16, W // 16)).astype(np.float32), (W, H), interpolation=cv2.INTER_CUBIC)
    fine = cv2.GaussianBlur(rng.normal(0, 1, (H, W)).astype(np.float32), (0, 0), 1.2)
    img = 125 + 8 * low + 5 * fine
    img[:, 52:56] = 200
    img[400:404, :] = 200
    return np.clip(img, 0, 255).astype(np.uint8)


class Obj:
    def __init__(self, name, kind, keys, fps, n, flicker=()):
        self.name, self.kind = name, kind
        self.keys = sorted(keys)
        t = np.arange(n) / fps
        ks = np.array([k[0] for k in self.keys], dtype=float)
        self.x = np.interp(t, ks, [k[1] for k in self.keys])
        self.y = np.interp(t, ks, [k[2] for k in self.keys])
        self.flicker = set(flicker)
        # frame of the last move (to know when the background has swallowed it)
        moved = np.zeros(n, dtype=bool)
        moved[1:] = np.hypot(np.diff(self.x), np.diff(self.y)) > 1e-6
        last = np.zeros(n, dtype=int) - 10 ** 9
        lm = -10 ** 9
        for f in range(n):
            if moved[f]:
                lm = f
            last[f] = lm
        self.last_move = last

    def visible(self, f):
        x, y = self.x[f], self.y[f]
        return -R < x < W + R and -R < y < H + R


class Scene:
    def __init__(self, seconds=60.0, fps=25.0, seed=3, absorb_s=3.0, merge_r=1.15, noise=2.0, det_noise=0.8):
        self.fps = float(fps)
        self.n = int(round(seconds * fps))
        self.times = [f / self.fps for f in range(self.n)]
        self.floor = floor_picture(seed)
        self.objs = []
        self.absorb = int(round(absorb_s * fps))
        self.merge = merge_r * R
        rng = np.random.default_rng(seed + 1)
        self.noise = [rng.normal(0, noise, (H, W)).astype(np.int16) for _ in range(7)]
        self.det_noise = det_noise
        self.rng = np.random.default_rng(seed + 2)
        self._dets = None

    # --- objects ------------------------------------------------------------------------
    def robot(self, name, keys):
        self.objs.append(Obj(name, "robot", keys, self.fps, self.n))
        return self

    def hand(self, name, keys):
        self.objs.append(Obj(name, "hand", keys, self.fps, self.n))
        return self

    def stain(self, name, x, y, flicker=()):
        self.objs.append(Obj(name, "stain", [(0, x, y), (1e9, x, y)], self.fps, self.n, flicker))
        return self

    def obj(self, name):
        return next(o for o in self.objs if o.name == name)

    def truth(self, name, f):
        o = self.obj(name)
        return (float(o.x[f]), float(o.y[f])) if o.visible(f) else None

    # --- pictures -----------------------------------------------------------------------
    def render(self, f):
        img = self.floor.copy()
        order = {"stain": 0, "robot": 1, "hand": 2}
        for o in sorted(self.objs, key=lambda o: order[o.kind]):
            if not o.visible(f):
                continue
            x, y = int(round(o.x[f])), int(round(o.y[f]))
            if o.kind == "stain":
                cv2.ellipse(img, (x, y), (8, 5), 30, 0, 360, 85, -1)
                cv2.circle(img, (x + 4, y - 3), 3, 95, -1)
            elif o.kind == "robot":
                h = int(R / 2)
                cv2.rectangle(img, (x - h, y - h), (x + h, y + h), 45, -1)
                cv2.circle(img, (x, y), 6, 150, -1)
                cv2.rectangle(img, (x + h - 6, y - 8), (x + h - 2, y + 8), 210, -1)
            else:   # a hand reaching in from the top edge
                cv2.rectangle(img, (x - 14, -5), (x + 14, y), 185, -1)
                cv2.ellipse(img, (x, y), (22, 16), 0, 0, 360, 192, -1)
        out = img.astype(np.int16) + self.noise[f % len(self.noise)]
        return np.clip(out, 0, 255).astype(np.uint8)

    def frames(self, budget_mb=256):
        return appearance.ArrayFrames(self.render, self.n, (W, H), robot_px=R, budget_mb=budget_mb)

    # --- what a motion finder would report ------------------------------------------------
    def dets(self, f):
        seen = []
        for o in self.objs:
            if not o.visible(f):
                continue
            if o.kind == "robot" and f - o.last_move[f] <= self.absorb:
                seen.append(["alvik", float(o.x[f]), float(o.y[f]), [o.name]])
            elif o.kind == "hand":
                seen.append(["hand", float(o.x[f]), float(o.y[f]), [o.name]])
            elif o.kind == "stain" and f in o.flicker:
                seen.append(["alvik", float(o.x[f]), float(o.y[f]), [o.name]])
        # robots close together are one blob
        merged = True
        while merged:
            merged = False
            for i in range(len(seen)):
                for j in range(i + 1, len(seen)):
                    a, b = seen[i], seen[j]
                    if a[0] == b[0] == "alvik" and math.hypot(a[1] - b[1], a[2] - b[2]) < self.merge:
                        seen[i] = ["alvik", (a[1] + b[1]) / 2, (a[2] + b[2]) / 2, a[3] + b[3]]
                        del seen[j]
                        merged = True
                        break
                if merged:
                    break
        out = []
        for cls, x, y, names in seen:
            out.append(dict(cls=cls, x=x + float(self.rng.normal(0, self.det_noise)),
                            y=y + float(self.rng.normal(0, self.det_noise)), score=0.8, angle90=None,
                            _who=names))
        return out

    def all_dets(self):
        if self._dets is None:
            self._dets = [self.dets(f) for f in range(self.n)]
        return self._dets

    def write_video(self, path, every=1):
        vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), self.fps, (W, H))
        for f in range(0, self.n, every):
            vw.write(cv2.cvtColor(self.render(f), cv2.COLOR_GRAY2BGR))
        vw.release()


def anchor_obj(obj_id, x, y, cls="alvik", status="confirmed"):
    return dict(obj=obj_id, cls=cls, x=float(x), y=float(y), heading_deg=None, heading_known=False,
                source="user", status=status)


# ------------------------------------------------------------------------------------------
# Speed check: a 16-minute recording at 25 fps
# ------------------------------------------------------------------------------------------

def long_scene(seconds=960.0, fps=25.0):
    """Three robots for a long recording: one parked all the time (confirmed once), one that drives
    a loop and stops for a minute every few minutes, one that drives most of the time; a hand
    reaching in now and then; a stain that flickers."""
    sc = Scene(seconds=seconds, fps=fps)
    sc.robot("parked", [(0, 560, 90), (seconds, 560, 90)])
    keys, t = [], 0.0
    corners = [(120, 120), (420, 120), (420, 360), (120, 360)]
    k = 0
    while t < seconds:
        x, y = corners[k % 4]
        keys.append((t, x, y))
        t += 5.0
        if k % 8 == 7:
            nx, ny = corners[(k + 1) % 4]   # arrived at the next corner: stops there a minute
            keys.append((t, nx, ny))
            t += 60.0
        k += 1
    sc.robot("looper", keys)
    keys, t, k = [], 0.0, 0
    rng = np.random.default_rng(5)
    while t < seconds:
        keys.append((t, float(rng.uniform(150, 600)), float(rng.uniform(380, 450))))
        t += float(rng.uniform(3, 8))
    sc.robot("wanderer", keys)
    hk = [(0, 300, -60)]
    for t0 in np.arange(30, seconds - 5, 90):
        hk += [(t0, 300, -60), (t0 + 1, 330, 200), (t0 + 2, 300, -60)]
    sc.hand("hand", hk)
    sc.stain("stain", 250, 250, flicker=[f for f in range(sc.n) if (f // 250) % 20 == 3])
    return sc


def bench(n_frames=24000, video=False, out_dir=None):
    import tracker
    sc = long_scene(seconds=n_frames / 25.0)
    t0 = time.time()
    dets = sc.all_dets()
    t_dets = time.time() - t0
    anchors = {0: [anchor_obj(1, 560, 90)]}
    res = {}
    if video:
        out_dir = out_dir or "."
        path = os.path.join(out_dir, "bench.avi")
        if not os.path.exists(path):
            t0 = time.time()
            sc.write_video(path)
            res["write_video_s"] = round(time.time() - t0, 1)
        frames = appearance.VideoFrames(path, R)
    else:
        frames = sc.frames()
    t0 = time.time()
    run = tracker.TrackRun(dets, sc.times, R, (W, H), frames=frames)
    rows, summary = run.run(anchors, {1})
    res["first_run_s"] = round(time.time() - t0, 1)
    res["decoded"] = frames.stats["decoded"]
    t0 = time.time()
    run.run(anchors, {1}, from_frame=n_frames // 2)
    res["rerun_from_middle_s"] = round(time.time() - t0, 1)
    t0 = time.time()
    tracker.TrackRun(dets, sc.times, R, (W, H)).run(anchors, {1})
    res["without_pictures_s"] = round(time.time() - t0, 1)
    res["frames"] = n_frames
    res["cached_mb"] = round(frames.cached_bytes / 1e6, 1)
    res["make_guesses_s"] = round(t_dets, 1)
    parked = [r for f in range(n_frames) for r in rows.get(f, []) if r["obj"] == 1]
    res["parked_found_frames"] = len(parked)
    res["parked_max_err_px"] = round(max(math.hypot(r["x"] - 560, r["y"] - 90) for r in parked), 2) if parked else None
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("what", choices=["bench"])
    ap.add_argument("--frames", type=int, default=24000)
    ap.add_argument("--video", action="store_true", help="write a Motion-JPEG video and read it back")
    ap.add_argument("--out", default=".")
    a = ap.parse_args()
    print(bench(a.frames, a.video, a.out))


if __name__ == "__main__":
    main()
