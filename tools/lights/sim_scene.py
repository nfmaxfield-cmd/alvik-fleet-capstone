#!/usr/bin/env python3
"""Simulated overhead frames with light patterns, for tests and the numbers in
tools/blink/README.md ("Light patterns"). Draws with the simulator's own
renderer (tools/sim/camera.py: floor, robot bodies, their blinking top LEDs,
the pattern, sensor noise) and the real camera preset
(tools/sim/configs/real_oct18.json: 262 cm up, 640 x 480, 25 fps; a
different scale is a different floor width in view). Robots move with
tools/blink/tests/synthworld.py's Robot (commands, blink-code timeline).
Each frame goes through JPEG (as the Pi's stream does).

Scores, per robot: time from the lights / blink code coming on to the first
correctly named row (lights) or fix (blink, via tools/live/livecore.py on the
same frames), rows named wrongly (another robot nearer, or more than 3 cm
from the named robot), heading error, and the share of frames with a named
row.

    python tools/lights/sim_scene.py --scene pass --mode l3 --scale 3.4
"""
import argparse
import json
import math
import os
import sys
import tempfile

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "live"), os.path.join(TOOLS, "blink"),
          os.path.join(TOOLS, "blink", "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import patterns as PT  # noqa: E402
import detect as D  # noqa: E402
import config as SC  # noqa: E402  (tools/sim/config.py)
import camera as CAM  # noqa: E402
import synthworld as SW  # noqa: E402

REAL = os.path.join(TOOLS, "sim", "configs", "real_oct18.json")
BLINK_CODE = {"Left": 1, "Center": 2, "Right": 3}


class Scene:
    def __init__(self, robots, codes, mode="l3", scale=3.4, seconds=8.0, seed=0, family=None, hide=None,
                 blink=True, lamps=True, brightness=None, blur_px=None, jpeg=95, lights_on=0.3):
        """robots: synthworld.Robot list (blink timeline: add_blink); codes: name -> lights code;
        hide: [(name, led index, t0, t1)] LEDs covered; scale: px per cm on the floor."""
        self.cfg = SC.load_config(REAL)
        self.cfg.camera.floor_width_cm = self.cfg.camera.width / scale
        self.cfg.camera.hands_per_min = 0.0
        self.cfg.lights.mode = mode
        self.cfg.lights.family = dict(family or {})
        if brightness is not None:
            self.cfg.lights.brightness = brightness
        if blur_px is not None:
            self.cfg.lights.blur_px = blur_px
        if mode == "strip":
            self.cfg.lights.led_d_cm = 0.2
        self.fam = PT.family(mode, **self.cfg.lights.family)
        self.rng = np.random.default_rng(seed)
        self.geom = CAM.CameraGeometry(self.cfg.camera)
        self.rend = CAM.Renderer(self.cfg, self.geom, self.rng)
        self.robots, self.codes, self.seconds = robots, dict(codes), seconds
        self.hide = hide or []
        self.blink, self.lamps, self.jpeg = blink, lamps, jpeg
        self.lights_on = lights_on
        self.dir = tempfile.mkdtemp(prefix="lights_scene_")
        self.calib = os.path.join(self.dir, "floor_calibration.json")
        with open(self.calib, "w") as f:
            json.dump(self.geom.calibration(), f)
        # distractors: static bright specks and a lamp flickering at random
        self.specks = [(self.rng.uniform(20, 620), self.rng.uniform(20, 460)) for _ in range(12)] if lamps else []
        self.lamp = (self.rng.uniform(40, 600), self.rng.uniform(40, 440)) if lamps else None

    def frames(self):
        c = self.cfg.camera
        n = int(self.seconds * c.fps)
        exp = c.exposure_ms / 1000.0
        for fi in range(n):
            tc = fi / c.fps + self.rng.normal(0, 0.0005)
            img = self.rend.bg.copy()
            truth = {}
            for r in self.robots:
                x, y, th = r.pose(tc)
                ts = tc - exp * (np.arange(4) + 0.5) / 4
                lv = float(np.mean([r.led(t)[0] for t in ts])) if self.blink else 0.0
                self.rend.draw_robot(img, x, y, th, 0.0, (lv, lv, lv), (lv, lv, lv))
                code = self.codes.get(r.name)
                if code is not None and tc >= self.lights_on:
                    lit = np.mean([np.array(self.fam.lit(code, t - self.lights_on), float) for t in ts], axis=0)
                    for name, k, t0, t1 in self.hide:
                        if name == r.name and t0 <= tc < t1:
                            lit[k] = 0.0
                    self.rend.draw_lights(img, self.fam, code, lit, x, y, th)
                truth[r.name] = (x, y, th)
            for (u, v) in self.specks:
                cv2.circle(img, (int(u * 16), int(v * 16)), 16 * 1, (250, 250, 250), -1, cv2.LINE_AA, 4)
            if self.lamp is not None and (math.sin(tc * 7.3) + math.sin(tc * 3.1)) > 0.6:
                cv2.circle(img, (int(self.lamp[0] * 16), int(self.lamp[1] * 16)), 16 * 2, (255, 255, 255), -1,
                           cv2.LINE_AA, 4)
            self.rend.add_noise(img)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)          # the camera is black-and-white
            if self.jpeg:                                        # as camrec.py's /live stream (grey JPEG)
                ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg])
                img = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
            yield fi, tc, img, truth


def write(scene, folder):
    """The scene as a camrec.py-style recording (video.avi MJPG, frames.csv, meta.json,
    floor_calibration.json) plus truth.csv (frame, t_wall, robot, x_cm, y_cm, th_deg). t_wall = capture time."""
    import csv
    os.makedirs(folder, exist_ok=True)
    c = scene.cfg.camera
    vw = cv2.VideoWriter(os.path.join(folder, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), c.fps, (c.width, c.height))
    with open(os.path.join(folder, "frames.csv"), "w", newline="") as ff, \
            open(os.path.join(folder, "truth.csv"), "w", newline="") as tf:
        fw, tw = csv.writer(ff), csv.writer(tf)
        fw.writerow(["frame", "t_wall", "t_mono"])
        tw.writerow(["frame", "t_wall", "robot", "x_cm", "y_cm", "th_deg"])
        for fi, tc, img, truth in scene.frames():
            vw.write(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
            fw.writerow([fi, "%.4f" % tc, "%.4f" % tc])
            for n, (x, y, th) in truth.items():
                tw.writerow([fi, "%.4f" % tc, n, "%.3f" % x, "%.3f" % y, "%.2f" % math.degrees(th)])
    vw.release()
    with open(os.path.join(folder, "meta.json"), "w") as f:
        json.dump(dict(device="lights-scene", width=c.width, height=c.height, fps_reported=c.fps,
                       exposure_ms=c.exposure_ms, label="synthetic"), f)
    with open(os.path.join(folder, "floor_calibration.json"), "w") as f:
        json.dump(scene.geom.calibration(), f)
    return folder


def run(scene, names=None, params=None, with_blink=True):
    """Lights tracker (and the blink live tracker) over the scene's frames. Returns a score dict."""
    import floormap as FMAP
    fam = scene.fam
    fm = FMAP.FloorMap(scene.calib, led_height_cm=fam.height_cm)
    tr = D.Tracker(fam, fm, names or scene.codes, params)
    lt = None
    if with_blink:
        import blink as B
        import livecore as LC
        fm4 = FMAP.FloorMap(scene.calib)
        bn = {BLINK_CODE.get(r.name, 4): r.name for r in scene.robots}
        p = B.Params(names=bn, px_per_cm=fm4.px_per_cm())
        p.exposure_s = scene.cfg.camera.exposure_ms / 1000.0
        lt = LC.LiveTracker(fm4, bn, p, LC.LiveParams(led_offset={None: (2.75, 0.0)}))
    rows, fixes, truths = [], [], {}
    for fi, tc, img, truth in scene.frames():
        truths[fi] = (tc, truth)
        rows += tr.step(img, tc, fi)
        if lt is not None:
            for m in lt.push(img, tc):
                if m["type"] == "fix":
                    fixes.append(m)
    return score(scene, rows, fixes, truths, tr)


def score(scene, rows, fixes, truths, tr=None, far_cm=3.0):
    names = [r.name for r in scene.robots]
    on = scene.lights_on
    res = dict(per={n: dict(first_lights=None, first_frames=None, first_blink=None, named=0, frames=0) for n in names},
               rows=len(rows), named=0, wrong=0, far=0, herr=[], perr=[])
    seen_frames = {n: set() for n in names}
    on_frames = sorted(fi for fi, (tc, _) in truths.items() if tc >= on)
    for fi in on_frames:
        for n in names:
            res["per"][n]["frames"] += 1
    for r in rows:
        if not r["robot"]:
            continue
        res["named"] += 1
        tc, truth = truths[r["frame"]]
        own = truth.get(r["robot"])
        d_own = math.hypot(r["x_cm"] - own[0], r["y_cm"] - own[1]) if own else math.inf
        closer = any(math.hypot(r["x_cm"] - p[0], r["y_cm"] - p[1]) < d_own for n, p in truth.items() if n != r["robot"])
        if own is None or closer:
            res["wrong"] += 1
            continue
        if d_own > far_cm:
            res["far"] += 1
            continue
        res["perr"].append(d_own)
        res["herr"].append(abs((r["heading_floor_deg"] - math.degrees(own[2]) + 180) % 360 - 180))
        pr = res["per"][r["robot"]]
        if pr["first_lights"] is None:
            pr["first_lights"] = tc - on
            pr["first_frames"] = sum(1 for fi in on_frames if fi <= r["frame"])     # frames shown, this one included
        if r["frame"] not in seen_frames[r["robot"]]:
            seen_frames[r["robot"]].add(r["frame"])
            pr["named"] += 1
    for m in fixes:
        pr = res["per"].get(m["robot"])
        if pr is not None and pr["first_blink"] is None:
            pr["first_blink"] = m["t_pi"] - on
    res["fps"] = scene.cfg.camera.fps
    res["stats"] = dict(tr.stats) if tr else {}
    return res


def summary(res):
    fps = res["fps"]
    h = np.array(res["herr"]) if res["herr"] else np.array([np.nan])
    e = np.array(res["perr"]) if res["perr"] else np.array([np.nan])
    out = dict(named_rows=res["named"], wrong=res["wrong"], far=res["far"],
               heading_p95_deg=round(float(np.nanpercentile(h, 95)), 2) if res["herr"] else None,
               heading_max_deg=round(float(np.nanmax(h)), 2) if res["herr"] else None,
               pos_p95_cm=round(float(np.nanpercentile(e, 95)), 2) if res["perr"] else None, robots={})
    for n, pr in res["per"].items():
        fl, fb = pr["first_lights"], pr["first_blink"]
        out["robots"][n] = dict(
            lights_first_s=None if fl is None else round(fl, 3),
            lights_first_frames=pr["first_frames"],
            blink_first_s=None if fb is None else round(fb, 3),
            named_share=round(pr["named"] / pr["frames"], 3) if pr["frames"] else None)
    return out


# ---------------------------------------------------------------- scenes
def _codes(mode, kw, dock, wide, strip):
    c = strip if mode == "strip" else (wide if len(kw.get("family", {}).get("side_cm", (0,))) >= 3 else dock)
    return dict(zip(("Left", "Center"), c))


def scene_pass(mode="l3", scale=3.4, seed=0, codes=None, seconds=8.0, **kw):
    """Two robots driving towards each other in neighbouring lanes (passing 12 cm apart), turning, blinking."""
    a = SW.Robot("Left", 30, 50, 0, code=1, t_end=seconds + 1)
    a.drive(0.3, 0)
    a.drive(3.0, 18)
    a.drive(1.5, 0, 120)
    a.drive(seconds, 10, 30)
    a.add_blink(0.3, seconds + 1, phase=0.3)
    b = SW.Robot("Center", 100, 62, 180, code=2, t_end=seconds + 1)
    b.drive(0.3, 0)
    b.drive(3.0, 15)
    b.drive(seconds, 0, -90)
    b.add_blink(0.3, seconds + 1, phase=0.41)
    codes = codes or _codes(mode, kw, ("S0L", "S0R"), ("S0L", "S2L"), ("1", "3"))
    return Scene([a, b], codes, mode=mode, scale=scale, seconds=seconds, seed=seed, **kw)


def scene_close(mode="l3", scale=3.4, seed=0, codes=None, seconds=6.0, **kw):
    """Two robots side by side, bodies 1 cm apart (centres 10.5 cm), driving in parallel then turning in place."""
    a = SW.Robot("Left", 40, 50, 0, code=1, t_end=seconds + 1)
    a.drive(0.5, 0)
    a.drive(2.5, 12)
    a.drive(seconds, 0, 60)
    a.add_blink(0.3, seconds + 1, phase=0.3)
    b = SW.Robot("Center", 40, 60.5, 0, code=2, t_end=seconds + 1)
    b.drive(0.5, 0)
    b.drive(2.5, 12)
    b.drive(seconds, 0, -60)
    b.add_blink(0.3, seconds + 1, phase=0.35)
    codes = codes or _codes(mode, kw, ("S0R", "S0L"), ("S1L", "S1R"), ("2", "4"))
    return Scene([a, b], codes, mode=mode, scale=scale, seconds=seconds, seed=seed, **kw)


def scene_single(mode="l3", scale=3.4, seed=0, code=None, seconds=5.0, x=61.0, y=61.0, **kw):
    a = SW.Robot("Left", x, y, 30, code=1, t_end=seconds + 1)
    a.drive(0.5, 0)
    a.drive(seconds, 8, 40)
    a.add_blink(0.3, seconds + 1, phase=0.3)
    return Scene([a], {"Left": code or _codes(mode, kw, ("S0L",), ("S1L",), ("2",))["Left"]}, mode=mode, scale=scale,
                 seconds=seconds, seed=seed, **kw)


def scene_three(mode="l3", scale=3.4, seed=0, codes=None, seconds=8.0, **kw):
    """Three robots with the default name table (printed holder: Left S0L, Center S0R, Right S0L/0011 - Left
    and Right differ only by the side-LED sequence), Right passing 12 cm from Left."""
    a = SW.Robot("Left", 30, 40, 0, code=1, t_end=seconds + 1)
    a.drive(0.3, 0)
    a.drive(seconds, 10, 15)
    a.add_blink(0.3, seconds + 1, phase=0.3)
    b = SW.Robot("Center", 90, 90, -90, code=2, t_end=seconds + 1)
    b.drive(0.5, 0)
    b.drive(seconds, 0, 50)
    b.add_blink(0.3, seconds + 1, phase=0.37)
    c = SW.Robot("Right", 100, 52, 180, code=3, t_end=seconds + 1)
    c.drive(0.3, 0)
    c.drive(4.0, 14)
    c.drive(seconds, 0, -70)
    c.add_blink(0.3, seconds + 1, phase=0.45)
    fam = PT.family(mode, **kw.get("family", {}))
    codes = codes or PT.parse_names("", fam)
    return Scene([a, b, c], codes, mode=mode, scale=scale, seconds=seconds, seed=seed, **kw)


SCENES = dict(passing=scene_pass, close=scene_close, single=scene_single, three=scene_three)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="passing", choices=sorted(SCENES))
    ap.add_argument("--mode", default="l3", choices=("l3", "strip"))
    ap.add_argument("--scale", type=float, default=3.4, help="px per cm on the floor (real camera: 3.4)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--seconds", type=float)
    ap.add_argument("--no-blink", action="store_true", help="robots' own blink LEDs dark (and no blink comparison)")
    ap.add_argument("--wide", action="store_true", help="l3: the longer L (patterns.WIDE_L3) instead of the printed holder")
    args = ap.parse_args(argv)
    kw = dict(mode=args.mode, scale=args.scale, seed=args.seed, blink=not args.no_blink)
    if args.wide:
        kw["family"] = dict(PT.WIDE_L3)
    if args.seconds:
        kw["seconds"] = args.seconds
    sc = SCENES[args.scene](**kw)
    res = run(sc, with_blink=not args.no_blink)
    print(json.dumps(dict(summary(res), stats=res["stats"]), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
