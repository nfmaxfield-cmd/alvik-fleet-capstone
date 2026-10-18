#!/usr/bin/env python3
"""Small synthetic overhead recordings with known truth, for testing the
blink tools without robots.

The recording folder looks like camrec.py's (video.avi MJPG, frames.csv,
meta.json, floor_calibration.json beside it), plus truth_lights.csv:

    frame,t_wall,robot,x_px,y_px,led,cx_px,cy_px,heading_img_deg

x_px,y_px = midpoint of the two LEDs, cx,cy = robot top centre (both as
seen, 4 cm above the floor), led = how lit the LEDs were during the
exposure (0..1).

Optionally a console session folder (runinfo, raw, sync, param, runend,
clock_sync) for the same robots and runs, and a robot-centre detections CSV.

The background is deliberately hostile: blotchy texture, bright paper
patches, tape lines, slow light changes and a drifting shadow, sensor noise;
distractors: a randomly flickering lamp, a 4 Hz flicker, a lamp with 200 ms
pulses at random intervals, a moving white object, a static white spot and
lone 300 ms flashes. Frames are dropped and some arrive late.

    python tools/blink/tests/synthworld.py <out> --scenario clutter
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np
import cv2

W, H = 640, 480
SLOT, LEAD_SLOTS = 0.1, 3    # firmware 2026-10-18a blink.h: a flash every (code + 2) slots


# ------------------------------------------------------------------ camera
class Cam:
    """Floor cm -> image px: scale s, small yaw, nadir (point under the lens)
    at image (u0, v0); points z cm up are pushed away from the nadir by
    hc / (hc - z), as a pinhole camera hc cm up would."""

    def __init__(self, s=4.0, yaw_deg=3.0, nadir=(60.0, 10.0), u0=322.0, v0=236.0, hc=115.0):
        self.s, self.yaw, self.nx, self.ny, self.u0, self.v0, self.hc = s, math.radians(yaw_deg), nadir[0], nadir[1], u0, v0, hc

    def project(self, x, y, z=0.0):
        k = self.hc / (self.hc - z)
        X = self.nx + (np.asarray(x, float) - self.nx) * k
        Y = self.ny + (np.asarray(y, float) - self.ny) * k
        c, sn = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = X - self.nx, Y - self.ny
        return self.u0 + self.s * (c * dx - sn * dy), self.v0 - self.s * (sn * dx + c * dy)

    def H(self):
        s, c, sn = self.s, math.cos(self.yaw), math.sin(self.yaw)
        nx, ny = self.nx, self.ny
        return [[s * c, -s * sn, self.u0 - s * c * nx + s * sn * ny],
                [-s * sn, -s * c, self.v0 + s * sn * nx + s * c * ny],
                [0.0, 0.0, 1.0]]

    def calibration(self):
        return dict(H_floor_to_px=self.H(), image_size=[W, H],
                    camera=dict(height_cm=self.hc, nadir_floor_cm=[self.nx, self.ny]), synthetic=True)

    def img_heading(self, th_floor_deg):
        return th_floor_deg + math.degrees(self.yaw)


# ------------------------------------------------------------------ robots
LED_FWD, LED_LAT, TOP_Z = 2.75, 2.55, 4.0
TRACK, WHEEL_R = 9.0, 1.7


class Robot:
    """A robot following a list of (seconds, v_cms, w_dps) commands from a start
    pose, and an LED timeline built from runs (see add_run)."""

    def __init__(self, name, x, y, th_deg, code=None, t_end=60.0):
        self.name, self.code = name, code
        self.x0, self.y0, self.th0 = x, y, math.radians(th_deg)
        self.cmds = []           # (t0, t1, v, w)
        self.t_cmd = 0.0
        self.led_events = []     # (t, level, colour) sorted; level 0..1 white; colour for idle lights
        self.blinks = []         # (t0, t1, code, phase t0)
        self.holds = []          # (t0, t1): LEDs forced dark (long blocking calls)
        self.freezes = []        # (t0, t1): code blocked, LEDs keep their state (shorter blocking calls)
        self.flashes = []        # (t_on, t_off)
        self.idle = None         # (bgr) colour of the left LED when not running, or None
        self.runs = []           # for the session writer
        self.steps = []          # (t0, t1, step) RAW step numbers
        self._dt = 0.002
        self.t_end = t_end
        self._built = False

    # motion
    def wait(self, s):
        return self.drive(s, 0, 0)

    def drive(self, s, v, w=0.0):
        self.cmds.append((self.t_cmd, self.t_cmd + s, v, w))
        self.t_cmd += s
        return self.t_cmd

    def _build(self):
        n = int(self.t_end / self._dt) + 2
        t = np.arange(n) * self._dt
        v = np.zeros(n)
        w = np.zeros(n)
        for t0, t1, vv, ww in self.cmds:
            m = (t >= t0) & (t < t1)
            v[m], w[m] = vv, ww
        th = self.th0 + np.concatenate([[0], np.cumsum(np.radians(w[:-1]) * self._dt)])
        x = self.x0 + np.concatenate([[0], np.cumsum(v[:-1] * np.cos(th[:-1]) * self._dt)])
        y = self.y0 + np.concatenate([[0], np.cumsum(v[:-1] * np.sin(th[:-1]) * self._dt)])
        vl = v - np.radians(w) * TRACK / 2
        vr = v + np.radians(w) * TRACK / 2
        wl = np.degrees(np.concatenate([[0], np.cumsum(vl[:-1] * self._dt)]) / WHEEL_R)
        wr = np.degrees(np.concatenate([[0], np.cumsum(vr[:-1] * self._dt)]) / WHEEL_R)
        self._t, self._x, self._y, self._th, self._v, self._w, self._wl, self._wr = t, x, y, th, v, w, wl, wr
        self._built = True

    def pose(self, t):
        if not self._built:
            self._build()
        i = int(np.clip(t / self._dt, 0, len(self._t) - 1))
        return self._x[i], self._y[i], self._th[i]

    def state(self, t):
        if not self._built:
            self._build()
        i = int(np.clip(t / self._dt, 0, len(self._t) - 1))
        return dict(v=self._v[i], w=self._w[i], wl=self._wl[i], wr=self._wr[i], x=self._x[i], y=self._y[i],
                    th=self._th[i])

    # lights
    def led(self, t):
        """(white level 0..1, idle colour or None) at time t."""
        for a, b in self.flashes:
            if a <= t < b:
                return 1.0, None
        for a, b, code, p0 in self.blinks:
            if a <= t < b:
                if any(h0 <= t < h1 for h0, h1 in self.holds):
                    return 0.0, None
                for f0, f1 in self.freezes:      # code blocked: the lights keep their state
                    if f0 <= t < f1:
                        t = f0
                        break
                slot = int(math.floor((t - p0) / SLOT))
                if slot < LEAD_SLOTS:
                    return 0.0, None
                return (1.0 if (slot - LEAD_SLOTS) % (code + 2) == 0 else 0.0), None
        for a, b in self.busy():
            if a <= t < b:
                return 0.0, None
        return 0.0, self.idle

    def busy(self):
        return [(r["t0"], r["t_end"]) for r in self.runs]

    def add_blink(self, t0, t1, code=None, phase=None):
        self.blinks.append((t0, t1, code or self.code, t0 if phase is None else phase))

    def add_run(self, t_start, kind, run_id, experiment=None, nudge_cm=2.0, calib_blink_s=3.0, drive_s=0.0,
                drive_v=20.0, turn_dps=0.0, cut_pulses=True, move_v=15.0):
        """Firmware-like logged run starting at t_start (robot must be idle; motion
        commands are appended, so call in time order). kind: calib | straight | turn | plain.
        Returns the end time."""
        assert t_start >= self.t_cmd - 1e-9
        self.wait(t_start - self.t_cmd)
        r = dict(t0=t_start, run=run_id, experiment=experiment or kind, steps=[])
        t = self.wait(0.5)                     # settle
        t = self.wait(0.15)                    # dark
        f_on = t
        t = self.wait(0.3)
        f_off = t
        self.flashes.append((f_on, f_off))
        t = self.wait(0.15)
        r["sync"] = [("start", f_on, f_off)]
        blink0 = t

        def move(cm, steps):
            nonlocal t
            h0 = t
            t = self.wait(0.2)                 # library move() blocks 200 ms before it starts
            dur = abs(cm) / move_v
            t = self.drive(dur, math.copysign(move_v, cm))
            t1 = t
            t = self.wait(0.1)                 # is_target_reached: blocks without going dark first
            if cut_pulses:
                self.holds.append((h0, t1))
                self.freezes.append((t1, t))
            return t

        if kind == "calib":
            s1 = t
            t = self.wait(calib_blink_s)
            self.steps.append((s1, t, 1))
            s2 = t
            move(3.0, 2)
            t = self.wait(0.3)
            self.steps.append((s2, t, 2))
            s3 = t
            move(-3.0, 3)
            t = self.wait(0.3)
            self.steps.append((s3, t, 3))
            s4 = t
            t = self.drive(0.4, 0, 60)
            t = self.drive(0.8, 0, -60)
            t = self.drive(0.4, 0, 60)
            self.steps.append((s4, t, 4))
            s5 = t
            t = self.wait(0.3)
            self.steps.append((s5, t, 5))
        else:
            if kind in ("straight", "turn") and nudge_cm > 0:
                s = t
                move(nudge_cm, -1)
                t = self.wait(0.2)
                move(-nudge_cm, -1)
                t = self.wait(0.3)
                self.steps.append((s, t, -1))
            s = t
            if drive_s:
                t = self.drive(drive_s, drive_v if kind != "turn" else 0, turn_dps)
            t = self.wait(0.5)
            self.steps.append((s, t, 0))
        self.add_blink(blink0, t)
        t = self.wait(0.15)
        e_on = t
        t = self.wait(0.3)
        e_off = t
        self.flashes.append((e_on, e_off))
        t = self.wait(0.15)
        r["sync"].append(("end", e_on, e_off))
        r["t_end"] = t
        r["blink"] = (blink0, e_on - 0.15)
        self.runs.append(r)
        return t


# ------------------------------------------------------------------ distractors
class Lamp:
    """A white spot at a fixed or moving place, with an on/off timeline."""

    def __init__(self, pos, radius, on_fn):
        self.pos, self.radius, self.on_fn = pos, radius, on_fn


def random_flicker(rng, t_end, on=(0.06, 0.4), off=(0.06, 0.6)):
    ev, t, lit = [], 0.0, False
    while t < t_end:
        d = rng.uniform(*(on if lit else off))
        ev.append((t, t + d, lit))
        t += d
        lit = not lit
    return lambda tt: next((1.0 if l else 0.0 for a, b, l in ev if a <= tt < b), 0.0)


def random_pulses(rng, t_end, on_s=0.2, gap=(0.3, 0.9)):
    ev, t = [], 0.3
    while t < t_end:
        ev.append((t, t + on_s))
        t += on_s + rng.uniform(*gap)
    return lambda tt: 1.0 if any(a <= tt < b for a, b in ev) else 0.0


# ------------------------------------------------------------------ rendering
class Scene:
    def __init__(self, cam, robots, lamps, seconds, seed=0, fps=30.0, drop=0.03, late=0.01, exposure=0.015,
                 noise=3.5, clutter=True, t_epoch=0.0):
        self.cam, self.robots, self.lamps, self.seconds = cam, robots, lamps, seconds
        self.rng = np.random.default_rng(seed)
        self.fps, self.drop, self.late, self.exposure, self.noise_sd, self.clutter = fps, drop, late, exposure, noise, clutter
        self.pi_offset = 0.04            # video clock minus laptop clock
        self.t_epoch = t_epoch           # laptop clock at scene time 0
        self.delay = 1 / fps             # frame reaches the Pi one frame after it was taken
        self._bg()

    def _bg(self):
        rng = self.rng
        low = cv2.resize(rng.normal(0, 1, (10, 14)).astype(np.float32), (W, H), interpolation=cv2.INTER_CUBIC)
        mid = cv2.resize(rng.normal(0, 1, (60, 80)).astype(np.float32), (W, H), interpolation=cv2.INTER_LINEAR)
        grain = cv2.GaussianBlur(rng.normal(0, 1, (H, W)).astype(np.float32), (0, 0), 0.7)
        base = np.array([120, 128, 136], np.float32)
        img = base + (low * 14 + mid * 10 + grain * 6)[:, :, None]
        img = np.clip(img, 0, 255).astype(np.uint8)
        if self.clutter:
            for _ in range(14):                     # paper scraps, bright
                cx, cy = rng.uniform(0, W), rng.uniform(0, H)
                w, h = rng.uniform(15, 70), rng.uniform(10, 50)
                box = cv2.boxPoints(((cx, cy), (w, h), rng.uniform(0, 180))).astype(np.int32)
                g = int(rng.uniform(195, 245))
                cv2.fillPoly(img, [box], (g, g, g - 5), cv2.LINE_AA)
            for _ in range(10):                     # dark objects
                cx, cy = rng.uniform(0, W), rng.uniform(0, H)
                cv2.circle(img, (int(cx), int(cy)), int(rng.uniform(4, 16)), (40, 45, 50), -1, cv2.LINE_AA)
            for _ in range(6):                      # tape lines, light and dark
                p0 = (int(rng.uniform(0, W)), int(rng.uniform(0, H)))
                p1 = (int(rng.uniform(0, W)), int(rng.uniform(0, H)))
                g = int(rng.choice([45, 230]))
                cv2.line(img, p0, p1, (g, g, g), int(rng.uniform(3, 8)), cv2.LINE_AA)
            for _ in range(60):                     # specks: small bright dots (static)
                cv2.circle(img, (int(rng.uniform(0, W)), int(rng.uniform(0, H))), 1, (235, 235, 235), -1, cv2.LINE_AA)
            # a checkered rug
            x0, y0 = int(rng.uniform(0, W - 160)), int(rng.uniform(0, H - 120))
            for i in range(8):
                for j in range(6):
                    if (i + j) % 2 == 0:
                        cv2.rectangle(img, (x0 + 20 * i, y0 + 20 * j), (x0 + 20 * i + 19, y0 + 20 * j + 19), (210, 205, 200), -1)
        self.bg = img
        yy, xx = np.mgrid[0:2 * H, 0:2 * W].astype(np.float32)
        self.shadow = 1 - 0.35 * np.exp(-(((xx - W) / 110) ** 2 + ((yy - H) / 90) ** 2))
        self.noisebank = []
        for _ in range(4):
            n = rng.normal(0, self.noise_sd, (H + 64, W + 64, 3))
            self.noisebank.append((np.clip(n, 0, 255).astype(np.uint8), np.clip(-n, 0, 255).astype(np.uint8)))

    def frame_times(self):
        rng = self.rng
        n = int(self.seconds * self.fps)
        tc = np.arange(n) / self.fps + rng.normal(0, 0.0005, n) + 0.01
        keep = rng.random(n) >= self.drop
        tc = tc[keep]
        delay = self.delay + np.abs(rng.normal(0, 0.003, len(tc)))
        late = rng.random(len(tc)) < self.late
        delay[late] += rng.uniform(0.03, 0.08, late.sum())    # a late frame...
        tw = self.t_epoch + tc + delay + self.pi_offset
        tw = np.maximum.accumulate(tw + np.arange(len(tw)) * 1e-6)
        return tc, tw

    def _robot_poly(self, x, y, th, pts):
        c, s = math.cos(th), math.sin(th)
        X = x + pts[:, 0] * c - pts[:, 1] * s
        Y = y + pts[:, 0] * s + pts[:, 1] * c
        u, v = self.cam.project(X, Y, TOP_Z)
        return np.round(np.stack([u, v], 1) * 16).astype(np.int32)

    def draw_robot(self, img, r, tc, gain):
        x, y, th = r.pose(tc)
        L = 9.5 / 2
        a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        body = np.stack([L * np.sign(np.cos(a)) * np.abs(np.cos(a)) ** 0.3, L * np.sign(np.sin(a)) * np.abs(np.sin(a)) ** 0.3], 1)
        top = body * 0.85
        wheel = np.array([[-1.7, 3.9], [1.7, 3.9], [1.7, 4.9], [-1.7, 4.9]])
        g = gain
        cv2.fillPoly(img, [self._robot_poly(x + 0.4, y - 0.4, th, body * 1.04)], (int(85 * g), int(92 * g), int(98 * g)), cv2.LINE_AA, 4)
        for sgn in (1, -1):
            cv2.fillPoly(img, [self._robot_poly(x, y, th, wheel * [1, sgn])], (30, 30, 32), cv2.LINE_AA, 4)
        cv2.fillPoly(img, [self._robot_poly(x, y, th, body)], (int(150 * g), int(152 * g), int(155 * g)), cv2.LINE_AA, 4)
        cv2.fillPoly(img, [self._robot_poly(x, y, th, top)], (int(205 * g), int(207 * g), int(210 * g)), cv2.LINE_AA, 4)
        cv2.fillPoly(img, [self._robot_poly(x, y, th, np.array([[3.7, -1.8], [4.6, -1.8], [4.6, 1.8], [3.7, 1.8]]))],
                     (35, 35, 40), cv2.LINE_AA, 4)
        # LEDs: exposure-averaged level
        ts = tc - self.exposure * (np.arange(5) + 0.5) / 5
        lv, col = 0.0, None
        for t in ts:
            l, c = r.led(t)
            lv += l / 5
            col = col or c
        c, s = math.cos(th), math.sin(th)
        for k, sgn in enumerate((1, -1)):
            fx, fy = LED_FWD, sgn * LED_LAT
            u, v = self.cam.project(x + fx * c - fy * s, y + fx * s + fy * c, TOP_Z)
            p = (int(round(float(u) * 16)), int(round(float(v) * 16)))
            off = np.array([65, 66, 70.0]) * g
            if col is not None and k == 0 and lv < 0.01:
                core = np.array(col, float)
                cv2.circle(img, p, int(2.4 * 16), tuple(float(q) for q in 0.5 * core + 0.5 * off), -1, cv2.LINE_AA, 4)
                cv2.circle(img, p, int(1.6 * 16), tuple(float(q) for q in core), -1, cv2.LINE_AA, 4)
                continue
            core = off * (1 - lv) + 255 * lv
            if lv > 0.01:
                glow = 0.55 * core + 0.45 * np.array([205, 207, 210.0]) * g
                cv2.circle(img, p, int(3.2 * 16), tuple(float(q) for q in glow), -1, cv2.LINE_AA, 4)
            else:
                cv2.circle(img, p, int(2.4 * 16), tuple(float(q) for q in off * 1.2), -1, cv2.LINE_AA, 4)
            cv2.circle(img, p, int(1.8 * 16), tuple(float(q) for q in core), -1, cv2.LINE_AA, 4)
        return x, y, th, lv

    def render(self, folder, truth_dir=None, detections=None):
        os.makedirs(folder, exist_ok=True)
        truth_dir = truth_dir or folder
        tc_all, tw_all = self.frame_times()
        vw = cv2.VideoWriter(os.path.join(folder, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), self.fps, (W, H))
        ff = open(os.path.join(folder, "frames.csv"), "w", newline="")
        fw = csv.writer(ff)
        fw.writerow(["frame", "t_wall", "t_mono", "cam_ms"])
        tf = open(os.path.join(truth_dir, "truth_lights.csv"), "w", newline="")
        tw = csv.writer(tf)
        tw.writerow(["frame", "t_wall", "robot", "x_px", "y_px", "led", "cx_px", "cy_px", "heading_img_deg"])
        df = None
        if detections:
            df = open(detections, "w", newline="")
            dw = csv.writer(df)
            dw.writerow(["frame", "t_wall", "x_px", "y_px", "score"])
        rng = self.rng
        for fi, (tc, twl) in enumerate(zip(tc_all, tw_all)):
            img = self.bg.copy()
            g = 1 + 0.08 * math.sin(2 * math.pi * tc / 7.0)
            ox = int((W / 2) + (W / 2) * math.sin(tc / 4.0))
            oy = int((H / 2) + (H / 3) * math.cos(tc / 5.0))
            gain = self.shadow[oy:oy + H, ox:ox + W] * g
            img = np.clip(img.astype(np.float32) * gain[:, :, None], 0, 255).astype(np.uint8)
            for r in self.robots:
                x, y, th, lv = self.draw_robot(img, r, tc, float(g))
                c, s = math.cos(th), math.sin(th)
                lu, lvv = self.cam.project(x + LED_FWD * c, y + LED_FWD * s, TOP_Z)
                cu, cv_ = self.cam.project(x, y, TOP_Z)
                hd = self.cam.img_heading(math.degrees(th))
                if 0 <= cu < W and 0 <= cv_ < H:
                    tw.writerow([fi, f"{twl:.4f}", r.name, f"{float(lu):.2f}", f"{float(lvv):.2f}", f"{lv:.3f}",
                                 f"{float(cu):.2f}", f"{float(cv_):.2f}", f"{hd:.2f}"])
                    if df and rng.random() > 0.05:
                        dw.writerow([fi, f"{twl:.4f}", f"{float(cu) + rng.normal(0, 1.0):.1f}",
                                     f"{float(cv_) + rng.normal(0, 1.0):.1f}", f"{rng.uniform(0.8, 0.95):.2f}"])
            for lamp in self.lamps:
                lv = float(np.mean([lamp.on_fn(tc - self.exposure * (k + 0.5) / 4) for k in range(4)]))
                pos = lamp.pos(tc) if callable(lamp.pos) else lamp.pos
                if lv > 0.01:
                    p = (int(pos[0] * 16), int(pos[1] * 16))
                    cv2.circle(img, p, int(lamp.radius * 1.7 * 16), tuple([float(160 + 60 * lv)] * 3), -1, cv2.LINE_AA, 4)
                    cv2.circle(img, p, int(lamp.radius * 16), tuple([float(140 + 115 * lv)] * 3), -1, cv2.LINE_AA, 4)
            if df and rng.random() < 0.02:
                dw.writerow([fi, f"{twl:.4f}", f"{rng.uniform(0, W):.1f}", f"{rng.uniform(0, H):.1f}", "0.40"])
            pn, nn = self.noisebank[int(rng.integers(4))]
            dy, dx = (int(a) for a in rng.integers(0, 64, 2))
            cv2.add(img, pn[dy:dy + H, dx:dx + W], dst=img)
            cv2.subtract(img, nn[dy:dy + H, dx:dx + W], dst=img)
            vw.write(img)
            fw.writerow([fi, f"{twl:.4f}", f"{twl - tw_all[0]:.4f}", ""])
        vw.release()
        ff.close()
        tf.close()
        if df:
            df.close()
        with open(os.path.join(folder, "meta.json"), "w") as f:
            json.dump(dict(device="synthetic", width=W, height=H, fps_requested=30, fps_reported=self.fps,
                           host="synthworld", label="synthetic"), f)
        with open(os.path.join(folder, "floor_calibration.json"), "w") as f:
            json.dump(self.cam.calibration(), f, indent=2)
        return len(tc_all)


# ------------------------------------------------------------------ console session
RAW_COLS = ["run", "ms", "step", "wl_deg", "wr_deg", "wl_rpm", "wr_rpm", "v_cms", "w_dps", "x_cm", "y_cm", "th_deg",
            "ax", "ay", "az", "gx", "gy", "gz", "roll_deg", "pitch_deg", "yaw_deg", "dL_cm", "dCL_cm", "dC_cm",
            "dCR_cm", "dR_cm", "dtop_cm", "dbot_cm", "lineL", "lineC", "lineR", "battery", "charging", "lifted"]


def write_session(folder, robots, t_epoch, seconds, seed=0, boots=None):
    """Console log folder for the robots' runs. Laptop time = t_epoch + scene time;
    robot ms = (scene time - t_boot) * 1000 with t_boot = -100 s - k."""
    rng = np.random.default_rng(seed + 99)
    os.makedirs(folder, exist_ok=True)
    files = {}

    def wr(name, header, row):
        if name not in files:
            f = open(os.path.join(folder, name + ".csv"), "w", newline="")
            w = csv.writer(f)
            w.writerow(["robot", "t_recv"] + header)
            files[name] = (f, w)
        files[name][1].writerow(row)

    cs_rows = []
    for k, r in enumerate(robots):
        t_boot = -100.0 - 7.3 * k
        boot = (boots or {}).get(r.name, "%04X" % (0xA100 + 0x111 * k))
        ms = lambda t: (t - t_boot) * 1000.0
        for n, run in enumerate(r.runs, 1):
            rid = run["run"]
            t0 = run["t0"]
            wr("runinfo", ["run", "experiment", "fw_version", "motor_fw", "ms_start", "battery", "charging", "wifi_dbm",
                           "route", "yaw_sign", "odo_sign"],
               [r.name, f"{t_epoch + t0 + 0.01:.4f}", rid, run["experiment"], "2026-10-17e", "1.0.3", int(ms(t0)), 90, 0,
                -50, "", 1, 1])
            for pname, val in (("sync_ms", 300), ("start_nudge_cm", 2), ("calib_blink_ms", 3000), ("blink_codes", 1)):
                wr("param", ["run", "name", "value"], [r.name, f"{t_epoch + t0 + 0.02:.4f}", rid, pname, f"{val:.4f}"])
            for tag, a, b in run["sync"]:
                wr("sync", ["run", "tag", "ms_on", "ms_off"], [r.name, f"{t_epoch + b + 0.01:.4f}", rid, tag,
                                                              int(round(ms(a))), int(round(ms(b)))])
            nrows = 0
            for t in np.arange(t0, run["t_end"], 0.02):
                st = r.state(t)
                step = 0
                for a, b, sv in r.steps:
                    if a <= t < b and a >= t0 - 1e-6 and b <= run["t_end"] + 1e-6:
                        step = sv
                row = dict(run=rid, ms=int(round(ms(t))), step=step, wl_deg=f"{st['wl']:.2f}", wr_deg=f"{st['wr']:.2f}",
                           v_cms=f"{st['v']:.2f}", w_dps=f"{st['w']:.2f}", az="1.00", battery=90, lifted=0)
                wr("raw", RAW_COLS, [r.name, f"{t_epoch + t + 0.008:.4f}"] + [row.get(c, 0) for c in RAW_COLS])
                nrows += 1
            wr("runend", ["run", "result", "ms_end", "duration_ms", "battery", "wifi_dbm", "raw_rows_sent", "packets_sent"],
               [r.name, f"{t_epoch + run['t_end'] + 0.01:.4f}", rid, "done", int(ms(run["t_end"])),
                int(1000 * (run["t_end"] - t0)), 89, -50, nrows, nrows + 50])
        tok = 0
        for tb in np.arange(-5.0, seconds + 5, 4.0):
            for j in range(3):
                tok += 1
                ts = tb + 0.05 * j
                rtt = rng.uniform(0.004, 0.02)
                mid = ts + rtt / 2
                cs_rows.append([r.name, boot, tok, f"{t_epoch + ts:.6f}", f"{t_epoch + ts + rtt:.6f}",
                                int(round(ms(mid + rng.normal(0, 0.001)))), f"{rtt * 1000:.1f}", ""])
    for f, _ in files.values():
        f.close()
    with open(os.path.join(folder, "clock_sync.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "boot", "token", "t_send", "t_recv", "robot_ms", "rtt_ms", "offset_s"])
        w.writerows(cs_rows)
    with open(os.path.join(folder, "session.json"), "w") as f:
        json.dump(dict(start="synthetic", start_unix=t_epoch, laptop="synthworld"), f)


# ------------------------------------------------------------------ calibration case
T_EPOCH = 1.79e9


def calib_case(out, seed=0):
    """Three robots: Left and Center run 'calibrate' 0.4 s apart (their rolls
    overlap in time), Right runs a straight run with its start nudge while
    they are still blinking, Left later a turn run with a nudge.
    Writes out/recording, out/session, out/detections.csv, out/truth_events.json."""
    rng = np.random.default_rng(seed)
    cam = Cam()
    T = 22.0
    L = Robot("Left", 35, 5, 30, code=1, t_end=T + 1)
    C = Robot("Center", 95, 25, 200, code=2, t_end=T + 1)
    R = Robot("Right", 70, -28, 100, code=3, t_end=T + 1)
    L.idle = (40, 220, 40)
    C.idle = (40, 220, 40)
    R.idle = (220, 220, 40)
    ev = []
    t = L.add_run(1.0, "calib", "A100-1")
    ev.append(dict(run="A100-1", robot="Left", kind="calib", heading_floor=30.0))
    C.add_run(1.4, "calib", "A211-1")
    ev.append(dict(run="A211-1", robot="Center", kind="calib", heading_floor=200.0 - 360))
    R.add_run(3.0, "straight", "A322-1", drive_s=1.5, drive_v=15)
    ev.append(dict(run="A322-1", robot="Right", kind="nudge", heading_floor=100.0))
    th_l = 30.0          # the calib wiggle turns +24, -48, +24 deg: back to 30
    L.add_run(max(t + 1.5, 12.0), "turn", "A100-2", drive_s=2.0, turn_dps=45)
    ev.append(dict(run="A100-2", robot="Left", kind="nudge", heading_floor=th_l))
    for r in (L, C, R):
        r.drive(T, 0)
    robots = [L, C, R]
    lamps = distractor_lamps(rng, T, dict(flicker=(560, 420), static=(300, 60)))
    sc = Scene(cam, robots, lamps, T, seed=seed, t_epoch=T_EPOCH)
    rec = os.path.join(out, "recording")
    sc.render(rec, truth_dir=out, detections=os.path.join(out, "detections.csv"))
    write_session(os.path.join(out, "session"), robots, T_EPOCH, T, seed=seed,
                  boots={"Left": "A100", "Center": "A211", "Right": "A322"})
    for e in ev:
        e["heading_img"] = cam.img_heading(e["heading_floor"])
        e["led_fwd_cm"], e["led_left_cm"] = LED_FWD, 0.0
    with open(os.path.join(out, "truth_events.json"), "w") as f:
        json.dump(dict(events=ev, video_offset=sc.pi_offset + sc.delay), f, indent=2)
    return ev


# ------------------------------------------------------------------ scenarios
def distractor_lamps(rng, seconds, where):
    """where: dict name -> (x, y) image px for each distractor."""
    lamps = []
    if "flicker" in where:
        lamps.append(Lamp(where["flicker"], 2.2, random_flicker(rng, seconds)))
    if "flicker4hz" in where:
        lamps.append(Lamp(where["flicker4hz"], 2.2, lambda t: 1.0 if (t * 4) % 1 < 0.5 else 0.0))
    if "pulses" in where:
        lamps.append(Lamp(where["pulses"], 2.0, random_pulses(rng, seconds)))
    if "static" in where:
        lamps.append(Lamp(where["static"], 2.0, lambda t: 1.0))
    if "moving" in where:
        (u0, v0), (u1, v1) = where["moving"]
        lamps.append(Lamp(lambda t: (u0 + (u1 - u0) * ((t / seconds) % 1), v0 + (v1 - v0) * ((t / seconds) % 1)), 4.0,
                          lambda t: 1.0))
    if "flash" in where:
        u, v = where["flash"]
        times = [3.0, 9.5, 16.0]
        lamps.append(Lamp((u, v), 2.2, lambda t: 1.0 if any(a <= t < a + 0.3 for a in times) else 0.0))
    return lamps


def scenario(name, seed=0, seconds=None):
    """(Scene, robots) for a named test scenario."""
    rng = np.random.default_rng(seed)
    cam = Cam()
    if name == "clutter":
        T = seconds or 24.0
        # (paths chosen so the robots never overlap: drawn on top of each other they would hide their lights)
        a = Robot("Left", 5, -40, 0, code=1, t_end=T + 1)     # back and forth at 30 cm/s, turning on the spot
        a.wait(0.5)
        for _ in range(5):
            a.drive(2.5, 30, 0)
            a.drive(1.0, 0, 180)
        a.drive(T, 0)
        a.add_blink(0.4, T, phase=0.4)
        b = Robot("Center", 35, 1.4, 0, code=2, t_end=T + 1)   # slow left arcs with stops
        for _ in range(6):
            b.drive(2.5, 10, 20)
            b.wait(1.5)
        b.drive(T, 0)
        b.add_blink(1.0, T, phase=1.13)
        c = Robot("Right", 120, 40, 90, code=3, t_end=T + 1)   # still, then turning on the spot
        c.wait(8)
        c.drive(8, 0, 45)
        c.drive(T, 0)
        c.add_blink(0.2, T, phase=0.71)
        c.holds += [(5.0, 5.3), (11.12, 11.4), (14.0, 14.25)]   # blocking calls: pulses cut short
        c.freezes += [(k * 0.97 + 0.35, k * 0.97 + 0.465) for k in range(25)]   # ...or stretched by 115 ms
        b.freezes += [(k * 1.31 + 0.6, k * 1.31 + 0.7) for k in range(18)]
        robots = [a, b, c]
        lamps = distractor_lamps(rng, T, dict(flicker=(80, 400), flicker4hz=(560, 60), pulses=(600, 420),
                                              static=(300, 60), moving=((20, 300), (620, 120)), flash=(150, 120)))
        return Scene(cam, robots, lamps, T, seed=seed), robots
    if name == "crossing":
        T = seconds or 18.0
        a = Robot("Left", 10, 10, 0, code=1, t_end=T + 1)
        a.wait(1.0)
        a.drive(4.0, 25)                 # x 10 -> 110 along y = 10 (crosses B's path at t ~ 3)
        a.wait(0.8)
        a.drive(0.8, 0, 225)             # turn around
        a.drive(4.0, 22)                 # back to x = 22 (crosses B again at t ~ 8.4)
        a.wait(0.2)
        a.drive(0.8, 0, 225)
        a.wait(0.4)
        a.drive(5.0, 20)                 # head-on pass with B in the next lane (11 cm) at t ~ 13
        a.drive(T, 0)
        a.add_blink(0.3, T, phase=0.3)
        b = Robot("Center", 60, 55, -90, code=2, t_end=T + 1)
        b.wait(1.6)
        b.drive(3.6, 25)                 # y 55 -> -35 along x = 60
        b.drive(0.8, 0, 225)
        b.wait(0.6)
        b.drive(2.24, 25)                # back up to y = 21
        b.drive(0.8, 0, 112.5)           # face -x
        b.wait(2.36)
        b.drive(4.0, 20)
        b.drive(T, 0)
        b.add_blink(0.3, T, phase=0.42)  # phases only 0.12 s apart: timing alone can't separate them
        robots = [a, b]
        lamps = distractor_lamps(rng, T, dict(flicker=(500, 400), static=(100, 380), moving=((600, 20), (40, 460))))
        return Scene(cam, robots, lamps, T, seed=seed), robots
    if name == "distractors":
        T = seconds or 16.0
        lamps = distractor_lamps(rng, T, dict(flicker=(80, 400), flicker4hz=(560, 60), pulses=(600, 420),
                                              static=(300, 60), moving=((20, 300), (620, 120)), flash=(150, 120)))
        lamps += distractor_lamps(np.random.default_rng(seed + 5), T, dict(flicker=(330, 250), pulses=(200, 200)))
        idle = Robot("Idle", 60, 10, 30, t_end=T + 1)       # idle robot: green left LED, steady
        idle.idle = (40, 220, 40)
        idle.drive(T, 0)
        white = Robot("White", 20, -20, 120, t_end=T + 1)   # idle after a demo run: steady WHITE left LED
        white.idle = (255, 255, 255)
        white.drive(T, 0)
        return Scene(cam, [idle, white], lamps, T, seed=seed), []
    raise ValueError(name)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--scenario", default="clutter")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)
    if args.scenario == "calib":
        calib_case(args.out, args.seed)
        print(f"calibration case -> {args.out}")
        return 0
    sc, _ = scenario(args.scenario, args.seed)
    n = sc.render(args.out)
    print(f"{n} frames -> {args.out}")


if __name__ == "__main__":
    sys.exit(main())
