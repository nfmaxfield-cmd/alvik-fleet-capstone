"""Simulated overhead camera (the Pi running tools/pi/camrec.py).

Camera model: a pinhole camera `height_cm` above the floor looking (almost)
straight down. Image +x runs along floor +x, image +y (down) along floor -y,
so a heading measured counter-clockwise on the floor is also counter-clockwise
on screen. A small yaw and tilt make it slightly imperfect, like a real mount.
Points above the floor (the robot's top, a hand) are projected from their
height, so they look slightly bigger and further from the image centre.

What is drawn: a plain floor (texture, uneven light, vignette), the tape axes,
each robot seen from above (rounded body, darker wheels at the sides, lighter
top, dark front sensor window, two LEDs near the front showing the colours the
firmware sets, white during the sync flashes), hands that pick robots up and
put them down, random hands reaching in, and now and then a box.

Clocks: a frame is TAKEN at true (laptop) time tc. The Pi stamps it when it
arrives, about one frame later, on its own clock:
    t_wall = tc + capture delay + pi_clock_offset_s
The LEDs' colour in a frame is their average over the exposure before tc.
"""
import bisect
import csv
import json
import math
import os
import queue
import threading

import cv2
import numpy as np


# ------------------------------------------------------------------ geometry
class CameraGeometry:
    def __init__(self, c):
        self.c = c
        self.W, self.H = c.width, c.height
        self.f = c.width / c.floor_width_cm * c.height_cm          # focal length, px
        self.cx, self.cy = c.width / 2.0, c.height / 2.0
        yaw = math.radians(c.yaw_deg)
        ta, tb = math.radians(c.tilt_deg[0]), math.radians(c.tilt_deg[1])
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        Rx = np.array([[1, 0, 0], [0, math.cos(ta), -math.sin(ta)], [0, math.sin(ta), math.cos(ta)]])
        Ry = np.array([[math.cos(tb), 0, math.sin(tb)], [0, 1, 0], [-math.sin(tb), 0, math.cos(tb)]])
        self.R = Rz @ Rx @ Ry
        self.K = np.array([[self.f, 0, self.cx], [0, self.f, self.cy], [0, 0, 1.0]])
        X0, Y0 = c.center_floor_cm
        self.X0, self.Y0, self.Hc = X0, Y0, c.height_cm
        # floor (z = 0) -> undistorted pixels: H = K R [[1,0,-X0],[0,-1,Y0],[0,0,Hc]]
        M = np.array([[1, 0, -X0], [0, -1, Y0], [0, 0, self.Hc]], dtype=float)
        Hm = self.K @ self.R @ M
        self.homography = Hm / Hm[2, 2]

    def project(self, X, Y, Z=0.0):
        """Floor coordinates (cm, numbers or arrays; Z = height above floor) -> pixel u, v."""
        R = self.R
        dx = np.asarray(X, float) - self.X0
        dy = -(np.asarray(Y, float) - self.Y0)
        dz = self.Hc - np.asarray(Z, float)
        px = R[0, 0] * dx + R[0, 1] * dy + R[0, 2] * dz
        py = R[1, 0] * dx + R[1, 1] * dy + R[1, 2] * dz
        pz = R[2, 0] * dx + R[2, 1] * dy + R[2, 2] * dz
        xn, yn = px / pz, py / pz
        if self.c.k1:
            fac = 1 + self.c.k1 * (xn * xn + yn * yn)
            xn, yn = xn * fac, yn * fac
        return self.f * xn + self.cx, self.f * yn + self.cy

    def calibration(self):
        return dict(
            note="floor cm -> image px. [u, v, w] = H @ [x_cm, y_cm, 1]; u/w, v/w are pixels (origin top-left). "
                 "Exact for points ON the floor when k1 = 0. With k1 != 0, apply the radial distortion after: "
                 "xn = (u - cx)/f, yn = (v - cy)/f, factor = 1 + k1 (xn^2 + yn^2), u' = cx + f xn factor. "
                 "Points above the floor (the robot's top is robot_height_cm up) project further from the centre: "
                 "use the full camera model below for those.",
            homography_floor_cm_to_px=self.homography.tolist(),
            image_size=[self.W, self.H],
            k1=self.c.k1, f_px=self.f, cx=self.cx, cy=self.cy,
            camera=dict(height_cm=self.Hc, center_floor_cm=[self.X0, self.Y0], yaw_deg=self.c.yaw_deg,
                        tilt_deg=list(self.c.tilt_deg), R=self.R.tolist(),
                        model="p_cam = R @ [x - X0, -(y - Y0), height_cm - z]; u = f p_cam.x/p_cam.z + cx; "
                              "v = f p_cam.y/p_cam.z + cy"),
            px_per_cm_at_floor=self.f / self.Hc,
            simulated=True)


# ------------------------------------------------------------------ shapes
def rounded_rect(L, W, r, n=5):
    """Outline of a rounded rectangle centred at 0 (forward = +x, left = +y), cm."""
    pts = []
    for cx, cy, a0 in ((L / 2 - r, W / 2 - r, 0), (-L / 2 + r, W / 2 - r, 90), (-L / 2 + r, -W / 2 + r, 180),
                       (L / 2 - r, -W / 2 + r, 270)):
        for k in range(n + 1):
            a = math.radians(a0 + 90.0 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return np.array(pts)


def rect(f0, f1, l0, l1):
    return np.array([(f0, l0), (f1, l0), (f1, l1), (f0, l1)], float)


def _poly(img, uv, color):
    pts = np.round(np.stack(uv, axis=-1) * 16).astype(np.int32)
    cv2.fillPoly(img, [pts], color, lineType=cv2.LINE_AA, shift=4)


def _scale(color, g):
    return tuple(float(min(255, max(0, c * g))) for c in color)


# ------------------------------------------------------------------ timelines
class RobotTrack:
    """Where one robot is (true pose, laptop time) and what its LEDs show."""

    def __init__(self, robot, model):
        tr = np.array(robot.traj, dtype=float)
        t = np.array([robot.t_at(ms) for ms in tr[:, 0]])
        order = np.argsort(t, kind="stable")
        self.t, self.x, self.y, self.th, self.lift = t[order], tr[order, 1], tr[order, 2], tr[order, 3], tr[order, 4]
        self.name = robot.name
        self.model = model
        self.led_t = [robot.t_at(ms) for ms, _, _ in robot.leds]
        self.led_c = [(np.array(l, float), np.array(r, float)) for _, l, r in robot.leds]

    def pose(self, t):
        x = float(np.interp(t, self.t, self.x))
        y = float(np.interp(t, self.t, self.y))
        th = float(np.interp(t, self.t, self.th))
        i = max(0, int(np.searchsorted(self.t, t, side="right")) - 1)
        return x, y, th, bool(self.lift[i])

    def led(self, t0, t1):
        """Average (left, right) LED colour (0..1 rgb) over [t0, t1]."""
        i = max(0, bisect.bisect_right(self.led_t, t0) - 1)
        accL, accR = np.zeros(3), np.zeros(3)
        t = t0
        while t < t1:
            nxt = self.led_t[i + 1] if i + 1 < len(self.led_t) else float("inf")
            seg = min(nxt, t1) - t
            accL += self.led_c[i][0] * seg
            accR += self.led_c[i][1] * seg
            t = min(nxt, t1)
            i += 1
            if i >= len(self.led_t):
                i = len(self.led_t) - 1
                accL += self.led_c[i][0] * (t1 - t)
                accR += self.led_c[i][1] * (t1 - t)
                break
        span = max(t1 - t0, 1e-9)
        return accL / span, accR / span


class Hand:
    """A hand reaching in from the operator's side (image bottom), to a floor point
    or following a carried robot, then leaving."""

    def __init__(self, t_in, t_out, target, enter_s=0.7, angle=0.0, z=14.0):
        self.t_in, self.t_out, self.target, self.enter_s, self.angle, self.z = t_in, t_out, target, enter_s, angle, z

    def active(self, t):
        return self.t_in - self.enter_s <= t <= self.t_out + self.enter_s

    def palm(self, t):
        x, y = self.target(min(max(t, self.t_in), self.t_out))
        if t < self.t_in:
            f = (self.t_in - t) / self.enter_s
        elif t > self.t_out:
            f = (t - self.t_out) / self.enter_s
        else:
            f = 0.0
        # come from 70 cm towards the operator (floor -y, the bottom of the image)
        a = math.radians(self.angle)
        return x + f * 70 * math.sin(a), y - f * 70 * math.cos(a), self.z


class Box:
    def __init__(self, t0, t1, x, y, ang):
        self.t0, self.t1, self.x, self.y, self.ang = t0, t1, x, y, ang


# ------------------------------------------------------------------ renderer
class Renderer:
    def __init__(self, cfg, geom, rng):
        self.cfg, self.g, self.rng = cfg, geom, rng
        c = cfg.camera
        W, H = c.width, c.height
        # floor: warm grey with blotchy texture and fine grain
        low = cv2.resize(rng.normal(0, 1, (12, 16)).astype(np.float32), (W, H), interpolation=cv2.INTER_CUBIC)
        grain = cv2.GaussianBlur(rng.normal(0, 1, (H, W)).astype(np.float32), (0, 0), 0.8)
        base = np.array([140, 152, 166], np.float32)              # BGR
        floor = base[None, None, :] + (low * 7 + grain * 4)[:, :, None]
        img = np.clip(floor, 0, 255).astype(np.uint8)
        # tape axes, drawn as many short pieces so lens distortion bends them correctly
        half = cfg.session.tape_width_cm / 2
        for x0, y0, x1, y1 in cfg.session.tape_lines:
            L = math.hypot(x1 - x0, y1 - y0)
            ux, uy = (x1 - x0) / L, (y1 - y0) / L
            nx, ny = -uy, ux
            s = np.linspace(0, L, max(2, int(L / 4)))
            side1 = np.stack([x0 + ux * s + nx * half, y0 + uy * s + ny * half], 1)
            side2 = np.stack([x0 + ux * s - nx * half, y0 + uy * s - ny * half], 1)[::-1]
            pts = np.vstack([side1, side2])
            _poly(img, geom.project(pts[:, 0], pts[:, 1]), (48, 46, 44))
        # light: uneven from one side, darker corners
        yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
        r2 = ((xx - W / 2) / (W / 2)) ** 2 + ((yy - H / 2) / (H / 2)) ** 2
        gain = (1 - c.vignette * r2 / 2) * (1 + c.light_gradient * (0.5 - xx / W) + 0.5 * c.light_gradient * (yy / H - 0.5))
        self.gain = gain
        self.bg = np.clip(img.astype(np.float32) * gain[:, :, None], 0, 255).astype(np.uint8)
        # Sensor noise: making fresh noise for every frame is slow, so cut a window at a
        # random offset out of a few bigger noise images (as good as fresh, pixel by pixel).
        self.noise = []
        self.pad = 96
        for _ in range(4):
            n = rng.normal(0, c.noise_sd, (H + self.pad, W + self.pad, 3))
            self.noise.append((np.clip(n, 0, 255).astype(np.uint8), np.clip(-n, 0, 255).astype(np.uint8)))
        m = cfg.robot
        # the shell is narrower than the robot: the wheels show at its sides
        self.shell_w = m.width_cm - 1.8
        self.body = rounded_rect(m.length_cm, self.shell_w, 1.6)
        self.top = rounded_rect(m.length_cm - 1.4, self.shell_w - 1.4, 1.0)

    def gain_at(self, u, v):
        ui = int(min(self.g.W - 1, max(0, u)))
        vi = int(min(self.g.H - 1, max(0, v)))
        return float(self.gain[vi, ui])

    def _to_floor(self, pts, cx, cy, th):
        c, s = math.cos(th), math.sin(th)
        return cx + pts[:, 0] * c - pts[:, 1] * s, cy + pts[:, 0] * s + pts[:, 1] * c

    def _robot_parts(self):
        """The robot's top view as (outline in robot frame, height above floor, colour), drawn in order."""
        m = self.cfg.robot
        ax = -m.axle_to_center_cm                       # parts are placed around the body centre
        parts = []
        for side in (1, -1):                            # wheels at the sides, on the axle
            parts.append((rect(ax - m.wheel_radius_cm, ax + m.wheel_radius_cm, side * (self.shell_w / 2 - 0.3),
                               side * (m.width_cm / 2)), 2.0 * m.wheel_radius_cm, (32, 32, 34)))
        parts.append((self.body, m.height_cm - 0.4, (168, 170, 172)))          # shell sides
        parts.append((self.top, m.height_cm, (228, 230, 232)))                 # lighter top plate
        parts.append((rect(m.length_cm / 2 - 1.3, m.length_cm / 2 - 0.4, -2.0, 2.0), m.height_cm, (40, 40, 44)))  # sensor window
        return parts

    def draw_robot(self, img, x, y, th, z_lift, ledL, ledR):
        m, g = self.cfg.robot, self.g
        if not hasattr(self, "_parts"):
            self._parts = self._robot_parts()
            self._parts_pts = np.vstack([p for p, _, _ in self._parts] + [self.body * 1.06])
            self._parts_z = np.concatenate([np.full(len(p), z) for p, z, _ in self._parts] + [np.zeros(len(self.body))])
            self._parts_n = [len(p) for p, _, _ in self._parts]
            fwd, lat = m.length_cm / 2 - 2.0, self.shell_w / 2 - 1.3
            self._leds = np.array([[fwd, lat], [fwd, -lat]])
        bx = x + m.axle_to_center_cm * math.cos(th)
        by = y + m.axle_to_center_cm * math.sin(th)
        # every corner of every part, and the LEDs, projected in one go
        pts = np.vstack([self._parts_pts, self._leds])
        z = np.concatenate([self._parts_z + z_lift, np.full(2, m.height_cm + z_lift)])
        X, Y = self._to_floor(pts, bx, by, th)
        n_sh = len(self.body)
        X[-2 - n_sh:-2] += 0.4                        # shadow: offset a little, on the floor
        Y[-2 - n_sh:-2] -= 0.4
        u, v = g.project(X, Y, z)
        uc, vc = g.project(bx, by, m.height_cm + z_lift)
        gain = self.gain_at(float(uc), float(vc))
        if z_lift < 0.5:                              # soft shadow only when standing on the floor
            _poly(img, (u[-2 - n_sh:-2], v[-2 - n_sh:-2]), _scale((105, 114, 124), gain))
        i = 0
        for (_, _, col), n in zip(self._parts, self._parts_n):
            _poly(img, (u[i:i + n], v[i:i + n]), _scale(col, gain))
            i += n
        # LEDs near the front corners: dark lens when off, glowing when on
        scale = g.f / (g.Hc - m.height_cm - z_lift)                       # px per cm at the robot's top
        bezel = _scale((70, 72, 74), gain)
        for k, col in enumerate((ledL, ledR)):
            off = np.array([60, 60, 62], float)
            on = np.array([col[2], col[1], col[0]]) * 255.0                 # rgb -> bgr
            lvl = float(np.max(col))
            core = off * (1 - lvl) + np.maximum(on, off) * lvl
            p = (int(round(float(u[-2 + k]) * 16)), int(round(float(v[-2 + k]) * 16)))
            if lvl > 0.02:
                glow = 0.6 * core + 0.4 * np.array(bezel)
                cv2.circle(img, p, int(0.75 * scale * 16), tuple(float(a) for a in glow), -1, cv2.LINE_AA, 4)
            else:
                cv2.circle(img, p, int(0.75 * scale * 16), bezel, -1, cv2.LINE_AA, 4)   # dark bezel
            cv2.circle(img, p, int(0.45 * scale * 16), tuple(float(a) for a in core), -1, cv2.LINE_AA, 4)

    def draw_hand(self, img, hand, t):
        g = self.g
        px, py, pz = hand.palm(t)
        u, v = g.project(px, py, pz)
        u, v = float(u), float(v)
        scale = g.f / (g.Hc - pz)
        gain = self.gain_at(u, v)
        skin = _scale((112, 142, 196), gain)
        a = math.radians(hand.angle)
        # arm: towards the image bottom (the operator), tilted by the hand's angle
        du, dv = -math.sin(a), math.cos(a)
        end = (u + du * 900, v + dv * 900)
        cv2.line(img, (int(u * 16), int(v * 16)), (int(end[0] * 16), int(end[1] * 16)),
                 _scale((100, 128, 178), gain), int(max(2, 7.5 * scale)), cv2.LINE_AA, 4)
        cv2.ellipse(img, (int(u * 16), int(v * 16)), (int(5.0 * scale * 16), int(5.5 * scale * 16)),
                    -hand.angle, 0, 360, skin, -1, cv2.LINE_AA, 4)
        for k, off in enumerate((-3.0, -1.0, 1.0, 3.0)):              # fingers point away from the arm
            fl = 7.5 if k in (1, 2) else 6.5
            fu = u - du * (4.0 + fl / 2) * scale + dv * off * scale
            fv = v - dv * (4.0 + fl / 2) * scale - du * off * scale
            cv2.ellipse(img, (int(fu * 16), int(fv * 16)), (int(0.95 * scale * 16), int(fl / 2 * scale * 16)),
                        -hand.angle, 0, 360, skin, -1, cv2.LINE_AA, 4)
        return u, v

    def draw_box(self, img, box):
        g = self.g
        X, Y = self._to_floor(rect(-9, 9, -6, 6), box.x, box.y, box.ang)
        u, v = g.project(box.x, box.y, 10.0)
        gain = self.gain_at(float(u), float(v))
        _poly(img, g.project(X, Y, 0.0), _scale((70, 100, 125), gain))           # sides (dark)
        _poly(img, g.project(X, Y, 10.0), _scale((92, 138, 178), gain))          # top
        X2, Y2 = self._to_floor(rect(-9, 9, -0.4, 0.4), box.x, box.y, box.ang)
        _poly(img, g.project(X2, Y2, 10.05), _scale((60, 120, 170), gain))       # tape strip
        return float(u), float(v)

    def add_noise(self, img):
        p, n = self.noise[int(self.rng.integers(len(self.noise)))]
        dy, dx = (int(a) for a in self.rng.integers(0, self.pad, 2))
        H, W = img.shape[:2]
        cv2.add(img, p[dy:dy + H, dx:dx + W], dst=img)
        cv2.subtract(img, n[dy:dy + H, dx:dx + W], dst=img)


# ------------------------------------------------------------------ recording
def frame_times(cfg, t_start, t_end, rng):
    """(tc, t_wall) for every frame that reaches the Pi."""
    c = cfg.camera
    n = int((t_end - t_start) * c.fps)
    k = np.arange(n)
    tc = t_start + k / c.fps * (1 + c.clock_ppm * 1e-6) + rng.normal(0, 0.0003, n)
    keep = rng.random(n) >= c.frame_drop_prob
    tc = tc[keep]
    delay = c.capture_delay_frames / c.fps + np.abs(rng.normal(0, c.capture_jitter_ms / 1000, len(tc)))
    t_wall = tc + delay + c.pi_clock_offset_s
    t_wall = np.maximum.accumulate(t_wall + np.arange(len(t_wall)) * 1e-7)      # arrive in order
    return tc, t_wall


def record(cfg, folder, tracks, hands, boxes, t_start, t_end, events, rng, start_iso, out_root, progress=None):
    """Write video.avi, frames.csv, events.csv, meta.json in `folder`, and
    camera_truth.csv, camera_detections.csv, floor_calibration.json in out_root."""
    c, dm, m = cfg.camera, cfg.detector, cfg.robot
    os.makedirs(folder, exist_ok=True)
    geom = CameraGeometry(c)
    rend = Renderer(cfg, geom, rng)
    tc_all, tw_all = frame_times(cfg, t_start, t_end, rng)
    writer = cv2.VideoWriter(os.path.join(folder, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), c.fps, (c.width, c.height))
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open a Motion-JPEG video writer")
    # JPEG-encoding a frame takes as long as drawing it: do it in a second thread
    # (OpenCV lets other Python code run meanwhile), so the two overlap.
    frames_q = queue.Queue(maxsize=64)

    def write_frames():
        while True:
            im = frames_q.get()
            if im is None:
                break
            writer.write(im)
    writer_thread = threading.Thread(target=write_frames, daemon=True)
    writer_thread.start()
    ff = open(os.path.join(folder, "frames.csv"), "w", newline="")
    fw = csv.writer(ff)
    fw.writerow(["frame", "t_wall", "t_mono"])
    tf = open(os.path.join(out_root, "camera_truth.csv"), "w", newline="")
    tw = csv.writer(tf)
    tw.writerow(["frame", "t_wall", "robot", "x_px", "y_px", "heading_img_deg"])
    df = open(os.path.join(out_root, "camera_detections.csv"), "w", newline="")
    dw = csv.writer(df)
    dw.writerow(["frame", "t_wall", "x_px", "y_px", "score"])
    blind = dm.blind_region_px
    t_mono0 = tw_all[0] if len(tw_all) else 0.0
    exp = c.exposure_ms / 1000
    W, H = c.width, c.height

    def in_blind(u, v):
        return blind is not None and blind[0] <= u <= blind[2] and blind[1] <= v <= blind[3]

    try:
        for fi, (tc, twall) in enumerate(zip(tc_all, tw_all)):
            img = rend.bg.copy()
            for b in boxes:
                if b.t0 <= tc <= b.t1:
                    bu, bv = rend.draw_box(img, b)
                    if 0 <= bu < W and 0 <= bv < H and rng.random() < dm.box_false_pos_prob and not in_blind(bu, bv):
                        dw.writerow([fi, f"{twall:.4f}", f"{bu + rng.normal(0, 4):.1f}", f"{bv + rng.normal(0, 4):.1f}",
                                     f"{rng.uniform(0.2, 0.55):.2f}"])
            seen = []
            for tr in tracks:
                x, y, th, lifted = tr.pose(tc)
                z = 7.0 if lifted else 0.0
                bx = x + m.axle_to_center_cm * math.cos(th)
                by = y + m.axle_to_center_cm * math.sin(th)
                u, v = geom.project(bx, by, m.height_cm + z)
                u, v = float(u), float(v)
                if -60 < u < W + 60 and -60 < v < H + 60:
                    ledL, ledR = tr.led(tc - exp, tc)
                    rend.draw_robot(img, x, y, th, z, ledL, ledR)
                if 0 <= u < W and 0 <= v < H:
                    u2, v2 = geom.project(bx + 5 * math.cos(th), by + 5 * math.sin(th), m.height_cm + z)
                    hd = math.degrees(math.atan2(-(float(v2) - v), float(u2) - u))
                    tw.writerow([fi, f"{twall:.4f}", tr.name, f"{u:.2f}", f"{v:.2f}", f"{hd:.2f}"])
                    seen.append((u, v))
            palms = []
            for h in hands:
                if h.active(tc):
                    palms.append(rend.draw_hand(img, h, tc))
            rend.add_noise(img)
            frames_q.put(img)
            fw.writerow([fi, f"{twall:.4f}", f"{twall - t_mono0:.4f}"])
            # ---- anonymous detections ----
            for u, v in seen:
                occluded = any(math.hypot(u - pu, v - pv) < 45 for pu, pv in palms)
                if in_blind(u, v):
                    continue
                if rng.random() < (dm.miss_prob_occluded if occluded else dm.miss_prob):
                    continue
                score = float(np.clip(rng.normal(0.6 if occluded else 0.9, 0.06), 0.05, 0.99))
                dw.writerow([fi, f"{twall:.4f}", f"{u + rng.normal(0, dm.sigma_px):.1f}",
                             f"{v + rng.normal(0, dm.sigma_px):.1f}", f"{score:.2f}"])
            for pu, pv in palms:
                if 0 <= pu < W and 0 <= pv < H and rng.random() < dm.hand_false_pos_prob and not in_blind(pu, pv):
                    dw.writerow([fi, f"{twall:.4f}", f"{pu + rng.normal(0, 6):.1f}", f"{pv + rng.normal(0, 6):.1f}",
                                 f"{rng.uniform(0.3, 0.75):.2f}"])
            if rng.random() < dm.false_pos_per_frame:
                fu, fv = rng.uniform(0, W), rng.uniform(0, H)
                if not in_blind(fu, fv):
                    dw.writerow([fi, f"{twall:.4f}", f"{fu:.1f}", f"{fv:.1f}", f"{rng.uniform(0.2, 0.6):.2f}"])
            if progress and fi % 3000 == 0:
                progress(fi, len(tc_all))
    finally:
        frames_q.put(None)
        writer_thread.join()
        writer.release()
        ff.close()
        tf.close()
        df.close()

    with open(os.path.join(folder, "events.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_wall", "frame", "text"])
        for t_true, text in events:
            t_pi = t_true + c.pi_clock_offset_s
            w.writerow([f"{t_pi:.3f}", int(np.searchsorted(tw_all, t_pi, side="right")), text])
    with open(os.path.join(folder, "meta.json"), "w") as f:
        json.dump(dict(device="0", width=W, height=H, fps_requested=30, fps_reported=float(c.fps), host="sim-pi",
                       start=start_iso, label="E1",
                       note="t_wall is when the Pi received the frame (SIMULATED recording)"), f, indent=2)
    with open(os.path.join(out_root, "floor_calibration.json"), "w") as f:
        json.dump(geom.calibration(), f, indent=2)
    return len(tc_all)
