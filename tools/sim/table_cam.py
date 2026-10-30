"""A simulated overhead camera over the school table (2026-10-27): a lighter table on a darker floor, the robots
(camera.Renderer), boxes dropped on the table, and a hand and arm reaching in from outside the table to put a box
down. Used by fleet_sim.py --brick-camera so the brick drop goes through the real "what changed" detector
(tools/fleet/brickwatch.py) instead of a perfect report. The camera's model for the detector comes from the same
geometry (true_model), as a perfect floor calibration would give.
"""
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
if HERE not in sys.path:
    sys.path.insert(0, HERE)
if os.path.join(TOOLS, "recognition") not in sys.path:
    sys.path.append(os.path.join(TOOLS, "recognition"))

import camera as SC   # noqa: E402
import config as SCFG  # noqa: E402


class TableCamera:
    def __init__(self, rect, seed=5, px_per_cm=3.4, tilt=3.0, yaw=1.5, noise_sd=2.5, table_grey=172, floor_grey=70):
        x0, y0, x1, y1 = rect
        self.rect = rect
        self._ppc, self._seed = px_per_cm, seed
        cfg = SCFG.load_config(os.path.join(HERE, "configs", "real_oct18.json"))
        c = cfg.camera
        c.width, c.height = 640, 480
        c.floor_width_cm = 640.0 / px_per_cm
        c.tilt_deg = [tilt, -0.6 * tilt]
        c.yaw_deg = yaw                       # image x along floor x (the table's long side across the picture)
        c.center_floor_cm = [(x0 + x1) / 2.0, (y0 + y1) / 2.0]
        c.noise_sd = noise_sd
        cfg.session.tape_lines = []
        self.cfg = cfg
        self.geom = SC.CameraGeometry(c)
        rng = np.random.default_rng(seed)
        self.R = SC.Renderer(cfg, self.geom, rng)
        g = self.geom
        img = np.full((g.H, g.W, 3), floor_grey, np.uint8)
        img = np.clip(img.astype(np.float32) + rng.normal(0, 4, (g.H, g.W, 1)), 0, 255).astype(np.uint8)
        n = 24
        edge = [(x0 + (x1 - x0) * k / n, y0) for k in range(n)] + [(x1, y0 + (y1 - y0) * k / n) for k in range(n)]
        edge += [(x1 - (x1 - x0) * k / n, y1) for k in range(n)] + [(x0, y1 - (y1 - y0) * k / n) for k in range(n)]
        self._fill(img, [(x, y, 0.0) for x, y in edge], (table_grey - 10, table_grey, table_grey + 8))
        grain = cv2.GaussianBlur(rng.normal(0, 1, (g.H, g.W)).astype(np.float32), (0, 0), 1.2) * 4
        img = np.clip(img.astype(np.float32) + grain[:, :, None], 0, 255).astype(np.uint8)
        self.bg = np.clip(img.astype(np.float32) * self.R.gain[:, :, None], 0, 255).astype(np.uint8)

    def _fill(self, img, pts, col):
        u, v = self.geom.project(np.array([p[0] for p in pts]), np.array([p[1] for p in pts]),
                                 np.array([p[2] for p in pts]))
        q = np.round(np.stack([u, v], 1) * 16).astype(np.int32)
        cv2.fillPoly(img, [q], col, lineType=cv2.LINE_AA, shift=4)

    def draw_box(self, img, poly, height_cm=6.0, side=(55, 62, 118), top=(72, 86, 150)):
        for k in range(len(poly)):
            a, b = poly[k], poly[(k + 1) % len(poly)]
            self._fill(img, [(a[0], a[1], 0.0), (b[0], b[1], 0.0), (b[0], b[1], height_cm), (a[0], a[1], height_cm)], side)
        self._fill(img, [(p[0], p[1], height_cm) for p in poly], top)

    def draw_hand(self, img, palm_xy, from_xy, z=12.0, width_cm=8.0):
        """An arm from from_xy (off the table) to a palm at palm_xy, z cm above the table."""
        (px, py), (fx, fy) = palm_xy, from_xy
        L = math.hypot(px - fx, py - fy) or 1.0
        nx, ny = -(py - fy) / L * width_cm / 2, (px - fx) / L * width_cm / 2
        self._fill(img, [(fx + nx, fy + ny, z), (px + nx, py + ny, z), (px - nx, py - ny, z), (fx - nx, fy - ny, z)],
                   (100, 128, 178))
        a = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        self._fill(img, [(px + 5.5 * math.cos(t), py + 5.5 * math.sin(t), z) for t in a], (112, 142, 196))

    def render(self, robots=(), boxes=(), hand=None):
        """Grey picture: robots [(x, y, th_rad)] or [(x, y, th_rad, left rgb, right rgb)] (their lights, 0-1; dark
        when left out), boxes [polygon], hand (palm_xy, from_xy) or None."""
        img = self.bg.copy()
        for poly in boxes:
            self.draw_box(img, poly)
        for r in robots:
            x, y, th = r[:3]
            ll, lr = (r[3], r[4]) if len(r) >= 5 else ((0, 0, 0), (0, 0, 0))
            self.R.draw_robot(img, x, y, th, 0.0, ll, lr)
        if hand is not None:
            self.draw_hand(img, hand[0], hand[1])
        self.R.add_noise(img)
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def true_model(self, path):
        """The true camera as a studio camera file (what a perfect floor calibration would give)."""
        from cameras import CameraModel
        x0, y0, x1, y1 = self.rect
        xs, ys = np.meshgrid(np.linspace(x0 - 10, x1 + 10, 9), np.linspace(y0 - 10, y1 + 10, 9))
        fl = np.column_stack([xs.ravel(), ys.ravel()])
        u, v = self.geom.project(fl[:, 0], fl[:, 1])
        m = CameraModel.from_points(np.column_stack([u, v]), fl, (self.geom.W, self.geom.H), height_cm=self.geom.Hc,
                                    k1="off", mirror="auto", name="overhead")
        d = m.to_dict()
        d["name"] = "overhead"
        with open(path, "w") as f:
            json.dump(d, f, indent=1)
        return path


# --------------------------------------------------------------------------------------- 2026-10-27: two cameras
WASHOUT_TOP = (196, 198, 200)     # washout: the robots' white tops as bright as the lamp-lit board (190): the real
WASHOUT_RIM = (186, 188, 190)     # board's 11:20 failure (tops 0-24 grey levels from the board)


def washed_out(tc, table_grey=190):
    """An overhead TableCamera whose board sits under the lamp (2026-10-27 11:20, board 193 of 255): the board is
    as bright as the robots' tops, so their bodies hardly stand out; their lights still do."""
    tc2 = TableCamera(tc.rect, px_per_cm=tc._ppc, table_grey=table_grey, seed=tc._seed)
    tc2.R.draw_robot(tc2.bg.copy(), 0.0, 0.0, 0.0, 0.0, (0, 0, 0), (0, 0, 0))      # builds the parts once
    parts = []
    for p, z, col in tc2.R._parts:
        if col == (228, 230, 232):
            col = WASHOUT_TOP
        else:
            col = WASHOUT_RIM          # the rim, the wheels' tops and the sensor window: glare off all of them
        parts.append((p, z, col))
    tc2.R._parts = parts
    tc2.washout = True
    return tc2


class SideCamera:
    """The side camera (EMEET C950 on the Pi as cam1): at the board's (0, 0) end, about 1.7 m up, 54 cm back from the
    y = 0 edge, looking down the board at 19-49 deg from straight down (docs/build-plans/two-camera-geometry.md).
    A plain pinhole camera; the robots are boxes whose vertical faces it sees. Under the lamp their faces are lit
    less than the board and their tops (light from above), so the side sees the bodies the washed-out overhead
    cannot."""

    def __init__(self, rect, pos=(45.0, -54.0, 170.0), look_at=None, f_px=520.0, size=(640, 480), seed=7,
                 board_grey=150, floor_grey=60, noise_sd=2.5):
        x0, y0, x1, y1 = rect
        self.rect = rect
        self.W, self.H = size
        self.f = f_px
        self.C = np.array(pos, float)
        tgt = np.array(look_at if look_at is not None else ((x0 + x1) / 2.0, (y0 + y1) / 2.0 - 4.0, 0.0), float)
        z = tgt - self.C
        z /= np.linalg.norm(z)
        up = np.array([0.0, 0.0, 1.0])
        xax = np.cross(z, up)          # picture's right
        xax /= np.linalg.norm(xax)
        yax = np.cross(z, xax)         # picture's down
        self.Rw = np.vstack([xax, yax, z])      # world -> camera
        self.rng = np.random.default_rng(seed)
        self.noise_sd = noise_sd
        img = np.full((self.H, self.W), float(floor_grey), np.float32)
        n = 24
        edge = [(x0 + (x1 - x0) * k / n, y0) for k in range(n)] + [(x1, y0 + (y1 - y0) * k / n) for k in range(n)]
        edge += [(x1 - (x1 - x0) * k / n, y1) for k in range(n)] + [(x0, y1 - (y1 - y0) * k / n) for k in range(n)]
        self._poly(img, [(x, y, 0.0) for x, y in edge], board_grey)
        grain = cv2.GaussianBlur(self.rng.normal(0, 1, (self.H, self.W)).astype(np.float32), (0, 0), 1.0) * 4
        self.bg = np.clip(img + grain, 0, 255).astype(np.float32)

    def project(self, p):
        q = self.Rw @ (np.asarray(p, float) - self.C)
        return self.f * q[0] / q[2] + self.W / 2.0, self.f * q[1] / q[2] + self.H / 2.0

    def _poly(self, img, pts, grey):
        uv = np.array([self.project(p) for p in pts])
        cv2.fillPoly(img, [np.round(uv * 16).astype(np.int32)], float(grey), lineType=cv2.LINE_AA, shift=4)

    def render(self, robots=()):
        """Grey picture of robots [(x, y, th_rad[, left rgb, right rgb])]: a 9.5 x 9.5 x 4 cm box each (faces shaded,
        top light), the two LEDs at the front of the top."""
        img = self.bg.copy()
        half, h = 4.75, 4.0
        order = sorted(robots, key=lambda r: -math.hypot(r[0] - self.C[0], r[1] - self.C[1]))
        for r in order:
            x, y, th = r[:3]
            c, s = math.cos(th), math.sin(th)
            corners = [(x + c * a - s * b, y + s * a + c * b) for a, b in ((half, half), (-half, half), (-half, -half),
                                                                          (half, -half))]
            faces = []
            for k in range(4):
                p, q = corners[k], corners[(k + 1) % 4]
                mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
                nx, ny = mx - x, my - y
                if nx * (self.C[0] - mx) + ny * (self.C[1] - my) <= 0:
                    continue                                  # faces away from the camera
                shade = 95.0 + 20.0 * abs(nx) / half          # the lamp is above: faces dim, some more than others
                faces.append(((p[0], p[1], 0.0), (q[0], q[1], 0.0), (q[0], q[1], h), (p[0], p[1], h), shade))
            for f in faces:
                self._poly(img, f[:4], f[4])
            self._poly(img, [(a, b, h) for a, b in corners], 222.0)
            for k, lat in enumerate((1.0, -1.0)):
                led = r[3 + k] if len(r) >= 5 else (0, 0, 0)
                # 2.55 cm either side: the same spot the overhead's renderer draws them (camera.py: shell_w / 2 - 1.3),
                # so both simulated cameras see one light at one place (corners-only setup, 2026-10-29; was 2.6)
                lx, ly = x + c * (half - 2.0) - s * lat * 2.55, y + s * (half - 2.0) + c * lat * 2.55
                u, v = self.project((lx, ly, h))
                lvl = float(np.max(led))
                # a 2 px disc blended at its sub-pixel spot (corners-only setup, 2026-10-29): drawn at whole pixels
                # (cv2.circle does not anti-alias a float picture) two lights 13 px apart read up to 5 % off
                x0i, y0i = int(math.floor(u)) - 4, int(math.floor(v)) - 4
                yy, xx = np.mgrid[y0i:y0i + 9, x0i:x0i + 9]
                cover = np.clip(2.5 - np.hypot(xx + 0.5 - u - 0.5, yy + 0.5 - v - 0.5), 0.0, 1.0).astype(np.float32)
                ys, xs = slice(max(0, y0i), max(0, min(self.H, y0i + 9))), slice(max(0, x0i), max(0, min(self.W, x0i + 9)))
                cv = cover[ys.start - y0i:ys.stop - y0i, xs.start - x0i:xs.stop - x0i]
                img[ys, xs] = img[ys, xs] * (1 - cv) + (70.0 + 185.0 * lvl) * cv
        img += self.rng.normal(0, self.noise_sd, img.shape).astype(np.float32)
        return np.clip(img, 0, 255).astype(np.uint8)

    def true_model(self, path, z=0.0):
        """The true camera as a studio camera file, fitted to floor points the way the setup page fits the side."""
        from cameras import CameraModel
        x0, y0, x1, y1 = self.rect
        xs, ys = np.meshgrid(np.linspace(x0 - 5, x1 + 5, 11), np.linspace(y0 - 5, y1 + 5, 11))
        fl = np.column_stack([xs.ravel(), ys.ravel()])
        uv = np.array([self.project((a, b, z)) for a, b in fl])
        ok = (uv[:, 0] > 0) & (uv[:, 0] < self.W) & (uv[:, 1] > 0) & (uv[:, 1] < self.H)
        m = CameraModel.from_points(uv[ok], fl[ok], (self.W, self.H), k1="off", mirror="auto", name="side")
        d = m.to_dict()
        d["name"] = "side"
        with open(path, "w") as f:
            json.dump(d, f, indent=1)
        return path
