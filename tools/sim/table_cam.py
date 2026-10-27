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
