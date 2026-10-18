"""Make tools/, tools/pi and tools/fusion importable, and a synthetic camera for tests."""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (TOOLS, os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "fusion"), os.path.join(TOOLS, "detector")):
    if p not in sys.path:
        sys.path.insert(0, p)


class Cam:
    """Pinhole camera above the floor, same model as tools/sim/camera.py:
    p_cam = R @ [x - X0, -(y - Y0), height - z]; pixel = f p/p_z (+ k1) + centre."""

    def __init__(self, height=115.0, centre=(60.0, 10.0), yaw=1.0, tilt=(0.8, -0.5), k1=0.0, floor_width=160.0,
                 size=(640, 480)):
        self.W, self.H = size
        self.f = self.W / floor_width * height
        self.c = np.array([self.W / 2, self.H / 2])
        y, a, b = math.radians(yaw), math.radians(tilt[0]), math.radians(tilt[1])
        Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
        Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
        Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
        self.R = Rz @ Rx @ Ry
        self.X0, self.Y0, self.Hc, self.k1 = centre[0], centre[1], height, k1

    def project(self, x, y, z=0.0, ideal=False):
        p = self.R @ np.array([x - self.X0, -(y - self.Y0), self.Hc - z])
        xn, yn = p[0] / p[2], p[1] / p[2]
        fac = 1 if ideal else 1 + self.k1 * (xn * xn + yn * yn)
        return self.f * xn * fac + self.c[0], self.f * yn * fac + self.c[1]

    def nadir(self):
        return (self.X0, self.Y0)


def floor_error(calib, cam, area=(-10, -40, 130, 60), z=0.0, step=5.0):
    """Errors (cm) of a calibration over a floor area, against the true camera."""
    import calibrate_floor as cf
    errs = []
    for x in np.arange(area[0], area[2] + 1e-9, step):
        for y in np.arange(area[1], area[3] + 1e-9, step):
            u, v = cam.project(x, y, z)
            ui, vi = cam.project(x, y, z, ideal=True)   # (strong barrel distortion folds far points back in)
            if 0 <= u < cam.W and 0 <= v < cam.H and -0.2 * cam.W <= ui < 1.2 * cam.W and -0.2 * cam.H <= vi < 1.2 * cam.H:
                X, Y = cf.to_floor(calib, u, v, z)
                errs.append(math.hypot(X - x, Y - y))
    return np.array(errs)
