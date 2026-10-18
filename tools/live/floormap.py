"""Image pixels <-> floor cm for points at a given height (the robot's top
lights), from a floor_calibration.json (tools/calibrate_floor.py, or the
simulator's). Same corrections as tools/fusion/runs.py uses offline: lens term,
homography, then the height correction (a point h cm up looks pushed away from
the floor point below the lens by camera height / (camera height - h))."""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (TOOLS, os.path.join(TOOLS, "fusion")):
    if p not in sys.path:
        sys.path.insert(0, p)

import runs as FR  # noqa: E402  (tools/fusion/runs.py)

LED_HEIGHT_CM = FR.ALVIK_TOP_CM     # the top lights sit on the robot's top, about 4 cm up


def find_calibration(recording=None, given=None):
    """The floor_calibration.json to use: the one given, else beside or above the recording."""
    if given:
        return given
    if recording:
        for c in (os.path.join(recording, "floor_calibration.json"),
                  os.path.join(os.path.dirname(os.path.abspath(recording)), "floor_calibration.json")):
            if os.path.exists(c):
                return c
    return None


class FloorMap:
    def __init__(self, path, led_height_cm=LED_HEIGHT_CM):
        self.path = path
        with open(path) as f:
            d = json.load(f)
        self.H, self.Hinv = FR.load_homography(path)
        self.dist = FR.load_distortion(path)
        self.height_cm = led_height_cm
        self.parallax = FR.load_parallax(path, obj_height_cm=led_height_cm)
        self.note = ""
        plane = d.get("plane_height_cm")
        if plane:
            # this homography maps the robot-top plane already (no camera height known)
            self.parallax = None
            self.note = "calibration maps the plane %g cm up (robot top): no height correction" % plane
        elif self.parallax is None:
            self.note = "calibration has no camera height: light positions not corrected for height (up to ~1.5 cm)"
        self.size = d.get("image_size") or [640, 480]

    def to_floor(self, u, v):
        """Pixel of a point led_height_cm up -> the floor point below it (cm)."""
        x, y = FR.apply_h(self.Hinv, *FR.undistort_px(u, v, self.dist))
        return FR.floor_from_top(x, y, self.parallax)

    def to_px(self, x, y, z=None):
        """Floor point (cm), z cm up (default: the light height) -> pixel."""
        z = self.height_cm if z is None else z
        if self.parallax:
            nx, ny, hc, _ = self.parallax
            k = hc / (hc - z)
            x, y = nx + (x - nx) * k, ny + (y - ny) * k
        u, v = FR.apply_h(self.H, x, y)
        if self.dist:
            k1, cx, cy, f = self.dist
            xn, yn = (u - cx) / f, (v - cy) / f
            fac = 1 + k1 * (xn * xn + yn * yn)
            u, v = cx + f * xn * fac, cy + f * yn * fac
        return u, v

    def px_per_cm(self, u=None, v=None):
        """Pixel scale (at the light height) around pixel (u, v), default the image centre."""
        u = self.size[0] / 2 if u is None else u
        v = self.size[1] / 2 if v is None else v
        x, y = self.to_floor(u, v)
        u1, v1 = self.to_px(x + 1, y)
        u2, v2 = self.to_px(x, y + 1)
        u0, v0 = self.to_px(x, y)
        return 0.5 * (math.hypot(u1 - u0, v1 - v0) + math.hypot(u2 - u0, v2 - v0))
