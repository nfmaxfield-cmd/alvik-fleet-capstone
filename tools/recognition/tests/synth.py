"""Synthetic cameras for the recognition tests (not a test module)."""
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
for p in (TOOLS, os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

from recognition.cameras import CameraModel  # noqa: E402

SIZE = (640, 480)


def look_at(pos, target, f, up=(0.0, 0.0, 1.0), k1=0.0, size=SIZE, name=None):
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    fwd = target - pos
    fwd /= np.linalg.norm(fwd)
    up = np.asarray(up, float)
    if abs(fwd @ up) > 0.999:
        up = np.array([0.0, 1.0, 0.0])
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.vstack([right, down, fwd])
    K = np.array([[f, 0, size[0] / 2], [0, f, size[1] / 2], [0, 0, 1.0]])
    return CameraModel(K, [k1, 0, 0, 0, 0], cv2.Rodrigues(R)[0], -R @ pos, size, name=name)


def cam_overhead(k1=0.0):
    """Near-nadir, 262 cm up over the board, tilted ~2 deg, image turned ~5 deg."""
    return look_at((63.0, 58.0, 262.0), (61.0 + 9.0, 61.0, 0.0), f=800.0, up=(math.sin(0.09), math.cos(0.09), 0),
                   k1=k1, name="A")


def cam_oblique(k1=0.0):
    """~45 deg down, 200 cm from the board centre, from the -x side."""
    d = 200.0 / math.sqrt(2)
    return look_at((61.0 - d, 61.0 + 10.0, d), (61.0, 61.0, 0.0), f=520.0, k1=k1, name="B")


def board_points(step=20.0, lo=1.0, hi=121.0):
    g = np.arange(lo, hi + 1e-9, step)
    return np.array([(x, y) for x in g for y in g])


def clicks(cam, xy, z=0.0, noise=0.0, seed=0):
    xy = np.asarray(xy, float)
    zz = np.broadcast_to(np.asarray(z, float), (len(xy),))
    u, v = cam.project(np.column_stack([xy, zz]))
    rng = np.random.default_rng(seed)
    uv = np.column_stack([u, v]) + rng.normal(0, noise, (len(xy), 2)) if noise else np.column_stack([u, v])
    ok = cam.in_image(uv[:, 0], uv[:, 1])
    return uv[ok], xy[ok], zz[ok]
