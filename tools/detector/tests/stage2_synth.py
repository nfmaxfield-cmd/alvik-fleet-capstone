"""Tiny synthetic scenes for the stage 2 training tests (not a test module): dark square robots with a
light bar at the front on a noisy floor, and a dark round stain that looks a bit like a robot."""
import math
import os

import cv2
import numpy as np

from conftest import HEADER

W, H, ROBOT = 320, 240, 24


def floor(rng):
    img = np.full((H, W, 3), (170, 175, 178), np.float32)
    img += cv2.GaussianBlur(rng.normal(0, 1, (H, W)).astype(np.float32), (0, 0), 10)[:, :, None] * 25
    img += rng.normal(0, 3, (H, W, 1)).astype(np.float32)
    return img


def draw_robot(img, x, y, h):
    pts = []
    for u, v in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        u, v = u * ROBOT / 2, v * ROBOT / 2
        c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
        pts.append((x + u * c - v * s, y - u * s - v * c))
    cv2.fillPoly(img, [np.round(np.array(pts)).astype(np.int32)], (42, 40, 38), lineType=cv2.LINE_AA)
    fx, fy = x + 0.3 * ROBOT * math.cos(math.radians(h)), y - 0.3 * ROBOT * math.sin(math.radians(h))
    cv2.circle(img, (int(fx), int(fy)), 3, (215, 215, 210), -1, cv2.LINE_AA)


def draw_stain(img, x, y):
    m = np.zeros((H, W), np.float32)
    cv2.circle(m, (int(x), int(y)), ROBOT // 2, 1.0, -1, cv2.LINE_AA)
    m = cv2.GaussianBlur(m, (0, 0), 1.5)
    img[:] = img * (1 - 0.8 * m[:, :, None]) + np.array((60, 58, 56), np.float32) * 0.8 * m[:, :, None]


def scene(rng, n_robots=1, stains=(), min_gap=40):
    """Returns (BGR uint8 image, [(x, y, heading)] robots)."""
    img = floor(rng)
    spots = [tuple(s) for s in stains]
    robots = []
    for _ in range(n_robots):
        for _try in range(50):
            x, y = rng.uniform(30, W - 30), rng.uniform(30, H - 30)
            if all(math.hypot(x - a, y - b) > min_gap for a, b in spots + [(r[0], r[1]) for r in robots]):
                robots.append((x, y, rng.uniform(0, 360)))
                break
    for sx, sy in stains:
        draw_stain(img, sx, sy)
    for x, y, h in robots:
        draw_robot(img, x, y, h)
    return np.clip(img, 0, 255).astype(np.uint8), robots


def write_recording(root, name, frames, fps=10.0):
    """frames: list of (image, rows) where rows are label rows (without the frame number), in HEADER
    order. Writes <root>/<name>/labels/{labels.csv,frames/}, and frames.csv. Returns the labels folder."""
    import csv
    ld = os.path.join(root, name, "labels")
    os.makedirs(os.path.join(ld, "frames"), exist_ok=True)
    with open(os.path.join(ld, "labels.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        for fr, (img, rows) in enumerate(frames):
            cv2.imwrite(os.path.join(ld, "frames", "%06d.jpg" % fr), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            for r in rows:
                w.writerow([fr] + list(r))
    return ld


def robot_row(x, y, h, status="confirmed", obj=1):
    return ["", obj, "alvik", "%.2f" % x, "%.2f" % y, "%.1f" % h, 1, "user", status, ""]


def reject_row(x, y, cls="alvik"):
    return ["", "", cls, "%.2f" % x, "%.2f" % y, "", 0, "motion", "rejected", "0.5"]
