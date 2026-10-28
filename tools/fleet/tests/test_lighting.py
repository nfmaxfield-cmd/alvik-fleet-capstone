"""The lighting check (2026-10-28): robots' contrast against the board per camera, in plain words.

    python3 -m pytest tools/fleet/tests/test_lighting.py -q
"""
import os
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import e02_camera as CAM  # noqa: E402
import lighting as LG  # noqa: E402

CAL = os.path.join(TOOLS, "experiment", "data", "overhead_floorcal_roam1.json")
SPOTS = [(40.0, 40.0), (80.0, 70.0), (60.0, 95.0)]


def picture(board, top, clip=0.0, seed=0):
    cam = CAM.load_camera(CAL)
    W, H = cam.image_size
    rng = np.random.default_rng(seed)
    g = np.clip(rng.normal(board, 3.0, (H, W)), 0, 255)
    for x, y in SPOTS:
        pts = [cam.project((x + dx, y + dy, CAM.ROBOT_TOP_CM)) for dx, dy in
               ((-4.75, -4.75), (4.75, -4.75), (4.75, 4.75), (-4.75, 4.75))]
        cv2.fillPoly(g, [np.round(np.array(pts)).astype(np.int32)], float(top))
    if clip:
        g[rng.random((H, W)) < clip] = 255
    return g.astype(np.uint8), cam


@pytest.mark.parametrize("top,verdict", [(235, "ok"), (213, "low"), (200, "washed out")])
def test_verdict_follows_the_robots_contrast(top, verdict):
    g, cam = picture(190, top)
    m = LG.measure(g, cam, spots=SPOTS)
    assert m["verdict"] == verdict, m
    if verdict == "washed out":
        assert "close the blinds or turn off the lamp above the table" in m["words"]


def test_finds_the_robots_itself_and_a_clipping_board():
    g, cam = picture(190, 235)
    m = LG.measure(g, cam, n_robots=3)
    assert m["found"] == 3 and m["verdict"] == "ok", m
    got = sorted((r["x"], r["y"]) for r in m["robots"])
    for (x, y), (a, b) in zip(sorted(SPOTS), got):
        assert abs(x - a) < 2.0 and abs(y - b) < 2.0
    g, cam = picture(240, 250, clip=0.05)
    m = LG.measure(g, cam, spots=SPOTS)
    assert m["verdict"] == "washed out" and "clipped" in m["words"]
