"""Filled-in floor at the board's edge (e02_camera.Tracker._fill_at_edge, 2026-10-28 19:34 DOE loop): a robot that
stands within a robot width of the edge when tracking starts has its floor filled in from round it. The plain fill
took the dark floor beyond the edge into the plywood under the robot, so plain plywood there read as "robot" and the
ladder put Center 5 cm from the x = 0 edge when its middle was 9-10 cm in (replay of the 19:42 find step).

    python3 -m pytest tools/experiment/tests/test_e02_edge_fill.py -q
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
TOOLS = os.path.dirname(EXP)
for p in (os.path.join(TOOLS, "sim"), EXP, os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live"), TOOLS):
    if p not in sys.path:
        sys.path.append(p)

import e02_camera as CAM      # noqa: E402
import e02_twocam as TW       # noqa: E402
import table_cam as TC        # noqa: E402

RECT = (0.0, 0.0, 122.0, 122.0)


@pytest.fixture(scope="module")
def cam(tmp_path_factory):
    d = tmp_path_factory.mktemp("cams")
    tc = TC.TableCamera(RECT, seed=18, px_per_cm=2.34)
    oc = CAM.load_camera(tc.true_model(str(d / "o.json")))
    return tc, oc


def board_mask(oc, shape):
    import cv2
    pts = [(x, 0.0) for x in np.linspace(0, 122, 13)] + [(122.0, y) for y in np.linspace(0, 122, 13)] + \
          [(x, 122.0) for x in np.linspace(122, 0, 13)] + [(0.0, y) for y in np.linspace(122, 0, 13)]
    bm = np.zeros(shape, np.uint8)
    cv2.fillPoly(bm, [np.array([[int(round(c)) for c in oc.project((x, y, 0.0))] for x, y in pts], np.int32)], 1)
    return bm > 0


def _track(cam, x, y, th=math.radians(200), frames=40):
    """The robot standing still at (x, y); the floor beyond the board darkened to 60 as in the room (19:42 find step:
    plywood ~190, the floor beyond the x = 0 edge 60-90)."""
    tc, oc = cam
    trk = TW.TwoCamTracker(oc, board_center=(61, 61), board_rect=RECT, side_cam=None, lights=False)
    L = (0, 0, 0)
    off = None

    def pic():
        nonlocal off
        im = tc.render([(x, y, th, L, L)])
        if off is None:
            off = ~board_mask(oc, im.shape[:2])
        im = im.copy()
        im[off] = 60
        return im
    trk.set_background([pic() for _ in range(5)], robots=[(x, y)])
    trk.add("R", x, y, heading=math.degrees(th))
    t = 1000.0
    for _ in range(frames):
        t += 0.08
        trk.push(pic(), t)
    return trk


def test_plywood_under_a_robot_at_the_edge_is_filled_from_plywood(cam):
    tc, oc = cam
    trk = _track(cam, 9.0, 60.0, frames=1)
    bm = board_mask(oc, trk.bg.shape)
    unk = trk.known == 0
    seen_board = trk.bg[bm & ~unk]
    filled_board = trk.bg[bm & unk]
    assert unk.any() and (bm & unk).any() and (~bm & unk).any()     # the disk crosses the edge
    # the filled plywood is plywood-bright everywhere, with no dark strip from the floor beyond the edge (the plain
    # fill: 5th percentile 64-65 against plywood 168)
    assert float(np.percentile(filled_board, 5)) > float(np.median(seen_board)) - 25.0


@pytest.mark.parametrize("x", [8.0, 9.0, 10.0])
def test_a_robot_standing_near_the_edge_is_placed_where_it_stands(cam, x):
    # the plain fill: 6.3, 6.9, 7.5 (1.7-2.5 cm towards the edge); real 19:42 frames: 5-6 for 9-10
    trk = _track(cam, x, 60.0)
    tr = trk.tracks["R"]
    assert math.hypot(tr.x - x, tr.y - 60.0) < 1.2


def test_a_robot_well_inside_the_board_is_unchanged(cam):
    trk = _track(cam, 40.0, 60.0)
    tr = trk.tracks["R"]
    assert math.hypot(tr.x - 40.0, tr.y - 60.0) < 2.0
