"""Measuring the object size on footage like the real overhead camera (Oct 21 roam1: the studio fell back
to 30 px where a robot is about 23 px across). What made it fail there, rebuilt here:
  - grey robots on a pale board: a robot differs from the floor mostly along its dark edges, so its
    blob is a ring or a U with the middle missing (too little area, too low solidity to be counted);
  - a light halo on the board on one side of each robot, which joins the blob and makes it bigger;
  - robots parked in one place for many of the sampled frames, and small things always lying there.
The real-footage check runs when MIRLT_ROAM1_VIDEO names the roam1 overhead video."""
import math
import os
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import motion  # noqa: E402

W, H = 640, 480
SIDE = 22.0                     # robot side in px (9.5 cm at 2.34 px/cm)


def pale_board(rng):
    base = np.full((H, W), 175.0, np.float32)
    for sigma, amp in ((30, 8.0), (4, 4.0)):
        n = cv2.GaussianBlur(rng.normal(0, 1, (H, W)).astype(np.float32), (0, 0), sigma)
        base += n * amp / max(1e-6, n.std())
    return base


def grey_robot(img, x, y, ang, rng):
    """A robot as the overhead camera sees it: the top nearly the board's grey, broken dark edges on three
    sides (a U), a small white light, and a light halo on the board on one side."""
    c, s = math.cos(math.radians(ang)), math.sin(math.radians(ang))

    def pt(u, v):
        return [x + u * c - v * s, y + u * s + v * c]
    h = SIDE / 2
    m = np.zeros((H, W), np.uint8)
    halo = np.array([pt(-h - 9, -h - 4), pt(-h - 9, h + 4), pt(-h, h + 4), pt(-h, -h - 4)], np.float32)
    cv2.fillPoly(m, [np.round(halo).astype(np.int32)], 1)
    img[m > 0] += 24.0 * (rng.random(int(m.sum())) < 0.6)  # the halo: patchily lighter than the board
    m[:] = 0
    cv2.fillPoly(m, [np.round(np.array([pt(-h, -h), pt(h, -h), pt(h, h), pt(-h, h)])).astype(np.int32)], 1)
    img[m > 0] = img[m > 0] - 6.0 + rng.normal(0, 4, int(m.sum()))   # top: about the board's grey
    e = h - 1.5                                           # 3 px edges, drawn inside the body
    m[:] = 0
    for a, b in ((pt(-e, -e), pt(e, -e)), (pt(e, -e), pt(e, e)), (pt(e, e), pt(-e, e))):
        cv2.line(m, tuple(int(round(v)) for v in a), tuple(int(round(v)) for v in b), 1, 3)
    sel = (m > 0) & (rng.random((H, W)) < 0.7)              # broken: parts of the edge match the board
    img[sel] -= 60.0
    cv2.circle(img, (int(round(x)), int(round(y))), 2, 235.0, -1)


def scene(seed=4, n=41):
    """n frames over a session: three robots parked for the first 40%, then driving about; four small
    things (9 px) always lying on the board."""
    rng = np.random.default_rng(seed)
    board = pale_board(rng)
    for (bx, by) in ((520, 400), (560, 90), (90, 420), (600, 300)):
        cv2.circle(board, (bx, by), 4, 95.0, -1)
    frames = []
    parked = [(150, 120, 10.0), (320, 140, 25.0), (470, 170, 40.0)]
    for k in range(n):
        img = board.copy()
        for j, (px, py, pa) in enumerate(parked):
            if k < 0.4 * n:
                x, y, a = px, py, pa
            else:
                x = 120 + (k * 37 + j * 140) % 400
                y = 110 + (k * 23 + j * 90) % 260
                a = (k * 31 + j * 50) % 360
            grey_robot(img, x, y, a, rng)
        img += rng.normal(0, 2.0, img.shape).astype(np.float32)
        g = np.clip(img, 0, 255).astype(np.uint8)
        frames.append(cv2.cvtColor(g, cv2.COLOR_GRAY2BGR))
    bg = np.median(np.stack(frames), axis=0).astype(np.uint8)
    return frames, bg


def test_size_of_grey_robots_with_a_halo_and_parked_spells_is_their_side():
    frames, bg = scene()
    p = motion.Params()
    meas = []
    for f in frames:
        meas.extend(motion.measure_object_sizes(f, bg, p))
    est = motion.object_size_from_measurements(meas)
    assert est is not None
    assert est == pytest.approx(SIDE, rel=0.12)


def test_size_needs_objects_at_several_places():
    frames, bg = scene()
    p = motion.Params()
    # only the parked spell: three places, one robot each, every frame the same
    meas = []
    for f in frames[:10]:
        meas.extend(motion.measure_object_sizes(f, bg, p))
    assert motion.object_size_from_measurements(meas) is None
    assert motion.object_size_from_measurements([]) is None


def test_a_plain_dark_robot_is_measured_as_before():
    """The synthetic recordings' robots (solid dark squares) keep their size."""
    rng = np.random.default_rng(2)
    board = np.clip(pale_board(rng), 0, 255).astype(np.uint8)
    bg = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)
    meas = []
    for k in range(8):
        f = bg.copy()
        for j in range(2):
            x, y = 100 + 60 * k + 10 * j, 100 + 150 * j
            rect = ((x, y), (30, 30), 15.0 * k)
            cv2.fillPoly(f, [np.round(cv2.boxPoints(rect)).astype(np.int32)], (60, 60, 65))
        meas.extend(motion.measure_object_sizes(f, bg, motion.Params()))
    assert motion.object_size_from_measurements(meas) == pytest.approx(30.0, rel=0.08)


ROAM1 = os.environ.get("MIRLT_ROAM1_VIDEO", "")


@pytest.mark.skipif(not (ROAM1 and os.path.exists(ROAM1)), reason="set MIRLT_ROAM1_VIDEO to the roam1 overhead video")
def test_real_roam1_recording_measures_about_23_px():
    """Oct 21: the robot's short side is 23.2 px by hand (9.5 cm at 2.34 px/cm is 22.2 px); before the fix
    this measurement failed and 30 px was used."""
    est = motion.estimate_robot_size(ROAM1)
    assert est is not None and 20.5 <= est <= 25.0


def test_a_motion_cache_made_with_the_stand_in_size_is_redone_once():
    import app
    assert app.size_failed_before({"robot_px": 30.0, "robot_px_source": "default"})
    assert not app.size_failed_before({"robot_px": 30.0, "robot_px_source": "default",
                                       "size_method": motion.SIZE_METHOD})      # tried with this way: kept
    assert not app.size_failed_before({"robot_px": 22.8, "robot_px_source": "estimated"})
    assert not app.size_failed_before({"robot_px": 24.0, "robot_px_source": "given"})
