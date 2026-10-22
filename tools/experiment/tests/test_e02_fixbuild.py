"""Tests for the 2026-10-22 E02 fixes (docs/build-plans/e02-fix-build.md): the floor-memory tracker, finding a
robot by a turn on its spot, paths round known robots, the camera / wheels check, and the robot mapping maths.

    python -m pytest tools/experiment/tests/test_e02_fixbuild.py -q
"""
import math

import cv2
import numpy as np
import pytest

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02_plan as PL
import e02_map as MAP
from test_e02_runner import FakeRobot, FakeWorld, sim_camera  # noqa: F401
from test_e02_runner import cam_setup  # noqa: F401  (fixture)


def _tracker(cam):
    import e02_camera as CAM
    return CAM.MotionTracker(cam, board_rect=(0.0, 0.0, 122.15, 121.9))


def test_the_spot_a_robot_leaves_is_never_taken_for_it(cam_setup):
    """2026-10-21: robots in the picture when it was taken; the one that drove off left a 'robot' behind."""
    w, sc, cam, _ = cam_setup
    mover, still = FakeRobot(40.0, 40.0, 0.0), FakeRobot(90.0, 90.0, 45.0)
    w.robots[:] = [mover, still]
    tr = _tracker(cam)
    tr.set_background([sc.render() for _ in range(3)], robots=[(mover.x, mover.y)])   # both robots IN the picture
    tr.park("S", still.x, still.y)
    tr.follow("M", mover.x, mover.y, t=0.0)
    t, errs = 0.0, []
    for k in range(150):
        t += 1 / 15.0
        mover.x += 8.0 / 15.0
        s = tr.push(sc.render(), t)
        if s is not None:
            errs.append(math.hypot(s[1] - mover.x, s[2] - mover.y))
    assert len(errs) > 120
    assert max(errs[20:]) < 2.5, max(errs[20:])
    # its old spot is floor again: nothing robot-shaped there
    assert not any(math.hypot(b[3] - 40.0, b[4] - 40.0) < 6.0 for b in tr.blobs())


def _paint_hand(img, cam, x, y, w_cm=8.0, l_cm=40.0, value=40):
    """A dark arm reaching in from the board's open x = 0 side to (x, y)."""
    pts = [(x, y - w_cm / 2), (x, y + w_cm / 2), (x - l_cm, y + w_cm / 2), (x - l_cm, y - w_cm / 2)]
    poly = np.array([cam.project((px, py, 5.0)) for px, py in pts], np.int32)
    out = img.copy()
    cv2.fillPoly(out, [poly], value)
    return out


def test_a_hand_next_to_a_still_robot_does_not_pull_the_tracker_away(cam_setup):
    w, sc, cam, _ = cam_setup
    r = FakeRobot(30.0, 60.0, 0.0)
    w.robots[:] = []
    bg = [sc.render() for _ in range(3)]
    w.robots[:] = [r]
    tr = _tracker(cam)
    tr.set_background(bg)
    tr.follow("R", r.x, r.y, t=0.0)
    t = 0.0
    for _ in range(10):
        t += 1 / 15.0
        tr.push(sc.render(), t)
    tr.hold(6.0)
    for k in range(60):
        t += 1 / 15.0
        img = _paint_hand(sc.render(), cam, 26.0 - 0.0 * k, 75.0)
        tr.push(img, t)
        p = tr.where()
        assert math.hypot(p[1] - r.x, p[2] - r.y) < 3.0, (k, p)


def test_robot_shaped_things_off_the_board_are_ignored(cam_setup):
    w, sc, cam, _ = cam_setup
    w.robots[:] = []
    bg = [sc.render() for _ in range(3)]
    tr = _tracker(cam)
    tr.set_background(bg)
    w.robots[:] = [FakeRobot(-25.0, 60.0, 0.0), FakeRobot(60.0, 60.0, 0.0)]
    tr.push(sc.render(), 0.1)
    tr.push(sc.render(), 0.2)
    bl = tr.blobs()
    assert any(math.hypot(b[3] - 60.0, b[4] - 60.0) < 3.0 for b in bl)
    assert not any(b[3] < -10.0 for b in bl)


def test_a_turn_on_the_spot_finds_the_robot_and_nothing_else(cam_setup):
    import e02_camera as CAM
    w, sc, cam, _ = cam_setup
    a, b, c = FakeRobot(30.0, 30.0, 10.0), FakeRobot(90.0, 40.0, 70.0), FakeRobot(60.0, 95.0, -30.0)
    w.robots[:] = [a, b, c]
    before = [sc.render() for _ in range(6)]
    b.th += 40.0
    after = [sc.render() for _ in range(6)]
    found = CAM.locate_change(cam, before, after, board_rect=(0.0, 0.0, 122.15, 121.9))
    assert found and math.hypot(found[0]["x"] - b.x, found[0]["y"] - b.y) < 3.0, found
    assert len(found) == 1
    # nothing turned: nothing found
    assert CAM.locate_change(cam, after, [sc.render() for _ in range(6)], board_rect=(0.0, 0.0, 122.15, 121.9)) == []


def _runner_with(positions):
    import e02 as R
    run = R.Runner.__new__(R.Runner)
    run.st = {"positions": positions}
    run.board = PL.Board(xboard_side=1, yboard_side=1)
    return run


def test_paths_go_round_a_robot_in_the_way_and_stay_on_the_board():
    run = _runner_with({"Right": [60.0, 60.0, None], "Center": [100.0, 20.0, 0.0]})
    p, target = (20.0, 60.0), (100.0, 60.0)
    wps = run.safe_path("Left", p, target)
    assert len(wps) == 2
    q = wps[0]
    assert run.board.on_board(q[0], q[1], margin=10.0)
    # both pieces clear Right by more than the park clearance
    for a, b in ((p, q), (q, target)):
        for t in np.linspace(0, 1, 50):
            x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            assert math.hypot(x - 60.0, y - 60.0) > PL.PARK_CLEAR_CM
    # nothing in the way: straight there
    assert run.safe_path("Left", (20.0, 20.0), (40.0, 30.0)) == [(40.0, 30.0)]


def test_camera_and_wheels_check_flags_a_camera_that_saw_something_else():
    import e02 as R
    run = R.Runner.__new__(R.Runner)
    run.fresh_id = {}
    run.disagree = {"L": 0}
    run.last_lid = {"L": 7}
    run.wheels_moved = lambda robot, lid: 20.0
    assert run.cross_check("L", (0.0, 0.0), (19.0, 1.0)) == ""
    assert "wheels say 20" in run.cross_check("L", (0.0, 0.0), (2.0, 0.0))
    assert "lost it" in run.cross_check("L", (0.0, 0.0), None)
    run.fresh_id["L"] = True                      # first leg after a turn on the spot: a rougher start
    assert run.cross_check("L", (0.0, 0.0), (11.0, 0.0)) == ""


def test_mapping_stop_kinds_and_contact_points():
    assert MAP.stop_kind("failed", "plywood edge (line)") == "edge"
    assert MAP.stop_kind("failed", "obstacle") == "board"
    assert MAP.stop_kind("done", "") is None
    x, y = MAP.contact_point(10.0, 20.0, 90.0, "edge")
    assert x == pytest.approx(10.0) and y == pytest.approx(20.0 + MAP.LINE_AHEAD_CM)
    x, y = MAP.contact_point(10.0, 20.0, 0.0, "board")
    assert x == pytest.approx(10.0 + MAP.TOF_AHEAD_CM + MAP.SAFE_FRONT_CM)
    assert len(MAP.ray_headings(16)) == 16


def test_mapping_fits_each_side_and_leaves_out_parked_robots():
    b = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=119.9, yboard_cm=122.4)
    rng = np.random.default_rng(1)
    pts = []
    for v in np.linspace(20, 100, 5):
        pts.append((float(v), float(119.9 + rng.normal(0, 0.5)), "board"))    # the x-board (y = const)
        pts.append((float(122.4 + rng.normal(0, 0.5)), float(v), "board"))    # the y-board (x = const)
        pts.append((float(v), float(0.8 + rng.normal(0, 0.5)), "edge"))       # open edge y = 0
        pts.append((float(1.0 + rng.normal(0, 0.5)), float(v), "edge"))       # open edge x = 0
    pts.append((30.0, 106.0, "board"))                                        # a parked robot stopped this ray
    sides = MAP.fit_sides(pts, b, others=[(30.0, 100.0)])
    assert set(sides) == {"x=0", "x=122.15", "y=0", "y=121.9"}
    assert sides["y=121.9"]["kind"] == "board" and sides["y=121.9"]["value"] == pytest.approx(119.9, abs=0.6)
    assert sides["x=122.15"]["value"] == pytest.approx(122.4, abs=0.6)
    assert sides["y=0"]["kind"] == "edge" and sides["x=0"]["kind"] == "edge"
    assert sides["y=121.9"]["n"] == 5


def test_rays_towards_a_parked_robot_are_skipped():
    assert MAP.ray_blocked((61.0, 61.0), 135.0, [("Right", (20.0, 100.0))]) == "Right"
    assert MAP.ray_blocked((61.0, 61.0), -45.0, [("Right", (20.0, 100.0))]) == ""
    assert MAP.ray_blocked((61.0, 61.0), 0.0, [("Center", (110.0, 75.0))]) == "Center"


def _clear_of(run, pts, others, clear):
    for a, b in zip(pts, pts[1:]):
        for t in np.linspace(0, 1, 60):
            x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            if any(math.hypot(x - o[0], y - o[1]) <= clear for o in others):
                return False
    return True


def test_paths_go_round_two_robots_and_give_up_when_walled_in():
    run = _runner_with({"A": [45.0, 60.0, None], "B": [75.0, 60.0, None]})
    p, target = (15.0, 60.0), (105.0, 60.0)
    wps = run.safe_path("Left", p, target)
    assert wps is not None and wps[-1] == target
    assert _clear_of(run, [p] + wps, [(45.0, 60.0), (75.0, 60.0)], PL.PARK_CLEAR_CM)
    # the target itself boxed in by robots on every side: no clear way
    box = {"N%d" % k: [60.0 + 20.0 * math.cos(k * math.pi / 3), 60.0 + 20.0 * math.sin(k * math.pi / 3), None]
           for k in range(6)}
    run2 = _runner_with(box)
    assert run2.safe_path("Left", (15.0, 15.0), (60.0, 60.0)) is None


def test_a_parked_robot_does_not_capture_the_tracker(cam_setup):
    w, sc, cam, _ = cam_setup
    mover, parked = FakeRobot(40.0, 60.0, 0.0), FakeRobot(62.0, 60.0, 0.0)
    w.robots[:] = []
    bg = [sc.render() for _ in range(3)]
    w.robots[:] = [mover, parked]
    tr = _tracker(cam)
    tr.set_background(bg)
    tr.park("P", parked.x, parked.y)
    tr.follow("M", mover.x, mover.y, t=0.0)
    t = 0.0
    for k in range(30):                     # it drives up next to the parked robot, then backs away
        t += 1 / 15.0
        mover.x += (0.5 if k < 15 else -0.5)
        s = tr.push(sc.render(), t)
        p = tr.where()
        assert math.hypot(p[1] - mover.x, p[2] - mover.y) < 4.0, (k, p)
