"""Tests for the brick demo (e02_bricks.py, the simulator's bricks, the runner's options).

    python -m pytest tools/experiment/tests/test_e02_bricks.py -q
"""
import math
import os

import numpy as np
import pytest

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02_plan as PL
import e02_bricks as BR
from test_e02_runner import FakeRobot, FakeWorld, sim_camera  # noqa: F401
from test_e02_runner import cam_setup  # noqa: F401  (fixture)

BOARD = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=121.3, yboard_cm=121.55)
RECT = (BOARD.x0, BOARD.y0, BOARD.x1, BOARD.y1)


def _occ_with(boxes, rect=RECT):
    """A grid with axis-aligned bricks: boxes [(x0, y0, x1, y1)] cm."""
    g = BR.Grid(rect)
    X, Y = g.centres()
    occ = np.zeros(g.shape, bool)
    for x0, y0, x1, y1 in boxes:
        occ |= (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
    return occ, g


# =============================================================================== finding the bricks
def _brick_world(w, sc, specs):
    import e02_sim
    w.brick_polys = [e02_sim.brick_corners(b) for b in e02_sim.parse_bricks(specs)]
    w.brick_h = 6.0
    w.bricks_on = False
    sc.bricks_drawn = False
    sc.bg_plain = sc.bg if not hasattr(sc, "bg_plain") else sc.bg_plain


def test_simulator_bricks_spec_and_corners():
    import e02_sim
    b = e02_sim.parse_bricks("40,60,20,10;80,30,20,10,90")
    assert len(b) == 2 and b[1]["deg"] == 90.0
    c = np.array(e02_sim.brick_corners(b[0]))
    assert c[:, 0].min() == pytest.approx(30.0) and c[:, 1].max() == pytest.approx(65.0)
    c = np.array(e02_sim.brick_corners(b[1]))           # turned 90 deg: long along y
    assert np.ptp(c[:, 1]) == pytest.approx(20.0) and np.ptp(c[:, 0]) == pytest.approx(10.0)
    assert e02_sim.parse_bricks("") == []
    with pytest.raises(ValueError):
        e02_sim.parse_bricks("1,2,3")


def test_bricks_found_where_they_are_and_robots_are_not_bricks(cam_setup):
    """The camera picture before and after the bricks go down: each brick's footprint is covered, the robots
    standing on the board are not taken for bricks, and nothing else shows up."""
    import e02_camera as CAM
    w, sc, cam, _ = cam_setup
    robots = [FakeRobot(100.0, 20.0, 90.0), FakeRobot(20.0, 100.0, 0.0)]
    w.robots[:] = robots
    _brick_world(w, sc, "45,60,20,10;80,45,20,10,90")
    try:
        tr = CAM.MotionTracker(cam, board_rect=RECT)
        tr.set_background([sc.render() for _ in range(5)], robots=[(r.x, r.y) for r in robots])
        floor = BR.floor_snapshot(tr)
        w.bricks_on = True
        frames = [sc.render() for _ in range(6)]
        found = BR.find_bricks(cam, frames, floor, RECT, robots=[(r.x, r.y) for r in robots])
    finally:
        w.bricks_on = False
        w.robots[:] = []
    bricks = found["bricks"]
    assert len(bricks) == 2, bricks
    occ, g = found["occ"], found["grid"]
    truth, _ = _occ_with([(35.0, 55.0, 55.0, 65.0), (75.0, 35.0, 85.0, 55.0)])
    inner, _ = _occ_with([(35.5, 55.5, 54.5, 64.5), (75.5, 35.5, 84.5, 54.5)])
    assert occ[inner].mean() > 0.97                       # the footprint is covered
    # beyond the footprint only a rim (the camera sees the top and a side, projected to the floor: conservative)
    d_out = BR.brick_distance(truth, g)
    assert float(d_out[occ].max()) < 4.0, float(d_out[occ].max())
    for b, (x, y) in zip(sorted(bricks, key=lambda b: b["x"]), ((45.0, 60.0), (80.0, 45.0))):
        assert math.hypot(b["x"] - x, b["y"] - y) < 3.0, b
        assert 17.0 <= b["length_cm"] <= 27.0 and 8.0 <= b["width_cm"] <= 16.0, b
    for r in robots:                                       # no brick near a robot
        assert all(math.hypot(b["x"] - r.x, b["y"] - r.y) > 15.0 for b in bricks)


def test_no_bricks_no_change(cam_setup):
    import e02_camera as CAM
    w, sc, cam, _ = cam_setup
    w.robots[:] = []
    tr = CAM.MotionTracker(cam, board_rect=RECT)
    tr.set_background([sc.render() for _ in range(5)])
    found = BR.find_bricks(cam, [sc.render() for _ in range(6)], BR.floor_snapshot(tr), RECT)
    assert found["bricks"] == [] and not found["occ"].any()


def test_a_hand_still_in_the_picture_is_seen_moving(cam_setup):
    from test_e02_fixbuild import _paint_hand
    w, sc, cam, _ = cam_setup
    w.robots[:] = []
    a = [sc.render() for _ in range(6)]
    b = [_paint_hand(sc.render(), cam, 50.0, 60.0) for _ in range(6)]
    region = BR.board_mask(cam, a[0].shape, RECT) > 0
    assert BR.moving_share(a, [sc.render() for _ in range(6)], region) < 0.004
    assert BR.moving_share(a, b, region) > 0.004


# =============================================================================== planning
def test_path_goes_through_the_gap_and_keeps_its_room():
    """A wall of bricks across the board with one 30 cm gap: the path goes through the gap, its centre never
    nearer a brick than the room kept (robot corner + margin), and it is a few straight legs."""
    occ, g = _occ_with([(50.0, 20.0, 60.0, 45.0), (50.0, 75.0, 60.0, 105.0)])
    sp = BR.free_space(occ, g, BOARD)
    plan = BR.plan_path(sp, (95.0, 40.0), (30.0, 40.0))
    assert plan["ok"], plan["why"]
    assert plan["min_room_cm"] >= sp["r_cm"] - 0.01
    assert 2 <= len(plan["points"]) <= 6
    gap_y = [p[1] for a, b in zip(plan["points"], plan["points"][1:]) for p in (a, b)]
    # it must cross x = 55 between y = 45 and y = 75
    crossed = False
    for a, b in zip(plan["points"], plan["points"][1:]):
        if (a[0] - 55.0) * (b[0] - 55.0) <= 0 and a[0] != b[0]:
            y = a[1] + (55.0 - a[0]) * (b[1] - a[1]) / (b[0] - a[0])
            crossed = 45.0 + sp["r_cm"] - 0.5 <= y <= 75.0 - sp["r_cm"] + 0.5
    assert crossed, plan["points"]
    assert gap_y


def test_no_way_through_a_closed_wall():
    occ, g = _occ_with([(50.0, -5.0, 60.0, 130.0)])
    sp = BR.free_space(occ, g, BOARD)
    plan = BR.plan_path(sp, (95.0, 40.0), (30.0, 40.0))
    assert not plan["ok"] and "no way" in plan["why"]


def test_open_board_is_one_straight_leg_and_edges_robots_are_kept_clear():
    occ, g = _occ_with([])
    sp = BR.free_space(occ, g, BOARD, others=[(60.0, 60.0)])
    plan = BR.plan_path(sp, (95.0, 30.0), (30.0, 30.0))
    assert plan["ok"] and len(plan["points"]) == 2
    # the far side of a robot in the way: the path bends round it, never within ROBOT_CLEAR_CM of its centre
    plan = BR.plan_path(sp, (95.0, 60.0), (30.0, 60.0))
    assert plan["ok"] and len(plan["points"]) >= 3
    pts = plan["points"]
    for a, b in zip(pts, pts[1:]):
        for t in np.linspace(0, 1, 50):
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            assert math.hypot(x - 60.0, y - 60.0) >= BR.ROBOT_CLEAR_CM - 1.0
            # open edges here: x = 0 and y = 0
            assert x >= BR.OPEN_EDGE_CLEAR_CM - 1.0 and y >= BR.OPEN_EDGE_CLEAR_CM - 1.0


def test_a_start_in_a_tight_spot_can_still_leave_it():
    """Where a front stop left it (close to a brick): the path may leave that spot, the goal must be clear."""
    occ, g = _occ_with([(50.0, 40.0, 60.0, 60.0)])
    sp = BR.free_space(occ, g, BOARD)
    start = (69.0, 50.0)                                  # 9 cm from the brick: not free, but not touching it
    assert not g.at(sp["free"], *start) and g.at(sp["soft"], *start)
    plan = BR.plan_path(sp, start, (90.0, 90.0))
    assert plan["ok"], plan["why"]
    assert not BR.plan_path(sp, (90.0, 90.0), start)["ok"]


def test_legs_drive_the_path_and_an_uncorrected_gyro_misses():
    pts = [(95.0, 40.0), (70.0, 60.0), (40.0, 60.0), (30.0, 40.0)]
    legs = BR.legs_from(pts[0], 180.0, pts)
    kinds = [lg["cmd"][0] for lg in legs]
    assert kinds.count("go") == 3 and kinds[0] == "turn"
    x, y, _ = BR.dead_reckon(pts[0], 180.0, legs)
    assert math.hypot(x - 30.0, y - 40.0) < 0.05
    x, y, _ = BR.dead_reckon(pts[0], 180.0, legs, gyro_err=0.93)     # a gyro that counts 93% of every turn
    assert math.hypot(x - 30.0, y - 40.0) > 2.0
    long = BR.legs_from((10.0, 10.0), 0.0, [(10.0, 10.0), (110.0, 10.0)], max_go=40.0)
    assert [lg["cmd"] for lg in long] == [("go", 33.33)] * 3


def test_crossing_is_chosen_through_the_bricks_with_a_robot_that_can_reach_it():
    occ, g = _occ_with([(55.0, 35.0, 65.0, 55.0), (35.0, 70.0, 55.0, 80.0)])
    robots = {"Left": BOARD.park(0)[:2], "Center": BOARD.park(1)[:2], "Right": BOARD.park(2)[:2]}
    ch = BR.choose_crossing(occ, g, BOARD, robots)
    assert ch is not None
    assert ch["straight_blocked_cm"] > 0                  # the straight line runs through the bricks
    assert ch["path"]["ok"] and len(ch["path"]["points"]) >= 3
    s, gl = ch["start"], ch["goal"]
    assert math.hypot(s[0] - gl[0], s[1] - gl[1]) > 60.0  # across the board
    assert ch["path"]["min_room_cm"] >= ch["space"]["r_cm"] - 0.01
    assert ch["approach"]["ok"] and ch["robot"] in robots
    for n, p in robots.items():
        if n != ch["robot"]:
            assert BR.min_room(dict(ch["space"], dist=np.hypot(*[a - b for a, b in zip(g.centres(), p)]).astype(
                np.float32)), ch["path"]["points"]) >= BR.ROBOT_CLEAR_CM - 1.0
    words = BR.plan_words(ch)
    assert ch["robot"] in words[0] and "round them" in words[2]


def test_closest_approach_and_words():
    occ, g = _occ_with([(50.0, 40.0, 60.0, 60.0)])
    sp = BR.free_space(occ, g, BOARD)
    ca = BR.closest_approach(sp, [(0.0, 80.0, 50.0), (1.0, 70.0, 50.0), (2.0, 70.0, 70.0)])
    assert ca["centre_cm"] == pytest.approx(10.0, abs=1.0) and ca["at"] == (70.0, 50.0)
    assert ca["body_cm"] == pytest.approx(10.0 - BR.ROBOT_HALF_CM, abs=1.0)
    c = dict(n=1, how="wheels and gyro only", end_err_cm=4.2, robot_thought_cm=0.6, closest=ca, failed="",
             legs_done=5, seconds=40.0)
    text = " ".join(BR.crossing_words(c))
    assert "4.2 cm from the goal" in text and "0.6 cm" in text and "all 5 legs" in text
    c["failed"] = "its front stop saw something"
    assert "did NOT finish" in " ".join(BR.crossing_words(c))
    assert BR.closest_approach(BR.free_space(*_occ_with([]), BOARD), [(0, 50, 50)]) is None


# =============================================================================== the runner keeps its defaults
def test_runner_leg_results_that_end_a_crossing():
    import e02
    stop = e02.Runner.crossing_stop
    assert stop("done", (1, 1)) == "" and stop("dup", (1, 1)) == ""
    assert "front stop" in stop("failed (obstacle 6 cm)", (1, 1))
    assert "edge guard" in stop("failed (plywood edge (L))", (1, 1))
    assert "watchdog" in stop("watchdog (9 cm from the open edge)", (1, 1))
    assert stop("camera and wheels disagree (x)", (1, 1)) == ""
    assert "lost" in stop("camera and wheels disagree (x)", None)
    assert "did not run" in stop("refused (busy)", (1, 1))


def test_the_brick_options_are_off_by_default():
    """Nick runs the pushed code this morning: without --bricks / --bricks-only nothing changes."""
    import e02
    src = open(e02.__file__).read()
    assert '"--bricks", action="store_true"' in src and '"--bricks-only", action="store_true"' in src
    assert 'getattr(a, "bricks_only", False)' in src and 'getattr(a, "bricks", False)' in src
