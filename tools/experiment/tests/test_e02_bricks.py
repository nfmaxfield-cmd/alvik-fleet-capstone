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
    assert "watchdog" in stop("failed (stop command)", (1, 1))
    assert stop("camera and wheels disagree (x)", (1, 1)) == ""
    assert "lost" in stop("camera and wheels disagree (x)", None)
    assert "did not run" in stop("refused (busy)", (1, 1))


def test_the_brick_options_are_off_by_default():
    """Nick runs the pushed code this morning: without --bricks / --bricks-only nothing changes."""
    import e02
    src = open(e02.__file__).read()
    assert '"--bricks", action="store_true"' in src and '"--bricks-only", action="store_true"' in src
    assert 'getattr(a, "bricks_only", False)' in src and 'getattr(a, "bricks", False)' in src


# =============================================================================== odd objects (Nick, 2026-10-22)
def _box(cx, cy, L, W, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [(cx + c * u - s * v, cy + s * u + c * v) for u, v in ((L / 2, W / 2), (-L / 2, W / 2), (-L / 2, -W / 2),
                                                                  (L / 2, -W / 2))]


def _paint_box(img, cam, corners, h, top, side):
    """An object h cm tall on the floor polygon `corners`, drawn as the camera sees it: its sides, then its top."""
    import cv2
    out = img.copy()
    for k in range(len(corners)):
        a, b = corners[k], corners[(k + 1) % len(corners)]
        q = np.array([cam.project((p[0], p[1], z)) for p, z in ((a, 0.0), (b, 0.0), (b, h), (a, h))])
        cv2.fillPoly(out, [np.round(q * 16).astype(np.int32)], side, lineType=cv2.LINE_AA, shift=4)
    q = np.array([cam.project((p[0], p[1], h)) for p in corners])
    cv2.fillPoly(out, [np.round(q * 16).astype(np.int32)], top, lineType=cv2.LINE_AA, shift=4)
    return out


def _floor(cam_setup):
    import e02_camera as CAM
    w, sc, cam, _ = cam_setup
    w.robots[:] = []
    tr = CAM.MotionTracker(cam, board_rect=RECT)
    tr.set_background([sc.render() for _ in range(5)])
    return sc, cam, BR.floor_snapshot(tr)


def _footprint_cover(found, corners):
    """Share of the object's true footprint cells (1 cm in from its edge) that are occupied."""
    import cv2
    g = found["grid"]
    X, Y = g.centres()
    poly = np.array(corners, np.float32)
    p = np.array(corners)
    w = min(np.hypot(*(p[0] - p[1])), np.hypot(*(p[1] - p[2])))
    inside = np.array([cv2.pointPolygonTest(poly, (float(x), float(y)), True) >= min(1.0, 0.3 * w)
                       for x, y in zip(X.ravel(), Y.ravel())]).reshape(g.shape)
    return float(found["occ"][inside].mean())


@pytest.mark.parametrize("name, corners, h, top, side, kind, shade", [
    # a charging cable, 2.5 mm thin and only a little lighter than the plywood: the old opening lost it
    ("cable", _box(60, 40, 60, 0.25, 20), 0.25, 200, 200, "long and thin", "brighter"),
    ("pen", _box(40, 80, 14, 1.0, -30), 1.0, 245, 220, "long and thin", "brighter"),
    ("shoe", _box(85, 40, 28, 10, 70), 10.0, 30, 20, "", "darker"),
    # a bottle 25 cm tall: the camera sees its top leaning away; the base must still be covered
    ("bottle", _box(90, 85, 7, 7, 0), 25.0, 200, 120, "", "brighter"),
])
def test_odd_objects_are_found_whole(cam_setup, name, corners, h, top, side, kind, shade):
    sc, cam, floor = _floor(cam_setup)
    frames = [_paint_box(sc.render(), cam, corners, h, top, side) for _ in range(6)]
    found = BR.find_bricks(cam, frames, floor, RECT)
    assert len(found["bricks"]) == 1, (name, found["bricks"])            # one piece, not broken up
    assert not found["too_much"], found["thr_raised"]
    b = found["bricks"][0]
    assert b["kind"] == kind and b["shade"] == shade, b
    c = np.array(corners).mean(axis=0)
    assert math.hypot(b["x"] - c[0], b["y"] - c[1]) < (6.0 if h > 20 else 3.0), b
    if h >= 1.0:
        assert _footprint_cover(found, corners) > 0.97, name
    else:                                                                 # a cable: its line is covered end to end
        L = max(math.hypot(corners[0][0] - corners[1][0], corners[0][1] - corners[1][1]),
                math.hypot(corners[1][0] - corners[2][0], corners[1][1] - corners[2][1]))
        assert b["length_cm"] >= L - 2.0, b
    text = " ".join(BR.brick_words(found["bricks"], found))
    assert "found 1 object" in text and shade in text


def test_specks_are_left_out_and_said(cam_setup):
    """A speck (a single dark pixel: dust, a crumb) is not an object, and the message says it was left out."""
    sc, cam, floor = _floor(cam_setup)
    u, v = (int(round(t)) for t in cam.project((50.0, 60.0, 0.0)))

    def dot(img):
        out = img.copy()
        out[v, u] = 20
        return out
    found = BR.find_bricks(cam, [dot(sc.render()) for _ in range(6)], floor, RECT)
    assert found["bricks"] == [] and len(found["specks"]) == 1
    text = " ".join(BR.brick_words(found["bricks"], found))
    assert "no objects" in text and "Left out: 1 speck" in text
    # a 1 x 1 cm thing is an object (a robot could catch it), listed as small
    frames = [_paint_box(sc.render(), cam, _box(50, 60, 1.0, 1.0, 0), 0.5, 20, 20) for _ in range(6)]
    found = BR.find_bricks(cam, frames, floor, RECT)
    assert len(found["bricks"]) == 1 and found["bricks"][0]["kind"] == "small"


def test_a_light_change_is_not_taken_for_objects(cam_setup):
    sc, cam, floor = _floor(cam_setup)
    img = sc.render()
    ramp = np.linspace(-70, 70, img.shape[1])[None, :]
    frames = [np.clip(sc.render().astype(np.float32) + ramp, 0, 255).astype(np.uint8) for _ in range(6)]
    found = BR.find_bricks(cam, frames, floor, RECT)
    assert found["too_much"]
    assert "the light changed or the camera moved" in " ".join(BR.brick_words(found["bricks"], found))


# =============================================================================== tight gap from measured precision
def _runs(robot, cond, errs, status="done"):
    return [dict(robot=robot, cond=cond, status=status, far_err_cm=f, end_err_cm=e) for f, e in errs]


def test_clearance_comes_from_the_measured_precision():
    rows = (_runs("Left", "KC", [(0.5, 1.0), (0.8, 1.2), (0.6, 2.0), (1.1, 0.9)]) +
            _runs("Left", "C", [(2.0, 3.0), (2.5, 4.0), (3.0, 6.0)]) +
            _runs("Left", "U", [(9.0, 12.0)]) + _runs("Right", "U", [(8.0, 10.0), (11.0, 15.0)]) +
            _runs("Left", "KC", [(30.0, 30.0)], status="pending") + _runs("Left", "K", [(0.1, 0.1)] * 5))
    kc = BR.gap_precision(rows, "Left", "camera")
    vals = [0.5, 1.0, 0.8, 1.2, 0.6, 2.0, 1.1, 0.9]
    p95 = float(np.percentile(vals, 95))
    assert kc["cond"] == "KC" and kc["runs"] == 4 and kc["n"] == 8          # pending and K runs left out
    assert kc["p95_cm"] == pytest.approx(p95, abs=0.01)
    assert kc["clearance_cm"] == pytest.approx(BR.ROBOT_HALF_CM + BR.GAP_K * p95, abs=0.01)
    assert kc["gap_cm"] == pytest.approx(2 * kc["clearance_cm"], abs=0.06) and "this robot" in kc["source"]
    # k scales it
    k2 = BR.gap_precision(rows, "Left", "camera", k=2.0)
    assert k2["clearance_cm"] == pytest.approx(BR.ROBOT_HALF_CM + 2.0 * p95, abs=0.01)
    # calibrated needs more room than camera fixes; too few of its own U runs: all robots' U runs
    c = BR.gap_precision(rows, "Left", "calibrated")
    u = BR.gap_precision(rows, "Left", "factory")
    assert kc["gap_cm"] < c["gap_cm"] < u["gap_cm"]
    assert u["runs"] == 3 and "all robots" in u["source"]
    # no data at all: the planner's chosen 3.5 cm
    none = BR.gap_precision([], "Left", "camera")
    assert none["p95_cm"] is None and none["margin_cm"] == BR.MARGIN_CM
    assert none["gap_cm"] == pytest.approx(2 * (BR.ROBOT_HALF_CM + BR.MARGIN_CM), abs=0.06)
    words = " ".join(BR.precision_words("Left", dict(factory=u, calibrated=c, camera=kc)))
    assert "The narrowest gap Left can safely make with its calibration is %d cm" % math.ceil(kc["gap_cm"]) in words
    assert "95th-percentile" in words


def test_the_tight_gap_is_found_and_crossed_straight_through_its_middle():
    # two objects 16 cm apart (edge to edge) across the middle, a third further off
    occ, g = _occ_with([(50.0, 30.0, 60.0, 52.0), (50.0, 68.0, 60.0, 90.0), (90.0, 95.0, 96.0, 101.0)])
    bricks = BR.bricks_from(occ, g)
    gaps = BR.find_gaps(occ, g, bricks)
    gap = gaps[0]
    assert gap["width_cm"] == pytest.approx(16.0, abs=1.01)
    assert gap["mid"][1] == pytest.approx(60.0, abs=1.0) and 50.0 <= gap["mid"][0] <= 60.0
    assert abs(gap["normal"][0]) > 0.95                          # the way through runs along x
    robots = {"Left": BOARD.park(0)[:2], "Center": BOARD.park(1)[:2], "Right": BOARD.park(2)[:2]}
    clear = {"Left": 6.0, "Center": 9.5, "Right": 7.0}             # Left is the most precise
    ch = BR.choose_gap_crossing(occ, g, BOARD, robots, gap, clear)
    assert ch is not None and ch["robot"] == "Left" and len(ch["path"]["points"]) == 2
    (sx, sy), (gx, gy) = ch["path"]["points"]
    assert abs(sy - 60.0) < 6.0 and abs(gy - 60.0) < 6.0 and (sx - 55.0) * (gx - 55.0) < 0   # across, through it
    assert ch["path"]["min_room_cm"] >= 6.0 + 0.5 - 0.01                    # its clearance kept (to cell centres)
    # nobody precise enough for this gap: no crossing
    assert BR.choose_gap_crossing(occ, g, BOARD, robots, gap, {n: 8.5 for n in robots}) is None
    text = " ".join(BR.gap_words(gaps, 12.0, "Left"))
    assert "safe for Left" in text and "objects" in text
    assert "too narrow" in " ".join(BR.gap_words(gaps, 30.0, "Left"))
    ch.update(clearance_cm=6.0)
    assert "straight through the 16" in " ".join(BR.plan_words(ch))


def test_a_third_object_inside_a_gap_closes_it():
    occ, g = _occ_with([(50.0, 30.0, 60.0, 52.0), (50.0, 68.0, 60.0, 90.0), (53.0, 58.0, 57.0, 62.0)])
    gaps = BR.find_gaps(occ, g)
    assert all(not (gp["width_cm"] > 14.0 and abs(gp["mid"][1] - 60.0) < 2.0) for gp in gaps), gaps


def test_the_watchdog_guards_the_objects():
    occ, g = _occ_with([(50.0, 40.0, 60.0, 60.0)])
    sp = BR.free_space(occ, g, BOARD)
    # 20 pictures a second, 10 cm/s
    towards = [(t * 0.05, 70.0 - 0.5 * t, 50.0) for t in range(14)]      # at the brick's face (x = 60)
    assert "closing" in BR.object_hazard(towards, sp)
    away = [(t * 0.05, 61.0 + 0.5 * t, 50.0) for t in range(14)]
    assert BR.object_hazard(away, sp) is None                             # backing out is always allowed
    still = [(t * 0.05, 64.0, 50.0) for t in range(14)]
    assert BR.object_hazard(still, sp) is None
    far = [(t * 0.05, 95.0 - 0.5 * t, 50.0) for t in range(14)]
    assert BR.object_hazard(far, sp) is None
    alongside = [(t * 0.05, 66.0, 20.0 + 0.5 * t) for t in range(14)]   # passing 6 cm from it: fine
    assert BR.object_hazard(alongside, sp) is None


def test_back_off_after_a_front_stop():
    occ, g = _occ_with([(50.0, 40.0, 60.0, 60.0)])
    sp = BR.free_space(occ, g, BOARD)
    assert BR.back_off_cm(sp, (67.0, 50.0), 180.0) == pytest.approx(12.0)   # facing the brick: straight back
    occ2, _ = _occ_with([(50.0, 40.0, 60.0, 60.0), (76.0, 40.0, 80.0, 60.0)])  # something behind it too
    sp2 = BR.free_space(occ2, g, BOARD)
    d = BR.back_off_cm(sp2, (67.0, 50.0), 180.0)
    assert 0.0 < d < 6.0 and 76.0 - (67.0 + d) >= BR.ROBOT_HALF_CM


def test_the_ladder_table():
    cs = [dict(n=1, method="factory", how=BR.METHOD_WORDS["factory"], end_err_cm=None, legs_done=1,
               failed="its front stop saw something; the safety stopped it, so this crossing counts as failed",
               closest=dict(centre_cm=7.0, body_cm=2.25)),
          dict(n=2, method="calibrated", how=BR.METHOD_WORDS["calibrated"], end_err_cm=2.4, legs_done=3, failed="",
               closest=dict(centre_cm=9.0, body_cm=4.25)),
          dict(n=3, method="camera", how=BR.METHOD_WORDS["camera"], end_err_cm=0.4, legs_done=5, failed="",
               closest=dict(centre_cm=10.0, body_cm=5.25))]
    lines = BR.ladder_words(cs)
    assert len(lines) == 5 and "factory settings" in lines[2] and "FAILED" in lines[2]
    assert "2.4 cm" in lines[3] and "0.4 cm" in lines[4] and "finished" in lines[4]
    assert BR.LADDER == ("factory", "calibrated", "camera") and BR.METHOD_COND["factory"] == "U"


def test_the_tight_gap_option_is_off_by_default():
    import e02
    src = open(e02.__file__).read()
    assert '"--tight-gap", action="store_true"' in src and 'getattr(self.a, "tight_gap", False)' in src
    assert "BR.object_hazard(smp, self.bricks_guard)" in src        # the watchdog guards the objects


def test_after_a_tight_crossing_the_way_back_is_through_the_gap():
    """At the goal beyond a tight gap, with no way round the objects: the camera-steered way back to the start may
    use the gap's own line (the corridor), and nothing else that is tight."""
    occ, g = _occ_with([(33.0, 53.0, 57.0, 67.0), (74.0, 53.0, 98.0, 67.0), (88.0, 85.0, 104.0, 95.0)])
    robots = {"Left": BOARD.park(0)[:2], "Center": BOARD.park(1)[:2], "Right": BOARD.park(2)[:2]}
    gap = [gp for gp in BR.find_gaps(occ, g) if gp["width_cm"] > 16.5][0]
    ch = BR.choose_gap_crossing(occ, g, BOARD, robots, gap, {n: 6.25 for n in robots})
    others = [p for n, p in robots.items() if n != ch["robot"]]
    sp = BR.free_space(occ, g, BOARD, others=others)
    end = (65.0, 87.0)
    assert not BR.plan_path(sp, end, ch["start"], escape_cm=25.0)["ok"]          # no way round
    cs = BR.corridor_space(sp, BR.gap_space(occ, g, BOARD, others, 6.25), ch["path"]["points"])
    back = BR.plan_path(cs, end, ch["start"], escape_cm=25.0)
    assert back["ok"] and back["min_room_cm"] >= 6.25 + 0.5 - 0.01
    X, Y = g.centres()
    off_line = np.abs(X - 65.5) > 3.5
    assert not (cs["free"] & ~sp["free"] & off_line).any()                     # tight only along the line
