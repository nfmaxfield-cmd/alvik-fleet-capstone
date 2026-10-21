"""Tests for the E02 laptop runner (lane L): schedule, board geometry, state file, steering maths, lost /
stuck decisions, self-calibration, and the camera side (corner calibration, board lines, motion tracker).

    python -m pytest tools/experiment/tests/test_e02_runner.py -q
    E02_SLOW=1 python -m pytest tools/experiment/tests/test_e02_runner.py -q -k dry   # + a short dry run (~6 min)
"""
import json
import math
import os
import random
import subprocess
import sys
import tempfile

import numpy as np
import pytest

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02_plan as PL
import e02_steer as ST
import e02_fit as FIT


# =============================================================================== schedule
def test_schedule_block0_is_all_U_and_covers_every_robot_and_route():
    robots = ["Left", "Center", "Right"]
    s = PL.make_schedule(robots, 3, seed=1)
    b0 = [r for r in s if r["block"] == 0]
    assert len(b0) == 9 and {r["cond"] for r in b0} == {"U"}
    assert {(r["robot"], r["route"]) for r in b0} == {(rb, rt) for rb in robots for rt in PL.ROUTES}
    assert [r["order"] for r in s] == list(range(1, len(s) + 1))


def test_regular_blocks_are_balanced():
    robots = ["Left", "Center", "Right"]
    s = PL.make_schedule(robots, 3, seed=7)
    for b in (1, 2, 3):
        bl = PL.balance(s, blocks={b})
        assert all(v == 1 for v in bl["cond_route"].values()) and len(bl["cond_route"]) == 12
        assert all(v == 1 for v in bl["cond_robot"].values()) and len(bl["cond_robot"]) == 12
    three = PL.balance(s, blocks={1, 2, 3})
    assert len(three["cell"]) == 36 and set(three["cell"].values()) == {1}


def test_schedule_is_shuffled_and_reproducible():
    a = PL.make_schedule(["Left", "Center", "Right"], 2, seed=3)
    b = PL.make_schedule(["Left", "Center", "Right"], 2, seed=3)
    c = PL.make_schedule(["Left", "Center", "Right"], 2, seed=4)
    key = lambda s: [(r["robot"], r["cond"], r["route"]) for r in s]   # noqa: E731
    assert key(a) == key(b) and key(a) != key(c)


def test_blocks_for_two_hours():
    assert PL.blocks_for(2.0, 110.0) == 5          # 9 + 5 x 12 = 69 runs, about 2.1 h
    assert PL.blocks_for(0.1, 110.0) == 1


def test_next_run_waits_for_a_fit_then_falls_back_to_priors():
    s = [dict(order=1, block=1, robot="Left", cond="C", route=1, status="pending"),
         dict(order=2, block=1, robot="Right", cond="U", route=1, status="pending")]
    assert PL.next_run(s, lambda r: False)["order"] == 2            # C waits behind the U run
    s[1]["status"] = "done"
    assert PL.next_run(s, lambda r: False)["order"] == 1            # only C left: runs (on priors)
    assert PL.next_run(s, lambda r: True)["order"] == 1
    assert PL.next_run(s, lambda r: True, unavailable={"Left"}) is None


# =============================================================================== geometry
@pytest.mark.parametrize("xs,ys", [(0, 0), (0, 1), (1, 0), (1, 1)])
def test_routes_and_parks_stay_on_the_board_for_every_board_corner(xs, ys):
    bd = PL.Board(xboard_side=xs, yboard_side=ys)
    for r in PL.ROUTES:
        ok, room = bd.route_fits(r)
        assert ok, (xs, ys, r, room)
    sx, sy, sh = bd.nominal_start()
    for i in range(3):
        px, py, ph = bd.park(i)
        assert bd.on_board(px, py, margin=10)
        # every park spot is well clear of every route
        for r in PL.ROUTES:
            out, back = bd.route_floor(r, (sx, sy), sh)
            assert PL.dist_to_path((px, py), [(sx, sy)] + out + back) > 25
        # parked robots face the x-board
        assert abs(PL.wrap180(ph - math.degrees(math.atan2(-bd.uy, 0)))) < 1e-6


def test_far_corner_layout_start_spot_and_routes():
    """Nick's layout (2026-10-21): boards along x = max and y = max; the origin corner stays open."""
    bd = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=121.3, yboard_cm=121.55)
    sx, sy, sh = bd.nominal_start()
    assert sh == 180.0 and bd.turn_sign == 1.0 and bd.firmware_sides() == (1, 1)
    assert sx == pytest.approx(121.55 - 4.75 - PL.TURN_ROOM_CM) and sy == pytest.approx(121.3 - 4.75 - 8.0)
    out, back = bd.route_floor(1, (sx, sy), sh)
    assert out[-1][0] - 4.75 > 5.0                       # the straight's far front stays 5+ cm off the open edge
    assert all(bd.route_fits(r)[0] for r in PL.ROUTES)
    px, py, ph = bd.park(0)
    assert py < 25 and ph == 90.0                        # parked along the open y = 0 edge, facing the x-board


def test_route_far_points():
    bd = PL.Board()
    out, back = bd.route_floor(1, (10, 10), 0)
    assert out[-1] == pytest.approx((110, 10)) and back[-1] == pytest.approx((10, 10))
    out, back = bd.route_floor(2, (10, 10), 0)
    assert out[-1] == pytest.approx((40, 40))           # left turn: towards the board's middle
    out, back = bd.route_floor(3, (10, 10), 0)
    assert out[-1] == pytest.approx((60, 60)) and back[-1] == pytest.approx((10, 10))


def test_state_saves_loads_and_redoes_interrupted_runs():
    with tempfile.TemporaryDirectory() as d:
        st = PL.State(d)
        st["schedule"] = PL.make_schedule(["Left"], 1, 1)
        st["schedule"][0]["status"] = "running"
        st.save()
        st2 = PL.State.load(d)
        redo = st2.interrupted()
        assert len(redo) == 1 and st2["schedule"][0]["status"] == "pending"
        assert "interrupted" in st2["schedule"][0]["notes"][0]


# =============================================================================== steering
def test_plan_leg_basics():
    assert ST.plan_leg((0, 0), 0.0, (0.5, 0.2))[0] == "done"
    k, v = ST.plan_leg((0, 0), 0.0, (0, 50))
    assert k == "turn" and v == pytest.approx(90.0)
    k, v = ST.plan_leg((0, 0), 90.0, (0, 50))
    assert k == "go" and v == pytest.approx(25.0)               # longest leg
    k, v = ST.plan_leg((0, 0), 90.0, (0, 50), heading_measured=False)
    assert k == "go" and v == pytest.approx(10.0)               # short until the heading is measured again
    k, v = ST.plan_leg((0, 0), 0.0, (-3, 0))
    assert k == "go" and v == pytest.approx(-3.0)               # just behind, in line: back up
    assert ST.plan_leg((0, 0), None, (30, 0))[0] == "probe"


def drive_sim(steer, x, y, th, g=1.08, w=0.97, cam_sd=0.3, seed=0, max_legs=60):
    """Run Steer against a robot whose turns are g times what it was told and whose legs are w times."""
    rng = random.Random(seed)
    for _ in range(max_legs):
        cam = (x + rng.gauss(0, cam_sd), y + rng.gauss(0, cam_sd))
        cmd = steer.next(cam)
        if cmd[0] == "done":
            return x, y, th, steer.legs
        if cmd[0] == "turn":
            th += cmd[1] * g
        else:
            x += cmd[1] * w * math.cos(math.radians(th))
            y += cmd[1] * w * math.sin(math.radians(th))
        cam2 = (x + rng.gauss(0, cam_sd), y + rng.gauss(0, cam_sd))
        steer.after(cmd, cam, cam2)
    raise AssertionError("did not arrive")


@pytest.mark.parametrize("seed", range(5))
def test_steer_reaches_waypoints_despite_gyro_and_wheel_errors(seed):
    s = ST.Steer([(110, 10), (10, 10)], heading=8.0, measured=False, final_heading=0.0)
    x, y, th, legs = drive_sim(s, 10.0, 10.0, 0.0, seed=seed)
    assert math.hypot(x - 10, y - 10) < 1.8
    assert abs(PL.wrap180(th)) < 8
    assert legs < 40


def test_heading_from_track():
    pts = [(i * 0.04, 10 + i * 0.3, 20 + i * 0.3) for i in range(30)]
    assert ST.heading_from_track(pts) == pytest.approx(45.0, abs=0.01)
    curve = [(i * 0.04, 10 * math.cos(i / 10), 10 * math.sin(i / 10)) for i in range(30)]
    assert ST.heading_from_track(curve, max_rms_cm=0.3) is None
    assert ST.heading_from_track(pts[:5]) is None                # too short


def test_lost_and_stuck_decisions():
    assert ST.judge_run(3.0, 2.0)[0] == "ok"
    v, why = ST.judge_run(18.0, 2.0)
    assert v == "lost" and "far point" in why
    assert ST.judge_run(2.0, 2.0, seen=False)[0] == "unseen"
    assert ST.judge_run(2.0, 2.0, on_board=False)[0] == "off_board"
    assert ST.leg_stuck(("go", 10.0), (0, 0), (1.0, 0))
    assert not ST.leg_stuck(("go", 10.0), (0, 0), (9.0, 0))
    assert not ST.leg_stuck(("turn", 90.0), (0, 0), (0, 0))
    lines = ST.nick_instructions("Center", "it is stuck", "near its spot", "the board")
    assert lines[0].startswith("Center needs your help") and any("press Enter" in ln for ln in lines)


# =============================================================================== self-calibration
def observations(g, b, w, routes=(1, 2, 3), seed=0, cam_sd=0.3):
    obs = []
    for k, route in enumerate(routes):
        rb = KIT.KinRobot(g=g, b=b, w=w, seed=seed * 10 + k, cam_sd=cam_sd, t0=1000.0 + 500 * k)
        KIT.u_run(rb, route)
        ser = FIT.raw_series(rb.rawrows)
        obs += FIT.pair(FIT.camera_stretches(rb.cam), ser, run="r%d" % k)
    return obs


@pytest.mark.parametrize("g,b,w", [(1.075, 0.2, 0.978), (1.05, -0.3, 0.99), (1.10, 0.0, 0.965)])
def test_fit_recovers_gyro_scale_drift_and_wheels(g, b, w):
    obs = observations(g, b, w)
    res = FIT.fit_robot(obs)
    assert res["gyro_ok"] and res["wheel_ok"], res["reasons"]
    v = res["values"]
    assert v["gyro_scale"] == pytest.approx(g, abs=0.01)
    assert v["gyro_drift_dps"] == pytest.approx(g * b, abs=0.15)
    assert v["wheel_scale"] == pytest.approx(w, abs=0.006)


def test_fit_ignores_a_bad_stretch():
    obs = observations(1.075, 0.1, 0.978)
    obs[3] = dict(obs[3], phi=obs[3]["phi"] + 25.0, cam_len=obs[3]["cam_len"] * 1.3)   # a hand, a reflection
    v = FIT.fit_robot(obs)["values"]
    assert v["gyro_scale"] == pytest.approx(1.075, abs=0.012)
    assert v["wheel_scale"] == pytest.approx(0.978, abs=0.008)


def test_fit_refuses_insane_numbers_and_keeps_the_previous_ones():
    prev = dict(gyro_scale=1.07, gyro_drift_dps=0.1, wheel_scale=0.98)
    obs = observations(1.30, 0.0, 0.85, routes=(1, 3))
    res = FIT.fit_robot(obs, previous=prev)
    assert not res["gyro_ok"] and not res["wheel_ok"]
    assert res["values"] == prev
    assert any("outside the sane range" in r for r in res["reasons"])


def test_fit_says_when_there_are_not_enough_turns():
    rb = KIT.KinRobot()
    rb.go(40)
    rb.wait(1)
    rb.go(30)
    obs = FIT.pair(FIT.camera_stretches(rb.cam), FIT.raw_series(rb.rawrows), run="a")
    res = FIT.fit_robot(obs)
    assert not res["gyro_ok"] and res["values"] is None


def test_camera_stretches_handle_backwards_driving():
    rb = KIT.KinRobot(th=90.0)
    rb.go(20)
    rb.wait(0.5)
    rb.go(-15)
    obs = FIT.pair(FIT.camera_stretches(rb.cam), FIT.raw_series(rb.rawrows), run="a")
    assert len(obs) == 2
    assert obs[0]["forward"] and not obs[1]["forward"]
    assert PL.wrap180(obs[0]["phi"] - obs[1]["phi"]) == pytest.approx(0.0, abs=3.0)


# =============================================================================== camera side
class FakeWorld:
    def __init__(self, xs=1, ys=1, inset=0.6):
        import threading
        self.stop = threading.Event()
        self.board = PL.Board(xboard_side=xs, yboard_side=ys)
        bd = self.board
        bd.xboard_cm = (bd.y0 + inset) if xs == 0 else (bd.y1 - inset)
        bd.yboard_cm = (bd.x0 + inset) if ys == 0 else (bd.x1 - inset)
        self.robots = []

    @staticmethod
    def view(r):
        return r.x, r.y, r.th, 0.0


class FakeRobot:
    def __init__(self, x, y, th):
        self.x, self.y, self.th = x, y, th
        self.lights = (0.0, 0.0)


def sim_camera(world, tilt=3.0, yaw=1.5):
    import argparse
    import e02_sim
    a = argparse.Namespace(px_per_cm=2.34, tilt=tilt, yaw=yaw, seed=3, fps=15, pi_offset=0.0)
    return e02_sim.SimCamera(world, a)


@pytest.fixture(scope="module")
def cam_setup():
    import e02_camera as CAM
    w = FakeWorld()
    sc = sim_camera(w)
    d = tempfile.mkdtemp()
    true_path = sc.true_camera_model(os.path.join(d, "true.json"))
    return w, sc, CAM.load_camera(true_path), d


def test_true_camera_model_matches_the_renderer(cam_setup):
    w, sc, cam, _ = cam_setup
    for x, y in ((0, 0), (61, 61), (120, 30), (20, 110)):
        u0, v0 = sc.geom.project(x, y)
        u1, v1 = cam.project((x, y, 0.0))
        assert math.hypot(u0 - u1, v0 - v1) < 0.3


def test_corner_calibration_from_the_picture(cam_setup):
    import e02_camera as CAM
    w, sc, cam, d = cam_setup
    img = sc.render()
    corners, why = CAM.find_board_corners(img)
    assert corners is not None, why
    res = CAM.make_calibration(corners, (img.shape[1], img.shape[0]), os.path.join(d, "cam", "overhead.json"))
    made = CAM.load_camera(os.path.join(d, "cam", "overhead.json"))
    # the made calibration puts board points within ~2 cm of the truth (the blocked boards hide the edges a little)
    worst = 0.0
    for x, y in ((10, 10), (61, 61), (110, 20), (20, 110), (110, 110)):
        u, v = cam.project((x, y, 0.0))
        xm, ym = made.to_floor(u, v)
        worst = max(worst, math.hypot(xm - x, ym - y))
    assert worst < 2.5, worst
    assert res["err_cm"] < 1.0


def test_board_lines_found_with_the_true_camera(cam_setup):
    import e02_camera as CAM
    w, sc, cam, _ = cam_setup
    img = sc.render()
    for which, true in (("x", w.board.xboard_cm), ("y", w.board.yboard_cm)):
        m = CAM.measure_board_line(img, cam, PL.Board(xboard_side=1, yboard_side=1), which)
        assert m is not None and m["n"] >= 12
        assert m["value"] == pytest.approx(true, abs=0.6), (which, m)


def test_motion_tracker_follows_the_moving_robot_and_keeps_parked_ones(cam_setup):
    import e02_camera as CAM
    w, sc, cam, _ = cam_setup
    parked = FakeRobot(61.0, 18.0, 90.0)
    mover = FakeRobot(20.0, 40.0, 0.0)
    w.robots[:] = [parked, mover]
    tr = CAM.MotionTracker(cam)
    # the background is taken with nothing on the board, then both robots are known
    w.robots[:] = []
    tr.set_background([sc.render() for _ in range(3)])
    w.robots[:] = [parked, mover]
    tr.park("P", parked.x, parked.y)
    tr.follow("M", mover.x, mover.y, t=0.0)
    errs = []
    t = 0.0
    for k in range(120):
        t += 1 / 15.0
        mover.x += 8.0 / 15.0                      # 8 cm/s along x
        s = tr.push(sc.render(), t)
        assert s is not None, k
        errs.append(math.hypot(s[1] - mover.x, s[2] - mover.y))
    assert float(np.median(errs)) < 1.0 and max(errs) < 2.0
    # the parked robot is still seen after all that background updating
    blobs = tr.blobs()
    assert any(math.hypot(b[3] - parked.x, b[4] - parked.y) < 2.0 for b in blobs)


def test_still_position():
    import e02_camera as CAM
    smp = [(i * 0.04, 10 + 0.05 * (i % 3), 20.0) for i in range(30)]
    assert CAM.still_position(smp) == pytest.approx((10.05, 20.0), abs=0.06)
    moving = [(i * 0.04, 10 + i * 0.4, 20.0) for i in range(30)]
    assert CAM.still_position(moving) is None


# =============================================================================== the whole thing
@pytest.mark.skipif(not os.environ.get("E02_SLOW"), reason="slow: set E02_SLOW=1 (about 6 minutes)")
def test_dry_run_short_session():
    repo = os.path.dirname(os.path.dirname(os.path.dirname(KIT.EXP)))
    cmd = [sys.executable, os.path.join(KIT.EXP, "e02.py"), "--dry-run", "--dry-true-calib", "--blocks", "1", "--routes",
           "1", "--naming-s", "10", "--skip-find-check", "--max-runs", "5", "--seed", "11"]
    out = subprocess.run(cmd, cwd=repo, capture_output=True, text=True, timeout=1200)
    assert "Saved in" in out.stdout, out.stdout[-3000:] + out.stderr[-3000:]
    sdir = out.stdout.split("Saved in")[-1].strip().splitlines()[0].strip()
    rep = json.load(open(os.path.join(sdir, "dry_run_report.json")))
    assert rep["summary"]["runs_done"] >= 4
    assert rep["summary"]["track_err_p95_cm"] < 3.0
