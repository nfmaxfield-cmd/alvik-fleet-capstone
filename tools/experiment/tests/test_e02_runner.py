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


def test_time_plan_keeps_about_two_hours_and_lists_runs_per_condition():
    nb = PL.blocks_for_time(2.0)                                  # 70 s run + 30 s find, 10 min setup
    s = PL.make_schedule(["Left", "Center", "Right"], nb, seed=1)
    total = PL.SETUP_S + len(s) * (PL.RUN_S + PL.FIND_S)
    assert 1.75 * 3600 <= total <= 2.0 * 3600 + 300
    lines = PL.plan_words(s)
    assert lines[0].startswith("Plan: %d runs" % len(s))
    for c in PL.CONDITIONS:
        n = sum(1 for r in s if r["cond"] == c)
        assert any(ln.strip().startswith(c + " ") and "%d runs" % n in ln for ln in lines), (c, lines)
    assert "about 2:0" in lines[-1] or "about 1:5" in lines[-1]


def test_next_run_waits_for_a_fit_then_falls_back_to_priors():
    s = [dict(order=1, block=1, robot="Left", cond="C", route=1, status="pending"),
         dict(order=2, block=1, robot="Right", cond="U", route=1, status="pending")]
    assert PL.next_run(s, lambda r: False)["order"] == 2            # C waits behind the U run
    s[1]["status"] = "done"
    assert PL.next_run(s, lambda r: False)["order"] == 1            # only C left: runs (on priors)
    assert PL.next_run(s, lambda r: True)["order"] == 1
    assert PL.next_run(s, lambda r: True, unavailable={"Left"}) is None
    s = [dict(order=1, block=1, robot="Left", cond="U", route=1, status="pending", find_redos=1),
         dict(order=2, block=1, robot="Right", cond="U", route=2, status="pending")]
    assert PL.next_run(s, lambda r: True)["order"] == 2            # a run whose find failed waits to the block's end


# =============================================================================== geometry
@pytest.mark.parametrize("xs,ys", [(0, 0), (0, 1), (1, 0), (1, 1)])
def test_routes_and_parks_stay_on_the_board_for_every_board_corner(xs, ys):
    bd = PL.Board(xboard_side=xs, yboard_side=ys)
    for r in PL.ROUTES:
        ok, open_room, board_room = bd.route_fits(r)
        assert ok and open_room >= 15, (xs, ys, r, open_room, board_room)
    sx, sy, sh = bd.nominal_start()
    fx, fy = bd.find_stage()
    spots = [bd.park(i) for i in range(3)]
    for i, (px, py, ph) in enumerate(spots):
        assert bd.on_board(px, py, margin=12)
        assert bd.on_board(*bd.stage(i), margin=12)
        # every park spot is well clear of every route (nominal; the worst-case drift is checked in the simulator)
        for r in PL.ROUTES:
            out, back = bd.route_floor(r, (sx, sy))
            assert PL.dist_to_path((px, py), [(sx, sy)] + out + back) > 25, (xs, ys, i, r)
        # parked robots face the find's staging point (their next drive), so they never face an edge
        assert abs(PL.wrap180(ph - math.degrees(math.atan2(fy - py, fx - px)))) < 1e-6
        # the other robots' drives from their spots to the find (and the parking drive back) keep clear of it
        for j in range(3):
            if j != i:
                qx, qy, _ = spots[j]
                path = [(qx, qy), bd.stage(j), (fx, fy)]
                assert PL.dist_to_path((px, py), path) > PL.PARK_CLEAR_CM, (xs, ys, i, j)
                assert PL.dist_to_path((px, py), [(sx, sy), bd.stage(j), (qx, qy)]) > PL.PARK_CLEAR_CM


def test_far_corner_layout_start_spot_and_routes():
    """Nick's layout (2026-10-21): boards along x = max and y = max; the origin corner stays open."""
    bd = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=121.3, yboard_cm=121.55)
    sx, sy, sh = bd.nominal_start()
    assert sh == 180.0 and bd.turn_sign == 1.0 and bd.firmware_sides() == (1, 1)
    assert sx == pytest.approx(121.55 - PL.START_CLEAR_CM) and sy == pytest.approx(121.3 - PL.START_CLEAR_CM)
    assert bd.route_heading(1) == pytest.approx(-135.0)  # the straight runs along the diagonal
    out, back = bd.route_floor(1, (sx, sy))
    assert min(out[-1]) - 4.75 > 15.0                    # its far front stays 15+ cm off both open edges
    assert all(bd.route_fits(r)[0] for r in PL.ROUTES)
    for i in range(3):                                    # parked off the diagonal, never near its far end
        px, py, ph = bd.park(i)
        assert PL.dist_to_path((px, py), [(sx, sy), out[-1]]) > 35


def test_route_far_points():
    bd = PL.Board()
    out, back = bd.route_floor(1, (10, 10), 0)
    assert out[-1] == pytest.approx((110, 10)) and bd.route_heading(1) == pytest.approx(45.0) and back[-1] == pytest.approx((10, 10))
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


def test_recovery_and_stuck_decisions():
    assert ST.judge_run(3.0, 2.0)[0] == ""
    assert ST.judge_run(18.0, 22.0)[0] == ""              # a U run drifting ~20 cm by design: no recovery, no flag
    v, why = ST.judge_run(30.0, 45.0)
    assert v == "far_off" and "45 cm" in why
    assert ST.judge_run(2.0, 2.0, seen=False)[0] == "unseen"
    assert ST.judge_run(2.0, 2.0, on_board=False)[0] == "off_board"
    assert ST.judge_run(5.0, 9.0, runend="stopped:obstacle ahead")[0] == "safety_stop"
    assert ST.judge_run(5.0, 9.0, runend="limited:plywood edge")[0] == "safety_stop"
    assert ST.judge_run(5.0, 9.0, runend="watchdog stop")[0] == "safety_stop"
    assert ST.leg_stuck(("go", 10.0), (0, 0), (1.0, 0))
    assert not ST.leg_stuck(("go", 10.0), (0, 0), (9.0, 0))
    assert not ST.leg_stuck(("turn", 90.0), (0, 0), (0, 0))
    lines = ST.nick_instructions("Center", "it is stuck", "near its spot", "the board")
    assert lines[0].startswith("Center needs your help") and any("press Enter" in ln for ln in lines)


def _track(x0, y0, vx, vy, n=10, dt=0.05):
    return [(100.0 + k * dt, x0 + vx * k * dt, y0 + vy * k * dt) for k in range(n)]


def test_camera_watchdog_stops_only_what_heads_for_a_hazard():
    bd = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=121.3, yboard_cm=121.55)   # open edges x = 0, y = 0
    assert ST.hazard(_track(40, 60, -8, 0), bd, {}) is None                           # far from the edge
    assert "open edge" in ST.hazard(_track(13, 60, -8, 0), bd, {})                    # 13 cm off, closing at 8 cm/s
    assert ST.hazard(_track(9, 60, 8, 0), bd, {}) is None                             # inside the zone, leaving it
    assert ST.hazard(_track(9, 60, 0, 0), bd, {}) is None                             # standing still
    assert "past the open edge" in ST.hazard(_track(-2, 60, -5, 0), bd, {})
    assert "backing" not in (ST.hazard(_track(13, 60, -8, 0), bd, {}) or "")
    parked = {"Right": (60.0, 40.0)}
    assert "Right" in ST.hazard(_track(60, 58, 0, -8), bd, parked)                   # heading for a parked robot
    assert ST.hazard(_track(60, 52, 0, 8), bd, parked) is None                        # driving away from it


def test_a_robot_left_near_an_open_edge_is_first_moved_straight_in():
    bd = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=121.3, yboard_cm=121.55)   # open edges x = 0, y = 0
    assert bd.escape_point(60, 60) is None
    assert bd.escape_point(11, 45) == pytest.approx((28.0, 45.0))
    assert bd.escape_point(50, 6) == pytest.approx((50.0, 28.0))


def test_kc_fixes_only_from_pictures_after_the_stop():
    import e02 as E
    moving = [(k * 0.04, 10 + k * 0.4, 20.0) for k in range(50)]
    assert E.kc_fix_from(moving) is None
    stop_t = moving[-1][0]
    still = moving + [(stop_t + 0.04 * (k + 1), moving[-1][1] + 0.05 * (k % 2), 20.0) for k in range(5)]
    assert E.kc_fix_from(still) is None                                               # 0.2 s still: not yet
    still += [(still[-1][0] + 0.04 * (k + 1), moving[-1][1], 20.0) for k in range(15)]
    x, y, t_pic, t_stop, seg = E.kc_fix_from(still)
    assert x == pytest.approx(moving[-1][1], abs=0.1) and t_stop == pytest.approx(stop_t, abs=0.1)
    assert t_stop + 0.1 >= stop_t                          # only pictures taken after it really stopped
    assert t_pic == still[-1][0] and len(seg) >= 10


def test_runner_safety_settings_go_with_every_find_and_route():
    import e02 as E
    assert ("safe_edge", 3.0) in E.SAFETY_SETTINGS and ("edge_bot_cm", 11.0) in E.SAFETY_SETTINGS and ("bumper_cm", 0.0) in E.SAFETY_SETTINGS
    r = E.Runner.__new__(E.Runner)
    r.board = PL.Board(xboard_side=1, yboard_side=1)
    cmds = r.board_params()
    for c in E.SAFETY_CMDS:
        assert c in cmds
    assert E.EXPECTED_FW == "2026-10-25c" and {"2026-10-22b", "2026-10-25a", "2026-10-25b"} <= set(E.OK_FW)   # older still run
    assert set(E.BORDER_FW) == {"2026-10-25a", "2026-10-25b", "2026-10-25c"}   # all have the robot's own border
    assert set(E.BACK_FW) == {"2026-10-25b", "2026-10-25c"}                     # 25c has everything 25b had


def test_switches_back_flag_adds_the_back_settings_per_robot():
    """--switches-back (Nick, 2026-10-25: one attachment, switches at the back): those robots get sw_on, sw_back,
    e02_find_reverse 1 and e02_back_face_cm with the safety settings (sent, then read back); the others don't; a
    robot on older firmware stops the session with what to do."""
    import types
    import e02 as E
    r = E.Runner.__new__(E.Runner)
    r.a = types.SimpleNamespace(switches_back="Left,Right")
    assert r.switches_back("Left") and r.switches_back("Right") and not r.switches_back("Center")
    assert ("e02_find_reverse", 1.0) in r.safety_list("Left") and ("sw_back", 1.0) in r.safety_list("Left")
    assert r.safety_list("Center") == E.SAFETY_SETTINGS
    r.a = types.SimpleNamespace(switches_back="all")
    assert r.switches_back("Center")
    r.a = types.SimpleNamespace(switches_back=None)
    assert not r.switches_back("Left")
    sent, said = [], []
    r.a = types.SimpleNamespace(switches_back="all")
    r.send = lambda n, c: sent.append((n, c))
    r.say = lambda *a, **k: said.append(a[0])
    r.st = {"setup": {}}

    class Lis:
        firmware = {"Left": "2026-10-25b"}

        def got_params(self, n, t0):
            return dict(E.SAFETY_SETTINGS + E.BACK_SETTINGS)
    r.listener = Lis()
    import time as _t
    orig = _t.sleep
    _t.sleep = lambda s: None
    try:
        r.safety_settings(["Left"])
        assert ("Left", "set e02_find_reverse 1") in sent and ("Left", "get sw_back") in sent
        assert r.st["setup"]["switches_back"] == ["Left"]
        r.listener.firmware = {"Left": "2026-10-25a"}
        try:
            r.safety_settings(["Left"])
            raise AssertionError("older firmware must stop the session")
        except SystemExit as e:
            assert "2026-10-25b" in str(e)
    finally:
        _t.sleep = orig


def test_default_floor_calibration_is_roam1s_and_lines_up_with_its_corners():
    import e02 as E
    import e02_camera as CAM
    cam = CAM.load_camera(E.DEFAULT_CALIB)
    corners = {(0.0, 0.0): (387.7, 337.6), (122.15, 0.0): (430.4, 62.0), (122.15, 121.9): (141.1, 6.9),
               (0.0, 121.9): (111.9, 298.9)}
    for (x, y), (u, v) in corners.items():
        pu, pv = cam.project((x, y, 0.0))
        assert math.hypot(pu - u, pv - v) < 4.0, ((x, y), (pu, pv))
    # 2.34 px/cm across the board's middle, as measured on the footage
    a, b = cam.project((61.0, 40.0, 0.0)), cam.project((61.0, 80.0, 0.0))
    assert 2.15 < math.hypot(a[0] - b[0], a[1] - b[1]) / 40.0 < 2.55


def _sim_robot(seed, g_under, bias, bd):
    import config
    import robot
    cfg = config.load_config()
    cfg.dock.contact_boards = True
    cfg.dock.wall_stall_frac = 0.15
    cfg.session.board_cm = [bd.x0, bd.y0, bd.x1, bd.y1]
    cfg.session.open_edges = ["x0", "y0"]
    r = robot.Robot("Left", cfg, random.Random(seed), config.firmware_params(), "test", 0.0, 0x2000 + seed, "ip")
    xb, yb = bd.xboard_cm, bd.yboard_cm
    def ccw(poly):          # the contact model wants the corners counter-clockwise
        a = sum(p[0] * q[1] - q[0] * p[1] for p, q in zip(poly, poly[1:] + poly[:1]))
        return poly if a > 0 else poly[::-1]
    r.world_polys = [ccw([[bd.x0 - 5, xb], [bd.x1 + 5, xb], [bd.x1 + 5, xb + 3], [bd.x0 - 5, xb + 3]]),
                     ccw([[yb, bd.y0 - 5], [yb + 3, bd.y0 - 5], [yb + 3, bd.y1 + 5], [yb, bd.y1 + 5]])]
    r.update_solids()
    r.idle_until(10.0)
    r.fw_yaw_sign = 1
    r.gyro_scale = g_under
    r.gyro_bias = bias
    r.params.update(start_nudge_cm=0, settle_start_ms=200, settle_end_ms=200)
    return r


@pytest.mark.parametrize("park_in", [0.0, 22.0])     # 22: park spots moved in while the camera looks bad (E.PARK_OPEN_MAX_CM)
@pytest.mark.parametrize("g_under,bias", [(-0.085, 0.3), (-0.085, -0.3), (-0.055, 0.3), (-0.055, -0.3), (-0.07, 0.0)])
def test_worst_case_uncalibrated_routes_never_reach_a_parked_robot(g_under, bias, park_in):
    """The simulator's robot (firmware e02.h copy) as in a U run of the session: from a park spot, `run find` on the
    prior corrections, then `run e02` (transit and start turn on those, the route itself UNCALIBRATED: gyro 5.5-8.5%
    short, creep +-0.3 deg/s, factory values from the start mark), Nick's layout. Its true path keeps PARK_CLEAR_CM
    from every other parked robot's centre and WATCH_EDGE_CM + 2 from the open edges, and every route finishes
    (no board, front stop or edge cuts it short)."""
    import e02 as E
    bd = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=121.3, yboard_cm=121.55)
    assert park_in in (0.0, E.PARK_OPEN_MAX_CM)
    bd.park_min_open_cm = park_in
    worst, worst_edge = 99.0, 99.0
    for route in PL.ROUTES:
        i = route % 3
        others = [bd.park(j)[:2] for j in range(3) if j != i]
        r = _sim_robot(31 + route, g_under, bias, bd)
        for c in E.Runner.board_params(type("R", (), {"board": bd})()):
            k, v = c.split()[1:]
            r.params[k] = float(v)
        r.params.update(e02_route=route, e02_mode=0, e02_route_deg=bd.route_heading(route), gyro_scale=1.075,
                        gyro_drift_dps=0.0, wheel_scale=0.979, e02_route_gyro_scale=1.0, e02_route_gyro_drift_dps=0.0,
                        e02_route_wheel_scale=1.0)
        px, py, ph = bd.park(i)
        r.x, r.y, r.th = px, py, math.radians(ph)
        r.pose.set(px, py, ph, 0.5, 1.0)
        r.traj.clear()
        r.run_experiment("find")
        f = [ln for _, ln in r.lines if ln.startswith("D,FIND,")][-1]
        assert ",ok," in f, f
        r.run_experiment("e02")
        rec = [ln for _, ln in r.lines if ln.startswith("D,E02,")][-1]
        assert ",done," in rec, (route, rec)
        for _, x, y, th, _l in r.traj:
            worst_edge = min(worst_edge, min(abs(x - v) if a == "x" else abs(y - v) for a, v in bd.open_edges()))
            for qx, qy in others:
                worst = min(worst, math.hypot(x - qx, y - qy))
    assert worst > PL.PARK_CLEAR_CM, worst
    assert worst_edge > PL.WATCH_EDGE_CM + 2.0, worst_edge


def test_finds_from_every_park_spot_pass_clear_of_the_other_parked_robots():
    bd = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=121.3, yboard_cm=121.55)
    import e02 as E
    for i in range(3):
        r = _sim_robot(41 + i, -0.085, 0.3, bd)
        for c in E.Runner.board_params(type("R", (), {"board": bd})()):
            k, v = c.split()[1:]
            r.params[k] = float(v)
        px, py, ph = bd.park(i)
        r.x, r.y, r.th = px, py, math.radians(ph)
        r.pose.set(px, py, ph, 0.5, 1.0)
        r.traj.clear()
        r.run_experiment("find")
        f = [ln for _, ln in r.lines if ln.startswith("D,FIND,")][-1]
        assert ",ok," in f, f
        for j in range(3):
            if j != i:
                qx, qy, _ = bd.park(j)
                assert min(math.hypot(x - qx, y - qy) for _, x, y, _t, _l in r.traj) > PL.PARK_CLEAR_CM, (i, j)


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


def test_fit_drift_check_can_refuse():
    """The 'not pinned down' check on the drift must be tighter than the drift's prior SD, or it never refuses."""
    assert FIT.MAX_SE["gyro_drift_dps"] < FIT.PRIOR_SD["gyro_drift_dps"]
    obs = observations(1.075, 0.2, 0.978, routes=(2,))
    res = FIT.fit_robot(obs[:3], prior_sd=dict(gyro_drift_dps=2.0))      # 2 turns, no useful prior: not pinned
    assert not res["gyro_ok"] and "not pinned down" in " ".join(res["reasons"])


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
    # 2026-10-24: parts are used as they pass; the gyro, not learned yet, keeps its starting values (and says so)
    assert not res["gyro_ok"] and res["wheel_ok"]
    assert sorted(res["values"]["prior_parts"]) == ["gyro_drift_dps", "gyro_scale"]
    assert any("gyro" in r for r in res["reasons"])


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
