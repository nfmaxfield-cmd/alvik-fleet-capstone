"""Two-camera tracking (tools/experiment/e02_twocam.py, 2026-10-27) on simulated pictures: the overhead washed out as
on the real board at 11:20 (tools/sim/table_cam.washed_out), the side camera (table_cam.SideCamera), the lights.

    python3 -m pytest tools/experiment/tests/test_e02_twocam.py -q        # about 40 s
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
def cams(tmp_path_factory):
    d = tmp_path_factory.mktemp("cams")
    tc = TC.TableCamera(RECT, seed=18, px_per_cm=2.34)
    oc = CAM.load_camera(tc.true_model(str(d / "o.json")))
    sc = TC.SideCamera(RECT)
    scm = CAM.load_camera(sc.true_model(str(d / "s.json")))
    return tc, TC.washed_out(tc), oc, sc, scm


def run(cams, robots, wash=True, side=True, lights=False, lag=0.0, lag_told=None, frames=120, drive=None,
        side_offset=(0.0, 0.0), side_bias=(0.0, 0.0)):
    """robots {name: [x, y, th rad]}; drive: (name, from frame, to frame, cm a frame). Returns (tracker, log of
    (frame, name, status, src, error cm))."""
    tc, tw, oc, sc, scm = cams
    over = tw if wash else tc
    trk = TW.TwoCamTracker(oc, board_center=(61, 61), board_rect=RECT, side_cam=scm if side else None,
                           side_lag_s=lag if lag_told is None else lag_told, lights=lights, side_bias=side_bias)
    L = (1, 1, 1) if lights else (0, 0, 0)

    def poses(rb):
        return [(x, y, th, L, L) for x, y, th in rb.values()]

    def side_poses(rb):
        return [(x + side_offset[0], y + side_offset[1], th, L, L) for x, y, th in rb.values()]
    spots = [(x, y) for x, y, _ in robots.values()]
    trk.set_background([over.render(poses(robots)) for _ in range(5)], robots=spots)
    trk.set_side_background([sc.render(side_poses(robots)) for _ in range(5)], robots=spots)
    for n, (x, y, th) in robots.items():
        trk.add(n, x, y, heading=math.degrees(th))
    t, log, past = 1000.0, [], []
    for k in range(frames):
        t += 0.04
        if drive and drive[1] <= k < drive[2]:
            n = drive[0]
            x, y, th = robots[n]
            if k == drive[1]:
                trk.command(n, "go", drive[3] * (drive[2] - drive[1]), t, 0.04 * (drive[2] - drive[1]),
                            heading=math.degrees(th), ramp=0.01)
            robots[n] = [x + drive[3] * math.cos(th), y + drive[3] * math.sin(th), th]
        past.append((t, {n: list(v) for n, v in robots.items()}))
        trk.push(over.render(poses(robots)), t)
        if k % 2 == 0:
            # the side picture shows the moment `lag` ago, stamped now
            then = min(past, key=lambda q: abs(q[0] - (t - lag)))[1]
            trk.push_side(sc.render(side_poses(then)), t)
        for n in robots:
            st = trk.state(n)
            x, y, _ = robots[n]
            log.append((k, n, st["status"], st["src"], math.hypot(st["x"] - x, st["y"] - y)))
    return trk, log


def three():
    return {"Left": [30.0, 40.0, 0.3], "Center": [75.0, 90.0, 2.0], "Right": [95.0, 30.0, -1.2]}


def test_washed_out_overhead_alone_does_not_see_the_bodies(cams):
    """The 11:20 failure, reproduced: the board as bright as the robots' tops."""
    _, log = run(cams, three(), side=False, frames=40)
    late = [r for r in log if r[0] >= 20]
    assert sum(r[2] == "seen" for r in late) <= 0.1 * len(late)      # (a stray piece of a body now and then)


def test_side_camera_sees_them_when_the_overhead_cannot(cams):
    trk, log = run(cams, three(), drive=("Left", 30, 110, 0.4))      # Left drives 32 cm at 10 cm/s
    late = [r for r in log if r[0] >= 10]
    assert all(r[2] == "seen" and r[3] == "side" for r in late), {(r[2], r[3]) for r in late}
    errs = [r[4] for r in late]
    assert np.median(errs) < 1.5 and np.percentile(errs, 95) < 3.0
    assert trk.history("Left", 0, body_only=True)               # a body sighting: L0's check passes


def test_both_cameras_on_a_plain_board(cams):
    _, log = run(cams, three(), wash=False, frames=40)
    late = [r for r in log if r[0] >= 10]
    assert all(r[3] == "both" for r in late)
    assert np.median([r[4] for r in late]) < 1.0


def test_lights_stand_in_when_no_camera_sees_the_body(cams):
    trk, log = run(cams, three(), side=False, lights=True, frames=40)
    late = [r for r in log if r[0] >= 10]
    assert all(r[2] == "seen" and r[3] == "lights" for r in late)
    assert np.median([r[4] for r in late]) < 1.5
    assert trk.history("Left", 0, body_only=True) == []          # lights are not a body: L0 still keeps it out
    assert trk.history("Left", 0)


def test_side_lag_is_taken_out(cams):
    """The side pictures 0.12 s late; Left drives at 10 cm/s. Told the lag, its spots line up; not told, they trail."""
    def err(told):
        _, log = run(cams, {"Left": [30.0, 40.0, 0.3]}, lag=0.12, lag_told=told, drive=("Left", 20, 110, 0.4))
        mv = [r[4] for r in log if 40 <= r[0] < 110 and r[2] == "seen"]
        return float(np.median(mv)) if mv else 99.0
    assert err(0.12) < 1.0
    assert err(0.0) > err(0.12) + 0.5


def test_robots_in_line_from_the_side_camera_are_not_judged_by_it(cams):
    """Two robots one behind the other from where the side camera stands: it says 'in line with', not a position."""
    sc = cams[3]
    C = sc.C
    a = (61.0, 40.0)
    ux, uy = a[0] - C[0], a[1] - C[1]
    L = math.hypot(ux, uy)
    b = (a[0] + 11.0 * ux / L, a[1] + 11.0 * uy / L)                # 11 cm further along the same line of sight
    trk, log = run(cams, {"Near": [a[0], a[1], 0.0], "Far": [b[0], b[1], 0.0]}, frames=20)
    obs = trk.tracks["Near"].obs["side"]
    assert obs[3] == "unsure" and "in line with" in obs[4]


def test_cameras_disagreeing_is_unsure_not_a_guess(cams):
    """The side camera's offset set 7 cm wrong across its line of sight: both cameras see the robot, 7 cm apart (inside the side's gate of
    8 cm, outside AGREE_CM): flagged unsure, no position."""
    _, log = run(cams, {"Left": [40.0, 60.0, 0.0]}, wash=False, side_bias=(7.0, 0.0), frames=30)
    late = [r for r in log if r[0] >= 10]
    assert all(r[2] == "unsure" for r in late), {(r[2], r[3]) for r in late}


def test_stretch_along_the_line_of_sight(cams):
    scm = cams[4]
    s_near, _ = TW.stretch_at(scm, 45.0, 5.0, TW.SIDE_Z_CM)
    s_far, _ = TW.stretch_at(scm, 45.0, 115.0, TW.SIDE_Z_CM)
    assert 1.0 < s_near < s_far < 2.0
    _, (ux, uy) = TW.stretch_at(scm, 45.0, 115.0, TW.SIDE_Z_CM)
    d_along = TW.sight_dist(scm, 45.0, 115.0, TW.SIDE_Z_CM, 3 * ux, 3 * uy)
    d_across = TW.sight_dist(scm, 45.0, 115.0, TW.SIDE_Z_CM, -3 * uy, 3 * ux)
    assert d_along == pytest.approx(3 / s_far, rel=1e-6) and d_across == pytest.approx(3.0, rel=1e-6)


def test_filled_in_floor_margin_is_measured(cams):
    trk, _ = run(cams, three(), side=False, frames=2)
    assert TW.UNKNOWN_EXTRA_RANGE[0] <= trk.UNKNOWN_EXTRA < 14.0          # a plain board fills in well
