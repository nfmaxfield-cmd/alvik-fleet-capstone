"""The camera's error map, the KC blend weight, the drift bound and the runner-stop label (2026-10-24 review)."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np  # noqa: E402

import e02_camerr as CE  # noqa: E402
import e02_fit as FIT  # noqa: E402
import e02_plan as PL  # noqa: E402
import e02_steer as ST  # noqa: E402


def test_no_samples_no_correction():
    m = CE.CameraErrorMap()
    dx, dy, sd = m.correction(50, 50)
    assert (dx, dy) == (0.0, 0.0) and sd == CE.NO_DATA_SD_CM


def test_marks_correct_nearby_and_not_far_away():
    m = CE.CameraErrorMap()
    for x, y in ((40, 40), (60, 60), (40, 60), (60, 40)):
        m.add(x, y, x + 2.0, y - 1.0, "mark")           # the camera reads 2 cm too far in x, 1 cm too low in y
    dx, dy, sd = m.correction(50, 50)
    assert abs(dx - 2.0) < 1e-6 and abs(dy + 1.0) < 1e-6
    assert sd < CE.NO_DATA_SD_CM
    x, y, _ = m.corrected(52.0, 49.0)
    assert abs(x - 50.0) < 1e-6 and abs(y - 50.0) < 1e-6
    # far from every mark: unchecked
    assert m.correction(200, 200)[2] == CE.NO_DATA_SD_CM


def test_marks_outweigh_finds():
    m = CE.CameraErrorMap()
    m.add(50, 50, 52, 50, "mark")
    m.add(50, 50, 50, 50, "find")
    dx = m.correction(50, 50)[0]
    assert abs(dx - 2.0 * CE.MARK_W / (CE.MARK_W + CE.FIND_W)) < 1e-6


def test_mark_xy_from_board_faces():
    b = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=119.2, yboard_cm=123.8)
    x, y = CE.mark_xy(b, 30.0, 30.0)
    assert abs(x - 93.8) < 1e-6 and abs(y - 89.2) < 1e-6
    b0 = PL.Board(xboard_side=0, yboard_side=0, xboard_cm=2.0, yboard_cm=1.0)
    x, y = CE.mark_xy(b0, 30.0, 10.0)
    assert abs(x - 31.0) < 1e-6 and abs(y - 12.0) < 1e-6


def test_blend_weight():
    assert CE.blend_weight(1.0, 1.0) == 0.5
    assert CE.blend_weight(0.5, 3.0) < 0.05               # just found itself: the camera barely moves it
    assert CE.blend_weight(10.0, 1.0) > 0.95              # lost: the camera nearly replaces it


def _turn_obs(g_true=1.075, d_true=0.0, d_fake=0.0, seed=0):
    """Stretches either side of turns (fit_turns' model): phi (camera) = g * psi (gyro raw) - d * t, with a fake
    time-varying term the fit could mistake for drift."""
    rng = np.random.default_rng(seed)
    obs = []
    for run in range(4):
        tm, psi = 0.0, 0.0
        for k in range(6):
            obs.append(dict(run=run, tm=tm, psi=psi, phi=g_true * psi - (d_true + d_fake) * tm + rng.normal(0, 0.5)))
            psi += 90.0 if k % 2 == 0 else 0.0
            tm += 6.0
    return obs


def test_drift_is_held_near_the_standing_still_value():
    obs = _turn_obs(d_fake=-0.2)
    free = FIT.fit_robot(obs, None, prior_sd=dict(gyro_drift_dps=0.1))
    held = FIT.fit_robot(obs, None, prior_sd=dict(gyro_drift_dps=0.1), measured_drift=0.0)
    assert abs(free["fitted"]["gyro_drift_dps"]) > FIT.DRIFT_BAND_DPS
    assert abs(held["fitted"]["gyro_drift_dps"]) <= FIT.DRIFT_BAND_DPS + 1e-9
    assert held["fitted"]["drift_held"] is not None
    assert any("held" in r for r in held["reasons"])


def test_drift_within_band_is_untouched():
    obs = _turn_obs(d_true=0.02)
    res = FIT.fit_robot(obs, None, prior_sd=dict(gyro_drift_dps=0.1), measured_drift=0.0)
    assert res["fitted"]["drift_held"] is None


def test_runner_stop_is_not_a_safety_stop():
    v, why = ST.judge_run(30.0, 30.0, runend="runner stop (finding it again)")
    assert v == "runner_stop"
    v, _ = ST.judge_run(30.0, 30.0, runend="stopped:stop command")
    assert v == "safety_stop"


# ----------------------------------------------------------------------------- the camera's creep and quality map
import cv2  # noqa: E402

import e02_camera as CAM  # noqa: E402


class _FlatCam:
    """Floor cm -> pixels: 4 px a cm, board at (40, 40) px."""
    def project(self, p):
        return np.array([40.0 + 4.0 * p[0], 40.0 + 4.0 * p[1]])

    def to_floor(self, u, v, z=0.0):
        return ((u - 40.0) / 4.0, (v - 40.0) / 4.0)


def _scene(shift=(0.0, 0.0), seed=3):
    rng = np.random.default_rng(seed)
    img = cv2.GaussianBlur((rng.random((480, 640)) * 255).astype(np.uint8), (0, 0), 3)
    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
    M = np.float32([[1, 0, shift[0]], [0, 1, shift[1]]])
    return cv2.warpAffine(img, M, (640, 480), borderMode=cv2.BORDER_REFLECT)


def test_shift_watch_measures_and_hands_the_shift_to_the_tracker():
    class T:
        px_shift = (0.0, 0.0)
    got = []
    w = CAM.CameraShiftWatch(_FlatCam(), (0, 0, 100, 80), T, on_shift=got.append)
    w(_scene(), 0.0)                         # the reference
    w(_scene((0.0, -2.0)), 10.0)
    assert got and abs(got[-1]["dv"] + 2.0) < 0.3 and abs(got[-1]["du"]) < 0.3
    assert got[-1]["warn"]
    assert abs(T.px_shift[1] + 2.0) < 0.3


def test_quality_map_marks_poor_cells():
    q = CAM.QualityMap()
    flat = np.full((480, 640), 190, np.uint8)               # a bright floor and a flat robot: no detail
    tex = (np.indices((480, 640)).sum(0) % 2 * 120 + 60).astype(np.uint8)   # a checkerboard: lots of detail
    for k in range(20):
        q.add(flat, 200, 200, 85.0, 105.0, "detector", k * 2.0)
        q.add(tex, 200, 200, 5.0, 65.0, "both", k * 2.0 + 1.0)
    assert q.poor(85.0, 105.0)
    assert not q.poor(5.0, 65.0)
    assert not q.poor(50.0, 50.0)            # never seen: not called poor
    rows = q.rows()
    assert {r["poor"] for r in rows} == {0, 1}


def test_kc_belief_walks_forward_along_the_route():
    import e02 as E
    wps = [[95, 90], [45, 90], [45, 40], [95, 40], [95, 90]]          # a square: start, ..., back at the start
    assert E.kc_belief(wps, (96, 91), 0)[0] == 0                     # at the start
    i, xy, dist, turned = E.kc_belief(wps, (46, 41), 0)
    assert i == 2 and xy == (45.0, 40.0) and abs(dist - 100.0) < 1e-6 and abs(turned - 180.0) < 1e-6
    assert E.kc_belief(wps, (94, 91), 3)[0] == 4                     # back at the start: the last waypoint, not the first
    assert E.kc_belief(wps, (70, 65), 0) is None                     # between waypoints: not sure


def test_leg_delta_with_a_board_pose_kept_from_the_session_before():
    """2026-10-24 Pi plan, stage 3: Left's records for `go 12` (pose known from stage 2's camera fixes). The leg
    moved 12 cm, not 112 (the board pose's y), and the turn records give the turn, not the board heading."""
    import e02 as E
    st = dict(x_cm="18.60", y_cm="100.40", th_deg="116.48")
    fin = dict(x_cm="13.29", y_cm="110.99", th_deg="116.56")
    x, y, th = E.leg_delta(st, fin)
    assert abs(x - 11.85) < 0.1 and abs(y) < 0.5 and abs(th) < 0.2
    x, y, th = E.leg_delta(dict(x_cm="63.31", y_cm="11.54", th_deg="116.87"),
                           dict(x_cm="63.48", y_cm="11.24", th_deg="-153.05"))       # turn 90
    assert abs(x) < 0.5 and abs(th - 90.08) < 0.1
    # a robot with no pose yet starts each idle leg at 0: the final record is the move
    assert E.leg_delta(dict(x_cm="0", y_cm="0", th_deg="0"), dict(x_cm="12", y_cm="0.1", th_deg="0.2"))[0] == 12.0
    assert E.leg_delta(None, dict(x_cm="12", y_cm="0", th_deg="0"))[0] == 12.0
