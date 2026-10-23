"""Explore (2026-10-23): the robots calibrate the camera. A synthetic session: three robots with wrong wheel and gyro
scales dance a square and drive rays to the board's sides; the camera's pixels come from a real calibration with
0.7 px noise. The fit must recover the camera (floor error against the truth), each robot's scales, and the board's
rectangle; a stretch where the tracker stayed on an old spot (the 2026-10-23 dry run) must be thrown out."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
TOOLS = os.path.dirname(EXP)
for p in (EXP, TOOLS, os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import e02_camera as CAM  # noqa: E402
import e02_explore as X  # noqa: E402

CALIB = os.path.join(EXP, "data", "overhead_floorcal_roam1.json")
W, H = 122.15, 121.9
TRUTH = {"Left": ((20, 106, 0), 0.99, 1.07), "Center": ((110, 16, 90), 1.0, 1.08), "Right": ((18, 74, 0), 1.02, 1.07)}


def hit(x, y, th):
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    best = None
    for k, (a, v) in {"x0": ("x", 0), "x1": ("x", W), "y0": ("y", 0), "y1": ("y", H)}.items():
        den = c if a == "x" else s
        if abs(den) < 1e-9:
            continue
        d = (v - (x if a == "x" else y)) / den
        if d > 0 and (best is None or d < best[0]):
            best = (d, k)
    return best


def session(cam, seed=0, stuck=None):
    """stuck = (robot, from_stop): from that stop on, the camera reports the robot's spot at that stop."""
    rng = np.random.default_rng(seed)
    robots = {}
    for n, ((x, y, th), s_t, g_t) in TRUTH.items():
        stops = []
        st = dict(tx=x, ty=y, tth=th, ox=0.0, oy=0.0, oth=0.0)

        def stop():
            u, v = cam.project((st["tx"], st["ty"], X.ROBOT_TOP_CM))
            stops.append(dict(x=st["ox"], y=st["oy"], th=st["oth"], u=u + rng.normal(0, 0.7), v=v + rng.normal(0, 0.7)))
            if stuck and stuck[0] == n and len(stops) > stuck[1]:
                stops[-1]["u"], stops[-1]["v"] = stops[stuck[1]]["u"], stops[stuck[1]]["v"]

        stop()
        seq = [("go", 15), ("turn", 90)] * 4 + [("turn", 72), ("ray", None)] * 5
        for kind, val in seq:
            if kind == "turn":
                st["tth"] += val
                st["oth"] += val / g_t * (1 + rng.normal(0, 0.003))
                stop()
                continue
            if kind == "ray":
                d, side = hit(st["tx"], st["ty"], st["tth"])
                kd = "edge" if side in ("x0", "y0") else "board"
                dist = d - (3.5 if kd == "edge" else 11.5)
                if dist <= 2:
                    continue
            else:
                dist = val
            c, s_ = math.cos(math.radians(st["tth"])), math.sin(math.radians(st["tth"]))
            od = dist / s_t * (1 + rng.normal(0, 0.004))
            for sign in ((1,) if kind == "go" else (1, -1)):
                st["tx"] += sign * dist * c
                st["ty"] += sign * dist * s_
                st["ox"] += sign * od * math.cos(math.radians(st["oth"]))
                st["oy"] += sign * od * math.sin(math.radians(st["oth"]))
                stop()
                if kind == "ray" and sign == 1:
                    stops[-1]["contact"] = kd
        robots[n] = stops
    return robots


def floor_error(cam, sol):
    pts = []
    for n, st in sol["robots"].items():
        path = X.to_world(X.rebuild(st, sol["s"][n], sol["g"][n]), sol["frames"][n])
        for (x, y, th), q in zip(path, st):
            if "contact" in q:
                a = 3.5 if q["contact"] == "edge" else 11.5
                pts.append((x + a * math.cos(math.radians(th)), y + a * math.sin(math.radians(th)), q["contact"]))
    rect = X.rectangle(X.sides(pts))
    assert rect is not None
    bf = X.board_frame(rect)
    cb = X.camera_in_board(sol, bf)
    import cameras
    m = cameras.CameraModel(cb["K"], cb["dist"], cb["rvec"], cb["tvec"], cb["image_size"])
    errs = []
    for gx in np.linspace(5, W - 5, 6):
        for gy in np.linspace(5, H - 5, 6):
            u, v = cam.project((gx, gy, X.ROBOT_TOP_CM))
            fx, fy = m.to_floor(u, v, X.ROBOT_TOP_CM)
            errs.append(math.hypot(fx - gx, fy - gy))
    return bf, float(np.median(errs)), float(max(errs))


@pytest.fixture(scope="module")
def cam():
    if not os.path.exists(CALIB):
        pytest.skip("no calibration file")
    return CAM.load_camera(CALIB)


@pytest.mark.parametrize("stuck", [None, ("Left", 12)])
def test_explore_recovers_camera_scales_and_board(cam, stuck):
    robots = session(cam, seed=1, stuck=stuck)
    sol = X.fit(robots, (640, 480))
    sol["robots"] = robots
    assert sol["rms_px"] < 3.0
    for n, (_, s_t, g_t) in TRUTH.items():
        assert abs(sol["s"][n] - s_t) < 0.03, n
        assert abs(sol["g"][n] - g_t) < 0.02, n
    if stuck:
        assert sol.get("outliers", 0) >= 3
    bf, med, mx = floor_error(cam, sol)
    assert abs(bf["W"] - W) < 4 and abs(bf["H"] - H) < 4
    assert med < 2.0 and mx < 5.0
