"""The multi-robot tracker (tools/experiment/e02_multitrack.py): every robot followed at once from the simulated
overhead camera, identity kept by prediction from the commanded legs, unsure robots flagged, not guessed."""
import math
import os
import tempfile

import numpy as np
import pytest

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02_multitrack as MT
from test_e02_runner import FakeRobot, FakeWorld, sim_camera


@pytest.fixture(scope="module")
def mcam():
    import e02_camera as CAM
    w = FakeWorld()
    sc = sim_camera(w)
    d = tempfile.mkdtemp()
    cam = CAM.load_camera(sc.true_camera_model(os.path.join(d, "true.json")))
    return w, sc, cam


def start(w, sc, cam, robots):
    """A tracker whose floor memory was taken with the board empty, then the robots put down and added."""
    w.robots[:] = []
    tr = MT.MultiTracker(cam, board_rect=(0.0, 0.0, 122.15, 121.9))
    tr.set_background([sc.render() for _ in range(3)])
    w.robots[:] = list(robots.values())
    for n, r in robots.items():
        tr.add(n, r.x, r.y, heading=r.th, t=0.0)
    return tr


def drive(w, sc, tr, robots, legs, seconds, fps=15.0, t0=0.0):
    """legs: {name: (vx, vy)} cm/s for every robot that drives (straight, constant speed). Each leg is told to the
    tracker as a `go` command. Returns the per-frame errors {name: [cm]} and statuses {name: [status]}."""
    for n, (vx, vy) in legs.items():
        sp = math.hypot(vx, vy)
        tr.command(n, "go", sp * seconds, t0, seconds, heading=math.degrees(math.atan2(vy, vx)), ramp=0.01)
    errs = {n: [] for n in robots}
    stat = {n: [] for n in robots}
    t = t0
    for _ in range(int(seconds * fps)):
        t += 1.0 / fps
        for n, (vx, vy) in legs.items():
            robots[n].x += vx / fps
            robots[n].y += vy / fps
        tr.push(sc.render(), t)
        for n, r in robots.items():
            s = tr.state(n)
            errs[n].append(math.hypot(s["x"] - r.x, s["y"] - r.y))
            stat[n].append(s["status"])
    return errs, stat, t


def test_three_robots_moving_at_once_are_all_followed(mcam):
    w, sc, cam = mcam
    robots = dict(A=FakeRobot(30.0, 30.0, 0.0), B=FakeRobot(90.0, 40.0, 90.0), C=FakeRobot(60.0, 95.0, 180.0))
    tr = start(w, sc, cam, robots)
    errs, stat, _ = drive(w, sc, tr, robots, dict(A=(9.0, 0.0), B=(0.0, 9.0), C=(-9.0, 0.0)), 4.0)
    for n in robots:
        assert float(np.median(errs[n])) < 1.0 and max(errs[n]) < 2.5, (n, max(errs[n]))
        assert stat[n].count("seen") >= 0.9 * len(stat[n]), (n, stat[n])


def test_two_robots_passing_close_keep_who_is_who(mcam):
    """Head-on pass, centres 17 cm apart sideways (bodies ~7.5 cm apart): each keeps its own name the whole way."""
    w, sc, cam = mcam
    robots = dict(A=FakeRobot(25.0, 52.0, 0.0), B=FakeRobot(95.0, 69.0, 180.0))
    tr = start(w, sc, cam, robots)
    errs, stat, _ = drive(w, sc, tr, robots, dict(A=(10.0, 0.0), B=(-10.0, 0.0)), 7.0)
    for n in robots:
        assert max(errs[n]) < 3.0, (n, max(errs[n]))          # a swap would be ~17 cm
    assert robots["A"].x > robots["B"].x                        # they really did pass each other


def test_a_crossing_just_behind_keeps_identity_by_prediction(mcam):
    """B crosses A's path just behind A (perpendicular, 2 s later): identity stays right."""
    w, sc, cam = mcam
    robots = dict(A=FakeRobot(30.0, 60.0, 0.0), B=FakeRobot(60.0, 25.0, 90.0))
    tr = start(w, sc, cam, robots)
    # A goes first along x; B starts 1 s later along y, passing (60, 60) about 2 s after A did
    tr.command("A", "go", 60.0, 0.0, 6.0, heading=0.0, ramp=0.01)
    tr.command("B", "go", 60.0, 1.0, 6.0, heading=90.0, ramp=0.01)
    t, worst = 0.0, {"A": 0.0, "B": 0.0}
    for k in range(int(7.0 * 15)):
        t += 1 / 15.0
        if t <= 6.0:
            robots["A"].x += 10.0 / 15
        if 1.0 < t <= 7.0:
            robots["B"].y += 10.0 / 15
        tr.push(sc.render(), t)
        for n, r in robots.items():
            s = tr.state(n)
            worst[n] = max(worst[n], math.hypot(s["x"] - r.x, s["y"] - r.y))
    assert worst["A"] < 3.0 and worst["B"] < 3.0, worst


def test_touching_robots_are_flagged_unsure_not_guessed(mcam):
    """Two robots pushed together (one outline in the picture): neither is given the other's position; both are
    flagged unsure (merged) and keep their predictions."""
    w, sc, cam = mcam
    robots = dict(A=FakeRobot(50.0, 60.0, 0.0), B=FakeRobot(70.0, 60.0, 180.0))
    tr = start(w, sc, cam, robots)
    errs, stat, t = drive(w, sc, tr, robots, dict(A=(2.5, 0.0), B=(-2.5, 0.0)), 2.0)   # centres 20 -> 10 cm
    sA, sB = tr.state("A"), tr.state("B")
    assert sA["status"] in ("unsure", "lost", "predicted") and sB["status"] in ("unsure", "lost", "predicted")
    assert "unsure" in stat["A"] and "unsure" in stat["B"]
    # its position is its own prediction, never the other robot's
    assert abs(sA["x"] - robots["A"].x) < 3.0 and abs(sB["x"] - robots["B"].x) < 3.0


def test_a_robot_taken_off_the_board_is_lost(mcam):
    w, sc, cam = mcam
    robots = dict(A=FakeRobot(40.0, 40.0, 0.0), B=FakeRobot(80.0, 80.0, 0.0))
    tr = start(w, sc, cam, robots)
    for k in range(10):
        tr.push(sc.render(), k / 15.0)
    w.robots[:] = [robots["A"]]                  # B lifted away
    t = 10 / 15.0
    for k in range(30):
        t += 1 / 15.0
        tr.push(sc.render(), t)
    assert tr.state("A")["status"] == "seen"
    assert tr.state("B")["status"] == "lost"


def test_prediction_uses_the_robots_own_report_once_done():
    tr = MT.Track("A", 10.0, 10.0, 0.0, heading=0.0)
    m = MT.Motion("go", 20.0, 1.0, 2.0, 0.0, ramp=0.3)
    assert m.disp(0.5) == (0.0, 0.0)
    assert m.disp(2.0)[0] == pytest.approx(10.0, abs=0.5)
    m.t_done, m.reported = 3.0, (18.0, 1.0)      # its wheels said 18 cm, a little to the left
    assert m.disp(4.0) == (18.0, 1.0)
    assert MT.trapezoid_fraction(1.0, 2.0, 0.3) == pytest.approx(0.5)
    assert tr.status == "seen"
