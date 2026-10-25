"""Items 3-5 (Nick, 2026-10-23 evening): board lines learned from the finds, confidence pauses, board corrections
when the pictures are late; and the explore checks (check before trust)."""
import math
import os
import tempfile

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02_plan as PL


class St(dict):
    def save(self):
        pass


def _runner():
    import e02 as E
    r = E.Runner.__new__(E.Runner)
    r.board = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=119.4, yboard_cm=123.4)
    r.st = St(positions={}, setup={"board_lines_image": {"x": dict(value=119.4), "y": dict(value=123.4)}})
    r.said, r.sent = [], []
    r.say = lambda m, quiet=False: r.said.append(m)
    r.mark = lambda m: None
    r.send = lambda robot, c: r.sent.append((robot, c))
    r.tracker = None
    r.unc = {}
    r.dir = tempfile.mkdtemp()
    r.order = "test"
    return r


def test_board_lines_move_to_the_touches_after_three_and_are_capped():
    r = _runner()
    r.tracker_area = lambda: None
    r.learn_board_lines("Left", {"x": 115.4})
    r.learn_board_lines("Center", {"x": 115.5})
    assert abs(r.board.xboard_cm - 119.4) < 1e-9            # two touches: not yet
    r.learn_board_lines("Right", {"x": 115.3})
    assert abs(r.board.xboard_cm - 115.4) < 0.2               # moved to where they feel it
    assert r.st["board_line_updates"][-1]["board"] == "x"
    for _ in range(6):
        r.learn_board_lines("Left", {"y": 100.0})
    assert abs(r.board.yboard_cm - (123.4 - 8.0)) < 1e-6      # capped 8 cm from the picture's line


def test_one_odd_robot_does_not_move_the_board():
    r = _runner()
    r.tracker_area = lambda: None
    for _ in range(3):
        r.learn_board_lines("Left", {"x": 119.3})
        r.learn_board_lines("Center", {"x": 119.5})
        r.learn_board_lines("Right", {"x": 112.0})         # Right's own oddity
    assert abs(r.board.xboard_cm - 119.4) < 0.5
    assert r.st["robot_board_offsets"]["Right"]["x"] < -5.0


def test_confidence_pause_takes_a_fix_past_the_threshold():
    import e02 as E
    r = _runner()
    r.pictures_late = lambda: False
    r.own_pose = lambda robot: (50.0, 50.0, 0.0)
    fixes = []

    def fix_now(robot, heading=None):
        fixes.append(robot)
        r.unc_reset(robot)
        return (51.0, 50.0)
    r.fix_now = fix_now
    r.unc["Left"] = dict(dist=150.0, turned=180.0)
    assert r.unc_cm("Left") > E.CONF_PAUSE_CM - 1.5
    r.confidence_pause("Left", "go", 60.0)
    assert fixes == ["Left"] and r.unc["Left"]["dist"] == 0.0
    row = r.st["confidence_pauses"][-1]
    assert row[5].startswith("paused for") and row[7] == 1.0
    r.confidence_pause("Left", "go", 10.0)                     # fresh: no pause
    assert fixes == ["Left"]


def test_late_pictures_mean_no_pause_and_a_board_correction():
    import e02 as E
    r = _runner()
    r.pictures_late = lambda: True
    r.own_pose = lambda robot: (60.0, 95.0, 90.0)              # facing +y, the x-board at 119.4 ahead
    r.fix_now = lambda robot, heading=None: (_ for _ in ()).throw(AssertionError("no camera fix when late"))
    r.unc["Left"] = dict(dist=400.0, turned=0.0)
    r.confidence_pause("Left", "go", 30.0)
    assert r.st["confidence_pauses"][-1][5].startswith("pictures late")
    r.front_limits = lambda robot=None: (0.0, 3.0)
    front = 119.4 - 100.0 - PL.ROBOT_HALF_CM                   # really at y = 100: 5 cm further than it believes
    q = r.board_correct("Left", front, (front, "the x-board", 1.0), "done")
    assert q is not None and abs(q[1] - 100.0) < 1e-6 and abs(q[0] - 60.0) < 1e-6
    assert r.sent and r.sent[-1][1].startswith("fix 60.00 100.00")
    assert E.MAP is not None


def test_compass_failure_asks_and_can_skip():
    r = _runner()
    r.unavailable = set()
    calls = []

    def _cal(robots):
        calls.append(list(robots))
        for n in robots:
            r.st.setdefault("calibrations", []).append([0, n, "x", "failed:yaw check failed" if n == "Center" else "done"])
    r._calibrate = _cal

    class A:
        auto = False
    r.a = A()
    answers = iter(["", "skip"])
    r.ask = lambda prompt, default="": next(answers)
    r.calibrate(["Left", "Center", "Right"])
    assert calls == [["Left", "Center", "Right"], ["Center"]]
    assert r.unavailable == {"Center"}
    assert any("FAILED" in m for m in r.said)


def test_calibration_note_says_when_an_old_calibration_is_used():
    r = _runner()
    r.st["setup"]["calibration_source"] = dict(kind="explore", session=True)
    assert r.calib_note() == ""
    r.st["setup"]["calibration_source"] = dict(kind="fallback", file="/x/overhead_floorcal_roam1.json", session=False)
    assert "FALLBACK" in r.calib_note() and "roam1" in r.calib_note()
    r.st["setup"]["calibration_source"] = dict(kind="file", file="/x/a.json", session=False)
    r.st["calibration"] = "/x/a.json"
    assert "not made by the robots" in r.calib_note()


def test_robot_check_rejects_nonsense():
    import e02_explore as X
    import numpy as np
    rng = np.random.default_rng(0)
    stops = [dict(x=0.0, y=0.0, th=0.0, d=0.0, dth=0.0, u=300.0, v=200.0)]
    for k in range(20):
        stops.append(dict(x=10.0, y=0.0, th=0.0, d=10.0 if k % 2 == 0 else 0.0, dth=0.0 if k % 2 == 0 else 90.0,
                          u=float(rng.uniform(0, 640)), v=float(rng.uniform(0, 480))))
    c = X.robot_check(stops)
    assert not c["ok"] and c["why"]
    assert math.isfinite(len(c["why"]))
    assert os.path.exists(os.path.dirname(X.__file__))


def test_no_pictures_waits_for_the_camera_instead_of_crashing():
    """2026-10-23 evening: the laptop slept, the Pi's recorder stopped, fresh_floor crashed on no pictures."""
    import pytest
    r = _runner()
    r.camera_alive = lambda: True

    class F:
        n = 0

        def wait_frames(self, k, timeout=0):
            F.n += 1
            return ["img"] if F.n >= 3 else []
    r.feed = F()
    assert r.wait_for_camera() == ["img"]
    r.CAMERA_WAIT_S = 0.0
    F.n = -100
    with pytest.raises(SystemExit):
        r.wait_for_camera()
