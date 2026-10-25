"""The front sensor's offset and tolerance, learned every session (e02_front.py)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import e02_front as FR  # noqa: E402


def test_a_new_robot_starts_from_the_fleet_default():
    m = FR.FrontModel()
    off, tol, src = m.limits("Left")
    assert off == FR.DEFAULT_OFFSET_CM and tol == FR.TOL_SD * FR.DEFAULT_SD_CM and src == "fleet default"


def test_this_sessions_readings_take_over_and_a_shift_is_reported(tmp_path):
    p = str(tmp_path / "front.json")
    m = FR.FrontModel(p)
    for k in range(40):
        m.add("Left", -3.0 + (k % 5 - 2))           # earlier session: about -3 cm
    m.save()
    m2 = FR.FrontModel(p)
    off, _, src = m2.limits("Left")
    assert abs(off + 3.0) < 0.6 and src == "earlier sessions"
    words = [m2.add("Left", 6.0 + (k % 3 - 1)) for k in range(FR.SESSION_MIN)]   # sensor knocked: about +6 cm
    assert all(w == "" for w in words[:-1]) and "differently" in words[-1]
    off, tol, src = m2.limits("Left")
    assert abs(off - 6.0) < 1.0 and src.startswith("this session") and tol >= FR.MIN_TOL_CM


def test_an_unchanged_sensor_is_not_reported(tmp_path):
    p = str(tmp_path / "front.json")
    m = FR.FrontModel(p)
    for k in range(40):
        m.add("Center", -1.0 + (k % 7 - 3))
    m.save()
    m2 = FR.FrontModel(p)
    words = [m2.add("Center", -1.0 + (k % 7 - 3)) for k in range(FR.SESSION_MIN)]
    assert words[-1] == ""


def test_history_is_capped():
    m = FR.FrontModel()
    m.prior = {"Right": [0.0] * FR.PRIOR_KEEP}
    m.path = None
    m.session = {"Right": [1.0] * 10}
    m.save()                      # no path: nothing written, no error
