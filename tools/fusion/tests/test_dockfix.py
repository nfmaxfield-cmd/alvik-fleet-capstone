"""Home dock and touch records as fusion measurements (dockfix.py).

    python -m pytest tools/fusion/tests/test_dockfix.py -q
"""
import math

import pytest

import dockfix
import engine as eng
import landmarks as lm
from ekf import D2R
from fusion_synth import straight_run

WWG = ["wheel_v", "wheel_w", "gyro"]


def run_with(records, kind, scale=1.0, drive_s=3.0, v=5.0):
    """A robot driving straight from (0, 0) heading +x (start pose taken from the tape) whose wheels read
    `scale` x the true distance, with one DOCK / TOUCH record at the end."""
    run, truth = straight_run("B-1", "Left", 0.0, 0.0, 0.0, v=v, drive_s=drive_s)
    for r in run.raw:
        for k in ("v_cms", "wl_rpm", "wr_rpm"):
            r[k] *= scale
    run.params = {"bump_ahead_cm": 6.09, "bump_half_cm": 3.91, "sw_gap_cm": 0.02}
    t_end = truth[-1][0]
    for r in records:
        r.setdefault("t", t_end - 0.5)
    setattr(run, kind, records)
    return run, truth


def final(tr):
    return tr.ekf.s[0], tr.ekf.s[1], tr.ekf.s[2]


def test_no_records_changes_nothing():
    run, _ = run_with([], "dock")
    a, _ = eng.run_engine([run], [], eng.config(sensors=WWG, start_from="tape", dock_poses=[[30.0, 0.0, 0.0]]))
    b, _ = eng.run_engine([run], [], eng.config(sensors=WWG, start_from="tape", dock_fix=False))
    assert final(a["B-1"]) == pytest.approx(final(b["B-1"]))


def test_dock_record_is_a_full_pose_fix():
    # the wheels read 3% long, so by the end the filter thinks it is ~0.45 cm further than it is
    true_end = 5.0 * 3.0                       # 15 cm
    wall = [true_end + 6.09, 0.4, 1.5]          # a dock whose docked pose is where the robot really is (+ a tilt)
    run, truth = run_with([dict(result="docked", stripe_c_cm="-1", travel_cm="5")], "dock", scale=1.03)
    cfg = eng.config(sensors=WWG, start_from="tape", dock_poses=[wall])
    tr, _ = eng.run_engine([run], [], cfg)
    x, y, th = final(tr["B-1"])
    ex, ey, eh = dockfix.dock_pose(wall, 6.09)
    assert x == pytest.approx(ex, abs=0.06)
    assert y == pytest.approx(ey, abs=0.06)
    assert th == pytest.approx(eh, abs=0.4 * D2R)
    assert len(tr["B-1"].dock_fixes) == 1


def test_dock_far_away_is_not_used():
    run, _ = run_with([dict(result="docked")], "dock")
    cfg = eng.config(sensors=WWG, start_from="tape", dock_poses=[[200.0, 200.0, 0.0]])
    tr, _ = eng.run_engine([run], [], cfg)
    assert not getattr(tr["B-1"], "dock_fixes", [])
    assert tr["B-1"].dock_rejected == 1


def test_touch_is_a_range_to_the_face():
    # robot ends at x = 15 (true); a board face at x = 15 + 6.09 + 0.02: its left lever clicked
    face = lm.Face("board front", (15.0 + 6.09 + 0.02, -20.0), (15.0 + 6.09 + 0.02, 20.0))
    run, _ = run_with([dict(result="touched", side="L")], "touch", scale=1.03)
    base = eng.config(sensors=WWG, start_from="tape", touch_faces=[face])
    tr, _ = eng.run_engine([run], [], base)
    x, _, _ = final(tr["B-1"])
    off, _ = eng.run_engine([run], [], eng.config(sensors=WWG, start_from="tape", dock_fix=False))
    x0, _, _ = final(off["B-1"])
    assert abs(x0 - 15.0) > 0.3                    # the wheels alone are 3% long
    assert abs(x - 15.0) < abs(x0 - 15.0) / 3       # the touch pulls the along-track position in
    assert tr["B-1"].touch_used == 1


def test_stripe_check_and_geometry_from_params():
    run, _ = run_with([], "dock")
    cfg = eng.config()
    a, half, gap = dockfix.geometry(run, cfg)
    assert (a, half, gap) == (6.09, 3.91, 0.02)          # the run's own PARAM values (run_with)
    expect = -6.09 - (cfg["dock_stripe_x0_cm"] - cfg["line_forward_cm"])
    rec = dict(stripe_c_cm="10.0", travel_cm=str(10.0 + expect + 0.1))
    assert dockfix.stripe_check(rec, run, cfg) == pytest.approx(0.1)
    assert math.isnan(dockfix.stripe_check(dict(stripe_c_cm="-1.000", travel_cm="20"), run, cfg))


def test_map_home_dock_is_a_landmark_face():
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "fleet"))
    import fleetmap as FM
    m = FM.normalize({"home_docks": [{"name": "h1", "x": 50, "y": 20, "heading": 90}]})
    faces = lm.faces_from_map(m)
    f = [x for x in faces if x.name == "dock h1 back"][0]
    assert f.length == pytest.approx(FM.HOME_DOCK_DEFAULTS["pocket_w_cm"])   # the pocket fits the bracket
    assert (f.a[1] + f.b[1]) / 2 == pytest.approx(20.0)
