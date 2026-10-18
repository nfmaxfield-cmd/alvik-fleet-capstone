"""Wheel size in the filter and the factor graph (engine wheel_scale / turn_scale).

    python -m pytest tools/fusion/tests -q
"""
import math

import pytest

import engine as eng
import graph
from fusion_synth import straight_run, sightings
from test_landmarks import drive, wall, end_err, WWG


def test_on_by_default_off_switch_and_unobservable_scale_never_moves():
    """Wheel size is on by default and wheel_scale False switches it off. Wheels + gyro only: nothing
    measures position, so k and kw stay exactly at their start and the path is the same as without the
    states (to rounding)."""
    assert eng.config()["wheel_scale"] is True
    for ws in (1.0, 1.05):
        run, truth = drive([wall(100.0)], wheel_scale=ws)
        for sens in (WWG, ["wheel_v", "wheel_w"], ["wheel_v", "gyro"]):
            off = eng.run_engine([run], None, eng.config(sensors=sens, wheel_scale=False))[0]["r1"]
            on = eng.run_engine([run], None, eng.config(sensors=sens, turn_scale=True))[0]["r1"]
            dflt = eng.run_engine([run], None, eng.config(sensors=sens))[0]["r1"]
            assert off.ekf.N == 6 and on.ekf.N == 8 and dflt.ekf.N == 8
            assert dflt.ekf.s[6] == 1.0
            assert on.ekf.s[6] == 1.0 and on.ekf.s[7] == 1.0
            a, b = off.trajectory()[-1], on.trajectory()[-1]
            assert math.hypot(a[1] - b[1], a[2] - b[2]) < 1e-6


def test_landmarks_learn_the_wheel_scale_and_remove_the_lag():
    faces = [wall(100.0)]
    run, truth = drive(faces, wheel_scale=1.05)
    plain = eng.run_engine([run], None, eng.config(sensors=WWG + ["tof"], wheel_scale=False), landmarks=faces)[0]["r1"]
    sc = eng.run_engine([run], None, eng.config(sensors=WWG + ["tof"]), landmarks=faces)[0]["r1"]
    assert sc.ekf.s[6] == pytest.approx(1.05, abs=0.01)
    assert end_err(plain, truth) > 0.6                       # the lag the filter had without it
    assert end_err(sc, truth) < 0.25
    g = graph.smooth(sc)
    assert math.hypot(g[-1][1] - truth[-1][1], g[-1][2] - truth[-1][2]) < 0.3


def test_camera_learns_scale_and_next_run_starts_from_it():
    """Two runs of the same robot: the first learns k from the camera; the second starts from it
    (scale_carry) with the camera blocked after 1.5 s, and drifts less than starting from 1."""
    runs, truths = [], []
    for i, t0 in enumerate((1000.0, 1100.0)):
        run, truth = straight_run(f"r{i}", "A", 0.0, 0.0, 0.0, t0=t0, still_s=1.0, drive_s=6.0)
        for r in run.raw:
            r["v_cms"] *= 1.05
            r["wl_rpm"] *= 1.05
            r["wr_rpm"] *= 1.05
        runs.append(run)
        truths.append(truth)
    dets = sightings(truths[0], seed=2) + [d for d in sightings(truths[1], seed=3) if d["t"] < 1101.5]
    errs = {}
    for carry in (True, False):
        cfg = eng.config(sensors=WWG + ["camera"], wheel_scale=True, scale_carry=carry)
        tr, _ = eng.run_engine(runs, dets, cfg)
        assert tr["r0"].ekf.s[6] == pytest.approx(1.05, abs=0.01)
        errs[carry] = end_err(tr["r1"], truths[1])
        if carry:
            assert tr["r1"].scale0["k"] == pytest.approx(tr["r0"].ekf.s[6])
            assert tr["r1"].scale0["sk"] < 0.01
    assert errs[True] < 0.5 * errs[False]


def test_turn_scale_only_with_gyro():
    assert eng.scale_prior(eng.config(sensors=["wheel_v", "wheel_w"], turn_scale=True)) is not None
    assert eng.scale_prior(eng.config(sensors=["wheel_v", "wheel_w"], turn_scale=True))["skw"] == 0.0
    assert eng.scale_prior(eng.config(sensors=WWG, turn_scale=True))["skw"] == 0.05


def test_calibration_run_scales_set_the_priors():
    """calibration_results.csv (firmware 2026-10-18b calibration runs): wheel_scale = camera cm / odometry cm,
    so wheels reading 5% long give wheel_scale 1/1.05 and the robot starts from k = 1.05; gyro_scale
    multiplies its gyro. Rows not rated ok are ignored."""
    import runs as rn
    rows = [dict(run="X-1", robot="A", event="calib", identity="agree", wheel_scale=1 / 1.05,
                 wheel_scale_spread=0.001, gyro_scale=1.01, gyro_scale_spread=0.0005),
            dict(run="X-2", robot="B", event="calib", identity="agree", wheel_scale=None,
                 wheel_scale_spread=float("nan"), gyro_scale=None, gyro_scale_spread=float("nan"))]
    sc = rn.calib_scales(rows)
    assert sc == {"A": dict(k=pytest.approx(1.05), sk=0.003, gs=1.01)}
    run, truth = drive([wall(100.0)], wheel_scale=1.05)
    cfg = eng.config(sensors=WWG, wheel_scale=True)
    tr = eng.run_engine([run], None, cfg, calib_rows=[dict(rows[0], robot="A")])[0]["r1"]
    assert tr.scale0["k"] == pytest.approx(1.05) and tr.scale0["sk"] == 0.003
    assert end_err(tr, truth) < 0.3                           # the wheels' 5% is taken out from the start
