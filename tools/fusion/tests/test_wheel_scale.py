"""Wheel size in the filter and the factor graph (engine wheel_scale / turn_scale).

    python -m pytest tools/fusion/tests -q
"""
import math

import pytest

import engine as eng
import graph
from fusion_synth import straight_run, sightings
from test_landmarks import drive, wall, end_err, WWG


def test_off_by_default_and_unobservable_scale_never_moves():
    """Wheels + gyro only: nothing measures position, so k and kw stay exactly at their start and the
    path is the same as without the states (to rounding)."""
    for ws in (1.0, 1.05):
        run, truth = drive([wall(100.0)], wheel_scale=ws)
        for sens in (WWG, ["wheel_v", "wheel_w"], ["wheel_v", "gyro"]):
            off = eng.run_engine([run], None, eng.config(sensors=sens))[0]["r1"]
            on = eng.run_engine([run], None, eng.config(sensors=sens, wheel_scale=True, turn_scale=True))[0]["r1"]
            assert off.ekf.N == 6 and on.ekf.N == 8
            assert on.ekf.s[6] == 1.0 and on.ekf.s[7] == 1.0
            a, b = off.trajectory()[-1], on.trajectory()[-1]
            assert math.hypot(a[1] - b[1], a[2] - b[2]) < 1e-6


def test_landmarks_learn_the_wheel_scale_and_remove_the_lag():
    faces = [wall(100.0)]
    run, truth = drive(faces, wheel_scale=1.05)
    plain = eng.run_engine([run], None, eng.config(sensors=WWG + ["tof"]), landmarks=faces)[0]["r1"]
    sc = eng.run_engine([run], None, eng.config(sensors=WWG + ["tof"], wheel_scale=True), landmarks=faces)[0]["r1"]
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
