"""Tests for the 2026-10-25 discovery follow-up: motion response over moving windows only, the wheel-speed
calibration window, the tilt test, the synthetic calibration battery, and the threshold experiment's log pieces.

    python -m pytest tools/discovery/tests -v"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

import synth                                                              # noqa: E402
from discovery import battery as B, features as F, load, replay           # noqa: E402
from discovery import threshold_doe as TD, truth as T                     # noqa: E402

FULL = [("stand", 10.0), ("spins", (4, 90.0)), ("drive", (2, 20.0)), ("square", (12, 15.0)), ("tilt", (2, 20.0)),
        ("lift", (2, 5.0)), ("line", (2,)), ("stand", 5.0)]


def _by_col(key, feats, res):
    return {key[s]: (T.truth_for(key[s], bool(feats[s].get("const"))), r) for s, r in res.items()}


@pytest.fixture(scope="module")
def battery_result(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("battery"))
    case, key, secs, dur = B.load_plan_case(d, FULL, seed=3)
    feats, R, res, cal = replay.discover(case)
    return key, feats, res, secs, dur


def test_moving_response_ignores_parked_flicker():
    """A stream that flickers by 30 units while parked and swings by 3 units within each second while driving:
    over moving windows only, its figure is the driving spread, not the parked flicker."""
    rng = np.random.default_rng(0)
    n = 2000                                           # 100 s at 20 Hz
    moving = np.zeros(n, bool)
    moving[1000:] = True
    x = np.where(moving, 50 + 3 * np.sin(np.arange(n) / 4.0) + rng.normal(0, 1, n),
                 50 + rng.choice([0, 30], n) + rng.normal(0, 1, n))
    seg = np.zeros(n, int)
    aor, nw = F.moving_response(x, seg, moving, 20.0, 1.0, 0.1)
    assert nw >= 40
    assert 1.5 < aor < 4.0                             # about 2.3: sd of 3 sin (2.1) and noise (1), over rest noise 1
    aor0, nw0 = F.moving_response(x, seg, np.zeros(n, bool), 20.0, 1.0, 0.1)
    assert np.isnan(aor0) and nw0 == 0                 # no motion: no figure


def test_wheel_speed_calibration_uses_the_stretch_only(tmp_path):
    """The synthetic robot logs wheel rpm 1.08 high, as the robots do (tools/raw_units.py): per logged unit the
    calibration must come out at 2 pi 1.7 / 60 / 1.08 cm/s within 4%, now without the 1 s margins either side
    of each drive (with them it read about 10% low)."""
    d = synth.make_session(str(tmp_path), seed=2)
    case, key = load.load_case(d, "Synth")
    feats, R, res, cal = replay.discover(case)
    by = {key[s]: c for s, c in cal.items()}
    assert by["wl_rpm"]["physical_per_unit"] == pytest.approx(2 * np.pi * 1.7 / 60 / 1.08, rel=0.04)


def test_battery_identifies_every_stream(battery_result):
    key, feats, res, secs, dur = battery_result
    lab = _by_col(key, feats, res)
    wrong = {c: (t, r["type"]) for c, (t, r) in lab.items() if t != r["type"]}
    assert not wrong, wrong
    assert 120 < dur < 170
    assert set(secs) == {"stand", "spins", "drive", "square", "tilt", "lift", "line"}


def test_tilt_gives_direct_evidence_for_the_other_imu_axes(battery_result):
    """With hand tilts, x/y gyro axes and the non-gravity accelerometer axes are accepted from the tilt test (not
    asked about from a shared noise floor), and the computed roll/pitch come out derived."""
    key, feats, res, secs, dur = battery_result
    lab = _by_col(key, feats, res)
    for col in ("ax", "ay", "gx", "gy"):
        t, r = lab[col]
        assert r["type"] == t and r["decision"] == "accept", (col, r["type"], r["confidence"], r["evidence"])
        assert any("tilt test" in e for e in r["evidence"]), col
    for col in ("roll_deg", "pitch_deg"):
        assert lab[col][1]["type"] == "derived", (col, lab[col][1]["evidence"])


def test_tilt_test_silent_without_tilts(tmp_path):
    """No tilt in the log: no stream gets tilt evidence (the rule must not fire on drives and turns)."""
    plan = [p for p in FULL if p[0] not in ("tilt", "lift")]
    case, key, secs, dur = B.load_plan_case(str(tmp_path), plan, seed=5)
    feats, R, res, cal = replay.discover(case)
    hits = [key[s] for s, r in res.items() if any("tilt test" in e for e in r["evidence"])]
    assert not hits, hits


def test_lift_makes_the_lifted_flag_a_switch(battery_result):
    key, feats, res, secs, dur = battery_result
    lab = _by_col(key, feats, res)
    assert lab["lifted"][0] == "switch" and lab["lifted"][1]["type"] == "switch"


def test_piece_holds_the_asked_motion(tmp_path):
    d = synth.make_session(str(tmp_path), seed=1)
    case, key = load.load_case(d, "Synth")
    ij, m = TD.cut(case, key, 20.0, seed=4)
    assert ij is not None and m == pytest.approx(20.0, abs=0.5)
    p = TD.piece(case, ij[0], ij[1], "t")
    assert len(p.t) == ij[1] - ij[0]
    assert all(p.t[0] <= L.t0 and L.t1 <= p.t[-1] for L in p.legs)
    assert TD.cut(case, key, 1e6, seed=4)[0] is None      # more motion than the case holds


def test_threshold_bootstrap_counts():
    """Per-robot counts at a threshold and the zero-wrong upper bound (rule of three)."""
    rows = []
    for c in ("A", "B"):
        for i in range(10):
            rows.append({"case": c, "piece": "p", "guess": "range", "truth": "range", "confidence": 0.95,
                         "top3": "range;line"})
        rows.append({"case": c, "piece": "p", "guess": "line", "truth": "range", "confidence": 0.75,
                     "top3": "line;range"})
    pr = TD.per_robot(rows, 0.80)
    assert pr[("A", "p")] == [10, 0, 1, 0, 0]
    st = TD.boot(rows, 0.70, n_boot=200)
    assert st["wrong_total"] == 2 and st["accepted_total"] == 22
    st = TD.boot(rows, 0.90, n_boot=200)
    assert st["wrong_total"] == 0 and st["wrong_share_hi"] == pytest.approx(3 / 20)
