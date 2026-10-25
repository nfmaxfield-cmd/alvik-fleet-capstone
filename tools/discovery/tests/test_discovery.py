"""Tests for blind sensor discovery (tools/discovery). Run from the repo root:

    python -m pytest tools/discovery/tests -v

They use a synthetic robot (synth.py) whose every stream's type is known, written as a session folder in the
console's layout, so the whole path (load -> features -> relations -> classify -> calibrate -> score) runs as on
real logs. About 30 s."""
import ast
import csv
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

import synth                                                     # noqa: E402
from discovery import load, features as F, classify as C        # noqa: E402
from discovery import calibrate as CAL, replay, score as S       # noqa: E402
from discovery import truth as T                                 # noqa: E402

PKG = os.path.dirname(HERE)


@pytest.fixture(scope="module")
def session(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("synth"))
    return synth.make_session(d, seed=1)


@pytest.fixture(scope="module")
def result(session):
    case, key = load.load_case(session, "Synth")
    feats, R, res, cal = replay.discover(case)
    return case, key, feats, R, res, cal


def _labels(key, feats, res):
    return {key[s]: (T.truth_for(key[s], feats[s].get("const")), r) for s, r in res.items()}


# ---- the rules of the game: no names reach the discoverer ----

def test_stream_ids_hide_names(session):
    case, key = load.load_case(session, "Synth")
    assert sorted(key.values()) == sorted(synth.COLUMNS)
    for s in case.streams:
        assert s.startswith("stream_") and s not in synth.COLUMNS
    # the order is shuffled (column position would be a hint)
    assert [key[s] for s in sorted(case.streams)] != synth.COLUMNS


@pytest.mark.parametrize("mod", ["features.py", "relations.py", "classify.py", "calibrate.py"])
def test_discoverer_never_imports_the_key(mod):
    tree = ast.parse(open(os.path.join(PKG, mod)).read())
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            names |= {a.name for a in node.names} | {node.module or ""}
        elif isinstance(node, ast.Import):
            names |= {a.name for a in node.names}
    assert not any("truth" in n or "score" in n for n in names), names


def test_labels_do_not_depend_on_stream_ids(session):
    """Another shuffle (other ids, other order) gives the same type for every real column."""
    a, ka = load.load_case(session, "Synth")
    b, kb = load.load_case(session, "Synth", seed_extra="other")
    assert ka != kb
    ra = replay.discover(a)[2]
    rb = replay.discover(b)[2]
    la = {ka[s]: r["type"] for s, r in ra.items()}
    lb = {kb[s]: r["type"] for s, r in rb.items()}
    assert la == lb


# ---- what it finds on the synthetic robot ----

def test_synthetic_accuracy(result):
    case, key, feats, R, res, cal = result
    lab = _labels(key, feats, res)
    right = sum(t == r["type"] for t, r in lab.values())
    assert right >= 33, {k: (t, r["type"]) for k, (t, r) in lab.items() if t != r["type"]}


@pytest.mark.parametrize("col,typ", [
    ("ms", "clock"), ("raw_n", "counter"), ("run", "id"), ("swL", "constant"),
    ("wl_deg", "wheel_angle"), ("wr_rpm", "wheel_speed"), ("gz", "gyro_rate"), ("az", "accel"),
    ("yaw_deg", "gyro_angle"), ("x_cm", "derived"), ("y_cm", "derived"), ("th_deg", "derived"),
    ("v_cms", "derived"), ("lineC", "line")])
def test_key_streams(result, col, typ):
    case, key, feats, R, res, cal = result
    s = [k for k, v in key.items() if v == col][0]
    assert res[s]["type"] == typ, (col, res[s]["type"], res[s]["evidence"])


def test_derived_streams_are_flagged_not_independent(result):
    """The pose (x, y, heading) and the drive speed are recognised as computed from other streams."""
    case, key, feats, R, res, cal = result
    flagged = {key[s] for s, r in res.items() if r["derived"]}
    assert {"x_cm", "y_cm", "th_deg", "v_cms"} <= flagged
    assert not flagged & {"wl_deg", "wr_deg", "gz", "az", "lineL"}


def test_centre_zone_is_found_as_the_mean_of_its_neighbours(result):
    case, key, feats, R, res, cal = result
    s = [k for k, v in key.items() if v == "dC_cm"][0]
    assert res[s]["derived"] and sorted(key[x] for x in res[s]["derived_of"]) == ["dCL_cm", "dCR_cm"]


def test_integration_pairs(result):
    case, key, feats, R, res, cal = result
    pairs = {(key[a], key[b]) for a, b, r in R["integration"]}
    assert ("wl_deg", "wl_rpm") in pairs and ("wr_deg", "wr_rpm") in pairs
    # the yaw angle integrates the turn rate: the gyro or its processed copy (w_dps), whichever fits best
    assert ("yaw_deg", "gz") in pairs or ("yaw_deg", "w_dps") in pairs


def test_calibration_recovers_known_scales(result):
    """Wheel: 2 pi 1.7 / 360 cm per degree; wheel speed: 2 pi 1.7 / 60 cm/s per rpm; gyro: the synthetic gyro
    reads 3% short, so the camera says 1/0.97 deg per unit; gravity axis: 1 g."""
    case, key, feats, R, res, cal = result
    by = {key[s]: c for s, c in cal.items()}
    assert by["wl_deg"]["physical_per_unit"] == pytest.approx(2 * np.pi * 1.7 / 360, rel=0.05)
    assert by["wr_rpm"]["physical_per_unit"] == pytest.approx(2 * np.pi * 1.7 / 60, rel=0.06)
    assert by["gz"]["physical_per_unit"] == pytest.approx(1 / 0.97, rel=0.03)
    assert by["gz"]["unit_guess"] == "same units"
    assert by["az"]["physical_per_unit"] == pytest.approx(1.0, rel=0.01)


def test_disguised_units_keep_the_labels(session, result):
    """Rescaled, offset and sign-flipped streams (units hidden) must keep the motion-based labels."""
    case, key, feats, R, res, cal = result
    c2 = replay.disguise(case, key, seed=3)
    res2 = replay.discover(c2)[2]
    for col in ("wl_deg", "wr_rpm", "gz", "x_cm", "y_cm", "th_deg", "v_cms", "dL_cm"):
        s = [k for k, v in key.items() if v == col][0]
        assert res2[s]["type"] == res[s]["type"], (col, res2[s]["type"], res2[s]["evidence"])


def test_short_still_log_does_not_overclaim(tmp_path):
    """15 s of mostly standing: nothing about motion can be known, so no motion sensor is accepted wrongly."""
    d = synth.make_session(str(tmp_path), seed=4, n_actions=3)
    case, key = load.load_case(d, "Synth")
    feats, R, res, cal = replay.discover(case)
    wrong_accepted = [key[s] for s, r in res.items() if r["decision"] == "accept"
                      and r["type"] != T.truth_for(key[s], feats[s].get("const"))]
    assert not wrong_accepted


# ---- pieces ----

def test_parse_text_and_numbers():
    x, txt = F.parse(np.array(["052E-3", "052E-4"], dtype=object))
    assert txt                         # a run id that would read as 0.052 is still text
    x, txt = F.parse(np.array(["1.5", "-2", ""], dtype=object))
    assert not txt and x[0] == 1.5 and np.isnan(x[2])


def test_wrap_detection():
    t = np.arange(2000) / 50.0
    ang = (t * 40.0) % 360.0
    seg = np.zeros(len(t), int)
    w, p = F.detect_wrap(ang, seg)
    assert w and p == pytest.approx(360, rel=0.02)
    rng = np.random.default_rng(0)
    zone = np.where(rng.random(2000) < 0.05, 400.0, 30 + rng.normal(0, 1, 2000))   # range dropouts
    assert not F.detect_wrap(zone, seg)[0]


def test_unit_snap():
    assert CAL.snap(57.0)[0] == "rad -> deg"
    assert CAL.snap(1.05)[0] == "same units"
    assert CAL.snap(0.0297)[0] == "no common unit"


def test_decide_thresholds():
    assert C.decide(0.95) == "accept" and C.decide(0.7) == "ask" and C.decide(0.3) == "unknown"
    assert C.decide(0.85, accept=0.8) == "accept"


def test_score_functions():
    rows = [{"truth": "range", "guess": "range", "confidence": 0.95, "top3": [("range", .95)], "derived": False,
             "column": "dL_cm"},
            {"truth": "line", "guess": "range", "confidence": 0.92, "top3": [("range", .92)], "derived": False,
             "column": "lineL"},
            {"truth": "derived", "guess": "derived", "confidence": 0.7, "top3": [("derived", .7)], "derived": True,
             "column": "x_cm"}]
    assert S.accuracy(rows) == (2, 3)
    types, M = S.confusion(rows)
    assert M[types.index("line"), types.index("range")] == 1
    sw = {(r["accept"], r["ask"]): r for r in S.sweep(rows)}
    assert sw[(0.9, 0.6)]["accepted"] == 2 and sw[(0.9, 0.6)]["accepted_wrong"] == 1
    assert sw[(0.9, 0.6)]["questions"] == 1
    cal = {b["band"]: b for b in S.calibration(rows)}
    assert cal["0.90-0.95"]["n"] == 1 and cal["0.90-0.95"]["right"] == 0
    assert S.derived_scores(rows) == {"computed": 1, "flagged": 1, "sensors": 2, "sensors_flagged": 0}


def test_gyro_angle_carries_derived_provenance(result):
    case, key, feats, R, res, cal = result
    s = [k for k, v in key.items() if v == "yaw_deg"][0]
    assert res[s]["type"] == "gyro_angle" and res[s]["derived"] and res[s]["derived_of"]


def test_find_sessions_uses_one_copy(tmp_path):
    for sub in ("a", "b"):
        synth.make_session(str(tmp_path / sub / "e02-20261231-000000"), seed=2, n_actions=2)
    found = load.find_sessions(str(tmp_path))
    assert list(found) == ["e02-20261231-000000"]
