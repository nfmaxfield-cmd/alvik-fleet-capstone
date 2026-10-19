"""Light patterns (tools/lights): pattern scheme, detector + tracker on simulated frames drawn by the
simulator's renderer with the real camera preset (262 cm, 640 x 480, 3.4 px/cm), the offline CLI, the live
link and the simulator hooks. About a minute.

    python -m pytest tools/lights/tests -q

The rule every scene checks: no row is ever named for the wrong robot (wrong = 0) and no named row is more
than 3 cm from the named robot (far = 0).
"""
import csv
import json
import math
import os

import numpy as np
import pytest

import patterns as PT
import detect as D
import sim_scene as S


def ok(res):
    return res["wrong"] == 0 and res["far"] == 0


# ---------------------------------------------------------------- the scheme
def test_codes_and_capacity():
    fam = PT.L3Family()
    assert fam.parse("S0L") == ("S0L", "1111") and fam.parse("S0R/0011") == ("S0R", "0011")
    with pytest.raises(ValueError):
        fam.parse("S0L/0110")               # a rotation of 0011: the same cycle, not a code of its own
    assert fam.capacity() == dict(single_frame=2, with_sequence=10)
    wide = PT.L3Family(**PT.WIDE_L3)
    assert wide.capacity() == dict(single_frame=6, with_sequence=30)
    assert PT.StripFamily().capacity()["single_frame"] == 5
    # the printed holder's LEDs (hardware/dock/dock_params.py LED_POS_MM: (0,0), (24,0), (0,14) mm)
    L = fam.leds("S0L")
    assert np.allclose(L[1] - L[0], [2.4, 0]) and np.allclose(L[2] - L[0], [0, 1.4])
    assert PT.parse_names("", fam) == {"Left": "S0L", "Center": "S0R", "Right": "S0L/0011"}
    assert PT.parse_names("", wide) == {"Left": "S0L", "Center": "S1L", "Right": "S2L"}
    with pytest.raises(ValueError):
        PT.parse_names("A=S0L,B=S0L", fam)  # two robots, one code
    # side LED of 0011 is dark in slots 0-1, lit in 2-3 (0.16 s slots)
    assert [fam.lit("S0L/0011", 0.16 * k + 0.01)[2] for k in range(4)] == [False, False, True, True]
    # strip codes: start double, stop single, never symmetric, no two lit code LEDs side by side
    for c, b in PT.STRIP_CODES.items():
        assert b[:3] == "110" and b[6:] == "01" and b != b[::-1] and "11" not in b[2:]


def test_min_pixels_at_the_real_camera():
    ppc = 3.4 * 262 / (262 - 5)               # at the LEDs' height
    m = PT.min_pixels(PT.L3Family(), ppc)
    assert 4.5 < m["min_gap_px"] < 5.5        # 14 mm side arm
    s = PT.min_pixels(PT.StripFamily(), ppc)
    assert s["min_gap_px"] < 2                # Modulino LEDs ~1.6 px apart: not resolvable


def test_rigid_fit():
    T = np.array([[0, 0], [2.4, 0], [0, 1.4]])
    th, c = 0.7, np.array([10.0, -3.0])
    O = D.to_world(th, c, T)
    th2, c2, sse = D.rigid_fit(T, O)
    assert th2 == pytest.approx(th) and np.allclose(c2, c) and sse < 1e-12


# ---------------------------------------------------------------- real camera, printed holder
@pytest.fixture(scope="module")
def passing():
    return S.run(S.scene_pass(seed=0))


def test_named_from_one_frame_never_wrong(passing):
    r = passing
    assert ok(r)
    for n, pr in r["per"].items():
        assert pr["first_frames"] is not None and pr["first_frames"] <= 5, n      # 0.2 s at 25 fps
        assert pr["first_blink"] is None or pr["first_lights"] < pr["first_blink"]
        assert pr["named"] / pr["frames"] > 0.9
    h = np.array(r["herr"])
    assert np.percentile(h, 95) < 2.5 and h.max() < 6.0
    assert np.percentile(r["perr"], 95) < 0.5


def test_two_robots_side_by_side():
    r = S.run(S.scene_close(seed=1), with_blink=False)
    assert ok(r)
    assert all(pr["named"] / pr["frames"] > 0.6 for pr in r["per"].values())


def test_three_robots_sequence_tells_same_holders_apart():
    """Left S0L and Right S0L/0011 wear the same holder: Right is told apart by its side-LED cycle (0.64 s)."""
    r = S.run(S.scene_three(seed=0), with_blink=False)
    assert ok(r)
    for n, pr in r["per"].items():
        assert pr["first_lights"] is not None and pr["first_lights"] < 1.6, n


def test_one_led_hidden():
    for k in range(3):                          # corner, front, side covered for 2 s once named
        r = S.run(S.scene_single(seconds=5.0, hide=[("Left", k, 1.0, 3.0)]), with_blink=False)
        assert ok(r)
        assert r["per"]["Left"]["named"] / r["per"]["Left"]["frames"] > 0.6, k


def test_unlisted_robot_never_named():
    """A robot whose pattern isn't in the name table (Right wears S0L/0101; the table has Other = S0L/0011)
    is never given a listed name, though its holder looks like Left's and Other's."""
    sc = S.scene_three(seed=2, codes={"Left": "S0L", "Center": "S0R", "Right": "S0L/0101"})
    r = S.run(sc, names={"Left": "S0L", "Center": "S0R", "Other": "S0L/0011"}, with_blink=False)
    assert r["wrong"] == 0 and r["far"] == 0
    assert r["per"]["Right"]["named"] == 0 and r["per"]["Left"]["named"] > 0


@pytest.mark.parametrize("scale", [2.6, 5.0])
def test_other_scales(scale):
    r = S.run(S.scene_pass(scale=scale, seed=3), with_blink=False)
    assert ok(r)
    if scale >= 5:
        assert all(pr["first_frames"] is not None and pr["first_frames"] <= 2 for pr in r["per"].values())
    else:                                       # printed L at 2.6 px/cm: 14 mm = 3.7 px, too merged: decoder off
        assert r["named"] == 0
        w = S.run(S.scene_pass(scale=scale, seed=3, family=dict(PT.WIDE_L3)), with_blink=False)
        assert ok(w) and w["named"] > 50


def test_wide_l_three_variants_one_frame():
    r = S.run(S.scene_three(family=dict(PT.WIDE_L3), seed=1), with_blink=False)
    assert ok(r)
    assert all(pr["first_frames"] is not None and pr["first_frames"] <= 3 for pr in r["per"].values())


# ---------------------------------------------------------------- the strip
def test_strip_unreadable_at_real_camera():
    sc = S.scene_pass(mode="strip", seconds=3.0)
    r = S.run(sc, with_blink=False)
    assert r["named"] == 0 and r["stats"]["full"] == 0


def test_strip_reads_at_8_px_per_cm():
    r = S.run(S.scene_close(mode="strip", scale=8.0, seed=0), with_blink=False)
    assert ok(r)
    assert all(pr["first_lights"] is not None for pr in r["per"].values())
    assert np.percentile(r["herr"], 95) < 3.0


# ---------------------------------------------------------------- CLI, live link, simulator
@pytest.fixture(scope="module")
def recording(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("lights_rec"))
    S.write(S.scene_pass(seconds=4.0, seed=4), d)
    return d


def test_offline_cli_csv(recording):
    import lights as LI
    assert LI.main([recording, "--names", "Left=S0L,Center=S0R"]) == 0
    rows = list(csv.DictReader(open(os.path.join(recording, "lights_detections.csv"))))
    truth = {}
    for t in csv.DictReader(open(os.path.join(recording, "truth.csv"))):
        truth.setdefault(int(t["frame"]), {})[t["robot"]] = (float(t["x_cm"]), float(t["y_cm"]))
    named = [r for r in rows if r["robot"]]
    assert len(named) > 100 and set(r["robot"] for r in named) == {"Left", "Center"}
    assert set(rows[0]) >= {"frame", "t_wall", "robot", "x_px", "y_px", "heading_img_deg", "p_Left", "p_Center",
                            "p_none", "off_fwd_cm", "off_left_cm", "led_height_cm", "source"}
    # the fusion's loader: floor positions of the LED centre, near the named robot
    import runs as rn
    Hm, Hinv = rn.load_homography(os.path.join(recording, "floor_calibration.json"))
    L = rn.load_lights(os.path.join(recording, "lights_detections.csv"), Hinv, 0.0,
                       calib_path=os.path.join(recording, "floor_calibration.json"))
    for r in L:
        if r["robot"]:
            x, y = truth[r["frame"]][r["robot"]]
            assert math.hypot(r["x"] - x, r["y"] - y) < 3.0
            assert r["heading"] is not None


def test_live_replay_sends_pattern_fixes(recording, tmp_path):
    import track_live as TL
    out = []
    TL.main(["--replay", recording, "--speed", "0", "--dry-run", "--quiet", "--lights", "l3",
             "--lights-names", "Left=S0L,Center=S0R", "--names", "1=Left,2=Center", "--log", str(tmp_path)])
    msgs = [json.loads(l) for l in open(os.path.join(str(tmp_path), "messages.jsonl"))]
    lf = [m for m in msgs if m["type"] == "fix" and m.get("src") == "lights"]
    assert len(lf) > 100 and {m["robot"] for m in lf} == {"Left", "Center"}
    assert all(k in lf[0] for k in ("robot", "t", "x", "y", "th", "sigma", "sigma_th", "id_p"))


def test_live_default_unchanged(recording, tmp_path):
    import track_live as TL
    TL.main(["--replay", recording, "--speed", "0", "--dry-run", "--quiet", "--log", str(tmp_path)])
    msgs = [json.loads(l) for l in open(os.path.join(str(tmp_path), "messages.jsonl"))]
    assert not any(m.get("src") == "lights" for m in msgs)


def test_simulator_hooks():
    import config as SC
    import camera as CAM
    import simulate as SIM
    cfg = SC.load_config(S.REAL)
    assert cfg.lights.mode == "off"

    class R:                                   # just what set_lights and RobotTrack need
        name = "Right"
        params = {"lights_mode": 0.0, "lights_seq": 0.0, "lights_strip_code": 0.0}
        holder = None
    cfg.lights.mode = "l3"
    rb = R()
    SIM.set_lights(rb, cfg.lights)
    assert rb.holder == "S0L" and rb.params["lights_mode"] == 1.0 and PT.SEQS[int(rb.params["lights_seq"])] == "0011"
    tr = CAM.RobotTrack.__new__(CAM.RobotTrack)
    tr.pat = [(10.0, True, "S0L/0011"), (20.0, False, None)]
    tr.pat_t = [p[0] for p in tr.pat]
    fam = PT.L3Family()
    assert tr.pattern(fam, 9.0, 9.01) is None
    code, lv = tr.pattern(fam, 10.40, 10.41)               # slot 2: side lit
    assert code == "S0L/0011" and list(lv) == [1.0, 1.0, 1.0]
    assert list(tr.pattern(fam, 10.05, 10.06)[1]) == [1.0, 1.0, 0.0]
    assert tr.pattern(fam, 25.0, 25.01) is None
