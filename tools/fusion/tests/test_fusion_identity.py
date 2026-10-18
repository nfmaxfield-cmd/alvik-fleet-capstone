"""Blink light measurement model, identity combination, heading from motion.

    python -m pytest tools/fusion/tests -q
"""
import math

import numpy as np
import pytest

import engine as eng
from fusion_synth import straight_run, sightings, blink_rows
from ekf import EKF, D2R, wrap

CFG = eng.config()


# ---------------------------------------------------------------- light-offset measurement model
def test_light_h_and_jacobian():
    k = EKF(10.0, 20.0, 0.7, 0.0)
    off = (2.75, 0.4)
    h, H = k.light_h(off)
    c, s = math.cos(0.7), math.sin(0.7)
    assert h == pytest.approx([10 + c * 2.75 - s * 0.4, 20 + s * 2.75 + c * 0.4])
    eps = 1e-6
    for j in range(6):
        sp = k.s.copy()
        sp[j] += eps
        hp, _ = k.light_h(off, sp)
        assert (hp - h) / eps == pytest.approx(H[:, j], abs=1e-5)


def test_light_fixes_correct_heading_of_a_well_placed_robot():
    """Position known well, heading 15 deg off: fixes of the light (2.75 cm ahead)
    pull the heading back - the light is a weak heading measurement."""
    k = EKF(0.0, 0.0, 15 * D2R, 0.0, sigma_xy=0.1, sigma_th=20 * D2R)
    for _ in range(60):
        k.cam_light((2.75, 0.0), (2.75, 0.0), sigma=0.3)
    assert abs(k.s[2]) < 1.5 * D2R
    assert math.hypot(k.s[0], k.s[1]) < 0.3


def test_light_fix_is_not_a_centre_fix():
    """A light seen exactly where the light should be leaves the centre where it is."""
    k = EKF(5.0, 5.0, 90 * D2R, 0.0)
    k.cam_light((5.0, 7.75), (2.75, 0.0))
    assert k.s[0] == pytest.approx(5.0, abs=1e-6) and k.s[1] == pytest.approx(5.0, abs=1e-6)


# ---------------------------------------------------------------- identity combination
def test_combine_identity_naive_bayes_and_sources():
    pb = {"A": 0.95, "B": 0.03, "none": 0.02}
    post, best, src = eng.combine_identity(pb, None, 0.9)
    assert best == "A" and src == "blink" and post["A"] == pytest.approx(0.95)
    pm = {"A": 0.5, "B": 0.49, "none": 0.01}
    post, best, src = eng.combine_identity(pb, pm, 0.9)
    expect = 0.95 * 0.5 / (0.95 * 0.5 + 0.03 * 0.49 + 0.02 * 0.01)
    assert best == "A" and src == "blink" and post["A"] == pytest.approx(expect)
    post, best, src = eng.combine_identity({"A": 0.8, "B": 0.2}, {"A": 0.8, "B": 0.2}, 0.9)
    assert post["A"] == pytest.approx(0.64 / 0.68) and src == "combined"
    _, _, src = eng.combine_identity({"A": 0.95, "B": 0.05}, {"A": 0.97, "B": 0.03}, 0.9)
    assert src == "both"
    _, best, src = eng.combine_identity({"A": 0.5, "B": 0.5}, {"A": 0.99, "B": 0.01}, 0.9)
    assert best == "A" and src == "motion"
    _, _, src = eng.combine_identity({"A": 0.6, "B": 0.4}, None, 0.9)
    assert src is None
    assert eng.combine_identity(None, None, 0.9) == ({}, None, None)


def test_blink_dist_hypotheses():
    probs = {"Left": 0.02, "Center": 0.9, "Right": 0.03, "other": 0.0}
    d = eng.blink_dist(["Left", "none", "elsewhere"], probs, 0.05)
    # Center is not running: its probability goes to "elsewhere"
    assert d["elsewhere"] > 0.9 and d["Left"] < 0.03 and sum(d.values()) == pytest.approx(1.0)
    d = eng.blink_dist(["Bob", "none", "elsewhere"], {"Left": 0.0, "other": 0.97}, 0.03)
    assert d["Bob"] > 0.95                        # a robot without its own column takes p_other


def test_motion_likelihood_prefers_matching_robot():
    good = eng.motion_loglik((5.0, 0.1), (5.0, 0.0), 1.0)
    bad = eng.motion_loglik((5.0, 0.1), (0.0, 5.0), 1.0)
    assert good > bad + 10


# ---------------------------------------------------------------- heading from motion gating
def row(vl=10.0, vr=10.0, turn_dps=0.0, lifted=False):
    return (0.0, 0.0, 0.0, 0.0, vl, vr, turn_dps * D2R, lifted)


def test_hfm_gate():
    ok = [row() for _ in range(10)]
    assert eng.hfm_gate(ok, CFG)
    assert not eng.hfm_gate(ok[:5] + [row(vl=1.0)] + ok[5:], CFG)       # one wheel slow
    assert not eng.hfm_gate(ok[:5] + [row(turn_dps=25)] + ok[5:], CFG)  # turning
    assert not eng.hfm_gate(ok[:5] + [row(lifted=True)] + ok[5:], CFG)  # picked up
    assert not eng.hfm_gate([row(vl=-10, vr=-10)] * 10, CFG)            # backing up
    assert not eng.hfm_gate([row(vl=-10, vr=10)] * 10, CFG)             # turning in place
    assert not eng.hfm_gate(ok[:1], CFG)                                # too little data


def test_hfm_heading_direction_noise_and_speed():
    th = 30 * D2R
    pa, ta, tb = (0.0, 0.0), 0.0, 1.0
    slow = eng.hfm_heading(pa, (2 * math.cos(th), 2 * math.sin(th)), ta, tb, 0.0, 0.8, CFG)
    assert slow is None                                                  # 2 cm/s < 3 cm/s
    m10 = eng.hfm_heading(pa, (10 * math.cos(th), 10 * math.sin(th)), ta, tb, 0.0, 0.8, CFG)
    m20 = eng.hfm_heading(pa, (20 * math.cos(th), 20 * math.sin(th)), ta, tb, 0.0, 0.8, CFG)
    assert m10[0] == pytest.approx(th) and m20[0] == pytest.approx(th)
    assert m20[1] < m10[1]                                               # noise shrinks with speed
    assert m10[1] == pytest.approx(math.sqrt(2) * 0.8 / 10 + 1.0 * D2R)
    turned = eng.hfm_heading(pa, (10 * math.cos(th), 10 * math.sin(th)), ta, tb, 3 * D2R, 0.8, CFG)
    assert turned[0] == pytest.approx(th + 3 * D2R)                      # chord -> heading at the end


# ---------------------------------------------------------------- engine scenarios
def two_robots():
    """A drives +x along y=0; B drives +x along y=12 (12 cm to its left), same time."""
    ra, ta = straight_run("A-1", "A", 0.0, 0.0, 0.0, t0=1000.0)
    rb, tb = straight_run("B-1", "B", 0.0, 12.0, 0.0, t0=1000.0, v=6.0)
    return [ra, rb], ta, tb


def test_unchanged_without_new_inputs():
    runs, ta, tb = two_robots()
    dets = sightings(ta, seed=1) + sightings(tb, seed=2)
    blink = blink_rows(ta, "A", track=1) + blink_rows(tb, "B", track=2)
    cfg = eng.config(sensors=["wheel_v", "wheel_w", "gyro", "camera"])
    a, _ = eng.run_engine(runs, dets, cfg)
    b, _ = eng.run_engine(runs, dets, cfg, blink=blink)       # blink given but the setup doesn't use it
    for rid in a:
        assert a[rid].trajectory() == b[rid].trajectory()


def test_named_sighting_never_goes_to_another_robot():
    """A light named B (0.95) right where A's light should be is not given to A."""
    runs, ta, tb = two_robots()
    fake = [dict(b, x=b["x"], y=b["y"]) for b in blink_rows(ta, "B", p=0.95, track=9)]
    cfg = eng.config(sensors=["wheel_v", "wheel_w", "gyro", "blink"], motion_identity=False)
    tracked, unknown = eng.run_engine(runs, None, cfg, blink=fake)
    assert not tracked["A-1"].fixes                           # never A's
    assert not tracked["B-1"].fixes                           # and not B's either: 12 cm outside its area
    assert [t["name"] for t in unknown.tracks] == ["B"]       # kept as an unknown object, carrying its name


def test_unsure_name_falls_back_to_position_rules():
    runs, ta, tb = two_robots()
    rows = blink_rows(ta, "A", p=0.7, track=1)                # under 0.9: anonymous
    cfg = eng.config(sensors=["wheel_v", "wheel_w", "gyro", "blink"])
    tracked, _ = eng.run_engine(runs, None, cfg, blink=rows)
    fx = tracked["A-1"].fixes
    assert fx and all(f["source"] == "position" for f in fx)
    assert not tracked["B-1"].fixes


def test_identity_by_motion_and_blink_sources():
    runs, ta, tb = two_robots()
    dets = sightings(ta, seed=1) + sightings(tb, seed=2)
    blink = blink_rows(ta, "A", track=1) + blink_rows(tb, "B", track=2)
    cfg = eng.config(sensors=["wheel_v", "wheel_w", "gyro", "camera", "blink"], motion_identity=True)
    tracked, _ = eng.run_engine(runs, dets, cfg, blink=blink)
    for rid, truth in (("A-1", ta), ("B-1", tb)):
        tr = tracked[rid]
        assert tr.cam_used > 300
        src = tr.id_sources
        assert src.get("blink", 0) + src.get("both", 0) > 0.5 * tr.cam_used
        # every fix is near this robot's own true path at that moment
        for f in tr.fixes:
            _, x, y, th = min(truth, key=lambda p: abs(p[0] - f["t"]))
            if f["kind"] == "blink":
                x, y = x + 2.75 * math.cos(th), y + 2.75 * math.sin(th)
            assert math.hypot(f["x"] - x, f["y"] - y) < 3.0
    a = tracked["A-1"].id_sources
    assert a.get("both", 0) > 0                               # while driving, motion agrees with the blink name


def test_motion_identity_rejects_still_lookalike_near_a_driving_robot():
    """A static look-alike 2.5 cm beside a driving robot's path, and the robot
    itself not detected while it passes (22-38 cm): the look-alike is inside
    the possible area, but it doesn't move the way the wheels say."""
    run, truth = straight_run("A-1", "A", 0.0, 0.0, 0.0, drive_s=6.0)
    allr = sightings(truth, seed=1)
    dets = [d for d in allr if not 22 <= d["x"] <= 38]
    look = [dict(t=d["t"], frame=d["frame"], x=30.0, y=2.5, score=1.0, heading=None) for d in allr]
    cfg_off = eng.config(sensors=["wheel_v", "wheel_w", "gyro", "camera"])
    cfg_on = eng.config(sensors=["wheel_v", "wheel_w", "gyro", "camera"], motion_identity=True)
    off, _ = eng.run_engine([run], dets + look, cfg_off)
    on, _ = eng.run_engine([run], dets + look, cfg_on)
    bad = lambda tr: sum(1 for f in tr.fixes if math.hypot(f["x"] - 30.0, f["y"] - 2.5) < 0.5)
    assert bad(off["A-1"]) >= 5                               # the position rules alone take it
    assert bad(on["A-1"]) == 0
    assert on["A-1"].id_sources.get("motion", 0) > 0


def test_heading_from_motion_fixes_a_bad_start_heading():
    """Tape start heading 6 deg off; the camera fixes' direction of travel
    brings it back faster than position fixes alone."""
    run, truth = straight_run("A-1", "A", 0.0, 0.0, 0.0, drive_s=8.0, still_s=0.5)
    run.tape_start = (0.0, 0.0, 6.0)
    dets = sightings(truth, seed=3, noise=0.8, every=6)        # 5 frames/s, noisy detector
    base = dict(sensors=["wheel_v", "wheel_w", "gyro", "camera"], noise=dict(q_pos=0.0))
    off, _ = eng.run_engine([run], dets, eng.config(**base))
    on, _ = eng.run_engine([run], dets, eng.config(heading_from_motion=True, **base))
    assert on["A-1"].hdg_used["motion"] >= 4
    t_mid = truth[0][0] + 0.5 + 3.0
    err = lambda tr: abs(wrap(min(tr.trajectory(), key=lambda p: abs(p[0] - t_mid))[3]))
    assert err(on["A-1"]) < err(off["A-1"])


def test_calib_heading_fix_applied():
    run, truth = straight_run("A-1", "A", 0.0, 0.0, 0.0)
    run.tape_start = (0.0, 0.0, 4.0)
    rows = [dict(run="A-1", robot="A", event="nudge", identity="agree", t=truth[50][0], heading=0.0, led=None)]
    cfg = eng.config(sensors=["wheel_v", "wheel_w", "gyro"], calib_heading=True)
    tr, _ = eng.run_engine([run], None, cfg, calib_rows=rows)
    assert tr["A-1"].hdg_used["calib"] == 1
    assert abs(tr["A-1"].trajectory()[-1][3]) < 2.0 * D2R
    tr0, _ = eng.run_engine([run], None, eng.config(sensors=["wheel_v", "wheel_w", "gyro"]),   # off by default
                            calib_rows=rows)
    assert tr0["A-1"].hdg_used["calib"] == 0
    assert abs(tr0["A-1"].trajectory()[-1][3] - 4.0 * D2R) < 0.5 * D2R


def test_detector_track_carries_a_fading_name():
    """Lights lit only every 0.3 s: detector sightings between flashes keep the
    name (source blink); after the lights stop, the name fades out."""
    runs, ta, tb = two_robots()
    dets = sightings(ta, seed=1) + sightings(tb, seed=2)
    lit = lambda rows: [b for b in rows if (b["frame"] % 9) < 3]          # 100 ms of every 300 ms
    blink = lit(blink_rows(ta, "A", track=1)) + lit(blink_rows(tb, "B", track=2))
    blink = [b for b in blink if b["t"] < ta[0][0] + 4.0]                  # lights stop after 4 s
    cfg = eng.config(sensors=["wheel_v", "wheel_w", "gyro", "camera", "blink"])
    tracked, _ = eng.run_engine(runs, dets, cfg, blink=blink)
    for rid in ("A-1", "B-1"):
        det = [f for f in tracked[rid].fixes if f["kind"] == "det"]
        t0 = ta[0][0]
        early = [f for f in det if t0 + 0.5 < f["t"] < t0 + 4.0]
        late = [f for f in det if f["t"] > t0 + 6.0]
        assert early and sum(f["source"] == "blink" for f in early) > 0.95 * len(early)
        assert late and all(f["source"] == "position" for f in late)


def test_carried_name_dropped_near_another_object():
    ct = eng.CamTracks(eng.config()).get(("d", 1))
    ct["link"], ct["link_t"], ct["ll"] = ("b", 1), 10.0, {"A": -1.0}
    eng.CamTracks.forget(ct)
    assert ct["link"] is None and ct["ll"] == {}
    bt = eng.CamTracks(eng.config()).get(("b", 1))
    bt["probs"] = {"A": 1.0}
    eng.CamTracks.forget(bt)
    assert bt["probs"] == {"A": 1.0}                                       # a light keeps blink.py's name
