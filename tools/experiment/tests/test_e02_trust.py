"""The learned sensor trust and the bad-sensor injection (Nick, 2026-10-24)."""
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import e02_degrade as DG  # noqa: E402
import e02_fit as FIT  # noqa: E402
import e02_trust as TR  # noqa: E402


def _cam_finds(t, sd, n=10, seed=1):
    rng = random.Random(seed)
    for _ in range(n):
        t.camera_at_find("Left", rng.gauss(0, sd), rng.gauss(0, sd))


def test_healthy_until_there_is_evidence():
    t = TR.SensorTrust()
    assert t.k("camera") == 1.0 and t.k("wheels", "Left") == 1.0
    assert t.update_flags(["Left"]) == []


def test_a_noisy_camera_is_caught_and_counted_less():
    t = TR.SensorTrust()
    _cam_finds(t, 0.8)
    assert t.k("camera") < TR.DEGRADED_K
    t2 = TR.SensorTrust()
    _cam_finds(t2, 3.0, n=12)
    assert t2.k("camera") > TR.DEGRADED_K
    ch = t2.update_flags(["Left"])
    assert ("camera", "*", "degraded") in [c[:3] for c in ch]
    # KC: the camera's sd is scaled up, so the fix leans on the robot more
    rsd, csd = t2.kc_sds("Left", 1.0, 1.0)
    assert csd > 2.0 and rsd == 1.0


def _stretches(t, robot, rms_cm, rms_deg=1.2, n=6):
    for _ in range(n):
        t.stretch_fit(robot, rms_cm, rms_deg, 10)


def test_wheels_blamed_on_the_one_robot_that_stands_out():
    t = TR.SensorTrust()
    _cam_finds(t, 0.8)
    _stretches(t, "Left", 1.3)
    _stretches(t, "Center", 1.4)
    _stretches(t, "Right", 4.0)               # Right's wheels are bad
    assert t.k("wheels", "Right") > TR.DEGRADED_K
    assert t.k("wheels", "Left") < TR.RECOVERED_K and t.k("wheels", "Center") < TR.RECOVERED_K
    flags = [c[:3] for c in t.update_flags(["Left", "Center", "Right"])]
    assert ("wheels", "Right", "degraded") in flags and ("wheels", "Left", "degraded") not in flags


def test_what_every_robot_shares_is_the_camera_not_the_wheels():
    # 2026-10-24 Pi plan: all three robots 2.1-2.5 cm a stretch with the session-1 calibration file
    t = TR.SensorTrust()
    _cam_finds(t, 0.8)
    for r, v in (("Left", 2.2), ("Center", 2.4), ("Right", 2.1)):
        _stretches(t, r, v)
    assert all(t.k("wheels", r) < TR.DEGRADED_K for r in ("Left", "Center", "Right"))
    assert t.update_flags(["Left", "Center", "Right"]) == []
    # and the self-calibration's limit grows with the camera's share: 2.1-2.5 cm passes
    assert t.wheel_gate_cm() > 2.5
    assert t.wheel_gate_cm(override=1.5) == 1.5


def test_one_robot_takes_the_cameras_share_from_the_finds():
    t = TR.SensorTrust()
    _cam_finds(t, 0.8)
    _stretches(t, "Right", 1.3)
    sd, n, how = t.own_sd("wheels", "Right")
    assert "finds" in how and sd < 1.0


def test_flags_recover_with_hysteresis_and_old_sessions_fade():
    t = TR.SensorTrust()
    _cam_finds(t, 3.0, n=12)
    t.update_flags([])
    assert t.bad("camera")
    for _ in range(4):
        t.new_session()                       # the bad readings' weights halve each session
    _cam_finds(t, 0.7, n=12, seed=5)
    t.update_flags([])
    assert not t.bad("camera")


def test_front_sensor_sd_takes_off_the_cameras_share():
    t = TR.SensorTrust()
    _cam_finds(t, 0.8)
    rng = random.Random(2)
    for _ in range(20):
        t.front_vs_map("Left", -2.0 + rng.gauss(0, 4.0))
    assert t.k("front", "Left") < TR.DEGRADED_K
    for _ in range(40):
        t.front_vs_map("Center", rng.gauss(0, 15.0))
    assert t.k("front", "Center") > TR.DEGRADED_K


def test_kc_gate_rejects_a_belief_the_camera_contradicts():
    w, rej = TR.kc_gate((98.4, 93.7), (106.8, 101.1), 0.6, 1.1)      # Right's KC run, 2026-10-24
    assert rej and w == TR.KC_GATE_WEIGHT
    w, rej = TR.kc_gate((98.4, 93.7), (99.0, 94.0), 0.6, 1.1)
    assert not rej and 0.1 < w < 0.3


def test_save_and_load(tmp_path):
    t = TR.SensorTrust()
    _cam_finds(t, 3.0, n=12)
    t.update_flags([])
    p = str(tmp_path / "trust.json")
    TR.save(t, p)
    t2 = TR.load(p)
    assert t2.bad("camera") and abs(t2.k("camera") - t.k("camera")) < 1e-9
    assert TR.load(str(tmp_path / "none.json")).k("camera") == 1.0


def test_degrade_parse_and_words():
    d = DG.Degrade("camera:noise=3,wheels:jitter=0.05,camera:bias=2/-1,front:bias=5")
    assert d.spec["camera"] == {"noise": 3.0, "bias": (2.0, -1.0)} and d.spec["front"]["bias"] == 5.0
    assert "DEGRADED ON PURPOSE" in d.words()
    assert not DG.Degrade("")
    try:
        DG.parse("lidar:noise=1")
        raise AssertionError("accepted an unknown sensor")
    except ValueError:
        pass


def test_degraded_camera_holds_its_offset_then_changes():
    d = DG.Degrade("camera:noise=3", seed=4)
    a = d.camera((0.0, 50.0, 50.0))
    b = d.camera((1.0, 50.0, 50.0))
    c = d.camera((DG.HOLD_S + 0.1, 50.0, 50.0))
    assert a[1:] == b[1:] and a[1:] != (50.0, 50.0) and c[1:] != a[1:]
    assert DG.Degrade("camera:drop=1.0").camera((0.0, 1.0, 1.0)) is None
    assert DG.Degrade("camera:bias=2/-1").camera((0.0, 10.0, 10.0))[1:3] == (12.0, 9.0)


def test_degraded_wheels_drive_and_report_the_same_factor():
    d = DG.Degrade("wheels:jitter=0.1", seed=3)
    v = d.corrections("Left", dict(gyro_scale=1.07, gyro_drift_dps=0.0, wheel_scale=0.98))
    fw = d.factors[("Left", "wheels")]
    assert abs(v["wheel_scale"] - 0.98 * fw) < 1e-12 and fw != 1.0 and v["gyro_drift_dps"] == 0.0
    rows = [dict(ms=0, x_cm=0.0, y_cm=0.0, yaw_deg=170.0), dict(ms=100, x_cm=10.0, y_cm=0.0, yaw_deg=-170.0)]
    out = d.raw_rows("Left", rows)
    assert abs(out[1]["x_cm"] - 10.0 * fw) < 1e-9
    assert abs(out[1]["yaw_deg"] - (-170.0)) < 1e-9          # gyro jitter off: the wrapped yaw comes back as it was
    assert abs(d.front(30.0) - 30.0) < 1e-9


def test_fit_uses_parts_as_they_pass():
    # a gyro that passes and wheels that are refused: the gyro's values are used, the wheels' from the prior
    obs = []
    import e02_testkit as KIT  # noqa: F401  (the kinematic robot)
    rb = KIT.KinRobot()
    for _ in range(3):
        rb.go(40)
        rb.turn(90)
        rb.go(30)
        rb.turn(90)
    obs = FIT.pair(FIT.camera_stretches(rb.cam), FIT.raw_series(rb.rawrows), run="a")
    res = FIT.fit_robot(obs, max_rms_cm=0.0)                  # every wheel fit refused
    if res["gyro_ok"]:
        assert res["values"] is not None and res["values"]["prior_parts"] == ["wheel_scale"]
        assert "not learned yet" in FIT.describe(res["values"])
