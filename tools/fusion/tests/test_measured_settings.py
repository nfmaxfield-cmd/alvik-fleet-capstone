"""Settings that come from the session's own files instead of simulator numbers:
camera view area, start-nudge heading noise, recording frame rate, per-robot
wheel radius, and the measured-noise file.

    python -m pytest tools/fusion/tests -q
"""
import json
import math

import pytest

import check_session as cs
import engine as eng
import runs as rn
from ekf import Noise, load_noise_file, D2R


def test_view_area_from_polygon_or_image_corners(tmp_path):
    p = tmp_path / "cal.json"
    p.write_text(json.dumps({"H_floor_to_px": [[2, 0, 0], [0, 2, 0], [0, 0, 1]],
                             "view_floor_polygon_cm": [[0, 0], [100, 0], [100, 50], [0, 50]]}))
    assert rn.view_area_cm2(str(p)) == pytest.approx(5000.0)
    p.write_text(json.dumps({"H_floor_to_px": [[2, 0, 0], [0, 2, 0], [0, 0, 1]], "image_size": [640, 480]}))
    assert rn.view_area_cm2(str(p)) == pytest.approx(320 * 240)          # 2 px per cm
    p.write_text(json.dumps({"H_floor_to_px": [[2, 0, 0], [0, 2, 0], [0, 0, 1]]}))
    assert rn.view_area_cm2(str(p)) is None
    assert eng.clutter_area(eng.config()) == 19200.0                       # auto with no calibration
    assert eng.clutter_area(eng.config(clutter_cm2=5000.0)) == 5000.0


def test_heading_px_sd_from_heading_check(tmp_path):
    p = tmp_path / "heading_check.csv"
    rows = ["run,robot,level,tape_heading_deg,camera_heading_deg,diff_deg,identity,moved_px,det_heading_deg"]
    for i, (d, det) in enumerate([(1.0, 0.5), (-1.0, -0.5), (0.5, 0.0), (-0.5, 0.2)]):
        rows.append(f"R{i},Left,L,0,{d},{d},agree,7.0,{det}")
    p.write_text("\n".join(rows) + "\n")
    pairs = [0.5 - 1.0, -0.5 + 1.0, 0.0 - 0.5, 0.2 + 0.5]
    m = sum(pairs) / 4
    sd = math.sqrt(sum((x - m) ** 2 for x in pairs) / 3 / 2)
    assert rn.heading_px_sd(str(p)) == pytest.approx(math.tan(math.radians(sd)) * 7.0)
    p.write_text(rows[0] + "\n" + rows[1] + "\n")
    assert rn.heading_px_sd(str(p)) is None                                 # too few runs


def test_recording_fps(tmp_path):
    assert cs.recording_fps(str(tmp_path), [0.0, 0.04, 0.08, 0.12]) == pytest.approx(25.0)
    (tmp_path / "meta.json").write_text(json.dumps({"fps_requested": 30, "fps_reported": 25.0}))
    assert cs.recording_fps(str(tmp_path), []) == 25.0
    assert cs.recording_fps(str(tmp_path / "none")) == 25.0             # nothing to go by: the T1 camera


def test_wheel_radius_per_robot_and_scale_prior():
    cfg = eng.config(wheel_radius_cm={"Left": 1.65, "default": 1.7}, wheel_scale=True)
    assert eng.radius_of(cfg, "Left") == 1.65 and eng.radius_of(cfg, "Right") == 1.7
    assert eng.scale_prior(cfg, None, "Left")["k"] == pytest.approx(1.7 / 1.65)
    assert eng.scale_prior(cfg, None, "Right")["k"] == pytest.approx(1.0)
    assert eng.scale_prior(eng.config(), None, "Left")["sk"] == 0.03        # on by default
    assert eng.scale_prior(eng.config(wheel_scale=False), None, "Left") is None   # off: no wheel-size states


def test_noise_file(tmp_path):
    p = tmp_path / "noise.json"
    p.write_text(json.dumps({"camera_xy_cm": 0.6, "camera_heading_deg": 5.0, "blink_xy_cm": 0.4,
                             "noise": {"g_base": 0.004}, "source": "T0 still placements"}))
    o = load_noise_file(str(p))
    assert o == {"cam_xy": 0.6, "cam_th": pytest.approx(5.0 * D2R), "blink_xy": 0.4, "g_base": 0.004}
    n = Noise(**o)
    assert n.cam_xy == 0.6
    p.write_text(json.dumps({"cam_xy_cm": 0.45, "cam_heading_deg": None, "blink_xy_cm": None,
                             "start_nudge_heading_deg": 1.7, "_made": "2026-10-18"}))   # accuracy_checks.py's file
    extra = {}
    assert load_noise_file(str(p), extra) == {"cam_xy": 0.45}
    assert extra == {"calib_heading_sigma_deg": 1.7}
    p.write_text(json.dumps({"noise": {"nonsense": 1}}))
    with pytest.raises(KeyError):
        load_noise_file(str(p))


def test_video_clock_file(tmp_path):
    """video_clock.json: video = laptop + offset + drift (laptop - t_ref); the drift only with use_drift."""
    d = dict(format="video-clock", version=1, offset_s=0.1, t_ref=1000.0, drift_ppm=50.0, use_drift=True)
    (tmp_path / "video_clock.json").write_text(json.dumps(d))
    c, how = rn.video_clock("unused", str(tmp_path))
    lap = 1600.0
    vid = lap + 0.1 + 50e-6 * (lap - 1000.0)
    assert rn.to_laptop(c, vid) == pytest.approx(lap, abs=1e-9)
    assert "ppm" in how
    d["use_drift"] = False
    (tmp_path / "video_clock.json").write_text(json.dumps(d))
    c, _ = rn.video_clock("unused", str(tmp_path))
    assert rn.to_laptop(c, vid) == pytest.approx(vid - 0.1)
    c, _ = rn.video_clock("unused", str(tmp_path), explicit=0.25)          # --video-offset wins
    assert c == 0.25 and rn.to_laptop(c, 10.0) == pytest.approx(9.75)


def test_still_from_wheel_angles():
    """Encoder test: still once neither wheel angle moved > 0.75 deg for 0.25 s, despite noisy wheel speeds;
    rows without wheel angles keep the old speed thresholds."""
    st_ = eng.StillTest(eng.config())
    out = []
    for i in range(60):                                   # 10 ms rows: 0.3 s still, then the wheels turn
        t = i * 0.01
        ang = 0.0 if i < 30 else (i - 29) * 0.5
        out.append(st_.step(dict(t=t, wl_deg=ang, wr_deg=-ang, wl_rpm=0.6, wr_rpm=-0.5, w_dps=1.0), 0.3))
    assert out[:25] == [False] * 25                       # not enough history yet
    assert all(out[26:30])                                # still, although the speeds read 0.6 rpm
    assert not any(out[32:])                              # moving (2 rows of slack: < 0.75 deg moved)
    old = eng.StillTest(eng.config())
    assert old.step(dict(t=0.0, wl_rpm=0.1, wr_rpm=0.1, w_dps=0.1), 0.05) is True
    assert old.step(dict(t=0.01, wl_rpm=0.6, wr_rpm=0.1, w_dps=0.1), 0.05) is False
