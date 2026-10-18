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
    assert cs.recording_fps(str(tmp_path / "none")) == 30.0


def test_wheel_radius_per_robot_and_scale_prior():
    cfg = eng.config(wheel_radius_cm={"Left": 1.65, "default": 1.7}, wheel_scale=True)
    assert eng.radius_of(cfg, "Left") == 1.65 and eng.radius_of(cfg, "Right") == 1.7
    assert eng.scale_prior(cfg, None, "Left")["k"] == pytest.approx(1.7 / 1.65)
    assert eng.scale_prior(cfg, None, "Right")["k"] == pytest.approx(1.0)
    assert eng.scale_prior(eng.config(), None, "Left") is None              # off: no wheel-size states


def test_noise_file(tmp_path):
    p = tmp_path / "noise.json"
    p.write_text(json.dumps({"camera_xy_cm": 0.6, "camera_heading_deg": 5.0, "blink_xy_cm": 0.4,
                             "noise": {"g_base": 0.004}, "source": "T0 still placements"}))
    o = load_noise_file(str(p))
    assert o == {"cam_xy": 0.6, "cam_th": pytest.approx(5.0 * D2R), "blink_xy": 0.4, "g_base": 0.004}
    n = Noise(**o)
    assert n.cam_xy == 0.6
    p.write_text(json.dumps({"noise": {"nonsense": 1}}))
    with pytest.raises(KeyError):
        load_noise_file(str(p))
