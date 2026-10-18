"""calib.py on a simulated calibration run of firmware 2026-10-18b (15 cm roll forward and
back, spin left and back): heading, wheel distance scale, gyro scale and the light's offset
from the turning centre come out close to the simulator's truth. One robot, one short run
after the calibration (tools/sim/simulate.py --video; about a minute)."""
import csv
import json
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TOOLS, "sim"))

import blink  # noqa: E402
import calib  # noqa: E402


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    import simulate
    out = str(tmp_path_factory.mktemp("calspin"))
    sheet = os.path.join(out, "sheet.csv")
    with open(sheet, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=simulate.SHEET_COLS)
        w.writeheader()
        w.writerow(dict(order=1, robot="Center", level="s10", commands="set straight_cm 10 / run straight"))
    cfgp = os.path.join(out, "cfg.json")
    with open(cfgp, "w") as f:
        json.dump(dict(camera=dict(hands_per_min=0.0, boxes_per_10min=0.0),
                       session=dict(after_run_s=2.0, before_run_s=2.0, carry_s=2.0, calib_wait_s=1.0,
                                    after_calib_s=1.0, video_lead_s=2.0, video_tail_s=2.0)), f)
    o = os.path.join(out, "o")
    simulate.main(["--out", o, "--sheet", sheet, "--seed", "21", "--video", "--config", cfgp])
    info = json.load(open(os.path.join(o, "sim_info.json")))
    c = info["config"]["camera"]
    off = c["pi_clock_offset_s"] + c["capture_delay_frames"] / c["fps"]
    rec = os.path.join(o, "recording")
    blink.main([rec, "--calib", os.path.join(o, "floor_calibration.json")])
    calib.main(["--session", os.path.join(o, "session"), "--recording", rec, "--calib",
                os.path.join(o, "floor_calibration.json"), "--detections", os.path.join(o, "camera_detections.csv"),
                "--video-offset", str(off)])
    with open(os.path.join(rec, "calibration_results.csv")) as f:
        rows = [r for r in csv.DictReader(f) if r["event"] == "calib"]
    truth = {}
    with open(os.path.join(o, "truth.csv")) as f:
        for r in csv.DictReader(f):
            truth.setdefault(r["run"], []).append((float(r["t_laptop"]), float(r["heading_deg"])))
    return info, rows, truth, off


def test_one_calib_row(sim):
    info, rows, truth, off = sim
    assert len(rows) == 1
    r = rows[0]
    assert r["identity"].startswith("agree")
    for k in ("heading_quality", "wheel_scale_quality", "gyro_scale_quality", "led_offset_quality"):
        assert r[k] == "ok", (k, r[k], r["note"])


def test_heading(sim):
    info, rows, truth, off = sim
    r = rows[0]
    t = float(r["t_video"]) - off
    ts = truth[r["run"]]
    true_h = [h for tt, h in ts if tt <= t - 0.05][-1]      # standing still before the roll
    err = (float(r["heading_floor_deg"]) - true_h + 180) % 360 - 180
    assert abs(err) < 0.5, (r["heading_floor_deg"], true_h)
    assert abs(float(r["heading_spread_deg"])) < 1.0


def test_wheel_scale(sim):
    info, rows, truth, off = sim
    rb = info["robots"]["Center"]
    # true distance per odometry cm: the true mean wheel radius over the nominal one, less a little slip
    expect = (rb["wheel_radius_left_cm"] + rb["wheel_radius_right_cm"]) / 2 / rb["nominal_radius_cm"]
    assert abs(float(rows[0]["wheel_scale"]) - expect) < 0.008, (rows[0]["wheel_scale"], expect)
    assert abs(float(rows[0]["moved_cm"]) - 15.0 * expect) < 0.3


def test_gyro_scale(sim):
    info, rows, truth, off = sim
    expect = 1 / (1 + info["robots"]["Center"]["gyro_scale_err"])
    assert abs(float(rows[0]["gyro_scale"]) - expect) < 0.002, (rows[0]["gyro_scale"], expect)


def test_led_offset_from_turning_centre(sim):
    info, rows, truth, off = sim
    m = info["config"]["robot"]
    fwd = m["length_cm"] / 2 - 2.0 + m["axle_to_center_cm"]     # tools/sim/camera.py draws the LEDs there
    r = rows[0]
    assert abs(float(r["spin_led_fwd_cm"]) - fwd) < 0.1, r["spin_led_fwd_cm"]
    assert abs(float(r["spin_led_left_cm"])) < 0.1, r["spin_led_left_cm"]
    assert abs(float(r["led_radius_cm"]) - math.hypot(fwd, 0)) < 0.1
