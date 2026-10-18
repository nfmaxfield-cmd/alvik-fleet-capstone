"""calib.py on simulated calibration runs of the current firmware (2026-10-18c: blink, wiggle,
spin left and back while looking around, turn to the roll heading, 15 cm roll forward and back):
heading, wheel distance scale, gyro scale and the light's offset from the turning centre come
out close to the simulator's truth. One robot, one short run after the calibration
(tools/sim/simulate.py --video; about a minute each). Three placements: open floor (rolls
straight ahead), a laptop cap straight ahead (it turns about 180 deg before rolling), and boxed
in by boards (no roll: spin results only)."""
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


BOX = (35.0, 30.0, 13.0)        # boards' centre lines 13 cm from (35, 30) on four sides: no room to roll


def run_sim(tmp_path_factory, name, seed, session=None, boxed=False):
    import simulate
    out = str(tmp_path_factory.mktemp(name))
    sheet = os.path.join(out, "sheet.csv")
    with open(sheet, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=simulate.SHEET_COLS)
        w.writeheader()
        w.writerow(dict(order=1, robot="Center", level="s10", commands="set straight_cm 10 / run straight"))
    cfgp = os.path.join(out, "cfg.json")
    sess = dict(after_run_s=2.0, before_run_s=2.0, carry_s=2.0, calib_wait_s=1.0,
                after_calib_s=1.0, video_lead_s=2.0, video_tail_s=2.0)
    sess.update(session or {})
    with open(cfgp, "w") as f:
        json.dump(dict(camera=dict(hands_per_min=0.0, boxes_per_10min=0.0), session=sess), f)
    extra = []
    if boxed:
        cx, cy, h = BOX
        m = dict(format="alvik-fleet-map", version=1, floor=dict(x_min=0, x_max=122, y_min=0, y_max=122),
                 boards=[dict(name=n, p1=a, p2=b, thickness_cm=1.5) for n, a, b in (
                     ("N", [cx - 15, cy + h], [cx + 15, cy + h]), ("S", [cx - 15, cy - h], [cx + 15, cy - h]),
                     ("E", [cx + h, cy - 15], [cx + h, cy + 15]), ("W", [cx - h, cy - 15], [cx - h, cy + 15]))])
        mp = os.path.join(out, "map.json")
        with open(mp, "w") as f:
            json.dump(m, f)
        extra = ["--map", mp]
    o = os.path.join(out, "o")
    simulate.main(["--out", o, "--sheet", sheet, "--seed", str(seed), "--video", "--config", cfgp] + extra)
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


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    return run_sim(tmp_path_factory, "calspin", 21)


@pytest.fixture(scope="module")
def turned(tmp_path_factory):
    # the laptop caps the roll straight ahead to 2 cm: it rolls the other way
    return run_sim(tmp_path_factory, "calturn", 22, session=dict(calib_commands=["set calib_dir_limits 0 2"]))


@pytest.fixture(scope="module")
def boxed(tmp_path_factory):
    cx, cy, _ = BOX
    return run_sim(tmp_path_factory, "calbox", 23, session=dict(calib_area_cm=[cx, cx, cy, cy]), boxed=True)


def test_one_calib_row(sim):
    info, rows, truth, off = sim
    assert len(rows) == 1
    r = rows[0]
    assert r["identity"].startswith("agree")
    for k in ("heading_quality", "wheel_scale_quality", "gyro_scale_quality", "led_offset_quality"):
        assert r[k] == "ok", (k, r[k], r["note"])
    assert r["look_result"] == "roll" and float(r["look_heading_deg"]) == 0.0
    assert float(r["commanded_cm"]) == 15.0
    assert (r["look_spin"], r["look_roll"], r["look_reason"], r["outcome"]) == ("done", "full", "", "done")


def check_accuracy(case, gyro_tol=0.002):
    """Heading, wheel scale, gyro scale and light offset within the 2026-10-18b run's test limits."""
    info, rows, truth, off = case
    assert len(rows) == 1
    r = rows[0]
    t = float(r["t_video"]) - off
    true_h = [h for tt, h in truth[r["run"]] if tt <= t - 0.05][-1]
    assert abs((float(r["heading_floor_deg"]) - true_h + 180) % 360 - 180) < 0.5
    rb = info["robots"]["Center"]
    expect = (rb["wheel_radius_left_cm"] + rb["wheel_radius_right_cm"]) / 2 / rb["nominal_radius_cm"]
    assert abs(float(r["wheel_scale"]) - expect) < 0.008, (r["wheel_scale"], expect)
    assert abs(float(r["gyro_scale"]) - 1 / (1 + rb["gyro_scale_err"])) < gyro_tol, r["gyro_scale"]
    m = info["config"]["robot"]
    fwd = m["length_cm"] / 2 - 2.0 + m["axle_to_center_cm"]
    assert abs(float(r["spin_led_fwd_cm"]) - fwd) < 0.1 and abs(float(r["spin_led_left_cm"])) < 0.1


def test_turned_before_the_roll(turned):
    info, rows, truth, off = turned
    r = rows[0]
    look = info["calib_runs"][0]["look"]
    assert look["result"] == "roll" and abs(abs(look["heading_deg"]) - 180) < 3, look
    assert r["look_result"] == "roll" and abs(abs(float(r["look_heading_deg"])) - 180) < 3
    assert r["identity"].startswith("agree")
    for k in ("heading_quality", "wheel_scale_quality", "gyro_scale_quality", "led_offset_quality"):
        assert r[k] == "ok", (k, r[k], r["note"])
    check_accuracy(turned)


def test_boxed_in_no_roll_spin_results_only(boxed):
    info, rows, truth, off = boxed
    assert info["calib_runs"][0]["result"] == "done"
    assert info["calib_runs"][0]["look"]["result"] == "no_room"
    assert len(rows) == 1
    r = rows[0]
    assert r["look_result"] == "no_room" and float(r["commanded_cm"]) == 0.0
    assert r["identity"] == "blink code only (no roll)" and r["robot_blink"] == "Center"
    assert "no roll" in r["note"]
    assert r["heading_floor_deg"] == "" and r["wheel_scale"] == ""
    rb = info["robots"]["Center"]
    # no empty-floor picture (no roll away): the spin pictures are aligned inside a disc, less precise
    assert abs(float(r["gyro_scale"]) - 1 / (1 + rb["gyro_scale_err"])) < 0.004, r["gyro_scale"]
    m = info["config"]["robot"]
    assert abs(float(r["led_radius_cm"]) - (m["length_cm"] / 2 - 2.0 + m["axle_to_center_cm"])) < 0.1
    # not measured for lack of room: neither ok nor a check, and the run is "limited (no room)", not failed
    assert r["led_offset_quality"].startswith("skipped: no room")
    assert r["heading_quality"] == r["wheel_scale_quality"] == "skipped: no room"
    assert r["gyro_scale_quality"] == "ok", r["gyro_scale_quality"]
    assert (r["look_spin"], r["look_roll"], r["look_reason"]) == ("done", "skipped", "no room")
    assert r["outcome"] == "limited (no room)"


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
