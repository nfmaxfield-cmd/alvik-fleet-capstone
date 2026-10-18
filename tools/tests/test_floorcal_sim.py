"""End to end on the simulator (about a minute): the camera is moved to a very
different pose (higher, turned 30 deg, tilted 10 deg, lens distortion; still
seeing all of the E01 sheet's start spots and the 122 cm board) and
survey mode recovers the mapping from the robot's own logs, the camera track
and the tape start poses only; still-point mode + simulated axis clicks too.
Skip with FLOORCAL_FAST=1."""
import json
import os
import subprocess
import sys

import numpy as np
import pytest

import calibrate_floor as cf
import floorcal_runs as fr
from conftest import TOOLS, floor_error
from floorcal_sim_check import TrueCam, click_points

pytestmark = pytest.mark.skipif(os.environ.get("FLOORCAL_FAST") == "1", reason="FLOORCAL_FAST=1")


@pytest.fixture(scope="module")
def moved_sim(tmp_path_factory):
    d = tmp_path_factory.mktemp("sim")
    cfg = d / "cfg.json"
    subprocess.run([sys.executable, os.path.join(TOOLS, "sim", "simulate.py"), "--write-config", str(cfg)],
                   check=True, capture_output=True)
    c = json.loads(cfg.read_text())
    c["camera"].update(height_cm=140.0, floor_width_cm=230.0, center_floor_cm=[60.0, 40.0], yaw_deg=30.0,
                       tilt_deg=[10.0, -6.0], k1=-0.04)
    cfg.write_text(json.dumps(c))
    out = d / "out"
    subprocess.run([sys.executable, os.path.join(TOOLS, "sim", "simulate.py"), "--out", str(out), "--video",
                    "--seed", "5", "--max-runs", "4", "--fps", "10", "--config", str(cfg),
                    # a frozen copy of the T1 sheet: editing the real sheet mustn't change this test's data
                    "--sheet", os.path.join(TOOLS, "tests", "data", "e01_sheet_oct18.csv")], check=True,
                   capture_output=True)
    # video clock offset ~ Pi clock offset + one frame of delivery; survey refines it (prior 0.1 s)
    return out, c["camera"]["pi_clock_offset_s"] + 1 / 10


def test_survey_recovers_moved_camera(moved_sim):
    sim, off = moved_sim
    cam = TrueCam(str(sim / "floor_calibration.json"))
    runs, _ = fr.rn.load_runs(str(sim / "session"), str(sim / "measurements.csv"))
    pairs = fr.survey_pairs(runs, fr.read_detections(str(sim / "camera_detections.csv")), off, fr.odometry_paths(runs))
    assert len(pairs) > 300
    res = fr.fit_survey(pairs, (cam.W, cam.H), height_cm=140.0, k1_mode="off")
    cal = cf.to_calibration(res)
    e = floor_error(cal, cam, z=4.0)
    assert e.mean() < 1.0 and np.percentile(e, 95) < 2.5, (e.mean(), np.percentile(e, 95))
    assert cal["view_floor_polygon_cm"] and cal["camera"]["height_cm"] == 140.0
    # without the tape height but with the webcam's focal length (kept from an earlier calibration)
    res2 = fr.fit_survey(pairs, (cam.W, cam.H), f_px=cam.f, k1_mode="off")
    assert abs(res2["height_cm"] - 140.0) < 6
    assert floor_error(cf.to_calibration(res2), cam, z=4.0).mean() < 1.2


def test_cli_survey_and_loro(moved_sim, tmp_path):
    sim, off = moved_sim
    out = tmp_path / "floor_calibration.json"
    r = subprocess.run([sys.executable, os.path.join(TOOLS, "calibrate_floor.py"), str(sim / "recording"), "--survey",
                        "--session", str(sim / "session"), "--sheet", str(sim / "measurements.csv"), "--detections",
                        str(sim / "camera_detections.csv"), "--video-offset", str(off), "--height-cm", "140",
                        "--k1", "off", "--out", str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    cal = json.loads(out.read_text())
    assert cal["fit"]["mode"] == "survey" and len(cal["fit"]["used_runs"]) >= 3
    import csv
    rows = list(csv.DictReader(open(tmp_path / "floor_calibration_loro.csv")))
    assert {r["run_id"] for r in rows} == set(cal["fit"]["used_runs"])
    with_end = {p["run"] for p in csv.DictReader(open(tmp_path / "survey_pairs.csv")) if p["kind"] == "tape_end"}
    for row in rows:
        held = json.loads((tmp_path / row["calibration"]).read_text())
        assert row["run_id"] not in held["fit"]["used_runs"]
        assert float(row["heldout_path_median_cm"]) < 3.0
        # its own end mark, not used for its file (only 4 runs here: with the one straight run held out, its
        # end at x = 110 cm is extrapolated, so this is loose). A run whose end mark the camera didn't see
        # gets a blank cell, not a number
        if row["run_id"] in with_end:
            assert float(row["heldout_end_mark_err_cm"]) < 5.0, row
        else:
            assert row["heldout_end_mark_err_cm"] == "", row
    # every run's end spot is in this camera's view, so every run must have its end mark
    assert with_end == set(cal["fit"]["used_runs"])
    assert cal["fit"]["survey"]["end_marks_used"] >= 3


def test_survey_end_marks_are_absolute_and_help(moved_sim):
    sim, off = moved_sim
    cam = TrueCam(str(sim / "floor_calibration.json"))
    runs, _ = fr.rn.load_runs(str(sim / "session"), str(sim / "measurements.csv"))
    dets = fr.read_detections(str(sim / "camera_detections.csv"))
    pairs = fr.survey_pairs(runs, dets, off, fr.odometry_paths(runs))
    ends, _ = fr.end_mark_pairs(runs, dets, off)
    assert len(ends) >= 3 and all(p["kind"] == "tape_end" for p in ends)
    e0 = floor_error(cf.to_calibration(fr.fit_survey(pairs, (cam.W, cam.H), 140.0, k1_mode="off")), cam, z=4.0)
    e1 = floor_error(cf.to_calibration(fr.fit_survey(pairs + ends, (cam.W, cam.H), 140.0, k1_mode="off")), cam, z=4.0)
    # with 6 runs end marks took the mean from 0.57 to 0.41 cm (p95 1.58 -> 1.02); with these 4 runs they are
    # about even (0.44 vs 0.55), so only check they don't hurt much
    assert e1.mean() < 1.0 and e1.mean() <= e0.mean() + 0.2, (e0.mean(), e1.mean())


def test_still_points_with_axis_clicks(moved_sim):
    sim, off = moved_sim
    cam = TrueCam(str(sim / "floor_calibration.json"))
    runs, _ = fr.rn.load_runs(str(sim / "session"), str(sim / "measurements.csv"))
    pts, notes = fr.still_points(runs, fr.read_detections(str(sim / "camera_detections.csv")), off)
    assert len(pts) >= 6
    for p in pts:   # every still point is where the true camera sees that tape pose (tape + detector noise)
        u, v = cam.project(p["x"], p["y"], 4.0)
        assert np.hypot(u - p["u"], v - p["v"]) < 6, p
    # E01 runs start on 3 spots (square, straight, L), so the still points alone are on few separate spots
    # (5 here): no RANSAC (it would throw out a good spot), a warning, a rough but sane fit
    alone = cf.fit(pts, (cam.W, cam.H), 140.0, None, "off")
    assert alone["n_inliers"] == len(pts)
    assert any("separate spots" in w for w in alone["warnings"]), alone["warnings"]
    assert floor_error(cf.to_calibration(alone), cam, z=4.0).mean() < 3.0
    with pytest.raises(cf.FitError):          # the square runs alone (one start spot, start = end): a point
        cf.fit([p for p in pts if p["x"] < 50 and p["y"] < 50], (cam.W, cam.H), 140.0, None, "off")
    axes = [(x, 0.0) for x in range(-10, 141, 10)] + [(0.0, y) for y in range(-40, 61, 10) if y]
    res = cf.fit(pts + click_points(cam, axes), (cam.W, cam.H), 140.0, None, "auto")
    assert floor_error(cf.to_calibration(res), cam, z=4.0).mean() < 0.6
