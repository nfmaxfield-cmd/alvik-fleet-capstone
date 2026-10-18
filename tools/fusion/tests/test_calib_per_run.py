"""Which calibration scores which run (circularity), and the Holm adjustment in replay.py."""
import json
import math
import os
import subprocess
import sys

import pytest

import fusion_synth  # noqa: F401  (puts tools/fusion and tools on the path)
import runs as rn
import replay

H = [[3.0, 0.0, 100.0], [0.0, -3.0, 400.0], [0.0, 0.0, 1.0]]


def calib(path, used_runs=(), mode="points", used_robots=None):
    fit = dict(mode=mode, used_runs=list(used_runs))
    if used_robots is not None:
        fit["used_robots"] = list(used_robots)
    path.write_text(json.dumps(dict(H_floor_to_px=H, fit=fit)))
    return str(path)


def loro_csv(tmp_path, rows, bom=False):
    p = tmp_path / "floor_calibration_loro.csv"
    text = "run_id,calibration,left_out,n_used,fit_rms_cm,note\n" + "".join(
        f"{r},{c},{r},10,0.5,\n" for r, c in rows)
    p.write_bytes((b"\xef\xbb\xbf" if bom else b"") + text.encode())
    return str(p)


def test_per_run_files_used_and_no_silent_fallback(tmp_path):
    main = calib(tmp_path / "floor_calibration.json", ["R1", "R2", "R3"], "survey")
    (tmp_path / "floor_calibration_loro").mkdir()
    calib(tmp_path / "floor_calibration_loro" / "R1.json", ["R2", "R3"], "survey")
    calib(tmp_path / "floor_calibration_loro" / "R2.json", ["R1", "R3"], "survey")
    per = loro_csv(tmp_path, [("R1", "floor_calibration_loro/R1.json"), ("R2", "floor_calibration_loro/R2.json"),
                              ("R3", "")], bom=True)                  # R3: no fit without it; R4: not listed
    assign, fell = rn.assign_calibrations(["R1", "R2", "R3", "R4"], main, per)
    assert assign["R1"].endswith(os.path.join("floor_calibration_loro", "R1.json"))
    assert assign["R3"] == os.path.abspath(main) and assign["R4"] == os.path.abspath(main)
    assert set(fell) == {"R3", "R4"} and "no calibration" in fell["R3"] and fell["R4"] == "not listed"
    infos = {}
    bad = rn.in_sample(assign, infos=infos)
    assert bad == [("R3", os.path.abspath(main))]                     # R3 falls back to a file made with it
    assert infos[assign["R1"]]["used_runs"] == {"R2", "R3"}


def test_calib_alone_is_in_sample(tmp_path):
    main = calib(tmp_path / "c.json", ["R1", "R2"])
    assign, _ = rn.assign_calibrations(["R1", "R2", "R9"], main)
    assert [r for r, _ in rn.in_sample(assign)] == ["R1", "R2"]
    clicks = calib(tmp_path / "clicks.json", [])                       # click calibration: no runs used
    assert rn.in_sample(rn.assign_calibrations(["R1"], clicks)[0]) == []


def test_same_robot_survey_note(tmp_path):
    robot_of = {"L1": "Left", "L2": "Left", "C1": "Center"}
    run_out = calib(tmp_path / "L1.json", ["L2", "C1"], "survey", ["Left", "Center"])      # leave one RUN out
    robot_out = calib(tmp_path / "Left.json", ["C1"], "survey", ["Center"])                 # leave one ROBOT out
    old = calib(tmp_path / "old.json", ["L2", "C1"], "survey")                              # no used_robots: derived
    pts = calib(tmp_path / "pts.json", ["L2", "C1"], "points")                              # tape points: no odometry
    infos = {}
    for p in (run_out, robot_out, old, pts):
        infos[os.path.abspath(p)] = rn.calib_info(p)
    a = lambda p: {"L1": os.path.abspath(p)}
    assert rn.same_robot_survey(a(run_out), robot_of, infos) == [("L1", "Left", os.path.abspath(run_out))]
    assert rn.same_robot_survey(a(robot_out), robot_of, infos) == []
    assert rn.same_robot_survey(a(old), robot_of, infos) and not rn.same_robot_survey(a(pts), robot_of, infos)


def test_holm():
    p = [0.01, float("nan"), 0.04, 0.03, 0.2]
    h = replay.holm(p)
    # sorted 0.01, 0.03, 0.04, 0.2 (m = 4): 0.04, 0.09, 0.09 (kept non-decreasing: max(0.09, 0.08)), 0.2
    assert math.isnan(h[1])
    assert [round(h[i], 6) for i in (0, 3, 2, 4)] == [0.04, 0.09, 0.09, 0.2]
    assert replay.holm([0.6, 0.5]) == [1.0, 1.0]
    assert all(hh >= pp for hh, pp in zip(h, p) if not math.isnan(pp))


def test_runs_py_refuses_in_sample(tmp_path):
    """runs.py --calib-per-run used to fall back silently to --calib for unlisted runs."""
    sess = tmp_path / "session"
    sess.mkdir()
    (sess / "runinfo.csv").write_text("robot,t_recv,run,experiment\nLeft,1000,R1,E1\n")
    (sess / "raw.csv").write_text("robot,t_recv,run,ms,step,v_cms\n" + "".join(
        f"Left,{1000 + i / 10},R1,{i * 100},0,0\n" for i in range(20)))
    det = tmp_path / "det.csv"
    det.write_text("frame,t_wall,x_px,y_px,score\n1,1000.5,150,350,0.9\n")
    main = calib(tmp_path / "floor_calibration.json", ["R1"], "survey")
    per = loro_csv(tmp_path, [("R2", "x.json")])
    tool = os.path.join(os.path.dirname(rn.__file__), "runs.py")
    cmd = [sys.executable, tool, "--session", str(sess), "--detections", str(det), "--calib", main,
           "--video-offset", "0", "--out", str(tmp_path / "out")]
    r = subprocess.run(cmd + ["--calib-per-run", per], capture_output=True, text=True)
    assert r.returncode != 0 and "Refusing" in r.stderr and "R1" in r.stderr
    r = subprocess.run(cmd + ["--allow-in-sample"], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert "WARNING" in r.stdout
    meta = json.load(open(tmp_path / "out" / "R1" / "meta.json"))
    assert meta["calib_file"] == os.path.abspath(main) and meta["calib_includes_run"] is True
