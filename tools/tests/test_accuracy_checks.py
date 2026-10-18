"""tools/accuracy_checks.py: the statistics helpers on made-up numbers, missing
inputs, the T0 camera path, and (about 20 s) a simulated T1 + E1b session with
a pretend camera made from the simulator's true poses. Skip the simulation
with ACCURACY_FAST=1."""
import csv
import json
import math
import os
import random
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest

from conftest import TOOLS

sys.path.insert(0, TOOLS)
import accuracy_checks as ac  # noqa: E402
import tape_results as tr  # noqa: E402

SHEET = os.path.join(TOOLS, "tests", "data", "e01_sheet_oct18.csv")


# ------------------------------------------------------------------ helpers
def test_bootstrap_interval_is_fixed_and_positive():
    rng = random.Random(4)
    errs = [abs(rng.gauss(0, 1)) ** 2 for _ in range(12)]       # skewed, never negative
    lo, hi = tr.bootstrap_ci(errs)
    assert (lo, hi) == tr.bootstrap_ci(errs)                    # same data, same interval
    assert 0 < lo < sum(errs) / len(errs) < hi
    assert all(math.isnan(v) for v in tr.bootstrap_ci([1.0]))


def test_split_is_ABBA_within_robot_and_route():
    rows = [dict(order=str(i), robot=rb, level=lv, run_id="")
            for i, (rb, lv) in enumerate([(rb, lv) for rb in ("Left", "Right") for lv in ("L", "square3") * 4], 1)]
    out = ac.split_rows(rows)
    for rb in ("Left", "Right"):
        for rt in ("L", "square"):
            halves = [d["half"] for d in out if d["robot"] == rb and d["route"] == rt]
            assert halves == ["tune", "score", "score", "tune"]
    random.Random(1).shuffle(rows)
    assert ac.split_rows(rows) == out                             # depends on the order column only


def test_within_group_slope_finds_a_trend_but_not_group_offsets():
    pts = [(g, x, 10 * gi + 0.5 * x + 0.01 * ((x * 7) % 3)) for gi, g in enumerate("abc") for x in range(6)]
    b, h, n = ac.within_group_slope(pts)
    assert abs(b - 0.5) < 0.01 and h < 0.05 and n == 18
    flat = [(g, x, 10 * gi + 0.1 * ((x * 7) % 3)) for gi, g in enumerate("abc") for x in range(6)]
    b, h, _ = ac.within_group_slope(flat)
    assert abs(b) <= h                                            # no trend: interval includes 0


def test_wilson_interval_with_no_errors():
    lo, hi = ac.wilson(0, 20)
    assert lo == 0 and 0.1 < hi < 0.2


def test_pair_low_full_matches_route_and_place_in_order():
    def exp(plan):
        rows = [dict(order=str(i), robot="Right", level=lv, run_id=f"r{i}") for i, lv in enumerate(plan, 1)]
        trials = [dict(r, pos_err=float(i)) for i, r in enumerate(rows, 1)]
        return SimpleNamespace(rows=rows, trials=trials, runs={})
    low = exp(["L", "straight100", "square3", "L"])
    full = exp(["square3", "L", "L", "straight100", "square3"])
    pairs = ac.pair_low_full(low, full)
    got = [(lt[0]["level"], lt[0]["order"], ft[0]["order"]) for lt, ft in pairs]
    assert sorted(got) == sorted([("L", "1", "2"), ("L", "4", "3"), ("straight100", "2", "4"), ("square3", "3", "1")])


def test_settled_shift_sees_a_bump_not_the_carry():
    rng = random.Random(2)

    def rows(track):
        return [dict(t=t / 25, x=x + rng.gauss(0, 0.2), y=y + rng.gauss(0, 0.2)) for t, (x, y) in enumerate(track)]
    still = [(50.0, 50.0)] * 75
    carry = [(50.0 + 0.6 * k, 50.0) for k in range(25)]            # picked up and moved away
    back = [(52.0, 50.3)] * 100                                    # put down for the next run
    assert ac.settled_shift(rows(still + carry + back), 0.0, next_start=(52.0, 50.0)) < 0.3
    bumped = still + [(50.9, 50.4)] * 100                          # knocked 1 cm while marking
    assert 0.8 < ac.settled_shift(rows(bumped), 0.0, next_start=(20.0, 20.0)) < 1.2


def test_turn_lag_finds_a_delay():
    rows, gz = [], []
    for k in range(150):
        ms = 1000 + k * 20
        w = 1.0 / (1 + math.exp(-(ms - 2000) / 40))               # turn starts at 2 s
        g = 1.0 / (1 + math.exp(-(ms - 2040) / 40))               # gyro 40 ms later
        rows.append(dict(ms=ms, rr=60 * w, lr=-60 * w))
        gz.append(90 * g)
    i0 = next(i for i, r in enumerate(rows) if r["ms"] >= 2000)
    assert abs(ac.turn_lag(rows, gz, i0) - 40) <= 4


# ------------------------------------------------------------------ missing inputs
def test_missing_inputs_say_not_available(tmp_path, capsys):
    t1 = tmp_path / "E01-baseline-dataset" / "data"
    t1.mkdir(parents=True)
    shutil.copy(SHEET, t1 / "measurements.csv")                   # planned sheet, nothing measured yet
    assert ac.main([str(t1.parent), str(tmp_path / "E01b-low-battery")]) == 0
    out = capsys.readouterr().out
    rows = list(csv.DictReader(open(t1 / "accuracy_checks.csv")))
    ids = {r["id"] for r in rows}
    assert ids == {str(i) for i in range(1, 15)}
    for r in rows:
        if r["id"] not in ("13", "14"):
            assert r["verdict"] == ac.NOTYET and r["result"].startswith("not available yet (needs"), r
    assert "not available yet" in out
    split = list(csv.DictReader(open(t1 / "split.csv")))
    assert len(split) == 36 and {d["half"] for d in split} == {"tune", "score"}
    noise = json.load(open(t1 / "measured_noise.json"))
    assert set(noise) >= {"cam_xy_cm", "cam_heading_deg", "blink_xy_cm", "start_nudge_heading_deg"}
    assert noise["cam_xy_cm"] is None


# ------------------------------------------------------------------ T0 placements seen by the camera
def test_t0_placements_give_camera_noise(tmp_path):
    t0 = tmp_path / "E00-measurement-check" / "data"
    rec = t0 / "video" / "T0"
    rec.mkdir(parents=True)
    shutil.copy(SHEET, t0 / "measurements.csv")
    s = 3.4                                                      # px per cm, image x = -y, image y = -x
    H = [[0, -s, 600], [-s, 0, 450], [0, 0, 1]]
    json.dump({"H_floor_to_px": H}, open(rec / "floor_calibration.json", "w"))
    rng = random.Random(3)
    spots = [(20 + 10 * i, 15 + 9 * i) for i in range(8)]
    pts, dets = [], []
    for k, (x, y) in enumerate(spots):
        frame = 100 + 50 * k
        pts.append(dict(id=k, kind="robot", u=0, v=0, x=x, y=y, z=4, frame=frame, spot=f"r{k}", group=f"spot P{k}",
                        enabled=True))
        for fr_ in range(frame - 3, frame + 4):
            xx, yy = x + 0.3 + rng.gauss(0, 0.5), y + rng.gauss(0, 0.5)           # 0.3 cm bias in x
            dets.append(dict(frame=fr_, t_wall=1000 + fr_ / 25, x_px=600 - s * yy, y_px=450 - s * xx, score=0.9))
    json.dump({"points": pts}, open(rec / "floor_points.json", "w"))
    with open(rec / "detections.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(dets[0]))
        w.writeheader()
        w.writerows(dets)
    assert ac.main([str(t0.parent), "--out", str(tmp_path)]) == 0
    rows = [r for r in csv.DictReader(open(tmp_path / "accuracy_checks.csv")) if r["id"] == "8"]
    assert rows[0]["subject"] == "detector position" and "8 still moments (8 from T0)" in rows[0]["result"]
    noise = json.load(open(tmp_path / "measured_noise.json"))
    assert abs(noise["_detail"]["cam_bias_cm"][0] - 0.3) < 0.25 and noise["cam_xy_cm"] < 0.5


# ------------------------------------------------------------------ simulated session
def _sheet(path, plan):
    rows = list(csv.DictReader(open(SHEET)))
    tmpl = {}
    for r in rows:
        tmpl.setdefault(r["level"], r)
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        for i, (robot, level) in enumerate(plan, 1):
            w.writerow(dict(tmpl[level], order=str(i), robot=robot, run_id=""))


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    if os.environ.get("ACCURACY_FAST") == "1":
        pytest.skip("ACCURACY_FAST=1")
    d = tmp_path_factory.mktemp("acc")
    plans = {"E01-baseline-dataset": ([(rb, lv) for rb, lvs in (("Left", ("straight100", "square3", "L", "straight100")),
                                                                ("Right", ("straight100", "square3", "L", "square3")))
                                       for lv in lvs], 7),
             "E01b-low-battery": ([("Right", lv) for lv in ("L", "straight100", "square3")], 8)}
    info = {}
    for name, (plan, seed) in plans.items():
        _sheet(d / f"{name}.csv", plan)
        out = d / f"sim_{name}"
        subprocess.run([sys.executable, os.path.join(TOOLS, "sim", "simulate.py"), "--out", str(out), "--seed",
                        str(seed), "--sheet", str(d / f"{name}.csv")], check=True, capture_output=True)
        data = d / name / "data"
        (data / "logs").mkdir(parents=True)
        shutil.copy(out / "measurements.csv", data / "measurements.csv")
        shutil.copytree(out / "session", data / "logs" / "s1")
        info[name] = json.load(open(out / "sim_info.json"))
        # a pretend camera: the true body centre 25 times a second, 0.3 cm noise, a fixed floor mapping
        # (no camera height, so no height correction), video clock = laptop clock + 0.1 s
        rec = data / "video" / "rec"
        rec.mkdir(parents=True)
        H = json.load(open(out / "floor_calibration.json"))["homography_floor_cm_to_px"]
        json.dump({"H_floor_to_px": H}, open(rec / "floor_calibration.json", "w"))
        rng = random.Random(seed)
        truth = list(csv.DictReader(open(out / "truth.csv")))
        bump = info[name]["runs"][0]["run"] if name.startswith("E01-") else None
        with open(rec / "detections.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["frame", "t_wall", "x_px", "y_px", "score"])
            for k, r in enumerate(truth[::2]):
                x, y = float(r["x_cm"]) + rng.gauss(0, 0.3), float(r["y_cm"]) + rng.gauss(0, 0.3)
                u, v, s = (H[i][0] * x + H[i][1] * y + H[i][2] for i in range(3))
                w.writerow([k, f"{float(r['t_laptop']) + 0.1:.4f}", f"{u / s:.2f}", f"{v / s:.2f}", "0.9"])
            # after the first run ends, the robot sits still, then gets knocked 1.5 cm while being marked
            if bump:
                run, nxt = info[name]["runs"][0], info[name]["runs"][1]
                ex, ey = run["end"][0], run["end"][1]
                dx, dy = ex - nxt["start"][0], ey - nxt["start"][1]       # knocked away from the next start
                k = 1.5 / (math.hypot(dx, dy) or 1)
                for j in range(int(25 * 6)):
                    t = run["t_end"] + j / 25
                    x, y = (ex, ey) if j < 75 else (ex + k * dx, ey + k * dy)
                    u, v, s = (H[i][0] * x + H[i][1] * y + H[i][2] for i in range(3))
                    w.writerow([100000 + j, f"{t + 0.1:.4f}", f"{u / s:.2f}", f"{v / s:.2f}", "0.9"])
    return d, info


def test_simulated_session(sim, capsys):
    d, info = sim
    t1 = d / "E01-baseline-dataset"
    assert ac.main([str(t1), str(d / "E01b-low-battery"), "--video-offset", "0.1"]) == 0
    rows = list(csv.DictReader(open(t1 / "data" / "accuracy_checks.csv")))
    by = {(r["id"], r["subject"]): r for r in rows}
    robots = info["E01-baseline-dataset"]["robots"]
    # 2: tape / wheels - 1 ~ true wheel size error minus the simulator's 0.2% rolling slip (tape noise ~0.3%)
    for rb in ("Left", "Right"):
        r = robots[rb]
        true = ((r["wheel_radius_left_cm"] + r["wheel_radius_right_cm"]) / 2 / r["nominal_radius_cm"] - 1) * 100 - 0.2
        assert abs(float(by[("2", rb)]["value"]) - true) < 0.8, (rb, by[("2", rb)]["result"], true)
        # 4: the gyro's scale error (the square's 1080 deg; tape heading noise ~1 deg = 0.1%)
        g = float(by[("4", rb)]["value"])
        assert abs(g - r["gyro_scale_err"] * 100) < 0.4, (rb, by[("4", rb)]["result"], r["gyro_scale_err"])
        assert by[("7", rb)]["verdict"] == ac.PASS                    # the simulator has no gyro delay
    # 6: the simulated wheels scrub ~3% in turns, the gyro doesn't: trust the gyro
    assert "trust the gyro" in by[("6", "all robots")]["result"]
    # 8: camera noise ~0.3 cm per sighting, a median of many sightings per still moment
    cam = by[("8", "detector position")]
    assert cam["verdict"] == ac.PASS and float(cam["value"]) < 0.5
    noise = json.load(open(t1 / "data" / "measured_noise.json"))
    assert noise["cam_xy_cm"] == pytest.approx(float(cam["value"]), abs=1e-3)
    # 9: the knocked run is found
    first = info["E01-baseline-dataset"]["runs"][0]["run"]
    assert by[("9", "all runs")]["verdict"] == ac.CHECK and first in by[("9", "all runs")]["result"]
    # 5 and 13: E1b paired with Right's T1 runs (L, straight and square each matched once)
    assert "3 route-matched pairs" in by[("13", "E1b vs T1 (Right)")]["result"]
    assert by[("13", "held-out split")]["verdict"] in (ac.PASS, ac.CHECK)
    assert (t1 / "data" / "split.csv").exists()
    for cid in ("1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "12", "13", "14"):
        assert any(r["id"] == cid and r["verdict"] != ac.NOTYET for r in rows), cid
    assert by[("11", "all runs")]["verdict"] == ac.NOTYET          # no calibration_results.csv here


# ------------------------------------------------------------------ simulator preset
def test_real_camera_preset_loads(tmp_path):
    sys.path.insert(0, os.path.join(TOOLS, "sim"))
    import config
    cfg = config.load_config(ac.SIM_PRESET)
    c = cfg.camera
    assert (c.width, c.height, c.fps, c.height_cm) == (640, 480, 25.0, 262.0)
    assert abs(c.width / c.floor_width_cm - 3.4) < 0.05
    out = tmp_path / "full.json"
    subprocess.run([sys.executable, os.path.join(TOOLS, "sim", "simulate.py"), "--config", ac.SIM_PRESET,
                    "--write-config", str(out)], check=True, capture_output=True)
    assert json.load(open(out))["camera"]["fps"] == 25.0


def test_identity_counts_no_room_calibrations_separately():
    """A calibration that skipped its roll for lack of room has no roll to name: it is its own state
    ("limited (no room)"), not an unreadable run, so it can't turn the check into CHECK."""
    def row(identity, outcome=""):
        return dict(identity=identity, outcome=outcome)
    cam = SimpleNamespace(calib_rows=[row("agree"), row("agree"), row("agree"),
                                      row("blink code only (no roll)", "limited (no room)"),
                                      row("blink code only (no roll)", "limited (no room)")])
    tb = ac.Table()
    ac.check_identity(tb, cam)
    r = [x for x in tb.rows if x["id"] == "11"][-1]
    assert r["verdict"] == ac.PASS, r
    assert "0 not readable" in r["result"] and "2 calibration(s) limited" in r["result"]
    # the same rows without the outcome column (older calib.py output) still count as not readable
    cam.calib_rows = [row("agree"), row("agree"), row("agree"), row("blink code only (no roll)"),
                      row("blink code only (no roll)")]
    tb = ac.Table()
    ac.check_identity(tb, cam)
    assert [x for x in tb.rows if x["id"] == "11"][-1]["verdict"] == ac.CHECK


def test_run_sheet_reads_the_calib_look_skip_columns():
    import run_sheet as rs
    import session_checks as sc
    w = rs.RunWatcher()
    vals = "AF76-1,0,7.2,0.0,0.0,2.1,-1.0,0.0,0.0,no_room,2.1;-1.0;-1.0;-1.0;-1.0;-1.0;-1.0;-1.0,,skipped,skipped,no room"
    w("Left", "D", "CALIB_LOOK", vals.split(","))
    look = w.looks["AF76-1"]
    assert (look["spin"], look["roll_step"], look["reason"]) == ("skipped", "skipped", "no room")
    assert sc.calib_outcome("done", look) == sc.CALIB_LIMITED
    assert sc.calib_outcome_text("done", look) == "limited (no room)"
    assert sc.calib_outcome("failed:spin timed out", look) == sc.CALIB_FAILED


def test_limited_runs_are_left_out_and_counted(tmp_path):
    """Runs the edge guard stopped before an open plywood edge ("limited: plywood edge", set by run_sheet.py):
    left out of every statistic, counted apart; one whose log says limited but whose row wasn't marked too."""
    import session_checks as sc
    assert sc.run_outcome("done") == sc.CALIB_DONE
    assert sc.run_outcome("limited:plywood edge") == sc.CALIB_LIMITED
    assert sc.run_limit_reason("limited:plywood edge") == "plywood edge"
    assert sc.run_outcome("stopped:cancel button") == sc.CALIB_FAILED and sc.run_outcome(None) == sc.CALIB_FAILED
    assert sc.calib_outcome_text("limited:plywood edge", None) == "limited (plywood edge)"
    d = tmp_path / "E01" / "data"
    d.mkdir(parents=True)
    cols = ["order", "robot", "level", "run_id", "target_x_cm", "target_y_cm", "target_heading_deg",
            "xf_cm", "yf_cm", "xr_cm", "yr_cm", "excluded", "note"]
    marks = dict(xf_cm="34.75", yf_cm="0", xr_cm="25.25", yr_cm="0")
    rows = [dict(order=1, robot="Left", level="straight30", run_id="A-1", target_x_cm=30, target_y_cm=0,
                 target_heading_deg=0, **marks),
            dict(order=2, robot="Left", level="straight30", run_id="A-2", excluded="1", note="limited: plywood edge"),
            dict(order=3, robot="Left", level="straight30", run_id="A-3", target_x_cm=30, target_y_cm=0,
                 target_heading_deg=0, note="limited: plywood edge (measured anyway)", **marks),
            dict(order=4, robot="Left", level="straight30", run_id="A-4", excluded="1", note="slipped")]
    with open(d / "measurements.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    trials = tr.load_trials(str(tmp_path / "E01"))
    assert [t["order"] for t in trials] == ["1"]
    assert [r["order"] for r in tr.limited_trials(str(tmp_path / "E01"))] == ["2", "3"]
    e = ac.Experiment(str(tmp_path / "E01"), "t1")
    assert len(e.trials) == 1 and len(e.limited) == 2
    # a measured, unmarked row whose RUNEND says limited is left out too
    e.runs["A-1"] = SimpleNamespace(result="limited:plywood edge")
    e.drop_limited()
    assert e.trials == [] and len(e.limited) == 3


def test_run_sheet_adds_a_replacement_for_a_limited_run():
    import run_sheet as rs
    rows = [dict(order="1", robot="Left", level="L", commands="run demo", run_id="A-1", xf_cm="1", note="x"),
            dict(order="2", robot="Left", level="sq", commands="run demo", run_id="", note=""),
            dict(order="3", robot="Right", level="L", commands="run demo", run_id="", note=""),
            dict(order="4", robot="Left", level="L", commands="run demo", run_id="", note="")]
    new = rs.add_replacement(rows, 0)
    assert [r["order"] for r in rows] == ["1", "2", "5", "3", "4"]       # end of Left's first block
    assert new["robot"] == "Left" and new["level"] == "L" and new["commands"] == "run demo"
    assert new["run_id"] == "" and new["xf_cm"] == ""
    assert new["note"] == "replaces order 1 (limited: plywood edge)"
    assert not tr.is_limited(new) and tr.is_limited(dict(note="limited: plywood edge"))
    assert tr.is_limited(dict(note="replaces order 1 (limited: plywood edge) limited: plywood edge"))
