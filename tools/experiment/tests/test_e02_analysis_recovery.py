"""tools/experiment/e02_analysis.py setup_recovery() on a tiny made-up session folder: the front sensor against the map
(front_checks.csv) and the recovery counts (e02_state.json auto_retries / help_calls / disagreements, video marks),
plus an older session without any of these records."""
import csv
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
TOOLS = os.path.dirname(EXP)
for p in (EXP, os.path.join(TOOLS, "recognition"), TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import e02_analysis as ea  # noqa: E402

FRONT_COLS = ["t", "robot", "leg", "value", "x_cm", "y_cm", "heading_deg", "heading_measured", "front_cm",
              "expected_cm", "what"]
USED_DIFFS = [1, 2, 3, 2, 2, 12, 2, 3]       # median 2, MAD 0.5 -> spread 0.7413, 7 of 8 within 8 cm


def _session(tmp_path, state=None, front=True):
    s = tmp_path / "e02-20261022-1400"
    s.mkdir()
    if front:
        with open(s / "front_checks.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(FRONT_COLS)
            for i, d in enumerate(USED_DIFFS):
                w.writerow([100 + i, "R1", "drive", 20, 30, 40, 0, 1, 30 + d, 30, "x-board"])
            w.writerow([200, "R1", "turn", 90, 30, 40, 90, 0, 45, 30, "x-board"])      # heading not measured
            w.writerow([201, "R2", "drive", 20, 30, 40, 0, 1, 70, 60, "y-board"])      # map distance over 50 cm
            w.writerow([202, "R2", "drive", 20, 30, 40, 0, 1, 25, "", ""])            # nothing on the map ahead
    if state is not None:
        (s / "e02_state.json").write_text(json.dumps(state))
    return str(s)


STATE = dict(
    auto_retries=[[1000.0, "R1", "lost"], [1030.0, "R1", "lost"],      # one episode, then Nick was called
                  [2000.0, "R1", "stuck"],                              # back on its own: avoided
                  [3000.0, "R2", "lost"]],                              # help 200 s later: outside 2 min, avoided
    help_calls=[[1080.0, "R1", "lost"], [3200.0, "R2", "off the board"]],
    disagreements=[[1500.0, "R2", "camera and wheels", "3"]])

EVENTS = [dict(t_wall=t, text=x) for t, x in [
    (999.0, "e02 auto-retry R1 1: lost"),                 # state has the retries: marks not counted twice
    (1010.0, "e02 sweep R2 for R1"),
    (1020.0, "e02 rescue R1 find start"), (1025.0, "e02 rescue R1 find end failed"),
    (1026.0, "e02 rescue R1 find start"), (1031.0, "e02 rescue R1 find end ok"),
    (2500.0, "e02 drive out R2"),
    (3300.0, "e02 help timeout R2"),                      # not a help call of its own
    (4000.0, "e02 7/20 R1 C r2 find start")]]


def _csv(p):
    with open(p, newline="") as fh:
        return list(csv.DictReader(fh))


def test_front_sensor_numbers_and_suggested_values(tmp_path):
    sdir = _session(tmp_path, STATE)
    out = str(tmp_path / "analysis")
    res = ea.setup_recovery(sdir, out, EVENTS)
    fs = res["front"]
    assert fs["n_rows"] == 11 and fs["n"] == 8
    assert fs["median"] == pytest.approx(2.0)
    assert fs["spread"] == pytest.approx(0.5 * 1.4826)
    assert fs["within"] == pytest.approx(7 / 8)
    assert fs["offset"] == 2.0 and fs["tol"] == 3.0            # 3 x 0.74 is under the 3 cm floor
    rows = _csv(os.path.join(out, "front_sensor.csv"))
    assert len(rows) == 11 and sum(int(r["used"]) for r in rows) == 8
    assert rows[5]["diff_cm"] == "12.0" and rows[-1]["diff_cm"] == ""
    text = "\n".join(res["lines"])
    assert "FRONT_OFFSET_CM = 2.0" in text and "FRONT_TOL_CM = 3.0" in text
    assert any("FRONT_OFFSET_CM = 2.0" in n for n in res["next"])


def test_too_few_readings_give_no_suggestion(tmp_path):
    sdir = _session(tmp_path, STATE)
    rows = _csv(os.path.join(sdir, "front_checks.csv"))[:3]
    with open(os.path.join(sdir, "front_checks.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, FRONT_COLS)
        w.writeheader()
        w.writerows(rows)
    res = ea.setup_recovery(sdir, str(tmp_path / "analysis"), EVENTS)
    assert res["front"]["n"] == 3 and res["front"]["offset"] is None
    assert "Too few readings" in "\n".join(res["lines"]) and not res["next"]


def test_recovery_counts_per_robot(tmp_path):
    sdir = _session(tmp_path, STATE)
    out = str(tmp_path / "analysis")
    res = ea.setup_recovery(sdir, out, EVENTS)
    by = {r["robot"]: r for r in _csv(os.path.join(out, "recovery.csv"))}
    assert set(by) == {"R1", "R2", "all"}
    want = dict(R1=dict(auto_retries=3, retry_episodes=2, help_calls_avoided=1, sweeps=1, rescue_finds=2, drive_outs=0,
                        help_calls=1, disagreements=0),
                R2=dict(auto_retries=1, retry_episodes=1, help_calls_avoided=1, sweeps=0, rescue_finds=0, drive_outs=1,
                        help_calls=1, disagreements=1))
    for rb, w in want.items():
        assert {k: int(by[rb][k]) for k in w} == w, rb
    assert int(by["all"]["help_calls_avoided"]) == 2 and int(by["all"]["auto_retries"]) == 4
    assert "e02_state.json" in "\n".join(res["lines"])


def test_marks_alone_when_the_state_has_no_lists(tmp_path):
    """A session whose state lacks auto_retries / help_calls: both come from the marks (one clock, never mixed)."""
    sdir = _session(tmp_path, dict(setup={}), front=False)
    ev = [dict(t_wall=t, text=x) for t, x in [(10.0, "e02 auto-retry R3 1: lost"), (40.0, "e02 auto-retry R3 2: lost"),
                                              (60.0, "e02 help R3: lost"), (500.0, "e02 auto-retry R3 1: stuck")]]
    rec, source = ea.recovery(sdir, ev)
    r3 = rec[0]
    assert source == "video marks"
    assert (r3["auto_retries"], r3["retry_episodes"], r3["help_calls"], r3["help_calls_avoided"]) == (3, 2, 1, 1)


def test_state_notes_stand_in_for_missing_events(tmp_path):
    st = dict(STATE, notes=[[t["t_wall"], t["text"], True] for t in EVENTS])
    sdir = _session(tmp_path, st, front=False)
    rec, _ = ea.recovery(sdir, None)
    by = {r["robot"]: r for r in rec}
    assert by["R1"]["sweeps"] == 1 and by["R1"]["rescue_finds"] == 2 and by["R2"]["drive_outs"] == 1


def test_an_older_session_without_the_records(tmp_path):
    sdir = _session(tmp_path, None, front=False)
    out = str(tmp_path / "analysis")
    res = ea.setup_recovery(sdir, out, [])
    assert res["recovery"] == [] and res["front"]["n"] == 0
    assert not os.path.exists(os.path.join(out, "front_sensor.csv"))
    assert not os.path.exists(os.path.join(out, "recovery.csv"))
    text = "\n".join(res["lines"])
    assert "no front_checks.csv" in text and "none recorded" in text


def test_report_keeps_the_house_headings(tmp_path):
    sdir = _session(tmp_path, STATE)
    out = str(tmp_path / "analysis")
    setup = ea.setup_recovery(sdir, out, EVENTS)
    path = ea.write_report(os.path.join(out, "report.md"), [], [], [], [], {}, [], setup=setup)
    text = open(path).read()
    heads = [ln for ln in text.splitlines() if ln.startswith("## ")]
    assert heads == ["## Premise", "## Data", "## Conclusion", "## What's next", "## Two sweeps"]
    assert text.index("## Data") < text.index("### Setup and recovery") < text.index("## Conclusion")
    assert text.index("## What's next") < text.index("Set FRONT_OFFSET_CM = 2.0") < text.index("## Two sweeps")
    assert text.index("Possible inaccuracies") < text.index("Help calls avoided:")
    assert text.count("MAD x 1.4826") == 1 and "how to read" not in text.lower()


def test_finds_section(tmp_path):
    import e02_analysis as A
    d = tmp_path / "robot-logs-1"
    d.mkdir()
    (d / "find.csv").write_text(
        "robot,t_recv,run,result,known_before,before_x,before_y,before_th,bump2_x,bump2_y,bump2_th,reset_x,reset_y,"
        "reset_th,after_x,after_y,after_th,corr1_cm,corr1_deg,corr2_cm,corr2_deg,turn_k,retrace_cm,face_cm,ms_total,"
        "battery\n"
        "Left,1000.0,A-1,ok,1,0,0,0,0,0,0,0,0,0,0,0,0,-3.0,2.0,4.0,-1.0,1.01,0.5,4.75,30000,90\n"
        "Left,1300.0,A-2,failed: x board: still turning after 1500 ms of pushing: not square,1,0,0,0,0,0,0,0,0,0,0,"
        "0,0,0,0,0,0,1.0,0,4.75,9000,90\n")
    (d / "fix.csv").write_text("robot,t_recv,run,ms,t_wall,cam_x,cam_y,cam_th,k,result\nLeft,880.0,-,1,880.0,1,1,0,1,replaced\n")
    rows, summ = A.finds(str(tmp_path))
    assert len(rows) == 2 and rows[0]["pos_err_cm"] == 5.0 and rows[0]["min_since_fix"] == 2.0
    s = summ[0]
    assert s["ok"] == 1 and s["failed"] == 1 and abs(s["drift_cm_per_min"] - 2.5) < 1e-9 and s["head_med_deg"] == 2.0
    assert "x board" in s["reasons"]
    L, _ = A.finds_lines(rows, summ)
    assert any("| Left | 1 / 1 | 5.0 (5.0)" in x for x in L)
