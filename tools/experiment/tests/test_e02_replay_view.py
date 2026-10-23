"""REPLAY mode of the E02 live view (e02_replay_view.py): the state file, log lines and Live lab inputs are
revealed only up to the replay time; the clock, the camera-clock offset and the pages."""
import csv
import datetime as dt
import json
import os

import cv2
import numpy as np
import pytest

import e02_testkit  # noqa: F401  (import paths)
import e02_live as LIVE
import e02_replay_view as RV
import e02_view as VIEW

TZ = dt.timezone(dt.timedelta(hours=-4))
T0 = dt.datetime(2026, 10, 21, 22, 0, 0, tzinfo=TZ).timestamp()      # 22:00:00 local


def hms(h, m, s):
    return dt.datetime(2026, 10, 21, h, m, s, tzinfo=TZ).timestamp()


STATE = {
    "board": {"rect": [0.0, 0.0, 122.15, 121.9], "xboard_side": 1, "yboard_side": 1, "xboard_cm": 121.0,
              "yboard_cm": 121.3, "face_cm": 4.75},
    "calibration": "C:\\nowhere\\overhead_floorcal_roam1.json",
    "schedule": [
        dict(cond="U", route=1, robot="A", order=1, block=0, status="done", t_start=T0 + 100, t_end=T0 + 200,
             end_err_cm=7.5, far_err_cm=2.0, cam_end=[1, 2]),
        dict(cond="C", route=2, robot="B", order=2, block=1, status="done", t_start=T0 + 210, t_end=T0 + 300,
             end_err_cm=1.5, far_err_cm=0.5),
        dict(cond="K", route=3, robot="A", order=3, block=1, status="pending"),
    ],
    "fits": {"A": {"values": {"gyro_scale": 1.07}, "history": [
        {"t": T0 + 199, "order": 1, "accepted": True, "values": {"gyro_scale": 1.05}, "fitted": {}},
        {"t": T0 + 400, "order": 3, "accepted": True, "values": {"gyro_scale": 1.07}, "fitted": {}}]}},
    "fit_data": {"A": [{"tm": T0 + 150}, {"tm": T0 + 350}]},
    "positions": {"A": [10.0, 20.0, 0.0], "B": [30.0, 40.0, 90.0]},
    "identify": [[T0 + 20, "A", 40.0, [[11.0, 21.0, 300, 50.0]]]],
    "notes": [[T0 + 10.0, "e02 calibrate A,B", True], [T0 + 100.0, "e02 1/3 A U r1 find start", True]],
    "help_calls": [[T0 + 250, "B", "lost"]],
    "durations": [100.0, 90.0],
    "setup": {"dry_run": False, "map": {"robot": "A", "t": T0 + 60, "rays": [{"cam_start": [60, 60], "point": [120, 60]}]}},
}

LOG = """[22:00:00] E02 session folder: x (launch 1)
[22:00:05]     -> A: status
[22:00:05] A: connected
[22:01:40] === Run 1 of 3 (block 0): A, U ===
  a continuation line without a time
[22:03:20]   Result: A U straight: far point 2.0 cm off, back at start 7.5 cm off
[22:05:00] Saved in x
"""


def test_parse_log_times_quiet_and_midnight():
    log = RV.parse_log(LOG.splitlines(), T0, TZ)
    assert log[0][0] == pytest.approx(T0)
    assert log[1][2] is True and log[2][2] is False          # "    -> A: status" was a quiet line
    assert log[4][0] == log[3][0]                            # no time: the one before
    late = RV.parse_log(["[23:59:58] a", "[00:00:03] b"], hms(23, 0, 0), TZ)
    assert late[1][0] - late[0][0] == pytest.approx(5.0)     # past midnight: the next day


def test_say_lines_only_up_to_t_without_quiet_lines():
    log = RV.parse_log(LOG.splitlines(), T0, TZ)
    assert RV.say_lines(log, T0 - 1) == []
    s = RV.say_lines(log, T0 + 5)
    assert s == ["[22:00:00] E02 session folder: x (launch 1)", "[22:00:05] A: connected"]
    s = RV.say_lines(log, T0 + 200)
    assert s[-1].endswith("back at start 7.5 cm off") and not any("Saved" in x for x in s)
    assert len(RV.say_lines(log, T0 + 1e4, keep=2)) == 2
    assert RV.say_lines(log, T0 + 1e4, keep=2)[-1] == "[22:05:00] Saved in x"


def test_schedule_results_revealed_at_each_runs_end():
    before = RV.gate_state(STATE, T0 + 50)
    assert [r["status"] for r in before["schedule"]] == ["pending", "pending", "pending"]
    assert "end_err_cm" not in before["schedule"][0] and "t_start" not in before["schedule"][0]
    running = RV.gate_state(STATE, T0 + 150)
    assert running["schedule"][0]["status"] == "running" and "end_err_cm" not in running["schedule"][0]
    assert LIVE.results_series(running) == []
    one = RV.gate_state(STATE, T0 + 250)
    assert [r["end_err_cm"] for r in LIVE.results_series(one)] == [7.5]
    assert one["schedule"][1]["status"] == "running"
    assert one["durations"] == [100.0]
    both = RV.gate_state(STATE, T0 + 1000)
    assert [r["end_err_cm"] for r in LIVE.results_series(both)] == [7.5, 1.5]
    assert both["schedule"][2]["status"] == "pending"
    assert STATE["schedule"][0]["status"] == "done"         # the full state is never changed


def test_fits_history_grows_with_time():
    assert RV.gate_state(STATE, T0 + 100)["fits"] == {}
    mid = RV.gate_state(STATE, T0 + 300)
    assert [c["gyro_scale"] for c in LIVE.calibration_series(mid)] == [1.05]
    assert mid["fits"]["A"]["values"] == {"gyro_scale": 1.05}
    assert len(mid["fit_data"]["A"]) == 1
    end = RV.gate_state(STATE, T0 + 500)
    assert [c["gyro_scale"] for c in LIVE.calibration_series(end)] == [1.05, 1.07]
    assert end["fits"]["A"]["values"] == {"gyro_scale": 1.07}


def test_map_lists_and_positions_gated():
    early = RV.gate_state(STATE, T0 + 30)
    assert "map" not in early["setup"] and early["setup"]["dry_run"] is False
    assert len(early["notes"]) == 1 and early["help_calls"] == []
    assert early["positions"] == {"A": [11.0, 21.0]}         # from identify
    later = RV.gate_state(STATE, T0 + 260, track_last={"A": (T0 + 255, 50.0, 60.0), "B": (T0 + 240, 5.0, 6.0)})
    assert later["setup"]["map"]["robot"] == "A"
    assert later["positions"] == {"A": [50.0, 60.0], "B": [5.0, 6.0]}
    assert len(later["help_calls"]) == 1
    over = RV.gate_state(STATE, T0 + 2000, session_over=True)
    assert over["positions"] == STATE["positions"]


def test_clock_speed_pause_jump_and_end():
    w = [0.0]
    c = RV.Clock(100.0, 200.0, start=110.0, speed=1, wall=lambda: w[0])
    w[0] = 5.0
    assert c.now() == pytest.approx(115.0)
    c.set(speed=4)
    w[0] = 6.0
    assert c.now() == pytest.approx(119.0)
    c.set(paused=True)
    w[0] = 50.0
    assert c.now() == pytest.approx(119.0)
    c.set(jump=150.0, paused=False)
    w[0] = 51.0
    assert c.now() == pytest.approx(154.0)
    w[0] = 500.0
    assert c.now() == 200.0 and c.paused                     # stops at the end
    c.set(jump=10.0)
    assert c.now() == 100.0                                  # jumps are kept inside the session


# ----------------------------------------------------------------------------- a whole small session
@pytest.fixture
def session(tmp_path):
    s = tmp_path / "e02-20261021-220000"
    (s / "robot-logs-1").mkdir(parents=True)
    json.dump(STATE, open(s / "e02_state.json", "w"))
    (s / "e02_log.txt").write_text(LOG)
    json.dump({"start": "2026-10-21T22:00:00-04:00", "start_unix": T0}, open(s / "robot-logs-1" / "session.json", "w"))
    with open(s / "track.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "robot", "x_cm", "y_cm", "order", "phase"])
        for k in range(50):
            w.writerow(["%.3f" % (T0 + 110 + 0.2 * k), "A", "%.2f" % (60 + 0.4 * k), "60.00", "1", "route"])
    with open(s / "robot-logs-1" / "raw.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t_recv", "run", "v_cms", "w_dps", "gz", "x_cm", "y_cm", "th_deg"])
        for k in range(100):
            w.writerow(["A", "%.3f" % (T0 + 110 + 0.1 * k), "AA-1", "2.0", "0.0", "0.0", "%.2f" % (0.2 * k), "0", "0"])
    rec = tmp_path / "rec"
    rec.mkdir()
    off = 0.7                                       # the Pi's clock runs 0.7 s ahead of the laptop's
    vw = cv2.VideoWriter(str(rec / "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), 10, (640, 480))
    with open(rec / "frames.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t_wall"])
        for k in range(30):                         # 10 fps from laptop T0+105 to T0+107.9; brightness = k * 8
            vw.write(np.full((480, 640, 3), k * 8, np.uint8))
            w.writerow([k, "%.4f" % (T0 + 105 + 0.1 * k + off)])
    vw.release()
    with open(rec / "events.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_wall", "frame", "text"])
        for t, text, _ in STATE["notes"]:
            w.writerow(["%.3f" % (t + off), 0, text])
    return s, rec


def test_runner_at_replay_time(session):
    s, rec = session
    video = RV.Video(*RV.find_recording(str(rec)))
    w = [0.0]
    r = RV.ReplayRunner(str(s), video=video, start=T0 + 106.0, wall=lambda: w[0])
    assert video.offset == pytest.approx(0.7) and "video marks" in video.offset_source
    assert r.label == "2026-10-21 22:00:00"
    r.step()
    assert abs(float(r.feed.last_img.mean()) - 10 * 8) < 6        # frame 10 is laptop time T0+106
    assert r.tracker.active is None and r.st["schedule"][0]["status"] == "running"  # started at T0+100
    assert r.lines[-2:] == ["[22:01:40] === Run 1 of 3 (block 0): A, U ===", "  a continuation line without a time"]
    w[0] = 9.0                                                     # T0+115: A is driving its run
    r.step()
    assert r.feed.last_img is None                                 # after the recording ends: no picture
    assert r.tracker.active == "A" and r.tracker.where()[1] == pytest.approx(60 + 0.4 * 25, abs=0.5)
    assert r.phase == "route" and r.order == "1"
    assert r.st["schedule"][0]["status"] == "running"
    f = r.lab.state()
    assert f["robot"] == "A" and f["n_camera"] > 0
    assert not any("Result" in x for x in r.lines)
    r.clock.set(jump=T0 + 250)
    r.step()
    assert [x["end_err_cm"] for x in LIVE.results_series(r.st)] == [7.5]
    assert r.tracker.active is None
    r.clock.set(jump=T0 + 1e4)
    r.step()
    assert r.phase == "session over" and r.st["positions"] == STATE["positions"]


def test_pages_replay_only(session):
    s, rec = session
    r = RV.ReplayRunner(str(s), video=RV.Video(*RV.find_recording(str(rec))), start=T0 + 106.0)
    r.step()
    v = RV.ReplayView(r, 0)
    for name in ("main", "lab"):
        html = v.page(name)
        assert "REPLAY of the session of 2026-10-21 22:00:00" in html and 'id="rp-bar"' in html
    live = VIEW.LiveView(r, 0)
    assert live.page("main") == VIEW.PAGE and live.page("lab") == VIEW.LAB_PAGE and "rp-bar" not in VIEW.PAGE
    assert live.extra_get("/replay.json") is None
    body, ctype = v.extra_get("/replay/control?paused=1&speed=2&jump=22:03:00")
    d = json.loads(body)
    assert d["paused"] and d["speed"] == 2 and d["now_hms"] == "22:03:00"
    assert v.status()["log"][-1] == "  a continuation line without a time"
    assert v.frame_jpg()[:2] == b"\xff\xd8"


def test_tracker_health_levels():
    """Item 6 (2026-10-23): the live view's tracker health from the tracker's counters and perf.csv."""
    import time
    import e02_view as V

    class T:
        active = "Left"
        last_seen = time.time() - 0.2

        def __init__(self):
            self.s = dict(both=0, lock=0, detector=0, held=0, none=0, checks=0, agree=0, disagree=0, relocks=0)

        def confidence(self):
            return dict(source="both", score=0.82, stats=dict(self.s))

    class R:
        tracker = T()
        perf_rows = [dict(fps_handled=15.0, age_max_s=1.4, dropped=3)]

    v = V.LiveView(R(), port=0)
    assert v.health()["pictures"] == 0
    R.tracker.s.update(both=80, lock=5, none=15, checks=10, disagree=3, relocks=1)
    h = v.health()
    assert h["pictures"] == 100 and abs(h["seen_share"] - 0.85) < 1e-9
    assert h["levels"]["seen"] == "amber" and h["levels"]["age"] == "red" and h["levels"]["disagree"] == "red"
    assert h["levels"]["score"] == "ok" and h["levels"]["seen_age"] == "ok"
