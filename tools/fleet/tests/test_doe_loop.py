"""The DOE loop's own parts (2026-10-27 evening): where a drive's line goes, the miss pattern, the distance-sensor
runs and the 1/2/3-in-a-row evaluation, the check before every move (restart, battery, settings read back), the clip
events, and upload_session's unique tag.

    python3 -m pytest tools/fleet/tests/test_doe_loop.py -q          # a few seconds
The loop itself against the simulated board: FLEET_DRY=1 DOE_DRY_ARGS="--sim-washout --sim-event restart:Center@2
--sim-event battery:Left=24@3" ./run-fleet-pi.sh doe-run 0.25 (docs/build-reports/e02-oct27-doe-loop.md).
"""
import csv
import json
import math
import os
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, os.path.join(TOOLS, "sim"), TOOLS, os.path.join(TOOLS, "pi")):
    if p not in sys.path:
        sys.path.append(p)

import doe_loop as D        # noqa: E402
import ladder as LD         # noqa: E402

RECT = (0.0, 0.0, 122.0, 122.0)


@pytest.mark.parametrize("length", D.LENGTHS)
def test_segment_keeps_25cm_from_edges_and_parked(length):
    parked = [(61.0, 61.0), (35.0, 90.0)]
    a, b = D.pick_segment(RECT, (40.0, 40.0), length, parked)
    assert math.dist(a, b) == pytest.approx(length, abs=0.01)
    for p in (a, b):
        assert LD.edge_dist(RECT, *p) >= D.CLEAR_CM - 1e-6
    assert LD.segs_clear([(a, b)], parked, 0) >= D.PARKED_CLEAR_CM - 1e-6


def test_segment_none_when_no_room():
    parked = [(x, y) for x in range(25, 100, 12) for y in range(25, 100, 12)]
    assert D.pick_segment(RECT, (40.0, 40.0), 40.0, parked) is None


def _session(tmp, raw_rows, stops, plan_lines=None, track=None):
    s = tmp
    with open(os.path.join(s, "raw.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t_recv", "dL_cm", "dCL_cm", "dC_cm", "dCR_cm", "dR_cm"])
        for r in raw_rows:
            w.writerow(r)
    with open(os.path.join(s, "coordinator.jsonl"), "w") as f:
        f.write(json.dumps(dict(t=0.0, event="step_sent", robot="Left", id=1, text="x")) + "\n")
        for t, info in stops:
            f.write(json.dumps(dict(t=t, event="step_report", robot="Left", id=2, state="failed", info=info)) + "\n")
        f.write(json.dumps(dict(t=100.0, event="step_report", robot="Left", id=3, state="done")) + "\n")
    if plan_lines:
        steps = [dict(type="drive", **{"from": [a[0], a[1], 0], "to": [b[0], b[1], 0]}) for a, b in plan_lines]
        json.dump(dict(robots={"Left": dict(steps=steps)}), open(os.path.join(s, "plan_given.json"), "w"))
    if track:
        os.makedirs(os.path.join(s, "bridge"), exist_ok=True)
        with open(os.path.join(s, "bridge", "track.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "robot", "x", "y", "status"])
            for t, x, y in track:
                w.writerow([t, "Left", x, y, "seen"])
    return s


def test_sensor_runs_single_flicker_vs_real_stop(tmp_path):
    far = [120, 120, 120, 120, 120]
    rows = []
    t = 1.0
    seq = [far, [120, 120, 120, 11.5, 120], far, [121, 120, 120, 120, 120],                                    # one flicker: 1 reading
           [120, 120, 120, 11.8, 14.0], [120, 120, 120, 11.6, 12.0], far,                 # 2 in a row: the stop
           [120, 12.5, 120, 120, 120], [120, 12.0, 120, 120, 120], [120, 11.0, 120, 120, 120], far]   # 3 in a row
    for z in seq:
        rows.append(["Left", "%.2f" % t] + z)
        rows.append(["Left", "%.2f" % (t + 0.02)] + z)        # the same sensor picture in the next RAW row
        t += 0.066
    t_stop = 1.0 + 5 * 0.066 + 0.01
    s = _session(str(tmp_path), rows, [(t_stop, "obstacle 11.6 cm ahead; zones 120 120 120 11.6 12.0")])
    summ, runs = D.sensor_runs(s, "Left")
    assert summ["readings"] == len(seq)
    assert [r["readings"] for r in runs] == [1, 2, 3]
    assert summ["single"] == 1 and summ["runs_2"] == 1 and summ["runs_3_plus"] == 1
    assert summ["stops"] == 1
    assert summ["stops_avoided_by_2"] == 0 and summ["stops_avoided_by_3"] == 1
    assert runs[0]["zones"] == "CR"


def test_miss_pattern_single_vs_runs(tmp_path):
    lines = [((30.0, 50.0), (70.0, 50.0))]
    off = {3: 3.5, 10: 4.0, 11: 4.2, 12: 3.8, 20: 3.1}
    track = [(1.0 + i * 0.1, 30.0 + i, 50.0 + off.get(i, 0.5)) for i in range(40)]
    s = _session(str(tmp_path), [], [], lines, track)
    mp = D.miss_pattern(s, os.path.join(s, "bridge"), "Left")
    assert mp["n"] == 40 and mp["over"] == 5 and mp["runs"] == 3 and mp["single"] == 2 and mp["longest"] == 3
    assert mp["in_runs_of_3_plus"] == 3


def test_clip_kinds():
    assert D.clip_kind(dict(event="step_report", state="failed", info="obstacle 11.6 cm ahead")) == "near-robot stop"
    assert D.clip_kind(dict(event="step_report", state="failed", info="plywood edge at 1 2 3 line")) == "edge stop"
    assert D.clip_kind(dict(event="step_report", state="held")) == "hold"
    assert D.clip_kind(dict(event="step_report", state="done")) is None


def test_sim_events():
    assert D.parse_sim_events(["restart:Center@2", "battery:Left=24@3"]) == [("restart", "Center", None, 2),
                                                                             ("battery", "Left", "24", 3)]
    with pytest.raises(SystemExit):
        D.parse_sim_events(["boom:Left@1"])


class FakeBench:
    """Answers like the robots: status (battery), clock (boot id, uptime), get (settings)."""

    def __init__(self, loop, robots):
        self.loop, self.r = loop, robots
        self.fw, self.sent = {}, []
        self.link = types.SimpleNamespace(session=types.SimpleNamespace(listeners=[]))

    def robots(self):
        return list(self.r)

    def send(self, n, cmd):
        self.sent.append((n, cmd))
        r = self.r[n]
        if r.get("silent"):
            return
        v = cmd.split()
        if v[0] == "status":
            self.fw[n] = ("2026-10-25c", r["battery"])
        elif v[0] == "clock":
            for fn in self.link.session.listeners:
                fn(n, "D", "CLOCK", [v[1], str(int(r["up"] * 1000)), r["boot"]])
        elif v[0] == "get":
            for fn in self.link.session.listeners:
                fn(n, "M", "", "%s = %.3f  (help)" % (v[1], r["params"][v[1]]))
        elif v[0] == "set" and v[1] in r["params"] and not r.get("lose_set"):
            r["params"][v[1]] = float(v[2])

    def wait(self, cond, s):
        return cond()


def _loop(tmp_path, robots):
    L = D.DoeLoop.__new__(D.DoeLoop)
    L.a = types.SimpleNamespace(dry=False)
    L.dir = str(tmp_path)
    L.state_path = os.path.join(L.dir, "state.json")
    L.st = {"passed": {"L1:%s" % n: 1 for n in robots}, "results": {}, "robots": {n: {"x": 60, "y": 60, "h": 0}
                                                                                    for n in robots},
            "kept_out": {}, "doe": {"i": {}, "boot": {}, "uptime": {}, "loops": 0}}
    L.got, L.files, L.frames, L.keeper, L.eye = {}, {}, None, None, None
    L.bench = FakeBench(L, robots)
    L.need_bench = lambda sub: L.bench
    L.recal = []

    def L1(sub, names):
        L.recal += names
        for n in names:
            L.st["passed"]["L1:%s" % n] = 1
        return True
    L.L1 = L1
    return L


def _good(boot="A1B2", up=100.0, battery=60):
    return dict(boot=boot, up=up, battery=battery, params=dict(gyro_scale=1.075, wheel_scale=0.979,
                                                               gyro_drift_dps=0.0))


def test_check_restart_battery_and_settings(tmp_path, monkeypatch):
    monkeypatch.setattr(LD, "corrections_for", lambda n: dict(gyro_scale=1.075, wheel_scale=0.979, gyro_drift_dps=0.0))
    monkeypatch.setattr(D.time, "sleep", lambda s: None)
    robots = {"Left": _good(), "Right": _good("C0DE"), "Center": _good("4305")}
    L = _loop(tmp_path, robots)
    assert sorted(L.check(str(tmp_path), list(robots), "loop 0")) == ["Center", "Left", "Right"]
    assert not L.recal
    # Center restarts (new boot id, factory settings), Left drops to 24 %, Right loses a `set wheel_scale`
    robots["Center"].update(boot="9DA8", up=20.0, params=dict(gyro_scale=1.0, wheel_scale=1.0, gyro_drift_dps=0.0))
    robots["Left"]["battery"] = 24
    robots["Right"]["params"]["wheel_scale"] = 1.0
    fine = L.check(str(tmp_path), list(robots), "loop 1")
    assert "Left" in L.st["kept_out"] and "battery 24%" in L.st["kept_out"]["Left"]
    assert L.recal == ["Center"]
    assert ("Center", "wake") in L.bench.sent and ("Center", "set wheel_scale 0.9790") in L.bench.sent
    assert ("Right", "set wheel_scale 0.9790") in L.bench.sent
    assert sorted(fine) == ["Center", "Right"]
    # a setting that will not stick parks the robot; a silent robot is woken, then parked
    robots["Right"]["params"]["gyro_scale"] = 1.0
    robots["Right"]["lose_set"] = True
    robots["Center"]["silent"] = True
    assert L.check(str(tmp_path), ["Right", "Center"], "loop 2") == []
    assert "does not read back" in L.st["kept_out"]["Right"] and "not answering" in L.st["kept_out"]["Center"]
    assert ("Center", "wake") in L.bench.sent
    rows = list(csv.DictReader(open(os.path.join(str(tmp_path), "battery.csv"))))
    assert {r["robot"] for r in rows} == {"Left", "Right", "Center"} and rows[0]["boot"] == "A1B2"


def test_free_tag():
    import upload_session as U
    have = {"doe-x", "doe-x-2"}
    assert U.free_tag("doe-x", "tok", exists=lambda t: t in have) == "doe-x-3"
    assert U.free_tag("doe-y", "tok", exists=lambda t: t in have) == "doe-y"


def test_summary_table(tmp_path):
    p = os.path.join(str(tmp_path), "drives.csv")
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["speed_cms", "length_cm", "p95_cm", "over_3cm", "miss_single", "sensor_low",
                                          "sensor_stops", "stops_avoided_by_2", "stops_avoided_by_3"])
        w.writeheader()
        w.writerow(dict(speed_cms=5.0, length_cm=25.0, p95_cm=1.2, over_3cm=0, miss_single=0, sensor_low=3,
                        sensor_stops=1, stops_avoided_by_2=1, stops_avoided_by_3=1))
    D.write_summary(str(tmp_path))
    s = open(os.path.join(str(tmp_path), "summary.md")).read()
    assert "| 5 | 25 | 1 | 1.20 | 0 | 0 | 3 | 1 | 1 / 1 |" in s


def _loop_stub(tmp_path, keep):
    L = D.DoeLoop.__new__(D.DoeLoop)
    L.a = types.SimpleNamespace(keep_going=keep)
    L.st = {"kept_out": {}, "doe": {}}
    L.bench, L.frames, L.files, L.row_err = None, None, {}, False
    L.dir = str(tmp_path)
    L.save = lambda: None
    L.keep_out = lambda n, why: L.st["kept_out"].__setitem__(n, why)
    return L


def test_keep_going_parks_only_for_safety(tmp_path):
    """--keep-going (Nick 2026-10-28 18:00): a turn-check FAIL, a crawl's wrong distance or a line check is logged as
    "would have parked" and the robot stays in; battery, lost by both cameras, edge stops still park."""
    L = _loop_stub(tmp_path, True)
    L.park("Center", "turn check FAIL (errors +3.9, -1.8, +7.9, -5.6, total +4.3 deg)")
    L.park("Left", "heading crawl: the camera saw Left crawl 15.0 cm (it was sent 10 cm): the camera or its wheels "
                   "are wrong", safety=D.lost_words("the camera saw Left crawl 15.0 cm"))
    L.park("Right", "drive 3: p95 9.1 cm, over the 8 cm safety limit")
    assert L.st["kept_out"] == {}
    assert [w["robot"] for w in L.st["doe"]["would_park"]] == ["Center", "Left", "Right"]
    L.park("Right", "battery 22% (under 25%)", safety=True)
    L.park("Left", "drive 2: the camera lost it", safety=D.lost_words("the camera lost it"))
    assert set(L.st["kept_out"]) == {"Right", "Left"}
    ev = open(os.path.join(str(tmp_path), "events.csv")).read()
    assert ev.count("would have parked") == 3 and ev.count(",parked,") == 2
    # without it, everything parks as before
    M = _loop_stub(tmp_path / "x", False)
    os.makedirs(M.dir, exist_ok=True)
    M.park("Center", "turn check FAIL")
    assert "Center" in M.st["kept_out"]


def test_keep_going_body_unseen_stays_in(tmp_path):
    L = _loop_stub(tmp_path, True)
    L.st["kept_out"]["Right"] = "no camera can see its body (only its lights)"
    L.st["kept_out"]["Left"] = "not found by its lights"
    L.keep_going_back(["Right", "Left"])
    assert "Right" not in L.st["kept_out"] and "Left" in L.st["kept_out"]


def test_lost_words():
    assert D.lost_words("the camera lost Left after turn 2")
    assert D.lost_words("drive 1 did not finish (edge stop)")
    assert not D.lost_words("the camera saw Left crawl 15.0 cm (it was sent 10 cm): the camera or its wheels are wrong")
    assert not D.lost_words("turn check FAIL (errors +3.9)")


def test_one_camera_turn_fail_is_held_not_parked(tmp_path):
    """Never set aside on one camera's evidence (Nick, 2026-10-28): a turn-check FAIL whose headings came from one
    camera is held (turn check due again, no drive this loop); with both cameras it parks as before."""
    L = _loop_stub(tmp_path, False)
    L.st["robots"] = {"Center": {"h": 10.0}}
    L.frames_on = L.frames_off = lambda *a: None
    res = dict(turns=[dict(turn=1, camera_deg=97.9, own_deg=89.7, error_deg=7.9)], total_error_deg=4.3,
               one_camera=True, evidence=["side", "side", "both", "side", "side"])
    L.turncheck = lambda n, sub: (False, res)
    L.row = lambda *a, **k: None
    assert L.turn(str(tmp_path), "Center", 2) is False
    assert "Center" not in L.st["kept_out"] and L.st["doe"]["turn_due"]["Center"] and L.last_held == "Center"
    assert "h" not in L.st["robots"]["Center"]
    res2 = dict(res, one_camera=False, evidence=["both"] * 5)
    L.turncheck = lambda n, sub: (False, res2)
    L.last_held = None
    L.turn(str(tmp_path), "Center", 3)
    assert "Center" in L.st["kept_out"]


def test_one_camera_crawl_fail_is_held(tmp_path, monkeypatch):
    L = _loop_stub(tmp_path, False)
    L.st["robots"] = {"Left": {"x": 88.0, "y": 42.0}}
    L.frames_on = L.frames_off = lambda *a: None
    L.pos = lambda n: (88.0, 42.0)
    L.heading = lambda n: None
    L.need_bench = L.need_eye = lambda *a: None
    L.eye = types.SimpleNamespace(still=lambda n: None)

    def crawl(n, p, h):
        e = LD.LadderStop("the camera saw Left crawl 15.0 cm (it was sent 10 cm): the camera or its wheels are wrong")
        e.evidence, e.heading = "side", -88.0
        raise e
    L.crawl = crawl
    assert L.drive(str(tmp_path), "Left", 1, 4) == "held"
    assert "Left" not in L.st["kept_out"]
    ev = open(os.path.join(str(tmp_path), "events.csv")).read()
    assert "held" in ev and "one camera only (side)" in ev


def test_lost_words_room_is_not_safety():
    assert not D.lost_words("turn check could not finish: Left is 24 cm from the x = 122 edge: the turn check needs it "
                            "25 cm in (it turns and crawls 10 cm). Not checked.")
    assert D.lost_words("drive 1 did not finish (plywood edge)")


def _room_stub(tmp_path, pos, heading=10.0, others=None):
    L = _loop_stub(tmp_path, True)
    L.rect = (0.0, 0.0, 122.0, 122.0)
    L.st["robots"] = {"Left": {"x": pos[0], "y": pos[1], "h": heading}}
    for m, q in (others or {}).items():
        L.st["robots"][m] = {"x": q[0], "y": q[1]}
    L.frames_on = L.frames_off = lambda *a: None
    L.need_bench = L.need_eye = lambda *a: None
    L.eye = types.SimpleNamespace(still=lambda n: L.pos(n))
    L.pos = lambda n: (L.st["robots"][n]["x"], L.st["robots"][n]["y"])
    L.heading = lambda n: L.st["robots"][n].get("h")
    L.others_of = lambda n: {m: L.pos(m) for m in L.st["robots"] if m != n}
    L.set_pose = lambda n, x, y, h=None, how=None: L.st["robots"][n].update(
        dict(x=x, y=y, **({} if h is None else {"h": h})))
    L.forget_heading = lambda n, why: L.st["robots"][n].pop("h", None)
    return L


def test_near_an_edge_it_drives_to_open_space_instead_of_parking(tmp_path):
    """19:27, 28 Oct: Left 24 cm from the x = 122 edge was parked ("the turn check needs it 25 cm in"); 19:34 Center
    at x = 0 the same; the loop ended at 19:44 with all three out. Now it is driven back to open space."""
    L = _room_stub(tmp_path, (98.0, 42.0), others={"Right": (46.0, 93.0)})
    drove = {}

    def drive_to(sub, tag, targets, cms=5.0, via=None):
        drove.update(targets)
        (x, y, h), = targets.values()
        L.st["robots"]["Left"].update(x=x, y=y)
        return {"Left": dict(x=x, y=y, h=h, how="camera line")}
    L.drive_to = drive_to
    assert L.ensure_room(str(tmp_path), "Left", 3) is True
    x, y, _ = drove["Left"]
    assert D.edge_dist(L.rect, x, y) >= D.ROOM_TARGET_CM and math.dist((x, y), (46.0, 93.0)) >= D.ROOM_TARGET_ROBOT_CM
    assert math.dist((x, y), (98.0, 42.0)) < 15.0           # the nearest open spot, not across the board
    assert "Left" not in L.st["kept_out"]
    assert "room made" in open(os.path.join(str(tmp_path), "events.csv")).read()
    # with room already: nothing moves
    drove.clear()
    assert L.ensure_room(str(tmp_path), "Left", 4) is True and not drove


def test_making_room_with_heading_unknown_crawls_first_and_a_failed_move_is_held(tmp_path):
    L = _room_stub(tmp_path, (5.0, 22.0), heading=None)
    crawled = []
    L.crawl = lambda n, p, h: crawled.append(p) or (-150.0, {})

    def drive_to(sub, tag, targets, cms=5.0, via=None):
        raise D.LadderStop("the drive to room-Left-3's start did not finish (camera and Left's own pose disagree)")
    L.drive_to = drive_to
    assert L.ensure_room(str(tmp_path), "Left", 3) is False
    assert crawled and "Left" not in L.st["kept_out"] and "h" not in L.st["robots"]["Left"]
    assert "held" in open(os.path.join(str(tmp_path), "events.csv")).read()


def test_lost_during_a_turn_check_crawl_is_held_not_parked(tmp_path):
    """19:43, 28 Oct: "the camera lost Right during its crawl" parked Right (and ended the loop); it stands still, so
    it is stopped and held, and the next find step looks for it."""
    L = _loop_stub(tmp_path, True)
    L.st["robots"] = {"Right": {"h": 10.0}}
    L.frames_on = L.frames_off = lambda *a: None
    sent = []
    L.bench = types.SimpleNamespace(send=lambda n, c: sent.append(c))

    def tc(n, sub):
        raise LD.LadderStop("the camera lost Right during its crawl (still not found after waiting and looking for "
                            "its lights)")
    L.turncheck = tc
    assert L.turn(str(tmp_path), "Right", 19) is False
    assert "Right" not in L.st["kept_out"] and "stop" in sent and L.st["doe"]["turn_due"]["Right"]


def test_l1_calibration_is_written_to_run_plans_day_record(tmp_path, monkeypatch):
    """19:23, 28 Oct: run_plan's day record still had Right's morning boot id, so it calibrated Right again inside
    drive 1 and the drive failed to start."""
    import engine_checks as EC
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    L = _loop(tmp_path, {"Right": _good(boot="6384")})
    L.got = {"Right": {"boot": "6384"}}
    L.note_calibrated(["Right"])
    day = EC.DayState()
    assert day.restarted("Right", "6384") is False
