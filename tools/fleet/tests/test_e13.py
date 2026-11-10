"""e13 (10 Nov 2026): school free roam inside the readable area (stray return), never give up (a held robot waits for a
hand and rejoins), robots sleep 10 min after a run, the risk-based no-pictures limit, camera discovery inside robot
finding, runs without roof tags, the detector's auto-labelling pipeline, school runs copied to the home Pi."""
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
sys.path[:0] = [FLEET, TOOLS, os.path.join(TOOLS, "sim")]

import freeroam_core as FC  # noqa: E402
import proven_workspace as PW  # noqa: E402
import safety_rules as SR  # noqa: E402


# -------------------------------------------------------------------------------------------- 1. the readable area
def _ws(limit_x=200.0):
    ws = PW.ProvenWorkspace((0.0, 0.0, 300.0, 160.0))
    ws.see((0.0, 0.0, 300.0, 160.0))
    if limit_x is not None:
        ws.limit_to(lambda x, y: x < limit_x)
    return ws


def test_workspace_limited_to_the_readable_area_keeps_the_table_raw():
    ws = _ws(200.0)
    assert ws.limited and ws.allowed(100, 80) and not ws.allowed(250, 80)
    assert ws.edge_dist(250, 80) < 0 < ws.edge_dist(250, 80, raw=True)        # unreadable but table
    assert ws.edge_dist(150, 80) == ws.edge_dist(150, 80)                     # cached
    assert 45 < ws.edge_dist(150, 80) < 55                                    # 50 cm to x = 200
    x, y = ws.nearest_allowed(250, 80, 6.7)
    assert x < 200 and abs(y - 80) < 3
    assert ws.summary()["readable_limit"] is True
    ws.unlimit()
    assert not ws.limited and ws.edge_dist(250, 80) > 0


def test_freeroam_limits_its_workspace_to_the_readmap(tmp_path, monkeypatch):
    import argparse
    import freeroam as FR
    import observe as OB
    rm = OB.ReadMap(300, 160)
    rm.seed(lambda x, y: 0.95 if x < 200 else 0.0, "test")
    p = tmp_path / "readmap.json"
    rm.save(str(p))
    r = FR.Runner.__new__(FR.Runner)
    r.a = argparse.Namespace(readable_map=str(p))
    ws = _ws(None)
    assert r.limit_readable(ws) > 0 and ws.limited and not ws.allowed(250, 80) and ws.allowed(100, 80)
    monkeypatch.setenv("MIRLT_ROAM_READABLE", "0")                  # the switch: the whole proven table (before e13)
    ws2 = _ws(None)
    assert r.limit_readable(ws2) is None and not ws2.limited


def test_mission_run_gives_the_school_free_roam_its_readability_map():
    src = open(os.path.join(FLEET, "mission_run.py")).read()
    assert '"--readable-map", os.path.join(out, "readmap_start.json")' in src


def test_a_robot_outside_the_readable_area_comes_back():
    import freeroam_fastsim as S
    sim = S.Sim(3, 0.03, dead_robot=False, pi_freeze=False, dropouts=False, board=(0, 0, 300, 160), workspace=True,
                starts={"Left": (60, 50, 0), "Center": (240, 80, 30.0), "Right": (100, 120, 180)})
    sim.ws.limit_to(lambda x, y: x < 215)
    sim.run()
    c = sim.ctrl
    assert c.counts.get("stray_returns", 0) >= 1
    assert sim.bots["Center"].x < 215                     # back inside, and it roamed on


def test_stray_return_switch_off(monkeypatch):
    old = FC.STRAY_RETURN
    FC.STRAY_RETURN = False
    try:
        c = FC.Controller(["A"], (0, 0, 300, 160), lambda *a: None, lambda *a: None, workspace=_ws(200.0))
        r = c.R["A"]
        r.cam, r.cam_arr = (0.0, 240.0, 80.0, 0.0), 0.0
        assert not c.strayed(r)
    finally:
        FC.STRAY_RETURN = old


# -------------------------------------------------------------------------------------------- 2. never give up
def _hand_sim(push=True):
    import freeroam_fastsim as S
    sim = S.Sim(1, 0.06, dead_robot=False, pi_freeze=False, edge_physics=True,
                starts={"Center": (3.5, 18.6, 117.0)}, ctrl_kw=dict(start_mode="wait"),
                pushes=[("Center", 120.0, 25.0, 0.0)] if push else [])
    r = sim.run()
    return sim.ctrl, r


def test_a_robot_held_at_an_edge_asks_for_a_hand_and_rejoins_when_moved_in():
    assert not FC.EDGE_PARK                                       # e12's park is the switch now, off by default
    c, r = _hand_sim(True)
    assert c.counts.get("need_hand") == 1 and c.counts.get("hand_rejoined") == 1
    assert c.R["Center"].mode != "out" and not c.needs_hand()
    assert r["moving_pct"]["Center"] > 10 and r["moving_pct"]["Left"] > 50 and r["moving_pct"]["Right"] > 50


def test_a_robot_waiting_for_a_hand_is_never_dropped_by_a_timer_and_its_battery_still_ends_it():
    c, r = _hand_sim(False)
    rc = c.R["Center"]
    assert rc.mode != "out" and c.needs_hand() and "NEEDS A HAND" in c.needs_hand()[0]["words"]
    rc.battery = c.floor
    assert c.need_hand_watch(rc) and rc.mode == "out" and "battery" in rc.out_why and not c.needs_hand()


def test_needs_hand_shown_in_state_status_and_mission_control(tmp_path, capsys):
    import freeroam as FR
    import start_job as SJ
    d = tmp_path / "run" / "fr"
    d.mkdir(parents=True)
    FR.write_edge_parked(str(d), [], [{"robot": "Center", "words": "Center NEEDS A HAND at 3, 20"}])
    (d / "state.json").write_text(json.dumps({"t0": 1, "now": 61, "robots": {}, "needs_hand": ["Center NEEDS A HAND"]}))
    assert FR.status(str(d)) == 0
    assert "NEEDS A HAND" in capsys.readouterr().out
    st = SJ.Starter.__new__(SJ.Starter)
    assert st.edge_parked_words(str(tmp_path / "run"), key="needs_hand")[0].startswith("Center NEEDS")
    assert st.edge_parked_words(str(tmp_path / "run")) == []
    assert "st.needs_hand" in open(os.path.join(FLEET, "mission_control.html")).read()


# -------------------------------------------------------------------------------------------- 3. sleep after a run
def test_robots_sleep_ten_minutes_after_a_run_unless_a_new_one_starts(tmp_path, monkeypatch):
    import idle_after_run as IA
    monkeypatch.delenv("PYTEST_CURRENT_TEST", raising=False)
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    monkeypatch.delenv("MIRLT_IDLE_SLEEP_MIN", raising=False)
    assert IA.idle_min() == 10.0
    IA.begin("visit")
    assert [g["kind"] for g in IA.going()] == ["visit"]
    at = IA.end("visit", spawn=False)
    d = IA._load()
    assert IA.going() == [] and abs(at - d["last_end"] - 600) < 1
    ok, why = IA.decide(d, d["last_end"], d["last_end"] + 601)
    assert ok and "10 min" in why
    d2 = dict(d, last_start=d["last_end"] + 30)                      # a new run started meanwhile
    assert IA.decide(d2, d["last_end"], d["last_end"] + 601)[0] is False
    d3 = dict(d, going=[{"pid": os.getpid(), "kind": "freeroam"}])  # one is going
    assert IA.decide(d3, d["last_end"], d["last_end"] + 601)[0] is False
    assert IA.end("x", dry=True, spawn=False) is None                # practice runs never schedule a sleep
    monkeypatch.setenv("MIRLT_IDLE_SLEEP_MIN", "0")                  # the switch: never (before e13)
    assert IA.end("visit", spawn=False) is None


def test_the_waiter_sends_the_sleep(tmp_path, monkeypatch):
    import idle_after_run as IA
    monkeypatch.delenv("PYTEST_CURRENT_TEST", raising=False)
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    monkeypatch.setenv("MIRLT_IDLE_SLEEP_MIN", "0.001")
    IA.end("calib", spawn=False)
    sent = []
    assert IA.wait(IA._load()["last_end"], sleep_fn=sent.append) and sent
    assert "SLEEP sent" in (tmp_path / "idle_sleep.log").read_text()


def test_terminal_runs_are_tracked():
    for f, kind in (("freeroam.py", "freeroam"), ("mission_run.py", "mission"), ("calib_run.py", "calib"),
                    ("ladder.py", "ladder")):
        assert 'IA.track("%s"' % kind in open(os.path.join(FLEET, f)).read(), f


# -------------------------------------------------------------------------------------------- 4. no pictures: risk
def test_no_picture_limit_follows_the_risk():
    b = (0, 0, 396, 168)
    still = SR.no_picture_limit(200, 84, 0, 0.0, 1.0, b)
    assert still["limit_s"] == float("inf")
    middle = SR.no_picture_limit(200, 84, 0, 10.0, 1.0, b)
    at_edge = SR.no_picture_limit(380, 84, 0, 10.0, 1.0, b)
    away = SR.no_picture_limit(380, 84, 180, 10.0, 1.0, b)
    assert at_edge["limit_s"] < 1.0 < away["limit_s"] < middle["limit_s"]
    unsure = SR.no_picture_limit(380, 84, 0, 10.0, 5.0, b)
    assert unsure["limit_s"] < at_edge["limit_s"]
    assert SR.no_picture_limit(390, 84, 0, 0.0, 1.0, b)["limit_s"] == 0.0     # no room even standing
    assert at_edge["slow_s"] == round(at_edge["limit_s"] * SR.NOPIC_SLOW_SHARE, 2)


def test_freeroam_slows_or_holds_by_each_robots_limit_and_logs_it():
    import freeroam_fastsim as S
    sim = S.Sim(2, 0.04, dead_robot=False, pi_freeze=False, dropouts=False, nopic=(60.0, 25.0))
    sim.run()
    c = sim.ctrl
    assert c.counts.get("nopic_held", 0) + c.counts.get("nopic_slowed", 0) >= 1
    assert sim.stats["falls"] == 0
    assert all(r.__dict__.get("nopic") is None for r in c.R.values())   # released when pictures came back


def test_frame_watch_limit_comes_from_the_stream():
    import frame_watch as FW
    clk = [0.0]
    w = FW.FrameWatch(["overhead"], clock=lambda: clk[0], thread=False, say=lambda s: None)
    assert w.limit_s("overhead") == FW.NO_PICTURE_S                   # until enough gaps are measured
    for _ in range(100):
        clk[0] += 0.08
        w.saw("overhead")
    assert abs(w.limit_s("overhead") - max(FW.GAP_FLOOR_S, FW.GAP_FACTOR * 0.08)) < 1e-6
    clk[0] += 0.5
    assert w.check() == ["overhead"]                                   # 0.5 s is long for a 12-a-second stream


# -------------------------------------------------------------------------------------------- 5. camera discovery
def test_clearance_is_derived_and_said_in_cm_and_inches():
    import camera_discovery as CD
    d = CD.clearance(0.24)
    assert abs(d["radius_cm"] - sum(v for _, v, _ in d["terms"])) < 0.1
    assert 15 < d["radius_cm"] < 25 and abs(d["radius_in"] - d["radius_cm"] / 2.54) < 0.1
    assert "in)" in d["words"] and len(d["terms"]) == 5
    assert CD.clearance(None)["radius_cm"] > d["radius_cm"]           # no camera model: a coarser margin


def test_discovery_finds_coverage_identity_heading_and_breaks_the_guesses():
    import camera_discovery as CD
    for tags in (True, False):
        rec = CD.run_sim(None, tags=tags, log=lambda s: None)
        assert rec["blind"] == ["laptop webcam"]                       # the extra camera that sees nothing: left out
        assert rec["cameras"]["USB 2.0 Camera"]["sees"] == ["Alpha", "Bravo"]   # two robots close together
        assert rec["cameras"]["EMEET C950"]["sees"] == ["Charlie"]
        for n, r in rec["robots"].items():
            tx, ty, th = rec["sim_truth"][n]
            assert math.dist(r["where_cm"], (tx, ty)) < 2.0
            assert abs((r["heading_deg"] - th + 180) % 360 - 180) < 5.0
            assert (r["tag_id"] is not None) == tags
            if tags:
                assert abs(r["tag_turn_deg"]) < 3.0
        notes = " ".join(rec["notes"])
        assert "'C920e' was not found" in notes and "'Logitech BRIO' was not found" in notes   # renamed, missing
        assert rec["roles_found"]["overhead"] == "USB 2.0 Camera"


# -------------------------------------------------------------------------------------------- 5b. no pictures before binding
def test_feed_check_frozen_blank_and_live_pictures(tmp_path):
    import numpy as np
    import camera_feed as CF
    fs = CF.FeedStats(str(tmp_path), ["overhead", "side"])
    rng = np.random.default_rng(1)
    still = rng.integers(0, 255, (120, 160)).astype(np.uint8)
    for k in range(10):
        fs.saw("overhead", still, k)                                   # the same picture again and again: frozen
        fs.saw("side", np.clip(still.astype(int) + rng.integers(-6, 7, still.shape), 0, 255).astype(np.uint8), k)
    fs.write(force=True)
    t = [0.0]
    ok, words, d = CF.check(str(tmp_path), wait_s=1.0, clock=lambda: t[0], sleep=lambda s: t.__setitem__(0, t[0] + s))
    assert ok and words.startswith("WARNING: camera overhead is sending no pictures")   # side is live: carry on
    assert "./run-fleet-pi.sh rec" in words and "USB cable" in words and "carries on with side" in words
    assert not CF.check(str(tmp_path), cams=["overhead"], wait_s=0.0)[0]           # the only camera: refused
    assert d["overhead"]["frozen"] == 9 and d["side"]["new"] == 10
    fs.ref_read("overhead", 2)                                         # a reference tag read proves it live
    fs.write(force=True)
    assert CF.check(str(tmp_path), wait_s=0.0)[0]


def test_a_stalled_stream_is_said_as_the_camera_never_the_robot(tmp_path):
    """The 10 Nov 11:48 school calibration: a real tag bridge on a stream that connects and sends nothing."""
    import socket
    import threading
    import time
    import calib_run as CR
    import camera_feed as CF
    import tag_cam as TCAM
    cam = TCAM.TagCamera((100, 80, 200), (100.01, 80, 0), size=(320, 240), table=(0, 0, 200, 160))
    cal = tmp_path / "overhead.json"
    cam.true_model().save(str(cal))
    tags = tmp_path / "tags.json"
    tags.write_text(json.dumps({"family": "tag25h9", "tag_cm": 8.0, "roof_cm": 9.5, "height_cm": 6.0,
                                "ids": {"tag0": 0}}))
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(4)
    port = srv.getsockname()[1]
    held = []
    threading.Thread(target=lambda: [held.append(srv.accept()) for _ in range(4)], daemon=True).start()
    folder = tmp_path / "run"
    p = subprocess.Popen([sys.executable, os.path.join(FLEET, "tag_bridge.py"), "--tags", str(tags), "--cam",
                          "overhead=127.0.0.1:%d,%s" % (port, cal), "--out", str(folder / "bridge"), "--to",
                          "127.0.0.1:59998", "--max-s", "6", "--no-ref-watch"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, cwd=REPO)
    try:
        ok, words, _ = CF.check(str(folder / "bridge"), wait_s=5.0)
        assert not ok and words.startswith("camera overhead is sending no pictures")

        class _Eye:
            pass
        e = _Eye()
        e.folder = str(folder)
        try:
            CR.feed_refusal(e, wait_s=0.5)
            assert False, "should refuse"
        except CR.BindError as err:
            assert "sending no pictures" in str(err) and "tag faces up" not in str(err)
    finally:
        p.wait(timeout=30)
        srv.close()


def test_discovery_says_a_stalled_camera_sends_no_pictures():
    import camera_discovery as CD
    rig = CD.SimRig(extra_blind=False)
    cams = rig.caminfos()
    cams.append(CD.CamInfo("stalled cam", lambda: None))           # connected, never a picture
    said = []
    rec = CD.discover(cams, sorted(rig.robots), rig.mover, log=said.append)
    assert "stalled cam" in rec["blind"] and rec["cameras"]["stalled cam"]["no_pictures"]
    assert any("camera stalled cam is sending no pictures" in w for w in said)


def test_grabber_returns_none_on_a_stalled_stream():
    import socket
    import camera_discovery as CD
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    g = CD.Grabber("127.0.0.1:%d" % srv.getsockname()[1], timeout=0.5)
    try:
        assert g.grab() is None
    finally:
        g.close()
        srv.close()


# -------------------------------------------------------------------------------------------- 6. tags a preference
def test_corner_tags_prefill_and_the_person_confirms(monkeypatch):
    import tag_job as TJ
    j = TJ.TagJob.__new__(TJ.TagJob)
    j.site, j.d, j.ref_cfg = {}, {"clicks": {}}, {"corners": {"1": 20, "2": 21, "3": 22, "4": 23}}
    j.save = lambda: None
    refs = {"overhead": {"20": [[0, 0], [10, 0], [10, 10], [0, 10]], "23": [[100, 50], [110, 50], [110, 60], [100, 60]],
                         "24": [[5, 5], [6, 5], [6, 6], [5, 6]]}}
    prop = j.corners_from_refs(refs)
    assert prop == {"overhead": {"1": [5.0, 5.0], "4": [105.0, 55.0]}}
    assert j.corner_confirm_mode() == "confirm" and not j._confirmed(prop)
    j.confirm_corners(prop)
    assert j._confirmed(prop) and not j._confirmed({"overhead": {"1": [50.0, 5.0], "4": [105.0, 55.0]}})
    monkeypatch.setenv("MIRLT_CORNER_CONFIRM", "auto")                # the switch: as before e13, no confirm
    assert j.corner_confirm_mode() == "auto"
    assert "confirm=False" in open(os.path.join(FLEET, "camera_selfcheck.py")).read()   # unattended refit unchanged
    page = open(os.path.join(FLEET, "mission_control.html")).read()
    assert "cornerConfirmCard" in page and "/api/tags/corners_confirm" in page


def test_sources_used_each_second_are_logged(tmp_path):
    import source_log as SL
    s = SL.SourceLog(str(tmp_path), ["Alpha", "Bravo"])
    s.tick(100.2)
    for src in ("tag_bridge", "tag_bridge:motion", "tag_bridge:lock", "cam_bridge:lights", "tag_bridge:ml"):
        s.add("Alpha", src)
    s.add(None, "tag_bridge:ml")
    s.tick(101.1)
    s.close()
    rows = list(__import__("csv").DictReader(open(tmp_path / "sources.csv")))
    a = next(r for r in rows if r["robot"] == "Alpha")
    b = next(r for r in rows if r["robot"] == "Bravo")
    assert (a["tags"], a["motion"], a["lock"], a["lights"], a["ml"], a["odometry_only"]) == ("1", "1", "1", "1", "1", "0")
    assert b["odometry_only"] == "1" and any(r["robot"] == "(unnamed)" for r in rows)


def test_trained_detector_source_is_off_without_a_model(tmp_path, monkeypatch):
    import tag_bridge as TB
    br = TB.Bridge.__new__(TB.Bridge)
    br.say, br.event = (lambda s: None), (lambda *a, **k: None)
    monkeypatch.delenv("MIRLT_ML_DETECTOR", raising=False)
    assert br.ml_init(str(tmp_path)) is None                          # off by default
    monkeypatch.setenv("MIRLT_ML_DETECTOR", "auto")
    assert br.ml_init(str(tmp_path)) is None                          # on, but no trained model yet: not used


# -------------------------------------------------------------------------------------------- 7. the detector's labels
def test_autolabel_makes_labels_the_detector_reads_and_evaluates_at_stops(tmp_path):
    sys.path.insert(0, os.path.join(TOOLS, "detector"))
    sys.path.insert(0, os.path.join(TOOLS, "label"))
    import autolabel as AL
    import detcore as DC
    import motion as MO
    run = AL.make_sim_run(str(tmp_path / "run1"), seed=2, n_frames=60)
    st = AL.label_run(run, str(tmp_path / "labels"), log=lambda s: None)["overhead"]
    assert st["labelled"] > 20 and st["labelled"] + st["skipped_unlabelled"] == st["frames"]
    s = DC.read_many([str(tmp_path / "labels")], verbose=False)
    assert s and all(o["cls"] == "alvik" and o["heading"] is not None for x in s for o in x["objects"])
    r = AL.evaluate_stops(run, st["folder"], MO.load, MO.predict)
    assert r["robots"] > 0 and r["rate"] > 0.8 and r["centre_cm"][0] < 2.0


# -------------------------------------------------------------------------------------------- 8. school runs home
def test_school_pull_copies_never_deletes_and_uploads_once(tmp_path):
    import shutil
    if not shutil.which("rsync") or not shutil.which("flock"):
        import pytest
        pytest.skip("rsync / flock not on this machine")
    sch = tmp_path / "school"
    run = sch / "alvik-fleet-capstone" / "logs" / "20261110-101010-mission"
    (run / "run").mkdir(parents=True)
    (run / "run" / "stops.csv").write_text("a\n")
    (run / "rfid_raw.csv").write_text("private\n")
    old = 1.79e9 - 7200
    for p in run.rglob("*"):
        os.utime(p, (old, old))
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    (bin_ / "fakessh").write_text("#!/bin/bash\nwhile [ $# -gt 0 ]; do case \"$1\" in -i|-o|-l) shift 2;; *) break;; "
                                  "esac; done\nshift\ncd %s && eval \"$@\"\n" % sch)
    (bin_ / "fakeup").write_text("#!/bin/bash\necho UPLOAD \"$@\"\n")
    for f in ("fakessh", "fakeup"):
        os.chmod(bin_ / f, 0o755)
    home = tmp_path / "home"
    home.mkdir()
    env = dict(os.environ, HOME=str(home), SCHOOL_SSH=str(bin_ / "fakessh"), SCHOOL_UPLOAD=str(bin_ / "fakeup"),
               SCHOOL_STICK=str(sch / "none"))
    sh = os.path.join(TOOLS, "pi", "school_pull.sh")
    out = subprocess.run(["bash", sh], env=env, capture_output=True, text=True).stdout
    assert "uploaded: 20261110-101010-mission" in out
    got = home / "school-runs" / "logs" / "20261110-101010-mission"
    assert (got / "run" / "stops.csv").exists() and not (got / "rfid_raw.csv").exists()
    (run / "run" / "stops.csv").write_text("b\n")
    out2 = subprocess.run(["bash", sh], env=env, capture_output=True, text=True).stdout
    assert "0 run folder(s) uploaded" in out2 and (got / "run" / "stops.csv").read_text() == "b\n"
    assert list((home / "school-runs" / ".replaced").rglob("stops.csv"))      # the replaced copy kept
    assert (run / "rfid_raw.csv").exists() and (run / "run" / "stops.csv").exists()   # the school end untouched


# -------------------------------------------------------------------------------------------- 2b. the calibration never gives up
class _Eye:
    def __init__(self):
        self.ages, self.loose_ = [], None

    def age(self, tag):
        return self.ages.pop(0) if self.ages else 99.0

    def loose_last(self, tag):
        return self.loose_


def test_calibration_hold_has_no_time_limit_asks_for_a_hand_and_takes_motion_sightings(monkeypatch):
    import threading
    import time
    import calib_run as CR
    cls = next(v for v in vars(CR).values() if isinstance(v, type) and hasattr(v, "hold_never_give_up"))
    r = cls.__new__(cls)
    r.tag, r.name, r.holds, r.D = 4, "AL-EB54", [], (100.0, 50.0, 90.0)
    r.stop_flag = threading.Event()
    r.a = type("A", (), {"lost_policy": "recover"})()
    evs, said, tried = [], [], []
    r.ev = lambda kind, **kw: evs.append(kind)
    r.set_status = lambda **kw: None
    r.recover = lambda est, *a: tried.append(est)
    monkeypatch.setattr(CR, "say", said.append)
    monkeypatch.setattr(CR, "HOLD_SAY_S", 0.2)
    e = _Eye()
    r.eye = e
    e.loose_ = (time.time() + 0.6, 120.0, 60.0, "tag_bridge:motion")   # the motion detector sees it
    t = threading.Thread(target=lambda: setattr(r, "res", r.hold_until_seen()))
    t.start()
    time.sleep(0.9)
    assert t.is_alive()                                              # not ended by a timer
    e.ages = [0.1]                                                   # the tag reads again
    t.join(5)
    assert not t.is_alive() and r.res > 0.8
    assert tried and tried[0][:2] == (120.0, 60.0)                   # the way back tried from the motion sighting
    assert "needs_a_hand" in evs and any("NEEDS A HAND" in s for s in said)
    monkeypatch.setenv("MIRLT_CALIB_GIVEUP", "1")                   # the switch: e12's 30 s end
    monkeypatch.setattr(CR, "LOST_GIVEUP_S", 0.2)
    import pytest
    with pytest.raises(CR.RunEnd):
        r.hold_until_seen()


def test_eye_keeps_motion_and_lock_sightings_apart_and_survives_a_null_heading():
    import calib_run as CR
    src = open(os.path.join(FLEET, "calib_run.py")).read()
    assert 'msg.get("th") is None' in src and "def loose_last" in src and '"--board"' in src


def test_a_turn_that_hunts_is_said_as_not_settled_and_a_robot_still_following_is_named():
    import calib_run as CR
    rb = CR.Robot.__new__(CR.Robot)
    rb.name = "AL-EB54"
    link = type("L", (), {})()
    link.lines = [(0, "AL-EB54", "ip", "D,FOLLOW,run,1,2,R,0,0,0,10.0,20.0,-98.4,0.0,0.0,0,0,0.0,0.0,1")] * 5
    rb.link = link
    d = rb.leg_diag(0, 7)
    assert d["follow_rows"] == 5 and d["leg_records"] == 0 and "still FOLLOWING" in d["words"]
    assert abs(d["own_th"] - (-98.4)) < 1e-6
    src = open(os.path.join(FLEET, "calib_run.py")).read()
    assert "TURN DID NOT SETTLE" in src and 'link.send(rb.ip, "stop")\n            if link.wait_for' in src


def test_readability_is_kept_per_tag_where_the_evidence_differs(tmp_path):
    import calib_run as CR
    import observe as OB
    cls = next(v for v in vars(CR).values() if isinstance(v, type) and hasattr(v, "readmap_tag"))
    r = cls.__new__(cls)
    r.tag, r.parent = 4, None
    r.S = type("S", (), {"L": 396.0, "W": 168.0})()
    r.a = type("A", (), {"dry": False})()
    r.site = {"conf_dir": str(tmp_path)}
    old = OB.ReadMap(396.0, 168.0)
    for _ in range(8):
        old.add(150, 80, False)                                      # tag 4 lost here in earlier runs
    old.save(str(tmp_path / "readmap_tag4.json"))
    m = r.readmap_tag()
    assert m.tries[m.ij(150, 80)] == 4 and not m.readable(150, 80)[0]
    assert r.readmap_tag() is m
    src = open(os.path.join(FLEET, "calib_run.py")).read()
    assert "readability_per_tag" in src and "readmap_tag%s.json" in src


def test_a_dim_room_is_said_from_the_brightness_and_parking_never_holds_the_end(tmp_path):
    import threading
    import calib_run as CR
    cls = next(v for v in vars(CR).values() if isinstance(v, type) and hasattr(v, "light_words"))
    r = cls.__new__(cls)
    r.parent, r.folder = None, str(tmp_path)
    (tmp_path / "bridge").mkdir()
    rows = ["t,camera,mean_grey"] + ["%d,overhead,%.1f" % (k, 120.0) for k in range(40)] + \
        ["%d,overhead,%.1f" % (40 + k, 70.0) for k in range(12)]       # the lights went out
    (tmp_path / "bridge" / "brightness.csv").write_text("\n".join(rows) + "\n")
    w = r.light_words()
    assert w.startswith("the room got darker: the camera's brightness dropped from 120 to 70")
    (tmp_path / "bridge" / "brightness.csv").write_text("\n".join(rows[:41]) + "\n")
    assert r.light_words() == ""                                            # steady light: nothing said
    r._ending, r.stop_flag = True, threading.Event()                        # parking after the run's end
    import pytest
    with pytest.raises(CR.RunEnd):
        r.hold_never_give_up()
    src = open(os.path.join(FLEET, "calib_run.py")).read()
    assert 'tries > 6 and os.environ.get("MIRLT_CALIB_GIVEUP", "0") == "1"' in src


# -------------------------------------------------------------------------------------------- never give up: the blind rule
def _blind_run(rule, start, gap, board=(0, 0, 300, 300)):
    import freeroam_fastsim as S
    old = FC.BLIND_RULE
    FC.BLIND_RULE = rule
    try:
        sim = S.Sim(4, 0.03, dead_robot=False, pi_freeze=False, dropouts=False, board=board,
                    starts={"Center": start}, cam_gaps=[("Center", gap[0], gap[1])])
        c, track = sim.ctrl, []
        orig = c.tick

        def tick(t):
            b = sim.bots["Center"]
            track.append((t, b.x, b.y))
            return orig(t)
        c.tick = tick
        sim.run()
    finally:
        FC.BLIND_RULE = old
    blind = [q for q in track if gap[0] <= q[0] <= gap[0] + gap[1]]
    d = sum(math.dist(a[1:], b[1:]) for a, b in zip(blind, blind[1:]))
    emin = min(min(x - board[0], board[2] - x, y - board[1], board[3] - y) for _, x, y in blind)
    return d, emin, sim.stats["falls"]


def test_blind_robot_far_from_edges_drives_on_and_one_near_an_edge_never_closes_in():
    d_edge, e1, f1 = _blind_run("edge", (150, 150, 0), (30.0, 60.0))
    d_cap, _, _ = _blind_run("cap", (150, 150, 0), (30.0, 60.0))
    assert d_edge > 2 * d_cap and d_edge > 150 and f1 == 0          # 60 s with no camera: it kept roaming
    _d, e2, f2 = _blind_run("edge", (22, 150, 180), (0.5, 60.0))
    assert e2 >= 21.0 and f2 == 0                                      # near the edge: never closer than it started
    for f in ("calib_run.py", "run_plan.py"):
        assert 'MIRLT_BLIND_RULE", "edge"' in open(os.path.join(FLEET, f)).read(), f


def test_a_held_robot_is_placed_by_scripted_clicks_and_its_heading_comes_from_a_safe_drive(tmp_path, monkeypatch):
    """Nick 12:56 / 13:01: blind and held near an edge -> asked; a click (projected through the camera model at roof
    height) sets it and restarts its error sum at the click's derived accuracy; it drives straight as far as that sum
    and its body allow, a second click gives its heading (+- atan(sigma sqrt 2 / drive)); it moves again; clicks and
    the residual at the next camera fix are logged."""
    import freeroam_fastsim as S
    import place_click as PC
    import tag_cam as TCAM
    monkeypatch.setenv("MIRLT_QUESTIONS_DIR", str(tmp_path))
    board = (0, 0, 200, 160)
    cam = TCAM.TagCamera((100, 80, 180), (100.01, 80, 0), size=(1280, 960), hfov_deg=74, table=board)
    model = cam.true_model()
    sim = S.Sim(5, 0.04, dead_robot=False, pi_freeze=False, dropouts=False, board=board,
                starts={"Center": (30, 80, 180)}, cam_gaps=[("Center", 2.0, 60.0)])
    c = sim.ctrl
    asked, done, heads = [], [], []
    c.place_fn = lambda n, p, **kw: (asked.append((c.t, kw.get("stage", "position"))), PC.request(n, p, **kw))
    c.place_residual_fn = lambda n, pl, x, y, res: (done.append(res), PC.residual(
        n, pl["t"], pl["x"], pl["y"], x, y, c.t, driven_cm=pl.get("driven_cm"), res_cm=res))
    orig_ev = c.event
    c.event = lambda r, k, w: (heads.append(w) if k == "heading from two clicks" else None, orig_ev(r, k, w))[1]
    orig = c.tick
    clicked = set()

    def tick(t):
        b = sim.bots["Center"]
        for (t0, stage) in asked:
            if stage not in clicked and t > t0 + 2.0:
                u, v = model.project((b.x, b.y, 6.0))
                PC.click("Center", "overhead", model, u + 1.0, v - 1.0, 6.0, who="test")
                clicked.add(stage)
                if stage == "heading":
                    c.truth_th = b.th                                   # its true heading after the drive (deg)
        a = PC.take_answer("Center")
        if a is not None:
            c.on_place("Center", a)
        return orig(t)
    c.tick = tick
    sim.run()
    stages = [s_ for _, s_ in asked]
    assert stages[:2] == ["position", "heading"], stages
    assert c.counts.get("placed", 0) >= 2 and c.counts.get("heading_clicks") == 1
    rows = list(__import__("csv").DictReader(open(tmp_path / "clicks.csv")))
    c2 = next(r for r in rows if r["how"].startswith("click 2"))
    clk = next(r for r in rows if r["how"] == "click")
    assert 0.3 < float(clk["sigma_cm"]) < 5.0 and float(clk["px_cm"]) < 0.5 and clk["who"] == "test"
    h = float(c2["heading_deg"])
    err = abs((h - c.truth_th + 180) % 360 - 180)
    assert err < 15.0, (h, c.truth_th)                                # the drive was long enough to tell its heading
    assert heads and "+-" in heads[0]
    assert done and any(r["residual_cm"] and r["driven_cm"] for r in rows)
    # (only clicks it had not driven away from teach the aim: learn_aim)


def test_click_aim_learned_only_from_clicks_the_robot_had_not_driven_away_from(tmp_path, monkeypatch):
    import place_click as PC
    monkeypatch.setenv("MIRLT_QUESTIONS_DIR", str(tmp_path))
    assert PC.learn_aim() is None
    for k in range(PC.CLICKS_TO_LEARN):
        PC.residual("A", 0, 0, 0, 0, 0, 1, driven_cm=0.5, res_cm=1.0 + 0.1 * k)
        PC.residual("A", 0, 0, 0, 0, 0, 1, driven_cm=80.0, res_cm=20.0)        # after a long blind drive: not the aim
    assert abs(PC.learn_aim() - 1.9) < 1e-6
