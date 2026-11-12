"""Mission Control build (Nick, 12 Nov 2026): (a) STOP really stops and says "stopping..." until every robot has
confirmed; (b) uploads on by default; (c) the Start screen's live map; the new page ("the table"), the old page behind
a setting; the Pi efficiency monitor. Offline only: fake processes, stubs, no robot, Pi or camera."""
import json
import os
import signal
import subprocess
import sys
import textwrap
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "pi")):
    if p not in sys.path:
        sys.path.insert(0, p)

import mission_control as MC   # noqa: E402
import stop_confirm as SC      # noqa: E402


# ------------------------------------------------------------------------------------------- (a) STOP really stops
LEADER = textwrap.dedent("""
    # A run leader like mission_run that does not pass the Ctrl+C on (busy in a step, or slow under load: the design
    # audit's practice tour carried on for 4 min after STOP). Its child drives the robots and ticks a file.
    import signal, subprocess, sys, time
    child = subprocess.Popen([sys.executable, "-c",
        "import sys, time\\nwhile True:\\n    open(sys.argv[1], 'a').write('.')\\n    time.sleep(0.05)",
        sys.argv[1], "fake_robot"])
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})   # after the child: a mask is inherited
    while True:
        time.sleep(0.2)
""")


def _ticks(p):
    try:
        return os.path.getsize(p)
    except OSError:
        return 0


def _still_ticking(p, wait_s=1.0):
    a = _ticks(p)
    time.sleep(wait_s)
    return _ticks(p) > a


@pytest.fixture
def leader(tmp_path):
    src = tmp_path / "leader.py"
    src.write_text(LEADER)
    return str(src)


def _kill_group(run):
    try:
        os.killpg(run.p.pid, signal.SIGKILL)
    except OSError:
        pass


def test_stop_reaches_the_driver_even_when_the_leader_does_not_pass_it_on(tmp_path, leader, monkeypatch):
    """Fails on the old behaviour: Run.stop sent Ctrl+C to the leader only, and the driver went on."""
    monkeypatch.delenv("MIRLT_STOP_GROUP", raising=False)
    tick = str(tmp_path / "tick")
    run = MC.Run("visit", str(tmp_path / "run"), [sys.executable, leader, tick])
    try:
        for _ in range(50):
            if _ticks(tick) > 3:
                break
            time.sleep(0.1)
        assert _still_ticking(tick, 0.4), "the fake driver should be driving before STOP"
        run.stop()
        time.sleep(1.0)
        assert not _still_ticking(tick, 1.0), "the robots' driver still runs after STOP"
        sw = SC.StopWatch([SC.Job("visit", run.p.pid, run.folder, [{"name": "Alpha"}, {"name": "Bravo"}])],
                          practice=True)
        assert sw.step() is True
        st = sw.state()
        assert st["phase"] == "stopped" and all(r["state"] == "confirmed" for r in st["robots"])
    finally:
        _kill_group(run)


def test_the_old_way_is_kept_behind_its_switch(tmp_path, leader, monkeypatch):
    """MIRLT_STOP_GROUP=0: the leader only, as before (kept, switched off): here the driver is NOT stopped."""
    monkeypatch.setenv("MIRLT_STOP_GROUP", "0")
    tick = str(tmp_path / "tick")
    run = MC.Run("visit", str(tmp_path / "run"), [sys.executable, leader, tick])
    try:
        for _ in range(50):
            if _ticks(tick) > 3:
                break
            time.sleep(0.1)
        run.stop()
        time.sleep(1.0)
        assert _still_ticking(tick, 0.6)
        sw = SC.StopWatch([SC.Job("visit", run.p.pid, run.folder, [{"name": "Alpha"}])], practice=True)
        assert sw.step() is False and sw.state()["phase"] == "stopping"
        assert "Stopping" in sw.state()["words"]
    finally:
        _kill_group(run)


def test_a_driver_that_ignores_ctrl_c_is_forced_after_the_derived_wait(tmp_path):
    stubborn = subprocess.Popen([sys.executable, "-c", "import signal, time\nsignal.signal(signal.SIGINT, "
                                 "signal.SIG_IGN)\nwhile True: time.sleep(0.05)", "fake_robot"],
                                start_new_session=True)
    time.sleep(0.5)                                    # its own command line, after the exec
    try:
        t = [0.0]
        sw = SC.StopWatch([SC.Job("freeroam", stubborn.pid, None, [{"name": "Left"}])], practice=True,
                          escalate_s=1.0, clock=lambda: t[0])
        os.killpg(stubborn.pid, signal.SIGINT)
        assert sw.step() is False                      # still there, inside the wait
        t[0] = 1.1
        sw.step()                                      # Ctrl+C again (ignored)
        t[0] = 2.1
        sw.step()                                      # SIGTERM
        for _ in range(30):
            if stubborn.poll() is not None:
                break
            time.sleep(0.1)
        stubborn.wait(timeout=5)
        assert sw.step() is True
        assert any("SIGTERM" in w for w in sw.state()["forced"])
    finally:
        if stubborn.poll() is None:
            stubborn.kill()


def test_real_robots_must_answer_and_a_silent_one_is_asked_again_never_given_up():
    heard = [{}, {"AL-2B7F": "10.0.0.5"}, {"AL-2B7F": "10.0.0.5"}, {}, {}, {"Left": "10.0.0.6"}]
    sent = []
    t = [0.0]
    sw = SC.StopWatch([SC.Job("visit", None, None, [{"name": "Alpha", "ref": "AL-2B7F", "ip": "10.0.0.5"},
                                                    {"name": "Left", "ref": "AL-1DC0", "ip": "10.0.0.6"}])],
                      practice=False, listen=lambda s: heard.pop(0) if heard else {}, send_stop=sent.append,
                      retry_s=1.0, clock=lambda: t[0])
    assert sw.step() is False
    st = {r["name"]: r for r in sw.state()["robots"]}
    assert st["Alpha"]["state"] == "no answer" and "no answer yet" in st["Alpha"]["words"]
    t[0] = 1.0
    assert sw.step() is False
    st = {r["name"]: r for r in sw.state()["robots"]}
    assert st["Alpha"]["state"] == "confirmed" and st["Left"]["state"] == "no answer"
    for k in range(2, 5):
        t[0] = float(k)
        assert sw.step() is False                   # Left silent: asked again, never "given up"
    assert {r["name"]: r for r in sw.state()["robots"]}["Left"]["tries"] == 5
    t[0] = 5.0
    assert sw.step() is True
    assert sw.state()["phase"] == "stopped" and len(sent) == 6


def test_a_held_log_port_is_said_and_retried():
    sw = SC.StopWatch([SC.Job("visit", None, None, [{"name": "Alpha", "ip": "10.0.0.5"}])], practice=False,
                      listen=lambda s: None, send_stop=lambda ips: None)
    assert sw.step() is False
    assert "held by another program" in sw.state()["robots"][0]["words"]


def test_the_start_screen_says_stopping_then_stopped_never_ended(tmp_path, leader, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("MIRLT_SITE", "home")
    monkeypatch.delenv("MIRLT_STOP_GROUP", raising=False)
    import start_job as SJB
    s = MC.Session(str(tmp_path / "mc"))
    st = SJB.Starter(s, where={"site": "home", "kind": "home", "words": "x"}, practice=True)
    tick = str(tmp_path / "tick")
    st.job = SJB.ProcJob("freeroam", [sys.executable, leader, tick], str(tmp_path / "run"), practice=True)
    st.active, st.active_practice, st.started_t = "freeroam", True, time.time()
    try:
        for _ in range(50):
            if _ticks(tick) > 3:
                break
            time.sleep(0.1)
        r = st.stop_all()
        assert r["stop"]["phase"] in ("stopping", "stopped")
        for _ in range(60):
            out = st.status()
            if out.get("stop", {}).get("phase") == "stopped":
                break
            time.sleep(0.2)
        assert out["stop"]["phase"] == "stopped"
        assert "stopped" in out["words"].lower() and "ended" not in out["words"].lower()
        assert not _still_ticking(tick, 0.6)
        names = [x["name"] for x in out["stop"]["robots"]]
        assert names and all(x["state"] == "confirmed" for x in out["stop"]["robots"])
        for _ in range(20):
            if os.path.exists(os.path.join(st.job.folder, "stop.json")):
                break
            time.sleep(0.2)
        rec = json.load(open(os.path.join(st.job.folder, "stop.json")))
        assert rec["phase"] == "stopped"                 # the run's own folder records the stop
    finally:
        try:
            os.killpg(st.job.p.pid, signal.SIGKILL)
        except OSError:
            pass


# ------------------------------------------------------------------------------------------- (b) uploads on by default
def test_uploads_are_on_by_default_and_an_old_unchosen_false_follows_it(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("MIRLT_SITE", "home")
    assert MC.RUN_DEFAULTS["upload"] is True
    d = tmp_path / "mc"
    d.mkdir()
    (d / "settings.json").write_text(json.dumps({"run": dict(MC.RUN_DEFAULTS, upload=False)}))   # saved whole, before
    S = MC.Session(str(d))
    assert S.settings["run"]["upload"] is True and S.upload_migrated
    # set by hand on the page: kept as set
    MC.mark_by_hand(S.settings, {"run": {"upload": False}})
    S.settings["run"]["upload"] = False
    S.save_settings()
    S2 = MC.Session(str(d))
    assert S2.settings["run"]["upload"] is False and not S2.upload_migrated
    # saving the whole run table unchanged is not "by hand"
    bh = {}
    MC.mark_by_hand({"run": {"upload": True}, "by_hand": bh}, {"run": {"upload": True, "timeout_s": 900}})
    assert bh == {}


def test_robot_runs_get_upload_and_home_free_roam_uploads_after_it_ends(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("MIRLT_SITE", "home")
    import start_job as SJB
    S = MC.Session(str(tmp_path / "mc"))
    st = SJB.Starter(S, where={"site": "home", "kind": "home", "words": "x"}, practice=False)
    got = {}

    class FakeJob:
        def __init__(self, kind, cmd, folder, env=None, practice=False):
            got["cmd"] = cmd
    monkeypatch.setattr(SJB, "ProcJob", FakeJob)
    st.start_home("freeroam", False)
    assert got["cmd"][:2] == ["bash", "-c"] and "freeroam-run" in got["cmd"][2] and "freeroam-upload" in got["cmd"][2]
    S.settings["run"]["upload"] = False
    st.start_home("freeroam", False)
    assert "freeroam-upload" not in " ".join(got["cmd"])
    assert st.upload_state(None, False, False)["state"] == "off"
    S.settings["run"]["upload"] = True
    f = tmp_path / "r"
    f.mkdir()
    assert st.upload_state(str(f), True, False)["state"] == "will upload"
    (f / "stdout.txt").write_text("...\nUploading x and recording y to release t\n")
    assert st.upload_state(str(f), False, False)["state"] == "uploading"
    (f / "stdout.txt").write_text("Uploading x\nAll up: https://github.com/o/r/releases/tag/t\n")
    u = st.upload_state(str(f), False, False)
    assert u["state"] == "uploaded" and u["link"].endswith("/tag/t")
    assert st.upload_state(str(f), False, True)["state"] == "practice"


def test_a_school_run_on_the_robots_passes_upload(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("MIRLT_SITE", "school-conference")
    S = MC.Session(str(tmp_path / "mc"))
    S.others_driving = lambda: ""
    got = {}

    class FakeRun:
        def __init__(self, kind, folder, cmd, env=None):
            got["cmd"] = cmd
            self.folder = folder
            self.p = type("P", (), {"pid": 0})()

        def status(self):
            return {}

        def alive(self):
            return False
    monkeypatch.setattr(MC, "Run", FakeRun)
    S.start_cam_run("visit", "robots", {"extra": []})
    assert "--upload" in got["cmd"]
    S.start_cam_run("visit", "sim-cameras", {"extra": []})
    assert "--upload" not in got["cmd"]                 # practice: nothing uploads


# ------------------------------------------------------------------------------------------- (c) the live map
def test_home_free_roam_files_give_the_live_table(tmp_path):
    import live_view as LV
    f = tmp_path
    now = 1000.0
    (f / "camera_track.csv").write_text("t,robot,x_cm,y_cm,th_deg,arrived\n" + "".join(
        "%.1f,Left,%.1f,40,370,0\n" % (now - 5 + 0.1 * i, 30 + i * 0.1) for i in range(50)) +
        "%.1f,Center,80,60,90,0\n" % (now - 9))
    (f / "follow.csv").write_text("robot,t_recv,run,ms,step_id,state,queued,last_done,last_failed,x_cm,y_cm,th_deg,"
                                  "pos_sd_cm,th_sd_deg\nLeft,1,r,1,0,run,0,0,0,34,40,10,0.8,1\n"
                                  "Center,1,r,1,0,run,0,0,0,82,61,92,3.0,2\n")
    (f / "battery.csv").write_text("t,robot,pct\n1,Left,88\n1,Center,71\n")
    (f / "edge_parked.json").write_text(json.dumps({"needs_hand": [{"words": "Center at the far edge: move it in"}]}))
    rs = {r["name"]: r for r in LV.from_freeroam(str(f), {"Left": "#2952CC", "Center": "#C2410C", "Right": "#7A3EC8"},
                                                 {"Left": "AL-1111"}, now=now)}
    L, C = rs["Left"], rs["Center"]
    assert L["seen"] and abs(L["x"] - 34.9) < 0.01 and L["th"] == 10.0 and L["ref"] == "AL-1111"
    assert L["pm_cm"] == round(2.45 * 0.8, 1) and L["battery"] == "88"
    assert not C["seen"] and C["x"] == 82.0 and "own wheels" in C["source"] and C["needs_hand"]
    assert rs["Right"]["x"] is None                      # not known yet: listed, not drawn


def test_a_tag_run_live_json_gives_the_live_table():
    import live_view as LV
    live = {"robots": {"Alpha": {"cam": [100, 50, 90], "conf": {"plus_minus_cm": 1.8, "source": "one camera"},
                                 "battery": 90, "doing": "drive"},
                       "Bravo": {"cam": None, "own": [200, 60, 0], "held": True,
                                 "conf": {"plus_minus_cm": 6.0, "source": "odometry only", "since_fix_s": 4}}}}
    rs = {r["name"]: r for r in LV.from_live(live, {}, {"Alpha": "AL-1DC0"}, ["Bravo needs a hand at the edge"])}
    assert rs["Alpha"]["seen"] and rs["Alpha"]["pm_cm"] == 1.8 and rs["Alpha"]["ref"] == "AL-1DC0"
    assert not rs["Bravo"]["seen"] and rs["Bravo"]["needs_hand"] and rs["Bravo"]["held"]


def test_the_start_screen_draws_the_live_table():
    html = open(os.path.join(FLEET, "mission_control.html")).read()
    assert "${liveMapCard(st)}" in html and 'api("/api/live")' in html
    src = open(os.path.join(FLEET, "mission_control.py")).read()
    assert 'path == "/api/live"' in src


# ------------------------------------------------------------------------------------------- Part 3: the Pi monitor
def test_the_planner_feed_writes_loop_cameras_and_detector(tmp_path):
    import monitor_feed as MF
    import tagfree as TF
    t = [100.0]
    fd = MF.Feed(str(tmp_path), every_s=2.0, clock=lambda: t[0])

    class Cam:
        def __init__(self, cam, age):
            self.cam, self.latest = cam, (100.0 - age, b"jpg")

    class Src:
        budget = TF.Budget(duty=0.25)

        def summary(self):
            return {"load": self.budget.summary(), "side_off": False}
    Src.budget.done(101.0, 300.0)                     # one call of 300 ms at t=101
    runner = type("R", (), {"frames": type("F", (), {"cams": [Cam("overhead", 0.2), Cam("side", 0.4)]})(),
                            "tagfree": type("T", (), {"source": Src()})()})()
    for k in range(41):
        t[0] = 100.0 + 0.05 * k
        fd.tick(0.004 if k % 10 else 0.030, runner)
    d = json.load(open(tmp_path / "monitor_feed.json"))
    assert d["loop"]["work_ms_max"] == 30.0 and 3.9 < d["loop"]["work_ms_p50"] < 4.1
    assert 49 < d["loop"]["period_ms_mean"] < 51
    assert set(d["cameras"]) == {"overhead", "side"}
    assert d["detector"]["load"]["calls"] == 1 and abs(d["detector"]["share_now"] - 0.075) < 1e-6   # 300 ms / 4 s
    assert TF.Budget(duty=0.25).share_now(0.0) == 0.0


def test_the_core_share_is_a_setting_with_the_old_default(tmp_path, monkeypatch):
    import tagfree as TF
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.delenv("MIRLT_DETECTOR_DUTY", raising=False)
    assert TF.duty_setting(path=str(tmp_path / "none.json"))[0] == TF.DUTY == 0.25
    monkeypatch.setenv("MIRLT_DETECTOR_DUTY", "10")
    assert TF.Budget().duty == 0.10 and TF.Budget().duty_from == "MIRLT_DETECTOR_DUTY"
    monkeypatch.setenv("MIRLT_DETECTOR_DUTY", "0.5")
    assert TF.Budget().duty == 0.5
    monkeypatch.delenv("MIRLT_DETECTOR_DUTY")
    f = tmp_path / "share.json"
    f.write_text(json.dumps({"duty": 0.1}))
    assert TF.duty_setting(path=str(f)) == (0.1, "share.json (the bench's choice)")


def _rows(n, **kw):
    out = []
    for i in range(n):
        r = {"t": str(1000 + 2 * i), "core0": "40", "core1": "30", "core2": "60", "core3": "55", "temp_c": "60",
             "throttled": "0x0", "throttle_now": "", "throttle_since_boot": "", "lag_overhead_s": "0.10",
             "lag_side_s": "0.12", "loop_work_ms_p95": "8", "det_share_pct": "5", "det_cap_pct": "25",
             "mem_avail_mb": "3000", "mem_total_mb": "4000", "disk_free_gb": "20"}
        r.update({k: str(v) for k, v in kw.items()}) if i == n - 1 else None
        out.append(r)
    return out


def test_pi_health_levels_come_from_documented_limits_and_the_runs_own_spread():
    import pi_monitor as PM
    h = PM.health(_rows(5), floor_gb=1.0)
    assert h["level"] == "ok"
    by = lambda h: {i["key"]: i for i in h["items"]}   # noqa: E731
    assert by(PM.health(_rows(5, temp_c=81), floor_gb=1.0))["temp_c"]["level"] == "red"           # documented 80 C
    assert by(PM.health(_rows(5, throttle_now="throttled"), floor_gb=1.0))["throttle"]["level"] == "red"
    assert by(PM.health(_rows(5, lag_overhead_s=1.2), floor_gb=1.0))["lag_overhead"]["level"] == "red"   # firmware 1 s
    assert by(PM.health(_rows(5, lag_overhead_s=0.5), floor_gb=1.0))["lag_overhead"]["level"] == "ok"    # too early
    assert by(PM.health(_rows(40, lag_overhead_s=0.5), floor_gb=1.0))["lag_overhead"]["level"] == "amber"  # own p95
    assert by(PM.health(_rows(5, loop_work_ms_p95=55), floor_gb=1.0))["loop"]["level"] == "red"   # its 50 ms target
    assert by(PM.health(_rows(5, det_share_pct=30), floor_gb=1.0))["detector"]["level"] == "amber"  # over its cap
    assert by(PM.health(_rows(5, disk_free_gb=0.9), floor_gb=1.0))["disk"]["level"] == "red"       # diskguard floor
    r = _rows(5, core1=100)
    r[-2]["core1"] = "100"
    assert by(PM.health(r, floor_gb=1.0))["core1"]["level"] == "red"
    nopi = _rows(3)
    for x in nopi:
        x["throttled"], x["temp_c"] = "", ""
    hh = by(PM.health(nopi, floor_gb=1.0))
    assert hh["throttle"]["level"] == "none" and hh["temp_c"]["level"] == "none"    # not a Pi: said, not an error


def test_picture_lag_is_read_from_the_recorders_picture_time():
    import socket
    import threading
    import pi_monitor as PM
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def serve():
        c, _ = srv.accept()
        c.recv(1024)
        c.sendall(b"HTTP/1.0 200 OK\r\n\r\n--frame\r\nContent-Type: image/jpeg\r\nX-Frame: 9\r\nX-Time: %.4f\r\n\r\n"
                  % (time.time() - 0.3))
        time.sleep(0.2)
        c.close()
    threading.Thread(target=serve, daemon=True).start()
    lag = PM.picture_lag(port)
    srv.close()
    assert lag is not None and 0.25 < lag < 0.8
    assert PM.picture_lag(1) is None                    # nothing answers: blank, never an error


def test_the_monitor_writes_its_csv_and_the_report_section(tmp_path):
    import pi_monitor as PM
    import monitor_feed as MF
    MF.Feed(str(tmp_path), every_s=0).tick(0.003)
    MF.Feed(str(tmp_path), every_s=0).write(time.time())
    p = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(1.6)"])
    smp = PM.Sampler(str(tmp_path), lag_fn=lambda port: 0.12 if port == 8000 else None)
    path = PM.watch(str(tmp_path), pid=p.pid, every_s=0.3, sampler=smp)
    p.wait()
    rows = PM.rows_of(path)
    assert len(rows) >= 3 and rows[0]["lag_overhead_s"] == "0.12" and rows[0]["lag_side_s"] == ""
    assert list(rows[0]) == PM.COLS
    assert os.path.exists(tmp_path / "pi_monitor.png") and "## Pi health" in open(tmp_path / "pi_monitor.md").read()


def test_mission_control_starts_the_monitor_beside_its_runs():
    src = open(os.path.join(FLEET, "mission_control.py")).read()
    assert src.count("monitor_for(folder, self.run.p.pid)") == 2 and 'path == "/api/pi_health"' in src
    assert "monitor_for(self.folder, self.p.pid)" in open(os.path.join(FLEET, "calib_job.py")).read()
    sh = open(os.path.join(os.path.dirname(TOOLS), "run-fleet-pi.sh")).read()
    assert "pi_monitor.py watch --newest 'logs/*-freeroam'" in sh
    assert "self.mfeed.tick(work, self)" in open(os.path.join(FLEET, "freeroam.py")).read()
    assert "self.mfeed.tick(None)" in open(os.path.join(FLEET, "run_plan.py")).read()


# ------------------------------------------------------------------------------------------- Part 2: the new page
def test_the_new_page_is_the_default_and_the_old_one_is_kept_behind_a_setting(tmp_path, monkeypatch):
    monkeypatch.delenv("MIRLT_MC_PAGE", raising=False)
    assert MC.page_for("/") == MC.TABLE_PAGE and MC.page_for("/classic") == MC.PAGE
    assert MC.page_for("/", env={"MIRLT_MC_PAGE": "classic"}) == MC.PAGE
    S = type("S", (), {"start": type("St", (), {"adv": {"classic_page": True}})()})()
    assert MC.page_for("/", S, env={}) == MC.PAGE and MC.page_for("/table", S, env={}) == MC.TABLE_PAGE
    import start_job as SJB
    assert SJB.ADV_DEFAULTS["classic_page"] is False


def test_the_new_page_has_what_nick_asked_for():
    h = open(os.path.join(FLEET, "mission_control_table.html")).read()
    for must in ('id="soundbtn"', "AudioContext", 'href="/classic"', 'id="safepill"', "data-saf=", "/api/pi_health",
                 "/api/live", "/api/place/click", 'class="ref"', "Stopping", "uploadPill", "@media (max-width:600px)"):
        assert must in h, must
    assert "localStorage" in h and "try {" in h          # the sound choice is a per-viewer convenience, guarded


# ------------------------------------------------------------------------------------------- the core-share bench
def test_the_bench_finds_parked_robots_and_picks_the_largest_share_inside_the_spread(tmp_path):
    import detector_share_bench as B
    D = type("D", (), {})

    def det(x, y):
        d = D()
        d.x, d.y = x, y
        return d
    t = [0.0]
    calls = []

    def src(name, now, why, region):
        calls.append(why)
        return [det(30.2, 40.1), det(90.0, 20.0)] if int(now) % 3 == 0 else []
    f = B.Finder(src, clock=lambda: t[0])
    for k in range(12):
        t[0] = float(k)
        f.step()
    sp = f.spots_from_seen()
    assert len(sp) == 2
    f.spots = sp
    for k in range(12, 40):
        t[0] = float(k)
        f.step()
    assert f.finds and max(f.finds) <= 3.0

    def mk(share, lag, loop, thr=""):
        d = tmp_path / ("share-%02d" % share)
        d.mkdir()
        with open(d / "pi_monitor.csv", "w") as fh:
            fh.write("t,lag_overhead_s,loop_work_ms_p95,core1,temp_c,throttle_now\n")
            for i in range(40):
                fh.write("%d,%.3f,%.2f,40,60,%s\n" % (i, lag + 0.01 * (i % 10), loop + 0.2 * (i % 10), thr))
        (d / "finds.json").write_text(json.dumps({"finds_s": [1.0 + share / 50.0] * 5 if share else []}))
    mk(0, 0.10, 6.0)
    mk(10, 0.11, 6.4)
    mk(25, 0.12, 6.6)
    mk(50, 0.25, 12.0)
    p, pick = B.report(str(tmp_path))
    assert pick == 25
    txt = open(p).read()
    assert "PASS" in txt and "fail: picture lag" in txt and "baseline (detector off)" in txt
    saved = B.apply(25, path=str(tmp_path / "share.json"))
    import tagfree as TF
    assert TF.duty_setting(env={}, path=saved)[0] == 0.25
