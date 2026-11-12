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

        def status(self):
            return {}

        def alive(self):
            return False
    monkeypatch.setattr(MC, "Run", FakeRun)
    S.start_cam_run("visit", "robots", {"extra": []})
    assert "--upload" in got["cmd"]
    S.start_cam_run("visit", "sim-cameras", {"extra": []})
    assert "--upload" not in got["cmd"]                 # practice: nothing uploads
