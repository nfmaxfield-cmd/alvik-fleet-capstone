"""Build e23 (19 Nov 2026): the fixes for Jay-Sun's school runs of 16 and 18 Nov
(docs/results/school-2026-11-18/report.md, causes A to H). Offline only: fake processes, stubs, the planner, and
numbers copied from the real run logs (tests/data/e23_move_checks.json: every "did NOT move" / "not physically
possible" verdict of 18 Nov with the robot's own poses and the camera's pictures round it). No robot, Pi or camera.

Each test fails on the code before e23 (f4acc77) and passes on e23; the build report lists which."""
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "pi"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import camera_discovery as CD   # noqa: E402
import observe as OB            # noqa: E402
import planner as P             # noqa: E402
import run_plan as RP           # noqa: E402
import safety_group as SG       # noqa: E402
import safety_rules as SR       # noqa: E402
import sites as SI              # noqa: E402
import stop_confirm as SC       # noqa: E402
from test_keep_going import coord, production_plan   # noqa: E402  (the coordinator and plan helpers)


# =================================================================================================== A. Stop
def _school_guess():
    """What robots_for_stop gives at school before any robot is named: the site's slots, no address, no WiFi name."""
    return [{"name": "Alpha", "ref": "", "ip": None, "bound": False},
            {"name": "Bravo", "ref": "", "ip": None, "bound": False}]


def test_a_stop_before_the_robots_are_named_ends_after_one_answered_stop():
    """School 16 Nov 17:43 and 18 Nov 16:53: the robots answer as AL-E7DC / AL-EB54, never as Alpha / Bravo. Before
    e23 the watcher asked for Alpha and Bravo every 3 s for ever."""
    sent, t = [], [0.0]
    sw = SC.StopWatch([SC.Job("visit", None, None, _school_guess())], practice=False,
                      listen=lambda s: {"AL-E7DC": "192.168.1.112", "AL-EB54": "192.168.1.126"},
                      send_stop=sent.append, retry_s=3.0, clock=lambda: t[0])
    assert sw.step() is True
    st = sw.state()
    assert st["phase"] == "stopped" and len(sent) == 1
    assert all(r["state"] == "confirmed" and r["unnamed"] for r in st["robots"])
    assert "AL-E7DC at 192.168.1.112" in st["robots"][0]["words"]
    for k in range(1, 20):                      # a minute later: nothing more is sent
        t[0] = 3.0 * k
        assert sw.step() is True
    assert len(sent) == 1


def test_a_held_log_port_is_still_waited_for_and_named_robots_are_still_asked_for_ever():
    sw = SC.StopWatch([SC.Job("visit", None, None, _school_guess())], practice=False, listen=lambda s: None,
                      send_stop=lambda ips: None)
    assert sw.step() is False                   # the port could not be read: nothing is known yet
    heard = [{}, {}, {"AL-1DC0": "10.0.0.6"}]
    t = [0.0]
    sw = SC.StopWatch([SC.Job("visit", None, None, [{"name": "Alpha", "ref": "AL-1DC0", "ip": "10.0.0.6",
                                                     "bound": True}])], practice=False,
                      listen=lambda s: heard.pop(0), send_stop=lambda ips: None, retry_s=1.0, clock=lambda: t[0])
    for k in range(2):
        t[0] = float(k)
        assert sw.step() is False               # a robot of the run's own binding: asked again, never given up
    t[0] = 2.0
    assert sw.step() is True


def test_the_old_watch_is_kept_behind_its_switch(monkeypatch):
    monkeypatch.setenv("MIRLT_STOP_UNNAMED", "wait")
    sw = SC.StopWatch([SC.Job("visit", None, None, _school_guess())], practice=False,
                      listen=lambda s: {"AL-E7DC": "192.168.1.112"}, send_stop=lambda ips: None)
    assert sw.step() is False and sw.state()["phase"] == "stopping"


def test_a_new_start_ends_the_watch_and_the_log_port_is_free(tmp_path):
    """The watcher's thread is stopped and joined by cancel(): no `stop` goes out afterwards and nothing listens."""
    sent = []
    port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    port.bind(("127.0.0.1", 0))
    pnum = port.getsockname()[1]
    port.close()

    def listen(s):                              # as stop_all.listen_for_robots: holds the port for the listen
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", pnum))
        time.sleep(0.2)
        rx.close()
        return {}
    sw = SC.StopWatch([SC.Job("visit", None, None, [{"name": "Alpha", "ref": "AL-1DC0", "ip": "10.0.0.6",
                                                     "bound": True}])], practice=False, listen=listen,
                      send_stop=sent.append, retry_s=0.1)
    sw.start()
    time.sleep(0.5)
    assert sw.state()["phase"] == "stopping" and sent
    assert sw.cancel("a new run was started") is True
    assert not sw.thread.is_alive()
    n = len(sent)
    time.sleep(0.6)
    assert len(sent) == n                       # nothing more after the new Start
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", pnum))                # the port is free
    rx.close()
    st = sw.state()
    assert st["phase"] == "stopped" and "a new run was started" in st["words"]


def _starter(tmp_path, monkeypatch):
    import mission_control as MC
    import start_job as SJB
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("MIRLT_SITE", "school-conference")
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path / "conf"))
    monkeypatch.setenv("MIRLT_HEARD_FILE", str(tmp_path / "conf" / "heard.json"))
    s = MC.Session(str(tmp_path / "mc"))
    return SJB, SJB.Starter(s, where={"site": "school-conference", "kind": "school", "words": "x"}, practice=False)


def test_stop_during_start_up_then_start_again_real_school_site(tmp_path, monkeypatch):
    """The whole chain of 16 Nov 17:43-17:48 and 18 Nov 16:53-17:04 with a fake run: Stop while the run is still
    opening cameras (no binding.json), Stop pressed twice. One watcher, it ends by itself, no `stop` goes out after
    it, and the next Start's hook finds nothing to end and the log port free."""
    SJB, st = _starter(tmp_path, monkeypatch)
    import stop_all as SA
    sends, popens = [], []
    monkeypatch.setattr(SA, "stop_robots", lambda ips, **k: sends.append(list(ips)) or len(ips))
    monkeypatch.setattr(SA, "broadcast_addrs", lambda *a, **k: ["255.255.255.255"])
    monkeypatch.setattr(SA, "listen_for_robots", lambda *a, **k: {"AL-E7DC": "192.168.1.112",
                                                                  "AL-EB54": "192.168.1.126"})
    leader = tmp_path / "leader.py"
    leader.write_text("import time\nwhile True:\n    time.sleep(0.1)\n")
    job = SJB.ProcJob("visit", [sys.executable, str(leader), "mission_run.py"], str(tmp_path / "run"))
    st.job, st.active, st.active_practice, st.started_t = job, "visit", False, time.time()
    real_popen = subprocess.Popen

    def fake_popen(cmd, *a, **k):               # the one-shot stop_all.py and tmux are never really run by a test
        popens.append(cmd)
        return real_popen([sys.executable, "-c", "pass"])
    monkeypatch.setattr(SJB.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(SJB.subprocess, "run", lambda *a, **k: (_ for _ in ()).throw(OSError("no tmux here")))
    try:
        rb = st.robots_for_stop()
        assert [r["name"] for r in rb] == ["Alpha", "Bravo"] and not any(r["bound"] or r["ip"] or r["ref"] for r in rb)
        st.stop_all()
        first = st.stopwatch
        st.stop_all()                           # pressed again
        assert st.stopwatch is first            # one watcher at a time
        for _ in range(80):
            if first.done_t is not None:
                break
            time.sleep(0.1)
        assert first.done_t is not None, first.state()
        assert first.state()["phase"] == "stopped"
        n = len(sends)
        assert n == 1                           # one `stop` round to the broadcast, then silence
        time.sleep(3.5)                         # (before e23: another round every 3 s, for ever)
        assert len(sends) == n
        st.end_stopwatch()                      # the next Start: nothing left to end
        assert st.wait_log_port(port=0, timeout_s=0.5) is True
    finally:
        try:
            os.killpg(job.p.pid, 9)
        except OSError:
            pass


def test_start_refuses_only_while_the_stopped_run_is_still_closing(tmp_path, monkeypatch):
    SJB, st = _starter(tmp_path, monkeypatch)
    sw = SC.StopWatch([SC.Job("visit", None, None, _school_guess())], practice=False, listen=lambda s: None,
                      send_stop=lambda ips: None)
    monkeypatch.setattr(sw, "live_drivers", lambda: [("visit", 1, "python3 /x/run_plan.py --y")])
    st.stopwatch = sw
    with pytest.raises(ValueError) as e:
        st.end_stopwatch()
    assert "still stopping" in str(e.value) and "run_plan.py" in str(e.value)
    monkeypatch.setattr(sw, "live_drivers", lambda: [])
    st.end_stopwatch()
    assert sw.done_t is not None and sw.cancelled


# =================================================================================================== B. cameras
def test_a_camera_that_blocks_on_open_does_not_hang_the_start():
    """School 18 Nov 16:31 and 16:53: the program sat inside cv2.VideoCapture for 4-5 min. The open now runs in a child
    with a time limit; the blocked camera is left out and said."""
    blocker = [sys.executable, "-c", "import sys, time\nprint('ready', flush=True)\ntime.sleep(120)"]
    said = []
    t0 = time.time()
    cams = CD.find_cameras(log=said.append, streams=[("stream-8000", "127.0.0.1:8000")],
                           nodes=[("USB camera", "/dev/video-blocks")], holders=lambda d: [],
                           opener=lambda dev, limit: CD.timed_open(dev, 1.0, child=blocker))
    assert time.time() - t0 < 6.0
    assert [c["name"] for c in cams] == ["stream-8000"]
    txt = "\n".join(said)
    assert "LOOKING FOR CAMERAS" in txt and "CAMERAS FOUND" in txt and "DID NOT OPEN" in txt


def test_only_usb_cameras_nobody_holds_are_opened(tmp_path):
    usb = tmp_path / "sys" / "devices" / "platform" / "usb1" / "1-1" / "1-1:1.0" / "video4linux" / "video0"
    codec = tmp_path / "sys" / "devices" / "platform" / "soc" / "codec" / "video4linux" / "video19"
    for d in (usb, codec):
        d.mkdir(parents=True)
    (usb / "device").symlink_to(usb.parent.parent)
    (codec / "device").symlink_to(codec.parent.parent)
    assert CD.is_usb_node(str(usb)) and not CD.is_usb_node(str(codec))
    # a node another program has open is found from /proc, never by opening it
    proc = tmp_path / "proc"
    (proc / "4242" / "fd").mkdir(parents=True)
    dev = tmp_path / "video0"
    dev.write_text("")
    (proc / "4242" / "fd" / "7").symlink_to(dev)
    (proc / "4242" / "cmdline").write_bytes(b"python3\0tools/pi/camrec.py\0")
    assert CD.held_by(str(dev), proc=str(proc)) == [(4242, "camrec.py")]
    opened, said = [], []
    cams = CD.find_cameras(log=said.append, streams=[("stream-8000", "127.0.0.1:8000")],
                           nodes=[("held", str(dev)), ("free", "/dev/free")],
                           holders=lambda d: CD.held_by(d, proc=str(proc)),
                           opener=lambda d, limit: opened.append(d) or ("ok", 0.7))
    assert opened == ["/dev/free"] and [c["name"] for c in cams] == ["stream-8000", "free"]
    assert any("held by camrec.py" in w for w in said)


def test_the_open_limit_comes_from_measured_opens(tmp_path):
    rec = str(tmp_path / "opens.json")
    assert CD.open_limit_s(rec) == round(CD.OPEN_FACTOR * CD.OPEN_LONGEST_S, 2)     # the school's 153 opens
    CD.note_open(2.5, rec)
    assert CD.open_limit_s(rec) == round(CD.OPEN_FACTOR * 2.5, 2)                   # this Pi's own record raises it
    d = json.load(open(os.path.join(REPO, "docs", "results", "e23-school-fixes", "open_times.json")))
    assert d["n"] >= 150 and abs(d["max"] - CD.OPEN_LONGEST_S) < 0.01


# =================================================================================================== C. the crash
def test_a_robot_that_rejoins_while_a_plan_without_it_is_made_does_not_crash():
    """School 18 Nov 17:09 (KeyError 'Alpha') and 18:31 (KeyError 'Bravo'), run_plan.py line 1526."""
    mm, plan = production_plan()
    c = coord(mm, plan)
    c.rejoin_pending = None
    dec = P.obstacle_decide(c.m, c.plan, c.states(), [], over=dict(c.plan["settings"]), t_now=1.0)
    ob = {"id": "out-Alpha", "shape": "circle", "center": [50, 50], "radius": 10, "source": "robot_out"}
    made_without = RP.Coordinator.without(plan, ["Alpha"])          # planned while Alpha was out ...
    th = threading.Thread(target=lambda: None)
    th.start()
    th.join()
    c.planning = dict(dec=dec, ob=ob, cause=None, improving=False, t_event=RP.now() - 5.1, over=dict(
        c.plan["settings"]), states=c.states(), result=(made_without, []), error=None, cpu=5.08, map_keys=set(),
        thread=th)
    c.deferred = [(None, None, None, False)]                         # ... and Alpha's rejoin asked for a re-plan
    RP.Coordinator.finish_replan(c)                                  # before e23: KeyError 'Alpha'
    assert c.plan is plan and c.abort is None                        # the plan made without it is not used
    assert c.replans[-1]["rejoined_meanwhile"] == ["Alpha"] and c.replans[-1]["kept"] is False
    assert c.obstacles == [(None, None)]                             # planned again, without its own out-obstacle
    assert c.deferred == []
    assert any("rejoined while a plan made without it" in w for w in c.said)


def test_a_crash_in_the_run_is_reported_as_crashed_never_done(tmp_path):
    """Before e23 the summary of a run that died on an exception said "done" (runs 21 and 25 of 18 Nov)."""
    code = ("import sys; sys.path.insert(0, %r)\n"
            "import run_plan as RP\n"
            "RP.start_robots = lambda *a, **k: None\n"
            "def boom(self, timeout_s):\n"
            "    raise KeyError('Alpha')\n"
            "RP.Coordinator.run = boom\n"
            "sys.exit(RP.main(['--map', %r, '--dry-run', '--yes', '--session', %r]))\n") % (
                FLEET, os.path.join(FLEET, "examples", "demo_map.json"), str(tmp_path / "s"))
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=300)
    s = json.load(open(tmp_path / "s" / "run_plan_summary.json"))
    assert s["result"].startswith("crashed: KeyError 'Alpha'"), (s["result"], r.stdout[-2000:], r.stderr[-2000:])
    assert s["crash"]["error"] == "KeyError" and "run_plan" not in s["result"].split("(")[0]
    assert r.returncode != 0 and "CRASHED" in r.stdout and "Result: crashed" in r.stdout


def test_mission_control_says_crashed(tmp_path, monkeypatch):
    SJB, st = _starter(tmp_path, monkeypatch)
    f = tmp_path / "runfolder"
    f.mkdir()
    (f / "mission_summary.json").write_text(json.dumps({"result": "crashed: KeyError 'Alpha' (run_plan.py line 1526)"}))
    assert st.run_result(str(f)).startswith("crashed")
    html = open(os.path.join(FLEET, "mission_control.html")).read()
    assert "st.crashed" in html and "CRASHED and did not finish" in open(os.path.join(FLEET, "start_job.py")).read()


# =================================================================================================== D. the move check
CASES = json.load(open(os.path.join(HERE, "data", "e23_move_checks.json")))["cases"]
PICTURE_AGE_S = 0.3        # the bridge's median picture age on 18 Nov (docs/results/e23-school-fixes/picture_age.json)


def _judge(case):
    """fast_done on one real verdict of 18 Nov: the step as sent, the robot's own poses as reported, the camera's
    pictures as the bridge logged them (each arriving PICTURE_AGE_S after it was taken). True: counted as NOT done."""
    c = RP.Coordinator.__new__(RP.Coordinator)
    n = case["robot"]
    stp = dict(case["step"], type="drive")
    c.plan = {"robots": {n: {"steps": [stp]}}}
    c.logged, c.said = [], []
    c.log = lambda ev, **k: c.logged.append((ev, k))
    c.say = c.said.append
    c.add_obstacle = lambda ob: None
    b = RP.Bot(n)
    t = case["t_done"]
    b.stat = [dict(status="started", t_started=case["t_started"], info="")]
    b.step_pose0 = {0: tuple(case["own_start"])}
    b.pose = tuple(case["own_end"])
    b.follow = {}
    b.cam_hist = [(q[0], q[0] + PICTURE_AGE_S, q[1], q[2], "overhead") for q in case["camera"]
                  if q[0] + PICTURE_AGE_S <= t]
    b.cam_t = b.cam_hist[-1][1] if b.cam_hist else None
    c.bots = {n: b}
    return RP.Coordinator.fast_done(c, b, 0, b.stat[0], t), c


def _short_cm(case):
    """How far short of the step's target the robot's own pose ended, ALONG the step's line (the firmware drives the
    line: a sideways offset is not its to close)."""
    s = case["step"]
    L = math.hypot(s["to"][0] - s["from"][0], s["to"][1] - s["from"][1])
    return ((s["to"][0] - case["own_end"][0]) * (s["to"][0] - s["from"][0]) +
            (s["to"][1] - case["own_end"][1]) * (s["to"][1] - s["from"][1])) / L


def test_the_real_false_verdicts_of_18_nov_are_now_moved_and_the_true_ones_stay():
    """29 verdicts in the logs (19 "did NOT move", 10 "not physically possible"). In 25 the robot's own pose ended at
    the step's target along its line (within 2.1 cm; most within 1 cm): the step happened. In 4 it ended 3 cm or more
    short (three are Bravo at the zone 4 outlet plate at 16:11, 24 cm short): those must still fail."""
    assert len(CASES) == 29 and sum(c["verdict"] == "not_moved" for c in CASES) == 19
    wrong, failed = [], 0
    for k, case in enumerate(CASES):
        truly_failed = _short_cm(case) >= OB.MIN_JUDGE_CM
        failed += truly_failed
        got, _c = _judge(case)
        if got != truly_failed:
            wrong.append((k, case["run"], case["robot"], case["verdict"], round(_short_cm(case), 1), got))
    assert not wrong, wrong
    assert failed == 4


def test_the_camera_votes_only_with_a_picture_newer_than_the_steps_end():
    case = next(c for c in CASES if c["run"].startswith("20261118-180601") and c["verdict"] == "not_moved"
                and c["length_cm"] > 15)
    got, c = _judge(case)
    assert got is False
    b = c.bots[case["robot"]]
    assert b.late_checks and b.late_checks[0]["length_cm"] > 5       # the camera has its say afterwards
    # a picture taken after the step's end that shows the robot where it started: logged as "did not move (late)"
    x0, y0 = b.late_checks[0]["cam0"]
    RP.Coordinator.late_move_check(c, b, (case["t_done"] + 0.4, case["t_done"] + 0.8, x0 + 0.5, y0, "overhead"))
    ev = [k for e, k in c.logged if e == "move_check_late"]
    assert ev and ev[0]["agrees"] is False and c.n_late_not_moved == 1 and not b.late_checks
    # ... and one that shows it at the step's end agrees
    got, c = _judge(case)
    b = c.bots[case["robot"]]
    RP.Coordinator.late_move_check(c, b, (case["t_done"] + 0.4, case["t_done"] + 0.8, case["own_end"][0],
                                          case["own_end"][1], "overhead"))
    assert [k for e, k in c.logged if e == "move_check_late"][0]["agrees"] is True


def test_at_target_is_the_firmwares_window_plus_the_robots_own_doubt():
    assert OB.at_target(0.9) and not OB.at_target(1.5) and OB.at_target(3.0, own_sd_cm=1.0)
    out = OB.move_outcome(17.5, 1.9, wheel_cm=17.4, accel_rolling=False, cam_moved_cm=None, cam_fresh=False,
                          end_off_cm=0.9)
    assert out["verdict"] == "moved" and out["used"] == ["own pose at its target"]
    # the camera with a picture newer than the end still out-votes a pose that only says it arrived
    out = OB.move_outcome(17.5, 1.9, wheel_cm=17.4, accel_rolling=False, cam_moved_cm=0.5, cam_fresh=True,
                          end_off_cm=0.9)
    assert out["verdict"] == "not_moved"


def test_the_old_move_check_is_kept_behind_its_switch(monkeypatch):
    monkeypatch.setattr(RP, "MOVE_CHECK_E23", False)
    case = next(c for c in CASES if c["run"].startswith("20261118-180601") and c["verdict"] == "impossible"
                and c["length_cm"] > 15)
    assert _judge(case)[0] is True                                   # judged on the plan's length, as before


# ------------------------------------------------------------------------------------------- D. planning ahead
def _long_plan(n_each=8):
    xs = [50.0 + 40.0 * k for k in range(n_each)]
    pts = {}
    for k, x in enumerate(xs):
        pts["A%d" % k] = [x, 45.0]
        pts["B%d" % k] = [x, 125.0]
    return production_plan(orders={"Alpha": ["A%d" % k for k in range(n_each)],
                                   "Bravo": ["B%d" % k for k in range(n_each)]}, points=pts)


def test_ahead_split_keeps_the_next_points_and_carries_the_rest():
    items = []
    for k in range(5):
        items += [{"label": "point P%d" % k, "goto": [10.0 * k, 0.0]}, {"label": "at point P%d" % k, "wait_s": 5}]
    items.append({"label": "home", "goto": [0.0, 0.0]})
    now_, later_ = P.ahead_split(items, 2)
    assert [v["label"] for v in now_] == ["point P0", "at point P0", "point P1", "at point P1"]
    assert [v["label"] for v in later_][:2] == ["point P2", "at point P2"] and later_[-1]["label"] == "home"
    assert P.ahead_split(items, 0) == (items, []) and P.ahead_split(items, 5) == (items, [])
    assert P.ahead_split(items[:4] + ["board1"], 1) == (items[:4] + ["board1"], [])     # not a plain point list


def test_a_replan_of_a_long_fixed_order_list_plans_a_few_points_and_loses_none():
    mm, plan = _long_plan(8)
    c = coord(mm, plan)
    dec = P.obstacle_decide(c.m, c.plan, c.states(), [], over=dict(c.plan["settings"], plan_ahead_points=2), t_now=1.0)
    t0 = time.perf_counter()
    newplan, _ = P.obstacle_replan(dec)
    near_s = time.perf_counter() - t0
    for n in ("Alpha", "Bravo"):
        e = newplan["robots"][n]
        planned = [v["label"] for v in e["visit_order"] if isinstance(v, dict) and v["label"].startswith("point ")]
        later = [v["label"] for v in e["later"] if v["label"].startswith("point ")]
        assert len(planned) == 2 and len(later) == 6
        assert planned + later == ["point %s%d" % (n[0], k) for k in range(8)]      # same points, same order
    dec = P.obstacle_decide(c.m, c.plan, c.states(), [], over=dict(c.plan["settings"], plan_ahead_points=0), t_now=1.0)
    t0 = time.perf_counter()
    whole, _ = P.obstacle_replan(dec)
    all_s = time.perf_counter() - t0
    assert "later" not in whole["robots"]["Alpha"]
    assert near_s < all_s                       # (the school Pi: 15 s for 20 + 20 points, 2.5-8.5 s for 4 + 4)
    # the next re-plan rolls on: the done points drop out, the next ones come in, in order
    c._install(newplan, keep={}, first={}, old=None)
    for n in ("Alpha", "Bravo"):
        b = c.bots[n]
        steps = newplan["robots"][n]["steps"]
        last = max(i for i, s in enumerate(steps) if str(s.get("label") or "").startswith("at point %s0" % n[0]))
        for i in range(last + 1):
            b.stat[i]["status"] = "done"
        b.pose = tuple(steps[last]["to"][:2]) + (math.degrees(steps[last]["to"][2]),)
    dec = P.obstacle_decide(c.m, c.plan, c.states(), [], over=dict(c.plan["settings"], plan_ahead_points=2), t_now=30.0)
    nxt, _ = P.obstacle_replan(dec)
    e = nxt["robots"]["Alpha"]
    assert [v["label"] for v in e["visit_order"] if isinstance(v, dict) and v["label"].startswith("point ")] == \
        ["point A1", "point A2"]
    assert [v["label"] for v in e["later"] if v["label"].startswith("point ")] == ["point A%d" % k for k in range(3, 8)]


def test_the_run_is_not_finished_while_a_list_goes_on_and_the_next_points_are_asked_for():
    mm, plan = _long_plan(6)
    c = coord(mm, plan)
    c.args.retry_s = 3.0
    dec = P.obstacle_decide(c.m, c.plan, c.states(), [], over=dict(c.plan["settings"], plan_ahead_points=2), t_now=1.0)
    newplan, _ = P.obstacle_replan(dec)
    c._install(newplan, keep={}, first={}, old=None)
    for n in c.names:
        for st in c.bots[n].stat:
            st["status"] = "done"
    assert RP.Coordinator.finished(c) is False                       # every planned step done, the lists go on
    c.pending_replan = None
    RP.Coordinator.feed_watch(c)
    assert c.pending_replan and "next points" in c.pending_replan[0]
    # the points not planned yet count as not done (a robot that goes out hands them over too)
    left = [g[0]["label"] for g in c.remaining_points(c.plan["robots"]["Alpha"], c.bots["Alpha"].stat)]
    assert left[-1] == "point A5" and "point A2" in left
    assert "point A5" in c.listed_labels("Alpha")


def test_the_run_is_not_done_while_handed_over_points_wait_for_a_replan_that_is_due():
    """Simulator, 19 Nov (late pictures, 20 + 20 points): Bravo was held 253 s in, its 25 points not done went to
    Alpha, which had finished its own; the re-plan for them was due 5 s later (past the re-plan cap) and the run ended
    "done" before it."""
    mm, plan = _long_plan(3)
    c = coord(mm, plan)
    for n in c.names:
        for st in c.bots[n].stat:
            st["status"] = "done"
    assert RP.Coordinator.finished(c) is True
    c.others_done_t = None
    items = [{"label": "point B9", "goto": [60.0, 40.0]}, {"label": "at point B9", "wait_s": 5.0}]
    RP.Coordinator.insert_visits(c, "Alpha", items)                  # what reassign_points does for a held Bravo
    assert RP.Coordinator.handed_points_waiting(c) == {"Alpha": ["point B9"]}
    c.pending_replan = ("re-plan after the cap, spaced", RP.now() + 5.0)
    assert RP.Coordinator.finished(c) is False
    # no re-plan due (it was made and could not place the point), or the point done since: the run can end
    c.pending_replan = None
    assert RP.Coordinator.finished(c) is True
    c.pending_replan = ("again", RP.now() + 5.0)
    c.done_labels = {"at point B9"}
    assert RP.Coordinator.finished(c) is True


def test_points_ahead_come_from_the_measured_planning_cost():
    mm, plan = _long_plan(4)
    c = coord(mm, plan)
    budget = float(c.plan["settings"]["replan_budget_s"])
    a0, c0 = c.PLAN_COST_SEED
    # nothing measured yet: the school Pi's line (0.4 s fixed + 0.36 s a point), two robots
    assert RP.Coordinator.plan_ahead_k(c) == max(c.PLAN_AHEAD_MIN, int((budget - a0) / c0 / 2)) == 2
    # one size measured, far quicker than the Pi (a desk machine): the Pi's line scaled to it
    for _ in range(2):
        RP.Coordinator.note_plan_cost(c, 0.4, 10)
    fixed, per, how = RP.Coordinator.plan_cost(c)
    assert abs(fixed + per * 10 - 0.4) < 1e-6 and "scaled" in how
    assert RP.Coordinator.plan_ahead_k(c) > 10
    # two sizes measured: a fitted line. A machine whose FIXED part alone is over the budget (the grid, built for
    # every re-plan) gains nothing from planning fewer points and pays FIXED at every extra re-plan: the whole list
    c.plan_costs = [(4, 12.4), (10, 13.0), (40, 16.0), (4, 12.5)]
    fixed, per, how = RP.Coordinator.plan_cost(c)
    assert 11.5 < fixed < 12.5 and 0.05 < per < 0.15 and "fitted" in how
    assert RP.Coordinator.plan_ahead_k(c) == c.PLAN_AHEAD_MAX
    # ... and one that is slow per point (the Pi on 18 Nov) plans the few points that fit its budget
    c.plan_costs = [(8, 3.3), (40, 14.8), (8, 3.2), (40, 15.0)]
    fixed, per, how = RP.Coordinator.plan_cost(c)
    assert abs(per - 0.36) < 0.03 and fixed < 0.6
    assert RP.Coordinator.plan_ahead_k(c) == 2
    assert any(e == "plan_ahead" and "fitted" in k["how"] for e, k in c.logged)


def test_no_improvement_try_after_a_replan_that_was_over_its_budget():
    src = open(os.path.join(FLEET, "run_plan.py")).read()
    k = src.index("if IMPROVE_SKIP_OVER and cpu > float(over[\"replan_budget_s\"]):")
    assert "self.improve_at = None" in src[k:k + 700] and src.index("elif newplan[\"summary\"].get(\"first_safe_only\")", k) > k
    assert SG.flag("MIRLT_IMPROVE_SKIP_OVER") and SG.OLD["MIRLT_IMPROVE_SKIP_OVER"] == "0"


# =================================================================================================== E. cameras, points
def test_the_bindings_bridge_reads_every_camera_the_run_uses(tmp_path, monkeypatch):
    """School 18 Nov 14:27: AL-E7DC stood where only camera 2 saw it; the binding read camera 1 alone."""
    import calib_run as CR
    side = tmp_path / "side.json"
    side.write_text("{}")
    a = types.SimpleNamespace(stream="127.0.0.1:8000", calib="x", side_stream="127.0.0.1:8001", side_calib=str(side))
    assert CR.bridge_side_args(a) == ["--cam", "side=127.0.0.1:8001,%s" % side]
    assert CR.bridge_side_args(types.SimpleNamespace(stream="s", calib="x")) == []            # nothing given
    a.side_calib = str(tmp_path / "missing.json")
    assert CR.bridge_side_args(a) == []                                                       # no model: camera 1 alone
    a.side_calib = str(side)
    monkeypatch.setenv("MIRLT_BIND_CAMS", "one")
    assert CR.bridge_side_args(a) == []                                                       # the old way, kept
    import mission_run as MR
    tc = tmp_path / "tagcal"
    tc.mkdir()
    args = types.SimpleNamespace(no_side=False, side_calib=None, side_stream="127.0.0.1:8001")
    assert MR.bind_side(args, str(tc)) is None                       # camera 2 has no model yet
    (tc / "side.json").write_text("{}")
    assert MR.bind_side(args, str(tc)) == ("127.0.0.1:8001", str(tc / "side.json"))
    args.no_side = True
    assert MR.bind_side(args, str(tc)) is None                       # the run does not use camera 2


def test_visit_points_outside_every_camera_are_warned_about_loudly(tmp_path, monkeypatch):
    """School 18 Nov 14:30: Bravo's P26 and P23 were in the half camera 1 does not read; "(kept as asked)" was all
    the run said."""
    import mission_run as MR
    said = []
    monkeypatch.setattr(MR, "say", said.append)

    class RM:                                   # camera 1 reads the near half only
        def readable(self, x, y):
            return (x < 200.0, "")
    a = types.SimpleNamespace(site_=SI.load("school-conference"), visit_order="Alpha:P14,P16;Bravo:P26,P23",
                              visit_keep_unreadable=True, readmap_=RM(), no_side=True)
    got = MR.visit_orders(a, ["Alpha", "Bravo"], str(tmp_path))
    assert got["Bravo"] == ["P26", "P23"]                            # kept, as asked
    warn = [w for w in said if w.startswith("WARNING: POINTS NO CAMERA OF THIS RUN CAN SEE")]
    assert len(warn) == 1 and "Bravo: P26, P23" in warn[0] and "camera 1 only" in warn[0]
    rec = json.load(open(tmp_path / "visit_order.json"))
    assert rec["warning"] == warn[0]
    html = open(os.path.join(FLEET, "mission_control.html")).read()
    assert "st.visit_warning" in html
    monkeypatch.setenv("MIRLT_VISIT_UNSEEN", "quiet")
    said.clear()
    MR.visit_orders(a, ["Alpha", "Bravo"], str(tmp_path))
    assert not [w for w in said if w.startswith("WARNING")]


def _point_coord():
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.logged, c.said = [], []
    c.log = lambda ev, **k: c.logged.append((ev, k))
    c.say = c.said.append
    c.t_run = RP.now() - 100.0
    c.outdir = None
    c.point_csv = lambda rec: None
    c.cam_info = lambda: {"moved": {}}
    c.cam_unc = lambda f: f["unc"]
    return c


def test_a_stop_far_from_its_point_is_not_credited():
    """School 18 Nov 16:11: P29 (261.6, 124.5) was credited to Bravo four times from (263, 100), 24 cm away, each
    "+-1.7 cm"."""
    c = _point_coord()
    b = RP.Bot("Bravo")
    t1 = RP.now()
    b.pose = (263.2, 100.3, 166.0)
    b.fix_log = [dict(t=t1 - 4.0 + 0.3 * k, x=263.0, y=100.0, unc=1.7, cams=["side"]) for k in range(12)]
    ok = RP.Coordinator.record_point(c, b, "point P29", t1 - 5.0, t1, target=[261.6, 124.5])
    assert ok is False and not c.__dict__.get("points")
    r = c.not_reached[0]
    assert r["reached"] is False and abs(r["miss_cm"] - 24.5) < 0.2 and r["reach_tol_cm"] < 6.0
    assert c.stop_misses == [r["miss_cm"]] or abs(c.stop_misses[0] - 24.5) < 0.2
    assert "NOT REACHED" in r["words"] and "24.5 cm" in r["words"]
    # the same stop at the point is credited as before
    b.fix_log = [dict(t=t1 - 4.0 + 0.3 * k, x=262.0, y=123.6, unc=1.7, cams=["side"]) for k in range(12)]
    b.point_target = [261.6, 124.5]                                  # as point_step hands it over
    assert RP.Coordinator.record_point(c, b, "point P29", t1 - 5.0, t1) is True
    assert c.points[0]["reached"] is True and c.points[0]["miss_cm"] < 1.5 and b.point_target is None


def test_a_dwell_finished_away_from_the_point_is_judged_against_the_point_not_the_pause_step():
    """Simulator, 19 Nov (late pictures): Alpha stood at P01 (30.5, 31.8), was sent on a recovery 3 s into its dwell
    and finished the dwell where the recovery left it (40.6, 31.5); the re-plan's pause step was planned there, so the
    stop was 1.6 cm from "its target" and credited, 10 cm from the point."""
    mm, plan = production_plan()
    c = coord(mm, plan)
    c.t_run = RP.now() - 50.0
    c.outdir = None
    c.point_csv = lambda rec: None
    c.cam_info = lambda: {"moved": {}}
    c.cam_unc = lambda f: f["unc"]
    c.said = []
    c.say = c.said.append
    px, py = RP.Coordinator.point_xy(c, "point 1")
    assert [px, py] == [80.0, 120.0] and RP.Coordinator.point_xy(c, "point 99") is None
    b = c.bots["Alpha"]
    t1 = RP.now()
    b.pose = (px + 10.1, py, 0.0)
    b.fix_log = [dict(t=t1 - 0.05, x=px + 10.1, y=py, unc=1.3, cams=["overhead"])]
    b.dwell = None
    stp = {"label": "at point 1", "type": "wait", "to": [px + 8.5, py, 0.0], "t_start": 0.0, "t_end": 0.1}
    RP.Coordinator.point_step(c, b, stp, "done", t1)
    assert not c.__dict__.get("points") and c.not_reached[0]["target"] == [px, py]
    assert abs(c.not_reached[0]["miss_cm"] - 10.1) < 0.1 and "at point 1" not in c.__dict__.get("done_labels", set())
    # the same dwell finished at the point is a visit
    b.pose = (px + 0.4, py, 0.0)
    b.fix_log = [dict(t=t1 - 0.05, x=px + 0.4, y=py, unc=1.3, cams=["overhead"])]
    RP.Coordinator.point_step(c, b, stp, "done", t1)
    assert c.points[0]["reached"] is True and "at point 1" in c.done_labels


def test_the_reach_tolerance_is_derived_and_a_missed_stop_is_not_done():
    c = _point_coord()
    derived = math.hypot(SR.TRACK_P95_CM, SR.stop_cm(None))
    seed = c.MISS_SEED_CM[0] + 3.0 * 1.4826 * c.MISS_SEED_CM[1]        # the school's measured scatter: 5.5 cm
    assert c.reach_tol_cm(1.7) == round(max(derived + 1.7, seed), 1) == 5.5
    assert c.reach_tol_cm(7.2) == round(derived + 7.2, 1)              # a doubtful position has its own wider +-
    d = json.load(open(os.path.join(REPO, "docs", "results", "e23-school-fixes", "real_stop_miss.json")))
    assert d["stops"] == 131 and abs(d["median_miss_cm"] - c.MISS_SEED_CM[0]) < 0.1
    src = open(os.path.join(FLEET, "run_plan.py")).read()
    k = src.index("ok = self.record_point(b, lab[3:], t0, t, interrupted=")
    assert "if ok is not False:" in src[k:k + 400] and "done_labels" in src[k:k + 500]
    # every stop the 18 Nov report lists as credited far from its point is refused at its stated +-
    for miss, pm in ((76.5, 1.6), (39.5, 1.6), (29.1, 2.4), (25.4, 4.6), (24.4, 1.7), (15.6, 1.6), (12.8, 1.7),
                     (11.2, 1.7)):
        assert miss > c.reach_tol_cm(pm)
    # ... and a table whose stops normally scatter wider (this run's own stops) is judged by its own scatter
    c.stop_misses = [3.9, 4.6, 5.2, 5.8, 4.4, 6.1, 5.0, 3.5]
    assert c.reach_tol_cm(1.7) > 6.1 and 24.5 > c.reach_tol_cm(1.7)


def test_the_old_crediting_is_kept_behind_its_switch(monkeypatch):
    monkeypatch.setattr(RP, "POINT_CREDIT_REACHED", False)
    c = _point_coord()
    b = RP.Bot("Bravo")
    t1 = RP.now()
    b.pose = (263.2, 100.3, 166.0)
    b.fix_log = [dict(t=t1 - 1.0, x=263.0, y=100.0, unc=1.7, cams=["side"])]
    assert RP.Coordinator.record_point(c, b, "point P29", t1 - 5.0, t1, target=[261.6, 124.5]) is True


def _fix_coord(pose_rows, max_age_ms=None):
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.logged, c.said = [], []
    c.log = lambda ev, **k: c.logged.append((ev, k))
    c.say = c.said.append
    b = RP.Bot("Alpha")
    b.offset = 1000.0
    b.pose_hist = list(pose_rows)
    b.pose = tuple(pose_rows[-1][1:])
    if max_age_ms is not None:
        b.params["fix_max_age_ms"] = str(max_age_ms)
    return c, b


def test_a_fix_too_old_for_the_robot_is_carried_forward_not_refused():
    """School 18 Nov 17:54, the leg to P33: the bridge stalled, the next picture reached the run 1.7 s old, the robot
    (firmware limit 1 s) refused every fix for the last 40 cm and stopped 11 cm past the point, 8.9 cm from the edge.
    Numbers from that leg: 13 cm/s along -y, the camera 2.6 cm behind the robot's own pose at the picture's time."""
    t = 5000.0
    rows = [(t - 2.0 + 0.1 * k, 313.0, 60.0 - 1.3 * k, -90.0) for k in range(21)]      # its own pose, 10 a second
    c, b = _fix_coord(rows)
    ev = {"t": t - 1.7, "x": 312.8, "y": 60.0 - 1.3 * 3 + 2.6, "sigma": 0.5}
    ms, x, y, th, sig = RP.Coordinator.project_old_fix(c, b, ev, -91.0, t)
    assert abs(y - (rows[-1][2] + 2.6)) < 0.05 and abs(x - 312.8) < 0.05      # the 2.6 cm, at where it is NOW
    assert ms == int(round((rows[-1][0] - b.offset) * 1000))                   # stamped as a fix for its newest pose
    assert sig > 0.5 and abs(th + 91.0) < 0.01                                 # wider +-: 22 cm of its own counting
    assert b.old_fix_n == 1 and any("carried forward" in w for w in c.said)
    # a fresh picture is sent as it is; so is one older than the pose history reaches (never guessed)
    assert RP.Coordinator.project_old_fix(c, b, dict(ev, t=t - 0.3), -91.0, t) is None
    assert RP.Coordinator.project_old_fix(c, b, dict(ev, t=t - 9.0), -91.0, t) is None
    # a robot that says it takes fixes up to 1.5 s old (the e21 firmware) is believed
    c, b = _fix_coord(rows, max_age_ms=1500)
    assert RP.Coordinator.project_old_fix(c, b, dict(ev, t=t - 1.2), -91.0, t) is None
    assert RP.Coordinator.project_old_fix(c, b, dict(ev, t=t - 1.7), -91.0, t) is not None


def test_a_leg_that_ends_inside_the_edge_rules_room_is_finished_slowly():
    """School 18 Nov 17:54: a 62 cm leg at 15 cm/s to P33, 19.7 cm from the long edge."""
    room = SR.slow_end_room_cm(15.0, "tags")
    assert room == round(SR.clearance_cm(SR.TRACK_P95_CM, 15.0, "tags") + 15.0 * 1.0, 1) and 24.0 < room < 28.0
    assert SR.slow_end_cms() == 5.0
    pts = {"NEAR": [313.0, 19.7], "MID": [200.0, 84.0]}
    over = {"cruise_cms": 15.0, "edge_slow_room_cm": room, "edge_slow_cms": SR.slow_end_cms(), "chunk_cm": 10000.0}

    def drives(o):
        import missions as MS
        import production as PR
        m, _ = PR.site_map((396.2, 167.6), ["Alpha"], points=pts)
        mm = MS.build(m, PR.program({"Alpha": ["MID", "NEAR"]}))
        plan = MS.plan_synced(mm, over=dict(o, separation_cm=8.0), t_limit_s=30.0)
        st = plan["robots"]["Alpha"]["steps"]
        k = next(i for i, s in enumerate(st) if str(s.get("label") or "") == "at point NEAR")
        return [s for s in st[:k] if s["type"] == "drive"]
    d = drives(over)
    last = d[-1]
    assert abs(last["to"][0] - 313.0) < 0.1 and abs(last["to"][1] - 19.7) < 0.1 and last["speed"] == 5.0
    L = math.hypot(last["to"][0] - last["from"][0], last["to"][1] - last["from"][1])
    assert 3.0 < L < room                                            # only the part inside the room
    assert d[-2]["speed"] == 15.0                                    # the leg up to the room at the tour's speed
    assert all(s["speed"] == 15.0 for s in drives({"cruise_cms": 15.0, "chunk_cm": 10000.0}))     # off: as before
    far = [s for s in d if abs(s["to"][0] - 200.0) < 0.1 and abs(s["to"][1] - 84.0) < 0.1]
    assert far and far[-1]["speed"] == 15.0                          # a point in the middle of the table: not slowed


# =================================================================================================== F. asleep, card
def test_the_wait_for_sleeping_robots_follows_the_sites_check_in(tmp_path, monkeypatch):
    """School 18 Nov 12:54: robots asleep, the start gave up after 3 min saying "every 10 min"; the school's check-in
    is hourly."""
    import calib_run as CR
    import wake_robots as WR
    import wiggle_bind as WB
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    school, home = SI.load("school-conference"), SI.load("home")
    w, ck = WB.wake_wait_min(school, 3.0)
    assert ck == 60.0 and abs(w - (60.0 + (WR.AWAKE_S + WR.BOOT_S) / 60.0)) < 0.01
    assert WB.wake_wait_min(home, 3.0)[1] == 10.0
    # the Pi put them to sleep 50 min ago: the next check-in is 10 min away, so that is the wait
    WR.note_sleep(None, 60, "idle", now=1000.0)
    w, ck = WB.wake_wait_min(school, 3.0, now=1000.0 + 50 * 60)
    assert abs(w - (10.0 + (WR.AWAKE_S + WR.BOOT_S) / 60.0)) < 0.01
    assert WB.wake_wait_min(school, 3.0, dry=True) == (3.0, None)
    words = CR.checkin_words(60, 42.0)
    assert "every 60 min" in words and "10 min" not in words and "power button" in words
    monkeypatch.setenv("MIRLT_WAKE_WAIT", "fixed")
    assert WB.wake_wait_min(school, 3.0) == (3.0, None)             # the old wait, kept


def test_the_card_full_refusal_says_exactly_how_to_free_space():
    import mission_run as MR
    w = MR.FREE_SPACE_STEPS
    assert "./run-fleet-pi.sh space --move" in w and "cd ~/alvik-fleet-capstone" in w and "nothing is deleted" in w
    assert w.index("1.") < w.index("2.") < w.index("3.") and "press Start again" in w
    assert "FREE_SPACE_STEPS))" in open(os.path.join(FLEET, "mission_run.py")).read()


# =================================================================================================== G. pins
def test_the_schools_typed_pins_match_what_the_robots_say():
    s = SI.load("school-conference")
    pins = s["tags"]["pins"]
    assert pins["AL-E7DC"]["tag"] == 3 and pins["AL-EB54"]["tag"] == 4 and pins["AL-1DC0"]["tag"] == 2
    raw = open(os.path.join(SI.SITES_DIR, "school-conference.json")).read()
    assert "_note_pins_2026_11_19" in raw and "AL-EB54 tag 3, AL-E7DC tag 4" in raw      # the old values are kept
    # ... and they match every two-robot binding of 18 Nov (copied from the runs' binding.json)
    seen = json.load(open(os.path.join(REPO, "docs", "results", "e23-school-fixes", "bindings_18nov.json")))
    assert len(seen["runs"]) >= 9
    for r in seen["runs"]:
        for w, t in r["tags"].items():
            assert pins[w]["tag"] == t, (r["run"], w, t)


def test_a_pin_that_the_wiggle_contradicts_is_logged_and_the_wiggle_wins():
    import wiggle_bind as WB
    expect = {"AL-EB54": 3, "AL-E7DC": 4}                           # the site file's pins before e23
    bound = {"AL-EB54": 4, "AL-E7DC": 3}                            # what turned with each robot
    got = WB.pin_disagreements(expect, bound, {"AL-EB54": "AL-EB54", "AL-E7DC": "AL-E7DC"})
    assert got == [("AL-E7DC", "AL-E7DC", 4, 3), ("AL-EB54", "AL-EB54", 3, 4)]
    assert WB.pin_disagreements(bound, bound) == []
    src = open(os.path.join(FLEET, "wiggle_bind.py")).read()
    assert "PIN DISAGREES" in src and '"pin_disagreed"' in src


# =================================================================================================== the group
def test_every_e23_switch_is_in_the_safety_group_and_its_doc():
    doc = open(os.path.join(REPO, "docs", "how-to", "safety-group.md")).read()
    for env, d, old in (("MIRLT_STOP_UNNAMED", "once", "wait"), ("MIRLT_CAMERA_PROBE", "usb", "e13"),
                        ("MIRLT_MOVE_CHECK", "e23", "e22"), ("MIRLT_IMPROVE_SKIP_OVER", "1", "0"),
                        ("MIRLT_PLAN_AHEAD", "near", "all"), ("MIRLT_POINT_CREDIT", "reached", "always"),
                        ("MIRLT_OLD_FIX", "project", "send"), ("MIRLT_EDGE_SLOW_END", "1", "0"),
                        ("MIRLT_BIND_CAMS", "run", "one"), ("MIRLT_WAKE_WAIT", "site", "fixed"),
                        ("MIRLT_VISIT_UNSEEN", "warn", "quiet")):
        assert SG.DEFAULT[env] == d and SG.OLD[env] == old
        assert "`%s`" % env in doc
