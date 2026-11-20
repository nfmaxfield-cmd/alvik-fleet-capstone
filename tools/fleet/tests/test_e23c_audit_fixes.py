"""Build e23c (19 Nov 2026): the fixes for the independent audit of e23 (docs/build-reports/e23-nov19-school-fixes.md,
"Audit of e23"). Offline only: stubs, the real run_plan / stop_confirm / wiggle_bind code, and the 131 stops the school
runs of 16 and 18 Nov recorded (tests/data/e23_real_stops.json, made by docs/results/e23-school-fixes/replay_stops.py
from the run folders). No robot, Pi or camera.

Each test fails on e23 as built (b202ea5) and passes here, except the ones named "kept behind its switch"."""
import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "pi"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import run_plan as RP           # noqa: E402
import safety_group as SG       # noqa: E402
import sites as SI              # noqa: E402
import stop_confirm as SC       # noqa: E402
from test_e23_school_fixes import _fix_coord, _point_coord, _school_guess   # noqa: E402


# ============================================================================== 1. Stop that no robot answered
def _watch(listen, sent, t, **kw):
    return SC.StopWatch([SC.Job("visit", None, None, _school_guess())], practice=False, listen=listen,
                        send_stop=lambda ips: sent.append(t[0]), retry_s=3.0, clock=lambda: t[0], **kw)


def test_a_stop_that_no_robot_answered_is_never_said_as_confirmed_and_is_sent_again():
    """The audit's stop_noanswer.py: no robot answers at all. e23 sent one `stop` and the page said "Stopped: every
    robot confirmed 0 s after STOP." """
    sent, t = [], [0.0]
    sw = _watch(lambda s: {}, sent, t)
    done = []
    for _ in range(40):                              # 20 s, a pass every 0.5 s
        done.append(sw.step())
        t[0] += 0.5
    st = sw.state()
    assert len(sent) == SC.UNNAMED_TRIES == 3 and sent == [0.0, 3.0, 6.0]      # sent again, then no more (cause A)
    assert done[0] is False and done[-1] is True and st["phase"] == "stopped"
    assert "confirmed" not in st["words"] and "no robot answered" in st["words"] and "sent 3 times" in st["words"]
    assert all(r["state"] == "unanswered" and "no robot answered" in r["words"] for r in st["robots"])
    assert any("no robot answered" in e[1] for e in st["events"])
    # while it is still sending, the page says so and counts nobody as confirmed
    sent, t = [], [0.0]
    sw = _watch(lambda s: {}, sent, t)
    sw.step()
    st = sw.state()
    assert st["phase"] == "stopping" and "0 of 2 robots confirmed" in st["words"]
    assert all(r["state"] == "no answer" and "no robot has answered yet" in r["words"] for r in st["robots"])


def test_an_answer_on_a_later_try_settles_it_and_says_who_answered():
    sent, t = [], [0.0]
    heard = [{}, {"AL-E7DC": "192.168.1.112"}]
    sw = _watch(lambda s: heard.pop(0) if heard else {}, sent, t)
    assert sw.step() is False
    t[0] = 3.0
    assert sw.step() is True and len(sent) == 2
    st = sw.state()
    assert all(r["state"] == "confirmed" for r in st["robots"])
    # one robot answered for two slots: the page says how many answered, not "every robot"
    assert "every robot confirmed" not in st["words"] and "1 answered (AL-E7DC)" in st["words"]


def test_settling_after_the_first_stop_is_kept_behind_its_switch(monkeypatch):
    monkeypatch.setenv("MIRLT_STOP_NO_ANSWER", "first")
    sent, t = [], [0.0]
    sw = _watch(lambda s: {}, sent, t)
    assert sw.step() is True and len(sent) == 1                 # as e23 was built: one `stop`, settled
    assert "confirmed" not in sw.state()["words"] and "no robot answered" in sw.state()["words"]   # honest all the same


# ============================================================================== 5. a second Stop against the watcher
def test_a_second_stop_and_the_watcher_share_a_lock():
    """add_jobs (Mission Control's request thread) changed self.robots while the watcher's thread went through it."""
    t = [0.0]
    sw = _watch(lambda s: {}, [], t)
    more = SC.Job("calibrate", 424242, None, [{"name": "Charlie", "ref": "", "ip": None, "bound": False}])
    th = threading.Thread(target=sw.add_jobs, args=([more],), daemon=True)
    with sw.lock:                                    # the watcher is inside a pass
        th.start()
        th.join(0.3)
        assert th.is_alive() and "Charlie" not in sw.robots       # the second Stop waits for the pass to end
    th.join(2.0)
    assert not th.is_alive() and "Charlie" in sw.robots
    # the listen (1 s on the Pi) is made WITHOUT the lock, so a second Stop and the page are not held up by it,
    # and a robot that joined during it is asked on the next pass
    def listen(s):
        sw.add_jobs([SC.Job("run", 434343, None, [{"name": "Delta", "ref": "", "ip": None, "bound": False}])])
        assert sw.lock.acquire(blocking=False) and (sw.lock.release() or True)
        return {}
    sw.listen = listen
    sw.step()
    assert sw.robots["Delta"]["state"] in ("stop sent", "no answer") and sw.state()["phase"] == "stopping"


# ============================================================================== 2. old pictures carried by motion only
def _rows(fn, n, t0=10.0, dt=0.1):
    """Pose rows (laptop time, x, y, th, frame): fn(k, t) -> (x, y, th, corrected_before_this_row)."""
    out, fr = [], 0
    for k in range(n):
        x, y, th, corr = fn(k, t0 + dt * k)
        fr += 1 if corr else 0
        out.append((t0 + dt * k, x, y, th, fr))
    return out


def test_a_correction_since_the_picture_is_not_counted_as_driving():
    """The audit's proj_test.py: a robot STANDING at (100, 50) whose own pose read 95 until a fix corrected it at
    10.5 s. A picture taken at 10.2 s (truth 100, 50) read at 11.5 s was sent as (105, 50) by e23."""
    rows = _rows(lambda k, t: (95.0 if t < 10.5 - 1e-9 else 100.0, 50.0, 0.0, abs(t - 10.5) < 1e-9), 16)
    c, b = _fix_coord(rows)
    ev = {"t": 10.2, "x": 100.0, "y": 50.0, "sigma": 0.5}
    ms, x, y, th, sig = RP.Coordinator.project_old_fix(c, b, ev, 0.0, 11.5)
    assert abs(x - 100.0) < 0.05 and abs(y - 50.0) < 0.05          # it did not drive: the picture's place is its place
    assert abs(th) < 0.01 and sig < 0.6
    assert c.logged[0][0] == "old_fix_carried" and c.logged[0][1]["corrections_removed"] == 1


def test_a_driving_robot_with_a_correction_in_the_window_is_carried_by_its_drive_alone():
    """10 cm/s along +x, corrected by +5 cm in x and -2 cm in y at 10.5 s, heading by 3 deg."""
    def fn(k, t):
        after = t >= 10.5 - 1e-9
        return (20.0 + 10.0 * (t - 10.0) + (5.0 if after else 0.0), 30.0 - (2.0 if after else 0.0),
                3.0 if after else 0.0, abs(t - 10.5) < 1e-9)
    rows = _rows(fn, 16)                                           # to 11.5 s
    c, b = _fix_coord(rows)
    ev = {"t": 10.2, "x": 50.0, "y": 60.0, "sigma": 0.5}           # the camera's place at the picture's time
    ms, x, y, th, sig = RP.Coordinator.project_old_fix(c, b, ev, 90.0, 11.5)
    assert abs(x - (50.0 + 13.0)) < 0.1 and abs(y - 60.0) < 0.1     # 1.3 s at 10 cm/s, nothing else
    assert abs(th - 90.0) < 0.1
    assert ms == int(round((rows[-1][0] - b.offset) * 1000))
    # a pure drive (no correction) is carried exactly as e23 carried it
    rows = _rows(lambda k, t: (20.0 + 10.0 * (t - 10.0), 30.0, 0.0, False), 16)
    c, b = _fix_coord(rows)
    assert abs(RP.Coordinator.project_old_fix(c, b, ev, 90.0, 11.5)[1] - 63.0) < 0.05


def test_nothing_but_corrected_steps_is_not_carried_by_a_guess():
    rows = _rows(lambda k, t: (20.0 + k, 30.0, 0.0, k > 0), 16)     # every step holds a correction
    c, b = _fix_coord(rows)
    ev = {"t": 10.2, "x": 50.0, "y": 60.0, "sigma": 0.5}
    assert RP.Coordinator.project_old_fix(c, b, ev, None, 11.5) is None
    assert c.logged and c.logged[0][0] == "old_fix_not_carried" and b.old_fix_dropped == 1


def _burst(e_cm, monkeypatch=None):
    """The audit's burst_test.py: a 1-D copy of the firmware's fix() (pose.h: Kalman gain, its history shifted with
    the pose) fed by the real project_old_fix. The robot drives +x at 10 cm/s, its own pose e_cm behind the truth;
    4 pictures reach the run 1.6, 1.4, 1.2 and 1.05 s old, 0.2 s apart. Returns own - truth after each."""
    c, b = _fix_coord([(0.0, 0.0, 0.0, 0.0, 0)])
    b.offset, b.pose_hist = 0.0, []
    v, off, var, fr = 10.0, -float(e_cm), 8.0, 0
    hist, out, pending = [], [], False
    arrivals = {round(3.0 + 0.2 * k, 1): a for k, a in enumerate((1.6, 1.4, 1.2, 1.05))}
    t = 0.0
    while t < 3.81:
        truth = v * t
        if pending:                                  # the robot's row after it used a fix: its frame number goes up
            fr, pending = fr + 1, False
        hist.append([t, truth + off])
        b.pose_hist.append((t, truth + off, 0.0, 0.0, fr))
        b.pose = (truth + off, 0.0, 0.0)
        age = arrivals.get(round(t, 1))
        if age is not None:
            tp = t - age
            pr = RP.Coordinator.project_old_fix(c, b, {"t": tp, "x": v * tp, "y": 0.0, "sigma": 0.5}, None, t)
            assert pr is not None
            ms, fx, _fy, _th, sig = pr
            h = min(hist, key=lambda q: abs(q[0] - ms / 1000.0))[1]
            k = var / (var + sig * sig)
            var = (1 - k) * var
            off += k * (fx - h)
            for q in hist:
                q[1] += k * (fx - h)
            pending = True
            out.append(off)
        t = round(t + 0.1, 1)
    return out


def test_a_burst_of_four_old_pictures_does_not_over_correct():
    """e23: own pose 3 cm behind the truth ended 3.0 cm AHEAD of it after the burst, 8 cm behind ended 6.7 cm ahead:
    each picture after the first was carried by the correction the one before it had caused."""
    for e in (3.0, 8.0):
        out = _burst(e)
        assert len(out) == 4
        assert all(-e - 1e-6 <= o <= 0.3 for o in out), out          # toward the truth, never past it
        assert abs(out[-1]) < 0.3, out                               # and it ends at the truth
        assert all(abs(b_) <= abs(a_) + 1e-6 for a_, b_ in zip(out, out[1:])), out


def test_counting_corrections_as_driving_is_kept_behind_its_switch(monkeypatch):
    monkeypatch.setattr(RP, "OLD_FIX_MOTION_ONLY", False)
    assert _burst(3.0)[-1] > 2.5 and _burst(8.0)[-1] > 6.0           # e23 as built (the audit's numbers)
    assert SG.DEFAULT["MIRLT_OLD_FIX_JUMPS"] == "remove" and SG.OLD["MIRLT_OLD_FIX_JUMPS"] == "count"
    assert SG.DEFAULT["MIRLT_OLD_FIX"] == "project" and SG.OLD["MIRLT_OLD_FIX"] == "send"     # the way before e23


def test_the_pose_history_marks_a_used_fix_and_a_pose_reset():
    """The frame number of a pose row goes up when fixes_used changed since the row before, or the Pi sent fpose."""
    c = RP.Coordinator.__new__(RP.Coordinator)
    sent = []
    c.link = type("L", (), {"send_name": lambda self, n, t: sent.append((n, t))})()
    b = RP.Bot("Alpha")
    c.bots = {"Alpha": b}
    b.offset = 0.0
    c.odometer = lambda *a, **k: None
    c.echo_pose = lambda *a, **k: None
    t0 = RP.now()

    def row(k, used, x):
        ms = int((t0 + 0.1 * k) * 1000)
        v = ["R-1", str(ms), "0", "idle", "0", "0", "0", "%.2f" % x, "0.00", "0.00", "0.5", "1.0", str(used), "0",
             "50.0", "0", "0"]
        RP.Coordinator.on_follow(c, b, t0 + 0.1 * k, v)
    row(0, 4, 10.0)
    row(1, 4, 11.0)
    row(2, 5, 14.0)                                   # it used a fix between rows 1 and 2
    row(3, 5, 15.0)
    assert [q[4] for q in b.pose_hist] == [0, 0, 1, 1]
    RP.Coordinator.send(c, "Alpha", "fpose 20.00 0.00 0.00 1.0 3.0")
    assert sent and abs(b.fpose_sent_t - RP.now()) < 1.0
    b.fpose_sent_t = t0 + 0.35                        # the reset went out between rows 3 and 4
    row(4, 5, 20.0)
    row(5, 5, 21.0)
    row(6, 5, 22.0)
    # the row that arrives just after the reset went out may have been made before the robot took it, so the row
    # after it is marked too (a step marked without a correction costs nothing: its drive is read from its neighbours)
    assert [q[4] for q in b.pose_hist][4:] == [2, 3, 3]
    mv = RP.Coordinator.motion_since(c, b, b.pose_hist[0][0])
    assert abs(mv[0] - 6.0) < 0.2 and mv[4] == 3      # six steps of 1 cm; the 3 cm and 5 cm jumps are not driving


# ============================================================================== 3. the stop tolerance
def _real_stops():
    return json.load(open(os.path.join(HERE, "data", "e23_real_stops.json")))["stops"]


def _replay(stops, run=None):
    """The real record_point on the recorded stops of one run (or each run in turn): [(stop, record)]."""
    out, cur, c = [], None, None
    for s in stops:
        if run is not None and s["run"] != run:
            continue
        if s["run"] != cur:
            cur, c = s["run"], _point_coord()
        b = RP.Bot(s["robot"])
        t1 = RP.now()
        b.pose = (s["miss_cm"], 0.0, 0.0)
        b.fix_log = [dict(t=t1 - 1.0, x=s["miss_cm"], y=0.0, unc=s["pm_cm"], cams=["overhead"])]
        ok = RP.Coordinator.record_point(c, b, "point " + s["point"], t1 - 2.0, t1, target=[0.0, 0.0])
        out.append((s, (c.__dict__.get("points") or [None])[-1] if ok else c.not_reached[-1]))
    return out


def test_the_tolerance_does_not_climb_with_misses_real_16_11_run():
    """18 Nov 16:11: Bravo stood 24.4, 24.5, 24.4 and 29.1 cm from P29. e23 refused each, but each widened its
    tolerance (6.0 -> 25.5 cm after the run's 11 stops): a fifth stop 24.4 cm away would have been credited."""
    got = _replay(_real_stops(), "20261118-161142-visit-robots")
    p29 = [(s, r) for s, r in got if s["point"] == "P29"]
    assert len(got) == 11 and len(p29) == 4 and all(r["reached"] is False for _, r in p29)
    tols = [r["reach_tol_cm"] for _, r in got]
    assert max(tols) <= round(RP.Coordinator.REACH_TOL_CAP * 5.52, 1) and max(tols) < 8.4
    # a fifth and a sixth P29 stop at the same place are refused too, and the tolerance stays where it was
    c = _point_coord()
    for s, _ in got + [(dict(p29[0][0]), None), (dict(p29[0][0]), None)]:
        b = RP.Bot("B")
        t1 = RP.now()
        b.pose = (s["miss_cm"], 0.0, 0.0)
        b.fix_log = [dict(t=t1 - 1.0, x=s["miss_cm"], y=0.0, unc=s["pm_cm"], cams=["side"])]
        ok = RP.Coordinator.record_point(c, b, "point " + s["point"], t1 - 2.0, t1, target=[0.0, 0.0])
    assert ok is False and c.not_reached[-1]["reach_tol_cm"] == tols[-1]
    # the generic case of the audit: 3 good stops and 3 far ones gave a 49 cm tolerance
    c = _point_coord()
    c.stop_misses = [2.5, 3.0, 2.8, 20.0, 20.0, 20.0]             # every stop (what e23 learned from)
    c.stop_misses_reached = [2.5, 3.0, 2.8]
    assert RP.Coordinator.reach_tol_cm(c, 1.7) == 5.5


def test_more_doubt_never_makes_a_stop_easier_to_credit_p01_and_p03():
    """18 Nov 15:03 P01: 5.6 cm off with +-10.9 cm; 17:32 P03: 7.0 cm off with +-9.9 cm. e23 added the +- to the
    tolerance (13.8 and 12.8 cm) and credited both."""
    got = _replay(_real_stops())
    by = {(s["run"][9:15], s["point"], s["miss_cm"]): r for s, r in got}
    p01, p03 = by[("150347", "P01", 5.6)], by[("173209", "P03", 7.0)]
    assert p01["reached"] is False and p03["reached"] is False
    assert p01["reach_tol_cm"] <= 8.3 and p03["reach_tol_cm"] <= 8.3
    # a stop whose best position is inside the tolerance but only doubtfully known is not credited, and says why
    c = _point_coord()
    tol, tol_far = RP.Coordinator.reach_tols(c)
    assert (tol, tol_far) == (5.5, 12.6)
    b = RP.Bot("Alpha")
    t1 = RP.now()
    b.pose = (5.0, 0.0, 0.0)
    for pm, want in ((1.7, True), (7.0, True), (9.0, False)):
        b.fix_log = [dict(t=t1 - 1.0, x=5.0, y=0.0, unc=pm, cams=["overhead"])]
        assert RP.Coordinator.record_point(c, b, "point P01", t1 - 2.0, t1, target=[0.0, 0.0]) is want
    assert "only known to" in c.not_reached[-1]["words"] and "14.0 cm away" in c.not_reached[-1]["words"]
    # over all 131 recorded stops: every stop the 18 Nov report lists as far from its point is refused, and no
    # refused stop was within the table's normal scatter with a normal +-
    refused = [(s, r) for s, r in got if r["reached"] is False]
    assert len(got) == 131 and len(refused) == 17
    assert all(s["miss_cm"] > 5.5 or s["miss_cm"] + s["pm_cm"] > 12.6 for s, _ in refused)
    assert sum(1 for s, r in got if s["miss_cm"] >= 8.8 and r["reached"]) == 0
    assert max(r["reach_tol_cm"] for _, r in got) <= 8.3


def test_the_tolerance_is_learned_from_reached_stops_and_bounded_both_ways():
    c = _point_coord()
    d = json.load(open(os.path.join(REPO, "docs", "results", "e23-school-fixes", "real_stop_miss.json")))
    assert tuple(d["seed_miss"]) == c.MISS_SEED_CM and tuple(d["seed_miss_plus_pm"]) == c.MISS_PM_SEED_CM
    # a run whose first reached stops are unusually tight does not narrow it below the table's measured scatter
    c.stop_misses_reached, c.stop_far_reached = [1.0, 1.1, 0.9, 1.0, 1.2, 1.0], [2.5, 2.6, 2.4, 2.5, 2.7, 2.5]
    assert RP.Coordinator.reach_tols(c) == (5.5, 12.6)
    # a table whose reached stops scatter wider widens it, to at most REACH_TOL_CAP times the measured value
    c.stop_misses_reached = [3.9, 4.6, 5.2, 5.4, 4.4, 5.5, 5.0, 3.5]
    assert 5.5 < RP.Coordinator.reach_tols(c)[0] <= 8.3
    c.stop_misses_reached = [1.0, 8.0, 2.0, 8.2, 3.0, 7.5, 4.0, 8.1]
    assert RP.Coordinator.reach_tols(c)[0] == round(1.5 * (2.82 + 3 * 1.4826 * 0.61), 1) == 8.3


def test_the_e23_tolerance_is_kept_behind_its_switch(monkeypatch):
    monkeypatch.setattr(RP, "REACH_TOL_BOUNDED", False)
    got = _replay(_real_stops(), "20261118-161142-visit-robots")
    assert got[-1][1]["reach_tol_cm"] > 15.0                      # it climbs with the misses, as e23 was built
    assert SG.DEFAULT["MIRLT_REACH_TOL"] == "bounded" and SG.OLD["MIRLT_REACH_TOL"] == "e23"


# ============================================================================== 4. a robot missing at the start
class _Link:
    """The robots' log port as calib_run.discover reads it: only AL-E7DC ever answers."""

    def __init__(self, names):
        self.cond, self.lines, self.sent, self.names = threading.Condition(), [], [], names

    def mark(self):
        return len(self.lines)

    def send(self, ip, text):
        self.sent.append((ip, text))
        if text == "status":
            with self.cond:
                for k, n in enumerate(self.names):
                    self.lines.append((time.time(), n, "192.168.1.%d" % (112 + k), "M,status idle"))


def test_a_missing_robot_does_not_freeze_the_start_for_the_whole_check_in(monkeypatch):
    """The audit's wake_test.py: at school the start waited up to 60.5 min whenever fewer robots answered than it
    looked for, saying nothing about which. Now the rest are waited for the usual time and the run starts."""
    import calib_run as CR
    import wiggle_bind as WB
    monkeypatch.setattr(CR.time, "sleep", lambda s: None)
    clock = [1000.0]

    def fake_time():
        clock[0] += 1.0                               # every look at the clock is a second later
        return clock[0]
    monkeypatch.setattr(CR.time, "time", fake_time)
    monkeypatch.setattr(CR, "say", lambda *a, **k: None)
    link = _Link(["AL-E7DC"])
    found = CR.discover(link, [], 60.5, False, count=2, checkin_min=60.0, partial_min=3.0)
    assert list(found) == ["AL-E7DC"]
    assert 180.0 <= clock[0] - 1000.0 < 240.0         # the usual 3 min, not 60.5
    # nobody answers: the long wait stands (a run needs a robot)
    clock[0] = 1000.0
    assert CR.discover(_Link([]), [], 5.0, False, count=2, checkin_min=60.0, partial_min=3.0) == {}
    assert clock[0] - 1000.0 >= 300.0
    # without partial_min (MIRLT_WAKE_PARTIAL=wait, and every other caller): the whole wait, as e23 was built
    clock[0] = 1000.0
    assert list(CR.discover(_Link(["AL-E7DC"]), [], 5.0, False, count=2, checkin_min=60.0)) == ["AL-E7DC"]
    assert clock[0] - 1000.0 >= 300.0
    # what the start says: how many, which pinned robots did not answer, and what it means
    school = SI.load("school-conference")
    w = WB.missing_words(school, {"AL-E7DC": "192.168.1.112"}, 2, 60.0)
    assert w.startswith("STARTING WITHOUT 1 ROBOT: 1 of 2 answered (AL-E7DC)")
    assert "AL-EB54" in w and "every 60 min" in w and "NOT driven and NOT tracked" in w
    assert WB.missing_words(school, {"a": 1, "b": 2}, 2, 60.0) == "" and WB.missing_words(school, {}, 2, 60.0) == ""
    src = open(os.path.join(FLEET, "wiggle_bind.py")).read()
    assert "partial_min=partial_min" in src and 'SG_value("MIRLT_WAKE_PARTIAL") != "wait"' in src


def test_the_wake_switches_say_what_the_code_does():
    import wake_robots as WR
    import wiggle_bind as WB
    rows = {s[0]: s for s in SG.SWITCHES}
    words = rows["MIRLT_WAKE_WAIT"][4]
    w, ck = WB.wake_wait_min(SI.load("school-conference"), 3.0)
    assert abs(w - 60.5) < 0.01 and "up to 60.5 min" in words and "waits its usual time" not in words
    assert abs(WR.wait_limit_s(10) / 60.0 - 10.5) < 0.01 and "10.5 min" in words
    assert SG.DEFAULT["MIRLT_WAKE_PARTIAL"] == "go" and SG.OLD["MIRLT_WAKE_PARTIAL"] == "wait"
    doc = open(os.path.join(REPO, "docs", "how-to", "safety-group.md")).read()
    for env in ("MIRLT_WAKE_PARTIAL", "MIRLT_STOP_NO_ANSWER", "MIRLT_OLD_FIX_JUMPS", "MIRLT_REACH_TOL"):
        assert "`%s`" % env in doc and rows[env][4] in doc
    assert words in doc and rows["MIRLT_STOP_UNNAMED"][4] in doc
    assert "not the whole way before e23" in rows["MIRLT_STOP_UNNAMED"][4]


# ============================================================================== 6. chosen numbers are marked
def test_the_numbers_the_audit_named_are_marked_chosen_in_the_code():
    rp = open(os.path.join(FLEET, "run_plan.py")).read()
    pl = open(os.path.join(FLEET, "planner.py")).read()
    sc = open(os.path.join(FLEET, "stop_confirm.py")).read()
    for src, name in ((rp, "LATE_MOVED_SHARE = 0.5"), (rp, "LATE_CHECK_KEEP_S = 5.0"), (pl, "SLOW_SPLIT_MIN_CM = 5.0"),
                      (rp, "POSE_HOLE_S = 1.0"), (rp, "PLAN_COST_KEEP_N = 20"), (rp, "PLAN_FIT_MIN_N = 3"),
                      (rp, "PLAN_FIT_MIN_SPREAD = 2"), (rp, "REACH_TOL_CAP = 1.5"), (rp, "MISS_OWN_N = 5")):
        k = src.index(name)
        assert "[chosen]" in src[k:k + 140], name
        assert "DOE" in src[k:k + 900], name
    k = sc.index("UNNAMED_TRIES = 3")
    assert "[chosen]" in sc[k - 600:k] and "DOE" in sc[k - 600:k]
    report = open(os.path.join(REPO, "docs", "build-reports", "e23-nov19-school-fixes.md")).read()
    for name in ("LATE_MOVED_SHARE", "LATE_CHECK_KEEP_S", "SLOW_SPLIT_MIN_CM", "POSE_HOLE_S", "PLAN_COST_KEEP_N",
                 "PLAN_FIT_MIN_N", "PLAN_FIT_MIN_SPREAD", "REACH_TOL_CAP", "MISS_OWN_N", "UNNAMED_TRIES"):
        assert "`%s`" % name in report, name
