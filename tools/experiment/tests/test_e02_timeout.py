"""Session timeouts (Nick, 2026-10-26; e02_timeout.py): an unattended session always ends, saves and uploads.
A hold nobody answers ends the session after --answer-timeout-min (30), or --after-end-timeout-min (10) once the
planned time is up; the hard cap (planned + 30 min) wraps up; questions with their own default keep it; the live
page shows when no answer ends the session; run-e02-pi.sh's runner window uploads after any end. No robots, no camera:
stand-in runners."""
import json
import os
import re
import sys
import threading
import time
import types
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import e02 as E  # noqa: E402
import e02_plan as PL  # noqa: E402
import e02_timeout as TO  # noqa: E402
import e02_view as V  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))


# --- the clock's rules ---------------------------------------------------------------------------------------------
def test_during_the_session_30_min():
    c = TO.SessionClock(1000.0, 2.0)                       # planned end 1000 + 7200
    t, key = c.wait_deadline(2000.0)
    assert (t, key) == (2000.0 + 1800.0, TO.NO_ANSWER)
    assert c.why(key) == "no answer for 30 min"


def test_after_the_planned_end_10_min():
    c = TO.SessionClock(0.0, 1.5)                          # 21:14 + 1.5 h, as on 2026-10-25
    end = 1.5 * 3600
    assert c.wait_deadline(end + 300.0) == (end + 300.0 + 600.0, TO.NO_ANSWER_AFTER_END)
    # asked 5 min before the planned end: 10 min after the end comes before 30 min after asking
    assert c.wait_deadline(end - 300.0) == (end + 600.0, TO.NO_ANSWER_AFTER_END)
    # asked 25 min before: 30 min after asking comes first
    assert c.wait_deadline(end - 1500.0) == (end + 300.0, TO.NO_ANSWER)
    assert c.why(TO.NO_ANSWER_AFTER_END).startswith("no answer for 10 min after the planned end")


def test_the_hard_cap_comes_first_late_on():
    c = TO.SessionClock(0.0, 1.0)
    cap = 3600.0 + 1800.0
    assert c.cap == cap
    assert c.wait_deadline(cap - 120.0) == (cap, TO.CAP)
    assert "planned time + 30 min" in c.why(TO.CAP)


def test_own_default_is_kept():
    """A question that takes its default after 15 min keeps it; only the hard cap can come first."""
    c = TO.SessionClock(0.0, 1.0)
    assert c.wait_deadline(100.0, own_timeout_s=900.0) == (None, None)
    cap = c.cap
    assert c.wait_deadline(cap - 60.0, own_timeout_s=900.0) == (cap, TO.CAP)


def test_zero_switches_a_rule_off():
    c = TO.SessionClock(0.0, 1.0, answer_min=0, after_end_min=0, cap_min=0)
    assert c.cap is None and c.wait_deadline(10.0) == (None, None)
    c2 = TO.SessionClock(0.0, 1.0, answer_min=0, after_end_min=10, cap_min=0)
    assert c2.wait_deadline(10.0) == (3600.0 + 600.0, TO.NO_ANSWER_AFTER_END)


def test_page_words():
    c = TO.SessionClock(0.0, 1.0)
    t = time.mktime((2026, 10, 25, 23, 5, 0, 0, 0, -1))
    assert c.page_words(t) == "No answer by 23:05 ends the session and uploads"
    assert c.page_words(t, uploads=False) == "No answer by 23:05 ends the session"


def test_options_and_defaults():
    a = types.SimpleNamespace(hours=1.5)
    c = TO.from_args(a, 0.0)
    assert (c.answer_min, c.after_end_min, c.cap_min) == (30.0, 10.0, 30.0)
    src = open(os.path.join(os.path.dirname(HERE), "e02.py")).read()
    for opt in ("--answer-timeout-min", "--after-end-timeout-min", "--hard-cap-min"):
        assert opt in src
    # the DOE notes say no DOE is needed for these convenience settings
    tsrc = open(os.path.join(os.path.dirname(HERE), "e02_timeout.py")).read()
    assert tsrc.count("no DOE needed") >= 4


# --- a stand-in runner ---------------------------------------------------------------------------------------------
class _View:
    def __init__(self):
        self.lock = threading.Lock()
        self.question = None
        self.answers = {}
        self.seen = []

    def log(self, line):
        pass


def _runner(tmp_path, clock, view=True, on_pi=True):
    r = E.Runner.__new__(E.Runner)
    r.a = types.SimpleNamespace(auto=False, web_ask=view, dry_run=False, on_pi=on_pi)
    st = PL.State(str(tmp_path))
    st["robots"] = ["Left", "Center", "Right"]
    r.st = st
    r.dir = str(tmp_path)
    r.said, r.sent, r.marks = [], [], []
    r.say = lambda m, quiet=False: r.said.append(m)
    r.mark = lambda t: r.marks.append(t)
    r.link = types.SimpleNamespace(send_name=lambda n, c: r.sent.append((n, c)), robots=lambda: [])
    r.view = _View() if view else None
    r.clock = clock
    r.end_lock = threading.Lock()
    r.ended = threading.Event()
    r.end_reason = None
    r.timed_out = False
    r.wrap_up = False
    r.hold = threading.Event()
    r.hold_lock = threading.Lock()
    r.hold_owner = None
    r.hold_answer = None
    r.last_hold_end = 0.0
    r.stop_requested = False
    r.held_out = set()
    r.hold_auto_refind = False
    r.tracker = None
    r.order = "5"
    r.interrupted = []
    r.interrupt_main = lambda: r.interrupted.append(time.time())
    return r


def _in_thread(fn, *a):
    box = []
    th = threading.Thread(target=lambda: box.append(fn(*a)), daemon=True)
    th.start()
    return th, box


def test_unanswered_hold_ends_the_session(tmp_path):
    """A hold nobody answers: every robot stopped, the session ends (s), the reason logged and saved."""
    c = TO.SessionClock(time.time(), 2.0, answer_min=0.01)          # 0.6 s
    r = _runner(tmp_path, c)
    th, box = _in_thread(r.hold_for_nick, "Right", "lost", "the camera lost it for 3 s")
    seen = None
    for _ in range(40):
        with r.view.lock:
            if r.view.question:
                seen = dict(r.view.question)
                break
        time.sleep(0.05)
    th.join(10)
    assert box == ["s"]
    assert seen and seen["ends_words"].startswith("No answer by ") and seen["ends_words"].endswith("and uploads")
    assert r.stop_requested and r.timed_out
    assert r.end_reason["key"] == TO.NO_ANSWER and r.end_reason["why"] == "no answer for 0.01 min"
    saved = json.load(open(os.path.join(str(tmp_path), "e02_state.json")))
    assert saved["session_end"]["key"] == TO.NO_ANSWER
    assert saved["holds"][-1][5] == "timeout"
    assert {n for n, cmd in r.sent if cmd == "stop"} == {"Left", "Center", "Right"}
    assert any("e02 session timeout no_answer" in m for m in r.marks)
    assert any("e02 hold timeout Right" in m for m in r.marks)
    assert not r.hold.is_set()


def test_hold_after_the_planned_end_uses_10_min(tmp_path):
    c = TO.SessionClock(time.time() - 7200.0, 1.0, answer_min=30, after_end_min=0.01, cap_min=120)   # planned end 1 h ago
    r = _runner(tmp_path, c)
    th, box = _in_thread(r.hold_for_nick, "Left", "border", "near the edge")
    th.join(10)
    assert box == ["s"] and r.end_reason["key"] == TO.NO_ANSWER_AFTER_END


def test_answered_hold_does_not_end(tmp_path):
    c = TO.SessionClock(time.time(), 2.0, answer_min=0.05)          # 3 s
    r = _runner(tmp_path, c)
    th, box = _in_thread(r.hold_for_nick, "Right", "lost", "lost")
    for _ in range(40):
        with r.view.lock:
            if r.view.question:
                r.view.answers[r.view.question["id"]] = "c"
                break
        time.sleep(0.05)
    th.join(10)
    assert box == ["c"] and not r.stop_requested and r.end_reason is None


def test_question_with_own_default_keeps_it(tmp_path):
    """A setup question that takes its default (here after 0.3 s): the default, the session goes on."""
    c = TO.SessionClock(time.time(), 2.0, answer_min=0.001)         # would end at 0.06 s if it applied
    r = _runner(tmp_path, c)
    assert r.web_ask("Carry on? (y/n) ", "y", 0.3) is None
    assert not r.stop_requested and r.end_reason is None


def test_question_without_default_ends_in_main_thread(tmp_path):
    """ask_timeout with no limit (None): the session timeout ends it like Ctrl+C in the main thread."""
    c = TO.SessionClock(time.time(), 2.0, answer_min=0.005)
    r = _runner(tmp_path, c)
    with pytest.raises(KeyboardInterrupt):
        r.ask_timeout("Press Enter when it is back: ", None)
    assert r.end_reason["key"] == TO.NO_ANSWER and "Press Enter" in r.end_reason["waiting_on"]


def test_hard_cap_ends_an_open_question(tmp_path):
    c = TO.SessionClock(time.time() - 3600.0 - 1790.0, 1.0)         # the cap is 10 s from now
    r = _runner(tmp_path, c)
    t, key = r.session_deadline(time.time(), 900.0)
    assert key == TO.CAP and abs(t - c.cap) < 1e-6


def test_console_wait_times_out(tmp_path, monkeypatch):
    """The laptop run (no live page): a console question nobody answers ends the session too."""
    c = TO.SessionClock(time.time(), 2.0, answer_min=0.005)
    r = _runner(tmp_path, c, view=False)
    gate = threading.Event()
    monkeypatch.setattr("builtins.input", lambda p="": (gate.wait(5), "late")[1])
    assert r.console_wait("Anything? ") is None
    gate.set()
    assert r.timed_out and r.end_reason["key"] == TO.NO_ANSWER


def test_cap_watch_wraps_up_then_stops(tmp_path):
    """At the cap: no new run (wrap_up), then, if the runner has not ended within the grace, every robot stopped."""
    c = TO.SessionClock(time.time() - 3600.0 - 1800.0, 1.0)         # the cap is now
    r = _runner(tmp_path, c)
    r.cap_watch(grace_s=0.2)
    assert r.wrap_up and r.timed_out and r.stop_requested and r.end_reason["key"] == TO.CAP
    assert any(cmd == "stop" for _, cmd in r.sent)


def test_cap_watch_gentle_when_the_run_finishes(tmp_path):
    c = TO.SessionClock(time.time() - 3600.0 - 1800.0, 1.0)
    r = _runner(tmp_path, c)
    threading.Timer(0.1, r.ended.set).start()                        # the session loop ended at the next run
    r.cap_watch(grace_s=3.0)
    assert r.wrap_up and not r.timed_out and not r.stop_requested


def test_ensure_end_interrupts_a_stuck_runner(tmp_path):
    r = _runner(tmp_path, TO.SessionClock(time.time(), 1.0))
    r.ensure_end(wait_s=0.1)
    assert len(r.interrupted) == 1
    r2 = _runner(tmp_path, TO.SessionClock(time.time(), 1.0))
    r2.ended.set()
    r2.ensure_end(wait_s=0.1)
    assert r2.interrupted == []


def test_first_reason_wins(tmp_path):
    r = _runner(tmp_path, TO.SessionClock(time.time(), 1.0))
    r.note_end(TO.CAP, "hard cap reached", parked=True)
    r.session_timeout(TO.NO_ANSWER, time.time(), "a hold")
    assert r.end_reason["key"] == TO.CAP and r.timed_out and r.stop_requested


def test_hold_after_timeout_stops_at_once(tmp_path):
    r = _runner(tmp_path, TO.SessionClock(time.time(), 1.0))
    r.session_timeout(TO.NO_ANSWER, time.time(), "x")
    th, box = _in_thread(r.hold_for_nick, "Left", "lost", "lost")
    th.join(5)
    assert box == ["s"]


# --- the live page -------------------------------------------------------------------------------------------------
class _R:
    phase, order, tracker, feed, st, cam, board = "setup", "", None, None, {}, None, None


def test_page_shows_the_countdown():
    v = V.LiveView(_R(), port=18097, bind="127.0.0.1")
    v.health = lambda: None
    v.start()
    try:
        with v.lock:
            v.question = dict(id=1, prompt="HOLD (lost robot): ...", default="", timeout_s=None, ends_at=123.0,
                              ends_words="No answer by 23:05 ends the session and uploads")
        s = None
        for _ in range(40):                    # the server may take a moment to listen
            try:
                s = json.loads(urllib.request.urlopen("http://127.0.0.1:18097/status.json", timeout=2).read())
                break
            except OSError:
                time.sleep(0.1)
        assert s["question"]["ends_words"] == "No answer by 23:05 ends the session and uploads"
        html = urllib.request.urlopen("http://127.0.0.1:18097/", timeout=2).read().decode()
        assert 'id="hend"' in html and 'id="askend"' in html and "no time limit" not in html
    finally:
        v.stop()


# --- run-e02-pi.sh -------------------------------------------------------------------------------------------------
def test_runner_window_uploads_after_any_end():
    """The runner window: e02.py, then upload_session.py --stop-recorder joined by ';' (runs whatever the exit
    code); production: the logs-only upload, also by ';'."""
    sh = open(os.path.join(REPO, "run-e02-pi.sh")).read()
    lines = [ln for ln in sh.splitlines() if "tmux new-" in ln and "e02.py --on-pi" in ln]
    assert len(lines) == 2
    for ln in lines:
        m = re.search(r"e02\.py --on-pi \$RUN_ARGS(.*?)upload_session\.py", ln)
        assert m and "&&" not in m.group(1) and "||" not in m.group(1), ln
    assert any("--stop-recorder" in ln for ln in lines)
    # the runner leaves at once after main() (os._exit), whatever thread is still waiting on a console
    src = open(os.path.join(os.path.dirname(HERE), "e02.py")).read()
    assert "os._exit(int(_code or 0))" in src


def test_recorder_pause_ends_like_a_hold(tmp_path, monkeypatch):
    """The recorder check's pause waits on Nick too: the same limits end it (here 0.6 s)."""
    c = TO.SessionClock(time.time(), 2.0, answer_min=0.01)
    r = _runner(tmp_path, c)
    r.a.pi, r.a.overhead_port, r.a.no_recording = "127.0.0.1", 8000, False
    r.feed = None
    r.marks_unconfirmed = 0
    monkeypatch.setattr(E.TKW, "recorder_used", lambda a: True)
    monkeypatch.setattr(E.CAM, "check_stream", lambda *a, **k: (False, "no answer", None, None))
    monkeypatch.setattr(E.CAM, "notes_answer", lambda *a, **k: False)
    th, box = _in_thread(r.recorder_ok)
    th.join(10)
    assert not th.is_alive()
    assert r.timed_out and r.end_reason["key"] == TO.NO_ANSWER and "recorder" in r.end_reason["waiting_on"]
