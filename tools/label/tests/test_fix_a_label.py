"""Stage 2 fix round A, labelling side: re-track after a save in a short window plus the background
(saves never hold the session for long), home visits never accepted unseen and their facing only
while there, a still object keeping its number, and "How often should I check?" recomputed when the
labels change, with a number or a plain reason."""
import os
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LABEL = os.path.dirname(HERE)
sys.path.insert(0, LABEL)
if HERE not in sys.path:
    sys.path.append(HERE)

import app          # noqa: E402
import tracker      # noqa: E402
from test_handoff_label import still_recording, fresh, open_with_frames, N  # noqa: E402


@pytest.fixture(scope="module")
def still(tmp_path_factory):
    d = still_recording(str(tmp_path_factory.mktemp("still_fa") / "rec"))
    app.Session(d, log=lambda *a: None, classes=["box", "unknown"])
    return d


def det(x, y, cls="box", score=0.8, **kw):
    return dict(dict(x=float(x), y=float(y), cls=cls, score=score), **kw)


def _label(s, f, objs, state="reviewed"):
    return s.save_frame(f, [dict(dict(obj=None, source="user", status="corrected"), **o) for o in objs], state, "user")


# ---------------------------------------------------------------- re-track: window now, the rest in the background
def _sliding(s):
    """A box sliding slowly across the picture on every frame (so the guesses after a label matter)."""
    for f in range(len(s.dets)):
        s.dets[f] = [det(40 + f * 1.0, 120)]
    s.run.snaps = {}
    s.guesses, s.summary = s.run.run(s._anchors(), s._reserved_ids())
    s._update_conf()


def test_retrack_runs_a_window_now_and_the_rest_in_the_background(tmp_path, still):
    d1, d2 = fresh(still, str(tmp_path / "a")), fresh(still, str(tmp_path / "b"))
    bg = open_with_frames(d1, range(0, N, 10))
    sync = open_with_frames(d2, range(0, N, 10))
    sync.background_retrack = False
    bg.SYNC_FRAMES = 20                     # 200 frames: a window of 20 now, the rest in the background
    for s in (bg, sync):
        _sliding(s)
    calls = []
    real = bg.run.run

    def spy(*a, **kw):
        calls.append(kw.get("until"))
        return real(*a, **kw)
    bg.run.run = spy
    for s in (bg, sync):
        _label(s, 30, [dict(cls="box", x=70.0, y=150.0)])       # the person moves it: guesses after 30 change
    assert calls and calls[0] == 30 + 20                          # the save itself ran a 20-frame window only
    assert bg.wait_settled(30)
    assert bg.tracking_view() is None and bg.stats()["tracking"] is None
    same = lambda s: {f: [(r["obj"], r["x"], r["y"]) for r in rs] for f, rs in s.guesses.items() if rs}   # noqa: E731
    assert same(bg) == same(sync)                                  # the background run gives the full result


def test_a_newer_save_replaces_a_background_run_and_settle_finishes_it(tmp_path, still):
    d1, d2 = fresh(still, str(tmp_path / "a")), fresh(still, str(tmp_path / "b"))
    bg = open_with_frames(d1, range(0, N, 10))
    sync = open_with_frames(d2, range(0, N, 10))
    sync.background_retrack = False
    bg.SYNC_FRAMES = 20
    for s in (bg, sync):
        _sliding(s)
    with bg.lock:                          # hold the lock: the first background run can't finish meanwhile
        _label(bg, 30, [dict(cls="box", x=70.0, y=150.0)])
        _label(bg, 120, [dict(cls="box", x=160.0, y=60.0)])
        assert bg.tracking_view() == dict(from_frame=30)          # the second still starts at the earlier change
        assert bg.settle()                                        # bulk actions finish it now
        assert bg.tracking_view() is None
    for f, o in ((30, dict(cls="box", x=70.0, y=150.0)), (120, dict(cls="box", x=160.0, y=60.0))):
        _label(sync, f, [o])
    bg.wait_settled(30)
    time.sleep(0.2)
    # the same places (a new label's number may differ: numbers handed out in the window are not reused)
    same = lambda s: {f: sorted((r["x"], r["y"]) for r in rs) for f, rs in s.guesses.items() if rs}   # noqa: E731
    assert same(bg) == same(sync)
    nums = [[r["obj"] for r in rs] for rs in bg.guesses.values()]
    assert all(len(n) == len(set(n)) for n in nums)


def test_trackrun_window_changes_nothing_kept(tmp_path, still):
    dets = [[det(40 + f, 120)] for f in range(N)]
    times = [f / 10.0 for f in range(N)]
    run = tracker.TrackRun(dets, times, 30.0, (320, 240))
    full, _ = run.run({}, set())
    snaps, rows_before = dict(run.snaps), dict(run._rows)
    anchors = {50: [dict(obj=1, cls="box", x=95.0, y=150.0, status="corrected")]}
    win, _ = run.run(anchors, {1}, from_frame=50, until=80)
    assert run.snaps.keys() == snaps.keys() and run._rows == rows_before          # nothing kept from the window
    assert max(win) < 80
    c = run.clone()
    rows, _ = c.run(anchors, {1}, from_frame=50, cancel=lambda: True)
    assert rows is None                                                           # a replaced run stops
    rows, _ = c.run(anchors, {1}, from_frame=50)
    run.adopt(c)
    for f in range(50, 80):
        assert [(r["obj"], r["x"], r["y"]) for r in win.get(f, [])] == [(r["obj"], r["x"], r["y"]) for r in rows.get(f, [])]


# ---------------------------------------------------------------- home visits: never accepted unseen
def test_frames_with_a_home_guess_are_never_accepted_without_a_person(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20, 30])
    g = lambda src: dict(obj=1, cls="box", x=50.0, y=50.0, heading_deg=None, heading_known=False,   # noqa: E731
                         heading_guess=None, source=src, score=0.97)
    s.guesses = {10: [g("home")], 20: [g("motion")], 30: [g("motion")]}
    s._update_conf()
    assert s.has_home_guess(10) and not s.has_home_guess(20)
    assert s.stats()["high_pending"] == 2
    done = s.auto_accept_frames()
    assert done == [20, 30] and s.frame_state(10) == "pending"


def test_home_facing_holds_only_while_the_object_sits_there():
    p = tracker.TrackerParams()
    p.front_classes = ("box",)
    tk = tracker.Tracker(30.0, (320, 240), p)
    rows = []
    for f in range(40):                    # 2 s at home (facing 90), then 2 s of outside-data guesses without one
        t = f / 10.0
        d = det(100, 100, score=0.97, source="home", heading_deg=90.0) if f < 20 else det(100, 100, source="log")
        rows.append(tk.step(f, t, [d], None))
    at_home = [r for r in rows[5][:1]]
    after = [r for r in rows[30][:1]]
    assert at_home and at_home[0]["heading_known"] and at_home[0]["heading_deg"] == 90.0
    assert after and not after[0]["heading_known"] and after[0]["heading_guess"] == 90.0


# ---------------------------------------------------------------- a still object keeps its number
def test_a_still_object_labelled_again_keeps_its_number(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 60, 120, 180])
    for f in range(N):
        s.dets[f] = []                                 # nothing found it: the guesses lose it between labels
    s.run.snaps = {}
    s.guesses, s.summary = s.run.run({}, set())
    a = _label(s, 10, [dict(cls="box", x=100.0, y=100.0)])["frame"]["objects"][0]["obj"]
    s.guesses = {f: [r for r in rs if r["obj"] != a] for f, rs in s.guesses.items()}     # the guesses lost it
    b = _label(s, 120, [dict(cls="box", x=102.0, y=101.0), dict(cls="box", x=250.0, y=50.0)])["frame"]["objects"]
    assert b[0]["obj"] == a                            # same spot, same type: the same object
    assert b[1]["obj"] != a                            # another object elsewhere: a number of its own
    c = _label(s, 180, [dict(cls="unknown", x=100.0, y=100.0)])["frame"]["objects"]
    assert c[0]["obj"] != a                            # another type at that spot: not the same object


def test_a_number_followed_away_is_not_reused(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 120])
    a = _label(s, 10, [dict(cls="box", x=100.0, y=100.0)])["frame"]["objects"][0]["obj"]
    s.guesses = {f: ([dict(obj=a, cls="box", x=200.0, y=100.0, heading_deg=None, heading_known=False,
                           heading_guess=None, source="track", score=0.9)] if 40 <= f < 80 else [])
                 for f in range(N)}
    o = _label(s, 120, [dict(cls="box", x=100.0, y=100.0)])["frame"]["objects"][0]
    assert o["obj"] != a                               # it drove away in between: maybe another one parked there


# ---------------------------------------------------------------- how often should I check
def test_checkin_numbers_or_a_plain_reason():
    times = [f / 10.0 for f in range(400)]
    # a box that jumps 40 px every 2 s: right after a label the guess is close, later it is not
    pos = lambda f: 100.0 + 40.0 * (f // 20)                                                       # noqa: E731
    dets = [[det(pos(f), 120)] for f in range(400)]
    anchors = {f: [dict(obj=1, cls="box", x=pos(f), y=120.0, status="corrected")] for f in range(0, 400, 7)}
    res = tracker.measure_checkins(dets, times, 30.0, anchors)
    assert res["fine_every_s"] is not None or res["why_short"] is not None
    # a guess that is always far off: no interval, the reason in counts
    bad = [[det(250, 200)] for f in range(400)]
    res = tracker.measure_checkins(bad, times, 30.0, anchors)
    assert res["fine_every_s"] is None and res["why_short"]["n"] >= 5
    w = res["why_short"]
    assert w["close"] + w["far"] + w["mixed"] + w["lost"] == w["n"] and w["close"] < w["n"]


def test_checkins_are_measured_again_when_the_labels_changed(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 50, 90, 130])
    for f in (10, 50):
        _label(s, f, [dict(cls="box", x=100.0, y=100.0)])
    s.start_checkins()
    for _ in range(200):
        if s.checkins["status"] != "running":
            break
        time.sleep(0.05)
    assert s.checkins_view()["status"] == "done" and not s.checkins_view().get("stale")
    s.checkins = None
    _label(s, 90, [dict(cls="box", x=100.0, y=100.0)])
    v = s.checkins_view()
    assert v["stale"] and (v["labelled_then"], v["labelled_now"]) == (2, 3)
    v = s.checkins_view(restart_if_stale=True)
    assert v["again"] and v["labelled_now"] == 3


def test_a_closed_session_never_takes_a_background_result(tmp_path, still):
    d = fresh(still, str(tmp_path / "a"))
    s = open_with_frames(d, range(0, N, 10))
    s.SYNC_FRAMES = 20
    _sliding(s)
    with s.lock:
        _label(s, 30, [dict(cls="box", x=70.0, y=150.0)])
        before = {f: list(rs) for f, rs in s.guesses.items()}
        s.close()                       # the studio opens the recording again
    time.sleep(1.0)
    assert s.guesses == before and s.tracking_view() is None
