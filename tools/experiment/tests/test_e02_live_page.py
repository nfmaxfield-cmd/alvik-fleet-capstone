"""Live-page fixes (2026-10-25, after the 15:10 session): a question about a picture shows the picture; "n" to the
green outline carries on with the stored calibration (no studio steps); the Note button writes the moment down; the
Live lab keeps one filter per robot, started at the camera, restart threshold 120."""
import csv
import json
import os
import sys
import threading
import time
import types
import urllib.error
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import e02_testkit  # noqa: E402,F401  (paths)

import e02  # noqa: E402
import e02_live as LIVE  # noqa: E402
import e02_view  # noqa: E402

PNG = (b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01\x08\x02\x00\x00\x00\x90wS\xde\x00"
       b"\x00\x00\x0cIDATx\x9cc\xf8\x0f\x00\x00\x01\x01\x00\x05\x18\xd8N\x00\x00\x00\x00IEND\xaeB`\x82")


class _R:
    phase, order, tracker, feed, st, cam, board = "setup", "", None, None, {}, None, None


def _get(url):
    return urllib.request.urlopen(url, timeout=3).read()


def _post(url, d):
    req = urllib.request.Request(url, data=json.dumps(d).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    return urllib.request.urlopen(req, timeout=3).read()


# --- 1. the picture inside the question ---------------------------------------------------------------------
def test_question_picture_is_served_by_question_id(tmp_path):
    pic = tmp_path / "check_calibration.png"
    pic.write_bytes(PNG)
    v = e02_view.LiveView(_R(), port=18194)
    v.health = lambda: None
    v.start()
    try:
        with v.lock:
            v.question = dict(id=123, prompt="Does the green outline sit ...? (y/n)", default="y", timeout_s=None,
                              image=str(pic))
        s = json.loads(_get("http://127.0.0.1:18194/status.json"))
        assert s["question"]["image"] is True and s["question"]["image_name"] == "check_calibration.png"
        assert str(tmp_path) not in json.dumps(s)               # the page never sees a file path
        assert _get("http://127.0.0.1:18194/question-image?id=123") == PNG
        with pytest.raises(urllib.error.HTTPError):
            _get("http://127.0.0.1:18194/question-image?id=999")   # only the open question's picture
        with v.lock:
            v.question = None
        with pytest.raises(urllib.error.HTTPError):
            _get("http://127.0.0.1:18194/question-image?id=123")
    finally:
        v.stop()


def test_page_has_picture_slot_yes_no_and_note_button():
    p = e02_view.PAGE
    for needle in ('id="askimg"', "/question-image?id=", 'id="askyn"', 'id="noteb"', "fetch('/note'"):
        assert needle in p


def _fake_runner(view, web=True):
    f = types.SimpleNamespace(view=view, hold=threading.Event(), hold_owner=None, said=[], marks=[],
                              a=types.SimpleNamespace(auto=False, web_ask=web))
    f.say = lambda s, quiet=False: f.said.append(s)
    f.mark = lambda s: f.marks.append(s)
    f._input = e02.Runner._input
    f.web_ask = types.MethodType(e02.Runner.web_ask, f)
    return f


def test_ask_carries_the_picture_to_the_page(tmp_path):
    pic = tmp_path / "check_board_edges.png"
    pic.write_bytes(PNG)
    v = e02_view.LiveView(_R(), port=0)
    f = _fake_runner(v)
    box = []
    th = threading.Thread(target=lambda: box.append(e02.Runner.ask(f, "Which edge? ", "top", image=str(pic))))
    th.start()
    for _ in range(50):
        with v.lock:
            q = dict(v.question) if v.question else None
        if q:
            break
        time.sleep(0.05)
    assert q and q["image"] == os.path.abspath(str(pic))
    with v.lock:
        v.answers[q["id"]] = "bottom"
    th.join(5)
    assert box == ["bottom"]


def test_question_without_picture_has_none(tmp_path):
    v = e02_view.LiveView(_R(), port=0)
    f = _fake_runner(v)
    box = []
    th = threading.Thread(target=lambda: box.append(e02.Runner.ask(f, "Carry on? ", "y",
                                                                    image=str(tmp_path / "missing.png"))))
    th.start()
    for _ in range(50):
        with v.lock:
            q = dict(v.question) if v.question else None
        if q:
            break
        time.sleep(0.05)
    assert q and "image" not in q          # a picture that was not saved is left out, the question still asked
    with v.lock:
        v.answers[q["id"]] = "y"
    th.join(5)
    assert box == ["y"]


# --- 2. "n" to the outline: the stored calibration, said plainly -----------------------------------------------
def _calib_runner(tmp_path):
    f = types.SimpleNamespace(st={"setup": {}}, said=[], marks=[])
    f.say = lambda s, quiet=False: f.said.append(s)
    f.mark = lambda s: f.marks.append(s)

    def no_studio(img):
        raise AssertionError("the studio steps must not start after an outline 'n'")
    f.studio_calibration = no_studio
    return f


def test_outline_no_keeps_the_stored_calibration(tmp_path):
    cal = tmp_path / "overhead_floorcal_roam1.json"
    cal.write_text("{}")
    f = _calib_runner(tmp_path)
    got = e02.Runner.outline_doubted(f, str(cal), None)
    assert got == str(cal)
    assert f.st["setup"]["calibration_doubted"]["file"] == str(cal)
    words = " ".join(f.said)
    assert "outline is off" in words and "stored camera calibration" in words and "4 corners" in words
    assert not any("studio" in w.lower() for w in f.said)
    # every run's line says so
    f.st["calibration"] = str(cal)
    note = e02.Runner.calib_note(f)
    assert "outline is off" in note


def test_outline_no_on_a_made_here_calibration_uses_the_default(tmp_path):
    if not os.path.exists(e02.DEFAULT_CALIB):
        pytest.skip("no stored calibration in this checkout")
    f = _calib_runner(tmp_path)
    assert e02.Runner.outline_doubted(f, None, None) == e02.DEFAULT_CALIB


def test_outline_no_with_nothing_stored_still_has_the_studio(tmp_path, monkeypatch):
    monkeypatch.setattr(e02, "DEFAULT_CALIB", str(tmp_path / "none.json"))
    f = _calib_runner(tmp_path)
    f.studio_calibration = lambda img: "studio.json"
    assert e02.Runner.outline_doubted(f, None, None) == "studio.json"


# --- 3. Note --------------------------------------------------------------------------------------------------
class _Tr:
    def __init__(self):
        self.active = "Left"
        self.protect = {"Left": (30.0, 40.0), "Center": (60.0, 61.5)}

    def where(self):
        return (time.time(), 31.2, 41.7)


class _NoteR(_R):
    def __init__(self, d):
        self.dir = str(d)
        self.t_launch = time.time() - 125.0
        self.phase, self.order = "exploring", ""
        self.tracker = _Tr()
        self.heads = {"Left": [92.0, True]}
        self.st = {"robots": ["Left", "Center", "Right"], "positions": {"Right": [100.0, 20.0, 180.0]}}


def test_note_writes_time_and_every_robot(tmp_path):
    v = e02_view.LiveView(_NoteR(tmp_path), port=18195)
    v.health = lambda: None
    v.start()
    try:
        d = json.loads(_post("http://127.0.0.1:18195/note", {}))
        assert d["ok"] and d["id"] == 1 and d["robots"] == 3 and d["robots_total"] == 3 and d["elapsed"].startswith("0:02:0")
        d2 = json.loads(_post("http://127.0.0.1:18195/note", dict(id=1, text="Left and Center touched")))
        assert d2["ok"]
        d3 = json.loads(_post("http://127.0.0.1:18195/note", dict(text="second")))
        assert d3["id"] == 2
    finally:
        v.stop()
    rows = list(csv.DictReader(open(tmp_path / e02_view.NOTES_CSV)))
    assert [r["note"] for r in rows] == ["1", "2"]
    a = rows[0]
    assert a["text"] == "Left and Center touched" and a["phase"] == "exploring" and a["followed"] == "Left"
    assert 124 <= float(a["elapsed_s"]) <= 130
    assert (a["Left x_cm"], a["Left y_cm"], a["Left heading_deg"], a["Left where_from"]) == ("31.2", "41.7", "92",
                                                                                           "camera, now")
    assert a["Center x_cm"] == "60.0" and a["Center where_from"].startswith("camera, last")
    assert a["Right x_cm"] == "100.0" and a["Right heading_deg"] == "180" and "session state" in a["Right where_from"]
    # a resumed session (a new page server) carries on the numbering
    v2 = e02_view.LiveView(_NoteR(tmp_path), port=0)
    assert v2.note("third")["id"] == 3
    assert len(list(csv.DictReader(open(tmp_path / e02_view.NOTES_CSV)))) == 3


def test_note_without_session_folder_says_why():
    d = e02_view.LiveView(_R(), port=0).note()
    assert not d["ok"] and "session folder" in d["error"]


def test_note_file_goes_up_with_the_session():
    # upload_session.py zips the whole session folder: nothing filtered by name
    src = open(os.path.join(os.path.dirname(e02_testkit.TOOLS), "tools", "pi", "upload_session.py")).read()
    assert "os.walk(folder)" in src


# --- 4. Live lab: one filter per robot, started at the camera, threshold 120 ---------------------------------
def _raw(run, x=500.0, y=-300.0, th=77.0, v=0.0, w=0.0, gz=0.0):
    return dict(run=run, v_cms=str(v), w_dps=str(w), gz=str(gz), x_cm=str(x), y_cm=str(y), th_deg=str(th))


def test_lab_waits_for_the_camera_and_starts_there():
    lab = LIVE.LiveLab()
    assert lab.T == 120.0
    lab.on_raw("Left", _raw("1"), 10.0)
    assert lab.f is None                       # the library's own odometry (500, -300) is not a floor position
    lab.on_camera("Left", 10.05, 20.0, 30.0)
    lab.on_raw("Left", _raw("1"), 10.1)
    assert lab.f is not None and abs(lab.f.s[0] - 20.0) < 0.5 and abs(lab.f.s[1] - 30.0) < 0.5
    assert lab.state()["heading_known"] is False


def test_lab_other_robots_rows_do_not_restart_it():
    lab = LIVE.LiveLab()
    lab.on_camera("Left", 0.0, 20.0, 30.0)
    lab.on_raw("Left", _raw("1"), 0.05)
    f_left = lab.f
    lab.on_camera("Center", 0.1, 80.0, 90.0)
    lab.on_raw("Center", _raw("2"), 0.12)
    assert lab.n_init == 2 and lab.robot == "Center"
    for k in range(10):                        # interleaved rows, as the listener hands them over
        lab.on_raw("Left", _raw("1"), 0.2 + 0.02 * k)
        lab.on_raw("Center", _raw("2"), 0.21 + 0.02 * k)
    assert lab.n_init == 2 and lab.resets == 0
    lab.on_raw("Left", _raw("1"), 0.5)
    assert lab.f is f_left                     # Left's own filter back, not a new one
    lab.on_camera("Left", 0.55, 20.0, 30.0)
    lab.on_raw("Left", _raw("3"), 0.6)         # a new run: a new filter, at the camera
    assert lab.n_init == 3


def test_lab_heading_from_a_straight_camera_drive():
    lab = LIVE.LiveLab()
    lab.on_camera("Left", 0.0, 0.0, 0.0)
    lab.on_raw("Left", _raw("1", th=0.0), 0.01)
    t, x = 0.02, 0.0
    while t < 1.5:                             # 10 cm/s straight along +y in the floor's frame
        lab.on_raw("Left", _raw("1", th=0.0, v=100.0), t)   # RAW v_cms is mm/s
        lab.on_camera("Left", t + 0.005, 0.0, 10.0 * t)
        t += 0.04
    st = lab.state(now=t)
    assert st["heading_known"] is True
    assert abs(((lab.f.s[2] * 180 / 3.141592653589793) - 90.0 + 180) % 360 - 180) < 15


DOE_SESSION = "stage2"


def test_lab_reproduces_the_restart_doe_on_stage2():
    """The DOE's chosen rule (camera start, 120) gave 11 restarts on stage 2 (e02-oct25-real-fusion.md)."""
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "analyses_oct25"))
    try:
        import live_restart_doe as D
        if not os.path.isdir(D.RF.sdir(DOE_SESSION)):
            raise ImportError
    except Exception:
        pytest.skip("the real sessions' logs are not on this machine")
    raw, track = D.load(DOE_SESSION)
    r = D.replay(raw, track, LIVE.LiveLab())
    assert r["restarts"] == 11


def test_replay_view_writes_no_notes(tmp_path):
    import e02_replay_view as RV
    v = RV.ReplayView(_NoteR(tmp_path), port=0)
    d = v.note("x")
    assert not d["ok"] and "replay" in d["error"]
    assert not (tmp_path / e02_view.NOTES_CSV).exists()


def test_hidden_parts_of_the_question_stay_hidden():
    # .yn and .qimg set display, which beats the hidden attribute unless the page says otherwise
    assert ".ask [hidden]{display:none!important}" in e02_view.PAGE
