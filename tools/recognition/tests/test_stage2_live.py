"""Stage 2, lane R: live use - the roll call, confidence with reasons, fresh confirmations.

Synthetic recording with a stand-in detector (reads truth.csv), like test_use_model.py. The page
is driven through its HTTP routes. No training; well under a minute.
"""
import csv
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
for p in (REC, os.path.join(TOOLS, "label"), os.path.join(TOOLS, "detector"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import use_model as um  # noqa: E402
from test_use_model import _label_synth  # noqa: E402


@pytest.fixture(scope="module")
def recording(tmp_path_factory):
    out = str(tmp_path_factory.mktemp("rec2") / "20261020-live_E1")
    _label_synth().make_recording(out, "main", seconds=6.0, seed=3)
    return out


class Truth:
    """Stand-in detector: called once per frame in order; `hide` = {truth obj: (first frame, last frame)}
    leaves that object out of the guesses for those frames (it is lost for a while)."""

    def __init__(self, folder, hide=None):
        self.by = {}
        with open(os.path.join(folder, "truth.csv"), newline="") as f:
            for r in csv.DictReader(f):
                if r["cls"] == "alvik":
                    self.by.setdefault(int(r["frame"]), []).append(
                        (int(r["obj"]), dict(cls="alvik", x=float(r["x"]), y=float(r["y"]),
                                             heading_deg=float(r["heading_deg"]), score=0.9)))
        self.hide, self.n = hide or {}, 0

    def __call__(self, frame):
        n = self.n
        self.n += 1
        return [dict(d) for k, d in self.by.get(n, []) if not (k in self.hide and self.hide[k][0] <= n <= self.hide[k][1])]


def get(view, path):
    return json.loads(urllib.request.urlopen(view.url + path, timeout=5).read())


def post(view, path, body=None):
    """-> (http code, reply)"""
    req = urllib.request.Request(view.url + path, data=json.dumps(body or {}).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        r = urllib.request.urlopen(req, timeout=5)
        return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def read(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def wait_roll(view, seconds=10):
    end = time.time() + seconds
    while time.time() < end:
        r = get(view, "rollcall")
        if r["active"]:
            return r
        time.sleep(0.05)
    raise AssertionError("the roll call never appeared on the page")


def start(recording, tmp_path, detector, pipe_kw=None, max_frames=150, name="t.csv"):
    """Run the recording on a thread with the page; returns (view, pipe, thread, out path, result)."""
    pipe = um.Pipeline(detector, 30.0, (640, 480), **(pipe_kw or {}))
    view = um.LiveView(0, "test", classes=["alvik", "unknown"], pipe=pipe)
    out = str(tmp_path / name)
    res = {}

    def go():
        src = um.open_source(recording)
        try:
            res["n"] = um.run(src, pipe, out, view=view, max_frames=max_frames, say=lambda *_: None, roll=True)
        except Exception as e:           # shown by the test that joins
            res["error"] = e
        finally:
            src.close()

    th = threading.Thread(target=go, daemon=True)
    th.start()
    return view, pipe, th, out, res


# --------------------------------------------------------------------------------------------
def test_roll_call_on_the_page_names_objects_and_starts_tracking(recording, tmp_path):
    view, pipe, th, out, res = start(recording, tmp_path, Truth(recording))
    try:
        roll = wait_roll(view)
        assert [o["obj"] for o in roll["objects"]] == [1, 2, 3]            # numbered, left to right
        assert roll["objects"][0]["x"] < roll["objects"][1]["x"] < roll["objects"][2]["x"]
        assert "alvik" in roll["classes"]
        page = urllib.request.urlopen(view.url, timeout=5).read().decode()
        assert "Roll call" in page and "Looks right" in page and "Redo the roll call" in page
        assert "alvik" not in page.lower() and "in real life" not in page.lower()
        # a bad request changes nothing and says so in plain words
        code, rep = post(view, "rollcall", {"objects": [{"x": "left", "y": 3}]})
        assert code == 400 and "Nothing was changed" in rep["error"]
        assert get(view, "rollcall")["active"]
        # the state sent to the page carries the roll call too
        s = get(view, "state?since=-1")
        assert s["roll"]["id"] == roll["id"] and s["image"].startswith("data:image/jpeg")
        # fix: name two, set the type of one, drag one a little
        objs = roll["objects"]
        objs[0]["name"], objs[1]["name"] = "Left", "Right"
        objs[2]["x"] += 2.0
        code, rep = post(view, "rollcall", {"objects": objs})
        assert code == 200 and rep["count"] == 3
        assert "Left" in rep["message"] and "take it back" in rep["message"]       # says what and how to undo
        assert not get(view, "rollcall")["active"]
        assert post(view, "rollcall", {"objects": objs})[0] == 409                  # nothing waits any more
        th.join(30)
    finally:
        view.stop_event.set()
        th.join(5)
        view.close()
    assert "error" not in res, res
    rows = read(out)
    assert list(rows[0].keys())[-2:] == ["name", "confirmed"]
    assert list(rows[0].keys())[:9] == um.BASE_COLUMNS + ["source"]            # older columns unchanged
    assert {r["obj"]: r["name"] for r in rows if r["name"]} == {"1": "Left", "2": "Right"}
    assert {r["name"] for r in rows if r["obj"] == "3"} == {""}
    assert {r["confirmed"] for r in rows if r["obj"] in ("1", "2")} == {"yes"}
    assert len({r["obj"] for r in rows}) == 3
    assert int(rows[0]["frame"]) >= 1                                           # the steady picture, not frame 0


def test_roll_call_in_the_console(recording, tmp_path):
    answers = iter(["n 1 Left", "d 3", "u", "t 2 unknown", "u", "a 100 100 alvik Extra one", ""])
    said = []
    out = str(tmp_path / "c.csv")
    pipe = um.Pipeline(Truth(recording), 30.0, (640, 480), classes=["alvik", "unknown"])
    src = um.open_source(recording)
    um.run(src, pipe, out, max_frames=40, say=said.append, roll=True, ask=lambda prompt: next(answers))
    src.close()
    text = "\n".join(said)
    assert "Deleted 3. Type u to take it back." in text and "Taken back." in text      # every action: how to undo
    assert "Added object 4 (alvik) at (100, 100)" in text and "Roll call confirmed: 4 object(s)" in text
    rows = read(out)
    assert {r["name"] for r in rows if r["obj"] == "1"} == {"Left"}
    assert {r["name"] for r in rows if r["obj"] == "4"} == {"Extra one"}
    assert {r["cls"] for r in rows if r["obj"] == "2"} == {"alvik"}                     # the type change was taken back
    # q stops the run without a table of rows
    pipe2 = um.Pipeline(Truth(recording), 30.0, (640, 480))
    src = um.open_source(recording)
    n = um.run(src, pipe2, str(tmp_path / "q.csv"), max_frames=20, say=lambda *_: None, roll=True, ask=lambda p: "q")
    src.close()
    assert n == 0
    # a bad line changes nothing
    ans = iter(["d x", "m 1", "x", ""])
    said = []
    objs = um.console_roll([dict(obj=1, cls="alvik", x=1.0, y=2.0, name="", heading_deg=None)], ["alvik"],
                           lambda p: next(ans), said.append)
    assert len(objs) == 1 and sum("Nothing changed" in s for s in said) == 3


def test_outside_names_the_found_objects():
    objs = [dict(obj=1, cls="alvik", x=100.0, y=100.0, name="", heading_deg=None),
            dict(obj=2, cls="alvik", x=300.0, y=100.0, name="", heading_deg=10.0)]
    um.apply_outside(objs, [dict(x=305, y=104, name="Right", heading_deg=99), dict(x=96, y=99, name="Left", heading_deg=45)], 30.0)
    assert [o["name"] for o in objs] == ["Left", "Right"]
    assert objs[0]["heading_deg"] == 45.0 and objs[1]["heading_deg"] == 10.0     # a seen heading is kept

    class Plug:
        def drafts(self, t_wall):
            return [dict(x=101, y=99, name="Left")]
    pipe = um.Pipeline(lambda i: [], 30.0)
    pipe.outside = Plug()
    got = um.draft_objects([dict(cls="alvik", x=100, y=100, score=0.9)], pipe, "1.0")
    assert got[0]["name"] == "Left"
    said = []
    assert um.load_outside(None, said.append) is None
    um.load_outside("some:file", said.append)                                      # missing part: said, not a crash
    assert said == [] or "Carrying on" in said[0]


def test_confidence_reasons_ask_and_reconfirm(recording, tmp_path):
    """Robot 1 goes unseen for frames 40-110: its confidence falls, the page asks about it alone, its rows
    say confirmed = no until a click on the page confirms it. The other robots carry on."""
    det = Truth(recording, hide={1: (40, 110)})
    view, pipe, th, out, res = start(recording, tmp_path, det, dict(ask_below=0.5, ask_after_s=0.6), max_frames=170)
    seen = dict(asks=None, reasons=set(), colours=set(), msg=None)
    try:
        roll = wait_roll(view)
        assert post(view, "rollcall", {"objects": roll["objects"]})[0] == 200
        deadline = time.time() + 30
        since = -1
        while time.time() < deadline and th.is_alive():
            s = get(view, "state?since=%d" % since)
            if s.get("same"):
                continue
            since = s["ver"]
            for o in s["objects"]:
                seen["colours"].add(o["colour"])
                if o["obj"] == 1 and o["why"] and o["colour"] != "green":
                    seen["reasons"].add(o["why"])
            if s["asks"] and seen["asks"] is None:
                seen["asks"] = s["asks"]
                assert [a["obj"] for a in s["asks"]] == [1]                     # just this one
                assert {o["obj"] for o in s["objects"]} >= {2, 3}               # the others are still shown
                code, rep = post(view, "confirm", {"obj": 1})
                assert code == 200 and "Undo" in rep["message"] and "confirmed" in rep["message"]
                seen["msg"] = rep["message"]
            if s["done"]:
                break
        th.join(30)
    finally:
        view.stop_event.set()
        th.join(5)
        view.close()
    assert "error" not in res, res
    assert seen["asks"] is not None and "not seen" in seen["asks"][0]["why"] and seen["asks"][0]["last_confirmed_s"] > 0.5
    assert any("not seen for" in r for r in seen["reasons"]) or any("score low" in r for r in seen["reasons"])
    assert {"green", "red"} <= seen["colours"]
    rows = [r for r in read(out) if r["obj"] == "1"]
    by = {int(r["frame"]): r["confirmed"] for r in rows}
    assert by[min(by)] == "yes"
    assert "no" in by.values()
    first_no = min(f for f, c in by.items() if c == "no")
    assert 40 <= first_no <= 95                                                   # from the moment it was lost
    # rows between losing and re-confirming are "no", after the click they are "yes" again
    stretch = [by[f] for f in sorted(by) if f >= first_no]
    switched = stretch.index("yes") if "yes" in stretch else None
    assert switched is not None and set(stretch[:switched]) == {"no"}
    # the other robots were never marked
    assert {r["confirmed"] for r in read(out) if r["obj"] in ("2", "3")} == {"yes"}


def test_remove_undo_and_unknown_object(recording, tmp_path):
    view, pipe, th, out, res = start(recording, tmp_path, Truth(recording), max_frames=120)
    try:
        roll = wait_roll(view)
        post(view, "rollcall", {"objects": roll["objects"]})
        for _ in range(100):                                 # wait for pictures to flow
            s = get(view, "state?since=-1")
            if s.get("objects"):
                break
        code, rep = post(view, "confirm", {"obj": 77})
        assert code == 404 and "nothing was changed" in rep["error"]
        assert post(view, "confirm", {"obj": 1, "x": 5})[0] == 400                # only one of x, y
        assert post(view, "confirm", {})[0] == 400
        assert post(view, "nonsense", {"obj": 1})[0] == 404
        code, rep = post(view, "remove", {"obj": 2})
        assert code == 200 and "Undo" in rep["message"]
        code, rep = post(view, "undo", {"obj": 2})
        assert code == 200 and "Taking back" in rep["message"]
        assert post(view, "rollcall/redo")[0] == 200                              # asks again; answer it unchanged
        r2 = wait_roll(view)
        assert r2["id"] != roll["id"] and {o["obj"] for o in r2["objects"]} == {1, 2, 3}
        assert post(view, "rollcall", {"objects": r2["objects"]})[0] == 200
        th.join(30)
    finally:
        view.stop_event.set()
        th.join(5)
        view.close()
    assert "error" not in res, res
    rows = read(out)
    assert {r["obj"] for r in rows} == {"1", "2", "3"}                            # robot 2 came back after the undo
    assert max(int(r["frame"]) for r in rows if r["obj"] == "2") > 60


def test_judge_reasons_in_plain_words():
    pipe = um.Pipeline(lambda i: [], 30.0, check=True, ask_below=0.5, ask_after_s=1.0)
    pipe.state[1] = dict(t_conf=0.0, f_conf=0, low_since=None, asked=False)

    def rec(f, obj, x, score, source):
        return dict(frame=f, obj=obj, cls="thing", x_px=x, y_px=50.0, score=score, source=source, colour="amber", age=40,
                    heading_deg=None)
    a = [rec(1, 1, 100.0, 0.3, "track"), rec(1, 2, 120.0, 0.9, "model")]
    pipe.last_seen[1] = -3.0
    pipe.judge(a, 1, 0.0)
    why = a[0]["why"]
    assert "not seen for 3 s" in why and "score low (30%)" in why and "two lookalikes close together" in why
    assert a[0]["confirmed"] == "no" and a[1]["confirmed"] == "auto"            # nobody confirmed #2
    b = [rec(2, 1, 100.0, 0.3, "track")]
    pipe.judge(b, 2, 1.5)                                                        # low for 1.5 s: now it asks
    assert b[0]["needs"] and b[0]["colour"] == "red" and pipe.asks(b)[0]["obj"] == 1
    assert any("Asking you to confirm just this one" in n for n in pipe.take_notes())
    # a dip that mends itself before the time is up goes back to yes
    pipe2 = um.Pipeline(lambda i: [], 30.0, check=True, ask_below=0.5, ask_after_s=5.0)
    pipe2.state[1] = dict(t_conf=0.0, f_conf=0, low_since=None, asked=False)
    c = [rec(1, 1, 100.0, 0.3, "model")]
    pipe2.judge(c, 1, 1.0)
    assert c[0]["confirmed"] == "no"
    d = [rec(2, 1, 100.0, 0.9, "model")]
    pipe2.judge(d, 2, 1.5)
    assert d[0]["confirmed"] == "yes" and d[0]["last_confirmed_s"] == 1.5


def test_without_roll_call_the_table_is_unchanged(recording, tmp_path):
    out = str(tmp_path / "plain.csv")
    pipe = um.Pipeline(Truth(recording), 30.0, (640, 480))
    src = um.open_source(recording)
    um.run(src, pipe, out, max_frames=30, say=lambda *_: None)
    src.close()
    assert list(read(out)[0].keys()) == um.BASE_COLUMNS + ["source"]


def test_options_reach_the_run(recording, tmp_path):
    a = um.build_parser().parse_args([recording, "--roll-call", "--ask-below", "0.4", "--ask-after", "2", "--outside", "x:y"])
    assert a.roll_call is True and a.ask_below == 0.4 and a.ask_after == 2 and a.outside == "x:y"
    assert um.build_parser().parse_args([recording]).roll_call is None
    assert um.build_parser().parse_args([recording, "--no-roll-call"]).roll_call is False
    out = str(tmp_path / "cmd.csv")
    said = []
    a = um.build_parser().parse_args([recording, "--roll-call", "--out", out, "--max-frames", "25", "--no-page"])
    assert um._run_main(a, recording, detector=Truth(recording), say=said.append, ask=lambda p: "") == 0
    assert list(read(out)[0].keys())[-2:] == ["name", "confirmed"]
    assert any("Roll call" in s for s in said)
