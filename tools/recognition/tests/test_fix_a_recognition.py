"""Stage 2 fix round A, studio side: home visits checked against the home (owner, type), robot logs with
a time offset and with no clock file (the offset found from speed), Line up again copying under the
lock and never reopening by itself, private files never taken from a shared project, the AI web
address (https only, never doubled, passed to the SDK), AI suggestions counted one by one, the live
confirm's undo putting everything back, the "default" camera, plain reasons when outside data is not
used, numbers in "finds nothing yet", the training pre-check and plain failures, and the "no model"
row of projects whose types motion can't tell apart."""
import json
import math
import os
import sys
import threading
import types
import zipfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
for p in (REC, os.path.join(TOOLS, "label"), os.path.join(TOOLS, "detector"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import ai_helper as ai  # noqa: E402
import homes as HM  # noqa: E402
import outside as O  # noqa: E402
import outsidelink  # noqa: E402
import project as projmod  # noqa: E402
import robotlogs as RL  # noqa: E402
import share  # noqa: E402
import studio as studiomod  # noqa: E402
import synth_logs as SL  # noqa: E402
import trainer  # noqa: E402
import use_model as um  # noqa: E402
from test_studio import fake_recording, write_labels  # noqa: E402


# =============================================================================== 1. home visits
class FakeRec:
    def __init__(self, n):
        self.n = n
        self.times = [f / 10.0 for f in range(n)]


class FakeSession:
    """What home_refiner reads: the recording's times, the guesses, the types, the roll call names."""

    def __init__(self, guesses, n=120, classes=("box", "unknown"), names=None):
        self.rec = FakeRec(n)
        self.guesses = guesses
        self.classes = list(classes)
        self.outside = {"drafts": {}} if names else None
        self._names = names or {}

    def names_at(self, f):
        return dict(self._names)


def _sitting(obj, cls, x=100.0, y=100.0, frames=range(0, 60)):
    return {f: [dict(obj=obj, cls=cls, x=x, y=y, score=0.6, source="motion")] for f in frames}


HOME = dict(name="Dock", camera="default", x=100.0, y=100.0, radius=10.0, owner="Left", kind="park", still_s=2.0,
            cls="box", facing_deg=90.0)


def test_a_home_visit_needs_the_homes_type_and_owner():
    ref = outsidelink.home_refiner([HOME], "default")
    # the right type, no roll call: used (a guess for the person to check), with no name claimed
    out = ref(FakeSession(_sitting(7, "box")))
    assert len(out["visits"]) == 1 and out["visits"][0]["name"] is None
    assert all(d["source"] == "home" and d["name"] is None for ds in out["dets"].values() for d in ds)
    assert "never accepted without you" in out["note"]
    # another type sitting there: not used, and said why
    out = ref(FakeSession(_sitting(7, "unknown")))
    assert not out["visits"] and "is not a box" in out["note"]
    # the roll call names someone else: not used
    out = ref(FakeSession(_sitting(7, "box"), names={7: "Right"}))
    assert not out["visits"] and "is Right, not Left" in out["note"]
    out = ref(FakeSession(_sitting(7, "box"), names={7: "Left"}))
    assert out["visits"][0]["name"] == "Left"


def test_find_visits_and_anchors_check_the_owner_by_name():
    tracks = {7: [(f, f / 10.0, 100.0, 100.0) for f in range(60)]}
    assert HM.find_visits([HOME], tracks, obj_names={7: "Right"}) == []
    v = HM.find_visits([HOME], tracks, obj_names={7: "Left"})
    assert v and v[0]["name"] == "Left"

    class Cam:
        def to_floor(self, u, v, z=0.0):
            return u / 10.0, v / 10.0

        def px_per_cm(self, *a):
            return 10.0
    home = dict(HOME, facing_deg=None)
    visits = HM.find_visits([home], tracks)
    assert visits and visits[0]["name"] is None
    # an unnamed visitor counts for the owner only when the owner's own stream stood still then
    moving = O.Stream("Left", np.arange(0, 10, 0.1), x=np.arange(0, 10, 0.1) * 10, y=np.zeros(100), frame="floor")
    data = O.OutsideData("csv", {"Left": moving})
    assert HM.anchors_from_visits(visits, [home], Cam(), outside=data) == {}
    assert HM.anchors_from_visits(visits, [home], Cam(), obj_names={7: "Right"}) == {}
    assert set(HM.anchors_from_visits(visits, [home], Cam())) == {"Left"}       # the caller vouches


def test_home_guesses_replace_only_guesses_of_their_type():
    import app
    s = app.Session.__new__(app.Session)
    s.dets = [[dict(x=101.0, y=100.0, cls="box", score=0.5), dict(x=103.0, y=99.0, cls="hand", score=0.8)]]
    s.log = lambda *a: None
    s.refined = None

    def refine(_s):
        return dict(dets={0: [dict(x=100.0, y=100.0, cls="box", score=0.97, source="home", replace_px=10.0)]},
                    visits=[], note="")
    s.run = types.SimpleNamespace(run=lambda *a, **k: ({}, []))
    s._anchors = lambda: {}
    s._reserved_ids = lambda: set()
    s._stage = lambda *a: None
    s._refine(refine)
    assert sorted(d["cls"] for d in s.dets[0]) == ["box", "hand"]        # the hand reaching in is kept
    assert [d["score"] for d in s.dets[0] if d["cls"] == "box"] == [0.97]


# =============================================================================== 4. robot logs: offset and clock
@pytest.fixture(scope="module")
def logs(tmp_path_factory):
    return SL.make_session(str(tmp_path_factory.mktemp("logs")), seconds=60.0, seed=4)


def test_robot_logs_take_a_time_offset(logs):
    ref = RL.read_alvik_logs(logs["logs"], recording=logs["recording"])
    moved = O.read_outside(logs["logs"], "alvik", recording=logs["recording"], offset_s=0.5)   # crashed before
    assert moved.clock_known and ref.clock_known
    n = ref.names()[0]
    assert moved.streams[n].t[0] == pytest.approx(ref.streams[n].t[0] + 0.5, abs=1e-6)
    assert not RL.read_alvik_logs(logs["logs"]).clock_known                    # no video_clock.json, no offset


def test_with_no_clock_file_the_offset_is_found_from_speed_and_said(logs):
    data = RL.read_alvik_logs(logs["logs"])                                    # the camera clock is not known
    tracks, names = {}, {}
    for k, sp in enumerate(logs["robots"]):
        tr = logs["truth"][sp.name]
        tracks[k] = np.column_stack([tr["t"][::3], tr["x"][::3], tr["y"][::3]])
        names[k] = sp.name
    got = outsidelink.find_offset(data, tracks, names)
    assert got["offset_s"] == pytest.approx(SL.PI_OFFSET_S, abs=0.05) and got["used"] >= 2
    assert "worked out from how fast each object moved" in got["text"] and "time offset box" in got["text"]
    none = outsidelink.find_offset(data, {0: tracks[0][:5]}, {0: names[0]})
    assert none["offset_s"] is None and "used as they are" in none["text"]


def test_a_plugin_that_cant_take_an_offset_is_said_plainly(tmp_path, monkeypatch):
    class NoOffset(O.Plugin):
        id = "no_offset_test"

        def read(self, folder, recording=None):
            return O.OutsideData(self.id)
    O.register(NoOffset)
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    with pytest.raises(ValueError) as e:
        outsidelink.read_data(proj, dict(path=str(tmp_path)), str(tmp_path), "no_offset_test", 0.5)
    assert "Clear the time offset box" in str(e.value) and "Nothing was changed" in str(e.value)


# =============================================================================== 5. Line up again
class LockSpy:
    def __init__(self):
        self.held = 0
        self.lock = threading.RLock()

    def acquire(self):
        self.lock.acquire()
        self.held += 1

    def release(self):
        self.lock.release()


def test_line_up_copies_the_labels_under_the_lock_and_never_reopens(tmp_path, monkeypatch):
    spy = LockSpy()
    sess = types.SimpleNamespace(guesses={}, frames={}, lock=spy)
    proj = types.SimpleNamespace()
    monkeypatch.setattr(outsidelink, "read_settings", lambda p, r: dict(path="x"))
    monkeypatch.setattr(outsidelink, "floor_camera", lambda p, r: (None, None))
    monkeypatch.setattr(outsidelink, "no_floor_text", lambda p, r: "no floor")
    with pytest.raises(ValueError):
        outsidelink.build(proj, dict(id="r"), sess)
    # the copy happens after the floor check; check the copy itself with a floor
    monkeypatch.setattr(outsidelink, "floor_camera", lambda p, r: (object(), "cam.json"))
    monkeypatch.setattr(outsidelink, "_frame_times", lambda r: {0: 1.0})

    class Stop(Exception):
        pass

    def read_data(*a):
        raise Stop()
    monkeypatch.setattr(outsidelink, "read_data", read_data)
    with pytest.raises(Stop):
        outsidelink.build(proj, dict(id="r"), sess)
    # the studio: a finished Line up waits for the page to ask before opening again
    st = studiomod.Studio.__new__(studiomod.Studio)
    st.proj = types.SimpleNamespace(data=dict(current=dict(rec="r")))
    st.outside_job = dict(status="done", applied=False, rec="r")
    opened = []
    st.reopen = lambda **kw: opened.append(1) or True
    assert st.outside_apply() is True and opened == [1] and st.outside_job["applied"]
    assert st.outside_apply() is False and opened == [1]                       # once only
    st.outside_job = dict(status="running")
    with pytest.raises(ValueError):
        st.outside_apply()


def test_build_reads_the_session_under_its_lock(tmp_path, monkeypatch):
    spy = LockSpy()
    seen = {}

    class Guesses(dict):
        def items(self):
            seen["held"] = spy.held
            return super().items()
    sess = types.SimpleNamespace(guesses=Guesses({0: [dict(obj=1, x=1.0, y=1.0, cls="box")]}), frames={}, lock=spy)
    monkeypatch.setattr(outsidelink, "read_settings", lambda p, r: dict(path="x"))
    monkeypatch.setattr(outsidelink, "floor_camera", lambda p, r: (object(), "cam.json"))
    monkeypatch.setattr(outsidelink, "_frame_times", lambda r: {0: 1.0})
    monkeypatch.setattr(outsidelink, "read_data", lambda *a: O.OutsideData("csv", {"A": O.Stream("A", [0.0, 1.0], x=[0, 1], y=[0, 1])}))
    proj = types.SimpleNamespace(class_heights=lambda: {}, camera_key=lambda r: "default", class_names=lambda: ["box"],
                                 motion_object_type=lambda: "box", data={})

    class Stop(Exception):
        pass
    monkeypatch.setattr(O, "match_names", lambda *a, **k: (_ for _ in ()).throw(Stop()))
    with pytest.raises(Exception):
        outsidelink.build(proj, dict(id="r"), sess)
    assert seen.get("held") == 1                                               # copied while holding the lock


# =============================================================================== 6. private files and the AI address
def test_import_never_takes_ai_settings_or_logs(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    for name, text in (("ai_settings.json", json.dumps(dict(enabled=True, base_url="https://evil.example"))),
                       ("ai_log.jsonl", "{}\n")):
        with open(os.path.join(proj.folder, name), "w") as f:
            f.write(text)
    out = str(tmp_path / "x.mxirl")
    share.export_project(proj, out)
    with zipfile.ZipFile(out) as z:
        assert not any(n.endswith(("ai_settings.json", "ai_log.jsonl")) for n in z.namelist())
        # a crafted file that does carry them (and a key file, and a temporary copy)
        crafted = str(tmp_path / "crafted.mxirl")
        with zipfile.ZipFile(crafted, "w") as w:
            for n in z.namelist():
                w.writestr(n, z.read(n))
            w.writestr("project/ai_settings.json", json.dumps(dict(enabled=True, base_url="https://evil.example")))
            w.writestr("project/ai_log.jsonl", "{}\n")
            w.writestr("project/ai.json", json.dumps(dict(keys=dict(anthropic="stolen"))))
            w.writestr("project/ai_settings.json.tmp", "{}")
    res = share.import_project(crafted, str(tmp_path / "in"))
    left = os.listdir(str(tmp_path / "in"))
    assert not any(n.startswith(("ai_settings.json", "ai_log.jsonl", "ai.json")) for n in left)
    assert res["dropped"] == sorted(["ai_settings.json", "ai_log.jsonl", "ai.json", "ai_settings.json.tmp"])
    assert any("AI helper starts off" in n or "AI helper off" in n for n in res["notes"])
    assert not ai.AIHelper(str(tmp_path / "in"), user_folder=str(tmp_path / "home")).is_on()


def test_the_ai_address_must_be_https_except_on_this_computer(tmp_path):
    assert ai.check_base_url("") == ""
    assert ai.check_base_url("https://gw.example.com/v1/") == "https://gw.example.com/v1"
    assert ai.check_base_url("http://localhost:8080/v1") == "http://localhost:8080/v1"
    assert ai.check_base_url("http://127.0.0.1:9/v1") == "http://127.0.0.1:9/v1"
    for bad in ("http://gw.example.com/v1", "ftp://x", "gw.example.com", "https://"):
        with pytest.raises(ai.AIError) as e:
            ai.check_base_url(bad)
        assert "Nothing was sent" in str(e.value)
    h = ai.AIHelper(str(tmp_path / "p"), user_folder=str(tmp_path / "home"))
    with pytest.raises(ai.AIError):
        h.configure(provider="openai_compatible", base_url="http://gw.example.com/v1")
    # a settings file edited by hand can't send the key over http either
    os.makedirs(str(tmp_path / "p"), exist_ok=True)
    with open(os.path.join(str(tmp_path / "p"), "ai_settings.json"), "w") as f:
        json.dump(dict(enabled=True, provider="openai_compatible", base_url="http://gw.example.com/v1"), f)
    with pytest.raises(ai.AIError):
        h._adapter()


def test_the_address_is_never_doubled_and_reaches_the_sdk(monkeypatch):
    assert ai.openai_url("https://x/v1") == "https://x/v1/chat/completions"
    assert ai.openai_url("https://x/v1/chat/completions") == "https://x/v1/chat/completions"
    assert ai.anthropic_url("https://gw") == "https://gw/v1/messages"
    assert ai.anthropic_url("https://gw/v1") == "https://gw/v1/messages"
    assert ai.anthropic_url("https://gw/v1/messages/") == "https://gw/v1/messages"
    made = {}

    class Client:
        def __init__(self, **kw):
            made.update(kw)
            self.messages = types.SimpleNamespace(create=lambda **p: types.SimpleNamespace(model_dump_json=lambda: "{}"))
    fake = types.ModuleType("anthropic")
    fake.Anthropic = Client
    for n in ("APIStatusError", "APITimeoutError", "APIConnectionError"):
        setattr(fake, n, type(n, (Exception,), {}))
    monkeypatch.setitem(sys.modules, "anthropic", fake)
    ai.sdk_post("https://gw.example.com/v1/messages", {"x-api-key": "k"}, {})
    assert made["base_url"] == "https://gw.example.com"
    made.clear()
    ai.sdk_post(ai.ANTHROPIC_URL, {"x-api-key": "k"}, {})
    assert "base_url" not in made


def test_ai_suggestions_are_counted_one_by_one(tmp_path):
    h = ai.AIHelper(str(tmp_path / "p"), user_folder=str(tmp_path / "home"))
    os.makedirs(str(tmp_path / "p"), exist_ok=True)
    for cid in ("a", "b", "c"):
        h._log(dict(event="call", id=cid, task="second_check", cost_usd=0.001))
    h.record_decision("a", True, "looked at 1", item="1")
    h.record_decision("a", False, "kept 2", item="2")
    h.record_decision("a", False, "kept 3", item="3")
    h.record_decision("b", None, "no suspects")
    c = studiomod.ai_counts(h.read_log())["tasks"][0]
    assert (c["asked"], c["taken"], c["turned_down"], c["nothing"], c["open"]) == (3, 1, 2, 1, 1)
    assert h.accepted_count() == 1
    html = studiomod.ai_time_saved_html(dict(tasks=[c], cost_usd=0.003))
    assert "Nothing suggested" in html


# =============================================================================== use_model
def _vars(o):
    out = {}
    for k, v in vars(o).items():
        if k in ("rows", "p"):      # p: the settings object (each pipeline has its own, same values)
            continue
        if k == "est":
            out[k] = _vars(v)
        elif hasattr(v, "__iter__") and not isinstance(v, (str, dict)):
            out[k] = list(v)
        else:
            out[k] = v
    return out


def test_live_confirm_then_undo_puts_everything_back():
    def run(cmds):
        pipe = um.Pipeline(lambda img: [], 30.0)
        pipe.check = True
        anchor = pipe.start_from([dict(obj=1, cls="alvik", x=100.0, y=100.0, name="A")], 0, 0.0)
        pipe.process(0, 0.0, "1.0", None, dets=[], anchor=anchor)
        for f in range(1, 12):
            pipe.process(f, f / 10.0, str(1 + f / 10.0), None, dets=[dict(cls="alvik", x=100.0 + 3 * f, y=100.0,
                                                                            score=0.9, heading_deg=None)])
        for c in cmds:
            pipe.queue(c)
        pipe.process(12, 1.2, "2.2", None, dets=[dict(cls="alvik", x=137.0, y=100.0, score=0.9, heading_deg=None)])
        tr = next(t for t in pipe.tk.tracks if t.id == 1)
        return pipe, tr
    plain, t0 = run([])
    undone, t1 = run([dict(kind="confirm", obj=1, x=150.0, y=120.0), dict(kind="undo", obj=1)])
    assert _vars(t1) == _vars(t0)
    assert undone.state == plain.state and undone.last_seen == plain.last_seen


def test_a_recording_without_a_camera_name_uses_the_default_camera(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    assert um.camera_model_paths(proj, "")[0].endswith(os.path.join("cameras", "default.json"))
    assert um.camera_model_paths(proj, "", "/r")[1:] == [os.path.join("/r", "camera.json"),
                                                         os.path.join("/r", "floor_calibration.json")]
    said = []
    pipe = types.SimpleNamespace(cam=None, camera_key="default")
    assert um.load_outside("csv:x.csv", said.append, proj=proj, pipe=pipe) is None
    assert "is NOT used" in said[0] and "\"default\"" in said[0]


def test_outside_data_that_names_nothing_says_why(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    p = str(tmp_path / "pos.csv")
    with open(p, "w") as f:
        f.write("t,name,x,y,frame\n1.0,A,0,0,own\n2.0,A,1,0,own\n")
    said = []
    pipe = types.SimpleNamespace(cam=object(), camera_key="default", heights={}, classes=["box"])
    um.load_outside("csv:" + p, said.append, proj=proj, pipe=pipe)
    assert "A: its own log" in said[0] and "no home positions for camera \"default\"" in said[0]
    # no time on the shared clock: said, not silently skipped
    notes = []
    pl = um.Pipeline(lambda i: [], 30.0)
    pl.outside = um.OutsideDrafts({}, None, {}, {})
    pl._note = notes.append
    um.draft_objects([dict(cls="box", x=1.0, y=1.0, score=0.9)], pl, "")
    assert notes and "no time on the shared clock" in notes[0]


# =============================================================================== training
def _project(tmp_path, rows, n=600):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    proj.set_classes([dict(name="alvik", has_front=True, size_px=30.0)])
    proj.data["settings"]["block_s"] = 5
    fake_recording(str(tmp_path / "r"), n=n)
    rid = proj.add_recording(str(tmp_path / "r"))[0]
    write_labels(proj.labels_dir(rid), rows)
    return proj, rid


def test_precheck_counts_each_part_and_needs_checked_frames_for_picking_and_scoring(tmp_path):
    auto = [[fr, "", "1", "alvik", 100, 100, "", 0, "user", "auto", ""] for fr in range(0, 600, 7)]
    proj, _ = _project(tmp_path, auto)
    pre = trainer.precheck(proj)
    assert not pre["ok"] and pre["counts"]["train"] >= 10 and pre["checked"]["val"] == 0
    assert "auto-accepted" in pre["text"] and "never used for picking or scoring" in pre["text"]
    assert "train.log" not in pre["text"] and "nothing was changed" in pre["text"]
    assert not [n for n in os.listdir(os.path.join(proj.folder, "models")) if n.startswith("_precheck")]
    good = [[fr, "", "1", "alvik", 100, 100, "", 0, "user", "confirmed", ""] for fr in range(0, 600, 7)]
    proj2, _ = _project(tmp_path / "b", good)
    pre = trainer.precheck(proj2)
    assert pre["ok"] and pre["text"].startswith("Frames: ")


def test_a_failed_step_is_explained_without_pointing_at_a_log():
    t = trainer.step_failed_text("train.py", ["epoch 1\n", "Need frames in both train and val.\n"])
    assert "auto-accepted frames are not used for that" in t and "log" not in t
    t = trainer.step_failed_text("evaluate.py", ["Traceback\n", "MemoryError\n"])
    assert "ran out of memory" in t
    t = trainer.step_failed_text("export.py", ["ModuleNotFoundError: No module named 'onnx'\n"])
    assert "onnx" in t and "Install" in t
    t = trainer.step_failed_text("train.py", ["weird thing happened\n"])
    assert "weird thing happened" in t and "Your labels are safe" in t and "log" not in t.lower()
    assert "log" not in trainer.TRAIN_FAILED_NO_MODEL


def test_finds_nothing_yet_gives_the_numbers_it_knows(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    first = dict(id="v1", status="ready", n_train_frames=40, train_minutes=4.6, metrics=dict(recall=0.0))
    proj.data["models"].append(first)
    t = trainer.nothing_found_text(proj, first)
    assert "40 frames for about 5 minutes" in t and "No model in this project has found objects yet" in t
    assert "80" in t and "20 minutes" in t
    good = dict(id="v2", status="ready", n_train_frames=150, train_minutes=20.0, metrics=dict(recall=0.8))
    proj.data["models"].append(good)
    t = trainer.nothing_found_text(proj, dict(id="v3", n_train_frames=60, minutes=10.0))
    assert "model v2 found objects after learning from 150 frames for about 20 minutes" in t


def test_no_model_row_is_scored_on_finding_objects_when_motion_cant_tell_types(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    proj.set_classes([dict(name="gripper", size_px=30.0), dict(name="cup", size_px=30.0)])
    assert trainer.motion_untyped(proj)
    proj2 = projmod.Project(str(tmp_path / "q"), create_name="Q")
    proj2.set_classes([dict(name="alvik", has_front=True, size_px=30.0)])
    assert not trainer.motion_untyped(proj2)



# =============================================================================== real footage (roam1): names stay put
def _speed_stream(name, phase, t):
    v = np.where(np.sin(t / 7.0 + phase) > 0.3, 10.0, 0.0)
    dist = np.concatenate([[0.0], np.cumsum(v[1:] * np.diff(t))])
    return O.Stream(name, t, dist=dist, heading=np.zeros(len(t))), dist


def test_line_up_names_every_object_of_a_stream_and_keeps_the_longest_as_its_main_one():
    # roam1: the person's labels gave each robot several numbers, and each Line up handed a name to whichever
    # number matched best by a hair (often a short stretch), so the labels stopped lining the data up
    t = np.arange(0.0, 200.0, 0.08)
    left, dl = _speed_stream("Left", 0.0, t)
    right, dr = _speed_stream("Right", 2.0, t)
    streams = {"Left": left, "Right": right}

    def track(dist, t0, t1, y=0.0):
        m = (t >= t0) & (t <= t1)
        return np.column_stack([t[m], dist[m], np.full(m.sum(), y)])
    tracks = {1: track(dl, 0, 120), 2: track(dl, 125, 160), 3: track(dl, 50, 60), 4: track(dr, 0, 200, 50.0),
              5: np.column_stack([t, np.full(len(t), 30.0), np.full(len(t), 80.0)])}       # 5: parked all along
    primary = {3: "Left", 4: "Right"}            # what the one-per-name match can pick: a short stretch
    main, every = outsidelink.name_objects(streams, tracks, primary)
    assert main == {1: "Left", 4: "Right"}
    assert every == {1: "Left", 2: "Left", 4: "Right"}       # 3 is on video with 1 at the same time; 5 never moved
