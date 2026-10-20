"""The "use it" tool (use_model.py): follow objects with a trained model, outside the studio.

Fast: a stand-in detector that reads the synthetic recording's truth, plus one tiny untrained
ONNX model for the real model interface. No training.
"""
import csv
import importlib.util
import json
import os
import sys
import threading
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
for p in (REC, os.path.join(TOOLS, "label"), os.path.join(TOOLS, "detector"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import use_model as um  # noqa: E402
import project as projmod  # noqa: E402


def _label_synth():
    spec = importlib.util.spec_from_file_location("label_synth", os.path.join(TOOLS, "label", "tests", "synth.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def recording(tmp_path_factory):
    """6 seconds (180 frames) of the synthetic 'main' scene, with truth.csv."""
    out = str(tmp_path_factory.mktemp("rec") / "20261020-test_E1")
    _label_synth().make_recording(out, "main", seconds=6.0, seed=3)
    return out


def _truth(folder):
    by = {}
    with open(os.path.join(folder, "truth.csv"), newline="") as f:
        for r in csv.DictReader(f):
            if r["cls"] == "alvik":
                by.setdefault(int(r["frame"]), []).append(dict(cls="alvik", x=float(r["x"]), y=float(r["y"]),
                                                               heading_deg=float(r["heading_deg"]), score=0.9))
    return by


class StubDetector:
    """Same interface as infer.predict(model, frame): called once per frame, in order."""

    def __init__(self, folder):
        self.truth = _truth(folder)
        self.n = 0

    def __call__(self, frame):
        out = [dict(d) for d in self.truth.get(self.n, [])]
        self.n += 1
        return out


def _read(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _new_project(folder):
    """A project with one object type that has a front (set explicitly: new projects start with none)."""
    proj = projmod.Project(folder, create_name="demo")
    proj.data["classes"] = [dict(name="alvik", has_front=True, size_px=None, height_cm=4.0),
                            dict(name="unknown", has_front=False, size_px=None, height_cm=0.0)]
    proj.save()
    return proj


def _pipe(recording, **kw):
    return um.Pipeline(StubDetector(recording), 30.0, (640, 480), **kw)


# --------------------------------------------------------------------------------------------
def test_confidence_colours():
    assert um.confidence_colour(0.9, True, 40) == "green"
    assert um.confidence_colour(0.9, True, 3) == "amber"          # too new to be sure
    assert um.confidence_colour(0.9, False, 40) == "amber"        # not found in this very picture
    assert um.confidence_colour(0.5, True, 40) == "amber"
    assert um.confidence_colour(0.2, True, 40) == "red"
    assert um.confidence_colour(0.1, False, 40) == "red"


def test_area_polygon_margin_and_spots():
    square = [[100, 100], [300, 100], [300, 300], [100, 300]]
    a = um.Area(square, margin_px=0, spots=[[200, 200, 10]])
    assert a.keep(150, 150) and not a.keep(50, 50) and not a.keep(203, 205)   # inside, outside, ignore spot
    assert um.Area(square, margin_px=30).keep(80, 200)                        # outside but within the margin
    assert not um.Area(square, margin_px=30).keep(50, 200)
    assert um.Area(None).keep(5, 5) and not um.Area(None).active
    assert um.Area([[1, 1], [2, 2]]).polygon is None                          # fewer than 3 corners = whole picture


def test_source_kinds_and_url():
    assert um.classify_source("0") == "camera"
    assert um.classify_source("http://st1maxfini.local:8000") == "url"
    assert um.classify_source("st1maxfini.local:8001") == "url"
    assert um.classify_source("/no/such/thing") is None
    assert um.normalise_url("st1maxfini.local:8000") == "http://st1maxfini.local:8000/stream"
    assert um.normalise_url("http://pi:8000/") == "http://pi:8000/stream"
    assert um.normalise_url("http://pi:8000/live") == "http://pi:8000/live"


def test_offline_recording_table(recording, tmp_path):
    out = str(tmp_path / "t.csv")
    src = um.open_source(recording)
    assert src.kind == "recording" and src.n == 180 and not src.live
    pipe = _pipe(recording)
    n = um.run(src, pipe, out, say=lambda *_: None)
    src.close()
    rows = _read(out)
    assert n == len(rows) > 400
    assert list(rows[0].keys()) == ["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg", "score", "source"]
    assert {r["cls"] for r in rows} == {"alvik"}
    assert len({r["obj"] for r in rows}) == 3                       # three robots, three steady numbers
    assert rows[0]["t_wall"].startswith("17600000")                 # from the recording's frames.csv
    frames = [int(r["frame"]) for r in rows]
    assert frames == sorted(frames) and min(frames) == 0            # the first frames of each object are filled in
    # positions agree with the truth to a few pixels
    truth = _truth(recording)
    errs = []
    for r in rows:
        if r["source"] != "model":
            continue
        best = min(((d["x"] - float(r["x_px"])) ** 2 + (d["y"] - float(r["y_px"])) ** 2) ** 0.5
                   for d in truth[int(r["frame"])])
        errs.append(best)
    assert errs and max(errs) < 5.0
    # the model's own heading is passed through
    assert sum(1 for r in rows if r["heading_deg"] != "") > len(rows) // 2
    assert all(0.0 <= float(r["score"]) <= 1.0 for r in rows)


def test_work_area_and_ignore_spot(recording, tmp_path):
    def table(area):
        out = str(tmp_path / "a.csv")
        src = um.open_source(recording)
        um.run(src, _pipe(recording, area=area), out, say=lambda *_: None)
        src.close()
        return _read(out)
    full = table(None)
    left = table(um.Area([[0, 0], [320, 0], [320, 480], [0, 480]], margin_px=0))
    assert 0 < len(left) < len(full)
    assert all(float(r["x_px"]) <= 320 + 30 for r in left)          # only the tracker's coasting can stray a little
    # an ignore spot on a robot's start removes what it covered
    t0 = _truth(recording)[0]
    spot = t0[0]
    cut = table(um.Area(None, 0, [[spot["x"], spot["y"], 60]]))
    assert len(cut) < len(full)


def test_floor_positions_when_calibrated(recording, tmp_path):
    from synth import cam_overhead
    proj = _new_project(str(tmp_path / "proj"))
    proj.add_recording(recording, camera="over")
    cam = cam_overhead()
    cam.save(proj.camera_model_path("over"))
    loaded = um.load_camera_model(proj, "over")
    assert loaded is not None
    assert um.pick_camera_name(proj, recording, None) == "over"
    pipe = _pipe(recording, camera_model=loaded, heights=um.heights_for(proj))
    out = str(tmp_path / "cm.csv")
    src = um.open_source(recording)
    um.run(src, pipe, out, max_frames=60, say=lambda *_: None)
    src.close()
    rows = _read(out)
    assert list(rows[0].keys())[-3:] == ["x_cm", "y_cm", "source"]
    r = next(r for r in rows if r["source"] == "model")
    z = um.heights_for(proj)["alvik"]
    x, y = cam.to_floor(float(r["x_px"]), float(r["y_px"]), z)
    assert abs(float(r["x_cm"]) - x) < 0.1 and abs(float(r["y_cm"]) - y) < 0.1
    assert um.load_camera_model(proj, "nothing") is None


def test_project_model_choice(tmp_path):
    proj = _new_project(str(tmp_path / "p"))
    with pytest.raises(um.UserError):
        um.resolve_model(proj)                                      # nothing trained yet: plain message
    for mid, f1 in (("v1", 0.5), ("v2", 0.9), ("v3", 0.7)):
        d = proj.model_dir(mid)
        os.makedirs(d)
        open(os.path.join(d, "model.onnx"), "w").close()
        proj.data["models"].append(dict(id=mid, status="ready", onnx="models/%s/model.onnx" % mid, metrics=dict(f1=f1)))
    proj.data["models"].append(dict(id="v4", status="failed"))
    proj.data["current"]["model"] = "none"
    assert um.resolve_model(proj)[0] == "v2"                        # none chosen: the best scored
    proj.data["current"]["model"] = "v3"
    assert um.resolve_model(proj)[0] == "v3"                        # the studio's choice
    assert um.resolve_model(proj, "v1")[0] == "v1"                  # asked for
    with pytest.raises(um.UserError):
        um.resolve_model(proj, "v4")


def test_real_onnx_model_through_the_command_line(recording, tmp_path):
    torch = pytest.importorskip("torch")
    import export
    from model import CentreNet, default_config
    cfg = default_config(width=0.5)
    cfg["score_thresh"] = 0.3
    pt, onnx = str(tmp_path / "m.pt"), str(tmp_path / "m.onnx")
    torch.save(dict(model=CentreNet(cfg).state_dict(), config=cfg), pt)
    export.export(pt, onnx)
    out = str(tmp_path / "onnx.csv")
    code = um.main([recording, "--onnx", onnx, "--out", out, "--max-frames", "12", "--no-page", "--min-score", "0.0"])
    assert code == 0
    rows = _read(out)
    with open(out) as f:
        assert f.readline().strip().split(",") == um.BASE_COLUMNS + ["source"]
    assert um.main([recording, "--out", out, "--no-page"]) == 1     # no model given: a plain message, not a crash


def test_project_run_uses_its_model_and_work_area(recording, tmp_path):
    """--project: the chosen model is used, the table goes to the project's tracks folder."""
    torch = pytest.importorskip("torch")
    import export
    from model import CentreNet, default_config
    proj = _new_project(str(tmp_path / "proj"))
    rid = proj.add_recording(recording, camera="")[0]
    cfg = default_config(width=0.5, classes=proj.class_names(), heading_classes=proj.heading_classes())
    cfg["score_thresh"] = 0.3
    d = proj.model_dir("v1")
    os.makedirs(d)
    pt = os.path.join(d, "best.pt")
    torch.save(dict(model=CentreNet(cfg).state_dict(), config=cfg), pt)
    export.export(pt, os.path.join(d, "model.onnx"))
    proj.data["models"].append(dict(id="v1", status="ready", onnx="models/v1/model.onnx", metrics={}))
    proj.data["cameras"][""] = dict(work_area=[[0, 0], [640, 0], [640, 480], [0, 480]], margin_px=0, ignore_spots=[[5, 5, 3]])
    proj.save()
    code = um.main([rid, "--project", proj.folder, "--max-frames", "8", "--no-page", "--min-score", "0.0"])
    assert code == 0
    made = os.listdir(os.path.join(proj.folder, "tracks"))
    assert len(made) == 1 and made[0].endswith("_v1.csv")


def test_live_source_streams_rows_as_they_happen(recording, tmp_path):
    """A camera-like source (live): rows are written while it runs, and the Stop button ends it."""
    import cv2

    class FakeLive(um.Source):
        live = True
        label = "fake camera"
        kind = "camera"

        def __init__(self):
            self.cap = cv2.VideoCapture(os.path.join(recording, "video.avi"))
            self.i = 0

        def read(self):
            ok, img = self.cap.read()
            if not ok:
                return None
            self.i += 1
            return self.i - 1, (self.i - 1) / 30.0, "%.3f" % (1.76e9 + self.i / 30.0), img

    view = um.LiveView(0, "fake camera")
    out = str(tmp_path / "live.csv")
    pipe = _pipe(recording)
    stop_at = 60
    real_process = pipe.process

    def process(f, t, tw, img):
        if f == stop_at:
            req = urllib.request.Request(view.url + "stop", data=b"", method="POST")
            urllib.request.urlopen(req, timeout=5).read()            # the page's Stop button
        return real_process(f, t, tw, img)
    pipe.process = process
    try:
        um.run(FakeLive(), pipe, out, view=view, say=lambda *_: None)
    finally:
        view.close()
    rows = _read(out)
    assert view.stop_event.is_set()
    assert 0 < len(rows) and max(int(r["frame"]) for r in rows) <= stop_at + 1      # stopped when asked
    assert max(int(r["frame"]) for r in rows) >= stop_at - 1


def test_live_page(recording):
    import numpy as np
    view = um.LiveView(0, "camera 0")
    try:
        page = urllib.request.urlopen(view.url, timeout=5).read().decode("utf-8")
        assert "Maxfield IRL Tracking" in page and "Identify. Recognize. Label." in page
        assert "frames per second" in page and "Stop" in page
        for colour in ("green", "amber", "red"):
            assert colour in page
        assert "in real life" not in page.lower() and "alvik" not in page.lower()
        # nothing yet: the page is told there is no news
        s = json.loads(urllib.request.urlopen(view.url + "state?since=-1", timeout=5).read())
        assert s.get("same")
        img = np.zeros((480, 640, 3), np.uint8)
        objs = [dict(obj=1, cls="thing", x_px=100.0, y_px=200.0, heading_deg=45.0, score=0.8, colour="green"),
                dict(obj=2, cls="thing", x_px=300.0, y_px=100.0, heading_deg=None, score=0.2, colour="red")]
        threading.Timer(0.2, view.publish, args=(7, img, objs, 12.5)).start()      # a frame arrives while the page waits
        s = json.loads(urllib.request.urlopen(view.url + "state?since=-1", timeout=5).read())
        assert s["frame"] == 7 and s["fps"] == 12.5 and s["image"].startswith("data:image/jpeg;base64,")
        assert [o["colour"] for o in s["objects"]] == ["green", "red"]
        assert not view.stop_event.is_set()
        urllib.request.urlopen(urllib.request.Request(view.url + "stop", data=b"", method="POST"), timeout=5).read()
        assert view.stop_event.is_set()
        view.finish("done")
        s = json.loads(urllib.request.urlopen(view.url + "state?since=7", timeout=5).read())
        assert s["done"] and s["message"] == "done"
    finally:
        view.close()


def test_plain_questions(recording, tmp_path):
    proj = _new_project(str(tmp_path / "proj"))
    proj.add_recording(recording, camera="")
    for mid in ("v1", "v2"):
        d = proj.model_dir(mid)
        os.makedirs(d)
        open(os.path.join(d, "model.onnx"), "w").close()
        proj.data["models"].append(dict(id=mid, status="ready", onnx="models/%s/model.onnx" % mid, metrics={}))
    proj.save()
    answers = iter([str(tmp_path / "nowhere"),      # a project that does not exist: asked again
                    proj.folder,
                    "v9", "v1",                      # a model that does not exist: asked again
                    "7", "1",                        # not 1-4, then a recording
                    "5", "1",                        # not in the list, then the first recording
                    "",                              # live page: default (no, for a recording)
                    ""])                             # let it choose the file name
    said = []
    args = um.ask_questions(lambda prompt: (said.append(prompt), next(answers))[1], lambda *m: said.append(" ".join(m)))
    assert args[0] == recording
    assert ["--project", proj.folder] == args[1:3] and "--model" in args and args[args.index("--model") + 1] == "v1"
    assert "--no-page" in args and "--out" not in args
    text = "\n".join(said)
    assert "Maxfield IRL Tracking" in text and "There is no project" in text
    assert "in real life" not in text.lower() and "alvik" not in text.lower()
    # a live camera, with the page and a time limit
    answers = iter(["0", str(tmp_path / "m.onnx"), "3", "", "", "30", "out.csv"])
    open(str(tmp_path / "m.onnx"), "w").close()
    args = um.ask_questions(lambda p: next(answers), lambda *m: None)
    assert args[0] == "0" and "--onnx" in args and "--page" in args and args[args.index("--seconds") + 1] == "30"
    assert args[args.index("--out") + 1] == "out.csv"


def test_bat_file_exists_and_calls_the_tool():
    bat = os.path.join(os.path.dirname(TOOLS), "track-with-model.bat")
    text = open(bat).read()
    assert "use_model.py" in text and "%*" in text and "Maxfield IRL Tracking" in text and "pause" in text
    assert "alvik" not in text.lower() and "in real life" not in text.lower()
