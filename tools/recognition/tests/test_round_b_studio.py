"""Fix round B, studio side: an ordinary video file made into a recording, failed models (hidden,
deleted, left out of shared files), the learning curve in model order with a caution, blank
heights, the "unknown" note for several types, the parts of the split, launchers and wording."""
import csv
import json
import os
import sys
import time
import zipfile

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
REPO = os.path.dirname(TOOLS)
for p in (REC, os.path.join(TOOLS, "label"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import project as projmod  # noqa: E402
import report  # noqa: E402
import share  # noqa: E402
import studio as studiomod  # noqa: E402
import trainer  # noqa: E402
import videofile  # noqa: E402
from test_studio import fake_recording  # noqa: E402


def phone_video(path, seconds=2.0, fps=15.0, size=(1280, 720)):
    """A small mp4 like a phone's: a dark square crossing a grey table."""
    w, h = size
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    assert vw.isOpened()
    rng = np.random.default_rng(2)
    base = cv2.GaussianBlur(rng.integers(100, 150, (h, w, 3), dtype=np.uint8), (7, 7), 0)
    n = int(seconds * fps)
    for f in range(n):
        img = base.copy()
        x = int(100 + (w - 200) * f / max(1, n - 1))
        cv2.rectangle(img, (x, h // 2 - 40), (x + 80, h // 2 + 40), (30, 30, 30), -1)
        vw.write(img)
    vw.release()
    return n


# ---------------------------------------------------------------- video files
def test_video_file_becomes_a_recording_folder(tmp_path):
    src = str(tmp_path / "Videos" / "table test.mp4")
    os.makedirs(os.path.dirname(src))
    n = phone_video(src)
    seen = []
    res = videofile.convert(src, progress=lambda d, t: seen.append((d, t)))
    folder = res["folder"]
    assert folder == str(tmp_path / "Videos" / "table test") and not res["reused"]
    assert os.path.isfile(os.path.join(folder, "video.avi"))
    assert seen and seen[-1][0] == seen[-1][1] == res["frames"]
    # the picture is made smaller (long side 960), the frames are all there, each with its time
    assert res["size"] == [960, 540] and res["scaled_from"] == [1280, 720]
    assert abs(res["frames"] - n) <= 1
    with open(os.path.join(folder, "frames.csv"), newline="") as f:
        rows = list(csv.DictReader(f))
    t = [float(r["t_mono"]) for r in rows]
    assert len(rows) == res["frames"] and t[0] == 0.0 and all(b > a for a, b in zip(t, t[1:]))
    assert abs(t[-1] - (len(t) - 1) / 15.0) < 0.05
    with open(os.path.join(folder, "meta.json")) as f:
        meta = json.load(f)
    assert meta["width"] == 960 and meta["height"] == 540 and abs(meta["fps_reported"] - 15.0) < 0.1
    assert meta["source_file"] == os.path.abspath(src)
    # the studio's own reader opens it, at the converted size
    import app
    rec = app.Recording(folder)
    assert rec.n == res["frames"] and (rec.width, rec.height) == (960, 540)
    # short: it says a few minutes work better
    assert res["notes"] and "few minutes" in res["notes"][0]
    # the same file again: the folder is reused, nothing is made twice
    again = videofile.convert(src)
    assert again["reused"] and again["folder"] == folder
    assert sorted(os.listdir(str(tmp_path / "Videos"))) == ["table test", "table test.mp4"]


def test_video_folder_goes_to_the_fallback_when_next_to_it_cant_be_written(tmp_path, monkeypatch):
    src = str(tmp_path / "ro" / "clip.mov")
    os.makedirs(os.path.dirname(src))
    phone_video(src, seconds=1.0, size=(320, 240))
    real = videofile._writable
    monkeypatch.setattr(videofile, "_writable", lambda d: False if os.path.abspath(d) == str(tmp_path / "ro") else real(d))
    res = videofile.convert(src, str(tmp_path / "proj" / "recordings"))
    assert res["folder"] == str(tmp_path / "proj" / "recordings" / "clip")
    assert res["size"] == [320, 240] and res["scaled_from"] is None


def test_unreadable_video_says_so_plainly_and_leaves_nothing(tmp_path):
    bad = tmp_path / "broken.mp4"
    bad.write_bytes(b"this is not a video" * 100)
    with pytest.raises(videofile.ConvertError) as e:
        videofile.convert(str(bad))
    assert "MP4" in str(e.value)
    assert sorted(os.listdir(str(tmp_path))) == ["broken.mp4"]


def test_stopping_a_conversion_removes_the_half_made_folder(tmp_path):
    src = str(tmp_path / "long.mp4")
    phone_video(src, seconds=2.0, size=(320, 240))
    calls = [0]

    def stop():
        calls[0] += 1
        return calls[0] > 5
    with pytest.raises(InterruptedError):
        videofile.convert(src, stop=stop)
    assert sorted(os.listdir(str(tmp_path))) == ["long.mp4"]


def _wait(cond, timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.1)
    return False


def test_add_recording_with_a_video_file_converts_adds_and_opens(tmp_path):
    src = str(tmp_path / "clip.mp4")
    phone_video(src, seconds=2.0, size=(320, 240))
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    assert st.add(src) == dict(converting=True)
    assert _wait(lambda: st.convert and st.convert["status"] != "running")
    job = st.convert_view()
    assert job["status"] == "done", job
    assert job["result"]["added"] == 1 and "stop" not in job
    rec = proj.rec(job["result"]["ids"][0])
    assert rec["path"] == str(tmp_path / "clip") and rec["name"] == "clip"
    assert _wait(lambda: st.session is not None and not st.loading)
    assert st.state()["convert"]["status"] == "done"
    # not a video, not a folder: a plain message
    other = tmp_path / "notes.txt"
    other.write_text("x")
    with pytest.raises(ValueError) as e:
        st.add(str(other))
    assert "not a video file" in str(e.value)
    with pytest.raises(ValueError):
        st.add("   ")
    # the project's own add_recording explains a file too
    with pytest.raises(ValueError) as e:
        proj.add_recording(src)
    assert "video file" in str(e.value)


# ---------------------------------------------------------------- failed models
def _model(proj, mid, status="ready", warning=None, frames=20, f1=0.5):
    os.makedirs(os.path.join(proj.model_dir(mid), "run"), exist_ok=True)
    with open(os.path.join(proj.model_dir(mid), "model.onnx"), "wb") as f:
        f.write(b"x" * 100)
    m = dict(id=mid, status=status, classes=proj.class_names(), n_train_frames=frames, created=projmod.now_iso(),
             metrics=dict(f1=f1, recall=f1, precision=f1), onnx="models/%s/model.onnx" % mid)
    if warning:
        m["warning"] = warning
    if status == "failed":
        m["error"] = "it stopped"
    proj.data["models"].append(m)
    proj.save()
    return m


def _curve(proj, pts, frames=50):
    projmod.write_json(os.path.join(proj.folder, "models", "curve.json"),
                       dict(points=pts, test=dict(frames=frames, blocks=3, block_s=30, note="3 time blocks")))


def _pt(mid, n, f1, objs=8):
    return dict(model=mid, label="No model (motion)" if mid == "none" else "Model " + mid, n_train_frames=n,
                precision=f1, recall=f1, f1=f1, centre_err_px=2.0, heading_err_deg=None, ms_per_frame=5.0,
                n_labels=40, n_objects=objs)


def test_failed_models_are_marked_deleted_and_never_reuse_a_number(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    _model(proj, "v1")
    _model(proj, "v2", warning=trainer.NOTHING_FOUND, f1=0.0)
    _model(proj, "v3", status="failed")
    _curve(proj, [_pt("none", 0, 0.2), _pt("v1", 20, 0.5), _pt("v2", 20, 0.0)])
    proj.data["current"]["model"] = "v2"
    proj.save()
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    flags = {m["id"]: m["failed"] for m in st.state()["models"]}
    assert flags == dict(v1=False, v2=True, v3=True)
    c = st.curve()
    assert [p["model"] for p in c["points"]] == ["none", "v1", "v2"] and c["points"][2]["failed"]
    # a working model can't be deleted
    with pytest.raises(ValueError):
        proj.delete_model("v1")
    st.delete_model("v3")
    assert not os.path.exists(proj.model_dir("v3"))
    st.delete_model("v2")                      # the one giving the guesses: back to no model
    assert proj.data["current"]["model"] == "none"
    assert [m["id"] for m in projmod.Project(proj.folder).models()] == ["v1"]
    assert [p["model"] for p in st.curve()["points"]] == ["none", "v1"]
    # the next model is v4, not v2 or v3 again (the time record names models by number)
    assert proj.new_model_id() == "v4"


def test_share_leaves_failed_models_out_unless_asked(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    _model(proj, "v1")
    _model(proj, "v2", warning=trainer.NOTHING_FOUND, f1=0.0)
    _model(proj, "v3", status="failed")
    _curve(proj, [_pt("none", 0, 0.2), _pt("v1", 20, 0.5), _pt("v2", 20, 0.0)])
    os.makedirs(os.path.join(proj.folder, "recordings", "clip"))
    with open(os.path.join(proj.folder, "recordings", "clip", "video.avi"), "wb") as f:
        f.write(b"v" * 1000)
    out = share.export_project(proj, str(tmp_path / "out.mxirl"))
    with zipfile.ZipFile(out["path"]) as z:
        names = z.namelist()
        pj = json.loads(z.read("project/project.json"))
        curve = json.loads(z.read("project/models/curve.json"))
        man = json.loads(z.read("manifest.json"))
    assert [m["id"] for m in pj["models"]] == ["v1"]
    assert not any(n.startswith("project/models/v2/") or n.startswith("project/models/v3/") for n in names)
    assert any(n.startswith("project/models/v1/") for n in names)
    assert [p["model"] for p in curve["points"]] == ["none", "v1"]
    assert man["left_out_models"] == ["v2", "v3"] and man["contents"]["models"] == 1
    assert not any(n.startswith("project/recordings/") for n in names)      # videos only with_video
    # the project itself is untouched
    assert [m["id"] for m in projmod.Project(proj.folder).models()] == ["v1", "v2", "v3"]
    out2 = share.export_project(proj, str(tmp_path / "all.mxirl"), include_failed=True)
    with zipfile.ZipFile(out2["path"]) as z:
        assert [m["id"] for m in json.loads(z.read("project/project.json"))["models"]] == ["v1", "v2", "v3"]


# ---------------------------------------------------------------- learning curve
def test_curve_is_in_model_order_and_warns_when_it_rests_on_little(tmp_path):
    assert sorted(["v10", "v2", "none", "v1"], key=trainer.version_order) == ["none", "v1", "v2", "v10"]
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    _model(proj, "v1")
    _model(proj, "v2")
    # v1 and v2 trained on the same frames: they stand side by side, in order
    _curve(proj, [_pt("v2", 20, 0.6), _pt("none", 0, 0.2), _pt("v1", 20, 0.5)], frames=12)
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    c = st.curve()
    assert [p["model"] for p in c["points"]] == ["none", "v1", "v2"]
    assert c["caution"] and "only 12 held-back frames" in c["caution"]
    svg = report._curve_svg(c["points"])
    xs = [float(x) for x in __import__("re").findall(r'<circle cx="([\d.]+)"', svg)][:3]
    assert len(set(xs)) == 3                        # no two points on one vertical line
    assert "20 frames" in svg and "no model" in svg
    html = report.html_report(proj)
    assert "Caution:" in html
    # enough frames and objects: no caution
    _curve(proj, [_pt("none", 0, 0.2), _pt("v1", 20, 0.5)], frames=80)
    assert st.curve()["caution"] is None
    _curve(proj, [_pt("none", 0, 0.2, objs=2), _pt("v1", 20, 0.5, objs=3)], frames=80)
    assert "only 3 different objects" in st.curve()["caution"]


def test_report_has_no_empty_brackets_without_a_note(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    projmod.write_json(os.path.join(proj.folder, "models", "curve.json"), dict(points=[], test={}))
    assert "()" not in report.html_report(proj)


# ---------------------------------------------------------------- object types
def test_blank_height_is_kept_blank_and_not_passed_between_cameras(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    proj.set_classes([dict(name="cube", height_cm=None), dict(name="peg", height_cm=""), dict(name="cup", height_cm=0),
                      dict(name="hand", height_cm=None, height_varies=True)])
    by = {c["name"]: c for c in projmod.Project(proj.folder).classes}
    assert by["cube"]["height_cm"] is None and by["peg"]["height_cm"] is None
    assert by["cup"]["height_cm"] == 0.0                   # 0 typed: on the floor
    assert by["hand"]["height_cm"] == 0.0 and by["hand"]["height_varies"]
    assert proj.no_height_classes() == ["cube", "peg"]
    assert proj.class_heights()["cube"] == 0.0             # readers that need a number still get one
    with pytest.raises(ValueError):
        proj.set_classes([dict(name="cube", height_cm=-3)])


def test_unknown_note_when_two_plain_types_cant_be_told_apart(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    before = proj.motion_object_type()
    proj.set_classes([dict(name="cube")])
    assert studiomod._unknown_note(before, proj) is None
    before = proj.motion_object_type()
    proj.set_classes([dict(name="cube"), dict(name="cylinder")])
    note = studiomod._unknown_note(before, proj)
    assert note and "unknown" in note and "cube, cylinder" in note
    # already that way before the change: said once, not every save
    assert studiomod._unknown_note(proj.motion_object_type(), proj) is None
    assert studiomod._unknown_note(studiomod._NO_BEFORE, proj)
    proj.set_classes([c for c in projmod.PRESETS[0]["classes"]])
    assert studiomod._unknown_note(studiomod._NO_BEFORE, proj) is None


def test_state_gives_recording_paths_and_the_number_of_cameras(tmp_path):
    fake_recording(str(tmp_path / "r1"))
    fake_recording(str(tmp_path / "r2"))
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    proj.add_recording(str(tmp_path / "r1"), "overhead")
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    s = st.state()
    assert s["recordings"][0]["path"] == str(tmp_path / "r1") and s["n_cameras"] == 1
    proj.add_recording(str(tmp_path / "r2"), "side")
    assert st.state()["n_cameras"] == 2


def test_import_with_an_empty_file_box_says_so(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    with pytest.raises(ValueError) as e:
        st.import_file("  ", str(tmp_path / "new"))
    assert "(empty)" not in str(e.value) and "box is empty" in str(e.value)


# ---------------------------------------------------------------- the split's parts
def test_part_function_matches_the_training_split(tmp_path):
    fake_recording(str(tmp_path / "r"), n=1800, fps=10.0)     # 3 minutes
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    rid = proj.add_recording(str(tmp_path / "r"))[0]
    rec = proj.rec(rid)
    part = trainer.part_function(proj, rec)
    assert part.block_s == trainer.choose_block_s(proj)
    # the same cut as build_dataset: write labels on every 5th frame and compare
    from test_studio import write_labels
    rows = [[f, "", 1, "unknown", "10.0", "10.0", "", 0, "user", "confirmed", ""] for f in range(0, 1800, 5)]
    write_labels(proj.labels_dir(rid), rows)
    assign, info = trainer.build_dataset(proj, str(tmp_path / "data"), part.block_s, trainer.guard_for(part.block_s))
    by_frame = {}
    for name, i in info.items():
        with open(os.path.join(str(tmp_path / "data"), name, "labels.csv"), newline="") as f:
            for r in csv.DictReader(f):
                by_frame[int(r["frame"])] = assign[name]
    for f in range(0, 1800, 5):
        assert part(f) == by_frame.get(f), f
    assert {part(f) for f in range(0, 1800, 5)} >= {"train", "val", "test", None}


# ---------------------------------------------------------------- launchers and wording
def test_launchers_try_py_then_python_and_say_plainly_when_neither_works():
    for name, script in (("start-recognition.bat", "start.py"), ("track-with-model.bat", "use_model.py")):
        with open(os.path.join(REPO, name), "rb") as f:
            raw = f.read()
        assert b"\r\n" in raw and b"\n" not in raw.replace(b"\r\n", b"")      # Windows line ends throughout
        text = raw.decode("ascii")
        assert text.index("py -3") < text.index('python -c')
        assert 'Python not found: install it from python.org and tick "Add python.exe to PATH".' in text
        assert "pause" in text.split(":nopython")[1]
        assert '%%MXPY%% "%%~dp0tools\\recognition\\%s" %%*' % script in text


def test_page_wording():
    with open(os.path.join(TOOLS, "label", "page.html"), encoding="utf-8") as f:
        page = f.read()
    for gone in ("middle value of every picture", "robot arm's reach", "95% range", "(empty)", "in real life"):
        assert gone not in page, gone
    assert "Show the full tour" in page and "Show failed models" in page and 'id="keysbtn"' in page
