"""Project as one file: export/import round trip, re-linking, safety."""
import json
import os
import sys
import zipfile

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
for p in (REC, os.path.join(TOOLS, "label"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import project as projmod  # noqa: E402
import share  # noqa: E402
from test_studio import fake_recording, write_labels  # noqa: E402


def tree(folder, skip=()):
    out = {}
    for r, _, fs in os.walk(folder):
        for f in fs:
            full = os.path.join(r, f)
            rel = os.path.relpath(full, folder).replace(os.sep, "/")
            if not rel.startswith(skip):
                out[rel] = open(full, "rb").read()
    return out


def make_project(tmp_path):
    raw = tmp_path / "raw"
    sess = raw / "20261019-210000_E1"
    fake_recording(str(sess / "cam0"))
    fake_recording(str(sess / "cam1"))
    with open(sess / "session.json", "w") as f:
        json.dump(dict(cameras=[dict(folder="cam0", role="overhead"), dict(folder="cam1", role=None)]), f)
    single = raw / "solo-run"
    fake_recording(str(single))
    proj = projmod.Project(str(tmp_path / "proj"), create_name="Shared Thing")
    ids = proj.add_recording(str(sess)) + proj.add_recording(str(single))
    proj.data["settings"]["train_minutes"] = 7
    proj.data["cameras"] = {"overhead": {"margin_px": 12}}
    proj.save()
    row = [0, 1000.0, 1, "alvik", 10, 20, 0, 1, "user", "confirmed", 1.0]
    write_labels(proj.labels_dir(ids[0]), [row])
    with open(os.path.join(proj.labels_dir(ids[0]), "review.json"), "w") as f:
        json.dump({"0": "done"}, f)
    md = proj.model_dir("v1")
    os.makedirs(os.path.join(md, "run"))
    open(os.path.join(md, "model.onnx"), "wb").write(b"onnx-bytes" * 100)
    open(os.path.join(md, "run", "best.pt"), "wb").write(b"pt")
    proj.data["models"].append(dict(id="v1", status="ready", classes=proj.class_names()))
    proj.save()
    projmod.write_json(os.path.join(proj.folder, "models", "curve.json"), dict(points=[1, 2]))
    projmod.write_json(os.path.join(proj.folder, "cameras", "overhead.json"), dict(k=1))
    os.makedirs(os.path.join(proj.folder, "stats"))
    open(os.path.join(proj.folder, "stats", "x.jsonl"), "w").write("{}\n")
    return proj, ids, raw


def test_round_trip_without_video(tmp_path):
    proj, ids, raw = make_project(tmp_path)
    before = tree(proj.folder)
    seen = []
    res = share.export_project(proj, str(tmp_path / "out" / "x.mxirl"), progress=lambda d, t: seen.append((d, t)))
    assert tree(proj.folder) == before                      # the source is untouched
    assert seen and seen[-1][0] == seen[-1][1]
    assert all(a <= b for a, b in zip([s[0] for s in seen], [s[0] for s in seen][1:]))
    m = share.read_manifest(res["path"])
    assert m["format"] == "mxirl" and m["project_name"] == "Shared Thing" and not m["with_video"]
    assert len(m["recordings"]) == 3 and not any(r["video_included"] for r in m["recordings"])
    assert m["contents"]["models"] == 1 and m["contents"]["has_learning_curve"]
    with zipfile.ZipFile(res["path"]) as zf:
        assert not any(n.endswith(".avi") for n in zf.namelist())

    got = share.import_project(res["path"], str(tmp_path / "copy"), recordings_dir=str(raw))
    assert tree(str(tmp_path / "copy")) == before           # labels, models, cameras, settings identical
    assert got["found"] == 3 and got["missing"] == [] and got["notes"] == []
    p2 = projmod.Project(str(tmp_path / "copy"))
    assert p2.data["settings"]["train_minutes"] == 7
    assert [r["path"] for r in p2.recordings()] == [r["path"] for r in proj.recordings()]


def test_missing_recordings_are_reported_and_project_opens(tmp_path):
    proj, ids, raw = make_project(tmp_path)
    f = share.export_project(proj, str(tmp_path / "a.mxirl"))["path"]
    elsewhere = tmp_path / "other"
    os.makedirs(elsewhere / "20261019-210000_E1")
    # only cam0 of the session was copied
    import shutil
    shutil.copytree(raw / "20261019-210000_E1" / "cam0", elsewhere / "20261019-210000_E1" / "cam0")
    # make the original paths stale so only the search can find things
    d = json.load(open(os.path.join(proj.folder, "project.json")))
    for r in d["recordings"]:
        r["path"] = "C:\\gone\\" + r["path"].replace("/", "\\").split("\\")[-1]
    projmod.write_json(os.path.join(proj.folder, "project.json"), d)
    f = share.export_project(proj, str(tmp_path / "b.mxirl"))["path"]
    got = share.import_project(f, str(tmp_path / "c"), recordings_dir=str(elsewhere))
    assert got["found"] == 1 and len(got["missing"]) == 2
    assert all(line.startswith("Recording not found: ") for line in got["notes"])
    p = projmod.Project(str(tmp_path / "c"))                # opens fine
    miss = share.missing_recordings(p)
    assert {m["id"] for m in miss} == {m["id"] for m in got["missing"]}
    # the person picks one by hand
    solo = [m for m in miss if m["name"] == "solo-run"][0]
    with pytest.raises(share.ShareError):
        share.relink(p, solo["id"], str(tmp_path / "nowhere"))
    share.relink(p, solo["id"], str(raw / "solo-run"))
    assert len(share.missing_recordings(projmod.Project(str(tmp_path / "c")))) == 1


def test_with_video_relinks_inside_project(tmp_path):
    proj, ids, raw = make_project(tmp_path)
    f = share.export_project(proj, str(tmp_path / "v.mxirl"), with_video=True)["path"]
    m = share.read_manifest(f)
    assert m["with_video"] and all(r["video_included"] for r in m["recordings"])
    got = share.import_project(f, str(tmp_path / "vc"))
    assert got["missing"] == []
    for r in projmod.Project(str(tmp_path / "vc")).recordings():
        assert r["path"].startswith(str(tmp_path / "vc" / "recordings"))
        assert os.path.exists(os.path.join(r["path"], "video.avi"))
        assert os.path.exists(os.path.join(r["path"], "frames.csv"))


def test_refuses_to_overwrite(tmp_path):
    proj, ids, raw = make_project(tmp_path)
    f = share.export_project(proj, str(tmp_path / "a.mxirl"))["path"]
    dest = tmp_path / "busy"
    os.makedirs(dest)
    (dest / "mine.txt").write_text("keep")
    with pytest.raises(share.ShareError, match="already has files"):
        share.import_project(f, str(dest))
    assert os.listdir(dest) == ["mine.txt"]
    os.makedirs(tmp_path / "empty")                          # an empty folder is fine
    share.import_project(f, str(tmp_path / "empty"))


def test_escaping_path_is_rejected(tmp_path):
    bad = tmp_path / "bad.mxirl"
    with zipfile.ZipFile(bad, "w") as zf:
        zf.writestr("manifest.json", json.dumps(dict(format="mxirl", format_version=1, recordings=[])))
        zf.writestr("project/project.json", "{}")
        zf.writestr("project/../../evil.txt", "x")
    dest = tmp_path / "d"
    with pytest.raises(share.ShareError, match="unsafe"):
        share.import_project(str(bad), str(dest))
    assert not (tmp_path / "evil.txt").exists() and not dest.exists()
    for name in ("/abs.txt", "C:/x.txt", "other/x.txt"):
        with zipfile.ZipFile(bad, "w") as zf:
            zf.writestr("manifest.json", json.dumps(dict(format="mxirl", format_version=1, recordings=[])))
            zf.writestr(name, "x")
        with pytest.raises(share.ShareError):
            share.import_project(str(bad), str(dest))


def test_not_a_project_file(tmp_path):
    p = tmp_path / "x.mxirl"
    p.write_text("hello")
    with pytest.raises(share.ShareError, match="not a Maxfield IRL Tracking"):
        share.import_project(str(p), str(tmp_path / "d"))


def test_cli(tmp_path, capsys):
    proj, ids, raw = make_project(tmp_path)
    assert share.main(["export", proj.folder, str(tmp_path / "c.mxirl")]) == 0
    import shutil
    shutil.rmtree(raw)                                       # the recordings are "elsewhere" now
    os.makedirs(tmp_path / "empty-search")
    assert share.main(["import", str(tmp_path / "c.mxirl"), str(tmp_path / "n"), "--recordings", str(tmp_path / "empty-search")]) == 0
    out = capsys.readouterr().out
    assert "Recordings found: 0 of 3." in out and "Recording not found" in out
    assert share.main(["import", str(tmp_path / "c.mxirl"), str(tmp_path / "n")]) == 1
    assert "already has files" in capsys.readouterr().out
    assert "in real life" not in out.lower()
