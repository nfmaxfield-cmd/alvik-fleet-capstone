"""Fix round A (data safety and training), studio side: project.json saves from several threads,
models left "training", a crash in the trainer thread, the split's block length and its plain
message, the "No model" baseline in the project's own types, plain training status, a model that
finds nothing, a missing second-camera recording, empty paths, and the share snapshot."""
import csv
import json
import os
import shutil
import sys
import threading
import time
import zipfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
for p in (REC, os.path.join(TOOLS, "label"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import project as projmod  # noqa: E402
import share  # noqa: E402
import studio as studiomod  # noqa: E402
import trainer  # noqa: E402
from test_studio import fake_recording, write_labels  # noqa: E402


# ---------------------------------------------------------------- project.json saves
def test_project_saves_from_several_threads_never_fail_or_corrupt(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    errs, bad = [], [0]
    stop = threading.Event()

    def writer(k):
        for i in range(150):
            try:
                p.data.setdefault("settings", {})["x%d" % k] = i
                if k == 0:
                    p.data["pad"] = "y" * (i % 50)
                p.save()
            except Exception as e:      # noqa: BLE001
                errs.append(repr(e))

    def reader():
        while not stop.is_set():
            try:
                with open(p.path, encoding="utf-8") as f:
                    json.load(f)
            except ValueError:
                bad[0] += 1
            except FileNotFoundError:
                bad[0] += 1          # the file must always be there
    ts = [threading.Thread(target=writer, args=(k,)) for k in range(3)]
    r = threading.Thread(target=reader)
    r.start()
    [t.start() for t in ts]
    [t.join() for t in ts]
    stop.set()
    r.join()
    assert errs == [] and bad[0] == 0
    assert json.load(open(p.path))["settings"]["x2"] == 149
    assert [f for f in os.listdir(p.folder) if f.endswith(".tmp")] == []
    # two Project objects for one folder share the lock
    assert projmod.Project(p.folder).lock is p.lock


def test_save_retries_while_another_program_holds_the_file(tmp_path, monkeypatch):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    real = os.replace
    calls = [0]

    def flaky(a, b):
        calls[0] += 1
        if calls[0] <= 2:
            raise PermissionError("in use")
        return real(a, b)
    monkeypatch.setattr(projmod.os, "replace", flaky)
    monkeypatch.setattr(projmod.time, "sleep", lambda s: None)
    p.data["name"] = "Saved anyway"
    p.save()
    assert calls[0] == 3 and json.load(open(p.path))["name"] == "Saved anyway"


# ---------------------------------------------------------------- the trainer thread
def test_models_left_training_are_marked_interrupted_at_start(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    p.data["models"].append(dict(id="v1", status="training", classes=p.class_names()))
    p.data["models"].append(dict(id="v2", status="ready", classes=p.class_names()))
    p.save()
    logs = []
    studiomod.Studio(projmod.Project(p.folder), log=logs.append, background=False)
    ms = {m["id"]: m for m in json.load(open(p.path))["models"]}
    assert ms["v1"]["status"] == "failed" and ms["v1"]["interrupted"] and "interrupted" in ms["v1"]["error"]
    assert ms["v2"]["status"] == "ready"
    assert any("interrupted" in m for m in logs)


def _wait_job(tr, timeout=60):
    t0 = time.time()
    while tr.busy() and time.time() - t0 < timeout:
        time.sleep(0.05)
    assert not tr.busy(), tr.job


def test_a_crash_in_the_trainer_thread_marks_the_job_failed(tmp_path, monkeypatch):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    tr = trainer.Trainer(p, log=lambda *a: None)

    def boom(*a, **k):
        raise TypeError("something unexpected")
    monkeypatch.setattr(trainer, "build_dataset", boom)
    mid = tr.start(minutes=1)
    _wait_job(tr)
    assert tr.job["status"] == "failed" and "unexpected problem" in tr.job["message"]
    assert "Your labels are safe" in tr.job["message"] and "TypeError" in tr.job["message"]
    assert p.model(mid)["status"] == "failed"
    # even when saving the project fails at the very start, the job never stays "running"
    real_save = p.save
    monkeypatch.setattr(p, "save", lambda: (_ for _ in ()).throw(FileNotFoundError("project.json.tmp")))
    tr.start(minutes=1)
    _wait_job(tr)
    assert tr.job["status"] == "failed" and "FileNotFoundError" in tr.job["message"]
    monkeypatch.setattr(p, "save", real_save)


def test_train_route_and_trainer_save_at_the_same_time(tmp_path):
    """The route saves the minutes while the trainer thread adds its model: both survive."""
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    st = studiomod.Studio(p, log=lambda *a: None, background=False)
    get, post = studiomod.make_routes(st)

    class H:
        def _send(self, code, body, ctype=None):
            self.code, self.body = code, body
    for _ in range(5):
        h = H()
        post(h, "/api/studio/train", dict(minutes=3))
        _wait_job(st.trainer)
        assert st.trainer.job["status"] == "failed"                 # no labels: a plain message
        assert "Not enough confirmed frames" in st.trainer.job["message"]
    saved = json.load(open(p.path))
    assert saved["settings"]["train_minutes"] == 3.0 and [m["status"] for m in saved["models"]] == ["failed"] * 5


# ---------------------------------------------------------------- the split
def _minutes_recording(tmp_path, name, seconds, fps=10.0):
    d = str(tmp_path / name)
    fake_recording(d, n=int(seconds * fps), fps=fps)
    return d


def test_block_length_from_the_recordings_length(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    p.add_recording(_minutes_recording(tmp_path, "long", 16 * 60), "top")
    assert trainer.choose_block_s(p) == 30.0                 # 16 minutes: 30 s blocks, as before
    q = projmod.Project(str(tmp_path / "q"), create_name="Q")
    q.add_recording(_minutes_recording(tmp_path, "five", 5 * 60), "top")
    assert trainer.choose_block_s(q) == 20.0                 # 5 minutes: 15 blocks of 20 s
    r = projmod.Project(str(tmp_path / "r"), create_name="R")
    r.add_recording(_minutes_recording(tmp_path, "one", 60), "top")
    assert trainer.choose_block_s(r) == 4.0                  # 1 minute: still at least 12 blocks
    assert trainer.guard_for(4.0) < 1.0
    # a project that trained with 20 s blocks keeps them when more footage arrives (nothing moves)
    q.data["models"].append(dict(id="v1", status="ready", block_s=20.0, classes=q.class_names()))
    q.add_recording(_minutes_recording(tmp_path, "more", 20 * 60), "top")
    assert trainer.choose_block_s(q) == 20.0


def test_every_part_gets_a_block_even_when_labels_sit_in_few_blocks(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    d = _minutes_recording(tmp_path, "rec", 5 * 60)
    rid = p.add_recording(d, "top")[0]
    # labels only in three 20 s blocks that the hash puts in scoring or picking: none in training
    blocks = [20 * k for k in range(15) if trainer._part("%s|%d" % (rid, k))[0] != "train"][:3]
    assert len(blocks) == 3
    rows = [[int((b + 10 + 0.2 * i) * 10), "", 1, "unknown", 100, 100, "", 0, "user", "confirmed", ""]
            for b in blocks for i in range(10)]
    write_labels(p.labels_dir(rid), rows)
    assign, info = trainer.build_dataset(p, str(tmp_path / "d"), 20.0, guard_s=1.0)
    assert len(assign) == 3 and set(assign.values()) == {"train", "val", "test"}


def test_split_message_gives_counts_per_part_and_what_to_do():
    msg = trainer.split_message(dict(train=0, val=12, test=17), dict(val=1, test=2), 30)
    assert "0 frames for learning (in 0 pieces)" in msg and "12 for picking" in msg and "17 held back" in msg
    assert "30-second pieces" in msg and "spread over more of the recording" in msg
    assert "only 0 confirmed frames" not in msg


# ---------------------------------------------------------------- the "No model" baseline
def test_no_model_baseline_uses_the_projects_types(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="Cubes")
    p.set_classes([dict(name="cube"), dict(name="unknown")])
    d = _minutes_recording(tmp_path, "rec", 30)
    rid = p.add_recording(d, "top")[0]
    cache = os.path.join(p.labels_dir(rid), "cache")
    os.makedirs(cache, exist_ok=True)
    # the motion finder names its object-sized blobs "alvik" whatever the project is about
    with open(os.path.join(cache, "motion_detections.csv"), "w") as f:
        f.write("frame,cls,x,y,heading_deg,score,angle90,source\n")
        for fr in range(300):
            f.write("%d,alvik,%.2f,100.00,,0.9000,,motion\n" % (fr, 100 + fr * 0.2))
    json.dump(dict(robot_px=30.0), open(os.path.join(cache, "motion_info.json"), "w"))
    mdir = tmp_path / "m"
    blk = mdir / "data" / ("%s__b00001" % rid)
    os.makedirs(blk)
    with open(blk / "labels.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg", "heading_known", "source",
                    "status", "score"])
        for fr in (120, 130, 140):
            w.writerow([fr, "", 1, "cube", 120, 100, "", 0, "user", "confirmed", ""])
    out = str(mdir / "motion_test_preds.csv")
    n = trainer.motion_predictions(p, {blk.name: dict(recording=rid)}, {blk.name: "test"}, out)
    got = list(csv.DictReader(open(out)))
    assert n == 3 and {r["cls"] for r in got} == {"cube"}            # mapped, so evaluate can match them


# ---------------------------------------------------------------- status and empty models
def test_training_status_in_plain_words():
    tp = trainer.TrainProgress(60, 2.0, t0=0.0)
    assert not tp.feed("Loaded and shrank 40 frames in 1 s")
    assert tp.feed("epoch   1    2.0s  loss 3.100  val 2.900  P nan R 0.000 F1 0.000  err nanpx  hd nandeg  []  *")
    t = tp.text(2.0)
    assert t.startswith("round 1 of about 60") and "finds nothing yet" in t and "nan" not in t
    for e in range(2, 5):
        tp.feed("epoch %3d    2.0s  loss 1.0  val 1.0  P 0.800 R 0.%d00 F1 0.700  err 2.00px  hd 9.0deg  []%s"
                % (e, e + 4, "  *" if e == 4 else ""))
    t = tp.text(8.0)
    assert "round 4 of about 60" in t and "min left" in t and "finds 80% of the objects" in t
    assert "P " not in t and "F1" not in t


def test_a_model_that_finds_nothing_is_never_the_starting_point(tmp_path):
    assert not trainer.finds_objects(dict(warning=trainer.NOTHING_FOUND))
    assert not trainer.finds_objects(dict(metrics=dict(recall=0.0)))
    assert not trainer.finds_objects(dict(val_f1=0.0))
    assert trainer.finds_objects(dict(metrics=dict(recall=0.4), val_f1=0.5))
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    for mid, extra in (("v1", dict()), ("v2", dict(warning=trainer.NOTHING_FOUND))):
        os.makedirs(os.path.join(p.model_dir(mid), "run"))
        open(os.path.join(p.model_dir(mid), "run", "best.pt"), "w").close()
        p.data["models"].append(dict(id=mid, status="ready", classes=p.class_names(), **extra))
    assert trainer.Trainer(p)._previous_same_classes().endswith(os.path.join("v1", "run", "best.pt"))
    st = studiomod.Studio(p, log=lambda *a: None, background=False).state()
    assert [m["warning"] for m in st["models"]] == [None, trainer.NOTHING_FOUND]


def test_best_val_f1_from_the_training_log(tmp_path):
    p = tmp_path / "train_log.csv"
    p.write_text("epoch,val_f1\n1,0.0\n2,0.0\n")
    assert trainer.best_val_f1(str(p)) == 0.0
    p.write_text("epoch,val_f1\n1,0.0\n2,0.41\n")
    assert trainer.best_val_f1(str(p)) == 0.41
    assert trainer.best_val_f1(str(tmp_path / "none.csv")) is None


# ---------------------------------------------------------------- second camera, empty paths
def test_opening_a_camera_when_the_other_cameras_recording_is_missing(tmp_path):
    import synth
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    sess = tmp_path / "S"
    fake_recording(str(sess / "cam0"), n=50, t0=500.0)
    fake_recording(str(sess / "cam1"), n=50, t0=500.0)
    with open(sess / "session.json", "w") as f:
        json.dump(dict(cameras=[dict(folder="cam0", role="overhead"), dict(folder="cam1", role="side")]), f)
    ids = proj.add_recording(str(sess))
    synth.cam_overhead().save(proj.camera_model_path("overhead"))
    synth.cam_oblique().save(proj.camera_model_path("side"))
    write_labels(proj.labels_dir(ids[1]), [[5, "", 1, "unknown", 10, 10, "", 0, "user", "confirmed", ""]])
    shutil.rmtree(sess / "cam1")              # the side camera's recording was not copied
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    assert st._cross_camera_guesses(proj.rec(ids[0])) is None
    assert "not found on this computer" in st.cross_note and "side" in st.cross_note


def test_empty_paths_are_refused_in_plain_words(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    for bad in ("", "   ", '""'):
        with pytest.raises(ValueError, match="the box is empty"):
            p.add_recording(bad)
    st = studiomod.Studio(p, log=lambda *a: None, background=False)
    get, post = studiomod.make_routes(st)

    class H:
        def _send(self, code, body, ctype=None):
            self.code, self.body = code, body
    with pytest.raises(ValueError, match="the box is empty"):
        post(H(), "/api/studio/add", dict(path=""))
    p.add_recording(_minutes_recording(tmp_path, "rec", 5), "top")
    rid = p.recordings()[0]["id"]
    with pytest.raises(ValueError, match="the box is empty"):
        st.relink(rid, "  ")
    with pytest.raises(ValueError, match="the box is empty"):
        st.relink(folder="")
    with pytest.raises(ValueError, match="the box is empty"):
        share.relink(p, rid, "")
    assert p.rec(rid)["path"].endswith("rec")                      # unchanged


# ---------------------------------------------------------------- share: one snapshot of the pair
def test_share_export_copies_labels_and_review_as_one_snapshot(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    rid = p.add_recording(_minutes_recording(tmp_path, "rec", 5), "top")[0]
    ld = p.labels_dir(rid)
    os.makedirs(ld, exist_ok=True)

    def write(k):
        open(os.path.join(ld, "labels.csv"), "w").write("frame\n%d\n" % k)
        time.sleep(0.01)
        open(os.path.join(ld, "review.json"), "w").write(json.dumps(dict(k=k)))
    write(0)
    lock = threading.RLock()
    stop = threading.Event()

    def saver():                       # a labelling session saving labels.csv, then review.json, under its lock
        k = 1
        while not stop.is_set():
            with lock:
                write(k)
            k += 1
            time.sleep(0.001)
    th = threading.Thread(target=saver)
    th.start()
    try:
        for i in range(5):
            out = share.export_project(p, str(tmp_path / ("s%d.mxirl" % i)), lock_for=lambda r: lock if r == rid else None)
            with zipfile.ZipFile(out["path"]) as zf:
                lab = zf.read("project/labels/%s/labels.csv" % rid).decode().split()[1]
                rev = json.loads(zf.read("project/labels/%s/review.json" % rid))["k"]
            assert int(lab) == rev
    finally:
        stop.set()
        th.join()
    # without a lock (the command line), reading waits until neither file changes
    got = share.read_pair(ld)
    assert int(got["labels.csv"].decode().split()[1]) == json.loads(got["review.json"])["k"]
