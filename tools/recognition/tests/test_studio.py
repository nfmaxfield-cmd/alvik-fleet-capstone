"""Recognition studio: projects, the honest split, cross-camera guesses, the web routes, training.

The training test takes a few minutes; it runs only with RECOGNITION_TRAIN=1.
"""
import csv
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
for p in (REC, os.path.join(TOOLS, "label"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import project as projmod  # noqa: E402
import studio as studiomod  # noqa: E402
import trainer  # noqa: E402

COLS = ["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg", "heading_known", "source", "status", "score"]


def fake_recording(folder, n=300, fps=10.0, t0=1000.0, size=(640, 480)):
    os.makedirs(folder, exist_ok=True)
    open(os.path.join(folder, "video.avi"), "wb").close()
    with open(os.path.join(folder, "frames.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t_mono", "t_wall"])
        for i in range(n):
            w.writerow([i, "%.3f" % (i / fps), "%.3f" % (t0 + i / fps)])
    with open(os.path.join(folder, "meta.json"), "w") as f:
        json.dump(dict(width=size[0], height=size[1], fps_reported=fps), f)


def write_labels(label_dir, rows):
    os.makedirs(os.path.join(label_dir, "frames"), exist_ok=True)
    with open(os.path.join(label_dir, "labels.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLS)
        for r in rows:
            w.writerow(r)
            open(os.path.join(label_dir, "frames", "%06d.jpg" % r[0]), "wb").close()


def test_add_two_camera_session(tmp_path):
    sess = tmp_path / "20261019-210000_E1"
    fake_recording(str(sess / "cam0"))
    fake_recording(str(sess / "cam1"))
    with open(sess / "session.json", "w") as f:
        json.dump(dict(cameras=[dict(index=0, folder="cam0", device=0, role="overhead"),
                                dict(index=1, folder="cam1", device=2, role=None)]), f)
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    ids = proj.add_recording(str(sess))
    assert len(ids) == 2
    recs = proj.recordings()
    assert {r["camera"] for r in recs} == {"overhead", "cam1"}
    assert all(r["session"] == "20261019-210000_E1" for r in recs)
    assert proj.add_recording(str(sess)) == []            # adding again changes nothing
    assert proj.data["current"]["rec"] == ids[0]
    # reopening reads the same project
    assert projmod.Project(str(tmp_path / "p")).recordings() == recs


def test_object_types_rules(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    fake_recording(str(tmp_path / "r"))
    rid = proj.add_recording(str(tmp_path / "r"))[0]
    write_labels(proj.labels_dir(rid), [[5, "", 1, "alvik", 10, 10, "", 0, "user", "confirmed", ""]])
    with pytest.raises(ValueError):                       # alvik is used: can't be renamed away
        proj.set_classes([dict(name="robot")])
    with pytest.raises(ValueError):
        proj.set_classes([dict(name="alvik"), dict(name="alvik")])
    with pytest.raises(ValueError):
        proj.set_classes([dict(name="bad name!")])
    proj.set_classes([dict(name="alvik", has_front=True, height_cm=4), dict(name="cube", height_cm=2.5)])
    # the catch-all "unknown" is always kept (added at the end)
    assert proj.class_names() == ["alvik", "cube", "unknown"] and proj.heading_classes() == ["alvik"]
    assert proj.class_heights()["cube"] == 2.5
    assert [c["locked"] for c in proj.class_view()] == [True, False, False]


def test_split_is_by_time_block_and_stable(tmp_path):
    """Blocks keep their part when labels are added, and two cameras' views of one moment share a part."""
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    proj.data["settings"]["block_s"] = 5
    sess = tmp_path / "S1"
    for cam in ("cam0", "cam1"):
        fake_recording(str(sess / cam), n=600)
    with open(sess / "session.json", "w") as f:
        json.dump(dict(cameras=[dict(folder="cam0", role="overhead"), dict(folder="cam1", role="side")]), f)
    ids = proj.add_recording(str(sess))

    def label(every):
        for rid in ids:
            rows = [[fr, "", 1, "alvik", 100, 100, "", 0, "user", "confirmed", ""] for fr in range(0, 600, every)]
            write_labels(proj.labels_dir(rid), rows)

    label(20)
    a1, info1 = trainer.build_dataset(proj, str(tmp_path / "d1"), block_s=5)
    label(7)
    a2, info2 = trainer.build_dataset(proj, str(tmp_path / "d2"), block_s=5)
    common = set(a1) & set(a2)
    assert common and all(a1[n] == a2[n] for n in common)
    assert {"train", "val", "test"} <= set(a2.values())
    by_block = {}
    for n, part in a2.items():
        by_block.setdefault(info2[n]["block"], set()).add(part)
    assert all(len(p) == 1 for p in by_block.values())      # cam0 and cam1 agree on every block
    # frames within 1 s of a block edge are left out
    for n in a2:
        with open(tmp_path / "d2" / n / "labels.csv") as f:
            for r in csv.DictReader(f):
                t = int(r["frame"]) / 10.0
                assert min(t % 5, 5 - t % 5) >= 1.0 - 1e-9


def test_cross_camera_guesses_in_studio(tmp_path):
    import synth
    cam_a, cam_b = synth.cam_overhead(), synth.cam_oblique()
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    sess = tmp_path / "S"
    fake_recording(str(sess / "cam0"), n=50, t0=500.0)
    fake_recording(str(sess / "cam1"), n=50, t0=500.02)            # 20 ms later: still the same moment
    with open(sess / "session.json", "w") as f:
        json.dump(dict(cameras=[dict(folder="cam0", role="overhead"), dict(folder="cam1", role="side")]), f)
    ids = proj.add_recording(str(sess))
    cam_a.save(proj.camera_model_path("overhead"))
    cam_b.save(proj.camera_model_path("side"))
    # a robot at floor (60, 40), top 4 cm up, heading +x on the floor, confirmed in the overhead camera
    ua, va = cam_a.project(np.array([[60.0, 40.0, 4.0]]))
    ua, va = float(np.ravel(ua)[0]), float(np.ravel(va)[0])
    ahead_u, ahead_v = cam_a.project(np.array([[63.0, 40.0, 4.0]]))
    h_img = (np.degrees(np.arctan2(-(float(np.ravel(ahead_v)[0]) - va), float(np.ravel(ahead_u)[0]) - ua))) % 360
    write_labels(proj.labels_dir(ids[0]), [[10, "", 1, "alvik", "%.2f" % ua, "%.2f" % va, "%.1f" % h_img, 1,
                                           "user", "confirmed", ""]])
    st = studiomod.Studio(proj, log=lambda *a: None)
    extra = st._cross_camera_guesses(proj.rec(ids[1]))
    assert extra and 10 in extra
    g = extra[10][0]
    ub, vb = cam_b.project(np.array([[60.0, 40.0, 4.0]]))
    assert abs(g["x"] - float(np.ravel(ub)[0])) < 2 and abs(g["y"] - float(np.ravel(vb)[0])) < 2
    assert g["source"] == "camera" and g["cls"] == "alvik" and g["score"] > 0.5
    assert st.cross_note


def _get(url):
    with urllib.request.urlopen(url, timeout=30) as r:
        return r.status, r.read()


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.status, json.loads(r.read())


@pytest.fixture(scope="module")
def synth_rec(tmp_path_factory):
    d = tmp_path_factory.mktemp("synthrec")
    subprocess.run([sys.executable, os.path.join(TOOLS, "label", "tests", "synth.py"), str(d / "rec"), "--seconds", "20"],
                   check=True, capture_output=True)
    return str(d / "rec")


def wait_open(st, timeout=120):
    t0 = time.time()
    while st.loading is not None and time.time() - t0 < timeout:
        time.sleep(0.1)
    assert st.session is not None, st.loading


def test_web_routes(tmp_path, synth_rec):
    proj = projmod.Project(str(tmp_path / "p"), create_name="Web")
    rid = proj.add_recording(synth_rec, "overhead")[0]
    st = studiomod.Studio(proj, log=lambda *a: None)
    st.open(rid, "none", wait=True)
    assert st.session is not None and st.loading is None
    get, post = studiomod.make_routes(st)
    server, url = studiomod.labelapp.serve(st.current, 0, extra_get=get, extra_post=post)
    th = threading.Thread(target=server.serve_forever, daemon=True)
    th.start()
    try:
        code, body = _get(url + "api/studio/state")
        s = json.loads(body)
        assert code == 200 and s["project"]["name"] == "Web" and s["recordings"][0]["id"] == rid
        code, body = _get(url + "api/info")
        assert json.loads(body)["classes"] == ["unknown"]          # a new project: only the catch-all until setup
        code, body = _get(url + "api/queue?mode=next")
        q = json.loads(body)["frames"]
        confs = [x["conf"] for x in q if x["state"] == "pending"]
        # least confident first; among guesses about as uncertain (within app.Session.SPREAD_BAND) the
        # order may take frames from each part of the training split in turn
        band = studiomod.labelapp.Session.SPREAD_BAND
        assert all(c <= min(confs[i:]) + band + 1e-9 for i, c in enumerate(confs))
        code, body = _get(url + "report")
        assert code == 200 and b"Maxfield IRL Tracking" in body
        # a new object type: saved, and the page's classes follow (the recording opens again)
        code, j = _post(url + "api/studio/classes", dict(classes=[dict(name="cube", height_cm=2)] + s["classes"]))
        wait_open(st)
        code, body = _get(url + "api/info")
        assert json.loads(body)["classes"] == ["cube", "unknown"]
        # a label with the new type is accepted
        f = q[0]["f"]
        code, j = _post(url + "api/frame", dict(frame=f, state="reviewed", how="user",
                                                objects=[dict(obj=None, cls="cube", x=100, y=100, source="user",
                                                              status="corrected")]))
        assert j["frame"]["objects"][0]["cls"] == "cube"
        assert os.path.exists(os.path.join(proj.labels_dir(rid), "labels.csv"))
        # labels live in the project, not next to the recording
        assert not os.path.exists(os.path.join(synth_rec, "labels", "labels.csv"))
    finally:
        server.shutdown()
        server.server_close()


@pytest.mark.skipif(os.environ.get("RECOGNITION_TRAIN") != "1", reason="slow: set RECOGNITION_TRAIN=1")
def test_train_and_score(tmp_path):
    synth_rec = str(tmp_path / "rec60")
    subprocess.run([sys.executable, os.path.join(TOOLS, "label", "tests", "synth.py"), synth_rec, "--seconds", "60"],
                   check=True, capture_output=True)
    proj = projmod.Project(str(tmp_path / "p"), create_name="Train")
    proj.finish_setup(classes=projmod.PRESETS[0]["classes"])      # the ready-made robot set
    proj.data["settings"]["block_s"] = 10
    proj.save()
    rid = proj.add_recording(synth_rec, "overhead")[0]
    st = studiomod.Studio(proj, log=lambda *a: None)
    st.open(rid, "none", wait=True)
    s = st.session
    truth = {}
    with open(os.path.join(synth_rec, "truth.csv")) as f:
        for r in csv.DictReader(f):
            truth.setdefault(int(r["frame"]), []).append(r)
    for f in s.selected:
        objs = [dict(obj=None, cls=r["cls"] if r["cls"] in s.classes else "unknown", x=float(r["x"]), y=float(r["y"]),
                     heading_deg=float(r["heading_deg"]) if r["cls"] == "alvik" else None,
                     heading_known=r["cls"] == "alvik", source="user", status="corrected")
                for r in truth.get(f, []) if 0 <= float(r["x"]) <= 640 and 0 <= float(r["y"]) <= 480]
        s.save_frame(f, objs, "reviewed", "user")
    mid = st.trainer.start(minutes=1.0)
    while st.trainer.busy():
        time.sleep(3)
    assert st.trainer.job["status"] == "done", st.trainer.job["message"]
    c = st.curve()
    assert {p["model"] for p in c["points"]} == {"none", mid}
    assert c["test"]["frames"] > 0
    st.open(rid, mid, wait=True)
    assert st.session and "model" in st.session.source_name
