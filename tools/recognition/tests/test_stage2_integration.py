"""Stage 2 integration (lane I): the new parts wired into the studio.

- the tracker follows objects by their look in the studio, with per-type motion settings from Object
  types; "How often should I check?" measured on the person's labels
- outside data (robot logs, a positions file) for a recording: guesses with names, branch votes, and a
  plain message when the floor calibration is missing
- home positions: stored per camera, visits become free confirmations, the Alvik dock preset
- the AI helper (stand-in): settings, indicator, every task, decisions counted, never in a shared file
- training: the copy-paste choice, what training used, by situation in the learning-curve view
- use_model.py: --outside through outside.py, and Tracker.confirm()

    python -m pytest tools/recognition/tests/test_stage2_integration.py -q
"""
import csv
import json
import math
import os
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import zipfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
LABEL = os.path.join(TOOLS, "label")
for p in (TOOLS, os.path.join(LABEL, "tests"), LABEL, REC, HERE):     # HERE first: its synth.py, not the label tests' one
    if p in sys.path:
        sys.path.remove(p)
    sys.path.insert(0, p)

import project as projmod  # noqa: E402
import studio as studiomod  # noqa: E402
import outsidelink  # noqa: E402
import tracker  # noqa: E402

SHOTS = os.environ.get("STAGE2_SHOTS")


def _get(url):
    with urllib.request.urlopen(url, timeout=60) as r:
        return json.loads(r.read())


def _get_text(url):
    with urllib.request.urlopen(url, timeout=60) as r:
        return r.read().decode()


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=180) as r:
        return json.loads(r.read())


def _post_err(url, body):
    with pytest.raises(urllib.error.HTTPError) as e:
        _post(url, body)
    return e.value.code, json.loads(e.value.read())


def wait_open(st, timeout=300):
    t0 = time.time()
    while (st.loading is not None or st.pending_reopen or st.session is None) and time.time() - t0 < timeout:
        time.sleep(0.1)
    assert st.session is not None, st.loading


def serve(st):
    get, post = studiomod.make_routes(st)
    server, url = studiomod.labelapp.serve(st.current, 0, extra_get=get, extra_post=post)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, url


@pytest.fixture(scope="module")
def synth10(tmp_path_factory):
    d = tmp_path_factory.mktemp("synth10i")
    subprocess.run([sys.executable, os.path.join(LABEL, "tests", "synth.py"), str(d / "rec"), "--seconds", "10"],
                   check=True, capture_output=True)
    return str(d / "rec")


@pytest.fixture(scope="module")
def roam(tmp_path_factory):
    """A synthetic roam session: three robots start docked, roam, visit their docks, park; their logs
    (with gyro scale error, drift, slip, clock offsets) and a rendered recording with its camera model."""
    import synth_logs as SL
    out = str(tmp_path_factory.mktemp("roam"))
    return SL.make_session(out, seconds=60.0, seed=4, visit_at=30.0, lift=False, render=True)


def studio_on(tmp_path, rec, classes=None, camera="overhead"):
    proj = projmod.Project(str(tmp_path / "p"), create_name="I")
    rid = proj.add_recording(rec, camera)[0]
    proj.finish_setup("I", classes or [dict(name="bot", has_front=True)], 5)
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    st.open(rid, "none", wait=True)
    wait_open(st)
    server, url = serve(st)
    return proj, st, rid, server, url


# =============================================================================== 1. tracking
def test_object_types_keep_motion_settings_and_the_tracker_reads_them(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="T")
    proj.set_classes([dict(name="cart", has_front=True, max_speed=6, can_be_carried="no", can_sit_still=True),
                      dict(name="hand", height_varies=True)])
    c = {x["name"]: x for x in proj.classes}
    assert c["cart"]["max_speed"] == 6.0 and c["cart"]["can_be_carried"] is False and c["cart"]["can_sit_still"] is True
    assert "max_speed" not in c["hand"]                     # blank: the usual, nothing stored
    p = tracker.TrackerParams.from_types(proj.type_settings())
    assert p.motion_of("cart") == dict(max_speed=6.0, can_be_carried=False, can_sit_still=True)
    assert p.motion_of("hand") == p.type_defaults("hand")
    with pytest.raises(ValueError):
        proj.set_classes([dict(name="cart", max_speed=0)])
    # an older project.json without the settings reads as the usual ones
    old = projmod.Project(str(tmp_path / "p"))
    assert tracker.TrackerParams.from_types(old.type_settings()).motion_of("unknown") == p.type_defaults("unknown")


def test_studio_follows_by_look_with_the_types_settings_and_measures_checkins(tmp_path, synth10):
    proj, st, rid, server, url = studio_on(tmp_path, synth10, [dict(name="bot", has_front=True, max_speed=4.0)])
    try:
        s = st.session
        assert s.frames_src is not None and s.run.looker is not None          # the appearance chain is on
        assert s.params.types == {"bot": dict(max_speed=4.0)}
        state = _get(url + "api/studio/state")
        assert state["motion_usual"]["bot"]["max_speed"] == 3.0                 # what "usual" means, for the page
        assert _get(url + "api/checkins")["status"] == "none"
        code, j = _post_err(url + "api/checkins", {})
        assert code == 400 and "at least 2 labelled frames" in j["error"]
        for f in s.selected[1:8:2]:
            objs = [dict(o, status="confirmed") for o in s.guess_objects(f)]
            if objs:
                s.save_frame(f, objs, "reviewed")
        v = _post(url + "api/checkins", {})
        t0 = time.time()
        while v["status"] == "running" and time.time() - t0 < 120:
            time.sleep(0.2)
            v = _get(url + "api/checkins")
        assert v["status"] == "done", v
        raw = _get_text(url + "api/checkins")
        assert "Infinity" not in raw                                          # the page's JSON reader refuses it
        r = v["result"]
        assert r["by_length"] and r["labelled_frames"] >= 2
        assert os.path.exists(os.path.join(proj.labels_dir(rid), "checkins.json"))
    finally:
        server.shutdown()


def test_split_vote_hook_settles_lookalikes_from_outside_data():
    import stage2_scenes as S2
    sc = S2.Scene(seconds=14)
    sc.robot("A", [(0, 150, 240), (2, 150, 240), (5, 304, 240), (8, 304, 240), (10.5, 304, 100), (14, 304, 100)])
    sc.robot("B", [(0, 490, 240), (2, 490, 240), (5, 336, 240), (8, 336, 240), (10.5, 336, 380), (14, 336, 380)])
    anchors = {0: [S2.anchor_obj(1, *sc.truth("A", 0)), S2.anchor_obj(2, *sc.truth("B", 0))]}
    run = tracker.TrackRun(sc.all_dets(), sc.times, S2.R)
    run.run(anchors, {1, 2})
    assert run.splits[0]["resolved"] is False                                 # without outside data: ask a person
    drafts = {f: [dict(name=n, x=sc.truth(n, f)[0], y=sc.truth(n, f)[1], sd_px=2.0) for n in ("A", "B")]
              for f in range(sc.n)}
    run2 = tracker.TrackRun(sc.all_dets(), sc.times, S2.R)
    run2.split_vote = outsidelink.split_voter(drafts, S2.R)
    rows, _ = run2.run(anchors, {1, 2})
    assert run2.splits[0]["resolved"] is True and run2.splits[0].get("outside")
    assert not any(r["split_unresolved"] for rs in rows.values() for r in rs)
    end = {r["obj"]: r for r in rows[sc.n - 1]}
    assert math.hypot(end[1]["x"] - sc.truth("A", sc.n - 1)[0], end[1]["y"] - sc.truth("A", sc.n - 1)[1]) < 5
    # the log decides, either way: when it says they swapped while together, the paths swap
    swapped = {f: [dict(x, name={"A": "B", "B": "A"}[x["name"]]) if f > 140 else x for x in v] for f, v in drafts.items()}
    run4 = tracker.TrackRun(sc.all_dets(), sc.times, S2.R)
    run4.split_vote = outsidelink.split_voter(swapped, S2.R)
    rows4, _ = run4.run(anchors, {1, 2})
    assert run4.splits[0]["resolved"] is True and run4.splits[0]["swapped"] is True
    end4 = {r["obj"]: r for r in rows4[sc.n - 1]}
    assert math.hypot(end4[1]["x"] - sc.truth("B", sc.n - 1)[0], end4[1]["y"] - sc.truth("B", sc.n - 1)[1]) < 5
    # a hook that fails never stops the tracking
    run3 = tracker.TrackRun(sc.all_dets(), sc.times, S2.R)
    run3.split_vote = lambda *a: 1 / 0
    run3.run(anchors, {1, 2})
    assert run3.splits[0]["resolved"] is False


def test_tracker_confirm_is_a_label_for_one_object():
    tk = tracker.Tracker(30.0)
    for f in range(8):
        tk.step(f, f / 25.0, [dict(x=100.0 + f, y=100.0, cls="alvik", score=0.6), dict(x=300.0, y=300.0, cls="alvik", score=0.6)])
    a, b = sorted(tk.tracks, key=lambda t: t.x)
    b_before = (b.x, b.y, b.last_score)
    assert tk.confirm(a.id, 8, 8 / 25.0, 120.0, 101.0) is a
    assert (a.x, a.y, a.last_score, a.confirmed, a.user_seen, a.conf_f) == (120.0, 101.0, 1.0, True, True, 8)
    assert (a.vx, a.vy) == (0.0, 0.0)
    assert (b.x, b.y, b.last_score) == b_before                             # the others are untouched
    assert tk.confirm(999, 8, 0.3) is None


# =============================================================================== 2/3. outside data and home positions
def _roam_project(tmp_path, roam):
    import homes as HM
    proj, st, rid, server, url = studio_on(tmp_path, roam["recording"], [
        dict(name="alvik", has_front=True, height_cm=4.0), dict(name="person", height_varies=True)], camera="")
    return proj, st, rid, server, url


def test_outside_data_without_floor_calibration_says_what_is_needed(tmp_path, synth10):
    proj, st, rid, server, url = studio_on(tmp_path, synth10)
    try:
        d = _get(url + "api/studio/outside")
        assert d["floor"]["ok"] is False and "CROSSCAM.md" in d["floor"]["text"] and "cameras/overhead.json" in d["floor"]["text"]
        assert {p["id"] for p in d["plugins"]} >= {"alvik", "csv"}
        code, j = _post_err(url + "api/studio/outside/set", dict(path=str(tmp_path)))
        assert code == 400 and "floor calibration" in j["error"] and "Nothing was changed" in j["error"]
        assert not os.path.exists(outsidelink.settings_path(proj, rid))
        code, j = _post_err(url + "api/studio/homes/dock", dict(camera="overhead", wall=[1, 1], mouth=[5, 5]))
        assert code == 400 and "floor calibration" in j["error"]
    finally:
        server.shutdown()


def test_robot_logs_become_named_guesses_and_home_visits_confirm(tmp_path, roam):
    import homes as HM
    proj, st, rid, server, url = _roam_project(tmp_path, roam)
    try:
        assert st.session.n_outside == 0
        # home positions from the dock preset (two clicks each), saved for this camera
        mp = json.load(open(os.path.join(roam["out"], "map.json")))
        cam = roam["camera"]
        homes = []
        for d in mp["home_docks"]:
            h = math.radians(d["heading"])
            wall = cam.project((d["x"], d["y"], HM.alvik_dock_geometry(2)["wall_h_cm"]))
            # the mouth: mouth_x_cm along the dock's own x (towards the back wall; the mouth's value is negative)
            mx, my = d["x"] + HM.alvik_dock_geometry(2)["mouth_x_cm"] * math.cos(h), d["y"] + HM.alvik_dock_geometry(2)["mouth_x_cm"] * math.sin(h)
            mouth = cam.project((mx, my, HM.alvik_dock_geometry(2)["wall_h_cm"]))
            got = _post(url + "api/studio/homes/dock", dict(camera="default", name=d["name"], owner=d["name"].split()[-1],
                                                           wall=list(wall), mouth=list(mouth), stations=False))["homes"]
            assert len(got) == 1 and got[0]["owner"] == d["name"].split()[-1] and got[0]["facing_deg"] is not None
            ref = HM.alvik_docks_from_map(dict(home_docks=[d]), "default", cam, stations=False)[0]
            assert math.hypot(got[0]["x"] - ref["x"], got[0]["y"] - ref["y"]) < 0.5
            homes += got
        r = _post(url + "api/studio/homes", dict(camera="default", homes=homes))
        assert r["before"] == [] and len(r["homes"]) == 3 and r["reopened"]
        wait_open(st)
        stored = projmod.Project(proj.folder).data["cameras"]["default"]["homes"]
        assert [h["name"] for h in stored] == [d["name"] for d in mp["home_docks"]]
        vis = st.session.refined["visits"]
        assert len(vis) >= 3                                   # each robot starts docked
        f = vis[0]["frame_start"]
        assert any(r["source"] == "home" for r in st.session.guesses.get(f + 1, []))
        assert _get(url + "api/studio/state")["camera"]["visits"]
        # the robot logs
        r = _post(url + "api/studio/outside/set", dict(path=roam["logs"], plugin=""))
        assert r["plugin"] == "alvik" and set(r["names"]) == {"Left", "Center", "Right"}
        t0 = time.time()
        while (st.outside_job or {}).get("status") == "running" and time.time() - t0 < 300:
            time.sleep(0.2)
        assert st.outside_job["status"] == "done", st.outside_job
        assert st.outside_job["applied"] is False and st.session.n_outside == 0     # never reopens by itself
        assert _post(url + "api/studio/outside/apply", {})["reopened"]
        time.sleep(0.5)
        wait_open(st)
        s = st.session
        assert s.n_outside > 0 and "robot logs" in s.guess_label()
        info = st.outside_job["result"]
        placed = {x["name"]: x for x in info["summary"]}
        assert all(placed[n]["frames"] > 0 for n in ("Left", "Center", "Right")), placed
        assert all(placed[n]["anchors"].get("home") for n in placed)
        # the roll call: names on the objects, close to the robot of that name
        f = s.selected[len(s.selected) // 2]
        view = _get(url + "api/frame?f=%d" % f)
        names = view["names"]
        assert names, view
        truth = roam["truth"]
        for o in view["objects"]:
            n = names.get(str(o["obj"]))
            if n:
                assert math.hypot(o["x"] - truth[n]["frame_u"][f], o["y"] - truth[n]["frame_v"][f]) < 15
        # Stop using it, and Z puts it back (the files are kept)
        assert _post(url + "api/studio/outside/clear", {})["moved"] == ["outside.json", "outside_drafts.json"]
        wait_open(st)
        assert st.session.n_outside == 0
        assert _post(url + "api/studio/outside/clear", dict(restore=True))["moved"]
        wait_open(st)
        assert st.session.n_outside > 0
    finally:
        server.shutdown()


def test_positions_file_plugin_and_use_model_outside(tmp_path, roam):
    """A positions file (floor cm on the shared clock) names objects at the roll call in use_model.py."""
    import use_model as UM
    truth = roam["truth"]
    p = tmp_path / "pos"
    p.mkdir()
    with open(p / "positions.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "name", "x", "y", "heading"])
        for n, tr in truth.items():
            for k in range(0, len(tr["t"]), 10):
                w.writerow(["%.3f" % tr["t"][k], n, "%.3f" % tr["x"][k], "%.3f" % tr["y"][k], "%.2f" % tr["heading"][k]])

    class Pipe:
        cam = roam["camera"]
        heights = {"alvik": 4.0}
        classes = ["alvik", "unknown"]
        camera_key = "default"
        robot_px = 30.0
    said = []
    out = UM.load_outside("csv:" + str(p), said.append, proj=None, pipe=Pipe())
    assert out is not None and "Lined up on the floor" in said[-1]
    f = 400
    t = roam["frame_times"][f]
    ds = out.drafts("%.4f" % t)
    assert {d["name"] for d in ds} == set(truth)
    for d in ds:
        assert math.hypot(d["x"] - truth[d["name"]]["frame_u"][f], d["y"] - truth[d["name"]]["frame_v"][f]) < 3
    objs = [dict(obj=i, x=truth[n]["frame_u"][f] + 2, y=truth[n]["frame_v"][f], name="", heading_deg=None)
            for i, n in enumerate(sorted(truth), 1)]
    UM.apply_outside(objs, ds, 30.0)
    assert [o["name"] for o in objs] == sorted(truth)
    # a plain path works too (the plug-in that recognises it); no floor calibration: said, and None
    assert UM.load_outside(str(p / "positions.csv"), said.append, pipe=Pipe()) is not None

    class NoCam(Pipe):
        cam = None
    assert UM.load_outside("csv:" + str(p), said.append, pipe=NoCam()) is None and "floor calibration" in said[-1]
    with pytest.raises(UM.UserError):
        UM.load_outside("csv:" + str(tmp_path / "nothing"), said.append, pipe=Pipe())


# =============================================================================== 4. AI helper
def test_ai_helper_in_the_studio_with_the_stand_in(tmp_path, synth10, monkeypatch):
    monkeypatch.setenv("MAXFIELD_IRL_HOME", str(tmp_path / "home"))
    proj, st, rid, server, url = studio_on(tmp_path, synth10)
    try:
        s0 = _get(url + "api/studio/state")
        assert s0["ai"]["on"] is False and s0["ai"]["indicator"] == "AI helper off"
        code, j = _post_err(url + "api/studio/ai/ask", dict(task="name_object", frame=st.session.selected[0], x=100, y=100))
        assert code == 400 and j["kind"] == "off" and "Nothing was sent" in j["error"]
        # settings: the key is kept outside the project; the indicator says where pictures can go
        r = _post(url + "api/studio/ai/settings", dict(enabled=True, provider="anthropic", key="sk-test-123"))
        assert r["status"]["on"] and "Anthropic" in r["status"]["indicator"] and r["key_changed"]
        assert "sk-test-123" not in open(os.path.join(proj.folder, "ai_settings.json")).read()
        assert os.path.exists(tmp_path / "home" / "ai.json")
        v = _get(url + "api/studio/ai")
        assert v["keys"]["anthropic"] is True and "sk-test" not in json.dumps(v)
        e = _post(url + "api/studio/ai/estimate", dict(task="name_object", frame=st.session.selected[0], x=320, y=240))
        assert e["n_images"] == 1 and e["cost_usd_max"] is not None
        _post(url + "api/studio/ai/settings", dict(enabled=True, provider="stand_in"))
        assert "stand-in" in _get(url + "api/studio/state")["ai"]["indicator"]
        # every task
        s = st.session
        f = s.selected[2]
        a = _post(url + "api/studio/ai/ask", dict(task="name_object", frame=f, x=320, y=240))
        assert a["source"] == "ai" and a["name"] and a["call_id"]
        b = _post(url + "api/studio/ai/ask", dict(task="setup_from_sentence", sentence="We track cubes and cylinders"))
        assert [c["name"] for c in b["classes"]] == ["cube", "cylinder"]
        d = _post(url + "api/studio/ai/ask", dict(task="draft_labels", frame=f, type="bot"))
        assert d["labels"] and all(l["trust"] == "low" for l in d["labels"])
        code, j = _post_err(url + "api/studio/ai/ask", dict(task="second_check", n=8))
        assert code == 400 and "confirm some frames" in j["error"]
        for g in s.selected[:6]:
            objs = [dict(o, status="confirmed") for o in s.guess_objects(g)]
            if objs:
                s.save_frame(g, objs, "reviewed")
        c = _post(url + "api/studio/ai/ask", dict(task="second_check", n=8))
        assert c["checked"] >= 1 and c["calls"] and all("frame" in x for x in c["sample"])
        code, j = _post_err(url + "api/studio/ai/ask", dict(task="object_or_background"))
        assert code == 400 and "Investigate a spot first" in j["error"]
        s.start_investigate(f, 320, 240)
        t0 = time.time()
        while s.inv["status"] == "running" and time.time() - t0 < 120:
            time.sleep(0.2)
        o = _post(url + "api/studio/ai/ask", dict(task="object_or_background"))
        assert o["verdict"] in ("object", "background", "unsure")
        # the person's answers are recorded and counted in Time saved
        _post(url + "api/studio/ai/decision", dict(call_id=a["call_id"], accepted=True))
        _post(url + "api/studio/ai/decision", dict(call_ids=c["calls"], accepted=False))
        ts = _get(url + "api/studio/timesaved_html")
        tasks = {t["task"]: t for t in ts["ai"]["tasks"]}
        assert tasks["name_object"]["taken"] == 1 and tasks["second_check"]["turned_down"] == len(c["calls"])
        assert "AI helper" in ts["html"] and "Turned down" in ts["html"]
    finally:
        server.shutdown()


def test_shared_project_file_leaves_out_the_ai_files(tmp_path, monkeypatch):
    import share
    import ai_helper
    monkeypatch.setenv("MAXFIELD_IRL_HOME", str(tmp_path / "home"))
    proj = projmod.Project(str(tmp_path / "p"), create_name="S")
    h = ai_helper.AIHelper(proj.folder)
    h.configure(enabled=True, provider="stand_in")
    h.set_key("sk-never-shared", provider="anthropic")
    h.run("setup_from_sentence", sentence="We track boxes")
    assert os.path.exists(os.path.join(proj.folder, "ai_settings.json"))
    assert os.path.exists(os.path.join(proj.folder, "ai_log.jsonl"))
    res = share.export_project(proj, str(tmp_path / "out.mxirl"))
    with zipfile.ZipFile(res["path"]) as z:
        names = z.namelist()
        blob = b"".join(z.read(n) for n in names)
    assert not any(n.endswith(("ai_settings.json", "ai_log.jsonl")) for n in names), names
    assert b"sk-never-shared" not in blob and b"ai_log" not in blob
    assert "project/project.json" in names


# =============================================================================== 5. training
def test_train_copy_paste_choice_and_learning_curve_extras(tmp_path, synth10, monkeypatch):
    proj, st, rid, server, url = studio_on(tmp_path, synth10)
    try:
        import trainer as trainmod
        # not enough checked frames yet: refused with the counts, the copy-paste choice kept as it was
        code, j = _post_err(url + "api/studio/train", dict(minutes=5, copy_paste="compare"))
        assert code == 400 and "training did not start" in j["error"]
        assert projmod.Project(proj.folder).data["settings"].get("copy_paste", "off") == "off"
        monkeypatch.setattr(st.trainer, "start", lambda minutes: "v1")
        monkeypatch.setattr(trainmod, "precheck", lambda p: dict(ok=True, text="Frames: enough."))
        r = _post(url + "api/studio/train", dict(minutes=5, copy_paste="compare"))
        r.pop("precheck")
        assert r == dict(model="v1", copy_paste="compare", copy_paste_before="off")
        assert projmod.Project(proj.folder).data["settings"]["copy_paste"] == "compare"
        code, j = _post_err(url + "api/studio/train", dict(minutes=5, copy_paste="maybe"))
        assert code == 400
        # a ready model with a comparison and hard negatives; scores by situation
        with proj.lock:
            proj.data["models"].append(dict(id="v1", status="ready", n_train_frames=40, created="2026-10-20T10:00:00",
                                            classes=proj.class_names(), metrics=dict(f1=0.8), onnx="models/v1/model.onnx",
                                            hard_negatives=dict(spots=7, frames=3, frames_only=1),
                                            copy_paste=dict(mode="compare", compared=True, used=False, winner="without",
                                                            text="Copy-paste made no clear difference, so this model was trained without it.",
                                                            without=dict(f1=0.8), **{"with": dict(f1=0.79)})))
            proj.save()
        os.makedirs(os.path.join(proj.folder, "models"), exist_ok=True)
        pt = dict(label="v1", recall=0.8, precision=0.8, f1=0.8, centre_err_px=2.0, heading_err_deg=5.0, ms_per_frame=4.0,
                  n_train_frames=40)
        projmod.write_json(os.path.join(proj.folder, "models", "curve.json"),
                           dict(points=[dict(pt, model="none", label="no model", f1=0.3), dict(pt, model="v1")],
                                test=dict(frames=40, blocks=2, block_s=30), updated="x"))
        row = dict(situation="alone", name="Alone", n_labels=40, n_objects=6, found=0.9, few=False, centre_err_px=2.0,
                   heading_err_deg=4.0)
        projmod.write_json(os.path.join(proj.folder, "models", "situations.json"),
                           dict(updated="x", test={}, models=dict(v1=[row, dict(row, situation="all", name="All")],
                                                                  none=[dict(row, found=0.4)])))
        c = _get(url + "api/studio/curve")
        assert "By situation" in c["situations_html"] and "Alone" in c["situations_html"]
        m = [x for x in c["models"] if x["id"] == "v1"][0]
        assert m["hard_negatives"]["spots"] == 7 and m["copy_paste"]["winner"] == "without"
        assert c["copy_paste"] == "compare"
    finally:
        server.shutdown()


# =============================================================================== 6/7. docs
def test_docs_cover_the_new_parts():
    readme = open(os.path.join(REC, "README.md"), encoding="utf-8").read()
    for words in ("Robot logs", "Home positions", "AI helper", "roll call", "By situation", "Copy-paste", "Reject here",
                  "Shift+Z", "How often should I check"):
        assert words in readme, words
    assert "in real life" not in readme.lower()
    df = open(os.path.join(os.path.dirname(TOOLS), "docs", "data-formats.md"), encoding="utf-8").read()
    for words in ("homes", "positions.csv", "ai_settings.json", "ai_log.jsonl", "outside.json", "checkins.json",
                  "chain_broken", "rejected_spots"):
        assert words in df, words


# =============================================================================== the page, in a browser
def _toast_has(page, *words, timeout=30000):
    js = "(ws) => { const t = document.getElementById('toast'); return t.classList.contains('show') && ws.every((w) => t.textContent.includes(w)); }"
    try:
        page.wait_for_function(js, arg=list(words), timeout=timeout)
    except Exception:
        raise AssertionError("no message with %r; the message shown: %r" % (words, page.text_content("#toast")))
    return page.text_content("#toast")


def _shot(page, name):
    if SHOTS:
        os.makedirs(SHOTS, exist_ok=True)
        page.screenshot(path=os.path.join(SHOTS, name + ".png"))


def _canvas_point(page, x, y):
    return page.evaluate("([x, y]) => { const b = view.getBoundingClientRect(); return [b.left + (x + 0.5) * S.scale, b.top + (y + 0.5) * S.scale]; }", [x, y])


def _idle(page, timeout=300000):
    page.wait_for_function("!document.getElementById('loading').classList.contains('on') && !S.waiting", timeout=timeout)


def test_click_through_every_new_part(tmp_path, roam, monkeypatch):
    pw = pytest.importorskip("playwright.sync_api")
    import homes as HM
    monkeypatch.setenv("MAXFIELD_IRL_HOME", str(tmp_path / "home"))
    proj, st, rid, server, url = _roam_project(tmp_path, roam)
    try:
        with pw.sync_playwright() as p:
            try:
                browser = p.chromium.launch()
            except Exception as e:
                pytest.skip("no Chromium for Playwright: %s" % e)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(url)
            page.wait_for_selector("#wask", timeout=60000)
            page.click("#wask")
            page.wait_for_function("S.frame !== null && !!S.img")
            assert not page.is_visible("#aiind") and not page.is_visible("#aibox")

            # --- Object types: how each type moves
            page.click("#smore"); page.click("#sclasses")
            page.wait_for_selector("#ctab .cspeed")
            assert "usual (3)" in page.get_attribute("#ctab tr:nth-child(2) .cspeed", "placeholder")
            _shot(page, "01-object-types-motion")
            page.fill("#ctab tr:nth-child(2) .cspeed", "4")
            page.select_option("#ctab tr:nth-child(2) .cstill", "yes")
            page.click("#csave")
            _toast_has(page, "Object types saved")
            _idle(page)
            assert projmod.Project(proj.folder).classes[0]["max_speed"] == 4.0

            # --- Train new model: the copy-paste choice (cancelled)
            page.click("#strain")
            page.wait_for_selector("#tcp")
            assert "keep whichever does better" in page.text_content("#tcp")
            _shot(page, "02-train-copy-paste")
            page.click("#modal .actions button:has-text(\"Cancel\")")

            # --- Work area: home positions (one by hand, one preset), saved, Z, Shift+Z
            page.keyboard.press("w")
            page.click("#wa-homes")
            page.wait_for_selector("#hnew")
            page.click("#hnew")
            page.mouse.click(*_canvas_point(page, 320, 240))
            page.mouse.click(*_canvas_point(page, 360, 240))
            page.wait_for_selector("#hadd")
            page.fill("#hname", "Tray")
            page.click("#hadd")
            _toast_has(page, "Home position Tray added", "not saved yet")
            mp = json.load(open(os.path.join(roam["out"], "map.json")))
            d = mp["home_docks"][0]
            g = HM.alvik_dock_geometry(2)
            cam = roam["camera"]
            hd = math.radians(d["heading"])
            wall = cam.project((d["x"], d["y"], g["wall_h_cm"]))
            mouth = cam.project((d["x"] + g["mouth_x_cm"] * math.cos(hd), d["y"] + g["mouth_x_cm"] * math.sin(hd), g["wall_h_cm"]))
            page.click("#wahomes .hpre")
            page.fill("#dname", "Dock L")
            page.fill("#down", "Left")
            page.mouse.click(*_canvas_point(page, *wall))
            page.mouse.click(*_canvas_point(page, *mouth))
            _toast_has(page, "Dock L added to the list")
            _shot(page, "03-home-positions")
            page.click("#hsave")
            _toast_has(page, "Home positions saved", "Z puts the old list back", timeout=300000)
            _idle(page)
            assert len(projmod.Project(proj.folder).data["cameras"]["default"]["homes"]) >= 2
            page.keyboard.press("z")
            _toast_has(page, "Undid: the home positions change", "Shift+Z puts it back", timeout=300000)
            _idle(page)
            assert projmod.Project(proj.folder).data["cameras"]["default"]["homes"] == []
            page.keyboard.press("Shift+Z")
            _toast_has(page, "Redid: the home positions change", timeout=300000)
            _idle(page)

            # --- Robot logs: use them, the names on the picture, stop, Z
            page.click("#smore"); page.click("#soutside")
            page.wait_for_selector("#opath")
            _shot(page, "04-outside-data-window")
            page.fill("#opath", roam["logs"])
            page.click("#ogo")
            _toast_has(page, "Outside data in use", "Z stops using it", timeout=300000)
            _idle(page)
            page.click("#tabs button[data-mode=all]")
            for _ in range(25):                                 # a frame where the logs name the objects
                page.wait_for_function("S.frame !== null")
                if page.evaluate("Object.keys(S.frame.names || {}).length") >= 1:
                    break
                page.keyboard.press("ArrowRight")
                page.wait_for_timeout(150)
            assert page.evaluate("Object.keys(S.frame.names || {}).length") >= 1      # only Left has a home here
            _shot(page, "05-names-from-robot-logs")
            page.click("#smore"); page.click("#soutside")
            page.wait_for_selector("#olineup")
            assert "Left" in page.text_content("#modal") and "home visits" in page.text_content("#modal")
            _shot(page, "06-outside-data-in-use")
            page.click("#ostop")
            _toast_has(page, "no longer used", "Z uses it again", timeout=300000)
            _idle(page)
            page.keyboard.press("z")
            _toast_has(page, "Undid: stopping the outside data", "used again", timeout=300000)
            _idle(page)

            # --- Learning curve: How often should I check?
            for g2 in st.session.selected[2:12:3]:
                objs = [dict(o, status="confirmed") for o in st.session.guess_objects(g2)]
                if objs:
                    st.session.save_frame(g2, objs, "reviewed")
            page.click("#smodels")
            page.wait_for_selector("#ckgo")
            page.click("#ckgo")
            page.wait_for_function("document.getElementById('ckbox') && /Check each object|However long|Not enough|Even shortly/.test(document.getElementById('ckbox').textContent)", timeout=180000)
            _shot(page, "07-how-often-should-i-check")
            page.keyboard.press("Escape")

            # --- AI helper (stand-in): settings, indicator, every button
            page.click("#smore"); page.click("#sai")
            page.wait_for_selector("#aisave")
            page.select_option("#aiprov", "stand_in")
            page.check("#aion")
            _shot(page, "08-ai-settings")
            page.click("#aisave")
            _toast_has(page, "AI helper switched on", "switches it off again")
            page.wait_for_selector("#aiind.on")
            assert page.is_visible("#aibox")
            page.evaluate("S.sel = S.frame.objects.length ? 0 : -1; draw(); renderSide();")
            if page.evaluate("S.sel") >= 0:
                page.click("#ainame")
                page.wait_for_selector("#aiyes")
                _shot(page, "09-ai-name")
                page.click("#aino")
                _toast_has(page, "turned down")
            page.click("#aicheck")
            page.wait_for_selector("#aigo")
            page.click("#aigo")
            page.wait_for_selector("#aiclose", timeout=60000)
            _shot(page, "10-ai-second-check")
            page.click("#aiclose")
            page.click("#aidraft")
            page.wait_for_selector("#aigo")
            page.click("#aigo")
            page.wait_for_selector("#aidrop", timeout=60000)
            _shot(page, "11-ai-draft-labels")
            n_before = page.evaluate("S.frame.objects.length")
            page.locator("[data-add]").first.click()
            _toast_has(page, "Added the AI's", "Z (or Del) takes it back")
            assert page.evaluate("S.frame.objects.length") == n_before + 1
            assert page.evaluate("S.frame.objects[S.frame.objects.length - 1].source") == "ai"
            page.wait_for_selector("#aidrop")
            page.click("#aidrop")
            page.keyboard.press("z")
            _toast_has(page, "Undid: added the AI's")
            # Investigate: Ask the AI
            page.mouse.move(*_canvas_point(page, 320, 240))
            page.keyboard.press("q")
            page.wait_for_selector("#invai", timeout=120000)
            page.click("#invai")
            page.wait_for_selector("#invaiyes", timeout=60000)
            _shot(page, "12-investigate-ask-the-ai")
            page.click("#invaiyes")
            _toast_has(page, "noted")
            page.click("#invclose")
            # Time saved counts the AI answers
            page.click("#smore"); page.click("#stime")
            page.wait_for_selector(".tsaved")
            assert "AI helper" in page.text_content(".tsaved")
            _shot(page, "13-time-saved-ai")
            page.keyboard.press("Escape")
            # the full tour shows the new buttons
            page.evaluate("() => { startTour(true); }")          # (it returns a promise that ends with the tour)
            titles = []
            for _ in range(60):
                titles.append(page.text_content("#tourcard h4"))
                if titles[-1] == "Robot logs / outside data":
                    _shot(page, "14-tour-robot-logs")
                box = page.evaluate("(() => { const b = document.getElementById('tourcard').getBoundingClientRect(); return [b.top, b.bottom]; })()")
                assert box[0] >= 0 and box[1] <= 900, (titles[-1], box)          # every card is on the screen
                if not page.is_visible("#tnext") or page.text_content("#tnext") == "Start labelling":
                    break
                page.keyboard.press("ArrowRight")
            assert {"Robot logs / outside data", "AI helper", "AI helper is on", "AI helper buttons"} <= set(titles), titles
            page.keyboard.press("Escape")
            assert not errors, errors

            # --- a new project: describe what you track in a sentence
            page2 = browser.new_page(viewport={"width": 1440, "height": 900})
            proj2 = projmod.Project(str(tmp_path / "new"), create_name="New")
            st2 = studiomod.Studio(proj2, log=lambda *a: None, background=False)
            server2, url2 = serve(st2)
            try:
                page2.goto(url2)
                page2.wait_for_selector("#tchoice")
                page2.click("#tchoice label[data-k=ai]")
                page2.wait_for_selector("#tsent")
                page2.fill("#tsent", "We track crates and cylinders on a conveyor")
                page2.select_option("#suprov", "stand_in")
                page2.click("#tsask")
                page2.wait_for_selector("#ctab .cname", timeout=60000)
                names = page2.eval_on_selector_all("#ctab .cname", "els => els.map((e) => e.value)")
                assert names[:2] == ["crate", "cylinder"], names
                _shot(page2, "15-setup-from-a-sentence")
                page2.fill("#pname", "Conveyor")
                page2.click("#setupgo")
                page2.wait_for_selector("#tourcard", timeout=30000)
                assert projmod.Project(proj2.folder).class_names()[:2] == ["crate", "cylinder"]
                import ai_helper
                assert ai_helper.AIHelper(proj2.folder).accepted_count() == 1
            finally:
                server2.shutdown()
            browser.close()
    finally:
        server.shutdown()
