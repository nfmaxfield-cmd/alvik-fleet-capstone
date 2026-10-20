"""The handoff build of the studio: name, generic wording, first-start setup and what's new, the
work area, height "it varies", unsure frames kept out of training, the background pass for the
second camera, and the user manual."""
import csv
import json
import os
import re
import shutil
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
ROOT = os.path.dirname(TOOLS)
for p in (REC, os.path.join(TOOLS, "label"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import project as projmod  # noqa: E402
import report  # noqa: E402
import studio as studiomod  # noqa: E402
import trainer  # noqa: E402
from test_studio import fake_recording, write_labels, COLS  # noqa: E402

BRAND, TAGLINE = "Maxfield IRL Tracking", "Identify. Recognize. Label."
PAGE = os.path.join(TOOLS, "label", "page.html")
README = os.path.join(REC, "README.md")


def read(p):
    with open(p, encoding="utf-8") as f:
        return f.read()


# ---------------------------------------------------------------- 1. name
def test_name_everywhere_a_person_sees_it(tmp_path):
    page = read(PAGE)
    assert "<title>%s</title>" % BRAND in page and TAGLINE in page
    assert read(README).startswith("# " + BRAND) and TAGLINE in read(README)
    bat = read(os.path.join(ROOT, "start-recognition.bat"))
    assert "title " + BRAND in bat
    start = read(os.path.join(REC, "start.py"))
    assert BRAND in start and TAGLINE in start
    proj = projmod.Project(str(tmp_path / "p"), create_name="Cubes")
    html = report.html(proj)
    assert BRAND in html and TAGLINE in html


def test_never_written_out():
    """The name's IRL is never spelled out."""
    files = [PAGE, README, os.path.join(ROOT, "start-recognition.bat")]
    for d in (REC, os.path.join(TOOLS, "label")):
        files += [os.path.join(d, f) for f in os.listdir(d) if f.endswith(".py") or f.endswith(".md")]
    phrase = " ".join(["in", "real", "life"])
    for f in files:
        assert phrase not in read(f).lower(), f


# ---------------------------------------------------------------- 2. generic
def _lines_with_alvik(text):
    return [ln for ln in text.splitlines() if "alvik" in ln.lower()]


def test_user_facing_text_is_generic(tmp_path):
    """No Alvik wording outside the ready-made "Alvik robots" set (and the code that reads
    older projects, or the motion finder's internal names)."""
    assert not _lines_with_alvik(read(PAGE))
    bad = [ln for ln in _lines_with_alvik(read(README)) if "Alvik robots" not in ln]
    assert not bad, bad
    for name in ("start.py", "studio.py", "report.py"):
        assert not _lines_with_alvik(read(os.path.join(REC, name))), name
    # project.py: only the preset, and the motion-name map that keeps older projects working
    src = read(os.path.join(REC, "project.py"))
    pre = src[src.index("PRESETS = ["):src.index("# Names the motion finder")]
    cmap = src[src.index("def motion_class_map"):src.index("def loose_classes")]
    rest = src.replace(pre, "").replace(cmap, "")
    assert not _lines_with_alvik(rest), _lines_with_alvik(rest)
    # app.py: only the standalone labelling app's default type list
    bad = [ln for ln in _lines_with_alvik(read(os.path.join(TOOLS, "label", "app.py"))) if not ln.startswith("CLASSES =")]
    assert not bad, bad
    # the report of a project that isn't about robots says nothing about them
    proj = projmod.Project(str(tmp_path / "p"), create_name="Conveyor")
    proj.finish_setup(classes=[dict(name="cube"), dict(name="cylinder")])
    fake_recording(str(tmp_path / "r"))
    proj.add_recording(str(tmp_path / "r"), "top")
    html = report.html(proj).lower()
    assert "alvik" not in html and not re.search(r"\brobots?\b", html)
    # what start.py and studio.py print
    out = subprocess.run([sys.executable, os.path.join(REC, "studio.py"), "--help"], capture_output=True, text=True).stdout
    assert BRAND in out and "alvik" not in out.lower()


def test_new_project_starts_with_only_the_catch_all(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    assert proj.class_names() == ["unknown"]
    assert proj.setup_view()["need_setup"]
    # colours come by position in the page; the motion finder's guesses go to the project's types
    proj.set_classes([dict(name="cube", has_front=False)])
    assert proj.class_names() == ["cube", "unknown"]
    assert proj.motion_class_map() == {"alvik": "cube", "hand": "unknown", "unknown": "unknown", "machine": "unknown"}
    proj.set_classes([dict(name="cube"), dict(name="cylinder")])
    assert proj.motion_class_map()["alvik"] == "unknown"          # two plain types: motion can't tell them apart
    proj.set_classes([dict(name="part"), dict(name="arm", height_varies=True)])
    m = proj.motion_class_map()
    assert m["alvik"] == "part" and m["hand"] == "arm"
    assert set(proj.loose_classes()) == {"arm", "unknown"}
    with pytest.raises(ValueError):
        proj.set_classes([dict(name="none")])


def test_preset_alvik_robots(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    preset = [p for p in projmod.PRESETS if p["title"] == "Alvik robots"][0]
    proj.finish_setup("Lab", preset["classes"], seconds=42)
    c = {x["name"]: x for x in proj.classes}
    assert proj.class_names() == ["alvik", "person", "unknown"]
    assert c["alvik"]["has_front"] and c["alvik"]["height_cm"] == 4.0 and not c["alvik"]["height_varies"]
    assert c["person"]["height_varies"] and c["unknown"]["height_varies"]
    assert proj.motion_class_map()["hand"] == "person" and proj.motion_class_map()["alvik"] == "alvik"
    assert proj.data["settings"]["setup_done"] and proj.data["settings"]["setup_seconds"] == 42
    assert proj.name == "Lab" and not proj.setup_view()["need_setup"] and not proj.setup_view()["whats_new"]


# ---------------------------------------------------------------- 3. older projects
OLD_CLASSES = [dict(name="alvik", has_front=True, size_px=None, height_cm=4.0),
               dict(name="hand", has_front=False, size_px=None, height_cm=10.0),
               dict(name="unknown", has_front=False, size_px=None, height_cm=0.0),
               dict(name="machine", has_front=True, size_px=None, height_cm=0.0)]


def old_project(tmp_path, labelled=True):
    """A project.json as the previous version wrote it."""
    folder = tmp_path / "old"
    fake_recording(str(tmp_path / "r"))
    data = dict(version=1, name="Old", created="2026-10-19T10:00:00", classes=[dict(c) for c in OLD_CLASSES],
                recordings=[dict(id="r", path=str(tmp_path / "r"), name="r", camera="overhead", session=None,
                                 added="2026-10-19T10:00:00")],
                models=[], cameras={}, current=dict(rec="r", model="none"), settings=dict(train_minutes=20, block_s=30))
    os.makedirs(folder / "labels" / "r", exist_ok=True)
    with open(folder / "project.json", "w") as f:
        json.dump(data, f, indent=1)
    if labelled:
        write_labels(str(folder / "labels" / "r"), [[5, "", 1, "alvik", 10, 10, "", 0, "user", "confirmed", ""]])
        with open(folder / "labels" / "r" / "review.json", "w") as f:
            json.dump(dict(frames={"5": dict(state="reviewed", how="user", changed=False, auto_conf=0.9,
                                             updated="2026-10-19T10:05:00")}), f)
    return str(folder)


def test_old_project_opens_unchanged_and_gets_whats_new(tmp_path):
    folder = old_project(tmp_path)
    labels_before = read(os.path.join(folder, "labels", "r", "labels.csv"))
    proj = projmod.Project(folder)
    assert proj.class_names() == ["alvik", "hand", "unknown", "machine"]
    v = proj.setup_view()
    assert not v["need_setup"] and v["whats_new"] and v["offer_varies"] == ["hand", "unknown"]
    assert proj.motion_class_map() == {"alvik": "alvik", "hand": "hand", "unknown": "unknown", "machine": "machine"}
    assert set(proj.loose_classes()) == {"hand", "unknown"}       # the tracker behaves as before
    assert proj.varies_classes() == []                              # nothing changes until confirmed
    assert proj.finish_whats_new(set_varies=True) == ["hand", "unknown"]
    proj2 = projmod.Project(folder)
    c = {x["name"]: x for x in proj2.classes}
    assert c["hand"]["height_varies"] and c["unknown"]["height_varies"] and not c["alvik"].get("height_varies")
    assert c["hand"]["height_cm"] == 10.0                           # kept, only no longer used
    assert not proj2.setup_view()["whats_new"]
    assert read(os.path.join(folder, "labels", "r", "labels.csv")) == labels_before


def test_old_project_whats_new_declined(tmp_path):
    proj = projmod.Project(old_project(tmp_path))
    assert proj.finish_whats_new(set_varies=False) == []
    assert projmod.Project(proj.folder).varies_classes() == []


def test_old_project_without_labels_gets_the_setup(tmp_path):
    proj = projmod.Project(old_project(tmp_path, labelled=False))
    assert proj.setup_view()["need_setup"]


# ---------------------------------------------------------------- 6. unsure kept out of training
def test_unsure_frames_are_left_out_of_the_dataset(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    proj.set_classes([dict(name="cube")])
    fake_recording(str(tmp_path / "r"), n=600)
    rid = proj.add_recording(str(tmp_path / "r"))[0]
    rows = []
    for fr in range(15, 600, 30):
        rows.append([fr, "", 1, "cube", 100, 100, "", 0, "user", "confirmed", ""])
        if fr == 135:
            rows.append([fr, "", 2, "cube", 200, 100, "", 0, "user", "unsure", ""])
    write_labels(proj.labels_dir(rid), rows)
    assign, info = trainer.build_dataset(proj, str(tmp_path / "d"), block_s=10)
    frames = []
    for name in assign:
        with open(tmp_path / "d" / name / "labels.csv") as f:
            frames += [int(r["frame"]) for r in csv.DictReader(f)]
    assert 135 not in frames and 165 in frames            # (135 would be kept: 13.5 s is far from a block edge)


# ---------------------------------------------------------------- 8. height varies
def test_varies_types_are_not_passed_between_cameras(tmp_path):
    import synth
    cam_a, cam_b = synth.cam_overhead(), synth.cam_oblique()
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    proj.set_classes([dict(name="part", height_cm=4), dict(name="arm", height_varies=True)])
    sess = tmp_path / "S"
    fake_recording(str(sess / "cam0"), n=50, t0=500.0)
    fake_recording(str(sess / "cam1"), n=50, t0=500.02)
    with open(sess / "session.json", "w") as f:
        json.dump(dict(cameras=[dict(folder="cam0", role="overhead"), dict(folder="cam1", role="side")]), f)
    ids = proj.add_recording(str(sess))
    cam_a.save(proj.camera_model_path("overhead"))
    cam_b.save(proj.camera_model_path("side"))
    ua, va = cam_a.project(np.array([[60.0, 40.0, 4.0]]))
    ua, va = float(np.ravel(ua)[0]), float(np.ravel(va)[0])
    write_labels(proj.labels_dir(ids[0]), [[10, "", 1, "part", "%.2f" % ua, "%.2f" % va, "", 0, "user", "confirmed", ""],
                                           [10, "", 2, "arm", "%.2f" % (ua + 40), "%.2f" % va, "", 0, "user", "confirmed", ""]])
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    extra = st._cross_camera_guesses(proj.rec(ids[1]))
    assert [g["cls"] for g in extra[10]] == ["part"]


def test_height_column_title(tmp_path):
    assert "Height of the marked point above the floor" in read(PAGE) and "(cm, for the second camera)" in read(PAGE)
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    proj.set_classes([dict(name="arm", height_varies=True)])
    assert "it varies" in report.html(proj) and "Height of the marked point above the floor" in report.html(proj)


# ---------------------------------------------------------------- 9. work area storage
def test_work_area_settings(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    assert projmod.Project.camera_key(dict(camera="")) == "default"
    assert projmod.Project.camera_key(dict(camera="overhead")) == "overhead"
    c = proj.camera_settings("overhead")
    assert c["work_area"] is None and c["margin_px"] is None and c["ignore_spots"] == [] and not c["asked"]
    proj.set_camera_settings("overhead", [[1, 2], [100, 2], [100, 90]], None, [[50, 50, 6]], [640, 480])
    c = projmod.Project(proj.folder).camera_settings("overhead")
    assert c["work_area"] == [[1.0, 2.0], [100.0, 2.0], [100.0, 90.0]] and c["ignore_spots"] == [[50.0, 50.0, 6.0]]
    assert c["asked"] and c["margin_px"] is None
    with pytest.raises(ValueError):
        proj.set_camera_settings("overhead", [[1, 2], [3, 4]])
    with pytest.raises(ValueError):
        proj.set_camera_settings("overhead", None, None, [[1, 1, 0]])


# ---------------------------------------------------------------- web routes and background pass
def _get(url):
    with urllib.request.urlopen(url, timeout=60) as r:
        return json.loads(r.read())


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read())


def wait_open(st, timeout=180):
    t0 = time.time()
    while (st.loading is not None or st.pending_reopen) and time.time() - t0 < timeout:
        time.sleep(0.1)
    assert st.session is not None, st.loading


@pytest.fixture(scope="module")
def synth10(tmp_path_factory):
    d = tmp_path_factory.mktemp("synth10")
    subprocess.run([sys.executable, os.path.join(TOOLS, "label", "tests", "synth.py"), str(d / "rec"), "--seconds", "10"],
                   check=True, capture_output=True)
    return str(d / "rec")


def test_setup_and_work_area_routes(tmp_path, synth10):
    proj = projmod.Project(str(tmp_path / "p"), create_name="Web")
    rid = proj.add_recording(synth10, "overhead")[0]
    st = studiomod.Studio(proj, log=lambda *a: None)
    st.open(rid, "none")
    get, post = studiomod.make_routes(st)
    server, url = studiomod.labelapp.serve(st.current, 0, extra_get=get, extra_post=post)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        s = _get(url + "api/studio/state")
        assert s["brand"] == dict(name=BRAND, tagline=TAGLINE)
        assert s["setup"]["need_setup"] and s["setup"]["presets"][0]["title"] == "Alvik robots"
        assert s["camera"]["key"] == "overhead" and not s["camera"]["settings"]["asked"]
        # setup while the recording is still opening: applied as soon as it is open
        _post(url + "api/studio/setup", dict(name="Robots", classes=[dict(name="bot", has_front=True)], seconds=12.5))
        wait_open(st)
        info = _get(url + "api/info")
        assert info["classes"] == ["bot", "unknown"] and info["area"]["work_area"] is None
        assert any(g["cls"] == "bot" for gs in st.session.guesses.values() for g in gs)
        s = _get(url + "api/studio/state")
        assert not s["setup"]["need_setup"] and s["project"]["name"] == "Robots"
        # the work area: saved per camera, and the recording opens again with it
        n_before = sum(len(g) for g in st.session.dets if g)
        r = _post(url + "api/studio/workarea", dict(camera="overhead", work_area=[[0, 0], [320, 0], [320, 480], [0, 480]],
                                                     margin_px=0, ignore_spots=[], image_size=[640, 480]))
        assert r["reopened"]
        wait_open(st)
        info = _get(url + "api/info")
        assert info["area"]["work_area"][1] == [320.0, 0.0] and info["area"]["dropped"] > 0
        assert all(d["x"] <= 320.5 for g in st.session.dets if g for d in g)
        assert sum(len(g) for g in st.session.dets if g) < n_before
        # "not now" only notes that it was offered
        _post(url + "api/studio/workarea", dict(camera="side", skip=True))
        assert proj.camera_settings("side")["asked"] and proj.camera_settings("side")["work_area"] is None
        # the unsure list and its count come through the page's routes
        q = _get(url + "api/queue?mode=unsure")
        assert q["frames"] == [] and q["stats"]["unsure"] == 0
    finally:
        server.shutdown()
        server.server_close()


def test_whats_new_route(tmp_path):
    proj = projmod.Project(old_project(tmp_path))
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    get, post = studiomod.make_routes(st)
    server, url = studiomod.labelapp.serve(st.current, 0, extra_get=get, extra_post=post)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        s = _get(url + "api/studio/state")
        assert s["setup"]["whats_new"] and s["setup"]["offer_varies"] == ["hand", "unknown"]
        r = _post(url + "api/studio/whats_new", dict(set_varies=True))
        assert r["changed"] == ["hand", "unknown"]
        assert not _get(url + "api/studio/state")["setup"]["whats_new"]
    finally:
        server.shutdown()
        server.server_close()


def test_second_camera_motion_pass_runs_in_the_background(tmp_path, synth10):
    sess = tmp_path / "S2"
    for cam in ("cam0", "cam1"):
        shutil.copytree(synth10, str(sess / cam), ignore=shutil.ignore_patterns("labels"))
    with open(sess / "session.json", "w") as f:
        json.dump(dict(cameras=[dict(folder="cam0", role="overhead"), dict(folder="cam1", role="side")]), f)
    proj = projmod.Project(str(tmp_path / "p"), create_name="Two")
    ids = proj.add_recording(str(sess))
    st = studiomod.Studio(proj, log=lambda *a: None)
    st.open(ids[0], "none", wait=True)
    assert st.bg_thread is not None
    seen = []
    while st.bg_thread.is_alive():
        if st.bg:
            seen.append(dict(st.bg))
        time.sleep(0.05)
    assert st.bg is None and seen and seen[0]["rid"] == ids[1] and "ready" in seen[0]["what"]
    rec1 = studiomod.labelapp.Recording(proj.rec(ids[1])["path"])
    assert studiomod.labelapp.has_motion_cache(rec1, os.path.join(proj.labels_dir(ids[1]), "cache"))
    t0 = time.time()
    st.open(ids[1], "none", wait=True)                      # quick now: the slow pass is done
    assert st.session.rec.folder == proj.rec(ids[1])["path"] and time.time() - t0 < 60


# ---------------------------------------------------------------- the page, in a browser
def test_first_start_in_a_browser(tmp_path, synth10):
    """Setup screen, the tour, and the offer to mark the work area (skipped without Playwright)."""
    pw = pytest.importorskip("playwright.sync_api")
    proj = projmod.Project(str(tmp_path / "p"), create_name="Browser")
    rid = proj.add_recording(synth10, "overhead")[0]
    st = studiomod.Studio(proj, log=lambda *a: None)
    st.open(rid, "none", wait=True)
    get, post = studiomod.make_routes(st)
    server, url = studiomod.labelapp.serve(st.current, 0, extra_get=get, extra_post=post)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with pw.sync_playwright() as p:
            try:
                browser = p.chromium.launch()
            except Exception as e:
                pytest.skip("no Chromium for Playwright: %s" % e)
            page = browser.new_page(viewport={"width": 1400, "height": 860})
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(url)
            page.wait_for_selector("#tchoice")
            assert page.is_disabled("#setupgo")                     # nothing judged before a choice
            page.click("#tchoice label[data-k=p0]")                 # the ready-made set
            assert page.input_value("#ctab tr:nth-child(2) .cname") == "alvik"
            page.click("#setupgo")
            page.wait_for_selector("#tour.on", timeout=60000)
            steps = 0
            while page.is_visible("#tour.on") and steps < 40:
                page.keyboard.press("ArrowRight")
                steps += 1
            assert steps >= 18
            page.wait_for_selector("#wask", timeout=30000)
            page.click("#wask")
            page.wait_for_timeout(500)
            page.keyboard.press("w")
            assert page.is_visible("#wabar")
            page.keyboard.press("Escape")
            assert not page.is_visible("#wabar")
            page.click("#helpbtn")
            assert page.is_visible("#tour.on")
            page.keyboard.press("Escape")
            browser.close()
        assert not errors, errors
        assert proj.class_names() == ["alvik", "person", "unknown"]
        assert projmod.Project(proj.folder).camera_settings("overhead")["asked"]
    finally:
        server.shutdown()
        server.server_close()


# ---------------------------------------------------------------- 11. the manual
def test_readme_is_a_manual():
    text = read(README)
    for heading in ("Install", "First start", "The loop", "Keys", "Unsure", "Investigate", "Work area",
                    "Two cameras", "Starting over", "Where your files are"):
        assert re.search(r"^## .*%s" % heading, text, re.M), heading
    for key in ("| ? |", "| M |", "| B |", "| V |", "| W |", "| Z |"):
        assert key in text, key
