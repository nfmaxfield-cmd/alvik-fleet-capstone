"""Investigate's studio parts (ignore this spot, the second camera as referee), the time-saving
stats of a project (record, estimates, report, CSV), the project-file buttons (share, open, switch,
"recording not found: locate it"), and the "use it" tool reading the studio's work area."""
import csv
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import urllib.error
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
import referee  # noqa: E402
import report  # noqa: E402
import studio as studiomod  # noqa: E402
import timesaved  # noqa: E402
import timing  # noqa: E402
from test_studio import fake_recording, write_labels  # noqa: E402

SHOTS = os.environ.get("LANE4_SHOTS")


def _cams():
    """This folder's synth.py (camera models); tools/label/tests has a synth.py of its own."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("recognition_synth", os.path.join(HERE, "synth.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def synth10(tmp_path_factory):
    d = tmp_path_factory.mktemp("synth10l4")
    subprocess.run([sys.executable, os.path.join(TOOLS, "label", "tests", "synth.py"), str(d / "rec"), "--seconds", "10"],
                   check=True, capture_output=True)
    return str(d / "rec")


def _get(url):
    with urllib.request.urlopen(url, timeout=60) as r:
        return json.loads(r.read())


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=120) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return dict(json.loads(e.read()), http=e.code)


def wait_open(st, timeout=180):
    t0 = time.time()
    while (st.loading is not None or st.pending_reopen) and time.time() - t0 < timeout:
        time.sleep(0.1)
    assert st.session is not None, st.loading


def serve(st):
    get, post = studiomod.make_routes(st)
    server, url = studiomod.labelapp.serve(st.current, 0, extra_get=get, extra_post=post)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, url


def ready_project(folder, rec, camera="overhead", name="Lane4"):
    proj = projmod.Project(folder, create_name=name)
    proj.finish_setup(name, [dict(name="cube", has_front=True)], seconds=33)
    rid = proj.add_recording(rec, camera)[0]
    key = proj.camera_key(proj.rec(rid))
    proj.set_camera_settings(key, None, None, [], None, asked=True)
    return proj, rid


# ---------------------------------------------------------------- time saved
def test_time_saved_of_a_project_with_estimates_and_report(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="Parts")
    proj.finish_setup("Parts", [dict(name="cube")], seconds=25)
    fake_recording(str(tmp_path / "r"))
    rid = proj.add_recording(str(tmp_path / "r"), "top")[0]
    ld = proj.labels_dir(rid)
    write_labels(ld, [[1, "", 1, "cube", 10, 10, "", 0, "user", "confirmed", ""],
                      [2, "", 1, "cube", 10, 10, "", 0, "user", "unsure", ""]])
    review = dict(frames={"1": dict(state="reviewed", how="user", changed=False, updated="2026-10-20T10:00:00"),
                          "2": dict(state="reviewed", how="user", changed=True, updated="2026-10-20T10:00:20"),
                          "3": dict(state="auto", how="auto", changed=False, updated="2026-10-20T10:01:00")})
    with open(os.path.join(ld, "review.json"), "w") as f:
        json.dump(review, f)
    review_before = open(os.path.join(ld, "review.json")).read()
    sp = timesaved.stats_path(proj, rid)
    for k in range(5):
        timing.append(sp, dict(frame=10 + k, outcome="corrected", active_s=20.0, edits=2, model="none", guesses="motion"))
    for k in range(5):
        timing.append(sp, dict(frame=20 + k, outcome="accepted", active_s=4.0, edits=0, model="v1", guesses="model"))
    timing.append(sp, dict(frame=20, outcome="undone"))
    timing.append(sp, dict(frame=20, outcome="accepted", active_s=3.0, edits=0, model="v1"))   # finished again
    timesaved.record_install(proj, 312.4)
    timesaved.record_install(proj, 1.0)                                    # only the first install counts
    frames = timesaved.project_frames(proj)
    est = [f for f in frames if f["estimated"]]
    assert sorted(f["frame"] for f in est) == [1, 2, 3]
    assert next(f for f in est if f["frame"] == 2)["outcome"] == "unsure"
    f20 = next(f for f in frames if f["frame"] == 20)
    assert f20["outcome"] == "accepted" and f20["active_s"] == 7.0         # all the time spent on it
    sm = timesaved.summary(proj)
    # measured and estimated frames are separate rows (models are compared only on measured frames)
    meas = {(r["model"], r["guesses"]): r for r in sm["by_model"] if r["measured"]}
    est_rows = [r for r in sm["by_model"] if not r["measured"]]
    assert meas[("none", "motion")]["frames"] == 5 and meas[("none", "motion")]["seconds_per_frame"] == 20.0
    assert sum(r["frames"] for r in est_rows if r["model"] == "none") == 2
    assert sum(r["frames"] for (m, g), r in meas.items() if m == "v1") == 5
    assert meas[("v1", "model")]["accepted_unchanged"] == 1.0
    assert sm["totals"]["seconds_per_frame"] == round((5 * 20.0 + 4 * 4.0 + 7.0) / 10, 1)   # measured only
    assert sm["totals"]["install_seconds"] == 312.4 and sm["totals"]["setup_seconds"] == 25
    assert sm["totals"]["estimated"] == 3 and sm["totals"]["auto"] == 1
    html = report.html(proj)
    assert "Time saved" in html and "Accepted unchanged" in html and "estimated" in html
    assert "hardest frames first" in html and "not counted as time saved" in html
    assert "first install 5 min" in html and "project setup 25 s" in html
    low = html.lower()
    assert "alvik" not in low and " robot" not in low
    assert open(os.path.join(ld, "review.json")).read() == review_before     # only read
    # the command line writes both tables
    out = subprocess.run([sys.executable, os.path.join(REC, "timesaved.py"), proj.folder], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    with open(os.path.join(proj.folder, "stats", "time-saved-summary.csv")) as f:
        assert f.readline().startswith("model,guesses,measured,frames,seconds_per_frame")


def test_report_without_any_frames_says_so(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="Empty")
    assert "No frames finished yet." in report.html(proj)


# ---------------------------------------------------------------- referee
def test_referee_says_why_it_is_not_available(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    fake_recording(str(tmp_path / "r"))
    rid = proj.add_recording(str(tmp_path / "r"), "top")[0]
    r = referee.look(proj, proj.rec(rid), 5, 100, 100, "unknown")
    assert not r["available"] and "two-camera session" in r["reason"]
    sess = tmp_path / "S"
    fake_recording(str(sess / "cam0"))
    fake_recording(str(sess / "cam1"))
    with open(sess / "session.json", "w") as f:
        json.dump(dict(cameras=[dict(folder="cam0", role="overhead"), dict(folder="cam1", role="side")]), f)
    ids = proj.add_recording(str(sess))
    r = referee.look(proj, proj.rec(ids[0]), 5, 100, 100, "unknown")
    assert not r["available"] and "isn't calibrated" in r["reason"]
    synth = _cams()
    synth.cam_overhead().save(proj.camera_model_path("overhead"))
    r = referee.look(proj, proj.rec(ids[0]), 5, 100, 100, "unknown")
    assert not r["available"] and "other camera" in r["reason"]


def test_referee_shows_the_other_camera_at_the_same_moment(tmp_path, synth10):
    synth = _cams()
    cam_a, cam_b = synth.cam_overhead(), synth.cam_oblique()
    sess = tmp_path / "S"
    for cam in ("cam0", "cam1"):
        shutil.copytree(synth10, str(sess / cam), ignore=shutil.ignore_patterns("labels"))
    with open(sess / "session.json", "w") as f:
        json.dump(dict(cameras=[dict(folder="cam0", role="overhead"), dict(folder="cam1", role="side")]), f)
    proj = projmod.Project(str(tmp_path / "p"), create_name="Two")
    proj.set_classes([dict(name="part", height_cm=4), dict(name="arm", height_varies=True)])
    ids = proj.add_recording(str(sess))
    cam_a.save(proj.camera_model_path("overhead"))
    cam_b.save(proj.camera_model_path("side"))
    u, v = cam_a.project((60.0, 40.0, 4.0))
    r = referee.look(proj, proj.rec(ids[0]), 30, u, v, "part")
    assert r["available"] and r["camera"] == "side" and r["frame"] == 30 and abs(r["dt_s"]) < 0.05
    ub, vb = cam_b.project((60.0, 40.0, 4.0))
    assert abs(r["point"][0] - ub) < 0.5 and abs(r["point"][1] - vb) < 0.5
    assert r["picture"].startswith("data:image/jpeg;base64,") and "4.0 cm above the floor" in r["text"]
    r2 = referee.look(proj, proj.rec(ids[0]), 30, u, v, "arm")              # height varies: the floor spot
    assert r2["available"] and "height varies" in r2["text"]
    # through the studio route
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    proj.data["current"] = dict(rec=ids[0], model="none")
    assert st.referee(30, u, v, "part")["available"]


# ---------------------------------------------------------------- studio routes
def test_old_ignore_spots_kept_and_the_ignore_spot_route_is_gone(tmp_path, synth10):
    """Stage 2: "Ignore this spot" is gone (a rejection applies to one frame: Reject here); ignore
    spots saved before keep working."""
    proj, rid = ready_project(str(tmp_path / "p"), synth10)
    proj.set_camera_settings("overhead", [[0, 0], [640, 0], [640, 480], [0, 480]], 12, [[5, 5, 4]], [640, 480])
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    st.open(rid, "none", wait=True)
    server, url = serve(st)
    try:
        g = next(g for gs in st.session.guesses.values() for g in gs)
        assert _post(url + "api/studio/ignore_spot", dict(camera="overhead", x=g["x"], y=g["y"], r=15))["http"] == 404
        c = proj.camera_settings("overhead")
        assert c["ignore_spots"] == [[5.0, 5.0, 4.0]] and c["work_area"] and c["margin_px"] == 12
        info = _get(url + "api/info")
        assert info["area"]["ignore_spots"] == [[5.0, 5.0, 4.0]]
        # investigate through the page's routes
        f = st.session.selected[2]
        _post(url + "api/investigate", dict(frame=f, x=100, y=100))
        t0 = time.time()
        while _get(url + "api/investigate")["status"] == "running" and time.time() - t0 < 60:
            time.sleep(0.1)
        v = _get(url + "api/investigate")
        assert v["status"] == "done" and v["result"]["kind"] in ("background", "background_visited", "moves", "appears")
        assert _post(url + "api/investigate/change", dict(items=[], action="nonsense"))["http"] == 400
        # time saved, as the panel and the CSV downloads get it
        fr = _get(url + "api/frame?f=%d" % f)
        _post(url + "api/frame", dict(frame=f, objects=fr["objects"], state="reviewed", how="user",
                                      timing=dict(shown="2026-10-20T10:00:00", active_s=9.5, edits=1)))
        h = _get(url + "api/studio/timesaved_html")
        assert "Time saved" in h["html"] and h["summary"]["totals"]["frames_by_person"] == 1
        with urllib.request.urlopen(url + "api/studio/timesaved.csv?what=frames") as resp:
            assert "attachment" in resp.headers["Content-Disposition"]
            body = resp.read().decode()
        assert body.splitlines()[0] == ",".join(timing.CSV_COLUMNS)
        assert (",9.5,1,accepted," in body) or (",9.5,1,corrected," in body)
        with urllib.request.urlopen(url + "api/studio/timesaved.csv?what=summary") as resp:
            assert resp.read().decode().startswith("model,")
    finally:
        server.shutdown()
        server.server_close()


def test_share_open_switch_and_locate_a_missing_recording(tmp_path, synth10):
    rec = str(tmp_path / "recs" / "rec")
    shutil.copytree(synth10, rec, ignore=shutil.ignore_patterns("labels"))
    proj, rid = ready_project(str(tmp_path / "p"), rec, name="Handover")
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    st.open(rid, "none", wait=True)
    s = st.session
    s.save_frame(s.selected[0], s.frame_view(s.selected[0])["objects"], "reviewed", "spot", dict(active_s=4))
    server, url = serve(st)

    def wait_share():
        t0 = time.time()
        while time.time() - t0 < 120:
            sh = _get(url + "api/studio/state")["share"]
            if sh and sh["status"] != "running":
                return sh
            time.sleep(0.1)
        raise AssertionError("share job did not finish")
    try:
        state = _get(url + "api/studio/state")
        assert state["documents"] and state["missing"] == [] and state["share"] is None
        assert _post(url + "api/studio/share/export", dict(path=""))["http"] == 400      # a plain message
        out = str(tmp_path / "out" / "handover.mxirl")
        _post(url + "api/studio/share/export", dict(path=out, with_video=False))
        sh = wait_share()
        assert sh["status"] == "done" and sh["result"]["path"] == out and sh["done"] == sh["total"] > 0
        _post(url + "api/studio/share/clear", {})
        assert _get(url + "api/studio/state")["share"] is None
        # the other computer: the recordings are elsewhere (moved), so the project opens without them
        shutil.move(str(tmp_path / "recs"), str(tmp_path / "moved"))
        assert _post(url + "api/studio/share/import", dict(file=str(tmp_path / "nope.mxirl"), dest="x"))["http"] == 400
        dest = str(tmp_path / "theirs")
        _post(url + "api/studio/share/import", dict(file=out, dest=dest))
        sh = wait_share()
        assert sh["status"] == "done" and sh["result"]["found"] == 0 and sh["result"]["recordings"] == 1
        assert "Recording not found" in sh["result"]["notes"][0]
        r = _post(url + "api/studio/switch", dict(folder=dest))
        assert r["ok"] and not r["opened"]
        state = _get(url + "api/studio/state")
        assert state["project"]["folder"] == dest and state["missing"][0]["id"] == rid
        r = _post(url + "api/studio/open", dict(rec=rid, model="none"))
        assert r["http"] == 409 and r["missing"]["id"] == rid and "Recording not found" in r["error"]
        bad = _post(url + "api/studio/relink", dict(rec=rid, path=str(tmp_path)))
        assert bad["http"] == 400 and "video.avi" in bad["error"]
        r = _post(url + "api/studio/relink", dict(folder=str(tmp_path / "moved")))
        assert r["found"] == [rid] and r["missing"] == []
        assert _post(url + "api/studio/open", dict(rec=rid, model="none"))["ok"]
        wait_open(st)
        assert st.session.stats()["reviewed"] == 1                            # the labels came along
        assert len(timing.read(timesaved.stats_path(st.proj, rid))) == 1      # and the time record
    finally:
        server.shutdown()
        server.server_close()


def test_studio_starts_with_a_missing_recording(tmp_path, synth10):
    """The studio's start must not stop on a missing recording (the page asks where it is)."""
    rec = str(tmp_path / "rec")
    shutil.copytree(synth10, rec, ignore=shutil.ignore_patterns("labels"))
    proj, rid = ready_project(str(tmp_path / "p"), rec)
    shutil.rmtree(rec)
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    with pytest.raises(studiomod.RecordingMissing):
        st.open(rid, "none")
    assert st.state()["missing"][0]["id"] == rid


def test_install_time_from_the_start_command(tmp_path):
    folder = str(tmp_path / "p")
    projmod.Project(folder, create_name="P")
    env = dict(os.environ, MXIRL_INSTALL_SECONDS="95.5", MXIRL_BANNER_SHOWN="1")
    code = ("import sys; sys.argv=['studio.py', %r, '--no-browser', '--port', '0'];"
            "import studio, threading;"
            "studio.labelapp.serve = lambda *a, **k: (_ for _ in ()).throw(SystemExit(0));"
            "studio.main()") % folder
    subprocess.run([sys.executable, "-c", code], cwd=REC, env=env, capture_output=True, text=True, timeout=120)
    assert timesaved.install_seconds(projmod.Project(folder)) == 95.5


# ---------------------------------------------------------------- the "use it" tool reads the studio's work area
def test_use_model_reads_the_work_area_the_studio_saved(tmp_path):
    import use_model as um
    import workarea
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    fake_recording(str(tmp_path / "r"))
    rid = proj.add_recording(str(tmp_path / "r"))[0]                       # no camera name
    key = proj.camera_key(proj.rec(rid))
    assert key == "default"
    proj.set_camera_settings(key, [[0, 0], [100, 0], [100, 100], [0, 100]], None, [[50, 50, 5]], [640, 480])
    cam = um.pick_camera_name(proj, str(tmp_path / "r"), None)
    a = um.Area.from_project(proj, cam, 20.0)
    ref = workarea.Area.from_settings(proj.camera_settings(key), default_margin=20.0)
    pts = [(10, 10), (50, 52), (110, 50), (130, 50), (300, 300)]
    assert [a.keep(x, y) for x, y in pts] == [ref.keeps(x, y) for x, y in pts] == [True, False, True, False, False]
    # a named camera
    fake_recording(str(tmp_path / "r2"))
    proj.add_recording(str(tmp_path / "r2"), "top")
    proj.set_camera_settings("top", None, 0, [[1, 1, 3]], None)
    assert not um.Area.from_project(proj, "top", 20.0).keep(1, 2)
    assert um.heights_for(projmod.Project(proj.folder))["unknown"] == 0.0


# ---------------------------------------------------------------- the page, in a browser
def test_investigate_time_saved_and_share_in_a_browser(tmp_path, synth10):
    pw = pytest.importorskip("playwright.sync_api")
    proj, rid = ready_project(str(tmp_path / "p"), synth10, name="Browser4")
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    st.open(rid, "none", wait=True)
    server, url = serve(st)
    try:
        with pw.sync_playwright() as p:
            try:
                browser = p.chromium.launch()
            except Exception as e:
                pytest.skip("no Chromium for Playwright: %s" % e)
            page = browser.new_page(viewport={"width": 1400, "height": 900})
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(url)
            page.wait_for_function("typeof S !== 'undefined' && S.frame && S.frame.objects.length > 0", timeout=60000)
            # point at an object and press Q
            pos = page.evaluate("""() => { const o = S.frame.objects[0], b = view.getBoundingClientRect();
                return { x: b.left + (o.x + 0.5) * S.scale, y: b.top + (o.y + 0.5) * S.scale, f: S.frame.frame, cls: o.cls }; }""")
            page.mouse.move(pos["x"], pos["y"])
            page.keyboard.press("q")
            page.wait_for_selector(".verdict", timeout=90000)
            assert page.locator("#invchart").count() == 1 and page.locator(".thumb").count() == 40
            page.wait_for_function("!document.querySelector('#invref') || !/looking/.test(document.querySelector('#invref').textContent)")
            assert "Second camera" in page.inner_text("#invref")             # not available: says why
            assert page.locator("#invlabels").count() == 1
            if SHOTS:
                page.screenshot(path=os.path.join(SHOTS, "investigate.png"))
            page.keyboard.press("Escape")
            assert not page.is_visible(".verdict")
            assert page.is_visible("#invlast")
            # accept the frame, then label at the spot and use "change all" from the investigation
            page.keyboard.press("Enter")
            page.wait_for_function("S.frame && S.frame.frame !== %d" % pos["f"], timeout=30000)
            page.click("#invlast")
            page.wait_for_selector("#invtype", timeout=30000)
            page.click("#invunsure")
            page.wait_for_selector("#invpreview.on", timeout=30000)        # stage 2: a preview first, then confirm
            page.click("#invyes")
            page.wait_for_function("S.stats && S.stats.unsure >= 1", timeout=30000)
            page.keyboard.press("Escape")
            page.wait_for_timeout(300)
            page.keyboard.press("z")                                          # one step back
            page.wait_for_function("S.stats && S.stats.unsure === 0", timeout=30000)
            # Track timeline
            page.click("#tlbtn")
            page.wait_for_selector("#tlsvg")
            if SHOTS:
                page.screenshot(path=os.path.join(SHOTS, "track-timeline.png"))
            page.keyboard.press("Escape")
            assert not page.is_visible("#tlsvg")
            # Time saved panel
            page.click("#smore")                  # under "More" in the studio bar
            page.click("#stime")
            page.wait_for_selector(".tsaved")
            assert "Time saved" in page.inner_text(".tsaved")
            if SHOTS:
                page.screenshot(path=os.path.join(SHOTS, "time-saved.png"))
            page.keyboard.press("Escape")
            # Share project dialog (filled in, then cancelled) and Open shared project
            page.click("#smore")                  # under "More" in the studio bar
            page.click("#sshare")
            assert page.input_value("#shpath").endswith("Browser4.mxirl")
            if SHOTS:
                page.screenshot(path=os.path.join(SHOTS, "share.png"))
            page.keyboard.press("Escape")
            page.click("#smore")                  # under "More" in the studio bar
            page.click("#sopen")
            page.fill("#opfile", "/somewhere/their-project.mxirl")
            assert page.input_value("#opdest").endswith("their-project")
            page.keyboard.press("Escape")
            # the tour knows the new parts (the full tour: the ? button shows the short one, whose last step offers it)
            page.click("#helpbtn")
            seen = set()
            for _ in range(60):
                if not page.is_visible("#tour.on"):
                    break
                seen.add(page.inner_text("#tourcard h4"))
                if page.is_visible("#tfull"):
                    page.click("#tfull")
                    continue
                page.keyboard.press("ArrowRight")
            assert {"Investigate a spot (Q)", "Track timeline", "Time saved", "Share project", "Open shared project"} <= seen
            browser.close()
        assert not errors, errors
        assert len([r for r in timing.read(timesaved.stats_path(proj, rid)) if r["outcome"] != "same"]) == 1
    finally:
        server.shutdown()
        server.server_close()
