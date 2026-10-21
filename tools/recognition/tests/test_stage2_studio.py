"""Stage 2, interface (lane U), in the studio: "Ignore this spot" is gone while old ignore spots keep
working (shown in Work area, removable, and a saved change can be put back), the manual, and a
click-through in a browser of the new messages, undo/redo, the clip, Investigate's three parts,
the jump back with Q and Reject here."""
import json
import os
import re
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
for p in (REC, os.path.join(TOOLS, "label"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import project as projmod  # noqa: E402
import studio as studiomod  # noqa: E402

README = os.path.join(REC, "README.md")
SHOTS = os.environ.get("STAGE2_SHOTS")          # a folder: the browser test saves its screenshots there


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
    d = tmp_path_factory.mktemp("synth10u")
    subprocess.run([sys.executable, os.path.join(TOOLS, "label", "tests", "synth.py"), str(d / "rec"), "--seconds", "10"],
                   check=True, capture_output=True)
    return str(d / "rec")


def studio_on(tmp_path, rec, spots=None):
    proj = projmod.Project(str(tmp_path / "p"), create_name="U")
    rid = proj.add_recording(rec, "overhead")[0]
    proj.finish_setup("U", [dict(name="bot", has_front=True)], 5)
    if spots is not None:
        proj.set_camera_settings("overhead", None, None, spots, [640, 480], asked=True)
    st = studiomod.Studio(proj, log=lambda *a: None, background=False)
    st.open(rid, "none", wait=True)
    get, post = studiomod.make_routes(st)
    server, url = studiomod.labelapp.serve(st.current, 0, extra_get=get, extra_post=post)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return proj, st, server, url


def test_ignore_this_spot_is_gone_and_old_ignore_spots_keep_working(tmp_path, synth10):
    proj, st, server, url = studio_on(tmp_path, synth10, spots=[[100.0, 100.0, 20.0], [500.0, 300.0, 15.0]])
    try:
        assert not hasattr(st, "ignore_spot")
        with pytest.raises(urllib.error.HTTPError) as e:
            _post(url + "api/studio/ignore_spot", dict(camera="overhead", x=1, y=1, r=5))
        assert e.value.code == 404
        info = _get(url + "api/info")
        assert info["area"]["ignore_spots"] == [[100.0, 100.0, 20.0], [500.0, 300.0, 15.0]]
        assert all(not (abs(d["x"] - 100) < 20 and abs(d["y"] - 100) < 20) for g in st.session.dets if g for d in g)
        # Work area: one old spot removed and saved; Z puts the old settings back through the same route
        r = _post(url + "api/studio/workarea", dict(camera="overhead", work_area=None, margin_px=None,
                                                     ignore_spots=[[500.0, 300.0, 15.0]], image_size=[640, 480]))
        assert r["reopened"]
        wait_open(st)
        assert _get(url + "api/info")["area"]["ignore_spots"] == [[500.0, 300.0, 15.0]]
        _post(url + "api/studio/workarea", dict(camera="overhead", work_area=None, margin_px=None,
                                                ignore_spots=[[100.0, 100.0, 20.0], [500.0, 300.0, 15.0]], image_size=[640, 480]))
        wait_open(st)
        assert len(projmod.Project(proj.folder).camera_settings("overhead")["ignore_spots"]) == 2
    finally:
        server.shutdown()
        server.server_close()


def test_manual_and_studio_text():
    text = open(README, encoding="utf-8").read()
    for words in ("| C |", "| Shift+Z |", "**1. What we found**", "**2. Look around**", "**3. Change things**",
                  "**Reject here**", "press Q\nto go back to it", "Old ignore spots", "rejected_spots",
                  "Every action says what it did"):
        assert words in text, words
    assert "press **Ignore this spot**" not in text and "adds one with its **Ignore this spot**" not in text
    assert "in real life" not in text.lower()
    doc = studiomod.__doc__
    assert "Reject here" in doc and '"Ignore this spot", and' not in doc


# ---------------------------------------------------------------- click-through in a browser
def _toast_has(page, *words, timeout=20000):
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


def test_click_through_messages_undo_redo_clip_investigate(tmp_path, synth10):
    pw = pytest.importorskip("playwright.sync_api")
    proj, st, server, url = studio_on(tmp_path, synth10)
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
            _toast_has(page, "whole picture is looked at", "Work area button (W)")
            page.wait_for_function("S.frame !== null && !!S.img")
            # a new object at a known place, then a drag: each says what and where, and Z
            x, y = 320.0, 240.0
            page.mouse.move(*_canvas_point(page, x, y))
            page.keyboard.press("n")
            msg = _toast_has(page, "Added #", "Z (or Del) takes it back")
            f0 = page.evaluate("S.frame.frame")
            assert "on frame %d" % f0 in msg
            sx, sy = _canvas_point(page, x, y)
            page.mouse.move(sx, sy)
            page.mouse.down()
            page.mouse.move(sx + 20, sy + 12, steps=4)
            page.mouse.up()
            _toast_has(page, "Moved #", "on frame %d" % f0, "Z takes it back")
            _shot(page, "01-moved-message")
            page.keyboard.press("z")
            _toast_has(page, "Undid: moved #", "back as it was", "Shift+Z puts it back")
            _shot(page, "02-undid")
            page.keyboard.press("Shift+Z")
            _toast_has(page, "Redid: moved #", "Z takes it back again")
            page.keyboard.press("Shift+Z")
            _toast_has(page, "Nothing to redo")
            # the clip
            page.keyboard.press("c")
            _toast_has(page, "Playing the video", "C again or Esc stops it")
            page.wait_for_function("CLIP.on && CLIP.frames.length > 5")
            page.wait_for_timeout(450)
            _shot(page, "03-clip")
            page.keyboard.press("Escape")
            _toast_has(page, "Clip stopped", "frame %d" % f0)
            # Investigate the added object: three parts, jump, Q back, Reject here
            ox, oy = page.evaluate("[S.frame.objects[S.sel].x, S.frame.objects[S.sel].y]")
            page.mouse.move(*_canvas_point(page, ox, oy))
            page.keyboard.press("q")
            page.wait_for_selector("#invchange", timeout=120000)
            for part in ("#invfound", "#invlook", "#invchange"):
                assert page.is_visible(part), part
            assert "only" in page.text_content("#invhere") or "remember" in page.text_content("#invhere")
            _shot(page, "04-investigate-three-parts")
            page.locator("#invlook .thumb").nth(3).click()
            msg = _toast_has(page, "Jumped to frame", "press Q to go back to it")
            assert not page.is_visible("#invchange")
            _shot(page, "05-jumped")
            page.keyboard.press("q")
            page.wait_for_selector("#invchange")
            _toast_has(page, "Back to the investigation")
            assert page.evaluate("S.frame.frame") == f0
            page.click("#invhere")
            page.wait_for_selector("#invpreview.on")
            assert "Yes, change it" in page.text_content("#invpreview")
            _shot(page, "06-reject-here-preview")
            page.click("#invno")
            _toast_has(page, "Cancelled: nothing was changed")
            n_spots = len(st.session.rejected_spots())
            page.click("#invhere")
            page.wait_for_selector("#invpreview.on")
            page.click("#invyes")
            msg = _toast_has(page, "on frame %d only" % f0, "Z takes it back")
            assert len(st.session.rejected_spots()) == n_spots + 1
            _shot(page, "07-rejected-here")
            page.keyboard.press("Escape")
            _toast_has(page, "Investigation closed", "Back to the last investigation")
            page.keyboard.press("z")
            _toast_has(page, "Undid: ", "Shift+Z puts it back")
            page.wait_for_function("(n) => true", arg=0)
            assert len(st.session.rejected_spots()) == n_spots
            # accept: what, where, and Z
            page.keyboard.press("Enter")
            _toast_has(page, "Accepted frame %d" % f0, "Z takes the accept back")
            _shot(page, "08-accepted")
            # the keys window lists C and Shift+Z
            page.keyboard.press("k")
            keys = page.text_content("#modal")
            assert "Shift+Z" in keys and "Short clip" in keys
            page.keyboard.press("Escape")
            browser.close()
        assert not errors, errors
    finally:
        server.shutdown()
        server.server_close()
