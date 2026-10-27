"""tools/pi/camera_setup.py and setup_conf.py (2026-10-27): the corner -> calibration maths on a simulated moved
overhead camera, the check picture, the config files the launchers read (and their fallback), the flash check's
measurements and recommendation, the lag onset, camrec's --overhead-exposure, and the page's API on --fake."""
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np
import pytest

PI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(PI))
if PI not in sys.path:
    sys.path.insert(0, PI)

import camera_setup as CS  # noqa: E402
import setup_conf as SC  # noqa: E402


@pytest.fixture
def conf(tmp_path, monkeypatch):
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path / "conf"))
    return tmp_path / "conf"


# ------------------------------------------------------------------ config the launchers read
def test_calib_falls_back_to_roam1_and_says_so(conf):
    path, words = SC.overhead_calib()
    assert path == SC.ROAM1_REL and "roam1" in words and "setup" in words


def test_calib_pointer_used_when_file_exists(conf, tmp_path):
    f = tmp_path / "cal.json"
    f.write_text("{}")
    SC.save_calib_pointer(str(f))
    assert SC.overhead_calib() == (str(f), "Overhead calibration from the camera setup page: %s" % f)


def test_calib_pointer_to_missing_file_falls_back(conf, tmp_path):
    SC.save_calib_pointer(str(tmp_path / "gone.json"))
    path, words = SC.overhead_calib()
    assert path == SC.ROAM1_REL and "not there" in words


def test_exposure_save_read_clear_and_garbage(conf):
    assert SC.overhead_exposure() == (None, {})
    SC.save_exposure(78.4, why="test")
    v, d = SC.overhead_exposure()
    assert v == 78.4 and d["why"] == "test"
    SC.save_exposure(None)
    assert SC.overhead_exposure()[0] is None
    os.makedirs(conf, exist_ok=True)
    (conf / "overhead_exposure.json").write_text("{not json")
    assert SC.overhead_exposure()[0] is None
    (conf / "overhead_exposure.json").write_text('{"exposure": -5}')
    assert SC.overhead_exposure()[0] is None


def test_cli_prints_what_the_shell_scripts_use(conf, tmp_path):
    env = dict(os.environ, MIRLT_CONF_DIR=str(conf))
    run = lambda *a: subprocess.run([sys.executable, os.path.join(PI, "setup_conf.py"), *a], env=env,
                                    capture_output=True, text=True)
    r = run("calib")
    assert r.stdout.strip() == SC.ROAM1_REL and "roam1" in r.stderr
    assert run("exposure").stdout == ""
    SC.save_exposure(60)
    assert run("exposure").stdout.strip() == "60"


# ------------------------------------------------------------------ corners -> calibration
def _moved_frame():
    w = CS.FakeWorld()
    src = CS.FakeSource(w, "overhead", fps=1000)
    ok, f = src.read()
    return f, src.cam


def test_found_corners_are_the_true_ones_in_the_runner_order():
    f, true = _moved_frame()
    corners, why = CS.CAM.find_board_corners(f)
    assert corners is not None, why
    want = CS.board_corners_px(true)          # (0,0), (W,0), (W,L), (0,L): e02.py classic_calibration's order
    err = np.hypot(*(np.asarray(corners) - want).T)
    assert err.max() < 2.0, err


def test_fit_puts_floor_points_where_the_true_camera_does(tmp_path):
    f, true = _moved_frame()
    corners, _ = CS.CAM.find_board_corners(f)
    cam, res = CS.fit(corners, (640, 480), str(tmp_path / "c.json"), height_cm=262)
    assert res["err_cm"] < 0.5 and not res["mirrored"]
    for x, y in ((10, 10), (61, 61), (110, 20), (20, 110), (115, 115)):
        a, b = np.array(cam.project((x, y, 0))), np.array(true.project((x, y, 0)))
        assert np.hypot(*(a - b)) < 2.0, (x, y, a, b)
        fx, fy = cam.to_floor(*true.project((x, y, 0)))
        assert abs(fx - x) < 1.0 and abs(fy - y) < 1.0
    d = json.load(open(tmp_path / "c.json"))
    assert d["name"] == "overhead" and d["studio_calibration"]["spec"]["corners_px"] == corners


def test_grid_picture_draws_the_outline_on_the_board():
    f, true = _moved_frame()
    pic = CS.draw_grid(f, true, CS.board_corners_px(true).tolist())
    u, v = (int(round(c)) for c in true.project((61, 0, 0)))      # mid-point of the x edge
    patch = pic[v - 2:v + 3, u - 2:u + 3].reshape(-1, 3)
    assert any(p[1] > 180 and p[0] < 80 and p[2] < 80 for p in patch)     # green


# ------------------------------------------------------------------ flash check
def _off_on(fraction):
    w = CS.FakeWorld()
    src = CS.FakeSource(w, "overhead", fps=1000)
    src.set_exposure(src.auto * fraction)
    off = src.read()[1]
    w.send("led 1 1 1 1500")
    on = src.read()[1]
    return off, on, CS.board_corners_px(src.cam).tolist()


def test_analyse_finds_three_robots_and_measures_contrast():
    off, on, corners = _off_on(1.0)
    r = CS.analyse_flash(off, on, corners)
    assert len(r["robots"]) == 3
    for b in r["robots"]:
        assert b["light"] > 60 and b["top"] > 80
    assert 150 < r["board"] < 180


def test_no_lights_is_said_plainly():
    off, _, corners = _off_on(1.0)
    r = CS.analyse_flash(off, off, corners)
    assert r["robots"] == [] and "no lights" in r["why"]
    assert CS.recommend([dict(exposure=120, fraction=1.0, result=r)])[0] is None


def test_recommend_picks_the_strongest_light_that_keeps_the_tops():
    rows = []
    for fr in CS.DOE_FRACTIONS:
        off, on, corners = _off_on(fr)
        rows.append(dict(exposure=120 * fr, fraction=fr, result=CS.analyse_flash(off, on, corners)))
    i, words = CS.recommend(rows)
    assert rows[i]["fraction"] == 0.5, words           # 1/4: tops 28 < 32; automatic: lights burnt into the board
    assert all(rows[k]["result"]["weakest_top"] < CS.TOP_MIN for k in (2, 3))


def test_onset_finds_the_first_lit_picture():
    ref = np.full((48, 64), 50, np.uint8)
    times = [k * 0.04 for k in range(20)]
    frames = []
    for t in times:
        f = ref.copy()
        if t >= 0.30:
            f[10:16, 10:16] = 200
        frames.append(f)
    t, rise = CS.onset(times, frames, ref, 0.2)
    assert abs(t - 0.32) < 1e-9 and rise > 100
    assert CS.onset(times, [ref] * 20, ref, 0.2)[0] is None


# ------------------------------------------------------------------ camrec: the overhead's own exposure
def test_camrec_overhead_exposure_only_for_the_overhead(monkeypatch):
    import camrec
    got = {}
    monkeypatch.setattr(camrec, "open_camera", lambda d, *a, **k: object())
    monkeypatch.setattr(camrec, "apply_controls", lambda cap, dev, exp, foc, wb, **k: got.__setitem__(dev, exp) or {})
    monkeypatch.setattr(camrec, "run_session", lambda *a, **k: None)
    camrec.main(["--device", "4,6", "--names", "overhead,side", "--no-lock", "--overhead-exposure", "60",
                 "--no-record", "--port", "0"])
    assert got == {"4": 60.0, "6": None}
    got.clear()
    camrec.main(["--device", "4", "--names", "overhead", "--no-lock", "--no-record", "--port", "0"])
    assert got == {"4": None}


# ------------------------------------------------------------------ the page on --fake
def test_page_api_dry_run(conf, tmp_path):
    w = CS.FakeWorld()
    over = CS.Grabber("overhead", CS.FakeSource(w, "overhead", 25.0))
    over.start()
    S = CS.Setup(over, None, w, str(tmp_path / "logs"), fake=True)
    S.grab()
    from http.server import ThreadingHTTPServer
    srv = ThreadingHTTPServer(("127.0.0.1", 0), CS.make_handler(S))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = "http://127.0.0.1:%d" % srv.server_address[1]

    def post(p, b):
        req = urllib.request.Request(base + p, json.dumps(b).encode(), {"Content-Type": "application/json"})
        return json.load(urllib.request.urlopen(req, timeout=20))
    try:
        st = json.load(urllib.request.urlopen(base + "/api/state", timeout=5))
        assert len(st["corners"]) == 4 and st["fake"]
        assert urllib.request.urlopen(base + "/", timeout=5).read().startswith(b"<!doctype html>")
        st = post("/api/save", {"corners": st["corners"], "height_cm": 262})
        path, words = SC.overhead_calib()
        assert path == st["saved"]["path"] and os.path.exists(path)
        assert json.load(open(path))["camera_setup"]["fake"] is True
        st = post("/api/exposure", {"value": 60})
        assert SC.overhead_exposure()[0] == 60
        post("/api/exposure", {"value": None})
        assert SC.overhead_exposure()[0] is None
    finally:
        srv.shutdown()
        over.stopping = True
