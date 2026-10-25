"""Mark-the-robot follow-ups (Nick, 2026-10-25 13:03): the camera scale check from things of known size (the robot's
top, later the dock's marker strip), by hand and automatically; the page uploading its own marks to GitHub (with
upload_session.py's token file and code); the ring starting on the runner's position when the tracker isn't
replayed (a moment opened from the live view). No network: GitHub is a fake. Run from the repo root:

    python -m pytest tools/label/tests/test_mark_followups.py -q
"""
import argparse
import csv
import io
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
import zipfile
from http.server import ThreadingHTTPServer

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import mark_footage as FT     # noqa: E402
import mark_robot as M        # noqa: E402
import size_check as SC       # noqa: E402
from test_mark_robot import recording, session, T_START  # noqa: E402,F401  (the shared synthetic recording)

REPO = M.REPO
CAM = M.Calib(M.DEFAULT_CALIB).cam


def render(corners, bright=230, floor=110, extra=None, seed=0):
    """A 640 x 480 picture with a filled quadrilateral, drawn 8 times finer and shrunk (real edges are partly
    covered pixels), plus camera noise. extra: more quadrilaterals [(corners, level)] (a glow, a shadow)."""
    S = 8
    big = np.full((480 * S, 640 * S), floor, np.uint8)
    for cs, lvl in (extra or []):
        cv2.fillPoly(big, [np.round((np.array(cs) + 0.5) * S - 0.5).astype(np.int32)], lvl)
    cv2.fillPoly(big, [np.round((np.array(corners) + 0.5) * S - 0.5).astype(np.int32)], bright)
    img = cv2.resize(big, (640, 480), interpolation=cv2.INTER_AREA)
    return cv2.add(img, np.random.default_rng(seed).integers(0, 6, img.shape, dtype=np.uint8))


# ------------------------------------------------------------------------------------------- known sizes
def test_known_sizes_come_from_the_hardware_files():
    sys.path.insert(0, os.path.join(REPO, "hardware", "dock"))
    import dock_params as D
    assert SC.ROBOT_TOP_W_CM == pytest.approx(D.ROBOT_WIDTH_MM / 10)
    assert SC.ROBOT_TOP_Z_CM == pytest.approx(D.ROBOT_TOP_Z_MM / 10)
    src = open(os.path.join(REPO, "hardware", "dock", "build_dock_v5.py")).read()
    assert "STRIP_N = (6, 2)" in src and "STRIP_CELL_MM = 15.0" in src           # 90 x 30 mm
    assert SC.STRIP_LONG_CM == 9.0 and SC.STRIP_SHORT_CM == 3.0
    assert "WALL_H_MM = D.WALL_H_MM" in src and D.WALL_H_MM == pytest.approx(SC.STRIP_Z_CM * 10)
    assert SC.known("dock_strip", strip_mm=90.2)["long_cm"] == pytest.approx(9.02)   # the measured print


# ------------------------------------------------------------------------------------------- measuring
@pytest.mark.parametrize("obj", ["robot", "dock_strip"])
def test_the_calibrations_own_prediction_measures_the_real_size(obj):
    for x, y, h in ((15, 15, 0), (61, 61, 30), (105, 110, 75), (20, 100, 135)):
        m = SC.measure(CAM, SC.predicted_corners(CAM, x, y, obj, h), obj)
        assert abs(m["scale_pct"]) < 0.01 and m["corner_dev_deg"] < 0.05
        assert m["long_cm"] == pytest.approx(SC.KNOWN[obj]["long_cm"], abs=1e-3)
        assert (m["x"], m["y"]) == pytest.approx((x, y), abs=0.01)


def test_a_wrong_scale_shows_and_so_does_its_axis():
    c = np.array(SC.predicted_corners(CAM, 60, 60, "robot", 0))
    big = c.mean(axis=0) + 1.03 * (c - c.mean(axis=0))
    m = SC.measure(CAM, big)
    assert m["scale_pct"] == pytest.approx(3.0, abs=0.4)
    stretched = c.copy()            # stretch the picture one way only: one axis's sides grow, the other's don't
    stretched[:, 0] = c[:, 0].mean() + 1.05 * (c[:, 0] - c[:, 0].mean())
    m = SC.measure(CAM, stretched)
    along = sorted([m["scale_x_pct"], m["scale_y_pct"]])
    assert along[0] == pytest.approx(0.0, abs=1.0) and along[1] > 3.0


def test_summary_says_ok_off_unsure_and_finds_a_tilt():
    rng = np.random.default_rng(2)

    def fits(f_scale, n=27):
        out = []
        for i in range(n):
            x = 122.15 * ((i % 3) + rng.uniform(0.1, 0.9)) / 3
            y = 121.9 * (((i // 3) % 3) + rng.uniform(0.1, 0.9)) / 3
            s = f_scale(x, y) + rng.normal(0, 0.5)
            out.append(dict(x=x, y=y, scale_pct=s, scale_x_pct=s, scale_y_pct=None,
                            region=list(SC.region_of(x, y))))
        return out
    ok = SC.summarize(fits(lambda x, y: 0.2))
    assert ok["verdict"] == "ok" and ok["spread"] == 9 and all(r["n"] == 3 for r in ok["regions"])
    off = SC.summarize(fits(lambda x, y: 4.0))
    assert off["verdict"] == "off" and "off by more" in SC.plain(off)
    assert SC.summarize(fits(lambda x, y: 0.0, n=1))["verdict"] == "unsure"
    tilt = SC.summarize(fits(lambda x, y: 4.0 * (x - 61) / 122))        # +-2 % from one side to the other
    assert tilt["trend"]["across_x_pct"] == pytest.approx(4.0, abs=0.8)
    assert abs(tilt["trend"]["across_y_pct"]) < 0.8
    assert SC.plain(SC.summarize([])) == "No size fits yet."


def test_one_hand_fit_is_good_to_about_one_and_a_half_percent():
    """What the page tells Nick: corners placed to half a pixel (sd) give a fit's scale to about 1.5 % (sd)."""
    rng = np.random.default_rng(3)
    v = []
    for _ in range(300):
        x, y = rng.uniform(10, 110, 2)
        c = np.array(SC.predicted_corners(CAM, x, y, "robot", rng.uniform(0, 90)))
        v.append(SC.measure(CAM, c + rng.normal(0, 0.5, c.shape))["scale_pct"])
    assert 1.0 < np.std(v) < 2.0


# ------------------------------------------------------------------------------------------- automatic outline
@pytest.mark.parametrize("bright", [230, 40])
def test_the_automatic_outline_measures_a_robot(bright):
    for i, (x, y, h) in enumerate(((20, 20, 0), (61, 61, 30), (100, 110, 75), (40, 90, 10))):
        c = SC.predicted_corners(CAM, x, y, "robot", h)
        a = SC.auto_outline(render(c, bright, seed=i), CAM, x + 1.0, y - 1.0, "robot")
        assert a is not None and abs(a["scale_pct"]) < 1.5, (x, y, a and a["scale_pct"])
    assert SC.auto_outline(np.full((480, 640), 110, np.uint8), CAM, 60, 60) is None        # nothing there


def test_glow_beside_the_robot_pads_the_automatic_outline_and_hand_fits_measure_it():
    pairs = []
    for i, (x, y, h) in enumerate(((20, 20, 0), (61, 61, 30), (100, 110, 75), (40, 90, 10), (90, 30, 50))):
        c = SC.predicted_corners(CAM, x, y, "robot", h)
        glow = SC.predicted_corners(CAM, x + 2.0, y, "robot", h)     # a bright patch sticking out 2 cm one side
        a = SC.auto_outline(render(c, 230, extra=[(glow, 225)], seed=i), CAM, x, y, "robot")
        hand = SC.measure(CAM, c)
        pairs.append((hand, a))
        assert a["scale_pct"] > 5.0
    pad = SC.auto_padding(pairs)
    assert pad["n"] == 5 and pad["median_pct"] > 5.0


# ------------------------------------------------------------------------------------------- in the page's session
def test_size_fits_are_saved_summarised_suggested_and_reloaded(recording, tmp_path):
    folder, truth = recording
    s = session(folder, tmp_path / "m", guesses=False, t_from="+0:01", t_to="+0:08")
    try:
        for k in range(4):
            u, v = s.calib.to_px(*truth[s.picks[k]])
            s.mark(k, "confirmed", u, v)
        sug = s.size_suggest()
        assert sug["k"] in range(4) and sug["fits_there"] == 0
        k = sug["k"]
        u, v = s.calib.to_px(*truth[s.picks[k]])
        st = s.size_start(k, "robot", u, v)
        assert st["source"] in ("the automatic outline", "the calibration's prediction of its size")
        assert len(st["corners"]) == 4 and st["fit"]["scale_pct"] is not None
        pred = SC.predicted_corners(s.calib.cam, *truth[s.picks[k]], "robot", 0)
        r = s.size_save(k, "robot", pred)
        assert abs(r["fit"]["scale_pct"]) < 0.01 and r["size"]["robot"]["n"] == 1
        assert s.size_suggest()["k"] != k                                  # the next one is another picture
        assert s.size_start(k, "robot", u, v)["source"] == "your fit"
        assert s.size_start(k, "robot", u, v, fresh=True)["source"] != "your fit"
        s.size_auto_run()
        for _ in range(200):
            if s.size_job["done"]:
                break
            time.sleep(0.05)
        assert s.size_job["error"] is None and s.size_job["of"] == 4
        st = s.size_state()
        assert st["n_auto"] >= 3 and "auto" in st and st["auto"]["summary"]["n"] == st["n_auto"]
        sm = json.load(open(tmp_path / "m" / "summary.json"))
        assert sm["size_check"]["robot"]["n"] == 1 and "robot_plain" in sm["size_check"]
        rows = list(csv.DictReader(open(tmp_path / "m" / "size_fits.csv")))
        assert len(rows) == 1 and rows[0]["how"] == "hand" and rows[0]["obj"] == "robot"
        assert os.path.exists(tmp_path / "m" / "size_auto.csv")
    finally:
        s.close()
    s2 = session(folder, tmp_path / "m", guesses=False, t_from="+0:01", t_to="+0:08")
    try:
        assert len(s2.size_fits) == 1 and s2.size_state()["n_auto"] >= 3
        assert s2.size_delete(k, "robot")["robot"]["n"] == 0
    finally:
        s2.close()


def test_size_endpoints_answer(recording, tmp_path):
    folder, truth = recording
    s = session(folder, tmp_path / "m", guesses=False, t_from="+0:01", t_to="+0:03")
    app = M.App(session=s, auto_upload=False)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), M.make_handler(app))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = "http://127.0.0.1:%d" % srv.server_address[1]

    def post(p, d):
        req = urllib.request.Request(base + p, json.dumps(d).encode(), {"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as r:
            return json.loads(r.read())
    try:
        u, v = s.calib.to_px(*truth[s.picks[0]])
        st = post("/api/size_start", dict(k=0, obj="robot", u=u, v=v))
        ev = post("/api/size_eval", dict(obj="robot", corners=st["corners"]))
        assert ev["fit"]["long_cm"] > 8
        sv = post("/api/size_save", dict(k=0, obj="dock_strip", corners=SC.predicted_corners(
            s.calib.cam, *truth[s.picks[0]], "dock_strip", 20)))
        assert sv["size"]["dock_strip"]["n"] == 1 and abs(sv["fit"]["scale_pct"]) < 0.01
        with urllib.request.urlopen(base + "/api/state") as r:
            state = json.loads(r.read())
        assert state["size"]["robot"]["n"] == 0 and state["upload"]["state"] == "never"
    finally:
        srv.shutdown()
        s.close()


def test_page_has_the_size_check_and_upload_controls():
    html = open(M.PAGE, encoding="utf-8").read()
    for el in ("sizebox", "szObj", "bSzStart", "bSzSave", "bSzReset", "bSzDel", "bSzDone", "bSzNext", "bSzAuto",
               "szGrid", "upbox", "upst", "bUpload"):
        assert 'id="%s"' % el in html, el
    for api in ("/api/size_start", "/api/size_eval", "/api/size_save", "/api/size_delete", "/api/size_suggest",
                "/api/size_auto", "/api/upload"):
        assert api in html, api
    assert "http://" not in html and "https://" not in html and "<link" not in html


# ------------------------------------------------------------------------------------------- uploading
class FakeGitHub:
    """Answers upload_session.api's requests like GitHub's release API would; records them."""

    def __init__(self, has_release=False, assets=(), refuse=None):
        self.calls = []
        self.has_release = has_release
        self.assets = list(assets)
        self.uploaded = {}
        self.refuse = refuse

    def __call__(self, req, timeout=None):
        m, url = req.get_method(), req.full_url
        self.calls.append((m, url))
        if self.refuse:
            raise urllib.error.HTTPError(url, self.refuse, "no", {}, io.BytesIO(b""))
        if m == "GET" and url.endswith("/releases/tags/" + M.UPLOAD_TAG):
            if not self.has_release:
                raise urllib.error.HTTPError(url, 404, "not found", {}, io.BytesIO(b""))
            return self._ans(self._rel())
        if m == "GET" and url.endswith("/repos/" + M.REPO_SLUG):
            return self._ans(dict(permissions=dict(push=True)))
        if m == "POST" and url.endswith("/releases"):
            self.has_release = True
            return self._ans(self._rel())
        if m == "DELETE":
            self.assets = [a for a in self.assets if a["url"] != url]
            return self._ans(None)
        if m == "POST" and "uploads.github.com" in url:
            data = req.data.read(1 << 30) if hasattr(req.data, "read") else req.data
            self.uploaded[url.split("name=")[1]] = data
            return self._ans(dict(name=url.split("name=")[1]))
        raise AssertionError((m, url))

    def _rel(self):
        return dict(upload_url="https://uploads.github.com/repos/x/releases/1/assets{?name,label}",
                    assets=[dict(a) for a in self.assets])

    @staticmethod
    def _ans(d):
        class R(io.BytesIO):
            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False
        return R(b"" if d is None else json.dumps(d).encode())


@pytest.fixture
def fake_github(monkeypatch):
    UP = M._uploader()

    def make(**kw):
        g = FakeGitHub(**kw)
        monkeypatch.setattr(UP, "_open", g)
        return g
    return make


def _marks_folder(tmp_path):
    out = tmp_path / "marks" / "oct22eve_left"
    out.mkdir(parents=True)
    (out / "marks.csv").write_text("item,kind\n0,pick\n")
    (out / "summary.json").write_text("{}")
    (out / "state.json").write_text("{}")             # not sent: the page's own bookkeeping
    return str(out)


def test_upload_creates_the_release_and_sends_one_zip(fake_github, tmp_path):
    g = fake_github()
    r = M.upload_marks(_marks_folder(tmp_path), "oct22eve_left", tok="t0k", log=lambda *a: None)
    assert r["asset"] == "marks-oct22eve_left.zip" and M.UPLOAD_TAG in r["url"]
    assert ("POST", "https://api.github.com/repos/%s/releases" % M.REPO_SLUG) in g.calls
    z = zipfile.ZipFile(io.BytesIO(g.uploaded["marks-oct22eve_left.zip"]))
    assert sorted(os.path.basename(n) for n in z.namelist()) == ["marks.csv", "summary.json"]


def test_upload_replaces_an_older_zip_of_the_same_name(fake_github, tmp_path):
    old = dict(name="marks-oct22eve_left.zip", url="https://api.github.com/repos/x/releases/assets/7")
    g = fake_github(has_release=True, assets=[old])
    M.upload_marks(_marks_folder(tmp_path), "oct22eve_left", tok="t0k", log=lambda *a: None)
    assert ("DELETE", old["url"]) in g.calls
    up = next(i for i, c in enumerate(g.calls) if "uploads.github.com" in c[1])
    assert g.calls.index(("DELETE", old["url"])) < up
    assert "marks-oct22eve_left.zip" in g.uploaded


def test_upload_without_a_token_says_so(monkeypatch, tmp_path):
    UP = M._uploader()
    monkeypatch.setattr(UP, "token", lambda: None)
    with pytest.raises(M.NoToken):
        M.upload_marks(_marks_folder(tmp_path), "x")


def test_the_page_uploads_by_itself_once_every_picture_is_marked(recording, tmp_path, monkeypatch):
    folder, truth = recording
    monkeypatch.setattr(M, "UPLOAD_WAIT_S", 0.05)
    s = session(folder, tmp_path / "m", guesses=False, t_from="+0:01", t_to="+0:03", checks=0)
    app = M.App(session=s)
    sent = []
    app.uploader = lambda out, name: sent.append((out, name)) or dict(asset=M.marks_asset_name(name), url="u")
    try:
        for k in range(len(s.picks)):
            u, v = s.calib.to_px(*truth[s.picks[k]])
            s.mark(k, "confirmed", u, v)
            if k < len(s.picks) - 1:
                time.sleep(0.15)
                assert not sent                     # not before every picture is marked
        for _ in range(100):
            if (s.upload_status or {}).get("state") == "up":
                break
            time.sleep(0.05)
        assert s.upload_status["state"] == "up" and len(sent) == 1 and sent[0][0] == s.out
        s.undo_last()                               # a change after the upload: it goes up again
        for _ in range(100):
            if len(sent) == 2 and s.upload_status["state"] == "up":
                break
            time.sleep(0.05)
        assert len(sent) == 2 and s.state()["upload"]["seq"] == s.changed_seq
    finally:
        s.close()


def test_the_page_says_when_there_is_no_token(recording, tmp_path, monkeypatch):
    folder, truth = recording
    s = session(folder, tmp_path / "m", guesses=False, t_from="+0:01", t_to="+0:02", checks=0)
    app = M.App(session=s)

    def no_token(out, name):
        raise M.NoToken("no GitHub token at ~/.config/mirlt/github_token")
    app.uploader = no_token
    try:
        app.kick_upload(s, now=True)
        for _ in range(100):
            if (s.upload_status or {}).get("state") == "no_token":
                break
            time.sleep(0.05)
        assert s.state()["upload"]["state"] == "no_token"
    finally:
        s.close()


def test_check_token_says_whether_the_saved_token_works(monkeypatch, capsys):
    UP = M._uploader()
    monkeypatch.setattr(UP, "token", lambda: "t0k")
    monkeypatch.setattr(UP, "_open", FakeGitHub())
    UP.check_token()
    assert "Token works" in capsys.readouterr().out
    monkeypatch.setattr(UP, "_open", FakeGitHub(refuse=401))
    with pytest.raises(SystemExit) as e:
        UP.check_token()
    assert "refused" in str(e.value) and "--set-token" in str(e.value)
    monkeypatch.setattr(UP, "token", lambda: None)
    with pytest.raises(SystemExit) as e:
        UP.check_token()
    assert "No token saved" in str(e.value)


# ------------------------------------------------------------------------------------------- a moment's folder
def test_a_moment_folder_opens_with_the_runners_position_as_the_start(recording, tmp_path):
    """What the live view's "Mark this moment" saves: pictures named by the recorder's frame numbers, frames.csv
    with the Pi times, stretch.json; the session's track.csv on the laptop clock (Pi time - offset)."""
    folder, truth = recording
    store = FT.open_store(folder)
    mom = tmp_path / "moment"
    mom.mkdir()
    frames = [f for f in store.frames if 100 <= f <= 150]
    off = 2.5                                         # laptop time + 2.5 s = Pi time
    with open(mom / "frames.csv", "w") as f:
        f.write("frame,t_wall,t_laptop\n")
        for fr in frames:
            (mom / ("%06d.jpg" % fr)).write_bytes(store.jpeg(fr))
            f.write("%d,%.4f,%.4f\n" % (fr, store.t(fr), store.t(fr) - off))
    (mom / "stretch.json").write_text(json.dumps(dict(name="moment-test", robot="Left", title="Left, a moment",
                                                      start_t=store.t(frames[0]), end_t=store.t(frames[-1]))))
    sd = tmp_path / "sess"
    sd.mkdir()
    with open(sd / "track.csv", "w") as f:
        f.write("t,robot,x_cm,y_cm\n")
        for fr in frames:
            x, y = truth[fr]
            f.write("%.4f,Left,%.2f,%.2f\n" % (store.t(fr) - off, x + 3.0, y))     # the runner 3 cm off
    a = argparse.Namespace(out=str(mom / "marks"), robot="Left", session=str(sd), clock_offset=off, no_guesses=True,
                           every=0.5, calib=M.DEFAULT_CALIB)
    s = M.session_for_folder(str(mom), a)
    try:
        assert s.guesser is None and s.title == "Left, a moment" and s.picks[0] in frames
        it = s.item_info(0)
        assert it["guess"]["source"] == "where the runner thought it was"
        gx, gy = s.calib.to_cm(it["guess"]["u"], it["guess"]["v"])
        tx, ty = truth[s.picks[0]]
        assert abs(gx - (tx + 3.0)) < 0.3 and abs(gy - ty) < 0.3
        assert it["runner"] is not None
        u, v = s.calib.to_px(tx, ty)
        s.mark(0, "corrected", u, v)
        row = s.marks[0]
        assert float(row["runner_diff_cm"]) == pytest.approx(3.0, abs=0.3)
    finally:
        s.close()
