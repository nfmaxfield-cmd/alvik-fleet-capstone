"""Tests for the mark-the-robot page (tools/label/mark_robot.py, mark_footage.py). Run from the repo root:

    python -m pytest tools/label/tests/test_mark_robot.py -v

They make a small synthetic recording (an MJPG video.avi + frames.csv, as the Pi recorder writes) with a dark robot
driving across a plain floor, drawn through the real overhead calibration (overhead_floorcal_roam1.json), so every
picture's true floor position is known. About 30 s.
"""
import csv
import io
import json
import math
import os
import sys
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import mark_footage as FT   # noqa: E402
import mark_robot as M      # noqa: E402

T_START = 1792711800.0
FPS = 25.0


def robot_xy(t):
    """The synthetic robot's floor position t seconds into the recording: still for 1 s, then 8 cm/s along x."""
    s = max(0.0, t - 1.0)
    return 35.0 + 8.0 * s, 60.0


def draw_frame(cal, x, y, rnd):
    img = np.full((480, 640, 3), 165, np.uint8)
    img = cv2.add(img, rnd.integers(0, 6, img.shape, dtype=np.uint8))
    h = M.ROBOT_CM / 2
    pts = [cal.to_px(x + dx, y + dy) for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h))]
    cv2.fillPoly(img, [np.array(pts, np.int32)], (45, 45, 45))
    c = cal.to_px(x + 2.0, y)
    cv2.circle(img, (int(c[0]), int(c[1])), 3, (230, 230, 230), -1)
    return img


@pytest.fixture(scope="module")
def recording(tmp_path_factory):
    """A 12 s recording folder: video.avi + frames.csv. Returns (folder, truth {frame: (x, y)})."""
    d = tmp_path_factory.mktemp("rec")
    cal = M.Calib(M.DEFAULT_CALIB)
    rnd = np.random.default_rng(1)
    w = cv2.VideoWriter(str(d / "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), FPS, (640, 480))
    truth = {}
    with open(d / "frames.csv", "w", newline="") as f:
        cw = csv.writer(f)
        cw.writerow(["frame", "t_wall", "t_mono", "cam_ms"])
        for k in range(int(12 * FPS)):
            t = k / FPS
            x, y = robot_xy(t)
            truth[k] = (x, y)
            w.write(draw_frame(cal, x, y, rnd))
            cw.writerow([k, "%.4f" % (T_START + t), "%.4f" % t, ""])
    w.release()
    return str(d), truth


def session(folder, out, guesses=True, **kw):
    import argparse
    return M.session_for_folder(folder, argparse.Namespace(out=str(out), **kw), guesses=guesses)


# ---------------------------------------------------------------------------------------------- footage
def test_avi_index_matches_the_chunks(recording):
    folder, truth = recording
    data = open(os.path.join(folder, "video.avi"), "rb").read()
    idx = FT.avi_index(lambda o, n: data[o:o + n], len(data))
    scan = FT.scan_chunks(data)
    assert idx is not None and len(idx) == len(truth) == len(scan)
    assert idx == scan                                   # the index points at the same JPEGs, in order
    assert all(data[o:o + 2] == b"\xff\xd8" for o, _ in idx)


def test_store_reads_pictures_by_frame_and_time(recording):
    folder, truth = recording
    st = FT.open_store(folder)
    assert isinstance(st, FT.AviStore) and st.frames[0] == 0 and st.frames[-1] == len(truth) - 1
    assert st.index(T_START + 2.0) == 50 and st.nearest(T_START + 2.01) == 50
    g = st.gray(50)
    assert g.shape == (480, 640)
    assert st.jpeg(50)[:2] == b"\xff\xd8"


def test_piece_of_a_split_video(recording, tmp_path):
    """A video cut by bytes (the release's video.part-ab): pictures found by their chunk headers, numbered from
    --first-frame."""
    folder, truth = recording
    data = open(os.path.join(folder, "video.avi"), "rb").read()
    idx = FT.scan_chunks(data)
    cut = idx[99][0] + 200                               # inside picture 99: the first whole one is 100
    p = tmp_path / "video.part-ab"
    p.write_bytes(data[cut:])
    st = FT.AviStore(str(p), os.path.join(folder, "frames.csv"), first_frame=100)
    assert st.frames[0] == 100
    assert st.jpeg(150) == data[idx[150][0]:idx[150][0] + idx[150][1]]


def test_folder_of_pictures(recording, tmp_path):
    folder, truth = recording
    st = FT.open_store(folder)
    for f in (10, 35, 60):
        (tmp_path / ("%06d.jpg" % f)).write_bytes(st.jpeg(f))
    fs = FT.open_store(str(tmp_path), frames_csv=os.path.join(folder, "frames.csv"))
    assert fs.frames == [10, 35, 60] and abs(fs.t(35) - (T_START + 35 / FPS)) < 1e-3


class _RangeServer:
    """Serves local files by Range requests, like GitHub's release downloads (an opener for urllib)."""

    def __init__(self, files):
        self.files = files
        self.requests = 0

    def __call__(self, req):
        self.requests += 1
        name = req.full_url.rsplit("/", 1)[1]
        data = self.files[name]
        rng = req.headers.get("Range")
        if rng:
            a, b = rng.split("=")[1].split("-")
            a, b = int(a), min(int(b), len(data) - 1)
            body, hdr = data[a:b + 1], {"Content-Range": "bytes %d-%d/%d" % (a, b, len(data))}
        else:
            body, hdr = data, {}
        return _Resp(body, dict(hdr, **{"Content-Length": str(len(body))}))


class _Resp(io.BytesIO):
    def __init__(self, body, headers):
        super().__init__(body)
        self.headers = headers

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


def test_fetch_downloads_only_the_stretch_from_parts(recording, tmp_path):
    """fetch(): the video's index from two small reads, then only the stretch's bytes; a video split into two parts
    is read as one file; the empty-board span relative to the start is kept too."""
    folder, truth = recording
    data = open(os.path.join(folder, "video.avi"), "rb").read()
    half = len(data) // 2 + 1234
    srv = _RangeServer({"video.part-aa": data[:half], "video.part-ab": data[half:],
                        "frames.csv": open(os.path.join(folder, "frames.csv"), "rb").read()})
    out = tmp_path / "stretch"
    FT.fetch(str(out), ["http://x/video.part-aa", "http://x/video.part-ab"], "http://x/frames.csv",
             T_START + 4.0, T_START + 6.0, extra_ranges=[("start", 0.0, 0.2)], meta=dict(robot="R"), opener=srv,
             log=lambda s: None)
    st = FT.open_store(str(out))
    assert isinstance(st, FT.JpegStore) and st.meta["robot"] == "R" and st.meta["recording_start_t"] == T_START
    assert st.frames[:6] == [0, 1, 2, 3, 4, 5]                       # the empty-board span
    stretch = [f for f in st.frames if f >= 50]
    assert stretch[0] == 75 and stretch[-1] == 150                    # 1 s before t0 to t1
    full = FT.AviStore(os.path.join(folder, "video.avi"), os.path.join(folder, "frames.csv"))
    for f in (0, 75, 112, 150):
        assert st.jpeg(f) == full.jpeg(f)
    assert os.path.getsize(out / "stretch.jpgs") < 0.4 * len(data)
    assert not os.path.exists(out / "frames.csv")


# ---------------------------------------------------------------------------------------------- calibration
def test_pixels_to_floor_cm_and_back():
    cal = M.Calib(M.DEFAULT_CALIB)
    for x, y in ((10.0, 15.0), (61.0, 61.0), (110.0, 100.0)):
        u, v = cal.to_px(x, y)
        bx, by = cal.to_cm(u, v)
        assert abs(bx - x) < 0.05 and abs(by - y) < 0.05
    assert 18 < cal.ring_px() < 28                     # a robot is about 22 px wide at the board's middle
    g = cal.grid(640, 480)
    assert g["nu"] == 41 and g["nv"] == 31


# ---------------------------------------------------------------------------------------------- marking
def _rows(out):
    with open(os.path.join(out, "marks.csv"), newline="") as f:
        return list(csv.DictReader(f))


def test_marks_are_saved_in_pixels_and_cm(recording, tmp_path):
    folder, truth = recording
    out = tmp_path / "marks"
    s = session(folder, out, guesses=False, t_from="+0:01", t_to="+0:06")
    assert len(s.picks) == 6 and s.picks[0] == 25
    cal = s.calib
    x, y = truth[s.picks[0]]
    u, v = cal.to_px(x, y)
    nxt = s.mark(0, "confirmed", u, v, moved_px=12.0, seconds=2.5)
    assert nxt == 1
    s.mark(1, "not_visible")
    s.mark(2, "skipped")
    s.mark(3, "unsure", u + 3, v)
    rows = _rows(out)
    assert rows[0].keys() >= set(M.COLUMNS)
    r0 = rows[0]
    assert r0["status"] == "corrected"                 # the ring was moved off the guess: saved as corrected
    assert abs(float(r0["x_cm"]) - x) < 0.05 and abs(float(r0["y_cm"]) - y) < 0.05
    assert abs(float(r0["u_px"]) - u) < 0.01 and r0["guess_diff_cm"] != "" and r0["frame"] == str(s.picks[0])
    assert abs(float(r0["t_wall"]) - (T_START + 1.0)) < 0.03
    assert rows[1]["status"] == "not_visible" and rows[1]["u_px"] == "" and rows[1]["x_cm"] == ""
    assert rows[2]["status"] == "skipped" and rows[3]["status"] == "unsure"
    sm = json.load(open(out / "summary.json"))
    assert sm["counts"] == dict(confirmed=0, corrected=1, unsure=1, not_visible=1, skipped=1)
    assert sm["marked"] == 4 and not sm["done"] and sm["calibration"].endswith("overhead_floorcal_roam1.json")
    # the replay experiments read only sure positions
    tr = M.load_truth(str(out))
    assert len(tr) == 1 and abs(tr[0]["x"] - x) < 0.05
    s.close()


def test_confirm_on_the_guess_stays_confirmed(recording, tmp_path):
    folder, truth = recording
    s = session(folder, tmp_path / "m", guesses=False, t_from="+0:01", t_to="+0:04")
    s.mark(0, "confirmed", *s.calib.to_px(*truth[s.picks[0]]))
    g = s.item_info(1)["guess"]
    assert g["source"] == "your last mark"            # no tracker: the ring follows your last mark
    s.mark(1, "confirmed", g["u"], g["v"])
    assert _rows(tmp_path / "m")[1]["status"] == "confirmed"
    s.close()


def test_resume_and_undo(recording, tmp_path):
    folder, truth = recording
    out = tmp_path / "marks"
    s = session(folder, out, guesses=False, t_from="+0:01", t_to="+0:06")
    for k in range(3):
        s.mark(k, "confirmed", *s.calib.to_px(*truth[s.picks[k]]))
    s.mark(1, "corrected", 300.0, 200.0)               # change picture 2 again
    s.close()
    s2 = session(folder, out, guesses=False, t_from="+0:01", t_to="+0:06")
    assert s2.picks == s.picks and len(s2.marks) == 3
    assert s2.state()["next"] == 3                     # carries on at the first unmarked picture
    assert s2.marks[1]["u_px"] == "300.00"
    k = s2.undo_last()                                 # undo survives the restart: picture 2's first mark is back
    assert k == 1 and s2.marks[1]["status"] == "corrected" and s2.marks[1]["u_px"] != "300.00"
    assert s2.undo_last() == 2 and 2 not in s2.marks
    assert s2.state()["next"] == 2
    s2.close()
    s3 = session(folder, out, guesses=False, t_from="+0:05", t_to="+0:09")   # a new range does not reshuffle
    assert s3.picks == s.picks
    s3.close()


def test_repeat_checks_after_the_last_picture(recording, tmp_path):
    folder, truth = recording
    out = tmp_path / "marks"
    s = session(folder, out, guesses=False, t_from="+0:00", t_to="+0:11.5")
    assert len(s.picks) == 12
    for k in range(12):
        s.mark(k, "confirmed", *s.calib.to_px(*truth[s.picks[k]]))
    checks = [i for i in s.items if i["kind"] == "check"]
    assert len(checks) == 3
    k = len(s.picks)
    it = s.item_info(k)
    assert it["kind"] == "check" and it["guess"]["source"] == "repeat check"
    off_cm = math.hypot(*np.subtract(s.calib.to_cm(it["guess"]["u"], it["guess"]["v"]),
                                     s.calib.to_cm(checks[0]["orig_u"], checks[0]["orig_v"])))
    assert 4.5 < off_cm < 7.5                          # the ring starts about 6 cm off
    s.mark(k, "confirmed", checks[0]["orig_u"] + 1.0, checks[0]["orig_v"])
    sm = json.load(open(out / "summary.json"))
    assert sm["repeat_checks"]["n"] == 1 and 0.3 < sm["repeat_checks"]["median"] < 0.6   # 1 px is about 0.43 cm
    assert sm["done"] and len(M.load_truth(str(out))) == 12   # checks are not truth rows
    s.close()


def test_recompute_cm_from_pixels(recording, tmp_path):
    """A from-scratch session may have no calibration yet: marks in pixels only, cm filled in later."""
    folder, truth = recording
    out = tmp_path / "marks"
    s = session(folder, out, guesses=False, no_calib=True, robot_px=22.0, t_from="+0:01", t_to="+0:03")
    assert not s.calib.ok and s.guesser is None
    u, v = M.Calib(M.DEFAULT_CALIB).to_px(*truth[s.picks[0]])
    s.mark(0, "confirmed", u, v)
    assert _rows(out)[0]["x_cm"] == ""
    s.close()
    assert M.recompute_cm(str(out), M.DEFAULT_CALIB) == 1
    r = _rows(out)[0]
    assert abs(float(r["x_cm"]) - truth[s.picks[0]][0]) < 0.05


def test_score_track_against_truth():
    truth = [dict(t=10.0, x=0.0, y=0.0), dict(t=11.0, x=10.0, y=0.0), dict(t=12.0, x=20.0, y=0.0),
             dict(t=13.0, x=30.0, y=0.0)]
    samples = [(10.0, 1.0, 0.0), (11.01, 17.0, 0.0), (12.0, None, None), (13.0, 30.0, 4.0), (14.0, 0.0, 0.0)]
    r = M.score_track(samples, truth)
    assert r["n"] == 4 and r["wrong"] == 1 and r["missing"] == 1 and r["share_wrong"] == 0.25


# ---------------------------------------------------------------------------------------------- the tracker's guesses
def test_tracker_guesses_follow_the_robot_and_restart_at_a_correction(recording, tmp_path):
    folder, truth = recording
    s = session(folder, tmp_path / "m", t_from="+0:00.5", t_to="+0:08.5", at="%g,%g" % truth[12])
    assert s.guesser is not None and s.seed0[1:] == truth[12]
    deadline = time.time() + 120
    while not all(s.item_info(k)["guess"]["ready"] for k in range(len(s.picks))):
        assert time.time() < deadline, "the tracker did not finish"
        time.sleep(0.1)
    errs = []
    for k in range(1, len(s.picks)):
        g = s.item_info(k)["guess"]
        assert g["source"].startswith("tracker")
        gx, gy = s.calib.to_cm(g["u"], g["v"])
        tx, ty = truth[s.picks[k]]
        errs.append(math.hypot(gx - tx, gy - ty))
    assert max(errs) < 3.0, errs
    # a correction (here: far off the tracker) starts the tracker again from that mark
    u, v = s.calib.to_px(*truth[s.picks[0]])
    s.mark(0, "confirmed", u, v)
    u3, v3 = s.calib.to_px(truth[s.picks[3]][0] + 6.0, truth[s.picks[3]][1])
    for k in (1, 2):
        g = s.item_info(k)["guess"]
        s.mark(k, "confirmed", g["u"], g["v"])
    s.mark(3, "corrected", u3, v3)
    assert _rows(tmp_path / "m")[3]["reseed"] == "1"
    assert s.guesser.want[0] == 3 and s.guesser.want[3] == "mark"
    s.close()


# ---------------------------------------------------------------------------------------------- the page
def test_page_has_the_controls_and_loads_nothing_from_the_internet():
    html = open(M.PAGE, encoding="utf-8").read()
    for el in ("bConfirm", "bUnsure", "bHidden", "bSkip", "bUndo", "bPrev", "bNext", "bPlay", "pbar", "cv", "mini",
               "zrow", "cards", "donebox"):
        assert 'id="%s"' % el in html, el
    for key in ('"Enter"', '"ArrowLeft"', '"Backspace"', '"w"', '"n"', '"s"', '"z"', '"p"'):
        assert key in html, key
    for ev in ("pointerdown", "pointermove", "pointerup"):        # mouse and touch
        assert ev in html
    assert 'name="viewport"' in html and "@media (max-width:820px)" in html
    assert "http://" not in html and "https://" not in html and "<link" not in html


def test_server_answers_the_page(recording, tmp_path):
    folder, truth = recording
    s = session(folder, tmp_path / "m", guesses=False, t_from="+0:01", t_to="+0:04")
    app = M.App(session=s)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), M.make_handler(app))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = "http://127.0.0.1:%d" % srv.server_address[1]

    def get(p):
        with urllib.request.urlopen(base + p) as r:
            return r.read()

    def post(p, d):
        req = urllib.request.Request(base + p, json.dumps(d).encode(), {"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as r:
            return json.loads(r.read())
    try:
        assert b"bConfirm" in get("/")
        st = json.loads(get("/api/state"))
        assert st["n_picks"] == 4 and st["robot"] == "the robot" and st["has_cm"]
        it = json.loads(get("/api/item?k=0"))
        assert it["frame"] == s.picks[0] and it["ring_r"] > 5
        assert get("/img/%d.jpg" % it["frame"])[:2] == b"\xff\xd8"
        u, v = s.calib.to_px(*truth[s.picks[0]])
        r = post("/api/mark", dict(k=0, status="confirmed", u=u, v=v, moved_px=3, seconds=1.2))
        assert r["next"] == 1 and r["state"]["marked"] == 1
        between = json.loads(get("/api/between?a=%d&b=%d" % (s.picks[0], s.picks[1])))
        assert 1 <= len(between["frames"]) <= 12 and between["frames"][-1] == s.picks[1]
        assert json.loads(get("/api/calgrid"))["step"] == 16
        r = post("/api/undo", {})
        assert r["k"] == 0 and r["state"]["marked"] == 0
    finally:
        srv.shutdown()
        s.close()


def test_presets_name_the_doe_stretches():
    """The three 2026-10-22 evening stretches of DOE-tracking.md DOE 1 (about 6 minutes in all)."""
    total = 0
    for name in M.PRESETS:
        p, m = M.preset_info(name)
        assert m["video"].endswith(".avi") and m["at"] and m["legs"]
        total += m["end_t"] - m["start_t"]
    assert 5.5 * 60 < total < 6.5 * 60


def test_release_files_are_found_for_both_upload_styles():
    """Pi uploads (video.avi.part-aa ..., frames.csv inside rec-...-small.zip) and laptop uploads (video.part-aa,
    frames.csv)."""
    pi = {"logs-e02-20261024-193526.zip", "rec-20261024-193518_E02-small.zip", "video.avi.part-ab",
          "video.avi.part-aa", "side_video.avi.part-aa"}
    v, fr, z, logs = M.pick_release_files(pi)
    assert v == ["video.avi.part-aa", "video.avi.part-ab"] and fr is None
    assert z == "rec-20261024-193518_E02-small.zip" and logs == "logs-e02-20261024-193526.zip"
    lap = {"events.csv", "frames.csv", "meta.json", "video.part-aa", "video.part-ab", "video.part-ac"}
    v, fr, z, logs = M.pick_release_files(lap)
    assert v == ["video.part-aa", "video.part-ab", "video.part-ac"] and fr == "frames.csv" and z is None and logs is None


def test_legs_from_the_runners_leg_csv(tmp_path):
    p = tmp_path / "leg.csv"
    p.write_text("robot,t_recv,run,id,kind,value,result,ms,x_cm,y_cm,th_deg,info\n"
                 "Right,100.0,-,1,turn,40,queued,0,0,0,0,\nRight,100.1,-,1,turn,40,started,0,0,0,0,\n"
                 "Right,101.5,-,1,turn,40,done,0,0,0,0,\nLeft,102.0,-,2,go,10,queued,0,0,0,0,\n"
                 "Right,103.0,-,3,go,10,queued,0,0,0,0,\nRight,104.0,-,3,go,10,done,0,0,0,0,\n")
    assert M.read_legs(str(p), "Right", offset=-1.6) == [[98.4, 99.9, "turn"], [101.4, 102.4, "go"]]


def test_a_quick_confirm_does_not_restart_the_tracker(recording, tmp_path):
    """Confirming a picture before the tracker has reached it: the mark waits for the tracker's guess and restarts
    it only if they disagree (here they agree), so fast marking never makes it start over and over."""
    folder, truth = recording
    s = session(folder, tmp_path / "m", t_from="+0:00.5", t_to="+0:10.5", at="%g,%g" % truth[12])
    seed = s.guesser.want
    last = len(s.picks) - 1
    s.mark(last, "confirmed", *s.calib.to_px(*truth[s.picks[last]]))
    deadline = time.time() + 120
    while _rows(tmp_path / "m")[0]["reseed"] == "pending" or not s.guesser.done_run:
        assert time.time() < deadline
        s.state()
        time.sleep(0.2)
    s.state()
    r = _rows(tmp_path / "m")[0]
    assert r["reseed"] == "0" and float(r["tracker_diff_cm"]) < 3.0
    assert s.guesser.want == seed
    s.close()
