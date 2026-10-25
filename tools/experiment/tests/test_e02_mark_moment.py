"""The live view's "Mark this moment" button (Nick, 2026-10-25): it saves the last pictures the runner kept, named by
the recorder's frame numbers, into <session>/moments/, starts the mark-the-robot page on them and answers with its
address; the runner itself is not touched. A fake runner with a feed that kept 30 s of synthetic pictures; the mark
page is started for real (a separate Python process) and asked for its state."""
import copy
import json
import os
import sys
import time
import types
import urllib.request
from collections import deque

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
sys.path.insert(0, EXP)
sys.path.insert(0, os.path.join(os.path.dirname(EXP), "label"))

import e02_camera as CAM  # noqa: E402
import e02_view as V      # noqa: E402
import mark_robot as M    # noqa: E402

PI_START = 1792711800.0
OFFSET = -1.75            # Pi time - laptop time (the Pi's clock runs behind here)


def fake_runner(folder, now):
    """A runner whose feed kept 30 s of pictures (25 a second, every second one: the tracker skipped some), Left
    followed, driving along x; track.csv and e02_state.json as the runner writes them."""
    cal = M.Calib(M.DEFAULT_CALIB)
    feed = CAM.Feed.__new__(CAM.Feed)            # only what kept() needs: the ring and its lock
    feed.cond = __import__("threading").Condition()
    feed.keep_s = 150.0
    feed.ring = deque()
    os.makedirs(folder, exist_ok=True)
    track = ["t,robot,x_cm,y_cm"]
    for k in range(0, 750, 2):
        t_lap = now - 30.0 + k / 25.0
        x, y = 30.0 + 2.0 * k / 25.0, 60.0
        img = np.full((480, 640), 110, np.uint8)
        pts = [cal.to_px(x + dx, y + dy) for dx, dy in ((-4.8, -4.8), (4.8, -4.8), (4.8, 4.8), (-4.8, 4.8))]
        cv2.fillPoly(img, [np.array(pts, np.int32)], 230)
        jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]
        feed.ring.append((t_lap, jpg, 5000 + k, t_lap + OFFSET))
        track.append("%.4f,Left,%.2f,%.2f" % (t_lap, x, y))
    with open(os.path.join(folder, "track.csv"), "w") as f:
        f.write("\n".join(track) + "\n")
    with open(os.path.join(folder, "e02_state.json"), "w") as f:
        json.dump(dict(calibration=M.DEFAULT_CALIB, robots=["Left", "Center"]), f)
    return types.SimpleNamespace(feed=feed, dir=folder, tracker=types.SimpleNamespace(active="Left"),
                                 st=dict(robots=["Left", "Center"]), phase="route", order=3)


def test_the_page_has_the_button():
    assert 'id="mmb"' in V.PAGE and "/mark-moment" in V.PAGE and "window.open" in V.PAGE


def test_kept_pictures_carry_the_recorders_frame_number():
    feed = CAM.Feed.__new__(CAM.Feed)
    feed.cond = __import__("threading").Condition()
    feed.ring = deque([(10.0, b"a", 7, 8.5), (11.0, b"b", 8, 9.5), (12.0, b"c", 9, 10.5)])
    assert [q[2] for q in feed.kept(10.5)] == [8, 9] and feed.kept(10.5, 11.5)[0][3] == 9.5


def test_mark_this_moment_saves_the_pictures_and_starts_the_mark_page(tmp_path):
    now = time.time()
    r = fake_runner(str(tmp_path / "e02-20261025-140000"), now)
    before = copy.deepcopy(r.st)
    view = V.LiveView(r, port=0)
    msg = view.start()
    port = view.srv.server_address[1]
    procs = []
    try:
        req = urllib.request.Request("http://127.0.0.1:%d/mark-moment" % port, b"{}",
                                     {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=60) as resp:
            d = json.loads(resp.read())
        procs = view.__dict__.get("_moment_procs", [])
        assert d["ok"], d
        assert d["robot"] == "Left" and d["frame"] == 5000 + 748 and d["url"] == "http://HOST:%d" % d["port"]
        assert 220 <= d["n"] <= 252                                # the last 20 s, every second picture
        assert "--release e02-session-20261025-140000 --robot Left" in d["later"]
        folder = d["folder"]
        assert folder.startswith(os.path.join(r.dir, "moments"))
        names = sorted(n for n in os.listdir(folder) if n.endswith(".jpg"))
        assert names[-1] == "005748.jpg" and len(names) == d["n"]
        meta = json.load(open(os.path.join(folder, "stretch.json")))
        assert meta["recorder_frames"][1] == 5748 and abs(meta["clock_offset_s"] - OFFSET) < 1e-3
        # the mark page answers, on these pictures, with the runner's position as the ring's start
        with urllib.request.urlopen("http://127.0.0.1:%d/api/state" % d["port"], timeout=30) as resp:
            st = json.loads(resp.read())
        assert st["robot"] == "Left" and st["has_cm"] and not st["guesses"] and st["n_picks"] >= 35
        with urllib.request.urlopen("http://127.0.0.1:%d/api/item?k=0" % d["port"], timeout=30) as resp:
            it = json.loads(resp.read())
        assert it["guess"]["source"] == "where the runner thought it was" and it["runner"] is not None
        assert it["frame"] >= 5000 and it["frame"] % 2 == 0      # the recorder's numbers
        assert r.st == before                                      # the runner was only read
        assert any("Moment marked" in ln for ln in view.lines)
    finally:
        for p in procs:
            p.terminate()
        view.stop()


def test_without_kept_pictures_it_says_why(tmp_path):
    r = types.SimpleNamespace(feed=None, dir=str(tmp_path), tracker=None, st={})
    d = V.LiveView(r, port=0).mark_moment()
    assert not d["ok"] and "keeps no pictures" in d["error"]
