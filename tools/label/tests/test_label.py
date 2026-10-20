"""Tests for the labelling tools. Run from the repo root:

    python -m pytest tools/label/tests -v

They make two synthetic recordings (about a minute) in a temporary folder.
The page screenshot test runs only if Playwright for Python is installed.
"""
import csv
import json
import math
import os
import shutil
import sys
import threading
import urllib.error
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import motion          # noqa: E402
import tracker         # noqa: E402
import select_frames   # noqa: E402
import app             # noqa: E402
import evaluate        # noqa: E402

# tools/recognition/tests has a synth.py too: load this folder's one by its path, so both
# suites can run in one pytest call
import importlib.util  # noqa: E402
_spec = importlib.util.spec_from_file_location("label_synth", os.path.join(HERE, "synth.py"))
synth = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(synth)


# ----------------------------------------------------------------------------
# Synthetic recordings (made once per test run)
# ----------------------------------------------------------------------------

@pytest.fixture(scope="session")
def recs(tmp_path_factory):
    base = tmp_path_factory.mktemp("synth")
    out = {}
    for name in ("main", "crowd"):
        out[name] = synth.make_recording(str(base / name), name)
    return out


@pytest.fixture(scope="session")
def results(recs):
    return {name: evaluate.evaluate(folder) for name, folder in recs.items()}


def fresh_copy(src, dst, keep_cache_from=None):
    """Copy a recording without labels (optionally reusing a detection cache)."""
    os.makedirs(dst, exist_ok=True)
    for n in ("video.avi", "frames.csv", "events.csv", "meta.json", "truth.csv"):
        shutil.copy(os.path.join(src, n), dst)
    if keep_cache_from:
        shutil.copytree(os.path.join(keep_cache_from, "labels", "cache"), os.path.join(dst, "labels", "cache"))
    # keep the video's modification time so the cache key still matches
    st = os.stat(os.path.join(src, "video.avi"))
    os.utime(os.path.join(dst, "video.avi"), (st.st_atime, st.st_mtime))
    return dst


# ----------------------------------------------------------------------------
# Tracker rules, on hand-made detections
# ----------------------------------------------------------------------------

R = 30.0


def det(x, y, cls="alvik", score=0.9, angle=0.0):
    return dict(cls=cls, x=x, y=y, heading_deg=None, score=score, angle90=angle)


def run(frames, anchors=None, fps=30.0):
    times = [f / fps for f in range(len(frames))]
    return tracker.run_tracker(frames, times, R, anchors=anchors)


def test_far_lookalike_does_not_steal_track():
    # A robot drives right; at frame 30 it is not detected but a look-alike
    # appears 200 px away. The look-alike must get its own number.
    frames = [[det(100 + 2 * f, 200)] for f in range(30)]
    frames += [[det(300, 400)] for f in range(30, 60)]
    rows, summary = run(frames)
    ids_late = {r["obj"]: (r["x"], r["y"]) for r in rows[59]}
    assert len(ids_late) == 2
    first = rows[29][0]["obj"]
    assert ids_late[first] == pytest.approx((158.0, 200.0), abs=1.0)   # stayed where it was last seen


def test_new_track_needs_confirmation():
    frames = [[] for _ in range(20)]
    for f in (5, 6, 7):                       # 3 frames only: noise
        frames[f] = [det(300, 300)]
    rows, _ = run(frames)
    assert not any(rows.get(f) for f in range(20))
    frames = [[det(300 + f, 300)] if f >= 5 else [] for f in range(20)]
    rows, _ = run(frames)
    assert rows[5] and rows[19]               # confirmed after 5, earlier frames filled in


def test_stop_keeps_position_and_heading():
    # Drive up (heading 90) for 2 s, then disappear (stopped and blended in) for 10 s.
    frames = [[det(200, 300 - 2 * f)] for f in range(60)] + [[] for _ in range(300)]
    rows, _ = run(frames)
    last = rows[359][0]
    assert (last["x"], last["y"]) == pytest.approx((200, 182), abs=1.0)
    assert last["heading_known"] and abs(motion.wrap180(last["heading_deg"] - 90)) < 5
    assert last["source"] == "track" and last["score"] < rows[59][0]["score"]


def test_short_lived_track_that_vanishes_ends_soon():
    # Seen for half a second, then never again: noise, not a parked robot.
    frames = [[det(200 + 2 * f, 300)] for f in range(15)] + [[] for _ in range(300)]
    rows, _ = run(frames)
    assert rows.get(14) and not rows.get(100)


def test_reversing_heading_is_a_guess():
    # Forward (right) 1 s, stop 0.5 s, then back up (moving left) 1 s with the body not turning.
    frames = [[det(100 + 2 * f, 200)] for f in range(30)]
    frames += [[det(158, 200)] for _ in range(15)]
    frames += [[det(158 - 1.5 * k, 200)] for k in range(1, 31)]
    rows, _ = run(frames)
    r = rows[70][0]
    assert not r["heading_known"]
    assert r["heading_guess"] is not None and abs(motion.wrap180(r["heading_guess"] - 0)) < 15


def test_turn_on_the_spot_is_a_guess_that_follows_the_body():
    frames = [[det(100 + 2 * f, 200, angle=0.0)] for f in range(30)]
    # stopped, body turning 3 deg per frame (on-screen counter-clockwise) for 20 frames = 60 deg
    frames += [[det(158, 200, angle=(3.0 * k) % 90.0)] for k in range(1, 21)]
    rows, _ = run(frames)
    r = rows[49][0]
    assert not r["heading_known"]
    assert abs(motion.wrap180(r["heading_guess"] - 60)) < 8


def test_user_anchor_seeds_a_track_for_a_robot_that_is_not_moving():
    frames = [[] for _ in range(60)] + [[det(300 + 2 * k, 300)] for k in range(60)]
    seed = {0: [dict(obj=7, cls="alvik", x=300.0, y=300.0, heading_deg=0.0, heading_known=True, status="corrected")]}
    rows, _ = run(frames, anchors=seed)
    assert [r["obj"] for r in rows[30]] == [7]                    # followed while it sits still
    assert [r["obj"] for r in rows[119]] == [7]                   # and once it drives off
    assert rows[119][0]["x"] == pytest.approx(418, abs=1)


def test_rejected_spot_stays_rejected():
    frames = [[det(400, 100)] for _ in range(90)]                 # a stain that never moves
    rows, _ = run(frames)
    oid = rows[10][0]["obj"]
    rej = {10: [dict(obj=oid, cls="alvik", x=400.0, y=100.0, heading_deg=None, status="rejected")]}
    rows, _ = run(frames, anchors=rej)
    assert not any(rows.get(f) for f in range(11, 90))


def test_partial_retrack_equals_full_run(results, recs):
    dets, info = motion.detect_recording(os.path.join(recs["main"], "video.avi"), n_frames=1200)
    times = [f / 30.0 for f in range(len(dets))]
    tr = tracker.TrackRun(dets, times, info["robot_px"])
    rows0, _ = tr.run()
    objs = [dict(r) for r in rows0[600]]
    objs[0]["x"] += 6
    anchors = {600: objs}
    part, _ = tr.run(anchors, from_frame=600)
    full, _ = tracker.run_tracker(dets, times, info["robot_px"], anchors=anchors)
    assert json.dumps(part, sort_keys=True) == json.dumps(full, sort_keys=True)


# ----------------------------------------------------------------------------
# Accuracy on synthetic recordings
# ----------------------------------------------------------------------------

def test_accuracy_main(results):
    t = results["main"]["tracked"]
    print("\nmain:", evaluate.fmt(results["main"]))
    assert t["detection_rate"] >= 0.99
    assert t["false_per_100_frames"] <= 3
    assert t["centre_err_p95"] <= 4.0
    assert t["heading_wrong_over_30"] <= 0.02
    assert t["id_switches"] == 0
    assert t["hand_detected"] >= 0.8
    assert t["by_phase"]["stop"][0] >= 0.99      # stopped robot is still found


def test_accuracy_crowd(results):
    t = results["crowd"]["tracked"]
    print("\ncrowd:", evaluate.fmt(results["crowd"]))
    moving = [v for k, v in t["by_phase"].items() if k != "park"]
    assert all(rate >= 0.99 for rate, n in moving)
    assert t["false_per_100_frames"] <= 3        # no ghost where the parked robot was
    assert t["id_switches"] == 0
    assert t["heading_wrong_over_30"] <= 0.02


def test_standalone_detector_interface(recs):
    model = motion.load(None)
    import cv2
    cap = cv2.VideoCapture(os.path.join(recs["main"], "video.avi"))
    out = None
    for _ in range(200):
        ok, frame = cap.read()
        out = motion.predict(model, frame)
    assert {"cls", "x", "y", "heading_deg", "score"} <= set(out[0])
    truth = evaluate.load_truth(recs["main"])[199]
    found = evaluate.match(truth, out, 15)
    assert len(found) >= 2      # live mode has no look-ahead background, but finds the movers


def test_robot_cut_off_by_edge_is_not_a_hand():
    import numpy as np
    rng = np.random.default_rng(0)
    floor = synth.make_floor(rng)
    bg = np.clip(floor, 0, 255).astype(np.uint8)
    img = floor.copy()
    synth.draw_robot(img, 8.0, 200.0, 0.0)        # centre 8 px from the left edge: half out of view
    synth.draw_robot(img, 300.0, 200.0, 30.0)     # a whole one for comparison
    frame = np.clip(img, 0, 255).astype(np.uint8)
    dets, _ = motion.find_blobs(frame, bg, None, 30.0, motion.Params())
    edge = [d for d in dets if d["x"] < 40]
    whole = [d for d in dets if abs(d["x"] - 300) < 5]
    assert len(edge) == 1 and edge[0]["cls"] == "alvik" and edge[0]["score"] < whole[0]["score"]


# ----------------------------------------------------------------------------
# Frame selection
# ----------------------------------------------------------------------------

def test_select_frames(tmp_path, recs):
    dets, info = motion.detect_recording(os.path.join(recs["main"], "video.avi"), n_frames=1200)
    times = [f / 30.0 for f in range(len(dets))]
    rows, _ = tracker.run_tracker(dets, times, info["robot_px"])
    cands = select_frames.candidates(rows, len(dets), times)
    kept, why = select_frames.pick(os.path.join(recs["main"], "video.avi"), cands, rows, times, str(tmp_path))
    regular = [f for f in kept if "regular" in why[f]]
    assert 30 <= len(regular) <= 41                      # about one per second over 40 s
    assert any("appear" in why[f] for f in kept)         # the hand arriving
    assert any("close" in why[f] for f in kept)          # two robots passing close
    assert all(os.path.exists(os.path.join(str(tmp_path), "%06d.jpg" % f)) for f in kept)
    assert len(kept) == len(set(kept))


def test_select_skips_near_duplicates():
    # Nothing moves at all for 60 s: regular frames are duplicates; keep one per 10 s.
    import numpy as np
    import cv2
    rows = {}
    times = [f / 30.0 for f in range(1800)]
    cands = select_frames.candidates(rows, 1800, times)
    d = os.path.join(os.path.dirname(HERE), "tests", "_tmp_still")
    os.makedirs(d, exist_ok=True)
    try:
        path = os.path.join(d, "still.avi")
        w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), 30, (160, 120))
        rng = np.random.default_rng(0)
        base = rng.integers(100, 140, (120, 160, 3), dtype=np.uint8)
        for _ in range(1800):
            w.write(base)
        w.release()
        kept, why = select_frames.pick(path, cands, rows, times)
        assert 6 <= len(kept) <= 9
    finally:
        shutil.rmtree(d, ignore_errors=True)


# ----------------------------------------------------------------------------
# The app: session logic and HTTP endpoints
# ----------------------------------------------------------------------------

def read_labels(folder):
    with open(os.path.join(folder, "labels", "labels.csv"), newline="") as f:
        return list(csv.DictReader(f))


class Server:
    def __init__(self, session):
        self.server, self.url = app.serve(session, port=0)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def get(self, path):
        with urllib.request.urlopen(self.url.rstrip("/") + path) as r:
            data = r.read()
            return json.loads(data) if r.headers.get("Content-Type", "").startswith("application/json") else data

    def post(self, path, body):
        req = urllib.request.Request(self.url.rstrip("/") + path, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return dict(json.loads(e.read()), http_status=e.code)

    def close(self):
        self.server.shutdown()
        self.server.server_close()


@pytest.fixture(scope="session")
def main_cache(tmp_path_factory, recs):
    """One app run on 'main' to make the motion cache other tests reuse."""
    d = fresh_copy(recs["main"], str(tmp_path_factory.mktemp("cache_main") / "rec"))
    app.Session(d, log=lambda *a: None)
    return d


def test_http_endpoints_and_labels_csv(tmp_path, main_cache):
    folder = fresh_copy(main_cache, str(tmp_path / "rec"), keep_cache_from=main_cache)
    s = app.Session(folder, log=lambda *a: None)
    srv = Server(s)
    try:
        info = srv.get("/api/info")
        assert info["n_frames"] == 1200 and info["guess_source"] == "motion"
        q = srv.get("/api/queue?mode=all")["frames"]
        assert len(q) == len(s.selected) and q[0]["state"] == "pending"
        page = srv.get("/")
        assert b"<canvas" in page and b"http://" not in page.replace(b"http://127.0.0.1", b"")   # no outside links
        f = [x["f"] for x in q if x["f"] > 300][0]
        fr = srv.get("/api/frame?f=%d" % f)
        jpg = srv.get("/img/%06d.jpg" % f)
        assert jpg[:2] == b"\xff\xd8"
        assert len(fr["objects"]) == 3 and all(o["status"] == "pending" for o in fr["objects"])

        # 1. Correct one guess: move it 6 px and set its heading.
        objs = fr["objects"]
        target = objs[0]
        target.update(x=target["x"] + 6, heading_deg=45.0, heading_known=True, status="corrected")
        res = srv.post("/api/frame", dict(frame=f, objects=objs, state="edited", how="user"))
        assert res["frame"]["state"] == "edited"
        rows = [r for r in read_labels(folder) if int(r["frame"]) == f]
        moved = [r for r in rows if int(r["obj"]) == target["obj"]][0]
        assert moved["status"] == "corrected" and float(moved["x_px"]) == pytest.approx(target["x"], abs=0.05)
        assert moved["heading_deg"] == "45.0" and moved["heading_known"] == "1" and moved["source"] == "motion"
        assert sum(r["status"] == "pending" for r in rows) == 2

        # 2. Accept the frame: the rest become confirmed.
        objs = res["frame"]["objects"]
        for o in objs:
            if o["status"] == "pending":
                o["status"] = "confirmed"
        srv.post("/api/frame", dict(frame=f, objects=objs, state="reviewed", how="user"))
        rows = [r for r in read_labels(folder) if int(r["frame"]) == f]
        assert sorted(r["status"] for r in rows) == ["confirmed", "confirmed", "corrected"]
        review = json.load(open(os.path.join(folder, "labels", "review.json")))
        assert review["frames"][str(f)]["state"] == "reviewed" and review["frames"][str(f)]["changed"]

        # 3. Mark another frame empty: rejected rows + one 'none' row.
        g = [x["f"] for x in q if x["f"] > f][0]
        objs = srv.get("/api/frame?f=%d" % g)["objects"]
        for o in objs:
            o["status"] = "rejected"
        srv.post("/api/frame", dict(frame=g, objects=objs, state="reviewed", how="user"))
        rows = [r for r in read_labels(folder) if int(r["frame"]) == g]
        assert [r["cls"] for r in rows].count("none") == 1
        none = [r for r in rows if r["cls"] == "none"][0]
        assert none["obj"] == "" and none["status"] == "confirmed" and none["source"] == "user"

        # 4. A user-added object gets a new number and source user, blank score.
        h = [x["f"] for x in q if x["f"] > g][0]
        objs = srv.get("/api/frame?f=%d" % h)["objects"]
        objs.append(dict(obj=None, cls="machine", x=50.0, y=60.0, heading_deg=None, heading_known=False,
                         source="user", status="corrected", score=None))
        res = srv.post("/api/frame", dict(frame=h, objects=objs, state="reviewed", how="user"))
        new = [o for o in res["frame"]["objects"] if o["cls"] == "machine"][0]
        assert new["obj"] >= info["stats"]["selected"] * 0 + 4
        row = [r for r in read_labels(folder) if int(r["frame"]) == h and r["cls"] == "machine"][0]
        assert row["source"] == "user" and row["score"] == "" and row["heading_deg"] == "" and row["heading_known"] == "0"

        # 5. Bad input is refused with a message, not a crash; posts that are
        #    not JSON (e.g. a form on some other web page) are refused.
        req = urllib.request.Request(srv.url + "api/auto_accept", data=b"{}", headers={"Content-Type": "text/plain"})
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(req)
        assert e.value.code == 403
        bad = srv.post("/api/frame", dict(frame=h, objects=[dict(cls="dog", x=1, y=1)], state="edited"))
        assert bad["http_status"] == 400 and "class" in bad["error"]

        # 6. Undo back to the guesses.
        srv.post("/api/frame", dict(frame=h, objects=[], state="pending"))
        assert all(r["status"] == "pending" for r in read_labels(folder) if int(r["frame"]) == h)

        # 7. Spot check + auto-accept.
        sp = srv.post("/api/spot/draw", dict(seed=3))
        assert len(sp["sample"]) >= min(10, sp["stats"]["high_pending"])
        acc = srv.post("/api/auto_accept", {})
        assert acc["accepted"] > 0
        st = srv.get("/api/queue?mode=spot")
        sf = st["frames"][0]["f"]
        fr = srv.get("/api/frame?f=%d" % sf)
        assert fr["state"] == "auto" and fr["in_spot"]
        srv.post("/api/frame", dict(frame=sf, objects=fr["objects"], state="reviewed", how="spot"))
        spot = srv.get("/api/queue?mode=spot")["stats"]["spot"]
        assert spot["checked"] == 1 and spot["agree"] == 1 and spot["agreement"] == 1.0
        srv.post("/api/position", dict(mode="spot", frame=sf))
    finally:
        srv.close()

    # 8. Restart: everything comes back.
    s2 = app.Session(folder, log=lambda *a: None)
    assert s2.frame_state(f) == "reviewed" and s2.frame_state(g) == "reviewed" and s2.frame_state(h) == "auto"
    assert s2.review["position"] == dict(mode="spot", frame=sf)
    assert [o["status"] for o in s2.frame_view(f)["objects"]].count("corrected") == 1
    assert s2.spot_result()["checked"] == 1


def test_correction_repropagates_forward(tmp_path, main_cache):
    """Moving a guess on one frame changes the guesses after it."""
    folder = fresh_copy(main_cache, str(tmp_path / "rec"), keep_cache_from=main_cache)
    s = app.Session(folder, log=lambda *a: None)
    f = s.selected[10]
    objs = s.frame_view(f)["objects"]
    victim = objs[0]["obj"]
    for o in objs:
        if o["obj"] == victim:
            o["status"] = "rejected"
    later = [g for g in s.selected if g > f][0]
    before = [o["obj"] for o in s.frame_view(later)["objects"]]
    res = s.save_frame(f, objs, "reviewed", "user")
    after = [o["obj"] for o in s.frame_view(later)["objects"]]
    assert victim in before and victim not in after       # the rejected track does not continue
    assert res["updated"] >= 1


def test_user_click_seeds_parked_robot(tmp_path, recs):
    """Crowd: robot 3 is parked for 25 s, so motion cannot see it. One click on
    frame 0 must make the tracker follow it, also after it drives off."""
    folder = fresh_copy(recs["crowd"], str(tmp_path / "rec"))
    s = app.Session(folder, log=lambda *a: None)
    truth = evaluate.load_truth(folder)
    parked = [t for t in truth[0] if t["obj"] == 3][0]
    objs = s.frame_view(0)["objects"]
    assert not any(math.hypot(o["x"] - parked["x"], o["y"] - parked["y"]) < 15 for o in objs)
    for o in objs:
        o["status"] = "confirmed"
    objs.append(dict(obj=None, cls="alvik", x=parked["x"] + 2, y=parked["y"] - 1, heading_deg=180.0,
                     heading_known=True, source="user", status="corrected", score=None))
    res = s.save_frame(0, objs, "reviewed", "user")
    new_id = [o for o in res["frame"]["objects"] if o["source"] == "user"][0]["obj"]

    def guess_near(frame, obj_id):
        t = [t for t in truth[frame] if t["obj"] == 3][0]
        rows = [r for r in s.guesses.get(frame, []) if r["obj"] == obj_id]
        return rows and math.hypot(rows[0]["x"] - t["x"], rows[0]["y"] - t["y"]) < 15

    assert guess_near(400, new_id)     # parked: followed (coasting)
    assert guess_near(1000, new_id)    # after driving off: same number, on the robot
    assert guess_near(1199, new_id)


def test_model_mode(tmp_path, main_cache):
    """--model: any module with load()/predict() makes the guesses."""
    folder = fresh_copy(main_cache, str(tmp_path / "rec"), keep_cache_from=main_cache)
    mod = tmp_path / "fake_detector.py"
    mod.write_text('''
import csv
def load(path):
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["cls"] == "alvik":
                rows.setdefault(int(r["frame"]), []).append(r)
    return {"rows": rows, "n": 0}
def predict(model, frame_bgr):
    rows = model["rows"].get(model["n"], [])
    model["n"] += 1
    return [dict(cls="alvik", x=float(r["x"]), y=float(r["y"]), heading_deg=float(r["heading_deg"]), score=0.95)
            for r in rows]
''')
    s = app.Session(folder, model=str(mod), weights=os.path.join(folder, "truth.csv"), log=lambda *a: None)
    assert s.source_name.startswith("model")
    f = s.selected[20]
    objs = s.frame_view(f)["objects"]
    truth = evaluate.load_truth(folder)[f]
    assert len(objs) == 3 and all(o["source"] == "model" and o["heading_known"] for o in objs)
    pairs = evaluate.match(truth, objs, 1.0)
    assert len(pairs) == 3
    for i, j, d in pairs:
        assert abs(motion.wrap180(objs[j]["heading_deg"] - truth[i]["heading"])) < 0.5
    assert os.path.exists(os.path.join(folder, "labels", "cache", "model_detections.csv"))


def test_page_screenshot(tmp_path, main_cache):
    """Render the page in headless Chromium and save a screenshot (skipped
    without Playwright)."""
    pw = pytest.importorskip("playwright.sync_api")
    folder = fresh_copy(main_cache, str(tmp_path / "rec"), keep_cache_from=main_cache)
    s = app.Session(folder, log=lambda *a: None)
    srv = Server(s)
    shot = os.environ.get("LABEL_SCREENSHOT", str(tmp_path / "page.png"))
    try:
        with pw.sync_playwright() as p:
            try:
                browser = p.chromium.launch()
            except Exception as e:
                pytest.skip("no Chromium for Playwright: %s" % e)
            page = browser.new_page(viewport={"width": 1400, "height": 800})
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(srv.url)
            page.wait_for_function("document.getElementById('fno').textContent !== '-'")
            first = page.inner_text("#fno")
            page.keyboard.press("Enter")            # accept, go next
            page.wait_for_function("document.getElementById('fno').textContent !== '%s'" % first)
            page.keyboard.press("ArrowRight")
            page.wait_for_timeout(300)
            page.mouse.move(700, 400)
            page.screenshot(path=shot)
            browser.close()
        assert not errors, errors
        assert s.frame_state(int(first)) == "reviewed"
    finally:
        srv.close()
