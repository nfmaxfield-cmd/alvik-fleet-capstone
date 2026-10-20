"""Fix round A (data safety), labelling side: same-picture accepts copy the person's positions and
never a heading, a robot turning in place is not "the same picture", "change all" and the undo
buttons never lose a person's work, a finished frame whose rows are missing is labelled again,
and the page refuses requests that don't name this computer."""
import csv
import json
import os
import sys
import threading
import urllib.request

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LABEL = os.path.dirname(HERE)
sys.path.insert(0, LABEL)
if HERE not in sys.path:
    sys.path.append(HERE)

import app          # noqa: E402
import select_frames  # noqa: E402
import timing       # noqa: E402
from test_handoff_label import still_recording, fresh, open_with_frames, rows, N  # noqa: E402


@pytest.fixture(scope="module")
def still(tmp_path_factory):
    d = still_recording(str(tmp_path_factory.mktemp("still_a") / "rec"))
    app.Session(d, log=lambda *a: None, classes=["box", "unknown"])
    return d


def spinning_recording(folder, n=300, fps=25.0):
    """A dark box turning in place (0.6 degrees a frame) on a still background."""
    os.makedirs(folder, exist_ok=True)
    W, H = 640, 480
    rng = np.random.default_rng(1)
    base = cv2.GaussianBlur(rng.integers(90, 150, (H, W, 3), dtype=np.uint8), (5, 5), 0)
    vw = cv2.VideoWriter(os.path.join(folder, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), fps, (W, H))
    for f in range(n):
        img = base.copy()
        ang = f * 0.6
        box = cv2.boxPoints(((320, 240), (36, 26), ang)).astype(np.int32)
        cv2.fillPoly(img, [box], (30, 30, 30))
        cv2.circle(img, (int(320 + 16 * np.cos(np.radians(ang))), int(240 + 16 * np.sin(np.radians(ang)))), 5,
                   (0, 0, 255), -1)
        vw.write(img)
    vw.release()
    with open(os.path.join(folder, "frames.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame", "t_mono", "t_wall"])
        for f in range(n):
            w.writerow([f, "%.3f" % (f / fps), "%.3f" % (1000 + f / fps)])
    json.dump(dict(width=W, height=H, fps_reported=fps), open(os.path.join(folder, "meta.json"), "w"))
    return folder


# ---------------------------------------------------------------- same picture
def test_same_picture_copies_the_persons_position_and_never_a_heading(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 5))
    # the person moved the box 3 px from where the guesses put it and set its heading
    obj = dict(obj=None, cls="box", x=103.0, y=98.0, heading_deg=45.0, heading_known=True, source="user",
               status="corrected")
    res = s.save_frame(120, [obj], "reviewed", "user")
    assert res["same"] == [125, 130, 135, 140, 145]
    for g in res["same"]:
        o = s.frames[g]["objects"][0]
        assert (o["x"], o["y"]) == (103.0, 98.0)                   # the person's position, not the guess's
        assert o["heading_deg"] is None and not o["heading_known"]  # the same picture does not prove a heading
    saved = [r for r in rows(s) if r["frame"] == "130"]
    assert saved[0]["heading_deg"] == "" and saved[0]["heading_known"] == "0" and saved[0]["x_px"] == "103.0"


def test_a_robot_turning_in_place_is_not_the_same_picture(tmp_path):
    d = spinning_recording(str(tmp_path / "spin"))
    n = 300
    extra = {f: [dict(cls="box", x=320.0, y=240.0, score=0.9, source="camera", heading_deg=None)] for f in range(n)}
    s = app.Session(d, log=lambda *a: None, classes=["box", "unknown"], extra_dets=extra, robot_px=36,
                    front_classes=["box"])
    s.selected = list(range(0, n, 25))
    s.reasons = {f: ["regular"] for f in s.selected}
    select_frames.extract(s.rec.video, s.selected, s.frames_dir)
    s._update_conf()
    s.save()
    objs = [dict(o, status="confirmed") for o in s.guess_objects(0)]
    # the small whole-picture test alone calls them the same (the box is a tiny part of the picture) ...
    assert all(float(np.abs(s._thumb(g) - s._thumb(0)).mean()) < select_frames.DUP_LEVEL for g in s.selected)
    # ... but each object's own patch at full size shows the turn
    res = s.save_frame(0, objs, "reviewed", "user")
    assert res["same"] == []
    assert s._patches_same(25, 25, [dict(x=320.0, y=240.0)])          # the same frame is the same, of course


# ---------------------------------------------------------------- change all, the undo buttons
def _auto_frame(s, f, x=60.0):
    s.frames[f] = dict(state="auto", how="auto", objects=[dict(
        obj=7 + f, cls="unknown", x=x, y=60.0, heading_deg=None, heading_known=False, heading_guess=None,
        source="motion", status="confirmed", score=0.9)], auto_conf=0.9, changed=False,
        updated="2026-10-01T09:00:00", empty=False)


def test_change_all_turns_auto_frames_into_reviewed_and_undo_auto_keeps_them(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20, 30, 40])
    _auto_frame(s, 30)
    _auto_frame(s, 40, x=200.0)
    s.save()
    r = s.change_at([dict(frame=30, obj=37)], "type", "box")
    st = s.frames[30]
    assert st["state"] == "reviewed" and st["how"] == "change_all" and st["objects"][0]["cls"] == "box"
    assert st["updated"] == "2026-10-01T09:00:00" and st["changed_all"]      # its own time; the save time stays
    review = json.load(open(s.review_path))["frames"]["30"]
    assert review["how"] == "change_all" and review["changed_all"] and review["updated"] == "2026-10-01T09:00:00"
    # "Undo auto-accept" takes back frame 40 (untouched) but never the frame the person changed
    assert s.undo_auto() == 1 and 30 in s.frames and 40 not in s.frames
    # the change itself is still undoable: back to the auto-accepted guess
    assert s.undo_change(r["token"])["undone"] == 1
    assert s.frames[30]["state"] == "auto" and s.frames[30]["objects"][0]["cls"] == "unknown"
    # frames changed only by "change all" are not timed as if a person checked them one by one
    est = {e["frame"]: e for e in timing.estimate_from_review(dict(frames={
        "1": dict(state="reviewed", how="user", updated="2026-10-01T09:00:00"),
        "2": dict(state="reviewed", how="change_all", updated="2026-10-01T09:00:30", changed_all="2026-10-03T10:00:00"),
        "3": dict(state="reviewed", how="user", updated="2026-10-01T09:00:08")}))}
    assert est[2]["outcome"] == "auto" and est[2]["active_s"] == 0.0 and est[3]["active_s"] == 8.0


def test_undo_change_all_keeps_a_later_edit(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20, 30])
    obj = [dict(obj=1, cls="box", x=50, y=50, status="corrected", source="user")]
    s.save_frame(20, obj, "reviewed", "spot")
    s.save_frame(30, [dict(obj=2, cls="box", x=52, y=50, status="corrected", source="user")], "reviewed", "spot")
    tok = s.change_at([dict(frame=20, obj=1), dict(frame=30, obj=2)], "unsure")["token"]
    s.save_frame(20, [dict(obj=1, cls="box", x=99, y=99, status="corrected", source="user")], "reviewed", "spot")
    res = s.undo_change(tok)
    assert res["kept"] == [20] and res["frames"] == [30]
    assert s.frames[20]["objects"][0]["x"] == 99.0                       # the later edit is kept
    assert s.frames[30]["objects"][0]["status"] == "corrected"           # the other frame is undone


def test_undoing_an_accept_of_a_same_picture_frame_restores_it_as_same(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 5))
    res = s.save_frame(0, [], "reviewed", "user")
    assert 50 in res["same"]
    view = s.frame_view(50)
    # the person opens frame 50 and accepts it, then presses Z: the page sends it back as it was
    s.save_frame(50, view["objects"], "reviewed", "user")
    assert s.frames[50]["state"] == "reviewed"
    s.save_frame(50, view["objects"], "auto", "same")
    assert s.frames[50]["how"] == "same" and s.frames[50]["same_as"] == 0
    assert s.undo_auto() == 0                                             # still vouched for by a person's accept
    assert json.load(open(s.review_path))["frames"]["50"]["same_as"] == 0


# ---------------------------------------------------------------- reloading
def test_finished_frame_without_rows_is_labelled_again(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20, 30, 40])
    obj = [dict(obj=1, cls="box", x=50, y=50, status="corrected", source="user")]
    s.save_frame(40, obj, "reviewed", "spot")
    s.save_frame(30, [], "reviewed", "spot")                                # a confirmed empty frame
    review = json.load(open(s.review_path))
    assert review["frames"]["30"].get("empty") is True and "empty" not in review["frames"]["40"]
    keep = [r for r in rows(s) if r["frame"] != "40"]
    with open(s.labels_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=app.LABEL_COLUMNS)
        w.writeheader()
        w.writerows(keep)
    logs = []
    s2 = app.Session(d, log=logs.append, classes=["box", "unknown"], loose_classes=["unknown"])
    assert s2.frame_state(40) == "pending" and s2.lost_on_load == [40]     # never "confirmed empty"
    assert any("no rows in labels.csv" in m for m in logs)
    assert s2.frames[30]["state"] == "reviewed" and s2.frames[30]["empty"]
    # review.json saying empty is believed even without the "none" row
    keep = [r for r in rows(s2) if r["frame"] != "30"]
    with open(s.labels_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=app.LABEL_COLUMNS)
        w.writeheader()
        w.writerows(keep)
    s3 = app.Session(d, log=lambda *a: None, classes=["box", "unknown"], loose_classes=["unknown"])
    assert s3.frames[30]["state"] == "reviewed" and s3.frames[30]["empty"] and s3.lost_on_load == []


# ---------------------------------------------------------------- the web page
def test_get_routes_check_the_host(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20])
    server, url = app.serve(s, 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with urllib.request.urlopen(url + "api/info", timeout=10) as r:
            info = json.loads(r.read())
        assert r.status == 200 and info["guess_label"] == "Guesses from motion (no model)"
        req = urllib.request.Request(url + "api/info", headers={"Host": "evil.example:%d" % server.server_address[1]})
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(req, timeout=10)
        assert e.value.code == 403
        req = urllib.request.Request(url, headers={"Host": "evil.example"})
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(req, timeout=10)
        assert e.value.code == 403
    finally:
        server.shutdown()
        server.server_close()


def test_write_atomic_uses_its_own_temporary_file(tmp_path):
    p = str(tmp_path / "x.csv")
    errs = []

    def w(k):
        for i in range(200):
            try:
                app.write_atomic(p, ("%d,%d\n" % (k, i)) * 50)
            except Exception as e:      # noqa: BLE001
                errs.append(e)
    ts = [threading.Thread(target=w, args=(k,)) for k in range(3)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    assert errs == [] and len(open(p).read().splitlines()) == 50
    assert [f for f in os.listdir(tmp_path) if f.endswith(".tmp")] == []
