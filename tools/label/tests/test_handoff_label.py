"""Labelling page and session: unsure objects, same-picture frames, the near-duplicate skip for
every rule, the work area filter, time left from the recent rate, and the page's keys."""
import csv
import json
import os
import re
import shutil
import sys

import numpy as np
import cv2
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LABEL = os.path.dirname(HERE)
sys.path.insert(0, LABEL)

import app            # noqa: E402
import select_frames  # noqa: E402
import workarea       # noqa: E402

W, H, FPS, N = 320, 240, 10.0, 200


def still_recording(folder, change_at=150):
    """A still picture for change_at frames, then a white block appears (the picture changes)."""
    os.makedirs(folder, exist_ok=True)
    rng = np.random.default_rng(1)
    base = cv2.GaussianBlur(rng.integers(90, 150, (H, W, 3), dtype=np.uint8), (5, 5), 0)
    other = base.copy()
    cv2.rectangle(other, (180, 40), (300, 200), (250, 250, 250), -1)
    vw = cv2.VideoWriter(os.path.join(folder, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), FPS, (W, H))
    for f in range(N):
        vw.write(base if f < change_at else other)
    vw.release()
    with open(os.path.join(folder, "frames.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame", "t_mono", "t_wall"])
        for f in range(N):
            w.writerow([f, "%.3f" % (f / FPS), "%.3f" % (1000 + f / FPS)])
    with open(os.path.join(folder, "meta.json"), "w") as fh:
        json.dump(dict(width=W, height=H, fps_reported=FPS), fh)
    return folder


@pytest.fixture(scope="module")
def still(tmp_path_factory):
    d = still_recording(str(tmp_path_factory.mktemp("still") / "rec"))
    app.Session(d, log=lambda *a: None, classes=["box", "unknown"])       # makes the motion cache once
    return d


def fresh(src, dst):
    shutil.copytree(src, dst)
    shutil.rmtree(os.path.join(dst, "labels", "frames"), ignore_errors=True)
    for name in ("labels.csv", "review.json"):
        p = os.path.join(dst, "labels", name)
        if os.path.exists(p):
            os.remove(p)
    return dst


def open_with_frames(folder, frames, **kw):
    """A session whose picked frames are exactly `frames` (pictures saved like the app does)."""
    s = app.Session(folder, log=lambda *a: None, classes=["box", "unknown"], loose_classes=["unknown"], **kw)
    s.selected = sorted(frames)
    s.reasons = {f: ["regular"] for f in s.selected}
    select_frames.extract(s.rec.video, s.selected, s.frames_dir)
    s._update_conf()
    s.save()
    return s


def rows(s):
    with open(s.labels_path, newline="") as f:
        return list(csv.DictReader(f))


# ---------------------------------------------------------------- unsure
def test_unsure_is_saved_listed_and_reloaded(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 10))
    obj = dict(obj=None, cls="box", x=100.0, y=100.0, source="user", status="unsure")
    s.save_frame(30, [obj], "reviewed", "user")
    r = [x for x in rows(s) if x["frame"] == "30"]
    assert len(r) == 1 and r[0]["status"] == "unsure" and r[0]["cls"] == "box"
    assert s.queue("unsure")[0]["f"] == 30 and s.stats()["unsure"] == 1
    assert s.frames[30]["changed"]
    s2 = app.Session(d, log=lambda *a: None, classes=["box", "unknown"])
    assert s2.frame_view(30)["objects"][0]["status"] == "unsure" and s2.unsure_frames() == [30]
    # resolved: no longer listed
    o = s2.frame_view(30)["objects"][0]
    o["status"] = "corrected"
    s2.save_frame(30, [o], "reviewed", "user")
    assert s2.unsure_frames() == [] and s2.stats()["unsure"] == 0


def test_bad_status_still_refused(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 10))
    with pytest.raises(ValueError):
        s.save_frame(30, [dict(cls="box", x=1, y=1, status="maybe")], "reviewed", "user")


# ---------------------------------------------------------------- same picture
def test_same_picture_frames_are_accepted_together_and_undone(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 5))
    # an empty frame accepted: every pending picked frame within 10 s with the same picture goes with it
    res = s.save_frame(0, [], "reviewed", "user")
    assert res["same"] == list(range(5, 101, 5))           # 10 s = 100 frames at 10 fps
    assert all(s.frames[g]["state"] == "auto" and s.frames[g]["how"] == "same" for g in res["same"])
    by_frame = {}
    for r in rows(s):
        by_frame.setdefault(int(r["frame"]), []).append(r)
    assert by_frame[50][0]["cls"] == "none" and by_frame[50][0]["status"] == "auto"
    review = json.load(open(s.review_path))
    assert review["frames"]["50"]["how"] == "same" and review["frames"]["50"]["same_as"] == 0
    assert s.stats()["same"] == len(res["same"])
    # "Undo auto-accept" leaves them (a person's accept vouches for them); Z (undo_same) takes them back
    assert s.undo_auto() == 0
    assert s.undo_same(res["same"]) == len(res["same"])
    assert all(s.frame_state(g) == "pending" for g in res["same"])


def test_same_picture_needs_the_same_objects_and_picture(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 5))
    obj = dict(obj=None, cls="box", x=100.0, y=100.0, heading_deg=None, source="user", status="corrected")
    res = s.save_frame(120, [obj], "reviewed", "user")
    # after 120 the tracker carries the box at its spot: same picture, same object -> accepted;
    # before 120 there is no guess there (objects differ); from 150 the picture differs
    assert res["same"] == [125, 130, 135, 140, 145]
    g = s.frames[130]["objects"]
    assert len(g) == 1 and g[0]["cls"] == "box" and abs(g[0]["x"] - 100) < 1 and g[0]["status"] == "confirmed"
    # reload keeps them
    s2 = app.Session(d, log=lambda *a: None, classes=["box", "unknown"], loose_classes=["unknown"])
    assert s2.frame_state(130) == "auto" and s2.frames[130]["how"] == "same"


def test_unsure_frame_does_not_pass_on(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 5))
    obj = dict(obj=None, cls="box", x=100.0, y=100.0, source="user", status="unsure")
    assert s.save_frame(120, [obj], "reviewed", "user")["same"] == []


def test_spot_check_accept_does_not_pass_on(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 5))
    assert s.save_frame(0, [], "reviewed", "spot")["same"] == []


# ---------------------------------------------------------------- near-duplicates for every rule
def test_close_frames_of_a_still_scene_are_not_stacked(tmp_path, still):
    times = [f / FPS for f in range(N)]
    rows_by_frame = {f: [dict(obj=1, x=100.0, y=100.0, score=0.9, source="motion", close=True),
                         dict(obj=2, x=110.0, y=100.0, score=0.9, source="motion", close=True)] for f in range(N)}
    cands = select_frames.candidates(rows_by_frame, N, times)
    n_close = sum(1 for rs in cands.values() if "close" in rs)
    assert n_close >= 30                                    # a close frame every 0.5 s
    kept, why = select_frames.pick(os.path.join(still, "video.avi"), cands, rows_by_frame, times)
    assert 0 in kept and N - 1 in kept                     # start and end always
    assert len(kept) <= 6                                    # not a stack of the same picture
    # frames that must be kept (already labelled) are kept even when they are duplicates
    kept2, _ = select_frames.pick(os.path.join(still, "video.avi"), cands, rows_by_frame, times, keep={33, 77})
    assert {33, 77} <= set(kept2)


# ---------------------------------------------------------------- work area
def test_workarea_geometry():
    sq = [[100, 100], [200, 100], [200, 200], [100, 200]]
    a = workarea.Area(sq, margin_px=10, ignore_spots=[[150, 150, 5]])
    assert a.keeps(120, 120) and a.keeps(205, 150) and not a.keeps(215, 150)
    assert not a.keeps(151, 151) and a.keeps(160, 160)
    assert workarea.Area(None, None, []).keeps(-50, 9999)                  # no area: keep everything
    assert workarea.Area(sq, None, [], default_margin=30).keeps(225, 150)  # blank margin: one object size
    dets = [[dict(x=120, y=120), dict(x=300, y=300)], None, []]
    out, n = workarea.filter_guesses(dets, a)
    assert n == 1 and out[0] == [dict(x=120, y=120)] and out[1] is None and dets[0][1] == dict(x=300, y=300)


def test_session_drops_guesses_outside_the_work_area(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    det = os.path.join(d, "labels", "cache", "motion_detections.csv")
    mtime = os.path.getmtime(det)
    extra = {50: [dict(cls="box", x=60.0, y=60.0, heading_deg=None, score=0.9, source="camera"),
                  dict(cls="box", x=300.0, y=230.0, heading_deg=None, score=0.9, source="camera"),
                  dict(cls="box", x=100.0, y=100.0, heading_deg=None, score=0.9, source="camera")]}
    area = dict(work_area=[[20, 20], [160, 20], [160, 160], [20, 160]], margin_px=5, ignore_spots=[[100, 100, 8]])
    s = app.Session(d, log=lambda *a: None, classes=["box", "unknown"], extra_dets=extra, area=area)
    assert [(g["x"], g["y"]) for g in s.dets[50]] == [(60.0, 60.0)]
    assert s.dropped_by_area >= 2
    assert os.path.getmtime(det) == mtime                   # the cached motion pass is not redone
    v = s.area_view()
    assert v["work_area"][1] == [160.0, 20.0] and v["margin_px"] == 5 and v["ignore_spots"] == [[100.0, 100.0, 8.0]]


def test_motion_class_map_renames_guesses():
    dets = [[dict(cls="alvik", x=1, y=1), dict(cls="hand", x=2, y=2), dict(cls="machine", x=3, y=3)], None]
    out = app.map_classes(dets, {"alvik": "cube", "hand": "unknown"}, ["cube", "unknown"])
    assert [d["cls"] for d in out[0]] == ["cube", "unknown", "unknown"] and out[1] is None
    assert [d["cls"] for d in app.map_classes(dets, None, ["x"])[0]] == []       # no catch-all: dropped


# ---------------------------------------------------------------- time left
def test_time_left_comes_from_the_recent_rate():
    c = app.RateClock(window=30)
    # slow start (10 frames/s for 60 s), then fast (100 frames/s)
    t, done = 0.0, 0
    for _ in range(60):
        t += 1
        done += 10
        c.update(done, 20000, now=t)
    for _ in range(40):
        t += 1
        done += 100
        left = c.update(done, 20000, now=t)
    assert abs(left - (20000 - done) / 100.0) < 0.15 * (20000 - done) / 100.0
    assert app.RateClock().update(5, 100, now=0.0) is None         # too early to say
    assert app.fmt_left(None) == "" and "min" in app.fmt_left(600) and "s left" in app.fmt_left(12)


# ---------------------------------------------------------------- the page
PAGE = open(os.path.join(LABEL, "page.html"), encoding="utf-8").read()


def test_page_lists_every_key():
    keys = re.findall(r"<kbd>(.*?)</kbd>", PAGE)
    for k in ["Enter", "H", "F", "U", "?", "Del", "N", "1-9", "E", "Tab", "I", "G", "M", "B", "V", "W", "Z", "Esc"]:
        assert k in keys, k


def test_page_brand_and_colours_by_position():
    assert "<title>Maxfield IRL Tracking</title>" in PAGE and "Identify. Recognize. Label." in PAGE
    assert "alvik" not in PAGE.lower()
    assert "--hand" not in PAGE and "PALETTE[i % PALETTE.length]" in PAGE


def test_tour_points_at_real_parts():
    sels = re.findall(r'\{ sel: "([^"]+)", title: "([^"]+)"(?:, kind: "(\w+)")?', PAGE)
    assert len(sels) >= 18
    ids = set(re.findall(r'id="([^"]+)"', PAGE))
    for sel, title, kind in sels:
        assert sel.startswith("#") and sel[1:].split(" ")[0] in ids, sel
    kinds = {sel: kind for sel, title, kind in sels}
    for sel in ("#autoacc", "#reset", "#strain"):
        assert kinds[sel] == "changes"
    for mode in ("next", "all", "low", "spot", "unsure"):
        assert '#tabs button[data-mode=%s]' % mode in kinds
