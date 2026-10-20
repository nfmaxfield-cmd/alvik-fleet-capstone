"""Investigate (key Q) and the time-saving record, in the labelling session.

Investigate: the scan's verdicts on a still picture where something appears, and on the moving
synthetic recording; the labels at a spot; "change all" as one undo step. Time record: one line
per finished frame, auto and same-picture frames, undone frames, the estimate for older frames,
and the summary by model."""
import csv
import json
import os
import re
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LABEL = os.path.dirname(HERE)
sys.path.insert(0, LABEL)
if HERE not in sys.path:
    sys.path.append(HERE)

import app          # noqa: E402
import investigate  # noqa: E402
import timing       # noqa: E402
from test_handoff_label import still_recording, fresh, open_with_frames, rows, N  # noqa: E402

PAGE = open(os.path.join(LABEL, "page.html"), encoding="utf-8").read()


@pytest.fixture(scope="module")
def still(tmp_path_factory):
    d = still_recording(str(tmp_path_factory.mktemp("still4") / "rec"))
    app.Session(d, log=lambda *a: None, classes=["box", "unknown"])
    return d


@pytest.fixture(scope="module")
def moving(tmp_path_factory):
    d = tmp_path_factory.mktemp("moving")
    subprocess.run([sys.executable, os.path.join(HERE, "synth.py"), str(d / "rec"), "--seconds", "20"],
                   check=True, capture_output=True)
    s = app.Session(str(d / "rec"), log=lambda *a: None)
    return s


# ---------------------------------------------------------------- the scan
def test_scan_background_and_something_put_down(still):
    rec = app.Recording(still)
    times = rec.times
    seen = []
    bg = investigate.scan(rec.video, rec.n, times, 50, 50, 12, 100, progress=lambda d, t: seen.append((d, t)), size=(320, 240))
    assert bg["kind"] == "background" and "never changes in this recording" in bg["headline"]
    assert "stayed still" in bg["detail"] and "second camera" in bg["detail"]          # never claims background for sure
    assert seen and seen[-1][0] == seen[-1][1]                             # the progress bar reaches the end
    assert len(bg["thumbs"]) == 40 and bg["background"].startswith("data:image/jpeg;base64,")
    assert bg["clips"] == [] and bg["runs"] == []
    # the white block appears at frame 150 and stays: "something put down"
    put = investigate.scan(rec.video, rec.n, times, 240, 120, 12, 180, size=(320, 240))
    assert put["kind"] == "appears" and put["now_changed"]
    assert put["stretch"]["frame_from"] >= 145 and put["stretch"]["frame_to"] == N - 1
    assert put["clips"] and put["clips"][0]["what"] == "something appears" and len(put["clips"][0]["pictures"]) >= 2
    assert put["runs"][0]["frame_from"] >= 145
    # the same spot before it appeared: looks as usual, with one visit listed
    before = investigate.scan(rec.video, rec.n, times, 240, 120, 12, 20, size=(320, 240))
    assert before["kind"] == "background_visited" and before["n_runs"] == 1
    assert len(before["line"]["change"]) == len(before["line"]["frames"]) == len(before["line"]["times"])


def test_scan_follows_a_moving_object(moving):
    s = moving
    import csv as _csv
    truth = list(_csv.DictReader(open(os.path.join(s.rec.folder, "truth.csv"))))
    parked = next(r for r in truth if r["obj"] == "3" and r["frame"] == "10")       # parked, then drives off
    res = investigate.scan(s.rec.video, s.rec.n, s.rec.times, float(parked["x"]), float(parked["y"]),
                           0.7 * s.robot_px, 10, s.guesses, size=(640, 480))
    assert res["kind"] == "moves"
    went = [m["went_to"] for m in res["moves"] if m["went_to"]]
    assert went and went[0]["frame"] > 60                                  # it leaves after its parked time (3 s)
    empty = investigate.scan(s.rec.video, s.rec.n, s.rec.times, 30, 30, 0.7 * s.robot_px, 200, s.guesses, size=(640, 480))
    assert empty["kind"] == "background"


def test_scan_can_be_stopped(still):
    rec = app.Recording(still)
    with pytest.raises(InterruptedError):
        investigate.scan(rec.video, rec.n, rec.times, 50, 50, 12, 10, stop=lambda: True, size=(320, 240))


def test_read_frames_jumps_to_the_right_frame(moving):
    import cv2
    import numpy as np
    want = [3, 4, 90, 333, 599]
    seq, got = {}, {}
    cap = cv2.VideoCapture(moving.rec.video)
    for f in range(600):
        ok, img = cap.read()
        if f in want:
            seq[f] = img
    cap.release()
    investigate.read_frames(moving.rec.video, want, lambda f, img: got.__setitem__(f, img))
    for f in want:
        assert np.array_equal(seq[f], got[f]), f


# ---------------------------------------------------------------- track timeline
def test_track_timeline(moving):
    s = moving
    tl = s.track_timeline()
    assert tl["n_frames"] == s.rec.n and tl["objects"] and tl["duration_s"] > 19
    for o in tl["objects"]:
        assert o["segments"][0][0] == o["first"] and o["segments"][-1][1] == o["last"]
        assert all(a <= b for a, b in o["segments"]) and len(o["breaks"]) == len(o["segments"]) - 1
    assert max(o["frames"] for o in tl["objects"]) > 300                  # a long track across the recording
    # a made-up jump is found
    f = next(f for f in sorted(s.guesses) if s.guesses[f])
    obj = s.guesses[f][0]["obj"]
    g2 = {k: [dict(r) for r in v] for k, v in s.guesses.items()}
    for r in g2.get(f + 1, []):
        if r["obj"] == obj:
            r["x"] += 200
    old, s.guesses = s.guesses, g2
    try:
        lane = next(o for o in s.track_timeline()["objects"] if o["obj"] == obj)
        assert f + 1 in lane["jumps"]
    finally:
        s.guesses = old


# ---------------------------------------------------------------- labels at the spot, change all
def _label(s, f, x, y, cls="box", status="corrected"):
    # "spot" (the spot check): frames with the same picture are not accepted along with it
    return s.save_frame(f, [dict(obj=None, cls=cls, x=x, y=y, source="user", status=status)], "reviewed", "spot")


def test_labels_at_a_spot_and_change_all_is_one_undo_step(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 10))
    for f in (20, 40, 60):
        _label(s, f, 101.0, 99.0)
    _label(s, 80, 200.0, 200.0)                                            # elsewhere: not at the spot
    labels = s.labels_at(100, 100, s.spot_radius())
    assert [l["frame"] for l in labels] == [20, 40, 60] and all(l["cls"] == "box" for l in labels)
    before = open(s.labels_path).read()
    items = [dict(frame=l["frame"], obj=l["obj"]) for l in labels]
    r = s.change_at(items, "type", "unknown")
    assert r["changed"] == 3 and r["frames"] == [20, 40, 60] and r["token"]
    assert {x["cls"] for x in rows(s) if x["frame"] in ("20", "40", "60")} == {"unknown"}
    r2 = s.change_at(items[:2], "unsure")
    assert s.unsure_frames() == [20, 40] and s.stats()["unsure"] == 2
    s.undo_change(r2["token"])
    assert s.unsure_frames() == []
    s.undo_change(r["token"])
    assert open(s.labels_path).read().split("\n")[1:] == before.split("\n")[1:]   # back exactly as it was
    r3 = s.change_at(items, "reject")
    assert {x["status"] for x in rows(s) if x["frame"] in ("20", "40", "60") and x["cls"] != "none"} == {"rejected"}
    assert s.frames[20]["empty"]                                            # nothing left on it (a "none" row)
    with pytest.raises(ValueError):
        s.undo_change(r["token"])                                           # each change is undone once
    with pytest.raises(ValueError):
        s.change_at(items, "type", "no-such-type")
    assert s.change_at(items, "reject")["changed"] == 0                     # already rejected: nothing to do
    s.undo_change(r3["token"])
    assert {x["status"] for x in rows(s) if x["frame"] in ("20", "40", "60")} == {"corrected"}
    assert not s.frames[20]["empty"]


def test_investigate_job_in_the_session(tmp_path, still):
    import time
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 10))
    _label(s, 160, 240.0, 120.0)
    v = s.start_investigate(160, 240, 120)
    assert v["status"] in ("running", "done")
    t0 = time.time()
    while s.inv["status"] == "running" and time.time() - t0 < 60:
        time.sleep(0.05)
    v = s.investigate_view()
    assert v["status"] == "done" and v["result"]["kind"] == "appears"
    assert [l["frame"] for l in v["labels"]] == [160]
    with pytest.raises(ValueError):
        s.start_investigate(160, -50, 120)                                   # outside the picture


# ---------------------------------------------------------------- time record
def test_time_record_per_finished_frame(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 10))
    obj = dict(obj=None, cls="box", x=50.0, y=50.0, source="user", status="corrected")
    s.save_frame(10, [obj], "edited", "user")                              # not finished: no line
    assert timing.read(s.stats_path) == []
    s.save_frame(10, [obj], "reviewed", "spot", dict(shown="2026-10-20T09:00:00", active_s=999, edits=4))
    s.save_frame(30, [], "reviewed", "spot", dict(shown="2026-10-20T09:00:00", active_s=6.5, edits=0))
    s.save_frame(50, [dict(obj, status="unsure")], "reviewed", "spot", dict(active_s=3))
    recs = timing.read(s.stats_path)
    mine = [(r["frame"], r["outcome"]) for r in recs if r["outcome"] != "same"]
    assert mine == [(10, "corrected"), (30, "accepted"), (50, "unsure")]
    recs = [r for r in recs if r["outcome"] != "same"]
    assert recs[0]["active_s"] == timing.CAP_S and recs[0]["edits"] == 4 and recs[0]["model"] == "none"
    assert recs[0]["guesses"] == "motion" and recs[0]["objects"] == 1
    s.save_frame(30, [], "pending")                                         # Reset this frame: no longer counts
    assert timing.read(s.stats_path)[-1] == dict(timing.read(s.stats_path)[-1], frame=30, outcome="undone")
    n = s.auto_accept()
    recs = timing.read(s.stats_path)
    assert sum(1 for r in recs if r["outcome"] == "auto") == n > 0
    s.undo_auto()
    assert sum(1 for r in timing.read(s.stats_path) if r["outcome"] == "undone") == n + 1
    s2 = open_with_frames(d, range(0, N, 10))                              # opening again adds nothing
    assert len(timing.read(s2.stats_path)) == len(timing.read(s.stats_path))
    latest = timing.latest_per_frame(timing.read(s.stats_path))
    assert latest[10][0]["outcome"] == "corrected" and latest[30][0]["outcome"] == "undone"
    # the labels themselves are untouched by the record
    assert {x["frame"] for x in rows(s) if x["status"] not in ("pending",)} >= {"10", "50"}


def test_time_record_same_picture_frames(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [100, 110, 120, 130])
    r = s.save_frame(100, [], "reviewed", "user", dict(active_s=2))
    assert r["same"]
    recs = timing.read(s.stats_path)
    assert sorted(x["frame"] for x in recs if x["outcome"] == "same") == sorted(r["same"])
    s.undo_same(r["same"])
    assert sum(1 for x in timing.read(s.stats_path) if x["outcome"] == "undone") == len(r["same"])


def test_estimate_for_frames_saved_before_the_record():
    review = dict(frames={
        "1": dict(state="reviewed", how="user", changed=False, updated="2026-10-20T10:00:00"),
        "2": dict(state="reviewed", how="user", changed=True, updated="2026-10-20T10:00:12"),
        "3": dict(state="reviewed", how="user", changed=False, updated="2026-10-20T10:30:00"),   # after a break
        "4": dict(state="auto", how="auto", changed=False, updated="2026-10-20T10:31:00"),
        "5": dict(state="auto", how="same", changed=False, updated="2026-10-20T10:31:00", same_as=3),
        "6": dict(state="edited", how="user", changed=True, updated="2026-10-20T10:32:00"),       # not finished
        "7": dict(state="reviewed", how="user", changed=False, updated="2026-10-20T11:00:10"),
    })
    est = {e["frame"]: e for e in timing.estimate_from_review(review, unsure_frames=[3], recorded=[7],
                                                               first_model_time="2026-10-20T10:20:00")}
    assert set(est) == {1, 2, 3, 4, 5}
    assert all(e["estimated"] for e in est.values())
    assert est[2]["active_s"] == 12.0 and est[2]["outcome"] == "corrected" and est[2]["model"] == "none"
    # a gap longer than CAP_S is a break, not work: that frame gets the usual gap (here the only other one, 12 s)
    assert est[3]["active_s"] == 12.0 and est[3]["outcome"] == "unsure" and est[3]["model"] == "unknown"
    assert est[1]["active_s"] is not None                                   # the first: the median of the others
    assert est[4]["outcome"] == "auto" and est[5]["outcome"] == "same" and est[4]["active_s"] == 0.0


def test_summary_by_model():
    def fr(model, outcome, s, edits=1, est=False):
        return dict(recording="r", frame=0, model=model, outcome=outcome, active_s=s, edits=edits, estimated=est)
    frames = ([fr("none", "corrected", 30) for _ in range(6)] + [fr("none", "accepted", 10) for _ in range(4)] +
              [fr("v1", "accepted", 5, 0) for _ in range(8)] + [fr("v1", "corrected", 15) for _ in range(2)] +
              [fr("v1", "auto", 0, 0) for _ in range(20)] + [fr("v1", "same", 0, 0) for _ in range(5)] +
              [fr("unknown", "accepted", 8, None, True)] + [fr("v1", "undone", 50)])
    sm = timing.summarize(frames, setup_seconds=40, install_seconds=300)
    by = {r["model"]: r for r in sm["by_model"]}
    assert [(r["model"], r["measured"]) for r in sm["by_model"]] == [("none", True), ("v1", True), ("unknown", False)]
    assert by["none"]["seconds_per_frame"] == 22.0 and by["none"]["accepted_unchanged"] == 0.4
    assert by["v1"]["seconds_per_frame"] == 7.0 and by["v1"]["accepted_unchanged"] == 0.8
    assert by["v1"]["auto"] == 20 and by["v1"]["same"] == 5 and by["unknown"]["estimated"] == 1
    t = sm["totals"]
    # frames accepted without looking are counted, never credited as time saved; the "if checked" figure is
    # a separate, labelled estimate
    assert t["baseline_measured"] and t["saved_by_accepting_s"] is None and t["estimate_if_checked_s"] == 25 * 22.0
    assert t["auto"] == 20 and t["same"] == 5
    assert t["saved_by_models_s"] == (22.0 - 7.0) * 10 and t["setup_seconds"] == 40 and t["install_seconds"] == 300
    assert t["seconds_per_frame"] == round((6 * 30 + 4 * 10 + 8 * 5 + 2 * 15) / 20, 1)    # measured frames only
    assert t["estimated_seconds_per_frame"] == 8.0
    csv_text = timing.summary_csv(sm)
    assert csv_text.startswith(",".join(timing.SUMMARY_COLUMNS)) and "estimate_if_checked_s" in csv_text
    lines = list(csv.reader(timing.frames_csv(frames).splitlines()))
    assert lines[0] == timing.CSV_COLUMNS and len(lines) == len(frames) + 1


def test_clean_timing_is_checked():
    assert timing.clean_timing(None) == (None, 0.0, 0)
    assert timing.clean_timing(dict(active_s="x", edits="y")) == (None, 0.0, 0)
    assert timing.clean_timing(dict(active_s=-5, edits=-2, shown="2026-10-20T10:00:00")) == ("2026-10-20T10:00:00", 0.0, 0)
    assert timing.clean_timing(dict(active_s=1e9))[1] == timing.CAP_S


def test_damaged_record_lines_are_skipped(tmp_path):
    p = str(tmp_path / "s.jsonl")
    timing.append(p, dict(frame=1, outcome="accepted", active_s=3))
    with open(p, "a") as f:
        f.write("{half a line\n\n")
    timing.append(p, dict(frame=2, outcome="corrected", active_s=4))
    assert [r["frame"] for r in timing.read(p)] == [1, 2]


# ---------------------------------------------------------------- the page
def test_page_has_investigate_key_and_buttons():
    keys = re.findall(r"<kbd>(.*?)</kbd>", PAGE)
    assert "Q" in keys
    for i in ("invbtn", "invlast", "tlbtn", "stime", "sshare", "sopen", "loadstop"):
        assert 'id="%s"' % i in PAGE, i
    sels = re.findall(r'\{ sel: "([^"]+)", title: "([^"]+)"', PAGE)
    assert {"#invbtn", "#stime", "#sshare", "#sopen"} <= {s for s, _ in sels}
    for word in ("Ignore this spot", "Change all to this type", "Reject all", "Mark all unsure",
                 "Recording not found: locate it", "Share project", "Open shared project", "Time saved"):
        assert word in PAGE, word
    assert "alvik" not in PAGE.lower()


def test_page_script_parses():
    import shutil
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    js = PAGE[PAGE.index("<script>") + 8:PAGE.rindex("</script>")]
    p = os.path.join(os.path.dirname(HERE), "tests", "__pycache__")
    os.makedirs(p, exist_ok=True)
    f = os.path.join(p, "page_check.js")
    with open(f, "w", encoding="utf-8") as fh:
        fh.write(js)
    r = subprocess.run([node, "--check", f], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
