"""Fix round B, labelling side: a located (copied or moved) recording keeps its saved guesses,
"Most useful next" spreads its first frames over the training split's parts, the list says which
frames hold an unsure object, and long first passes report their progress often."""
import json
import os
import shutil
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LABEL = os.path.dirname(HERE)
sys.path.insert(0, LABEL)
if HERE not in sys.path:
    sys.path.append(HERE)

import app          # noqa: E402
import motion       # noqa: E402
from test_handoff_label import still_recording, fresh, open_with_frames, N  # noqa: E402


@pytest.fixture(scope="module")
def still(tmp_path_factory):
    d = still_recording(str(tmp_path_factory.mktemp("still_b") / "rec"))
    app.Session(d, log=lambda *a: None, classes=["box", "unknown"])
    return d


# ---------------------------------------------------------------- cache survives a move
def test_moved_recording_keeps_its_motion_guesses(tmp_path, still):
    src = fresh(still, str(tmp_path / "a" / "rec"))
    s = app.Session(src, log=lambda *a: None, classes=["box", "unknown"])
    cache = os.path.join(src, "labels", "cache")
    assert app.has_motion_cache(s.rec, cache)
    # copied somewhere else: a new path and a new file time
    dst = str(tmp_path / "b" / "rec")
    shutil.copytree(src, dst)
    later = time.time() + 3600
    os.utime(os.path.join(dst, "video.avi"), (later, later))
    rec2 = app.Recording(dst)
    assert app.has_motion_cache(rec2, os.path.join(dst, "labels", "cache"))
    said = []
    app.Session(dst, log=said.append, classes=["box", "unknown"])
    assert any("Using saved motion guesses" in m for m in said)
    assert not any("first time only" in m for m in said)


def test_older_cache_keys_still_count_but_another_video_does_not(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    rec = app.Recording(d)
    cache = os.path.join(d, "labels", "cache")
    info_p = os.path.join(cache, "motion_info.json")
    with open(info_p) as f:
        info = json.load(f)
    st = os.stat(rec.video)
    # the key as earlier versions wrote it: [size, file time]
    info["key"]["video"] = [st.st_size, int(st.st_mtime) - 999]
    with open(info_p, "w") as f:
        json.dump(info, f)
    assert app.has_motion_cache(rec, cache)
    info["key"]["video"] = [st.st_size + 1, int(st.st_mtime)]
    with open(info_p, "w") as f:
        json.dump(info, f)
    assert not app.has_motion_cache(rec, cache)
    # the new key: size, frames and length must all match
    sig = app.video_signature(rec)
    assert app.same_video(sig, sig)
    assert not app.same_video(dict(sig, frames=sig["frames"] + 1), sig)
    assert not app.same_video(dict(sig, seconds=sig["seconds"] + 1), sig)


# ---------------------------------------------------------------- most useful next, spread over parts
def _parts_by_block(f):
    """A made-up split: blocks of 30 frames, block k goes to train/val/test in turn; the first and
    last 2 frames of a block are at its edge."""
    k, r = divmod(f, 30)
    if r < 2 or r > 27:
        return None
    return ("train", "val", "test")[k % 3]


def test_most_useful_next_spreads_the_first_frames_over_the_parts(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 3), part_of=_parts_by_block)
    # every frame equally uncertain: plain order would take the first 9 frames of one block
    s.conf = {f: 0.5 for f in s.selected}
    q = [x["f"] for x in s.queue("next")]
    first = [_parts_by_block(f) for f in q[:9]]
    assert first.count("train") >= 2 and first.count("val") >= 2 and first.count("test") >= 2
    # frames at a block edge (never used for training) come after the others that are as uncertain
    edge = [f for f in s.selected if _parts_by_block(f) is None]
    assert edge and q[-len(edge):] == sorted(edge, key=lambda f: (s.conf_of(f), f))
    # ... but a clearly more uncertain one still comes first
    e0 = edge[0]
    s.conf = {f: (0.1 if f == e0 else 0.5) for f in s.selected}
    assert s.queue("next")[0]["f"] == e0
    # never more than SPREAD_BAND out of "least confident first"
    import random
    rnd = random.Random(3)
    s.conf = {f: round(rnd.random(), 2) for f in s.selected}
    c = [x["conf"] for x in s.queue("next") if x["state"] == "pending"]
    assert all(v <= min(c[i:]) + s.SPREAD_BAND + 1e-9 for i, v in enumerate(c))
    # less certain frames still come before clearly more certain ones, whatever their part
    s.conf = {f: (0.2 if _parts_by_block(f) == "test" else 0.9) for f in s.selected}
    q = [x["f"] for x in s.queue("next")]
    n_test = sum(1 for f in s.selected if _parts_by_block(f) == "test")
    assert all(_parts_by_block(f) == "test" for f in q[:n_test])


def test_counts_per_part_and_unsure_flags(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 3), part_of=_parts_by_block)
    box = dict(obj=None, cls="box", x=100.0, y=100.0, source="user", status="corrected")
    s.save_frame(6, [box], "reviewed", "user")         # train
    s.save_frame(36, [box], "reviewed", "user")        # val
    s.save_frame(30, [box], "reviewed", "user")        # edge
    s.save_frame(66, [dict(box, status="unsure")], "reviewed", "user")    # test, but unsure: not counted
    # (frames with the same picture as an accepted one are accepted with it, and count too)
    want = dict(train=0, val=0, test=0, edge=0)
    for f, st in s.frames.items():
        if f != 66 and st["state"] in ("reviewed", "auto"):
            want[_parts_by_block(f) or "edge"] += 1
    assert s.part_counts() == want
    assert want["train"] >= 1 and want["val"] >= 1 and want["edge"] >= 1
    assert s.stats()["parts"] == s.part_counts()
    flags = {x["f"]: x["unsure"] for x in s.queue("all")}
    assert flags[66] is True and flags[6] is False
    # review.json's summary is unchanged in form (no parts in it)
    with open(s.review_path) as f:
        assert "parts" not in json.load(f)["summary"]


def test_without_a_split_the_order_and_counts_are_as_before(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 3))
    s.conf = {f: (0.5 if f % 2 else 0.4) for f in s.selected}
    q = [x["f"] for x in s.queue("next")]
    assert q == sorted(s.selected, key=lambda f: (s.conf_of(f), f))
    assert s.part_counts() is None and s.stats()["parts"] is None


# ---------------------------------------------------------------- progress
def test_first_pass_reports_progress_often_including_the_size_step(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    shutil.rmtree(os.path.join(d, "labels", "cache"), ignore_errors=True)
    seen = []
    app.Session(d, log=lambda *a: None, classes=["box", "unknown"],
                progress=lambda what, done, total: seen.append((what, done, total)))
    motion_steps = [x for x in seen if x[0] == "motion"]
    size_steps = [x for x in seen if x[0] == "size"]
    assert size_steps and size_steps[-1][1] == size_steps[-1][2]
    # about every 30 frames, not every 300
    assert len(motion_steps) >= N // 30


def test_read_frames_at_reports_progress(still):
    seen = []
    imgs = motion.read_frames_at(os.path.join(still, "video.avi"), [5, 100, 150], progress=lambda d, t: seen.append((d, t)))
    assert set(imgs) == {5, 100, 150}
    assert seen and seen[-1] == (151, 151)
