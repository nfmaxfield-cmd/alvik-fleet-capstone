"""Reading labels (which rows count) and splitting by recording."""
import os

import pytest

import detcore
from conftest import write_labels


def test_label_rules(tmp_path):
    rows = [
        [0, 0, 1, "alvik", 100, 100, 45, 1, "user", "confirmed", ""],
        [1, 0, 1, "alvik", 100, 100, 45, 1, "track", "pending", "0.8"],     # half-reviewed: whole frame out
        [1, 0, 2, "alvik", 200, 100, 45, 1, "user", "confirmed", ""],
        [2, 0, "", "none", "", "", "", 0, "user", "confirmed", ""],          # confirmed empty frame
        [3, 0, 7, "alvik", 300, 300, "", 0, "motion", "rejected", "0.4"],   # wrong guess: dropped
        [3, 0, 3, "hand", 50, 60, "", 0, "user", "corrected", ""],
        [4, 0, 1, "alvik", 10, 20, 90, 0, "user", "confirmed", ""],          # heading not known
        [5, 0, 1, "forklift", 10, 20, "", 0, "user", "confirmed", ""],       # class not in the list
    ]
    d = write_labels(str(tmp_path / "20261020-141500_E1" / "labels"), rows)
    s = {x["frame"]: x for x in detcore.read_labels(d, verbose=False)}
    assert sorted(s) == [0, 2, 3, 4, 5]
    assert s[0]["objects"][0]["heading"] == 45
    assert s[2]["objects"] == []
    assert [o["cls"] for o in s[3]["objects"]] == ["hand"]
    assert s[4]["objects"][0]["heading"] is None
    assert s[5]["objects"] == []
    assert s[0]["recording"] == "20261020-141500_E1"


def test_find_label_dirs(tmp_path):
    for n in ("a", "b"):
        write_labels(str(tmp_path / n / "labels"), [[0, 0, 1, "alvik", 1, 1, "", 0, "user", "confirmed", ""]])
    assert len(detcore.find_label_dirs(str(tmp_path))) == 2
    assert len(detcore.find_label_dirs(str(tmp_path / "a"))) == 1
    assert len(detcore.find_label_dirs(str(tmp_path / "a" / "labels"))) == 1


def test_split_keeps_recordings_whole():
    counts = {f"rec{i}": 100 + 10 * i for i in range(10)}
    a = detcore.split_recordings(counts, 0.15, 0.15, seed=3)
    assert set(a) == set(counts)
    assert {"train", "val", "test"} == set(a.values())
    assert a == detcore.split_recordings(counts, 0.15, 0.15, seed=3)       # repeatable
    samples = [dict(recording=r, frame=f) for r, n in counts.items() for f in range(n)]
    parts = {p: {s["recording"] for s in detcore.select(samples, a, p)} for p in ("train", "val", "test")}
    assert not (parts["train"] & parts["val"]) and not (parts["train"] & parts["test"]) and not (parts["val"] & parts["test"])


def test_split_small_and_explicit():
    a = detcore.split_recordings({"a": 10, "b": 10, "c": 10}, 0.15, 0.15, seed=0)
    assert sorted(a.values()) == ["test", "train", "val"]
    a = detcore.split_recordings({"a": 10, "b": 10, "c": 10, "d": 5}, val=["b"], test=["d"])
    assert a == {"a": "train", "b": "val", "c": "train", "d": "test"}
    with pytest.raises(ValueError):
        detcore.split_recordings({"a": 10, "b": 10})
    with pytest.raises(ValueError):
        detcore.split_recordings({"a": 10, "b": 10, "c": 1}, val=["zzz"])


def test_split_roundtrip(tmp_path):
    a = {"a": "train", "b": "val", "c": "test"}
    p = os.path.join(tmp_path, "split.json")
    detcore.save_split(a, p)
    assert detcore.load_split(p) == a


def test_distinct_objects_counts_objects_not_frames():
    import detcore
    s = [dict(recording="r1", frame=f, objects=[dict(cls="unknown", obj="90"), dict(cls="alvik", obj="0"),
                                                dict(cls="alvik", obj="1")]) for f in range(5)]
    s.append(dict(recording="r2", frame=0, objects=[dict(cls="unknown", obj="90"), dict(cls="hand", obj="")]))
    assert detcore.distinct_objects(s) == {"unknown": 2, "alvik": 2, "hand": 1}
