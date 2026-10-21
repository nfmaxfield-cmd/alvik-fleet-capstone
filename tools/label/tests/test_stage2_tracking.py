"""Stage 2 tracking (lane T): the appearance chain, split branches, rejected spots that inform,
the check-in measurement, and per-type motion settings. All on synthetic scenes (stage2_scenes.py)
whose truth is known."""
import math
import os
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LABEL = os.path.dirname(HERE)
sys.path.insert(0, LABEL)
if HERE not in sys.path:
    sys.path.append(HERE)

import appearance       # noqa: E402
import tracker          # noqa: E402
import stage2_scenes as S  # noqa: E402

R = S.R
OLD_KEYS = {"frame", "obj", "cls", "x", "y", "heading_deg", "heading_known", "heading_guess", "score", "source", "close"}
NEW_KEYS = {"chain_len", "chain_broken", "appearance_score", "still", "since_confirmed", "confirm_due",
            "split_unresolved", "split_id", "rejected_spot", "flags"}


def rows_of(rows, obj, frames):
    out = {}
    for f in frames:
        for r in rows.get(f, []):
            if r["obj"] == obj:
                out[f] = r
    return out


def near(rows, f, x, y, d=0.5 * R):
    return [r for r in rows.get(f, []) if math.hypot(r["x"] - x, r["y"] - y) < d]


def who(rows, sc, f, names):
    """obj -> name of the truth object it sits on (and the error)."""
    out = {}
    for r in rows.get(f, []):
        best = min(names, key=lambda n: math.hypot(r["x"] - sc.truth(n, f)[0], r["y"] - sc.truth(n, f)[1]))
        out[r["obj"]] = (best, math.hypot(r["x"] - sc.truth(best, f)[0], r["y"] - sc.truth(best, f)[1]))
    return out


# ------------------------------------------------------------------------------ scenes
@pytest.fixture(scope="module")
def stop_scene():
    """Drives 4 s, stops for 70 s, drives on."""
    sc = S.Scene(seconds=82)
    sc.robot("A", [(0, 100, 240), (4, 220, 240), (74, 220, 240), (78, 340, 240)])
    return sc


@pytest.fixture(scope="module")
def hand_scene():
    """A robot that never moves (confirmed once, on frame 0); a hand passes over it at 20-22 s; a
    second robot drives about elsewhere."""
    sc = S.Scene(seconds=120)
    sc.robot("P", [(0, 320, 260), (200, 320, 260)])
    sc.robot("M", [(0, 80, 420), (5, 80, 420), (25, 560, 420), (60, 560, 420), (80, 80, 440), (120, 80, 440)])
    sc.hand("H", [(0, 320, -60), (20, 320, -60), (20.6, 322, 262), (21.2, 318, 255), (21.8, 320, -60)])
    return sc


def crossing(kind):
    sc = S.Scene(seconds=14)
    if kind == "x":            # two identical robots cross at an X, without stopping
        sc.robot("A", [(0, 80, 160), (2, 80, 160), (12, 560, 320), (14, 560, 320)])
        sc.robot("B", [(0, 80, 320), (2, 80, 320), (12, 560, 160), (14, 560, 160)])
    elif kind == "shallow":    # a flat X: the directions differ little
        sc.robot("A", [(0, 80, 190), (2, 80, 190), (12, 560, 290), (14, 560, 290)])
        sc.robot("B", [(0, 80, 290), (2, 80, 290), (12, 560, 190), (14, 560, 190)])
    else:                      # they meet head on, stay touching for 3 s, leave up and down
        sc.robot("A", [(0, 150, 240), (2, 150, 240), (5, 304, 240), (8, 304, 240), (10.5, 304, 100), (14, 304, 100)])
        sc.robot("B", [(0, 490, 240), (2, 490, 240), (5, 336, 240), (8, 336, 240), (10.5, 336, 380), (14, 336, 380)])
    return sc


@pytest.fixture(scope="module")
def stain_scene():
    """A stain the motion finder reports in three bursts; robot C drives over it at ~31 s; robot D is
    put down right on it at 50 s and stays."""
    bursts = list(range(100, 160)) + list(range(600, 660)) + list(range(1500, 1560))
    sc = S.Scene(seconds=70)
    sc.stain("S", 400, 300, flicker=bursts)
    sc.robot("C", [(0, 60, 300), (26, 60, 300), (34, 600, 300), (70, 600, 300)])
    sc.robot("D", [(0, 400, -100), (50, 400, -100), (50.04, 400, 300), (70, 400, 300)])
    return sc


# ------------------------------------------------------------------------------ 1. appearance chain
def test_robot_that_stops_for_a_minute_stays_found_without_drift(stop_scene):
    sc = stop_scene
    rows, summary = tracker.run_tracker(sc.all_dets(), sc.times, R, frames=sc.frames())
    assert len(summary) == 1, summary                      # one object, never lost
    obj = summary[0]["obj"]
    stopped = range(int(4.5 * 25), int(73.5 * 25))          # 69 s standing still
    got = rows_of(rows, obj, stopped)
    assert len(got) == len(stopped)
    later = [got[f] for f in stopped if f >= 9 * 25]        # once the motion finder lost it
    assert all(r["source"] == "track" and r["still"] and not r["chain_broken"] for r in later)
    xs, ys = {r["x"] for r in later}, {r["y"] for r in later}
    assert len(xs) == 1 and len(ys) == 1                   # held exactly: no drift at all
    assert math.hypot(later[0]["x"] - 220, later[0]["y"] - 240) < 1.0
    assert later[-1]["chain_len"] > 1500
    # it drives on with the same number
    assert rows_of(rows, obj, [sc.n - 30])
    # without pictures the old rule still holds (dropped after 45 s unseen): existing callers unchanged
    _, old = tracker.run_tracker(sc.all_dets(), sc.times, R)
    assert len(old) == 2


def test_object_that_never_moves_is_found_from_one_confirmation(hand_scene):
    sc = hand_scene
    anchors = {0: [S.anchor_obj(7, 320, 260)]}
    run = tracker.TrackRun(sc.all_dets(), sc.times, R, frames=sc.frames())
    rows, _ = run.run(anchors, {7})
    got = rows_of(rows, 7, range(sc.n))
    assert len(got) == sc.n                                   # every frame of the 2 minutes
    hand = set(range(19 * 25, 23 * 25))
    for f, r in got.items():
        assert (r["x"], r["y"]) == (320.0, 260.0)            # exactly where the person put it
        if f > 0 and f not in hand:
            assert r["source"] == "track" and not r["chain_broken"]
            assert r["since_confirmed"] == f
    assert got[0]["source"] == "user" and got[0]["score"] == 1.0
    # confidence falls slowly with the chain's length (a still object: slowly)
    assert got[sc.n - 1]["score"] < got[10]["score"]
    assert got[sc.n - 1]["score"] > 0.6
    # the other robot is followed as usual
    m = who(rows, sc, 30 * 25, ["P", "M"])
    assert sorted(v[0] for v in m.values()) == ["M", "P"]


def test_hand_passing_over_breaks_the_chain_and_flags_it(hand_scene):
    sc = hand_scene
    run = tracker.TrackRun(sc.all_dets(), sc.times, R, frames=sc.frames())
    rows, _ = run.run({0: [S.anchor_obj(7, 320, 260)]}, {7})
    got = rows_of(rows, 7, range(sc.n))
    broken = sorted(f for f, r in got.items() if r["chain_broken"])
    assert broken, "the hand over the robot should break the chain"
    assert broken[0] >= int(19.5 * 25) and broken[-1] <= int(22.5 * 25)
    for f in broken:
        assert "chain_broken" in got[f]["flags"] and got[f]["score"] < 0.5
        assert "chain_broken" in run.frame_flags(f)
    # picked up again at the very same place once the hand is gone
    after = got[broken[-1] + 5]
    assert not after["chain_broken"] and (after["x"], after["y"]) == (320.0, 260.0)
    # the hand never takes the robot's number, and the queue gets the stretch
    assert all(r["cls"] == "alvik" for r in got.values())
    issues = [i for i in run.issues() if i["kind"] == "chain_broken"]
    assert len(issues) == 1 and issues[0]["objs"] == [7]
    assert issues[0]["first"] == broken[0] and issues[0]["ask_frame"] == broken[-1] + 1


# ------------------------------------------------------------------------------ 2. split branches
@pytest.mark.parametrize("kind", ["x", "shallow"])
@pytest.mark.parametrize("pictures", [False, True])
def test_identical_robots_crossing_are_told_apart_by_motion(kind, pictures):
    sc = crossing(kind)
    anchors = {0: [S.anchor_obj(1, *sc.truth("A", 0)), S.anchor_obj(2, *sc.truth("B", 0))]}
    run = tracker.TrackRun(sc.all_dets(), sc.times, R, frames=sc.frames() if pictures else None)
    rows, summary = run.run(anchors, {1, 2})
    assert [s["obj"] for s in summary] == [1, 2]
    end = who(rows, sc, sc.n - 1, ["A", "B"])
    assert end[1][0] == "A" and end[2][0] == "B", end
    assert end[1][1] < 3 and end[2][1] < 3
    assert run.splits and run.splits[0]["resolved"] is True
    assert not any(r["split_unresolved"] for rs in rows.values() for r in rs)


@pytest.mark.parametrize("pictures", [False, True])
def test_lookalikes_that_stopped_together_are_flagged(pictures):
    sc = crossing("meet")
    anchors = {0: [S.anchor_obj(1, *sc.truth("A", 0)), S.anchor_obj(2, *sc.truth("B", 0))]}
    run = tracker.TrackRun(sc.all_dets(), sc.times, R, frames=sc.frames() if pictures else None)
    rows, _ = run.run(anchors, {1, 2})
    sp = run.splits[0]
    assert sp["resolved"] is False and sp["objs"] == [1, 2]
    flagged = sorted(f for f, rs in rows.items() if any(r["split_unresolved"] for r in rs))
    assert flagged and flagged[0] == sp["ask_frame"] and flagged[-1] == sp["last"]
    assert flagged[0] > 8 * 25                       # the frames after they came apart
    for f in flagged:
        assert run.frame_flags(f)[0] == "split_unresolved"
        assert all(r["split_id"] == sp["split_id"] and r["score"] <= 0.5 for r in rows[f])
    first = run.issues()[0]
    assert first["kind"] == "split_unresolved" and first["ask_frame"] == sp["ask_frame"]
    # a person's label after they came apart settles it: no flag any more
    f = sp["ask_frame"] + 5
    anchors[f] = [S.anchor_obj(1, *sc.truth("A", f)), S.anchor_obj(2, *sc.truth("B", f))]
    rows2, _ = run.run(anchors, {1, 2}, from_frame=f)
    assert not any(r["split_unresolved"] for rs in rows2.values() for r in rs)
    end = who(rows2, sc, sc.n - 1, ["A", "B"])
    assert end[1][0] == "A" and end[2][0] == "B"


def test_redoing_from_part_way_gives_the_same_rows():
    """The studio re-runs from the last snapshot after each label: splits, chains and flags must
    come out exactly as in a run from the start."""
    sc = crossing("x")
    anchors = {0: [S.anchor_obj(1, *sc.truth("A", 0)), S.anchor_obj(2, *sc.truth("B", 0))]}
    frames = sc.frames()
    run = tracker.TrackRun(sc.all_dets(), sc.times, R, frames=frames, snap_every=50)
    full, _ = run.run(anchors, {1, 2})
    for start in (100, 160, 220):
        again, _ = run.run(anchors, {1, 2}, from_frame=start)
        assert again == full
    fresh, _ = tracker.TrackRun(sc.all_dets(), sc.times, R, frames=frames).run(anchors, {1, 2})
    assert fresh == full


# ------------------------------------------------------------------------------ 3. rejections inform
@pytest.mark.parametrize("how", ["label", "reject_here"])
def test_rejected_stain_stays_down_and_a_robot_on_it_is_still_found(stain_scene, how):
    sc = stain_scene
    run = tracker.TrackRun(sc.all_dets(), sc.times, R, frames=sc.frames())
    rows, _ = run.run({})
    stain = near(rows, 130, 400, 300)
    assert len(stain) == 1                                 # the guess the person rejects
    if how == "label":
        anchors = {130: [dict(S.anchor_obj(stain[0]["obj"], 400, 300), status="rejected")]}
    else:
        anchors = {}
        assert run.remember_rejected(130, 400, 300, 15) == 130
        assert run.rejected_spots() == [dict(frame=130, x=400.0, y=300.0, r=15.0)]
    rows, _ = run.run(anchors, from_frame=130)
    # the rejection counts on that frame and after it ...
    assert not near(rows, 131, 400, 300)
    # ... later bursts at the spot that look like the remembered spot stay down
    for f in range(600, 660):
        assert all(r["score"] <= 0.1 for r in near(rows, f, 400, 300)), f
    # ... and before it nothing changed (it informs later decisions only)
    assert near(rows, 129, 400, 300)
    # a robot driving over the stain is found as usual
    over = near(rows, 776, 400, 300)
    assert len(over) == 1 and over[0]["score"] > 0.5 and not over[0]["rejected_spot"]
    # a robot put down right on the stain looks different: found (the old rule hid it)
    put = near(rows, 1260, 400, 300)
    assert len(put) == 1 and put[0]["score"] > 0.5
    if how == "reject_here":
        assert run.forget_rejected(130, 400, 300, 15)
        rows, _ = run.run({}, from_frame=130)
        assert near(rows, 640, 400, 300)                  # forgotten: the stain shows again


def test_without_pictures_rejected_spots_keep_the_old_rule(stain_scene):
    sc = stain_scene
    rows, _ = tracker.run_tracker(sc.all_dets(), sc.times, R)
    obj = near(rows, 130, 400, 300)[0]["obj"]
    rows, _ = tracker.run_tracker(sc.all_dets(), sc.times, R,
                                  anchors={130: [dict(S.anchor_obj(obj, 400, 300), status="rejected")]})
    assert not near(rows, 640, 400, 300)
    assert not near(rows, 1260, 400, 300)                 # hidden until it moves away (as before)


# ------------------------------------------------------------------------------ 4. check-in measurement
def test_checkin_measurement_reports_error_against_chain_length(hand_scene):
    sc = hand_scene
    an = {}
    for f in (0, 50, 300, 1000, 2900):
        an[f] = [S.anchor_obj(1, *sc.truth("P", f)), S.anchor_obj(2, *sc.truth("M", f))]
    res = tracker.measure_checkins(sc.all_dets(), sc.times, R, an, frames=sc.frames())
    assert len(res["pairs"]) == 8
    for p in res["pairs"]:
        assert p["found"] and p["error_r"] < 0.25 and p["other_id"] is None
        assert p["gap_frames"] == p["to_frame"] - p["from_frame"]
    still = [p for p in res["pairs"] if p["obj"] == 1]
    # every frame of the chain counts, except the ones where the hand covered it (20-22 s)
    assert [p["chain_len"] for p in still if not p["from_frame"] < 520 < p["to_frame"]] == \
        [p["gap_frames"] for p in still if not p["from_frame"] < 520 < p["to_frame"]]
    covered = [p for p in still if p["from_frame"] < 520 < p["to_frame"]][0]
    assert covered["gap_frames"] - 40 < covered["chain_len"] < covered["gap_frames"]
    assert sum(b["n"] for b in res["by_length"]) == 8
    assert res["suggested_every_s"] == 120.0
    # without the chain the parked robot is dropped after 45 s unseen: the measurement shows it
    res0 = tracker.measure_checkins(sc.all_dets(), sc.times, R, an)
    lost = [p for p in res0["pairs"] if not p["found"]]
    assert lost and all(p["gap_s"] > 45 for p in lost)
    assert res0["suggested_every_s"] == 30.0


# ------------------------------------------------------------------------------ 5. per-type motion
def test_motion_settings_per_type():
    p = tracker.TrackerParams.from_types([
        dict(name="alvik", has_front=True), dict(name="cart", max_speed=1.0, can_sit_still=False),
        dict(name="person"), "unknown"], loose_classes=("person", "unknown"))
    assert p.motion_of("alvik") == dict(max_speed=3.0, can_be_carried=True, can_sit_still=True)
    assert p.motion_of("cart") == dict(max_speed=1.0, can_be_carried=True, can_sit_still=False)
    assert p.motion_of("person")["can_sit_still"] is False
    assert p.motion_of("unknown")["can_sit_still"] is True        # a cup put down on the table
    assert p.motion_of("never-heard-of")["max_speed"] == 3.0
    tk = tracker.Tracker(R, params=p)
    slow, fast = tracker.Track(1, 0, 0, "cart", 0, 0.0, R, tk.mp), tracker.Track(2, 0, 0, "alvik", 0, 0.0, R, tk.mp)
    slow.voted, fast.voted = "cart", "alvik"
    assert tk._gate(slow, 0.5) < tk._gate(fast, 0.5)


def test_a_type_that_cannot_sit_still_is_not_chained(stop_scene):
    sc = stop_scene
    dets = [[dict(d, cls="cart") for d in ds] for ds in sc.all_dets()]
    p = tracker.TrackerParams.from_types([dict(name="cart", can_sit_still=False)])
    _, summary = tracker.run_tracker(dets, sc.times, R, params=p, frames=sc.frames())
    assert len(summary) == 2                       # dropped after 45 s unseen, as before


# ------------------------------------------------------------------------------ outputs and pictures
def test_rows_keep_their_old_fields_and_add_the_new_ones(stop_scene):
    sc = stop_scene
    for frames in (None, sc.frames()):
        rows, _ = tracker.run_tracker(sc.all_dets()[:300], sc.times[:300], R, frames=frames)
        for rs in rows.values():
            for r in rs:
                assert set(r) == OLD_KEYS | NEW_KEYS
                assert isinstance(r["flags"], list)


def test_video_frames_match_the_pictures_and_are_cached(tmp_path, stop_scene):
    sc = stop_scene
    path = str(tmp_path / "v.avi")
    short = S.Scene(seconds=8)
    short.robot("A", [(0, 100, 240), (8, 400, 240)])
    short.write_video(path)
    vf = appearance.VideoFrames(path, R)
    af = short.frames()
    assert vf.scale == af.scale == 0.5
    for f in (0, 120, 37, 199, 5):                      # forwards and backwards
        a = vf.window(f, 0, 0, vf.w, vf.h).astype(int)
        b = af.window(f, 0, 0, af.w, af.h).astype(int)
        assert np.abs(a - b).mean() < 3.0
    look = appearance.Looker(vf, R)
    tpl = look.patch(60, *short.truth("A", 60))
    hit = look.search(61, *short.truth("A", 60), 0.5 * R, tpl)
    assert abs(hit["x"] - short.truth("A", 61)[0]) < 1.5 and hit["score"] > 0.9
    n = vf.stats["decoded"]
    look.patch(60, *short.truth("A", 60))
    assert vf.stats["decoded"] == n                      # second time from the cache
    vf.close()


def test_similarity_tells_a_stain_from_a_robot_on_it(stain_scene):
    sc = stain_scene
    look = appearance.Looker(sc.frames(), R)
    empty = look.patch(130, 400, 300)
    assert appearance.similarity(empty, look.patch(640, 400, 300)) > 0.8
    assert appearance.similarity(empty, look.patch(1300, 400, 300)) < 0.3
    assert appearance.similarity(empty, look.patch(776, 400, 300)) < 0.3


def test_speed_on_a_long_recording():
    """5 minutes at 25 fps with a parked robot followed by its look the whole time. The 16-minute
    figure (24,000 frames) is measured with `stage2_scenes.py bench` and given in the build report."""
    res = S.bench(7500)
    assert res["parked_found_frames"] == 7500 and res["parked_max_err_px"] == 0.0
    assert res["first_run_s"] < 90
    assert res["rerun_from_middle_s"] < res["first_run_s"]
