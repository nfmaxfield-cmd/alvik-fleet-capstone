"""tools/recognition/crosscam.py on synthetic cameras (synth.py) and synthetic robot paths.

    python -m pytest tools/recognition/tests -q
"""
import csv
import math

import numpy as np
import pytest

import synth
from recognition.cameras import CameraModel
from recognition.crosscam import cross_guesses, disagreements, load_frame_times, match_frames

HEIGHTS = {"alvik": 4.0, "machine": 10.0}


@pytest.fixture(scope="module")
def cams():
    A, B = synth.cam_overhead(), synth.cam_oblique()
    uva, xya, _ = synth.clicks(A, synth.board_points(20), noise=0.5, seed=11)
    uvb, xyb, _ = synth.clicks(B, synth.board_points(20), noise=0.5, seed=12)
    return A, B, CameraModel.from_points(uva, xya, synth.SIZE, height_cm=262.0), CameraModel.from_points(
        uvb, xyb, synth.SIZE)


def frame_times(n, fps, t0, jitter=0.0, seed=0):
    rng = np.random.default_rng(seed)
    return {i: t0 + i / fps + (rng.uniform(-jitter, jitter) if jitter else 0.0) for i in range(n)}


# ---------------------------------------------------------------- match_frames
def test_match_frames_same_clock():
    ta = frame_times(100, 25, 1000.0, jitter=0.003, seed=1)
    tb = frame_times(100, 25, 1000.013, jitter=0.003, seed=2)
    m = match_frames(ta, tb)
    assert len(m) >= 98
    for b, a in m.items():
        assert abs(ta[a] - tb[b]) <= 0.5 / 25 + 1e-9
        assert min(abs(ta[k] - tb[b]) for k in ta) == pytest.approx(abs(ta[a] - tb[b]))   # the nearest one


def test_match_frames_offset_clocks_and_gaps():
    ta = frame_times(200, 25, 0.0)                              # t_mono of camrec A: starts at 0
    tb = {f: t for f, t in frame_times(200, 25, 0.0).items()}   # camrec B started 2.317 s later, own zero
    true_b_to_a = {b: round((t + 2.317) * 25) for b, t in tb.items()}
    assert sum(match_frames(ta, tb)[b] == true_b_to_a[b] for b in range(100)) == 0   # wrong without the offset
    m = match_frames(ta, tb, offset_s=2.317)
    assert all(m[b] == true_b_to_a[b] for b in m)
    assert set(m) == {b for b, a in true_b_to_a.items() if a < 200}                  # past A's end: no match
    # a gap in A: B frames inside it get no partner (default max_dt = half a frame interval)
    ta_gap = {f: t for f, t in ta.items() if not 100 <= f < 110}
    m2 = match_frames(ta_gap, tb, offset_s=2.317)
    assert not any(100 <= a < 110 for a in m2.values()) and len(m2) == len(m) - 10
    # list input (index = frame) and an explicit max_dt
    assert match_frames([0.0, 0.04, 0.08], [0.05], max_dt=0.005) == {}
    assert match_frames([0.0, 0.04, 0.08], [0.05], max_dt=0.02) == {0: 1}


def test_load_frame_times_reads_t_wall(tmp_path):
    p = tmp_path / "frames.csv"
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t_wall", "t_mono", "cam_ms"])
        w.writerow([0, "1760000000.1000", "0.0000", "1.0"])
        w.writerow([1, "1760000000.1400", "0.0400", ""])
    assert load_frame_times(p) == {0: 1760000000.1, 1: 1760000000.14}
    assert load_frame_times(p, "t_mono") == {0: 0.0, 1: 0.04}


# ---------------------------------------------------------------- a scene: robots driving under both cameras
def robot_pose(k, t):
    """Robot k: floor (x, y), floor heading (deg) at time t; slow circles (~10 cm/s)."""
    cx, cy, r, w = [(40, 45, 20, 0.5), (85, 80, 15, -0.6), (70, 25, 10, 0.8)][k]
    a = w * t + k
    x, y = cx + r * math.cos(a), cy + r * math.sin(a)
    head = math.degrees(a + math.copysign(math.pi / 2, w))
    return x, y, head % 360


def img_label(cam, x, y, z, hf, obj, cls="alvik", status="confirmed", known=True, score=None):
    u, v = cam.project((x, y, z))
    h = math.radians(hf)
    u1, v1 = cam.project((x + 0.01 * math.cos(h), y + 0.01 * math.sin(h), z))
    return dict(obj=obj, cls=cls, x=u, y=v, heading_deg=math.degrees(math.atan2(-(v1 - v), u1 - u)) % 360
                if known else None, heading_known=known, status=status, score=score)


def _ang(a, b):
    return abs((a - b + 180) % 360 - 180)


def test_cross_guesses_end_to_end(cams):
    A, B, mA, mB = cams
    ta = frame_times(50, 25, 500.0, jitter=0.002, seed=3)
    tb = frame_times(50, 25, 500.011, jitter=0.002, seed=4)
    labels_a = {}
    for f, t in ta.items():
        objs = [img_label(A, *robot_pose(k, t)[:2], 4.0, robot_pose(k, t)[2], obj=k + 1) for k in range(2)]
        x, y, h = robot_pose(2, t)
        objs.append(img_label(A, x, y, 4.0, h, obj=3, status="pending", score=0.9))       # unconfirmed guess
        objs.append(img_label(A, 20, 100, 4.0, 0, obj=4, status="rejected"))
        objs.append(img_label(A, 100, 40, 10.0, 0, obj=5, cls="machine", known=False, status="corrected"))
        labels_a[f] = objs
    g = cross_guesses(labels_a, mA, mB, ta, tb, synth.SIZE, HEIGHTS)
    assert len(g) >= 48
    pairs = match_frames(ta, tb, return_dt=True)
    worst_px, worst_h = 0.0, 0.0
    for fb, guesses in g.items():
        fa, dt = pairs[fb]
        assert {q["from_obj"] for q in guesses} == {1, 2, 5}             # pending and rejected left out
        for q in guesses:
            assert q["source"] == "camera" and q["from_frame"] == fa and 0 < q["score"] <= 1
            if q["cls"] == "machine":
                u, v = B.project((100, 40, 10.0))
                assert q["heading_deg"] is None and not q["heading_known"]
            else:
                x, y, h = robot_pose(q["from_obj"] - 1, ta[fa])                 # where it was in A's frame
                u, v = B.project((x, y, 4.0))
                hb = img_label(B, x, y, 4.0, h, 0)["heading_deg"]
                worst_h = max(worst_h, _ang(q["heading_deg"], hb))
            worst_px = max(worst_px, math.hypot(q["x"] - u, q["y"] - v))
    assert worst_px < 2.0 and worst_h < 3.0, (worst_px, worst_h)
    # the unconfirmed guess comes along when asked, with its own score folded in
    g2 = cross_guesses(labels_a, mA, mB, ta, tb, synth.SIZE, HEIGHTS, min_score=0.8)
    q3 = [q for q in next(iter(g2.values())) if q["from_obj"] == 3][0]
    q1 = [q for q in next(iter(g2.values())) if q["from_obj"] == 1][0]
    assert q3["score"] < q1["score"]
    assert not any(q["from_obj"] == 3 for q in next(iter(cross_guesses(labels_a, mA, mB, ta, tb, synth.SIZE, HEIGHTS,
                                                                       min_score=0.95).values())))


def test_cross_guesses_score_falls_with_time_gap_and_unknown_height(cams):
    _, _, mA, mB = cams
    lab = {0: [img_label(mA, 60, 60, 4.0, 0, 1)]}
    near = cross_guesses(lab, mA, mB, {0: 0.0}, {0: 0.0}, None, HEIGHTS)[0][0]["score"]
    far = cross_guesses(lab, mA, mB, {0: 0.0}, {0: 0.1}, None, HEIGHTS, max_dt=0.2)[0][0]["score"]
    unk = cross_guesses({0: [dict(lab[0][0], cls="unknown")]}, mA, mB, {0: 0.0}, {0: 0.0}, None, HEIGHTS)[0][0]
    assert near > 0.9 and far < near - 0.1 and unk["score"] < near, (near, far, unk["score"])
    assert cross_guesses(lab, mA, mB, {0: 0.0}, {0: 0.1}, None, HEIGHTS) == {}         # beyond max_dt: nothing
    # an object B cannot see gives no guess
    off = {0: [img_label(mA, -30, 80, 4.0, 0, 1)]}
    assert cross_guesses(off, mA, mB, {0: 0.0}, {0: 0.0}, None, HEIGHTS) == {}
    # Session.frames-style values are accepted too
    assert cross_guesses({0: dict(state="reviewed", objects=lab[0])}, mA, mB, {0: 0.0}, {0: 0.0}, None, HEIGHTS)


# ---------------------------------------------------------------- disagreements
def test_disagreements_catch_move_missing_and_class(cams):
    A, B, mA, mB = cams
    t = {f: f / 25 for f in range(6)}
    poses = [(40, 45, 30), (85, 80, 200), (70, 25, 90)]

    def lab(cam, f, moved=False, drop=None, cls3="alvik", extra=None):
        out = []
        for k, (x, y, h) in enumerate(poses):
            if k == drop:
                continue
            if moved and k == 0:
                x += 5.0
            out.append(img_label(cam, x, y, 4.0, h, obj=10 * (cam is B) + k + 1, cls=cls3 if k == 2 else "alvik"))
        return out + (extra or [])

    off_b = img_label(A, -30, 80, 4.0, 0, obj=9)          # only A can see this spot: never "missing in B"
    la = {f: lab(A, f, extra=[off_b]) for f in t}
    lb = {0: lab(B, 0),                                    # agree
          1: lab(B, 1, moved=True),                        # robot 1 is 5 cm off in B
          2: lab(B, 2, drop=1),                            # robot 2 missing in B
          3: lab(B, 3, cls3="machine"),                    # robot 3 called a machine in B
          4: lab(B, 4, moved=True),                        # ... but frame 4 is not fully reviewed in B:
          5: lab(B, 5)}
    lb[4][1]["status"] = "pending"                         # skipped
    la[5] = la[5][1:]                                      # robot 1 missing in A
    d = disagreements(la, lb, mA, mB, t, t, HEIGHTS)
    kinds = sorted((r["frame_b"], r["kind"]) for r in d)
    assert kinds == [(1, "moved"), (2, "missing_in_b"), (3, "class"), (5, "missing_in_a")], d
    mv = [r for r in d if r["kind"] == "moved"][0]
    assert abs(mv["dist_cm"] - 5.0) < 0.6 and mv["obj_a"] == 1 and mv["obj_b"] == 11
    ms = [r for r in d if r["kind"] == "missing_in_b"][0]
    assert ms["obj_a"] == 2 and ms["obj_b"] is None and ms["dist_cm"] is None
    cl = [r for r in d if r["kind"] == "class"][0]
    assert cl["cls"] == "alvik" and cl["cls_b"] == "machine"
    # agreeing frame within the tolerance: the calibration noise alone stays well under 2 cm
    assert not disagreements({0: la[0]}, {0: lb[0]}, mA, mB, t, t, HEIGHTS, tol_cm=0.8)
