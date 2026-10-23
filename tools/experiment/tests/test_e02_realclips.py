"""Real-footage regression test for the E02 camera tracker: four clips from the first real session (2026-10-21,
the basement camera, hands and all) replayed through the tracker whenever it changes.

The footage (setup.avi, 1.3 GB, and its frames.csv) is not in git: it is the GitHub release `e02-setup-oct21`.
Get it with
    python tools/experiment/get_clips.py
(into tools/experiment/data/clips/), or point E02_CLIPS at a folder that has them. Without it these tests skip.
setup.avi starts at frame 52200 of the recording's frames.csv; the clips are time ranges of it.

What each clip checks (tests/data/realclips/manifest.json; golden tracks next to it, made with
E02_CLIPS_UPDATE=1 after a change was checked by eye on the replay's contact sheet):
  left_parking       Left driven to its corner past Center: followed the whole way
  center_leaves      Center leaves its spot and drives into Right: no "robot" left behind on its old spot
  hand_near_center   Nick's hand reaches in next to Center (22:44:42): the old tracker jumped onto the hand
  center_drive_hand  Center drives, a hand comes in, robots moved by hand: never jumps to anything
Pass: no step of more than 6 cm between frames 0.2 s apart; wherever the golden track and this run both saw the
robot they agree within 3 cm; it sees the robot in at least the golden share of frames minus 5 points.
"""
import csv
import json
import math
import os

import pytest

import e02_testkit as KIT  # noqa: F401  (sets the import paths)

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data", "realclips")
CLIPS = os.environ.get("E02_CLIPS") or os.path.join(os.path.dirname(HERE), "data", "clips")
MAN = json.load(open(os.path.join(DATA, "manifest.json")))


def _have(name):
    return os.path.exists(os.path.join(CLIPS, "setup.avi")) and os.path.exists(os.path.join(CLIPS, "frames.csv"))


def _run(name):
    import e02_camera as CAM
    import e02_replay as R
    m = MAN[name]
    rec = R.Recording(os.path.join(CLIPS, "setup.avi"), os.path.join(CLIPS, "frames.csv"), 52200)
    cam = CAM.load_camera(os.path.join(os.path.dirname(HERE), "data", "overhead_floorcal_roam1.json"))
    spans = m.get("moving") or []
    moving = (lambda t: any(a <= t <= b for a, b in spans))
    i1 = m["first_frame_in_setup"] + m["n"] - 1
    return R.replay(rec, cam, m["robot"], tuple(m["at"]), [], m["start_t"], rec.t_frame(i1) - 0.02, moving_fn=moving)


@pytest.mark.parametrize("name", sorted(MAN))
def test_tracker_on_real_footage(name):
    if not _have(name):
        pytest.skip("real footage not here (python tools/experiment/get_clips.py)")
    samples = _run(name)
    gold_path = os.path.join(DATA, name + "_golden.csv")
    if os.environ.get("E02_CLIPS_UPDATE"):
        with open(gold_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x_cm", "y_cm"])
            for t, x, y in samples:
                w.writerow(["%.3f" % t, "" if x is None else "%.2f" % x, "" if y is None else "%.2f" % y])
    seen = [(t, x, y) for t, x, y in samples if x is not None]
    for a, b in zip(seen, seen[1:]):
        if b[0] - a[0] <= 0.2:
            assert math.hypot(b[1] - a[1], b[2] - a[2]) <= 6.0, ("jump", name, a, b)
    gold = {round(float(r["t"]), 3): (float(r["x_cm"]), float(r["y_cm"])) if r["x_cm"] else None
            for r in csv.DictReader(open(gold_path))}
    worst = 0.0
    for t, x, y in seen:
        g = gold.get(round(t, 3))
        if g:
            worst = max(worst, math.hypot(x - g[0], y - g[1]))
    assert worst <= 3.0, (name, worst)
    share = len(seen) / max(1, len(samples))
    gshare = sum(1 for v in gold.values() if v) / max(1, len(gold))
    assert share >= gshare - 0.05, (name, share, gshare)


OCT22 = json.load(open(os.path.join(DATA, "oct22_left.json")))


def test_tracker_on_oct22_footage():
    """2026-10-22 10:39:58-10:40:55: Left driven in the corner by the x-board, Right parked 30 cm away. This
    morning the tracker took Right for Left during a 167-deg turn (10:40:03) and then lost Left against the board.
    Replayed the way the runner drives the tracker now: the board and its walls, the other robots parked, each leg
    telling it where Left can be (turn: 7 cm, drive d: d + 8 cm), held still between legs.
    Pass: never within 1.1 robot widths of Right's spot; never more than 3.5 cm past the x-board's wall limit; seen
    in at least 70% of the frames (75% when this test was written; 20% for the tracker of this morning)."""
    import e02_camera as CAM
    import e02_plan as PL
    import e02_replay as R
    m = OCT22
    video = os.path.join(CLIPS, "oct22_video.part-ab")
    frames = os.path.join(CLIPS, "oct22_frames.csv")
    if not (os.path.exists(video) and os.path.exists(frames)):
        pytest.skip("Oct 22 footage not here (python tools/experiment/get_clips.py)")
    rec = R.MjpgPiece(video, frames, m["first_frame"])
    cam = CAM.load_camera(os.path.join(os.path.dirname(HERE), "data", "overhead_floorcal_roam1.json"))
    bd = PL.Board.from_dict(m["board"])
    tr = CAM.MotionTracker(cam, ((bd.x0 + bd.x1) / 2, (bd.y0 + bd.y1) / 2), board_rect=(bd.x0, bd.y0, bd.x1, bd.y1))
    wall_y = bd.xboard_cm + bd.uy * PL.ROBOT_HALF_CM
    tr.set_area((bd.x0, bd.y0, bd.x1, bd.y1), walls=[("y", wall_y), ("x", bd.yboard_cm + bd.ux * PL.ROBOT_HALF_CM)])
    i0, i1 = rec.index(m["start_t"]), rec.index(m["end_t"])
    at, others = tuple(m["at"]), [tuple(o) for o in m["others"]]
    tr.set_background([rec.read(i) for i in range(i0 - 12, i0)], robots=[at] + others)
    for k, o in enumerate(others):
        tr.park("o%d" % k, *o)
    tr.follow(m["robot"], at[0], at[1], t=rec.t_frame(i0))
    tr.hold(6.0)
    held, seen, n = True, [], 0
    for i in range(i0, i1 + 1):
        t = rec.t_frame(i)
        cur = [g for g in m["legs"] if g[0] - 0.1 <= t <= g[1] + 0.3]
        if cur and held:
            p = tr.where()
            tr.clear_expect()
            if p:
                tr.hint = (p[1], p[2], 7.0 if cur[0][2] == "turn" else cur[0][3] + 8.0, t + 30)
            held = False
        elif not cur and not held:
            tr.hold(6.0)
            held = True
        s = tr.push(rec.read(i), t)
        n += 1
        if s is not None:
            seen.append(s)
    right = others[1]
    for t, x, y in seen:
        assert math.hypot(x - right[0], y - right[1]) >= CAM.MotionTracker.OTHER_CM, ("took Right", t, x, y)
        past = (y - wall_y) if bd.uy < 0 else (wall_y - y)       # how far past the wall limit, outwards
        assert past <= 3.5, ("past the wall", t, x, y)
    assert len(seen) / n >= 0.70, len(seen) / n


def test_lock_on_oct22_evening():
    """The lock-on tracker (e02_lock.py), 2026-10-22 19:29:59-19:32:15: Left with its lights steady while driving,
    beside the x-board; the runner (the floor-memory detector alone) lost it four times here. Handed Left once at
    the start, told nothing about its look; the empty board's edges taken out; its look taken again after each turn
    on its spot. Pass: seen in at least 85% of pictures (91% when written) and within 6 cm of each of the runner's
    four camera fixes (checked by eye on the footage: on the robot in every 5 s picture)."""
    import e02_camera as CAM
    import e02_lock as L
    import e02_replay as R
    m = json.load(open(os.path.join(DATA, "oct22eve_left.json")))
    video, frames = os.path.join(CLIPS, m["video"]), os.path.join(CLIPS, m["frames"])
    if not (os.path.exists(video) and os.path.exists(frames)):
        pytest.skip("Oct 22 evening footage not here (python tools/experiment/get_clips.py)")
    rec = R.MjpgPiece(video, frames, 0)
    cam = CAM.load_camera(os.path.join(os.path.dirname(HERE), "data", "overhead_floorcal_roam1.json"))
    rpx = CAM.ROBOT_CM * cam.px_per_cm(61, 61, CAM.ROBOT_TOP_CM)
    floor = rec.read(rec.index(rec.t_frame(0) + m["floor_t_after_start"]))
    lk = L.LockTracker(cam, rpx, CAM.ROBOT_TOP_CM, floor=floor)
    i0, i1 = rec.index(m["start_t"]), rec.index(m["end_t"])
    assert lk.start(rec.read(i0), m["at"][0], m["at"][1], rec.t_frame(i0))
    legs = m["legs"]
    out, seen = [], 0
    for i in range(i0, i1 + 1):
        t, img = rec.t_frame(i), rec.read(i)
        if not lk.locked:
            out.append((t, None))
            continue
        if any(g[2] == "turn" and g[1] + 0.3 < t <= g[1] + 0.345 for g in legs):
            lk.retake(img, t)
        cur = [g for g in legs if g[0] - 0.1 <= t <= g[1] + 0.3]
        pred = (lk.pos[1], lk.pos[2]) if cur and cur[0][2] == "turn" else None
        s = lk.step(img, t, predict=pred, still=not cur, in_leg=bool(cur))
        seen += s is not None
        out.append((t, s))
    assert seen >= 0.85 * len(out), (seen, len(out))
    for tf, x, y in m["fixes"]:
        near = [s for t, s in out if s is not None and abs(t - tf) < 0.5]
        assert near, ("not seen at fix", tf)
        assert min(math.hypot(s[1] - x, s[2] - y) for s in near) < 6.0, ("off at fix", tf, x, y, near[0])
