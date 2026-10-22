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
