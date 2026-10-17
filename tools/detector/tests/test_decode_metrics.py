"""Turning output maps into detections, and scoring detections."""
import json
import math

import numpy as np
import pytest

import detcore

CLASSES = ["alvik", "hand", "unknown", "machine"]


def cfg(mode="sincos"):
    return dict(classes=CLASSES, stride=4, heading_classes=["alvik", "machine"], heading_mode=mode,
                front_frac=0.35, obj_size_px=dict(detcore.DEFAULT_OBJ_SIZE_PX))


def maps(h=60, w=80, hd_ch=2):
    return np.zeros((4, h, w), np.float32), np.zeros((2, h, w), np.float32), np.zeros((hd_ch, h, w), np.float32)


def test_decode_peak_offset_heading_sincos():
    heat, off, hd = maps()
    heat[0, 10, 20] = 0.9
    heat[0, 10, 21] = 0.5          # a neighbour: suppressed by the 3x3 max
    off[:, 10, 20] = (0.25, 0.5)
    r = math.radians(30)
    hd[:, 10, 20] = (math.sin(r), math.cos(r))
    heat[1, 40, 60] = 0.7          # a hand: no heading even if the maps hold one
    hd[:, 40, 60] = (1, 0)
    heat[2, 5, 5] = 0.2            # below threshold
    # input 320x240 from a 640x480 frame: scale 0.5
    d = detcore.decode(heat, off, hd, cfg(), (0.5, 0.5), 0.3)
    assert [x["cls"] for x in d] == ["alvik", "hand"]
    a = d[0]
    assert a["x"] == pytest.approx(((20.25 * 4) + 0.5) / 0.5 - 0.5)
    assert a["y"] == pytest.approx(((10.5 * 4) + 0.5) / 0.5 - 0.5)
    assert a["heading_deg"] == pytest.approx(30, abs=1e-4)
    assert d[1]["heading_deg"] is None


def test_decode_undoes_stretch():
    """If the input is stretched (e.g. 4:3 frame to a square input) the heading is corrected back."""
    heat, off, hd = maps()
    heat[0, 10, 20] = 0.9
    hd[:, 10, 20] = (math.sin(math.radians(45)), math.cos(math.radians(45)))
    # x squeezed twice as much as y: a 45 deg arrow in the input is shallower in the frame
    d = detcore.decode(heat, off, hd, cfg(), (0.25, 0.5), 0.3)
    assert d[0]["heading_deg"] == pytest.approx(math.degrees(math.atan2(1 / 0.5, 1 / 0.25)), abs=1e-4)


def test_decode_front_point():
    """Front mode: heading = direction from the centre to the front point at about the expected distance."""
    heat, off, hd = maps(hd_ch=3)
    heat[0, 20, 20] = 0.9          # robot centre at cell (20, 20)
    hd[0, 18, 22] = 0.8            # front point 2 cells right and 2 up: 45 deg, 11.3 px at input scale 1
    hd[0, 30, 40] = 0.9            # a front point far away: belongs to nobody
    heat[0, 50, 70] = 0.8          # second robot with no front point near it
    d = detcore.decode(heat, off, hd, cfg("front"), (1.0, 1.0), 0.3)
    assert d[0]["heading_deg"] == pytest.approx(45, abs=1e-4)
    assert d[1]["heading_deg"] is None


def test_preprocess_shapes():
    frame = np.zeros((480, 640, 3), np.uint8)
    x, s = detcore.preprocess(frame, 320, 240, 3)
    assert x.shape == (1, 3, 240, 320) and x.dtype == np.float32 and s == (0.5, 0.5)
    x, _ = detcore.preprocess(frame, 320, 240, 1)
    assert x.shape == (1, 1, 240, 320)


def test_scorer_counts_and_errors():
    sc = detcore.Scorer(CLASSES, radius=8)
    gts = [dict(cls="alvik", x=100, y=100, heading=10), dict(cls="alvik", x=200, y=200, heading=None),
           dict(cls="hand", x=50, y=50, heading=None)]
    preds = [dict(cls="alvik", x=103, y=104, heading_deg=350, score=0.9),   # 5 px off, 20 deg off (wraps)
             dict(cls="alvik", x=300, y=300, heading_deg=0, score=0.8),     # false alarm
             dict(cls="hand", x=100, y=100, heading_deg=None, score=0.7)]   # wrong class at a robot: false alarm
    sc.add(preds, gts)
    rows = {r["cls"]: r for r in sc.rows()}
    a = rows["alvik"]
    assert (a["tp"], a["fp"], a["fn"]) == (1, 1, 1)
    assert a["precision"] == 0.5 and a["recall"] == 0.5
    assert a["centre_err_px_mean"] == pytest.approx(5.0)
    assert a["heading_err_deg_mean"] == pytest.approx(20.0)
    assert rows["hand"]["tp"] == 0 and rows["hand"]["fn"] == 1
    assert rows["all"]["tp"] == 1 and rows["all"]["fp"] == 2 and rows["all"]["fn"] == 2


def test_greedy_matching_prefers_confident():
    pairs = detcore.match_frame([dict(x=0, y=0, score=0.2), dict(x=3, y=0, score=0.9)], [dict(x=1, y=0)], 8)
    assert pairs == [(1, 0, 2.0)]


def test_homography_inverse(tmp_path):
    # floor cm -> px: 4 px per cm, origin at (100, 400), y up on the floor = y down in the image
    H = [[4, 0, 100], [0, -4, 400], [0, 0, 1]]
    p = tmp_path / "floor_calibration.json"
    p.write_text(json.dumps({"H_floor_to_image": H}))
    to_cm = detcore.load_homography(str(p))
    assert to_cm(140, 360) == pytest.approx((10.0, 10.0))
    p.write_text(json.dumps(H))
    assert detcore.load_homography(str(p))(100, 400) == pytest.approx((0.0, 0.0))


def test_per_class_radius():
    assert detcore.parse_radius("8") == 8.0
    assert detcore.parse_radius("8,hand=25") == {"default": 8.0, "hand": 25.0}
    sc = detcore.Scorer(CLASSES, {"default": 8.0, "hand": 25.0})
    sc.add([dict(cls="hand", x=20, y=0, heading_deg=None, score=1), dict(cls="alvik", x=20, y=0, heading_deg=None, score=1)],
           [dict(cls="hand", x=0, y=0, heading=None), dict(cls="alvik", x=0, y=0, heading=None)])
    rows = {r["cls"]: r for r in sc.rows()}
    assert rows["hand"]["tp"] == 1 and rows["alvik"]["tp"] == 0
