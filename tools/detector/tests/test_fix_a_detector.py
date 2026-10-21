"""Stage 2 fix round A, detector side: a rejected guess on top of a real (or unchecked) object is not
taught as "nothing here", and the model reports one object per place whatever its type."""
import os

import cv2
import numpy as np

import detcore

HEAD = "frame,t_wall,obj,cls,x_px,y_px,heading_deg,heading_known,source,status,score\n"


def _rec(tmp_path, lines, frames):
    d = tmp_path / "rec1"
    os.makedirs(d / "frames")
    for f in frames:
        cv2.imwrite(str(d / "frames" / ("%06d.jpg" % f)), np.zeros((20, 20, 3), np.uint8))
    (d / "labels.csv").write_text(HEAD + "".join(l + "\n" for l in lines))
    return str(d)


def test_rejected_spots_on_real_objects_are_not_negatives(tmp_path):
    d = _rec(tmp_path, [
        "5,0.2,1,alvik,100,100,,0,user,corrected,",
        "5,0.2,2,alvik,104,101,,0,track,rejected,0.8",      # a duplicate a little off the real robot: dropped
        "5,0.2,3,alvik,200,200,,0,track,rejected,0.8",      # an empty spot: kept
        "6,0.3,1,alvik,100,100,,0,track,pending,0.7",       # an unchecked guess on a negatives-only frame
        "6,0.3,2,alvik,110,100,,0,track,rejected,0.8",      # beside it: dropped
        "6,0.3,3,alvik,300,50,,0,track,rejected,0.8",       # away from it: kept
    ], [5, 6])
    got = {s["frame"]: s for s in detcore.read_labels(d, ["alvik"], verbose=False, neg_only=True)}
    assert [(n["x"], n["y"]) for n in got[5]["negatives"]] == [(200.0, 200.0)]
    assert got[6]["neg_only"] and [(n["x"], n["y"]) for n in got[6]["negatives"]] == [(300.0, 50.0)]
    # the distance follows the type's size
    got = {s["frame"]: s for s in detcore.read_labels(d, ["alvik"], verbose=False, neg_only=True,
                                                      sizes={"alvik": 4.0})}
    assert len(got[5]["negatives"]) == 2


def test_one_object_per_place_whatever_its_type():
    dets = [dict(cls="alvik", x=100.0, y=100.0, score=0.6), dict(cls="unknown", x=103.0, y=101.0, score=0.8),
            dict(cls="alvik", x=160.0, y=100.0, score=0.7)]
    out = detcore.one_per_place(dets, {"alvik": 30, "unknown": 30})
    assert [(d["cls"], d["x"]) for d in out] == [("unknown", 103.0), ("alvik", 160.0)]     # the higher score kept
    # decode applies it: the same object marked under two types gives one guess
    cfg = dict(classes=["alvik", "unknown"], stride=4, heading_classes=[], heading_mode="sincos")
    heat = np.zeros((2, 32, 32), np.float32)
    heat[0, 10, 10], heat[1, 10, 11] = 0.7, 0.9
    heat[0, 25, 25] = 0.8
    off = np.zeros((2, 32, 32), np.float32)
    hd = np.zeros((2, 32, 32), np.float32)
    got = detcore.decode(heat, off, hd, cfg, (1.0, 1.0), score_thresh=0.3)
    assert len(got) == 2 and got[0]["cls"] == "unknown" and got[0]["score"] == np.float32(0.9)
