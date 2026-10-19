"""The synthetic data's unknown objects, and the hue-turn augmentation."""
import csv
import os
import random

import numpy as np

import make_synthetic
from dataset import AugmentConfig, DetDataset


def _unknown_ids(folder):
    with open(os.path.join(folder, "labels.csv")) as f:
        return {r["obj"] for r in csv.DictReader(f) if r["cls"] == "unknown"}


def test_v2_has_varied_unknowns_and_v1_is_unchanged(tmp_path):
    rng = np.random.default_rng(3)
    n_v2 = 0
    for i in range(6):
        d = str(tmp_path / f"v2_{i}")
        make_synthetic.make_recording(d, 8, rng, 1.79e9)
        n_v2 += len(_unknown_ids(d))
    assert n_v2 >= 6                       # on average ~1.4 objects per recording
    rng = np.random.default_rng(3)
    for i in range(6):
        d = str(tmp_path / f"v1_{i}")
        make_synthetic.make_recording(d, 8, rng, 1.79e9, style="v1")
        assert _unknown_ids(d) <= {"90"}   # v1: at most the one cup


def test_hue_turn_keeps_grey_and_moves_colour():
    random.seed(0)
    np.random.seed(0)
    ds = DetDataset([], dict(in_w=32, in_h=32, stride=4, classes=["alvik"], in_ch=3), cache=False)
    ds.aug = AugmentConfig(hue_p=1.0, photometric=True, blur_p=0, noise_std=0, jpeg_p=0)
    img = np.zeros((32, 32, 3), np.uint8)
    img[:16] = 60                          # grey (a robot)
    img[16:] = (40, 200, 40)               # green (BGR)
    moved = 0
    for _ in range(10):
        out = ds._photometric(img).astype(int)
        g = out[:16]
        assert np.abs(g - g.mean(axis=2, keepdims=True)).max() <= 12    # grey stays grey (white balance aside)
        b, gr, r = out[24, 16]
        moved += not (gr > b + 40 and gr > r + 40)
    assert moved >= 5                      # the green part usually turns into another colour
