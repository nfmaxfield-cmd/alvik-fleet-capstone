"""Headings must turn and mirror correctly with the picture during augmentation."""
import random

import cv2
import numpy as np
import pytest

import detcore
from conftest import write_labels


def tf(h, M, x=100.0, y=100.0):
    return detcore.transform_objects([dict(cls="alvik", x=x, y=y, heading=h)], np.asarray(M, np.float64))[0]


@pytest.mark.parametrize("h,expected", [(0, 180), (30, 150), (90, 90), (200, 340)])
def test_mirror_left_right(h, expected):
    o = tf(h, [[-1, 0, 639], [0, 1, 0]])
    assert o["x"] == pytest.approx(539)
    assert detcore.wrap_deg(o["heading"] - expected) == pytest.approx(0, abs=1e-6)


@pytest.mark.parametrize("h,expected", [(0, 0), (90, 270), (45, 315)])
def test_mirror_up_down(h, expected):
    o = tf(h, [[1, 0, 0], [0, -1, 479]])
    assert detcore.wrap_deg(o["heading"] - expected) == pytest.approx(0, abs=1e-6)


def test_rotation_is_counter_clockwise_on_screen():
    # OpenCV: positive angle = counter-clockwise as seen on screen; heading should go up by the same angle
    M = cv2.getRotationMatrix2D((320, 240), 90, 1.0)
    o = tf(10, M, x=420, y=240)          # a point to the right of the centre ...
    assert o["x"] == pytest.approx(320, abs=1e-6)
    assert o["y"] == pytest.approx(140, abs=1e-6)   # ... ends up above it (y down)
    assert o["heading"] == pytest.approx(100, abs=1e-6)


def test_wrap():
    assert detcore.wrap_deg(190) == pytest.approx(-170)
    assert detcore.wrap_deg(-180) == pytest.approx(180)
    assert detcore.wrap_deg(350 - 10) == pytest.approx(-20)


def test_augmented_picture_and_heading_agree(tmp_path):
    """Draw a dark dot at the centre and a bright dot 24 px ahead, then apply
    random rotations / flips / zooms and check that the labelled heading
    still points from the dark dot to the bright one in the new picture."""
    from dataset import AugmentConfig, DetDataset
    from model import default_config
    img = np.full((480, 640, 3), 128, np.uint8)
    x, y, h = 300.0, 220.0, 35.0
    dx, dy = detcore.heading_to_vec(h)
    cv2.circle(img, (int(x), int(y)), 5, (0, 0, 0), -1)
    cv2.circle(img, (int(round(x + 24 * dx)), int(round(y + 24 * dy))), 5, (255, 255, 255), -1)
    d = write_labels(str(tmp_path / "rec" / "labels"),
                     [[0, 0, 1, "alvik", x, y, h, 1, "user", "confirmed", ""]], {0: img})
    cfg = default_config()
    ds = DetDataset(detcore.read_labels(d), cfg, augment=AugmentConfig(photometric=False, shift=0.0))
    for seed in range(12):
        random.seed(seed)
        it = ds[0]
        o = it["objects"][0]
        im = it["image"].numpy()[0]       # one channel is enough
        dark = np.unravel_index(np.argmin(cv2.GaussianBlur(im, (5, 5), 0)), im.shape)
        bright = np.unravel_index(np.argmax(cv2.GaussianBlur(im, (5, 5), 0)), im.shape)
        seen = detcore.vec_to_heading(bright[1] - dark[1], bright[0] - dark[0])
        assert abs(detcore.wrap_deg(seen - o["heading"])) < 15, (seed, seen, o["heading"])
        assert abs(dark[1] - o["x"]) < 2.5 and abs(dark[0] - o["y"]) < 2.5
