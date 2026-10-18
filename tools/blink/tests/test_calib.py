"""Tests for calib.py on a synthetic session + recording: Left and Center
calibrate 0.4 s apart (their rolls overlap), Right does a start nudge while
they are still blinking, Left later a turn run with a nudge."""
import csv
import json
import math
import os

import numpy as np
import pytest

import blink
import calib
import synthworld


def test_phase_shift_finds_a_moving_patch():
    rng = np.random.default_rng(1)
    import cv2
    floor = cv2.GaussianBlur(rng.normal(120, 20, (128, 128)).astype(np.float32), (0, 0), 1.5)
    A, B = floor.copy(), floor.copy()
    for img, (dx, dy) in ((A, (0, 0)), (B, (7.4, -4.2))):
        cv2.rectangle(img, (int(round((50 + dx) * 16)), int(round((50 + dy) * 16))),
                      (int(round((80 + dx) * 16)), int(round((78 + dy) * 16))), 220.0, -1, cv2.LINE_AA, 4)
        cv2.circle(img, (int(round((60 + dx) * 16)), int(round((60 + dy) * 16))), 4 * 16, 40.0, -1, cv2.LINE_AA, 4)
    dx, dy, q = calib.phase_shift(A, B, 3, 15)
    assert abs(dx - 7.4) < 0.4 and abs(dy + 4.2) < 0.4 and q > 8


@pytest.fixture(scope="module")
def case(tmp_path_factory):
    out = str(tmp_path_factory.mktemp("calcase"))
    ev = synthworld.calib_case(out)
    rec = os.path.join(out, "recording")
    blink.main([rec])
    calib.main(["--session", os.path.join(out, "session"), "--recording", rec,
                "--detections", os.path.join(out, "detections.csv")])
    with open(os.path.join(rec, "calibration_results.csv")) as f:
        rows = {r["run"]: r for r in csv.DictReader(f)}
    return ev, rows


def test_every_event_found(case):
    ev, rows = case
    assert set(rows) == {e["run"] for e in ev}
    for e in ev:
        assert rows[e["run"]]["event"] == e["kind"]


def test_identity_blink_and_log_agree(case):
    ev, rows = case
    for e in ev:
        r = rows[e["run"]]
        if e["kind"] == "calib":
            assert r["identity"].startswith("agree"), r
            assert r["robot_blink"] == e["robot"]


def test_heading_from_roll(case):
    ev, rows = case
    for e in ev:
        r = rows[e["run"]]
        err = (float(r["heading_floor_deg"]) - e["heading_floor"] + 180) % 360 - 180
        assert abs(err) < 3.0, (e, r["heading_floor_deg"])
        erri = (float(r["heading_img_deg"]) - e["heading_img"] + 180) % 360 - 180
        assert abs(erri) < 3.0
        assert abs(float(r["moved_cm"]) - (3.0 if e["kind"] == "calib" else 2.0)) < 0.3


def test_led_offset(case):
    ev, rows = case
    for e in ev:
        r = rows[e["run"]]
        assert abs(float(r["led_fwd_cm"]) - e["led_fwd_cm"]) < 0.5, r
        assert abs(float(r["led_left_cm"])) < 0.5, r
