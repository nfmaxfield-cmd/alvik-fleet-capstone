"""Tests for blink.py: the decoder on its own, then whole synthetic videos.

    python -m pytest tools/blink/tests -q        (about 1-2 minutes)
"""
import math
import os

import numpy as np
import pytest

import blink
import score


# ------------------------------------------------------------------ decoder units
def flashes(code, t0, n, rng, jitter=0.012, drop=0.0, delay=0.0):
    """Flash start times of a robot: one flash every (code + 2) x 100 ms."""
    T = (code + 2) * 0.1
    out = []
    for k in range(n):
        if rng.random() < drop:
            continue                      # held dark: flash missed
        t = t0 + k * T + rng.normal(0, jitter)
        if rng.random() < delay:
            t += rng.uniform(0.0, 0.115)  # code blocked at the flash start
        out.append(t)
    return np.array(out)


@pytest.mark.parametrize("code", [1, 2, 3, 4])
def test_decode_rate_each_code(code):
    rng = np.random.default_rng(code)
    p = blink.Params()
    for t0 in (0.0, 0.37, 1.21):
        dec = blink.decode_rate(flashes(code, 100 + t0, 14, rng, drop=0.1, delay=0.1), p)
        assert dec["code"] == code
        assert dec["probs"][code] > 0.9
        assert abs(dec["period_s"] - (code + 2) * 0.1) < 0.02


def test_decode_rate_says_none_for_other_rhythms():
    rng = np.random.default_rng(3)
    p = blink.Params()
    # a lamp flickering at 4 Hz: each interval is "nearly" a Left period
    flick = 50 + np.arange(30) * 0.25 + rng.normal(0, 0.012, 30)
    assert blink.decode_rate(flick, p)["p_none"] > 0.9
    # random flashes
    nones = []
    for _ in range(20):
        a = np.cumsum(rng.uniform(0.25, 0.9, 12))
        nones.append(blink.decode_rate(a, p)["p_none"])
    assert np.median(nones) > 0.9


def test_late_frame_repair():
    t = np.arange(10) / 30.0
    t[5] += 0.05          # frame 5 reached the Pi late, frame 6 right after it
    t[6] = max(t[6], t[5] + 0.001)
    r = blink.repair_late_frames(t)
    assert abs(r[5] - 5 / 30.0) < 0.01
    assert np.all(np.diff(r) > 0)


# ------------------------------------------------------------------ synthetic videos
def test_clutter_three_robots(scenes):
    s = scenes["clutter"]
    res = score.evaluate(s["rows"], s["truth"])
    print(score.describe(res))
    assert res["accuracy"] >= 0.9
    assert res["false_rows"] == 0 and res["wrong_rows"] == 0
    assert np.median(res["pos_err"]) < 0.5
    assert {r["robot"] for r in s["rows"]} - {""} == {"Left", "Center", "Right"}


def test_crossing_robots_keep_their_names(scenes):
    s = scenes["crossing"]
    res = score.evaluate(s["rows"], s["truth"])
    print(score.describe(res))
    assert res["wrong_rows"] == 0 and res["false_rows"] == 0
    for robot, (n, ok) in res["by_robot"].items():
        assert ok / n >= 0.8, robot


def test_distractors_are_never_named(scenes):
    """Flickering lamps, a 4 Hz flicker, a lamp with 200 ms pulses at random
    gaps, a moving white object, a static white spot, lone 300 ms flashes, an
    idle robot with a steady white light: they may be reported as unnamed
    flashing tracks (fusion decides), but never with a robot's name."""
    rows = scenes["distractors"]["rows"]
    assert not [r for r in rows if r["robot"]]


def test_sync_flash_is_not_a_pulse(scenes):
    s = scenes["clutter"]
    near = [g for g in s["segs"] if math.hypot(g.x[0] - 150, g.y[0] - 120) < 6]
    assert any(g.kind == "sync" for g in near)
    assert not any(g.kind == "pulse" for g in near)


def test_names_are_configurable(scenes):
    s = scenes["crossing"]
    p = blink.Params(names=blink.parse_names("1=Alpha,2=Beta"))
    tracks = blink.decode_tracks(s["tracks"], p, s["segs"])
    rows = blink.rows_for(tracks, s["frame_t"], p)
    assert {r[2] for r in rows} - {""} == {"Alpha", "Beta"}
    blink.decode_tracks(s["tracks"], s["p"], s["segs"])     # put the shared fixture back


def test_output_columns(scenes):
    import csv
    with open(os.path.join(scenes["clutter"]["folder"], "blink_detections.csv")) as f:
        hdr = next(csv.reader(f))
    assert hdr[:7] == ["frame", "t_wall", "robot", "x_px", "y_px", "code", "score"]
    assert hdr[-5:] == ["p_Left", "p_Center", "p_Right", "p_other", "p_none"]


def test_every_flashing_track_is_reported(scenes):
    """Unreadable tracks are reported too (with probabilities), never dropped."""
    s = scenes["distractors"]
    assert len({r["track"] for r in s["rows"]}) >= 3
