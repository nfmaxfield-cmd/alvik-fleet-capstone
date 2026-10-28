"""The overhead's washout guard (2026-10-28): exposure stays automatic unless the board clips; then it steps down,
says so, logs it, and goes back to automatic once the board has stopped clipping for a while.

    python3 -m pytest tools/pi/tests/test_camrec_washout.py -q
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import camrec as CR  # noqa: E402


class FakeCam:
    def __init__(self, exposure=200.0):
        self.mode, self.exposure_v = "auto", exposure
        self.calls = []

    def exposure(self):
        return self.exposure_v

    def manual(self, v):
        self.mode, self.exposure_v = "manual", v
        self.calls.append(("manual", v))
        return True

    def auto(self):
        self.mode = "auto"
        self.calls.append(("auto",))
        return True


def board(level, clip_share=0.0, h=480, w=640):
    g = np.full((h, w), level, np.uint8)
    if clip_share:
        n = int(clip_share * h * w)
        g.reshape(-1)[np.random.default_rng(1).choice(h * w, n, replace=False)] = 255
    return g


def guard(cam, said):
    return CR.WashoutGuard(cam.exposure, cam.manual, cam.auto, log=said.append,
                           mark=lambda text, t: said.append("MARK " + text))


def test_board_that_does_not_clip_is_left_alone():
    """28 Oct 16:40: board ~193, nothing at 250+: the guard does nothing (exposure stays automatic)."""
    cam, said = FakeCam(), []
    g = guard(cam, said)
    for k in range(60):
        g.check(board(193), 1000.0 + k)
    assert cam.calls == [] and cam.mode == "auto" and said == []


def test_clipping_board_steps_down_then_back_to_automatic():
    cam, said = FakeCam(200.0), []
    g = guard(cam, said)
    t = 0.0
    g.check(board(240, 0.10), t)                  # one bright check: not yet
    assert cam.calls == []
    t += 2.0
    g.check(board(240, 0.10), t)                  # second in a row: 200 -> 160
    assert cam.calls == [("manual", 160)] and "washed out" in said[0] and "was automatic, 200" in said[0]
    assert any(s.startswith("MARK ") for s in said)          # in the camera's events.csv too
    for _ in range(2):
        t += 2.0
        g.check(board(240, 0.10), t)
    assert cam.calls[-1] == ("manual", 128)
    for _ in range(10):                            # clear, but not yet for 60 s
        t += 2.0
        g.check(board(190), t)
    assert cam.mode == "manual"
    for _ in range(30):
        t += 2.0
        g.check(board(190), t)
    assert cam.mode == "auto" and "back to automatic" in said[-1]


def test_floor_of_the_steps_and_plain_words():
    cam, said = FakeCam(100.0), []
    g = guard(cam, said)
    t = 0.0
    for _ in range(40):
        t += 2.0
        g.check(board(250, 0.5), t)
    assert min(v for _, v in [c for c in cam.calls if c[0] == "manual"]) >= 30      # MIN_FRACTION 0.3 of 100
    assert "close the blinds or turn off the lamp" in said[-1]


def test_clipped_share_counts_only_the_middle():
    g = board(190)
    g[:40, :] = 255                                # the edge of the picture (not the board) clips
    assert CR.WashoutGuard.clipped_share(g) == 0.0
