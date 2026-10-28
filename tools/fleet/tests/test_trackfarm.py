"""trackfarm.py (item 2 of the one-system build): windowed tracking finds what the full picture finds."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
for p in (os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "sim"),
          os.path.join(TOOLS, "recognition"), TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import table_cam as TC  # noqa: E402
import e02_camera as CAM  # noqa: E402
import trackfarm as TF  # noqa: E402

RECT = (0, 0, 122, 61)
STARTS = {"Left": (15.0, 15.0), "Right": (105.0, 45.0)}


def _poses(t):
    return [(15 + 5 * t, 15, 0.0), (105 - 5 * t, 45, math.pi)]


def _run(cam, tc, **kw):
    w = TF.CamWorker("overhead", cam, RECT, 8.0, STARTS, **kw)
    for _ in range(5):
        w.step(tc.render(_poses(0)), 0.0)
    out = []
    for k in range(1, 25):
        t = k * 0.04
        P = _poses(t)
        w.snapshot({n: dict(x=x, y=y, status="seen", t=t - 0.04, vx=5.0 if n == "Left" else -5.0, vy=0.0)
                    for n, (x, y, _) in zip(STARTS, P)})
        out.append(w.step(tc.render(P), t))
    return out


def test_windows_find_both_robots_far_apart(tmp_path):
    """Two robots at opposite ends: the second window lies outside the first one's crop (the bug fixed on 28 Oct:
    a window clipped to the previous window's size came out empty). Windowed outlines match the full picture's."""
    tc = TC.TableCamera(RECT, seed=12)
    tc.true_model(str(tmp_path / "ov.json"))
    cam = CAM.load_camera(str(tmp_path / "ov.json"))
    roi = _run(cam, tc, roi=True)
    full = _run(cam, tc, roi=False)
    assert sum(r["mode"] == "roi" for r in roi) >= 18
    assert not TF.STILL_SKIP                       # off by default (bench: a side-only robot's spots jumped)
    for a, b in zip(roi, full):
        assert len(a["cands"]) == 2
        for c in a["cands"]:
            d = min(math.hypot(c["u"] - o["u"], c["v"] - o["v"]) for o in b["cands"])
            assert d < 1.5, (a["mode"], c, b["cands"])
