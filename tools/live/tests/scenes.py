"""Run the live tracker over a synthetic scene (tools/blink/tests/synthworld.py)
and score its fixes against the scene's robots."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LIVE = os.path.dirname(HERE)
TOOLS = os.path.dirname(LIVE)
for p in (LIVE, TOOLS, os.path.join(TOOLS, "blink"), os.path.join(TOOLS, "blink", "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import synthworld as SW  # noqa: E402
import floormap as FMAP  # noqa: E402
import livecore as LC  # noqa: E402
import sources as S  # noqa: E402
import blink as B  # noqa: E402


def render(out, name, seed, seconds=None):
    sc, robots = SW.scenario(name, seed, seconds)
    if not os.path.exists(os.path.join(out, "video.avi")):
        sc.render(out)
    return sc, robots


def track(folder, lp=None):
    fmap = FMAP.FloorMap(os.path.join(folder, "floor_calibration.json"))
    p = B.Params(px_per_cm=fmap.px_per_cm())
    p.exposure_s = B.exposure_from_meta(folder)[0]
    tr = LC.LiveTracker(fmap, None, p, lp)
    msgs = []
    for fr in S.ReplaySource(folder, speed=0):
        msgs += tr.push(fr.img, fr.t_pi)
    return tr, msgs


def score(sc, robots, msgs, near_cm=5.0):
    """Per fix: error to the named robot; wrong if another robot is closer than near_cm
    (or the name isn't a robot in the scene); far = more than 3 cm from the named
    robot (the robots ignore fixes further than that from their own pose)."""
    shift = sc.pi_offset + sc.delay + sc.t_epoch
    by = {r.name: r for r in robots}
    res = dict(fixes=0, wrong=0, far=0, err=[], herr=[], per={})
    for m in msgs:
        if m["type"] != "fix":
            continue
        res["fixes"] += 1
        res["per"][m["robot"]] = res["per"].get(m["robot"], 0) + 1
        t = m["t_pi"] - shift
        r = by.get(m["robot"])
        d_own = None
        if r is not None:
            x, y, th = r.pose(t)
            d_own = math.hypot(m["x"] - x, m["y"] - y)
        closer = False
        for o in robots:
            if o is r:
                continue
            x, y, _ = o.pose(t)
            d = math.hypot(m["x"] - x, m["y"] - y)
            if d < near_cm and (d_own is None or d < d_own):
                closer = True
        if r is None or closer:
            res["wrong"] += 1
            continue
        res["err"].append(d_own)
        res["far"] += d_own > 3.0
        res["herr"].append(abs((m["th"] - math.degrees(th) + 180) % 360 - 180))
    return res
