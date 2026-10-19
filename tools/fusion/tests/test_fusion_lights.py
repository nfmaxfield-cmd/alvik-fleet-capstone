"""The light-pattern identity source (sensor "lights": tools/lights/lights.py rows) in the fusion engine.

    python -m pytest tools/fusion/tests -q
"""
import math
import random

import engine as eng
from fusion_synth import straight_run, sightings, blink_rows
from ekf import D2R, wrap

OFF = (-0.7, -0.23)          # the printed L's LED centre in the robot frame (patterns.L3Family default)


def two_robots():
    ra, ta = straight_run("A-1", "A", 0.0, 0.0, 0.0, t0=1000.0)
    rb, tb = straight_run("B-1", "B", 0.0, 12.0, 0.0, t0=1000.0, v=6.0)
    return [ra, rb], ta, tb


def light_rows(truth, robot, p=0.99, names=("A", "B"), track=1, seed=3, hd_sd=1.0, fps=25.0):
    probs = {n: (p if n == robot else (1 - p) / len(names)) for n in names}
    rows = sightings(truth, fps=fps, off=OFF, track=track, robot=robot if p >= 0.9 else "", probs=probs,
                     p_none=max(0.0, 1 - sum(probs.values())), seed=seed, kind="full")
    rng = random.Random(seed)
    for r in rows:
        th = min(truth, key=lambda q: abs(q[0] - r["t"]))[3]
        r["heading"] = wrap(th + rng.gauss(0, hd_sd) * D2R)
        r["off"] = OFF
        r.pop("kind")
    return rows


def test_lights_name_and_place_both_robots():
    runs, ta, tb = two_robots()
    lights = light_rows(ta, "A", track=1) + light_rows(tb, "B", track=2, seed=4)
    cfg = eng.config(sensors=["wheel_v", "wheel_w", "gyro", "lights"], motion_identity=False)
    tracked, _ = eng.run_engine(runs, None, cfg, lights=lights)
    for rid, truth in (("A-1", ta), ("B-1", tb)):
        tr = tracked[rid]
        assert tr.cam_used > 150
        assert tr.id_sources.get("lights", 0) == tr.cam_used          # named by the pattern alone
        assert tr.hdg_used["lights"] > 150
        for f in tr.fixes:
            _, x, y, th = min(truth, key=lambda q: abs(q[0] - f["t"]))
            x += math.cos(th) * OFF[0] - math.sin(th) * OFF[1]
            y += math.sin(th) * OFF[0] + math.cos(th) * OFF[1]
            assert math.hypot(f["x"] - x, f["y"] - y) < 2.0
        th = tr.trajectory()[-1][3]
        assert abs(math.degrees(wrap(th - truth[-1][3]))) < 2.0


def test_named_pattern_never_goes_to_another_robot():
    """A pattern named B (0.99) right where A is: not A's, and kept as an unknown object named B."""
    runs, ta, tb = two_robots()
    fake = light_rows(ta, "B", track=9)
    cfg = eng.config(sensors=["wheel_v", "wheel_w", "gyro", "lights"], motion_identity=False)
    tracked, unknown = eng.run_engine(runs, None, cfg, lights=fake)
    assert not tracked["A-1"].fixes and not tracked["B-1"].fixes
    assert [t["name"] for t in unknown.tracks] == ["B"]


def test_unchanged_without_the_sensor():
    runs, ta, tb = two_robots()
    dets = sightings(ta, seed=1) + sightings(tb, seed=2)
    blink = blink_rows(ta, "A", track=1) + blink_rows(tb, "B", track=2)
    lights = light_rows(ta, "A", track=1) + light_rows(tb, "B", track=2, seed=4)
    for sensors in (["wheel_v", "wheel_w", "gyro", "camera"], ["wheel_v", "wheel_w", "gyro", "camera", "blink"]):
        cfg = eng.config(sensors=sensors, motion_identity=True)
        a, _ = eng.run_engine(runs, dets, cfg, blink=blink)
        b, _ = eng.run_engine(runs, dets, cfg, blink=blink, lights=lights)    # given, but not a sensor of this setup
        for rid in a:
            assert a[rid].trajectory() == b[rid].trajectory()


def test_blink_and_lights_together():
    runs, ta, tb = two_robots()
    blink = blink_rows(ta, "A", track=1) + blink_rows(tb, "B", track=2)
    lights = light_rows(ta, "A", track=1) + light_rows(tb, "B", track=2, seed=4)
    cfg = eng.config(sensors=["wheel_v", "wheel_w", "gyro", "blink", "lights"], motion_identity=True)
    tracked, _ = eng.run_engine(runs, None, cfg, blink=blink, lights=lights)
    for rid in ("A-1", "B-1"):
        src = tracked[rid].id_sources
        assert src.get("lights", 0) + src.get("both", 0) > 100
        assert src.get("blink", 0) + src.get("both", 0) > 100
        assert {f["kind"] for f in tracked[rid].fixes} == {"blink", "lights"}
