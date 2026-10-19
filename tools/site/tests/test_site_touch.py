"""The optional touch-probe survey step and the home dock as a landmark (tools/site), on the simulator.

    python -m pytest tools/site/tests/test_site_touch.py -q      (about 20 s)
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SITE = os.path.dirname(HERE)
sys.path.insert(0, SITE)

import pipeline as PL  # noqa: E402
import simrun as SR  # noqa: E402
import sitemap as SM  # noqa: E402
import survey as SV  # noqa: E402
import fleetmap as FM  # noqa: E402

WORLD = {"format": "alvik-fleet-map", "version": 1, "floor": {"x_min": 0, "x_max": 122, "y_min": 0, "y_max": 122},
         "boards": [{"name": "A", "p1": [100, 20], "p2": [100, 60], "thickness_cm": 1.5}],
         "robots": [{"name": "Left", "start": [60, 40, 0]}],
         "home_docks": [{"name": "h1", "x": 30.0, "y": 100.0, "heading": 90.0}]}


def test_touch_survey_pins_the_board_face():
    user = json.loads(json.dumps(WORLD))
    user["boards"][0]["p1"] = [101.0, 20.0]          # clicked 1 cm off (along the face's normal)
    user["boards"][0]["p2"] = [101.0, 60.0]
    cfg = SR.SIMCFG.load_config()
    cfg.dock.contact_boards = True                   # the bumper really touches the board
    site = SR.SimSite(FM.normalize(WORLD), cfg, seed=4, params={"sw_on": 1})
    m = FM.normalize(user)
    L = SM.landmarks_from_map(m, include=["A"])
    spots = SV.touch_viewpoints(SV.survey_map(m, "Left", others_parked=False), L)
    assert len(spots) == 4                           # both long faces, two points each
    start = m["robots"][0]["start"]
    plan, kept, _ = SV.plan_views(m, "Left", start, spots)
    steps = plan["robots"]["Left"]["steps"]
    assert sum(st["type"] == "touch" for st in steps) == 4
    run, failed, ok = site.follow("Left", start, steps)
    assert failed is None, failed
    runs = site.runs("Left")
    touches = [t for r in runs for t in r.touch]
    assert len(touches) == 4 and all(t["result"] == "touched" for t in touches)
    res = PL.solve_site(user, [], runs, site.sightings, ["Left"])
    assert len(res.prob.touches) == 4
    est = res.est
    # the face at x = 100 - 0.75 (truth), clicked 1 cm off. A touch measures the face against the robot's
    # station pose, which the camera gives to ~0.1-0.2 cm: that, not the touch, limits the absolute position
    err = [abs(est.anchor("A", a)[0] - 99.25) for a in ("left p1", "left p2")]
    assert max(err) < 0.3, err
    # without the touches the solve has only the camera and the front sensor (and its range bias) at 3 cm
    for r in runs:
        r.touch = []
    res0 = PL.solve_site(user, [], runs, site.sightings, ["Left"])
    assert len(res0.prob.touches) == 0
    err0 = [abs(res0.est.anchor("A", a)[0] - 99.25) for a in ("left p1", "left p2")]
    print("\nface error with touches %.3f / %.3f cm, without %.3f / %.3f cm" % (*err, *err0))


def test_home_dock_is_a_landmark_in_the_locked_map():
    m = FM.normalize(WORLD)
    L = SM.landmarks_from_map(m)
    dock = [l for l in L if l.kind == "dock"]
    assert [l.name for l in dock] == ["dock h1"]
    d = dock[0]
    assert np.allclose(d.center, [30.0, 100.0])
    moved = d.moved_shape(np.array([0.5, -0.2, 1.0, 0.0]))
    assert moved["x"] == 30.5 and moved["y"] == 99.8 and moved["heading"] == 91.0
    est = SM.Estimate(L, [], cam_sys_cm=0.1)
    est.g["dock h1"] = np.array([0.5, -0.2, 1.0, 0.0])
    lm = SM.locked_map(m, est, 1, audit=[], camera={}, survey={})
    assert lm["home_docks"][0]["x"] == 30.5 and lm["home_docks"][0]["heading"] == 91.0
    assert "dock h1" in lm["site"]["landmarks"]
