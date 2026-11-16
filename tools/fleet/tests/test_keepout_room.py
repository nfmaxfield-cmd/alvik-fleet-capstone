"""(2026-11-16, Nick: school Visit points ran into the outlet plates) The paths keep the edge rule's room from a site's
keep-outs: the robot's body and the planner's 1.5 cm, plus the measured track error and stopping distance. The stickers
stay where they are; MIRLT_KEEPOUT_ROOM=body is the way before."""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
sys.path.insert(0, FLEET)

import production as PR  # noqa: E402
import safety_rules as SR  # noqa: E402
import sites as SI  # noqa: E402


def _school():
    return SI.load("school-conference")


def test_room_is_the_edge_rules_terms_without_the_body():
    want = SR.TRACK_P95_CM + SR.stop_cm(None) + SR.tracking_bias_cm("tags")
    assert abs(PR.keepout_room_cm(_school()) - want) < 0.01
    assert 3.5 < PR.keepout_room_cm(_school()) < 5.0


def test_widened_polygon_keeps_the_room_all_round():
    sq = [[0, 0], [10, 0], [10, 6], [0, 6]]
    w = PR.widen_polygon(sq, 4.0)
    import geometry as G
    # every point of the grown outline is at least 4 cm from the measured one (never inside the true offset)
    d = G.points_to_polygon(np.array(w, float), np.array(sq, float))
    assert float(d.min()) >= 4.0 - 1e-6
    assert float(d.max()) < 4.0 / math.cos(math.pi / 16) + 0.01     # (outline rounded to 0.01 cm)


def test_school_map_plans_round_the_widened_plates_and_keeps_the_stickers():
    m, words, _p, _o = PR.site_production(_school())
    names = [k["name"] for k in m["keepouts"]]
    assert {"middle outlet box", "zone 1 outlet plate", "zone 4 outlet plate"} <= set(names)
    measured = {k["name"]: k["polygon"] for k in PR.site_keepouts(_school())}
    for k in m["keepouts"]:
        assert "paths keep 4.1 cm more" in k.get("note", "")
        xs = [p[0] for p in k["polygon"]]
        mx = [p[0] for p in measured[k["name"]]]
        assert min(xs) < min(mx) - 4.0 and max(xs) > max(mx) + 4.0
    assert any("keep-outs (" in w for w in words)
    import sticker_points as SP
    pts, _z, _w = SP.points_cm(_school())
    for lab in ("P10", "P12", "P32"):     # the stickers nearest the plates are not moved
        assert np.allclose(m["spots"][lab][:2], pts[lab][:2], atol=0.01), lab


def test_every_sticker_keeps_turn_room_from_the_widened_plates():
    import fleetmap as FM
    import planner as PL
    m, _w, _p, _o = PR.site_production(_school())
    world, thr = FM.World(m), PL.Planner(m).grid.turn_thr
    tight = [lab for lab, p in m["spots"].items() if float(world.clearance(np.array([p[:2]]))[0]) < thr]
    assert tight == []


def test_old_way_behind_the_switch(monkeypatch):
    monkeypatch.setenv("MIRLT_KEEPOUT_ROOM", "body")
    assert PR.keepout_room_cm(_school()) == 0.0
    m, _w, _p, _o = PR.site_production(_school())
    measured = {k["name"]: k["polygon"] for k in PR.site_keepouts(_school())}
    assert all(np.allclose(k["polygon"], measured[k["name"]]) for k in m["keepouts"])
