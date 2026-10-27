"""School demo engine (docs/build-plans/mission-control.md, build 2026-10-27): re-plans, the general workspace map,
warm-up, brick watch and run modes."""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
if FLEET not in sys.path:
    sys.path.insert(0, FLEET)

import fleetmap as FM        # noqa: E402
import planner as P          # noqa: E402

DEMO = os.path.join(FLEET, "examples", "demo_map.json")


@pytest.fixture(scope="module")
def demo():
    m = FM.load(DEMO)
    return m, P.make_plan(m)


# ---------------------------------------------------------------------------- item 1: re-plans

def test_settings_parse_booleans_from_words():
    assert P.settings({"replan_first_safe": "false"}).replan_first_safe is False
    assert P.settings({"replan_first_safe": "1"}).replan_first_safe is True


def test_carry_on_horizon_is_the_first_safe_budget_only_when_first_safe():
    assert P.carry_on_s(P.settings()) == pytest.approx(0.5)
    assert P.carry_on_s(P.settings({"replan_first_safe": False})) == pytest.approx(2.0)


def test_first_safe_replan_stops_at_its_first_plan_and_is_safe(demo):
    m, plan = demo
    t = 0.5 * plan["summary"]["makespan_s"]
    ob = {"id": "b", "shape": "circle", "center": [61.0, 61.0], "radius": 6.0, "source": "camera"}
    states = P.states_at(plan, t)
    new, _ = P.obstacle_event(m, plan, states, [ob])
    assert new["summary"]["first_safe_only"] is True
    assert new["summary"]["orders_tried"] <= 3          # the first safe order (each order tries up to 3 holds)
    m2 = FM.normalize(dict(m, obstacles=[ob]))
    rep = P.verify(m2, new)
    assert rep["min_separation_cm"] >= 5.0 and rep["min_static_clearance_cm"] >= -1e-6
    for r in new["robots"].values():                     # new steps wait only for the short horizon
        for st in r["steps"]:
            if not st.get("committed") and st["type"] != "wait":
                assert st["t_start"] >= 0.5 - 1e-6
                break


def test_old_search_still_available(demo):
    m, plan = demo
    t = 0.5 * plan["summary"]["makespan_s"]
    ob = {"id": "b", "shape": "circle", "center": [61.0, 61.0], "radius": 6.0, "source": "camera"}
    over = dict(plan["settings"], replan_first_safe=False)
    new, _ = P.obstacle_event(m, plan, P.states_at(plan, t), [ob], over=over)
    assert new["summary"]["first_safe_only"] is False
    assert new["summary"]["orders_tried"] > 3


# ---------------------------------------------------------------------------- item 2: general workspace map

import numpy as np            # noqa: E402
import workspace as WS        # noqa: E402

ROBOTS3 = [("Left", [30, 20, 90]), ("Center", [76, 20, 90]), ("Right", [122, 20, 90])]


def school(sides=None):
    m = WS.blank_map((0, 0, 152, 76), ROBOTS3)
    for k, v in (sides or {}).items():
        m = WS.set_side(m, k, *v) if isinstance(v, tuple) else WS.set_side(m, k, v)
    return m


def test_map_without_sides_keeps_the_old_meaning():
    m = FM.load(DEMO)
    assert "sides" not in m
    w = FM.World(m)
    assert w.sides is None
    assert float(w.edge_dist(np.array([[10.0, 50.0]]))[0]) == pytest.approx(10.0)


def test_open_parts_count_the_margin_barrier_parts_do_not():
    m = school({"x_min": "barrier", "x_max": "open", "y_min": ("partial", [[0, 60]]), "y_max": "unknown"})
    w = FM.World(m)
    om = m["open_margin_cm"]
    assert float(w.edge_dist(np.array([[10.0, 38.0]]))[0]) == pytest.approx(10.0)          # barrier at x = 0
    assert float(w.edge_dist(np.array([[142.0, 38.0]]))[0]) == pytest.approx(10.0 - om)    # open at x = 152
    assert float(w.edge_dist(np.array([[30.0, 8.0]]))[0]) == pytest.approx(8.0)            # barrier part of y = 0
    assert float(w.edge_dist(np.array([[100.0, 8.0]]))[0]) == pytest.approx(8.0 - om)      # open part of y = 0
    assert float(w.edge_dist(np.array([[100.0, 70.0]]))[0]) == pytest.approx(6.0 - om)     # unknown = open
    assert float(w.edge_dist(np.array([[-1.0, 38.0]]))[0]) < 0                              # off the table


def test_side_parts_split_a_partial_side():
    m = school({"y_min": ("partial", [[20, 50], [90, 120]])})
    bar, opn = FM.side_parts(m, "y_min")
    assert bar == [[20, 50], [90, 120]] and opn == [[0, 20], [50, 90], [120, 152]]


def test_bad_sides_are_refused():
    with pytest.raises(FM.MapError):
        school({"y_min": ("partial", [])})
    with pytest.raises(FM.MapError):
        FM.normalize({"sides": {"left": "open"}})


def test_spots_and_waits_become_planner_tasks():
    m = school({k: "barrier" for k in FM.SIDES})
    m, a = WS.add_spot(m, (40, 50))
    m, b = WS.add_spot(m, (110, 50), heading=0)
    assert (a, b) == ("A", "B")
    m["tasks"] = {"Center": {"visit": [{"spot": "A"}, {"wait_s": 3}, {"spot": "B"}], "park": True}}
    m = FM.normalize(m)
    v = m["tasks"]["Center"]["visit"]
    assert v[0] == {"goto": [40.0, 50.0], "spot": "A"} and v[1] == {"wait_s": 3.0}
    plan = P.make_plan(m)
    st = plan["robots"]["Center"]["steps"]
    pauses = [s for s in st if s["phase"] == "pause"]
    assert len(pauses) == 1 and pauses[0]["t_end"] - pauses[0]["t_start"] == pytest.approx(3.0, abs=0.11)
    with pytest.raises(FM.MapError):
        FM.normalize(dict(m, tasks={"Center": [{"spot": "Z"}]}))


def test_plan_keeps_off_open_edges_by_the_margin():
    m = school({"x_min": "open", "x_max": "open", "y_min": "open", "y_max": "open"})
    m, _ = WS.add_spot(m, (6, 38))                       # a spot too near the open left side
    m["tasks"] = {"Left": {"visit": [{"spot": "A"}]}}
    with pytest.raises(P.PlanError):
        P.make_plan(FM.normalize(m))
    m2 = WS.set_side(m, "x_min", "barrier")               # the same spot by a barrier is reachable... if there is room
    pl = P.Planner(m2)
    assert pl.grid.turnable([6 + pl.s.safety_margin_cm + pl.body.r_turn, 38])


def test_nogo_zones_are_kept_out_until_deleted():
    m = school({k: "barrier" for k in FM.SIDES})
    m, nm = WS.add_nogo(m, WS.circle_poly((76, 45), 8), source="warm-up bump", note="bumped by Left")
    assert WS.nogo_zones(m, "proposed")[0]["name"] == nm
    pl = P.Planner(m)
    assert not pl.grid.turnable([76, 45])
    m = WS.confirm_nogo(m, nm)
    assert WS.nogo_zones(m, "confirmed")
    m = WS.delete_nogo(m, nm)
    assert not WS.nogo_zones(m) and P.Planner(m).grid.turnable([76, 45])
    h1 = FM.map_hash(WS.add_nogo(m, [[0, 0], [5, 0], [5, 5]], status="proposed")[0])
    h2 = FM.map_hash(WS.add_nogo(m, [[0, 0], [5, 0], [5, 5]], status="confirmed")[0])
    assert h1 == h2                                       # confirming does not change the paths


def test_runner_board_from_the_workspace():
    m = school({"x_min": "barrier", "x_max": "open", "y_min": "open", "y_max": "partial" and "open"})
    b = WS.to_board(m)
    assert sorted(b.open_edges()) == [("x", 152.0), ("y", 0.0), ("y", 76.0)]
    assert "x" in b.missing and "y" not in b.missing
    import e02_plan as EP
    b2 = EP.Board.from_dict(b.to_dict())
    assert sorted(b2.open_edges()) == sorted(b.open_edges())
    old = EP.Board()
    assert old.sides is None and len(old.open_edges()) == 2       # the two-board layout is unchanged
    allbar = WS.to_board(school({k: "barrier" for k in FM.SIDES}))
    assert allbar.open_edges() == [] and allbar.edge_room(50, 30)[0] == float("inf")


def test_firmware_params_cover_all_four_sides():
    m = school({"x_min": "barrier", "x_max": "open", "y_min": ("partial", [[0, 60]]), "y_max": "unknown"})
    p = WS.firmware_params(m, 6.0)
    assert p["bd_on"] == 1 and p["border_cm"] == 6.0
    assert (p["bd_k_x0"], p["bd_k_x1"], p["bd_k_y0"], p["bd_k_y1"]) == (0, 1, 2, 1)
    assert (p["bd_lo_y0"], p["bd_hi_y0"]) == (60.0, 152.0)
    assert p["bd_x1"] == 152.0 and p["bd_y1"] == 76.0
    assert any("partly barrier" in w for w in WS.words(m))


# ---------------------------------------------------------------------------- item 3: firmware border + WiFi

import shutil       # noqa: E402
import subprocess   # noqa: E402

REPO = os.path.dirname(os.path.dirname(FLEET))
FW = os.path.join(REPO, "firmware", "home_lab")


@pytest.mark.skipif(shutil.which("g++") is None, reason="needs g++")
def test_firmware_border_host_test_passes():
    r = subprocess.run(["sh", os.path.join(REPO, "tools", "fwcheck", "border_test.sh")], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert r.stdout.count("ok ") == 11


def test_firmware_version_and_second_network_placeholders():
    ino = open(os.path.join(FW, "home_lab.ino")).read()
    assert '#define FW_VERSION "2026-10-27a"' in ino
    ex = open(os.path.join(FW, "secrets.example.h")).read()
    assert "// #define WIFI_SSID2" in ex and "// #define WIFI_PASSWORD2" in ex      # placeholders, commented out
    net = open(os.path.join(FW, "net.h")).read()
    assert "#ifdef WIFI_SSID2" in net and "beginNet(netIndex)" in net


def test_every_border_param_exists_in_the_firmware():
    sys.path.insert(0, os.path.join(REPO, "tools", "sim"))
    import config
    names = {n for n, _ in config.firmware_params()}
    m = school({"y_min": ("partial", [[0, 60]]), "x_min": ("partial", [[10, 20]]), "x_max": ("partial", [[0, 5]]),
                "y_max": ("partial", [[100, 120]])})
    import run_plan as RP
    for c in RP.border_commands(m, 6.0):
        assert c.split()[1] in names, c
    assert RP.border_commands(m, 6.0)[-1] == "set bd_on 1"


def test_simulated_border_twin_matches_the_firmware_cases():
    sys.path.insert(0, os.path.join(REPO, "tools", "sim"))
    import fw_follow

    cls = fw_follow.FollowMixin
    pv = {"bd_on": 1, "bd_x0": 0, "bd_x1": 152, "bd_y0": 0, "bd_y1": 76, "border_cm": 6, "bd_k_x0": 0, "bd_k_x1": 1,
          "bd_k_y0": 2, "bd_lo_y0": 60, "bd_hi_y0": 152, "bd_k_y1": 1}
    o = cls.__new__(cls)
    o.P = lambda k: pv.get(k, 0)
    assert o.bd_dist4(3, 38) == pytest.approx(38)
    assert o.bd_dist4(148, 38) == pytest.approx(4)
    assert o.bd_dist4(30, 3) == pytest.approx(73)
    assert o.bd_dist4(100, 3) == pytest.approx(3)
    assert o.bd_dist4(56, 3) == pytest.approx(3)
    pv["bd_on"] = 0
    assert o.bd_dist4(148, 38) is None


# ---------------------------------------------------------------------------- item 4: warm-up

import warmup as WU   # noqa: E402


def test_warmup_tasks_cover_every_corner_and_side_and_plan_safely():
    m = school()
    wm = WU.plan_tasks(m)
    labels = [v.get("label") for t in wm["tasks"].values() for v in t["visit"]]
    for c in ("bottom-left corner", "top-left corner", "bottom-right corner", "top-right corner"):
        assert labels.count(c) == 2                    # the goto and its pause
    feels = [v for t in wm["tasks"].values() for v in t["visit"] if "probe" in v]
    assert len(feels) == 2 * 4 + 2 * 2                 # 152 cm sides: 4 feels, 76 cm sides: 2
    plan = P.make_plan(wm)
    st = [s for r in plan["robots"].values() for s in r["steps"]]
    probes = [s for s in st if s["phase"] == "probe"]
    assert len(probes) == 12 and all(s["front_min_cm"] == 3.0 and s["speed"] == 3.0 for s in probes)
    assert all(s.get("label") for s in probes)
    assert sum(1 for s in st if s["phase"] == "pause" and s.get("label", "").endswith("corner")) == 4
    assert plan["summary"]["min_separation_cm"] >= 5.0


def _rec(state, info, pose, cam=None, robot="Left", t=10.0):
    return {"state": state, "info": info, "pose": pose, "cam": cam, "robot": robot, "t_run": t, "front_cm": 3.0}


def test_probe_answers():
    m = school()
    # open: the edge guard tripped with the centre 3.6 cm in from x = 152, facing it -> the edge 0.1 cm past the line
    r = WU.classify_probe(m, "x_max", 38, _rec("failed", "plywood edge at 148.4 38.0 0.0 line", [148.4, 38, 0]))
    assert r["kind"] == "open" and r["offset_cm"] == pytest.approx(-0.1, abs=0.05)
    # barrier: the front sensor met something 2 cm ahead with the centre 6 cm in (front face 1.25 cm in)
    r = WU.classify_probe(m, "y_min", 50, _rec("failed", "obstacle 2.0 cm ahead; zones 2 2 2 2 2", [50, 6, -90]))
    assert r["kind"] == "barrier" and r["offset_cm"] == pytest.approx(0.75, abs=0.1)
    # an object 20 cm in from the side
    r = WU.classify_probe(m, "y_min", 50, _rec("failed", "obstacle 2.5 cm ahead; zones 3 3 2.5 3 3", [50, 27, -90]))
    assert r["kind"] == "object" and r["offset_cm"] < -15
    assert WU.classify_probe(m, "y_min", 50, _rec("done", "", [50, 1, -90]))["kind"] == "none"
    assert WU.classify_probe(m, "y_min", 50, None)["kind"] == "unfelt"


def test_side_kinds_from_feels():
    m = school()
    pr = [{"side": "y_min", "along": a, "kind": k} for a, k in ((19, "barrier"), (57, "barrier"), (95, "open"),
                                                                 (133, "open"))]
    pr += [{"side": "x_min", "along": a, "kind": "open"} for a in (19, 57)]
    pr += [{"side": "x_max", "along": a, "kind": "barrier"} for a in (19, 57)]
    k = WU.side_kinds(m, pr)
    assert k["y_min"] == {"kind": "partial", "barrier": [[0.0, 77.0]], "source": "warm-up"}
    assert k["x_min"]["kind"] == "open" and k["x_max"]["kind"] == "barrier" and k["y_max"]["kind"] == "unknown"
    m2 = WU.apply(m, {"sides": k, "where": {"x_max": {"offset_cm": 2.0}}, "zones": []}, move_sides=True)
    assert m2["floor"]["x_max"] == pytest.approx(154.0) and m2["sides"]["y_min"]["kind"] == "partial"


def test_mapping_view_draws_tracks_and_contacts_by_kind():
    import mapview as MV
    m = school({"x_min": "barrier", "x_max": "open", "y_min": ("partial", [[0, 70]]), "y_max": "unknown"})
    res = {"sides": m["sides"], "zones": [{"polygon": [[50, 50], [58, 50], [58, 58]]}],
           "contacts": [{"robot": "Left", "kind": "open", "point": [151, 30]},
                        {"robot": "Left", "kind": "barrier", "point": [1, 30]},
                        {"robot": "Right", "kind": "object", "point": [54, 54]}]}
    s = MV.svg(m, res, {"Left": [(30, 18), (40, 30), (50, 30)], "Right": [(120, 18), (120, 40)]})
    assert s.count("<polyline") == 2 and "#2952CC" in s and "#7A3EC8" in s
    assert s.count('r="9"') == 3 and 'stroke-dasharray="3 3"' in s and 'r="4" fill="#D92D20"' in s
    assert "<polygon" in s


# ---------------------------------------------------------------------------- item 5: brick drop

def _table_cam(tmp_path):
    sys.path.insert(0, os.path.join(REPO, "tools", "sim"))
    import table_cam as TC
    import e02_camera as CAM
    tc = TC.TableCamera((0, 0, 152, 76))
    cam = CAM.load_camera(tc.true_model(str(tmp_path / "cam.json")))
    return tc, cam


def _brick(cx, cy, ang=30.0, L=20.0, W=9.0):
    a = math.radians(ang)
    c, s_ = math.cos(a), math.sin(a)
    return [[cx + c * u - s_ * v, cy + s_ * u + c * v] for u, v in ((-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2),
                                                                     (-L / 2, W / 2))]


import math   # noqa: E402


def test_brick_watch_ignores_hands_and_moving_robots_and_confirms_a_still_brick(tmp_path):
    import brickwatch as BW
    tc, cam = _table_cam(tmp_path)
    robots = [(30.0, 18.0, 1.57), (76.0, 18.0, 1.57), (122.0, 18.0, 1.57)]
    sent = []
    w = BW.BrickWatch(cam, (0, 0, 152, 76), send=sent.append)
    w.start([tc.render(robots) for _ in range(5)], [(x, y) for x, y, _ in robots])
    t = 0.0
    brick = _brick(90.0, 45.0)
    # the robots drive about: nothing is reported
    for k in range(12):
        t += 0.25
        rb = [(30 + 3 * k, 18 + 2 * k, 0.6), (60.0, 18.0 + 3 * k, 1.57), (122.0 - 3 * k, 18.0, 3.14)]
        assert w.check(tc.render(rb), [(x, y) for x, y, _ in rb], t) == []
    # a hand comes in from the top side and puts the brick down: ignored while it is there
    for k in range(6):
        t += 0.25
        assert w.check(tc.render(rb, boxes=[brick], hand=((90.0, 45.0), (90.0, 130.0))), [(x, y) for x, y, _ in rb],
                       t) == []
    assert w.hand_seen >= 6
    # the hand has gone: the brick is confirmed after CONFIRM_CHECKS still checks, once
    got = []
    for k in range(6):
        t += 0.25
        got += w.check(tc.render(rb, boxes=[brick]), [(x, y) for x, y, _ in rb], t)
    assert len(got) == 1 and len(sent) == 1
    ob = got[0]
    assert ob["type"] == "obstacle" and ob["source"] == "camera" and ob["shape"] == "polygon"
    cx = sum(p[0] for p in ob["points"]) / len(ob["points"])
    cy = sum(p[1] for p in ob["points"]) / len(ob["points"])
    assert math.hypot(cx - 90.0, cy - 45.0) < 3.0
    import geometry as G2
    import numpy as np2
    covered = G2.points_in_polygon(np2.array(brick), np2.array(ob["points"]))
    d_out = [float(G2.points_to_polygon(np2.array([p]), np2.array(ob["points"]))[0]) for p, c in zip(brick, covered) if not c]
    assert covered.all(), d_out                            # the brick's footprint is inside the reported outline


# ---------------------------------------------------------------------------- item 6: run modes

import missions as MS   # noqa: E402

SCHOOL = os.path.join(FLEET, "examples", "school_map.json")


def test_instruction_lists():
    spots = {"A": [10, 10], "B": [20, 20], "C": [30, 30]}
    v = MS.instructions("go to A · wait 5 s · go to C · repeat 3x", spots)
    assert v == [{"spot": "A"}, {"wait_s": 5.0}, {"spot": "C"}] * 3
    assert MS.instructions("go to b; wait 2", spots) == [{"spot": "B"}, {"wait_s": 2.0}]
    with pytest.raises(ValueError):
        MS.instructions("go to Z", spots)
    with pytest.raises(ValueError):
        MS.instructions("dance", spots)


def test_mow_lanes_cover_the_table_and_slide_round_a_brick():
    m = FM.load(SCHOOL)
    v, notes = MS.mow(m)
    ys = sorted({p["goto"][1] for p in v})
    assert ys[0] == pytest.approx(MS.least_open_room(m) + 1.0, abs=0.1) and all(b - a <= 12.0 + 1e-6 for a, b in zip(ys, ys[1:]))
    assert any("not 10" in n for n in notes)              # the inset raised to the planner's room from open sides
    # a brick on a lane end: the end slides along its lane, the plan still covers every lane
    m2 = dict(m, robots=[r for r in m["robots"] if r["name"] == "Center"],
              tasks={"Center": {"visit": v, "park": False}})
    end = v[1]["goto"]
    m2["obstacles"] = [{"id": "b", "shape": "circle", "center": [end[0], end[1]], "radius": 5.0, "source": "camera"}]
    plan = P.make_plan(m2)
    assert plan["summary"]["min_static_clearance_cm"] >= -1e-6
    lane_ys = {round(st["to"][1], 1) for st in plan["robots"]["Center"]["steps"] if st["type"] == "drive"}
    for y in ys:
        assert any(abs(y - ly) < 0.2 for ly in lane_ys), y


def test_chicken_swerves_at_15_and_never_under_8():
    m = FM.load(SCHOOL)
    m = dict(m, robots=[{"name": "Left", "start": [20, 38, 0]}, {"name": "Center", "start": [140, 62, 0]},
                        {"name": "Right", "start": [132, 38, 180]}])
    ch = MS.chicken(m)
    sw = [v["goto"] for v in ch["Left"] if v.get("label", "").endswith("swerve")]
    swr = [v["goto"] for v in ch["Right"] if v.get("label", "").endswith("swerve")]
    front_gap = math.dist(sw[0], swr[0]) - 2 * 4.75
    assert front_gap == pytest.approx(15.0, abs=0.01) and len(sw) == 2
    m["tasks"] = {k: {"visit": v, "park": True} for k, v in ch.items()}
    m["mission"] = {"planner": {"separation_cm": MS.SETTINGS["chicken_min_cm"]}}
    plan = P.make_plan(m)
    assert plan["summary"]["min_separation_cm"] >= 8.0 - 1e-6


def test_show_keeps_its_beats():
    m = FM.load(SCHOOL)
    sh, end, figs = MS.show(m)
    mm = dict(m, tasks={k: {"visit": v, "park": True} for k, v in sh.items()},
              mission={"planner": {"separation_cm": 8.0}})
    plan = P.make_plan(mm)
    sm = MS.summary(mm, plan)
    assert sm["beats"] >= 30 and sm["beat_late_median_s"] <= 0.1 and sm["beats_late_over_0_5s"] <= 4
    assert sm["closest_cm"] >= 8.0 and [f["name"] for f in figs][0] == "Line up"


def test_mondays_routine_plans():
    m = FM.load(SCHOOL)
    mm = MS.monday(m)
    assert mm["mission"]["planner"]["priority_first"] == "Center"
    plan = P.make_plan(mm, t_limit_s=40)
    assert plan["priority_order"][0] == "Center"
    assert plan["summary"]["min_separation_cm"] >= 8.0 - 1e-6
    labels = {st.get("label") for r in plan["robots"].values() for st in r["steps"]}
    for lab in ("chicken 1 swerve", "chicken 2 swerve", "roam", "follow the leader", "mow lane 1 start", "split"):
        assert lab in labels, lab
    for r in mm["robots"]:
        last = plan["robots"][r["name"]]["steps"][-1]
        assert last["type"] == "park" and last["to"][:2] == pytest.approx(r["park"][:2], abs=0.05)
    # Left and Right park only once Center has mowed its last lane
    t_last_lane = max(st["t_end"] for st in plan["robots"]["Center"]["steps"] if "mow lane" in (st.get("label") or ""))
    for n in ("Left", "Right"):
        park_steps = [st for st in plan["robots"][n]["steps"] if st["phase"] == "park" or st["type"] == "park"]
        assert park_steps[-1]["t_start"] >= t_last_lane - 1e-6


# ---------------------------------------------------------------------------- Phase 2: Mission Control

def test_mission_control_api(tmp_path):
    import threading
    import urllib.request
    import urllib.error
    import json as J
    import mission_control as MC
    s = MC.Session(str(tmp_path / "mc"), SCHOOL)
    srv, url = MC.serve(s, port=0)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        def call(path, body=None, host="127.0.0.1"):
            req = urllib.request.Request(url.rstrip("/") + path, data=None if body is None else J.dumps(body).encode(),
                                         headers={"Content-Type": "application/json", "Host": host})
            try:
                with urllib.request.urlopen(req) as r:
                    return r.status, J.loads(r.read())
            except urllib.error.HTTPError as e:
                return e.code, J.loads(e.read())
        page = urllib.request.urlopen(url).read().decode()
        assert "Mission Control" in page and "Details" in page
        c, st = call("/api/state")
        assert c == 200 and st["map"]["floor"]["x_max"] == 152
        # every setting group is there to see and edit (Nick: nothing locked)
        assert set(st["defaults"]) == {"planner", "mission", "run", "warmup"}
        assert st["defaults"]["mission"]["chicken_swerve_cm"] == 15.0 and st["defaults"]["mission"]["chicken_min_cm"] == 8.0
        c, st = call("/api/settings", {"mission": {"chicken_rounds": 1}})
        assert c == 200 and st["settings"]["mission"]["chicken_rounds"] == 1
        c, st = call("/api/map", {"spots": {"A": [30, 40], "B": [120, 40]}})
        assert c == 200 and set(st["map"]["spots"]) == {"A", "B"}
        c, r = call("/api/mission", {"mode": "spots", "lists": {"Center": "go to A, wait 1 s, go to B"}})
        assert c == 200 and r["preview"]["makespan_s"] > 0 and "Center" in r["preview"]["frames"]
        assert r["preview"]["summary"]["closest_cm"] >= 8.0
        c, e = call("/api/mission", {"mode": "spots", "lists": {"Center": "go to Z"}})
        assert c == 400 and "no spot Z" in e["error"]
        c, _ = call("/api/map", {"spots": {}}, host="evil.example")
        assert c == 403
        c, e = call("/api/run/pause", {"on": True})
        assert c == 400                                   # nothing running
    finally:
        srv.shutdown()
