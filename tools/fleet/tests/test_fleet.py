"""Tests for the fleet planner. Run from the repo folder:

    python -m pytest tools/fleet/tests -q

They take about two minutes (several full plans and simulations).
"""
import copy
import itertools
import json
import math
import os
import sys
import threading
import urllib.error
import urllib.request

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
sys.path.insert(0, FLEET)

import fleetmap as FM        # noqa: E402
import geometry as G         # noqa: E402
import planner as P          # noqa: E402
import simulate_plan as SP   # noqa: E402

EX = os.path.join(FLEET, "examples")


def demo_map():
    return FM.load(os.path.join(EX, "demo_map.json"))


@pytest.fixture(scope="module")
def demo_plan():
    return P.make_plan(demo_map())


# ----------------------------------------------------------------------------
# Map and dock geometry
# ----------------------------------------------------------------------------

def test_dock_pose_is_1cm_from_board_facing_it():
    m = FM.normalize({"boards": [{"name": "A", "p1": [30, 2], "p2": [50, 2], "thickness_cm": 1.5}]})
    d = FM.docks(m)["A"]
    # surface at y = 2 + 0.75; front face 1 cm out; centre 4.75 behind the front face
    assert d["dock"][0] == pytest.approx(40.0)
    assert d["dock"][1] == pytest.approx(2 + 0.75 + 1.0 + 4.75)
    assert math.degrees(d["dock"][2]) == pytest.approx(-90.0)
    assert d["predock"][1] == pytest.approx(d["dock"][1] + 15.0)
    body = FM.body_of(m)
    corners = body.corners([d["dock"]])[0]
    rect = G.segment_rect([30, 2], [50, 2], 1.5)
    assert G.polygon_polygon_dist(corners, rect) == pytest.approx(1.0, abs=1e-9)


def test_dock_settings_and_sides():
    base = {"boards": [{"name": "B", "p1": [120, 40], "p2": [120, 60], "thickness_cm": 2.0, "dock_at": 0.25}],
            "robot": {"front_offset_cm": 5.5}, "dock": {"gap_cm": 0.5, "predock_cm": 20}}
    d = FM.docks(FM.normalize(base))["B"]
    assert d["side"] == "left"                       # end 1 -> end 2 runs +y; the floor's middle is to its left
    assert d["dock"][0] == pytest.approx(120 - 1.0 - 0.5 - 5.5)
    assert d["dock"][1] == pytest.approx(45.0)
    assert math.degrees(d["dock"][2]) == pytest.approx(0.0)
    assert d["predock"][0] == pytest.approx(d["dock"][0] - 20)
    m2 = copy.deepcopy(base)
    m2["boards"][0]["side"] = "right"
    assert FM.docks(FM.normalize(m2))["B"]["dock"][0] > 120


def test_map_checks_and_hash():
    with pytest.raises(FM.MapError):
        FM.normalize({"boards": [{"name": "A", "p1": [0, 0], "p2": [5, 0]}, {"name": "A", "p1": [0, 9], "p2": [5, 9]}]})
    with pytest.raises(FM.MapError):
        FM.normalize({"robots": [{"name": "Left", "start": [10, 10, 0]}], "tasks": {"Left": ["nope"]}})
    with pytest.raises(FM.MapError):
        FM.normalize({"obstacles": [{"shape": "circle", "center": [1, 1], "radius": 2, "source": "radar"}]})
    m = demo_map()
    h = FM.map_hash(m)
    m2 = copy.deepcopy(m)
    m2["tasks"]["Left"]["visit"] = ["C"]
    assert FM.map_hash(m2) == h                     # tasks do not change the hash
    m2["obstacles"].append({"id": "x", "shape": "circle", "center": [60, 60], "radius": 4, "source": "camera"})
    assert FM.map_hash(m2) != h
    assert FM.validate(m) == []


def test_floor_default_is_4ft_square():
    f = FM.normalize({})["floor"]
    assert (f["x_max"] - f["x_min"], f["y_max"] - f["y_min"]) == (122.0, 122.0)


def test_fast_clash_test_never_misses():
    body = G.Body()
    rng = np.random.default_rng(3)
    a = np.c_[rng.uniform(0, 30, 5000), rng.uniform(0, 30, 5000), rng.uniform(-3, 3, 5000)]
    b = np.c_[rng.uniform(0, 30, 5000), rng.uniform(0, 30, 5000), rng.uniform(-3, 3, 5000)]
    exact = G.body_gaps(body, a, b)
    fast = G.rects_clash_each(body, a, b, 6.0)
    assert not np.any((exact < 6.0) & ~fast)


# ----------------------------------------------------------------------------
# Single-robot routes
# ----------------------------------------------------------------------------

def one_robot_map(obstacles=(), visit=None):
    return FM.normalize({
        "boards": [{"name": "A", "p1": [100, 120], "p2": [120, 120]}],
        "obstacles": list(obstacles),
        "robots": [{"name": "Left", "start": [20, 20, 0]}],
        "tasks": {"Left": visit if visit is not None else ["A"]}})


def test_route_is_straight_drives_and_turns_and_avoids_obstacles():
    m = one_robot_map([{"id": "post", "shape": "circle", "center": [60, 60], "radius": 12, "source": "user"}])
    plan = P.make_plan(m)
    steps = plan["robots"]["Left"]["steps"]
    assert {s["type"] for s in steps} <= {"turn", "drive", "approach", "wait", "park"}
    world = FM.World(m)
    body = FM.body_of(m)
    s = plan["settings"]
    prev = FM.pose_in(plan["robots"]["Left"]["start"])
    for st in steps:
        a, b = FM.pose_in(st["from"]), FM.pose_in(st["to"])
        assert math.dist(prev[:2], a[:2]) < 0.05                   # steps join up
        if st["type"] == "drive":
            assert abs(G.wrap(a[2] - b[2])) < 1e-6                  # heading fixed while driving
            d = math.atan2(b[1] - a[1], b[0] - a[0])
            if st["reverse"]:
                d = G.wrap(d + math.pi)
            assert abs(G.wrap(d - a[2])) < math.radians(0.6)        # drives where it faces
            if st["phase"] == "travel":
                c = world.shape_clearance(body.drive_sweep(a[:2], b[:2], a[2]))
                assert c >= s["safety_margin_cm"] - 1e-6
        if st["type"] in ("turn", "park"):
            assert math.dist(a[:2], b[:2]) < 1e-6                   # turns in place
            assert float(world.clearance(np.array([a[:2]]))[0]) >= body.r_turn + s["safety_margin_cm"] - 1e-6
        prev = b
    turns = [st for st in steps if st["type"] == "turn" and st["phase"] == "travel"]
    assert len(turns) <= 6                                          # few waypoints after smoothing
    rep = plan["summary"]
    assert rep["planned_dock_gaps_cm"] == [pytest.approx(1.0, abs=1e-6)]
    last = steps[-1]
    assert last["type"] == "park" and last["to"] == pytest.approx([20, 20, 0], abs=0.05)


def test_approach_is_flagged_for_the_distance_sensor():
    plan = P.make_plan(one_robot_map())
    ap = [s for s in plan["robots"]["Left"]["steps"] if s["type"] == "approach"]
    assert len(ap) == 1
    a = ap[0]
    assert a["stop_at_front_cm"] == 1.0 and a["speed"] == plan["settings"]["approach_cms"]
    assert math.dist(a["from"][:2], a["to"][:2]) == pytest.approx(15.0, abs=1e-6)
    assert a["max_travel_cm"] > 15.0
    phases = [s["phase"] for s in plan["robots"]["Left"]["steps"]]
    i = phases.index("approach")
    assert phases[i - 1] == "align" and phases[i + 1] == "dwell" and phases[i + 2] == "backout"


def test_unreachable_board_is_reported():
    m = one_robot_map([{"id": "wall", "shape": "polygon", "points": [[85, 95], [122, 95], [122, 100], [85, 100]],
                        "source": "user"},
                       {"id": "wall2", "shape": "polygon", "points": [[85, 95], [90, 95], [90, 122], [85, 122]],
                        "source": "user"}])
    with pytest.raises(P.PlanError) as e:
        P.make_plan(m)
    assert "board A" in str(e.value)


# ----------------------------------------------------------------------------
# Several robots
# ----------------------------------------------------------------------------

def concurrent_sweeps_meet(plan, m):
    """Independent check of the planner's rule: no two robots ever do, at the
    same time, steps whose swept areas come within the separation."""
    body = FM.body_of(m)
    s = P.settings(plan["settings"])
    thr = P._thr(body, s)
    occ = {}
    for n, r in plan["robots"].items():
        lst = []
        for st in r["steps"]:
            p = P.step_to_prim(st)
            p["dur"] = int(round((st["t_end"] - st["t_start"]) / s.tick_s))
            lst.append(P.Occ(int(round(st["t_start"] / s.tick_s)), int(round(st["t_end"] / s.tick_s)),
                             P._sweep_samples(p, s), body, None))
        if r["steps"]:
            fin = r["steps"][-1]
            lst.append(P.Occ(lst[-1].e, P.INF_TICK, np.array([FM.pose_in(fin["to"])]), body, None))
        occ[n] = lst
    bad = []
    for a, b in itertools.combinations(occ, 2):
        for A in occ[a]:
            for B in occ[b]:
                if A.s <= B.e and B.s <= A.e and P.sweeps_meet(A, B, body, thr):
                    bad.append((a, b, A.s, B.s))
    return bad


def test_demo_plan_keeps_robots_apart(demo_plan):
    m = demo_map()
    sm = demo_plan["summary"]
    assert sm["min_separation_cm"] >= demo_plan["settings"]["separation_cm"]
    assert sm["min_static_clearance_cm"] >= demo_plan["settings"]["safety_margin_cm"] - 1e-6
    assert sm["approach_min_clearance_cm"] > 0
    assert concurrent_sweeps_meet(demo_plan, m) == []
    for n, r in demo_plan["robots"].items():
        boards = [s["board"] for s in r["steps"] if s["type"] == "approach"]
        assert boards == [v for v in r["visit_order"] if v != "park"]
        assert r["steps"][-1]["type"] == "park"


def test_step_order_rules_point_backwards_in_time(demo_plan):
    for n, r in demo_plan["robots"].items():
        for st in r["steps"]:
            for other, j in st["after"]:
                assert demo_plan["robots"][other]["steps"][j]["t_end"] <= st["t_start"] + 1e-6


@pytest.mark.parametrize("case", ["x", "swap"])
def test_three_robots_crossing_routes(case):
    """x: three routes that cross near the middle of the floor. swap: Left and
    Right trade places head-on along the same line while Center crosses it;
    each goal is the other's start, so someone has to step aside."""
    if case == "x":
        robots = [{"name": "Left", "start": [20, 35, 45]}, {"name": "Center", "start": [61, 15, 90]},
                  {"name": "Right", "start": [102, 35, 135]}]
        goals = {"Left": [100, 95], "Center": [61, 105], "Right": [22, 95]}
    else:
        robots = [{"name": "Left", "start": [20, 61, 0]}, {"name": "Center", "start": [61, 20, 90]},
                  {"name": "Right", "start": [102, 61, 180]}]
        goals = {"Left": [102, 61], "Center": [61, 102], "Right": [20, 61]}
    m = FM.normalize({"robots": robots,
                      "tasks": {n: {"visit": [{"goto": g}], "park": False} for n, g in goals.items()}})
    plan = P.make_plan(m)
    assert plan["summary"]["min_separation_cm"] >= 5.0
    assert concurrent_sweeps_meet(plan, m) == []
    deps = sum(len(s["after"]) for r in plan["robots"].values() for s in r["steps"])
    assert deps > 0                                  # someone has to give way to someone
    for n, g in goals.items():
        assert plan["robots"][n]["steps"][-1]["to"][:2] == pytest.approx(g, abs=0.05)
    sim = SP.Sim(m, copy.deepcopy(plan), dict(SP.DEFAULT_SIM), seed=5, verbose=False)
    res = sim.run_sim()
    assert res["finished"] and res["min_body_gap_cm"] >= 5.0


def test_robots_too_close_to_turn_make_room():
    """Side by side, 16 cm apart: neither can turn in place without coming
    within the separation of the other, and each has to pass the other.
    One drives a few cm straight first to make room."""
    m = FM.normalize({"robots": [{"name": "Left", "start": [55, 60, 90]}, {"name": "Right", "start": [71, 60, 90]}],
                      "tasks": {"Left": {"visit": [{"goto": [105, 60]}], "park": False},
                                "Right": {"visit": [{"goto": [20, 60]}], "park": False}}})
    plan = P.make_plan(m)
    phases = [s["phase"] for r in plan["robots"].values() for s in r["steps"]]
    assert "escape" in phases
    assert plan["summary"]["min_separation_cm"] >= 5.0
    assert concurrent_sweeps_meet(plan, m) == []


def test_shared_board_is_used_one_robot_at_a_time():
    m = FM.normalize({
        "boards": [{"name": "A", "p1": [51, 2], "p2": [71, 2]}],
        "robots": [{"name": "Left", "start": [25, 60, 0]}, {"name": "Right", "start": [97, 60, 180]}],
        "tasks": {"Left": ["A"], "Right": ["A"]}})
    plan = P.make_plan(m)
    spans = []
    for n, r in plan["robots"].items():
        ph = [s for s in r["steps"] if s.get("board") == "A" and s["phase"] in ("approach", "dwell", "backout")]
        spans.append((ph[0]["t_start"], ph[-1]["t_end"]))
    spans.sort()
    assert spans[0][1] <= spans[1][0]                # never docked at the same time
    assert plan["summary"]["min_separation_cm"] >= 5.0


def test_any_order_picks_the_quicker_order():
    m = FM.normalize({
        "boards": [{"name": "Far", "p1": [100, 120], "p2": [120, 120]}, {"name": "Near", "p1": [2, 30], "p2": [2, 50]}],
        "robots": [{"name": "Left", "start": [20, 20, 90]}],
        "tasks": {"Left": {"visit": ["Far", "Near"], "any_order": False, "park": False}}})
    fixed = P.make_plan(m)
    m["tasks"]["Left"]["any_order"] = True
    free = P.make_plan(m)
    assert fixed["robots"]["Left"]["visit_order"] == ["Far", "Near"]
    assert free["robots"]["Left"]["visit_order"] == ["Near", "Far"]
    assert free["summary"]["makespan_s"] < fixed["summary"]["makespan_s"]


def test_park_test_returns_home():
    m = FM.load(os.path.join(EX, "park_test_map.json"))
    plan = P.make_plan(m)
    assert plan["summary"]["min_separation_cm"] >= 5.0
    for r in m["robots"]:
        last = plan["robots"][r["name"]]["steps"][-1]
        assert last["type"] == "park"
        assert last["to"] == pytest.approx(r["park"], abs=0.05)


def test_plan_json_format(demo_plan, tmp_path):
    p = tmp_path / "plan.json"
    P.save_plan(demo_plan, str(p))
    back = json.loads(p.read_text())
    assert back["format"] == "alvik-fleet-plan" and back["version"] == 1
    assert back["map_hash"] == FM.map_hash(demo_map())
    for k in ("settings", "priority_order", "docks", "summary", "robots", "plan_id"):
        assert k in back
    st = back["robots"]["Left"]["steps"][0]
    for k in ("i", "type", "phase", "from", "to", "speed", "t_start", "t_end", "move_s", "board", "after"):
        assert k in st


# ----------------------------------------------------------------------------
# Surprises
# ----------------------------------------------------------------------------

def test_obstacle_event_stops_the_robot_that_would_hit_it(demo_plan):
    m = demo_map()
    t = 12.0
    states = P.states_at(demo_plan, t)
    # put an obstacle on Center's route about 1 s ahead of it
    steps = demo_plan["robots"]["Center"]["steps"]
    i = states["Center"]["step"]
    ahead = next(s for s in steps[i:] if s["type"] == "drive" and s["t_start"] >= t)
    c = ahead["to"][:2]
    ob = {"id": "brick", "shape": "circle", "center": c, "radius": 4.0, "source": "camera"}
    newplan, events = P.obstacle_event(m, demo_plan, states, [ob])
    ev = {e["robot"]: e for e in events}
    assert set(ev) == {"Left", "Center", "Right"}
    within = ahead["t_start"] - t < demo_plan["settings"]["stop_horizon_s"]
    if within:
        assert ev["Center"]["action"] == "stopped"
    # nothing new starts before the new plan can be ready
    for n, r in newplan["robots"].items():
        for st in r["steps"]:
            if not st.get("committed") and st["type"] != "wait":
                assert st["t_start"] >= demo_plan["settings"]["replan_budget_s"] - 1e-6
                break
    m2 = FM.normalize(dict(m, obstacles=[ob]))
    rep = P.verify(m2, newplan)
    assert rep["min_separation_cm"] >= 5.0
    assert rep["min_static_clearance_cm"] >= -1e-6
    # every board still gets visited
    done_before = {n: [s["board"] for s in demo_plan["robots"][n]["steps"][:states[n]["step"]] if s["type"] == "approach"]
                   for n in demo_plan["robots"]}
    for n, r in newplan["robots"].items():
        later = [s["board"] for s in r["steps"] if s["type"] == "approach"]
        want = [v for v in demo_plan["robots"][n]["visit_order"] if v != "park"]
        assert done_before[n] + later == want or (later and done_before[n][-1:] == later[:1])


def test_replan_at_many_moments_always_gives_a_safe_plan(demo_plan):
    m = demo_map()
    ob = {"id": "post", "shape": "circle", "center": [60, 74], "radius": 6, "source": "user"}
    m2 = FM.normalize(dict(m, obstacles=[ob]))
    T = demo_plan["summary"]["makespan_s"]
    for t in np.linspace(0, T, 8):
        newplan, events = P.obstacle_event(m, demo_plan, P.states_at(demo_plan, float(t)), [ob])
        rep = P.verify(m2, newplan)
        assert rep["min_separation_cm"] >= 5.0, t


# ----------------------------------------------------------------------------
# Kinematic simulation
# ----------------------------------------------------------------------------

def test_sim_docks_within_tolerance_and_never_collides(demo_plan):
    for seed in (1, 2):
        sim = SP.Sim(demo_map(), copy.deepcopy(demo_plan), dict(SP.DEFAULT_SIM, stall_prob=0.1, stall_s=3.0),
                     seed=seed, verbose=False)
        res = sim.run_sim()
        assert res["finished"] and not res["collision"]
        assert res["min_body_gap_cm"] >= 5.0
        assert res["dockings"] == 6 and res["docking_all_ok"]
        assert res["dock_gap_error_cm_max"] <= 0.05      # ideal distance sensor


def test_sim_brick_mid_run_replans():
    m = demo_map()
    plan = P.make_plan(m)
    sim = SP.Sim(m, plan, dict(SP.DEFAULT_SIM, brick_ahead_s=1.2), seed=4, verbose=False,
                 brick=SP.parse_brick("auto@30"))
    res = sim.run_sim()
    assert res["replans"], "the brick should have caused a re-plan"
    assert res["finished"] and not res["collision"]
    assert res["min_body_gap_cm"] >= 5.0 - 0.5
    assert res["dockings"] == 6 and res["docking_all_ok"]
    acts = [e["action"] for e in res["replans"][0]["events"]]
    assert "stopped" in acts and "continues" in acts


def test_time_only_execution_is_not_safe_with_delays(demo_plan):
    """Why the step order rules exist: timing alone breaks when a robot runs late."""
    worst = math.inf
    for seed in (600, 601):
        sim = SP.Sim(demo_map(), copy.deepcopy(demo_plan),
                     dict(SP.DEFAULT_SIM, gate="time", stall_prob=0.1, stall_s=3.0), seed=seed, verbose=False)
        worst = min(worst, sim.run_sim()["min_body_gap_cm"])
    assert worst < 5.0


# ----------------------------------------------------------------------------
# Web page server
# ----------------------------------------------------------------------------

def test_app_endpoints(tmp_path):
    import app
    mp = tmp_path / "map.json"
    mp.write_text(json.dumps(FM.load(os.path.join(EX, "park_test_map.json"))))
    sess = app.Session(str(mp), str(tmp_path / "plan.json"))
    server, url = app.serve(sess, port=0)
    th = threading.Thread(target=server.serve_forever, daemon=True)
    th.start()
    try:
        def call(path, body=None, host="127.0.0.1"):
            req = urllib.request.Request(url.rstrip("/") + path,
                                         data=None if body is None else json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json", "Host": host})
            try:
                with urllib.request.urlopen(req) as r:
                    return r.status, json.loads(r.read())
            except urllib.error.HTTPError as e:
                return e.code, json.loads(e.read())
        page = urllib.request.urlopen(url).read().decode()
        assert "<canvas" in page and "https://" not in page
        c, st = call("/api/state")
        assert c == 200 and st["map"]["floor"]["x_max"] == 122
        c, r = call("/api/plan", {"map": st["map"], "settings": {}})
        assert c == 200 and r["plan"]["summary"]["min_separation_cm"] >= 5
        c, e = call("/api/event", {"map": st["map"], "plan": r["plan"], "t": 5.0,
                                   "obstacle": {"id": "b", "shape": "circle", "center": [61, 60], "radius": 5}})
        assert c == 200 and {x["robot"] for x in e["events"]} == {"Left", "Center", "Right"}
        c, s = call("/api/save", {"map": st["map"], "plan": r["plan"]})
        assert c == 200 and (tmp_path / "plan.json").exists()
        c, _ = call("/api/save", {"map": st["map"]}, host="evil.example")
        assert c == 403
        c, bad = call("/api/check", {"map": {"boards": [{"name": "x", "p1": [0, 0], "p2": [0, 0]}]}})
        assert c == 400 and "shorter" in bad["error"]
    finally:
        server.shutdown()
        server.server_close()
