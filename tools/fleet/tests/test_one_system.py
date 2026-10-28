"""One system (build e02-oct28-one-system.md): blocks, Monday's default mission, the arc move, the dance step."""
import math
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(FLEET))
sys.path.insert(0, FLEET)

import engine_checks as EC  # noqa: E402
import fleetmap as FM       # noqa: E402
import missions as MS       # noqa: E402
import planner as P         # noqa: E402
import run_plan as RP       # noqa: E402
import workspace as WS      # noqa: E402

TABLES = [(122, 61), (130, 70), (137, 76)]


def table(W, H):
    return WS.blank_map((0, 0, W, H), [("Left", [20, 20, 90]), ("Center", [60, 20, 90]), ("Right", [100, 20, 90])])


@pytest.fixture(scope="module")
def plans():
    out = {}
    for W, H in TABLES:
        mm = MS.build(table(W, H), MS.monday_program())
        out[(W, H)] = (mm, P.make_plan(mm))
    return out


@pytest.mark.parametrize("size", TABLES)
def test_monday_fits_every_table_size(plans, size):
    mm, plan = plans[size]
    assert set(plan["robots"]) == {"Left", "Right"}
    gap = EC.plan_gap(plan, mm)
    assert gap["closest_cm"] >= 8.0                    # Nick's safety net
    for n, r in plan["robots"].items():
        arcs = [st for st in r["steps"] if st["type"] == "arc"]
        assert sum(st["arc"]["sweep_deg"] for st in arcs) == pytest.approx(540.0, abs=0.01)   # 1.5 times round
        labels = [st.get("label") or "" for st in r["steps"]]
        assert any(lb.startswith("chicken") for lb in labels) and r["steps"][-1]["type"] == "dance"
    assert not plan.get("stranded")


def test_monday_circle_together_and_swapped(plans):
    mm, plan = plans[(122, 61)]
    a = [st for st in plan["robots"]["Left"]["steps"] if st["type"] == "arc"]
    b = [st for st in plan["robots"]["Right"]["steps"] if st["type"] == "arc"]
    assert a[0]["t_start"] == pytest.approx(b[0]["t_start"], abs=0.25)      # they set off together
    assert a[-1]["t_end"] == pytest.approx(b[-1]["t_end"], abs=0.25)
    # Left came from the low-x end and leaves the circle on the high-x side (swapped)
    assert a[0]["from"][0] < 61 < a[-1]["to"][0] and b[0]["from"][0] > 61 > b[-1]["to"][0]
    # the meet: each robot's first arc waits for the other robot's meet step (sync)
    sync_l = next(st["i"] for st in plan["robots"]["Left"]["steps"] if st.get("sync"))
    sync_r = next(st["i"] for st in plan["robots"]["Right"]["steps"] if st.get("sync"))
    assert ["Right", sync_r] in plan["robots"]["Left"]["steps"][sync_l + 1]["after"] or \
        any(x[0] == "Right" and x[1] >= sync_r for x in plan["robots"]["Left"]["steps"][sync_l + 1]["after"])


def test_monday_mows_every_lane_once(plans):
    mm, plan = plans[(130, 70)]
    lanes, _, _, _ = MS.lanes_x(mm, None)
    seen = []
    for r in mm["tasks"].values():
        for v in r["visit"]:
            if "lane" in v and v.get("end") == 0:
                seen.append(round(v["lane"][0], 2))
    assert sorted(seen) == sorted(round(x, 2) for x in lanes)


def test_build_rejects_bad_programs():
    m = table(122, 61)
    with pytest.raises(ValueError):
        MS.build(m, {"robots": ["Left", "Right"], "blocks": [{"block": "circle", "robots": ["Left", "Right"]}]})
    with pytest.raises(ValueError):
        MS.build(m, {"robots": ["Left", "Nobody"], "blocks": []})
    with pytest.raises(ValueError):
        MS.build(m, {"robots": ["Left"], "blocks": [{"block": "fly", "robot": "Left"}]})


def test_blocks_go_to_spot_wait_dance():
    m = table(122, 61)
    m = WS.add_spot(m, (40, 30), "A")[0]
    prog = {"robots": ["Center"], "starts": {"Center": {"at": [0.5, 0.0]}},
            "blocks": [{"block": "go-to-spot", "robot": "Center", "spot": "A"}, {"block": "wait", "robots": ["Center"],
                                                                                "seconds": 2},
                       {"block": "dance", "robots": ["Center"], "seconds": 3}]}
    mm = MS.build(m, prog)
    plan = P.make_plan(mm)
    st = plan["robots"]["Center"]["steps"]
    assert st[-1]["type"] == "dance" and st[-1]["dance_ms"] == 3000
    assert RP.step_command(9, st[-1]) == "fs 9 Z 3000"


def test_arc_step_command_and_geometry():
    s = P.settings()
    ap = P.arc_prims([61, 30], 15, math.pi, math.radians(540), s)
    assert len(ap) == 18 and all(abs(math.degrees(p["arc"]["sweep"]) - 30) < 1e-6 for p in ap)
    plan_st = P.to_steps([(0, ap[0])], [[]], s)[0]
    cmd = RP.step_command(4, plan_st)
    assert cmd.startswith("fs 4 C 61.00 30.00 15.00 180.00 30.00 8.00")
    # every pose along the arc is on the circle, facing along it (anticlockwise: angle + 90)
    for q in P.step_poses(plan_st, 10):
        assert math.hypot(q[0] - 61, q[1] - 30) == pytest.approx(15, abs=1e-6)
        ang = math.atan2(q[1] - 30, q[0] - 61)
        assert math.cos(q[2] - ang - math.pi / 2) == pytest.approx(1.0, abs=1e-6)


def test_replan_mid_circle_does_the_rest_not_the_whole_circle(plans):
    mm, plan = plans[(122, 61)]
    a = [st for st in plan["robots"]["Left"]["steps"] if st["type"] == "arc"]
    t = a[5]["t_start"] + 0.3                         # a sixth of the way round
    states = P.states_at(plan, t)
    # (a Monday-length re-plan needs more than the default 2 s budget's first try: run_plan retries with more)
    dec = P.obstacle_decide(mm, plan, states, [], over=dict(plan["settings"], replan_budget_s=8.0), t_now=t)
    newplan, _ = P.obstacle_replan(dec)
    assert not newplan.get("stranded")
    for n in ("Left", "Right"):
        sw = sum(st["arc"]["sweep_deg"] for st in newplan["robots"][n]["steps"] if st["type"] == "arc")
        assert 540 - 6 * 30 - 1 <= sw <= 540 - 5 * 30 + 1     # what was left, not 540 again
    assert EC.plan_gap(newplan, mm)["closest_cm"] >= 8.0


def test_dance_holds_its_turn_circle_for_good():
    """A robot dancing until switched off occupies its whole turn circle: another robot planned past it keeps clear
    of the circle, not just of the body."""
    m = table(122, 61)
    m["robots"] = [r for r in m["robots"] if r["name"] in ("Left", "Right")]
    m["robots"][0]["start"] = [40, 30, 90]
    m["robots"][1]["start"] = [100, 30, 180]
    m["tasks"] = {"Left": {"visit": [{"dance": 0}], "park": False},
                  "Right": {"visit": [{"goto": [16, 48]}], "park": False}}
    m["mission"] = {"planner": {"separation_cm": 8.0}}
    plan = P.make_plan(FM.normalize(m))
    body = FM.body_of(FM.normalize(m))
    d = plan["robots"]["Left"]["steps"][-1]
    assert d["type"] == "dance"
    cx, cy = d["to"][:2]
    import geometry as G
    import numpy as np
    for st in plan["robots"]["Right"]["steps"]:
        if st["t_end"] <= d["t_start"]:
            continue
        for q in P.step_poses(st, 10):
            # Right's body against every heading Left's body takes while it dances: at least the 8 cm separation
            spins = np.array([[cx, cy, a] for a in np.linspace(0, 2 * math.pi, 24)])
            assert G.body_gaps(body, spins, np.repeat(np.array([q]), 24, 0)).min() >= 8.0 - 0.3


def test_firmware_host_compiles():
    """firmware/home_lab with the stand-in headers (tools/fwcheck): NOT the Arduino toolchain."""
    r = subprocess.run(["sh", os.path.join(REPO, "tools", "fwcheck", "check.sh")], capture_output=True, text=True)
    assert r.returncode == 0 and r.stdout.strip() == "", r.stdout[-2000:]
    src = open(os.path.join(REPO, "firmware", "home_lab", "home_lab.ino")).read()
    assert '#define FW_VERSION "2026-10-28a"' in src
    ex = open(os.path.join(REPO, "firmware", "home_lab", "secrets.example.h")).read()
    assert '#define WIFI_SSID2     "ISE"' in ex and "type-the-school-password-here" in ex


def test_mission_control_plans_the_block_program(tmp_path):
    import mission_control as MC
    S = MC.Session(str(tmp_path))
    S.set_map({"floor": {"x_min": 0, "y_min": 0, "x_max": 130, "y_max": 70}})
    st = S.state()
    assert st["program"]["robots"] == ["Left", "Right"] and [b["block"] for b in st["program"]["blocks"]][3] == "circle"
    r = S.make_mission({"mode": "program", "program": st["program"]})
    pv = r["preview"]
    assert set(pv["frames"]) == {"Left", "Right"} and pv["summary"]["closest_cm"] >= 8.0
    assert len(pv["paths"]["Left"]) > 20                     # the circle drawn through points along it
    assert os.path.exists(os.path.join(str(tmp_path), "program.json"))
    two = S.default_program(["Center", "Right"])["program"]
    assert two["robots"] == ["Center", "Right"]


def test_mission_control_table_resize_carries_starts_and_board_wait(tmp_path):
    import mission_control as MC
    S = MC.Session(str(tmp_path))
    S.set_map({"floor": {"x_min": 0, "y_min": 0, "x_max": 122, "y_max": 122}})
    S.set_map({"floor": {"x_min": 0, "y_min": 0, "x_max": 122, "y_max": 61}})
    for r in S.map["robots"]:                 # every start spot inside the smaller table, room to turn
        assert MC.START_IN_CM - 1e-6 <= r["start"][0] <= 122 - MC.START_IN_CM + 1e-6
        assert MC.START_IN_CM - 1e-6 <= r["start"][1] <= 61 - MC.START_IN_CM + 1e-6
    with pytest.raises(ValueError):
        S.setup_board_away()                  # not waiting
    S.setup_job = {"running": True, "steps": [], "wait_board": True}
    assert S.setup_board_away()["wait_board"] and S.setup_job["board_away"]
