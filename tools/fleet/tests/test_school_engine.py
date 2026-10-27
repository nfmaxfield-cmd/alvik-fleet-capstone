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
