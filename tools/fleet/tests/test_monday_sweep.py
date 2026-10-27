"""Monday sweep (2026-10-27): the re-plan bug it found, and the sweep's own pieces (tools/fleet/sweeps)."""
import json
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
for p in (FLEET, os.path.join(FLEET, "sweeps")):
    if p not in sys.path:
        sys.path.insert(0, p)

import fleetmap as FM      # noqa: E402
import planner as P        # noqa: E402
import warmup as WU        # noqa: E402
import workspace as WS     # noqa: E402


def _states(plan, t, special):
    out = {}
    for n, r in plan["robots"].items():
        if n in special:
            out[n] = special[n]
            continue
        i = next((k for k, st in enumerate(r["steps"]) if st["t_end"] > t), len(r["steps"]))
        out[n] = {"pose": list(P.pose_at(plan, n, t)), "step": i, "in_progress": False, "dwell_left_s": None}
    return out


def _strip(x0, x1, y0, y1):
    return {"shape": "polygon", "points": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]], "source": "edge_guard"}


def test_second_replan_keeps_the_visits_after_a_carried_on_step():
    """Warm-up dry run 2026-10-27 (152 x 76, all sides open): Right carried on its bottom feel through Left's edge
    re-plan, so the new plan began with that feel (task 0) while its own visit list started at task 1. When the feel
    then ended at the edge, the next re-plan took Right's remaining visits from index 0 - 1 = -1: only the last one
    (the top feel), and Right's two right-side feels and both corners were dropped without a word."""
    W, H = 152, 76
    m = WS.blank_map((0, 0, W, H), [("Left", [30.0, 18.0, 90.0]), ("Center", [76.0, 18.0, 90.0]),
                                    ("Right", [122.0, 18.0, 90.0])])
    wm = WU.plan_tasks(m)
    plan0 = P.make_plan(wm)
    r0 = plan0["robots"]["Right"]["steps"]
    k = next(i for i, st in enumerate(r0) if st["phase"] == "probe")
    t1 = (r0[k]["t_start"] + r0[k]["t_end"]) / 2.0
    # 1. someone else's edge stop while Right is half-way through its feel: Right carries the feel on
    st1 = _states(plan0, t1, {"Right": {"pose": list(P.pose_at(plan0, "Right", t1)), "step": k, "in_progress": True,
                                        "dwell_left_s": None}})
    plan1, _ = P.obstacle_event(wm, plan0, st1, [_strip(-21.0, 59.0, -22.2, 2.8)])
    r1 = plan1["robots"]["Right"]
    assert r1["steps"][0]["phase"] == "probe" and r1["steps"][0]["task"] < r1.get("task_offset", 0), \
        "precondition: the carried-on feel's task is before the new plan's own visits"
    feels_before = [v.get("label") for v in r1["visit_order"] if isinstance(v, dict)]
    assert "feel right side at 19" in feels_before
    # 2. Right's own feel ends at the bottom edge (edge guard): plan again from there
    pose = [r1["steps"][0]["to"][0], 10.7, -math.pi / 2]
    wm1 = dict(wm)
    wm1["obstacles"] = list(wm.get("obstacles", [])) + [_strip(-21.0, 59.0, -22.2, 2.8)]
    st2 = _states(plan1, 0.5, {"Right": {"pose": pose, "step": 0, "in_progress": False, "dwell_left_s": None}})
    plan2, _ = P.obstacle_event(wm1, plan1, st2, [_strip(93.0, 173.0, -22.3, 2.7)])
    labels = [v.get("label") for v in plan2["robots"]["Right"]["visit_order"] if isinstance(v, dict)]
    for lab in ("feel right side at 19", "feel right side at 57", "feel top side at 133"):
        assert lab in labels, (lab, labels)


def test_a_robot_with_nothing_left_to_do_is_not_driven_through():
    """A robot with no visits (finished, or parked early) was given an empty timeline without checking that standing
    where it is clears the others: with the first pass's 3 s hold, a robot planned before it drove straight through
    it (plan min separation 0.0 cm). Monday sweep 2026-10-27 (angled camera, sides moved): Center's lanes into the
    parked Right, stopped only by Center's front sensor at 2.8 cm."""
    m = WS.blank_map((0, 0, 152, 76), [("A", [30.0, 38.0, 0.0]), ("B", [76.0, 38.0, 90.0])])
    m["tasks"] = {"A": {"visit": [{"goto": [122.0, 38.0]}], "park": False}, "B": {"visit": [], "park": False}}
    plan = P.make_plan(m, t_limit_s=20)
    assert plan["summary"]["min_separation_cm"] >= plan["settings"]["separation_cm"] - 1e-6
