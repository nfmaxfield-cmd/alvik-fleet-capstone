#!/usr/bin/env python3
"""Re-plan bench: the same obstacle re-plans with the old search (the whole budget looking for a shorter plan) and
with the first safe plan sent at once (replan_first_safe, Nick 2026-10-27 00:08), as in
docs/build-reports/e02-oct26-noboards-and-replan.md: the demo map, two obstacles, 10 moments each.

    python3 tools/fleet/bench_replan.py [--out bench.json]

Per re-plan it records: wall time of the whole re-plan (decide + plan), the first safe plan's time, the plan's
finish time (makespan) and the closest two robots in it. "Robots held" = the new plan arrived after the carry-on
horizon (robots stand at their hold point until it does), and by how long.
"""
import argparse
import json
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM  # noqa: E402
import planner as P    # noqa: E402

OBSTACLES = [{"id": "brick-a", "shape": "circle", "center": [61.0, 61.0], "radius": 6.0, "source": "camera"},
             {"id": "brick-b", "shape": "polygon", "points": [[80, 40], [92, 40], [92, 48], [80, 48]], "source": "camera"}]


def one(m, plan, t, ob, first_safe):
    over = dict(plan["settings"])
    over["replan_first_safe"] = first_safe
    states = P.states_at(plan, t)
    t0 = time.perf_counter()
    dec = P.obstacle_decide(m, plan, states, [ob], over=over)
    newplan, _ = P.obstacle_replan(dec)
    wall = time.perf_counter() - t0
    s = P.settings(over)
    horizon = P.carry_on_s(s)
    return {"t": t, "obstacle": ob["id"], "first_safe": first_safe, "wall_s": round(wall, 3),
            "first_plan_s": newplan["summary"].get("first_plan_s"), "makespan_s": newplan["summary"]["makespan_s"],
            "closest_cm": newplan["summary"].get("min_separation_cm"), "orders": newplan["summary"]["orders_tried"],
            "carry_on_s": horizon, "held_s": round(max(0.0, wall - horizon), 3),
            "stranded": newplan.get("stranded", [])}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(HERE, "examples", "demo_map.json"))
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    m = FM.load(a.map)
    plan = P.make_plan(m)
    T = plan["summary"]["makespan_s"]
    moments = [round(T * (k + 1) / 12.0, 1) for k in range(10)]
    rows = []
    for ob in OBSTACLES:
        for t in moments:
            for fs in (False, True):          # old then new, one after the other on the same machine
                rows.append(one(m, plan, t, ob, fs))
    out = {}
    for fs, name in ((False, "old"), (True, "new")):
        r = [x for x in rows if x["first_safe"] == fs]
        w = [x["wall_s"] for x in r]
        fp = [x["first_plan_s"] for x in r if x["first_plan_s"] is not None]
        out[name] = {"n": len(r), "wall_median_s": statistics.median(w), "wall_worst_s": max(w),
                     "first_plan_median_s": statistics.median(fp) if fp else None,
                     "first_plan_worst_s": max(fp) if fp else None,
                     "orders_median": statistics.median(x["orders"] for x in r),
                     "sum_makespan_s": round(sum(x["makespan_s"] for x in r), 1),
                     "closest_cm": min((x["closest_cm"] for x in r if x["closest_cm"] is not None), default=None),
                     "held": sum(1 for x in r if x["held_s"] > 0), "held_worst_s": max(x["held_s"] for x in r),
                     "stranded": sum(1 for x in r if x["stranded"])}
    print(json.dumps(out, indent=1))
    if a.out:
        with open(a.out, "w") as f:
            json.dump({"summary": out, "rows": rows}, f, indent=1)


if __name__ == "__main__":
    main()
