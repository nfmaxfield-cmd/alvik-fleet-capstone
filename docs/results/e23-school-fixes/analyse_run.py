#!/usr/bin/env python3
"""(e23) One Visit points run folder (a dry run or a real one) in numbers: how it ended, the points asked, reached, not
reached, skipped and never tried; FALSE CREDITS (a recorded stop whose simulator truth, or camera position for a
real run, is further from the point than the stop's tolerance); the planning time of every re-plan; "did not move"
verdicts; the closest two robots and the closest approach to a keep-out or the edge (the simulator's truth).
    analyse_run.py RUN_FOLDER [OUT.json]"""
import csv, json, math, os, statistics, sys
run = sys.argv[1]
sess = os.path.join(run, "run")
def load(p, d=None):
    try:
        return json.load(open(p))
    except (OSError, ValueError):
        return d
ms = load(os.path.join(run, "mission_summary.json"), {}) or {}
rs = load(os.path.join(sess, "run_plan_summary.json"), {}) or {}
vo = load(os.path.join(run, "visit_order.json"), {}) or {}
orders = vo.get("orders") or {}
site = load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "tools", "fleet", "sites",
                         "school-conference.json"), {}) or {}
pts = {}
try:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "tools", "fleet"))
    import sticker_points as SP, sites as SI
    pts = {k: v[:2] for k, v in SP.points_cm(SI.load("school-conference"))[0].items()}
except Exception as e:      # noqa: BLE001
    print("(points not loaded: %s)" % e)
truth = {}
tp = os.path.join(sess, "sim", "truth.csv")
if os.path.exists(tp):
    for r in csv.DictReader(open(tp)):
        try:
            truth.setdefault(r.get("robot") or r.get("name"), []).append((float(r["t"]), float(r.get("x_cm", r.get("x"))),
                                                                          float(r.get("y_cm", r.get("y")))))
        except (KeyError, ValueError, TypeError):
            pass
def truth_at(n, t):
    v = truth.get(n) or []
    return min(v, key=lambda q: abs(q[0] - t))[1:] if v else None
out = {"folder": os.path.basename(run), "result": ms.get("result") or rs.get("result"), "duration_s": rs.get("duration_s"),
       "planned_s": rs.get("planned_makespan_s"), "crash": (rs.get("crash") or {}).get("why")}
asked = {n: list(v) for n, v in orders.items()}
points = rs.get("points") or []
reached, false_credit, worst = {}, [], 0.0
for q in points:
    lab = str(q.get("point", "")).replace("point ", "")
    n = q.get("robot")
    reached.setdefault(n, [])
    if lab not in reached[n]:
        reached[n].append(lab)
    tgt = pts.get(lab)
    pos = None
    if q.get("t_abs") and truth:
        pos = truth_at(n, float(q["t_abs"]) - float(q.get("dwell_s") or 0.0) / 2.0)
        src = "simulator truth"
    if pos is None and q.get("x") is not None:
        pos, src = (q["x"], q["y"]), "the run's own position"
    if tgt and pos:
        d = math.hypot(pos[0] - tgt[0], pos[1] - tgt[1])
        worst = max(worst, d)
        tol = q.get("reach_tol_cm") or (math.hypot(1.7, 2.4) + float(q.get("plus_minus_cm") or 0.0))
        if d > tol:
            false_credit.append({"robot": n, "point": lab, "off_cm": round(d, 1), "tol_cm": tol, "by": src})
nr = rs.get("points_not_reached") or []
skipped = rs.get("points_skipped") or []
out["asked"] = {n: len(v) for n, v in asked.items()}
out["reached"] = {n: len(set(v)) for n, v in reached.items()}
out["reached_by_anyone"] = len({l for v in reached.values() for l in v})
out["asked_total"] = len({l for v in asked.values() for l in v})
out["not_reached_stops"] = [{"robot": q["robot"], "point": q["point"], "miss_cm": q.get("miss_cm")} for q in nr]
out["skipped"] = [{"robot": q["robot"], "point": q["point"], "why": q["why"][:90]} for q in skipped]
done = {l for v in reached.values() for l in v}
out["never_reached"] = sorted({l for v in asked.values() for l in v} - done)
out["false_credits"] = false_credit
out["worst_credited_stop_off_cm"] = round(worst, 1)
rp = [r for r in rs.get("replans") or []]
cpu = [r["cpu_s"] for r in rp if r.get("cpu_s") is not None]
out["replans"] = len(rp)
out["replan_cpu_s"] = {"n": len(cpu), "median": round(statistics.median(cpu), 2) if cpu else None,
                       "max": round(max(cpu), 2) if cpu else None, "sum": round(sum(cpu), 1),
                       "share_of_run": round(sum(cpu) / rs["duration_s"], 3) if cpu and rs.get("duration_s") else None,
                       "over_budget": sum(1 for r in rp if r.get("cpu_s", 0) > (r.get("budget_s") or 1e9))}
out["improve_tries"] = sum(1 for r in rp if r.get("source") == "improve")
out["points_planned_per_replan"] = sorted({r.get("points_planned") for r in rp if r.get("points_planned") is not None})
out["plan_ahead"] = rs.get("plan_ahead")
ob = rs.get("observe") or {}
out["did_not_move"] = ob.get("did_not_move")
out["late_not_moved"] = ob.get("late_not_moved")
out["old_fixes_carried"] = rs.get("old_fixes_carried")
out["robots_out"] = len(rs.get("robots_out") or [])
out["rejoins"] = len(rs.get("rejoins") or [])
sim = rs.get("simulator") or {}
out["sim"] = {"closest_two_robots_cm": sim.get("min_body_gap_cm"), "closest_static_cm": sim.get("min_static_clearance_cm"),
              "closest_static_at": sim.get("min_static_at"), "contact": sim.get("collision"),
              "closest_open_edge_cm": sim.get("min_open_edge_cm"), "hung": sim.get("hung"),
              "events": [e.get("kind") if isinstance(e, dict) else str(e)[:40] for e in (sim.get("events") or [])][:12]}
# closest approach to the site's keep-outs and to the table's edge, from the simulator's truth (the robot's MIDDLE;
# its body reaches 4.75 cm further along its sides, 6.7 cm at its corners)
def seg_d(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)
def inside(p, poly):
    c = False
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]:
            c = not c
    return c
kos = []
for k in site.get("keepouts") or []:
    poly = k.get("points") or k.get("polygon")
    if not poly and k.get("rect"):
        x0, y0, x1, y1 = k["rect"]
        poly = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    if not poly and k.get("centre_in") and k.get("size_in") and k.get("use", True):
        cx, cy = (2.54 * float(v) for v in k["centre_in"])
        hw, hh = (1.27 * float(v) for v in k["size_in"])
        poly = [[cx - hw, cy - hh], [cx + hw, cy - hh], [cx + hw, cy + hh], [cx - hw, cy + hh]]
    if poly and k.get("use", True):
        kos.append((k.get("name") or k.get("id") or "keep-out", poly))
tb = site.get("table") or {}
Lt, Wt = float(tb.get("length_cm") or 396.2), float(tb.get("width_cm") or 167.6)
ko_min, edge_min, pair_min = (None, None), None, None
for n, v in truth.items():
    for t, x, y in v[::3]:
        e = min(x, y, Lt - x, Wt - y)
        edge_min = e if edge_min is None else min(edge_min, e)
        for nm, poly in kos:
            d = 0.0 if inside((x, y), poly) else min(seg_d((x, y), poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))
            if ko_min[0] is None or d < ko_min[0]:
                ko_min = (d, "%s by %s" % (nm, n))
names = sorted(truth)
if len(names) >= 2:
    a, b = truth[names[0]], truth[names[1]]
    j = 0
    for t, x, y in a[::3]:
        while j + 1 < len(b) and abs(b[j + 1][0] - t) <= abs(b[j][0] - t):
            j += 1
        d = math.hypot(x - b[j][1], y - b[j][2])
        pair_min = d if pair_min is None else min(pair_min, d)
out["truth"] = {"closest_middles_cm": None if pair_min is None else round(pair_min, 1),
                "closest_middle_to_keepout_cm": None if ko_min[0] is None else round(ko_min[0], 1), "keepout": ko_min[1],
                "closest_middle_to_edge_cm": None if edge_min is None else round(edge_min, 1), "keepouts": len(kos)}
print(json.dumps(out))
if len(sys.argv) > 2:
    json.dump(out, open(sys.argv[2], "w"), indent=1)
