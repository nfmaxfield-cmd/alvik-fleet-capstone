#!/usr/bin/env python3
"""(e23, 19 Nov, simulator only) The acceptance table: one row per run folder made by seed_runs.py, measured the same
way for the code before e23 and for e23, from the SIMULATOR'S TRUTH (run/sim/truth.csv), not from what the run
credited itself.
    acc_table.py OUT.json OUT.md FOLDER [FOLDER ...]
Version 1 (2026-11-19).

  asked            points on the two lists
  credited         different points the run recorded a stop for
  truly reached    different points with a recorded stop whose TRUE position, over the stop's own dwell, was within
                   the ruler below of the point's place (the site's sticker points)
  false credits    recorded stops outside that ruler
  ruler            the tolerance e23 derives for a stop: the larger of 2.9 cm + the stop's own +- and 5.5 cm (the
                   school's measured stop scatter; run_plan.reach_tol_cm). The same ruler for both codes
  standing         seconds the robots stood (true speed under 0.5 cm/s [chosen: well under the slowest drive, 4 cm/s])
                   between the run's first and last step, minus the dwells asked at the points truly reached; both
                   robots added
  closest          the simulator's own closest two bodies and closest body to an open edge; the closest robot middle
                   to a keep-out from the truth (a body's side is 4.75 cm from its middle)"""
import csv, json, math, os, statistics, sys
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..", "..")
sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
import sticker_points as SP, sites as SI   # noqa: E402
SITE = SI.load("school-conference")
PTS = {k: v[:2] for k, v in SP.points_cm(SITE)[0].items()}
STILL_CMS = 0.5


def load(p, d=None):
    try:
        return json.load(open(p))
    except (OSError, ValueError):
        return d


def keepouts():
    out = []
    for k in SITE.get("keepouts") or []:
        if k.get("centre_in") and k.get("size_in") and k.get("use", True):
            cx, cy = (2.54 * float(v) for v in k["centre_in"])
            hw, hh = (1.27 * float(v) for v in k["size_in"])
            out.append((k.get("name") or "keep-out", (cx - hw, cy - hh, cx + hw, cy + hh)))
    return out


def rect_d(p, r):
    dx = max(r[0] - p[0], 0.0, p[0] - r[2])
    dy = max(r[1] - p[1], 0.0, p[1] - r[3])
    return math.hypot(dx, dy)


def one(folder):
    sess = os.path.join(folder, "run")
    ms = load(os.path.join(folder, "mission_summary.json"), {}) or {}
    rs = load(os.path.join(sess, "run_plan_summary.json"), {}) or {}
    case = load(os.path.join(folder, "case.json"), {}) or {}
    vo = (load(os.path.join(folder, "visit_order.json"), {}) or {}).get("orders") or {}
    truth = {}
    tp = os.path.join(sess, "sim", "truth.csv")
    if os.path.exists(tp):
        for r in csv.DictReader(open(tp)):
            truth.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"])))
    asked = sorted({l for v in vo.values() for l in v})
    credited, true_ok, false_cr, worst = set(), set(), [], 0.0
    dwell_true = 0.0
    for q in rs.get("points") or []:
        lab = str(q.get("point", "")).replace("point ", "")
        n = q.get("robot")
        credited.add(lab)
        tgt, tv = PTS.get(lab), truth.get(n) or []
        if not tgt or not tv or not q.get("t_abs"):
            continue
        t1 = float(q["t_abs"])
        t0 = t1 - max(0.0, float(q.get("dwell_s") or 0.0))
        win = [v for v in tv if t0 - 0.05 <= v[0] <= t1 + 0.05] or [min(tv, key=lambda v: abs(v[0] - t1))]
        d = statistics.median([math.hypot(v[1] - tgt[0], v[2] - tgt[1]) for v in win])
        ruler = max(2.9 + float(q.get("plus_minus_cm") or 0.0), 5.5)
        if d <= ruler:
            true_ok.add(lab)
            dwell_true += max(0.0, float(q.get("dwell_s") or 0.0))
            worst = max(worst, d)
        else:
            false_cr.append({"robot": n, "point": lab, "off_cm": round(d, 1), "ruler_cm": round(ruler, 1),
                             "dwell_s": q.get("dwell_s")})
    # standing time from the truth, between the first and the last step of the run
    t_a = t_b = None
    cj = os.path.join(sess, "coordinator.jsonl")
    if os.path.exists(cj):
        for l in open(cj, errors="replace"):
            if '"labelled_step"' in l or '"step_sent"' in l:
                try:
                    t = json.loads(l).get("t")
                except ValueError:
                    continue
                if t:
                    t_a = t if t_a is None else t_a
                    t_b = t
    stand = 0.0
    if t_a and t_b:
        for n, tv in truth.items():
            tv = [v for v in tv if t_a <= v[0] <= t_b]
            for a, b in zip(tv, tv[1:]):
                dt = b[0] - a[0]
                if 0 < dt < 2.0 and math.hypot(b[1] - a[1], b[2] - a[2]) / dt < STILL_CMS:
                    stand += dt
    ko_min, ko_name, pair = None, None, None
    kos = keepouts()
    for n, tv in truth.items():
        for v in tv[::2]:
            for nm, r in kos:
                d = rect_d(v[1:], r)
                if ko_min is None or d < ko_min:
                    ko_min, ko_name = d, "%s (%s)" % (nm, n)
    sim = rs.get("simulator") or {}
    rp = rs.get("replans") or []
    cpu = [r["cpu_s"] for r in rp if r.get("cpu_s") is not None]
    res = str(ms.get("result") or rs.get("result") or ("no summary" if not rs else "?"))
    capped = os.path.exists(os.path.join(folder, "CAPPED.txt"))
    row = {
        "case": os.path.basename(os.path.normpath(folder)), "options": case.get("case"),
        "code": "e23" if os.path.basename(os.path.normpath(folder)).startswith("new") else "before e23 (f4acc77)",
        "result": res + (" (stopped by the runner at its 25 min cap)" if capped else ""),
        "crashed": res.startswith("crashed") or bool(rs.get("crash")),
        "duration_s": rs.get("duration_s"), "planned_s": rs.get("planned_makespan_s"),
        "asked": len(asked), "credited": len(credited & set(asked)) if asked else len(credited),
        "truly_reached": len(true_ok & set(asked)) if asked else len(true_ok),
        "false_credits": false_cr, "worst_true_stop_off_cm": round(worst, 1),
        "said_not_reached": len(rs.get("points_not_reached") or []),
        "skipped": len(rs.get("points_skipped") or []),
        "completed": bool(res == "done" and asked and len(true_ok & set(asked)) == len(asked)),
        "standing_s": round(max(0.0, stand - dwell_true), 1), "standing_raw_s": round(stand, 1),
        "replans": len(rp), "replan_median_s": round(statistics.median(cpu), 2) if cpu else None,
        "replan_sum_s": round(sum(cpu), 1), "robots_out": len(rs.get("robots_out") or []),
        "rejoins": len(rs.get("rejoins") or []), "did_not_move": (rs.get("observe") or {}).get("did_not_move"),
        "closest_two_bodies_cm": sim.get("min_body_gap_cm"), "contact": sim.get("collision"),
        "closest_body_to_open_edge_cm": sim.get("min_open_edge_cm"), "hung_over_edge": sim.get("hung"),
        "closest_middle_to_keepout_cm": None if ko_min is None else round(ko_min, 1), "keepout": ko_name,
        "plan_ahead": rs.get("plan_ahead"),
    }
    return row


def main():
    out_json, out_md, folders = sys.argv[1], sys.argv[2], sys.argv[3:]
    rows = [one(f) for f in folders]
    json.dump(rows, open(out_json, "w"), indent=1)
    h = ["Case", "Code", "How it ended", "Run time (s; planned)", "Truly reached / asked", "Credited",
         "False credits", "Crashed", "Standing (s)", "Re-plans (median s)", "Robots held or out",
         "Closest two bodies (cm)", "Closest body to an open edge (cm)", "Closest middle to a keep-out (cm)"]
    L = ["| " + " | ".join(h) + " |", "|" + "---|" * len(h)]
    for r in rows:
        L.append("| " + " | ".join(str(v) for v in (
            r["case"], r["code"], r["result"], "%s (%s)" % (r["duration_s"], r["planned_s"]),
            "%d / %d" % (r["truly_reached"], r["asked"]), r["credited"], len(r["false_credits"]),
            "yes" if r["crashed"] else "no", r["standing_s"], "%d (%s)" % (r["replans"], r["replan_median_s"]),
            r["robots_out"], r["closest_two_bodies_cm"], r["closest_body_to_open_edge_cm"],
            "%s: %s" % (r["closest_middle_to_keepout_cm"], r["keepout"]))) + " |")
    open(out_md, "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
