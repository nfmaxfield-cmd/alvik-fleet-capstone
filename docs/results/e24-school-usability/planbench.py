#!/usr/bin/env python3
"""(e24, simulator only) The same re-plans timed on two versions of the planner.

    planbench.py REPO DECS_DIR [OUT.json] [--repeat N]

DECS_DIR holds re-plan inputs dumped by a dry run (MIRLT_REPLAN_DUMP=<dir>, run_plan.py; run_case.py's dump=1): each
dec_NNN.pkl is what one re-plan was given. Each is planned again with REPO's planner (planner.obstacle_replan), N times
(the fastest counts: the machine's other work only ever adds time), and the plan's steps are kept as a fingerprint so
two versions can be compared step for step. Run once per version, then compare the two OUT files with --compare.

    planbench.py --compare BEFORE.json AFTER.json [OUT.md]

Prints, per re-plan: points planned, seconds before and after (the FIRST time each version met that re-plan, in the
run's own order: a map met earlier in the run is remembered by e24's planner, a new one is not), and whether the two
plans are the same step for step.
Changelog
  2026-11-24  first version (e24).
"""
import glob, hashlib, json, os, pickle, statistics, sys, time
VERSION = "e24-planbench-2026-11-24"


def fingerprint(plan):
    rows = []
    for n, e in sorted((plan.get("robots") or {}).items()):
        for s in e.get("steps") or []:
            rows.append((n, s.get("type"), s.get("label"), [round(float(v), 2) for v in (s.get("to") or [])],
                         round(float(s.get("t_start") or 0), 2), round(float(s.get("t_end") or 0), 2)))
    return hashlib.sha1(json.dumps(rows).encode()).hexdigest()[:12], len(rows)


def points(plan):
    n = 0
    for e in (plan.get("robots") or {}).values():
        n += sum(1 for v in (e.get("visit_order") or []) if isinstance(v, dict) and "goto" in v and
                 str(v.get("label", "")).startswith("point "))
    return n


def run(repo, decs, out, repeat):
    sys.path.insert(0, os.path.join(repo, "tools", "fleet"))
    os.environ.setdefault("HOME", "/tmp")
    import planner as P
    res = []
    for f in sorted(glob.glob(os.path.join(decs, "dec_*.pkl"))):
        best, fp, npts, err, first = None, None, None, None, None
        for _ in range(repeat):
            with open(f, "rb") as fh:
                d = pickle.load(fh)          # a fresh copy each time: the planner changes what it is given
            st = d["dec"]["s"]
            for k in [k for k in list(st) if k not in P.DEFAULT_SETTINGS]:
                del st[k]                     # a setting this version of the planner does not know (dwell_at_cm)
            if "dwell_at_cm" in st:
                st["dwell_at_cm"] = 0.0       # e24's drive-back for a stop due elsewhere is off here: both versions
                #                               are given the same job, so the plans can be compared step for step
            t0 = time.perf_counter()
            try:
                plan, _ev = P.obstacle_replan(d["dec"])
            except Exception as e:  # noqa: BLE001
                err = "%s: %s" % (type(e).__name__, e)
                break
            dt = time.perf_counter() - t0
            if best is None:
                first = dt                    # the first time this version meets this re-plan, in the run's own order
                #                               (what a run pays: a map met before is remembered, a new one is not)
            best = dt if best is None else min(best, dt)
            fp, npts = fingerprint(plan), points(plan)
        res.append({"dec": os.path.basename(f), "s": None if best is None else round(best, 4),
                    "first_s": None if first is None else round(first, 4), "points": npts,
                    "fingerprint": fp, "error": err, "stranded": None if err else plan.get("stranded")})
        print(res[-1], flush=True)
    json.dump({"version": VERSION, "repo": repo, "repeat": repeat, "replans": res}, open(out, "w"), indent=1)


def compare(a, b, out_md=None):
    A, B = json.load(open(a)), json.load(open(b))
    bb = {r["dec"]: r for r in B["replans"]}
    rows, ratios, same = [], [], 0
    for r in A["replans"]:
        q = bb.get(r["dec"])
        if not q or r["s"] is None or q["s"] is None:
            continue
        eq = r["fingerprint"] == q["fingerprint"]
        same += eq
        ratios.append((q.get("first_s") or q["s"]) / (r.get("first_s") or r["s"]))
        rows.append((r["dec"], r["points"], r.get("first_s") or r["s"], q.get("first_s") or q["s"],
                     (q.get("first_s") or q["s"]) / (r.get("first_s") or r["s"]), eq))
    lines = ["| Re-plan | Points planned | Before (s) | After (s) | After / before | Same plan step for step |", "|---|---|---|---|---|---|"]
    for d, n, x, y, k, eq in rows:
        lines.append("| %s | %s | %.3f | %.3f | %.2f | %s |" % (d, n, x, y, k, "yes" if eq else "NO"))
    summ = {"n": len(rows), "same_plans": same, "before_s_median": round(statistics.median(r[2] for r in rows), 3),
            "after_s_median": round(statistics.median(r[3] for r in rows), 3),
            "ratio_median": round(statistics.median(ratios), 3), "ratio_min": round(min(ratios), 3),
            "ratio_max": round(max(ratios), 3), "before_s_sum": round(sum(r[2] for r in rows), 2),
            "after_s_sum": round(sum(r[3] for r in rows), 2)}
    # the line through (points, seconds) for the code before: what the stand-in's factor is calibrated from
    xs, ys = [r[1] for r in rows], [r[2] for r in rows]
    if len(set(xs)) > 1:
        mx, my = statistics.mean(xs), statistics.mean(ys)
        b1 = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
        summ["before_line"] = {"fixed_s": round(my - b1 * mx, 4), "s_per_point": round(b1, 4)}
    print(json.dumps(summ, indent=1))
    if out_md:
        open(out_md, "w").write("\n".join(lines) + "\n\n" + json.dumps(summ, indent=1) + "\n")
    return summ


if __name__ == "__main__":
    if sys.argv[1] == "--compare":
        compare(sys.argv[2], sys.argv[3], sys.argv[4] if len(sys.argv) > 4 else None)
    else:
        rep = int(sys.argv[sys.argv.index("--repeat") + 1]) if "--repeat" in sys.argv else 3
        run(os.path.abspath(sys.argv[1]), sys.argv[2], sys.argv[3] if len(sys.argv) > 3 and not sys.argv[3].startswith("--") else "/tmp/planbench.json", rep)
