#!/usr/bin/env python3
"""(e23c, simulator only) The acceptance runs over seeds: one row per run folder made by seed_runs.py (folders named
<case>-s<seed>), measured by acc_table.one (the simulator's truth), plus counts from the run's own log, and the range
of each number per case.
    seeds_table.py ACC_DIR OUT.json OUT.md
Version 1 (2026-11-19)."""
import collections, glob, json, os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import acc_table as AT   # noqa: E402

ACC, OUTJ, OUTM = sys.argv[1], sys.argv[2], sys.argv[3]
rows = []
for folder in sorted(glob.glob(os.path.join(ACC, "*-s[0-9]*"))):
    m = re.match(r"(.+)-s(\d+)$", os.path.basename(folder))
    if not m or not os.path.isdir(folder) or folder.endswith("-home"):
        continue
    r = AT.one(folder)
    ev = collections.Counter()
    try:
        for ln in open(os.path.join(folder, "run", "coordinator.jsonl")):
            try:
                ev[json.loads(ln).get("event")] += 1
            except ValueError:
                pass
    except OSError:
        pass
    try:
        out = open(os.path.join(folder, "mission_stdout.txt"), errors="replace").read()
    except OSError:
        out = ""
    capped = os.path.exists(os.path.join(folder, "CAPPED.txt"))
    case = json.load(open(os.path.join(folder, "case.json")))
    rows.append({"case": m.group(1), "seed": int(m.group(2)), "ended": "capped" if capped else (r.get("result") or "?"),
                 "wall_s": case.get("wall_s"), "run_s": r.get("duration_s"), "planned_s": r.get("planned_s"),
                 "asked": r.get("asked"), "truly": r.get("truly_reached"), "credited": r.get("credited"),
                 "false_credits": len(r.get("false_credits") or []), "said_not_reached": r.get("said_not_reached"),
                 "skipped": r.get("skipped"), "crashed": r.get("crashed"), "traceback": "Traceback" in out,
                 "replans": r.get("replans"), "replan_median_s": r.get("replan_median_s"),
                 "held_or_out": r.get("robots_out"), "veers": ev.get("veer", 0),
                 "veer_not_at_point": ev.get("veer_not_at_point", 0), "points_skipped": ev.get("point_skipped", 0),
                 "rejoined_mid_plan": out.count("rejoined while a plan"),
                 "old_fix_carried_first_and_every_50th": ev.get("old_fix_carried", 0),
                 "old_fix_not_carried": ev.get("old_fix_not_carried", 0),
                 "closest_two_bodies_cm": r.get("closest_two_bodies_cm"), "contact": r.get("contact"),
                 "closest_body_to_open_edge_cm": r.get("closest_body_to_open_edge_cm"),
                 "hung_over_edge": len(r.get("hung_over_edge") or []),
                 "closest_middle_to_keepout_cm": r.get("closest_middle_to_keepout_cm"),
                 "plan_ahead_points_each": (r.get("plan_ahead") or {}).get("points_each")})


def rng(vals, fmt="%s"):
    vals = [v for v in vals if v is not None]
    if not vals:
        return "-"
    lo, hi = min(vals), max(vals)
    return fmt % lo if lo == hi else (fmt + " to " + fmt) % (lo, hi)


L = ["| List | Seed | How it ended | Truly reached / asked | False credits | Run time (s; planned) | Re-plans (median s) | "
     "Turn-backs (veers) | Robots held or out | Points dropped | Closest two bodies (cm) | Closest body to an open edge (cm) | "
     "Closest middle to a keep-out (cm) |", "|" + "---|" * 13]
summary = {}
for case in sorted({r["case"] for r in rows}, key=lambda c: (len(c), c)):
    rs = sorted([r for r in rows if r["case"] == case], key=lambda r: r["seed"])
    for r in rs:
        L.append("| %s | %d | %s | %s / %s | %s | %.0f (%.0f) | %s (%s) | %s | %s | %s | %.1f | %.1f | %.1f |" % (
            case, r["seed"], {"capped": "**not finished** at the runner's cap"}.get(r["ended"], r["ended"]), r["truly"],
            r["asked"], r["false_credits"], r["run_s"] or 0, r["planned_s"] or 0, r["replans"], r["replan_median_s"],
            r["veers"], r["held_or_out"], r["points_skipped"], r["closest_two_bodies_cm"],
            r["closest_body_to_open_edge_cm"], r["closest_middle_to_keepout_cm"]))
    summary[case] = {"runs": len(rs), "finished": sum(r["ended"] != "capped" for r in rs),
                     "truly": rng([r["truly"] for r in rs]), "asked": rs[0]["asked"],
                     "false_credits": rng([r["false_credits"] for r in rs]),
                     "run_s": rng([r["run_s"] for r in rs], "%.0f"), "replans": rng([r["replans"] for r in rs]),
                     "veers": rng([r["veers"] for r in rs]), "held_or_out": rng([r["held_or_out"] for r in rs]),
                     "closest_two_bodies_cm": rng([r["closest_two_bodies_cm"] for r in rs], "%.1f"),
                     "closest_body_to_open_edge_cm": rng([r["closest_body_to_open_edge_cm"] for r in rs], "%.1f"),
                     "closest_middle_to_keepout_cm": rng([r["closest_middle_to_keepout_cm"] for r in rs], "%.1f"),
                     "contact": any(r["contact"] for r in rs), "hung": sum(r["hung_over_edge"] for r in rs),
                     "crashed": sum(bool(r["crashed"]) or r["traceback"] for r in rs)}
    s = summary[case]
    L.append("| **%s, range of %d runs** | | %d of %d finished | **%s** / %s | %s | %s | %s | %s | %s | | %s | %s | %s |" % (
        case, s["runs"], s["finished"], s["runs"], s["truly"], s["asked"], s["false_credits"], s["run_s"], s["replans"],
        s["veers"], s["held_or_out"], s["closest_two_bodies_cm"], s["closest_body_to_open_edge_cm"],
        s["closest_middle_to_keepout_cm"]))
json.dump({"rows": rows, "summary": summary}, open(OUTJ, "w"), indent=1)
open(OUTM, "w").write("\n".join(L) + "\n")
print("\n".join(L))
