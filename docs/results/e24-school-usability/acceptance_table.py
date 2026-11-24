#!/usr/bin/env python3
"""(e24, simulator only) The acceptance table: every run of run_case.py in ACC_DIR (names B-<case>-s<seed> for the
code before, 39df4cf, and A-<case>-s<seed> for e24), one line per run in acceptance/runs.jsonl and a table of RANGES
over the seeds in acceptance/table.md.

    acceptance_table.py ACC_DIR OUT_DIR

A range is "lowest to highest over the seeds run"; with two or three seeds it says what happened in those runs, not
what will happen in the next one.
Changelog
  2026-11-24  first version (e24).
"""
import glob, json, os, re, sys
ACC, OUT = sys.argv[1], sys.argv[2]
ORDER = ["four", "twenty", "crossed", "restart", "yaw", "batt", "miss"]
WORDS = {"four": "4 + 4 points, a list per robot", "twenty": "20 + 20 points, a list per robot",
         "crossed": "20 + 20, each list in the other robot's half", "restart": "8 + 8, Bravo restarts 60 s in",
         "yaw": "4 + 4, a 25 degree gyro jump at Alpha's turn (40 s in)", "batt": "6 + 6, Bravo starts at 22 % (floor 25 %)",
         "miss": "4 + 4, Bravo is not there"}
rows = []
for jp in sorted(glob.glob(os.path.join(ACC, "[AB]-*-s*.json"))):
    name = os.path.basename(jp)[:-5]
    m = re.match(r"([AB])-(\w+?)-s(\d+)$", name)
    if not m:
        continue
    d = json.load(open(jp))
    folder = os.path.join(ACC, name)
    s = {}
    try:
        s = json.load(open(os.path.join(folder, "run", "run_plan_summary.json")))
    except Exception:  # noqa: BLE001
        pass
    case = {}
    try:
        case = json.load(open(os.path.join(folder, "case.json")))
    except Exception:  # noqa: BLE001
        pass
    th, vr = s.get("turn_heading") or {}, s.get("visit_result") or {}
    bl = s.get("blind") if isinstance(s.get("blind"), dict) else {}     # (a list of blind spells before e24)
    tr, sim = d.get("truth") or {}, d.get("sim") or {}
    rows.append(dict(
        code="before" if m.group(1) == "B" else "after", case=m.group(2), seed=int(m.group(3)),
        result=d.get("result"), capped=os.path.exists(os.path.join(folder, "CAPPED.txt")),
        asked=d.get("asked_total"), reached=d.get("reached_by_anyone"), false_credits=len(d.get("false_credits") or []),
        duration_s=d.get("duration_s"), wall_s=case.get("wall_s"), planned_s=d.get("planned_s"), replans=d.get("replans"),
        replan_cpu_s=(d.get("replan_cpu_s") or {}).get("sum"), robots_out=d.get("robots_out"), rejoins=d.get("rejoins"),
        closest_keepout_cm=tr.get("closest_middle_to_keepout_cm"), closest_edge_cm=tr.get("closest_middle_to_edge_cm"),
        closest_robots_cm=tr.get("closest_middles_cm"), contact=sim.get("contact"),
        headline=vr.get("headline"), missed_lines=sum(len(r.get("missed") or []) for r in (vr.get("robots") or {}).values()),
        retried=len(vr.get("retried") or []), heading_checks=th.get("checks"), heading_corrected=len(th.get("corrected") or []),
        yaw_jumps=len(th.get("yaw_jumps") or []), standing_stays=bl.get("standing_stays"),
        slowed=bl.get("slowed")))
os.makedirs(OUT, exist_ok=True)
doe = [dict(r, ahead=r["case"][3:]) for r in rows if r["case"].startswith("doe")]     # the plan-ahead experiment's runs
rows = [r for r in rows if not r["case"].startswith("doe")]
with open(os.path.join(OUT, "runs.jsonl"), "w") as f:
    for r in rows:
        f.write(json.dumps(r) + "\n")
if doe:
    with open(os.path.join(OUT, "doe_runs.jsonl"), "w") as f:
        for r in doe:
            f.write(json.dumps(r) + "\n")


def rng(vals, fmt="%.0f"):
    vals = [v for v in vals if isinstance(v, (int, float)) and v == v and abs(v) != float("inf")]
    if not vals:
        return "-"
    lo, hi = min(vals), max(vals)
    return fmt % lo if fmt % lo == fmt % hi else (fmt % lo + " to " + fmt % hi)


L = ["| Case | Code | Seeds | Points reached of asked | Runs whose last line said done | Run time (s) | Re-plans | Times a robot was held out | Nearest a robot's middle came to a keep-out (cm) | to an open edge (cm) |",
     "|---|---|---|---|---|---|---|---|---|---|"]
for c in ORDER:
    for code in ("before", "after"):
        g = [r for r in rows if r["case"] == c and r["code"] == code]
        if not g:
            continue
        asked = 8 if c == "miss" else g[0]["asked"]     # (the run counts the lists of the robots it found: 4; 8 were typed)
        L.append("| %s | %s | %s | %s of %s | %d of %d | %s | %s | %s | %s | %s |" % (
            WORDS.get(c, c), code, ", ".join(str(r["seed"]) for r in g), rng([r["reached"] for r in g]), asked,
            sum(1 for r in g if (r["headline"] or "").startswith("Done") or (r["headline"] is None and r["result"] == "done")), len(g), rng([r["duration_s"] for r in g]),
            rng([r["replans"] for r in g]), rng([r["robots_out"] for r in g]),
            rng([r["closest_keepout_cm"] for r in g], "%.1f"), rng([r["closest_edge_cm"] for r in g], "%.1f")))
open(os.path.join(OUT, "table.md"), "w").write("\n".join(L) + "\n")
print("\n".join(L))

if doe:
    D = ["| Points planned ahead per robot | Runs | Runs with every point reached | Points reached of %s | Run time (s): lowest, middle, highest | Time spent planning (s): middle | Re-plans: middle |" % doe[0]["asked"],
         "|---|---|---|---|---|---|---|"]
    med = lambda v: sorted(v)[len(v) // 2] if v else float("nan")
    for a in ("1", "2", "3", "all"):
        g = [r for r in doe if r["ahead"] == a]
        if not g:
            continue
        du = [r["duration_s"] or 0 for r in g]
        D.append("| %s | %d | %d | %s | %.0f, %.0f, %.0f | %.1f | %.0f |" % (
            "whole list" if a == "all" else a, len(g), sum(1 for r in g if r["reached"] == r["asked"] and r["result"] == "done"),
            rng([r["reached"] for r in g]), min(du), med(du), max(du), med([r["replan_cpu_s"] or 0 for r in g]),
            med([r["replans"] or 0 for r in g])))
    open(os.path.join(OUT, "doe_table.md"), "w").write("\n".join(D) + "\n")
    print("\n".join(D))
