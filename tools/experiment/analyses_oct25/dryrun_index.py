"""Index of every simulator dry run on disk (logs/e02-*-dryrun), one row each: the plan line, runs done / failed /
not reached (e02_state.json schedule), each condition's median end error (camera at the route's end against its
start, cm; the simulated camera), holds (log lines "HOLD: every robot is stopped."), and the degraded sensor if any.

    python3 tools/experiment/analyses_oct25/dryrun_index.py logs > docs/results/data/dryrun_index_<date>.md

Used by docs/results/simulation-register.md (Appendix B). Read-only."""
import glob, json, os, re, statistics, sys
root = sys.argv[1]
print("| Dry run | Plan | Runs done / failed / other | End error median U / C / K / KC (cm) | Holds | Degraded | Other |")
print("|---|---|---|---|---|---|---|")
for d in sorted(glob.glob(os.path.join(root, "e02-*-dryrun"))):
    name = os.path.basename(d).replace("e02-", "").replace("-dryrun", "")
    st = {}
    try:
        st = json.load(open(os.path.join(d, "e02_state.json")))
    except Exception:
        pass
    sched = st.get("schedule", []) or []
    done = sum(1 for r in sched if r.get("status") == "done")
    failed = sum(1 for r in sched if r.get("status") == "failed")
    other = len(sched) - done - failed
    errs = {}
    for r in sched:
        if r.get("status") == "done" and r.get("end_err_cm") is not None:
            errs.setdefault(r["cond"], []).append(r["end_err_cm"])
    e = " / ".join("%.1f" % statistics.median(errs[c]) if c in errs else "-" for c in ("U", "C", "K", "KC"))
    log = ""
    p = os.path.join(d, "e02_log.txt")
    if os.path.exists(p):
        log = open(p, errors="replace").read()
    holds = log.count("HOLD: every robot is stopped.")
    m = re.search(r"Plan: ([^\n]*)", log)
    plan = (m.group(1)[:50] if m else ("explore" if "explore" in log[:3000].lower() else "?"))
    deg = re.findall(r"DEGRADED ON PURPOSE: ([^)\n]*)", log)
    deg = deg[0] if deg else ""
    other_bits = []
    if "explore" in log.lower(): other_bits.append("explore")
    if "kept pose" in log.lower() or "kept-pose" in log.lower(): other_bits.append("kept pose")
    hp = log.count("help call") 
    if os.path.exists(os.path.join(d, "dry_run_report.json")):
        rep = json.load(open(os.path.join(d, "dry_run_report.json"))).get("summary", {})
        other_bits.append("track err med %.1f" % rep.get("track_err_median_cm", float("nan")))
    print("| %s | %s | %d / %d / %d | %s | %d | %s | %s |" % (name, plan, done, failed, other, e, holds, deg, ", ".join(other_bits)))
