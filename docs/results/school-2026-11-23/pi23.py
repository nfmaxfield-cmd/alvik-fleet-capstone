#!/usr/bin/env python3
"""The Pi while each plan ran (23 Nov 2026 school runs): cores, processes, temperature, throttling, recorder lag,
pictures read per second, picture age when read, picture stalls, gaps between camera fixes, re-plan times.
Read-only. Usage: python3 -I pi23.py <jayruns23 folder> [out.json]
Window: from the plan's start to the last coordinator event. Definitions:
  recorder lag   pi_monitor.csv lag_*_s (now minus the newest picture's capture time at the recorder)
  picture age    bridge/detect_times.csv t_done - t (picture stamp to the bridge having read it)
  staleness      at any moment, now minus the stamp of the newest picture the bridge has finished, per camera
  stall          a stretch where staleness is over STALL_S; its length = the largest staleness reached
  fix gap        time between camera fixes sent for one robot (bridge/fixes.csv)
STALL_S = 1.0 s is the oldest fix the robots take (fix_max_age_ms 1000 in every run's param.csv): not chosen here.
Changelog: 2026-11-24 first version."""
import csv, json, os, sys, collections, statistics as S
ROOT = sys.argv[1]; STALL_S = 1.0
def q(v, p):
    v = sorted(v); return v[int(p * (len(v) - 1))] if v else None
def rnd(x, n=2): return None if x is None else round(x, n)
def rows(p):
    if not os.path.exists(p): return []
    with open(p, newline="") as f: return list(csv.DictReader(f))
def fl(r, k):
    try: return float(r[k])
    except (ValueError, KeyError, TypeError): return None
out = []
for d in sorted(os.listdir(ROOT)):
    p = os.path.join(ROOT, d); cj = os.path.join(p, "run", "coordinator.jsonl")
    if not os.path.exists(cj): continue
    L = []
    for l in open(cj, errors="replace"):
        try: L.append(json.loads(l))
        except ValueError: pass
    t0 = next((e["t"] for e in L if e.get("event") == "message" and "plan plan-" in e.get("text", "") and "started" in e["text"]), None)
    if t0 is None: continue
    t1 = L[-1]["t"]; R = {"run": d[:15], "clock": d[9:11] + ":" + d[11:13], "plan_s": round(t1 - t0)}
    so = open(os.path.join(p, "run_plan_stdout.txt"), errors="replace").read()
    import re
    R["camrec_pid"] = (re.search(r"camrec.py pid (\d+)", so) or [None, None])[1]
    R["robots"] = len(set(e["robot"] for e in L if e.get("event") == "step_sent"))
    cpu = [r for r in rows(os.path.join(p, "cpu.csv")) if (fl(r, "t") or 0) >= t0 and (fl(r, "t") or 0) <= t1]
    for k in ("core0", "core1", "core2", "core3", "temp_c", "arm_mhz", "recorder_cpu", "tag_bridge_cpu", "run_plan_cpu", "load1", "mem_avail_mb"):
        v = [fl(r, k) for r in cpu if fl(r, k) is not None]
        R[k] = rnd(S.median(v), 1) if v else None
        if k in ("temp_c", "core1", "run_plan_cpu"): R[k + "_p95"] = rnd(q(v, 0.95), 1)
    pm = [r for r in rows(os.path.join(p, "pi_monitor.csv")) if t0 <= (fl(r, "t") or 0) <= t1]
    R["rows"] = len(pm); R["throttled_now_rows"] = sum(1 for r in pm if r.get("throttle_now"))
    R["throttle_words"] = collections.Counter(r.get("throttle_now") for r in pm if r.get("throttle_now")).most_common(3)
    R["card_gb_end"] = fl(pm[-1], "disk_free_gb") if pm else None
    for cam in ("overhead", "side"):
        v = [fl(r, "lag_%s_s" % cam) for r in pm if fl(r, "lag_%s_s" % cam) is not None]
        R["lag_" + cam] = {"median": rnd(S.median(v)) if v else None, "p95": rnd(q(v, 0.95)), "max": rnd(max(v)) if v else None,
                           "over1_pct": rnd(100.0 * sum(x > 1 for x in v) / len(v), 1) if v else None, "none_rows": len(pm) - len(v)}
    dt = collections.defaultdict(list)
    for r in rows(os.path.join(p, "bridge", "detect_times.csv")):
        t, td, ms = fl(r, "t"), fl(r, "t_done"), fl(r, "ms")
        if t is None or td is None or td < t0 or td > t1: continue
        dt[r["cam"]].append((t, td, ms or 0.0))
    R["cams"] = {}; ep_all = {}
    for cam, v in dt.items():
        v.sort(key=lambda x: x[1])
        age = [b - a for a, b, _ in v]
        # staleness: just before picture i is finished, the newest finished stamp is the max so far
        eps = []; newest = v[0][0]; t_prev = v[0][1]
        for a, b, _ in v[1:]:
            st = b - newest                       # the largest staleness in (t_prev, b)
            if st > STALL_S: eps.append((b - (st - STALL_S), st))      # (when it crossed the limit, how stale it got)
            newest = max(newest, a); t_prev = b
        ep_all[cam] = eps
        dur = (v[-1][1] - v[0][1]) or 1.0
        gaps = [b[1] - a[1] for a, b in zip(v, v[1:])]
        between = [b[0] - a[0] for a, b in zip(eps, eps[1:])]
        R["cams"][cam] = {"pictures": len(v), "per_s": rnd(len(v) / dur), "age_median": rnd(S.median(age)), "age_p95": rnd(q(age, 0.95)),
                          "age_max": rnd(max(age)), "age_over1_pct": rnd(100.0 * sum(x > 1 for x in age) / len(age), 1),
                          "detect_ms_median": rnd(S.median(x[2] for x in v), 0), "detect_ms_p95": rnd(q([x[2] for x in v], 0.95), 0),
                          "stalls": len(eps), "stalls_per_min": rnd(60.0 * len(eps) / dur), "stall_len_median": rnd(S.median(e[1] for e in eps)) if eps else None,
                          "stall_len_p90": rnd(q([e[1] for e in eps], 0.9)), "stall_len_max": rnd(max(e[1] for e in eps)) if eps else None,
                          "stall_time_pct": rnd(100.0 * sum(e[1] - STALL_S for e in eps) / dur, 1),
                          "between_stalls_median_s": rnd(S.median(between), 1) if between else None,
                          "read_gap_p95": rnd(q(gaps, 0.95)), "read_gap_max": rnd(max(gaps)) if gaps else None}
    # both cameras stale at once (a stall the stand-in models): overlap of the two cameras' stall stretches
    if len(ep_all) == 2:
        a, b = [sorted((s, s + ln - STALL_S) for s, ln in ep_all[k]) for k in sorted(ep_all)]
        both = [(max(x[0], y[0]), min(x[1], y[1])) for x in a for y in b if min(x[1], y[1]) > max(x[0], y[0])]
        R["both_stalls"] = len(both); R["both_stall_len_median"] = rnd(S.median(STALL_S + e - s for s, e in both)) if both else None
        R["both_stalls_per_min"] = rnd(60.0 * len(both) / (t1 - t0))
    fx = collections.defaultdict(list); fcam = collections.defaultdict(collections.Counter)
    for r in rows(os.path.join(p, "bridge", "fixes.csv")):
        t = fl(r, "t")
        if t is not None and t0 <= t <= t1: fx[r["robot"]].append(t); fcam[r["robot"]][r["cams"]] += 1
    R["fixes"] = {}
    for rb, ts in fx.items():
        ts.sort(); g = [b - a for a, b in zip(ts, ts[1:])]
        if g: R["fixes"][rb] = {"n": len(ts), "gap_median": rnd(S.median(g)), "gap_p90": rnd(q(g, 0.9)), "gap_max": rnd(max(g), 1),
                                "over1_time_pct": rnd(100.0 * sum(x for x in g if x > 1.0) / (t1 - t0), 1), "by_cam": dict(fcam[rb])}
    fu = {}
    for r in rows(os.path.join(p, "run", "follow.csv")):
        try: fu[(r["robot"], r["run"])] = (int(r["fixes_used"]), int(r["fixes_rejected"]))
        except (ValueError, KeyError): pass
    u = collections.Counter(); rj = collections.Counter()
    for (rb, _), (a, b) in fu.items(): u[rb] += a; rj[rb] += b
    R["fix_used_refused"] = {rb: (u[rb], rj[rb]) for rb in u}
    rp = [e for e in L if e.get("event") == "replan"]
    c = [e["cpu_s"] for e in rp]
    R["replan"] = {"n": len(c), "median": rnd(S.median(c)) if c else None, "p90": rnd(q(c, 0.9)), "max": rnd(max(c)) if c else None,
                   "busy_pct": rnd(100.0 * sum(c) / (t1 - t0), 0), "over_budget": sum(1 for e in rp if e["cpu_s"] > e.get("budget_s", 2.0)),
                   "by_points": sorted((e.get("points_planned"), rnd(e["cpu_s"], 1)) for e in rp if e.get("points_planned") is not None)}
    R["first_plan_s"] = (lambda m: float(m.group(1)) if m else None)(re.search(r"planned in ([0-9.]+) s", so))
    lost = [e["lost_s"] for e in L if e.get("event") == "tracker_back" and e["t"] >= t0]
    R["tracker_lost"] = {"n": len(lost), "median": rnd(S.median(lost)) if lost else None, "max": rnd(max(lost)) if lost else None}
    of = [e["age_s"] for e in L if e.get("event") == "old_fix_carried"]
    R["old_fix"] = {"n": len(of), "age_max": rnd(max(of)) if of else None}
    out.append(R)
if len(sys.argv) > 2: json.dump(out, open(sys.argv[2], "w"), indent=1)
for R in out:
    print("==", R["clock"], "plan %d s, %d robot(s), recorder pid %s" % (R["plan_s"], R["robots"], R["camrec_pid"]))
    print("   cores %s %s %s %s | recorder %s%% tag_bridge %s%% run_plan %s%% (95%% %s) | temp %s (95%% %s) C | %s MHz | throttled now %d of %d rows %s | card %s GB" % (
        R["core0"], R["core1"], R["core2"], R["core3"], R["recorder_cpu"], R["tag_bridge_cpu"], R["run_plan_cpu"], R["run_plan_cpu_p95"], R["temp_c"], R["temp_c_p95"], R["arm_mhz"],
        R["throttled_now_rows"], R["rows"], R["throttle_words"], R["card_gb_end"]))
    print("   recorder lag:", R["lag_overhead"], R["lag_side"])
    for cam, v in R["cams"].items(): print("   %s:" % cam, v)
    print("   both cameras stale at once:", R.get("both_stalls"), "median len", R.get("both_stall_len_median"), "per min", R.get("both_stalls_per_min"))
    print("   fixes:", R["fixes"], "used/refused", R["fix_used_refused"])
    print("   replans:", R["replan"], "first plan", R["first_plan_s"], "| tracker lost", R["tracker_lost"], "| old fixes", R["old_fix"])
