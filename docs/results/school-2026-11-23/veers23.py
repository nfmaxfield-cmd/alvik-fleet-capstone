#!/usr/bin/env python3
"""Turn-backs (the 'veer' stage) in the 23 Nov school runs, counted the way the 20-run simulator study counted them.
Read-only. Usage: python3 -I veers23.py <jayruns23 folder> [out.json]
  stall          one robot's camera loss: coordinator 'tracker_back' gives its end and length (lost_s)
  moving         the robot's own pose (run/follow.csv) moved faster than 3 cm/s over the 0.6 s round the stall's start
                 (the simulator study used the same 3 cm/s [chosen there] on true positions; here the truth is not
                 known while the camera is away, so the robot's own pose stands in)
  turn-back      a 'veer' event; 'at the first moving stall' = a veer by that robot between the first moving stall's
                 start and 0.5 s after its end (as in turnback-20runs-nov20/scripts/first_stall.py)
  drove back     a 'recover_start' whose reason is the veer (the robot really turned and drove ~10 cm back)
  stayed         'veer_not_at_point' (e23 fix I: a robot standing in its dwell stays)
Changelog: 2026-11-24 first version."""
import bisect, csv, json, math, os, sys, collections, statistics as S
ROOT = sys.argv[1]; MOVING = 3.0; out = []
for d in sorted(os.listdir(ROOT)):
    cj = os.path.join(ROOT, d, "run", "coordinator.jsonl")
    if not os.path.exists(cj): continue
    L = []
    for l in open(cj, errors="replace"):
        try: L.append(json.loads(l))
        except ValueError: pass
    t0 = next((e["t"] for e in L if e.get("event") == "message" and "plan plan-" in e.get("text", "") and "started" in e["text"]), None)
    if t0 is None: continue
    t1 = L[-1]["t"]
    own = collections.defaultdict(list)
    for r in csv.DictReader(open(os.path.join(ROOT, d, "run", "follow.csv"), newline="")):
        try:
            if r["state"] != "nopose": own[r["robot"]].append((float(r["t_recv"]), float(r["x_cm"]), float(r["y_cm"]), r["state"]))
        except ValueError: pass
    ts = {k: [x[0] for x in v] for k, v in own.items()}
    def at(rb, t):
        v = own[rb]; i = min(max(bisect.bisect_left(ts[rb], t), 0), len(v) - 1); return v[i]
    def speed(rb, t):
        if rb not in own: return 0.0
        a, b = at(rb, t - 0.3), at(rb, t + 0.3)
        return 0.0 if b[0] <= a[0] else math.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0])
    stalls = sorted((e["t"] - e["lost_s"], e["t"], e["robot"], e["lost_s"]) for e in L if e.get("event") == "tracker_back" and e["t"] - e["lost_s"] >= t0)
    veers = [e for e in L if e.get("event") == "veer" and e["t"] >= t0]
    drove = [e for e in L if e.get("event") == "recover_start" and "veer" in e.get("why", "") and e["t"] >= t0]
    stayed = [e for e in L if e.get("event") == "veer_not_at_point" and e["t"] >= t0]
    V = []
    for v in veers:
        sp = speed(v["robot"], v["t"] - 0.3)           # just before the veer was called
        st = next((s for s in stalls if s[2] == v["robot"] and s[0] - 0.2 <= v["t"] <= s[1] + 0.5), None)
        edge = min(v["own"][0], v["own"][1], 396.2 - v["own"][0], 167.6 - v["own"][1])
        V.append({"t_run": round(v["t"] - t0, 1), "robot": v["robot"], "speed": round(sp, 1), "moving": sp > MOVING, "state": at(v["robot"], v["t"])[3],
                  "since_lost_cm": v.get("dist_cm"), "in_stall_len": round(st[3], 2) if st else None, "edge_cm": round(edge, 1),
                  "drove_back": any(abs(x["t"] - v["t"]) < 1.0 and x["robot"] == v["robot"] for x in drove)})
    mstalls = [s for s in stalls if speed(s[2], s[0]) > MOVING]
    def veered(s): return any(v["robot"] == s[2] and s[0] <= v["t"] <= s[1] + 0.5 for v in veers)
    first = mstalls[0] if mstalls else None
    R = {"run": d[:15], "clock": d[9:11] + ":" + d[11:13], "plan_s": round(t1 - t0), "stalls": len(stalls), "moving_stalls": len(mstalls),
         "stall_len_median": round(S.median(s[3] for s in stalls), 2) if stalls else None,
         "moving_stall_len": [round(s[3], 2) for s in mstalls],
         "moving_stalls_with_veer": sum(1 for s in mstalls if veered(s)),
         "first_moving_stall_t": round(first[0] - t0, 1) if first else None, "first_len": round(first[3], 2) if first else None,
         "veer_at_first": veered(first) if first else None,
         "veers": len(veers), "veers_moving": sum(1 for v in V if v["moving"]), "veers_standing": sum(1 for v in V if not v["moving"]),
         "drove_back": len(drove), "stayed_at_point": len(stayed), "veer_list": V,
         "veered_stall_lens": [round(s[3], 2) for s in mstalls if veered(s)]}
    out.append(R)
    print("%s plan %4d s | stalls %3d (median %.2f s), moving %3d, of those with a turn-back %d | first moving stall at %s s (%.2f s): turn-back %s | veers %d (moving %d, standing %d), drove back %d, stayed at point %d" % (
        R["clock"], R["plan_s"], R["stalls"], R["stall_len_median"] or 0, R["moving_stalls"], R["moving_stalls_with_veer"], R["first_moving_stall_t"], R["first_len"] or 0, R["veer_at_first"],
        R["veers"], R["veers_moving"], R["veers_standing"], R["drove_back"], R["stayed_at_point"]))
    for v in V: print("      ", v)
tot = collections.Counter()
for R in out:
    for k in ("stalls", "moving_stalls", "moving_stalls_with_veer", "veers", "veers_moving", "veers_standing", "drove_back", "stayed_at_point", "plan_s"): tot[k] += R[k]
    tot["first_yes"] += 1 if R["veer_at_first"] else 0; tot["first_n"] += 1 if R["veer_at_first"] is not None else 0
print(dict(tot), "veers per minute %.2f" % (60.0 * tot["veers"] / tot["plan_s"]))
if len(sys.argv) > 2: json.dump({"runs": out, "total": dict(tot)}, open(sys.argv[2], "w"), indent=1)
