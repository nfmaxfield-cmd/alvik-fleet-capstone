#!/usr/bin/env python3
"""For every "did NOT move" / "not physically possible" verdict in run/coordinator.jsonl: where the plan said the drive
step starts, where the robot's own pose really was when it started the step, how far it moved, and how far from the
step's target it ended. Also: re-plan computing time per run and the age of each plan when it reached the robots.
Usage: python3 -I notmoved.py <jayruns folder>"""
import json, math, os, sys, re, collections
ROOT = sys.argv[1]
tot = collections.Counter()
for d in sorted(os.listdir(ROOT)):
    f = os.path.join(ROOT, d, "run", "coordinator.jsonl")
    if not os.path.exists(f): continue
    L = []
    for l in open(f):
        try: L.append(json.loads(l))
        except ValueError: pass
    t0 = next((r["t"] for r in L if r.get("event") == "message" and "plan plan-" in r.get("text", "") and "started" in r["text"]), None)
    if t0 is None: continue
    sent = {}      # (robot, id) -> (text, t, plan, i)
    last_sent = {}
    rep = collections.defaultdict(dict)
    out = []
    for r in L:
        e = r.get("event")
        if e == "step_sent":
            sent[(r["robot"], r["id"])] = r; last_sent[(r["robot"], r.get("plan"), r["i"])] = r
        elif e == "step_report":
            rep[(r["robot"], r["id"])][r["state"]] = r
        elif e == "did_not_move":
            # the newest sent step with this robot and index i
            c = [s for (rb, pl, i), s in last_sent.items() if rb == r["robot"] and i == r["i"] and s["t"] <= r["t"]]
            if not c: continue
            s = max(c, key=lambda q: q["t"])
            p = s["text"].split()
            if len(p) < 7 or p[2] != "D": continue
            x0, y0, x1, y1 = map(float, p[3:7])
            rr = rep[(r["robot"], s["id"])]
            st, dn = rr.get("started"), rr.get("done")
            if not st or not dn: continue
            stale = math.hypot(st["x"] - x0, st["y"] - y0)
            moved = math.hypot(dn["x"] - st["x"], dn["y"] - st["y"])
            left = math.hypot(dn["x"] - x1, dn["y"] - y1)
            kind = "arrived (false alarm)" if left <= 3.0 else "short of target"
            out.append((round(r["t"] - t0, 1), r["robot"], r["verdict"], r["length_cm"], r["took_s"], round(stale, 1), round(moved, 1), round(left, 1), kind))
            tot[(r["verdict"], kind)] += 1
    rp = [r for r in L if r.get("event") == "replan"]
    cpu = [r["cpu_s"] for r in rp]
    dur = L[-1]["t"] - t0
    src = collections.Counter(r["source"] for r in rp)
    print("== %s run %.0f s; re-plans %d (%s); planner busy %.0f s = %.0f %% of the run; median %.1f s, max %.1f s" % (
        d[:15], dur, len(rp), dict(src), sum(cpu), 100 * sum(cpu) / max(dur, 1), sorted(cpu)[len(cpu) // 2] if cpu else 0, max(cpu) if cpu else 0))
    for o in out:
        print("   t %6.1f %s %-10s plan length %5.1f cm, took %5.2f s | plan's start was %5.1f cm from where the robot was | it moved %5.1f cm | ended %5.1f cm from the step's target: %s" % o)
print(dict(tot))
