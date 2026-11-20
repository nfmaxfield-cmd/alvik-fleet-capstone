#!/usr/bin/env python3
"""Turn-back at the first moving stall, for one run folder (made by seed_runs.py).
Stall = overhead camera 'camera_no_pictures' .. next 'camera_pictures' (coordinator.jsonl), after the run's clock starts (first event with t_run).
Stall start = t(no_pictures) - age_s. A robot is MOVING in a stall if its simulator-truth speed over
[start-0.3 s, start+0.3 s] is > 3 cm/s [chosen: drive speed is 10 cm/s; turning in place gives ~0].
First moving stall = first stall with at least one moving robot. Turn-back there = a 'veer' event by a moving robot
with t in [start, pictures back + 0.5 s].
Prints JSON: {first_moving_stall_t_run, moving, veer_at_first: bool, stalls, veers}"""
import bisect, csv, json, math, os, sys
def analyse(folder):
    run = os.path.join(folder, "run")
    ev = []
    for ln in open(os.path.join(run, "coordinator.jsonl")):
        try: ev.append(json.loads(ln))
        except ValueError: pass
    tr = {}
    for r in csv.DictReader(open(os.path.join(run, "sim", "truth.csv"))):
        tr.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"])))
    def pos(n, t):
        v = tr[n]; i = bisect.bisect_left([q[0] for q in v], t); i = min(max(i, 0), len(v) - 1); return v[i]
    def speed(n, t):
        a, b = pos(n, t - 0.3), pos(n, t + 0.3)
        return 0.0 if b[0] <= a[0] else math.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0])
    t_first_step = min((e["t"] for e in ev if e.get("event") == "step_sent"), default=None)
    t_run0 = None
    for e in ev:
        if "t_run" in e: t_run0 = e["t"] - e["t_run"]; break
    stalls, cur = [], None
    for e in ev:
        if e.get("cam") != "overhead": continue
        if e.get("event") == "camera_no_pictures" and cur is None:
            cur = e["t"] - (e.get("age_s") or 0)
        elif e.get("event") == "camera_pictures" and cur is not None:
            stalls.append((cur, e["t"])); cur = None
    veers = [e for e in ev if e.get("event") == "veer"]
    out = {"stalls": 0, "first_moving_stall_t_run": None, "moving": [], "veer_at_first": False, "n_veers": len(veers)}
    for s, b in stalls:
        if t_run0 is None or s < t_run0: continue
        out["stalls"] += 1
        mv = [n for n in tr if speed(n, s) > 3.0]
        if not mv: continue
        out["first_moving_stall_t_run"] = round(s - t_run0, 1) if t_run0 else None
        out["moving"] = {n: round(speed(n, s), 1) for n in mv}
        out["veer_at_first"] = any(v.get("robot") in mv and s <= v["t"] <= b + 0.5 for v in veers)
        break
    return out
if __name__ == "__main__":
    for f in sys.argv[1:]:
        print(os.path.basename(f.rstrip("/")), json.dumps(analyse(f)))
