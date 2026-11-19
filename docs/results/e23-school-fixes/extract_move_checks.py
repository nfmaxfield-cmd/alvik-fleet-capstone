#!/usr/bin/env python3
"""(e23) The real "did NOT move" / "not physically possible" verdicts of Jay-Sun's school runs (18 Nov), as a small
test fixture: for each verdict the step as planned, the robot's own pose when it started and ended the step, the
times, and the camera fixes of the seconds round it (the picture's own time). Read-only on the run folders.
    python3 -I extract_move_checks.py <jayruns folder> <out.json>"""
import csv, glob, json, os, re, sys
root, out = sys.argv[1], sys.argv[2]
cases = []
for run in sorted(glob.glob(os.path.join(root, "20261118-*"))):
    fc = os.path.join(run, "run", "coordinator.jsonl")
    if not os.path.exists(fc):
        continue
    evs = []
    for ln in open(fc):
        try:
            evs.append(json.loads(ln))
        except ValueError:
            pass
    fixes = {}
    fx = os.path.join(run, "bridge", "fixes.csv")
    if os.path.exists(fx):
        for r in csv.DictReader(open(fx)):
            try:
                fixes.setdefault(r["robot"], []).append((float(r["t"]), float(r["x"]), float(r["y"])))
            except (ValueError, KeyError):
                pass
    sent, started = {}, {}
    for e in evs:
        ev = e.get("event")
        if ev == "step_sent":
            sent[(e["robot"], e["id"])] = e
        elif ev == "step_report" and e.get("state") == "started":
            started[(e["robot"], e["id"])] = e
        elif ev == "step_report" and e.get("state") == "done":
            last_done = e
            sent.setdefault(("done", e["robot"]), {})[e["id"]] = e
        elif ev == "did_not_move":
            n = e["robot"]
            dn = max((d for d in sent.get(("done", n), {}).values() if d["t"] <= e["t"] + 0.01), key=lambda d: d["t"],
                     default=None)
            if dn is None:
                continue
            ss, st = sent.get((n, dn["id"])), started.get((n, dn["id"]))
            m = re.match(r"fs \d+ D ([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+) (\d)", (ss or {}).get("text", ""))
            if not (ss and st and m):
                continue
            x0, y0, x1, y1, th, v, rev = (float(g) for g in m.groups())
            cam = [[round(t, 3), x, y] for t, x, y in fixes.get(n, []) if st["t"] - 3.0 <= t <= dn["t"] + 3.0]
            cases.append({"run": os.path.basename(run), "robot": n, "verdict": e["verdict"], "score": e.get("score"),
                          "used": e.get("used"), "length_cm": e["length_cm"], "took_s": e["took_s"],
                          "step": {"from": [x0, y0, th], "to": [x1, y1, th], "speed": v, "reverse": bool(rev)},
                          "t_started": round(st["t"], 3), "t_done": round(dn["t"], 3),
                          "own_start": [st["x"], st["y"], st["th"]], "own_end": [dn["x"], dn["y"], dn["th"]],
                          "camera": cam})
json.dump({"what": __doc__.strip().splitlines()[0], "cases": cases}, open(out, "w"), indent=0)
import math
for c in cases:
    end = math.hypot(c["step"]["to"][0] - c["own_end"][0], c["step"]["to"][1] - c["own_end"][1])
    left = math.hypot(c["step"]["to"][0] - c["own_start"][0], c["step"]["to"][1] - c["own_start"][1])
    print(c["run"][9:15], c["robot"], c["verdict"].ljust(10), "planned %5.1f  left at start %5.1f  took %5.2f  own end off %4.1f  cam rows %d" % (
        c["length_cm"], left, c["took_s"], end, len(c["camera"])))
print(len(cases))
