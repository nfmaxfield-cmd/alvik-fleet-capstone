#!/usr/bin/env python3
"""For each "did NOT move" verdict (not the 'impossible' ones): how far the robot's own pose moved during the step and
how far the newest camera reading had moved by the time the step was reported done (labelled_step records).
Usage: python3 -I notmoved_cam.py <jayruns folder>"""
import json, math, os, sys
ROOT = sys.argv[1]
n = tot_cam_small = 0
for d in sorted(os.listdir(ROOT)):
    f = os.path.join(ROOT, d, "run", "coordinator.jsonl")
    if not os.path.exists(f): continue
    L = [json.loads(l) for l in open(f) if l.strip().startswith("{")]
    lab = {}
    for k, r in enumerate(L):
        if r.get("event") == "labelled_step" and r.get("phase") == "travel":
            lab[(r["robot"], r["plan"], r["i"], r["state"])] = r
        if r.get("event") == "did_not_move" and r["verdict"] == "not_moved":
            # the labelled_step "done" right after this event
            dn = next((q for q in L[k:k + 12] if q.get("event") == "labelled_step" and q["robot"] == r["robot"] and q["i"] == r["i"] and q["state"] == "done"), None)
            if not dn: continue
            st = lab.get((r["robot"], dn["plan"], dn["i"], "started"))
            if not st or not st.get("cam") or not dn.get("cam"): continue
            own = math.hypot(dn["pose"][0] - st["pose"][0], dn["pose"][1] - st["pose"][1])
            cam = math.hypot(dn["cam"][0] - st["cam"][0], dn["cam"][1] - st["cam"][1])
            n += 1; tot_cam_small += cam < 0.5 * r["length_cm"]
            print("%s %s step %5.1f cm in %.2f s: own pose moved %5.1f cm, newest camera reading had moved %5.1f cm (score %.2f)" % (
                d[:15], r["robot"], r["length_cm"], r["took_s"], own, cam, r["score"]))
print("%d verdicts; in %d the newest camera reading had moved less than half the step" % (n, tot_cam_small))
