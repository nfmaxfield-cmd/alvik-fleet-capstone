#!/usr/bin/env python3
"""Spells where the tracking layer NAMED a robot's camera sighting but its own pose was too far for its own filter to
take the fix (over the firmware's fix gate, about 6 cm: e17 section 3, item 1e). e17 turns only eliminations and
wiggles into CORRECTIONS (an fpose), so a robot named all along but this far off is not corrected by e17. Logs only.

    python3 named_far.py RUNS_ROOT REPLAY_OUT OUT.json

For each run, robot and mode (off / on): the spells of consecutive named sightings (no break over 2 s) with own vs
camera over FIX_GATE_CM, how long, how far, and whether the robot stood (FOLLOW idle) through it.

Changelog
  2026-11-12 v1  first version (replay-all-nov12)
"""
import bisect
import csv
import glob
import json
import math
import os
import sys

VERSION = "named-far-2026-11-12v1"
FIX_GATE_CM = 6.0     # as e17: firmware fix_gate 4 sigmas with var 1 cm2 and sigma 1.5 (about 6 cm); not chosen here
BREAK_S = 2.0
MIN_S = 5.0           # CHOSEN: report spells from 5 s (a fresh fix and the camera settling take about 1 s)


def main(root, replay_out, out_path):
    res = {}
    for R in sorted(glob.glob(os.path.join(root, "*-freeroam"))):
        run = os.path.basename(R)
        if not os.path.exists(os.path.join(replay_out, f"{run}-on-w1.json")):
            continue
        own, cam = {}, {}
        for r in csv.DictReader(open(R + "/follow.csv")):
            if r["state"] != "nopose":
                own.setdefault(r["robot"], []).append((float(r["t_recv"]), float(r["x_cm"]), float(r["y_cm"]),
                                                       r["state"]))
        for r in csv.DictReader(open(R + "/bridge/track.csv")):
            if r["status"] == "seen":
                cam.setdefault(r["robot"], {})[round(float(r["t"]), 2)] = (float(r["x"]), float(r["y"]))
        res[run] = {}
        for mode in ("off", "on"):
            j = json.load(open(os.path.join(replay_out, f"{run}-{mode}-w1.json")))
            out = []
            for n, ts in j["named_t"].items():
                O = own.get(n)
                if not O:
                    continue
                ots = [q[0] for q in O]
                far = []
                for t in ts:
                    c = cam.get(n, {}).get(round(t, 2))
                    if c is None:
                        continue
                    i = min(len(O) - 1, max(0, bisect.bisect(ots, t) - 1))
                    g = math.hypot(O[i][1] - c[0], O[i][2] - c[1])
                    far.append((t, g, O[i][3] == "idle"))
                spell = []
                for k, (t, g, idle) in enumerate(far + [(1e18, 0.0, False)]):
                    if g > FIX_GATE_CM and (not spell or t - spell[-1][0] <= BREAK_S):
                        spell.append((t, g, idle))
                        continue
                    if spell and spell[-1][0] - spell[0][0] >= MIN_S:
                        out.append(dict(robot=n, t=round(spell[0][0], 2), dur_s=round(spell[-1][0] - spell[0][0], 1),
                                        gap_med_cm=round(sorted(q[1] for q in spell)[len(spell) // 2], 1),
                                        stood=all(q[2] for q in spell)))
                    spell = [(t, g, idle)] if g > FIX_GATE_CM else []
            res[run][mode] = dict(spells=len(out), total_s=round(sum(s["dur_s"] for s in out), 1),
                                  longest_s=max([s["dur_s"] for s in out], default=0.0),
                                  stood_s=round(sum(s["dur_s"] for s in out if s["stood"]), 1), list=out[:100])
        print(run, {m: {k: v for k, v in d.items() if k != "list"} for m, d in res[run].items()})
    json.dump(dict(version=VERSION, runs=res), open(out_path, "w"), indent=1)


if __name__ == "__main__":
    main(*sys.argv[1:4])
