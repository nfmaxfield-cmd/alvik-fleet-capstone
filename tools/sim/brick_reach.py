#!/usr/bin/env python3
"""How near a robot gets to a real brick before its distance-sensor stop halts it, per stop rule (Nick, 2026-10-29:
"how much earlier/later does a robot now stop for a real brick?"). The simulated firmware (robot.py + fw_follow.py:
motion.h's stop rule, follow.h's limit, the sensor's ~15 new readings a second, its noise and lag, braking) drives a
straight follow step at a 20 x 9 cm brick lying across its path; the gap is measured on the TRUE pose: the brick's
face to the robot's front face, once it has stopped.

    python3 tools/sim/brick_reach.py [--seeds 12] [--out file.json]

Two cases: "driving" (the brick 40 cm ahead, the robot at cruise speed when it gets there) and "standing" (the brick
already there, 9-14 cm ahead, when the step starts: the case right after a turn). Limits 12 / 10 / 9 / 8 cm,
readings in a row 2 / 3, speeds 7.5 / 10 / 13 cm/s.
"""
import argparse
import json
import math
import os
import random
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import config  # noqa: E402
import robot  # noqa: E402

FRONT_CM = 4.75     # fleetmap: the front face 4.75 cm ahead of the wheel axle


class ScriptRadio:
    def __init__(self, cmds):
        self.cmds = sorted(cmds)
        self.out = []

    def poll(self, rb):
        t = rb.t_at(rb.ms)
        if self.cmds and self.cmds[0][0] <= t:
            return self.cmds.pop(0)[1]
        return None

    def transmit(self, rb, text):
        self.out.append(text)


def one(limit, n, speed, seed, case="driving"):
    cfg = config.load_config()
    r = robot.Robot("Left", cfg, random.Random(seed), config.firmware_params(), "t", 0.0, 0x1234, "ip")
    r.idle_until(5.0)
    x0 = 20.0
    r.x, r.y, r.th = x0, 30.0, 0.0
    r.fw_yaw_sign = 1
    rng = random.Random(seed * 31 + 7)
    gap0 = 40.0 if case == "driving" else rng.uniform(9.0, 14.0)
    bx = x0 + FRONT_CM + gap0                      # the brick's near face
    r.world_polys = [[(bx, 20.0), (bx + 9.0, 20.0), (bx + 9.0, 40.0), (bx, 40.0)]]
    cmds = [(5.5, "set follow_look_cm %g" % limit), (5.52, "set obs_n %d" % n), (5.6, "run follow"),
            (8.0, "fpose %.2f 30 0 0.3 1" % x0), (8.1, "fs 1 D %.2f 30 %.2f 30 0 %.1f 0 -" % (x0, x0 + 90.0, speed)),
            (25.0, "stop")]
    r.radio = ScriptRadio(cmds)
    while True:
        c = r.radio.poll(r)
        if c:
            r.handle_command(c, False)
        if r.pending >= 0:
            r.pending = -1
            r.run_experiment("follow")
            break
        r.advance(10)
    steps = [ln.split(",") for ln in r.radio.out if ln.startswith("D,STEP,")]
    st = [v for v in steps if v[3] == "1" and v[4] in ("failed", "done")]
    why = st[0][-1] if st else "?"
    gap = bx - (r.x + FRONT_CM * math.cos(r.th))
    return dict(limit=limit, n=n, speed=speed, seed=seed, case=case, gap_cm=round(gap, 2), start_gap_cm=round(gap0, 2),
                stopped=why.startswith("obstacle"), why=why[:60], touched=gap <= 0.0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seeds", type=int, default=12)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    rows = []
    for case in ("driving", "standing"):
        for speed in ((7.5, 10.0, 13.0) if case == "driving" else (10.0,)):
            for limit in (12.0, 10.0, 9.0, 8.0):
                for n in (2, 3):
                    for s in range(a.seeds):
                        rows.append(one(limit, n, speed, s, case))
    print("%-9s %5s %5s %3s  %s" % ("case", "cm/s", "limit", "n", "gap left at the stop, cm: median (min-max), stopped"))
    summ = []
    keys = sorted({(r["case"], r["speed"], r["limit"], r["n"]) for r in rows}, key=lambda k: (k[0], k[1], -k[2], k[3]))
    for k in keys:
        g = [r for r in rows if (r["case"], r["speed"], r["limit"], r["n"]) == k]
        gaps = [r["gap_cm"] for r in g]
        s = dict(case=k[0], speed=k[1], limit=k[2], n=k[3], median=round(statistics.median(gaps), 2),
                 min=min(gaps), max=max(gaps), stopped=sum(r["stopped"] for r in g), runs=len(g),
                 touched=sum(r["touched"] for r in g))
        summ.append(s)
        print("%-9s %5.1f %5.1f %3d  %5.2f (%.2f-%.2f), %d/%d stopped, %d touched" % (
            k[0], k[1], k[2], k[3], s["median"], s["min"], s["max"], s["stopped"], s["runs"], s["touched"]))
    if a.out:
        with open(a.out, "w") as f:
            json.dump({"rows": rows, "summary": summ}, f, indent=1)


if __name__ == "__main__":
    main()
