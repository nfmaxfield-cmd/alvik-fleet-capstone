#!/usr/bin/env python3
"""Score a track_live.py run on a simulated recording against the simulator's truth.

    python tools/sim/simulate.py --out sim_out --video --robots Left,Center,Right --config tools/sim/configs/real_oct18.json
    python tools/live/track_live.py --replay sim_out/recording --log sim_out/live
    python tools/live/eval_live.py --messages sim_out/live/messages.jsonl --sim sim_out

For every fix: where the simulator says that robot (the wheel-axle midpoint,
the point fixes give) really was when the picture was taken. Reports per robot:
fixes, fixes per second while it was in view, running and blinking (lights on at
least once that second), share of those seconds with a fix, position and heading error, fix age (picture -> sent), and
WRONG NAMES: fixes closer than 5 cm to a different robot than the one named,
or naming a robot that wasn't running there. Writes eval.json beside the messages.
"""
import argparse
import bisect
import csv
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Truth:
    """Per robot, poses at known times; at(t) interpolates within max_gap."""

    def __init__(self, rows, max_gap=0.06):
        self.by = defaultdict(list)
        for t, robot, x, y, th in rows:
            self.by[robot].append((t, x, y, th))
        for v in self.by.values():
            v.sort()
        self.ts = {r: [q[0] for q in v] for r, v in self.by.items()}
        self.max_gap = max_gap

    def at(self, robot, t):
        v = self.by.get(robot)
        if not v:
            return None
        ts = self.ts[robot]
        i = bisect.bisect_left(ts, t)
        if i == 0 or i >= len(ts):
            return None
        a, b = v[i - 1], v[i]
        if b[0] - a[0] > self.max_gap:
            return None
        w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return (a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), a[3] + w * wrap180(b[3] - a[3]))


def load_sim(sim):
    with open(os.path.join(sim, "sim_info.json")) as f:
        info = json.load(f)
    cam = info["config"]["camera"]
    shift = cam["pi_clock_offset_s"] + cam["capture_delay_frames"] / cam["fps"]
    rows = []
    with open(os.path.join(sim, "truth.csv"), newline="") as f:
        for r in csv.DictReader(f):
            if r.get("lifted") not in (None, "", "0"):
                continue
            rows.append((float(r["t_laptop"]), r["robot"], float(r["x_cm"]), float(r["y_cm"]), float(r["heading_deg"])))
    # seconds each robot was in view (camera_truth) while running (truth.csv has rows only during runs)
    # with its lights on at least once in that second (blinking; the lights are dark during
    # blocking calls such as the library's move())
    truth = Truth(rows)
    in_view = defaultdict(set)
    ct = os.path.join(sim, "camera_truth.csv")
    if os.path.exists(ct):
        with open(ct, newline="") as f:
            for r in csv.DictReader(f):
                t = float(r["t_wall"]) - shift
                if r.get("led_on") == "1" and truth.at(r["robot"], t) is not None:
                    in_view[r["robot"]].add(int(math.floor(t)))
    return truth, shift, in_view


def score(msgs, truth, shift, in_view=None, near_cm=5.0):
    """msgs: fix dicts with robot, t_pi, x, y, th. Returns (per-robot dict, wrong list)."""
    out = defaultdict(lambda: dict(n=0, err=[], herr=[], age=defaultdict(list), secs=set(), wrong=0, unverified=0))
    wrong = []
    names = set(truth.by)
    for m in msgs:
        if m.get("type") != "fix":
            continue
        t = m["t_pi"] - shift
        r = out[m["robot"]]
        r["n"] += 1
        if "t_sent" in m and "t" in m:
            r["age"][m.get("path", "?")].append((m["t_sent"] - m["t"]) * 1000.0)
        r["secs"].add(int(math.floor(t)))
        own = truth.at(m["robot"], t)
        d_own = math.hypot(m["x"] - own[0], m["y"] - own[1]) if own else None
        others = []
        for o in names - {m["robot"]}:
            q = truth.at(o, t)
            if q is not None:
                others.append((math.hypot(m["x"] - q[0], m["y"] - q[1]), o))
        closer = [o for d, o in others if d < near_cm and (d_own is None or d < d_own)]
        if closer:
            r["wrong"] += 1
            wrong.append(dict(m, true_robot=closer[0]))
            continue
        if own is None:
            r["unverified"] += 1
            continue
        r["err"].append(d_own)
        if m.get("th") is not None:
            r["herr"].append(abs(wrap180(m["th"] - own[2])))
    res = {}
    for name, r in sorted(out.items()):
        e, h = np.array(r["err"]), np.array(r["herr"])
        view = in_view.get(name, set()) if in_view else set()
        res[name] = dict(
            fixes=r["n"], wrong_name=r["wrong"], unverified=r["unverified"],
            err_cm=dict(median=_q(e, 50), p95=_q(e, 95), max=_q(e, 100), rms=round(float(np.sqrt(np.mean(e ** 2))), 3) if len(e) else None),
            heading_err_deg=dict(median=_q(h, 50), p95=_q(h, 95), max=_q(h, 100)),
            over_3cm=int((e > 3).sum()) if len(e) else 0,
            seconds_in_view_blinking=len(view),
            fixes_per_s_in_view=round(r["n"] / len(view), 2) if view else None,
            seconds_with_fix=round(len(r["secs"] & view) / len(view), 3) if view else None,
            fix_age_ms={k: dict(median=_q(np.array(v), 50), p95=_q(np.array(v), 95), n=len(v)) for k, v in r["age"].items()})
    for name in (in_view or {}):
        if name not in res:
            res[name] = dict(fixes=0, seconds_in_view_blinking=len(in_view[name]))
    return res, wrong


def _q(a, q):
    return round(float(np.percentile(a, q)), 3) if len(a) else None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--messages", required=True)
    ap.add_argument("--sim", required=True, help="simulate.py --out folder")
    args = ap.parse_args(argv)
    truth, shift, in_view = load_sim(args.sim)
    with open(args.messages) as f:
        msgs = [json.loads(line) for line in f if line.strip()]
    res, wrong = score(msgs, truth, shift, in_view)
    n_s = sum(1 for m in msgs if m.get("type") == "sighting")
    out = dict(robots=res, wrong_names=len(wrong), wrong_examples=wrong[:10], sightings_held_back=n_s)
    print(json.dumps(out, indent=1))
    with open(os.path.join(os.path.dirname(os.path.abspath(args.messages)), "eval.json"), "w") as f:
        json.dump(out, f, indent=1)
    return 1 if wrong else 0


if __name__ == "__main__":
    sys.exit(main())
