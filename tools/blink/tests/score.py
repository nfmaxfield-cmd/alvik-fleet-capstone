"""Compare blink_detections rows with a synthetic or simulated truth.

truth rows: dict(frame, t, robot, x, y, led) with x, y the true LED midpoint
(px) and led the exposure-averaged LED level (0..1).
"""
import csv
import math
from collections import defaultdict

import numpy as np


def read_truth(path):
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            out.append(dict(frame=int(r["frame"]), t=float(r["t_wall"]), robot=r["robot"], x=float(r["x_px"]),
                            y=float(r["y_px"]), led=float(r.get("led") or r.get("led_on") or 0)))
    return out


def evaluate(rows, truth, blinking=None, match_px=12.0, lit_level=0.5, names=None):
    """rows: blink rows (dicts from blink.read_blink_csv); truth: read_truth().
    blinking: {robot: [(t0, t1), ...]} wall-clock spans where each robot blinked
    (default: from its first to its last lit truth frame).
    Returns dict with:
      robot_seconds: blinking robot-seconds in view (seconds with 2+ frames showing the robot's light lit)
      correct_seconds: of those, seconds where the robot's light was reported
                       with its own name and never with another name
      accuracy: correct / robot_seconds
      pos_err: list of px errors (lit rows matched to a truth light)
      false_rows: NAMED rows not within match_px of any robot's light
      unnamed_rows: rows reported without a name (tracks whose code was not readable)
      wrong_rows: rows matched to a robot but carrying another robot's name
      lit_frames / found_frames: truth frames with led >= lit_level, and how many had a row
    """
    names = names or {}
    tf = defaultdict(list)
    for r in truth:
        tf[r["frame"]].append(r)
    if blinking is None:
        blinking = {}
        lit = defaultdict(list)
        for r in truth:
            if r["led"] >= lit_level:
                lit[r["robot"]].append(r["t"])
        for k, ts in lit.items():
            blinking[k] = [(min(ts), max(ts))]
    pos_err, false_rows, wrong_rows, unnamed = [], 0, 0, 0
    per_sec = defaultdict(lambda: [0, 0])     # (robot, second) -> [right, wrong]
    found = set()
    for r in rows:
        if not r["robot"]:                    # reported, but no name given (code not readable)
            unnamed += 1
            continue
        cand = tf.get(r["frame"], [])
        best, bd = None, 1e9
        for c in cand:
            d = math.hypot(c["x"] - r["x"], c["y"] - r["y"])
            if d < bd:
                best, bd = c, d
        if best is None or bd > match_px:
            false_rows += 1
            continue
        key = (best["robot"], int(math.floor(best["t"])))
        if r["robot"] == names.get(best["robot"], best["robot"]):
            per_sec[key][0] += 1
            if r.get("lit", 1):
                pos_err.append(bd)
                found.add((r["frame"], best["robot"]))
        else:
            per_sec[key][1] += 1
            wrong_rows += 1
    lit_count = defaultdict(int)       # a blinking robot-second: its light was lit in 2+ frames of that second
    for c in truth:
        if c["led"] >= lit_level:
            lit_count[(c["robot"], int(math.floor(c["t"])))] += 1
    in_view = {k for k, v in lit_count.items() if v >= 2}
    total = correct = 0
    seconds_by_robot = defaultdict(lambda: [0, 0])
    for robot, spans in blinking.items():
        for t0, t1 in spans:
            for s in range(int(math.ceil(t0)), int(math.floor(t1))):
                # only seconds when the robot was in view
                if (robot, s) not in in_view:
                    continue
                total += 1
                rw = per_sec.get((robot, s), [0, 0])
                ok = rw[0] > 0 and rw[1] == 0
                correct += ok
                seconds_by_robot[robot][0] += 1
                seconds_by_robot[robot][1] += ok
    lit_frames = [(r["frame"], r["robot"]) for r in truth if r["led"] >= lit_level]
    return dict(unnamed_rows=unnamed, robot_seconds=total, correct_seconds=correct, accuracy=correct / total if total else float("nan"),
                pos_err=pos_err, false_rows=false_rows, wrong_rows=wrong_rows, by_robot=dict(seconds_by_robot),
                lit_frames=len(lit_frames), found_frames=sum(1 for k in lit_frames if k in found))


def describe(res):
    e = np.array(res["pos_err"]) if res["pos_err"] else np.array([float("nan")])
    return (f"decode accuracy {res['correct_seconds']}/{res['robot_seconds']} robot-seconds = {100 * res['accuracy']:.1f}%; "
            f"lit frames found {res['found_frames']}/{res['lit_frames']}; "
            f"position error median {np.median(e):.2f} px, 95% {np.percentile(e, 95):.2f} px; "
            f"false named rows {res['false_rows']}, wrong-name rows {res['wrong_rows']}, unnamed rows {res['unnamed_rows']}; by robot "
            + ", ".join(f"{k} {v[1]}/{v[0]}" for k, v in sorted(res["by_robot"].items())))
