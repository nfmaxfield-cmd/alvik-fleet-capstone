"""Which test moves should a discovery calibration run include? Synthetic, logs only (tools/discovery/battery.py).

    python3 tools/discovery/battery_doe.py      (about 5-10 min on 2 busy cores)

Plans: today's 9 s calibration log (stand, a small wiggle, stand); the full battery; the full battery minus one move
at a time (what each move is worth when the rest are there); a stand plus one move at a time (what a move gives on its
own); and two compact batteries. Three robots (seeds) each, plain and with units disguised (every numeric stream
rescaled, half sign-flipped). Scored at accept 0.90 / ask 0.60: streams right (top guess), accepted right, accepted
wrong, questions, per stream family. Writes results/battery_doe.json.

Caveat stated in the report: the synthetic robot is my model (IMU physics, noise from the real logs), so these numbers
say what each move makes possible for this discoverer, not what a real robot will give."""
import json
import os
import sys
import tempfile
import time
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from discovery import battery as B, replay as RP, truth as T          # noqa: E402

ACCEPT, ASK = 0.90, 0.60
SEEDS = (11, 12, 13)
FAMILY = {"ax": "IMU other axes", "ay": "IMU other axes", "gx": "IMU other axes", "gy": "IMU other axes",
          "az": "gravity axis + yaw rate", "gz": "gravity axis + yaw rate",
          "wl_deg": "wheels", "wr_deg": "wheels", "wl_rpm": "wheels", "wr_rpm": "wheels",
          "dL_cm": "range zones", "dCL_cm": "range zones", "dC_cm": "range zones", "dCR_cm": "range zones",
          "dR_cm": "range zones", "dtop_cm": "range zones", "dbot_cm": "range zones",
          "lineL": "line sensors", "lineC": "line sensors", "lineR": "line sensors",
          "lifted": "lifted flag",
          "v_cms": "derived", "w_dps": "derived", "x_cm": "derived", "y_cm": "derived", "th_deg": "derived",
          "roll_deg": "derived", "pitch_deg": "derived", "yaw_deg": "derived"}
FULL = [("stand", 10.0), ("spins", (4, 90.0)), ("drive", (2, 20.0)), ("square", (12, 15.0)), ("tilt", (2, 20.0)),
        ("lift", (2, 5.0)), ("line", (2,)), ("stand", 5.0)]
OPTIONAL = ("spins", "drive", "square", "tilt", "lift", "line")


def plans():
    out = {"today 9 s log": [("stand", 3.0), ("spins", (2, 15.0)), ("stand", 4.0)], "full battery": FULL}
    for m in OPTIONAL:
        out["full minus " + m] = [p for p in FULL if p[0] != m]
    for m in OPTIONAL:
        out["stand + " + m] = [("stand", 10.0)] + [p for p in FULL if p[0] == m] + [("stand", 5.0)]
    out["compact A (no square, no line)"] = [("stand", 10.0), ("spins", (4, 90.0)), ("drive", (2, 20.0)),
                                             ("tilt", (2, 20.0)), ("lift", (2, 5.0)), ("stand", 5.0)]
    out["compact B (one tilt, one lift)"] = [("stand", 10.0), ("spins", (4, 90.0)), ("drive", (2, 20.0)),
                                             ("tilt", (1, 20.0)), ("lift", (1, 5.0)), ("stand", 5.0)]
    # hands-free: a printed ramp (pitch) and one-wheel bump (roll) instead of hand tilts, no lift; the line move
    # stands in for creeping to an open plywood edge until the edge guard trips (both give the line sensors a
    # large step while the robot drives)
    hf = [("stand", 10.0), ("spins", (4, 90.0)), ("drive", (2, 20.0)), ("square", (12, 15.0)), ("ramp", (2, 14.0, 3.0)),
          ("bump", (2, 6.4, 3.0)), ("line", (2,)), ("stand", 5.0)]
    out["hands-free battery (ramp, bump, edge)"] = hf
    out["hands-free minus bump"] = [p for p in hf if p[0] != "bump"]
    out["hands-free minus ramp"] = [p for p in hf if p[0] != "ramp"]
    return out


def score(key, feats, res):
    fam = {}
    for s, r in res.items():
        col = key[s]
        truth = T.truth_for(col, bool(feats[s].get("const")))
        f = FAMILY.get(col, "passive (clock, ids, constants)")
        d = fam.setdefault(f, {"streams": 0, "right": 0, "accepted_right": 0, "accepted_wrong": 0, "questions": 0})
        d["streams"] += 1
        d["right"] += truth == r["type"]
        if r["type"] != "unknown" and r["confidence"] >= ACCEPT:
            d["accepted_right" if truth == r["type"] else "accepted_wrong"] += 1
        elif r["type"] != "unknown" and r["confidence"] >= ASK:
            d["questions"] += 1
    return fam


def main():
    out, t_all = {}, time.time()
    for name, plan in plans().items():
        runs = []
        for seed in SEEDS:
            d = tempfile.mkdtemp(prefix="battery_")
            case, key, secs, dur = B.load_plan_case(d, plan, seed=seed)
            for variant in ("plain", "disguised"):
                c = case if variant == "plain" else RP.disguise(case, key, seed=zlib.crc32(name.encode()) % 1000 + seed)
                feats, R, res, cal = RP.discover(c)
                runs.append({"seed": seed, "variant": variant, "duration_s": dur, "move_s": secs,
                             "families": score(key, feats, res)})
        out[name] = runs
        tot = lambda v, k: np.mean([sum(f[k] for f in r["families"].values()) for r in runs if r["variant"] == v])
        print("%-34s %5.0f s  plain: right %.1f acc %.1f wrong %.1f q %.1f | disguised: right %.1f acc %.1f "
              "wrong %.1f q %.1f" % (name, runs[0]["duration_s"], tot("plain", "right"), tot("plain", "accepted_right"),
                                     tot("plain", "accepted_wrong"), tot("plain", "questions"),
                                     tot("disguised", "right"), tot("disguised", "accepted_right"),
                                     tot("disguised", "accepted_wrong"), tot("disguised", "questions")), flush=True)
    with open(os.path.join(HERE, "results", "battery_doe.json"), "w") as f:
        json.dump(out, f, indent=1, default=float)
    print("done in %.0f s" % (time.time() - t_all))


if __name__ == "__main__":
    main()
