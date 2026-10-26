"""legcheck.py RUN_FOLDER [ROBOT ...] : each explore leg's own record (distance d, turn dth) against the simulator's
truth between the same two stops: where a robot's own path and its real path part (a leg longer or shorter than it
says, a turn that is not what it says). Prints the legs off by more than 3 cm or 5 deg."""
import json
import math
import os
import sys

import numpy as np

import exsum


def main():
    d = sys.argv[1]
    st = json.load(open(os.path.join(d, "e02_state.json")))
    tr = exsum.truth_reader(d)
    obs = st["setup"]["explore_obs"]
    for n in (sys.argv[2:] or list(obs)):
        stops = obs[n]
        bad, tot = [], 0
        for i in range(1, len(stops)):
            a, b = stops[i - 1], stops[i]
            if a.get("t") is None or b.get("t") is None:
                continue
            xa, ya, ha = exsum.at(tr, n, a["t"])
            xb, yb, hb = exsum.at(tr, n, b["t"])
            hm = math.radians(ha + ((hb - ha + 180) % 360 - 180) / 2)
            dt = (xb - xa) * math.cos(hm) + (yb - ya) * math.sin(hm)
            tt = (hb - ha + 180) % 360 - 180
            tot += 1
            dd, dth = b.get("d", 0.0), b.get("dth", 0.0)
            gyro = 1.0 / 1.07
            if abs(dt - dd) > 3.0 or abs(tt * gyro - dth) > 5.0 and abs(abs(tt * gyro - dth) - 360) > 5:
                bad.append((i, round(dd, 1), round(dt, 1), round(dth, 1), round(tt, 1), b.get("contact", ""),
                            round(b["t"] - stops[0]["t"])))
        print("%s: %d legs, %d off: %s" % (n, tot, len(bad), bad[:20]))


if __name__ == "__main__":
    main()
