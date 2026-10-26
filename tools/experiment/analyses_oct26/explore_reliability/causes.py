"""causes.py KEEP_FOLDER [...] : for each kept dry run, what went wrong in the robots' own records against the
simulator's truth: legs whose record never arrived (recorded as no move, or marked lost, while the robot moved or
turned), legs whose wheels overcounted (the body moved 25 % or 5 cm less than counted: a creep against a board, a hang
at an edge), and which robots explore left out. Prints one line per run and the totals."""
import json
import math
import os
import sys

import exsum


def run(d):
    st = json.load(open(os.path.join(d, "e02_state.json")))
    tr = exsum.truth_reader(d)
    obs = st["setup"].get("explore_obs") or {}
    lost, over, lost_marked = [], [], 0
    for n, v in obs.items():
        if n not in tr:
            continue
        for i in range(1, len(v)):
            a, b = v[i - 1], v[i]
            xa, ya, ha = exsum.at(tr, n, a["t"])
            xb, yb, hb = exsum.at(tr, n, b["t"])
            mv = math.hypot(xb - xa, yb - ya)
            tu = (hb - ha + 180) % 360 - 180
            if b.get("lost"):
                lost_marked += 1
            if abs(b.get("d") or 0) < 0.01 and abs(b.get("dth") or 0) < 0.01 and (mv > 2 or abs(tu) > 5):
                lost.append((n, i, round(mv, 1), round(tu, 1)))
            elif abs(b.get("dth") or 0) < 10 and abs(b.get("d") or 0) >= 2:
                own = abs(b.get("d_own", b["d"]))
                if own - mv > 5 or (own - mv > 1.5 and mv < 0.75 * own):
                    over.append((n, i, round(own, 1), round(mv, 1)))
    tries = st["setup"].get("explore_tries") or []
    last = tries[-1] if tries else {}
    return dict(lost=lost, over=over, lost_marked=lost_marked, left_out=last.get("left_out") or [],
                stalls=len(st["setup"].get("explore_creep_stalls") or []), repaired=last.get("repaired"),
                seen=last.get("seen_contacts"), tries=len(tries), passed=bool((st["setup"].get("explore") or {}).get("W")))


def main():
    tot = dict(runs=0, with_lost=0, with_over=0, left_out=0)
    for d in sys.argv[1:]:
        r = run(d)
        tot["runs"] += 1
        tot["with_lost"] += bool(r["lost"])
        tot["with_over"] += bool(r["over"])
        tot["left_out"] += len(r["left_out"])
        print(os.path.basename(d), json.dumps(r))
    print("TOTAL", json.dumps(tot))


if __name__ == "__main__":
    main()
