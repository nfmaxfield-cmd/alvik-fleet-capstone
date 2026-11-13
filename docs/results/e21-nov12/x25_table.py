#!/usr/bin/env python3
"""e21: the 25-run comparison's table (hard simulator, the same seeds and faults as e20's), from the run JSONs.

    python3 docs/results/e21-nov12/x25_table.py docs/results/e21-nov12/x25-e20 docs/results/e21-nov12/x25-e21 ...

Per folder: falls, runs with body contact, name swaps, runs with a free stand over 60 s (and the longest), lost spells
(the controller's "lost": no camera fix accepted; count per robot-hour and their 90th percentile length, the median
over runs), and the share of robot time in each of Nick's stages (e21 code only).

Changelog: e21-2026-11-12a first version.
"""
import glob
import json
import os
import sys


def rows(d):
    out = {}
    for f in sorted(glob.glob(os.path.join(d, "*.json"))):
        out[os.path.basename(f)[:-5]] = json.load(open(f))
    return out


def summ(R):
    b = [v["broke"] for v in R.values()]
    lost = [v["measures"].get("lost") or {} for v in R.values()]
    med = lambda xs: sorted(xs)[len(xs) // 2] if xs else None
    st = {}
    for v in R.values():
        for k, x in (v.get("stage_s") or {}).items():
            st[k] = st.get(k, 0.0) + x
    T = sum(st.values()) or None
    return dict(runs=len(R), falls=sum(x.get("falls") or 0 for x in b),
                contact_runs=sum(1 for x in b if x.get("contacts")),
                swaps=sum(x.get("swaps") or 0 for x in b),
                stand60_runs=sum(1 for x in b if (x.get("longest_free_stand_s") or 0) > 60),
                longest_stand_s=max([x.get("longest_free_stand_s") or 0 for x in b] or [0]),
                hangs=sum(x.get("hangs") or 0 for x in b),
                lost_per_robot_h=med([q.get("per_robot_h") for q in lost if q.get("per_robot_h") is not None]),
                lost_p90_s=med([(q.get("length_s") or {}).get("p90") for q in lost if (q.get("length_s") or {}).get("p90")
                                is not None]),
                stages_pct={k: round(100.0 * x / T, 1) for k, x in st.items()} if T else None,
                stages_s={k: round(x, 0) for k, x in st.items()})


if __name__ == "__main__":
    for d in sys.argv[1:]:
        R = rows(d)
        print(os.path.basename(d.rstrip("/")), json.dumps(summ(R)))
        for k, v in R.items():
            b = v["broke"]
            print("   %-28s falls %d contact %d swaps %d stand %.0f s" % (k, b.get("falls") or 0, b.get("contacts") or 0,
                                                                         b.get("swaps") or 0,
                                                                         b.get("longest_free_stand_s") or 0))
