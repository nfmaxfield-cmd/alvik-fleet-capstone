#!/usr/bin/env python3
"""e22 acceptance (the brief, 13 Nov): the 25-run hard simulator, e22 against e20 and e21 on the same seeds and faults.

    python3 docs/results/e22-nov13/accept.py x25-e20 x25-e21 x25-e22 [x25-e22-other ...]   (folders in this directory)

Per run: falls, body contact, the longest stand by a robot free to move, waits for a person ("NEEDS A HAND" events: the
old hold-and-ask; questions asked). The acceptance lines: zero waits for a person; runs with a stand over 60 s fewer than
e20's; no run with more falls than e20 in that run; no run with contact where e20 had none.

Changelog: e22-2026-11-13a first version.
"""
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def runs(d):
    p = d if os.path.isdir(d) else os.path.join(HERE, d)
    return {os.path.basename(f)[:-5]: json.load(open(f)) for f in sorted(glob.glob(os.path.join(p, "*.json")))}


def waits(v):
    b = v["broke"]
    c = b.get("counts") or {}
    return (b.get("needs_hand") or 0) + (c.get("need_hand") or 0)


def main(argv):
    names = argv
    R = [runs(d) for d in names]
    keys = sorted(set().union(*[set(r) for r in R]))
    print("%-26s " % "run" + " | ".join("%-24s" % os.path.basename(n.rstrip("/")) for n in names))
    for k in keys:
        cells = []
        for r in R:
            v = r.get(k)
            if v is None:
                cells.append("%-24s" % "-")
                continue
            b = v["broke"]
            cells.append("%-24s" % ("F%d C%d stand %4.0f w%d" % (b.get("falls") or 0, 1 if b.get("contacts") else 0,
                                                                b.get("longest_free_stand_s") or 0, waits(v))))
        print("%-26s " % k + " | ".join(cells))
    base = R[0]
    out = {}
    for n, r in zip(names, R):
        tot = dict(runs=len(r), falls=sum(v["broke"].get("falls") or 0 for v in r.values()),
                   contact_runs=sum(1 for v in r.values() if v["broke"].get("contacts")),
                   stand60_runs=sum(1 for v in r.values() if (v["broke"].get("longest_free_stand_s") or 0) > 60),
                   longest_stand_s=round(max([v["broke"].get("longest_free_stand_s") or 0 for v in r.values()] + [0]), 1),
                   waits=sum(waits(v) for v in r.values()),
                   swaps=sum(v["broke"].get("swaps") or 0 for v in r.values()),
                   runs_more_falls_than_base=[k for k, v in r.items() if k in base and (v["broke"].get("falls") or 0) >
                                              (base[k]["broke"].get("falls") or 0)],
                   runs_contact_not_in_base=[k for k, v in r.items() if k in base and v["broke"].get("contacts") and
                                             not base[k]["broke"].get("contacts")])
        out[os.path.basename(n.rstrip("/"))] = tot
        print(os.path.basename(n.rstrip("/")), json.dumps(tot))
    return out


if __name__ == "__main__":
    main(sys.argv[1:])
