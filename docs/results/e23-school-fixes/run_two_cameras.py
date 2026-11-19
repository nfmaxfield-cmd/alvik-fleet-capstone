#!/usr/bin/env python3
"""(e23, simulator only) mission_run.py with the school's camera 2 switched ON for this one run, as it was on the
school Pi on 18 Nov from 15:03 (the repo's site file still says cameras.side.use false). Nothing is written to the
site file. usage: run_two_cameras.py REPO <mission_run arguments...>"""
import os, sys
REPO = sys.argv[1]
sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
sys.argv = [os.path.join(REPO, "tools", "fleet", "mission_run.py")] + sys.argv[2:]
import sites as SI
_load = SI.load
def load(name=None, *a, **k):
    d = _load(name, *a, **k)
    if d.get("name") == "school-conference":
        d.setdefault("cameras", {}).setdefault("side", {})["use"] = True
    return d
SI.load = load
import mission_run as MR
sys.exit(MR.main())
