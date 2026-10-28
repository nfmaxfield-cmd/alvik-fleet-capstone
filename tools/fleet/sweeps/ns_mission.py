#!/usr/bin/env python3
"""mission_run.py, unchanged, in a network namespace of its own (Monday 2-robot sweep, 2026-10-28), so two dry runs
can share the machine without meeting each other's ports (8765/8766 cameras, 5010/5011 fixes and poses, the robots'):

    unshare -n python3 tools/fleet/sweeps/ns_mission.py <mission_run.py arguments>
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
for p in (HERE, FLEET):
    if p not in sys.path:
        sys.path.insert(0, p)

from rp_shim import loopback_up   # noqa: E402

if __name__ == "__main__":
    loopback_up()
    import mission_run as MR
    sys.exit(MR.main(sys.argv[1:]))
