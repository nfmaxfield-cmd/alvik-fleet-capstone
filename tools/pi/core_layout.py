#!/usr/bin/env python3
"""(e17, home 11 Nov: cores 2-3 at 99 % for 4 h, the camera bridge handling 4-5 of 25 pictures a second, fixes 0.44-0.95 s
late, ~400 robot watchdog stops) Which of the Pi's cores each program runs on: a SETTING, MIRLT_CORE_LAYOUT.

  A  as up to 11 Nov: the recorder on cores 0-1 (one camera: core 1), the planner AND the camera bridge on 2-3
  B  (the default: Nick chose B on 12 Nov, pending the bench; e17 had it as Claude's guess) the recorder on core 0, the
     planner (free roam, run_plan, mission_run, the ladder) on core 1, the camera bridge (cam_bridge / tag_bridge) on
     cores 2-3
  C  nothing pinned (Linux spreads them)

run-fleet-pi.sh reads it (core_layout in the script) for the recorder and the planner and exports MIRLT_BRIDGE_CORES;
the programs that start a bridge put bridge_prefix() in front of its command. Under 4 cores, or with no taskset,
nothing is pinned whatever the layout (said by the script).

    python3 tools/pi/core_layout.py [A|B|C]      prints: recorder planner bridge (one line, "-" for unpinned)
"""
import os
import shutil
import sys

LAYOUTS = {"A": dict(recorder2="0,1", recorder1="1", planner="2,3", bridge="2,3"),
           "B": dict(recorder2="0", recorder1="0", planner="1", bridge="2,3"),
           "C": dict(recorder2="", recorder1="", planner="", bridge="")}
DEFAULT = "B"      # Nick chose B (12 Nov), pending the bench (./run-fleet-pi.sh bench N layouts); e17: Claude's guess


def layout(name=None):
    n = (name or os.environ.get("MIRLT_CORE_LAYOUT") or DEFAULT).strip().upper()
    return n if n in LAYOUTS else DEFAULT


def cores(role, name=None, ncpu=None):
    """The cores for role ("recorder2", "recorder1", "planner", "bridge") under the layout, or "" (unpinned)."""
    if (ncpu if ncpu is not None else (os.cpu_count() or 1)) < 4:
        return ""
    return LAYOUTS[layout(name)][role]


def bridge_prefix(env=None):
    """The command prefix that pins a camera bridge (["taskset", "-c", "2,3"]) or [] (MIRLT_BRIDGE_CORES unset or
    empty, no taskset, under 4 cores)."""
    env = os.environ if env is None else env
    c = (env.get("MIRLT_BRIDGE_CORES") or "").strip()
    if not c or (os.cpu_count() or 1) < 4 or shutil.which("taskset") is None:
        return []
    return ["taskset", "-c", c]


if __name__ == "__main__":
    n = layout(sys.argv[1] if len(sys.argv) > 1 else None)
    print(n, *[(cores(r, n) or "-") for r in ("recorder2", "recorder1", "planner", "bridge")])
