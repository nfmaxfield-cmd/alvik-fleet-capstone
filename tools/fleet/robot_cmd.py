#!/usr/bin/env python3
"""Send one console command to the robots, print what they answer for a while, and stop (run-fleet-pi.sh's bench
checks, 2026-10-27). The same link as the console (tools/alvik_console.py), without typing in it.

    python3 tools/fleet/robot_cmd.py --robot 192.168.1.250 --robot 192.168.1.252 status
    python3 tools/fleet/robot_cmd.py --robot 192.168.1.252 --to Center --wait-s 8 "turn 90"
    python3 tools/fleet/robot_cmd.py --robot ... --wait-s 75 calibrate

Prints every message and record line the robots send (as the console does) and, at the end, one line per robot with
its firmware (from `status`). Ends with `stop` to every robot, as the console's quit does.
"""
import argparse
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import alvik_console as AC   # noqa: E402

FW = re.compile(r"status (\S+) \(firmware ([^)]+)\).*battery (\d+)%")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", nargs="+", help="the console command, e.g. status, \"turn 90\", calibrate")
    ap.add_argument("--robot", action="append", default=[], help="robot IP (repeatable)")
    ap.add_argument("--to", default="all", help="a robot's name (Left, Center, Right) or all")
    ap.add_argument("--wait-s", type=float, default=4.0, help="listen this long after sending")
    ap.add_argument("--find-s", type=float, default=8.0, help="wait this long for the robots to answer first")
    ap.add_argument("--repeat", type=int, default=1, help="send it this many times, --wait-s apart")
    ap.add_argument("--out", help="log folder (default logs/<date-time>-cmd)")
    ap.add_argument("--then", action="append", default=[], metavar="CMD",
                    help="another command after the first, --wait-s later (repeatable; e.g. the settings read-back: "
                         "\"set gyro_scale 1.075\" --then \"get gyro_scale\")")
    a = ap.parse_args(argv)
    cmd = " ".join(a.command)
    out = a.out or os.path.join(REPO, "logs", time.strftime("%Y%m%d-%H%M%S") + "-cmd")
    os.makedirs(out, exist_ok=True)
    seen = {}

    def listen(name, kind, rtype, vals):
        text = vals if isinstance(vals, str) else ",".join(str(v) for v in vals) if vals else ""
        m = FW.search(text)
        if m:
            seen[m.group(1)] = (m.group(2), m.group(3))
    link = AC.Link(out, a.robot)
    link.session.listeners.append(listen)
    try:
        t0 = time.time()
        while time.time() - t0 < a.find_s and len(link.robots()) < max(1, len(a.robot)):
            time.sleep(0.2)
        names = link.robots()
        print("Robots heard: %s" % (", ".join(names) or "none"), flush=True)
        targets = names if a.to == "all" else [a.to]
        if a.to != "all" and a.to not in names:
            print("%s has not answered (is it on, and on this WiFi?)" % a.to)
            return 1
        for k in range(max(1, a.repeat)):
            for n in targets:
                link.send_name(n, cmd)
            print("Sent %r to %s%s" % (cmd, ", ".join(targets), "" if a.repeat == 1 else " (%d of %d)" % (k + 1, a.repeat)),
                  flush=True)
            time.sleep(a.wait_s)
        for c in a.then:
            for n in targets:
                link.send_name(n, c)
            print("Sent %r to %s" % (c, ", ".join(targets)), flush=True)
            time.sleep(a.wait_s)
        if cmd != "status":
            for n in targets:
                link.send_name(n, "status")
            time.sleep(1.5)
    finally:
        link.close()
    for n in sorted(seen):
        print("%s: firmware %s, battery %s%%" % (n, seen[n][0], seen[n][1]))
    print("Logs: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
