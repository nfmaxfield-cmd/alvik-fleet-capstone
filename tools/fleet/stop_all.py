#!/usr/bin/env python3
"""Stop everything that drives the robots, at once (session plan 2026-10-28: "the one command"). Safe to run any time,
also when nothing is running.

    ./run-fleet-pi.sh stopall                     (the same as the line below, with the three robots' addresses)
    python3 tools/fleet/stop_all.py --robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253

In order:
  1. Ctrl+C to every fleet run started in tmux (monday, practice, doe, ladder, ladder2, mc's own run is step 2): each
     program's own stop then halts its robots and closes its logs (the upload follows where the program uploads).
  2. Mission Control's Stop (POST /api/run/stop on 127.0.0.1:8095), if Mission Control is open.
  3. `stop` straight to every robot's command port (UDP 5006), STOP_REPEATS times, STOP_GAP_S apart. This does not
     listen on the log port, so it works while another program holds it (robot_cmd.py and the console cannot).
Prints what it did. Never kills a process.
"""
import argparse
import json
import socket
import subprocess
import sys
import time
import urllib.request

CMD_PORT = 5006            # tools/alvik_console.py CMD_PORT (laptop/Pi -> robot)
ROBOT_IPS = ("192.168.1.250", "192.168.1.252", "192.168.1.253")
TMUX_RUNS = ("monday", "practice", "doe", "ladder", "ladder2")
MC_URL = "http://127.0.0.1:8095/api/run/stop"
STOP_REPEATS = 3           # chosen: UDP can drop a packet (27 Oct: up to 1 in 10); three copies 0.2 s apart. No DOE:
STOP_GAP_S = 0.2           # a repeated `stop` is harmless on the robot


def tmux_ctrl_c(sessions=TMUX_RUNS, run=subprocess.run):
    done = []
    for s in sessions:
        try:
            if run(["tmux", "has-session", "-t", s], capture_output=True).returncode == 0:
                run(["tmux", "send-keys", "-t", s, "C-c"], capture_output=True)
                done.append(s)
        except OSError:          # no tmux on this machine
            break
    return done


def mc_stop(url=MC_URL, timeout=2.0):
    try:
        req = urllib.request.Request(url, data=b"{}", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            st = json.loads(r.read().decode() or "null")
        return "stopped its run" if st else "open, no run going"
    except Exception:            # noqa: BLE001  (closed, or nothing to stop)
        return None


def stop_robots(ips, port=CMD_PORT, repeats=STOP_REPEATS, gap_s=STOP_GAP_S):
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = 0
    try:
        for k in range(repeats):
            for ip in ips:
                try:
                    tx.sendto(b"stop", (ip, port))
                    sent += 1
                except OSError:
                    pass
            if k < repeats - 1:
                time.sleep(gap_s)
    finally:
        tx.close()
    return sent


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", action="append", default=[], help="a robot's IP (default: the three robots)")
    ap.add_argument("--port", type=int, default=CMD_PORT)
    ap.add_argument("--mc-url", default=MC_URL)
    ap.add_argument("--no-tmux", action="store_true")
    a = ap.parse_args(argv)
    ips = a.robot or list(ROBOT_IPS)
    runs = [] if a.no_tmux else tmux_ctrl_c()
    print("1. Ctrl+C sent to: %s" % (", ".join(runs) if runs else "(no fleet run in tmux)"), flush=True)
    mc = mc_stop(a.mc_url)
    print("2. Mission Control: %s" % (mc or "not open"), flush=True)
    n = stop_robots(ips, a.port)
    print("3. `stop` sent %d times to %s" % (n, ", ".join(ips)), flush=True)
    print("Done. The robots should be standing still. If one still moves: pick it up and switch it off.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
