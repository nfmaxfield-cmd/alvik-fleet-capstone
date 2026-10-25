#!/usr/bin/env python3
"""Put robots to sleep, or wake them, from the Pi or the laptop (firmware 2026-10-25a; Nick, 2026-10-24: robots
left switched on drain about 14% an hour). Not while a session runs (it uses the same WiFi port).

    python3 tools/pi/robot_sleep.py sleep 30 192.168.1.250 192.168.1.252    # sleep, checking in every 30 min
    python3 tools/pi/robot_sleep.py wake 192.168.1.250                        # waits for its next check-in, wakes it

A sleeping robot hears nothing until its check-in (it wakes every N minutes for sleep_awake_s, 20 s): `wake` keeps
sending until the robot answers, for up to --max-min minutes."""
import argparse
import socket
import sys
import time

CMD_PORT = 5006   # laptop -> robot (tools/alvik_console.py)
LOG_PORT = 5005   # robot -> laptop


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["sleep", "wake"])
    ap.add_argument("args", nargs="+", help="sleep: minutes then robot IPs; wake: robot IPs")
    ap.add_argument("--max-min", type=float, default=35.0, help="wake: give up after this long")
    a = ap.parse_args()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if a.what == "sleep":
        minutes, ips = int(a.args[0]), a.args[1:]
        for ip in ips:
            tx.sendto(("sleep %d" % minutes).encode(), (ip, CMD_PORT))
            print("%s: sleep %d sent" % (ip, minutes))
        return 0
    ips = a.args
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind(("", LOG_PORT))
    except OSError:
        sys.exit("Port %d is in use: a session (or the console) is running. Stop it first." % LOG_PORT)
    rx.settimeout(2.0)
    left = set(ips)
    end = time.time() + a.max_min * 60
    print("Waking %s: sending `wake` every 2 s until each answers (up to %.0f min) ..." % (", ".join(ips), a.max_min))
    while left and time.time() < end:
        for ip in list(left):
            tx.sendto(b"wake", (ip, CMD_PORT))
        try:
            data, (src, _) = rx.recvfrom(2048)
        except socket.timeout:
            continue
        if src in left and b"awake (it stays awake" in data:
            print("%s: awake" % src)
            left.discard(src)
    if left:
        print("Not answered: %s (switched off, flat, or asleep longer than --max-min)" % ", ".join(sorted(left)))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
