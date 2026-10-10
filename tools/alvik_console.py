#!/usr/bin/env python3
"""Laptop console for firmware/home_lab, for one robot or several.

Receives every robot's WiFi log, saves each record type to its own CSV file
(with a `robot` column), and sends the commands you type.

    python tools/alvik_console.py                 # logs go to logs/<date-time>/
    python tools/alvik_console.py --robot 192.168.1.42 --robot 192.168.1.43
                                                  # only if robots aren't found

Console commands (everything else is sent to the robots, e.g. `run turn`):
    robots              list robots heard from
    use AL-3F2A | all   who your commands go to (default: all)
    AL-3F2A: <command>  send one command to one robot (the 4 hex digits alone work too)
    stop                always stops every robot
    stats               packet counts per robot
    quit                stop every robot and exit

Standard library only; Python 3.8+. On Windows, allow Python through the
firewall when asked (private networks), or no packets arrive.
"""
import argparse
import csv
import datetime as dt
import os
import socket
import sys
import threading

LOG_PORT = 5005   # robot -> laptop
CMD_PORT = 5006   # laptop -> robot
MOVING = ("run lap", "run grid")   # experiments where robots drive around


class Session:
    """Tracks robots, writes H/D records to CSV, counts dropped packets."""

    def __init__(self, outdir, show_rows, on_new_robot):
        self.outdir = outdir
        self.show_rows = show_rows
        self.on_new_robot = on_new_robot
        self.robots = {}       # name -> dict(ip, last_seq, received, dropped)
        self.headers = {}      # record type -> column list (without robot)
        self.writers = {}      # record type -> (file, csv.writer)
        self.versions = {}     # record type -> how many files opened so far
        self.lock = threading.Lock()

    # ---- CSV ----
    def _open(self, rtype, cols):
        if rtype in self.writers:
            self.writers[rtype][0].close()
        os.makedirs(self.outdir, exist_ok=True)
        v = self.versions.get(rtype, 0)
        name = f"{rtype.lower()}.csv" if v == 0 else f"{rtype.lower()}_{v + 1}.csv"
        self.versions[rtype] = v + 1
        f = open(os.path.join(self.outdir, name), "w", newline="")
        w = csv.writer(f)
        w.writerow(["robot"] + cols)
        f.flush()
        self.writers[rtype] = (f, w)
        self.headers[rtype] = cols

    # ---- one packet ----
    def handle(self, payload, ip):
        parts = payload.split("|", 2)
        if len(parts) == 3 and parts[1].isdigit():
            name, seq, line = parts[0], int(parts[1]), parts[2]
        elif len(parts) >= 2 and parts[0].isdigit():      # older firmware: "seq|line"
            name, seq, line = ip, int(parts[0]), payload.split("|", 1)[1]
        else:
            name, seq, line = ip, None, payload
        new = False
        with self.lock:
            r = self.robots.get(name)
            if r is None or r["ip"] != ip:
                r = self.robots.setdefault(name, dict(ip=ip, last_seq=None, received=0, dropped=0))
                r["ip"] = ip
                new = True
            if seq is not None:
                last = r["last_seq"]
                if last is not None and seq > last + 1:
                    r["dropped"] += seq - last - 1
                if last is None or seq > last or seq < 5:   # seq < 5: robot rebooted
                    r["last_seq"] = seq
            r["received"] += 1

            kind, _, rest = line.partition(",")
            if kind == "M":
                print(f"{name}: {rest}")
            elif kind == "H":
                rtype, _, cols = rest.partition(",")
                cols = cols.split(",")
                if self.headers.get(rtype) != cols:
                    self._open(rtype, cols)
            elif kind == "D":
                rtype, _, vals = rest.partition(",")
                if rtype not in self.writers:
                    print(f"[{name}: data for {rtype} arrived before its header; skipped]")
                else:
                    f, w = self.writers[rtype]
                    w.writerow([name] + vals.split(","))
                    f.flush()
                    if self.show_rows or rtype not in ("STREAM", "TRACE"):
                        print(f"  {name} {rtype}: {vals}")
            else:
                print(f"{name}? {line}")
        if new:
            print(f"[{name} is talking from {ip}]")
            self.on_new_robot(name)   # reply so the robot learns our address

    def close(self):
        with self.lock:
            for f, _ in self.writers.values():
                f.close()
            self.writers.clear()

    def find(self, key):
        """Robot name from a full name or its last few characters."""
        key = key.strip().upper()
        with self.lock:
            names = list(self.robots)
        exact = [n for n in names if n.upper() == key]
        if exact:
            return exact[0]
        tail = [n for n in names if n.upper().endswith(key)]
        return tail[0] if len(tail) == 1 else None


def receiver(sock, session, stop):
    while not stop.is_set():
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            session.handle(data.decode("utf-8", "replace").strip(), addr[0])
        except Exception as e:   # keep receiving even if one packet is bad
            print(f"[could not handle a packet: {e}]")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", action="append", default=[], help="robot IP, if robots aren't found automatically (repeatable)")
    ap.add_argument("--outdir", help="folder for the CSV files (default logs/<date-time>)")
    ap.add_argument("--show-stream", action="store_true", help="also print STREAM and TRACE rows")
    args = ap.parse_args()

    outdir = args.outdir or os.path.join("logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S"))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def send_ip(ip, cmd):
        tx.sendto(cmd.encode(), (ip, CMD_PORT))

    def send_name(name, cmd):
        with session.lock:
            ip = session.robots[name]["ip"]
        send_ip(ip, cmd)

    def send_all(cmd):
        with session.lock:
            ips = {r["ip"] for r in session.robots.values()}
        for ip in ips or set(args.robot) or {"255.255.255.255"}:
            send_ip(ip, cmd)

    session = Session(outdir, args.show_stream, on_new_robot=lambda n: send_name(n, "ping"))

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("", LOG_PORT))
    rx.settimeout(0.5)
    stop = threading.Event()
    threading.Thread(target=receiver, args=(rx, session, stop), daemon=True).start()

    def hint():
        if not session.robots and not stop.is_set():
            print("\n[nothing heard yet. Are the robots on and on the same WiFi? Did Windows "
                  "firewall ask about Python? If you know a robot's IP (Arduino Serial Monitor "
                  "shows it), restart with --robot <ip>]")
    timer = threading.Timer(8.0, hint)
    timer.daemon = True
    timer.start()

    print(f"Listening on UDP {LOG_PORT}; saving to {outdir}")
    print("Waiting for robots... (switch them on, or press reset)")
    send_all("ping")
    target = "all"
    try:
        while True:
            try:
                line = input(f"[{target}]> ").strip()
            except EOFError:
                break
            if not line:
                continue
            low = line.lower()
            if low in ("quit", "exit"):
                break
            if low == "stop":
                send_all("stop")
                continue
            if low == "robots" or low == "stats":
                with session.lock:
                    rows = sorted(session.robots.items())
                if not rows:
                    print("no robots heard yet")
                for n, r in rows:
                    tot = r["received"] + r["dropped"]
                    loss = 100.0 * r["dropped"] / tot if tot else 0.0
                    print(f"  {n:10s} {r['ip']:15s} received {r['received']:6d}  dropped {r['dropped']:4d} ({loss:.1f}%)")
                continue
            if low.startswith("use "):
                key = line[4:].strip()
                if key.lower() == "all":
                    target = "all"
                else:
                    n = session.find(key)
                    if n:
                        target = n
                    else:
                        print(f"no single robot matches '{key}' (type robots)")
                continue
            # one-off "NAME: command"
            dest, cmd = target, line
            head, sep, tail = line.partition(":")
            if sep and " " not in head.strip():
                n = session.find(head)
                if n:
                    dest, cmd = n, tail.strip()
            if dest == "all":
                with session.lock:
                    count = len(session.robots)
                if count > 1 and cmd.lower().startswith(MOVING):
                    ok = input(f"  {count} robots would drive at once with no collision avoidance. Type yes to go: ")
                    if ok.strip().lower() != "yes":
                        continue
                send_all(cmd)
            else:
                send_name(dest, cmd)
    except KeyboardInterrupt:
        pass
    finally:
        if session.robots:
            send_all("stop")   # never leave robots driving when the console closes
        stop.set()
        session.close()
        print(f"\nSaved to {outdir}.")
        with session.lock:
            for n, r in sorted(session.robots.items()):
                tot = r["received"] + r["dropped"]
                loss = 100.0 * r["dropped"] / tot if tot else 0.0
                print(f"  {n}: {r['received']} packets, {r['dropped']} dropped ({loss:.1f}%)")


if __name__ == "__main__":
    sys.exit(main())
