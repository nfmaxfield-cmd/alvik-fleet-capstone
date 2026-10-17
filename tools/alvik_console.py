#!/usr/bin/env python3
"""Laptop console for firmware/home_lab, for one robot or several.

Receives every robot's WiFi log, saves each record type to its own CSV file
(with `robot` and `t_recv` columns: t_recv is the laptop's clock, Unix seconds,
when the line arrived), and sends the commands you type.

Every 10 s it also asks each robot for its clock and saves the answers to
clock_sync.csv, so robot time can be converted to laptop time afterwards.

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
import json
import os
import platform
import socket
import subprocess
import sys
import threading
import time

LOG_PORT = 5005   # robot -> laptop
CMD_PORT = 5006   # laptop -> robot
MOVING = ("run lap", "run grid", "run demo", "run straight")   # experiments where robots drive around
QUIET = ("STREAM", "TRACE", "RAW", "CLOCK", "PARAM")   # saved but not printed (too many rows)
CLOCK_EVERY_S = 10


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
        self.clock_sent = {}   # token -> (robot name, laptop send time)
        self.rows = {}         # record type -> rows written
        self.renamed = {}      # old robot name -> new name
        self.clock_file = None

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
        w.writerow(["robot", "t_recv"] + cols)
        f.flush()
        self.writers[rtype] = (f, w)
        self.headers[rtype] = cols

    # ---- clock sync ----
    def clock_row(self, name, token, robot_ms, boot, t_recv):
        """Called with the lock held. Pairs a robot's clock answer with when we asked."""
        sent = self.clock_sent.pop(token, None)
        if sent is None or sent[0] != name or sent[1] is None:
            return
        t_send = sent[1]
        if self.clock_file is None:
            os.makedirs(self.outdir, exist_ok=True)
            f = open(os.path.join(self.outdir, "clock_sync.csv"), "w", newline="")
            w = csv.writer(f)
            w.writerow(["robot", "boot", "token", "t_send", "t_recv", "robot_ms", "rtt_ms", "offset_s"])
            self.clock_file = (f, w)
        f, w = self.clock_file
        # Assume the answer was made halfway through the round trip.
        offset = (t_send + t_recv) / 2 - robot_ms / 1000.0
        w.writerow([name, boot, token, f"{t_send:.4f}", f"{t_recv:.4f}", robot_ms,
                    f"{(t_recv - t_send) * 1000:.1f}", f"{offset:.4f}"])
        f.flush()

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
        t_recv = time.time()
        with self.lock:
            r = self.robots.get(name)
            if r is None:
                # A renamed robot (e.g. AL-3F2A -> Left) keeps its address: drop the old name.
                for old in [n for n, v in self.robots.items() if v["ip"] == ip and n != name]:
                    print(f"[{old} is now {name}]")
                    del self.robots[old]
                    self.renamed[old] = name
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
                if rtype == "CLOCK":   # pair it with our question even if its header packet was lost
                    parts = vals.split(",")
                    if len(parts) >= 2 and parts[1].strip().isdigit():
                        self.clock_row(name, parts[0], int(parts[1]), parts[2] if len(parts) > 2 else "", t_recv)
                if rtype not in self.writers:
                    print(f"[{name}: data for {rtype} arrived before its header; skipped]")
                else:
                    f, w = self.writers[rtype]
                    w.writerow([name, f"{t_recv:.4f}"] + vals.split(","))
                    self.rows[rtype] = self.rows.get(rtype, 0) + 1
                    if rtype not in ("RAW", "STREAM", "TRACE") or self.rows[rtype] % 250 == 0:
                        f.flush()   # high-rate files are flushed every 250 rows (~5 s) and on close
                    if self.show_rows or rtype not in QUIET:
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
            if self.clock_file:
                self.clock_file[0].close()
                self.clock_file = None

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


def write_session_info(outdir):
    """Who, where and with what code this session was recorded."""
    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True,
                                cwd=os.path.dirname(os.path.abspath(__file__)), timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        commit = ""
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "session.json"), "w") as f:
        json.dump(dict(start=dt.datetime.now().astimezone().isoformat(timespec="seconds"), start_unix=time.time(),
                       laptop=platform.node(), python=platform.python_version(), repo_commit=commit or "unknown",
                       note="t_recv in every CSV is the laptop clock (Unix seconds) when the line arrived"), f, indent=2)


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
            r = session.robots.get(name)
        if r is None:
            print(f"[{name} isn't connected (type robots)]")
            return
        send_ip(r["ip"], cmd)

    def send_all(cmd):
        with session.lock:
            ips = {r["ip"] for r in session.robots.values()}
        for ip in ips or set(args.robot) or {"255.255.255.255"}:
            send_ip(ip, cmd)

    session = Session(outdir, args.show_stream, on_new_robot=lambda n: send_name(n, "ping"))
    write_session_info(outdir)

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("", LOG_PORT))
    rx.settimeout(0.5)
    stop = threading.Event()
    threading.Thread(target=receiver, args=(rx, session, stop), daemon=True).start()

    def clock_loop():
        """Ask each robot for its clock: as soon as it appears, then every CLOCK_EVERY_S.
        Each check is a burst of 3; analysis keeps the one with the shortest round trip."""
        token, last = 0, {}
        while not stop.wait(1.0):
            with session.lock:
                robots = list(session.robots)
            for name in robots:
                if time.time() - last.get(name, 0) < CLOCK_EVERY_S:
                    continue
                last[name] = time.time()
                for _ in range(3):
                    token += 1
                    tok = f"c{token}"
                    with session.lock:
                        ip = session.robots.get(name, {}).get("ip")
                        session.clock_sent[tok] = (name, None)
                        if len(session.clock_sent) > 300:      # forget questions nobody answered
                            for k in list(session.clock_sent)[:150]:
                                del session.clock_sent[k]
                    if ip is None:
                        break
                    t_send = time.time()                       # stamped right at sending
                    try:
                        send_ip(ip, f"clock {tok}")
                    except OSError:
                        break
                    with session.lock:
                        if tok in session.clock_sent:
                            session.clock_sent[tok] = (name, t_send)
                    time.sleep(0.05)
    threading.Thread(target=clock_loop, daemon=True).start()

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
            if target != "all":
                with session.lock:
                    known = target in session.robots
                    renamed = session.renamed.get(target)
                if not known:   # the robot was renamed (or went away): follow it
                    target = renamed if renamed else "all"
                    print(f"[now talking to {target}]")
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
            time.sleep(1.2)    # let a stopped run's end flash and RUNEND record arrive before closing files
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
