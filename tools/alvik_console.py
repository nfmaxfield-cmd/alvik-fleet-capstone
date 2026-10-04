#!/usr/bin/env python3
"""Laptop console for firmware/home_lab.

Receives the robot's WiFi log, saves every record type to its own CSV file,
and sends the commands you type to the robot.

    python tools/alvik_console.py              # logs go to logs/<date-time>/
    python tools/alvik_console.py --robot 192.168.1.42   # if broadcast fails

Type `help` once it says the robot is talking. `quit` exits.
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


class Session:
    """Writes H/D records to CSV files and tracks packet loss."""

    def __init__(self, outdir, show_rows, on_robot_found=None):
        self.on_robot_found = on_robot_found
        self.outdir = outdir
        self.show_rows = show_rows
        self.headers = {}      # record type -> column list
        self.writers = {}      # record type -> (file, csv.writer)
        self.versions = {}     # record type -> file suffix when columns change
        self.last_seq = None
        self.received = 0
        self.dropped = 0
        self.robot = None
        self.lock = threading.Lock()

    def _open(self, rtype, cols):
        if rtype in self.writers:
            self.writers[rtype][0].close()
        os.makedirs(self.outdir, exist_ok=True)
        v = self.versions.get(rtype, 0)
        name = f"{rtype.lower()}.csv" if v == 0 else f"{rtype.lower()}_{v + 1}.csv"
        self.versions[rtype] = v + 1
        f = open(os.path.join(self.outdir, name), "w", newline="")
        w = csv.writer(f)
        w.writerow(cols)
        f.flush()
        self.writers[rtype] = (f, w)
        self.headers[rtype] = cols

    def handle(self, payload, addr):
        with self.lock:
            if self.robot != addr[0]:
                self.robot = addr[0]
                print(f"\n[robot is talking from {self.robot}]")
                if self.on_robot_found:
                    self.on_robot_found()   # reply so the robot learns our address
            seq_txt, sep, line = payload.partition("|")
            if not sep:
                line = payload
            else:
                try:
                    seq = int(seq_txt)
                    if self.last_seq is not None and seq > self.last_seq + 1:
                        self.dropped += seq - self.last_seq - 1
                    if self.last_seq is None or seq > self.last_seq or seq < 5:
                        self.last_seq = seq   # seq < 5: robot rebooted
                except ValueError:
                    line = payload
            self.received += 1
            kind, _, rest = line.partition(",")
            if kind == "M":
                print(f"robot: {rest}")
            elif kind == "H":
                rtype, _, cols = rest.partition(",")
                cols = cols.split(",")
                if self.headers.get(rtype) != cols:
                    self._open(rtype, cols)
            elif kind == "D":
                rtype, _, vals = rest.partition(",")
                if rtype not in self.writers:
                    print(f"[data for {rtype} arrived before its header; skipped]")
                    return
                f, w = self.writers[rtype]
                w.writerow(vals.split(","))
                f.flush()
                if self.show_rows or rtype != "STREAM" and rtype != "TRACE":
                    print(f"  {rtype}: {vals}")
            else:
                print(f"? {line}")

    def close(self):
        with self.lock:
            for f, _ in self.writers.values():
                f.close()
            self.writers.clear()


def receiver(sock, session, stop):
    while not stop.is_set():
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            session.handle(data.decode("utf-8", "replace").strip(), addr)
        except Exception as e:   # keep receiving even if one line is bad
            print(f"[could not handle a packet: {e}]")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", help="robot IP, if it can't be found automatically")
    ap.add_argument("--outdir", help="folder for the CSV files (default logs/<date-time>)")
    ap.add_argument("--show-stream", action="store_true", help="also print STREAM and TRACE rows")
    args = ap.parse_args()

    outdir = args.outdir or os.path.join("logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S"))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def send(cmd):
        target = session.robot or "255.255.255.255"
        tx.sendto(cmd.encode(), (target, CMD_PORT))

    session = Session(outdir, args.show_stream, on_robot_found=lambda: send("ping"))
    if args.robot:
        session.robot = args.robot

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("", LOG_PORT))
    rx.settimeout(0.5)
    stop = threading.Event()
    t = threading.Thread(target=receiver, args=(rx, session, stop), daemon=True)
    t.start()

    def hint():
        if session.received == 0 and not stop.is_set():
            print("\n[nothing heard yet. Is the robot on and on the same WiFi? Did Windows "
                  "firewall ask about Python? If you know the robot's IP (shown in the Arduino "
                  "Serial Monitor), restart with --robot <ip>]")
    timer = threading.Timer(8.0, hint)
    timer.daemon = True
    timer.start()

    print(f"Listening on UDP {LOG_PORT}; saving to {outdir}")
    print("Waiting for the robot... (switch it on, or press its reset)")
    send("ping")
    try:
        while True:
            try:
                cmd = input("> ").strip()
            except EOFError:
                break
            if not cmd:
                continue
            if cmd in ("quit", "exit"):
                break
            if cmd == "stats":
                print(f"packets received {session.received}, dropped {session.dropped}")
                continue
            send(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        if session.robot:
            send("stop")   # never leave the robot driving when the console closes
        stop.set()
        session.close()
        total = session.received + session.dropped
        loss = 100.0 * session.dropped / total if total else 0.0
        print(f"\nSaved to {outdir}. Packets: {session.received} received, "
              f"{session.dropped} dropped ({loss:.1f}%).")


if __name__ == "__main__":
    sys.exit(main())
