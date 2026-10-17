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



_T0_WALL, _T0_PERF = time.time(), time.perf_counter()


def now():
    """Wall-clock time (Unix seconds) with sub-millisecond resolution. On
    Windows before Python 3.13, time.time() only ticks every ~15.6 ms, which
    would show up in every round-trip and latency figure."""
    return _T0_WALL + (time.perf_counter() - _T0_PERF)

def _firmware_headers():
    """Column lists of every record type, read from the firmware source
    (net::sendLine("H,TYPE,...") calls), so data whose header packet was lost
    over WiFi can still be saved."""
    import re
    out = {}
    fw = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "firmware", "home_lab")
    try:
        names = [n for n in os.listdir(fw) if n.endswith((".h", ".ino"))]
    except OSError:
        return out
    call = re.compile(r'sendLine\(((?:\s*"(?:[^"\\]|\\.)*")+)\s*\)')
    lit = re.compile(r'"((?:[^"\\]|\\.)*)"')
    for n in names:
        try:
            with open(os.path.join(fw, n), encoding="utf-8", errors="replace") as f:
                src = f.read()
        except OSError:
            continue
        for m in call.finditer(src):
            text = "".join(lit.findall(m.group(1)))
            if text.startswith("H,"):
                parts = text.split(",")
                out[parts[1]] = parts[2:]
    return out


KNOWN_HEADERS = _firmware_headers()

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
        self.listeners = []    # functions(name, kind, rtype, values) called for every M and D line
        self.quiet = False     # True: don't print rows/messages (another program shows its own)
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
        t_recv = now()
        events = []
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
                events.append((name, "M", "", rest))
                if not self.quiet:
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
                if rtype not in self.writers and rtype != "CLOCK":
                    # Its header packet was lost: fall back to the header in the firmware source.
                    known = KNOWN_HEADERS.get(rtype)
                    if known and len(known) == len(vals.split(",")):
                        self._open(rtype, known)
                        print(f"[{name}: {rtype} header packet lost; using the header from the firmware source]")
                if rtype not in self.writers:
                    if rtype != "CLOCK":   # clock answers are already saved above
                        print(f"[{name}: data for {rtype} arrived before its header; skipped]")
                else:
                    f, w = self.writers[rtype]
                    w.writerow([name, f"{t_recv:.4f}"] + vals.split(","))
                    self.rows[rtype] = self.rows.get(rtype, 0) + 1
                    if rtype not in ("RAW", "STREAM", "TRACE") or self.rows[rtype] % 250 == 0:
                        f.flush()   # high-rate files are flushed every 250 rows (~5 s) and on close
                    events.append((name, "D", rtype, vals.split(",")))
                    if not self.quiet and (self.show_rows or rtype not in QUIET):
                        print(f"  {name} {rtype}: {vals}")
            else:
                print(f"{name}? {line}")
        if new:
            if not self.quiet:
                print(f"[{name} is talking from {ip}]")
            self.on_new_robot(name)   # reply so the robot learns our address
        for ev in events:             # outside the lock, so a listener can send commands
            for fn in self.listeners:
                try:
                    fn(*ev)
                except Exception as e:
                    print(f"[listener error: {e}]")

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


class Link:
    """The WiFi link to the robots: receives and logs everything, checks clocks,
    sends commands. Used by this console and by tools/run_sheet.py."""

    def __init__(self, outdir, robot_ips=(), show_rows=False):
        self.outdir = outdir
        self.robot_ips = list(robot_ips)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.session = Session(outdir, show_rows, on_new_robot=lambda n: self.send_name(n, "ping"))
        write_session_info(outdir)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # No SO_REUSEADDR: with it, a second program on this port binds fine
        # and each robot packet then reaches only one of the two, silently
        # splitting the log. Without it, the second program stops here.
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        try:
            self.rx.bind(("", LOG_PORT))
        except OSError as e:
            raise SystemExit(f"Port {LOG_PORT} is already in use: another console, run_sheet or commtest is "
                             f"probably still running. Close it first. ({e})")
        self.rx.settimeout(0.5)
        self.stop = threading.Event()
        self.threads = [threading.Thread(target=receiver, args=(self.rx, self.session, self.stop), daemon=True),
                        threading.Thread(target=self._clock_loop, daemon=True)]
        for t in self.threads:
            t.start()
        self.send_all("ping")

    def send_ip(self, ip, cmd):
        self.tx.sendto(cmd.encode(), (ip, CMD_PORT))

    def send_name(self, name, cmd):
        with self.session.lock:
            r = self.session.robots.get(name)
        if r is None:
            if not self.session.quiet:
                print(f"[{name} isn't connected (type robots)]")
            return False
        self.send_ip(r["ip"], cmd)
        return True

    def send_all(self, cmd):
        with self.session.lock:
            ips = {r["ip"] for r in self.session.robots.values()}
        for ip in ips or set(self.robot_ips) or {"255.255.255.255"}:
            self.send_ip(ip, cmd)

    def robots(self):
        with self.session.lock:
            return sorted(self.session.robots)

    def _clock_loop(self):
        """Ask each robot for its clock: as soon as it appears, then every CLOCK_EVERY_S.
        Each check is a burst of 3; analysis keeps the one with the shortest round trip."""
        session = self.session
        token, last = 0, {}
        while not self.stop.wait(1.0):
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
                    t_send = now()                             # stamped right at sending
                    try:
                        self.send_ip(ip, f"clock {tok}")
                    except OSError:
                        break
                    with session.lock:
                        if tok in session.clock_sent:
                            session.clock_sent[tok] = (name, t_send)
                    time.sleep(0.05)

    def close(self):
        """Stop every robot, wait for their end records, close the files."""
        if self.session.robots:
            self.send_all("stop")   # never leave robots driving when the program closes
            time.sleep(1.2)         # let a stopped run's end flash and RUNEND record arrive
        self.stop.set()
        for t in self.threads:
            t.join(timeout=2)
        self.rx.close()          # free the port for the next program
        self.tx.close()
        self.session.close()

    def summary(self):
        with self.session.lock:
            rows = sorted(self.session.robots.items())
        return [(n, r["received"], r["dropped"],
                 100.0 * r["dropped"] / (r["received"] + r["dropped"]) if r["received"] + r["dropped"] else 0.0)
                for n, r in rows]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", action="append", default=[], help="robot IP, if robots aren't found automatically (repeatable)")
    ap.add_argument("--outdir", help="folder for the CSV files (default logs/<date-time>)")
    ap.add_argument("--show-stream", action="store_true", help="also print STREAM and TRACE rows")
    args = ap.parse_args()

    outdir = args.outdir or os.path.join("logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S"))
    link = Link(outdir, args.robot, args.show_stream)
    session, stop = link.session, link.stop
    send_name, send_all = link.send_name, link.send_all

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
        link.close()
        print(f"\nSaved to {outdir}.")
        for n, got, dropped, loss in link.summary():
            print(f"  {n}: {got} packets, {dropped} dropped ({loss:.1f}%)")


if __name__ == "__main__":
    sys.exit(main())
