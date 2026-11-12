#!/usr/bin/env python3
"""Stop everything that drives the robots, at once (session plan 2026-10-28: "the one command"). Safe to run any time,
also when nothing is running.

    ./run-fleet-pi.sh stopall                     (the site's robots, a broadcast and every robot heard: below)
    python3 tools/fleet/stop_all.py               (the same)
    python3 tools/fleet/stop_all.py --only-listed --robot 192.168.1.250 ...    (the old way: only these addresses)

In order:
  1. Ctrl+C to every fleet run started in tmux (monday, practice, doe, ladder, ladder2, calib, freeroam, mc's own run is step 2): each
     program's own stop then halts its robots and closes its logs (the upload follows where the program uploads).
  2. Mission Control's Stop (POST /api/run/stop on 127.0.0.1:8095), if Mission Control is open.
  3. `stop` straight to every robot's command port (UDP 5006), STOP_REPEATS times, STOP_GAP_S apart. This does not
     need the log port, so it works while another program holds it (robot_cmd.py and the console cannot).

Which robots step 3 reaches (risk review 2 Nov, #2: the school robots have no fixed address, and the old list was home's
three addresses only). Since 2026-11-03, all of these together:
  a. the addresses given with --robot, else the site's robots with a fixed address (home: .250/.252/.253, as before);
  b. a BROADCAST `stop` (255.255.255.255 and each network's own broadcast address, e.g. 192.168.1.255): every robot on
     the Pi's WiFi hears it, whatever its address (the robots already answer a broadcast `ping` this way);
  c. every robot HEARD on the WiFi: the robots the fleet programs heard in the last HEARD_KEEP_H hours (each program's
     link writes them to ~/.config/mirlt/heard_robots.json), plus any robot that answers on the log port within
     LISTEN_S, when the port is free (while a run holds it, a and b still go out).
--only-listed keeps the old behaviour (a alone; with no --robot: home's three addresses). --no-broadcast and
--no-heard switch b or c off. A broken site file never stops this command: it says so and still broadcasts.
Prints what it did. Never kills a process.
"""
import argparse
import json
import os
import re
import socket
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
CMD_PORT = 5006            # tools/alvik_console.py CMD_PORT (laptop/Pi -> robot)
LOG_PORT = 5005            # robot -> laptop/Pi
ROBOT_IPS = ("192.168.1.250", "192.168.1.252", "192.168.1.253")
TMUX_RUNS = ("monday", "practice", "doe", "ladder", "ladder2", "calib", "freeroam")   # freeroam: 2026-11-05
MC_URL = "http://127.0.0.1:8095/api/run/stop"
STOP_REPEATS = 3           # chosen: UDP can drop a packet (27 Oct: up to 1 in 10); three copies 0.2 s apart. No DOE:
STOP_GAP_S = 0.2           # a repeated `stop` is harmless on the robot
LISTEN_S = 1.0             # chosen: robots send a status line about once a second while idle and answer a `ping` at
                           # once; 1 s catches every robot that is on. No DOE: a stop must not wait long
HEARD_KEEP_H = 24.0        # chosen: a robot heard today is worth a unicast `stop` (it costs one packet). No DOE
HEARD_FILE = os.environ.get("MIRLT_HEARD_FILE") or os.path.expanduser("~/.config/mirlt/heard_robots.json")


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


def stop_robots(ips, port=CMD_PORT, repeats=STOP_REPEATS, gap_s=STOP_GAP_S, broadcast=False):
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if broadcast:
        tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
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


# ------------------------------------------------------------------------------------------------ who to reach
def site_ips():
    """(the site's fixed addresses, words). Never raises: a broken site file is said, not fatal (this is the stop)."""
    try:
        sys.path.insert(0, HERE)
        import sites as SI
        s = SI.load()
        ips = [r["ip"] for r in s["robots"] if r.get("ip")]
        return ips, "site %s: %s" % (s["name"], ", ".join(ips) if ips else "no fixed addresses (robots found on the WiFi)")
    except BaseException as e:   # noqa: BLE001  (SystemExit from a broken site file included)
        return [], "the site file could not be read (%s): broadcast and heard robots only" % str(e).splitlines()[0]


def broadcast_addrs(run=subprocess.run):
    """255.255.255.255 plus each IPv4 network's own broadcast address (from `ip -4 -o addr`; 'brd 192.168.1.255')."""
    out = ["255.255.255.255"]
    try:
        r = run(["ip", "-4", "-o", "addr"], capture_output=True, text=True, timeout=2)
        for m in re.finditer(r"\bbrd (\d+\.\d+\.\d+\.\d+)", r.stdout or ""):
            if m.group(1) not in out:
                out.append(m.group(1))
    except (OSError, subprocess.SubprocessError):
        pass
    return out


def heard_from_file(path=None, keep_h=HEARD_KEEP_H, now=None):
    """{name: ip} of the robots the fleet's links heard in the last keep_h hours."""
    path = path or HEARD_FILE
    now = time.time() if now is None else now
    try:
        with open(path) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return {}
    out = {}
    for name, v in (d or {}).items():
        try:
            if now - float(v.get("t", 0)) <= keep_h * 3600 and v.get("ip"):
                out[str(name)] = str(v["ip"])
        except (AttributeError, TypeError, ValueError):
            continue
    return out


def remember(name, ip, path=None, now=None):
    """The link's note of a robot heard (alvik_console.Link calls it): heard_robots.json, {name: {ip, t}}."""
    if str(ip).startswith("127."):
        return                    # (e19, audit H2) a simulated robot (loopback) never replaces a real robot's address
    path = path or HEARD_FILE
    try:
        try:
            with open(path) as f:
                d = json.load(f)
        except (OSError, ValueError):
            d = {}
        d[str(name)] = {"ip": str(ip), "t": round(time.time() if now is None else now, 1)}
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = "%s.%d.tmp" % (path, os.getpid())
        with open(tmp, "w") as f:
            json.dump(d, f, indent=1)
        os.replace(tmp, path)
    except OSError:
        pass


def listen_for_robots(seconds=LISTEN_S, port=LOG_PORT, ping_to=("255.255.255.255",)):
    """{name: ip} of robots that talk on the log port within `seconds` (a `ping` broadcast first makes them answer).
    (None, words) when the port is held by a run (normal mid-run: the broadcast and the list still reach them)."""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind(("", port))
    except OSError:
        rx.close()
        return None
    rx.settimeout(0.1)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    for a in ping_to:
        try:
            tx.sendto(b"stop", (a, CMD_PORT))      # `stop` (not ping): anything that hears it stops now, and answers
        except OSError:
            pass
    tx.close()
    out = {}
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            try:
                data, addr = rx.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            parts = data.decode("utf-8", "replace").split("|", 2)
            name = parts[0] if len(parts) == 3 and parts[1].isdigit() else addr[0]
            out[name] = addr[0]
    finally:
        rx.close()
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", action="append", default=[], help="a robot's IP (repeatable; default: the site's)")
    ap.add_argument("--port", type=int, default=CMD_PORT)
    ap.add_argument("--mc-url", default=MC_URL)
    ap.add_argument("--no-tmux", action="store_true")
    ap.add_argument("--only-listed", action="store_true",
                    help="the old way: only the --robot addresses (none given: home's three); no broadcast, no heard")
    ap.add_argument("--no-broadcast", action="store_true")
    ap.add_argument("--no-heard", action="store_true")
    ap.add_argument("--listen-s", type=float, default=LISTEN_S)
    a = ap.parse_args(argv)
    runs = [] if a.no_tmux else tmux_ctrl_c()
    print("1. Ctrl+C sent to: %s" % (", ".join(runs) if runs else "(no fleet run in tmux)"), flush=True)
    mc = mc_stop(a.mc_url)
    print("2. Mission Control: %s" % (mc or "not open"), flush=True)
    if a.only_listed:
        ips = a.robot or list(ROBOT_IPS)
        n = stop_robots(ips, a.port)
        print("3. `stop` sent %d times to %s (only these: --only-listed)" % (n, ", ".join(ips)), flush=True)
    else:
        if a.robot:
            listed, how = list(a.robot), "given: %s" % ", ".join(a.robot)
        else:
            listed, how = site_ips()
        n = stop_robots(listed, a.port) if listed else 0
        print("3a. Listed robots (%s): `stop` sent %d times" % (how, n), flush=True)
        if not a.no_broadcast:
            bc = broadcast_addrs()
            n = stop_robots(bc, a.port, broadcast=True)
            print("3b. Broadcast `stop` to every robot on the WiFi (%s): sent %d times" % (", ".join(bc), n), flush=True)
        if not a.no_heard:
            heard = heard_from_file()
            live = listen_for_robots(a.listen_s, ping_to=[] if a.no_broadcast else broadcast_addrs())
            if live:
                heard.update(live)
            extra = sorted(set(heard.values()) - set(listed))
            n = stop_robots(extra, a.port) if extra else 0
            print("3c. Robots heard on the WiFi: %s%s; `stop` sent %d times to the ones not listed above" % (
                ", ".join("%s at %s" % kv for kv in sorted(heard.items())) or "none",
                "" if live is not None else " (the log port is held by a run: robots it holds got 3a/3b and its own "
                                            "Ctrl+C)", n), flush=True)
    print("Done. The robots should be standing still. If one still moves: pick it up and switch it off.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
