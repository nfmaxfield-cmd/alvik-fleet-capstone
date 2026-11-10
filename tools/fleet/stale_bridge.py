#!/usr/bin/env python3
"""A camera reader (tag_bridge.py / cam_bridge.py) left running by an earlier run holds its UDP port, and the next
run's reader cannot start: the run then sees no roof tag at all (9 Nov school: a 15:56 tour's reader, orphaned when
Mission Control was restarted, held port 5011 and three tour starts failed with "no roof tag is seen").

Before a run starts its reader: whatever holds the port is found (/proc/net/udp -> socket inode -> process). If it is
one of OUR camera readers and ORPHANED (its parent is no run of ours: the run that started it has gone), it is stopped
(SIGINT, then SIGTERM), said plainly. Anything else holding the port is left alone and said plainly too.
Switch: MIRLT_STALE_BRIDGE=0 (the old way: nothing checked).

Changelog
  2026-11-10a  first version (Nick 9 Nov: "fix it so it doesn't happen again").
"""
import os
import signal
import time

VERSION = "2026-11-10a"
READERS = ("tag_bridge.py", "cam_bridge.py")
OUR_RUNS = ("mission_run.py", "calib_run.py", "wiggle_bind.py", "freeroam.py", "run_plan.py", "mission_control.py",
            "e02.py", "ladder.py")


def _cmd(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            return f.read().replace(b"\0", b" ").decode(errors="replace").strip()
    except OSError:
        return ""


def _ppid(pid):
    try:
        with open("/proc/%d/stat" % pid) as f:
            return int(f.read().rsplit(")", 1)[1].split()[1])
    except (OSError, ValueError, IndexError):
        return 0


def holders(port, proc="/proc"):
    """The pids holding UDP `port` on any address (IPv4 and IPv6)."""
    inodes = set()
    for fn in ("net/udp", "net/udp6"):
        try:
            with open(os.path.join(proc, fn)) as f:
                next(f, None)
                for ln in f:
                    p = ln.split()
                    if len(p) > 9 and int(p[1].rsplit(":", 1)[1], 16) == int(port):
                        inodes.add(p[9])
        except OSError:
            continue
    out = []
    if not inodes:
        return out
    for d in os.listdir(proc):
        if not d.isdigit():
            continue
        try:
            for fd in os.listdir(os.path.join(proc, d, "fd")):
                try:
                    t = os.readlink(os.path.join(proc, d, "fd", fd))
                except OSError:
                    continue
                if t.startswith("socket:[") and t[8:-1] in inodes:
                    out.append(int(d))
                    break
        except OSError:
            continue
    return out


def orphaned(pid):
    """True when no run of ours is its parent (the run that started it is gone)."""
    pp = _ppid(pid)
    return pp <= 1 or not any(r in _cmd(pp) for r in OUR_RUNS)


def clear(ports=(5011,), log=print, wait_s=5.0):
    """Stop orphaned camera readers holding `ports`. Returns [(pid, what was done)]."""
    if os.environ.get("MIRLT_STALE_BRIDGE", "1") == "0":
        return []
    done = []
    for port in ports:
        for pid in holders(port):
            if pid == os.getpid():
                continue
            c = _cmd(pid)
            if not any(r in c for r in READERS):
                log("PORT %d is held by another program (pid %d: %s): left alone; the camera reader may not start"
                    % (port, pid, c[:120] or "?"))
                done.append((pid, "left alone (not a camera reader)"))
                continue
            if not orphaned(pid):
                log("PORT %d is held by a camera reader of a run still going (pid %d): left alone. Stop that run first."
                    % (port, pid))
                done.append((pid, "left alone (its run is going)"))
                continue
            log("LEFTOVER camera reader from an earlier run held port %d (pid %d): stopping it" % (port, pid))
            for sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    os.kill(pid, sig)
                except OSError:
                    break
                t0 = time.time()
                while time.time() - t0 < wait_s and os.path.exists("/proc/%d" % pid):
                    time.sleep(0.2)
                if not os.path.exists("/proc/%d" % pid):
                    break
            gone = not os.path.exists("/proc/%d" % pid)
            done.append((pid, "stopped" if gone else "would not stop"))
            if not gone:
                log("  it would not stop: in the Pi window, kill %d" % pid)
    return done


if __name__ == "__main__":
    import sys
    print(clear(tuple(int(p) for p in sys.argv[1:]) or (5011,)))
