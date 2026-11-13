#!/usr/bin/env python3
"""Wake sleeping robots at the start of every run (e22, the 13 Nov 08:14 home run: a free roam started from Mission
Control found none of the three robots and none moved; they were in deep sleep from "Robots sleep as soon as a run ends",
start_job.py sleep_when_done). A sleeping robot hears nothing until its check-in: every N minutes (the site's
sleep_checkin_min, sent as `sleep N`) it wakes for sleep_awake_s (20 s), says "awake for a check-in", and stays awake if
`wake` reaches it then (firmware home_lab.ino). So a run start never just fails to find robots: it keeps sending `wake`
(to every robot it hears the moment it is heard, to their known addresses and the broadcast) for one whole check-in
interval plus the awake window, says plainly which robots are "asleep, waking (next check-in in ~N s)", and writes the
same into the run's wake.json for Mission Control.

    import wake_robots as WR
    WR.note_sleep(names, checkin_min, why)          # where `sleep N` is sent (start_job, idle_after_run)
    w = WR.Waker(link, names, ips, say=print, status_path=os.path.join(run_dir, "wake.json"))
    while not w.done() and not w.expired(): w.step(); pump(0.3)

Changelog: e22-2026-11-13a first version (free roam, missions and visit points; calibration already waked: calib_run).
"""
import json
import os
import time

VERSION = "wake-robots-e22-2026-11-13a"
AWAKE_S = 20.0          # the firmware's sleep_awake_s (params.h): how long a check-in stays awake. Not chosen
BOOT_S = 10.0           # a check-in's boot and WiFi join before it is heard (the firmware's start, measured on the
                        # 5 Nov home check-ins: a few seconds). CHOSEN margin; DOE: no (it only lengthens the wait)
WAKE_EVERY_S = 1.0      # `wake` resent this often to robots not heard yet (wake_loop.py's rate): well inside the 20 s
                        # awake window. Not a tuning number
DEFAULT_CHECKIN_MIN = 10.0   # the sites' sleep_checkin_min default (start_job.sleep_robots)


def conf_dir():
    return os.path.expanduser(os.environ.get("MIRLT_CONF_DIR") or "~/.config/mirlt")


def state_path():
    return os.path.join(conf_dir(), "sleep_state.json")


def note_sleep(names, checkin_min, why="", path=None, now=None):
    """Remember when robots were sent `sleep N` (all robots: names None), so a later start can say when each checks in."""
    p = path or state_path()
    try:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        try:
            d = json.load(open(p))
        except (OSError, ValueError):
            d = {}
        rec = dict(t=time.time() if now is None else float(now), checkin_min=float(checkin_min), why=why)
        if names:
            for n in names:
                d.setdefault("robots", {})[n] = rec
        else:
            d["all"] = rec
            d["robots"] = {}
        tmp = p + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f, indent=1)
        os.replace(tmp, p)
    except OSError:
        pass


def sleep_record(name, path=None):
    try:
        d = json.load(open(path or state_path()))
    except (OSError, ValueError):
        return None
    return (d.get("robots") or {}).get(name) or d.get("all")


def checkin_min(site=None, name=None, path=None):
    """The check-in interval in force for this robot: the one it was put to sleep with, else the site's, else 10."""
    rec = sleep_record(name, path) if name else None
    if rec and rec.get("checkin_min"):
        return float(rec["checkin_min"])
    try:
        return float((site or {}).get("sleep_checkin_min") or DEFAULT_CHECKIN_MIN)
    except (TypeError, ValueError):
        return DEFAULT_CHECKIN_MIN


def next_checkin_s(name, now=None, path=None):
    """Seconds to this robot's next check-in from when it was sent to sleep (None: not known)."""
    rec = sleep_record(name, path)
    if not rec:
        return None
    now = time.time() if now is None else now
    per = 60.0 * float(rec.get("checkin_min") or DEFAULT_CHECKIN_MIN)
    if per <= 0:
        return None
    return per - ((now - float(rec["t"])) % per)


def wait_limit_s(ck_min):
    """How long a start waits for a sleeping robot: one whole check-in interval, its awake window and its boot (derived
    from the interval it was sent to sleep with; DOE: no)."""
    return 60.0 * float(ck_min) + AWAKE_S + BOOT_S


class Waker:
    """Wakes and waits for the robots of a run. link: alvik_console.Link (send_name, send_ip, send_all, robots)."""

    def __init__(self, link, names, ips=(), site=None, say=print, status_path=None, sleep_path=None, now=time.time):
        self.link, self.names, self.ips = link, list(names), list(ips or ())
        self.say, self.status_path, self.sleep_path, self.now = say, status_path, sleep_path, now
        self.t0 = now()
        self.ck = {n: checkin_min(site, n, sleep_path) for n in self.names}
        self.limit = max(wait_limit_s(c) for c in self.ck.values()) if self.names else 0.0
        self.woken, self.last_wake, self.last_say = set(), 0.0, -1e9
        self.write()

    def heard(self):
        return set(self.link.robots()) & set(self.names)

    def done(self):
        return set(self.names) <= self.heard()

    def expired(self):
        return self.now() - self.t0 >= self.limit

    def words(self, n):
        if n in self.heard():
            return "awake" + (" (woken at its check-in)" if n in self.woken else "")
        nx = next_checkin_s(n, self.now(), self.sleep_path)
        if nx is not None:
            return "asleep, waking (next check-in in ~%.0f s)" % nx
        left = max(0.0, self.limit - (self.now() - self.t0))
        return "not heard yet: asleep or off; waking (it checks in every %.0f min; waiting %.0f s more)" % (
            self.ck.get(n, DEFAULT_CHECKIN_MIN), left)

    def step(self):
        t = self.now()
        for n in self.heard() - self.woken:
            self.link.send_name(n, "wake")       # heard at its check-in (or awake already: `wake` only says so)
            self.woken.add(n)
        if t - self.last_wake >= WAKE_EVERY_S and not self.done():
            self.last_wake = t
            self.link.send_all("wake")
            for ip in self.ips + ["255.255.255.255"]:
                self.link.send_ip(ip, "wake")
        if t - self.last_say >= 15.0 and not self.done():
            self.last_say = t
            self.say("Waiting for sleeping robots: " + "; ".join("%s %s" % (n, self.words(n)) for n in self.names
                                                                    if n not in self.heard()))
        self.write()

    def write(self):
        if not self.status_path:
            return
        try:
            d = dict(t=self.now(), limit_s=round(self.limit, 1), since_s=round(self.now() - self.t0, 1),
                     robots={n: dict(words=self.words(n), awake=n in self.heard(),
                                     next_checkin_s=next_checkin_s(n, self.now(), self.sleep_path)) for n in self.names})
            tmp = self.status_path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(d, f)
            os.replace(tmp, self.status_path)
        except OSError:
            pass
