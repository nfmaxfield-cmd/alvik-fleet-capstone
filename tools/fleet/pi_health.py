#!/usr/bin/env python3
"""The computer's own heat and throttling, logged with every run (build e14; school 17:46, a dim room: the tag bridge's
detection time per picture climbed from 122 to 421 ms in a minute on the hot school Pi, 84 C on 9 Nov).

read(run=None): {"temp_c", "throttled" (the raw hex), "now": [flags], "since_boot": [flags]} from
/sys/class/thermal/thermal_zone0/temp and `vcgencmd get_throttled` (a Raspberry Pi; elsewhere the fields are None).
The flags are the firmware's bits: under-voltage, frequency capped, throttled, soft temperature limit (bits 0-3 now,
16-19 since boot). Logger(out): writes pi_health.csv every EVERY_S (cheap: one file read and one short command).
Tests stub `run` and the thermal file; a build never runs it against a real Pi.
"""
import os
import subprocess
import time

VERSION = "pi-health-2026-11-10e14"
EVERY_S = 5.0            # a row this often. Not tuned (a log)
THERMAL = "/sys/class/thermal/thermal_zone0/temp"
BITS = {0: "under-voltage", 1: "frequency capped", 2: "throttled", 3: "soft temperature limit"}


def _run(cmd):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
        return r.returncode, r.stdout
    except (OSError, subprocess.SubprocessError):
        return 127, ""


def flags(value):
    v = int(value)
    return [w for b, w in BITS.items() if v & (1 << b)], [w for b, w in BITS.items() if v & (1 << (b + 16))]


def read(run=None, thermal=THERMAL):
    run = run or _run
    out = {"temp_c": None, "throttled": None, "now": None, "since_boot": None}
    try:
        with open(thermal) as f:
            out["temp_c"] = round(int(f.read().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        pass
    rc, txt = run(["vcgencmd", "get_throttled"])
    if rc == 0 and "=" in txt:
        try:
            v = int(txt.strip().split("=")[-1], 16)
            out["throttled"] = hex(v)
            out["now"], out["since_boot"] = flags(v)
        except ValueError:
            pass
    return out


class Logger:
    def __init__(self, out, run=None, thermal=THERMAL, clock=time.time):
        self.path = os.path.join(out, "pi_health.csv") if out else None
        self.run, self.thermal, self.clock = run, thermal, clock
        self.t = -1e9
        self.last = None

    def tick(self, extra=None):
        """A row when EVERY_S has passed: time, temperature, throttle flags, and the caller's extra (the detection
        ms per camera). Returns the reading (or None)."""
        t = self.clock()
        if t - self.t < EVERY_S:
            return None
        self.t = t
        r = read(self.run, self.thermal)
        self.last = r
        if self.path:
            new = not os.path.exists(self.path)
            try:
                with open(self.path, "a") as f:
                    if new:
                        f.write("t,temp_c,throttled,now,since_boot,detect_ms\n")
                    f.write("%.2f,%s,%s,%s,%s,%s\n" % (
                        t, "" if r["temp_c"] is None else r["temp_c"], r["throttled"] or "",
                        "|".join(r["now"] or []), "|".join(r["since_boot"] or []),
                        "|".join("%s=%.0f" % kv for kv in (extra or {}).items())))
            except OSError:
                pass
        return r

    def words(self):
        r = self.last or {}
        if r.get("temp_c") is None and not r.get("throttled"):
            return ""
        w = "%s C" % r["temp_c"] if r.get("temp_c") is not None else ""
        if r.get("now"):
            w += (", " if w else "") + "NOW " + ", ".join(r["now"])
        elif r.get("since_boot"):
            w += (", " if w else "") + "earlier: " + ", ".join(r["since_boot"])
        return w
