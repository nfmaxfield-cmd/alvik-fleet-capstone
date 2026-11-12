#!/usr/bin/env python3
"""What only the planner knows, for the Pi efficiency monitor (tools/pi/pi_monitor.py; Mission Control build, Nick
12 Nov: "we need to keep a close eye on this").

The planner (free roam's loop, run_plan's loop) calls Feed.tick() once a loop. Every EVERY_S it writes
monitor_feed.json in its own run folder (one small file replaced in place):
  loop      the loop's own work time a pass (ms: median, 95th percentile, max) and its period (ms, mean) over the last
            EVERY_S; free roam aims at freeroam.TICK_S (50 ms) a pass
  cameras   each camera's newest picture age as the planner holds it (s; free roam's frames, the recorder's pictures)
  detector  the trained detector's own numbers (tagfree.DetectorSource.summary(): calls, ms a call, its share of its
            core in the last window, refused for load, its cap)
Read-only: nothing here changes what the planner does; any error is swallowed (the feed is skipped, never the run).

Changelog
  2026-11-12a  first version (Mission Control build)
"""
import json
import os
import time

VERSION = "monitor-feed-2026-11-12a"
EVERY_S = 2.0      # chosen: as the monitor's own row (pi_monitor.EVERY_S). No DOE: a log's resolution
NAME = "monitor_feed.json"


def pct(xs, q):
    if not xs:
        return None
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))]


class Feed:
    def __init__(self, folder, every_s=EVERY_S, clock=time.time):
        self.path = os.path.join(folder, NAME) if folder else None
        self.every_s, self.clock = float(every_s), clock
        self.work, self.periods = [], []
        self.last_tick = None
        self.last_write = clock()
        self.n = 0

    def tick(self, work_s=None, runner=None, tagfree=None):
        """One pass of the planner's loop: its work time (s, or None when only the period is known)."""
        try:
            now = self.clock()
            if self.last_tick is not None:
                self.periods.append(now - self.last_tick)
            self.last_tick = now
            if work_s is not None:
                self.work.append(float(work_s))
            self.n += 1
            if now - self.last_write >= self.every_s:
                self.write(now, runner, tagfree)
        except Exception:  # noqa: BLE001  (the feed never stops the run)
            pass

    def write(self, now, runner=None, tagfree=None):
        self.last_write = now
        w = [x * 1000.0 for x in self.work]
        p = [x * 1000.0 for x in self.periods]
        d = {"t": round(now, 3), "version": VERSION, "pid": os.getpid(),
             "loop": {"n": len(p), "work_ms_p50": _r(pct(w, 0.5)), "work_ms_p95": _r(pct(w, 0.95)),
                      "work_ms_max": _r(max(w) if w else None), "period_ms_mean": _r(sum(p) / len(p) if p else None),
                      "period_ms_p95": _r(pct(p, 0.95))},
             "cameras": cameras_of(runner, now), "detector": detector_of(runner, tagfree, now)}
        self.work, self.periods = [], []
        if not self.path:
            return d
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.replace(tmp, self.path)
        return d


def _r(v, k=2):
    return None if v is None else round(v, k)


def cameras_of(runner, now):
    """{camera: newest picture age (s)} from the planner's frames (free roam: runner.frames.cams, each .latest =
    (t, picture)); {} when there are none (practice)."""
    out = {}
    fr = getattr(runner, "frames", None) if runner is not None else None
    for c in getattr(fr, "cams", None) or []:
        lt = getattr(c, "latest", None)
        try:
            out[str(getattr(c, "cam", "camera"))] = round(now - float(lt[0]), 3) if lt else None
        except (TypeError, ValueError, IndexError):
            continue
    return out


def detector_of(runner, tagfree, now):
    tf = tagfree if tagfree is not None else (getattr(runner, "tagfree", None) if runner is not None else None)
    src = getattr(tf, "source", None) if tf is not None else None
    if src is None or not hasattr(src, "summary"):
        return None
    s = src.summary()
    b = getattr(src, "budget", None)
    s["share_now"] = round(b.share_now(now), 4) if b is not None and hasattr(b, "share_now") else None
    return s
