#!/usr/bin/env python3
"""No pictures from a camera (build e12, Nick 10 Nov: "a warning or a stop when the cameras send no pictures").

The camera bridges (tag_bridge.py at a roof-tag site, cam_bridge.py at home) call FrameWatch.saw(camera) for every
picture the recorder's stream gives. A small thread checks every CHECK_S: a camera with no new picture for NO_PICTURE_S
is said LOUDLY (a line in capitals, repeated every REPEAT_S while it lasts), written as a `no_pictures` event in the
bridge's events.jsonl and sent to the run as a message {"type": "camera", "state": "no_pictures", ...} on the same port
as the fixes. When pictures come back: `pictures_back` (event and message, state "pictures") with how long it lasted.

What the robots do is NOT decided here: with no pictures there are no camera fixes, and each run's existing safety rules
for a robot the camera no longer sees take over (the calibration stops a leg after LOST_HOLD_S and holds; school
missions hold a lost robot; free roam drives on its dead-reckoning bubble, then find-me / holds). They carry on when
fixes come back (graceful degradation: a run ends only where those rules say safety cannot be kept). The runs print the
message loudly and log it (freeroam events, run_plan log, calib events) so the cause is named, not just "lost".

Switch: MIRLT_FRAME_WATCH=0 (the bridges as before e12).
"""
import os
import threading
import time

VERSION = "2026-11-10e13"   # e13 (10 Nov, Nick: derived, risk-based thresholds): the camera's "no pictures" is now a
#                             gap longer than this stream's own normal gaps (derived from the pictures it sent), not a
#                             fixed 3 s; what each ROBOT does is decided by the run from its own risk
#                             (safety_rules.no_picture_limit). MIRLT_FRAME_WATCH_GAP=fixed: e12's 3 s. 2026-11-10a: e12
NO_PICTURE_S = 3.0      # e12's fixed 3 s: now only (a) the switch MIRLT_FRAME_WATCH_GAP=fixed and (b) the start-up limit
                        # until GAP_MIN_N gaps have been measured (the stream is connecting)
GAP_FACTOR = 3.0        # (e13) "no pictures" = no picture for GAP_FACTOR x this stream's 99th-percentile gap between
                        # pictures (its last GAP_WINDOW gaps): a stream that normally hiccups for 0.2 s is called silent
                        # after 0.6 s. CHOSEN; DOE row (docs/doe-backlog.md, e13): the factor against false "no
                        # pictures" in real runs' logs (the 5-minute file roll-over included)
GAP_FLOOR_S = 0.3       # (e13) never under this (half the runs' 0.6 s "lost" definition). CHOSEN, no DOE (a floor)
GAP_WINDOW = 300        # gaps kept per camera (10-20 s of pictures). Not a tuning number
GAP_MIN_N = 50          # gaps measured before the derived limit is used
REPEAT_S = 10.0         # chosen: said again this often while it lasts. No DOE (a reminder)
CHECK_S = 0.5           # how often the thread looks. No DOE


def enabled():
    import sys as _s
    _d = os.path.dirname(os.path.abspath(__file__))
    if _d not in _s.path:
        _s.path.insert(0, _d)
    import safety_group as SG_                    # (e19, audit 8: through the Safety group, as every run type)
    return SG_.flag("MIRLT_FRAME_WATCH")


class FrameWatch:
    def __init__(self, cams, send=None, event=None, say=print, gap_s=None, clock=time.time, thread=True):
        self.cams = list(cams)
        self.send, self.event, self.say_ = send, event, say
        self.fixed = gap_s is not None or os.environ.get("MIRLT_FRAME_WATCH_GAP", "derived") == "fixed"
        self.gap_s, self.clock = float(gap_s if gap_s is not None else NO_PICTURE_S), clock
        self.intervals = {c: [] for c in self.cams}   # (e13) the gaps between this camera's pictures
        t = clock()
        self.last = {c: t for c in self.cams}       # the start counts as a picture (the stream is connecting)
        self.out = {}                               # camera -> (since, last said)
        self.gaps = []                              # (camera, start, length s) of every gap that ended
        self.lock = threading.Lock()
        self.stop_ = threading.Event()
        self.th = None
        if thread:
            self.th = threading.Thread(target=self._loop, daemon=True, name="frame-watch")
            self.th.start()

    def saw(self, cam):
        now = self.clock()
        back = None
        with self.lock:
            if cam in self.last and cam not in self.out:
                iv = self.intervals.setdefault(cam, [])
                iv.append(now - self.last[cam])
                del iv[:-GAP_WINDOW]
            self.last[cam] = now
            if cam in self.out:
                since, _ = self.out.pop(cam)
                back = now - since
                self.gaps.append((cam, since, back))
        if back is not None:
            self._tell(cam, "pictures", "CAMERA %s: pictures again after %.1f s with none; the robots carry on as their "
                                        "rules allow" % (cam, back), gap_s=round(back, 2))

    def limit_s(self, cam):
        """(e13) This camera's "no pictures" limit: GAP_FACTOR x its 99th-percentile gap between pictures (at least
        GAP_FLOOR_S); e12's fixed NO_PICTURE_S until GAP_MIN_N gaps are measured, or with the switch."""
        iv = self.intervals.get(cam) or []
        if self.fixed or len(iv) < GAP_MIN_N:
            return self.gap_s
        q = sorted(iv)[min(len(iv) - 1, int(0.99 * len(iv)))]
        return max(GAP_FLOOR_S, GAP_FACTOR * q)

    def check(self):
        now = self.clock()
        tell = []
        with self.lock:
            for c, t in self.last.items():
                age = now - t
                if age < self.limit_s(c):
                    continue
                st = self.out.get(c)
                if st is None:
                    self.out[c] = (t, now)
                    tell.append((c, age, True))
                elif now - st[1] >= REPEAT_S:
                    self.out[c] = (st[0], now)
                    tell.append((c, age, False))
        for c, age, first in tell:
            lim = self.limit_s(c)
            self._tell(c, "no_pictures", "CAMERA %s: NO NEW PICTURE FOR %.1f S%s (its normal gaps call for one within "
                                         "%.1f s). Each robot is slowed or held by its own risk (speed, heading, room to "
                                         "the edge); they carry on when pictures come back. Check the camera's cable and "
                                         "the recorder (./run-fleet-pi.sh status)" % (
                                             c.upper(), age, "" if first else " (STILL NONE)", lim),
                       age_s=round(age, 1), first=first, limit_s=round(lim, 2))
        return [c for c, _a, _f in tell]

    def none_now(self):
        with self.lock:
            return sorted(self.out)

    def _tell(self, cam, state, words, **kw):
        try:
            self.say_(words)
        except Exception:  # noqa: BLE001
            pass
        if self.event:
            try:
                self.event("no_pictures" if state == "no_pictures" else "pictures_back", cam=cam, **kw)
            except Exception:  # noqa: BLE001
                pass
        if self.send:
            try:
                self.send(dict({"type": "camera", "state": state, "cam": cam, "t": round(self.clock(), 3),
                                "words": words}, **kw))
            except Exception:  # noqa: BLE001
                pass

    def _loop(self):
        while not self.stop_.wait(CHECK_S):
            self.check()

    def close(self):
        self.stop_.set()
        if self.th is not None:
            self.th.join(timeout=2.0)


def camera_words(msg):
    """The words a run says for a {"type": "camera"} message (None for anything else)."""
    if not isinstance(msg, dict) or msg.get("type") != "camera":
        return None
    return str(msg.get("words") or "CAMERA %s: %s" % (msg.get("cam"), msg.get("state")))
