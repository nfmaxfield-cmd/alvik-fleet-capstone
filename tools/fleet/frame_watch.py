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

VERSION = "2026-11-10a"
NO_PICTURE_S = 3.0      # chosen: 3 s with no new picture (the recorder gives 15-30 a second, so 3 s is 45+ missing
                        # pictures). Not checked against real logs here. DOE: no; a log check of the longest gap between
                        # pictures in normal runs (the 5-minute file roll-over included) would confirm it is never reached
REPEAT_S = 10.0         # chosen: said again this often while it lasts. No DOE (a reminder)
CHECK_S = 0.5           # how often the thread looks. No DOE


def enabled():
    return os.environ.get("MIRLT_FRAME_WATCH", "1") != "0"


class FrameWatch:
    def __init__(self, cams, send=None, event=None, say=print, gap_s=NO_PICTURE_S, clock=time.time, thread=True):
        self.cams = list(cams)
        self.send, self.event, self.say_ = send, event, say
        self.gap_s, self.clock = float(gap_s), clock
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
            self.last[cam] = now
            if cam in self.out:
                since, _ = self.out.pop(cam)
                back = now - since
                self.gaps.append((cam, since, back))
        if back is not None:
            self._tell(cam, "pictures", "CAMERA %s: pictures again after %.1f s with none; the robots carry on as their "
                                        "rules allow" % (cam, back), gap_s=round(back, 2))

    def check(self):
        now = self.clock()
        tell = []
        with self.lock:
            for c, t in self.last.items():
                age = now - t
                if age < self.gap_s:
                    continue
                st = self.out.get(c)
                if st is None:
                    self.out[c] = (t, now)
                    tell.append((c, age, True))
                elif now - st[1] >= REPEAT_S:
                    self.out[c] = (st[0], now)
                    tell.append((c, age, False))
        for c, age, first in tell:
            self._tell(c, "no_pictures", "CAMERA %s: NO NEW PICTURE FOR %.0f S%s. Robots the camera no longer sees are "
                                         "held or stopped by the run's safety rules; they carry on when pictures come "
                                         "back. Check the camera's cable and the recorder (./run-fleet-pi.sh status)" % (
                                             c.upper(), age, "" if first else " (STILL NONE)"), age_s=round(age, 1),
                       first=first)
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
