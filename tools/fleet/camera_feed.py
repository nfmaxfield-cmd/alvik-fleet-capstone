#!/usr/bin/env python3
"""Is each camera actually sending NEW pictures? Checked before any binding (build e13; coordinator, from the real
school calibration at 11:48 on 10 Nov: the tag bridge ran 16 s with "overhead 0 ms" on every line and no tag read, the
reference tags included, so the recorder's stream sent no pictures; yet the run said "no roof tag is seen by the
camera ... check that the robot is in zones 1-2 and its tag faces up". A stalled camera must never be blamed on a robot
or a tag).

The tag bridge writes <its folder>/frames.json about once a second (tag_bridge.Bridge.feed_write): per camera, the
pictures decoded and how many were NEW (different from the one before: a frozen stream that re-sends one picture, or a
blank one, does not count), and the reference tags read. check() waits until every camera shows new pictures rising
(or a reference tag read); a camera that does not is said plainly, and when NO camera sends pictures the run is
refused with:

    camera overhead is sending no pictures: is the recorder running (./run-fleet-pi.sh rec) and the USB cable in?

Switch: MIRLT_FEED_CHECK=0 (no check: the way before e13).
"""
import json
import os
import time

VERSION = "camera-feed-2026-11-10e13"
NEW_MIN = 3             # new pictures that prove a camera is live: more than one (a single picture can be the last one
                        # buffered before a stall). Not a tuning number: any live camera gives 10-30 a second
WAIT_S = 12.0           # CHOSEN: how long the check waits for the first new pictures (the bridge needs a few seconds
                        # to connect to the stream and load its model). DOE: no; a log check of the time from the
                        # bridge's start to its first picture in real runs (bridge.txt) would set it
POLL_S = 0.5
DIFF_GREY = 0.5         # a picture is NEW when it differs from the one before by more than this mean grey (sensor
                        # noise alone is 2-4 grey levels: a real live picture always does). Not a tuning number
BLANK_STD = 1.0         # a picture with less spread than this is blank (a covered lens, a dead stream)


def enabled():
    return os.environ.get("MIRLT_FEED_CHECK", "1") != "0"


def words_for(cam):
    return ("camera %s is sending no pictures: is the recorder running (./run-fleet-pi.sh rec) and the USB cable in? "
            "(Nothing moved; this is not the robots or their tags.)" % cam)


def read(folder):
    try:
        with open(os.path.join(folder, "frames.json")) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def check(folder, cams=None, wait_s=WAIT_S, clock=time.time, sleep=time.sleep):
    """(ok, words, per camera {new, pictures, refs}). folder: the bridge's output folder. cams: the cameras that must
    send (None: every camera in frames.json; none listed there after wait_s: 'the camera reader never started')."""
    if not enabled():
        return True, "camera check off (MIRLT_FEED_CHECK=0)", {}
    t_end = clock() + wait_s
    base = None
    d = {}
    while True:
        d = read(folder).get("cams") or {}
        if base is None and d:
            base = {c: int(v.get("new", 0)) for c, v in d.items()}
        names = list(cams) if cams else list(d)
        live = {c: (int((d.get(c) or {}).get("new", 0)) >= NEW_MIN or int((d.get(c) or {}).get("refs", 0)) > 0)
                for c in names}
        if names and all(live.values()):
            return True, "camera%s sending pictures: %s" % ("s" if len(names) > 1 else "", ", ".join(
                "%s (%d new)" % (c, int((d.get(c) or {}).get("new", 0))) for c in names)), d
        if clock() >= t_end:
            break
        sleep(POLL_S)
    if not names:
        return False, words_for("overhead") + " (the camera reader never reported a picture)", d
    dead = [c for c, ok in live.items() if not ok]
    if len(dead) < len(names):
        # (degrade, not stop: CLAUDE.md) another camera is live: the run goes on with it, the dead one said plainly
        return True, "WARNING: " + " ".join(words_for(c) for c in dead) + " The run carries on with %s." % ", ".join(
            c for c in names if c not in dead), d
    return False, " ".join(words_for(c) for c in dead), d


class FeedStats:
    """Kept by the tag bridge: per camera pictures, new pictures, reference tags read; written as frames.json."""

    def __init__(self, folder, cams):
        self.folder = folder
        self.s = {c: dict(pictures=0, new=0, blank=0, frozen=0, refs=0, last_t=None) for c in cams}
        self.prev = {}
        self.t_w = 0.0

    def saw(self, cam, grey, t):
        import numpy as np
        st = self.s.setdefault(cam, dict(pictures=0, new=0, blank=0, frozen=0, refs=0, last_t=None))
        st["pictures"] += 1
        small = grey[::8, ::8].astype(np.float32)
        if float(small.std()) < BLANK_STD:
            st["blank"] += 1
            return False
        p = self.prev.get(cam)
        self.prev[cam] = small
        if p is not None and p.shape == small.shape and float(np.abs(small - p).mean()) <= DIFF_GREY:
            st["frozen"] += 1
            return False
        st["new"] += 1
        st["last_t"] = round(float(t), 3)
        return True

    def ref_read(self, cam, n=1):
        self.s.setdefault(cam, dict(pictures=0, new=0, blank=0, frozen=0, refs=0, last_t=None))["refs"] += n

    def write(self, now=None, force=False):
        now = time.time() if now is None else now
        if not force and now - self.t_w < 1.0:
            return
        self.t_w = now
        try:
            os.makedirs(self.folder, exist_ok=True)
            tmp = os.path.join(self.folder, "frames.json.tmp")
            with open(tmp, "w") as f:
                json.dump({"version": VERSION, "t": round(now, 3), "cams": self.s}, f)
            os.replace(tmp, os.path.join(self.folder, "frames.json"))
        except OSError:
            pass
