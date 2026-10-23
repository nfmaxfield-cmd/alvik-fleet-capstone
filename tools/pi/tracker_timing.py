#!/usr/bin/env python3
"""How fast is the tracker on this computer? (Nick, 2026-10-23: could a whole session run on the Pi?)

Replays a stretch of a recorded session through the live tracker (e02_camera.MotionTracker) three ways and prints
the time per picture: decoding the picture alone, the motion detector alone, and the detector with the lock-on
tracker (what the runner uses). Run it on the Pi and on the laptop with the same footage to compare.

    python3 tools/pi/tracker_timing.py                       # on the Pi: this morning's recording, 30 s of Left
    python3 tools/pi/tracker_timing.py --seconds 60
    python tools\\pi\\tracker_timing.py --video recordings_oct23\\video.avi --frames recordings_oct23\\frames.csv

Defaults: the 2026-10-23 morning recording on the Pi (~/recordings/20261023-084942_E02), 30 s from 09:14:35 laptop
time, Left at (37.0, 104.5) cm (track.csv), the Pi's clock 1.6 s ahead of the laptop's.
"""
import argparse
import csv
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
EXP = os.path.join(TOOLS, "experiment")
for p in (EXP, TOOLS, os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import e02_camera as CAM  # noqa: E402

REC = os.path.expanduser("~/recordings/20261023-084942_E02")


def frames_table(path):
    with open(path) as f:
        return [(int(r["frame"]), float(r["t_wall"])) for r in csv.DictReader(f)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default=os.path.join(REC, "video.avi"))
    ap.add_argument("--frames", default=os.path.join(REC, "frames.csv"))
    ap.add_argument("--t0", type=float, default=1792761275.0, help="laptop time to start at (track.csv)")
    ap.add_argument("--offset", type=float, default=1.6, help="the Pi's clock minus the laptop's (s)")
    ap.add_argument("--at", type=float, nargs=2, default=(36.98, 104.54), help="the robot's place then (cm)")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--calib", default=os.path.join(EXP, "data", "overhead_floorcal_roam1.json"))
    a = ap.parse_args()
    if not os.path.exists(a.video):
        sys.exit("No video at %s (give --video and --frames)" % a.video)
    tab = frames_table(a.frames)
    t_pi = a.t0 + a.offset
    i0 = min(range(len(tab)), key=lambda k: abs(tab[k][1] - t_pi))
    n = int(a.seconds * 25)
    cap = cv2.VideoCapture(a.video)
    cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, i0 - 12))
    t = time.perf_counter()
    imgs = []
    for _ in range(n + 12):
        ok, fr = cap.read()
        if not ok:
            break
        imgs.append(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY) if fr.ndim == 3 else fr)
    dec_ms = 1000 * (time.perf_counter() - t) / max(1, len(imgs))
    if len(imgs) < 24:
        sys.exit("Too few pictures read from %s" % a.video)
    cam = CAM.load_camera(a.calib)
    ts = [tab[min(len(tab) - 1, i0 - 12 + k)][1] - a.offset for k in range(len(imgs))]
    print("%s: %d pictures (%.0f s) from frame %d" % (os.uname().nodename if hasattr(os, "uname") else "this computer",
                                                      len(imgs) - 12, (len(imgs) - 12) / 25.0, i0))
    print("  decoding a picture:            %5.1f ms" % dec_ms)
    for name, lock in (("motion detector alone", False), ("detector + lock-on (runner)", True)):
        tr = CAM.MotionTracker(cam, (61.0, 61.0), board_rect=(0.0, 0.0, 122.15, 121.9))
        tr.use_lock = lock
        tr.set_background(imgs[:12], robots=[tuple(a.at)])
        tr.follow("R", a.at[0], a.at[1], t=ts[12])
        seen, t = 0, time.perf_counter()
        per = []
        for k in range(12, len(imgs)):
            s0 = time.perf_counter()
            seen += tr.push(imgs[k], ts[k]) is not None
            per.append(time.perf_counter() - s0)
        per = np.array(per) * 1000
        print("  %-30s %5.1f ms a picture (95%%: %5.1f ms) -> about %4.1f pictures a second; robot seen in %3.0f%%" % (
            name + ":", per.mean(), np.percentile(per, 95), 1000.0 / max(1e-6, per.mean() + dec_ms),
            100.0 * seen / len(per)))


if __name__ == "__main__":
    main()
