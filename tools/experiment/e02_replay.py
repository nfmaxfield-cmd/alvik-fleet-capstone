#!/usr/bin/env python3
"""Replay a recording through the E02 camera tracker, offline: how would the tracker have followed a robot on
real footage? For checking tracker changes against what really happened (the first real session, 2026-10-21,
GitHub release e02-setup-oct21), before anyone stands at the board again.

    python tools/experiment/e02_replay.py --video setup.avi --frames frames.csv --first-frame 52200 \
        --robot Center --at 69.5,73.7 --others 20.2,108.6 --t0 22:41:12 --t1 22:43:00 \
        --old logs/e02-.../track.csv --out replay_center

--first-frame: the recording's frame number of the video's first frame (a video cut from the full recording
starts later: setup.avi starts at frame 52200). Times are the recording day's local time (or epoch seconds).
--at: where the robot is at --t0 (floor cm); --others: robots standing still (x,y;x,y). The floor memory is
taken from the frames just before --t0. Writes <out>.csv (t, x, y, seen) and <out>.png: one picture per
--every seconds with the new tracker's position (green ring), the runner's position at the time (red cross,
from --old) and every outline that differs from the floor (thin: yellow robot-shaped, grey not).
"""
import argparse
import bisect
import csv
import datetime as dt
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import e02_camera as CAM  # noqa: E402
import e02_plan as PL  # noqa: E402

TZ = dt.timezone(dt.timedelta(hours=-4))


class Recording:
    """Frames of a recording by wall time. The video may be a cut: first_frame is the full recording's frame
    number of its first frame."""

    def __init__(self, video, frames_csv, first_frame=0, day=None):
        self.cap = cv2.VideoCapture(video)
        self.ts = [float(r["t_wall"]) for r in csv.DictReader(open(frames_csv))]
        self.first = first_frame
        self.n = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.day = day or dt.datetime.fromtimestamp(self.ts[0], TZ).date()
        self.pos = None

    def t_of(self, s):
        if isinstance(s, (int, float)):
            return float(s)
        try:
            return float(s)
        except ValueError:
            h, m, sec = s.split(":")
            secf = float(sec)
            return dt.datetime(self.day.year, self.day.month, self.day.day, int(h), int(m), int(secf),
                               int(round((secf % 1) * 1e6)), tzinfo=TZ).timestamp()

    def index(self, t):
        """Video frame index at (or just after) wall time t."""
        return max(0, min(self.n - 1, bisect.bisect_left(self.ts, t) - self.first))

    def t_frame(self, i):
        return self.ts[i + self.first]

    def read(self, i):
        if self.pos != i:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, f = self.cap.read()
        self.pos = i + 1 if ok else None
        return cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) if ok else None


class MjpgPiece:
    """Frames of a piece of an MJPG .avi cut by bytes (a recording split for upload, e.g. release
    e02-session-oct22's video.part-ab): each frame is a '00dc' chunk holding one JPEG, so a piece without the file's
    header can still be read. first_frame = the full recording's frame number of the piece's first whole frame
    (the recording's frame count minus the frames in the piece, for the last piece)."""

    def __init__(self, path, frames_csv, first_frame=None):
        import mmap
        self.f = open(path, "rb")
        self.m = mmap.mmap(self.f.fileno(), 0, access=mmap.ACCESS_READ)
        self.idx = []
        i = 0
        while True:
            j = self.m.find(b"00dc", i)
            if j < 0:
                break
            n = int.from_bytes(self.m[j + 4:j + 8], "little")
            if 0 < n < 2_000_000 and self.m[j + 8:j + 10] == b"\xff\xd8" and j + 8 + n <= len(self.m):
                self.idx.append((j + 8, n))
                i = j + 8 + n
            else:
                i = j + 4
        self.ts = [float(r["t_wall"]) for r in csv.DictReader(open(frames_csv))]
        self.n = len(self.idx)
        self.first = len(self.ts) - self.n if first_frame is None else first_frame
        self.day = dt.datetime.fromtimestamp(self.ts[0], TZ).date()

    def index(self, t):
        return max(0, min(self.n - 1, bisect.bisect_left(self.ts, t) - self.first))

    def t_frame(self, i):
        return self.ts[i + self.first]

    def read(self, i):
        if not 0 <= i < self.n:
            return None
        o, n = self.idx[i]
        return cv2.imdecode(np.frombuffer(self.m[o:o + n], np.uint8), cv2.IMREAD_GRAYSCALE)


def load_old_track(path):
    out = []
    if path and os.path.exists(path):
        for r in csv.DictReader(open(path)):
            out.append((float(r["t"]), r["robot"], float(r["x_cm"]), float(r["y_cm"])))
    return out


def old_at(track, t, robot, within=0.5):
    best = None
    for tt, rb, x, y in track:
        if rb == robot and abs(tt - t) <= within and (best is None or abs(tt - t) < best[0]):
            best = (abs(tt - t), x, y)
    return best[1:] if best else None


def legs_moving(path, robot, pad_s=0.8):
    """From a leg.csv: a function t -> is the robot driving a leg now (from its 'queued' to its final record)."""
    spans, open_ = [], {}
    for r in csv.DictReader(open(path)):
        if r["robot"] != robot:
            continue
        t, lid, res = float(r["t_recv"]), r["id"], r["result"]
        if res in ("queued", "started"):
            open_.setdefault(lid, t)
        elif res in ("done", "failed", "refused", "dup") and lid in open_:
            spans.append((open_.pop(lid) - pad_s, t + pad_s))
    return lambda t: any(a <= t <= b for a, b in spans)


def replay(rec, cam, robot, at, others, t0, t1, board_rect=PL.Board().to_dict()["rect"], every=1.0, old=None,
           out=None, expect_fn=None, bg_s=0.5, cols=4, moving_fn=None, tracker_kw=None):
    """Feed the frames t0..t1 through a fresh tracker that follows `robot` from `at`. Returns the samples
    [(t, x, y or None)] and writes <out>.csv / .png when out is given. tracker_kw: extra MotionTracker arguments
    (the side camera: z_cm, size_per_spot)."""
    tr = CAM.MotionTracker(cam, ((board_rect[0] + board_rect[2]) / 2, (board_rect[1] + board_rect[3]) / 2),
                           board_rect=tuple(board_rect), **(tracker_kw or {}))
    i0, i1 = rec.index(t0), rec.index(t1)
    nb = max(3, int(round(bg_s * 25)))
    bg = [rec.read(i) for i in range(max(0, i0 - nb), i0)]
    tr.set_background([b for b in bg if b is not None], robots=[at] + list(others))
    for k, p in enumerate(others):
        tr.park("other%d" % k, p[0], p[1])
    tr.follow(robot, at[0], at[1], t=rec.t_frame(i0))
    samples, sheet = [], []
    next_pic = rec.t_frame(i0)
    for i in range(i0, i1 + 1):
        img = rec.read(i)
        if img is None:
            break
        t = rec.t_frame(i)
        if expect_fn:
            h = expect_fn(t)
            if h:
                tr.expect(h[0], h[1], h[2])
        if moving_fn is not None:
            mv = moving_fn(t)
            if mv and tr.hint is not None and tr.hint[3] == float("inf"):
                tr.clear_expect()
            elif not mv and (tr.hint is None or tr.hint[3] != float("inf")):
                tr.hold()
        s = tr.push(img, t)
        p = tr.where()
        samples.append((t, s[1] if s else None, s[2] if s else None))
        if out and t >= next_pic:
            next_pic += every
            sheet.append(draw(img, cam, tr, p, old_at(old or [], t, robot), t))
    if out:
        with open(out + ".csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x_cm", "y_cm", "seen"])
            for t, x, y in samples:
                w.writerow(["%.3f" % t, "" if x is None else "%.2f" % x, "" if y is None else "%.2f" % y, int(x is not None)])
        if sheet:
            rows = []
            for k in range(0, len(sheet), cols):
                row = sheet[k:k + cols]
                row += [np.zeros_like(sheet[0])] * (cols - len(row))
                rows.append(np.hstack(row))
            cv2.imwrite(out + ".png", np.vstack(rows))
    return samples


def draw(img, cam, tr, p, old, t, crop=None):
    v = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for c in tr.last_cands:
        cv2.circle(v, (int(c["u"]), int(c["v"])), 4, (0, 220, 220) if c["shape"] else (140, 140, 140), 1)
    if p is not None:
        u, vv = cam.project((p[1], p[2], CAM.ROBOT_TOP_CM))
        cv2.circle(v, (int(u), int(vv)), int(tr.rpx * 0.7), (0, 255, 0), 2)
    if old is not None:
        u, vv = cam.project((old[0], old[1], CAM.ROBOT_TOP_CM))
        if np.isfinite(u) and np.isfinite(vv):
            cv2.drawMarker(v, (int(u), int(vv)), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
    cv2.putText(v, dt.datetime.fromtimestamp(t, TZ).strftime("%H:%M:%S"), (8, 470), 0, 0.6, (0, 0, 255), 2)
    v = cv2.resize(v, (320, 240))
    return v


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", required=True)
    ap.add_argument("--frames", required=True)
    ap.add_argument("--first-frame", type=int, default=0)
    ap.add_argument("--calib", default=os.path.join(HERE, "data", "overhead_floorcal_roam1.json"))
    ap.add_argument("--robot", required=True)
    ap.add_argument("--at", required=True)
    ap.add_argument("--others", default="")
    ap.add_argument("--t0", required=True)
    ap.add_argument("--t1", required=True)
    ap.add_argument("--old")
    ap.add_argument("--legs", help="the robot logs' leg.csv: between its legs the robot stands still (tracker.hold)")
    ap.add_argument("--every", type=float, default=1.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    rec = Recording(a.video, a.frames, a.first_frame)
    cam = CAM.load_camera(a.calib)
    at = tuple(float(v) for v in a.at.split(","))
    others = [tuple(float(v) for v in p.split(",")) for p in a.others.split(";") if p.strip()]
    s = replay(rec, cam, a.robot, at, others, rec.t_of(a.t0), rec.t_of(a.t1), every=a.every,
               old=load_old_track(a.old), out=a.out, moving_fn=legs_moving(a.legs, a.robot) if a.legs else None)
    seen = sum(1 for q in s if q[1] is not None)
    print("%d frames, robot seen in %d (%.0f%%); wrote %s.csv, %s.png" % (len(s), seen, 100.0 * seen / max(1, len(s)),
                                                                        a.out, a.out))


if __name__ == "__main__":
    main()
