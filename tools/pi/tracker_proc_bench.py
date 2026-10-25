#!/usr/bin/env python3
"""Pictures a second the tracker keeps up with, the old way against the new (2026-10-25 evening, MIRLT).

A recorded stretch is played at its recorded pace (25 a second, as live: a picture the tracker is too busy for is
skipped) through, in turn:
  before   the 15:45 arrangement: e02_camera.Feed (a thread of this process) with e02_camera.MotionTracker;
  thread   the same thread, with the new tracker (e02_alltrack.AllTracker: faster matching, every robot watched);
  process  the tracker process (e02_trackproc.ProcFeed): the new tracker in its own process, pinned to --cores.
In every mode this process also runs --load busy Python threads (the runner's other work, which competes with a
tracker thread for Python's lock; chosen: 1 thread busy half the time), so the "before" figure is the runner's
situation, not an idle one. The robot to follow and where the others stand come from the session's track.csv when
given (--track), else the first robot-shaped outlines.

    python3 tools/pi/tracker_proc_bench.py --clip <folder with clip.bin + clip_index.csv> --track <track.csv>
    python3 tools/pi/tracker_proc_bench.py --clip ~/recordings/<session>/cam0 --start 30000 --seconds 60

Prints, per mode: pictures handled a second (median of 1-s bins), the tracker's ms a picture, the picture's age when
its position was ready, and this process's and the tracker process's CPU. Writes --out (CSV) if given.
"""
import argparse
import csv
import os
import statistics as STS
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
EXP = os.path.join(TOOLS, "experiment")
for p in (EXP, TOOLS, os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import e02_alltrack as AT  # noqa: E402
import e02_camera as CAM  # noqa: E402
import e02_plan as PL  # noqa: E402
import e02_trackproc as TP  # noqa: E402

LOAD_BUSY_MS, LOAD_IDLE_MS = 5.0, 5.0     # chosen stand-in for the runner's other threads (half of one core in pure
                                          # Python). DOE: replace by a recorded profile of the runner (threads.csv)


class Load:
    def __init__(self, n):
        self.stop = False
        self.ths = [threading.Thread(target=self._run, daemon=True) for _ in range(n)]

    def _run(self):
        while not self.stop:
            end = time.perf_counter() + LOAD_BUSY_MS / 1000.0
            x = 0
            while time.perf_counter() < end:
                x += 1
            time.sleep(LOAD_IDLE_MS / 1000.0)

    def __enter__(self):
        for t in self.ths:
            t.start()
        return self

    def __exit__(self, *a):
        self.stop = True


class Decoded:
    """ClipPics, decoded in the reading thread (as the live stream source does)."""

    def __init__(self, src):
        self.src = src

    def __iter__(self):
        for p in self.src:
            if p.img is None:
                p.img = cv2.imdecode(np.frombuffer(p.jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
            yield p

    def close(self):
        self.src.close()


def track_rows(path, t_from, t_to):
    out = []
    if not path:
        return out
    with open(path) as f:
        for r in csv.DictReader(f):
            t = float(r["t"])
            if t <= t_to:
                out.append((t, r["robot"], float(r["x_cm"]), float(r["y_cm"])))
    return out


def driver(tr, rows, t_rec0, t_wall0, stop):
    """Replays the runner's follow / park calls from track.csv at the recorded pace."""
    last, act, i = {}, None, 0
    while i < len(rows) and rows[i][0] < t_rec0:
        last[rows[i][1]] = rows[i][2:]
        i += 1
    for n, q in last.items():
        tr.robot_at(q[0], q[1])
        tr.park(n, q[0], q[1])
    while i < len(rows) and not stop.is_set():
        t, rb, x, y = rows[i]
        d = t_wall0 + (t - t_rec0) - time.time()
        if d > 0:
            stop.wait(d)
        if rb != act:
            if act is not None and act in last:
                tr.park(act, *last[act])
            tr.follow(rb, x, y)
            act = rb
        last[rb] = (x, y)
        i += 1
    return last


def run_mode(mode, a, cam, board, rows, bg_imgs, robots0):
    perf = []
    src_spec = "%s@1@%d@%d" % (a.clip, a.start, int(a.seconds * 25))
    t_rec0 = None
    with open(os.path.join(a.clip, "clip_index.csv")) if os.path.exists(os.path.join(a.clip, "clip_index.csv")) \
            else open(os.path.join(a.clip, "frames.csv")) as f:
        rr = list(csv.DictReader(f))
        t_rec0 = float(rr[a.start]["t_wall"]) - TP.CAPTURE_DELAY_S
    centre = ((board.x0 + board.x1) / 2, (board.y0 + board.y1) / 2)
    walls = [("y", board.xboard_cm + board.uy * PL.ROBOT_HALF_CM), ("x", board.yboard_cm + board.ux * PL.ROBOT_HALF_CM)]
    stop = threading.Event()
    cpu0, w0 = time.process_time(), time.time()
    child_cpu = None
    with Load(a.load):
        if mode in ("before", "thread"):
            tr = (CAM.MotionTracker if mode == "before" else AT.AllTracker)(cam, centre)
            tr.set_area((board.x0, board.y0, board.x1, board.y1), walls=walls)
            tr.set_background(bg_imgs, robots=robots0)
            src = Decoded(TP.make_source("clip:" + src_spec))
            feed = CAM.Feed("x", tr, CAM.History(), source=src)
            feed.newest_only = True
            feed.on_perf = perf.append
            t_wall0 = time.time()
            th = threading.Thread(target=driver, args=(tr, rows, t_rec0, t_wall0, stop), daemon=True)
            feed.start()
            th.start()
            feed.join(a.seconds + 30)
            stop.set()
        else:
            feed = TP.ProcFeed("clip:" + src_spec, CAM.History(), cores=a.cores or None, cv_threads=a.cv_threads,
                               log=lambda m: None)
            feed.on_perf = perf.append
            feed.start()
            tr = feed.make_tracker(cam, centre)
            tr.set_area((board.x0, board.y0, board.x1, board.y1), walls=walls)
            tr.set_background(bg_imgs, robots=robots0)
            t_wall0 = time.time()
            th = threading.Thread(target=driver, args=(tr, rows, t_rec0, t_wall0, stop), daemon=True)
            th.start()
            pid = feed.pid
            end = time.time() + a.seconds + 30
            while feed.is_alive() and time.time() < end:
                time.sleep(0.2)
                try:
                    with open("/proc/%d/stat" % pid) as f_:
                        s = f_.read()
                    p_ = s[s.rindex(")") + 2:].split()
                    child_cpu = (int(p_[11]) + int(p_[12])) / os.sysconf("SC_CLK_TCK")
                except (OSError, ValueError):
                    pass
            stop.set()
            feed.close()
    wall = time.time() - w0
    cpu = time.process_time() - cpu0
    rows_ = perf[2:-1] if len(perf) > 4 else perf
    fps = [r["fps_handled"] for r in rows_]
    ms = [r["ms_mean"] for r in rows_]
    age = [r["age_mean_s"] for r in rows_]
    return dict(mode=mode, fps_median=round(STS.median(fps), 1) if fps else None,
                fps_p10=round(sorted(fps)[len(fps) // 10], 1) if fps else None,
                ms_median=round(STS.median(ms), 1) if ms else None,
                age_median_s=round(STS.median(age), 3) if age else None, seconds=len(rows_),
                this_cpu_pct=round(100 * cpu / wall, 1),
                tracker_proc_cpu_pct=None if child_cpu is None else round(100 * child_cpu / wall, 1),
                load_threads=a.load, cores=a.cores, cv_threads=a.cv_threads, cpus=os.cpu_count())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--clip", required=True, help="a folder with clip.bin + clip_index.csv, or video.avi + frames.csv")
    ap.add_argument("--track", default=None, help="the session's track.csv (who is followed when)")
    ap.add_argument("--calib", default=os.path.join(EXP, "data", "overhead_floorcal_roam1.json"))
    ap.add_argument("--state", default=None, help="the session's e02_state.json (the board)")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--load", type=int, default=1, help="busy Python threads beside the tracker (the runner's work)")
    ap.add_argument("--modes", default="before,thread,process")
    ap.add_argument("--cores", default="2,3" if (os.cpu_count() or 1) >= 4 else "",
                    help="the tracker process's cores (default 2,3 on a 4-core Pi; none elsewhere)")
    ap.add_argument("--cv-threads", type=int, default=TP.CV_THREADS)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    a.cores = [int(c) for c in a.cores.split(",") if c.strip()]
    cam = CAM.load_camera(a.calib)
    board = PL.Board()
    if a.state:
        import json
        with open(a.state) as f:
            board = PL.Board.from_dict(json.load(f)["board"])
    src = TP.make_source("clip:%s@0@%d@5" % (a.clip, a.start))
    bg_imgs = [cv2.imdecode(np.frombuffer(p.jpeg, np.uint8), 0) if p.img is None else p.img for p in src]
    t_first = None
    with open(os.path.join(a.clip, "clip_index.csv")) if os.path.exists(os.path.join(a.clip, "clip_index.csv")) \
            else open(os.path.join(a.clip, "frames.csv")) as f:
        rr = list(csv.DictReader(f))
        t_first = float(rr[a.start]["t_wall"])
    rows = track_rows(a.track, t_first - 3600, t_first + a.seconds + 5)
    last = {}
    for t, rb, x, y in rows:
        if t < t_first:
            last[rb] = (x, y)
    robots0 = list(last.values())
    out = []
    for m in a.modes.split(","):
        r = run_mode(m.strip(), a, cam, board, rows, bg_imgs, robots0)
        print(r, flush=True)
        out.append(r)
    if a.out:
        with open(a.out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
            w.writeheader()
            w.writerows(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
