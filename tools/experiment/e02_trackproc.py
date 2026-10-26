"""The camera tracker in its own process (2026-10-25 evening, MIRLT; Nick approved 19:03).

Why: in the 15:45 session the runner's tracker fell to 10-12 pictures a second once it followed a robot, while the
recorder (a separate program on the same Pi) held 25. The tracker was a thread of the runner's one Python process,
taking turns with the runner's other threads for Python's interpreter lock (GIL). The Pi has 4 cores.

What runs where:
  - the recorder (tools/pi/camrec.py, unchanged) opens the camera, as today, and serves every picture at /live;
    in a production session (no video) there is no recorder and this process opens the camera itself, so the camera
    is opened once either way;
  - this process (the "tracker process", pinned to 2 cores on the Pi): reads the pictures, decodes each into a ring in
    shared memory (e02_shmframes: no picture is ever pickled), runs the tracker (e02_alltrack.AllTracker: the
    followed robot as before, plus every parked robot in every picture) on the newest picture, keeps the last
    pictures' JPEG bytes (for the runner's measured frames and the live page's moments) and saves event clips
    (e02_framesaver) when asked;
  - the runner: gets every picture's number and every tracked position over a pipe (a few hundred bytes each), reads
    pictures from the shared memory when it needs them, and sends its commands (follow, park, hold, ...) back.

Three threads in this process: the reader (network or camera), the decoder (every picture into the ring), the tracker
(the newest decoded picture, commands applied between pictures). OpenCV and numpy let go of the GIL in their heavy
parts, so the decoder runs beside the tracker, and OpenCV uses CV_THREADS threads.

The runner side duck-types the old objects, so e02.py changes little:
  ProcFeed     like e02_camera.Feed (last_img, wait_frames, collect, hooks, kept, frame_at, on_sample, on_perf, ...)
  ProcTracker  like e02_camera.MotionTracker (follow, park, hold, idle, forget, expect, where, active, last_cands, ...)
A command changes the runner's copy at once (where(), active) and the tracker process's at its next picture; the
positions that come back are only taken once the process says it has applied the runner's latest command, so a
position from before a follow() can never be taken for the newly followed robot.

Run a replay through it (the bench): tools/pi/tracker_proc_bench.py.
"""
import os
import pickle
import queue
import struct
import subprocess
import sys
import threading
import time
import traceback
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live"), os.path.join(TOOLS, "pi")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Chosen numbers (DOE notes: docs/doe-backlog.md rows "Tracker process").
CV_THREADS = 2            # Nick's split: 2 cores for the tracker. DOE: 1/2/3 threads x pinned/unpinned on the Pi,
                          # pictures a second and the runner's own lag (tools/pi/tracker_proc_bench.py --cv-threads)
TRACKER_CORES = (2, 3)    # chosen: the recorder on core 1, the runner and the live page on core 0 (run-e02-pi.sh); the
                          # USB camera's interrupts land on core 0 on the Pi 4. DOE: the same bench, cores 0-1 vs 2-3
START_TIMEOUT_S = 20.0    # the process must say it is ready within this (imports and the first picture). No DOE
CALL_TIMEOUT_S = 10.0     # a question to the process (candidates, kept pictures) must be answered within this. No DOE
DECODE_QUEUE = 4          # pictures waiting to be decoded; past it the oldest is dropped (counted). No DOE: a buffer
CAPTURE_DELAY_S = 0.04    # e02_camera.Feed's own figure (the camera's capture delay), unchanged


# =============================================================================== messages
def send_msg(f, obj, lock=None):
    data = pickle.dumps(obj, protocol=4)
    if lock is not None:
        with lock:
            f.write(struct.pack("<I", len(data)))
            f.write(data)
            f.flush()
    else:
        f.write(struct.pack("<I", len(data)))
        f.write(data)
        f.flush()


def _exact(f, n):
    out = b""
    while len(out) < n:
        b = f.read(n - len(out))
        if not b:
            raise EOFError("pipe closed")
        out += b
    return out


def recv_msg(f):
    n = struct.unpack("<I", _exact(f, 4))[0]
    return pickle.loads(_exact(f, n))


# =============================================================================== picture sources (child side)
class Pic:
    __slots__ = ("jpeg", "img", "t_pi", "pi_frame", "t_arrived", "t")

    def __init__(self, jpeg, t_pi, pi_frame, t_arrived, img=None):
        self.jpeg, self.t_pi, self.pi_frame, self.t_arrived, self.img = jpeg, t_pi, pi_frame, t_arrived, img
        self.t = None


class StreamPics:
    """camrec.py's /live (the same as tools/live/sources.StreamSource, which it reuses), keeping each picture's JPEG
    bytes instead of decoding it here."""

    def __init__(self, url, log=print):
        import sources as S
        self.src = S.StreamSource(url, log=log)
        self.closed = False
        self.log = log

    def __iter__(self):
        k = 0
        while not self.closed:
            try:
                self.src._connect()
                for h, body, arrived in self.src.parts():
                    if self.closed:
                        return
                    pf = int(h.get("x-frame", k))
                    yield Pic(body, float(h["x-time"]), pf, arrived)
                    k += 1
            except (OSError, ConnectionError, KeyError, ValueError) as e:
                if self.closed:
                    return
                self.log("stream: %s; reconnecting in 1 s" % e)
                time.sleep(1.0)

    def close(self):
        self.closed = True
        self.src.closed = True
        try:
            if self.src.sock is not None:
                self.src.sock.close()
        except Exception:
            pass


class DevicePics:
    """The camera itself (a production session: no recorder). Opened as camrec.py opens it (MJPG, 640x480); the
    camera's own JPEG bytes are kept when OpenCV hands them over undecoded (CAP_PROP_CONVERT_RGB 0), else each
    picture is decoded by OpenCV and the bytes made again (JPEG 80)."""

    def __init__(self, device=0, width=640, height=480, fps=25, log=print, opener=None):
        self.log = log
        if opener is None:
            import camrec as CR

            def opener(d, w, h, f):
                return CR.open_camera(d, w, h, f, fail=lambda m: (_ for _ in ()).throw(RuntimeError(m)))
        self.cap = opener(device, width, height, fps)
        self.raw = False
        try:
            import cv2
            self.raw = bool(self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0))
        except Exception:
            self.raw = False
        self.closed = False
        self.n = 0

    def __iter__(self):
        import cv2
        while not self.closed:
            ok, fr = self.cap.read()
            now = time.time()
            if not ok or fr is None:
                time.sleep(0.01)
                continue
            if fr.ndim == 2 and fr.shape[0] == 1 or fr.ndim == 1:
                jpeg = fr.reshape(-1).tobytes()           # the camera's own MJPG bytes
                img = None
            else:
                img = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY) if fr.ndim == 3 else fr
                ok2, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                jpeg = buf.tobytes() if ok2 else None
            self.n += 1
            yield Pic(jpeg, now, self.n - 1, now, img=img)

    def close(self):
        self.closed = True
        try:
            self.cap.release()
        except Exception:
            pass


class ClipPics:
    """A recording replayed as if live (the bench and the tests). Either a recorder folder (video.avi + frames.csv)
    or a clip of raw JPEGs (clip.bin + clip_index.csv: frame, off, size, t_wall). speed 1: at the recorded pace (a
    picture the tracker is too busy for is skipped, as live); speed 0: every picture, each one only once the tracker
    has taken the previous one (lockstep: for checking results, not speed). Times: the recorded Pi times, arriving
    now."""

    def __init__(self, path, speed=1.0, start=0, count=0, lockstep=None, log=print):
        import csv
        self.speed = float(speed)
        self.closed = False
        self.lockstep = lockstep
        if os.path.isdir(path) and os.path.exists(os.path.join(path, "clip_index.csv")):
            self.kind = "bin"
            with open(os.path.join(path, "clip_index.csv")) as f:
                self.rows = [(int(r["frame"]), int(r["off"]), int(r["size"]), float(r["t_wall"]))
                             for r in csv.DictReader(f)]
            self.blob = open(os.path.join(path, "clip.bin"), "rb")
        else:
            self.kind = "avi"
            folder = path if os.path.isdir(path) else os.path.dirname(path)
            vid = os.path.join(folder, "video.avi") if os.path.isdir(path) else path
            with open(os.path.join(folder, "frames.csv")) as f:
                self.rows = [(int(r["frame"]), 0, 0, float(r["t_wall"])) for r in csv.DictReader(f)]
            import cv2
            self.cap = cv2.VideoCapture(vid)
            if start:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, start)
        self.rows = self.rows[start:start + count] if count else self.rows[start:]

    def __iter__(self):
        import cv2
        t_start, t0 = time.time(), None
        for fr, off, size, tw in self.rows:
            if self.closed:
                return
            if self.kind == "bin":
                self.blob.seek(off)
                jpeg, img = self.blob.read(size), None
            else:
                ok, im = self.cap.read()
                if not ok:
                    return
                img = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) if im.ndim == 3 else im
                ok2, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                jpeg = buf.tobytes() if ok2 else None
            if t0 is None:
                t0 = tw
            if self.speed > 0:
                due = t_start + (tw - t0) / self.speed
                d = due - time.time()
                if d > 0:
                    time.sleep(d)
            elif self.lockstep is not None:
                self.lockstep.acquire()
            yield Pic(jpeg, tw, fr, time.time(), img=img)

    def close(self):
        self.closed = True


def make_source(spec, log=print, lockstep=None):
    """'stream:<url>' | 'device:<n>' | 'clip:<path>[@speed[@start[@count]]]'."""
    kind, _, rest = spec.partition(":")
    if kind == "stream":
        return StreamPics(rest, log=log)
    if kind == "device":
        return DevicePics(int(rest or 0), log=log)
    if kind == "clip":
        p = rest.split("@")
        return ClipPics(p[0], speed=float(p[1]) if len(p) > 1 else 1.0, start=int(p[2]) if len(p) > 2 else 0,
                        count=int(p[3]) if len(p) > 3 else 0, lockstep=lockstep, log=log)
    raise ValueError("unknown picture source %r" % spec)


# =============================================================================== the tracker process
class Child:
    """The tracker process's own state and threads. cfg: dict(source, session_dir, cores, cv_threads, slots,
    capture_delay_s, keep_s, save_frames, frames_dir, lockstep)."""

    def __init__(self, cfg, out, inp):
        self.cfg = cfg
        self.out = out
        self.inp = inp
        self.wlock = threading.Lock()
        self.stopped = False
        self.cmds = queue.Queue()
        self.ring = None
        self.tracker = None
        self.tracking = True
        self.cmd_done = 0
        self.dec_q = deque()
        self.dec_cond = threading.Condition()
        self.new_cond = threading.Condition()
        self.newest = -1
        self.dropped_dec = 0
        self.skipped = 0
        self.lockstep = threading.Semaphore(1) if cfg.get("lockstep") else None
        import sources as S
        self.clock = S.ClockEnvelope()
        self.cap = float(cfg.get("capture_delay_s", CAPTURE_DELAY_S))
        self.keep_s = float(cfg.get("keep_s", 0.0))
        self.keep = deque()                       # (t, jpeg, pi_frame, t_pi)
        self.keep_lock = threading.Lock()
        self.saver = None
        self.perf = dict(t0=None, n=0, push_s=0.0, push_max=0.0, age_max=0.0, age_sum=0.0, skipped0=0, cpu0=None)
        self.err = None

    def send(self, obj):
        try:
            send_msg(self.out, obj, self.wlock)
        except (BrokenPipeError, OSError, ValueError):
            self.stopped = True

    def log(self, text):
        self.send(("log", str(text)))

    # --- setup --------------------------------------------------------------------------------------------------
    def setup(self):
        import cv2
        cv2.setNumThreads(int(self.cfg.get("cv_threads", CV_THREADS)))
        cores = self.cfg.get("cores")
        if cores:
            try:
                have = os.sched_getaffinity(0) | set(range(os.cpu_count() or 1))
                want = {int(c) for c in cores if int(c) < (os.cpu_count() or 1)}
                if want and want <= have:
                    os.sched_setaffinity(0, want)
            except (OSError, AttributeError, ValueError) as e:
                self.log("cores not set: %s" % e)
        # numpy's maths library: as many threads as OpenCV (set before numpy is first used for maths)
        if self.cfg.get("save_frames") and self.cfg.get("frames_dir"):
            import e02_framesaver as FS
            self.saver = FS.FrameSaver(self.cfg["frames_dir"], log=self.log)

    # --- threads ------------------------------------------------------------------------------------------------
    def reader(self, src):
        try:
            for pic in src:
                if self.stopped:
                    break
                pic.t = self._t(pic)                      # the runner's clock (this thread only: one clock fit)
                if self.saver is not None and pic.jpeg is not None:
                    self.saver.add(pic.jpeg, pic.t, pic.pi_frame)
                with self.dec_cond:
                    if len(self.dec_q) >= DECODE_QUEUE:
                        self.dec_q.popleft()
                        self.dropped_dec += 1
                    self.dec_q.append(pic)
                    self.dec_cond.notify()
        except Exception as e:
            self.err = "picture source: %s" % e
            self.log(self.err)
        finally:
            with self.dec_cond:
                self.dec_q.append(None)
                self.dec_cond.notify()

    def _t(self, pic):
        self.clock.add(pic.t_arrived, pic.t_pi)
        return pic.t_pi + self.clock.offset - self.cap

    def decoder(self):
        import cv2
        import numpy as np
        while not self.stopped:
            with self.dec_cond:
                while not self.dec_q and not self.stopped:
                    self.dec_cond.wait(0.5)
                if self.stopped:
                    break
                pic = self.dec_q.popleft()
            if pic is None:
                with self.new_cond:
                    self.newest = -2                      # the source ended
                    self.new_cond.notify_all()
                break
            img = pic.img
            if img is None and pic.jpeg is not None:
                img = cv2.imdecode(np.frombuffer(pic.jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            t = pic.t
            if self.ring is None:
                import e02_shmframes as SF
                self.ring = SF.FrameRing.create(slots=int(self.cfg.get("slots", SF.SLOTS)), height=img.shape[0],
                                                width=img.shape[1])
                self.send(("ready", dict(ring=self.ring.name, h=img.shape[0], w=img.shape[1], slots=self.ring.slots,
                                         pid=os.getpid(), cores=sorted(os.sched_getaffinity(0)))))
            try:
                n = self.ring.write(img, t, pic.t_pi, pic.pi_frame, pic.t_arrived)
            except ValueError as e:
                self.log("picture skipped: %s" % e)
                continue
            if self.keep_s > 0:
                jpeg = pic.jpeg
                if jpeg is None:
                    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    jpeg = buf.tobytes() if ok else None
                if jpeg is not None:
                    with self.keep_lock:
                        self.keep.append((t, jpeg, pic.pi_frame, pic.t_pi))
                        while self.keep and t - self.keep[0][0] > self.keep_s:
                            self.keep.popleft()
            self.send(("frm", n, t, pic.pi_frame, pic.t_pi, self.clock.offset))
            with self.new_cond:
                self.newest = n
                self.new_cond.notify_all()

    def cmd_reader(self):
        try:
            while True:
                m = recv_msg(self.inp)
                if m[0] == "stop":
                    break
                self.cmds.put(m)
                with self.new_cond:
                    self.new_cond.notify_all()
        except (EOFError, OSError, pickle.UnpicklingError):
            pass
        self.stopped = True
        with self.new_cond:
            self.new_cond.notify_all()
        with self.dec_cond:
            self.dec_cond.notify_all()

    # --- commands -----------------------------------------------------------------------------------------------
    def apply(self, m):
        """m = (kind, id, op, args, kwargs): kind 'c' (a command, no answer) or 'q' (a question: answered)."""
        kind, mid, op, a, k = m
        try:
            res = self._op(op, a, k)
            ok = True
        except Exception as e:
            res, ok = "%s: %s\n%s" % (type(e).__name__, e, traceback.format_exc(limit=3)), False
        if kind == "c":
            self.cmd_done = max(self.cmd_done, mid)
            if not ok:
                self.log("command %s failed: %s" % (op, res))
        else:
            self.send(("rep", mid, ok, res))

    def _op(self, op, a, k):
        tr = self.tracker
        if op == "make":
            import e02_alltrack as AT
            cam, centre, use_lock = a
            self.tracker = AT.AllTracker(cam, centre, **k)
            if use_lock is not None:
                self.tracker.use_lock = bool(use_lock)
            return None
        if op == "tracking":
            self.tracking = bool(a[0])
            return None
        if op == "keep_s":
            self.keep_s = float(a[0])
            return None
        if op == "kept":
            t_from, t_to = a
            with self.keep_lock:
                return [q for q in self.keep if q[0] >= t_from and (t_to is None or q[0] <= t_to)]
        if op == "frame_at":
            t, within = a
            with self.keep_lock:
                best = min(self.keep, key=lambda q: abs(q[0] - t)) if self.keep else None
            return None if best is None or abs(best[0] - t) > within else best
        if op == "event":
            if self.saver is None:
                return None
            ev = self.saver.event(*a, **k)
            return ev
        if op == "saver_summary":
            return None if self.saver is None else self.saver.summary()
        if op == "set_background_from":
            # pictures already in the ring (numbers), so they cross no pipe
            nums, robots = a
            frames = [r[0] for r in (self.ring.read(n) for n in nums) if r is not None]
            if tr is not None and frames:
                tr.set_background(frames, robots=robots)
            return len(frames)
        if op == "setattr":
            name, value = a
            if tr is not None:
                setattr(tr, name, value)
            return None
        if op == "hand_moved_discard":
            if tr is not None:
                with tr.lock:
                    tr.hand_moved.discard(a[0])
            return None
        if op == "floor_snapshot":
            if tr is None:
                return None
            with tr.lock:
                if tr.bg is None:
                    return None
                return dict(bg=tr.bg.copy(), known=tr.known.copy(), offset=float(tr.offset), thr=float(tr.thr),
                            unknown_extra=float(getattr(tr, "UNKNOWN_EXTRA", 14.0)))
        if op == "ping":
            return dict(pid=os.getpid(), cores=sorted(os.sched_getaffinity(0)))
        if tr is None:
            raise RuntimeError("no tracker yet (%s)" % op)
        if op == "candidates":
            with tr.lock:
                return tr.candidates()
        fn = getattr(tr, op)
        return fn(*a, **k)

    def drain(self):
        while True:
            try:
                m = self.cmds.get_nowait()
            except queue.Empty:
                return
            self.apply(m)

    # --- the tracker loop ---------------------------------------------------------------------------------------
    def track_loop(self):
        last = -1
        while not self.stopped:
            self.drain()
            with self.new_cond:
                if self.newest == last or self.newest == -1:
                    self.new_cond.wait(0.05)
                n = self.newest
            if n == -2:
                break
            if n < 0 or n == last:
                continue
            if self.lockstep is not None and self.tracker is None:
                continue                                     # lockstep (a check): no picture goes by untracked
            if last >= 0 and n > last + 1:
                self.skipped += n - last - 1
            last = n
            r = self.ring.read(n)
            if r is None:
                continue
            img, (t, t_pi, pf, t_arr) = r
            self.drain()                                     # commands that came while waiting
            tr = self.tracker if self.tracking else None
            a = time.time()
            s = None
            if tr is not None:
                try:
                    s = tr.push(img, t)
                except Exception as e:
                    self.log("tracker error: %s" % traceback.format_exc(limit=4))
                    self.err = "tracker: %s" % e
            b = time.time()
            if self.lockstep is not None:
                self.lockstep.release()
            if tr is not None:
                self.send(("trk", self._state(tr, n, t, s)))
            self._perf_add(a, b, t, tr)
        self.stopped = True

    def _state(self, tr, n, t, s):
        with tr.lock:
            lk = tr.lk
            lc = tr.last_check
            conf = dict(source=tr.source, score=round(lk.score, 2) if lk is not None else None,
                        locked=bool(lk is not None and lk.locked),
                        check_age_s=None if lc is None or tr.frame_t is None else round(tr.frame_t - lc[0], 1),
                        check_cm=None if lc is None else lc[1])
            d = dict(n=n, t=t, s=s, cmd=self.cmd_done, active=tr.active, pos=tr.pos, last_seen=tr.last_seen,
                     misses=tr.misses, source=tr.source, frame_t=tr.frame_t, protect=dict(tr.protect),
                     hand_moved=sorted(tr.hand_moved), moved_by_hand=tr.moved_by_hand, conf=conf,
                     stats=dict(tr.stats), stats_hold=dict(tr.stats_hold), in_hold=tr.in_hold,
                     last_cands=[dict(c) for c in (tr.last_cands or [])], hint=tr.hint, offset=float(tr.offset),
                     thr=float(tr.thr), lock_off=tr.lock_off, use_lock=tr.use_lock, tn=tr.n)
        if hasattr(tr, "parked_state"):
            d["parked"] = tr.parked_state()
            d["park_events"] = tr.take_park_events()
        return d

    def _perf_add(self, a, b, t, tr):
        p = self.perf
        if p["t0"] is None:
            p["t0"], p["skipped0"] = a, self.skipped
            p["cpu0"] = time.process_time()
        p["n"] += 1
        p["push_s"] += b - a
        p["push_max"] = max(p["push_max"], b - a)
        age = b - t
        p["age_sum"] += age
        p["age_max"] = max(p["age_max"], age)
        if b - p["t0"] >= 1.0:
            dt = b - p["t0"]
            cpu = time.process_time()
            row = dict(t=round(b, 3), fps_handled=round(p["n"] / dt, 1), dropped=self.skipped - p["skipped0"],
                       ms_mean=round(1000 * p["push_s"] / p["n"], 1), ms_max=round(1000 * p["push_max"], 1),
                       age_mean_s=round(p["age_sum"] / p["n"], 3), age_max_s=round(p["age_max"], 3),
                       wait_s=0.0, source=(getattr(tr, "source", None) or "") if tr is not None else "")
            extra = dict(proc_cpu_pct=round(100.0 * (cpu - p["cpu0"]) / dt, 1), decode_dropped=self.dropped_dec)
            self.perf = dict(t0=b, n=0, push_s=0.0, push_max=0.0, age_max=0.0, age_sum=0.0, skipped0=self.skipped,
                             cpu0=cpu)
            self.send(("perf", row, extra))

    def run(self):
        self.setup()
        th_c = threading.Thread(target=self.cmd_reader, daemon=True, name="commands")
        th_c.start()
        try:
            src = make_source(self.cfg["source"], log=self.log, lockstep=self.lockstep)
        except Exception as e:
            self.send(("fail", "picture source: %s" % e))
            return 2
        self.src = src
        th_r = threading.Thread(target=self.reader, args=(src,), daemon=True, name="reader")
        th_d = threading.Thread(target=self.decoder, daemon=True, name="decoder")
        th_r.start()
        th_d.start()
        try:
            self.track_loop()
        finally:
            self.stopped = True
            try:
                src.close()
            except Exception:
                pass
            if self.saver is not None:
                try:
                    self.saver.close()
                    self.send(("saver", self.saver.summary()))
                except Exception:
                    pass
            self.send(("end", self.err))
            if self.ring is not None:
                time.sleep(0.2)
                self.ring.close()
        return 0


def child_main():
    inp = sys.stdin.buffer
    out = sys.stdout.buffer
    sys.stdout = sys.stderr                        # prints go to the log, never into the message pipe
    cfg = recv_msg(inp)
    try:
        return Child(cfg, out, inp).run()
    except Exception:
        try:
            send_msg(out, ("fail", traceback.format_exc()))
        except Exception:
            pass
        return 1


# =============================================================================== the runner's side
class _Clock:
    def __init__(self):
        self.offset = None


class _Lk:
    """What the runner reads of the lock-on tracker (score, locked)."""

    def __init__(self):
        self.score = 0.0
        self.locked = False


class _NoLock:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class _HandMoved(set):
    """tracker.hand_moved: a set the runner reads and discards from; a discard is sent to the tracker process."""

    def __init__(self, owner):
        super().__init__()
        self._owner = owner

    def discard(self, x):
        super().discard(x)
        self._owner._cmd("hand_moved_discard", x)


class ProcTracker:
    """The runner's handle on the tracker in the tracker process: the methods and fields e02.py, e02_view.py,
    e02_bricks.py and e02_dance.py use of e02_camera.MotionTracker."""

    FORWARD_ATTRS = ("use_lock", "in_hold", "px_shift", "frozen", "WATCH_PARKED", "MOVE_SPOT", "PARK_EVERY_N")

    def __init__(self, proc, cam, centre, use_lock=True):
        d = self.__dict__
        d["_proc"] = proc
        d["cam"] = cam
        d["z"] = 3.7
        import e02_camera as CAM
        d["_CAM"] = CAM
        d["z"] = CAM.ROBOT_TOP_CM
        d["rpx"] = CAM.ROBOT_CM * cam.px_per_cm(centre[0], centre[1], CAM.ROBOT_TOP_CM)
        d["area"] = d["rpx"] ** 2
        d["lock"] = _NoLock()
        d["active"] = None
        d["pos"] = None
        d["last_seen"] = 0.0
        d["misses"] = 0
        d["source"] = None
        d["frame_t"] = None
        d["protect"] = {}
        d["hand_moved"] = _HandMoved(self)
        d["moved_by_hand"] = 0
        d["stats"] = dict(both=0, lock=0, detector=0, held=0, none=0, checks=0, agree=0, disagree=0, relocks=0)
        d["stats_hold"] = dict(both=0, lock=0, detector=0, held=0, none=0)
        d["in_hold"] = False
        d["use_lock"] = use_lock
        d["px_shift"] = (0.0, 0.0)
        d["frozen"] = False
        d["last_cands"] = []
        d["hint"] = None
        d["lk"] = _Lk()
        d["conf"] = {}
        d["parked"] = {}
        d["board_rect"] = None
        d["walls"] = []
        d["offset"] = 0.0
        d["thr"] = 16.0
        d["n"] = 0
        d["lock_off"] = (0.0, 0.0)
        d["_sent"] = 0                    # the latest command's number
        d["_mlock"] = __import__("threading").RLock()
        d["on_park_event"] = None         # function(t, robot, kind, dict) for each parked-robot event

    # --- forwarding ---------------------------------------------------------------------------------------------
    def __setattr__(self, name, value):
        self.__dict__[name] = value
        if name in self.FORWARD_ATTRS:
            self._cmd("setattr", name, value)

    def _cmd(self, op, *a, **k):
        return self._proc.command(self, op, *a, **k)

    def _ask(self, op, *a, **k):
        return self._proc.call(op, *a, **k)

    def _update(self, d):
        """A tracked picture's state from the process (the dispatch thread)."""
        with self._mlock:
            fresh = d["cmd"] >= self._sent
            u = self.__dict__
            if fresh:
                u["active"], u["pos"], u["last_seen"], u["misses"] = d["active"], d["pos"], d["last_seen"], d["misses"]
                u["protect"] = d["protect"]
                u["hint"] = d["hint"]
                hm = u["hand_moved"]
                set.clear(hm)
                set.update(hm, d["hand_moved"])
            u["source"], u["frame_t"] = d["source"], d["frame_t"]
            u["moved_by_hand"] = d["moved_by_hand"]
            u["stats"], u["stats_hold"] = d["stats"], d["stats_hold"]
            u["last_cands"] = d["last_cands"]
            u["conf"] = d["conf"]
            u["lk"].score = d["conf"].get("score") or 0.0
            u["lk"].locked = bool(d["conf"].get("locked"))
            u["offset"], u["thr"], u["n"], u["lock_off"] = d["offset"], d["thr"], d["tn"], d["lock_off"]
            u["parked"] = d.get("parked") or {}
            return fresh

    # --- what MotionTracker offers ------------------------------------------------------------------------------
    def where(self):
        return self.pos

    def confidence(self):
        c = dict(self.conf)
        c["stats"] = dict(self.stats)
        return c

    def parked_state(self):
        return dict(self.parked)

    def _px(self, x, y):
        u, v = self.cam.project((x, y, self.z))
        du, dv = self.px_shift
        return u + du, v + dv

    def _floor(self, u, v):
        du, dv = self.px_shift
        return self.cam.to_floor(u - du, v - dv, self.z)

    def follow(self, robot, x, y, t=None):
        with self._mlock:
            t = time.time() if t is None else t
            u = self.__dict__
            u["active"], u["pos"], u["last_seen"], u["misses"], u["hint"] = robot, (t, x, y), t, 0, None
            u["protect"] = dict(self.protect, **{robot: (x, y)})
            self._cmd("follow", robot, x, y, t)

    def park(self, robot, x, y):
        with self._mlock:
            self.__dict__["protect"] = dict(self.protect, **{robot: (x, y)})
            self._cmd("park", robot, x, y)

    def idle(self):
        with self._mlock:
            u = self.__dict__
            u["active"], u["pos"], u["hint"] = None, None, None
            self._cmd("idle")

    def forget(self, robot):
        with self._mlock:
            u = self.__dict__
            p = dict(self.protect)
            p.pop(robot, None)
            u["protect"] = p
            if self.active == robot:
                u["active"], u["pos"], u["hint"] = None, None, None
            self._cmd("forget", robot)

    def clear_protect(self):
        with self._mlock:
            self.__dict__["protect"] = {}
            self._cmd("clear_protect")

    def hold(self, radius_cm=6.0):
        with self._mlock:
            self._cmd("hold", radius_cm)

    def expect(self, x, y, radius_cm, for_s=10.0, kind=None):
        with self._mlock:
            self.__dict__["hint"] = (x, y, radius_cm, time.time() + for_s)
            self._cmd("expect", x, y, radius_cm, for_s, kind)

    def clear_expect(self):
        with self._mlock:
            self.__dict__["hint"] = None
            self._cmd("clear_expect")

    def robot_at(self, x, y, scale=1.2):
        self._cmd("robot_at", x, y, scale)

    def refresh_spot(self, x, y, scale=1.0):
        self._cmd("refresh_spot", x, y, scale)

    def set_area(self, board_rect, walls=()):
        self.__dict__["board_rect"] = tuple(board_rect)
        self.__dict__["walls"] = list(walls)
        self._cmd("set_area", tuple(board_rect), list(walls))

    def set_background(self, frames, robots=None):
        """frames: pictures (the runner's copies). The ones still in the shared ring go by number (no copy through
        the pipe); others (rare) are sent."""
        nums = self._proc.numbers_of(frames)
        if nums is not None:
            self._ask("set_background_from", nums, list(robots or []))
        else:
            self._ask("set_background", list(frames), robots=list(robots or []))

    def candidates(self, g=None, motion=None):
        return self._ask("candidates")

    def blobs(self, g=None):
        return [(c["u"], c["v"], c["area"], c["x"], c["y"]) for c in self.candidates() if c["shape"]]

    def on_area(self, x, y):
        if self.board_rect is None:
            return True
        x0, y0, x1, y1 = self.board_rect
        m = 8.0
        return min(x0, x1) - m <= x <= max(x0, x1) + m and min(y0, y1) - m <= y <= max(y0, y1) + m

    def floor_snapshot(self):
        return self._ask("floor_snapshot")

    @property
    def bg(self):
        s = self.floor_snapshot()
        return None if s is None else s["bg"]

    @property
    def known(self):
        s = self.floor_snapshot()
        return None if s is None else s["known"]

    def push(self, img, t):
        raise RuntimeError("the tracker runs in the tracker process; pictures go to it, not to push()")


class _ProcHandle:
    """The tracker process: start, messages, commands and questions."""

    def __init__(self, cfg, log=print, on_msg=None, python=None):
        self.cfg = cfg
        self.log = log
        self.on_msg = on_msg
        self.python = python or sys.executable
        self.p = None
        self.ring = None
        self.ready = threading.Event()
        self.fail = None
        self.wlock = threading.Lock()
        self.calls = {}
        self.next_id = 0
        self.id_lock = threading.Lock()
        self.reader = None
        self.ended = None
        self.info = {}
        self.stderr_path = None

    def start(self, timeout=START_TIMEOUT_S):
        env = dict(os.environ)
        th = str(int(self.cfg.get("cv_threads", CV_THREADS)))
        for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
            env.setdefault(k, th)
        d = self.cfg.get("session_dir")
        err = subprocess.DEVNULL
        if d:
            os.makedirs(d, exist_ok=True)
            self.stderr_path = os.path.join(d, "tracker_process.log")
            err = open(self.stderr_path, "ab")
        self.p = subprocess.Popen([self.python, "-u", os.path.abspath(__file__), "--child"], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=err, env=env, bufsize=0)
        if err is not subprocess.DEVNULL:
            err.close()
        cores = self.cfg.get("cores")
        if cores:
            try:
                n = os.cpu_count() or 1
                want = {int(c) for c in cores if int(c) < n}
                if want:
                    os.sched_setaffinity(self.p.pid, want)
            except (OSError, AttributeError):
                pass
        send_msg(self.p.stdin, self.cfg, self.wlock)
        self.reader = threading.Thread(target=self._read, daemon=True, name="tracker-process-pipe")
        self.reader.start()
        if not self.ready.wait(timeout):
            raise RuntimeError(self.fail or "the tracker process did not start within %.0f s" % timeout)
        if self.fail:
            raise RuntimeError(self.fail)
        return self

    def _read(self):
        import e02_shmframes as SF
        try:
            while True:
                m = recv_msg(self.p.stdout)
                k = m[0]
                if k == "ready":
                    self.info = m[1]
                    self.ring = SF.FrameRing.attach(m[1]["ring"])
                    self.ready.set()
                elif k == "fail":
                    self.fail = m[1]
                    self.ready.set()
                elif k == "rep":
                    ev = self.calls.get(m[1])
                    if ev is not None:
                        ev[1] = (m[2], m[3])
                        ev[0].set()
                elif k == "log":
                    self.log("(tracker process: %s)" % m[1])
                elif k == "end":
                    self.ended = m[1] or "ended"
                if self.on_msg is not None and k in ("frm", "trk", "perf", "saver"):
                    try:
                        self.on_msg(m)
                    except Exception as e:
                        self.log("(tracker process: a message failed in the runner: %s)" % e)
        except (EOFError, OSError, pickle.UnpicklingError, ValueError) as e:
            if self.ended is None:
                self.ended = "pipe closed (%s)" % e
        finally:
            self.ready.set()
            for ev in list(self.calls.values()):
                ev[0].set()

    def alive(self):
        return self.p is not None and self.p.poll() is None and self.ended is None

    def _id(self):
        with self.id_lock:
            self.next_id += 1
            return self.next_id

    def command(self, op, *a, **k):
        i = self._id()
        try:
            send_msg(self.p.stdin, ("c", i, op, a, k), self.wlock)
        except (BrokenPipeError, OSError, ValueError, AttributeError):
            pass
        return i

    def call(self, op, *a, timeout=CALL_TIMEOUT_S, **k):
        i = self._id()
        slot = [threading.Event(), None]
        self.calls[i] = slot
        try:
            send_msg(self.p.stdin, ("q", i, op, a, k), self.wlock)
            if not slot[0].wait(timeout) or slot[1] is None:
                raise RuntimeError("the tracker process did not answer %s (%s)" % (op, self.ended or "timeout"))
            ok, res = slot[1]
            if not ok:
                raise RuntimeError("tracker process, %s: %s" % (op, res))
            return res
        finally:
            self.calls.pop(i, None)

    def close(self, timeout=5.0):
        try:
            send_msg(self.p.stdin, ("stop",), self.wlock)
            self.p.stdin.close()
        except Exception:
            pass
        try:
            self.p.wait(timeout)
        except Exception:
            try:
                self.p.kill()
            except Exception:
                pass
        if self.reader is not None:
            self.reader.join(2.0)
        if self.ring is not None:
            self.ring.close()


class ProcFeed:
    """The runner's camera feed with the tracker in its own process: what e02.py and e02_view.py use of
    e02_camera.Feed. source: 'stream:<url>' (the recorder's /live), 'device:0' (the camera itself: production) or
    'clip:<path>[@speed]' (a replay)."""

    def __init__(self, source, history, log=print, session_dir=None, cores=None, cv_threads=CV_THREADS,
                 capture_delay_s=CAPTURE_DELAY_S, save_frames=False, frames_dir=None, slots=None, lockstep=False,
                 python=None):
        self.source = source
        self.history = history
        self.log = log
        self.cfg = dict(source=source, session_dir=session_dir, cores=list(cores) if cores else None,
                        cv_threads=cv_threads, capture_delay_s=capture_delay_s, keep_s=0.0, save_frames=save_frames,
                        frames_dir=frames_dir or (os.path.join(session_dir, "frames") if session_dir else None),
                        lockstep=lockstep)
        if slots:
            self.cfg["slots"] = slots
        self.python = python
        self.proc = None
        self._tracker = None
        self._tracker_on = True
        self.clock = _Clock()
        self.cap = capture_delay_s
        self.frames = 0
        self.last_t = None
        self.last_n = -1
        self.cond = threading.Condition()
        self.hooks = []
        self.on_sample = None
        self.sample_filter = None
        self.history_raw = None
        self.on_perf = None
        self.on_perf_extra = None          # function(dict): the process's own CPU share and decode drops, once a second
        self.on_park_event = None          # function(t, robot, kind, dict)
        self._keep_s = 0.0
        self.err = None
        self.stopped = False
        self.skipped = 0
        self.lag = 0.0
        self.ident = None
        self.n_trk = 0
        self.dropped_stale = 0             # tracked positions not taken (from before the runner's latest command)
        self.saver_summary = None
        self.make_args = None
        self.area_args = None
        self.q = queue.Queue()
        self.q_warned = False
        self.disp = None
        self.hook_th = None
        self.hook_cond = threading.Condition()
        self.hook_next = None

    # --- life ---------------------------------------------------------------------------------------------------
    def start(self):
        if self.proc is not None and self.proc.alive():
            return self                                   # already running (make_feed starts it)
        self.proc = _ProcHandle(self.cfg, log=self.log, on_msg=self._on_msg, python=self.python)
        self.proc.start()
        if self._keep_s:
            self.proc.command("keep_s", self._keep_s)
        return self

    def is_alive(self):
        return self.proc is not None and self.proc.alive() and not self.stopped

    @property
    def pid(self):
        return None if self.proc is None or self.proc.p is None else self.proc.p.pid

    def close(self):
        self.stopped = True
        if self.proc is not None:
            self.proc.close()
        self.q.put(None)
        if self.disp is not None:
            self.disp.join(5.0)
        with self.hook_cond:
            self.hook_cond.notify_all()

    def restart(self):
        """A new tracker process after the old one died: same source, same tracker set-up (calibration, area, lock
        choice); the floor memory is taken again from fresh pictures with every known robot where it was last seen,
        and the followed robot is followed again from its last place."""
        old_tr = self._tracker
        state = None
        if old_tr is not None:
            state = dict(active=old_tr.active, pos=old_tr.pos, protect=dict(old_tr.protect))
        try:
            if self.proc is not None:
                self.proc.close(timeout=1.0)
        except Exception:
            pass
        self.stopped = False
        self.start()
        if self.make_args is not None and old_tr is not None:
            cam, centre, use_lock = self.make_args
            self.proc.command("make", cam, centre, use_lock)
            if self.area_args is not None:
                self.proc.command("set_area", *self.area_args)
            old_tr.__dict__["_proc"] = self
            frames = self.wait_frames(10, timeout=10)
            robots = list(state["protect"].values()) if state else []
            if frames:
                old_tr.set_background(frames, robots=robots)
            for r, q in (state["protect"] if state else {}).items():
                old_tr.park(r, q[0], q[1])
            if state and state["active"] and state["pos"]:
                old_tr.follow(state["active"], state["pos"][1], state["pos"][2])
        return self

    # --- the tracker ----------------------------------------------------------------------------------------------
    def make_tracker(self, cam, centre, use_lock=True):
        """A new tracker in the process (as e02_camera.MotionTracker(cam, centre)); it is the one tracking now."""
        self.make_args = (cam, centre, use_lock)
        self.area_args = None
        tr = ProcTracker(self, cam, centre, use_lock)
        self._tracker = tr
        self._tracker_on = True
        self.proc.command("make", cam, centre, use_lock)
        self.proc.command("tracking", True)
        return tr

    def _get_tracker(self):
        return self._tracker if self._tracker_on else None

    def _set_tracker(self, tr):
        """None: the process stops tracking (the dance's own tracker works meanwhile); the tracker back: it goes on.
        Any other tracker object is refused (it would run in the runner)."""
        if tr is None:
            self._tracker_on = False
            self.proc.command("tracking", False)
        elif tr is self._tracker:
            self._tracker_on = True
            self.proc.command("tracking", True)
        else:
            raise ValueError("ProcFeed tracks with its own process; use make_tracker()")

    tracker = property(_get_tracker, _set_tracker)   # (one name each: tests/test_e02_oct22 no_method_is_defined_twice)

    def command(self, tr, op, *a, **k):
        """A command from tracker tr (only the current tracker's commands go through)."""
        if tr is not self._tracker or self.proc is None:
            return None
        if op == "set_area":
            self.area_args = a
        i = self.proc.command(op, *a, **k)
        tr.__dict__["_sent"] = i
        return i

    def call(self, op, *a, **k):
        return self.proc.call(op, *a, **k)

    def numbers_of(self, frames):
        """The ring numbers of these pictures (the runner's copies, from last_img / wait_frames), if every one is still
        in the ring unchanged (checked picture by picture); else None (they are then sent through the pipe)."""
        import numpy as np
        nums = []
        ids = getattr(self, "_ids", {})
        for f in frames:
            n = ids.get(id(f))
            if n is None:
                return None
            r = self.proc.ring.read(n) if self.proc.ring is not None else None
            if r is None or r[0].shape != f.shape or not np.array_equal(r[0], f):
                return None
            nums.append(n)
        return nums

    # --- messages (the pipe thread) -------------------------------------------------------------------------------
    def _on_msg(self, m):
        """The pipe's thread: picture numbers at once (cheap); positions and the rest to the dispatch thread, so a
        slow runner never stops the pipe (a full pipe would stop the tracker process)."""
        k = m[0]
        if k == "frm":
            _, n, t, pf, t_pi, off = m
            self.clock.offset = off
            with self.cond:
                self.frames += 1
                self.last_t = t
                self.last_n = n
                self.cond.notify_all()
            if self.hooks:
                with self.hook_cond:
                    self.hook_next = (n, t)
                    self.hook_cond.notify()
                self._start_threads()
            return
        self._start_threads()
        self.q.put(m)
        if self.q.qsize() > 250 and not self.q_warned:
            self.q_warned = True
            self.log("(the runner is behind the tracker process by %d messages)" % self.q.qsize())

    def _start_threads(self):
        if self.disp is None:
            self.disp = threading.Thread(target=self._dispatch, daemon=True, name="tracker-process-dispatch")
            self.disp.start()
        if self.hook_th is None and self.hooks:
            self.hook_th = threading.Thread(target=self._run_hooks, daemon=True, name="tracker-process-hooks")
            self.hook_th.start()

    def _run_hooks(self):
        """The camera hooks (naming check, camera shift watch, the dance's tracker): the newest picture each time
        they are ready for one, as the old feed gave them the pictures its tracker took."""
        while not self.stopped:
            with self.hook_cond:
                while self.hook_next is None and not self.stopped:
                    self.hook_cond.wait(0.5)
                if self.stopped:
                    return
                n, t = self.hook_next
                self.hook_next = None
            hooks = list(self.hooks)
            if not hooks:
                continue
            r = self.proc.ring.read(n) if (self.proc is not None and self.proc.ring is not None) else None
            if r is None:
                continue
            for h in hooks:
                try:
                    h(r[0], t)
                except Exception as e:
                    self.log("(camera hook failed: %s)" % e)

    def _dispatch(self):
        self.ident = threading.get_ident()
        while True:
            m = self.q.get()
            if m is None:
                return
            try:
                self._handle(m)
            except Exception as e:
                self.log("(tracker process message failed in the runner: %s)" % e)

    def _handle(self, m):
        k = m[0]
        if k == "trk":
            d = m[1]
            tr = self._tracker
            if tr is None:
                return
            self.n_trk += 1
            self.lag = max(0.0, time.time() - d["t"])
            fresh = tr._update(d)
            for ev in d.get("park_events") or []:
                cb = self.on_park_event or tr.on_park_event
                if cb is not None:
                    try:
                        cb(*ev)
                    except Exception as e:
                        self.log("(parked-robot event failed in the runner: %s)" % e)
            s = d["s"]
            if s is None:
                return
            if not fresh:
                self.dropped_stale += 1
                return
            if self.history_raw is not None:
                self.history_raw.add(s)
            if self.sample_filter is not None:
                s = self.sample_filter(s)
            if s is not None:
                self.history.add(s)
                if self.on_sample:
                    self.on_sample(s)
        elif k == "perf":
            row, extra = m[1], m[2]
            self.skipped = self.skipped + int(row.get("dropped") or 0)
            if self.on_perf:
                self.on_perf(row)
            if self.on_perf_extra:
                self.on_perf_extra(dict(extra, t=row["t"]))
        elif k == "saver":
            self.saver_summary = m[1]

    # --- pictures -----------------------------------------------------------------------------------------------
    def _read(self, n):
        r = self.proc.ring.read(n) if (self.proc is not None and self.proc.ring is not None) else None
        if r is None:
            return None
        img = r[0]
        if not hasattr(self, "_ids"):
            self._ids = {}
        if len(self._ids) > 4096:
            self._ids.clear()
        self._ids[id(img)] = n
        return img

    @property
    def last_img(self):
        if self.last_n < 0:
            return None
        for n in (self.last_n, self.last_n - 1):
            img = self._read(n)
            if img is not None:
                return img
        return None

    @property
    def recent(self):
        if self.proc is None or self.proc.ring is None or self.last_n < 0:
            return deque()
        return deque((i for i in (self._read(n) for n in range(max(0, self.last_n - 29), self.last_n + 1))
                      if i is not None), maxlen=30)

    def wait_frames(self, n, timeout=10.0):
        """Wait for n new pictures; returns them (grey) or fewer on timeout."""
        nums = []
        end = time.time() + timeout
        with self.cond:
            seen = self.last_n
            while len(nums) < n and time.time() < end:
                self.cond.wait(0.2)
                if self.last_n > seen:
                    nums.append(self.last_n)
                    seen = self.last_n
        out = [self._read(k) for k in nums]
        return [i for i in out if i is not None]

    def collect(self, seconds):
        """Every picture for the next `seconds`: (grey pictures, their times)."""
        with self.cond:
            n0 = self.last_n
        time.sleep(seconds)
        with self.cond:
            n1 = self.last_n
        imgs, ts = [], []
        for k in range(n0 + 1, n1 + 1):
            r = self.proc.ring.read(k) if self.proc.ring is not None else None
            if r is not None:
                imgs.append(r[0])
                ts.append(r[1][0])
        return imgs, ts

    def _get_keep_s(self):
        return self._keep_s

    def _set_keep_s(self, v):
        self._keep_s = float(v)
        if self.proc is not None:
            self.proc.command("keep_s", self._keep_s)

    keep_s = property(_get_keep_s, _set_keep_s)

    def kept(self, t_from, t_to=None):
        try:
            return self.proc.call("kept", t_from, t_to)
        except Exception:
            return []

    def frame_at(self, t, within=0.3):
        import cv2
        import numpy as np
        try:
            q = self.proc.call("frame_at", t, within)
        except Exception:
            return None
        if q is None:
            return None
        return cv2.imdecode(np.frombuffer(q[1], np.uint8), cv2.IMREAD_UNCHANGED)

    # --- frame saving -------------------------------------------------------------------------------------------
    def event(self, kind, text="", t=None, robot=""):
        """Ask the tracker process to keep this moment (still + clip), if it saves frames."""
        if self.proc is None or not self.cfg.get("save_frames"):
            return None
        return self.proc.command("event", kind, text, t, robot)

    def frames_summary(self, timeout=3.0):
        try:
            return self.proc.call("saver_summary", timeout=timeout)
        except Exception:
            return self.saver_summary


if __name__ == "__main__" and "--child" in sys.argv:
    _code = child_main()
    try:
        sys.stderr.flush()
    except Exception:
        pass
    os._exit(_code or 0)          # (a daemon thread still reading the pipe must not hang the interpreter's shutdown)
