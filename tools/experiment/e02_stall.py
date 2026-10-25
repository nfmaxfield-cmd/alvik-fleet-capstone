"""What blocks the camera loop (2026-10-25 runner fixes, part 2a): a watch on the runner's own threads.

The camera loop is a thread of the runner's process, so a long calculation in any other thread of it takes turns
with the camera for Python's interpreter lock (session-1510 report Q5: explore's fit). perf.csv shows that the
camera fell behind, not why. This watch says why, every session, at a cost well under a millisecond a tick:

- every TICK_S it wakes; a wake STALL_S or more late means no Python thread of the runner could run for that long
  (the camera loop included). The stall goes in stalls.csv with what every thread was running at that moment
  (the innermost frame in the runner's own files, and the innermost frame of all): the camera thread itself (the
  tracker), or another thread holding the lock (a fit, a picture drawn, a file written).
- every PROFILE_EVERY ticks it notes what each thread is running (thread_profile in summary(): where each
  thread spends its time, by function, counted in ticks), and once a second each thread's CPU time from
  /proc/self/task (Linux; threads.csv): which thread eats the CPU when the pictures a second drop, and a row
  "wake_late_ms": how late this watch's own wakes were that second on average. A long calculation in pure Python
  hands the interpreter lock round every 5 ms, so it shows here (each wake waits up to 5 ms; the camera loop, which
  lets go of the lock in every OpenCV call, waits that long after each one), not as a stall.

Tests: tests/test_e02_runner_fixes.py."""
import collections
import csv
import os
import sys
import threading
import time

TICK_S = 0.02          # chosen: 50 wakes a second (cost ~0.1 ms each). No DOE: resolution of the measurement
STALL_S = 0.10         # 100 ms: the build's definition of "blocks the camera loop" (Nick's brief, 2026-10-25)
PROFILE_EVERY = 5      # chosen: a profile sample every 0.1 s. No DOE: resolution of the measurement


def _where(frame, ours_only):
    f = frame
    while f is not None:
        fn = f.f_code.co_filename
        if not ours_only or os.path.basename(fn).startswith(("e02", "sources", "cameras", "tracker")):
            return "%s:%s:%d" % (os.path.basename(fn), f.f_code.co_name, f.f_lineno)
        f = f.f_back
    return ""


class StallWatch(threading.Thread):
    def __init__(self, folder, camera_ident=None, log=None):
        super().__init__(daemon=True, name="stallwatch")
        self.folder = folder
        self.camera_ident = camera_ident            # function -> the camera thread's ident (it can be restarted)
        self.log = log or (lambda m: None)
        self.quit = threading.Event()
        self.stalls = []
        self.profile = collections.Counter()
        self.ticks = 0
        self._sf = self._sw = self._tf = self._tw = None
        self._cpu_prev = {}
        self._cpu_t = None
        self._late = [0.0, 0]

    def names(self):
        cam = self.camera_ident() if self.camera_ident else None
        out = {}
        for t in threading.enumerate():
            out[t.ident] = ("camera" if t.ident == cam else
                            "main" if t is threading.main_thread() else (t.name or "thread"))
        return out

    def run(self):
        try:
            self._loop()
        except Exception as e:          # a measurement: never take the runner down
            self.log("  (stall watch stopped: %s)" % e)

    def _loop(self):
        last = time.perf_counter()
        while not self.quit.is_set():
            time.sleep(TICK_S)
            now = time.perf_counter()
            late = now - last - TICK_S
            last = now
            self.ticks += 1
            self._late[0] += max(0.0, late)
            self._late[1] += 1
            if late >= STALL_S or self.ticks % PROFILE_EVERY == 0:
                frames = sys._current_frames()
                nm = self.names()
                me = threading.get_ident()
                seen = []
                for ident, fr in frames.items():
                    if ident == me:
                        continue
                    name = nm.get(ident, "thread")
                    ours, inner = _where(fr, True), _where(fr, False)
                    if inner.startswith(("threading.py:wait", "threading.py:_wait", "selectors.py", "socket.py",
                                         "queue.py:get")):
                        continue                    # waiting: not holding anything
                    seen.append((name, ours, inner))
                    if self.ticks % PROFILE_EVERY == 0:
                        self.profile[(name, ours)] += 1
                if late >= STALL_S:
                    self._stall(time.time(), late, seen)
            if self._cpu_t is None or now - self._cpu_t >= 1.0:
                self._cpu(now)

    def _stall(self, t, late, seen):
        row = [round(t, 3), round(1000.0 * late)] + ["%s | %s | %s" % s for s in seen]
        self.stalls.append(row)
        try:
            if self._sw is None:
                self._sf = open(os.path.join(self.folder, "stalls.csv"), "a", newline="")
                self._sw = csv.writer(self._sf)
                if self._sf.tell() == 0:
                    self._sw.writerow(["t", "stall_ms", "threads (name | innermost runner frame | innermost frame)"])
            self._sw.writerow(row)
            self._sf.flush()
        except OSError:
            pass

    def _cpu(self, now):
        """threads.csv: each thread's CPU seconds in the last second (Linux /proc)."""
        self._cpu_t = now
        base = "/proc/self/task"
        if not os.path.isdir(base):
            return
        byid = {getattr(t, "native_id", None): t for t in threading.enumerate()}
        nm = self.names()
        tick = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100
        rows = []
        if self._late[1]:
            rows.append([round(time.time(), 2), "wake_late_ms", round(1000.0 * self._late[0] / self._late[1], 2)])
        self._late = [0.0, 0]
        for tid in os.listdir(base):
            try:
                with open(os.path.join(base, tid, "stat")) as f:
                    parts = f.read().rsplit(")", 1)[1].split()
                cpu = (int(parts[11]) + int(parts[12])) / float(tick)
            except (OSError, IndexError, ValueError):
                continue
            prev = self._cpu_prev.get(tid)
            self._cpu_prev[tid] = cpu
            if prev is None:
                continue
            th = byid.get(int(tid))
            name = nm.get(th.ident, th.name) if th is not None else "native-%s" % tid
            if cpu - prev > 0:
                rows.append([round(time.time(), 2), name, round(cpu - prev, 3)])
        try:
            if self._tw is None:
                self._tf = open(os.path.join(self.folder, "threads.csv"), "a", newline="")
                self._tw = csv.writer(self._tf)
                if self._tf.tell() == 0:
                    self._tw.writerow(["t", "thread", "cpu_s"])
            self._tw.writerows(rows)
            self._tf.flush()
        except OSError:
            pass

    def summary(self, top=8):
        """dict: stalls (count, total and worst ms, the most common culprit frames) and each thread's profile."""
        culprits = collections.Counter()
        for r in self.stalls:
            for s in r[2:]:
                name, ours, _ = s.split(" | ")
                culprits["%s %s" % (name, ours)] += r[1]
        prof = collections.defaultdict(list)
        for (name, ours), k in self.profile.most_common():
            if len(prof[name]) < top:
                prof[name].append([ours, k])
        return dict(stalls=len(self.stalls), stall_ms_total=int(sum(r[1] for r in self.stalls)),
                    stall_ms_worst=int(max([r[1] for r in self.stalls] or [0])),
                    culprits_ms=culprits.most_common(top), ticks=self.ticks,
                    thread_profile={k: v for k, v in prof.items()})

    def stop(self):
        self.quit.set()
        for f in (self._sf, self._tf):
            try:
                if f:
                    f.close()
            except OSError:
                pass
