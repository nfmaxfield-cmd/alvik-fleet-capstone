"""Planned load during a session, and the runner watching its own speed (Nick, 2026-10-25).

Nick: "we can use uploads later to throttle speed and test latency improvements ... a great opportunity to showcase
adaptability to real world changes", and "how will the system live track its own processing speed to identify and
react when it slows?" (2026-10-24 19:57-20:15: an upload during Right's session halved the runner's pictures a second).

Two parts:

1. Load (e02.py --load KIND, --load-on-s, --load-off-s): a background load switched on and off in blocks from the first
   run on, each switch marked in the log and on the video (load.csv, e02_state.json "load_blocks"), so a session
   compares the same system with and without it. KIND:
     cpu[:N]    N busy processes (default 1; the Pi has 4 cores, the recorder and runner use about 2)
     disk:DIR   read the files under DIR and compress them, over and over (what an upload does on the Pi: reads the
                USB drive and works on the data), e.g. the recordings folder
     cmd:COMMAND  any command, started at each "on" and stopped at each "off" (a real upload, e.g.
                python3 tools/pi/upload_session.py ...)
2. Speed watch (SpeedWatch): every perf row (once a second: pictures handled, picture age) against the session's own
   baseline, learned from its first BASE_ROWS seconds with no load. Slow = for SLOW_HOLD_S seconds in a row, the
   pictures handled a second below SLOW_FPS_FRAC x baseline, or the picture age above SLOW_AGE_FRAC x baseline.
   The runner says so, marks the video, and records each slow span (start, end, worst values, load on or off).
   Reacting to it (moving the border in by speed x picture age, or driving slower) is the next step, to be designed
   from what these sessions show.

Pure Python (multiprocessing, subprocess); tests: tests/test_e02_load.py."""
import gzip
import multiprocessing as mp
import os
import shlex
import signal
import statistics
import subprocess
import threading
import time

BASE_ROWS = 120          # chosen: the baseline is the first 2 minutes of perf rows with no load (the setup's start)
SLOW_FPS_FRAC = 0.75     # chosen (DOE note: 2026-10-24's upload took 19-21 down to 11-14 = 0.55-0.7 x; a load test
SLOW_AGE_FRAC = 1.5      # at two strengths would set both); age: 0.11 s -> 0.13-0.17 s was 1.2-1.5 x
SLOW_HOLD_S = 5          # chosen: this many seconds in a row before "slow" (one slow second is a hiccup)


def _busy(stop):
    x = 0
    while not stop.is_set():
        for _ in range(20000):
            x = (x * 1103515245 + 12345) & 0x7FFFFFFF


def _disk(stop, folder):
    while not stop.is_set():
        files = []
        for root, _d, fs in os.walk(folder):
            files += [os.path.join(root, f) for f in fs]
        if not files:
            time.sleep(1.0)
            continue
        for p in files:
            if stop.is_set():
                return
            try:
                with open(p, "rb") as f:
                    while not stop.is_set():
                        b = f.read(1 << 20)
                        if not b:
                            break
                        gzip.compress(b, compresslevel=6)
            except OSError:
                continue


def parse(spec):
    """'cpu', 'cpu:2', 'disk:/path', 'cmd:...' -> (kind, arg). Raises ValueError for anything else."""
    kind, _, arg = (spec or "").partition(":")
    kind = kind.strip().lower()
    if kind == "cpu":
        n = int(arg) if arg.strip() else 1
        if not 1 <= n <= 8:
            raise ValueError("cpu:N needs N from 1 to 8")
        return "cpu", n
    if kind == "disk":
        if not arg.strip():
            raise ValueError("disk:DIR needs a folder")
        return "disk", arg.strip()
    if kind == "cmd":
        if not arg.strip():
            raise ValueError("cmd:COMMAND needs a command")
        return "cmd", arg.strip()
    raise ValueError("--load: cpu[:N], disk:DIR or cmd:COMMAND")


class Load:
    """Switches the load on and off: on_s seconds on, off_s off, from start() until stop(). log(text) and
    mark(text) are the runner's; record(row) gets a dict per switch."""

    def __init__(self, spec, on_s=300.0, off_s=300.0, log=print, mark=None, record=None):
        self.kind, self.arg = parse(spec)
        self.spec = spec
        self.on_s, self.off_s = float(on_s), float(off_s)
        self.log, self.mark, self.record = log, mark or (lambda t: None), record or (lambda r: None)
        self.on = False
        self._procs = []
        self._cmd = None
        self._ev = None
        self._quit = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def _loop(self):
        while not self._quit.is_set():
            self._switch(True)
            if self._quit.wait(self.on_s):
                break
            self._switch(False)
            if self._quit.wait(self.off_s):
                break
        self._switch(False)

    def _switch(self, on):
        if on == self.on:
            return
        if on:
            if self.kind == "cpu":
                self._ev = mp.Event()
                self._procs = [mp.Process(target=_busy, args=(self._ev,), daemon=True) for _ in range(self.arg)]
            elif self.kind == "disk":
                self._ev = mp.Event()
                self._procs = [mp.Process(target=_disk, args=(self._ev, self.arg), daemon=True)]
            for p in self._procs:
                p.start()
            if self.kind == "cmd":
                self._cmd = subprocess.Popen(shlex.split(self.arg), stdout=subprocess.DEVNULL,
                                             stderr=subprocess.DEVNULL, start_new_session=True)
        else:
            if self._ev is not None:
                self._ev.set()
            for p in self._procs:
                p.join(timeout=3.0)
                if p.is_alive():
                    p.terminate()
            self._procs, self._ev = [], None
            if self._cmd is not None and self._cmd.poll() is None:
                try:
                    os.killpg(self._cmd.pid, signal.SIGTERM)
                    self._cmd.wait(timeout=5.0)
                except Exception:
                    try:
                        self._cmd.kill()
                    except Exception:
                        pass
            self._cmd = None
        self.on = on
        words = "load %s: %s" % ("ON" if on else "off", self.spec)
        self.log("  " + words)
        self.mark("e02 " + words)
        self.record(dict(t=round(time.time(), 1), on=on, spec=self.spec))

    def stop(self):
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout=10.0)
        self._switch(False)


class SpeedWatch:
    """feed(row) once a second with a perf row (fps_handled, age_mean_s); load_on() says whether the planned load is
    on. Returns 'slow' / 'normal' when the state changes, else None. spans: [dict(t0, t1, fps_min, age_max, load)]."""

    def __init__(self, base_rows=BASE_ROWS, fps_frac=SLOW_FPS_FRAC, age_frac=SLOW_AGE_FRAC, hold_s=SLOW_HOLD_S):
        self.base_rows, self.fps_frac, self.age_frac, self.hold_s = base_rows, fps_frac, age_frac, hold_s
        self._base = []
        self.base_fps = self.base_age = None
        self.slow = False
        self._run = 0
        self._ok = 0
        self.spans = []

    def is_slow_row(self, row):
        if self.base_fps is None:
            return False
        return (row["fps_handled"] < self.fps_frac * self.base_fps or
                row["age_mean_s"] > self.age_frac * self.base_age)

    def feed(self, row, load_on=False):
        if self.base_fps is None:
            if not load_on:
                self._base.append(row)
            if len(self._base) >= self.base_rows:
                self.base_fps = statistics.median(r["fps_handled"] for r in self._base)
                self.base_age = max(0.01, statistics.median(r["age_mean_s"] for r in self._base))
            return None
        bad = self.is_slow_row(row)
        if not self.slow:
            self._run = self._run + 1 if bad else 0
            if self._run >= self.hold_s:
                self.slow, self._ok = True, 0
                self.spans.append(dict(t0=row["t"], t1=None, fps_min=row["fps_handled"], age_max=row["age_mean_s"],
                                       load=bool(load_on)))
                return "slow"
            return None
        sp = self.spans[-1]
        sp["fps_min"] = min(sp["fps_min"], row["fps_handled"])
        sp["age_max"] = max(sp["age_max"], row["age_mean_s"])
        sp["load"] = sp["load"] or bool(load_on)
        self._ok = 0 if bad else self._ok + 1
        if self._ok >= self.hold_s:
            self.slow, self._run = False, 0
            sp["t1"] = row["t"]
            return "normal"
        return None
