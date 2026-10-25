"""Heavy calculations out of the camera's process (2026-10-25 runner fixes, part 2a).

The runner's camera loop (e02_camera.Feed) is a thread in the runner's own Python process. Any long calculation in
another thread of that process takes turns with it for Python's one interpreter lock, and the camera loop falls
behind: on 2026-10-25 15:10 explore's two camera fits (EX.fit, 25-28 s each on the Pi) took the pictures handled a
second from 24 down to 3.8 and 5.4 (session-1510 report, Q5).

Worker: one helper process, started on first use and kept for the session, that runs a module-level function and
sends back its result. The runner's thread that asked waits on a pipe (which lets the camera loop run). The helper
runs at a lower priority (os.nice WORKER_NICE) with its maths libraries on one thread each, so on the Pi (4 cores:
camera recorder, runner, helper) it does not take the camera's core either. If the helper cannot start or dies, the
call runs in the runner as before, and the runner says so.

    W = Worker(log=print)
    sol = W.call(EX.fit, robots, size)          # same arguments and result as EX.fit(robots, size)

Pure Python (subprocess, pickle); tests: tests/test_e02_runner_fixes.py."""
import os
import pickle
import struct
import subprocess
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
WORKER_NICE = 10       # chosen: below the runner and the recorder (both 0). No DOE: any positive value gives the
                       # camera first call on a busy core; the before/after measurement is the check
CALL_TIMEOUT_S = 600.0  # chosen: 20x the Pi's longest fit (28 s); a helper that takes longer is stuck. No DOE


def _send(fh, obj):
    b = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    fh.write(struct.pack("<Q", len(b)))
    fh.write(b)
    fh.flush()


def _recv(fh):
    h = fh.read(8)
    if len(h) < 8:
        raise EOFError("the helper process closed")
    n = struct.unpack("<Q", h)[0]
    b = fh.read(n)
    if len(b) < n:
        raise EOFError("the helper process closed mid-answer")
    return pickle.loads(b)


class Worker:
    def __init__(self, log=None, paths=None):
        self.log = log or (lambda m: None)
        self.paths = list(paths if paths is not None else sys.path)
        self.p = None
        self.lock = threading.Lock()
        self.failed = None            # why the helper could not be used (then calls run in the runner)
        self.calls = []               # (function, seconds, where: 'helper' | 'runner')

    def start(self):
        if self.p is not None and self.p.poll() is None:
            return True
        if self.failed:
            return False
        env = dict(os.environ)
        for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
            env[k] = "1"
        env["E02_OFFLOAD_PATHS"] = os.pathsep.join(self.paths)
        try:
            self.p = subprocess.Popen([sys.executable, "-u", os.path.abspath(__file__), "--worker"],
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
            _send(self.p.stdin, ("ping",))
            if _recv(self.p.stdout) != "pong":
                raise RuntimeError("no answer")
            return True
        except Exception as e:
            self.failed = "the helper process did not start (%s)" % e
            self.log("  (heavy calculations: %s; they run in the runner, the camera may slow down)" % self.failed)
            self.p = None
            return False

    def call(self, fn, *args, **kw):
        """fn(*args, **kw) in the helper process (fn: a module-level function); in the runner if the helper is
        not available. Exceptions raised by fn are raised here."""
        t0 = time.time()
        with self.lock:
            if self.start():
                try:
                    _send(self.p.stdin, ("call", fn.__module__, fn.__name__, args, kw))
                    ok, res = self._wait()
                    self.calls.append((fn.__name__, round(time.time() - t0, 2), "helper"))
                    if ok:
                        return res
                    raise res
                except (EOFError, OSError, pickle.PicklingError, TimeoutError) as e:
                    self.failed = "the helper process failed (%s)" % e
                    self.log("  (heavy calculations: %s; this one and the rest run in the runner)" % self.failed)
                    self.stop()
        t0 = time.time()
        res = fn(*args, **kw)
        self.calls.append((fn.__name__, round(time.time() - t0, 2), "runner"))
        return res

    def _wait(self):
        out = {}

        def rd():
            try:
                out["v"] = _recv(self.p.stdout)
            except Exception as e:      # noqa: BLE001 (handed back below)
                out["e"] = e
        th = threading.Thread(target=rd, daemon=True)
        th.start()
        th.join(CALL_TIMEOUT_S)
        if th.is_alive():
            raise TimeoutError("no answer in %.0f s" % CALL_TIMEOUT_S)
        if "e" in out:
            raise out["e"] if isinstance(out["e"], (EOFError, OSError)) else EOFError(str(out["e"]))
        tag, val = out["v"]
        if tag == "ok":
            return True, val
        if isinstance(val, BaseException):
            return False, val
        return False, RuntimeError(str(val))

    def stop(self):
        p, self.p = self.p, None
        if p is None:
            return
        try:
            _send(p.stdin, ("quit",))
        except Exception:
            pass
        try:
            p.wait(timeout=3.0)
        except Exception:
            p.kill()


def _worker():
    try:
        os.nice(WORKER_NICE)
    except (OSError, AttributeError):
        pass
    for q in reversed((os.environ.get("E02_OFFLOAD_PATHS") or "").split(os.pathsep)):
        if q and q not in sys.path:
            sys.path.insert(0, q)
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import importlib
    inp, out = sys.stdin.buffer, sys.stdout.buffer
    sys.stdout = sys.stderr              # a print in a called function must not break the pipe
    while True:
        try:
            job = _recv(inp)
        except EOFError:
            return 0
        if job[0] == "quit":
            return 0
        if job[0] == "ping":
            _send(out, "pong")
            continue
        _, mod, name, args, kw = job
        try:
            res = getattr(importlib.import_module(mod), name)(*args, **kw)
            _send(out, ("ok", res))
        except Exception as e:           # noqa: BLE001 (the runner raises it)
            try:
                pickle.dumps(e)
                _send(out, ("err", e))
            except Exception:
                _send(out, ("err", "%s: %s\n%s" % (type(e).__name__, e, traceback.format_exc())))


if __name__ == "__main__" and "--worker" in sys.argv:
    sys.exit(_worker())
