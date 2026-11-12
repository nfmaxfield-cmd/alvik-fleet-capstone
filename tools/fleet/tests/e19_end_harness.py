"""(e19, audit H1/H2) Runs ONE simulated ("dry") free roam inside `unshare -rn` with its own HOME, records every command
sent to the robots and when each end step ran, and writes them as JSON. Used by test_e19_audit_fixes.py.

    unshare -rn python3 e19_end_harness.py OUT.json RUNDIR MODE [SECONDS]
MODE: sigterm-diskfull  SIGTERM SECONDS after the roam starts, and every end-of-run write fails with "No space left on device"
      timecap           the run's own time cap (--hours from SECONDS)
      plain             SIGTERM SECONDS after the roam starts, nothing fails (for the settings-folder test)
"""
import json
import os
import signal
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import freeroam as F   # noqa: E402

out, rundir, mode = sys.argv[1], sys.argv[2], sys.argv[3]
secs = float(sys.argv[4]) if len(sys.argv) > 4 else 15.0
T = {}
sent = []
_lock = threading.Lock()


def _rec(n, c):
    with _lock:
        sent.append((time.time(), n, c))


orig_init = F.Runner.__init__


def init(self, *a, **k):
    orig_init(self, *a, **k)
    sn = self.link.send_name

    def send_name(n, c, *a_, **k_):
        _rec(n, c)
        return sn(n, c, *a_, **k_)
    self.link.send_name = send_name


F.Runner.__init__ = init
for meth in ("roam", "end_save", "write_state", "save_workspace", "close"):
    orig = getattr(F.Runner, meth)

    def wrap(self, *a, _o=orig, _m=meth, **k):
        T.setdefault(_m, time.time())
        if mode == "sigterm-diskfull" and _m in ("write_state", "save_workspace") and (not a or a[0] is not None):
            raise OSError(28, "No space left on device")
        return _o(self, *a, **k)
    setattr(F.Runner, meth, wrap)

argv = ["run", "--dry", "--dir", rundir, "--seed", "3"]
if mode == "timecap":
    argv += ["--hours", "%.5f" % (secs / 3600.0)]
else:
    argv += ["--hours", "0.5"]

    def fire():
        t0 = time.time()
        while "roam" not in T and time.time() - t0 < 180:      # SECONDS counted from the roam's start (the start-up's
            time.sleep(0.1)                                     # length varies with the machine's load)
        time.sleep(secs)
        T["signal"] = time.time()
        os.kill(os.getpid(), signal.SIGTERM)
    threading.Thread(target=fire, daemon=True).start()
err = None
try:
    code = F.main(argv)
except BaseException as e:  # noqa: BLE001
    err = repr(e)
    code = None
json.dump(dict(T=T, sent=sent, code=code, err=err), open(out, "w"))
