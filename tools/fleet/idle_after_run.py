#!/usr/bin/env python3
"""Robots sleep 10 minutes after a run ends when no new run has started (e13, Nick 10 Nov).

Why: on 9 Nov the home robots were left awake after runs started from the terminal and lost about 19 % an hour
(e12 report, from the release logs). Mission Control already sleeps the robots when its own runs end (Advanced "sleep
when a run ends") and after a refused start with 10 minutes idle (e11); runs started from the terminal
(./run-fleet-pi.sh freeroam, visit, calib, ladder ...) slept nobody. Now every run (the Python entry points of
freeroam.py, mission_run.py, calib_run.py, ladder.py; whoever started them) hands its robots to the same rule:

    the run ends  ->  a detached waiter starts  ->  after IDLE_MIN minutes, if no run started since and none is going,
    every robot is sent `sleep <check-in>` on the WiFi broadcast (tools/pi/robot_sleep.py, as Mission Control does)

The runs going and the last start are kept in <conf dir>/run_activity.json (a run that dies leaves its pid there; a pid
no longer alive does not count). Practice and dry runs never schedule a sleep (no real robots).

Switch: MIRLT_IDLE_SLEEP_MIN (default 10, Nick's number; 0 = never: the way before e13). The waiter's log:
<conf dir>/idle_sleep.log.

    python3 tools/fleet/idle_after_run.py status          what is going, when the last run ended, the next sleep
    python3 tools/fleet/idle_after_run.py wait --after T  (the waiter itself; started by the runs)
"""
import argparse
import contextlib
import json
import os
import subprocess
import sys
import time

VERSION = "idle-after-run-2026-11-10e14"   # e14: dark-picture time does not count (dark_watch)
IDLE_MIN = 10.0          # Nick's number (10 Nov); no DOE (his choice of how long a person may take between runs)
HERE = os.path.dirname(os.path.abspath(__file__))
SLEEP_PY = os.path.join(os.path.dirname(HERE), "pi", "robot_sleep.py")


def conf_dir():
    return os.path.expanduser(os.environ.get("MIRLT_CONF_DIR") or "~/.config/mirlt")


def act_path():
    return os.path.join(conf_dir(), "run_activity.json")


def idle_min():
    try:
        return float(os.environ.get("MIRLT_IDLE_SLEEP_MIN", IDLE_MIN))
    except ValueError:
        return IDLE_MIN


def _load():
    try:
        with open(act_path()) as f:
            d = json.load(f)
            return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def _save(d):
    try:
        os.makedirs(conf_dir(), exist_ok=True)
        tmp = act_path() + ".%d.tmp" % os.getpid()
        with open(tmp, "w") as f:
            json.dump(d, f, indent=1)
        os.replace(tmp, act_path())
    except OSError:
        pass


def _alive(pid):
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, ValueError, TypeError):
        return False


def going(d=None):
    """The runs still going (their process alive)."""
    d = _load() if d is None else d
    return [g for g in d.get("going", []) if _alive(g.get("pid"))]


def is_dry(argv):
    a = list(argv if argv is not None else sys.argv[1:])
    return any(x in ("--dry", "--practice-real", "--dry-run", "--sim") or x.startswith("--dry=") for x in a) or \
        os.environ.get("FLEET_DRY") == "1"


def off():
    return idle_min() <= 0 or bool(os.environ.get("PYTEST_CURRENT_TEST"))


def begin(kind, dry=False):
    """A run starts: recorded (a waiter that wakes later sees it and does not sleep the robots)."""
    if off():
        return
    d = _load()
    d["going"] = going(d) + [dict(pid=os.getpid(), kind=kind, t=time.time(), dry=bool(dry))]
    if not dry:
        d["last_start"] = time.time()
    _save(d)


def end(kind, dry=False, spawn=True):
    """A run ends: removed from the list; a real run schedules the sleep (a detached waiter). Returns the waiter's
    planned time (None: none)."""
    if off():
        return None
    d = _load()
    d["going"] = [g for g in going(d) if g.get("pid") != os.getpid()]
    at = None
    if not dry:
        now = time.time()
        d["last_end"] = now
        at = now + idle_min() * 60.0
        d["sleep_at"] = at
        d["sleep_kind"] = kind
    _save(d)
    if at is not None and spawn:
        try:
            subprocess.Popen([sys.executable, os.path.abspath(__file__), "wait", "--after", "%.3f" % d["last_end"]],
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True, close_fds=True)
            print("The robots will be put to sleep in %.0f min if no new run starts (MIRLT_IDLE_SLEEP_MIN=0: never)."
                  % idle_min(), flush=True)
        except OSError as e:
            print("(the sleep after the run could not be scheduled: %s)" % e, flush=True)
    return at


@contextlib.contextmanager
def track(kind, argv=None, dry=None):
    """with track("freeroam", argv): run ... : begin, then end (also on an exception or Ctrl+C)."""
    dr = is_dry(argv) if dry is None else bool(dry)
    begin(kind, dr)
    try:
        yield
    finally:
        end(kind, dr)


def decide(d, after, now):
    """(sleep?, why) for a waiter scheduled by the run that ended at `after`."""
    if (d.get("last_start") or 0) > after:
        return False, "a new run started after this one ended"
    g = going(d)
    if [x for x in g if not x.get("dry")]:
        return False, "a run is going (%s)" % ", ".join(x.get("kind", "?") for x in g)
    if (d.get("last_end") or 0) > after + 1e-3:
        return False, "a later run ended; its own waiter decides"
    if now - after < idle_min() * 60.0 - 1.0:
        return False, "not yet"
    return True, "%.0f min with no run since the %s run ended" % (idle_min(), d.get("sleep_kind") or "last")


def log(line):
    try:
        os.makedirs(conf_dir(), exist_ok=True)
        with open(os.path.join(conf_dir(), "idle_sleep.log"), "a") as f:
            f.write("%s %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), line))
    except OSError:
        pass


def wait(after, sleep_fn=None, dark=None):
    """(e14) Time while a camera's picture is dark (dark_watch.is_dark_now) does not count towards the idle minutes."""
    if dark is None:
        try:
            import dark_watch as DW
            dark = DW.is_dark_now
        except Exception:  # noqa: BLE001
            dark = lambda: False  # noqa: E731
    t_end = after + idle_min() * 60.0
    while time.time() < t_end:
        t0 = time.time()
        time.sleep(min(30.0, max(0.5, t_end - time.time())))
        if dark():
            t_end += time.time() - t0
            log("the picture is dark (the camera adjusting to the light): this time does not count")
        if (_load().get("last_start") or 0) > after:
            log("no sleep: a new run started")
            return False
    ok, why = decide(_load(), after, time.time())
    if not ok:
        log("no sleep: " + why)
        return False
    ck = 10
    try:
        import sites as SI
        ck = int((SI.load() or {}).get("sleep_checkin_min", 10))
    except Exception:  # noqa: BLE001
        pass
    if sleep_fn is None:
        r = subprocess.run([sys.executable, SLEEP_PY, "sleep", str(ck), "255.255.255.255"], timeout=30,
                           capture_output=True, text=True)
        log("SLEEP sent (%s): every robot `sleep %d`; rc %s %s" % (why, ck, r.returncode, (r.stdout or r.stderr or "")
                                                                   .strip()[-200:]))
    else:
        sleep_fn(ck)
        log("SLEEP sent (%s)" % why)
    d = _load()
    d["slept_at"] = time.time()
    d.pop("sleep_at", None)
    _save(d)
    return True


def status():
    d = _load()
    g = going(d)
    print("Runs going: %s" % (", ".join("%s (pid %s)" % (x.get("kind"), x.get("pid")) for x in g) or "none"))
    for k in ("last_start", "last_end", "sleep_at", "slept_at"):
        if d.get(k):
            print("  %-10s %s" % (k, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(d[k]))))
    print("Rule: robots sleep %s after a run ends with no new run." % (
        "%.0f min" % idle_min() if idle_min() > 0 else "never (MIRLT_IDLE_SLEEP_MIN=0)"))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd")
    w = sp.add_parser("wait")
    w.add_argument("--after", type=float, required=True)
    sp.add_parser("status")
    a = ap.parse_args(argv)
    if a.cmd == "wait":
        sys.path.insert(0, HERE)
        return 0 if wait(a.after) else 1
    return status()


if __name__ == "__main__":
    sys.exit(main())
