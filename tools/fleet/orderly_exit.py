#!/usr/bin/env python3
"""Orderly exit for programs whose threads run native code (build e12, 10 Nov: an abort "FATAL: exception not rethrown"
was seen on the Pi).

That glibc message comes from pthread_exit's forced unwind passing through C++ code that catches everything and does
not rethrow. Python (3.11 on the Pi) calls pthread_exit for a daemon thread that wakes up while the interpreter is
shutting down; a camera / tag-detector thread inside OpenCV or pupil_apriltags at that moment aborts the whole process
(and its last log lines and summary can be lost). It could not be reproduced on the build machine (Python 3.13, which
handles such threads differently), so this is a fix by reading, not by a reproduced crash.

finish(code): the program has already asked its threads to stop; wait up to WAIT_S for them to end, flush the output,
then exit normally when none is left, or with os._exit (no interpreter shutdown, so no thread is torn down inside native
code) when some are still running. Files the program wrote are closed by then (each caller closes its own first).
Switch: MIRLT_ORDERLY_EXIT=0 (exit as before).
"""
import os
import sys
import threading
import time

VERSION = "2026-11-10e14"   # e14 (school 17:46): stop_on_signals / quiet_shutdown: a SIGTERM sets a flag and is never raised (a KeyboardInterrupt raised inside w.join while native detector threads ran ended in "terminate called without an active exception"); MIRLT_SIGTERM_RAISE=1 the old raise. 2026-11-10a (e12): finish()
WAIT_S = 3.0      # chosen: longer than one picture's detection on a Pi 4 (about 0.1-1 s). No DOE (a shutdown wait)


def others():
    me = threading.current_thread()
    return [t for t in threading.enumerate() if t is not me and t.is_alive() and not isinstance(t, threading._DummyThread)]


def finish(code=0, wait_s=WAIT_S, log=None):
    code = int(code or 0) if not isinstance(code, int) else code
    if os.environ.get("MIRLT_ORDERLY_EXIT", "1") == "0":
        sys.exit(code)
    end = time.time() + wait_s
    for t in others():
        t.join(max(0.0, end - time.time()))
    left = others()
    for s in (sys.stdout, sys.stderr):
        try:
            s.flush()
        except Exception:  # noqa: BLE001
            pass
    if not left:
        sys.exit(code)
    if log:
        try:
            log("(exiting without the interpreter's shutdown: %d thread(s) still running: %s)" % (
                len(left), ", ".join(t.name for t in left)))
            sys.stdout.flush()
        except Exception:  # noqa: BLE001
            pass
    os._exit(code)


def stop_on_signals(names=("SIGTERM",)):
    """(e14, school 17:46: `timeout 60 tag_bridge.py`) The signals named set a flag; nothing is raised. The old handler
    raised KeyboardInterrupt from a generator lambda, wherever the main thread happened to be: once it landed inside
    w.join(timeout=8) while the native detector threads still ran, and the process ended in a C++ abort ("terminate
    called without an active exception"). The program's main loop checks the returned Event and leaves through its own
    orderly path. Switch MIRLT_SIGTERM_RAISE=1: the old raise (kept, off)."""
    import signal
    ev = threading.Event()
    if os.environ.get("MIRLT_SIGTERM_RAISE", "0") == "1":
        def _raise(*_):
            raise KeyboardInterrupt()
        for n in names:
            signal.signal(getattr(signal, n), _raise)
        return ev
    for n in names:
        signal.signal(getattr(signal, n), lambda *_: ev.set())
    return ev


def quiet_shutdown(ev=None):
    """(e14) Called first thing on the way out: from now on SIGTERM and Ctrl+C only set the flag, so nothing can be
    raised inside a thread join or a file close while the shutdown runs."""
    import signal
    ev = ev if ev is not None else threading.Event()
    if threading.current_thread() is not threading.main_thread() or os.environ.get("MIRLT_SIGTERM_RAISE", "0") == "1":
        return ev
    for n in ("SIGTERM", "SIGINT"):
        try:
            signal.signal(getattr(signal, n), lambda *_: ev.set())
        except (ValueError, OSError):
            pass
    return ev
