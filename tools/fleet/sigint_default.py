#!/usr/bin/env python3
"""(e24, the 24 Nov usability audit, B1) Ctrl+C always reaches a run program.

A program started from a shell that IGNORES Ctrl+C (a script's `&`, nohup, an AI agent's background shell) inherits
"ignore" for SIGINT, and Python then never raises KeyboardInterrupt: Mission Control's STOP (a Ctrl+C to the run's
process group) did nothing, and the run stopped only 52 s later at the SIGTERM step, practice robots driving on
meanwhile. Each run program calls restore() first thing; Mission Control and mission_run also start their children
with child_default (SIGINT back to its default before the child's program starts).

Safety switch MIRLT_SIGINT_RESTORE (tools/fleet/safety_group.py): 1 (automatic) | 0: left as inherited, as before e24.

Changelog
  2026-11-24  first version (e24).
"""
import signal

VERSION = "sigint-default-2026-11-24"


def on():
    try:
        import safety_group as SG
        return SG.flag("MIRLT_SIGINT_RESTORE")
    except Exception:  # noqa: BLE001  (a program outside tools/fleet's path: the safe default)
        return True


def restore():
    """SIGINT ignored at start -> Python's own handler (KeyboardInterrupt). Returns True when it changed something."""
    try:
        if on() and signal.getsignal(signal.SIGINT) == signal.SIG_IGN:
            signal.signal(signal.SIGINT, signal.default_int_handler)
            return True
    except (ValueError, OSError):      # not the main thread (tests)
        pass
    return False


def child_default():
    """preexec_fn for subprocess.Popen: the child starts with SIGINT at its default, whatever this process inherited."""
    try:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
    except (ValueError, OSError):
        pass


def preexec():
    """The preexec_fn to pass (None when the switch is off: exactly as before e24)."""
    return child_default if on() else None
