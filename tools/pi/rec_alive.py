#!/usr/bin/env python3
"""Is the recorder really sending pictures? (MIRLT, 2026-10-28.)

Why: at 16:24 `./run-fleet-pi.sh doe` said "Recorder already running (tmux session fleet)" because the tmux window
`rec` existed, but no pictures came from 127.0.0.1:8000, and the loop stopped at L0 with "No pictures from the
overhead camera". The launcher now asks this for one picture from each camera before trusting a running recorder.

    python3 tools/pi/rec_alive.py [--side] [--timeout 3]
Exit 0: every camera asked sent a picture; 1: one did not (it says which). --side: the side camera (8001) too.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (os.path.join(os.path.dirname(HERE), "experiment"),):
    if p not in sys.path:
        sys.path.insert(0, p)

TIMEOUT_S = 3.0     # chosen: a running recorder sends 20-29 pictures a second, so 3 s is ~60 chances. No DOE: a guard


def alive(port, timeout=TIMEOUT_S, host="127.0.0.1", check=None):
    """(ok, words) for the camera at host:port: at least one picture within timeout."""
    if check is None:
        import e02_camera as CAM
        check = CAM.check_stream
    ok, words, _, _ = check(host, port, timeout=timeout, n=1)
    return ok, words


def main(argv=None, check=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--side", action="store_true")
    ap.add_argument("--timeout", type=float, default=TIMEOUT_S)
    a = ap.parse_args(argv)
    bad = []
    for name, port in [("overhead", 8000)] + ([("side", 8001)] if a.side else []):
        ok, words = alive(port, a.timeout, check=check)
        if not ok:
            bad.append("%s camera (127.0.0.1:%d): %s" % (name, port, words))
    if bad:
        print("The recorder's window is open but pictures are missing: " + "; ".join(bad))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
