#!/usr/bin/env python3
"""How many camera pictures a second actually reach this laptop from the Pi's /live stream, at a few JPEG
qualities (2026-10-22: the runner saw about 2 a second; the camera records 25).

    python tools/live/stream_rate.py st1maxfini.local
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sources as S  # noqa: E402


def rate(host, q, seconds=8.0):
    src = S.StreamSource("%s:8000/live?q=%d" % (host, q), log=lambda *a: None)
    t0, n, size = time.time(), 0, None
    try:
        for fr in src:
            n += 1
            size = fr.img.shape
            if time.time() - t0 > seconds:
                break
    finally:
        src.close()
    return n / (time.time() - t0), size


if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "st1maxfini.local"
    for q in (95, 80, 60):
        fps, size = rate(host, q)
        print("JPEG quality %d: %.1f pictures a second" % (q, fps), flush=True)
