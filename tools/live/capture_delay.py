#!/usr/bin/env python3
"""Measure the camera's capture delay (picture taken -> the Pi stamps it) for
track_live.py --capture-delay-ms, from one run where all three were going:
camrec.py recording, run_plan.py (or the console) logging the robots, and
track_live.py --stream.

    python tools/live/capture_delay.py --session logs/<run_plan session> \\
        --recording <camrec recording folder> --clock logs/<live log>/clock.csv

How: every logged run starts and ends with a 300 ms white flash. The robot's
log says when (robot clock -> laptop clock, as tools/check_session.py does);
the video shows when the Pi got the frames (Pi clock). Their difference is
(Pi - laptop clock) + capture delay (tools/fusion/runs.py video_offset). The
live link's clock.csv measured (laptop - Pi clock) at the same time. So
    capture delay = video offset + (laptop - Pi offset from clock.csv)
(a few ms too small: clock.csv's offset includes the shortest network trip).
Without --clock, give the Pi-laptop offset another way (--pi-minus-laptop-s,
e.g. 0 if both run NTP)."""
import argparse
import csv
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (TOOLS, os.path.join(TOOLS, "fusion")):
    if p not in sys.path:
        sys.path.insert(0, p)


def clock_offset(path):
    """Median laptop - Pi offset (s) from track_live.py's clock.csv."""
    with open(path, newline="") as f:
        v = [float(r["offset_s"]) for r in csv.DictReader(f) if r.get("offset_s")]
    if not v:
        raise SystemExit("%s has no offsets" % path)
    return st.median(v), (max(v) - min(v))


def capture_delay(video_offset_s, laptop_minus_pi_s):
    return video_offset_s + laptop_minus_pi_s


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True, help="console / run_plan.py log folder of the same runs")
    ap.add_argument("--recording", required=True, help="camrec.py recording made at the same time")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--clock", help="track_live.py's clock.csv from the same time")
    g.add_argument("--pi-minus-laptop-s", type=float, help="Pi clock minus laptop clock, if known another way")
    args = ap.parse_args(argv)
    import runs as FR
    vo = FR.video_offset(args.session, args.recording)
    if vo is None:
        raise SystemExit("No start/end flashes matched between the video and the robots' logs.")
    off, n = vo[0], vo[2]
    if args.clock:
        lp, spread = clock_offset(args.clock)
        note = "clock.csv offset spread %.1f ms" % (spread * 1000)
    else:
        lp, note = -args.pi_minus_laptop_s, "offset given"
    d = capture_delay(off, lp)
    print("video offset %.1f ms from %d flashes (spread %.1f ms); laptop - Pi %.1f ms (%s)"
          % (off * 1000, n, vo[1] * 1000, lp * 1000, note))
    print("capture delay: %.0f ms  ->  track_live.py ... --capture-delay-ms %.0f" % (d * 1000, d * 1000))
    if not 0.0 <= d <= 0.2:
        print("That is outside 0-200 ms: check that the three were recorded at the same time.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
