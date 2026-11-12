#!/usr/bin/env python3
"""Every uploaded free-roam run with camera logs, replayed through the tracking layer with e17 OFF and ON
(replay_one.py, one process each), then summarised to runs_summary.csv / robots_summary.csv. Logs only.

    python3 replay_all.py RUNS_ROOT OUT_DIR [--watch 0|1] [--jobs 2]

RUNS_ROOT holds each run's folder (the release zips unpacked; raw data stays out of the repo). A run is found by
its name anywhere under RUNS_ROOT (the folder that has follow.csv and bridge/track.csv).

Changelog
  2026-11-12 v1  first version (replay-all-nov12)
"""
import argparse
import concurrent.futures as cf
import os
import subprocess
import sys

VERSION = "replay-all-2026-11-12v1"
HERE = os.path.dirname(os.path.abspath(__file__))
TAG = "" if float(os.environ.get("REPLAY_LAG_S", "0")) == 0 else "-lag%s" % os.environ["REPLAY_LAG_S"]
RUNS = ["20261105-143553-freeroam", "20261105-155001-freeroam", "20261106-210129-freeroam",
        "20261107-122722-freeroam", "20261107-133604-freeroam", "20261107-143835-freeroam",
        "20261107-144400-freeroam", "20261109-123542-freeroam", "20261111-082651-freeroam",
        "20261111-085837-freeroam"]
NO_LOGS = ["20261106-061650-freeroam", "20261109-125248-freeroam"]   # uploaded, but ended before any FOLLOW / bridge


def find(root, name):
    for d, subs, files in os.walk(root, followlinks=True):
        if os.path.basename(d) == name and "follow.csv" in files and os.path.exists(os.path.join(d, "bridge",
                                                                                                    "track.csv")):
            return d
    return None


def one(folder, mode, out, watch):
    if os.path.exists(out):
        return out
    subprocess.run([sys.executable, "-I", os.path.join(HERE, "replay_one.py"), folder, mode, out, watch], check=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("out")
    ap.add_argument("--watch", default="1")
    ap.add_argument("--jobs", type=int, default=2)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    jobs = []
    for r in RUNS:
        f = find(a.root, r)
        if f is None:
            print("not found:", r)
            continue
        for m in ("off", "on"):
            jobs.append((f, m, os.path.join(a.out, f"{r}-{m}-w{a.watch}{TAG}.json"), a.watch))
    # the long runs first, so the two workers finish together
    jobs.sort(key=lambda j: -os.path.getsize(os.path.join(j[0], "follow.csv")))
    with cf.ProcessPoolExecutor(a.jobs) as ex:
        for f in cf.as_completed([ex.submit(one, *j) for j in jobs]):
            print("done", f.result(), flush=True)


if __name__ == "__main__":
    main()
