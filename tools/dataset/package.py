"""Package E02 sessions as a dataset others can use (the publication plan, 2026-10-25): one folder per session with
its logs and the recorder's frame list, a manifest (files, sizes, row counts, columns, sha256) and a data card
(DATASET.md) that explains every file and column.

    python3 tools/dataset/package.py --out dataset logs/e02-20261024-153612 [more session folders ...]
        [--recording /media/.../recordings/20261024-153604_E02] [--video] [--zip]

What it does to the files:
  - copies the session's CSVs, state, log, check pictures and snapshots, and each robot-logs-N folder;
  - with --recording: the recorder's frames.csv, events.csv and meta.json (the video only with --video: it is GBs);
  - scrubs the Pi's home folder and the WiFi's local addresses from text files (/home/<user> -> ~, 192.168.x.y ->
    robot-N in order of first appearance); never the numbers;
  - writes manifest.json and DATASET.md at the top.
It reads nothing else and changes nothing in the session folders. Tests: tools/dataset/test_package.py."""
import argparse
import csv
import hashlib
import json
import os
import re
import shutil
import sys
import time

TEXT_EXT = (".csv", ".txt", ".json", ".jsonl", ".md")
SKIP = ("sensor_trust",)                      # learned state, not data
HOME_RE = re.compile(r"/home/[A-Za-z0-9_.-]+")
IP_RE = re.compile(r"\b192\.168\.\d{1,3}\.\d{1,3}\b")

# what each file is (the data card); columns are listed from the files themselves
FILES = {
    "e02_log.txt": "The runner's own log, line by line, as it ran (times are local).",
    "e02_state.json": "The session's state at the end: plan, robots, the board, learned corrections, holds, trust.",
    "track.csv": "The overhead camera's position of the robot being followed, every picture it was seen.",
    "perf.csv": "The runner's camera processing each second: pictures handled, ms a picture, picture age.",
    "battery.csv": "Every robot's battery every 5 minutes, with whether it was moving or still.",
    "front_checks.csv": "Each front-sensor reading after a leg against what the camera and board map expected.",
    "runs.csv": "One row per scored run: condition (U, C, K, KC), route, start/end/far errors.",
    "e02_runs.csv": "The run plan as written at the start.",
    "fits.csv": "Each self-calibration fit: wheel scale, gyro scale and drift, and their residuals.",
    "kc_stops.csv": "Camera fixes during KC runs: belief, camera, blended position.",
    "confidence.csv": "Confidence pauses: where the robot stopped to let the camera catch up.",
    "quality_map.csv": "How well the camera saw robots in each cell of the board.",
    "raw.csv": "The robot's 20 Hz record during a run: wheels, gyro, accelerometer, distance sensors, line sensors.",
    "leg.csv": "Each leg the robot drove (go/turn): start and end pose by its own wheels and gyro.",
    "fix.csv": "Each camera fix sent to the robot: its pose before and after.",
    "edge.csv": "The robot's edge (drop) sensor events.",
    "runinfo.csv": "Per robot run: firmware, battery, WiFi signal, route.",
    "runend.csv": "Per robot run: how it ended, packets sent.",
    "param.csv": "Every setting the robot reported.",
    "clock.csv": "Robot clock readings (for matching robot ms to wall time).",
    "clock_sync.csv": "Round trips used to match robot clocks to the runner's.",
    "sync.csv": "Light flashes used to match the robot's clock to the video.",
    "calib_look.csv": "The compass check's look around at setup.",
    "frames.csv": "The recorder: one row per video picture (frame number, wall time, camera's own ms).",
    "events.csv": "The recorder: marks the runner put on the video (holds, answers, runs).",
    "meta.json": "The recorder's camera settings and its frame check.",
}
COLS = {
    "t": "wall time, Unix seconds", "t_recv": "wall time the runner received it, Unix seconds",
    "x_cm": "board x, cm (origin at the blocked corner)", "y_cm": "board y, cm", "th_deg": "heading, deg",
    "ms": "robot clock, ms since it started", "battery": "battery, %", "robot": "Left, Center or Right",
    "run": "robot run number", "source": "which tracker part saw it: lock, detector, both, held",
}


def scrub_text(s, ipmap):
    # the Pi's user name, wherever it appears (its host name and live-view address use it too)
    users = ipmap.setdefault("_users", set())
    users.update(m.group(0).split("/")[2] for m in HOME_RE.finditer(s))
    s = HOME_RE.sub("~", s)
    for u in sorted(users, key=len, reverse=True):
        s = re.sub(r"\b%s\b" % re.escape(u), "pi-host", s)

    def ip(m):
        return ipmap.setdefault(m.group(0), "robot-%d" % (1 + sum(1 for k in ipmap if k != "_users")))
    return IP_RE.sub(ip, s)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def csv_info(path):
    with open(path, newline="", errors="replace") as f:
        r = csv.reader(f)
        head = next(r, [])
        n = sum(1 for _ in r)
    return head, n


def copy_one(src, dst, ipmap):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    if src.endswith(TEXT_EXT):
        with open(src, errors="replace") as f:
            s = f.read()
        with open(dst, "w") as f:
            f.write(scrub_text(s, ipmap))
    else:
        shutil.copyfile(src, dst)


def package_session(sess, out, recording=None, video=False, ipmap=None):
    """Copy one session (and its recording) into out/<session name>. Returns the manifest entries."""
    ipmap = {} if ipmap is None else ipmap
    name = os.path.basename(os.path.normpath(sess))
    dest = os.path.join(out, name)
    entries = []
    for root, _dirs, files in os.walk(sess):
        for fn in sorted(files):
            if any(fn.startswith(s) for s in SKIP):
                continue
            src = os.path.join(root, fn)
            rel = os.path.relpath(src, sess)
            copy_one(src, os.path.join(dest, rel), ipmap)
            entries.append(os.path.join(name, rel))
    if recording:
        for fn in sorted(os.listdir(recording)):
            if fn.endswith((".avi", ".mp4", ".mkv")) or ".part-" in fn:
                if not video:
                    continue
            src = os.path.join(recording, fn)
            if os.path.isdir(src):
                continue
            copy_one(src, os.path.join(dest, "recording", fn), ipmap)
            entries.append(os.path.join(name, "recording", fn))
    return entries


def manifest(out, entries):
    rows = []
    for rel in entries:
        p = os.path.join(out, rel)
        e = dict(path=rel, bytes=os.path.getsize(p), sha256=sha256(p))
        if p.endswith(".csv"):
            e["columns"], e["rows"] = csv_info(p)
        rows.append(e)
    return rows


def data_card(rows, sessions):
    lines = ["# MIRLT E02 dataset", "",
             "Sessions of three Arduino Alvik robots on a plywood board under an overhead camera (ISE 401 capstone).",
             "Packaged %s by tools/dataset/package.py. Sessions: %s." % (time.strftime("%Y-%m-%d"), ", ".join(sessions)),
             "", "Home folders and the WiFi's local addresses are replaced (~, robot-N); nothing else is changed.",
             "Integrity: manifest.json has every file's sha256.", "", "## Files", ""]
    seen = set()
    for r in rows:
        base = os.path.basename(r["path"])
        if base in seen or base not in FILES:
            continue
        seen.add(base)
        lines.append("- **%s**: %s" % (base, FILES[base]))
        if r.get("columns"):
            lines.append("  Columns: " + ", ".join("`%s`" % c for c in r["columns"]))
    lines += ["", "## Common columns", ""] + ["- `%s`: %s" % kv for kv in COLS.items()]
    lines += ["", "## Not included", "",
              "- The video, unless packaged with --video (it is published separately as release files).",
              "- The learned sensor trust (state, not data)."]
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("sessions", nargs="+", help="session folders (logs/e02-...)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--recording", action="append", default=[],
                    help="the recorder's folder for each session, in the same order (optional)")
    ap.add_argument("--video", action="store_true", help="include the video files (GBs)")
    ap.add_argument("--zip", action="store_true", help="also write <out>.zip")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    ipmap, entries = {}, []
    for i, s in enumerate(a.sessions):
        if not os.path.isdir(s):
            print("No session folder %s" % s)
            return 1
        rec = a.recording[i] if i < len(a.recording) else None
        entries += package_session(s, a.out, rec, a.video, ipmap)
    rows = manifest(a.out, entries)
    with open(os.path.join(a.out, "manifest.json"), "w") as f:
        json.dump(dict(made=time.strftime("%Y-%m-%dT%H:%M:%S"), files=rows), f, indent=1)
    with open(os.path.join(a.out, "DATASET.md"), "w") as f:
        f.write(data_card(rows, [os.path.basename(os.path.normpath(s)) for s in a.sessions]))
    print("Packaged %d files from %d sessions in %s" % (len(rows), len(a.sessions), a.out))
    if a.zip:
        z = shutil.make_archive(os.path.normpath(a.out), "zip", a.out)
        print("Zip: %s" % z)
    return 0


if __name__ == "__main__":
    sys.exit(main())
