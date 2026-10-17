#!/usr/bin/env python3
"""Check that a recording session is complete and lines up, right after you
record it.

    python tools/check_session.py logs/20261018-101500
    python tools/check_session.py logs/20261018-101500 --video recordings/20261018-101455_E1

Reports, per robot and run:
  - full sensor log (RAW): rows received vs sent, rate, longest gap
  - battery at the start and end, and how the run ended
  - clock: how robot time converts to laptop time, round-trip times, drift
  - the robot's still-and-level IMU reading (checks the units)
With --video (a camrec.py recording folder; needs opencv-python and numpy):
  - camera frame rate and dropped frames
  - the light flashes found in the video, matched to the robot's own flash
    records, and the time offset between video and robot clocks

The laptop and Pi clocks are set over the internet and usually agree within
tens of milliseconds; the flash offset measures that difference plus the
camera's own delay, which is what latency work needs.
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics as st
import sys
from collections import defaultdict


def rows(folder, name):
    out = []
    for path in sorted(glob.glob(os.path.join(folder, name + "*.csv"))):
        base = os.path.basename(path)[:-4]
        if base != name and not (base.startswith(name + "_") and base[len(name) + 1:].isdigit()):
            continue
        with open(path, newline="") as f:
            out += list(csv.DictReader(f))
    return out


def fnum(x, default=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


# ------------------------------------------------------------------ clock
def clock_fits(folder):
    """robot -> (a, b, info): laptop_time = a + b * robot_ms / 1000, from the
    fastest answer in each burst of clock checks."""
    by = defaultdict(list)
    for r in rows(folder, "clock_sync"):
        by[r["robot"]].append(r)
    fits = {}
    for robot, rs in by.items():
        rs.sort(key=lambda r: fnum(r["t_send"]))
        bursts, cur = [], []
        for r in rs:   # answers less than 1 s apart belong to one burst
            if cur and fnum(r["t_send"]) - fnum(cur[-1]["t_send"]) > 1.0:
                bursts.append(cur)
                cur = []
            cur.append(r)
        if cur:
            bursts.append(cur)
        best = [min(b, key=lambda r: fnum(r["rtt_ms"])) for b in bursts]
        xs = [fnum(r["robot_ms"]) / 1000 for r in best]
        ys = [(fnum(r["t_send"]) + fnum(r["t_recv"])) / 2 for r in best]
        rtts = [fnum(r["rtt_ms"]) for r in rs]
        if len(best) >= 2 and max(xs) - min(xs) > 5:
            mx, my = st.mean(xs), st.mean(ys)
            b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
            a = my - b * mx
        else:
            b = 1.0
            a = st.median(y - x for x, y in zip(xs, ys))
        resid = [y - (a + b * x) for x, y in zip(xs, ys)]
        fits[robot] = (a, b, dict(n=len(rs), bursts=len(best), rtt_med=st.median(rtts), rtt_max=max(rtts),
                                  drift_ppm=(b - 1) * 1e6, resid_ms=max(abs(e) for e in resid) * 1000))
    return fits


def to_laptop(fits, robot, ms):
    a, b, _ = fits[robot]
    return a + b * ms / 1000


# ------------------------------------------------------------------ video
def video_flashes(folder, sync_ms_guess=300):
    """Find light flashes: a jump in brightness in a small spot, followed by
    the opposite jump about sync_ms later. Returns candidate "on" frames as
    [(t_wall, frame, strength)]; the caller keeps the strongest one near each
    flash the robot logged, which rules out look-alikes elsewhere in the video."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("  (install opencv-python and numpy to check the video: pip install opencv-python numpy)")
        return None, None
    frames = []
    with open(os.path.join(folder, "frames.csv"), newline="") as f:
        for r in csv.DictReader(f):
            frames.append(fnum(r["t_wall"]))
    cap = cv2.VideoCapture(os.path.join(folder, "video.avi"))
    fps = 30.0
    if len(frames) > 2 and frames[-1] > frames[0]:
        fps = (len(frames) - 1) / (frames[-1] - frames[0])
    span = sync_ms_guess / 1000 * fps                  # flash length in frames
    smin, smax = max(2, int(span) - 1), int(math.ceil(span)) + 1
    # A light flash: pixels that brighten at frame j, STAY bright for about the
    # flash length, then darken. Moving robots and hands brighten and darken
    # pixels too, but almost never in that pattern.
    buf, scores = [], []   # scores[j] = (strength, x, y) for "a flash turned on at frame j"
    k = 4                  # strongest pixels averaged; a light seen from 2 m covers only a few pixels

    def score_at(b):       # b[0] is the frame before the candidate "on" frame
        before = b[0]
        on = np.min(np.stack(b[1:1 + smin]), axis=0) - before          # stayed brighter than before
        offs = [b[1 + s - 1] - b[1 + s] for s in range(smin, smax + 1) if 1 + s < len(b)]
        off = np.max(np.stack(offs), axis=0)                            # then went dark again
        p = np.minimum(on, off).ravel()
        idx = np.argpartition(p, -k)[-k:]
        ys, xs = np.unravel_index(idx, before.shape)
        return float(np.mean(p[idx])), float(np.median(xs)), float(np.median(ys))

    while True:
        ok, img = cap.read()
        if not ok:
            break
        buf.append(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.int16))
        if len(buf) == smax + 2:
            scores.append(score_at(buf))
            buf.pop(0)
    cap.release()
    if len(scores) < 3:
        return [], frames
    v = [x[0] for x in scores]
    med = st.median(v)
    mad = st.median(abs(x - med) for x in v)
    thr = max(30.0, med + 10 * mad)
    # candidate "on" frames: (laptop-clock time of the frame, frame number, strength)
    found = [(frames[j + 1], j + 1, sc[0]) for j, sc in enumerate(scores) if sc[0] > thr and j + 1 < len(frames)]
    return found, frames


# ------------------------------------------------------------------ report
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="console session folder, e.g. logs/20261018-101500")
    ap.add_argument("--video", nargs="*", default=[], help="camrec.py recording folder(s) from the same session")
    args = ap.parse_args()
    S = args.session

    info = rows(S, "runinfo")
    if not info:
        print("No RUNINFO records here: was the robot running firmware 2026-10-17a or later?")
    ends = {(r["robot"], r["run"]): r for r in rows(S, "runend")}
    raw = defaultdict(list)
    for r in rows(S, "raw"):
        raw[(r["robot"], r["run"])].append(r)
    params = defaultdict(dict)
    for r in rows(S, "param"):
        params[(r["robot"], r["run"])][r["name"]] = r["value"]

    print(f"\nSESSION {os.path.basename(os.path.normpath(S))}")
    try:
        with open(os.path.join(S, "session.json")) as f:
            meta = json.load(f)
        print(f"  laptop {meta.get('laptop')}, repo commit {meta.get('repo_commit')}, started {meta.get('start')}")
    except (OSError, ValueError):
        pass

    print("\nRUNS")
    hdr = ["robot", "run", "experiment", "fw", "result", "secs", "battery", "raw got/sent", "loss", "rate/s", "max gap ms", "az still"]
    table = []
    problems = []
    for r in info:
        key = (r["robot"], r["run"])
        e = ends.get(key, {})
        rr = raw.get(key, [])
        ms = sorted(fnum(x["ms"]) for x in rr)
        sent = int(fnum(e.get("raw_rows_sent"), 0)) if e else 0
        got = len(rr)
        loss = 100 * (1 - got / sent) if sent else float("nan")
        rate = (len(ms) - 1) / ((ms[-1] - ms[0]) / 1000) if len(ms) > 2 and ms[-1] > ms[0] else float("nan")
        gap = max((b - a for a, b in zip(ms, ms[1:])), default=float("nan"))
        still = [fnum(x["az"]) for x in rr if x.get("step") == "0"][:20]   # before the robot moves
        az = st.mean(still) if still else float("nan")
        secs = fnum(e.get("duration_ms")) / 1000 if e else float("nan")
        bat = f"{r['battery']}->{e.get('battery', '?')}%"
        table.append([r["robot"], r["run"], r["experiment"], r["fw_version"], e.get("result", "no end record"),
                      f"{secs:.1f}", bat, f"{got}/{sent}", f"{loss:.1f}%", f"{rate:.0f}", f"{gap:.0f}", f"{az:.2f}"])
        if not e:
            problems.append(f"{key}: no RUNEND record (run still going, or its packet was lost)")
        if sent and loss > 2:
            problems.append(f"{key}: {loss:.1f}% of sensor rows lost; consider raw_hz 25")
        if gap == gap and gap > 200:
            problems.append(f"{key}: a {gap:.0f} ms gap in the sensor log")
    if table:
        w = [max(len(h), *(len(str(t[i])) for t in table)) for i, h in enumerate(hdr)]
        print("  " + "  ".join(h.ljust(w[i]) for i, h in enumerate(hdr)))
        for t in table:
            print("  " + "  ".join(str(c).ljust(w[i]) for i, c in enumerate(t)))
        print("  az still: vertical acceleration while the robot sits still before moving; about 1.00 means the"
              "\n  accelerometer columns are in g.")

    print("\nCLOCKS (robot time -> laptop time)")
    fits = clock_fits(S)
    if not fits:
        print("  no clock checks recorded")
    for robot, (a, b, i) in sorted(fits.items()):
        print(f"  {robot}: {i['n']} checks in {i['bursts']} bursts; round trip median {i['rtt_med']:.1f} ms, "
              f"worst {i['rtt_max']:.1f} ms; drift {i['drift_ppm']:+.0f} ppm; fit within {i['resid_ms']:.1f} ms")
        if i["rtt_med"] > 50:
            problems.append(f"{robot}: slow WiFi round trips ({i['rtt_med']:.0f} ms median)")

    syncs = rows(S, "sync")
    if args.video:
        print("\nVIDEO")
    for vf in args.video:
        found, frames = video_flashes(vf)
        if frames is None:
            continue
        name = os.path.basename(os.path.normpath(vf))
        if len(frames) > 2:
            iv = [b - a for a, b in zip(frames, frames[1:])]
            fps = (len(frames) - 1) / (frames[-1] - frames[0])
            slow = sum(1 for x in iv if x > 1.5 * st.median(iv))
            print(f"  {name}: {len(frames)} frames, {fps:.1f} fps, {slow} late frames (>1.5x normal spacing), "
                  f"longest gap {max(iv) * 1000:.0f} ms")
        offs = []
        for s in syncs:
            if s["robot"] not in fits:
                continue
            t = to_laptop(fits, s["robot"], fnum(s["ms_on"]))
            # Pi and laptop clocks normally agree within a second: search that window only
            cands = [f for f in found if abs(f[0] - t) < 1.0]
            near = max(cands, key=lambda f: f[2], default=None)
            if near:
                offs.append(near[0] - t)
                print(f"    {s['robot']} run {s['run']} {s['tag']} flash: video frame {near[1]}, "
                      f"video minus robot time {1000 * (near[0] - t):+.0f} ms")
            elif frames and frames[0] - 1 <= t <= frames[-1] + 1:
                print(f"    {s['robot']} run {s['run']} {s['tag']} flash: not seen in the video "
                      f"(lights out of view, or too small at this camera height?)")
                problems.append(f"flash {s['run']} {s['tag']} not found in {name}")
        if len(offs) >= 2:
            print(f"  video vs robot clock: {1000 * st.mean(offs):+.0f} ms on average, spread "
                  f"{1000 * st.stdev(offs):.0f} ms (one frame is {1000 / 30:.0f} ms)")
            if st.stdev(offs) > 0.05:
                problems.append("flash offsets vary by more than 50 ms: check the Pi's and laptop's clocks")

    print("\nCHECK")
    if problems:
        for p in problems:
            print("  - " + p)
    else:
        print("  everything lines up")
    return 0


if __name__ == "__main__":
    sys.exit(main())
