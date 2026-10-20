#!/usr/bin/env python3
"""Pick the frames of a recording worth labelling, and save them as JPEGs.

(Named select_frames.py, not select.py: a file called select.py next to
app.py would hide Python's own 'select' module, which the web server needs.)

What gets picked, in plain words:
  - about one frame per second (every_s);
  - extra frames at moments where guesses are most likely to be wrong:
      appear     a tracked object first shows up
      disappear  a tracked object is seen for the last time
      close      two objects are close together (mix-ups happen here)
      low        a guess has a low score
  - the first and last frame.
Any picked frame except the first and last (and frames already labelled) is
skipped when it is a near-duplicate of the last picked frame (picture almost
the same and every object within a couple of pixels of where it was), unless
nothing has been picked for max_gap_s. So two objects parked side by side
don't give a stack of the same picture every half second.
Picked frames are written as labels/frames/<frame:06d>.jpg.

    python tools/label/select_frames.py <recording folder>   # dry run: prints what it would pick
"""
import argparse
import collections
import math
import os
import sys

import numpy as np
import cv2

DUP_LEVEL = 1.5     # mean grey-level difference of the small pictures below which two frames look the same


def candidates(rows_by_frame, n_frames, times, every_s=1.0, low_score=0.6, close_every_s=0.5):
    """Frames worth a look and why. rows_by_frame: frame -> tracker rows.
    Returns {frame: set of reasons}."""
    reasons = collections.defaultdict(set)
    reasons[0].add("start")
    reasons[n_frames - 1].add("end")
    # About one frame per second (by clock time, so dropped frames don't matter).
    last_slot = None
    for f in range(n_frames):
        slot = int(math.floor(times[f] / every_s))
        if slot != last_slot:
            reasons[f].add("regular")
            last_slot = slot
    # Track starts and ends.
    first, last_seen = {}, {}
    for f in sorted(rows_by_frame):
        for r in rows_by_frame[f]:
            first.setdefault(r["obj"], f)
            if r["source"] != "track":
                last_seen[r["obj"]] = f
    for obj, f in first.items():
        reasons[f].add("appear")
    for obj, f in last_seen.items():
        if f < n_frames - 1:
            reasons[f].add("disappear")
    # Close pairs and low scores: the start of each episode, then one frame
    # every close_every_s (close) or the lowest-scoring frame per second (low).
    close_slot, low_best = None, {}
    in_close = in_low = False
    for f in range(n_frames):
        rows = rows_by_frame.get(f, [])
        close = any(r.get("close") for r in rows)
        if close:
            slot = int(math.floor(times[f] / close_every_s))
            if not in_close or slot != close_slot:
                reasons[f].add("close")
                close_slot = slot
        in_close = close
        lows = [r["score"] for r in rows if r["source"] != "user" and r["score"] < low_score]
        if lows:
            if not in_low:
                reasons[f].add("low")
            sec = int(math.floor(times[f]))
            m = min(lows)
            if sec not in low_best or m < low_best[sec][0]:
                low_best[sec] = (m, f)
        in_low = bool(lows)
    for m, f in low_best.values():
        reasons[f].add("low")
    return reasons


def save_jpeg(path, img, quality=95):
    """Write a JPEG. (cv2.imwrite cannot handle non-English letters in Windows
    folder names, so encode in memory and let Python write the file.)"""
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise IOError("could not encode frame for " + path)
    with open(path, "wb") as f:
        f.write(buf.tobytes())


def _thumb(img):
    """Small grey picture with its average removed (so a lighting flicker
    alone does not make two frames look different)."""
    g = cv2.cvtColor(cv2.resize(img, (80, 60), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY).astype(np.float32)
    g = cv2.GaussianBlur(g, (3, 3), 0)
    return g - g.mean()


def _same_objects(a, b, px=2.0):
    if len(a) != len(b):
        return False
    pa = sorted((r["obj"], r["x"], r["y"]) for r in a)
    pb = sorted((r["obj"], r["x"], r["y"]) for r in b)
    return all(oa == ob and math.hypot(xa - xb, ya - yb) <= px for (oa, xa, ya), (ob, xb, yb) in zip(pa, pb))


def pick(video_path, reasons, rows_by_frame, times, out_dir=None, max_gap_s=10.0, dup_level=DUP_LEVEL,
         merge_frames=3, keep=(), quality=95, progress=None):
    """Go through the video once, decide which candidate frames to keep, and
    (if out_dir is given) write each kept frame as <frame:06d>.jpg.
    keep: frames that must be kept (e.g. already reviewed).
    Returns (kept frames sorted, {frame: sorted reasons})."""
    keep = set(keep)
    events = sorted({f for f, rs in reasons.items() if rs - {"regular"}} | keep)
    # Event frames within merge_frames of each other: keep the first, merge reasons.
    merged = {}
    last = None
    for f in events:
        if last is not None and f - last <= merge_frames and f not in keep and last not in keep:
            merged[last] |= reasons[f]
            continue
        merged[f] = set(reasons.get(f, ())) | ({"kept"} if f in keep and not reasons.get(f) else set())
        last = f
    wanted = set(merged) | {f for f, rs in reasons.items() if "regular" in rs}
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    cap = cv2.VideoCapture(video_path)
    kept, out_reasons = [], {}
    last_thumb, last_f = None, None
    f = 0
    top = max(wanted) if wanted else -1
    while f <= top and cap.grab():
        if f in wanted:
            ok, img = cap.retrieve()
            if ok:
                th = _thumb(img)
                rs = merged.get(f, set()) | reasons.get(f, set())
                take = f in keep or bool(rs & {"start", "end"})
                if not take:
                    dup = (last_thumb is not None and float(np.abs(th - last_thumb).mean()) < dup_level
                           and _same_objects(rows_by_frame.get(f, []), rows_by_frame.get(last_f, [])))
                    take = not dup or (times[f] - times[last_f] >= max_gap_s)
                if take:
                    kept.append(f)
                    out_reasons[f] = sorted(merged.get(f, set()) | reasons.get(f, set()))
                    last_thumb, last_f = th, f
                    if out_dir:
                        path = os.path.join(out_dir, "%06d.jpg" % f)
                        if not os.path.exists(path):
                            save_jpeg(path, img, quality)
        if progress and f % 600 == 0:
            progress(f, top + 1)
        f += 1
    cap.release()
    if progress:
        progress(top + 1, top + 1)
    return kept, out_reasons


def extract(video_path, frames, out_dir, quality=95):
    """Write the given frames as JPEGs (only those not already there)."""
    os.makedirs(out_dir, exist_ok=True)
    need = sorted(f for f in frames if not os.path.exists(os.path.join(out_dir, "%06d.jpg" % f)))
    if not need:
        return 0
    wanted = set(need)
    cap = cv2.VideoCapture(video_path)
    f, n = 0, 0
    while f <= need[-1] and cap.grab():
        if f in wanted:
            ok, img = cap.retrieve()
            if ok:
                save_jpeg(os.path.join(out_dir, "%06d.jpg" % f), img, quality)
                n += 1
        f += 1
    cap.release()
    return n


def main():
    import motion
    import tracker
    ap = argparse.ArgumentParser(description="Show which frames would be picked for labelling (writes nothing).")
    ap.add_argument("recording")
    args = ap.parse_args()
    video = os.path.join(args.recording, "video.avi")
    dets, info = motion.detect_recording(video)
    times = [f / info["fps"] for f in range(len(dets))]
    rows, _ = tracker.run_tracker(dets, times, info["robot_px"])
    rs = candidates(rows, len(dets), times)
    kept, why = pick(video, rs, rows, times)
    counts = collections.Counter(r for f in kept for r in why[f])
    print("%d frames picked out of %d (%.1f per minute): %s" % (len(kept), len(dets),
          60.0 * len(kept) / max(times[-1], 1e-9), dict(counts)))


if __name__ == "__main__":
    sys.exit(main())
