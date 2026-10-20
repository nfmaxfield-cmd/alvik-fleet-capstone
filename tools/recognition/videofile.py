#!/usr/bin/env python3
"""An ordinary video file (from a phone, a webcam app, a screen recorder) made into a recording folder.

    python tools/recognition/videofile.py <video file> [<folder to put the recording in>]

The studio's "Add recording" does this by itself when it is given a video file. A recording
folder is what the Pi's recorder writes (tools/pi/camrec.py):

  video.avi    the frames (Motion-JPEG, so every frame can be read on its own)
  frames.csv   frame, t_wall (Unix seconds), t_mono (seconds since the start), from the video's
               own timing (phones often vary the frame rate a little; each frame keeps its time)
  meta.json    width, height, frame rate, and which file it was made from

The folder goes next to the video file, named after it (holiday.mp4 -> holiday/), unless that
folder can't be written: then into the given fallback folder (the studio passes the project's
"recordings" folder). The video file itself is never changed. Large videos are made smaller
(at most MAX_SIDE pixels on the long side) so that finding the moving objects and labelling
stay quick. Converting the same file again finds the folder made last time and reuses it.
"""
import csv
import datetime as dt
import io
import json
import os
import shutil
import sys
import time

VIDEO_EXTS = (".mp4", ".mov", ".m4v", ".avi", ".mkv", ".wmv", ".webm", ".3gp", ".mpg", ".mpeg", ".mts", ".m2ts")
MAX_SIDE = 960          # long side of the converted picture, in pixels (smaller videos keep their size)
QUALITY = 90            # JPEG quality of each frame in video.avi
SHORT_S = 120.0         # below this, say that a few minutes or more works best
MADE_BY = "Maxfield IRL Tracking (Add recording: video file)"


class ConvertError(ValueError):
    """A plain-words reason the video could not be made into a recording."""


def is_video_file(path):
    return os.path.isfile(path) and os.path.splitext(path)[1].lower() in VIDEO_EXTS


def _signature(path):
    st = os.stat(path)
    return dict(source_file=os.path.abspath(path), source_size=st.st_size, source_mtime=int(st.st_mtime))


def _writable(folder):
    try:
        os.makedirs(folder, exist_ok=True)
        probe = os.path.join(folder, ".mxirl-write-test-%d" % os.getpid())
        with open(probe, "w") as f:
            f.write("x")
        os.remove(probe)
        return True
    except OSError:
        return False


def _made_from(folder):
    """The meta.json source signature of a recording folder made by this module (or None)."""
    p = os.path.join(folder, "meta.json")
    if not (os.path.exists(os.path.join(folder, "video.avi")) and os.path.exists(p)):
        return None
    try:
        with open(p, encoding="utf-8") as f:
            m = json.load(f)
    except (OSError, ValueError):
        return None
    if m.get("made_by") != MADE_BY:
        return None
    return dict(source_size=m.get("source_size"), source_mtime=m.get("source_mtime"))


def destination(video_path, fallback_parent=None):
    """(folder the recording goes in, an existing conversion of the same file to reuse or None).
    Next to the video when that can be written, else under fallback_parent."""
    video_path = os.path.abspath(video_path)
    stem = os.path.splitext(os.path.basename(video_path))[0].strip() or "video"
    sig = _signature(video_path)
    parents = [os.path.dirname(video_path)]
    if fallback_parent:
        parents.append(os.path.abspath(fallback_parent))
    for parent in parents:
        # the same file converted before (in either place): reuse it
        for k in range(1, 50):
            cand = os.path.join(parent, stem if k == 1 else "%s-%d" % (stem, k))
            if not os.path.exists(cand):
                break
            made = _made_from(cand)
            if made and made["source_size"] == sig["source_size"] and made["source_mtime"] == sig["source_mtime"]:
                return cand, cand
    for parent in parents:
        if not _writable(parent):
            continue
        for k in range(1, 1000):
            cand = os.path.join(parent, stem if k == 1 else "%s-%d" % (stem, k))
            if not os.path.exists(cand) and not os.path.exists(cand + ".partial"):
                return cand, None
    raise ConvertError("There is no folder this computer lets me write the recording into (next to the video: %s)."
                       % os.path.dirname(video_path))


def convert(video_path, fallback_parent=None, progress=None, stop=None, max_side=MAX_SIDE, quality=QUALITY):
    """Make a recording folder from a video file. progress(done, total): frames so far, and about
    how many there are. stop(): return True to stop (the half-made folder is removed).
    Returns dict(folder, frames, seconds, fps, size, scaled_from, reused, notes)."""
    import cv2
    video_path = os.path.abspath(os.path.expanduser(video_path))
    if not os.path.isfile(video_path):
        raise ConvertError("There is no file at %s." % video_path)
    folder, reuse = destination(video_path, fallback_parent)
    if reuse:
        with open(os.path.join(reuse, "meta.json"), encoding="utf-8") as f:
            m = json.load(f)
        if progress:
            progress(1, 1)
        return dict(folder=reuse, frames=m.get("n_frames"), seconds=m.get("seconds"), fps=m.get("fps_reported"),
                    size=[m.get("width"), m.get("height")], scaled_from=m.get("scaled_from"), reused=True,
                    notes=_notes(m.get("seconds") or 0.0))
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ConvertError("This video file can't be read: %s. Save or export it again as an MP4 file "
                           "(most phone and webcam apps can) and try again." % os.path.basename(video_path))
    fps_file = cap.get(cv2.CAP_PROP_FPS) or 0.0
    if not (1.0 <= fps_file <= 1000.0):
        fps_file = 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    ok, img = cap.read()
    if not ok or img is None:
        cap.release()
        raise ConvertError("No pictures could be read from %s. Save or export it again as an MP4 file and try "
                           "again." % os.path.basename(video_path))
    h0, w0 = img.shape[:2]
    k = min(1.0, float(max_side) / max(w0, h0)) if max_side else 1.0
    w, h = (int(round(w0 * k)) // 2 * 2, int(round(h0 * k)) // 2 * 2) if k < 1.0 else (w0, h0)
    tmp = folder + ".partial"
    if os.path.exists(tmp):
        shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    out = cv2.VideoWriter(os.path.join(tmp, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), fps_file, (w, h))
    try:
        if not out.isOpened():
            raise ConvertError("The recording's video could not be written in %s." % os.path.dirname(folder))
        try:
            out.set(cv2.VIDEOWRITER_PROP_QUALITY, quality)
        except Exception:
            pass
        times, n, last_t = [], 0, None
        t_seen = False
        t_last_tick = 0.0
        while ok and img is not None:
            if stop and stop():
                raise InterruptedError("stopped")
            t = (cap.get(cv2.CAP_PROP_POS_MSEC) or 0.0) / 1000.0
            if t > 0:
                t_seen = True
            if img.shape[1] != w or img.shape[0] != h:
                img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            out.write(img)
            times.append(t)
            n += 1
            now = time.time()
            if progress and (now - t_last_tick > 0.25 or n == 1):
                t_last_tick = now
                progress(n, max(total, n))
            ok, img = cap.read()
        out.release()
        cap.release()
        # each frame's time from the video itself; frames whose time is missing or goes backwards
        # get the time one frame after the one before
        step = 1.0 / fps_file
        clean = []
        for i, t in enumerate(times):
            if not t_seen:
                t = i * step
            if last_t is not None and t <= last_t:
                t = last_t + step
            clean.append(t)
            last_t = t
        t0 = clean[0]
        clean = [t - t0 for t in clean]
        seconds = clean[-1] + step if clean else 0.0
        fps = (len(clean) - 1) / clean[-1] if len(clean) > 1 and clean[-1] > 0 else fps_file
        sig = _signature(video_path)
        start_wall = sig["source_mtime"] - seconds          # a phone writes the file when the recording ends
        buf = io.StringIO()
        wr = csv.writer(buf, lineterminator="\n")
        wr.writerow(["frame", "t_wall", "t_mono"])
        for i, t in enumerate(clean):
            wr.writerow([i, "%.4f" % (start_wall + t), "%.4f" % t])
        with open(os.path.join(tmp, "frames.csv"), "w", newline="", encoding="utf-8") as f:
            f.write(buf.getvalue())
        meta = dict(device="video file", width=w, height=h, fps_requested=round(fps_file, 3), fps_reported=round(fps, 3),
                    n_frames=len(clean), seconds=round(seconds, 2),
                    start=dt.datetime.fromtimestamp(start_wall).isoformat(timespec="seconds"),
                    scaled_from=[w0, h0] if (w, h) != (w0, h0) else None, made_by=MADE_BY,
                    converted=dt.datetime.now().isoformat(timespec="seconds"),
                    note="made from an ordinary video file; t_wall is estimated from the file's time (the end of "
                         "the recording), so it is only as right as the phone's or computer's clock", **sig)
        with open(os.path.join(tmp, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
        os.replace(tmp, folder)
    except BaseException:
        try:
            out.release()
        except Exception:
            pass
        cap.release()
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    if progress:
        progress(n, n)
    return dict(folder=folder, frames=n, seconds=round(seconds, 1), fps=round(fps, 2), size=[w, h],
                scaled_from=meta["scaled_from"], reused=False, notes=_notes(seconds))


def _notes(seconds):
    if seconds and seconds < SHORT_S:
        return ["This video is only %d s long. A few minutes or more works best: training cuts the recording into "
                "time pieces and needs some for learning, some for picking the best round and some for scoring."
                % round(seconds)]
    return []


def main(argv=None):
    a = list(sys.argv[1:] if argv is None else argv)
    if not a or a[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    last = [0.0]

    def prog(done, total):
        if time.time() - last[0] > 0.5 or done == total:
            last[0] = time.time()
            print("  %d / %d frames" % (done, total), end="\r", flush=True)
    try:
        res = convert(a[0], a[1] if len(a) > 1 else None, progress=prog)
    except ConvertError as e:
        print("\n" + str(e))
        return 1
    print("\n%s: %s (%d frames, %.0f s, %dx%d)" % ("Already made" if res["reused"] else "Made", res["folder"],
                                                   res["frames"] or 0, res["seconds"] or 0, res["size"][0], res["size"][1]))
    for n in res["notes"]:
        print(n)
    return 0


if __name__ == "__main__":
    sys.exit(main())
