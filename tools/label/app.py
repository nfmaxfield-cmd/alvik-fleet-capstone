#!/usr/bin/env python3
"""Labelling app for the overhead camera: the computer guesses, you confirm
or correct.

    python tools/label/app.py <recording folder>
    python tools/label/app.py <recording folder> --model tools/detector/detector.py --weights model.pt
    python tools/label/app.py <recording folder> --port 8766

What it does, in order:
  1. Makes its own guesses for every frame of the recording: by default from
     motion (motion.py), or with --model from any detector that has the
     interface in docs/data-formats.md (load(path), predict(model, frame)).
     The guesses are cached in <recording>/labels/cache/, so the second start
     is quick.
  2. Follows the guesses from frame to frame (tracker.py), so every object
     keeps one number.
  3. Picks about one frame per second plus extra frames at tricky moments
     (select_frames.py) and saves them as labels/frames/<frame:06d>.jpg.
  4. Opens a page in your browser (served only on this computer, 127.0.0.1).
     You press Enter when the guesses are right, or fix them with the mouse
     and keys. Each fix is fed back to the tracker, so the frames after it
     get better guesses too.
  5. Saves labels/labels.csv and labels/review.json after every change, in
     the format of docs/data-formats.md. Close the window or press Ctrl+C
     any time; next start continues where you left off.

Only numpy and opencv-python are needed. Stop with Ctrl+C.
"""
import argparse
import collections
import csv
import datetime as dt
import importlib.util
import json
import math
import os
import random
import sys
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import motion          # noqa: E402
import tracker         # noqa: E402
import select_frames   # noqa: E402
import workarea        # noqa: E402
import timing          # noqa: E402

BRAND = "Maxfield IRL Tracking"
TAGLINE = "Identify. Recognize. Label."

# Object types when this labelling app runs on its own (the recognition studio passes the
# project's own types instead).
CLASSES = ["alvik", "hand", "unknown", "machine"]
# unsure: the person can't tell what it is yet; frames holding one are kept out of training
# and scoring until it is resolved.
STATUSES = ["pending", "confirmed", "corrected", "rejected", "unsure"]
SAME_PICTURE_S = 10.0      # same-picture frames are looked for this far either side of an accepted frame
# ... and each accepted object's own patch, at full size, must differ by less than this mean grey level
# (after removing the patch's average): a robot that turned or shifted a little in place is not "the same"
PATCH_LEVEL = 2.5


def _mem_status(st):
    """labels.csv writes "auto" for rows of auto-accepted frames; in memory
    they are confirmed rows of a frame whose state is auto."""
    return "confirmed" if st == "auto" else st


# Frames the tracker asks a person to look at, most urgent first, with the reason shown on the
# frame in plain words. "Most useful next" puts frames with any of these before everything else.
FLAG_TEXT = collections.OrderedDict([
    ("split_unresolved", "two lookalikes crossed here: which is which?"),
    ("chain_broken", "something passed over an object that was being followed: is it still there, and the same one?"),
    ("confirm_due", "this object has been followed a long way since it was last confirmed: is the guess still right?"),
])
OTHER_FLAG_TEXT = "the tracker asks for a look here"
CLIP_SECONDS = 1.0         # the clip on key C: this long before and after the frame
CLIP_MAX_FRAMES = 31       # ... at most this many pictures (spread evenly; the frame itself always in)
CLIP_WIDTH = 640           # ... made smaller than this width


# ---------------------------------------------------------------- hooks for the tracker (lane T)
# The tracker (tracker.py) will learn from a rejected spot: it keeps what the empty spot looks like
# there, scores later guesses that look like it down hard, and still finds something that looks
# different. Until that is wired in, the page's "Reject here" still rejects the guess on its frame
# and the spot is kept in review.json ("rejected_spots"), so nothing is lost.
def remember_rejected_spot(session, spot):
    """Tell the tracker to remember an empty spot. spot: dict(frame, x, y, r, saved).
    Returns True when the tracker took it (then the guesses are followed again from that frame).
    TODO(integration, lane T): match the name and arguments of the tracker's rejected-spot API."""
    fn = getattr(session.run, "remember_rejected", None)
    if not callable(fn):
        return False
    try:
        fn(frame=int(spot["frame"]), x=float(spot["x"]), y=float(spot["y"]), r=float(spot["r"]))
        return True
    except Exception as e:          # never stops the labelling
        session.log("The tracker could not take the rejected spot: %s" % e)
        return False


def forget_rejected_spot(session, spot):
    """Undo of remember_rejected_spot (Z after Reject here).
    TODO(integration, lane T): match the tracker's API."""
    fn = getattr(session.run, "forget_rejected", None)
    if not callable(fn):
        return False
    try:
        fn(frame=int(spot["frame"]), x=float(spot["x"]), y=float(spot["y"]), r=float(spot["r"]))
        return True
    except Exception as e:
        session.log("The tracker could not forget the rejected spot: %s" % e)
        return False
# where a label or guess came from: motion finder, tracker, trained model, a person, outside data such as a
# robot's log ("log"), the other camera ("camera"), a home position visit ("home"), the AI helper ("ai")
SOURCES = ["motion", "track", "model", "user", "log", "camera", "home", "ai"]
LABEL_COLUMNS = ["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg", "heading_known",
                 "source", "status", "score"]
CACHE_VERSION = 1


def now_iso():
    return dt.datetime.now().isoformat(timespec="seconds")


def write_atomic(path, text):
    """Write a file so that a crash never leaves half a file: write a temporary
    file, then swap it in. On Windows the swap fails while the file is open in
    another program (Excel), so it retries for a moment, then raises."""
    tmp = "%s.%d-%d.tmp" % (path, os.getpid(), threading.get_ident())    # its own name per writer
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    for attempt in range(10):
        try:
            os.replace(tmp, path)
            return
        except PermissionError:
            time.sleep(0.2)
    try:
        os.remove(tmp)
    except OSError:
        pass
    raise PermissionError("%s is open in another program (Excel?). Close it; your work is kept "
                          "and will be saved with the next change." % os.path.basename(path))


class RateClock:
    """Time left for a long job, from the recent rate (the last `window` seconds), not from
    the start: the first frames of a video are often quicker or slower than the rest."""

    def __init__(self, window=30.0):
        self.window = window
        self.samples = collections.deque()

    def update(self, done, total, now=None):
        """Note progress; returns the seconds left, or None while there is too little to go on."""
        now = time.time() if now is None else now
        self.samples.append((now, done))
        while len(self.samples) > 2 and now - self.samples[1][0] >= self.window:
            self.samples.popleft()
        t0, d0 = self.samples[0]
        if now - t0 < 2.0 or done <= d0 or not total:
            return None
        return max(0.0, (total - done) * (now - t0) / (done - d0))


def fmt_left(s):
    """Seconds left in plain words."""
    if s is None:
        return ""
    if s < 60:
        return "about %d s left" % max(1, int(round(s / 5.0) * 5) or 1)
    if s < 3600:
        return "about %d min left" % int(round(s / 60.0))
    return "about %.1f h left" % (s / 3600.0)


def wilson(k, n, z=1.96):
    """95% range for a proportion k/n (Wilson score interval)."""
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


# ----------------------------------------------------------------------------
# Recording and guesses
# ----------------------------------------------------------------------------

class Recording:
    """A folder written by tools/pi/camrec.py."""

    def __init__(self, folder):
        self.folder = os.path.abspath(folder)
        self.video = os.path.join(self.folder, "video.avi")
        if not os.path.exists(self.video):
            raise SystemExit("No video.avi in %s. Give the recording folder (the one with video.avi, "
                             "frames.csv and meta.json)." % self.folder)
        self.name = os.path.basename(self.folder.rstrip("/\\"))
        self.meta = {}
        mp = os.path.join(self.folder, "meta.json")
        if os.path.exists(mp):
            with open(mp) as f:
                self.meta = json.load(f)
        self.fps = float(self.meta.get("fps_reported") or self.meta.get("fps_requested") or 30.0)
        self.t_wall, self.times = [], []
        fp = os.path.join(self.folder, "frames.csv")
        if os.path.exists(fp):
            with open(fp, newline="") as f:
                for r in csv.DictReader(f):
                    self.t_wall.append(r.get("t_wall", ""))
                    try:
                        self.times.append(float(r["t_mono"]))
                    except (KeyError, ValueError):
                        self.times.append(len(self.times) / self.fps)
        if not self.t_wall:
            n = motion.count_frames(self.video)
            self.t_wall = [""] * n
            self.times = [f / self.fps for f in range(n)]
        # Make sure times never go backwards (the tracker divides by time steps).
        for i in range(1, len(self.times)):
            if self.times[i] <= self.times[i - 1]:
                self.times[i] = self.times[i - 1] + 1e-3
        self.n = len(self.times)
        self.width = int(self.meta.get("width") or 640)
        self.height = int(self.meta.get("height") or 480)


def _fmt(v, nd=1):
    return "" if v is None else ("%." + str(nd) + "f") % v


def save_detections(path, dets):
    """Detections cache: one row per guess; a frame the detector skipped has a
    row with cls 'skip'."""
    lines = ["frame,cls,x,y,heading_deg,score,angle90,source"]
    for f, ds in enumerate(dets):
        if ds is None:
            lines.append("%d,skip,,,,,," % f)
            continue
        for d in ds:
            if "angle90" not in d:
                a = ""
            elif d["angle90"] is None:
                a = "hidden"
            else:
                a = "%.2f" % d["angle90"]
            lines.append("%d,%s,%.2f,%.2f,%s,%.4f,%s,%s" % (
                f, d["cls"], d["x"], d["y"], _fmt(d.get("heading_deg")), d["score"], a, d.get("source", "")))
    write_atomic(path, "\n".join(lines) + "\n")


def load_detections(path, n):
    dets = [[] for _ in range(n)]
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            fr = int(r["frame"])
            if fr >= n:
                continue
            if r["cls"] == "skip":
                dets[fr] = None
                continue
            d = dict(cls=r["cls"], x=float(r["x"]), y=float(r["y"]),
                     heading_deg=float(r["heading_deg"]) if r["heading_deg"] else None,
                     score=float(r["score"]))
            if r["angle90"] == "hidden":
                d["angle90"] = None
            elif r["angle90"]:
                d["angle90"] = float(r["angle90"])
            if r.get("source"):
                d["source"] = r["source"]
            dets[fr].append(d)
    return dets


def video_signature(rec):
    """What identifies a recording's video for the caches: its size, frame count and length. Not
    its folder or file time, so a recording copied or moved elsewhere (and located again) keeps
    its saved guesses instead of redoing the slow first pass."""
    st = os.stat(rec.video)
    dur = (rec.times[-1] - rec.times[0]) if rec.times else 0.0
    return dict(size=st.st_size, frames=rec.n, seconds=round(dur, 2))


def same_video(stored, sig):
    """stored: a cache key's "video" entry: the signature above, or (caches written before it)
    [size, file time], which matches on the size (the frame count is checked separately)."""
    if isinstance(stored, dict):
        return stored == sig
    return isinstance(stored, list) and len(stored) == 2 and stored[0] == sig["size"]


def same_key(stored, key):
    """Two cache keys match: everything equal, the video compared with same_video()."""
    if not isinstance(stored, dict) or set(stored) != set(key):
        return False
    return all(same_video(stored[k], key[k]) if k == "video" else stored[k] == key[k] for k in key)


def motion_cache_key(rec, robot_px=None):
    return dict(kind="motion", version=CACHE_VERSION, video=video_signature(rec), robot_px=robot_px, n=rec.n)


def has_motion_cache(rec, cache_dir, robot_px=None):
    info_path = os.path.join(cache_dir, "motion_info.json")
    if not (os.path.exists(info_path) and os.path.exists(os.path.join(cache_dir, "motion_detections.csv"))):
        return False
    try:
        with open(info_path) as f:
            info = json.load(f)
        return same_key(info.get("key"), motion_cache_key(rec, robot_px)) and not size_failed_before(info)
    except ValueError:
        return False


def size_failed_before(info):
    """A motion cache whose object size could not be measured (the 30 px stand-in was used) by an older way
    of measuring it: worth one more try with the current one (motion.SIZE_METHOD; Oct 21: real footage)."""
    return info.get("robot_px_source") == "default" and int(info.get("size_method") or 1) < motion.SIZE_METHOD


def motion_guesses(rec, cache_dir, robot_px=None, progress=None, log=print, size_progress=None):
    """The motion finder's guesses for every frame of a Recording, from the cache in cache_dir
    or worked out (the first time; slow) and cached. Returns (guesses per frame, object size px).
    progress(done, total): optional. size_progress(done, total): optional, while it first measures
    how big the objects are (a few seconds on a long recording)."""
    os.makedirs(cache_dir, exist_ok=True)
    key = motion_cache_key(rec, robot_px)
    info_path = os.path.join(cache_dir, "motion_info.json")
    det_path = os.path.join(cache_dir, "motion_detections.csv")
    if has_motion_cache(rec, cache_dir, robot_px):
        with open(info_path) as f:
            info = json.load(f)
        log("Using saved motion guesses from labels/cache/.")
        return load_detections(det_path, rec.n), (robot_px or info["robot_px"])
    try:
        with open(info_path) as f:
            if size_failed_before(json.load(f)):
                log("The saved motion guesses used a stand-in object size (it could not be measured then); "
                    "measuring it again and finding the moving objects again (labels/cache/; your labels are unchanged).")
    except (OSError, ValueError):
        pass
    log("Finding moving objects in the recording (first time only) ...")
    dets, minfo = motion.detect_recording(rec.video, fps=rec.fps, robot_px=robot_px, times=rec.times,
                                          n_frames=rec.n, progress=progress, size_progress=size_progress)
    dets.extend([[] for _ in range(rec.n - len(dets))])
    info = dict(key=key, robot_px=minfo["robot_px"], robot_px_source=minfo["robot_px_source"],
                size_method=motion.SIZE_METHOD)
    log("Object size: %.1f px (%s)" % (minfo["robot_px"], minfo["robot_px_source"]))
    save_detections(det_path, dets)
    write_atomic(info_path, json.dumps(info, indent=1))
    return dets, minfo["robot_px"]


def map_classes(dets, class_map, classes):
    """Rename guesses' types with class_map (e.g. the motion finder's names -> the project's);
    a type the project doesn't have becomes "unknown" if it has that, else the guess is dropped."""
    out = []
    for ds in dets:
        if ds is None:
            out.append(None)
            continue
        keep = []
        for d in ds:
            c = (class_map or {}).get(d["cls"], d["cls"])
            if c not in classes:
                if "unknown" not in classes:
                    continue
                c = "unknown"
            if c != d["cls"]:
                d = dict(d, cls=c)
            keep.append(d)
        out.append(keep)
    return out


def load_model(module_path, weights, classes=None):
    """Import a detector module from a file and load its weights.
    Returns a function frame -> list of guesses. A class the project does not
    know becomes "unknown" if the project has that class, else it is dropped."""
    classes = classes or CLASSES
    module_path = os.path.abspath(module_path)
    if not os.path.exists(module_path):
        raise SystemExit("--model file not found: " + module_path)
    folder = os.path.dirname(module_path)
    if folder not in sys.path:
        sys.path.insert(0, folder)       # so the module can import files next to it
    spec = importlib.util.spec_from_file_location("label_detector", module_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for name in ("load", "predict"):
        if not hasattr(mod, name):
            raise SystemExit("%s has no %s() function (see docs/data-formats.md, 'Detector interface')"
                             % (module_path, name))
    model = mod.load(weights)

    def run(frame):
        out = []
        for d in mod.predict(model, frame) or []:
            cls = str(d.get("cls", "unknown"))
            if cls not in classes:
                if "unknown" not in classes:
                    continue
                cls = "unknown"
            h = d.get("heading_deg")
            out.append(dict(cls=cls, x=float(d["x"]), y=float(d["y"]),
                            heading_deg=None if h is None else float(h) % 360.0,
                            score=float(d.get("score", 0.5)), source="model"))
        return out
    return run


def run_model_over(video, n, run, step=1, progress=None):
    import cv2
    dets = []
    cap = cv2.VideoCapture(video)
    for f in range(n):
        if f % step:
            ok = cap.grab()
            dets.append(None)
        else:
            ok, img = cap.read()
            dets.append(run(img) if ok else [])
        if not ok:
            dets.extend([None] * (n - len(dets)))
            break
        if progress and f % 30 == 0:
            progress(f + 1, n)
    cap.release()
    if progress:
        progress(n, n)
    return dets


# ----------------------------------------------------------------------------
# The labelling session: all state, and the files
# ----------------------------------------------------------------------------

class Session:
    def __init__(self, folder, model=None, weights=None, model_step=1, robot_px=None, reselect=False,
                 low_threshold=0.6, log=print, classes=None, labels_dir=None, extra_dets=None, progress=None,
                 class_map=None, loose_classes=None, front_classes=None, area=None, stats_path=None, model_id=None,
                 part_of=None, look=True, type_settings=None, outside=None, refine=None):
        """classes: the object types this project labels (default CLASSES).
        labels_dir: where labels, frames and cached guesses go (default <recording>/labels;
        the recognition studio keeps them in its project folder instead).
        extra_dets: optional dict frame -> list of extra guesses (e.g. from another camera,
        source "camera"), merged with this recording's own guesses before tracking.
        progress: optional callback(what, done, total) for long steps (total 0: amount unknown).
        class_map: the motion finder's guess names -> this project's types (motion guesses only).
        loose_classes / front_classes: types the tracker lets move fast and end soon / types
        that have a front (defaults: the tracker's own).
        area: the camera's work area (a project.json cameras[...] entry, see workarea.py):
        guesses outside it or in an ignore spot are dropped before tracking.
        stats_path: the time-saving record (timing.py; default labels/stats.jsonl).
        model_id: the name of the guess source in that record ("none" = motion only).
        part_of: optional function frame -> "train", "val", "test" (the part of the training split
        the frame's time block goes to) or None (left out, at a block edge): "Most useful next"
        then spreads the first frames over the three parts, and the counts per part are shown."""
        self.part_of = part_of
        self.log = log
        self.classes = list(classes or CLASSES)
        self.progress_cb = progress
        self.extra_dets = extra_dets or {}
        self.class_map = class_map
        self.area_settings = dict(area) if area else None
        self.dropped_by_area = 0
        self._thumbs = {}
        self.rec = Recording(folder)
        self.labels_dir = os.path.abspath(labels_dir) if labels_dir else os.path.join(self.rec.folder, "labels")
        self.frames_dir = os.path.join(self.labels_dir, "frames")
        self.cache_dir = os.path.join(self.labels_dir, "cache")
        self.labels_path = os.path.join(self.labels_dir, "labels.csv")
        self.review_path = os.path.join(self.labels_dir, "review.json")
        os.makedirs(self.cache_dir, exist_ok=True)
        self.lock = threading.RLock()
        self.model, self.weights, self.model_step = model, weights, max(1, int(model_step))
        self.low_threshold = low_threshold
        self.save_error = None
        self.frames = {}        # frame -> dict(state, how, objects, auto_conf, changed, updated)
        self.review = {}
        self.dirty_from = None  # earliest frame whose anchor was added without re-tracking yet
        self.stats_path = stats_path or os.path.join(self.labels_dir, "stats.jsonl")
        self.model_id = model_id or ("none" if not model else os.path.splitext(os.path.basename(weights or model))[0])
        self.guess_kind = ("model" if model else "motion") + ("+camera" if extra_dets else "")
        self.inv = None          # the running or last Investigate scan
        self.bulk_undo = {}      # Investigate's "change all": token -> the frames as they were
        self._was_auto = {}      # frame -> how it was auto-accepted, while a person has it open (for undo)
        self.lost_on_load = []
        self.outside = dict(outside) if outside else None
        self.refined = None      # what refine() added (home visits), for the page
        self.checkins = None     # the "How often should I check?" measurement while it runs
        self._track_gen = 0      # bumped by every re-track: a background run that is not the newest is dropped
        self._bg_from = None     # a re-track from this frame is still running in the background
        self._settled = threading.Event()
        self._settled.set()

        self._load_review()
        self.robot_px = robot_px
        self.dets, self.source_name = self._guesses_per_frame()
        if not self.model:
            self.dets = map_classes(self.dets, self.class_map, self.classes)
        if self.extra_dets:
            for f, ds in self.extra_dets.items():
                if 0 <= f < len(self.dets) and ds:
                    if self.dets[f] is None:
                        self.dets[f] = []
                    self.dets[f] = list(self.dets[f]) + [dict(d) for d in ds]
        # the work area: applied to the guesses as they are built (cached guesses are not redone)
        self.area = workarea.Area.from_settings(self.area_settings, default_margin=float(self.robot_px))
        self.dets, self.dropped_by_area = workarea.filter_guesses(self.dets, self.area)
        if self.dropped_by_area:
            self.log("Work area: left out %d guesses outside it or in ignore spots." % self.dropped_by_area)
        self.n_outside = self._merge_outside()
        if self.n_outside:
            self.guess_kind += "+log"
        params = tracker.TrackerParams.from_types(type_settings)
        if loose_classes is not None:
            params.loose_classes = tuple(loose_classes)
        if front_classes is not None:
            params.front_classes = tuple(front_classes)
        self.params = params
        self.det_source = "model" if model else "motion"
        self.look = bool(look)
        self.frames_src = self._pictures() if self.look else None
        self.run = tracker.TrackRun(self.dets, self.rec.times, self.robot_px,
                                    (self.rec.width, self.rec.height), params=params,
                                    det_source=self.det_source, frames=self.frames_src)
        if self.outside and self.n_outside:
            vote = self.outside.get("split_vote")
            if vote is None and callable(self.outside.get("make_vote")):
                vote = self.outside["make_vote"](float(self.robot_px))
            self.run.split_vote = vote
        for spot in self.review.get("rejected_spots", []):      # what "Reject here" taught it before
            remember_rejected_spot(self, spot)
        self._load_labels()
        self._stage("Following the guesses from frame to frame ...")
        self.guesses, self.summary = self.run.run(self._anchors(), self._reserved_ids())
        if refine is not None:
            self._refine(refine)
        self._select(reselect)
        self._update_conf()
        self.save()

    # --- stage 2: pictures for the tracker, outside data, home visits, check-ins ------------------
    def _pictures(self):
        """The recording's pictures for the appearance chain (None if they can't be read: the tracker
        then works from the guesses alone, as before)."""
        try:
            import appearance
            return appearance.VideoFrames(self.rec.video, float(self.robot_px), n=self.rec.n)
        except Exception as e:          # never stops the labelling
            self.log("Following objects by their look is off for this recording: %s" % e)
            return None

    def _merge_outside(self):
        """Add the outside data's draft labels to the guesses (they are followed like any other guess)."""
        if not self.outside:
            return 0
        n = 0
        for f, ds in (self.outside.get("drafts") or {}).items():
            f = int(f)
            if not (0 <= f < len(self.dets)) or not ds:
                continue
            add = []
            for d in ds:
                if d.get("cls") not in self.classes:
                    continue
                add.append(dict(x=float(d["x"]), y=float(d["y"]), cls=d["cls"], score=float(d.get("score") or 0.5),
                                heading_deg=d.get("heading_deg") if d.get("heading_known") else None,
                                source=d.get("source") or "log", name=d.get("name")))
            if add:
                self.dets[f] = list(self.dets[f] or []) + add
                n += len(add)
        if n:
            self.log("Outside data: %d guesses from %s added." % (n, self.outside.get("label") or "outside data"))
        return n

    def _refine(self, refine):
        """Home visits (or anything else refine() finds after the first tracking): add its guesses and
        follow the guesses again from the first frame they touch."""
        try:
            out = refine(self)
        except Exception as e:          # never stops the labelling
            traceback.print_exc()
            self.log("Home positions were not used this time: %s" % e)
            return
        if not out:
            return
        added = []
        for f, ds in (out.get("dets") or {}).items():
            f = int(f)
            if 0 <= f < len(self.dets) and ds:
                have = list(self.dets[f] or [])
                for d in ds:
                    # replace_px: the guess replaces other guesses of its type this close (a home visit is
                    # exact); guesses of other types there (a hand reaching in) are kept
                    rp = d.get("replace_px")
                    if rp:
                        have = [e for e in have if e.get("cls") != d.get("cls") or
                                math.hypot(e["x"] - d["x"], e["y"] - d["y"]) > rp]
                self.dets[f] = have + [{k: v for k, v in d.items() if k != "replace_px"} for d in ds]
                added.append(f)
        self.refined = {k: v for k, v in out.items() if k != "dets"}
        if added:
            self._stage("Following the guesses again with the home visits ...")
            self.guesses, self.summary = self.run.run(self._anchors(), self._reserved_ids(), from_frame=min(added))
        if out.get("note"):
            self.log(out["note"])

    def names_at(self, f):
        """{obj: name} on frame f: the outside data's name of the draft nearest each guess (roll call)."""
        if not self.outside:
            return {}
        ds = [d for d in (self.outside.get("drafts") or {}).get(f, []) if d.get("name")]
        if not ds:
            return {}
        rows = self.frames[f]["objects"] if f in self.frames else self.guesses.get(f, [])
        out, lim = {}, 0.75 * float(self.robot_px)
        for o in rows:
            best = min(ds, key=lambda d: math.hypot(d["x"] - o["x"], d["y"] - o["y"]))
            if math.hypot(best["x"] - o["x"], best["y"] - o["y"]) <= lim:
                out[o["obj"]] = best["name"]
        return out

    def start_checkins(self):
        """"How often should I check?": replay the tracking between the person's labels (in the
        background) and measure how far the guesses drifted (tracker.measure_checkins). Saved as
        labels/checkins.json."""
        with self.lock:
            if self.checkins and self.checkins.get("status") == "running":
                return self.checkins_view()
            anchors = self._anchors()
            n = sum(1 for objs in anchors.values() if any(o["status"] != "rejected" for o in objs))
            if n < 2:
                raise ValueError("Needs at least 2 labelled frames in this recording; it has %d. Label a few more "
                                 "frames first (nothing was changed)." % n)
            job = dict(status="running", done=0, total=0, left="", result=None, error=None)
            self.checkins = job
        clock = RateClock(window=10.0)

        def progress(done, total):
            job.update(done=done, total=total, left=fmt_left(clock.update(done, total)))

        def work():
            frames = None
            try:
                frames = self._pictures() if self.look else None
                res = tracker.measure_checkins(self.dets, self.rec.times, self.robot_px, anchors,
                                               (self.rec.width, self.rec.height), params=self.params,
                                               det_source=self.det_source, frames=frames, progress=progress)
                res = dict(res, measured=now_iso(), labelled_frames=n, fps=self.rec.fps)
                res.pop("pairs", None)
                # JSON has no "infinity": the last time bin is "longer", and no limit is said in words
                for b in res.get("by_length") or []:
                    if b["up_to_s"] == float("inf"):
                        b["up_to_s"] = None
                if res.get("suggested_every_s") == float("inf"):
                    res["suggested_every_s"], res["no_limit"] = None, True
                write_atomic(os.path.join(self.labels_dir, "checkins.json"), json.dumps(res, indent=1))
                job.update(status="done", result=res)
            except Exception as e:
                traceback.print_exc()
                job.update(status="failed", error=str(e))
            finally:
                if frames is not None:
                    try:
                        frames.close()
                    except Exception:
                        pass
        threading.Thread(target=work, daemon=True).start()
        return self.checkins_view()

    def _labelled_count(self):
        return sum(1 for st in self.frames.values() if st["state"] in ("edited", "reviewed")
                   and any(o["status"] != "rejected" for o in st["objects"]))

    def checkins_view(self, restart_if_stale=False):
        """The running measurement, or the last saved one (labels/checkins.json), or nothing. A saved one
        made on fewer (or other) labels than there are now is marked stale (labelled_then, labelled_now);
        with restart_if_stale it is measured again at once (in the background)."""
        j = self.checkins
        if j and j["status"] == "running":
            return dict(j)
        if j and j["status"] == "failed":
            return dict(j)
        p = os.path.join(self.labels_dir, "checkins.json")
        if os.path.exists(p):
            try:
                with open(p) as f:
                    res = json.load(f)
            except ValueError:
                res = None
            if res is not None:
                now_n = self._labelled_count()
                then_n = res.get("labelled_frames")
                if then_n is not None and then_n != now_n:
                    if restart_if_stale and now_n >= 2:
                        try:
                            v = self.start_checkins()
                            return dict(v, again=True, labelled_then=then_n, labelled_now=now_n)
                        except ValueError:
                            pass
                    return dict(status="done", result=res, stale=True, labelled_then=then_n, labelled_now=now_n)
                return dict(status="done", result=res)
        return dict(status="none")

    # --- guesses ---------------------------------------------------------------
    def _stage(self, text):
        """A step whose length isn't known: say what is happening."""
        self.log(text)
        if self.progress_cb:
            self.progress_cb(text, 0, 0)

    def _progress(self, what):
        clock = RateClock()

        def cb(done, total):
            if self.progress_cb:
                self.progress_cb(what, done, total)
            left = fmt_left(clock.update(done, total))
            print("  %s: %d / %d frames%s    " % (what, done, total, (" (%s)" % left) if left else ""),
                  end="\r", flush=True)
            if done >= total:
                print()
        return cb

    def _guesses_per_frame(self):
        rec = self.rec
        st = os.stat(rec.video)
        if self.model:
            mst = os.stat(self.model)
            wst = os.stat(self.weights) if self.weights and os.path.exists(self.weights) else None
            key = dict(kind="model", version=CACHE_VERSION, video=video_signature(rec),
                       module=os.path.abspath(self.model), module_mtime=int(mst.st_mtime),
                       weights=os.path.abspath(self.weights) if self.weights else None,
                       weights_mtime=int(wst.st_mtime) if wst else None, step=self.model_step, n=rec.n)
            name = "model"
            source_name = "model: %s (%s)" % (os.path.basename(self.model),
                                              os.path.basename(self.weights) if self.weights else "no weights")
        else:
            dets, self.robot_px = motion_guesses(rec, self.cache_dir, self.robot_px, self._progress("motion"),
                                                 self.log, size_progress=self._progress("size"))
            return dets, "motion"
        info_path = os.path.join(self.cache_dir, name + "_info.json")
        det_path = os.path.join(self.cache_dir, name + "_detections.csv")
        if os.path.exists(info_path) and os.path.exists(det_path):
            with open(info_path) as f:
                info = json.load(f)
            if same_key(info.get("key"), key):
                self.log("Using saved %s guesses from labels/cache/." % name)
                if not self.robot_px:
                    self.robot_px = info["robot_px"]
                return load_detections(det_path, rec.n), source_name
        if not self.robot_px:
            self.robot_px = self._robot_size()
        self.log("Running the model on the recording (every %d. frame) ..." % self.model_step)
        run = load_model(self.model, self.weights, self.classes)
        dets = run_model_over(rec.video, rec.n, run, self.model_step, self._progress("model"))
        info = dict(key=key, robot_px=self.robot_px)
        save_detections(det_path, dets)
        write_atomic(info_path, json.dumps(info, indent=1))
        return dets, source_name

    def _robot_size(self):
        mi = os.path.join(self.cache_dir, "motion_info.json")
        if os.path.exists(mi):
            with open(mi) as f:
                return json.load(f)["robot_px"]
        est = motion.estimate_robot_size(self.rec.video, self.rec.n)
        return est or motion.Params().default_robot_px

    # --- saved state -------------------------------------------------------------
    def _load_review(self):
        if os.path.exists(self.review_path):
            try:
                with open(self.review_path) as f:
                    self.review = json.load(f)
            except ValueError:
                bad = self.review_path + ".broken-%d" % int(time.time())
                os.replace(self.review_path, bad)
                self.log("review.json could not be read; kept it as %s and starting it afresh" % bad)
                self.review = {}
        self.low_threshold = float(self.review.get("settings", {}).get("low_threshold", self.low_threshold))

    def _load_labels(self):
        """Rebuild what the user did from labels.csv + review.json."""
        states = self.review.get("frames", {})
        rows_by_frame = {}
        if os.path.exists(self.labels_path):
            with open(self.labels_path, newline="") as f:
                for r in csv.DictReader(f):
                    rows_by_frame.setdefault(int(r["frame"]), []).append(r)
        lost = []
        for fs, st in states.items():
            f = int(fs)
            objs, empty = [], False
            if st.get("state", "reviewed") in ("reviewed", "auto") and not rows_by_frame.get(f) and not st.get("empty"):
                # review.json says finished, but labels.csv holds nothing for it (no objects, and no
                # "none" row that an empty frame gets): its labels were lost, so it is labelled again
                lost.append(f)
                continue
            for r in rows_by_frame.get(f, []):
                if r["cls"] == "none":
                    empty = True
                    continue
                h = r["heading_deg"]
                objs.append(dict(obj=int(r["obj"]), cls=r["cls"], x=float(r["x_px"]), y=float(r["y_px"]),
                                 heading_deg=float(h) if h else None, heading_known=r["heading_known"] == "1",
                                 heading_guess=None, source=r["source"], status=_mem_status(r["status"]),
                                 score=float(r["score"]) if r["score"] else None))
            self.frames[f] = dict(state=st.get("state", "reviewed"), how=st.get("how", "user"), objects=objs,
                                  auto_conf=st.get("auto_conf"), changed=st.get("changed", False),
                                  updated=st.get("updated"), empty=empty or (not objs and bool(st.get("empty"))),
                                  same_as=st.get("same_as"), changed_all=st.get("changed_all"))
        if lost:
            self.log("%d frame%s saved as finished had no rows in labels.csv (frame %s); %s back to be labelled."
                     % (len(lost), "" if len(lost) == 1 else "s", ", ".join(str(f) for f in sorted(lost)[:20]),
                        "it is" if len(lost) == 1 else "they are"))
        self.lost_on_load = sorted(lost)
        # Frames with user rows in labels.csv that review.json does not know (it was lost).
        for f, rows in rows_by_frame.items():
            if f not in self.frames and any(r["status"] != "pending" for r in rows):
                was_auto = all(r["status"] == "auto" for r in rows)
                self.frames[f] = dict(state="auto" if was_auto else "reviewed", how="auto" if was_auto else "user",
                                      objects=[], auto_conf=None, changed=not was_auto, updated=None, empty=False)
                for r in rows:
                    if r["cls"] == "none":
                        self.frames[f]["empty"] = True
                        continue
                    h = r["heading_deg"]
                    self.frames[f]["objects"].append(dict(
                        obj=int(r["obj"]), cls=r["cls"], x=float(r["x_px"]), y=float(r["y_px"]),
                        heading_deg=float(h) if h else None, heading_known=r["heading_known"] == "1",
                        heading_guess=None, source=r["source"], status=_mem_status(r["status"]),
                        score=float(r["score"]) if r["score"] else None))
        if self.frames:
            n_rev = sum(1 for v in self.frames.values() if v["state"] in ("edited", "reviewed"))
            self.log("Continuing: %d frames already labelled." % n_rev)

    def _anchors(self):
        """The user's labels the tracker must follow (not auto-accepted frames)."""
        return {f: st["objects"] for f, st in self.frames.items() if st["state"] in ("edited", "reviewed")}

    def _reserved_ids(self):
        return {o["obj"] for st in self.frames.values() for o in st["objects"]}

    def next_obj_id(self):
        """A number no object in this recording has (guesses or labels)."""
        top = max([0, self.run.max_id] + [s["obj"] for s in self.summary] + list(self._reserved_ids()))
        return top + 1

    # --- frame selection ---------------------------------------------------------
    def _select(self, reselect):
        sel = self.review.get("selected")
        if sel and not reselect:
            self.selected = sorted(int(f) for f in sel)
            self.reasons = {int(k): v for k, v in self.review.get("reasons", {}).items()}
            if any(not os.path.exists(os.path.join(self.frames_dir, "%06d.jpg" % f)) for f in self.selected):
                self._stage("Saving the pictures of the picked frames ...")
            n = select_frames.extract(self.rec.video, self.selected, self.frames_dir)
            if n:
                self.log("Re-extracted %d missing frame pictures." % n)
            return
        self._stage("Picking frames to label ...")
        cands = select_frames.candidates(self.guesses, self.rec.n, self.rec.times, low_score=self.low_threshold)
        kept, why = select_frames.pick(self.rec.video, cands, self.guesses, self.rec.times, self.frames_dir,
                                       keep=set(self.frames), progress=self._progress("pick"))
        self.selected = kept
        self.reasons = why
        mins = max(self.rec.times[-1], 1e-9) / 60.0
        self.log("Picked %d frames (%.0f per minute of video)." % (len(kept), len(kept) / mins))

    # --- confidence --------------------------------------------------------------
    def _guess_conf(self, f):
        rows = self.guesses.get(f, [])
        reasons = set(self.reasons.get(f, []))
        if rows:
            c = min(r["score"] for r in rows)
        else:
            # Nothing guessed: suspicious if objects are guessed nearby in time,
            # or if nothing was found in the whole recording.
            span = int(5 * self.rec.fps)
            near = any(self.guesses.get(g) for g in range(max(0, f - span), min(self.rec.n, f + span + 1), 15))
            c = 0.5 if (near or not self.guesses) else 0.9
        if reasons & {"appear", "disappear", "close"}:
            c = min(c, 0.55)
        return round(c, 3)

    def _update_conf(self):
        self.conf = {f: self._guess_conf(f) for f in self.selected}

    def has_home_guess(self, f):
        """A home visit's guess is on frame f (shown for a person to check, never accepted unseen)."""
        return any(r.get("source") == "home" for r in self.guesses.get(f, ()))

    def conf_of(self, f):
        st = self.frames.get(f)
        if st and st.get("auto_conf") is not None:
            return st["auto_conf"]
        return self.conf.get(f, self._guess_conf(f))

    # --- views -------------------------------------------------------------------
    def guess_objects(self, f):
        out = []
        for r in self.guesses.get(f, []):
            hd, hk, hg = r["heading_deg"], r["heading_known"], r["heading_guess"]
            if hk and r["source"] in ("motion", "track"):
                # a heading worked out from motion is only a guess (robots reverse and spin):
                # shown dashed, saved as unknown unless the person sets it
                hd, hk, hg = None, False, (hd if hg is None else hg)
            out.append(dict(obj=r["obj"], cls=r["cls"], x=r["x"], y=r["y"], heading_deg=hd,
                            heading_known=hk, heading_guess=hg,
                            source=r["source"], status="pending", score=r["score"], close=bool(r.get("close"))))
        return out

    def guess_label(self):
        """Where the guesses come from, in plain words, for the page's header."""
        src = ("model %s" % self.model_id) if self.model else "motion (no model)"
        out = (self.outside or {}).get("label") if getattr(self, "n_outside", 0) else None
        return "Guesses from " + src + (" and the other camera" if self.extra_dets else "") + \
            (" and %s" % out if out else "")

    def frame_state(self, f):
        st = self.frames.get(f)
        return st["state"] if st else "pending"

    def frame_view(self, f):
        st = self.frames.get(f)
        if st:
            objects = [dict(o) for o in st["objects"]]
        else:
            objects = self.guess_objects(f)
        spot = self.review.get("spot_check", {}).get("sample", [])
        return dict(frame=f, t_wall=self.rec.t_wall[f] if f < len(self.rec.t_wall) else "",
                    time_s=round(self.rec.times[f] - self.rec.times[0], 3), state=self.frame_state(f),
                    how=st["how"] if st else None, objects=objects, conf=self.conf_of(f),
                    reasons=self.reasons.get(f, []), in_spot=f in spot, empty=bool(st and st.get("empty")),
                    next_obj_id=self.next_obj_id(), names={str(k): v for k, v in self.names_at(f).items()},
                    flags=[dict(key=k, text=FLAG_TEXT.get(k, OTHER_FLAG_TEXT)) for k in self.frame_flags(f)])

    def frame_flags(self, f):
        """What the tracker asks a person to look at on frame f, most urgent first (FLAG_TEXT order,
        then any other flag). HOOK (lane T): read from the tracker's output rows (a row with
        split_unresolved / chain_broken set), and from TrackRun.frame_flags(f) if the tracker has it."""
        found = set()
        for r in self.guesses.get(f, []):
            for k in FLAG_TEXT:
                if r.get(k):
                    found.add(k)
            for k in r.get("flags") or ():
                found.add(str(k))
        fn = getattr(self.run, "frame_flags", None)
        if callable(fn):
            try:
                found.update(str(k) for k in (fn(f) or ()))
            except Exception:           # never stops the list
                pass
        return [k for k in FLAG_TEXT if k in found] + sorted(k for k in found if k not in FLAG_TEXT)

    def _flag_rank(self, f):
        """0 for the most urgent flag, ...; None for a frame without flags."""
        fl = self.frame_flags(f)
        if not fl:
            return None
        keys = list(FLAG_TEXT)
        return keys.index(fl[0]) if fl[0] in keys else len(keys)

    def queue(self, mode):
        if mode == "next":
            # most useful first: frames the tracker asks a person to look at (two lookalikes it can't
            # tell apart, a broken chain), then frames nobody has finished, least confident guesses first
            # (ties: earliest first). Finished frames follow, so the list still shows them.
            todo = [f for f in self.selected if self.frame_state(f) in ("pending", "edited")]
            ranks = {f: self._flag_rank(f) for f in todo}
            flagged = sorted((f for f in todo if ranks[f] is not None), key=lambda f: (ranks[f], self.conf_of(f), f))
            todo = sorted((f for f in todo if ranks[f] is None), key=lambda f: (self.conf_of(f), f))
            if self.part_of:
                todo = self._spread(todo)
            todo = flagged + todo
            done = [f for f in self.selected if f not in set(todo)]
            frames = todo + done
        elif mode == "low":
            frames = [f for f in self.selected if self.conf_of(f) < self.low_threshold]
        elif mode == "spot":
            frames = sorted(self.review.get("spot_check", {}).get("sample", []))
        elif mode == "unsure":
            frames = self.unsure_frames()
        else:
            frames = list(self.selected)
        uns = set(self.unsure_frames())
        out = []
        for f in frames:
            q = dict(f=f, state=self.frame_state(f), conf=self.conf_of(f), unsure=f in uns)
            fl = self.frame_flags(f)
            if fl:
                q["flag"] = fl[0]
            out.append(q)
        return out

    def flagged_left(self):
        """Picked frames not finished yet that the tracker asks a person to look at."""
        return sum(1 for f in self.selected if self.frame_state(f) in ("pending", "edited") and self.frame_flags(f))

    # the training split's parts, and the share of the footage each gets (trainer.py)
    PARTS = ("train", "val", "test")
    PART_SHARE = dict(train=0.65, val=0.15, test=0.2)
    SPREAD_BAND = 0.1      # guesses this close in confidence count as equally useful

    def _part(self, f):
        try:
            return self.part_of(f)
        except Exception:          # never stops the labelling: the frame just isn't spread
            return "train"

    def part_counts(self):
        """Finished frames (by a person, or accepted) per part of the training split, not counting
        frames with an unsure object; "edge": frames at a block edge, which training leaves out."""
        if not self.part_of:
            return None
        out = dict(train=0, val=0, test=0, edge=0)
        for f, st in self.frames.items():
            if st["state"] not in ("reviewed", "auto") or any(o["status"] == "unsure" for o in st["objects"]):
                continue
            out[self._part(f) or "edge"] += 1
        return out

    def _spread(self, todo):
        """Reorder frames (least confident first) so that the first ones are spread over the training
        split's parts: among frames about as uncertain as the most uncertain one left (within
        SPREAD_BAND), take one from the part with the fewest finished frames for its share. A frame at
        a block edge (training leaves it out) comes only once nothing else is about as uncertain."""
        counts = self.part_counts() or {}
        lanes = {p: collections.deque() for p in self.PARTS}
        edge = collections.deque()
        for f in todo:                       # todo is sorted by (confidence, frame)
            p = self._part(f)
            (lanes[p] if p in lanes else edge).append(f)
        out = []
        while any(lanes.values()) or edge:
            heads = [self.conf_of(q[0]) for q in lanes.values() if q] + ([self.conf_of(edge[0])] if edge else [])
            low = min(heads)
            ok = [p for p in self.PARTS if lanes[p] and self.conf_of(lanes[p][0]) <= low + self.SPREAD_BAND]
            if not ok:                       # the edge frame is clearly the most uncertain one left
                out.append(edge.popleft())
                continue
            p = min(ok, key=lambda p: (counts.get(p, 0) / self.PART_SHARE[p], self.conf_of(lanes[p][0]),
                                       lanes[p][0]))
            out.append(lanes[p].popleft())
            counts[p] = counts.get(p, 0) + 1
        return out

    def area_view(self):
        """The work area as the page draws it (margin_px: the margin actually used)."""
        a = self.area
        return dict(work_area=[list(p) for p in a.polygon] if a.polygon else None, margin_px=a.margin,
                    ignore_spots=[list(s) for s in a.spots], dropped=self.dropped_by_area)

    def unsure_frames(self):
        """Frames holding an object marked unsure (kept out of training until resolved)."""
        return sorted(f for f, st in self.frames.items() if any(o["status"] == "unsure" for o in st["objects"]))

    def stats(self):
        sel = self.selected
        states = [self.frame_state(f) for f in sel]
        low = [f for f in sel if self.conf_of(f) < self.low_threshold]
        high_pending = [f for f in sel if self.frame_state(f) == "pending" and self.conf_of(f) >= self.low_threshold
                        and not self.has_home_guess(f)]
        spot = self.spot_result()
        return dict(selected=len(sel), reviewed=states.count("reviewed"), edited=states.count("edited"),
                    auto=states.count("auto"), pending=states.count("pending"), low=len(low),
                    low_left=sum(1 for f in low if self.frame_state(f) in ("pending", "edited")),
                    high_pending=len(high_pending), spot=spot, low_threshold=self.low_threshold,
                    unsure=len(self.unsure_frames()), parts=self.part_counts(), flagged_left=self.flagged_left(),
                    same=sum(1 for st in self.frames.values() if st["state"] == "auto" and st.get("how") == "same"),
                    save_error=self.save_error, tracking=self.tracking_view())

    # --- spot check ---------------------------------------------------------------
    def draw_spot(self, fraction=0.1, min_n=10, seed=None):
        """Random sample of the high-confidence frames nobody has looked at
        (not yet reviewed, or auto-accepted)."""
        self.settle()
        pool = [f for f in self.selected
                if self.frame_state(f) in ("pending", "auto") and self.conf_of(f) >= self.low_threshold]
        n = min(len(pool), max(min_n, int(math.ceil(fraction * len(pool)))))
        seed = int(time.time()) if seed is None else seed
        sample = sorted(random.Random(seed).sample(pool, n)) if n else []
        self.review["spot_check"] = dict(sample=sample, seed=seed, drawn=now_iso(), pool=len(pool),
                                         fraction=fraction, threshold=self.low_threshold)
        self.save()
        return sample

    def spot_result(self):
        sc = self.review.get("spot_check", {})
        sample = sc.get("sample", [])
        checked = [f for f in sample if self.frame_state(f) == "reviewed"]
        agree = [f for f in checked if not self.frames[f].get("changed")]
        n_obj = n_obj_changed = 0
        for f in checked:
            for o in self.frames[f]["objects"]:
                if o["source"] == "user":
                    n_obj_changed += 1          # a robot the guesses missed
                    n_obj += 1
                else:
                    n_obj += 1
                    n_obj_changed += o["status"] in ("corrected", "rejected", "unsure")
        lo, hi = wilson(len(agree), len(checked))
        return dict(sample=len(sample), checked=len(checked), agree=len(agree),
                    agreement=round(len(agree) / len(checked), 3) if checked else None,
                    range95=[round(lo, 3), round(hi, 3)] if checked else None,
                    objects=n_obj, objects_changed=n_obj_changed, pool=sc.get("pool"))

    # --- changes ------------------------------------------------------------------
    def _clean_objects(self, objects):
        out = []
        next_id = self.next_obj_id()
        seen = set()
        for o in objects:
            cls = o.get("cls")
            if cls not in self.classes:
                raise ValueError("unknown class %r" % cls)
            status = o.get("status", "corrected")
            if status not in STATUSES:
                raise ValueError("unknown status %r" % status)
            source = o.get("source", "user")
            if source not in SOURCES:
                raise ValueError("unknown source %r" % source)
            x, y = float(o["x"]), float(o["y"])
            if not (-1 <= x <= self.rec.width + 1 and -1 <= y <= self.rec.height + 1):
                raise ValueError("position outside the picture: %.1f, %.1f" % (x, y))
            obj = o.get("obj")
            if obj is None or obj == "" or int(obj) in seen:
                obj = next_id
                next_id += 1
            obj = int(obj)
            seen.add(obj)
            h = o.get("heading_deg")
            known = bool(o.get("heading_known")) and h is not None
            score = o.get("score")
            out.append(dict(obj=obj, cls=cls, x=round(x, 1), y=round(y, 1),
                            heading_deg=round(float(h) % 360.0, 1) if known else None,
                            heading_known=known,
                            heading_guess=None if known or o.get("heading_guess") is None
                            else round(float(o["heading_guess"]) % 360.0, 1),
                            source=source, status=status,
                            score=None if (source == "user" or score is None) else round(float(score), 3)))
        return out

    def _keep_numbers(self, f, objs):
        """An object labelled where an object of the same type was labelled before, at the same spot
        (within half an object size), keeps that object's number: a still object the guesses lost (or
        picked up again under a new number) is the same object, not a new one each time it is labelled.
        Only fresh numbers (on no other labelled frame) are changed, never to a number used on this
        frame, and not when the older number was followed away from the spot in between (another
        object of that type may have taken its place)."""
        if not objs:
            return objs
        here_guess = {r["obj"]: r for r in self.guesses.get(f, [])}
        labelled = {g: st for g, st in self.frames.items() if g != f and st["state"] in ("edited", "reviewed")}
        if not labelled:
            return objs
        used_elsewhere = {o["obj"] for st in labelled.values() for o in st["objects"]}
        taken = {o["obj"] for o in objs}
        lim = 0.5 * float(self.robot_px)
        order = sorted(labelled, key=lambda g: (abs(g - f), g > f))
        for o in objs:
            if o["status"] == "rejected" or o["obj"] in used_elsewhere:
                continue
            for g in order:
                hit = next((p for p in labelled[g]["objects"] if p["status"] != "rejected" and p["cls"] == o["cls"]
                            and math.hypot(p["x"] - o["x"], p["y"] - o["y"]) <= lim), None)
                if hit is None:
                    continue
                old = hit["obj"]
                elsewhere = here_guess.get(old)
                lo, hi = (g, f) if g < f else (f, g)
                left = any(math.hypot(r["x"] - o["x"], r["y"] - o["y"]) > lim
                           for h in range(lo + 1, hi) for r in self.guesses.get(h, ()) if r["obj"] == old)
                if old not in taken and not left and (
                        elsewhere is None or math.hypot(elsewhere["x"] - o["x"], elsewhere["y"] - o["y"]) <= lim):
                    taken.discard(o["obj"])
                    o["obj"] = old
                    taken.add(old)
                break
        return objs

    def _same_as_guesses(self, f, objects):
        g = {r["obj"]: r for r in self.guesses.get(f, [])}
        if len(objects) != len(g):
            return False
        for o in objects:
            r = g.get(o["obj"])
            if (r is None or o["status"] not in ("pending", "confirmed") or o["cls"] != r["cls"]
                    or abs(o["x"] - r["x"]) > 0.05 or abs(o["y"] - r["y"]) > 0.05):
                return False
        return True

    def save_frame(self, f, objects, state, how=None, timing_info=None):
        """The page sends the whole frame after every change. timing_info: sent with a frame the
        person finishes (shown, active_s, edits) for the time-saving record."""
        with self.lock:
            f = int(f)
            if f not in self.frames and f not in set(self.selected):
                raise ValueError("frame %d is not one of the frames picked for labelling" % f)
            if state not in ("pending", "edited", "reviewed", "auto"):
                raise ValueError("unknown frame state %r" % state)
            old = self.frames.get(f)
            old_anchor = old["objects"] if old and old["state"] in ("edited", "reviewed") else None
            if state == "pending":
                self.frames.pop(f, None)
                new_anchor = None
            else:
                objs = self._keep_numbers(f, self._clean_objects(objects))
                changed = any(o["status"] in ("corrected", "rejected", "unsure") or o["source"] == "user"
                              for o in objs)
                auto_conf = old["auto_conf"] if old and old.get("auto_conf") is not None else self.conf_of(f)
                empty = state == "reviewed" and not any(o["status"] != "rejected" for o in objs)
                same_as = None
                if state == "auto":          # undo back to an auto-accepted frame
                    # a same-picture frame goes back to being one (not a plain auto-accepted frame)
                    was = self._was_auto.get(f)
                    if how == "same" and was and was.get("how") == "same":
                        same_as = was.get("same_as")
                    else:
                        how = "auto"
                    changed = False
                    empty = not any(o["status"] != "rejected" for o in objs)
                elif old is not None and old["state"] == "auto":
                    self._was_auto[f] = dict(how=old.get("how"), same_as=old.get("same_as"))
                self.frames[f] = dict(state=state, how=how or (old or {}).get("how") or "user", objects=objs,
                                      auto_conf=auto_conf, changed=changed, updated=now_iso(), empty=empty,
                                      same_as=same_as)
                new_anchor = objs if state != "auto" else None
            updated = 0
            if new_anchor != old_anchor:
                if new_anchor is not None and old_anchor is None and self._same_as_guesses(f, new_anchor):
                    # Accepted unchanged: the guesses after it would hardly change.
                    # Note it, and re-track later together with the next real change.
                    self.dirty_from = f if self.dirty_from is None else min(self.dirty_from, f)
                else:
                    updated = self.retrack(f)
            if state == "reviewed" and timing_info is not None:
                st = self.frames[f]
                shown, active, edits = timing.clean_timing(timing_info)
                self._record(f, timing.outcome_of(st["objects"], st["changed"]), active, edits, shown, st["how"],
                             sum(1 for o in st["objects"] if o["status"] != "rejected"))
            elif old is not None and old["state"] in ("reviewed", "auto") and state not in ("reviewed", "auto"):
                self._record(f, "undone")
            same = []
            if state == "reviewed" and (old is None or old["state"] != "reviewed") and (how or "user") == "user":
                same = self.accept_same_picture(f)
            self.save()
            return dict(frame=self.frame_view(f), updated=updated, stats=self.stats(), same=same)

    # --- same-picture frames ----------------------------------------------------------
    def _thumb(self, f):
        """The small grey picture select_frames compares, from the saved JPEG (None if missing)."""
        if f in self._thumbs:
            return self._thumbs[f]
        import numpy as np
        import cv2
        th = None
        path = os.path.join(self.frames_dir, "%06d.jpg" % f)
        if os.path.exists(path):
            img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                th = select_frames._thumb(img)
        if len(self._thumbs) > 2000:
            self._thumbs.clear()
        self._thumbs[f] = th
        return th

    def same_picture_frames(self, f, window_s=SAME_PICTURE_S, level=None):
        """Picked frames nobody has looked at, within window_s of frame f, whose picture is
        near-identical to f's (the test select_frames uses) and whose guesses sit where f's
        accepted objects are (same count, same type, each within a small distance)."""
        import numpy as np
        level = select_frames.DUP_LEVEL if level is None else level
        st = self.frames.get(f)
        if not st:
            return []
        objs = [o for o in st["objects"] if o["status"] != "rejected"]
        if any(o["status"] == "unsure" for o in objs):
            return []
        th = self._thumb(f)
        if th is None:
            return []
        t = self.rec.times[f]
        tol = max(2.0, 0.15 * float(self.robot_px))
        out = []
        for g in self.selected:
            if g == f or abs(self.rec.times[g] - t) > window_s or self.frame_state(g) != "pending":
                continue
            gth = self._thumb(g)
            if gth is None or float(np.abs(gth - th).mean()) >= level:
                continue
            if _match_objects(objs, self.guess_objects(g), tol) is None:
                continue
            if not self._patches_same(f, g, objs):
                continue
            out.append(g)
        return out

    def _grey(self, f):
        """The saved picture of frame f, full size, grey and lightly blurred (None if missing)."""
        import numpy as np
        import cv2
        path = os.path.join(self.frames_dir, "%06d.jpg" % f)
        if not os.path.exists(path):
            return None
        img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None:
            return None
        return cv2.GaussianBlur(img.astype(np.float32), (3, 3), 0)

    def _patches_same(self, f, g, objs, level=PATCH_LEVEL):
        """Each accepted object's patch (about one object across) looks the same at full size in
        both frames: the small whole-picture test misses an object that turned in place."""
        if not objs:
            return True
        a, b = self._grey(f), self._grey(g)
        if a is None or b is None or a.shape != b.shape:
            return False
        h, w = a.shape
        half = max(6, int(round(0.75 * float(self.robot_px))))
        for o in objs:
            x, y = int(round(o["x"])), int(round(o["y"]))
            x0, x1, y0, y1 = max(0, x - half), min(w, x + half + 1), max(0, y - half), min(h, y + half + 1)
            if x1 - x0 < 3 or y1 - y0 < 3:
                continue
            pa, pb = a[y0:y1, x0:x1], b[y0:y1, x0:x1]
            if float(abs((pa - pa.mean()) - (pb - pb.mean())).mean()) >= level:
                return False
        return True

    def accept_same_picture(self, f):
        """Accept the same-picture frames of a frame the person just accepted (state auto,
        how "same"), with the accepted frame's objects. Returns the frames accepted."""
        frames = self.same_picture_frames(f)
        if not frames:
            return []
        acc = [o for o in self.frames[f]["objects"] if o["status"] != "rejected"]
        tol = max(2.0, 0.15 * float(self.robot_px))
        for g in frames:
            guesses = self.guess_objects(g)
            pairs = _match_objects(acc, guesses, tol)
            objs = []
            for o, gg in pairs:
                # the person's own position on the accepted frame (the picture is the same); the
                # heading is left unknown: the same picture does not prove the same heading
                objs.append(dict(obj=o["obj"], cls=o["cls"], x=o["x"], y=o["y"],
                                 heading_deg=None, heading_known=False,
                                 heading_guess=None, source=o["source"] if o.get("source") in SOURCES else "track",
                                 status="confirmed", score=o.get("score")))
            self.frames[g] = dict(state="auto", how="same", objects=objs, auto_conf=self.conf_of(g),
                                  changed=False, updated=now_iso(), empty=not objs, same_as=f)
            self._record(g, "same", how="same", n_objects=len(objs))
        self.log("Also accepted %d frames with the same picture as frame %d." % (len(frames), f))
        return frames

    def undo_same(self, frames):
        """Take back same-picture frames (only frames accepted that way)."""
        with self.lock:
            n = 0
            for g in frames:
                st = self.frames.get(int(g))
                if st and st["state"] == "auto" and st.get("how") == "same":
                    del self.frames[int(g)]
                    self._record(int(g), "undone")
                    n += 1
            if n:
                self.save()
            return n

    # Following the guesses again after a change: a short window right away (what the person looks at
    # next), the rest of a long recording in the background, so a save never holds the session for
    # seconds. A newer change replaces a background run that has not finished.
    SYNC_FRAMES = 750           # frames followed again right away (30 s at 25 frames/s)
    background_retrack = True

    def retrack(self, f):
        """Redo the tracking from frame f on. Returns how many selected,
        not yet labelled frames got different guesses (in the part followed again right away)."""
        start = f if self.dirty_from is None else min(f, self.dirty_from)
        if self._bg_from is not None:
            start = min(start, self._bg_from)
        before = {g: [(r["obj"], r["x"], r["y"], r["cls"]) for r in self.guesses.get(g, [])]
                  for g in self.selected if g > start and self.frame_state(g) == "pending"}
        anchors = {g: [dict(o) for o in objs] for g, objs in self._anchors().items()}
        reserved = self._reserved_ids()
        self._track_gen += 1
        gen = self._track_gen
        if not self.background_retrack or self.rec.n - start <= 2 * self.SYNC_FRAMES:
            self.guesses, self.summary = self.run.run(anchors, reserved, from_frame=start)
            self._bg_from = None
            self._settled.set()
        else:
            until = start + self.SYNC_FRAMES
            rows, _summ = self.run.run(anchors, reserved, from_frame=start, until=until)
            merged = {g: r for g, r in rows.items() if g < until}
            for g, r in self.guesses.items():
                if g >= until:
                    merged[g] = r
            self.guesses = merged
            self._bg_from = start
            self._settled.clear()
            clone = self.run.clone()
            threading.Thread(target=self._bg_track, args=(gen, clone, anchors, reserved, start), daemon=True).start()
        self.dirty_from = None
        self._update_conf()
        n = 0
        for g, old in before.items():
            new = [(r["obj"], r["x"], r["y"], r["cls"]) for r in self.guesses.get(g, [])]
            if len(new) != len(old) or any(a[0] != b[0] or a[3] != b[3] or math.hypot(a[1] - b[1], a[2] - b[2]) > 1.0
                                           for a, b in zip(sorted(old), sorted(new))):
                n += 1
        return n

    def _bg_track(self, gen, clone, anchors, reserved, start):
        """The rest of a re-track, in the background (see retrack). Kept only if no newer change came."""
        try:
            rows, summ = clone.run(anchors, reserved, from_frame=start, cancel=lambda: self._track_gen != gen)
        except Exception as e:          # never stops the labelling: the next change (or settle) runs it again
            traceback.print_exc()
            self.log("Following the guesses again in the background stopped: %s" % e)
            return
        if rows is None:
            return
        with self.lock:
            if self._track_gen != gen or getattr(self, "closed", False):
                return
            self.run.adopt(clone)
            self.guesses, self.summary = rows, summ
            self._bg_from = None
            self._update_conf()
            self.save()
            self._settled.set()

    def close(self):
        """This session is being replaced (the recording opens again, or another one): a re-track still
        running in the background is dropped, so it can never write this session's files afterwards."""
        with self.lock:
            self._track_gen += 1
            self._bg_from = None
            self.closed = True
            self._settled.set()

    def settle(self):
        """Finish a background re-track now (for bulk actions that need every frame's guesses up to
        date: accept the rest, spot check, Investigate). Call with or without the lock."""
        with self.lock:
            if self._bg_from is None:
                return False
            start = self._bg_from
            self._track_gen += 1
            self.guesses, self.summary = self.run.run(self._anchors(), self._reserved_ids(), from_frame=start)
            self._bg_from = None
            self._update_conf()
            self._settled.set()
            return True

    def wait_settled(self, timeout=None):
        """Wait (without holding the lock) until no re-track runs in the background. True if none."""
        return self._settled.wait(timeout)

    def tracking_view(self):
        """For the page: the guesses after this frame are still being followed again (or None)."""
        return None if self._bg_from is None else dict(from_frame=self._bg_from)

    def auto_accept(self):
        """Accept the guesses on every high-confidence frame nobody has looked at. Returns how many."""
        return len(self.auto_accept_frames())

    def undo_auto(self):
        """Take back every frame accepted by "Accept remaining high-confidence frames". Returns how many."""
        return len(self.undo_auto_frames())

    def auto_accept_frames(self, frames=None):
        """Accept the guesses on every high-confidence frame nobody has looked at. frames: only these
        (Shift+Z on the page puts back frames Z took back; still only frames nobody has looked at).
        Returns the frames accepted."""
        with self.lock:
            self.settle()
            n = 0
            done = []
            only = None if frames is None else {int(f) for f in frames}
            for f in self.selected:
                if self.frame_state(f) != "pending":
                    continue
                if only is None and self.conf_of(f) < self.low_threshold or only is not None and f not in only:
                    continue
                gobjs = self.guess_objects(f)
                if self.has_home_guess(f):
                    continue        # a home visit's guess is shown for a person to check, never accepted unseen
                objs = []
                for o in gobjs:
                    o["status"] = "confirmed"
                    o.pop("close", None)
                    if not o["heading_known"]:
                        o["heading_deg"] = None
                    objs.append(o)
                self.frames[f] = dict(state="auto", how="auto", objects=objs, auto_conf=self.conf_of(f),
                                      changed=False, updated=now_iso(), empty=not objs)
                self._record(f, "auto", how="auto", n_objects=len(objs))
                n += 1
                done.append(f)
            self.save()
            return done

    def undo_auto_frames(self, frames=None):
        """Take back the frames accepted by "Accept remaining high-confidence frames" (not the
        same-picture ones, which a person's accept vouches for). frames: only these (Z on the page
        takes back one accept). Returns the frames taken back."""
        with self.lock:
            only = None if frames is None else {int(f) for f in frames}
            # only frames still exactly as the auto-accept left them: never same-picture frames (a
            # person's accept vouches for them) or frames a person changed since (e.g. with change all)
            fs = [f for f, st in self.frames.items() if st["state"] == "auto" and st.get("how") == "auto"
                  and not st.get("changed") and (only is None or f in only)]
            for f in fs:
                del self.frames[f]
                self._record(f, "undone")
            self.save()
            return sorted(fs)

    # --- track timeline ---------------------------------------------------------------------
    def track_timeline(self, max_objects=80):
        """One lane per followed object across the recording: when it was followed (segments),
        where the following broke off for over a second (breaks), where it jumped further than an
        object can move between neighbouring frames (jumps), and the frames with a label."""
        with self.lock:
            by_obj = {}
            for f, rs in self.guesses.items():
                for r in rs:
                    by_obj.setdefault(r["obj"], []).append((f, r["x"], r["y"], r["cls"]))
            labelled = {}
            for f, st in self.frames.items():
                for o in st["objects"]:
                    if o["status"] != "rejected":
                        labelled.setdefault(o["obj"], set()).add(f)
            gap = max(2, int(round(self.rec.fps)))
            jump_px = 1.5 * float(self.robot_px)
            out = []
            for obj, rs in by_obj.items():
                rs.sort()
                segs, breaks, jumps = [], [], []
                a = prev = rs[0]
                for cur in rs[1:]:
                    df = cur[0] - prev[0]
                    if df > gap:
                        segs.append([a[0], prev[0]])
                        breaks.append(prev[0])
                        a = cur
                    elif math.hypot(cur[1] - prev[1], cur[2] - prev[2]) > jump_px * max(1, df) ** 0.5:
                        jumps.append(cur[0])
                    prev = cur
                segs.append([a[0], prev[0]])
                kinds = collections.Counter(r[3] for r in rs)
                out.append(dict(obj=obj, cls=kinds.most_common(1)[0][0], first=rs[0][0], last=rs[-1][0], frames=len(rs),
                                segments=segs, breaks=breaks, jumps=jumps[:200],
                                labelled=sorted(labelled.get(obj, ()))))
            out.sort(key=lambda o: -o["frames"])
            hidden = max(0, len(out) - max_objects)
            out = sorted(out[:max_objects], key=lambda o: (o["first"], o["obj"]))
            return dict(n_frames=self.rec.n, duration_s=round(self.rec.times[-1] - self.rec.times[0], 2),
                        objects=out, hidden=hidden, picked=self.selected)

    # --- time-saving record ---------------------------------------------------------------
    def _record(self, f, outcome, active_s=0.0, edits=0, shown=None, how=None, n_objects=None):
        """One line in the time-saving record (timing.py). Never stops the labelling."""
        try:
            timing.append(self.stats_path, dict(frame=int(f), shown=shown, saved=now_iso(), active_s=active_s,
                                                edits=edits, outcome=outcome, how=how, guesses=self.guess_kind,
                                                model=self.model_id, objects=n_objects))
        except OSError as e:
            self.log("Time record not written: %s" % e)

    # --- investigate a spot -----------------------------------------------------------------
    def spot_radius(self):
        """About one object: the circle Investigate looks at, and the size of an ignore spot."""
        return round(max(6.0, 0.7 * float(self.robot_px)), 1)

    def start_investigate(self, frame, x, y, r=None):
        """Scan the spot across the recording in the background (investigate.py)."""
        import investigate
        r = float(r) if r else self.spot_radius()
        x, y, frame = float(x), float(y), int(frame)
        if not (0 <= x <= self.rec.width and 0 <= y <= self.rec.height):
            raise ValueError("the spot is outside the picture")
        with self.lock:
            if self.inv and self.inv["status"] == "running":
                self.inv["stop"] = True
            self.settle()
            job = dict(status="running", frame=frame, x=x, y=y, r=r, done=0, total=0, left="", result=None,
                       error=None, stop=False, started=time.time())
            self.inv = job
        clock = RateClock(window=10.0)

        def progress(done, total):
            job.update(done=done, total=total, left=fmt_left(clock.update(done, total)))

        def work():
            try:
                res = investigate.scan(self.rec.video, self.rec.n, self.rec.times, x, y, r, frame, self.guesses,
                                       progress=progress, stop=lambda: job["stop"],
                                       size=(self.rec.width, self.rec.height))
                job.update(result=res, status="done")
            except InterruptedError:
                job.update(status="stopped")
            except Exception as e:      # shown in the page in plain words
                traceback.print_exc()
                job.update(status="failed", error=str(e))
        threading.Thread(target=work, daemon=True).start()
        return self.investigate_view()

    def investigate_view(self):
        j = self.inv
        if not j:
            return dict(status="none")
        out = {k: v for k, v in j.items() if k not in ("stop", "started")}
        out["labels"] = self.labels_at(j["x"], j["y"], j["r"])
        out["spot_radius"] = j["r"]
        return out

    def labels_at(self, x, y, r):
        """Every label already made at the spot: objects within r of (x, y) on frames a person or
        the auto-accept finished. Newest first is not needed: in frame order."""
        out = []
        t0 = self.rec.times[0]
        for f in sorted(self.frames):
            st = self.frames[f]
            for o in st["objects"]:
                if math.hypot(o["x"] - x, o["y"] - y) <= r:
                    out.append(dict(frame=f, obj=o["obj"], cls=o["cls"], status=o["status"], state=st["state"],
                                    how=st.get("how"), time_s=round(self.rec.times[f] - t0, 2),
                                    x=o["x"], y=o["y"]))
        return out

    def _plan_change(self, items, action, cls):
        """What "change all" would do: [(frame, object, new values), ...], in frame order."""
        want = {}
        for it in items or []:
            want.setdefault(int(it["frame"]), set()).add(int(it["obj"]))
        plan = []
        for f, objs in sorted(want.items()):
            st = self.frames.get(f)
            if not st:
                continue
            for o in st["objects"]:
                if o["obj"] not in objs:
                    continue
                if action == "type":
                    if o["status"] == "rejected" or o["cls"] == cls and o["status"] not in ("unsure",):
                        continue
                    new = dict(cls=cls, status="corrected")
                elif action == "reject":
                    if o["status"] == "rejected":
                        continue
                    new = dict(status="rejected")
                else:
                    if o["status"] == "unsure":
                        continue
                    new = dict(status="unsure")
                plan.append((f, o, new))
        return plan

    @staticmethod
    def _preview_rows(plan, limit=200):
        return [dict(frame=f, obj=o["obj"], cls=o["cls"], status=o["status"], new_cls=new.get("cls", o["cls"]),
                     new_status=new["status"]) for f, o, new in plan[:limit]]

    def _touched(self, st):
        """A frame a person just changed from Investigate (change all, Reject here)."""
        st["changed"] = True
        # its own time: "updated" stays the frame's last save by a person (the time-saving
        # estimates read the gaps between those)
        st["changed_all"] = now_iso()
        if st["state"] == "auto":
            # a person changed it: it is no longer a guess accepted without looking, so
            # "Undo auto-accept" leaves it alone
            st["state"], st["how"] = "reviewed", "change_all"
            st.pop("same_as", None)
        if st["state"] in ("reviewed", "auto"):
            st["empty"] = not any(o["status"] != "rejected" for o in st["objects"])

    def _keep_for_undo(self, before, spots=None):
        """One undo step for the page's Z: the frames as they were (None: the frame had no entry) and
        as they are now; spots: dict(remove=[...], add=[...]) of rejected spots that undoing changes.
        Returns its token."""
        self._bulk_n = getattr(self, "_bulk_n", 0) + 1
        token = "c%d" % self._bulk_n
        self.bulk_undo[token] = dict(before=before, after={f: json.dumps(self.frames.get(f), sort_keys=True) for f in before},
                                     spots=spots or dict(remove=[], add=[]))
        return token

    def change_at(self, items, action, cls=None, preview=False):
        """Investigate's "change all": items = [{frame, obj}, ...]; action "type" (to cls),
        "reject" or "unsure". One undo step: returns a token for undo_change().
        preview: change nothing, say what would change (changed, frames, items)."""
        if action not in ("type", "reject", "unsure"):
            raise ValueError("unknown change %r" % action)
        if action == "type" and cls not in self.classes:
            raise ValueError("unknown object type %r" % cls)
        with self.lock:
            plan = self._plan_change(items, action, cls)
            if preview:
                return dict(preview=True, changed=len(plan), frames=sorted({f for f, _, _ in plan}),
                            items=self._preview_rows(plan))
            before, n, first = {}, 0, None
            for f, o, new in plan:
                st = self.frames[f]
                if f not in before:
                    before[f] = json.loads(json.dumps(st))
                o.update(new)
                n += 1
            for f in before:
                st = self.frames[f]
                self._touched(st)
                if st["state"] in ("edited", "reviewed"):
                    first = f if first is None else min(first, f)
            token = self._keep_for_undo(before) if before else None
            updated = self.retrack(first) if first is not None else 0
            self.save()
            return dict(changed=n, frames=sorted(before), token=token, updated=updated, stats=self.stats())

    def undo_change(self, token):
        """Put the frames of one "change all" (or Reject here) back as they were. Returns, with the
        counts, a new token that puts them back again (the page's Shift+Z)."""
        with self.lock:
            got = self.bulk_undo.pop(token, None)
            if not got:
                raise ValueError("nothing to undo for that change (the studio was restarted?)")
            if isinstance(got, tuple):                       # an older form: (before, after)
                got = dict(before=got[0], after=got[1], spots=dict(remove=[], add=[]))
            before, after = got["before"], got["after"]
            first, done, kept, back = None, [], [], {}
            for fs, st in before.items():
                f = int(fs)
                now = self.frames.get(f)
                if json.dumps(now, sort_keys=True) != after.get(f):
                    kept.append(f)          # changed again since: keep the newer work
                    continue
                back[f] = json.loads(json.dumps(now))
                if now is not None and now["state"] in ("edited", "reviewed"):     # it was a label the tracker followed
                    first = f if first is None else min(first, f)
                if st is None:
                    self.frames.pop(f, None)
                else:
                    self.frames[f] = st
                done.append(f)
                if st is not None and st["state"] in ("edited", "reviewed"):
                    first = f if first is None else min(first, f)
            spots = got.get("spots") or {}
            told = False
            for sp in spots.get("remove", []):
                if sp in self.review.get("rejected_spots", []):
                    self.review["rejected_spots"].remove(sp)
                told = forget_rejected_spot(self, sp) or told
            for sp in spots.get("add", []):
                self.review.setdefault("rejected_spots", []).append(sp)
                told = remember_rejected_spot(self, sp) or told
            if told:          # the tracker's memory changed: follow the guesses again from the spot's frame
                f0 = min(sp["frame"] for sp in spots.get("remove", []) + spots.get("add", []))
                first = f0 if first is None else min(first, f0)
            token2 = self._keep_for_undo(back, dict(remove=spots.get("add", []), add=spots.get("remove", []))) \
                if (back or spots.get("remove") or spots.get("add")) else None
            updated = self.retrack(first) if first is not None else 0
            self.save()
            return dict(undone=len(done), frames=sorted(done), kept=sorted(kept), updated=updated, stats=self.stats(),
                        token=token2)

    # --- Reject here: one frame, and the tracker remembers the empty spot -------------------------
    def rejected_spots(self):
        return list(self.review.get("rejected_spots", []))

    def reject_here(self, frame, x, y, r=None, preview=False):
        """Investigate's "Reject here": reject the guesses at the spot (within r of x, y) on this one
        frame, and remember the empty spot so the tracker scores later guesses that look like it down
        hard (remember_rejected_spot). Never leaves the spot out of other frames. Objects a person
        added there are kept (Del removes them). One undo step (token for undo_change)."""
        f, x, y = int(frame), float(x), float(y)
        r = float(r) if r else self.spot_radius()
        with self.lock:
            st = self.frames.get(f)
            if st is None and f not in set(self.selected):
                raise ValueError("frame %d is not one of the frames picked for labelling" % f)
            objs = st["objects"] if st else self.guess_objects(f)
            near = [o for o in objs if o["status"] != "rejected" and math.hypot(o["x"] - x, o["y"] - y) <= r]
            hit = [o for o in near if o.get("source") != "user"]
            yours = [o["obj"] for o in near if o.get("source") == "user"]
            if preview:
                return dict(preview=True, frame=f, x=x, y=y, r=r, yours=yours,
                            items=[dict(frame=f, obj=o["obj"], cls=o["cls"], status=o["status"], new_cls=o["cls"],
                                        new_status="rejected") for o in hit])
            before = {f: json.loads(json.dumps(st)) if st else None}
            if hit:
                if st is None:
                    # a frame nobody has touched: it becomes "changed, not finished", like Del on a guess
                    clean = self._clean_objects([{k: v for k, v in o.items() if k != "close"} for o in objs])
                    st = dict(state="edited", how="user", objects=clean, auto_conf=self.conf_of(f), changed=True,
                              updated=now_iso(), empty=False)
                    self.frames[f] = st
                ids = {o["obj"] for o in hit}
                for o in st["objects"]:
                    if o["obj"] in ids and o.get("source") != "user" and o["status"] != "rejected":
                        o["status"] = "rejected"
                self._touched(st)
            spot = dict(frame=f, x=round(x, 1), y=round(y, 1), r=round(r, 1), saved=now_iso())
            self.review.setdefault("rejected_spots", []).append(spot)
            told = remember_rejected_spot(self, spot)
            token = self._keep_for_undo(before if hit else {}, dict(remove=[spot], add=[]))
            updated = 0
            if (hit and self.frames[f]["state"] in ("edited", "reviewed")) or told:
                updated = self.retrack(f)
            self.save()
            return dict(frame=f, rejected=sorted(o["obj"] for o in hit), yours=yours, token=token, spot=spot,
                        tracker_told=told, updated=updated, stats=self.stats())

    # --- a short clip around a frame (key C) -------------------------------------------------------
    def clip(self, f, seconds=CLIP_SECONDS, max_frames=CLIP_MAX_FRAMES, width=CLIP_WIDTH):
        """The video about `seconds` before and after frame f, as pictures to play over and over at
        their own pace: dict(frame, frames=[dict(frame, dt, picture)], interval_ms, before_s, after_s)."""
        import bisect
        import cv2
        import investigate
        f = int(f)
        if not 0 <= f < self.rec.n:
            raise ValueError("there is no frame %d in this recording" % f)
        t = self.rec.times
        lo = bisect.bisect_left(t, t[f] - seconds)
        hi = max(f, bisect.bisect_right(t, t[f] + seconds) - 1)
        want = list(range(lo, hi + 1))
        if len(want) > max_frames:
            pick = {int(round(lo + k * (hi - lo) / (max_frames - 1))) for k in range(max_frames)} | {f}
            want = sorted(pick)
        pics = {}

        def keep(g, img):
            h, w = img.shape[:2]
            if w > width:
                img = cv2.resize(img, (width, int(round(h * width / w))), interpolation=cv2.INTER_AREA)
            pics[g] = investigate._jpeg(img, 78)
        investigate.read_frames(self.rec.video, want, keep)
        got = [g for g in want if g in pics]
        if not got:
            raise ValueError("the video around frame %d could not be read" % f)
        span = (t[got[-1]] - t[got[0]]) if len(got) > 1 else 0.0
        return dict(frame=f, frames=[dict(frame=g, dt=round(t[g] - t[f], 2), picture=pics[g]) for g in got],
                    interval_ms=int(round(1000 * span / (len(got) - 1))) if len(got) > 1 else 100,
                    before_s=round(t[f] - t[got[0]], 2), after_s=round(t[got[-1]] - t[f], 2))

    def set_position(self, mode, f):
        with self.lock:
            self.review["position"] = dict(mode=mode, frame=int(f))
            self._write_review()

    def set_low_threshold(self, v):
        with self.lock:
            self.low_threshold = min(1.0, max(0.0, float(v)))
            self.save()

    # --- files ----------------------------------------------------------------------
    def label_rows(self):
        rows = []
        tw = self.rec.t_wall
        for f in sorted(set(self.selected) | set(self.frames)):
            t = tw[f] if f < len(tw) else ""
            st = self.frames.get(f)
            objs = st["objects"] if st else self.guess_objects(f)
            for o in sorted(objs, key=lambda o: o["obj"]):
                known = bool(o.get("heading_known")) and o.get("heading_deg") is not None
                status = o["status"]
                if st and st["state"] == "auto" and status == "confirmed":
                    status = "auto"          # accepted without a person looking at it
                rows.append([f, t, o["obj"], o["cls"], "%.1f" % o["x"], "%.1f" % o["y"],
                             "%.1f" % o["heading_deg"] if known else "", 1 if known else 0,
                             o["source"], status, "" if o.get("score") is None else "%.3f" % o["score"]])
            if st and st["state"] in ("reviewed", "auto") and not any(o["status"] != "rejected" for o in objs):
                rows.append([f, t, "", "none", "", "", "", 0,
                             "user" if st["state"] == "reviewed" else self.run.det_source,
                             "confirmed" if st["state"] == "reviewed" else "auto", ""])
        return rows

    def _write_review(self):
        r = self.review
        sel = set(self.selected)
        r.update(version=1, recording=self.rec.name, guess_source=self.source_name,
                 robot_px=round(float(self.robot_px), 2), n_frames=self.rec.n,
                 selected=self.selected, reasons={str(f): v for f, v in sorted(self.reasons.items()) if f in sel},
                 settings=dict(low_threshold=self.low_threshold), updated=now_iso())
        r["frames"] = {str(f): _review_entry(st) for f, st in sorted(self.frames.items())}
        r["summary"] = {k: v for k, v in self.stats().items() if k not in ("save_error", "parts", "flagged_left")}
        r["help"] = ("frames: state reviewed = checked by a person (how: user, or spot = in the spot check); "
                     "edited = changed but Enter not pressed yet; auto = accepted without looking because the "
                     "guesses were high-confidence (how: auto), or because the picture was the same as a frame "
                     "a person accepted (how: same, same_as = that frame); their rows in labels.csv say auto. "
                     "how change_all = changed from Investigate (\"change all\", or Reject here; changed_all = when). "
                     "rejected_spots = spots where a guess was rejected with Reject here (frame, x, y, r): the "
                     "tracker scores later guesses that look like the empty spot down; other frames are not changed. "
                     "empty = finished with nothing in it. "
                     "Pending rows in labels.csv are guesses nobody has checked; unsure rows are objects the "
                     "person could not identify yet (their frames are kept out of training and scoring).")
        write_atomic(self.review_path, json.dumps(r, indent=1))

    def save(self):
        """labels.csv first, then review.json. Keeps going if a file is locked."""
        try:
            lines = [",".join(LABEL_COLUMNS)]
            for row in self.label_rows():
                lines.append(",".join(str(v) for v in row))
            write_atomic(self.labels_path, "\n".join(lines) + "\n")
            self._write_review()
            self.save_error = None
        except PermissionError as e:
            self.save_error = str(e)
            self.log("NOT SAVED: " + str(e))


# ----------------------------------------------------------------------------
# Web server
# ----------------------------------------------------------------------------

def _review_entry(st):
    e = dict(state=st["state"], how=st["how"], changed=st["changed"], auto_conf=st["auto_conf"], updated=st["updated"])
    if st.get("same_as") is not None:
        e["same_as"] = st["same_as"]
    if st.get("empty") and st["state"] in ("reviewed", "auto"):
        e["empty"] = True              # finished with nothing in it (labels.csv has its "none" row)
    if st.get("changed_all"):
        e["changed_all"] = st["changed_all"]     # when Investigate's "change all" last changed it
    return e


def _match_objects(objs, guesses, tol):
    """Pair each object with a guess of the same type within tol px (nearest first). Returns the
    pairs, or None unless every object and every guess is paired."""
    if len(objs) != len(guesses):
        return None
    cand = sorted((math.hypot(o["x"] - g["x"], o["y"] - g["y"]), i, j)
                  for i, o in enumerate(objs) for j, g in enumerate(guesses) if o["cls"] == g["cls"])
    ui, uj, pairs = set(), set(), []
    for d, i, j in cand:
        if d > tol:
            break
        if i in ui or j in uj:
            continue
        ui.add(i)
        uj.add(j)
        pairs.append((objs[i], guesses[j]))
    return pairs if len(pairs) == len(objs) else None


def make_handler(session, extra_get=None, extra_post=None):
    """session: a Session, or a function returning the current one (the recognition
    studio switches recordings and models while the server runs; it returns None
    while a new one is loading). extra_get / extra_post: optional functions
    (handler, path, query_or_body) -> True if they answered the request."""
    page_path = os.path.join(HERE, "page.html")
    current = session if callable(session) else (lambda: session)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype="application/json"):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(n).decode() or "{}") if n else {}

        def _guard(self, fn):
            try:
                fn()
            except (ValueError, KeyError, TypeError) as e:
                self._send(400, dict(error=str(e)))
            except Exception as e:   # keep the server alive, show the problem in the page
                traceback.print_exc()
                self._send(500, dict(error="%s: %s" % (type(e).__name__, e)))

        def _local_host(self):
            """The Host header names this computer (stops a web page on another site reaching the
            labels through a DNS trick)."""
            return (self.headers.get("Host") or "").split(":")[0] in ("127.0.0.1", "localhost")

        def do_GET(self):
            if not self._local_host():
                self._send(403, dict(error="refused: open the page at http://127.0.0.1"))
                return
            self._guard(self._get)

        def do_POST(self):
            # Only this page may change labels: a web page open in another tab
            # cannot send JSON to 127.0.0.1 without the browser asking us first
            # (which we never allow), and the Host check stops DNS tricks.
            ctype = self.headers.get("Content-Type") or ""
            if not self._local_host() or not ctype.startswith("application/json"):
                self._send(403, dict(error="refused: changes must come from the labelling page"))
                return
            self._guard(self._post)

        def _get(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path in ("/", "/index.html"):
                with open(page_path, "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
                return
            if extra_get and extra_get(self, u.path, q):
                return
            s = current()
            if s is None:
                self._send(503, dict(error="loading", loading=True))
                return
            if u.path == "/api/info":
                with s.lock:
                    self._send(200, dict(recording=s.rec.name, folder=s.rec.folder, n_frames=s.rec.n,
                                         fps=s.rec.fps, width=s.rec.width, height=s.rec.height,
                                         robot_px=s.robot_px, classes=s.classes, guess_source=s.source_name,
                                         guess_label=s.guess_label(),
                                         position=s.review.get("position"), stats=s.stats(),
                                         labels_path=s.labels_path, area=s.area_view()))
            elif u.path == "/api/queue":
                mode = q.get("mode", ["all"])[0]
                with s.lock:
                    self._send(200, dict(mode=mode, frames=s.queue(mode), stats=s.stats()))
            elif u.path == "/api/tracking":
                # cheap: is a re-track still running in the background (the page's "following again" note)
                self._send(200, dict(tracking=s.tracking_view()))
            elif u.path == "/api/tracks":
                self._send(200, s.track_timeline())
            elif u.path == "/api/investigate":
                with s.lock:
                    self._send(200, s.investigate_view())
            elif u.path == "/api/frame":
                f = int(q["f"][0])
                with s.lock:
                    self._send(200, s.frame_view(f))
            elif u.path == "/api/clip":
                self._send(200, s.clip(int(q["f"][0])))
            elif u.path == "/api/checkins":
                self._send(200, s.checkins_view(restart_if_stale=True))
            elif u.path.startswith("/img/"):
                name = os.path.basename(u.path)
                path = os.path.join(s.frames_dir, name)
                if not name.endswith(".jpg") or not os.path.exists(path):
                    self._send(404, dict(error="no picture " + name))
                    return
                with open(path, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "max-age=3600")
                self.end_headers()
                self.wfile.write(data)
            else:
                self._send(404, dict(error="not found"))

        def _post(self):
            u = urlparse(self.path)
            b = self._body()
            if extra_post and extra_post(self, u.path, b):
                return
            s = current()
            if s is None:
                self._send(503, dict(error="loading", loading=True))
                return
            if u.path == "/api/frame":
                self._send(200, s.save_frame(b["frame"], b.get("objects", []), b.get("state", "edited"), b.get("how"),
                                             b.get("timing")))
            elif u.path == "/api/investigate":
                self._send(200, s.start_investigate(b["frame"], b["x"], b["y"], b.get("r")))
            elif u.path == "/api/investigate/stop":
                if s.inv:
                    s.inv["stop"] = True
                self._send(200, dict(ok=True))
            elif u.path == "/api/investigate/change":
                self._send(200, s.change_at(b.get("items") or [], b.get("action"), b.get("cls"), bool(b.get("preview"))))
            elif u.path == "/api/investigate/reject_here":
                self._send(200, s.reject_here(b["frame"], b["x"], b["y"], b.get("r"), bool(b.get("preview"))))
            elif u.path == "/api/investigate/undo":
                self._send(200, s.undo_change(b.get("token")))
            elif u.path == "/api/checkins":
                self._send(200, s.start_checkins())
            elif u.path == "/api/position":
                s.set_position(b.get("mode", "all"), b.get("frame", 0))
                self._send(200, dict(ok=True))
            elif u.path == "/api/auto_accept":
                fs = s.auto_accept_frames(b.get("frames"))
                self._send(200, dict(accepted=len(fs), frames=fs, stats=s.stats()))
            elif u.path == "/api/auto_undo":
                fs = s.undo_auto_frames(b.get("frames"))
                self._send(200, dict(undone=len(fs), frames=fs, stats=s.stats()))
            elif u.path == "/api/undo_same":
                n = s.undo_same(b.get("frames") or [])
                self._send(200, dict(undone=n, stats=s.stats()))
            elif u.path == "/api/spot/draw":
                with s.lock:
                    sample = s.draw_spot(float(b.get("fraction", 0.1)), int(b.get("min_n", 10)), b.get("seed"))
                    self._send(200, dict(sample=sample, stats=s.stats()))
            elif u.path == "/api/settings":
                s.set_low_threshold(b["low_threshold"])
                self._send(200, dict(stats=s.stats()))
            else:
                self._send(404, dict(error="not found"))

    return Handler


def serve(session, port=8765, open_browser=True, extra_get=None, extra_post=None):
    try:
        server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(session, extra_get, extra_post))
    except OSError:
        raise SystemExit("Port %d is busy (is the app already open in another window?). "
                         "Close that, or start with --port %d" % (port, port + 1))
    server.daemon_threads = True
    url = "http://127.0.0.1:%d/" % server.server_address[1]
    return server, url


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", help="recording folder (with video.avi, frames.csv, meta.json)")
    ap.add_argument("--model", help="detector module (.py) with load(path) and predict(model, frame)")
    ap.add_argument("--weights", help="file passed to the detector's load()")
    ap.add_argument("--model-step", type=int, default=1, help="run the model on every n-th frame (default 1)")
    ap.add_argument("--robot-px", type=float, default=None, help="robot size in pixels (default: measured)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--reselect", action="store_true",
                    help="pick the frames to label again (frames already labelled are kept)")
    ap.add_argument("--no-browser", action="store_true", help="do not open the browser")
    ap.add_argument("--no-look", action="store_true",
                    help="follow objects by the guesses only, not also by their look (quicker first start)")
    args = ap.parse_args()
    if args.weights and not args.model:
        ap.error("--weights needs --model")
    session = Session(args.recording, model=args.model, weights=args.weights, model_step=args.model_step,
                      robot_px=args.robot_px, reselect=args.reselect, look=not args.no_look)
    server, url = serve(session, args.port)
    st = session.stats()
    print("\nLabels: %s" % session.labels_path)
    print("%d frames to label, %d done. Open %s  (Ctrl+C stops; everything is already saved)"
          % (st["selected"], st["reviewed"], url))
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped. Labels are saved.")
    finally:
        server.server_close()


if __name__ == "__main__":
    sys.exit(main())
