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

CLASSES = ["alvik", "hand", "unknown", "machine"]
STATUSES = ["pending", "confirmed", "corrected", "rejected"]


def _mem_status(st):
    """labels.csv writes "auto" for rows of auto-accepted frames; in memory
    they are confirmed rows of a frame whose state is auto."""
    return "confirmed" if st == "auto" else st
SOURCES = ["motion", "track", "model", "user", "log", "camera"]
LABEL_COLUMNS = ["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg", "heading_known",
                 "source", "status", "score"]
CACHE_VERSION = 1


def now_iso():
    return dt.datetime.now().isoformat(timespec="seconds")


def write_atomic(path, text):
    """Write a file so that a crash never leaves half a file: write a temporary
    file, then swap it in. On Windows the swap fails while the file is open in
    another program (Excel), so it retries for a moment, then raises."""
    tmp = path + ".tmp"
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
    raise PermissionError("%s is open in another program (Excel?). Close it; your work is kept "
                          "and will be saved with the next change." % os.path.basename(path))


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
        if progress and f % 300 == 0:
            progress(f + 1, n)
    cap.release()
    return dets


# ----------------------------------------------------------------------------
# The labelling session: all state, and the files
# ----------------------------------------------------------------------------

class Session:
    def __init__(self, folder, model=None, weights=None, model_step=1, robot_px=None, reselect=False,
                 low_threshold=0.6, log=print, classes=None, labels_dir=None, extra_dets=None, progress=None):
        """classes: the object types this project labels (default CLASSES).
        labels_dir: where labels, frames and cached guesses go (default <recording>/labels;
        the recognition studio keeps them in its project folder instead).
        extra_dets: optional dict frame -> list of extra guesses (e.g. from another camera,
        source "camera"), merged with this recording's own guesses before tracking.
        progress: optional callback(what, done, total) for long steps."""
        self.log = log
        self.classes = list(classes or CLASSES)
        self.progress_cb = progress
        self.extra_dets = extra_dets or {}
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

        self._load_review()
        self.robot_px = robot_px
        self.dets, self.source_name = self._guesses_per_frame()
        if self.extra_dets:
            for f, ds in self.extra_dets.items():
                if 0 <= f < len(self.dets) and ds:
                    if self.dets[f] is None:
                        self.dets[f] = []
                    self.dets[f] = list(self.dets[f]) + [dict(d) for d in ds]
        self.run = tracker.TrackRun(self.dets, self.rec.times, self.robot_px,
                                    (self.rec.width, self.rec.height),
                                    det_source="model" if model else "motion")
        self._load_labels()
        self.log("Following the guesses from frame to frame ...")
        self.guesses, self.summary = self.run.run(self._anchors(), self._reserved_ids())
        self._select(reselect)
        self._update_conf()
        self.save()

    # --- guesses ---------------------------------------------------------------
    def _progress(self, what):
        start = time.time()

        def cb(done, total):
            if self.progress_cb:
                self.progress_cb(what, done, total)
            el = time.time() - start
            left = "" if done < 300 else " (about %.0f s left)" % (el / done * (total - done))
            print("  %s: %d / %d frames%s    " % (what, done, total, left), end="\r", flush=True)
            if done >= total:
                print()
        return cb

    def _guesses_per_frame(self):
        rec = self.rec
        st = os.stat(rec.video)
        if self.model:
            mst = os.stat(self.model)
            wst = os.stat(self.weights) if self.weights and os.path.exists(self.weights) else None
            key = dict(kind="model", version=CACHE_VERSION, video=[st.st_size, int(st.st_mtime)],
                       module=os.path.abspath(self.model), module_mtime=int(mst.st_mtime),
                       weights=os.path.abspath(self.weights) if self.weights else None,
                       weights_mtime=int(wst.st_mtime) if wst else None, step=self.model_step, n=rec.n)
            name = "model"
            source_name = "model: %s (%s)" % (os.path.basename(self.model),
                                              os.path.basename(self.weights) if self.weights else "no weights")
        else:
            key = dict(kind="motion", version=CACHE_VERSION, video=[st.st_size, int(st.st_mtime)],
                       robot_px=self.robot_px, n=rec.n)
            name = "motion"
            source_name = "motion"
        info_path = os.path.join(self.cache_dir, name + "_info.json")
        det_path = os.path.join(self.cache_dir, name + "_detections.csv")
        if os.path.exists(info_path) and os.path.exists(det_path):
            with open(info_path) as f:
                info = json.load(f)
            if info.get("key") == key:
                self.log("Using saved %s guesses from labels/cache/." % name)
                if not self.robot_px:
                    self.robot_px = info["robot_px"]
                return load_detections(det_path, rec.n), source_name
        if self.model:
            if not self.robot_px:
                self.robot_px = self._robot_size()
            self.log("Running the model on the recording (every %d. frame) ..." % self.model_step)
            run = load_model(self.model, self.weights, self.classes)
            dets = run_model_over(rec.video, rec.n, run, self.model_step, self._progress("model"))
            info = dict(key=key, robot_px=self.robot_px)
        else:
            self.log("Finding moving objects in the recording (first time only) ...")
            dets, minfo = motion.detect_recording(rec.video, fps=rec.fps, robot_px=self.robot_px, times=rec.times,
                                                  n_frames=rec.n, progress=self._progress("motion"))
            dets.extend([[] for _ in range(rec.n - len(dets))])
            self.robot_px = minfo["robot_px"]
            info = dict(key=key, robot_px=self.robot_px, robot_px_source=minfo["robot_px_source"])
            self.log("Robot size: %.1f px (%s)" % (self.robot_px, minfo["robot_px_source"]))
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
        for fs, st in states.items():
            f = int(fs)
            objs, empty = [], False
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
                                  updated=st.get("updated"), empty=empty)
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
            n = select_frames.extract(self.rec.video, self.selected, self.frames_dir)
            if n:
                self.log("Re-extracted %d missing frame pictures." % n)
            return
        self.log("Picking frames to label ...")
        cands = select_frames.candidates(self.guesses, self.rec.n, self.rec.times, low_score=self.low_threshold)
        kept, why = select_frames.pick(self.rec.video, cands, self.guesses, self.rec.times, self.frames_dir,
                                       keep=set(self.frames))
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

    def conf_of(self, f):
        st = self.frames.get(f)
        if st and st.get("auto_conf") is not None:
            return st["auto_conf"]
        return self.conf.get(f, self._guess_conf(f))

    # --- views -------------------------------------------------------------------
    def guess_objects(self, f):
        out = []
        for r in self.guesses.get(f, []):
            out.append(dict(obj=r["obj"], cls=r["cls"], x=r["x"], y=r["y"], heading_deg=r["heading_deg"],
                            heading_known=r["heading_known"], heading_guess=r["heading_guess"],
                            source=r["source"], status="pending", score=r["score"], close=bool(r.get("close"))))
        return out

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
                    next_obj_id=self.next_obj_id())

    def queue(self, mode):
        if mode == "next":
            # most useful first: frames nobody has finished, least confident guesses first
            # (ties: earliest first). Finished frames follow, so the list still shows them.
            todo = [f for f in self.selected if self.frame_state(f) in ("pending", "edited")]
            todo.sort(key=lambda f: (self.conf_of(f), f))
            done = [f for f in self.selected if f not in set(todo)]
            frames = todo + done
        elif mode == "low":
            frames = [f for f in self.selected if self.conf_of(f) < self.low_threshold]
        elif mode == "spot":
            frames = sorted(self.review.get("spot_check", {}).get("sample", []))
        else:
            frames = list(self.selected)
        return [dict(f=f, state=self.frame_state(f), conf=self.conf_of(f)) for f in frames]

    def stats(self):
        sel = self.selected
        states = [self.frame_state(f) for f in sel]
        low = [f for f in sel if self.conf_of(f) < self.low_threshold]
        high_pending = [f for f in sel if self.frame_state(f) == "pending" and self.conf_of(f) >= self.low_threshold]
        spot = self.spot_result()
        return dict(selected=len(sel), reviewed=states.count("reviewed"), edited=states.count("edited"),
                    auto=states.count("auto"), pending=states.count("pending"), low=len(low),
                    low_left=sum(1 for f in low if self.frame_state(f) in ("pending", "edited")),
                    high_pending=len(high_pending), spot=spot, low_threshold=self.low_threshold,
                    save_error=self.save_error)

    # --- spot check ---------------------------------------------------------------
    def draw_spot(self, fraction=0.1, min_n=10, seed=None):
        """Random sample of the high-confidence frames nobody has looked at
        (not yet reviewed, or auto-accepted)."""
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
                    n_obj_changed += o["status"] in ("corrected", "rejected")
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

    def save_frame(self, f, objects, state, how=None):
        """The page sends the whole frame after every change."""
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
                objs = self._clean_objects(objects)
                changed = any(o["status"] in ("corrected", "rejected") or o["source"] == "user" for o in objs)
                auto_conf = old["auto_conf"] if old and old.get("auto_conf") is not None else self.conf_of(f)
                empty = state == "reviewed" and not any(o["status"] != "rejected" for o in objs)
                if state == "auto":          # undo back to an auto-accepted frame
                    how, changed = "auto", False
                    empty = not any(o["status"] != "rejected" for o in objs)
                self.frames[f] = dict(state=state, how=how or (old or {}).get("how") or "user", objects=objs,
                                      auto_conf=auto_conf, changed=changed, updated=now_iso(), empty=empty)
                new_anchor = objs if state != "auto" else None
            updated = 0
            if new_anchor != old_anchor:
                if new_anchor is not None and old_anchor is None and self._same_as_guesses(f, new_anchor):
                    # Accepted unchanged: the guesses after it would hardly change.
                    # Note it, and re-track later together with the next real change.
                    self.dirty_from = f if self.dirty_from is None else min(self.dirty_from, f)
                else:
                    updated = self.retrack(f)
            self.save()
            return dict(frame=self.frame_view(f), updated=updated, stats=self.stats())

    def retrack(self, f):
        """Redo the tracking from frame f on. Returns how many selected,
        not yet labelled frames got different guesses."""
        start = f if self.dirty_from is None else min(f, self.dirty_from)
        before = {g: [(r["obj"], r["x"], r["y"], r["cls"]) for r in self.guesses.get(g, [])]
                  for g in self.selected if g > start and self.frame_state(g) == "pending"}
        self.guesses, self.summary = self.run.run(self._anchors(), self._reserved_ids(), from_frame=start)
        self.dirty_from = None
        self._update_conf()
        n = 0
        for g, old in before.items():
            new = [(r["obj"], r["x"], r["y"], r["cls"]) for r in self.guesses.get(g, [])]
            if len(new) != len(old) or any(a[0] != b[0] or a[3] != b[3] or math.hypot(a[1] - b[1], a[2] - b[2]) > 1.0
                                           for a, b in zip(sorted(old), sorted(new))):
                n += 1
        return n

    def auto_accept(self):
        """Accept the guesses on every high-confidence frame nobody has looked at."""
        with self.lock:
            n = 0
            for f in self.selected:
                if self.frame_state(f) != "pending" or self.conf_of(f) < self.low_threshold:
                    continue
                objs = []
                for o in self.guess_objects(f):
                    o["status"] = "confirmed"
                    o.pop("close", None)
                    if not o["heading_known"]:
                        o["heading_deg"] = None
                    objs.append(o)
                self.frames[f] = dict(state="auto", how="auto", objects=objs, auto_conf=self.conf_of(f),
                                      changed=False, updated=now_iso(), empty=not objs)
                n += 1
            self.save()
            return n

    def undo_auto(self):
        with self.lock:
            fs = [f for f, st in self.frames.items() if st["state"] == "auto"]
            for f in fs:
                del self.frames[f]
            self.save()
            return len(fs)

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
        r["frames"] = {str(f): dict(state=st["state"], how=st["how"], changed=st["changed"],
                                    auto_conf=st["auto_conf"], updated=st["updated"])
                       for f, st in sorted(self.frames.items())}
        r["summary"] = {k: v for k, v in self.stats().items() if k != "save_error"}
        r["help"] = ("frames: state reviewed = checked by a person (how: user, or spot = in the spot check); "
                     "edited = changed but Enter not pressed yet; auto = accepted without looking because the "
                     "guesses were high-confidence (their rows in labels.csv say auto). Pending rows in "
                     "labels.csv are guesses nobody has checked.")
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

        def do_GET(self):
            self._guard(self._get)

        def do_POST(self):
            # Only this page may change labels: a web page open in another tab
            # cannot send JSON to 127.0.0.1 without the browser asking us first
            # (which we never allow), and the Host check stops DNS tricks.
            host = (self.headers.get("Host") or "").split(":")[0]
            ctype = self.headers.get("Content-Type") or ""
            if host not in ("127.0.0.1", "localhost") or not ctype.startswith("application/json"):
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
                                         position=s.review.get("position"), stats=s.stats(),
                                         labels_path=s.labels_path))
            elif u.path == "/api/queue":
                mode = q.get("mode", ["all"])[0]
                with s.lock:
                    self._send(200, dict(mode=mode, frames=s.queue(mode), stats=s.stats()))
            elif u.path == "/api/frame":
                f = int(q["f"][0])
                with s.lock:
                    self._send(200, s.frame_view(f))
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
                self._send(200, s.save_frame(b["frame"], b.get("objects", []), b.get("state", "edited"), b.get("how")))
            elif u.path == "/api/position":
                s.set_position(b.get("mode", "all"), b.get("frame", 0))
                self._send(200, dict(ok=True))
            elif u.path == "/api/auto_accept":
                n = s.auto_accept()
                self._send(200, dict(accepted=n, stats=s.stats()))
            elif u.path == "/api/auto_undo":
                n = s.undo_auto()
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
    args = ap.parse_args()
    if args.weights and not args.model:
        ap.error("--weights needs --model")
    session = Session(args.recording, model=args.model, weights=args.weights, model_step=args.model_step,
                      robot_px=args.robot_px, reselect=args.reselect)
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
