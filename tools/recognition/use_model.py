#!/usr/bin/env python3
"""Maxfield IRL Tracking - Identify. Recognize. Label.

Use a trained model on its own, without the studio: follow the objects in a
recording, a video file or a live camera, give each object a steady number, and
write where every object is, frame by frame, to a table (a .csv file you can open
in Excel).

    python tools/recognition/use_model.py <what to watch> --project <project folder>
    python tools/recognition/use_model.py <what to watch> --onnx <model file>
    python tools/recognition/use_model.py                  (no arguments: asks you plain questions)

<what to watch> is one of
    a recording folder (the one with video.avi in it) or a video file
    a camera number, for example 0
    a network camera address, for example http://st1maxfini.local:8000
      (the Pi's preview address; /stream is added for you)
    a recording's name in the project (with --project)

Useful options
    --model v3            which model of the project (default: the one chosen in the studio,
                          else the best scored)
    --camera-name NAME    which of the project's cameras this is (work area and floor
                          positions come from it; found by itself for project recordings)
    --out file.csv        where to save the table (default: in the project's "tracks" folder)
    --page                also show the live picture in your web browser (on by default for
                          cameras); --no-page turns it off
    --seconds 60          stop after running this long (clock time, not video time)
    --max-frames 500      stop after that many frames (for part of a recording)
    --every 2             look at every 2nd frame only (faster; the objects are carried along)
    --robot-px 30         object size in pixels, if the project does not know it
    --roll-call           start with a roll call (on by default for a live camera with the page)
    --no-roll-call        do not
    --ask-below 0.5       ask for a fresh confirmation of an object whose score stays below this ...
    --ask-after 3         ... for this many seconds
    --outside plugin:path optional positions log (a robot arm's or conveyor's controller, a robot's
                          own log, any file of positions and headings on the shared clock) that can
                          name the objects at the roll call: csv:<positions.csv>, or alvik:<Alvik
                          robot log folder> (outside.py; needs the camera's floor calibration, made
                          in the studio: More, then Floor calibration)

Roll call: at the start the page shows the first steady picture with what the model sees, each
object numbered. One click on "Looks right" confirms; or fix it first (drag a circle, delete one,
add one, set its type and name). Tracking then starts from those. Without the page the same is
asked in this window. Named objects keep their names in the table. While it runs, each object
shows green, amber or red, when it was last confirmed and, if it is not green, why in plain words.
When an object's score stays low for a few seconds the page asks for a fresh confirmation of just
that object; the others carry on. Every action on the page says what changed and how to take it back.

The table has one row per object per frame:
    frame, t_wall, obj, cls, x_px, y_px, heading_deg, score, x_cm, y_cm, source, name, confirmed
x_cm, y_cm (position on the floor) are there only when the project has a floor
calibration for the camera. source says "model" when the object was found in that
frame and "track" when it was carried along from where it was last seen.
name and confirmed are there only when the roll call and confirmations are in use (they are
last, so older readers of the table are not disturbed). name is what the person called the
object. confirmed is "yes" while a person's confirmation still holds, "no" from the moment the
object was lost until it is confirmed again, and "auto" for objects no person has confirmed.
"""
import argparse
import csv
import json
import math
import os
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (os.path.join(TOOLS, "label"), os.path.join(TOOLS, "detector"), HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

PRODUCT = "Maxfield IRL Tracking"
TAGLINE = "Identify. Recognize. Label."
BASE_COLUMNS = ["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg", "score"]
CM_COLUMNS = ["x_cm", "y_cm"]
CHECK_COLUMNS = ["name", "confirmed"]        # only with the roll call / confirmations; always last
NEEDED = [("numpy", "numpy"), ("cv2", "opencv-python"), ("onnxruntime", "onnxruntime")]
DEFAULT_ROBOT_PX = 30.0


class UserError(Exception):
    """Something the person can fix; the message is in plain words."""


# --------------------------------------------------------------------------------------------
# Packages (the .bat runs this file straight away, so it installs what it needs, like the studio)
# --------------------------------------------------------------------------------------------

def missing_packages():
    import importlib
    out = []
    for mod, pip in NEEDED:
        try:
            importlib.import_module(mod)
        except Exception:
            out.append(pip)
    return out


def ensure_packages():
    todo = missing_packages()
    if not todo:
        return
    import subprocess
    print("First start: some packages are missing (%s). Installing them now; this needs the internet." % ", ".join(todo))
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade"] + todo)
    except subprocess.CalledProcessError as e:
        raise UserError("Installing failed (%s). Check the internet connection and try again." % e)
    still = missing_packages()
    if still:
        raise UserError("Still missing after installing: %s" % ", ".join(still))
    print("All packages installed.\n")


# --------------------------------------------------------------------------------------------
# Confidence colour
# --------------------------------------------------------------------------------------------

GREEN_SCORE, AMBER_SCORE, SETTLED_FRAMES = 0.6, 0.35, 15

LEGEND = [
    ("green", "Sure: found clearly in this picture and followed steadily."),
    ("amber", "Fairly sure: new, a little unclear, or out of sight for a moment."),
    ("red", "Not sure: check this one by eye."),
]
ASK_BELOW, ASK_AFTER_S, UNSEEN_NOTE_S, STEADY_FRAMES, STEADY_MAX = 0.5, 3.0, 1.0, 3, 45


def confidence_colour(score, seen, age_frames):
    """green / amber / red for one object, from its score, whether the model found it in this
    very frame (seen) and how many frames it has been followed (age_frames)."""
    if seen and score >= GREEN_SCORE and age_frames >= SETTLED_FRAMES:
        return "green"
    if score >= AMBER_SCORE:
        return "amber"
    return "red"


# --------------------------------------------------------------------------------------------
# Work area and ignore spots
# --------------------------------------------------------------------------------------------

def _workarea():
    import workarea
    return workarea


class Area(_workarea().Area):
    """Drops guesses outside the work area (plus a margin) or inside an ignore spot: the studio's
    own rule (tools/label/workarea.py), so the same guesses are kept here as in the studio.
    polygon: list of [x, y] image px (None or fewer than 3 points = the whole picture).
    spots: list of [x, y, r]."""

    def __init__(self, polygon=None, margin_px=0.0, spots=None):
        super().__init__(polygon, float(margin_px or 0.0), [s[:3] for s in (spots or []) if len(s) >= 3],
                         default_margin=0.0)

    def keep(self, x, y):
        return self.keeps(float(x), float(y))

    def filter(self, dets):
        return [d for d in dets if self.keep(d["x"], d["y"])]

    @classmethod
    def from_project(cls, proj, camera, default_margin_px):
        """The camera's work area as the studio stores it: project.json cameras[<camera name>], or
        cameras["default"] for recordings added without a camera name."""
        key = proj.camera_key(dict(camera=camera or ""))
        cams = proj.data.get("cameras") or {}
        cfg = cams.get(key) or {}
        margin = cfg.get("margin_px")
        return cls(cfg.get("work_area"), default_margin_px if margin is None else margin, cfg.get("ignore_spots"))


# --------------------------------------------------------------------------------------------
# Project: model, camera, object size
# --------------------------------------------------------------------------------------------

def load_project(folder):
    import project as projmod
    folder = os.path.abspath(os.path.expanduser(str(folder).strip().strip('"')))
    if not os.path.exists(os.path.join(folder, "project.json")):
        raise UserError("There is no project in %s (a project folder has a file called project.json in it)." % folder)
    return projmod.Project(folder)


def _model_file(proj, m):
    rel = m.get("onnx") or os.path.join("models", m["id"], "model.onnx")
    p = os.path.join(proj.folder, rel)
    return p if m.get("status", "ready") == "ready" and os.path.exists(p) else None


def ready_models(proj):
    """[(model entry, onnx path)] for the project's finished models, oldest first."""
    out = []
    for m in proj.models():
        p = _model_file(proj, m)
        if p:
            out.append((m, p))
    return out


def resolve_model(proj, model_id=None):
    """(model id, onnx path): the model asked for, else the one chosen in the studio, else the
    best scored, else the newest."""
    ready = ready_models(proj)
    if not ready:
        raise UserError("This project has no finished model yet. Train one in the studio first "
                        "(the \"Train new model\" button).")
    if model_id:
        for m, p in ready:
            if m["id"] == model_id:
                return m["id"], p
        raise UserError("This project has no finished model called %s. Finished models: %s."
                        % (model_id, ", ".join(m["id"] for m, _ in ready)))
    chosen = (proj.data.get("current") or {}).get("model")
    for m, p in ready:
        if m["id"] == chosen:
            return m["id"], p

    def f1(item):
        v = (item[0].get("metrics") or {}).get("f1")
        return v if isinstance(v, (int, float)) else -1.0
    best = max(ready, key=f1)
    if f1(best) >= 0:
        return best[0]["id"], best[1]
    return ready[-1][0]["id"], ready[-1][1]


def recording_camera(proj, path):
    """The project camera name of a recording folder, if the project knows that folder."""
    here = os.path.normcase(os.path.abspath(path))
    for r in proj.recordings():
        if os.path.normcase(os.path.abspath(r["path"])) == here:
            return r.get("camera") or ""
    return None


def find_recording(proj, name):
    """A recording of the project by its id or name -> its folder, or None."""
    for r in proj.recordings():
        if name in (r["id"], r.get("name")):
            return r["path"]
    return None


def pick_camera_name(proj, source_path, given):
    if given is not None:
        return given
    if source_path:
        c = recording_camera(proj, source_path)
        if c is not None:
            return c
    names = [n for n in (proj.data.get("cameras") or {}) if n != "default"]   # "default": no camera name
    for r in proj.recordings():
        if r.get("camera") and r["camera"] not in names:
            names.append(r["camera"])
    return names[0] if len(names) == 1 else ""


def camera_model_paths(proj, camera, folder=None):
    """Where a camera's floor calibration may be, in order (as the studio looks): the project's
    cameras/<camera name>.json, cameras/default.json for a recording without a camera name, then a
    camera.json or floor_calibration.json in the recording folder."""
    key = (camera or "").strip() or "default"
    out = [proj.camera_model_path(key)]
    if folder:
        out += [os.path.join(folder, "camera.json"), os.path.join(folder, "floor_calibration.json")]
    return [p for p in out if p]


def load_camera_model(proj, camera, folder=None, say=print):
    """Floor calibration of a project camera (cameras.CameraModel) or None. A recording without a camera
    name uses the "default" camera, as in the studio."""
    import cameras
    for p in camera_model_paths(proj, camera, folder):
        if not os.path.exists(p):
            continue
        try:
            return cameras.CameraModel.load(p)
        except Exception as e:
            say("The floor calibration %s could not be read (%s); trying the next place." % (p, e))
    return None


def robot_size(proj, override=None):
    """Object size in pixels: given, else set in the project, else measured by the studio, else 30."""
    if override:
        return float(override)
    for c in proj.classes:
        if c.get("has_front") and c.get("size_px"):
            return float(c["size_px"])
    vals = []
    for r in proj.recordings():
        rp = os.path.join(proj.labels_dir(r["id"]), "review.json")
        if os.path.exists(rp):
            try:
                with open(rp, encoding="utf-8") as f:
                    vals.append(float(json.load(f).get("robot_px")))
            except (ValueError, TypeError, OSError):
                pass
    vals = sorted(v for v in vals if v and v > 0)
    return vals[len(vals) // 2] if vals else DEFAULT_ROBOT_PX


# --------------------------------------------------------------------------------------------
# Sources: recording folder, video file, camera, network address
# --------------------------------------------------------------------------------------------

def classify_source(spec):
    """'recording' | 'video' | 'camera' | 'url' for what the person typed."""
    s = str(spec).strip().strip('"')
    if re.match(r"^\d+$", s):
        return "camera"
    if re.match(r"^(https?|rtsp|rtmp)://", s, re.I):
        return "url"
    if os.path.isdir(s):
        return "recording"
    if os.path.isfile(s):
        return "video"
    if re.match(r"^[A-Za-z0-9._-]+(\.local)?:\d{2,5}(/.*)?$", s):
        return "url"
    return None


def normalise_url(spec):
    """Add http:// and the picture stream path to a Pi preview address."""
    s = str(spec).strip().strip('"')
    if not re.match(r"^[a-z]+://", s, re.I):
        s = "http://" + s
    m = re.match(r"^(https?://[^/]+)(/?)$", s, re.I)
    if m:
        s = m.group(1) + "/stream"
    return s


class Source:
    """Frames one at a time: read() -> (index, t seconds, t_wall text, image) or None at the end."""

    live = False
    n = None
    fps = 30.0
    kind = ""
    label = ""
    folder = None

    def read(self):
        raise NotImplementedError

    def close(self):
        pass


class VideoSource(Source):
    """A recording folder (video.avi + frames.csv) or a plain video file."""

    def __init__(self, spec):
        import cv2
        spec = os.path.abspath(str(spec).strip().strip('"'))
        if os.path.isdir(spec):
            video = os.path.join(spec, "video.avi")
            if not os.path.exists(video):
                subs = sorted(d for d in os.listdir(spec) if os.path.exists(os.path.join(spec, d, "video.avi")))
                if subs:
                    raise UserError("%s holds several camera folders (%s). Give the folder of one camera."
                                    % (spec, ", ".join(subs)))
                raise UserError("There is no video.avi in %s." % spec)
            self.kind, self.folder = "recording", spec
        else:
            video, self.kind = spec, "video"
            side = os.path.dirname(spec)
            self.folder = side if os.path.exists(os.path.join(side, "frames.csv")) and spec.lower().endswith("video.avi") else None
        self.label = os.path.basename(self.folder or video) if self.folder else os.path.basename(video)
        self.cap = cv2.VideoCapture(video)
        if not self.cap.isOpened():
            raise UserError("The video %s could not be opened." % video)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.fps = fps if fps and fps > 1 else 30.0
        n = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        self.times, self.t_wall = [], []
        fp = os.path.join(self.folder, "frames.csv") if self.folder else None
        if fp and os.path.exists(fp):
            with open(fp, newline="", encoding="utf-8") as f:
                for r in csv.DictReader(f):
                    self.t_wall.append(r.get("t_wall", ""))
                    try:
                        self.times.append(float(r["t_mono"]))
                    except (KeyError, ValueError, TypeError):
                        pass
            if len(self.times) != len(self.t_wall):
                self.times = []
            if n <= 0 or abs(n - len(self.t_wall)) <= 2:
                n = len(self.t_wall) or n
        self.n = n or None
        self.idx = 0

    def read(self):
        ok, img = self.cap.read()
        if not ok:
            return None
        f = self.idx
        self.idx += 1
        t = self.times[f] if f < len(self.times) else f / self.fps
        tw = self.t_wall[f] if f < len(self.t_wall) else ""
        return f, t, tw, img

    def close(self):
        self.cap.release()


class LiveSource(Source):
    """A camera or network stream. A reader thread keeps the newest picture, so a model slower
    than the camera skips pictures instead of falling further and further behind."""

    live = True

    def __init__(self, spec, kind):
        import cv2
        self.kind = kind
        if kind == "camera":
            idx = int(spec)
            self.cap = cv2.VideoCapture(idx) if os.name == "nt" else cv2.VideoCapture(idx, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.label = "camera %d" % idx
            problem = "Camera %d could not be opened. Is it plugged in, and not used by another program?" % idx
        else:
            url = normalise_url(spec)
            self.cap = cv2.VideoCapture(url)
            self.label = url
            problem = ("The camera at %s could not be opened. Is it switched on and on the same network? "
                       "Can you see its picture in a web browser?" % url)
        if not self.cap.isOpened():
            raise UserError(problem)
        self._lock = threading.Condition()
        self._latest, self._seq, self._given, self._ended = None, 0, 0, False
        self._stop = False
        self._t0 = time.time()
        self._count = 0
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        fails = 0
        while not self._stop:
            ok, img = self.cap.read()
            if not ok:
                fails += 1
                if fails > 30:
                    break
                time.sleep(0.05)
                continue
            fails = 0
            with self._lock:
                self._latest = (time.time(), img)
                self._seq += 1
                self._lock.notify_all()
        with self._lock:
            self._ended = True
            self._lock.notify_all()

    def read(self, timeout=5.0):
        with self._lock:
            end = time.time() + timeout
            while self._seq == self._given and not self._ended:
                left = end - time.time()
                if left <= 0:
                    return None
                self._lock.wait(left)
            if self._seq == self._given:
                return None
            self._given = self._seq
            now, img = self._latest
        f = self._count
        self._count += 1
        return f, now - self._t0, "%.3f" % now, img

    def close(self):
        self._stop = True
        time.sleep(0.05)
        self.cap.release()


def open_source(spec):
    kind = classify_source(spec)
    if kind is None:
        raise UserError("I could not find \"%s\". Give a recording folder, a video file, a camera number "
                        "(like 0) or a camera address (like http://st1maxfini.local:8000)." % spec)
    if kind in ("recording", "video"):
        return VideoSource(spec)
    return LiveSource(spec, kind)


# --------------------------------------------------------------------------------------------
# The tracking pipeline: guesses -> work area -> steady numbers -> floor positions
# --------------------------------------------------------------------------------------------

class Pipeline:
    """One frame at a time: process(...) runs the detector (any function image -> list of
    dict(cls, x, y, heading_deg, score), such as infer.predict), drops guesses outside the
    work area, and tracks them. Offline, finished_rows() gives every row once all frames are in.

    With check=True (the roll call and confirmations) every object also gets a name, a
    confirmed state, "last confirmed N s ago" and a plain-words reason when it is not green;
    see judge(). A person's confirmations come in through queue() and are applied between frames."""

    def __init__(self, detector, robot_px=DEFAULT_ROBOT_PX, img_size=(640, 480), area=None, camera_model=None,
                 heights=None, classes=None, heading_classes=None, every=1, check=False,
                 ask_below=ASK_BELOW, ask_after_s=ASK_AFTER_S):
        import tracker
        self.detector = detector
        self.area = area or Area()
        self.cam = camera_model
        self.heights = heights or {}
        self.classes = list(classes) if classes else None
        self.heading_classes = set(heading_classes) if heading_classes is not None else None
        self.every = max(1, int(every))
        self.robot_px = float(robot_px)
        self.tk = tracker.Tracker(robot_px, img_size, det_source="model")
        self.t_wall = {}
        self.calls = 0
        self.check = bool(check)
        self.ask_below, self.ask_after_s = float(ask_below), float(ask_after_s)
        self.outside = None            # optional outside data (see load_outside)
        self.names = {}                # obj -> name the person gave
        self.state = {}                # obj -> dict(t_conf, f_conf, low_since, asked): a person confirmed it
        self.last_seen = {}            # obj -> t it was last found by the model
        self.judged = {}               # (frame, obj) -> "yes" / "no" / "auto", for finished_rows
        self.notes = []                # plain sentences for the person, taken by take_notes()
        self.undo = {}                 # obj -> how to take back the last confirmation / removal
        self._cmds = []
        self._lock = threading.Lock()
        self._asked_said = set()

    @property
    def columns(self):
        return BASE_COLUMNS + (CM_COLUMNS if self.cam is not None else []) + ["source"] + (
            CHECK_COLUMNS if self.check else [])

    def _clean(self, dets):
        out = []
        for d in dets or []:
            cls = str(d.get("cls", "unknown"))
            if self.classes is not None and cls not in self.classes:
                if "unknown" not in self.classes:
                    continue
                d = dict(d, cls="unknown", heading_deg=None)
            out.append(d)
        return self.area.filter(out)

    def look(self, image):
        """Run the detector once on a picture and keep what the work area allows."""
        self.calls += 1
        return self._clean(self.detector(image))

    # --- the person's confirmations ----------------------------------------------------------
    def queue(self, cmd):
        """cmd: dict(kind="confirm"|"remove"|"undo", obj=N, x=, y=). Applied before the next rows are made."""
        with self._lock:
            self._cmds.append(dict(cmd))

    def take_notes(self):
        with self._lock:
            out, self.notes = self.notes, []
        return out

    def _note(self, text):
        with self._lock:
            self.notes.append(text)

    def label(self, obj):
        n = self.names.get(obj)
        return "%s (#%d)" % (n, obj) if n else "#%d" % obj

    def start_from(self, objs, f, t):
        """Roll call result -> the anchor list for the tracker, and the person's names and
        confirmations. objs: dicts with obj, x, y, cls, name, heading_deg."""
        anchor, keep = [], set()
        for o in objs:
            oid = int(o["obj"])
            keep.add(oid)
            hd = o.get("heading_deg")
            anchor.append(dict(obj=oid, x=float(o["x"]), y=float(o["y"]), cls=o.get("cls") or "unknown",
                               heading_deg=hd, heading_known=hd is not None, status="confirmed"))
            if o.get("name"):
                self.names[oid] = str(o["name"])
            else:
                self.names.pop(oid, None)
            self.state[oid] = dict(t_conf=t, f_conf=f, low_since=None, asked=False)
            self.last_seen[oid] = t
        for oid in list(self.state):                    # objects the person left out are no longer followed
            if oid not in keep:
                self.state.pop(oid, None)
                self.names.pop(oid, None)
        return anchor

    def process(self, f, t, t_wall, image, dets=None, anchor=None):
        """Track one frame. Returns this frame's objects: dicts with the table columns plus
        colour (green/amber/red) and age. dets: this frame's guesses when already looked at (the
        roll call does that); anchor: the roll call's objects, to start tracking from."""
        self.t_wall[f] = t_wall
        if dets is None and (f % self.every == 0 or self.calls == 0):
            dets = self.look(image)
        rows = self.tk.step(f, t, dets, anchor=anchor)
        by_id = {tr.id: tr for tr in self.tk.tracks}
        rows = self._apply_commands(rows, by_id, f, t)
        out = []
        for r in rows:
            tr = by_id.get(r["obj"])
            self._fix_heading(r, tr)
            rec = self._record(r, t_wall)
            seen = r["source"] != "track"
            age = f - tr.f_first if tr is not None else 0
            rec["colour"] = confidence_colour(r["score"], seen, age)
            rec["age"] = age
            out.append(rec)
        if self.check:
            self.judge(out, f, t)
        return out

    def _fix_heading(self, r, tr):
        """The tracker only reports a heading for two fixed class names; use the project's own
        list of types that have a front instead."""
        ok = self.heading_classes is None or r["cls"] in self.heading_classes
        if not ok:
            r["heading_deg"] = None
        elif r["heading_deg"] is None and tr is not None and tr.h_status == "known" and tr.heading is not None:
            r["heading_deg"] = round(tr.heading, 1)

    def _record(self, r, t_wall=None):
        rec = dict(frame=r["frame"], t_wall=self.t_wall.get(r["frame"], "") if t_wall is None else t_wall,
                   obj=r["obj"], cls=r["cls"], x_px=r["x"], y_px=r["y"], heading_deg=r["heading_deg"],
                   score=r["score"], source="track" if r["source"] == "track" else "model")
        if self.cam is not None:
            z = self.heights.get(r["cls"], 0.0)
            x, y = self.cam.to_floor(r["x"], r["y"], z)
            ok = math.isfinite(x) and math.isfinite(y)
            rec["x_cm"], rec["y_cm"] = (x, y) if ok else (None, None)
        if self.check:
            rec["name"] = self.names.get(r["obj"], "")
            rec["confirmed"] = self.judged.get((r["frame"], r["obj"]), "auto")
        return rec

    # --- confidence: how sure, since when, and why not -----------------------------------------
    def judge(self, recs, f, t):
        """Add to each object: confirmed ("yes" / "no" / "auto"), last_confirmed_s, why (plain
        words), needs (a fresh confirmation is wanted) and name. An object a person confirmed
        goes to "no" the moment its score drops below ask_below; if it stays low for ask_after_s
        (or was already asked) the page asks for a fresh confirmation, and it stays "no" until
        one comes. A dip that mends by itself before then goes back to "yes"."""
        for rec in recs:
            oid = rec["obj"]
            if rec["source"] == "model":
                self.last_seen[oid] = t
            st = self.state.get(oid)
            unseen = t - self.last_seen.get(oid, t)
            low = rec["score"] < self.ask_below
            near = any(o is not rec and o["cls"] == rec["cls"]
                       and math.hypot(o["x_px"] - rec["x_px"], o["y_px"] - rec["y_px"]) < 1.5 * self.robot_px
                       for o in recs)
            why = []
            if unseen >= UNSEEN_NOTE_S:
                why.append("not seen for %d s" % int(unseen))
            if low:
                why.append("score low (%d%%)" % round(rec["score"] * 100))
            if near:
                why.append("two lookalikes close together")
            conf = "auto"
            rec["needs"] = False
            rec["last_confirmed_s"] = None
            if st is not None:
                rec["last_confirmed_s"] = max(0.0, t - st["t_conf"])
                if low:
                    if st["low_since"] is None:
                        st["low_since"] = t
                    if not st["asked"] and t - st["low_since"] >= self.ask_after_s:
                        st["asked"] = True
                elif st["low_since"] is not None and not st["asked"]:
                    st["low_since"] = None            # a dip that mended itself
                conf = "no" if (st["low_since"] is not None or st["asked"]) else "yes"
                rec["needs"] = bool(st["asked"])
                if st["asked"] and oid not in self._asked_said:
                    self._asked_said.add(oid)
                    self._note("%s may have been lost (%s). Asking you to confirm just this one; the others carry on. "
                               "Its rows in the table say confirmed = no until you do." % (
                                   self.label(oid), ", ".join(why) or "its score fell"))
                elif not st["asked"]:
                    self._asked_said.discard(oid)
            if not why:
                why.append("found clearly" if rec["colour"] == "green" else
                           "just started being followed" if rec["age"] < SETTLED_FRAMES else "fairly steady")
            if rec["needs"]:
                rec["colour"] = "red"
            rec["why"] = ", ".join(why)
            rec["confirmed"] = conf
            rec["name"] = self.names.get(oid, "")
            self.judged[(rec["frame"], oid)] = conf

    def asks(self, recs):
        """The objects waiting for a fresh confirmation, for the page."""
        return [dict(obj=r["obj"], name=r.get("name", ""), cls=r["cls"], x=r["x_px"], y=r["y_px"], why=r.get("why", ""),
                     last_confirmed_s=r.get("last_confirmed_s")) for r in recs if r.get("needs")]

    # --- applying what the person did on the page ----------------------------------------------
    _KEEP = ("x", "y", "vx", "vy", "f_last", "t_last", "miss_run", "last_score", "last_det_score", "user_seen",
             "cls_locked", "confirmed", "hits", "pos_hist", "hist", "conf_f")

    def _apply_commands(self, rows, by_id, f, t):
        with self._lock:
            cmds, self._cmds = self._cmds, []
        for c in cmds:
            oid = int(c.get("obj", -1))
            if c["kind"] == "confirm":
                tr = by_id.get(oid)
                if tr is None:
                    self._note("Object #%d is not being followed any more, so there was nothing to confirm." % oid)
                    continue
                # the whole track as it was (every field but its rows), so undo puts all of it back
                snap = tr.clone(with_rows=False)
                prev = dict(st=dict(self.state[oid]) if oid in self.state else None,
                            tr={a: v for a, v in vars(snap).items() if a != "rows"},
                            last_seen=self.last_seen.get(oid), asked_said=oid in self._asked_said,
                            reserved=int(oid) in self.tk.reserved)
                # what the tracker itself does for a person's label, for this one object
                self.tk.confirm(tr, f, t, c.get("x"), c.get("y"))
                self.state[oid] = dict(t_conf=t, f_conf=f, low_since=None, asked=False)
                self.last_seen[oid] = t
                self._asked_said.discard(oid)
                self.undo[oid] = ("confirm", prev)
                for r in rows:
                    if r["obj"] == oid:
                        r["x"], r["y"], r["score"] = round(tr.x, 1), round(tr.y, 1), 1.0
            elif c["kind"] == "remove":
                tr = by_id.get(oid)
                if tr is None:
                    continue
                self.tk.tracks.remove(tr)
                self.tk.finished.append(tr)
                self.undo[oid] = ("remove", dict(tr=tr, st=self.state.pop(oid, None)))
                rows = [r for r in rows if r["obj"] != oid]
            elif c["kind"] == "undo":
                what = self.undo.pop(oid, None)
                if what is None:
                    self._note("There is nothing to take back for %s." % self.label(oid))
                    continue
                kind, prev = what
                if kind == "confirm":
                    tr = by_id.get(oid)
                    if tr is not None:
                        for a, v in prev["tr"].items():
                            setattr(tr, a, v)
                    if prev["st"] is None:
                        self.state.pop(oid, None)
                    else:
                        prev["st"]["asked"] = bool(prev["st"].get("asked"))
                        self.state[oid] = prev["st"]
                    if prev.get("last_seen") is None:
                        self.last_seen.pop(oid, None)
                    else:
                        self.last_seen[oid] = prev["last_seen"]
                    if prev.get("asked_said"):
                        self._asked_said.add(oid)
                    if prev.get("reserved") is False:
                        self.tk.reserved.discard(int(oid))
                else:
                    tr = prev["tr"]
                    if tr in self.tk.finished:
                        self.tk.finished.remove(tr)
                    if tr not in self.tk.tracks:
                        tr.f_last, tr.t_last = f, t
                        self.tk.tracks.append(tr)
                    if prev["st"] is not None:
                        self.state[oid] = prev["st"]
                self._note("%s is back as it was before." % self.label(oid))
        return rows

    def finished_rows(self):
        """Every confirmed object's rows over the whole run, in frame order (objects that were only
        confirmed a few frames after they appeared get those first frames filled in)."""
        rows = []
        for f, rs in sorted(self.tk.all_rows().items()):
            for r in rs:
                rows.append(self._record(r))
        return rows


def csv_line(rec, columns):
    out = []
    for c in columns:
        v = rec.get(c)
        if v is None:
            out.append("")
        elif c in ("x_px", "y_px", "x_cm", "y_cm"):
            out.append("%.1f" % v)
        elif c == "score":
            out.append("%.3f" % v)
        elif c == "heading_deg":
            out.append("%.1f" % v)
        else:
            out.append(str(v))
    return out


def heights_for(proj):
    """Height above the floor of each type's marked point; types whose height varies use the floor."""
    out = {}
    for c in proj.classes:
        varies = c.get("height_varies", c.get("varies"))
        out[c["name"]] = 0.0 if varies else float(c.get("height_cm") or 0.0)
    return out


# --------------------------------------------------------------------------------------------
# The live page
# --------------------------------------------------------------------------------------------

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Maxfield IRL Tracking</title>
<style>
  :root { --bg:#14161a; --panel:#1c1f24; --panel2:#23272e; --line:#31363f; --text:#e7e9ec; --dim:#9aa1ab;
          --accent:#6cb6ff; --green:#3ddc97; --amber:#ffb347; --red:#ff6b6b; }
  * { box-sizing: border-box; }
  html, body { margin:0; height:100%; background:var(--bg); color:var(--text);
    font:14px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif; }
  body { display:grid; grid-template-rows:auto 1fr; overflow:hidden; }
  header { display:flex; align-items:baseline; gap:14px; padding:8px 14px; background:var(--panel);
    border-bottom:1px solid var(--line); flex-wrap:wrap; }
  header h1 { font-size:15px; margin:0; font-weight:600; }
  header .tag { color:var(--dim); font-size:11px; letter-spacing:.04em; }
  header .src { color:var(--dim); font-size:12px; margin-left:auto; }
  main { display:grid; grid-template-columns:1fr 340px; min-height:0; }
  #stage { padding:10px; display:flex; flex-direction:column; align-items:center; justify-content:center; gap:8px;
    min-width:0; min-height:0; position:relative; }
  canvas { background:#000; border-radius:4px; max-width:100%; max-height:calc(100% - 78px); }
  aside { background:var(--panel); border-left:1px solid var(--line); overflow-y:auto; padding:12px; }
  aside h2 { font-size:12px; text-transform:uppercase; letter-spacing:.06em; color:var(--dim); margin:16px 0 6px; }
  aside h2:first-child { margin-top:0; }
  .fps { font-size:30px; font-weight:600; } .fps small { font-size:13px; color:var(--dim); font-weight:400; }
  .row { display:grid; grid-template-columns:12px 1fr auto; gap:2px 8px; align-items:center; padding:5px 7px;
    border-radius:6px; background:var(--panel2); margin-bottom:4px; font-size:13px; }
  .row .why { grid-column:2 / 4; color:var(--dim); font-size:12px; }
  .dot { width:12px; height:12px; border-radius:50%; }
  .green { background:var(--green); } .amber { background:var(--amber); } .red { background:var(--red); }
  .muted { color:var(--dim); font-size:12px; }
  .legend { display:grid; grid-template-columns:12px 1fr; gap:6px 8px; font-size:12.5px; align-items:start; }
  .legend .dot { margin-top:3px; }
  .bar { height:10px; background:var(--panel2); border-radius:5px; overflow:hidden; margin:6px 0 2px; }
  .bar span { display:block; height:100%; background:var(--accent); width:0; }
  button { padding:8px 10px; background:var(--panel2); color:var(--text); border:1px solid var(--line);
    border-radius:6px; cursor:pointer; font:inherit; }
  button:hover { border-color:var(--accent); } button:disabled { opacity:.45; cursor:default; }
  button.go { background:#1f4d3a; border-color:#2f8a63; font-weight:600; width:100%; margin-top:8px; }
  button.stop { width:100%; margin-top:14px; padding:10px; background:#5a2326; border-color:#8a3a3e; font-weight:600; }
  button.stop:hover { border-color:var(--red); }
  input[type=text], select { background:#14161a; color:var(--text); border:1px solid var(--line); border-radius:4px;
    padding:3px 5px; font:inherit; font-size:12.5px; min-width:0; }
  #msg { margin-top:10px; color:var(--accent); }
  .panel { background:var(--panel2); border:1px solid var(--line); border-radius:8px; padding:10px; margin-bottom:10px; }
  .panel.roll { border-color:var(--accent); }
  .panel.ask { border-color:var(--red); }
  .panel p { margin:0 0 6px; } .panel .btns { display:flex; flex-wrap:wrap; gap:6px; margin-top:6px; }
  .rollrow { display:grid; grid-template-columns:22px 1fr 1fr auto; gap:4px; align-items:center; margin-bottom:5px; }
  /* messages have their own strip under the picture: they never cover it (the newest two; all are in the list) */
  #toasts { flex:0 0 70px; width:100%; display:flex; flex-direction:column; justify-content:flex-end; gap:4px;
    overflow:hidden; font-size:13px; }
  #toasts .toast { padding:5px 10px; }
  .toast { background:#243447; border:1px solid var(--accent); border-radius:8px; padding:8px 12px; transition:opacity 1.2s;
    box-shadow:0 2px 8px #0008; }
  .toast.fade { opacity:0; }
  .toast button { margin-left:8px; padding:2px 8px; }
  #feed { font-size:12px; color:var(--dim); padding-left:16px; margin:0; }
</style></head><body>
<header><h1>Maxfield IRL Tracking</h1><span class="tag">Identify. Recognize. Label.</span><span class="src" id="src"></span></header>
<main>
  <div id="stage"><canvas id="view" width="640" height="480"></canvas><div id="toasts"></div></div>
  <aside>
    <div id="rollbox" class="panel roll" style="display:none">
      <p><b>Roll call.</b> This is what the model sees, each object numbered. Keep the objects where they are until you press the green button.</p>
      <div id="rolllist"></div>
      <div class="btns"><button id="addbtn">Add a circle</button><button id="rundo">Undo (Z)</button></div>
      <p class="muted" style="margin-top:8px">Drag a circle to move it. Pick a type and give it a name if you like; named objects keep their names in the table.</p>
      <button class="go" id="rgo">Looks right: start tracking</button>
    </div>
    <h2>Speed</h2>
    <div class="fps"><span id="fps">-</span> <small>frames per second</small></div>
    <div class="muted" id="count"></div>
    <div id="progress" style="display:none"><div class="bar"><span id="pbar"></span></div><div class="muted" id="ptxt"></div></div>
    <div id="asks"></div>
    <h2>Objects in this picture</h2>
    <div id="objs"><div class="muted">Waiting for the first picture ...</div></div>
    <div class="btns" style="margin-top:6px"><button id="redo">Redo the roll call</button></div>
    <h2>What the colours mean</h2>
    <div class="legend" id="legend"></div>
    <h2>What just happened</h2>
    <ul id="feed"><li>Nothing yet.</li></ul>
    <button class="stop" id="stop">Stop</button>
    <div id="msg"></div>
  </aside>
</main>
<script>
const COL = {green:"#3ddc97", amber:"#ffb347", red:"#ff6b6b"};
const legend = __LEGEND__;
document.getElementById("legend").innerHTML = legend.map(l =>
  '<span class="dot '+l[0]+'"></span><span>'+l[1]+'</span>').join("");
const cv = document.getElementById("view"), cx = cv.getContext("2d");
const $ = id => document.getElementById(id);
let since = -1, done = false, img = new Image(), last = null;
let R = null;            // the roll call being edited: {id, objs, classes, undo[], add, drag}
let pick = null;         // an object waiting for a click on the picture ("it moved")
let lastNote = 0, lastUndoObj = null;
function esc(s) { return String(s).replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c])); }
function nm(o) { return (o.name ? esc(o.name) + " (#" + o.obj + ")" : "#" + o.obj); }
function ago(s) { return s === null || s === undefined ? "never confirmed by you" : "last confirmed " + Math.round(s) + " s ago"; }

// Every action says what changed and how to take it back. It fades by itself and stays in the list.
function say(text, undo) {
  const t = document.createElement("div"); t.className = "toast"; t.textContent = text;
  if (undo) { const b = document.createElement("button"); b.textContent = "Undo"; b.onclick = () => { undo(); t.remove(); }; t.appendChild(b); }
  $("toasts").appendChild(t);
  while ($("toasts").children.length > 2) $("toasts").removeChild($("toasts").firstChild);
  setTimeout(() => t.classList.add("fade"), 9000); setTimeout(() => t.remove(), 10500);
  const li = document.createElement("li"); li.textContent = text; const f = $("feed");
  if (f.firstChild && f.firstChild.textContent === "Nothing yet.") f.innerHTML = "";
  f.insertBefore(li, f.firstChild); while (f.children.length > 6) f.removeChild(f.lastChild);
}
async function post(path, body) {
  try {
    const r = await fetch(path, {method:"POST", headers:{"Content-Type":"application/json"}, body: JSON.stringify(body || {})});
    return await r.json();
  } catch (e) { return {error: "The page could not reach the program (it may have finished)."}; }
}
async function act(path, body, undoObj) {
  const j = await post(path, body);
  say(j.message || j.error || "Done.", undoObj != null ? () => undoAct(undoObj) : null);
  if (undoObj != null && !j.error) lastUndoObj = undoObj;
  return j;
}
async function undoAct(obj) { const j = await post("/undo", {obj: obj}); say(j.message || j.error || "Done."); }

function draw(s) {
  if (s.w && (cv.width !== s.w || cv.height !== s.h)) { cv.width = s.w; cv.height = s.h; }
  cx.drawImage(img, 0, 0, cv.width, cv.height);
  cx.font = "13px system-ui,sans-serif";
  if (R) {
    R.objs.forEach(o => {
      cx.strokeStyle = cx.fillStyle = "#6cb6ff"; cx.lineWidth = 2;
      cx.beginPath(); cx.arc(o.x, o.y, 16, 0, 6.2832); cx.stroke();
      if (o.heading_deg !== null && o.heading_deg !== undefined) {
        const a = o.heading_deg * Math.PI / 180;
        cx.beginPath(); cx.moveTo(o.x, o.y); cx.lineTo(o.x + 28 * Math.cos(a), o.y - 28 * Math.sin(a)); cx.stroke();
      }
      cx.fillText(o.obj + (o.name ? " " + o.name : "") + " " + o.cls, o.x + 18, o.y - 14);
    });
    return;
  }
  for (const o of s.objects) {
    const c = COL[o.colour] || "#fff";
    cx.strokeStyle = c; cx.fillStyle = c; cx.lineWidth = o.needs ? 4 : 2;
    cx.beginPath(); cx.arc(o.x, o.y, 16, 0, 6.2832); cx.stroke();
    if (o.heading !== null) {
      const a = o.heading * Math.PI / 180;
      cx.beginPath(); cx.moveTo(o.x, o.y); cx.lineTo(o.x + 28 * Math.cos(a), o.y - 28 * Math.sin(a)); cx.stroke();
    }
    cx.fillText("#" + o.obj + " " + (o.name ? o.name + " " : "") + o.cls, o.x + 18, o.y - 14);
  }
}
function side(s) {
  $("fps").textContent = s.fps ? s.fps.toFixed(1) : "-";
  $("src").textContent = s.source || "";
  $("count").textContent = "Frame " + s.frame + (s.total ? " of " + s.total : "");
  const p = $("progress");
  if (s.total) { p.style.display = "block"; $("pbar").style.width = (100 * s.frame / s.total) + "%";
    $("ptxt").textContent = s.left ? "About " + s.left + " left" : ""; }
  $("objs").innerHTML = s.objects.length ? s.objects.map(o =>
    '<div class="row"><span class="dot '+o.colour+'"></span><span>'+nm(o)+' '+esc(o.cls)+'</span><span class="muted">'+
    Math.round(o.score*100)+'%</span><span class="why">'+esc(o.why || "")+(o.confirmed ? " - " + ago(o.last_confirmed_s) : "")+
    '</span></div>').join("") : '<div class="muted">No objects found in this picture.</div>';
  $("asks").innerHTML = (s.asks || []).map(a =>
    '<div class="panel ask"><p><b>Is '+nm(a)+' still where the circle is?</b></p><p class="muted">'+esc(a.why)+'; '+ago(a.last_confirmed_s)+
    '. The others carry on. Its rows say "no" under confirmed until you answer.</p><div class="btns">'+
    '<button data-k="yes" data-o="'+a.obj+'">Yes, it is there</button>'+
    '<button data-k="move" data-o="'+a.obj+'">It is elsewhere: I will click it</button>'+
    '<button data-k="gone" data-o="'+a.obj+'">It is gone</button></div></div>').join("");
  document.querySelectorAll("#asks button").forEach(b => b.onclick = () => askClick(b.dataset.k, +b.dataset.o, s));
}
function askClick(k, obj, s) {
  const o = (s.asks || []).find(a => a.obj === obj) || {name: "", obj: obj};
  if (k === "yes") act("/confirm", {obj: obj}, obj);
  else if (k === "gone") act("/remove", {obj: obj}, obj);
  else { pick = obj; cv.style.cursor = "crosshair"; say("Click on the picture where " + nm(o) + " is now. Nothing changes until you click; press Esc to cancel."); }
}

// ---- roll call editing (all in the page until the green button) ----
function rollStart(r) {
  R = {id: r.id, objs: JSON.parse(JSON.stringify(r.objects)), classes: r.classes, undo: [], add: false, drag: null};
  $("rollbox").style.display = "block"; rollList();
  say("Roll call: " + R.objs.length + " object(s) found. Fix anything that is wrong, then press the green button. Nothing is tracked until you do.");
}
function rollList() {
  if (!R) return;
  $("rolllist").innerHTML = R.objs.length ? R.objs.map((o, i) =>
    '<div class="rollrow"><b>'+o.obj+'</b><select data-i="'+i+'" class="ty">'+R.classes.map(c => '<option'+(c===o.cls?' selected':'')+'>'+esc(c)+'</option>').join("")+'</select>'+
    '<input type="text" data-i="'+i+'" class="nmi" placeholder="name (optional)" value="'+esc(o.name || "")+'"><button data-i="'+i+'" class="del">Delete</button></div>').join("")
    : '<p class="muted">Nothing was found. Press "Add a circle" and click where an object is.</p>';
  document.querySelectorAll(".ty").forEach(e => e.onchange = () => {
    const o = R.objs[+e.dataset.i], was = o.cls; R.undo.push(["cls", o.obj, was]); o.cls = e.value;
    say("Circle " + o.obj + " is now type " + o.cls + " (was " + was + "). Press Z or Undo to put it back."); });
  document.querySelectorAll(".nmi").forEach(e => e.onchange = () => {
    const o = R.objs[+e.dataset.i], was = o.name || ""; R.undo.push(["name", o.obj, was]); o.name = e.value.trim();
    say("Circle " + o.obj + " is now called \\"" + o.name + "\\" (was \\"" + was + "\\"). Press Z or Undo to put it back."); });
  document.querySelectorAll(".del").forEach(e => e.onclick = () => {
    const i = +e.dataset.i, o = R.objs.splice(i, 1)[0]; R.undo.push(["del", o]); rollList(); redraw();
    say("Deleted circle " + o.obj + " (" + o.cls + "). Press Z or Undo to bring it back."); });
}
function rollUndo() {
  if (!R || !R.undo.length) { say("Nothing to undo in the roll call."); return; }
  const u = R.undo.pop();
  if (u[0] === "del") { R.objs.push(u[1]); say("Brought back circle " + u[1].obj + "."); }
  else if (u[0] === "add") { R.objs = R.objs.filter(o => o.obj !== u[1]); say("Removed the circle " + u[1] + " you added."); }
  else if (u[0] === "move") { const o = R.objs.find(o => o.obj === u[1]); if (o) { o.x = u[2]; o.y = u[3]; } say("Put circle " + u[1] + " back where it was."); }
  else { const o = R.objs.find(o => o.obj === u[1]); if (o) o[u[0] === "cls" ? "cls" : "name"] = u[2]; say("Put circle " + u[1] + " back as it was."); }
  rollList(); redraw();
}
function redraw() { if (last) draw(last); }
function cpos(e) { const b = cv.getBoundingClientRect(); return [(e.clientX - b.left) * cv.width / b.width, (e.clientY - b.top) * cv.height / b.height]; }
cv.onmousedown = e => {
  if (!R) return; const [x, y] = cpos(e);
  if (R.add) {
    const id = Math.max(0, ...R.objs.map(o => o.obj)) + 1;
    R.objs.push({obj: id, x: x, y: y, cls: R.classes[0], name: "", heading_deg: null}); R.undo.push(["add", id]);
    R.add = false; cv.style.cursor = ""; rollList(); redraw();
    say("Added circle " + id + " at the spot you clicked (type " + R.classes[0] + "). Press Z or Undo to remove it."); return;
  }
  let best = null, bd = 24;
  R.objs.forEach(o => { const d = Math.hypot(o.x - x, o.y - y); if (d < bd) { bd = d; best = o; } });
  if (best) R.drag = {o: best, x0: best.x, y0: best.y};
};
cv.onmousemove = e => { if (R && R.drag) { const [x, y] = cpos(e); R.drag.o.x = x; R.drag.o.y = y; redraw(); } };
window.onmouseup = () => {
  if (R && R.drag) { const d = R.drag; R.drag = null;
    if (Math.hypot(d.o.x - d.x0, d.o.y - d.y0) > 2) { R.undo.push(["move", d.o.obj, d.x0, d.y0]);
      say("Moved circle " + d.o.obj + ". Press Z or Undo to put it back."); } }
};
cv.onclick = e => {
  if (pick === null || R) return; const [x, y] = cpos(e), o = pick; pick = null; cv.style.cursor = "";
  act("/confirm", {obj: o, x: x, y: y}, o);
};
$("addbtn").onclick = () => { if (R) { R.add = true; cv.style.cursor = "crosshair"; say("Click on the picture where the new object is. Nothing is added until you click."); } };
$("rundo").onclick = rollUndo;
$("rgo").onclick = async () => {
  if (!R) return; const body = {objects: R.objs}; const j = await post("/rollcall", body);
  say(j.message || j.error || "Done.");
  if (!j.error) { R = null; $("rollbox").style.display = "none"; }
};
$("redo").onclick = () => act("/rollcall/redo", {});
window.addEventListener("keydown", e => {
  if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "SELECT")) return;
  if (e.key === "Escape" && pick !== null) { pick = null; cv.style.cursor = ""; say("Cancelled. Nothing changed."); }
  if ((e.key === "z" || e.key === "Z")) { if (R) rollUndo(); else if (lastUndoObj !== null) { undoAct(lastUndoObj); lastUndoObj = null; } }
});
function finish(text) {
  done = true; $("msg").textContent = text; $("stop").disabled = true;
}
async function loop() {
  while (!done) {
    try {
      const r = await fetch("/state?since=" + since);
      const s = await r.json();
      (s.notes || []).forEach(n => { if (n.id > lastNote) { lastNote = n.id; say(n.text); } });
      if (s.done) { finish(s.message || "Finished."); }
      if (s.same) { continue; }
      since = s.ver;
      await new Promise(res => { img.onload = res; img.onerror = res; img.src = s.image; });
      if (s.roll && (!R || R.id !== s.roll.id)) rollStart(s.roll);
      if (!s.roll && R) { R = null; $("rollbox").style.display = "none"; }
      last = s; draw(s); side(s);
    } catch (e) {
      if (since >= 0) { finish("Finished. You can close this page."); } else { await new Promise(r => setTimeout(r, 500)); }
    }
  }
}
$("stop").onclick = async () => {
  $("stop").disabled = true;
  $("msg").textContent = "Stopping and saving ...";
  say("Stopping. The table so far is being saved. To take it back, start a new run: nothing already saved is lost.");
  try { await fetch("/stop", {method:"POST"}); } catch (e) {}
};
loop();
</script></body></html>
"""


class LiveView:
    """The small local web page. publish() is called by the tracking loop for every frame; the page
    asks for news with /state (waits up to a second for a new frame) and stops the run with /stop.

    Routes for the roll call and confirmations (all local, JSON):
      GET  /rollcall        the roll call being shown now ({"active": false} when none)
      POST /rollcall        {"objects": [{obj, x, y, cls, name, heading_deg}]}: confirm it
      POST /rollcall/redo   ask for the roll call again, starting from the objects as tracked now
      GET  /asks            the objects waiting for a fresh confirmation
      POST /confirm         {"obj": N, "x": px, "y": px}: this one is here (x, y optional: where it now is)
      POST /remove          {"obj": N}: this one is gone
      POST /undo            {"obj": N}: take back the last confirm or remove of that object
    Every POST answers {"ok": true, "message": "what changed, and how to take it back"}, or
    {"error": "plain words"} with a 4xx code."""

    def __init__(self, port=0, source_label="", classes=None, pipe=None):
        self.cond = threading.Condition()
        self.stop_event = threading.Event()
        self.source_label = source_label
        self.classes = list(classes) if classes else ["unknown"]
        self.pipe = pipe                  # confirmations are queued to it
        self.seq, self.frame, self.image, self.objects, self.fps = -1, -1, None, [], 0.0
        self.total, self.left, self.message, self.done = None, "", "", False
        self.size = (640, 480)
        self.asks, self.notes, self._note_id = [], [], 0
        self.roll = None                  # dict(id, frame, objects, classes) while a roll call is shown
        self._roll_n = 0
        self._roll_result = None
        self._roll_event = threading.Event()
        self.redo_event = threading.Event()
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body, ctype):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionError):
                    pass

            def _json(self, code, obj):
                self._send(code, json.dumps(obj).encode("utf-8"), "application/json")

            def _local(self):
                ok = (self.headers.get("Host") or "").split(":")[0] in ("127.0.0.1", "localhost")
                if not ok:
                    self._send(403, b"refused: open the page at http://127.0.0.1", "text/plain")
                return ok

            def _body(self):
                try:
                    n = int(self.headers.get("Content-Length") or 0)
                    raw = self.rfile.read(n) if n else b""
                    return json.loads(raw.decode("utf-8")) if raw.strip() else {}
                except (ValueError, UnicodeDecodeError):
                    return None

            def do_GET(self):
                if not self._local():
                    return
                path, _, qs = self.path.partition("?")
                if path == "/":
                    page = PAGE.replace("__LEGEND__", json.dumps(LEGEND))
                    self._send(200, page.encode("utf-8"), "text/html; charset=utf-8")
                elif path == "/state":
                    m = re.search(r"since=(-?\d+)", qs)
                    self._send(200, json.dumps(view.state(int(m.group(1)) if m else -1)).encode("utf-8"),
                               "application/json")
                elif path == "/rollcall":
                    with view.cond:
                        r = view.roll
                    self._json(200, dict(r, active=True) if r else dict(active=False))
                elif path == "/asks":
                    with view.cond:
                        a = list(view.asks)
                    self._json(200, dict(asks=a))
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self):
                if not self._local():
                    return
                path = self.path.split("?")[0]
                if path == "/stop":
                    view.stop_event.set()
                    self._send(200, b"{}", "application/json")
                    return
                body = self._body()
                if body is None or not isinstance(body, dict):
                    self._json(400, dict(error="That request could not be read, so nothing was changed."))
                    return
                code, reply = view.handle_post(path, body)
                self._json(code, reply)

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        self.url = "http://127.0.0.1:%d/" % self.port
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    # --- the page's actions -----------------------------------------------------------------
    def _name_of(self, obj):
        with self.cond:
            for o in self.objects:
                if o["obj"] == obj:
                    return "%s (#%d)" % (o["name"], obj) if o.get("name") else "#%d" % obj
        return None

    def handle_post(self, path, body):
        """-> (http code, reply dict). The reply always says what changed and how to take it back."""
        if path == "/rollcall":
            with self.cond:
                r = self.roll
            if r is None:
                return 409, dict(error="No roll call is waiting, so nothing was changed.")
            objs, err = clean_roll(body.get("objects"), self.classes)
            if err:
                return 400, dict(error=err + " Nothing was changed.")
            self._roll_result = objs
            self._roll_event.set()
            names = ", ".join(o["name"] or "#%d" % o["obj"] for o in objs) or "none"
            return 200, dict(ok=True, count=len(objs), message=(
                "Roll call confirmed: %d object(s) (%s). Tracking starts from them now. To take it back, press "
                "\"Redo the roll call\": it asks again, and rows already in the table stay." % (len(objs), names)))
        if path == "/rollcall/redo":
            if self.pipe is None or not self.pipe.check:
                return 409, dict(error="This run was started without a roll call, so nothing was changed.")
            self.redo_event.set()
            return 200, dict(ok=True, message=(
                "Asking for the roll call again. Tracking pauses until you confirm it. To take it back, press the "
                "green button without changing anything: tracking carries on from the objects as they are."))
        obj = body.get("obj")
        if not isinstance(obj, int) or isinstance(obj, bool):
            return 400, dict(error="Which object? Nothing was changed.")
        label = self._name_of(obj)
        if label is None and path != "/undo":
            return 404, dict(error="Object #%d is not in the picture now, so nothing was changed." % obj)
        label = label or "#%d" % obj
        if self.pipe is None:
            return 409, dict(error="This run does not take confirmations, so nothing was changed.")
        if path == "/confirm":
            cmd = dict(kind="confirm", obj=obj)
            for k in ("x", "y"):
                v = body.get(k)
                if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v):
                    cmd[k] = float(v)
            if ("x" in cmd) != ("y" in cmd):
                return 400, dict(error="Give both x and y, or neither. Nothing was changed.")
            self.pipe.queue(cmd)
            where = "at the spot you clicked" if "x" in cmd else "where it is now"
            return 200, dict(ok=True, message=(
                "%s confirmed %s. Its rows in the table say confirmed = yes from here. To take it back, press Undo "
                "(or Z): it goes back to how it was." % (label, where)))
        if path == "/remove":
            self.pipe.queue(dict(kind="remove", obj=obj))
            return 200, dict(ok=True, message=(
                "%s will no longer be followed; its rows so far stay in the table. To take it back, press Undo (or Z)." % label))
        if path == "/undo":
            self.pipe.queue(dict(kind="undo", obj=obj))
            return 200, dict(ok=True, message="Taking back the last change to %s. To change it again, use its buttons." % label)
        return 404, dict(error="Unknown request, so nothing was changed.")

    # --- from the tracking loop -------------------------------------------------------------
    def publish(self, f, image, objects, fps, total=None, left="", asks=None, notes=None):
        with self.cond:
            self.seq += 1                 # a new picture (or a new roll call) for the page, even on the same frame
            self.frame, self.image, self.fps, self.total, self.left = f, image, fps, total, left
            self.size = (image.shape[1], image.shape[0])
            self.objects = [dict(obj=o["obj"], cls=o["cls"], x=o["x_px"], y=o["y_px"], heading=o["heading_deg"],
                                 score=o["score"], colour=o["colour"], name=o.get("name", ""), why=o.get("why", ""),
                                 confirmed=o.get("confirmed", ""), last_confirmed_s=o.get("last_confirmed_s"),
                                 needs=bool(o.get("needs"))) for o in objects]
            self.asks = list(asks or [])
            for n in notes or []:
                self._note_id += 1
                self.notes.append(dict(id=self._note_id, text=n))
            self.notes = self.notes[-30:]
            self.cond.notify_all()

    def show_roll(self, f, image, objs):
        """Show the roll call: the picture and the numbered draft objects. Tracking waits."""
        self._roll_event.clear()
        self._roll_result = None
        self._roll_n += 1
        with self.cond:
            self.roll = dict(id=self._roll_n, frame=f, classes=list(self.classes), objects=[
                dict(obj=o["obj"], cls=o["cls"], x=float(o["x"]), y=float(o["y"]), name=o.get("name", ""),
                     heading_deg=o.get("heading_deg")) for o in objs])
        self.publish(f, image, [], 0.0)

    def wait_roll(self, stop, poll=0.1):
        """Block until the person confirms the roll call (-> list of objects) or the run is stopped (-> None)."""
        while not stop.is_set():
            if self._roll_event.wait(poll):
                break
        with self.cond:
            self.roll = None
            self.cond.notify_all()
        return None if stop.is_set() and self._roll_result is None else self._roll_result

    def finish(self, message):
        with self.cond:
            self.done, self.message = True, message
            self.cond.notify_all()

    def state(self, since):
        with self.cond:
            if self.seq == since and not self.done:
                self.cond.wait(1.0)
            notes = list(self.notes)
            if self.done and self.seq == since:
                return dict(same=True, done=True, message=self.message, notes=notes)
            if self.seq < 0 or self.seq == since:
                return dict(same=True, done=self.done, message=self.message, notes=notes)
            img, objs, f, fps, total, left = self.image, list(self.objects), self.frame, self.fps, self.total, self.left
            ver = self.seq
            w, h = self.size
            done, message = self.done, self.message
            asks, roll = list(self.asks), self.roll
        import cv2
        import base64
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        data = "data:image/jpeg;base64," + base64.b64encode(jpg.tobytes()).decode("ascii") if ok else ""
        return dict(frame=f, ver=ver, image=data, objects=objs, fps=fps, total=total, left=left, w=w, h=h,
                    source=self.source_label, done=done, message=message, asks=asks, roll=roll, notes=notes)

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def clean_roll(objs, classes):
    """What the page sent -> (list of tidy objects, None), or (None, a plain sentence about what is wrong)."""
    if not isinstance(objs, list):
        return None, "The roll call has no list of objects."
    out, used = [], set()
    for o in objs:
        if not isinstance(o, dict):
            return None, "One of the objects could not be read."
        try:
            x, y = float(o["x"]), float(o["y"])
        except (KeyError, TypeError, ValueError):
            return None, "An object has no position."
        if not (math.isfinite(x) and math.isfinite(y)):
            return None, "An object has no position."
        hd = o.get("heading_deg")
        hd = float(hd) if isinstance(hd, (int, float)) and not isinstance(hd, bool) and math.isfinite(hd) else None
        cls = str(o.get("cls") or classes[0])
        name = re.sub(r"[\r\n,]+", " ", str(o.get("name") or "")).strip()[:40]
        oid = o.get("obj")
        oid = oid if isinstance(oid, int) and not isinstance(oid, bool) and oid > 0 and oid not in used else None
        out.append(dict(obj=oid, x=x, y=y, cls=cls, name=name, heading_deg=hd))
        if oid:
            used.add(oid)
    nxt = max(used or {0}) + 1
    for o in out:
        if o["obj"] is None:
            o["obj"] = nxt
            used.add(nxt)
            nxt += 1
    return out, None


# --------------------------------------------------------------------------------------------
# Roll call: what the model sees at the start, confirmed or fixed by the person once
# --------------------------------------------------------------------------------------------

def _split_spec(spec):
    """"plugin:path" -> (plugin, path); a plain path (also C:\\... on Windows) -> (None, path)."""
    import outside
    ids = [p[0] for p in outside.plugins()]
    if ":" in spec:
        head, rest = spec.split(":", 1)
        if head in ids:
            return head, rest
    return None, spec


class OutsideDrafts:
    """Outside data lined up on the floor, for the roll call: drafts(t_wall text) -> where each named
    object is in the picture at that moment (outside.draft_labels at one time)."""

    def __init__(self, aligners, camera, cls_of, heights, plugin=""):
        self.aligners, self.camera, self.cls_of, self.heights, self.plugin = aligners, camera, cls_of, heights, plugin

    def drafts(self, t_wall_text):
        import outside
        try:
            t = float(t_wall_text)
        except (TypeError, ValueError):
            raise ValueError("this picture has no time on the shared clock (a video file or a camera without "
                             "frames.csv), so the outside data can't say who is where")
        d = outside.draft_labels(self.aligners, {0: t}, self.camera, cls_of=self.cls_of, class_heights=self.heights,
                                 plugin=self.plugin)
        return [dict(x=o["x"], y=o["y"], heading_deg=o["heading_deg"], name=o["name"], cls=o["cls"]) for o in d.get(0, [])]


def load_outside(spec, say=print, proj=None, pipe=None, recording=None):
    """Optional outside data (positions and headings on the shared clock), given as --outside plugin:path
    (or just a path: the plug-in that recognises it is used). It is read with outside.py, lined up on the
    floor with the project's home positions (the log's own "I am home" events) and used only to suggest
    names (and headings) for the objects found at the roll call. Needs the camera's floor calibration; a
    stream that is not lined up yet (a robot's own dead reckoning with no home event) names nothing.
    Returns an OutsideDrafts or None (said in plain words); a file that can't be read is a UserError."""
    if not spec:
        return None
    try:
        import outside
        import homes as homesmod
    except ImportError:
        say("Outside data (%s) was asked for, but the outside-data part (outside.py) is missing. "
            "Carrying on without it; the objects can still be named by hand at the roll call." % spec)
        return None
    cam = getattr(pipe, "cam", None)
    if cam is None:
        say("Outside data (%s) is NOT used: it needs the floor calibration of camera \"%s\" (which pixel is which "
            "spot on the floor), and there is none (make one in the studio: More, then Floor calibration). Carrying on "
            "without it; name the objects by hand at the roll call."
            % (spec, getattr(pipe, "camera_key", None) or "default"))
        return None
    plugin, path = _split_spec(spec)
    try:
        data = outside.read_outside(os.path.expanduser(path), plugin, recording=recording)
    except Exception as e:
        raise UserError("The outside data %s could not be read (%s). Check the plug-in name and the folder or file, "
                        "or leave --outside out." % (spec, e))
    heights = dict(getattr(pipe, "heights", {}) or {})
    classes = list(getattr(pipe, "classes", None) or [])
    cls_of = {n: (st.cls if (st.cls in classes or not classes) else (classes[0] if classes else "unknown"))
              for n, st in data.streams.items()}
    anchors = {}
    key = getattr(pipe, "camera_key", None) or "default"
    hl = homesmod.homes_of(proj.data, key) if proj is not None else []
    if hl and data.streams:
        z = heights.get(next(iter(cls_of.values())), 0.0)
        for n, lst in homesmod.events_to_anchors(data.events, hl, cam, height_cm=z).items():
            anchors.setdefault(n, []).extend(lst)
    als = outside.align(data, anchors)
    lined = [n for n, st in data.streams.items() if st.frame == "floor" or anchors.get(n)]
    not_lined = sorted(n for n in data.streams if n not in lined)
    if not data.streams:
        why = "It holds no positions, so it names nothing."
    elif not_lined:
        reason = ("there are no home positions for camera \"%s\" in the project" % key if not hl else
                  "its log has no \"at home\" event at one of this camera's home positions")
        why = ("%s: its own log (in its own start frame) can't be placed on the floor because %s, so it names "
               "nothing here; name %s by hand at the roll call." % (", ".join(not_lined), reason,
                                                                   "it" if len(not_lined) == 1 else "them"))
    else:
        why = ""
    if not getattr(data, "clock_known", True):
        why += (" Its clock is not tied to the video's (no clock file), so its times were used as they are; "
                "a name can land on the wrong object if the clocks differ by more than a moment.")
    say("Outside data: %s. %s%s" % (data.summary(), ("Lined up on the floor: %s. " % ", ".join(sorted(lined))) if lined
                                     else "", why.strip()))
    return OutsideDrafts(als, cam, cls_of, heights, data.plugin)


def draft_objects(dets, pipe=None, t_wall=""):
    """The model's guesses on the steady picture -> numbered objects, left to right."""
    objs = []
    for i, d in enumerate(sorted(dets or [], key=lambda d: (d["x"], d["y"])), 1):
        hd = d.get("heading_deg")
        objs.append(dict(obj=i, cls=str(d.get("cls") or "unknown"), x=float(d["x"]), y=float(d["y"]), name="",
                         heading_deg=float(hd) if isinstance(hd, (int, float)) else None))
    out = getattr(pipe, "outside", None)
    if out is not None and objs:
        try:
            drafts = list(out.drafts(t_wall) or [])
        except Exception as e:
            drafts = []
            note = getattr(pipe, "_note", None)
            if callable(note):
                note("The outside data could not suggest names this time (%s); name the objects by hand." % e)
        apply_outside(objs, drafts, pipe.robot_px)
    return objs


def apply_outside(objs, drafts, robot_px):
    """Each found object takes the name (and a missing heading) of the nearest outside draft within
    1.5 object sizes, one draft per object."""
    free = [d for d in drafts if d.get("x") is not None and d.get("y") is not None]
    for o in objs:
        best, bd = None, 1.5 * robot_px
        for d in free:
            dist = math.hypot(d["x"] - o["x"], d["y"] - o["y"])
            if dist < bd:
                best, bd = d, dist
        if best is not None:
            free.remove(best)
            if best.get("name") and not o.get("name"):
                o["name"] = str(best["name"])
            if o.get("heading_deg") is None and best.get("heading_deg") is not None:
                o["heading_deg"] = float(best["heading_deg"])


def find_steady(source, pipe, stop):
    """Look at pictures until the objects stop moving about (the same ones in the same places for a few
    pictures in a row, at most STEADY_MAX pictures). -> (item, dets) of the steady one, or None if the
    source ended or the run was stopped. Every picture is shown to the detector exactly once."""
    prev, run, item, dets = None, 0, None, []
    for _ in range(STEADY_MAX):
        if stop.is_set():
            return None
        item = source.read()
        if item is None:
            return None
        dets = pipe.look(item[3])
        pos = sorted((d["x"], d["y"]) for d in dets)
        same = bool(pos) and prev is not None and len(pos) == len(prev) and all(
            math.hypot(a[0] - b[0], a[1] - b[1]) <= 8.0 for a, b in zip(pos, prev))
        run = run + 1 if same else 0
        prev = pos
        if run >= STEADY_FRAMES - 1:
            break
    return item, dets


def _show(objs, say):
    if not objs:
        say("  (nothing found)")
    for o in objs:
        say("  %d  %s%s at (%d, %d)%s" % (o["obj"], o["cls"], ("  called " + o["name"]) if o["name"] else "",
                                          o["x"], o["y"], ("  facing %d" % o["heading_deg"]) if o.get("heading_deg") is not None else ""))


def console_roll(objs, classes, ask=input, say=print):
    """The roll call in this window (for --no-page). -> the confirmed objects, or None if the person quit."""
    objs = [dict(o) for o in objs]
    hist = []
    say("")
    say("Roll call: this is what the model sees in the first steady picture.")
    _show(objs, say)
    help_ = ("Press Enter if this is right. Or type: d 2 (delete 2), m 2 300 200 (move 2), a 300 200 [type] [name] (add), "
             "n 1 Left (name 1), t 1 type (set type 1), u (take back the last change), q (stop). Types: %s." % ", ".join(classes))
    while True:
        line = ask(help_ + "\n> ").strip()
        if not line or line.lower() in ("y", "yes", "ok"):
            say("Roll call confirmed: %d object(s). Tracking starts from them. To take it back, run again." % len(objs))
            return objs
        p = line.split()
        c = p[0].lower()
        try:
            if c == "q":
                return None
            if c == "u":
                if not hist:
                    say("Nothing to take back.")
                    continue
                objs = hist.pop()
                say("Taken back.")
            elif c in ("d", "m", "n", "t"):
                oid = int(p[1])
                tgt = next((o for o in objs if o["obj"] == oid), None)
                if tgt is None:
                    say("There is no object %d. Nothing changed." % oid)
                    continue
                hist.append([dict(o) for o in objs])
                if c == "d":
                    objs = [o for o in objs if o is not tgt]
                    say("Deleted %d. Type u to take it back." % oid)
                elif c == "m":
                    tgt["x"], tgt["y"] = float(p[2]), float(p[3])
                    say("Moved %d to (%d, %d). Type u to put it back." % (oid, tgt["x"], tgt["y"]))
                elif c == "n":
                    tgt["name"] = " ".join(p[2:])[:40]
                    say("Object %d is now called \"%s\". Type u to put it back." % (oid, tgt["name"]))
                else:
                    if p[2] not in classes:
                        hist.pop()
                        say("Unknown type %s. Types: %s. Nothing changed." % (p[2], ", ".join(classes)))
                        continue
                    tgt["cls"] = p[2]
                    say("Object %d is now type %s. Type u to put it back." % (oid, p[2]))
            elif c == "a":
                x, y = float(p[1]), float(p[2])
                cls = p[3] if len(p) > 3 and p[3] in classes else classes[0]
                rest = p[4:] if len(p) > 3 and p[3] in classes else p[3:]
                hist.append([dict(o) for o in objs])
                oid = max([o["obj"] for o in objs] or [0]) + 1
                objs.append(dict(obj=oid, cls=cls, x=x, y=y, name=" ".join(rest)[:40], heading_deg=None))
                say("Added object %d (%s) at (%d, %d). Type u to remove it." % (oid, cls, x, y))
            else:
                say("I did not understand that. Nothing changed.")
                continue
        except (IndexError, ValueError):
            say("I did not understand that. Nothing changed.")
            continue
        _show(objs, say)


def confirm_roll(objs, img, f, view, pipe, stop, say, ask, classes):
    """Show the draft and wait for the person (page, else this window). -> objects, or None (stopped)."""
    if view is not None:
        view.show_roll(f, img, objs)
        say("The roll call is on the page: check the numbered circles, then press the green button.")
        return view.wait_roll(stop)
    return console_roll(objs, classes, ask, say)


def roll_classes(pipe, dets):
    names = list(pipe.classes or [])
    for d in dets or []:
        if d.get("cls") and d["cls"] not in names:
            names.append(d["cls"])
    return names or ["unknown"]


def start_roll(source, pipe, view, stop, say, ask):
    """The roll call at the start. -> dict(item, dets, anchor) for the first picture, or None."""
    st = find_steady(source, pipe, stop)
    if st is None:
        return None
    item, dets = st
    f, t, tw, img = item
    classes = roll_classes(pipe, dets)
    if view is not None:
        view.classes = classes
    objs = confirm_roll(draft_objects(dets, pipe, tw), img, f, view, pipe, stop, say, ask, classes)
    if objs is None:
        return None
    return dict(item=item, dets=dets, anchor=pipe.start_from(objs, f, t))


def redo_roll(pipe, recs, img, f, t, view, stop, say, ask):
    """The roll call again, starting from the objects as tracked now. -> anchor for the next picture, or None."""
    heads = {r["obj"]: r.get("heading_deg") for r in recs}
    draft = [dict(obj=tr.id, cls=tr.cls, x=tr.x, y=tr.y, name=pipe.names.get(tr.id, ""), heading_deg=heads.get(tr.id))
             for tr in pipe.tk.tracks if tr.confirmed]
    classes = roll_classes(pipe, draft)
    if view is not None:
        view.classes = classes
    objs = confirm_roll(draft, img, f, view, pipe, stop, say, ask, classes)
    return None if objs is None else pipe.start_from(objs, f, t)


# --------------------------------------------------------------------------------------------
# Running
# --------------------------------------------------------------------------------------------

def _nice_time(s):
    s = int(round(s))
    if s < 60:
        return "%d s" % s
    return "%d min %02d s" % (s // 60, s % 60)


class SaveError(UserError):
    """The table could not be saved (the message says where the rows so far are)."""


def partial_path(out_path):
    """Rows are written here as they come, so a run that stops early still leaves them."""
    root, ext = os.path.splitext(out_path)
    return root + ".partial" + (ext or ".csv")


def _replace(src, dst):
    """Swap a finished file in; on Windows another program (Excel) may hold the target for a moment."""
    for _ in range(15):
        try:
            os.replace(src, dst)
            return
        except PermissionError:
            time.sleep(0.2)
    os.replace(src, dst)


def run(source, pipe, out_path, view=None, seconds=0, max_frames=0, say=print, stop=None, quiet_progress=False,
        roll=False, ask=input):
    """Watch the source and write the table. Returns the number of rows written.
    Every row is written as it happens to <name>.partial.csv. At the end the table is written to a
    temporary file and renamed to out_path: for recordings and video files the finished table (the
    first frames of each object filled in), for cameras the rows as they happened. If the run stops
    early or the save fails, the rows so far stay in the .partial.csv file (SaveError says so).
    roll=True starts with the roll call (on the page when there is one, else in this window with ask)
    and tracks from what the person confirmed; the pictures looked at while waiting for a steady one
    are not in the table."""
    stop = stop or (view.stop_event if view else threading.Event())
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    if roll:
        pipe.check = True
    cols = pipe.columns
    live = bool(source.live)
    part = partial_path(out_path)
    try:
        fh = open(part, "w", newline="", encoding="utf-8")
    except OSError as e:
        raise SaveError("The table can't be saved at %s (%s). Pick another place with --out." % (out_path, e))
    writer = csv.writer(fh)
    writer.writerow(cols)
    n_rows = 0
    n_live = 0
    t_start = time.time()
    stamps = []
    last_say = 0.0
    frames = 0
    total = source.n if not live else None
    if max_frames:
        total = min(total, max_frames) if total else max_frames
    finished = False
    first, anchor_next, objs = None, None, []
    try:
        if roll:
            first = start_roll(source, pipe, view, stop, say, ask)
            if first is None and live and not stop.is_set():
                raise UserError("No pictures are coming from %s. Check that it is on and connected." % source.label)
        while not stop.is_set() and (first is not None or not roll or frames > 0):
            if max_frames and frames >= max_frames:
                break
            if seconds and time.time() - t_start >= seconds:
                break
            dets = anchor = None
            if first is not None:
                item, dets, anchor, first = first["item"], first["dets"], first["anchor"], None
            else:
                item = source.read()
                anchor, anchor_next = anchor_next, None
            if item is None:
                if live and not stop.is_set() and time.time() - t_start < 3 and frames == 0:
                    raise UserError("No pictures are coming from %s. Check that it is on and connected." % source.label)
                break
            f, t, tw, img = item
            extra = {}
            if dets is not None:
                extra["dets"] = dets
            if anchor is not None:
                extra["anchor"] = anchor
            objs = pipe.process(f, t, tw, img, **extra)
            frames += 1
            now = time.time()
            stamps.append(now)
            if len(stamps) > 30:
                stamps.pop(0)
            fps = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0
            left = ""
            if total and fps > 0:
                left = _nice_time(max(0, total - frames) / fps)
            for o in objs:
                writer.writerow(csv_line(o, cols))
                n_live += 1
            if frames % 20 == 0:
                fh.flush()
            notes = pipe.take_notes() if pipe.check else []
            if view is not None:
                view.publish(f, img, objs, fps, total, left, asks=pipe.asks(objs) if pipe.check else None, notes=notes)
            else:
                for n in notes:
                    say(n)
            if roll and view is not None and view.redo_event.is_set():
                view.redo_event.clear()
                anchor_next = redo_roll(pipe, objs, img, f, t, view, stop, say, ask)
            if not quiet_progress and now - last_say >= 0.5:
                last_say = now
                if total:
                    say("  frame %d of %d (%d%%), %.1f frames a second, about %s left"
                        % (frames, total, min(100, 100 * frames // total), fps, left or "?"))
                else:
                    say("  %d frames so far, %.1f frames a second, %d objects in the last picture" % (frames, fps, len(objs)))
        finished = True
    finally:
        fh.close()
        if not finished and n_live:
            say("Stopped early: the %d rows so far are in %s" % (n_live, part))
    try:
        if live:
            _replace(part, out_path)
            n_rows = n_live
        else:
            rows = pipe.finished_rows()
            tmp = "%s.%d.tmp" % (out_path, os.getpid())
            try:
                with open(tmp, "w", newline="", encoding="utf-8") as fh2:
                    w2 = csv.writer(fh2)
                    w2.writerow(cols)
                    for rec in rows:
                        w2.writerow(csv_line(rec, cols))
                _replace(tmp, out_path)
            finally:
                if os.path.exists(tmp):
                    os.remove(tmp)
            n_rows = len(rows)
            os.remove(part)
            n_no = sum(1 for r in rows if r.get("confirmed") == "no")
            if n_no:
                say("%d of the %d rows say confirmed = no: on those frames the object's score had stayed low (lost, "
                    "hidden or doubtful) and no person had confirmed it again yet. Their positions are the "
                    "tracker's best guess; check them before relying on them." % (n_no, n_rows))
    except OSError as e:
        raise SaveError("The table could not be saved to %s (%s). Is it open in another program, such as Excel? "
                        "The rows as they came are kept in %s." % (out_path, e, part))
    return n_rows


def explain_empty(min_score):
    """Why a run can end with no rows, in plain words."""
    return ("The model found no objects it was sure enough about (its minimum score is %s). Try a lower --min-score "
            "(for example 0.2), or label more frames and train the model longer, then run it again."
            % ("the model's own setting, usually 0.3" if min_score is None else "%.2f" % min_score))


# --------------------------------------------------------------------------------------------
# Asking plain questions (when started with no arguments, e.g. by double-clicking the .bat)
# --------------------------------------------------------------------------------------------

def default_project_folder():
    docs = os.path.join(os.path.expanduser("~"), "Documents")
    return os.path.join(docs if os.path.isdir(docs) else os.path.expanduser("~"), "recognition-project")


def _clean_path(s):
    return os.path.expanduser(s.strip().strip('"').strip("'"))


def ask_questions(ask=input, say=print):
    """Plain questions -> a list of command-line arguments for main()."""
    say("%s - %s" % (PRODUCT, TAGLINE))
    say("")
    say("This follows the objects in a recording or a live camera with a model you trained,")
    say("and saves where every object was, frame by frame, in a table you can open in Excel.")
    say("")
    args = []
    proj = None
    while True:
        default = default_project_folder()
        a = ask("Which project? Type its folder and press Enter.\n"
                "Just press Enter to use %s\nOr type 0 to give a model file instead: " % default).strip()
        if a == "0":
            while True:
                m = _clean_path(ask("Type the path of the model file (it ends in .onnx): "))
                if m and os.path.isfile(m):
                    args += ["--onnx", m]
                    break
                say("That file was not found. Try again.")
            break
        folder = _clean_path(a) if a else default
        try:
            proj = load_project(folder)
        except UserError as e:
            say(str(e))
            say("")
            continue
        args += ["--project", proj.folder]
        break
    if proj is not None:
        try:
            ready = ready_models(proj)
            chosen, _ = resolve_model(proj)
        except UserError as e:
            say(str(e))
            raise
        if len(ready) > 1:
            say("")
            say("The project has these finished models: %s." % ", ".join(m["id"] for m, _ in ready))
            while True:
                a = ask("Which model? Press Enter for %s (the one the studio would use): " % chosen).strip()
                if not a:
                    break
                if any(m["id"] == a for m, _ in ready):
                    args += ["--model", a]
                    break
                say("There is no finished model called %s." % a)
    say("")
    say("What should it watch?")
    say("  1  A recording (a folder with video.avi in it)")
    say("  2  A video file")
    say("  3  A camera plugged into this computer")
    say("  4  A camera on the network (for example the Pi's preview address)")
    while True:
        kind = ask("Type 1, 2, 3 or 4 and press Enter: ").strip()
        if kind in ("1", "2", "3", "4"):
            break
        say("Please type just 1, 2, 3 or 4.")
    source = None
    if kind == "1":
        recs = [r for r in proj.recordings() if os.path.exists(r["path"])] if proj is not None else []
        if recs:
            say("")
            say("The project's recordings:")
            for i, r in enumerate(recs, 1):
                say("  %d  %s%s" % (i, r.get("name") or r["id"], ("  (camera %s)" % r["camera"]) if r.get("camera") else ""))
        while True:
            a = ask("%sType its number%s, or paste the folder: " % ("" if recs else "", " from the list" if recs else "")).strip()
            if recs and a.isdigit() and 1 <= int(a) <= len(recs):
                source = recs[int(a) - 1]["path"]
                break
            p = _clean_path(a)
            if p and os.path.isdir(p):
                source = p
                break
            say("That folder was not found. Try again.")
    elif kind == "2":
        while True:
            p = _clean_path(ask("Type or paste the path of the video file: "))
            if p and os.path.isfile(p):
                source = p
                break
            say("That file was not found. Try again.")
    elif kind == "3":
        while True:
            a = ask("Which camera number? Usually 0. Press Enter for 0: ").strip() or "0"
            if a.isdigit():
                source = a
                break
            say("Please type a number such as 0 or 1.")
    else:
        while True:
            a = ask("Type the camera's address, for example http://st1maxfini.local:8000 : ").strip()
            if a and classify_source(a) == "url":
                source = a
                break
            say("That does not look like an address. It starts with http:// or is like name.local:8000.")
    args.insert(0, source)
    live = kind in ("3", "4")
    a = ask("Show the live picture in your web browser while it works? Press Enter for %s (type %s to change): "
            % ("yes" if live else "no", "n" if live else "y")).strip().lower()
    show = live if not a else a.startswith("y")
    args.append("--page" if show else "--no-page")
    if live:
        a = ask("It keeps going until you press the Stop button on the page (or Ctrl+C here).\n"
                "Stop by itself after how many seconds? Press Enter for no limit: ").strip()
        if a.isdigit() and int(a) > 0:
            args += ["--seconds", a]
    a = _clean_path(ask("Where to save the table? Press Enter to let it choose a name in the project, "
                        "or type a file name: "))
    if a:
        args += ["--out", a]
    say("")
    return args


# --------------------------------------------------------------------------------------------
# Command line
# --------------------------------------------------------------------------------------------

def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", nargs="?", help="recording folder, video file, camera number or camera address")
    ap.add_argument("--project", help="project folder (uses its chosen model, work area and floor calibration)")
    ap.add_argument("--model", help="which model of the project, e.g. v3")
    ap.add_argument("--onnx", help="a model file (.onnx) instead of a project")
    ap.add_argument("--camera-name", help="which of the project's cameras this is")
    ap.add_argument("--out", help="where to save the table (.csv)")
    ap.add_argument("--page", dest="page", action="store_true", default=None, help="show the live page")
    ap.add_argument("--no-page", dest="page", action="store_false", help="do not show the live page")
    ap.add_argument("--no-browser", action="store_true", help="start the page but do not open the browser")
    ap.add_argument("--port", type=int, default=0, help="port for the live page (default: any free one)")
    ap.add_argument("--seconds", type=float, default=0,
                    help="stop after it has run this many seconds (clock time while it runs, not seconds of video; "
                         "for part of a recording use --max-frames)")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after this many frames")
    ap.add_argument("--every", type=int, default=1, help="look at every n-th frame only")
    ap.add_argument("--robot-px", type=float, help="object size in pixels")
    ap.add_argument("--min-score", type=float, help="ignore model guesses below this score (0 to 1)")
    ap.add_argument("--threads", type=int, help="processor threads for the model")
    ap.add_argument("--roll-call", dest="roll_call", action="store_true", default=None,
                    help="start with a roll call (default: yes for a live camera with the page)")
    ap.add_argument("--no-roll-call", dest="roll_call", action="store_false", help="do not start with a roll call")
    ap.add_argument("--ask-below", type=float, default=ASK_BELOW,
                    help="ask for a fresh confirmation of an object whose score stays below this (0 to 1)")
    ap.add_argument("--ask-after", type=float, default=ASK_AFTER_S,
                    help="... for this many seconds")
    ap.add_argument("--outside", help="a positions log (robot arm, conveyor, robot ...) to help name the objects at "
                    "the roll call, as plugin:path (csv:<positions.csv>, alvik:<Alvik robot log folder>)")
    return ap


def default_out(proj, source, model_id, spec):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    if source.live:
        base = "live-%s" % stamp
    else:
        # a new name every run: an earlier table is never overwritten
        base = (re.sub(r"[^A-Za-z0-9_.-]+", "-", (source.label or "run")).strip("-.") or "run") + "_" + stamp
    if model_id:
        base += "_" + model_id
    folder = os.path.join(proj.folder, "tracks") if proj is not None else os.getcwd()
    return os.path.join(folder, base + ".csv")


def make_pipeline(a, source, say=print, detector=None):
    """Everything chosen from the arguments: model, project settings, area, floor calibration."""
    import infer
    proj, model_id, onnx, cam_name = None, None, a.onnx, None
    if a.project:
        proj = load_project(a.project)
        if not onnx:
            model_id, onnx = resolve_model(proj, a.model)
    if not onnx and detector is None:
        raise UserError("Say which model to use: --project <project folder> or --onnx <model file>.")
    if onnx and not os.path.isfile(onnx):
        raise UserError("The model file %s was not found." % onnx)
    if detector is None:
        model = infer.load(onnx, threads=a.threads, score_thresh=a.min_score)
        detector = lambda img: infer.predict(model, img)   # noqa: E731
        classes_in_model = list(model.classes)
        heading = list(model.cfg.get("heading_classes", [])) or None
    else:
        classes_in_model, heading = None, None
    robot = DEFAULT_ROBOT_PX
    area, cam, heights, classes = Area(), None, {}, classes_in_model
    if proj is not None:
        robot = robot_size(proj, a.robot_px)
        cam_name = pick_camera_name(proj, source.folder if source.kind in ("recording", "video") else None, a.camera_name)
        area = Area.from_project(proj, cam_name, robot)
        cam = load_camera_model(proj, cam_name, source.folder if source.kind == "recording" else None, say)
        heights = heights_for(proj)
        classes = proj.class_names()
        heading = proj.heading_classes()
        say("Project: %s%s" % (proj.name, "" if not model_id else "   model: %s" % model_id))
        if cam_name:
            say("Camera: %s" % cam_name)
        say("Work area: %s" % ("set (guesses outside it are ignored)" if area.polygon else "the whole picture")
            + ("; %d ignored spot(s)" % len(area.spots) if area.spots else ""))
        say("Positions on the floor (cm): %s" % ("yes" if cam is not None else
                                                "no (no floor calibration for camera \"%s\": looked for %s)" % (
                                                    (cam_name or "").strip() or "default", ", ".join(
                                                        camera_model_paths(proj, cam_name, source.folder if source.kind == "recording" else None)))))
    else:
        if a.robot_px:
            robot = a.robot_px
        say("Model file: %s" % onnx)
    pipe = Pipeline(detector, robot, (640, 480), area, cam, heights, classes, heading, a.every,
                    ask_below=getattr(a, "ask_below", ASK_BELOW), ask_after_s=getattr(a, "ask_after", ASK_AFTER_S))
    pipe.camera_key = (cam_name or "").strip() or "default"
    return pipe, proj, model_id


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        ensure_packages()
        if not argv:
            argv = ask_questions()
        a = build_parser().parse_args(argv)
        if not a.source:
            raise UserError("Say what to watch: a recording folder, a video file, a camera number or a camera address.")
        spec = a.source
        if a.project and classify_source(spec) is None:
            p = find_recording(load_project(a.project), spec)
            if p:
                spec = p
        return _run_main(a, spec)
    except UserError as e:
        print("\n" + str(e))
        return 1
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


def _run_main(a, spec, detector=None, say=print, ask=input):
    print("%s - %s" % (PRODUCT, TAGLINE))
    source = open_source(spec)
    try:
        pipe, proj, model_id = make_pipeline(a, source, say, detector)
        out = a.out or default_out(proj, source, model_id, spec)
        show = a.page if a.page is not None else bool(source.live)
        roll = a.roll_call if a.roll_call is not None else bool(source.live and show)
        pipe.check = True if roll else pipe.check
        pipe.outside = load_outside(getattr(a, "outside", None), say, proj=proj, pipe=pipe,
                                    recording=source.folder if source.kind == "recording" else None)
        view = None
        if show:
            view = LiveView(a.port, source.label, classes=pipe.classes, pipe=pipe)
            say("Live page: %s" % view.url)
            if not a.no_browser:
                import webbrowser
                try:
                    webbrowser.open(view.url)
                except Exception:
                    pass
        say("Watching: %s" % source.label)
        say("The table will be saved to: %s" % out)
        if roll:
            say("Roll call first: the objects it sees are shown %s; confirm or fix them once, then tracking starts."
                % ("on the page" if view is not None else "here"))
        if source.live:
            say("Press the Stop button on the page (or Ctrl+C here) to finish.")
        t0 = time.time()
        end_msg = None
        try:
            n = run(source, pipe, out, view, a.seconds, a.max_frames, say, roll=roll, ask=ask)
            end_msg = "Finished. The table is saved in %s" % out
            if n == 0:
                end_msg += ". It has no rows: " + explain_empty(a.min_score)
        except SaveError as e:
            end_msg = "Not saved: %s" % e
            say("")
            say(end_msg)
            return 1
        finally:
            if view is not None:
                view.finish(end_msg or "Stopped before the end; nothing more was saved.")
                time.sleep(1.2)
                view.close()
        dt = time.time() - t0
        say("")
        say("Done: %d rows for %d objects in %s. Saved: %s" % (n, len(pipe.tk.summary()), _nice_time(dt), out))
        if n == 0:
            say(explain_empty(a.min_score))
        return 0
    finally:
        source.close()


if __name__ == "__main__":
    sys.exit(main())
