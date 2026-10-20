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
    --seconds 60          stop after this long;  --max-frames 500 stops after that many frames
    --every 2             look at every 2nd frame only (faster; the objects are carried along)
    --robot-px 30         object size in pixels, if the project does not know it

The table has one row per object per frame:
    frame, t_wall, obj, cls, x_px, y_px, heading_deg, score, x_cm, y_cm, source
x_cm, y_cm (position on the floor) are there only when the project has a floor
calibration for the camera. source says "model" when the object was found in that
frame and "track" when it was carried along from where it was last seen.
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


def load_camera_model(proj, camera):
    """Floor calibration of a project camera (cameras.CameraModel) or None."""
    p = proj.camera_model_path(camera)
    if not p or not os.path.exists(p):
        return None
    import cameras
    try:
        return cameras.CameraModel.load(p)
    except Exception as e:
        print("The floor calibration of camera %s could not be read (%s); positions are given in pixels only." % (camera, e))
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
    work area, and tracks them. Offline, finished_rows() gives every row once all frames are in."""

    def __init__(self, detector, robot_px=DEFAULT_ROBOT_PX, img_size=(640, 480), area=None, camera_model=None,
                 heights=None, classes=None, heading_classes=None, every=1):
        import tracker
        self.detector = detector
        self.area = area or Area()
        self.cam = camera_model
        self.heights = heights or {}
        self.classes = list(classes) if classes else None
        self.heading_classes = set(heading_classes) if heading_classes is not None else None
        self.every = max(1, int(every))
        self.tk = tracker.Tracker(robot_px, img_size, det_source="model")
        self.t_wall = {}
        self.calls = 0

    @property
    def columns(self):
        return BASE_COLUMNS + (CM_COLUMNS if self.cam is not None else []) + ["source"]

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

    def process(self, f, t, t_wall, image):
        """Track one frame. Returns this frame's objects: dicts with the table columns plus
        colour (green/amber/red) and age."""
        self.t_wall[f] = t_wall
        dets = None
        if f % self.every == 0 or self.calls == 0:
            dets = self._clean(self.detector(image))
            self.calls += 1
        rows = self.tk.step(f, t, dets)
        by_id = {tr.id: tr for tr in self.tk.tracks}
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
        return rec

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
  main { display:grid; grid-template-columns:1fr 310px; min-height:0; }
  #stage { padding:10px; display:flex; align-items:center; justify-content:center; min-width:0; min-height:0; }
  canvas { background:#000; border-radius:4px; max-width:100%; max-height:100%; }
  aside { background:var(--panel); border-left:1px solid var(--line); overflow-y:auto; padding:12px; }
  aside h2 { font-size:12px; text-transform:uppercase; letter-spacing:.06em; color:var(--dim); margin:16px 0 6px; }
  aside h2:first-child { margin-top:0; }
  .fps { font-size:30px; font-weight:600; } .fps small { font-size:13px; color:var(--dim); font-weight:400; }
  .row { display:grid; grid-template-columns:12px 1fr auto; gap:8px; align-items:center; padding:5px 7px;
    border-radius:6px; background:var(--panel2); margin-bottom:4px; font-size:13px; }
  .dot { width:12px; height:12px; border-radius:50%; }
  .green { background:var(--green); } .amber { background:var(--amber); } .red { background:var(--red); }
  .muted { color:var(--dim); font-size:12px; }
  .legend { display:grid; grid-template-columns:12px 1fr; gap:6px 8px; font-size:12.5px; align-items:start; }
  .legend .dot { margin-top:3px; }
  .bar { height:10px; background:var(--panel2); border-radius:5px; overflow:hidden; margin:6px 0 2px; }
  .bar span { display:block; height:100%; background:var(--accent); width:0; }
  button { width:100%; margin-top:14px; padding:10px; background:#5a2326; color:var(--text); border:1px solid #8a3a3e;
    border-radius:6px; cursor:pointer; font:inherit; font-weight:600; }
  button:hover { border-color:var(--red); } button:disabled { opacity:.45; cursor:default; }
  #msg { margin-top:10px; color:var(--accent); }
</style></head><body>
<header><h1>Maxfield IRL Tracking</h1><span class="tag">Identify. Recognize. Label.</span><span class="src" id="src"></span></header>
<main>
  <div id="stage"><canvas id="view" width="640" height="480"></canvas></div>
  <aside>
    <h2>Speed</h2>
    <div class="fps"><span id="fps">-</span> <small>frames per second</small></div>
    <div class="muted" id="count"></div>
    <div id="progress" style="display:none"><div class="bar"><span id="pbar"></span></div><div class="muted" id="ptxt"></div></div>
    <h2>Objects in this picture</h2>
    <div id="objs"><div class="muted">Waiting for the first picture ...</div></div>
    <h2>What the colours mean</h2>
    <div class="legend" id="legend"></div>
    <button id="stop">Stop</button>
    <div id="msg"></div>
  </aside>
</main>
<script>
const COL = {green:"#3ddc97", amber:"#ffb347", red:"#ff6b6b"};
const legend = __LEGEND__;
document.getElementById("legend").innerHTML = legend.map(l =>
  '<span class="dot '+l[0]+'"></span><span>'+l[1]+'</span>').join("");
const cv = document.getElementById("view"), cx = cv.getContext("2d");
let since = -1, done = false, img = new Image();
function draw(s) {
  if (s.w && (cv.width !== s.w || cv.height !== s.h)) { cv.width = s.w; cv.height = s.h; }
  cx.drawImage(img, 0, 0, cv.width, cv.height);
  for (const o of s.objects) {
    const c = COL[o.colour] || "#fff";
    cx.strokeStyle = c; cx.fillStyle = c; cx.lineWidth = 2;
    cx.beginPath(); cx.arc(o.x, o.y, 16, 0, 6.2832); cx.stroke();
    if (o.heading !== null) {
      const a = o.heading * Math.PI / 180;
      cx.beginPath(); cx.moveTo(o.x, o.y); cx.lineTo(o.x + 28 * Math.cos(a), o.y - 28 * Math.sin(a)); cx.stroke();
    }
    cx.font = "13px system-ui,sans-serif";
    cx.fillText("#" + o.obj + " " + o.cls, o.x + 18, o.y - 14);
  }
}
function side(s) {
  document.getElementById("fps").textContent = s.fps ? s.fps.toFixed(1) : "-";
  document.getElementById("src").textContent = s.source || "";
  document.getElementById("count").textContent = "Frame " + s.frame + (s.total ? " of " + s.total : "");
  const p = document.getElementById("progress");
  if (s.total) { p.style.display = "block"; document.getElementById("pbar").style.width = (100 * s.frame / s.total) + "%";
    document.getElementById("ptxt").textContent = s.left ? "About " + s.left + " left" : ""; }
  const box = document.getElementById("objs");
  box.innerHTML = s.objects.length ? s.objects.map(o =>
    '<div class="row"><span class="dot '+o.colour+'"></span><span>#'+o.obj+' '+o.cls+'</span><span class="muted">'+
    Math.round(o.score*100)+'%</span></div>').join("") : '<div class="muted">No objects found in this picture.</div>';
}
function finish(text) {
  done = true; document.getElementById("msg").textContent = text;
  document.getElementById("stop").disabled = true;
}
async function loop() {
  while (!done) {
    try {
      const r = await fetch("/state?since=" + since);
      const s = await r.json();
      if (s.done) { finish(s.message || "Finished."); }
      if (s.same) { continue; }
      since = s.frame;
      await new Promise(res => { img.onload = res; img.onerror = res; img.src = s.image; });
      draw(s); side(s);
    } catch (e) {
      if (since >= 0) { finish("Finished. You can close this page."); } else { await new Promise(r => setTimeout(r, 500)); }
    }
  }
}
document.getElementById("stop").onclick = async () => {
  document.getElementById("stop").disabled = true;
  document.getElementById("msg").textContent = "Stopping and saving ...";
  try { await fetch("/stop", {method:"POST"}); } catch (e) {}
};
loop();
</script></body></html>
"""


class LiveView:
    """The small local web page. publish() is called by the tracking loop for every frame; the page
    asks for news with /state (waits up to a second for a new frame) and stops the run with /stop."""

    def __init__(self, port=0, source_label=""):
        self.cond = threading.Condition()
        self.stop_event = threading.Event()
        self.source_label = source_label
        self.seq, self.image, self.objects, self.fps = -1, None, [], 0.0
        self.total, self.left, self.message, self.done = None, "", "", False
        self.size = (640, 480)
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

            def do_GET(self):
                path, _, qs = self.path.partition("?")
                if path == "/":
                    page = PAGE.replace("__LEGEND__", json.dumps(LEGEND))
                    self._send(200, page.encode("utf-8"), "text/html; charset=utf-8")
                elif path == "/state":
                    m = re.search(r"since=(-?\d+)", qs)
                    self._send(200, json.dumps(view.state(int(m.group(1)) if m else -1)).encode("utf-8"),
                               "application/json")
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self):
                if self.path.split("?")[0] == "/stop":
                    view.stop_event.set()
                    self._send(200, b"{}", "application/json")
                else:
                    self._send(404, b"not found", "text/plain")

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        self.url = "http://127.0.0.1:%d/" % self.port
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def publish(self, f, image, objects, fps, total=None, left=""):
        with self.cond:
            self.seq, self.image, self.fps, self.total, self.left = f, image, fps, total, left
            self.size = (image.shape[1], image.shape[0])
            self.objects = [dict(obj=o["obj"], cls=o["cls"], x=o["x_px"], y=o["y_px"], heading=o["heading_deg"],
                                 score=o["score"], colour=o["colour"]) for o in objects]
            self.cond.notify_all()

    def finish(self, message):
        with self.cond:
            self.done, self.message = True, message
            self.cond.notify_all()

    def state(self, since):
        with self.cond:
            if self.seq == since and not self.done:
                self.cond.wait(1.0)
            if self.done and self.seq == since:
                return dict(same=True, done=True, message=self.message)
            if self.seq < 0 or self.seq == since:
                return dict(same=True, done=self.done, message=self.message)
            img, objs, f, fps, total, left = self.image, list(self.objects), self.seq, self.fps, self.total, self.left
            w, h = self.size
            done, message = self.done, self.message
        import cv2
        import base64
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        data = "data:image/jpeg;base64," + base64.b64encode(jpg.tobytes()).decode("ascii") if ok else ""
        return dict(frame=f, image=data, objects=objs, fps=fps, total=total, left=left, w=w, h=h,
                    source=self.source_label, done=done, message=message)

    def close(self):
        self.server.shutdown()
        self.server.server_close()


# --------------------------------------------------------------------------------------------
# Running
# --------------------------------------------------------------------------------------------

def _nice_time(s):
    s = int(round(s))
    if s < 60:
        return "%d s" % s
    return "%d min %02d s" % (s // 60, s % 60)


def run(source, pipe, out_path, view=None, seconds=0, max_frames=0, say=print, stop=None, quiet_progress=False):
    """Watch the source and write the table. Returns the number of rows written.
    Recordings and video files: the table is written at the end (so the first frames of each object
    are filled in). Cameras: every row is written as it happens."""
    stop = stop or (view.stop_event if view else threading.Event())
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    cols = pipe.columns
    live = bool(source.live)
    fh = None
    writer = None
    n_rows = 0
    if live:
        fh = open(out_path, "w", newline="", encoding="utf-8")
        writer = csv.writer(fh)
        writer.writerow(cols)
    t_start = time.time()
    stamps = []
    last_say = 0.0
    frames = 0
    try:
        while not stop.is_set():
            if max_frames and frames >= max_frames:
                break
            if seconds and time.time() - t_start >= seconds:
                break
            item = source.read()
            if item is None:
                if live and not stop.is_set() and time.time() - t_start < 3 and frames == 0:
                    raise UserError("No pictures are coming from %s. Check that it is on and connected." % source.label)
                break
            f, t, tw, img = item
            objs = pipe.process(f, t, tw, img)
            frames += 1
            now = time.time()
            stamps.append(now)
            if len(stamps) > 30:
                stamps.pop(0)
            fps = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0
            left = ""
            total = source.n if not live else None
            if total and fps > 0:
                left = _nice_time(max(0, total - f - 1) / fps)
            if live:
                for o in objs:
                    writer.writerow(csv_line(o, cols))
                    n_rows += 1
                if frames % 20 == 0:
                    fh.flush()
            if view is not None:
                view.publish(f, img, objs, fps, total, left)
            if not quiet_progress and now - last_say >= 0.5:
                last_say = now
                if total:
                    say("  frame %d of %d (%d%%), %.1f frames a second, about %s left"
                        % (f + 1, total, 100 * (f + 1) // total, fps, left or "?"))
                else:
                    say("  %d frames so far, %.1f frames a second, %d objects in the last picture" % (frames, fps, len(objs)))
        if not live:
            rows = pipe.finished_rows()
            with open(out_path, "w", newline="", encoding="utf-8") as fh2:
                w2 = csv.writer(fh2)
                w2.writerow(cols)
                for rec in rows:
                    w2.writerow(csv_line(rec, cols))
            n_rows = len(rows)
    finally:
        if fh is not None:
            fh.close()
    return n_rows


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
    ap.add_argument("--seconds", type=float, default=0, help="stop after this many seconds")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after this many frames")
    ap.add_argument("--every", type=int, default=1, help="look at every n-th frame only")
    ap.add_argument("--robot-px", type=float, help="object size in pixels")
    ap.add_argument("--min-score", type=float, help="ignore model guesses below this score (0 to 1)")
    ap.add_argument("--threads", type=int, help="processor threads for the model")
    return ap


def default_out(proj, source, model_id, spec):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    if source.live:
        base = "live-%s" % stamp
    else:
        base = re.sub(r"[^A-Za-z0-9_.-]+", "-", (source.label or "run")).strip("-.") or "run"
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
        cam = load_camera_model(proj, cam_name)
        heights = heights_for(proj)
        classes = proj.class_names()
        heading = proj.heading_classes()
        say("Project: %s%s" % (proj.name, "" if not model_id else "   model: %s" % model_id))
        if cam_name:
            say("Camera: %s" % cam_name)
        say("Work area: %s" % ("set (guesses outside it are ignored)" if area.polygon else "the whole picture")
            + ("; %d ignored spot(s)" % len(area.spots) if area.spots else ""))
        say("Positions on the floor (cm): %s" % ("yes" if cam is not None else
                                                "no (no floor calibration for this camera in the project)"))
    else:
        if a.robot_px:
            robot = a.robot_px
        say("Model file: %s" % onnx)
    return Pipeline(detector, robot, (640, 480), area, cam, heights, classes, heading, a.every), proj, model_id


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


def _run_main(a, spec, detector=None, say=print):
    print("%s - %s" % (PRODUCT, TAGLINE))
    source = open_source(spec)
    try:
        pipe, proj, model_id = make_pipeline(a, source, say, detector)
        out = a.out or default_out(proj, source, model_id, spec)
        show = a.page if a.page is not None else bool(source.live)
        view = None
        if show:
            view = LiveView(a.port, source.label)
            say("Live page: %s" % view.url)
            if not a.no_browser:
                import webbrowser
                try:
                    webbrowser.open(view.url)
                except Exception:
                    pass
        say("Watching: %s" % source.label)
        say("The table will be saved to: %s" % out)
        if source.live:
            say("Press the Stop button on the page (or Ctrl+C here) to finish.")
        t0 = time.time()
        try:
            n = run(source, pipe, out, view, a.seconds, a.max_frames, say)
        finally:
            if view is not None:
                view.finish("Finished. The table is saved in %s" % out)
                time.sleep(1.2)
                view.close()
        dt = time.time() - t0
        say("")
        say("Done: %d rows for %d objects in %s. Saved: %s" % (n, len(pipe.tk.summary()), _nice_time(dt), out))
        return 0
    finally:
        source.close()


if __name__ == "__main__":
    sys.exit(main())
