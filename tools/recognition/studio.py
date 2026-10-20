#!/usr/bin/env python3
"""Recognition studio: label, train and compare object detectors in one window.

    python tools/recognition/studio.py <project folder>                 open (or create) a project
    python tools/recognition/studio.py <project folder> --add <recording or session folder> [--camera overhead]
    python tools/recognition/studio.py <project folder> --relink <folder the recordings were copied to>

A project folder holds everything: object types, labels, every trained model and its
scores (see project.py). The browser page is the labelling page (tools/label) with a
studio bar on top:

- Recording: switch between recordings (and cameras).
- Guesses from: "No model" shows the system starting from scratch (motion and tracking
  only); any trained model version can be picked instead. Your labels never change when
  you switch; only the unconfirmed guesses do.
- Train new model: trains on every confirmed frame, in the background, while you keep
  labelling. Every model is scored on the same held-back frames (trainer.py).
- Models & learning curve: how the guesses improve as confirmed frames grow.
- Object types: name, has a front (heading), size, height above the floor.
- Add recording: a recording folder, or a two-camera session folder.

With two cameras calibrated (cameras/<name>.json, see CROSSCAM.md), labels confirmed in
one camera become ready-made guesses in the other camera's frames (source "camera").

Only this computer can reach the page (127.0.0.1). Ctrl+C stops; everything is saved.
"""
import argparse
import csv
import json
import os
import sys
import threading
import time
import traceback
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
LABEL = os.path.join(TOOLS, "label")
DETECTOR = os.path.join(TOOLS, "detector")
for p in (HERE, LABEL, DETECTOR):
    if p not in sys.path:
        sys.path.insert(0, p)

import app as labelapp   # noqa: E402
import project as projmod  # noqa: E402
import trainer as trainmod  # noqa: E402

INFER = os.path.join(DETECTOR, "infer.py")


class Studio:
    def __init__(self, proj, log=print):
        self.proj = proj
        self.log = log
        self.session = None
        self.loading = None
        self.lock = threading.RLock()
        self.trainer = trainmod.Trainer(proj, log=log)
        self.cross_note = None

    # --- opening a recording with a guess source ------------------------------------------
    def current(self):
        return None if self.loading else self.session

    def open(self, rid, model="none", wait=False):
        proj = self.proj
        if not rid:
            raise ValueError("add a recording first")
        rec = proj.rec(rid)
        if not os.path.exists(os.path.join(rec["path"], "video.avi")):
            raise ValueError("can't find the recording %s (moved? use --relink)" % rec["path"])
        weights = None
        if model and model != "none":
            m = proj.model(model)
            if m.get("status") != "ready":
                raise ValueError("model %s is not ready" % model)
            if m.get("classes") != proj.class_names():
                raise ValueError("model %s was trained on other object types (%s); train a new one"
                                 % (model, ", ".join(m["classes"])))
            weights = os.path.join(proj.folder, m["onnx"])
        with self.lock:
            if self.loading:
                raise ValueError("still opening the last one; wait a moment")
            self.loading = dict(what="Opening %s ..." % rec["name"], done=0, total=0)
            self.session = None
        t = threading.Thread(target=self._open, args=(rid, model or "none", weights), daemon=True)
        t.start()
        if wait:
            t.join()

    def _open(self, rid, model, weights):
        proj = self.proj
        rec = proj.rec(rid)

        def progress(what, done, total):
            names = {"motion": "Finding moving objects (first time only)", "model": "Running model %s" % model}
            self.loading.update(what=names.get(what, what), done=done, total=total)

        try:
            extra = self._cross_camera_guesses(rec)
            s = labelapp.Session(rec["path"], model=INFER if weights else None, weights=weights,
                                 classes=proj.class_names(), labels_dir=proj.labels_dir(rid), extra_dets=extra,
                                 progress=progress, log=self.log)
            with self.lock:
                self.session = s
                proj.data["current"] = dict(rec=rid, model=model)
                proj.save()
                self.loading = None
        except Exception as e:
            traceback.print_exc()
            with self.lock:
                self.loading = dict(what="Could not open %s" % rec["name"], error=str(e), done=0, total=0)
            time.sleep(6)
            with self.lock:
                self.loading = None

    def _cross_camera_guesses(self, rec):
        """Confirmed labels from the other cameras of the same session, moved into this camera's frames."""
        self.cross_note = None
        if not rec.get("session"):
            return None
        try:
            import cameras
            import crosscam
        except ImportError:
            return None
        proj = self.proj
        mine = proj.camera_model_path(rec.get("camera"))
        if not mine or not os.path.exists(mine):
            return None
        cam_b = cameras.CameraModel.load(mine)
        # t_wall: the Pi's clock, shared by every camera on it (t_mono is shared too within one
        # two-camera session, but not between separately started recordings)
        times_b = crosscam.load_frame_times(os.path.join(rec["path"], "frames.csv"))
        b_meta = _meta(rec["path"])
        out, n = {}, 0
        for other in proj.recordings():
            if other["id"] == rec["id"] or other.get("session") != rec["session"]:
                continue
            path_a = proj.camera_model_path(other.get("camera"))
            if not path_a or not os.path.exists(path_a):
                continue
            objs = _confirmed_objects(proj.labels_dir(other["id"]))
            if not objs:
                continue
            cam_a = cameras.CameraModel.load(path_a)
            g = crosscam.cross_guesses(objs, cam_a, cam_b, crosscam.load_frame_times(os.path.join(other["path"], "frames.csv")), times_b,
                                       (b_meta.get("width", 640), b_meta.get("height", 480)), proj.class_heights())
            for f, ds in g.items():
                out.setdefault(f, []).extend(ds)
                n += len(ds)
        if n:
            self.cross_note = "%d guesses from the other camera" % n
            self.log("Cross-camera: " + self.cross_note)
        return out or None

    # --- state for the page -----------------------------------------------------------------
    def state(self):
        proj = self.proj
        recs = []
        total = 0
        for r in proj.recordings():
            n = _done_frames(proj.labels_dir(r["id"]))
            total += n
            recs.append(dict(id=r["id"], name=r["name"], camera=r.get("camera", ""), labelled=n,
                             missing=not os.path.exists(os.path.join(r["path"], "video.avi"))))
        models = [dict(id=m["id"], status=m["status"], n_train_frames=m.get("n_train_frames", 0),
                       metrics=m.get("metrics", {}), created=m.get("created"), error=m.get("error"),
                       compatible=m.get("classes") == proj.class_names()) for m in proj.models()]
        models = [m for m in models if m["status"] != "ready" or m["compatible"]] + \
                 [m for m in models if m["status"] == "ready" and not m["compatible"]]
        job = self.trainer.job
        return dict(project=dict(name=proj.name, folder=proj.folder), recordings=recs, models=models,
                    current=proj.data.get("current", {}), classes=proj.class_view(), job=job,
                    loading=self.loading, settings=proj.data.get("settings", {}), labelled_total=total,
                    cross_note=self.cross_note)

    def curve(self):
        p = os.path.join(self.proj.folder, "models", "curve.json")
        if not os.path.exists(p):
            return dict(points=[], test=dict(frames=0, note="no model trained yet"))
        with open(p) as f:
            return json.load(f)


def _meta(folder):
    p = os.path.join(folder, "meta.json")
    if os.path.exists(p):
        with open(p) as f:
            return json.load(f)
    return {}


def _done_frames(label_dir):
    p = os.path.join(label_dir, "review.json")
    if not os.path.exists(p):
        return 0
    try:
        with open(p) as f:
            fr = json.load(f).get("frames", {})
        return sum(1 for v in fr.values() if v.get("state") in ("reviewed", "auto"))
    except ValueError:
        return 0


def _confirmed_objects(label_dir):
    """frame -> objects (as in app.Session.frames) for frames a person finished."""
    p = os.path.join(label_dir, "labels.csv")
    if not os.path.exists(p):
        return {}
    out = {}
    with open(p, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r["status"] not in ("confirmed", "corrected", "auto") or r["cls"] == "none":
                continue
            h = r["heading_deg"]
            out.setdefault(int(r["frame"]), []).append(dict(
                obj=int(r["obj"]) if r["obj"] else None, cls=r["cls"], x=float(r["x_px"]), y=float(r["y_px"]),
                heading_deg=float(h) if h else None, heading_known=r["heading_known"] == "1",
                status="confirmed" if r["status"] == "auto" else r["status"],
                score=float(r["score"]) if r["score"] else None))
    return out


def make_routes(studio):
    def get(h, path, q):
        if path == "/api/studio/state":
            h._send(200, studio.state())
        elif path == "/api/studio/curve":
            h._send(200, studio.curve())
        elif path == "/report":
            import report
            h._send(200, report.html(studio.proj), "text/html; charset=utf-8")
        else:
            return False
        return True

    def post(h, path, b):
        if path == "/api/studio/open":
            studio.open(b.get("rec"), b.get("model", "none"))
            h._send(200, dict(ok=True))
        elif path == "/api/studio/train":
            mid = studio.trainer.start(float(b.get("minutes") or 20))
            studio.proj.data["settings"]["train_minutes"] = float(b.get("minutes") or 20)
            studio.proj.save()
            h._send(200, dict(model=mid))
        elif path == "/api/studio/classes":
            studio.proj.set_classes(b.get("classes") or [])
            cur = studio.proj.data.get("current", {})
            model = cur.get("model", "none")
            if model != "none":
                try:
                    if studio.proj.model(model).get("classes") != studio.proj.class_names():
                        model = "none"
                except KeyError:
                    model = "none"
            studio.open(cur.get("rec"), model, wait=True)
            h._send(200, dict(ok=True))
        elif path == "/api/studio/add":
            added = studio.proj.add_recording(b.get("path", ""), b.get("camera") or None)
            if added and studio.session is None and not studio.loading:
                studio.open(added[0], "none")
            h._send(200, dict(added=len(added), ids=added))
        else:
            return False
        return True

    return get, post


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", help="project folder (created if it doesn't exist)")
    ap.add_argument("--name", help="project name, when creating it")
    ap.add_argument("--add", nargs="+", metavar="FOLDER", help="add recording (or two-camera session) folders, then open")
    ap.add_argument("--camera", help="camera name for --add of a single recording (e.g. overhead)")
    ap.add_argument("--relink", metavar="FOLDER", help="find moved recordings under this folder")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()

    proj = projmod.Project(a.project, create_name=a.name or os.path.basename(os.path.abspath(a.project)))
    if a.relink:
        print("Found %d moved recordings." % proj.relink(a.relink))
    for folder in a.add or []:
        ids = proj.add_recording(folder, a.camera)
        print("Added %d recording(s) from %s" % (len(ids), folder))
    studio = Studio(proj)
    cur = proj.data.get("current", {})
    if cur.get("rec"):
        model = cur.get("model", "none")
        try:
            studio.open(cur["rec"], model)
        except (ValueError, KeyError) as e:
            print("Opening %s with no model (%s)" % (cur["rec"], e))
            studio.open(cur["rec"], "none")
    get, post = make_routes(studio)
    server, url = labelapp.serve(studio.current, a.port, extra_get=get, extra_post=post)
    print("\nProject: %s  (%s)" % (proj.name, proj.folder))
    print("%d recording(s), %d model(s). Open %s  (Ctrl+C stops; everything is already saved)"
          % (len(proj.recordings()), len(proj.models()), url))
    if not proj.recordings():
        print("No recordings yet: press 'Add recording' on the page, or start again with --add <folder>.")
    if not a.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped. Everything is saved.")
    finally:
        server.server_close()


if __name__ == "__main__":
    sys.exit(main())
