#!/usr/bin/env python3
"""Maxfield IRL Tracking (Identify. Recognize. Label.): the recognition studio.
Label, train and compare object detectors in one window.

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
- Object types: name, has a front (heading), size, height above the floor (or "it varies").
- Add recording: a recording folder, or a two-camera session folder.
- Work area: per camera, the part of the picture to look in, plus ignore spots.
- Investigate (key Q on the page): one spot across the whole recording; "Ignore this spot", and
  the second camera as a referee when both cameras are calibrated.
- Time saved: seconds per frame and frames accepted without looking, by model version (CSV too).
- Share project / Open shared project: the whole project as one .mxirl file (share.py).

A new project first asks for its name and object types (setup), then shows a short tour.

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
import timesaved  # noqa: E402
import timing  # noqa: E402


class RecordingMissing(ValueError):
    """The recording's folder (video.avi) isn't where the project says: the page offers to locate it."""

    def __init__(self, rec):
        self.rec = rec
        ValueError.__init__(self, "Recording not found: %s. It was at %s" % (rec.get("name") or rec["id"], rec["path"]))

INFER = os.path.join(DETECTOR, "infer.py")


class Studio:
    def __init__(self, proj, log=print, background=True):
        self.proj = proj
        self.log = log
        self.session = None
        self.loading = None
        self.lock = threading.RLock()
        self.trainer = trainmod.Trainer(proj, log=log)
        self.cross_note = None
        self.background = background     # run the other cameras' motion pass in the background
        self.bg = None                   # what the background pass is doing (shown in the top bar)
        self.bg_thread = None
        self.pending_reopen = False      # settings changed while opening: open again when done
        self.share = None                # Share project / Open shared project: what it is doing

    # --- opening a recording with a guess source ------------------------------------------
    def current(self):
        return None if (self.loading or self.pending_reopen) else self.session

    def loading_view(self):
        if self.loading:
            return self.loading
        if self.pending_reopen:
            return dict(what="Applying your changes ...", done=0, total=0, left_s=None, left="")
        return None

    def open(self, rid, model="none", wait=False):
        proj = self.proj
        if not rid:
            raise ValueError("add a recording first")
        rec = proj.rec(rid)
        if not os.path.exists(os.path.join(rec["path"], "video.avi")):
            raise RecordingMissing(rec)
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
            self.loading = dict(what="Opening %s ..." % rec["name"], done=0, total=0, left_s=None)
            self.session = None
        t = threading.Thread(target=self._open, args=(rid, model or "none", weights), daemon=True)
        t.start()
        if wait:
            t.join()

    def reopen(self, only_camera=None):
        """Open the current recording again (after object types or the work area changed), with
        the current model if it still fits the object types. Returns True if it is reopening."""
        proj = self.proj
        cur = proj.data.get("current", {})
        if not cur.get("rec"):
            return False
        try:
            rec = proj.rec(cur["rec"])
        except KeyError:
            return False
        if only_camera is not None and proj.camera_key(rec) != only_camera:
            return False
        model = cur.get("model", "none")
        if model != "none":
            try:
                if proj.model(model).get("classes") != proj.class_names():
                    model = "none"
            except KeyError:
                model = "none"
        with self.lock:
            busy = self.loading is not None
            if busy:
                # still opening: open again as soon as that is done (with the new settings)
                self.pending_reopen = True

        def later():
            while self.loading is not None:
                time.sleep(0.2)
            try:
                self.open(cur["rec"], model)
            finally:
                self.pending_reopen = False
        if busy:
            threading.Thread(target=later, daemon=True).start()
        else:
            self.open(cur["rec"], model)
        return True

    def _open(self, rid, model, weights):
        proj = self.proj
        rec = proj.rec(rid)
        clock = dict(what=None, clock=None)

        def progress(what, done, total):
            names = {"motion": "Finding moving objects (first time only)", "model": "Running model %s" % model,
                     "pick": "Picking frames to label"}
            what = names.get(what, what)
            if clock["what"] != what:
                clock.update(what=what, clock=labelapp.RateClock())
            left = clock["clock"].update(done, total) if total else None
            self.loading = dict(what=what, done=done, total=total, left_s=left, left=labelapp.fmt_left(left))

        try:
            self._wait_background(rid)
            extra = self._cross_camera_guesses(rec)
            s = labelapp.Session(rec["path"], model=INFER if weights else None, weights=weights,
                                 classes=proj.class_names(), labels_dir=proj.labels_dir(rid), extra_dets=extra,
                                 progress=progress, log=self.log, class_map=proj.motion_class_map(),
                                 loose_classes=proj.loose_classes(), front_classes=proj.heading_classes(),
                                 area=proj.camera_settings(proj.camera_key(rec)),
                                 stats_path=timesaved.stats_path(proj, rid), model_id=model)
            with self.lock:
                self.session = s
                proj.data["current"] = dict(rec=rid, model=model)
                proj.save()
                self.loading = None
            self._start_background(rid)
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
                                       (b_meta.get("width", 640), b_meta.get("height", 480)), proj.class_heights(),
                                       skip_classes=tuple(proj.varies_classes()))
            for f, ds in g.items():
                out.setdefault(f, []).extend(ds)
                n += len(ds)
        if n:
            self.cross_note = "%d guesses from the other camera" % n
            self.log("Cross-camera: " + self.cross_note)
        return out or None

    # --- the other cameras' motion pass, in the background ------------------------------------
    def _start_background(self, rid):
        """After a recording of a two-camera session opens, find the moving objects in the
        session's other cameras in the background, so switching to them is quick."""
        if not self.background or (self.bg_thread and self.bg_thread.is_alive()):
            return
        rec = self.proj.rec(rid)
        if not rec.get("session"):
            return
        todo = []
        for other in self.proj.recordings():
            if other["id"] == rid or other.get("session") != rec["session"]:
                continue
            if not os.path.exists(os.path.join(other["path"], "video.avi")):
                continue
            try:
                r = labelapp.Recording(other["path"])
            except (SystemExit, Exception):
                continue
            if not labelapp.has_motion_cache(r, os.path.join(self.proj.labels_dir(other["id"]), "cache")):
                todo.append((other, r))
        if not todo:
            return
        self.bg_thread = threading.Thread(target=self._background, args=(todo,), daemon=True)
        self.bg_thread.start()

    def _background(self, todo):
        for other, r in todo:
            clock = labelapp.RateClock()
            name = "%s%s" % (other["name"], " [%s]" % other["camera"] if other.get("camera") else "")
            self.bg = dict(rid=other["id"], what="Getting %s ready: finding moving objects" % name, done=0,
                           total=r.n, left_s=None, left="")

            def progress(done, total, bg=self.bg, clock=clock):
                left = clock.update(done, total)
                bg.update(done=done, total=total, left_s=left, left=labelapp.fmt_left(left))
            try:
                labelapp.motion_guesses(r, os.path.join(self.proj.labels_dir(other["id"]), "cache"), None, progress,
                                        log=lambda *a: None)
                self.log("Background: %s is ready." % name)
            except Exception as e:      # the normal open will try again (and show the problem)
                self.log("Background pass for %s stopped: %s" % (name, e))
        self.bg = None

    def _wait_background(self, rid):
        """Opening a recording whose background pass is running: wait for it, showing its progress."""
        while self.bg_thread and self.bg_thread.is_alive() and self.bg and self.bg.get("rid") == rid:
            b = dict(self.bg)
            self.loading = dict(what="Finding moving objects (first time only)", done=b.get("done", 0),
                                total=b.get("total", 0), left_s=b.get("left_s"), left=b.get("left", ""))
            time.sleep(0.3)

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
        cur = proj.data.get("current", {})
        camera = None
        if cur.get("rec"):
            try:
                rec = proj.rec(cur["rec"])
                key = proj.camera_key(rec)
                camera = dict(key=key, name=rec.get("camera") or "", settings=proj.camera_settings(key))
            except KeyError:
                pass
        return dict(brand=dict(name=labelapp.BRAND, tagline=labelapp.TAGLINE),
                    project=dict(name=proj.name, folder=proj.folder), recordings=recs, models=models,
                    current=cur, classes=proj.class_view(), job=job,
                    loading=self.loading_view(), settings=proj.data.get("settings", {}), labelled_total=total,
                    cross_note=self.cross_note, setup=proj.setup_view(), camera=camera, background=self.bg,
                    share=self.share_view(), missing=[r for r in recs if r["missing"]],
                    documents=_documents())

    # --- investigate: ignore spot, second camera ------------------------------------------------
    def ignore_spot(self, camera, x, y, r):
        """Add an ignore spot to the camera's work area, then open the recording again with it."""
        key = camera or "default"
        c = self.proj.camera_settings(key)
        spots = list(c["ignore_spots"]) + [[float(x), float(y), float(r)]]
        self.proj.set_camera_settings(key, c["work_area"], c["margin_px"], spots, c["image_size"], asked=c["asked"])
        return self.reopen(only_camera=key)

    def referee(self, frame, x, y, cls):
        import referee
        cur = self.proj.data.get("current", {})
        if not cur.get("rec"):
            return dict(available=False, reason="No recording open.")
        try:
            return referee.look(self.proj, self.proj.rec(cur["rec"]), int(frame), float(x), float(y), cls)
        except Exception as e:          # the rest of Investigate keeps working
            traceback.print_exc()
            return dict(available=False, reason="The second camera could not be used: %s" % e)

    # --- share / open a project file ----------------------------------------------------------
    def share_view(self):
        return dict(self.share) if self.share else None

    def _share_job(self, what, fn):
        if self.share and self.share.get("status") == "running":
            raise ValueError("still busy with the last project file; wait a moment")
        clock = labelapp.RateClock(window=10.0)
        job = dict(status="running", what=what, done=0, total=0, left="", result=None, error=None)
        self.share = job

        def progress(done, total):
            job.update(done=done, total=total, left=labelapp.fmt_left(clock.update(done, total)))

        def work():
            try:
                job.update(result=fn(progress), status="done")
            except Exception as e:
                if not isinstance(e, ValueError):
                    traceback.print_exc()
                job.update(status="failed", error=str(e))
        threading.Thread(target=work, daemon=True).start()
        return job

    def export(self, path, with_video=False):
        import share
        path = (path or "").strip().strip('"')
        if not path:
            raise ValueError("Say where to save the file.")

        def run(progress):
            res = share.export_project(self.proj, os.path.expanduser(path), with_video=bool(with_video), progress=progress)
            m = res["manifest"]
            return dict(path=res["path"], mb=round(res["bytes"] / 1e6, 1), recordings=len(m["recordings"]),
                        models=m["contents"]["models"], with_video=m["with_video"], warnings=m["warnings"])
        return self._share_job("Saving the project as one file", run)

    def import_file(self, file, dest, recordings=None):
        import share
        file = os.path.expanduser((file or "").strip().strip('"'))
        dest = os.path.expanduser((dest or "").strip().strip('"'))
        recordings = os.path.expanduser((recordings or "").strip().strip('"')) or None
        if not file or not os.path.isfile(file):
            raise ValueError("There is no file at %s." % (file or "(empty)"))
        if not dest:
            raise ValueError("Say which new folder the project should go in.")
        share.read_manifest(file)          # a plain message right away if it isn't a project file

        def run(progress):
            res = share.import_project(file, dest, recordings, progress=progress)
            return dict(dest=res["dest"], name=res["manifest"].get("project_name"), recordings=res["recordings"],
                        found=res["found"], notes=res["notes"],
                        missing=[dict(m, id=m["id"]) for m in res["missing"]])
        return self._share_job("Opening the project file", run)

    def switch_project(self, folder):
        """Open another project folder in this studio (after Open shared project)."""
        if self.trainer.busy():
            raise ValueError("A model is still training in this project; wait until it is done.")
        if self.loading:
            raise ValueError("still opening the last one; wait a moment")
        proj = projmod.Project(os.path.expanduser(folder))
        with self.lock:
            self.proj = proj
            self.trainer = trainmod.Trainer(proj, log=self.log)
            self.session = None
            self.cross_note = None
        cur = proj.data.get("current", {})
        opened = False
        if cur.get("rec"):
            try:
                self.open(cur["rec"], cur.get("model", "none"))
                opened = True
            except RecordingMissing:
                pass
            except (ValueError, KeyError):
                try:
                    self.open(cur["rec"], "none")
                    opened = True
                except (ValueError, KeyError):
                    pass
        return opened

    def relink(self, rid=None, path=None, folder=None):
        """Locate a missing recording: its own folder (rid + path), or a folder to search for all of them."""
        import share
        if folder:
            found, still = share.relink_from_folder(self.proj, os.path.expanduser(folder.strip().strip('"')))
            return dict(found=found, missing=still, notes=share.describe_missing(still))
        rec = share.relink(self.proj, rid, path)
        return dict(found=[rec["id"]], missing=share.missing_recordings(self.proj),
                    notes=share.describe_missing(share.missing_recordings(self.proj)))

    def curve(self):
        p = os.path.join(self.proj.folder, "models", "curve.json")
        if not os.path.exists(p):
            return dict(points=[], test=dict(frames=0, note="no model trained yet"))
        with open(p) as f:
            return json.load(f)


def _documents():
    """Where to suggest saving a project file: the Documents folder (or the home folder)."""
    docs = os.path.join(os.path.expanduser("~"), "Documents")
    return docs if os.path.isdir(docs) else os.path.expanduser("~")


def _download(h, data, name, ctype="text/csv; charset=utf-8"):
    body = data.encode("utf-8")
    h.send_response(200)
    h.send_header("Content-Type", ctype)
    h.send_header("Content-Length", str(len(body)))
    h.send_header("Content-Disposition", 'attachment; filename="%s"' % name)
    h.send_header("Cache-Control", "no-store")
    h.end_headers()
    h.wfile.write(body)


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
        elif path == "/api/studio/timesaved":
            h._send(200, timesaved.summary(studio.proj))
        elif path == "/api/studio/timesaved_html":
            import report
            sm = timesaved.summary(studio.proj)
            h._send(200, dict(html=report.time_saved_html(studio.proj, sm), summary=sm))
        elif path == "/api/studio/timesaved.csv":
            what = (q.get("what") or ["frames"])[0]
            slug = projmod.slug(studio.proj.name)
            if what == "summary":
                _download(h, timing.summary_csv(timesaved.summary(studio.proj)), "%s-time-saved-summary.csv" % slug)
            else:
                _download(h, timing.frames_csv(timesaved.project_frames(studio.proj)), "%s-time-saved-frames.csv" % slug)
        else:
            return False
        return True

    def post(h, path, b):
        if path == "/api/studio/open":
            try:
                studio.open(b.get("rec"), b.get("model", "none"))
            except RecordingMissing as e:
                h._send(409, dict(error=str(e), missing=dict(id=e.rec["id"], name=e.rec.get("name"),
                                                             camera=e.rec.get("camera"), path=e.rec["path"])))
                return True
            h._send(200, dict(ok=True))
        elif path == "/api/studio/ignore_spot":
            reopened = studio.ignore_spot(b.get("camera"), b["x"], b["y"], b["r"])
            h._send(200, dict(ok=True, reopened=reopened))
        elif path == "/api/studio/referee":
            h._send(200, studio.referee(b.get("frame", 0), b["x"], b["y"], b.get("cls")))
        elif path == "/api/studio/share/export":
            studio.export(b.get("path"), b.get("with_video"))
            h._send(200, dict(ok=True, share=studio.share_view()))
        elif path == "/api/studio/share/import":
            studio.import_file(b.get("file"), b.get("dest"), b.get("recordings"))
            h._send(200, dict(ok=True, share=studio.share_view()))
        elif path == "/api/studio/share/clear":
            if studio.share and studio.share.get("status") != "running":
                studio.share = None
            h._send(200, dict(ok=True))
        elif path == "/api/studio/switch":
            opened = studio.switch_project(b.get("folder") or "")
            h._send(200, dict(ok=True, opened=opened))
        elif path == "/api/studio/relink":
            res = studio.relink(b.get("rec"), b.get("path"), b.get("folder"))
            h._send(200, res)
        elif path == "/api/studio/train":
            mid = studio.trainer.start(float(b.get("minutes") or 20))
            studio.proj.data["settings"]["train_minutes"] = float(b.get("minutes") or 20)
            studio.proj.save()
            h._send(200, dict(model=mid))
        elif path == "/api/studio/classes":
            studio.proj.set_classes(b.get("classes") or [])
            studio.reopen()
            h._send(200, dict(ok=True))
        elif path == "/api/studio/setup":
            before = studio.proj.class_names()
            studio.proj.finish_setup(b.get("name"), b.get("classes"), b.get("seconds"))
            if studio.proj.class_names() != before:
                studio.reopen()
            h._send(200, dict(ok=True))
        elif path == "/api/studio/whats_new":
            changed = studio.proj.finish_whats_new(bool(b.get("set_varies")))
            h._send(200, dict(ok=True, changed=changed))
        elif path == "/api/studio/workarea":
            key = b.get("camera") or "default"
            if b.get("skip"):
                c = studio.proj.camera_settings(key)
                studio.proj.set_camera_settings(key, c["work_area"], c["margin_px"], c["ignore_spots"],
                                                c["image_size"], asked=True)
                h._send(200, dict(ok=True, reopened=False))
                return True
            studio.proj.set_camera_settings(key, b.get("work_area"), b.get("margin_px"), b.get("ignore_spots"),
                                            b.get("image_size"), asked=True)
            reopened = studio.reopen(only_camera=key)
            h._send(200, dict(ok=True, reopened=reopened))
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

    if not os.environ.get("MXIRL_BANNER_SHOWN"):
        print("\n%s\n%s\n" % (labelapp.BRAND, labelapp.TAGLINE))
    proj = projmod.Project(a.project, create_name=a.name or os.path.basename(os.path.abspath(a.project)))
    if a.relink:
        print("Found %d moved recordings." % proj.relink(a.relink))
    for folder in a.add or []:
        ids = proj.add_recording(folder, a.camera)
        print("Added %d recording(s) from %s" % (len(ids), folder))
    if os.environ.get("MXIRL_INSTALL_SECONDS"):
        try:
            timesaved.record_install(proj, float(os.environ["MXIRL_INSTALL_SECONDS"]))
        except (OSError, ValueError):
            pass
    studio = Studio(proj)
    cur = proj.data.get("current", {})
    if cur.get("rec"):
        model = cur.get("model", "none")
        try:
            studio.open(cur["rec"], model)
        except RecordingMissing as e:
            print("%s. The page will ask where it is now." % e)
        except (ValueError, KeyError) as e:
            print("Opening %s with no model (%s)" % (cur["rec"], e))
            try:
                studio.open(cur["rec"], "none")
            except (ValueError, KeyError) as e2:
                print("Could not open it: %s" % e2)
    get, post = make_routes(studio)
    server, url = labelapp.serve(studio.current, a.port, extra_get=get, extra_post=post)
    print("\n%s is running. Project: %s  (%s)" % (labelapp.BRAND, proj.name, proj.folder))
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
