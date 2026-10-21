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
- Learning curve: how the guesses improve as confirmed frames grow, and the list of models
  (failed ones hidden unless asked for; they can be deleted).
- Object types: name, has a front (heading), size, height above the floor (or "it varies").
- Add recording: a recording folder, a two-camera session folder, or an ordinary video file
  (mp4, mov, avi ... from a phone or webcam app: made into a recording folder, videofile.py).
- Work area: per camera, the part of the picture to look in. Ignore spots made by older versions
  keep working and are shown there, where they can be removed; new ones are not made (a rejection
  applies to one frame and teaches the tracker instead: "Reject here").
- Investigate (key Q on the page): one spot across the whole recording, in three parts (what it
  found / look around / change things); "Reject here" rejects the guess on that frame and the tracker
  remembers the empty spot; the second camera as a referee when both cameras are calibrated.
- Time saved: seconds per frame and frames accepted without looking, by model version (CSV too), and the
  AI helper's suggestions taken and turned down.
- Robot logs / outside data (More): a log folder or positions file for a recording, read by a plug-in
  (outsidelink.py, outside.py); its guesses join the others (source "log"), name the objects (roll call)
  and help the tracker decide which lookalike is which. Needs the camera's floor calibration.
- Home positions (in Work area): spots where an object's place and facing are known exactly (presets
  place one from two clicks); a visit is a free confirmation.
- AI helper (More): optional, off by default; settings, cost estimate, and its suggestions (ai_helper.py).
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
import outsidelink  # noqa: E402


class AIAskError(ValueError):
    """The AI helper could not answer: plain words for the person, and what kind of problem (ai_helper.AIError)."""

    def __init__(self, message, kind):
        ValueError.__init__(self, message)
        self.kind = kind


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
        stuck = trainmod.mark_interrupted(proj)       # left "training" by a studio that was closed mid-training
        if stuck:
            log("Model %s was still marked as training when the studio last closed: marked as stopped (interrupted)."
                % ", ".join(stuck))
        self.cross_note = None
        self.background = background     # run the other cameras' motion pass in the background
        self.bg = None                   # what the background pass is doing (shown in the top bar)
        self.bg_thread = None
        self.pending_reopen = False      # settings changed while opening: open again when done
        self.share = None                # Share project / Open shared project: what it is doing
        self.convert = None              # Add recording with a video file: making it into a recording
        self.outside_job = None          # outside data being read and lined up

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
                     "pick": "Picking frames to label",
                     "size": "Measuring how big the objects are (first time only)"}
            what = names.get(what, what)
            if clock["what"] != what:
                clock.update(what=what, clock=labelapp.RateClock())
            left = clock["clock"].update(done, total) if total else None
            self.loading = dict(what=what, done=done, total=total, left_s=left, left=labelapp.fmt_left(left))

        try:
            self._wait_background(rid)
            extra = self._cross_camera_guesses(rec)
            try:
                part_of = trainmod.part_function(proj, rec)
            except Exception as e:      # the recording still opens; "Most useful next" is just not spread
                self.log("Could not work out the training parts: %s" % e)
                part_of = None
            key = proj.camera_key(rec)
            try:
                outside = outsidelink.load_drafts(proj, rid)
            except Exception as e:          # the recording still opens, without the outside data
                self.log("The outside data's guesses could not be read: %s" % e)
                outside = None
            import homes as homesmod
            s = labelapp.Session(rec["path"], model=INFER if weights else None, weights=weights,
                                 classes=proj.class_names(), labels_dir=proj.labels_dir(rid), extra_dets=extra,
                                 progress=progress, log=self.log, class_map=proj.motion_class_map(),
                                 loose_classes=proj.loose_classes(), front_classes=proj.heading_classes(),
                                 area=proj.camera_settings(key),
                                 stats_path=timesaved.stats_path(proj, rid), model_id=model, part_of=part_of,
                                 type_settings=proj.type_settings(), outside=outside,
                                 refine=outsidelink.home_refiner(homesmod.homes_of(proj.data, key), key))
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
        # t_wall: the Pi's clock, shared by every camera on it (t_mono is shared too within one
        # two-camera session, but not between separately started recordings)
        mine_times = os.path.join(rec["path"], "frames.csv")
        if not os.path.exists(mine_times):
            self.cross_note = "No help from the other camera: this recording has no frame times (frames.csv)."
            return None
        try:
            cam_b = cameras.CameraModel.load(mine)
            times_b = crosscam.load_frame_times(mine_times)
        except Exception as e:          # the recording still opens, without the other camera's help
            self.cross_note = "No help from the other camera: its setup could not be read (%s)." % e
            self.log("Cross-camera: " + self.cross_note)
            return None
        b_meta = _meta(rec["path"])
        out, n, skipped = {}, 0, []
        for other in proj.recordings():
            if other["id"] == rec["id"] or other.get("session") != rec["session"]:
                continue
            path_a = proj.camera_model_path(other.get("camera"))
            if not path_a or not os.path.exists(path_a):
                continue
            objs = _confirmed_objects(proj.labels_dir(other["id"]))
            if not objs:
                continue
            other_times = os.path.join(other["path"], "frames.csv")
            if not os.path.exists(other_times):
                # the other camera's recording is not on this computer (not copied, or not located yet)
                skipped.append(other.get("camera") or other.get("name") or other["id"])
                continue
            try:
                cam_a = cameras.CameraModel.load(path_a)
                g = crosscam.cross_guesses(objs, cam_a, cam_b, crosscam.load_frame_times(other_times), times_b,
                                           (b_meta.get("width", 640), b_meta.get("height", 480)), proj.class_heights(),
                                           skip_classes=tuple(proj.varies_classes() + proj.no_height_classes()))
            except Exception as e:
                skipped.append(other.get("camera") or other.get("name") or other["id"])
                self.log("Cross-camera: %s skipped (%s)" % (other["id"], e))
                continue
            for f, ds in g.items():
                out.setdefault(f, []).extend(ds)
                n += len(ds)
        notes = []
        if n:
            notes.append("%d guesses from the other camera" % n)
        if skipped:
            notes.append("no help from the %s camera: its recording was not found on this computer"
                         % ", ".join(sorted(set(skipped))))
        if notes:
            self.cross_note = "; ".join(notes)
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
            recs.append(dict(id=r["id"], name=r["name"], camera=r.get("camera", ""), labelled=n, path=r["path"],
                             missing=not os.path.exists(os.path.join(r["path"], "video.avi"))))
        models = [dict(id=m["id"], status=m["status"], n_train_frames=m.get("n_train_frames", 0),
                       metrics=m.get("metrics", {}), created=m.get("created"), error=m.get("error"),
                       warning=m.get("warning"), interrupted=bool(m.get("interrupted")),
                       failed=projmod.Project.model_failed(m),
                       compatible=m.get("classes") == proj.class_names()) for m in proj.models()]
        models = [m for m in models if m["status"] != "ready" or m["compatible"]] + \
                 [m for m in models if m["status"] == "ready" and not m["compatible"]]
        job = self.trainer.job
        cur = proj.data.get("current", {})
        camera = None
        if cur.get("rec"):
            try:
                import homes as homesmod
                rec = proj.rec(cur["rec"])
                key = proj.camera_key(rec)
                ses = self.session
                camera = dict(key=key, name=rec.get("camera") or "", settings=proj.camera_settings(key),
                              homes=homesmod.homes_of(proj.data, key),
                              visits=((ses.refined or {}).get("visits") if ses is not None and not self.loading else None),
                              floor=bool(outsidelink.floor_camera(proj, rec)[0] is not None))
            except KeyError:
                pass
        return dict(brand=dict(name=labelapp.BRAND, tagline=labelapp.TAGLINE),
                    project=dict(name=proj.name, folder=proj.folder), recordings=recs, models=models,
                    current=cur, classes=proj.class_view(), job=job,
                    loading=self.loading_view(), settings=proj.data.get("settings", {}), labelled_total=total,
                    cross_note=self.cross_note, setup=proj.setup_view(), camera=camera, background=self.bg,
                    share=self.share_view(), missing=[r for r in recs if r["missing"]],
                    documents=_documents(), n_cameras=proj.n_cameras(), convert=self.convert_view(),
                    ai=self.ai_status(), outside=self.outside_view(), motion_usual=self.motion_usual(),
                    home_presets=outsidelink.HOME_PRESETS)

    # --- per-type motion: what "usual" means for each type ------------------------------------
    def motion_usual(self):
        import tracker
        p = tracker.TrackerParams()
        p.loose_classes = tuple(self.proj.loose_classes())
        return {n: p.type_defaults(n) for n in self.proj.class_names()}

    # --- AI helper ------------------------------------------------------------------------------
    def ai(self):
        import ai_helper
        return ai_helper.AIHelper(self.proj.folder)

    def ai_status(self):
        try:
            return self.ai().status()
        except Exception as e:          # never stops the page
            return dict(on=False, indicator="AI helper: settings could not be read (%s)" % e, calls=0)

    def ai_view(self):
        import ai_helper
        h = self.ai()
        st = h.status()
        log = h.read_log()
        calls = [e for e in log if not e.get("error")]
        return dict(status=st, settings={k: v for k, v in h.settings().items()},
                    providers=[dict(id=p, title=ai_helper.PROVIDER_TITLES[p]) for p in ai_helper.PROVIDERS],
                    keys={p: bool(h.keys.get(p)) for p in ai_helper.PROVIDERS},
                    counts=ai_counts(log), defaults=dict(anthropic=ai_helper.DEFAULT_ANTHROPIC_MODEL,
                                                         openai_compatible=ai_helper.DEFAULT_OPENAI_MODEL),
                    n_calls=len(calls), user_folder=ai_helper.user_dir())

    def ai_settings(self, b):
        """Change the AI helper's settings (and key). Returns (status before, status after)."""
        import ai_helper
        h = self.ai()
        before = h.settings()
        ch = {k: b[k] for k in ("enabled", "provider", "model", "base_url") if k in b}
        if "enabled" in ch:
            ch["enabled"] = bool(ch["enabled"])
        for k in ("model", "base_url"):
            if k in ch:
                ch[k] = str(ch[k] or "").strip()
        try:
            h.configure(**ch)
            if b.get("clear_key"):
                h.keys.clear(h.settings()["provider"])
            elif (b.get("key") or "").strip():
                h.set_key(b["key"].strip())
        except ai_helper.AIError as e:
            raise ValueError(str(e))
        return dict(before={k: before.get(k) for k in ("enabled", "provider", "model", "base_url")}, status=h.status(),
                    key_changed=bool(b.get("clear_key") or (b.get("key") or "").strip()))

    def _ai_inputs(self, task, b):
        """The pictures and words for a task, from what is open (never more than the task needs)."""
        import numpy as np
        s = self.session
        known = self.proj.class_names()
        if task == "setup_from_sentence":
            return dict(sentence=b.get("sentence") or "")
        if s is None:
            raise ValueError("Open a recording first.")
        half = max(16, int(round(1.2 * float(s.robot_px))))
        if task == "object_or_background":
            j = s.inv
            if not j or j.get("status") != "done" or not j.get("result"):
                raise ValueError("Investigate a spot first (Q): the AI is shown the spot now and how it usually looks.")
            res = j["result"]
            now = _decode_data_url(res.get("now_picture"))
            usual = _decode_data_url(res.get("background"))
            if now is None or usual is None:
                raise ValueError("This investigation has no usual picture of the spot to compare with.")
            strip = [_decode_data_url(t["picture"]) for t in (res.get("thumbs") or []) if t.get("changed")][:4]
            return dict(now=now, usual=usual, strip=[c for c in strip if c is not None], known_types=known)
        if task == "name_object":
            img = _frame_picture(s, b["frame"])
            return dict(crop=_crop(img, float(b["x"]), float(b["y"]), half), known_types=known)
        if task == "second_check":
            n = max(1, min(48, int(b.get("n") or 8)))
            pool = []
            for f, st in s.frames.items():
                if st["state"] != "reviewed":
                    continue
                for o in st["objects"]:
                    if o["status"] in ("confirmed", "corrected"):
                        pool.append((f, o))
            if not pool:
                raise ValueError("No frame in this recording is confirmed yet: confirm some frames first.")
            rng = np.random.default_rng(int(b.get("seed") or 0) or None)
            pick = [pool[i] for i in sorted(rng.choice(len(pool), size=min(n, len(pool)), replace=False))]
            items, cache = [], {}
            for f, o in pick:
                if f not in cache:
                    cache[f] = _frame_picture(s, f)
                items.append(dict(id="%d:%d" % (f, o["obj"]), label=o["cls"], crop=_crop(cache[f], o["x"], o["y"], half),
                                  frame=f, obj=o["obj"]))
            return dict(items=items, known_types=known)
        if task == "draft_labels":
            t = (b.get("type") or "").strip()
            if t not in known:
                raise ValueError("Pick one of this project's object types first.")
            return dict(frame=_frame_picture(s, b["frame"]), type_name=t, description=b.get("description") or "")
        raise ValueError("Unknown AI task %s." % task)

    def ai_estimate(self, task, b):
        import ai_helper
        try:
            kw = self._ai_inputs(task, b)
            if task == "second_check":
                kw = dict(items=[{k: v for k, v in it.items() if k in ("id", "label", "crop")} for it in kw["items"]],
                          known_types=kw["known_types"])
            return self.ai().estimate_cost(task, **kw)
        except ai_helper.AIError as e:
            raise ValueError(str(e))

    def ai_ask(self, task, b):
        """Ask the AI helper; every answer is a suggestion marked AI (nothing changes until the person takes it)."""
        import ai_helper
        h = self.ai()
        kw = self._ai_inputs(task, b)
        try:
            if task == "second_check":
                where = {it["id"]: (it["frame"], it["obj"], it["label"]) for it in kw["items"]}
                items = [{k: v for k, v in it.items() if k in ("id", "label", "crop")} for it in kw["items"]]
                res = h.second_check(items, known_types=kw["known_types"])
                for sp in res["suspects"]:
                    f, o, lab = where.get(sp["id"], (None, None, None))
                    sp.update(frame=f, obj=o, label=lab)
                res["sample"] = [dict(id=i, frame=v[0], obj=v[1], label=v[2]) for i, v in where.items()]
                return res
            return getattr(h, task)(**kw)
        except ai_helper.AIError as e:
            raise AIAskError(str(e), e.kind)

    # --- outside data (robot logs, a positions file) --------------------------------------------
    def outside_view(self):
        proj = self.proj
        cur = proj.data.get("current", {})
        out = dict(job=dict(self.outside_job) if self.outside_job else None)
        if not cur.get("rec"):
            return out
        try:
            rec = proj.rec(cur["rec"])
        except KeyError:
            return out
        out.update(rec=rec["id"], settings=outsidelink.read_settings(proj, rec["id"]),
                   info=outsidelink.drafts_info(proj, rec["id"]))
        return out

    def outside_details(self):
        """For the Robot logs / outside data window."""
        proj = self.proj
        v = self.outside_view()
        v["plugins"] = outsidelink.plugin_list()
        if v.get("info"):
            v["info"]["plugin_title"] = next((p["title"] for p in v["plugins"] if p["id"] == v["info"].get("plugin")),
                                             v["info"].get("plugin"))
        if v.get("rec"):
            rec = proj.rec(v["rec"])
            cam, path = outsidelink.floor_camera(proj, rec)
            v["floor"] = dict(path=path, ok=cam is not None, text=None if cam is not None else outsidelink.no_floor_text(proj, rec),
                              looked=outsidelink.floor_paths(proj, rec))
            v["has_clock"] = os.path.exists(os.path.join(rec["path"], "frames.csv"))
        return v

    def outside_set(self, path, plugin=None, offset_s=0.0):
        proj = self.proj
        cur = proj.data.get("current", {})
        if not cur.get("rec"):
            raise ValueError("Open a recording first: outside data belongs to one recording.")
        rec = proj.rec(cur["rec"])
        path = (path or "").strip().strip('"')
        if not path:
            raise ValueError("Type or paste the log folder or file first (the box is empty).")
        if outsidelink.floor_camera(proj, rec)[0] is None:
            raise ValueError(outsidelink.no_floor_text(proj, rec))
        data = outsidelink.read_data(proj, rec, path, plugin, offset_s)      # a plain message now if it can't be read
        if not data.streams:
            raise ValueError("Nothing in %s could be placed (%s). Nothing was changed." % (path, data.summary()))
        before = outsidelink.read_settings(proj, rec["id"])
        outsidelink.save_settings(proj, rec["id"], path, plugin or data.plugin, offset_s)
        self.outside_build()
        return dict(before=before, plugin=data.plugin, names=data.names(), notes=data.notes, summary=data.summary())

    def outside_build(self):
        """Line the outside data up again (with the labels made since) and open the recording with its guesses."""
        proj = self.proj
        cur = proj.data.get("current", {})
        s = self.session
        if s is None or self.loading:
            raise ValueError("The recording is still opening; wait a moment.")
        if self.outside_job and self.outside_job.get("status") == "running":
            raise ValueError("Still lining up the outside data; wait a moment.")
        rec = proj.rec(cur["rec"])
        job = dict(status="running", what="Reading the outside data ...", result=None, error=None)
        self.outside_job = job

        def say(t):
            job["what"] = t
            self.log("Outside data: " + t)

        def work():
            try:
                res = outsidelink.build(proj, rec, s, say)
                job["what"] = "Opening the recording with the new guesses ..."
                self.reopen()                  # starts opening (loading is set) before the page hears "done"
                job.update(result=res, status="done", what="Done")
            except Exception as e:
                if not isinstance(e, ValueError):
                    traceback.print_exc()
                job.update(status="failed", error=str(e))
        threading.Thread(target=work, daemon=True).start()
        return job

    def outside_clear(self, restore=False):
        proj = self.proj
        cur = proj.data.get("current", {})
        if not cur.get("rec"):
            raise ValueError("Open a recording first.")
        moved = (outsidelink.restore_settings if restore else outsidelink.clear_settings)(proj, cur["rec"])
        self.outside_job = None
        if moved:
            self.reopen()
        return moved

    # --- home positions ---------------------------------------------------------------------------
    def set_homes(self, key, homes):
        import homes as homesmod
        with self.proj.lock:
            before = homesmod.homes_of(self.proj.data, key)
            stored = homesmod.set_homes(self.proj.data, key, homes or [])
            self.proj.save()
        reopened = self.reopen(only_camera=key)
        return dict(homes=stored, before=before, reopened=reopened)

    def dock_preset(self, key, name, wall, mouth, owner=None, version=2, stations=True, preset=None):
        """A preset's home positions from two clicks (not saved: the page adds them to its list); the
        presets are listed in outsidelink.HOME_PRESETS."""
        import homes as homesmod
        pr = outsidelink.home_preset(preset or outsidelink.HOME_PRESETS[0]["id"])
        proj = self.proj
        cur = proj.data.get("current", {})
        rec = proj.rec(cur["rec"]) if cur.get("rec") else None
        if rec is None or proj.camera_key(rec) != key:
            raise ValueError("Open a recording of this camera first.")
        cam, _ = outsidelink.floor_camera(proj, rec)
        if cam is None:
            raise ValueError("The %s preset needs this camera's floor calibration (to know how high it stands in the "
                             "picture). %s" % (pr["title"], outsidelink.no_floor_text(proj, rec).split(". ", 1)[1]))
        name = (name or "").strip() or pr["name"]
        return outsidelink.preset_homes(pr["id"], name, key, cam, [float(v) for v in wall], [float(v) for v in mouth],
                                        owner=(owner or "").strip() or None, version=int(version), stations=bool(stations))

    # --- investigate: second camera ----------------------------------------------------------------
    # (Investigate's old "Ignore this spot", which left a spot out of every frame, is gone: a rejection
    # now applies to one frame and teaches the tracker, app.Session.reject_here.)
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

    def export(self, path, with_video=False, include_failed=False):
        import share
        path = (path or "").strip().strip('"')
        if not path:
            raise ValueError("Say where to save the file.")

        def run(progress):
            res = share.export_project(self.proj, os.path.expanduser(path), with_video=bool(with_video), progress=progress,
                                       lock_for=self._save_lock_for, include_failed=bool(include_failed))
            m = res["manifest"]
            return dict(path=res["path"], mb=round(res["bytes"] / 1e6, 1), recordings=len(m["recordings"]),
                        models=m["contents"]["models"], with_video=m["with_video"], warnings=m["warnings"],
                        left_out=m.get("left_out_models", []))
        return self._share_job("Saving the project as one file", run)

    def _save_lock_for(self, rid):
        """The save lock of a recording open for labelling (its labels.csv and review.json are
        written together under it), or None."""
        s = self.session
        cur = self.proj.data.get("current", {})
        if s is not None and cur.get("rec") == rid and \
                os.path.normcase(s.labels_dir) == os.path.normcase(self.proj.labels_dir(rid)):
            return s.lock
        return None

    def import_file(self, file, dest, recordings=None):
        import share
        file = os.path.expanduser((file or "").strip().strip('"'))
        dest = os.path.expanduser((dest or "").strip().strip('"'))
        recordings = os.path.expanduser((recordings or "").strip().strip('"')) or None
        if not file:
            raise ValueError("Type or paste the project file first (the box is empty).")
        if not os.path.isfile(file):
            raise ValueError("There is no file at %s." % file)
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
            trainmod.mark_interrupted(proj)
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
        if folder is not None and not str(folder).strip().strip('"'):
            raise ValueError("Type or paste the folder to search first (the box is empty).")
        if rid and path is not None and not str(path).strip().strip('"'):
            raise ValueError("Type or paste where the recording is now first (the box is empty).")
        if not folder and not (rid and path):
            raise ValueError("Type or paste where the recording is now first (the box is empty).")
        if folder:
            found, still = share.relink_from_folder(self.proj, os.path.expanduser(folder.strip().strip('"')))
            return dict(found=found, missing=still, notes=share.describe_missing(still))
        rec = share.relink(self.proj, rid, path)
        return dict(found=[rec["id"]], missing=share.missing_recordings(self.proj),
                    notes=share.describe_missing(share.missing_recordings(self.proj)))

    def curve(self):
        """models/curve.json for the page: points in model order (no model first), each marked
        failed (finds nothing yet) or not, and a plain caution when the scores rest on little."""
        p = os.path.join(self.proj.folder, "models", "curve.json")
        if not os.path.exists(p):
            c = dict(points=[], test=dict(frames=0, note="no model trained yet"))
        else:
            with open(p) as f:
                c = json.load(f)
        failed = {m["id"] for m in self.proj.models() if projmod.Project.model_failed(m)}
        have = {m["id"] for m in self.proj.models()}
        c["points"] = sorted([dict(pt, failed=pt.get("model") in failed) for pt in c.get("points", [])
                              if pt.get("model") == "none" or pt.get("model") in have],
                             key=lambda pt: trainmod.version_order(pt.get("model")))
        c["caution"] = trainmod.score_caution(c)
        t = c.get("test") or {}
        bs = t.get("block_s")
        if not bs:                                         # older files: the length is only in the note
            import re
            m = re.search(r"of (\d+(?:\.\d+)?) s", t.get("note") or "")
            bs = float(m.group(1)) if m else None
        if t.get("blocks") and bs:                         # the note in plain words (older files said "1 time blocks")
            t["note"] = "%d time block%s of %.0f s held back; never trained on" % (
                t["blocks"], "" if t["blocks"] == 1 else "s", bs)
        c["models"] = [dict(id=m["id"], status=m["status"], failed=projmod.Project.model_failed(m),
                            n_train_frames=m.get("n_train_frames", 0), created=m.get("created"),
                            error=m.get("error"), warning=m.get("warning"), metrics=m.get("metrics", {}),
                            interrupted=bool(m.get("interrupted")),
                            current=self.proj.data.get("current", {}).get("model") == m["id"],
                            copy_paste=m.get("copy_paste"), hard_negatives=m.get("hard_negatives"))
                       for m in self.proj.models()]
        # by situation (the best model, beside no model) and what training used, as in the full report
        try:
            import report
            best = None
            for pt in c["points"]:
                if pt.get("model") != "none" and pt.get("f1") is not None and not pt.get("failed") and \
                        (best is None or pt["f1"] > best["f1"]):
                    best = pt
            c["situations_html"] = report.situations_html(self.proj, best) if best else ""
        except Exception as e:            # the rest of the window still shows
            self.log("By situation could not be shown: %s" % e)
            c["situations_html"] = ""
        c["copy_paste"] = trainmod.paste_mode(self.proj)
        return c

    def delete_model(self, mid):
        """Delete a failed or empty model (the page asks first). If it is the one giving the
        guesses, the recording is opened again with no model."""
        if self.trainer.busy() and self.trainer.job.get("model") == mid:
            raise ValueError("model %s is still training" % mid)
        was_current = self.proj.data.get("current", {}).get("model") == mid
        self.proj.delete_model(mid)
        if was_current:
            self.reopen()
        return was_current

    # --- a video file made into a recording ----------------------------------------------------
    def convert_view(self):
        return {k: v for k, v in self.convert.items() if k != "stop"} if self.convert else None

    def add(self, path, camera=None):
        """Add recording: a recording or session folder (added at once), or a video file (made
        into a recording folder in the background; see convert_view). Returns dict(added, ids) or
        dict(converting=True)."""
        import videofile
        path = (path or "").strip().strip('"').strip()
        if not path:
            raise ValueError("Type or paste the recording folder or video file first (the box is empty).")
        full = os.path.abspath(os.path.expanduser(path))
        if os.path.isfile(full):
            if not videofile.is_video_file(full):
                raise ValueError("%s is not a video file this studio knows (%s), nor a recording folder."
                                 % (os.path.basename(full), ", ".join(e[1:] for e in videofile.VIDEO_EXTS)))
            self._convert(full, camera)
            return dict(converting=True)
        added = self.proj.add_recording(full, camera)
        if added and self.session is None and not self.loading:
            self.open(added[0], "none")
        return dict(added=len(added), ids=added)

    def _convert(self, video, camera=None):
        import videofile
        if self.convert and self.convert.get("status") == "running":
            raise ValueError("still making the last video file into a recording; wait a moment")
        clock = labelapp.RateClock(window=10.0)
        job = dict(status="running", what="Making a recording from %s" % os.path.basename(video), done=0, total=0,
                   left="", result=None, error=None, stop=False)
        self.convert = job

        def progress(done, total):
            job.update(done=done, total=total, left=labelapp.fmt_left(clock.update(done, total)))

        def work():
            try:
                res = videofile.convert(video, os.path.join(self.proj.folder, "recordings"), progress=progress,
                                        stop=lambda: job["stop"])
                ids = self.proj.add_recording(res["folder"], camera)
                res.update(ids=ids, added=len(ids))
                if ids and self.session is None and not self.loading:
                    self.open(ids[0], "none")
                job.update(result=res, status="done")
            except InterruptedError:
                job.update(status="stopped")
            except Exception as e:
                if not isinstance(e, ValueError):
                    traceback.print_exc()
                job.update(status="failed", error=str(e))
        threading.Thread(target=work, daemon=True).start()
        return job


_NO_BEFORE = object()


# --- AI helper: what it was asked, and the person's answers ---------------------------------------------
AI_TASK_WORDS = dict(object_or_background="object or background?", name_object="name an object",
                     second_check="second check of labels", draft_labels="draft labels for a type",
                     setup_from_sentence="set up from a sentence")


def ai_counts(log):
    """Suggestions per task: asked, taken (accepted), turned down, not answered, and the cost so far."""
    out, cost = {}, 0.0
    for e in log:
        if e.get("error"):
            continue
        t = out.setdefault(e.get("task", "?"), dict(task=e.get("task"), words=AI_TASK_WORDS.get(e.get("task"), e.get("task")),
                                                     asked=0, taken=0, turned_down=0, open=0))
        t["asked"] += 1
        a = e.get("accepted")
        t["taken" if a is True else "turned_down" if a is False else "open"] += 1
        cost += float(e.get("cost_usd") or 0.0)
    return dict(tasks=list(out.values()), cost_usd=round(cost, 4))


def ai_time_saved_html(counts):
    import html as _html
    rows = counts.get("tasks") or []
    if not rows:
        return ("<h2>AI helper</h2><p class='muted'>The AI helper has not been asked anything in this project (it is off "
                "unless you switch it on under More, AI helper).</p>")
    out = ["<h2>AI helper</h2><p class='muted'>Every suggestion it made, and what you did with it. A suggestion only "
           "changes a label when you take it, and then it is your label (Z takes it back).</p>",
           "<table><tr><th>Asked to</th><th>Times</th><th>Taken</th><th>Turned down</th><th>Not answered</th></tr>"]
    for r in rows:
        out.append("<tr><td>%s</td><td>%d</td><td>%d</td><td>%d</td><td>%d</td></tr>"
                   % (_html.escape(r["words"] or ""), r["asked"], r["taken"], r["turned_down"], r["open"]))
    out.append("</table><p class='muted'>Cost so far, as the service's prices were guessed: about $%.3f.</p>"
               % counts.get("cost_usd", 0.0))
    return "".join(out)


def _decode_data_url(u):
    """A picture sent to the page as data:image/jpeg;base64,... back into an image (None if empty)."""
    import base64
    import numpy as np
    import cv2
    if not u or "," not in u:
        return None
    b = base64.b64decode(u.split(",", 1)[1])
    return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)


def _frame_picture(session, f):
    """Frame f's picture: the saved picked frame, else read from the video."""
    import numpy as np
    import cv2
    p = os.path.join(session.frames_dir, "%06d.jpg" % int(f))
    if os.path.exists(p):
        img = cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            return img
    cap = cv2.VideoCapture(session.rec.video)
    try:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(f))
        ok, img = cap.read()
    finally:
        cap.release()
    if not ok:
        raise ValueError("frame %d could not be read from the video" % int(f))
    return img


def _crop(img, x, y, half):
    h, w = img.shape[:2]
    x0, x1 = max(0, int(round(x - half))), min(w, int(round(x + half)) + 1)
    y0, y1 = max(0, int(round(y - half))), min(h, int(round(y + half)) + 1)
    return img[y0:y1, x0:x1].copy()


def _unknown_note(before, proj):
    """Plain words for when the guesses with no model show objects as "unknown": the motion finder
    can only tell "an object" from "something reaching in", so with two or more plain types it can't
    say which type an object is. before: proj.motion_object_type() before the change (_NO_BEFORE:
    just say it when it applies)."""
    after = proj.motion_object_type()
    plain = [c["name"] for c in proj.classes if c["name"] != projmod.CATCH_ALL and not c.get("height_varies")]
    if after != projmod.CATCH_ALL or len(plain) < 2:
        return None
    if before is not _NO_BEFORE and before == after:
        return None
    return ("With %d object types (%s), the guesses before any model is trained can't tell which type an object "
            "is, so they show it as \"%s\". Give each object its type with the number keys (1-9); once a model "
            "is trained on your labels, its guesses use your types." % (len(plain), ", ".join(plain), projmod.CATCH_ALL))


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
            try:
                counts = ai_counts(studio.ai().read_log())
            except Exception:
                counts = dict(tasks=[], cost_usd=0.0)
            h._send(200, dict(html=report.time_saved_html(studio.proj, sm) + ai_time_saved_html(counts), summary=sm,
                              ai=counts))
        elif path == "/api/studio/ai":
            h._send(200, studio.ai_view())
        elif path == "/api/studio/outside":
            h._send(200, studio.outside_details())
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
        elif path == "/api/studio/referee":
            h._send(200, studio.referee(b.get("frame", 0), b["x"], b["y"], b.get("cls")))
        elif path == "/api/studio/share/export":
            studio.export(b.get("path"), b.get("with_video"), b.get("include_failed"))
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
            cp = b.get("copy_paste")
            if cp is not None and cp not in ("off", "compare", "on"):
                raise ValueError("copy-paste must be off, compare or on")
            with studio.proj.lock:
                before_cp = trainmod.paste_mode(studio.proj)
                if cp is not None:
                    studio.proj.data.setdefault("settings", {})["copy_paste"] = cp
                studio.proj.save()
            mid = studio.trainer.start(float(b.get("minutes") or 20))
            with studio.proj.lock:
                studio.proj.data["settings"]["train_minutes"] = float(b.get("minutes") or 20)
                studio.proj.save()
            h._send(200, dict(model=mid, copy_paste=trainmod.paste_mode(studio.proj), copy_paste_before=before_cp))
        elif path == "/api/studio/classes":
            before = studio.proj.motion_object_type()
            studio.proj.set_classes(b.get("classes") or [])
            studio.reopen()
            h._send(200, dict(ok=True, note=_unknown_note(before, studio.proj)))
        elif path == "/api/studio/setup":
            before = studio.proj.class_names()
            studio.proj.finish_setup(b.get("name"), b.get("classes"), b.get("seconds"))
            if studio.proj.class_names() != before:
                studio.reopen()
            h._send(200, dict(ok=True, note=_unknown_note(_NO_BEFORE, studio.proj)))
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
            h._send(200, studio.add(b.get("path", ""), b.get("camera") or None))
        elif path == "/api/studio/convert/stop":
            if studio.convert and studio.convert.get("status") == "running":
                studio.convert["stop"] = True
            h._send(200, dict(ok=True))
        elif path == "/api/studio/convert/clear":
            if studio.convert and studio.convert.get("status") != "running":
                studio.convert = None
            h._send(200, dict(ok=True))
        elif path == "/api/studio/delete_model":
            reopened = studio.delete_model(str(b.get("model") or ""))
            h._send(200, dict(ok=True, reopened=reopened))
        elif path == "/api/studio/ai/settings":
            h._send(200, studio.ai_settings(b))
        elif path == "/api/studio/ai/estimate":
            h._send(200, studio.ai_estimate(str(b.get("task") or ""), b))
        elif path == "/api/studio/ai/ask":
            try:
                h._send(200, studio.ai_ask(str(b.get("task") or ""), b))
            except AIAskError as e:
                h._send(400, dict(error=str(e), kind=e.kind))
        elif path == "/api/studio/ai/decision":
            ids = b.get("call_ids") or ([b["call_id"]] if b.get("call_id") else [])
            for cid in ids:
                studio.ai().record_decision(str(cid), bool(b.get("accepted")), b.get("note") or "")
            h._send(200, dict(ok=True, recorded=len(ids)))
        elif path == "/api/studio/outside/set":
            h._send(200, studio.outside_set(b.get("path"), b.get("plugin") or None, b.get("offset_s") or 0.0))
        elif path == "/api/studio/outside/rebuild":
            studio.outside_build()
            h._send(200, dict(ok=True))
        elif path == "/api/studio/outside/clear":
            h._send(200, dict(moved=studio.outside_clear(bool(b.get("restore")))))
        elif path == "/api/studio/outside/done":
            if studio.outside_job and studio.outside_job.get("status") != "running":
                studio.outside_job = None
            h._send(200, dict(ok=True))
        elif path == "/api/studio/homes":
            h._send(200, studio.set_homes(b.get("camera") or "default", b.get("homes") or []))
        elif path == "/api/studio/homes/dock":
            h._send(200, dict(homes=studio.dock_preset(b.get("camera") or "default", b.get("name"), b["wall"], b["mouth"],
                                                       b.get("owner"), b.get("version") or 2, b.get("stations", True),
                                                       b.get("preset"))))
        elif path == "/api/studio/show_failed":
            with studio.proj.lock:
                studio.proj.data.setdefault("settings", {})["show_failed_models"] = bool(b.get("on"))
                studio.proj.save()
            h._send(200, dict(ok=True))
        else:
            return False
        return True

    return get, post


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", help="project folder (created if it doesn't exist)")
    ap.add_argument("--name", help="project name, when creating it")
    ap.add_argument("--add", nargs="+", metavar="FOLDER", help="add recording (or two-camera session) folders, or video "
                    "files, then open")
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
        if os.path.isfile(os.path.expanduser(folder)):
            import videofile
            print("Making a recording from the video file %s ..." % folder)
            try:
                res = videofile.convert(folder, os.path.join(proj.folder, "recordings"),
                                        progress=lambda d, t: print("  %d / %d frames" % (d, t), end="\r", flush=True))
            except ValueError as e:
                print("\n%s" % e)
                continue
            print("\n%s" % "\n".join(["Recording folder: " + res["folder"]] + res["notes"]))
            folder = res["folder"]
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
