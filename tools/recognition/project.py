"""A recognition project: everything one labelling-and-training effort needs, in one folder.

    <project>/
      project.json        object types, recordings, model versions, current choices
      labels/<rec id>/    labels.csv, review.json, frames/, cache/   (one per recording)
      models/<vN>/        model.onnx, run/ (best.pt, split.json, log), train.log, meta.json
      models/curve.json   every model (and "no model") scored on the same held-back frames
      cameras/<name>.json camera models for sharing labels between cameras (optional)

The recordings themselves stay where they are (they are large); project.json
holds their paths. Copying a project to another computer: copy the folder and
the recordings, then fix paths with `studio.py <project> --relink <new recordings folder>`.
"""
import csv
import datetime as dt
import json
import os
import re
import time

VERSION = 1

# Object type fields: name, has_front (the model learns a heading), size_px (None = measured),
# height_cm (where the labelled point sits above the floor: used only to pass labels to the
# second camera), height_varies (True: the height is not fixed, so the type is not passed
# between cameras), note (where to mark it, shown to the person). Older projects lack the
# last two; they read as False and "".
CATCH_ALL = "unknown"          # always kept: anything that is none of the other types
CATCH_ALL_TYPE = dict(name=CATCH_ALL, has_front=False, size_px=None, height_cm=0.0, height_varies=True,
                      note="anything else, or not sure what it is")

# A new project starts with only the catch-all type; the first-start setup asks for the rest.
DEFAULT_CLASSES = [dict(CATCH_ALL_TYPE)]

# Ready-made sets of object types offered by the first-start setup.
PRESETS = [
    dict(id="alvik", title="Alvik robots", classes=[
        dict(name="alvik", has_front=True, size_px=None, height_cm=4.0, height_varies=False,
             note="the robot: mark the middle of its top"),
        dict(name="person", has_front=False, size_px=None, height_cm=0.0, height_varies=True,
             note="hands, arms, sometimes feet: mark the end reaching in"),
        dict(CATCH_ALL_TYPE),
    ]),
]

# Names the motion finder (tools/label/motion.py) gives its guesses, mapped to a project's types
# by motion_class_map(): an object-sized blob, a big blob reaching in from the edge, other.
REACHING_NAMES = ("hand", "person")

# Bumped when the "what's new" screen for existing projects changes.
WHATS_NEW_VERSION = 1


def now_iso():
    return dt.datetime.now().isoformat(timespec="seconds")


def slug(s):
    s = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(s)).strip("-.")
    return s or "rec"


def write_json(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1)
        f.flush()
        os.fsync(f.fileno())
    for _ in range(10):
        try:
            os.replace(tmp, path)
            return
        except PermissionError:
            time.sleep(0.2)
    raise PermissionError("%s is open in another program; close it and try again" % path)


class Project:
    def __init__(self, folder, create_name=None):
        self.folder = os.path.abspath(folder)
        self.path = os.path.join(self.folder, "project.json")
        if os.path.exists(self.path):
            with open(self.path, encoding="utf-8") as f:
                self.data = json.load(f)
        elif create_name is not None:
            os.makedirs(self.folder, exist_ok=True)
            self.data = dict(version=VERSION, name=create_name or os.path.basename(self.folder), created=now_iso(),
                             classes=[dict(c) for c in DEFAULT_CLASSES], recordings=[], models=[], cameras={},
                             current=dict(rec=None, model="none"), settings=dict(train_minutes=20, block_s=30))
            self.save()
        else:
            raise FileNotFoundError("No project.json in %s" % self.folder)
        for d in ("labels", "models", "cameras"):
            os.makedirs(os.path.join(self.folder, d), exist_ok=True)

    # --- basics ---------------------------------------------------------------------------
    def save(self):
        self.data["updated"] = now_iso()
        write_json(self.path, self.data)

    @property
    def name(self):
        return self.data["name"]

    @property
    def classes(self):
        return self.data["classes"]

    def class_names(self):
        return [c["name"] for c in self.classes]

    def heading_classes(self):
        return [c["name"] for c in self.classes if c.get("has_front")]

    def class_heights(self):
        return {c["name"]: float(c.get("height_cm") or 0.0) for c in self.classes}

    def varies_classes(self):
        """Types whose height is not fixed: never passed between cameras."""
        return [c["name"] for c in self.classes if c.get("height_varies")]

    def motion_class_map(self):
        """The motion finder's guess names -> this project's types. Its object-sized blobs go to
        the one plain type (or "alvik" in projects that have it), big blobs reaching in from the
        edge to the hand/person type, everything else to the catch-all."""
        names = self.class_names()
        catch = CATCH_ALL if CATCH_ALL in names else None
        reaching = next((n for n in REACHING_NAMES if n in names), None)
        if reaching is None:
            reaching = next((c["name"] for c in self.classes if c.get("height_varies") and c["name"] != CATCH_ALL), None)
        plain = [n for n in names if n not in (CATCH_ALL, reaching)]
        obj = "alvik" if "alvik" in names else (plain[0] if len(plain) == 1 else catch)
        out = {"alvik": obj or catch, "hand": reaching or catch, "unknown": catch,
               "machine": "machine" if "machine" in names else catch}
        return {k: v for k, v in out.items() if v}

    def loose_classes(self):
        """Types the tracker lets move fast and end soon (hands, the catch-all, and anything
        whose height varies)."""
        return sorted(set(n for n in self.class_names() if n in REACHING_NAMES or n == CATCH_ALL)
                      | set(self.varies_classes()))

    # --- first start ---------------------------------------------------------------------------
    def has_labels(self):
        """True when any frame of any recording has been finished by a person or accepted."""
        for r in self.recordings():
            p = os.path.join(self.labels_dir(r["id"]), "review.json")
            if os.path.exists(p):
                try:
                    with open(p, encoding="utf-8") as f:
                        if json.load(f).get("frames"):
                            return True
                except ValueError:
                    continue
        return False

    def setup_view(self):
        """What the page shows before labelling: the setup screen (a project that was never set
        up and has no labels), or a one-time "what's new" (older projects that have labels)."""
        st = self.data.setdefault("settings", {})
        done = bool(st.get("setup_done"))
        labelled = False if done else self.has_labels()
        need_setup = not done and not labelled
        seen = int(st.get("whats_new_seen") or 0)
        whats_new = not need_setup and seen < WHATS_NEW_VERSION
        offer_varies = [c["name"] for c in self.classes
                        if c["name"] in ("hand", CATCH_ALL) and not c.get("height_varies")] if whats_new else []
        return dict(need_setup=need_setup, whats_new=whats_new, offer_varies=offer_varies,
                    presets=PRESETS, catch_all=CATCH_ALL_TYPE)

    def finish_setup(self, name=None, classes=None, seconds=None):
        """Save the first-start setup: project name and object types (the catch-all is added
        if missing)."""
        if name is not None:
            name = str(name).strip()
            if not name:
                raise ValueError("the project needs a name")
            self.data["name"] = name[:80]
        if classes is not None:
            self.set_classes(classes, save=False)
        st = self.data.setdefault("settings", {})
        st["setup_done"] = True
        st["whats_new_seen"] = WHATS_NEW_VERSION
        if seconds is not None:
            st["setup_seconds"] = round(float(seconds), 1)
        st["setup_finished"] = now_iso()
        self.save()

    def finish_whats_new(self, set_varies=False):
        """An older project confirmed the what's-new screen; with set_varies, hand and unknown
        become "height varies"."""
        changed = []
        if set_varies:
            for c in self.classes:
                if c["name"] in ("hand", CATCH_ALL) and not c.get("height_varies"):
                    c["height_varies"] = True
                    changed.append(c["name"])
        st = self.data.setdefault("settings", {})
        st["setup_done"] = True
        st["whats_new_seen"] = WHATS_NEW_VERSION
        self.save()
        return changed

    # --- work area per camera --------------------------------------------------------------------
    @staticmethod
    def camera_key(rec):
        """Work areas are kept per camera: by the recording's camera name, or "default" for
        recordings added without one."""
        return (rec.get("camera") or "").strip() or "default"

    def camera_settings(self, key):
        """project.json cameras[key]: work_area (corner points in image px, or None = whole
        picture), margin_px (None = one object size), ignore_spots ([x, y, r] in px), asked
        (the person has been offered to mark it)."""
        c = (self.data.get("cameras") or {}).get(key) or {}
        return dict(work_area=c.get("work_area"), margin_px=c.get("margin_px"),
                    ignore_spots=list(c.get("ignore_spots") or []), asked=bool(c.get("asked")),
                    image_size=c.get("image_size"))

    def set_camera_settings(self, key, work_area=None, margin_px=None, ignore_spots=None, image_size=None,
                            asked=True):
        poly = None
        if work_area:
            poly = [[round(float(x), 1), round(float(y), 1)] for x, y in work_area]
            if len(poly) < 3:
                raise ValueError("a work area needs at least 3 corners")
        spots = []
        for s in ignore_spots or []:
            x, y, r = (float(v) for v in s)
            if r <= 0:
                raise ValueError("an ignore spot needs a size")
            spots.append([round(x, 1), round(y, 1), round(r, 1)])
        m = None if margin_px in (None, "") else max(0.0, float(margin_px))
        cams = self.data.setdefault("cameras", {})
        cams[key] = dict(cams.get(key) or {}, work_area=poly, margin_px=m, ignore_spots=spots, asked=bool(asked),
                         image_size=list(image_size) if image_size else None, updated=now_iso())
        self.save()
        return self.camera_settings(key)

    # --- recordings -------------------------------------------------------------------------
    def recordings(self):
        return self.data["recordings"]

    def rec(self, rid):
        for r in self.recordings():
            if r["id"] == rid:
                return r
        raise KeyError("no recording %r in this project" % rid)

    def labels_dir(self, rid):
        return os.path.join(self.folder, "labels", rid)

    def add_recording(self, path, camera=None):
        """Add a recording folder (with video.avi), or a two-camera session folder
        (with session.json and one recording folder per camera). Returns the added ids."""
        path = os.path.abspath(os.path.expanduser(path.strip().strip('"')))
        if not os.path.isdir(path):
            raise ValueError("Not a folder: %s" % path)
        sess_file = os.path.join(path, "session.json")
        found = []
        if os.path.exists(sess_file):
            with open(sess_file, encoding="utf-8") as f:
                sess = json.load(f)
            cams = sess.get("cameras") or []
            for i, c in enumerate(cams):
                sub = c.get("folder") or c.get("dir") or c.get("name")
                folder = os.path.join(path, sub) if sub else None
                if not folder or not os.path.exists(os.path.join(folder, "video.avi")):
                    continue
                found.append((folder, c.get("role") or c.get("name") or "cam%d" % i, os.path.basename(path)))
            if not found:
                raise ValueError("session.json lists no camera folders with a video.avi in %s" % path)
        elif os.path.exists(os.path.join(path, "video.avi")):
            found.append((path, camera or "", None))
        else:
            subs = sorted(d for d in os.listdir(path) if os.path.exists(os.path.join(path, d, "video.avi")))
            if not subs:
                raise ValueError("No video.avi in %s (give the recording folder from the Pi)" % path)
            for d in subs:
                found.append((os.path.join(path, d), camera or "", None))
        have = {os.path.normcase(r["path"]) for r in self.recordings()}
        ids = {r["id"] for r in self.recordings()}
        added = []
        for folder, cam, session in found:
            if os.path.normcase(folder) in have:
                continue
            base = slug((session + "_" + cam) if session else os.path.basename(folder))
            rid, k = base, 2
            while rid in ids:
                rid, k = "%s-%d" % (base, k), k + 1
            ids.add(rid)
            self.data["recordings"].append(dict(id=rid, path=folder, name=os.path.basename(session or folder),
                                                camera=cam, session=session, added=now_iso()))
            added.append(rid)
        if added:
            if not self.data["current"].get("rec"):
                self.data["current"]["rec"] = added[0]
            self.save()
        return added

    def relink(self, new_root):
        """Point recordings at a new folder (after copying the project to another computer)."""
        n = 0
        for r in self.recordings():
            if os.path.exists(r["path"]):
                continue
            parts = r["path"].replace("\\", "/").split("/")
            for k in range(len(parts) - 1, 0, -1):
                cand = os.path.join(new_root, *parts[k:])
                if os.path.exists(os.path.join(cand, "video.avi")):
                    r["path"] = cand
                    n += 1
                    break
        self.save()
        return n

    # --- object types -----------------------------------------------------------------------
    def used_classes(self):
        used = set()
        for r in self.recordings():
            p = os.path.join(self.labels_dir(r["id"]), "labels.csv")
            if os.path.exists(p):
                with open(p, newline="", encoding="utf-8") as f:
                    for row in csv.DictReader(f):
                        if row.get("status") and row["status"] != "pending":
                            used.add(row.get("cls", ""))
        used.discard("none")
        return used

    def set_classes(self, classes, save=True):
        """Replace the object types. The catch-all type ("unknown") is always kept: it is added
        at the end when missing."""
        classes = [dict(c) for c in classes]
        names = [str(c.get("name", "")).strip() for c in classes]
        if any(not n for n in names):
            raise ValueError("every object type needs a name")
        if CATCH_ALL not in names:
            classes.append(dict(CATCH_ALL_TYPE))
            names.append(CATCH_ALL)
        if len(set(names)) != len(names):
            raise ValueError("two object types have the same name")
        if len(names) > 9:
            raise ValueError("at most 9 object types (keys 1-9), including \"%s\"" % CATCH_ALL)
        for n in names:
            if not re.match(r"^[A-Za-z0-9_-]{1,32}$", n):
                raise ValueError("object type %r: use letters, digits, - and _ only" % n)
            if n.lower() == "none":
                raise ValueError("\"none\" can't be a type name (it marks an empty frame in labels.csv)")
        missing = self.used_classes() - set(names)
        if missing:
            raise ValueError("these types are used in confirmed labels and can't be removed or renamed: %s"
                             % ", ".join(sorted(missing)))
        out = []
        for c in classes:
            size = c.get("size_px")
            varies = bool(c.get("height_varies"))
            out.append(dict(name=str(c["name"]).strip(), has_front=bool(c.get("has_front")),
                            size_px=float(size) if size else None,
                            height_cm=float(c.get("height_cm") or 0.0),
                            height_varies=varies, note=str(c.get("note") or "").strip()[:120]))
        self.data["classes"] = out
        if save:
            self.save()

    def class_view(self):
        used = self.used_classes()
        return [dict(c, locked=c["name"] in used, height_varies=bool(c.get("height_varies")),
                     note=c.get("note") or "", catch_all=c["name"] == CATCH_ALL) for c in self.classes]

    # --- models -------------------------------------------------------------------------------
    def models(self):
        return self.data["models"]

    def model(self, mid):
        for m in self.models():
            if m["id"] == mid:
                return m
        raise KeyError("no model %r" % mid)

    def model_dir(self, mid):
        return os.path.join(self.folder, "models", mid)

    def new_model_id(self):
        n = 1 + max([0] + [int(m["id"][1:]) for m in self.models() if re.match(r"^v\d+$", m["id"])])
        return "v%d" % n

    # --- cameras --------------------------------------------------------------------------------
    def camera_model_path(self, camera):
        return os.path.join(self.folder, "cameras", slug(camera) + ".json") if camera else None
