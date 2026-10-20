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

# Default object types for the Alvik lab. height_cm: where the labelled point sits above
# the floor (the robot's top for an Alvik), used to share labels between cameras.
DEFAULT_CLASSES = [
    dict(name="alvik", has_front=True, size_px=None, height_cm=4.0),
    dict(name="hand", has_front=False, size_px=None, height_cm=10.0),
    dict(name="unknown", has_front=False, size_px=None, height_cm=0.0),
    dict(name="machine", has_front=True, size_px=None, height_cm=0.0),
]


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

    def set_classes(self, classes):
        names = [str(c.get("name", "")).strip() for c in classes]
        if not names or any(not n for n in names):
            raise ValueError("every object type needs a name")
        if len(set(names)) != len(names):
            raise ValueError("two object types have the same name")
        if len(names) > 9:
            raise ValueError("at most 9 object types (keys 1-9)")
        for n in names:
            if not re.match(r"^[A-Za-z0-9_-]{1,32}$", n):
                raise ValueError("object type %r: use letters, digits, - and _ only" % n)
        missing = self.used_classes() - set(names)
        if missing:
            raise ValueError("these types are used in confirmed labels and can't be removed or renamed: %s"
                             % ", ".join(sorted(missing)))
        out = []
        for c in classes:
            size = c.get("size_px")
            out.append(dict(name=str(c["name"]).strip(), has_front=bool(c.get("has_front")),
                            size_px=float(size) if size else None, height_cm=float(c.get("height_cm") or 0.0)))
        self.data["classes"] = out
        self.save()

    def class_view(self):
        used = self.used_classes()
        return [dict(c, locked=c["name"] in used) for c in self.classes]

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
