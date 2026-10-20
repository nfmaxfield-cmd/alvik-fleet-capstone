#!/usr/bin/env python3
"""Maxfield IRL Tracking: a whole project as one file (.mxirl).

    python tools/recognition/share.py export <project folder> <file or folder> [--with-video]
    python tools/recognition/share.py import <file> <new folder> [--recordings <folder to search>]

A .mxirl file is a zip: manifest.json, and every file of the project under project/.
Recording videos are left out unless --with-video (they are large); the manifest still lists
each recording with its original path and name, so a copy on another computer can be found again.

For the studio:
    export_project(project_or_folder, out_path, with_video=False, progress=None) -> dict
    import_project(file, dest, recordings_dir=None, progress=None) -> dict
    read_manifest(file) -> dict
    missing_recordings(project) -> list of dicts (id, name, camera, session, path)
    relink(project, rid, new_path)            one recording, picked by the person
    relink_from_folder(project, folder) -> (found ids, still missing dicts)
    describe_missing(missing) -> plain-word lines
progress(done, total) counts bytes.
Exporting only reads the project: it never changes the source, and it re-reads any small file
that changed while it was being copied (the studio can keep running).
"""
import argparse
import datetime as dt
import json
import os
import shutil
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import project as projmod  # noqa: E402

FORMAT = "mxirl"
FORMAT_VERSION = 1
SOFTWARE = "Maxfield IRL Tracking"
EXT = ".mxirl"
CHUNK = 1 << 20
SMALL = 64 << 20          # files up to this size are checked for changes while copying


class ShareError(ValueError):
    """A problem in plain words, safe to show a person."""


def _now():
    return dt.datetime.now().isoformat(timespec="seconds")


def _base(path):
    return str(path).replace("\\", "/").rstrip("/").split("/")[-1]


def _as_project(p):
    return p if isinstance(p, projmod.Project) else projmod.Project(p)


# --- export ------------------------------------------------------------------------------------
def _walk(folder, skip=()):
    out = []
    for root, dirs, files in os.walk(folder):
        dirs[:] = sorted(d for d in dirs if d != "__pycache__")
        for fn in sorted(files):
            if fn.endswith(".tmp"):
                continue
            full = os.path.join(root, fn)
            if os.path.abspath(full) in skip:
                continue
            out.append((full, os.path.relpath(full, folder).replace(os.sep, "/")))
    return out


def _sig(path):
    st = os.stat(path)
    return st.st_size, st.st_mtime_ns


def _write_file(zf, full, arc, tick, compress=True):
    """Add one file. Small files are read whole and re-read if they changed meanwhile."""
    ctype = zipfile.ZIP_DEFLATED if compress else zipfile.ZIP_STORED
    try:
        before = _sig(full)
        if before[0] <= SMALL:
            for _ in range(6):
                with open(full, "rb") as f:
                    data = f.read()
                if _sig(full) == before and len(data) == before[0]:
                    break
                before = _sig(full)
            zi = zipfile.ZipInfo.from_file(full, arc)
            zi.compress_type = ctype
            zf.writestr(zi, data)
            tick(len(data))
        else:
            zi = zipfile.ZipInfo.from_file(full, arc)
            zi.compress_type = zipfile.ZIP_STORED
            with open(full, "rb") as f, zf.open(zi, "w", force_zip64=True) as w:
                while True:
                    b = f.read(CHUNK)
                    if not b:
                        break
                    w.write(b)
                    tick(len(b))
        return True
    except FileNotFoundError:
        return False


def export_project(project, out_path, with_video=False, progress=None):
    proj = _as_project(project)
    out_path = os.path.abspath(out_path)
    if os.path.isdir(out_path):
        out_path = os.path.join(out_path, projmod.slug(proj.name) + EXT)
    elif not os.path.splitext(out_path)[1]:
        out_path += EXT
    if os.path.abspath(out_path).startswith(proj.folder + os.sep):
        raise ShareError("Save the file outside the project folder.")
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    # one consistent copy of project.json (it is replaced atomically by the studio)
    with open(proj.path, "rb") as f:
        pj_bytes = f.read()
    pdata = json.loads(pj_bytes.decode("utf-8"))

    files = [(p, a) for p, a in _walk(proj.folder, skip={os.path.abspath(out_path)})
             if a != "project.json"]
    recs, vid_files, warnings = [], [], []
    for r in pdata.get("recordings", []):
        e = dict(id=r["id"], name=r.get("name"), camera=r.get("camera"), session=r.get("session"),
                 original_path=r["path"], video_included=False)
        if with_video:
            if os.path.isdir(r["path"]) and os.path.exists(os.path.join(r["path"], "video.avi")):
                stored = "recordings/" + projmod.slug(r["id"])
                for full, rel in _walk(r["path"]):
                    vid_files.append((full, stored + "/" + rel))
                e.update(video_included=True, stored_path=stored)
            else:
                warnings.append("The video for recording %s was not found, so it is not in the file." % r["id"])
        recs.append(e)

    def size(p):
        try:
            return os.path.getsize(p)
        except OSError:
            return 0
    total = len(pj_bytes) + sum(size(p) for p, _ in files + vid_files)
    state = dict(done=0)

    def tick(n):
        state["done"] += n
        if progress:
            progress(min(state["done"], total), total)

    def count(prefix):
        return len({a.split("/")[1] for _, a in files if a.startswith(prefix) and a.count("/") >= 2})
    inside = dict(files=len(files) + 1, recordings_with_labels=count("labels/"),
                  models=len([m for m in pdata.get("models", [])]), cameras=len([a for _, a in files if a.startswith("cameras/")]),
                  has_learning_curve=any(a == "models/curve.json" for _, a in files),
                  has_stats=any(a.startswith("stats/") for _, a in files),
                  video_files=len(vid_files))
    manifest = dict(format=FORMAT, format_version=FORMAT_VERSION, software=SOFTWARE,
                    project_name=pdata.get("name", proj.name), created=_now(), with_video=bool(with_video),
                    contents=inside, recordings=recs, warnings=warnings)

    tmp = out_path + ".part"
    try:
        with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED, allowZip64=True) as zf:
            zf.writestr("manifest.json", json.dumps(manifest, indent=1))
            zf.writestr("project/project.json", pj_bytes)
            tick(len(pj_bytes))
            for full, arc in files:
                if not _write_file(zf, full, "project/" + arc, tick):
                    pass
            for full, arc in vid_files:
                _write_file(zf, full, "project/" + arc, tick, compress=False)
        os.replace(tmp, out_path)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise
    if progress:
        progress(total, total)
    return dict(path=out_path, bytes=os.path.getsize(out_path), manifest=manifest)


# --- import --------------------------------------------------------------------------------------
def _safe_member(name):
    n = name.replace("\\", "/")
    if not n or n.startswith("/") or (len(n) > 1 and n[1] == ":") or "\x00" in n:
        return None
    parts = [p for p in n.split("/") if p not in ("", ".")]
    if not parts or ".." in parts:
        return None
    return "/".join(parts)


def read_manifest(file):
    try:
        with zipfile.ZipFile(file) as zf:
            m = json.loads(zf.read("manifest.json").decode("utf-8"))
    except (zipfile.BadZipFile, KeyError, ValueError, OSError):
        raise ShareError("This is not a Maxfield IRL Tracking project file (%s)." % _base(file))
    if m.get("format") != FORMAT:
        raise ShareError("This is not a Maxfield IRL Tracking project file (%s)." % _base(file))
    if int(m.get("format_version", 0)) > FORMAT_VERSION:
        raise ShareError("This file was made by a newer version of the software. Update the software first.")
    return m


def _index_videos(root, max_depth=6):
    """Every folder under root holding video.avi -> {folder name: [paths]}."""
    idx = {}
    root = os.path.abspath(root)
    base_depth = root.count(os.sep)
    for r, dirs, files in os.walk(root):
        if r.count(os.sep) - base_depth >= max_depth:
            dirs[:] = []
        if "video.avi" in files:
            idx.setdefault(os.path.basename(r), []).append(r)
    return idx


def _find(idx, rec):
    orig = rec.get("path") or rec.get("original_path") or ""
    cands = sorted(idx.get(_base(orig), []))
    if rec.get("session"):
        cands = [c for c in cands if os.path.basename(os.path.dirname(c)) == rec["session"]]
    elif len(cands) > 1:
        parent = _base("/".join(orig.replace("\\", "/").split("/")[:-1]))
        pref = [c for c in cands if os.path.basename(os.path.dirname(c)) == parent]
        cands = pref or cands
    return cands[0] if cands else None


def missing_recordings(project):
    """Recordings whose folder or video.avi can't be found (the project still opens)."""
    proj = _as_project(project)
    return [dict(id=r["id"], name=r.get("name"), camera=r.get("camera"), session=r.get("session"), path=r["path"])
            for r in proj.recordings() if not os.path.exists(os.path.join(r["path"], "video.avi"))]


def relink(project, rid, new_path):
    """Point one recording at a folder the person picked (the recording folder, with video.avi)."""
    proj = _as_project(project)
    rec = proj.rec(rid)
    new_path = os.path.abspath(os.path.expanduser(str(new_path).strip().strip('"')))
    if not os.path.exists(os.path.join(new_path, "video.avi")):
        raise ShareError("There is no video.avi in %s. Pick the recording's own folder." % new_path)
    rec["path"] = new_path
    proj.save()
    return rec


def relink_from_folder(project, folder):
    """Search `folder` for every missing recording (by session/recording folder name, then camera folder)."""
    proj = _as_project(project)
    if not os.path.isdir(folder):
        raise ShareError("The folder %s does not exist." % folder)
    idx = _index_videos(folder)
    found, changed = [], False
    for m in missing_recordings(proj):
        hit = _find(idx, m)
        if hit:
            proj.rec(m["id"])["path"] = hit
            found.append(m["id"])
            changed = True
    if changed:
        proj.save()
    return found, missing_recordings(proj)


def describe_missing(missing):
    out = []
    for m in missing:
        what = m.get("name") or m["id"]
        if m.get("session") and m.get("camera"):
            what += " (camera %s)" % m["camera"]
        out.append("Recording not found: %s. It was at %s" % (what, m["path"]))
    return out


def import_project(file, dest, recordings_dir=None, progress=None):
    manifest = read_manifest(file)
    if recordings_dir and not os.path.isdir(recordings_dir):
        raise ShareError("The folder %s does not exist. Nothing was imported." % recordings_dir)
    dest = os.path.abspath(dest)
    if os.path.exists(dest):
        if not os.path.isdir(dest) or os.listdir(dest):
            raise ShareError("The folder %s already has files in it. Pick a new or empty folder; nothing was changed." % dest)
        created = False
    else:
        created = True
    with zipfile.ZipFile(file) as zf:
        items = []
        for zi in zf.infolist():
            if zi.filename == "manifest.json":
                continue
            safe = _safe_member(zi.filename)
            if safe is None or not safe.startswith("project/"):
                raise ShareError("This file is damaged or unsafe (it contains a file outside the project: %r). Nothing was imported." % zi.filename)
            if zi.is_dir():
                continue
            target = os.path.abspath(os.path.join(dest, safe[len("project/"):]))
            if not target.startswith(dest + os.sep):
                raise ShareError("This file is damaged or unsafe (%r). Nothing was imported." % zi.filename)
            items.append((zi, target))
        if not any(t == os.path.join(dest, "project.json") for _, t in items):
            raise ShareError("This file has no project in it.")
        total = sum(zi.file_size for zi, _ in items) or 1
        done = 0
        try:
            os.makedirs(dest, exist_ok=True)
            for zi, target in items:
                os.makedirs(os.path.dirname(target), exist_ok=True)
                with zf.open(zi) as src, open(target, "wb") as dst:
                    while True:
                        b = src.read(CHUNK)
                        if not b:
                            break
                        dst.write(b)
                        done += len(b)
                        if progress:
                            progress(min(done, total), total)
        except BaseException:
            if created:
                shutil.rmtree(dest, ignore_errors=True)
            else:
                for n in os.listdir(dest):
                    p = os.path.join(dest, n)
                    shutil.rmtree(p, ignore_errors=True) if os.path.isdir(p) else os.remove(p)
            raise
    if progress:
        progress(total, total)

    proj = projmod.Project(dest)
    # included videos come first; paths are rewritten without touching anything else
    for e in manifest.get("recordings", []):
        if e.get("video_included") and e.get("stored_path"):
            try:
                proj.rec(e["id"])["path"] = os.path.join(dest, *e["stored_path"].split("/"))
            except KeyError:
                pass
    projmod.write_json(proj.path, proj.data)
    if recordings_dir:
        relink_from_folder(proj, recordings_dir)
    miss = missing_recordings(proj)
    n = len(proj.recordings())
    return dict(project=proj, dest=dest, manifest=manifest, recordings=n, found=n - len(miss),
                missing=miss, notes=describe_missing(miss))


# --- command line ----------------------------------------------------------------------------------
class _Bar:
    def __init__(self):
        self.last = -1

    def __call__(self, done, total):
        pct = int(100 * done / max(total, 1))
        if pct != self.last and pct % 10 == 0:
            self.last = pct
            print("  %d%% (%.0f of %.0f MB)" % (pct, done / 1e6, total / 1e6))
            sys.stdout.flush()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Save a Maxfield IRL Tracking project as one file, or open one.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export", help="project folder -> one .mxirl file")
    e.add_argument("project")
    e.add_argument("file")
    e.add_argument("--with-video", action="store_true", help="also put the recordings' videos in the file (large)")
    i = sub.add_parser("import", help="a .mxirl file -> a new project folder")
    i.add_argument("file")
    i.add_argument("dest")
    i.add_argument("--recordings", help="a folder to search for the recordings (matched by folder name)")
    a = ap.parse_args(argv)
    try:
        if a.cmd == "export":
            print("Saving the project as one file...")
            res = export_project(a.project, a.file, with_video=a.with_video, progress=_Bar())
            m = res["manifest"]
            print("Done: %s (%.1f MB)" % (res["path"], res["bytes"] / 1e6))
            print("It holds the project '%s': %d recording(s) listed, %d model(s), videos %s."
                  % (m["project_name"], len(m["recordings"]), m["contents"]["models"],
                     "included" if m["with_video"] else "not included (use --with-video to include them)"))
            for w in m["warnings"]:
                print(w)
        else:
            print("Opening the project file...")
            res = import_project(a.file, a.dest, a.recordings, progress=_Bar())
            print("Done: the project '%s' is now in %s" % (res["manifest"]["project_name"], res["dest"]))
            print("Recordings found: %d of %d." % (res["found"], res["recordings"]))
            for line in res["notes"]:
                print(line)
            if res["missing"]:
                print("The project still opens. In the studio, point each missing recording to its folder.")
    except ShareError as ex:
        print(str(ex))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
