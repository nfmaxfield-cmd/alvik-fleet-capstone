#!/usr/bin/env python3
"""Free the Pi's card safely (2026-10-28): delete only session folders whose upload to GitHub is CONFIRMED.

"Confirmed" means: a release on GitHub holds this session, and every file the upload made from it is there with
exactly the size the folder makes today. The zips are not kept on the Pi, so each one is rebuilt in memory (nothing
written; the same code as the upload, tools/pi/upload_session.py zip_folder) and only its size is compared: a file
added, removed or changed since the upload changes the size, and the folder is then NOT deleted. Big files (> 50 MB)
went up as parts: every part must be there, adding up to the file's size.

What it looks at:
  - every session folder under logs/ (logs/<date-time>-..., logs/e02-...);
  - its frames folder (frames_where.txt: the USB drive's fleet-frames/<session>/frames; or <session>/frames);
  - the upload's own zip copies in .upload-tmp (on the drive, or ~/.upload-tmp): a copy whose name and exact size are
    on GitHub.
Anything not confirmed is listed with the reason and never deleted (dry runs are never uploaded, so never deleted).

    python3 tools/pi/clean_uploaded.py                 # the list only (nothing deleted)
    python3 tools/pi/clean_uploaded.py --yes           # delete what the list marks DELETE
    python3 tools/pi/clean_uploaded.py --frames-only   # keep every logs folder; only frames and zip copies
run-fleet-pi.sh clean-uploaded [--yes] [--frames-only] runs this. The GitHub token (~/.config/mirlt/github_token) is
used if it is there (more requests an hour); the repository is public, so it also works without one. It is never shown.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "fleet")):
    if p not in sys.path:
        sys.path.insert(0, p)

import upload_session as US   # noqa: E402
import diskguard as DG        # noqa: E402

SESSION_RE = re.compile(r"^(\d{8}-\d{6}-.+|e02-.+)$")
RUNNING = ("doe", "monday", "ladder", "ladder2", "e02")     # tmux sessions of runs that write into logs/


class Sink:
    """A file that only counts: zipfile writes into it (seeking back to fix headers) and size is the zip's size."""

    def __init__(self):
        self.pos = self.size = 0

    def write(self, b):
        self.pos += len(b)
        self.size = max(self.size, self.pos)
        return len(b)

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = off if whence == 0 else (self.pos + off if whence == 1 else self.size + off)
        return self.pos

    def seekable(self):
        return True

    def flush(self):
        pass


def zip_size(folder, small_only=False, exclude=()):
    s = Sink()
    US.zip_folder(folder, s, small_only=small_only, exclude=tuple(exclude))
    return s.size


def _get(url, tok):
    req = urllib.request.Request(url)
    if tok:
        req.add_header("Authorization", "Bearer " + tok)
    req.add_header("Accept", "application/vnd.github+json")
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read() or b"[]")


def releases(tok=None, get=None):
    """{tag: {asset name: size}} of every release (only assets GitHub reports as fully uploaded)."""
    get = get or _get
    out, page = {}, 1
    while True:
        rows = get("https://api.github.com/repos/%s/releases?per_page=100&page=%d" % (US.REPO_SLUG, page), tok)
        for r in rows:
            out[r["tag_name"]] = {a["name"]: int(a["size"]) for a in r.get("assets", [])
                                  if a.get("state", "uploaded") == "uploaded"}
        if len(rows) < 100:
            return out
        page += 1


def manifest(name):
    try:
        with open(os.path.join(US.STATE_DIR, "uploads", name + ".json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def tags_for(name, rels, man=None):
    """The releases that may hold this session: the manifest's tag first, then any tag ending in the session's name
    (doe-<name>, mission-<name>, p0-<name>, e02-session-<date-time>), with or without -2, -3 (--unique-tag)."""
    core = name[4:] if name.startswith("e02-") else name
    pat = re.compile(r"(^|-)%s(-\d+)?$" % re.escape(core))
    out = []
    if man and man.get("tag") in rels:
        out.append(man["tag"])
    out += sorted(t for t in rels if pat.search(t) and t not in out)
    return out


def parts_ok(folder, assets):
    """Every file over the small-files size went up as <path>.part-aa, -ab, ...: all there, sizes adding up."""
    for root, _, files in os.walk(folder):
        for fn in files:
            p = os.path.join(root, fn)
            size = os.path.getsize(p)
            if size <= US.SMALL_BYTES:
                continue
            stem = os.path.relpath(p, folder).replace(os.sep, "_")
            k, got = 0, 0
            for start in range(0, size, US.PART_BYTES):
                part = "%s.part-%s%s" % (stem, chr(97 + k // 26), chr(97 + k % 26))
                if part not in assets or assets[part] != min(US.PART_BYTES, size - start):
                    return False, "%s not all on GitHub" % stem
                got += assets[part]
                k += 1
            if got != size:
                return False, "%s's parts do not add up" % stem
    return True, ""


def check_frames(fd, assets, say=print):
    """(confirmed, why) for a frames folder against one release's assets."""
    name = "rec-%s-small.zip" % os.path.basename(fd.rstrip("/"))
    if name not in assets:
        return False, "%s is not in the release" % name
    raw = DG.folder_bytes(fd)
    if raw > 3 * assets[name] + 1e6 or raw < assets[name] / 3:
        return False, "%s on GitHub is %d bytes; the folder (%d bytes) cannot make that" % (name, assets[name], raw)
    say("    rebuilding %s in memory to compare its size (%.1f GB: about a minute a GB on the Pi) ..." % (
        name, raw / 1e9))
    z = zip_size(fd, small_only=True)
    if z != assets[name]:
        return False, "%s: GitHub has %d bytes, the folder makes %d now (changed since the upload)" % (
            name, assets[name], z)
    return parts_ok(fd, assets)


def check_session(name, sess, fd, rels, say=print):
    """What can go for this session: dict(logs, frames, tag, why). logs: its logs folder is confirmed; frames: its
    frames folder is confirmed (True when it has none). Each part may be confirmed by a different release."""
    man = manifest(name)
    tags = tags_for(name, rels, man)
    has_frames = fd is not None and os.path.isdir(fd)
    frames_inside = has_frames and sess is not None and os.path.dirname(os.path.normpath(fd)) == os.path.normpath(sess)
    logs_ok, frames_ok, used, why = False, not has_frames, [], []
    if not tags:
        why.append("no release on GitHub holds it")
    for tag in tags:
        assets = rels[tag]
        if sess is not None and not logs_ok:
            lz = "logs-%s.zip" % name
            if lz not in assets:
                why.append("%s is not in %s" % (lz, tag))
            else:
                excl = [tuple(man["exclude"])] if man.get("tag") == tag and "exclude" in man else [(), ("frames",)]
                for ex in excl:
                    if zip_size(sess, exclude=ex) == assets[lz]:
                        logs_ok = True
                        used.append(tag)
                        if frames_inside and "frames" not in ex:
                            frames_ok = True          # the frames went up inside the logs zip
                        break
                if not logs_ok:
                    why.append("%s: GitHub has %d bytes, the folder makes another size now (changed since the "
                               "upload)" % (lz, assets[lz]))
        if has_frames and not frames_ok:
            ok, w = check_frames(fd, assets, say)
            if ok:
                frames_ok = True
                if tag not in used:
                    used.append(tag)
            else:
                why.append(w)
        if (logs_ok or sess is None) and frames_ok:
            break
    return dict(logs=logs_ok, frames=frames_ok, tag=", ".join(used), why="; ".join(why))


def zip_copies(rels):
    """The upload's zip copies whose name and exact size are an asset on GitHub: [(path, bytes, tag)]."""
    dirs = {os.path.expanduser("~/.upload-tmp"), os.path.join(os.path.dirname(DG.rec_out().rstrip("/")), ".upload-tmp")}
    d = DG.drive_mount()
    if d:
        dirs.add(os.path.join(d, ".upload-tmp"))
    by = {}
    for tag, assets in rels.items():
        for n, s in assets.items():
            by.setdefault((n, s), tag)
    out = []
    for dd in sorted(dirs):
        if not os.path.isdir(dd):
            continue
        for fn in sorted(os.listdir(dd)):
            p = os.path.join(dd, fn)
            if os.path.isfile(p) and not os.path.islink(p):
                s = os.path.getsize(p)
                out.append((p, s, by.get((fn, s))))
    return out


def running():
    out = []
    for s in RUNNING:
        try:
            if subprocess.call(["tmux", "has-session", "-t", s], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL) == 0:
                out.append(s)
        except OSError:
            pass
    return out


def _gb(n):
    return "%.2f GB" % (n / 1e9) if n >= 1e8 else "%.0f MB" % (n / 1e6)


def plan(logs_dir, rels, frames_only=False, say=print):
    """[(path, bytes, where, what)] to delete, and [(name, why)] kept."""
    drive = DG.drive_mount()
    ff = os.path.join(drive, "fleet-frames") if drive else None
    names = set()
    if os.path.isdir(logs_dir):
        names |= {n for n in os.listdir(logs_dir) if SESSION_RE.match(n) and os.path.isdir(os.path.join(logs_dir, n))}
    if ff and os.path.isdir(ff):
        names |= {n for n in os.listdir(ff) if os.path.isdir(os.path.join(ff, n))}
    dele, kept = [], []
    for name in sorted(names):
        sess = os.path.join(logs_dir, name)
        sess = sess if os.path.isdir(sess) else None
        fd = DG.where_frames(sess) if sess else None
        if (fd is None or not os.path.isdir(fd)) and ff and os.path.isdir(os.path.join(ff, name, "frames")):
            fd = os.path.join(ff, name, "frames")
        if fd is not None and not os.path.isdir(fd):
            fd = None
        if name.endswith("-dryrun") or name.endswith("-dry"):
            kept.append((name, "a dry run: never uploaded"))
            continue
        say("  checking %s ..." % name)
        r = check_session(name, sess, fd, rels, say)
        fd_inside = fd is not None and sess is not None and os.path.dirname(os.path.normpath(fd)) == \
            os.path.normpath(sess)
        if sess and r["logs"] and r["frames"] and not frames_only:
            dele.append((sess, DG.folder_bytes(sess), "card", "%s (logs%s; on GitHub as %s)" % (
                name, " and frames" if fd_inside else "", r["tag"])))
            if fd is not None and not fd_inside:
                dele.append((fd, DG.folder_bytes(fd), "drive", "%s frames (on GitHub as %s)" % (name, r["tag"])))
            continue
        if fd is not None and r["frames"]:
            dele.append((fd, DG.folder_bytes(fd), "card" if fd_inside or DG.same_fs(fd, logs_dir) else "drive",
                         "%s frames only (on GitHub as %s)" % (name, r["tag"])))
            if sess and frames_only and r["logs"]:
                kept.append((name, "its logs folder is confirmed on GitHub (%s), kept because of --frames-only"
                             % r["tag"]))
            elif sess:
                kept.append((name, "its logs folder: " + (r["why"] or "not confirmed")))
            continue
        if frames_only and r["logs"]:
            kept.append((name, "its logs folder is confirmed on GitHub (%s), kept because of --frames-only" % r["tag"]))
            continue
        kept.append((name, r["why"] or "not confirmed"))
    for p, s, tag in zip_copies(rels):
        if tag:
            dele.append((p, s, "card" if DG.same_fs(p, logs_dir) else "drive",
                         "upload copy %s (the same size is on GitHub in %s)" % (os.path.basename(p), tag)))
        else:
            kept.append((os.path.basename(p), "an upload copy GitHub does not have at this size (an upload that "
                                              "did not finish: run the upload again)"))
    return dele, kept


def safe_path(p, logs_dir):
    """Only inside logs/, a fleet-frames folder or an .upload-tmp folder, never a link."""
    rp = os.path.realpath(p)
    if os.path.islink(p):
        return False
    roots = [os.path.realpath(logs_dir)]
    d = DG.drive_mount()
    if d:
        roots += [os.path.realpath(os.path.join(d, "fleet-frames")), os.path.realpath(os.path.join(d, ".upload-tmp"))]
    roots += [os.path.realpath(os.path.expanduser("~/.upload-tmp")),
              os.path.realpath(os.path.join(os.path.dirname(DG.rec_out().rstrip("/")), ".upload-tmp"))]
    return any(rp != r and rp.startswith(r + os.sep) for r in roots)


def main(argv=None, get=None, say=print):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yes", action="store_true", help="delete what the list marks DELETE")
    ap.add_argument("--frames-only", action="store_true", help="keep every logs folder")
    ap.add_argument("--logs", default=os.path.join(REPO, "logs"))
    a = ap.parse_args(argv)
    say("Asking GitHub which sessions are uploaded ...")
    try:
        rels = releases(US.token(), get=get)
    except (urllib.error.URLError, OSError, ValueError) as e:
        say("Could not read the releases from GitHub (%s): nothing can be confirmed, so nothing is deleted. Check the "
            "WiFi and try again." % e)
        return 2
    say("%d releases on GitHub." % len(rels))
    dele, kept = plan(a.logs, rels, a.frames_only, say)
    say("")
    say("KEPT (not confirmed on GitHub; never deleted):")
    for n, why in kept:
        say("  keep    %s: %s" % (n, why))
    if not kept:
        say("  (none)")
    say("")
    say("CONFIRMED on GitHub (exact sizes match):")
    tot = {"card": 0, "drive": 0}
    for p, s, where, what in dele:
        tot[where] += s
        say("  DELETE  %9s  %-5s  %s   [%s]" % (_gb(s), where, what, p))
    if not dele:
        say("  (none)")
    say("")
    say("That frees %s on the card and %s on the USB drive." % (_gb(tot["card"]), _gb(tot["drive"])))
    if not a.yes:
        say("Nothing was deleted. To delete exactly the DELETE lines above: ./run-fleet-pi.sh clean-uploaded --yes")
        return 0
    busy = running()
    if busy:
        say("NOT DELETED: a run is going (tmux %s). Run this again after it ends." % ", ".join(busy))
        return 1
    freed = 0
    for p, s, where, what in dele:
        if not safe_path(p, a.logs):
            say("  skipped (outside the folders this may touch): %s" % p)
            continue
        try:
            if os.path.isdir(p):
                shutil.rmtree(p)
            else:
                os.remove(p)
            freed += s
            say("  deleted %s" % p)
        except OSError as e:
            say("  could not delete %s: %s" % (p, e))
    say("Freed %s. The card now has %.1f GB free." % (_gb(freed), DG.free_gb(a.logs) or 0.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
