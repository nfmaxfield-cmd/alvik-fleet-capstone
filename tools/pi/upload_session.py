#!/usr/bin/env python3
"""Upload a finished session to GitHub (a release), so it can be fetched while Nick is away (Nick, 2026-10-24).

What goes up, as assets of one release (tag e02-session-<date-time>):
  - logs-<session>.zip: the runner's session folder (logs/e02-<date-time>), everything in it;
  - rec-<folder>-small.zip: the recording's small files (frames.csv, events.csv, meta.json, ...);
  - each video, split into parts under 1.9 GB (GitHub's limit is 2 GB a file): <file>.part-aa, -ab, ...
An interrupted upload can be run again: parts already in the release are skipped.

The GitHub token is read from ~/.config/mirlt/github_token (a fine-grained token for this repository with
"Contents: read and write"; set up once, see docs/pi-run.md). Nothing is deleted on the Pi. The laptop's
mark-the-robot page (tools/label/mark_robot.py) uploads its marks with the same token file and code (on Windows the
file is C:/Users/<you>/.config/mirlt/github_token; docs/how-to/mark-the-robot.md sets it up).

    python3 tools/pi/upload_session.py --recordings /media/<user>/<drive>/recordings
    python3 tools/pi/upload_session.py --recordings ... --stop-recorder     # first Ctrl+C the recorder (tmux e02:rec)
    python3 tools/pi/upload_session.py --set-token                          # once: paste the GitHub token
    python3 tools/pi/upload_session.py --check-token                        # does the saved token still work?
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
import zipfile

REPO_SLUG = "nfmaxfield-cmd/alvik-fleet-capstone"
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
TOKEN_PATH = os.path.expanduser("~/.config/mirlt/github_token")
PART_BYTES = 1900 * 1024 * 1024
SMALL_BYTES = 50 * 1024 * 1024    # files up to this size go in the small-files zip
CHUNK = 4 * 1024 * 1024


def token():
    try:
        with open(TOKEN_PATH) as f:
            return f.read().strip()
    except OSError:
        return None


_open = urllib.request.urlopen     # tests replace it (no network)


def api(method, url, tok, data=None, ctype="application/json", length=None):
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("Authorization", "Bearer " + tok)
    req.add_header("Accept", "application/vnd.github+json")
    req.add_header("X-GitHub-Api-Version", "2022-11-28")
    if data is not None:
        req.add_header("Content-Type", ctype)
        if length is not None:
            req.add_header("Content-Length", str(length))
    with _open(req, timeout=600) as r:
        body = r.read()
        return json.loads(body) if body else {}


def release_for(tag, tok, body):
    base = "https://api.github.com/repos/%s/releases" % REPO_SLUG
    try:
        return api("GET", base + "/tags/" + tag, tok)
    except urllib.error.HTTPError as e:
        if e.code != 404:
            raise
    d = json.dumps(dict(tag_name=tag, name=tag, body=body, target_commitish="main", prerelease=True)).encode()
    return api("POST", base, tok, d)


class _Slice:
    """A file-like view of bytes [start, start + n) of a file, read in chunks (no 2 GB in memory)."""

    def __init__(self, path, start, n):
        self.f = open(path, "rb")
        self.f.seek(start)
        self.left = n

    def read(self, size=-1):
        if self.left <= 0:
            return b""
        size = self.left if size is None or size < 0 else min(size, self.left)
        b = self.f.read(min(size, CHUNK))
        self.left -= len(b)
        return b

    def close(self):
        self.f.close()


def upload(rel, name, path, tok, start=0, n=None, have=()):
    if name in have:
        print("  %s: already up, skipped" % name, flush=True)
        return
    n = os.path.getsize(path) - start if n is None else n
    url = rel["upload_url"].split("{")[0] + "?name=" + urllib.request.quote(name)
    for attempt in range(3):
        src = _Slice(path, start, n)
        try:
            t0 = time.time()
            print("  %s: %.0f MB ..." % (name, n / 1e6), flush=True)
            api("POST", url, tok, src, "application/octet-stream", n)
            print("  %s: up (%.0f s)" % (name, time.time() - t0), flush=True)
            return
        except (urllib.error.URLError, OSError) as e:
            print("  %s: failed (%s), trying again" % (name, e), flush=True)
            time.sleep(10)
        finally:
            src.close()
    raise SystemExit("Upload of %s failed 3 times: run this again later (finished parts are skipped)." % name)


def zip_folder(folder, out, small_only=False):
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for root, _, files in os.walk(folder):
            for fn in files:
                p = os.path.join(root, fn)
                if small_only and os.path.getsize(p) > SMALL_BYTES:
                    continue
                z.write(p, os.path.relpath(p, os.path.dirname(folder)))
    return out


def stop_recorder():
    """Ctrl+C to the recorder's tmux window, then wait for it to save."""
    if subprocess.call(["tmux", "has-session", "-t", "e02"], stderr=subprocess.DEVNULL) != 0:
        return
    subprocess.call(["tmux", "send-keys", "-t", "e02:rec", "C-c"])
    print("Recorder told to stop; waiting 20 s for it to save ...", flush=True)
    time.sleep(20)


def set_token():
    """Paste the token once (it is not shown); it is checked against GitHub and saved for this user only."""
    import getpass
    tok = getpass.getpass("Paste the GitHub token (it will not show) and press Enter: ").strip()
    if not tok:
        raise SystemExit("Nothing pasted: not saved.")
    try:
        r = api("GET", "https://api.github.com/repos/%s" % REPO_SLUG, tok)
    except urllib.error.HTTPError as e:
        raise SystemExit("GitHub refused that token (%s): not saved. Check it was copied whole, and that it has this "
                         "repository with Contents: read and write." % e.code)
    if not (r.get("permissions") or {}).get("push"):
        raise SystemExit("That token can read the repository but not write to it: give it Contents: read and write. "
                         "Not saved.")
    os.makedirs(os.path.dirname(TOKEN_PATH), exist_ok=True)
    with open(TOKEN_PATH, "w") as f:
        f.write(tok + "\n")
    try:
        os.chmod(TOKEN_PATH, 0o600)
    except OSError:
        pass
    print("Saved. This computer can upload to %s now (saved in %s)." % (REPO_SLUG, TOKEN_PATH))


def check_token():
    """Is the saved token there, and can it still write to the repository? Prints one plain line."""
    tok = token()
    if not tok:
        raise SystemExit("No token saved on this computer (%s). Save one with --set-token." % TOKEN_PATH)
    try:
        r = api("GET", "https://api.github.com/repos/%s" % REPO_SLUG, tok)
    except urllib.error.HTTPError as e:
        raise SystemExit("GitHub refused the saved token (%s): it has expired or was deleted. Make a new one and save "
                         "it with --set-token." % e.code)
    except (urllib.error.URLError, OSError) as e:
        raise SystemExit("Could not reach GitHub (%s): check the WiFi and try again." % e)
    if not (r.get("permissions") or {}).get("push"):
        raise SystemExit("The saved token can read the repository but not write to it: make a new one with "
                         "Contents: read and write, and save it with --set-token.")
    print("Token works: this computer can upload to %s." % REPO_SLUG)


def main():
    if "--set-token" in sys.argv:
        return set_token()
    if "--check-token" in sys.argv:
        return check_token()
    ap = argparse.ArgumentParser()
    ap.add_argument("--recordings", required=True, help="the folder the recorder saved into")
    ap.add_argument("--session", default="", help="the runner's session folder (default: the newest logs/e02-*)")
    ap.add_argument("--stop-recorder", action="store_true")
    ap.add_argument("--logs-only", action="store_true", help="only the session folder (between the sessions of a plan)")
    ap.add_argument("--no-logs", action="store_true", help="only the recording (the end of a plan)")
    ap.add_argument("--tag", default="", help="the release's tag (default: e02-session-<the session's date-time>)")
    a = ap.parse_args()
    if a.stop_recorder:
        stop_recorder()
    tok = token()
    if not tok:
        raise SystemExit("No GitHub token at %s: nothing uploaded. The files stay on the Pi and the USB drive "
                         "(docs/pi-run.md, 'GitHub token', sets it up)." % TOKEN_PATH)
    sess = a.session or max((d for d in glob.glob(os.path.join(REPO, "logs", "e02-*")) if "-dryrun" not in d),
                            key=os.path.getmtime, default="")
    recs = sorted((d for d in glob.glob(os.path.join(a.recordings, "*")) if os.path.isdir(d)), key=os.path.getmtime)
    rec = "" if a.logs_only else (recs[-1] if recs else "")
    if not sess:
        raise SystemExit("No session folder found under logs/.")
    name = os.path.basename(sess)
    tag = a.tag or ("e02-session-" + name.replace("e02-", ""))
    print("Uploading %s and recording %s to release %s" % (name, os.path.basename(rec) or "(none)", tag), flush=True)
    rel = release_for(tag, tok, "Session %s, uploaded from the Pi at the end of the session (tools/pi/upload_session.py)."
                      " Video parts: join them in order (cat video.avi.part-* > video.avi)." % name)
    have = {x["name"] for x in rel.get("assets", [])}
    tmp = os.path.join(os.path.dirname(a.recordings.rstrip("/")), ".upload-tmp")
    os.makedirs(tmp, exist_ok=True)
    if not a.no_logs:
        z = zip_folder(sess, os.path.join(tmp, "logs-%s.zip" % name))
        upload(rel, os.path.basename(z), z, tok, have=have)
    if rec:
        z = zip_folder(rec, os.path.join(tmp, "rec-%s-small.zip" % os.path.basename(rec)), small_only=True)
        upload(rel, os.path.basename(z), z, tok, have=have)
        for root, _, files in os.walk(rec):
            for fn in sorted(files):
                p = os.path.join(root, fn)
                size = os.path.getsize(p)
                if size <= SMALL_BYTES:
                    continue
                stem = os.path.relpath(p, rec).replace(os.sep, "_")
                k = 0
                for start in range(0, size, PART_BYTES):
                    part = "%s.part-%s%s" % (stem, chr(97 + k // 26), chr(97 + k % 26))
                    upload(rel, part, p, tok, start, min(PART_BYTES, size - start), have=have)
                    k += 1
    print("All up: https://github.com/%s/releases/tag/%s" % (REPO_SLUG, tag), flush=True)


if __name__ == "__main__":
    sys.exit(main())
