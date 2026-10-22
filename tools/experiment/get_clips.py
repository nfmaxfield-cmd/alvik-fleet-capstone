#!/usr/bin/env python3
"""Download the real footage the regression tests replay into tools/experiment/data/clips/, for
tests/test_e02_realclips.py: GitHub release `e02-setup-oct21` (setup.avi, 1.3 GB, and frames.csv) and the last piece
of release `e02-session-oct22` (video.part-ab, 0.3 GB, and its frames.csv, saved as oct22_frames.csv). Needs access to the repository: a GitHub token in
GITHUB_TOKEN (or `gh auth token`).

    python tools/experiment/get_clips.py
"""
import json
import os
import subprocess
import sys
import urllib.request

REPO = "nfmaxfield-cmd/alvik-fleet-capstone"
GET = (("e02-setup-oct21", {"setup.avi": "setup.avi", "frames.csv": "frames.csv"}),
       ("e02-session-oct22", {"video.part-ab": "oct22_video.part-ab", "frames.csv": "oct22_frames.csv"}))
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "clips")


def token():
    t = os.environ.get("GITHUB_TOKEN")
    if t:
        return t
    try:
        return subprocess.run(["gh", "auth", "token"], capture_output=True, text=True).stdout.strip() or None
    except OSError:
        return None


def get(url, tok, accept="application/vnd.github+json"):
    req = urllib.request.Request(url, headers={"Accept": accept, **({"Authorization": "token " + tok} if tok else {})})
    return urllib.request.urlopen(req)


def main():
    tok = token()
    os.makedirs(OUT, exist_ok=True)
    for tag, want in GET:
        get_release(tag, want, tok)
    print("clips in", OUT)


def get_release(tag, want, tok):
    rel = json.load(get("https://api.github.com/repos/%s/releases/tags/%s" % (REPO, tag), tok))
    for a in rel["assets"]:
        if a["name"] not in want:
            continue
        path = os.path.join(OUT, want[a["name"]])
        if os.path.exists(path) and os.path.getsize(path) == a["size"]:
            print("have", a["name"])
            continue
        print("getting", a["name"], "(%.0f MB)" % (a["size"] / 1e6))
        with get(a["url"], tok, "application/octet-stream") as r, open(path, "wb") as f:
            while True:
                b = r.read(1 << 20)
                if not b:
                    break
                f.write(b)


if __name__ == "__main__":
    sys.exit(main())
