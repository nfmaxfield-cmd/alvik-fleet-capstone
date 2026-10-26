#!/usr/bin/env python3
"""Stand-in recorder for start-page tests: makes a recording folder with a sparse 'video' (no disk used) and small
files, then waits for Ctrl+C (as camrec.py does in tmux window e02:rec)."""
import json
import os
import sys
import time

out = sys.argv[1]
side = len(sys.argv) > 2 and sys.argv[2] == "1"
size_mb = int(os.environ.get("DRY_VIDEO_MB", "700"))
d = os.path.join(out, time.strftime("%Y%m%d-%H%M%S") + "_E02")
os.makedirs(d, exist_ok=True)
for cam in ["overhead"] + (["side"] if side else []):
    with open(os.path.join(d, cam + ".avi"), "wb") as f:
        f.truncate(size_mb * 1024 * 1024)
with open(os.path.join(d, "meta.json"), "w") as f:
    json.dump(dict(label="E02", dry=True), f)
print("Stand-in recorder: %s (%d MB sparse video each). Ctrl+C stops it." % (d, size_mb), flush=True)
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Recorder: saved.", flush=True)
