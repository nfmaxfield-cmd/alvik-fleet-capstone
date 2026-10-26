#!/usr/bin/env python3
"""Stand-in for tools/pi/upload_session.py (same file name, so the start page sees it as the upload): the real code with GitHub stubbed, for start-page tests: the same code, progress file and output;
requests are answered locally and each upload takes bytes / DRY_UPLOAD_MBPS seconds (default 40 MB/s)."""
import io
import json
import os
import sys
import time

import importlib.util

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("real_upload_session",
                                               os.path.join(HERE, "..", "..", "upload_session.py"))
U = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(U)

MBPS = float(os.environ.get("DRY_UPLOAD_MBPS", "40"))


class _Resp(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def fake_open(req, timeout=None):
    url = req.full_url
    if req.get_method() == "GET" and "/releases/tags/" in url:
        import urllib.error
        raise urllib.error.HTTPError(url, 404, "not found", {}, None)
    if req.get_method() == "POST" and url.endswith("/releases"):
        return _Resp(json.dumps(dict(upload_url="https://uploads.example/assets{?name}", assets=[])).encode())
    data = req.data
    if hasattr(data, "read"):
        while True:
            b = data.read(4 * 1024 * 1024)
            if not b:
                break
            time.sleep(len(b) / (MBPS * 1e6))
    return _Resp(b"{}")


U._open = fake_open
U.token = lambda: "dry-token"
argv = sys.argv[1:]
if "--session" not in argv:
    import glob
    ds = sorted(glob.glob(os.path.join(U.REPO, "logs", "e02-*")), key=os.path.getmtime)
    if ds:
        argv += ["--session", ds[-1]]
sys.argv = [sys.argv[0]] + argv
sys.exit(U.main())
