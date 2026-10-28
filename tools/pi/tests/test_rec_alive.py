"""The recorder start-up check (2026-10-28): a recorder window with no pictures is restarted, not trusted.

    python3 -m pytest tools/pi/tests/test_rec_alive.py -q
"""
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(PI))
sys.path.insert(0, PI)
sys.path.insert(0, os.path.join(os.path.dirname(PI), "experiment"))

import rec_alive as RA  # noqa: E402


def _jpeg():
    import cv2
    import numpy as np
    return cv2.imencode(".jpg", np.full((48, 64), 120, np.uint8))[1].tobytes()


def _live_server():
    body = _jpeg()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                for n in range(200):
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(body)).encode()
                                     + b"\r\nX-Frame: " + str(n).encode() + b"\r\nX-Time: %.4f\r\n\r\n" % time.time()
                                     + body + b"\r\n")
                    time.sleep(0.04)
            except OSError:
                return
    srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_alive_sees_pictures_and_their_absence():
    srv = _live_server()
    try:
        ok, words = RA.alive(srv.server_address[1], timeout=3.0)
        assert ok, words
    finally:
        srv.shutdown()
    ok, words = RA.alive(_free_port(), timeout=1.0)
    assert not ok and "no pictures" in words


def test_main_names_the_missing_camera(capsys):
    def check(host, port, timeout, n):
        return (port == 8000, "fake", None, 0.0)
    assert RA.main(["--side"], check=check) == 1
    assert "side camera (127.0.0.1:8001)" in capsys.readouterr().out
    assert RA.main([], check=check) == 0


@pytest.mark.skipif(shutil.which("tmux") is None, reason="needs tmux")
def test_launcher_restarts_a_recorder_window_without_pictures(tmp_path):
    """The 16:24 case: tmux session fleet with a window `rec`, nothing on 127.0.0.1:8000. `./run-fleet-pi.sh rec`
    must not say "already running": it says pictures are missing and closes that window to start afresh (here the
    fresh start then stops at "no overhead camera found": this machine has no cameras). A private tmux server
    (TMUX_TMPDIR) keeps the test away from any real session."""
    with socket.socket() as s:
        if s.connect_ex(("127.0.0.1", 8000)) == 0:
            pytest.skip("something serves 127.0.0.1:8000 here")
    env = dict(os.environ, TMUX_TMPDIR=str(tmp_path), FLEET_DRY="0")
    env.pop("TMUX", None)
    subprocess.run(["tmux", "new-session", "-d", "-s", "fleet", "-n", "rec", "sleep 600"], env=env, check=True)
    try:
        r = subprocess.run(["bash", os.path.join(REPO, "run-fleet-pi.sh"), "rec"], env=env, cwd=REPO,
                           capture_output=True, text=True, timeout=60)
        out = r.stdout + r.stderr
        assert "already running" not in out.lower() or "pictures coming" not in out, out
        assert "pictures are missing" in out, out
        assert "Restarting the recorder window" in out, out
        w = subprocess.run(["tmux", "list-windows", "-t", "fleet", "-F", "#W"], env=env, capture_output=True,
                           text=True)
        assert "rec" not in w.stdout.split(), w.stdout
    finally:
        subprocess.run(["tmux", "kill-server"], env=env)
