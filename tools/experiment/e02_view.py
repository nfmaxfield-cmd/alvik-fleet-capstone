"""A live view of the E02 runner in a browser (http://localhost:8090 on the laptop): the overhead picture with
what the runner thinks, so the first minutes of a session are easy to watch (2026-10-22; the first screen of the
Live lab, docs/build-plans/live-lab-outline.md).

Drawn on the picture: the board outline; every robot the runner knows of (name, where it is parked or was found);
the robot being followed (green ring), and the hold circle while it should stand still; everything that differs
from the floor memory (yellow: robot-shaped, grey: not, e.g. a hand); the robot-map rays when mapping. Beside
it: the phase, the run, and the runner's last lines (camera / wheels checks, help calls). Read-only: the page can't
drive anything.

    started by e02.py (--view-port, default 8090; 0 = off); open http://localhost:8090 on the laptop
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>E02 live view</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#111;--fg:#eee;--dim:#999;--ok:#5c5;--warn:#e95}
body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
main{display:flex;flex-wrap:wrap;gap:16px;padding:16px}
img{max-width:100%;width:800px;border:1px solid #333}
#side{flex:1;min-width:260px}
h1{font-size:18px;margin:0 0 8px}
.k{color:var(--dim)} pre{white-space:pre-wrap;font-size:13px;line-height:1.35}
</style></head><body><main>
<img id="pic" src="/frame.jpg" alt="overhead camera with the runner's view">
<div id="side"><h1>E02 live view</h1>
<div><span class="k">Phase:</span> <span id="phase">-</span></div>
<div><span class="k">Following:</span> <span id="active">-</span></div>
<div><span class="k">Run:</span> <span id="order">-</span></div>
<p class="k">Green ring: the robot being followed (dashed circle: it should stand still there). Yellow dots: things
that look like a robot. Grey dots: other changes (a hand, a shadow). Labels: robots the runner knows of.</p>
<pre id="log"></pre></div></main>
<script>
let busy=false;
function tick(){
  if(!busy){busy=true;const im=new Image();im.onload=()=>{document.getElementById('pic').src=im.src;busy=false};
    im.onerror=()=>{busy=false};im.src='/frame.jpg?t='+Date.now();}
  fetch('/status.json').then(r=>r.json()).then(s=>{
    for(const k of ['phase','active','order'])document.getElementById(k).textContent=s[k]||'-';
    document.getElementById('log').textContent=(s.log||[]).join('\\n');}).catch(()=>{});
}
setInterval(tick,400);
</script></body></html>"""


class LiveView:
    """Serves the page. runner: an e02.Runner (read only: feed.last_img, tracker, st, cam, board, phase, order)."""

    def __init__(self, runner, port=8090):
        self.r = runner
        self.port = port
        self.lines = []
        self.lock = threading.Lock()
        self.srv = None

    def log(self, line):
        with self.lock:
            self.lines.append(line)
            del self.lines[:-14]

    def start(self):
        view = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                path = self.path.split("?")[0]
                try:
                    if path == "/":
                        body, ctype = PAGE.encode(), "text/html; charset=utf-8"
                    elif path == "/frame.jpg":
                        body, ctype = view.frame_jpg(), "image/jpeg"
                    elif path == "/status.json":
                        body, ctype = json.dumps(view.status()).encode(), "application/json"
                    else:
                        self.send_error(404)
                        return
                except Exception as e:          # never break the session over a picture
                    body, ctype = ("view error: %s" % e).encode(), "text/plain"
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        try:
            self.srv = ThreadingHTTPServer(("127.0.0.1", self.port), H)
        except OSError as e:
            return "live view not started (port %d: %s)" % (self.port, e)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        return "live view: open http://localhost:%d in a browser" % self.port

    def stop(self):
        if self.srv:
            self.srv.shutdown()

    def status(self):
        r = self.r
        tr = r.tracker
        with self.lock:
            log = list(self.lines)
        return dict(phase=getattr(r, "phase", ""), order=getattr(r, "order", ""),
                    active=(tr.active if tr is not None and tr.active else ""), log=log, t=time.time())

    def frame_jpg(self):
        r = self.r
        img = r.feed.last_img if r.feed is not None else None
        if img is None:
            img = np.zeros((480, 640), np.uint8)
        v = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
        cam, tr, bd = r.cam, r.tracker, r.board
        if cam is not None:
            def px(x, y, z=0.0):
                u, w = cam.project((x, y, z))
                return (int(round(u)), int(round(w))) if np.isfinite(u) and np.isfinite(w) else None
            if bd is not None:
                pts = [px(bd.x0, bd.y0), px(bd.x1, bd.y0), px(bd.x1, bd.y1), px(bd.x0, bd.y1)]
                if all(pts):
                    cv2.polylines(v, [np.array(pts, np.int32)], True, (0, 160, 0), 1)
            for c in list(getattr(tr, "last_cands", []) or []):
                cv2.circle(v, (int(c["u"]), int(c["v"])), 4, (0, 220, 220) if c["shape"] else (150, 150, 150), -1)
            for n, q in list((r.st.get("positions") or {}).items()):
                if q:
                    p = px(q[0], q[1], 3.7)
                    if p:
                        cv2.putText(v, n, (p[0] + 12, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
            if tr is not None:
                pos, hint = tr.where(), tr.hint
                if pos is not None:
                    p = px(pos[1], pos[2], 3.7)
                    if p:
                        cv2.circle(v, p, int(tr.rpx * 0.75), (0, 255, 0), 2)
                if hint is not None:
                    p = px(hint[0], hint[1], 3.7)
                    if p:
                        rr = int(hint[2] * cam.px_per_cm(hint[0], hint[1], 3.7))
                        for a in range(0, 360, 30):
                            cv2.ellipse(v, p, (rr, rr), 0, a, a + 15, (0, 255, 0), 1)
            m = (r.st.get("setup") or {}).get("map") or {}
            for ray in m.get("rays", []):
                s0, pt = ray.get("cam_start"), ray.get("point")
                if s0 and pt:
                    a, b = px(s0[0], s0[1]), px(pt[0], pt[1])
                    if a and b:
                        cv2.line(v, a, b, (0, 0, 255) if ray.get("kind") == "edge" else (255, 160, 0), 1)
        ok, buf = cv2.imencode(".jpg", v, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()
