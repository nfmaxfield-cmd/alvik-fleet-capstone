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
<div id="side"><h1>E02 live view <a href="/lab" style="font-size:13px;color:#8cf">live lab &rarr;</a></h1>
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
                    elif path == "/lab":
                        body, ctype = LAB_PAGE.encode(), "text/html; charset=utf-8"
                    elif path == "/lab.json":
                        body, ctype = json.dumps(view.lab_state()).encode(), "application/json"
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

    def lab_state(self):
        import e02_live as LIVE
        r = self.r
        lab = getattr(r, "lab", None)
        st = r.st
        return dict(filter=lab.state() if lab is not None else dict(robot=None),
                    calibration=LIVE.calibration_series(st), results=LIVE.results_series(st),
                    phase=getattr(r, "phase", ""), t=time.time())

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
            lab = getattr(r, "lab", None)
            if lab is not None:
                fs = lab.state()
                if fs.get("robot") and tr is not None and tr.active == fs["robot"]:
                    p = px(fs["x"], fs["y"], 3.7)
                    if p:
                        k = cam.px_per_cm(fs["x"], fs["y"], 3.7)
                        ax = (max(1, int(fs["sd_major_cm"] * k)), max(1, int(fs["sd_minor_cm"] * k)))
                        cv2.ellipse(v, p, ax, -fs["angle_deg"], 0, 360, (255, 0, 255), 1)
            m = (r.st.get("setup") or {}).get("map") or {}
            for ray in m.get("rays", []):
                s0, pt = ray.get("cam_start"), ray.get("point")
                if s0 and pt:
                    a, b = px(s0[0], s0[1]), px(pt[0], pt[1])
                    if a and b:
                        cv2.line(v, a, b, (0, 0, 255) if ray.get("kind") == "edge" else (255, 160, 0), 1)
        ok, buf = cv2.imencode(".jpg", v, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()


LAB_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Live lab</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0e1116;--card:#171b22;--fg:#e8eaed;--dim:#9aa0a6;--wheels:#f2a33a;--gyro:#5aa9ff;--camera:#3ccf8e;
--grid:#2a2f38;--U:#e06c75;--C:#e5c07b;--K:#61afef;--KC:#98c379}
body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
header{padding:14px 16px 0}h1{font-size:20px;margin:0}header p{color:var(--dim);margin:4px 0 0}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px;padding:16px}
.card{background:var(--card);border-radius:10px;padding:14px}
.card h2{font-size:15px;margin:0 0 4px}.card .note{color:var(--dim);font-size:13px;margin:6px 0 0}
img{width:100%;border-radius:6px}
.bar{display:flex;align-items:center;gap:8px;margin:8px 0}.bar span{width:64px;color:var(--dim)}
.track{flex:1;height:14px;background:var(--grid);border-radius:7px;overflow:hidden}
.fill{height:100%;border-radius:7px;transition:width .4s}
.big{font-size:18px;margin:8px 0}
svg text{fill:var(--dim);font-size:11px}
</style></head><body>
<header><h1>Live lab</h1><p>What the system is learning while the robots run. <a href="/" style="color:#8cf">camera view</a></p></header>
<main>
<div class="card"><h2>The camera, with the filter's view</h2><img id="pic" src="/frame.jpg" alt="overhead camera">
<p class="note">Magenta: where the filter thinks the moving robot is, and how unsure it is (the ellipse; smaller is surer).</p></div>
<div class="card"><h2>How much the filter trusts each sensor now</h2>
<div id="robot" class="note"></div>
<p style="margin:10px 0 2px">Where it is</p>
<div class="track" style="height:22px;display:flex"><div id="p_motion" class="fill" style="background:var(--wheels);border-radius:0;width:50%"></div><div id="p_camera" class="fill" style="background:var(--camera);border-radius:0;width:50%"></div></div>
<div class="note"><span style="color:var(--wheels)">&#9632;</span> wheels and gyro <b id="v_motion">-</b> &nbsp; <span style="color:var(--camera)">&#9632;</span> camera <b id="v_camera">-</b></div>
<p style="margin:12px 0 2px">Which way it turns</p>
<div class="track" style="height:22px;display:flex"><div id="t_gyro" class="fill" style="background:var(--gyro);border-radius:0;width:50%"></div><div id="t_wheels" class="fill" style="background:var(--wheels);border-radius:0;width:50%"></div></div>
<div class="note"><span style="color:var(--gyro)">&#9632;</span> gyro <b id="v_gyro">-</b> &nbsp; <span style="color:var(--wheels)">&#9632;</span> wheels <b id="v_wheels">-</b></div>
<div id="words" class="big"></div>
<p class="note">Where it is: how much of any error the camera corrects within a second, against what the wheels and
gyro carry forward. Which way it turns: the gyro doesn't slip, the wheels do, more the faster it turns.
How unsure it is now: <b id="sigma">-</b>.</p></div>
<div class="card"><h2>Self-calibration, settling run by run</h2><svg id="cal" viewBox="0 0 340 210" width="100%"></svg>
<p class="note">Each line is one robot's gyro scale as the system learned it from the camera: it starts at a guess and settles.</p></div>
<div class="card"><h2>Results so far: how far from its start each robot ended</h2><svg id="res" viewBox="0 0 340 210" width="100%"></svg>
<p class="note">One dot per run, by condition; the bar is the average. Lower is better.</p></div>
</main>
<script>
const COL={U:'var(--U)',C:'var(--C)',K:'var(--K)',KC:'var(--KC)'};
const RCOL=['#f2a33a','#5aa9ff','#3ccf8e','#c678dd','#e06c75'];
let busy=false;
function pic(){if(busy)return;busy=true;const im=new Image();im.onload=()=>{document.getElementById('pic').src=im.src;busy=false};im.onerror=()=>{busy=false};im.src='/frame.jpg?t='+Date.now();}
function svgEl(t,a){const e=document.createElementNS('http://www.w3.org/2000/svg',t);for(const k in a)e.setAttribute(k,a[k]);return e}
function axes(svg,ylab,lo,hi){svg.innerHTML='';for(let i=0;i<=4;i++){const y=20+i*40;svg.appendChild(svgEl('line',{x1:40,x2:330,y1:y,y2:y,stroke:'#2a2f38'}));
 const t=svgEl('text',{x:4,y:y+4});const val=hi-(hi-lo)*i/4;t.textContent=val.toFixed(ylab=='gyro'?3:(Number.isInteger(val)?0:1));svg.appendChild(t)}}
function cal(rows){const svg=document.getElementById('cal');const v=rows.filter(r=>r.gyro_scale!=null);
 let lo=1.0,hi=1.12;v.forEach(r=>{lo=Math.min(lo,r.gyro_scale-0.01);hi=Math.max(hi,r.gyro_scale+0.01)});axes(svg,'gyro',lo,hi);
 const robots=[...new Set(v.map(r=>r.robot))];const n=Math.max(2,...v.map(r=>r.order||0));
 robots.forEach((rb,i)=>{const pts=v.filter(r=>r.robot==rb).map(r=>[40+290*(r.order||0)/n,20+160*(hi-r.gyro_scale)/(hi-lo)]);
  if(pts.length)svg.appendChild(svgEl('polyline',{points:pts.map(p=>p.join(',')).join(' '),fill:'none',stroke:RCOL[i%5],'stroke-width':2}));
  pts.forEach(p=>svg.appendChild(svgEl('circle',{cx:p[0],cy:p[1],r:3,fill:RCOL[i%5]})));
  const t=svgEl('text',{x:44+i*70,y:205});t.textContent=rb;t.setAttribute('style','fill:'+RCOL[i%5]);svg.appendChild(t)});}
function res(rows){const svg=document.getElementById('res');let hi=8;rows.forEach(r=>hi=Math.max(hi,Math.ceil(r.end_err_cm/8)*8));axes(svg,'cm',0,hi);
 ['U','C','K','KC'].forEach((c,i)=>{const x=80+i*70;const v=rows.filter(r=>r.cond==c);
  v.forEach((r,j)=>svg.appendChild(svgEl('circle',{cx:x-12+(j%5)*6,cy:20+160*(1-r.end_err_cm/hi),r:3,fill:COL[c],opacity:.8})));
  if(v.length){const m=v.reduce((a,r)=>a+r.end_err_cm,0)/v.length;svg.appendChild(svgEl('line',{x1:x-18,x2:x+18,y1:20+160*(1-m/hi),y2:20+160*(1-m/hi),stroke:COL[c],'stroke-width':3}))}
  const t=svgEl('text',{x:x-8,y:200});t.textContent=c+' ('+v.length+')';svg.appendChild(t)});}
function tick(){pic();fetch('/lab.json').then(r=>r.json()).then(s=>{const f=s.filter||{};
 document.getElementById('robot').textContent=f.robot?('Following '+f.robot+(f.run?' (run '+f.run+')':'')):'No robot moving right now ('+(s.phase||'-')+').';
 const w=f.weights;if(w){const set=(id,v)=>{document.getElementById(id).style.width=(100*v)+'%'};
  set('p_motion',w.position.motion);set('p_camera',w.position.camera);set('t_gyro',w.turning.gyro);set('t_wheels',w.turning.wheels);
  const txt=(id,v)=>{document.getElementById(id).textContent=Math.round(100*v)+'%'};
  txt('v_motion',w.position.motion);txt('v_camera',w.position.camera);txt('v_gyro',w.turning.gyro);txt('v_wheels',w.turning.wheels)}
 document.getElementById('words').textContent=f.words||'';document.getElementById('sigma').textContent=f.sigma_xy_cm!=null?f.sigma_xy_cm+' cm':'-';
 cal(s.calibration||[]);res(s.results||[]);}).catch(()=>{});}
setInterval(tick,700);tick();
</script></body></html>"""
