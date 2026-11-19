#!/usr/bin/env python3
"""(e23, simulator only) Mission Control's practice mode at the school site, driven through its page: the warning
about listed points no camera of the run sees, Stop pressed (twice) while the run is still starting, Start again, the
second run to its end; then the page for a crashed run. Screenshots into OUT_DIR, the timeline into ACC_DIR.
    mc_shots.py REPO OUT_DIR ACC_DIR"""
import json, os, subprocess, sys, tempfile, time, urllib.request
REPO, OUT, ACC = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2]), os.path.abspath(sys.argv[3])
PORT = 8098
os.makedirs(OUT, exist_ok=True); os.makedirs(ACC, exist_ok=True)
d = tempfile.mkdtemp(prefix="e23shots-")
conf = os.path.join(d, "conf"); os.makedirs(conf)
env = dict(os.environ, HOME=d, MIRLT_CONF_DIR=conf, MIRLT_HEARD_FILE=os.path.join(conf, "heard_robots.json"))
mc = subprocess.Popen([sys.executable, os.path.join(REPO, "tools/fleet/mission_control.py"), "--dir", os.path.join(d, "mc"),
                       "--dry", "--sim-room", "school", "--port", str(PORT)], cwd=REPO, env=env,
                      stdout=open(os.path.join(ACC, "mission_control.txt"), "w"), stderr=subprocess.STDOUT)
def get(p):
    return json.load(urllib.request.urlopen("http://127.0.0.1:%d%s" % (PORT, p), timeout=10))
def post(p, body):
    rq = urllib.request.Request("http://127.0.0.1:%d%s" % (PORT, p), data=json.dumps(body).encode(),
                                headers={"Content-Type": "application/json"})
    try:
        return json.load(urllib.request.urlopen(rq, timeout=60))
    except urllib.error.HTTPError as e:
        return {"http_error": e.code, "body": e.read().decode("utf-8", "replace")[:600]}
T0, log = time.time(), []
def note(w):
    log.append([round(time.time() - T0, 1), w]); print("%7.1f %s" % (time.time() - T0, w), flush=True)
def status():
    return get("/api/start").get("status") or {}
ORDER = "Alpha:P14,P16,P11;Bravo:P26,P23,P12"
body = {"kind": "visit", "order": ORDER, "keep_unreadable": True, "clear_space": True}
from playwright.sync_api import sync_playwright
try:
    for _ in range(240):
        try:
            get("/api/start"); break
        except Exception:
            time.sleep(0.5)
    with sync_playwright() as p:
        b = p.chromium.launch()
        pg = b.new_page(viewport={"width": 1280, "height": 1100})
        pg.on("dialog", lambda dl: dl.accept())
        pg.goto("http://127.0.0.1:%d/" % PORT)
        pg.wait_for_timeout(4000)
        r = post("/api/start/run", body)
        note("Start pressed: started=%s" % r.get("started", r))
        pg.wait_for_timeout(7000)
        st = status()
        note("7 s in (still starting): running=%s; %s" % (st.get("running"), st.get("words")))
        r = post("/api/start/stop", {}); note("Stop pressed: %s" % r.get("stopped"))
        r = post("/api/start/stop", {}); note("Stop pressed again: %s" % r.get("stopped"))
        pg.reload(); pg.wait_for_timeout(2500)
        for _ in range(120):
            st = status()
            if not st.get("running") and not st.get("stopping"):
                break
            time.sleep(0.5)
        note("after Stop: %s" % st.get("words"))
        first = st.get("folder")
        pg.reload(); pg.wait_for_timeout(2500)
        pg.screenshot(path=os.path.join(OUT, "e23-feature-stop-during-start-up-stopped.png"))
        r = post("/api/start/run", body)
        note("Start pressed again: started=%s %s" % (r.get("started"), r.get("http_error") or ""))
        t1 = time.time(); shot = False
        while time.time() - t1 < 700:
            time.sleep(4.0)
            st = status()
            if not shot and st.get("running") and st.get("visit_warning") and (st.get("points") or time.time() - t1 > 100):
                pg.reload(); pg.wait_for_timeout(3000)
                pg.screenshot(path=os.path.join(OUT, "e23-feature-second-start-runs-with-unseen-points-warning.png"))
                note("second run going: %s; points %s" % (st.get("words"), st.get("points"))); shot = True
            if st.get("kind") and not st.get("running") and not st.get("stopping"):
                break
        note("second run ended: %s" % st.get("words"))
        second = st.get("folder")
        json.dump(st, open(os.path.join(ACC, "status_second_run.json"), "w"), indent=1, default=str)
        pg.reload(); pg.wait_for_timeout(3000)
        pg.screenshot(path=os.path.join(OUT, "e23-feature-second-run-ended.png"))
        # the page for a crash: this finished practice run's summary rewritten with the words of the 18 Nov crash
        ms = os.path.join(second or "", "mission_summary.json")
        if os.path.exists(ms):
            s = json.load(open(ms))
            json.dump(s, open(os.path.join(ACC, "second_run_mission_summary.json"), "w"), indent=1, default=str)
            s["result"] = "crashed: KeyError 'Alpha' (run_plan.py line 1526)"
            json.dump(s, open(ms, "w"))
            pg.reload(); pg.wait_for_timeout(3000)
            note("crash words shown: %s" % status().get("words"))
            pg.screenshot(path=os.path.join(OUT, "e23-feature-crashed-run-is-said-as-crashed.png"))
        json.dump({"first_run": first, "second_run": second, "timeline": log}, open(os.path.join(ACC, "timeline.json"), "w"), indent=1)
        for nm, f in (("first", first), ("second", second)):
            for fn in ("mission_stdout.txt", "run_plan_stdout.txt", "visit_order.json", "stop.json"):
                src = os.path.join(f or "", fn)
                if os.path.exists(src):
                    open(os.path.join(ACC, "%s_%s" % (nm, fn)), "w").write(open(src, errors="replace").read()[-60000:])
        b.close()
finally:
    mc.terminate()
    try:
        mc.wait(15)
    except Exception:
        mc.kill()
print("DIR", d)
