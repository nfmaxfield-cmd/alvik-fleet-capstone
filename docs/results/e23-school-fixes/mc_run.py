#!/usr/bin/env python3
"""(e23, simulator only) One practice Visit points run through Mission Control's own page API, as a person would start
it: `mission_control.py --dry --sim-room school`, a list per robot, Start; optionally Stop during the start-up and
Start again. Writes status.json and prints the run folders.
    mc_run.py REPO OUT_DIR "Alpha:P01,P02;Bravo:P21,P22" [--stop-after S] [--port N] [--max-s S]"""
import json, os, subprocess, sys, tempfile, time, urllib.request
REPO, OUT, ORDER = sys.argv[1], sys.argv[2], sys.argv[3]
opt = dict(zip(sys.argv[4::2], sys.argv[5::2]))
PORT = int(opt.get("--port", 8097))
STOP_AFTER = float(opt.get("--stop-after", 0) or 0)
MAX_S = float(opt.get("--max-s", 1500))
os.makedirs(OUT, exist_ok=True)
d = tempfile.mkdtemp(prefix="e23mc-")
conf = os.path.join(d, "conf"); os.makedirs(conf)
env = dict(os.environ, HOME=d, MIRLT_CONF_DIR=conf, MIRLT_HEARD_FILE=os.path.join(conf, "heard_robots.json"))
mc = subprocess.Popen([sys.executable, os.path.join(REPO, "tools/fleet/mission_control.py"), "--dir", os.path.join(d, "mc"),
                       "--dry", "--sim-room", "school", "--port", str(PORT)], cwd=REPO, env=env,
                      stdout=open(os.path.join(OUT, "mission_control.txt"), "w"), stderr=subprocess.STDOUT)
def get(p):
    return json.load(urllib.request.urlopen("http://127.0.0.1:%d%s" % (PORT, p), timeout=10))
def post(p, body):
    rq = urllib.request.Request("http://127.0.0.1:%d%s" % (PORT, p), data=json.dumps(body).encode(),
                                headers={"Content-Type": "application/json"})
    try:
        return json.load(urllib.request.urlopen(rq, timeout=60))
    except urllib.error.HTTPError as e:
        return {"http_error": e.code, "body": e.read().decode("utf-8", "replace")[:600]}
log = []
def note(w):
    log.append([round(time.time() - T0, 1), w]); print("%7.1f %s" % (time.time() - T0, w), flush=True)
T0 = time.time()
try:
    for _ in range(240):
        try:
            get("/api/start"); break
        except Exception:
            time.sleep(0.5)
    time.sleep(4.0)
    body = {"kind": "visit", "order": ORDER, "keep_unreadable": True, "clear_space": True}
    r = post("/api/start/run", body)
    note("Start pressed: %s" % json.dumps(r)[:200])
    if STOP_AFTER:
        time.sleep(STOP_AFTER)
        st = get("/api/start").get("status") or {}
        note("before Stop: running=%s words=%s" % (st.get("running"), st.get("words")))
        r = post("/api/start/stop", {})
        note("Stop pressed: %s" % json.dumps(r.get("stopped")))
        r = post("/api/start/stop", {})
        note("Stop pressed again: %s" % json.dumps(r.get("stopped")))
        first = (get("/api/start").get("status") or {}).get("folder")
        for _ in range(120):
            st = get("/api/start").get("status") or {}
            if not st.get("running") and not st.get("stopping"):
                break
            time.sleep(0.5)
        note("after Stop: %s" % st.get("words"))
        json.dump(st, open(os.path.join(OUT, "status_after_stop.json"), "w"), indent=1, default=str)
        r = post("/api/start/run", body)
        note("Start pressed again: %s" % json.dumps(r)[:200])
    t1 = time.time()
    st = {}
    while time.time() - t1 < MAX_S:
        time.sleep(5.0)
        st = get("/api/start").get("status") or {}
        if st.get("kind") and not st.get("running") and not st.get("stopping"):
            break
    note("ended: %s" % st.get("words"))
    json.dump(st, open(os.path.join(OUT, "status.json"), "w"), indent=1, default=str)
    json.dump(log, open(os.path.join(OUT, "timeline.json"), "w"), indent=1)
finally:
    mc.terminate()
    try:
        mc.wait(15)
    except Exception:
        mc.kill()
print("DIR", d)
