#!/usr/bin/env python3
# (2026-11-16, Claude, simulator only) Mission Control's Visit points with a list per robot: plan two lists on the school
# table, start a practice run, follow it to the end; screenshots. usage: vp_shots.py REPO OUT_DIR
import json, os, subprocess, sys, tempfile, time, urllib.request
REPO, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)
d = tempfile.mkdtemp()
conf = os.path.join(d, "conf"); os.makedirs(conf)
env = dict(os.environ, HOME=d, MIRLT_CONF_DIR=conf, MIRLT_HEARD_FILE=os.path.join(conf, "heard_robots.json"))
mc = subprocess.Popen([sys.executable, os.path.join(REPO, "tools/fleet/mission_control.py"), "--dir", os.path.join(d, "mc"),
                       "--dry", "--sim-room", "school", "--port", "8099"], cwd=REPO, env=env,
                      stdout=open(os.path.join(d, "mc.txt"), "w"), stderr=subprocess.STDOUT)
def get(p):
    return json.load(urllib.request.urlopen("http://127.0.0.1:8099" + p, timeout=5))
for _ in range(120):
    try:
        get("/api/start"); break
    except Exception:
        time.sleep(0.5)
from playwright.sync_api import sync_playwright
ALPHA, BRAVO = sys.argv[3].split(","), sys.argv[4].split(",")
try:
    with sync_playwright() as p:
        b = p.chromium.launch()
        pg = b.new_page(viewport={"width": 1280, "height": 1000})
        pg.on("dialog", lambda dl: dl.accept())
        pg.goto("http://127.0.0.1:8099/")
        pg.wait_for_timeout(5000)
        pg.click('[data-k="visit"]'); pg.wait_for_timeout(800)
        pg.screenshot(path=os.path.join(OUT, "shared-list-default.png"))
        pg.click("#vpeach"); pg.wait_for_timeout(500)
        for lab in ALPHA:
            pg.click('.stk[data-p="%s"]' % lab); pg.wait_for_timeout(250)
        pg.click('[data-who="Bravo"]'); pg.wait_for_timeout(300)
        for lab in BRAVO:
            pg.click('.stk[data-p="%s"]' % lab); pg.wait_for_timeout(250)
        pg.wait_for_timeout(800)
        pg.screenshot(path=os.path.join(OUT, "planned.png"))
        pg.click("#start"); t0 = time.time()
        shot_mid = False
        while time.time() - t0 < 900:
            pg.wait_for_timeout(5000)
            st = get("/api/start").get("status") or {}
            if not shot_mid and time.time() - t0 > 90:
                pg.screenshot(path=os.path.join(OUT, "running.png")); shot_mid = True
            if st.get("kind") and not st.get("running") and not st.get("stopping"):
                break
        pg.wait_for_timeout(3000)
        pg.screenshot(path=os.path.join(OUT, "after.png"))
        st = get("/api/start").get("status") or {}
        json.dump(st, open(os.path.join(OUT, "status.json"), "w"), indent=1, default=str)
        b.close()
finally:
    mc.terminate()
    try:
        mc.wait(10)
    except Exception:
        mc.kill()
print("DIR", d)
