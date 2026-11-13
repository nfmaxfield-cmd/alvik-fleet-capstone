#!/usr/bin/env python3
# e22 (Claude's tracing for the report; simulator and logs only): Mission Control's map page with each robot's live error shape (Playwright screenshots).
# Run from the repo root (the report's Appendix B has the commands). Changelog: e22-2026-11-13a first version.
"""Mission Control's map page with each robot's live error shape: the real mission_control.py (--dry, a temp folder),
its /api/start told a free roam is running and its /api/live fed the shapes captured from a hard-simulator run (what
shape_live.json carries every second). Screenshots at desktop and phone width."""
import json, os, subprocess, sys, tempfile, time, urllib.request
REPO, snaps_f, out_dir, snap_i = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
snaps = json.load(open(snaps_f))
d = tempfile.mkdtemp()
env = dict(os.environ, HOME=d, MIRLT_CONF_DIR=os.path.join(d, "conf"))
mc = subprocess.Popen([sys.executable, os.path.join(REPO, "tools/fleet/mission_control.py"), "--dir", os.path.join(d, "mc"),
                       "--dry", "--port", "8097"], cwd=REPO, env=env, stdout=open(os.path.join(d, "mc.txt"), "w"),
                      stderr=subprocess.STDOUT)
for _ in range(80):
    try:
        urllib.request.urlopen("http://127.0.0.1:8097/api/start", timeout=1); break
    except Exception:
        time.sleep(0.5)
COL = {"Left": "#2952CC", "Center": "#C2410C", "Right": "#7A3EC8"}
from playwright.sync_api import sync_playwright
try:
    with sync_playwright() as p:
        b = p.chromium.launch()
        for w, h, tag in ((1280, 1000, "desktop"), (600, 1200, "600px")):
            pg = b.new_page(viewport={"width": w, "height": h})
            def start(route):
                r = route.fetch(); j = r.json()
                j["status"] = dict(j.get("status") or {}, kind="freeroam", running=True, folder="logs/20261113-freeroam-sim",
                                   lines=[])
                route.fulfill(response=r, json=j)
            def live(route):
                r = route.fetch(); j = r.json()
                sn = snaps[snap_i]
                rob = []
                for q in sn["robots"]:
                    rob.append(dict(name=q["name"], ref="", colour=COL.get(q["name"]), x=q["x"], y=q["y"], th=q["th"],
                                    pm_cm=q["shape_draw"]["extent"], shape=q["shape"], shape_draw=q["shape_draw"],
                                    stage=q["stage"], rescue=q["rescue"], seen=q["seen"],
                                    source="the overhead camera" if q["seen"] else "its own wheels and gyro (simulated)",
                                    since_fix_s=None, battery=q.get("battery"), doing=q.get("doing") or "", held=q["lost"],
                                    out="", needs_hand=False))
                j["robots"] = rob
                j["table"] = {"x_min": 0, "y_min": 0, "x_max": 122, "y_max": 122}
                route.fulfill(response=r, json=j)
            pg.route("**/api/start", start)
            pg.route("**/api/live", live)
            pg.goto("http://127.0.0.1:8097/")
            pg.wait_for_timeout(4000)
            pg.screenshot(path=os.path.join(out_dir, "mc-live-shape-%s.png" % tag), full_page=False)
            el = pg.query_selector("#map")
            if el:
                el.screenshot(path=os.path.join(out_dir, "mc-live-shape-map-%s.png" % tag))
            pg.close()
        b.close()
finally:
    mc.terminate()
    try:
        mc.wait(5)
    except Exception:
        mc.kill()
print("ok", out_dir)
