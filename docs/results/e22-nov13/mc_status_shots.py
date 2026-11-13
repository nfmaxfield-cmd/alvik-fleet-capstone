#!/usr/bin/env python3
# e22 (Claude's screenshots for the report; simulator only): Mission Control's table page with (1) the status line
# while nothing runs and the robots sleep, (2) a free roam started from the terminal shown read-only, waiting for a
# sleeping robot ("asleep, waking"), then (3) with the woken robot driving and every robot's live error shape.
# usage: mc_status_shots.py REPO OUT_DIR. Changelog: e22-2026-11-13a first version.
import json, os, subprocess, sys, tempfile, time, urllib.request
REPO, OUT = sys.argv[1], sys.argv[2]
d = tempfile.mkdtemp()
runs = os.path.join(d, "logs")
os.makedirs(runs)
conf = os.path.join(d, "conf")
os.makedirs(conf)
env = dict(os.environ, HOME=d, MIRLT_CONF_DIR=conf, MIRLT_RUNS_GLOB=os.path.join(runs, "*-freeroam*"),
           MIRLT_HEARD_FILE=os.path.join(conf, "heard_robots.json"))
sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
os.environ.update(MIRLT_CONF_DIR=conf)
import wake_robots as WR   # noqa: E402
WR.note_sleep(None, 2, "the run ended")      # the robots were sent `sleep 2` when the last run ended
mc = subprocess.Popen([sys.executable, os.path.join(REPO, "tools/fleet/mission_control.py"), "--dir", os.path.join(d, "mc"),
                       "--dry", "--port", "8098"], cwd=REPO, env=env, stdout=open(os.path.join(d, "mc.txt"), "w"),
                      stderr=subprocess.STDOUT)
for _ in range(80):
    try:
        urllib.request.urlopen("http://127.0.0.1:8098/api/start", timeout=1); break
    except Exception:
        time.sleep(0.5)
from playwright.sync_api import sync_playwright   # noqa: E402
fr = None
try:
    with sync_playwright() as p:
        b = p.chromium.launch()
        pg = b.new_page(viewport={"width": 1280, "height": 900})
        pg.goto("http://127.0.0.1:8098/")
        pg.wait_for_timeout(4000)
        pg.screenshot(path=os.path.join(OUT, "status-nothing-running.png"))
        # a free roam from the terminal (as ./run-fleet-pi.sh freeroam does), Center asleep: its check-in 75 s away
        fr = subprocess.Popen([sys.executable, os.path.join(REPO, "tools/fleet/freeroam.py"), "run", "--dry", "--hours",
                               "0.05", "--robots", "Left,Center,Right", "--sim-minutes", "30", "--sim-asleep",
                               "Center:1.5:0.85", "--dir", os.path.join(runs, "20261113-090000-freeroam")], cwd=REPO,
                              env=env, stdout=open(os.path.join(d, "fr.txt"), "w"), stderr=subprocess.STDOUT)
        pg.wait_for_timeout(45000)
        pg.screenshot(path=os.path.join(OUT, "status-terminal-run-waking.png"))
        for _ in range(60):
            if "compass checked" in open(os.path.join(d, "fr.txt")).read():
                break
            time.sleep(2)
        pg.wait_for_timeout(25000)
        pg.screenshot(path=os.path.join(OUT, "status-terminal-run-live-shapes.png"))
        pg.set_viewport_size({"width": 600, "height": 1200})
        pg.wait_for_timeout(2500)
        pg.screenshot(path=os.path.join(OUT, "status-terminal-run-600px.png"))
        b.close()
finally:
    for q in (fr, mc):
        if q is not None:
            q.terminate()
            try:
                q.wait(10)
            except Exception:
                q.kill()
print(open(os.path.join(d, "fr.txt")).read()[-600:])
