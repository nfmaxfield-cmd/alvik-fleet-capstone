#!/usr/bin/env python3
"""(e24, simulator only) Stop during start-up, with Mission Control started from a shell that IGNORES Ctrl+C (the
24 Nov audit's B1). Mission Control must already be running in practice mode at BASE (started with SIGINT ignored:
docs/results/e24-school-usability/README.md says how).

    stop_case.py BASE WAIT_S [ORDER]      e.g. stop_case.py http://127.0.0.1:8295 25

Starts a practice Visit points run, presses STOP after WAIT_S, and times: until the run program has ended, and how far
the simulated robots' true positions moved after the STOP (truth.csv). Prints one JSON line.
Changelog
  2026-11-24  first version (e24), after the audit's scripts/stop_startup.py.
"""
import csv, glob, json, os, sys, time, urllib.request
VERSION = "e24-stop-case-2026-11-24"
BASE, WAIT = sys.argv[1].rstrip("/"), float(sys.argv[2])
ORDER = sys.argv[3] if len(sys.argv) > 3 else "Alpha:P01,P02;Bravo:P12,P14"


def api(path, body=None):
    req = urllib.request.Request(BASE + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=20) as r:
        return json.load(r)


api("/api/start/run", {"kind": "visit", "attended": False, "clear_space": True, "order": ORDER})
t0 = time.time()
time.sleep(WAIT)
st = api("/api/start")["status"]
folder = st.get("folder")
phase = st.get("phase")
api("/api/start/stop", {})
ts = time.time()
ended = None
confirmed = None
while time.time() - ts < 120:
    s = api("/api/start")["status"]
    if confirmed is None and s.get("stop") and s["stop"].get("phase") == "stopped":
        confirmed = time.time() - ts
    if not s.get("running"):
        ended = time.time() - ts
        break
    time.sleep(0.5)
time.sleep(3)
moved = {}
try:
    tp = glob.glob(os.path.join(folder, "run", "sim", "truth.csv"))[0]
    rows = list(csv.DictReader(open(tp)))
    tk = "t" if "t" in rows[0] else list(rows[0])[0]
    by = {}
    for r in rows:
        by.setdefault(r.get("robot") or r.get("name"), []).append(r)
    for n, rr in by.items():
        after = [r for r in rr if float(r[tk]) >= ts]
        if after:
            x0, y0 = float(after[0]["x_cm"]), float(after[0]["y_cm"])
            moved[n] = round(max(((float(r["x_cm"]) - x0) ** 2 + (float(r["y_cm"]) - y0) ** 2) ** 0.5 for r in after), 1)
except Exception as e:  # noqa: BLE001
    moved = {"error": str(e)}
print(json.dumps({"version": VERSION, "stop_after_s": WAIT, "phase_at_stop": phase, "run_ended_after_stop_s": None if ended is None else round(ended, 1),
                  "page_said_stopped_after_s": None if confirmed is None else round(confirmed, 1),
                  "moved_after_stop_cm": moved, "folder": folder}))
