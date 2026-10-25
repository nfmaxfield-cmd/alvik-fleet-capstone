"""The live view's operations console (2026-10-25, Nick's approved mockup and his three asks): the state JSON carries
the sensor trust table, the processing-speed series, the timeline, plan, results, robots (battery, sensors,
distance to the open edge), self-calibration and the open hold; the page has the hold buttons and the plain
explanations; answering through the page still works; the picture still draws. No robots, no camera: a fake
runner with the attributes the view reads (make_runner is also used for the screenshots)."""
import json
import os
import random
import re
import sys
import threading
import time
import types
import urllib.request

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import e02_load as LD  # noqa: E402
import e02_plan as PL  # noqa: E402
import e02_trust as TR  # noqa: E402
import e02_view as V  # noqa: E402


class _Cam:
    """A straight-down camera: 4 px a cm, the board's (0, 0) at pixel (70, 470)."""

    def project(self, p):
        return 70.0 + 4.0 * p[0], 470.0 - 3.6 * p[1] - 1.5 * p[2]

    def px_per_cm(self, x, y, z=0.0):
        return 4.0


class _Tracker:
    def __init__(self, active, x, y):
        self.active, self.pos, self.hint, self.rpx, self.last_cands = active, (time.time(), x, y), None, 24, []

    def where(self):
        return (time.time(), self.pos[1], self.pos[2])


def make_runner(folder, hold=False, question=None):
    """A runner half an hour into a session: three robots, 7 of 24 runs scored, one hold earlier, a slow patch in
    the camera processing, a front sensor that looks bad. hold=True: a border hold is open now."""
    rng = random.Random(4)
    now = time.time()
    os.makedirs(folder, exist_ok=True)
    st = PL.State(folder)
    names = ["Left", "Center", "Right"]
    st["robots"] = names
    sched = PL.make_schedule(names, 2, 7)
    for i, q in enumerate(sched):
        if i < 7:
            q.update(status="done", t_start=now - 1500 + i * 150, t_end=now - 1400 + i * 150,
                     end_err_cm=round(abs(rng.gauss({"U": 6, "C": 3.5, "K": 2.5, "KC": 1.5}[q["cond"]], 1)), 2))
        elif i == 7:
            q.update(status="running", t_start=now - 40)
    st["schedule"] = sched
    st["positions"] = {"Left": [30.0, 25.0, 0.0], "Center": [60.0, 100.0, 90.0], "Right": [100.0, 30.0, 180.0]}
    st["holds"] = [[round(now - 900, 1), "Center", "lost", "the camera lost it for 4 s", "3", "c", 42.0]]
    if hold:
        st["holds"].append([round(now - 75, 1), "Right", "border", "it stopped itself 9 cm from an open edge",
                            "8", None, None])
    st["setup"]["pi"] = dict(overhead=True, side=True, notes=True, fps=24.8)
    for k, n in enumerate(names):
        hist = []
        for j, order in enumerate((1, 4, 7)):
            g = 1.06 + 0.01 * k + 0.02 / (j + 1)
            w = 0.975 + 0.004 * k - 0.006 / (j + 1)
            hist.append(dict(accepted=True, order=order, values=dict(gyro_scale=g, gyro_drift_dps=0.1, wheel_scale=w),
                             fitted={}))
        st["fits"][n] = dict(values=hist[-1]["values"], history=hist)
    tr = TR.SensorTrust()
    for _ in range(6):
        tr.camera_at_find("Left", rng.gauss(0, 0.8), rng.gauss(0, 0.8))
    for n in names:
        for _ in range(4):
            tr.stretch_fit(n, 1.2 + rng.random() * 0.2, 1.3 + rng.random() * 0.2, 8)
    for _ in range(8):
        tr.front_vs_map("Right", rng.choice((-1, 1)) * (20 + 10 * rng.random()))
    tr.front_vs_map("Left", 2.0)
    tr.update_flags(names)
    perf = []
    sw = LD.SpeedWatch()                  # as e02.on_perf feeds it: each row gets the runner's load and slow marks
    for i in range(300):
        t = now - 300 + i
        load = 180 <= i < 240             # a planned load block (an upload, say)
        slow = 200 <= i < 225
        row = dict(t=t, fps_handled=(12.0 + rng.random() * 3) if slow else 23.5 + rng.random() * 1.5, dropped=0,
                   ms_mean=20, ms_max=35, age_mean_s=(0.3 if slow else 0.12) + rng.random() * 0.03,
                   age_max_s=(0.6 if slow else 0.25) + rng.random() * 0.1, wait_s=0, source="pi",
                   phase="route" if i % 150 > 40 else "find")
        sw.feed(row, load_on=load)
        perf.append(dict(row, load=load, slow=bool(sw.slow)))
    with open(os.path.join(folder, "battery.csv"), "w") as f:
        f.write("t,robot,role,state,still_s,battery,phase,source\n")
        for i in range(20):
            for k, n in enumerate(names):
                f.write("%.1f,%s,driving,still,60,%d,route,status\n" % (now - 1800 + i * 90, n, 96 - k * 9 - i))
    img = np.full((480, 640), 70, np.uint8)
    img[30:470, 70:560] = 115
    view = types.SimpleNamespace()
    r = types.SimpleNamespace(
        a=types.SimpleNamespace(home=False, dry_run=True, on_pi=True, degrade=""), dir=folder, st=st, trust=tr,
        perf_rows=perf, speed=sw, load=types.SimpleNamespace(on=False), t_launch=now - 1800, phase="route", order="8", tracker=_Tracker("Right", 100.0, 30.0),
        feed=types.SimpleNamespace(last_img=img), cam=_Cam(), board=PL.Board(xboard_side=1, yboard_side=1),
        listener=types.SimpleNamespace(battery={"Left": (now - 30, 77)}), hold=threading.Event(),
        stop_requested=False, held_out=set(), unavailable=set(), moves={"Right": dict(run="8")},
        leg_live={"Right": now - 2} if not hold else {}, leg_kind={"Right": "go"}, exploring=False, lab=None,
        bricks_view=None, view=view)
    r.border_cm = lambda: 15.0
    r.fw_border_cm = lambda: 10.0
    r.border_frame = lambda x, y: (r.board, x, y)
    if hold:
        r.hold.set()
    return r


def test_state_has_trust_table_and_perf_series(tmp_path):
    r = make_runner(str(tmp_path / "e02-20261025-101500-dryrun"))
    v = V.LiveView(r, port=0)
    s = v.status()
    json.dumps(s)                                    # the page gets it as JSON
    L = s["live"]
    assert not L.get("errors"), L.get("errors")
    rows = {q["key"]: q for q in L["trust"]["rows"]}
    assert set(rows) == {"camera/*"} | {"%s/%s" % (s_, n) for n in ("Left", "Center", "Right")
                                        for s_ in ("wheels", "gyro", "front")}
    assert rows["camera/*"]["unit"] == "cm" and rows["wheels/Left"]["unit"] == "cm/stretch"
    assert rows["gyro/Left"]["unit"] == "deg/stretch" and rows["front/Left"]["unit"] == "cm"
    assert rows["front/Right"]["state"] == "bad" and rows["front/Left"]["state"] == "little"
    assert rows["camera/*"]["state"] == "ok" and rows["camera/*"]["n"] >= TR.MIN_N["camera"]
    assert rows["front/Right"]["lag"] == ""
    r.trust.flags["camera/*"] = True                    # flagged earlier, the numbers healthy since: said so
    v._trust_cache = None
    cam = {q["key"]: q for q in v.status()["live"]["trust"]["rows"]}["camera/*"]
    assert cam["state"] == "bad" and "next check" in cam["lag"]
    q = rows["wheels/Left"]
    assert abs(q["k"] - q["sd"] / q["healthy"]) < 0.02 and q["trend"]
    p = L["perf"]
    assert len(p["rows"]) >= 290 and p["slow_s"] >= 15 and p["rows"][-1][0] <= 0
    assert p["slow"] is False                        # the last second was fine
    assert L["header"]["state"] == "running" and L["header"]["session"] == "e02-20261025-101500-dryrun"
    assert L["header"]["rec_fps"] == 24.8 and L["header"]["held_s"] == 42
    assert L["plan"]["done"] == 7 and L["plan"]["total"] == len(r.st["schedule"])
    assert len(L["results"]) == 7
    assert {c["robot"] for c in L["calib"]} == {"Left", "Center", "Right"}
    tl = L["timeline"]
    assert tl["runs"] and tl["holds"] and {q[2] for q in tl["segs"]} <= {"setup", "runs", "finding"}
    rb = {q["name"]: q for q in L["robots"]}
    assert rb["Right"]["cls"] == "follow" and rb["Right"]["edge"]["where"] == "live"
    assert rb["Right"]["edge"]["dist_cm"] == 30.0          # open edges x = 0 and y = 0 (boards at the far sides)
    assert rb["Right"]["edge"]["border_cm"] == 15.0 and rb["Right"]["edge"]["fw_border_cm"] == 10.0
    assert rb["Left"]["battery"] == 77 and len(rb["Left"]["battery_trend"]) == 21
    assert [c["name"] for c in rb["Left"]["sensors"]] == ["Camera", "Wheels", "Gyro", "Front"]
    assert L["hold"] is None


def test_slow_is_the_runners_own(tmp_path):
    """The chart's slow seconds, lines and rule are the runner's speed watch (baseline-relative), and the planned
    load blocks come through."""
    r = make_runner(str(tmp_path / "w"))
    v = V.LiveView(r, port=0)
    p = v.status()["live"]["perf"]
    sw = r.speed
    assert p["baseline"]["fps"] == round(sw.base_fps, 1) and abs(p["slow_fps"] - sw.fps_frac * sw.base_fps) < 0.1
    assert abs(p["slow_age_s"] - sw.age_frac * sw.base_age) < 0.001 and "baseline" in p["rule"]
    assert 15 <= p["slow_s"] <= 25 and p["load_s"] == 60 and p["load"] is False
    assert p["slow"] is False
    sw.slow = True
    p = v.status()["live"]["perf"]
    assert p["slow"] and "pictures a second" in p["why"] and v.status()["live"]["header"]["slow"]


def test_slow_now_and_stalled(tmp_path):
    """Without the runner's watcher (a replay, an old session): this view's own fixed limits."""
    r = make_runner(str(tmp_path / "s"))
    r.speed = None
    for q in r.perf_rows:
        q.pop("slow", None)
    r.perf_rows[-1].update(fps_handled=4.0)
    v = V.LiveView(r, port=0)
    p = v.status()["live"]["perf"]
    assert p["slow"] and "pictures a second" in p["why"] and p["rule"].startswith("under 10")
    r.perf_rows[-1].update(fps_handled=24.0, t=time.time() - 10)
    h = v.status()["live"]["header"]
    assert h["slow"] and "no picture handled" in h["slow_why"]


def test_hold_state_and_page_buttons(tmp_path):
    r = make_runner(str(tmp_path / "h"), hold=True)
    v = V.LiveView(r, port=0)
    L = v.status()["live"]
    assert L["header"]["state"] == "hold"
    assert L["hold"]["robot"] == "Right" and L["hold"]["kind"] == "border" and L["hold"]["since_s"] >= 74
    html = v.page("main")
    for a, words in (("c", "Carry on"), ("a", "Carry on, don't ask"), ("o", "Leave"), ("s", "Stop the session")):
        assert re.search(r'data-a="%s"( class="\w+")?><b>%s' % (a, words), html), a
    # every chart and table: one plain sentence and what is measured / where the data comes from
    for txt in ("Session timeline", "One square per planned run", "How far from its starting spot",
                "How many camera pictures the runner works through each second", "Data: the runner's perf rows",
                "planned load on",
                "How much each sensor is trusted", "Data: the runner's sensor trust", "Self-calibration, settling",
                "The overhead camera's latest picture", "virtual border"):
        assert txt in html, txt
    assert "how to read" not in html.lower()
    assert 'href="/lab"' in html and "fonts.googleapis.com" in html and "system-ui" in html


def test_answer_a_hold_through_the_page(tmp_path):
    """The hold's question goes on the page through the view's question (as e02.web_ask puts it there); the
    page's button posts {id, text: 'o'} to /answer and the runner gets 'o'."""
    r = make_runner(str(tmp_path / "a"), hold=True)
    v = V.LiveView(r, port=18097, bind="127.0.0.1")
    v.start()
    try:
        box = []
        th = threading.Thread(target=lambda: box.append(v.ask("HOLD (virtual border): Right: ... Answer: c, a, o, s",
                                                              "", timeout_s=10)))
        th.start()
        q = None
        for _ in range(50):
            s = json.loads(urllib.request.urlopen("http://127.0.0.1:18097/status.json", timeout=3).read())
            q = s.get("question")
            if q:
                break
            time.sleep(0.1)
        assert q and s["live"]["hold"]["robot"] == "Right"
        req = urllib.request.Request("http://127.0.0.1:18097/answer", data=json.dumps(dict(id=q["id"], text="o")).encode(),
                                     method="POST", headers={"Content-Type": "application/json"})
        assert urllib.request.urlopen(req, timeout=3).read() == b"ok"
        th.join(5)
        assert box == ["o"]
        assert b"Carry on, don't ask" in urllib.request.urlopen("http://127.0.0.1:18097/", timeout=3).read()
        assert b"Live lab" in urllib.request.urlopen("http://127.0.0.1:18097/lab", timeout=3).read()
        lab = json.loads(urllib.request.urlopen("http://127.0.0.1:18097/lab.json", timeout=3).read())
        assert "calibration" in lab
        assert urllib.request.urlopen("http://127.0.0.1:18097/frame.jpg", timeout=3).read()[:2] == b"\xff\xd8"
    finally:
        v.stop()


def test_picture_has_border_band(tmp_path):
    r = make_runner(str(tmp_path / "f"))
    v = V.LiveView(r, port=0)
    img = v.frame_bgr()
    # a floor pixel 5 cm inside the open x edge (x = x0 = 0 with yboard_side 1): tinted amber (more red than blue)
    u, w = r.cam.project((5.0, 60.0, 0.0))
    b, g, rr = (int(c) for c in img[int(w), int(u)])
    assert rr > b + 20
    u, w = r.cam.project((60.0, 60.0, 0.0))             # the middle of the board: untouched grey
    b, g, rr = (int(c) for c in img[int(w), int(u)])
    assert abs(rr - b) < 5
    r.exploring = True                                  # no band while exploring (no fitted board yet)
    img = v.frame_bgr()
    u, w = r.cam.project((5.0, 60.0, 0.0))
    b, g, rr = (int(c) for c in img[int(w), int(u)])
    assert abs(rr - b) < 5


def test_old_runner_shapes_still_work():
    """A runner with none of the new attributes (the webask test's, a replay's): status() still answers and the
    console's parts come back empty instead of failing."""
    class R:
        phase, order, tracker, feed, st, cam, board = "setup", "", None, None, {}, None, None
    v = V.LiveView(R(), port=0)
    s = v.status()
    assert s["phase"] == "setup" and s["question"] is None
    L = s["live"]
    assert L["header"]["state"] == "running" and L["trust"] is None and L["perf"]["rows"] == []
    assert L["robots"] == [] and L["plan"]["runs"] == []
    assert v.frame_jpg()[:2] == b"\xff\xd8"
