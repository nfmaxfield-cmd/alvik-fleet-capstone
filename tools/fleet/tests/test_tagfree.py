"""Build tag-free (12 Nov): robots with no roof tag found, named and given a heading from the trained detector (on
demand), the robots' own odometry and gyro, and the tracking layer's naming rules. No robot, no Pi, no camera: fake
detectors, the e17 rig, the fast simulator's detector model and (where the model file is given in
MIRLT_DETECTOR_MODEL, not in git) the real detector on a drawn picture."""
import math
import os
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "detector")):
    if p not in sys.path:
        sys.path.insert(0, p)

import e17_harness as H        # noqa: E402
import tagfree as TF           # noqa: E402
import tracking_layer as TL    # noqa: E402

THREE = {"Left": (31.0, 64.0, 0.0), "Center": (90.0, 30.0, 90.0), "Right": (90.0, 95.0, 180.0)}
HOMO = [[2.0, 0.0, 100.0], [0.0, -2.0, 400.0], [0.0, 0.0, 1.0]]     # floor cm -> picture px (y flipped, as a camera)


def floor_to_px(x, y):
    return 2.0 * x + 100.0, -2.0 * y + 400.0


class FakeModel:
    """Stands in for infer.OnnxDetector: a list of (x_px, y_px, heading_deg_on_screen) per call; sleep_ms per call."""

    def __init__(self, objs, sleep_ms=0.0):
        self.objs, self.sleep_ms, self.n = list(objs), sleep_ms, 0

    def __call__(self, bgr):
        self.n += 1
        if self.sleep_ms:
            time.sleep(self.sleep_ms / 1000.0)
        return [dict(cls="alvik", x=u, y=v, heading_deg=h, score=0.9) for u, v, h in self.objs]


def picture(t=0.0):
    import numpy as np
    return lambda: (t, np.zeros((480, 640), np.uint8))


# -- the switch and the model file: decided automatically, a missing model is said, never an error -------------------
def test_the_switch_is_auto_by_default_and_a_missing_model_leaves_the_layer_as_it_was(monkeypatch, tmp_path):
    for e in TF.MODEL_ENVS + (TF.SWITCH_ENV,):
        monkeypatch.delenv(e, raising=False)
    monkeypatch.setenv("HOME", str(tmp_path))
    assert TF.switch() == "auto"
    assert TF.find_model() is None
    L = TL.TrackingLayer(["A"], ("motion",))
    said = []
    assert TF.attach(L, None, say=said.append) is None
    assert L.ml_on is False and "feed" not in L.__dict__ and "no detector model" in said[0]
    m = tmp_path / "d.onnx"
    m.write_bytes(b"x")
    monkeypatch.setenv("MIRLT_DETECTOR_MODEL", str(m))
    assert TF.find_model() == str(m)
    monkeypatch.setenv(TF.SWITCH_ENV, "0")
    assert TF.attach(L, lambda *a: [], say=said.append) is None and L.ml_on is False


def test_the_runner_hook_degrades_without_a_model_and_passes_the_heading_sigma(monkeypatch, tmp_path):
    for e in TF.MODEL_ENVS:
        monkeypatch.delenv(e, raising=False)
    monkeypatch.setenv("HOME", str(tmp_path))

    class R:
        frames = None
        a = None
    assert TF.for_runner(R(), say=lambda s: None) == (None, None)
    src = open(os.path.join(FLEET, "freeroam.py")).read()
    assert "TF.attach(self.layer" in src and 'sigma_th=f.get("sigma_th", 3.0)' in src
    assert "tagfree=self.tagfree.summary()" in src


# -- the detector source: cameras, floor positions, headings, side camera position only, the load limit --------------
def test_overhead_detections_go_to_the_floor_with_a_heading_and_never_a_name():
    u, v = floor_to_px(50.0, 60.0)
    cam = TF.Camera("overhead", picture(1.0), TF.homography_to_floor(HOMO), True, TF.SIGMA_OVERHEAD_CM)
    src = TF.DetectorSource(FakeModel([(u, v, 90.0)]), [cam], sync=True, clock=lambda: 1.0)
    got = src("A", 1.0, "unaccounted", (50.0, 60.0, 20.0))
    assert len(got) == 1 and got[0].name is None and got[0].src == "ml"
    assert abs(got[0].x - 50.0) < 1e-6 and abs(got[0].y - 60.0) < 1e-6
    assert abs(TF.wrap180(got[0].th - 90.0)) < 1e-6        # screen-up is floor +y here (the picture's y is flipped)
    assert src("B", 1.0, "unaccounted", (50.0, 60.0, 20.0)) == []      # the same picture is not handed in twice


def test_the_side_camera_is_used_only_where_the_overhead_found_nothing_and_never_for_heading():
    to = TF.homography_to_floor(HOMO)
    over = TF.Camera("overhead", picture(1.0), to, True, TF.SIGMA_OVERHEAD_CM)
    side_m = FakeModel([floor_to_px(20.0, 20.0) + (45.0,)])
    side = TF.Camera("side", picture(1.0), to, False, TF.SIGMA_SIDE_CM)
    om = FakeModel([floor_to_px(100.0, 100.0) + (0.0,)])

    class Both:
        def __init__(self):
            self.k = 0

        def __call__(self, bgr):
            self.k += 1
            return (om if self.k % 2 else side_m)(bgr)
    src = TF.DetectorSource(Both(), [over, side], sync=True, clock=lambda: 1.0)
    got = src("A", 1.0, "unaccounted", (20.0, 20.0, 10.0))        # A is near (20, 20): the overhead has nothing there
    assert sorted(round(d.x) for d in got) == [20, 100]
    s = [d for d in got if round(d.x) == 20][0]
    assert s.th is None and s.sigma == TF.SIGMA_SIDE_CM and src.counts["side_used"] == 1
    src2 = TF.DetectorSource(FakeModel([floor_to_px(20.0, 20.0) + (0.0,)]), [over, side], sync=True,
                             clock=lambda: 1.0)
    src2("A", 1.0, "unaccounted", (20.0, 20.0, 10.0))
    assert src2.counts["pictures"] == 1                          # the overhead had it: the side camera never decoded


def test_the_load_limit_keeps_the_detector_to_its_share_and_backs_off_a_slow_call():
    b = TF.Budget(duty=0.25, budget_ms=80.0)
    t, n = 0.0, 0
    while t < 20.0:
        if b.allow(t):
            b.done(t, 30.0)
            n += 1
        t += 0.05
    assert n / 20.0 <= 0.25 / 0.030 + 0.5                       # about 8 calls a second at 30 ms and a 25 % share
    b2 = TF.Budget(duty=1.0, budget_ms=80.0)
    b2.done(0.0, 200.0)
    assert not b2.allow(0.5) and b2.allow(0.0 + TF.COOLDOWN_K * 0.2 + 0.01)
    to = TF.homography_to_floor(HOMO)
    cams = [TF.Camera("overhead", picture(1.0), to, True, 1.6), TF.Camera("side", picture(1.0), to, False, 2.6)]
    said = []
    src = TF.DetectorSource(FakeModel([], sleep_ms=5.0), cams, sync=True, budget=TF.Budget(duty=1.0, budget_ms=1.0),
                            say=said.append, clock=lambda: 100.0)
    for k in range(TF.OVER_N):
        src.budget.cool_until = -1e18
        src.cache.clear()
        src("A", 100.0 + k, "unaccounted", (0.0, 0.0, 5.0))
    assert src.side_off and said and "side camera dropped" in said[0]


def test_the_worker_thread_never_makes_the_layer_wait_and_hands_each_result_in_once():
    u, v = floor_to_px(30.0, 30.0)
    cam = TF.Camera("overhead", lambda: (time.time(), __import__("numpy").zeros((10, 10), "uint8")),
                    TF.homography_to_floor(HOMO), True, 1.6)
    src = TF.DetectorSource(FakeModel([(u, v, 0.0)], sleep_ms=30.0), [cam], sync=False,
                            budget=TF.Budget(duty=1.0, budget_ms=1e9))
    t0 = time.time()
    assert src("A", 0.0, "unaccounted", None) == []              # asked: the worker starts, nothing yet
    assert time.time() - t0 < 0.02
    got = []
    end = time.time() + 2.0
    while not got and time.time() < end:
        time.sleep(0.01)
        got = src("A", 0.1, "unaccounted", None)
    assert len(got) == 1 and abs(got[0].x - 30.0) < 1e-6
    assert src("A", 0.2, "unaccounted", None) == []                # handed in once; the next result is still running


# -- the heading: the overhead detector fused with the robot's own --------------------------------------------------
def test_a_heading_that_agrees_is_kept_and_a_wrong_own_heading_is_caught_within_two_pictures():
    import random
    rng = random.Random(3)
    F = TF.HeadingFuser()
    for k in range(30):
        F.update("A", k * 0.5, 10.0, 10.0 + rng.gauss(0, 12.0))
    assert not F.significant("A")                                 # agreeing within its noise: the gyro is kept
    for bias in (30.0, 71.0):
        F2 = TF.HeadingFuser()
        caught = None
        for k in range(10):
            F2.update("B", k * 0.5, 40.0 + bias, 40.0 + rng.gauss(0, 12.0))
            if F2.significant("B") and caught is None:
                caught = k + 1
        assert caught is not None and caught <= 3, (bias, caught)
        assert abs(TF.wrap180(F2.heading("B", 40.0 + bias)[0] - 40.0)) < 15.0


def test_a_detector_that_keeps_disagreeing_raises_a_doubt_then_is_distrusted_for_that_robot():
    F = TF.HeadingFuser()
    for k in range(8):
        F.update("A", k * 0.5, 0.0, 0.0)
    v = [F.update("A", 5.0 + k * 0.5, 0.0, 120.0)[2] for k in range(3)]
    assert "doubt" in v and F.counts["doubts"] == 1 and F.heading("A", 0.0) == (None, None)
    F.reset("A")                                                  # the heading check settled it
    for k in range(4):
        F.update("A", 10.0 + k * 0.5, 0.0, 0.0)
    for k in range(3):
        F.update("A", 20.0 + k * 0.5, 0.0, 120.0)
    assert "A" in F.distrusted and F.update("A", 30.0, 0.0, 0.0)[2] == "distrusted"


def test_no_heading_is_compared_while_the_robot_turns():
    L = TL.TrackingLayer(["A"], ("motion",), board=H.BOARD, camera_watch=False)
    st = TF.TagFree(L, None, TF.HeadingFuser(), None, "1", None, "")
    L.own("A", 0.0, 50.0, 50.0, 0.0, False)
    L.own("A", 0.1, 50.0, 50.0, 9.0, False)                       # 90 deg/s
    f = dict(robot="A", t=0.1, x=50.0, y=50.0, th=40.0, sigma=1.6, src="ml")
    st.after(0.1, [f], [TL.Det("ml", 0.1, 50.0, 50.0, 40.0)])
    assert f["th"] is None and st.fuser.counts.get("skipped_turning") == 1


# -- naming: the layer's own rules with the detector's sightings ---------------------------------------------------
def _layer_with(fn, names=("A", "B")):
    L = TL.TrackingLayer(list(names), ("motion",), board=H.BOARD, camera_watch=False)
    st = TF.attach(L, fn, mode="1")
    return L, st


def test_a_standing_robot_the_motion_detector_cannot_see_is_found_and_named_by_elimination():
    calls = []

    def fn(name, t, why, region):                 # the detector sees both robots where they really are
        calls.append(name)
        return [TL.Det("ml", t, 30.0, 30.0, 0.0), TL.Det("ml", t, 95.0, 70.0, 90.0)]
    L, st = _layer_with(fn)
    L.seed("A", 0.0, 30.0, 30.0)
    L.seed("B", 0.0, 60.0, 90.0)
    named = {}
    for k in range(1, 60):
        t = k * 0.1
        L.own("A", t, 30.0, 30.0, 0.0, True)
        L.own("B", t, 60.0, 90.0, 90.0, True)                     # B's own pose 35 cm off; it stands; motion sees nothing
        L.set_unaccounted("B", "lost")
        for f in L.feed(t, [TL.Det("motion", t, 30.0, 30.0)]):
            named.setdefault(f["robot"], []).append(f)
    assert "B" in calls and "A" not in calls                      # only the robot that needs it
    b = named.get("B") or []
    assert b and all(math.hypot(f["x"] - 95.0, f["y"] - 70.0) < 3.0 for f in b)
    assert any(f.get("how") == "elimination" for f in b)
    off = TL.TrackingLayer(["A", "B"], ("motion",), board=H.BOARD, camera_watch=False)       # no detector: never found
    off.seed("A", 0.0, 30.0, 30.0)
    off.seed("B", 0.0, 60.0, 90.0)
    got = []
    for k in range(1, 60):
        t = k * 0.1
        off.own("A", t, 30.0, 30.0, 0.0, True)
        off.own("B", t, 60.0, 90.0, 90.0, True)
        got += [f for f in off.feed(t, [TL.Det("motion", t, 30.0, 30.0)]) if f["robot"] == "B"]
    assert got == []


def test_auto_mode_leaves_a_robot_a_roof_tag_names_to_the_tag():
    asked = []

    def fn(name, t, why, region):
        asked.append(name)
        return []
    src = TF.SimDetector()
    src.sim = None
    L, st = _layer_with(None)                                     # nothing attached: None
    assert st is None
    L = TL.TrackingLayer(["A"], ("tags", "motion"), board=H.BOARD, camera_watch=False)
    st = TF.attach(L, src, mode="auto")
    L.seed("A", 0.0, 30.0, 30.0)
    L.own("A", 0.1, 30.0, 30.0, 0.0, True)
    L.feed(0.1, [TL.Det("tags", 0.1, 30.0, 30.0, 0.0, name="A")])
    assert src.tag_seen.get("A") == 0.1
    L.want_detector("A", "heading check")
    L.own("A", 0.2, 30.0, 30.0, 0.0, True)
    L.feed(0.2, [])
    assert src.counts["skipped_tag"] >= 1


def test_the_rig_corrects_a_tagless_standing_robot_30_cm_off_from_the_detector_alone():
    """The e17 rig with every lights sighting switched off: only the detector (on demand, the true spots + noise)."""
    import random
    rng = random.Random(5)
    R = H.Rig(THREE, own_off={"Left": (18.0, 24.0, 0.0)})
    R.layer.sources = ("motion",)                                 # the rig's lights never reach the layer

    def det(name, t, why, region):
        return [TL.Det("ml", t, x + rng.gauss(0, 1.0), y + rng.gauss(0, 1.0), th + rng.gauss(0, 10.0))
                for x, y, th in (R.truth[n] for n in R.names)]
    st = TF.attach(R.layer, det, ctrl=R.c, mode="1")
    R.run(12.0)
    assert R.first("CORRECTED", "Left") is not None
    x, y, _ = R.own("Left")
    assert math.hypot(x - 31.0, y - 64.0) < 5.0
    assert R.layer.counts["ml_calls"] > 0 and st.summary()["fuser"]["counts"]["doubts"] == 0


def test_a_heading_in_doubt_asks_the_heading_check_and_the_detector_until_it_ends():
    class Rob:
        def __init__(self):
            self.mode, self.hd_confirm = "idle", None

    class Ctrl:
        t = 0.0

        def __init__(self):
            self.R = {"A": Rob()}
            self.ev = []

        def event(self, r, kind, words):
            self.ev.append(kind)
    c = Ctrl()
    L = TL.TrackingLayer(["A"], ("motion",), board=H.BOARD, camera_watch=False)
    st = TF.attach(L, TF.SimDetector(), ctrl=c, mode="1")
    for k in range(8):
        st.fuser.update("A", k * 0.5, 0.0, 0.0)
    L.own("A", 10.0, 50.0, 50.0, 0.0, True)
    L.own("A", 10.1, 50.0, 50.0, 0.0, True)
    for k in range(3):
        f = dict(robot="A", t=10.0 + k, x=50.0, y=50.0, th=None, sigma=1.6, src="ml")
        st.after(10.0 + k, [f], [TL.Det("ml", 10.0 + k, 50.0, 50.0, 130.0)])
    assert c.R["A"].hd_confirm is not None and "heading in doubt" in c.ev
    st.before(13.0)
    assert L.want_ml.get("A", "").startswith("heading in doubt")
    c.R["A"].hd_confirm = None                                    # the check ended (confirmed or set from the camera)
    st.before(14.0)
    assert "A" not in L.want_ml and "A" not in st.fuser.S


# -- the simulator's detector breaks the guesses ---------------------------------------------------------------------
def test_the_simulated_detector_misses_more_half_over_an_edge_and_under_the_lamp_and_can_be_wrong_about_heading():
    class Bot:
        def __init__(self, n, x, y, th):
            self.name, self.x, self.y, self.th, self.dead, self.v, self.state = n, x, y, th, False, 0.0, "idle"

    class Sim:
        board = H.BOARD
        cam_fault = None
        cam_correct = None
        bots = {"Edge": Bot("Edge", 2.0, 60.0, 0.0), "Lamp": Bot("Lamp", 30.0, 30.0, 0.0),
                "Mid": Bot("Mid", 90.0, 90.0, 0.0)}
    sd = TF.SimDetector({"lamp": [[0, 0, 60, 60]], "ml_heading_bias": {"Mid": 90.0}, "ml_ms": 1.0}, seed=2,
                        budget=TF.Budget(duty=1.0))
    sd.sim = Sim()
    seen = {"Edge": 0, "Lamp": 0, "Mid": 0}
    hd = []
    for k in range(400):
        for d in sd("Mid", k * 0.5, "test", None):
            n = min(Sim.bots, key=lambda q: math.hypot(Sim.bots[q].x - d.x, Sim.bots[q].y - d.y))
            seen[n] += 1
            if n == "Mid":
                hd.append(TF.wrap180(d.th))
    assert seen["Edge"] < 0.7 * 400 and seen["Lamp"] < 0.9 * 400 and seen["Mid"] > 0.9 * 400
    assert abs(sorted(hd)[len(hd) // 2] - 90.0) < 5.0
    sd2 = TF.SimDetector({"side_only": [[60, 60, 130, 130]]}, budget=TF.Budget(duty=1.0))
    sd2.sim = Sim()
    got = sd2("Mid", 0.0, "test", None)
    assert any(d.th is None and math.hypot(d.x - 90, d.y - 90) < 10 for d in got)


def test_a_slow_pi_is_held_to_the_detectors_share_in_the_simulator():
    class Sim:
        board = H.BOARD
        cam_fault = None
        cam_correct = None
        bots = {}
    sd = TF.SimDetector({"ml_ms": 150.0})
    sd.sim = Sim()
    for k in range(200):
        sd("A", k * 0.25, "test", None)
    assert sd.counts["refused_load"] > 0 and sd.budget.n_over == sd.counts["ran"]
    assert sd.counts["ran"] * 0.150 <= 0.25 * 50.0 + 0.6


# -- the real model (not in git: MIRLT_DETECTOR_MODEL) on a drawn picture ----------------------------------------------
@pytest.mark.skipif(not os.environ.get("MIRLT_DETECTOR_MODEL") or
                    not os.path.exists(os.environ.get("MIRLT_DETECTOR_MODEL", "")),
                    reason="the detector model is not in git (release pending); set MIRLT_DETECTOR_MODEL to run this")
def test_the_real_model_loads_runs_on_an_empty_board_and_finds_nothing():
    import numpy as np
    import infer as INF
    m = INF.load(os.environ["MIRLT_DETECTOR_MODEL"], threads=1)
    cam = TF.Camera("overhead", lambda: (1.0, np.full((480, 640, 3), 128, np.uint8)), TF.homography_to_floor(HOMO),
                    True, 1.6)
    src = TF.DetectorSource(m, [cam], sync=True, clock=lambda: 1.0)
    assert src("A", 1.0, "unaccounted", None) == []
    assert src.budget.n == 1 and src.budget.ms_all[0] < 2000.0
