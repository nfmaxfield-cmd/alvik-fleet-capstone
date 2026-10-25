"""The sensor-trust fix (Nick, 2026-10-25 13:03): every robot's sensors judged on their own first (against the camera,
the board touches and their own history), the robots compared only afterwards as a second check; the face readings.
The numbers are the 2026-10-24/25 simulator grid's own evidence (/tmp/claude-0/grid trust files, fit_data)."""
import csv
import os
import random
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import e02_testkit  # noqa: E402,F401  (paths)
import e02_front as FR  # noqa: E402
import e02_trust as TR  # noqa: E402

ROBOTS = ["Left", "Center", "Right"]

# gyro stretch rms (deg) per fit and its weight, from the simulator grid
GYRO_HEALTHY = dict(Left=[(0.09, .8), (0.082, .6), (0.147, .8), (0.35, .6)],
                    Center=[(0.07, .6), (0.075, .8), (0.307, .8), (0.412, .8)],
                    Right=[(0.082, .8), (0.209, .8), (0.126, .6), (0.194, .8)])
GYRO_RIGHT_JITTER = [(0.204, .8), (2.177, .5), (0.861, .8), (0.875, .8)]      # gyro@Right:jitter=0.05
WHEELS_HEALTHY = dict(Left=[(0.172, .8), (0.229, .6), (0.141, .8), (0.278, .6)],
                      Center=[(0.14, .6), (0.184, .8), (0.364, .8), (0.152, .8)],
                      Right=[(0.259, .8), (0.16, .8), (0.111, .6), (0.28, .8)])
# per-run wheel scales
RUNS_HEALTHY = dict(Left=[0.9778, 0.9746, 0.9758, 0.9806, 0.9776, 0.9747, 0.976, 0.9756],
                    Center=[0.9962, 0.999, 0.9928, 0.9976, 0.9905, 0.9969, 0.9933, 0.9972],
                    Right=[0.9643, 0.9687, 0.9664, 0.9648, 0.9637, 0.9681, 0.962, 0.9673])
RUNS_JITTER08 = dict(Right=[0.9736, 0.975, 1.005, 1.0058, 0.9937, 0.9987, 1.0595, 1.0575],
                     Left=[1.0014, 1.0039, 0.8873, 0.8881, 0.9347, 0.9385, 0.9001, 0.8975],
                     Center=[0.8651, 0.8735, 1.0158, 1.0214, 1.0278, 1.0269, 1.0751, 1.0788])
WHEELS_JITTER08 = dict(Left=[(0.278, .8), (0.693, .8), (0.34, .6), (0.641, .6)],
                       Center=[(0.322, .6), (7.911, .9), (0.647, .9), (1.72, .9)],
                       Right=[(0.164, .8), (1.269, .6), (0.221, .9), (3.097, .9)])


def _spread(n, sd):
    """n values evenly spread over a normal of this sd (deterministic: no lucky or unlucky draw)."""
    from statistics import NormalDist
    return [NormalDist(0.0, sd).inv_cdf((i + 0.5) / n) for i in range(n)]


def _camera(t, sd=0.5, n=24, seed=1):
    # finds count for the camera once their robot's wheels are judged: "Cam", a robot judged healthy here
    if len([it for it in t.items if it["robot"] == "Cam" and it.get("session") == t.session]) < 4:
        for _ in range(4):
            t.run_scale("Cam", 0.98, 150.0)
    rng = random.Random(seed)
    for _ in range(n):
        t.camera_at_find("Cam", rng.gauss(0, sd), rng.gauss(0, sd))


def _fits(t, robot, wheels=None, gyro=None):
    for i in range(max(len(wheels or []), len(gyro or []))):
        wv = wheels[i] if wheels and i < len(wheels) else None
        gv = gyro[i] if gyro and i < len(gyro) else None
        n = 10 * (wv or gv)[1]
        t.stretch_fit(robot, wv[0] if wv else None, gv[0] if gv else None, n)


def _healthy_session(t, runs=True):
    _camera(t)
    for r in ROBOTS:
        _fits(t, r, WHEELS_HEALTHY[r], GYRO_HEALTHY[r])
        if runs:
            for s in RUNS_HEALTHY[r]:
                t.run_scale(r, s, 150.0)


def _session(t, robot_data, robots=ROBOTS):
    """The runner's order: per robot and fit, the fit then its two runs (a U block), flags checked after each.
    robot_data: {robot: (wheel fits, gyro fits, run scales)}. Returns the 'degraded' (sensor, robot) seen."""
    out = []
    n = max(max(len(w or []), len(g or []), (len(rn or []) + 1) // 2) for w, g, rn in robot_data.values())
    for i in range(n):
        for r, (w, g, runs) in robot_data.items():
            _fits(t, r, [w[i]] if w and i < len(w) else None, [g[i]] if g and i < len(g) else None)
            out += [(s, rr) for s, rr, what, _ in t.update_flags(robots) if what == "degraded"]
            for sc in (runs or [])[2 * i:2 * i + 2]:
                t.run_scale(r, sc, 150.0)
                out += [(s, rr) for s, rr, what, _ in t.update_flags(robots) if what == "degraded"]
    return sorted(out)


def _flags(t):
    return sorted((s, r) for s, r, what, _ in t.update_flags(ROBOTS) if what == "degraded")


def test_healthy_session_no_flags_cold_and_trained():
    t = TR.SensorTrust()
    _healthy_session(t)
    assert _flags(t) == []
    t.new_session()
    _healthy_session(t)                          # judged against its own history now: still nothing
    assert _flags(t) == []
    j = t.judge("gyro", "Center", ROBOTS)
    assert "own history" in j["checks"][0]["src"]


def test_gyro_jitter_on_one_robot_cold_start_caught_by_the_second_check():
    # failure 1 of the grid: Right 1.1 deg a stretch, not flagged against the Alvik prior (1.0 deg)
    t = TR.SensorTrust()
    _camera(t)
    for r in ("Left", "Center"):
        _fits(t, r, WHEELS_HEALTHY[r], GYRO_HEALTHY[r])
    _fits(t, "Right", WHEELS_HEALTHY["Right"], GYRO_RIGHT_JITTER)
    own = t.checks("gyro", "Right")[0]
    assert own["src"] == "the Alvik prior" and own["k"] < TR.DEGRADED_K      # on its own, with no history: can't tell
    assert _flags(t) == [("gyro", "Right")]
    assert "second check" in t.reason("gyro", "Right")


def test_gyro_jitter_caught_against_its_own_history_even_alone():
    t = TR.SensorTrust()
    _camera(t)
    _fits(t, "Right", WHEELS_HEALTHY["Right"], GYRO_HEALTHY["Right"])
    t.new_session()
    _camera(t, seed=2)
    _fits(t, "Right", WHEELS_HEALTHY["Right"], GYRO_RIGHT_JITTER)
    ch = [c[:3] for c in t.update_flags(["Right"])]       # one robot: no second check possible
    assert ("gyro", "Right", "degraded") in ch
    assert "own history" in t.reason("gyro", "Right")


def test_wheels_jitter_on_every_robot_caught_robot_by_robot():
    # failure 2 of the grid: wheels:jitter=0.08, only Center flagged (the shared part was taken for the camera).
    # Cold (no history): Left and Center by their own runs; Right's draws were mild (runs 2.6% apart, the real robots'
    # healthy median) and its stretches about 2 x the prior: on the line (flagged here, not in the grid replay)
    t = TR.SensorTrust()
    _camera(t)
    got = _session(t, {r: (WHEELS_JITTER08[r], GYRO_HEALTHY[r], RUNS_JITTER08[r]) for r in ROBOTS})
    assert {("wheels", "Center"), ("wheels", "Left")} <= set(got) and all(s == "wheels" for s, _ in got)
    assert t.reason("wheels", "Left").startswith("runs")
    # trained (a healthy session first): all three, Right by its stretches against its own history
    t = TR.SensorTrust()
    _healthy_session(t)
    t.update_flags(ROBOTS)
    t.new_session()
    _camera(t, seed=2)
    got = _session(t, {r: (WHEELS_JITTER08[r], GYRO_HEALTHY[r], RUNS_JITTER08[r]) for r in ROBOTS})
    assert got == [("wheels", "Center"), ("wheels", "Left"), ("wheels", "Right")]
    assert "own history" in t.reason("wheels", "Right")


def test_one_robots_runs_check_needs_no_other_robot():
    t = TR.SensorTrust()
    _camera(t)
    assert _session(t, {"Left": (None, None, RUNS_JITTER08["Left"])}, ["Left"]) == [("wheels", "Left")]
    assert "the prior" in t.reason("wheels", "Left")


def test_short_runs_do_not_count():
    t = TR.SensorTrust()
    assert not t.run_scale("Left", 0.9, TR.RUN_MIN_LEN_CM - 1)
    assert t.run_scale("Left", 0.9, TR.RUN_MIN_LEN_CM + 1)


def test_a_flagged_session_never_becomes_the_baseline():
    t = TR.SensorTrust()
    _camera(t)
    _session(t, {"Left": (None, None, RUNS_JITTER08["Left"])}, ["Left"])
    assert t.bad("wheels", "Left")
    t.new_session()
    _camera(t, seed=3)
    _session(t, {"Left": (None, None, RUNS_JITTER08["Left"])}, ["Left"])   # the same bad wheel: not "as usual"
    c = [c for c in t.checks("wheels", "Left") if c["check"] == "runs"][0]
    assert c["src"] == "the prior" and t.bad("wheels", "Left")


def test_a_flag_holds_to_the_end_of_its_session():
    t = TR.SensorTrust()
    _camera(t)
    _session(t, {"Left": (None, None, RUNS_JITTER08["Left"][:4])}, ["Left"])
    assert t.bad("wheels", "Left")
    for sc in RUNS_HEALTHY["Left"] * 3:            # many calm runs after it: the spread falls, the flag stays
        t.run_scale("Left", sc, 150.0)
        assert t.update_flags(["Left"]) == []
    assert t.bad("wheels", "Left")


def test_a_healthy_session_clears_the_flag():
    t = TR.SensorTrust()
    _camera(t)
    _session(t, {"Left": (None, None, RUNS_JITTER08["Left"])}, ["Left"])
    assert t.bad("wheels", "Left")
    t.new_session()
    _camera(t, seed=4)
    for s in RUNS_HEALTHY["Left"]:
        t.run_scale("Left", s, 150.0)
    ch = [c[:3] for c in t.update_flags(["Left"])]
    assert ("wheels", "Left", "recovered") in ch


def test_no_wheel_or_gyro_flag_while_the_camera_looks_bad():
    t = TR.SensorTrust()
    _camera(t, sd=3.0)
    for r in ROBOTS:
        for s in RUNS_JITTER08[r]:
            t.run_scale(r, s, 150.0)
    ch = t.update_flags(ROBOTS)
    assert [c[:2] for c in ch] == [("camera", "*")]


def test_second_check_waits_until_every_robot_has_its_own_verdict():
    t = TR.SensorTrust()
    _camera(t)
    _fits(t, "Left", None, GYRO_HEALTHY["Left"])
    _fits(t, "Right", None, GYRO_RIGHT_JITTER)
    # Center has no fits yet: nothing to compare with for all three
    assert t.second_check("gyro", "Right", ROBOTS) is None
    assert t.second_check("gyro", "Right", ["Left", "Right"]) is not None


def test_the_second_check_only_adds():
    # every robot worse together: each is still flagged by its own checks (no "all alike" exemption)
    t = TR.SensorTrust()
    _camera(t)
    for r in ROBOTS:
        for s in RUNS_JITTER08[r]:
            t.run_scale(r, s, 150.0)
        _fits(t, r, [(2.5, .8)] * 4, GYRO_HEALTHY[r])
    assert _flags(t) == [("wheels", "Center"), ("wheels", "Left"), ("wheels", "Right")]


def test_front_changed_flags_it_for_the_session():
    t = TR.SensorTrust()
    _camera(t)
    t.mark_changed("front", "Left", "Left's front sensor reads +8.0 cm differently")
    ch = t.update_flags(["Left"])
    assert ("front", "Left", "degraded") in [c[:3] for c in ch]
    assert t.reason("front", "Left").startswith("changed")
    t.new_session()
    assert not t.changed


def test_front_face_readings_count_without_the_cameras_share():
    t = TR.SensorTrust()
    _camera(t, sd=0.8)
    for v in _spread(8, 1.0):
        t.front_face("Left", v)
    assert t.front_sd("Left")[0] < 2.0
    c = t.checks("front", "Left")[0]
    assert "at a board" in c["check"]
    t2 = TR.SensorTrust()
    _camera(t2, sd=0.8)
    for v in _spread(8, 12.0):
        t2.front_face("Left", v)
    assert t2.k("front", "Left") > TR.DEGRADED_K


def test_front_against_its_own_history():
    t = TR.SensorTrust()
    _camera(t)
    for v in _spread(8, 2.0):
        t.front_vs_map("Left", v)
    t.new_session()
    _camera(t, seed=6)
    for v in _spread(8, 6.5):
        t.front_vs_map("Left", v)                         # noise 6 cm on top: under the prior's 2 x 5 cm
    c = t.checks("front", "Left")[0]
    assert "own history" in c["src"] and c["k"] > TR.DEGRADED_K


def test_save_and_load_keeps_reasons_and_exclusions(tmp_path):
    t = TR.SensorTrust()
    _camera(t)
    _session(t, {"Left": (None, None, RUNS_JITTER08["Left"])}, ["Left"])
    t.new_session()
    p = str(tmp_path / "t.json")
    TR.save(t, p)
    t2 = TR.load(p)
    assert t2.reason("wheels", "Left").startswith("runs")
    assert any(it.get("excl") for it in t2.items if it["kind"] == "run_scale")


def test_summary_names_each_check():
    t = TR.SensorTrust()
    _healthy_session(t)
    lines = t.summary(ROBOTS)
    assert any("runs" in ln and "Left" in ln for ln in lines)


# --- the runner's side ------------------------------------------------------------------------------------------

def test_run_scales_per_run():
    import e02 as E
    obs = [dict(run="a", cam_len=50.0, bel_len=51.0), dict(run="a", cam_len=60.0, bel_len=61.0),
           dict(run="b", cam_len=40.0, bel_len=40.0)]                    # b: one stretch only
    out = E.run_scales(obs)
    assert len(out) == 1 and out[0][0] == "a" and abs(out[0][1] - 110.0 / 112.0) < 1e-9 and out[0][2] == 112.0


class _Board:
    pass


def _face_runner(tmp_path, front=12.0, ahead=(10.0, "the y-board", 1.0), n_done=0):
    import e02 as E
    r = E.Runner.__new__(E.Runner)
    r.a = types.SimpleNamespace(no_face_reading=False, no_front_learn=False)
    r.st = {"face_readings": {"Left": n_done}}
    r.board = _Board()
    r.moves = {}
    r.heads = {}
    r.dir = str(tmp_path)
    r.degrade = types.SimpleNamespace(spec={})
    r.trust = TR.SensorTrust()
    _camera(r.trust)
    r.front = FR.FrontModel(None)
    sent = []

    def front_now(robot):
        sent.append(dict(r.moves))
        return front
    r.front_now = front_now
    r.ahead_cm = lambda robot, p, h: ahead
    r.trust_note = lambda save=True: None
    r.say = lambda *a, **k: None
    r.mark = lambda *a, **k: None
    return r, sent


def test_face_reading_uses_the_board_touch_pose_and_records_it(tmp_path):
    r, sent = _face_runner(tmp_path)
    fr = (0, dict(after_x="20.0", after_y="30.0", after_th="0.0"))
    res = r.face_reading("Left", fr)
    assert res == 2.0                                      # 12 cm read, 10 cm by the touches
    assert sent and sent[0]["Left"]["run"] == "-"          # an idle leg, no camera fix (begin_moves) first
    assert "Left" not in r.moves                           # and the moves are closed again
    assert r.heads["Left"] == [2.0, True]                  # the 2 deg turn is kept in its heading
    assert [it["kind"] for it in r.trust.items if it["sensor"] == "front"] == ["face"]
    rows = list(csv.reader(open(os.path.join(str(tmp_path), "front_checks.csv"))))
    assert rows[1][2] == "face"
    assert r.st["face_readings"]["Left"] == 1


def test_face_reading_stops_at_the_target_and_skips_what_is_not_a_board(tmp_path):
    import e02 as E
    r, sent = _face_runner(tmp_path, n_done=E.Runner.FACE_TARGET)
    fr = (0, dict(after_x="20.0", after_y="30.0", after_th="0.0"))
    assert r.face_reading("Left", fr) is None and not sent
    r2, _ = _face_runner(tmp_path, ahead=(10.0, "Center", 1.0))
    assert r2.face_reading("Left", fr) is None
    assert r2.st["face_readings"]["Left"] == 0


def test_camera_still_check_skips_a_robot_during_its_find():
    # 2026-10-25 simulator, front@Left:noise=6: each setup find (`run find`, not an idle leg) was measured as a still
    # robot, 4.8-7.2 cm of "scatter", and the camera was flagged bad 3 times
    import time as _t
    import e02 as E
    r = E.Runner.__new__(E.Runner)
    r.trust = TR.SensorTrust()
    r.trust_note = lambda save=True: None
    r.leg_live, r.moves, r.leg_end = {}, {}, {}
    now = _t.time()
    smp = [(now - 3.0 + 0.1 * i, 50.0, 50.0, 0) for i in range(30)]
    r.history = types.SimpleNamespace(since=lambda t0: smp)
    r.find_live = {"Left": float("inf")}
    r.camera_still_check("Left", {})
    assert not r.trust.items                       # its find is going
    r.find_live["Left"] = now - 1.0
    r.camera_still_check("Left", {})
    assert not r.trust.items                       # just ended: the window still holds the driving
    r.find_live["Left"] = now - 10.0
    r.camera_still_check("Left", {})
    assert len(r.trust.items) == 2                 # standing since: counted
    # another robot's find, or no find at all, does not stop it
    r2 = E.Runner.__new__(E.Runner)
    r2.trust, r2.trust_note = TR.SensorTrust(), (lambda save=True: None)
    r2.leg_live, r2.moves, r2.leg_end, r2.history = {}, {}, {}, r.history
    r2.find_live = {"Center": float("inf")}
    r2.camera_still_check("Left", {})
    assert len(r2.trust.items) == 2


def test_two_finds_do_not_flag_the_camera_four_do():
    # 2026-10-25 rerun, wheels:jitter=0.08: the first two finds read 1.3-2.2 cm (a find's back-off is the wheels'
    # counting) and flagged the camera; the finds now need CAMERA_FIND_MIN items (4 finds)
    t = TR.SensorTrust()
    _camera(t, n=0)
    for v in ((-1.27, -1.59), (1.64, 2.17)):
        t.camera_at_find("Cam", *v)
    assert t.update_flags(ROBOTS) == []
    for v in ((0.1, -0.2), (0.2, 0.05)):                 # two good finds after them: still healthy
        t.camera_at_find("Cam", *v)
    assert t.k("camera") < TR.DEGRADED_K
    t2 = TR.SensorTrust()
    _camera(t2, n=0)
    for v in _spread(TR.CAMERA_FIND_MIN, 3.0):           # a camera really 3 cm off: caught by the 4th find
        t2.camera_at_find("Cam", v, -v)
    assert ("camera", "*", "degraded") in [c[:3] for c in t2.update_flags(ROBOTS)]


def test_a_find_counts_for_the_camera_only_once_its_robots_wheels_are_judged():
    # 2026-10-25 rerun, wheels:jitter=0.08 on every robot: 3 of the first 4 setup finds read 1.2-2.2 cm (a find's pose
    # leans on the robot's own counting) and flagged a healthy camera. Each robot's wheels are judged first.
    t = TR.SensorTrust()
    for v in (-1.27, -1.59, 1.64, 2.17, 1.19, 1.17, 0.4, 0.69):
        t.camera_at_find("Left", v, v)
    assert t.camera_sd()[1] == 0 and t.update_flags(["Left"]) == []
    for sc in RUNS_HEALTHY["Left"][:4]:                  # Left's wheels judged healthy: its finds count now
        t.run_scale("Left", sc, 150.0)
    assert t.camera_sd()[1] == 16
    t.flags["wheels/Left"] = True                       # flagged wheels: their finds never count
    assert t.camera_sd()[1] == 0
    old = TR.SensorTrust(dict(items=[dict(sensor="camera", robot="*", kind="find", v=0.5, w=1.0, t=1.0, session=0)
                                     for _ in range(8)]))
    assert old.camera_sd()[1] == 8                      # saved items from before (no robot named) count as before


def test_front_noise_caught_at_a_board_and_against_the_others():
    # 2026-10-25 rerun, front@Left:noise=6 (its own evidence): Left read 4 cm at a board and 9.8 cm against the map,
    # Center and Right 1 cm at a board; one 5 cm bar for both kinds let it pass (k 1.7)
    t = TR.SensorTrust()
    _camera(t, sd=0.4)
    for v in (0.8, 8.2, -6.5, 2.7, 4.6):
        t.front_face("Left", v)
    for v in (3.2, -6.5, -9.7, 6.7, -11.0, 5.1):
        t.front_vs_map("Left", v)
    for v in (-0.9, -0.8, -0.5, -0.8, -0.5):
        t.front_face("Center", v)
    for v in (1.8, -0.3, 0.9, -0.2, 0.5, -0.1):
        t.front_face("Right", v)
    face = [c for c in t.checks("front", "Left") if c["check"].endswith("at a board")][0]
    assert face["k"] >= TR.DEGRADED_K and face["src"] == "the prior"
    assert _flags(t) == [("front", "Left")]
    sc = t.second_check("front", "Left", ROBOTS)
    assert sc is not None and sc["ratio"] > TR.STANDS_OUT_FLAG
    assert any("at a board" in ln for ln in t.summary(ROBOTS))
