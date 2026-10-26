"""The 2026-10-25 runner fixes (docs/build-reports/e02-oct25-runner-fixes.md): the five 15:10 fixes, the helper
process for heavy calculations, the stall watch and the speed guard."""
import math
import os
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "recognition"))

import numpy as np  # noqa: E402
import e02 as E  # noqa: E402
import e02_explore as EX  # noqa: E402
import e02_offload as OFF  # noqa: E402
import e02_plan as PL  # noqa: E402
import e02_stall as STL  # noqa: E402
import e02_steer as ST  # noqa: E402


def _track(x0, y0, x1, y1, n=6, dt=0.1, t0=0.0):
    return [(t0 + k * dt, x0 + (x1 - x0) * k / (n - 1), y0 + (y1 - y0) * k / (n - 1)) for k in range(n)]


def _border_stub(kind="go"):
    r = types.SimpleNamespace()
    r.board = PL.Board(xboard_side=1, yboard_side=1)     # open edges x = 0 and y = 0
    r.exploring = False
    r.seek, r.edge_touch, r.guard, r.cam, r.moves = set(), None, None, None, {}
    r.leg_live, r.leg_kind = {"Right": 0.0}, {"Right": kind}
    r.border_frame = types.MethodType(E.Runner.border_frame, r)
    r.border_cm = types.MethodType(E.Runner.border_cm, r)
    return r


# --- fix 1: the border's distance is signed ---------------------------------------------------------------------
def test_backing_in_from_past_the_line_is_not_approaching():
    r = _border_stub()
    # 15:14:22, Right: centre 4.1 cm past the line after a line-sensor edge stop, backing 2.5 cm back in
    assert E.Runner.border_check(r, "Right", _track(-4.1, 60.0, -1.6, 60.0)) == ""
    assert E.Runner.border_check(r, "Right", _track(60.0, -5.5, 60.0, -3.0)) == ""      # the y edge the same
    # the old abs() distance read the same track as "3 cm from the edge, moving towards it"
    d0, d1 = abs(-4.1 - 0.0), abs(-1.6 - 0.0)
    assert d0 - d1 > 0


def test_past_the_line_and_going_further_out_still_trips():
    r = _border_stub()
    why = E.Runner.border_check(r, "Right", _track(-1.0, 60.0, -4.0, 60.0))
    assert "past an open edge's line" in why and "further out" in why


def test_inside_the_line_towards_the_edge_still_trips_and_the_high_edges_too():
    r = _border_stub()
    assert "from an open edge" in E.Runner.border_check(r, "Right", _track(7.0, 60.0, 4.0, 60.0))
    bd = PL.Board(xboard_side=0, yboard_side=0)          # open edges at the high ends
    hi = [(a, v) for a, v in bd.open_edges() if a == "x"][0][1]
    assert ST.inward_cm(bd, "x", hi, hi - 3.0) == 3.0 and ST.inward_cm(bd, "x", hi, hi + 2.0) == -2.0
    r.board = bd
    assert E.Runner.border_check(r, "Right", _track(hi + 4.0, 60.0, hi + 1.5, 60.0)) == ""    # past, coming back in
    assert E.Runner.border_check(r, "Right", _track(hi - 4.0, 60.0, hi - 1.0, 60.0)) != ""    # inside, towards it


def test_border_band_counts_a_centre_far_past_the_line():
    r = _border_stub()
    r.tracker = types.SimpleNamespace(active="Right")
    r.history = types.SimpleNamespace(since=lambda t: [(time.time(), -12.0, 60.0)])
    assert E.Runner.in_border_band(r, "Right")           # abs() said 12 cm: outside the 8-15 cm band


# --- fix 5: a turn on the spot gets RECHECK_S before "lost" ---------------------------------------------------
def _lost_stub(kind, t_leg, last_t, started=True):
    """started: the robot's own 'started' LEG record arrived at t_leg (2026-10-26: the lost check counts from it)."""
    import threading as _th
    recs = {"LEG": [(t_leg, "Left", {"id": "7", "state": "started"})] if started else []}
    r = types.SimpleNamespace(leg_live={"Left": t_leg}, leg_kind={"Left": kind}, leg_val={"Left": 175.0},
                              refinding=set(), seek=set(), speed_now={}, last_lid={"Left": 7},
                              listener=types.SimpleNamespace(cv=_th.Condition(), recs=recs),
                              history=types.SimpleNamespace(last_t=lambda: last_t, since=lambda t: []))
    return r


def test_a_turning_robot_unseen_for_3_s_is_not_lost_but_a_driving_one_is():
    now = 100.0
    r = _lost_stub("turn", now - 3.2, now - 3.0)            # 15:19:16, Left: a 3.0 s gap in a 175 deg turn
    assert E.Runner.lost_why(r, "Left", now) == ""
    r = _lost_stub("turn", now - 4.0, now - 3.6)            # past RECHECK_S: lost
    assert "while it turns" in E.Runner.lost_why(r, "Left", now)
    r = _lost_stub("go", now - 3.2, now - 1.6)              # driving: LOST_DRIVE_S as before
    assert "while it drives" in E.Runner.lost_why(r, "Left", now)


# --- fix 3: finish a leg a hold cut short ---------------------------------------------------------------------
class _Listener:
    def __init__(self):
        self.legs = {}

    def leg_final(self, robot, run, lid):
        return self.legs.get(lid, (None, None))[1]

    def leg_start(self, robot, run, lid):
        return self.legs.get(lid, (None, None))[0]


def _leg_stub(plan):
    """plan: per leg sent, (state, info, done amount, hold kind or None, answer)."""
    r = types.SimpleNamespace(hold=threading.Event(), stop_requested=False, held_out=set(), st={"setup": {}},
                              listener=_Listener(), sent=[], said=[], explore_obs_live=None)
    r.say = lambda m, quiet=False: r.said.append(m)
    lid = [100]

    def once(robot, kind, val, timeout=None):
        k = len(r.sent)
        r.sent.append((kind, round(val, 2)))
        state, info, done, hold, ans = plan[k]
        lid[0] += 1
        r.listener.legs[lid[0]] = (dict(x_cm=0.0, y_cm=0.0, th_deg=0.0),
                                   dict(x_cm=done if kind == "go" else 0.0, y_cm=0.0,
                                        th_deg=done if kind == "turn" else 0.0, state=state))
        if hold:
            r.st.setdefault("holds", []).append([time.time(), robot, hold, "why", "explore", ans, 1.0, ""])
            if ans == "o":
                r.held_out.add(robot)
        return state, info, lid[0]
    r._explore_leg_once = once
    r.explore_stop = lambda *a, **k: None
    for name in ("EXPLORE_RESUME_MAX", "EXPLORE_RESUME_MIN_CM", "EXPLORE_RESUME_MIN_DEG"):
        setattr(r, name, getattr(E.Runner, name))
    return r


def test_a_turn_cut_by_a_lost_hold_is_finished_after_carry_on():
    # 15:19:16, Left: 175 deg turn stopped at 72 deg by a lost hold, "c"; before the fix, nothing finished it
    r = _leg_stub([("failed", "stop command", 72.16, "lost", "c"), ("done", "", 102.84, None, None)])
    state, info, lid = E.Runner.explore_leg(r, "Left", "turn", 175.0)
    assert r.sent == [("turn", 175.0), ("turn", 102.84)] and state == "done"
    assert r.explore_resumed[("Left", lid)]["val"] == 102.84


def test_no_finishing_for_a_left_out_robot_a_border_stop_on_a_drive_or_a_plain_stop():
    r = _leg_stub([("failed", "stop command", 72.0, "lost", "o")])
    E.Runner.explore_leg(r, "Left", "turn", 175.0)
    assert len(r.sent) == 1
    r = _leg_stub([("failed", "stop command", 30.0, "border", "c")])
    E.Runner.explore_leg(r, "Left", "go", 62.0)            # never on towards an edge it was held at
    assert len(r.sent) == 1
    r = _leg_stub([("failed", "stop command", 30.0, None, None)])
    E.Runner.explore_leg(r, "Left", "go", 62.0)            # the watchdog's own stop (no hold): not finished
    assert len(r.sent) == 1
    r = _leg_stub([("failed", "stop command", 173.5, "lost", "c")])
    E.Runner.explore_leg(r, "Left", "turn", 175.0)         # 1.5 deg left: not worth a leg
    assert len(r.sent) == 1


def test_a_drive_cut_by_a_lost_hold_is_finished_and_carries_its_first_part():
    r = _leg_stub([("failed", "stop command", 40.0, "lost", "c"), ("done", "", 65.0, None, None)])
    state, info, lid = E.Runner.explore_leg(r, "Center", "go", 105.0)
    assert r.sent == [("go", 105.0), ("go", 65.0)]
    assert r.explore_resumed[("Center", lid)]["d_before"] == 40.0


# --- fix 2: a left-out robot drops out of explore -------------------------------------------------------------
def test_a_left_out_robot_gets_no_more_rays():
    r = types.SimpleNamespace(held_out={"Right"}, st={"setup": {}}, said=[], EXPLORE_RAYS=8)
    r.say = lambda m, quiet=False: r.said.append(m)
    r.explore_one_ray = lambda *a: (_ for _ in ()).throw(AssertionError("a ray for a left-out robot"))
    E.Runner.explore_rays(r, "Right", {"Right": []}, 0.5)
    assert r.st["setup"]["explore_rays_not_driven"] == [["Right", 0, 8]]
    assert E.Runner.explore_one_ray(r, "Right", {"Right": []}, 90.0, 2) is None


def test_a_left_out_robots_unsent_leg_is_no_stop():
    r = types.SimpleNamespace(held_out={"Right"})
    r.listener = types.SimpleNamespace(leg_final=lambda *a: dict(state="refused", info="left out by Nick"))
    r.settle = lambda **k: (_ for _ in ()).throw(AssertionError("a stop recorded"))
    obs = {"Right": [dict(u=1.0, v=1.0)]}
    assert E.Runner.explore_stop(r, "Right", obs, 7) is None and len(obs["Right"]) == 1


def _square_obs(x0, y0, n=8):
    """A robot's explore stops: legs of 20 cm and 90 deg turns, the camera at 1 px a cm (u = x, v = y)."""
    obs, x, y, th = [], x0, y0, 0.0
    obs.append(dict(x=0.0, y=0.0, th=0.0, d=0.0, dth=0.0, u=x, v=y))
    for k in range(n):
        if k % 2 == 0:
            x += 20.0 * math.cos(math.radians(th))
            y += 20.0 * math.sin(math.radians(th))
            obs.append(dict(x=20.0, y=0.0, th=0.0, d=20.0, dth=0.0, u=x, v=y))
        else:
            th += 90.0
            obs.append(dict(x=0.0, y=0.0, th=90.0, d=0.0, dth=90.0, u=x, v=y))
    return obs


class _IdCam:
    def to_floor(self, u, v, z=0.0):
        return (u, v)


def _explore_stub(positions, obs):
    r = types.SimpleNamespace(held_out=set(), unavailable=set(), st={"positions": positions, "setup": {}},
                              cam=_IdCam(), said=[], EXPLORE_RAY_CM=E.Runner.EXPLORE_RAY_CM,
                              EXPLORE_SECTORS=8, EXPLORE_SECTOR_MIN=2)
    r.say = lambda m, quiet=False: r.said.append(m)
    r.explore_ray_blocked = types.MethodType(E.Runner.explore_ray_blocked, r)
    r.explore_clear_leg = types.MethodType(E.Runner.explore_clear_leg, r)
    return r


def test_gap_rays_never_go_to_a_left_out_robot_and_only_driven_rays_count():
    obs = {"Right": _square_obs(80.0, 20.0), "Left": _square_obs(20.0, 80.0)}
    r = _explore_stub({"Right": [100.0, 40.0, None], "Left": [20.0, 100.0, None]}, obs)
    r.explore_gaps = lambda names, o: ([(45.0, 0)], (60.0, 60.0))
    r.tracker = types.SimpleNamespace(follow=lambda *a: None, hold=lambda *a: None, park=lambda *a: None)
    r.history = types.SimpleNamespace(clear=lambda: None)
    r.moves, r.settle = {}, lambda **k: None
    got = []
    r.explore_one_ray = lambda n, o, h, k: got.append(n) or None       # not driven (e.g. it would pass a robot)
    r.held_out.add("Right")
    assert E.Runner.explore_gap_rays(r, ["Right", "Left"], obs) == 0 and got == ["Left"]
    r.explore_one_ray = lambda n, o, h, k: got.append(n) or ""         # driven, nothing met
    assert E.Runner.explore_gap_rays(r, ["Right", "Left"], obs) == 1


# --- fix 4: robots kept apart during explore ------------------------------------------------------------------
def test_home_and_return_legs_are_checked_like_rays():
    obs = {"Left": _square_obs(20.0, 20.0)}
    here = (obs["Left"][-1]["u"], obs["Left"][-1]["v"])
    # Center parked 38 cm straight ahead of Left's last stop
    path = EX.rebuild(obs["Left"], 1.0, 1.0)
    th = path[-1][2]
    ahead = (here[0] + 38.0 * math.cos(math.radians(th)), here[1] + 38.0 * math.sin(math.radians(th)))
    r = _explore_stub({"Center": [ahead[0], ahead[1], None]}, obs)
    assert r.explore_ray_blocked("Left", obs, th, length_cm=60.0) == "Center"
    assert r.explore_ray_blocked("Left", obs, th, length_cm=8.0) == ""          # stops well short of it
    cm, who = r.explore_clear_leg("Left", obs, th, 60.0)
    assert who == "Center" and cm == 0.0           # 25 cm clearance at its end too: even 15 cm ends 25 from it
    cm, who = r.explore_clear_leg("Left", obs, th + 180.0, 60.0)
    assert who == "" and cm == 60.0


def test_watchdog_hazard_checks_parked_robots_with_no_board():
    smp = _track(0.0, 0.0, 3.0, 0.0, n=12, dt=0.05)            # 5 cm/s towards a parked robot at (16, 0)
    assert "parked Center" in (ST.hazard(smp, None, {"Center": (16.0, 0.0)}) or "")
    assert ST.hazard(smp, None, {"Center": (-16.0, 0.0)}) is None   # driving away
    assert ST.hazard(_track(0.0, 0.0, -3.0, 0.0, n=12, dt=0.05), None, {}) is None   # no edges without a board


# --- part 2a: the helper process ------------------------------------------------------------------------------
def test_the_helper_process_gives_the_same_answer_and_raises_the_same_errors():
    w = OFF.Worker()
    try:
        own = [(0, 0), (20, 0), (20, 20), (0, 20), (10, 10)]
        flo = [(5 + y, 3 - x) for x, y in own]                       # turned 90 deg and moved
        A1, t1 = w.call(EX.own_to_floor_map, own, flo)
        A0, t0 = EX.own_to_floor_map(own, flo)
        assert np.allclose(A1, A0) and np.allclose(t1, t0)
        assert w.calls[-1][2] == "helper" and w.p is not None and w.p.pid != os.getpid()
        try:
            w.call(EX.own_to_floor_map, own, [(0, 0), (1, 1)])        # bad input: numpy's error comes back
            raised = False
        except Exception:
            raised = True
        assert raised
        assert w.call(EX.own_to_floor_map, own, flo) is not None      # and the helper is still up
    finally:
        w.stop()


def test_without_a_helper_the_call_runs_in_the_runner():
    w = OFF.Worker()
    w.failed = "test"
    assert w.call(EX.own_to_floor_map, [(0, 0)], [(0, 0)]) is None
    assert w.calls[-1][2] == "runner"


# --- part 2a: the stall watch ---------------------------------------------------------------------------------
def test_the_stall_watch_names_what_held_the_interpreter(tmp_path):
    sw = STL.StallWatch(str(tmp_path))
    sw.start()
    time.sleep(0.2)
    t0 = time.time()
    n = 0
    while time.time() - t0 < 0.35 and n < 40:
        sum(range(4 * 10 ** 6))          # one C call that keeps the lock (as a long numpy call or a big pickle)
        n += 1
    time.sleep(0.3)
    sw.stop()
    sm = sw.summary()
    assert sm["ticks"] > 10
    assert os.path.exists(os.path.join(str(tmp_path), "threads.csv"))
    if sm["stalls"]:                                   # a 4e6 sum takes ~30-100 ms: not every machine stalls 100
        assert any(k.startswith("main") for k, _ in sm["culprits_ms"])


# --- part 2b: the speed guard ---------------------------------------------------------------------------------
def _speed_stub(fps, age, home=True, t_row=None):
    r = _border_stub()
    r.a = types.SimpleNamespace(home=home)
    r.perf_rows = [dict(t=t_row if t_row is not None else time.time(), fps_handled=fps, age_mean_s=age)]
    r.speed_now = {"Right": 10.0}
    r.st = {"positions": {}}
    r.phase, r.order = "exploring", "explore"
    r.stops, r.said, r.marks = [], [], []
    r.link = types.SimpleNamespace(send_name=lambda n, c: r.stops.append((n, c)))
    r.say = lambda m, quiet=False: r.said.append(m)
    r.mark = lambda m: r.marks.append(m)
    r.stop_requested = False
    for m in ("camera_lag_s", "speed_guard_cm", "speed_guard_check", "in_band", "speed_guard_wait", "border_check"):
        setattr(r, m, types.MethodType(getattr(E.Runner, m), r))
    return r


def test_the_guard_band_grows_with_the_lag_above_budget():
    r = _speed_stub(24.0, 0.09)                        # healthy: lag 0.13 s, inside home's 0.2 s
    assert r.speed_guard_cm("Right") == 0.0
    r = _speed_stub(3.8, 0.33)                         # 15:10's worst second: lag 0.59 s
    assert abs(r.speed_guard_cm("Right") - 10.0 * (0.33 + 1 / 3.8 - E.SPEED_LAG_BUDGET_HOME_S)) < 1e-6
    r.speed_now["Right"] = 4.0                         # a crawl drives less in the same lag
    assert r.speed_guard_cm("Right") < 2.0
    r = _speed_stub(3.8, 0.33, home=False)             # away: the 15 cm border already holds 0.56 s
    assert r.speed_guard_cm("Right") < 0.4
    r = _speed_stub(24.0, 0.09, t_row=time.time() - 4.0)   # no row for 4 s: the camera loop is stuck
    assert r.speed_guard_cm("Right") >= 10.0 * (3.0 - 0.2) - 1e-6 or r.speed_guard_cm("Right") == E.SPEED_GUARD_MAX_CM


def test_the_guard_stops_a_robot_in_the_wider_band_only_while_the_camera_is_slow():
    smp = _track(12.0, 60.0, 10.0, 60.0)               # 10 cm from the edge (home border 8), 4 cm/s towards it
    r = _speed_stub(24.0, 0.09)
    assert not r.speed_guard_check("Right", smp, {}) and r.stops == []
    r = _speed_stub(3.8, 0.33)                         # band 8 + 3.9 cm: stopped, no hold
    assert r.speed_guard_check("Right", smp, {})
    assert ("Right", "stop") in r.stops and r.st["speed_guard_stops"][0][1] == "Right"
    r = _speed_stub(3.8, 0.33)
    assert not r.speed_guard_check("Right", _track(10.0, 60.0, 12.0, 60.0), {})   # moving away: allowed


def test_a_move_in_the_band_waits_for_the_camera_and_one_outside_does_not():
    r = _speed_stub(3.8, 0.33)
    r.tracker, r.history = None, None
    r.st["positions"] = {"Right": [10.0, 60.0, None], "Left": [60.0, 60.0, None]}
    assert r.speed_guard_wait("Left", "go 10") == 0.0          # 60 cm from both edges: goes at once

    def recover():
        time.sleep(0.6)
        r.perf_rows.append(dict(t=time.time(), fps_handled=24.0, age_mean_s=0.09))
        time.sleep(0.6)
        r.perf_rows.append(dict(t=time.time(), fps_handled=24.0, age_mean_s=0.09))
    threading.Thread(target=recover).start()
    waited = r.speed_guard_wait("Right", "go 10")
    assert 0.9 < waited < 5.0 and r.st["speed_guard_waits"][0][-1] is True


# --- explore's creep: an open edge with an object beyond it is not a board (Nick, 2026-10-25) -------------------
def _creep_stub(steps, lag=0.15):
    """steps: per creep leg, (state, info, cm driven)."""
    r = types.SimpleNamespace(held_out=set(), st={"setup": {}}, sent=[], legs=[], said=[], speed_now={})
    r.say = lambda m, quiet=False: r.said.append(m)
    r.send = lambda n, c: r.sent.append(c)
    r.explore_speed = lambda n, cms: r.speed_now.__setitem__(n, cms)
    r.camera_lag_s = lambda: lag
    finals = {}

    def leg(n, kind, val, timeout=None):
        k = len(r.legs)
        state, info, cm = steps[k]
        r.legs.append((kind, val, r.speed_now.get(n)))
        finals[k] = dict(state=state, info=info, cm=cm)
        return state, info, k
    r.explore_leg = leg
    r.listener = types.SimpleNamespace(leg_final=lambda n, run, lid: finals[lid])
    r.explore_stop = lambda n, obs, lid, contact=None: dict(d=finals[lid]["cm"], contact=contact)
    return r


def test_an_object_past_an_open_edge_reads_as_an_open_edge():
    # 15:10: the front stop at 7 cm from a light object past the bottom edge; creeping, the floor ends after 4 cm
    r = _creep_stub([("done", "front=4.0", 3.0), ("failed", "plywood edge (line)", 1.0)])
    o = dict(d=40.0, contact="board")
    kind, cm = E.Runner.explore_creep(r, "Left", {}, o, dict(state="failed", info="obstacle front=7.0"), 3)
    assert kind == "edge" and cm == 4.0 and "contact" not in o
    assert r.sent[0] == "set safe_front_cm %g" % E.EXPLORE_CREEP_FRONT_CM and r.sent[-1] == "set safe_front_cm 7"
    rec = r.st["setup"]["explore_creeps"][0]
    assert rec[5] == "edge" and rec[6] == 3.0                 # the object about 3 cm on from where the floor ended


def test_a_board_out_from_the_plywood_with_floor_under_the_gap_is_a_blocked_side():
    r = _creep_stub([("done", "front=4.1", 3.0), ("failed", "obstacle front=1.9", 2.2)])
    kind, cm = E.Runner.explore_creep(r, "Left", {}, dict(d=40.0, contact="board"),
                                      dict(state="failed", info="obstacle"), 2)      # no reading: 7 cm assumed
    assert kind == "board" and abs(cm - 5.2) < 1e-9
    assert all(v <= E.EXPLORE_CREEP_STEP_CM for _, v, _ in r.legs)


def test_the_creep_slows_down_with_a_slow_camera():
    r = _creep_stub([("failed", "plywood edge (bottom)", 1.0)], lag=0.59)          # 15:10's worst second
    E.Runner.explore_creep(r, "Left", {}, dict(d=40.0), dict(state="failed", info="obstacle"), 1)
    cms = r.legs[0][2]
    assert cms <= E.EXPLORE_CREEP_UNSEEN_CM / 0.59 + 1e-9 and cms < E.EXPLORE_CREEP_CMS


def test_the_watchdog_hazard_still_works_at_4_pictures_a_second():
    # 2026-10-25 simulator, camera slowed to 4 a second: a ray drove into parked Right; the check wanted 6 pictures
    smp = _track(0.0, 0.0, 4.8, 0.0, n=5, dt=0.25)             # 4 a second, 4 cm/s towards Right at (16, 0)
    assert "parked Right" in (ST.hazard(smp, None, {"Right": (16.0, 0.0)}) or "")
    assert ST.hazard(smp[-2:], None, {"Right": (16.0, 0.0)}) is None      # two pictures: too few to judge
