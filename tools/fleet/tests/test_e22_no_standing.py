"""e22 (Nick 13 Nov): robots never wait for a person (the recovery ladder), wrong-robot sightings refused (shape
exclusivity, motion signature, track continuity), the escape's guards, the robots' own escape off, the Pi's speed
(planning paced, the spatial cull, a resumable search), the live error shape for Mission Control, the back reach's
refit, and the simulator's measured bumps with the edge checked. Offline only.

Changelog: e22-2026-11-13a first version. e22b: the ladder's clock is not restarted by moves planned and refused;
the simulator's seeds do not depend on the machine's speed. e22c: the boxed-in corner (Nick 13 Nov ~10:00).
"""
import json
import math
import os
import random
import subprocess
import sys
import time

import numpy as np
import pytest

FLEET = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(FLEET))
sys.path.insert(0, FLEET)

import error_shape as ES         # noqa: E402
import freeroam_core as FC       # noqa: E402
import odo_trust as OT           # noqa: E402
import safety_group as SG        # noqa: E402

BOARD = (0.0, 0.0, 122.0, 122.0)


def _ctrl(names=("A", "B")):
    out, logs = [], []
    c = FC.Controller(list(names), BOARD, lambda n, t: out.append((n, t)), lambda tb, row: logs.append((tb, row)))
    c.t = 1.0
    return c, out, logs


def _place(c, n, x, y, th, t=1.0, lost=False):
    r = c.R[n]
    c.on_follow(n, t - 0.2, "idle", x, y, th)
    c.on_follow(n, t, "idle", x, y, th)
    r.cam = (t, x, y, None)
    r.cam_arr = t
    r.mode = "idle"
    if lost:
        c.t = t
        c.start_lost(r, "test")
    return r


def _drive(c, n, pts, t0, dt=0.1, cam=None):
    """Its own pose along pts (one FOLLOW row each dt) and, with cam, a sighting at cam(p) each step (named to it)."""
    t = t0
    for (x, y) in pts:
        t += dt
        c.t = t
        c.on_follow(n, t, "drive", x, y, c.R[n].own[3] if c.R[n].own else 0.0)
        if cam is not None:
            q = cam((x, y))
            c.id_hist_add(c.R[n], t, q[0], q[1])
    return t


# -- the switches -------------------------------------------------------------------------------------------------------
def test_the_e22_switches_are_in_the_safety_group_with_their_old_ways():
    rows = {r[0]: r for r in SG.SWITCHES}
    assert rows["MIRLT_SELF_RESCUE"][1:3] == ("ladder", "ask")
    assert rows["MIRLT_ID_GUARD"][1:3] == ("1", "0")
    assert rows["MIRLT_FW_ESCAPE"][1:3] == ("pi", "robot")
    assert rows["MIRLT_PLAN_BUDGET"][1:3] == ("derived", "every_tick")
    assert FC.SELF_RESCUE and FC.ID_GUARD and not FC.FW_ESCAPE_ROBOT and FC.PLAN_BUDGET


# -- 2. who the camera saw --------------------------------------------------------------------------------------------
def test_a_sighting_inside_another_robots_shape_is_ambiguous_and_refused():
    c, out, logs = _ctrl()
    _place(c, "A", 40.0, 60.0, 0.0)
    _place(c, "B", 52.0, 60.0, 180.0)
    ok, why = c.id_check(c.R["A"], 1.0, 51.0, 60.0)
    assert not ok and "B" in why


def test_a_mirrored_chain_moving_against_its_wheels_is_refused():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 40.0, 60.0, 0.0)
    pts = [(40.0 + k, 60.0) for k in range(1, 11)]
    t = _drive(c, "A", pts, 1.0, cam=lambda p: (122.0 - p[0], p[1]))     # mirrored left-right: moves the other way
    r.cam = None
    ok, why = c.id_check(r, t + 0.05, 122.0 - 50.5, 60.0)
    assert not ok and "unlike its own wheels" in why


def test_a_chain_that_moves_with_its_wheels_inside_its_shape_is_its_own():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 40.0, 60.0, 0.0)
    pts = [(40.0 + k, 60.0) for k in range(1, 11)]
    t = _drive(c, "A", pts, 1.0, cam=lambda p: (p[0] + 3.0, p[1]))      # its own pose 3 cm behind: inside its shape
    r.cam = None
    ok, why = c.id_check(r, t + 0.05, 53.5, 60.0)
    assert ok, why


def test_it_has_not_moved_since_its_last_fix_so_a_sighting_elsewhere_is_not_it():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 40.0, 60.0, 0.0, t=1.0)
    for k in range(10):
        c.t = 1.0 + 0.1 * (k + 1)
        c.on_follow("A", c.t, "idle", 40.0, 60.0, 0.0)
        c.id_hist_add(r, c.t, 49.0, 60.0)
    ok, why = c.id_check(r, c.t, 49.0, 60.0)
    assert not ok and "has not moved" in why


def test_the_guard_off_is_e21(monkeypatch):
    monkeypatch.setattr(FC, "ID_GUARD", False)
    c, out, logs = _ctrl()
    _place(c, "A", 40.0, 60.0, 0.0)
    _place(c, "B", 52.0, 60.0, 180.0)
    assert c.id_check(c.R["A"], 1.0, 51.0, 60.0)[0]


def test_a_correction_from_another_robots_spot_is_held_back():
    c, out, logs = _ctrl()
    rA = _place(c, "A", 40.0, 60.0, 0.0)
    _place(c, "B", 70.0, 60.0, 180.0)
    n0 = len(out)
    res = c.correct_from_camera(rA, 1.0, 70.0, 60.0, None, 1.0, "elimination")
    assert res is None
    assert not any("fpose" in str(t) for n, t in out[n0:])
    assert any(tb == "identity" for tb, row in logs)


# -- 1. never wait for a person: the ladder ------------------------------------------------------------------------------
def test_a_robot_that_stands_takes_a_ladder_rung_after_the_derived_dwell_and_nobody_is_asked():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 60.0, 60.0, 0.0)
    dwell = c.rescue_dwell(r)
    assert 0.5 < dwell < 30.0
    t = 1.0
    for k in range(int((dwell + 2.0) / 0.1)):
        t += 0.1
        c.t = t
        c.on_follow("A", t, "idle", 60.0, 60.0, 0.0)
        r.mode, r.move = "idle", None
        r.busy_until = t + 10.0               # its own state machines keep it waiting
        c.rescue_watch(r)
        if r.move is not None:
            break
    assert r.move is not None and str(r.move["kind"]).startswith(("rescue", "outback"))
    assert any(tb == "rescue" for tb, row in logs)
    assert not any("NEEDS A HAND" in str(row.get("kind", "")) for tb, row in logs if tb == "events")


def test_moves_planned_and_refused_every_second_do_not_keep_the_ladder_away():
    # (e22b) the 25-run batch, missing_cam seed 1: a reverse refused every 1.5 s for 300 s restarted e22's first clock
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 60.0, 60.0, 0.0)
    dwell = c.rescue_dwell(r)
    every = max(2, int(dwell / 2.0 / 0.1))                    # a refused plan twice in each dwell
    t = 1.0
    fired = False
    for k in range(int((dwell * 3 + 5.0) / 0.1)):
        t += 0.1
        c.t = t
        c.on_follow("A", t, "idle", 60.0, 60.0, 0.0)
        if k % every == 0:
            r.mode, r.move = "move", dict(kind="refused")     # planned, then refused: its own pose never moves
        else:
            r.mode, r.move = "idle", None
        c.rescue_watch(r)
        if r.move is not None and str(r.move.get("kind")).startswith(("rescue", "outback")):
            fired = True
            break
    assert fired
    assert any(tb == "rescue" for tb, row in logs)


def test_the_simulator_never_defers_planning_by_the_machines_clock():
    import freeroam_fastsim as FS
    if "MIRLT_PLAN_SLOWDOWN" in os.environ:
        pytest.skip("MIRLT_PLAN_SLOWDOWN set for this session")
    s = FS.Sim(1, 0.01) if hasattr(FS, "Sim") else None
    if s is None:
        pytest.skip("no Sim class")
    assert s.ctrl.plan_slowdown == 0.0


def test_asking_for_a_click_is_a_note_not_a_wait():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 60.0, 60.0, 0.0)
    called = []
    c.place_fn = lambda *a, **k: called.append(a)
    c.ask_place(r)
    assert not called and r.__dict__.get("place_asked") is None
    assert any(tb == "notes" for tb, row in logs)


def test_the_old_way_still_asks_for_a_click(monkeypatch):
    monkeypatch.setattr(FC, "SELF_RESCUE", False)
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 60.0, 60.0, 0.0)
    called = []
    c.place_fn = lambda *a, **k: called.append(a)
    c.ask_place(r)
    assert called and r.place_asked is not None


def test_nobody_is_waited_for_even_when_someone_watches():
    c, out, logs = _ctrl(("A",))
    c.attended = True
    assert c.attended_live() is False


def test_the_ladder_moves_inward_when_a_move_gains_room():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 20.0, 60.0, 180.0, lost=True)          # facing the near edge, lost
    for k in range(10):
        c.t += 0.2
        c.on_follow("A", c.t, "drive", 20.0 - 0.5 * (k + 1), 60.0, 180.0)
    r.mode, r.move = "idle", None
    p, h = c.pos(r), c.heading(r)
    g = c.rescue_move(r, p, h)
    assert g is not None
    end = g["tr"].end
    assert FC.edge_dist(BOARD, *end) >= FC.edge_dist(BOARD, *p) - 0.5


def test_a_lost_robot_is_no_longer_held_unsure():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 12.0, 60.0, 0.0)
    r.unsure = dict(t=1.0, why="test", tried=0)
    c.start_lost(r, "test")
    c.edge_watch(r)
    assert r.__dict__.get("unsure") is None


def test_learned_coverage_counts_finds_where_robots_are_found():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 60.0, 60.0, 0.0)
    for k in range(10):
        c.t = 1.0 + 0.5 * k
        r.cam, r.cam_arr = (c.t, 60.0, 60.0, None), c.t
        c.coverage_learn()
    hi = c.found_chance_learned(60.0, 60.0)
    lo = c.found_chance_learned(10.0, 110.0)
    assert hi > lo


# -- 4. the robots' own escape off -------------------------------------------------------------------------------------
def test_the_pi_sends_fix_escape_n_0_to_e21_firmware():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 60.0, 60.0, 0.0)
    r.fw = "2026-11-13e22"
    c.fw_filter_params(r)
    assert any(t == "set fix_escape_n 0" for n, t in out)


def test_the_firmware_default_is_0_for_the_first_flash():
    par = open(os.path.join(REPO, "firmware", "home_lab", "params.h")).read()
    line = [ln for ln in par.splitlines() if '"fix_escape_n"' in ln][0]
    assert line.split(",")[1].strip() == "0"


# -- 6. the Pi's speed ----------------------------------------------------------------------------------------------------
def test_the_spatial_cull_never_changes_an_answer_under_plan_cc(monkeypatch):
    rng = random.Random(3)
    n_culled = 0
    for _ in range(400):
        p0 = (rng.uniform(0, 122), rng.uniform(0, 122))
        q0 = (rng.uniform(0, 122), rng.uniform(0, 122))
        A = FC.Traj.move(0.0, p0, rng.uniform(-180, 180), 0.0, rng.uniform(5, 60), 7.5)
        B = FC.Traj.move(0.0, q0, rng.uniform(-180, 180), 0.0, rng.uniform(5, 60), 7.5)
        monkeypatch.setattr(FC, "CULL", True)
        got = FC.clearance(A, B, now=0.0)
        monkeypatch.setattr(FC, "CULL", False)
        exact = FC.clearance(A, B, now=0.0)
        assert FC.clearance_lb(A, B) <= exact + 1e-9
        for need in (FC.REFLEX_CC, FC.PLAN_CC):
            assert (got < need) == (exact < need)
        n_culled += got != exact
    assert n_culled > 50                       # the cull does skip most far pairs


def test_pick_resumes_next_tick_when_the_ticks_time_runs_out(monkeypatch):
    c, out, logs = _ctrl(("A", "B"))
    r = _place(c, "A", 60.0, 60.0, 0.0)
    _place(c, "B", 30.0, 30.0, 0.0)
    c.tick_t0 = time.perf_counter() - 1.0          # the tick's time already spent
    res = c.pick(r)
    assert res is FC.DEFER
    c.tick_t0 = time.perf_counter()
    res = c.pick(r)
    assert res is not FC.DEFER


def test_planning_waits_a_tick_when_the_budget_is_spent_but_never_twice():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 60.0, 60.0, 0.0)
    c.tick_t0 = time.perf_counter() - 1.0
    c.plans_tick = 1
    c.plan_s = [0.01] * 50
    assert c.plan_slot(r) is False
    r.plan_denied = True
    assert c.plan_slot(r) is True


def test_the_shape_fit_is_read_once_a_tick():
    tr = OT.Trust("A")
    sh = ES.Shape("A", tr)
    sh.anchor(0.0, 60.0, 60.0, 0.0, 2.45)
    sh.axes_fit.get()
    calls = []
    orig = tr._fleet_n
    tr._fleet_n = lambda: (calls.append(1), orig())[1]
    for _ in range(20):
        sh.axes_fit.get()
    assert len(calls) == 0
    ES.EPOCH[0] += 1
    sh.axes_fit.get()
    assert len(calls) == 1


def test_the_timing_bench_runs_and_writes_its_file(tmp_path):
    r = subprocess.run([sys.executable, os.path.join(REPO, "tools", "pi", "plan_timing_bench.py"), "--quick",
                        "--hours", "0.003", "--out", str(tmp_path)], capture_output=True, text=True, timeout=600,
                       env=dict(os.environ, HOME=str(tmp_path)))
    assert r.returncode == 0, r.stderr[-500:]
    assert "slowest 1 in 100 ticks" in r.stdout and "RESULT" in r.stdout
    fs = list(tmp_path.glob("plan-timing-*.json"))
    assert fs and json.load(open(fs[0]))["cases"][0]["ticks"] > 10


# -- 7. the shape every second for Mission Control -------------------------------------------------------------------
def test_the_shape_to_draw_and_the_live_view_reads_it(tmp_path):
    import live_view as LV
    sh = ES.Shape("A", OT.Trust("A"))
    sh.anchor(0.0, 60.0, 60.0, 0.0, 2.45)
    for k in range(20):
        sh.drive(60.0 + k + 1, 60.0, 0.0, 1.0, 0.0, False)
    d = sh.draw()
    assert set(d) >= {"x", "y", "iso", "a", "b", "ang", "off", "gx", "gy", "extent"}
    (tmp_path / "shape_live.json").write_text(json.dumps(dict(t=time.time(), robots={"A": dict(
        shape=d, summary=sh.summary(), stage="veer", lost=True, rescue=True)})))
    (tmp_path / "follow.csv").write_text("t,robot,x_cm,y_cm,th_deg,state,pos_sd_cm\n%f,A,80,60,0,drive,1.0\n" %
                                         time.time())
    rows = LV.from_freeroam(str(tmp_path), {"A": "#123456"}, {})
    a = [q for q in rows if q["name"] == "A"][0]
    assert a["shape_draw"]["a"] == d["a"] and a["stage"] == "veer" and a["rescue"]
    html = open(os.path.join(FLEET, "mission_control_table.html")).read()
    assert "shape_draw" in html and "setInterval(liveLoad, 1000)" in html


# -- 5. the back reach refitted from each run's record ---------------------------------------------------------------
def test_the_back_reach_moves_with_the_robots_own_record():
    tr = OT.Trust("A")
    sh = ES.Shape("A", tr)
    f0 = dict(sh.axes_fit._compute())
    assert f0["b_back"] == pytest.approx(ES.PRIOR["b_back"], rel=0.05) and f0["prior_b_back"] == ES.PRIOR["b_back"]
    for k in range(60):                            # a robot whose wheels never leave it behind
        s = 10.0 + (k % 3) * 15.0
        tr.add_sample(s, 1.0, t=float(k), al=0.5, ac=0.5)
    f1 = dict(sh.axes_fit._compute())
    assert f1["source"] == "record" and f1["b_back"] < f0["b_back"]


# -- 8. the simulator's measured bumps -----------------------------------------------------------------------------------
def test_the_measured_bump_rate_comes_from_the_real_runs():
    import fastsim_realism as R
    assert 0.5 < R.MEASURED_BUMPS["per_robot_h"] < 5.0
    d = json.load(open(os.path.join(REPO, "docs", "results", "e22-nov13", "bump_rate.json")))
    assert d["felt"] >= 10 and d["robot_h"] > 10
    assert R.MEASURED_BUMPS["per_robot_h"] == pytest.approx(d["bumps_per_robot_h"])


def test_a_bump_over_the_edge_is_a_fall_at_once():
    import freeroam_fastsim as F
    import fastsim_realism as R
    prof = R.load_profile("measured")
    s = R.make_class(F)(1, 0.01, profile="measured", prof=prof, faults={}, camera="detections", sensors=True,
                        edge_physics=True)
    b = s.bots[s.names[0]]
    b.x = 2.0
    s.bumps[b.name] = [[0.0, -5.0, 0.0, False]]
    s.t = 0.1
    s._prev = {}
    s._bumps()
    assert b.dead and s.stats["falls"] >= 1


# -- 2/3 with the whole flow: the ratchet's standing robot (11 Nov 08:26) is still corrected, from its own sightings -----
def test_a_standing_robot_31_cm_off_is_corrected_once_its_own_move_shows_the_sighting_is_its_own():
    sys.path.insert(0, os.path.join(FLEET, "tests"))
    import e17_harness as H
    R = H.Rig({"Left": (30.0, 30.0, 0.0), "Center": (60.0, 90.0, 90.0), "Right": (95.0, 30.0, 180.0)}, seed=2)
    R.run(3.0)
    R.off["Center"] = [31.0 * 0.6, -31.0 * 0.8, 0.0]       # its own pose 31 cm off, the robot standing
    R.run(40.0)
    ev = [e for e in R.events if e[1] == "Center" and e[2] == "CORRECTED"]
    assert ev, [e[2] for e in R.events if e[1] == "Center"][-20:]
    assert R.own_gap("Center") < 4.0
    assert all(R.own_gap(n) < 6.0 for n in ("Left", "Right"))        # nobody else corrected onto its spot


# -- missions' veer (Nick 13 Nov): it leaves its lane only where its shape stays clear of the edge and others' shapes -----
def test_a_missions_turn_back_is_checked_with_its_shape_and_the_others_shapes():
    import types
    import run_plan as RP
    cls = [c for c in vars(RP).values() if isinstance(c, type) and hasattr(c, "recover_shape_ok")][0]
    sh = ES.Shape("A", OT.Trust("A"))
    sh.anchor(0.0, 60.0, 60.0, 0.0, 2.45)
    for k in range(20):
        sh.drive(60.0 + k + 1, 60.0, 0.0, 1.0, 0.0, False)
    me = types.SimpleNamespace(name="A", out=False, pose=(80.0, 60.0, 0.0), blind=dict(shape=sh))
    far = types.SimpleNamespace(name="B", out=False, pose=(20.0, 110.0, 0.0), blind=None)
    rect = (0.0, 0.0, 122.0, 122.0)
    self_ = types.SimpleNamespace(bots={"A": me, "B": far}, RECOVER_GAP_CM=cls.RECOVER_GAP_CM)
    ok, why = cls.recover_shape_ok(self_, me, me.blind, 80.0, 60.0, 62.0, 60.0, rect)
    assert ok, why
    near = types.SimpleNamespace(name="B", out=False, pose=(66.0, 72.0, 0.0), blind=None)    # beside its way back
    self_.bots["B"] = near
    ok, why = cls.recover_shape_ok(self_, me, me.blind, 80.0, 60.0, 62.0, 60.0, rect)
    assert not ok and "B" in why
    self_.bots["B"] = far
    ok, why = cls.recover_shape_ok(self_, me, me.blind, 80.0, 60.0, 118.0, 60.0, rect)       # toward the far edge
    assert not ok and "edge" in why


def test_a_shape_checked_move_at_the_blind_limit_is_not_eased_to_a_stop_the_moment_it_starts():
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 9.0, 60.0, 0.0, lost=True)          # lost, near the edge: its blind limit
    assert c.blind_stop(r, c.pos(r), 5.0)
    for kind in FC.SHAPE_CHECKED_KINDS:
        r.mode = "move"
        r.move = dict(kind=kind, cell=None, turn=0.0, L=0.0, v=5.0, t0=c.t, p0=c.pos(r), h0=0.0, h1=0.0,
                      end=c.pos(r), phase="turn", n=1, drive_id=None, turn_id=None, t_drive=None)
        c.watch(r)
        assert r.move is not None and r.move["kind"] == kind, kind
    r.move["kind"] = "creep"                               # anything else still eases to a stop there
    c.watch(r)
    assert r.move is None


def test_a_sighting_on_another_robots_own_track_is_ambiguous_even_when_its_own_pose_is_wrong():
    c, out, logs = _ctrl()
    rA = _place(c, "A", 40.0, 60.0, 0.0)
    rB = _place(c, "B", 100.0, 100.0, 0.0)                 # B's own pose is wrong: it is really at 52, 60
    c.id_hist_add(rB, 0.9, 52.0, 60.0)
    ok, why = c.id_check(rA, 1.0, 52.3, 60.1)
    assert not ok and "B's own track" in why


def test_a_waiting_reseed_is_checked_again_when_it_lands():
    c, out, logs = _ctrl()
    rA = _place(c, "A", 40.0, 60.0, 0.0)
    _place(c, "B", 47.0, 60.0, 180.0)                       # B now stands on the spot
    rA.fpose_pending = dict(t_pic=1.0, x=47.0, y=60.0, th=None, sig=1.0, sig_th=3.0, why="re-seed", t0=1.0)
    n0 = len(out)
    c.fpose_pending_watch(rA)
    assert rA.fpose_pending is None and not any("fpose" in t for n, t in out[n0:])


# -- waking sleeping robots at a run's start (the 13 Nov 08:14 home run: none found, all asleep) ----------------------
class _FakeLink:
    def __init__(self):
        self.heard, self.sent = set(), []

    def robots(self):
        return sorted(self.heard)

    def send_name(self, n, c):
        self.sent.append((n, c))
        return True

    def send_all(self, c):
        self.sent.append(("*", c))

    def send_ip(self, ip, c):
        self.sent.append((ip, c))


def test_the_waker_waits_one_check_in_says_when_and_wakes_each_robot_the_moment_it_is_heard(tmp_path):
    import wake_robots as WR
    sp = str(tmp_path / "sleep_state.json")
    clock = [1000.0]
    WR.note_sleep(None, 10, "the run ended", path=sp, now=1000.0)
    link = _FakeLink()
    clock[0] = 1000.0 + 7 * 60.0                        # 7 min after `sleep 10`: its check-in is ~3 min away
    w = WR.Waker(link, ["Left", "Center"], ["192.168.1.250"], say=lambda *a: None,
                 status_path=str(tmp_path / "wake.json"), sleep_path=sp, now=lambda: clock[0])
    assert w.limit == pytest.approx(10 * 60 + WR.AWAKE_S + WR.BOOT_S)
    assert "asleep, waking (next check-in in ~180 s)" == w.words("Left")
    w.step()
    assert ("255.255.255.255", "wake") in link.sent and ("192.168.1.250", "wake") in link.sent
    d = json.load(open(tmp_path / "wake.json"))
    assert d["robots"]["Left"]["words"].startswith("asleep, waking")
    link.heard.add("Left")                              # its check-in is heard: `wake` to it at once
    clock[0] += 180.0
    w.step()
    assert ("Left", "wake") in link.sent and not w.done() and not w.expired()
    clock[0] += 10 * 60.0 + 31.0
    assert w.expired()


def test_mission_control_says_asleep_waking_instead_of_none_found(tmp_path):
    import live_view as LV
    (tmp_path / "wake.json").write_text(json.dumps(dict(t=time.time(), robots={
        "Left": dict(words="asleep, waking (next check-in in ~120 s)", awake=False)})))
    rows = LV.from_freeroam(str(tmp_path), {"Left": "#123456"}, {})
    left = [q for q in rows if q["name"] == "Left"][0]
    assert left["waking"].startswith("asleep, waking") and "waking" in left["source"]
    html = open(os.path.join(FLEET, "mission_control_table.html")).read()
    assert "r.waking ? esc(r.waking)" in html


def test_a_free_roam_start_wakes_a_robot_asleep_and_it_joins(tmp_path):
    """The simulator (fleet_sim models deep sleep: deaf until its check-in, which is a fresh boot): Center starts asleep,
    checking in 36 s after the start; the free roam waits, wakes it at its check-in, gives it its compass check (a
    wake is a restart) and it drives with the others."""
    r = subprocess.run([sys.executable, os.path.join(FLEET, "freeroam.py"), "run", "--dry", "--hours", "0.025",
                        "--robots", "Left,Center,Right", "--sim-minutes", "30", "--sim-asleep", "Center:1:0.6",
                        "--dir", str(tmp_path / "run")], capture_output=True, text=True, timeout=600,
                       env=dict(os.environ, HOME=str(tmp_path), MIRLT_CONF_DIR=str(tmp_path / "conf")))
    out = r.stdout + r.stderr
    assert r.returncode == 0, out[-1500:]
    assert "Asleep? Waking them" in out and "Awake: " in out and "Center" in out.split("Awake: ")[1].split("\n")[0]
    assert "compass checked" in out
    assert "Center did not start following" not in out
    assert json.load(open(tmp_path / "run" / "wake.json"))["robots"]["Center"]["awake"]


# -- Mission Control (Nick 13 Nov 08:28): a status line, a free roam from the terminal shown, stale requests cleared ------
def test_a_free_roam_started_from_the_terminal_is_found_by_its_fresh_files(tmp_path, monkeypatch):
    import live_view as LV
    old = tmp_path / "20261112-100000-freeroam"
    new = tmp_path / "20261113-090000-freeroam"
    for d in (old, new):
        d.mkdir()
        (d / "follow.csv").write_text("t,robot\n")
    os.utime(old / "follow.csv", (time.time() - 3600, time.time() - 3600))
    monkeypatch.setenv("MIRLT_RUNS_GLOB", str(tmp_path / "*-freeroam*"))
    assert LV.terminal_freeroam_folder() == str(new)
    os.utime(new / "follow.csv", (time.time() - 60, time.time() - 60))
    assert LV.terminal_freeroam_folder() is None


def test_the_page_has_a_status_line_and_a_restart_button_always():
    html = open(os.path.join(FLEET, "mission_control_table.html")).read()
    assert 'id="statusline"' in html and 'id="restartmc2"' in html and "T.headline" in html
    assert "LIVE.terminal" in html
    src = open(os.path.join(FLEET, "start_job.py")).read()
    assert "def headline(self)" in src and '"headline": self.headline()' in src
    assert "freeroam-stop" in src                       # STOP reaches a free roam started from the terminal
    assert "earlier_runs_cleared" in src                # needs-a-hand requests of an earlier run are not shown


def test_no_reverse_while_its_own_frame_is_off_the_cameras():
    """(measured seeds 3 and 6: reverses off the edge with its own pose 16-34 cm from the camera's) the robot drives in
    its own frame: while its own pose and the camera disagree beyond its own filter's gate, its pose is unconfirmed (no
    reverse toward an edge)."""
    import types
    c, out, logs = _ctrl(("A",))
    r = _place(c, "A", 60.0, 60.0, 0.0)
    r.pfuse = types.SimpleNamespace(disagree_cm=20.0)
    assert c.pose_unconfirmed(r)
    r.pfuse = types.SimpleNamespace(disagree_cm=1.0)
    assert not c.pose_unconfirmed(r)


# -- (e22c, Nick 13 Nov ~10:00) two robots stood for a long time and boxed the third into a corner -------------------------
def _boxed(monkeypatch, on):
    import freeroam_fastsim as FS
    monkeypatch.setattr(FC, "BOX_ASK", on)
    # Left in the corner facing it; Center and Right standing (held: their heading not known) across its way out
    starts = {"Left": (13.0, 13.0, 225.0), "Center": (31.0, 12.0, 90.0), "Right": (12.0, 31.0, 0.0)}
    s = FS.Sim(1, 0.05, starts=starts, held={"Center": "heading", "Right": "heading"}, edge_physics=True,
               sensors=True)
    s.ctrl.plan_slowdown = 0.0
    out_t = []
    p0 = {n: (b.x, b.y) for n, b in s.bots.items()}
    moved = {"Center": 0.0, "Right": 0.0}
    otick = s.ctrl.tick

    def tick(t):
        res = otick(t)
        b = s.bots["Left"]
        if not out_t and math.hypot(b.x, b.y) > 45.0:
            out_t.append(t)
        for n in moved:
            moved[n] = max(moved[n], math.dist(p0[n], (s.bots[n].x, s.bots[n].y)))
        return res
    s.ctrl.tick = tick
    summ = s.run()
    return out_t, moved, summ, s.ctrl


def test_a_robot_boxed_into_a_corner_by_two_standing_robots_gets_out_without_a_person(monkeypatch, tmp_path):
    monkeypatch.setenv("HOME", str(tmp_path))
    out_t, moved, summ, c = _boxed(monkeypatch, True)
    assert out_t, "the cornered robot never got 45 cm out of the corner"
    assert max(moved.values()) > 10.0            # a standing robot was moved out of its way, not only Left
    assert summ.get("falls", 0) == 0 and summ.get("contacts", 0) == 0
    assert c.counts.get("room_made", 0) >= 1 and c.counts.get("room_asked_boxed", 0) >= 2   # both were asked


def test_the_boxed_corner_reproduced_with_the_old_way(monkeypatch, tmp_path):
    # MIRLT_BOX_ASK=0: only a lost robot asks, and only the nearest one on its line; held robots never move
    monkeypatch.setenv("HOME", str(tmp_path))
    out_t, moved, summ, c = _boxed(monkeypatch, False)
    assert not out_t and moved["Center"] < 1.0 and moved["Right"] < 1.0
