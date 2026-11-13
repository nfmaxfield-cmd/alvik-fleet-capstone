"""(e22: 3 test(s) describing the ways before e22 pin MIRLT_SELF_RESCUE=ask and MIRLT_ID_GUARD=0 through the e22_old_ways fixture in conftest.py.) e08 Phase A (Nick 7 Nov): edge safety and shared watches. One test group per decision A1-A11.

    python3 -m pytest tools/fleet/tests/test_e08_phase_a.py -q
"""
import json
import math
import os
import re
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(FLEET))
for p in (FLEET, os.path.join(REPO, "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)

import freeroam_core as FC       # noqa: E402
import observe as OB             # noqa: E402
import safety_rules as SR        # noqa: E402

BOARD = (0.0, 0.0, 122.24, 121.92)


def ctrl(names=("Left", "Center", "Right"), **kw):
    sent, rows = [], []
    c = FC.Controller(list(names), BOARD, lambda n, t: sent.append((n, t)), lambda tb, r: rows.append((tb, r)),
                      seed=3, **kw)
    return c, sent, rows


def place(c, name, x, y, th, t=0.0, fw="2026-11-05d"):
    c.t = t
    c.on_fw(name, fw)
    c.on_heard(name, t)
    c.on_battery(name, 100, t)
    c.on_follow(name, t, "idle", x, y, th, 0)
    c.on_fix(name, t, x, y, th)
    c.begin(name)


def table(rows, name):
    return [r for t, r in rows if t == name]


def events(rows, kind):
    return [r for t, r in rows if t == "events" and r["kind"] == kind]


# ------------------------------------------------------------------------------------------------ A1 fused position
def test_a1_disagreement_widens_at_once_not_after_2s():
    pf = SR.PoseFuse("widen")
    pf.feed(0.0, (10.0, 10.0), (10.0, 10.0))
    assert pf.unc_extra() == 0.0
    pf.feed(0.1, (10.0, 14.6), (10.0, 10.0))           # 4.6 cm at once (6 Nov run 1)
    assert abs(pf.unc_extra() - 4.6) < 0.01
    pf.feed(0.2, (10.0, 10.2), (10.0, 10.0))           # agreeing again: comes down over DISAGREE_DECAY_S, not at once
    assert 0.2 < pf.unc_extra() < 4.6
    old = SR.PoseFuse("wait2s")
    old.feed(0.1, (10.0, 14.6), (10.0, 10.0))
    assert old.unc_extra() == 0.0                       # the old rule: nothing widened


@pytest.mark.usefixtures("e22_old_ways")   # (e22) the way before e22, kept behind its switches
def test_a1_controller_uncertainty_includes_the_disagreement_and_reseeds_without_stopping():
    c, sent, rows = ctrl(names=("Center",))
    place(c, "Center", 40.0, 40.0, 0.0)
    r = c.R["Center"]
    u0 = c.unc(r)
    for k in range(1, 30):
        t = 0.1 * k
        c.t = t
        c.on_follow("Center", t, "idle", 40.0, 46.0, 0.0, 0)
        c.on_fix("Center", t, 40.0, 40.0, 0.0)
    assert c.unc(r) >= u0 + 5.0
    assert table(rows, "pose")[0]["what"] == "doubt"
    assert any(t.startswith("fpose") for n, t in sent)
    assert not [t for n, t in sent if t == "halt"]      # re-seeded without a halt (the old rule halted)


def test_a1_wait2s_switch_keeps_the_old_rule():
    c, sent, rows = ctrl(names=("Center",), pose_rule="wait2s")
    place(c, "Center", 40.0, 40.0, 0.0)
    for k in range(1, 30):
        t = 0.1 * k
        c.t = t
        c.on_follow("Center", t, "idle", 40.0, 46.0, 0.0, 0)
        c.on_fix("Center", t, 40.0, 40.0, 0.0)
    assert events(rows, "pose re-seeded")              # observe.PoseAgree's 2 s rule, as before
    assert not table(rows, "pose")


# ------------------------------------------------------------------------------------------------ A2 the edge rule
def test_a2_derived_clearance_and_the_old_numbers_behind_the_switch():
    assert abs(SR.clearance_cm(1.0, 13.0, "lights") - (6.72 + 1.0 + 2.4 + 5.0)) < 1e-6
    assert abs(SR.clearance_cm(1.0, None, "tags", stops_here=False) - 7.72) < 1e-6
    assert SR.edge_room_cm("missions") == 15.8
    assert SR.edge_room_cm("school") == 10.8           # roof tags: no camera edge bias
    assert SR.edge_room_cm("ladder") == SR.edge_room_cm("doe") == 18.5
    for k, v in SR.FIXED_EDGE_CM.items():
        assert SR.edge_room_cm(k, rule="fixed") == v
    assert SR.stop_cm(13.0, {13.0: 3.0}) == 3.0        # a robot's own lead raises it, never lowers it
    assert SR.stop_cm(13.0, {13.0: 0.3}) == 2.4


def test_a2_whole_swept_path_and_the_shrink_rule():
    ok, _ = SR.path_allowed([(60, 60), (60, 30)], BOARD, 1.0, 10.0)
    assert ok
    ok, why = SR.path_allowed([(60, 30), (60, 20), (60, 12)], BOARD, 1.0, 10.0)   # a bulge into the clearance
    assert not ok and "clearance" in why
    ok, _ = SR.path_allowed([(60, 10), (60, 14), (60, 20)], BOARD, 1.0, 5.0, stop_at_end=False)   # inward: allowed
    assert ok
    ok, _ = SR.path_allowed([(60, 10), (60, 9)], BOARD, 1.0, 5.0)                 # outward from inside: refused
    assert not ok


def test_a2_6nov_move46_refused_under_the_derived_rule_taken_under_the_old_one():
    """Center at the end of move 45: the camera 8.7 cm from y = 0, its own pose 4.6 cm further in; move 46 was a
    -200 deg curve (radius 11 cm) planned from the stale end."""
    tr = FC.curve_traj(0.2, [(18.6, 12.5, 163.0, -0.09123, 38.2, 8.75)], -1)
    new, _, _ = ctrl(names=("Center",))
    place(new, "Center", 17.5, 8.7, 163.0)
    r = new.R["Center"]
    r.pfuse = SR.PoseFuse("widen")
    r.pfuse.feed(0.0, (18.6, 12.5), (17.5, 8.7))
    assert not new.edge_ok(r, tr, 8.75, stops=False)
    old, _, _ = ctrl(names=("Center",), edge_rule="fixed", pose_rule="wait2s")
    place(old, "Center", 18.6, 12.5, 163.0)
    assert old.curve_ok(old.R["Center"], tr)


def test_a2_sure_unsure_judgement():
    assert SR.edge_judgement(2.0, True, 0.5)["state"] == "clear"
    assert SR.edge_judgement(-2.0, True, 1.0)["state"] == "sure_near"
    assert SR.edge_judgement(-2.0, True, 4.6)["state"] == "unsure"
    assert SR.edge_judgement(-2.0, False, None)["state"] == "unsure"
    assert SR.edge_judgement(-2.0, False, None, tilted=True)["state"] == "sure_near"
    assert SR.inward_agreed(BOARD, (10, 60), (11, 60), 180.0, reverse=True)       # backing goes +x: inward
    assert not SR.inward_agreed(BOARD, (10, 60), (11, 60), 180.0, reverse=False)  # forward: outward, never
    assert not SR.inward_agreed(BOARD, (10, 60), None, 0.0)                         # no camera: not agreed


def test_a2_questions_hook_round_trip(tmp_path):
    q = SR.Questions(run="t", folder=str(tmp_path))
    qid = q.ask("Center", "Is it near the edge?")
    assert [x["id"] for x in SR.open_questions(str(tmp_path))] == [qid]
    assert q.answer(qid) is None
    SR.write_answer(qid, "It is near the edge", str(tmp_path))
    assert q.answer(qid) == "It is near the edge"
    assert SR.open_questions(str(tmp_path)) == []


@pytest.mark.usefixtures("e22_old_ways")   # (e22) the way before e22, kept behind its switches
def test_a2_unsure_attended_asks_and_holds_alone_moves_inward_or_holds(tmp_path):
    for attended in (True, False):
        asks = SR.Questions(run="t", folder=str(tmp_path / ("a" if attended else "b")))
        c, sent, rows = ctrl(names=("Center",), attended=attended, questions=asks)
        place(c, "Center", 9.0, 60.0, 180.0)            # 9 cm from x = 0, facing it
        r = c.R["Center"]
        for k in range(1, 6):
            t = 0.1 * k
            c.t = t
            c.on_follow("Center", t, "idle", 14.0, 60.0, 180.0, 0)    # its own pose 5 cm further in: unsure
            c.on_fix("Center", t, 9.0, 60.0, 180.0)
        c.t = 1.0
        c.edge_watch(r)
        assert r.edge_state == "unsure"
        r.busy_until = 0.0
        c.unsure_step(r)
        if attended:
            assert SR.open_questions(asks.dir) and r.move is None and r.stopped_hard
        else:
            assert r.move is not None and r.move["rev"]          # backwards = +x: inward by both, never forward


# ------------------------------------------------------------------------------------------------ A3 tilt recovery
def test_a3_tries_never_towards_the_drop():
    assert SR.tilt_recovery_tries(180.0, 0.0, True)[:3] == ["reverse", "reverse_left", "reverse_right"]
    t = SR.tilt_recovery_tries(117.0, 0.0, False)
    assert t[0].startswith("pivot") and t[-1] == "hold" and "reverse" not in t


def test_a3_tilted_robot_backs_out_inward_even_with_no_general_room_then_pivots_with_the_limit_restored():
    c, sent, rows = ctrl(names=("Center",))
    place(c, "Center", 5.4, 18.6, 117.0)
    r = c.R["Center"]
    for k in range(OB.LEVEL_N):
        c.on_raw("Center", 0.01 * k, 0.0, 0.0, 0.0)
    for k in range(150):
        c.on_raw("Center", 1.0 + 0.02 * k, 6.1, 1.0, 0.0)
    assert r.tilted
    c.t = 5.0
    c.on_fix("Center", 5.0, 5.4, 18.6, 117.0)
    r.mode, r.busy_until = "idle", 0.0
    c.start_move(r)
    assert r.move is not None and r.move["kind"].startswith("recover:") and (r.move["rev"] or r.move["L"] == 0)
    rows_ = table(rows, "recovery")
    assert rows_ or r.move["kind"].startswith("recover:")
    # a pivot raises the tilt stop for its step only
    c2, sent2, rows2 = ctrl(names=("Center",))
    place(c2, "Center", 3.0, 60.0, 90.0)
    r2 = c2.R["Center"]
    for k in range(OB.LEVEL_N):
        c2.on_raw("Center", 0.01 * k, 0.0, 0.0, 0.0)
    for k in range(150):
        c2.on_raw("Center", 1.0 + 0.02 * k, 6.0, 0.0, 0.0)
    r2.mode, r2.busy_until = "idle", 0.0
    r2.recover = dict(tries=["pivot_left", "hold"], i=0, n=1, inward=0.0, back_in=False, trail=False)
    c2.recover_step(r2)
    raised = [t for n, t in sent2 if t.startswith("set tilt_stop_deg")]
    assert raised and float(raised[-1].split()[-1]) > SR.tilt_limits()["tilt_stop_deg"]
    c2.end_move(r2)
    assert [t for n, t in sent2 if t.startswith("set tilt_stop_deg")][-1] == "set tilt_stop_deg %g" % \
        SR.tilt_limits()["tilt_stop_deg"]


def test_a3_sim_hung_robot_recovers_and_the_old_rule_stands():
    import rehearse_phase_a as RH
    jobs = {j[0]: j for j in RH.case_runs("b", 0.03) if j[1] == 1}
    after = RH.one(jobs["b-after"])
    assert after["recovered_t"] and after["falls"] == 0


# ------------------------------------------------------------------------------------------------ A4 tilt limits
def test_a4_tilt_limits_from_the_logs_sent_as_runtime_params():
    lim = SR.tilt_limits()
    assert lim["tilt_stop_deg"] == 4.0 and lim["tilt_ms"] == 300.0
    assert OB.TILT_STOP_DEG == 4.0 and OB.STAND_TILT_DEG == 2.5
    assert "set tilt_stop_deg 4" in OB.safety_commands(3)
    assert SR.TILT_LIMITS_OLD["tilt_stop_deg"] == 5.0


# ------------------------------------------------------------------------------------------------ A5 shared watches
def test_a5_heading_watch_resets_a_wrong_own_heading():
    c, sent, rows = ctrl(names=("Center",))
    place(c, "Center", 30.0, 60.0, 0.0)
    r = c.R["Center"]
    r.move = dict(kind="menu", phase="drive", rev=False, curve=False, turn=0.0, L=40.0, v=5.0, t0=0.0, n=1)
    t, x = 0.0, 30.0
    for k in range(80):                     # it drives along +y (its fixes keep its position) while its own heading
        t += 0.1                            # says 0 (+x): the heading lost after a restart
        c.t = t
        c.on_follow("Center", t, "drive", 30.0, 60.0 + 0.5 * k, 0.0, 1)
        c.on_fix("Center", t, 30.0, 60.0 + 0.5 * k, None)
    assert table(rows, "heading") and any(e["kind"] == "heading re-set" for e in table(rows, "events"))


def test_a5_stop_lead_surface_mark_and_border_fire():
    c, sent, rows = ctrl(names=("Center",), leads={"Center": {5.0: 0.5, 10.0: 0.8}})
    place(c, "Center", 60.0, 60.0, 0.0)
    tr = FC.Traj.move(0.2, (60.0, 60.0), 0.0, 0.0, 20.0, 10.0)
    r = c.R["Center"]
    c.start_move_res = None
    res = ("menu", None, 0.0, 20.0, 10.0, tr)
    r.mode = "idle"
    orig = c.pick
    c.pick = lambda rr: res
    c.start_move(r)
    c.pick = orig
    d = [t for n, t in sent if t.startswith("fs") and " D " in t][-1].split()
    assert abs(float(d[5]) - 79.2) < 0.05         # 20 cm sent 0.8 cm short
    # a line trip with the camera well inside: a surface mark, logged, no keep-out
    n0 = len(c.keepouts)
    c.felt(r, "plywood edge at 60.0 60.0 0.0 line")
    assert table(rows, "surface_marks")[-1]["kind"] == "surface_mark" and len(c.keepouts) == n0
    c.border_fire(r, "virtual border at 6 60 0 5.9")
    assert c.counts["border_fires"] == 1


# ------------------------------------------------------------------------------------------------ A6 keep-outs
def test_a6_unconfirmed_fade_confirmed_stay_one_rule():
    assert not SR.keepout_confirmed("front sensor")
    assert SR.keepout_confirmed("front sensor", felt_before=True)
    assert SR.keepout_confirmed("edge guard") and SR.keepout_confirmed("tilt") and SR.keepout_confirmed("hazard")
    assert OB.trip_keepout(10, 10, 0)["confirmed"] is True
    import front_stop as FS
    assert FS.UNCONFIRMED_S == SR.UNCONFIRMED_S
    c, sent, rows = ctrl(names=("Center",))
    place(c, "Center", 60.0, 60.0, 0.0)
    r = c.R["Center"]
    c.felt(r, "obstacle 5.0 cm ahead")
    assert len(c.keepouts) == 1 and not c.keepouts[0]["confirmed"]
    c.t = 16.0
    c.clear_keepouts()
    assert not c.keepouts and table(rows, "keepouts")[-1]["what"].startswith("faded")
    c.felt(r, "obstacle 5.0 cm ahead")                  # felt again at the same place: confirmed, stays
    assert c.keepouts[-1]["confirmed"]
    c.t = 60.0
    c.clear_keepouts()
    assert c.keepouts
    cp, _, _ = ctrl(names=("Center",), keepout_rule="permanent")
    place(cp, "Center", 60.0, 60.0, 0.0)
    cp.felt(cp.R["Center"], "obstacle 5.0 cm ahead")
    cp.t = 100.0
    cp.clear_keepouts()
    assert cp.keepouts                                  # the old way


# ------------------------------------------------------------------------------------------------ A7 no sitting out
def test_a7_silent_robot_held_then_rejoins_old_switch_out():
    for out in (False, True):
        c, sent, rows = ctrl(names=("Center",), silent_out=out)
        place(c, "Center", 60.0, 60.0, 0.0)
        r = c.R["Center"]
        t = 0.0
        while t < 130.0:                                 # silent: nothing heard, the camera still sees it
            t += 0.5
            c.on_fix("Center", t, 60.0, 60.0, 0.0)
            c.tick(t)
        if out:
            assert r.mode == "out"
            continue
        assert r.mode != "out" and r.held and r.held["kind"] == "silent"
        while t < 140.0:                                 # heard again, its own pose agrees with the camera
            t += 0.1
            c.on_heard("Center", t)
            c.on_follow("Center", t, "idle", 60.0, 60.0, 0.0, 0)
            c.on_fix("Center", t, 60.0, 60.0, 0.0)
            c.tick(t)
        assert r.held is None and events(rows, "rejoined")


@pytest.mark.usefixtures("e22_old_ways")   # (e22) the way before e22, kept behind its switches
def test_a7_heading_held_robot_rejoins_after_a_camera_watched_nudge():
    import rehearse_phase_a as RH
    jobs = {j[0]: j for j in RH.case_runs("c", 0.03) if j[1] == 1}
    after = RH.one(jobs["c-after"])
    assert "Right" in after["rejoined_t"]


# ------------------------------------------------------------------------------------------------ A8 keep going
def test_a8_keep_going_is_the_default_everywhere_stop_is_the_switch():
    import sites as SI
    assert SI.POLICY_DEFAULTS["on_problem"] == "keep_going"
    assert json.load(open(os.path.join(FLEET, "sites", "home.json")))["on_problem"] == "keep_going"
    src = open(os.path.join(FLEET, "run_plan.py")).read()
    assert re.search(r'"--on-problem", choices=\("stop", "keep_going"\), default="keep_going"', src)
    d = open(os.path.join(FLEET, "doe_loop.py")).read()
    assert '"--keep-going", action="store_true", default=True' in d and '"--no-keep-going"' in d
    assert 'or "keep_going"      # (e08, Q7)' in open(os.path.join(FLEET, "mission_control.py")).read()


# ------------------------------------------------------------------------------------------------ A9 E02 runner
def test_a9_start_page_main_start_is_mission_control_e02_behind_a_switch():
    src = open(os.path.join(REPO, "tools", "pi", "launcher.py")).read()
    sys.path.insert(0, os.path.join(REPO, "tools", "pi"))
    import launcher as LA
    assert LA.OLD_RUNNER is False
    assert "Show the old runner" in src and "Open Mission Control" in src
    assert "old_runner" in src and os.path.exists(os.path.join(REPO, "run-e02-pi.sh"))   # nothing deleted


# ------------------------------------------------------------------------------------------------ A10 superseded
def test_a10_calib_sleeps_at_end_from_the_script_and_monday_demo_switched_off():
    sh = open(os.path.join(REPO, "run-fleet-pi.sh")).read()
    assert 'SLP="--sleep-at-end"' in sh and "awake) SLP=" in sh
    assert "MIRLT_MONDAY_DEMO" in sh
    import mission_control as MC
    assert MC.MONDAY_DEMO is False
    import missions as MS
    assert MS.monday_program()                           # kept, not deleted


# ------------------------------------------------------------------------------------------------ A11 battery
def test_a11_one_battery_floor_per_run_type():
    import engine_checks as EC
    assert EC.battery_floor({}) == 25
    assert EC.battery_floor({}, run_type="freeroam") == 10
    assert EC.battery_floor({"MIRLT_MIN_BATTERY": "7"}, run_type="freeroam") == 7
    assert FC.BATTERY_FLOOR == 10
