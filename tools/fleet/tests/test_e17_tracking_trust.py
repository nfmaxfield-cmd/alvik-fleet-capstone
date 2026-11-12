"""Build e17 (12 Nov): the tracking layer observes, then trusts (the home 11 Nov runs: Left held 84 min 48 cm from where
the camera saw it standing), corrections reach the robot, late fixes and doubtful headings, never still (Nick 12 Nov),
the core layout, the ML hook, frames at 4 a second, a robot with no readable points. No robot, no Pi."""
import json
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "analysis")):
    if p not in sys.path:
        sys.path.insert(0, p)

import e17_harness as H        # noqa: E402
import freeroam_core as FC     # noqa: E402
import tracking_layer as TL    # noqa: E402

THREE = {"Left": (31.0, 64.0, 0.0), "Center": (90.0, 30.0, 90.0), "Right": (90.0, 95.0, 180.0)}


# -- 1a / 1d / 1e: a standing robot whose own pose is far off, the camera steady ----------------------------------------
@pytest.mark.parametrize("d", [10.0, 20.0, 30.0, 48.0])
def test_a_standing_robot_far_from_its_own_pose_is_corrected_from_the_camera_within_seconds(d, monkeypatch):
    monkeypatch.setattr(FC, "MOTION_PROOF", False)   # (e19) e17's way: elimination at once, any distance;
    # with motion proof (the default since e19): test_e19_audit_fixes.py
    R = H.Rig(THREE, own_off={"Left": (d * 0.6, d * 0.8, 0.0)})
    R.run(10.0)
    c = R.first("CORRECTED", "Left")
    assert c is not None and c[0] <= 3.0                       # ~1 s steady + the fix's 0.45 s delay
    assert R.gap("Left") < 3.0 and R.own_gap("Left") < 3.0      # the robot's own pose was set (fpose, not refused)
    assert R.fw_refused["Left"] == 0
    assert "NEEDS A HAND" not in R.kinds() and "REJOINED" not in R.kinds()
    assert "correction revoked" not in R.kinds()                 # its own pose's jump (the fpose) is not a drive
    assert any(t == "fpose" for _, n, t in [(s[0], s[1], s[2].split()[0]) for s in R.sent] if n == "Left")


@pytest.mark.parametrize("d", [20.0, 30.0, 48.0])
def test_the_old_way_never_takes_it(d):
    R = H.Rig(THREE, own_off={"Left": (d * 0.6, d * 0.8, 0.0)}, eliminate=False, gate_record=False)
    R.run(10.0)
    assert R.first("CORRECTED", "Left") is None and R.gap("Left") > d - 3.0


def test_left_at_1139_lost_with_its_own_pose_48_cm_off_is_corrected_not_held(monkeypatch):
    monkeypatch.setattr(FC, "MOTION_PROOF", False)   # (e19) e17's way: elimination at once, any distance;
    # with motion proof (the default since e19): test_e19_audit_fixes.py
    """Left, 11:39:16: the camera saw it standing at (31, 64); its own pose said (66, 98), 24 cm from the y = 122 edge."""
    R = H.Rig({"Left": (31.2, 64.2, -115.0), "Center": (95.0, 30.0, 90.0), "Right": (95.0, 70.0, 180.0)},
              own_off={"Left": (66.1 - 31.2, 97.9 - 64.2, 60.0)})
    R.run(0.1)
    R.c.start_lost(R.c.R["Left"], "test: the camera lost it")
    R.run(20.0)
    c = R.first("CORRECTED", "Left")
    assert c is not None and c[0] < 3.0 and R.gap("Left") < 3.0
    assert R.c.R["Left"].lost is None and "correction revoked" not in R.kinds("Left")
    assert R.c.R["Left"].__dict__.get("hd_confirm") is not None or R.first("heading confirmed", "Left") or \
        R.first("heading set", "Left") or R.first("heading not confirmed", "Left")   # a big jump asks for its heading
    assert "REJOINED" not in R.kinds("Left")
    corr = [l for l in R.logs if l.get("table") == "corrections"]
    assert corr and abs(corr[0]["d_cm"] - 48.6) < 3.0


def test_a_held_robot_that_needed_a_hand_is_CORRECTED_never_REJOINED(monkeypatch):
    monkeypatch.setattr(FC, "MOTION_PROOF", False)   # (e19) e17's way: elimination at once, any distance;
    # with motion proof (the default since e19): test_e19_audit_fixes.py
    c = FC.Controller(["Left"], H.BOARD, lambda n, s: None, lambda *a: None, pose_source="blend")
    c.never_still = False
    r = c.R["Left"]
    for k in range(10):
        c.t = 0.1 * k
        c.on_follow("Left", c.t, "idle", 66.0, 98.0, 0.0)
        c.on_fix("Left", c.t, 66.0, 98.0, None)
    c.begin("Left")
    r.unsure = dict(t=c.t, why="test", tried=0)
    c.t = 10.0
    c.need_hand_watch(r)                                         # it checks again itself first ...
    c.t = 15.0
    c.need_hand_watch(r)                                         # ... then asks
    assert c.needs_hand()
    c.on_follow("Left", 15.1, "idle", 66.0, 98.0, 0.0)
    c.t = 15.6
    c.on_fix("Left", 15.2, 31.0, 64.0, None, arrival=15.6, how="elimination")
    r.unsure = None                                              # (edge_watch: the corrected spot is clear)
    ev = []
    c.log = lambda table, row: ev.append(row) if table == "events" else None
    c.need_hand_watch(r)
    assert not c.needs_hand()
    assert c.counts.get("corrected_rejoins") == 1 and not c.counts.get("hand_rejoined")
    assert any(e["kind"] == "CORRECTED" and "nobody moved" in e["text"] for e in ev)


def test_the_switch_restores_the_e16_way(monkeypatch):
    monkeypatch.setattr(FC, "CAMERA_CORRECT", False)
    c = FC.Controller(["Left"], H.BOARD, lambda n, s: None, lambda *a: None, pose_source="blend")
    c.on_follow("Left", 0.0, "idle", 66.0, 98.0, 0.0)
    c.on_fix("Left", 0.0, 31.0, 64.0, None, how="elimination")
    assert not c.counts.get("camera_corrections")


# -- 1b: two robots unaccounted -> a wiggle, no swap ------------------------------------------------------------------
def test_two_robots_unaccounted_a_wiggle_names_one_then_elimination_the_other_no_swap():
    R = H.Rig({"Left": (31.0, 64.0, 0.0), "Center": (95.0, 100.0, 90.0), "Right": (85.0, 40.0, 180.0)},
              own_off={"Left": (40.0, -20.0, 0.0), "Right": (-40.0, 20.0, 0.0)})     # each own pose near the other
    R.run(20.0)
    assert R.first("identity wiggle", "Left") is not None
    assert any("named by wiggle" in str(l.get("what")) for l in R.logs)
    assert R.gap("Left") < 3.0 and R.gap("Right") < 3.0                           # neither took the other's spot
    assert R.layer.counts["elim_ambiguous"] >= 1


def test_with_no_wiggle_possible_nothing_is_guessed():
    R = H.Rig({"Left": (31.0, 64.0, 0.0), "Center": (95.0, 100.0, 90.0), "Right": (85.0, 40.0, 180.0)},
              own_off={"Left": (40.0, -20.0, 0.0), "Right": (-40.0, 20.0, 0.0)}, wiggle=False)
    R.run(12.0)
    assert R.first("CORRECTED") is None
    assert any(l.get("what") == "elimination unsure" for l in R.logs)


def test_a_spot_another_robot_can_claim_is_never_eliminated():
    L = TL.TrackingLayer(["A", "B"], ("motion", "lights"), board=H.BOARD)
    L.seed("A", 0.0, 50.0, 50.0)
    L.seed("B", 0.0, 90.0, 90.0)
    named = []
    for k in range(1, 60):
        t = k * 0.1
        L.own("A", t, 50.0, 50.0, 0.0, True)
        L.own("B", t, 20.0, 20.0, 0.0, True)        # B's own pose far off; its true spot is beside A's
        dets = [TL.Det("lights", t, 50.0, 50.0)] + ([TL.Det("lights", t, 60.0, 50.0)] if t > 1.0 else [])
        named += [f for f in L.feed(t, dets) if f["robot"] == "B"]
    assert not named                                  # 10 cm from A: inside CLOSE_CM of A, never B by elimination


def test_an_elimination_is_revoked_when_the_spot_stays_while_the_robot_drives():
    L = TL.TrackingLayer(["A"], ("motion", "lights"), board=H.BOARD)
    L.seed("A", 0.0, 30.0, 30.0)
    got = []
    for k in range(1, 120):
        t = k * 0.1
        x = 30.0 if t < 3.0 else 30.0 + (t - 3.0) * 5.0        # it stands, then drives away (its own wheels)
        L.own("A", t, x, 30.0, 0.0, t < 3.0)
        dets = [TL.Det("lights", t, 70.0, 70.0)]                  # a brick-like steady spot, never the robot
        got += [f for f in L.feed(t, dets) if f.get("how")]
    assert got and L.counts["elim_revoked"] >= 1
    assert L.take_revoked() and L.not_robot


# -- 1c: the gate from the measured record ------------------------------------------------------------------------
def test_the_gate_starts_at_the_e09_numbers_and_grows_with_the_measured_record():
    L = TL.TrackingLayer(["A"], ("motion",), board=H.BOARD)
    assert L.gate_max("A") == TL.GATE_MAX_CM and L.drift("A") == (TL.GATE_PER_CM, 0.0)
    for k in range(8):
        L.record("A", float(k), 30.0 + k, 20.0)
    per, p90 = L.drift("A")
    assert p90 >= 35.0 and per > 1.0 - 1e-9 or per >= 1.5 - 1e-9 or per == 1.0
    assert L.gate_max("A") > TL.GATE_MAX_CM and L.gate_max("A") <= 0.5 * 121.92 + 1e-6    # the board caps it
    L2 = TL.TrackingLayer(["A"], ("motion",), board=H.BOARD, gate_record=False)
    for k in range(8):
        L2.record("A", float(k), 30.0, 20.0)
    assert L2.gate_max("A") == TL.GATE_MAX_CM


# -- the root cause found on the logs: a re-seed read as a drive at the Pi's picture rate --------------------------------
def test_a_reseed_step_at_4_pictures_a_second_is_a_reset_not_a_drive(monkeypatch):
    def resets(rule):
        monkeypatch.setattr(TL, "OWN_RESET_RULE", rule)
        L = TL.TrackingLayer(["A"], ("motion",), eliminate=False)
        L.seed("A", 0.0, 50.0, 50.0)
        t, x = 0.0, 50.0
        for k in range(8):                     # driving 10 cm/s, own pose sampled once a picture (4 a second)
            t, x = t + 0.25, x + 2.5
            L.own("A", t, x, 50.0, 0.0, False)
        L.own("A", t + 0.25, x + 2.5 - 10.8, 50.0, 0.0, False)      # Right, 08:35:47: a 10.8 cm re-seed
        return L.counts.get("own_resets", 0)
    assert resets("speed") == 0              # e10's 5 cm + 30 cm/s x 0.25 s = 12.5 cm: read as a drive
    assert resets("velocity") == 1


def test_the_0826_replay_names_right_within_seconds_where_the_old_layer_waited_28_s(monkeypatch):
    import layer_replay as LR
    d = os.path.join(HERE, "data", "e17_home_0826_right_hold")
    monkeypatch.setattr(TL, "OWN_RESET_RULE", "speed")
    old = LR.replay(d, lag_s=0.0, e17=False, at_pictures=True)
    assert old["unnamed_longest"]["Right"]["longest_s"] > 20.0          # the run's 08:35:47 hold
    new = LR.replay(d, lag_s=0.0, e17=True, at_pictures=True)
    assert new["unnamed_longest"]["Right"]["longest_s"] < 10.0 and not new["wrong_name"]
    assert any(x["robot"] == "Right" and x["how"] == "elimination" and x["bridge_name"] == "Right"
               for x in new["taken"])
    monkeypatch.setattr(TL, "OWN_RESET_RULE", "velocity")
    both = LR.replay(d, lag_s=0.0, e17=True, at_pictures=True)
    assert both["unnamed_longest"]["Right"]["longest_s"] < 5.0


# -- 1e: a re-seed never lands mid-turn, and is projected ------------------------------------------------------------
def test_a_reseed_waits_for_the_turn_to_end_and_is_projected_to_now():
    sent = []
    c = FC.Controller(["A"], H.BOARD, lambda n, s: sent.append(s), lambda *a: None, pose_source="blend")
    r = c.R["A"]
    for k in range(10):                       # turning 40 deg/s on its spot
        c.t = 0.1 * k
        c.on_follow("A", c.t, "turn", 60.0, 60.0, 4.0 * k)
    assert c.send_fpose(r, 0.5, 70.0, 60.0, None) is False and not any(s.startswith("fpose") for s in sent)
    for k in range(10, 20):                   # it stops turning and drives 10 cm/s along +x
        c.t = 0.1 * k
        c.on_follow("A", c.t, "drive", 60.0 + (k - 10), 60.0, 36.0)
        c.tick(c.t)
    fp = [s for s in sent if s.startswith("fpose")]
    assert len(fp) == 1
    x = float(fp[0].split()[1])
    assert x > 70.5                         # the picture's spot moved on by its own driving since (projected)


# -- 2: late fixes and doubtful headings ----------------------------------------------------------------------------
def _driving(rate_rec=True, age=1.2):
    sent = []
    c = FC.Controller(["A", "B"], H.BOARD, lambda n, s: sent.append(s), lambda *a: None, pose_source="blend")
    c.never_still = False
    for n in "AB":
        c.on_fw(n, "2026-11-05c")
    r = c.R["A"]
    for k in range(40):
        c.t = 0.1 * k
        c.on_follow("A", c.t, "drive", 30.0 + 1.0 * k, 112.0, 0.0)
        c.on_follow("B", c.t, "idle", 100.0, 30.0, 0.0)
        if k % 2 == 0:
            c.on_fix("B", c.t, 100.0, 30.0, None, arrival=c.t + 0.2)
    c.on_fix("A", c.t - age, 30.0 + 10 * (c.t - age), 112.0, None, arrival=c.t)
    if rate_rec:
        for k in range(4):
            c.drift_add(r, 12.0, 20.0, "test")                    # it drifts 0.6 cm per cm (a heading error)
    r.mode = "move"
    r.move = dict(kind="curve", phase="drive", t_drive=0.5, v=10.0, curve=True, k=0.0, L=60.0, x=30.0, y=112.0,
                  h=0.0, end=(90.0, 112.0))
    return c, r, sent


def test_late_fixes_and_a_measured_drift_slow_or_stop_a_drive_near_an_edge():
    c, r, sent = _driving()
    c.late_fix_watch(r)
    assert r.__dict__.get("late") is not None and any(s.startswith("fspd") for s in sent)
    assert float([s for s in sent if s.startswith("fspd")][-1].split()[1]) < 1.0
    c.plan_next(r)                                                 # nothing queued meanwhile
    assert not r.queue


def test_with_the_prior_drift_rate_a_drive_in_the_open_is_not_slowed():
    c, r, sent = _driving(rate_rec=False, age=0.5)
    r.move.update(y=60.0)
    c.on_fix("A", c.t - 0.1, 66.0, 60.0, None, arrival=c.t)
    for k in range(5):
        c.on_follow("A", c.t + 0.01 * k, "drive", 66.0, 60.0, 0.0)
    c.late_fix_watch(r)
    assert r.__dict__.get("late") is None


def test_after_a_heading_reset_the_next_move_is_a_crawl_the_camera_confirms():
    R = H.Rig({"Left": (30.0, 30.0, 0.0), "Center": (60.0, 60.0, 0.0), "Right": (95.0, 95.0, 180.0)},
              own_off={"Center": (0.0, 0.0, 30.0)})
    R.run(2.0)
    r = R.c.R["Center"]
    r.busy_until = 0.0
    r.hd_confirm = dict(t=R.t, why="test", tries=0, stage=None)

    def park_when_done(R_):
        if R_.c.R["Center"].__dict__.get("hd_confirm") is None:
            R_.c.R["Center"].busy_until = 1e9
    R.run(25.0, hook=park_when_done)
    assert R.first("heading set", "Center") is not None and R.first("heading confirmed", "Center") is not None
    assert abs(R.off["Center"][2]) < FC.HD_CONFIRM_DEG + 1.0


def test_a_doubtful_heading_next_to_a_robot_waits_for_room_never_touches():
    R = H.Rig({"Center": (60.0, 60.0, 0.0), "Right": (80.0, 60.0, 180.0)}, own_off={"Center": (0.0, 0.0, 70.0)})
    R.run(2.0)
    R.c.R["Center"].busy_until = 0.0
    R.c.R["Center"].hd_confirm = dict(t=R.t, why="test 70 deg", tries=0, stage=None)
    mind = [99.0]
    R.run(10.0, hook=lambda R_: mind.__setitem__(0, min(mind[0], math.dist(R_.truth["Center"][:2],
                                                                           R_.truth["Right"][:2]))))
    assert R.first("heading check waits", "Center") is not None
    assert mind[0] >= 2 * FC.HALF + FC.REFLEX_GAP - 1e-6


# -- Nick 12 Nov: never still -------------------------------------------------------------------------------------
def test_an_unsure_robot_near_an_edge_keeps_creeping_toward_the_middle():
    for ns in (False, True):
        R = H.Rig({"Left": (60.0, 113.0, 90.0), "Center": (100.0, 30.0, 90.0), "Right": (20.0, 30.0, 180.0)},
                  never_still=ns)
        R.run(2.0)
        r = R.c.R["Left"]
        r.busy_until = 0.0
        r.unsure = dict(t=R.t, why="test: unsure near an edge", tried=0)
        R.run(15.0)
        if ns:
            assert R.c.counts.get("creeps", 0) >= 3
            assert R.longest_still("Left", 3.0) <= R.c.still_pause(r) + 1.0
            assert R.truth["Left"][1] < 113.0 - 10.0                     # toward the middle, away from the edge
        else:
            assert not R.c.counts.get("creeps")                        # the switch: e16's holds and inward steps


def _readable_ws():
    """A readable area x < 70 cm (school-like) inside the board: the proven workspace limited to it."""
    import proven_workspace as PW
    ws = PW.ProvenWorkspace(H.BOARD)
    ws.see(H.BOARD)
    ws.limit_to(lambda x, y: x < 70.0, "readable")
    return ws


def test_a_robot_outside_the_readable_area_creeps_back_into_it():
    R = H.Rig({"Left": (95.0, 60.0, 0.0), "Center": (30.0, 30.0, 90.0), "Right": (30.0, 95.0, 180.0)},
              never_still=True)
    R.c.ws = _readable_ws()
    R.run(30.0)
    assert R.truth["Left"][0] < 70.0
    assert R.c.counts.get("creeps", 0) >= 1


def test_no_robot_stands_longer_than_the_derived_pause_while_held():
    R = H.Rig({"Left": (60.0, 113.0, 90.0), "Center": (100.0, 20.0, 90.0), "Right": (12.0, 60.0, 180.0)},
              never_still=True)
    R.run(1.0)
    R.c.R["Left"].unsure = dict(t=R.t, why="test", tried=0)
    R.c.start_lost(R.c.R["Center"], "test")
    R.run(30.0)
    for n in R.names:
        assert R.longest_still(n, 2.0) <= R.c.still_pause(R.c.R[n]) + 1.5, n


def test_never_still_is_said_in_claude_md_and_switchable(monkeypatch):
    txt = open(os.path.join(os.path.dirname(TOOLS), "CLAUDE.md")).read()
    assert "Never still (Nick, 2026-11-12)" in txt and "MIRLT_NEVER_STILL=0" in txt
    monkeypatch.setattr(FC, "NEVER_STILL", False)
    c = FC.Controller(["A"], H.BOARD, lambda n, s: None, lambda *a: None)
    assert c.never_still is False


# -- 4: the ML detector on demand --------------------------------------------------------------------------------------
def test_the_detector_runs_only_for_robots_that_need_it_at_a_capped_rate():
    calls = []

    def fn(name, t, why, around):
        calls.append((name, t, why))
        return [TL.Det("ml", t, 70.0, 70.0)] if name == "B" else []
    L = TL.TrackingLayer(["A", "B"], ("motion",), ml_fn=fn, ml_on_demand=True, board=H.BOARD)
    L.seed("A", 0.0, 30.0, 30.0)
    L.seed("B", 0.0, 90.0, 90.0)
    for k in range(1, 50):
        t = k * 0.1
        L.own("A", t, 30.0, 30.0, 0.0, True)
        L.own("B", t, 90.0, 90.0, 0.0, True)
        L.feed(t, [TL.Det("motion", t, 30.0, 30.0)])              # A seen every picture; B never
    assert calls and all(n == "B" for n, _, _ in calls)
    ts = [t for _, t, _ in calls]
    assert all(b - a >= 1.0 / TL.ML_ROBOT_HZ - 1e-6 for a, b in zip(ts, ts[1:]))
    assert L.counts["ml_dets"] >= 1
    off = TL.TrackingLayer(["A"], ("motion",), ml_fn=fn)        # default: off
    assert off.ml_on is False


# -- 3: the core layout, the bench ------------------------------------------------------------------------------------
def test_the_core_layouts_and_the_bridge_prefix(monkeypatch):
    import core_layout as CL
    assert CL.DEFAULT == "B" and CL.layout(None) in ("A", "B", "C")
    assert CL.cores("planner", "A", ncpu=4) == "2,3" and CL.cores("bridge", "A", ncpu=4) == "2,3"
    assert CL.cores("planner", "B", ncpu=4) == "1" and CL.cores("bridge", "B", ncpu=4) == "2,3"
    assert CL.cores("recorder2", "B", ncpu=4) == "0" and CL.cores("planner", "C", ncpu=4) == ""
    assert CL.cores("planner", "B", ncpu=2) == ""
    assert CL.bridge_prefix({}) == []
    src = open(os.path.join(os.path.dirname(TOOLS), "run-fleet-pi.sh")).read()
    assert "layouts" in src and 'taskset -c 2,3"' not in src and "plan_pin" in src


def test_the_bench_reports_pictures_a_second_fix_delay_and_watchdog_stops(tmp_path):
    import pi_bench as PB
    d = tmp_path / "layout-B"
    (d / "real_bridge").mkdir(parents=True)
    with open(d / "system.csv", "w") as f:
        f.write("t,load1,busy_pct,iowait_pct,n_running,n_dstate,freq_mhz,temp_c,throttled,mem_avail_mb,cores\n"
                "0,2.0,150,1,2,0,1800,70,0x0,900,4\n")
    with open(d / "real_bridge" / "detect_times.csv", "w") as f:
        f.write("t,cam,ms,windows,full_searches,grey,found,mode,t_done\n")
        for k in range(40):
            f.write("%.3f,overhead,30,2,0,90,3,w,%.3f\n" % (k * 0.2, k * 0.2 + 0.25))
    (d / "practice_run").mkdir()
    with open(d / "practice_run" / "events.csv", "w") as f:
        f.write("t,robot,kind,text\n1,A,watchdog,its watchdog stopped the step\n")
    s = PB.summarise(str(d))
    assert abs(s["pictures_per_s"] - 40 / 7.8) < 0.1 and abs(s["lag_s_med"] - 0.25) < 1e-6
    assert s["watchdog_stops"] == 1
    assert "Pictures the real camera's bridge handled" in PB.report(str(d))


# -- Nick 12 Nov answers 1 and 4 --------------------------------------------------------------------------------------
def test_frames_are_kept_at_4_a_second_by_default(monkeypatch):
    import doe_loop as DL
    monkeypatch.delenv("MIRLT_FRAMES_HZ", raising=False)
    assert DL.frames_rates(12.0, 10.0) == (4.0, 4.0)
    monkeypatch.setenv("MIRLT_FRAMES_HZ", "12")
    assert DL.frames_rates(12.0, 10.0) == (12.0, 12.0)
    monkeypatch.setenv("MIRLT_FRAMES_HZ", "before")
    assert DL.frames_rates(12.0, 10.0) == (12.0, 10.0)


def test_a_robot_with_no_readable_points_goes_to_the_nearest_readable_one(monkeypatch):
    import mission_run as MR
    monkeypatch.setattr(MR, "say", lambda *a: None)
    pts = {"P01": (20.0, 20.0), "P02": (40.0, 20.0), "P03": (200.0, 20.0)}
    readable = lambda x, y: (x < 100.0, "")                       # noqa: E731
    out = MR.to_readable_when_empty({"A": ["P01"], "B": []}, pts, {"A": (20.0, 20.0), "B": (190.0, 20.0)},
                                    readable, ["A", "B"])
    assert out["B"] == ["P02"]                                    # the nearest readable point nobody else has
    monkeypatch.setattr(MR, "EMPTY_TO_READABLE", False)
    assert MR.to_readable_when_empty({"A": ["P01"], "B": []}, pts, {"A": (20, 20), "B": (190, 20)}, readable,
                                     ["A", "B"])["B"] == []
