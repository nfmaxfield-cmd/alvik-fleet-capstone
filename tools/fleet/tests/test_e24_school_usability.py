"""Build e24 (24 Nov 2026): the school runs of 23 Nov (docs/results/school-2026-11-23/report.md) and the Mission Control
usability audit of 24 Nov. Offline only: stubs and the real run_plan / planner / Mission Control code. No robot, Pi or
camera.

Each test fails on the code before e24 (39df4cf) and passes here, except the ones named "kept behind its switch"."""
import json
import math
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "pi"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np              # noqa: E402
import pytest                   # noqa: E402
import run_plan as RP           # noqa: E402
import safety_group as SG       # noqa: E402


# ====================================================================== Q-a. the heading check after a turn (N1)
class _Src:
    live = True


def _turn_coord(leg_from, leg_to, own_th, floor=(0.0, 0.0, 396.2, 167.6), obstacles=()):
    """A coordinator whose robot Alpha has just finished a turn (step 0) and has a leg (step 1) waiting."""
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.logged, c.said, c.sent = [], [], []
    c.log = lambda ev, **k: c.logged.append((ev, k))
    c.say = c.said.append
    c.send = lambda n, text: c.sent.append((n, text))
    c.source = _Src()
    c.m = {"floor": dict(x_min=floor[0], y_min=floor[1], x_max=floor[2], y_max=floor[3]),
           "obstacles": [dict(o) for o in obstacles]}
    c.plan_no, c.t_run, c.plan_start = 0, RP.now() - 30.0, RP.now() - 30.0
    c.planning, c.pending_replan = None, None
    b = RP.Bot("Alpha")
    b.out = None
    t = RP.now()
    b.pose, b.pose_t = (leg_from[0], leg_from[1], own_th), t
    b.pose_hist = [(t - 3.0 + 0.1 * k, leg_from[0], leg_from[1], own_th) for k in range(31)]
    steps = [{"type": "turn", "from": [leg_from[0], leg_from[1], 0.0], "to": [leg_from[0], leg_from[1], own_th]},
             {"type": "drive", "from": [leg_from[0], leg_from[1], own_th], "to": [leg_to[0], leg_to[1], own_th],
              "speed": 10.0}]
    c.plan = {"robots": {"Alpha": {"steps": steps}}}
    c.steps = lambda n: steps
    b.stat = [dict(status="done", t_sent=t - 2.5, t_done=t - 1.0, text="", info="", sends=1),
              dict(status="unsent", t_sent=0.0, text="", info="", sends=0)]
    b.cam_hist = []
    c.bots = {"Alpha": b}
    c.names = ["Alpha"]
    c.control = lambda bb, text, ok=None: c.sent.append((bb.name, text))
    return c, b, t


def _pic(b, t_pic, th, x=None, y=None):
    b.cam_hist.append((t_pic, t_pic + 0.1, b.pose[0] if x is None else x, b.pose[1] if y is None else y, "overhead", th))


def test_a_yaw_jump_at_a_turn_is_found_by_the_camera_and_the_turn_is_driven_again():
    """School 23 Nov, run 2: Alpha's own heading said 92 deg after its first turn, the camera 64; nothing compared the
    two and its 108 cm leg ended on the zone 1 outlet plate. Here: own -139, the camera -165 (the run's numbers)."""
    assert RP.TURN_HEADING_CHECK
    c, b, t = _turn_coord((150.0, 120.0), (68.0, 49.0), -139.0)
    _pic(b, t - 0.6, -165.0)
    _pic(b, t - 0.3, -165.4)
    assert RP.Coordinator.turn_heading_open(c, b, 1, t) is False            # the leg is NOT sent
    fp = [x for x in c.sent if x[1].startswith("fpose")]
    assert len(fp) == 1 and abs(float(fp[0][1].split()[3]) - (-165.2)) < 0.3      # heading set from the camera
    assert b.stat[0]["status"] == "unsent"                                   # the turn is driven again
    assert any(ev == "turn_heading" and k["result"] == "corrected" and abs(k["dth_deg"] + 26.2) < 0.3
               for ev, k in c.logged)
    assert any("heading" in w and "camera" in w for w in c.said)
    # the turn again, done; the camera now agrees: the leg goes
    t2 = t + 2.0
    b.stat[0].update(status="done", t_done=t2 - 0.8)
    b.pose_hist += [(t + 0.1 * k, 150.0, 120.0, -139.0) for k in range(1, 21)]
    b.pose = (150.0, 120.0, -139.0)
    _pic(b, t2 - 0.4, -139.3)
    assert RP.Coordinator.turn_heading_open(c, b, 1, t2) is True
    assert RP.Coordinator.turn_heading_summary(c)["corrected"][0]["robot"] == "Alpha"


def test_it_waits_for_a_fresh_camera_heading_only_when_the_next_leg_is_at_risk():
    """The risk rule: a leg that a heading 30 deg wrong could carry onto a keep-out (the zone 1 plate's place) or to an
    edge waits for a picture taken after the turn; a short leg in the middle of the table drives at once."""
    plate = {"id": "zone 1 outlet plate", "shape": "polygon", "points": [[55, 80], [75, 80], [75, 95], [55, 95]]}
    c, b, t = _turn_coord((150.0, 120.0), (68.0, 49.0), -139.0, obstacles=[plate])
    _pic(b, t - 1.5, -139.0)                     # a picture from BEFORE the turn ended does not count
    assert RP.Coordinator.turn_heading_open(c, b, 1, t) is False
    assert not c.sent and b.stat[0]["status"] == "done"                      # standing, nothing corrected
    assert any(ev == "turn_heading_wait" for ev, _ in c.logged)
    risky, room, at = RP.Coordinator.leg_heading_risk(c, "Alpha", c.steps("Alpha")[1], RP.Coordinator.YAW_JUMP_POSSIBLE_DEG)
    assert risky and room < 6.72 + 2.4 and at != 0.0
    _pic(b, t + 0.2, -139.5)                     # the fresh picture agrees: it drives
    assert RP.Coordinator.turn_heading_open(c, b, 1, t + 0.4) is True
    # nothing at risk: 20 cm in the middle of the table goes at once, and is compared on the way
    c, b, t = _turn_coord((200.0, 84.0), (220.0, 84.0), 0.0)
    assert RP.Coordinator.turn_heading_open(c, b, 1, t) is True
    assert b.thead_late is not None
    b.stat[1].update(status="started", t_sent=t)
    _pic(b, t + 0.3, 25.0)                       # the first picture after the turn: 25 deg off
    RP.Coordinator.turn_heading_watch(c)
    assert any(x[1] == "halt" for x in c.sent) and any(x[1].startswith("fpose") for x in c.sent)
    assert c.pending_replan is not None and c.pending_replan[0] == "Alpha"


def test_a_gyro_step_its_own_turn_rate_does_not_explain_is_a_yaw_jump():
    """Run 2 at 0.3 s: the raw yaw rose 25.7 deg in one 29 ms sample while the gyro's rate read 21 deg/s."""
    c, b, t = _turn_coord((200.0, 84.0), (220.0, 84.0), 0.0)
    RP.Coordinator.yaw_jump_watch(c, b, t, 10.0, 21.0)
    RP.Coordinator.yaw_jump_watch(c, b, t + 0.029, 35.7, 21.0)
    assert b.yaw_jump["deg"] == 25.7 and any(ev == "yaw_jump_seen" for ev, _ in c.logged)
    # a fast honest turn (120 deg/s for 0.1 s = 12 deg) is not one
    c, b, t = _turn_coord((200.0, 84.0), (220.0, 84.0), 0.0)
    RP.Coordinator.yaw_jump_watch(c, b, t, 10.0, 120.0)
    RP.Coordinator.yaw_jump_watch(c, b, t + 0.1, 22.0, 120.0)
    assert getattr(b, "yaw_jump", None) is None
    # a jump seen makes the next check wait for the camera even on a leg with nothing at risk
    c, b, t = _turn_coord((200.0, 84.0), (220.0, 84.0), 0.0)
    b.yaw_jump = dict(t=t - 2.0, deg=25.0)
    assert RP.Coordinator.turn_heading_open(c, b, 1, t) is False


def test_the_heading_check_is_one_safety_switch_with_the_old_way_behind_it():
    assert SG.DEFAULT["MIRLT_TURN_HEADING"] == "check" and SG.OLD["MIRLT_TURN_HEADING"] == "off"
    for env in ("MIRLT_TURN_HEADING", "MIRLT_SIGINT_RESTORE", "MIRLT_WRAPUP_GUARD", "MIRLT_VEER_RULE",
                "MIRLT_BLIND_STANDING", "MIRLT_PLAN_FAST"):
        assert env in SG.DEFAULT and SG.OLD[env] != SG.DEFAULT[env]


def test_the_simulator_can_make_a_gyro_jump_at_a_turns_start():
    """sim_shim cond yaw_jump: the robot's own heading steps 25 deg while it has not turned."""
    sys.path.insert(0, os.path.join(FLEET, "sweeps"))
    src = open(os.path.join(FLEET, "sweeps", "sim_shim.py")).read()
    assert 'cond.get("yaw_jump")' in src and "kind=\"yaw_jump\"" in src


# ============================================================================ B1. Ctrl+C reaches a run program
def test_a_run_program_started_from_a_shell_that_ignores_ctrl_c_still_takes_it(tmp_path):
    """The audit's B1: Mission Control started with `&` from a script; STOP's Ctrl+C was ignored for 52 s."""
    prog = tmp_path / "prog.py"
    prog.write_text("import sys, time, signal\nsys.path.insert(0, %r)\nimport sigint_default as SD\n"
                    "print('ignored at start', signal.getsignal(signal.SIGINT) == signal.SIG_IGN, flush=True)\n"
                    "print('restored', SD.restore(), flush=True)\n"
                    "try:\n    print('ready', flush=True)\n    time.sleep(30)\nexcept KeyboardInterrupt:\n    print('got it', flush=True)\n" % FLEET)

    def ignore():
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    p = subprocess.Popen([sys.executable, str(prog)], stdout=subprocess.PIPE, text=True, preexec_fn=ignore)
    assert p.stdout.readline().strip() == "ignored at start True"
    assert p.stdout.readline().strip() == "restored True"
    assert p.stdout.readline().strip() == "ready"
    time.sleep(0.2)
    t0 = time.time()
    p.send_signal(signal.SIGINT)
    out, _ = p.communicate(timeout=10)
    assert "got it" in out and time.time() - t0 < 5.0
    # the old way, kept behind its switch
    p = subprocess.Popen([sys.executable, str(prog)], stdout=subprocess.PIPE, text=True, preexec_fn=ignore,
                         env=dict(os.environ, MIRLT_SIGINT_RESTORE="0"))
    assert p.stdout.readline().strip() == "ignored at start True"
    assert p.stdout.readline().strip() == "restored False"
    p.kill()
    # every run program calls it, and Mission Control starts its runs with SIGINT at its default
    for f in ("run_plan.py", "mission_run.py", "tag_bridge.py", "cam_bridge.py", "mission_control.py",
              os.path.join("sweeps", "rp_shim.py"), os.path.join("sweeps", "sim_shim.py")):
        assert "_SD.restore()" in open(os.path.join(FLEET, f)).read(), f
    assert "preexec_fn=SDF.preexec()" in open(os.path.join(FLEET, "mission_control.py")).read()
    assert "preexec_fn=SDF.preexec()" in open(os.path.join(FLEET, "start_job.py")).read()


# ================================================================== D1 / D2. picture stalls: slow, or stay put
def _blind_coord(pose, moved_cm=0.0, started_drive=False):
    c, b, t = _turn_coord((pose[0], pose[1]), (pose[0] + 50.0, pose[1]), pose[2])
    b.stat[1].update(status="started" if started_drive else "unsent")
    bl = dict(t0=t - 1.2, start=[pose[0] - moved_cm, pose[1]], last=list(pose[:2]), dist=moved_cm, said=False)
    return c, b, bl, t


def test_a_standing_robot_anywhere_stays_put_when_the_camera_loses_it():
    """School 23 Nov: 13 of the 16 turn-backs that drove back were robots standing somewhere (waiting for a re-plan,
    turning, just rejoined). e23 kept only a robot in its dwell at a point."""
    assert RP.BLIND_STANDING_STAY
    c, b, bl, t = _blind_coord((30.0, 20.0, 0.0))
    assert RP.Coordinator.standing_blind(c, b, bl, 0.0, t) is True
    # it has driven 6 cm unseen, or its leg has begun: the driving rules
    c, b, bl, t = _blind_coord((30.0, 20.0, 0.0), moved_cm=6.0)
    assert RP.Coordinator.standing_blind(c, b, bl, 0.5, t) is False
    c, b, bl, t = _blind_coord((30.0, 20.0, 0.0), started_drive=True)
    assert RP.Coordinator.standing_blind(c, b, bl, 0.0, t) is False


def test_a_driving_robot_with_little_room_slows_instead_of_turning_back():
    """Nick, 24 Nov (D1): turn back only when its doubt, at its speed, could reach the edge; otherwise keep going
    slowly until pictures return. 10 cm/s with 12 cm of room: 12 < 10 x 1.97 s, so it is slowed to 5 cm/s."""
    assert RP.VEER_REACH
    c, b, bl, t = _blind_coord((30.0, 20.0, 0.0), moved_cm=8.0, started_drive=True)
    RP.Coordinator.blind_slow(c, b, bl, 12.0, 10.0)
    assert ("Alpha", "fspd 0.50") in c.sent and bl["slowed"]
    assert any(ev == "blind_slow" and k["scale"] == 0.5 for ev, k in c.logged)
    n = len(c.sent)
    RP.Coordinator.blind_slow(c, b, bl, 11.0, 5.0)                       # once per loss
    assert len(c.sent) == n
    # plenty of room: its planned speed
    c, b, bl, t = _blind_coord((200.0, 84.0, 0.0), moved_cm=8.0, started_drive=True)
    RP.Coordinator.blind_slow(c, b, bl, 60.0, 10.0)
    assert not c.sent
    # how long until pictures are usually back: the school's measured 90th percentile until the run has its own
    assert RP.Coordinator.blind_back_s(c) == 1.97
    c.blinds = [dict(blind_s=1.0 + 0.1 * k) for k in range(10)]
    assert abs(RP.Coordinator.blind_back_s(c) - 1.9) < 1e-9


# ====================================================================================== planning speed (geometry)
def test_the_quick_polygon_distance_gives_the_same_numbers_as_the_old_one():
    import geometry as G
    rng = np.random.default_rng(24)
    for _ in range(400):
        poly = rng.uniform(0, 100, (int(rng.integers(3, 9)), 2))
        p = rng.uniform(-20, 120, (int(rng.integers(1, 30)), 2))
        assert np.array_equal(G._points_to_polygon_loop(p, poly), G._points_to_polygon_fast(p, poly))
    assert G.points_to_polygon is G._points_to_polygon_fast


# ============================================================ D3 / D4 / Q-b / Q-c / N4. the points ledger
class _Args:
    on_problem = "keep_going"
    visit_lists = "per_robot"
    visit_missing = ""
    lost_robot_points = "reassign"
    dedupe_points = "on"
    no_rejoin = False
    battery_floor = 25
    retry_s = 3.0


def _item(p, xy):
    return [{"goto": list(xy), "label": "point " + p}, {"wait_s": 5.0, "label": "at point " + p}]


def _ledger_coord(lists, spots=None, missing=""):
    """Two robots with a list each; every step of every list still unsent. lists = {"Alpha": ["P01", "P02"], ...}."""
    spots = spots or {"P%02d" % k: [20.0 + 9.0 * k, 40.0 + (k % 5) * 20.0] for k in range(1, 41)}
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.logged, c.said, c.sent = [], [], []
    c.log = lambda ev, **k: c.logged.append((ev, k))
    c.say = c.said.append
    c.send = lambda n, text: c.sent.append((n, text))
    c.source = _Src()
    c.args = _Args()
    c.args.visit_missing = missing
    c.m = {"floor": dict(x_min=0.0, y_min=0.0, x_max=396.2, y_max=167.6), "obstacles": [], "spots": spots}
    c.plan_no, c.t_run, c.plan_start = 0, RP.now() - 30.0, RP.now() - 30.0
    c.planning, c.pending_replan, c.deferred = None, None, []
    c.abort = None
    c.names, c.bots, robots = [], {}, {}
    for n, labs in lists.items():
        vo, steps = [], []
        for k, p in enumerate(labs):
            vo += _item(p, spots[p])
            steps += [{"type": "drive", "label": "point " + p, "task": 2 * k, "from": [0, 0, 0], "to": spots[p] + [0]},
                      {"type": "wait", "label": "at point " + p, "task": 2 * k + 1, "phase": "dwell"}]
        vo.append({"goto": [100.0, 80.0], "label": "home"})
        steps.append({"type": "drive", "label": "home", "task": 2 * len(labs), "from": [0, 0, 0], "to": [100, 80, 0]})
        robots[n] = {"start": [100.0, 80.0, 90.0], "visit_order": vo, "steps": steps, "task_offset": 0, "pending": None}
        b = RP.Bot(n)
        b.out = None
        b.pose = (100.0, 80.0, 90.0)
        b.stat = [dict(status="unsent", t_sent=0.0, text="", info="", sends=0) for _ in steps]
        c.bots[n] = b
        c.names.append(n)
    c.plan = {"robots": robots, "settings": {}, "summary": {"makespan_s": 100.0}}
    c.steps = lambda n: c.plan["robots"][n]["steps"]
    RP.Coordinator.ledger_init(c)
    return c


def _done(c, n, upto_label=None, credit=()):
    """Mark robot n's steps done up to and including the step labelled upto_label (all when None); credit the points."""
    for st, stp in zip(c.bots[n].stat, c.steps(n)):
        st["status"] = "done"
        if upto_label is not None and stp["label"] == upto_label:
            break
    for p in credit:
        c.__dict__.setdefault("points", []).append({"robot": n, "point": "point " + p})
        c.__dict__.setdefault("done_labels", set()).add("at point " + p)


def _watch(c, times=3):
    for _ in range(times):
        c._ledger_t = 0.0
        RP.Coordinator.ledger_watch(c)


def test_a_point_that_leaves_every_list_gets_a_line_and_is_tried_once_more_at_the_end_of_the_list():
    """School 23 Nov: 13 of the 15 refused points were never credited; in run 1 a re-plan cut in as Alpha arrived at
    P13 and the point vanished with no line (N4)."""
    assert RP.POINT_RETRY
    c = _ledger_coord({"Alpha": ["P01", "P13", "P02"], "Bravo": ["P21"]})
    assert RP.Coordinator.ledger_counts(c) == (4, 0)
    # a new plan left P13 out of Alpha's list (nothing written), while Alpha is still on its list
    e = c.plan["robots"]["Alpha"]
    e["visit_order"] = [v for v in e["visit_order"] if "P13" not in v["label"]]
    keep = [k for k, stp in enumerate(e["steps"]) if "P13" not in stp["label"]]
    e["steps"] = [e["steps"][k] for k in keep]
    c.bots["Alpha"].stat = [c.bots["Alpha"].stat[k] for k in keep]
    _done(c, "Alpha", "at point P01", credit=["P01"])
    _watch(c)
    drop = [k for ev, k in c.logged if ev == "point_dropped"]
    assert len(drop) == 1 and drop[0]["point"] == "point P13" and drop[0]["list_of"] == "Alpha"
    assert any("P13" in w and "in no robot's list" in w and "tried once more" in w for w in c.said)
    assert RP.Coordinator.ledger_pending(c) is True                      # the run is not over while it waits
    assert c.pending_replan is None and not [x for ev, x in c.logged if ev == "points_retried"]   # not yet: Alpha has P02
    # Alpha reaches the end of its list (only its trip home left): P13 goes back in, once
    _done(c, "Alpha", "at point P02", credit=["P02"])
    _watch(c)
    assert [k["points"] for ev, k in c.logged if ev == "points_retried"] == [["point P13"]]
    assert "point P13" in RP.Coordinator.listed_labels(c, "Alpha") and c.pending_replan is not None
    assert c.ledger["point P13"]["tries"] == 1
    # it is missed again (refused): reported, not tried a third time
    c.pending_replan = None
    e = c.plan["robots"]["Alpha"]
    e["visit_order"] = [v for v in e["visit_order"] if "P13" not in v["label"]]
    if e.get("pending"):
        e["pending"] = None
    c.not_reached = [{"robot": "Alpha", "point": "point P13", "miss_cm": 19.1, "reach_tol_cm": 6.2, "t_run": 80.0}]
    _watch(c)
    assert len([1 for ev, k in c.logged if ev == "points_retried"]) == 1
    assert RP.Coordinator.ledger_pending(c) is False
    vr = RP.Coordinator.ledger_summary(c, "done")
    assert vr["headline"] == "Ended: 2 of 4 points reached"
    miss = vr["robots"]["Alpha"]["missed"]
    assert [q["point"] for q in miss] == ["P13"] and miss[0]["tried_again"] and "19.1 cm" in miss[0]["why"]
    assert "calibration" in miss[0]["next"]


def test_done_is_said_only_when_every_point_was_reached():
    c = _ledger_coord({"Alpha": ["P01", "P02"], "Bravo": ["P21"]})
    _done(c, "Alpha", credit=["P01", "P02"])
    _done(c, "Bravo", credit=["P21"])
    vr = RP.Coordinator.ledger_summary(c, "done")
    assert vr["headline"] == "Done: all 3 points reached" and not vr["robots"]["Alpha"]["missed"]
    assert vr["robots"]["Alpha"]["reached_points"] == ["P01", "P02"]
    # stopped by a person half way: the count, and the reason is the stop
    c = _ledger_coord({"Alpha": ["P01", "P02"], "Bravo": ["P21"]})
    _done(c, "Alpha", "at point P01", credit=["P01"])
    vr = RP.Coordinator.ledger_summary(c, "interrupted")
    assert vr["headline"] == "Ended: 1 of 3 points reached"
    assert all("stopped before" in q["why"] for o in vr["robots"].values() for q in o["missed"])


def test_with_a_list_per_robot_a_held_robots_points_wait_for_it():
    """The audit's B4 (run 20261124-094026): Alpha held, its P08 went to Bravo, Bravo refused it, Alpha rejoined 80 s
    later and went home without it."""
    assert RP.HELD_POINTS_WAIT
    c = _ledger_coord({"Alpha": ["P01", "P02", "P08"], "Bravo": ["P35", "P36"]})
    c.handle_obstacle = lambda *a, **k: None
    c.out_obstacle = lambda b: {"id": "out-" + b.name, "shape": "polygon", "points": [[0, 0], [1, 0], [1, 1]]}
    c.control = lambda b, text, ok=None: None
    _done(c, "Alpha", "at point P01", credit=["P01"])
    a = c.bots["Alpha"]
    RP.Coordinator.retire(c, a, "camera and its own pose disagreed 3 times in a minute", kind="restarted")
    assert not [1 for ev, k in c.logged if ev == "points_reassigned"]
    assert [k["points"] for ev, k in c.logged if ev == "points_wait"] == [["point P02", "point P08"]]
    assert "point P08" not in RP.Coordinator.listed_labels(c, "Bravo")
    assert RP.Coordinator.ledger_open(c)["point P08"] == "held"
    # Bravo finishes its own list and Alpha is still held: now they go to Bravo
    _done(c, "Bravo", "at point P36", credit=["P35", "P36"])
    _watch(c)
    assert [sorted(k["to"]["Bravo"]) for ev, k in c.logged if ev == "points_reassigned"] == [["point P02", "point P08"]]
    assert a.handed_over is True


def test_a_robot_named_in_the_order_and_not_found_has_its_points_taken_by_one_that_finished():
    c = _ledger_coord({"Alpha": ["P01"]}, missing="Bravo:P35,P36")
    assert RP.Coordinator.ledger_counts(c) == (3, 0) and c.ledger["point P35"]["absent"]
    _done(c, "Alpha", "at point P01", credit=["P01"])
    _watch(c)
    assert [k["points"] for ev, k in c.logged if ev == "points_retried"] == [["point P35", "point P36"]]
    assert {"point P35", "point P36"} <= RP.Coordinator.listed_labels(c, "Alpha")
    vr = RP.Coordinator.ledger_summary(c, "interrupted")
    assert vr["robots"]["Bravo"]["asked"] == 2 and vr["headline"] == "Ended: 1 of 3 points reached"


def test_a_robot_under_the_battery_floor_finishes_its_point_goes_home_and_hands_over_the_rest():
    """School 23 Nov, run 11: Bravo started at 19 % (floor 25 %) and ran its list."""
    assert RP.BATTERY_LOW_HOME
    c = _ledger_coord({"Alpha": ["P01", "P02"], "Bravo": ["P35", "P36", "P34"]})
    b = c.bots["Bravo"]
    _done(c, "Bravo", "point P35")                 # it has arrived at P35 and is about to dwell there
    b.battery = 19
    assert RP.Coordinator.battery_home(c, b, 25) is True
    labs = [v["label"] for v in c.plan["robots"]["Bravo"]["visit_order"]]
    assert labs == ["point P35", "at point P35", "home"]                 # this point, then home
    assert {"point P36", "point P34"} <= RP.Coordinator.listed_labels(c, "Alpha")
    assert c.pending_replan is not None and any("home spot" in w and "19 %" in w for w in c.said)
    assert RP.Coordinator.battery_home(c, b, 25) is False                # once
    # the only robot running: it finishes the run, as before
    c = _ledger_coord({"Alpha": ["P01", "P02"]})
    c.bots["Alpha"].battery = 19
    assert RP.Coordinator.battery_home(c, c.bots["Alpha"], 25) is False


def test_a_held_robot_is_never_given_up_on_and_a_rejoined_robot_without_a_route_is_not_held_again():
    assert RP.REJOIN_NO_CAP and RP.REJOIN_STRANDED_WAIT
    c = _ledger_coord({"Alpha": ["P01"], "Bravo": ["P21"]})
    b = c.bots["Alpha"]
    b.out, b.out_kind, b.out_entry, b.rejoins_n = "held", "restarted", {"steps": []}, 25
    assert RP.Coordinator.rejoinable(c, b)              # 23 Nov run 5: out for good after its 10th rejoin
    src = open(os.path.join(FLEET, "run_plan.py")).read()
    assert "rejoin_waits_for_route" in src
