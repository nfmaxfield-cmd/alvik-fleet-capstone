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
