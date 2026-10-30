"""Turn pause, step away and the 8 cm lock (build e02-oct29-turn-pause.md, 2026-10-29)."""
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
sys.path.insert(0, FLEET)

import missions as MS       # noqa: E402
import run_plan as RP       # noqa: E402
import workspace as WS      # noqa: E402


# ---------------------------------------------------------------------------- 1. the pause after a turn

def _coord(steps, turn_pause=True):
    class Src:
        live = True

    class A:
        pass
    A.turn_pause = turn_pause
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.args, c.source, c.plan_no, c.t_run = A(), Src(), 0, RP.now() - 10
    c.plan = {"robots": {"Left": {"steps": steps}}}
    b = RP.Bot("Left")
    b.stat = [dict(status="unsent", t_sent=0.0, text="", info="", sends=0) for _ in steps]
    c.bots = {"Left": b}
    c.logged, c.said = [], []
    c.log = lambda ev, **k: c.logged.append((ev, k))
    c.say = c.said.append
    return c, b


TURN = {"type": "turn", "from": [20, 20, 90], "to": [20, 20, 0]}
TRIM = {"type": "turn", "from": [20, 20, 90], "to": [20, 20, 85]}
WAIT = {"type": "wait", "from": [20, 20, 0], "to": [20, 20, 0]}
DRIVE = {"type": "drive", "from": [20, 20, 0], "to": [40, 20, 0], "label": "mow lane 1 end"}


def test_drive_after_turn_waits_for_the_camera_then_goes():
    c, b = _coord([TURN, WAIT, DRIVE])
    t0 = RP.now()
    assert c.turn_before("Left", 2) == 0                       # waits in between do not count
    assert not c.turn_pause_open(b, 2, t0)                     # the turn is not done: not queued behind it
    b.stat[0].update(status="done", t_done=t0)
    b.pose, b.pose_t = (20.0, 20.0, 0.0), t0 + 0.05
    assert not c.turn_pause_open(b, 2, t0 + 0.1)               # no fresh fix yet
    # fixes from both cameras, 2.5 cm off: the robot has not taken them yet
    b.cam_hist = [(t0 + 0.05, t0 + 0.30, 22.5, 20.0, "both"), (t0 + 0.15, t0 + 0.40, 22.5, 20.0, "both")]
    b.pose_t = t0 + 0.7
    assert not c.turn_pause_open(b, 2, t0 + 0.7)               # 2.5 cm apart: stand
    b.pose = (22.1, 20.0, 0.0)                                 # it took them
    assert c.turn_pause_open(b, 2, t0 + 0.8)
    ev, rec = c.logged[-1]
    assert ev == "turn_pause" and rec["result"] == "agreed" and rec["how"] == "both"
    assert rec["d_cm"] == pytest.approx(0.4, abs=0.01)
    s = c.turn_pause_summary()["Left"]
    assert s["pauses"] == 1 and s["both"] == 1 and s["timed_out"] == 0


def test_old_fixes_and_fixes_not_taken_yet_do_not_count():
    c, b = _coord([TURN, DRIVE])
    t0 = RP.now()
    b.stat[0].update(status="done", t_done=t0)
    b.pose, b.pose_t = (20.0, 20.0, 0.0), t0 + 0.5
    b.cam_hist = [(t0 - 0.3, t0 - 0.1, 20.0, 20.0, "both"),     # a picture from before the turn ended
                  (t0 + 0.2, t0 + 0.4, 20.0, 20.0, "both")]     # arrived 0.1 s before the pose: not taken yet
    assert not c.turn_pause_open(b, 1, t0 + 0.5)


def test_one_camera_only_after_a_second_and_timeout_drives_on():
    c, b = _coord([TURN, DRIVE])
    t0 = RP.now()
    b.stat[0].update(status="done", t_done=t0)
    b.cam_hist = [(t0 - 0.5, t0 - 0.4, 20.0, 20.0, "both"),     # both saw it while it turned
                  (t0 + 0.1, t0 + 0.2, 20.3, 20.0, "side")]
    b.pose, b.pose_t = (20.0, 20.0, 0.0), t0 + 0.5
    assert not c.turn_pause_open(b, 1, t0 + 0.5)               # one camera: not yet (the other may see it again)
    assert c.turn_pause_open(b, 1, t0 + RP.Coordinator.TURN_PAUSE_ONE_CAM_S + 0.01)
    assert c.logged[-1][1]["how"] == "one camera (side)"
    # on one camera all along (a washed-out overhead): that camera's fix is taken at once
    c3, b3 = _coord([TURN, DRIVE])
    b3.stat[0].update(status="done", t_done=t0)
    b3.cam_hist = [(t0 - 0.5, t0 - 0.4, 20.0, 20.0, "side"), (t0 + 0.1, t0 + 0.2, 20.3, 20.0, "side")]
    b3.pose, b3.pose_t = (20.0, 20.0, 0.0), t0 + 0.5
    assert c3.turn_pause_open(b3, 1, t0 + 0.5)
    # a robot the camera cannot find: it drives on after the timeout, as before, and says so
    c2, b2 = _coord([TURN, DRIVE])
    b2.stat[0].update(status="done", t_done=t0)
    assert not c2.turn_pause_open(b2, 1, t0 + 1.0)
    assert c2.turn_pause_open(b2, 1, t0 + RP.Coordinator.TURN_PAUSE_MAX_S + 0.01)
    assert c2.logged[-1][1]["result"] == "timed out" and "driving on" in c2.said[-1]


def test_no_pause_for_trims_without_the_flag_or_without_a_camera():
    c, b = _coord([TRIM, DRIVE])
    assert c.turn_before("Left", 1) is None
    assert c.turn_pause_open(b, 1, RP.now())
    c, b = _coord([TURN, DRIVE], turn_pause=False)
    assert c.turn_pause_open(b, 1, RP.now())
    c, b = _coord([TURN, DRIVE])
    c.source.live = False
    assert c.turn_pause_open(b, 1, RP.now())


def test_chicken_swerve_and_pass_are_not_paused():
    """Nick, 29 Oct: Chicken swerves and keeps going past the other robot (no stand at the jog or the turn back)."""
    run_in = dict(DRIVE, label="chicken 1 swerve", task=18)
    jog_turn = dict(TURN, task=19)
    jog_a = dict(DRIVE, label=None, task=19)                   # a goal cut in two: only its last piece is labelled
    jog_b = dict(DRIVE, label="chicken 2 sidestep", task=19)
    back_turn = dict(TURN, task=20)
    passing = dict(DRIVE, label="chicken 2 pass", task=20)
    after = dict(TURN, task=22)
    lane = dict(DRIVE, label="mow lane 1 end", task=22)
    steps = [run_in, jog_turn, jog_a, jog_b, back_turn, passing, after, lane]
    c, b = _coord(steps)
    assert c.turn_before("Left", 2) is None                    # the jog
    assert c.turn_before("Left", 5) is None                    # the turn back and on past
    assert c.turn_before("Left", 7) == 6                       # an ordinary drive after a turn still stands
    b.stat[1].update(status="done", t_done=RP.now())
    assert c.turn_pause_open(b, 2, RP.now()) and not c.logged  # sent at once, no pause logged
    # Monday's own plan steps: the pieces of the jog and the pass, by their goal
    assert c.chicken_keep_going(steps, 2) and not c.chicken_keep_going(steps, 0)


def test_mission_control_runs_pass_the_turn_pause(tmp_path):
    import mission_control as MC
    S = MC.Session(str(tmp_path / "mc"))
    assert S.settings["run"]["turn_pause"] is True
    src = open(os.path.join(FLEET, "mission_run.py")).read()
    assert 'rp += ["--turn-pause"]' in src and "--no-turn-pause" in src


# ---------------------------------------------------------------------------- 2. the waiting robot steps away

@pytest.mark.parametrize("size", [(122, 61), (130, 70), (137, 76)])
def test_waiting_robot_steps_away_after_the_circle(size):
    """Without mirroring (a program with no "mirror"): the lanes 12 cm apart put the first inner lanes too close, so one
    robot steps away. Monday mirrors (2026-10-29 20:06): see test_mirrored_monday_needs_no_step_away."""
    W, H = size
    m = WS.blank_map((0, 0, W, H), [("Left", [20, 20, 90]), ("Center", [60, 20, 90]), ("Right", [100, 20, 90])])
    prog = MS.monday_program(("Left", "Right"))
    prog["mirror"] = []           # explicitly not mirrored (a program without the key is Monday's saved default)
    mm = MS.build(m, prog)
    note = [n for n in mm["mission"]["notes"] if "steps" in n and "away" in n]
    assert len(note) == 1
    who = [n for n in ("Left", "Right") if any(v.get("label") == "step away" for v in mm["tasks"][n]["visit"])]
    assert len(who) == 1
    w = who[0]
    o = "Right" if w == "Left" else "Left"
    vis = mm["tasks"][w]["visit"]
    k = next(i for i, v in enumerate(vis) if v.get("label") == "step away")
    assert "arc" in vis[k - 1]                                  # straight after the circle
    first_other = next(v for v in mm["tasks"][o]["visit"] if str(v.get("label", "")).startswith("mow lane"))
    gap = abs(vis[k]["goto"][0] - first_other["goto"][0]) - 9.5
    assert gap >= MS.STEP_AWAY_GAP_CM - 1e-6


@pytest.mark.parametrize("size", [(122, 61), (130, 70), (137, 76)])
def test_mirrored_monday_needs_no_step_away(size):
    """Mirrored lanes: the innermost pair is mirror_centre_gap apart, so after the circle both mow at once with the
    bodies STEP_AWAY_GAP_CM apart and nobody waits or steps away."""
    W, H = size
    m = WS.blank_map((0, 0, W, H), [("Left", [20, 20, 90]), ("Center", [60, 20, 90]), ("Right", [100, 20, 90])])
    mm = MS.build(m, MS.monday_program(("Left", "Right")))
    assert not [n for n in mm["mission"]["notes"] if "steps" in n and "away" in n]
    firsts = []
    for n in ("Left", "Right"):
        vis = mm["tasks"][n]["visit"]
        k = next(i for i, v in enumerate(vis) if "arc" in v)
        firsts.append(next(v for v in vis[k:] if str(v.get("label", "")).startswith("mow lane"))["goto"][0])
    assert abs(firsts[0] - firsts[1]) - 9.5 >= MS.STEP_AWAY_GAP_CM - 1e-6


# ---------------------------------------------------------------------------- 3. the 8 cm lock

def test_mission_settings_cannot_go_under_the_net():
    s = MS.settings({"separation_cm": 5.0, "chicken_min_cm": 6.0})
    assert s["separation_cm"] == 8.0 and s["chicken_min_cm"] == 8.0
    s = MS.settings({"chicken_min_cm": 10.0, "separation_cm": 8.0})
    assert s["separation_cm"] == 10.0                           # never under the mission's own chicken_min_cm


def test_planner_settings_raised_for_two_robots_only():
    over = {"separation_cm": 5.0}
    said = MS.gap_lock_planner(over, n_robots=2)
    assert over["separation_cm"] == 8.0 and said and "raised" in said[0]
    over = {"separation_cm": 5.0}
    assert MS.gap_lock_planner(over, n_robots=1) == [] and over["separation_cm"] == 5.0
    over = {}
    MS.gap_lock_planner(over, n_robots=2)
    assert over["separation_cm"] == 8.0                         # missing: the planner's own default would be 5


def test_plan_synced_holds_the_net_against_a_saved_5():
    m = WS.blank_map((0, 0, 122, 61), [("Left", [20, 20, 90]), ("Right", [100, 20, 90])])
    mm = dict(m, tasks={"Left": {"visit": [{"goto": [30, 40]}]}, "Right": {"visit": [{"goto": [90, 40]}]}},
              mission={"name": "t", "settings": MS.settings(), "planner": {"separation_cm": 8.0}})
    said = []
    plan = MS.plan_synced(mm, over={"separation_cm": 5.0}, log=said.append)
    assert plan["settings"]["separation_cm"] == 8.0
    assert any("raised" in w for w in said)


def test_mission_control_save_planner_settings_raises_and_says_so(tmp_path):
    import mission_control as MC
    d = str(tmp_path / "mc")
    os.makedirs(d)
    with open(os.path.join(d, "settings.json"), "w") as f:
        json.dump({"planner": {"separation_cm": 5.0}, "mission": {"separation_cm": 6.0}}, f)
    S = MC.Session(d)
    assert S.settings["planner"]["separation_cm"] == 8.0 and S.settings["mission"]["separation_cm"] == 8.0
    assert len(S.state()["gap_notices"]) == 2
    saved = json.load(open(os.path.join(d, "settings.json")))
    assert saved["planner"]["separation_cm"] == 8.0
    # the planner screen shows the net, so saving it unchanged says nothing
    assert S.state()["defaults"]["planner"]["separation_cm"] == 8.0
    S.settings["planner"].update({"separation_cm": 8.0})
    assert S.lock_gaps("page") == []
    S.settings["planner"].update({"separation_cm": 4.0})
    assert S.lock_gaps("page") and S.settings["planner"]["separation_cm"] == 8.0


def test_mission_run_and_run_plan_raise_the_net(capsys):
    import mission_run as MR

    class A:
        robots, min_gap_cm, below_net_doe, dry = "Left,Right", 5.0, False, False
    a = A()
    MR.gap_args(a)
    assert a.min_gap_cm == 8.0 and "8 cm LOCK" in capsys.readouterr().out
    a.min_gap_cm, a.below_net_doe = 5.0, True
    with pytest.raises(SystemExit):
        MR.gap_args(a)                                          # the DOE is for the simulator only

    class B:
        min_gap_cm, below_net_doe, dry_run = 0.0, False, False
    b = B()
    RP.gap_lock(b, 2)
    assert b.min_gap_cm == 8.0
    b.min_gap_cm, b.below_net_doe = 5.0, True
    RP.gap_lock(b, 2)
    assert b.min_gap_cm == 8.0                                  # --below-net-doe without --dry-run: still 8
    b.min_gap_cm, b.dry_run = 5.0, True
    RP.gap_lock(b, 2)
    assert b.min_gap_cm == 5.0                                  # the simulator's safety-net DOE
    b.min_gap_cm, b.dry_run, b.below_net_doe = 0.0, False, False
    RP.gap_lock(b, 1)
    assert b.min_gap_cm == 0.0                                  # one robot: nothing to keep apart
