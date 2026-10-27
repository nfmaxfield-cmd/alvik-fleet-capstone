"""The 2026-10-27 edge fixes (docs/build-reports/e02-oct27-edge-fixes.md; Nick approved items 1-4 at 07:45), each
against last night's logged numbers (stage 2, e02-20261027-002947; docs/build-reports/e02-oct27-session-edges.md):
1 who's who after passing a parked robot, 2 one-picture jumps near edges, 3 own pose after a barrier-kept find,
4 a rescue that drives on inward."""
import json
import math
import os
import sys
import threading
import types
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import e02 as E  # noqa: E402
import e02_plan as PL  # noqa: E402

RUN4 = json.load(open(os.path.join(HERE, "data", "e02_oct27_run4.json")))
BOARD = PL.Board.from_dict(RUN4["board"])        # roam1, no boards: all four sides open


class _Hist:
    def __init__(self, smp):
        self.q = [tuple(s) for s in smp]
        self.now = 0.0

    def since(self, t0, t1=None):
        return [s for s in self.q if t0 <= s[0] <= (self.now if t1 is None else t1)]


class _St(dict):
    def save(self):
        pass


def _say(r):
    r.said = []
    r.say = lambda m, quiet=False: r.said.append(m)
    r.mark = lambda t: None


# --- fix 1: who's who --------------------------------------------------------------------------------------------
def _identity_stub():
    r = types.SimpleNamespace(st=_St(positions={"Right": RUN4["right"] + [None], "Left": [95.0, 94.0, None]}))
    _say(r)
    r.history = _Hist(RUN4["camera"])
    r.listener = types.SimpleNamespace(odo={"Left": deque(maxlen=400)})
    r.ID_ODO_FRESH_S = E.Runner.ID_ODO_FRESH_S
    return r


def test_run4_the_wheels_catch_the_swap_with_set_aside_right_20_s_before_the_bad_fix():
    """00:58:25 Left reached its far point 13 cm from set-aside Right's spot; from 00:58:28 the camera followed Right.
    Left's wheels drove it back 15 cm by 00:58:36 while the spot moved 3 cm: the doubt says so (the bad fix came at
    00:58:56, after which Left was sent to the x = 122 edge)."""
    r = _identity_stub()
    odo = RUN4["odometry"]
    k = 0
    opened = said = None
    for i in range(0, 300):
        now = i * 0.1
        r.history.now = now
        while k < len(odo) and odo[k][0] <= now:
            r.listener.odo["Left"].append((odo[k][0], odo[k][1], odo[k][2], "4931-7"))
            k += 1
        why = E.Runner.identity_tick(r, "Left", now)
        if opened is None and r.__dict__.get("id_doubt", {}).get("Left"):
            opened = now
        if why:
            said = (now, why)
            break
    assert opened is not None and 4.0 < opened < 7.0                    # at its far point, next to Right
    assert said is not None and 14.5 < said[0] < 17.0, said              # 00:58:34.5-37: 15 cm into the drive back
    assert "Right" in said[1] and ("wheels moved 15 cm" in said[1] or "wheels moved 16 cm" in said[1])
    rec = r.st["identity_doubts"][-1]
    assert rec[5] == "wheels disagree" and rec[7] < 5.0                   # the spot moved under 5 cm


def test_a_robot_the_camera_really_follows_past_a_parked_one_is_confirmed_by_its_wheels():
    r = _identity_stub()
    # Left drives 30 cm past Right's spot (13 cm from it) and the camera follows it: the wheels agree
    cam = [(0.1 * i, 18.0 + 1.0 * i, 18.0) for i in range(31)]
    r.history = _Hist(cam)
    for i in range(31):
        now = 0.1 * i
        r.history.now = now
        r.listener.odo["Left"].append((now, -24.0 + 1.0 * i * 1.02, 8.5, "r"))   # wheels 2 % long
        assert E.Runner.identity_tick(r, "Left", now) == ""
    assert "Left" not in r.id_doubt and r.st["identity_doubts"][-1][5] == "wheels agree"


def test_an_open_doubt_at_the_next_fix_with_its_own_pose_110_cm_away_stops_everyone_and_asks_its_lights():
    r = _identity_stub()
    r.id_doubt = {"Left": dict(t=0.0, cam=(18.9, 17.7), near="Right", gap=13.3, odo0=None)}
    r.own_latest_xy = lambda n: tuple(RUN4["left_end_own"])              # its own end of run 4 (e02.csv)
    held = []
    r.hold_for_nick = lambda n, kind, why: held.append((n, kind, why)) or "c"
    r.held_out = set()
    r.settle = lambda min_s=0.3: (95.6, 94.0)                            # where its lights put it
    p = E.Runner.identity_at_fix(r, "Left", tuple(RUN4["camera_at_fix"]))
    assert held and held[0][1] == "lost" and "Right" in held[0][2] and "110 cm" in held[0][2]
    assert p == (95.6, 94.0)
    # its own pose agreeing (an uncalibrated U run's 16 cm is within ID_FIX_AGREE_CM 15? no: 12 cm is)
    r.id_doubt = {"Left": dict(t=0.0, cam=(18.9, 17.7), near="Right", gap=13.3, odo0=None)}
    r.own_latest_xy = lambda n: (95.8, 94.3)
    held.clear()
    assert E.Runner.identity_at_fix(r, "Left", (90.0, 83.7)) == (90.0, 83.7) and not held


def test_run4s_route_is_not_run_its_far_point_is_under_25_cm_from_set_aside_right():
    r = types.SimpleNamespace(board=BOARD, unavailable={"Right"}, st={"positions": {"Right": RUN4["right"] + [None]}})
    blk = E.Runner.route_blocked_by_aside(r, 1)                           # straight 100 cm, run 4's route
    assert blk is not None and blk[0] == "Right" and 20.0 < blk[1] < 22.0   # 20.6-20.9 cm: allowed before
    r.st["positions"]["Right"] = [60.0, 25.0, None]                       # well clear of the far point and path
    assert E.Runner.route_blocked_by_aside(r, 1) is None


# --- fix 2: jumps near edges -------------------------------------------------------------------------------------
RIGHT_0054 = [(0.0, 16.86, 12.55, 'lock'), (0.079, 16.86, 12.55, 'lock'), (0.2, 16.86, 12.55, 'lock'),
              (0.439, 16.86, 12.55, 'lock'), (0.48, 16.86, 12.55, 'lock'), (0.627, 16.86, 12.55, 'lock'),
              (0.719, 16.86, 12.55, 'lock'), (0.8, 16.86, 12.55, 'lock'), (0.88, 16.86, 12.55, 'lock'),
              (0.96, 16.86, 12.55, 'lock'), (1.039, 16.86, 12.55, 'lock'), (1.16, 11.24, 13.1, 'both'),
              (1.406, 16.3, 12.6, 'lock'), (1.48, 16.3, 12.6, 'lock'), (1.521, 16.3, 12.6, 'lock'),
              (1.6, 11.26, 13.05, 'both'), (1.72, 15.34, 12.69, 'lock'), (1.8, 15.34, 12.69, 'lock'),
              (1.88, 15.34, 12.69, 'lock'), (1.96, 15.34, 12.69, 'lock')]   # track.csv, t - 1793076847.5 (00:54:07.5)
LEFT_0043 = [(0.071, 11.33, 97.54, 'detector'), (0.51, 17.74, 100.52, 'lock'), (0.671, 14.63, 99.07, 'both'),
             (0.751, 15.46, 99.45, 'lock'), (0.831, 15.87, 99.65, 'lock'), (0.872, 16.12, 99.76, 'lock'),
             (0.959, 16.29, 99.84, 'lock'), (1.117, 16.41, 99.9, 'lock'), (1.311, 17.12, 100.23, 'lock'),
             (1.4, 17.12, 100.23, 'lock'), (1.471, 11.56, 97.6, 'both'), (1.672, 15.99, 99.74, 'lock'),
             (1.73, 15.99, 99.74, 'lock'), (1.872, 15.99, 99.74, 'lock'), (1.96, 16.05, 106.83, 'detector'),
             (2.078, 11.24, 97.69, 'detector'), (2.211, 11.52, 97.66, 'detector'), (2.271, 11.27, 97.74, 'detector'),
             (2.474, 16.65, 106.75, 'detector'), (2.581, 16.53, 106.5, 'detector'), (2.751, 16.6, 106.58, 'both'),
             (2.991, 18.89, 108.3, 'lock'), (3.071, 18.89, 108.3, 'lock')]   # t - 1793076189.5 (00:43:09.5)


def _edge_stub(robot, kind, track):
    r = types.SimpleNamespace(board=BOARD, exploring=False, seek=set(), edge_touch=None, guard=None, cam=None,
                              moves={}, leg_live={robot: 0.0}, leg_kind={robot: kind})
    r.border_frame = types.MethodType(E.Runner.border_frame, r)
    r.border_cm = types.MethodType(E.Runner.border_cm, r)
    r.history = _Hist([q[:3] for q in track])
    r.src_log = [(q[0], q[3]) for q in track]
    return r


def _ticks(r, robot, t_from, t_to):
    out = []
    t = t_from
    while t <= t_to + 1e-9:
        r.history.now = t
        smp = r.history.since(t - 0.6)
        if len(smp) >= 3 and t - smp[-1][0] <= 1.0:
            why = E.Runner.border_check(r, robot, smp)
            if why:
                out.append((round(t, 2), why))
        t += 0.1
    return out


def test_right_0054_one_picture_jumps_no_longer_hold_a_drive_away_from_the_edges(monkeypatch):
    """Right's `go 4` at 00:54:08 (away from both edges) was held "12 cm from an open edge, moving towards it" by two
    single pictures at x = 11.2 among 16.3-15.3 (its rescue then failed and it was left out)."""
    r = _edge_stub("Right", "go", RIGHT_0054)
    monkeypatch.setattr(E, "drop_jumps", lambda s, jump_cm=None, half=2: list(s))     # as before the fix
    old = _ticks(r, "Right", 1.3, 2.0)                                    # the go 4 was live from 00:54:08.75
    assert any("12 cm from an open edge" in w and "moving towards" in w for _, w in old)   # the logged hold
    monkeypatch.undo()
    assert _ticks(_edge_stub("Right", "go", RIGHT_0054), "Right", 1.1, 2.0) == []


def test_left_0043_three_detector_pictures_no_longer_hold_a_4_7_deg_turn():
    """Left's turn -4.7 sent at 00:43:10.8: three detector pictures at (11.3, 97.7), 5 cm from the lock at (16, 100),
    held it ("moved 5.1 cm ... now 11 cm"): the median of the last second takes them out."""
    # as logged: the start was one picture (16.41) and the check the median of the last 3 (11.27 at 00:43:11.9)
    old_last3 = [q[1] for q in LEFT_0043 if q[0] <= 2.4][-3:]
    assert abs((16.41 - sorted(old_last3)[1]) - 5.1) < 0.1                # "moved 5.1 cm", now 11.27
    r = _edge_stub("Left", "turn", LEFT_0043)
    r.turn_deg = {"Left": 4.7}
    r.history.now = 1.30                                                  # the turn's send
    r.turn_start_xy = {"Left": E.recent_median(r.history.since(1.30 - 3 * E.SPIN_MEDIAN_S))}
    assert 15.8 < r.turn_start_xy["Left"][0] < 16.4
    for phase in (0.0, 0.025, 0.05, 0.075):                              # every tick phase, not one
        assert _ticks(r, "Left", 1.4 + phase, 3.0) == []
    # and the median never comes within the allowance at any picture's moment
    allow = E.spin_allowance_cm(4.7)
    for k in range(len(LEFT_0043)):
        if LEFT_0043[k][0] > 1.3:
            m = E.recent_median([q[:3] for q in LEFT_0043[:k + 1]])
            assert r.turn_start_xy["Left"][0] - m[0] < allow - 1.0


def test_a_detector_spot_past_the_line_is_not_trusted_but_a_lock_there_still_holds():
    # moving out past y = 0 at 4 cm/s (the stage-2 nose-on-the-floor spots read -5.8 / -6.5)
    track = [(0.1 * k, 40.0, 2.0 - 0.4 * k) for k in range(8)]
    r = _edge_stub("Right", "go", [q + ("lock",) for q in track])
    r.history.now = track[-1][0]
    assert "moving further out" in E.Runner.border_check(r, "Right", track[-6:])
    r.src_log = [(q[0], "detector") for q in track]
    assert E.Runner.border_check(r, "Right", track[-6:]) == ""


def test_drop_jumps_leaves_a_steady_drive_alone():
    drive = [(0.1 * k, 30.0 - 1.0 * k, 50.0) for k in range(8)]          # 10 cm/s at 10 pictures a second
    assert E.drop_jumps(drive) == drive
    jumped = list(drive)
    jumped[4] = (0.4, 21.0, 50.0)                                         # one picture 5 cm off
    assert abs(E.drop_jumps(jumped)[4][1] - 26.0) < 1.01


# --- fix 3: own pose after a barrier-kept find --------------------------------------------------------------------
def _own_stub(own_xy):
    r = types.SimpleNamespace(st=_St(), tracker=None, history=None, own_until={"Right": 100.0})
    _say(r)
    r.own_latest_xy = lambda n: own_xy
    return r


def test_after_the_barrier_rights_nose_spot_at_y_3_6_is_not_used_its_own_pose_is():
    """00:53:53 Right's find felt a barrier; its own pose (16.8, 13.0); the camera's spot 3.6 cm from y = 0 (its nose
    at the line): two watchdog stops and a camera fix followed."""
    r = _own_stub((16.76, 12.98))
    assert E.Runner.own_pose_kept(r, "Right", cam_xy=(16.72, 3.63), now=101.0) == (16.76, 12.98)
    assert E.Runner.own_pose_kept(r, "Right", cam_xy=(17.56, 3.63), now=105.0) == (16.76, 12.98)
    # the camera comes back to the body: within 3 cm, the camera is used again
    assert E.Runner.own_pose_kept(r, "Right", cam_xy=(17.2, 14.9), now=110.0) is None
    assert "Right" not in r.own_until and r.st["own_pose_kept"][-1][2] == "agreed"


def test_the_own_pose_spell_ends_after_its_time_limit():
    r = _own_stub((16.76, 12.98))
    assert E.Runner.own_pose_kept(r, "Right", cam_xy=(16.72, 3.63), now=100.0 + E.OWN_TRUST_MAX_S + 1) is None
    assert r.st["own_pose_kept"][-1][2] == "expired"


# --- fix 4: a rescue that drives on inward ------------------------------------------------------------------------
def _rescue_stub(rec):
    r = types.SimpleNamespace(board=BOARD, exploring=False, tracker=None, st=_St(), moves={})
    _say(r)
    r.listener = types.SimpleNamespace(cv=threading.Condition(), recs={"LEG": [(1e12, "Right", rec)]})
    r.sent = []
    r.begin_moves = lambda n, why="": True
    r.fix_now = lambda n: None
    r.send_leg = lambda n, text, timeout: r.sent.append(text) or ("done", "")
    r.drive_was_inward = types.MethodType(E.Runner.drive_was_inward, r)
    return r


# Right's last forward drive before the 00:54:09 hold (leg.csv id 754450) and its rescue's start (11.87, 13.49)
RIGHT_GO4 = {"kind": "go", "value": "4.00", "result": "failed", "x_cm": "11.87", "y_cm": "13.49", "th_deg": "52.21",
             "info": "stop command"}


def test_right_0057_rescue_drives_on_inward_instead_of_into_the_corner(monkeypatch):
    monkeypatch.setattr(E.time, "time", lambda: 1e12)
    r = _rescue_stub(RIGHT_GO4)
    assert "inward from x = 0 and y = 0" in E.Runner.drive_was_inward(r, "Right", RIGHT_GO4)
    assert E.Runner.back_out(r, "Right", within_s=900.0, inward_ok=True) is True
    assert r.sent == ["go 6.0"]                                           # was `go -6.0` into its own 10 cm border
    r.sent.clear()
    assert E.Runner.back_out(r, "Right", within_s=900.0) is True and r.sent == ["go -6.0"]   # others unchanged


def test_a_drive_towards_the_edge_is_still_reversed():
    towards = dict(RIGHT_GO4, th_deg="232.2")                             # at the corner, facing it
    r = _rescue_stub(towards)
    assert E.Runner.drive_was_inward(r, "Right", towards) == ""
    along = dict(RIGHT_GO4, x_cm="11.9", y_cm="60.0", th_deg="95.0")      # along x = 0: not clearly inward
    assert E.Runner.drive_was_inward(r, "Right", along) == ""
    far = dict(RIGHT_GO4, x_cm="60.0", y_cm="60.0")                       # no edge near
    assert E.Runner.drive_was_inward(r, "Right", far) == ""
