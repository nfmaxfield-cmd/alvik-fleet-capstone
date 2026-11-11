"""Build e16, items 7 and 8 (home free roam 20261111-082651-freeroam, commit 4732a48, 08:45-08:48):
Right stopped at an edge and stood ("it can still back up but it's not, it's just still"), was unsure near the edge,
asked 5 times with nobody there, said NEEDS A HAND at 77, 125, then "REJOINED ... moved in" one second later with
nobody touching it, then Center drove into it and pushed it off. The real sequence replayed on the controller (the
simulator's stand-in, as the e10 incident tests); each fix and its switch. No robot, no Pi."""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
for p in (FLEET, os.path.dirname(FLEET)):
    if p not in sys.path:
        sys.path.insert(0, p)

import freeroam_core as FC        # noqa: E402
import observe as OB              # noqa: E402

HOME = (0.0, 0.0, 122.24, 121.92)


def ctrl(names=("Right",), **kw):
    sent = []
    c = FC.Controller(list(names), HOME, lambda n, s: sent.append((n, s)), lambda *a: None, pose_source="blend",
                      **kw)
    c.sent = sent
    return c


def feed(c, name, t, x, y, th, cam=True, state="idle", cam_xy=None):
    c.t = t
    c.on_follow(name, t, state, x, y, th)
    if cam:
        cx, cy = cam_xy if cam_xy is not None else (x, y)
        c.on_fix(name, t, cx, cy, None)


def drive_in(c, name="Right", x0=77.0, y_end=118.5, bend=True, t0=1.0):
    """Right's way in: a curve that straightens onto heading 90 (towards the y = 122 edge), ending y_end."""
    t, k = t0, 0
    pts = []
    y = y_end - 40.0
    while y <= y_end + 1e-9:
        x = x0 - (0.004 * (y_end - y) ** 2 if bend else 0.0)
        pts.append((x, y))
        y += 1.0
    for i, (x, y) in enumerate(pts):
        if i == 0:
            th = 90.0
        else:
            th = math.degrees(math.atan2(y - pts[i - 1][1], x - pts[i - 1][0]))
        feed(c, name, t, x, y, th)
        t += 0.1
        k += 1
    return t


# ------------------------------------------------------------------------------------------------------- item 7
def test_track_back_target_reverses_over_the_track_as_far_as_the_rule_needs():
    pts = [(50.0, 80.0 + i, 90.0) for i in range(40)]                  # straight in, now at y = 119
    tgt = OB.track_back_target(pts, (50.0, 119.0), 90.0, lambda q: 121.92 - q[1] >= 14.0)
    assert tgt["cleared"] and abs(tgt["end"][1] - (121.92 - 14.0)) < 1.01 and 10.0 < tgt["L"] <= 12.01
    # a curved track: only the chord that stays on it, then it goes on from there
    cur = [(50.0 - 0.02 * (39 - i) ** 2, 80.0 + i, 90.0) for i in range(40)]
    for i in range(1, 40):
        cur[i] = (cur[i][0], cur[i][1], math.degrees(math.atan2(cur[i][1] - cur[i - 1][1], cur[i][0] - cur[i - 1][0])))
    tgt2 = OB.track_back_target(cur, (cur[-1][0], cur[-1][1]), cur[-1][2], lambda q: 121.92 - q[1] >= 40.0)
    assert tgt2 is not None and not tgt2["cleared"] and tgt2["L"] < 20.0
    assert OB.track_back_target([(1, 1, 0)], (1, 1), 0.0, lambda q: True) is None       # no track: none


def test_an_edge_stop_backs_out_along_its_own_track_then_replans():
    c = ctrl()
    r = c.R["Right"]
    r.curve, r.fw = True, "2026-11-05d"
    t = drive_in(c, y_end=118.5)
    r.mode = "idle"
    c.felt(r, "plywood edge at 78.5 120.5 -9.0 bottom")
    assert r.back_off and r.back_off["edge"]
    c.start_curve(r)
    assert r.move is not None and r.move["kind"] == "reverse" and r.move["rev"] and r.move["track_back"]
    need = c.end_need(r, FC.BACK_V)
    assert c.counts.get("track_backs") == 1
    assert FC.edge_dist(HOME, *r.move["end"]) >= need - 0.5 or r.back_off is not None    # clear, or the next chord
    assert any(" 1 -" in s for n, s in c.sent if s.startswith("fs"))                      # a reverse drive


def test_the_old_way_behind_the_switch(monkeypatch):
    monkeypatch.setattr(FC, "EDGE_BACKOUT", False)
    c = ctrl()
    r = c.R["Right"]
    r.curve, r.fw = True, "2026-11-05d"
    drive_in(c, y_end=118.5)
    r.mode = "idle"
    c.felt(r, "plywood edge at 78.5 120.5 -9.0 bottom")
    c.start_curve(r)
    assert c.counts.get("track_backs", 0) == 0
    assert r.move is None or not r.move.get("track_back")


def test_a_robot_on_its_track_behind_blocks_the_reverse_and_it_holds():
    c = ctrl(("Right", "Center"))
    r = c.R["Right"]
    r.curve, r.fw = True, "2026-11-05d"
    t = drive_in(c, y_end=118.5)
    feed(c, "Center", t, 77.0, 104.0, 0.0)                      # standing on Right's track, 14 cm behind it
    c.R["Center"].mode = "idle"
    c.R["Center"].traj = FC.Traj.stand(c.t, 77.0, 104.0, m=c.margin0(c.R["Center"]))
    r.mode = "idle"
    assert c.track_backout(r) is None and c.counts.get("track_back_blocked") == 1


# ------------------------------------------------------------------------------------------------------- 8a
def _held_right(c, t=1.0, n=12, jitter=0.6):
    r = c.R["Right"]
    for k in range(n):
        j = jitter * (1 if k % 2 else -1)
        feed(c, "Right", t + 0.1 * k, 77.0 + j, 116.0, 90.0, cam_xy=(77.0 - j, 116.0 + j))
    r.mode = "idle"
    r.unsure = dict(t=c.t, why="its own pose 6.8 cm from the camera's", tried=0)
    c.edge_stuck_since = None
    r.edge_stuck_since = c.t - 10.0
    assert c.need_hand_watch(r) is False and c.needs_hand()
    return r, c.t


def test_no_rejoin_without_evidence_the_home_0848_case():
    c = ctrl()
    r, t = _held_right(c)
    for k in range(30):                                        # nobody touches it
        feed(c, "Right", t + 0.1 * (k + 1), 77.0, 116.0 + (0.5 if k % 2 else -0.5), 90.0)
        c.need_hand_watch(r)
    r.unsure = None                                            # the unsure state clears (the judgement flips)
    for k in range(5):
        feed(c, "Right", t + 3.1 + 0.1 * k, 77.0, 116.0, 90.0)
        c.need_hand_watch(r)
    assert c.counts.get("hand_rejoined", 0) == 0 and c.needs_hand() and c.counts.get("rejoin_refused", 0) >= 1
    assert c.pinned(r)                                          # 8b: still a fixed obstacle


def test_a_real_hand_move_is_a_rejoin():
    c = ctrl()
    r, t = _held_right(c)
    for k in range(20):                                        # still held, then moved 20 cm in by a hand
        feed(c, "Right", t + 0.1 * (k + 1), 77.0, 116.0 + (0.3 if k % 2 else -0.3), 90.0)
        c.need_hand_watch(r)
    for k in range(6):
        feed(c, "Right", t + 2.5 + 0.1 * k, 77.0, 96.0 + (0.3 if k % 2 else -0.3), 90.0)
        c.need_hand_watch(r)
    r.unsure = None
    c.need_hand_watch(r)
    assert c.counts.get("hand_rejoined") == 1 and not c.needs_hand()
    assert c.pinned(r)                                          # rejoined, not driven away yet


def test_a_lift_is_evidence_too():
    c = ctrl()
    r, t = _held_right(c)
    feed(c, "Right", t + 2.5, 77.0, 96.0, 90.0)
    c.on_raw_motion("Right", c.t, 0.0, 0.0, lifted=True)
    for k in range(4):
        feed(c, "Right", t + 2.6 + 0.1 * k, 77.0, 96.0, 90.0)
    r.unsure = None
    c.need_hand_watch(r)
    assert c.counts.get("hand_rejoined") == 1


def test_the_old_rejoin_behind_the_switch(monkeypatch):
    monkeypatch.setattr(FC, "REJOIN_EVIDENCE", False)
    c = ctrl()
    r, t = _held_right(c)
    r.unsure = None
    c.need_hand_watch(r)
    assert c.counts.get("hand_rejoined") == 1


def test_moved_in_evidence_ignores_jitter():
    held = [(77.0 + (0.8 if k % 2 else -0.8), 116.0) for k in range(20)]
    ok, why = OB.moved_in_evidence(held, [(77.0, 119.0)] * 3)        # 3 cm: inside 4 x sqrt2 x 0.8 = 4.5 cm
    assert not ok and "limit" in why
    ok, _ = OB.moved_in_evidence(held, [(77.0, 104.0)] * 3)
    assert ok
    ok, _ = OB.moved_in_evidence(held, [(77.0, 116.0), (77.0, 104.0), (77.0, 116.0)])   # one jump: not sustained
    assert not ok


# ------------------------------------------------------------------------------------------------------- 8b
def test_a_pinned_robot_is_kept_clear_with_its_doubt():
    c = ctrl(("Right", "Center"))
    r, t = _held_right(c)
    feed(c, "Right", c.t + 0.1, 77.0, 116.0, 90.0, cam_xy=(77.0, 109.5))   # own 6.5 cm from the camera's
    ce = c.R["Center"]
    feed(c, "Center", c.t, 40.0, 100.0, 0.0)
    ce.mode = "idle"
    assert c.pinned(r) and c.pinned_extra(r) >= 6.0
    p = c.pos(r)
    # Center's planned drive passes 17 cm from where the run puts Right: fine for a free robot, not for a pinned one
    tr = FC.Traj.move(c.t + 0.2, (40.0, p[1] - 17.5), 0.0, 0.0, 70.0, 7.5, m0=c.margin0(ce))
    assert not c.feasible(ce, tr)
    assert c.need(ce, r) >= FC.REFLEX_CC + 6.0
    import importlib
    old = FC.PINNED_OBSTACLE
    try:
        FC.PINNED_OBSTACLE = False
        assert c.pinned_extra(r) == 0.0 and c.need(ce, r) == FC.REFLEX_CC
    finally:
        FC.PINNED_OBSTACLE = old


# ------------------------------------------------------------------------------------------------------- 8c
def test_a_standing_robot_pushed_stops_the_robot_near_it():
    c = ctrl(("Right", "Center"))
    rr, ce = c.R["Right"], c.R["Center"]
    t = 1.0
    for k in range(40):                                        # Right stands, noise 0.4 cm
        feed(c, "Right", t, 77.0 + (0.4 if k % 2 else -0.4), 110.0, 90.0)
        rr.mode = "idle"
        c.contact_watch(rr)
        t += 0.1
    ce.move = dict(kind="menu", phase="drive", end=(77.0, 105.0), v=7.5, rev=False)
    ce.mode = "move"
    feed(c, "Center", t, 77.0, 95.0, 90.0, state="drive")
    ce.mode = "move"
    for k in range(6):                                          # Right's spot moves 4 cm and stays (a push)
        feed(c, "Right", t + 0.1 * k, 77.0, 114.0, 90.0)
        rr.mode = "idle"
        c.contact_watch(rr)
    assert c.counts.get("contacts", 0) >= 1
    assert ce.move is None and ce.back_off is not None          # Center stopped and backs away


def test_a_driving_robot_whose_camera_lags_its_wheels_stops():
    c = ctrl()
    r = c.R["Right"]
    r.move = dict(kind="menu", phase="drive", end=(60.0, 100.0), v=10.0, rev=False, t_drive=0.0)
    r.mode = "move"
    for k in range(15):                                        # wheels 10 cm/s, the camera sees 1 cm/s
        feed(c, "Right", 1.0 + 0.1 * k, 40.0 + 1.0 * k, 60.0, 0.0, cam_xy=(40.0 + 0.1 * k, 60.0), state="drive")
        r.mode = "move"
    c.contact_watch(r)
    assert c.counts.get("contacts", 0) == 0                      # one slow check is not enough
    c.contact_watch(r)
    assert c.counts.get("contacts") == 1 and r.move is None and r.back_off is not None


def test_an_accelerometer_jolt_while_standing_is_a_contact():
    c = ctrl()
    r = c.R["Right"]
    feed(c, "Right", 1.0, 60.0, 60.0, 0.0)
    r.mode = "idle"
    for k in range(40):
        c.on_raw_motion("Right", 1.0 + 0.02 * k, 0.003 * (1 if k % 2 else -1), 0.002, False)
    assert c.counts.get("contacts", 0) == 0
    c.on_raw_motion("Right", 2.0, 0.25, 0.1, False)
    assert c.counts.get("contacts") == 1


# ------------------------------------------------------------------------------------------------------- 8d
class Asks:
    def __init__(self):
        self.n = 0

    def ask(self, name, text, choices):
        self.n += 1
        return "q%d" % self.n

    def answer(self, q):
        return None

    def close(self, q, why):
        pass


def test_a_question_nobody_answers_makes_the_run_decide_by_itself():
    a = Asks()
    c = ctrl(attended=True, questions=a)
    r = c.R["Right"]
    feed(c, "Right", 1.0, 60.0, 60.0, 90.0)
    c.ask(r, "Is it near the edge?", ("near", "clear"))
    assert a.n == 1 and c.attended_live()
    c.t = 1.0 + FC.ASK_WAIT_S + 0.1
    c.question_watch(r)
    assert not c.attended_live() and c.counts.get("unanswered") == 1
    c.ask(r, "Is it near the edge?", ("near", "clear"))
    assert a.n == 1                                              # not asked again: it decides by itself


def test_keep_asking_behind_the_switch(monkeypatch):
    monkeypatch.setattr(FC, "ASK_UNATTENDED", True)
    a = Asks()
    c = ctrl(attended=True, questions=a)
    r = c.R["Right"]
    feed(c, "Right", 1.0, 60.0, 60.0, 90.0)
    c.ask(r, "q", ("a",))
    c.t = 100.0
    c.question_watch(r)
    assert c.attended_live()


# ------------------------------------------------------------------------------------------------------- 7, missions
def test_missions_back_out_along_the_track_as_far_as_the_rule_needs(monkeypatch):
    import types
    import run_plan as RP
    logs = []
    stub = types.SimpleNamespace(m={"floor": dict(x_min=0.0, y_min=0.0, x_max=122.24, y_max=121.92)},
                                 args=types.SimpleNamespace(edge_tracking="lights"), fp_live=lambda b: False,
                                 log=lambda kind, **kw: logs.append((kind, kw)))
    tr = OB.Trail()
    for i in range(50):
        tr.add(60.0, 70.0 + i, 90.0)
    b = types.SimpleNamespace(name="Right", trail=tr, last_cam=(60.0, 119.0, 90.0))
    cls = next(v for v in vars(RP).values() if isinstance(v, type) and hasattr(v, "track_back"))
    tgt = cls.track_back(stub, b, 60.0, 119.0, 90.0, [])
    import safety_rules as SR
    need = SR.clearance_cm(SR.CAM_SIGMA_CM, 5.0, "lights", stops_here=True)
    assert tgt["cleared"] and abs((121.92 - tgt["end"][1]) - need) < 1.01 and logs[0][0] == "track_back"
    assert cls.track_back(stub, b, 60.0, 119.0, 90.0, [(60.0, 110.0)]) is None        # a robot on its track


# ------------------------------------------------------------------------------------------------------- 6a
def test_every_robot_held_never_ends_the_run_by_default(monkeypatch):
    import test_experiments as TE
    import run_plan as RP
    monkeypatch.setattr(RP, "HELD_NEVER_END", True)
    c = TE._coord()
    for _ in range(5):                                   # held again and again: no cap
        c.retire(c.bots["Bravo"], "nothing heard", kind="silent")
        c.retire(c.bots["Alpha"], "nothing heard", kind="silent")
        assert not c.abort
    c.t_run = RP.now() - 500.0
    for b in c.bots.values():
        b.out_t = RP.now() - 400.0                       # far past the old 90 s wait
    c.planning = None
    c.rejoin_watch()
    assert not c.abort and any("never ends by itself" in s for s in c.said)
