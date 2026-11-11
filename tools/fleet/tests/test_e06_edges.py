"""Build e06: the edge-safety rules in the free roam controller (rules 3', 3b, 4' and the side-edge invariant), driven
through the controller as the robots' records would drive it (the cases Nick and the 4 h analysis named)."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import freeroam_core as FC   # noqa: E402
import observe as OB         # noqa: E402

BOARD = (0.0, 0.0, 122.24, 121.92)


def ctrl(names=("Left",)):
    sent, rows = [], []
    c = FC.Controller(list(names), BOARD, lambda n, t: sent.append((n, t)), lambda tb, r: rows.append((tb, r)), seed=3)
    return c, sent, rows


def place(c, name, x, y, th, t=0.0, fw="2026-11-05d", fix=True):
    c.t = t
    c.on_fw(name, fw)
    c.on_heard(name, t)
    c.on_battery(name, 100, t)
    c.on_follow(name, t, "idle", x, y, th, 0)
    if fix:
        c.on_fix(name, t, x, y, th)
    c.begin(name)


def events(rows, kind):
    return [r for tb, r in rows if tb == "events" and r.get("kind") == kind]


def test_camera_washed_out_stuck_reverse_uses_its_own_trail():
    """The camera washed out (no fixes) while the robot stands near the x = 0 edge after a front stop: a reverse is
    allowed only over the ground it has just driven (its trail), never blind beyond it."""
    c, sent, rows = ctrl()
    r = c.R["Left"]
    place(c, "Left", 60.0, 60.0, 180.0)
    for k in range(1, 31):                         # it drove 30 cm towards x = 0 (its own pose)
        c.t = 0.1 * k
        c.on_follow("Left", c.t, "drive", 60.0 - k, 60.0, 180.0, 0)
    c.start_lost(r)                                 # the camera washed out
    res = c.back_move(r, Ls=(14.0, 8.0))
    assert res is not None and res[0] == "back off"  # along its trail: no camera needed
    r.trail = OB.Trail()                             # no trail (e.g. it turned there): nothing shows the way back
    r.trail.add(30.0, 60.0, 180.0)
    assert c.back_move(r, Ls=(14.0, 8.0)) is None


def test_instant_done_is_a_failure_and_is_not_repeated():
    """A firmware that answers every move 'done' at once (Left's 4,289 reverses): impossible = failure; the same
    reverse is not repeated unchanged; three failures at one spot = a hazard kept out for everyone."""
    c, sent, rows = ctrl()
    r = c.R["Left"]
    place(c, "Left", 30.0, 60.0, 0.0)
    for n in range(4):
        c.t += 0.5
        c.on_fix("Left", c.t, 30.0, 60.0, 0.0)
        mv = dict(kind="back off", cell=None, turn=0.0, L=14.0, v=5.0, t0=c.t, p0=(30.0, 60.0), h0=0.0, h1=0.0,
                  end=(16.0, 60.0), phase="drive", n=n + 1, drive_id=100 + n, turn_id=None, t_drive=c.t, rev=True)
        r.move, r.mode = mv, "move"
        r.steps[100 + n] = {"status": "started"}
        c.t += 0.05
        c.on_step("Left", 100 + n, "done", "")
    assert len(events(rows, "did not move")) >= 3
    assert events(rows, "hazard")
    assert any(q["src"] == "hazard" for q in c.keepouts)


def test_wheels_slip_caught_by_accelerometer_and_front_range():
    """Wheels turning, robot not moving (slip): its own pose says 14 cm, the accelerometer felt no rolling and the
    front range to the wall ahead did not change: NOT done, even with the camera blind."""
    c, sent, rows = ctrl()
    r = c.R["Left"]
    place(c, "Left", 60.0, 60.0, 0.0)
    c.start_lost(r)
    t0 = c.t
    mv = dict(kind="menu", cell=None, turn=0.0, L=14.0, v=5.0, t0=t0, p0=(60.0, 60.0), h0=0.0, h1=0.0,
              end=(74.0, 60.0), phase="drive", n=1, drive_id=7, turn_id=None, t_drive=t0, rev=False)
    r.move, r.mode = mv, "move"
    r.steps[7] = {"status": "started"}
    for k in range(30):
        c.t = t0 + 0.1 * k
        c.on_follow("Left", c.t, "drive", 60.0 + 14.0 * k / 29, 60.0, 0.0, 7)   # the wheels' count
        c.on_raw_evidence("Left", c.t, 0.001, 40.0, 50)                         # no jolt; front range unchanged
    c.on_step("Left", 7, "done", "")
    assert events(rows, "did not move")


def test_shallow_side_approach_refused_by_the_invariant():
    """Center 5 Nov: a drive 27 deg off the x = 0 edge line, ending 5 cm from it: its possible-locations circle would
    cross the edge on the way, so the move is never planned."""
    c, sent, rows = ctrl()
    r = c.R["Left"]
    place(c, "Left", 21.0, 74.0, -117.0)
    tr = c.make(r, 0.0, 33.0, 10.0)
    assert not c.invariant_ok(r, tr)
    tr2 = c.make(r, 117.0, 30.0, 10.0)               # turn to face +x and drive inwards: fine
    assert c.invariant_ok(r, tr2)


def test_clock_ratchet_stale_camera_grows_the_circle(monkeypatch):
    """The clock ratchet (4 h run): every fix arrives 1.3 s stale; at 10 cm/s that is 13 cm of doubt, so a robot
    25 cm from an edge may only move inwards. (e16, 9b) Only for a robot that may have moved: one standing since its
    fix keeps the fix's own doubt (MIRLT_STAND_UNC); this test pins the old way to keep checking the edge rule."""
    c, sent, rows = ctrl()
    r = c.R["Left"]
    place(c, "Left", 25.0, 60.0, 180.0)
    r.cam_arr = c.t - 1.3
    c.t += 0.0
    if c.standing(r):
        assert c.unc(r) < 2.0                       # (e16, 9b) standing: no growth
    monkeypatch.setattr(FC, "STAND_UNC", False)
    assert c.unc(r) > 13.0
    assert not c.invariant_ok(r, c.make(r, 0.0, 6.0, 10.0))         # further towards x = 0: refused
    assert c.invariant_ok(r, c.make(r, 180.0, 20.0, 10.0))          # turn and go inwards: shrinks the overlap


def test_tilt_never_forward_and_back_off_needs_evidence():
    """A standing tilt (a wheel over the edge, 3.4 deg under the 5 deg stop): the next move is a back-off over
    evidence, never forward; with no evidence it stands."""
    c, sent, rows = ctrl()
    r = c.R["Left"]
    place(c, "Left", 9.0, 60.0, 180.0)
    for k in range(OB.LEVEL_N):
        c.on_raw("Left", 0.01 * k, 0.0, 0.0, 0.0)
    t = 1.0
    for k in range(150):
        c.on_raw("Left", t + 0.02 * k, 3.4, 1.9, 0.0)
    assert events(rows, "tilt stop")
    assert r.tilted and r.back_off is not None
    r.trail = OB.Trail()                             # no trail behind, camera fresh but the back is at room ...
    c.t = 5.0
    c.on_fix("Left", c.t, 9.0, 60.0, 180.0)
    c.R["Left"].busy_until = 0
    r.mode = "idle"
    c.start_move(r)
    if r.move is not None:
        # (e08, A3) the recovery tries: a reverse inward (the camera and its own pose agree backing goes inward) or a
        # short pivot; never a forward drive
        assert r.move["kind"] in ("back off", "reverse") or (str(r.move["kind"]).startswith("recover:") and
                                                              (r.move["rev"] or r.move["L"] == 0))
    else:
        assert events(rows, "trophy break")                    # stands, said
