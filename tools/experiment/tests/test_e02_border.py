"""The virtual border, holding for Nick and the battery log (Nick, 2026-10-24 evening)."""
import os
import sys
import threading
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import e02 as E  # noqa: E402
import e02_plan as PL  # noqa: E402


def _stub(board=None, exploring=False):
    r = types.SimpleNamespace()
    r.board = board or PL.Board(xboard_side=1, yboard_side=1)     # open edges: x = 0 and y = 0
    r.exploring = exploring
    r.seek = set()
    r.edge_touch = None
    r.guard = None
    r.cam = None
    r.leg_live, r.leg_kind = {"Left": 0.0}, {"Left": "go"}
    r.moves = {}
    r.border_frame = types.MethodType(E.Runner.border_frame, r)
    r.border_cm = types.MethodType(E.Runner.border_cm, r)
    return r


def _track(x0, y0, x1, y1, n=6, dt=0.1):
    return [(k * dt, x0 + (x1 - x0) * k / (n - 1), y0 + (y1 - y0) * k / (n - 1)) for k in range(n)]


def test_border_trips_moving_towards_an_open_edge_inside_the_line():
    r = _stub()
    why = E.Runner.border_check(r, "Left", _track(16.0, 60.0, 13.0, 60.0))      # 13 cm from x = 0, 6 cm/s towards it
    assert "13 cm from an open edge" in why


def test_border_lets_it_move_away_stand_still_or_stay_outside():
    r = _stub()
    assert E.Runner.border_check(r, "Left", _track(12.0, 60.0, 14.0, 60.0)) == ""     # away from the edge
    assert E.Runner.border_check(r, "Left", _track(12.0, 60.0, 12.0, 60.0)) == ""     # still
    assert E.Runner.border_check(r, "Left", _track(30.0, 60.0, 20.0, 60.0)) == ""     # outside the line
    assert E.Runner.border_check(r, "Left", _track(60.0, 30.0, 60.0, 20.0)) == ""     # the y edge, outside


def test_border_ignores_tracker_jumps_and_purposeful_edge_crawls():
    r = _stub()
    assert E.Runner.border_check(r, "Left", _track(60.0, 60.0, 10.0, 60.0)) == ""     # 100 cm/s: a jump
    r.seek.add("Left")
    assert E.Runner.border_check(r, "Left", _track(16.0, 60.0, 13.0, 60.0)) == ""     # explore's forward crawl
    r.seek.clear()
    r.edge_touch = "Left"
    assert E.Runner.border_check(r, "Left", _track(16.0, 60.0, 13.0, 60.0)) == ""     # the edge find


def test_border_while_exploring_goes_through_the_guard_camera():
    class Cam:                       # provisional camera: 1 px a cm, offset 100 px
        def project(self, p):
            return (p[0] + 100.0, p[1] + 100.0)

    class Guard:                     # the calibration file's camera: 2 px a cm
        def to_floor(self, u, v, z=0.0):
            return (u / 2.0, v / 2.0)

    r = _stub(exploring=True)
    r.board = None
    r.cam = Cam()
    r.guard = (Guard(), PL.Board(xboard_side=1, yboard_side=1))
    # provisional x from -68 to -74: pixels 32 -> 26: guard floor x 16 -> 13 cm (the stage-3 back towards the edge)
    why = E.Runner.border_check(r, "Left", _track(-68.0, 20.0, -74.0, 20.0))
    assert "13 cm" in why
    r.guard = None
    assert E.Runner.border_check(r, "Left", _track(-68.0, 20.0, -74.0, 20.0)) == ""   # no guard: nothing to go by


def test_hold_blocks_moves_and_left_out_robots_are_not_sent():
    sent = []
    r = types.SimpleNamespace(hold=threading.Event(), hold_answer=None, held_out=set(), hold_owner=None,
                              stop_requested=False,
                              link=types.SimpleNamespace(send_name=lambda n, c: sent.append((n, c))),
                              say=lambda *a, **k: None)
    r.raise_if_stopped = types.MethodType(E.Runner.raise_if_stopped, r)
    send = types.MethodType(E.Runner.send, r)
    r.hold.set()

    def answer():
        import time
        time.sleep(0.4)
        r.hold_answer = "c"
        r.hold.clear()
    threading.Thread(target=answer).start()
    send("Right", "go 10.00 5")                 # waits for the answer, then goes
    assert sent == [("Right", "go 10.00 5")]
    r.held_out.add("Right")
    send("Right", "turn 90.00 6")               # left out: never sent
    send("Right", "status")                     # answers only: still sent
    assert sent[-1] == ("Right", "status") and len(sent) == 2


def test_hold_stop_answer_ends_the_session():
    r = types.SimpleNamespace(hold=threading.Event(), hold_answer="s", held_out=set(), hold_owner=None,
                              stop_requested=False,
                              link=types.SimpleNamespace(send_name=lambda n, c: None), say=lambda *a, **k: None)
    r.raise_if_stopped = types.MethodType(E.Runner.raise_if_stopped, r)
    r.hold.set()

    def answer_s():
        r.stop_requested = True
        r.hold.clear()
    threading.Timer(0.3, answer_s).start()
    try:
        E.Runner.send(r, "Right", "go 10.00 7")
        raise AssertionError("not stopped")
    except KeyboardInterrupt:
        pass


def test_border_lets_it_drive_along_the_edge():
    r = _stub()
    # 12 cm from x = 0, 4 cm/s along the edge with 1.5 cm/s drift towards it: allowed
    assert E.Runner.border_check(r, "Left", _track(12.75, 60.0, 12.0, 62.0)) == ""
    # the same speed straight at it: held
    assert E.Runner.border_check(r, "Left", _track(14.0, 60.0, 12.0, 60.0)) != ""


def test_border_ignores_a_turn_on_the_spot():
    r = _stub()
    r.leg_live["Left"], r.leg_kind["Left"] = 0.0, "turn"
    assert E.Runner.border_check(r, "Left", _track(14.0, 60.0, 12.0, 60.0)) == ""
    r.leg_kind["Left"] = "go"
    assert E.Runner.border_check(r, "Left", _track(14.0, 60.0, 12.0, 60.0)) != ""


def test_border_ignores_a_robot_standing_still():
    r = _stub()
    r.leg_live.clear()
    assert E.Runner.border_check(r, "Left", _track(14.0, 60.0, 12.0, 60.0)) == ""      # settling, not driving
    r.moves["Left"] = dict(run="E806-30", why="U")
    assert E.Runner.border_check(r, "Left", _track(14.0, 60.0, 12.0, 60.0)) != ""      # its firmware run drives it


def test_stop_answer_is_sticky_after_the_hold():
    # 2026-10-24 review: an "s" given while the main thread was not waiting in send() was lost
    r = types.SimpleNamespace(hold=threading.Event(), held_out=set(), hold_owner=None, stop_requested=True,
                              link=types.SimpleNamespace(send_name=lambda n, c: None), say=lambda *a, **k: None)
    r.raise_if_stopped = types.MethodType(E.Runner.raise_if_stopped, r)
    E.Runner.send(r, "Right", "status")                  # answers only: still allowed
    try:
        E.Runner.send(r, "Right", "go 10.00 9")
        raise AssertionError("moved after s")
    except KeyboardInterrupt:
        pass


def test_crawl_end_window_drops_older_pictures():
    r = _stub()
    r.seek_end = {"Left": 0.25}
    # samples before 0.25 s were the crawl towards the edge; after it, the robot backs away: allowed
    assert E.Runner.border_check(r, "Left", _track(16.0, 60.0, 13.0, 60.0)[:3] + _track(13.0, 60.0, 14.0, 60.0, n=3)) == ""


class _Trust:
    def __init__(self, bad, k):
        self._bad, self._k = bad, k

    def bad(self, sensor, robot="*"):
        return self._bad and sensor == "camera"

    def k(self, sensor, robot="*"):
        return self._k if sensor == "camera" else 1.0


def test_a_bad_camera_shrinks_the_working_area_and_a_healthy_one_gives_it_back():
    r = _stub()
    assert E.Runner.border_cm(r) == E.BORDER_CM                          # no trust yet: the plain border
    r.trust = _Trust(False, 3.0)
    assert E.Runner.border_cm(r) == E.BORDER_CM                          # not flagged: unchanged
    r.trust = _Trust(True, 2.5)
    b = E.Runner.border_cm(r)
    assert E.BORDER_CM < b < E.BORDER_CM + E.BORDER_EXTRA_MAX_CM         # flagged: moved in, by its extra noise
    r.trust = _Trust(True, 40.0)
    assert E.Runner.border_cm(r) == E.BORDER_CM + E.BORDER_EXTRA_MAX_CM  # never more than the cap
    track = _track(22.0, 60.0, 19.0, 60.0)                               # 19 cm from x = 0, towards it
    assert "border is 25 cm" in E.Runner.border_check(r, "Left", track)
    r.trust = _Trust(False, 1.0)
    assert E.Runner.border_check(r, "Left", track) == ""                 # healthy again: 19 cm is inside the area


def test_a_bad_camera_leaves_the_border_to_robots_that_have_their_own():
    r = _stub()
    r.trust = _Trust(True, 2.5)
    track = _track(16.0, 60.0, 13.0, 60.0)
    r.listener = types.SimpleNamespace(firmware={"Left": "2026-10-25a"})
    assert E.Runner.border_check(r, "Left", track) == ""                 # its own border is the line
    r.listener = types.SimpleNamespace(firmware={"Left": "2026-10-22b"})
    assert "from an open edge" in E.Runner.border_check(r, "Left", track)   # older firmware: the camera still holds
