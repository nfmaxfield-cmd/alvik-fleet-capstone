"""Fixes from the 2026-10-22 morning session (docs/build-plans/e02-oct22-build.md)."""
import math

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02_plan as PL
import e02_camera as CAM


def _board():
    return PL.Board(xboard_side=1, yboard_side=1, xboard_cm=119.4, yboard_cm=123.4)


def test_park_spots_do_not_trigger_the_move_in():
    """Every park spot is 17+ cm from an open edge: leaving it must not start with 'moving it in first'."""
    b = _board()
    for i in range(3):
        x, y, _ = b.park(i)
        assert b.escape_point(x, y) is None, (i, x, y, b.edge_room(x, y))


def test_move_in_point_keeps_clear_of_the_boards():
    """Left at (19.7, 118.0), by the open edge and right against the x-board (10:41): its move-in point used to
    be 1.4 cm from the board's line, so it drove into the board and looped."""
    b = _board()
    q = b.escape_point(10.0, 118.0)
    assert q is not None
    assert abs(q[1] - b.xboard_cm) >= PL.ESCAPE_BOARD_CM - 0.01
    assert b.on_board(q[0], q[1], margin=10.0)


def test_pieces_of_one_robot_merge():
    tr = CAM.MotionTracker.__new__(CAM.MotionTracker)
    a = dict(x=10.0, y=10.0, area=300, moving=0.0)
    c = dict(x=14.0, y=10.0, area=100, moving=0.5)
    far = dict(x=40.0, y=10.0, area=300, moving=0.0)
    out = tr._merge([a, c, far])
    assert len(out) == 2
    m = [o for o in out if o["x"] < 30][0]
    assert abs(m["x"] - 11.0) < 1e-6 and m["moving"] == 0.5


def test_away_turn_points_to_the_middle():
    import e02 as E
    r = E.Runner.__new__(E.Runner)
    r.board = _board()
    ang = r.away_turn((20.0, 100.0), 90.0)       # facing +y (towards the x-board); the middle is at +x
    h = 90.0 + ang
    assert math.cos(math.radians(h)) > 0.5, ang
