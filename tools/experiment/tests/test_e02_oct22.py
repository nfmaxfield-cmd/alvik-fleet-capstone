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


def test_ahead_by_the_map():
    """The front-sensor check's expectation: the nearest board line or robot straight ahead, from the front face."""
    import e02 as E
    r = E.Runner.__new__(E.Runner)
    r.board = _board()
    r.st = {"positions": {"Center": [60.0, 50.0, None]}}
    d, what = r.ahead_cm("Left", (60.0, 100.0), 90.0)          # facing +y: the x-board at 119.4
    assert what == "the x-board" and abs(d - (119.4 - 100.0 - PL.ROBOT_HALF_CM)) < 1e-6
    d, what = r.ahead_cm("Left", (60.0, 20.0), 90.0)           # Center 30 cm ahead, in the way
    assert what == "Center" and abs(d - (30.0 - 2 * PL.ROBOT_HALF_CM)) < 1e-6
    d, what = r.ahead_cm("Left", (60.0, 60.0), 0.0)            # facing +x: the y-board at 123.4
    assert what == "the y-board"


def test_finder_sweep_points_at_the_lost_robot():
    """A parked robot turns in 30-deg steps; the reading that is short of the map is the lost robot."""
    import e02 as E
    r = E.Runner.__new__(E.Runner)
    r.board = _board()
    lost_at = (60.0, 60.0)
    r.st = {"robots": ["Left", "Center"], "positions": {"Center": [100.0, 60.0, 0.0], "Left": [10.0, 10.0, None]}}
    r.heads = {"Center": [0.0, True]}
    r.unavailable = set()
    r.moves = {}
    r.say = lambda *a, **k: None
    r.mark = lambda *a, **k: None
    r.begin_moves = lambda *a, **k: True
    state = {"h": 0.0}

    def send_leg(robot, text, timeout):
        state["h"] = E.ST.wrap180(state["h"] + float(text.split()[1]))
        ux, uy = math.cos(math.radians(state["h"])), math.sin(math.radians(state["h"]))
        ox, oy = lost_at[0] - 100.0, lost_at[1] - 60.0
        t = ox * ux + oy * uy
        off = abs(-ox * uy + oy * ux)
        if t > 0 and off < PL.ROBOT_HALF_CM:
            front = t - 2 * PL.ROBOT_HALF_CM
        else:
            a = r.ahead_cm("Center", (100.0, 60.0), state["h"])
            front = a[0] if a else 300.0
        return "done", "front=%.1f" % front
    r.send_leg = send_leg
    seen = {}
    r.identify = lambda robot, near=None: seen.setdefault("near", near) and (near[0], near[1])
    p = r.sweep_for("Left")
    assert p is not None
    assert math.hypot(p[0] - lost_at[0], p[1] - lost_at[1]) < 3.0, p
