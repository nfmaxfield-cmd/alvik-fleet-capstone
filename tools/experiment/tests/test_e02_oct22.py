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
    d, what, _ = r.ahead_cm("Left", (60.0, 100.0), 90.0)          # facing +y: the x-board at 119.4
    assert what == "the x-board" and abs(d - (119.4 - 100.0 - PL.ROBOT_HALF_CM)) < 1e-6
    d, what, _ = r.ahead_cm("Left", (60.0, 20.0), 90.0)           # Center 30 cm ahead, in the way
    assert what == "Center" and abs(d - (30.0 - 2 * PL.ROBOT_HALF_CM)) < 1e-6
    d, what, _ = r.ahead_cm("Left", (60.0, 60.0), 0.0)            # facing +x: the y-board at 123.4
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


def test_tracker_area_stops_at_the_walls():
    """The live tracker now has the board area (it had none: it followed hands 47 cm off the board) and stops a
    robot's centre a little past its wall limit."""
    tr = CAM.MotionTracker.__new__(CAM.MotionTracker)
    import threading
    tr.lock = threading.Lock()
    tr.margin = 8.0
    tr.set_area((0.0, 0.0, 122.15, 121.9), walls=[("y", 114.66), ("x", 118.68)])
    assert tr.on_area(60.0, 60.0)
    assert tr.on_area(60.0, -5.0)                      # an open edge: margin allowed
    assert not tr.on_area(60.0, 114.66 + tr.WALL_CM + 0.5)
    assert not tr.on_area(-47.0, 112.0)                # where the hands were followed, 10:57


def test_no_method_is_defined_twice():
    """2026-10-22: a new recover() was silently replaced by an older method of the same name further down the class,
    and a help call looped forever. Every class in the runner's modules defines each method once."""
    import ast
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for fn in sorted(os.listdir(here)):
        if not (fn.startswith("e02") and fn.endswith(".py")):
            continue
        tree = ast.parse(open(os.path.join(here, fn)).read())
        for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
            names = [f.name for f in cls.body if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef))]
            dup = sorted({n for n in names if names.count(n) > 1})
            assert not dup, (fn, cls.name, dup)


def test_move_in_point_keeps_clear_of_both_open_edges():
    """Review 2026-10-22: in the corner where the two open edges meet, the point was moved in from one only."""
    b = _board()
    q = b.escape_point(5.0, 5.0)
    assert q is not None
    for a, v in b.open_edges():
        assert abs((q[0] if a == "x" else q[1]) - v) >= 27.9, (q, a, v)


def test_front_limits_from_data_unless_learning():
    import e02 as E
    import types
    r = E.Runner.__new__(E.Runner)
    r.a = types.SimpleNamespace(front_learn=False)
    r.front_seen = {"all": [4.0] * 20}
    assert r.front_limits() == (E.FRONT_OFFSET_CM, E.FRONT_TOL_CM)
    r.a.front_learn = True
    off, tol = r.front_limits()
    assert off == 4.0 and tol == E.FRONT_LEARN_MIN_TOL


def test_drive_blind_stops_when_landmarks_disagree():
    """Driving on its own pose: the front sensor reading far from what the map says ahead ends the drive."""
    import e02 as E
    import types
    r = E.Runner.__new__(E.Runner)
    r.board = _board()
    r.a = types.SimpleNamespace(front_learn=False)
    r.front_seen = {}
    r.st = {"positions": {}}
    r.begin_moves = lambda *a, **k: True
    pose = {"p": (60.0, 80.0, 90.0)}                    # facing the x-board (119.4): 34.6 cm ahead of its front
    r.own_pose = lambda robot, since=0.0: pose["p"]

    def send_leg(robot, text, timeout):
        v, a = text.split()
        if v == "go":
            x, y, th = pose["p"]
            pose["p"] = (x, y + float(a), th)
        return "done", "front=5.0"                       # something right in front: not on the map
    r.send_leg = send_leg
    ok, words = r.drive_blind("Left", [(60.0, 100.0)])
    assert not ok and "front sensor" in words


def _search_runner():
    import e02 as E
    import types
    r = E.Runner.__new__(E.Runner)
    r.board = _board()
    r.a = types.SimpleNamespace(front_learn=False)
    r.st = {"robots": ["Left", "Center", "Right"],
            "positions": {"Center": [110.0, 17.0, 90.0], "Right": [17.0, 76.0, 0.0], "Left": [19.0, 108.0, None]}}
    r.unavailable = set()
    r.heads = {}
    r.moves = {}
    r.say = r.mark = r.send = lambda *a, **k: None
    r.activate = lambda n: True
    r.set_corrections = lambda *a, **k: None
    r.move_corrections = lambda n: None
    r.board_params = lambda: []
    r.do_find = lambda n, order: {"ok": True}
    r.parked = []
    r.park = lambda n: (r.parked.append(n), (True, ""))[1]
    r.help_from_nick = lambda *a, **k: True
    return r, E


def test_lookouts_skip_robots_and_go_nearest_first():
    r, E = _search_runner()
    pts = r.lookouts("Center", "Left")
    assert len(pts) >= 3
    for q in pts:
        assert r.board.edge_room(q[0], q[1])[0] >= 30.0           # all well in from the open edges
        for n in ("Right",):
            o = r.st["positions"][n]
            assert math.hypot(q[0] - o[0], q[1] - o[1]) >= E.SEARCH_CLEAR_CM


def test_search_finds_the_lost_robot_and_parks_the_searcher():
    """Nick, 2026-10-22: rather than stop, another robot searches: its own find, lookouts, a sweep at each."""
    r, E = _search_runner()
    lost_at = (60.0, 60.0)
    where = {"p": (110.0, 17.0)}

    def drive_to(n, lp, **k):
        where["p"] = lp
        r.heads[n] = [0.0, True]
        return True, ""
    r.drive_to = drive_to
    r.settle = lambda **k: where["p"]

    def sweep_hits(n, q, h, lost):
        if math.hypot(q[0] - lost_at[0], q[1] - lost_at[1]) < 70:
            return [(20.0, lost_at[0] + 1.0, lost_at[1])], h
        return [], h
    r.sweep_hits = sweep_hits
    r.identify = lambda robot, near=None: (near[0], near[1]) if near and math.hypot(
        near[0] - lost_at[0], near[1] - lost_at[1]) < 5 else None
    p = r.search_for("Left")
    assert p is not None and math.hypot(p[0] - lost_at[0], p[1] - lost_at[1]) < 3
    assert r.parked == ["Center"]


def test_search_takes_a_blocked_drive_as_a_hit():
    r, E = _search_runner()
    r.drive_to = lambda n, lp, **k: (r.heads.__setitem__(n, [90.0, True]), (False, "something keeps blocking its way"))[1]
    r.settle = lambda **k: (60.0, 40.0)
    r.sweep_hits = lambda *a: ([], 0.0)
    r.identify = lambda robot, near=None: None              # the camera can't see it there either
    p = r.search_for("Left")
    assert p is not None and abs(p[0] - 60.0) < 1 and p[1] > 50.0  # just ahead of the stopped searcher
