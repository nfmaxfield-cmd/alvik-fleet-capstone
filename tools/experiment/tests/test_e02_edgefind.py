"""The edge find (2026-10-23): locating against any two boundaries, boards or open edges, with no camera."""
import math
import random

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02_edgefind as EF
import e02_map as MAP

W, H = 122.15, 121.9


def simulate(start, own_err_deg, a, b, kinds, rng):
    """A robot at start (board frame x, y, th) whose own frame is turned own_err_deg and shifted from the board's.
    Drives the edge find's touches perfectly; returns touches in its own frame and the truth transform."""
    rot = math.radians(-own_err_deg)                 # board -> own
    ox, oy = rng.uniform(-30, 30), rng.uniform(-30, 30)

    def own(x, y, th):
        c, s = math.cos(rot), math.sin(rot)
        return (c * x - s * y + ox, s * x + c * y + oy, EF.wrap(th - own_err_deg))

    x, y, _ = start
    touches = []
    for on, bd, side_off in (("A", a, 0.0), ("A", a, EF.sidestep_sign(a, b, x, y) * EF.SIDESTEP_CM), ("B", b, 0.0)):
        kind = kinds[bd[0]]
        h = EF.facing(bd, x, y)
        ahead = MAP.LINE_AHEAD_CM if kind == "edge" else MAP.TOF_AHEAD_CM + MAP.SAFE_FRONT_CM
        # the stop pose: the axle `ahead` short of the boundary, at the point along it
        if bd[0] == "x":
            sx = bd[1] - ahead * math.cos(math.radians(h))
            sy = y + (side_off if on == "A" else 0.0) * (1 if h == 0.0 else -1)
        else:
            sy = bd[1] - ahead * math.sin(math.radians(h))
            sx = x - (side_off if on == "A" else 0.0) * (1 if h == 90.0 else -1)
        ux, uy, uth = own(sx, sy, h)
        touches.append(dict(x=ux + rng.gauss(0, 0.2), y=uy + rng.gauss(0, 0.2), th=uth, kind=kind, on=on))
    return touches, (own_err_deg, ox, oy), own


def test_edge_find_recovers_pose_boards_and_edges():
    rng = random.Random(3)
    for kinds in ({"x": "board", "y": "board"}, {"x": "edge", "y": "board"}, {"x": "board", "y": "edge"}):
        for _ in range(20):
            x, y = rng.uniform(30, 90), rng.uniform(30, 90)
            sides = [("x", 0.0, "edge"), ("x", W, kinds["x"]), ("y", 0.0, "edge"), ("y", H, kinds["y"])]
            a, b = EF.choose_boundaries(x, y, sides)
            err = rng.uniform(-25, 25)
            touches, truth, own = simulate((x, y, 0.0), err, a, b, {a[0]: a[2], b[0]: b[2]}, rng)
            sol = EF.solve(touches, a, b)
            assert sol is not None
            assert abs(EF.wrap(sol["rot"] - err)) < 3.0
            # a pose in its own frame maps back to the truth
            ox, oy, oth = own(x, y, 45.0)
            bx, by, bth = EF.to_board(sol, ox, oy, oth)
            assert math.hypot(bx - x, by - y) < 1.5, (kinds, a, b, bx - x, by - y)
            assert abs(EF.wrap(bth - 45.0)) < 3.0


def test_choose_boundaries_avoids_two_open_edges():
    sides = [("x", 0.0, "edge"), ("x", W, "board"), ("y", 0.0, "edge"), ("y", H, "board")]
    a, b = EF.choose_boundaries(10.0, 10.0, sides)          # near the open corner
    assert "board" in (a[2], b[2])
    a, b = EF.choose_boundaries(100.0, 100.0, sides)        # near the blocked corner: both boards
    assert a[2] == b[2] == "board"


def test_sidestep_moves_away_from_b():
    a, b = ("x", W, "board"), ("y", H, "board")
    x, y = 100.0, 100.0
    sgn = EF.sidestep_sign(a, b, x, y)
    # facing +x, left is +y (toward B at y = H): the sidestep must go right (-1)
    assert sgn == -1


def test_sidestep_goes_where_there_is_room():
    """2026-10-23 dry run: facing the x = 122 edge at y = 21, away from the far board took it into the y = 0 edge."""
    sides = [("x", 0.0, "edge"), ("x", 121.9, "edge"), ("y", 0.0, "edge"), ("y", 122.4, "board")]
    sgn, room = EF.sidestep_room(("x", 121.9, "edge"), 113.7, 20.9, sides)
    # facing +x, left is +y: room 101 cm that way, 21 cm the other
    assert sgn == 1 and room > 100
