"""The edge find (Nick, 2026-10-23): a robot works out where it is by touching any two boundaries that meet at a
right angle, boards or open edges alike, with no camera. For the remove-a-board test: take a board away without
telling the robots, and they still locate themselves (the open edge where the board was is felt by the edge
sensors instead of the front sensor, and the map is corrected).

The touches: boundary A twice, a sidestep apart (that gives the heading against A), then boundary B once (the
other coordinate). Each touch ends where the robot itself stops: the front stop at a board, the edge guard at an
open edge (e02_map.contact_point: the point it met, a little ahead of its axle). Between touches the robot's own
wheels and gyro carry the pose (short legs: a centimetre or so).

Frames: the robot's own frame is its own pose (rebuilt from the legs' records, like explore); the board frame is
the map's. A boundary is ('x', value) (the line x = value) or ('y', value).

Pure functions; the driving is in e02.py (Runner.edge_find). Unit-tested in tests/test_e02_edgefind.py."""
import math

import e02_map as MAP

SIDESTEP_CM = 25.0          # chosen: 25 cm between the two touches on A; a 1 cm error in a touch is then 2.3 deg
SIDESTEP_MIN_CM = 10.0      # chosen: a sidestep cut short by an edge is still used from this far (1 cm -> 5.7 deg)
BACK_CM = 8.0               # backing off a board before turning (blind, short)
BACK_EDGE_CM = 9.5          # backing off an open edge: the sidestep then runs beside it with room for a slant
CORNER_KEEP_CM = 20.0       # touches no nearer than this to where A and B meet (both guards at once otherwise)
APPROACH_EXTRA_CM = 15.0    # drive this much past where the map puts the boundary before giving up (not found)


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def path_clear(x, y, boundary, others, clear_cm=20.0):
    """No other robot within clear_cm of the straight drive from (x, y) to the boundary (2026-10-23 dry run: a
    robot driving to the far board met a parked robot, and the camera watchdog stopped it)."""
    h = math.radians(facing(boundary, x, y))
    ux, uy = math.cos(h), math.sin(h)
    length = abs((x if boundary[0] == "x" else y) - boundary[1])
    for ox, oy in others:
        t = (ox - x) * ux + (oy - y) * uy
        if 0 < t < length + 5 and abs(-(ox - x) * uy + (oy - y) * ux) < clear_cm:
            return False
    return True


def blockers(x, y, boundary, others, clear_cm=20.0):
    """Names of the robots within clear_cm of the straight drive from (x, y) to the boundary. others: {name: (x, y)}."""
    out = []
    for n, (ox, oy) in others.items():
        if not path_clear(x, y, boundary, [(ox, oy)], clear_cm):
            out.append(n)
    return out


def make_way_spot(bx, by, x, y, boundary, others, inside, clear_cm=20.0, step_cm=8.0):
    """Where a robot standing at (bx, by) in the path from (x, y) to the boundary should move to: the nearest spot
    sideways out of that path (clear_cm + a body to spare), on the board (inside(x, y) True), and not nearer than
    clear_cm to any other robot (others: [(x, y)]). None if there is none within 60 cm."""
    h = math.radians(facing(boundary, x, y))
    ux, uy = math.cos(h), math.sin(h)
    px, py = -uy, ux                                   # across the path
    off = -(bx - x) * uy + (by - y) * ux               # its side of the path's line
    need = clear_cm + 10.0
    for k in range(0, 9):
        for sgn in ((1, -1) if off >= 0 else (-1, 1)):
            d = sgn * need + sgn * k * step_cm - off   # from where it stands, across
            tx, ty = bx + d * px, by + d * py
            if not inside(tx, ty):
                continue
            if any(math.hypot(tx - ox, ty - oy) < clear_cm for ox, oy in others):
                continue
            return (tx, ty)
    return None


def choose_boundaries(x, y, sides, corner_skip=None, others=()):
    """The two boundaries to touch from (x, y): the nearest 'x' line and the nearest 'y' line from sides
    [('x', value, kind), ('y', value, kind)] (kind 'board' or 'edge'). Returns (A, B): A is the nearer one (touched
    twice). corner_skip: an (x, y) corner whose two lines are both open edges (never touched together: Nick,
    2026-10-22, both edge guards at once with nothing behind) - then the farther 'y' or 'x' line is used."""
    xs = sorted([s for s in sides if s[0] == "x"], key=lambda s: (not path_clear(x, y, s, others), abs(x - s[1])))
    ys = sorted([s for s in sides if s[0] == "y"], key=lambda s: (not path_clear(x, y, s, others), abs(y - s[1])))
    if not xs or not ys:
        return None
    a, b = xs[0], ys[0]
    if a[2] == "edge" and b[2] == "edge":
        # two open edges meet there: use a board for one of them if there is one
        alt_x = next((s for s in xs if s[2] == "board"), None)
        alt_y = next((s for s in ys if s[2] == "board"), None)
        if alt_x and (not alt_y or abs(x - alt_x[1]) <= abs(y - alt_y[1])):
            a = alt_x
        elif alt_y:
            b = alt_y
    first, second = (a, b) if abs((x if a[0] == "x" else y) - a[1]) <= abs((y if b[0] == "y" else x) - b[1]) else (b, a)
    return first, second


def facing(boundary, x, y):
    """The board-frame heading (deg) that faces a boundary from (x, y)."""
    a, v = boundary[0], boundary[1]
    if a == "x":
        return 0.0 if v > x else 180.0
    return 90.0 if v > y else -90.0


def sidestep_sign(boundary_a, boundary_b, x, y):
    """Which way to sidestep along A (+1: turn left 90, -1: turn right 90, facing A) so the second touch moves away
    from B (the corner)."""
    h = math.radians(facing(boundary_a, x, y))
    lx, ly = -math.sin(h), math.cos(h)            # the robot's left, facing A
    bx = boundary_b[1] - x if boundary_b[0] == "x" else 0.0
    by = boundary_b[1] - y if boundary_b[0] == "y" else 0.0
    return -1 if lx * bx + ly * by > 0 else 1


def sidestep_room(boundary_a, x, y, sides):
    """The sidestep direction along A with the most room: (+1 turn left / -1 turn right, facing A), and that room
    (cm to the nearest boundary across A's direction on that side). 2026-10-23 dry run: away-from-B took a robot
    21 cm from the open edge on the other side into it; room on both sides decides now."""
    h = math.radians(facing(boundary_a, x, y))
    lx, ly = -math.sin(h), math.cos(h)            # its left, facing A
    best = None
    for sgn in (1, -1):
        dx, dy = sgn * lx, sgn * ly
        room = float("inf")
        for s_ in sides:
            if s_[0] == boundary_a[0]:
                continue                          # parallel to A
            d = ((s_[1] - x) / dx) if s_[0] == "x" and abs(dx) > 1e-6 else (
                ((s_[1] - y) / dy) if s_[0] == "y" and abs(dy) > 1e-6 else -1)
            if d > 0:
                room = min(room, d)
        if best is None or room > best[1]:
            best = (sgn, room)
    return best


def solve(touches, a, b):
    """touches: [dict(x, y, th (own frame, the axle at the stop), kind ('board' | 'edge'), on ('A' | 'B'))] - two
    on A, one on B. a, b: the boundaries ('x' | 'y', value, ...). Returns dict(rot (deg: own -> board), tx, ty,
    gap_cm (between the A touches)) or None. Board = R(rot) * own + (tx, ty)."""
    ta = [t for t in touches if t["on"] == "A"]
    tb = [t for t in touches if t["on"] == "B"]
    if len(ta) < 2 or not tb:
        return None
    pa = [MAP.contact_point(t["x"], t["y"], t["th"], t["kind"], t.get("cue", "")) for t in ta]
    pb = MAP.contact_point(tb[0]["x"], tb[0]["y"], tb[0]["th"], tb[0]["kind"], tb[0].get("cue", ""))
    dx, dy = pa[1][0] - pa[0][0], pa[1][1] - pa[0][1]
    if math.hypot(dx, dy) < 4.0:
        return None
    own_dir = math.degrees(math.atan2(dy, dx))
    board_dir = 90.0 if a[0] == "x" else 0.0      # an x = value line runs along y
    rot = wrap(board_dir - own_dir)
    if abs(rot) > 90.0:
        rot = wrap(rot + 180.0)                    # a line has no direction: the smaller turn
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))

    def R(p):
        return (c * p[0] - s * p[1], s * p[0] + c * p[1])

    ra = [R(p) for p in pa]
    rb = R(pb)
    if a[0] == "x":
        tx = a[1] - 0.5 * (ra[0][0] + ra[1][0])
        ty = b[1] - rb[1]
    else:
        ty = a[1] - 0.5 * (ra[0][1] + ra[1][1])
        tx = b[1] - rb[0]
    return dict(rot=rot, tx=tx, ty=ty, gap_cm=math.hypot(dx, dy))


def to_board(sol, x, y, th):
    c, s = math.cos(math.radians(sol["rot"])), math.sin(math.radians(sol["rot"]))
    return (c * x - s * y + sol["tx"], s * x + c * y + sol["ty"], wrap(th + sol["rot"]))
