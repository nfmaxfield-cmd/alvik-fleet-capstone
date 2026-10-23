"""Robot perimeter mapping (Nick, 2026-10-21): the robots feel out the board, the camera only watches. One robot
drives rays out from the middle of the board; each ray ends where the robot itself stops:
  - a BOARD (blocked edge): its front distance sensor stops it ~safe_front_cm short of it ("obstacle");
  - an OPEN EDGE: its line sensors see the plywood end ("plywood edge (...)"), the edge guard stops it;
  - nothing within reach: the ray is too short to say (no point).
The stop pose comes from the robot's own wheels and gyro since a camera fix at the middle (a ray is short, so
that is good to a centimetre or two), and the thing it met is a little ahead of it. Points are then sorted to
the board's four sides and a line fitted per side: what kind of side it is (board / open edge), and where.

Why: the camera found the boards from how bright the plywood is, and the left board's line came out uneven
where a shadow lies on the plywood (2026-10-21). Touch doesn't care about light. It is also the first real
measurement of the edge guard (where exactly a robot stops before an open edge), and the first piece of the
Live lab's self-mapping (docs/build-plans/live-lab-outline.md): the travel area is learned, not typed in.

Pure functions; the driving is in e02.py (Runner.map_board). Unit-tested in tests/test_e02_map.py."""
import json
import math

import numpy as np

LINE_AHEAD_CM = 3.5         # line sensors ahead of the wheel axle (firmware edge.h)
TOF_AHEAD_CM = 4.5          # front distance sensor ahead of the axle (params.h tof_ahead_cm)
SAFE_FRONT_CM = 7.0         # the front stop's distance (params.h safe_front_cm)
RAY_CM = 95.0               # longest ray (firmware travel cap: the leg + 10 cm; e02_go_max_cm 110)


def ray_headings(n=16, offset_deg=11.25):
    """n headings round the circle, offset so no ray points straight into a corner."""
    return [((offset_deg + 360.0 * k / n + 180.0) % 360.0) - 180.0 for k in range(n)]


CORNER_SKIP_DEG = 25.0   # chosen: no ray within this of the open corner's bearing (a DOE would not help: a safety margin)


def toward_corner(start, heading_deg, corner, within_deg=CORNER_SKIP_DEG):
    """True when the ray from start points within within_deg of the corner where the two open edges meet (Nick,
    2026-10-22 evening: never aim at it; both edge guards would have to stop it at once, with nothing behind)."""
    b = math.degrees(math.atan2(corner[1] - start[1], corner[0] - start[0]))
    return abs(((heading_deg - b) + 180.0) % 360.0 - 180.0) <= within_deg


def ray_blocked(start, heading_deg, others, clear_cm=22.0, length_cm=RAY_CM):
    """Name of the first robot standing within clear_cm of the ray (it would stop the ray, or the camera watchdog
    would), or ''. others: [(name, (x, y))]."""
    ux, uy = math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg))
    best = None
    for n, (x, y) in others:
        t = (x - start[0]) * ux + (y - start[1]) * uy
        if t < 0 or t > length_cm:
            continue
        d = abs(-(x - start[0]) * uy + (y - start[1]) * ux)
        if d < clear_cm and (best is None or t < best[0]):
            best = (t, n)
    return best[1] if best else ""


def stop_kind(leg_state, info):
    """What ended a ray leg: 'edge', 'board' (front stop) or None (done: nothing met, or something else)."""
    text = (info or "").lower()
    if leg_state == "failed" and "edge" in text:
        return "edge"
    if leg_state == "failed" and ("obstacle" in text or "front" in text):
        return "board"
    return None


def contact_point(x, y, th_deg, kind):
    """The point the robot met, from its axle pose at the stop: an edge just ahead of its line sensors, a board
    its front stop distance ahead of the distance sensor."""
    ahead = LINE_AHEAD_CM if kind == "edge" else TOF_AHEAD_CM + SAFE_FRONT_CM
    return (x + ahead * math.cos(math.radians(th_deg)), y + ahead * math.sin(math.radians(th_deg)))


def side_of(point, board):
    """Which side of the board rectangle a point is nearest: ('x', value) | ('y', value), the side's line."""
    x, y = point
    cands = [("x", board.x0, abs(x - board.x0)), ("x", board.x1, abs(x - board.x1)),
             ("y", board.y0, abs(y - board.y0)), ("y", board.y1, abs(y - board.y1))]
    a, v, _ = min(cands, key=lambda c: c[2])
    return a, v


def fit_sides(points, board, others=(), robot_clear_cm=16.0):
    """points: [(x, y, kind)] contact points. Points near another robot (parked robots stop a ray too) are left out.
    Returns {side key: dict(axis, nominal, value, spread, n, kind, kinds)}; side key like 'x=0' (the line x = 0).
    value: the median of the points' coordinate across the side (a board's face, or where the plywood ends)."""
    groups = {}
    for x, y, kind in points:
        if any(math.hypot(x - o[0], y - o[1]) < robot_clear_cm for o in others):
            continue
        a, v = side_of((x, y), board)
        groups.setdefault((a, v), []).append((x, y, kind))
    out = {}
    for (a, v), pts in groups.items():
        vals = np.array([p[0] if a == "x" else p[1] for p in pts])
        kinds = [p[2] for p in pts]
        kind = max(set(kinds), key=kinds.count)
        med = float(np.median(vals))
        out["%s=%g" % (a, round(v, 2))] = dict(axis=a, nominal=round(v, 2), value=round(med, 2),
                                               spread=round(float(np.ptp(vals)), 2) if len(vals) > 1 else 0.0,
                                               n=len(pts), kind=kind, kinds={k: kinds.count(k) for k in set(kinds)})
    return out


def summary_words(sides, board):
    """Plain sentences for the console: each side, what it is and where, against the camera's numbers."""
    lines = []
    for key, s in sorted(sides.items()):
        what = "a board" if s["kind"] == "board" else "an open edge" if s["kind"] == "edge" else s["kind"]
        line = "  %s side (%s = %.1f cm on the camera's outline): %s, felt at %s = %.1f cm (%d rays, within %.1f cm)" % (
            key, s["axis"], s["nominal"], what, s["axis"], s["value"], s["n"], s["spread"])
        lines.append(line)
    return lines


def layout_check(sides, board, tol_cm=10.0):
    """The felt sides against the layout taken at setup (which edges have boards). Returns (safe, risky): sentences
    for a board felt where an open edge was taken (harmless: the plan keeps away from it anyway) and for an open
    edge felt where a board was taken (a find drives into that 'board' and would go over the edge)."""
    opens = board.open_edges()
    safe, risky = [], []
    for key, sd in sorted(sides.items()):
        if sd["kind"] not in ("board", "edge"):
            continue
        is_open = any(a == sd["axis"] and abs(v - sd["nominal"]) < tol_cm for a, v in opens)
        if sd["kind"] == "board" and is_open:
            safe.append("  the %s side was taken as an open edge, but %d rays felt a board there" % (key, sd["n"]))
        elif sd["kind"] == "edge" and not is_open:
            risky.append("  the %s side was taken as a board, but %d rays felt an open edge there" % (key, sd["n"]))
    return safe, risky


def save(path, robot, rays, sides, board):
    with open(path, "w") as f:
        json.dump(dict(robot=robot, rays=rays, sides=sides, board=board.to_dict()), f, indent=1)
