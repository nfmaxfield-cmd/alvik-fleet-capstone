"""The brick demo (Nick, 2026-10-22): after calibration and mapping, Nick puts bricks anywhere on the open board;
the camera spots "what has changed" (the bricks), plans a path round them, and a robot crosses the board:
  crossing 1: wheels + gyro only, with the corrections learned in calibration (dead reckoning). The camera plans the
              path from where the bricks are, then only watches and measures;
  crossing 2: the same path, with camera fixes between legs (the camera picks each next leg from where it sees the
              robot, and tells the robot where it is).
The robots' bump, cliff and distance sensors are not used for steering; the firmware's edge guard and front stop
stay on for safety, and a crossing they stop counts as failed.

Pure functions (the driving is in e02.py, Runner.brick_demo). Unit-tested in tests/test_e02_bricks.py.

What is a brick: every pixel that differs from the floor memory (e02_camera.MotionTracker: what the floor looked
like just before Nick put the bricks down), where no robot stands, projected onto the floor (z = 0) as a grid of
1 cm cells. Projecting at z = 0 is conservative: a brick's top and the side facing the camera land on and just
beyond its true footprint, so the footprint is always covered, plus a sliver on the far side (its height times
its distance from the camera's foot, over the camera's height: about 1-2 cm here). A brick's shadow counts too.

The planner: every cell the robot's centre may pass: at least ROBOT_REACH_CM + MARGIN_CM from any brick cell
(the body's corner when it turns on the spot, plus room for the drift of dead reckoning), OPEN_EDGE_CLEAR_CM from the
open edges, BOARD_CLEAR_CM from the blocked boards' faces, ROBOT_CLEAR_CM from the other robots. A* on that grid
(a small extra cost for passing close to a brick, so it keeps to the middle of a gap when it can), then simplified
to a few straight legs (a shortcut is taken only if it keeps at least as much room from the bricks as the A* path)."""
import heapq
import json
import math

import cv2
import numpy as np

ROBOT_HALF_CM = 4.75                     # half the Alvik's 9.5 cm length (e02_plan.ROBOT_HALF_CM)
ROBOT_REACH_CM = ROBOT_HALF_CM * math.sqrt(2.0)   # its body's corner, turning on the spot: 6.7 cm
MARGIN_CM = 3.5                          # chosen: room for dead-reckoning drift between the path and a brick
COMFORT_CM = 5.0                         # beyond the margin, the planner prefers this much more room (soft)
OPEN_EDGE_CLEAR_CM = 18.0                # centre from an open edge (the camera watchdog stops at 9 cm)
BOARD_CLEAR_CM = 12.0                    # centre from a blocked board's face (front stop at 7 cm ahead of the sensor)
ROBOT_CLEAR_CM = 22.0                    # centre from another robot's centre (e02_plan.PARK_CLEAR_CM + 4, as in safe_path)
MIN_BRICK_CM2 = 8.0                      # smaller changes are noise (a brick on its side is ~ 6 x 20 cm)
MAX_GO_CM = 90.0                         # longest single go leg (firmware travel cap: the leg + 10 cm, e02_go_max_cm 110)
MIN_TURN_DEG = 1.0                       # smaller turns are not sent (e02_steer.MIN_TURN_DEG)
MAX_BENDS = 3                            # a path with more bends than this trades some room for fewer turns
SIDE_CM = 8.0                            # crossing ends: this far inside the clear area from each side
ROBOT_TOP_CM = 3.7                       # the camera sees a robot's top (e02_camera.ROBOT_TOP_CM)
ROBOT_CM = 9.5


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def bearing(p, q):
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))


# =============================================================================== what changed: the bricks
def floor_snapshot(tracker):
    """A copy of the tracker's floor memory (taken just before Nick puts the bricks down): dict(bg, known, offset,
    thr). The tracker keeps learning afterwards (a brick standing still for 25 s becomes floor to it), so the bricks
    are found against this copy, not the live memory."""
    with tracker.lock:
        if tracker.bg is None:
            return None
        return dict(bg=tracker.bg.copy(), known=tracker.known.copy(), offset=float(tracker.offset),
                    thr=float(tracker.thr), unknown_extra=float(getattr(tracker, "UNKNOWN_EXTRA", 14.0)))


def robot_mask(cam, shape, robots, scale=1.5, z_cm=ROBOT_TOP_CM):
    """uint8 mask (1 = a robot may be there): a disk of scale x the robot's size round each robot's (x, y)."""
    m = np.zeros(shape[:2], np.uint8)
    for x, y in robots:
        u, v = cam.project((x, y, z_cm))
        if not (np.isfinite(u) and np.isfinite(v)):
            continue
        r = int(round(scale * ROBOT_CM * cam.px_per_cm(x, y, z_cm)))
        cv2.circle(m, (int(round(u)), int(round(v))), r, 1, -1)
    return m


def board_mask(cam, shape, rect, z_cm=0.0):
    """uint8 mask (1 = on the board rectangle) in the picture."""
    x0, y0, x1, y1 = rect
    n = 12
    edge = [(x0 + (x1 - x0) * k / n, y0) for k in range(n)] + [(x1, y0 + (y1 - y0) * k / n) for k in range(n)]
    edge += [(x1 - (x1 - x0) * k / n, y1) for k in range(n)] + [(x0, y1 - (y1 - y0) * k / n) for k in range(n)]
    u, v = cam.project(np.array([(x, y, z_cm) for x, y in edge], float))
    ok = np.isfinite(u) & np.isfinite(v)
    m = np.zeros(shape[:2], np.uint8)
    if ok.sum() >= 3:
        cv2.fillPoly(m, [np.round(np.stack([u[ok], v[ok]], 1)).astype(np.int32)], 1)
    return m


def change_mask(frames, floor, ignore=None):
    """Pixels (uint8 0 / 255) where the median of `frames` (grey) differs from the floor memory `floor` (a
    floor_snapshot), the picture's overall brightness shift (auto exposure) taken out. ignore: uint8 mask of pixels
    to leave out (robots)."""
    g = np.median(np.stack([cv2.GaussianBlur(f, (3, 3), 0).astype(np.float32) for f in frames]), axis=0)
    bg, known = floor["bg"], floor["known"]
    keep = known > 0
    if ignore is not None:
        keep &= ignore == 0
    d = g - bg
    sub = d[::4, ::4][keep[::4, ::4]]
    off = float(np.median(sub)) if sub.size > 100 else floor.get("offset", 0.0)
    ad = np.abs(d - off)
    mad = float(np.median(np.abs(sub - off))) if sub.size > 100 else 0.0
    thr = max(floor.get("thr", 16.0), 5.0 * 1.4826 * mad)
    lim = np.where(keep, thr, thr + floor.get("unknown_extra", 14.0)).astype(np.float32)
    mask = (ad > lim).astype(np.uint8) * 255
    if ignore is not None:
        mask[ignore > 0] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return mask


def moving_share(frames_a, frames_b, region=None, thr=20.0):
    """Share of the pixels (in region, a bool mask, or all) that changed between two sets of frames: something
    still moving (a hand, a shadow) when Nick pressed Enter."""
    a = np.median(np.stack([f.astype(np.float32) for f in frames_a]), axis=0)
    b = np.median(np.stack([f.astype(np.float32) for f in frames_b]), axis=0)
    d = a - b
    d = np.abs(d - float(np.median(d)))
    ch = cv2.morphologyEx((d > thr).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0
    if region is not None:
        return float(ch[region].mean()) if region.any() else 0.0
    return float(ch.mean())


class Grid:
    """1 cm (cell_cm) cells over the board rectangle; cell (j, i) = row j along y, column i along x."""

    def __init__(self, rect, cell_cm=1.0):
        x0, y0, x1, y1 = rect
        self.x0, self.y0 = min(x0, x1), min(y0, y1)
        self.cell = float(cell_cm)
        self.nx = int(math.ceil((max(x0, x1) - self.x0) / self.cell))
        self.ny = int(math.ceil((max(y0, y1) - self.y0) / self.cell))

    @property
    def shape(self):
        return (self.ny, self.nx)

    def centres(self):
        xs = self.x0 + (np.arange(self.nx) + 0.5) * self.cell
        ys = self.y0 + (np.arange(self.ny) + 0.5) * self.cell
        return np.meshgrid(xs, ys)

    def cell_of(self, x, y):
        i = int(math.floor((x - self.x0) / self.cell))
        j = int(math.floor((y - self.y0) / self.cell))
        return min(max(j, 0), self.ny - 1), min(max(i, 0), self.nx - 1)

    def xy(self, j, i):
        return (self.x0 + (i + 0.5) * self.cell, self.y0 + (j + 0.5) * self.cell)

    def inside(self, x, y):
        return self.x0 <= x < self.x0 + self.nx * self.cell and self.y0 <= y < self.y0 + self.ny * self.cell

    def at(self, arr, x, y):
        j, i = self.cell_of(x, y)
        return arr[j, i]


def occupancy(cam, mask, grid, dilate_px=2, min_area_cm2=MIN_BRICK_CM2):
    """Floor cells (bool, grid.shape) whose floor point (z = 0) the camera sees as changed. Small specks (under
    min_area_cm2) are left out."""
    m = cv2.dilate(mask, np.ones((2 * dilate_px + 1, 2 * dilate_px + 1), np.uint8)) if dilate_px else mask
    X, Y = grid.centres()
    pts = np.column_stack([X.ravel(), Y.ravel(), np.zeros(X.size)])
    u, v = cam.project(pts)
    ok = np.isfinite(u) & np.isfinite(v)
    ui = np.where(ok, np.round(u), -1).astype(int)
    vi = np.where(ok, np.round(v), -1).astype(int)
    ok &= (ui >= 0) & (vi >= 0) & (ui < m.shape[1]) & (vi < m.shape[0])
    occ = np.zeros(X.size, bool)
    occ[ok] = m[vi[ok], ui[ok]] > 0
    occ = occ.reshape(grid.shape)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(occ.astype(np.uint8), connectivity=8)
    min_cells = min_area_cm2 / grid.cell ** 2
    for k in range(1, n):
        if stats[k, cv2.CC_STAT_AREA] < min_cells:
            occ[lab == k] = False
    return occ


def bricks_from(occ, grid):
    """The separate bricks (connected pieces of occupied cells): [dict(x, y, length_cm, width_cm, angle_deg,
    area_cm2, outline [[x, y] ...])], biggest first. Two bricks touching come out as one piece."""
    n, lab, stats, cents = cv2.connectedComponentsWithStats(occ.astype(np.uint8), connectivity=8)
    out = []
    for k in range(1, n):
        cells = (lab == k).astype(np.uint8)
        cnts, _ = cv2.findContours(cells, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        c = max(cnts, key=cv2.contourArea)
        pts = c.reshape(-1, 2).astype(float)
        # cell indices -> floor cm (cell corners: the outline goes round the cells)
        hull = cv2.convexHull(c).reshape(-1, 2).astype(float)
        outline = [[round(grid.x0 + (p[0] + 0.5) * grid.cell, 1), round(grid.y0 + (p[1] + 0.5) * grid.cell, 1)]
                   for p in hull]
        (cx, cy), (w, h), ang = cv2.minAreaRect(pts.astype(np.float32))
        L, W = max(w, h) + 1.0, min(w, h) + 1.0
        if w < h:
            ang += 90.0
        x, y = grid.xy(cents[k][1], cents[k][0])
        out.append(dict(x=round(x, 1), y=round(y, 1), length_cm=round(L * grid.cell, 1), width_cm=round(W * grid.cell, 1),
                        angle_deg=round(wrap180(ang), 1), area_cm2=round(float(stats[k, cv2.CC_STAT_AREA]) * grid.cell ** 2, 1),
                        outline=outline))
    out.sort(key=lambda b: -b["area_cm2"])
    return out


def find_bricks(cam, frames, floor, board_rect, robots=(), cell_cm=1.0):
    """Everything that changed since `floor` (a floor_snapshot), away from the robots, as floor cells.
    robots: [(x, y)] where robots stand. Returns dict(occ, grid, bricks, mask)."""
    ign = robot_mask(cam, frames[0].shape, robots)
    mask = change_mask(frames, floor, ignore=ign)
    grid = Grid(board_rect, cell_cm)
    occ = occupancy(cam, mask, grid)
    return dict(occ=occ, grid=grid, bricks=bricks_from(occ, grid), mask=mask)


# =============================================================================== where the robot may go
def brick_distance(occ, grid):
    """cm from every cell's centre to the nearest brick cell's centre (1e3 where there are no bricks)."""
    if not occ.any():
        return np.full(grid.shape, 1e3, np.float32)
    d = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5)
    return d.astype(np.float32) * grid.cell


def free_space(occ, grid, board, others=(), margin_cm=MARGIN_CM, open_cm=OPEN_EDGE_CLEAR_CM, board_cm=BOARD_CLEAR_CM,
               robot_cm=ROBOT_CLEAR_CM):
    """Where the robot's centre may go. board: an e02_plan.Board (open edges, board faces). others: [(x, y)] the
    other robots. Returns dict(grid, occ, dist (cm to a brick), free (bool), soft (bool: not touching anything: the
    robot may start or end a move there even if it is not free), r_cm (the room kept from a brick))."""
    X, Y = grid.centres()
    dist = brick_distance(occ, grid)
    r = ROBOT_REACH_CM + margin_cm
    free = dist > r
    soft = dist > ROBOT_HALF_CM + 1.5
    for a, v in board.open_edges():
        d = np.abs((X if a == "x" else Y) - v)
        free &= d >= open_cm
        soft &= d >= 10.0
    dxb = (Y - board.xboard_cm) * board.uy            # inward distance from the x-board's face
    dyb = (X - board.yboard_cm) * board.ux
    free &= (dxb >= board_cm) & (dyb >= board_cm)
    soft &= (dxb >= ROBOT_HALF_CM + 1.5) & (dyb >= ROBOT_HALF_CM + 1.5)
    for ox, oy in others:
        d = np.hypot(X - ox, Y - oy)
        free &= d >= robot_cm
        soft &= d >= 12.0
    return dict(grid=grid, occ=occ, dist=dist, free=free, soft=soft, r_cm=r, board=board, others=list(others))


def _astar(passable, cost, s, g):
    """8-connected A* on a bool grid; cost: extra cost per cell (float array). s, g: (j, i). Returns [(j, i)] or None."""
    ny, nx = passable.shape
    if not (passable[s] and passable[g]):
        return None
    SQ2 = math.sqrt(2.0)
    nb = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (1, 1, SQ2)]

    def h(c):
        dj, di = abs(c[0] - g[0]), abs(c[1] - g[1])
        return (dj + di) + (SQ2 - 2.0) * min(dj, di)
    best = {s: 0.0}
    came = {}
    q = [(h(s), 0.0, s)]
    while q:
        f, gc, c = heapq.heappop(q)
        if c == g:
            path = [c]
            while c in came:
                c = came[c]
                path.append(c)
            return path[::-1]
        if gc > best.get(c, 1e18):
            continue
        j, i = c
        for dj, di, step in nb:
            jj, ii = j + dj, i + di
            if jj < 0 or ii < 0 or jj >= ny or ii >= nx or not passable[jj, ii]:
                continue
            if dj and di and not (passable[j, ii] and passable[jj, i]):
                continue                          # no cutting a corner between two blocked cells
            nc = (jj, ii)
            ng = gc + step * (1.0 + cost[jj, ii])
            if ng < best.get(nc, 1e18):
                best[nc] = ng
                came[nc] = c
                heapq.heappush(q, (ng + h(nc), ng, nc))
    return None


def _segment_ok(space, passable, a, b, need_cm, step_cm=0.5):
    """A straight piece a -> b stays on passable cells, at least need_cm from every brick."""
    grid = space["grid"]
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(math.ceil(L / step_cm)) + 1)
    xs = np.linspace(a[0], b[0], n)
    ys = np.linspace(a[1], b[1], n)
    ii = np.clip(np.floor((xs - grid.x0) / grid.cell).astype(int), 0, grid.nx - 1)
    jj = np.clip(np.floor((ys - grid.y0) / grid.cell).astype(int), 0, grid.ny - 1)
    if not passable[jj, ii].all():
        return False
    return float(space["dist"][jj, ii].min()) >= need_cm - 1e-6


def plan_path(space, start, goal, escape_cm=20.0):
    """A path for the robot's centre from start to goal (floor cm), round the bricks. The start may be in a tight
    spot (not free but not touching anything, e.g. where a front stop left it): the path may leave it within
    escape_cm. Returns dict(ok, points [(x, y)] start..goal, length_cm, min_room_cm (closest centre to a brick),
    why)."""
    grid = space["grid"]
    free, soft = space["free"], space["soft"]
    s, g = grid.cell_of(*start), grid.cell_of(*goal)
    X, Y = grid.centres()
    passable = free | (soft & (np.hypot(X - start[0], Y - start[1]) <= escape_cm))
    if not passable[s]:
        return dict(ok=False, points=[], why="the start is too close to a brick, an edge or a robot")
    if not free[g]:
        return dict(ok=False, points=[], why="the goal is too close to a brick, an edge or a robot")
    r = space["r_cm"]
    cost = 2.0 * np.clip((r + COMFORT_CM - space["dist"]) / COMFORT_CM, 0.0, 1.0)
    cells = _astar(passable, cost, s, g)
    if cells is None:
        return dict(ok=False, points=[], why="no way through: the bricks (or the robots) block every way")
    pts = [tuple(start)] + [grid.xy(*c) for c in cells[1:-1]] + [tuple(goal)]
    d_along = [float(grid.at(space["dist"], *p)) for p in pts]
    # shortcuts: from each kept point, the farthest point reachable in a straight line that keeps at least `keep`
    # cm from the bricks (or as much as the A* path itself had over that stretch, where it was tighter: a start in
    # a tight spot). Every bend is a turn the gyro can get a little wrong, so fewer bends matter as much as room:
    # the most room that still gives at most MAX_BENDS bends; if none does, the fewest bends.
    best = None
    for keep in (r + COMFORT_CM, r + 3.0, r + 1.5, r):
        out = _shortcuts(space, passable, pts, d_along, keep)
        if best is None or len(out) < len(best):
            best = out
        if len(out) - 2 <= MAX_BENDS:
            best = out
            break
    out = best
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(out, out[1:]))
    room = min_room(space, out)
    return dict(ok=True, points=[(round(float(p[0]), 2), round(float(p[1]), 2)) for p in out],
                length_cm=round(length, 1), min_room_cm=round(room, 1), why="")


def _shortcuts(space, passable, pts, d_along, keep):
    out = [pts[0]]
    k = 0
    while k < len(pts) - 1:
        nxt = k + 1
        for m in range(len(pts) - 1, k + 1, -1):
            need = min(keep, min(d_along[k:m + 1]))
            if _segment_ok(space, passable, pts[k], pts[m], need):
                nxt = m
                break
        out.append(pts[nxt])
        k = nxt
    return out


def min_room(space, points, step_cm=0.5):
    """The closest the robot's centre comes to a brick cell along a polyline (cm)."""
    best = 1e3
    grid = space["grid"]
    for a, b in zip(points, points[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(math.ceil(L / step_cm)) + 1)
        for t in np.linspace(0.0, 1.0, n):
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            if grid.inside(x, y):
                best = min(best, float(grid.at(space["dist"], x, y)))
    return best


def legs_from(start, heading, points, max_go=MAX_GO_CM, min_turn=MIN_TURN_DEG):
    """The go / turn legs that drive the path `points` (start .. goal) from `start` facing `heading` (deg), as the
    robot would drive them on its own wheels and gyro: [dict(cmd=('turn', deg) | ('go', cm), to=waypoint index)].
    Long straight pieces are split (no turn between)."""
    legs = []
    cur, h = tuple(start), float(heading)
    for k, w in enumerate(points[1:], start=1):
        d = math.hypot(w[0] - cur[0], w[1] - cur[1])
        if d < 0.5:
            continue
        b = bearing(cur, w)
        t = wrap180(b - h)
        if abs(t) >= min_turn:
            legs.append(dict(cmd=("turn", round(t, 2)), to=k))
            h = b
        n = int(math.ceil(d / max_go))
        for _ in range(n):
            legs.append(dict(cmd=("go", round(d / n, 2)), to=k))
        cur = (w[0], w[1])
    return legs


def dead_reckon(start, heading, legs, gyro_err=1.0, wheel_err=1.0):
    """Where a robot ends that turns gyro_err x and drives wheel_err x what it is told (for tests and the
    plain-words 'what an uncorrected robot would do')."""
    x, y, h = start[0], start[1], heading
    for lg in legs:
        k, v = lg["cmd"]
        if k == "turn":
            h += v * gyro_err
        else:
            x += v * wheel_err * math.cos(math.radians(h))
            y += v * wheel_err * math.sin(math.radians(h))
    return x, y, h


# =============================================================================== where to cross
def _straight_blocked(space, a, b, step_cm=1.0):
    """How many cm of the straight line a -> b run through the room kept round the bricks."""
    grid = space["grid"]
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(L / step_cm) + 1)
    xs = np.linspace(a[0], b[0], n)
    ys = np.linspace(a[1], b[1], n)
    ii = np.clip(np.floor((xs - grid.x0) / grid.cell).astype(int), 0, grid.nx - 1)
    jj = np.clip(np.floor((ys - grid.y0) / grid.cell).astype(int), 0, grid.ny - 1)
    return float((space["dist"][jj, ii] <= space["r_cm"]).sum()) * L / n


def crossing_candidates(board, n_lanes=5, side_cm=SIDE_CM):
    """Start / goal pairs straight across the board, both ways, along both axes: [(start, goal, words)]. The ends sit
    side_cm inside the clear area at each side (the blocked board's face, or the open edge)."""
    out = []
    xb_in = board.yboard_cm + board.ux * (BOARD_CLEAR_CM + side_cm)          # x near the y-board
    ox = [v for a, v in board.open_edges() if a == "x"][0]
    x_open = ox - board.ux * (OPEN_EDGE_CLEAR_CM + side_cm)                  # inwards from the open edge: -ux
    yb_in = board.xboard_cm + board.uy * (BOARD_CLEAR_CM + side_cm)
    oy = [v for a, v in board.open_edges() if a == "y"][0]
    y_open = oy - board.uy * (OPEN_EDGE_CLEAR_CM + side_cm)
    ylo, yhi = sorted((yb_in, y_open))
    xlo, xhi = sorted((xb_in, x_open))
    for f in np.linspace(0.1, 0.9, n_lanes):
        y = ylo + f * (yhi - ylo)
        out.append(((xb_in, y), (x_open, y), "from the blocked board across to the open edge"))
        out.append(((x_open, y), (xb_in, y), "from the open edge across to the blocked board"))
        x = xlo + f * (xhi - xlo)
        out.append(((x, yb_in), (x, y_open), "from the blocked board across to the open edge"))
        out.append(((x, y_open), (x, yb_in), "from the open edge across to the blocked board"))
    return [(tuple(round(float(v), 2) for v in s), tuple(round(float(v), 2) for v in g), w) for s, g, w in out]


def choose_crossing(occ, grid, board, robots, prefer_blocked=True):
    """Pick the crossing for the demo: a start on one side, a goal straight across, the path between them round the
    bricks, and the robot that starts it (the nearest that can get to the start without touching a brick).
    robots: {name: (x, y)} every robot on the board. Prefers a crossing whose straight line runs through the bricks
    (so the path has to go round them), then the one that runs through them most; of the robots that can do it, the
    one with the shortest way to the start. Returns dict(robot, start, goal,
    path (plan_path result), approach (the robot's way to the start), words, straight_blocked_cm, space) or None."""
    spaces = {}

    def space_for(name):
        if name not in spaces:
            spaces[name] = free_space(occ, grid, board, others=[p for n, p in robots.items() if n != name])
        return spaces[name]
    any_space = free_space(occ, grid, board, others=[])
    cands = []
    for s, g, words in crossing_candidates(board):
        blk = _straight_blocked(any_space, s, g)
        cands.append((blk > 0 if prefer_blocked else 0, blk, -abs(s[0] - g[0]) - abs(s[1] - g[1]), s, g, words))
    cands.sort(key=lambda c: (c[0], c[1], c[2]), reverse=True)
    for _, blk, _, s, g, words in cands:
        best = None
        for name in robots:
            sp = space_for(name)
            if not (grid.at(sp["free"], *s) and grid.at(sp["free"], *g)):
                continue
            path = plan_path(sp, s, g)
            if not path["ok"]:
                continue
            appr = plan_path(sp, robots[name], s, escape_cm=25.0)
            if not appr["ok"]:
                continue
            if best is None or appr["length_cm"] < best["approach"]["length_cm"]:
                best = dict(robot=name, start=s, goal=g, path=path, approach=appr, words=words,
                            straight_blocked_cm=round(blk, 1), space=sp)
        if best is not None:
            return best
    return None


# =============================================================================== measuring and words
def closest_approach(space, track):
    """Closest the robot's centre came to a brick over a camera track [(t, x, y)] (or [(x, y)]): dict(centre_cm,
    body_cm (centre minus half the robot: about the gap between its side and the brick), at (x, y)), or None."""
    best = None
    grid = space["grid"]
    if not space["occ"].any():
        return None
    for q in track:
        x, y = (q[1], q[2]) if len(q) >= 3 else (q[0], q[1])
        if not grid.inside(x, y):
            continue
        d = float(grid.at(space["dist"], x, y))
        if best is None or d < best[0]:
            best = (d, (round(x, 1), round(y, 1)))
    if best is None:
        return None
    # the distance map is centre to the nearest brick CELL CENTRE: half a cell more to the brick's edge
    c = max(0.0, best[0] - 0.5 * grid.cell)
    return dict(centre_cm=round(c, 1), body_cm=round(c - ROBOT_HALF_CM, 1), at=best[1])


def brick_words(bricks):
    if not bricks:
        return ["  The camera found no bricks on the board."]
    lines = ["  The camera found %d brick%s:" % (len(bricks), "" if len(bricks) == 1 else "s")]
    for k, b in enumerate(bricks, 1):
        lines.append("    %d. at (%.0f, %.0f) cm, about %.0f x %.0f cm" % (k, b["x"], b["y"], b["length_cm"], b["width_cm"]))
    return lines


def plan_words(ch):
    p = ch["path"]
    turns = max(0, len(p["points"]) - 2)
    return ["  Plan: %s drives %s, from (%.0f, %.0f) to (%.0f, %.0f) cm." % (ch["robot"], ch["words"], ch["start"][0],
                                                                         ch["start"][1], ch["goal"][0], ch["goal"][1]),
            "  The path is %.0f cm long with %d bend%s; its centre stays at least %.0f cm from every brick (the robot's "
            "side about %.0f cm)." % (p["length_cm"], turns, "" if turns == 1 else "s", p["min_room_cm"],
                                      p["min_room_cm"] - ROBOT_HALF_CM),
            ("  A straight line would have run %.0f cm through the bricks' space, so it has to go round them."
             % ch["straight_blocked_cm"]) if ch["straight_blocked_cm"] > 0 else
            "  (The straight line is clear of the bricks: no crossing had bricks in its way.)"]


def crossing_words(c):
    """One crossing's result in plain words."""
    name = "Crossing %d (%s)" % (c["n"], c["how"])
    if c.get("end_err_cm") is None:
        return ["  %s: the camera did not see where it ended." % name]
    lines = ["  %s: ended %.1f cm from the goal%s." % (
        name, c["end_err_cm"], (" (the robot itself thought %.1f cm)" % c["robot_thought_cm"])
        if c.get("robot_thought_cm") is not None else "")]
    ca = c.get("closest")
    if ca:
        lines.append("    Closest to a brick: its centre %.1f cm away (its side about %.1f cm)%s." % (
            ca["centre_cm"], max(0.0, ca["body_cm"]), "  TOUCHED or nearly" if ca["body_cm"] < 0.5 else ""))
    if c.get("failed"):
        lines.append("    It did NOT finish: %s (counted as a failed crossing)." % c["failed"])
    else:
        lines.append("    It finished all %d legs in %.0f s." % (c["legs_done"], c["seconds"]))
    return lines


def save(path, d):
    def clean(o):
        if isinstance(o, dict):
            return {k: clean(v) for k, v in o.items() if k != "space"}
        if isinstance(o, (list, tuple)):
            return [clean(v) for v in o]
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        if isinstance(o, np.ndarray):
            return None
        return o
    with open(path, "w") as f:
        json.dump(clean(d), f, indent=1)
