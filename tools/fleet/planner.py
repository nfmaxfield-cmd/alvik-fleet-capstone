"""Fleet planner: paths for every robot made of straight drives and turns in
place, timed so robots never come closer than `separation_cm`, written as
plan.json (format in docs/data-formats.md, "Fleet plan").

How it works (plain words in README.md, design in DESIGN.md):

1. Occupancy grid, 1 cm cells. A cell is free when the robot could turn in
   place there: its centre is at least (robot turning radius + safety margin)
   from every board, keep-out, obstacle and the floor edge.
2. Shortest route over the grid (Dijkstra, from scipy), then "string
   pulling": keep only the corners the robot really needs, checking that the
   straight drive between them stays clear. Result: few waypoints, straight
   drives between them, turns in place at them.
3. Timing. Each robot's route becomes a list of steps (turn, drive,
   approach, wait, park) with durations from the speed settings. Robots are
   planned one after another (a priority order). Each one is timed against a
   reservation table: where the robots planned before it are at every 0.1 s.
   A small search along its route decides where to wait, and for how long,
   so its body never comes within `separation_cm` of another body. If waiting
   is not enough (someone is parked in the way), the route is re-planned
   around the spots where the others stand still.
4. Every priority order is tried; the one where the last robot finishes
   soonest wins (ties: the smallest sum of finish times).
5. Step order rules ("after" lists). For every pair of steps of two robots
   whose areas come within the separation distance, the one that starts
   earlier in the plan must finish before the other starts. A robot that
   obeys these rules cannot hit another robot even when it runs late.
"""
import copy
import itertools
import json
import math
import time

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

import fleetmap as FM
import geometry as G

PLAN_FORMAT = "alvik-fleet-plan"
PLAN_VERSION = 1

DEFAULT_SETTINGS = {
    "cell_cm": 1.0,             # occupancy grid cell size
    "safety_margin_cm": 1.5,    # extra room kept from boards, keep-outs, obstacles, floor edge
    "separation_cm": 5.0,       # closest two robot bodies may come
    "cruise_cms": 10.0,         # straight drives
    "approach_cms": 3.0,        # final approach pre-dock -> dock
    "backout_cms": 5.0,         # reversing dock -> pre-dock
    "turn_dps": 60.0,           # average turn rate in place
    "turn_settle_s": 0.3,       # pause after each turn
    "drive_settle_s": 0.2,      # pause after a straight run ends
    "dwell_s": 2.0,             # time spent docked at a board
    "chunk_cm": 15.0,           # long drives are split into steps of at most this
    "tick_s": 0.1,              # time resolution of the plan
    "max_wait_s": 120.0,        # most waiting one robot may add
    "max_orders": 24,           # priority orders tried at most
    "stop_horizon_s": 2.0,      # obstacle event: stop a robot that would reach it within this
    "replan_budget_s": 2.0,     # obstacle event: time allowed for the new plan
    "skip_unreachable": False,  # drop tasks that cannot be reached instead of failing
}


class PlanError(RuntimeError):
    pass


class S(dict):
    """Settings with attribute access."""
    __getattr__ = dict.__getitem__


def settings(over=None):
    s = S(DEFAULT_SETTINGS)
    for k, v in (over or {}).items():
        if k not in DEFAULT_SETTINGS:
            raise PlanError("unknown setting %r" % k)
        s[k] = type(DEFAULT_SETTINGS[k])(v)
    return s


# ----------------------------------------------------------------------------
# Grid and routes
# ----------------------------------------------------------------------------

class Grid:
    """Clearance grid for one World and the routes over it."""

    def __init__(self, world, body, s, base=None, circles=()):
        """base + circles: reuse another grid's clearance and add circular
        obstacles (much quicker than starting over)."""
        self.world, self.body, self.s = world, body, s
        x0, x1, y0, y1 = world.box
        c = s.cell_cm
        self.x0, self.y0, self.c = x0, y0, c
        self.nx = max(1, int(math.floor((x1 - x0) / c)))
        self.ny = max(1, int(math.floor((y1 - y0) / c)))
        xs = x0 + (np.arange(self.nx) + 0.5) * c
        ys = y0 + (np.arange(self.ny) + 0.5) * c
        X, Y = np.meshgrid(xs, ys)
        self.centres = np.stack([X.ravel(), Y.ravel()], axis=1)
        if base is not None:
            self.clear = base.clear.copy()
            if len(circles):
                C = np.array([cc for cc, _ in circles], float)
                R = np.array([rr for _, rr in circles], float)
                # only cells close enough to change "free" (or the line-of-sight
                # test, whose threshold is lower) need the new distance
                cl2 = self.clear.reshape(self.ny, self.nx)
                for (cx, cy), rr in zip(C, R):
                    w = rr + body.r_turn + s.safety_margin_cm + 2.0
                    i0 = max(0, int((cx - w - x0) // c)); i1 = min(self.nx, int((cx + w - x0) // c) + 1)
                    j0 = max(0, int((cy - w - y0) // c)); j1 = min(self.ny, int((cy + w - y0) // c) + 1)
                    if i0 >= i1 or j0 >= j1:
                        continue
                    d = np.hypot(xs[i0:i1][None, :] - cx, ys[j0:j1][:, None] - cy) - rr
                    np.minimum(cl2[j0:j1, i0:i1], d, out=cl2[j0:j1, i0:i1])
        else:
            self.clear = world.clearance(self.centres)              # (ny*nx,)
        self.turn_thr = body.r_turn + s.safety_margin_cm
        self.free = self.clear >= self.turn_thr
        if base is not None:
            # the base grid's edges, minus those that touch a newly blocked cell
            er, ec, ew = base._edges
            keep = self.free[er] & self.free[ec]
            self._edges = (er[keep], ec[keep], ew[keep])
            n = self.nx * self.ny
            rr, cc, ww = self._edges
            self.graph = csr_matrix((np.concatenate([ww, ww]), (np.concatenate([rr, cc]), np.concatenate([cc, rr]))),
                                    shape=(n, n))
        else:
            self._graph()
        self._dij = {}
        # line-of-sight test threshold: half width + margin, plus the worst
        # error of reading clearance from the nearest cell centre (0.71 cell)
        # and of sampling the line every half cell (0.25 cell)
        self.los_thr = body.r_side + s.safety_margin_cm + 0.96 * c

    def _graph(self):
        nx, ny = self.nx, self.ny
        F = self.free.reshape(ny, nx)
        idx = np.arange(nx * ny).reshape(ny, nx)
        rows, cols, w = [], [], []
        for dy, dx, cost in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
            a = F[max(0, -dy):ny - max(0, dy), max(0, -dx):nx - max(0, dx)]
            b = F[max(0, dy):, max(0, dx):][:a.shape[0], :a.shape[1]] if dx >= 0 else \
                F[dy:, :nx + dx][:a.shape[0], :a.shape[1]]
            ia = idx[max(0, -dy):ny - max(0, dy), max(0, -dx):nx - max(0, dx)]
            ib = idx[max(0, dy):, max(0, dx):][:a.shape[0], :a.shape[1]] if dx >= 0 else \
                idx[dy:, :nx + dx][:a.shape[0], :a.shape[1]]
            if dx < 0:
                a = F[:ny - dy, -dx:]
                ia = idx[:ny - dy, -dx:]
                b = F[dy:, :nx + dx]
                ib = idx[dy:, :nx + dx]
            ok = a & b
            rows.append(ia[ok])
            cols.append(ib[ok])
            w.append(np.full(int(ok.sum()), cost * self.c))
        r = np.concatenate(rows)
        cc = np.concatenate(cols)
        ww = np.concatenate(w)
        self._edges = (r, cc, ww)
        n = nx * ny
        self.graph = csr_matrix((np.concatenate([ww, ww]), (np.concatenate([r, cc]), np.concatenate([cc, r]))),
                                shape=(n, n))

    def cell(self, p):
        i = int(np.clip((p[0] - self.x0) // self.c, 0, self.nx - 1))
        j = int(np.clip((p[1] - self.y0) // self.c, 0, self.ny - 1))
        return j * self.nx + i

    def point_clear(self, p):
        return float(self.world.clearance(np.asarray(p, float)[None, :2])[0])

    def turnable(self, p):
        return self.point_clear(p) >= self.turn_thr - 1e-9

    def los(self, a, b):
        """Can the robot drive straight a -> b (both turnable)? Checked on the
        grid with a conservative bound; the result is re-checked exactly in
        verify_plan."""
        a = np.asarray(a[:2], float)
        b = np.asarray(b[:2], float)
        L = float(np.linalg.norm(b - a))
        n = max(2, int(math.ceil(L / (0.5 * self.c))) + 1)
        t = np.linspace(0.0, 1.0, n)
        pts = a + t[:, None] * (b - a)
        i = np.clip(((pts[:, 0] - self.x0) // self.c).astype(int), 0, self.nx - 1)
        j = np.clip(((pts[:, 1] - self.y0) // self.c).astype(int), 0, self.ny - 1)
        return bool(np.all(self.clear[j * self.nx + i] >= self.los_thr))

    def _entry(self, p):
        """Free cell next to point p that the robot can drive to straight."""
        ci = self.cell(p)
        if self.free[ci]:
            return ci
        i0, j0 = ci % self.nx, ci // self.nx
        best = None
        for r in range(1, 4):
            for j in range(j0 - r, j0 + r + 1):
                for i in range(i0 - r, i0 + r + 1):
                    if 0 <= i < self.nx and 0 <= j < self.ny:
                        k = j * self.nx + i
                        if self.free[k]:
                            d = math.dist(p[:2], self.centres[k])
                            if (best is None or d < best[0]) and self.los(p, self.centres[k]):
                                best = (d, k)
            if best:
                return best[1]
        return None

    def route(self, a, b):
        """Waypoints a -> b (a and b must be turnable). None if no route."""
        a = np.asarray(a[:2], float)
        b = np.asarray(b[:2], float)
        if math.dist(a, b) < 1e-6:
            return [a, b]
        if self.los(a, b):
            return [a, b]
        ca, cb = self._entry(a), self._entry(b)
        if ca is None or cb is None:
            return None
        if cb not in self._dij:
            dist, pred = dijkstra(self.graph, directed=False, indices=cb, return_predecessors=True)
            self._dij[cb] = (dist, pred)
        dist, pred = self._dij[cb]
        if not np.isfinite(dist[ca]):
            return None
        cells = [ca]
        while cells[-1] != cb:
            cells.append(int(pred[cells[-1]]))
        pts = [a] + [self.centres[k] for k in cells] + [b]
        return self._pull(pts)

    def geodesic(self, a, b):
        """Route length a -> b over the grid (inf if none)."""
        ca, cb = self._entry(np.asarray(a, float)), self._entry(np.asarray(b, float))
        if ca is None or cb is None:
            return math.inf
        if cb not in self._dij:
            self._dij[cb] = dijkstra(self.graph, directed=False, indices=cb, return_predecessors=True)
        return float(self._dij[cb][0][ca]) + math.dist(a[:2], self.centres[ca]) + math.dist(b[:2], self.centres[cb])

    def _pull(self, pts):
        """String pulling: from each kept point jump to the furthest point
        still in straight view."""
        out = [pts[0]]
        i, n = 0, len(pts)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and not self.los(pts[i], pts[j]):
                j -= 1 if j - i < 8 else max(1, (j - i) // 8)
            # step forward again in case the coarse steps skipped visible points
            while j + 1 < n and self.los(pts[i], pts[j + 1]):
                j += 1
            out.append(pts[j])
            i = j
        return [np.asarray(p, float) for p in out]


# ----------------------------------------------------------------------------
# Steps ("primitives") and their timing
# ----------------------------------------------------------------------------

def _ticks(sec, s):
    return max(1, int(math.ceil(sec / s.tick_s - 1e-9)))


def prim(kind, frm, to, s, *, speed=None, move_s=0.0, settle=0.0, reverse=False, board=None, task=None,
         phase="travel", turn=None, dur=None):
    p = {"type": kind, "from": [float(v) for v in frm], "to": [float(v) for v in to], "speed": speed,
         "move_s": float(move_s), "reverse": bool(reverse), "board": board, "task": task, "phase": phase,
         "turn": turn}
    p["dur"] = int(dur) if dur is not None else _ticks(move_s + settle, s)
    return p


def turn_prim(frm, heading, s, kind="turn", phase="travel", board=None, task=None):
    d = G.wrap(heading - frm[2])
    return prim(kind, frm, [frm[0], frm[1], frm[2] + d], s, speed=s.turn_dps,
                move_s=abs(math.degrees(d)) / s.turn_dps, settle=s.turn_settle_s, phase=phase, board=board,
                task=task, turn=d)


def drive_prims(a, b, s, speed=None, reverse=False, kind="drive", phase="travel", board=None, task=None,
                heading=None):
    """Straight a -> b, cut into pieces of at most chunk_cm."""
    a = np.asarray(a[:2], float)
    b = np.asarray(b[:2], float)
    L = float(np.linalg.norm(b - a))
    if L < 1e-6:
        return []
    v = speed or s.cruise_cms
    if heading is None:
        d = (b - a) / L
        heading = math.atan2(-d[1], -d[0]) if reverse else math.atan2(d[1], d[0])
    n = 1 if kind == "approach" else max(1, int(math.ceil(L / s.chunk_cm - 1e-9)))
    out = []
    for k in range(n):
        p0 = a + (b - a) * k / n
        p1 = a + (b - a) * (k + 1) / n
        out.append(prim(kind, [p0[0], p0[1], heading], [p1[0], p1[1], heading], s, speed=v,
                        move_s=(L / n) / v, settle=s.drive_settle_s if k == n - 1 else 0.0, reverse=reverse,
                        phase=phase, board=board, task=task))
    return out


def wait_prim(pose, ticks, s, phase="wait", board=None, task=None):
    return prim("wait", pose, pose, s, dur=ticks, phase=phase, board=board, task=task)


def prim_poses(p, s, step=None):
    """Poses (dur+1, 3) at every tick of a step (or every `step` seconds)."""
    dt = s.tick_s if step is None else step
    n = int(round(p["dur"] * s.tick_s / dt))
    t = np.arange(n + 1) * dt
    f = np.clip(t / p["move_s"], 0.0, 1.0) if p["move_s"] > 0 else np.ones_like(t)
    a = np.array(p["from"], float)
    b = np.array(p["to"], float)
    out = np.empty((len(t), 3))
    if p["type"] in ("turn", "park"):
        d = p["turn"] if p.get("turn") is not None else G.wrap(b[2] - a[2])
        out[:, 0], out[:, 1] = a[0], a[1]
        out[:, 2] = a[2] + f * d
    else:
        out[:, 0] = a[0] + f * (b[0] - a[0])
        out[:, 1] = a[1] + f * (b[1] - a[1])
        out[:, 2] = a[2]
    return out


def pose_in_prim(p, t, s):
    """Pose at time t (s) after the step began."""
    f = min(1.0, max(0.0, t / p["move_s"])) if p["move_s"] > 0 else 1.0
    a, b = p["from"], p["to"]
    if p["type"] in ("turn", "park"):
        d = p["turn"] if p.get("turn") is not None else G.wrap(b[2] - a[2])
        return [a[0], a[1], a[2] + f * d]
    return [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2]]


# ----------------------------------------------------------------------------
# Building a robot's steps from its tasks
# ----------------------------------------------------------------------------

class Job:
    """What one robot has to do in this plan."""

    def __init__(self, name, start, visits, park, park_pose, start_tick=0, lead=None, frozen=None,
                 task_offset=0, any_order=False):
        self.name = name
        self.start = list(start)          # [x, y, rad]
        self.visits = list(visits)        # board names or {"goto": [...]}
        self.park = park
        self.park_pose = list(park_pose)
        self.start_tick = start_tick      # remainder may not start before this tick
        self.lead = lead or []            # fixed steps done first (rest of an interrupted visit)
        self.frozen = frozen or []        # [(tick, prim)] already committed (replan prefix)
        self.task_offset = task_offset    # task numbering continues from the old plan
        self.any_order = any_order

    def hold(self):
        """Where this job's own new steps start."""
        return list(self.frozen[-1][1]["to"]) if self.frozen else list(self.start)


def _visit_prims(item, pose, task, grid, docks, s, body):
    """Steps to do one task from `pose`. Returns (prims, end pose)."""
    out = []
    if isinstance(item, str):
        d = docks[item]
        goal = d["predock"]
        out += _travel(pose, goal, grid, s, task, "board %s" % item)
        pose = out[-1]["to"] if out else pose
        out.append(turn_prim(pose, d["dock"][2], s, phase="align", board=item, task=task))
        pose = out[-1]["to"]
        out += _visit_tail(pose, item, d, task, s, "approach")
        return out, out[-1]["to"]
    g = item["goto"]
    out += _travel(pose, g, grid, s, task, "point (%.1f, %.1f)" % (g[0], g[1]))
    pose = out[-1]["to"] if out else pose
    if len(g) > 2:
        out.append(turn_prim(pose, math.radians(g[2]), s, phase="travel", task=task))
    return [p for p in out if not (p["type"] == "turn" and abs(p["turn"]) < math.radians(0.5))], \
        (out[-1]["to"] if out else pose)


def _visit_tail(pose, board, d, task, s, phase):
    """The fixed part of a board visit from `phase` on: approach, dwell, back out."""
    out = []
    if phase == "approach":
        ap = drive_prims(pose, d["dock"], s, speed=s.approach_cms, kind="approach", phase="approach",
                         board=board, task=task, heading=d["dock"][2])
        out += ap
        pose = ap[-1]["to"] if ap else pose
        phase = "dwell"
    if phase == "dwell":
        out.append(wait_prim(pose, _ticks(s.dwell_s, s), s, phase="dwell", board=board, task=task))
        phase = "backout"
    if phase == "backout":
        out += drive_prims(pose, d["predock"], s, speed=s.backout_cms, reverse=True, phase="backout",
                           board=board, task=task, heading=pose[2])
    return out


def _travel(pose, goal, grid, s, task, what):
    if not grid.turnable(goal):
        raise PlanError("%s: no room to turn there (%.1f cm clear, needs %.1f)"
                        % (what, grid.point_clear(goal), grid.turn_thr))
    pts = grid.route(pose, goal)
    if pts is None:
        raise PlanError("%s cannot be reached: no clear route" % what)
    out, h = [], pose[2]
    for a, b in zip(pts[:-1], pts[1:]):
        L = float(np.linalg.norm(b - a))
        if L < 0.05:
            continue
        hd = math.atan2(b[1] - a[1], b[0] - a[0])
        if abs(G.wrap(hd - h)) >= math.radians(0.5):
            out.append(turn_prim([a[0], a[1], h], hd, s, task=task))
        out += drive_prims(a, b, s, task=task)
        h = hd
    return out


def _escape(job, grid, world, body, s):
    """A robot that is too close to something to turn backs away (or drives
    on) in a straight line until it can turn. Returns steps."""
    p = job.start
    if grid.turnable(p):
        return []
    here = world.shape_clearance(body.corners([p])[0])
    need = min(s.safety_margin_cm, here) - 1e-6
    fwd = np.array([math.cos(p[2]), math.sin(p[2])])
    for sign, rev in ((-1, True), (1, False)):
        for dist in np.arange(1.0, 31.0, 1.0):
            q = np.array(p[:2]) + sign * dist * fwd
            if not grid.turnable(q):
                continue
            sweep = body.drive_sweep(p[:2], q, p[2])
            if world.shape_clearance(sweep) >= need:
                return drive_prims(p, q, s, speed=s.backout_cms, reverse=rev, phase="escape", heading=p[2])
    raise PlanError("robot %s is boxed in at (%.1f, %.1f): move it by hand" % (job.name, p[0], p[1]))


def _nearest_free(grid, p, frm, max_cm=40.0):
    """Closest point to p where the robot can turn and that it can reach from frm."""
    d = np.hypot(grid.centres[:, 0] - p[0], grid.centres[:, 1] - p[1])
    cand = np.nonzero(grid.free & (d < max_cm))[0]
    for k in cand[np.argsort(d[cand])][:200]:
        if np.isfinite(grid.geodesic(frm, grid.centres[k])):
            return grid.centres[k].tolist()
    return None


def build_prims(job, order, grid, world, docks, s, body, warnings, allow_skip=True):
    """All of a job's steps, for a given order of its visits."""
    out = []
    pose = list(job.hold())
    if job.lead:
        out += job.lead
        pose = out[-1]["to"]
    else:
        out += _escape(Job(job.name, pose, [], False, pose), grid, world, body, s)
        if out:
            pose = out[-1]["to"]
    for pos, k in enumerate(order):
        item = job.visits[k]
        task = job.task_offset + pos
        try:
            ps, pose2 = _visit_prims(item, pose, task, grid, docks, s, body)
        except PlanError as e:
            if s.skip_unreachable and allow_skip:
                warnings.append("robot %s skips task %d: %s" % (job.name, task + 1, e))
                continue
            raise PlanError("robot %s, task %d: %s" % (job.name, task + 1, e))
        out += ps
        pose = pose2
    if job.park:
        task = job.task_offset + len(job.visits)
        pk = list(job.park_pose)
        if not grid.turnable(pk):
            alt = _nearest_free(grid, pk, pose)
            if alt is None:
                raise PlanError("robot %s: park spot blocked and no free spot near it" % job.name)
            warnings.append("robot %s: park spot (%.1f, %.1f) is blocked; parks at (%.1f, %.1f) instead"
                            % (job.name, pk[0], pk[1], alt[0], alt[1]))
            pk = [alt[0], alt[1], pk[2]]
        try:
            out += _travel(pose, pk, grid, s, task, "park spot")
        except PlanError as e:
            raise PlanError("robot %s: %s" % (job.name, e))
        pose = out[-1]["to"] if out else pose
        out.append(turn_prim(pose, pk[2], s, kind="park", phase="park", task=task))
    return out


# ----------------------------------------------------------------------------
# Timing against the reservation table
# ----------------------------------------------------------------------------

def _timeline(sched, start_pose, s):
    """Poses at every tick from 0 for a schedule [(tick, prim)]; the robot
    stands at start_pose before its first step."""
    if not sched:
        return np.array([start_pose], float)
    end = sched[-1][0] + sched[-1][1]["dur"]
    T = np.empty((end + 1, 3))
    T[:] = start_pose
    for t0, p in sched:
        T[t0:t0 + p["dur"] + 1] = prim_poses(p, s)
    return T


def _slack(body, s):
    """How much two bodies can close in between two 0.1 s samples."""
    m = max(s.cruise_cms * s.tick_s, math.radians(s.turn_dps) * s.tick_s * body.r_turn)
    return m


def _others_at(T, t0, n):
    if t0 + n <= len(T):
        return T[t0:t0 + n]
    if t0 >= len(T):
        return np.repeat(T[-1:], n, axis=0)
    return np.vstack([T[t0:], np.repeat(T[-1:], t0 + n - len(T), axis=0)])


INF_TICK = 10 ** 9


def _pkey(pr):
    """Key that identifies a step's swept area by its geometry."""
    t = pr.get("turn")
    return (pr["type"], tuple(np.round(pr["from"], 4)), tuple(np.round(pr["to"], 4)), round(pr["move_s"], 4),
            pr["dur"], None if t is None else round(t, 5))


def _sweep_samples(pr, s):
    """Poses along a step, deduplicated (a wait is one pose)."""
    if pr["type"] == "wait":
        return np.array([pr["from"]], float)
    P = prim_poses(pr, s)
    if len(P) > 40:
        P = P[np.linspace(0, len(P) - 1, 40).round().astype(int)]
    return P


class Occ:
    """One stretch of time one robot spends doing one step: [s, e] in ticks,
    and the poses it passes through (its swept area)."""
    __slots__ = ("s", "e", "P", "c", "r", "key")

    def __init__(self, s_, e_, P, body, key):
        self.s, self.e, self.P = s_, e_, P
        self.c = P[:, :2].mean(0)
        self.r = float(np.max(np.linalg.norm(P[:, :2] - self.c, axis=1))) + body.r_turn
        self.key = key


def occupancy(sched, start_pose, s, body, hold_forever=True, hold_until=None):
    """A robot's timeline as Occ stretches: standing at its start before its
    first step, each step, then standing at its last pose for good."""
    out = []
    first = sched[0][0] if sched else INF_TICK
    if first > 0 or not sched:
        P = np.array([start_pose], float)
        out.append(Occ(0, first if sched else (INF_TICK if hold_until is None else hold_until), P, body,
                       ("pose",) + tuple(np.round(P[0], 6))))
    for t0, pr in sched:
        out.append(Occ(t0, t0 + pr["dur"], _sweep_samples(pr, s), body, _pkey(pr)))
    if sched and hold_forever:
        P = np.array([sched[-1][1]["to"]], float)
        end = sched[-1][0] + sched[-1][1]["dur"]
        stop = INF_TICK if hold_until is None else max(end, hold_until)
        out.append(Occ(end, stop, P, body, ("pose",) + tuple(np.round(P[0], 6))))
    return out


_OVERLAP_CACHE = {}


def sweeps_meet(A, B, body, thr):
    """Do two swept areas come within thr of each other anywhere?"""
    if np.hypot(*(A.c - B.c)) > A.r + B.r + thr:
        return False
    key = (A.key, B.key, thr) if A.key is not None and B.key is not None else None
    if key is not None and key in _OVERLAP_CACHE:
        return _OVERLAP_CACHE[key]
    PA, PB = A.P, B.P
    ia = np.repeat(np.arange(len(PA)), len(PB))
    ib = np.tile(np.arange(len(PB)), len(PA))
    d = np.hypot(PA[ia, 0] - PB[ib, 0], PA[ia, 1] - PB[ib, 1])
    close = d < 2 * body.r_turn + thr
    res = bool(close.any() and np.any(G.bodies_within(body, PA[ia[close]], PB[ib[close]], thr)))
    if key is not None:
        if len(_OVERLAP_CACHE) > 200000:
            _OVERLAP_CACHE.clear()
        _OVERLAP_CACHE[key] = res
        _OVERLAP_CACHE[(B.key, A.key, thr)] = res
    return res


def _thr(body, s):
    return s.separation_cm + _slack(body, s)


def schedule(prims, start_pose, t_start, others, s, body, wait_early=False):
    """When to wait, and for how long, along a fixed list of steps.

    Rule: this robot may not be doing a step (or standing somewhere) at the
    same time as another robot does a step whose swept area comes within
    the separation of it. Because no two such steps ever overlap in time,
    the step order rules ("after") reproduce the plan exactly, and robots
    that obey them stay apart even when some run late.

    For every step k it works out, for all ticks at once, whether standing
    at the pose before step k is allowed at tick t (stand_ok) and whether
    setting off on step k at tick t is (go_ok), then sweeps through the
    steps keeping every tick the robot could be at each pose, and takes the
    earliest finish from which standing still is allowed for good. Exact at
    the tick resolution. Returns [(tick, prim)] with wait steps, or None.

    others: list of Occ lists (see occupancy())."""
    thr = _thr(body, s)
    K = len(prims)
    dur = [p["dur"] for p in prims]
    pose_at = [np.array(start_pose, float)] + [np.array(p["to"], float) for p in prims]
    occ = [o for lst in others for o in lst]
    last_finite = max([o.s for o in occ] + [o.e for o in occ if o.e < INF_TICK] + [0])
    H = max(t_start + sum(dur) + int(s.max_wait_s / s.tick_s), last_finite) + 2
    t0 = t_start
    n = H - t0

    def table(oc, d, skip_initial=False):
        bad = np.zeros(n, bool)
        for o in occ:
            if skip_initial and o.s == 0 and len(o.P) == 1 and np.allclose(oc.P, pose_at[0][None]):
                continue     # both standing where they already are: nothing anyone can change
            lo, hi = o.s - d, o.e        # departures that overlap [o.s, o.e]
            if hi < t0 or lo >= H:
                continue
            if sweeps_meet(oc, o, body, thr):
                bad[max(lo, t0) - t0:min(hi, H - 1) - t0 + 1] = True
        return ~bad

    stand_ok = []
    for k in range(K + 1):
        P = pose_at[k][None]
        stand_ok.append(table(Occ(0, 0, P, body, ("pose",) + tuple(np.round(P[0], 6))), 0, skip_initial=(k == 0)))
    stand_ok[0][0] = True
    reach_in = np.zeros(n, bool)
    reach_in[0] = True
    entries = []
    for k in range(K + 1):
        ok = stand_ok[k]
        e = reach_in & ok
        entries.append(e)
        run_id = np.cumsum(~ok)
        cnt = np.cumsum(e)
        first = np.r_[True, run_id[1:] != run_id[:-1]]
        idx = np.maximum.accumulate(np.where(first, np.arange(n), 0))
        reach = ok & (cnt - (cnt[idx] - e[idx]) > 0)
        if k == K:
            break
        go_ok = table(Occ(0, 0, _sweep_samples(prims[k], s), body, _pkey(prims[k])), dur[k])
        dep = reach & go_ok
        nxt = np.zeros(n, bool)
        d = dur[k]
        if d < n:
            nxt[d:] = dep[:n - d]
        reach_in = nxt
        if not reach_in.any():
            return None
    okK = stand_ok[K]
    safe_after = np.flip(np.logical_and.accumulate(np.flip(okK)))
    fin = np.nonzero(entries[K] & safe_after)[0]
    if len(fin) == 0:
        return None
    t = int(fin[0])
    plan = []
    for k in range(K, 0, -1):
        tdep = t - dur[k - 1]
        plan.append((tdep, k - 1, t))
        ok, en = stand_ok[k - 1], entries[k - 1]
        tt, pick = tdep, None
        while tt >= 0 and ok[tt]:
            if en[tt]:
                pick = tt
                if wait_early:
                    break
            tt -= 1
        t = pick
    plan.reverse()
    out, cur = [], 0
    for tdep, k, tarr in plan:
        if tdep > cur:
            out.append((t0 + cur, wait_prim(pose_at[k].tolist(), tdep - cur, s, phase="wait",
                                            task=prims[k]["task"])))
        out.append((t0 + tdep, prims[k]))
        cur = tarr
    return out


# ----------------------------------------------------------------------------
# The whole fleet
# ----------------------------------------------------------------------------

class Planner:
    def __init__(self, m, over=None):
        self.m = FM.normalize(m)
        self.s = settings(over)
        self.body = FM.body_of(self.m)
        self.docks = FM.docks(self.m)
        self.world = FM.World(self.m)
        self.grid = Grid(self.world, self.body, self.s)
        self.warnings = []
        self.allow_strand = False
        self._fallback = False
        self._stranded = set()
        self.deadline = None

    # -- visit orders --------------------------------------------------------
    def _goal_xy(self, item):
        return self.docks[item]["predock"][:2] if isinstance(item, str) else item["goto"][:2]

    def candidate_orders(self, job, keep=3):
        n = len(job.visits)
        base = list(range(n))
        if not job.any_order or n < 2:
            return [base]
        pts = [job.hold()[:2]] + [self._goal_xy(v) for v in job.visits] + [job.park_pose[:2]]
        D = np.array([[self.grid.geodesic(a, b) for b in pts] for a in pts])

        def cost(o):
            seq = [0] + [k + 1 for k in o] + ([n + 1] if job.park else [])
            return sum(D[a, b] for a, b in zip(seq[:-1], seq[1:]))
        if n <= 6:
            allo = sorted(itertools.permutations(base), key=cost)
        else:   # greedy nearest-next, then the given order
            o, left, cur = [], set(base), 0
            while left:
                k = min(left, key=lambda j: D[cur, j + 1])
                o.append(k)
                left.remove(k)
                cur = k + 1
            allo = [tuple(o), tuple(base)]
        out = []
        for o in allo:
            if np.isfinite(cost(o)) and list(o) not in out:
                out.append(list(o))
            if len(out) >= keep:
                break
        return out or [base]

    # -- one robot against fixed others ----------------------------------------
    def _schedule_job(self, job, prims_options, others):
        """Best (earliest finishing) timeline for one job over several step
        lists, against the others' occupancy. Falls back to routing around
        where the others stand still. Returns the full timeline or None."""
        s, body = self.s, self.body
        t0 = max(job.start_tick, (job.frozen[-1][0] + job.frozen[-1][1]["dur"]) if job.frozen else 0)
        start_pose = job.hold()
        best = None
        for prims in prims_options:
            sch = schedule(prims, start_pose, t0, others, s, body) if prims else []
            if sch is not None:
                end = sch[-1][0] + sch[-1][1]["dur"] if sch else t0
                if best is None or end < best[0]:
                    best = (end, sch)
        long_wait = best is not None and any(p["type"] == "wait" and p["phase"] == "wait" and p["dur"] * s.tick_s > 10
                                             for _, p in best[1])
        late = self.deadline is not None and time.perf_counter() > self.deadline
        if best is None and late and self._fallback:
            return None          # out of time in the last-resort pass: this robot stays put
        if best is None or (long_wait and not late):
            alt = self._reroute(job, prims_options, others, start_pose, t0)
            if alt is not None and (best is None or alt[0] < best[0]):
                best = alt
        if best is None and not job.lead and not (late and self._fallback):
            best = self._make_room(job, others, start_pose, t0)
        if best is None:
            return None
        sch = best[1]
        full = list(job.frozen)
        gap_from = (job.frozen[-1][0] + job.frozen[-1][1]["dur"]) if job.frozen else 0
        if sch and sch[0][0] > gap_from:
            full.append((gap_from, wait_prim(start_pose, sch[0][0] - gap_from, s, phase="hold")))
        full += sch
        return full

    def _avoid_variants(self, job, others, start_pose):
        """Maps with circles round the other robots: where they stand still,
        then also along their routes, then the same with more room. Each
        circle is shrunk so it never covers this robot's own start or goals
        (the timing check still keeps the separation)."""
        s, body = self.s, self.body
        R = body.r_turn + s.separation_cm + _slack(body, s) - s.safety_margin_cm
        mine = [np.array(start_pose[:2])] + [np.array(self._goal_xy(v), float) for v in job.visits]
        if job.park:
            mine.append(np.array(job.park_pose[:2], float))

        def radius(sp):
            room = min(float(np.linalg.norm(sp - q)) for q in mine) - self.grid.turn_thr - 0.5
            r = min(R, room)
            return r if r >= body.r_in else None
        rest, lanes = [], []
        for lst in others:
            for o in lst:
                if len(o.P) == 1 and (o.e - o.s) * s.tick_s >= 2.0:
                    rest.append(o.P[0, :2])
                elif o.e < INF_TICK:
                    lanes += [o.P[i, :2] for i in range(0, len(o.P), 4)] + [o.P[-1, :2]]
        rest = [(p, radius(p)) for p in rest]
        rest = [(p, r) for p, r in rest if r is not None]
        lanes = [(p, radius(p)) for p in lanes]
        lanes = [(p, r) for p, r in lanes if r is not None]
        variants = []
        if rest:
            variants.append(rest)
        if lanes:
            variants.append(rest + lanes)
            variants.append(rest + [(p, r + 5.0 if r >= R - 1e-9 else r) for p, r in lanes])
        for circ in variants:
            extra = [{"id": "robot", "shape": "circle", "center": p.tolist(), "radius": r} for p, r in circ]
            w2 = FM.World(self.m, extra=extra)
            yield w2, Grid(w2, body, s, base=self.grid, circles=circ)

    @staticmethod
    def _no_long_wait(sch, s):
        return not any(p["type"] == "wait" and p["phase"] == "wait" and p["dur"] * s.tick_s > 10 for _, p in sch)

    def _reroute(self, job, prims_options, others, start_pose, t0):
        """Route around the other robots (see _avoid_variants)."""
        s, body = self.s, self.body
        best = None
        for w2, g2 in self._avoid_variants(job, others, start_pose):
            for vo in self._orders_for(job):
                try:
                    prims2 = build_prims(job, vo, g2, w2, self.docks, s, body, [], allow_skip=False)
                except PlanError:
                    continue
                sch = schedule(prims2, start_pose, t0, others, s, body)
                if sch is not None:
                    end = sch[-1][0] + sch[-1][1]["dur"] if sch else t0
                    if best is None or end < best[0]:
                        best = (end, sch)
            if best is not None and self._no_long_wait(best[1], s):
                break
        return best

    def _make_room(self, job, others, start_pose, t0):
        """Two robots standing too close to turn (a turn sweeps a circle)
        block each other. Try a short straight move first, forwards or
        backwards along the current heading, then a route from there, on
        the plain map or around the other robots."""
        s, body = self.s, self.body
        h = np.array([math.cos(start_pose[2]), math.sin(start_pose[2])])
        here = self.world.shape_clearance(body.corners([start_pose])[0])
        moves = []
        for dist in (5.0, 10.0, 15.0):
            for sign in (1, -1):
                qpt = np.array(start_pose[:2]) + sign * dist * h
                if not self.grid.turnable(qpt):
                    continue
                if self.world.shape_clearance(body.drive_sweep(start_pose[:2], qpt, start_pose[2])) < \
                        min(s.safety_margin_cm, here) - 1e-6:
                    continue
                moves.append((qpt, sign))
        if not moves:
            return None
        best = None
        for qpt, sign in moves:
            jq = Job(job.name, [qpt[0], qpt[1], start_pose[2]], job.visits, job.park, job.park_pose,
                     task_offset=job.task_offset, any_order=job.any_order)
            nudge = drive_prims(start_pose, qpt, s, speed=s.cruise_cms if sign > 0 else s.backout_cms,
                                reverse=sign < 0, phase="escape", heading=start_pose[2], task=job.task_offset)
            maps = itertools.chain([(self.world, self.grid)], self._avoid_variants(jq, others, jq.start))
            for w2, g2 in maps:
                try:
                    rest = build_prims(jq, self._orders_for(job)[0], g2, w2, self.docks, s, body, [],
                                       allow_skip=False)
                except PlanError:
                    continue
                sch = schedule(nudge + rest, start_pose, t0, others, s, body)
                if sch is not None:
                    end = sch[-1][0] + sch[-1][1]["dur"]
                    if best is None or end < best[0]:
                        best = (end, sch)
                    if self._no_long_wait(sch, s):
                        break
            if best is not None and self._no_long_wait(best[1], s):
                break
        return best

    def _orders_for(self, job):
        return self._cand.get(job.name, [list(range(len(job.visits)))])

    def _prims(self, job, vo, cache):
        key = (job.name, tuple(vo))
        if key not in cache:
            cache[key] = build_prims(job, vo, self.grid, self.world, self.docks, self.s, self.body, self.warnings)
        return cache[key]

    # -- one priority order ----------------------------------------------------
    def _plan_order(self, jobs, order, visit_orders, prim_cache, hold_s=None, strand=False):
        """Plan robots one after another. Robots not planned yet are assumed
        to stand where they are: for good (hold_s None), or only for hold_s
        seconds. Either way the plan is valid: each robot planned later is
        checked against the earlier ones' complete timelines."""
        s, body = self.s, self.body
        scheds = {}
        OC = {}
        for j in jobs:
            hu = None
            if hold_s is not None:
                fe = (j.frozen[-1][0] + j.frozen[-1][1]["dur"]) if j.frozen else 0
                hu = max(fe, j.start_tick) + int(hold_s / s.tick_s)
            OC[j.name] = occupancy(j.frozen, j.start, s, body, hold_until=hu)
        for name in order:
            job = next(j for j in jobs if j.name == name)
            prims = self._prims(job, visit_orders[name], prim_cache)
            others = [OC[o] for o in OC if o != name]
            full = self._schedule_job(job, [prims], others)
            if full is None:
                if not strand:
                    return None
                # last resort: this robot stays where it is (others plan around it)
                full = list(job.frozen)
                self._stranded.add(name)
            scheds[name] = full
            OC[name] = occupancy(full, job.start, s, body)
        return scheds

    def _refine(self, jobs, scheds, prim_cache, deadline):
        """Re-plan each robot against everyone else's complete timeline (no
        more "parked for good" guesses about robots planned later), keeping
        any earlier finish. Valid because the conflict rule is symmetric."""
        s, body = self.s, self.body
        scheds = dict(scheds)

        def end(n):
            return scheds[n][-1][0] + scheds[n][-1][1]["dur"] if scheds[n] else 0
        for _ in range(3):
            improved = False
            for name in sorted(scheds, key=end, reverse=True):
                if time.perf_counter() > deadline:
                    return scheds
                job = next(j for j in jobs if j.name == name)
                others = [occupancy(scheds[o], next(j for j in jobs if j.name == o).start, s, body)
                          for o in scheds if o != name]
                opts = []
                for vo in self._orders_for(job):
                    try:
                        opts.append(self._prims(job, vo, prim_cache))
                    except PlanError:
                        pass
                full = self._schedule_job(job, opts, others)
                if full is not None:
                    e_new = full[-1][0] + full[-1][1]["dur"] if full else 0
                    if e_new < end(name):
                        scheds[name] = full
                        improved = True
            if not improved:
                break
        return scheds

    def plan(self, jobs, t_limit_s=60.0):
        """Plan all jobs. Returns (schedules, info)."""
        t_begin = time.perf_counter()
        self.deadline = t_begin + t_limit_s
        names = [j.name for j in jobs]
        cand = {j.name: self.candidate_orders(j) for j in jobs}
        self._cand = cand
        visit_orders = {n: cand[n][0] for n in names}
        prim_cache = {}
        # most promising first (the anytime limit may stop the search early):
        # robots with the most work to do go first
        work = {}
        for j in jobs:
            try:
                work[j.name] = sum(p["dur"] for p in self._prims(j, visit_orders[j.name], prim_cache))
            except PlanError:
                raise
        # a robot in the middle of a docking (backing out, say) has a fixed
        # next move and is in front of a board others may need: it goes first
        busy = {j.name: 1 if j.lead else 0 for j in jobs}
        orders = sorted(itertools.permutations(names),
                        key=lambda o: [(-busy[n], -work[n]) for n in o])[: self.s.max_orders]
        tried = []
        best = None

        def score(sc):
            ends = [sc[n][-1][0] + sc[n][-1][1]["dur"] if sc[n] else 0 for n in names]
            return (max(ends + [0]), sum(ends))

        def run(vo, ords, holds=(3.0, None, 8.0)):
            nonlocal best
            for od in ords:
                for hold_s in holds:
                    if (best is not None or self.allow_strand) and time.perf_counter() - t_begin > t_limit_s:
                        return
                    sc = self._plan_order(jobs, od, vo, prim_cache, hold_s=hold_s)
                    sc_ = score(sc) if sc else None
                    tried.append({"order": list(od), "others_wait_s": hold_s,
                                  "visit_orders": {k: list(v) for k, v in vo.items()},
                                  "makespan_s": round(sc_[0] * self.s.tick_s, 1) if sc_ else None})
                    if sc and (best is None or sc_ < best[0]):
                        best = (sc_, sc, list(od), {k: list(v) for k, v in vo.items()})

        run(visit_orders, orders)
        # robots whose visits may be reordered: try their other good orders
        for _ in range(2):
            improved = False
            for n in names:
                for alt in cand[n][1:]:
                    if best is None:
                        break
                    vo = dict(best[3])
                    vo[n] = alt
                    before = best[0]
                    run(vo, [tuple(best[2])] + [o for o in orders if list(o) != best[2]][:5])
                    if best[0] < before:
                        improved = True
            if not improved:
                break
        stranded = set()
        if best is None and self.allow_strand:
            # no order moves everyone safely: let the robots that cannot move
            # stay where they are, and keep the order that strands the fewest
            self._fallback = True
            for od in orders:
                if best is not None and time.perf_counter() > self.deadline:
                    break
                self._stranded = set()
                sc = self._plan_order(jobs, od, visit_orders, prim_cache, hold_s=None, strand=True)
                k = (len(self._stranded),) + score(sc)
                if best is None or k < best[0]:
                    best = (k, sc, list(od), {n: list(v) for n, v in visit_orders.items()})
                    stranded = set(self._stranded)
                if not stranded:
                    break
            best = (best[0][1:], best[1], best[2], best[3])
            for n in sorted(stranded):
                self.warnings.append("robot %s stays where it is for now: no safe route yet (it gets one at "
                                     "the next re-plan, e.g. once the others have moved)" % n)
        if best is None:
            raise PlanError("no priority order gave a plan without robots meeting: the routes may be too "
                            "crowded (try fewer robots at once, a smaller separation, or other start spots)")
        refined = self._refine(jobs, best[1], prim_cache, t_begin + t_limit_s * 1.5)
        best = (score(refined), refined, best[2], best[3])
        info = {"priority_order": best[2], "visit_orders": best[3], "orders_tried": tried, "stranded": sorted(stranded),
                "planning_time_s": round(time.perf_counter() - t_begin, 3)}
        return best[1], info


# ----------------------------------------------------------------------------
# Step order rules ("after") and checks
# ----------------------------------------------------------------------------

def dependencies(scheds, body, s):
    """For each robot and step: the other robots' steps that must be done
    before it starts. Two steps whose swept areas come within the separation
    never overlap in time in a plan (see schedule()), so the earlier one must
    finish before the later one starts. Only the latest such step per other
    robot is listed; earlier ones follow from that robot's own order."""
    thr = _thr(body, s)
    names = list(scheds)
    occ = {n: [Occ(t0, t0 + p["dur"], _sweep_samples(p, s), body, _pkey(p)) for t0, p in scheds[n]] for n in names}
    after = {n: [dict() for _ in scheds[n]] for n in names}
    for a, b in itertools.permutations(names, 2):
        for i, A in enumerate(occ[a]):
            for j, B in enumerate(occ[b]):
                if B.e <= A.s and sweeps_meet(A, B, body, thr):
                    after[a][i][b] = max(after[a][i].get(b, -1), j)
    return {n: [sorted([[r, j] for r, j in d.items()]) for d in after[n]] for n in names}


def verify(m, plan, step_s=0.02):
    """Independent check of a plan: closest approach of every pair of robots
    (sampled every step_s), and clearance of every step's swept area from
    the static obstacles, computed exactly. Returns a report dict."""
    m = FM.normalize(m)
    body = FM.body_of(m)
    s = settings(plan["settings"])
    world = FM.World(m)
    names = list(plan["robots"])
    T_end = max([r["finish_s"] for r in plan["robots"].values()] + [0.0])
    ts = np.arange(0.0, T_end + step_s, step_s)
    poses = {n: np.array([pose_at(plan, n, t) for t in ts]) for n in names}
    min_sep, at = math.inf, None
    near = []
    warn_at = s.separation_cm + 3.0
    for a, b in itertools.combinations(names, 2):
        g = G.body_gaps(body, poses[a], poses[b]) if len(ts) else np.array([math.inf])
        k = int(np.argmin(g))
        if g[k] < min_sep:
            min_sep, at = float(g[k]), (a, b, round(float(ts[k]), 2))
        close = g < warn_at
        i = 0
        while i < len(close):
            if close[i]:
                j = i
                while j + 1 < len(close) and close[j + 1]:
                    j += 1
                gm = float(g[i:j + 1].min())
                near.append({"robots": [a, b], "t_from": round(float(ts[i]), 2), "t_to": round(float(ts[j]), 2),
                             "min_gap_cm": round(gm, 2), "kind": "too close" if gm < s.separation_cm else "near miss"})
                i = j + 1
            else:
                i += 1
    min_static, where = math.inf, None
    docks = FM.docks(m)
    for n in names:
        for st in plan["robots"][n]["steps"]:
            p = step_to_prim(st)
            if st["type"] in ("approach", "backout") or st["phase"] in ("approach", "backout", "escape", "dwell"):
                continue   # checked below against everything but the target board
            if st["type"] in ("turn", "park"):
                c = float(world.clearance(np.array(p["from"][:2])[None])[0]) - body.r_turn
            elif st["type"] == "drive":
                c = world.shape_clearance(body.drive_sweep(p["from"][:2], p["to"][:2], p["from"][2]))
            else:
                c = world.shape_clearance(body.corners([p["from"]])[0])
            if c < min_static:
                min_static, where = c, (n, st["i"], st["type"])
    approach = []
    for n in names:
        for st in plan["robots"][n]["steps"]:
            if st["phase"] in ("approach", "backout", "dwell") and st.get("board"):
                p = step_to_prim(st)
                w2 = FM.World(m, exclude_boards=(st["board"],))
                c = w2.shape_clearance(body.drive_sweep(p["from"][:2], p["to"][:2], p["from"][2]))
                approach.append(c)
                if st["type"] == "approach":
                    d = docks[st["board"]]
                    front = np.array(p["to"][:2]) + body.front * np.array([math.cos(p["to"][2]), math.sin(p["to"][2])])
                    gap = float(np.dot(front - np.array(d["surface"]), np.array(d["normal"])))
                    approach.append(("gap", gap))
    gaps = [g[1] for g in approach if isinstance(g, tuple)]
    cl = [g for g in approach if not isinstance(g, tuple)]
    return {"min_separation_cm": round(min_sep, 2), "closest_pair": at, "near_misses": near,
            "min_static_clearance_cm": round(min_static, 2) if np.isfinite(min_static) else None,
            "tightest_step": where,
            "approach_min_clearance_cm": round(min(cl), 2) if cl else None,
            "planned_dock_gaps_cm": [round(g, 3) for g in gaps]}


# ----------------------------------------------------------------------------
# plan.json
# ----------------------------------------------------------------------------

def _deg(p):
    return FM.pose_out(p)


def to_steps(sched, after, s, t0=0.0, committed=0, front_ray=None):
    out = []
    for i, (tk, p) in enumerate(sched):
        st = {"i": i, "type": p["type"], "phase": p["phase"], "from": _deg(p["from"]), "to": _deg(p["to"]),
              "t_start": round(tk * s.tick_s, 2), "t_end": round((tk + p["dur"]) * s.tick_s, 2),
              "move_s": round(p["move_s"], 3), "board": p["board"], "task": p["task"], "after": after[i]}
        if p["type"] in ("drive", "approach"):
            st["speed"] = p["speed"]
            st["reverse"] = p["reverse"]
        elif p["type"] in ("turn", "park"):
            st["speed"] = p["speed"]
            st["turn_deg"] = round(math.degrees(p["turn"]), 2)
        else:
            st["speed"] = None
        if p["type"] == "drive" and not p["reverse"] and front_ray is not None:
            st["front_clear_cm"] = front_ray(p["to"])
        if p["type"] == "approach":
            L = math.dist(p["from"][:2], p["to"][:2])
            st["stop_at_front_cm"] = None   # filled by make_plan from the map
            st["max_travel_cm"] = round(L + 3.0, 2)
        if p["type"] == "drive" and p["phase"] == "backout":
            pass
        if i < committed:
            st["committed"] = True
        out.append(st)
    return out


def _front_ray_fn(m, body, max_cm=100.0):
    """Distance the front sensor should read at a pose: from the front face
    centre straight ahead to the nearest board or obstacle on the map
    (keep-outs and the floor edge are not walls). None = nothing in range."""
    polys = [G.segment_rect(b["p1"], b["p2"], b["thickness_cm"]) for b in m["boards"]]
    for o in m["obstacles"]:
        if o["shape"] == "polygon":
            polys.append(np.array(o["points"], float))
        else:
            c, r = np.array(o["center"]), o["radius"]
            a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
            polys.append(np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a)])

    def f(pose):
        u = np.array([math.cos(pose[2]), math.sin(pose[2])])
        o = np.array(pose[:2]) + body.front * u
        best = max_cm
        for poly in polys:
            for i in range(len(poly)):
                a, b = poly[i], poly[(i + 1) % len(poly)]
                e = b - a
                den = u[0] * (-e[1]) - u[1] * (-e[0])
                if abs(den) < 1e-12:
                    continue
                rr = a - o
                t = (rr[0] * (-e[1]) - rr[1] * (-e[0])) / den
                sg = (u[0] * rr[1] - u[1] * rr[0]) / den
                if 0 <= t < best and 0 <= sg <= 1:
                    best = t
        return round(best, 1) if best < max_cm else None
    return f


def step_to_prim(st):
    p = {"type": st["type"], "from": FM.pose_in(st["from"]), "to": FM.pose_in(st["to"]),
         "move_s": st["move_s"], "phase": st["phase"], "board": st.get("board"), "task": st.get("task"),
         "speed": st.get("speed"), "reverse": st.get("reverse", False),
         "turn": math.radians(st["turn_deg"]) if "turn_deg" in st else None,
         "dur_s": st["t_end"] - st["t_start"]}
    return p


def pose_at(plan, name, t):
    """Planned pose [x, y, rad] of a robot at plan time t (seconds)."""
    steps = plan["robots"][name]["steps"]
    if not steps or t <= steps[0]["t_start"]:
        return FM.pose_in(plan["robots"][name]["start"])
    for st in steps:
        if st["t_start"] <= t < st["t_end"]:
            return pose_in_prim(step_to_prim(st), t - st["t_start"], None)
    return FM.pose_in(steps[-1]["to"])


def make_plan(m, jobs=None, over=None, t0_s=0.0, replan_of=None, events=None, t_limit_s=60.0, allow_strand=False):
    """Plan from a map (its robots' start spots and tasks), or from given jobs."""
    m = FM.normalize(m)
    pl = Planner(m, over)
    pl.allow_strand = bool(allow_strand)
    s, body = pl.s, pl.body
    if jobs is None:
        jobs = jobs_from_map(m)
    if not jobs:
        raise PlanError("no robots in the map")
    scheds, info = pl.plan(jobs, t_limit_s=t_limit_s)
    names = [j.name for j in jobs]
    sc = {n: scheds[n] for n in names}
    front_ray = _front_ray_fn(m, body)
    after = dependencies(sc, body, s)
    robots = {}
    for j in jobs:
        steps = to_steps(sc[j.name], after[j.name], s, committed=len(j.frozen), front_ray=front_ray)
        if j.start_tick > 0:
            # nothing new starts before the new plan can be ready
            for st in steps[len(j.frozen):]:
                if st["type"] != "wait":
                    st["not_before_s"] = round(j.start_tick * s.tick_s, 2)
                    break
        for st in steps:
            if st["type"] == "approach":
                st["stop_at_front_cm"] = m["dock"]["gap_cm"]
        vo = info["visit_orders"][j.name]
        if j.name in info["stranded"]:
            # work it still has to do, kept for the next re-plan
            lead = j.lead[0] if j.lead else None
            robots_pending = {"visits": j.visits, "task_offset": j.task_offset, "park": j.park,
                              "lead_board": lead["board"] if lead else None, "lead_phase": lead["phase"] if lead else None}
        else:
            robots_pending = None
        robots[j.name] = {"start": FM.pose_out(j.start), "task_offset": j.task_offset, "pending": robots_pending,
                          "visit_order": [j.visits[k] for k in vo] + (["park"] if j.park else []),
                          "finish_s": steps[-1]["t_end"] if steps else 0.0,
                          "steps": steps}
    used = sorted({v for j in jobs for v in j.visits if isinstance(v, str)})
    plan = {"format": PLAN_FORMAT, "version": PLAN_VERSION, "created": FM.now_iso(),
            "map_hash": FM.map_hash(m), "t0_s": t0_s, "replan_of": replan_of,
            "settings": dict(s), "priority_order": info["priority_order"],
            "docks": {b: {"dock": FM.pose_out(pl.docks[b]["dock"]), "predock": FM.pose_out(pl.docks[b]["predock"])}
                      for b in used},
            "events": events or [], "warnings": pl.warnings, "stranded": info["stranded"], "robots": robots}
    rep = verify(m, plan)
    fin = [r["finish_s"] for r in robots.values()]
    plan["summary"] = {"makespan_s": max(fin) if fin else 0.0, "sum_finish_s": round(sum(fin), 2),
                       "planning_time_s": info["planning_time_s"], "orders_tried": len(info["orders_tried"]),
                       "order_results": info["orders_tried"][:30], **rep}
    plan["plan_id"] = plan_id(plan)
    return plan


def plan_id(plan):
    import hashlib
    body = {k: plan[k] for k in ("map_hash", "robots", "t0_s")}
    return "plan-" + hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()[:10]


def jobs_from_map(m):
    jobs = []
    for r in m["robots"]:
        t = m["tasks"].get(r["name"], {"visit": [], "any_order": False, "park": True})
        jobs.append(Job(r["name"], FM.pose_in(r["start"]), t["visit"], t["park"], FM.pose_in(r["park"]),
                        any_order=t["any_order"]))
    return jobs


def save_plan(plan, path):
    import os
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(plan, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


# ----------------------------------------------------------------------------
# Re-planning after a surprise
# ----------------------------------------------------------------------------

def _tasks_of(m, name):
    return m["tasks"].get(name, {"visit": [], "any_order": False, "park": True})


def _remaining_prim(st, pose, s, dwell_left_s=None):
    """The rest of a step that is under way, from the robot's pose now."""
    p = step_to_prim(st)
    if st["type"] in ("drive", "approach"):
        L = math.dist(pose[:2], p["to"][:2])
        v = st["speed"] or s.cruise_cms
        return prim(st["type"], [pose[0], pose[1], p["from"][2]], [p["to"][0], p["to"][1], p["from"][2]], s,
                    speed=v, move_s=L / v, settle=s.drive_settle_s, reverse=st.get("reverse", False),
                    phase=st["phase"], board=st.get("board"), task=st.get("task"))
    if st["type"] in ("turn", "park"):
        return turn_prim(pose, p["to"][2], s, kind=st["type"], phase=st["phase"], board=st.get("board"),
                         task=st.get("task"))
    left = dwell_left_s if dwell_left_s is not None else (st["t_end"] - st["t_start"])
    return wait_prim(pose, _ticks(max(left, s.tick_s), s), s, phase=st["phase"], board=st.get("board"),
                     task=st.get("task"))


def states_at(plan, t):
    """Where every robot is at plan time t, as the planned schedule has it
    (used by the web page to try an obstacle at a chosen moment)."""
    out = {}
    for n, r in plan["robots"].items():
        steps = r["steps"]
        i = len(steps)
        prog = None
        for k, st in enumerate(steps):
            if t < st["t_end"]:
                i = k
                prog = t > st["t_start"]
                break
        dwell_left = None
        if i < len(steps) and steps[i]["type"] == "wait" and prog:
            dwell_left = steps[i]["t_end"] - t
        out[n] = {"pose": pose_at(plan, n, t), "step": i, "in_progress": bool(prog), "dwell_left_s": dwell_left}
    return out


def obstacle_event(m, plan, states, new_obstacles, over=None):
    """Something new is on the floor. Decide which robots stop at once and
    which carry on, then plan the rest of everyone's tasks.

    states: robot -> {"pose": [x, y, rad], "step": index of the step under
    way (or next), "in_progress": bool, "dwell_left_s": float or None}, all
    at the moment of the event (plan time of `plan`, t_event).

    Rules (Nick, 2026-10-17):
    - a robot whose motion over the next stop_horizon_s would bring its body
      within the safety margin of the new obstacle stops at once;
    - the others carry on with the steps they have until the first step end
      at or after replan_budget_s from now (the moment the new plan is
      ready), then hold; the new plan starts from those poses;
    - a carrying-on robot whose next steps would meet a stopped robot (or
      another one carrying on) holds earlier, or stops at once.

    Returns (new plan, events). New plan times count from the event."""
    m = FM.normalize(m)
    m2 = copy.deepcopy(m)
    for o in new_obstacles:
        m2["obstacles"].append(o)
    m2 = FM.normalize(m2)
    s = settings(over if over is not None else plan.get("settings"))
    body = FM.body_of(m2)
    w_new = FM.World({**m2, "boards": [], "keepouts": [], "obstacles": []},
                     extra=[FM._norm_obstacle(o, i) for i, o in enumerate(new_obstacles)])
    w_new.box = (-1e9, 1e9, -1e9, 1e9)    # only the new obstacles count here
    names = list(plan["robots"])
    budget_ticks = _ticks(s.replan_budget_s, s)
    events = []
    rest = {}
    for n in names:
        st = states[n]
        steps = plan["robots"][n]["steps"]
        i = st["step"]
        seq = []
        if i < len(steps):
            if st["in_progress"]:
                seq.append((steps[i], _remaining_prim(steps[i], st["pose"], s, st.get("dwell_left_s"))))
            else:
                seq.append((steps[i], _remaining_prim(steps[i], st["pose"], s)) if steps[i]["type"] != "wait"
                           else (steps[i], wait_prim(st["pose"], _ticks(steps[i]["t_end"] - steps[i]["t_start"], s), s,
                                                     phase=steps[i]["phase"], board=steps[i].get("board"),
                                                     task=steps[i].get("task"))))
            for stp in steps[i + 1:]:
                seq.append((stp, _with_dur(step_to_prim(stp), s)))
        rest[n] = seq

    def hits(p):
        if p["type"] in ("drive", "approach"):
            poly = body.drive_sweep(p["from"][:2], p["to"][:2], p["from"][2])
            return w_new.shape_clearance(poly) < s.safety_margin_cm
        if p["type"] in ("turn", "park"):
            return float(w_new.clearance(np.array(p["from"][:2])[None])[0]) < body.r_turn + s.safety_margin_cm
        return w_new.shape_clearance(body.corners([p["from"]])[0]) < s.safety_margin_cm

    # 1. who stops at once, and how far the others go
    keep = {}
    for n in names:
        seq = rest[n]
        if not seq:
            keep[n] = 0
            if plan["robots"][n].get("pending"):
                events.append({"robot": n, "action": "holds", "reason": "was waiting for a route; gets one now"})
            else:
                events.append({"robot": n, "action": "done", "reason": "already finished"})
            continue
        t, k, stop_reason = 0.0, 0, None
        horizon = s.stop_horizon_s
        # motion within the horizon (waits count as no time: the robot may move sooner)
        for stp, p in seq:
            if p["type"] != "wait" and hits(p):
                stop_reason = "the new obstacle is %.1f s ahead on its path (stop horizon %.1f s)" % (t, horizon)
                break
            if p["type"] != "wait":
                t += p["dur"] * s.tick_s
            if t >= horizon:
                break
        if stop_reason:
            keep[n] = 0
            events.append({"robot": n, "action": "stopped", "reason": stop_reason})
            continue
        # steps up to the first step end at or after the replan budget
        t, k = 0.0, 0
        while k < len(seq) and t < s.replan_budget_s:
            if hits(seq[k][1]):
                break
            t += seq[k][1]["dur"] * s.tick_s
            k += 1
        keep[n] = k
        if k == 0:
            events.append({"robot": n, "action": "holds", "reason": "was not moving; waits for the new plan"})
        else:
            events.append({"robot": n, "action": "continues",
                           "reason": "path clear for the next %.1f s; carries on %d step(s), then holds"
                           % (horizon, k)})
    # 2. carrying-on robots must not meet stopped robots or each other
    thr = s.separation_cm + _slack(body, s)
    for _ in range(10):
        sched = {}
        for n in names:
            seq = rest[n][:keep[n]]
            tk, sc = 0, []
            for stp, p in seq:
                sc.append((tk, p))
                tk += p["dur"]
            sched[n] = sc
        TL = {n: _timeline(sched[n], states[n]["pose"], s) for n in names}
        changed = False
        for a, b in itertools.combinations(names, 2):
            L = max(len(TL[a]), len(TL[b]))
            A = _others_at(TL[a], 0, L)
            B = _others_at(TL[b], 0, L)
            # the planner's own (cautious) test, so the new plan can start from here;
            # being too close already at the moment of the event is not counted
            hit = G.bodies_within(body, A, B, thr)
            hit[0] = False
            # both still exactly where they were at the event: nothing to fix by holding earlier
            at_start = np.all(np.isclose(A, A[0][None]), axis=1) & np.all(np.isclose(B, B[0][None]), axis=1)
            at_start = np.logical_and.accumulate(at_start)
            hit &= ~at_start
            bad = np.nonzero(hit)[0]
            if len(bad) == 0:
                continue
            tb = int(bad[0])
            # the robot that would move into the other gives way (stopped robots never move)
            cands = [r for r in (a, b) if keep[r] > 0]
            if not cands:
                continue
            def old_start(r):
                for tk, p in sched[r]:
                    if tk <= tb < tk + p["dur"]:
                        return tk
                return 10 ** 9
            r = max(cands, key=lambda r: (old_start(r), r))
            other = b if r == a else a
            # hold before the step that runs at tb
            k_new = 0
            for idx, (tk, p) in enumerate(sched[r]):
                if tk + p["dur"] <= tb:
                    k_new = idx + 1
            if k_new >= keep[r]:
                k_new = keep[r] - 1
            keep[r] = max(0, k_new)
            ev = next(e for e in events if e["robot"] == r)
            why = "stopped" if next(e for e in events if e["robot"] == other)["action"] == "stopped" else "holding"
            if keep[r] == 0 and states[r]["in_progress"]:
                ev.update(action="stopped", reason="would come within %.0f cm of %s, which is %s"
                          % (s.separation_cm, other, why))
            elif keep[r] == 0:
                ev.update(action="holds", reason="waits where it is, to keep clear of %s" % other)
            else:
                ev.update(action="continues", reason="carries on %d step(s), then holds, to keep clear of %s"
                          % (keep[r], other))
            changed = True
        if not changed:
            break
    # 3. new jobs: committed steps, then the rest of the tasks from the hold pose
    jobs = []
    for n in names:
        seq = rest[n]
        st = states[n]
        r_map = next(r for r in m2["robots"] if r["name"] == n)
        tasks = _tasks_of(m2, n)
        old = plan["robots"][n]
        frozen, tk = [], 0
        for stp, p in seq[:keep[n]]:
            frozen.append((tk, p))
            tk += p["dur"]
        hold = frozen[-1][1]["to"] if frozen else st["pose"]
        later = [stp for stp, _ in seq[keep[n]:] if stp["phase"] not in ("wait", "hold")]
        pend = old.get("pending")
        if not later and pend:
            # it had no route last time: its work is kept in "pending"
            lead = []
            if pend.get("lead_board"):
                d = FM.dock_for(m2, next(b for b in m2["boards"] if b["name"] == pend["lead_board"]))
                lead = _visit_tail(hold, pend["lead_board"], d, pend["task_offset"], s, pend["lead_phase"])
            jobs.append(Job(n, st["pose"], pend["visits"], pend["park"], FM.pose_in(r_map["park"]),
                            start_tick=budget_ticks, lead=lead, frozen=frozen, task_offset=pend["task_offset"],
                            any_order=tasks["any_order"]))
            continue
        if not later:
            jobs.append(Job(n, st["pose"], [], False, hold, start_tick=budget_ticks, frozen=frozen))
            continue
        nxt = later[0]
        task_i = nxt["task"] if nxt.get("task") is not None else old.get("task_offset", 0)
        seq_visits = [v for v in old["visit_order"] if v != "park"]
        off = old.get("task_offset", 0)
        lead = []
        if nxt.get("board") and nxt["phase"] in ("approach", "dwell", "backout"):
            d = FM.dock_for(m2, next(b for b in m2["boards"] if b["name"] == nxt["board"]))
            lead = _visit_tail(hold, nxt["board"], d, task_i, s, nxt["phase"])
            if keep[n] == 0 and nxt["phase"] == "dwell" and st.get("dwell_left_s") is not None and lead:
                lead[0]["dur"] = _ticks(max(st["dwell_left_s"], s.tick_s), s)
            task_i += 1
        remaining = seq_visits[task_i - off:]
        jobs.append(Job(n, st["pose"], remaining, tasks["park"], FM.pose_in(r_map["park"]),
                        start_tick=budget_ticks, lead=lead, frozen=frozen, task_offset=task_i,
                        any_order=tasks["any_order"]))
    over2 = dict(s)
    over2["skip_unreachable"] = True
    newplan = make_plan(m2, jobs=jobs, over=over2, replan_of=plan.get("plan_id"), events=events,
                        t_limit_s=0.45 * s.replan_budget_s, allow_strand=True)
    for n in newplan.get("stranded", []):
        ev = next(e for e in events if e["robot"] == n)
        ev["reason"] += "; no safe route yet, stays put until the next re-plan"
    return newplan, events


def _with_dur(p, s):
    p = dict(p)
    p["dur"] = max(1, int(round(p.pop("dur_s") / s.tick_s)))
    return p


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Plan every robot's route from map.json; writes plan.json.")
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", default="plan.json")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="change a planner setting, e.g. --set separation_cm=6 (any of DEFAULT_SETTINGS)")
    args = ap.parse_args()
    over = {}
    for kv in args.set:
        k, _, v = kv.partition("=")
        over[k.strip()] = v.strip()
    m = FM.load(args.map)
    for w in FM.validate(m):
        print("Map warning:", w)
    try:
        plan = make_plan(m, over={k: (v.lower() in ("1", "true", "yes")) if isinstance(DEFAULT_SETTINGS.get(k), bool)
                                  else v for k, v in over.items()})
    except PlanError as e:
        print("No plan:", e)
        return 1
    save_plan(plan, args.out)
    sm = plan["summary"]
    print("Plan saved to %s" % args.out)
    print("  last robot done after %.1f s; closest two robots come %.2f cm apart; planned in %.2f s"
          % (sm["makespan_s"], sm["min_separation_cm"], sm["planning_time_s"]))
    for n, r in plan["robots"].items():
        print("  %-7s %3d steps, done at %5.1f s: %s" % (n, len(r["steps"]), r["finish_s"],
                                                          " -> ".join(v if isinstance(v, str) else "point" for v in r["visit_order"])))
    for w in plan["warnings"]:
        print("  warning:", w)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
