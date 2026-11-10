#!/usr/bin/env python3
"""The PROVEN WORKSPACE (e10, Phase C2, 7 Nov; Nick: "The system should only operate within areas that it's proven to
be workspace ... limit the workspace to what's visible ... or use cliff detection and odometry corrections to send the
robot ahead to map areas the camera can't see well").

A grid (CELL_CM) over the area the board might cover (the typed board, a starting guess, grown by GROW_CM each side).
Each cell is one of:
  UNKNOWN   nothing shows it is table
  SEEN      the camera shows table there: inside the board outline the camera's own setup gives (the corner tiles at
            school, the corner clicks / typed board at home, checked by the start's discovery), where the camera can
            actually see (its coverage; not a dark patch)
  FELT      a robot's body went over it with its edge sensors reading table (its line sensors and the look-ahead bottom
            zone), placed by its pose (camera, or odometry where the camera cannot see: its uncertainty is taken off
            the felt strip's sides)
  DROP      an edge sensor felt a drop there (or beyond it): never proven again in this run, whatever else says
PROVEN = SEEN or FELT, and not DROP. The planner keeps every robot's derived edge clearance from the nearest cell that
is not proven, exactly as it kept it from the board's edges before (freeroam_core.edge_dist). The map grows during the
run (felt strips, probes ahead) and is saved per site; a saved map is a starting guess (observe, then trust): its FELT
cells are used again only when this run's board outline agrees with the one it was made on (BOARD_AGREE_CM).

The switch (keep features, switch them off): freeroam_core.Controller(workspace=None) keeps the plain board rectangle,
as before e10.
"""
import json
import math
import os

import numpy as np

VERSION = "proven-workspace-2026-11-10e13"   # e13 (10 Nov): limit_to() a readable area (school free roam plans
                                            # only where the camera reads tags); field(raw=True) ignores the limit
VERSION_A = "proven-workspace-2026-11-07a"

CELL_CM = 2.5           # CHOSEN: a quarter of a body; the planner's edge distances are then within ~1.8 cm (conservative)
GROW_CM = 20.0          # CHOSEN: the area searched past the typed board on each side (a board bigger than typed can be
                        # proven there by feel); DOE: no
BOARD_AGREE_CM = 3.0    # CHOSEN (as feel_table.AGREE_CM + 1): a saved map's felt cells are used again only on a board
                        # outline within this of the one it was made on
UNKNOWN, SEEN, FELT, DROP, PROBE = 0, 1, 2, 3, 4
WORDS = {UNKNOWN: "unknown", SEEN: "seen by the camera", FELT: "felt by a robot", DROP: "a drop",
         PROBE: "being probed (a robot's own edge sensors guard it)"}


class ProvenWorkspace(tuple):
    """A tuple (x0, y0, x1, y1): the outer limit (the typed board grown by GROW_CM), so code that reads a board rectangle
    still works; plus the grid. Use edge_dist() / dist_v() for the distance to the nearest cell that is not proven."""

    def __new__(cls, board, grow_cm=GROW_CM, cell_cm=CELL_CM):
        b = [float(v) for v in board]
        outer = (b[0] - grow_cm, b[1] - grow_cm, b[2] + grow_cm, b[3] + grow_cm)
        return super().__new__(cls, outer)

    def __init__(self, board, grow_cm=GROW_CM, cell_cm=CELL_CM):
        self.board = tuple(float(v) for v in board)      # the outline the camera's setup gives (a starting guess)
        self.cell = float(cell_cm)
        self.nx = int(math.ceil((self[2] - self[0]) / self.cell))
        self.ny = int(math.ceil((self[3] - self[1]) / self.cell))
        self.g = np.zeros((self.nx, self.ny), dtype=np.int8)
        self.felt_n = np.zeros((self.nx, self.ny), dtype=np.int16)
        self.version = 0                 # bumped on every change (the controller re-reads the distance field)
        self._field = None
        self._field_v = -1
        self.events = []                 # (what, detail) for the run's log
        self.probe_cells = 0
        self.allow = None                # (e13) None: no limit; else a bool grid: the cells the camera has proven
                                         # READABLE (school: roof tags read there). Proven ground outside it is real
                                         # table, kept for turning and returning, but never planned into
        self._rfield, self._rfield_v = None, -1

    # -- the grid -------------------------------------------------------------------------------------------------
    def ij(self, x, y):
        return int((x - self[0]) // self.cell), int((y - self[1]) // self.cell)

    def centre(self, i, j):
        return self[0] + (i + 0.5) * self.cell, self[1] + (j + 0.5) * self.cell

    def _mask_rect(self, r, whole=False):
        """Cells whose centres lie inside rectangle r (whole: cells wholly inside it: never more ground than there is)."""
        xs = self[0] + (np.arange(self.nx) + 0.5) * self.cell
        ys = self[1] + (np.arange(self.ny) + 0.5) * self.cell
        h = 0.5 * self.cell if whole else 0.0
        mx = (xs - h >= r[0] - 1e-9) & (xs + h <= r[2] + 1e-9)
        my = (ys - h >= r[1] - 1e-9) & (ys + h <= r[3] + 1e-9)
        return np.outer(mx, my)

    def proven_mask(self, raw=False):
        m = (self.g == SEEN) | (self.g == FELT) | (self.g == PROBE)
        if self.allow is not None and not raw:
            m = m & self.allow
        return m

    # -- (e13) the readable area ------------------------------------------------------------------------------------
    def limit_to(self, readable_fn, why="readable"):
        """Plan only where readable_fn(x, y) is True for the cell's centre (the readability map: the camera reads roof
        tags there). Returns the readable proven area in cm2. Ground outside stays SEEN / FELT (it is table) and counts
        in field(raw=True), which the stray return uses to turn and to come back."""
        a = np.zeros((self.nx, self.ny), dtype=bool)
        for i in range(self.nx):
            for j in range(self.ny):
                try:
                    a[i, j] = bool(readable_fn(*self.centre(i, j)))
                except Exception:      # noqa: BLE001
                    a[i, j] = False
        self.allow = a
        self.events.append(("limited", "planning limited to the %s area: %d cm2 of proven ground" % (
            why, round(float(self.proven_mask().sum()) * self.cell * self.cell))))
        self._changed()
        return round(float(self.proven_mask().sum()) * self.cell * self.cell)

    def unlimit(self):
        self.allow = None
        self._changed()

    @property
    def limited(self):
        return self.allow is not None

    def allowed(self, x, y):
        if self.allow is None:
            return True
        i, j = self.ij(x, y)
        return 0 <= i < self.nx and 0 <= j < self.ny and bool(self.allow[i, j])

    def nearest_allowed(self, x, y, min_room_cm=0.0):
        """The centre of the nearest cell that is proven AND readable with at least min_room_cm to its limit (None if
        none)."""
        f = self.field()
        ok = f >= min_room_cm
        if not ok.any():
            ok = f > 0
            if not ok.any():
                return None
        i, j = np.nonzero(ok)
        cx = self[0] + (i + 0.5) * self.cell
        cy = self[1] + (j + 0.5) * self.cell
        k = int(np.argmin((cx - x) ** 2 + (cy - y) ** 2))
        return float(cx[k]), float(cy[k])

    def provisional(self, x0, y0, x1, y1, radius):
        """A probe's corridor: UNKNOWN cells within radius of the segment become PROBE (counted as proven while the
        probe runs: the robot's own bottom zone and line sensors guard it, slowly). Returns the cells, to clear."""
        L = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(L / self.cell))
        cells = set()
        for k in range(n + 1):
            cx, cy = x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n
            i0, j0 = self.ij(cx - radius, cy - radius)
            i1, j1 = self.ij(cx + radius, cy + radius)
            for i in range(max(0, i0), min(self.nx, i1 + 1)):
                for j in range(max(0, j0), min(self.ny, j1 + 1)):
                    if self.g[i, j] == UNKNOWN:
                        px, py = self.centre(i, j)
                        if math.hypot(px - cx, py - cy) <= radius:
                            cells.add((i, j))
        for i, j in cells:
            self.g[i, j] = PROBE
        if cells:
            self._changed()
        return list(cells)

    def clear_provisional(self, cells):
        k = 0
        for i, j in cells:
            if self.g[i, j] == PROBE:
                self.g[i, j] = UNKNOWN
                k += 1
        if k:
            self._changed()
        return k

    def state(self, x, y):
        i, j = self.ij(x, y)
        if 0 <= i < self.nx and 0 <= j < self.ny:
            return int(self.g[i, j])
        return UNKNOWN

    def proven(self, x, y):
        return self.state(x, y) in (SEEN, FELT)

    def _changed(self):
        self.version += 1

    # -- evidence ------------------------------------------------------------------------------------------------
    def see(self, board=None, cover=None, blind=()):
        """The camera's view: cells inside the board outline, inside the camera's coverage rectangles (None: all), and
        not in a blind patch (dark, glare the camera cannot read) become SEEN (DROP stays DROP)."""
        b = self.board if board is None else tuple(board)
        m = self._mask_rect(b, whole=True)
        if cover is not None:
            c = np.zeros_like(m)
            for r in cover:
                c |= self._mask_rect(r, whole=True)
            m &= c
        for r in blind or ():
            m &= ~self._mask_rect(r)
        new = m & (self.g == UNKNOWN)
        self.g[new] = SEEN
        if new.any():
            self._changed()
        return int(new.sum())

    def unsee(self, rects):
        """The camera can no longer vouch for these (a patch found dark during the run): SEEN back to UNKNOWN (felt
        cells stay)."""
        m = np.zeros((self.nx, self.ny), dtype=bool)
        for r in rects:
            m |= self._mask_rect(r)
        k = m & (self.g == SEEN)
        self.g[k] = UNKNOWN
        if k.any():
            self._changed()
        return int(k.sum())

    def felt_strip(self, x0, y0, x1, y1, half_w=3.0, unc=0.0):
        """A robot's body went from (x0, y0) to (x1, y1) with its edge sensors reading table: the strip under its wheels
        (half_w each side of its middle line), narrowed by its position uncertainty, is FELT. Returns cells newly
        proven."""
        w = half_w - max(0.0, unc)
        if w <= 0.0:
            return 0
        L = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(L / (self.cell * 0.5)))
        new = 0
        for k in range(n + 1):
            f = k / n
            cx, cy = x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
            i0, j0 = self.ij(cx - w, cy - w)
            i1, j1 = self.ij(cx + w, cy + w)
            for i in range(max(0, i0), min(self.nx, i1 + 1)):
                for j in range(max(0, j0), min(self.ny, j1 + 1)):
                    px, py = self.centre(i, j)
                    if math.hypot(px - cx, py - cy) > w + 0.5 * self.cell:
                        continue
                    if self.g[i, j] in (UNKNOWN, PROBE):
                        self.g[i, j] = FELT
                        new += 1
                    if self.g[i, j] != DROP:
                        self.felt_n[i, j] = min(32000, self.felt_n[i, j] + 1)
        if new:
            self._changed()
        return new

    def drop_at(self, x, y, heading_deg, reach_cm=0.0, width_cm=12.0, depth_cm=30.0):
        """An edge sensor felt a drop: the drop line lies reach_cm ahead of (x, y) along heading; cells from that line on,
        width_cm wide and depth_cm deep, are DROP (never proven again this run)."""
        h = math.radians(heading_deg)
        ux, uy = math.cos(h), math.sin(h)
        lx, ly = x + reach_cm * ux, y + reach_cm * uy
        k = 0
        for i in range(self.nx):
            for j in range(self.ny):
                px, py = self.centre(i, j)
                a = (px - lx) * ux + (py - ly) * uy
                b = -(px - lx) * uy + (py - ly) * ux
                if -0.5 * self.cell <= a <= depth_cm and abs(b) <= width_cm / 2.0:
                    if self.g[i, j] != DROP:
                        self.g[i, j] = DROP
                        k += 1
        if k:
            self._changed()
            self.events.append(("drop", dict(x=round(lx, 1), y=round(ly, 1), heading=round(heading_deg), cells=k)))
        return k

    def cut_side(self, side, at):
        """A wall or edge found on one of the board's sides (side 0 x_min, 1 y_min, 2 x_max, 3 y_max) at coordinate
        `at`: every cell past it is DROP."""
        xs = self[0] + (np.arange(self.nx) + 0.5) * self.cell
        ys = self[1] + (np.arange(self.ny) + 0.5) * self.cell
        if side == 0:
            m = np.outer(xs < at, np.ones(self.ny, bool))
        elif side == 2:
            m = np.outer(xs > at, np.ones(self.ny, bool))
        elif side == 1:
            m = np.outer(np.ones(self.nx, bool), ys < at)
        else:
            m = np.outer(np.ones(self.nx, bool), ys > at)
        k = m & (self.g != DROP)
        self.g[k] = DROP
        if k.any():
            self._changed()
        return int(k.sum())

    # -- the planner's distance ----------------------------------------------------------------------------------
    def field(self, raw=False):
        """Per cell: the distance (cm) from its centre to the nearest cell that is not proven (minus half a cell: to
        that cell's near side, roughly), 0 or less outside. raw (e13): ignoring the readable limit (the table itself)."""
        if raw:
            if self.allow is None:
                return self.field()
            if self._rfield is not None and self._rfield_v == self.version:
                return self._rfield
        elif self._field is not None and self._field_v == self.version:
            return self._field
        from scipy import ndimage
        pm = self.proven_mask(raw=raw)
        pad = np.zeros((self.nx + 2, self.ny + 2), dtype=bool)
        pad[1:-1, 1:-1] = pm
        d = ndimage.distance_transform_edt(pad, sampling=self.cell)[1:-1, 1:-1]
        f = np.where(pm, d - 0.5 * self.cell, -self.cell)
        if raw:
            self._rfield, self._rfield_v = f.astype(np.float64), self.version
            return self._rfield
        self._field, self._field_v = f.astype(np.float64), self.version
        return self._field

    def edge_dist(self, x, y, raw=False):
        """Distance (cm) from (x, y) to the nearest unproven ground (negative outside the proven area). Bilinear between
        cell centres, so it moves smoothly with the robot. raw (e13): to the table's own unproven ground, ignoring the
        readable limit."""
        f = self.field(raw=raw)
        fx = (x - self[0]) / self.cell - 0.5
        fy = (y - self[1]) / self.cell - 0.5
        i0, j0 = int(math.floor(fx)), int(math.floor(fy))
        if i0 < 0 or j0 < 0 or i0 + 1 >= self.nx or j0 + 1 >= self.ny:
            return -self.cell
        ax, ay = fx - i0, fy - j0
        v = (f[i0, j0] * (1 - ax) * (1 - ay) + f[i0 + 1, j0] * ax * (1 - ay) + f[i0, j0 + 1] * (1 - ax) * ay +
             f[i0 + 1, j0 + 1] * ax * ay)
        return float(v)

    def dist_v(self, xs, ys):
        f = self.field()
        fx = (np.asarray(xs, float) - self[0]) / self.cell - 0.5
        fy = (np.asarray(ys, float) - self[1]) / self.cell - 0.5
        i0 = np.floor(fx).astype(int)
        j0 = np.floor(fy).astype(int)
        bad = (i0 < 0) | (j0 < 0) | (i0 + 1 >= self.nx) | (j0 + 1 >= self.ny)
        i0c = np.clip(i0, 0, self.nx - 2)
        j0c = np.clip(j0, 0, self.ny - 2)
        ax, ay = np.clip(fx - i0c, 0, 1), np.clip(fy - j0c, 0, 1)
        v = (f[i0c, j0c] * (1 - ax) * (1 - ay) + f[i0c + 1, j0c] * ax * (1 - ay) + f[i0c, j0c + 1] * (1 - ax) * ay +
             f[i0c + 1, j0c + 1] * ax * ay)
        return np.where(bad, -self.cell, v)

    def middle(self):
        """The proven area's middle (where an escape heads), or the board's."""
        pm = self.proven_mask()
        if not pm.any():
            return ((self.board[0] + self.board[2]) / 2, (self.board[1] + self.board[3]) / 2)
        i, j = np.nonzero(pm)
        f = self.field()
        k = int(np.argmax(f[i, j]))
        return self.centre(int(i[k]), int(j[k]))

    # -- probing ahead ------------------------------------------------------------------------------------------
    def frontier(self, x, y, max_cm=80.0, min_room_cm=10.0):
        """Unknown cells next to proven ones, within max_cm of (x, y), whose proven neighbour has min_room_cm of proven
        ground round it (a probe starts from there). Returns [(d, ux, uy, sx, sy)]: distance, the unknown cell's centre,
        and the proven start spot."""
        pm = self.proven_mask()
        un = self.g == UNKNOWN
        pm = (self.g == SEEN) | (self.g == FELT)
        f = self.field()
        out = []
        i0, j0 = self.ij(x - max_cm, y - max_cm)
        i1, j1 = self.ij(x + max_cm, y + max_cm)
        for i in range(max(1, i0), min(self.nx - 1, i1 + 1)):
            for j in range(max(1, j0), min(self.ny - 1, j1 + 1)):
                if not un[i, j]:
                    continue
                for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    a, b = i + di, j + dj
                    if pm[a, b]:
                        ux, uy = self.centre(i, j)
                        d = math.hypot(ux - x, uy - y)
                        if d <= max_cm:
                            out.append((d, ux, uy, di, dj))
                        break
        out.sort()
        return out

    # -- summary, saving --------------------------------------------------------------------------------------------
    def counts(self):
        a = self.cell * self.cell
        return dict(seen_cm2=round(float((self.g == SEEN).sum()) * a), felt_cm2=round(float((self.g == FELT).sum()) * a),
                    drop_cm2=round(float((self.g == DROP).sum()) * a),
                    proven_cm2=round(float(self.proven_mask().sum()) * a),
                    readable_limit=self.allow is not None)

    def summary(self):
        b = self.board
        bm = self._mask_rect(b)
        pm = self.proven_mask()
        return dict(version=VERSION, board=[round(v, 1) for v in b], cell_cm=self.cell, **self.counts(),
                    board_proven_pct=round(100.0 * float((pm & bm).sum()) / max(1, int(bm.sum())), 1),
                    outside_board_proven_cm2=round(float((pm & ~bm).sum()) * self.cell * self.cell),
                    probe_cells=self.probe_cells, events=self.events[-20:])

    def to_json(self):
        g = self.g.ravel().tolist()
        runs, cur, n = [], None, 0          # run-length encoding
        for v in g:
            if v == cur:
                n += 1
            else:
                if cur is not None:
                    runs.append([cur, n])
                cur, n = v, 1
        runs.append([cur, n])
        return dict(version=VERSION, board=list(self.board), outer=list(self), cell=self.cell, nx=self.nx, ny=self.ny,
                    rle=runs)

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.to_json(), f)
        os.replace(tmp, path)

    def merge_saved(self, path):
        """A saved map of this site: its FELT cells (and drops) are used again when its board outline agrees with this
        run's within BOARD_AGREE_CM (observe, then trust: otherwise the table or the camera moved, and it is ignored).
        Returns what was done, in words."""
        try:
            with open(path) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return "no saved workspace for this site"
        if max(abs(a - b) for a, b in zip(d.get("board", []), self.board)) > BOARD_AGREE_CM or \
                abs(float(d.get("cell", 0)) - self.cell) > 1e-6 or list(d.get("outer", [])) != [float(v) for v in self]:
            return "the saved workspace was made on another board outline: not used"
        flat = []
        for v, n in d["rle"]:
            flat.extend([v] * n)
        g = np.array(flat, dtype=np.int8).reshape(d["nx"], d["ny"])
        k = (g == FELT) & (self.g == UNKNOWN)
        self.g[k] = FELT
        kd = (g == DROP) & (self.g != DROP)
        self.g[kd] = DROP
        if k.any() or kd.any():
            self._changed()
        return "the saved workspace added %d felt cells and %d drop cells" % (int(k.sum()), int(kd.sum()))


def site_path(conf_dir, site="home"):
    return os.path.join(os.path.expanduser(conf_dir), "workspace", "%s.json" % site)
