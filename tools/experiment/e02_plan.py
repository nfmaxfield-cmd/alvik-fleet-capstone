"""E02 session plan: the block schedule, the board geometry (routes, park spots, safe area) and the
resumable state file. No hardware, no network: everything here is plain maths and files, so it is
unit-tested in tools/experiment/tests.

Conditions (docs/build-plans/accuracy-experiment-E02.md):
  U   uncalibrated: corrections at factory (`corr reset`), robot drives the route on its own sensors
  C   calibrated: the corrections learned in this session from the camera-measured U runs
  K   camera only: the laptop steers single `go` / `turn` legs from camera fixes; corrections at factory
  KC  calibrated sensors + camera fixes (`fix ...`, 2-5 a second) while the robot drives the route itself
So the four make a 2 x 2: calibration (no / yes) x camera in the loop (no / yes).

Schedule: block 0 is all U (every robot x every route once, shuffled), so every robot has U data to
learn from before its first C or KC run (the report must say the randomisation was restricted this way).
Every later block has each condition x route once (12 runs), shuffled; the robot of each run is
(route + condition + block) mod 3, so every block gives each robot each condition once, and every 3
blocks give each robot each condition x route once.

Board frame: the floor calibration's cm. The x-board is the blocked board along an x edge (a line
y = xboard_cm), the y-board the one along a y edge (x = yboard_cm); side 0 = at the low end of the other
axis, 1 = at the high end. They meet at the corner (yboard_cm, xboard_cm).
"""
import datetime as dt
import json
import math
import os
import random

CONDITIONS = ("U", "C", "K", "KC")
ROUTES = (1, 2, 3)
ROUTE_NAMES = {1: "straight 100 cm", 2: "L 30-30 cm", 3: "square 50 cm"}
ROUTE_SHORT = {1: "straight", 2: "L", 3: "square"}
COND_WORDS = {"U": "uncalibrated", "C": "calibrated", "K": "camera-steered", "KC": "calibrated + camera fixes"}

ROBOT_HALF_CM = 4.75          # half the Alvik's 9.5 cm length
FIND_BACKOFF_CM = 8.0         # firmware e02_find_back_cm: `run find` backs off this far from each board
FACE_CM = 4.75                # axle to the face that touches a board, no nose bracket (bracket: 6.10)
TURN_ROOM_CM = 3.75           # start spot: this much more than the face from the y-board, so the robot can turn
                              # there (a 9.5 cm square's corners sweep 6.7 cm round its centre)
PARK_EDGE_CM = 18.0           # park spots this far (centre) inside the open edge opposite the x-board
PARK_STAGE_CM = 30.0          # staging point this far before the park spot (towards the routes)
PARK_XS = (25.0, 61.0, 97.0)  # park spots along x, from the y-board, one per robot
SQUARE_CM = 50.0
STRAIGHT_CM = 100.0
L_CM = 30.0


# =============================================================================== schedule
def regular_block(robots, b, rng):
    """One balanced block: each condition x route once, robot (route + condition + b) mod n."""
    n = len(robots)
    runs = []
    for ci, c in enumerate(CONDITIONS):
        for ri, r in enumerate(ROUTES):
            runs.append(dict(cond=c, route=r, robot=robots[(ri + ci + b) % n]))
    rng.shuffle(runs)
    return runs


def warmup_block(robots, rng):
    runs = [dict(cond="U", route=r, robot=rb) for rb in robots for r in ROUTES]
    rng.shuffle(runs)
    return runs


def blocks_for(hours, run_s, n_robots=3):
    """How many regular blocks fit in `hours` after the warm-up block (at least 1)."""
    total = hours * 3600.0 / max(run_s, 1.0)
    return max(1, int(round((total - len(ROUTES) * n_robots) / (len(CONDITIONS) * len(ROUTES)))))


def make_schedule(robots, n_blocks, seed):
    """[dict(order, block, robot, cond, route, status='pending')]: block 0 = warm-up (all U), then
    n_blocks regular blocks."""
    rng = random.Random(seed)
    out = []
    for b in range(n_blocks + 1):
        runs = warmup_block(robots, rng) if b == 0 else regular_block(robots, b - 1, rng)
        for r in runs:
            r.update(order=len(out) + 1, block=b, status="pending")
            out.append(r)
    return out


def balance(schedule, blocks=None):
    """Counts per (cond), (cond, route), (cond, robot), (cond, route, robot): for tests and the report."""
    rows = [r for r in schedule if blocks is None or r["block"] in blocks]
    out = dict(cond={}, cond_route={}, cond_robot={}, cell={})
    for r in rows:
        for key, k in (("cond", r["cond"]), ("cond_route", (r["cond"], r["route"])),
                       ("cond_robot", (r["cond"], r["robot"])), ("cell", (r["cond"], r["route"], r["robot"]))):
            out[key][k] = out[key].get(k, 0) + 1
    return out


def next_run(schedule, has_fit, unavailable=()):
    """The next pending run to do, or None. A C or KC run for a robot with no accepted fit yet waits behind
    the other runs of its block: has_fit(robot) -> bool. If only such runs are left in the block, the first
    one runs on the priors (the caller records that). Robots in `unavailable` (flat battery, waiting for
    Nick) are skipped. Returns the run dict (from the schedule)."""
    pend = [r for r in schedule if r["status"] == "pending" and r["robot"] not in unavailable]
    if not pend:
        return None
    block = pend[0]["block"]
    in_block = [r for r in pend if r["block"] == block]
    for r in in_block:
        if r["cond"] in ("C", "KC") and not has_fit(r["robot"]):
            continue
        return r
    return in_block[0]


# =============================================================================== geometry
def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def rot(local, x0, y0, h_deg):
    c, s = math.cos(math.radians(h_deg)), math.sin(math.radians(h_deg))
    return [(x0 + c * u - s * v, y0 + s * u + c * v) for u, v in local]


class Board:
    """The board and its two blocked edges, in the floor calibration's cm."""

    def __init__(self, rect=(0.0, 0.0, 122.15, 121.9), xboard_side=0, yboard_side=0, xboard_cm=None, yboard_cm=None,
                 face_cm=FACE_CM):
        self.x0, self.y0, self.x1, self.y1 = (float(v) for v in rect)
        self.face_cm = float(face_cm)
        self.xboard_side, self.yboard_side = int(xboard_side), int(yboard_side)
        self.xboard_cm = float(xboard_cm) if xboard_cm is not None else (self.y0 if self.xboard_side == 0 else self.y1)
        self.yboard_cm = float(yboard_cm) if yboard_cm is not None else (self.x0 if self.yboard_side == 0 else self.x1)

    def to_dict(self):
        return dict(rect=[self.x0, self.y0, self.x1, self.y1], xboard_side=self.xboard_side,
                    yboard_side=self.yboard_side, xboard_cm=round(self.xboard_cm, 2), yboard_cm=round(self.yboard_cm, 2),
                    face_cm=self.face_cm)

    @classmethod
    def from_dict(cls, d):
        return cls(d["rect"], d["xboard_side"], d["yboard_side"], d.get("xboard_cm"), d.get("yboard_cm"),
                   d.get("face_cm", FACE_CM))

    def firmware_sides(self):
        """e02_xboard_side / e02_yboard_side as the firmware wants them: -1 = the board is at lower y / x than
        the robot, +1 = higher."""
        return (-1 if self.xboard_side == 0 else 1), (-1 if self.yboard_side == 0 else 1)

    # inward directions from the blocked boards
    @property
    def ux(self):
        return 1.0 if self.yboard_side == 0 else -1.0

    @property
    def uy(self):
        return 1.0 if self.xboard_side == 0 else -1.0

    @property
    def corner(self):
        return (self.yboard_cm, self.xboard_cm)

    @property
    def start_heading(self):
        """Nominal route heading: away from the y-board, along the x-board."""
        return 0.0 if self.ux > 0 else 180.0

    @property
    def turn_sign(self):
        """+1 = the route's turns go left (towards the board's middle), -1 = right."""
        return self.ux * self.uy

    @property
    def face_xboard(self):
        """Heading that faces the x-board (how a robot waits before `run find`)."""
        return -90.0 if self.uy > 0 else 90.0

    def nominal_start(self):
        """The route start spot (sent as e02_start_x / e02_start_y): as close to the y-board as still lets the
        robot turn (so the 100 cm straight keeps the most room before the open edge), and the find's back-off
        from the x-board. Heading: away from the y-board."""
        cx, cy = self.corner
        return (cx + self.ux * (self.face_cm + TURN_ROOM_CM), cy + self.uy * (self.face_cm + FIND_BACKOFF_CM),
                self.start_heading)

    def open_edges(self):
        """The two edges without a board: [('x', value) | ('y', value)] (x = value or y = value lines)."""
        return [("x", self.x1 if self.yboard_side == 0 else self.x0), ("y", self.y1 if self.xboard_side == 0 else self.y0)]

    def edge_room(self, x, y):
        """Distance (cm) from a point to the nearest open edge, and to the nearest board line."""
        open_d = min(abs(x - v) if a == "x" else abs(y - v) for a, v in self.open_edges())
        board_d = min(abs(y - self.xboard_cm), abs(x - self.yboard_cm))
        return open_d, board_d

    def on_board(self, x, y, margin=0.0):
        return (min(self.x0, self.x1) + margin <= x <= max(self.x0, self.x1) - margin and
                min(self.y0, self.y1) + margin <= y <= max(self.y0, self.y1) - margin)

    def park(self, i):
        """(x, y, heading) of park spot i: along the open edge opposite the x-board, facing the x-board."""
        y = (self.y1 - PARK_EDGE_CM) if self.uy > 0 else (self.y0 + PARK_EDGE_CM)
        x = self.yboard_cm + self.ux * PARK_XS[i % len(PARK_XS)]
        return (x, y, self.face_xboard)

    def stage(self, i):
        x, y, _ = self.park(i)
        return (x, y - self.uy * PARK_STAGE_CM)

    # routes ----------------------------------------------------------------------------------------
    def route_local(self, route):
        """(out, back) waypoints in the route's own frame (forward x, left y), from (0, 0). The far point is
        out[-1]; back ends at (0, 0). Turns go towards the board's middle (turn_sign)."""
        s = self.turn_sign
        if route == 1:
            return [(STRAIGHT_CM, 0.0)], [(0.0, 0.0)]
        if route == 2:
            return [(L_CM, 0.0), (L_CM, s * L_CM)], [(L_CM, 0.0), (0.0, 0.0)]
        if route == 3:
            a = SQUARE_CM
            return [(a, 0.0), (a, s * a)], [(0.0, s * a), (0.0, 0.0)]
        raise ValueError("route must be 1, 2 or 3")

    def route_floor(self, route, start_xy, heading=None):
        """Route waypoints on the floor from a start point (default heading: the nominal one)."""
        h = self.start_heading if heading is None else heading
        out, back = self.route_local(route)
        return rot(out, start_xy[0], start_xy[1], h), rot(back, start_xy[0], start_xy[1], h)

    def route_fits(self, route, start=None, guard_cm=3.0):
        """Does the route, from the start spot, keep the robot's front guard_cm inside the open edges and its
        body off the boards? Returns (ok, closest room cm: front to an open edge, or body to a board)."""
        sx, sy, sh = start or self.nominal_start()
        out, back = self.route_floor(route, (sx, sy), sh)
        pts = [(sx, sy)] + out + back
        open_room = min(self.edge_room(x, y)[0] for x, y in pts) - max(self.face_cm, ROBOT_HALF_CM)
        board_room = min(self.edge_room(x, y)[1] for x, y in pts) - ROBOT_HALF_CM
        worst = min(open_room, board_room)
        return worst >= guard_cm, worst


def dist_to_path(p, pts):
    """Shortest distance from point p to a polyline."""
    best = math.inf
    for a, b in zip(pts[:-1], pts[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
        best = min(best, math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy))
    if len(pts) == 1:
        best = math.hypot(p[0] - pts[0][0], p[1] - pts[0][1])
    return best


# =============================================================================== state file
class State:
    """Everything needed to carry on after a crash, in <session>/e02_state.json (written atomically after
    every change)."""

    VERSION = 1

    def __init__(self, folder, data=None):
        self.folder = folder
        self.path = os.path.join(folder, "e02_state.json")
        self.d = data or dict(version=self.VERSION, created=dt.datetime.now().isoformat(timespec="seconds"),
                              schedule=[], fits={}, fit_data={}, positions={}, board=None, setup={},
                              naming_check=None, launches=0, log_dirs=[], notes=[], durations=[])

    @classmethod
    def load(cls, folder):
        p = os.path.join(folder, "e02_state.json")
        with open(p, encoding="utf-8") as f:
            d = json.load(f)
        if d.get("version") != cls.VERSION:
            raise ValueError("%s was written by another version of e02.py" % p)
        return cls(folder, d)

    def save(self):
        os.makedirs(self.folder, exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.d, f, indent=1)
        os.replace(tmp, self.path)

    def __getitem__(self, k):
        return self.d[k]

    def __setitem__(self, k, v):
        self.d[k] = v

    def get(self, k, default=None):
        return self.d.get(k, default)

    def setdefault(self, k, default):
        return self.d.setdefault(k, default)

    # schedule helpers
    def counts(self):
        s = self.d["schedule"]
        done = sum(1 for r in s if r["status"] in ("done", "failed"))
        return done, len(s)

    def mean_run_s(self, default=110.0):
        d = [x for x in self.d.get("durations") or [] if x > 5]
        return sum(d[-20:]) / len(d[-20:]) if d else default

    def interrupted(self):
        """Runs that were started but never finished (the program stopped mid-run): back to pending, with
        a note, so they are redone."""
        out = []
        for r in self.d["schedule"]:
            if r["status"] == "running":
                r["status"] = "pending"
                r.setdefault("notes", []).append("interrupted (program stopped mid-run): redone")
                out.append(r)
        return out
