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
OPEN_EDGE_MIN_CM = 15.0       # every route keeps the robot's front this far from the open edges
FACE_CM = 4.75                # axle to the face that touches a board, no nose bracket (bracket: 6.10)
START_CLEAR_CM = 28.0         # route start spot: its centre this far from both blocked boards' faces (2026-10-21b; was
                              # 9 / 13 cm). An uncalibrated return ends ~15-25 cm off by design (gyro 7% short): from
                              # here it no longer runs into a board, so U runs are not cut short by the front stop;
                              # the diagonal straight's far point stays 22 cm (centre) from the open edges
FIND_STAGE_CM = 25.0          # firmware e02_find_stage_cm: the find starts from this far off both faces
# Park spots, in board words: (u, v) = cm in from the y-board's face and from the x-board's face (centre). In the
# two corners off the routes' diagonal (2026-10-21b; they were along the open edge, 20 cm from the end of the
# diagonal straight). Checked in tools/experiment/tests (the simulator's robot driving every route uncalibrated,
# gyro 5.5-8.5% short, drift +-0.3 deg/s): no route comes within PARK_CLEAR_CM of a parked robot's centre.
PARK_UV = ((103.0, 14.0), (12.0, 102.0), (105.0, 45.0))
ESCAPE_NEAR_CM = 14.0         # a robot's centre nearer an open edge than this goes straight in first (park spots are 17+)
ESCAPE_BOARD_CM = 25.0        # ... to a point at least this far from both board lines
PARK_STAGE_CM = 20.0          # staging point this far in front of the park spot; it backs the rest of the way in
PARK_CLEAR_CM = 18.0          # nothing a robot drives may come nearer a parked robot's centre than this (two bodies
                              # touch at 9.5-13.4 cm centre to centre)
WATCH_EDGE_CM = 9.0           # camera watchdog: stop a robot whose centre is (about to be, 0.3 s ahead) this close to an
                              # open edge (its front 4 cm short of it; the firmware edge guard stops it 3.5 cm from it)
WATCH_ROBOT_CM = 15.0         # ... or to a parked robot's centre, while moving towards it
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


RUN_S = 70.0          # measured (dry runs and the 2026-10-21 timing): one route run with its parking, s
FIND_S = 30.0         # plus its find (about 27 s measured), s
SETUP_S = 600.0       # setup checks, finding and parking the robots, the find check, the 2-minute naming check


def blocks_for_time(hours, run_s=RUN_S, find_s=FIND_S, setup_s=SETUP_S, n_robots=3, slack_s=300.0):
    """The most regular blocks (after the all-U block) whose whole session (setup + runs at run_s + find_s each)
    stays within hours (+ slack_s); at least 1."""
    per = run_s + find_s
    nb = 1
    while setup_s + (len(ROUTES) * n_robots + (nb + 1) * len(CONDITIONS) * len(ROUTES)) * per <= hours * 3600 + slack_s:
        nb += 1
    return nb


def plan_words(schedule, run_s=RUN_S, find_s=FIND_S, setup_s=SETUP_S):
    """Plain lines: runs per condition (and per route), and the time it should take."""
    per = run_s + find_s
    n = len(schedule)
    lines = ["Plan: %d runs (block 0: %d U runs to learn the corrections from, then %d blocks of 12)." % (
        n, sum(1 for r in schedule if r["block"] == 0), len({r["block"] for r in schedule}) - 1)]
    for c in CONDITIONS:
        rs = [r for r in schedule if r["cond"] == c]
        if rs:
            lines.append("  %-2s %-27s %3d runs (straight %d, L %d, square %d)" % (
                c, COND_WORDS[c], len(rs), *(sum(1 for r in rs if r["route"] == k) for k in ROUTES)))
    tot = setup_s + n * per
    lines.append("Time: about %d:%02d in all (setup about %d min, then %.0f s a run: %.0f s route and parking + %.0f s "
                 "find)." % (tot // 3600, (tot % 3600) // 60, setup_s // 60, per, run_s, find_s))
    return lines


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
    in_block = sorted([r for r in pend if r["block"] == block], key=lambda r: r.get("find_redos", 0))   # redos last
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

    def route_heading(self, route):
        """Heading the route starts along. The straight (100 cm) runs along the board's diagonal, away from
        the blocked corner (about 170 cm of room: the far point stays well off the open edges); the L and the
        square run along the x-board, away from the y-board, and turn towards the middle."""
        if route == 1:
            return math.degrees(math.atan2(self.uy, self.ux))
        return self.start_heading

    @property
    def turn_sign(self):
        """+1 = the route's turns go left (towards the board's middle), -1 = right."""
        return self.ux * self.uy

    @property
    def face_xboard(self):
        """Heading that faces the x-board (how a robot waits before `run find`)."""
        return -90.0 if self.uy > 0 else 90.0

    def uv(self, u, v):
        """Floor point u cm in from the y-board's face and v cm in from the x-board's face."""
        return (self.yboard_cm + self.ux * u, self.xboard_cm + self.uy * v)

    def nominal_start(self):
        """The route start spot (sent as e02_start_x / e02_start_y): START_CLEAR_CM from both boards (an
        uncalibrated return does not reach them; the 100 cm diagonal straight still ends 20+ cm off the open
        edges). Heading: away from the y-board."""
        x, y = self.uv(START_CLEAR_CM, START_CLEAR_CM)
        return (x, y, self.start_heading)

    def find_stage(self):
        """Where `run find` starts from (firmware e02_find_stage_cm from both faces; the axle)."""
        return self.uv(FIND_STAGE_CM, FIND_STAGE_CM)

    def open_edges(self):
        """The two edges without a board: [('x', value) | ('y', value)] (x = value or y = value lines)."""
        return [("x", self.x1 if self.yboard_side == 0 else self.x0), ("y", self.y1 if self.xboard_side == 0 else self.y0)]

    def edge_room(self, x, y):
        """Distance (cm) from a point to the nearest open edge, and to the nearest board line."""
        open_d = min(abs(x - v) if a == "x" else abs(y - v) for a, v in self.open_edges())
        board_d = min(abs(y - self.xboard_cm), abs(x - self.yboard_cm))
        return open_d, board_d

    def escape_point(self, x, y, near_cm=ESCAPE_NEAR_CM, to_cm=28.0, board_room_cm=ESCAPE_BOARD_CM):
        """A point straight in from the nearest open edge (to_cm from it) when (x, y) is within near_cm of it, else
        None: where a robot left near an edge goes first, so the way to its spot never runs along the edge.
        2026-10-22: near_cm was 20, and every park spot is 17-20 cm from an open edge, so every robot leaving its
        spot was "moved in first"; and in the corner by a board the point came out 1.4 cm from the board's line
        (Left drove into the board and looped). Now the point also keeps board_room_cm from both board lines."""
        best = None
        for a, v in self.open_edges():
            d = abs((x if a == "x" else y) - v)
            if d < near_cm and (best is None or d < best[0]):
                best = (d, a, v)
        if best is None:
            return None
        d, a, v = best
        lo = min(self.x0, self.x1) if a == "x" else min(self.y0, self.y1)
        inward = 1.0 if abs(v - lo) < 1e-6 else -1.0
        q = [v + inward * to_cm, y] if a == "x" else [x, v + inward * to_cm]
        # ... and from the other open edge too (the corner where the two open edges meet; review 2026-10-22)
        for a2, v2 in self.open_edges():
            if a2 == a:
                continue
            k = 0 if a2 == "x" else 1
            lo2 = min(self.x0, self.x1) if a2 == "x" else min(self.y0, self.y1)
            in2 = 1.0 if abs(v2 - lo2) < 1e-6 else -1.0
            if (q[k] - v2) * in2 < to_cm:
                q[k] = v2 + in2 * to_cm
        # keep clear of the board lines (the other coordinate, along the edge)
        if a == "x":
            if abs(q[1] - self.xboard_cm) < board_room_cm:
                q[1] = self.xboard_cm + (board_room_cm if self.uy > 0 else -board_room_cm)
        else:
            if abs(q[0] - self.yboard_cm) < board_room_cm:
                q[0] = self.yboard_cm + (board_room_cm if self.ux > 0 else -board_room_cm)
        return tuple(q)

    def on_board(self, x, y, margin=0.0):
        return (min(self.x0, self.x1) + margin <= x <= max(self.x0, self.x1) - margin and
                min(self.y0, self.y1) + margin <= y <= max(self.y0, self.y1) - margin)

    def park(self, i):
        """(x, y, heading) of park spot i (PARK_UV), facing the find's staging point (its next drive)."""
        u, v = PARK_UV[i % len(PARK_UV)]
        x, y = self.uv(u, v)
        sx, sy = self.find_stage()
        return (x, y, math.degrees(math.atan2(sy - y, sx - x)))

    def stage(self, i):
        """The point PARK_STAGE_CM in front of park spot i: the robot gets there, turns to the spot's heading and
        backs into the spot (so it faces the open board, never an edge, when it next drives forward)."""
        x, y, h = self.park(i)
        return (x + PARK_STAGE_CM * math.cos(math.radians(h)), y + PARK_STAGE_CM * math.sin(math.radians(h)))

    def park_words(self, i, side_of):
        """Plain words for where park spot i is: side_of(x, y) -> 'top' / 'left' ... of the camera picture."""
        u, v = PARK_UV[i % len(PARK_UV)]
        # name each board by a point well along it, not near the shared corner (near the corner both read
        # "top"; 2026-10-21)
        ex, ey = self.uv(0.0, 60.0), self.uv(60.0, 0.0)
        return ("about %.0f cm (%.0f in) from the blocked board on the %s of the camera picture and %.0f cm (%.0f in) "
                "from the blocked board on the %s" % (u, u / 2.54, side_of(*ex), v, v / 2.54, side_of(*ey)))

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
        """Route waypoints on the floor from a start point (default heading: route_heading)."""
        h = self.route_heading(route) if heading is None else heading
        out, back = self.route_local(route)
        return rot(out, start_xy[0], start_xy[1], h), rot(back, start_xy[0], start_xy[1], h)

    def route_fits(self, route, start=None, open_cm=OPEN_EDGE_MIN_CM, board_cm=2.0):
        """Does the route, from the start spot, keep the robot's front open_cm from the open edges and its body
        board_cm off the boards? Returns (ok, front-to-open-edge room cm, body-to-board room cm)."""
        sx, sy, _ = start or self.nominal_start()
        out, back = self.route_floor(route, (sx, sy))
        pts = [(sx, sy)] + out + back
        open_room = min(self.edge_room(x, y)[0] for x, y in pts) - max(self.face_cm, ROBOT_HALF_CM)
        board_room = min(self.edge_room(x, y)[1] for x, y in pts) - ROBOT_HALF_CM * math.sqrt(2)
        return open_room >= open_cm and board_room >= board_cm, open_room, board_room


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
