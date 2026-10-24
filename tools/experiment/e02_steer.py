"""E02 steering maths: camera-steered legs (condition K, parking, putting a lost robot back), heading
from the camera track, and the lost / stuck / off-the-board decisions. Pure functions and small
classes, no network: unit-tested in tools/experiment/tests.

How a camera-steered drive works (K): the laptop knows where the robot is (camera) but its heading only
from how it moved. So:
  - after a `go` leg of at least MIN_HEADING_CM, the heading is the direction the camera saw it move
    (reversed for a backwards leg): measured, good to about a degree;
  - after a `turn` leg, the heading is the old one plus the turn asked for: the robot's own gyro did the
    turn, so it may be off by a few degrees (more when uncalibrated). The next `go` is kept short
    (SHORT_LEG_CM), then the heading is measured again and corrected;
  - legs are at most MAX_LEG_CM, so a heading error is caught before it grows.
"""
import math

import numpy as np

MIN_HEADING_CM = 3.0     # a go leg at least this long gives a measured heading
MAX_LEG_CM = 25.0        # longest single go leg
SHORT_LEG_CM = 10.0      # longest go leg right after a turn (heading not yet measured)
AIM_TOL_DEG = 2.0        # turn first when the target's bearing is further off than this...
AIM_TOL_NEAR_CM = 0.6    # ...or when the miss sideways at the target would exceed this
ARRIVE_CM = 1.5          # close enough to a waypoint
NEAR_CM = 3.0            # this close, only roll along (turning round for a sideways centimetre makes it worse)
SMALL_TURN_DEG = 15.0    # a turn up to this keeps the heading "measured" (the gyro's few % of it is < 1 deg)
REVERSE_CM = 5.0         # a target behind the robot and this close: back up instead of turning round
PROBE_CM = 4.0           # forward roll that measures the heading before the final turn
MIN_TURN_DEG = 1.0       # turns smaller than this are not sent (the robot can't do them well)
RECOVER_CM = 40.0        # camera end this far from its start spot: too far off to trust the drive back (needs recovery)
LOST_CM = RECOVER_CM     # old name (2026-10-21a flagged runs "lost" at 15 cm: most U runs, by design)
STUCK_FRACTION = 0.3     # a leg that moved less than this share of what was asked: stuck


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def bearing(p, q):
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))


def plan_leg(pos, heading, target, heading_measured=True, arrive_cm=ARRIVE_CM, max_leg=MAX_LEG_CM):
    """The next leg towards target from pos with the given heading (deg, None = unknown).
    Returns ('done', 0) | ('turn', deg, left +) | ('go', cm, + forward) | ('probe', cm)."""
    dx, dy = target[0] - pos[0], target[1] - pos[1]
    d = math.hypot(dx, dy)
    if d <= arrive_cm:
        return ("done", 0.0)
    if heading is None:
        return ("probe", MIN_HEADING_CM + 1.0)
    err = wrap180(bearing(pos, target) - heading)
    along = d * math.cos(math.radians(err))
    side = d * math.sin(math.radians(err))
    if d <= NEAR_CM:
        # nearly there: roll along if that helps, else stop (a sideways centimetre can't be fixed by turning)
        if abs(along) > arrive_cm * 0.5 and abs(side) <= NEAR_CM:
            return ("go", round(along, 2))
        return ("done", 0.0)
    if d <= REVERSE_CM and abs(side) <= arrive_cm:
        # close, and nearly in line (ahead or behind): just roll along
        return ("go", round(along, 2))
    if d <= REVERSE_CM and abs(err) > 90:
        # close behind and off to the side: turn so the back faces it, then reverse
        back_err = wrap180(err - 180.0)
        if abs(back_err) >= MIN_TURN_DEG:
            return ("turn", round(back_err, 2))
        return ("go", round(along, 2))
    tol = max(MIN_TURN_DEG, min(AIM_TOL_DEG, math.degrees(math.atan2(AIM_TOL_NEAR_CM, max(d, 1e-6)))))
    if abs(err) > tol:
        return ("turn", round(err, 2))
    limit = max_leg if heading_measured else min(max_leg, SHORT_LEG_CM)
    step = d if d <= limit + 5.0 else limit
    return ("go", round(step, 2))


class Steer:
    """Drives a robot through waypoints by single legs. Feed it the camera position before each leg and
    what the camera saw during the last leg; it says what to send next.

        s = Steer(waypoints, heading=..., measured=True)
        while True:
            cmd = s.next(pos)              # ('go', cm) / ('turn', deg) / ('done', 0) / ('probe', cm)
            ... send it, wait for the leg to end ...
            s.after(cmd, start_pos, end_pos)
    """

    def __init__(self, waypoints, heading=None, measured=False, arrive_cm=ARRIVE_CM, max_leg=MAX_LEG_CM,
                 final_heading=None, heading_tol=3.0, max_legs=60):
        self.wp = list(waypoints)
        self.i = 0
        self.heading = heading
        self.measured = measured
        self.arrive = arrive_cm
        self.max_leg = max_leg
        self.final_heading = final_heading
        self.heading_tol = heading_tol
        self.legs = 0
        self.max_legs = max_legs
        self.reached = []          # (waypoint index, position) when each waypoint was reached
        self.probes = 0            # final heading: short forward probes made to measure it
        self.final_turns = 0

    @property
    def waypoint(self):
        return self.wp[self.i] if self.i < len(self.wp) else None

    def next(self, pos):
        if self.legs >= self.max_legs:
            return ("gave_up", 0.0)
        while self.i < len(self.wp):
            last = self.i == len(self.wp) - 1
            cmd = plan_leg(pos, self.heading, self.wp[self.i], self.measured,
                           arrive_cm=self.arrive if last else max(self.arrive, 2.0), max_leg=self.max_leg)
            if cmd[0] != "done":
                return cmd
            self.reached.append((self.i, tuple(pos)))
            self.i += 1
        if self.wp and self.reached:
            # back onto the last waypoint if a probe (or a nudge) moved it off
            cmd = plan_leg(pos, self.heading, self.wp[-1], self.measured, arrive_cm=self.arrive, max_leg=self.max_leg)
            if cmd[0] != "done":
                return cmd
        if self.final_heading is not None and self.heading is not None:
            if not self.measured and self.probes < 2:
                # the heading came from a big turn: roll PROBE_CM forward (then back) so the camera measures it
                self.probes += 1
                return ("probe", PROBE_CM)
            e = wrap180(self.final_heading - self.heading)
            if abs(e) > self.heading_tol and self.final_turns < 3:
                self.final_turns += 1
                return ("turn", round(e, 2))
        return ("done", 0.0)

    def after(self, cmd, p0, p1):
        """Update the heading from a finished leg: cmd as sent, p0 / p1 camera positions before / after."""
        self.legs += 1
        kind, val = cmd[0], cmd[1]
        if kind in ("go", "probe"):
            moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if moved >= MIN_HEADING_CM:
                h = bearing(p0, p1)
                self.heading = h if val >= 0 else wrap180(h + 180.0)
                self.measured = True
        elif kind == "turn" and self.heading is not None:
            self.heading = wrap180(self.heading + val)
            self.measured = self.measured and abs(val) <= SMALL_TURN_DEG


def heading_from_track(samples, min_cm=MIN_HEADING_CM, max_rms_cm=0.6):
    """Direction of travel (deg) of a stretch of camera samples [(t, x, y), ...] if it is long and
    straight enough, else None. The robot may have driven backwards: the caller decides."""
    if len(samples) < 3:
        return None
    x0, y0 = samples[0][1], samples[0][2]
    x1, y1 = samples[-1][1], samples[-1][2]
    L = math.hypot(x1 - x0, y1 - y0)
    if L < min_cm:
        return None
    ux, uy = (x1 - x0) / L, (y1 - y0) / L
    off = [(s[1] - x0) * -uy + (s[2] - y0) * ux for s in samples]
    rms = math.sqrt(sum(o * o for o in off) / len(off))
    if rms > max_rms_cm:
        return None
    # least squares direction through all points (less noisy than the two ends)
    n = len(samples)
    mx = sum(s[1] for s in samples) / n
    my = sum(s[2] for s in samples) / n
    sxx = sum((s[1] - mx) ** 2 for s in samples)
    syy = sum((s[2] - my) ** 2 for s in samples)
    sxy = sum((s[1] - mx) * (s[2] - my) for s in samples)
    a = 0.5 * math.atan2(2 * sxy, sxx - syy)
    h = math.degrees(a)
    if math.cos(math.radians(h)) * ux + math.sin(math.radians(h)) * uy < 0:
        h = wrap180(h + 180.0)
    return h


def leg_stuck(cmd, p0, p1):
    """True when a go leg moved far less than asked (blocked, wheels slipping, or the edge guard)."""
    if cmd[0] not in ("go", "probe") or abs(cmd[1]) < 2.0:
        return False
    moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    return moved < STUCK_FRACTION * abs(cmd[1])


SAFETY_STOPS = ("obstacle", "plywood edge", "watchdog", "drove further", "stop command")


def judge_run(far_err_cm, end_err_cm, on_board=True, seen=True, recover_cm=RECOVER_CM, runend="done", lost_cm=None):
    """Did the run need a RECOVERY (not the same as an outlier: outliers are decided statistically within a
    condition by the analysis; every run is parked by camera afterwards anyway, which flags nothing)?
    Recovery = the robot was too far off to safely carry on by itself, or headed for a board, an open edge or a
    parked robot: 'unseen' (the camera lost it), 'off_board', 'safety_stop' (a front stop, the edge guard, the
    camera watchdog or the travel cap ended the route early: the run did not finish), 'runner_stop' (the runner's
    own `stop` to find the robot again ended it: not a safety event; 2026-10-24 review), 'far_off' (it ended more
    than recover_cm from its start spot). Returns (recovery or '', plain words)."""
    if lost_cm is not None:
        recover_cm = lost_cm
    if not seen:
        return "unseen", "the camera lost sight of it"
    if not on_board:
        return "off_board", "the camera sees it off the board"
    low = (runend or "done").lower()
    if "runner stop" in low:
        return "runner_stop", "the runner itself stopped its route to find it again (%s)" % runend
    if low != "done" and any(w in low for w in SAFETY_STOPS):
        return "safety_stop", "a safety stop ended its route early (%s)" % runend
    if end_err_cm is not None and end_err_cm > recover_cm:
        return "far_off", "it ended %.0f cm from its start spot (more than %.0f cm)" % (end_err_cm, recover_cm)
    return "", ""


def hazard(samples, board, parked, horizon_s=0.3, edge_cm=None, robot_cm=None):
    """Camera watchdog: is the moving robot (samples [(t, x, y), ...], newest last) at, or within horizon_s of,
    an open edge (centre nearer than edge_cm) or a parked robot (centre nearer than robot_cm), while moving
    towards it? parked: {name: (x, y)} (not the mover). Moving away from a hazard is always allowed (so it can be
    brought back). Returns plain words or None."""
    import e02_plan as PL
    edge_cm = PL.WATCH_EDGE_CM if edge_cm is None else edge_cm
    robot_cm = PL.WATCH_ROBOT_CM if robot_cm is None else robot_cm
    if not samples:
        return None
    win = [q for q in samples if q[0] >= samples[-1][0] - 0.5]
    if len(win) < 6 or win[-1][0] - win[0][0] < 0.3:
        return None
    # medians of the first and last few pictures: one stray camera position must not look like a dash
    k = max(2, len(win) // 4)
    t0, t1 = float(np.median([q[0] for q in win[:k]])), float(np.median([q[0] for q in win[-k:]]))
    x0, y0 = float(np.median([q[1] for q in win[:k]])), float(np.median([q[2] for q in win[:k]]))
    x, y = float(np.median([q[1] for q in win[-k:]])), float(np.median([q[2] for q in win[-k:]]))
    vx, vy = (x - x0) / max(t1 - t0, 1e-3), (y - y0) / max(t1 - t0, 1e-3)
    sp = math.hypot(vx, vy)
    if sp > 30.0:                                # faster than any robot drives: a tracking jump, not a robot
        return None
    if sp < 1.0:                                 # standing still (camera noise): nothing is closing in
        vx = vy = 0.0
    px, py = x + vx * horizon_s, y + vy * horizon_s
    for a, v in board.open_edges():
        lo = min(board.x0, board.x1) if a == "x" else min(board.y0, board.y1)
        sgn = 1.0 if abs(v - lo) < 1e-6 else -1.0            # inward distance: + on the board, - past the edge
        d0 = sgn * ((x if a == "x" else y) - v)
        d1 = sgn * ((px if a == "x" else py) - v)
        if d1 < edge_cm and d1 < d0 - 0.5:          # closing at 1.7+ cm/s, not just drifting along it
            return ("%.0f cm from the open edge and heading for it" % d0) if d0 >= 0 else \
                ("%.0f cm past the open edge and still going" % -d0)
    for n, (qx, qy) in (parked or {}).items():
        d0 = math.hypot(x - qx, y - qy)
        d1 = math.hypot(px - qx, py - qy)
        if d1 < robot_cm and d1 < d0 - 0.5:
            return "%.0f cm from parked %s and heading for it" % (d0, n)
    return None


def nick_instructions(robot, problem, where_words, facing_words):
    """The plain, exact steps for Nick when a robot needs a hand (stuck or off the board)."""
    return [
        "%s needs your help: %s." % (robot, problem),
        "1. Go to the board. %s is the robot whose two lights are flashing white now (it flashes for 3 s; "
        "type f and press Enter here to flash it again)." % robot,
        "2. Pick it up and put it down flat on the board %s, with its front (the end with the dark sensor "
        "window) facing %s." % (where_words, facing_words),
        "3. Step back so your hands are out of the picture, then come back to this window and press Enter.",
        "   (If it is broken or flat, take it off the board, type skip and press Enter: its remaining runs are put "
        "aside.)",
    ]
