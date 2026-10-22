"""The robot dance (Nick, 2026-10-22: "choreograph a dance where the robots are moving and turning all around the
board, just barely missing each other but never crashing"). Three Alviks, about 60-90 s, set to a beat we make
ourselves; lights flash on the beat; how close they pass comes from their MEASURED precision.

    python tools/experiment/e02.py --dance-only ...      setup, then the dance, then parking
    python tools/experiment/e02.py --dance ...           the E02 session, then the dance

What is in here (the driving itself is e02.py's Runner.dance -> DanceRun below):
  Speeds       how long a `go` / `turn` idle leg takes, fitted from the session's own LEG records (started -> done,
               minus the firmware's 0.35 s stand-still while its edge guard learns the floor), per robot; the
               2026-10-21 real robots' legs when there are too few.
  gap_rule     the gap the plan keeps between two robots' bodies: k x their proven 95th-percentile position error
               (the error of a robot against the plan at the same moment, while dancing: position and timing
               together), with k = K_GAP = 2 (both robots off by that much, straight at each other, at the same
               moment: they still don't touch). "Proven" = the upper 95% confidence bound of the 95th percentile, from
               earlier dances' per-move errors (59 moves at least); until then a conservative default.
  Choreo       the composer: per-robot moves (go cm / turn deg / flash) placed on a beat grid; figures (sync spins,
               ripples, near-miss swaps, crossings just behind, orbits, follow-the-leader, a tight finale) built
               from them; "go as soon as it is safe" places a move at the first half-beat where it keeps the gap.
  check        simulates every robot's commanded motion with its measured speeds and checks: every pair keeps the
               gap at all times; nobody nearer an open edge than the gap or a blocked board than 3 cm; nothing in
               front of a moving robot's front sensor that would stop it; every move ends before the next begins;
               flashes only while standing still; and the same with timing jitter and speed errors (robust) against
               the hard minimum the supervisor stops at.
  make_music   a royalty-free beat at the dance's BPM, synthesised here (count-in, drums, bass, chords, a crash
               at every new figure, a bell on every light flash).
  DanceRun     the execution: robots to their start poses, a count-in, every move sent at its beat (early by the
               radio delay and the firmware's stand-still), camera fixes and corrected legs between moves, a safety
               supervisor that stops ALL robots when a pair is predicted closer than the hard minimum or a robot
               is lost, then the numbers (closest approach planned vs actual, timing error) and parking.
  DancePage    the live view's dance page (http://localhost:8090/dance): plays the beat in the browser in sync with
               the robots, draws the planned and actual paths over the camera picture.

Lights: the only light command the firmware has that works standing still without a run is `identify` (both LEDs
white, 250 ms on / 250 ms off, 6 times: 2 flashes a second for 3 s, and it blocks the robot for those 3 s). At 120
BPM that is one flash per beat, so flashes are sent so they start on a beat, only while the robot stands still.
Wanted from the firmware (not made): a non-blocking `led <r> <g> <b> [ms]` (or `pulse`) command, so lights can
flash on the beat while a robot moves and each robot can have its own colour.
"""
import glob
import json
import math
import os
import sys
import threading
import time
import wave
from collections import defaultdict, deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

# --- the robot and the firmware ------------------------------------------------------------------------------------
ROBOT_HALF_CM = 4.75
R_BODY = ROBOT_HALF_CM * math.sqrt(2.0)  # 6.72 cm: the circle its square body sweeps turning on the spot. Every gap
                                         # here is between these circles: the shells' real gap is the same or more
SENSOR_FWD_CM = 4.5                      # front distance sensor ahead of the axle (tools/sim config)
TOF_HALF_DEG = 27.0                      # its outer zones look about this far to each side (zones +-22 deg, 11 wide)
SAFE_FRONT_CM = 7.0                      # firmware safe_front_cm: a go leg stops if something is nearer than this...
FRONT_END_CM = 2.0                       # ...within the rest of the leg + front_end_cm
LEARN_S = 0.35                           # firmware: an idle leg stands still edge_learn_ms (300) + 50 ms first
FLASH_S = 3.0                            # `identify`: 6 flashes, 2 a second, blocking
FLASH_HZ = 2.0
BACK_MAX_CM = 9.5                        # firmware e02_back_max_cm 10: longest backwards leg
GRID_BEATS = 0.5                         # moves start on half beats
MAX_PIECE_DEG = 160.0                    # longest planned turn leg (the firmware's limit is 180; corrections need room)
SLACK_S = 0.45                          # a robot's next move starts at least this long after its last one ends
BOARD_CLEAR_CM = 3.0                     # body circle to a blocked board's face, at least
EDGE_STOP_CM = 9.0                       # supervisor: a centre about to be this near an open edge (as the watchdog)
UNSURE_STOP_S = 1.5                      # supervisor: a robot the camera is unsure of this long stops the dance
HORIZON_S = 0.6                          # supervisor: looks this far ahead
JITTER_S, SPEED_TOL = 0.25, 0.08         # robust check: every move up to this early / late, speeds this much off
COUNT_IN_BEATS = 8                       # two bars of count-in before the robots start

# --- the gap rule ------------------------------------------------------------------------------------------------
K_GAP = 2.0                  # gap = K_GAP x e95: both robots off by their 95th percentile, straight at each other,
                             # at the same moment, still don't touch (chosen; stated in every summary)
DEFAULT_E95_CM = 6.0         # CHOSEN, not measured: no dance has been measured yet. Why 6: the camera-steered parking
                             # legs are allowed 5 cm of camera / wheels disagreement (e02.py CHECK_CM) before they
                             # count as wrong, plus ~1 cm of timing (0.1 s at 9 cm/s)
GAP_FLOOR_CM = 4.0           # never planned closer than this (camera outlines start to merge, front sensors to fire)
HARD_FLOOR_CM = 3.0
MIN_PROOF_MOVES = 59         # the fewest per-move errors whose maximum is a 95% upper bound of the 95th percentile

# measured on the real robots, 2026-10-21 (release e02-setup-oct21, logs.zip, every robot's leg.csv: 56 go and 58 turn
# idle legs, started -> done minus LEARN_S): go 12 cm 1.63 s, 25 cm 3.09 s; turn 15 deg 0.82 s, 90 deg 1.90 s
REAL_OCT21 = dict(go_lag_s=0.28, go_cms=8.9, turn_lag_s=0.60, turn_dps=69.0)

ROLES = ("A", "B", "C")
COLORS = ("#f2a33a", "#5aa9ff", "#3ccf8e", "#c678dd", "#e06c75", "#e5c07b", "#56b6c2", "#ff7eb6")


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def bearing(p, q):
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))


def seg_dist(p, a, b):
    """Distance from point p to the segment a-b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


# =============================================================================== speeds
class Speeds:
    """How long a leg's MOTION takes (from when its wheels start, LEARN_S after the robot started the leg):
    go: go_lag_s + |cm| / go_cms; turn: turn_lag_s + |deg| / turn_dps."""

    def __init__(self, go_lag_s, go_cms, turn_lag_s, turn_dps, source="", n_go=0, n_turn=0):
        self.go_lag_s, self.go_cms = float(go_lag_s), float(go_cms)
        self.turn_lag_s, self.turn_dps = float(turn_lag_s), float(turn_dps)
        self.source, self.n_go, self.n_turn = source, n_go, n_turn

    @classmethod
    def default(cls):
        return cls(source="the real robots' legs of 2026-10-21 (no fit from this session)", **REAL_OCT21)

    def duration(self, kind, value):
        if kind == "go":
            return self.go_lag_s + abs(value) / self.go_cms
        if kind == "turn":
            return self.turn_lag_s + abs(value) / self.turn_dps
        if kind == "flash":
            return FLASH_S
        return 0.0

    def ramp(self, kind="go"):
        return min(self.go_lag_s, 0.5) if kind == "go" else min(self.turn_lag_s, 0.6)

    def scaled(self, k):
        """The same robot k times faster (k < 1: slower), lags unchanged."""
        return Speeds(self.go_lag_s, self.go_cms * k, self.turn_lag_s, self.turn_dps * k, self.source)

    def to_dict(self):
        return dict(go_lag_s=round(self.go_lag_s, 3), go_cms=round(self.go_cms, 2), turn_lag_s=round(self.turn_lag_s, 3),
                    turn_dps=round(self.turn_dps, 1), source=self.source, n_go=self.n_go, n_turn=self.n_turn)

    @classmethod
    def from_dict(cls, d):
        return cls(d["go_lag_s"], d["go_cms"], d["turn_lag_s"], d["turn_dps"], d.get("source", ""), d.get("n_go", 0),
                   d.get("n_turn", 0))

    def words(self):
        return "drives %.1f cm/s (+%.2f s to start and stop), turns %.0f deg/s (+%.2f s); %s" % (
            self.go_cms, self.go_lag_s, self.turn_dps, self.turn_lag_s, self.source)


def leg_pairs(recs, robot):
    """[(kind, value, motion seconds)] of a robot's finished idle legs, from LEG records [(t, robot, rec)]."""
    started, out = {}, []
    for t, n, rec in recs:
        if n != robot:
            continue
        state = rec.get("state") or rec.get("result")
        key = (rec.get("run"), rec.get("id"))
        try:
            ms = float(rec.get("ms"))
            val = float(rec.get("value"))
        except (TypeError, ValueError):
            continue
        if state == "started":
            started[key] = ms
        elif state == "done" and key in started and rec.get("kind") in ("go", "turn"):
            out.append((rec["kind"], val, (ms - started.pop(key)) / 1000.0 - LEARN_S))
    return out


def fit_all(recs, names, base=None):
    """Speeds per robot from the session's LEG records: each robot's own legs where there are enough, else every
    robot's legs together (same firmware, same settings), else the real robots' 2026-10-21 values."""
    base = base or Speeds.default()
    pooled_pairs = []
    for n in names:
        pooled_pairs += leg_pairs(recs, n)
    pooled = fit_speeds(pooled_pairs, base)
    pooled.source = pooled.source.replace("this session's legs", "this session's legs (all robots together)")
    return {n: fit_speeds(leg_pairs(recs, n), pooled) for n in names}


def fit_speeds(pairs, base=None):
    """Speeds from [(kind, value, motion s)]; each of go / turn fitted only with enough legs of different sizes
    (else the base's, by default the real robots' 2026-10-21 values)."""
    base = base or Speeds.default()
    out = dict(go_lag_s=base.go_lag_s, go_cms=base.go_cms, turn_lag_s=base.turn_lag_s, turn_dps=base.turn_dps)
    n = dict(go=0, turn=0)
    # turns under 25 deg are left out: the IMU turn slows down near its target, so small turns don't follow the
    # straight line the dance's bigger turns do
    for kind, lo, rng, vlim, llim in (("go", 3.0, 8.0, (3.0, 25.0), (0.0, 1.5)), ("turn", 25.0, 30.0, (25.0, 250.0), (0.0, 1.8))):
        pts = [(abs(v), s) for k, v, s in pairs if k == kind and abs(v) >= lo and 0.05 < s < 30.0]
        if len(pts) < 4 or (max(p[0] for p in pts) - min(p[0] for p in pts)) < rng:
            continue
        x, y = np.array([p[0] for p in pts]), np.array([p[1] for p in pts])
        slope, icpt = np.polyfit(x, y, 1)
        if slope <= 0:
            continue
        rate = 1.0 / slope
        if not (vlim[0] <= rate <= vlim[1] and llim[0] <= icpt <= llim[1]):
            continue
        out[kind + ("_cms" if kind == "go" else "_dps")] = float(rate)
        out[kind + "_lag_s"] = float(icpt)
        n[kind] = len(pts)
    src = []
    if n["go"]:
        src.append("driving fitted from %d of this session's legs" % n["go"])
    if n["turn"]:
        src.append("turning fitted from %d legs" % n["turn"])
    if not src:
        return Speeds(source=base.source, **{k: out[k] for k in out})
    if not (n["go"] and n["turn"]):
        src.append("the rest: " + base.source)
    return Speeds(source="; ".join(src), n_go=n["go"], n_turn=n["turn"], **out)


# =============================================================================== the gap rule
def upper_p95(vals, conf=0.95):
    """A conf upper confidence bound of the 95th percentile from samples (order statistic, binomial), or None
    with too few samples (fewer than MIN_PROOF_MOVES for the default)."""
    from scipy.stats import binom
    v = sorted(float(x) for x in vals)
    n = len(v)
    for j in range(1, n + 1):
        if binom.cdf(j - 1, n, 0.95) >= conf:
            return v[j - 1]
    return None


def gap_rule(samples=None, source="", k=K_GAP, default_e95=DEFAULT_E95_CM, proxy=None, timing_p95_s=None):
    """The gaps the dance keeps (between the body circles, cm): gap_cm (planned: every pair at all times; also
    from the open edges) and hard_cm (the supervisor stops everything below it). samples: per-move errors (cm) of
    earlier dances with this control method; proxy: (p95 cm, words) from another measure when there are no dances.
    Returns dict(e95_cm, p95_cm, n, k, gap_cm, hard_cm, source, proven, words)."""
    samples = [float(s) for s in (samples or []) if s is not None and math.isfinite(float(s))]
    ub = upper_p95(samples) if len(samples) >= MIN_PROOF_MOVES else None
    p95 = float(np.percentile(samples, 95)) if samples else None
    if ub is not None:
        e95, why, proven = ub, "proven: the 95%% upper bound of the 95th percentile of %d measured dance moves (%s)" % (
            len(samples), source or "earlier dances"), True
    elif proxy is not None:
        e95, why, proven = max(float(proxy[0]), 0.5), "a stand-in: %s (only %d dance moves measured; %d needed)" % (
            proxy[1], len(samples), MIN_PROOF_MOVES), False
    else:
        e95, why, proven = default_e95, "the conservative default (chosen, not measured: %d dance moves measured so " \
                                        "far; %d needed to prove a smaller one)" % (len(samples), MIN_PROOF_MOVES), False
    gap = max(GAP_FLOOR_CM, k * e95)
    hard = max(HARD_FLOOR_CM, e95)
    words = ("Gap rule: two robots' bodies stay at least %.1f cm apart (%.1f x %.1f cm, their 95th-percentile position "
             "error while dancing; %s). The safety supervisor stops every robot if two are about to come closer "
             "than %.1f cm." % (gap, k, e95, why, hard))
    # the robust check's timing and speed errors: measured ones once the precision is proven (they are part of
    # what e95 measured), else the chosen JITTER_S / SPEED_TOL
    jit = JITTER_S if not (proven and timing_p95_s) else max(0.05, min(JITTER_S, float(timing_p95_s)))
    tol = SPEED_TOL if not proven else 0.04
    return dict(e95_cm=round(e95, 2), p95_cm=round(p95, 2) if p95 is not None else None, n=len(samples), k=k,
                gap_cm=round(gap, 2), hard_cm=round(hard, 2), source=why, proven=proven, words=words,
                jitter_s=round(jit, 3), speed_tol=tol)


def load_precision(logs_dir, dry_run, k=K_GAP, proxy=None):
    """gap_rule from every earlier dance's per-move errors in logs_dir/e02-*/dance_result.json (dry runs only use
    dry runs, real sessions only real ones: the simulator's precision proves nothing about the real robots)."""
    vals, n_d, tim = [], 0, []
    for p in sorted(glob.glob(os.path.join(logs_dir, "e02-*", "dance_result.json"))):
        if ("-dryrun" in os.path.basename(os.path.dirname(p))) != bool(dry_run):
            continue
        try:
            d = json.load(open(p))
        except (OSError, ValueError):
            continue
        v = [x for x in d.get("move_dev_cm") or [] if isinstance(x, (int, float))]
        if v:
            vals += v
            n_d += 1
            tp = (d.get("timing_robot_s") or {}).get("p95_abs")
            if isinstance(tp, (int, float)):
                tim.append(float(tp))
    src = "%d earlier %sdance%s" % (n_d, "simulated " if dry_run else "", "" if n_d == 1 else "s")
    return gap_rule(vals, src, k=k, proxy=proxy, timing_p95_s=max(tim) if tim else None)


# =============================================================================== the dance
class Dance:
    """A dance: per-role start poses and moves. A move: dict(kind go|turn|flash, value, beat (its motion starts on
    this beat after dance beat 0), t0 (s), fig (figure index), x0 y0 h0 / x1 y1 h1 (planned poses before / after),
    aim (a turn before a go: the go's planned end, to aim at), piece (k of n for a turn split in pieces))."""

    def __init__(self, bpm, roles, start, moves, figures, gap_rule_=None, name="Alvik dance"):
        self.bpm = float(bpm)
        self.spb = 60.0 / self.bpm
        self.roles = list(roles)
        self.start = {r: tuple(start[r]) for r in roles}
        self.moves = {r: list(moves.get(r, [])) for r in roles}
        self.figures = list(figures)            # [dict(name, words, beat)]
        self.rule = gap_rule_
        self.name = name
        self.count_in = COUNT_IN_BEATS

    def end_beat(self, speeds=None):
        end = 0.0
        for r in self.roles:
            for m in self.moves[r]:
                d = speeds[r].duration(m["kind"], m["value"]) if speeds else m.get("dur", 0.0)
                end = max(end, m["beat"] + d / self.spb)
        return end

    def to_dict(self):
        return dict(name=self.name, bpm=self.bpm, roles=self.roles, start=self.start, moves=self.moves,
                    figures=self.figures, rule=self.rule, count_in=self.count_in)

    @classmethod
    def from_dict(cls, d):
        dn = cls(d["bpm"], d["roles"], d["start"], d["moves"], d["figures"], d.get("rule"), d.get("name", "Alvik dance"))
        dn.count_in = d.get("count_in", COUNT_IN_BEATS)
        return dn


class Traj:
    """One robot's planned motion from its start pose and moves, with its speeds. jitter: {move index: s} added to
    planned starts; a move never starts before the previous one ended (the firmware queues it)."""

    def __init__(self, start, moves, speeds, spb, jitter=None, speed_k=1.0):
        self.segs = []                       # (t_start, t_end, kind, value, x0, y0, h0, x1, y1, h1)
        x, y, h = start
        prev_end = -1e9
        sp = speeds if speed_k == 1.0 else speeds.scaled(speed_k)
        for i, m in enumerate(moves):
            t0 = m["beat"] * spb + (jitter.get(i, 0.0) if jitter else 0.0)
            t0 = max(t0, prev_end + (LEARN_S if jitter else 0.0))
            dur = sp.duration(m["kind"], m["value"])
            x1, y1, h1 = x, y, h
            if m["kind"] == "go":
                x1 = x + m["value"] * math.cos(math.radians(h))
                y1 = y + m["value"] * math.sin(math.radians(h))
            elif m["kind"] == "turn":
                h1 = h + m["value"]
            self.segs.append((t0, t0 + dur, m["kind"], m["value"], x, y, h, x1, y1, h1, sp.ramp(m["kind"])))
            x, y, h = x1, y1, h1
            prev_end = t0 + dur
        self.start = start
        self.end_pose = (x, y, h)
        self.t_end = prev_end if self.segs else 0.0

    def sample(self, t):
        """Arrays x, y, h (deg), fwd (in a forward go), rem (cm of that go still to come) at times t."""
        t = np.asarray(t, float)
        x = np.full(t.shape, float(self.start[0]))
        y = np.full(t.shape, float(self.start[1]))
        h = np.full(t.shape, float(self.start[2]))
        fwd = np.zeros(t.shape, bool)
        rem = np.zeros(t.shape)
        for k, (s0, s1, kind, val, x0, y0, h0, x1, y1, h1, ramp) in enumerate(self.segs):
            nxt = self.segs[k + 1][0] if k + 1 < len(self.segs) else np.inf
            after = t >= s1
            x[after], y[after], h[after] = x1, y1, h1
            during = (t >= s0) & (t < s1)
            if during.any():
                tau = t[during] - s0
                f = _trap_vec(tau, s1 - s0, ramp)
                if kind == "go":
                    x[during] = x0 + (x1 - x0) * f
                    y[during] = y0 + (y1 - y0) * f
                    h[during] = h0
                    if val > 0:
                        fwd[during] = True
                        rem[during] = val * (1.0 - f)
                elif kind == "turn":
                    x[during], y[during] = x0, y0
                    h[during] = h0 + (h1 - h0) * f
                else:
                    x[during], y[during], h[during] = x0, y0, h0
            del nxt
        return x, y, h, fwd, rem

    def pose_at(self, t):
        x, y, h, _, _ = self.sample(np.array([t]))
        return float(x[0]), float(y[0]), float(h[0])


def _trap_vec(tau, dur, ramp):
    r = np.minimum(max(ramp, 1e-6), dur / 2.0)
    total = dur - r
    s = np.where(tau < r, 0.5 * tau * tau / r,
                 np.where(tau <= dur - r, 0.5 * r + (tau - r), total - 0.5 * (dur - tau) ** 2 / r))
    return np.clip(s / total, 0.0, 1.0)


# =============================================================================== the stage
class Stage:
    """Where the dance may go on this board: s / t = cm in from the y-board's / x-board's face (the blocked boards),
    so the same dance works for any corner the boards are in. The box the figures are drawn in keeps every body
    circle the gap from the open edges and well off the boards."""

    def __init__(self, board, gap_cm):
        self.b = board
        self.g = gap_cm
        open_s = abs(next(v for a, v in board.open_edges() if a == "x") - board.yboard_cm)
        open_t = abs(next(v for a, v in board.open_edges() if a == "y") - board.xboard_cm)
        lo = R_BODY + 8.0
        self.s_lo, self.s_hi = lo, open_s - R_BODY - gap_cm - 1.0
        self.t_lo, self.t_hi = lo, open_t - R_BODY - gap_cm - 1.0
        self.cs, self.ct = (self.s_lo + self.s_hi) / 2.0, (self.t_lo + self.t_hi) / 2.0
        self.H = min(self.s_hi - self.s_lo, self.t_hi - self.t_lo) / 2.0

    def P(self, ds, dt):
        """A floor point ds / dt from the box's centre (stage directions: away from the y-board / the x-board)."""
        return self.b.uv(self.cs + ds, self.ct + dt)

    @property
    def O(self):
        return self.P(0.0, 0.0)

    def polar(self, r, ang_deg):
        """A point r cm from the centre, at angle ang (stage frame: 0 = away from the y-board, 90 = away from the
        x-board)."""
        return self.P(r * math.cos(math.radians(ang_deg)), r * math.sin(math.radians(ang_deg)))


def edge_clearance(board, x, y):
    """(body circle to the nearest open edge, body circle to the nearest blocked board's face), cm (arrays ok)."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    opens = []
    for a, v in board.open_edges():
        lo = min(board.x0, board.x1) if a == "x" else min(board.y0, board.y1)
        sgn = 1.0 if abs(v - lo) < 1e-6 else -1.0
        opens.append(sgn * ((x if a == "x" else y) - v))
    od = np.minimum(opens[0], opens[1]) - R_BODY
    bd = np.minimum((y - board.xboard_cm) * board.uy, (x - board.yboard_cm) * board.ux) - R_BODY
    return od, bd


# =============================================================================== the composer
class Choreo:
    """Builds a Dance figure by figure. Every primitive starts a move on the half-beat grid, no earlier than
    `beat` and than the robot is free (its last move ended SLACK_S ago); `safe=True` moves it later, half a beat at
    a time, until the move keeps `need` cm from every robot already planned (a near miss, on purpose)."""

    def __init__(self, board, bpm, speeds, rule, roles=ROLES, margin=0.0):
        self.board = board
        self.bpm = float(bpm)
        self.spb = 60.0 / self.bpm
        self.sp = speeds
        self.rule = rule
        self.roles = list(roles)
        self.need = rule["gap_cm"] + margin
        self.D = 2.0 * R_BODY + self.need + 0.3        # centre to centre when passing at the gap
        self.stage = Stage(board, rule["gap_cm"])
        self.pose, self.free, self.moves = {}, {}, {r: [] for r in self.roles}
        self.figs = []
        self.cur = -1
        self._tr = {}

    # --- basics ------------------------------------------------------------------------------------------
    def place(self, role, xy, heading):
        self.pose[role] = (float(xy[0]), float(xy[1]), float(heading))
        self.free[role] = -1e9
        self._tr.pop(role, None)

    def figure(self, name, words):
        self.figs.append(dict(name=name, words=words, beat=None))
        self.cur = len(self.figs) - 1

    def ready(self, role):
        return self._grid((self.free[role] + SLACK_S) / self.spb) if self.free[role] > -1e8 else 0.0

    def at(self, roles):
        return max(self.ready(r) for r in roles)

    @staticmethod
    def _grid(beat):
        return math.ceil(beat / GRID_BEATS - 1e-9) * GRID_BEATS

    def _traj(self, role):
        if role not in self._tr:
            self._tr[role] = Traj(self.start_pose(role), self.moves[role], self.sp[role], self.spb)
        return self._tr[role]

    def start_pose(self, role):
        return self._start[role]

    def _add(self, role, kind, value, beat=None, **extra):
        b = self.ready(role) if beat is None else max(self._grid(beat), self.ready(role))
        x, y, h = self.pose[role]
        x1, y1, h1 = x, y, h
        if kind == "go":
            x1, y1 = x + value * math.cos(math.radians(h)), y + value * math.sin(math.radians(h))
        elif kind == "turn":
            h1 = wrap180(h + value)
        dur = self.sp[role].duration(kind, value)
        m = dict(kind=kind, value=round(float(value), 2), beat=b, t0=round(b * self.spb, 3), fig=self.cur,
                 x0=round(x, 2), y0=round(y, 2), h0=round(h, 2), x1=round(x1, 2), y1=round(y1, 2), h1=round(h1, 2),
                 dur=round(dur, 3), **extra)
        self.moves[role].append(m)
        self.pose[role] = (x1, y1, h1)
        self.free[role] = b * self.spb + dur
        self._tr.pop(role, None)
        if self.cur >= 0 and self.figs[self.cur]["beat"] is None:
            prev = [f["beat"] for f in self.figs[:self.cur] if f.get("beat") is not None]
            self.figs[self.cur]["beat"] = max([b] + prev)
        return b + dur / self.spb

    # --- primitives -------------------------------------------------------------------------------------
    def turn(self, role, deg, beat=None, **extra):
        # pieces of at most MAX_PIECE_DEG: the firmware takes -180..180, and a correction (up to 20 deg) must fit
        n = max(1, int(math.ceil(abs(deg) / MAX_PIECE_DEG - 1e-9)))
        end = None
        for k in range(n):
            end = self._add(role, "turn", deg / n, beat if k == 0 else None, piece=[k + 1, n], **extra)
        return end

    def turn_to(self, role, heading, beat=None, min_deg=2.0, **extra):
        d = wrap180(heading - self.pose[role][2])
        if abs(d) < min_deg:
            return beat if beat is not None else self.ready(role)
        return self.turn(role, d, beat, **extra)

    def go(self, role, d, beat=None, safe=False, **extra):
        if d < -BACK_MAX_CM:
            raise ValueError("backwards legs are at most %.1f cm (blind)" % BACK_MAX_CM)
        if not safe:
            return self._add(role, "go", d, beat, **extra)
        return self._safe(role, "go", d, beat, **extra)

    def flash(self, role, beat=None):
        return self._add(role, "flash", 0, beat)

    def goto(self, role, pt, beat=None, face=None, safe=False):
        """Turn to face pt, drive there; then turn to `face` (deg) if given. safe: the drive waits for a safe moment,
        and when another robot STANDS in the way (no moment is safe), it goes round it by a waypoint beside it.
        Returns the end beat."""
        if not safe:
            end = self._goto(role, pt, beat, False)
        else:
            snap = self._snap(role)
            try:
                end = self._goto(role, pt, beat, True)
            except RuntimeError as e:
                end = None
                for wp in self._detours(role, pt):
                    self._restore(role, snap)
                    try:
                        self._goto(role, wp, beat, True)
                        end = self._goto(role, pt, None, True)
                        break
                    except RuntimeError:
                        end = None
                if end is None:
                    self._restore(role, snap)
                    raise RuntimeError("%s: no safe way to (%.0f, %.0f): %s" % (role, pt[0], pt[1], e))
        if face is not None:
            end = self.turn_to(role, face, None)
        return end

    def _goto(self, role, pt, beat, safe):
        x, y, h = self.pose[role]
        d = math.hypot(pt[0] - x, pt[1] - y)
        end = beat if beat is not None else self.ready(role)
        if d >= 1.0:
            self.turn_to(role, bearing((x, y), pt), beat, aim=[round(pt[0], 2), round(pt[1], 2)])
            end = self._safe(role, "go", d, None) if safe else self._add(role, "go", d, None)
        return end

    def snap_all(self):
        return ({r: list(v) for r, v in self.moves.items()}, dict(self.pose), dict(self.free),
                [dict(f) for f in self.figs], self.cur)

    def restore_all(self, snap):
        self.moves = {r: list(v) for r, v in snap[0].items()}
        self.pose, self.free = dict(snap[1]), dict(snap[2])
        self.figs = [dict(f) for f in snap[3]]
        self.cur = snap[4]
        self._tr = {}

    def _snap(self, role):
        return (list(self.moves[role]), self.pose[role], self.free[role], [f.get("beat") for f in self.figs])

    def _restore(self, role, snap):
        self.moves[role], self.pose[role], self.free[role] = list(snap[0]), snap[1], snap[2]
        for f, b in zip(self.figs, snap[3]):
            f["beat"] = b
        self._tr.pop(role, None)

    def inside(self, pt, extra=0.5, board_min=8.0):
        od, bd = edge_clearance(self.board, pt[0], pt[1])
        return float(od) >= self.rule["gap_cm"] + extra and float(bd) >= board_min

    def _detours(self, role, pt):
        """Waypoints beside the robots standing near the straight way from role's pose to pt, shortest first."""
        p = self.pose[role][:2]
        L = math.hypot(pt[0] - p[0], pt[1] - p[1])
        if L < 1e-6:
            return []
        ux, uy = (pt[0] - p[0]) / L, (pt[1] - p[1]) / L
        n = (-uy, ux)
        cands = []
        for o in self.roles:
            if o == role:
                continue
            q = self.pose[o][:2]
            along = max(0.0, min(L, (q[0] - p[0]) * ux + (q[1] - p[1]) * uy))
            side = math.hypot(p[0] + ux * along - q[0], p[1] + uy * along - q[1])
            if side > self.D + 4.0:
                continue
            for sgn in (1.0, -1.0):
                for k in (1.0, 1.3):
                    w = (q[0] + sgn * n[0] * (self.D + 3.0) * k, q[1] + sgn * n[1] * (self.D + 3.0) * k)
                    if self.inside(w):
                        cands.append(w)
        mid = (p[0] + ux * L / 2, p[1] + uy * L / 2)
        for sgn in (1.0, -1.0):
            for k in (1.0, 1.6):
                w = (mid[0] + sgn * n[0] * (self.D + 3.0) * k, mid[1] + sgn * n[1] * (self.D + 3.0) * k)
                if self.inside(w):
                    cands.append(w)
        cands.sort(key=lambda w: math.hypot(w[0] - p[0], w[1] - p[1]) + math.hypot(pt[0] - w[0], pt[1] - w[1]))
        return cands

    def _safe(self, role, kind, value, beat, step=GRID_BEATS, max_wait_beats=40, **extra):
        """Add the move at the first half beat (from `beat` / when free) where it keeps `need` from every other
        robot's planned motion, standing where it ends afterwards."""
        b0 = self.ready(role) if beat is None else max(self._grid(beat), self.ready(role))
        saved = (list(self.moves[role]), self.pose[role], self.free[role])
        for k in range(int(max_wait_beats / step) + 1):
            b = b0 + k * step
            self._add(role, kind, value, b, **extra)
            if self.min_gap(role) >= self.need:
                return b + self.sp[role].duration(kind, value) / self.spb
            self.moves[role], self.pose[role], self.free[role] = list(saved[0]), saved[1], saved[2]
            self._tr.pop(role, None)
        raise RuntimeError("no safe moment for %s's %s %.0f within %d beats" % (role, kind, value, max_wait_beats))

    def min_gap(self, role, t_from=None):
        """The smallest gap (body circles) between `role` and every other robot, from the start of role's last move
        (or t_from) until everyone has stopped."""
        mv = self.moves[role]
        t0 = (mv[-1]["beat"] * self.spb if mv else 0.0) if t_from is None else t_from
        t1 = max(self._traj(r).t_end for r in self.roles if r in self._start) + 2.0
        t = np.arange(t0, max(t1, t0 + 0.1), 0.05)
        xa, ya, _, _, _ = self._traj(role).sample(t)
        best = np.inf
        for r in self.roles:
            if r == role or r not in self._start:
                continue
            xb, yb, _, _, _ = self._traj(r).sample(t)
            best = min(best, float(np.min(np.hypot(xa - xb, ya - yb))) - 2.0 * R_BODY)
        return best

    def dance(self, name="Alvik dance"):
        return Dance(self.bpm, self.roles, self._start, self.moves, self.figs, self.rule, name)


# =============================================================================== the 3-robot dance
RING_K = 0.9                    # the ring the robots dance on: this share of the stage's half-size


def ring_radius(stage):
    return RING_K * stage.H


def start_formation(board, rule):
    """Start poses: a triangle on the dance ring (A at 90 deg, B at 210, C at 330, stage angles), every robot facing
    the middle. {role: (x, y, heading)}."""
    st = Stage(board, rule["gap_cm"])
    rho = ring_radius(st)
    out = {}
    for role, ang in zip(ROLES, (90.0, 210.0, 330.0)):
        p = st.polar(rho, ang)
        out[role] = (round(p[0], 2), round(p[1], 2), round(bearing(p, st.O), 1))
    return out


def author_dance(board, rule, speeds, bpm=120.0, margin=0.0):
    """The 3-robot dance (about 60-90 s). speeds: {role: Speeds}. The robots stand on a ring round the middle of the
    stage, so every figure starts where the last one ended:
     1 wake up            all three flash on the beat (6 flashes)
     2 ripple             a look to the side and back that runs round the triangle, one robot after the other
     3 near-miss swap     two trade places across the ring on parallel lines exactly the gap apart, so they pass
                          shoulder to shoulder
     4 crossing           one drives straight across the middle; another cuts across its path just behind it
     5 orbit              the third takes the middle and turns to watch; the other two circle round it on the ring
     6 follow the leader  it goes back to the ring, and all three drive round the ring together, each following the
                          one in front
     7 finale             everyone straight into a tight triangle in the middle (each pair the gap apart), a spin and
                          a half together (it ends facing out), the lights on the beat, a bow
    Each figure tries a few variants (which robot does what, which way round) and keeps the first one that every
    move's safety search accepts; the whole dance is then checked (check())."""
    import itertools
    c = Choreo(board, bpm, speeds, rule, margin=margin)
    st = c.stage
    c.rho = ring_radius(st)
    start = start_formation(board, rule)
    c._start = {r: start[r] for r in ROLES}
    for r in ROLES:
        c.place(r, start[r][:2], start[r][2])
    c.ang = {r: a for r, a in zip(ROLES, (90.0, 210.0, 330.0))}
    A, B, C = ROLES

    c.figure("Wake up", "All three flash their lights on the beat.")
    for r in ROLES:
        c.flash(r, 0.0)
    c.figure("Ripple", "A look to the side and back runs round the triangle, one robot after the other.")
    b = c.at(ROLES)
    for k, r in enumerate((A, B, C)):
        c.turn(r, 40.0, b + 1.0 * k)
        c.turn(r, -40.0)
    c.figure("Near-miss swap", "Two robots trade places across the ring on parallel lines: they pass shoulder to "
                               "shoulder, just the gap apart.")
    _variants(c, [lambda p=p, d=d: fig_swap(c, p[0], p[1], p[2], d)
                  for p in ((A, B, C), (B, C, A), (C, A, B)) for d in (1.0, -1.0)])
    c.figure("Crossing", "One robot drives straight across the middle; another cuts across its path just behind it.")
    _variants(c, [lambda x=x: fig_crossing(c, *x) for x in crossing_options(c)[:12]])
    c.figure("Orbit", "One robot takes the middle and turns to watch; the other two circle round it on the ring.")
    _variants(c, [lambda m=m, d=d: fig_orbit(c, m, [r for r in ROLES if r != m], d)
                  for m in sorted(ROLES, key=lambda r: _still_since(c, r)) for d in (1.0, -1.0)])
    c.figure("Follow the leader", "Back to the ring, and all three drive round it together, each following the one "
                                  "in front.")
    _variants(c, [lambda d=d: fig_follow(c, d, steps=1) for d in (c.orbit_dir, -c.orbit_dir)])
    c.figure("Finale", "Everyone straight into a tight triangle in the middle, each pair just the gap apart; a spin "
                       "and a half together, the lights on the beat, and a bow.")
    _variants(c, [lambda: fig_finale(c)])
    del itertools
    return c.dance()


def _still_since(c, role):
    mv = c.moves[role]
    return mv[-1]["beat"] if mv else 0.0


def _variants(c, fns):
    """Try each way of doing a figure until one works (every safe move found its moment)."""
    snap = c.snap_all()
    ang = dict(getattr(c, "ang", {}))
    err = None
    for fn in fns:
        try:
            fn()
            return
        except RuntimeError as e:
            err = e
            c.restore_all(snap)
            c.ang = dict(ang)
    raise err or RuntimeError("no variant")


def _ring(c, ang):
    return c.stage.polar(c.rho, ang)


def _ring_go(c, role, ang, beat=None, safe=True):
    """role drives along the chord to ring angle ang (turning to aim first)."""
    p = _ring(c, ang)
    c.goto(role, p, beat, safe=safe)
    c.ang[role] = ang % 360.0


def fig_swap(c, a, b, third, sgn):
    """a and b trade places: a drives to b's ring angle minus delta, b to a's minus delta (delta the same way round
    for both): the two chords are parallel, 2 rho sin(span / 2) sin(delta / 2) apart: delta is chosen so that is
    the pass distance D. The third must stay clear."""
    span = (c.ang[b] - c.ang[a]) % 360.0
    sep = 2.0 * c.rho * abs(math.sin(math.radians(span / 2.0)))
    if sep < c.D + 2.0:
        raise RuntimeError("too close to swap")
    delta = 2.0 * math.degrees(math.asin(min(1.0, (c.D + 0.3) / sep)))
    ta, tb = c.ang[b] - sgn * delta, c.ang[a] - sgn * delta
    pa, pb = _ring(c, ta), _ring(c, tb)
    p3 = c.pose[third][:2]
    if min(math.hypot(pa[0] - p3[0], pa[1] - p3[1]), math.hypot(pb[0] - p3[0], pb[1] - p3[1])) < c.D + 2.0:
        raise RuntimeError("the swap ends too near the third robot")
    bt = c.at((a, b))
    c.turn_to(a, bearing(c.pose[a][:2], pa), bt, aim=list(pa))
    c.turn_to(b, bearing(c.pose[b][:2], pb), bt, aim=list(pb))
    bt = c.at((a, b))
    c.go(a, math.hypot(pa[0] - c.pose[a][0], pa[1] - c.pose[a][1]), bt)
    c.go(b, math.hypot(pb[0] - c.pose[b][0], pb[1] - c.pose[b][1]), bt)
    c.ang[a], c.ang[b] = ta % 360.0, tb % 360.0
    t0 = bt * c.spb
    if min(c.min_gap(a, t0), c.min_gap(b, t0), c.min_gap(third, t0)) < c.need:
        raise RuntimeError("the swap passes too near")
    c.turn_to(third, bearing(c.pose[third][:2], c.stage.O), bt)      # the third watches the middle


def _segs_cross(p1, p2, q1, q2):
    def orient(a, b, c_):
        return (b[0] - a[0]) * (c_[1] - a[1]) - (b[1] - a[1]) * (c_[0] - a[0])
    return orient(p1, p2, q1) * orient(p1, p2, q2) < 0 and orient(q1, q2, p1) * orient(q1, q2, p2) < 0


def crossing_options(c):
    """(first, second, first's ring angle to go to, second's) for the crossing, best first: the two chords cross
    near the middle at a good angle; every end spot and the third robot keep the pass distance."""
    O, D = c.stage.O, c.D
    out = []
    for x in ROLES:
        for y in ROLES:
            if y == x:
                continue
            z = next(r for r in ROLES if r not in (x, y))
            px, py, pz = c.pose[x][:2], c.pose[y][:2], c.pose[z][:2]
            for dx in (180.0, 165.0, 195.0, 150.0, 210.0):
                tx = (c.ang[x] + dx) % 360.0
                ex = _ring(c, tx)
                for ty in range(0, 360, 15):
                    ey = _ring(c, float(ty))
                    if not _segs_cross(px, ex, py, ey):
                        continue
                    if min(math.hypot(ex[0] - q[0], ex[1] - q[1]) for q in (py, pz, ey)) < D + 3.0:
                        continue
                    if min(math.hypot(ey[0] - q[0], ey[1] - q[1]) for q in (pz, ex)) < D + 3.0:
                        continue
                    if seg_dist(pz, px, ex) < D + 1.0 or seg_dist(pz, py, ey) < D + 1.0:
                        continue
                    if seg_dist(py, px, ex) < D + 1.0 or seg_dist(ex, py, ey) < D + 1.0:
                        continue
                    ang = abs(wrap180(bearing(px, ex) - bearing(py, ey)))
                    ang = min(ang, 180.0 - ang)
                    mid = min(seg_dist(O, px, ex), seg_dist(O, py, ey))
                    out.append((abs(90.0 - ang) + 0.5 * mid + 0.2 * abs(dx - 180.0), (x, y, tx, float(ty))))
    out.sort(key=lambda o: o[0])
    return [o[1] for o in out]


def fig_crossing(c, first, second, tx, ty):
    """first drives straight across (to ring angle tx); second, already aimed, cuts across its path just behind
    it (to ty) as soon as that is safe; the third watches."""
    ex, ey = _ring(c, tx), _ring(c, ty)
    bt = c.at((first, second))
    c.turn_to(first, bearing(c.pose[first][:2], ex), bt, aim=list(ex))
    c.turn_to(second, bearing(c.pose[second][:2], ey), bt, aim=list(ey))
    c.go(first, math.hypot(ex[0] - c.pose[first][0], ex[1] - c.pose[first][1]), c.ready(first), safe=True)
    c.figs[c.cur]["beat"] = c.moves[first][-1]["beat"]
    c.go(second, math.hypot(ey[0] - c.pose[second][0], ey[1] - c.pose[second][1]), c.ready(second), safe=True)
    c.ang[first], c.ang[second] = tx, ty
    third = next(r for r in ROLES if r not in (first, second))
    c.turn_to(third, bearing(c.pose[third][:2], c.stage.O))


def fig_orbit(c, mid, orbiters, direction, steps=2):
    """mid drives into the middle; the two others step round the ring (60 deg chords) together, mid turning with
    them. Needs the ring's chords to pass mid at the gap (rho cos 30 >= D)."""
    O = c.stage.O
    if c.rho * math.cos(math.radians(30.0)) < c.D + 0.5:
        raise RuntimeError("the ring is too small to orbit round a robot")
    a, b = orbiters
    sep = abs(wrap180(c.ang[a] - c.ang[b]))
    if 2.0 * c.rho * math.sin(math.radians(sep / 2.0)) < c.D + 2.0:
        raise RuntimeError("the orbiters are too close together")
    c.goto(mid, O, safe=True)
    for step in range(steps):
        bt = c.at((a, b))
        if step == 0:
            c.figs[c.cur]["beat"] = bt                   # the orbit, for the page, starts when they start circling
        for r in (a, b):
            p = _ring(c, c.ang[r] + 60.0 * direction)
            c.turn_to(r, bearing(c.pose[r][:2], p), bt, aim=list(p))
        bt = c.at((a, b))
        for r in (a, b):
            p = _ring(c, c.ang[r] + 60.0 * direction)
            c.go(r, math.hypot(p[0] - c.pose[r][0], p[1] - c.pose[r][1]), bt, safe=True)
            c.ang[r] = (c.ang[r] + 60.0 * direction) % 360.0
        c.turn(mid, 60.0 * direction, bt)
    c.mid = mid
    c.orbit_dir = direction


def fig_follow(c, direction, steps=2):
    """The robot in the middle goes back out to the ring where there is most room; then all three drive round the
    ring together (60 deg chords), each following the one in front."""
    mid = c.mid
    others = [r for r in ROLES if r != mid]
    best = None
    for ang in range(0, 360, 5):
        p = _ring(c, float(ang))
        room = min(math.hypot(p[0] - c.pose[r][0], p[1] - c.pose[r][1]) for r in others)
        if best is None or room > best[0]:
            best = (room, float(ang))
    if best[0] < c.D + 6.0:
        raise RuntimeError("no room on the ring")
    _ring_go(c, mid, best[1], safe=True)
    for step in range(steps):
        bt = c.at(ROLES)
        for r in ROLES:
            p = _ring(c, c.ang[r] + 60.0 * direction)
            c.turn_to(r, bearing(c.pose[r][:2], p), bt, aim=list(p))
        bt = c.at(ROLES)
        for r in ROLES:
            p = _ring(c, c.ang[r] + 60.0 * direction)
            c.go(r, math.hypot(p[0] - c.pose[r][0], p[1] - c.pose[r][1]), bt, safe=True)
            c.ang[r] = (c.ang[r] + 60.0 * direction) % 360.0


def fig_finale(c):
    """Straight in to a tight triangle round the middle (each pair the pass distance apart), a spin and a half
    together (3 half turns: it ends facing out), flashes on the beat, a bow (out and back)."""
    O = c.stage.O
    rf = (c.D + 0.2) / math.sqrt(3.0)
    order = sorted(ROLES, key=lambda r: c.ang[r])
    best = None
    for rot in range(0, 120, 2):                        # the triangle turned to suit where they stand
        dev = sum(abs(wrap180(c.ang[r] - (rot + 120.0 * k))) for k, r in enumerate(order))
        if best is None or dev < best[0]:
            best = (dev, float(rot))
    spots = {r: c.stage.polar(rf, best[1] + 120.0 * k) for k, r in enumerate(order)}
    bt = c.at(ROLES)
    for r in ROLES:
        c.goto(r, spots[r], bt, safe=True)
    bt = c.at(ROLES)
    for r in ROLES:
        c.turn(r, 540.0, bt)
    bt = c.at(ROLES)
    for r in ROLES:
        c.flash(r, bt)
    bt = c.at(ROLES)
    for r in ROLES:
        c.go(r, 5.0, bt)
    bt = c.at(ROLES)
    for r in ROLES:
        c.go(r, -5.0, bt)
    del O


def author_and_check(board, rule, speeds, bpm=120.0):
    """author_dance, checked; if the check finds a problem (the robust part, usually), again with a little more
    room, up to 6 cm more. Returns (dance, check result)."""
    last = None
    for margin in (0.0, 1.0, 2.0, 4.0, 6.0):
        try:
            dn = author_dance(board, rule, speeds, bpm, margin=margin)
        except RuntimeError as e:
            last = (None, dict(ok=False, problems=[str(e)]))
            continue
        chk = check(dn, speeds, board, rule)
        chk["margin_cm"] = margin
        if chk["ok"]:
            return dn, chk
        last = (dn, chk)
    return last


# =============================================================================== the checker
def _front_stop(board, x, y, h, rem, others):
    """Would the front sensor of a robot at (x, y, heading h deg), rem cm into... see something nearer than its
    stop limit? others: [(x, y)] body circles. Returns the distance it would see, or None."""
    lim = min(SAFE_FRONT_CM, rem + FRONT_END_CM)
    sx, sy = x + SENSOR_FWD_CM * math.cos(math.radians(h)), y + SENSOR_FWD_CM * math.sin(math.radians(h))
    best = None
    for a in np.linspace(-TOF_HALF_DEG, TOF_HALF_DEG, 7):
        dx, dy = math.cos(math.radians(h + a)), math.sin(math.radians(h + a))
        for ox, oy in others:
            fx, fy = ox - sx, oy - sy
            proj = fx * dx + fy * dy
            perp2 = fx * fx + fy * fy - proj * proj
            if proj > 0 and perp2 < R_BODY ** 2:
                d = proj - math.sqrt(R_BODY ** 2 - perp2)
                if d < lim and (best is None or d < best):
                    best = d
        # the blocked boards' faces
        for (axis, v) in (("y", board.xboard_cm), ("x", board.yboard_cm)):
            comp = dy if axis == "y" else dx
            pos = sy if axis == "y" else sx
            if abs(comp) > 1e-6:
                d = (v - pos) / comp
                if 0 < d < lim and (best is None or d < best):
                    best = d
    return best


def check(dance, speeds, board, rule, dt=0.05, robust_n=40, seed=5):
    """Simulate the commanded motion with each robot's speeds and check the dance (see the module doc). speeds:
    {role: Speeds}. Returns dict(ok, problems [words], closest {pair: dict(gap_cm, t, beat, figure)}, closest_pair,
    open_edge_cm, board_cm, front_risks, robust_gap_cm, duration_s, beats, n_moves)."""
    spb = dance.spb
    roles = dance.roles
    trs = {r: Traj(dance.start[r], dance.moves[r], speeds[r], spb) for r in roles}
    T = max([tr.t_end for tr in trs.values()] + [0.0])
    t = np.arange(-0.5, T + 1.0, dt)
    S = {r: trs[r].sample(t) for r in roles}
    problems = []
    gap = rule["gap_cm"]

    def fig_at(sec):
        b = sec / spb
        name = ""
        for f in dance.figures:
            if f.get("beat") is not None and f["beat"] <= b + 1e-9:
                name = f["name"]
        return name

    closest = {}
    for i, a in enumerate(roles):
        for b_ in roles[i + 1:]:
            g = np.hypot(S[a][0] - S[b_][0], S[a][1] - S[b_][1]) - 2.0 * R_BODY
            k = int(np.argmin(g))
            closest[(a, b_)] = dict(gap_cm=round(float(g[k]), 2), t=round(float(t[k]), 2), beat=round(float(t[k] / spb), 1),
                                    figure=fig_at(t[k]))
            if g[k] < gap - 1e-6:
                problems.append("%s and %s come %.1f cm apart (at %.1f s, %s): the gap is %.1f cm" % (
                    a, b_, g[k], t[k], fig_at(t[k]) or "start", gap))
    od_min, bd_min = np.inf, np.inf
    front = []
    for r in roles:
        od, bd = edge_clearance(board, S[r][0], S[r][1])
        k = int(np.argmin(od))
        od_min = min(od_min, float(od[k]))
        if od[k] < gap - 1e-6:
            problems.append("%s comes %.1f cm from an open edge (at %.1f s, %s): it must keep %.1f cm" % (
                r, od[k], t[k], fig_at(t[k]), gap))
        k = int(np.argmin(bd))
        bd_min = min(bd_min, float(bd[k]))
        if bd[k] < BOARD_CLEAR_CM - 1e-6:
            problems.append("%s comes %.1f cm from a blocked board (at %.1f s)" % (r, bd[k], t[k]))
        x, y, h, fwd, rem = S[r]
        for k in np.nonzero(fwd)[0][::2]:
            others = [(S[o][0][k], S[o][1][k]) for o in roles if o != r]
            d = _front_stop(board, x[k], y[k], h[k], rem[k], others)
            if d is not None:
                front.append(dict(role=r, t=round(float(t[k]), 2), seen_cm=round(d, 1)))
    if front:
        f0 = front[0]
        problems.append("%s's front sensor would see something %.1f cm ahead at %.1f s and stop it (%d moments)" % (
            f0["role"], f0["seen_cm"], f0["t"], len(front)))
    # timing: a move must not start before the previous ended (+ the stand-still the next leg needs)
    overlaps = []
    for r in roles:
        prev_end = None
        for m in dance.moves[r]:
            t0 = m["beat"] * spb
            if prev_end is not None and t0 < prev_end + SLACK_S - 0.05:
                overlaps.append((r, m["beat"]))
            prev_end = t0 + speeds[r].duration(m["kind"], m["value"])
            if m["kind"] == "go" and m["value"] < -BACK_MAX_CM:
                problems.append("%s backs up %.1f cm in one leg (at most %.1f: backing up is blind)" % (
                    r, -m["value"], BACK_MAX_CM))
    if overlaps:
        problems.append("%d moves start before the robot's previous move has ended (first: %s at beat %.1f)" % (
            len(overlaps), overlaps[0][0], overlaps[0][1]))
    flash_ok = abs((FLASH_HZ * spb) - round(FLASH_HZ * spb)) < 1e-6 and round(FLASH_HZ * spb) >= 1
    n_flash = sum(1 for r in roles for m in dance.moves[r] if m["kind"] == "flash")
    # robust: per-move start jitter and per-robot speed errors; the closest pair against the hard minimum
    rng = np.random.default_rng(seed)
    J, TOL = rule.get("jitter_s", JITTER_S), rule.get("speed_tol", SPEED_TOL)
    robust = np.inf
    worst = None
    for k in range(robust_n):
        tr2 = {}
        for r in roles:
            jit = {i: float(rng.uniform(-J, J)) for i in range(len(dance.moves[r]))}
            tr2[r] = Traj(dance.start[r], dance.moves[r], speeds[r], spb, jitter=jit,
                          speed_k=float(rng.uniform(1 - TOL, 1 + TOL)))
        T2 = max(tr.t_end for tr in tr2.values())
        tt = np.arange(-0.5, T2 + 1.0, 0.1)
        S2 = {r: tr2[r].sample(tt) for r in roles}
        for i, a in enumerate(roles):
            for b_ in roles[i + 1:]:
                g = np.hypot(S2[a][0] - S2[b_][0], S2[a][1] - S2[b_][1]) - 2.0 * R_BODY
                j = int(np.argmin(g))
                if g[j] < robust:
                    robust, worst = float(g[j]), (a, b_, float(tt[j]))
    if robust < rule["hard_cm"]:
        problems.append("with timing up to %.2f s off and speeds %d%% off, %s and %s could come %.1f cm apart (at "
                        "about %.0f s): below the %.1f cm hard minimum" % (J, round(TOL * 100), worst[0],
                                                                         worst[1], robust, worst[2], rule["hard_cm"]))
    cp = min(closest.items(), key=lambda kv: kv[1]["gap_cm"]) if closest else None
    return dict(ok=not problems, problems=problems, closest={"%s-%s" % k: v for k, v in closest.items()},
                closest_pair=("%s-%s" % cp[0], cp[1]) if cp else None, open_edge_cm=round(od_min, 2),
                board_cm=round(bd_min, 2), front_risks=front[:20], robust_gap_cm=round(robust, 2),
                robust_worst=worst, robust_jitter_s=J, robust_speed_tol=TOL, duration_s=round(T, 1), beats=round(T / spb, 1),
                n_moves=sum(len(dance.moves[r]) for r in roles), n_flash=n_flash, flash_on_beat=flash_ok)


def summary_words(dance, chk, rule, robot_of=None, speeds=None):
    """Plain lines for the console and the page."""
    name = (lambda r: robot_of.get(r, r)) if robot_of else (lambda r: r)
    lines = []
    lines.append("The dance: %.0f s at %d beats a minute (%d beats after a %d-beat count-in), %d moves for %s." % (
        chk["duration_s"], round(dance.bpm), round(chk["beats"]), dance.count_in, chk["n_moves"],
        ", ".join("%s (%s)" % (name(r), r) for r in dance.roles)))
    for i, f in enumerate(dance.figures):
        if f.get("beat") is None:
            continue
        lines.append("  %d. %s, from %0.0f s: %s" % (i + 1, f["name"], f["beat"] * dance.spb, f["words"]))
    lines.append(rule["words"])
    if chk.get("closest_pair"):
        pr, c = chk["closest_pair"]
        a, b = pr.split("-")
        lines.append("Closest the plan comes: %.1f cm between %s and %s, at %.0f s (%s). Open edges: %.1f cm at the "
                     "nearest; blocked boards: %.1f cm." % (c["gap_cm"], name(a), name(b), c["t"], c["figure"],
                                                            chk["open_edge_cm"], chk["board_cm"]))
    lines.append("With every move up to %.2f s early or late and speeds %d%% off, the closest any pair could come is "
                 "%.1f cm (the supervisor stops everything below %.1f cm)." % (
                     chk.get("robust_jitter_s", JITTER_S), round(chk.get("robust_speed_tol", SPEED_TOL) * 100),
                     chk["robust_gap_cm"], rule["hard_cm"]))
    if chk["n_flash"]:
        lines.append("Lights: %d flashes, %s." % (
            chk["n_flash"], "on the beat (the firmware's `identify` flashes twice a second, one per beat at this "
                            "tempo)" if chk["flash_on_beat"] else "NOT on the beat at this tempo (`identify` flashes "
                                                                  "twice a second): they are left out"))
    if speeds:
        for r in dance.roles:
            lines.append("  %s %s" % (name(r), speeds[r].words()))
    for p in chk.get("problems", []):
        lines.append("PROBLEM: " + p)
    return lines


# =============================================================================== running it
def assign_roles(formation, positions):
    """{role: robot}: who goes to which start spot, the least driving in all. formation {role: (x, y, h)},
    positions {robot: (x, y)}."""
    import itertools
    roles, robots = list(formation), list(positions)
    if len(robots) <= 7:
        best = None
        for perm in itertools.permutations(robots, len(roles)):
            cost = sum(math.hypot(formation[r][0] - positions[n][0], formation[r][1] - positions[n][1])
                       for r, n in zip(roles, perm))
            if best is None or cost < best[0]:
                best = (cost, perm)
        return dict(zip(roles, best[1]))
    from scipy.optimize import linear_sum_assignment
    M = np.array([[math.hypot(formation[r][0] - positions[n][0], formation[r][1] - positions[n][1]) for n in robots]
                  for r in roles])
    ri, ci = linear_sum_assignment(M)
    return {roles[i]: robots[j] for i, j in zip(ri, ci)}


def corrected_command(m, pos, heading, nxt=None, max_turn_fix=20.0):
    """The leg to send for planned move m, from where the camera sees the robot (pos, or None) and the heading the
    runner believes (or None): turns aim at their go's planned end (so a sideways error is taken out by the turn),
    goes drive to their planned end along the robot's real heading. Corrections are capped (a bad camera reading
    can't make a wild leg). Returns (value, words)."""
    v = float(m["value"])
    if m["kind"] == "turn":
        if heading is None:
            return v, "nominal (heading unknown)"
        last_piece = not m.get("piece") or m["piece"][0] == m["piece"][1]
        if not last_piece:
            return v, "nominal (part of a longer turn)"
        target = m["h1"]
        if m.get("aim") and pos is not None and math.hypot(m["aim"][0] - pos[0], m["aim"][1] - pos[1]) > 8.0:
            target = bearing(pos, m["aim"])
        pieces_before = (m["piece"][0] - 1) * v if m.get("piece") else 0.0
        fix = wrap180(target - (heading + v))
        del pieces_before
        fix = max(-max_turn_fix, min(max_turn_fix, fix))
        cmd = max(-179.0, min(179.0, v + fix))          # the firmware refuses turns beyond 180
        return cmd, ("aimed: %+.1f deg" % (cmd - v)) if abs(cmd - v) >= 0.5 else "as planned"
    if m["kind"] == "go":
        if pos is None or heading is None:
            return v, "nominal (no camera position or heading)"
        u = (math.cos(math.radians(heading)), math.sin(math.radians(heading)))
        along = (m["x1"] - pos[0]) * u[0] + (m["y1"] - pos[1]) * u[1]
        cap = max(4.0, 0.25 * abs(v))
        cmd = v + max(-cap, min(cap, along - v))
        if v < 0:
            cmd = max(-BACK_MAX_CM, min(-0.5, cmd))
        elif v > 0:
            cmd = max(0.5, cmd)
        return cmd, ("%+.1f cm" % (cmd - v)) if abs(cmd - v) >= 0.3 else "as planned"
    return v, ""


# =============================================================================== music
def make_music(path, bpm, n_beats, count_in=COUNT_IN_BEATS, accents=(), flashes=(), sr=22050, seed=7):
    """A royalty-free beat (made here, nothing sampled): count_in beats of sticks, then n_beats of drums (kick on 1
    and 3, snare on 2 and 4, eighth-note hats), an eighth-note bass and soft chords over A minor - F - C - G, a crash
    on every beat in `accents` (dance beats: the figures' starts), a bell on every beat in `flashes`; two bars to end.
    Beat k of the file sounds at k x 60 / bpm s (k = count_in is the dance's beat 0). Mono 16-bit WAV."""
    rng = np.random.default_rng(seed)
    spb = 60.0 / bpm
    total = count_in + int(math.ceil(n_beats / 4.0)) * 4 + 8
    N = int(total * spb * sr) + sr
    out = np.zeros(N, np.float32)

    def add(sig, t, gain=1.0):
        i = int(round(t * sr))
        if i >= N:
            return
        j = min(N, i + len(sig))
        out[i:j] += gain * sig[:j - i]

    def env(n, tau):
        return np.exp(-np.arange(n) / (tau * sr)).astype(np.float32)

    tt = lambda s: np.arange(int(s * sr)) / sr  # noqa: E731
    k_t = tt(0.25)
    f = 45.0 + 110.0 * np.exp(-k_t / 0.03)
    kick = (np.sin(2 * np.pi * np.cumsum(f) / sr) * env(len(k_t), 0.09)).astype(np.float32)
    nz = rng.normal(0, 1, int(0.2 * sr)).astype(np.float32)
    snare = (np.diff(nz, prepend=0.0) * 0.5 * env(len(nz), 0.05) +
             np.sin(2 * np.pi * 185 * tt(0.2)) * env(len(nz), 0.04) * 0.5).astype(np.float32)
    hz = rng.normal(0, 1, int(0.06 * sr)).astype(np.float32)
    hat = (np.diff(np.diff(hz, prepend=0.0), prepend=0.0) * 0.25 * env(len(hz), 0.012)).astype(np.float32)
    oh = rng.normal(0, 1, int(0.3 * sr)).astype(np.float32)
    ohat = (np.diff(np.diff(oh, prepend=0.0), prepend=0.0) * 0.2 * env(len(oh), 0.08)).astype(np.float32)
    cz = rng.normal(0, 1, int(1.6 * sr)).astype(np.float32)
    crash = (np.diff(cz, prepend=0.0) * 0.35 * env(len(cz), 0.45)).astype(np.float32)
    stick_t = tt(0.04)
    stick = (np.sin(2 * np.pi * 1600 * stick_t) * env(len(stick_t), 0.008)).astype(np.float32)
    bell_t = tt(0.9)
    bell = ((np.sin(2 * np.pi * 1318.5 * bell_t) + 0.5 * np.sin(2 * np.pi * 2637 * bell_t) +
             0.25 * np.sin(2 * np.pi * 3955 * bell_t)) * env(len(bell_t), 0.25) * 0.18).astype(np.float32)

    def tone(freq, dur, harm=(1.0, 0.5, 0.33, 0.25), tau=0.15, attack=0.01):
        x = tt(dur)
        s = sum(a * np.sin(2 * np.pi * freq * (i + 1) * x) for i, a in enumerate(harm))
        e = np.minimum(1.0, x / attack) * np.exp(-x / tau)
        return (s * e).astype(np.float32)

    roots = (110.0, 87.31, 130.81, 98.0)              # A2 F2 C3 G2
    chords = ((220.0, 261.63, 329.63), (174.61, 220.0, 261.63), (196.0, 261.63, 329.63), (196.0, 246.94, 293.66))
    for k in range(count_in):                         # the count-in: sticks, the bar's first a little louder
        add(stick, k * spb, 0.9 if k % 4 == 0 else 0.6)
        if k >= count_in - 4:
            add(hat, k * spb, 0.8)
    acc = set(int(round(a)) for a in accents)
    fl = set(int(round(a)) for a in flashes)
    n_play = total - count_in - 4
    for j in range(n_play):
        t = (count_in + j) * spb
        bar, beat = divmod(j, 4)
        ch = bar % 4
        if beat in (0, 2):
            add(kick, t, 0.9)
        if beat == 2:
            add(kick, t + spb * 0.5, 0.5)
        if beat in (1, 3):
            add(snare, t, 0.55)
        add(hat, t, 0.5)
        add(hat if beat != 3 else ohat, t + spb * 0.5, 0.35)
        for e8 in (0.0, 0.5):
            add(tone(roots[ch] * (2.0 if (beat == 3 and e8) else 1.0), spb * 0.45, tau=0.12), t + e8 * spb, 0.22)
        if beat == 0:
            for fq in chords[ch]:
                add(tone(fq, spb * 3.8, harm=(1.0, 0.2), tau=1.4, attack=0.15), t, 0.06)
        if j in acc:
            add(crash, t, 0.7)
        if j in fl:
            add(bell, t, 0.8)
    t_end = (total - 4) * spb                         # the ending: one last chord and a crash
    add(crash, t_end, 0.8)
    add(kick, t_end, 1.0)
    for fq in chords[0]:
        add(tone(fq, spb * 4, harm=(1.0, 0.2), tau=1.5, attack=0.02), t_end, 0.08)
    peak = float(np.max(np.abs(out))) or 1.0
    pcm = (np.clip(out / peak * 0.85, -1, 1) * 32767).astype(np.int16)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(pcm.tobytes())
    return dict(path=path, seconds=round(len(pcm) / sr, 2), beats=total, bpm=bpm, count_in=count_in)


# =============================================================================== the live page
DANCE_PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Robot dance</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0e1116;--card:#171b22;--fg:#e8eaed;--dim:#9aa0a6;--ok:#3ccf8e;--bad:#e06c75;--warn:#e5c07b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
header{display:flex;flex-wrap:wrap;align-items:center;gap:12px;padding:14px 16px 0}
h1{font-size:20px;margin:0}#phase{font-size:15px;color:var(--dim)}
button{background:#2b6cb0;color:#fff;border:0;border-radius:8px;padding:9px 14px;font-size:15px;cursor:pointer}
button.off{background:#333}
main{display:grid;grid-template-columns:minmax(0,3fr) minmax(260px,1fr);gap:14px;padding:14px 16px}
@media (max-width:820px){main{grid-template-columns:1fr}}
.card{background:var(--card);border-radius:10px;padding:12px}
.pic{position:relative;width:100%}.pic img{width:100%;display:block;border-radius:6px}
.pic svg{position:absolute;left:0;top:0;width:100%;height:100%}
.note{color:var(--dim);font-size:13px;margin:8px 0 0}
.big{font-size:32px;font-variant-numeric:tabular-nums;margin:4px 0}
.row{display:flex;justify-content:space-between;margin:6px 0}.row b{font-variant-numeric:tabular-nums}
#beat{width:18px;height:18px;border-radius:50%;background:#333;display:inline-block;vertical-align:middle;transition:background .08s}
#stop{color:var(--bad);font-weight:600}
</style></head><body>
<header><h1>Robot dance</h1><span id="beat"></span><span id="phase">waiting for the dance</span>
<button id="snd">Start the sound</button></header>
<main>
<div class="card"><div class="pic"><img id="pic" src="/frame.jpg" alt="overhead camera"><svg id="ov" preserveAspectRatio="none"></svg></div>
<p class="note">Dashed lines: where each robot is meant to drive; solid lines: where the camera saw it go; a ring round a
robot that pulses is its lights flashing; a grey ring means the camera is not sure where that robot is.</p></div>
<div class="card">
<div class="note">Now</div><div class="big" id="fig">-</div><div id="words" class="note"></div>
<div class="row"><span>Closest pair right now</span><b id="gapnow">-</b></div>
<div class="row"><span>Closest the plan ever comes</span><b id="gapplan">-</b></div>
<div class="row"><span>Gap the plan keeps</span><b id="gaprule">-</b></div>
<div class="row"><span>Safety stop below</span><b id="hard">-</b></div>
<p id="stop"></p>
<p class="note" id="rule"></p>
<p class="note">The music is made by the program at the dance's tempo. The robots and the music run on the laptop's
clock: this page measures the difference between its own clock and the laptop's and starts the music so its beats
fall when the robots' moves do.</p>
</div></main>
<script>
let off=0,best=1e9,st=null,ctx=null,buf=null,src=null,srcAt=0,srcPos=0,wantSound=false,lastBeat=-1;
const $=id=>document.getElementById(id);
async function sync(){try{const a=performance.now();const r=await fetch('/dance/clock.json?'+Date.now());const j=await r.json();
 const b=performance.now();const rtt=b-a;if(rtt<=best*1.5+2){off=j.t*1000-(performance.timeOrigin+(a+b)/2);best=Math.min(best,rtt)}}catch(e){}}
function now(){return (performance.timeOrigin+performance.now()+off)/1000}
async function loadSound(){if(buf||!st||!st.music)return;const r=await fetch(st.music);const ab=await r.arrayBuffer();buf=await ctx.decodeAudioData(ab)}
function lat(){return ctx?(ctx.outputLatency||ctx.baseLatency||0):0}
function stopSound(){if(src){try{src.stop()}catch(e){}src=null}}
function startSound(){if(!ctx||!buf||!st||!st.music0)return;stopSound();let pos=now()-st.music0+lat();
 if(pos>=buf.duration)return;src=ctx.createBufferSource();src.buffer=buf;src.connect(ctx.destination);
 if(pos<0){src.start(ctx.currentTime-pos,0);srcAt=ctx.currentTime-pos;srcPos=0}else{src.start(0,pos);srcAt=ctx.currentTime;srcPos=pos}}
function checkDrift(){if(!src||!st||!st.music0)return;const exp=now()-st.music0+lat();const act=ctx.currentTime-srcAt+srcPos;
 if(ctx.currentTime>=srcAt&&Math.abs(exp-act)>0.05)startSound()}
$('snd').onclick=async()=>{wantSound=!wantSound;$('snd').textContent=wantSound?'Sound on':'Start the sound';$('snd').className=wantSound?'off':'';
 if(wantSound){ctx=ctx||new (window.AudioContext||window.webkitAudioContext)();await ctx.resume();await loadSound();startSound()}else stopSound()};
function el(t,a){const e=document.createElementNS('http://www.w3.org/2000/svg',t);for(const k in a)e.setAttribute(k,a[k]);return e}
function draw(){const s=st;if(!s||!s.img)return;const ov=$('ov');ov.setAttribute('viewBox','0 0 '+s.img[0]+' '+s.img[1]);ov.innerHTML='';
 for(const [n,p] of Object.entries(s.planned||{})){if(p.pts.length>1)ov.appendChild(el('polyline',{points:p.pts.map(q=>q.join(',')).join(' '),fill:'none',stroke:p.color,'stroke-width':2,'stroke-dasharray':'6 5',opacity:.75}))}
 for(const [n,p] of Object.entries(s.actual||{})){if(p.pts.length>1)ov.appendChild(el('polyline',{points:p.pts.map(q=>q.join(',')).join(' '),fill:'none',stroke:p.color,'stroke-width':3}))}
 const t=now();for(const [n,r] of Object.entries(s.robots||{})){const sure=r.status=='seen';
  const flash=r.flash_until&&t<r.flash_until&&((t-r.flash_from)%0.5)<0.25;
  ov.appendChild(el('circle',{cx:r.u,cy:r.v,r:r.rpx*(flash?1.25:0.9),fill:flash?'rgba(255,255,255,.35)':'none',stroke:sure?r.color:'#888','stroke-width':sure?3:2,'stroke-dasharray':sure?'':'4 4'}));
  const tx=el('text',{x:r.u+r.rpx+4,y:r.v-r.rpx,fill:r.color,'font-size':16,'font-weight':600});tx.textContent=n;ov.appendChild(tx)}}
function beatDot(){if(!st||!st.t0)return;const b=(now()-st.t0)/st.spb;const k=Math.floor(b);
 if(k!==lastBeat){lastBeat=k;$('beat').style.background=(k%4===0)?'#fff':'#5aa9ff';setTimeout(()=>{$('beat').style.background='#333'},110)}}
async function tick(){try{const r=await fetch('/dance/state.json?'+Date.now());st=await r.json();
 const t=now();let ph=st.phase_words||st.phase||'';
 if(st.t0&&st.phase=='dancing'){const b=(t-st.t0)/st.spb;ph=b<0?('starting in '+Math.ceil(-b*st.spb)+' s'):('beat '+Math.floor(b)+' of '+Math.round(st.beats))}
 $('phase').textContent=ph;$('fig').textContent=st.figure||'-';$('words').textContent=st.figure_words||'';
 $('gapnow').textContent=st.gap_now!=null?st.gap_now.toFixed(1)+' cm':'-';$('gapplan').textContent=st.gap_plan!=null?st.gap_plan.toFixed(1)+' cm':'-';
 $('gaprule').textContent=st.gap_rule!=null?st.gap_rule.toFixed(1)+' cm':'-';$('hard').textContent=st.hard!=null?st.hard.toFixed(1)+' cm':'-';
 $('stop').textContent=st.stop||'';$('rule').textContent=st.rule_words||'';draw();
 if(wantSound&&ctx){await loadSound();if(!src&&st.phase=='dancing')startSound();if(st.phase=='stopped')stopSound()}}catch(e){}}
function pic(){const im=new Image();im.onload=()=>{$('pic').src=im.src};im.src='/frame.jpg?t='+Date.now()}
sync();setInterval(sync,3000);setInterval(tick,300);setInterval(pic,250);setInterval(beatDot,30);setInterval(checkDrift,2000);
</script></body></html>"""


class DancePage:
    """Adds the dance pages to a running e02_view.LiveView without changing it: /dance, /dance/state.json,
    /dance/clock.json, /dance/music.wav."""

    def __init__(self, view, run):
        self.view, self.run = view, run
        self.old = view.extra_get

        def extra_get(path):
            p = path.split("?")[0]
            if p == "/dance":
                return DANCE_PAGE.encode(), "text/html; charset=utf-8"
            if p == "/dance/state.json":
                return json.dumps(run.page_state()).encode(), "application/json"
            if p == "/dance/clock.json":
                return json.dumps(dict(t=time.time())).encode(), "application/json"
            if p == "/dance/music.wav" and run.music_path and os.path.exists(run.music_path):
                with open(run.music_path, "rb") as f:
                    return f.read(), "audio/wav"
            return self.old(path)
        view.extra_get = extra_get

    def detach(self):
        self.view.extra_get = self.old


# =============================================================================== the execution
class DanceRun:
    """Runs a dance on the real (or simulated) robots from inside an e02 Runner (see the module doc)."""

    FINAL = ("done", "failed", "refused", "dup")

    def __init__(self, runner, bpm=120.0):
        self.r = runner
        self.E = sys.modules[type(runner).__module__]
        self.bpm = float(bpm)
        self.lock = threading.RLock()
        self.abort = threading.Event()
        self.quit = threading.Event()
        self.phase, self.phase_words = "planning", "planning the dance"
        self.stops, self.notes = [], []
        self.legs = {}                      # leg id -> dict
        self.last_leg = {}                  # robot -> leg id
        self.rtt = defaultdict(lambda: deque(maxlen=20))
        self.T0 = self.music0 = None
        self.mt = None
        self.dance = None
        self.robot_of, self.role_of = {}, {}
        self.head = {}
        self.music_path = None
        self.chk = None
        self.rule = None
        self.speeds = {}
        self.flashes = []                   # (robot, t_from)
        self.page = None
        self.start = {}
        self._leg_i = 0
        self.gap_now = None
        self.planned_px = {}

    def say(self, msg, quiet=False):
        self.r.say(msg, quiet=quiet)

    # --- timing helpers ----------------------------------------------------------------------------------
    def one_way(self, robot):
        v = list(self.rtt[robot])
        return 0.5 * float(np.median(v)) if v else 0.03

    def lead(self, robot):
        """How long before a move's beat its leg is sent: the radio one way + the firmware's stand-still."""
        return self.one_way(robot) + LEARN_S

    # --- the whole thing ---------------------------------------------------------------------------------
    def run(self):
        r = self.r
        st = r.st
        names = [n for n in st["robots"] if n not in r.unavailable and st["positions"].get(n)]
        print("")
        self.say("=== The dance ===")
        if len(names) != 3:
            self.say("The dance is written for three robots; %d are on the board and ready (%s). No dance this time." % (
                len(names), ", ".join(names) or "none"))
            return None
        r.phase, r.order, r.active = "dance", "dance", None
        # 1. the precision rule; the start spots (they depend on the board and the gap only)
        self.rule = load_precision(os.path.join(REPO, "logs"), r.a.dry_run)
        self.say("  " + self.rule["words"])
        form = start_formation(r.board, self.rule)
        pos = {n: tuple(st["positions"][n][:2]) for n in names}
        self.robot_of = assign_roles(form, pos)
        self.role_of = {v: k for k, v in self.robot_of.items()}
        self.start = form
        if getattr(r, "view", None) is not None:
            self.page = DancePage(r.view, self)
            self.say("  Watch and listen: http://localhost:%d/dance (press 'Start the sound' once)." % r.a.view_port)
        self.phase, self.phase_words = "starting", "the robots are going to their start spots"
        if not self.to_start():
            self.say("The robots could not all get to their start spots: no dance this time; parking them.")
            self.phase, self.phase_words = "done", "no dance (start spots)"
            self.park_all(names)
            return None
        # 2. how fast each robot drives and turns: every leg of this session so far (setup, the drives to the start)
        with r.listener.cv:
            recs = list(r.listener.recs["LEG"])
        self.speeds = fit_all(recs, names)
        for n in names:
            self.say("  %s %s" % (n, self.speeds[n].words()))
        # 3. the plan for these robots, and its check
        sp_role = {role: self.speeds[n] for role, n in self.robot_of.items()}
        dn, chk = author_and_check(r.board, self.rule, sp_role, self.bpm)
        if dn is None or not chk["ok"]:
            for p in chk.get("problems", []):
                self.say("  PROBLEM: " + p)
            self.say("The dance did not pass its check (above): no dance this time; parking them.")
            self.phase, self.phase_words = "done", "no dance (the check)"
            self.park_all(names)
            return None
        self.dance, self.chk = dn, chk
        for line in summary_words(dn, chk, self.rule, self.robot_of, sp_role):
            self.say(line)
        flashes = [m["beat"] for role in dn.roles for m in dn.moves[role] if m["kind"] == "flash"] if chk["flash_on_beat"] else []
        self.music_path = os.path.join(r.dir, "dance_music.wav")
        mus = make_music(self.music_path, self.bpm, chk["beats"] + 2, dn.count_in,
                         accents=[f["beat"] for f in dn.figures if f.get("beat") is not None], flashes=flashes)
        with open(os.path.join(r.dir, "dance_plan.json"), "w") as f:
            json.dump(dict(dance=dn.to_dict(), check=_jsonable(chk), robot_of=self.robot_of,
                           speeds={n: s.to_dict() for n, s in self.speeds.items()}, music=mus), f, indent=1)
        r.mark("e02 dance plan %.0f s" % chk["duration_s"])
        # 4. every robot followed at once from here
        if not self.start_tracking(names):
            self.say("The camera could not see all three robots at their start spots: no dance this time.")
            self.restore_tracking(names)
            self.park_all(names)
            return None
        out = None
        try:
            self.perform(names)
        finally:
            self.quit.set()
            time.sleep(0.3)
            out = self.results(names)
            self.restore_tracking(names)
        self.park_all(names)
        return out

    def to_start(self):
        """Each robot camera-steered to its start pose, one at a time, going round the others; a robot whose start
        spot another robot stands on waits until that one has left."""
        r = self.r
        todo = list(self.robot_of.items())
        tries = defaultdict(int)
        while todo:
            def blocked(item):
                role, n = item
                x, y, _ = self.start[role]
                return any(math.hypot(x - q[0], y - q[1]) < 2 * R_BODY + 8.0
                           for m, q in r.st["positions"].items() if m != n and q)
            free = [it for it in todo if not blocked(it)]
            if not free:
                self.say("  Every start spot has another robot on or next to it.")
                return False
            role, n = free[0]
            x, y, h = self.start[role]
            self.say("  %s (%s) to its start spot (%.0f, %.0f) cm, facing %.0f deg ..." % (n, role, x, y, h))
            if not r.activate(n):
                p = r.identify(n)
                r.moves.pop(n, None)
                if p is None or r.acquire_heading(n, p) is None:
                    return False
            r.set_corrections(n, r.move_corrections(n))
            ok, why = r.drive_to(n, (x, y), final_heading=h, arrive_cm=2.0, max_leg=self.E.PARK_LEG_CM)
            r.end_moves(n)
            p = r.settle(min_s=0.3)
            if p is not None:
                r.st["positions"][n] = [p[0], p[1], r.heads.get(n, [None])[0]]
                r.tracker.park(n, p[0], p[1])
            if not ok:
                tries[n] += 1
                self.say("  %s did not get there: %s" % (n, why))
                if tries[n] >= 2:
                    return False
                continue
            todo.remove((role, n))
        r.st.save()
        r.active = None
        return True

    def start_tracking(self, names):
        import e02_multitrack as MT
        r = self.r
        r.tracker.idle()
        frames = r.feed.wait_frames(10, timeout=8)
        if len(frames) < 3:
            return False
        mt = MT.MultiTracker(r.cam, board_rect=r.board_rect())
        pos = {n: r.st["positions"][n] for n in names}
        mt.set_background(frames, robots=[tuple(q[:2]) for q in pos.values()])
        for n in names:
            h = r.heads.get(n, [None])[0]
            mt.add(n, pos[n][0], pos[n][1], heading=h)
            self.head[n] = [h, bool(r.heads.get(n, [None, False])[1])]
        self.saved_tracker = r.feed.tracker
        r.feed.tracker = None                       # the one-robot tracker rests while the multi-tracker works
        r.feed.hooks.append(mt.push)
        self.mt = mt
        time.sleep(1.5)
        bad = [n for n in names if mt.state(n)["status"] != "seen"]
        if bad:
            self.say("  The camera is not sure of %s at the start." % ", ".join(bad))
            return False
        self.say("  The camera follows all three at once now.")
        return True

    def restore_tracking(self, names):
        r = self.r
        if self.mt is not None:
            try:
                r.feed.hooks.remove(self.mt.push)
            except ValueError:
                pass
            for n in names:
                s = self.mt.state(n)
                if s is not None:
                    r.st["positions"][n] = [s["sure_x"], s["sure_y"], (self.head.get(n) or [None])[0]]
                    r.heads[n] = [(self.head.get(n) or [None])[0], False]
            r.st.save()
        if getattr(self, "saved_tracker", None) is not None:
            r.feed.tracker = self.saved_tracker
        try:
            r.fresh_floor()
        except Exception as e:
            self.say("  (fresh floor picture failed: %s)" % e, quiet=True)

    def park_all(self, names):
        r = self.r
        self.phase, self.phase_words = "parking", "parking"
        try:
            r.park_all([n for n in names if n in r.st["positions"]])
        finally:
            self.phase, self.phase_words = "done", "done"
            if self.page is not None:
                pass

    # --- the performance -----------------------------------------------------------------------------------
    def perform(self, names):
        r = self.r
        dn = self.dance
        spb = dn.spb
        with r.listener.cv:
            self._leg_i = len(r.listener.recs["LEG"])
        for n in names:
            r.set_corrections(n, r.move_corrections(n))
        for n in names:
            self.send_fix(n)
        # the planned paths on the picture
        self.planned_px = self.planned_paths()
        lead_in = 2.0
        self.T0 = time.time() + lead_in + dn.count_in * spb
        self.music0 = self.T0 - dn.count_in * spb
        self.phase, self.phase_words = "dancing", "dancing"
        r.mark("e02 dance start")
        self.say("  The music starts in %.0f s; the robots %.0f s later (count-in: %d beats)." % (
            lead_in, dn.count_in * spb, dn.count_in))
        events = []
        for role, n in self.robot_of.items():
            for i, m in enumerate(dn.moves[role]):
                if m["kind"] == "flash" and not self.chk["flash_on_beat"]:
                    continue
                t_beat = self.T0 + m["beat"] * spb
                events.append((t_beat - (self.lead(n) if m["kind"] != "flash" else self.one_way(n)), n, role, i))
        events.sort()
        sup = threading.Thread(target=self.supervisor, daemon=True)
        pump = threading.Thread(target=self.pump, daemon=True)
        sup.start()
        pump.start()
        for k, (t_send, n, role, i) in enumerate(events):
            # recompute: the radio delay is measured as it goes
            m = dn.moves[role][i]
            t_beat = self.T0 + m["beat"] * spb
            t_send = t_beat - (self.lead(n) if m["kind"] != "flash" else self.one_way(n))
            while not self.abort.is_set() and time.time() < t_send:
                time.sleep(min(0.01, max(0.0, t_send - time.time())))
            if self.abort.is_set():
                break
            try:
                self.do_move(n, role, i, m, t_beat)
            except Exception as e:                     # one move's trouble must not leave robots running
                self.say("  (dance: %s's move %d: %s)" % (n, i, e))
        # wait for the last legs to end
        end = time.time() + 12.0
        while time.time() < end and not self.abort.is_set():
            with self.lock:
                open_ = [l for l in self.legs.values() if l["state"] not in self.FINAL]
            if not open_:
                break
            time.sleep(0.1)
        self.T_end = time.time()
        if self.abort.is_set():
            self.phase, self.phase_words = "stopped", "stopped by the safety supervisor"
        else:
            self.phase, self.phase_words = "finished", "finished"
        r.mark("e02 dance end %s" % ("stopped" if self.abort.is_set() else "done"))

    def send_fix(self, n):
        """Tell a robot where the camera sees it (it stands still): `fix x y th|- t_wall`. Also resets the
        firmware's blind-backing allowance."""
        s = self.mt.state(n)
        if s is None or s["status"] != "seen":
            return False
        sp = self.mt.still_at(n, time.time() - 0.6, min_s=0.3)
        if sp is None:
            return False
        h = self.head.get(n, [None])[0]
        self.r.send(n, "fix %.2f %.2f %s %.3f" % (sp[0], sp[1], "-" if h is None else "%.1f" % h, time.time()))
        return True

    def do_move(self, n, role, i, m, t_beat):
        r = self.r
        with self.lock:
            prev = self.legs.get(self.last_leg.get(n))
        late = prev is not None and prev["state"] not in self.FINAL
        if m["kind"] == "flash":
            if late:
                self.notes.append("%s's flash at beat %.1f left out: it was still moving" % (n, m["beat"]))
                return
            r.send(n, "identify")
            with self.lock:
                self.flashes.append((n, t_beat))
            return
        s = self.mt.state(n)
        pos = (s["x"], s["y"]) if (s and s["status"] == "seen" and not late) else None
        h = self.head.get(n, [None])[0]
        nxt = self.dance.moves[role][i + 1] if i + 1 < len(self.dance.moves[role]) else None
        value, how = corrected_command(m, pos, h, nxt)
        if pos is not None:                             # it stands still (its last leg is done): tell it where it is
            r.send(n, "fix %.2f %.2f %s %.3f" % (pos[0], pos[1], "-" if h is None else "%.1f" % h, s["t_seen"]))
        lid = r.next_id(n)
        t_send = time.time()
        r.send(n, "%s %.2f %d" % (m["kind"], value, lid))
        sp = self.speeds[n]
        t_motion = (prev["t_end_est"] + LEARN_S) if late and prev.get("t_end_est") else t_send + self.lead(n)
        dur = sp.duration(m["kind"], value)
        with self.lock:
            self.legs[lid] = dict(id=lid, robot=n, role=role, i=i, kind=m["kind"], value=value, planned=m["value"],
                                  how=how, t_beat=t_beat, t_send=t_send, state="sent", resent=False, late=late,
                                  t_motion_est=t_motion, t_end_est=t_motion + dur, heading=h)
            self.last_leg[n] = lid
        self.mt.command(n, m["kind"], value, t_motion, dur, heading=h, ramp=sp.ramp(m["kind"]))

    def pump(self):
        """LEG records -> leg states, the radio delay, the multi-tracker's motion model, headings; resends a leg
        nobody answered (same id: the firmware answers `dup` to a repeat)."""
        r = self.r
        while not self.quit.is_set():
            time.sleep(0.03)
            with r.listener.cv:
                recs = r.listener.recs["LEG"][self._leg_i:]
                self._leg_i += len(recs)
            now = time.time()
            for t, n, rec in recs:
                try:
                    lid = int(rec.get("id"))
                except (TypeError, ValueError):
                    continue
                with self.lock:
                    lg = self.legs.get(lid)
                if lg is None or lg["robot"] != n:
                    continue
                state = rec.get("state") or rec.get("result")
                xy = (_num(rec.get("x_cm")), _num(rec.get("y_cm")), _num(rec.get("th_deg")))
                if state == "queued" and "t_queued" not in lg:
                    lg["t_queued"] = t
                    self.rtt[n].append(t - lg["t_send"])
                elif state == "started":
                    lg["t_started"] = t
                    lg["pose_started"] = xy
                    t_mot = t - self.one_way(n) + LEARN_S
                    lg["t_motion_robot"] = t_mot
                    lg["t_end_est"] = t_mot + self.speeds[n].duration(lg["kind"], lg["value"])
                    self.mt.started(n, t_mot)
                if state in self.FINAL:
                    lg["state"] = state
                    lg["info"] = rec.get("info") or ""
                    lg["t_done"] = t
                    lg["pose_done"] = xy
                    dxy = None
                    ps = lg.get("pose_started")
                    if ps and None not in ps and None not in xy:
                        dxy = (xy[0] - ps[0], xy[1] - ps[1])
                    self.mt.leg_done(n, t - self.one_way(n), *(dxy or (None, None)))
                    self.after_leg(n, lg)
                    if state == "failed" and not self.abort.is_set():
                        self.stop_all("%s's own safety stopped its leg (%s)" % (n, lg["info"] or "failed"))
                    elif state == "refused" and not self.abort.is_set():
                        self.notes.append("%s refused a leg: %s" % (n, lg["info"]))
                else:
                    lg["state"] = state
            # resend what got no answer at all
            with self.lock:
                silent = [lg for lg in self.legs.values() if lg["state"] == "sent" and not lg["resent"]
                          and now - lg["t_send"] > 0.45]
            for lg in silent:
                lg["resent"] = True
                r.send(lg["robot"], "%s %.2f %d" % (lg["kind"], lg["value"], lg["id"]))
                self.notes.append("%s's leg %d resent (no answer in 0.45 s)" % (lg["robot"], lg["id"]))

    def after_leg(self, n, lg):
        """The robot's heading after a leg: a go measured by the camera (the way it moved), a turn by its own gyro."""
        h, measured = self.head.get(n, [None, False])
        if lg["kind"] == "turn" and h is not None:
            ps, pd = lg.get("pose_started"), lg.get("pose_done")
            turned = lg["value"]
            if ps and pd and ps[2] is not None and pd[2] is not None:
                turned = lg["value"] + wrap180((pd[2] - ps[2]) - lg["value"])
            self.head[n] = [wrap180(h + turned), measured and abs(turned) < 15.0]
        elif lg["kind"] == "go" and abs(lg["value"]) >= 3.0:
            t0 = lg.get("t_motion_robot") or lg["t_motion_est"]
            before = self.mt.history(n, t0 - 0.6, t0 + 0.1)
            after = self.mt.history(n, lg["t_done"] - 0.5, lg["t_done"] + 0.3)
            if len(before) >= 2 and len(after) >= 2:
                p0 = (float(np.median([q[1] for q in before])), float(np.median([q[2] for q in before])))
                p1 = (float(np.median([q[1] for q in after])), float(np.median([q[2] for q in after])))
                if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) >= 3.0:
                    hb = bearing(p0, p1)
                    self.head[n] = [hb if lg["value"] > 0 else wrap180(hb + 180.0), True]
        self.mt.set_heading(n, self.head[n][0])

    def stop_all(self, why):
        """Stop every robot (three times, a lost packet must not matter) and end the dance."""
        if self.abort.is_set():
            return
        self.abort.set()
        r = self.r
        for _ in range(3):
            for n in self.robot_of.values():
                r.send(n, "stop")
            time.sleep(0.1)
        beat = (time.time() - self.T0) / self.dance.spb if self.T0 else None
        self.stops.append(dict(t=round(time.time(), 2), beat=round(beat, 1) if beat is not None else None, why=why))
        self.say("  STOPPED EVERY ROBOT (dance supervisor): %s" % why)
        r.mark("e02 dance supervisor stop")
        self.phase, self.phase_words = "stopped", "stopped: " + why

    def supervisor(self):
        """10 times a second while dancing: a pair predicted (now and HORIZON_S ahead, from the camera and the
        commanded motion) closer than the hard minimum, a robot about to be within EDGE_STOP_CM of an open edge, or a
        robot the camera has been unsure of / lost for UNSURE_STOP_S: stop everything."""
        r = self.r
        hard = self.rule["hard_cm"]
        names = list(self.robot_of.values())
        while not self.quit.is_set() and not self.abort.is_set():
            time.sleep(0.1)
            try:
                now = time.time()
                if self.T0 is None or now < self.T0 - 1.0:
                    continue
                sts = {n: self.mt.state(n) for n in names}
                for n, s in sts.items():
                    if s["status"] == "lost" or (s["status"] != "seen" and now - s["since"] > UNSURE_STOP_S):
                        self.stop_all("the camera lost %s (%s)" % (n, s["why"] or s["status"]))
                        break
                if self.abort.is_set():
                    break
                p_now = {n: self.mt.predict(n, now) for n in names}
                p_fut = {n: self.mt.predict(n, now + HORIZON_S) for n in names}
                gaps = []
                for i, a in enumerate(names):
                    for b in names[i + 1:]:
                        g0 = math.hypot(p_now[a][0] - p_now[b][0], p_now[a][1] - p_now[b][1]) - 2 * R_BODY
                        g1 = math.hypot(p_fut[a][0] - p_fut[b][0], p_fut[a][1] - p_fut[b][1]) - 2 * R_BODY
                        gaps.append(g0)
                        if min(g0, g1) < hard:
                            self.stop_all("%s and %s %s %.1f cm apart (the hard minimum is %.1f cm)" % (
                                a, b, "are" if g0 <= g1 else "were about to be", min(g0, g1), hard))
                            break
                    if self.abort.is_set():
                        break
                self.gap_now = min(gaps) if gaps else None
                if self.abort.is_set():
                    break
                for n in names:
                    od0, _ = edge_clearance(r.board, p_now[n][0], p_now[n][1])
                    od1, _ = edge_clearance(r.board, p_fut[n][0], p_fut[n][1])
                    if float(od1) + R_BODY < EDGE_STOP_CM and float(od1) < float(od0) - 0.3:
                        self.stop_all("%s was about to come within %.0f cm of an open edge" % (n, EDGE_STOP_CM))
                        break
            except Exception as e:                     # the supervisor must never die quietly
                self.say("  (dance supervisor: %s)" % e, quiet=True)

    # --- the numbers --------------------------------------------------------------------------------------------
    def results(self, names):
        r = self.r
        dn = self.dance
        if self.T0 is None:
            return None
        spb = dn.spb
        T_end = getattr(self, "T_end", time.time())
        sp_role = {role: self.speeds[n] for role, n in self.robot_of.items()}
        trs = {role: Traj(dn.start[role], dn.moves[role], sp_role[role], spb) for role in dn.roles}
        # 1. closest approach: camera (sure sightings, interpolated where the robot was seen within 0.3 s)
        t = np.arange(self.T0 - 0.5, T_end, 0.1)
        cam = {}
        for n in names:
            h = self.mt.history(n, self.T0 - 1.0, T_end + 0.5)
            if len(h) < 5:
                continue
            ht = np.array([q[0] for q in h])
            x = np.interp(t, ht, [q[1] for q in h])
            y = np.interp(t, ht, [q[2] for q in h])
            idx = np.clip(np.searchsorted(ht, t), 1, len(ht) - 1)
            near = np.minimum(np.abs(ht[idx] - t), np.abs(ht[idx - 1] - t)) <= 0.3
            cam[n] = (x, y, near)
        act = _closest(t, cam, self.T0, spb)
        truth = self.truth_closest(names, t) if r.a.dry_run else None
        # 2. against the plan: every sure sighting vs where the plan has the robot at that moment
        devs, move_dev = [], []
        for role, n in self.robot_of.items():
            h = self.mt.history(n, self.T0, T_end)
            if not h:
                continue
            ht = np.array([q[0] for q in h])
            px, py, _, _, _ = trs[role].sample(ht - self.T0)
            d = np.hypot(np.array([q[1] for q in h]) - px, np.array([q[2] for q in h]) - py)
            devs += [float(v) for v in d]
            mv = dn.moves[role]
            for i, m in enumerate(mv):
                a = self.T0 + m["beat"] * spb
                b = self.T0 + mv[i + 1]["beat"] * spb if i + 1 < len(mv) else a + sp_role[role].duration(m["kind"], m["value"]) + 1.0
                if self.abort.is_set() and self.stops and a > self.stops[0]["t"]:
                    break
                k = (ht >= a) & (ht < b)
                if k.sum() >= 3 and m["kind"] != "flash":
                    move_dev.append(round(float(np.max(d[k])), 2))
        # 3. timing: go legs, when the camera saw the robot start moving vs its beat
        t_err_cam, t_err_robot = [], []
        with self.lock:
            legs = list(self.legs.values())
        for lg in legs:
            if lg.get("t_motion_robot"):
                t_err_robot.append(lg["t_motion_robot"] - lg["t_beat"])
            if lg["kind"] != "go" or abs(lg["value"]) < 4.0 or lg.get("late"):
                continue
            h = self.mt.history(lg["robot"], lg["t_beat"] - 1.5, lg["t_beat"] + 3.0)
            pre = [q for q in h if q[0] <= lg["t_send"]]
            if len(pre) < 3:
                continue
            p0 = (float(np.median([q[1] for q in pre[-6:]])), float(np.median([q[2] for q in pre[-6:]])))
            sp = self.speeds[lg["robot"]]
            t1cm = _time_to(1.0, abs(lg["value"]), sp.duration("go", lg["value"]), sp.ramp("go"))
            for q in h:
                if q[0] > lg["t_send"] and math.hypot(q[1] - p0[0], q[2] - p0[1]) > 1.0:
                    t_err_cam.append(q[0] - t1cm - lg["t_beat"])
                    break
        out = dict(dry_run=bool(r.a.dry_run), bpm=self.bpm, t0=self.T0, music0=self.music0, robot_of=self.robot_of,
                   rule=self.rule, speeds={n: s.to_dict() for n, s in self.speeds.items()},
                   planned_closest=self.chk.get("closest_pair"), planned_closest_all=self.chk.get("closest"),
                   planned_robust_cm=self.chk.get("robust_gap_cm"), actual_closest_camera=act,
                   actual_closest_truth=truth, timing_camera_s=_stats(t_err_cam), timing_robot_s=_stats(t_err_robot),
                   dev_cm=_stats(devs), move_dev_cm=move_dev, stops=self.stops, aborted=self.abort.is_set(),
                   notes=self.notes[:50], flashes=len(self.flashes), legs=len(legs),
                   legs_late=sum(1 for lg in legs if lg.get("late")), legs_failed=sum(1 for lg in legs if lg["state"] == "failed"),
                   radio_one_way_s={n: round(self.one_way(n), 4) for n in names},
                   tracker_events=list(self.mt.events)[-60:], duration_s=round(T_end - self.T0, 1),
                   planned_s=self.chk.get("duration_s"))
        with open(os.path.join(r.dir, "dance_result.json"), "w") as f:
            json.dump(_jsonable(out), f, indent=1)
        r.st["dance"] = dict(t=round(self.T0, 1), aborted=out["aborted"], stops=self.stops,
                             planned_closest_cm=(self.chk.get("closest_pair") or [None, {}])[1].get("gap_cm"),
                             actual_closest_cm=(act or {}).get("gap_cm"),
                             true_closest_cm=(truth or {}).get("gap_cm"),
                             timing_median_s=out["timing_camera_s"].get("median_abs"))
        r.st.save()
        for line in result_words(out, self.robot_of):
            self.say(line)
        return out

    def truth_closest(self, names, t):
        """Dry run: the simulator's true positions (sim/truth.csv), the closest any two bodies really came."""
        import csv
        p = os.path.join(self.r.dir, "sim", "truth.csv")
        if not os.path.exists(p):
            return None
        tr = defaultdict(list)
        t_lo, t_hi = t[0] - 1.0, t[-1] + 1.0
        try:
            with open(p) as f:
                for row in csv.DictReader(f):
                    tt = float(row["t"])
                    if t_lo <= tt <= t_hi and row["robot"] in names:
                        tr[row["robot"]].append((tt, float(row["x_cm"]), float(row["y_cm"])))
        except (OSError, ValueError, KeyError):
            return None
        S = {}
        for n, rows in tr.items():
            if len(rows) < 5:
                continue
            ht = np.array([q[0] for q in rows])
            S[n] = (np.interp(t, ht, [q[1] for q in rows]), np.interp(t, ht, [q[2] for q in rows]),
                    np.ones(len(t), bool))
        return _closest(t, S, self.T0, self.dance.spb)

    # --- the page -------------------------------------------------------------------------------------------
    def planned_paths(self):
        r = self.r
        dn = self.dance
        out = {}
        for k, (role, n) in enumerate(sorted(self.robot_of.items())):
            tr = Traj(dn.start[role], dn.moves[role], self.speeds[n], dn.spb)
            tt = np.arange(0.0, tr.t_end + 0.5, 0.25)
            x, y, _, _, _ = tr.sample(tt)
            pts = []
            for a, b in zip(x, y):
                u, v = r.cam.project((float(a), float(b), 3.7))
                if np.isfinite(u) and np.isfinite(v):
                    pts.append([round(float(u), 1), round(float(v), 1)])
            out[n] = dict(color=COLORS[k % len(COLORS)], pts=pts)
        return out

    def page_state(self):
        r = self.r
        dn = self.dance
        out = dict(phase=self.phase, phase_words=self.phase_words, t=time.time(), bpm=self.bpm,
                   spb=dn.spb if dn else None, t0=self.T0, music0=self.music0,
                   beats=self.chk["beats"] if self.chk else None, music="/dance/music.wav" if self.music_path else None,
                   gap_rule=self.rule["gap_cm"] if self.rule else None, hard=self.rule["hard_cm"] if self.rule else None,
                   rule_words=self.rule["words"] if self.rule else "", gap_now=self.gap_now,
                   gap_plan=(self.chk.get("closest_pair") or [None, {}])[1].get("gap_cm") if self.chk else None,
                   stop=("Stopped: " + self.stops[0]["why"]) if self.stops else "")
        if r.cam is not None:
            out["img"] = list(r.cam.image_size)
        out["planned"] = self.planned_px
        if dn is not None and self.T0 is not None:
            b = (time.time() - self.T0) / dn.spb
            fig = None
            for f in dn.figures:
                if f.get("beat") is not None and f["beat"] <= b:
                    fig = f
            if fig:
                out["figure"], out["figure_words"] = fig["name"], fig["words"]
            elif b < 0:
                out["figure"], out["figure_words"] = "Count-in", "The music counts in two bars; then they start."
        if self.mt is not None:
            robots, actual = {}, {}
            for k, (role, n) in enumerate(sorted(self.robot_of.items())):
                s = self.mt.state(n)
                if s is None:
                    continue
                u, v = r.cam.project((s["x"], s["y"], 3.7))
                rpx = 4.75 * r.cam.px_per_cm(s["x"], s["y"], 3.7)
                fl = [f for f in self.flashes if f[0] == n]
                robots[n] = dict(u=round(float(u), 1), v=round(float(v), 1), rpx=round(float(rpx), 1), status=s["status"],
                                 color=COLORS[k % len(COLORS)], flash_from=fl[-1][1] if fl else None,
                                 flash_until=(fl[-1][1] + FLASH_S) if fl else None)
                h = self.mt.history(n, (self.T0 or time.time()) - 0.5)[-900::3]
                pts = []
                for q in h:
                    uu, vv = r.cam.project((q[1], q[2], 3.7))
                    pts.append([round(float(uu), 1), round(float(vv), 1)])
                actual[n] = dict(color=COLORS[k % len(COLORS)], pts=pts)
            out["robots"], out["actual"] = robots, actual
        return _jsonable(out)


def _num(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def _time_to(cm, total_cm, dur, ramp):
    """Seconds after its motion starts that a go leg of total_cm (taking dur s) has covered cm."""
    if total_cm <= 0:
        return 0.0
    import e02_multitrack as MT
    lo, hi = 0.0, dur
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        if MT.trapezoid_fraction(mid, dur, ramp) * total_cm < cm:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _closest(t, S, T0, spb):
    """The closest two bodies came: S {name: (x array, y array, valid mask)} at times t."""
    best = None
    names = sorted(S)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            ok = S[a][2] & S[b][2]
            if not ok.any():
                continue
            g = np.hypot(S[a][0] - S[b][0], S[a][1] - S[b][1]) - 2.0 * R_BODY
            g = np.where(ok, g, np.inf)
            k = int(np.argmin(g))
            if best is None or g[k] < best["gap_cm"]:
                best = dict(gap_cm=round(float(g[k]), 2), pair="%s-%s" % (a, b), t=round(float(t[k] - T0), 2),
                            beat=round(float((t[k] - T0) / spb), 1))
    return best


def _stats(v):
    if not v:
        return dict(n=0)
    a = np.array(v, float)
    return dict(n=len(a), median=round(float(np.median(a)), 3), median_abs=round(float(np.median(np.abs(a))), 3),
                p95_abs=round(float(np.percentile(np.abs(a), 95)), 3), max_abs=round(float(np.max(np.abs(a))), 3))


def _jsonable(o):
    if isinstance(o, dict):
        return {str(k) if not isinstance(k, str) else k: _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, float) and not math.isfinite(o):
        return None
    return o


def result_words(out, robot_of=None):
    """Plain lines after a dance."""
    L = []
    pc = out.get("planned_closest")
    if pc:
        L.append("Closest approach, planned: %.1f cm (%s, at %.0f s)." % (pc[1]["gap_cm"], pc[0], pc[1]["t"]))
    a = out.get("actual_closest_camera")
    if a:
        L.append("Closest approach, as the camera saw it: %.1f cm (%s, at %.0f s)." % (a["gap_cm"], a["pair"], a["t"]))
    tr = out.get("actual_closest_truth")
    if tr:
        L.append("Closest approach, the simulator's truth: %.1f cm (%s, at %.0f s)." % (tr["gap_cm"], tr["pair"], tr["t"]))
    tc, trb = out.get("timing_camera_s") or {}, out.get("timing_robot_s") or {}
    if tc.get("n"):
        L.append("Timing: the camera saw drives start %.2f s from their beat (median; 95%% within %.2f s, %d drives)." % (
            tc["median_abs"], tc["p95_abs"], tc["n"]))
    if trb.get("n"):
        L.append("By the robots' own records every leg started %.2f s from its beat (median; 95%% within %.2f s)." % (
            trb["median_abs"], trb["p95_abs"]))
    d = out.get("dev_cm") or {}
    if d.get("n"):
        L.append("Against the plan, at the same moment: %.1f cm median, 95%% within %.1f cm (%d camera sightings; %d "
                 "moves measured for the gap rule)." % (d["median"], d["p95_abs"], d["n"], len(out.get("move_dev_cm") or [])))
    if out.get("stops"):
        for s in out["stops"]:
            L.append("Safety supervisor STOPPED the dance at beat %s: %s." % (s.get("beat"), s["why"]))
    else:
        L.append("Safety supervisor: no stops.")
    if out.get("legs_failed"):
        L.append("%d legs were stopped by a robot's own sensors." % out["legs_failed"])
    if out.get("legs_late"):
        L.append("%d legs were sent while the robot was still finishing the one before (they ran late)." % out["legs_late"])
    return L
