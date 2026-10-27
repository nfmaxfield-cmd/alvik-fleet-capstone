"""Follow every robot at once while several move (the dance, 2026-10-22; Nick: "just barely missing each other but
never crashing"). Built on e02_camera.MotionTracker's floor memory: what the floor looks like at every spot, kept
fresh wherever no robot is, so anything robot-shaped that differs from it is a robot (or a hand).

Who is who is decided by PREDICTION, never by looks (three Alviks look the same from above): the runner tells the
tracker every leg it sends (`command`: go / turn, when it should start, how long it should take, which way the robot
faces), and the robot's own LEG records refine it (`started`: when it really began; `leg_done`: how far its wheels
and gyro say it went). Each frame, every robot's predicted position = where the camera last saw it for sure + the
motion its commands explain since then. The outlines in the picture are then matched to the predictions (the
assignment with the least total distance, inside a gate round each prediction).

Unsure is flagged, not guessed. A robot's sighting is only taken when it is clear:
  - merged: one outline as big as two robots, near two predictions (robots touching or nearly): both are UNSURE and
    keep their predictions;
  - contested: an outline about as near two predictions (within AMBIG_CM of each other), and the other robot did not
    clearly find its own: both UNSURE;
  - two things where one robot should be (a hand next to it, its own shadow split off): UNSURE;
  - nothing robot-shaped near the prediction: PREDICTED (missed) and, after LOST_S, LOST.
A robot that is unsure or lost is drawn bigger in the floor memory's "don't learn here" mask, so the floor never
learns a robot. The dance's safety supervisor stops every robot when one stays unsure or lost too long.

Positions are the outline's centre on the plane of the robot's top (e02_camera.ROBOT_TOP_CM), as MotionTracker's.
"""
import math
import threading
import time
from collections import deque

import cv2
import numpy as np

import e02_camera as CAM

GATE_CM = 6.0             # a sighting this near a robot's prediction can be that robot...
GATE_GROW_CMS = 10.0      # ...plus this much per second since it was last seen for sure (at most GATE_MAX_CM)
GATE_MAX_CM = 18.0
AMBIG_CM = 2.5            # two explanations closer than this to each other: not sure which is right
MERGE_AREA = 1.55         # an outline this many robot areas big near two predictions: robots merged in the picture
LOST_S = 1.2              # no sure sighting this long: lost
TURN_SWAY_CM = 1.0        # a robot turning on the spot: its outline's centre may sway this much (axle not centred)


def trapezoid_fraction(tau, dur, ramp):
    """Share of a leg covered tau s after its motion started: speed ramps up over `ramp` s, holds, ramps down over
    `ramp` s (the whole leg takes `dur` s). 0 before, 1 after."""
    if tau <= 0 or dur <= 0:
        return 0.0
    if tau >= dur:
        return 1.0
    r = min(max(ramp, 1e-6), dur / 2.0)
    total = dur - r                                  # distance in units of cruise speed x s
    if tau < r:
        s = 0.5 * tau * tau / r
    elif tau <= dur - r:
        s = 0.5 * r + (tau - r)
    else:
        u = dur - tau
        s = total - 0.5 * u * u / r
    return max(0.0, min(1.0, s / total))


class Motion:
    """One commanded leg as the tracker predicts it: a straight drive of `value` cm along heading `h` (deg) or a
    turn on the spot, starting at laptop time t0, taking dur s. After the robot reports the leg done, the
    displacement its own wheels and gyro measured replaces the model's."""

    def __init__(self, kind, value, t0, dur, h, ramp=0.3):
        self.kind, self.value, self.t0, self.dur, self.h, self.ramp = kind, float(value), float(t0), float(dur), h, ramp
        self.reported = None              # (dx, dy) from the robot's LEG records, once done
        self.t_done = None

    def disp(self, t):
        """Displacement (dx, dy) cm from the start of the leg at time t."""
        if self.kind != "go" or self.h is None:
            return 0.0, 0.0
        if self.reported is not None and t >= self.t_done:
            return self.reported
        f = trapezoid_fraction(t - self.t0, self.dur, self.ramp)
        d = self.value * f
        c, s = math.cos(math.radians(self.h)), math.sin(math.radians(self.h))
        if self.reported is not None:             # done, but t is before the report: scale the report by the model
            return self.reported[0] * f, self.reported[1] * f
        return d * c, d * s

    def moving(self, t):
        end = self.t_done if self.t_done is not None else self.t0 + self.dur
        return self.t0 - 0.05 <= t <= end + 0.05

    def speed(self, t):
        if self.kind != "go" or not self.moving(t):
            return 0.0
        return abs(self.value) / max(self.dur - self.ramp, 0.2)


class Track:
    def __init__(self, name, x, y, t, heading=None):
        self.name = name
        self.x, self.y = float(x), float(y)       # best estimate now (camera if sure, else prediction)
        self.bx, self.by, self.bt = float(x), float(y), float(t)   # last SURE sighting
        self.status = "seen"                      # seen | predicted | unsure | lost
        self.why = ""
        self.t_status = t                         # since when it has this status
        self.heading = heading
        self.motions = deque(maxlen=12)
        self.hist = deque(maxlen=20000)           # (t, x, y, status)
        self.area = None


class MultiTracker(CAM.MotionTracker):
    """Follows every robot it is told about, at once. Thread-safe (push from the camera thread; command / state
    from the runner)."""

    def __init__(self, cam, board_center=(61.0, 61.0), board_rect=None, margin_cm=8.0, thr=16.0, alpha=0.02, **kw):
        super().__init__(cam, board_center, thr=thr, alpha=alpha, board_rect=board_rect, margin_cm=margin_cm, **kw)
        self.tracks = {}
        self.events = deque(maxlen=500)           # (t, robot, what): status changes, for the record
        self.n_frames = 0

    # --- what the runner tells it ----------------------------------------------------------------------
    def add(self, name, x, y, heading=None, t=None):
        with self.lock:
            t = time.time() if t is None else t
            self.tracks[name] = Track(name, x, y, t, heading)
            self.protect[name] = (x, y)

    def remove(self, name):
        with self.lock:
            self.tracks.pop(name, None)
            self.protect.pop(name, None)

    def command(self, name, kind, value, t0, dur, heading=None, ramp=0.3):
        """A leg was sent: `go` value cm along heading (deg) or `turn` value deg, its motion expected from laptop
        time t0 for dur s."""
        with self.lock:
            tr = self.tracks.get(name)
            if tr is None:
                return
            if heading is not None:
                tr.heading = heading
            tr.motions.append(Motion(kind, value, t0, dur, tr.heading if kind == "go" else None, ramp))

    def started(self, name, t0):
        """The robot's LEG record says its latest leg's motion began at t0 (laptop time): shift the model."""
        with self.lock:
            tr = self.tracks.get(name)
            if tr and tr.motions and tr.motions[-1].reported is None:
                tr.motions[-1].t0 = float(t0)

    def leg_done(self, name, t, dx=None, dy=None):
        """The latest leg ended at laptop time t; (dx, dy): how far the robot's own wheels and gyro say it went."""
        with self.lock:
            tr = self.tracks.get(name)
            if tr and tr.motions:
                m = tr.motions[-1]
                m.t_done = float(t)
                if m.kind == "go":
                    if dx is not None and dy is not None:
                        m.reported = (float(dx), float(dy))
                    else:
                        m.reported = m.disp(m.t0 + m.dur)

    def set_heading(self, name, heading):
        with self.lock:
            if name in self.tracks:
                self.tracks[name].heading = heading

    # --- prediction ----------------------------------------------------------------------------------------
    def _disp(self, tr, ta, tb):
        dx = dy = 0.0
        for m in tr.motions:
            a, b = m.disp(ta), m.disp(tb)
            dx += b[0] - a[0]
            dy += b[1] - a[1]
        return dx, dy

    def _predict(self, tr, t):
        dx, dy = self._disp(tr, tr.bt, t)
        return tr.bx + dx, tr.by + dy

    def predict(self, name, t=None):
        """Where the commands say the robot is at time t (from its last sure sighting), or None."""
        with self.lock:
            tr = self.tracks.get(name)
            return None if tr is None else self._predict(tr, time.time() if t is None else t)

    def _speed(self, tr, t):
        return max([m.speed(t) for m in tr.motions] or [0.0])

    def _gate(self, tr, t):
        g = GATE_CM + min(GATE_MAX_CM - GATE_CM, GATE_GROW_CMS * max(0.0, t - tr.bt - 0.1))
        g += 0.3 * self._speed(tr, t)               # moving: its start time is only known to ~0.3 s
        return g

    # --- every frame -------------------------------------------------------------------------------------
    def _covered(self, shape):
        m = np.zeros(shape, np.uint8)
        for tr in self.tracks.values():
            grow = 0.0
            if tr.status != "seen":
                grow = min(1.5, GATE_GROW_CMS * max(0.0, (self.frame_t or tr.bt) - tr.bt) / CAM.ROBOT_CM)
            m |= self._disk_mask(shape, [(tr.x, tr.y)], 1.25 + grow)
            if (tr.bx, tr.by) != (tr.x, tr.y) and tr.status != "seen":
                m |= self._disk_mask(shape, [(tr.bx, tr.by)], 1.25)
        return m

    def push(self, img, t):
        """One frame (grey uint8) taken at laptop time t. Usable as a Feed hook (img, t)."""
        g = cv2.GaussianBlur(img, (3, 3), 0).astype(np.float32)
        with self.lock:
            self.n += 1
            self.n_frames += 1
            dt_frame = (t - self.frame_t) if self.frame_t is not None else 0.04
            self.frame, self.frame_t = g, t
            if self.bg is None:
                self._init_floor([g])
                self.prev = g
                return None
            if self.prev is None or self.prev.shape != g.shape:
                self.prev = g
            motion = (np.abs(g - self.prev) > self.MOTION_THR).astype(np.uint8)
            self.prev = g
            self.stable = np.where(motion > 0, 0, np.minimum(self.stable + 1, 30000)).astype(np.int16)
            if self.n % 5 == 0:
                self._update_offset(g)
            cands = [c for c in self.candidates(g, motion) if self.on_area(c["x"], c["y"])]
            self.last_cands = cands
            if self.tracks:
                self._assign(cands, t)
            if not self.frozen:
                self._learn(g, dt_frame)
            return None

    def _verdicts(self, names, preds, gates, cands, dist=None):
        """Each track's verdict on this picture's outlines: (usable outlines, {track index: ("seen", outline index) |
        ("unsure", why) | ("missed", why)}). dist(c, p): the distance of outline c from prediction p (default: plain
        distance on the floor; the side camera stretches it along its line of sight)."""
        dist = dist or (lambda c, p: math.hypot(c["x"] - p[0], c["y"] - p[1]))
        trs = names
        def A_at(c):        # a robot's area in pixels where the outline is (the side camera's scale changes 2-3x)
            return self._rpx_at(c["x"], c["y"]) ** 2 if self.size_per_spot else self.area
        usable = [c for c in cands if c["shape"] or (MERGE_AREA * A_at(c) * 0.8 <= c["area"] <= 4.0 * A_at(c))]
        D = np.full((len(trs), len(usable)), np.inf)
        for i, p in enumerate(preds):
            for j, c in enumerate(usable):
                d = dist(c, p)
                if d <= gates[i]:
                    D[i, j] = d
        verdict = {}                                   # track index -> ("seen", cand) | ("unsure", why) | ("missed", why)
        # 1. merged outlines: one big outline in the gates of two or more robots
        merged_c = set()
        for j, c in enumerate(usable):
            who = [i for i in range(len(trs)) if np.isfinite(D[i, j])]
            if len(who) >= 2 and c["area"] >= MERGE_AREA * A_at(c):
                merged_c.add(j)
                for i in who:
                    verdict[i] = ("unsure", "merged with %s in the picture" % ", ".join(
                        names[k] for k in who if k != i))
        # 2. the assignment with the least total distance among the rest (only robot-shaped outlines)
        rows = [i for i in range(len(trs)) if i not in verdict]
        cols = [j for j in range(len(usable)) if j not in merged_c and usable[j]["shape"]]
        pick = {}
        if rows and cols:
            sub = D[np.ix_(rows, cols)]
            big = 1e6
            cost = np.where(np.isfinite(sub), sub, big)
            try:
                from scipy.optimize import linear_sum_assignment
                ri, ci = linear_sum_assignment(cost)
            except Exception:                          # no scipy: greedy (fine for a handful of robots)
                ri, ci, used = [], [], set()
                for k in np.argsort(cost, axis=None):
                    a, b = divmod(int(k), cost.shape[1])
                    if a in ri or b in used:
                        continue
                    ri.append(a)
                    ci.append(b)
                    used.add(b)
            for a, b in zip(ri, ci):
                if cost[a, b] < big:
                    pick[rows[a]] = cols[b]
        # 3. is each pick clear?
        taken = set(pick.values())
        for i in rows:
            if i not in pick:
                verdict[i] = ("missed", "nothing robot-shaped where it should be")
                continue
            j = pick[i]
            d = D[i, j]
            # (a) another outline nobody took, about as near: two things where one robot should be
            alt = [D[i, k] for k in cols if k != j and k not in taken and np.isfinite(D[i, k])]
            if alt and min(alt) < d + AMBIG_CM:
                verdict[i] = ("unsure", "two things where it should be")
                continue
            # (b) another robot about as near this outline, which did not clearly get its own
            contested = False
            for k in range(len(trs)):
                if k == i or not np.isfinite(D[k, j]) or D[k, j] >= d + AMBIG_CM:
                    continue
                own = pick.get(k)
                if own is not None and D[k, own] + AMBIG_CM < D[k, j]:
                    continue                           # it clearly has its own outline
                contested = True
                verdict[i] = ("unsure", "as near to %s's predicted spot" % names[k])
                if k not in verdict or verdict[k][0] == "seen":
                    verdict[k] = ("unsure", "as near to %s's predicted spot" % names[i])
            if not contested and i not in verdict:
                verdict[i] = ("seen", j)
        return usable, verdict

    def _assign(self, cands, t):
        names = list(self.tracks)
        trs = [self.tracks[n] for n in names]
        preds = [self._predict(tr, t) for tr in trs]
        gates = [self._gate(tr, t) for tr in trs]
        usable, verdict = self._verdicts(names, preds, gates, cands)
        # 4. update
        for i, tr in enumerate(trs):
            kind, what = verdict.get(i, ("missed", ""))
            old = tr.status
            if kind == "seen":
                c = usable[what]
                tr.x, tr.y = c["x"], c["y"]
                tr.bx, tr.by, tr.bt = c["x"], c["y"], t
                tr.area = c["area"]
                tr.status, tr.why = "seen", ""
                self.protect[tr.name] = (tr.x, tr.y)
            else:
                tr.x, tr.y = preds[i]
                if kind == "unsure":
                    tr.status, tr.why = "unsure", what
                else:
                    tr.status = "lost" if t - tr.bt > LOST_S else "predicted"
                    tr.why = what
                    if tr.status == "lost":
                        tr.why = "not seen for sure for %.1f s" % (t - tr.bt)
                if old == "seen" or tr.status != old:
                    pass
            if tr.status != old:
                tr.t_status = t
                self.events.append((round(t, 3), tr.name, tr.status + ((": " + tr.why) if tr.why else "")))
            tr.hist.append((t, tr.x, tr.y, tr.status))

    # --- what the runner and the view read ------------------------------------------------------------
    def state(self, name):
        """dict(x, y, status, why, t_seen, since, heading) or None."""
        with self.lock:
            tr = self.tracks.get(name)
            if tr is None:
                return None
            return dict(x=tr.x, y=tr.y, status=tr.status, why=tr.why, t_seen=tr.bt, since=tr.t_status,
                        heading=tr.heading, sure_x=tr.bx, sure_y=tr.by)

    def states(self):
        return {n: self.state(n) for n in list(self.tracks)}

    def history(self, name, t0=0.0, t1=None, sure_only=True):
        """[(t, x, y)] of the robot between t0 and t1 (only sure sightings, unless sure_only=False)."""
        with self.lock:
            tr = self.tracks.get(name)
            if tr is None:
                return []
            return [(q[0], q[1], q[2]) for q in tr.hist if q[0] >= t0 and (t1 is None or q[0] <= t1)
                    and (q[3] == "seen" or not sure_only)]

    def still_at(self, name, t_from, min_s=0.3, max_spread_cm=1.0):
        """The robot's position if it has stood still (sure sightings) for the last min_s since t_from, else None."""
        smp = self.history(name, max(t_from, time.time() - 2.0))
        return CAM.still_position(smp, min_s=min_s, max_spread_cm=max_spread_cm / 2.5) if smp else None


def closest_pairs(tracks_xy, r_body):
    """Every pair's gap now: {(a, b): centre distance - 2 r_body} from {name: (x, y)}."""
    names = sorted(tracks_xy)
    out = {}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            pa, pb = tracks_xy[a], tracks_xy[b]
            out[(a, b)] = math.hypot(pa[0] - pb[0], pa[1] - pb[1]) - 2.0 * r_body
    return out
