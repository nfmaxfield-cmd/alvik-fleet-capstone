"""How much to trust each sensor, learned by the system itself (Nick, 2026-10-24: "the system needs to learn how
to weigh each sensor's accuracy confidence for itself. I want to be able to swap out any one of those sensors for
an ineffective version and have the system catch the error and account for it").

Four sensors: the overhead camera, each robot's wheels, its gyro and its front distance sensor.

THE RULE (Nick, 2026-10-25 13:03): "we should not compare one robot's wheels or anything else to another robot until
they've each been fully calibrated independently." So every robot's sensors are first judged ON THEIR OWN, against
references that do not involve another robot, and against their OWN history; the robots are compared with each other
only afterwards, as a second check. A sensor is flagged when any one check says it has clearly changed or is clearly
worse than its own baseline (DEGRADED_K x); cleared when every check is back under RECOVERED_K.

The checks (each gives k = this session's value / the sensor's own baseline):

  camera   the finds: right after a find the robot's position is known from touching both boards (to about a
           millimetre); the camera's position there, corrected by its error map from the OTHER finds, against it.
           And a robot standing still: its camera position's scatter. Against the Alvik prior (HEALTHY camera).
  wheels   1. stretches: the self-calibration's stretches (camera against wheels, rms per fit), this session's
              against the robot's own earlier sessions.
           2. runs: each run's wheel scale (the camera's length over the wheels' length, that run alone). A healthy
              wheel gives the same scale every run (0.1-0.5% apart in the simulator); one that counts wrongly by a
              different amount every run does not (2.6-12% apart at 5-10% jitter). This session's spread against its
              own earlier spread, never below the REAL robots' healthy 2.6% (their camera errors differ across the
              board and runs go to different places: 0.4-4.8% apart when healthy). Needs no other robot.
  gyro     stretches: the same as the wheels' check 1, with the stretches' heading rms (deg).
  front    1. map: the reading against the camera-and-map distance ahead (after its learned offset, about zero, so a
              bias counts); the camera's share is taken off.
           2. face: right after a nose-first find the robot is square to the y board at a distance known from the
              board touches (no camera): a front reading there (e02.py face_reading).
           3. changed: e02_front's "reads differently from earlier sessions" report flags it for the session.
  second   (wheels, gyro) only once every robot has its own verdict from this session: a robot whose stretches are
           STANDS_OUT_FLAG x the median of the robots that look healthy is flagged too. It never clears or blocks a
           flag from the robot's own checks.

The baseline of a check: the robot's own items from earlier sessions (items from a session in which that sensor was
flagged are left out, so a bad part never becomes the baseline), once there are HIST_MIN of them; until then the
Alvik prior from the real robots (HEALTHY, with the camera's measured share added where the check includes it).
Floors (BASE_FLOOR) keep a very quiet history from making ordinary noise look bad.

Not changed by this: the self-calibration's limit (wheel_gate_cm) and the "every robot shares it" note still use
what the robots share (that is about how much camera noise to allow in a fit, not about which sensor is bad).

Persisted between sessions (logs/sensor_trust.json): each item's weight halves every new session (DECAY_PER_SESSION)
and is dropped below 0.01 (about 7 sessions), so a history follows a sensor that changes. All limits are chosen, not
measured: DOE notes beside them, rows in docs/doe-backlog.md. Tests: tests/test_e02_trust.py."""
import json
import math
import os
import statistics
import time

# The Alvik prior: healthy noise (1 sd) from the real robots (2026-10-23 and 2026-10-24 sessions). Since 2026-10-25
# only the STARTING baseline of a sensor that has no history of its own yet; each robot's own replaces it.
CAMERA_STILL_HEALTHY = 0.4   # cm per axis: a still robot's camera position wobbles this much over a few seconds
                             # (the KC replay's still pictures: 0.3-1 cm; chosen inside that; a DOE with a robot
                             # standing still at several places would set it)
HEALTHY = dict(
    camera=0.8,    # cm per axis at a find, after the error map (leave one out): 0.71 / 1.03 on 2026-10-24's stages
    wheels=0.6,    # cm per stretch, the wheels' own share: the 2026-10-23 morning's rms 1.25 with the camera's 0.8
    gyro=1.0,      # deg per stretch, the gyro's own share (the morning's rms 1.39 less the camera's heading share)
    front=5.0,     # cm, front sensor against the map after its offset, about zero (MAD x 1.48 about the median:
                   # 4.0-4.6 in four sessions, medians -3 to +1)
)
RUN_SPREAD_PRIOR = 0.026  # measured: a healthy wheel's run-to-run scale spread on the REAL robots, median of 12
                          # robot-sessions (2026-10-23 laptop, 2026-10-24 Pi stages 1-2): 0.4-4.8%. The runs check's
                          # baseline never goes below it (own history can raise the bar, not lower it): the simulator's
                          # 0.1-0.5% would make the real robots' ordinary 2-5% look bad (trust_fix_real_replay.py). DOE:
                          # more real sessions; split by laptop / Pi camera and by route, which may explain the spread
FACE_HEALTHY = 2.0        # chosen: a healthy front sensor's sd at a board, its distance known from the board touches
                          # (no camera, no map), before a robot has a history; the simulator's healthy ones read 0.6 cm,
                          # the real robots' are not measured yet (DOE: the first real session's face readings set it)
FACE_MIN = 4              # chosen: face readings this session before that check speaks (up to 8 come in a session)
FLOOR = dict(camera=0.3, wheels=0.2, gyro=0.3, front=1.0)    # learned sds are never taken below these
BASE_FLOOR = dict(wheels=0.25, gyro=0.3, front=2.0, face=1.0, runs=RUN_SPREAD_PRIOR)
                          # chosen (runs: measured, above): a baseline is never taken below these (cm / deg / cm /
                          # spread): the simulator's healthy robots sit at 0.11-0.28 cm, 0.04-0.41 deg, 1-3 cm; a
                          # session's 3-4 fits scatter by up to 2x about their own level (DOE: replay healthy
                          # sessions against each other's history at 0.2 / 0.25 / 0.35 and count false flags)
DEGRADED_K = 2.0          # chosen: twice its baseline = looks bad (DOE: the swap test at 1.5 / 2 / 3)
RECOVERED_K = 1.5         # chosen: back under this = healthy again (hysteresis: no flapping at the line)
MIN_N = dict(camera=4, wheels=3, gyro=3, front=6, runs=4)     # chosen: this session's items before a check speaks
CAMERA_FIND_MIN = 8       # chosen: find items (2 per find) before the finds speak for the camera (one odd find is
                          # outvoted). Finds count only once their robot's wheels are judged (_find_counts): with 8%
                          # jitter on every robot, 3 of the first 4 setup finds read 1.2-2.2 cm (2026-10-25 rerun). A
                          # camera bad from the start is caught standing still. DOE: the grid at 4 / 8 / 12 (bad camera
                          # caught how soon, false flags)
HIST_MIN = dict(wheels=3, gyro=3, front=6, runs=4)            # chosen: earlier items before they replace the prior
DECAY_PER_SESSION = 0.5   # chosen: an item's weight halves every new session
STRETCH_CAM_FACTOR = math.sqrt(2.0)   # a stretch has two camera ends
CAM_HEADING_HEALTHY_DEG = 1.0   # the camera's share of a stretch's heading rms when healthy: the 2026-10-23 morning's
                                # rms 1.39 deg = sqrt(gyro 1.0^2 + camera 1.0^2); x the camera's k with one robot
GATE_MAX_CM = 3.0         # chosen: the wheel fit's limit never above this, however noisy the camera looks (2026-10-24
                          # review: a shared term can't tell "the camera across the board" from "every robot's wheels")
STANDS_OUT = 1.5          # (2026-10-24; kept for the replays) the old gate: this robot's stretch rms 1.5 x the median
STANDS_OUT_FLAG = 3.0     # chosen: the second check flags a robot whose stretches are 3 x the healthy robots' median
                          # (the simulator's healthy robots differ by up to 1.5x; Right's 5% gyro jitter was 4.4x;
                          # DOE: the same replay at 2 / 3 / 4)
RUN_CHI_FLAG = 20.0       # chosen: chi2/run at which the runs check reaches DEGRADED_K. Healthy, real and simulated:
                          # at most 7.4 (Center on the slow-camera 15:45 session: k 1.2); the simulator's 5-10% jittered
                          # wheels 43 and up (k 2.9 and up). 20 sits midway on a log scale. For independent camera errors
                          # 3.4 is already 1-in-1000 with 8 runs; the margin is for camera errors shared within a run (a
                          # slow camera). DOE: replay all sessions at 10 / 20 / 40, and the simulator at 2 / 4% jitter for
                          # the smallest fault caught
RUN_CAM_FLOOR_CM = 0.3    # chosen: the camera's miss per stretch is never taken below this (a run of near-perfect
                          # stretches must not make an ordinary run-to-run difference look huge)
RUN_MIN_LEN_CM = 40.0     # chosen: a run's stretches must add up to this before its scale counts (camera noise of
                          # ~1 cm at each end is then under 2% of it; most runs have 150-300 cm)
GATE_K = 1.5              # chosen: refuse a wheel fit only beyond 1.5 x what the camera and healthy wheels explain
KC_GATE_SIGMA = 3.0       # chosen: a camera position this many sds from the belief means the belief is wrong
KC_GATE_WEIGHT = 0.9      # then the fix takes this much of the camera


def robust_sd(values):
    """sd of values about zero, robust to a few outliers: median |v| / 0.6745 (a normal's median absolute value)."""
    v = sorted(abs(x) for x in values)
    if not v:
        return None
    n = len(v)
    med = v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])
    return med / 0.6745


def weighted_rms(items):
    """[(value, weight)] -> sqrt(sum w v^2 / sum w), or None."""
    tw = sum(w for _, w in items)
    if tw <= 0:
        return None
    return math.sqrt(sum(w * v * v for v, w in items) / tw)


def rel_spread(values):
    """Relative spread of run scales about their median: MAD x 1.4826 / median (robust to one odd run)."""
    if len(values) < 2:
        return None
    med = statistics.median(values)
    if med <= 0:
        return None
    return statistics.median([abs(v - med) for v in values]) * 1.4826 / med


class SensorTrust:
    """Evidence per sensor and robot ('*' for the camera, which all robots share), and what it says."""

    def __init__(self, data=None):
        d = data or {}
        self.items = d.get("items", [])        # dict(sensor, robot, kind, v, w, t, session[, excl])
        self.session = d.get("session", 0)
        self.flags = d.get("flags", {})        # "sensor/robot" -> True while it looks bad
        self.events = d.get("events", [])      # [t, sensor, robot, "degraded" | "recovered", k]
        self.reasons = d.get("reasons", {})    # "sensor/robot" -> the check that flagged it (words)
        self.changed = d.get("changed", {})    # "sensor/robot" -> words: changed this session (e02_front)
        self.flag_session = d.get("flag_session", {})   # "sensor/robot" -> the session its flag was raised in
        self._cache, self._stamp = {}, None

    def _memo(self, key, fn):
        """The checks are re-read for every flag update (several a second at times): cached until the evidence
        changes (items added, a new session)."""
        stamp = (len(self.items), self.session, self.items[-1]["t"] if self.items else None,
                 tuple(sorted(k for k, v in self.flags.items() if v)))
        if stamp != self._stamp:
            self._cache, self._stamp = {}, stamp
        if key not in self._cache:
            self._cache[key] = fn()
        return self._cache[key]

    # --- evidence -----------------------------------------------------------------------------------------
    def new_session(self):
        # what a flagged sensor gave in the session it was flagged never becomes its baseline
        for it in self.items:
            if it.get("session") == self.session and self.flags.get("%s/%s" % (it["sensor"], it["robot"])):
                it["excl"] = 1
        self.session += 1
        self.changed = {}
        for it in self.items:
            it["w"] = round(it["w"] * DECAY_PER_SESSION, 6)
        self.items = [it for it in self.items if it["w"] >= 0.01]

    def add(self, sensor, robot, kind, v, t=None, w=1.0):
        self.items.append(dict(sensor=sensor, robot=robot, kind=kind, v=round(float(v), 5), w=w,
                               t=round(t or time.time(), 1), session=self.session))

    def camera_at_find(self, robot, resid_x, resid_y, t=None):
        """The camera's leave-one-out error at a find (cm, per axis). Kept with the robot that made it ("by"): a
        find's pose leans on that robot's wheels (camera_sd)."""
        for v in (resid_x, resid_y):
            self.add("camera", "*", "find", v, t)
            self.items[-1]["by"] = robot

    def camera_still(self, robot, sd_x, sd_y, t=None):
        """A robot standing still: its camera position's scatter over a few seconds (cm per axis). Needs no find,
        so a camera that is bad from the start is caught during setup (2026-10-24 simulator: with the camera off
        by 4 cm, the session never reached a find, and the front sensor was blamed instead)."""
        self.add("camera", "*", "still", sd_x, t)
        self.add("camera", "*", "still", sd_y, t)

    def stretch_fit(self, robot, rms_cm, rms_deg, n, t=None):
        """A self-calibration fit's rms over n stretches (with weight n, capped)."""
        w = min(float(n), 10.0) / 10.0
        if rms_cm is not None:
            self.add("wheels", robot, "stretch_rms", rms_cm, t, w)
        if rms_deg is not None:
            self.add("gyro", robot, "stretch_rms", rms_deg, t, w)

    def run_scale(self, robot, scale, length_cm, t=None, pairs=None):
        """One run's wheel scale on its own: the camera's length over the wheels' length, summed over its stretches
        (length_cm: the wheels' total). Short runs are not counted (RUN_MIN_LEN_CM). pairs: the run's stretches
        [(camera cm, wheels cm)]; each one's miss against the run's own scale is kept too ("run_resid"): that scatter is
        the camera's (a wheel fault that changes run to run can't show inside one run), and it says how far apart the
        runs' scales would be from the camera alone (_runs_check)."""
        if scale is None or not length_cm or length_cm < RUN_MIN_LEN_CM or scale <= 0:
            return False
        self.add("wheels", robot, "run_scale", scale, t)
        self.items[-1]["len"] = round(float(length_cm), 2)
        if pairs:
            self.items[-1]["n"] = len(pairs)
            for c, b in pairs:
                self.add("wheels", robot, "run_resid", c - scale * b, t)
        return True

    def front_vs_map(self, robot, resid_cm, t=None):
        self.add("front", robot, "map", resid_cm, t)

    def front_face(self, robot, resid_cm, t=None):
        """A front reading square to a board at a distance known from the board touches (no camera), after the
        sensor's learned offset."""
        self.add("front", robot, "face", resid_cm, t)

    def mark_changed(self, sensor, robot, words):
        """An independent check says this sensor reads differently from its own history (e02_front): flagged for the
        rest of the session (update_flags)."""
        self.changed["%s/%s" % (sensor, robot)] = words

    def _items(self, sensor, robot=None, kind=None):
        return [it for it in self.items if it["sensor"] == sensor and (robot is None or it["robot"] == robot)
                and (kind is None or it["kind"] == kind)]

    def _now(self, its):
        return [it for it in its if it.get("session") == self.session]

    def _hist(self, its):
        return [it for it in its if it.get("session", self.session) < self.session and not it.get("excl")]

    # --- the camera (one for all robots) ------------------------------------------------------------------
    def camera_sd(self):
        return self._memo(("camera_sd",), self._camera_sd)

    def _camera_sd(self):
        """(sd cm per axis, n): the camera at the finds, after its error map; or, if worse in proportion, its scatter
        standing still (scaled to the finds' healthy level). Healthy if too little evidence. Each kind from THIS
        session's evidence once it has MIN_N of it (2026-10-25 replay: pooled with a healthy history, a camera with 4 cm
        of noise was not flagged at all), earlier sessions' until then."""
        def sd_of(its):
            rep = []
            for it in its:
                rep += [it["v"]] * max(1, int(round(it["w"] * 10)))
            return robust_sd(rep)

        def fresh(its, m):
            now = self._now(its)
            return now if len(now) >= m else its
        judged = {}
        finds = fresh([it for it in self._items("camera", kind="find") if self._find_counts(it, judged)],
                      CAMERA_FIND_MIN)
        still = fresh(self._items("camera", kind="still"), MIN_N["camera"])
        best, n = HEALTHY["camera"], 0
        cands = []
        if len(finds) >= CAMERA_FIND_MIN:
            cands.append(sd_of(finds))
            n += len(finds)
        if len(still) >= MIN_N["camera"]:
            # a still scatter of CAMERA_STILL_HEALTHY counts as healthy (the finds' 0.8); worse scales up with it
            cands.append(sd_of(still) / CAMERA_STILL_HEALTHY * HEALTHY["camera"])
            n += len(still)
        if not cands:
            return best, len(finds) + len(still)
        return max(FLOOR["camera"], max(cands)), n

    def _find_counts(self, it, judged=None):
        """Does this find count against the camera? Its pose is the board touches plus the robot's own counting of
        its back-off and of the moves between the boards, so only once that robot's wheels have been judged on their
        own (MIN_N fits or runs this session) and are not flagged (Nick's rule, 2026-10-25: each calibrated on its
        own first; the wheels:jitter=0.08 rerun: setup finds at 1.2-2.2 cm flagged a healthy camera). Until then the
        camera is judged standing still. Items from before 2026-10-25 (no "by") count as before."""
        r = it.get("by")
        if r is None:
            return True
        if self.flags.get("wheels/%s" % r):
            return False
        if it.get("session") != self.session:
            return True
        judged = {} if judged is None else judged
        if r not in judged:
            n_fits = len(self._now(self._items("wheels", r, "stretch_rms")))
            n_runs = len(self._now(self._items("wheels", r, "run_scale")))
            judged[r] = n_fits >= MIN_N["wheels"] or n_runs >= MIN_N["runs"]
        return judged[r]

    def camera_known(self):
        """Is there enough evidence about the camera to blame another sensor's disagreement with it?"""
        return len(self._items("camera")) >= MIN_N["camera"]

    def camera_share(self, sensor):
        """The camera's share of a stretch's rms from the camera's OWN evidence (finds, standing still): never from
        what the robots have in common."""
        csd = self.camera_sd()[0]
        if sensor == "wheels":
            return STRETCH_CAM_FACTOR * csd
        return CAM_HEADING_HEALTHY_DEG * csd / HEALTHY["camera"]

    # --- what the robots share (the self-calibration's limit and a note only; never a flag) ----------------
    def shared_stretch(self, sensor):
        """The camera's share of the stretches' rms, from what all robots have in common: the robots' typical mean
        square less a healthy own share. None with fewer than two robots' evidence. Used for the self-calibration's
        limit (wheel_gate_cm), not to judge a sensor."""
        per = {}
        for it in self._items(sensor, kind="stretch_rms"):
            per.setdefault(it["robot"], []).append((it["v"], it["w"]))
        ms = sorted(weighted_rms(v) ** 2 for r, v in per.items()
                    if len(v) >= MIN_N[sensor] and weighted_rms(v) is not None)
        if len(ms) < 2:
            return None
        base = ms[0] if len(ms) == 2 else ms[len(ms) // 2] if len(ms) % 2 else 0.5 * (ms[len(ms) // 2 - 1] + ms[len(ms) // 2])
        return math.sqrt(max(0.0, base - HEALTHY[sensor] ** 2))

    def camera_stretch_share(self, sensor):
        """The camera's share of a stretch's rms for the self-calibration's limit: shared term (2+ robots) or from
        the finds (one robot)."""
        sh = self.shared_stretch(sensor)
        if sh is not None:
            return sh, "shared by the robots"
        return self.camera_share(sensor), "from the finds"

    def shared_unexplained(self, sensor="wheels"):
        """With 2+ robots: is what they all share far more than the camera at the finds explains? (A note in the
        summary: the camera away from the find corner, or every robot's wheels alike; the runs check tells.)"""
        sh = self.shared_stretch(sensor)
        if sh is None:
            return False
        return sh > 2.0 * self.camera_share(sensor)

    # --- each robot's own checks ----------------------------------------------------------------------------
    def _stretch_check(self, sensor, robot):
        c = self._memo(("stretch", sensor, robot), lambda: self._stretch_check0(sensor, robot))
        return dict(c) if c else None

    def _stretch_check0(self, sensor, robot):
        """This session's stretch rms against the robot's own earlier sessions (or the prior). dict or None."""
        its = self._items(sensor, robot, "stretch_rms")
        now = self._now(its)
        if len(now) < MIN_N[sensor]:
            return None
        v = weighted_rms([(it["v"], it["w"]) for it in now])
        hist = self._hist(its)
        if len(hist) >= HIST_MIN[sensor]:
            base, src = weighted_rms([(it["v"], it["w"]) for it in hist]), "its own history (%d fits)" % len(hist)
        else:
            base = math.sqrt(HEALTHY[sensor] ** 2 + self.camera_share(sensor) ** 2)
            src = "the Alvik prior"
        base = max(base, BASE_FLOOR[sensor])
        return dict(check="stretches", v=v, base=base, k=v / base, n=len(now), src=src)

    def _runs_check(self, robot):
        c = self._memo(("runs", robot), lambda: self._runs_check0(robot))
        return dict(c) if c else None

    def _runs_check0(self, robot):
        """Do this robot's runs disagree on its wheel scale by more than the camera explains? (2026-10-25, real session
        15:45: Center's wheels flagged by their stretches at 2.3 x, but the error was the same on short and long
        stretches and its scale held 0.963-0.981: the camera's error, charged to the wheels.) Each run's scale has a
        scatter from the camera alone: the camera's miss per stretch (its stretches about their own run's scale)
        x sqrt(stretches) / the run's length. chi2/run (fine print: each run's scale minus the median, squared, over
        that camera-only scatter squared; the median of those x 2.2, so one odd run doesn't move it) is about 1 when
        the camera explains the spread: healthy robots at most 7.4 (real, the slow-camera 15:45 session), the
        simulator's jittered wheels 43 and up. k = DEGRADED_K x sqrt(chi2/run / RUN_CHI_FLAG). Evidence from before
        (no stretches kept) is judged by the spread against the prior, as before."""
        its = self._items("wheels", robot, "run_scale")
        now_its = self._now(its)
        now = [it["v"] for it in now_its]
        if len(now) < MIN_N["runs"]:
            return None
        res = [it["v"] for it in self._now(self._items("wheels", robot, "run_resid"))]
        if all(it.get("n") and it.get("len") for it in now_its) and len(res) >= len(now) + 2:
            a = robust_sd(res) ** 2 * len(res) / max(1, len(res) - len(now))
            a = max(a, RUN_CAM_FLOOR_CM ** 2)
            med = statistics.median(now)
            z2 = [((it["v"] - med) / (math.sqrt(it["n"] * a) / it["len"])) ** 2 for it in now_its]
            chi = statistics.median(z2) / 0.455
            return dict(check="runs", v=chi, base=1.0, k=DEGRADED_K * math.sqrt(chi / RUN_CHI_FLAG), n=len(now),
                        src="the camera's own scatter, %.1f cm a stretch" % math.sqrt(a), spread=rel_spread(now))
        v = rel_spread(now)
        if v is None:
            return None
        per = {}
        for it in self._hist(its):
            per.setdefault(it["session"], []).append(it["v"])
        devs = []
        for vals in per.values():                    # about each session's own median: a refit between sessions
            if len(vals) >= 2:                       # is not a spread
                med = statistics.median(vals)
                devs += [(x - med) / med for x in vals]
        if len(devs) >= HIST_MIN["runs"]:
            base = statistics.median([abs(d) for d in devs]) * 1.4826
            src = "its own history (%d runs)" % len(devs)
        else:
            base, src = RUN_SPREAD_PRIOR, "the prior"
        base = max(base, BASE_FLOOR["runs"])
        return dict(check="runs", v=v, base=base, k=v / base, n=len(now), src=src)

    def _front_own(self, its, kind):
        """The front sensor's own sd from items of one kind (map: the camera's share taken off; face: as it is)."""
        if not its:
            return None
        sd = robust_sd([it["v"] for it in its])
        if kind == "map":
            sd = math.sqrt(max(0.0, sd ** 2 - self.camera_sd()[0] ** 2))
        return max(FLOOR["front"], sd)

    def _front_check(self, robot, kind="map"):
        c = self._memo(("front", robot, kind), lambda: self._front_check0(robot, kind))
        return dict(c) if c else None

    def _front_check0(self, robot, kind):
        """map: readings against the camera-and-map distance (prior HEALTHY front). face: readings square to a board
        at a distance known from the board touches, no camera (prior FACE_HEALTHY): two checks, since a face reading
        is far tighter than a map reading (2026-10-25 rerun: Left's 6 cm noise read 4 cm at a board and 9.8 cm against
        the map, Center and Right 1 cm at a board; one 5 cm bar for both let it pass)."""
        its = [it for it in self._items("front", robot) if it["kind"] == kind]
        now = self._now(its)
        if len(now) < (MIN_N["front"] if kind == "map" else FACE_MIN):
            return None
        v = self._front_own(now, kind)
        hist = self._hist(its)
        prior = HEALTHY["front"] if kind == "map" else FACE_HEALTHY
        if len(hist) >= (HIST_MIN["front"] if kind == "map" else FACE_MIN):
            base, src = self._front_own(hist, kind), "its own history (%d readings)" % len(hist)
        else:
            base, src = prior, "the Alvik prior" if kind == "map" else "the prior"
        base = max(base, BASE_FLOOR["front"] if kind == "map" else BASE_FLOOR["face"])
        return dict(check="front, " + ("against the map" if kind == "map" else "at a board"), v=v, base=base,
                    k=v / base, n=len(now), src=src)

    def checks(self, sensor, robot):
        """This robot's own checks of one sensor (no other robot involved): [dict(check, v, base, k, n, src)]."""
        if sensor == "wheels":
            st = self._stretch_check("wheels", robot)
            if st is not None and self._runs_check(robot) is not None and self._runs_check(robot)["src"].startswith(
                    "the camera's own"):
                # with the camera-corrected runs check in place the stretches only inform: on the real robots their
                # rms is mostly the camera's (15:45 session: the same on short and long stretches)
                st["info"] = True
            out = [st, self._runs_check(robot)]
        elif sensor == "gyro":
            out = [self._stretch_check("gyro", robot)]
        elif sensor == "front":
            out = [self._front_check(robot, "map"), self._front_check(robot, "face")]
        else:
            return []
        return [c for c in out if c is not None]

    def second_check(self, sensor, robot, robots):
        """After every robot has its own verdict from this session: does this robot's stretch rms (wheels, gyro) or
        its face readings (front: the same camera-free test for every robot) stand out from the robots that look
        healthy? dict(check, v, base, k, ...) with k scaled so that STANDS_OUT_FLAG x = DEGRADED_K, or None."""
        if sensor not in ("wheels", "gyro", "front") or len(robots) < 2:
            return None
        get = (lambda r: self._front_check(r, "face")) if sensor == "front" else \
            (lambda r: self._stretch_check(sensor, r))
        mine = get(robot)
        if mine is None:
            return None
        others = []
        for r in robots:
            if r == robot:
                continue
            c = get(r)
            if c is None:
                return None                          # not every robot judged on its own yet: no comparing
            if not self.flags.get("%s/%s" % (sensor, r)):
                others.append(c["v"])
        if not others:
            return None
        med = statistics.median(others)
        base = max(med, BASE_FLOOR["face" if sensor == "front" else sensor])
        ratio = mine["v"] / base
        return dict(check="stands out from the other robots", v=mine["v"], base=base, ratio=ratio,
                    k=ratio * DEGRADED_K / STANDS_OUT_FLAG, n=mine["n"], src="%d other robots" % len(others))

    def judge(self, sensor, robot, robots=None):
        """dict(k, worst, checks): k the worst check's (1.0 with no check yet). robots: for the second check. (No
        "every robot alike" exemption: it hid Right's 8% wheels in the trained replay, and the real all-alike rises
        (2026-10-24 Pi stage 2: 1.6-1.9 x) stay under the flag line without it: trust_fix_real_replay.py.)"""
        cs = self.checks(sensor, robot)
        sc = self.second_check(sensor, robot, robots or [])
        if sc is not None:
            cs.append(sc)
        live = [c for c in cs if not c.get("info")]
        if not live:
            return dict(k=1.0, worst=None, checks=cs, live=[])
        worst = max(live, key=lambda c: c["k"])
        return dict(k=worst["k"], worst=worst, checks=cs, live=live)

    # --- the old views (the viewer, the summary, the replays) ---------------------------------------------
    def own_sd(self, sensor, robot):
        """(sd, n, how): a robot's wheels or gyro, its stretch rms this session (all sessions while this one has
        too little), with its baseline in words."""
        c = self._stretch_check(sensor, robot)
        if c is not None:
            return c["v"], c["n"], "baseline %.2f from %s" % (c["base"], c["src"])
        its = self._items(sensor, robot, "stretch_rms")
        if len(its) < MIN_N[sensor]:
            return HEALTHY[sensor], len(its), "too little evidence yet"
        return weighted_rms([(it["v"], it["w"]) for it in its]), len(its), "earlier sessions"

    def front_sd(self, robot):
        """(sd, n): the front sensor's own sd this session, the worse of its map and face checks (all sessions' map
        readings while this one has too little)."""
        cs = [c for c in (self._front_check(robot, "map"), self._front_check(robot, "face")) if c]
        if cs:
            c = max(cs, key=lambda c: c["k"])
            return c["v"], c["n"]
        its = [it for it in self._items("front", robot) if it["kind"] == "map"]
        if len(its) < MIN_N["front"]:
            return HEALTHY["front"], len(self._items("front", robot))
        return self._front_own(its, "map"), len(its)

    def k(self, sensor, robot="*", robots=None):
        """The trust factor: this session's value / the sensor's own baseline, the worst check (1 = as usual)."""
        if sensor == "camera":
            return self.camera_sd()[0] / HEALTHY["camera"]
        return self.judge(sensor, robot, robots)["k"]

    def bad(self, sensor, robot="*"):
        return bool(self.flags.get("%s/%s" % (sensor, robot)))

    def stands_out(self, sensor, robot):
        """(2026-10-24; replays only) is this robot's stretch rms at least STANDS_OUT x the robots' median? No longer
        used for flags: a robot is judged on its own first (Nick, 2026-10-25)."""
        per = {}
        for it in self._items(sensor, kind="stretch_rms"):
            per.setdefault(it["robot"], []).append((it["v"], it["w"]))
        rms = {r: weighted_rms(v) for r, v in per.items() if len(v) >= MIN_N[sensor]}
        if len(rms) < 2 or robot not in rms:
            return True
        vals = sorted(rms.values())
        med = vals[len(vals) // 2] if len(vals) % 2 else 0.5 * (vals[len(vals) // 2 - 1] + vals[len(vals) // 2])
        return rms[robot] >= STANDS_OUT * med

    def update_flags(self, robots):
        """Flags with hysteresis; returns the changes [(sensor, robot, 'degraded' | 'recovered', k)]; the check that
        decided is in self.reasons. While the camera is unknown or looks bad, no wheel, gyro or front sensor is newly
        flagged: their checks are all against the camera, so a disagreement can't be blamed on them."""
        out = []
        kk = self.k("camera")
        if not self.flags.get("camera/*") and kk >= DEGRADED_K:
            self.flags["camera/*"] = True
            self.reasons["camera/*"] = "the camera at the finds / standing still: %.1f x healthy" % kk
            out.append(("camera", "*", "degraded", kk))
        elif self.flags.get("camera/*") and kk < RECOVERED_K:
            self.flags["camera/*"] = False
            out.append(("camera", "*", "recovered", kk))
        cam_ok = self.camera_known() and not self.bad("camera")
        # first every robot on its own; then (second check) against the others
        for s in ("wheels", "gyro", "front"):
            for r in robots:
                key = "%s/%s" % (s, r)
                j = self.judge(s, r, robots)
                kk, w = j["k"], j["worst"]
                ch = self.changed.get(key)
                if not self.flags.get(key):
                    if not cam_ok:
                        continue
                    if ch or kk >= DEGRADED_K:
                        self.flags[key] = True
                        self.flag_session[key] = self.session
                        self.reasons[key] = ("changed: " + ch) if ch else self._words(w)
                        out.append((s, r, "degraded", max(kk, DEGRADED_K) if ch and kk < DEGRADED_K else kk))
                elif not ch and kk < RECOVERED_K and j["live"] and self.flag_session.get(key) != self.session:
                    # a robot's sensor flagged this session stays flagged to its end (2026-10-25 replay: Center's
                    # wheels at 8% flagged, cleared and flagged again as its few runs' spread wobbled); the next
                    # session's evidence clears it
                    self.flags[key] = False
                    out.append((s, r, "recovered", kk))
        for s, r, what, k_ in out:
            self.events.append([round(time.time(), 1), s, r, what, round(k_, 2)])
        return out

    @staticmethod
    def _words(c):
        if not c:
            return ""
        if c["check"] == "runs" and "spread" in c:
            return "runs: scale %.1f%% apart run to run, %.0f x what the camera explains (chi2/run; %s)" % (
                100 * (c["spread"] or 0), c["v"], c["src"])
        if c["check"] == "runs":
            return "runs: scale %.1f%% apart run to run, baseline %.1f%% (%s)" % (100 * c["v"], 100 * c["base"], c["src"])
        if "ratio" in c:
            return "%s: %.2f, %.1f x their median %.2f (second check)" % (c["check"], c["v"], c["ratio"], c["base"])
        return "%s: %.2f, baseline %.2f (%s)" % (c["check"], c["v"], c["base"], c["src"])

    def reason(self, sensor, robot="*"):
        return self.reasons.get("%s/%s" % (sensor, robot), "")

    # --- the runner's uses ----------------------------------------------------------------------------------
    def wheel_gate_cm(self, override=None):
        """The self-calibration's limit for camera against wheels (rms per stretch): GATE_K x what the camera's
        share and healthy wheels explain (2026-10-24 Pi plan: a fixed 1.5 cm refused every fit when the camera
        file was noisier than the laptop's)."""
        if override is not None:
            return override
        cam, _ = self.camera_stretch_share("wheels")
        return min(GATE_MAX_CM, GATE_K * math.sqrt(cam ** 2 + HEALTHY["wheels"] ** 2))

    def kc_sds(self, robot, robot_sd, camera_sd):
        """KC: the model's sds scaled by what has been learned (the robot's own position: its wheels' k, while they
        are flagged; the camera: its k)."""
        kw = self.k("wheels", robot) if self.bad("wheels", robot) else 1.0
        return robot_sd * max(1.0, kw), camera_sd * max(1.0, self.k("camera"))

    def summary(self, robots):
        """Plain lines for the log."""
        csd, cn = self.camera_sd()
        lines = ["camera: %.1f cm per axis at the finds (%d readings; healthy %.1f)%s" % (
            csd, cn, HEALTHY["camera"], " LOOKS BAD" if self.bad("camera") else "")]
        if self.shared_unexplained("wheels"):
            lines.append("every robot's stretches are noisier than the camera at the finds explains: the camera away "
                         "from the find corner, or every robot's wheels alike (each robot's runs check tells which)")
        for r in robots:
            parts = []
            for s, unit in (("wheels", "cm a stretch"), ("gyro", "deg a stretch"), ("front", "cm")):
                j = self.judge(s, r, robots)
                if not j["checks"]:
                    parts.append("%s: too little evidence yet" % s)
                    continue
                cs = []
                for c in j["checks"]:
                    if c["check"] == "runs" and "spread" in c:
                        cs.append("runs %.1f%% apart (%d), chi2/run %.1f (camera alone: 1; %s)" % (
                            100 * (c["spread"] or 0), c["n"], c["v"], c["src"]))
                    elif c["check"] == "runs":
                        cs.append("runs %.1f%% apart (%d; baseline %.1f%%, %s)" % (100 * c["v"], c["n"], 100 * c["base"],
                                                                                   c["src"]))
                    elif "ratio" in c:
                        cs.append("%.1f x the other robots" % c["ratio"])
                    else:
                        cs.append("%s%.2f %s (%d; baseline %.2f, %s)%s" % (
                            (c["check"].split(", ")[1] + " ") if s == "front" else "", c["v"], unit, c["n"], c["base"],
                            c["src"], ", shown only: the runs check decides" if c.get("info") else ""))
                parts.append("%s k %.1f: %s" % (s, j["k"], "; ".join(cs)))
            bad = [s for s in ("wheels", "gyro", "front") if self.bad(s, r)]
            lines.append("%s: %s%s" % (r, " | ".join(parts), (" LOOKS BAD: " + ", ".join(bad)) if bad else ""))
        return lines

    def to_dict(self):
        return dict(items=self.items, session=self.session, flags=self.flags, events=self.events[-200:],
                    reasons=self.reasons, changed=self.changed, flag_session=self.flag_session)


def load(path):
    try:
        with open(path) as f:
            return SensorTrust(json.load(f))
    except (OSError, ValueError):
        return SensorTrust()


def save(st, path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(st.to_dict(), f)
    os.replace(tmp, path)


def kc_gate(belief, camera, robot_sd, camera_sd):
    """KC's check of the robot's belief against the camera (2026-10-24 Pi plan: Right's KC run started 12 cm from
    where it believed and the first fix gave the camera only 20%). Returns (weight towards the camera, rejected?):
    the usual variance weighting, unless the camera is more than KC_GATE_SIGMA combined sds away, which says the
    belief is wrong (then KC_GATE_WEIGHT)."""
    vr, vc = robot_sd ** 2, camera_sd ** 2
    w = vr / (vr + vc) if vr + vc > 0 else 0.5
    d = math.hypot(camera[0] - belief[0], camera[1] - belief[1])
    if d > KC_GATE_SIGMA * math.sqrt(vr + vc) and w < KC_GATE_WEIGHT:
        return KC_GATE_WEIGHT, True
    return w, False
