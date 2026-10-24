"""How much to trust each sensor, learned by the system itself (Nick, 2026-10-24: "the system needs to learn how
to weigh each sensor's accuracy confidence for itself. I want to be able to swap out any one of those sensors for
an ineffective version and have the system catch the error and account for it").

Four sensors: the overhead camera, each robot's wheels, its gyro and its front distance sensor. For each, a learned
noise (sd) from how well it agrees with something else, and a trust factor k = learned sd / healthy sd (1 = as
healthy; DEGRADED_K or more = this sensor looks bad, and it is said loudly). Where the evidence comes from:

  camera   the finds: right after a find the robot's position is known from touching both boards (to about a
           millimetre); the camera's position there, corrected by its error map built from the OTHER finds
           (leave one out), against it. Per axis.
  wheels   the self-calibration's stretches (camera against wheels, rms per stretch, e02_fit). A stretch's rms
           holds the camera's share and the wheels' share; which is which comes from what the robots share:
           the camera is the same for every robot, the wheels are each robot's own. With two or more robots, the
           part common to all of them is the camera's (the shared term); what one robot has on top is its wheels.
           With one robot, the camera's share comes from the finds instead.
  gyro     the same, with the stretches' heading rms (deg).
  front    the front sensor against the map (front_checks: reading minus the camera-and-map distance ahead). The
           camera's share (its sd) is taken off.

How the runner uses them (e02.py): the self-calibration's limit for camera against wheels comes from the camera's
shared term and the healthy wheel sd (not a fixed 1.5 cm); KC's blend weights use the learned sds (the robot's own
uncertainty x its wheels' k, the camera's x the camera's k) and refuse a belief the camera contradicts far beyond
both; a front sensor that looks bad no longer makes the camera suspect; every sensor's state is in the log, the
state file and the session summary.

Persisted between sessions (logs/sensor_trust.json): each evidence item keeps a weight that halves every new session
(DECAY_PER_SESSION), so the trust follows a sensor that changes (a swapped part) within a session or two.

The healthy sds are the median of the 2026-10-23 and 2026-10-24 sessions (numbers beside them). All limits are
chosen, not measured: DOE notes beside them. Pure functions and a small class; tests in tests/test_e02_trust.py."""
import json
import math
import os
import time

# healthy noise (1 sd), from real sessions (2026-10-24, tools: this module's docstring)
HEALTHY = dict(
    camera=0.8,    # cm per axis at a find, after the error map (leave one out): 0.71 / 1.03 on 2026-10-24's stages
    wheels=0.6,    # cm per stretch, the wheels' own share: the 2026-10-23 morning's rms 1.25 with the camera's 0.8
    gyro=1.0,      # deg per stretch, the gyro's own share (the morning's rms 1.39 less the camera's heading share)
    front=4.5,     # cm, front sensor against the map, robust sd (MAD x 1.48): 4.0-4.6 in four sessions
)
FLOOR = dict(camera=0.3, wheels=0.2, gyro=0.3, front=1.0)    # learned sds are never taken below these
DEGRADED_K = 2.0          # chosen: twice its healthy noise = looks bad (DOE: the swap test at 1.5 / 2 / 3)
RECOVERED_K = 1.5         # chosen: back under this = healthy again (hysteresis: no flapping at the line)
MIN_N = dict(camera=4, wheels=3, gyro=3, front=6)     # chosen: evidence items before a sensor's k is used
DECAY_PER_SESSION = 0.5   # chosen: an item's weight halves every new session
STRETCH_CAM_FACTOR = math.sqrt(2.0)   # a stretch has two camera ends
CAM_HEADING_LEN_CM = 40.0             # a typical stretch: the camera's heading noise is about sqrt(2) sd / length
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


class SensorTrust:
    """Evidence per sensor and robot ('*' for the camera, which all robots share), and what it says."""

    def __init__(self, data=None):
        d = data or {}
        self.items = d.get("items", [])        # dict(sensor, robot, kind, v, w, t, session)
        self.session = d.get("session", 0)
        self.flags = d.get("flags", {})        # "sensor/robot" -> True while it looks bad
        self.events = d.get("events", [])      # [t, sensor, robot, "degraded" | "recovered", k]

    # --- evidence -----------------------------------------------------------------------------------------
    def new_session(self):
        self.session += 1
        for it in self.items:
            it["w"] = round(it["w"] * DECAY_PER_SESSION, 6)
        self.items = [it for it in self.items if it["w"] >= 0.01]

    def add(self, sensor, robot, kind, v, t=None):
        self.items.append(dict(sensor=sensor, robot=robot, kind=kind, v=round(float(v), 4), w=1.0,
                               t=round(t or time.time(), 1), session=self.session))

    def camera_at_find(self, robot, resid_x, resid_y, t=None):
        """The camera's leave-one-out error at a find (cm, per axis)."""
        self.add("camera", "*", "find", resid_x, t)
        self.add("camera", "*", "find", resid_y, t)

    def stretch_fit(self, robot, rms_cm, rms_deg, n, t=None):
        """A self-calibration fit's rms over n stretches: kept as the mean square (with weight n, capped)."""
        w = min(float(n), 10.0) / 10.0
        if rms_cm is not None:
            self.items.append(dict(sensor="wheels", robot=robot, kind="stretch_rms", v=round(rms_cm, 4), w=w,
                                   t=round(t or time.time(), 1), session=self.session))
        if rms_deg is not None:
            self.items.append(dict(sensor="gyro", robot=robot, kind="stretch_rms", v=round(rms_deg, 4), w=w,
                                   t=round(t or time.time(), 1), session=self.session))

    def front_vs_map(self, robot, resid_cm, t=None):
        self.add("front", robot, "map", resid_cm, t)

    def _items(self, sensor, robot=None):
        return [it for it in self.items if it["sensor"] == sensor and (robot is None or it["robot"] == robot)]

    # --- what the evidence says -----------------------------------------------------------------------------
    def camera_sd(self):
        """(sd cm per axis, n): the camera at the finds, after its error map. Healthy if too little evidence."""
        its = self._items("camera")
        if len(its) < MIN_N["camera"]:
            return HEALTHY["camera"], len(its)
        # robust: the weighted items' magnitudes, repeated by weight in tenths
        rep = []
        for it in its:
            rep += [it["v"]] * max(1, int(round(it["w"] * 10)))
        return max(FLOOR["camera"], robust_sd(rep)), len(its)

    def shared_stretch(self, sensor):
        """The camera's share of the stretches' rms (cm or deg), from what all robots have in common: the robots'
        typical mean square less a healthy own share. None with fewer than two robots' evidence."""
        per = {}
        for it in self._items(sensor):
            per.setdefault(it["robot"], []).append((it["v"], it["w"]))
        ms = sorted(weighted_rms(v) ** 2 for r, v in per.items()
                    if len(v) >= MIN_N[sensor] and weighted_rms(v) is not None)
        if len(ms) < 2:
            return None
        # two robots: the cleaner one (a bad one must not raise the camera's share); three or more: the median
        # (one bad robot can't move it, and one unusually clean robot doesn't make the others look bad)
        base = ms[0] if len(ms) == 2 else ms[len(ms) // 2] if len(ms) % 2 else 0.5 * (ms[len(ms) // 2 - 1] + ms[len(ms) // 2])
        return math.sqrt(max(0.0, base - HEALTHY[sensor] ** 2))

    def camera_stretch_share(self, sensor):
        """The camera's share of a stretch's rms: shared term (2+ robots) or from the finds (one robot)."""
        sh = self.shared_stretch(sensor)
        if sh is not None:
            return sh, "shared by the robots"
        csd = self.camera_sd()[0]
        if sensor == "wheels":
            return STRETCH_CAM_FACTOR * csd, "from the finds"
        return math.degrees(STRETCH_CAM_FACTOR * csd / CAM_HEADING_LEN_CM), "from the finds"

    def own_sd(self, sensor, robot):
        """(sd, n, how): a robot's wheels or gyro, its own share of the stretches' rms (healthy if too little)."""
        its = self._items(sensor, robot)
        if len(its) < MIN_N[sensor]:
            return HEALTHY[sensor], len(its), "too little evidence yet"
        rms = weighted_rms([(it["v"], it["w"]) for it in its])
        cam, how = self.camera_stretch_share(sensor)
        return max(FLOOR[sensor], math.sqrt(max(0.0, rms ** 2 - cam ** 2))), len(its), "camera's share " + how

    def front_sd(self, robot):
        its = self._items("front", robot)
        if len(its) < MIN_N["front"]:
            return HEALTHY["front"], len(its)
        vals = [it["v"] for it in its]
        med = sorted(vals)[len(vals) // 2]
        sd = robust_sd([v - med for v in vals])
        csd = self.camera_sd()[0]
        return max(FLOOR["front"], math.sqrt(max(0.0, sd ** 2 - csd ** 2))), len(its)

    def k(self, sensor, robot="*"):
        """The trust factor: learned sd / healthy sd (1 = healthy)."""
        if sensor == "camera":
            return self.camera_sd()[0] / HEALTHY["camera"]
        if sensor == "front":
            return self.front_sd(robot)[0] / HEALTHY["front"]
        return self.own_sd(sensor, robot)[0] / HEALTHY[sensor]

    def bad(self, sensor, robot="*"):
        return bool(self.flags.get("%s/%s" % (sensor, robot)))

    def update_flags(self, robots):
        """Flags with hysteresis; returns the changes [(sensor, robot, 'degraded' | 'recovered', k)]."""
        out = []
        keys = [("camera", "*")] + [(s, r) for r in robots for s in ("wheels", "gyro", "front")]
        for s, r in keys:
            kk = self.k(s, r)
            key = "%s/%s" % (s, r)
            if not self.flags.get(key) and kk >= DEGRADED_K:
                self.flags[key] = True
                out.append((s, r, "degraded", kk))
            elif self.flags.get(key) and kk < RECOVERED_K:
                self.flags[key] = False
                out.append((s, r, "recovered", kk))
        for s, r, what, kk in out:
            self.events.append([round(time.time(), 1), s, r, what, round(kk, 2)])
        return out

    # --- the runner's uses ----------------------------------------------------------------------------------
    def wheel_gate_cm(self, override=None):
        """The self-calibration's limit for camera against wheels (rms per stretch): GATE_K x what the camera's
        share and healthy wheels explain (2026-10-24 Pi plan: a fixed 1.5 cm refused every fit when the camera
        file was noisier than the laptop's)."""
        if override is not None:
            return override
        cam, _ = self.camera_stretch_share("wheels")
        return GATE_K * math.sqrt(cam ** 2 + HEALTHY["wheels"] ** 2)

    def kc_sds(self, robot, robot_sd, camera_sd):
        """KC: the model's sds scaled by what has been learned (the robot's own position: its wheels' k; the
        camera: its k)."""
        return robot_sd * max(1.0, self.k("wheels", robot)), camera_sd * max(1.0, self.k("camera"))

    def summary(self, robots):
        """Plain lines for the log."""
        csd, cn = self.camera_sd()
        lines = ["camera: %.1f cm per axis at the finds (%d readings; healthy %.1f)%s" % (
            csd, cn, HEALTHY["camera"], " LOOKS BAD" if self.bad("camera") else "")]
        for r in robots:
            ws, wn, how = self.own_sd("wheels", r)
            gs, gn, _ = self.own_sd("gyro", r)
            fs, fn = self.front_sd(r)
            bad = [s for s in ("wheels", "gyro", "front") if self.bad(s, r)]
            lines.append("%s: wheels %.1f cm a stretch (%d fits), gyro %.1f deg (%d), front %.1f cm (%d)%s; %s" % (
                r, ws, wn, gs, gn, fs, fn, (" LOOKS BAD: " + ", ".join(bad)) if bad else "", how))
        return lines

    def to_dict(self):
        return dict(items=self.items, session=self.session, flags=self.flags, events=self.events[-200:])


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
