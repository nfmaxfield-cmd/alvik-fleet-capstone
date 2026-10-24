"""A sensor made bad on purpose, in software (Nick, 2026-10-24: "runs in which we deliberately degrade a sensor's
input and see if the program can adapt"; software only). Every degraded session says so on every run's line and
in its state, and the analysis keeps it apart.

    --degrade camera:noise=3            the camera's positions off by a random amount (sd 3 cm per axis) that holds
                                        for HOLD_S seconds and then changes (a still robot is not averaged back to
                                        the truth; a wobbly mount or a bad lens does the same)
    --degrade camera:bias=2/-1          the camera's positions 2 cm off in x and -1 in y everywhere
    --degrade camera:drop=0.3           30% of the camera's positions lost (the tracker "lost it" more often)
    --degrade wheels:jitter=0.05        every drive's wheel counting off by a random factor (sd 5%), new for every run
                                        or move: the robot drives by it AND reports it (its own position and the
                                        self-calibration's raw counts both wrong by the same factor)
    --degrade gyro:jitter=0.03          the same for the gyro's turns
    --degrade gyro:drift=0.2            0.2 deg/s more drift than it has
    --degrade front:bias=5              the front sensor reads 5 cm long
    --degrade front:noise=3             the front sensor's readings with random noise (sd 3 cm)
Several at once: comma-separated (camera:noise=3,front:bias=5).

The amounts are chosen per test; the swap test's DOE (docs/build-plans/e02-oct22-build.md) sets the grid."""
import math
import random

HOLD_S = 2.0      # chosen: how long a camera noise offset holds (longer than KC's still pictures, 0.3 s)


def parse(spec):
    """'camera:noise=3,wheels:jitter=0.05' -> {'camera': {'noise': 3.0}, 'wheels': {'jitter': 0.05}}."""
    out = {}
    for part in (spec or "").split(","):
        part = part.strip()
        if not part:
            continue
        sensor, _, rest = part.partition(":")
        key, _, val = rest.partition("=")
        sensor, key = sensor.strip().lower(), key.strip().lower()
        if sensor not in ("camera", "wheels", "gyro", "front") or not key:
            raise ValueError("--degrade: %r is not sensor:what=amount (camera, wheels, gyro, front)" % part)
        if key == "bias" and "/" in val:
            out.setdefault(sensor, {})[key] = tuple(float(v) for v in val.split("/"))
        else:
            out.setdefault(sensor, {})[key] = float(val)
    return out


class Degrade:
    def __init__(self, spec, seed=0):
        self.spec = parse(spec) if isinstance(spec, str) else dict(spec or {})
        self.rng = random.Random(seed)
        self._cam_off = (0.0, 0.0)
        self._cam_until = -1.0
        self.factors = {}           # (robot, sensor) -> the factor of its current run or move

    def __bool__(self):
        return bool(self.spec)

    def words(self):
        if not self.spec:
            return ""
        return "DEGRADED ON PURPOSE: " + ", ".join("%s %s" % (s, " ".join("%s=%s" % kv for kv in d.items()))
                                                   for s, d in self.spec.items())

    # camera: a sample (t, x, y, ...) -> the degraded sample, or None (dropped)
    def camera(self, s):
        c = self.spec.get("camera")
        if not c or s is None:
            return s
        if c.get("drop") and self.rng.random() < c["drop"]:
            return None
        x, y = s[1], s[2]
        b = c.get("bias")
        if b:
            bx, by = (b, 0.0) if not isinstance(b, tuple) else (b[0], b[1] if len(b) > 1 else 0.0)
            x, y = x + bx, y + by
        if c.get("noise"):
            if s[0] >= self._cam_until:
                self._cam_off = (self.rng.gauss(0.0, c["noise"]), self.rng.gauss(0.0, c["noise"]))
                self._cam_until = s[0] + HOLD_S
            x, y = x + self._cam_off[0], y + self._cam_off[1]
        return (s[0], x, y) + tuple(s[3:])

    # wheels and gyro: the factor for a robot's next run or move (new every call)
    def new_factors(self, robot):
        for sensor in ("wheels", "gyro"):
            j = (self.spec.get(sensor) or {}).get("jitter")
            self.factors[(robot, sensor)] = max(0.5, 1.0 + self.rng.gauss(0.0, j)) if j else 1.0
        return self.factors[(robot, "wheels")], self.factors[(robot, "gyro")]

    def corrections(self, robot, values):
        """The corrections sent to the robot: its wheels and gyro scale x this run's factors, its drift + the extra."""
        fw, fg = self.new_factors(robot)
        v = dict(values)
        v["wheel_scale"] = v["wheel_scale"] * fw
        v["gyro_scale"] = v["gyro_scale"] * fg
        v["gyro_drift_dps"] = v["gyro_drift_dps"] + (self.spec.get("gyro") or {}).get("drift", 0.0)
        return v

    def raw_rows(self, robot, rows):
        """The robot's RAW rows as the self-calibration reads them: distance from the run's first row x the wheels'
        factor, raw yaw x the gyro's (what a bad encoder or gyro reports)."""
        fw = self.factors.get((robot, "wheels"), 1.0)
        fg = self.factors.get((robot, "gyro"), 1.0)
        if (fw == 1.0 and fg == 1.0) or not rows:
            return rows
        x0, y0, yaw0 = rows[0].get("x_cm"), rows[0].get("y_cm"), rows[0].get("yaw_deg")
        out = []
        prev, turned = yaw0, 0.0          # the raw yaw wraps at +-180: its turning is unwrapped before scaling
        for r in rows:
            q = dict(r)
            if None not in (x0, y0, r.get("x_cm"), r.get("y_cm")):
                q["x_cm"] = x0 + fw * (r["x_cm"] - x0)
                q["y_cm"] = y0 + fw * (r["y_cm"] - y0)
            if None not in (yaw0, prev, r.get("yaw_deg")):
                turned += (r["yaw_deg"] - prev + 180.0) % 360.0 - 180.0
                prev = r["yaw_deg"]
                q["yaw_deg"] = (yaw0 + fg * turned + 180.0) % 360.0 - 180.0
            out.append(q)
        return out

    def front(self, cm):
        f = self.spec.get("front")
        if not f or cm is None or cm <= 0:
            return cm
        cm = cm + f.get("bias", 0.0)
        if f.get("noise"):
            cm += self.rng.gauss(0.0, f["noise"])
        return max(0.1, cm)
