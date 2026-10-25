"""Synthetic calibration-battery logs: a two-wheeled robot doing short test moves, with the Alvik's RAW columns,
for judging which moves a discovery calibration run should include (logs only; nothing here touches a robot).

Why not tools/sim: its IMU has no tilt (gx, gy are bias + noise; roll and pitch are noise), so it cannot show what a
tilt or a lift adds. tests/synth.py has no tilt either. This model adds the physics those moves need:
  - a tilt (by hand) turns the x or y gyro at the tilt rate and moves gravity between the accelerometer axes
    (ax = sin pitch, ay = -sin roll cos pitch, az = -cos roll cos pitch, in g, az reading -1 flat as the Alvik does);
  - a lift adds the vertical acceleration to az, lifts the line sensors off the floor (they read low) and the
    bottom zone's floor distance grows; the robot's lifted flag reads 1 while it is up;
  - drives add forward acceleration to ax; a drive over a tape stripe raises the line sensors under it.
Rest noise and number formats copy the real logs (2026-10-23/24, Left): accelerometer 0.001 g, gyro 0.03 deg/s on
a 0.07 deg/s step, roll/pitch 0.01 deg, line counts about 52 +- 1 on plywood, range zones about 0.8 cm.

Moves (each returns its own duration; the plan strings them together):
  stand(s)             robot still (rest noise; groups of streams that flicker together at rest)
  spins(n, deg)        in-place turns, alternately left and right, commanded (turn legs)
  drive(n, cm)         straight forward and back, commanded (go legs), facing a board
  square(n, cm)        drive, turn 90, drive, ... : drive stretches with a heading change between (camera turn pairs)
  tilt(n, deg)         by hand: nose up and back, then each side and back, n times
  lift(n, cm)          by hand: straight up, hold, put back down
  line(n)              drive across a tape stripe and back, commanded
"""
import csv
import json
import math
import os

import sys

import numpy as np

from . import load

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import raw_units                                             # noqa: E402  (tools/raw_units.py)

HZ = 50.0
R_WHEEL = 1.7
HALF_AXLE = 4.8
BOX = 120.0
COLUMNS = ["run", "ms", "step", "wl_deg", "wr_deg", "wl_rpm", "wr_rpm", "v_cms", "w_dps", "x_cm", "y_cm", "th_deg",
           "ax", "ay", "az", "gx", "gy", "gz", "roll_deg", "pitch_deg", "yaw_deg", "dL_cm", "dCL_cm", "dC_cm",
           "dCR_cm", "dR_cm", "dtop_cm", "dbot_cm", "lineL", "lineC", "lineR", "battery", "charging", "lifted",
           "swL", "swR", "raw_n"]
TAPE_W = 2.0           # a tape stripe across the floor, laid 12 cm ahead of the robot before the line move


def _ray(x, y, th_deg):
    a = math.radians(th_deg)
    c, s = math.cos(a), math.sin(a)
    best = 400.0
    for wall, axis in ((0.0, 0), (BOX, 0), (0.0, 1), (BOX, 1)):
        d = c if axis == 0 else s
        p = x if axis == 0 else y
        if abs(d) < 1e-6:
            continue
        t = (wall - p) / d
        if t > 0:
            best = min(best, t)
    return best


def _bump(T, n):
    """A smooth 0 -> 1 -> 0 profile over n samples (raised cosine) and its rate per second."""
    u = np.arange(n) / max(1, n - 1)
    p = 0.5 * (1 - np.cos(2 * np.pi * u))
    rate = np.gradient(p) * HZ
    return p, rate


class Bot:
    def __init__(self, seed=1):
        self.rng = np.random.default_rng(seed)
        r = self.rng
        self.x, self.y, self.th = 60.0, 60.0, 0.0
        self.wl = self.wr = 0.0
        self.roll = self.pitch = 0.0
        self.z = 0.0
        self.yaw_int = float(r.uniform(0, 360))
        self.t = 1000.0
        self.n = 0
        self.step = 0
        self.battery = 91
        self.gbias = (float(r.normal(-0.9, 0.2)), float(r.normal(0.3, 0.2)), float(r.normal(0.7, 0.2)))
        self.abias = (float(r.normal(0.018, 0.005)), float(r.normal(-0.012, 0.005)))
        self.frame = None
        self.line = [52, 53, 51]
        self.rows, self.legs, self.cam = [], [], []
        self.prev_v = 0.0
        self.prev_vz = 0.0
        self.tape = None       # (x0, y0, nx, ny): a point on the stripe's centre line and its normal

    # ---- one 20 ms sample
    def sample(self, v=0.0, w=0.0, droll=0.0, dpitch=0.0, vz=0.0):
        r, dt = self.rng, 1.0 / HZ
        vl = v - math.radians(w) * HALF_AXLE
        vr = v + math.radians(w) * HALF_AXLE
        self.wl += math.degrees(vl * dt / R_WHEEL)
        self.wr += math.degrees(vr * dt / R_WHEEL)
        self.x += v * dt * math.cos(math.radians(self.th))
        self.y += v * dt * math.sin(math.radians(self.th))
        self.th += w * dt
        self.roll += droll * dt
        self.pitch += dpitch * dt
        self.z += vz * dt
        a_fwd = (v - self.prev_v) / dt / 981.0
        a_up = (vz - self.prev_vz) / dt / 981.0
        self.prev_v, self.prev_vz = v, vz
        moving = abs(v) > 0.1 or abs(w) > 0.5
        hand = abs(droll) + abs(dpitch) + abs(vz) > 0.01 or self.z > 0.3 or abs(self.roll) + abs(self.pitch) > 0.3
        vib = 0.004 if moving else (0.003 if hand else 0.0)
        rp, pp = math.radians(self.roll), math.radians(self.pitch)
        ax = math.sin(pp) + a_fwd + self.abias[0] + r.normal(0, 0.001 + vib)
        ay = -math.sin(rp) * math.cos(pp) + self.abias[1] + r.normal(0, 0.001 + vib)
        az = -math.cos(rp) * math.cos(pp) - a_up + r.normal(0, 0.001 + vib)
        q = 0.061                                                     # gyro step (deg/s)
        gx = round((droll + self.gbias[0] + r.normal(0, 0.03 + 0.3 * hand)) / q) * q
        gy = round((dpitch + self.gbias[1] + r.normal(0, 0.03 + 0.3 * hand)) / q) * q
        gz = round((w * 0.98 + self.gbias[2] + r.normal(0, 0.02)) / q) * q
        self.yaw_int += gz * dt
        # what a robot logs (tools/raw_units.py): rpm, v_cms, w_dps 1.08 high, v_cms in mm/s; rpm steps of 0.66
        sp = raw_units.robot_columns(vl / (2 * math.pi * R_WHEEL) * 60, vr / (2 * math.pi * R_WHEEL) * 60,
                                     R_WHEEL, 2 * HALF_AXLE)
        rpm_l = round(sp["wl_rpm"] / 0.66) * 0.66
        rpm_r = round(sp["wr_rpm"] / 0.66) * 0.66
        if self.frame is None or self.n % 3 == 0:                     # fused orientation, slower tick
            self.frame = self.frame or {}
            self.frame["yaw"] = self.yaw_int % 360.0
            self.frame["roll"] = math.degrees(math.atan2(-ay, -az)) + r.normal(0, 0.01)
            self.frame["pitch"] = math.degrees(math.atan2(ax, -az)) + r.normal(0, 0.01)
        if "zones" not in self.frame or self.n % 3 == 1:              # range frames (about 17 Hz)
            up = abs(self.pitch) > 8 or self.z > 3          # looking over the boards: nothing in range
            zs = []
            for a in (22, 11, -11, -22):
                d = _ray(self.x, self.y, self.th + a)
                zs.append(float(r.uniform(150, 360)) if up else d + r.normal(0, 0.8))
            self.frame["zones"] = [zs[0], zs[1], 0.5 * (zs[1] + zs[2]), zs[2], zs[3]]
            self.frame["top"] = float(r.uniform(150, 360)) if up else _ray(self.x, self.y, self.th) + r.normal(0, 1.0)
            floor = 7.0 + self.z + 0.3 * max(0.0, self.pitch)
            self.frame["bot"] = floor + r.normal(0, 0.05)
        if self.n % 4 == 0:                                           # line sensors
            lift = min(1.0, self.z / 1.5 + max(0.0, self.pitch) / 10.0)
            out = []
            for lat in (1.2, 0.0, -1.2):
                px = self.x + 3.0 * math.cos(math.radians(self.th)) - lat * math.sin(math.radians(self.th))
                py = self.y + 3.0 * math.sin(math.radians(self.th)) + lat * math.cos(math.radians(self.th))
                on_tape = self.tape is not None and \
                    abs((px - self.tape[0]) * self.tape[2] + (py - self.tape[1]) * self.tape[3]) < TAPE_W / 2
                base = 600.0 if on_tape else 52.0
                val = base + (20.0 - base) * lift
                out.append(int(round(val + r.normal(0, 25 if on_tape else 1.0))))
            self.line = out
        if self.n and self.n % int(HZ * 900) == 0:      # about 1% per 15 min
            self.battery -= 1
        pose_th = (self.wr - self.wl) * R_WHEEL / (2 * HALF_AXLE)
        lifted = 1 if self.z > 1.0 else 0
        self.rows.append({
            "run": "CB01-1", "ms": 50000 + int(self.n * 20 + r.integers(0, 3)), "step": self.step,
            "wl_deg": "%.2f" % self.wl, "wr_deg": "%.2f" % self.wr, "wl_rpm": "%.2f" % rpm_l, "wr_rpm": "%.2f" % rpm_r,
            "v_cms": "%.2f" % ((rpm_l + rpm_r) * math.pi * R_WHEEL / 60.0 * raw_units.V_COL_PER_CMS),
            "w_dps": "%.2f" % ((rpm_r - rpm_l) * 2 * math.pi * R_WHEEL / 60.0 / (2 * HALF_AXLE) * 57.2958),
            "x_cm": "%.2f" % (self.x - 60.0), "y_cm": "%.2f" % (self.y - 60.0), "th_deg": "%.2f" % pose_th,
            "ax": "%.3f" % ax, "ay": "%.3f" % ay, "az": "%.3f" % az,
            "gx": "%.2f" % gx, "gy": "%.2f" % gy, "gz": "%.2f" % gz,
            "roll_deg": "%.2f" % self.frame["roll"], "pitch_deg": "%.2f" % self.frame["pitch"],
            "yaw_deg": "%.2f" % self.frame["yaw"],
            "dL_cm": "%.1f" % self.frame["zones"][0], "dCL_cm": "%.1f" % self.frame["zones"][1],
            "dC_cm": "%.1f" % self.frame["zones"][2], "dCR_cm": "%.1f" % self.frame["zones"][3],
            "dR_cm": "%.1f" % self.frame["zones"][4], "dtop_cm": "%.1f" % self.frame["top"],
            "dbot_cm": "%.1f" % self.frame["bot"],
            "lineL": self.line[0], "lineC": self.line[1], "lineR": self.line[2], "battery": self.battery,
            "charging": 0, "lifted": lifted, "swL": -1, "swR": -1, "raw_n": self.n,
        })
        if self.n % 3 == 0:
            self.cam.append((self.t + r.normal(0, 0.005), self.x + r.normal(0, 0.3), self.y + r.normal(0, 0.3)))
        self.t += dt
        self.n += 1

    # ---- moves
    def hold(self, s):
        for _ in range(int(round(s * HZ))):
            self.sample()
        return s

    def leg(self, kind, value, speed):
        """A commanded go (cm, speed cm/s) or turn (deg, speed deg/s) with a smooth start and stop."""
        dur = abs(value) / speed + 0.4
        n = int(round(dur * HZ))
        prof = np.clip(np.minimum(np.arange(n), n - 1 - np.arange(n)) / (0.2 * HZ), 0, 1)
        prof = prof * abs(value) / (prof.sum() / HZ)
        self.step += 1
        t0 = self.t
        for p in prof:
            sp = math.copysign(p, value)
            self.sample(v=sp if kind == "go" else 0.0, w=sp if kind == "turn" else 0.0)
        self.legs.append({"kind": kind, "value": value, "t0": t0, "t1": self.t})
        return dur

    def stand(self, s=10.0):
        return self.hold(s)

    def spins(self, n=4, deg=90.0):
        d = 0.0
        for i in range(n):
            d += self.leg("turn", deg if i % 2 == 0 else -deg, 60.0) + self.hold(1.0)
        return d

    def drive(self, n=2, cm=20.0):
        d = 0.0
        for i in range(n):
            d += self.leg("go", cm, 10.0) + self.hold(1.0) + self.leg("go", -cm, 10.0) + self.hold(1.0)
        return d

    def square(self, n=12, cm=15.0):
        d = 0.0
        for i in range(n):
            d += self.leg("go", cm, 10.0) + self.hold(0.5) + self.leg("turn", 90.0 if i % 4 != 3 else -90.0, 60.0) \
                + self.hold(0.5)
        return d

    def tilt(self, n=2, deg=20.0, s=2.0):
        d = 0.0
        m = int(round(s * HZ))
        for k in range(n):
            for axis, sign in (("pitch", 1), ("roll", 1), ("roll", -1)):
                p, rate = _bump(s, m)
                for rr in rate:
                    if axis == "pitch":
                        self.sample(dpitch=sign * deg * rr)
                    else:
                        self.sample(droll=sign * deg * rr)
                self.roll = self.pitch = 0.0
                d += s + self.hold(1.0)
        return d

    def lift(self, n=2, cm=5.0, s=3.0):
        d = 0.0
        for k in range(n):
            m = int(round(s * HZ))
            third = m // 3
            up = cm / (third / HZ)
            for i in range(m):
                vz = up * math.sin(math.pi * (i % third) / third) * (math.pi / 2) if i < third else \
                    (-up * math.sin(math.pi * (i - 2 * third) / third) * (math.pi / 2) if i >= 2 * third else 0.0)
                self.sample(vz=vz)
            self.z = 0.0
            d += s + self.hold(1.0)
        return d

    def ramp(self, n=2, pitch_deg=14.0, speed=4.0):
        """Hands-free pitch: drive over a printed ramp-and-plateau (1 cm rise over 8 cm, 6 cm plateau, 8 cm down),
        commanded, starting 6 cm before it so the start and stop of the drive fall on flat floor. The drive
        wheels climb first while the rear caster is still on the floor, so the nose tips up (about atan(1 / 4) =
        14 deg for a caster 4 cm behind the axle); it levels when the caster climbs, tips nose down as the wheels
        go down, and levels again. Then back over it in reverse. 34 cm each way."""
        d = 0.0
        lead = 6.0
        for k in range(n):
            for direction in (1, -1):
                L = 2 * lead + 22.0
                m = int(round(L / speed * HZ))
                m0 = int(round(lead / speed * HZ))
                q = (m - 2 * m0) // 4
                self.step += 1
                t0 = self.t
                for i in range(m):
                    j = i - m0
                    seg = j // q if 0 <= j < 4 * q else -1
                    u = (j % q) / max(1, q - 1) if seg >= 0 else 0.0
                    rate = pitch_deg * np.pi * np.sin(2 * np.pi * u) / (q / HZ) / 2.0
                    sign = {0: 1, 2: -1}.get(seg, 0) * direction
                    self.sample(v=direction * speed, dpitch=sign * rate)
                self.pitch = 0.0
                self.legs.append({"kind": "go", "value": direction * L, "t0": t0, "t1": self.t})
                d += m / HZ + self.hold(1.0)
        return d

    def bump(self, n=2, roll_deg=6.4, speed=4.0):
        """Hands-free roll: drive one wheel over a printed half-width bump (1 cm high, 10 cm long, rounded):
        roll about atan(1 / 8.9) = 6.4 deg up and back. Forward over it, then back over it (commanded)."""
        d = 0.0
        for k in range(n):
            for direction in (1, -1):
                L = 20.0
                m = int(round(L / speed * HZ))
                self.step += 1
                t0 = self.t
                p, rate = _bump(L / speed, m // 2)
                for i in range(m):
                    j = i - m // 4
                    rr = rate[j] if 0 <= j < len(rate) else 0.0
                    self.sample(v=direction * speed, droll=roll_deg * rr * (1 if k % 2 == 0 else -1))
                self.roll = 0.0
                self.legs.append({"kind": "go", "value": direction * L, "t0": t0, "t1": self.t})
                d += m / HZ + self.hold(1.0)
        return d

    def line_cross(self, n=2):
        """Drive across a tape stripe laid 12 cm ahead (by hand, before the move) and back."""
        c, sn = math.cos(math.radians(self.th)), math.sin(math.radians(self.th))
        self.tape = (self.x + 15.0 * c, self.y + 15.0 * sn, c, sn)
        d = 0.0
        for k in range(n):
            d += self.leg("go", 25.0, 10.0) + self.hold(0.5) + self.leg("go", -25.0, 10.0) + self.hold(0.5)
        return d


PLANS = {
    # what the current 9 s calibration log holds: stand, a compass wiggle, stand
    "today_9s": [("stand", 3.0), ("spins", (2, 15.0)), ("stand", 4.0)],
}
MOVES = ("stand", "spins", "drive", "square", "tilt", "lift", "line")
DEFAULT_ARGS = {"stand": (10.0,), "spins": (4, 90.0), "drive": (2, 20.0), "square": (12, 15.0), "tilt": (2, 20.0),
                "lift": (2, 5.0), "line": (2,)}


def run_plan(plan, seed=1):
    """plan: [(move, args)]; returns the Bot (rows, legs, cam) and {move: seconds}."""
    b = Bot(seed)
    secs = {}
    for mv, a in plan:
        a = a if isinstance(a, tuple) else (a,)
        fn = {"stand": b.stand, "spins": b.spins, "drive": b.drive, "square": b.square, "tilt": b.tilt,
              "lift": b.lift, "line": b.line_cross, "ramp": b.ramp, "bump": b.bump}[mv]
        secs[mv] = secs.get(mv, 0.0) + fn(*a)
        b.hold(0.5)
    return b, secs


def write_session(d, plan, seed=1, robot="Synth"):
    b, secs = run_plan(plan, seed)
    sub = os.path.join(d, "robot-logs-1")
    os.makedirs(sub, exist_ok=True)
    rng = np.random.default_rng(seed + 7)
    with open(os.path.join(sub, "raw.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t_recv"] + COLUMNS)
        last = 0.0
        for i, r in enumerate(b.rows):
            last = max(last, 1000.0 + i / HZ + abs(rng.normal(0, 0.01)))
            w.writerow([robot, "%.4f" % last] + [r[c] for c in COLUMNS])
    with open(os.path.join(sub, "leg.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t_recv", "run", "id", "kind", "value", "result", "ms", "x_cm", "y_cm", "th_deg", "info"])
        for i, L in enumerate(b.legs):
            w.writerow([robot, "%.4f" % L["t0"], "CB01-1", i, L["kind"], "%.2f" % L["value"], "started", 0, 0, 0, 0, ""])
            w.writerow([robot, "%.4f" % L["t1"], "CB01-1", i, L["kind"], "%.2f" % L["value"], "done", 0, 0, 0, 0, ""])
    with open(os.path.join(sub, "runinfo.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t_recv", "run", "experiment", "fw_version"])
        w.writerow([robot, "1000.0", "CB01-1", "battery", "synthetic"])
    with open(os.path.join(d, "track.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "robot", "x_cm", "y_cm", "order", "phase", "source", "lock_score"])
        for t, x, y in b.cam:
            w.writerow(["%.3f" % t, robot, "%.2f" % x, "%.2f" % y, "", "", "lock", "1"])
    with open(os.path.join(d, "sim_info.json"), "w") as f:      # simulator marker: RAW speeds in robot units
        json.dump({"source": "tools/discovery/battery.py", "raw_units": raw_units.RAW_UNITS}, f)
    with open(os.path.join(d, "e02_state.json"), "w") as f:
        json.dump({"board": {"xboard_cm": BOX, "yboard_cm": BOX, "xboard_side": 1, "yboard_side": 1}}, f)
    return d, secs, len(b.rows) / HZ


def load_plan_case(d, plan, seed=1):
    _, secs, dur = write_session(d, plan, seed)
    case, key = load.load_case(d, "Synth")
    return case, key, secs, dur
