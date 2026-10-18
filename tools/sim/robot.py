"""One simulated Alvik: its body (true motion), its sensors, and a copy of the
home_lab firmware's run logic.

Two poses are kept apart on purpose:
  - the TRUE pose (self.x, self.y, self.th): where the robot really is, on the
    floor, moved by what the wheels really do (true wheel sizes, slip);
  - the robot's OWN odometry (self.odo_*): what the robot believes, worked out
    from its (quantized) wheel encoders with the NOMINAL wheel radius and
    track, exactly as dead reckoning on the robot does it.

How the firmware controls the robot decides where the errors come from:
  - straight moves (library move()) stop when the ENCODERS say the distance is
    done, so along-track error comes from wheel radius error and slip, and a
    left/right radius mismatch curves the path;
  - turns (imuTurnTo) stop when the GYRO says the heading is reached, so the
    real heading error after a turn comes from gyro bias and scale error, not
    from the wheels.

Time: the robot runs on its own clock (self.ms, its millis()). Everything it
sends is stamped with that clock; the network module turns robot time into
laptop time later, using the clock offset and drift of this robot.
"""
import math
import random

from config import firmware_raw_columns

EXP_NAMES = ["stream", "turn", "lap", "grid", "demo", "straight", "calib"]
EXP_CALIB = 6
# showSelection() colours, (r, g, b) on/off, per experiment index (home_lab.ino);
# calib (6) never becomes the selection (2026-10-17g), so its entry is unused
EXP_COLORS = [(0, 0, 1), (0, 1, 0), (1, 1, 0), (1, 0, 1), (1, 1, 1), (0, 1, 1), (0, 0, 0)]
BLINK_SLOT_MS = 100        # blink.h
BLINK_LEAD_SLOTS = 3         # dark slots right after blink::start()
OFF, WHITE = (0, 0, 0), (1, 1, 1)

# printf format of each RAW column, copied from rawlog.h's tick()
RAW_FORMATS = {"run": "%s", "ms": "%d", "step": "%d"}
for _c in "wl_deg wr_deg wl_rpm wr_rpm v_cms w_dps x_cm y_cm th_deg roll_deg pitch_deg yaw_deg".split():
    RAW_FORMATS[_c] = "%.2f"
for _c in "ax ay az".split():
    RAW_FORMATS[_c] = "%.4f"
for _c in "gx gy gz".split():
    RAW_FORMATS[_c] = "%.3f"
for _c in "dL_cm dCL_cm dC_cm dCR_cm dR_cm dtop_cm dbot_cm".split():
    RAW_FORMATS[_c] = "%.1f"
for _c in "lineL lineC lineR battery charging lifted".split():
    RAW_FORMATS[_c] = "%d"


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def uniform_pm(rng, half):
    return rng.uniform(-half, half)


class Move:
    """A library move(): both wheels follow the same trapezoid speed profile
    (in wheel degrees), ramping up, cruising, ramping down."""

    def __init__(self, dist_deg, vmax, acc, phiL0, phiR0, direction):
        self.d, self.vmax, self.acc = dist_deg, vmax, acc
        self.phiL0, self.phiR0, self.dir = phiL0, phiR0, direction
        self.t = 0.0
        if dist_deg < vmax * vmax / acc:          # too short to reach full speed
            self.vpk = math.sqrt(dist_deg * acc)
            self.t_acc = self.vpk / acc
            self.t_cruise = 0.0
        else:
            self.vpk = vmax
            self.t_acc = vmax / acc
            self.t_cruise = (dist_deg - vmax * vmax / acc) / vmax
        self.total = 2 * self.t_acc + self.t_cruise
        self.eL = self.eR = 0.0                   # each wheel's wander from the profile, degrees
        self.nL = self.nR = 0.0

    def pos(self, t):
        a, ta, tc = self.acc, self.t_acc, self.t_cruise
        if t <= 0:
            return 0.0
        if t < ta:
            return 0.5 * a * t * t
        p1 = 0.5 * a * ta * ta
        if t < ta + tc:
            return p1 + self.vpk * (t - ta)
        if t < self.total:
            td = t - ta - tc
            return p1 + self.vpk * tc + self.vpk * td - 0.5 * a * td * td
        return self.d


def ray_polys(px, py, dx, dy, polys, best):
    """Distance along the ray (px, py) + s (dx, dy) to the first polygon edge, if below best."""
    for poly in polys:
        n = len(poly)
        for i in range(n):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % n]
            ex, ey = bx - ax, by - ay
            den = dx * (-ey) - dy * (-ex)
            if abs(den) < 1e-12:
                continue
            rx, ry = ax - px, ay - py
            s = (rx * (-ey) - ry * (-ex)) / den
            if s <= 0 or s >= best:
                continue
            g = (dx * ry - dy * rx) / den
            if 0.0 <= g <= 1.0:
                best = s
    return best


class Robot:
    def __init__(self, name, cfg, rng, params, fw_version, t_boot, boot_id, ip):
        self.name, self.cfg, self.rng = name, cfg, rng
        self.m, self.imu_m, self.tof_m, self.line_m = cfg.robot, cfg.imu, cfg.tof, cfg.line
        self.params = dict(params)               # name -> value (live, changed by `set`)
        self.param_order = [n for n, _ in params]
        self.fw_version = fw_version
        self.boot_id = boot_id
        self.ip = ip
        self.raw_cols = firmware_raw_columns()
        m, im = self.m, self.imu_m

        # ---- this robot's true physical values (drawn once) ----
        scale = 1 + uniform_pm(rng, m.radius_scale_err_pct) / 100
        mism = uniform_pm(rng, m.radius_mismatch_pct) / 100
        self.rL = m.wheel_radius_cm * scale * (1 - mism / 2)
        self.rR = m.wheel_radius_cm * scale * (1 + mism / 2)
        self.track = m.wheel_track_cm * (1 + uniform_pm(rng, m.track_err_pct) / 100)
        if name in m.radius_scale_err_pct_by_robot:          # exact values for tests (the draws above still
            scale = 1 + m.radius_scale_err_pct_by_robot[name] / 100     # happen, so the random stream is unchanged)
            self.rL = m.wheel_radius_cm * scale * (1 - mism / 2)
            self.rR = m.wheel_radius_cm * scale * (1 + mism / 2)
        if name in m.track_err_pct_by_robot:
            self.track = m.wheel_track_cm * (1 + m.track_err_pct_by_robot[name] / 100)
        self.gyro_scale = uniform_pm(rng, im.gyro_scale_err_pct) / 100
        self.gyro_bias = rng.gauss(0, im.gyro_bias_dps)
        self.acc_bias = [rng.gauss(0, im.accel_bias_g) for _ in range(3)]
        self.gxy_bias = [rng.gauss(0, 0.2), rng.gauss(0, 0.2)]
        self.drift_ppm = uniform_pm(rng, cfg.net.clock_drift_ppm)
        self.t_boot = t_boot                      # laptop time (Unix s) of this robot's millis() = 0
        self.battery = rng.uniform(*m.battery_start_pct)
        self.wifi_dbm_mean = rng.uniform(*cfg.net.wifi_dbm)

        # ---- state ----
        self.ms = 0.0
        self.x, self.y, self.th = cfg.session.parked_pose[0], cfg.session.parked_pose[1], math.radians(cfg.session.parked_pose[2])
        self.phiL = self.phiR = 0.0               # true wheel angles, degrees
        self.wL = self.wR = 0.0                   # wheel speeds, deg/s
        self.cmdL = self.cmdR = 0.0
        self.mode = "vel"                          # "vel" (set_wheels_speed / brake) or "move" (library move)
        self.mv = None
        self.slipL = self.slipR = m.move_slip_mean_pct / 100
        self.turn_slip = m.turn_slip_mean_pct / 100
        self.v = self.w = self.a_fwd = 0.0
        self.encL = self.encR = 0.0               # last encoder readings (quantized)
        self.odo_x = self.odo_y = self.odo_th = 0.0   # odometry; th in degrees, not wrapped
        self.yaw_raw = rng.uniform(-180, 180)     # IMU fused yaw (its own zero)
        self.lifted = False
        self.lift = None                          # dict while lifted
        self.kidnap_at = None
        self.kidnap_plan = None
        # what the front distance sensor sees besides the room walls: polygons [[x, y], ...]
        # (boards, obstacles), and a function giving the other robots' outlines (fleet_sim.py)
        self.world_polys = []
        self.others = None
        self._tof_ms = -1e9
        self._tof_vals = None
        self._tof_phase = rng.uniform(0, 66.0)     # when the free-running sensor's frames fall

        # ---- firmware state ----
        self.run_n = 0
        self.run_id = ""
        self.selected = 1                          # turn (green) until a run picks something
        self.abort_reason = ""
        self.fail = ""
        self.raw_on = False
        self.raw_next = 0
        self.raw_period = 20
        self.raw_rows = 0
        self.step = 0
        self.fw_yaw_sign = 0                       # gYawSign: 0 until a compass check since power-on
        self.fw_odo_sign = 1                       # gOdoSign
        self.blink_on = False                      # blink.h state
        self.blink_code = 0
        self.blink_lit = False
        self.blink_t0 = 0

        # ---- records ----
        self.lines = []        # [robot_ms, text]  every line the robot sends (H/D/M)
        self.leds = [(0.0, EXP_COLORS[self.selected], OFF)]   # (robot_ms, left rgb, right rgb)
        self.busy = []         # [start_ms, end_ms]: not reading WiFi commands
        self.traj = []         # (robot_ms, x, y, th_rad, lifted)
        self.runs = []         # one dict per run
        self.kidnaps = []
        self._record()

    # ================================================= clock
    def ms_at(self, t):
        """Robot millis() (float) at laptop time t."""
        return (t - self.t_boot) * 1000.0 * (1 + self.drift_ppm * 1e-6)

    def t_at(self, ms):
        """Laptop time of robot millis() value ms."""
        return self.t_boot + ms / 1000.0 / (1 + self.drift_ppm * 1e-6)

    def millis(self):
        return int(self.ms)

    # ================================================= physics
    def _record(self):
        self.traj.append((self.ms, self.x, self.y, self.th, 1 if self.lifted else 0))

    def advance(self, dt_ms, busy=False):
        """Let dt_ms of robot time pass."""
        if dt_ms <= 0:
            return
        if busy:
            self.busy.append([self.ms, self.ms + dt_ms])
        end = self.ms + dt_ms
        while self.ms < end - 1e-9:
            if self.kidnap_at is not None and self.ms >= self.kidnap_at and not self.lifted:
                self._start_kidnap()
            still = (self.mode == "vel" and not self.lifted and self.cmdL == 0 and self.cmdR == 0
                     and abs(self.wL) < 1e-3 and abs(self.wR) < 1e-3)
            h_ms = end - self.ms if still else min(2.0, end - self.ms)
            if self.lifted:
                h_ms = min(h_ms, self.lift["drop_ms"] - self.ms) if self.lift["drop_ms"] > self.ms else h_ms
                h_ms = max(h_ms, 1e-3)
            if self.kidnap_at is not None and not self.lifted and self.kidnap_at > self.ms:
                h_ms = min(h_ms, self.kidnap_at - self.ms)
            self._physics(h_ms / 1000.0, still)
            if self.lifted and self.ms >= self.lift["drop_ms"] - 1e-9:
                self._drop()
        self._record()

    def _physics(self, h, still):
        m, im, rng = self.m, self.imu_m, self.rng
        if still:
            self.wL = self.wR = 0.0
            dL = dR = 0.0
        elif self.lifted:
            # picked up: wheels stop (the "pause" model) and nothing reaches the floor
            self.wL = self.wR = 0.0
            dL = dR = 0.0
            if self.mode == "move" and self.mv is not None:
                pass                                  # the move's clock is paused too
        elif self.mode == "move":
            mv = self.mv
            mv.t += h
            p = mv.pos(mv.t)
            # each wheel wanders a little around the profile (motor control isn't perfect)
            # Smooth wander: a slow random signal (nL, nR), smoothed again into
            # eL, eR, so the wheel speeds stay smooth (as a motor controller makes them).
            a = math.exp(-h / m.wheel_track_noise_tau_s)
            s = m.wheel_track_noise_deg * math.sqrt(1 - a * a)
            if mv.t < mv.total:
                mv.nL = mv.nL * a + rng.gauss(0, s)
                mv.nR = mv.nR * a + rng.gauss(0, s)
                k = min(1.0, h / m.wheel_track_noise_tau_s)
                mv.eL += (mv.nL - mv.eL) * k
                mv.eR += (mv.nR - mv.eR) * k
            tL = mv.phiL0 + mv.dir * p + mv.eL
            tR = mv.phiR0 + mv.dir * p + mv.eR
            dL, dR = tL - self.phiL, tR - self.phiR
            self.wL, self.wR = dL / h, dR / h
        else:
            tauL = m.brake_tau_s if self.cmdL == 0 else m.motor_tau_s
            tauR = m.brake_tau_s if self.cmdR == 0 else m.motor_tau_s
            self.wL += (self.cmdL - self.wL) * (1 - math.exp(-h / tauL))
            self.wR += (self.cmdR - self.wR) * (1 - math.exp(-h / tauR))
            if self.cmdL == 0 and abs(self.wL) < 0.05:
                self.wL = 0.0
            if self.cmdR == 0 and abs(self.wR) < 0.05:
                self.wR = 0.0
            dL, dR = self.wL * h, self.wR * h
        self.phiL += dL
        self.phiR += dR

        # ---- where the robot really goes ----
        th0 = self.th
        if self.lifted:
            lf = self.lift
            f = min(1.0, max(0.0, (self.ms + h * 1000 - lf["lift_ms"]) / (lf["drop_ms"] - lf["lift_ms"])))
            nx = lf["x0"] + (lf["x1"] - lf["x0"]) * f
            ny = lf["y0"] + (lf["y1"] - lf["y0"]) * f
            nth = lf["th0"] + lf["dth"] * f
            ds = math.hypot(nx - self.x, ny - self.y)
            self.x, self.y, self.th = nx, ny, nth
        elif dL or dR:
            opposite = dL * dR < 0
            if opposite:                                   # in-place turn: wheels scrub
                sl = sr = self.turn_slip
            else:
                sl, sr = self.slipL, self.slipR
            # extra slip while speeding up or slowing down
            extra = m.accel_slip_pct_per_g / 100 * abs(self.a_fwd) / 981.0
            sL = math.radians(dL) * self.rL * (1 - sl - extra)
            sR = math.radians(dR) * self.rR * (1 - sr - extra)
            ds = (sL + sR) / 2
            dth = (sR - sL) / self.track
            self.x += ds * math.cos(self.th + dth / 2)
            self.y += ds * math.sin(self.th + dth / 2)
            self.th += dth
        else:
            ds = 0.0
        rate = (self.th - th0) / h                         # rad/s, true
        v = 0.0 if self.lifted else ds / h
        # forward acceleration, smoothed a little (as the accelerometer's own filter does)
        a_new = (v - self.v) / h if h < 0.05 else 0.0
        self.a_fwd += (a_new - self.a_fwd) * min(1.0, h / 0.02)
        self.v, self.w = v, rate

        # ---- encoders and the robot's own odometry (nominal radius and track) ----
        res = m.encoder_res_deg
        eL = round(self.phiL / res) * res
        eR = round(self.phiR / res) * res
        dEL, dER = eL - self.encL, eR - self.encR
        self.encL, self.encR = eL, eR
        if dEL or dER:
            r0, b0 = m.wheel_radius_cm, m.wheel_track_cm
            sl, sr = math.radians(dEL) * r0, math.radians(dER) * r0
            d, dth = (sl + sr) / 2, math.degrees((sr - sl) / b0)
            mid = math.radians(self.odo_th + dth / 2)
            self.odo_x += d * math.cos(mid)
            self.odo_y += d * math.sin(mid)
            self.odo_th += dth

        # ---- gyro: bias wanders, scale error, noise integrated into the fused yaw ----
        self.gyro_bias += im.gyro_bias_walk_dps_per_rt_s * math.sqrt(h) * rng.gauss(0, 1)
        arw = im.yaw_arw_deg_per_rt_s * (5 if self.lifted else 1)
        self.yaw_raw += im.yaw_sign * (math.degrees(rate) * (1 + self.gyro_scale) + self.gyro_bias) * h \
            + arw * math.sqrt(h) * rng.gauss(0, 1)
        if self.yaw_raw > 180 or self.yaw_raw <= -180:
            self.yaw_raw = wrap180(self.yaw_raw)

        moving = abs(self.wL) + abs(self.wR) > 1
        self.battery -= (m.battery_moving_pct_per_h if moving else m.battery_idle_pct_per_h) * h / 3600
        self.ms += h * 1000

    # ---- picking the robot up (carried by hand, or kidnapped mid-run) ----
    def start_lift(self, x1, y1, th1_rad, dur_ms, reason):
        dth = wrap180(math.degrees(th1_rad - self.th))
        self.lifted = True
        self.lift = dict(lift_ms=self.ms, drop_ms=self.ms + dur_ms, x0=self.x, y0=self.y, th0=self.th,
                         x1=x1, y1=y1, dth=math.radians(dth), reason=reason)
        self._record()

    def _drop(self):
        lf = self.lift
        self.x, self.y, self.th = lf["x1"], lf["y1"], lf["th0"] + lf["dth"]
        self.lifted = False
        self.lift = None
        self._record()

    def _start_kidnap(self):
        k, rng = self.kidnap_plan, self.rng
        self.kidnap_at = None
        for _ in range(200):                     # set down somewhere still on the clear floor
            d = rng.uniform(*k["move_cm"])
            ang = rng.uniform(-math.pi, math.pi)
            x1, y1 = self.x + d * math.cos(ang), self.y + d * math.sin(ang)
            if -5 <= x1 <= 130 and -35 <= y1 <= 55:
                break
        dth = rng.uniform(*k["turn_deg"]) * rng.choice([-1, 1])
        th1 = self.th + math.radians(dth)
        self.kidnaps.append(dict(run=self.run_id, lift_ms=self.ms, drop_ms=self.ms + k["lifted_s"] * 1000,
                                 x0=self.x, y0=self.y, th0_deg=math.degrees(self.th),
                                 x1=x1, y1=y1, th1_deg=math.degrees(th1), mode=k["mode"]))
        self.start_lift(x1, y1, th1, k["lifted_s"] * 1000, "kidnap")

    def idle_until(self, t):
        """Sit still (or stay carried) until laptop time t."""
        self.advance(self.ms_at(t) - self.ms)

    def carry_to(self, pose, t0, t1):
        """Picked up by hand at laptop time t0, put down at `pose` (x, y, heading deg) at t1.
        The gyro feels the rotation; the wheels don't turn, so odometry doesn't change."""
        self.idle_until(t0)
        self.start_lift(pose[0], pose[1], math.radians(pose[2]), (t1 - t0) * 1000 * (1 + self.drift_ppm * 1e-6), "carry")
        self.idle_until(t1)
        if self.lifted:
            self._drop()

    # ================================================= sensors
    def heading(self):
        """Firmware heading(): IMU yaw times gYawSign (found by the compass check), so left turns
        are positive. Before any check the firmware's sign is 0 and it refuses runs that turn; here
        the true sign stands in, so the motion helpers also work on their own (unit tests)."""
        return wrap180((self.fw_yaw_sign or self.imu_m.yaw_sign) * self.yaw_raw)

    def settled_pose(self, ms, after_ms=300.0):
        """True pose (x, y, th) after_ms after robot time ms, from the whole trajectory (analysis)."""
        import bisect
        t = ms + after_ms
        k = bisect.bisect_left([p[0] for p in self.traj], t)
        k = min(max(k, 0), len(self.traj) - 1)
        return self.traj[k][1:4]

    def pose_at_ms(self, ms):
        """True pose (x, y, th) at robot time ms, from the recorded trajectory (recent past only)."""
        tr = self.traj
        if not tr or ms >= tr[-1][0]:
            return self.x, self.y, self.th
        for k in range(len(tr) - 1, max(0, len(tr) - 400), -1):
            a, b = tr[k - 1], tr[k]
            if a[0] <= ms:
                f = (ms - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 1.0
                return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * (b[3] - a[3])
        return tr[max(0, len(tr) - 400)][1:4]

    def _tof_true(self, ang_deg, others=None, pose=None):
        t = self.tof_m
        x0, y0, th0 = pose if pose is not None else (self.x, self.y, self.th)
        a = th0 + math.radians(ang_deg)
        px = x0 + t.sensor_forward_cm * math.cos(th0)
        py = y0 + t.sensor_forward_cm * math.sin(th0)
        dx, dy = math.cos(a), math.sin(a)
        best = float("inf")
        for wall, d, p in ((t.arena_x[0], dx, px), (t.arena_x[1], dx, px), (t.arena_y[0], dy, py), (t.arena_y[1], dy, py)):
            if abs(d) > 1e-9:
                s = (wall - p) / d
                if s > 0:
                    best = min(best, s)
        if self.world_polys:
            best = ray_polys(px, py, dx, dy, self.world_polys, best)
        if others:
            best = ray_polys(px, py, dx, dy, others, best)
        return best

    def tof(self):
        """get_distance(): the five front zones. The sensor delivers a new frame tof_hz times a
        second (the reading is held in between); below min_range_cm it is untrustworthy (here:
        any value under min_range_cm)."""
        t, rng = self.tof_m, self.rng
        if t.hz > 0:
            period = 1000.0 / t.hz
            frame = math.floor((self.ms - self._tof_phase) / period) * period + self._tof_phase
            if self._tof_vals is not None and frame <= self._tof_ms:
                return self._tof_vals
            self._tof_ms = frame
            pose = self.pose_at_ms(frame)          # the sensor runs on its own: the newest frame was taken then
        else:
            self._tof_ms = self.ms
            pose = None
        others = self.others() if self.others else None
        out = []
        w = t.zone_width_deg / 3.0
        errs = t.zone_angle_err_deg_by_robot.get(self.name) or [0.0] * len(t.zone_angles_deg)
        bias, scale = t.range_err_by_robot.get(self.name) or (0.0, 0.0)
        for ang, err in zip(t.zone_angles_deg, errs):
            ang = ang + err
            d = min(self._tof_true(ang + k * w, others, pose) for k in (-1, 0, 1)) if w > 0 else \
                self._tof_true(ang, others, pose)
            if d >= t.max_range_cm:
                out.append(t.max_range_cm)
            elif d < t.min_range_cm:
                out.append(rng.uniform(0.0, t.min_range_cm))
            else:
                out.append(max(0.0, d * (1 + scale / 100) + bias + rng.gauss(0, t.noise_cm + d * t.noise_pct / 100)))
        self._tof_vals = out
        return out

    def line_sensors(self):
        lm, rng = self.line_m, self.rng
        if self.lifted:
            return [int(max(0, rng.gauss(20, 5))) for _ in range(3)]
        s = self.cfg.session
        half = s.tape_width_cm / 2
        out = []
        c, sn = math.cos(self.th), math.sin(self.th)
        for lat in (lm.spacing_cm, 0.0, -lm.spacing_cm):          # L is on the robot's left
            px = self.x + lm.sensor_forward_cm * c - lat * sn
            py = self.y + lm.sensor_forward_cm * sn + lat * c
            cover = 0.0
            for x0, y0, x1, y1 in s.tape_lines:
                vx, vy = x1 - x0, y1 - y0
                L2 = vx * vx + vy * vy
                u = max(0.0, min(1.0, ((px - x0) * vx + (py - y0) * vy) / L2))
                dist = math.hypot(px - (x0 + u * vx), py - (y0 + u * vy))
                cover = max(cover, min(1.0, max(0.0, (half + lm.footprint_cm / 2 - dist) / lm.footprint_cm)))
            val = lm.floor_reading + (lm.tape_reading - lm.floor_reading) * cover
            noise = lm.floor_noise + (lm.tape_noise - lm.floor_noise) * cover
            out.append(int(max(0, rng.gauss(val, noise))))
        return out

    def raw_values(self, now):
        m, im, rng = self.m, self.imu_m, self.rng
        moving = abs(self.wL) + abs(self.wR) > 1
        wl_rpm = self.wL / 6 + rng.gauss(0, m.wheel_speed_noise_rpm)
        wr_rpm = self.wR / 6 + rng.gauss(0, m.wheel_speed_noise_rpm)
        # get_drive_speed(): the library's own conversion with nominal sizes
        vl = wl_rpm * 2 * math.pi * m.wheel_radius_cm / 60
        vr = wr_rpm * 2 * math.pi * m.wheel_radius_cm / 60
        v_cms = (vl + vr) / 2
        w_dps = math.degrees((vr - vl) / m.wheel_track_cm)
        an = im.accel_noise_g + (im.accel_vibration_g if moving else 0) + (0.05 if self.lifted else 0)
        ax = self.a_fwd / 981.0 + self.acc_bias[0] + rng.gauss(0, an)
        ay = self.v * self.w / 981.0 + self.acc_bias[1] + rng.gauss(0, an)
        az = 1.0 + self.acc_bias[2] + rng.gauss(0, an)
        gn = 3.0 if self.lifted else im.gyro_noise_dps
        gx = self.gxy_bias[0] + rng.gauss(0, gn)
        gy = self.gxy_bias[1] + rng.gauss(0, gn)
        gz = im.yaw_sign * (math.degrees(self.w) * (1 + self.gyro_scale) + self.gyro_bias) + rng.gauss(0, im.gyro_noise_dps)
        tn = im.tilt_noise_deg * (10 if self.lifted else 1)
        roll = math.degrees(self.acc_bias[1]) + rng.gauss(0, tn)
        pitch = -math.degrees(self.acc_bias[0]) + rng.gauss(0, tn)
        d = self.tof()
        dtop = min(self.tof_m.max_range_cm, d[2] * (1 + self.tof_m.top_extra_pct / 100))
        dbot = max(0.0, rng.gauss(self.tof_m.bottom_cm, 0.3))
        lL, lC, lR = self.line_sensors()
        vals = dict(run=self.run_id, ms=now, step=self.step, wl_deg=self.encL, wr_deg=self.encR,
                    wl_rpm=wl_rpm, wr_rpm=wr_rpm, v_cms=v_cms, w_dps=w_dps,
                    x_cm=self.odo_x, y_cm=self.odo_y, th_deg=self.odo_th,
                    ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz, roll_deg=roll, pitch_deg=pitch,
                    yaw_deg=wrap180(self.yaw_raw),
                    dL_cm=d[0], dCL_cm=d[1], dC_cm=d[2], dCR_cm=d[3], dR_cm=d[4], dtop_cm=dtop, dbot_cm=dbot,
                    lineL=lL, lineC=lC, lineR=lR, battery=int(self.battery), charging=0,
                    lifted=1 if self.lifted else 0)
        return vals

    # ================================================= firmware: logging
    def P(self, name):
        return self.params.get(name, 0.0)

    def send(self, text):
        self.lines.append([self.ms, text])

    def set_leds(self, left, right):
        self.leds.append((self.ms, left, right))

    def show_selection(self, running):
        c = EXP_COLORS[self.selected]
        self.set_leds(c, c if running else OFF)

    def tick(self):
        """rawlog::tick(): at most one RAW record per period, without bursting to catch up.
        blink::tick() runs first, on every call, so the blink lights only change here."""
        self.blink_tick()
        if not self.raw_on:
            return
        now = self.millis()
        if now - self.raw_next < 0:
            return
        self.raw_next += self.raw_period
        if now - self.raw_next > self.raw_period:
            self.raw_next = now + self.raw_period
        vals = self.raw_values(now)
        parts = []
        for c in self.raw_cols:
            if c in vals:
                parts.append(RAW_FORMATS.get(c, "%s") % vals[c])
            else:
                parts.append("0")                 # a column the simulator doesn't know yet
        self.send("D,RAW," + ",".join(parts))
        self.raw_rows += 1

    def raw_start(self):
        hz = self.P("raw_hz")
        self.raw_rows = 0
        self.step = 0
        self.raw_on = hz > 0
        if not self.raw_on:
            return
        self.raw_period = int(1000.0 / min(hz, 100.0))
        self.send("H,RAW," + ",".join(self.raw_cols))
        self.raw_next = self.millis()

    # ================================================= firmware: blink codes (blink.h)
    def blink_code_for(self):
        c = int(self.P("blink_code"))
        if 1 <= c <= 4:
            return c
        return {"Left": 1, "Center": 2, "Right": 3}.get(self.name, 4)

    def blink_set(self, v):
        if v == self.blink_lit:
            return
        self.blink_lit = v
        k = 1 if v else 0
        self.set_leds((k, k, k), (k, k, k))

    @staticmethod
    def blink_slot_lit(s, n):
        """Rate code (2026-10-18a): one 100 ms flash every (n + 2) slots."""
        return s % (n + 2) == 0

    def blink_start(self):
        self.blink_on = self.P("blink_codes") > 0
        if not self.blink_on:
            return
        self.blink_code = self.blink_code_for()
        self.blink_t0 = self.millis()
        self.blink_lit = True                      # force the first set() to send
        self.blink_set(False)

    def blink_tick(self):
        if not self.blink_on:
            return
        slot = (self.millis() - self.blink_t0) // BLINK_SLOT_MS
        if slot < BLINK_LEAD_SLOTS:                # 300 ms dark first, so the start flash stands alone
            self.blink_set(False)
            return
        slot -= BLINK_LEAD_SLOTS
        self.blink_set(self.blink_slot_lit(int(slot), self.blink_code))

    def blink_hold(self):
        if self.blink_on:
            self.blink_set(False)

    def blink_stop(self):
        if not self.blink_on:
            return
        self.blink_set(False)
        self.blink_on = False

    # ================================================= firmware: motion
    def loop_delay(self, d):
        self.advance(d + self.rng.uniform(*self.m.loop_overhead_ms))

    def set_wheels_speed(self, l_rpm, r_rpm):
        if self.mode == "move":
            self.mode = "vel"
        self.cmdL, self.cmdR = l_rpm * 6.0, r_rpm * 6.0      # rpm -> wheel deg/s

    def brake(self):
        self.mode = "vel"
        self.mv = None
        self.cmdL = self.cmdR = 0.0

    def should_abort(self):
        self.tick()
        if self.abort_reason:
            return True
        if self.P("safe_lift") > 0 and self.lifted:
            self.abort_reason = "robot lifted"
        else:
            lim = self.P("safe_front_cm")
            if lim > 0:
                dc = self.tof()[2]                 # the robot's own (held, noisy) centre-zone reading
                if 0 < dc < lim:
                    self.abort_reason = "obstacle ahead"
        if self.abort_reason:
            self.brake()
            return True
        return False

    def wait_ms(self, ms):
        t0 = self.millis()
        while self.millis() - t0 < ms:
            if self.should_abort():
                return False
            self.loop_delay(5)
        return True

    def log_wait(self, ms):
        """Logs, but no safety checks and no WiFi commands (used around the sync flash)."""
        t0 = self.millis()
        while self.millis() - t0 < ms:
            self.tick()
            self.advance(5.3, busy=True)

    def tick_wait(self, ms):
        """motion.h tickWait(): logs (and blinks), no safety checks, no WiFi commands."""
        self.log_wait(ms)

    def calibrate_yaw_sign(self, force=False):
        """motion.h calibrateYawSign(): a timed wiggle left, read the raw yaw change, wiggle back."""
        if self.fw_yaw_sign != 0 and not force:
            return True
        self.send("M,checking compass direction: small wiggle left and back")
        y0, o0 = self.yaw_raw, self.odo_th
        self.set_wheels_speed(-20, 20)              # left wheel back, right forward = turn left
        self.tick_wait(300)
        self.brake()
        self.tick_wait(300)
        dy = wrap180(self.yaw_raw - y0)
        d_odo = wrap180(self.odo_th - o0)
        if abs(d_odo) >= 2.0:
            self.fw_odo_sign = 1 if d_odo > 0 else -1
        if abs(dy) < 2.0:
            self.send("M,yaw check failed: heading moved only %.1f deg" % dy)
            self.fail = "yaw check failed"
            return False
        self.fw_yaw_sign = 1 if dy > 0 else -1
        self.send("M,yaw check ok: left turn moved raw yaw %.1f deg, sign %d" % (dy, self.fw_yaw_sign))
        self.set_wheels_speed(20, -20)              # undo the nudge
        self.tick_wait(300)
        self.brake()
        self.tick_wait(300)
        return True

    def imu_turn_to(self, target):
        rng, m = self.rng, self.m
        self.turn_slip = min(0.2, max(0.0, rng.gauss(m.turn_slip_mean_pct, m.turn_slip_sd_pct) / 100))
        t0, in_tol = self.millis(), 0
        kp, vmax, vmin, tol = self.P("turn_kp"), self.P("turn_max_rpm"), self.P("turn_min_rpm"), self.P("turn_tol_deg")
        while True:
            if self.should_abort():
                return False
            if self.millis() - t0 > self.P("turn_timeout_ms"):
                self.brake()
                self.send("M,turn timed out")
                self.fail = "turn timed out"
                return False
            err = wrap180(target - self.heading())
            if abs(err) <= tol:
                self.brake()
                if not in_tol:
                    in_tol = self.millis()
                if self.millis() - in_tol >= self.P("turn_hold_ms"):
                    return True
            else:
                in_tol = 0
                v = min(vmax, max(vmin, kp * abs(err)))
                if err > 0:
                    self.set_wheels_speed(-v, v)
                else:
                    self.set_wheels_speed(v, -v)
            self.loop_delay(5)

    def library_move(self, cm):
        m, rng = self.m, self.rng
        t0 = self.millis()
        limit = 3000 + abs(cm) * 400
        self.tick()
        self.blink_hold()                                 # dark before the call that blocks
        self.advance(m.library_block_ms, busy=True)       # move() waits inside the library first
        r0 = m.wheel_radius_cm
        dist_deg = abs(cm) / (2 * math.pi * r0) * 360.0      # the library converts with the nominal radius
        vmax = m.move_speed_cms / (2 * math.pi * r0) * 360.0
        acc = m.move_accel_cms2 / (2 * math.pi * r0) * 360.0
        self.mode = "move"
        self.mv = Move(dist_deg, vmax, acc, self.phiL, self.phiR, 1 if cm >= 0 else -1)
        self.slipL = max(0.0, rng.gauss(m.move_slip_mean_pct, m.move_slip_sd_pct) / 100)
        self.slipR = max(0.0, rng.gauss(m.move_slip_mean_pct, m.move_slip_sd_pct) / 100)
        while not (self.mv is not None and self.mv.t >= self.mv.total + m.move_done_settle_ms / 1000):
            if self.should_abort():
                return False
            if self.millis() - t0 > limit:
                self.brake()
                self.abort_reason = "move timed out"
                return False
            self.loop_delay(10)
        self.mode, self.mv = "vel", None
        self.cmdL = self.cmdR = self.wL = self.wR = 0.0
        self.advance(m.target_reached_ms, busy=True)      # the successful is_target_reached() waits
        return True

    def roll_straight(self, cm, rpm):
        x0, y0 = self.odo_x, self.odo_y
        t0 = self.millis()
        self.set_wheels_speed(rpm, rpm)
        while math.hypot(self.odo_x - x0, self.odo_y - y0) < cm:
            if self.should_abort():
                return False
            if self.millis() - t0 > 3000 + cm * 600:
                self.brake()
                self.abort_reason = "wheels did not cover the roll distance (stalled?)"
                return False
            self.loop_delay(5)
        return True

    def drive_circle(self, radius, speed):
        w_deg = speed / radius * 57.2958
        prev, turned = self.heading(), 0.0
        t0 = self.millis()
        limit = 360.0 / w_deg * 1000.0 * 2.0 + 3000
        r0, b0 = self.m.wheel_radius_cm, self.m.wheel_track_cm
        w = math.radians(w_deg)
        vl, vr = speed - w * b0 / 2, speed + w * b0 / 2      # alvik.drive(): nominal sizes
        to_rpm = 60 / (2 * math.pi * r0)
        self.set_wheels_speed(vl * to_rpm, vr * to_rpm)
        while abs(turned) < 360.0 - self.P("demo_circle_lead_deg"):
            if self.should_abort():
                return False
            if self.millis() - t0 > limit:
                self.brake()
                self.abort_reason = "circle timed out"
                return False
            h = self.heading()
            turned += wrap180(h - prev)
            prev = h
            self.loop_delay(5)
        self.brake()
        return True

    # ================================================= firmware: experiments
    def sync_flash(self, tag):
        if self.P("sync_flash") <= 0:
            return
        self.set_leds(OFF, OFF)
        self.log_wait(150)
        on = self.millis()
        self.set_leds(WHITE, WHITE)
        self.log_wait(int(self.P("sync_ms")))
        self.set_leds(OFF, OFF)
        off = self.millis()
        self.log_wait(150)
        self.send("H,SYNC,run,tag,ms_on,ms_off")
        self.send("D,SYNC,%s,%s,%d,%d" % (self.run_id, tag, on, off))
        self.syncs.append((tag, on, off))

    def wifi_dbm(self):
        return int(round(self.rng.gauss(self.wifi_dbm_mean, 2)))

    def log_run_info(self, i):
        self.send("H,RUNINFO,run,experiment,fw_version,motor_fw,ms_start,battery,charging,wifi_dbm,route,"
                  "yaw_sign,odo_sign")
        self.send("D,RUNINFO,%s,%s,%s,%s,%d,%d,%d,%d,%s,%d,%d" % (self.run_id, EXP_NAMES[i], self.fw_version,
                                                                 self.cfg.session.motor_fw, self.millis(),
                                                                 int(self.battery), 0, self.wifi_dbm(), "SSLSE",
                                                                 self.fw_yaw_sign, self.fw_odo_sign))
        self.send("H,PARAM,run,name,value")
        for name in self.param_order:
            self.send("D,PARAM,%s,%s,%.4f" % (self.run_id, name, self.params[name]))
            self.advance(2.3, busy=True)              # delay(2) between PARAM packets

    def receive_command(self, text):
        """A console command arrived (set ... only; `run` goes through run_experiment)."""
        parts = text.split()
        if parts[0] == "set" and len(parts) == 3 and parts[1] in self.params:
            self.params[parts[1]] = float(parts[2])
            self.send("M,%s = %.3f" % (parts[1], float(parts[2])))
        else:
            self.send("M,unknown command '%s' (type help)" % text)

    def demo_log(self, step):
        self.step += 1
        self.send("D,DEMO_STEP,%s,%s,%s,%.1f,%.1f,%.1f,%d" % (self.run_id, self.name, step, self.odo_x, self.odo_y,
                                                            self.odo_th, self.millis()))

    def _arm_kidnap(self):
        if self.kidnap_plan is not None:
            self.kidnap_at = self.ms + self.kidnap_plan["at_s"] * 1000

    def run_straight(self):
        if not self.calibrate_yaw_sign():
            return
        self.send("H,STRAIGHT,run,robot,result,cmd_cm,rpm,odo_x_cm,odo_y_cm,odo_theta_deg,imu_dyaw_deg,total_ms,battery")
        d, rpm = self.P("straight_cm"), self.P("straight_rpm")
        self.odo_x = self.odo_y = self.odo_th = 0.0             # reset_pose(0, 0, 0)
        start = self.heading()
        t0 = self.millis()
        self._arm_kidnap()
        ok = self.roll_straight(d, rpm) if rpm > 0 else self.library_move(d)
        self.brake()
        self.step = 1
        ok = ok and self.wait_ms(self.P("settle_ms"))
        dyaw = wrap180(self.heading() - start)
        result = "done" if ok else ("aborted:" + self.abort_reason if self.abort_reason else "failed:" + self.fail)
        self.send("D,STRAIGHT,%s,%s,%s,%.0f,%.0f,%.2f,%.2f,%.2f,%.2f,%d,%d" % (
            self.run_id, self.name, result, d, rpm, self.odo_x, self.odo_y, self.odo_th, dyaw,
            self.millis() - t0, int(self.battery)))
        self.send("M,run %s: odometry says %.1f cm along, %.1f cm sideways. Now mark and measure the real end point."
                  % (self.run_id, self.odo_x, self.odo_y))

    def run_demo(self):
        route = int(self.P("demo_route"))
        left = route == 1 or (route == 0 and self.name.lower() == "left")
        center = route == 2 or (route == 0 and self.name.lower() == "center")
        square = route == 3
        if not (left or center or square):
            self.send("M,%s has no demo routine of its own. Pick one with: set demo_route 1 (L), 2 (circle) or 3 (square)" % self.name)
            self.fail = "no demo routine"
            return
        if not self.calibrate_yaw_sign():
            return
        self.send("H,DEMO_STEP,run,robot,step,x_cm,y_cm,theta_deg,ms")
        self.send("H,DEMO_END,run,robot,route,result,return_err_cm,heading_err_deg,total_ms,a_cm,b_cm,r_cm,battery")
        self.odo_x = self.odo_y = self.odo_th = 0.0
        start = self.heading()
        a, b, r = self.P("demo_a_cm"), self.P("demo_b_cm"), self.P("demo_r_cm")
        t0 = self.millis()
        self._arm_kidnap()
        ok = True
        self.demo_log("start")
        if square:
            side = self.P("square_cm")
            laps = max(1, int(self.P("square_laps")))
            a, b, r = side, laps, 0
            for k in range(1, 4 * laps + 1):
                if not ok:
                    break
                ok = ok and self.library_move(side)
                if ok:
                    self.demo_log("side")
                ok = ok and self.imu_turn_to(wrap180(start + 90.0 * k))
                if ok:
                    self.demo_log("turn_left")
        elif left:
            seq = [("forward_a", lambda: self.library_move(a)),
                   ("turn_right", lambda: self.imu_turn_to(wrap180(start - 90))),
                   ("forward_b", lambda: self.library_move(b)),
                   (None, lambda: self.wait_ms(500)),
                   ("back_b", lambda: self.library_move(-b)),
                   ("turn_back_left", lambda: self.imu_turn_to(start)),
                   ("back_a", lambda: self.library_move(-a))]
            for label, fn in seq:
                ok = ok and fn()
                if ok and label:
                    self.demo_log(label)
        else:
            seq = [("forward_a", lambda: self.library_move(a)),
                   ("circle", lambda: self.drive_circle(r, self.P("demo_speed_cms"))),
                   ("square_up", lambda: self.imu_turn_to(start)),
                   ("back_a", lambda: self.library_move(-a))]
            for label, fn in seq:
                ok = ok and fn()
                if ok:
                    self.demo_log(label)
        self.brake()
        err = math.hypot(self.odo_x, self.odo_y)
        head_err = wrap180(self.heading() - start)
        result = "done" if ok else ("aborted:" + self.abort_reason if self.abort_reason else "failed:" + self.fail)
        self.send("D,DEMO_END,%s,%s,%s,%s,%.1f,%.1f,%d,%.0f,%.0f,%.0f,%d" % (
            self.run_id, self.name, "square" if square else ("L" if left else "circle"), result, err, head_err,
            self.millis() - t0, a, b, r, int(self.battery)))
        self.send("M,%s demo %s: back within %.1f cm of start (wheel odometry), heading off by %.1f deg. Measure with a ruler too."
                  % (self.name, "finished" if ok else "stopped", err, head_err))

    def run_calib(self):
        """runCalib() from home_lab.ino: RAW step 1 still and blinking, 2 forward, 3 back, 4 wiggle, 5 settle."""
        self.send("M,%s calibration: blinking code %d, rolling forward and back, then a small wiggle"
                  % (self.name, self.blink_code_for()))
        n = max(3.0, self.P("start_nudge_cm"))
        self.step = 1
        if not self.wait_ms(int(self.P("calib_blink_ms"))):
            return
        self.step = 2
        if not self.library_move(n) or not self.wait_ms(300):
            return
        self.step = 3
        if not self.library_move(-n) or not self.wait_ms(300):
            return
        self.step = 4
        if not self.calibrate_yaw_sign(True):
            return
        self.step = 5
        if not self.wait_ms(300):
            return
        self.send("M,calibrated: now place the robot on its start mark")

    def start_nudge(self, i):
        """startNudge(): roll start_nudge_cm forward and back after the start flash (RAW step -1)."""
        n = self.P("start_nudge_cm")
        if n <= 0 or i not in (1, 4, 5):
            return True
        self.step = -1
        ok = self.library_move(n) and self.wait_ms(200) and self.library_move(-n) and self.wait_ms(300)
        self.step = 0
        return ok

    def run_experiment(self, exp):
        """runExperiment() from home_lab.ino. Returns a dict describing the run, or None if the
        firmware refused to start it (turn/grid/demo/straight before any compass check)."""
        i = EXP_NAMES.index(exp)
        if i in (1, 3, 4, 5) and self.fw_yaw_sign == 0:       # needsYawSign(), started from the laptop
            self.send("M,not calibrated since power-on: lift the robot off its mark, send calibrate, put it back, "
                      "then start again (%s not started)" % exp)
            return None
        self.run_n += 1
        self.run_id = "%04X-%d" % (self.boot_id, self.run_n)
        self.abort_reason = self.fail = ""
        if i != EXP_CALIB:
            self.selected = i                  # a calibration keeps showing the chosen experiment
        self.syncs = []
        self.show_selection(True)
        self.send("M,run %s: %s starting" % (self.run_id, exp))
        t0 = self.millis()
        ms_runinfo = self.ms
        self.log_run_info(i)
        self.raw_start()
        self.wait_ms(self.P("settle_start_ms"))
        self.sync_flash("start")
        self.show_selection(True)
        self.blink_start()                     # after the start flash, so the two can't be confused
        ms_blink = self.ms if self.blink_on else None
        if not self.abort_reason:
            self.start_nudge(i)
        if not self.abort_reason:
            if exp == "straight":
                self.run_straight()
            elif exp == "demo":
                self.run_demo()
            elif exp == "calib":
                self.run_calib()
            else:
                raise ValueError("the simulator runs only the straight, demo and calib experiments, not " + exp)
        self.brake()
        self.kidnap_at = None
        self.blink_stop()
        self.sync_flash("end")
        self.raw_on = False
        result = ("stopped:" + self.abort_reason) if self.abort_reason else \
                 ("failed:" + self.fail) if self.fail else "done"
        self.send("H,RUNEND,run,result,ms_end,duration_ms,battery,wifi_dbm,raw_rows_sent,packets_sent")
        # packets_sent (net::seq) is only known once clock answers are merged in: filled later
        self.send("D,RUNEND,%s,%s,%d,%d,%d,%d,%d,{SEQ}" % (self.run_id, result, self.millis(), self.millis() - t0,
                                                         int(self.battery), self.wifi_dbm(), self.raw_rows))
        ms_end = self.ms
        if self.abort_reason:
            self.send("M,run %s: stopped (%s)" % (self.run_id, self.abort_reason))
        else:
            self.send("M,run %s: %s finished" % (self.run_id, exp))
        self.abort_reason = ""
        self.show_selection(False)
        run = dict(run=self.run_id, exp=exp, ms_start=ms_runinfo, ms_end=ms_end, result=result,
                   battery=int(self.battery), syncs=list(self.syncs), ms_blink=ms_blink,
                   blink_code=self.blink_code if ms_blink is not None else 0)
        self.runs.append(run)
        return run
