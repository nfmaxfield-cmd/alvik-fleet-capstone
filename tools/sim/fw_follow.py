"""Copy of the firmware's own-pose tracker, straight drives, survey drive and
step follower (firmware/home_lab/pose.h, follow.h, experiments.h runSurvey),
step for step, for the simulated robot (robot.py mixes this in).

Kept line-for-line close to the C++ so a change there is easy to mirror here.

Changelog: 2026-11-12e21 pose.h's e21 filter mirrored (the linear sd growth pose_sd_per_cm, pose_hist_ms over a 96-entry
history, refusals counted by reason, the consistency escape fix_escape_n, FOLLOW's five new columns).
The robot's sensors and motors are the simulator's: wheel odometry from the
quantized encoders with nominal sizes (the library's get_pose), heading from
the simulated gyro, the front distance sensor from ray casts against the room,
the map's boards and obstacles and (optionally) the other robots.
"""
import math

DEG2RAD = math.pi / 180.0


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def num(t):
    """follow.h num(): '-' or not a number -> NaN."""
    if t is None or t == "-":
        return float("nan")
    try:
        return float(t)
    except ValueError:
        return float("nan")


def isnan(v):
    return v != v


class PoseTracker:
    """pose.h, namespace pose."""
    NH = 96                 # (2026-11-12e21) pose.h NH: 96 x 40 ms
    H_MS = 40

    def __init__(self, rb):
        self.rb = rb
        self.on = False
        self.known = False
        self.x = self.y = 0.0
        self.th_off = 0.0
        self.var = self.var_th = 0.0
        self.dist = 0.0
        self.fixes_used = self.fixes_rejected = 0
        self.rej = dict(old=0, far=0, hist=0, future=0, first=0)
        self.escapes = 0
        self.esc = []
        self.last_reject = ""
        self.hist = []          # [(ms, x, y, th)], oldest first, at most NH
        self.h_last = 0
        self.last_ms = 0
        self.last_lx = self.last_ly = self.last_th = 0.0
        self.q_cm = self.q_deg = self.q_dps = 0.0
        self.drift_dps = 0.0     # gyro heading drift measured while docked (dock.h): taken off th() as time passes

    def lib_pose(self):
        rb = self.rb
        return rb.odo_x, rb.odo_y, rb.odo_th

    def th(self):
        return wrap180(self.rb.heading() + self.th_off)

    def push(self, now):
        self.hist.append((now, self.x, self.y, self.th()))
        if len(self.hist) > self.NH:
            self.hist.pop(0)
        self.h_last = now

    def start(self, known, x0=0.0, y0=0.0, th0=0.0):
        P = self.rb.P
        self.last_lx, self.last_ly, _ = self.lib_pose()
        self.x, self.y = x0, y0
        self.th_off = wrap180(th0 - self.rb.heading())
        self.last_th = self.th()
        self.var = self.var_th = 0.0
        self.dist = 0.0
        self.drift_dps = 0.0
        self.known = known
        self.fixes_used = self.fixes_rejected = 0
        self.rej = dict(old=0, far=0, hist=0, future=0, first=0)
        self.escapes = 0
        self.esc = []
        self.hist = []
        self.last_ms = self.rb.millis()
        self.q_cm, self.q_deg, self.q_dps = P("pose_q_cm"), P("pose_q_deg"), P("pose_q_dps")
        self.push(self.last_ms)
        self.on = True

    def stop(self):
        self.on = False

    def resume(self):
        """pose.h resume() (E02): tracking on again without forgetting x, y, th_off, known."""
        P = self.rb.P
        self.last_lx, self.last_ly, _ = self.lib_pose()
        self.last_th = self.th()
        self.dist = 0.0
        self.drift_dps = 0.0
        self.fixes_used = self.fixes_rejected = 0
        self.rej = dict(old=0, far=0, hist=0, future=0, first=0)
        self.escapes = 0
        self.esc = []
        self.hist = []
        self.last_ms = self.rb.millis()
        self.q_cm, self.q_deg, self.q_dps = P("pose_q_cm"), P("pose_q_deg"), P("pose_q_dps")
        self.push(self.last_ms)
        self.on = True

    def set(self, nx, ny, nth, sig, sig_th):
        self.x, self.y = nx, ny
        self.th_off = wrap180(nth - self.rb.heading())
        self.last_th = self.th()
        self.var, self.var_th = sig * sig, sig_th * sig_th
        self.esc = []
        self.known = True
        self.hist = []
        self.push(self.rb.millis())

    def tick(self):
        if not self.on:
            return
        lx, ly, lth = self.lib_pose()
        dx, dy = lx - self.last_lx, ly - self.last_ly
        self.last_lx, self.last_ly = lx, ly
        ds = (dx * math.cos(lth * DEG2RAD) + dy * math.sin(lth * DEG2RAD)) * self.rb.P("wheel_scale")   # real cm (corr.h)
        now = self.rb.millis()
        if self.drift_dps:
            self.th_off = wrap180(self.th_off - self.drift_dps * (now - self.last_ms) / 1000.0)
        h = self.th()
        dh = wrap180(h - self.last_th)
        mid = (self.last_th + 0.5 * dh) * DEG2RAD
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.dist += ds
        if self.rb.P("pose_sd_per_cm") > 0:          # (2026-11-12e21) the sd grows linearly
            self.var = (math.sqrt(self.var) + self.rb.P("pose_sd_per_cm") * abs(ds)) ** 2
        else:
            self.var += self.q_cm * abs(ds)
        self.var_th += self.q_deg * abs(dh) + self.q_dps * (now - self.last_ms) / 1000.0
        self.last_th = h
        self.last_ms = now
        if now - self.h_last >= self.H_MS:
            self.push(now)

    def at(self, t):
        if not self.hist:
            return None
        n = self.hist[-1]
        if t >= n[0]:
            now = self.rb.millis()
            span = now - n[0]
            f = (t - n[0]) / span if span > 0 else 1.0
            f = min(f, 1.0)
            return (n[1] + f * (self.x - n[1]), n[2] + f * (self.y - n[2]), n[3] + f * wrap180(self.th() - n[3]))
        now0 = self.rb.millis()
        hist_ms = self.rb.P("pose_hist_ms")
        for k in range(len(self.hist) - 1, 0, -1):
            b, a = self.hist[k], self.hist[k - 1]
            if hist_ms > 0 and now0 - a[0] > hist_ms:
                break                                  # (2026-11-12e21) only pose_hist_ms of it
            if t >= a[0]:
                f = (t - a[0]) / (b[0] - a[0])
                return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * wrap180(b[3] - a[3]))
        return None

    def reject(self, why, key=None):
        self.last_reject = why
        self.fixes_rejected += 1
        if key:
            self.rej[key] = self.rej.get(key, 0) + 1
        return False

    def fix(self, t, fx, fy, fth, sig, sig_th):
        P = self.rb.P
        now = self.rb.millis()
        if now - t > P("fix_max_age_ms"):
            return self.reject("old", "old")
        if t - now > 200:
            return self.reject("future", "future")
        if sig <= 0:
            sig = 0.1
        if sig_th <= 0:
            sig_th = 1.0
        if not self.known:
            if isnan(fth):
                return self.reject("first fix needs a heading", "first")
            self.set(fx, fy, fth, sig, sig_th)
            self.fixes_used += 1
            return True
        h = self.at(t)
        if h is None:
            return self.reject("older than the pose history", "hist")
        hx, hy, hth = h
        ex, ey = fx - hx, fy - hy
        S = self.var + sig * sig
        gate = P("fix_gate")
        if ex * ex + ey * ey > gate * gate * S and ex * ex + ey * ey > 9.0:
            need = int(P("fix_escape_n"))           # (2026-11-12e21) the consistency escape
            if need > 0:
                self.esc.append((ex, ey))
                del self.esc[:-need]
                if len(self.esc) >= need:
                    mx = sum(q[0] for q in self.esc) / len(self.esc)
                    my = sum(q[1] for q in self.esc) / len(self.esc)
                    if all((q[0] - mx) ** 2 + (q[1] - my) ** 2 <= gate * gate * sig * sig for q in self.esc):
                        self.x += mx
                        self.y += my
                        self.hist = [(ms, qx + mx, qy + my, qth) for (ms, qx, qy, qth) in self.hist]
                        self.var = sig * sig + 0.25 * (mx * mx + my * my)
                        self.esc = []
                        self.escapes += 1
                        self.fixes_used += 1
                        self.last_reject = ""
                        return True
            return self.reject("too far from the own pose", "far")
        self.esc = []
        k = self.var / S
        dx, dy = k * ex, k * ey
        self.var = (1 - k) * self.var
        dth = 0.0
        if not isnan(fth):
            eth = wrap180(fth - hth)
            Sth = self.var_th + sig_th * sig_th
            if not (eth * eth > gate * gate * Sth and abs(eth) > 5.0):
                kt = self.var_th / Sth
                dth = kt * eth
                self.var_th = (1 - kt) * self.var_th
        c, s = math.cos(dth * DEG2RAD), math.sin(dth * DEG2RAD)
        rx, ry = self.x - hx, self.y - hy
        self.x = hx + c * rx - s * ry + dx
        self.y = hy + s * rx + c * ry + dy
        new = []
        for (ms, qx, qy, qth) in self.hist:
            qx, qy = qx - hx, qy - hy
            new.append((ms, hx + c * qx - s * qy + dx, hy + s * qx + c * qy + dy, wrap180(qth + dth)))
        self.hist = new
        self.th_off = wrap180(self.th_off + dth)
        self.last_th = self.th()
        self.fixes_used += 1
        return True

    def sd(self):
        return math.sqrt(self.var)

    def sd_th(self):
        return math.sqrt(self.var_th)


DRIVE_OK, DRIVE_ABORT, DRIVE_STEPSTOP, DRIVE_TIMEOUT = range(4)
NQ = 8


class FollowMixin:
    """Methods mixed into robot.Robot: pose.h's drives, the survey and follow.h."""

    def follow_init(self):
        self.pose = PoseTracker(self)
        self.tick_hook = None
        self.front_limit = -1.0
        self.front_soft = False
        self.step_stop = ""
        self.front_trip = 0.0
        self.front_low_since = 0
        self.front_path = False
        self.front_end_base = -1.0
        self.radio = None            # live simulation: delivers commands (fleet_sim.py)
        self.f_active = False
        self.f_q = []
        self.f_max_id = self.f_cur = self.f_last_done = self.f_last_failed = 0
        self.f_state = "idle"
        self.f_travel = 0.0
        self.f_next_log = 0
        self.f_period = 100
        self.f_quiet_said = False
        self.f_info = ""
        self.last_rx = 0
        self.dock_log = []           # simulation only: true pose at the end of every approach
        self.f_dock_hdr = self.f_touch_hdr = False
        self.survey_legs = []
        # (2026-11-05c) continuous curves (follow.h / pose.h driveCurve): R pieces, coast, farc, dodge
        self.curving = self.dodging = self.dodged = False
        self.coast_t0 = 0
        self.c_v = self.c_k = 0.0
        self.arc_set = None
        self.curve_look = -1.0
        self.dodge_k = self.dodge_cm = 0.0

    # ------------------------------------------------------------- pose.h drives
    def set_drive(self, v, w):
        r, half = 1.7, 4.45
        dv = w * DEG2RAD * half
        k = 60.0 / (2.0 * math.pi * r)
        self.set_wheels_speed(max(-70.0, min(70.0, (v - dv) * k)), max(-70.0, min(70.0, (v + dv) * k)))

    def drive_line(self, x0, y0, x1, y1, h, vmax, rev):
        """(11-05d) a reverse drive is not stopped by a tilt (gTiltBackOK)."""
        self.tilt_back_ok = bool(rev)
        try:
            return self._drive_line(x0, y0, x1, y1, h, vmax, rev)
        finally:
            self.tilt_back_ok = False

    def _drive_line(self, x0, y0, x1, y1, h, vmax, rev):
        P, pose = self.P, self.pose
        ux, uy = math.cos(h * DEG2RAD), math.sin(h * DEG2RAD)
        d = -1.0 if rev else 1.0
        mx, my = d * ux, d * uy
        L = (x1 - x0) * mx + (y1 - y0) * my
        kct, maxc, kh, acc = P("drive_kct"), P("drive_max_corr"), P("drive_kh"), P("drive_accel")
        vmin, lead_s = 2.0, 0.03
        t0 = last = self.millis()
        limit = 3000 + int(abs(L) / max(vmax, 1.0) * 2500.0)
        v = 0.0
        slack = 0.0          # (2026-11-05b) time slowed by fspd, added to the limit
        while True:
            if self.should_abort():
                return DRIVE_ABORT if self.abort_reason else DRIVE_STEPSTOP
            now = self.millis()
            dt = (now - last) / 1000.0
            last = now
            if getattr(self, "f_steer", None) is not None and not rev:    # (2026-11-05b) fsteer
                x1, y1 = self.f_steer
                self.f_steer = None
                h = math.degrees(math.atan2(y1 - pose.y, x1 - pose.x))
                mx, my = math.cos(h * DEG2RAD), math.sin(h * DEG2RAD)
                L = (x1 - pose.x) * mx + (y1 - pose.y) * my
                t0, slack = now, 0.0
                limit = 3000 + int(abs(L) / max(vmax, 1.0) * 2500.0)
            sc = max(0.0, min(1.5, getattr(self, "f_scale", 1.0)))
            if sc < 1.0:
                slack += (1.0 - sc) * dt * 1000.0
            rem = (x1 - pose.x) * mx + (y1 - pose.y) * my
            if rem <= v * lead_s + 0.05:
                break
            if self.front_end_base >= 0:
                self.front_limit = min(self.front_end_base, max(rem, 0.0) + P("front_end_cm"))
            if now - t0 > limit + slack:
                self.brake()
                self.fail = "drive timed out"
                return DRIVE_TIMEOUT
            cross = mx * (pose.y - y1) - my * (pose.x - x1)
            corr = max(-maxc, min(maxc, -kct * cross)) if rem > 3.0 else 0.0
            herr = wrap180(h + corr - pose.th())
            vt = min(vmax * sc, math.sqrt(2.0 * acc * max(rem, 0.0)) + vmin * min(sc, 1.0))
            v = min(vt, v + acc * dt + 0.5)
            w = max(-60.0, min(60.0, kh * herr))
            self.set_drive(d * v, w)
            self.loop_delay(5)
        self.brake()
        return DRIVE_OK

    def front_in_path(self):
        """motion.h frontInPath()."""
        d = self.tof()
        half, best = self.P("path_half_cm"), 0.0
        for k, a in enumerate((22.0, 11.0, 0.0, -11.0, -22.0)):
            if d[k] <= 0:
                continue
            lat, fwd = d[k] * math.sin(a * DEG2RAD), d[k] * math.cos(a * DEG2RAD)
            if abs(lat) <= half and (best == 0 or fwd < best):
                best = fwd
        return best

    def turn_to_floor(self, h):
        return self.imu_turn_to(wrap180(h - self.pose.th_off))

    # ------------------------------------------------------------- survey
    def survey_leg(self, leg, cmd, driven, how):
        d = self.tof()
        p = self.pose
        self.send("D,SURVEY_LEG,%s,%d,%.0f,%.1f,%s,%.2f,%.2f,%.2f,%.1f,%d" % (
            self.run_id, leg, cmd, driven, how, p.x, p.y, p.th(), d[2], self.millis()))
        self.survey_legs.append(dict(run=self.run_id, leg=leg, cmd=cmd, driven=driven, end=how))

    def run_survey(self):
        if not self.calibrate_yaw_sign():
            return
        P, pose = self.P, self.pose
        self.send("H,SURVEY_LEG,run,leg,cmd_cm,driven_cm,end,x_cm,y_cm,th_deg,front_cm,ms")
        self.send("H,SURVEY_END,run,result,legs,path_cm,return_cm,x_cm,y_cm,th_deg,total_ms,size_cm,battery")
        size, v = P("survey_size_cm"), P("survey_cms")
        pose.start(True, 0.0, 0.0, 0.0)
        self.tick_hook = pose.tick
        self.front_soft = True
        self.front_limit = max(P("survey_clear_cm"), P("safe_front_cm"))
        self.step_stop = ""
        t0 = self.millis()
        cx = cy = path = 0.0
        legs = 0
        ok, blocked = True, False
        k = 0
        while True:
            k += 1
            L = 10.0 * ((k + 1) // 2)
            if L > size:
                break
            h = 90.0 * (k - 1)
            self.step = k
            self.front_limit = P("safe_front_cm")
            if k > 1 and not self.turn_to_floor(wrap180(h)):
                if self.step_stop:
                    blocked = True
                    self.survey_leg(k, L, 0, "obstacle")
                else:
                    ok = False
                break
            hx, hy = math.cos(h * DEG2RAD), math.sin(h * DEG2RAD)
            d0 = pose.dist
            self.front_limit = max(P("survey_clear_cm"), P("safe_front_cm"))
            self.front_path = True
            r = self.drive_line(cx, cy, cx + L * hx, cy + L * hy, wrap180(h), v, False)
            self.front_path = False
            driven = pose.dist - d0
            path += abs(driven)
            legs = k
            if r == DRIVE_STEPSTOP:
                blocked = True
                self.survey_leg(k, L, driven, "obstacle")
                break
            if r != DRIVE_OK:
                ok = False
                self.survey_leg(k, L, driven, "stopped" if r == DRIVE_ABORT else "timeout")
                break
            self.survey_leg(k, L, driven, "full")
            cx += L * hx
            cy += L * hy
        self.step_stop = ""
        back = 0.0
        if ok and not self.abort_reason:
            self.step = legs + 1
            d = math.hypot(pose.x, pose.y)
            if d > 1.0:
                hb = math.degrees(math.atan2(-pose.y, -pose.x))
                sx, sy = pose.x, pose.y
                self.front_limit = P("safe_front_cm")
                ok = self.turn_to_floor(hb)
                d0 = pose.dist
                self.front_limit = max(P("survey_clear_cm"), P("safe_front_cm"))
                self.front_path = True
                if ok:
                    ok = self.drive_line(sx, sy, 0.0, 0.0, hb, v, False) == DRIVE_OK
                self.front_path = False
                back = pose.dist - d0
                path += abs(back)
                if not ok and self.step_stop:
                    self.fail = "blocked on the way back"
            self.step_stop = ""
            self.front_limit = P("safe_front_cm")
            if ok:
                ok = self.turn_to_floor(0.0)
        self.brake()
        self.tick_hook = None
        self.front_soft = False
        self.front_limit = -1.0
        self.step_stop = ""
        pose.stop()
        result = ("done:blocked" if blocked else "done") if ok else \
            ("aborted:" + self.abort_reason if self.abort_reason else "failed:" + self.fail)
        self.send("D,SURVEY_END,%s,%s,%d,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%.0f,%d" % (
            self.run_id, result, legs, path, back, pose.x, pose.y, pose.th(), self.millis() - t0, size,
            int(self.battery)))
        self.send("M,%s survey %s: %d legs, %.0f cm driven, back within %.1f cm of the start (own pose)"
                  % (self.name, result, legs, path, math.hypot(pose.x, pose.y)))

    # ------------------------------------------------------------- follow.h
    def read_front(self):
        return self.tof()[2]

    def f_report(self, sid, st, why):
        p = self.pose
        self.send("D,STEP,%s,%d,%s,%d,%.2f,%.2f,%.2f,%.1f,%s" % (
            self.run_id, sid, st, self.millis(), p.x, p.y, p.th(), self.read_front(), why))

    def f_log(self):
        now = self.millis()
        if now - self.f_next_log < 0:
            return
        self.f_next_log = now + self.f_period
        p = self.pose
        rj = getattr(p, "rej", {})
        self.send("D,FOLLOW,%s,%d,%d,%s,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.1f,%.2f,%d,%d,%d,%d,%d,%d" % (
            self.run_id, now, self.f_cur, self.f_state if p.known else "nopose", len(self.f_q), self.f_last_done,
            self.f_last_failed, p.x, p.y, p.th(), p.sd(), p.sd_th(), p.fixes_used, p.fixes_rejected,
            self.read_front(), self.f_travel, now - self.last_rx, rj.get("old", 0), rj.get("far", 0),
            rj.get("hist", 0), rj.get("future", 0), getattr(p, "escapes", 0)))

    def f_tick(self):
        self.pose.tick()
        # (2026-11-05b) heartbeat watchdog (follow.h tickHook)
        if (getattr(self, "fw_version", "") >= "2026-11-05b" and self.f_cur and self.P("follow_wd_ms") > 0 and
                self.millis() - self.last_rx > self.P("follow_wd_ms") and not self.step_stop and not self.abort_reason):
            self.step_stop = "watchdog"
            self.brake()
        # the four-sided virtual border (border.h, firmware 2026-10-27a): only the step stops
        if self.bd_follow_tick() and self.f_cur and not self.step_stop and not self.abort_reason:
            self.step_stop = "border"
            self.brake()
        self.f_log()

    # ---- border.h (firmware 2026-10-27a): the twin of the four-sided border ----
    BD_TOWARDS_CM = 0.5
    BD_MOSTLY = 0.5
    BD_HALF_CM = 5.0

    def bd_dist4(self, px, py):
        P = self.P
        if P("bd_on") <= 0:
            return None
        d = None
        for k, lo, hi, inside, along in (("bd_k_x0", "bd_lo_x0", "bd_hi_x0", px - P("bd_x0"), py),
                                         ("bd_k_x1", "bd_lo_x1", "bd_hi_x1", P("bd_x1") - px, py),
                                         ("bd_k_y0", "bd_lo_y0", "bd_hi_y0", py - P("bd_y0"), px),
                                         ("bd_k_y1", "bd_lo_y1", "bd_hi_y1", P("bd_y1") - py, px)):
            kind = int(P(k))
            if kind == 1 or (kind == 2 and P(lo) - self.BD_HALF_CM <= along <= P(hi) + self.BD_HALF_CM):
                d = inside if d is None else min(d, inside)
        return d

    def bd_reset(self):
        self.bd_ref = None
        self.bd_ref_dist = 0.0

    def bd_follow_tick(self):
        P = self.P
        if not getattr(self.pose, "known", False) or P("border_cm") <= 0 or P("bd_on") <= 0:
            self.bd_ref = None
            return False
        d = self.bd_dist4(self.pose.x, self.pose.y)
        self.bd_last = d
        if d is None:
            self.bd_ref = None
            return False
        dist = self.pose.dist
        if getattr(self, "bd_ref", None) is None:
            self.bd_ref, self.bd_ref_dist = d, dist
            return False
        if d > self.bd_ref or abs(dist - self.bd_ref_dist) > 30.0:
            self.bd_ref, self.bd_ref_dist = d, dist
        if d >= P("border_cm") or self.bd_ref - d < self.BD_TOWARDS_CM:
            return False
        if self.bd_ref - d < self.BD_MOSTLY * abs(dist - self.bd_ref_dist):
            return False
        self.bd_ref, self.bd_ref_dist = d, dist
        return True

    def f_drop(self, keep_up_to, why):
        kept = []
        for s in self.f_q:
            if keep_up_to and s["id"] <= keep_up_to:
                kept.append(s)
            else:
                self.f_report(s["id"], "dropped", why)
        self.f_q = kept

    def follow_command(self, verb, rest):
        tok = rest.split()[:16]
        n = len(tok)
        if verb == "fix":
            if n < 5:
                return
            sig_th = num(tok[5]) if n > 5 else 1.0
            try:
                t = int(tok[0])
            except ValueError:
                t = 0
            self.pose.fix(t, num(tok[1]), num(tok[2]), num(tok[3]), num(tok[4]), 1.0 if isnan(sig_th) else sig_th)
            return
        if verb == "fpose":
            if n < 3 or any(isnan(num(t)) for t in tok[:3]):
                self.send("M,usage: fpose x y heading [sigma_cm sigma_deg]")
                return
            self.pose.set(num(tok[0]), num(tok[1]), num(tok[2]), num(tok[3]) if n > 3 else 0.5,
                          num(tok[4]) if n > 4 else 2.0)
            self.send("M,%s: pose set to %.1f %.1f %.1f" % (self.name, self.pose.x, self.pose.y, self.pose.th()))
            return
        if verb == "hold":
            try:
                keep = int(tok[0]) if n > 0 else 0
            except ValueError:
                keep = 0
            self.f_drop(keep, "hold")
            self.f_report(keep, "held", "")
            return
        if verb in ("fspd", "fsteer"):              # (2026-11-05b) change the running drive (follow.h)
            if verb == "fspd":
                sc = num(tok[0]) if n > 0 else float("nan")
                if isnan(sc) or sc < 0 or sc > 1.5:
                    self.send("M,usage: fspd <scale 0-1.5>")
                    return
                self.f_scale = sc
                if sc <= 0 and self.curving and not self.f_cur:
                    self.curving = False
                    self.brake()
                    self.c_v = 0.0
            else:
                if n < 2 or isnan(num(tok[0])) or isnan(num(tok[1])):
                    self.send("M,usage: fsteer <x> <y>")
                    return
                self.f_steer = (num(tok[0]), num(tok[1]))
            st = getattr(self, "f_steer", None)
            self.send("D,FSPD,%s,%d,%d,%s,%.2f,%.2f,%.2f" % (self.run_id, self.millis(), self.f_cur, verb,
                                                           getattr(self, "f_scale", 1.0), st[0] if st else 0.0,
                                                           st[1] if st else 0.0))
            return
        if verb == "farc":                          # (2026-11-05c) bend the running curve piece or the coast
            k = num(tok[0]) if n > 0 else float("nan")
            L = num(tok[1]) if n > 1 else float("nan")
            if isnan(k) or abs(k) > 0.5 or (not isnan(L) and L <= 0):
                self.send("M,usage: farc <k 1/cm> [<len cm>]")
                return
            if self.f_cur and self.f_state == "curve":
                self.arc_set = (k, L)
                self.f_drop(0, "farc")
            elif self.curving:
                self.c_k, self.coast_t0 = k, self.millis()
            else:
                self.send("M,farc: no curve running")
                return
            self.send("D,FSPD,%s,%d,%d,farc,%.2f,%.4f,%.2f" % (self.run_id, self.millis(), self.f_cur,
                                                              getattr(self, "f_scale", 1.0), k, -1.0 if isnan(L) else L))
            return
        if verb == "halt":
            if self.curving and not self.f_cur:
                self.curving = False
                self.brake()
                self.c_v = 0.0
            self.f_drop(0, "halt")
            if self.f_cur:
                self.step_stop = "halted"
            self.f_report(0, "halted", "")
            return
        if n < 2:
            self.send("M,usage: fs <id> <T|D|A|W> ...")
            return
        try:
            sid = int(tok[0])
        except ValueError:
            sid = 0
        if sid == 0:
            self.f_report(0, "bad", "id must be 1 or more")
            return
        if sid <= self.f_max_id:
            self.f_report(sid, "dup", "")
            return
        if len(self.f_q) >= NQ:
            self.f_report(sid, "full", "")
            return
        s = dict(id=sid, type=tok[1][:1], rev=0, fmin=-1.0)
        ok = True
        typ = s["type"]
        if typ == "T" and n >= 3:
            s["h"] = num(tok[2])
            ok = not isnan(s["h"])
            if n >= 4 and not isnan(num(tok[3])):
                s["fmin"] = num(tok[3])
        elif (typ == "D" and n >= 9) or (typ == "A" and n >= 11):
            for k, key in enumerate(("x0", "y0", "x1", "y1", "h", "speed")):
                s[key] = num(tok[2 + k])
            ok = not any(isnan(s[k]) for k in ("x0", "y0", "x1", "y1", "h", "speed")) and s["speed"] > 0
            if typ == "D":
                try:
                    s["rev"] = 1 if int(tok[8]) else 0      # atoi()
                except ValueError:
                    s["rev"] = 0
                if n >= 10 and not isnan(num(tok[9])):
                    s["fmin"] = num(tok[9])
            else:
                s["handover"], s["gap"], s["max_travel"] = num(tok[8]), num(tok[9]), num(tok[10])
                ok = ok and not any(isnan(s[k]) for k in ("handover", "gap", "max_travel"))
                if n >= 12 and not isnan(num(tok[11])):
                    s["fmin"] = num(tok[11])
        elif typ == "C" and n >= 8:
            # follow.h (2026-10-28a): fs <id> C <cx> <cy> <r> <a0> <sweep> <cms> <fmin>
            for k, key in enumerate(("cx", "cy", "r", "a0", "sweep", "speed")):
                s[key] = num(tok[2 + k])
            ok = not any(isnan(s[k]) for k in ("cx", "cy", "r", "a0", "sweep", "speed")) and s["speed"] > 0 \
                and s["r"] > 0
            if n >= 9 and not isnan(num(tok[8])):
                s["fmin"] = num(tok[8])
        elif typ == "R" and n >= 8 and getattr(self, "fw_version", "") >= "2026-11-05c":
            for k, key in enumerate(("x0", "y0", "h", "k", "len", "speed")):
                s[key] = num(tok[2 + k])
            ok = not any(isnan(s[k]) for k in ("x0", "y0", "h", "k", "len", "speed")) and s["speed"] > 0 and \
                s["len"] > 0 and abs(s["k"]) <= 0.5
            if n >= 9 and not isnan(num(tok[8])):
                s["fmin"] = num(tok[8])
        elif typ == "Z" and n >= 3:
            try:
                s["dance_ms"] = int(tok[2])
            except ValueError:
                ok = False
        elif typ == "W" and n >= 3:
            try:
                s["wait_ms"] = int(tok[2])
            except ValueError:
                ok = False
        elif typ == "K" and n >= 7:
            for k, key in enumerate(("x1", "y1", "h", "max_travel", "back")):
                s[key] = num(tok[2 + k])
            ok = not any(isnan(s[k]) for k in ("x1", "y1", "h", "max_travel", "back"))
        elif typ == "P" and n >= 5:
            for k, key in enumerate(("h", "max_travel", "back")):
                s[key] = num(tok[2 + k])
            ok = not any(isnan(s[k]) for k in ("h", "max_travel", "back"))
        else:
            ok = False
        if not ok:
            self.f_report(sid, "bad", "could not read the step")
            return
        self.f_max_id = sid
        self.f_q.append(s)
        self.f_report(sid, "queued", "")

    def front_limit_for(self, fmin, drive=False):
        P = self.P
        lim = max(P("follow_look_cm"), P("safe_front_cm")) if drive else P("safe_front_cm")
        if fmin >= 0:
            lim = min(lim, fmin + P("tof_recess_cm") - P("front_margin_cm"))
        return lim if lim > 0 else 0.0

    def approach(self, s):
        P, pose = self.P, self.pose
        ux, uy = math.cos(s["h"] * DEG2RAD), math.sin(s["h"] * DEG2RAD)
        L = (s["x1"] - s["x0"]) * ux + (s["y1"] - s["y0"]) * uy
        remA = (s["x1"] - pose.x) * ux + (s["y1"] - pose.y) * uy
        cap = s["max_travel"] - (L - remA)
        recess, tmin, margin = P("tof_recess_cm"), P("tof_min_cm"), P("front_margin_cm")
        fuse, acc, kh, kct = P("fuse_from_cm"), P("drive_accel"), P("drive_kh"), P("drive_kct")
        target = s["gap"] + recess
        expect0 = remA + s["gap"] + recess
        tof_ms = int(P("tof_ms"))
        lag = P("tof_lag_ms") / 1000.0
        d0, total, wheel_to, v = pose.dist, 0.0, 0.0, 0.0
        n = low = 0
        wheels = False
        t0 = last = next_read = self.millis()
        limit = 5000 + int(max(remA, 0.0) / max(s["speed"], 0.5) * 3000.0)
        self.f_info = "no handover"
        while True:
            if self.should_abort():
                return False
            now = self.millis()
            dt = (now - last) / 1000.0
            last = now
            self.f_travel = pose.dist - d0
            travel = self.f_travel
            if travel >= cap - 0.02:
                self.f_info = "travel cap %.1f cm reached; %d readings" % (cap, n)
                break
            if now - t0 > limit:
                self.brake()
                self.fail = "approach timed out"
                return False
            if not wheels and now - next_read >= 0:
                next_read = now + tof_ms
                d = self.read_front()
                if d > tmin:
                    at = travel - v * lag
                    if d < expect0 - at - margin:
                        low += 1
                        if low >= 2:
                            self.step_stop = "obstacle"
                            self.front_trip = d
                            self.brake()
                            return False
                    else:
                        low = 0
                        if d <= fuse:
                            total += d + at
                            n += 1
                        if d <= s["handover"] and n >= 3:
                            est = total / n
                            wheel_to = est - target
                            wheels = True
                            self.f_info = "%d readings; wheels from %.2f cm" % (n, d)
                if not wheels and n < 3 and travel >= remA - 0.5:
                    wheel_to = remA
                    wheels = True
                    self.f_info = "wheels only: no distance readings"
            vt = s["speed"]
            if wheels:
                wheel_to = min(wheel_to, cap)
                rem = wheel_to - travel
                if rem <= v * 0.03 + 0.02:
                    break
                vt = min(vt, math.sqrt(2.0 * acc * max(rem, 0.0)) + 0.5)
            v = min(vt, v + acc * dt + 0.3)
            along = (s["x1"] - pose.x) * ux + (s["y1"] - pose.y) * uy
            cross = ux * (pose.y - s["y1"]) - uy * (pose.x - s["x1"])
            corr = max(-2.0, min(2.0, -kct * cross)) if along > 3.0 else 0.0
            herr = wrap180(s["h"] + corr - pose.th())
            self.set_drive(v, max(-20.0, min(20.0, kh * herr)))
            self.loop_delay(5)
        self.brake()
        self.f_travel = pose.dist - d0
        return True

    def drive_arc(self, s, cont=False):
        """pose.h driveArc() (firmware 2026-10-28a): round the circle on the own pose. Heading: the circle's tangent
        plus arc_kr deg per cm of radial error (towards the circle), turn rate fed forward (v / r). Progress: the
        robot's angle round the centre, unwrapped from the step's a0; it stops at a0 + sweep. cont: the next queued
        step carries on round the same circle (no slowing at the end)."""
        P, pose = self.P, self.pose
        cx, cy, r, sw, vmax = s["cx"], s["cy"], s["r"], s["sweep"], s["speed"]
        sg = 1.0 if sw >= 0 else -1.0
        end = s["a0"] + sw
        phi0 = math.degrees(math.atan2(pose.y - cy, pose.x - cx))
        prog = s["a0"] + wrap180(phi0 - s["a0"])
        prev = phi0
        kr, kh, acc = P("arc_kr"), P("drive_kh"), P("drive_accel")
        t0 = last = self.millis()
        limit = 3000 + int(abs(sw) * DEG2RAD * r / max(vmax, 1.0) * 2500.0)
        v = vmax if cont else 0.0
        while True:
            if self.should_abort():
                return False
            now = self.millis()
            dt = (now - last) / 1000.0
            last = now
            phi = math.degrees(math.atan2(pose.y - cy, pose.x - cx))
            prog += wrap180(phi - prev)
            prev = phi
            rem = sg * (end - prog) * DEG2RAD * r              # cm still to go round the circle
            if rem <= v * 0.03 + 0.05:
                break
            if now - t0 > limit:
                self.brake()
                self.fail = "arc timed out"
                return False
            e = math.hypot(pose.x - cx, pose.y - cy) - r       # + = outside the circle
            h = phi + sg * 90.0 + sg * max(-20.0, min(20.0, kr * e))
            herr = wrap180(h - pose.th())
            vt = vmax if cont else min(vmax, math.sqrt(2.0 * acc * max(rem, 0.0)) + 2.0)
            v = min(vt, v + acc * dt + 0.5)
            w = sg * math.degrees(v / r) + max(-60.0, min(60.0, kh * herr))
            self.set_drive(v, w)
            self.loop_delay(5)
        if not cont:
            self.brake()
        return True

    def open_side(self):
        d = [v if v > 0 else 999.0 for v in self.tof()]
        return 1 if min(d[0], d[1]) >= min(d[3], d[4]) else -1

    def drive_curve(self, x0, y0, h0, k, L, vmax):
        """pose.h driveCurve() (firmware 2026-11-05c): a piece of constant curvature on the own pose, no slowing at the
        end (the next piece or the coast carries on); farc bends it from where it is; the front check dodges."""
        P, pose = self.P, self.pose
        k *= P("arc_k_scale")
        L *= P("arc_d_scale")
        kr, kct, kh, acc = P("arc_kr"), P("arc_kct"), P("drive_kh"), P("drive_accel")
        st = {}

        def setup():
            st["line"] = abs(k) < 1e-4
            st["R"] = 0.0 if st["line"] else 1.0 / abs(k)
            st["sg"] = 1.0 if k >= 0 else -1.0
            if not st["line"]:
                hr = h0 * DEG2RAD
                st["cx"] = x0 - math.sin(hr) * st["sg"] * st["R"]
                st["cy"] = y0 + math.cos(hr) * st["sg"] * st["R"]
                st["prev"] = math.degrees(math.atan2(y0 - st["cy"], x0 - st["cx"]))
            st["s"] = 0.0
        setup()
        t0 = last = self.millis()
        slack = 0.0
        limit = 3000 + int(L / max(vmax, 1.0) * 2500.0)
        v = self.c_v
        low_n, prev_f = 0, -1.0
        while True:
            if self.should_abort():
                self.c_v = 0.0
                return DRIVE_ABORT if self.abort_reason else DRIVE_STEPSTOP
            now = self.millis()
            dt = (now - last) / 1000.0
            last = now
            if self.arc_set is not None:
                nk, nl = self.arc_set
                self.arc_set = None
                left = L - st["s"]
                x0, y0, h0 = pose.x, pose.y, pose.th()
                k = nk * P("arc_k_scale")
                L = left if isnan(nl) else nl * P("arc_d_scale")
                setup()
                t0, slack = now, 0.0
                limit = 3000 + int(L / max(vmax, 1.0) * 2500.0)
            if st["line"]:
                ux, uy = math.cos(h0 * DEG2RAD), math.sin(h0 * DEG2RAD)
                st["s"] = (pose.x - x0) * ux + (pose.y - y0) * uy
                cross = ux * (pose.y - y0) - uy * (pose.x - x0)
                href, lean = h0, max(-20.0, min(20.0, -kct * cross))
            else:
                phi = math.degrees(math.atan2(pose.y - st["cy"], pose.x - st["cx"]))
                st["s"] += st["sg"] * wrap180(phi - st["prev"]) * DEG2RAD * st["R"]
                st["prev"] = phi
                e = math.hypot(pose.x - st["cx"], pose.y - st["cy"]) - st["R"]
                href, lean = phi + st["sg"] * 90.0, st["sg"] * max(-20.0, min(20.0, kr * e))
            rem = L - st["s"]
            sc = max(0.0, min(1.5, getattr(self, "f_scale", 1.0)))
            if sc < 1.0:
                slack += (1.0 - sc) * dt * 1000.0
            if rem <= v * 0.03 + 0.05:
                break
            if now - t0 > limit + slack:
                self.brake()
                self.c_v = 0.0
                self.fail = "curve timed out"
                return DRIVE_TIMEOUT
            if self.curve_look > 0:
                d = self.front_in_path()
                fresh = d != prev_f
                prev_f = d
                if 0 < d < self.curve_look:
                    if fresh:
                        low_n += 1
                    need = int(P("obs_n"))
                    if low_n >= (need if need > 0 else 2):
                        self.front_trip = d
                        if P("arc_dodge_r") > 0 and sc > 0:
                            self.dodge_k = self.open_side() / P("arc_dodge_r")
                            self.dodge_cm = d
                            self.c_k = self.dodge_k
                            self.c_v = min(max(v, P("arc_vmin")), P("arc_dodge_cms"))
                            return "dodge"
                        self.step_stop = "obstacle"
                        self.brake()
                        self.c_v = 0.0
                        return DRIVE_STEPSTOP
                elif fresh:
                    low_n = 0
            vt = vmax * sc
            v = min(vt, v + acc * dt + 0.3) if v < vt else max(vt, v - acc * dt - 0.3)
            herr = wrap180(href + lean - pose.th())
            w = (0.0 if st["line"] else st["sg"] * math.degrees(v / st["R"])) + max(-60.0, min(60.0, kh * herr))
            self.set_drive(v, w)
            self.c_v = v
            self.c_k = 0.0 if st["line"] else st["sg"] / st["R"]
            self.loop_delay(5)
        return DRIVE_OK

    def dance(self, ms):
        """follow step Z (firmware 2026-10-28a dance.h): spins, wiggles and lights where it stands, for ms (0 = until
        stopped). The simulator turns; the lights are not drawn."""
        t0 = self.millis()
        moves = [(150.0, 2400), (0.0, 300), (-90.0, 350), (90.0, 350), (-90.0, 350), (90.0, 350), (0.0, 300),
                 (-150.0, 2400), (0.0, 600)]
        k = 0
        while ms <= 0 or self.millis() - t0 < ms:
            w, d = moves[k % len(moves)]
            k += 1
            self.set_drive(0.0, w)
            t1 = self.millis()
            while self.millis() - t1 < d:
                if self.should_abort():
                    self.brake()
                    return False
                self.loop_delay(5)
        self.brake()
        return True

    def run_step(self, s):
        self.f_info = ""
        t = s["type"]
        if t == "R":                                   # (2026-11-05c)
            if not self.curving and abs(wrap180(s["h"] - self.pose.th())) > 30.0:
                self.front_limit = self.front_limit_for(s["fmin"])
                if not self.turn_to_floor(s["h"]):
                    return False
            if not self.curving:
                self.c_v = 0.0
            self.front_limit = 0.0
            self.curve_look = self.front_limit_for(s["fmin"], True)
            d0 = self.pose.dist
            r = self.drive_curve(s["x0"], s["y0"], s["h"], s["k"], s["len"], s["speed"])
            self.curve_look = -1.0
            self.f_travel = self.pose.dist - d0
            if r == "dodge":
                self.dodged = True
                return False
            return r == DRIVE_OK
        if t == "C":
            self.front_limit = self.front_limit_for(s["fmin"])
            if abs(wrap180(s["a0"] + (90.0 if s["sweep"] >= 0 else -90.0) - self.pose.th())) > 20.0:
                if not self.turn_to_floor(s["a0"] + (90.0 if s["sweep"] >= 0 else -90.0)):
                    return False
            d0 = self.pose.dist
            ok = self.drive_arc(s, cont=s.get("cont", False))
            self.f_travel = self.pose.dist - d0
            return ok
        if t == "Z":
            self.front_limit = 0.0
            return self.dance(s.get("dance_ms", 0))
        if t == "T":
            self.front_limit = self.front_limit_for(s["fmin"])
            return self.turn_to_floor(s["h"])
        if t == "D":
            self.front_limit = 0.0 if s["rev"] else self.front_limit_for(s["fmin"])
            if abs(wrap180(s["h"] - self.pose.th())) > 10.0 and not self.turn_to_floor(s["h"]):
                return False
            self.front_limit = 0.0 if s["rev"] else self.front_limit_for(s["fmin"], True)
            self.front_end_base = -1.0 if s["rev"] else self.front_limit
            self.front_path = not s["rev"]
            d0 = self.pose.dist
            r = self.drive_line(s["x0"], s["y0"], s["x1"], s["y1"], s["h"], s["speed"], bool(s["rev"]))
            self.front_path = False
            self.front_end_base = -1.0
            self.f_travel = self.pose.dist - d0
            return r == DRIVE_OK
        if t == "A":
            self.front_limit = 0.0
            ok = self.approach(s)
            if ok:
                self.dock_log.append(dict(id=s["id"], ms=self.millis(), x=self.x, y=self.y, th=self.th,
                                          pose=(self.pose.x, self.pose.y, self.pose.th()), info=self.f_info))
            return ok
        if t == "W":
            self.front_limit = 0.0
            return self.wait_ms(s.get("wait_ms", 0))
        if t in ("K", "P"):
            ok = self.run_dock_step(s)            # fw_dock.py (dock.h)
            if ok and t == "P" and self.params.get("sw_back", 0.0) > 0:
                # a board visit backing in (tools/fleet: touch, then forward gap): logged like an approach's end
                self.dock_log.append(dict(id=s["id"], ms=self.millis(), x=self.x, y=self.y, th=self.th,
                                          pose=(self.pose.x, self.pose.y, self.pose.th()), info=self.f_info,
                                          face="back"))
            return ok
        self.fail = "unknown step type"
        return False

    def run_follow(self):
        if not self.calibrate_yaw_sign():
            return
        P = self.P
        self.send("H,STEP,run,id,state,ms,x_cm,y_cm,th_deg,front_cm,info")
        self.send("H,FOLLOW,run,ms,step_id,state,queued,last_done,last_failed,x_cm,y_cm,th_deg,pos_sd_cm,th_sd_deg,"
                  "fixes_used,fixes_rejected,front_cm,travel_cm,quiet_ms,rej_old,rej_far,rej_hist,rej_future,escapes")
        if getattr(self, "fw_version", "") >= "2026-11-05b":
            self.send("H,FSPD,run,ms,step_id,verb,scale,x,y")
        self.f_q = []
        self.f_max_id = self.f_cur = self.f_last_done = self.f_last_failed = 0
        self.f_dock_hdr = self.f_touch_hdr = False
        self.f_state = "idle"
        self.f_travel = 0.0
        self.f_quiet_said = False
        self.f_period = int(1000.0 / max(1.0, P("follow_log_hz")))
        self.f_next_log = self.millis()
        self.pose.start(False)
        self.f_active = True
        self.tick_hook = self.f_tick
        self.front_soft = True
        self.edge_soft = True
        self.front_limit = 0.0
        self.step_stop = ""
        self.send("M,%s follow: ready for steps (needs fpose or a camera fix first)" % self.name)
        while True:
            if self.should_abort():
                if self.abort_reason:
                    break
                self.step_stop = ""
                if self.curving:                       # (2026-11-05c) the edge guard (braked) ends a coast
                    self.curving = self.dodging = False
                    self.c_v = 0.0
            quiet = self.millis() - self.last_rx
            if quiet > P("follow_quiet_ms"):
                if not self.f_quiet_said and not self.f_q and not self.f_cur:
                    self.send("M,%s follow: nothing from the laptop for %d ms; standing still" % (self.name, quiet))
                    self.f_quiet_said = True
            else:
                self.f_quiet_said = False
            if self.curving and not (self.f_q and self.f_q[0]["type"] == "R" and self.pose.known):
                wd = P("follow_wd_ms") > 0 and quiet > P("follow_wd_ms")
                d = self.front_in_path()
                why = None if self.f_q else ("watchdog" if wd else "no pose" if not self.pose.known else
                                             "nothing queued" if self.millis() - self.coast_t0 > P("arc_coast_ms") else
                                             "something close ahead" if 0 < d < P("arc_dodge_stop_cm") else None)
                if self.f_q or why:
                    self.brake()
                    self.curving = self.dodging = False
                    self.c_v = 0.0
                    if why:
                        self.send("M,%s follow: coast ended (%s): stopped" % (self.name, why))
                else:
                    self.f_state = "dodge" if self.dodging else "coast"
                    v = max(P("arc_vmin"), self.c_v)
                    self.set_drive(v, math.degrees(self.c_k * v))
                    self.loop_delay(5)
                    continue
            if not self.f_q or not self.pose.known:
                self.f_state = "idle"
                self.loop_delay(5)
                continue
            if getattr(self, "fw_version", "") >= "2026-11-05b" and P("follow_wd_ms") > 0 and quiet > P("follow_wd_ms"):     # (2026-11-05b) watchdog
                self.f_drop(0, "watchdog")
                self.f_state = "idle"
                self.loop_delay(5)
                continue
            s = self.f_q.pop(0)
            nx = self.f_q[0] if self.f_q else None
            s["cont"] = bool(s["type"] == "C" and nx is not None and nx["type"] == "C" and
                             abs(nx["cx"] - s["cx"]) < 0.05 and abs(nx["cy"] - s["cy"]) < 0.05 and
                             abs(nx["r"] - s["r"]) < 0.05 and (nx["sweep"] >= 0) == (s["sweep"] >= 0))
            self.f_cur = s["id"]
            self.step = s["id"]
            self.f_state = {"T": "turn", "D": "drive", "A": "approach", "W": "wait", "K": "dock",
                            "P": "touch", "C": "arc", "Z": "dance", "R": "curve"}.get(s["type"], "?")
            self.f_travel = 0.0
            self.step_stop = ""
            self.fail = ""
            if not (s["type"] == "R" and self.curving):
                self.f_scale = 1.0                      # (2026-11-05b) every step at its own speed and line
            self.f_steer = None                         # (2026-11-05c) an R piece run on keeps the scale
            self.dodged = False
            self.bd_reset()
            self.f_report(s["id"], "started", "")
            self.front_low_n = 0                       # obs_n counts readings within this step (follow.h)
            if (self.fw_version >= "2026-11-05d" and "tilt_stop_deg" in self.params and self.P("tilt_stop_deg") > 0 and
                    self.tilt_dev() > self.P("tilt_stop_deg") and not (s["type"] == "D" and s.get("rev"))):
                # (11-05d) tilted: no forward drive or turn starts; only a straight backward drive
                self.step_stop = "tilted %.1f deg: only a backward drive is allowed" % self.tilt_last
                ok = False
            else:
                ok = self.run_step(s)
            rolling = s["type"] == "R" and (ok or self.dodged) and not self.abort_reason
            if not (ok and s.get("cont")) and not rolling:
                self.brake()
            if not rolling:
                self.curving = self.dodging = False
                self.c_v = 0.0
            else:
                self.curving, self.dodging, self.coast_t0 = True, self.dodged, self.millis()
            self.front_limit = 0.0
            self.front_path = False
            self.front_end_base = -1.0
            if self.abort_reason:
                self.f_last_failed = s["id"]
                self.f_report(s["id"], "failed", self.abort_reason)
                self.f_cur = 0
                break
            if ok:
                self.f_last_done = s["id"]
                self.f_report(s["id"], "done", self.f_info)
            else:
                if self.dodged:
                    why = "dodged obstacle %.1f cm ahead; curving %s at %.1f cm/s" % (
                        self.dodge_cm, "left" if self.dodge_k > 0 else "right", self.c_v)
                elif self.step_stop == "obstacle":
                    d = self.tof()
                    why = "obstacle %.1f cm ahead; zones %.1f %.1f %.1f %.1f %.1f" % ((self.front_trip,) + tuple(d))
                elif self.step_stop == "border":
                    why = "virtual border at %.1f %.1f %.1f %.1f" % (self.pose.x, self.pose.y, self.pose.th(),
                                                                     getattr(self, "bd_last", None) or 0.0)
                elif self.step_stop == "edge":
                    why = "plywood edge at %.1f %.1f %.1f %s" % (self.pose.x, self.pose.y, self.pose.th(),
                                                               getattr(self, "edge_last_cue", ""))
                else:
                    why = self.step_stop or self.fail or "failed"
                self.f_last_failed = s["id"]
                self.f_report(s["id"], "failed", why)
                self.f_drop(0, "an earlier step failed")
                self.step_stop = ""
                self.fail = ""
            self.f_cur = 0
            self.step = 0
            self.f_state = "idle"
        self.f_drop(0, "run ended")
        self.curving = self.dodging = False
        self.c_v = 0.0
        self.f_active = False
        self.tick_hook = None
        self.front_soft = False
        self.edge_soft = False
        self.front_limit = -1.0
        self.step_stop = ""
        self.pose.stop()
