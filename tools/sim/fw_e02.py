"""E02 in the simulator: a copy of firmware/home_lab/e02.h (2026-10-22a) (bump detection without switches, push-to-square,
`run find`, `run e02`, camera fixes, camera-steered legs), step for step, mixed into robot.Robot.

What the simulator adds for it (robot.py, config.py): the boards are solids for the body (dock.contact_boards,
fw_dock.resolve_contacts); a wheel the contact holds back keeps only dock.wall_stall_frac of its commanded speed
(the motor stalls); the accelerometer's driving axis feels the stop (robot.accel_x: the body's own deceleration,
smoothed, plus vibration noise); floor seams (session.seams) jolt it and dip the wheels; heading() applies
gyro_scale / gyro_drift_dps and the own pose wheel_scale, as the firmware does. All of these are GUESSES
until the real robot has bumped a real board on camera; the tests check the firmware logic against them.
"""
import math

from fw_follow import DRIVE_OK, isnan, wrap180

DEG2RAD = math.pi / 180.0
UNSET = -999.0
NPATH = 200
NQ = 4


class Bump:
    def __init__(self):
        self.n = 0
        self.board = "x"
        self.ok = False
        self.why = ""
        self.cue = "-"
        self.ms_start = self.ms_contact = self.ms_detect = 0
        self.jolt = self.stall_frac = 0.0
        self.x = self.y = self.th = 0.0
        self.travel = self.overshoot = self.front = 0.0
        self.seams = 0
        self.square_deg = 0.0
        self.push_ms = 0
        self.squared = False
        self.true_contact = None          # simulation only: the true pose when the bump was detected


class Board:
    def __init__(self, nx, ny, c, toward, name):
        self.nx, self.ny, self.c, self.toward, self.name = nx, ny, c, toward, name


class FindRec:
    def __init__(self):
        self.result = "failed"
        self.why = ""
        self.known_before = 0
        self.bx = self.by = self.bth = 0.0
        self.b2x = self.b2y = self.b2th = 0.0
        self.rx = self.ry = self.rth = 0.0
        self.ax = self.ay = self.ath = 0.0
        self.corr1 = self.corr1_deg = self.corr2 = self.corr2_deg = 0.0
        self.turn_k = 1.0
        self.shift = 0.0


class Mark:
    def __init__(self):
        self.set = False
        self.x = self.y = self.th = 0.0
        self.ms = 0


class E02Mixin:
    """Mixed into robot.Robot: e02.h."""

    def e02_init(self):
        self.e_active = False
        self.e_finding = False
        self.e_mode1 = False
        self.e_cap = 0.0
        self.e_travel = 0.0
        self.e_last_dist = 0.0
        self.e_last_move = 0
        self.e_since_set = 0.0                 # wheel travel since the pose was last set (fix, find)
        self.e_back_since = 0.0                # backwards go legs since the last camera fix or find (blind: capped)
        self.e_fixes_used = self.e_fixes_rejected = 0
        self.e_path = []                       # [(ds, rel)] between the two bumps of a find
        self.e_path_on = False
        self.e_g0 = self.e_seg_ds = self.e_seg_rel = 0.0
        self.e_q = []
        self.e_last_id = 0
        self.e_end_marked = False
        self.e_idle_leg = False
        self.e_start, self.e_far, self.e_end = Mark(), Mark(), Mark()
        self.e_bumps = []                      # simulation only: every Bump (with the true pose at detection)
        self.e_finds = []                      # simulation only: (FindRec, true pose at the end)
        self.e_routes = []                     # simulation only: dict per e02 run with the true poses at the marks

    # ------------------------------------------------------------------ tracking
    def e_path_push(self):
        if self.e_seg_ds == 0:
            return
        rel = self.e_seg_rel / abs(self.e_seg_ds)
        if len(self.e_path) < NPATH:
            self.e_path.append((self.e_seg_ds, rel))
        else:
            ds0, r0 = self.e_path[-1]
            self.e_path[-1] = (ds0 + self.e_seg_ds, (r0 * abs(ds0) + rel * abs(self.e_seg_ds)) / (abs(ds0) + abs(self.e_seg_ds)))
        self.e_seg_ds = self.e_seg_rel = 0.0

    def e_tick_hook(self):
        p = self.pose
        p.tick()
        ds = p.dist - self.e_last_dist
        self.e_last_dist = p.dist
        self.e_travel += abs(ds)
        self.e_since_set += abs(ds)
        if self.e_path_on and ds != 0:
            self.e_seg_ds += ds
            self.e_seg_rel += wrap180(self.heading() - self.e_g0) * abs(ds)
            if abs(self.e_seg_ds) >= 0.5:
                self.e_path_push()
        sl, sr = self.wheels_speed()
        if abs(sl) + abs(sr) > 4.0 or abs(ds) > 0.05:
            self.e_last_move = self.millis()
        if self.e_cap > 0 and self.e_travel > self.e_cap and not self.abort_reason:
            self.abort_reason = "drove further than the route allows"

    def e_begin(self):
        if self.pose.known:
            self.pose.resume()
        else:
            self.pose.start(False)
        self.e_last_dist = 0.0
        self.e_travel = 0.0
        self.e_cap = 0.0
        self.e_last_move = -10 ** 9                  # runs start from standing still
        self.e_fixes_used = self.e_fixes_rejected = 0
        self.e_path_on = False
        self.tick_hook = self.e_tick_hook
        self.e_active = True

    def e_end_run(self):
        self.brake()
        self.e_path_on = False
        self.e_active = self.e_finding = self.e_mode1 = False
        self.e_cap = 0.0
        self.tick_hook = None
        self.front_soft = False
        self.edge_soft = False
        self.front_limit = -1.0
        self.front_path = False
        self.front_end_base = -1.0
        self.step_stop = ""
        self.pose.stop()

    def face_cm(self):
        f = self.P("e02_face_cm")
        if f > 0:
            return f
        return self.P("bump_ahead_cm") if self.P("bumper_cm") > 0 else self.P("tof_ahead_cm") + self.P("tof_recess_cm")

    def e_board(self, xb):
        P = self.P
        side = 1.0 if P("e02_xboard_side" if xb else "e02_yboard_side") > 0 else -1.0   # 0 or -1 low, 1 high
        pos = P("e02_xboard_cm" if xb else "e02_yboard_cm")
        a = P("e02_xboard_deg" if xb else "e02_yboard_deg") * DEG2RAD
        if xb:
            nx, ny = -math.sin(a) * side, math.cos(a) * side
            c = ny * pos
        else:
            nx, ny = math.cos(a) * side, math.sin(a) * side
            c = nx * pos
        return Board(nx, ny, c, math.degrees(math.atan2(ny, nx)), "x" if xb else "y")

    def to_go(self, b):
        return b.c - self.face_cm() - (b.nx * self.pose.x + b.ny * self.pose.y)

    def e_edge_off(self):
        """e02.h edgeOff(): the edge guard must be on before anything here drives. '' = fine."""
        return "" if self.P("safe_edge") > 0 else "edge guard off (safe_edge 0): set safe_edge 1 (or 2 / 3) first"

    # ------------------------------------------------------------------ records
    def bump_header(self):
        self.send("H,BUMP,run,ms,n,board,result,cue,ms_start,ms_contact,ms_detect,delay_ms,jolt_g,stall_frac,x_cm,y_cm,"
                  "th_deg,travel_cm,overshoot_cm,front_cm,jolts_ignored,square_deg,push_ms,squared")

    def bump_record(self, b):
        self.send("D,BUMP,%s,%d,%d,%s,%s,%s,%d,%d,%d,%d,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%d,%d" % (
            self.run_id, b.ms_detect or self.millis(), b.n, b.board, "bump" if b.ok else b.why, b.cue, b.ms_start, b.ms_contact, b.ms_detect,
            (b.ms_detect - b.ms_contact) if b.ms_detect else -1, b.jolt, b.stall_frac, b.x, b.y, b.th, b.travel,
            b.overshoot, b.front, b.seams, b.square_deg, b.push_ms, 1 if b.squared else 0))
        self.e_bumps.append(b)

    # ------------------------------------------------------------------ bump and square
    def e_creep(self, h, max_cm, b):
        P, pose = self.P, self.pose
        v, kh = P("bump_creep_cms"), P("drive_kh")
        jolt_thr, frac, far_cm = P("bump_jolt_g"), P("bump_stall_frac"), P("bump_far_cm")
        stall_ms, stall_only = int(P("bump_stall_ms")), int(P("bump_stall_only_ms"))
        win, spin = int(P("bump_jolt_win_ms")), int(P("bump_spinup_ms"))
        touch_cm, touch_ms = P("bump_touch_cm"), int(P("bump_touch_ms"))
        touch_since, dist_touch = 0, 0.0
        cmd_rpm = v * 60.0 / (2.0 * math.pi * 1.7)
        saved = self.front_limit
        self.front_limit = 0.0
        d0 = pose.dist
        base = peak = last_jolt_g = dist_jolt = dist_stall = 0.0
        have_base = False
        t0 = self.millis()
        last_jolt = stall_since = 0
        limit = 4000 + int(max_cm / max(v, 0.5) * 1500.0)
        b.ms_start = t0
        ok = False
        while True:
            if self.should_abort():
                break
            now = self.millis()
            travel = pose.dist - d0
            ax = self.accel_x()
            if not have_base:
                base, have_base = ax, True
            dev = abs(ax - base)
            armed = now - t0 >= spin
            if not armed or dev < jolt_thr:
                base += (ax - base) * 0.05
            elif armed:
                if not last_jolt or now - last_jolt > win:
                    last_jolt_g = 0.0
                    dist_jolt = travel
                    if not stall_since:
                        b.seams += 1
                last_jolt = now | 1
                last_jolt_g = max(last_jolt_g, dev)
            if stall_since:
                peak = max(peak, dev)
            sl, sr = self.wheels_speed()
            meas = 0.5 * (sl + sr)
            stalled = armed and meas < frac * cmd_rpm
            if stalled and not stall_since:
                stall_since, dist_stall, peak = now | 1, travel, dev
            if not stalled:
                stall_since = 0
            # touch (firmware 2026-10-22a): the front sensor's middle zones at most touch_cm for touch_ms
            if touch_cm > 0:
                dd = self.tof()
                near = min(dd[1], dd[2], dd[3])
                if near <= touch_cm:
                    if not touch_since:
                        touch_since, dist_touch = now | 1, travel
                    if now - touch_since >= touch_ms:
                        self.brake()
                        b.ms_detect = now
                        b.cue = "touch"
                        jolted = bool(last_jolt) and last_jolt + win - touch_since >= 0 and touch_since + win - last_jolt >= 0
                        early = jolted and last_jolt - touch_since < 0
                        b.ms_contact = last_jolt if early else touch_since
                        b.travel = dist_jolt if early else dist_touch
                        b.jolt = last_jolt_g if jolted else 0.0
                        b.stall_frac = meas / cmd_rpm
                        if jolted and b.seams > 0:
                            b.seams -= 1
                        b.front = near
                        b.true_contact = (self.x, self.y, self.th)
                        at = pose.at(b.ms_contact)
                        b.x, b.y, b.th = at if at is not None else (pose.x, pose.y, pose.th())
                        ok = True
                        break
                else:
                    touch_since = 0
            if stall_since:
                s = now - stall_since
                jolted = bool(last_jolt) and last_jolt + win - stall_since >= 0
                if (jolted and s >= stall_ms) or s >= stall_only:
                    self.brake()
                    b.ms_detect = now
                    b.cue = "jolt+stall" if jolted else "stall"
                    early = jolted and last_jolt - stall_since < 0
                    b.ms_contact = last_jolt if early else stall_since
                    b.travel = dist_jolt if early else dist_stall
                    b.jolt = max(peak, last_jolt_g) if jolted else peak
                    b.stall_frac = meas / cmd_rpm
                    if jolted and b.seams > 0:
                        b.seams -= 1
                    b.front = self.tof()[2]
                    b.true_contact = (self.x, self.y, self.th)
                    at = pose.at(b.ms_contact)
                    b.x, b.y, b.th = at if at is not None else (pose.x, pose.y, pose.th())
                    if far_cm < b.front < 100.0:
                        b.why = "stalled with something %.0f cm ahead: not at a board" % b.front
                        break
                    ok = True
                    break
            if travel > max_cm:
                b.why = "no bump within %.0f cm" % max_cm
                break
            if now - t0 > limit:
                b.why = "creep timed out"
                break
            herr = wrap180(h - pose.th())
            self.set_drive(v, max(-20.0, min(20.0, kh * herr)))
            self.loop_delay(5)
        self.brake()
        ts = self.millis()
        while self.millis() - ts < 400:
            if self.should_abort():
                ok = False
                break
            sl, sr = self.wheels_speed()
            if self.millis() - ts > 60 and abs(sl) < 0.5 and abs(sr) < 0.5:
                break
            self.loop_delay(5)
        if b.ms_detect:
            b.overshoot = (pose.dist - d0) - b.travel
        self.front_limit = saved
        b.ok = ok
        return ok

    def e_square(self, b):
        P = self.P
        rpm, still = P("push_rpm"), P("push_still_deg")
        max_ms, min_ms = int(P("push_ms")), int(P("push_min_ms"))
        saved = self.front_limit
        self.front_limit = 0.0
        h0 = h_ref = self.heading()
        t0 = t_ref = self.millis()
        settled, ok = False, True
        self.set_wheels_speed(rpm, rpm)
        while True:
            if self.should_abort():
                ok = False
                break
            now = self.millis()
            if now - t_ref >= 250:
                hh = self.heading()
                if now - t0 >= min_ms and abs(wrap180(hh - h_ref)) <= still:
                    settled = True
                    break
                h_ref, t_ref = hh, now
            if now - t0 >= max_ms:
                break
            self.loop_delay(5)
        self.brake()
        b.push_ms = self.millis() - t0
        if ok and not self.wait_ms(250):
            ok = False
        self.front_limit = saved
        b.square_deg = wrap180(self.heading() - h0)
        b.squared = settled
        b.true_square = (self.x, self.y, self.th)                  # simulation only
        if ok and not settled:
            b.why = "still turning after %d ms of pushing: not square" % b.push_ms
            b.ok = False
            self.fail = b.why
            return False
        return ok

    def e_straight(self, h, cm, cms):
        p = self.pose
        ux, uy = math.cos(h * DEG2RAD), math.sin(h * DEG2RAD)
        return self.drive_line(p.x, p.y, p.x + cm * ux, p.y + cm * uy, h, cms, cm < 0) == DRIVE_OK

    # ------------------------------------------------------------------ find
    def find_record(self, f, ms):
        self.send("H,FIND,run,result,known_before,before_x,before_y,before_th,bump2_x,bump2_y,bump2_th,reset_x,"
                  "reset_y,reset_th,after_x,after_y,after_th,corr1_cm,corr1_deg,corr2_cm,corr2_deg,turn_k,retrace_cm,"
                  "face_cm,ms_total,battery")
        res = f.result + (": " + f.why if f.why else "")
        self.send("D,FIND,%s,%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,"
                  "%.2f,%.2f,%d,%d" % (self.run_id, res, f.known_before, f.bx, f.by, f.bth, f.b2x, f.b2y, f.b2th, f.rx,
                                       f.ry, f.rth, f.ax, f.ay, f.ath, f.corr1, f.corr1_deg, f.corr2, f.corr2_deg,
                                       f.turn_k, f.shift, self.face_cm(), ms, int(self.battery)))

    def one_board(self, bd, n, known, f, b):
        P, pose = self.P, self.pose
        b.n, b.board = n, bd.name
        self.step = 1 if n == 1 else 5
        self.front_limit = 0.0
        if not self.turn_to_floor(bd.toward):
            f.why = "turn to the %s board %s" % (bd.name, "stopped" if self.abort_reason else "failed")
            return False
        max_cm = P("e02_find_max_cm")
        slack = P("e02_find_tol_pct") / 100.0 * self.e_since_set
        if known:
            expect = self.to_go(bd)
            margin = P("e02_find_margin_cm") + slack
            max_cm = min(max_cm, max(0.0, expect) + margin)
            pre = min(expect - P("e02_find_slow_cm"), P("e02_transit_max_cm"))
            if pre > 2.0:
                self.e_cap += pre
                self.front_limit = 0.5 * P("e02_find_slow_cm")
                self.front_soft = True
                ok = self.e_straight(bd.toward, pre, P("e02_cms"))
                self.front_soft = False
                self.front_limit = 0.0
                self.front_low_since = 0
                if not ok and self.step_stop == "obstacle":
                    ok, self.step_stop = True, ""
                if not ok:
                    f.why = "drive towards the %s board stopped (%s)" % (bd.name, self.abort_reason or self.fail)
                    return False
                max_cm = min(P("e02_find_max_cm"), P("e02_find_slow_cm") + margin)
        self.step = 2 if n == 1 else 6
        ok = self.e_creep(bd.toward, max(max_cm, 1.0), b)
        if ok and known:
            off = bd.c - self.face_cm() - (bd.nx * b.x + bd.ny * b.y)
            if off > P("e02_find_tol_cm") + slack:  # sooner: something in the way; later can only be the board
                b.why = "bump %.1f cm sooner than the %s board should be" % (off, bd.name)
                b.ok = ok = False
        if not ok:
            self.bump_record(b)
            if self.abort_reason == "plywood edge ahead":
                f.why = "edge guard tripped before the %s board (an open plywood edge, not a board)" % bd.name
            elif self.abort_reason:
                f.why = "stopped (%s)" % self.abort_reason
            else:
                f.why = "%s board: %s" % (bd.name, b.why)
            return False
        self.step = 3 if n == 1 else 7
        sq = self.e_square(b)
        self.bump_record(b)
        if not sq:
            f.why = ("stopped (%s)" % self.abort_reason) if self.abort_reason else "%s board: %s" % (bd.name, b.why)
            return False
        return True

    def run_find(self):
        P, pose = self.P, self.pose
        t0 = self.millis()
        self.bump_header()
        f = FindRec()
        known = pose.known
        f.known_before = 1 if known else 0
        bx, by = self.e_board(True), self.e_board(False)
        face, back = self.face_cm(), P("e02_find_back_cm")
        if self.e_edge_off():
            f.why = self.e_edge_off()
            self.fail = f.why
            self.find_record(f, 0)
            self.e_finds.append((f, (self.x, self.y, self.th)))
            self.send("M,%s find refused: %s" % (self.name, f.why))
            return
        self.e_begin()
        self.e_finding = True
        if not known:
            pose.start(True, 0.0, 0.0, bx.toward)
            self.send("M,%s find: position unknown (no find or camera fix since power-on): assuming it faces the x "
                      "board" % self.name)
        f.bx, f.by, f.bth = pose.x, pose.y, pose.th()
        self.e_cap = 2 * P("e02_find_max_cm") + 2 * back + 20
        ok = False
        while True:                                   # do { ... } while (false)
            stage = P("e02_find_stage_cm")
            det = bx.nx * by.ny - bx.ny * by.nx
            if known and stage > 0 and abs(det) > 0.5:
                c1, c2 = bx.c - stage, by.c - stage
                sx, sy = (c1 * by.ny - bx.ny * c2) / det, (bx.nx * c2 - c1 * by.nx) / det
                d = math.hypot(sx - pose.x, sy - pose.y)
                self.e_cap += d
                self.step = 0
                if not self.e_drive_to(sx, sy, False):
                    f.why = "drive to the staging point stopped"
                    break
            b1 = Bump()
            if not self.one_board(bx, 1, known, f, b1):
                break
            off1 = self.to_go(bx)
            f.corr1 = off1
            f.corr1_deg = wrap180(bx.toward - pose.th())
            pose.set(pose.x + off1 * bx.nx, pose.y + off1 * bx.ny, bx.toward, 0.05, 0.3)
            a1x, a1y = pose.x, pose.y
            self.e_g0 = self.heading()
            self.e_path = []
            self.e_seg_ds = self.e_seg_rel = 0.0
            self.e_path_on = True
            self.step = 4
            self.front_limit = 0.0
            if not self.e_straight(bx.toward, -back, 3.0):
                f.why = "backing off the x board stopped"
                break
            b2 = Bump()
            if not self.one_board(by, 2, known, f, b2):
                break
            self.e_path_push()
            self.e_path_on = False
            rel2 = wrap180(self.heading() - self.e_g0)
            true_turn = wrap180(by.toward - bx.toward)
            k = 1.0
            if abs(rel2) > 30.0 and abs(true_turn) > 30.0:
                k = true_turn / rel2
                if k < 0.8 or k > 1.25:
                    f.why = ("the gyro turned %.1f deg between the boards but they are %.1f deg apart "
                             "(board sides or angles set wrong?)" % (rel2, true_turn))
                    break
            f.turn_k = k
            px, py = a1x, a1y
            for ds, rel in self.e_path:
                th = (bx.toward + k * rel) * DEG2RAD
                px += ds * math.cos(th)
                py += ds * math.sin(th)
            f.b2x, f.b2y, f.b2th = pose.x, pose.y, pose.th()
            f.shift = math.hypot(px - pose.x, py - pose.y)
            f.corr2 = by.c - face - (by.nx * pose.x + by.ny * pose.y)
            f.corr2_deg = wrap180(by.toward - pose.th())
            off2 = by.c - face - (by.nx * px + by.ny * py)
            pose.set(px + off2 * by.nx, py + off2 * by.ny, by.toward, 0.05, 0.3)
            self.e_since_set = 0.0
            self.e_back_since = 0.0
            f.rx, f.ry, f.rth = pose.x, pose.y, pose.th()
            self.e_reset_true = (self.x, self.y, self.th)          # simulation only: true pose at the reset
            self.step = 8
            self.front_limit = 0.0
            if not self.e_straight(by.toward, -back, 3.0):
                f.why = "backing off the y board stopped"
                break
            ok = True
            break
        self.e_path_on = False
        f.ax, f.ay, f.ath = pose.x, pose.y, pose.th()
        if ok:
            f.result = "ok"
        else:
            if not known:
                pose.known = False
            if not self.abort_reason and not self.fail:
                self.fail = f.why
        self.find_record(f, self.millis() - t0)
        self.e_finds.append((f, (self.x, self.y, self.th)))
        if ok:
            self.send("M,%s find: at %.1f, %.1f facing %.1f deg (corrections %.1f cm / %.1f deg and %.1f cm / %.1f "
                      "deg; turn_k %.3f)" % (self.name, pose.x, pose.y, pose.th(), f.corr1, f.corr1_deg, f.corr2,
                                             f.corr2_deg, f.turn_k))
        else:
            self.send("M,%s find failed: %s" % (self.name, f.why))
        self.e_end_run()

    # ------------------------------------------------------------------ routes
    def e_turn_sign(self, param):
        v = self.P(param)
        if v > 0:
            return 1.0
        if v < 0:
            return -1.0
        bx = self.e_board(True)
        h = self.e_start.th * DEG2RAD
        return -1.0 if (-math.sin(h) * bx.nx + math.cos(h) * bx.ny) > 0 else 1.0

    def e_waypoints(self, route):
        P = self.P
        if route == 1:
            L = P("e02_straight_cm")
            return [(L, 0.0), (0.0, 0.0)], 0
        if route == 2:
            a, t = P("e02_l_cm"), self.e_turn_sign("e02_l_turn")
            return [(a, 0.0), (a, t * a), (a, 0.0), (0.0, 0.0)], 1
        s, t = P("e02_square_cm"), self.e_turn_sign("e02_square_turn")
        return [(s, 0.0), (s, t * s), (0.0, t * s), (0.0, 0.0)], 1

    def e_route_len(self, route):
        P = self.P
        if route == 1:
            return 2 * P("e02_straight_cm")
        if route == 2:
            return 4 * P("e02_l_cm")
        return 4 * P("e02_square_cm")

    def e_mark(self, m):
        p = self.pose
        m.set, m.x, m.y, m.th, m.ms = True, p.x, p.y, p.th(), self.millis()
        m.true = (self.x, self.y, self.th)                         # simulation only

    def e02_header(self):
        self.send("H,E02,run,route,mode,result,fix_k,start_x,start_y,start_th,far_goal_x,far_goal_y,far_x,far_y,far_th,"
                  "end_x,end_y,end_th,ms_start,ms_far,ms_end,path_cm,legs,fixes_used,fixes_rejected,gyro_scale,"
                  "gyro_drift_dps,wheel_scale,battery")

    def e_far_goal(self, route):
        w, far = self.e_waypoints(route if 1 <= route <= 3 else 1)
        s = self.e_start
        c, sn = math.cos(s.th * DEG2RAD), math.sin(s.th * DEG2RAD)
        return s.x + c * w[far][0] - sn * w[far][1], s.y + sn * w[far][0] + c * w[far][1]

    def e02_record(self, route, mode, result, legs):
        gx, gy = self.e_far_goal(route)
        s, f, e = self.e_start, self.e_far, self.e_end
        P = self.P
        self.send("D,E02,%s,%d,%d,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%.1f,%d,%d,"
                  "%d,%.4f,%.4f,%.4f,%d" % (self.run_id, route, mode, result, P("e02_fix_k"), s.x, s.y, s.th, gx, gy,
                                            f.x, f.y, f.th, e.x, e.y, e.th, s.ms, f.ms, e.ms, self.e_travel, legs,
                                            self.e_fixes_used, self.e_fixes_rejected, P("gyro_scale"),
                                            P("gyro_drift_dps"), P("wheel_scale"), int(self.battery)))

    def e_leg_pause(self):
        ms = self.P("e02_leg_pause_ms")
        return ms == 0 or self.wait_ms(ms)

    def e_drive_to(self, wx, wy, pause):
        p = self.pose
        dx, dy = wx - p.x, wy - p.y
        if math.hypot(dx, dy) < 1.0:
            return True
        hb = math.degrees(math.atan2(dy, dx))
        self.front_limit = 0.0
        if not self.turn_to_floor(hb) or (pause and not self.e_leg_pause()):
            return False
        dx, dy = wx - p.x, wy - p.y
        hb2 = math.degrees(math.atan2(dy, dx))
        if abs(wrap180(hb2 - hb)) > 2.0 and not self.turn_to_floor(hb2):
            return False
        self.front_end_base = self.P("safe_front_cm")
        self.front_limit = self.front_end_base
        r = self.drive_line(p.x, p.y, wx, wy, hb2, self.P("e02_cms"), False)
        self.front_end_base = -1.0
        self.front_limit = 0.0
        return r == DRIVE_OK and (not pause or self.e_leg_pause())

    def e_leg_to(self, wx, wy):
        return self.e_drive_to(wx, wy, True)

    def e_redo(self, wx, wy):
        r = self.P("e02_redo_cm")
        if r <= 0 or math.hypot(wx - self.pose.x, wy - self.pose.y) <= max(r, 1.0):
            return True
        return self.e_leg_to(wx, wy)

    def e_route_corrections(self):
        """e02.h routeCorrections(): the condition's corrections from the route's start on (-999 = keep)."""
        for src, dst in (("e02_route_gyro_scale", "gyro_scale"), ("e02_route_gyro_drift_dps", "gyro_drift_dps"),
                         ("e02_route_wheel_scale", "wheel_scale")):
            v = self.P(src)
            if v != UNSET:
                self.params[dst] = v

    def e_turn_to(self, h):
        self.front_limit = 0.0
        return self.turn_to_floor(h) and self.e_leg_pause()

    # ------------------------------------------------------------------ camera-steered legs
    def leg_header(self):
        self.send("H,LEG,run,id,kind,value,result,ms,x_cm,y_cm,th_deg,info")

    def add_front(self, info):
        """e02.h addFront() (2026-10-22b): the front sensor's middle zone standing still, after a finished leg."""
        d = self.tof()
        return (info + " " if info else "") + "front=%.1f" % d[2]

    def leg_record(self, lid, kind, value, result, info):
        p = self.pose
        self.send("D,LEG,%s,%d,%s,%.2f,%s,%d,%.2f,%.2f,%.2f,%s" % (
            self.run_id if (self.e_active and not self.e_idle_leg) else "-", lid, {"g": "go", "t": "turn"}.get(kind, "mark"), value, result,
            self.millis(), p.x, p.y, p.th(), info))

    def e_leg_command(self, verb, rest, running):
        tok = rest.split()[:4]
        kind = "g" if verb == "go" else "t" if verb == "turn" else "m"
        arg = 0.0
        try:
            lid = int(tok[1]) if len(tok) > 1 else 0
        except ValueError:
            lid = 0
        idle = not running and kind != "m"
        if not idle and (not running or not self.e_active or not self.e_mode1 or self.e_idle_leg):
            self.leg_header()
            self.leg_record(lid, kind, arg, "refused", "a leg is running" if self.e_idle_leg else
                            "marks are for a camera-steered e02 run" if kind == "m" else
                            "busy: not in a camera-steered e02 run")
            return
        if idle:
            self.leg_header()
        good = len(tok) >= 1
        if good and kind == "m":
            m = {"start": 0.0, "far": 1.0, "end": 2.0}.get(tok[0].lower())
            good = m is not None
            arg = m if good else 0.0
        elif good:
            try:
                arg = float(tok[0])
            except ValueError:
                good = False
        if not good:
            self.leg_record(lid, kind, arg, "refused", "could not read it (go <cm> | turn <deg> | mark start|far|end)")
            return
        if kind == "g" and (arg > self.P("e02_go_max_cm") or arg < -self.P("e02_back_max_cm")):
            self.leg_record(lid, kind, arg, "refused", "too long (e02_go_max_cm / e02_back_max_cm)")
            return
        if kind == "t" and abs(arg) > 180.0:
            self.leg_record(lid, kind, arg, "refused", "turns are -180 to 180 deg")
            return
        if kind == "g" and self.e_edge_off():
            self.leg_record(lid, kind, arg, "refused", self.e_edge_off())
            return
        if kind == "g" and arg < 0 and self.e_back_since - arg > self.P("e02_back_max_cm") + 0.05:
            self.leg_record(lid, kind, arg, "refused", "backing blind: more than e02_back_max_cm since the last camera fix")
            return
        if lid > 0 and lid <= self.e_last_id:
            self.leg_record(lid, kind, arg, "dup", "")
            return
        if idle:
            if lid > 0:
                self.e_last_id = lid
            self.leg_record(lid, kind, arg, "queued", "")
            self.e_idle_tick((kind, arg, lid))       # the firmware runs it from loop() next: here at once
            return
        if len(self.e_q) >= NQ:
            self.leg_record(lid, kind, arg, "refused", "queue full")
            return
        if lid > 0:
            self.e_last_id = lid
        self.e_q.append((kind, arg, lid))
        self.leg_record(lid, kind, arg, "queued", "")

    def e_run_leg(self, kind, arg):
        p = self.pose
        if kind == "g":
            self.front_end_base = self.P("safe_front_cm") if arg > 0 else -1.0
            self.front_limit = min(self.P("safe_front_cm"), arg + self.P("front_end_cm")) if arg > 0 else 0.0
            d0 = p.dist
            ok = self.e_straight(p.th(), arg, self.P("e02_cms"))
            if arg < 0:
                self.e_back_since += abs(p.dist - d0)
            self.front_end_base = -1.0
            self.front_limit = 0.0
            return ok, ""
        if kind == "t":
            self.front_limit = 0.0
            return self.turn_to_floor(wrap180(p.th() + arg)), ""
        m = int(arg)
        if m == 0:
            self.e_mark(self.e_start)
        elif m == 1:
            self.e_mark(self.e_far)
            if not self.wait_ms(self.P("e02_pause_ms")):
                return False, ""
        else:
            self.e_mark(self.e_end)
            self.e_end_marked = True
        return True, ("start", "far", "end")[m]

    def e_idle_tick(self, leg):
        """e02.h idleTick(): one idle leg, with the edge guard, soft stops and the length limits."""
        kind, arg, lid = leg
        self.abort_reason = self.fail = self.step_stop = ""
        self.e_begin()
        self.e_idle_leg = True
        self.front_soft = True
        self.edge_soft = True
        self.e_cap = abs(arg) + 10.0
        self.edge_begin()
        self.leg_record(lid, kind, arg, "started", "")
        self.front_limit = 0.0          # standing still: nothing to stop; the leg sets its own front check
        good = self.wait_ms(self.P("edge_learn_ms") + 50)
        info = ""
        if good:
            good, info = self.e_run_leg(kind, arg)
        self.brake()
        if self.abort_reason:
            self.leg_record(lid, kind, arg, "failed", self.abort_reason)
        elif good:
            self.leg_record(lid, kind, arg, "done", self.add_front(info))
        else:
            why = ("plywood edge (%s)" % getattr(self, "edge_last_cue", "")) if self.step_stop == "edge" else \
                (self.step_stop or self.fail or "failed")
            self.leg_record(lid, kind, arg, "failed", why)
        self.edge_stop()
        self.e_idle_leg = False
        self.e_end_run()
        self.abort_reason = self.fail = ""

    # ------------------------------------------------------------------ camera fix
    def e_fix_command(self, rest, running):
        tok = rest.split()[:5]
        self.send("H,FIX,run,ms,t_wall,cam_x,cam_y,cam_th,k,result,before_x,before_y,before_th,after_x,after_y,after_th")
        if len(tok) < 4:
            self.send("M,usage: fix <x> <y> <heading|-> <t_wall>")
            return
        p = self.pose

        def num(s):
            try:
                return float(s)
            except ValueError:
                return 0.0
        x, y = num(tok[0]), num(tok[1])
        th = float("nan") if tok[2] == "-" else num(tok[2])
        k = 1.0 if (not running or self.e_mode1 or self.e_idle_leg) else max(0.0, min(1.0, self.P("e02_fix_k")))
        bx, by, bth = p.x, p.y, (p.th() if p.known else float("nan"))
        if running and not self.e_active:
            result = "rejected:another run is going"
        elif self.e_finding:
            result = "rejected:finding (the boards set the pose)"
        elif running and self.millis() - self.e_last_move < self.P("e02_fix_still_ms"):
            result = "rejected:moving"
        elif not p.known:
            if isnan(th):
                result = "rejected:the first fix needs a heading"
            else:
                p.set(x, y, th, 0.5, 1.0)
                result = "set"
        else:
            nth = p.th() if isnan(th) else wrap180(p.th() + k * wrap180(th - p.th()))
            p.set(p.x + k * (x - p.x), p.y + k * (y - p.y), nth, 0.5, 1.0)
            result = "replaced" if k >= 1.0 else "blended"
        if result.startswith("rejected"):
            self.e_fixes_rejected += 1
        else:
            self.e_fixes_used += 1
            if k >= 1.0 or result == "set":
                self.e_since_set = 0.0
                self.e_back_since = 0.0
        self.send("D,FIX,%s,%d,%s,%.2f,%.2f,%.2f,%.2f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f" % (
            self.run_id if running else "-", self.millis(), tok[3], x, y, th, k, result, bx, by, bth, p.x, p.y,
            p.th() if p.known else float("nan")))

    # ------------------------------------------------------------------ the run
    def run_e02(self):
        P, p = self.P, self.pose
        self.e02_header()
        route = int(P("e02_route"))
        mode = 1 if P("e02_mode") >= 0.5 else 0
        self.e_start, self.e_far, self.e_end = Mark(), Mark(), Mark()
        if route < 1 or route > 3:
            self.fail = "e02_route must be 1, 2 or 3"
            self.e02_record(route, mode, "failed:" + self.fail, 0)
            return
        if self.e_edge_off():
            self.fail = self.e_edge_off()
            self.e02_record(route, mode, "failed:" + self.fail, 0)
            return
        if not p.known:
            self.fail = "position unknown: run find first (or send a camera fix)"
            self.e02_record(route, mode, "failed:" + self.fail, 0)
            return
        self.e_begin()
        self.e_mode1 = mode == 1
        self.front_limit = 0.0
        legs = 0
        ok = False
        if not self.e_mode1:
            while True:
                sx, sy = P("e02_start_x"), P("e02_start_y")
                self.e_cap = P("e02_transit_max_cm") + 10.0
                if sx != UNSET and sy != UNSET:
                    d = math.hypot(sx - p.x, sy - p.y)
                    if d > P("e02_transit_max_cm"):
                        self.fail = "start spot %.0f cm away (more than e02_transit_max_cm)" % d
                        break
                    self.step = 0
                    if not self.e_leg_to(sx, sy):
                        break
                hs = P("e02_route_deg")
                if hs == UNSET:
                    hs = wrap180(self.e_board(False).toward + 180.0)
                if not self.e_turn_to(hs):
                    break
                self.e_route_corrections()
                self.e_cap = self.e_travel + self.e_route_len(route) + P("e02_margin_cm")
                self.e_mark(self.e_start)
                w, far = self.e_waypoints(route)
                s = self.e_start
                c, sn = math.cos(s.th * DEG2RAD), math.sin(s.th * DEG2RAD)
                good = True
                for i, (wx, wy) in enumerate(w):
                    if not good:
                        break
                    self.step = i + 1
                    fx, fy = s.x + c * wx - sn * wy, s.y + sn * wx + c * wy
                    good = self.e_leg_to(fx, fy)
                    if good and (i == far or i == len(w) - 1):
                        good = self.e_redo(fx, fy)
                    legs += 1
                    if good and i == far:
                        self.e_mark(self.e_far)
                        good = self.wait_ms(P("e02_pause_ms"))
                if not good:
                    break
                self.step = len(w) + 1
                if not self.e_turn_to(s.th):
                    break
                self.e_mark(self.e_end)
                ok = True
                break
        else:
            self.leg_header()
            self.e_q = []
            self.e_last_id = 0
            self.e_end_marked = False
            self.front_soft = True
            self.edge_soft = True
            self.e_cap = self.e_route_len(route) + P("e02_transit_max_cm") + P("e02_margin_cm")
            self.e_mark(self.e_start)
            self.send("M,%s e02: camera-steered, waiting for go / turn / mark" % self.name)
            while True:
                if self.should_abort():
                    if self.abort_reason:
                        break
                    self.step_stop = ""
                if self.millis() - self.last_rx > P("e02_quiet_ms") and not self.e_q:
                    self.fail = "nothing from the laptop for %.0f s" % (P("e02_quiet_ms") / 1000.0)
                    break
                if not self.e_q:
                    self.loop_delay(5)
                    continue
                kind, arg, lid = self.e_q.pop(0)
                legs += 1
                self.step = legs
                self.step_stop = ""
                self.fail = ""
                self.leg_record(lid, kind, arg, "started", "")
                good, info = self.e_run_leg(kind, arg)
                self.brake()
                if self.abort_reason:
                    self.leg_record(lid, kind, arg, "failed", self.abort_reason)
                    break
                if good:
                    self.leg_record(lid, kind, arg, "done", self.add_front(info))
                else:
                    if self.step_stop == "edge":
                        why = "plywood edge (%s)" % getattr(self, "edge_last_cue", "")
                    else:
                        why = self.step_stop or self.fail or "failed"
                    self.leg_record(lid, kind, arg, "failed", why)
                    self.step_stop = ""
                    self.fail = ""
                if self.e_end_marked:
                    ok = True
                    break
            if not self.e_end.set:
                self.e_mark(self.e_end)
        if not ok and not self.e_end.set:
            self.e_mark(self.e_end)
        if ok:
            result = "done"
        elif self.abort_reason == "plywood edge ahead":
            result = "limited:plywood edge"
        elif self.abort_reason:
            result = "stopped:" + self.abort_reason
        else:
            result = "failed:" + (self.fail or "?")
        self.e02_record(route, mode, result, legs)
        self.e_routes.append(dict(run=self.run_id, route=route, mode=mode, result=result,
                                  start=self.e_start, far=self.e_far, end=self.e_end,
                                  far_goal=self.e_far_goal(route)))
        ex, ey = self.e_end.x - self.e_start.x, self.e_end.y - self.e_start.y
        self.send("M,%s e02 route %d %s: by its own reckoning it ended %.1f cm from its start, heading %.1f deg off"
                  % (self.name, route, "done" if ok else result, math.hypot(ex, ey),
                     wrap180(self.e_end.th - self.e_start.th)))
        self.e_end_run()
