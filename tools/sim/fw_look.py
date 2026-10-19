"""Copy of firmware look.h and runCalib() (home_lab.ino, 2026-10-18c): the calibration run looks
around while it spins, then rolls where there is room. Kept line for line with the firmware so
the simulator's calibration runs (and their RAW logs and CALIB_LOOK records) match the robot's.
"""
import math

NB = 120                 # directions, 3 deg apart; bin 0 = straight ahead
BIN = 3.0
ZONE_HALF = 5.5
SEEN_HALF = 24.0
NOTHING = 999.0
ZONE_DEG = (22.0, 11.0, 0.0, -11.0, -22.0)
MAXCAP = 8
CAP_HALF = 22.5
DEG2RAD = math.pi / 180.0


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def bin_of(rel):
    a = math.fmod(rel, 360.0)
    if a < 0:
        a += 360.0
    return int(math.floor(a / BIN + 0.5)) % NB


class Look:
    """look.h: the map of what the front sensor saw around the turning centre, and the choice."""

    def __init__(self, rb):
        self.rb = rb
        self.min_r = [0.0] * NB
        self.seen = [False] * NB
        self.h0 = 0.0
        self.active = False
        self.near_now = 0.0
        self.caps = []           # [(deg, cm)], `set calib_dir_limits ...`

    def reset(self):
        self.min_r = [0.0] * NB
        self.seen = [False] * NB
        self.h0 = self.rb.heading()
        self.near_now = 0.0

    def add_point(self, rel, z, d):
        x = -ZONE_HALF
        while x <= ZONE_HALF + 0.01:
            self.seen[bin_of(rel + z + x)] = True
            x += BIN
        self.seen[bin_of(rel + z + ZONE_HALF)] = True
        if d <= 0 or d < self.rb.P("tof_min_cm"):
            return
        s = self.rb.P("tof_ahead_cm")
        zr = z * DEG2RAD
        px, py = s + d * math.cos(zr), d * math.sin(zr)
        r = math.hypot(px, py)
        b = bin_of(rel + math.degrees(math.atan2(py, px)))
        if self.min_r[b] == 0 or r < self.min_r[b]:
            self.min_r[b] = r
        if self.near_now == 0 or r < self.near_now:
            self.near_now = r

    def sample(self):
        rb = self.rb
        d = rb.tof()
        rel = wrap180(rb.heading() - self.h0)
        for k in range(5):
            self.add_point(rel, ZONE_DEG[k], d[k])
        self.add_point(rel, 0.0, rb.tof_top())

    def tick_hook(self):
        if self.active:
            self.sample()

    def room_at(self, h):
        half, s, best = self.rb.P("path_half_cm"), self.rb.P("tof_ahead_cm"), NOTHING
        for b in range(NB):
            r = self.min_r[b]
            if r <= 0:
                continue
            a = (b * BIN - h) * DEG2RAD
            fwd, lat = r * math.cos(a), r * math.sin(a)
            if fwd > 0 and abs(lat) <= half and fwd - s < best:
                best = fwd - s
        return best

    def seen_around(self, h):
        x = -SEEN_HALF
        while x <= SEEN_HALF + 0.01:
            if not self.seen[bin_of(h + x)]:
                return False
            x += BIN
        return True

    def cap_at(self, h):
        c = -1.0
        for deg, cm in self.caps:
            if abs(wrap180(h - deg)) <= CAP_HALF and (c < 0 or cm < c):
                c = cm
        return c

    def roll_at(self, h):
        if not self.seen_around(h):
            return -1.0
        P = self.rb.P
        r = min(P("calib_roll_cm"), self.room_at(h) - P("safe_front_cm") - P("calib_margin_cm") - P("bumper_cm"))
        c = self.cap_at(h)
        if c >= 0:
            r = min(r, c)
        return r

    def choose(self):
        """(heading, room, cap, roll, full): see look.h choose()."""
        want = self.rb.P("calib_roll_cm")
        full = [self.roll_at(b * BIN) >= want - 0.01 for b in range(NB)]
        n_full = sum(full)
        h = 0.0
        if 0 < n_full < NB:
            best_len, best_mid = 0, 0.0
            for b in range(NB):
                if not full[b] or full[(b + NB - 1) % NB]:
                    continue
                n = 0
                while n < NB and full[(b + n) % NB]:
                    n += 1
                mid = wrap180((b + (n - 1) / 2.0) * BIN)
                if n > best_len or (n == best_len and abs(mid) < abs(best_mid)):
                    best_len, best_mid = n, mid
            h = best_mid
        elif n_full == 0:
            best = -1e9
            for b in range(NB):
                hb = wrap180(b * BIN)
                r = self.roll_at(hb)
                if r > best + 0.01 or (abs(r - best) <= 0.01 and abs(hb) < abs(h)):
                    best, h = r, hb
        return h, self.room_at(h), self.cap_at(h), self.roll_at(h), n_full > 0

    def room_summary(self):
        return ";".join("%.1f" % (self.room_at(k * 45.0) if self.seen_around(k * 45.0) else -1.0) for k in range(8))

    def set_limits(self, rest):
        """`set calib_dir_limits <deg> <cm> [...]` / `set calib_dir_limits off`."""
        rest = rest.strip()
        self.caps = []
        if not rest or rest.lower() == "off":
            self.rb.send("M,calib_dir_limits off")
            return
        vals = [v for v in rest.replace(",", " ").replace(":", " ").split()]
        try:
            vals = [float(v) for v in vals]
        except ValueError:
            self.rb.send("M,calib_dir_limits: '%s' is not a number (none set)" % rest)
            return
        if len(vals) % 2 or len(vals) > 2 * MAXCAP:
            self.rb.send("M,usage: set calib_dir_limits <deg> <cm> [<deg> <cm> ...] (at most 8 pairs) or off")
            return
        self.caps = [(wrap180(vals[2 * k]), vals[2 * k + 1]) for k in range(len(vals) // 2)]
        self.rb.send("M,calib_dir_limits%s (deg:cm, relative to where the robot faces; next calibration only)"
                     % "".join(" %.0f:%.1f" % c for c in self.caps))

    def caps_text(self):
        return ";".join("%.0f:%.1f" % c for c in self.caps)


def calib_steps(rb, lk):
    """home_lab.ino calibSteps(). Returns nothing; rb.abort_reason / rb.fail say why it stopped."""
    P = rb.P
    spin = P("calib_spin_deg")
    rb.calib_spin = "done" if spin != 0 else "off"     # gCalibSpin / gCalibRoll / gCalibNoRoom
    rb.calib_roll = "skipped"
    rb.calib_no_room = rb.calib_edge = False
    still = int(max(300.0, P("calib_still_ms")))
    rb.send("M,%s calibration: blinking code %d, a small wiggle%s, then a roll of up to %.0f cm forward and back "
            "where it finds room" % (rb.name, rb.blink_code_for(), ", a slow spin each way" if spin != 0 else "",
                                     max(3.0, P("calib_roll_cm"))))
    rb.step = 1
    lk.reset()
    lk.active = True
    if not rb.wait_ms(int(P("calib_blink_ms"))):
        return
    near_start = lk.near_now
    lk.active = False
    rb.step = 2
    if not rb.calibrate_yaw_sign(True):
        return
    rb.step = 3
    lk.reset()
    lk.active = True
    if not rb.wait_ms(still):
        return
    nearest = lk.near_now
    if near_start > 0 and (nearest <= 0 or near_start < nearest):
        nearest = near_start
    spun = False
    spin_r = P("calib_spin_r_cm") + P("bumper_cm")          # a nose bumper's corners sweep further out
    if spin != 0 and 0 < nearest < spin_r:
        rb.calib_spin = "skipped"
        rb.calib_no_room = True
        rb.send("M,%s: something is %.1f cm from its middle, inside the %.0f cm circle it needs to spin: no spin "
                "(no gyro scale or light offset this time); it can only look straight ahead"
                % (rb.name, nearest, 2 * spin_r))
    elif spin != 0:
        rb.step = 4
        if not rb.spin_by(abs(spin), P("calib_spin_rpm")):
            return
        rb.step = 5
        if not rb.wait_ms(still):
            return
        rb.step = 6
        if not rb.spin_by(-abs(spin), P("calib_spin_rpm")):
            return
        rb.step = 7
        if not rb.wait_ms(still):
            return
        spun = True
    lk.active = False
    ch, room, cap, roll_len, full = lk.choose()
    roll_min = max(1.0, P("calib_roll_min_cm"))
    roll = roll_len >= roll_min
    st = dict(target=wrap180(lk.h0 + ch), turn=0.0, live=0.0, L=roll_len if roll else 0.0,
              result="no_room" if not roll else ("roll" if full else "short"))
    _turn_and_roll(rb, lk, st, ch, room, cap, roll, roll_min, still)
    reason = calib_reason(rb)
    rb.send("H,CALIB_LOOK,run,spun,near_cm,heading_deg,turn_deg,room_cm,cap_cm,live_cm,roll_cm,result,room8,caps,"
            "spin,roll,reason")
    rb.send("D,CALIB_LOOK,%s,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%s,%s,%s,%s,%s" % (
        rb.run_id, 1 if spun else 0, nearest, ch, st["turn"], room, cap, st["live"], st["L"], st["result"],
        lk.room_summary(), lk.caps_text(), rb.calib_spin, rb.calib_roll, reason))
    rb.calib_look = dict(spun=spun, near_cm=nearest, heading_deg=ch, turn_deg=st["turn"], room_cm=room, cap_cm=cap,
                         live_cm=st["live"], roll_cm=st["L"], result=st["result"], h0=lk.h0, spin=rb.calib_spin,
                         roll=rb.calib_roll, reason=reason)


def calib_reason(rb):
    """home_lab.ino calibReason()."""
    if rb.calib_no_room and rb.calib_edge:
        return "no room; plywood edge"
    return "no room" if rb.calib_no_room else ("plywood edge" if rb.calib_edge else "")


def _turn_and_roll(rb, lk, st, ch, room, cap, roll, roll_min, still):
    """The turnAndRoll lambda in calibSteps(): turn, look again, roll; fills st."""
    P = rb.P
    rb.step = 8
    if roll:
        st["turn"] = wrap180(st["target"] - rb.heading())
        if abs(st["turn"]) > P("turn_tol_deg") and not rb.imu_turn_to(st["target"]):
            return
    t0 = rb.millis()
    while rb.millis() - t0 < still:
        if rb.should_abort():
            return
        f = rb.front_in_path()
        if f > 0 and (st["live"] == 0 or f < st["live"]):
            st["live"] = f
        rb.loop_delay(5)
    if roll and st["live"] > 0:
        fit = st["live"] - P("safe_front_cm") - P("calib_margin_cm")
        if fit < st["L"]:
            st["L"] = fit
            st["result"] = "short" if fit >= roll_min else "no_room_after_turn"
            if fit < roll_min:
                st["L"] = 0.0
    L = st["L"]
    rb.calib_roll = "skipped" if L <= 0 else ("full" if st["result"] == "roll" else "short")
    if L <= 0:
        rb.calib_no_room = True
        rb.send("M,%s: no room to roll (best direction %.0f deg from where it faced: %.0f cm clear ahead%s, a %.0f cm "
                "roll needs %.0f): no heading or wheel scale this time. Give it 13 in (33 cm) of clear floor in some "
                "direction and calibrate again for those." % (
                    rb.name, ch, room, ", capped by the laptop" if cap >= 0 else "", roll_min,
                    roll_min + P("safe_front_cm") + P("calib_margin_cm")))
        return
    if abs(ch) > 2:
        rb.send("M,%s: rolling %.1f cm towards %.0f deg (left positive) from where it faced" % (rb.name, L, ch))
    rb.front_limit = -1.0
    rb.step = 9
    x0, y0 = rb.odo_x, rb.odo_y
    if not rb.library_move(L):
        if rb.abort_reason != "plywood edge ahead":
            return
        # the edge guard refused to roll on: not a failure; keep the roll if long enough, retrace it
        rb.abort_reason = ""
        rb.calib_edge = True
        planned = L
        rb.calib_roll, st["result"], st["L"] = "skipped", "edge", 0.0    # until it knows how far it went
        if not rb.wait_ms(still):
            return
        went = math.hypot(rb.odo_x - x0, rb.odo_y - y0)
        keep = went >= roll_min
        rb.send("M,%s: stopped at a plywood edge after %.1f cm of a %.1f cm roll; %s" % (
            rb.name, went, planned, "keeping that roll, rolling back" if keep else "too short to measure, rolling back"))
        rb.calib_roll = "edge" if keep else "skipped"
        st["L"] = went if keep else 0.0
        rb.step = 10
        if went >= 0.5 and (not rb.library_move(-went) or not rb.wait_ms(still)):
            return
        rb.front_limit = 0.0
        return
    if not rb.wait_ms(still):
        return
    rb.step = 10
    if not rb.library_move(-L) or not rb.wait_ms(still):
        return
    rb.front_limit = 0.0


def run_calib(rb):
    """home_lab.ino runCalib()."""
    lk = rb.look
    saved_hook = rb.tick_hook
    rb.front_limit = 0.0
    rb.tick_hook = lk.tick_hook
    rb.calib_look = None
    calib_steps(rb, lk)
    lk.active = False
    lk.caps = []
    rb.tick_hook = saved_hook
    rb.front_limit = -1.0              # back to the usual front stop for the next run
    if rb.abort_reason or rb.fail:
        return
    if rb.calib_no_room or rb.calib_edge:      # not a failure: it did what the space allowed (RUNEND done; "limited")
        if rb.calib_edge:
            r = " roll cut short at a plywood edge;" if rb.calib_roll == "edge" else \
                " no roll (stopped at a plywood edge too soon);"
        else:
            r = " no roll (no room in any direction);" if rb.calib_roll == "skipped" else ""
        rb.send("M,calibrated, limited (%s):%s%s the rest is kept. Now place the robot on its start mark" % (
            calib_reason(rb), " no spin (something inside its spin circle);" if rb.calib_spin == "skipped" else "", r))
    else:
        rb.send("M,calibrated: now place the robot on its start mark")
