"""The home dock and the touch probe in the simulator: the dock's geometry on the
floor, the robot's body as a solid (contact model), the two nose switches, the
dock's floor stripe under the line sensors, and a copy of the firmware's
switches.h and dock.h (robot.py mixes DockMixin in).

Contact model (position-based): after every physics step, any part of the
robot's outline (bumper face, chamfered corners, sides, rear) that has entered
a solid is pushed back out along the solid's surface normal. The push is
shared between rolling forward/back (easy), turning about the axle (medium,
config dock.turn_mobility per cm^2 of lever arm) and sliding sideways on the
tyres (hard, dock.lat_mobility), so a front corner meeting a funnel wall mostly
turns the robot and pushes it back, as rubber tyres do, and a robot pushing
into a flat wall stops while its wheels slip (its encoders keep counting, as
on the real robot). Solids: each dock's walls (convex pieces), and the boards
(world_polys) when dock.contact_boards is set (touch probe tests).

Switches: each lever's contact point sits on the bumper's front face,
bump_half_cm left / right of the centre line. It is closed while the gap from
there straight ahead to a solid is at most sw_gap_cm (opens again beyond
sw_gap_cm + sw_hyst_cm); for sw_bounce_ms after each change the raw reading
chatters at random. The firmware copy below debounces it exactly as
switches.h does.
"""
import math

from fw_follow import wrap180, isnan, DRIVE_OK

DEG2RAD = math.pi / 180.0
NONE = -1.0


# ------------------------------------------------------------------ geometry
def dock_to_floor(dock, px, py):
    """A dock-frame point (X into the dock, 0 at the back wall's inner face; Y left) on the floor."""
    x0, y0, h = dock[0], dock[1], dock[2] * DEG2RAD
    c, s = math.cos(h), math.sin(h)
    return (x0 + c * px - s * py, y0 + s * px + c * py)


def floor_to_dock(dock, fx, fy):
    x0, y0, h = dock[0], dock[1], dock[2] * DEG2RAD
    c, s = math.cos(h), math.sin(h)
    dx, dy = fx - x0, fy - y0
    return (c * dx + s * dy, -s * dx + c * dy)


def dock_solids_local(m, seat=(0.0, 0.0)):
    """The dock's walls as convex polygons (counter-clockwise), dock frame, cm. seat: (shift cm, tilt deg) of
    the back wall's effective face this time (simulation only: dust, print ripples)."""
    hw, t = m.pocket_w_cm / 2.0, m.wall_t_cm
    xp = -m.pocket_depth_cm                      # pocket starts (funnel ends)
    xm = xp - m.funnel_len_cm                    # mouth
    fl = m.funnel_flare_cm
    k = math.tan(seat[1] * DEG2RAD)
    out = [[(seat[0] + k * (hw + t), -hw - t), (m.back_depth_cm, -hw - t), (m.back_depth_cm, hw + t),
            (seat[0] - k * (hw + t), hw + t)]]  # back block
    for sg in (1.0, -1.0):
        wall = [(xp, sg * hw), (0.0, sg * hw), (0.0, sg * (hw + t)), (xp, sg * (hw + t))]           # pocket side
        # funnel side: inner face from (xm, hw + fl) to (xp, hw), thickness t outwards (along Y)
        fun = [(xm, sg * (hw + fl)), (xp, sg * hw), (xp, sg * (hw + t)), (xm, sg * (hw + fl + t))]
        for poly in (wall, fun):
            if sg < 0:
                poly = poly[::-1]
            out.append(poly)
    return out


def dock_solids(dock, m, seat=(0.0, 0.0)):
    return [[dock_to_floor(dock, x, y) for x, y in poly] for poly in dock_solids_local(m, seat)]


def funnel_half_width(m, x):
    """Inner half-width of the dock at dock X (between the walls); None outside the dock."""
    hw = m.pocket_w_cm / 2.0
    xp = -m.pocket_depth_cm
    xm = xp - m.funnel_len_cm
    if xp <= x <= 0:
        return hw
    if xm <= x < xp:
        return hw + m.funnel_flare_cm * (xp - x) / m.funnel_len_cm
    return None


def bracket_fitted(rb):
    """Is the nose bracket on? It follows the robot's own settings, as the real robots are set up: bumper_cm > 0
    (the bracket's stick-out) or sw_on (its switches, which only exist on the bracket). Firmware default: both 0,
    no bracket (Oct 2026: the switches have not arrived, the robots run bare)."""
    P = getattr(rb, "params", None) or {}
    return P.get("bumper_cm", 0.0) > 0 or P.get("sw_on", 0.0) > 0


def robot_outline(rb, bracket=None):
    """The robot's outline (body frame: x forward from the axle, y left), counter-clockwise. With the nose bracket
    (bracket_fitted): its face bump_ahead_cm ahead of the axle; the bracket (bumper_w_cm wide, bumper_len_cm from
    its face back to its arms' ends) is wider than the robot; the outline must stay convex, so behind the arms it
    narrows in a straight line to the robot's rear corners (slightly wider than the real robot there: on the safe
    side). Without it: the bare 9.5 x 9.5 cm body (front face length/2 ahead of the body centre)."""
    m, dm = rb.m, rb.cfg.dock
    if bracket is None:
        bracket = bracket_fitted(rb)
    hw = m.width_cm / 2.0
    rear = m.length_cm / 2.0 - m.axle_to_center_cm
    if not bracket:
        fa, ch = m.length_cm / 2.0 + m.axle_to_center_cm, dm.chamfer_cm
        return [(-rear, -hw), (fa - ch, -hw), (fa, -hw + ch), (fa, hw - ch), (fa - ch, hw), (-rear, hw)]
    bw = max(hw, dm.bumper_w_cm / 2.0)
    fa, ch = dm.bump_ahead_cm, dm.chamfer_cm
    xa = max(-rear + 0.1, fa - dm.bumper_len_cm)          # the arms' back end
    if bw <= hw + 1e-9:
        return [(-rear, -hw), (fa - ch, -hw), (fa, -hw + ch), (fa, hw - ch), (fa - ch, hw), (-rear, hw)]
    return [(-rear, -hw), (xa, -bw), (fa - ch, -bw), (fa, -bw + ch), (fa, bw - ch), (fa - ch, bw), (xa, bw),
            (-rear, hw)]


def body_points(rb):
    """Sample points of the outline the contact model keeps out of solids (body frame)."""
    out = robot_outline(rb)
    pts = []
    for i in range(len(out)):
        a, b = out[i], out[(i + 1) % len(out)]
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(L / 1.0)))
        for k in range(n):
            pts.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
    return pts


def edges_of(poly):
    """[(ax, ay, nx, ny)] per side of a convex CCW polygon, with its outward unit normal."""
    out = []
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        L = math.hypot(ex, ey)
        if L >= 1e-12:
            out.append((ax, ay, ey / L, -ex / L))
    return out


def _inside_e(edges, px, py):
    best = None
    for ax, ay, nx, ny in edges:
        d = (px - ax) * nx + (py - ay) * ny      # > 0 outside this side
        if d >= 0:
            return None
        if best is None or -d < best[0]:
            best = (-d, nx, ny)
    return best


def _inside(poly, px, py):
    """(depth, nx, ny) if the point is inside the convex CCW polygon: how far, and the outward normal of the
    nearest side; else None."""
    return _inside_e(edges_of(poly), px, py)


def _push(rb, rx, ry, nx, ny, depth):
    """Move the body so that its point at body offset (rx, ry) moves `depth` along (nx, ny)."""
    dm = rb.cfg.dock
    c, s = math.cos(rb.th), math.sin(rb.th)
    wx, wy = c * rx - s * ry, s * rx + c * ry
    jt = nx * (-wy) + ny * wx                    # d(point . n) / d(heading)
    nu = nx * c + ny * s
    nv = -nx * s + ny * c
    mf, ml, mt = 1.0, dm.lat_mobility, dm.turn_mobility
    w = mf * nu * nu + ml * nv * nv + mt * jt * jt
    if w < 1e-12:
        return
    lam = depth / w
    rb.x += lam * (mf * nu * c - ml * nv * s)
    rb.y += lam * (mf * nu * s + ml * nv * c)
    rb.th += lam * mt * jt


def resolve_contacts(rb):
    """Push the robot out of every solid it has entered. Returns True if anything touched."""
    solids = rb.solids
    if not solids:
        return False
    rb.contact_refresh()
    reach = rb.contact_reach
    near = []
    for poly, (cx, cy, r), bb, ed in solids:
        if math.hypot(rb.x - cx, rb.y - cy) <= r + reach:
            near.append((poly, bb, ed))
    if not near:
        return False
    touched = False
    pts = rb.contact_pts
    outline = rb.contact_out_edges
    for _ in range(10):
        worst = 0.0
        c, s = math.cos(rb.th), math.sin(rb.th)
        for poly, (x0, y0, x1, y1), ed in near:
            for rx, ry in pts:
                px, py = rb.x + c * rx - s * ry, rb.y + s * rx + c * ry
                if px < x0 or px > x1 or py < y0 or py > y1:
                    continue
                hit = _inside_e(ed, px, py)
                if hit and hit[0] > 1e-6:
                    _push(rb, rx, ry, hit[1], hit[2], hit[0])
                    c, s = math.cos(rb.th), math.sin(rb.th)
                    worst = max(worst, hit[0])
            for vx, vy in poly:                  # a wall's corner poking into the robot's side
                dx, dy = vx - rb.x, vy - rb.y
                if dx * dx + dy * dy > reach * reach:
                    continue
                bx, by = c * dx + s * dy, -s * dx + c * dy
                hit = _inside_e(outline, bx, by)
                if hit and hit[0] > 1e-6:
                    # the robot's point there must move against the robot side's outward normal
                    mx, my = c * hit[1] - s * hit[2], s * hit[1] + c * hit[2]
                    _push(rb, bx, by, -mx, -my, hit[0])
                    c, s = math.cos(rb.th), math.sin(rb.th)
                    worst = max(worst, hit[0])
        if worst <= 1e-6:
            break
        touched = True
    return touched


def ray_solids(rb, px, py, dx, dy, best):
    import robot as R
    return R.ray_polys(px, py, dx, dy, [p[0] for p in rb.solids], best)


# ------------------------------------------------------------------ the mixin
class DockMixin:
    """Mixed into robot.Robot: the dock world, the switches (switches.h) and dock.h."""

    def dock_init(self):
        dm = self.cfg.dock
        self.solids = []                         # [(polygon, (cx, cy, radius))]
        self.dock_list = [list(d) for d in dm.docks]
        polys = []
        for d in self.dock_list:
            polys += dock_solids(d, dm)
        self.dock_polys = polys
        self.contact_refresh()
        self._solids_from = None
        self.update_solids()
        # switches.h
        self.sw_on = False
        self.sw_raw = [False, False]
        self.sw_st = [False, False]
        self.sw_since = [0, 0]
        self.sw_first = [0, 0]
        self.sw_press_ms = [0, 0]
        self.sw_true = [False, False]            # simulation: the contact itself, before bounce
        self.sw_change = [-1e9, -1e9]
        self.edge_line_off = False
        self.docks_done = []                     # simulation only: true pose at every docking / touch
        self.touches = []

    def contact_refresh(self):
        """(Re)build the contact body when the bracket setting changed (`set bumper_cm` / `set sw_on`)."""
        b = bracket_fitted(self)
        if getattr(self, "_contact_bracket", None) == b:
            return
        self._contact_bracket = b
        self.contact_outline = robot_outline(self, b)
        self.contact_pts = body_points(self)
        self.contact_out_edges = edges_of(self.contact_outline)
        self.contact_reach = max(math.hypot(x, y) for x, y in self.contact_outline) + 0.5

    def redraw_seat(self):
        dm = self.cfg.dock
        if not self.dock_list or (dm.seat_sd_cm <= 0 and dm.seat_sd_deg <= 0):
            return
        polys = []
        for d in self.dock_list:
            polys += dock_solids(d, dm, (self.sw_rng.gauss(0, dm.seat_sd_cm), self.sw_rng.gauss(0, dm.seat_sd_deg)))
        self.dock_polys = polys
        self.update_solids()

    def update_solids(self):
        """Rebuild the solid list (call after changing world_polys or the docks)."""
        polys = list(self.dock_polys)
        if self.cfg.dock.contact_boards:
            polys += [list(map(tuple, p)) for p in self.world_polys]
        out = []
        for p in polys:
            cx = sum(q[0] for q in p) / len(p)
            cy = sum(q[1] for q in p) / len(p)
            bb = (min(q[0] for q in p), min(q[1] for q in p), max(q[0] for q in p), max(q[1] for q in p))
            out.append((p, (cx, cy, max(math.hypot(q[0] - cx, q[1] - cy) for q in p)), bb, edges_of(p)))
        self.solids = out
        self._solids_from = (len(self.world_polys), self.cfg.dock.contact_boards)

    # ---- the true switch contact and the stripe ----
    def lever_gap(self, k):
        """Gap (cm) from lever k's contact point on the bumper face straight ahead to a solid."""
        dm = self.cfg.dock
        if self._solids_from != (len(self.world_polys), dm.contact_boards):
            self.update_solids()
        if not self.solids:
            return float("inf")
        lat = dm.bump_half_cm if k == 0 else -dm.bump_half_cm
        c, s = math.cos(self.th), math.sin(self.th)
        px = self.x + c * dm.bump_ahead_cm - s * lat
        py = self.y + s * dm.bump_ahead_cm + c * lat
        # start a hair behind the face so a face already touching still reads ~0
        return ray_solids(self, px - c * 0.05, py - s * 0.05, c, s, 50.0) - 0.05

    def sw_read_raw(self, k):
        dm = self.cfg.dock
        if self.lifted:
            closed = False
        else:
            g = self.lever_gap(k)
            closed = g <= dm.sw_gap_cm if not self.sw_true[k] else g <= dm.sw_gap_cm + dm.sw_hyst_cm
        if closed != self.sw_true[k]:
            self.sw_true[k] = closed
            self.sw_change[k] = self.ms
        if self.ms - self.sw_change[k] < dm.sw_bounce_ms:
            return self.sw_rng.random() < 0.5
        return closed

    def stripe_cover(self, px, py, footprint):
        """How much of a line sensor's footprint at floor point (px, py) is on a dock's floor stripe (0..1)."""
        dm = self.cfg.dock
        best = 0.0
        for d in self.dock_list:
            x, y = floor_to_dock(d, px, py)
            hw = funnel_half_width(dm, x)
            if hw is None:
                hw = funnel_half_width(dm, min(max(x, dm.stripe_x0_cm), dm.stripe_x1_cm))
            if hw is None or abs(y) > hw:
                continue
            a, b = x - footprint / 2, x + footprint / 2
            ov = max(0.0, min(b, dm.stripe_x1_cm) - max(a, dm.stripe_x0_cm))
            best = max(best, min(1.0, ov / footprint))
        return best

    # ---- switches.h ----
    def sw_configure(self):
        P = self.P
        want = P("sw_on") > 0
        l, r = int(P("sw_left_pin")), int(P("sw_right_pin"))
        if not want or l < 0 or r < 0 or l == r:
            self.sw_on = False
            return
        self.sw_on = True
        now = self.millis()
        for k in range(2):
            v = self.sw_read_raw(k)
            self.sw_raw[k] = self.sw_st[k] = v
            self.sw_since[k] = now
            self.sw_first[k] = self.sw_press_ms[k] = now if v else 0

    def sw_tick(self):
        if not self.sw_on:
            return
        now = self.millis()
        deb = max(0.0, self.P("sw_debounce_ms"))
        for k in range(2):
            v = self.sw_read_raw(k)
            if v != self.sw_raw[k]:
                self.sw_raw[k] = v
                self.sw_since[k] = now
                if v and not self.sw_st[k] and not self.sw_first[k]:
                    self.sw_first[k] = now | 1
            if self.sw_raw[k] != self.sw_st[k] and now - self.sw_since[k] >= deb:
                self.sw_st[k] = self.sw_raw[k]
                if self.sw_st[k]:
                    self.sw_press_ms[k] = self.sw_first[k] or now
                else:
                    self.sw_press_ms[k] = 0
                    self.sw_first[k] = 0
            if not self.sw_raw[k] and not self.sw_st[k] and now - self.sw_since[k] >= deb:
                self.sw_first[k] = 0

    def sw_left(self):
        return self.sw_on and self.sw_st[0]

    def sw_right(self):
        return self.sw_on and self.sw_st[1]

    def sw_col(self, k):
        return (1 if self.sw_st[k] else 0) if self.sw_on else -1

    # ---- dock.h ----
    def own_pose(self):
        p = self.pose
        if p.on and p.known:
            return p.x, p.y, p.th()
        return self.odo_x, self.odo_y, self.fw_odo_sign * self.odo_th

    def _median(self, v):
        v = sorted(v)
        n = len(v)
        return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])

    def dock_creep(self, dock_mode, max_cm=-1.0):
        """dock.h creep(). Returns (ok, result dict)."""
        P = self.P
        r = dict(ok=False, why="", ms_start=0, ms_first=0, ms_both=0, ms_press=[0, 0], travel=0.0,
                 travel_at=[NONE, NONE], first=-1, pivot=0.0, stripe=[NONE, NONE, NONE], x=0.0, y=0.0, th=0.0,
                 x0=0.0, y0=0.0, th0=0.0, front0=0.0, funnel=0.0, twist_at=NONE)
        self.redraw_seat()                       # simulation only: this docking's wall imperfection
        if not self.sw_on:
            r["why"] = "nose switches not configured (sw_on 0)"
            self.fail = r["why"]
            return False, r
        self.sw_tick()
        if self.sw_raw[0] or self.sw_raw[1]:
            r["why"] = "%s switch pressed before the start" % ("left" if self.sw_raw[0] else "right")
            self.fail = r["why"]
            return False, r
        saved = self.front_limit
        self.front_limit = 0.0
        level = [0.0, 0.0, 0.0]
        stripe_on = dock_mode and P("stripe_on") > 0.5     # dock version 1's stripe; version 2 has none
        if stripe_on:
            buf = [[], [], []]
            t0 = self.millis()
            while len(buf[0]) < 24 and self.millis() - t0 < 300:
                if self.should_abort():
                    self.front_limit = saved
                    return False, r
                for k, v in enumerate(self.line_sensors()):
                    buf[k].append(v)
                self.loop_delay(5)
            level = [self._median(b) for b in buf]
        self.edge_line_off = dock_mode
        lx0, ly0 = self.odo_x, self.odo_y
        r["x0"], r["y0"], r["th0"] = self.own_pose()
        r["front0"] = self.tof()[2]
        creep, press, pivot = P("dock_rpm"), P("dock_press_rpm"), P("dock_pivot_rpm")
        if max_cm <= 0:
            max_cm = P("dock_max_cm")
        sthr, sdir = P("stripe_thr"), P("stripe_dir")
        pivot_ms = P("dock_pivot_ms")
        limit = 5000 + max_cm / max(0.2, creep * 0.178) * 2000.0
        r["ms_start"] = self.millis()
        h_first = 0.0
        h_start, twist_deg = self.heading(), P("dock_twist_deg")
        ok = False
        sn = [0, 0, 0]
        while True:
            if self.should_abort():
                break
            now = self.millis()
            travel = math.hypot(self.odo_x - lx0, self.odo_y - ly0)
            r["travel"] = travel
            for k in range(2):
                if self.sw_first[k] and r["travel_at"][k] == NONE:
                    r["travel_at"][k] = travel
                    r["ms_press"][k] = self.sw_first[k]
            if r["first"] < 0 and r["twist_at"] == NONE and abs(wrap180(self.heading() - h_start)) > twist_deg:
                r["twist_at"] = travel
            if stripe_on:
                vals = self.line_sensors()
                for k in range(3):
                    if r["stripe"][k] != NONE:
                        continue
                    d = vals[k] - level[k]
                    sv = d if sdir > 0 else -d if sdir < 0 else abs(d)
                    if sv >= sthr:
                        sn[k] += 1
                        if sn[k] >= 2:
                            r["stripe"][k] = travel
                    else:
                        sn[k] = 0
            L, R = self.sw_left(), self.sw_right()
            if (L or R) and r["first"] < 0:
                r["first"] = 0 if L else 1
                r["ms_first"] = now
                h_first = self.heading()
                r["funnel"] = wrap180(h_first - h_start)
                if not dock_mode:
                    self.brake()
                    r["ms_both"] = self.sw_press_ms[r["first"]]
                    ok = True
                    break
            if dock_mode and L and R:
                self.brake()
                r["ms_both"] = now
                ok = True
                break
            if travel > max_cm:
                r["why"] = "no click within %.0f cm" % max_cm
                break
            if r["first"] >= 0 and now - r["ms_first"] > pivot_ms:
                r["why"] = "%s switch never clicked" % ("right" if r["first"] == 0 else "left")
                break
            if now - r["ms_start"] > limit:
                r["why"] = "creep timed out (stalled?)"
                break
            if not L and not R:
                self.set_wheels_speed(creep, creep)
            elif L:
                self.set_wheels_speed(press, pivot)
            else:
                self.set_wheels_speed(pivot, press)
            self.loop_delay(5)
        self.brake()
        if ok and dock_mode:
            r["pivot"] = wrap180(self.heading() - h_first)
            t0 = self.millis()
            self.set_wheels_speed(press, press)
            while self.millis() - t0 < P("dock_seat_ms"):
                if self.should_abort():
                    ok = False
                    break
                self.loop_delay(5)
            self.brake()
            if ok and not self.wait_ms(200):
                ok = False
            if ok and not (self.sw_left() and self.sw_right()):
                r["why"] = "lost contact after seating (L %d R %d)" % (self.sw_left(), self.sw_right())
                ok = False
        self.edge_line_off = False
        self.front_limit = saved
        r["x"], r["y"], r["th"] = self.own_pose()
        if not ok and not self.abort_reason and not self.step_stop and r["why"]:
            self.fail = r["why"]
        r["ok"] = ok
        return ok, r

    def dock_hold(self):
        ms = self.P("dock_still_ms")
        if ms == 0:
            return True, 0.0, 0.0, 0
        h0, total, n = self.heading(), 0.0, 0
        t0 = self.millis()
        while self.millis() - t0 < ms:
            if self.should_abort():
                return False, 0.0, 0.0, 0
            total += self._gz()
            n += 1
            self.loop_delay(10)
        still = self.millis() - t0
        drift = wrap180(self.heading() - h0) / (still / 1000.0) if self.fw_yaw_sign else 0.0
        return True, drift, (total / n if n else 0.0), still

    def _gz(self):
        im = self.imu_m
        return im.yaw_sign * (math.degrees(self.w) * (1 + self.gyro_scale) + self.gyro_bias) + \
            self.rng.gauss(0, im.gyro_noise_dps)

    def dock_header(self):
        self.send("H,DOCK,run,ms_start,ms_first,ms_left,ms_right,ms_both,first,result,travel_cm,travel_left_cm,"
                  "travel_right_cm,pivot_deg,stripe_l_cm,stripe_c_cm,stripe_r_cm,frame,x0_cm,y0_cm,th0_deg,x_cm,y_cm,"
                  "th_deg,still_ms,drift_dps,gz_mean,drift_used,n,funnel_deg,twist_at_cm")

    def dock_record(self, r, still, drift, gz, used, n):
        p = self.pose
        self.send("D,DOCK,%s,%d,%d,%d,%d,%d,%s,%s,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,"
                  "%.2f,%d,%.4f,%.4f,%d,%d,%.2f,%.3f" % (
                      self.run_id, r["ms_start"], r["ms_first"], r["ms_press"][0], r["ms_press"][1], r["ms_both"],
                      "L" if r["first"] == 0 else "R" if r["first"] == 1 else "-",
                      "docked" if r["ok"] else r["why"], r["travel"], r["travel_at"][0], r["travel_at"][1], r["pivot"],
                      r["stripe"][0], r["stripe"][1], r["stripe"][2], "floor" if (p.on and p.known) else "odo",
                      r["x0"], r["y0"], r["th0"], r["x"], r["y"], r["th"], still, drift, gz, used, n,
                      r.get("funnel", 0.0), r.get("twist_at", NONE)))

    def touch_header(self):
        self.send("H,TOUCH,run,ms_start,ms_click,side,result,travel_cm,travel_left_cm,travel_right_cm,frame,x0_cm,"
                  "y0_cm,th0_deg,x_cm,y_cm,th_deg,front0_cm,n")

    def touch_record(self, r, n):
        p = self.pose
        side = "-" if r["first"] < 0 else "LR" if (self.sw_left() and self.sw_right()) else \
            "L" if r["first"] == 0 else "R"
        trav = r["travel_at"][r["first"]] if r["first"] >= 0 else r["travel"]
        self.send("D,TOUCH,%s,%d,%d,%s,%s,%.3f,%.3f,%.3f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d" % (
            self.run_id, r["ms_start"], r["ms_both"], side, "touched" if r["ok"] else r["why"], trav,
            r["travel_at"][0], r["travel_at"][1], "floor" if (p.on and p.known) else "odo", r["x0"], r["y0"], r["th0"],
            r["x"], r["y"], r["th"], r["front0"], n))
        self.touches.append(dict(run=self.run_id, n=n, ok=r["ok"], travel=trav, first=r["first"],
                                 true=(self.x, self.y, self.th)))

    def dock_once(self, n, wall_x, wall_y, h, max_cm=-1.0):
        ok, r = self.dock_creep(True, max_cm)
        drift = gz = 0.0
        still = 0
        used = 0
        p = self.pose
        if ok:
            self.docks_done.append(dict(run=self.run_id, n=n, ms=self.ms, true=(self.x, self.y, self.th), r=r))
            if p.on and not (isnan(wall_x) or isnan(wall_y) or isnan(h)):
                a = self.P("bump_ahead_cm")
                p.set(wall_x - a * math.cos(h * DEG2RAD), wall_y - a * math.sin(h * DEG2RAD), h, 0.05, 0.3)
            ok, drift, gz, still = self.dock_hold()
            if ok and self.fw_yaw_sign and abs(drift) <= self.P("dock_drift_max_dps"):
                used = 1
                if p.on:
                    p.drift_dps = drift
                if p.on and not isnan(h):
                    p.set(p.x, p.y, h, 0.05, 0.3)
        self.dock_record(r, still, drift, gz, used, n)
        return ok, r

    def run_dock(self):
        self.dock_header()
        n = max(1, int(self.P("dock_n")))
        back = self.P("dock_back_cm")
        if back <= 0:
            n = 1
        self.front_limit = 0.0                   # the whole run faces the dock's wall
        for i in range(1, n + 1):
            self.step = i
            ok, _ = self.dock_once(i, float("nan"), float("nan"), float("nan"))
            if not ok:
                return
            if back > 0 and not (self.library_move(-back) and self.wait_ms(500)):
                return
        self.send("M,%s dock: %d docking%s done" % (self.name, n, "" if n == 1 else "s"))

    def run_touch(self):
        self.touch_header()
        n = max(1, int(self.P("touch_n")))
        back = self.P("touch_back_cm")
        self.front_limit = 0.0                   # the whole run faces the board it touches
        for i in range(1, n + 1):
            self.step = i
            ok, r = self.dock_creep(False)
            self.touch_record(r, i)
            if not ok:
                return
            if not self.wait_ms(300):
                return
            if back > 0 and not (self.library_move(-back) and self.wait_ms(500)):
                return

    # ---- follow steps K and P (follow.h runStep) ----
    def run_dock_step(self, s):
        self.front_limit = 0.0
        if abs(wrap180(s["h"] - self.pose.th())) > 3.0 and not self.turn_to_floor(s["h"]):
            return False
        if s["type"] == "K":
            if not self.f_dock_hdr:
                self.dock_header()
                self.f_dock_hdr = True
            ok, r = self.dock_once(s["id"], s["x1"], s["y1"], s["h"], s["max_travel"])
        else:
            if not self.f_touch_hdr:
                self.touch_header()
                self.f_touch_hdr = True
            ok, r = self.dock_creep(False, s["max_travel"])
            self.touch_record(r, s["id"])
        self.f_travel = r["travel"]
        if not ok:
            return False
        if s["type"] == "K":
            self.f_info = "docked; pivot %.1f deg; stripe %.2f cm" % (r["pivot"], r["stripe"][1])
        else:
            self.f_info = "touch %s %.3f cm" % ("L" if r["first"] == 0 else "R", r["travel_at"][r["first"]])
        if s["back"] > 0:
            if not self.wait_ms(300):
                return False
            p = self.pose
            ux, uy = math.cos(s["h"] * DEG2RAD), math.sin(s["h"] * DEG2RAD)
            if self.drive_line(p.x, p.y, p.x - s["back"] * ux, p.y - s["back"] * uy, s["h"], 3.0, True) != DRIVE_OK:
                return False
        return True
