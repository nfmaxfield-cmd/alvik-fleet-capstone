"""E02 firmware logic in the simulator (firmware e02.h, copied in tools/sim/fw_e02.py): bump detection without
switches, push-to-square, `run find`, `run e02` (own sensors and camera-steered), camera fixes.

    python -m pytest tools/sim/tests/test_e02.py -v          (about 1-2 minutes)

The contact, stall, jolt and seam numbers are GUESSES (config.py: dock.wall_stall_frac, session.seams*,
imu.accel_vibration_g raised here to the 0.05 g measured on the real robots while creeping). The gyro under-reads
turns by 7% here (session notes 2026-10-19 / 2026-10-21: camera 1.069-1.075). These tests check the firmware's
logic against that model, not the real robot.
"""
import math
import os
import random
import re
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import config  # noqa: E402
import robot  # noqa: E402

# the two blocked edges of the 122 x 122 cm plywood: faces along y = 0 (x board) and x = 0 (y board)
XBOARD = [[-3.0, -3.0], [125.0, -3.0], [125.0, 0.0], [-3.0, 0.0]]
YBOARD = [[-3.0, -3.0], [0.0, -3.0], [0.0, 125.0], [-3.0, 125.0]]
GYRO_UNDER = -0.07                  # the gyro counts 93% of every turn
GYRO_FIX = 1.0 / 0.93               # the corr.h gyro_scale that undoes it


def make(name="Left", seed=0, stall=0.15, vib=0.05, seams=(), boards=(XBOARD, YBOARD), **params):
    cfg = config.load_config()
    cfg.dock.contact_boards = True
    cfg.dock.wall_stall_frac = stall
    cfg.imu.accel_vibration_g = vib
    cfg.session.seams = [list(s) for s in seams]
    r = robot.Robot(name, cfg, random.Random(seed), config.firmware_params(), "test", 0.0, 0x2000 + seed, "ip")
    r.world_polys = [list(b) for b in boards]
    r.update_solids()
    r.idle_until(10.0)
    r.fw_yaw_sign = 1                       # compass already checked
    r.gyro_scale = GYRO_UNDER
    r.params.update(start_nudge_cm=0, bumper_cm=1.34, safe_edge=1, e02_xboard_cm=0, e02_yboard_cm=0, e02_xboard_side=0,
                    e02_yboard_side=0, settle_start_ms=200, settle_end_ms=200)
    r.params.update(params)
    return r


def place(r, x, y, th_deg):
    r.x, r.y, r.th = x, y, math.radians(th_deg)


def records(r, kind):
    cols, out = None, []
    for _, line in r.lines:
        if line.startswith("H," + kind + ","):
            cols = line.split(",")[2:]
        elif line.startswith("D," + kind + ",") and cols:
            out.append(dict(zip(cols, line.split(",")[2:])))
    return out


def ang(a):
    return (a + 180.0) % 360.0 - 180.0


def face_of(r):
    """True distance of the robot's front face (bracket) from the x board / y board, and its heading."""
    return r.y - r.cfg.dock.bump_ahead_cm, r.x - r.cfg.dock.bump_ahead_cm


class Laptop:
    """Stands in for the laptop's radio: runs a script (a generator) that yields a command to send, a number of
    robot ms to wait, or a function of the robot to wait for."""

    def __init__(self, rb, script, limit_ms=240000):
        self.rb, self.gen, self.item, self.until = rb, script(rb), None, None
        self.sent = []
        self.done = False
        self.deadline = rb.ms + limit_ms             # a script that waits for ever stops the run instead
        self.timed_out = False

    def poll(self, rb):
        if rb.ms > self.deadline and not self.timed_out:
            self.timed_out = True
            return "stop"
        while not self.done:
            if self.item is None:
                try:
                    self.item = next(self.gen)
                except StopIteration:
                    self.done = True
                    return None
                if isinstance(self.item, (int, float)):
                    self.until = rb.ms + self.item
            it = self.item
            if isinstance(it, str):
                self.item = None
                self.sent.append(it)
                return it
            if isinstance(it, (int, float)):
                if rb.ms < self.until:
                    return None
                self.item = None
                continue
            if not it(rb):
                return None
            self.item = None
        return None

    def transmit(self, rb, text):
        pass


def leg_state(rb, lid):
    st = [x["result"] for x in records(rb, "LEG") if x["id"] == str(lid)]
    return st[-1] if st else None


def camera(rb, rng, sd_cm=0.3, sd_deg=1.0):
    """What the camera sees: the true pose with noise (cm, deg)."""
    return rb.x + rng.gauss(0, sd_cm), rb.y + rng.gauss(0, sd_cm), math.degrees(rb.th) + rng.gauss(0, sd_deg)


class BumpTests(unittest.TestCase):
    def creep_into_x_board(self, seed, gap=12.0, th_err=0.0, **kw):
        """Pose known (facing the x board exactly as far as it believes); the true heading th_err off."""
        r = make(seed=seed, e02_find_stage_cm=0, **kw)
        place(r, 50.0, r.cfg.dock.bump_ahead_cm + gap, -90.0 + th_err)
        r.pose.set(50.0, r.cfg.dock.bump_ahead_cm + gap, -90.0, 0.5, 1.0)
        return r, r.run_experiment("find")

    def test_bump_detected_with_realistic_noise(self):
        """10 creeps with 0.05 g of creep vibration: every one is a bump at the board, its believed contact is where
        the board really is (to 0.6 cm), it is caught within 0.3 s, and the wheels (slipping) count under 1.5 cm after the contact."""
        for seed in range(10):
            r, run = self.creep_into_x_board(seed, gap=rng_gap(seed))
            b = records(r, "BUMP")[0]
            self.assertEqual(b["result"], "bump", (seed, b))
            self.assertIn(b["cue"], ("jolt+stall", "stall"))
            # believed contact: the face on the board (the pose was right before the creep)
            self.assertAlmostEqual(float(b["y_cm"]) - r.P("bump_ahead_cm"), 0.0, delta=0.6, msg=str(b))
            self.assertLess(int(b["delay_ms"]), 300, b)
            self.assertLess(abs(float(b["overshoot_cm"])), 1.5, b)
            tb = r.e_bumps[0].true_contact
            self.assertLess(abs(tb[1] - r.cfg.dock.bump_ahead_cm), 0.1)          # it really was at the board

    def test_no_false_bump_on_a_seam(self):
        """A plywood seam halfway to the board (a 0.4 g jolt and a 50 ms dip to 50% speed, or a hard one: 0.6 g
        and 70 ms at 20%) is not a bump: the bump comes at the board, and the jolt is counted as ignored."""
        for jolt, dip_ms, dip in ((0.4, 50.0, 0.5), (0.6, 70.0, 0.2)):
            for seed in range(3):
                r = make(seed=seed, seams=[[-10.0, 10.0, 130.0, 10.0]], e02_find_stage_cm=0)
                r.cfg.session.seam_jolt_g, r.cfg.session.seam_dip_ms, r.cfg.session.seam_dip_frac = jolt, dip_ms, dip
                place(r, 50.0, 22.0, -90.0)
                r.pose.set(50.0, 22.0, -90.0, 0.5, 1.0)
                r.run_experiment("find")
                self.assertTrue(r.seam_jolts, "the creep crossed the seam")
                b = records(r, "BUMP")[0]
                self.assertEqual(b["result"], "bump", b)
                self.assertAlmostEqual(r.e_bumps[0].true_contact[1], r.cfg.dock.bump_ahead_cm, delta=0.15)
                self.assertGreaterEqual(int(b["jolts_ignored"]), 1, b)

    def test_push_to_square_fixes_the_approach_angle(self):
        """Coming in 5 to 10 degrees crooked (its own heading thinks it is straight), the push leaves it square to
        the board to 0.5 degrees, and the find takes the board's heading as its own."""
        for th_err in (5.0, -7.0, 10.0, -10.0):
            r, _ = self.creep_into_x_board(3, gap=10.0, th_err=th_err)
            b = records(r, "BUMP")[0]
            self.assertEqual(b["result"], "bump", b)
            self.assertEqual(b["squared"], "1", b)
            # truly square to the board once pushed
            self.assertAlmostEqual(math.degrees(r.e_bumps[0].true_square[2]), -90.0, delta=0.5, msg=str(b))
            # what bump 1 changed in its heading = the turn the push made, as its (7% short) gyro counted it
            f = records(r, "FIND")[0]
            self.assertAlmostEqual(float(f["corr1_deg"]), th_err * 0.93, delta=0.7, msg=str((th_err, f)))

    def test_slipping_wheels_fail_safely(self):
        """If the wheels never stall (they only slip against the board), there is no bump: the find fails plainly
        within the creep limit instead of pushing on."""
        r = make(stall=1.0, e02_find_max_cm=30)
        place(r, 50.0, 25.0, -88.0)
        run = r.run_experiment("find")
        self.assertTrue(run["result"].startswith("failed:x board: no bump within"), run["result"])
        self.assertIn("no bump within", records(r, "FIND")[0]["result"])
        self.assertFalse(r.pose.known)                       # it never learnt where it is

    def test_stall_far_from_a_board_is_not_a_bump(self):
        """Something low that stalls the wheels while the front sensor still sees the board 30 cm ahead (a cable,
        a lip): the find fails, it does not take that for the board."""
        r = make()
        r.cfg.dock.dock_tof = False                          # solid for the body, unseen by the distance sensor
        r.dock_polys = [[(40.0, 34.0), (60.0, 34.0), (60.0, 34.5), (40.0, 34.5)]]
        r.update_solids()
        place(r, 50.0, 46.0, -90.0)
        run = r.run_experiment("find")
        self.assertIn("not at a board", records(r, "FIND")[0]["result"])
        self.assertTrue(run["result"].startswith("failed:"), run["result"])


def rng_gap(seed):
    return 6.0 + 2.0 * (seed % 5)


class FindTests(unittest.TestCase):
    def test_find_gives_the_position_within_1_cm(self):
        """From 8 different spots, headings 0-12 deg off, pose unknown (assumes it faces the x board; within 45 cm
        of the y board) or known roughly (3 cm, 4 deg off): the reset pose is within 1 cm and 0.6 deg of the
        truth, and the pose after backing off too."""
        rng = random.Random(7)
        worst = 0.0
        for k in range(8):
            known = k % 2 == 1
            r = make(seed=100 + k)
            # no pose: it must start within reach of the y board (send a camera fix first otherwise)
            x, y, th = rng.uniform(25, 70) if known else rng.uniform(20, 45), rng.uniform(15, 45), -90.0 + rng.uniform(-12, 12)
            place(r, x, y, th)
            if known:
                r.pose.set(x + rng.uniform(-3, 3), y + rng.uniform(-3, 3), th + rng.uniform(-4, 4), 0.5, 1.0)
            run = r.run_experiment("find")
            self.assertEqual(run["result"], "done", (k, records(r, "FIND")))
            f = records(r, "FIND")[0]
            self.assertTrue(f["result"].startswith("ok"), f)
            self.assertEqual(f["known_before"], "1" if known else "0")
            tx, ty, tth = r.e_reset_true
            ex, ey = float(f["reset_x"]) - tx, float(f["reset_y"]) - ty
            self.assertLess(math.hypot(ex, ey), 1.0, (k, f, r.e_reset_true))
            self.assertLess(abs(ang(float(f["reset_th"]) - math.degrees(tth))), 0.6, (k, f))
            self.assertLess(math.hypot(r.pose.x - r.x, r.pose.y - r.y), 1.0)
            self.assertAlmostEqual(float(f["turn_k"]), GYRO_FIX, delta=0.03, msg=str(f))
            worst = max(worst, math.hypot(ex, ey))
            self.assertEqual(len(records(r, "BUMP")), 2)
            self.assertEqual(len([s for s in records(r, "SYNC") if s["tag"] == "start"]), 1)   # sync flash
        print("\nfind: worst reset position error %.2f cm over 8 finds" % worst)

    def test_find_without_the_retrace_would_be_off(self):
        """turn_k matters: with the gyro under-reading 7% the robot drives to the y board 6-7 degrees crooked,
        so the plain own pose drifts sideways; retrace_cm says how much the re-trace moved it (> 1 cm here)."""
        r = make(seed=5)
        place(r, 45.0, 35.0, -90.0)
        r.run_experiment("find")
        f = records(r, "FIND")[0]
        self.assertGreater(float(f["retrace_cm"]), 1.0, f)

    def test_find_stops_at_an_open_edge(self):
        """No x board (an open plywood edge there) and the edge guard on: the find fails, saying so; the robot
        stops before the edge."""
        r = make(boards=(YBOARD,), safe_edge=1, edge_dir=1, edge_thr=400, e02_find_max_cm=40)
        r.cfg.session.board_cm = [0.0, 0.0, 122.0, 122.0]
        r.cfg.session.open_edges = ["y0"]
        place(r, 50.0, 25.0, -90.0)
        run = r.run_experiment("find")
        self.assertEqual(run["result"], "limited:plywood edge")
        self.assertIn("edge guard tripped", records(r, "FIND")[0]["result"])
        self.assertGreater(r.y, 0.0)                                   # still on the board

    def test_find_rejects_a_bump_far_from_where_the_board_should_be(self):
        """Pose known, but something stands 10 cm before the x board (another robot): not taken for the board."""
        block = [[40.0, 12.0], [60.0, 12.0], [60.0, 14.0], [40.0, 14.0]]
        r = make(boards=(XBOARD, YBOARD, block), e02_find_stage_cm=0)
        place(r, 50.0, 30.0, -90.0)
        r.pose.set(50.0, 30.0, -90.0, 0.5, 1.0)
        run = r.run_experiment("find")
        self.assertIn("sooner than the x board should be", records(r, "FIND")[0]["result"])
        self.assertTrue(run["result"].startswith("failed:"))


class RouteTests(unittest.TestCase):
    STARTS = {1: (10.0, 61.0), 2: (40.0, 80.0), 3: (35.0, 35.0)}     # E01's start spots, facing +x

    def run_route(self, route, seed, corrected=True, **params):
        r = make(seed=seed)
        place(r, 30.0, 30.0, -85.0)
        self.assertEqual(r.run_experiment("find")["result"], "done")
        if corrected:
            r.params["gyro_scale"] = GYRO_FIX                         # C: learnt between the find and the route
        sx, sy = self.STARTS[route]
        r.params.update(e02_route=route, e02_start_x=sx, e02_start_y=sy, e02_route_deg=0, **params)
        run = r.run_experiment("e02")
        return r, run, records(r, "E02")[-1], r.e_routes[-1]

    def test_routes_return_to_start(self):
        """Calibrated (C), each route comes back within 2 cm of where it really started and reaches its far point
        within 2.5 cm; the E02 record carries believed start / far / end and the wheels stay within the cap."""
        for route in (1, 2, 3):
            r, run, e, R = self.run_route(route, seed=route)
            self.assertEqual(run["result"], "done", e)
            self.assertEqual(e["result"], "done")
            s, f, en = R["start"], R["far"], R["end"]
            self.assertLess(math.hypot(en.true[0] - s.true[0], en.true[1] - s.true[1]), 2.0, (route, e))
            gx, gy = R["far_goal"]
            self.assertLess(math.hypot(f.true[0] - gx, f.true[1] - gy), 2.5, (route, e))
            # believed poses: the start is the start spot, the end is back on it by its own reckoning
            self.assertAlmostEqual(float(e["start_x"]), self.STARTS[route][0], delta=1.0)
            self.assertLess(math.hypot(float(e["end_x"]) - float(e["start_x"]), float(e["end_y"]) - float(e["start_y"])), 0.5)
            self.assertGreaterEqual(int(e["ms_far"]) - int(e["ms_start"]), 1000)
            self.assertLess(float(e["path_cm"]), r.P("e02_transit_max_cm") + 2 * 100 + r.P("e02_margin_cm"))

    def test_uncorrected_gyro_misses_and_the_record_says_where_it_believed(self):
        """U (factory corrections): the same square misses by several cm while its own belief says it came back:
        the experiment's contrast. The E02 record keeps the corrections in use."""
        r, run, e, R = self.run_route(3, seed=3, corrected=False)
        self.assertEqual(run["result"], "done")
        s, en = R["start"], R["end"]
        self.assertGreater(math.hypot(en.true[0] - s.true[0], en.true[1] - s.true[1]), 4.0)
        self.assertLess(math.hypot(float(e["end_x"]) - float(e["start_x"]), float(e["end_y"]) - float(e["start_y"])), 0.5)
        self.assertEqual(float(e["gyro_scale"]), 1.0)

    def test_e02_needs_a_position(self):
        r = make()
        run = r.run_experiment("e02")
        self.assertIn("position unknown", run["result"])

    def test_heading_does_not_jump_when_the_correction_changes(self):
        """motion.h 2026-10-21a: a new gyro_scale counts from then on; the own pose keeps its heading."""
        r = make()
        r.advance(100)
        h0 = r.heading()
        r.params["gyro_scale"] = GYRO_FIX
        r.advance(100)
        self.assertAlmostEqual(r.heading(), h0, delta=0.05)

    def test_leg_pause_and_kc_fixes(self):
        """KC: calibrated sensors, and fixes blended (k 0.5) during the pauses after each turn and drive; the camera
        says where it is (here: the truth with 0.3 cm / 1 deg noise). Fixes sent while it drives are rejected
        (moving)."""
        rng = random.Random(4)

        def script(rb):
            while not rb.e_end.set:
                yield 150
                if rb.e_active:
                    x, y, th = camera(rb, rng)
                    yield "fix %.2f %.2f %.2f 1792400000.%d" % (x, y, th, rb.millis())
        r = make(seed=4)
        place(r, 30.0, 30.0, -85.0)
        r.run_experiment("find")
        r.params.update(e02_route=3, e02_start_x=35, e02_start_y=35, e02_route_deg=0, e02_leg_pause_ms=600,
                        e02_fix_k=0.5, gyro_scale=GYRO_FIX)
        r.radio = Laptop(r, script)
        run = r.run_experiment("e02")
        r.radio = None
        self.assertEqual(run["result"], "done")
        fx = records(r, "FIX")
        res = [x["result"] for x in fx]
        self.assertIn("blended", res)
        self.assertIn("rejected:moving", res)
        R = r.e_routes[-1]
        s, en = R["start"], R["end"]
        self.assertLess(math.hypot(en.true[0] - s.true[0], en.true[1] - s.true[1]), 2.0)


class CameraSteeredTests(unittest.TestCase):
    def test_camera_steered_square(self):
        """K (mode 1): the laptop sends a camera fix (replace) before every leg and works out each turn and drive
        from it, re-aiming and correcting once as a camera-steered runner should (the robot's own gyro is 7% short
        here); the robot drives only those legs; the square ends within 1.5 cm of its start."""
        rng = random.Random(9)
        S = 40.0
        start = (35.0, 35.0)
        corners = [(start[0] + S, start[1]), (start[0] + S, start[1] + S), (start[0], start[1] + S), start]
        ids = iter(range(1, 1000))

        def wait_done(lid):
            return lambda rb: leg_state(rb, lid) in ("done", "failed")

        def fix(rb):
            x, y, th = camera(rb, rng)
            return "fix %.2f %.2f %.2f 1792400000.0" % (x, y, th)

        def aim(rb, th_goal):
            # turn, look again with the camera, turn the rest (the robot's own gyro may be 7% short)
            for _ in range(3):
                yield 400
                yield fix(rb)
                yield 50
                err = ang(th_goal - rb.pose.th())
                if abs(err) <= 1.5:
                    return
                lid = next(ids)
                yield "turn %.2f %d" % (err, lid)
                yield wait_done(lid)

        def goto(rb, tx, ty):
            for _ in range(2):                              # drive, look, one correction if > 1 cm off
                yield 400
                yield fix(rb)
                yield 50
                if math.hypot(tx - rb.pose.x, ty - rb.pose.y) <= 1.0:
                    return
                yield from aim(rb, math.degrees(math.atan2(ty - rb.pose.y, tx - rb.pose.x)))
                lid = next(ids)
                yield "go %.2f %d" % (math.hypot(tx - rb.pose.x, ty - rb.pose.y), lid)
                yield wait_done(lid)

        def script(rb):
            yield 300
            yield from goto(rb, *start)
            yield from aim(rb, 0.0)
            lid = next(ids)
            yield "mark start %d" % lid
            yield wait_done(lid)
            for i, (cx, cy) in enumerate(corners):
                yield from goto(rb, cx, cy)
                if i == 1:
                    lid = next(ids)
                    yield "e02 far %d" % lid                 # the laptop lane's spelling of mark far
                    yield wait_done(lid)
            yield from aim(rb, 0.0)
            lid = next(ids)
            yield "mark end %d" % lid

        r = make(seed=9)
        place(r, 30.0, 30.0, -85.0)
        r.run_experiment("find")
        r.params.update(e02_mode=1, e02_route=3, e02_square_cm=S, e02_quiet_ms=1e9)   # K: fixes replace (k unused)
        r.radio = Laptop(r, script)
        run = r.run_experiment("e02")
        r.radio = None
        self.assertEqual(run["result"], "done", records(r, "E02"))
        e = records(r, "E02")[-1]
        self.assertEqual(e["mode"], "1")
        legs = records(r, "LEG")
        self.assertTrue(all(x["result"] != "failed" for x in legs), [x for x in legs if x["result"] == "failed"])
        self.assertIn("replaced", [x["result"] for x in records(r, "FIX")])
        R = r.e_routes[-1]
        s, f, en = R["start"], R["far"], R["end"]
        self.assertLess(math.hypot(en.true[0] - s.true[0], en.true[1] - s.true[1]), 1.5, e)
        self.assertLess(math.hypot(f.true[0] - (start[0] + S), f.true[1] - (start[1] + S)), 1.5, e)

    def test_legs_are_checked(self):
        """Duplicates are acknowledged but not driven twice; too-long and backwards-too-far legs are refused; legs
        outside a camera-steered run are refused; the travel cap stops a laptop that keeps sending."""
        def script(rb):
            yield 300
            yield "go 5 1"
            yield lambda rb: leg_state(rb, 1) == "done"
            yield "go 5 1"                                  # resent: dup
            yield "go 500 2"                                # too long
            yield "go -30 3"                                # backwards too far
            yield "turn 200 4"                              # beyond +/-180
            for k in range(10, 80, 2):                      # back and forth until the travel cap (ids go up)
                yield "turn 180 %d" % k
                yield (lambda kk: (lambda rb: leg_state(rb, kk) in ("done", "failed")))(k)
                yield "go 40 %d" % (k + 1)
                yield (lambda kk: (lambda rb: leg_state(rb, kk) in ("done", "failed") or bool(rb.abort_reason)))(k + 1)
        r = make(seed=2)
        place(r, 30.0, 30.0, -85.0)
        r.run_experiment("find")
        # outside a run: marks are refused
        r.handle_command("mark far", False)
        self.assertEqual(records(r, "LEG")[-1]["result"], "refused")
        r.params.update(e02_mode=1, e02_route=1, e02_straight_cm=20, e02_quiet_ms=1e9, e02_start_x=-999)
        place(r, 60.0, 60.0, math.degrees(r.th))
        r.pose.set(60.0, 60.0, math.degrees(r.th), 0.5, 1.0)
        r.radio = Laptop(r, script)
        run = r.run_experiment("e02")
        r.radio = None
        legs = records(r, "LEG")
        st = [(x["id"], x["result"]) for x in legs]
        self.assertIn(("1", "dup"), st)
        self.assertIn(("2", "refused"), st)
        self.assertIn(("3", "refused"), st)
        self.assertIn(("4", "refused"), st)
        self.assertEqual(len([s for s in st if s == ("1", "done")]), 1)
        self.assertEqual(run["result"], "stopped:drove further than the route allows")
        cap = 2 * 20 + r.P("e02_transit_max_cm") + r.P("e02_margin_cm")
        self.assertLess(float(records(r, "E02")[-1]["path_cm"]), cap + 3.0)

    def test_stop_always_works(self):
        """A stop in the middle of a creep, a route leg or a camera-steered wait ends the run at once."""
        def stopper(at_ms):
            def script(rb):
                yield at_ms
                yield "stop"
            return script
        r = make()
        place(r, 50.0, 40.0, -90.0)
        r.radio = Laptop(r, stopper(4000))
        run = r.run_experiment("find")
        self.assertEqual(run["result"], "stopped:stop command")
        self.assertIn("stopped (stop command)", records(r, "FIND")[0]["result"])
        r.radio = None
        r2 = make(seed=1)
        place(r2, 30.0, 30.0, -85.0)
        r2.run_experiment("find")
        r2.params.update(e02_route=1, e02_start_x=10, e02_start_y=61, e02_route_deg=0)
        r2.radio = Laptop(r2, stopper(6000))
        run = r2.run_experiment("e02")
        self.assertEqual(run["result"], "stopped:stop command")
        self.assertTrue(records(r2, "E02")[-1]["result"].startswith("stopped:stop command"))


class IdleLegTests(unittest.TestCase):
    def test_idle_go_and_turn(self):
        """With no run going, go / turn still work (the laptop parks a robot, finds it in the picture, brings a lost
        one back): LEG records with run "-", the length limits, the pose kept and moved."""
        r = make(seed=3)
        place(r, 60.0, 60.0, 0.0)
        r.handle_command("fix 60 60 0 1.0", False)
        r.handle_command("go 12 5", False)
        legs = records(r, "LEG")
        self.assertEqual([x["result"] for x in legs], ["queued", "started", "done"])
        self.assertEqual(legs[-1]["run"], "-")
        self.assertAlmostEqual(r.x, 72.0, delta=0.5)
        self.assertAlmostEqual(r.pose.x, 72.0, delta=0.5)
        r.handle_command("turn 90 6", False)
        self.assertEqual(records(r, "LEG")[-1]["result"], "done")
        self.assertAlmostEqual(r.pose.th(), 90.0, delta=2.0)
        r.handle_command("go 500 7", False)
        self.assertEqual(records(r, "LEG")[-1]["result"], "refused")
        r.handle_command("go -40 8", False)
        self.assertEqual(records(r, "LEG")[-1]["result"], "refused")
        r.handle_command("go 5 6", False)                    # id not above the last: dup, not driven
        self.assertEqual(records(r, "LEG")[-1]["result"], "dup")

    def test_idle_go_stops_at_an_open_edge(self):
        r = make(seed=3, safe_edge=1, edge_dir=1, edge_thr=400)
        r.cfg.session.board_cm = [0.0, 0.0, 122.0, 122.0]
        r.cfg.session.open_edges = ["x1"]
        place(r, 100.0, 60.0, 0.0)
        r.handle_command("go 40 1", False)
        last = records(r, "LEG")[-1]
        self.assertEqual(last["result"], "failed")
        self.assertIn("plywood edge", last["info"])
        self.assertLess(r.x + r.cfg.line.sensor_forward_cm, 122.5)


class BoardSideTests(unittest.TestCase):
    def test_boards_at_the_high_ends(self):
        """e02_xboard_side 1 / e02_yboard_side 1 (the laptop's 0|1 convention: 1 = the high end): boards along
        y = 122 and x = 122; the find works there too, and an L turns towards the middle."""
        xb = [[-3.0, 122.0], [125.0, 122.0], [125.0, 125.0], [-3.0, 125.0]]
        yb = [[122.0, -3.0], [125.0, -3.0], [125.0, 125.0], [122.0, 125.0]]
        r = make(seed=8, boards=(xb, yb), e02_xboard_cm=122, e02_yboard_cm=122, e02_xboard_side=1, e02_yboard_side=1)
        place(r, 80.0, 90.0, 95.0)
        r.handle_command("fix 80.3 89.8 94 1.0", False)
        run = r.run_experiment("find")
        self.assertEqual(run["result"], "done", records(r, "FIND"))
        f = records(r, "FIND")[0]
        tx, ty, _ = r.e_reset_true
        self.assertLess(math.hypot(float(f["reset_x"]) - tx, float(f["reset_y"]) - ty), 1.0)
        self.assertAlmostEqual(float(f["reset_x"]), 122.0 - r.P("bump_ahead_cm"), delta=0.01)
        # L from beside the x board, heading -x (away from the y board): the turn goes towards y < 122
        r.params.update(gyro_scale=GYRO_FIX, e02_route=2)
        run = r.run_experiment("e02")
        self.assertEqual(run["result"], "done")
        R = r.e_routes[-1]
        self.assertLess(R["far"].true[1], R["start"].true[1] - 20.0)


# Nick's session layout (2026-10-21): the blocked boards on the sides OPPOSITE the floor origin, along y = 122
# (the x board) and x = 122 (the y board), meeting at the far corner; the origin corner and its edges open.
MAX_XB = [[-3.0, 122.0], [125.0, 122.0], [125.0, 125.0], [-3.0, 125.0]]
MAX_YB = [[122.0, -3.0], [125.0, -3.0], [125.0, 125.0], [122.0, 125.0]]
MAX_PARAMS = dict(e02_xboard_cm=122, e02_yboard_cm=122, e02_xboard_side=1, e02_yboard_side=1)


def make_max(seed=0, **params):
    p = dict(MAX_PARAMS)
    p.update(params)
    r = make(seed=seed, boards=(MAX_XB, MAX_YB), **p)
    r.cfg.session.board_cm = [0.0, 0.0, 122.0, 122.0]
    r.cfg.session.open_edges = ["x0", "y0"]
    return r


class FarCornerLayoutTests(unittest.TestCase):
    def test_find_against_the_far_corner(self):
        """Boards at x = 122 and y = 122 (driving towards +x and +y to bump): 12 finds from spots all over the
        sheet (camera fix first; uncorrected gyro; edge guard on its line sensors) end within 1 cm and 0.6 deg."""
        rng = random.Random(12)
        worst = 0.0
        for k in range(12):
            r = make_max(seed=600 + k, safe_edge=1, edge_dir=1, edge_thr=400)
            x, y, th = rng.uniform(15, 105), rng.uniform(15, 105), rng.uniform(-180, 180)
            place(r, x, y, th)
            r.handle_command("fix %.2f %.2f %.2f 1.0" % (x + rng.gauss(0, 0.3), y + rng.gauss(0, 0.3),
                                                          th + rng.gauss(0, 1)), False)
            run = r.run_experiment("find")
            self.assertEqual(run["result"], "done", (k, records(r, "FIND")))
            f = records(r, "FIND")[0]
            tx, ty, tth = r.e_reset_true
            e = math.hypot(float(f["reset_x"]) - tx, float(f["reset_y"]) - ty)
            self.assertLess(e, 1.0, (k, f))
            self.assertLess(abs(ang(float(f["reset_th"]) - math.degrees(tth))), 0.6)
            self.assertAlmostEqual(float(f["reset_x"]), 122.0 - r.P("bump_ahead_cm"), delta=0.01)   # faces +x at the end
            self.assertAlmostEqual(float(f["reset_th"]), 0.0, delta=0.01)
            worst = max(worst, e)
            self.assertFalse(r.edge_trips)
        print("\nfar-corner finds: worst reset error %.2f cm over 12" % worst)

    def test_unknown_pose_facing_the_x_board(self):
        """No fix: parked facing the x board (+y), within reach of the y board: the find still works."""
        r = make_max(seed=7)
        place(r, 90.0, 85.0, 95.0)
        self.assertEqual(r.run_experiment("find")["result"], "done", records(r, "FIND"))
        tx, ty, _ = r.e_reset_true
        f = records(r, "FIND")[0]
        self.assertEqual(f["known_before"], "0")
        self.assertLess(math.hypot(float(f["reset_x"]) - tx, float(f["reset_y"]) - ty), 1.0)

    def test_routes_from_the_far_corner(self):
        """C: the three routes start where the find leaves them (or a start spot), heading away from the y board
        (-x), turning towards the middle (-y); each comes back within 2 cm and stays on the sheet."""
        for route, start in ((1, (112.0, 61.0)), (2, None), (3, None)):
            r = make_max(seed=700 + route, safe_edge=1, edge_dir=1, edge_thr=400)
            place(r, 70.0, 80.0, 60.0)
            r.handle_command("fix 70 80 60 1.0", False)
            self.assertEqual(r.run_experiment("find")["result"], "done")
            r.params.update(gyro_scale=GYRO_FIX, e02_route=route)
            if start:
                r.params.update(e02_start_x=start[0], e02_start_y=start[1])
            run = r.run_experiment("e02")
            self.assertEqual(run["result"], "done", records(r, "E02")[-1])
            R = r.e_routes[-1]
            s, f, en = R["start"], R["far"], R["end"]
            self.assertLess(math.hypot(en.true[0] - s.true[0], en.true[1] - s.true[1]), 2.0, route)
            self.assertLess(f.true[0], s.true[0] - 25.0)                       # out along -x
            if route > 1:
                self.assertLess(f.true[1], s.true[1] - 25.0)                   # turned towards the middle (-y)
            self.assertTrue(all(0 < p[1] < 122 and 0 < p[2] < 122 for p in r.traj[-2000:]))

    def test_camera_steered_leg_towards_the_far_corner(self):
        """Mode 1 in this layout: go legs towards +x / +y, fixes replace."""
        def script(rb):
            yield 400
            yield "fix %.2f %.2f %.2f 1.0" % (rb.x, rb.y, math.degrees(rb.th))
            yield 50
            yield "turn %.2f 1" % ang(45.0 - rb.pose.th())
            yield lambda rb: leg_state(rb, 1) == "done"
            yield "go 20 2"
            yield lambda rb: leg_state(rb, 2) == "done"
            yield "e02 end 3"
        r = make_max(seed=9)
        place(r, 60.0, 60.0, 0.0)
        r.handle_command("fix 60 60 0 1.0", False)
        r.params.update(e02_mode=1, e02_route=3, e02_quiet_ms=1e9)
        r.radio = Laptop(r, script)
        run = r.run_experiment("e02")
        r.radio = None
        self.assertEqual(run["result"], "done")
        self.assertGreater(r.x, 70.0)
        self.assertGreater(r.y, 70.0)
        self.assertIn("replaced", [x["result"] for x in records(r, "FIX")])


class FixTests(unittest.TestCase):
    def test_fix_blend_replace_and_first_fix(self):
        r = make()
        # no pose yet: the first fix sets it (k does not matter), and needs a heading
        r.handle_command("fix 10 20 - 1.0", False)
        self.assertEqual(records(r, "FIX")[-1]["result"], "rejected:the first fix needs a heading")
        r.handle_command("fix 10 20 30 1.0", False)
        self.assertEqual(records(r, "FIX")[-1]["result"], "set")
        self.assertAlmostEqual(r.pose.x, 10.0)
        self.assertAlmostEqual(r.pose.th(), 30.0, delta=0.01)
        # idle: a fix replaces, whatever e02_fix_k
        r.params["e02_fix_k"] = 0.5
        r.handle_command("fix 11 21 31 1.5", False)
        self.assertEqual(records(r, "FIX")[-1]["result"], "replaced")
        self.assertEqual((round(r.pose.x, 6), round(r.pose.y, 6)), (11.0, 21.0))
        # KC blend (during a run): halfway, heading halfway (the shorter way round)
        r.handle_command("fix 10 20 30 1.9", False)
        r.e_active = True                                  # as inside `run e02`, standing still
        r.handle_command("fix 14 24 40 2.0", True)
        f = records(r, "FIX")[-1]
        self.assertEqual(f["result"], "blended")
        self.assertAlmostEqual(r.pose.x, 12.0, delta=1e-6)
        self.assertAlmostEqual(r.pose.y, 22.0, delta=1e-6)
        self.assertAlmostEqual(r.pose.th(), 35.0, delta=0.01)
        r.handle_command("fix 12 22 -170 3.0", True)                    # 35 -> -170 the short way is +155: 112.5
        self.assertAlmostEqual(ang(r.pose.th() - 112.5), 0.0, delta=0.01)
        # position only
        r.handle_command("fix 20 22 - 4.0", True)
        self.assertAlmostEqual(r.pose.x, 16.0, delta=1e-6)
        self.assertAlmostEqual(ang(r.pose.th() - 112.5), 0.0, delta=0.01)
        # K: replace
        r.params["e02_fix_k"] = 1.0
        r.handle_command("fix 50 60 -90 5.0", True)
        self.assertEqual(records(r, "FIX")[-1]["result"], "replaced")
        self.assertEqual((round(r.pose.x, 6), round(r.pose.y, 6)), (50.0, 60.0))
        self.assertEqual(records(r, "FIX")[-1]["t_wall"], "5.0")

    def test_fix_then_find_uses_it(self):
        """A camera fix before the first find (pose unknown at power-on) lets the find drive straight to the
        boards and check the bumps against them (known_before 1)."""
        r = make(seed=11)
        place(r, 60.0, 40.0, 150.0)                    # facing away from both boards
        r.handle_command("fix %.2f %.2f %.2f 1.0" % (r.x + 0.4, r.y - 0.3, 151.0), False)
        run = r.run_experiment("find")
        self.assertEqual(run["result"], "done", records(r, "FIND"))
        f = records(r, "FIND")[0]
        self.assertEqual(f["known_before"], "1")
        tx, ty, _ = r.e_reset_true
        self.assertLess(math.hypot(float(f["reset_x"]) - tx, float(f["reset_y"]) - ty), 1.0)


    def test_find_after_a_route_without_corrections(self):
        """U: after a square without corrections its own pose is well off (10-20 cm). Find again: with a camera
        fix first it ends within 1 cm. Without one it either still finds itself (a later-than-expected bump is
        taken: only the board can be there) or refuses plainly (a bump much sooner than expected might be
        another robot) and keeps its old pose: it never resets to a wrong one."""
        for with_fix in (True, False):
            r = make(seed=31)
            place(r, 30.0, 30.0, -85.0)
            r.run_experiment("find")
            r.params.update(e02_route=3, e02_start_x=35, e02_start_y=35, e02_route_deg=0)
            r.run_experiment("e02")
            off = math.hypot(r.pose.x - r.x, r.pose.y - r.y)
            self.assertGreater(off, 5.0)
            if with_fix:
                r.handle_command("fix %.2f %.2f %.2f 7.0" % (r.x + 0.3, r.y - 0.2, math.degrees(r.th) + 0.8), False)
            run = r.run_experiment("find")
            f = records(r, "FIND")[-1]
            if not with_fix and run["result"] != "done":
                self.assertIn("sooner than the x board should be", f["result"])
                self.assertGreater(math.hypot(r.pose.x - r.x, r.pose.y - r.y), 3.0)    # old (wrong) pose kept
                continue
            self.assertEqual(run["result"], "done", (with_fix, f))
            tx, ty, _ = r.e_reset_true
            self.assertLess(math.hypot(float(f["reset_x"]) - tx, float(f["reset_y"]) - ty), 1.0, (with_fix, f))


class ParityTests(unittest.TestCase):
    def test_every_parameter_used_exists_in_params_h(self):
        """fw_e02.py and e02.h read only parameters params.h defines (a typo would read 0 on the robot)."""
        fw = dict(config.firmware_params())
        repo = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
        for path in (os.path.join(repo, "firmware", "home_lab", "e02.h"), os.path.join(os.path.dirname(HERE), "fw_e02.py")):
            names = set(re.findall(r'P\("(\w+)"', open(path).read()))
            names |= set(re.findall(r'"(e02_\w+|bump_\w+|push_\w+)"', open(path).read()))
            names = {n for n in names if not n.endswith("_side") or n in fw}
            missing = sorted(n for n in names if n not in fw and n not in ("e02_xboard_side", "e02_yboard_side"))
            self.assertEqual(missing, [], path)

    def test_record_headers_match(self):
        """The simulator sends the same BUMP / FIND / E02 / LEG / FIX columns as the firmware."""
        repo = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
        fw = open(os.path.join(repo, "firmware", "home_lab", "e02.h")).read()
        sim = open(os.path.join(os.path.dirname(HERE), "fw_e02.py")).read()

        def headers(text):
            out = {}
            for m in re.finditer(r'"H,(BUMP|FIND|E02|LEG|FIX),((?:[^"]|"\s*")*)"', text):
                out[m.group(1)] = re.sub(r'"\s*"', "", m.group(2))
            return out
        self.assertEqual(headers(fw), headers(sim))
        self.assertEqual(set(headers(fw)), {"BUMP", "FIND", "E02", "LEG", "FIX"})


class FixRound21bTests(unittest.TestCase):
    """Firmware 2026-10-21b: idle legs next to a board, the edge guard required, blind backing capped, the bare body
    (no nose bracket) in the contact model."""

    def test_idle_turn_and_go_next_to_a_board(self):
        """A robot squared on a board (front 2 cm off it, well inside safe_front_cm) can turn in place, back off,
        and roll up to it again: the front sensor never fails these idle legs. A go that would drive into the board
        is still stopped (soft: the leg fails "obstacle", nothing hits)."""
        r = make(seed=21, bumper_cm=0)
        face = r.face_cm()
        place(r, 50.0, face + 2.0, -90.0)                      # x board along y = 0, facing it
        r.handle_command("fix 50 %.2f -90 1.0" % (face + 2.0), False)
        r.handle_command("turn 45 1", False)
        self.assertEqual(records(r, "LEG")[-1]["result"], "done", records(r, "LEG")[-1])
        r.handle_command("turn -45 2", False)
        self.assertEqual(records(r, "LEG")[-1]["result"], "done")
        r.handle_command("go -6 3", False)
        self.assertEqual(records(r, "LEG")[-1]["result"], "done")
        r.handle_command("go 4 4", False)                       # back towards it, ends 4 cm off: allowed
        self.assertEqual(records(r, "LEG")[-1]["result"], "done", records(r, "LEG")[-1])
        y_before = r.y
        r.handle_command("go 10 5", False)                      # would end inside the board
        last = records(r, "LEG")[-1]
        self.assertEqual(last["result"], "failed")
        self.assertIn("obstacle", last["info"])
        self.assertGreater(r.y - face, 0.3)                     # stopped short, never touched
        self.assertLess(y_before - r.y, 4.0)

    def test_everything_that_drives_refuses_with_the_edge_guard_off(self):
        r = make(seed=22, safe_edge=0)
        place(r, 60.0, 60.0, 0.0)
        r.handle_command("fix 60 60 0 1.0", False)
        r.handle_command("go 10 1", False)
        last = records(r, "LEG")[-1]
        self.assertEqual(last["result"], "refused")
        self.assertIn("safe_edge", last["info"])
        self.assertAlmostEqual(r.x, 60.0, delta=0.01)
        r.run_experiment("find")
        self.assertIn("safe_edge", records(r, "FIND")[-1]["result"])
        self.assertAlmostEqual(r.x, 60.0, delta=0.01)
        r.params["e02_route"] = 1
        r.run_experiment("e02")
        self.assertIn("safe_edge", records(r, "E02")[-1]["result"])
        self.assertAlmostEqual(r.x, 60.0, delta=0.01)
        r.handle_command("turn 30 2", False)                     # turning in place stays allowed
        self.assertEqual(records(r, "LEG")[-1]["result"], "done")

    def test_blind_backing_is_capped_until_a_camera_fix(self):
        r = make(seed=23)
        place(r, 60.0, 60.0, 0.0)
        r.handle_command("fix 60 60 0 1.0", False)
        r.handle_command("go -6 1", False)
        self.assertEqual(records(r, "LEG")[-1]["result"], "done")
        r.handle_command("go -6 2", False)                       # 12 cm blind in all: refused
        last = records(r, "LEG")[-1]
        self.assertEqual(last["result"], "refused")
        self.assertIn("backing blind", last["info"])
        r.handle_command("go 3 3", False)                        # forward legs do not reset it ...
        r.handle_command("go -6 4", False)
        self.assertEqual(records(r, "LEG")[-1]["result"], "refused")
        r.handle_command("fix %.2f %.2f 0 2.0" % (r.x, r.y), False)   # ... a camera fix does
        r.handle_command("go -6 5", False)
        self.assertEqual(records(r, "LEG")[-1]["result"], "done")

    def test_a_fix_during_an_idle_leg_replaces(self):
        r = make(seed=24)
        place(r, 60.0, 60.0, 0.0)
        r.handle_command("fix 60 60 0 1.0", False)
        r.e_begin()
        r.e_idle_leg = True
        r.handle_command("fix 70 60 0 1.5", True)
        fx = records(r, "FIX")[-1]
        self.assertEqual(fx["result"], "replaced")
        self.assertAlmostEqual(r.pose.x, 70.0, delta=0.01)
        r.e_idle_leg = False
        r.e_end_run()

    def test_contact_body_follows_bumper_cm(self):
        """No bracket fitted (bumper_cm 0, the firmware default and the robots in Oct 2026): the bare body touches the
        board, face 4.75 cm ahead of the axle, and the find's reset agrees; with bumper_cm 1.34 the bracket (6.10)."""
        for bumper, face in ((0.0, 4.75), (1.34, 6.10)):
            r = make(seed=25, bumper_cm=bumper)
            self.assertAlmostEqual(r.face_cm(), face, delta=0.01)
            place(r, 45.0, 35.0, -90.0)
            r.pose.set(45.0, 35.0, -90.0, 0.5, 1.0)
            run = r.run_experiment("find")
            self.assertEqual(run["result"], "done", records(r, "FIND"))
            b = r.e_bumps[0]
            tx, ty, tth = b.true_square
            self.assertAlmostEqual(ty, face, delta=0.35, msg=(bumper, ty))     # axle this far from the x board
            tx, ty, tth = r.e_reset_true
            f = records(r, "FIND")[-1]
            self.assertLess(math.hypot(float(f["reset_x"]) - tx, float(f["reset_y"]) - ty), 1.0, (bumper, f))


class MissedBumpTests(unittest.TestCase):
    """The dry run's occasional missed bump (config dock.miss_bump_prob / miss_jolt_prob): a find that misses the
    bump altogether fails safely; one that only misses the jolt still finds the board, by the stall."""

    def test_missed_bump_fails_the_find_safely(self):
        r = make(seed=26)
        r.cfg.dock.miss_bump_prob = 1.0
        place(r, 45.0, 35.0, -90.0)
        r.pose.set(45.0, 35.0, -90.0, 0.5, 1.0)
        r.run_experiment("find")
        f = records(r, "FIND")[-1]
        self.assertTrue(f["result"].startswith("failed"), f)
        self.assertIn("no bump", f["result"])
        self.assertGreater(r.y, 0.0)                         # never through the board

    def test_missed_jolt_is_found_by_the_stall(self):
        r = make(seed=27)
        r.cfg.dock.miss_jolt_prob = 1.0
        place(r, 45.0, 35.0, -90.0)
        r.pose.set(45.0, 35.0, -90.0, 0.5, 1.0)
        run = r.run_experiment("find")
        self.assertEqual(run["result"], "done", records(r, "FIND"))
        self.assertTrue(all(b["result"] == "bump" for b in records(r, "BUMP")))


class DocsTests(unittest.TestCase):
    def test_docs_list_the_same_record_columns_as_the_firmware(self):
        """docs/data-formats.md "E02" gives each record's header line (H,BUMP,...): exactly the firmware's."""
        repo = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
        fw = open(os.path.join(repo, "firmware", "home_lab", "e02.h")).read()
        doc = open(os.path.join(repo, "docs", "data-formats.md")).read()
        doc = doc[doc.index("## E02"):]
        heads = {m.group(1): re.sub(r'"\s*"', "", m.group(2))
                 for m in re.finditer(r'"H,(BUMP|FIND|E02|LEG|FIX),((?:[^"]|"\s*")*)"', fw)}
        documented = dict(re.findall(r"`H,(BUMP|FIND|E02|LEG|FIX),([^`]+)`", doc))
        self.assertEqual(documented, heads)


if __name__ == "__main__":
    unittest.main()
