"""The calibration run's look-around (firmware look.h / runCalib 2026-10-18c, copied in
tools/sim/fw_look.py): put down near boards, it spins first, then rolls where there is room.

    python -m unittest tools/sim/tests/test_look.py -v
"""
import math
import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path.insert(0, SIM)

import config  # noqa: E402
import fw_look  # noqa: E402
import robot  # noqa: E402

sys.path.insert(0, os.path.dirname(SIM))                        # tools/
import session_checks as sc  # noqa: E402

START = (50.0, 0.0, 0.0)        # robot centre (wheel axle midpoint), facing +x; front face at x 54.75
HALF = 4.75                     # half the 9.5 cm body


def board(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


FRONT10 = board(64.75, -30, 66, 30)              # 10 cm ahead of the front face
RIGHT12 = board(20, -17.75, 66, -16.5)            # 11.75 cm beside the right side
BACK8 = board(36, -30, 37.25, 30)                # 8 cm behind the back
LEFT12 = board(20, 16.5, 66, 17.75)               # 11.75 cm beside the left side


def make_robot(seed, polys, pose=START):
    cfg = config.load_config()
    r = robot.Robot("Left", cfg, random.Random(seed), config.firmware_params(), "t", 0.0, 0x1234, "ip")
    r.radio = None
    r.idle_until(5.0)
    r.x, r.y, r.th = pose[0], pose[1], math.radians(pose[2])
    r.world_polys = polys
    return r


def inside(p, poly):
    x, y = p
    xs = [q[0] for q in poly]
    ys = [q[1] for q in poly]
    return min(xs) < x < max(xs) and min(ys) < y < max(ys)       # the boards here are axis-aligned boxes


def corners(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return [(x + fx * c - ly * s, y + fx * s + ly * c) for fx, ly in ((HALF, HALF), (HALF, -HALF), (-HALF, HALF),
                                                                      (-HALF, -HALF), (HALF, 0), (-HALF, 0))]


def steps_of(r):
    out = []
    for _, line in r.lines:
        if line.startswith("D,RAW,"):
            st = int(line.split(",")[4])
            if not out or out[-1] != st:
                out.append(st)
    return out


def look_record(r):
    head = [line for _, line in r.lines if line.startswith("H,CALIB_LOOK,")]
    rows = [line for _, line in r.lines if line.startswith("D,CALIB_LOOK,")]
    assert len(head) == 1 and len(rows) == 1
    return dict(zip(head[0].split(",")[2:], rows[0].split(",")[2:]))


def pose_at(r, ms):
    """True pose at robot time ms (the last trajectory point before it)."""
    best = r.traj[0]
    for p in r.traj:
        if p[0] > ms:
            break
        best = p
    return best[1:4]


def roll_truth(r):
    """True (distance, direction deg) of the forward roll: pose at the end of RAW step 8 to the
    pose at the end of step 9 (the far end, still)."""
    ms = {}
    for _, line in r.lines:
        if line.startswith("D,RAW,"):
            p = line.split(",")
            ms[int(p[4])] = int(p[3])
    a, b = pose_at(r, ms[8]), pose_at(r, ms[9])
    return math.hypot(b[0] - a[0], b[1] - a[1]), math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


class LookTests(unittest.TestCase):
    def run_calib(self, seed, polys, pose=START, before=None):
        r = make_robot(seed, polys, pose)
        if before:
            for c in before:
                r.handle_command(c, False)
        run = r.run_experiment("calib")
        # the body never enters a board, at any moment of the run
        for ms, x, y, th, _ in r.traj:
            if ms < run["ms_start"]:
                continue
            for p in corners(x, y, th):
                self.assertFalse(any(inside(p, b) for b in polys), (ms, p))
        return r, run

    def test_open_floor_rolls_straight_ahead(self):
        r, run = self.run_calib(1, [])
        self.assertEqual(run["result"], "done")
        self.assertEqual(steps_of(r), [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        rec = look_record(r)
        self.assertEqual(rec["result"], "roll")
        self.assertEqual((rec["spin"], rec["roll"], rec["reason"]), ("done", "full", ""))
        self.assertEqual(sc.calib_outcome(run["result"], rec), sc.CALIB_DONE)
        self.assertTrue(any(t == "M,calibrated: now place the robot on its start mark" for _, t in r.lines))
        self.assertEqual(float(rec["heading_deg"]), 0.0)
        self.assertEqual(float(rec["roll_cm"]), 15.0)
        self.assertEqual(rec["spun"], "1")
        dist, _ = roll_truth(r)
        self.assertAlmostEqual(dist, 15.0, delta=0.5)

    def test_facing_a_board_turns_and_rolls_the_full_roll(self):
        for seed in (2, 3, 4):
            r, run = self.run_calib(seed, [FRONT10])
            self.assertEqual(run["result"], "done", seed)
            rec = look_record(r)
            self.assertEqual(rec["result"], "roll", seed)
            self.assertEqual(float(rec["roll_cm"]), 15.0)
            self.assertLess(abs(abs(float(rec["heading_deg"])) - 180), 20, rec)    # away from the board
            dist, direction = roll_truth(r)
            self.assertAlmostEqual(dist, 15.0, delta=0.5)
            self.assertLess(abs(abs(fw_look.wrap180(direction)) - 180), 25, direction)

    def test_nose_bumper_needs_more_room(self):
        """bumper_cm (a nose bumper sticks out in front of the sensor): the roll keeps that much more room,
        and the spin needs that much more room around it."""
        r = make_robot(1, [])
        lk = fw_look.Look(r)
        lk.reset()
        for rel in range(-180, 180, 2):
            lk.add_point(float(rel), 0.0, 20.0)           # something 20 cm ahead of the sensor all round
        plain = lk.roll_at(0.0)
        r.params["bumper_cm"] = 1.34
        self.assertGreater(plain, 5.0)
        self.assertAlmostEqual(lk.roll_at(0.0), plain - 1.34, delta=0.01)
        # spin: a board the sweep sees ~8.9 cm from the turning centre: outside the plain 8 cm circle, inside
        # 8 + 1.34 (the zones see a flat board ~1.5 cm nearer than straight ahead: the widest zones' slant rays)
        near = [board(50.0 + 10.5, -30, 62.0, 30)]
        r0, _ = self.run_calib(5, near)
        self.assertNotEqual(look_record(r0)["spin"], "skipped")
        r1, _ = self.run_calib(5, near, before=["set bumper_cm 1.34"])
        self.assertEqual(look_record(r1)["spin"], "skipped")

    def test_boxed_in_on_three_sides_picks_the_open_side(self):
        for seed, open_side, polys, want in ((5, "left", [FRONT10, RIGHT12, BACK8], 90),
                                            (6, "right", [FRONT10, LEFT12, BACK8], -90),
                                            (7, "back", [FRONT10, LEFT12, RIGHT12], 180)):
            r, run = self.run_calib(seed, polys)
            self.assertEqual(run["result"], "done", open_side)
            rec = look_record(r)
            self.assertIn(rec["result"], ("roll", "short"), (open_side, rec))
            self.assertLess(abs(fw_look.wrap180(float(rec["heading_deg"]) - want)), 20, (open_side, rec))
            dist, direction = roll_truth(r)
            self.assertAlmostEqual(dist, float(rec["roll_cm"]), delta=0.5)
            self.assertGreaterEqual(float(rec["roll_cm"]), 10.0, (open_side, rec))
            self.assertLess(abs(fw_look.wrap180(direction - want)), 25, (open_side, direction))

    def test_nowhere_to_roll_reports_and_still_spins(self):
        r, run = self.run_calib(8, [FRONT10, LEFT12, RIGHT12, BACK8])
        self.assertEqual(run["result"], "done")
        self.assertEqual(steps_of(r), [0, 1, 2, 3, 4, 5, 6, 7, 8])
        rec = look_record(r)
        self.assertEqual(rec["result"], "no_room")
        self.assertEqual(float(rec["roll_cm"]), 0.0)
        said = [t for _, t in r.lines if t.startswith("M,")]
        self.assertTrue(any("no room to roll" in t for t in said), said)
        # not a failure: RUNEND done, CALIB_LOOK says what was skipped and why (the "limited (no room)" outcome)
        self.assertEqual((rec["spin"], rec["roll"], rec["reason"]), ("done", "skipped", "no room"))
        self.assertTrue(any(t.startswith("M,calibrated, limited (no room): no roll") for t in said), said)
        self.assertEqual(sc.calib_outcome(run["result"], rec), sc.CALIB_LIMITED)

    def test_something_inside_the_spin_circle_no_spin(self):
        # a board 2.5 cm from the front face: 7.25 cm from the turning centre (< calib_spin_r_cm 8)
        r, run = self.run_calib(9, [board(57.25, -30, 58.5, 30)])
        self.assertEqual(run["result"], "done")
        st = steps_of(r)
        self.assertNotIn(4, st)
        self.assertNotIn(9, st)
        rec = look_record(r)
        self.assertEqual(rec["spun"], "0")
        self.assertEqual(rec["result"], "no_room")
        self.assertEqual((rec["spin"], rec["roll"], rec["reason"]), ("skipped", "skipped", "no room"))
        self.assertEqual(sc.calib_outcome(run["result"], rec), sc.CALIB_LIMITED)
        self.assertLess(abs(math.degrees(r.th)), 3)          # it stayed facing the board (only the wiggle)

    def test_laptop_caps(self):
        # caps in every direction: rolls only as far as the smallest cap allows
        caps = " ".join("%d 9" % d for d in range(-135, 181, 45))
        r, run = self.run_calib(10, [], before=["set calib_dir_limits " + caps])
        rec = look_record(r)
        self.assertEqual(rec["result"], "short")
        self.assertEqual(float(rec["roll_cm"]), 9.0)
        self.assertEqual(float(rec["cap_cm"]), 9.0)
        self.assertEqual(len(rec["caps"].split(";")), 8)
        self.assertEqual(r.look.caps, [])                   # used once, then cleared
        # a cap straight ahead only: it rolls the full 15 cm the other way
        r, run = self.run_calib(11, [], before=["set calib_dir_limits 0 2"])
        rec = look_record(r)
        self.assertEqual(float(rec["roll_cm"]), 15.0)
        self.assertLess(abs(abs(float(rec["heading_deg"])) - 180), 3)
        # every direction capped below calib_roll_min_cm: no roll
        r, run = self.run_calib(12, [], before=["set calib_dir_limits " + " ".join("%d 3" % d for d in range(-135, 181, 45))])
        self.assertEqual(look_record(r)["result"], "no_room")

    def test_limits_parsing(self):
        self.assertEqual(sc.calib_outcome("failed:yaw check failed", {"reason": ""}), sc.CALIB_FAILED)
        self.assertEqual(sc.calib_outcome("stopped:plywood edge ahead", {"reason": "no room"}), sc.CALIB_FAILED)
        self.assertEqual(sc.calib_outcome(None, None), sc.CALIB_FAILED)
        self.assertEqual(sc.calib_outcome_text("done", {"reason": "plywood edge"}), "limited (plywood edge)")
        self.assertEqual(sc.calib_outcome_text("done", {"reason": "no room; plywood edge"}),
                         "limited (no room; plywood edge)")
        self.assertEqual(sc.calib_outcome_text("done", {"reason": ""}), "done")
        self.assertEqual(sc.calib_outcome("done", {"result": "no_room"}), sc.CALIB_LIMITED)   # without the new columns
        self.assertEqual(sc.calib_outcome("done", None), sc.CALIB_DONE)                     # before 2026-10-18c
        r = make_robot(13, [])
        sent = []
        r.send = lambda t: sent.append(t)
        r.look.set_limits("0 10 90:12, -90 8")
        self.assertEqual(r.look.caps, [(0.0, 10.0), (90.0, 12.0), (-90.0, 8.0)])
        self.assertEqual(r.look.cap_at(10), 10.0)
        self.assertEqual(r.look.cap_at(-80), 8.0)
        self.assertEqual(r.look.cap_at(180), -1.0)
        r.look.set_limits("0 10 90")
        self.assertEqual(r.look.caps, [])
        self.assertIn("usage", sent[-1])
        r.look.set_limits("0 10")
        r.look.set_limits("off")
        self.assertEqual(r.look.caps, [])

    def test_room_from_the_sweep(self):
        """The map's room ahead of a board seen during the spin: never more than the truth (the
        zones' width only makes things look nearer), and only a little less. look.h keeps the
        NEAREST reading per direction over the whole sweep, so with sensor noise (here sd about
        0.8 cm at 30 cm, a few dozen frames on the board) it comes out low by about 2.5 cm
        (without noise: 0.25 cm). Over 120 seeds: 26.7 to 28.3 cm for a true 30.25, so one seed
        can sit just past 3 cm: each seed within 4 cm, the median within 3."""
        true = 30.0 + self.cfg_recess()
        rooms = []
        for seed in (14, 15, 16, 17, 18):
            r, run = self.run_calib(seed, [board(84.75, -40, 86, 40)])       # 30 cm ahead of the front face
            room0 = float(look_record(r)["room8"].split(";")[0])
            self.assertLessEqual(room0, true + 0.5, seed)
            self.assertGreater(room0, true - 4.0, seed)
            rooms.append(room0)
        self.assertGreater(sorted(rooms)[len(rooms) // 2], true - 3.0, rooms)

    @staticmethod
    def cfg_recess():
        return dict(config.firmware_params())["tof_recess_cm"]


if __name__ == "__main__":
    unittest.main()
