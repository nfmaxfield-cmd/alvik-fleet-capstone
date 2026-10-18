"""Tests for the simulated survey drive and step follower (fw_follow.py, the
copy of firmware pose.h / follow.h / runSurvey). Run from the repo folder:

    python -m unittest discover tools/sim/tests -v
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
import robot  # noqa: E402

BOARD_A = [(30, 1.25), (50, 1.25), (50, 2.75), (30, 2.75)]   # demo map board A; surface y = 2.75


class ScriptRadio:
    """Commands at given robot-clock laptop times; collects what the robot sends."""

    def __init__(self, cmds):
        self.cmds = sorted(cmds)
        self.out = []

    def poll(self, rb):
        t = rb.t_at(rb.ms)
        if self.cmds and self.cmds[0][0] <= t:
            return self.cmds.pop(0)[1]
        return None

    def transmit(self, rb, text):
        self.out.append(text)

    def steps(self):
        return [line.split(",") for line in self.out if line.startswith("D,STEP,")]


def make_robot(seed=0, pose=(40.0, 23.5, -90.0)):
    cfg = config.load_config()
    r = robot.Robot("Left", cfg, random.Random(seed), config.firmware_params(), "t", 0.0, 0x1234, "ip")
    r.idle_until(5.0)
    r.x, r.y, r.th = pose[0], pose[1], math.radians(pose[2])
    r.fw_yaw_sign = 1
    return r


def run_follow(r, cmds):
    """The firmware's idle loop until `run follow` arrives, then the follow run until `stop`."""
    r.radio = ScriptRadio(cmds)
    while True:
        c = r.radio.poll(r)
        if c:
            r.handle_command(c, False)
        if r.pending >= 0:
            r.pending = -1
            return r.run_experiment("follow")
        r.advance(10)


class SurveyTests(unittest.TestCase):
    def test_spiral_and_back(self):
        r = make_robot(1, (60.0, 10.0, 0.0))
        run = r.run_experiment("survey")
        self.assertEqual(run["result"], "done")
        legs = [line for _, line in r.lines if line.startswith("D,SURVEY_LEG")]
        self.assertEqual(len(legs), 16)                          # 10,10,...,80,80 with survey_size_cm 80
        self.assertEqual([int(line.split(",")[4]) for line in legs[:4]], [10, 10, 20, 20])
        self.assertTrue(all(line.split(",")[6] == "full" for line in legs))
        end = [line for _, line in r.lines if line.startswith("D,SURVEY_END")][0].split(",")
        self.assertEqual(end[3], "done")
        self.assertLess(math.hypot(r.x - 60.0, r.y - 10.0), 1.5)  # back where it started (true pose)
        steps = sorted({int(line.split(",")[4]) for _, line in r.lines if line.startswith("D,RAW,")})
        self.assertEqual(steps, [-1, 0] + list(range(1, 18)))     # nudge, legs 1..16, 17 = the drive back

    def test_leg_ends_early_at_an_obstacle(self):
        r = make_robot(2, (60.0, 10.0, 0.0))
        r.world_polys = [[(95, -40), (100, -40), (100, 60), (95, 60)]]   # a wall 35 cm ahead (+x)
        run = r.run_experiment("survey")
        legs = [line.split(",") for _, line in r.lines if line.startswith("D,SURVEY_LEG")]
        self.assertEqual(legs[-1][6], "obstacle")
        end = [line for _, line in r.lines if line.startswith("D,SURVEY_END")][0].split(",")
        self.assertEqual(end[3], "done:blocked")
        self.assertEqual(run["result"], "done")
        self.assertLess(math.hypot(r.x - 60.0, r.y - 10.0), 1.5)
        self.assertLess(r.x + 4.75, 95.0)                            # never touched the wall


class PoseTests(unittest.TestCase):
    def test_late_fix_is_applied_where_the_robot_was(self):
        """A fix that is 300 ms old corrects the pose by the error the robot had THEN."""
        r = make_robot(3, (0.0, 0.0, 0.0))
        p = r.pose
        p.start(True, 0.0, 0.0, 0.0)
        r.tick_hook = p.tick
        p.var = 4.0                                    # unsure: a fix should pull most of the way
        r.set_wheels_speed(30, 30)
        for _ in range(100):
            r.should_abort()
            r.loop_delay(5)
        t_then = r.millis() - 300
        then = p.at(t_then)
        # the camera says: 300 ms ago the robot was 1 cm further left than it thought
        ok = p.fix(t_then, then[0], then[1] + 1.0, then[2], 0.3, 1.0)
        self.assertTrue(ok)
        k = 4.0 / (4.0 + 0.09)
        self.assertAlmostEqual(p.at(t_then)[1] - then[1], k * 1.0, places=2)
        r.brake()

    def test_old_and_wild_fixes_are_ignored(self):
        r = make_robot(4, (0.0, 0.0, 0.0))
        p = r.pose
        p.start(True, 0.0, 0.0, 0.0)
        r.advance(2000)
        p.tick()
        self.assertFalse(p.fix(r.millis() - 1500, 0.0, 0.0, 0.0, 0.3, 1.0))
        self.assertEqual(p.last_reject, "old")
        self.assertFalse(p.fix(r.millis(), 20.0, 0.0, 0.0, 0.3, 1.0))
        self.assertEqual(p.last_reject, "too far from the own pose")


class FollowTests(unittest.TestCase):
    def test_protocol_ack_dup_bad_hold_halt(self):
        r = make_robot(5, (20.0, 30.0, 0.0))
        cmds = [(5.5, "run follow"), (8.0, "fpose 20 30 0 0.3 1"),
                (8.1, "fs 1 W 3000"), (8.12, "fs 1 W 3000"), (8.14, "fs 2 Q 1"), (8.16, "fs 2 D 20 30 30 30 0 10 0 -"),
                (8.18, "fs 3 D 30 30 40 30 0 10 0 -"), (8.2, "hold 2"), (8.3, "fs 4 T 90 -"), (8.4, "halt"),
                (12.0, "stop")]
        run_follow(r, cmds)
        st = [(int(v[3]), v[4]) for v in r.radio.steps()]
        self.assertIn((1, "queued"), st)
        self.assertIn((1, "dup"), st)                  # a resent step is acked again, not run twice
        self.assertIn((2, "bad"), st)
        self.assertIn((2, "queued"), st)
        self.assertIn((3, "dropped"), st)              # hold 2 keeps 2, drops 3
        self.assertIn((2, "held"), st)
        self.assertIn((1, "failed"), st)               # halt stops the wait at once
        self.assertIn((0, "halted"), st)
        self.assertIn((4, "dropped"), st)
        self.assertNotIn((2, "started"), st)

    def test_steps_run_in_order_and_report_done(self):
        r = make_robot(6, (20.0, 30.0, 0.0))
        cmds = [(5.5, "run follow"), (8.0, "fpose 20 30 0 0.3 1"),
                (8.1, "fs 1 D 20 30 35 30 0 10 0 -"), (8.15, "fs 2 T 90 -"), (8.2, "fs 3 D 35 30 35 40 90 10 0 -"),
                (8.25, "fs 4 D 35 40 35 30 90 5 1 -"), (20.0, "stop")]
        run_follow(r, cmds)
        done = [int(v[3]) for v in r.radio.steps() if v[4] == "done"]
        self.assertEqual(done, [1, 2, 3, 4])
        self.assertAlmostEqual(r.x, 35.0, delta=0.8)   # backed out to (35, 30), still facing +y
        self.assertAlmostEqual(r.y, 30.0, delta=0.8)
        self.assertAlmostEqual(math.degrees(r.th) % 360, 90.0, delta=4.0)

    def test_approach_docks_near_1cm_and_never_touches(self):
        errs = []
        for seed in range(8):
            r = make_robot(seed, (40.0, 23.5, -90.0))
            r.world_polys = [BOARD_A]
            run_follow(r, [(5.5, "run follow"), (8.0, "fpose 40 23.5 -90 0.3 1"),
                           (8.1, "fs 1 A 40 23.5 40 8.5 -90 3 3 1 15.5 1"), (8.2, "fs 2 W 1000"), (20.0, "stop")])
            self.assertEqual(len(r.dock_log), 1, seed)
            x, y, th = r.settled_pose(r.dock_log[0]["ms"])
            gap = y + 4.75 * math.sin(th) - 2.75
            errs.append(gap - 1.0)
            closest = min(y0 for _, x0, y0, _, _ in r.traj if abs(x0 - 40.0) < 3.0)
            self.assertGreater(closest - 4.75, 2.75 + 0.3, seed)   # front face never closer than 0.3 cm
        self.assertLess(max(abs(e) for e in errs), 0.35)
        self.assertLess(abs(sum(errs) / len(errs)), 0.15)

    def test_something_in_front_of_the_board_fails_the_approach(self):
        r = make_robot(7, (40.0, 23.5, -90.0))
        r.world_polys = [BOARD_A, [(38, 12), (42, 12), (42, 14), (38, 14)]]   # a block 9 cm before the board
        run_follow(r, [(5.5, "run follow"), (8.0, "fpose 40 23.5 -90 0.3 1"),
                       (8.1, "fs 1 A 40 23.5 40 8.5 -90 3 3 1 15.5 1"), (20.0, "stop")])
        failed = [v for v in r.radio.steps() if v[4] == "failed"]
        self.assertTrue(failed and failed[0][-1].startswith("obstacle"))
        self.assertGreater(r.y - 4.75, 14.0)               # stopped before the block

    def test_planned_pass_near_a_board_does_not_stop(self):
        """A drive ending 4 cm in front of a board: the 7 cm safety stop must not trip (fmin from the plan)."""
        r = make_robot(8, (40.0, 40.0, -90.0))
        r.world_polys = [BOARD_A]
        # ends with the front face 4 cm from the board surface: centre at y = 2.75 + 4 + 4.75
        run_follow(r, [(5.5, "run follow"), (8.0, "fpose 40 40 -90 0.3 1"),
                       (8.1, "fs 1 D 40 40 40 11.5 -90 10 0 4.0"), (16.0, "stop")])
        self.assertIn("done", [v[4] for v in r.radio.steps() if v[3] == "1"])

    def test_quiet_laptop_finishes_buffer_then_stands_still(self):
        r = make_robot(9, (20.0, 30.0, 0.0))
        run_follow(r, [(5.5, "run follow"), (8.0, "fpose 20 30 0 0.3 1"),
                       (8.1, "fs 1 D 20 30 30 30 0 10 0 -"), (8.12, "fs 2 D 30 30 40 30 0 10 0 -"), (20.0, "stop")])
        self.assertEqual([int(v[3]) for v in r.radio.steps() if v[4] == "done"], [1, 2])
        self.assertTrue(any("nothing from the laptop" in line for line in r.radio.out))
        self.assertAlmostEqual(r.x, 40.0, delta=0.8)


if __name__ == "__main__":
    unittest.main()
