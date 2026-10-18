"""Plywood edge guard (firmware edge.h, safe_edge) in the simulator.

    python -m unittest tools/sim/tests/test_edge.py -v

The sensor readings past an edge are GUESSES (config.py LineModel.past_edge_reading,
TofModel bottom-zone geometry) until the E0c edge rows are in; these tests check the
guard's logic and stopping distance, not the real sensors.
"""
import math
import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import config  # noqa: E402
import robot  # noqa: E402

EDGE_X = 122.0
PAINTER_TAPE = 260.0          # guess: paper tape reflects nearly like the plywood (150)
GUARD = dict(edge_dir=1, edge_thr=400, edge_bot_cm=12)


def board_config(open_edges=("x1",), tape_lines=None):
    cfg = config.load_config()
    cfg.session.board_cm = [0.0, 0.0, 122.0, 122.0]
    cfg.session.open_edges = list(open_edges)
    cfg.session.tape_lines = tape_lines if tape_lines is not None else []
    cfg.line.tape_reading = PAINTER_TAPE
    return cfg


def make(cfg, x, y, th_deg, seed=0, **params):
    r = robot.Robot("Left", cfg, random.Random(seed), config.firmware_params(), "test", 0.0, 0x1234, "ip")
    r.idle_until(10.0)
    r.x, r.y, r.th = x, y, math.radians(th_deg)
    r.fw_yaw_sign = 1                       # compass already checked
    r.params.update(start_nudge_cm=0, **params)
    return r


def wheels(r):
    """Floor points under the two wheels (axle ends, nominal track)."""
    h = r.track / 2
    return [(r.x - h * math.sin(r.th), r.y + h * math.cos(r.th)), (r.x + h * math.sin(r.th), r.y - h * math.cos(r.th))]


def straight(r, cm, rpm):
    r.params.update(straight_cm=cm, straight_rpm=rpm)
    return r.run_experiment("straight")


class EdgeGuardTests(unittest.TestCase):
    def test_off_by_default(self):
        self.assertEqual(dict(config.firmware_params())["safe_edge"], 0.0)
        r = make(board_config(), 80.0, 61.0, 0.0)
        run = straight(r, 60, 0)
        self.assertEqual(run["result"], "done")             # nothing stops it: it drives past the edge
        self.assertGreater(r.x, EDGE_X)
        self.assertFalse([t for _, t in r.lines if t.startswith("D,EDGE")])

    def test_guard_that_never_trips_changes_nothing(self):
        cfg = config.load_config()                           # no board: the old simulator
        a = make(cfg, 0.0, 0.0, 0.0, seed=3)
        b = make(cfg, 0.0, 0.0, 0.0, seed=3, safe_edge=3, edge_thr=1e6, edge_bot_cm=1e6)
        ra, rb = straight(a, 60, 0), straight(b, 60, 0)
        self.assertEqual(ra["result"], rb["result"])
        self.assertEqual(a.traj, b.traj)
        strip = lambda r: [l for l in r.lines if "EDGE" not in l[1] and ",PARAM," not in l[1]]   # PARAM shows the setting
        self.assertEqual(strip(a), strip(b))
        self.assertTrue(any(t.startswith("D,EDGE") and ",base," in t for _, t in b.lines))

    def test_line_cue_stops_with_wheels_on_board(self):
        # library move() (10 cm/s), 30 rpm (5.3 cm/s), 70 rpm (motor top speed, 12.5 cm/s)
        for rpm in (0, 30, 70):
            for seed in range(3):
                r = make(board_config(), 80.0, 61.0, 0.0, seed=seed, safe_edge=1, **GUARD)
                run = straight(r, 60, rpm)
                self.assertEqual(run["result"], "stopped:plywood edge ahead", (rpm, seed))
                margin = min(EDGE_X - wx for wx, _ in wheels(r))
                self.assertGreater(margin, 2.0, (rpm, seed))     # line sensors 3.5 cm ahead of the axle
                self.assertLess(margin, 3.5, (rpm, seed))

    def test_bottom_zone_cue_stops_earlier(self):
        for rpm in (0, 70):
            r = make(board_config(), 80.0, 61.0, 0.0, safe_edge=2, **GUARD)
            run = straight(r, 60, rpm)
            self.assertEqual(run["result"], "stopped:plywood edge ahead")
            self.assertEqual(r.edge_trips[-1][1], "bottom")
            self.assertGreater(EDGE_X - r.x, 6.0)

    def test_tape_in_the_middle_is_not_an_edge(self):
        cfg = board_config(tape_lines=[[0.0, 61.0, 122.0, 61.0]])
        r = make(cfg, 61.0, 90.0, -90.0, safe_edge=3, **GUARD)        # drives -y across the tape
        run = straight(r, 50, 0)
        self.assertEqual(run["result"], "done")
        self.assertLess(r.y, 45.0)

    def test_tape_beside_an_open_edge(self):
        # x-axis painter's tape 1 cm inside an open y = 0 edge: the guard must wait for the edge itself
        cfg = board_config(open_edges=("y0",), tape_lines=[[0.0, 1.0, 122.0, 1.0]])
        cfg.session.tape_width_cm = 1.9
        r = make(cfg, 61.0, 30.0, -90.0, safe_edge=1, **GUARD)
        run = straight(r, 40, 0)
        self.assertEqual(run["result"], "stopped:plywood edge ahead")
        _, _, (x, y, th) = r.edge_trips[-1]
        sensor_y = y - r.line_m.sensor_forward_cm
        self.assertLess(sensor_y, 0.3)                       # past the edge, not on the tape (0.05 .. 1.95)
        self.assertGreater(min(wy for _, wy in wheels(r)), 2.0)

    def test_calibration_roll_respects_it(self):
        r = make(board_config(), EDGE_X - 12.0, 61.0, 0.0, safe_edge=1, **GUARD)   # 15 cm roll would leave
        run = r.run_experiment("calib")
        self.assertEqual(run["result"], "stopped:plywood edge ahead")
        self.assertGreater(min(EDGE_X - wx for wx, _ in wheels(r)), 2.0)

    def test_survey_drive_respects_it(self):
        # own-pose straight drives (pose.h driveLine, survey_cms 10 cm/s), as follow drives use
        r = make(board_config(), 95.0, 61.0, 0.0, safe_edge=1, **GUARD)
        run = r.run_experiment("survey")
        self.assertEqual(run["result"], "stopped:plywood edge ahead")
        self.assertGreater(min(EDGE_X - wx for wx, _ in wheels(r)), 2.0)

    def test_backing_up_is_blind(self):
        r = make(board_config(), EDGE_X - 10.0, 61.0, 180.0, safe_edge=3, **GUARD)   # facing away
        run = straight(r, -20, 0)
        self.assertEqual(run["result"], "done")
        self.assertGreater(r.x, EDGE_X)

    def test_slant_limit(self):
        # ang = angle between the heading and the x = 122 edge line. Met at 60 deg, the outer line
        # sensor reaches the edge before the outer wheel; at 30 deg the wheel gets there first
        # (edge.h: below about 45 deg the line cue alone is too late)
        for ang, wheel_first in ((60.0, False), (30.0, True)):
            r = make(board_config(), 90.0, 40.0, 90.0 - ang, safe_edge=1, **GUARD)
            straight(r, 80, 0)
            self.assertTrue(r.edge_trips, ang)
            margin = min(EDGE_X - wx for wx, _ in wheels(r))
            self.assertEqual(margin < 0, wheel_first, (ang, margin))

    def test_start_on_tape(self):
        # dark line-following tape (above lf_line_thr) along y = 61 under the line sensors at the start
        def run(width, y):
            cfg = board_config(tape_lines=[[0.0, 61.0, 122.0, 61.0]])
            cfg.session.tape_width_cm = width
            cfg.line.tape_reading = 850.0
            r = make(cfg, 40.0, y, 0.0, safe_edge=1, **GUARD)
            r.params.update(straight_cm=0, straight_rpm=1)
            r.run_experiment("straight")
            return [t for _, t in r.lines if t.startswith("D,EDGE")]
        self.assertIn(",nobase,", run(4.0, 61.0)[0])         # all three on it: line cue off this run
        rec = run(1.9, 62.2)[0]                              # only the right sensor (1.2 cm right of centre) on it
        self.assertIn(",base,", rec)
        self.assertLess(float(rec.split(",")[10]), 300)       # baseR taken from the other two (plywood)


if __name__ == "__main__":
    unittest.main()
