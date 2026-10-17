"""Tests for the simulator.  Run from the repo folder:

    python -m unittest discover tools/sim/tests -v
"""
import csv
import json
import math
import os
import random
import re
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path.insert(0, SIM)

import config  # noqa: E402
import robot  # noqa: E402
import simulate  # noqa: E402


def perfect_config():
    """Every error and noise switched off: the robot should do exactly what it is told."""
    cfg = config.load_config()
    m, im = cfg.robot, cfg.imu
    m.radius_scale_err_pct = m.radius_mismatch_pct = m.track_err_pct = 0.0
    m.wheel_track_noise_deg = 0.0
    m.move_slip_mean_pct = m.move_slip_sd_pct = m.accel_slip_pct_per_g = 0.0
    m.turn_slip_mean_pct = m.turn_slip_sd_pct = 0.0
    m.encoder_res_deg = 1e-6
    im.gyro_bias_dps = im.gyro_bias_walk_dps_per_rt_s = im.yaw_arw_deg_per_rt_s = 0.0
    im.gyro_scale_err_pct = 0.0
    return cfg


def make_robot(cfg, seed=0):
    r = robot.Robot("Left", cfg, random.Random(seed), config.firmware_params(), "test", 0.0, 0x1234, "ip")
    r.idle_until(10.0)
    r.x = r.y = r.th = 0.0
    return r


class RobotModelTests(unittest.TestCase):
    def test_perfect_straight(self):
        r = make_robot(perfect_config())
        r.library_move(120)
        self.assertAlmostEqual(r.x, 120.0, delta=0.05)
        self.assertAlmostEqual(r.y, 0.0, delta=0.05)
        self.assertAlmostEqual(r.odo_x, r.x, delta=0.05)    # odometry = truth when nothing is wrong

    def test_odometry_cannot_see_wheel_size_error(self):
        cfg = perfect_config()
        r = make_robot(cfg)
        r.rL = r.rR = cfg.robot.wheel_radius_cm * 1.01      # real wheels 1% bigger than nominal
        r.library_move(100)
        self.assertAlmostEqual(r.odo_x, 100.0, delta=0.05)  # the robot believes it drove 100 cm
        self.assertAlmostEqual(r.x, 101.0, delta=0.1)       # it really drove 101 cm

    def test_turn_error_comes_from_gyro_scale(self):
        cfg = perfect_config()
        r = make_robot(cfg)
        r.gyro_scale = 0.02                                 # gyro reads 2% too much
        start = r.heading()
        r.imu_turn_to(robot.wrap180(start + 90))
        turned_true = math.degrees(r.th)
        turned_imu = robot.wrap180(r.heading() - start)
        self.assertLess(abs(turned_imu - 90), 1.6)                        # inside the firmware's tolerance
        self.assertAlmostEqual(turned_true, turned_imu / 1.02, delta=0.05)

    def test_turn_slip_shows_in_odometry_only(self):
        cfg = perfect_config()
        cfg.robot.turn_slip_mean_pct = 5.0
        r = make_robot(cfg)
        start = r.heading()
        r.imu_turn_to(robot.wrap180(start + 90))
        self.assertAlmostEqual(math.degrees(r.th), robot.wrap180(r.heading() - start), delta=0.05)
        self.assertGreater(r.odo_th, math.degrees(r.th) * 1.04)   # wheels turned more than the robot did

    def test_lift_freezes_odometry(self):
        r = make_robot(perfect_config())
        h0 = r.heading()
        r.start_lift(30.0, 20.0, math.radians(45), 2000, "test")
        r.advance(2500)
        self.assertEqual((r.odo_x, r.odo_y, r.odo_th), (0.0, 0.0, 0.0))   # wheels didn't turn
        self.assertAlmostEqual(r.x, 30.0, delta=1e-6)
        self.assertAlmostEqual(robot.wrap180(r.heading() - h0), 45.0, delta=0.1)   # the gyro felt the rotation

    def test_raw_columns_match_firmware(self):
        r = make_robot(perfect_config())
        r.run_id = "1234-1"
        r.params["raw_hz"] = 50
        r.raw_start()
        r.advance(25)
        r.tick()
        header = [t for _, t in r.lines if t.startswith("H,RAW,")][0].split(",")[2:]
        data = [t for _, t in r.lines if t.startswith("D,RAW,")][0].split(",")[2:]
        self.assertEqual(len(header), len(data))
        self.assertEqual(header, config.firmware_raw_columns())


class CameraTests(unittest.TestCase):
    def test_homography_matches_projection(self):
        import numpy as np
        import camera
        cfg = config.load_config()
        g = camera.CameraGeometry(cfg.camera)
        for x, y in ((0, 0), (120, 0), (-10, 50), (60, -40)):
            u, v = g.project(x, y, 0.0)
            p = g.homography @ np.array([x, y, 1.0])
            self.assertAlmostEqual(float(u), p[0] / p[2], places=6)
            self.assertAlmostEqual(float(v), p[1] / p[2], places=6)
        # scale: about width / floor_width pixels per cm near the centre
        u0, _ = g.project(60, 10)
        u1, _ = g.project(70, 10)
        self.assertAlmostEqual(float(u1 - u0) / 10, cfg.camera.width / cfg.camera.floor_width_cm, delta=0.1)
        # floor +x is image +x, floor +y is image up
        _, va = g.project(60, 0)
        _, vb = g.project(60, 20)
        self.assertLess(float(vb), float(va))


class SessionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.out = tempfile.mkdtemp(prefix="simtest_")
        cls.sheet = os.path.join(cls.out, "sheet.csv")
        cols = simulate.SHEET_COLS
        rows = [dict(order=1, robot="Left", level="straight120",
                     commands="set straight_rpm 0 / set straight_cm 120 / run straight", target_x_cm=120),
                dict(order=2, robot="Left", level="L",
                     commands="set demo_route 1 / set demo_a_cm 30 / set demo_b_cm 30 / run demo", target_x_cm=0),
                dict(order=3, robot="Center", level="square1",
                     commands="set demo_route 3 / set square_cm 50 / set square_laps 1 / run demo", target_x_cm=0)]
        with open(cls.sheet, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            for r in rows:
                w.writerow(r)
        simulate.main(["--out", os.path.join(cls.out, "o"), "--sheet", cls.sheet, "--seed", "3", "--video",
                       "--fps", "10", "--kidnap", "2"])
        cls.o = os.path.join(cls.out, "o")

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.out, ignore_errors=True)

    def info(self):
        with open(os.path.join(self.o, "sim_info.json")) as f:
            return json.load(f)

    def read(self, *p):
        with open(os.path.join(self.o, *p), newline="") as f:
            return list(csv.DictReader(f))

    def test_files_and_columns(self):
        with open(os.path.join(config.REPO, "docs", "data-formats.md")) as f:
            doc = f.read()
        for name in ("raw", "runinfo", "param", "runend", "sync", "clock", "straight", "demo_step", "demo_end"):
            m = re.search(r"\| %s\.csv \| ([^|]+) \|" % name, doc)
            with open(os.path.join(self.o, "session", name + ".csv")) as f:
                header = f.readline().strip().split(",")
            self.assertEqual(header[:2], ["robot", "t_recv"])
            self.assertEqual(header[2:], m.group(1).split(" (")[0].strip().split(","), name)
        for p in ("session/clock_sync.csv", "session/session.json", "truth.csv", "measurements.csv",
                  "recording/video.avi", "recording/frames.csv", "recording/events.csv", "recording/meta.json",
                  "camera_truth.csv", "camera_detections.csv", "floor_calibration.json", "sim_info.json"):
            self.assertTrue(os.path.exists(os.path.join(self.o, p)), p)

    def test_run_ids_and_sheet(self):
        sheet = self.read("measurements.csv")
        self.assertEqual(len(sheet), 3)
        for r in sheet:
            self.assertRegex(r["run_id"], r"^[0-9A-F]{4}-\d+$")
            for k in ("xf0_cm", "yf0_cm", "xr0_cm", "yr0_cm", "xf_cm", "yf_cm", "xr_cm", "yr_cm"):
                float(r[k])
        self.assertIn("safe_lift 0", sheet[1]["commands"])   # the kidnapped run
        ends = {r["run"]: r for r in self.read("session", "runend.csv")}
        self.assertEqual(set(ends), {r["run_id"] for r in sheet})

    def test_truth_50hz_and_kidnap(self):
        truth = self.read("truth.csv")
        runs = {}
        for r in truth:
            runs.setdefault(r["run"], []).append(float(r["t_laptop"]))
        for ts in runs.values():
            gaps = [b - a for a, b in zip(ts, ts[1:])]
            self.assertAlmostEqual(sum(gaps) / len(gaps), 0.02, places=4)
        lifted = [r for r in truth if r["lifted"] == "1"]
        self.assertTrue(100 * 0.9 <= len(lifted) <= 100 * 1.1)              # ~2 s at 50 per second
        raw = self.read("session", "raw.csv")
        self.assertTrue(any(r["lifted"] == "1" for r in raw))

    def test_tape_marks_match_truth(self):
        info = self.info()
        sheet = {r["run_id"]: r for r in self.read("measurements.csv")}
        for run in info["runs"]:
            s = sheet[run["run"]]
            x = (float(s["xf_cm"]) + float(s["xr_cm"])) / 2
            y = (float(s["yf_cm"]) + float(s["yr_cm"])) / 2
            self.assertLess(math.hypot(x - run["end"][0], y - run["end"][1]), 0.6)

    def test_clock_sync_offsets(self):
        info = self.info()
        for r in self.read("session", "clock_sync.csv"):
            rb = info["robots"][r["robot"]]
            true_t = rb["t_boot"] + float(r["robot_ms"]) / 1000 / (1 + rb["clock_drift_ppm"] * 1e-6)
            mid = (float(r["t_send"]) + float(r["t_recv"])) / 2
            self.assertLess(abs(mid - true_t), float(r["rtt_ms"]) / 1000 / 2 + 0.002)

    def test_camera_truth_in_image(self):
        ct = self.read("camera_truth.csv")
        self.assertTrue(ct)
        for r in ct[::50]:
            self.assertTrue(0 <= float(r["x_px"]) < 640 and 0 <= float(r["y_px"]) < 480)
        frames = self.read("recording", "frames.csv")
        import cv2
        cap = cv2.VideoCapture(os.path.join(self.o, "recording", "video.avi"))
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        self.assertEqual(n, len(frames))


if __name__ == "__main__":
    unittest.main()
