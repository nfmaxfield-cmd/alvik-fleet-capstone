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

    def test_blink_pattern_from_name(self):
        cfg = perfect_config()
        for name, code in (("Left", 1), ("Center", 2), ("Right", 3), ("AL-3F2A", 4)):
            r = robot.Robot(name, cfg, random.Random(0), config.firmware_params(), "t", 0.0, 0x1234, "ip")
            self.assertEqual(r.blink_code_for(), code)
            lit = [robot.Robot.blink_slot_lit(k, code) for k in range(24)]
            self.assertEqual(lit[:code + 2], [True] + [False] * (code + 1))   # one flash per n + 2 slots
            self.assertEqual(lit[code + 2:], lit[:24 - code - 2])
            r.params["blink_code"] = 2
            self.assertEqual(r.blink_code_for(), 2)

    def test_blink_dark_during_library_block(self):
        r = make_robot(perfect_config())
        r.fw_yaw_sign = 1
        r.blink_start()
        r.wait_ms(305)                          # past the 300 ms lead: the first pulse is lit
        self.assertEqual(r.leds[-1][1], (1, 1, 1))
        k = len(r.leds)
        t_call = r.ms
        r.library_move(5)
        # nothing but "dark" may be recorded between the call and the end of the 200 ms block
        early = [(ms, left) for ms, left, _ in r.leds[k:] if ms < t_call + r.m.library_block_ms]
        self.assertTrue(early)
        self.assertEqual(early[-1][1], (0, 0, 0))

    def test_pulse_never_stretched_into_sync_flash(self):
        """The library's successful is_target_reached() blocks 100 ms with no hold() before it, so a
        pulse lit then is stretched. Since 2026-10-17g flashes are 100 ms with at least 200 ms gaps,
        so a stretched flash must stay well under the 300 ms sync flash and never reach the next one.
        Ends a 2 cm move at every point of the blink period, for codes 1 and 4."""
        cfg = perfect_config()
        for code in (1, 4):
            longest, rises = 0.0, []
            for start_ms in range(0, 1800, 7):
                r = make_robot(cfg, seed=start_ms)
                r.fw_yaw_sign = 1
                r.params["blink_code"] = code
                r.blink_start()
                r.wait_ms(2000 + start_ms)
                k = len(r.leds)
                r.library_move(2)
                r.wait_ms(400)
                on = None
                for ms, left, _ in r.leds[k - 1:]:
                    if max(left) > 0 and on is None:
                        on = ms
                    elif max(left) == 0 and on is not None:
                        longest, on = max(longest, ms - on), None
            self.assertLess(longest, 230.0, "code %d: longest blink pulse %.0f ms" % (code, longest))
            self.assertGreater(longest, 150.0)                 # the stretch does happen

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


def runs_of(o):
    with open(os.path.join(o, "sim_info.json")) as f:
        info = json.load(f)
    return info, info["runs"] + info["calib_runs"]


def raw_by_run(o):
    out = {}
    with open(os.path.join(o, "session", "raw.csv"), newline="") as f:
        for r in csv.DictReader(f):
            out.setdefault(r["run"], []).append(r)
    for v in out.values():
        v.sort(key=lambda r: int(r["ms"]))     # a delayed packet can arrive after later ones
    return out


def raw_gaps(info, raw_rows, robot_name, min_ms=80):
    """(t0, t1) laptop times of gaps in the RAW log: the robot was blocked inside the library
    (200 ms in move(), 100 ms in the successful is_target_reached())."""
    ms = [int(r["ms"]) for r in raw_rows]
    return [(laptop_t(info, robot_name, a), laptop_t(info, robot_name, b)) for a, b in zip(ms, ms[1:]) if b - a >= min_ms]


def laptop_t(info, robot_name, ms):
    rb = info["robots"][robot_name]
    return rb["t_boot"] + ms / 1000 / (1 + rb["clock_drift_ppm"] * 1e-6)


class CalibAndNudgeChecks:
    """Shared by the session test classes (needs self.o)."""

    def test_calib_run_per_robot(self):
        info, _ = runs_of(self.o)
        with open(os.path.join(self.o, "session", "runinfo.csv"), newline="") as f:
            ri = list(csv.DictReader(f))
        raw = raw_by_run(self.o)
        for name in info["robots"]:
            mine = sorted((int(r["ms_start"]), r) for r in ri if r["robot"] == name)
            exps = [r["experiment"] for _, r in mine]
            self.assertEqual(exps[0], "calib", name)            # before the robot's first sheet row
            self.assertEqual(exps.count("calib"), 1, name)      # once per power-on
            self.assertEqual(mine[0][1]["yaw_sign"], "0")       # not checked yet when it starts
            self.assertTrue(all(r["yaw_sign"] == "1" for _, r in mine[1:]))
            steps = [int(r["step"]) for r in raw[mine[0][1]["run"]]]
            first = [s for k, s in enumerate(steps) if k == 0 or s != steps[k - 1]]
            self.assertEqual(first, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], name)
            # the spins: about a full turn left (step 6), then back right (step 8), by the gyro;
            # the front distance zones are logged all the way round
            rows = raw[mine[0][1]["run"]]
            for st, sgn in ((6, 1), (8, -1)):
                seg = [r for r in rows if int(r["step"]) == st]
                yaw = [float(r["yaw_deg"]) for r in seg]
                turned = sum(((b - a + 180) % 360) - 180 for a, b in zip(yaw, yaw[1:]))
                self.assertAlmostEqual(sgn * turned, 359.0, delta=6.0, msg=(name, st))
                self.assertGreater(len(seg), 400, (name, st))          # ~11 s at 50 rows/s
                self.assertTrue(all(r["dC_cm"] not in ("", None) for r in seg))
        for c in info["calib_runs"]:
            self.assertEqual(c["result"], "done")
            # put down in view, off the start mark
            self.assertGreater(math.hypot(c["start"][0], c["start"][1]), 15)

    def test_start_nudge(self):
        info, _ = runs_of(self.o)
        raw = raw_by_run(self.o)
        with open(os.path.join(self.o, "truth.csv"), newline="") as f:
            truth = {}
            for r in csv.DictReader(f):
                truth.setdefault(r["run"], []).append((float(r["t_laptop"]), float(r["x_cm"]), float(r["y_cm"])))
        n = config.firmware_params()
        n = dict(n)["start_nudge_cm"]
        for run in info["runs"]:
            if run["exp"] not in ("demo", "straight", "turn"):
                continue
            rows = raw[run["run"]]
            steps = [int(r["step"]) for r in rows]
            self.assertIn(-1, steps, run["run"])
            k0, k1 = steps.index(-1), len(steps) - 1 - steps[::-1].index(-1)
            self.assertTrue(all(s == 0 for s in steps[:k0]))
            self.assertTrue(all(s == -1 for s in steps[k0:k1 + 1]))     # one block, before the route
            self.assertIn(steps[k1 + 1], (0, 1))      # demo's demoLog("start") counts 1 at once
            # the robot's own odometry goes out n cm and back
            xs = [float(r["x_cm"]) for r in rows[k0:k1 + 1]]
            self.assertAlmostEqual(max(xs) - xs[0], n, delta=0.3)
            self.assertLess(abs(xs[-1] - xs[0]), 0.3)
            # and so does the truth
            t0 = laptop_t(info, run["robot"], float(rows[k0]["ms"]))
            t1 = laptop_t(info, run["robot"], float(rows[k1]["ms"]))
            tr = [p for p in truth[run["run"]] if t0 - 0.05 <= p[0] <= t1 + 0.05]
            x0, y0 = tr[0][1], tr[0][2]
            h = math.radians(run["start"][2])
            along = [(x - x0) * math.cos(h) + (y - y0) * math.sin(h) for _, x, y in tr]
            self.assertAlmostEqual(max(along), n, delta=0.3)
            self.assertLess(abs(along[-1]), 0.3)
            self.assertLess(min(along), -0.0 + 0.3)


class SessionTests(CalibAndNudgeChecks, unittest.TestCase):
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
        calib = {r["run"] for r in self.read("session", "runinfo.csv") if r["experiment"] == "calib"}
        self.assertEqual(set(ends), {r["run_id"] for r in sheet} | calib)

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


class BlinkSessionTests(CalibAndNudgeChecks, unittest.TestCase):
    """Three robots (codes 1, 2, 3), short runs, the camera at 30 frames a second."""

    @classmethod
    def setUpClass(cls):
        cls.out = tempfile.mkdtemp(prefix="simblink_")
        sheet = os.path.join(cls.out, "sheet.csv")
        rows = [dict(order=1, robot="Left", level="straight30", commands="set straight_cm 30 / run straight"),
                dict(order=2, robot="Center", level="L", commands="set demo_route 1 / set demo_a_cm 15 / "
                                                                   "set demo_b_cm 10 / run demo"),
                dict(order=3, robot="Right", level="straight30", commands="set straight_cm 30 / run straight"),
                dict(order=4, robot="Left", level="straight30", commands="set straight_cm 30 / run straight")]
        with open(sheet, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=simulate.SHEET_COLS)
            w.writeheader()
            for r in rows:
                w.writerow(r)
        cfgp = os.path.join(cls.out, "cfg.json")
        with open(cfgp, "w") as f:
            json.dump(dict(session=dict(after_run_s=2.0, before_run_s=2.0, carry_s=2.0, cmd_spacing_s=0.4,
                                        calib_wait_s=1.0, after_calib_s=1.0),
                           camera=dict(hands_per_min=0.0, boxes_per_10min=0.0)), f)
        cls.o = os.path.join(cls.out, "o")
        simulate.main(["--out", cls.o, "--sheet", sheet, "--seed", "5", "--video", "--fps", "30",
                       "--config", cfgp])

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.out, ignore_errors=True)

    def lit_series(self):
        """Per run with blinking: (run info, [(t, led_on)]) from camera_truth, frame capture times
        on the laptop clock, between blink start and the end flash."""
        info, runs = runs_of(self.o)
        c = info["config"]["camera"]
        lag = c["pi_clock_offset_s"] + c["capture_delay_frames"] / c["fps"]   # t_wall - capture time, +- 2 ms
        with open(os.path.join(self.o, "camera_truth.csv"), newline="") as f:
            ct = list(csv.DictReader(f))
        out = []
        for run in runs:
            if run["blink_start"] is None:
                continue
            end_on = [s["ms_on"] for s in run["sync"] if s["tag"] == "end"][0]
            t_stop = laptop_t(info, run["robot"], end_on) - 0.15                  # blink::stop()
            ser = [(float(r["t_wall"]) - lag, int(r["led_on"])) for r in ct if r["robot"] == run["robot"]]
            ser = [(t, v) for t, v in ser if run["blink_start"] - 0.1 <= t < t_stop]   # dark before it
            out.append((run, ser, t_stop))
        return info, out

    def test_sim_info_blink_code(self):
        info, _ = runs_of(self.o)
        self.assertEqual({n: r["blink_code"] for n, r in info["robots"].items()}, dict(Left=1, Center=2, Right=3))

    def test_led_on_follows_code(self):
        """Identity is the flash period, (code + 2) x 100 ms. From camera_truth: the time between
        flash starts (rising edges of led_on) is one period (+-50 ms: 30 fps frames and tick
        latency) whenever the robot was not blocked in the library between them; a block can
        only drop flashes or start one late, never add one. The median period names the robot."""
        info, series = self.lit_series()
        raw = raw_by_run(self.o)
        per_robot = {}
        for run, ser, t_stop in series:
            period = (run["blink_code"] + 2) * 0.1
            gaps = raw_gaps(info, raw[run["run"]], run["robot"])
            rise_t = [t1 for (t0, p), (t1, q) in zip(ser, ser[1:]) if q and not p]
            self.assertGreaterEqual(len(rise_t), 5, run["run"])
            self.assertGreater(rise_t[0] - run["blink_start"], 0.28)          # 300 ms dark first
            for a, b in zip(rise_t, rise_t[1:]):
                d = b - a
                blocked = any(g0 < b and g1 > a - 0.12 for g0, g1 in gaps)   # a block just before a can start it late
                if blocked:
                    self.assertGreater(d, period - 0.11, (run["run"], d))
                else:
                    self.assertAlmostEqual(d, period, delta=0.05, msg=(run["run"], d))
                    per_robot.setdefault(run["robot"], []).append(d)
        for name, ds in per_robot.items():
            code = info["robots"][name]["blink_code"]
            self.assertAlmostEqual(sorted(ds)[len(ds) // 2], (code + 2) * 0.1, delta=0.02, msg=name)
            self.assertGreater(len(ds), 10, name)

    def test_no_pulse_as_long_as_sync_flash(self):
        """Blink flashes are 100 ms (3 frames at 30 fps), up to ~215 ms when the library's 100 ms
        reached wait stretches one; the sync flash is 300 ms (9-10 frames)."""
        info, series = self.lit_series()
        fps = info["config"]["camera"]["fps"]
        sync_frames = round(dict(config.firmware_params())["sync_ms"] / 1000 * fps)
        n_pulses, longest = 0, 0
        for run, ser, _ in series:
            n = 0
            for t, v in ser:
                if v:
                    n += 1
                    continue
                if n:
                    n_pulses += 1
                    longest = max(longest, n)
                n = 0
        self.assertGreater(n_pulses, 30)
        self.assertLessEqual(longest, round(0.23 * fps), longest)
        self.assertLess(longest, sync_frames - 1)
        # the sync flashes themselves are there, at full length
        with open(os.path.join(self.o, "camera_truth.csv"), newline="") as f:
            ct = [r for r in csv.DictReader(f) if r["robot"] == "Left"]
        n, best = 0, 0
        for r in ct:
            n = n + 1 if r["led_on"] == "1" else 0
            best = max(best, n)
        self.assertGreaterEqual(best, sync_frames - 1)

class MapBoardsTests(unittest.TestCase):
    def test_map_boards_reach_raw_zones(self):
        """simulate.py --map: a board 40 cm ahead shows in RAW dC (sensor 4.5 cm ahead of the axle)."""
        d = tempfile.mkdtemp()
        try:
            path = os.path.join(d, "map.json")
            with open(path, "w") as f:
                json.dump({"boards": [{"name": "A", "p1": [40.75, -20], "p2": [40.75, 20], "thickness_cm": 1.5}]}, f)
            cfg = perfect_config()
            cfg.tof.noise_cm = cfg.tof.noise_pct = 0.0
            cfg.tof.hz = 0                       # fresh reading from the pose set here
            r = make_robot(cfg)
            r.world_polys = simulate.map_polys(path)
            vals = r.raw_values(r.ms)
            self.assertAlmostEqual(vals["dC_cm"], 40.0 - 4.5, delta=1e-6)
            self.assertLess(vals["dL_cm"], 60.0)
        finally:
            shutil.rmtree(d)


if __name__ == "__main__":
    unittest.main()
