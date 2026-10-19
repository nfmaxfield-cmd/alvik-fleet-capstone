"""Home dock, nose switches and touch probe (firmware dock.h, switches.h) in the simulator.

    python -m unittest tools/sim/tests/test_dock.py -v        (about 2-3 minutes: 60 dockings)

The contact model and the switch numbers are GUESSES (config.py DockModel, fw_dock.py); the dock and bumper
geometry is hardware/dock/dock_params.py (robot numbers from Arduino's CAD model of the Alvik). These tests
check the firmware logic and what the geometry promises, not the real robot.
"""
import math
import os
import random
import statistics
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import config  # noqa: E402
import robot  # noqa: E402

DOCK = [100.0, 50.0, 0.0]          # back wall's inner face middle at (100, 50), docked robots face +x
SPREAD = {}                        # filled by test_twenty_dockings_per_robot, printed at the end


def make(name="Left", seed=0, docks=(DOCK,), contact_boards=False, **params):
    cfg = config.load_config()
    cfg.dock.docks = [list(d) for d in docks]
    cfg.dock.contact_boards = contact_boards
    r = robot.Robot(name, cfg, random.Random(seed), config.firmware_params(), "test", 0.0, 0x1234 + seed, "ip")
    r.idle_until(10.0)
    r.fw_yaw_sign = 1                       # compass already checked
    r.params.update(start_nudge_cm=0, **params)
    return r


def place_before_dock(r, gap_cm, lat_cm, th_deg):
    """Robot's bumper face gap_cm in front of the dock mouth, lat_cm to the side, turned th_deg."""
    dm = r.cfg.dock
    mouth = DOCK[0] - dm.pocket_depth_cm - dm.funnel_len_cm
    r.x = mouth - gap_cm - dm.bump_ahead_cm * math.cos(math.radians(th_deg))
    r.y = DOCK[1] + lat_cm
    r.th = math.radians(th_deg)


def docked_error(r):
    """True pose at the docking minus where the dock says it is: (along cm, sideways cm, heading deg)."""
    dm = r.cfg.dock
    x, y, th = r.docks_done[-1]["true"]
    return x - (DOCK[0] - dm.bump_ahead_cm), y - DOCK[1], math.degrees(th)


def records(r, kind):
    cols = None
    out = []
    for _, line in r.lines:
        if line.startswith("H," + kind + ","):
            cols = line.split(",")[2:]
        elif line.startswith("D," + kind + ",") and cols:
            out.append(dict(zip(cols, line.split(",")[2:])))
    return out


class DockTests(unittest.TestCase):
    def test_off_by_default(self):
        P = dict(config.firmware_params())
        self.assertEqual(P["sw_on"], 0.0)
        r = make()
        place_before_dock(r, 3.0, 0.0, 0.0)
        run = r.run_experiment("dock")
        self.assertTrue(run["result"].startswith("failed:nose switches not configured"), run["result"])
        raw = records(r, "RAW")
        self.assertTrue(raw and all(x["swL"] == "-1" and x["swR"] == "-1" for x in raw))
        self.assertEqual(records(r, "DOCK")[0]["result"], "nose switches not configured (sw_on 0)")
        self.assertEqual(records(r, "RAW")[-1]["raw_n"], str(len(raw) - 1))   # raw_n still last and counting

    def test_twenty_dockings_per_robot(self):
        """20 dockings per robot from hand-placed starts inside the funnel's reach: every one docks, and the
        true pose at the docking (what the robot's pose is reset to) spreads by about the pocket's play."""
        for name, seed in (("Left", 11), ("Center", 22), ("Right", 33)):
            rng = random.Random(seed)
            errs = []
            r = make(name, seed, sw_on=1, dock_back_cm=0, dock_still_ms=300, settle_end_ms=100)
            for k in range(20):
                place_before_dock(r, rng.uniform(1.0, 8.0), rng.uniform(-1.0, 1.0), rng.uniform(-8.0, 8.0))
                run = r.run_experiment("dock")
                self.assertEqual(run["result"], "done", (name, k, records(r, "DOCK")[-1]))
                errs.append(docked_error(r))
            a, s, h = zip(*errs)
            SPREAD[name] = dict(along_sd_mm=10 * statistics.pstdev(a), side_sd_mm=10 * statistics.pstdev(s),
                                heading_sd_deg=statistics.pstdev(h), along_max_mm=10 * max(map(abs, a)),
                                side_max_mm=10 * max(map(abs, s)), heading_max_deg=max(map(abs, h)))
            self.assertLess(max(map(abs, a)), 0.03)      # along the axis: within 0.3 mm
            self.assertLess(max(map(abs, s)), 0.08)      # sideways: the pocket's 0.5 mm each side + a tilted seat
            self.assertLess(max(map(abs, h)), 0.4)       # heading
        print("\nsimulated docking spread (true pose at docking vs the dock's pose):")
        for name, v in SPREAD.items():
            print("  %-6s along sd %.2f mm (max %.2f), sideways sd %.2f mm (max %.2f), heading sd %.2f deg (max %.2f)"
                  % (name, v["along_sd_mm"], v["along_max_mm"], v["side_sd_mm"], v["side_max_mm"],
                     v["heading_sd_deg"], v["heading_max_deg"]))

    def test_crooked_approach_ends_square(self):
        r = make(sw_on=1, dock_back_cm=0, dock_still_ms=300)
        place_before_dock(r, 4.0, 0.8, 10.0)
        self.assertEqual(r.run_experiment("dock")["result"], "done")
        d = records(r, "DOCK")[0]
        self.assertEqual(d["result"], "docked")
        self.assertIn(d["first"], ("L", "R"))
        along, side, head = docked_error(r)
        self.assertLess(abs(head), 0.4)
        self.assertLess(abs(side), 0.06)
        self.assertLess(abs(along), 0.03)

    def test_outside_the_funnel_fails_safely(self):
        r = make(sw_on=1, dock_back_cm=0, dock_max_cm=12)
        place_before_dock(r, 3.0, 9.0, 0.0)          # beside the dock: misses it entirely
        run = r.run_experiment("dock")
        self.assertTrue(run["result"].startswith("failed:no click within 12 cm"), run["result"])

    def test_stripe_reading_matches_the_geometry(self):
        """The centre line sensor reaches the stripe's far edge stripe_x0 + (axle-to-sensor) before the axle's
        docked position: travel from the stripe to the docking is fixed by the dock."""
        r = make(sw_on=1, dock_back_cm=0, dock_still_ms=300, safe_edge=1, edge_dir=1, edge_thr=400)
        place_before_dock(r, 3.0, 0.0, 0.0)
        self.assertEqual(r.run_experiment("dock")["result"], "done")       # the edge guard took no notice
        d = records(r, "DOCK")[0]
        dm, lm = r.cfg.dock, r.cfg.line
        expect = (-dm.bump_ahead_cm) - (dm.stripe_x0_cm - lm.sensor_forward_cm)   # axle travel stripe -> docked
        got = float(d["travel_cm"]) - float(d["stripe_c_cm"])
        self.assertAlmostEqual(got, expect, delta=0.15)

    def test_touch_gives_the_board_distance(self):
        """Touch a board 20 times from 3 cm: the travel at the click + the bumper geometry = the board's
        distance to better than 0.5 mm."""
        board = [[120.0, 30.0], [121.5, 30.0], [121.5, 70.0], [120.0, 70.0]]
        r = make(sw_on=1, touch_n=20, touch_back_cm=1.0, contact_boards=True)
        r.world_polys = [board]
        dm = r.cfg.dock
        r.x, r.y, r.th = 120.0 - 3.0 - dm.bump_ahead_cm, 50.0, 0.0
        self.assertEqual(r.run_experiment("touch")["result"], "done")
        t = records(r, "TOUCH")
        self.assertEqual(len(t), 20)
        for rec, tr in zip(t, r.touches):
            self.assertEqual(rec["result"], "touched")
            # the face the robot found, from where it started this creep (true start = true end - true travel)
            x, y, th = tr["true"]
            lat = dm.bump_half_cm if tr["first"] == 0 else -dm.bump_half_cm
            lever_x = x + dm.bump_ahead_cm * math.cos(th) - lat * math.sin(th)
            gap = 120.0 - lever_x                                   # at the lever that clicked, once braked
            self.assertTrue(-0.001 <= gap <= dm.sw_gap_cm + 0.001, gap)   # stopped short of a bump
        first = float(t[0]["travel_cm"])
        self.assertAlmostEqual(first + r.P("sw_gap_cm"), 3.0, delta=0.05)   # the 3 cm placed, to < 0.5 mm
        sd = statistics.pstdev(float(x["travel_cm"]) for x in t[1:])
        self.assertLess(sd, 0.02)

    def test_follow_dock_step_resets_the_pose(self):
        """Follow step K: the own pose is reset from the dock (to ~0.5 mm) even when it was 1 cm off."""
        r = make(sw_on=1, dock_still_ms=1000)
        place_before_dock(r, 4.0, 0.3, 3.0)
        r.pending = -1
        cmds = ["fpose %.2f %.2f %.2f 0.5 2" % (r.x + 1.0, r.y - 0.7, math.degrees(r.th) + 2.0),
                "fs 1 K %.2f %.2f %.2f 15 0" % (DOCK[0], DOCK[1], DOCK[2])]
        queue = list(cmds)

        class Radio:
            def poll(self, rb):
                return queue.pop(0) if (queue and rb.f_active) else None

            def transmit(self, rb, text):
                pass
        r.radio = Radio()
        r.params.update(follow_quiet_ms=1e9)
        orig = r.should_abort

        def abort_when_done():
            if r.millis() > 120000:
                r.abort_reason = "test timeout"
            if any("D,STEP" in l and ",1,done," in l for _, l in r.lines) or \
                    any("D,STEP" in l and ",1,failed," in l for _, l in r.lines):
                r.abort_reason = r.abort_reason or "stop command"
            return orig()
        r.should_abort = abort_when_done
        r.run_experiment("follow")
        self.assertTrue(any(l.startswith("D,STEP") and ",done," in l and "docked" in l for _, l in r.lines))
        x, y, th = r.docks_done[-1]["true"]
        d = records(r, "DOCK")[0]
        self.assertEqual(d["frame"], "floor")
        self.assertEqual(d["drift_used"], "1")
        # the pose the robot set is the dock's, and the robot really is there
        self.assertAlmostEqual(r.pose.x, DOCK[0] - r.P("bump_ahead_cm"), delta=0.05)
        self.assertAlmostEqual(x, DOCK[0] - r.cfg.dock.bump_ahead_cm, delta=0.03)
        self.assertAlmostEqual(r.pose.y, y, delta=0.06)



class TestGeometryAgrees(unittest.TestCase):
    """The bracket and dock numbers are written in several places; they must all be dock_params.py's."""

    def test_firmware_fusion_fleet_and_site_defaults_match_dock_params(self):
        dm = config.load_config().dock
        fw = dict(config.firmware_params())
        repo = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
        for k in ("bump_ahead_cm", "bump_half_cm", "sw_gap_cm"):
            self.assertAlmostEqual(fw[k], getattr(dm, k), delta=0.006, msg="params.h " + k)
        for sub in ("fusion", "fleet", "site"):
            sys.path.append(os.path.join(repo, "tools", sub))
        import engine
        import fleetmap
        import planner
        import solve
        for k in ("bump_ahead_cm", "bump_half_cm", "sw_gap_cm"):
            self.assertAlmostEqual(engine.DEFAULT[k], getattr(dm, k), delta=0.006, msg="fusion engine " + k)
            self.assertAlmostEqual(solve.DEFAULTS[k], getattr(dm, k), delta=0.006, msg="site solve " + k)
        self.assertAlmostEqual(planner.BUMP_AHEAD_CM, dm.bump_ahead_cm, delta=0.006)
        self.assertAlmostEqual(fleetmap.HOME_DOCK_DEFAULTS["pocket_w_cm"], dm.pocket_w_cm, delta=0.006)
        self.assertAlmostEqual(fleetmap.HOME_DOCK_DEFAULTS["pocket_depth_cm"], dm.pocket_depth_cm, delta=0.006)
        self.assertAlmostEqual(fleetmap.BUMPER_W_FITTED_CM, dm.bumper_w_cm, delta=0.006)
        # the pocket fits the bracket with the pocket's play, and the bracket is wider than the robot
        self.assertAlmostEqual(dm.pocket_w_cm - dm.bumper_w_cm, 0.1, delta=0.001)
        self.assertGreater(dm.bumper_w_cm, 9.6)


if __name__ == "__main__":
    unittest.main()
