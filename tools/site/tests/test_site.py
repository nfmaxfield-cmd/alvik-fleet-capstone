"""Tests for the site setup (tools/site): the joint solver, the audit, the locked map file, the change check.

    python -m pytest tools/site/tests -q
"""
import json
import math
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SITE = os.path.dirname(HERE)
sys.path.insert(0, SITE)

import sitemap as SM  # noqa: E402
import observe as OB  # noqa: E402
import solve as SO  # noqa: E402
import change as CH  # noqa: E402
import fleetmap as FM  # noqa: E402
import landmarks as LM  # noqa: E402

D2R = math.pi / 180

BASE_MAP = {"format": "alvik-fleet-map", "version": 1, "floor": {"x_min": 0, "x_max": 122, "y_min": 0, "y_max": 122},
            "boards": [{"name": "A", "p1": [100, 20], "p2": [100, 60], "thickness_cm": 1.5},
                       {"name": "B", "p1": [30, 100], "p2": [80, 100], "thickness_cm": 1.5}],
            "robots": [{"name": "Left", "start": [50, 40, 0]}]}


def moved_map(m, name, dx, dy):
    m = json.loads(json.dumps(m))
    for b in m["boards"]:
        if b["name"] == name:
            b["p1"] = [b["p1"][0] + dx, b["p1"][1] + dy]
            b["p2"] = [b["p2"][0] + dx, b["p2"][1] + dy]
    return m


def zone_readings(pose, faces, offs=(0, 0, 0, 0, 0), bias=0.0, scale=1.0, rng=None, sd=0.0):
    """What the five zones read at pose (x, y, deg): one ray each, the first face hit."""
    cfg = SO.ENG.config()
    fwd = cfg["tof_forward_cm"] - cfg["tof_recess_cm"]
    th = pose[2] * D2R
    p = np.array([pose[0] + fwd * math.cos(th), pose[1] + fwd * math.sin(th)])
    out = []
    for zi, a in enumerate(cfg["tof_zone_deg"]):
        ang = th + (a + offs[zi]) * D2R
        hit = LM.nearest(p, (math.cos(ang), math.sin(ang)), faces)
        if hit is None or hit[0] > 120:
            out.append(None)
            continue
        v = hit[0] * scale + bias + (rng.normal(0, sd) if rng is not None else 0.0)
        out.append((v, 20, 0.3))
    return out


def synthetic_stations(true_map, poses, rng, robot="Left", offs=(0, 0, 0, 0, 0), bias=0.0, scale=1.0, cam_sd=0.05):
    faces = LM.faces_from_map(FM.normalize(true_map))
    out = []
    for k, ps in enumerate(poses):
        st = OB.Station(robot, "R1", float(k), float(k) + 1)
        st.cam = (ps[0] + rng.normal(0, cam_sd), ps[1] + rng.normal(0, cam_sd))
        st.cam_th = ps[2] + rng.normal(0, 0.3)
        st.n_cam = 20
        st.zones = zone_readings(ps, faces, offs, bias, scale, rng, 0.1)
        st.odo = (ps[0], ps[1], ps[2])
        st.yaw = ps[2]
        st.idx = k
        out.append(st)
    return out


def poses_facing(board_x0=99.25, ys=(28, 40, 52), dists=(15, 25, 35)):
    """Robot poses looking at board A's left face (x = 99.25) from several distances and three headings."""
    out = []
    for y in ys:
        for d in dists:
            for h in (-12.0, 0.0, 12.0):
                out.append((board_x0 - d - 4.5, y, h))
    return out


class SolverTests(unittest.TestCase):
    def test_recovers_a_moved_board(self):
        """The clicks put board A 1 cm off in x and 0.6 cm in y; ranging it from many poses (with the
        camera's pose of the robot, and the range bias known from the bench rows) brings it back to within
        0.15 cm across the face (x)."""
        rng = np.random.default_rng(1)
        true = BASE_MAP
        clicked = moved_map(BASE_MAP, "A", 1.0, 0.6)
        sts = synthetic_stations(true, poses_facing(), rng)
        m = FM.normalize(clicked)
        L = SM.landmarks_from_map(m, prior_sigma_cm=1.0)
        E = SM.edges_from_map(m, [])
        prob = SO.Problem(L, E, sts, OB.odometry_links(sts), ["Left"], opts=dict(corners=False, tof_zone_width_deg=0.0,
                                                                                   width_prior_deg=0.01),
                          robot_priors={"Left": dict(bias_cm=0.0, bias_sd_cm=0.05)})
        res = SO.solve(prob)
        dx = res.est.g["A"][0]
        self.assertAlmostEqual(dx, -1.0, delta=0.15)
        # along the board (y) ranging the long face says nothing: the prior (clicks) stays, and its sd says so
        self.assertGreater(math.sqrt(res.est.cov_g["A"][1, 1]), 0.5)
        self.assertLess(math.sqrt(res.est.cov_g["A"][0, 0]), 0.15)

    def test_range_bias_and_board_trade_off_without_a_prior(self):
        """A range bias looks exactly like the board being further away: with a tight bench prior on the bias the
        board comes out right; the uncertainty across the face is larger without it."""
        rng = np.random.default_rng(2)
        sts = synthetic_stations(BASE_MAP, poses_facing(), rng, bias=0.4)
        m = FM.normalize(BASE_MAP)
        L = SM.landmarks_from_map(m)
        E = SM.edges_from_map(m, [])
        o = dict(corners=False, tof_zone_width_deg=0.0, width_prior_deg=0.01)
        loose = SO.solve(SO.Problem(L, E, sts, OB.odometry_links(sts), ["Left"], opts=o))
        tight = SO.solve(SO.Problem(L, E, sts, OB.odometry_links(sts), ["Left"], opts=o,
                                    robot_priors={"Left": dict(bias_cm=0.4, bias_sd_cm=0.05)}))
        self.assertAlmostEqual(tight.est.g["A"][0], 0.0, delta=0.1)
        self.assertGreater(math.sqrt(loose.est.cov_g["A"][0, 0]), 2 * math.sqrt(tight.est.cov_g["A"][0, 0]))

    def test_tape_measurement_is_merged(self):
        """A tape-measured corner pins the board along its length."""
        rng = np.random.default_rng(3)
        clicked = moved_map(BASE_MAP, "A", 0.0, 1.5)
        sts = synthetic_stations(BASE_MAP, poses_facing(), rng)
        m = FM.normalize(clicked)
        L = SM.landmarks_from_map(m)
        E = SM.edges_from_map(m, [])
        truth = {l.name: l for l in SM.landmarks_from_map(FM.normalize(BASE_MAP))}
        tape = [("A", "left p1", truth["A"].anchors["left p1"]), ("A", "left p2", truth["A"].anchors["left p2"])]
        res = SO.solve(SO.Problem(L, E, sts, OB.odometry_links(sts), ["Left"],
                                  opts=dict(corners=False, tof_zone_width_deg=0.0, width_prior_deg=0.01), tape=tape,
                                  robot_priors={"Left": dict(bias_cm=0.0, bias_sd_cm=0.05)}))
        self.assertAlmostEqual(res.est.g["A"][1], -1.5, delta=0.1)


class AuditTests(unittest.TestCase):
    def points(self):
        return [dict(key=f"{n}: {a}", landmark=n, anchor=a, xy=np.array([0.0, 0.0]), sigma=s, kind="board", cov=None)
                for n, a, s in (("A", "p1", 0.4), ("A", "p2", 0.35), ("B", "p1", 0.3), ("C", "p1", 0.1),
                                ("D", "p1", 0.12), ("E", "p1", 0.11))]

    def test_largest_uncertainty_from_different_landmarks_plus_one_random(self):
        pick = SM.pick_audit(self.points(), n_worst=2, n_random=1, seed=4)
        self.assertEqual([p["key"] for p in pick[:2]], ["A: p1", "B: p1"])
        self.assertEqual(pick[2]["why"], "random check")
        self.assertNotIn(pick[2]["landmark"], ("A", "B"))

    def test_first_setup_audits_every_landmark(self):
        pts = self.points() + [dict(key="edge x0: 50%", landmark="edge x0", anchor=0.5, xy=np.zeros(2), sigma=0.2,
                                    kind="edge", cov=None)]
        pick = SM.pick_audit(pts, n_worst=99, seed=3)
        self.assertEqual(sorted(p["landmark"] for p in pick), ["A", "B", "C", "D", "E", "edge x0"])
        self.assertIn("A: p1", [p["key"] for p in pick])            # each landmark's least sure point

    def test_exclude_points_used_as_data(self):
        pick = SM.pick_audit(self.points(), seed=1, exclude={"A: p1"})
        self.assertNotIn("A: p1", [p["key"] for p in pick])

    def test_pass_rule(self):
        self.assertAlmostEqual(SM.audit_allowed(0.0), 0.3)              # the floor: the tape can't do better
        self.assertAlmostEqual(SM.audit_allowed(0.4), 3 * math.hypot(0.4, 0.1))
        p = dict(kind="board", xy=np.array([10.0, 10.0]), sigma=0.1)
        self.assertTrue(SM.audit_compare(p, (10.2, 10.1))["passed"])
        self.assertFalse(SM.audit_compare(p, (10.5, 10.0))["passed"])     # allowed 3 x hypot(0.1, 0.1) = 0.42
        e = dict(kind="edge", xy=np.array([0.0, 30.0]), normal=np.array([-1.0, 0.0]), sigma=0.1)
        c = SM.audit_compare(e, (0.2, 37.0))                               # along the edge does not count
        self.assertAlmostEqual(c["error_cm"], 0.2)

    def test_fill_in_takes_the_worst_point_per_landmark_above_target(self):
        need = SM.pick_fill_in(self.points(), target_cm=0.32)
        self.assertEqual([p["key"] for p in need], ["A: p1"])


class LockTests(unittest.TestCase):
    def test_write_read_and_edit_invalidates(self):
        d = tempfile.mkdtemp()
        try:
            m = FM.normalize(BASE_MAP)
            L = SM.landmarks_from_map(m)
            E = SM.edges_from_map(m, ["x0"])
            est = SM.Estimate(L, E, g={"A": np.array([0.5, -0.2, 0.3, 0.1]), "B": np.zeros(4)}, cam_sys_cm=0.15)
            audit = [dict(round=1, passed=True, points=[])]
            lm = SM.locked_map(m, est, SM.next_version(d), audit, SM.camera_info(None), dict(note="test"),
                               robots={"Left": dict(zone_offset_deg=[0] * 5, range_bias_cm=0.1, range_scale=1.0)},
                               sensor=dict(zone_width_deg=7.0))
            SM.write_lock(d, lm, est)
            m2, site = SM.read_lock(d)
            self.assertIsNotNone(site)
            self.assertEqual(site["version"], 1)
            self.assertEqual(site["format"], SM.SITE_FORMAT)
            self.assertIn("sigma_cm", site["landmarks"]["A"])
            self.assertAlmostEqual(m2["boards"][0]["p1"][0], est.anchor("A", "left p1")[0]
                                   + (m2["boards"][0]["p1"][0] - est.anchor("A", "left p1")[0]), places=6)
            self.assertEqual(SM.next_version(d), 2)
            # the fleet map reader still reads it (the site block is ignored)
            FM.normalize(m2)
            # landmarks.csv: the fusion reads it; sigma columns are extra
            faces = LM.load(os.path.join(d, "landmarks.csv"))
            self.assertEqual(len(faces), 8)
            with open(os.path.join(d, "landmarks.csv")) as f:
                self.assertIn("sigma_cm", f.readline())
            # an edit after locking (e.g. a board dragged in the fleet page) unlocks it
            m2["boards"][0]["p1"][0] += 1.0
            with open(os.path.join(d, "map.json"), "w") as f:
                json.dump(m2, f)
            self.assertIsNone(SM.read_lock(d)[1])
        finally:
            shutil.rmtree(d)

    def test_score_against_tape(self):
        """score.py: a truth file equal to the map scores 0; edges count across the edge only."""
        import score as SC
        d = tempfile.mkdtemp()
        try:
            m = FM.normalize(BASE_MAP)
            est = SM.Estimate(SM.landmarks_from_map(m), SM.edges_from_map(m, ["x0"]), cam_sys_cm=0.1)
            SM.write_lock(d, SM.locked_map(m, est, 1, [], SM.camera_info(None), {}), est)
            pts = SC.map_points(os.path.join(d, "map.json"))
            truth = {k: v[0] + (np.array([0.0, 3.0]) if k.startswith("edge") else 0.0) for k, v in pts.items()}
            rows = SC.score(pts, truth)
            self.assertEqual(len(rows), 8 + 3)
            self.assertLess(max(r["error_cm"] for r in rows), 1e-6)
        finally:
            shutil.rmtree(d)

    def test_board_stretch(self):
        m = FM.normalize(BASE_MAP)
        a = SM.landmarks_from_map(m)[0]
        p = a.move(a.anchors["left p2"], np.array([0.0, 0.0, 0.0, 1.0]))
        self.assertAlmostEqual(p[1] - a.anchors["left p2"][1], 0.5)      # each end half the stretch out


class ChangeCheckTests(unittest.TestCase):
    def make_lock(self, d):
        m = FM.normalize(BASE_MAP)
        L = SM.landmarks_from_map(m)
        est = SM.Estimate(L, SM.edges_from_map(m, []), cam_sys_cm=0.0)
        for n in est.cov_g:
            est.cov_g[n] = np.diag([0.01, 0.01, 0.01, 0.01])
        lm = SM.locked_map(m, est, 1, [], SM.camera_info(None), {}, robots={}, sensor={})
        SM.write_lock(d, lm, est)
        return os.path.join(d, "map.json")

    def run_check(self, world, n=80):
        d = tempfile.mkdtemp()
        try:
            cc = CH.ChangeCheck(self.make_lock(d), say=None)
            faces = LM.faces_from_map(FM.normalize(world))
            rng = np.random.default_rng(5)
            flags = []
            for k in range(n):
                pose = (70.0, 30.0 + 20 * (k % 3), 0.0)
                z = [None if v is None else v[0] for v in zone_readings(pose, faces, rng=rng, sd=0.3)]
                flags += cc.feed("Left", 0.2 * k, pose, z, 0.2)
            return flags
        finally:
            shutil.rmtree(d)

    def test_moved_board_is_flagged(self):
        flags = self.run_check(moved_map(BASE_MAP, "A", 1.5, 0.0))
        self.assertEqual([f["landmark"] for f in flags], ["A"])
        self.assertGreater(flags[0]["median_resid_cm"], 1.0)

    def test_unmoved_board_is_not_flagged(self):
        self.assertEqual(self.run_check(BASE_MAP), [])

    def test_unsure_pose_is_not_used(self):
        d = tempfile.mkdtemp()
        try:
            cc = CH.ChangeCheck(self.make_lock(d), say=None)
            self.assertEqual(cc.feed("Left", 0.0, (70, 30, 0), [10, 10, 10, 10, 10], pose_sd=2.0), [])
            self.assertEqual(len(cc.hist), 0)
        finally:
            shutil.rmtree(d)


class SimulatedSurveyTests(unittest.TestCase):
    def test_short_survey_on_the_simulator(self):
        """End to end on the simulator, one robot, a few spots and one edge: the plans run on the firmware's
        follower copy, stations come out, the solve runs, the edge guard trips at the open edge."""
        import pipeline as PL
        import simrun as SR
        import sim_eval as SE
        world = json.load(open(SE.WORLD))
        tm = FM.normalize(world)
        user = PL.clicked_map(world, 1.0, seed=2, edge_sigma_cm=0.0)   # the edge clicked right: the guard must trip
                                                                       # before the planned stop 3 cm inside it
        site = SR.SimSite(tm, SE.sim_config(None), seed=2, board_cm=[0, 0, 122, 122], open_edges=["x0"])
        E = SM.edges_from_map(FM.normalize(user), ["x0"])
        opts = dict(max_edge_points=1)
        done, trips, t = PL.run_sim_survey(site, user, ["Left"], E, opts=opts, log=lambda *_: None, max_spots=3)
        self.assertGreaterEqual(len(done["Left"]), 2)
        self.assertEqual([tr["tripped"] for tr in trips["Left"]], [True])
        runs = site.runs("Left")
        res = PL.solve_site(user, ["x0"], runs, site.sightings, ["Left"])
        self.assertGreater(res.summary()["stations"], 8)
        self.assertEqual(res.summary()["edge_trips"], 1)
        # one trip keeps the edge near the truth (the edge's prior is 1 cm)
        err = PL.map_errors(res.est, world)["edge x0"]
        self.assertLess(err, 0.6)
        # without the camera's heading (the detector often has none): the start headings come from the gyro and
        # the straight drives, and the stations still come out facing the right way
        sight = {r: [(t, x, y, None) for t, x, y, _ in v] for r, v in site.sightings.items()}
        sts = OB.stations(runs, sight)
        SO.heading_guesses(sts)
        withh = OB.stations(runs, site.sightings)
        diffs = [abs((a.heading_guess - b.cam_th + 180) % 360 - 180) for a, b in zip(sts, withh)]
        self.assertLess(float(np.median(diffs)), 5.0)


if __name__ == "__main__":
    unittest.main()
