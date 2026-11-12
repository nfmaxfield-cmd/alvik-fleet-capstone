"""e10 (Phase C, 7 Nov): discovery at the start, the proven workspace, the fixes (the filter's hang case, the camera
watch's false alarms, 95 % uncertainty), the preferences page and quiz."""
import json
import math
import os
import random
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
for p in (FLEET, os.path.dirname(FLEET)):
    if p not in sys.path:
        sys.path.insert(0, p)

import freeroam_core as FC        # noqa: E402
import fused_pose as FP           # noqa: E402
import proven_workspace as PW     # noqa: E402
import start_discovery as D       # noqa: E402
import tracking_layer as TL       # noqa: E402

HOME = (0.0, 0.0, 122.24, 121.92)


# ------------------------------------------------------------------------------------------------- the workspace (C2)
def test_a_fully_seen_board_keeps_the_rectangle_and_never_says_more_room_than_there_is():
    ws = PW.ProvenWorkspace(HOME)
    ws.see()
    for x, y in ((61.0, 61.0), (20.0, 61.0), (61.0, 10.0), (110.0, 100.0), (7.0, 7.0)):
        d = ws.edge_dist(x, y)
        r = min(x - HOME[0], HOME[2] - x, y - HOME[1], HOME[3] - y)
        assert r - 2.6 <= d <= r + 0.01            # conservative by at most a cell
    assert ws.edge_dist(-5.0, 50.0) < 0
    assert FC.edge_dist(ws, 61.0, 61.0) == pytest.approx(ws.edge_dist(61.0, 61.0))


def test_unseen_ground_is_not_workspace_until_felt_and_a_drop_is_never_proven_again(tmp_path):
    ws = PW.ProvenWorkspace(HOME)
    ws.see(cover=[(-30.0, -30.0, 85.0, 160.0)])
    assert ws.edge_dist(84.0, 60.0) < 2.0 and not ws.proven(100.0, 60.0)
    assert ws.felt_strip(80.0, 60.0, 100.0, 60.0, half_w=4.0) > 0
    assert ws.proven(95.0, 60.0)
    ws.drop_at(95.0, 60.0, 0.0, reach_cm=4.0)
    assert not ws.proven(100.0, 60.0)
    ws.felt_strip(95.0, 60.0, 105.0, 60.0, half_w=4.0)
    assert not ws.proven(100.0, 60.0)          # a drop stays a drop
    cells = ws.provisional(85.0, 40.0, 100.0, 40.0, 10.0)
    assert ws.edge_dist(95.0, 40.0) > 0 and not ws.proven(95.0, 40.0)   # the planner may go; nothing is proven yet
    ws.clear_provisional(cells)
    assert ws.edge_dist(95.0, 40.0) < 0
    p = str(tmp_path / "home.json")
    ws.save(p)
    ws2 = PW.ProvenWorkspace(HOME)
    assert "added" in ws2.merge_saved(p) and ws2.proven(90.0, 60.0)
    ws3 = PW.ProvenWorkspace((0.0, 0.0, 110.0, 121.92))
    assert "another board" in ws3.merge_saved(p)


def test_the_controller_plans_inside_the_proven_workspace():
    import freeroam_fastsim as S
    sim = S.Sim(3, 0.03, dead_robot=False, pi_freeze=False, edge_physics=True, bottom_zone=True, dropouts=False,
                workspace=True, cover=[(-30.0, -30.0, 85.0, 160.0)],
                starts={"Left": (40.0, 90.0, 0.0), "Center": (62.0, 40.0, 90.0), "Right": (25.0, 30.0, 45.0)})
    r = sim.run()
    assert r["contacts"] == 0 and r["falls"] == 0
    # nobody's middle goes where nothing is proven (x > 85 less the derived clearance)
    xs = [int(k.split(",")[0]) * 5 for k in r["phase_c"]["visits"]]
    assert max(xs) < 85


# ------------------------------------------------------------------------------------------------- discovery (C1)
def disc(seed=1, **kw):
    import freeroam_fastsim as S
    base = dict(dead_robot=False, pi_freeze=False, sensors=True, fw="2026-11-05d", edge_physics=True,
                bottom_zone=True, dropouts=False, camera="detections", scene=dict(tracking="lights"), layer_kw={},
                ctrl_kw=dict(pose_source="blend"), discover=True, workspace=True)
    base.update(kw)
    sim = S.Sim(seed, 0.03, **base)
    return sim, sim.discover()


def test_discovery_passes_a_right_camera_and_catches_a_mirrored_one():
    sim, res = disc(1)
    assert res["camera"] == "ok" and res["calibration"] == "as configured" and abs(res["scale"] - 1.0) < 0.05
    sim, res = disc(1, cam_fault=dict(mirror=True))
    assert res["camera"] == "wrong name" and res["calibration"] == "mirrored"
    x, y = sim.cam_map(30.0, 40.0)                 # the chosen calibration undoes the fault
    assert abs(x - 30.0) < 2.0 and abs(y - 40.0) < 2.0


def test_discovery_finds_the_boards_size_turned_tags_and_the_clock():
    sim, res = disc(2, cam_fault=dict(scale=1.1))
    assert 1.06 < res["scale"] < 1.14 and res["board_found"][2] < 128.0
    sim, res = disc(3, scene=dict(tracking="tags"), tag_yaw={"Left": 90.0, "Right": 180.0}, cam_clock=0.7)
    assert res["tag_yaw"].get("Left") == pytest.approx(-90.0) and abs(res["tag_yaw"].get("Right")) == pytest.approx(180)
    assert 0.45 < res["clock_s"] < 0.95


def test_no_camera_narrows_the_run_and_nothing_crosses_an_edge():
    import freeroam_fastsim as S
    sim = S.Sim(1, 0.03, dead_robot=False, pi_freeze=False, sensors=True, edge_physics=True, bottom_zone=True,
                dropouts=False, camera="detections", scene=dict(tracking="lights", fault="none"), layer_kw={},
                discover=True, workspace=True)
    r = sim.run()
    assert r["phase_c"]["discovery"]["camera"] == "missing"
    assert r["edge_min_body_cm"] > 0 and r["falls"] == 0 and r["contacts"] == 0


def test_calibration_is_pooled_on_the_pi_and_saved_on_the_robot_once_sure(tmp_path):
    conf = str(tmp_path)
    p = D.pool_fit(conf, "Left", dict(wheel=1.02, wheel_se=0.02, gyro=0.99, gyro_se=0.03), in_use={"wheel_scale": 0.97})
    assert not p["wheel"]["sure"] and D.corr_commands(p) == []
    for _ in range(8):
        # (e19, audit 7) each fit keeps the value in use it was measured against (the pool is absolute)
        p = D.pool_fit(conf, "Left", dict(wheel=1.02, wheel_se=0.02, gyro=0.99, gyro_se=0.03),
                       in_use={"wheel_scale": 0.97})
    assert p["wheel"]["sure"] and p["wheel"]["runs"] == 9
    cmds = D.corr_commands(p, {"wheel_scale": 0.97})
    assert cmds and cmds[0].startswith("corr save wheel_scale 0.98")
    assert json.load(open(D.cal_path(conf, "Left")))["robot"] == "Left"


# ------------------------------------------------------------------------------------------------- the fixes (C4)
def test_late_raw_rows_still_count_and_a_watched_standing_robot_has_little_doubt():
    fp = FP.FusedPose("A")
    fp.start(0.0, 30.0, 30.0, 0.0)
    rng = random.Random(1)
    t = 0.0
    for k in range(300):
        t = k * 0.1
        fp.fix(t - 0.15, 30.0 + rng.gauss(0, 0.5), 30.0 + rng.gauss(0, 0.5), None, 1.0, "motion", now=t + 0.05)
        fp.raw(t, 0.0, 0.0, 0.5, True)          # stamped before the fix moved the filter to "now"
    assert getattr(fp, "n_late_used", 0) > 200
    assert fp.unc_cm(t) < 3.0                   # e09: 5-20 cm here, so near an edge nothing fitted
    assert FP.K_SIGMA == 8.0 and FP.K_TRACK == pytest.approx(2.45)


def test_an_empty_move_ends_at_once():
    out = []
    c = FC.Controller(["A"], HOME, lambda n, s: out.append(s), lambda *a: None)
    r = c.R["A"]
    c.t = 10.0
    mv = dict(turn_id=None, drive_id=None)
    assert c.empty_move(r, mv, (30.0, 30.0), "edge turn")
    assert c.counts["empty_moves"] == 1 and r.busy_until == pytest.approx(10.0 + FC.EMPTY_MOVE_PAUSE_S)


def test_the_filter_in_charge_frees_the_hung_robot_and_it_drives_on():
    import rehearse_phase_a as RA
    tmpl = [kw for name, sd, kw in RA.case_runs("b") if name.endswith("after")][0]
    k = dict(tmpl)
    k["sensors"] = True
    k["ctrl_kw"] = dict(k["ctrl_kw"], pose_source="ekf")
    r = RA.one(("b-ekf", 1, k))
    assert r["contacts"] == 0 and r["falls"] == 0
    assert r["moving_pct"]["Center"] > 60.0         # e09: 1-37 %


def watch_drive(mirror, sigma=0.8, seed=1, n=900):
    """A robot driving S-free left curves (radius ~20 cm), sightings with the lock-on's noise, 12 a second."""
    rng = random.Random(seed)
    L = TL.TrackingLayer(["A", "B"], ("lock",), identity="predict", camera_watch=True)
    pos = {"A": [60.0, 40.0, 0.0], "B": [60.0, 90.0, 180.0]}
    for nm, (x, y, h) in pos.items():
        L.seed(nm, 0.0, x, y)
    for k in range(n):
        t = k / 12.0
        dets = []
        for nm, p in pos.items():
            p[2] += 2.4                                   # 2.4 deg per picture: a 20 cm curve at ~10 cm/s
            p[0] += 0.85 * math.cos(math.radians(p[2]))
            p[1] += 0.85 * math.sin(math.radians(p[2]))
            L.own(nm, t, p[0], p[1], p[2], False)
            x = p[0] + rng.gauss(0, sigma)
            y = p[1] + rng.gauss(0, sigma)
            if mirror:
                x = 122.0 - x
            L.T[nm].sure = (t, x, y)
            dets.append(TL.Det("lock", t, x, y, name=nm))
        L.feed(t, dets)
    return L


def test_the_camera_watch_has_no_false_votes_on_noisy_curves_and_still_catches_a_mirror():
    old = TL.CAM_WATCH_METHOD
    try:
        TL.CAM_WATCH_METHOD = "e10"
        good = [watch_drive(False, seed=s) for s in range(1, 5)]
        assert all(L.counts.get("camera_distrusted", 0) == 0 for L in good)
        assert all(L.cam_checks.count(False) == 0 for L in good)
        assert watch_drive(True).counts.get("camera_distrusted", 0) >= 1
    finally:
        TL.CAM_WATCH_METHOD = old


def test_pictures_at_most_one_a_minute_per_robot():
    c = FC.Controller(["A", "B"], HOME, lambda n, s: None, lambda *a: None)
    c.t = 5.0
    c.want_picture(c.R["A"], "x")
    c.t = 30.0
    c.want_picture(c.R["A"], "x")
    c.want_picture(c.R["B"], "x")
    c.t = 66.0
    c.want_picture(c.R["A"], "x")
    assert [p["robot"] for p in c.take_pictures()] == ["A", "B", "A"]


# ------------------------------------------------------------------------------------------------- preferences (C5)
def test_the_quiz_fills_the_page_and_the_page_reaches_the_runs():
    import start_job as SJ
    p = SJ.quiz_to_prefs({"where": "auto", "length": "30", "watching": "yes", "caution": "explore", "robots": "pick",
                          "robots_names": "Left, Right"})
    assert p["quiz_done"] and p["run_min"] == 30.0 and p["watching"] == "yes" and p["probe_ahead"]
    a = SJ.prefs_args(p)
    assert "--probe-ahead" in a and a[a.index("--robots") + 1] == "Left,Right"
    env = SJ.prefs_env(dict(SJ.PREFS_DEFAULTS, pose_source="ekf", camera_watch=True))
    assert env["MIRLT_POSE_SOURCE"] == "ekf" and env["MIRLT_CAMERA_WATCH"] == "1" and env["MIRLT_DISCOVERY"] == "1"
    d = SJ.prefs_env(SJ.PREFS_DEFAULTS)                # the defaults are the build's own
    assert d["MIRLT_CAMERA_WATCH"] == "1" and d["MIRLT_POSE_SOURCE"] == "blend" and d["MIRLT_SCHOOL_ROAM"] == "freeroam"
    assert SJ.prefs_path("/x", "school-conference").endswith("prefs/school-conference.json")


def test_every_e08_e09_switch_is_on_the_page():
    import start_job as SJ
    for k in ("edge_rule", "pose_rule", "keepout_rule", "silent_out", "tilt_recovery", "pose_source", "gyro_heading",
              "old_tracking", "brick_watch", "camera_watch", "identity", "lock", "discovery", "workspace",
              "probe_ahead"):
        assert k in SJ.PREFS_DEFAULTS


def test_the_runners_take_the_new_switches():
    import freeroam as FR
    import mission_run as MR
    import argparse
    src = open(os.path.join(FLEET, "freeroam.py")).read()
    for flag in ("--no-discovery", "--typed-board", "--probe-ahead", "--external-bridge", "--no-shuffle",
                 "--picture-every-s", "--careful", "--tracking"):
        assert flag in src
    assert MR.roam_controller(argparse.Namespace(roam_controller=None)) in ("planner", "freeroam")


def test_an_own_pose_reset_is_not_read_as_motion_and_a_standing_robot_is_not_a_brick():
    """7 Nov home run, 12:33:47: Left's own pose was re-set 14 cm to the camera's; the layer read it as a drive
    backwards and Left went 13 s without a fix, its own spot taken for a new object."""
    def run(resets):
        old = TL.OWN_RESETS
        TL.OWN_RESETS = resets
        try:
            # (e17) the old way pinned without e17's elimination and gate record (they would also recover it)
            L = TL.TrackingLayer(["Left", "Right"], ("motion", "lights"), eliminate=None if resets else False,
                                 gate_record=None if resets else False)
            L.seed("Left", 0.0, 62.0, 26.0)
            L.seed("Right", 0.0, 20.0, 90.0)
            gaps, last = 0.0, 0.0
            for k in range(1, 200):
                t = k * 0.1
                ox = 76.0 if t < 2.0 else 62.0            # its own frame re-set by 14 cm at 2 s, the robot standing
                L.own("Left", t, ox + min(t, 1.0) * 5.0 if t < 2.0 else ox + 5.0, 21.0, 0.0, t > 1.0)
                L.own("Right", t, 20.0, 90.0, 0.0, True)
                fx = L.feed(t, [TL.Det("motion", t, 67.0, 26.0), TL.Det("motion", t, 20.0, 90.0)])
                if any(f["robot"] == "Left" for f in fx):
                    gaps, last = max(gaps, t - last), t
            return max(gaps, 19.9 - last), L.counts.get("bricks", 0)
        finally:
            TL.OWN_RESETS = old
    g, bricks = run(True)
    assert g < 1.0 and bricks == 0
    g_old, bricks_old = run(False)                       # the switch: as e09
    assert g_old > 5.0


def test_the_filters_uncertainty_learns_what_it_missed_at_camera_gaps():
    """(e10, 7 Nov) the real 7 Nov run 1 replay: the filter's 95 % undercovered (79-92 %). At the end of each camera gap
    the fix's miss beyond the uncertainty is learned and added; MIRLT_UNC_LEARN=0 the old way."""
    out = {}
    for learn in (True, False):
        fp = FP.FusedPose("r", learn=True)
        fp.unc_learn = learn
        fp.fix(0.0, 50, 50, None, 1.0, "motion")
        t = 0.0
        for i in range(12):
            for k in range(3):
                t += 0.08
                fp.raw(t, 0.0, 0.0, 0.0, True, 0.0, None, (50, 50, 50))
                fp.fix(t, 50, 50, None, 1.0, "motion")
            t += 1.5                                         # a camera gap
            fp.raw(t, 0.0, 0.0, 0.0, True, 0.0, None, (50, 50, 50))
            fp.fix(t, 50 + (4 if i % 2 else -4), 50, None, 1.0, "motion")   # the camera 4 cm off after it
        out[learn] = (fp.unc_add, fp.unc_cm(t))
    assert out[True][0] > 1.0 and out[False][0] == 0.0
    assert out[True][1] > out[False][1] + 1.0
