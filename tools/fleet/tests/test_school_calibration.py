"""The school calibration run (calib_run.py, 4 Nov 2026): binding by the wiggle (one robot, two robots, a spare tag, no
tag moving, two tags moving), the fit recovering known values, the safety checks (edge, outlets, the camera's view),
the robot's own leg poses chained, the tag offset from the turning point, the four-point height check against
Jay-Sun's numbers, and the one-camera table fit from tags 20, 23, 24, 25 only (21 and 22 too small to read). No robot,
no Pi."""
import math
import os
import random
import sys
import threading
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import calib_run as CR      # noqa: E402
import sites as SI          # noqa: E402

L, W = 396.2, 167.6


# ------------------------------------------------------------------------------------------------- binding
class FakeEye:
    """Tags whose heading the fake robots move: {id: heading}; a still spare tag; reads every call."""

    def __init__(self, tags):
        self.h = dict(tags)
        self.t = {i: [] for i in tags}

    def seen_ids(self, within_s=1.5):
        return sorted(self.h)

    def window(self, i, t0, t1):
        return [(time.time(), 0, 0, self.h[i])]

    def last(self, i):
        return (time.time(), 0, 0, self.h[i])


class FakeRobot:
    def __init__(self, name, eye, tags, ip="127.0.0.9", turn_scale=1.0):
        self.name, self.eye, self.tags, self.ip, self.k = name, eye, tags, ip, turn_scale

    def leg(self, kind, value, **kw):
        for t in self.tags:
            self.eye.h[t] = CR.wrap180(self.eye.h[t] + value * self.k)
        return {"result": "done", "info": "", "start": (0, 0, 0, 0), "end": (0, 0, value, 1000)}


def test_binding_one_robot_with_a_spare_tag_lying_still():
    eye = FakeEye({2: 90.0, 3: 10.0})
    rb = FakeRobot("AL-1DC0", eye, [2])
    assert CR.bind_by_wiggle([rb], eye, pause_s=0, log=lambda s: None) == {"AL-1DC0": 2}


def test_binding_two_robots_each_their_own_tag():
    eye = FakeEye({2: 0.0, 5: 45.0})
    a, b = FakeRobot("AL-1DC0", eye, [5]), FakeRobot("AL-2B7F", eye, [2])
    assert CR.bind_by_wiggle([a, b], eye, pause_s=0, log=lambda s: None) == {"AL-1DC0": 5, "AL-2B7F": 2}


def test_binding_refuses_when_no_tag_moves_or_two_move():
    eye = FakeEye({2: 0.0})
    with pytest.raises(CR.BindError) as e:
        CR.bind_by_wiggle([FakeRobot("AL-1DC0", eye, [])], eye, pause_s=0, log=lambda s: None)
    assert "no roof tag moved" in str(e.value)
    eye = FakeEye({2: 0.0, 3: 0.0})
    with pytest.raises(CR.BindError) as e:
        CR.bind_by_wiggle([FakeRobot("AL-1DC0", eye, [2, 3])], eye, pause_s=0, log=lambda s: None)
    assert "2 tags moved" in str(e.value)
    eye = FakeEye({})
    with pytest.raises(CR.BindError) as e:
        CR.bind_by_wiggle([FakeRobot("AL-1DC0", eye, [])], eye, pause_s=0, log=lambda s: None)
    assert "no roof tag is seen" in str(e.value)


def test_binding_names_from_the_wifi_unless_the_site_names_it_and_the_old_way_is_kept():
    site = SI.load("school-conference")
    assert CR.site_name_for(site, "AL-1DC0", "192.168.1.134") == "AL-1DC0"      # Alpha / Bravo list no ip
    s2 = dict(site, robots=[{"name": "Charlie", "ip": "192.168.1.134"}])
    assert CR.site_name_for(s2, "AL-1DC0", "192.168.1.134") == "Charlie"
    assert CR.fixed_binding(site, ["Alpha", "Bravo"]) == {"Alpha": 0, "Bravo": 1}
    with pytest.raises(CR.BindError):
        CR.fixed_binding(site, ["AL-1DC0"])


# ------------------------------------------------------------------------------------------------- the fit
ROLLS = (20, 45, 70, 20, 45, 70, 20, 45, 70, 20, 45, 70)   # phase A's rolls, out and back
TOUR_LEGS = (25, 48, 33, 61, 40, 29, 55, 37, 44, 66, 31, 52, 27, 45, 58, 35, 49, 30, 62, 41)   # phase A's tour legs


def _synthetic(true, old, n_turns=12, rolls=ROLLS + TOUR_LEGS, still_s=60.0, seed=1, cam_sd=0.4, coast=0.3):
    rng = np.random.default_rng(seed)
    g, d, w = true["gyro_scale"], true["gyro_drift_dps"], true["wheel_scale"]
    gs0, gd0, ws0 = old["gyro_scale"], old["gyro_drift_dps"], old["wheel_scale"]
    turns = []
    for k in range(n_turns):
        cmd = (90, -90, 170, -170)[k % 4]
        dt = 0.3 + abs(cmd) / 60.0
        raw = (cmd + gd0 * dt) / gs0               # the robot turns until its belief says cmd
        real = g * raw - d * dt
        turns.append((cmd, dt, real + rng.normal(0, cam_sd)))
    still = [((g * 0 + d * still_s) / g * gs0 - gd0 * still_s, still_s)]   # belief change standing still
    rr = []
    for Lc in rolls:
        counted = Lc / ws0
        rr.append((Lc, w * counted + coast + rng.normal(0, 0.3)))
    return turns, still, rr


def test_fit_recovers_the_true_values_within_the_stated_precision():
    true = {"wheel_scale": 0.97, "gyro_scale": 1.03, "gyro_drift_dps": 0.3}
    old = {"wheel_scale": 1.0, "gyro_scale": 1.0, "gyro_drift_dps": 0.0}
    t, s, r = _synthetic(true, old)
    f = CR.fit_values(t, s, r, old)
    for k in CR.PARAMS:
        assert abs(f[k] - true[k]) <= 2.5 * f["se_" + k], (k, f[k], f["se_" + k])     # within its own +-
        assert f["verdict"][k] == "ok", (k, f["verdict"][k])
    assert abs(f["coast_cm"] - 0.3) < 0.5


def test_fit_from_other_stored_values_and_refuses_too_little_data():
    true = {"wheel_scale": 0.985, "gyro_scale": 1.07, "gyro_drift_dps": -0.05}
    old = {"wheel_scale": 0.979, "gyro_scale": 1.075, "gyro_drift_dps": 0.0}
    t, s, r = _synthetic(true, old, seed=4)
    f = CR.fit_values(t, s, r, old)
    assert all(abs(f[k] - true[k]) <= 2.5 * f["se_" + k] for k in CR.PARAMS)
    f2 = CR.fit_values(t[:2], [], r[:1], old)
    assert f2["verdict"]["gyro_scale"] == "not enough data"


# ------------------------------------------------------------------------------------------------- safety, poses
def test_site_points_edges_outlets_and_the_cameras_view():
    S = CR.Site(SI.load("school-conference"))
    z = S.zone12()
    assert z[0] == "P01" and z[-1] == "P20" and len(z) == 20
    assert S.edge == 13.0 and any("outlet" in k["name"] for k in S.keepouts) and any("reference" in k["name"]
                                                                                     for k in S.keepouts)
    ax, ay = S.aim("P04")              # 13.3 cm from the edge: aimed 1 cm further in
    assert ay >= 14.0 - 1e-6 and ax == S.points["P04"][0]
    p11 = S.points["P11"]
    assert S.problems(p11, (p11[0] - 60, p11[1])) == []                         # the rolls' direction
    assert any("outlet" in q for q in S.problems(p11, (p11[0] + 45, p11[1])))   # the middle outlet box
    assert any("edge" in q for q in S.problems(p11, (p11[0], 5.0)))
    assert any("view" in q for q in S.problems((180, 120), (215, 120)))
    for lab in z:                      # every point reached from P11 straight or round one keep-out
        a, b = S.points["P11"], S.aim(lab)
        assert S.problems(a, b) == [] or S.via(a, b) is not None, lab


def test_chained_leg_poses_and_tag_offset():
    a = (100.0, 50.0, 90.0)
    rel = CR.relative((0, 0, 0, 0), (10.0, 0.0, 0.0, 0))
    b = CR.compose(a, rel)
    assert abs(b[0] - 100) < 1e-9 and abs(b[1] - 60) < 1e-9
    c = CR.compose(b, CR.relative((5, 5, 30, 0), (5, 5, 120, 0)))
    assert abs(c[2] - 180) < 1e-9 or abs(c[2] + 180) < 1e-9
    o = (1.5, -0.8)
    samp = []
    for th in range(0, 360, 45):
        cs, sn = math.cos(math.radians(th)), math.sin(math.radians(th))
        samp.append((150 + cs * o[0] - sn * o[1], 80 + sn * o[0] + cs * o[1], th))
    (cx, cy), (ox, oy), rms = CR.tag_offset(samp)
    assert abs(cx - 150) < 1e-6 and abs(ox - 1.5) < 1e-6 and abs(oy + 0.8) < 1e-6 and rms < 1e-6


# ------------------------------------------------------------------------------------------------- heights
def test_height_model_reproduces_jay_suns_pattern_with_the_camera_above_the_floor():
    S = CR.Site(SI.load("school-conference"))
    z_floor = CR.four_point([-45 * 2.54, W / 2, (90 - 33) * 2.54], 6.19, S.points)    # 7.5 ft above the floor
    z_table = CR.four_point([-45 * 2.54, W / 2, 90 * 2.54], 6.19, S.points)           # 7.5 ft above the table
    worst_floor = max(abs(z_floor[p][0] - v) for p, v in CR.JAY_IN.items())
    worst_table = max(abs(z_table[p][0] - v) for p, v in CR.JAY_IN.items())
    assert worst_floor <= 0.7 and worst_table > 1.0
    order = sorted(CR.JAY_IN, key=lambda p: z_floor[p][0])
    assert order[-2:] == ["P11", "P15"]                     # the far points the most, as Jay-Sun found


def test_sim_camera_reads_zones_1_2_not_the_far_corners():
    cam = CR.SchoolCamera(z_cm=145.0)
    for lab in ("P01", "P05", "P11", "P15", "P19", "P20"):
        x, y = CR.Site(SI.load("school-conference")).points[lab]
        assert cam.tag_px(x, y, 6.19, 8.0, 90) >= CR.READ_PX, lab
    for x, y in ((L - 7.75, 7.75), (L - 7.75, W - 7.75)):        # tags 21, 22 (12 cm, flat)
        px = cam.tag_px(x, y, 0.18, 12.0)
        assert px is None or px < CR.READ_PX


# ------------------------------------------------------------------------------------------------- the table fit
def _school_cam_cfg(z=145.0):
    import tag_cam as CAMS
    cfg = CAMS.default_config(table=(396.24, 167.64))
    cfg["cams"] = [dict(name="overhead", pos=[-45 * 2.54, 167.64 / 2 + 3.0, z], look_at=[150.0, 84.0, 0.0],
                        size=[1280, 960], hfov_deg=70.4, blur_px=0.9, noise=3.0, k1=0.0, roll_deg=0.5, port=8765)]
    return cfg


def test_rendered_school_picture_reads_20_23_24_25_and_not_21_22():
    import tag_cam as CAMS
    import ref_tags as RT
    cam = CAMS.cams_from_config(_school_cam_cfg())[0]
    seen = {d.id for d in RT.detector().detect_full(cam.render([]))}
    assert {20, 23, 24} <= seen, seen
    assert not ({21, 22} & seen), seen


def test_one_camera_table_fit_from_the_four_near_tags():
    """The site fit (site_fit.full) from one camera at the TV end, reference tags 20, 23, 24, 25 only, and roof-tag reads
    in zones 1-2 at 6.19 cm: its floor error in zones 1-2 against the true camera."""
    import tag_cam as CAMS
    import ref_tags as RT
    import site_fit as SF
    from cameras import CameraModel
    cfg = _school_cam_cfg()
    cam = CAMS.cams_from_config(cfg)[0]
    truth = cam.true_model()
    rng = np.random.default_rng(3)
    Lt, Wt = cfg["table"]
    rc = RT.default_config()
    al = {str(k): v for k, v in cfg["ref_tags"]["along"].items()}
    obs = {"cameras": {"overhead": {"size": [1280, 960], "f_prior": None}}, "corners": {"overhead": {}}, "tag_cm": 8.0,
           "tag_height_cm": 6.19, "stickers": {}, "tag_groups": [], "distances": [],
           "table": {"length_cm": Lt, "width_cm": Wt},
           "ref_tags": {"layout": RT.layout(rc), "tag_cm": RT.TAG_CM, "z_cm": RT.Z_CM, "offset_cm": RT.OFFSET_CM,
                        "reads": {"overhead": {}}, "truth_along": al}}
    for i, (x, y, th) in RT.world_poses(rc, Lt, Wt, along=al).items():
        if i in ("21", "22"):
            continue                                  # too small to read from the TV end (Jay-Sun)
        uv = cam.project(SF.tag_corners_world(x, y, th, RT.Z_CM, RT.TAG_CM))
        obs["ref_tags"]["reads"]["overhead"][i] = (uv + rng.normal(0, 0.25, uv.shape)).tolist()
    for k in range(24):
        x, y, th = rng.uniform(20, 190), rng.uniform(20, Wt - 20), rng.uniform(-180, 180)
        uv = cam.project(SF.tag_corners_world(x, y, th, 6.19, 8.0))
        obs["tag_groups"].append({"id": 2, "reads": {"overhead": (uv + rng.normal(0, 0.3, uv.shape)).tolist()},
                                  "truth": [x, y, th]})
    res = SF.full(obs)
    m = CameraModel.from_dict(res["cameras"]["overhead"])
    xs, ys = np.meshgrid(np.linspace(13, 192, 15), np.linspace(13, Wt - 13, 8))
    P = np.column_stack([xs.ravel(), ys.ravel()])
    errs = []
    for z in (0.0, 6.19):
        u, v = truth.project(np.column_stack([P, np.full(len(P), z)]))
        x2, y2 = m.to_floor(u, v, z)
        errs.append(np.hypot(x2 - P[:, 0], y2 - P[:, 1]))
    e = np.concatenate(errs)
    assert np.nanmax(e) < 2.0, np.nanmax(e)
    assert abs(m.center[2] - 145.0) < 6.0


# ------------------------------------------------------------------------------------------------- Mission Control
def test_mission_control_calibration_card_job(tmp_path):
    import json
    import calib_job as CJ

    class FakeRun:
        def alive(self):
            return True

    class FakeSession:
        dir = str(tmp_path)
        run = None
    S = FakeSession()
    d = tmp_path / "calib" / "20261104-2100-dry"
    d.mkdir(parents=True)
    (d / "status.json").write_text(json.dumps({"phase": "done", "robot": "AL-1DC0", "tag": 2, "words": "ok"}))
    (d / "report.md").write_text("# report")
    job = CJ.CalibJob(S)                         # the newest earlier run is shown after a restart
    st = job.status()
    assert st["phase"] == "done" and st["robot"] == "AL-1DC0" and st["report"] and not st["running"]
    assert job.file("report.md") == b"# report"
    with pytest.raises(ValueError):
        job.file("../../etc/passwd")
    S.run = FakeRun()
    with pytest.raises(ValueError) as e:
        job.start({"dry": True})
    assert "Mission Control run" in str(e.value)


def test_cautious_hold_and_keep_going_switch(tmp_path, monkeypatch):
    import argparse
    # (e07) the intended default since 11-05c is "recover" (the site's calibration.lost_policy, else recover): the
    # watch stops a leg whose tag is lost exactly as "hold" does, then the recovery drives back to the last read
    site = SI.load("school-conference")
    assert str((site.get("calibration") or {}).get("lost_policy") or "recover") == "recover"
    a = argparse.Namespace(lost_policy="recover", dry=True)
    run = CR.CalibRun(a, site, str(tmp_path))
    run.tag = 2

    class E0:
        def age(self, t):
            return 2.0

        def last(self, t):
            return (10.0, 10.0, 0.0)
    run.eye = E0()
    run._leg_moving = True
    assert run.watch() is True and run._lost_stop          # recover: the leg stops too, its last read kept
    a = argparse.Namespace(lost_policy="hold", dry=True)
    run = CR.CalibRun(a, SI.load("school-conference"), str(tmp_path))
    run.tag = 2

    class E:
        def __init__(self, age):
            self.a = age

        def age(self, t):
            return self.a

        def last(self, t):
            return None
    run.eye = E(2.0)
    run._leg_moving = True
    assert run.watch() is True                   # tag unseen 2 s while driving: stop
    run.eye = E(0.2)
    assert run.watch() is False
    run.a.lost_policy = "keep_going"
    run.eye = E(5.0)
    assert run.watch() is False                  # the experiments' policy: drive on
    run.a.lost_policy = "hold"
    run.eye = E(99.0)
    monkeypatch.setattr(CR, "LOST_GIVEUP_S", 0.3)
    run.a.lost_giveup = True                     # (11-10g) the old way behind its switch: the hold ends the run
    with pytest.raises(CR.RunEnd):
        run.hold_until_seen()
    # (11-10g, never give up) by default the hold keeps waiting (the robot is stopped) until seen or Stop
    import threading
    run.a.lost_giveup = False
    run.stop_flag = threading.Event()
    threading.Timer(0.8, run.stop_flag.set).start()
    with pytest.raises(CR.RunEnd, match="stopped"):
        run.hold_until_seen()                    # waited past 2 x LOST_GIVEUP_S, ended only by Stop
