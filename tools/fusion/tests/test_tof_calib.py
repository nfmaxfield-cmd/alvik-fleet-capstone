"""Landmark ranging with shared errors handled (consider states, joint update,
adaptive weighting) and the distance-sensor calibration (tof_calib.py).

    python -m pytest tools/fusion/tests -q
"""
import json
import math

import numpy as np
import pytest

import engine as eng
import graph
import landmarks as lm
import tof_calib as tc
from ekf import EKF, D2R
from fusion_synth import sightings
from test_landmarks import drive, wall, end_err, WWG, FWD

CAM = WWG + ["camera"]
CAMTOF = WWG + ["camera", "tof"]


def run_cfg(run, dets, faces, **kw):
    sens = kw.pop("sensors")
    return eng.run_engine([run], dets, eng.config(sensors=sens, **kw), landmarks=faces)[0]["r1"]


def test_consider_states_keep_a_misplaced_board_from_hurting():
    """The board is 1.5 cm further than the map says. Plain landmark ranging pulls the camera's good
    estimate off; with the board's uncertainty carried (tof_consider) it stays as good as the camera."""
    run, truth = drive([wall(100.0)], wheel_scale=1.0)
    mapped = [wall(101.5)]
    dets = sightings(truth, seed=4)
    cam = end_err(run_cfg(run, dets, None, sensors=CAM), truth)
    plain = end_err(run_cfg(run, dets, mapped, sensors=CAMTOF), truth)
    cons = run_cfg(run, dets, mapped, sensors=CAMTOF, tof_consider=True, tof_face_sigma_cm=1.5)
    assert plain > 3 * cam
    assert end_err(cons, truth) < cam + 0.05
    assert cons.ekf.N == 8 + 3                     # (wheel-size states,) one face, range bias, mount turn
    assert np.all(cons.ekf.s[8:] == 0.0)           # never estimated
    g = graph.smooth(cons)
    assert math.hypot(g[-1][1] - truth[-1][1], g[-1][2] - truth[-1][2]) < cam + 0.1


def test_joint_update_matches_sequential_when_uncorrelated():
    k1 = EKF(0.0, 0.0, 0.0, 0.0, sigma_xy=2.0)
    k2 = EKF(0.0, 0.0, 0.0, 0.0, sigma_xy=2.0)
    J1, J2 = np.array([-1.0, 0.0, 0.0]), np.array([-0.9, -0.3, 2.0])
    k1.landmark_range(38.0, 40.0, J1, 0.5)
    k1.landmark_range(41.0, 42.0 + J2 @ k1.s[:3], J2, 0.7)       # linear model: h moves with the state
    k2.landmark_ranges([38.0, 41.0], [40.0, 42.0], [J1, J2], np.diag([0.25, 0.49]))
    assert np.allclose(k1.s, k2.s) and np.allclose(k1.P, k2.P)
    k3 = EKF(0.0, 0.0, 0.0, 0.0, sigma_xy=2.0)     # correlated errors: less information
    k3.landmark_ranges([38.0, 41.0], [40.0, 42.0], [J1, J2], np.array([[0.25, 0.28], [0.28, 0.49]]))
    assert k3.P[0, 0] > k2.P[0, 0]


def test_adaptive_weighting_reduces_a_persistent_bias():
    run, truth = drive([wall(100.0)], wheel_scale=1.0)
    mapped = [wall(101.5)]
    dets = sightings(truth, seed=4)
    plain = end_err(run_cfg(run, dets, mapped, sensors=CAMTOF), truth)
    adapt = end_err(run_cfg(run, dets, mapped, sensors=CAMTOF, tof_adapt=True), truth)
    assert adapt < plain


def test_decimation_uses_fewer_frames():
    faces = [wall(100.0)]
    run, _ = drive(faces)
    a = run_cfg(run, None, faces, sensors=WWG + ["tof"]).tof_counts()
    b = run_cfg(run, None, faces, sensors=WWG + ["tof"], tof_every=3).tof_counts()
    assert b["used"] < 0.4 * a["used"]


# ---------------------------------------------------------------- calibration
def synth_data(model_true_p, faces, robots, cfg, n=3000, seed=1, noise=0.3):
    """Readings from known parameters: poses facing the boards from 15-60 cm."""
    rng = np.random.default_rng(seed)
    rows = dict(run=[], robot=[], zone=[], face=[], t=[], z=[], x=[], y=[], th=[], vx=[], vy=[], vth=[])
    P = np.diag([0.01, 0.01, 1e-6, 1, 1, 1])
    i = 0
    while len(rows["z"]) < n:
        i += 1
        x, y = rng.uniform(10, 50), rng.uniform(-20, 20)
        th = rng.uniform(-0.4, 0.4) + (0.0 if rng.random() < 0.6 else math.pi / 2)
        s = np.array([x, y, th, 0, 0, 0])
        r = int(rng.integers(len(robots)))
        sp = 10.0 if rng.random() < 0.5 else -10.0          # driving towards and away (delay vs bias)
        for zi, ang in enumerate(cfg["tof_zone_deg"]):
            why, info = lm.predict_zone(s, P, ang * D2R, faces, cfg)
            if why:
                continue
            for k, v in (("run", f"R{i % 6}"), ("robot", r), ("zone", zi), ("face", info["face"]), ("t", i * 0.07),
                         ("z", 0.0), ("x", x), ("y", y), ("th", th), ("vx", sp * math.cos(th)),
                         ("vy", sp * math.sin(th)), ("vth", 0.0)):
                rows[k].append(v)
    d = {k: np.array(v) for k, v in rows.items()}
    m = tc.Model(d, robots, faces, cfg)
    d["z"] = m.predict(model_true_p) + rng.normal(0, noise, len(d["x"]))
    return d


def test_calibration_recovers_zone_directions_bias_delay_and_board_position():
    faces = lm.faces_from_map({"boards": [{"name": "A", "p1": [70, -40], "p2": [70, 40], "thickness_cm": 1.5},
                                          {"name": "B", "p1": [0, 70], "p2": [80, 70], "thickness_cm": 1.5}],
                               "obstacles": []})
    cfg = eng.config(tof_max_incidence_deg=45.0, tof_assoc_deg=0.0)
    robots = ["Left", "Right"]
    m0 = tc.Model({"face": np.zeros(1, int)}, robots, faces, cfg)
    p_true = m0.x0()
    R = len(robots)
    p_true[:7 * R].reshape(R, 7)[0, :5] = [3.0, 2.0, 2.5, 1.5, 3.0]       # Left's sensor turned ~2.5 deg
    p_true[:7 * R].reshape(R, 7)[1, :5] = [-1.0, -2.0, -1.5, -2.5, -1.0]
    p_true[:7 * R].reshape(R, 7)[0, 5] = 0.4                               # range bias +-0.4 cm (the robots'
    p_true[:7 * R].reshape(R, 7)[1, 5] = -0.4                              # average is held at 0)
    p_true[7 * R + 1] = 0.03                                               # 30 ms frame delay
    g = p_true[7 * R + 2:].reshape(m0.G, 3)
    g[m0.groups.index("A")] = [0.8, -0.6, 0.0]                             # board A 1 cm off
    g[m0.groups.index("B")] = [0.0, 1.0, 1.0]
    d = synth_data(p_true, faces, robots, cfg)
    model = tc.Model(d, robots, faces, cfg, fit_width=False)
    sigma = np.full(len(d["z"]), 0.3)
    p = tc.fit(model, sigma, max_nfev=80)
    off, bias, scale, width, delay, gg = model.unpack(p)
    t_off, t_bias, _, _, t_delay, t_g = model.unpack(p_true)
    assert np.abs(off - t_off).max() < 0.4
    assert abs(bias[0] - 0.4) < 0.15
    assert abs(delay - 0.03) < 0.01
    ia = model.groups.index("A")
    assert abs(gg[ia, 0] - 0.8) < 0.25                  # the along-normal shift is what the sensor sees
    # residuals: from several cm to the noise
    r0 = d["z"] - model.predict(model.x0())
    r1 = d["z"] - model.predict(p)
    assert np.sqrt(np.mean(r1 ** 2)) < 0.4 < np.sqrt(np.mean(r0 ** 2))


def test_calibration_file_roundtrip_and_engine_use(tmp_path):
    faces = [wall(100.0)]
    cfg = eng.config()
    m = tc.Model({"face": np.zeros(1, int), "robot": np.zeros(1, int)}, ["A"], faces, cfg)
    p = m.x0()
    p[5] = 0.0                                           # robot A: bias
    p[7 + 2] = -1.5                                      # the wall 1.5 cm nearer than the map (dx)
    cal = tc.cal_dict(m, p, (0.5, 0.01), (0.0, 0.0), ["X-1"], "map.json",
                      dict(board_cm={"wall": 0.2}, bias_cm={"A": 0.1}, yaw_deg={"A": 0.2}))
    path = tmp_path / "tof_calibration.json"
    path.write_text(json.dumps(cal))
    back = lm.load_cal(str(path))
    assert back["faces"][0].a[0] == pytest.approx(98.5)
    assert back["used_runs"] == ["X-1"]
    run, truth = drive([wall(98.5)], wheel_scale=1.0)
    plain = run_cfg(run, None, faces, sensors=WWG + ["tof"])
    with_cal = eng.run_engine([run], None, eng.config(sensors=WWG + ["tof"]), landmarks=faces, tof_cal=back)[0]["r1"]
    assert end_err(with_cal, truth) < 0.3 < end_err(plain, truth)
    assert with_cal.tcfg["tof_noise_rel"] == 0.01
    per = tmp_path / "loro.csv"
    per.write_text("run_id,calibration\nX-1,tof_calibration.json\n")
    assert lm.read_cal_per_run(str(per)) == {"X-1": str(path)}


def test_face_group_names():
    assert tc.face_group("A left") == "A" and tc.face_group("board 2 end p1") == "board 2"
    assert tc.face_group("block edge 3") == "block" and tc.face_group("wall") == "wall"


def test_spin_reference_from_camera_and_gyro():
    """A calibration run: 1 s still (step 1), then a 360 deg spin left at 36 deg/s (step 6) on the spot at
    (50, 20). The reference: camera position, gyro heading (bias from the still part) anchored to calib.py's
    start heading of 30 deg."""
    import runs as rn
    run = rn.Run("C-1", "A")
    rows, dets = [], []
    for i in range(1100):
        t = 1000.0 + i * 0.01
        spin = i >= 100
        rate = 36.0 if spin else 0.0
        rows.append(dict(t=t, step=6 if spin else 1, gz=rate + 0.5))           # gyro bias 0.5 deg/s
        if i % 3 == 0:
            dets.append(dict(t=t, x=50.0 + 0.1 * math.sin(i), y=20.0 + 0.1 * math.cos(i)))
    run.raw = rows
    crow = [dict(run="C-1", robot="A", event="calib", heading=30.0, t=1000.5, gs=1.0)]
    ref = tc.spin_reference(run, dets, crow, D2R)
    assert ref is not None
    i = np.searchsorted(ref[:, 0], 1006.0)
    assert ref[i, 1] == pytest.approx(50.0, abs=0.1) and ref[i, 2] == pytest.approx(20.0, abs=0.1)
    true_th = 30.0 + 36.0 * (ref[i, 0] - 1001.0)
    assert ref[i, 3] / D2R == pytest.approx(true_th, abs=0.5)
    assert tc.spin_reference(run, dets, [], D2R) is None                     # no measured heading: not used


def test_landmark_update_can_leave_heading_alone():
    """tof_heading_with_camera False: while the camera is tracking, a range reading corrects position only."""
    k = EKF(0.0, 0.0, 0.0, 0.0, sigma_xy=2.0, sigma_th=3 * D2R)
    J = np.array([-1.0, -0.3, 8.0])                 # an oblique hit: the reading says something about heading too
    k2 = EKF(0.0, 0.0, 0.0, 0.0, sigma_xy=2.0, sigma_th=3 * D2R)
    k.landmark_range(38.0, 40.0, J, 0.5)
    k2.landmark_range(38.0, 40.0, J, 0.5, freeze=(2,))
    assert k.s[2] != 0.0 and k2.s[2] == 0.0
    assert k2.s[0] > 0.5                            # position still corrected
    assert k2.P[2, 2] == pytest.approx((3 * D2R) ** 2, rel=1e-6) or k2.P[2, 2] <= (3 * D2R) ** 2


def test_systematic_leftover():
    """A constant error per run x zone x face (sd 0.3 cm) on top of 0.3 cm noise: systematic() finds ~0.3."""
    faces = lm.faces_from_map({"boards": [{"name": "A", "p1": [70, -40], "p2": [70, 40], "thickness_cm": 1.5}],
                               "obstacles": []})
    cfg = eng.config(tof_max_incidence_deg=45.0, tof_assoc_deg=0.0)
    m0 = tc.Model({"face": np.zeros(1, int)}, ["L"], faces, cfg)
    d = synth_data(m0.x0(), faces, ["L"], cfg, n=6000, seed=3)
    rng = np.random.default_rng(5)
    off = {}
    for i, key in enumerate(zip(d["run"], d["zone"], d["face"])):
        d["z"][i] += off.setdefault(key, rng.normal(0, 0.3))
    m = tc.Model(d, ["L"], faces, cfg)
    assert tc.systematic(m, m.x0()) == pytest.approx(0.3, abs=0.12)


def test_landmarks_only_as_camera_fallback():
    """tof_camera_gap_s 1: while the camera corrects the robot, landmark readings are skipped (the result is the
    camera's alone); once the camera is blocked for longer than 1 s they are used."""
    run, truth = drive([wall(100.0)], wheel_scale=1.0)
    mapped = [wall(101.5)]
    dets = sightings(truth, seed=4)
    cam = run_cfg(run, dets, None, sensors=CAM)
    fb = run_cfg(run, dets, mapped, sensors=CAMTOF, tof_camera_gap_s=1.0)
    assert fb.trajectory() == cam.trajectory()
    assert fb.tof_counts()["used"] == 0
    blocked = [d for d in dets if not (1003.0 <= d["t"] <= 1006.0)]
    fb2 = run_cfg(run, blocked, mapped, sensors=CAMTOF, tof_camera_gap_s=1.0)
    assert fb2.tof_counts()["used"] > 50


def test_tof_gate_split_and_decision(tmp_path):
    """--tof-gate: A B B A split per robot and route; landmarks kept only if not worse on the tuning runs;
    tables keep the scoring runs only, which take the twin's numbers when the gate says off."""
    import replay as rp
    sheet = tmp_path / "m.csv"
    sheet.write_text("order,robot,level,run_id\n" + "".join(f"{i},A,L,R{i}\n" for i in range(1, 9)))
    half = rp.gate_split("auto", str(sheet))
    assert [half[f"R{i}"] for i in range(1, 9)] == ["tune", "score", "score", "tune"] * 2
    nan = float("nan")
    def row(cfg, rid, e):
        return dict(config=cfg, run_id=rid, end_err_cm=e, end_head_err_deg=0.0, path_rmse_cm=nan,
                    path_max_cm=nan, head_rmse_deg=0.0)
    res = []
    for i in range(1, 9):
        rid = f"R{i}"
        res += [row("good", rid, 0.2), row("good [no landmarks]", rid, 0.3),
                row("bad", rid, 0.5 if half[rid] == "tune" else 0.1), row("bad [no landmarks]", rid, 0.3)]
    out, rep_ = rp.apply_gate(res, ["good", "bad"], half, False)
    dec = {r["setup"]: r["decision"] for r in rep_}
    assert dec == {"good": "landmarks ON", "bad": "landmarks OFF"}
    assert all(r["half"] == "score" for r in out) and len(out) == 4 * 4
    assert all(r["end_err_cm"] == 0.3 for r in out if r["config"] == "bad")       # the twin's numbers


def test_learning_the_board_while_the_camera_tracks_never_moves_the_pose():
    """tof_learn_map: with the camera tracking, readings only refine the board's offset (learned close to the
    true 1.5 cm); the pose is exactly the camera's. Once the camera is blocked, the learned board is used."""
    run, truth = drive([wall(100.0)], wheel_scale=1.0)
    mapped = [wall(101.5)]
    dets = sightings(truth, seed=4)
    cam = run_cfg(run, dets, None, sensors=CAM)
    kw = dict(sensors=CAMTOF, tof_consider=True, tof_face_sigma_cm=2.0, tof_camera_gap_s=1.0, tof_learn_map=True)
    lrn = run_cfg(run, dets, mapped, **kw)
    a, b = cam.trajectory(), lrn.trajectory()
    assert max(math.hypot(p[1] - q[1], p[2] - q[2]) for p, q in zip(a, b)) < 1e-9
    j = lrn.cons_map[("face", "wall")] + lrn.ekf.n_base
    assert abs(lrn.ekf.s[j]) == pytest.approx(1.5, abs=0.3)        # the face is 1.5 cm off its map position
    run5, truth5 = drive([wall(100.0)], wheel_scale=1.05)          # wheels 5% long, wheel size not learned
    blocked = [d for d in sightings(truth5, seed=4) if d["t"] < 1004.0]
    lb = run_cfg(run5, blocked, mapped, **dict(kw, wheel_scale=False))
    plain = run_cfg(run5, blocked, mapped, **dict(kw, wheel_scale=False, tof_learn_map=False))
    assert end_err(lb, truth5) < 0.85 * end_err(plain, truth5)


def test_covariance_intersection_update():
    k = EKF(0.0, 0.0, 0.0, 0.0, sigma_xy=2.0)
    J = np.array([-1.0, 0.0, 0.0])
    k.landmark_ci(38.0, 40.0, J, 0.5)
    assert 0.5 < k.s[0] < 2.0 and k.P[0, 0] < 4.0
    k2 = EKF(0.0, 0.0, 0.0, 0.0, sigma_xy=2.0)
    k2.landmark_range(38.0, 40.0, J, 0.5)
    assert k.P[0, 0] > k2.P[0, 0]                                  # never as confident as the plain update
