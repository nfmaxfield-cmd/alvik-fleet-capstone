"""Landmark ranging: ray-to-face geometry, the measurement model, gating, and
that nothing changes without a map.

    python -m pytest tools/fusion/tests -q
"""
import json
import math
import random

import numpy as np
import pytest

import engine as eng
import graph
import landmarks as lm
from ekf import EKF, D2R
from fusion_synth import straight_run

CFG = eng.config()
FWD = CFG["tof_forward_cm"] - CFG["tof_recess_cm"]      # 4.5 cm
WWG = ["wheel_v", "wheel_w", "gyro"]


def wall(x=100.0, y0=-50.0, y1=50.0, name="wall"):
    return lm.Face(name, (x, y0), (x, y1))


# ---------------------------------------------------------------- geometry
def test_ray_face_hit_miss_parallel_behind():
    f = wall(10.0, -5.0, 5.0)
    s, g = lm.ray_face((0.0, 0.0), (1.0, 0.0), f)
    assert s == pytest.approx(10.0) and g == pytest.approx(5.0)
    d = (math.cos(0.3), math.sin(0.3))
    s, g = lm.ray_face((0.0, 0.0), d, f)
    assert s == pytest.approx(10.0 / math.cos(0.3)) and g == pytest.approx(5.0 + 10.0 * math.tan(0.3))
    assert lm.ray_face((0.0, 0.0), (math.cos(0.6), math.sin(0.6)), f) is None     # passes beyond the end
    assert lm.ray_face((0.0, 0.0), (0.0, 1.0), f) is None                          # parallel
    assert lm.ray_face((0.0, 0.0), (-1.0, 0.0), f) is None                         # behind the sensor
    assert lm.ray_face((20.0, 0.0), (-1.0, 0.0), f)[0] == pytest.approx(10.0)      # seen from the other side


def test_nearest_face_occludes_the_one_behind():
    faces = [wall(50.0, name="far"), wall(30.0, -5, 5, name="near")]
    s, i, _ = lm.nearest((0.0, 0.0), (1.0, 0.0), faces)
    assert faces[i].name == "near" and s == pytest.approx(30.0)
    s, i, _ = lm.nearest((0.0, 20.0), (1.0, 0.0), faces)
    assert faces[i].name == "far" and s == pytest.approx(50.0)
    assert lm.nearest((0.0, 0.0), (-1.0, 0.0), faces) is None


@pytest.mark.parametrize("ang_deg", [-22.0, 0.0, 11.0])
def test_line_range_value_and_jacobian(ang_deg):
    s = np.array([12.0, -3.0, 0.4])
    a, n = np.array([60.0, -40.0]), np.array([math.cos(0.2), math.sin(0.2)])
    ang = ang_deg * D2R
    d, J = lm.line_range(s, ang, FWD, a, n)
    p = lm.sensor_pose(s, FWD)
    q = p + d * np.array([math.cos(s[2] + ang), math.sin(s[2] + ang)])
    assert (q - a) @ n == pytest.approx(0.0, abs=1e-9)                   # the end point is on the line
    for j in range(3):
        sp = s.copy()
        sp[j] += 1e-6
        assert (lm.line_range(sp, ang, FWD, a, n)[0] - d) / 1e-6 == pytest.approx(J[j], rel=1e-4, abs=1e-4)


def test_predict_zone_square_on_and_angled():
    faces = [wall(40.0 + FWD)]                     # face 40 cm ahead of the sensor
    P = np.eye(6) * 0.01
    s = np.array([0.0, 0.0, 0.0, 0, 0, 0])
    why, info = lm.predict_zone(s, P, 0.0, faces, CFG)
    assert why is None and info["h"] == pytest.approx(40.0)
    why, _ = lm.predict_zone(s, P, 22 * D2R, faces, CFG)
    assert why == "grazing"                        # 22 deg off square-on: over the default 20 deg limit
    why, info = lm.predict_zone(s, P, 22 * D2R, faces, eng.config(tof_max_incidence_deg=60.0))
    assert why is None and info["h"] == pytest.approx(40.0 / math.cos(22 * D2R))


def test_predict_zone_rejects_edges_ambiguity_range_grazing():
    P = np.diag([0.01, 0.01, 1e-6, 1, 1, 1])        # 0.1 cm, 0.06 deg
    s = np.array([0.0, 0.0, 0.0, 0, 0, 0])
    # the ray lands 1 cm from the face's end: too close (margin 2 cm). (With the default +-5.5 deg
    # association check the neighbouring rays would already miss it: switched off here.)
    one_ray = eng.config(tof_assoc_deg=0.0)
    assert lm.predict_zone(s, P, 0.0, [lm.Face("short", (40, -10), (40, 1))], CFG)[0] == "ambiguous"
    why, _ = lm.predict_zone(s, P, 0.0, [lm.Face("short", (40, -10), (40, 1))], one_ray)
    assert why == "edge"
    # the margin grows with how unsure the robot is sideways
    why, _ = lm.predict_zone(s, P, 0.0, [lm.Face("f", (40, -10), (40, 4))], one_ray)
    assert why is None
    Pbig = np.diag([0.01, 4.0, 1e-6, 1, 1, 1])     # 2 cm sideways
    why, _ = lm.predict_zone(s, Pbig, 0.0, [lm.Face("f", (40, -10), (40, 4))], one_ray)
    assert why == "edge"
    # two faces close in angle (a board corner in the zone's slice): skip
    corner = [lm.Face("front", (40, -20), (40, 1)), lm.Face("side", (40, 1), (60, 1))]
    why, _ = lm.predict_zone(s, P, 0.0, corner, CFG)
    assert why == "ambiguous"
    # nothing mapped in that direction; or the face is beyond the sensor's range
    why, _ = lm.predict_zone(s, P, 0.0, [wall(-50.0)], CFG)
    assert why == "no_face"
    why, _ = lm.predict_zone(s, P, 0.0, [wall(200.0, -200, 200)], CFG)
    assert why == "no_face"
    # hit at 70 deg from square-on
    cfg60 = eng.config(tof_max_incidence_deg=60.0)
    c, sn = 150 * math.cos(20 * D2R), 150 * math.sin(20 * D2R)       # a face through (30, 0) at 20 deg to the ray
    why, _ = lm.predict_zone(s, P, 0.0, [lm.Face("slant", (30 - c, -sn), (30 + c, sn))], cfg60)
    assert why == "grazing"


def test_zone_width_takes_the_nearest_point_in_the_slice():
    cfg = eng.config(tof_zone_width_deg=11.0)
    faces = [wall(40.0 + FWD, -100, 100)]
    P = np.eye(6) * 0.01
    s = np.array([0.0, 0.0, 0.0, 0, 0, 0])
    _, info = lm.predict_zone(s, P, 22 * D2R, faces, cfg)          # the slice's near edge is 16.5 deg off square
    assert info["h"] == pytest.approx(40.0 / math.cos(16.5 * D2R))
    _, info = lm.predict_zone(s, P, 0.0, faces, cfg)
    assert info["h"] == pytest.approx(40.0)


def test_faces_from_map_and_csv(tmp_path):
    m = {"boards": [{"name": "A", "p1": [30, 10], "p2": [50, 10], "thickness_cm": 2.0}],
         "obstacles": [{"id": "block", "shape": "polygon", "points": [[0, 0], [4, 0], [4, 2]], "source": "user"},
                       {"id": "seen", "shape": "polygon", "points": [[0, 0], [4, 0], [4, 2]], "source": "camera"},
                       {"id": "cup", "shape": "circle", "center": [5, 5], "radius": 2, "source": "user"}]}
    p = tmp_path / "map.json"
    p.write_text(json.dumps(m))
    faces = lm.load(str(p))
    names = [f.name for f in faces]
    assert names[:4] == ["A left", "A end p2", "A right", "A end p1"]
    assert len(faces) == 4 + 3                          # the camera-found obstacle and the circle are not landmarks
    assert faces[0].a == pytest.approx([30, 11]) and faces[0].b == pytest.approx([50, 11])
    assert faces[2].a == pytest.approx([50, 9]) and faces[2].b == pytest.approx([30, 9])
    c = tmp_path / "landmarks.csv"
    lm.write_csv(faces[:2], str(c))
    back = lm.load(str(c))
    assert [f.name for f in back] == names[:2] and back[1].b == pytest.approx(faces[1].b, abs=0.01)


# ---------------------------------------------------------------- filter and engine
def test_ekf_range_update_pulls_along_track_only():
    """A wall straight ahead says how far along the robot is, nothing about sideways."""
    k = EKF(0.0, 0.0, 0.0, 0.0, sigma_xy=2.0, sigma_th=0.5 * D2R)
    faces = [wall(40.0 + FWD)]
    for _ in range(30):
        why, info = lm.predict_zone(k.s, k.P, 0.0, faces, CFG)
        assert why is None
        k.landmark_range(38.0, info["h"], info["J"], 0.5)     # the robot is really 2 cm further on
    assert k.s[0] == pytest.approx(2.0, abs=0.1)
    assert k.s[1] == pytest.approx(0.0, abs=1e-6)
    assert math.sqrt(k.P[1, 1]) == pytest.approx(2.0, rel=0.01)


def zones_for(truth, faces, hz=15.0, noise=0.0, seed=3, clutter=None, cfg=CFG):
    """RAW zone columns along a truth path: a new frame every 1/hz s (held in
    between, as on the robot); `clutter(t, zone)` may return a shorter reading
    (something unmapped in front)."""
    rng = random.Random(seed)
    rows, last, vals = [], -1e9, None
    for t, x, y, th in truth:
        if t - last >= 1.0 / hz - 1e-9:
            last = t
            p = lm.sensor_pose((x, y, th), FWD)
            vals = []
            for zi, a in enumerate(cfg["tof_zone_deg"]):
                h = lm.nearest(p, (math.cos(th + a * D2R), math.sin(th + a * D2R)), faces)
                d = h[0] + rng.gauss(0, noise * (0.5 + 0.03 * h[0])) if h else 350.0
                if clutter:
                    c = clutter(t, zi)
                    d = min(d, c) if c is not None else d
                vals.append(d)
        rows.append(dict(zip(lm.ZONE_COLS, vals)))
    return rows


def drive(faces, clutter=None, wheel_scale=1.05, noise=1.0):
    """A robot driving 60 cm towards a wall; its wheels over-read by 5%."""
    run, truth = straight_run("r1", "A", 0.0, 0.0, 0.0, v=10.0, still_s=1.0, drive_s=6.0)
    for r, z in zip(run.raw, zones_for(truth, faces, noise=noise, clutter=clutter)):
        r.update({k: f"{v:.1f}" for k, v in z.items()})
        r["v_cms"] *= wheel_scale
        r["wl_rpm"] *= wheel_scale
        r["wr_rpm"] *= wheel_scale
    return run, truth


def end_err(tracked, truth):
    s = tracked.trajectory()[-1]
    return math.hypot(s[1] - truth[-1][1], s[2] - truth[-1][2])


def test_landmark_ranging_fixes_wheel_scale_error():
    faces = [wall(100.0)]
    run, truth = drive(faces)
    base = eng.run_engine([run], None, eng.config(sensors=WWG))[0]["r1"]
    tof = eng.run_engine([run], None, eng.config(sensors=WWG + ["tof"]), landmarks=faces)[0]["r1"]
    assert end_err(base, truth) > 2.5                       # 5% of 60 cm
    # most of it removed; some lag stays (the filter has no wheel-size state, so the wheels keep pushing)
    assert end_err(tof, truth) < 0.35 * end_err(base, truth)
    c = tof.tof_counts()
    assert c["used"] > 300 and c["gated"] < 0.01 * c["used"]
    assert c["used"] + c["gated"] + c["no_face"] + c["ambiguous"] + c["edge"] + c["grazing"] == 5 * len(
        [1 for e in tof.events if e[2] == "tof"])
    # each sensor frame once: about 15 a second over 8 s, not every 10 ms RAW row
    assert 100 <= sum(1 for e in tof.events if e[2] == "tof") <= 130
    g = graph.smooth(tof)
    assert math.hypot(g[-1][1] - truth[-1][1], g[-1][2] - truth[-1][2]) < 0.35 * end_err(base, truth)


def test_unmapped_object_is_gated_and_does_not_move_the_pose():
    """A hand 15 cm in front of the sensor for 2 s in the middle zones: those
    readings are rejected and the end pose is the same as without the hand."""
    faces = [wall(100.0)]
    hand = lambda t, zi: (15.0 if 1003.0 <= t <= 1005.0 and zi in (1, 2, 3) else None)
    run0, truth = drive(faces)
    run1, _ = drive(faces, clutter=hand)
    cfg = eng.config(sensors=WWG + ["tof"])
    a = eng.run_engine([run0], None, cfg, landmarks=faces)[0]["r1"]
    b = eng.run_engine([run1], None, cfg, landmarks=faces)[0]["r1"]
    cb = b.tof_counts()
    assert cb["gated"] >= 25                   # 3 zones x ~30 frames, all rejected
    assert cb["gated"] <= 3 * 32
    used = [i["z"] for e in b.events if e[2] == "tof" for o, i in e[3]["res"] if o == "used"]
    assert min(used) > 20.0                    # not one hand reading got in
    assert end_err(b, truth) < end_err(a, truth) + 0.3


def test_readings_out_of_range_are_not_used():
    faces = [wall(200.0)]                      # beyond 120 cm the whole run
    run, _ = drive(faces)
    tr = eng.run_engine([run], None, eng.config(sensors=WWG + ["tof"]), landmarks=faces)[0]["r1"]
    c = tr.tof_counts()
    assert c["used"] == 0 and c["gated"] == 0 and c["bad_reading"] > 0


def test_no_landmarks_changes_nothing():
    """No map (or "tof" not in the setup): the filter runs exactly as before."""
    faces = [wall(100.0)]
    run, _ = drive(faces)
    ref = eng.run_engine([run], None, eng.config(sensors=WWG))[0]["r1"].trajectory()
    for cfg, lms in ((eng.config(sensors=WWG + ["tof"]), None), (eng.config(sensors=WWG + ["tof"]), []),
                     (eng.config(sensors=WWG), faces)):
        tr = eng.run_engine([run], None, cfg, landmarks=lms)[0]["r1"]
        assert tr.trajectory() == ref
        assert not any(e[2] == "tof" for e in tr.events)
