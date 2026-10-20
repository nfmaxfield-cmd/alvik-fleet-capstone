"""tools/recognition/cameras.py on synthetic cameras (synth.py): a near-nadir one 262 cm up
and an oblique one ~45 deg down from 200 cm away, both 640x480.

    python -m pytest tools/recognition/tests -q
"""
import importlib.util
import json
import math
import os

import numpy as np
import pytest

import synth
from recognition.cameras import (CalibrationError, CameraModel, floor_heading_to_image, image_heading_to_floor,
                                 transfer)

GRID = synth.board_points(10.0)     # 13 x 13 over the 122 cm board


def floor_err(model, true, z=0.0):
    """cm error of model.to_floor over the board, where the true camera sees it."""
    xyz = np.column_stack([GRID, np.full(len(GRID), z)])
    u, v = true.project(xyz)
    ok = true.in_image(u, v)
    x, y = model.to_floor(u[ok], v[ok], z)
    return np.hypot(x - GRID[ok, 0], y - GRID[ok, 1])


def both_visible(a, b, z):
    xyz = np.column_stack([GRID, np.full(len(GRID), z)])
    return xyz[a.in_image(*a.project(xyz)) & b.in_image(*b.project(xyz))]


@pytest.fixture(scope="module")
def fitted():
    """Both cameras fitted from 0.5 px-noise clicks on a 20 cm grid (A with its tape height)."""
    A, B = synth.cam_overhead(), synth.cam_oblique()
    uva, xya, _ = synth.clicks(A, synth.board_points(20), noise=0.5, seed=1)
    uvb, xyb, _ = synth.clicks(B, synth.board_points(20), noise=0.5, seed=2)
    return A, B, CameraModel.from_points(uva, xya, synth.SIZE, height_cm=262.0), CameraModel.from_points(
        uvb, xyb, synth.SIZE)


def test_true_cameras_round_trip():
    for cam in (synth.cam_overhead(k1=-0.05), synth.cam_oblique(k1=-0.1)):
        xyz = np.column_stack([GRID, np.full(len(GRID), 4.0)])
        u, v = cam.project(xyz)
        ok = cam.in_image(u, v)
        x, y = cam.to_floor(u[ok], v[ok], 4.0)
        assert np.abs(x - GRID[ok, 0]).max() < 1e-6 and np.abs(y - GRID[ok, 1]).max() < 1e-6
    assert all(np.isnan(synth.cam_oblique().project((0.0, 0.0, 500.0))))     # behind the camera


@pytest.mark.parametrize("make,height", [(synth.cam_overhead, 262.0), (synth.cam_oblique, None)])
def test_from_points_noise_free_recovers_pose(make, height):
    cam = make()
    uv, xy, _ = synth.clicks(cam, synth.board_points(20))
    m = CameraModel.from_points(uv[:12], xy[:12], synth.SIZE, height_cm=height)
    assert m.fit["rms_px"] < 0.5
    assert np.linalg.norm(m.center - cam.center) < 1.0, (m.center, cam.center)
    assert abs(m.f / cam.f - 1) < 0.01
    assert not m.fit["warnings"]


def test_from_points_overhead_without_height_warns_but_floor_is_right():
    cam = synth.cam_overhead()
    uv, xy, _ = synth.clicks(cam, synth.board_points(20))
    m = CameraModel.from_points(uv, xy, synth.SIZE)
    assert m.fit["rms_px"] < 0.5
    assert any("not fixed" in w for w in m.fit["warnings"])
    assert floor_err(m, cam).max() < 0.1


def test_points_at_a_known_height_fix_the_focal_length():
    cam = synth.cam_overhead()
    uv0, xy0, z0 = synth.clicks(cam, synth.board_points(30))
    uv4, xy4, z4 = synth.clicks(cam, synth.board_points(30, lo=16, hi=106), z=12.0)     # e.g. box tops
    m = CameraModel.from_points(np.vstack([uv0, uv4]), np.vstack([xy0, xy4]), synth.SIZE,
                                floor_z=np.concatenate([z0, z4]))
    assert abs(m.center[2] - 262.0) < 3.0 and not m.fit["warnings"], m.describe()


@pytest.mark.parametrize("make", [synth.cam_overhead, synth.cam_oblique])
def test_from_points_floor_error_with_half_pixel_noise(make):
    cam = make()
    uv, xy, _ = synth.clicks(cam, synth.board_points(20), noise=0.5, seed=3)
    m = CameraModel.from_points(uv, xy, synth.SIZE)
    e = floor_err(m, cam)
    assert e.max() < 0.5, (e.mean(), e.max())
    assert m.fit["loo_rms_cm"] < 0.5 and 0.3 < m.fit["rms_px"] < 0.9


def test_lens_term_fitted_for_distorted_oblique():
    cam = synth.cam_oblique(k1=-0.12)
    uv, xy, _ = synth.clicks(cam, synth.board_points(15), noise=0.3, seed=4)
    m = CameraModel.from_points(uv, xy, synth.SIZE)
    assert m.fit["k1"]["kept"] and abs(m.dist[0] + 0.12) < 0.03
    assert floor_err(m, cam).max() < 0.5


def test_too_few_or_collinear_points_refused():
    cam = synth.cam_oblique()
    uv, xy, _ = synth.clicks(cam, synth.board_points(20))
    with pytest.raises(CalibrationError):
        CameraModel.from_points(uv[:5], xy[:5], synth.SIZE)
    line = np.array([(x, 30.0) for x in range(10, 120, 10)])
    uvl, xyl, _ = synth.clicks(cam, line)
    with pytest.raises(CalibrationError):
        CameraModel.from_points(uvl, xyl, synth.SIZE)


def test_transfer_at_robot_height(fitted):
    A, B, mA, mB = fitted
    pts = both_visible(A, B, 4.0)
    assert len(pts) > 60
    worst = {"true": 0.0, "fitted": 0.0}
    for x, y, z in pts:
        ua, va = A.project((x, y, z))
        ub, vb = B.project((x, y, z))
        for key, (ca, cb) in (("true", (A, B)), ("fitted", (mA, mB))):
            u2, v2, _, vis = transfer(ca, cb, ua, va, 4.0)
            assert vis
            worst[key] = max(worst[key], math.hypot(u2 - ub, v2 - vb))
        # and back, B -> A
        u3, v3, _, _ = transfer(mB, mA, ub, vb, 4.0)
        worst["fitted"] = max(worst["fitted"], math.hypot(u3 - ua, v3 - va))
    assert worst["true"] < 1e-3 and worst["fitted"] < 2.0, worst


def _img_heading(cam, x, y, z, hf):
    """True image heading: direction of a tiny floor step along floor heading hf."""
    h = math.radians(hf)
    u0, v0 = cam.project((x, y, z))
    u1, v1 = cam.project((x + 0.01 * math.cos(h), y + 0.01 * math.sin(h), z))
    return math.degrees(math.atan2(-(v1 - v0), u1 - u0)) % 360


def _ang(a, b):
    return abs((a - b + 180) % 360 - 180)


def test_heading_transfer(fitted):
    A, B, mA, mB = fitted
    worst = 0.0
    for x, y, z in both_visible(A, B, 4.0)[::3]:
        for hf in range(0, 360, 30):
            ua, va = A.project((x, y, z))
            _, _, hb, vis = transfer(mA, mB, ua, va, 4.0, heading_deg=_img_heading(A, x, y, z, hf))
            worst = max(worst, _ang(hb, _img_heading(B, x, y, z, hf)))
            assert _ang(image_heading_to_floor(A, ua, va, _img_heading(A, x, y, z, hf), 4.0), hf) < 0.01
    assert worst < 3.0, worst
    # and the helper going the other way
    assert _ang(floor_heading_to_image(A, 60, 60, 4.0, 90.0, lead_cm=0.01), _img_heading(A, 60, 60, 4.0, 90.0)) < 1e-3


def test_transfer_reports_not_visible():
    A, B = synth.cam_overhead(), synth.cam_oblique()
    # a spot off the board that A sees but B does not (below the near edge of B's image)
    p = (-30.0, 80.0, 4.0)
    ua, va = A.project(p)
    assert A.in_image(ua, va) and not B.in_image(*B.project(p))
    assert transfer(A, B, ua, va, 4.0, heading_deg=10.0)[3] is False
    assert transfer(A, B, *A.project((60.0, 60.0, 4.0)), 4.0)[3] is True
    assert transfer(A, B, 320.0, 240.0, 300.0)[3] is False       # plane above camera A: ray never gets there


def test_save_load_round_trip(tmp_path, fitted):
    _, _, mA, _ = fitted
    p = tmp_path / "cam.json"
    mA.save(p)
    m2 = CameraModel.load(p)
    assert np.allclose(m2.project((30.0, 40.0, 4.0)), mA.project((30.0, 40.0, 4.0)))
    d = json.loads(p.read_text())
    assert d["model"] == "pinhole" and abs(d["camera_position_cm"][2] - 262) < 3 and d["fit"]["rms_px"] > 0
    assert "Lens at" in m2.describe()


# ---------------------------------------------------------------- floor_calibration.json
def _calibration_json(tmp_path, cam, height=None, k1_mode="off", noise=0.3, name="floor_calibration.json"):
    """A floor_calibration.json made the way tools/calibrate_floor.py makes it: clicked floor
    points -> cf.fit -> cf.to_calibration."""
    import calibrate_floor as cf
    uv, xy, _ = synth.clicks(cam, synth.board_points(20), noise=noise, seed=7)
    pts = [dict(id=i + 1, kind="floor", u=float(a), v=float(b), x=float(x), y=float(y), z=0.0, enabled=True)
           for i, ((a, b), (x, y)) in enumerate(zip(uv, xy))]
    res = cf.fit(pts, synth.SIZE, height, None, k1_mode, area=[0, 0, 122, 122])
    p = tmp_path / name
    p.write_text(json.dumps(cf.to_calibration(res)))
    return p


def _px_err(model, true, z=4.0):
    xyz = np.column_stack([GRID, np.full(len(GRID), z)])
    u0, v0 = true.project(xyz)
    u1, v1 = model.project(xyz)
    ok = true.in_image(u0, v0)
    return np.hypot(u0 - u1, v0 - v1)[ok]


@pytest.mark.parametrize("make,height,k1", [(synth.cam_overhead, 262.0, 0.0), (synth.cam_overhead, 262.0, -0.05),
                                            (synth.cam_oblique, None, 0.0), (synth.cam_oblique, 141.4, -0.1)])
def test_from_floor_calibration(tmp_path, make, height, k1):
    cam = make(k1=k1)
    m = CameraModel.from_floor_calibration(_calibration_json(tmp_path, cam, height, "fit" if k1 else "off"))
    e = _px_err(m, cam)
    assert e.max() < 1.0, (e.mean(), e.max())
    assert np.linalg.norm(m.center - cam.center) < 3.0, (m.center, cam.center)
    assert m.fit["method"] == "from_floor_calibration" and not m.fit["warnings"]


def test_from_floor_calibration_needs_height_for_overhead(tmp_path):
    p = _calibration_json(tmp_path, synth.cam_overhead(), None)
    with pytest.raises(CalibrationError, match="--height-cm"):
        CameraModel.from_floor_calibration(p)
    d = json.loads(p.read_text())
    del d["H_floor_to_px"]
    p.write_text(json.dumps(d))
    with pytest.raises(CalibrationError, match="homography"):
        CameraModel.from_floor_calibration(p)


def test_from_floor_calibration_matches_simulator_camera_convention(tmp_path):
    """tools/tests/conftest.py Cam (the simulator's camera model, y axis flipped) -> calibrate_floor
    -> CameraModel: the same pixels for robot tops."""
    spec = importlib.util.spec_from_file_location(
        "floorcal_conftest", os.path.join(synth.TOOLS, "tests", "conftest.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    import calibrate_floor as cf
    cam = mod.Cam(height=140.0, centre=(60.0, 40.0), yaw=30.0, tilt=(10.0, -6.0), k1=-0.04, floor_width=230.0)
    rng = np.random.default_rng(0)
    pts = []
    for i, (x, y) in enumerate([(x, y) for x in range(-10, 131, 15) for y in range(-30, 111, 15)]):
        u, v = cam.project(x, y)
        if 0 <= u < 640 and 0 <= v < 480:
            pts.append(dict(id=i, kind="floor", u=u + rng.normal(0, 0.3), v=v + rng.normal(0, 0.3), x=float(x),
                            y=float(y), z=0.0, enabled=True))
    res = cf.fit(pts, (640, 480), 140.0, None, "fit", area=[-10, -30, 130, 110])
    p = tmp_path / "floor_calibration.json"
    p.write_text(json.dumps(cf.to_calibration(res)))
    m = CameraModel.from_floor_calibration(p)
    worst = 0.0
    for x in range(0, 121, 20):
        for y in range(-20, 101, 20):
            u, v = cam.project(x, y, 4.0)
            if 0 <= u < 640 and 0 <= v < 480:
                worst = max(worst, math.hypot(*(np.array(m.project((x, y, 4.0))) - (u, v))))
    assert worst < 1.5, worst
    assert abs(m.center[2] - 140.0) < 2 and np.hypot(m.center[0] - 60, m.center[1] - 40) < 4, m.center
