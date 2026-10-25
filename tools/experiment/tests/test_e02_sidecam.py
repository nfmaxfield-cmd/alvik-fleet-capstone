"""The side camera, offline (2026-10-24, second-camera plan steps 2-3): make_calibration's camera name, putting a
steep view's board corners in order, the calibration from robots seen by both cameras (e02_sidecam), and the motion
tracker's robot size taken at each spot (MotionTracker size_per_spot)."""
import json
import math
import os
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import e02_camera as CAM  # noqa: E402
import e02_sidecam as S  # noqa: E402
from cameras import CameraModel  # noqa: E402

DATA = os.path.join(os.path.dirname(HERE), "data")
ROAM1_CORNERS = [[387.7, 337.6], [430.4, 62.0], [141.1, 6.9], [111.9, 298.9]]   # overhead, session notes Oct 21


def steep_camera(C=(61.0, -30.0, 110.0), target=(61.0, 70.0, 0.0), f=380.0, size=(640, 480)):
    """A pinhole camera at C looking at target (no lens bend): a steep side view like the plan's mount (42 deg from
    straight down, the whole board in the picture), scale ~2.3x near to far."""
    C, T = np.array(C, float), np.array(target, float)
    fw = (T - C) / np.linalg.norm(T - C)
    r = np.cross(fw, [0.0, 0.0, 1.0])
    r /= np.linalg.norm(r)
    d = np.cross(fw, r)
    R = np.array([r, d, fw])
    rvec = cv2.Rodrigues(R)[0].ravel()
    K = np.array([[f, 0, size[0] / 2], [0, f, size[1] / 2], [0, 0, 1.0]])
    return CameraModel(K, np.zeros(5), rvec, -R @ C, size, name="side")


# ----------------------------------------------------------------------------------- make_calibration's name
def test_make_calibration_keeps_overhead_as_the_default_name(tmp_path):
    p = str(tmp_path / "cam.json")
    res = CAM.make_calibration(ROAM1_CORNERS, (640, 480), p, height_cm=262.0)
    assert json.load(open(p))["name"] == "overhead"
    assert res["err_cm"] < 3.0


def test_make_calibration_writes_the_given_name(tmp_path):
    p = str(tmp_path / "side.json")
    CAM.make_calibration(ROAM1_CORNERS, (640, 480), p, height_cm=262.0, name="side")
    d = json.load(open(p))
    assert d["name"] == "side"
    assert CAM.load_camera(p).name == "side"


# ----------------------------------------------------------------------------------- corner order, steep view
def test_order_corners_like_puts_a_steep_views_corners_in_floor_order():
    cam = steep_camera()
    W, L = CAM.BOARD_W_CM, CAM.BOARD_L_CM
    truth = [list(cam.project((x, y, 0.0))) for x, y in ((0, 0), (W, 0), (W, L), (0, L))]
    rough = [[u + 9.0, v - 7.0] for u, v in truth]                     # a rough model, ~11 px off
    for perm in ([2, 0, 3, 1], [3, 2, 1, 0], [1, 3, 0, 2]):
        found = [truth[i] for i in perm]
        out, worst = CAM.order_corners_like(found, rough)
        assert np.allclose(out, truth, atol=0.1)
        assert worst < 15.0


def test_find_board_corners_order_is_wrong_for_a_steep_view_and_order_corners_like_fixes_it():
    """A drawn board seen by the steep camera: find_board_corners finds it, but in picture order; matching to the
    projected corners gives floor order, and make_calibration then puts (61, 61) where the camera sees it."""
    cam = steep_camera()
    W, L = CAM.BOARD_W_CM, CAM.BOARD_L_CM
    truth = np.array([cam.project((x, y, 0.0)) for x, y in ((0, 0), (W, 0), (W, L), (0, L))])
    img = np.full((480, 640), 40, np.uint8)
    cv2.fillPoly(img, [np.round(truth).astype(np.int32)], 190)
    found, why = CAM.find_board_corners(img)
    assert found is not None, why
    out, worst = CAM.order_corners_like(found, truth)
    assert worst < 4.0
    assert not np.allclose(found, out, atol=4.0)             # picture order was not floor order


def test_side_calibration_from_ordered_corners_places_the_middle(tmp_path):
    cam = steep_camera()
    W, L = CAM.BOARD_W_CM, CAM.BOARD_L_CM
    truth = [list(cam.project((x, y, 0.0))) for x, y in ((0, 0), (W, 0), (W, L), (0, L))]
    shuffled = [truth[i] for i in (2, 3, 0, 1)]
    out, _ = CAM.order_corners_like(shuffled, truth)
    p = str(tmp_path / "side.json")
    CAM.make_calibration(out, (640, 480), p, name="side")
    side = CAM.load_camera(p)
    u, v = cam.project((61.0, 61.0, 0.0))
    x, y = side.to_floor(u, v, 0.0)
    assert math.hypot(x - 61.0, y - 61.0) < 1.0


# ----------------------------------------------------------------------------------- calibration from robots
def _synthetic_pairs(cam, n=1500, wrong=0.35, noise_px=1.0, seed=1):
    rng = np.random.default_rng(seed)
    xy = rng.uniform([5, 5], [117, 117], size=(n, 2))
    u, v = cam.project(np.column_stack([xy, np.full(n, S.SIDE_Z_CM)]))
    uv = np.column_stack([u, v]) + rng.normal(0, noise_px, (n, 2))
    bad = rng.random(n) < wrong                              # paired with another robot: anywhere else
    xy_pair = xy.copy()
    xy_pair[bad] = rng.uniform([5, 5], [117, 117], size=(bad.sum(), 2))
    return np.column_stack([xy_pair, uv, np.arange(n) * 0.04])


def test_fit_from_pairs_recovers_a_side_camera_despite_wrong_pairs():
    cam = steep_camera()
    P = _synthetic_pairs(cam)
    fit, info = S.fit_from_pairs(P)
    assert info["homography_inliers"] < len(P)
    errs = []
    for x, y in ((10, 10), (61, 61), (110, 20), (20, 110), (110, 110)):
        u, v = cam.project((x, y, S.SIDE_Z_CM))
        fx, fy = fit.to_floor(u, v, S.SIDE_Z_CM)
        errs.append(math.hypot(fx - x, fy - y))
    assert max(errs) < 1.5, errs


def test_fit_from_pairs_refuses_too_few_pairs():
    cam = steep_camera()
    with pytest.raises(ValueError):
        S.fit_from_pairs(_synthetic_pairs(cam, n=6, wrong=0.0))


def test_find_offset_recovers_the_side_clock():
    """Two robots driving loops; the side camera's frames start 0.6 s after t = 0 at 29 frames a second."""
    cam = steep_camera()
    dt, true_off = 1 / 29.0, 0.6
    rng = np.random.default_rng(3)

    def where(k, t):
        cx, cy, r, w = ((40, 40, 25, 0.5), (80, 75, 30, -0.4))[k]
        return cx + r * math.cos(w * t), cy + r * math.sin(w * t)
    ots = np.arange(0.0, 60.0, 0.04)
    oxy = [[where(k, t) for k in (0, 1)] for t in ots]
    side = {}
    for f in range(int((60 - true_off) / dt)):
        t = true_off + f * dt
        l = []
        for k in (0, 1):
            x, y = where(k, t)
            u, v = cam.project((x, y, S.SIDE_Z_CM))
            l.append((u + rng.normal(0, 0.8), v + rng.normal(0, 0.8), 0.8))
        side[f] = l
    best, _ = S.find_offset(side, ots, oxy, dt, 0.0, 1.2, 0.1)
    assert abs(best - true_off) < 0.11


def test_save_side_calibration_writes_name_and_board_corners(tmp_path):
    cam = steep_camera()
    p = S.save_side_calibration(cam, str(tmp_path / "s.json"), dict(pairs=1), note="test")
    d = json.load(open(p))
    assert d["name"] == "side" and len(d["board_corners_px"]) == 4
    back = CAM.load_camera(p)
    assert np.allclose(back.project((30.0, 40.0, 0.0)), cam.project((30.0, 40.0, 0.0)), atol=1e-6)


# ----------------------------------------------------------------------------------- regions, still moments
def test_region_names():
    assert S.region(115, 115) == "blocked corner"
    assert S.region(118, 50) == "along a board"
    assert S.region(50, 118) == "along a board"
    assert S.region(3, 50) == "open edge"
    assert S.region(61, 61) == "middle"
    assert S.region(-20, 50) == "off"
    assert S.region_split(118, 50) == "board at x=122" and S.region_split(50, 118) == "board at y=122"
    assert S.region_split(3, 50) == "open edge x=0" and S.region_split(50, 3) == "open edge y=0"
    assert S.region_split(61, 61) == "middle"


def test_still_moments_and_side_at_agree_for_a_parked_robot():
    cam = steep_camera()
    ots = np.arange(0.0, 3.0, 0.04)
    oxy = [[(50.0 + (0.0 if t > 1.0 else 20 * (1.0 - t)), 70.0)] for t in ots]   # drives in, parks at (50, 70)
    st = S.still_moments(S.overhead_tracks(ots, oxy))
    assert st and all(abs(x - 50.0) < 0.3 and abs(y - 70.0) < 0.3 for _, x, y in st)
    dt = 1 / 29.0
    u, v = cam.project((50.0, 70.0, S.SIDE_Z_CM))
    side = {f: [(u, v, 1.0)] for f in range(int(3.0 / dt))}
    sx, sy, n = S.side_at(cam, side, st[-1][0], st[-1][1], st[-1][2], 0.0, dt)
    assert math.hypot(sx - 50.0, sy - 70.0) < 0.1 and n >= 2


def test_presence_counts_side_only_where_the_overhead_has_nothing():
    cam = steep_camera()
    dt = 1 / 29.0
    ots = np.arange(0.0, 2.0, 0.04)
    oxy = [[(61.0, 61.0)] for _ in ots]                                 # overhead sees only the middle robot
    side = {}
    for f in range(int(2.0 / dt)):
        side[f] = [(*cam.project((61.0, 61.0, S.SIDE_Z_CM)), 1.0), (*cam.project((115.0, 115.0, S.SIDE_Z_CM)), 1.0)]
    p = S.presence(cam, side, ots, oxy, 0.0, dt, bin_s=0.5)
    assert p["middle"]["both"] >= 3 and p["middle"]["side_only"] == 0
    assert p["blocked corner"]["side_only"] >= 3 and p["blocked corner"]["both"] == 0


# ----------------------------------------------------------------------------------- tracker: size per spot
def test_tracker_default_size_is_unchanged():
    ov = CAM.load_camera(os.path.join(DATA, "overhead_floorcal_roam1.json"))
    t = CAM.MotionTracker(ov)
    assert t.size_per_spot is False
    assert t._rpx_at(5.0, 5.0) == t.rpx and t.area_min == t.area
    assert t.rpx == pytest.approx(CAM.ROBOT_CM * ov.px_per_cm(61.0, 61.0, CAM.ROBOT_TOP_CM))


def _robot_picture(cam, x, y):
    """Floor 120, a dark robot at (x, y) drawn as the box a side camera sees: its top and its near faces."""
    img = np.full((480, 640), 120, np.uint8)
    h = CAM.ROBOT_CM / 2
    pts = [cam.project((x + a, y + b, z)) for a, b in ((-h, -h), (h, -h), (h, h), (-h, h)) for z in (0.0, CAM.ROBOT_TOP_CM)]
    cv2.fillConvexPoly(img, cv2.convexHull(np.round(np.array(pts)).astype(np.int32)), 30)
    return img


def _shaped(cam, x, y, per):
    t = CAM.MotionTracker(cam, z_cm=S.SIDE_Z_CM, size_per_spot=per)
    t.set_background([np.full((480, 640), 120, np.uint8)])
    g = cv2.GaussianBlur(_robot_picture(cam, x, y), (3, 3), 0).astype(np.float32)
    c = [c for c in t.candidates(g) if math.hypot(c["x"] - x, c["y"] - y) < 2.0]
    return bool(c and c[0]["shape"])


@pytest.mark.parametrize("x,y", [(61.0, 3.0), (5.0, 5.0)])
def test_near_the_steep_camera_a_robot_is_refused_with_the_middles_size_and_kept_with_its_own(x, y):
    """Near the steep camera a robot outline is ~2.3x the far one's width: with one size from the board's middle the
    shape check calls it too big; with the size at its own spot it is a robot."""
    cam = steep_camera()
    assert cam.px_per_cm(61, 3, CAM.ROBOT_TOP_CM) / cam.px_per_cm(61, 118, CAM.ROBOT_TOP_CM) > 2.0
    assert _shaped(cam, x, y, per=False) is False
    assert _shaped(cam, x, y, per=True) is True


def test_with_size_per_spot_a_robot_is_robot_shaped_all_over_the_board():
    cam = steep_camera()
    for x, y in ((61, 3), (61, 61), (61, 118), (5, 5), (118, 118), (5, 118), (118, 5)):
        assert _shaped(cam, x, y, per=True), (x, y)


def test_side_floorcal_roam1_file_loads_and_sees_the_board():
    p = os.path.join(DATA, "side_floorcal_roam1.json")
    if not os.path.exists(p):
        pytest.skip("no side calibration file")
    cam = CAM.load_camera(p)
    assert cam.name == "side"
    assert cam.center[2] > 50                                   # above the floor
    u, v = cam.project((61.0, 61.0, 0.0))
    assert cam.in_image(u, v)


# ----------------------------------------------------------------------------------- the real side clock (2026-10-25)
def test_frame_clock_maps_frames_to_times_and_back(tmp_path):
    p = tmp_path / "frames.csv"
    p.write_text("frame,t_wall,t_mono,cam_ms\n0,100.0,0.5,0\n1,100.04,0.54,0\n2,100.2,0.70,0\n3,100.24,0.74,0\n")
    clk = S.FrameClock.from_csv(str(p))
    assert clk(2) == pytest.approx(0.70)
    assert S.side_time(1, clk, None) == pytest.approx(0.54)
    assert S.side_time(1, 0.5, 0.04) == pytest.approx(0.54)          # the fitted form still works
    assert clk.frame_at(0.62) == pytest.approx(1.5)
    assert S._side_frame(0.62, clk, None) == pytest.approx(1.5)


def test_side_at_works_with_the_real_clock():
    cam = steep_camera()
    times = {f: 0.5 + f * 0.0344 for f in range(100)}
    clk = S.FrameClock(times)
    u, v = cam.project((40.0, 50.0, S.SIDE_Z_CM))
    side = {f: [(u, v, 1.0)] for f in range(100)}
    sx, sy, n = S.side_at(cam, side, 2.0, 40.0, 50.0, clk, None)
    assert math.hypot(sx - 40.0, sy - 50.0) < 0.1 and n >= 5


# ----------------------------------------------------------------------------------- calibration from the picture
def _floor_texture(seed=5, px_per_cm=4):
    rng = np.random.default_rng(seed)
    W, L = int(126 * px_per_cm), int(126 * px_per_cm)
    t = rng.normal(0, 1, (L // 4, W // 4)).astype(np.float32)
    t = cv2.resize(cv2.GaussianBlur(t, (0, 0), 1.2), (W, L), interpolation=cv2.INTER_CUBIC)
    t = (t - t.min()) / (np.ptp(t) + 1e-9) * 200 + 30
    return t.astype(np.uint8), px_per_cm


def _render_floor(cam, tex, ppc, off_cm=2.0):
    """The floor texture (pixel (i, j) = floor ((j / ppc) - off_cm, (i / ppc) - off_cm)) as cam sees it (no lens)."""
    R = cam.R
    Hf = cam.K @ np.column_stack([R[:, 0], R[:, 1], cam.tvec])          # floor (x, y, 1) -> image
    T = np.array([[1.0 / ppc, 0, -off_cm], [0, 1.0 / ppc, -off_cm], [0, 0, 1]])
    return cv2.warpPerspective(tex, Hf @ T, tuple(cam.image_size), flags=cv2.INTER_LINEAR, borderValue=20)


def test_grain_pairs_and_fit_recover_the_side_camera_from_the_floor_texture():
    over = steep_camera(C=(61.0, 55.0, 260.0), target=(61.0, 61.0, 0.0), f=560.0)
    side = steep_camera()
    tex, ppc = _floor_texture()
    P, info = S.grain_pairs(_render_floor(over, tex, ppc), _render_floor(side, tex, ppc), over)
    assert info["inliers"] >= 30
    fit = S.fit_from_floor_points(P)
    errs = []
    for x, y in ((10, 10), (61, 61), (110, 20), (20, 110), (110, 110)):
        u, v = side.project((x, y, 0.0))
        fx, fy = fit.to_floor(u, v, 0.0)
        errs.append(math.hypot(fx - x, fy - y))
    assert max(errs) < 1.5, errs


def test_grain_pairs_refuses_pictures_that_do_not_match():
    over = steep_camera(C=(61.0, 55.0, 260.0), target=(61.0, 61.0, 0.0), f=560.0)
    tex, ppc = _floor_texture(seed=5)
    other, _ = _floor_texture(seed=9)
    with pytest.raises(ValueError):
        S.grain_pairs(_render_floor(over, tex, ppc), _render_floor(steep_camera(), other, ppc), over)


def test_tape_check_measures_distance_to_blue_tape_and_counts_edges_without_tape():
    cam = steep_camera()
    img = np.full((480, 640, 3), 90, np.uint8)
    a, b = cam.project((0.0, 0.0, 0.0)), cam.project((0.0, 120.0, 0.0))
    cv2.line(img, (int(round(a[0])) + 8, int(round(a[1]))), (int(round(b[0])) + 8, int(round(b[1]))), (200, 60, 20), 1)
    # blue (BGR) tape 8 px to the right: the nearest tape pixel is 8 px x sin(the edge's slant) away
    out = S.tape_check(img, cam, {"x=0": ((0.0, 0.0), (0.0, 120.0)), "x=W": ((120.0, 0.0), (120.0, 120.0))})
    assert 5.0 <= out["x=0"]["median_px"] <= 8.5 and out["x=0"]["missing"] == 0
    assert out["x=W"]["missing"] == out["x=W"]["n"] and out["x=W"]["median_px"] is None


# ----------------------------------------------------------------------------------- both trackers compared
def test_compare_tracks_counts_by_region():
    so = [(t * 0.2, 61.0, 61.0) if t < 10 else (t * 0.2, None, None) for t in range(20)]
    ss = [(t * 0.2 + 0.01, 61.5, 61.0) if t >= 5 else (t * 0.2 + 0.01, 90.0, 20.0) for t in range(20)]
    c = S.compare_tracks(so, ss)
    m = c["middle"]
    assert m["both"] == 5 and m["disagree"] == 5 and m["side_only"] >= 9 and m["overhead_only"] == 0


class _FakeRec:
    """e02_replay's Recording interface over drawn pictures: a robot box driving along y at 10 cm/s."""

    def __init__(self, cam, n=40, fps=20.0):
        self.cam, self.n, self.fps = cam, n, fps

    def index(self, t):
        return max(0, min(self.n - 1, int(round(t * self.fps))))

    def t_frame(self, i):
        return i / self.fps

    def where(self, i):
        return 61.0, 20.0 + 10.0 * max(0.0, i / self.fps - 0.5)

    def read(self, i):
        if not 0 <= i < self.n:
            return None
        return _robot_picture(self.cam, *self.where(i))


def test_replay_passes_tracker_kw_and_default_is_unchanged():
    import e02_replay as R
    cam = steep_camera()
    rec = _FakeRec(cam)
    # floor memory comes from the frames before t0: start at 0.5 s with the robot at its spot
    smp = R.replay(rec, cam, "robot", rec.where(10), [], 0.5, 1.9, tracker_kw=dict(z_cm=S.SIDE_Z_CM, size_per_spot=True))
    seen = [(t, x, y) for t, x, y in smp if x is not None]
    assert len(seen) >= 0.6 * len(smp)
    t, x, y = seen[-1]
    assert math.hypot(x - rec.where(rec.index(t))[0], y - rec.where(rec.index(t))[1]) < 3.0
    import inspect
    assert inspect.signature(R.replay).parameters["tracker_kw"].default is None
