"""The corners-only setup (tools/fleet/corner_setup.py, setup_job.py): the measure on drawn cameras, the words for bad
clicks, the checks' agreement rule, the roll's room, the recorder handling, and Mission Control's Setup state."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import corner_setup as CS   # noqa: E402
import setup_job as SJ      # noqa: E402
from cameras import CameraModel   # noqa: E402


def cam(C, look, f, size=(640, 480)):
    z = np.asarray(look, float) - C
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0, 0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    K = np.array([[f, 0, size[0] / 2], [0, f, size[1] / 2], [0, 0, 1.0]])
    return CameraModel(K, [0] * 5, CS._rvec(R), -R @ np.asarray(C, float), size)


TRUE = {"overhead": cam([70, 40, 250], [68, 38, 0], 600), "side": cam([40, -55, 170], [68, 34, 0], 520)}


def make_obs(Lc=137.0, Wc=76.0, click_sd=0.7, spot_sd=0.2, seed=1, side_f=1.0, roll_scale=1.0, body=None,
             cams=None, robots=None):
    rng = np.random.default_rng(seed)
    cams = cams or TRUE
    obs = {"cameras": {}, "spots": {}, "pairs": [], "rolls": [], "bodies": []}
    for c, m in cams.items():
        u, v = m.project(np.array([[0, 0, 0], [Lc, 0, 0], [Lc, Wc, 0], [0, Wc, 0]], float))
        K = m.K.copy()
        if c == "side":
            K[0, 0] *= side_f
            K[1, 1] *= side_f
        obs["cameras"][c] = dict(K=K.tolist(), dist=[0] * 5, size=list(m.image_size),
                                 corners_px=(np.column_stack([u, v]) + rng.normal(0, click_sd, (4, 2))).tolist())
    robots = robots or {"Left": (17, 17, 90), "Right": (Lc - 17, 17, 90), "Center": (Lc / 2, Wc / 2, 30)}
    for n, (x, y, h) in robots.items():
        th = math.radians(h)
        for when, dx in (("start", 0.0), ("far", CS.ROLL_CM)):
            cx, cy = x + dx * math.cos(th), y + dx * math.sin(th)
            for s_, sg in (("L", 1), ("R", -1)):
                p = (cx - sg * math.sin(th) * CS.LIGHT_SPACING_CM / 2, cy + sg * math.cos(th) * CS.LIGHT_SPACING_CM / 2,
                     CS.LIGHT_Z_CM)
                obs["spots"]["%s/%s/%s" % (n, when, s_)] = {
                    c: (np.array(m.project(p)) + rng.normal(0, spot_sd, 2)).tolist() for c, m in cams.items()}
            obs["pairs"].append(("%s/%s/L" % (n, when), "%s/%s/R" % (n, when)))
        for s_ in ("L", "R"):
            obs["rolls"].append(("%s/start/%s" % (n, s_), "%s/far/%s" % (n, s_), CS.ROLL_CM * roll_scale))
        if body:
            half = body / 2
            cth, sth = math.cos(th), math.sin(th)
            pts = [(x + cth * a - sth * b, y + sth * a + cth * b, CS.LIGHT_Z_CM)
                   for a, b in ((half, half), (-half, half), (-half, -half), (half, -half))]
            obs["bodies"].append(dict(robot=n, camera="overhead", z_cm=CS.LIGHT_Z_CM,
                                      corners_px=[list(cams["overhead"].project(q)) for q in pts]))
    return obs


@pytest.mark.parametrize("table", [(122, 61), (128, 72), (137, 76), (122, 122)])
def test_size_and_cameras_come_back(table):
    r = CS.solve(make_obs(*table))
    assert r["ok"], r.get("words")
    assert abs(r["size_cm"][0] - table[0]) < 1.5 and abs(r["size_cm"][1] - table[1]) < 1.0
    assert r["cams_agree_cm"] < 0.5
    for c, m in TRUE.items():
        got = CameraModel.from_dict(r["cameras"][c]["model"])
        for x, y in ((10, 10), (table[0] - 10, table[1] - 10), (table[0] / 2, table[1] / 2)):
            u, v = m.project((x, y, 0.0))
            fx, fy = got.to_floor(u, v)
            assert math.hypot(fx - x, fy - y) < 1.5
    assert "Your table measures %.0f x %.0f cm" % r["size_cm"] in CS.result_line(r)


def test_clockwise_clicks_refused_before_anything_moves():
    obs = make_obs()
    c = obs["cameras"]["overhead"]["corners_px"]
    obs["cameras"]["overhead"]["corners_px"] = [c[0], c[3], c[2], c[1]]       # 2 and 4 swapped: clockwise from above
    r = CS.solve(obs)
    assert not r["ok"]
    assert any("swap 2 and 4" in w for w in r["words"])


def test_pictures_numbered_differently_are_named():
    obs = make_obs(137, 76)
    c = obs["cameras"]["side"]["corners_px"]
    obs["cameras"]["side"]["corners_px"] = c[1:] + c[:1]                     # the side's 1 is the overhead's 2
    r = CS.solve(obs)
    assert not r["ok"]
    assert any("numbered differently" in w for w in r["words"])


def test_side_lens_off_size_from_the_overhead_and_said():
    r = CS.solve(make_obs(122.15, 121.9, side_f=0.8))                       # the side's focal length 20 % short
    assert r["ok"]
    assert abs(r["size_cm"][0] - 122.15) < 2.0 and abs(r["size_cm"][1] - 121.9) < 2.0
    assert r["size_from"] == "overhead"
    assert any("side camera reads" in w for w in r["agree"])
    assert r["cameras"]["side"]["f_refit"] and abs(r["cameras"]["side"]["f_px"] / 520.0 - 1) < 0.03   # refitted


def test_a_check_more_than_2pct_off_is_left_out_and_said():
    r = CS.solve(make_obs(roll_scale=1.05))                                  # wheels read 5 % long
    assert "roll" in r["left_out"] and "lights" in r["used"]
    assert any("rolls on the wheels" in w and "left out" in w for w in r["agree"])
    assert abs(r["size_cm"][1] - 76) < 1.0
    r = CS.solve(make_obs(roll_scale=1.01, body=CS.BODY_CM))                 # within 2 %: used
    assert "roll" in r["used"] and "body" in r["used"]


def test_lights_alone_on_one_camera():
    obs = make_obs(cams={"overhead": TRUE["overhead"]})
    obs["rolls"] = []
    r = CS.solve(obs)
    assert r["ok"] and abs(r["size_cm"][1] - 76) < 1.0


def test_no_lights_no_scale_words():
    obs = make_obs()
    obs.update(spots={}, pairs=[], rolls=[])
    r = CS.solve(obs)
    assert not r["ok"] and any("lights were seen" in w for w in r["words"])


def test_scale_formula_matches_projection():
    p = CS.rect_pose(make_obs(click_sd=0)["cameras"]["overhead"]["corners_px"], TRUE["overhead"].K, [0] * 5, (640, 480))
    a, b = TRUE["overhead"].project((40, 30, 3.7)), TRUE["overhead"].project((47.08, 30, 3.7))
    s = CS.scale_from(p, [(a, b)], 7.08, 3.7)[0]
    assert abs(s - 76.0) < 0.05


def test_roll_order_keeps_room():
    rect = (0, 0, 137, 76)
    assert CS.roll_order((30, 30), 90, rect, {}) == 1
    assert CS.roll_order((30, 64), 90, rect, {}) == -1                       # facing an edge 12 cm away: backwards
    assert CS.roll_order((30, 13), 90, (0, 0, 137, 26), {}) is None          # no room either way
    assert CS.roll_order((30, 30), 90, rect, {"Right": (30, 45)}) == -1      # another robot ahead


def test_light_spots_on_drawn_pictures():
    dark = [np.full((60, 80), 100, np.uint8) for _ in range(3)]
    lit = [d.copy() for d in dark]
    for im in lit:
        im[20:23, 30:33] = 230
    sp = CS.light_spots(dark, lit)
    assert len(sp) == 1 and abs(sp[0][1] - 31) < 0.2 and abs(sp[0][2] - 21) < 0.2


def test_recorder_started_when_not_running(monkeypatch):
    state = {"up": False, "ran": 0}
    monkeypatch.setattr(SJ, "port_answers", lambda port, **k: state["up"])
    monkeypatch.setattr(SJ, "REC_WAIT_S", 2.0)

    def run():
        state["ran"] += 1
        state["up"] = True
    ok, w = SJ.start_recorder(say=lambda s: None, runner=run)
    assert ok and state["ran"] == 1
    ok, w = SJ.start_recorder(say=lambda s: None, runner=run)                 # running: not started again
    assert ok and state["ran"] == 1 and "running" in w


def test_recorder_stopped_for_the_chessboard_way(monkeypatch):
    state = {"up": True}
    monkeypatch.setattr(SJ, "port_answers", lambda port, **k: state["up"])
    monkeypatch.setattr(SJ.time, "sleep", lambda s: None)
    assert SJ.stop_recorder(say=lambda s: None, runner=lambda: state.update(up=False)) is True
    assert SJ.stop_recorder(say=lambda s: None, runner=lambda: None) is False    # not running: nothing to restart


def test_mission_control_state_has_the_corner_setup(tmp_path):
    import mission_control as MC
    S = MC.Session(str(tmp_path))
    st = S.state()
    assert st["setup_way"] == "corners" and "corner_setup" in st
    with pytest.raises(ValueError):
        S.corner.measure({"conn": "sim", "corners": {"overhead": [[0, 0]]}})
    S.corner.keep(way="chessboard")
    assert MC.Session(str(tmp_path)).corner.way == "chessboard"


def test_size_override_and_marked_table(tmp_path):
    import mission_control as MC
    S = MC.Session(str(tmp_path))
    res = {"size_cm": (136.9, 75.8)}
    S.corner.job = {"steps": []}
    S.corner.apply_size(res, {"warmup": True})
    assert S.map["floor"]["x_max"] == 136.9 and S.map["floor"]["y_max"] == 75.8
    S.corner.apply_size(res, {"warmup": True, "size": "137x76"})
    assert S.map["floor"]["x_max"] == 137.0
    f = dict(S.map["floor"])
    S.corner.apply_size({"size_cm": (122.1, 121.8)}, {"warmup": False})       # a table marked on the plywood
    assert S.map["floor"] == f and S.map["surface_cm"] == [122.1, 121.8]
