"""Landmarks picked on the video (tools/landmark_pick.py, calibrate_floor.py
Landmarks mode / --landmarks-from): a synthetic camera 262 cm above the floor,
a floor calibration fitted from its clicks, known boards and blocks projected
to pixels and fed back as clicks; the saved landmarks must come back within
0.5 cm and load with tools/fusion/landmarks.py.

    python -m pytest tools/tests -q
"""
import csv
import json
import math
import threading
import urllib.request

import numpy as np
import pytest

import calibrate_floor as cf
import landmark_pick as lp
import landmarks as LM
from conftest import Cam

HEIGHT = 262.0
GRID = [(x, y) for x in range(0, 123, 20) for y in range(0, 123, 20)]


def make_cam(k1=0.0):
    return Cam(height=HEIGHT, centre=(55.0, 66.0), yaw=2.0, tilt=(0.8, -0.5), k1=k1, floor_width=200.0)


def make_calibration(tmp_path, cam, k1_mode="off"):
    pts = []
    for i, (x, y) in enumerate(GRID):
        u, v = cam.project(x, y)
        pts.append(dict(id=i + 1, kind="floor", u=u, v=v, x=float(x), y=float(y), z=0.0, enabled=True))
    res = cf.fit(pts, (640, 480), height_cm=HEIGHT, k1_mode=k1_mode, area=[0, 0, 122, 122])
    path = tmp_path / "floor_calibration.json"
    path.write_text(json.dumps(cf.to_calibration(res), default=cf._jsonable))
    return str(path)


# true layout: boards by centre line, blocks by footprint
BOARDS = {"A": ((20.0, 5.0), (60.0, 5.0), 2.0),          # along x near the bottom edge
          "B": ((110.0, 30.0), (112.0, 80.0), 1.5),      # along y, far side, slightly turned
          "C": ((15.0, 100.0), (45.0, 115.0), 2.0)}       # diagonal
BLOCK = [(70.0, 40.0), (89.5, 43.0), (88.2, 52.5), (68.7, 49.5)]   # cinderblock-ish, turned ~9 deg
BLOCK_H = 19.4


def near_face(cam, p1, p2, t):
    """The foot the camera sees: the long face on the nadir's side."""
    p1, p2 = np.array(p1), np.array(p2)
    u = (p2 - p1) / np.hypot(*(p2 - p1))
    n = np.array([-u[1], u[0]])
    s = np.sign((np.array(cam.nadir()) - p1) @ n)
    return p1 + s * n * t / 2, p2 + s * n * t / 2


def picks_for(cam, top_block=False):
    out = []
    for nm, (p1, p2, t) in BOARDS.items():
        a, b = near_face(cam, p1, p2, t)
        out.append(dict(name=nm, kind="board", thickness_cm=t, clicks=[list(cam.project(*a)), list(cam.project(*b))],
                        frame=10))
    z = BLOCK_H if top_block else 0.0
    out.append(dict(name="cinderblock", kind="block", clicked_at="top" if top_block else "floor", height_cm=BLOCK_H,
                    clicks=[list(cam.project(x, y, z)) for x, y in BLOCK]))
    return dict(landmarks=out)


def true_faces():
    m = dict(boards=[dict(name=k, p1=list(v[0]), p2=list(v[1]), thickness_cm=v[2]) for k, v in BOARDS.items()],
             obstacles=[dict(id="cinderblock", shape="polygon", source="user", points=[list(p) for p in BLOCK])])
    return {f.name: f for f in LM.faces_from_map(m)}


def assert_faces_match(faces, tol=0.5):
    want = true_faces()
    got = {f.name: f for f in faces}
    assert set(want) <= set(got), set(want) - set(got)
    for nm, f in want.items():
        g = got[nm]
        e = max(np.hypot(*(g.a - f.a)), np.hypot(*(g.b - f.b)))
        assert e < tol, (nm, e, g.a, f.a, g.b, f.b)


@pytest.mark.parametrize("top_block", [False, True])
def test_cli_clicks_to_landmarks_within_half_cm(tmp_path, top_block):
    cam = make_cam()
    cal = make_calibration(tmp_path, cam)
    picks = tmp_path / "picks.json"
    picks.write_text(json.dumps(picks_for(cam, top_block)))
    out = tmp_path / "landmarks.csv"
    r = cf.main([str(tmp_path), "--landmarks-from", str(picks), "--out", cal, "--landmarks-out", str(out)])
    assert r["ok"], r
    faces = LM.load(str(out))
    assert len(faces) == 3 * 4 + 4
    assert_faces_match(faces)
    rows = list(csv.DictReader(open(out)))
    assert {r["source"] for r in rows} == {"user-picked"}
    assert {r["landmark"] for r in rows} == {"A", "B", "C", "cinderblock"}


def test_lens_distortion_calibration(tmp_path):
    cam = make_cam(k1=-0.12)
    cal, _ = lp.load_calibration(make_calibration(tmp_path, cam, k1_mode="fit"))
    pk = picks_for(cam)
    pk["landmarks"][0]["height_cm"] = 12.0                            # board top drawn, for checking by eye
    res = lp.compute_all(cal, pk)
    assert all(r["ok"] for r in res), [r["errors"] for r in res]
    top = np.array(res[0]["top_outline"])
    want = [cam.project(*c, 12.0) for c in lp.board_rect(*BOARDS["A"])]
    assert np.abs(top - want).max() < 0.5 and res[1]["top_outline"] is None
    assert_faces_match([LM.Face(n, a, b) for r in res for n, a, b in r["faces"]])


def test_map_update_and_fusion_reads_it(tmp_path):
    cam = make_cam()
    cal = make_calibration(tmp_path, cam)
    mp = tmp_path / "map.json"
    mp.write_text(json.dumps({"floor": {"x_min": 0, "x_max": 122, "y_min": 0, "y_max": 122},
                              "boards": [{"name": "A", "p1": [0, 0], "p2": [10, 0], "side": "left", "dock_at": 0.3}],
                              "obstacles": [{"id": "cup", "shape": "circle", "center": [80, 40], "radius": 4,
                                             "source": "user"},
                                            {"id": "old", "shape": "polygon", "points": [[1, 1], [5, 1], [5, 5]],
                                             "source": "user", "note": "user-picked earlier"}]}))
    picks = tmp_path / "picks.json"
    picks.write_text(json.dumps(picks_for(cam)))
    r = cf.main([str(tmp_path), "--landmarks-from", str(picks), "--out", cal, "--map", str(mp)])
    assert r["ok"] and (tmp_path / "landmarks.csv").is_file() and (tmp_path / "map.json.bak").is_file()
    m = json.loads(mp.read_text())
    a = next(b for b in m["boards"] if b["name"] == "A")
    assert a["side"] == "left" and a["dock_at"] == 0.3            # docking settings kept
    ids = [o["id"] for o in m["obstacles"]]
    assert "cup" in ids and "old" not in ids and "cinderblock" in ids  # picked-then-deleted block removed
    assert_faces_match(LM.load(str(mp)))


def test_tape_correction_in_inches_replaces_click(tmp_path):
    cam = make_cam()
    cal, _ = lp.load_calibration(make_calibration(tmp_path, cam))
    pk = picks_for(cam)
    a = pk["landmarks"][0]
    a["clicks"][1][0] += 15.0                                       # a bad click, ~6 cm off ...
    a["fix"] = [None, "23 5/8, 2 1/16"]                              # ... fixed by tape: (60.01, 5.24) cm
    r = lp.compute(cal, a)
    e = r["ends"][1]
    assert r["ok"] and abs(e["x"] - 60.01) < 0.01 and abs(e["y"] - 5.24) < 0.01
    assert e["diff_cm"] > 2 and any("tape and your click" in w for w in r["warnings"])
    a["fix"] = [None, "23 5 8, 2"]                                   # typo: refused, click used, said so
    r = lp.compute(cal, a)
    assert not r["ok"] and r["ends"][1]["tape_error"] and r["ends"][1]["tape"] is None


def test_inches_and_tape_parsing():
    assert lp.inches("36 3/32") == pytest.approx(36.09375)
    assert lp.inches("36-3/32") == pytest.approx(36.09375)
    assert lp.inches('12.5"') == 12.5 and lp.inches("3/8") == 0.375
    for bad in ("36 32", "3/2", "1 8/8", "abc", ""):
        assert lp.inches(bad) is None, bad
    assert lp.parse_tape("10, 20") == pytest.approx((25.4, 50.8))
    assert lp.parse_tape("40.2, 31.75 cm") == pytest.approx((40.2, 31.75))
    assert lp.parse_tape([1, 2]) == (1.0, 2.0)
    with pytest.raises(ValueError):
        lp.parse_tape("10 20")
    assert lp.to_in(2.54 * 15.75) == '15 3/4"'


def test_block_footprints():
    sq = lp.block_footprint([(0, 0), (10, 5)])
    assert [list(c) for c in sq] == [[0, 0], [10, 0], [10, 5], [0, 5]]
    three = lp.block_footprint([(0, 0), (10, 0), (10, 5)])
    assert list(three[3]) == [0, 5]
    four = lp.block_footprint([(0, 0), (10, 5), (10, 0), (0, 5)])   # clicked out of order: sorted round
    assert len({tuple(c) for c in four}) == 4
    f = LM.faces_from_map(dict(boards=[], obstacles=[dict(id="b", shape="polygon", source="user",
                                                          points=[list(c) for c in four])]))
    assert sum(fc.length for fc in f) == pytest.approx(30.0)          # perimeter, no crossing edges


def test_problems_said_plainly(tmp_path):
    cam = make_cam()
    r = lp.compute(None, dict(name="A", kind="board", clicks=[[1, 2]]))
    assert not r["ok"]
    r = lp.save_all(dict(landmarks=[]), str(tmp_path / "nope.json"), str(tmp_path / "l.csv"))
    assert not r["ok"] and "No floor calibration" in r["error"]
    cal, _ = lp.load_calibration(make_calibration(tmp_path, cam))
    res = lp.compute_all(cal, dict(landmarks=[dict(name="A", kind="board", clicks=[[100, 100]]),
                                              dict(name="a", kind="block", clicked_at="top", clicks=[[1, 1], [9, 9]])]))
    assert any("both ends" in e for e in res[0]["errors"]) and any("two landmarks" in e for e in res[0]["errors"])
    assert any("height" in e for e in res[1]["errors"])


def test_existing_rows_from_elsewhere_kept(tmp_path):
    cam = make_cam()
    cal = make_calibration(tmp_path, cam)
    out = tmp_path / "landmarks.csv"
    out.write_text("name,x1_cm,y1_cm,x2_cm,y2_cm\nwall,0,130,122,130\nA left,1,1,2,2\n")
    r = lp.save_all(picks_for(cam), cal, str(out))
    assert r["ok"]
    names = [f.name for f in LM.load(str(out))]
    assert "wall" in names and names.count("A left") == 1               # old A replaced, the wall kept
    assert_faces_match(LM.load(str(out)))


def test_page_landmarks_round_trip(tmp_path):
    import cv2
    from http.server import ThreadingHTTPServer
    cam = make_cam()
    cv2.imwrite(str(tmp_path / "calib_frame.jpg"), np.full((480, 640, 3), 120, np.uint8))
    src = cf.Source(str(tmp_path))
    app = cf.App(src, str(tmp_path / "floor_points.json"), str(tmp_path / "floor_calibration.json"),
                 dict(height_cm=None, nadir_floor_cm=None, robot_top_cm=4.0, k1="off", area=[0, 0, 122, 122]), [], [],
                 None)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), cf.make_handler(app))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"

    def post(path, body):
        req = urllib.request.Request(base + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
        return json.load(urllib.request.urlopen(req))
    try:
        assert b"Landmarks" in urllib.request.urlopen(base + "/").read()
        r = post("/api/landmarks", picks_for(cam))
        assert r["cal_ok"] is False and "No floor calibration" in r["cal_error"]    # said plainly, clicks kept
        assert (tmp_path / "landmarks_picked.json").is_file()
        make_calibration(tmp_path, cam)
        r = json.load(urllib.request.urlopen(base + "/api/landmarks"))
        assert r["cal_ok"] and all(x["ok"] for x in r["results"]) and r["results"][0]["outline"]
        e = r["results"][0]["ends"][0]
        assert e["x_in"].endswith('"')
        r = post("/api/landmarks/save", {})
        assert r["ok"], r
        assert_faces_match(LM.load(str(tmp_path / "landmarks.csv")))
        app2 = cf.App(src, str(tmp_path / "fp.json"), str(tmp_path / "floor_calibration.json"), {}, [], [], None)
        assert len(app2.landmarks_json()["picks"]["landmarks"]) == 4                 # resumes
    finally:
        srv.shutdown()
