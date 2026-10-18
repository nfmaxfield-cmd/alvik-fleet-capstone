"""tools/calibrate_floor.py: fitting, outliers, robot-top points, lens term, readers,
held-out (leave-one-run-out) calibrations and the local page. No display needed.

    python -m pytest tools/tests -q
"""
import csv
import json
import math
import os
import threading
import urllib.request

import numpy as np
import pytest

import calibrate_floor as cf
from conftest import Cam, floor_error

AXES = [(x, 0.0) for x in range(-10, 141, 10)] + [(0.0, y) for y in range(-40, 61, 10) if y]
SPREAD = AXES + [(50, 40), (100, 40), (120, -30), (40, -30), (100, -20)]   # plus a few measured dots


def clicks(cam, pts, noise=1.0, seed=0, z=0.0):
    rng = np.random.default_rng(seed)
    out = []
    for i, (x, y) in enumerate(pts):
        u, v = cam.project(x, y, z)
        out.append(dict(id=i + 1, kind="floor" if not z else "robot", u=u + rng.normal(0, noise),
                        v=v + rng.normal(0, noise), x=float(x), y=float(y), z=z, enabled=True))
    return out


def test_axes_clicks_recover_floor_within_half_cm():
    cam = Cam()
    res = cf.fit(clicks(cam, SPREAD), (640, 480), k1_mode="off")
    cal = cf.to_calibration(res)
    e = floor_error(cal, cam)
    assert e.mean() < 0.3 and e.max() < 0.8, (e.mean(), e.max())
    assert res["rms_cm"] < 0.6
    assert not any(p["flagged"] for p in res["points"])


def test_bad_click_is_flagged_and_left_out():
    cam = Cam()
    pts = clicks(cam, SPREAD)
    pts[5]["x"] += 10.0                        # typed 60 instead of 50
    res = cf.fit(pts, (640, 480), k1_mode="off")
    bad = [p for p in res["points"] if p["flagged"]]
    assert [p["id"] for p in bad] == [6]
    assert not res["points"][5]["inlier"]
    assert floor_error(cf.to_calibration(res), cam).max() < 0.8


def test_points_on_one_line_refused():
    cam = Cam()
    with pytest.raises(cf.FitError):
        cf.fit(clicks(cam, [(x, 0) for x in range(0, 100, 10)]), (640, 480), k1_mode="off")
    with pytest.raises(cf.FitError):
        cf.fit(clicks(cam, [(0, 0), (10, 0), (0, 10)]), (640, 480))


E01_AREA = [-10, -40, 130, 60]     # the simulated camera views the E01 floor


def test_coverage_warning_for_axes_only():
    res = cf.fit(clicks(Cam(), AXES), (640, 480), k1_mode="off", area=E01_AREA)
    assert any("cover only" in w for w in res["warnings"])
    res = cf.fit(clicks(Cam(), SPREAD + [(130, 60), (-10, 60), (130, -40), (-10, -40)]), (640, 480), k1_mode="off",
                 area=E01_AREA)
    assert not any("cover only" in w for w in res["warnings"])


def test_default_area_from_fleet_map(tmp_path):
    p = tmp_path / "map.json"
    assert cf.default_area(str(p)) == [0.0, 0.0, 122.0, 122.0]          # no map: the plywood default
    p.write_text(json.dumps({"floor": {"x_min": -5, "x_max": 118.5, "y_min": 0, "y_max": 121}}))
    assert cf.default_area(str(p)) == [-5.0, 0.0, 118.5, 121.0]
    p.write_text(json.dumps({"floor": {"x_min": 5, "x_max": 1}}))      # broken: fall back
    assert cf.default_area(str(p)) == [0.0, 0.0, 122.0, 122.0]


def test_robot_top_points_need_and_use_camera_height():
    """E00 robot mode: tape-measured centres paired with the robot's top (4 cm up)."""
    cam = Cam()
    spots = [(x, y) for x in (0, 30, 60, 90, 120) for y in (-30, 0, 30, 55)]
    pts = clicks(cam, spots, z=4.0, seed=1)
    no_h = cf.fit(pts, (640, 480), k1_mode="off")
    assert any("without a camera height" in w for w in no_h["warnings"])
    with_h = cf.fit(pts, (640, 480), height_cm=115.0, k1_mode="off")
    e_no = floor_error(cf.to_calibration(no_h), cam)
    e_h = floor_error(cf.to_calibration(with_h), cam)
    assert e_h.mean() < 0.3 and e_h.mean() < 0.5 * e_no.mean(), (e_h.mean(), e_no.mean())
    assert "estimated" in with_h["nadir_source"]
    assert math.dist(with_h["nadir"], cam.nadir()) < 3.0
    # robot top seen through the calibration (as tools/fusion does) lands on the true floor point
    assert floor_error(cf.to_calibration(with_h), cam, z=4.0).mean() < 0.3
    given = cf.fit(pts, (640, 480), height_cm=115.0, nadir=cam.nadir(), k1_mode="off")
    assert given["nadir_source"].startswith("plumb")


def test_lens_term_found_when_edges_bend():
    cam = Cam(k1=-0.08)
    grid = [(x, y) for x in range(-10, 141, 15) for y in range(-40, 61, 15)]
    pts = clicks(cam, grid, noise=0.7, seed=2)
    off = cf.fit(pts, (640, 480), height_cm=115.0, k1_mode="off")
    auto = cf.fit(pts, (640, 480), height_cm=115.0, k1_mode="auto")
    assert auto["k1"] < -0.03 and auto["k1_info"]["k1_kept"]
    e_off, e_auto = floor_error(cf.to_calibration(off), cam), floor_error(cf.to_calibration(auto), cam)
    assert e_auto.mean() < 0.6 * e_off.mean(), (e_auto.mean(), e_off.mean())
    # no distortion -> auto leaves k1 at 0
    flat = cf.fit(clicks(Cam(), grid, noise=0.7, seed=2), (640, 480), height_cm=115.0, k1_mode="auto")
    assert flat["k1"] == 0.0


def test_file_readable_by_fusion_and_detector(tmp_path):
    import runs as rn
    import detcore
    cam = Cam(k1=-0.08)
    grid = [(x, y) for x in range(-10, 141, 15) for y in range(-40, 61, 15)]
    res = cf.fit(clicks(cam, grid, noise=0.3, seed=3), (640, 480), height_cm=115.0, k1_mode="fit")
    p = tmp_path / "floor_calibration.json"
    p.write_text(json.dumps(cf.to_calibration(res)))
    H, Hinv = rn.load_homography(str(p))
    par = rn.load_parallax(str(p))
    dist = rn.load_distortion(str(p))
    assert par is not None and dist is not None
    to_cm = detcore.load_homography(str(p))
    for x, y in ((0, 0), (120, 50), (-5, -35)):
        u, v = cam.project(x, y)
        assert math.dist(to_cm(u, v), (x, y)) < 0.6
        assert math.dist(rn.apply_h(Hinv, *rn.undistort_px(u, v, dist)), (x, y)) < 0.6
        ut, vt = cam.project(x, y, 4.0)    # the robot's top
        assert math.dist(rn.floor_from_top(*rn.apply_h(Hinv, *rn.undistort_px(ut, vt, dist)), par), (x, y)) < 0.6


def test_readers_unchanged_without_k1(tmp_path):
    import runs as rn
    import detcore
    H = [[3.0, 0.1, 100.0], [0.05, -3.0, 400.0], [0.0, 0.0, 1.0]]
    p = tmp_path / "c.json"
    p.write_text(json.dumps({"H_floor_to_px": H}))
    assert rn.load_distortion(str(p)) is None
    assert rn.undistort_px(10.0, 20.0, None) == (10.0, 20.0)
    to_cm = detcore.load_homography(str(p))
    u, v = rn.apply_h(H, 10.0, 10.0)
    assert to_cm(u, v) == pytest.approx((10.0, 10.0))
    p.write_text(json.dumps({"H_floor_to_px": H, "k1": 0.0, "cx": 320, "cy": 240, "f_px": 460}))
    assert rn.load_distortion(str(p)) is None
    assert detcore.load_homography(str(p))(u, v) == pytest.approx((10.0, 10.0))


def test_view_polygon_contains_what_the_camera_sees():
    cam = Cam()
    res = cf.fit(clicks(cam, SPREAD), (640, 480), height_cm=115.0, k1_mode="off")
    cal = cf.to_calibration(res)
    poly = np.array(cal["view_floor_polygon_cm"], np.float32)
    import cv2
    assert cv2.pointPolygonTest(poly, (60.0, 10.0), False) > 0         # under the image centre
    assert cv2.pointPolygonTest(poly, (60.0, 200.0), False) < 0        # far outside the view
    # the view is about 160 x 120 cm
    assert 150 * 110 < cv2.contourArea(poly) < 175 * 135
    assert "view_robot_polygon_cm" in cal


def test_sheet_spots_e00_format(tmp_path):
    src = os.path.join(os.path.dirname(cf.__file__), "..", "docs", "experiments", "E00-measurement-check", "data",
                       "measurements.csv")
    rows = list(csv.DictReader(open(src)))
    rows[0].update(xf_cm="15.0", yf_cm="20.0", xr_cm="5.0", yr_cm="20.0")
    rows[1].update(xf_cm="50", yf_cm="10", xr_cm="50", yr_cm="0", excluded="x")
    p = tmp_path / "m.csv"
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    spots = cf.read_sheet_spots(str(p))
    assert len(spots) == 1 and spots[0]["x"] == 10.0 and spots[0]["y"] == 20.0 and spots[0]["heading"] == 0.0
    ev = [dict(frame=100, text="trial 1 Left P03 run x start"), dict(frame=300, text="trial 1 run x end done")]
    cf.spot_frames(spots, ev)
    assert spots[0]["frame"] == 300


def test_snap_to_detection():
    dets = {f: [(100.0 + 0.3 * (f % 2), 200.0), (300.0, 50.0)] for f in range(40)}
    s = cf.snap_to_detection(dets, 20, 104, 197)
    assert s and abs(s[0] - 100.15) < 0.2 and s[2] >= 20
    assert cf.snap_to_detection(dets, 20, 200, 200) is None


def test_leave_one_run_out_has_no_circularity(tmp_path):
    """Each held-out file is made without its run; the manifest says which file scores which run."""
    cam = Cam()
    rng = np.random.default_rng(4)
    pts = []
    for k in range(8):
        for which, (x, y) in (("start", (rng.uniform(-5, 125), rng.uniform(-35, 55))),
                              ("end", (rng.uniform(-5, 125), rng.uniform(-35, 55)))):
            u, v = cam.project(x, y, 4.0)
            pts.append(dict(id=len(pts) + 1, kind="run", u=u + rng.normal(0, 1), v=v + rng.normal(0, 1), x=x, y=y,
                            z=4.0, group=f"R{k}", spot=f"R{k}-{which}", enabled=True))
    out = tmp_path / "floor_calibration.json"

    def fit_without(excl):
        return cf.fit([p for p in pts if p["group"] not in excl], (640, 480), 115.0, None, "off")
    man, rows = cf.write_loro(str(out), [p["group"] for p in pts], fit_without, cf.points_heldout(pts), log=lambda *_: 0)
    assert len(rows) == 8
    for r in rows:
        cal = json.load(open(tmp_path / r["calibration"]))
        assert r["run_id"] not in cal["fit"]["used_runs"] and cal["held_out_runs"] == [r["run_id"]]
        assert r["heldout_start_err_cm"] < 1.5 and r["heldout_end_err_cm"] < 1.5
    man2, rows2 = cf.write_loro(str(out), [p["group"] for p in pts], fit_without, cf.points_heldout(pts), folds=3,
                                log=lambda *_: 0)
    assert len({r["calibration"] for r in rows2}) == 3 and len(rows2) == 8


def test_page_server_round_trip(tmp_path):
    """The local page's API: state, add points, fit, save (no browser)."""
    import cv2
    from http.server import ThreadingHTTPServer
    cam = Cam()
    img = np.full((480, 640, 3), 140, np.uint8)
    cv2.imwrite(str(tmp_path / "calib_frame.jpg"), img)
    src = cf.Source(str(tmp_path))
    app = cf.App(src, str(tmp_path / "floor_points.json"), str(tmp_path / "floor_calibration.json"),
                 dict(height_cm=115.0, nadir_floor_cm=None, robot_top_cm=4.0, k1="auto", area=None), [], [], None)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), cf.make_handler(app))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        st = json.load(urllib.request.urlopen(base + "/api/state"))
        assert st["size"] == [640, 480] and st["ok"] is False
        assert urllib.request.urlopen(base + "/frame?i=0").read()[:2] == b"\xff\xd8"
        assert b"Floor calibration" in urllib.request.urlopen(base + "/").read()
        body = json.dumps(dict(points=clicks(cam, SPREAD), settings={})).encode()
        req = urllib.request.Request(base + "/api/points", body, {"Content-Type": "application/json"})
        r = json.load(urllib.request.urlopen(req))
        assert r["ok"] and r["fit"]["rms_cm"] < 0.6 and r["fit"]["grid"]
        r = json.load(urllib.request.urlopen(urllib.request.Request(base + "/api/save", b"{}")))
        assert r["ok"] and os.path.isfile(r["path"]) and os.path.isfile(r["check"])
        # resume: a new App reads the saved points
        app2 = cf.App(src, str(tmp_path / "floor_points.json"), str(tmp_path / "x.json"), {}, [], [], None)
        assert len(app2.points) == len(SPREAD) and app2.settings["height_cm"] == 115.0
    finally:
        srv.shutdown()


def test_e00_both_passes_at_a_spot_keep_their_heldout_errors(tmp_path):
    """E00: rows 1 and 13 are the same placement spot (one group, left out together).
    Each pass's held-out error has its own column (keyed by sheet row), none overwritten."""
    cam = Cam()
    rng = np.random.default_rng(5)
    pts = []
    spots = [(rng.uniform(0, 120), rng.uniform(-35, 55)) for _ in range(8)]
    for k, (x, y) in enumerate(spots):
        for order in (k + 1, k + 13):                       # two passes, two sheet rows, same spot
            u, v = cam.project(x, y, 4.0)
            du = 6.0 if order == 13 else 0.0                # spot P0's second pass clicked 6 px off
            pts.append(dict(id=len(pts) + 1, kind="robot", u=u + du, v=v, x=x, y=y, z=4.0, group=f"spot P{k}",
                            spot=f"{order}-end", enabled=True))

    def fit_without(excl):
        return cf.fit([p for p in pts if p["group"] not in excl], (640, 480), 115.0, None, "off")
    man, rows = cf.write_loro(str(tmp_path / "floor_calibration.json"), [p["group"] for p in pts], fit_without,
                              cf.points_heldout(pts), log=lambda *_: 0)
    r0 = next(r for r in rows if r["run_id"] == "spot P0")
    assert "heldout_1_end_err_cm" in r0 and "heldout_13_end_err_cm" in r0
    assert r0["heldout_13_end_err_cm"] > r0["heldout_1_end_err_cm"] + 0.5       # the 6 px pass is clearly worse
    header = open(man).readline()
    assert "heldout_end_err_cm" not in header
    # run points keep the documented names
    rp = [dict(id=1, u=100, v=100, x=0, y=0, z=0, group="ab12-3", spot="ab12-3-start"),
          dict(id=2, u=200, v=100, x=10, y=0, z=0, group="ab12-3", spot="ab12-3-end")]
    keys = cf.points_heldout(rp)(fit_without(set()), "ab12-3")
    assert set(keys) == {"heldout_start_err_cm", "heldout_end_err_cm"}


def test_leave_one_robot_out(tmp_path):
    """--holdout robot: one calibration per robot, made without any of its runs; one row per run."""
    cam = Cam()
    rng = np.random.default_rng(6)
    robot_of = {f"R{k}": ("Left", "Center", "Right")[k % 3] for k in range(9)}
    pts = []
    for rid in robot_of:
        for which in ("start", "end"):
            x, y = rng.uniform(-5, 125), rng.uniform(-35, 55)
            u, v = cam.project(x, y, 4.0)
            pts.append(dict(id=len(pts) + 1, kind="run", u=u, v=v, x=x, y=y, z=4.0, group=rid,
                            spot=f"{rid}-{which}", enabled=True))
    seen = []

    def fit_without(excl):
        seen.append(set(excl))
        res = cf.fit([p for p in pts if p["group"] not in excl], (640, 480), 115.0, None, "off")
        res["used_runs"] = sorted({p["group"] for p in pts if p["group"] not in excl})
        res["used_robots"] = sorted({robot_of[r] for r in res["used_runs"]})
        return res
    members = {}
    for rid, rb in robot_of.items():
        members.setdefault(rb, []).append(rid)
    man, rows = cf.write_loro(str(tmp_path / "floor_calibration.json"), list(members), fit_without,
                              cf.points_heldout(pts), members=members, holdout="robot", log=lambda *_: 0)
    assert os.path.basename(man) == "floor_calibration_loro_robot.csv" and len(rows) == 9
    assert {frozenset(s) for s in seen} == {frozenset(v) for v in members.values()}
    for r in rows:
        cal = json.load(open(tmp_path / r["calibration"]))
        assert robot_of[r["run_id"]] not in cal["fit"]["used_robots"]
        assert cal["held_out_robots"] == [robot_of[r["run_id"]]] and r["holdout"] == "robot"
        assert not set(members[robot_of[r["run_id"]]]) & set(cal["fit"]["used_runs"])


def test_fleet_map_found_in_current_folder_or_given(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(cf, "FLEET_MAP", str(tmp_path / "nowhere" / "map.json"))
    monkeypatch.setattr(cf, "MAP_PATH", None)
    assert cf.find_map() is None and cf.default_area() == [0.0, 0.0, 122.0, 122.0]
    (tmp_path / "map.json").write_text(json.dumps({"floor": {"x_min": 0, "x_max": 150, "y_min": 0, "y_max": 90}}))
    assert cf.find_map() == str(tmp_path / "map.json")              # tools/fleet/app.py's default spot
    assert "map.json" in capsys.readouterr().out
    assert cf.default_area() == [0.0, 0.0, 150.0, 90.0]
    other = tmp_path / "other.json"
    other.write_text(json.dumps({"floor": {"x_min": 0, "x_max": 100, "y_min": 0, "y_max": 100}}))
    monkeypatch.setattr(cf, "MAP_PATH", cf.find_map(str(other)))
    assert cf.default_area() == [0.0, 0.0, 100.0, 100.0] and "--map" in capsys.readouterr().out
    with pytest.raises(SystemExit):
        cf.find_map(str(tmp_path / "missing.json"))


BOM_SHEET = ("order,robot,level,commands,run_id,target_x_cm,target_y_cm,target_heading_deg,xf0_cm,yf0_cm,xr0_cm,"
             "yr0_cm,xf_cm,yf_cm,xr_cm,yr_cm,battery_pct,excluded,note\n"
             "1,Left,1,run,ab12-1,10,0,0,4.5,0,-4.5,0,14.5,0,5.5,0,90,,\n")


def test_excel_bom_sheet_read_everywhere(tmp_path):
    """Excel's "CSV UTF-8" starts with a BOM: the first column must still be 'order'."""
    import tape_results
    import runs as rn
    folder = tmp_path / "E01"
    (folder / "data").mkdir(parents=True)
    sheet = folder / "data" / "measurements.csv"
    sheet.write_bytes(b"\xef\xbb\xbf" + BOM_SHEET.encode("utf-8"))
    spots = cf.read_sheet_spots(str(sheet))
    assert spots and spots[0]["order"] == "1" and spots[0]["key"] == "1-start"
    trials = tape_results.load_trials(str(folder))
    assert len(trials) == 1 and trials[0]["order"] == "1"
    sess = tmp_path / "session"
    sess.mkdir()
    (sess / "runinfo.csv").write_text("robot,t_recv,run,experiment\nLeft,1000,ab12-1,E1\n")
    (sess / "raw.csv").write_text("robot,t_recv,run,ms\nLeft,1000.0,ab12-1,0\nLeft,1000.1,ab12-1,100\n")
    runs, _ = rn.load_runs(str(sess), str(sheet))
    assert runs[0].tape_start == (0.0, 0.0, 0.0) and runs[0].level == "1"


def test_make_run_order_reads_bom_and_writes_plain_utf8(tmp_path):
    import subprocess
    import sys
    import make_run_order as mro
    out = tmp_path / "m.csv"
    out.write_bytes(b"\xef\xbb\xbf" + BOM_SHEET.encode())          # a measured sheet saved from Excel
    cmd = [sys.executable, mro.__file__, sorted(mro.EXPERIMENTS)[0], "--out", str(out)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    assert "already has 1 measured trials" in r.stdout, r.stdout + r.stderr     # it saw xf_cm through the BOM
    r = subprocess.run(cmd + ["--force"], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert out.read_bytes()[:5] == b"order"                          # written back without a BOM


def test_second_server_on_same_port_is_refused():
    import socket
    import sys
    s1 = cf.make_server(0, cf.make_handler(None))
    port = s1.server_address[1]
    try:
        assert cf.LocalServer.allow_reuse_address == (os.name != "nt")
        with pytest.raises(SystemExit, match="already running"):
            cf.make_server(port, cf.make_handler(None))
        sys.path.insert(0, os.path.join(os.path.dirname(cf.__file__), "fleet"))
        import app
        assert app.LocalServer.allow_reuse_address == (os.name != "nt")
        with pytest.raises(SystemExit, match="already running"):
            app.serve(None, port)
    finally:
        s1.server_close()
