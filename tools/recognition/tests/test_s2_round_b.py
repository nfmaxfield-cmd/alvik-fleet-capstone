"""Stage 2 fix round B, studio side: floor calibration made in the studio (a rectangle or measured points,
mirrored axes found by themselves, the fit error in cm, a grid for the eye, stored where every reader looks,
undo), positions already in picture pixels, which clock a positions file uses and Line up by a moment, the
guesses before "Line up again" kept for Z, generic home-position presets (a tray's slots kept together), the
Alvik dock only for Alvik projects, stopping a training (Z), taking a recording back out (Z), the object
types' undo, plain words in the report (one row per guess source, no "near a person" without people)."""
import csv
import json
import math
import os
import sys
import threading
import time
import types

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
for p in (REC, os.path.join(TOOLS, "label"), os.path.join(TOOLS, "detector"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import cameras  # noqa: E402
import crosscam  # noqa: E402
import floorcal  # noqa: E402
import homes as HM  # noqa: E402
import outside as O  # noqa: E402
import outsidelink  # noqa: E402
import project as projmod  # noqa: E402
import report  # noqa: E402
import studio as studiomod  # noqa: E402
import trainer  # noqa: E402
import use_model as um  # noqa: E402
from test_studio import fake_recording, write_labels  # noqa: E402

W, H = 640, 480


def _look(C, T):
    z = np.asarray(T, float) - C
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0, 0])
    x /= np.linalg.norm(x)
    return np.vstack([x, np.cross(z, x), z])


def true_camera(C=(40.0, -60.0, 120.0), T=(40.0, 40.0, 0.0), f=520.0):
    C = np.asarray(C, float)
    R = _look(C, T)
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])
    return cameras.CameraModel(K, None, cv2.Rodrigues(R)[0].ravel(), -R @ C, (W, H))


def rect_clicks(cam, w=60.0, ln=42.0, order=(0, 1, 2, 3)):
    fl = np.array([[0, 0, 0], [w, 0, 0], [w, ln, 0], [0, ln, 0]], float)
    u, v = cam.project(fl)
    pts = np.column_stack([u, v])
    return [pts[i].tolist() for i in order]


# =============================================================================== cameras: mirrored axes
def test_a_mirrored_floor_is_found_and_every_reader_uses_the_persons_axes():
    cam = true_camera()
    rng = np.random.default_rng(3)
    P = np.column_stack([rng.uniform(0, 80, 10), rng.uniform(0, 60, 10), np.zeros(10)])
    u, v = cam.project(P)
    img = np.column_stack([u, v])
    mir = P[:, :2] * [1, -1]                         # the person measured y the other way round
    with pytest.raises(cameras.CalibrationError):
        cameras.CameraModel.from_points(img, mir, (W, H), k1="off")          # as before: plainly refused
    m = cameras.CameraModel.from_points(img, mir, (W, H), k1="off", mirror="auto")
    assert m.mirror_y and m.fit["mirror"]["used"] and "above the floor" in m.fit["mirror"]["why"]
    assert m.center[2] > 100                                                 # the camera is above the floor
    x, y = m.to_floor(u, v)
    assert np.allclose(np.column_stack([x, y]), mir, atol=0.05)
    uu, vv = m.project(np.column_stack([mir, np.zeros(10)]))
    assert np.allclose(uu, u, atol=0.05) and np.allclose(vv, v, atol=0.05)
    assert m.fit["points"][0]["y"] == pytest.approx(mir[0, 1])               # stored in the person's numbers
    m2 = cameras.CameraModel.from_dict(json.loads(json.dumps(m.to_dict())))
    assert m2.mirror_y and np.allclose(m2.to_floor(u[:3], v[:3])[1], mir[:3, 1], atol=0.05)
    # heading conversions and the camera-to-camera transfer keep the person's axes too
    h = cameras.image_heading_to_floor(m2, u[0], v[0], cameras.floor_heading_to_image(m2, mir[0, 0], mir[0, 1], 0, 30.0))
    assert h == pytest.approx(30.0, abs=0.5)
    ub, vb, _, vis = cameras.transfer(m2, m, u[1], v[1], 0.0)
    assert vis and abs(ub - u[1]) < 0.1 and abs(vb - v[1]) < 0.1
    # files without the flag mean what they meant
    d = cam.to_dict()
    assert "mirror_y" not in d and not cameras.CameraModel.from_dict(d).mirror_y


# =============================================================================== floor calibration
def test_rectangle_fit_its_error_grid_and_axes():
    cam = true_camera()
    res = floorcal.fit(dict(mode="rectangle", corners_px=rect_clicks(cam), width_cm=60, length_cm=42), (W, H))
    assert res["ok"] and res["err_cm"] < 0.2 and not res["mirrored"]
    assert "corner 1 to corner 2" in res["axes_words"] and "Fit error about" in res["text"]
    x, y = res["model"].to_floor(*rect_clicks(cam)[2])
    assert (x, y) == pytest.approx((60, 42), abs=0.2)
    lines = res["grid"]["lines"]
    assert len(lines) >= 6 and all(len(l["pts"]) >= 2 for l in lines)
    assert res["axes"]["origin"] == pytest.approx(rect_clicks(cam)[0], abs=0.5)
    # clicked the other way round: a mirror image, found by itself, still right in the person's numbers
    rev = floorcal.fit(dict(mode="rectangle", corners_px=rect_clicks(cam, order=(0, 3, 2, 1)), width_cm=42, length_cm=60), (W, H))
    assert rev["mirrored"] and rev["err_cm"] < 0.2 and "mirror image" in rev["axes_words"]
    x, y = rev["model"].to_floor(*rect_clicks(cam)[1])                   # the true (60, 0) is the person's (0, 60)
    assert (x, y) == pytest.approx((0, 60), abs=0.2)
    # a mis-click shows as an error; corners that cross are refused in plain words
    c = rect_clicks(cam)
    c[2][0] += 40
    bad = floorcal.fit(dict(mode="rectangle", corners_px=c, width_cm=60, length_cm=42), (W, H))
    assert bad["err_cm"] > res["err_cm"]
    with pytest.raises(ValueError, match="cross"):
        floorcal.fit(dict(mode="rectangle", corners_px=rect_clicks(cam, order=(0, 2, 1, 3)), width_cm=60, length_cm=42), (W, H))
    with pytest.raises(ValueError, match="4 corners"):
        floorcal.fit(dict(mode="rectangle", corners_px=c[:3], width_cm=60, length_cm=42), (W, H))
    with pytest.raises(ValueError, match="width and length"):
        floorcal.fit(dict(mode="rectangle", corners_px=c, width_cm=0, length_cm=42), (W, H))


def test_measured_points_find_a_wrong_number_and_a_straight_down_camera_says_its_height_is_guessed():
    cam = true_camera()
    rng = np.random.default_rng(7)
    P = np.column_stack([rng.uniform(0, 80, 9), rng.uniform(0, 60, 9)])
    u, v = cam.project(np.column_stack([P, np.zeros(9)]))
    px = (np.column_stack([u, v]) + rng.normal(0, 0.4, (9, 2))).tolist()
    good = floorcal.fit(dict(mode="points", points_px=px, points_cm=P.tolist()), (W, H))
    assert good["ok"] and good["err_cm"] < 1.0 and good["worst"] is None
    assert all(p["loo_cm"] is not None for p in good["points"])
    wrong = P.copy()
    wrong[4] += [9.0, 0]
    bad = floorcal.fit(dict(mode="points", points_px=px, points_cm=wrong.tolist()), (W, H))
    assert not bad["ok"] and bad["worst"]["index"] == 4 and any("Point 5" in n for n in bad["notes"])
    with pytest.raises(ValueError, match="at least 6"):
        floorcal.fit(dict(mode="points", points_px=px[:5], points_cm=P[:5].tolist()), (W, H))
    with pytest.raises(ValueError, match="x of point 2"):
        floorcal.fit(dict(mode="points", points_px=px, points_cm=[P[0].tolist(), ["", 3]] + P[2:].tolist()), (W, H))
    # straight down, no lens height: floor spots right, the height plainly guessed; with the height it's measured
    down = true_camera(C=(40, 30, 200), T=(40.01, 30.3, 0))
    res = floorcal.fit(dict(mode="rectangle", corners_px=rect_clicks(down), width_cm=60, length_cm=42), (W, H))
    assert res["err_cm"] < 0.3 and any("guessed" in n for n in res["notes"])
    res2 = floorcal.fit(dict(mode="rectangle", corners_px=rect_clicks(down), width_cm=60, length_cm=42, height_cm=200), (W, H))
    assert abs(res2["model"].center[2] - 200) < 3 and not any("guessed" in n for n in res2["notes"])


def test_saved_where_every_reader_looks_and_put_back(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    fake_recording(str(tmp_path / "r1"))
    p.add_recording(str(tmp_path / "r1"))
    rec = p.recordings()[0]
    assert floorcal.summary(p, "default") is None
    cam = true_camera()
    spec = dict(mode="rectangle", corners_px=rect_clicks(cam), width_cm=60, length_cm=42)
    res, before = floorcal.save(p, "default", spec, (W, H))
    assert before is None and os.path.exists(os.path.join(p.folder, "cameras", "default.json"))
    sm = floorcal.summary(p, "default")
    assert sm["made_here"] and sm["method"] == "rectangle" and sm["spec"]["width_cm"] == 60 and sm["err_cm"] < 0.2
    # every reader: outside data, use_model, the camera file loader
    fm, path = outsidelink.floor_camera(p, rec)
    assert fm is not None and path.endswith("default.json")
    um_cam = um.load_camera_model(p, "", rec["path"])
    assert um_cam is not None and um_cam.to_floor(*rect_clicks(cam)[2]) == pytest.approx((60, 42), abs=0.3)
    assert cameras.CameraModel.load(path).to_floor(*rect_clicks(cam)[1]) == pytest.approx((60, 0), abs=0.3)
    # a second save keeps the first for Z; put() puts it back, and None takes it away
    res2, before2 = floorcal.save(p, "default", dict(spec, width_cm=61), (W, H))
    assert before2["studio_calibration"]["spec"]["width_cm"] == 60
    was = floorcal.put(p, "default", before2)
    assert was["studio_calibration"]["spec"]["width_cm"] == 61 and floorcal.summary(p, "default")["spec"]["width_cm"] == 60
    floorcal.put(p, "default", None)
    assert floorcal.summary(p, "default") is None and outsidelink.floor_camera(p, rec) == (None, None)
    with pytest.raises(Exception):
        floorcal.put(p, "default", dict(nothing="here"))                 # only a camera file can be put back
    # the studio's own calls
    st = studiomod.Studio(p, log=lambda *a: None, background=False)
    out = st.floorcal_save("default", spec, (W, H))
    assert out["before"] is None and out["result"]["ok"] and "model" not in out["result"] and out["reopened"] is False
    assert st.floorcal_view()["current"]["made_here"] and st.state()["camera"]["floorcal"]["method"] == "rectangle"
    assert st.state()["camera"]["floor"] is True
    back = st.floorcal_put("default", out["before"])
    assert back["summary"] is None and back["before"]["name"] == "default"


def test_no_floor_text_points_at_the_studio_screen(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    fake_recording(str(tmp_path / "r1"))
    p.add_recording(str(tmp_path / "r1"), "overhead")
    t = outsidelink.no_floor_text(p, p.recordings()[0])
    assert "Floor calibration" in t and "camera overhead" in t and "CROSSCAM" not in t and "cameras.py" not in t
    assert "picture pixels" in t and "Nothing was changed" in t


# =============================================================================== positions in pixels, the clock
def _csv(path, rows, cols=("t", "name", "x", "y", "heading")):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in rows:
            w.writerow(r)


def test_positions_in_picture_pixels_need_no_calibration(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    fake_recording(str(tmp_path / "r1"), n=50)
    p.add_recording(str(tmp_path / "r1"))
    rec = p.recordings()[0]
    f = str(tmp_path / "pos.csv")
    _csv(f, [(1000.0 + k / 10, "A", 100 + k, 200, 90.0) for k in range(50)])
    data = outsidelink.read_data(p, rec, f, None, 0.0, "px")
    st = data.streams["A"]
    assert st.frame == "floor" and st.heading[0] == pytest.approx(-90.0)    # stored in the floor's turning sense
    pm, words = outsidelink.floor_camera(p, rec, dict(units="px"))
    assert isinstance(pm, outsidelink.PixelMap) and "no floor calibration" in words and pm.image_size == (640, 480)
    assert pm.project((12.0, 34.0, 4.0)) == (12.0, 34.0) and pm.to_floor(5.0, 6.0) == (5.0, 6.0)
    assert not pm.in_image(700.0, 10.0) and pm.in_image(10.0, 10.0)
    # the heading comes out as written (0 = right, counter-clockwise on screen): 90 = up the picture
    assert O.image_heading(pm, 100, 200, 0, float(st.heading[0])) == pytest.approx(90.0)
    s = outsidelink.save_settings(p, rec["id"], f, "csv", 0.0, "px")
    assert s["units"] == "px" and outsidelink.read_settings(p, rec["id"])["units"] == "px"
    assert "units" not in outsidelink.save_settings(p, rec["id"], f, "csv", 0.0, "cm")     # cm: the file as before
    # a robot's own path can't be given in pixels: said plainly
    g = str(tmp_path / "own.csv")
    _csv(g, [(1000.0 + k / 10, "R", k, 0, "", "own") for k in range(20)], cols=("t", "name", "x", "y", "heading", "frame"))
    with pytest.raises(ValueError, match="own path"):
        outsidelink.read_data(p, rec, g, None, 0.0, "px")
    # the studio takes pixel positions without a floor calibration (and still refuses cm positions without one)
    st_ = studiomod.Studio(p, log=lambda *a: None, background=False)
    st_.outside_build = lambda: None
    out = st_.outside_set(f, None, 0.0, "px")
    assert out["units"] == "px" and out["names"] == ["A"]
    with pytest.raises(ValueError, match="Floor calibration"):
        st_.outside_set(f, None, 0.0, "cm")
    with pytest.raises(ValueError, match="number of seconds"):
        st_.outside_set(f, None, "soon", "px")


def test_the_clock_is_said_plainly_and_a_moment_lines_the_data_up(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    fake_recording(str(tmp_path / "r1"), n=100, t0=1760000000.0)
    p.add_recording(str(tmp_path / "r1"))
    rec = p.recordings()[0]
    ck = outsidelink.clock_info(rec)
    assert ck["ok"] and ck["t_first"] == pytest.approx(1760000000.0) and not ck["estimated"]
    assert "t_wall" in ck["text"] and "frames.csv" in ck["text"] and "1760000000.00" in ck["text"]
    meta = json.load(open(os.path.join(rec["path"], "meta.json")))
    meta.update(device="video file")
    json.dump(meta, open(os.path.join(rec["path"], "meta.json"), "w"))
    ck = outsidelink.clock_info(rec)
    assert ck["estimated"] and "estimated from the file's time" in ck["text"] and "Line up by a moment" in ck["text"]
    # the data's own clock starts at 500 s; frame 30 (t_wall t0 + 3.0) is when the data says 512.5
    f = str(tmp_path / "pos.csv")
    _csv(f, [(500.0 + k / 10, "A", k, 0, "") for k in range(200)])
    st = dict(path=f, plugin="csv")
    off, words = outsidelink.offset_from_moment(p, rec, st, 30, "512.5", "file")
    assert off == pytest.approx(1760000003.0 - 512.5, abs=1e-3) and "Frame 30" in words
    off2, words2 = outsidelink.offset_from_moment(p, rec, st, 30, 12.5, "from_start")
    assert off2 == pytest.approx(off, abs=1e-3) and "after the data's first sample" in words2
    with pytest.raises(ValueError, match="number of seconds"):
        outsidelink.offset_from_moment(p, rec, st, 30, "noon", "file")
    with pytest.raises(ValueError, match="no time"):
        outsidelink.offset_from_moment(p, rec, st, 5000, "1", "file")
    data = outsidelink.read_data(p, rec, f, None, off)
    assert data.streams["A"].t[125] == pytest.approx(1760000003.0, abs=1e-3)
    assert outsidelink.time_words("csv").startswith("the t column") and "t_recv" in outsidelink.time_words("alvik")
    # the studio: set it (typed or by a moment), with the old value kept for Z; a failed line-up keeps the old one
    sto = studiomod.Studio(p, log=lambda *a: None, background=False)
    p.data["current"]["rec"] = rec["id"]
    outsidelink.save_settings(p, rec["id"], f, "csv", 1.5)
    sto.outside_build = lambda: None
    r = sto.outside_offset(frame=30, log_time="512.5")
    assert r["before"] == 1.5 and r["offset_s"] == pytest.approx(off, abs=1e-3)
    assert outsidelink.read_settings(p, rec["id"])["offset_s"] == pytest.approx(off, abs=1e-3)
    sto.outside_build = lambda: (_ for _ in ()).throw(ValueError("could not"))
    with pytest.raises(ValueError):
        sto.outside_offset(offset_s=9.0)
    assert outsidelink.read_settings(p, rec["id"])["offset_s"] == pytest.approx(off, abs=1e-3)
    det = sto.outside_details()
    assert det["clock"]["ok"] and det["offset_s"] == pytest.approx(off, abs=1e-3) and det["time_words"]


def test_line_up_again_keeps_the_guesses_before_for_z(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    d = p.labels_dir("r")
    os.makedirs(d)
    with pytest.raises(ValueError, match="not kept"):
        outsidelink.swap_drafts(p, "r")
    dp = os.path.join(d, outsidelink.DRAFTS_FILE)
    json.dump(dict(n=1, made="old", frames={}), open(dp, "w"))
    json.dump(dict(n=2, made="new", frames={}), open(dp + outsidelink.BEFORE, "w"))
    assert outsidelink.swap_drafts(p, "r")["made"] == "new"
    assert outsidelink.swap_drafts(p, "r")["made"] == "old"                  # its own reverse (Shift+Z)
    assert json.load(open(dp + outsidelink.BEFORE))["made"] == "new"


# =============================================================================== home presets
def test_a_tray_is_one_home_per_slot_kept_together():
    corners = [[100, 100], [400, 100], [400, 250], [100, 250]]
    hs = outsidelink.tray_homes("Tray", "default", corners, 2, 3, robot_px=30)
    assert [h["name"] for h in hs] == ["Tray A1", "Tray A2", "Tray A3", "Tray B1", "Tray B2", "Tray B3"]
    assert all(h["group"] == "Tray" and h["preset"] == "tray" and h["facing_deg"] is None for h in hs)
    assert (hs[0]["x"], hs[0]["y"]) == pytest.approx((150, 137.5)) and (hs[5]["x"], hs[5]["y"]) == pytest.approx((350, 212.5))
    assert 4 <= hs[0]["radius"] <= 18
    faced = outsidelink.tray_homes("T2", "default", corners, 1, 2, facing=True, cls="box")
    assert faced[0]["facing_deg"] == pytest.approx(0.0, abs=0.5) and faced[0]["cls"] == "box"
    # the group survives saving into the project (make_home keeps it)
    data = {}
    stored = HM.set_homes(data, "default", hs)
    assert stored[0]["group"] == "Tray"
    with pytest.raises(ValueError, match="At most"):
        outsidelink.tray_homes("Big", "default", corners, 8, 8)
    with pytest.raises(ValueError, match="4 corners"):
        outsidelink.tray_homes("T", "default", corners[:3], 1, 1)


def test_presets_are_generic_and_the_alvik_dock_only_for_alvik_projects(tmp_path):
    ids = [p["id"] for p in outsidelink.HOME_PRESETS]
    assert ids[:3] == ["spot", "spot_facing", "tray"] and "alvik_dock" in ids
    assert all(not p["needs_floor"] for p in outsidelink.HOME_PRESETS if p["kind"] == "generic")
    assert all("touch-and-go" not in json.dumps(p) and "post station" not in json.dumps(p) for p in outsidelink.HOME_PRESETS)
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    p.set_classes([dict(name="box"), dict(name="unknown")])
    st = studiomod.Studio(p, log=lambda *a: None, background=False)
    assert "alvik_dock" not in [x["id"] for x in st.home_presets()]
    p.set_classes([dict(name="alvik", has_front=True), dict(name="unknown")])
    assert "alvik_dock" in [x["id"] for x in st.home_presets()]
    # the dock preset gets its group too
    cam = true_camera(C=(40, 30, 200), T=(40.01, 30.3, 0))
    homes = outsidelink.preset_homes("alvik_dock", "Dock 1", "default", cam, cam.project((40, 30, 6.0)),
                                     cam.project((50, 30, 6.0)), stations=True)
    assert len(homes) == 4 and {h["group"] for h in homes} == {"Dock 1"}
    assert all("touch-and-go" not in (h.get("note") or "") for h in homes)


# =============================================================================== training: Z stops it
def test_stopping_a_training_marks_it_stopped_by_the_person(tmp_path, monkeypatch):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    t = trainer.Trainer(p, log=lambda *a: None)

    def slow(self, mid, minutes, python, entry, mdir, logf):
        self._call([sys.executable, "-c", "import time; time.sleep(30)"], logf, 60)
    monkeypatch.setattr(trainer.Trainer, "_train", slow)
    assert t.stop() is False                                    # nothing running: nothing to stop
    mid = t.start(1.0)
    for _ in range(100):
        if t.proc is not None:
            break
        time.sleep(0.05)
    assert t.stop("v99") is False and t.busy()                   # another model: not this one
    assert t.stop(mid) is True
    for _ in range(200):
        if not t.busy():
            break
        time.sleep(0.05)
    assert t.job["status"] == "failed" and "you stopped it" in t.job["message"]
    m = p.model(mid)
    assert m["status"] == "failed" and m["interrupted"] and "you stopped it" in m["error"]
    assert projmod.Project.model_failed(m)


# =============================================================================== Add recording: Z takes it out
def test_a_recording_with_no_work_can_be_taken_out_again(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    for k in (1, 2):
        fake_recording(str(tmp_path / ("r%d" % k)))
    a = p.add_recording(str(tmp_path / "r1"))[0]
    b = p.add_recording(str(tmp_path / "r2"))[0]
    write_labels(p.labels_dir(a), [])
    json.dump(dict(frames={"3": dict(state="reviewed")}), open(os.path.join(p.labels_dir(a), "review.json"), "w"))
    st = studiomod.Studio(p, log=lambda *a: None, background=False)
    opened = []
    st.open = lambda rid, model="none", wait=False: opened.append(rid)
    out = st.remove_recordings([a, b], open_rec=a)
    assert [r["id"] for r in out["removed"]] == [b] and out["kept"][0]["id"] == a and "1 finished frame" in out["kept"][0]["why"]
    assert [r["id"] for r in p.recordings()] == [a] and os.path.isdir(str(tmp_path / "r2"))
    assert opened == [] and out["reopened"] is False                # the open one (a) was not taken out
    # the open recording taken out: another one is opened instead (the one the page names)
    c = p.add_recording(str(tmp_path / "r2"))[0]
    p.data["current"]["rec"] = c
    out = st.remove_recordings([c], open_rec=a)
    assert out["reopened"] and opened == [a] and p.data["current"]["rec"] == a
    assert st.state()["work_area_asked"] == {"default": False}


# =============================================================================== object types: Z
def test_saving_object_types_returns_the_old_ones_for_z(tmp_path):
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    p.set_classes([dict(name="box", max_speed=4.0), dict(name="unknown")])
    st = studiomod.Studio(p, log=lambda *a: None, background=False)
    st.reopen = lambda **kw: False
    get, post = studiomod.make_routes(st)
    sent = {}
    h = types.SimpleNamespace(_send=lambda code, body, *a: sent.update(code=code, body=body))
    assert post(h, "/api/studio/classes", dict(classes=[dict(name="box"), dict(name="ball"), dict(name="unknown")]))
    before = sent["body"]["before"]
    assert [c["name"] for c in before] == ["box", "unknown"] and before[0]["max_speed"] == 4.0
    post(h, "/api/studio/classes", dict(classes=before))                    # Z: back as they were
    assert p.class_names() == ["box", "unknown"] and p.classes[0]["max_speed"] == 4.0


# =============================================================================== the report's words
def test_time_saved_names_each_guess_source_once_and_no_people_row_without_people(tmp_path):
    assert report._model_name("none", "motion") != report._model_name("none", "motion+log")
    assert "positions log" in report._model_name("none", "motion+log")
    p = projmod.Project(str(tmp_path / "p"), create_name="P")
    p.set_classes([dict(name="box"), dict(name="unknown")])
    os.makedirs(os.path.join(p.folder, "models"), exist_ok=True)
    rows = [dict(situation=k, name=n, n_labels=30, n_objects=5, found=0.5, few=False, centre_err_px=1.0, heading_err_deg=None)
            for k, n in (("alone", "On its own"), ("close", "Close together"), ("person", "Near a person"), ("all", "All"))]
    json.dump(dict(models={"v1": rows, "none": rows}), open(os.path.join(p.folder, "models", "situations.json"), "w"))
    html = report.situations_html(p, dict(model="v1", f1=0.5))
    assert "On its own" in html and "Near a person</td>" not in html
    p.set_classes([dict(name="box"), dict(name="hand"), dict(name="unknown")])
    assert "Near a person</td>" in report.situations_html(p, dict(model="v1", f1=0.5))


def test_the_live_page_keeps_its_messages_off_the_picture():
    src = open(os.path.join(REC, "use_model.py"), encoding="utf-8").read()
    assert '<div id="stage"><canvas id="view" width="640" height="480"></canvas><div id="toasts"></div></div>' in src
    assert "position:fixed; left:14px; bottom:14px" not in src
    assert "positions log" in um.__doc__ and "Floor calibration" in um.__doc__
