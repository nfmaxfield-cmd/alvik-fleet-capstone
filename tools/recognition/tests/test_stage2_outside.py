"""Outside data (outside.py), the Alvik robot-log plug-in (robotlogs.py) and home positions
(homes.py), on synthetic paths and on a synthetic roam session with matching robot logs
(synth_logs.py: gyro scale ~0.925, drift ~0.3 deg/s, wheel scale ~0.98, slip, clock offsets, lost rows).

    python -m pytest tools/recognition/tests/test_stage2_outside.py -q
"""
import csv
import json
import math
import os
import shutil

import numpy as np
import pytest

import synth
import synth_logs as SL
from recognition import homes as HM
from recognition import outside as O
from recognition import robotlogs as RL
from recognition.cameras import CameraModel

H_ALVIK = 4.0


# =============================================================================== helpers
def own_path(seed=0, seconds=60.0, g_true=0.925, drift=0.3, wheel=0.98, start=(40.0, 50.0, 30.0), t0=1000.0,
             sign=1.0):
    """A robot driving straight stretches and turns on the floor, and its own (wrong) dead reckoning.
    Returns (stream, truth dict of arrays t, x, y, h)."""
    rng = np.random.default_rng(seed)
    dt = 0.02
    x, y, h = start
    t = t0
    T, X, Y, Hh, D, R = [], [], [], [], [], []
    dist, raw_h = 0.0, 17.0
    plan = []
    while sum(p[1] for p in plan) < seconds:
        plan += [("drive", rng.uniform(1.0, 3.0), rng.choice([8.0, 10.0, -6.0])), ("turn", 1.0, rng.uniform(-120, 120)),
                 ("stop", rng.uniform(0.2, 1.0), 0.0)]
    for kind, secs, val in plan:
        for _ in range(int(secs / dt)):
            v = val if kind == "drive" else 0.0
            w = val if kind == "turn" else 0.0
            T.append(t)
            X.append(x)
            Y.append(y)
            Hh.append(h)
            D.append(dist)
            R.append(raw_h)
            hm = math.radians(h + w * dt / 2)
            x += v * dt * math.cos(hm)
            y += v * dt * math.sin(hm)
            h += w * dt
            dist += v * dt / wheel * (1 + rng.normal(0, 0.01))
            raw_h += sign * (g_true * w + drift) * dt
            t += dt
    st = O.Stream("R", T, dist=D, heading=R, frame="own", cls="alvik",
                  priors=dict(g=sign * 1.0, g_sd=0.1, d=0.0, d_sd=0.5, s=1.0, s_sd=0.05))
    return st, dict(t=np.array(T), x=np.array(X), y=np.array(Y), h=np.array(Hh))


def truth_at(tr, ts):
    return (np.interp(ts, tr["t"], tr["x"]), np.interp(ts, tr["t"], tr["y"]),
            np.degrees(np.interp(ts, tr["t"], np.unwrap(np.radians(tr["h"])))) % 360)


def herr(a, b):
    return np.abs((np.asarray(a) - np.asarray(b) + 180.0) % 360.0 - 180.0)


@pytest.fixture(scope="module")
def cam():
    return synth.cam_overhead()


@pytest.fixture(scope="module")
def sess(tmp_path_factory):
    out = str(tmp_path_factory.mktemp("roam"))
    return SL.make_session(out, seconds=160.0, seed=5, visit_at=75.0)


# =============================================================================== outside.py: alignment
def test_full_anchor_places_the_path_exactly_at_the_anchor():
    st, tr = own_path()
    t_a = st.t[500]
    x, y, h = truth_at(tr, t_a)
    al = O.Aligner(st).add(O.Anchor(t_a, x, y, h, sd_cm=0.2, sd_deg=0.5, kind="home"))
    p = al.pose_at([t_a, t_a + 2.0])
    assert p["ok"].all()
    assert math.hypot(p["x"][0] - x, p["y"][0] - y) < 0.05
    assert herr(p["heading"][0], h) < 0.1
    assert p["sd_cm"][1] > p["sd_cm"][0]           # confidence falls away from the anchor


def test_two_position_anchors_fix_the_rotation_and_one_does_not():
    st, tr = own_path(drift=0.0, g_true=1.0, wheel=1.0)
    ta, tb = st.t[300], st.t[1200]
    xa, ya, _ = truth_at(tr, ta)
    xb, yb, _ = truth_at(tr, tb)
    al = O.Aligner(st).add(O.Anchor(ta, xa, ya, None, sd_cm=0.3))
    assert not al.pose_at([ta + 1])["ok"][0]        # one position: the rotation is unknown
    al.add(O.Anchor(tb, xb, yb, None, sd_cm=0.3))
    ts = st.t[300:1200:50]
    p = al.pose_at(ts)
    x, y, h = truth_at(tr, ts)
    assert p["ok"].all()
    assert np.max(np.hypot(p["x"] - x, p["y"] - y)) < 1.5
    assert np.max(herr(p["heading"], h)) < 3.0


def test_heading_anchor_alone_gives_rotation():
    st, tr = own_path(drift=0.0, g_true=1.0, wheel=1.0)
    ta = st.t[300]
    x, y, h = truth_at(tr, ta)
    al = O.Aligner(st).add(O.Anchor(ta, x, y, None, sd_cm=0.3), O.Anchor(ta + 0.01, None, None, h, sd_deg=1.0,
                                                                           kind="motion"))
    p = al.pose_at([ta + 3.0])
    xt, yt, _ = truth_at(tr, ta + 3.0)
    assert p["ok"][0] and math.hypot(p["x"][0] - xt, p["y"][0] - yt) < 1.0


def test_realignment_at_a_later_anchor_removes_the_drift():
    st, tr = own_path(seconds=80, drift=0.5)
    t0, t1 = st.t[10], st.t[-10]
    a0 = O.Anchor(t0, *truth_at(tr, t0), sd_cm=0.2, sd_deg=0.5, kind="home")
    a1 = O.Anchor(t1, *truth_at(tr, t1), sd_cm=0.2, sd_deg=0.5, kind="home")
    only_start = O.Aligner(st).add(a0)
    both = O.Aligner(st).add(a0, a1)
    tq = np.array([t1 - 1.0])
    x, y, _ = truth_at(tr, tq)
    e_start = math.hypot(only_start.pose_at(tq)["x"][0] - x[0], only_start.pose_at(tq)["y"][0] - y[0])
    e_both = math.hypot(both.pose_at(tq)["x"][0] - x[0], both.pose_at(tq)["y"][0] - y[0])
    assert e_both < 1.0 < e_start


def test_causal_mode_uses_only_earlier_anchors():
    st, tr = own_path()
    ta = st.t[-100]
    al = O.Aligner(st).add(O.Anchor(ta, *truth_at(tr, ta), sd_cm=0.2, sd_deg=0.5, kind="home"))
    assert al.pose_at([ta - 5])["ok"][0]
    assert not al.pose_at([ta - 5], causal=True)["ok"][0]
    assert al.pose_at([ta + 0.5], causal=True)["ok"][0]


def test_segments_keep_anchors_apart():
    st, tr = own_path()
    st.segment[len(st) // 2:] = 1                  # carried in the middle
    ta = st.t[100]
    al = O.Aligner(st).add(O.Anchor(ta, *truth_at(tr, ta), sd_cm=0.2, sd_deg=0.5, kind="home"))
    p = al.pose_at([st.t[200], st.t[-50]])
    assert p["ok"][0] and not p["ok"][1]


def test_lifted_samples_are_not_placed():
    st, tr = own_path()
    st.valid[400:500] = False
    ta = st.t[100]
    al = O.Aligner(st).add(O.Anchor(ta, *truth_at(tr, ta), sd_cm=0.2, sd_deg=0.5, kind="home"))
    assert not al.pose_at([st.t[450]])["ok"][0]


def test_anchor_before_the_log_is_held_at_its_start():
    st, tr = own_path()
    a = O.Anchor(st.t[0] - 20.0, *truth_at(tr, st.t[0]), sd_cm=0.2, sd_deg=0.5, kind="home")
    p = O.Aligner(st).add(a).pose_at([st.t[0]])
    assert p["ok"][0]
    far = O.Anchor(st.t[0] - 500.0, *truth_at(tr, st.t[0]), sd_cm=0.2, sd_deg=0.5, kind="home")
    assert not O.Aligner(st).add(far).pose_at([st.t[0]])["ok"][0]


def labels_every(st, tr, every_s, noise=0.3, seed=1):
    rng = np.random.default_rng(seed)
    out = []
    for t in np.arange(st.t[0] + 1, st.t[-1], every_s):
        x, y, _ = truth_at(tr, t)
        out.append(O.Anchor(t, x + rng.normal(0, noise), y + rng.normal(0, noise), None, sd_cm=0.5))
    return out


def test_fit_recovers_gyro_scale_drift_and_wheel_scale():
    st, tr = own_path(seconds=120, g_true=0.925, drift=0.3, wheel=0.98)
    al = O.Aligner(st).add(*labels_every(st, tr, 8.0))
    c = al.fit_calibration()
    assert c["fitted"]
    assert c["g"] == pytest.approx(1 / 0.925, rel=0.02)
    assert c["d"] == pytest.approx(0.3 / 0.925, abs=0.08)
    assert c["s"] == pytest.approx(0.98, abs=0.02)
    assert "calibration fitted" in al.notes[-1]


def test_fit_finds_a_flipped_heading_sign_when_it_was_not_checked():
    st, tr = own_path(seconds=90, sign=-1.0)
    st.sign_known = False
    st.priors["g"] = 1.0                            # the plug-in doesn't know the sign
    al = O.Aligner(st).add(*labels_every(st, tr, 8.0))
    c = al.fit_calibration()
    assert c["fitted"] and c["g"] < 0
    ts = st.t[::200]
    p = al.pose_at(ts)
    x, y, _ = truth_at(tr, ts)
    assert np.nanmedian(np.hypot(p["x"] - x, p["y"] - y)) < 1.5


def test_fit_needs_enough_anchors_and_rejects_nonsense():
    st, tr = own_path()
    al = O.Aligner(st).add(*labels_every(st, tr, 25.0)[:2])
    assert not al.fit_calibration()["fitted"]
    assert "needs" in al.notes[-1]
    rng = np.random.default_rng(3)
    junk = [O.Anchor(t, rng.uniform(0, 120), rng.uniform(0, 120), None, sd_cm=0.1) for t in st.t[::300]]
    al2 = O.Aligner(st).add(*junk)
    c = al2.fit_calibration()
    if c["fitted"]:                                 # a fit of noise must at least stay plausible
        assert 0.7 <= abs(c["g"]) <= 1.4 and 0.8 <= c["s"] <= 1.25


def test_tune_allowances_makes_the_confidence_honest():
    st, tr = own_path(seconds=120)
    al = O.Aligner(st).add(*labels_every(st, tr, 6.0))
    al.fit_calibration()
    rep = O.tune_allowances(al)
    assert rep is not None and rep["n"] >= 5
    assert abs(math.log(rep["median_ratio_after"] / O.RAYLEIGH_MEDIAN)) <= abs(
        math.log(max(rep["median_ratio_before"], 1e-9) / O.RAYLEIGH_MEDIAN)) + 1e-9
    assert O.tune_allowances(O.Aligner(st).add(*labels_every(st, tr, 40.0))) is None   # too few labels


# =============================================================================== outside.py: outputs
def test_draft_labels_are_pending_log_labels_in_the_picture(cam):
    st, tr = own_path(start=(60.0, 60.0, 0.0), g_true=1.0, drift=0.0, wheel=1.0)   # format, not accuracy
    ta = st.t[0]
    al = O.Aligner(st).add(O.Anchor(ta, *truth_at(tr, ta), sd_cm=0.2, sd_deg=0.5, kind="home"))
    times = {k: float(t) for k, t in enumerate(np.arange(st.t[0] - 10, st.t[0] + 6, 0.5))}
    d = O.draft_labels({"R": al}, times, cam, obj_names={"R": 7}, class_heights={"alvik": H_ALVIK}, plugin="test")
    assert d and min(d) >= 20 - 6                   # nothing long before the log starts (3 s hold)
    f = max(d)
    o = d[f][0]
    for k in ("obj", "cls", "x", "y", "heading_deg", "heading_known", "source", "status", "score", "name", "x_cm",
              "y_cm", "sd_cm", "heading_sd_deg", "outside"):
        assert k in o
    assert o["obj"] == 7 and o["source"] == "log" and o["status"] == "pending" and o["cls"] == "alvik"
    x, y, h = truth_at(tr, times[f])
    u, v = cam.project((float(x), float(y), H_ALVIK))
    assert math.hypot(o["x"] - u, o["y"] - v) < 2.0
    assert o["heading_known"]
    assert herr(o["heading_deg"], O.image_heading(cam, float(x), float(y), H_ALVIK, float(h))) < 3.0


def test_draft_labels_leave_out_low_confidence(cam):
    st, tr = own_path(start=(60.0, 60.0, 0.0), seconds=60, drift=0.6)
    ta = st.t[0]
    al = O.Aligner(st, O.Settings(min_score=0.5)).add(O.Anchor(ta, *truth_at(tr, ta), sd_cm=0.2, sd_deg=0.5))
    times = {k: float(t) for k, t in enumerate(st.t[::25])}
    d = O.draft_labels({"R": al}, times, cam, settings=al.settings)
    assert d and max(d) < max(times)                 # the far end has drifted too far to offer
    assert all(o["score"] >= 0.5 for v in d.values() for o in v)


def test_score_branches_votes_for_the_branch_that_follows_the_log(cam):
    st, tr = own_path(start=(60.0, 60.0, 0.0), g_true=1.0, drift=0.0, wheel=1.0)
    ta = st.t[100]
    al = O.Aligner(st).add(O.Anchor(ta, *truth_at(tr, ta), sd_cm=0.2, sd_deg=0.5))
    ts = st.t[150:400:25]
    x, y, h = truth_at(tr, ts)
    right = [(t, *cam.project((float(a), float(b), H_ALVIK))) for t, a, b in zip(ts, x, y)]
    wrong = [(t, *cam.project((float(a) + 6.0, float(b) - 3.0, H_ALVIK))) for t, a, b in zip(ts, x, y)]
    res = O.score_branches(al, {"a": right, "b": wrong}, cam, H_ALVIK)
    assert res["a"]["p"] > 0.95 and res["a"]["informative"]
    assert res["a"]["mean_err_cm"] < 1.0 < res["b"]["mean_err_cm"]
    early = [(st.t[0] - 100, 300, 200)]                        # before the log: no vote
    res2 = O.score_branches(al, {"a": early, "b": early}, cam, H_ALVIK)
    assert res2["a"]["p"] == pytest.approx(0.5) and not res2["a"]["informative"]


def test_score_candidates(cam):
    st, tr = own_path(start=(60.0, 60.0, 0.0), g_true=1.0, drift=0.0, wheel=1.0)
    ta = st.t[100]
    al = O.Aligner(st).add(O.Anchor(ta, *truth_at(tr, ta), sd_cm=0.2, sd_deg=0.5))
    t = st.t[150]
    x, y, _ = truth_at(tr, t)
    u, v = cam.project((float(x), float(y), H_ALVIK))
    r = O.score_candidates({"R": al}, t, [(u, v), (u + 40, v)], cam, H_ALVIK)["R"]
    assert r[0] > 0.6 > 0.01 > r[1]


def test_heading_from_the_first_forward_stretch_seen_on_video():
    st, tr = own_path(drift=0.0)
    track = np.column_stack([tr["t"][::5], tr["x"][::5], tr["y"][::5]])
    a = O.heading_anchor_from_motion(st, track, min_cm=8.0)
    assert len(a) == 1 and a[0].kind == "motion" and a[0].heading is not None
    assert herr(a[0].heading, truth_at(tr, a[0].t)[2]) < 3.0
    # the stretch must be driven forwards: a reversing robot's motion is not its facing
    every = O.heading_anchor_from_motion(st, track, min_cm=8.0, first_only=False)
    for b in every:
        k = np.searchsorted(st.t, b.t)
        assert st.dist[min(k + 5, len(st) - 1)] >= st.dist[max(k - 5, 0)]
        assert herr(b.heading, truth_at(tr, b.t)[2]) < 4.0


def test_time_offset_from_speed():
    st, tr = own_path(seconds=60)
    track = np.column_stack([tr["t"][::2] + 0.35, tr["x"][::2], tr["y"][::2]])   # camera clock 0.35 s ahead
    off, corr = O.estimate_time_offset(st, track)
    assert off == pytest.approx(0.35, abs=0.04) and corr > 0.8


def test_match_names_pairs_logs_with_tracks():
    s1, t1 = own_path(seed=1)
    s2, t2 = own_path(seed=2)
    s2.name = "Q"
    tracks = {10: np.column_stack([t2["t"], t2["x"], t2["y"]]), 11: np.column_stack([t1["t"], t1["x"], t1["y"]])}
    m, detail = O.match_names({"R": s1, "Q": s2}, tracks)
    assert m == {10: "Q", 11: "R"}


def test_linear_clock_chains():
    a = O.LinearClock(100.0, 1.001, "robot")
    b = O.LinearClock(0.35, 1.0, "pi")
    c = a.then(b)
    assert float(c(10.0)) == pytest.approx(float(b(a(10.0))))
    assert "robot" in c.text and "pi" in c.text


def test_homography_map_matches_the_camera_model(tmp_path, cam):
    import cv2
    g = synth.board_points(20.0)
    u, v = cam.project(np.column_stack([g, np.zeros(len(g))]))
    H, _ = cv2.findHomography(g, np.column_stack([u, v]))
    C = cam.center
    hm = O.HomographyMap(H, (640, 480), float(C[2]), (float(C[0]), float(C[1])))
    for (x, y, z) in ((20.0, 30.0, 0.0), (90.0, 100.0, 4.0)):
        assert np.allclose(hm.project((x, y, z)), cam.project((x, y, z)), atol=0.05)
        uu, vv = cam.project((x, y, z))
        assert np.allclose(hm.to_floor(uu, vv, z), (x, y), atol=0.02)
    p = tmp_path / "floor_calibration.json"
    p.write_text(json.dumps(dict(H_floor_to_px=H.tolist(), image_size=[640, 480])))
    m = O.load_floor_map(str(p))                     # straight down, no height: homography only
    assert np.allclose(m.project((50.0, 50.0, 0.0)), cam.project((50.0, 50.0, 0.0)), atol=0.1)
    cam.save(str(tmp_path / "camera.json"))
    assert isinstance(O.load_floor_map(str(tmp_path / "camera.json")), CameraModel)


def test_generic_csv_plugin(tmp_path):
    p = tmp_path / "positions.csv"
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "name", "x", "y", "heading", "frame"])
        for k in range(50):
            w.writerow([100 + 0.1 * k, "arm", 10 + 0.5 * k, 20, 0, "floor"])
            w.writerow([100 + 0.1 * k, "cart", 0.4 * k, 0.0, 0, "own"])
        w.writerow(["bad", "arm", "x", "y", "", ""])
    assert "csv" in [p_[0] for p_ in O.plugins()] and "alvik" in [p_[0] for p_ in O.plugins()]
    data = O.read_outside(str(tmp_path), offset_s=0.5)
    assert data.plugin == "csv" and set(data.names()) == {"arm", "cart"}
    assert data.notes and "1 rows" in data.notes[0]
    arm = O.Aligner(data.streams["arm"]).pose_at([100.5 + 1.0])
    assert arm["ok"][0] and arm["x"][0] == pytest.approx(15.0, abs=0.01)
    cart = data.streams["cart"]
    assert cart.frame == "own" and cart.dist[-1] == pytest.approx(0.4 * 49, abs=1e-6)
    with pytest.raises(ValueError):
        O.read_outside(str(tmp_path / "nothing"))


# =============================================================================== robotlogs.py
def test_reads_every_robot_with_clock_and_signs(sess):
    data = RL.read_alvik_logs(sess["logs"], recording=sess["recording"])
    assert set(data.names()) == {"Left", "Center", "Right"}
    assert "video_clock.json" in data.clock
    assert not [n for n in data.notes if "damaged" in n]
    for name, st in data.streams.items():
        assert st.sign_known and st.cls == "alvik"
        assert st.priors["g"] == pytest.approx(1.08)        # gyro_scale 1 in param.csv = out of the box: Alvik prior
        tr = sess["truth"][name]
        # times on the Pi clock: the stream's speed lines up with the truth's with no leftover offset
        track = np.column_stack([tr["t"][::3], tr["x"][::3], tr["y"][::3]])
        off, corr = O.estimate_time_offset(st, track, search_s=1.0, step_s=0.01)
        assert abs(off) <= 0.05 and corr > 0.8
    assert data.streams["Center"].meta["segments"] == 2       # Center was carried once
    assert data.streams["Left"].meta["segments"] == 1


def test_events_from_the_logs(sess):
    data = RL.read_alvik_logs(sess["logs"], recording=sess["recording"])
    kinds = {e["kind"] for e in data.events}
    assert {"docked", "run_start", "run_end", "flash", "lifted", "put_down"} <= kinds
    for e in [e for e in data.events if e["kind"] == "docked"]:
        docked = sess["truth"][e["name"]]["docked"]
        assert any(a - 2.0 <= e["t"] <= b + 1.0 for a, b in docked)


def copy_logs(sess, tmp_path, drop=(), edit=None):
    d = tmp_path / "logs"
    shutil.copytree(sess["logs"], d)
    for f in drop:
        os.remove(d / f)
    if edit:
        edit(d)
    return str(d)


def test_without_video_clock_the_offset_is_noted_and_can_be_found(sess, tmp_path):
    data = RL.read_alvik_logs(sess["logs"])                  # no recording given
    assert any("assumed equal" in n for n in data.notes)
    st = data.streams["Left"]
    tr = sess["truth"]["Left"]
    off, _ = O.estimate_time_offset(st, np.column_stack([tr["t"][::3], tr["x"][::3], tr["y"][::3]]))
    assert off == pytest.approx(SL.PI_OFFSET_S, abs=0.04)
    given = RL.read_alvik_logs(sess["logs"], laptop_to_wall=SL.PI_OFFSET_S)
    assert given.streams["Left"].t[0] == pytest.approx(
        RL.read_alvik_logs(sess["logs"], recording=sess["recording"]).streams["Left"].t[0], abs=1e-6)


def test_without_clock_sync_the_arrival_times_set_the_clock(sess, tmp_path):
    ref = RL.read_alvik_logs(sess["logs"], recording=sess["recording"])
    d = copy_logs(sess, tmp_path, drop=("clock_sync.csv",))
    data = RL.read_alvik_logs(d, recording=sess["recording"])
    assert any("arrival times" in n for n in data.notes)
    for n in ref.names():
        assert np.median(data.streams[n].t - ref.streams[n].t) == pytest.approx(0.0, abs=0.03)


def test_without_runinfo_the_sign_is_fitted(sess, tmp_path, cam):
    d = copy_logs(sess, tmp_path, drop=("runinfo.csv",))
    data = RL.read_alvik_logs(d, recording=sess["recording"])
    assert not data.streams["Center"].sign_known          # Center's gyro sign is flipped in the logs
    assert any("runinfo" in n for n in data.notes)
    res = SL.measure(dict(sess, logs=d), "labels")
    rows = [r for r in res["rows"] if r[0] == "Center"]
    assert res["calib"]["Center"]["fitted"] and res["calib"]["Center"]["g"] < 0
    assert np.median([r[3] for r in rows]) < 2.0


def test_damaged_and_missing_files(sess, tmp_path):
    def damage(d):
        with open(d / "raw.csv", "a", newline="") as f:
            f.write("Left,123.4,1a2b-9,55,0,1\n")                       # a line cut short by a crash
            f.write("Left,\0\0\0garbage\n")
        (d / "dock.csv").write_text("")                               # empty file
    d = copy_logs(sess, tmp_path, drop=("runend.csv", "sync.csv", "param.csv"), edit=damage)
    data = RL.read_alvik_logs(d, recording=sess["recording"])
    assert set(data.names()) == {"Left", "Center", "Right"}
    assert any("damaged" in n for n in data.notes)
    empty = tmp_path / "empty"
    empty.mkdir()
    e = RL.read_alvik_logs(str(empty))
    assert not e.streams and "no raw.csv" in e.notes[0]
    assert not RL.AlvikLogs().detect(str(empty)) and RL.AlvikLogs().detect(sess["logs"])
    assert O.read_outside(sess["logs"], recording=sess["recording"]).plugin == "alvik"


def test_a_robot_renamed_during_the_session_is_one_robot(sess, tmp_path):
    def rename(d):
        rows = list(csv.reader(open(d / "raw.csv")))
        n = 0
        for r in rows[1:]:
            if r[0] == "Left" and n < 300:
                r[0] = "AL-1A2B"
                n += 1
        with open(d / "raw.csv", "w", newline="") as f:
            csv.writer(f).writerows(rows)
    d = copy_logs(sess, tmp_path, edit=rename)
    data = RL.read_alvik_logs(d, recording=sess["recording"])
    assert set(data.names()) == {"Left", "Center", "Right"}
    assert any("AL-1A2B -> Left" in n for n in data.notes)


def test_rows_lost_at_a_run_start_keep_the_turn(tmp_path):
    """A turn whose rows were lost right at a run start is kept; a reset to 0 is recognised."""
    head = ["robot", "t_recv"] + SL.RAW_COLS
    rows = []

    def row(t, run, x, y, th, yaw):
        v = {c: 0 for c in SL.RAW_COLS}
        v.update(run=run, ms=int(t * 1000), x_cm=x, y_cm=y, th_deg=th, yaw_deg=yaw)
        rows.append(["Left", 1000.0 + t] + [v[c] for c in SL.RAW_COLS])
    for k in range(50):                       # run 1: 10 cm straight at heading 0
        row(0.02 * k, "ab12-1", 0.2 * k, 0.0, 0.0, 50.0)
    # run 2: the library pose reset to 0; its first rows (a 40 deg turn) were lost
    for k in range(50):
        row(3.0 + 0.02 * k, "ab12-2", 0.2 * k * math.cos(math.radians(40)), 0.2 * k * math.sin(math.radians(40)),
            40.0, 90.0)
    with open(tmp_path / "raw.csv", "w", newline="") as f:
        csv.writer(f).writerows([head] + rows)
    with open(tmp_path / "runinfo.csv", "w", newline="") as f:
        csv.writer(f).writerows([["robot", "t_recv", "run", "experiment", "fw_version", "motor_fw", "ms_start", "battery",
                                  "charging", "wifi_dbm", "route", "yaw_sign", "odo_sign"],
                                 ["Left", 1000, "ab12-1", "demo", "", "", 0, 0, 0, 0, "", 1, 1],
                                 ["Left", 1003, "ab12-2", "demo", "", "", 3000, 0, 0, 0, "", 1, 1]])
    st = RL.read_alvik_logs(str(tmp_path), laptop_to_wall=0.0).streams["Left"]
    assert st.heading[-1] - st.heading[0] == pytest.approx(40.0, abs=0.5)
    assert st.dist[-1] == pytest.approx(9.8 + 9.8, abs=0.3)          # no jump back to 0 counted as driving


# =============================================================================== homes.py
def test_make_home_validates_and_stores_in_the_project(tmp_path):
    h = HM.make_home("tray", "default", 100, 120, None, 10)
    assert h["kind"] == "park" and h["facing_deg"] is None and h["still_s"] == 2.0
    for bad in (dict(name=""), dict(radius=0), dict(kind="other"), dict(x=float("nan"))):
        kw = dict(name="a", camera="default", x=1, y=1, radius=5)
        kw.update(bad)
        with pytest.raises(ValueError):
            HM.make_home(**kw)
    from recognition.project import Project
    pr = Project(str(tmp_path / "proj"), create_name="p")
    HM.set_homes(pr.data, "default", [h, dict(h, name="dock", facing_deg=90.0)])
    with pytest.raises(ValueError):
        HM.set_homes(pr.data, "default", [h, h])
    pr.save()
    pr.set_camera_settings("default", work_area=[[0, 0], [10, 0], [10, 10]])   # another lane's save keeps them
    again = Project(str(tmp_path / "proj"))
    assert [x["name"] for x in HM.homes_of(again.data)] == ["tray", "dock"]
    assert HM.homes_of(again.data, "other") == []


def test_find_visits_needs_still_and_inside():
    home = HM.make_home("h", "default", 100, 100, 90.0, 10, still_s=2.0)
    t = np.arange(0, 20, 0.1)
    stay = [(k, float(tt), 100.5, 99.8) for k, tt in enumerate(t[:40])]                  # 4 s still: a visit
    passing = [(100 + k, float(tt), 60 + 5 * k, 100.0) for k, tt in enumerate(t[40:80])]  # drives through
    short = [(200 + k, float(tt), 101, 101) for k, tt in enumerate(t[80:95])]             # 1.5 s: too short
    v = HM.find_visits([home], {1: stay + passing + short})
    assert len(v) == 1 and v[0]["obj"] == 1 and v[0]["still_s"] >= 3.8 and v[0]["off_px"] < 1
    c = HM.confirmations(v, [home])
    assert c[0]["source"] == "home" and c[0]["x"] == 100 and c[0]["heading_deg"] == 90.0


def test_station_visits_are_short(cam):
    homes = HM.alvik_dock_homes_floor("d", "default", cam, 61.0, 4.0, -90.0, owner="Center")
    st = [h for h in homes if h["kind"] == "station"]
    assert len(st) == 3 and all(h["still_s"] < 1.0 for h in st)
    s = st[0]
    rows = [(k, 0.1 * k, s["x"], s["y"]) for k in range(6)]
    assert HM.find_visits(homes, {5: rows})


def test_alvik_dock_preset_geometry(cam):
    homes = HM.alvik_dock_homes_floor("dock Left", "default", cam, 25.0, 4.0, -90.0, owner="Left")
    park = homes[0]
    assert park["kind"] == "park" and park["owner"] == "Left"
    assert park["x_cm"] == pytest.approx(25.0, abs=1e-6) and park["y_cm"] == pytest.approx(4.0 + 6.1, abs=0.01)
    assert park["heading"] == pytest.approx(270.0)
    x, y, h = HM.home_floor_pose(park, cam, H_ALVIK)
    assert (x, y, h) == (park["x_cm"], park["y_cm"], park["heading"])
    names = {h_["name"]: h_ for h_ in homes}
    back = names["dock Left back station"]
    # back station: the robot drives towards the dock's back face (facing the opposite way to a docked robot)
    assert back["heading"] == pytest.approx(90.0, abs=1e-6)
    assert back["y_cm"] < 4.0                       # behind the wall, outside the dock
    # two clicks (wall top edge middle, mouth middle) give the same park pose
    wall = cam.project((25.0, 4.0, 2.2))
    mouth = cam.project((25.0, 4.0 + 6.2, 2.2))
    from_clicks = HM.alvik_dock_homes("dock Left", "default", cam, wall, mouth, owner="Left")[0]
    assert from_clicks["x_cm"] == pytest.approx(25.0, abs=0.05) and from_clicks["y_cm"] == pytest.approx(10.1, abs=0.05)
    assert herr(from_clicks["heading"], 270.0) < 0.5
    mp = dict(home_docks=[dict(name="h1", x=25.0, y=4.0, heading=-90.0)])
    assert HM.alvik_docks_from_map(mp, "default", cam, owners={"h1": "Left"}, stations=False)[0]["owner"] == "Left"


def test_visits_become_anchors_by_owner_name_or_stillness(sess):
    cam = sess["camera"]
    mp = json.load(open(os.path.join(sess["out"], "map.json")))
    ft = sess["frame_times"]
    tracks = {tr["obj"]: [(f, ft[f], tr["frame_u"][f], tr["frame_v"][f]) for f in sorted(ft)[::2]]
              for tr in sess["truth"].values()}
    owners = {"dock " + s.name: s.name for s in sess["robots"]}
    homes = HM.alvik_docks_from_map(mp, "default", cam, owners=owners, stations=False)
    visits = HM.find_visits(homes, tracks)
    assert len(visits) == 9                         # each robot: start, mid-session visit, end
    by_owner = HM.anchors_from_visits(visits, homes, cam, height_cm=H_ALVIK)
    assert {n: len(a) for n, a in by_owner.items()} == {"Left": 6, "Center": 6, "Right": 6}
    anon = HM.alvik_docks_from_map(mp, "default", cam, stations=False)          # nobody owns a dock
    data = RL.read_alvik_logs(sess["logs"], recording=sess["recording"])
    mid = [v for v in HM.find_visits(anon, tracks) if 30 < v["t_start"] - SL.T_START < 140]
    by_still = HM.anchors_from_visits(mid, anon, cam, height_cm=H_ALVIK, outside=data)
    for v in mid:
        name = [s.name for s in sess["robots"] if s.obj == v["obj"]][0]
        assert name in by_still
    ev = HM.events_to_anchors(data.events, homes, cam, H_ALVIK)
    assert set(ev) == {"Left", "Center", "Right"}
    st = data.streams["Left"]
    a = HM.start_anchor(st, homes[0], cam, H_ALVIK)
    assert a.kind == "start" and a.t == st.t[0]


# =============================================================================== measured on the synthetic session
def test_measured_errors_on_the_synthetic_session(sess):
    a = SL.table(SL.measure(sess, "start"))
    b_res = SL.measure(sess, "start+visits")
    b = SL.table(b_res)
    d = SL.table(SL.measure(sess, "start+visits+labels"))
    g = SL.measure(sess, "labels", prior_g=1.0, label_every_s=12.0)
    allrow = {k: [r for r in t if r["bin"] == "all"][0] for k, t in (("a", a), ("b", b), ("d", d))}
    # re-alignment at the dock visits beats starting in the dock alone
    assert allrow["b"]["med_cm"] < allrow["a"]["med_cm"]
    assert allrow["b"]["p90_cm"] < 3.0 and allrow["b"]["med_deg"] < 2.0
    assert allrow["d"]["p90_cm"] < 2.5 and allrow["d"]["med_deg"] < 2.0
    # the confidence is honest enough: most errors within twice the stated sd
    assert allrow["d"]["within_2sd"] > 0.65          # 0.86 for a perfectly judged sd
    # no Alvik knowledge: the fit still finds the gyro under-reporting and the drift
    for sp in sess["robots"]:
        c = g["calib"][sp.name]
        assert c["fitted"]
        assert abs(c["g"]) == pytest.approx(1 / sp.gyro, rel=0.02)
        assert c["d"] * np.sign(c["g"]) == pytest.approx(sp.drift / sp.gyro, abs=0.1)
    live = SL.table(SL.measure(sess, "start+visits", causal=True))
    assert [r for r in live if r["bin"] == "all"][0]["p90_cm"] < 5.0


def test_draft_labels_from_the_logs_land_on_the_robots(sess):
    cam = sess["camera"]
    data = RL.read_alvik_logs(sess["logs"], recording=sess["recording"])
    mp = json.load(open(os.path.join(sess["out"], "map.json")))
    homes = HM.alvik_docks_from_map(mp, "default", cam, owners={"dock " + s.name: s.name for s in sess["robots"]},
                                    stations=False)
    ft = sess["frame_times"]
    tracks = {tr["obj"]: [(f, ft[f], tr["frame_u"][f], tr["frame_v"][f]) for f in sorted(ft)[::2]]
              for tr in sess["truth"].values()}
    anchors = HM.anchors_from_visits(HM.find_visits(homes, tracks), homes, cam, height_cm=H_ALVIK)
    als = O.align(data, anchors)
    frames = sorted(ft)[::10]
    names = {s.name: s.obj for s in sess["robots"]}
    d = O.draft_labels(als, ft, cam, obj_names=names, class_heights={"alvik": H_ALVIK}, frames=frames, plugin="alvik")
    errs, n_known, n = [], 0, 0
    for f, objs in d.items():
        for o in objs:
            tr = sess["truth"][o["name"]]
            errs.append(math.hypot(o["x"] - tr["frame_u"][f], o["y"] - tr["frame_v"][f]))
            n += 1
            if o["heading_known"]:
                n_known += 1
                assert herr(o["heading_deg"], tr["frame_himg"][f]) < 10.0
    assert n > 0.6 * 3 * len(frames) * 0.9
    assert np.median(errs) < 4.0 and np.percentile(errs, 90) < 10.0     # px (robots are ~30 px)
    assert n_known > 0.5 * n
