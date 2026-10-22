"""tools/experiment/e02_analysis.py on made-up numbers (do the statistics recover known distributions?) and on a tiny
synthetic overhead video with known truth (does the frame-by-frame measurement and contact detection find it?)."""
import csv
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
TOOLS = os.path.dirname(EXP)
for p in (EXP, os.path.join(TOOLS, "recognition"), TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import e02_analysis as ea  # noqa: E402
import e02_synth as es  # noqa: E402


# ------------------------------------------------------------------ statistics on made-up numbers
def test_describe_matches_the_textbook_numbers():
    d = ea.describe([1.0, 2.0, 3.0, 4.0, 10.0])
    assert d["n"] == 5 and d["mean"] == pytest.approx(4.0) and d["median"] == 3.0
    assert d["sd"] == pytest.approx(math.sqrt(12.5))
    half = 2.7764451 * d["sd"] / math.sqrt(5)             # t(0.975, 4 df) x SD / sqrt(n)
    assert d["ci_lo"] == pytest.approx(4.0 - half, abs=1e-4) and d["ci_hi"] == pytest.approx(4.0 + half, abs=1e-4)
    assert d["sd_lo"] < d["sd"] < d["sd_hi"]
    one = ea.describe([5.0])
    assert one["n"] == 1 and math.isnan(one["sd"]) and math.isnan(one["ci_lo"])
    assert ea.describe([])["n"] == 0
    assert ea.describe([1.0, float("nan"), 3.0])["n"] == 2


def test_the_confidence_interval_of_the_mean_covers_the_truth_about_95_percent_of_the_time():
    rng = np.random.default_rng(11)
    hit = sum(1 for _ in range(600)
              if (lambda d: d["ci_lo"] <= 3.0 <= d["ci_hi"])(ea.describe(rng.normal(3.0, 1.5, 12), boot=0)))
    assert 0.92 <= hit / 600 <= 0.98


def test_the_bootstrap_interval_of_the_sd_contains_the_true_sd_for_a_good_sample():
    rng = np.random.default_rng(5)
    d = ea.describe(rng.normal(0, 2.0, 80))
    assert d["sd_lo"] < 2.0 < d["sd_hi"] and d["sd_hi"] - d["sd_lo"] < 1.2
    x = list(rng.normal(0, 2.0, 30))
    assert ea.describe(x) == ea.describe(x)               # same data, same interval


def _spec(**kw):
    base = {"U": dict(end_cm=(3.0, 1.0), end_deg=(0.0, 4.0)), "C": dict(end_cm=(2.0, 1.0), end_deg=(0.0, 4.0)),
            "K": dict(end_cm=(3.0, 1.0), end_deg=(0.0, 12.0)), "KC": dict(end_cm=(3.0, 1.0), end_deg=(0.0, 4.0))}
    base.update(kw)
    return base


def test_welch_and_mann_whitney_find_a_real_shift_and_not_a_fake_one():
    rng = np.random.default_rng(2)
    rows = es.fake_runs(rng, 60, _spec())
    ea.flag_runs(rows)
    summ, comps = ea.summarise(rows)
    get = lambda a, b, m="end_err_cm", s="all runs": next(    # noqa: E731
        c for c in comps if c["scope"] == "all" and c["metric"] == m and c["set"] == s and (c["cond_a"], c["cond_b"]) == (a, b))
    uc = get("U", "C")                                   # U is 1 cm (about 1 SD) higher than C
    assert uc["welch_p"] < 1e-4 and uc["mwu_p"] < 1e-4 and uc["welch_p_holm"] < 1e-3
    assert 0.6 < uc["cohen_d"] < 1.6 and uc["mean_diff"] > 0.5 and uc["diff_lo"] < uc["mean_diff"] < uc["diff_hi"]
    assert uc["rank_biserial"] > 0.3                      # positive: U runs sit higher than C runs
    uk = get("U", "K")                                   # same distribution: no real difference
    assert uk["welch_p"] > 0.01 and abs(uk["cohen_d"]) < 0.6 and abs(uk["rank_biserial"]) < 0.3
    assert uc["welch_df"] > 100


def test_brown_forsythe_flags_unequal_spread_only():
    rng = np.random.default_rng(8)
    rows = es.fake_runs(rng, 60, _spec())
    ea.flag_runs(rows)
    _, comps = ea.summarise(rows)
    pick = lambda a, b: next(c for c in comps if c["scope"] == "all" and c["metric"] == "end_head_abs_deg"   # noqa: E731
                             and c["set"] == "all runs" and (c["cond_a"], c["cond_b"]) == (a, b))
    assert pick("U", "K")["bf_p"] < 0.01                  # heading SD 4 vs 12 deg
    assert pick("U", "KC")["bf_p"] > 0.01                 # same SD


def test_effect_sizes_are_exact_for_complete_separation_and_signed():
    r = ea.compare([10, 11, 12, 13], [1, 2, 3, 4])
    assert r["rank_biserial"] == pytest.approx(1.0) and r["mwu_U"] == 16.0 and r["cohen_d"] > 5
    r2 = ea.compare([1, 2, 3, 4], [10, 11, 12, 13])
    assert r2["rank_biserial"] == pytest.approx(-1.0) and r2["mean_diff"] < 0
    few = ea.compare([1.0], [2.0, 3.0])
    assert math.isnan(few["welch_p"]) and few["n_a"] == 1


def test_holm_is_monotone_and_never_below_the_raw_p():
    p = [0.01, 0.04, 0.03, float("nan")]
    h = ea.holm(p)
    assert h[0] == pytest.approx(0.03) and h[2] == pytest.approx(0.06) and h[1] == pytest.approx(0.06)
    assert math.isnan(h[3]) and all(hh >= pp for hh, pp in zip(h[:3], p[:3]))


def test_outliers_are_flagged_and_the_tables_show_with_and_without():
    rng = np.random.default_rng(4)
    rows = es.fake_runs(rng, 12, {"U": dict(end_cm=(3.0, 0.5), end_deg=(0, 3)), "C": dict(end_cm=(3.0, 0.5), end_deg=(0, 3))})
    rows[0]["end_err_cm"] = 40.0                          # a lost robot
    ea.flag_runs(rows)
    assert rows[0]["outlier_end_err_cm"] == 1 and sum(r["outlier_end_err_cm"] for r in rows) <= 3
    summ, _ = ea.summarise(rows)
    a = next(s for s in summ if s["scope"] == "all" and s["metric"] == "end_err_cm" and s["condition"] == "U" and s["set"] == "all runs")
    b = next(s for s in summ if s["scope"] == "all" and s["metric"] == "end_err_cm" and s["condition"] == "U" and s["set"] == "without outliers")
    assert a["mean"] > b["mean"] + 2 and b["n"] < a["n"] and b["mean"] < 4
    rec = es.fake_runs(rng, 6, {"U": dict(end_cm=(3.0, 0.5), end_deg=(0, 3))})
    rec[2]["recovery"] = "far_off"                       # driven back by the runner: a note, never an outlier by itself
    ea.flag_runs(rec)
    assert rec[2]["outlier_end_err_cm"] == 0


def test_outliers_are_decided_within_condition_and_route():
    """The long straight's errors are larger by design: pooled with the L and the square they would all look like
    outliers; within its own route none is."""
    rng = np.random.default_rng(5)
    rows = es.fake_runs(rng, 15, {"U": dict(end_cm=(3.0, 0.5), end_deg=(0, 3))})
    for r in rows:
        if r["route"] == "straight":
            r["end_err_cm"] += 15.0
    ea.flag_runs(rows)
    assert sum(r["outlier_end_err_cm"] for r in rows) <= 1


def test_the_permutation_test_respects_robots():
    """A robot effect, not a condition effect: Left is 5 cm worse whatever the condition, and runs more of A. Welch
    sees a difference between A and B; the permutation within robot x route does not."""
    rng = np.random.default_rng(8)
    a, sa, b, sb = [], [], [], []
    for rb, na, nb in (("Left", 8, 2), ("Right", 2, 8)):
        for _ in range(na):
            a.append(3.0 + (5.0 if rb == "Left" else 0.0) + rng.normal(0, 0.5))
            sa.append((rb, "straight"))
        for _ in range(nb):
            b.append(3.0 + (5.0 if rb == "Left" else 0.0) + rng.normal(0, 0.5))
            sb.append((rb, "straight"))
    assert ea.compare(a, b)["welch_p"] < 0.05
    d, p, k = ea.strat_perm(a, sa, b, sb)
    assert k == 2 and abs(d) < 1.0 and p > 0.05
    b2 = [v - 2.0 for v in b]                             # a real condition effect is found
    assert ea.strat_perm(a, sa, b2, sb)[1] < 0.01


def test_scopes_split_by_route_and_robot():
    rows = es.fake_runs(np.random.default_rng(1), 6, {"U": dict(end_cm=(3, 1), end_deg=(0, 3)), "C": dict(end_cm=(2, 1), end_deg=(0, 3))})
    names = [s for s, _ in ea.scopes(rows)]
    assert names[0] == "all" and "route=L" in names and "robot=Left" in names


# ------------------------------------------------------------------ small pieces
def test_pose_algebra_round_trips():
    p, rel = (10.0, -4.0, 70.0), (6.0, -2.0, 25.0)
    q = ea.compose(p, rel)
    r = ea.relative(q, p)
    assert r[0] == pytest.approx(6.0) and r[1] == pytest.approx(-2.0) and r[2] == pytest.approx(25.0)
    assert ea.wrap180(190) == pytest.approx(-170)
    assert abs(ea.circ_median([179, -179, 178])) > 177                   # around the wrap, not near 0


def test_marks_runs_and_stage_aliases_are_read():
    ev = [dict(t_wall="10.5", frame="3", text="E02 run 7 start"), dict(t_wall="12", frame="9", text="run 7 far"),
          dict(t_wall="14", frame="12", text="E02 run 7 end"), dict(t_wall="9", frame="1", text="Run 7: find"),
          dict(t_wall="1", frame="0", text="hello")]
    m = ea.parse_marks(ev)
    assert m == {"7": dict(t_start=10.5, t_far=12.0, t_end=14.0, t_find=9.0)}
    assert ea.canon_route("2") == "L" and ea.canon_route("Square") == "square" and ea.canon_route("1") == "straight"
    row = dict(Start_X_cm="1.5", start_y_cm="2", start_th_deg="90")
    assert ea.stage_pose(row, "start") == (1.5, 2.0, 90.0)
    assert ea.stage_pose(dict(x_far="1", y_far="2", theta_far="3"), "far") == (1.0, 2.0, 3.0)
    assert ea.stage_pose(dict(start_x_cm="1"), "start") is None


# ------------------------------------------------------------------ measuring the robot in a picture
@pytest.fixture(scope="module")
def cam():
    return es.make_camera()


def test_measure_pose_finds_position_and_heading_at_many_angles(cam):
    rng = np.random.default_rng(0)
    for th in (0, 2, 10, 30, 44, 60, 92, 135, 200, 271, -170):
        p = (70.3 + th * 0.01, 100.2, th)
        u, v = cam.project((p[0], p[1], ea.TOP_CM))
        m = ea.measure_pose(es.render(cam, p, rng), (u + 3, v - 2), th + 20, 19.0)
        assert m is not None
        fp = ea.floor_pose(cam, m)
        assert math.hypot(fp[0] - p[0], fp[1] - p[1]) < 0.2             # cm (the picture has 0.5 cm pixels)
        assert abs(float(ea.wrap180(fp[2] - th))) < 2.5                 # deg


def test_heading_takes_the_side_nearest_the_hint(cam):
    p = (80.0, 100.0, 30.0)
    img = es.render(cam, p, np.random.default_rng(0))
    u, v = cam.project((p[0], p[1], ea.TOP_CM))
    ang = [ea.measure_pose(img, (u, v), h, 19.0)["ang"] for h in (30, 120, 210, -60)]
    assert all(abs(float(ea.wrap180(a - h))) < 3 for a, h in zip(ang, (30, 120, 210, -60)))


def test_nothing_found_on_an_empty_floor_and_a_hint_far_from_the_robot_picks_the_nearest_blob(cam):
    rng = np.random.default_rng(0)
    img = es._floor().astype(np.uint8)
    assert ea.measure_pose(img, (160, 120), 0, 19.0) is None
    p = (60.0, 110.0, 0.0)
    img = es.render(cam, p, rng)
    u, v = cam.project((p[0], p[1], ea.TOP_CM))
    m = ea.measure_pose(img, (u + 12, v + 8), 0, 19.0)
    assert m is not None and math.hypot(m["u"] - u, m["v"] - v) < 1.0


def test_still_segments_and_contact_frame_on_a_made_up_track():
    rng = np.random.default_rng(3)
    t = np.arange(0, 6.0, 0.04)
    x = np.where(t < 2.0, 12 * t, 24.0)                                  # drives 12 cm/s, stops dead at t = 2.0
    x = np.where(t > 3.0, 24.0 - 8 * (t - 3.0), x)                       # backs off
    x = np.clip(x, 20.0, None)
    xy = np.column_stack([x, np.zeros_like(x)]) + rng.normal(0, 0.08, (len(t), 2))
    head = np.zeros(len(t))
    ev = ea.contact_frames(t, xy, head)
    assert len(ev) == 1
    assert abs(t[ev[0][0]] - 2.0) <= 0.1 and ev[0][1] == pytest.approx(24.0, abs=0.3)
    segs = ea.still_segments(t, xy, 0.6, 0.5)
    assert any(t[a] <= 2.2 and t[b] >= 2.8 for a, b in segs)             # the hold at the board


def test_a_turn_on_the_spot_is_not_a_contact():
    t = np.arange(0, 5.0, 0.04)
    head = np.clip(t - 1.0, 0, 1.0) * 90.0
    xy = np.column_stack([np.full_like(t, 90.0), np.full_like(t, 71.0)])
    assert ea.contact_frames(t, xy, head) == []


# ------------------------------------------------------------------ the whole thing on a synthetic video
SPECS = [
    dict(run_id="1", robot="Left", condition="U", route="straight", end_err=(1.2, -0.6, 3.0), far_err=(0.8, 0.0, -2.0),
         bel_bias=(0.6, 0.0, 1.0)),
    dict(run_id="2", robot="Right", condition="C", route="L", end_err=(0.3, 0.4, -2.5), far_err=(-0.5, 0.5, 4.0)),
    dict(run_id="3", robot="Left", condition="K", route="square", end_err=(-0.9, 0.9, 1.0), far_err=(0.0, 1.0, 0.0), lost=1,
         routes={"square": (20.0, -20.0, -90.0)}),                           # turns right: the side is read from the camera
    dict(run_id="4", robot="Right", condition="KC", route="straight", end_err=(0.0, -1.5, 5.0), far_err=(0.5, 0.5, 0.0)),
]


@pytest.fixture(scope="module")
def session(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("e02"))
    sess = es.make_session(d, SPECS, seed=7)
    out = os.path.join(d, "out")
    res = ea.analyse(sess["recording"], sess["logs"], sess["runs"], sess["calib"], out, routes=es.ROUTES_SMALL, quiet=True)
    return sess, res, out


def test_per_run_poses_and_errors_match_the_truth(session):
    sess, res, _ = session
    rows = {r["run_id"]: r for r in res["rows"]}
    assert set(rows) == {"1", "2", "3", "4"}
    for sp in SPECS:
        r, tr = rows[sp["run_id"]], sess["truth"][sp["run_id"]]
        assert not r["flags"].replace("needed recovery: lost", "").strip(), r["flags"]
        for nm in ("start", "far", "end"):
            tp = tr[nm]
            assert math.hypot(r[nm + "_x"] - tp[0], r[nm + "_y"] - tp[1]) < 0.25
            assert abs(float(ea.wrap180(r[nm + "_th"] - tp[2]))) < 2.5
        ex, ey, eth = sp["end_err"]
        assert r["end_err_cm"] == pytest.approx(math.hypot(ex, ey), abs=0.3)
        assert r["end_err_deg"] == pytest.approx(eth, abs=2.5)
        fx, fy, fth = sp["far_err"]
        assert r["far_err_cm"] == pytest.approx(math.hypot(fx, fy), abs=0.3)
        assert r["far_err_deg"] == pytest.approx(fth, abs=2.5)


def test_return_to_start_in_pixels_agrees_without_the_calibration(session):
    _, res, _ = session
    for r, sp in zip(res["rows"], SPECS):
        want_px = math.hypot(*sp["end_err"][:2]) * es.SCALE
        assert r["px_return_dist"] == pytest.approx(want_px, abs=0.6)
        assert r["px_return_deg"] == pytest.approx(sp["end_err"][2], abs=2.5)


def test_the_robots_belief_is_compared_as_displacement_from_its_start(session):
    _, res, _ = session
    r1 = next(r for r in res["rows"] if r["run_id"] == "1")
    assert r1["bel_end_err_cm"] == pytest.approx(0.6, abs=0.4)           # planted belief bias (0.6 cm ahead)
    assert r1["bel_end_err_deg"] == pytest.approx(1.0, abs=2.5)
    r2 = next(r for r in res["rows"] if r["run_id"] == "2")
    assert r2["bel_end_err_cm"] < 0.4                                    # an honest belief
    assert all(r["find_err_cm"] < 1.5 for r in res["rows"])             # find: believed start vs camera
    assert all(r["believed_in_floor_frame"] == 1 for r in res["rows"])


def test_contact_frames_and_bump_metrics(session):
    sess, res, _ = session
    assert len(res["bumps"]) == 8 and all("cam_x" in b for b in res["bumps"])
    for b in res["bumps"]:
        tr = sess["truth"][b["run_id"]]
        t_true = tr["t_c1"] if b["board"] == "x" else tr["t_c2"]
        assert abs(b["t_contact"] - t_true) <= 2.5 / es.FPS              # within two frames
        c_true = tr["c1"] if b["board"] == "x" else tr["c2"]
        assert math.hypot(b["cam_x"] - c_true[0], b["cam_y"] - c_true[1]) < 0.7
        assert b["pos_err_cm"] < 1.3                                     # robot's belief was drawn within 0.3 cm per axis
        assert b["delay_ms"] == 80 and b["jolt"] > 1.0 and b["overshoot_cm"] == 0.3
        assert 20 <= b["cam_delay_ms"] <= 140                            # the robot's record said 80 ms
        assert "along_err_cm" in b and "cross_err_cm" in b


def test_outputs_exist_and_the_report_follows_the_house_rules(session):
    _, res, out = session
    for f in ("runs.csv", "bumps.csv", "summary.csv", "comparisons.csv", "report.md", os.path.join("tracks", "1.csv"),
              os.path.join("tracks", "4.csv")):
        assert os.path.getsize(os.path.join(out, f)) > 0, f
    runs = list(csv.DictReader(open(os.path.join(out, "runs.csv"))))
    assert len(runs) == 4 and "end_err_cm" in runs[0] and "px_return_dist" in runs[0]
    track = list(csv.DictReader(open(os.path.join(out, "tracks", "1.csv"))))
    assert len(track) > 400 and {"frame", "t_wall", "x_cm", "th_deg", "u_px"} <= set(track[0])
    text = open(os.path.join(out, "report.md")).read()
    heads = [ln for ln in text.splitlines() if ln.startswith("## ")]
    assert heads[:3] == ["## Premise", "## Data", "## Conclusion"] and "## What's next" in heads
    assert "New tests that may now be needed" in text and "Possible inaccuracies" in text
    assert text.index("New tests that may now be needed") < text.index("Possible inaccuracies") and heads[-1] == "## Two sweeps"
    assert "how to read" not in text.lower() and "glossary" not in text.lower()
    assert "Welch's t-test:" not in text                  # one run per condition: no test shown, so no fine print for it


def test_a_run_that_needed_recovery_is_noted_kept_and_not_an_outlier(session):
    _, res, _ = session
    r3 = next(r for r in res["rows"] if r["run_id"] == "3")
    assert r3["recovery"] == "lost" and "needed recovery" in r3["flags"] and r3["outlier_end_err_cm"] == 0
    assert math.isfinite(r3["end_err_cm"])


def test_charts_only_when_there_is_a_spread_to_show(tmp_path):
    rng = np.random.default_rng(6)
    rows = es.fake_runs(rng, 12, _spec())
    ea.flag_runs(rows)
    p = ea.spread_chart(rows, "end_err_cm", "End position error (cm)", str(tmp_path / "a.png"))
    assert p and os.path.getsize(p) > 5000
    one = [r for r in rows if r["condition"] == "U"]
    assert ea.spread_chart(one, "end_err_cm", "x", str(tmp_path / "b.png")) is None      # a single group is not a comparison
    few = [r for r in rows if r["condition"] == "U"][:2] + [r for r in rows if r["condition"] == "C"][:2]
    assert ea.spread_chart(few, "end_err_cm", "x", str(tmp_path / "c.png")) is None      # two dots per group is too few
    bumps = [dict(board="x" if i % 2 else "y", bel_floor_x=50 + i, bel_floor_y=50 + i + 0.3, cam_x=50.2 + i, cam_y=50 + i)
             for i in range(8)]
    assert ea.bump_chart(bumps, str(tmp_path / "d.png"))
    assert ea.bump_chart(bumps[:2], str(tmp_path / "e.png")) is None


def test_the_test_statistics_are_explained_in_fine_print_once_when_first_shown(tmp_path):
    rows = es.fake_runs(np.random.default_rng(12), 10, _spec())
    ea.flag_runs(rows)
    summ, comps = ea.summarise(rows)
    text = open(ea.write_report(str(tmp_path / "r.md"), rows, [], summ, comps, {}, [])).read()
    assert text.count("Welch's t-test:") == 1 and text.count("Cohen's d:") == 1 and text.count("Brown-Forsythe") >= 1
    assert text.index("95% CI of the mean:") < text.index("Welch's t-test:")
    assert "<sub>" in text and "Conclusion" in text and text.index("Premise") < text.index("Data") < text.index("Conclusion")


def test_report_with_no_runs_for_some_conditions_does_not_invent_a_comparison(tmp_path):
    rows = es.fake_runs(np.random.default_rng(9), 5, {"U": dict(end_cm=(3, 1), end_deg=(0, 3))})
    ea.flag_runs(rows)
    summ, comps = ea.summarise(rows)
    assert comps == []
    out = ea.write_report(str(tmp_path / "r.md"), rows, [], summ, comps, {}, [])
    text = open(out).read()
    assert "No runs yet for: C, K, KC" in text and "fewer than two conditions" in text
    assert "Welch's t-test:" not in text                  # no test statistic shown, so no fine print about it


def test_runs_can_be_built_from_the_video_marks_alone(tmp_path):
    sess = es.make_session(str(tmp_path), SPECS[:1], seed=3)
    res = ea.analyse(sess["recording"], [], None, sess["calib"], str(tmp_path / "o"), routes=es.ROUTES_SMALL, quiet=True)
    r = res["rows"][0]
    assert r["run_id"] == "1" and r["condition"] == "" and math.isfinite(r["px_return_dist"])
    assert r["route"] == "" and math.isnan(r["far_err_cm"])        # no route known: no far-point intention, end still measured


def test_command_line_runs_one_run_and_writes_the_report(session, tmp_path):
    sess, _, _ = session
    out = str(tmp_path / "cli")
    ea.main(["--recording", sess["recording"], "--logs", sess["logs"], "--runs", sess["runs"], "--calib", sess["calib"],
             "--out", out, "--only", "2", "--bump-frame", "own"])
    assert os.path.exists(os.path.join(out, "report.md"))
    rows = list(csv.DictReader(open(os.path.join(out, "runs.csv"))))
    assert [r["run_id"] for r in rows] == ["2"]
