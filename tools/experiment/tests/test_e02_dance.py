"""The dance (tools/experiment/e02_dance.py): speeds from LEG records, the gap rule, the composer and its checker,
the authored 3-robot dance, the corrected legs, role assignment, the music and the dance page."""
import json
import math
import os
import tempfile
import wave

import numpy as np
import pytest

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02_plan as PL
import e02_dance as DN


def board(xs=1, ys=1, inset=0.6):
    bd = PL.Board(xboard_side=xs, yboard_side=ys)
    bd.xboard_cm = (bd.y0 + inset) if xs == 0 else (bd.y1 - inset)
    bd.yboard_cm = (bd.x0 + inset) if ys == 0 else (bd.x1 - inset)
    return bd


REAL = DN.Speeds.default()
SIM = DN.Speeds(0.13, 10.0, 0.75, 94.0, "simulator")


# =============================================================================== speeds
def leg_recs(robot, legs, t0=1000.0):
    """LEG records (t, robot, rec) for legs [(kind, value, motion s)]: started, then done after LEARN_S + motion."""
    out, ms = [], 50000.0
    for i, (kind, val, mot) in enumerate(legs):
        out.append((t0, robot, dict(run="-", id=str(i + 1), kind=kind, value="%.2f" % val, state="started", ms="%d" % ms)))
        ms += (DN.LEARN_S + mot) * 1000.0
        out.append((t0, robot, dict(run="-", id=str(i + 1), kind=kind, value="%.2f" % val, state="done", ms="%d" % ms)))
        ms += 800.0
    return out


def test_speeds_fitted_from_the_sessions_own_legs():
    legs = [("go", d, 0.2 + d / 9.5) for d in (5.0, 10.0, 25.0, 40.0, 60.0)] + \
           [("turn", a, 0.5 + a / 80.0) for a in (30.0, 40.0, 90.0, 140.0)] + [("go", -6.0, 0.2 + 6 / 9.5)] + \
           [("turn", 8.0, 0.3)]                                                # small turns are left out of the fit
    sp = DN.fit_speeds(DN.leg_pairs(leg_recs("Left", legs), "Left"))
    assert sp.go_cms == pytest.approx(9.5, rel=0.02) and sp.go_lag_s == pytest.approx(0.2, abs=0.03)
    assert sp.turn_dps == pytest.approx(80.0, rel=0.03) and sp.turn_lag_s == pytest.approx(0.5, abs=0.05)
    assert sp.n_go == 6 and sp.n_turn == 4 and "this session" in sp.source
    # too few legs (all the same size): the real robots' 2026-10-21 values, said so
    few = DN.fit_speeds(DN.leg_pairs(leg_recs("Left", [("turn", 40.0, 1.0)] * 6), "Left"))
    assert few.turn_dps == REAL.turn_dps and "2026-10-21" in few.source
    # a robot with too few legs of its own gets every robot's legs together
    recs = leg_recs("Left", legs) + leg_recs("Right", [("turn", 40.0, 1.0)] * 3, t0=2000.0)
    both = DN.fit_all(recs, ["Left", "Right"])
    assert both["Right"].go_cms == pytest.approx(9.5, rel=0.02) and "all robots together" in both["Right"].source
    # the real 2026-10-21 legs: 25 cm in about 3.1 s, 90 deg in about 1.9 s
    assert REAL.duration("go", 25.0) == pytest.approx(3.09, abs=0.1)
    assert REAL.duration("turn", 90.0) == pytest.approx(1.9, abs=0.1)


# =============================================================================== the gap rule
def test_gap_rule_default_then_proven_and_shrinking():
    d = DN.gap_rule([])
    assert d["gap_cm"] == pytest.approx(DN.K_GAP * DN.DEFAULT_E95_CM) and not d["proven"]
    assert d["hard_cm"] == pytest.approx(DN.DEFAULT_E95_CM) and "default" in d["words"]
    rng = np.random.default_rng(1)
    few = list(np.abs(rng.normal(0, 1.2, 40)))            # not enough moves to prove anything yet
    assert not DN.gap_rule(few)["proven"]
    many = list(np.abs(rng.normal(0, 1.2, 300)))           # 300 dance moves, ~2.4 cm at the 95th percentile
    pr = DN.gap_rule(many, "test dances", timing_p95_s=0.12)
    assert pr["proven"] and pr["e95_cm"] >= pr["p95_cm"]   # an upper bound, never below the measured 95th
    assert pr["e95_cm"] < 3.5 and pr["gap_cm"] < d["gap_cm"]
    assert pr["jitter_s"] == pytest.approx(0.12) and pr["speed_tol"] < DN.SPEED_TOL
    assert DN.gap_rule([0.1] * 100)["gap_cm"] == DN.GAP_FLOOR_CM          # never closer than the floor


def test_upper_p95_needs_59_samples():
    assert DN.upper_p95(list(range(58))) is None
    assert DN.upper_p95(list(range(59))) == 58


def test_precision_read_from_earlier_dances_dry_and_real_kept_apart():
    d = tempfile.mkdtemp()
    for name, dry in (("e02-20261022-100000-dryrun", True), ("e02-20261022-110000", False)):
        os.makedirs(os.path.join(d, name))
        with open(os.path.join(d, name, "dance_result.json"), "w") as f:
            json.dump(dict(move_dev_cm=[1.0 if dry else 4.0] * 70, timing_robot_s=dict(p95_abs=0.1)), f)
    sim = DN.load_precision(d, dry_run=True)
    real = DN.load_precision(d, dry_run=False)
    assert sim["e95_cm"] == pytest.approx(1.0) and real["e95_cm"] == pytest.approx(4.0)
    assert sim["proven"] and "simulated" in sim["source"]


# =============================================================================== composer and checker
def two_robot_dance(gap_rule, y_b=60.0, bpm=120.0, d=27.0):
    """A and B drive at each other along y = 60 (B offset to y_b): head on when y_b = 60."""
    start = dict(A=(30.0, 60.0, 0.0), B=(92.0, y_b, 180.0))
    moves = dict(A=[dict(kind="go", value=d, beat=0.0)], B=[dict(kind="go", value=d, beat=0.0)])
    return DN.Dance(bpm, ["A", "B"], start, moves, [dict(name="test", words="", beat=0.0)], gap_rule)


def test_checker_finds_a_collision_and_passes_a_near_miss():
    rule = DN.gap_rule([])
    bd = board()
    sp = dict(A=REAL, B=REAL)
    bad = DN.check(two_robot_dance(rule), sp, bd, rule)
    assert not bad["ok"] and any("apart" in p for p in bad["problems"])
    assert bad["closest"]["A-B"]["gap_cm"] < 0                  # they would meet head on
    # the same, B's line moved sideways by 2 R + gap: a pass at exactly the gap
    ok = DN.check(two_robot_dance(rule, y_b=60.0 + 2 * DN.R_BODY + rule["gap_cm"] + 0.2, d=45.0), sp, bd, rule)
    assert ok["closest"]["A-B"]["gap_cm"] == pytest.approx(rule["gap_cm"] + 0.2, abs=0.1)
    assert not any("apart" in p for p in ok["problems"])


def test_checker_finds_open_edges_boards_front_stops_and_overlaps():
    rule = DN.gap_rule([])
    bd = board()                                # boards along x = max and y = max; open edges x = 0 and y = 0
    sp = dict(A=REAL)
    start = dict(A=(30.0, 60.0, 180.0))
    edge = DN.Dance(120, ["A"], start, dict(A=[dict(kind="go", value=15.0, beat=0.0)]), [], rule)
    chk = DN.check(edge, sp, bd, rule)
    assert any("open edge" in p for p in chk["problems"])
    # a leg planned up to the y-board's face (x = 121.55): its front sensor stops it short, and it is too near
    start = dict(A=(90.0, 60.0, 0.0))
    wall = DN.Dance(120, ["A"], start, dict(A=[dict(kind="go", value=28.0, beat=0.0)]), [], rule)
    chk = DN.check(wall, sp, bd, rule)
    assert any("front sensor" in p for p in chk["problems"]) and chk["front_risks"]
    assert any("blocked board" in p for p in chk["problems"])
    # the firmware lets a leg come close to a board it ends in front of (rest of the leg + 2 cm): no front stop
    near = DN.Dance(120, ["A"], start, dict(A=[dict(kind="go", value=21.0, beat=0.0)]), [], rule)
    assert not DN.check(near, sp, bd, rule)["front_risks"]
    # a robot just ahead of the sensor, within the rest of the leg: it would see it
    assert DN._front_stop(bd, 60.0, 60.0, 0.0, 10.0, [(60.0 + 4.5 + DN.R_BODY + 3.0, 61.0)]) == pytest.approx(3.0, abs=0.2)
    assert DN._front_stop(bd, 60.0, 60.0, 0.0, 10.0, [(60.0, 80.0)]) is None
    lap = DN.Dance(120, ["A"], dict(A=(60.0, 60.0, 0.0)), dict(A=[dict(kind="turn", value=90.0, beat=0.0),
                                                                    dict(kind="turn", value=90.0, beat=1.0)]), [], rule)
    assert any("before the robot's previous move" in p for p in DN.check(lap, sp, bd, rule)["problems"])


def test_safe_moves_wait_for_their_moment_a_near_miss_on_purpose():
    """B crosses A's path square to it 'as soon as it is safe': it passes just behind A, at the gap."""
    rule = DN.gap_rule([])
    bd = board()
    c = DN.Choreo(bd, 120, dict(A=REAL, B=REAL, C=REAL), rule, roles=("A", "B", "C"))
    c._start = dict(A=(25.0, 60.0, 0.0), B=(60.0, 25.0, 90.0), C=(95.0, 100.0, 180.0))
    for r, p in c._start.items():
        c.place(r, p[:2], p[2])
    c.go("A", 70.0, 0.0)
    c.go("B", 70.0, 0.0, safe=True)
    g = c.min_gap("B", 0.0)
    assert rule["gap_cm"] <= g < rule["gap_cm"] + 3.0, g
    assert c.moves["B"][-1]["beat"] > 0.0                       # it waited


@pytest.mark.parametrize("speeds", [REAL, SIM], ids=["real robots", "simulator"])
@pytest.mark.parametrize("sides", [(1, 1), (0, 0), (0, 1)])
def test_the_authored_dance_passes_its_check_and_lasts_60_to_90_s(speeds, sides):
    rule = DN.gap_rule([])
    bd = board(*sides)
    sp = {r: speeds for r in DN.ROLES}
    dn, chk = DN.author_and_check(bd, rule, sp)
    assert chk["ok"], chk["problems"]
    assert 60.0 <= chk["duration_s"] <= 92.0, chk["duration_s"]
    pr, c = chk["closest_pair"]
    assert rule["gap_cm"] <= c["gap_cm"] < rule["gap_cm"] + 1.5          # just barely missing
    assert chk["robust_gap_cm"] >= rule["hard_cm"]
    assert chk["open_edge_cm"] >= rule["gap_cm"] and chk["board_cm"] >= DN.BOARD_CLEAR_CM
    names = [f["name"] for f in dn.figures]
    for want in ("Wake up", "Near-miss swap", "Crossing", "Orbit", "Follow the leader", "Finale"):
        assert want in names
    kinds = {m["kind"] for r in dn.roles for m in dn.moves[r]}
    assert kinds == {"go", "turn", "flash"} and chk["flash_on_beat"]
    for r in dn.roles:
        beats = [m["beat"] for m in dn.moves[r]]
        assert beats == sorted(beats) and all(abs(b * 2 - round(b * 2)) < 1e-9 for b in beats)   # half-beat grid
        assert all(-DN.BACK_MAX_CM <= m["value"] for m in dn.moves[r] if m["kind"] == "go")
        assert all(abs(m["value"]) <= 180.0 for m in dn.moves[r] if m["kind"] == "turn")
    # every robot ends facing out of the final triangle (parking drives forward first)
    O = DN.Stage(bd, rule["gap_cm"]).O
    for r in dn.roles:
        last = [m for m in dn.moves[r] if m["kind"] != "flash"][-1]
        out = DN.bearing(O, (last["x1"], last["y1"]))
        assert abs(DN.wrap180(last["h1"] - out)) < 25.0
    words = DN.summary_words(dn, chk, rule)
    assert any("Gap rule" in w for w in words) and any("Closest the plan comes" in w for w in words)
    assert json.loads(json.dumps(dn.to_dict()))["bpm"] == 120.0


def test_a_proven_smaller_gap_brings_them_closer():
    bd = board()
    sp = {r: REAL for r in DN.ROLES}
    proven = DN.gap_rule(list(np.full(80, 2.0)), "test", timing_p95_s=0.1)
    dn, chk = DN.author_and_check(bd, proven, sp)
    assert chk["ok"], chk["problems"]
    assert chk["closest_pair"][1]["gap_cm"] < DN.gap_rule([])["gap_cm"]


def test_flashes_are_left_out_when_the_tempo_does_not_fit_twice_a_second():
    rule = DN.gap_rule([])
    dn, chk = DN.author_and_check(board(), rule, {r: REAL for r in DN.ROLES}, bpm=100.0)
    assert chk["ok"] and not chk["flash_on_beat"]
    assert any("NOT on the beat" in w for w in DN.summary_words(dn, chk, rule))


# =============================================================================== running it
def test_roles_go_to_the_nearest_start_spots():
    form = dict(A=(10.0, 10.0, 0.0), B=(100.0, 10.0, 0.0), C=(50.0, 100.0, 0.0))
    pos = dict(Left=(98.0, 12.0), Center=(52.0, 95.0), Right=(12.0, 8.0))
    assert DN.assign_roles(form, pos) == dict(A="Right", B="Left", C="Center")


def test_legs_are_corrected_from_the_camera_but_capped():
    go = dict(kind="go", value=30.0, x0=0.0, y0=0.0, h0=0.0, x1=30.0, y1=0.0, h1=0.0)
    v, _ = DN.corrected_command(go, (2.0, 0.5), 0.0)
    assert v == pytest.approx(28.0)                          # 2 cm further along: 2 cm shorter
    v, _ = DN.corrected_command(go, (-40.0, 0.0), 0.0)
    assert v == pytest.approx(30.0 + 7.5)                    # a wild camera reading: capped at 25 %
    assert DN.corrected_command(go, None, 0.0)[0] == 30.0    # no camera position: as planned
    back = dict(go, value=-6.0, x1=-6.0)
    assert -DN.BACK_MAX_CM <= DN.corrected_command(back, (3.0, 0.0), 0.0)[0] < 0
    turn = dict(kind="turn", value=90.0, h0=0.0, h1=90.0, aim=[0.0, 40.0])
    v, _ = DN.corrected_command(turn, (3.0, 0.0), 2.0)       # aims at the go's end from where it really is
    assert v == pytest.approx(DN.bearing((3.0, 0.0), (0.0, 40.0)) - 2.0, abs=0.01)
    v, _ = DN.corrected_command(dict(turn, aim=None), (0.0, 0.0), 30.0)
    assert v == pytest.approx(90.0 - 20.0)                   # heading 30 off: capped at 20 deg
    spin = dict(kind="turn", value=180.0, h0=0.0, h1=180.0, piece=[1, 3])
    assert DN.corrected_command(spin, (0.0, 0.0), 10.0)[0] == 180.0   # inside a long spin: as planned


def test_the_music_is_at_the_dance_tempo_with_a_count_in():
    d = tempfile.mkdtemp()
    p = os.path.join(d, "m.wav")
    info = DN.make_music(p, 120.0, 32, count_in=8, accents=[0, 16], flashes=[4])
    with wave.open(p) as w:
        sr, n = w.getframerate(), w.getnframes()
        x = np.frombuffer(w.readframes(n), np.int16).astype(float)
    assert info["count_in"] == 8 and abs(n / sr - info["seconds"]) < 0.01
    assert n / sr >= (8 + 32) * 0.5
    # loud onsets fall on the beat grid: energy right after each beat is far above the energy just before it
    spb = int(sr * 0.5)
    after = [np.abs(x[k * spb: k * spb + 400]).mean() for k in range(8, 40)]
    before = [np.abs(x[k * spb - 1200: k * spb - 800]).mean() for k in range(8, 40)]
    assert np.median(after) > 2.0 * np.median(before)


def test_the_dance_page_is_added_to_the_live_view_without_changing_it():
    class View:
        def extra_get(self, path):
            return (b"old", "text/plain") if path == "/other" else None

    class Run:
        music_path = None

        def page_state(self):
            return dict(phase="dancing")
    v = View()
    DN.DancePage(v, Run())
    body, ctype = v.extra_get("/dance")
    assert b"Start the sound" in body and "html" in ctype
    assert json.loads(v.extra_get("/dance/state.json")[0])["phase"] == "dancing"
    assert "t" in json.loads(v.extra_get("/dance/clock.json")[0])
    assert v.extra_get("/other") == (b"old", "text/plain") and v.extra_get("/nothing") is None


def test_the_dance_options_are_off_by_default():
    import e02
    import argparse
    seen = {}

    def fake_launch(a, state, sdir, sim):
        seen["a"] = a
        return 0
    old_launch, old_start = e02.launch, e02.start_sim
    e02.launch = fake_launch
    try:
        d = tempfile.mkdtemp()
        old_repo = e02.REPO
        e02.REPO = d
        e02.main(["--auto"])
    finally:
        e02.launch, e02.start_sim = old_launch, old_start
        e02.REPO = old_repo
    a = seen["a"]
    assert isinstance(a, argparse.Namespace)
    assert not a.dance and not a.dance_only and a.dance_bpm == 120.0
