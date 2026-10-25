"""2026-10-25 (backing in, learned uncertainty, find recovery, heading fixes): e02_unc (the uncertainty model learned
from the robot's own camera corrections), its wiring in the runner, the find's recovery ladder and hold, and the
heading fixes 1-4 of docs/build-reports/e02-oct25-heading.md (with a simulator robot reproducing the placeholder)."""
import json
import math
import os
import random
import sys
import tempfile
import types

import numpy as np

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02_plan as PL
import e02_unc as U

sys.path.insert(0, os.path.join(KIT.TOOLS, "sim", "tests"))


# --------------------------------------------------------------------------------------------- e02_unc
def synth(true, n, seed=1, cam=0.8, out_share=0.0, turns=(0, 90, 180, 360)):
    rng = np.random.default_rng(seed)
    obs = []
    for _ in range(n):
        d = float(rng.uniform(5, 100))
        t = float(rng.choice(turns))
        s = math.hypot(float(U.sd_of(true, d, t)), cam)
        r = float(np.hypot(*rng.normal(0, s, 2)))
        if rng.random() < out_share:
            r = float(rng.uniform(15, 60))
        obs.append((d, t, r, cam))
    return obs


def test_fit_recovers_the_turning_term_with_outliers():
    true = (0.4, 0.01, 0.6, 0.02)
    f = U.fit(synth(true, 200, out_share=0.08))
    assert set(f["free"]) == {"base_cm", "deg", "deg_per_deg"}
    assert f["p"][1] == U.DEFAULT[1]                          # per_cm held: it can't be told from deg
    ref_true, ref_fit = float(U.sd_of(true, 50, 180)), float(U.sd_of(f["p"], 50, 180))
    assert abs(ref_fit - ref_true) < 0.25 * ref_true, (ref_fit, ref_true)
    assert 0.01 < f["eps"] < 0.2                               # the outliers are found as outliers


def test_floors_hold_and_terms_without_evidence_are_held():
    # a perfect robot: the fit may not go below the floors
    obs = [(d, t, 0.01, 0.0) for d in (10, 30, 60, 90) for t in (0, 180, 360)] * 4
    f = U.fit(obs)
    assert all(p >= fl - 1e-9 for p, fl in zip(f["p"], U.FLOOR))
    # only short drives and no turns: deg and deg_per_deg stay at the start values
    f = U.fit([(5.0, 0.0, 0.5, 0.8)] * 40, start=(0.5, 0.01, 0.7, 0.03))
    assert f["free"] == ("base_cm",) and f["p"][2:] == (0.7, 0.03)


def test_clean_drops_what_is_not_a_correction():
    assert U.clean([(0, 0, 1, 0.8), (500, 0, 1, 0.8), (20, 0, 100, 0.8), (20, float("nan"), 1, 0.8),
                    (20, -90, 1, 0.8)]) == [(20.0, 90.0, 1.0, 0.8)]


def test_model_starts_from_defaults_then_history_then_this_session_and_reports_a_change():
    d = tempfile.mkdtemp()
    path = os.path.join(d, "unc.json")
    m = U.UncModel(path)
    assert m.params("Left") == U.DEFAULT and "defaults" in m.model("Left")["src"]
    for o in synth((0.5, 0.01, 0.5, 0.014), 60, seed=2):
        assert m.add("Left", *o) == ""                          # the same as the defaults: never reported
    assert "this session" in m.model("Left")["src"]
    m.save()
    m2 = U.UncModel(path)
    assert "earlier sessions" in m2.model("Left")["src"]
    assert "every robot" in m2.model("Center")["src"]           # a robot never seen: every robot's history
    # the robot changed (its gyro got worse): reported once, when this session's fit is first used
    said = [m2.add("Left", *o) for o in synth((0.5, 0.01, 0.5, 0.06), 60, seed=3)]
    hits = [w for w in said if w]
    assert len(hits) == 1 and "Left's own position uncertainty" in hits[0], said
    assert m2.sd("Left", 50, 180) > 1.6 * float(U.sd_of(U.DEFAULT, 50, 180))
    js = json.load(open(path))
    assert len(js["robots"]["Left"]) == 60


def test_summary_and_heading_sd():
    m = U.UncModel(None)
    for o in synth(U.DEFAULT, 40, seed=4):
        m.add("Right", *o, kind="kc")
    s = m.summary("Right")
    assert s["corrections"] == 40 and s["kinds"] == {"kc": 40} and 0.8 < s["within_2_45_sd"] <= 1.0
    assert m.heading_sd("Right", 0.0) == m.params("Right")[2]


# --------------------------------------------------------------------------------------------- the runner
class St(dict):
    def save(self):
        pass


def _runner(own=(50.0, 50.0, 0.0), cam=(51.0, 50.0)):
    import e02 as E
    r = E.Runner.__new__(E.Runner)
    r.a = types.SimpleNamespace(dry_run=True, no_unc_learn=False, auto=True)
    r.board = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=119.4, yboard_cm=123.4)
    r.st = St(positions={}, setup={"board_lines_image": {"x": dict(value=119.4), "y": dict(value=123.4)}}, robots={})
    r.said, r.sent, r.marks = [], [], []
    r.say = lambda m, quiet=False: r.said.append(m)
    r.mark = lambda m: r.marks.append(m)
    r.send = lambda robot, c: r.sent.append((robot, c))
    r.tracker = types.SimpleNamespace(active="Left", clear_expect=lambda: None)
    r.listener = object()
    r.settle = lambda **k: cam
    r.own_pose = lambda robot, since=0.0: own
    r.heads, r.unc, r.fix_t, r.moves = {}, {}, {}, {}
    r.camerr = None
    r.uncm = U.UncModel(None)
    r.dir = tempfile.mkdtemp()
    r.order = "test"
    return r


def _fixes(r):
    return [c for _, c in r.sent if c.startswith("fix ")]


def test_fix_now_sends_only_a_measured_heading():
    r = _runner()
    r.heads["Left"] = [90.0, False]                       # a placeholder / a commanded turn: not measured
    r.fix_now("Left")
    assert _fixes(r)[-1].split()[3] == "-"
    r.heads["Left"] = [12.0, True]
    r.fix_now("Left")
    assert _fixes(r)[-1].split()[3] == "12.0"
    r.fix_now("Left", position_only=True)
    assert _fixes(r)[-1].split()[3] == "-"


def test_no_pose_at_all_gets_the_placeholder_and_that_stretch_is_not_learned():
    r = _runner(own=None)
    r.heads["Left"] = [90.0, False]
    r.fix_now("Left")
    assert _fixes(r)[-1].split()[3] == "90.0"
    assert r.unc["Left"]["clean"] is False and r.st["fix_unmeasured_heading"]


def test_begin_moves_uses_the_placeholder_only_without_a_pose():
    r = _runner()
    r.set_corrections = lambda robot, c: []
    r.move_corrections = lambda robot: {}
    r.heads["Left"] = [None, False]
    r.begin_moves("Left")
    assert _fixes(r)[-1].split()[3] == "-"               # its own pose: its own gyro heading kept
    r2 = _runner(own=None)
    r2.set_corrections, r2.move_corrections = (lambda robot, c: []), (lambda robot: {})
    r2.heads["Left"] = [None, False]
    r2.begin_moves("Left")
    assert float(_fixes(r2)[-1].split()[3]) == r2.board.face_xboard


def test_heading_push_after_a_measured_leg():
    r = _runner(own=(50.0, 50.0, 30.0))
    assert r.heading_push("Left", 35.0) is False and not _fixes(r)     # within 8 deg: left alone
    assert r.heading_push("Left", 70.0) is True
    assert _fixes(r)[-1].split()[3] == "70.0" and r.st["heading_pushes"][-1][4] == 40.0


def test_unc_learns_from_clean_chains_only():
    r = _runner(own=(50.0, 50.0, 0.0), cam=(53.0, 54.0))
    r.unc_reset("Left")
    r.unc["Left"].update(dist=40.0, turned=180.0)
    r.heads["Left"] = [0.0, True]
    r.fix_now("Left", 0.0)
    assert r.uncm.session["Left"] == [[40.0, 180.0, 5.0, 0.8]]
    rows = open(os.path.join(r.dir, "unc_obs.csv")).read().splitlines()
    assert rows[0].startswith("t,robot,kind,dist_cm") and len(rows) == 2
    # a run since the last fix: its own driving is not counted, so nothing is learned
    r.watch = types.SimpleNamespace(started={}, wait_start=lambda *a: "run1")
    r.start_run("Left", "run e02")
    r.unc["Left"]["dist"] = 30.0
    r.fix_now("Left", 0.0)
    assert len(r.uncm.session["Left"]) == 1
    # a find does not spoil it (its end pose is exact)
    r.unc["Left"]["dist"] = 30.0
    r.start_run("Left", "run find")
    r.fix_now("Left", 0.0)
    assert len(r.uncm.session["Left"]) == 2


def test_unc_cm_uses_the_learned_model_and_the_fixed_one_on_request():
    import e02 as E
    r = _runner()
    r.unc["Left"] = dict(dist=50.0, turned=180.0)
    assert abs(r.unc_cm("Left") - float(U.sd_of(U.DEFAULT, 50, 180))) < 1e-9
    assert E.UNC_DEG_PER_DEG == 0.014
    for o in synth((0.5, 0.01, 0.5, 0.06), 40, seed=5):
        r.uncm.add("Left", *o)
    assert r.unc_cm("Left") > 1.5 * float(U.sd_of(U.DEFAULT, 50, 180))
    r.a.no_unc_learn = True
    assert abs(r.unc_cm("Left") - float(U.sd_of(U.DEFAULT, 50, 180))) < 1e-9


# --------------------------------------------------------------------------------------------- simulator robot
def test_simulator_reproduces_the_placeholder_heading_and_the_fix_removes_it():
    """A simulated robot (tools/sim) standing at a known pose facing 20 deg; the runner's heading for it is the
    +-90 deg placeholder (after an identify, unmeasured). The old begin_moves wrote it into the robot: a 20 cm drive
    then ends ~20 cm from where the robot believes. The fixed one keeps its gyro heading: under 1 cm."""
    import test_e02 as SIM
    errs = {}
    for how in ("old", "fixed"):
        rb = SIM.make(seed=77, gyro_scale=SIM.GYRO_FIX)
        SIM.place(rb, 60.0, 60.0, 20.0)
        rb.handle_command("fix 60.00 60.00 20.0 1.0", False)          # its pose: known and right
        r = _runner()
        r.set_corrections, r.move_corrections = (lambda robot, c: []), (lambda robot: {})
        r.send = lambda robot, c: rb.handle_command(c, False)
        r.settle = lambda **k: (rb.x, rb.y)
        r.own_pose = lambda robot, since=0.0: (rb.pose.x, rb.pose.y, rb.pose.th())
        r.heads["Left"] = [90.0, False]                               # the placeholder, unmeasured
        if how == "old":
            rb.handle_command("fix %.2f %.2f 90.0 2.0" % (rb.x, rb.y), False)   # what begin_moves used to send
        else:
            r.begin_moves("Left")
        rb.handle_command("go 20 11", False)
        leg = SIM.records(rb, "LEG")[-1]
        errs[how] = math.hypot(float(leg["x_cm"]) - rb.x, float(leg["y_cm"]) - rb.y)
    assert errs["old"] > 15.0 and errs["fixed"] < 1.0, errs


# --------------------------------------------------------------------------------------------- find hold words
def test_find_hold_words_say_what_where_and_what_to_check():
    import e02 as E
    b = PL.Board(xboard_side=1, yboard_side=1, xboard_cm=119.4, yboard_cm=123.4)
    res = "failed: x board: no board seen (front 139 cm; the board should be 20) (not backing in)"
    assert E.find_hold_kind(res) == "no board seen"
    why, detail = E.find_hold_words("Left", res, (60.0, 95.0, 90.0, "the camera"), b)
    assert "x-board" in why and "139 cm" in why and "20 cm" in why
    assert "(60, 95) cm by the camera" in detail and "24 cm from the x-board" in detail
    assert "pressed against the side of the plywood" in detail and "Carry on" in detail and "edge find" in detail
    why, detail = E.find_hold_words("Right", "failed: y board: no board behind within 7 cm", None, b)
    assert "y-board" in why and "back switches" in detail and "not known" in detail
    assert E.find_hold_kind("failed: stopped (plywood edge ahead)") is None


# --------------------------------------------------------------------------------------------- recovery ladder
SEEN = "failed: x board: no board seen (front 139 cm; the board should be 20) (not backing in)"


def _ladder(front, line, retry_ok, seen_res=SEEN):
    r = _runner(own=(60.0, 95.0, 90.0), cam=(60.0, 95.0))
    r.activate = lambda robot: True
    r.begin_moves = lambda robot, why="": True
    r.end_moves = lambda robot: None
    r.legs = []
    r.leg = lambda robot, cmd, run=None: (r.legs.append(cmd), (None, None, "done"))[1]
    r.front_now = lambda robot: front
    r.front_limits = lambda robot=None: (0.0, 3.0)
    r.camera_board_line = lambda which: line
    r.drives = []
    r.drive_to = lambda robot, spot, **k: (r.drives.append(spot), (True, ""))[1]
    r.finds = []

    def do_board_find(robot, order):
        r.finds.append(order)
        if retry_ok:
            return dict(ok=True, result="ok", words="found itself", run="r2", t0=0.0, board_lines={})
        return dict(ok=False, result=seen_res, words="find did not finish", run="r2", t0=0.0)
    r.do_board_find = do_board_find
    r.edge_calls = []
    r.edge_find = lambda robot, order: (r.edge_calls.append(robot), dict(ok=True, result="ok", words="edge"))[1]
    r.recheck_routes = lambda why: None
    r.holds = []
    r.hold_for_nick = lambda robot, kind, why, detail=None: (r.holds.append((kind, why, detail)), "o")[1]
    r.find_hold_where = lambda robot: (60.0, 95.0, 90.0, "the camera")
    r.learn_board_lines = lambda robot, lines: None
    r.listener = types.SimpleNamespace(latest=lambda *a: None)
    return r


def test_ladder_board_there_retry_from_another_spot_works_no_hold():
    r = _ladder(front=19.0, line=119.5, retry_ok=True)
    out, settled, tried = r.find_recover("Left", "t1", dict(ok=False, result=SEEN, words="x"))
    assert settled and out["ok"] and r.finds == ["t1"] and not r.holds and not r.edge_calls
    steps = [s_["step"] for s_ in r.st["find_recoveries"][-1]["steps"]]
    assert steps == ["fix", "look", "picture", "retry"]
    assert r.legs == []                                                            # it already faced the board
    spot = r.drives[0]
    assert abs(spot[1] - (119.4 - r.FIND_RESTAGE_CM)) < 1e-6 and abs(spot[0] - 60.0) >= r.FIND_RESTAGE_ALONG_CM
    assert any(c.startswith("fix ") for _, c in r.sent)                            # (1) the fresh camera fix


def test_ladder_both_say_gone_marks_the_board_missing_and_edge_finds():
    r = _ladder(front=None, line=None, retry_ok=False)
    out, settled, tried = r.find_recover("Left", "t1", dict(ok=False, result=SEEN, words="x"))
    assert settled and r.edge_calls == ["Left"] and "x" in r.board.missing and not r.finds
    assert ("y", r.board.y1) in r.board.open_edges()                          # its line is an open edge now
    assert r.st["board_changes"][-1][2].startswith("x board gone")
    assert [s_["step"] for s_ in r.st["find_recoveries"][-1]["steps"]][-1] == "missing"


def test_ladder_disagreeing_checks_retry_then_hold_with_what_it_tried():
    r = _ladder(front=140.0, line=119.6, retry_ok=False)               # sensor: gone; picture: there
    out = r.do_find("Left", "t1")
    assert not out["ok"] and len(r.holds) == 1 and not r.edge_calls and "x" not in r.board.missing
    kind, why, detail = r.holds[0]
    assert kind == "find" and detail.startswith("What the runner tried first: a fresh camera fix")
    assert "front sensor read 140 cm" in detail and "the find again from" in detail
    assert len(r.finds) == 2                                           # the first find, and the retry


def test_ladder_left_look_turns_to_face_the_board():
    r = _ladder(front=19.0, line=119.5, retry_ok=True)
    r.own_pose = lambda robot, since=0.0: (60.0, 95.0, -120.0)
    r.find_recover("Left", "t1", dict(ok=False, result=SEEN, words="x"))
    assert r.legs[0] == ("turn", -150.0)                             # from -120 to +90 (the x-board ahead)


def test_live_page_hold_carries_the_detail():
    import e02_view as V
    r = types.SimpleNamespace(st={"holds": [[1.0, "Left", "find", "its find stopped", "t", None, None, "Check: x"]]},
                              hold=types.SimpleNamespace(is_set=lambda: True))
    v = V.LiveView.__new__(V.LiveView) if hasattr(V, "LiveView") else None
    if v is None:
        cls = next(c for c in vars(V).values() if isinstance(c, type) and hasattr(c, "_live_hold"))
        v = cls.__new__(cls)
    v.r = r
    h = v._live_hold()
    assert h["kind"] == "find" and h["detail"] == "Check: x"


def test_ladder_look_drives_away_from_the_board_before_turning():
    r = _ladder(front=19.0, line=119.5, retry_ok=True)
    r.own_pose = lambda robot, since=0.0: (60.0, 112.0, -90.0)      # backed in: 7 cm from the x-board, facing away
    r.settle = lambda **k: (60.0, 102.0)
    f, expect = r.find_look("Left", "x", (60.0, 112.0))
    assert r.legs[0][0] == "go" and abs(r.legs[0][1] - (r.FIND_LOOK_ROOM_CM - 7.4 + 2.0)) < 1e-6
    assert r.legs[1][0] == "turn" and abs(r.legs[1][1]) == 180.0
    assert abs(expect - (119.4 - 102.0 - PL.ROBOT_HALF_CM)) < 1e-6
    r2 = _ladder(front=19.0, line=119.5, retry_ok=True)
    r2.own_pose = lambda robot, since=0.0: (60.0, 112.0, 90.0)      # facing the board, too near to turn: no look
    assert r2.find_look("Left", "x", (60.0, 112.0)) == (None, None) and r2.legs == []


def test_find_heading_prefers_a_measured_heading_when_its_own_is_stale():
    import time as _t
    r = _runner(own=(50.0, 50.0, -143.0))                  # its gyro crept while it stood parked
    r.heads["Left"] = [13.0, True]
    recs = {"LEG": (_t.time() - 300.0, {}), "FIND": None}
    r.listener = types.SimpleNamespace(latest=lambda kind, robot, *a, **k: recs[kind])
    assert r.find_heading_ok("Left") == (13.0, False) and r.st["find_heading_stale"]
    recs["LEG"] = (_t.time() - 2.0, {})                    # it just drove: its own heading counts
    assert r.find_heading_ok("Left") == (None, True)
