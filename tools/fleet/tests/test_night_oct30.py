"""The 29 Oct night DOE loop (doe-20261029-211053-doe, build report e02-oct30-overnight.md): crawls choose their way
from the heading the loop has or had, never into the robots' own virtual border; a refused back leg gets the robot a
pose; a start-up that sent nothing keeps the heading; the side camera's learned offset carries over; a resent leg's
refusal is not its end; a drive's park spot keeps off the edges."""
import math
import os
import sys
import time
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
sys.path.insert(0, FLEET)

import ladder as LD        # noqa: E402
import doe_loop as DL      # noqa: E402

RECT = (0.0, 0.0, 122.0, 122.0)


def _lad(others=None, robots=None):
    L = LD.Ladder.__new__(LD.Ladder)
    L.rect = RECT
    L.others_of = lambda n: dict(others or {})
    L.st = {"robots": robots or {}, "kept_out": {}, "passed": {}}
    L.save = lambda: None
    return L


# ------------------------------------------------------------------ 1. which way a crawl goes (the three that ended the run)

def test_center_2136_known_heading_backs_first():
    """Center at (93.4, 72.6) facing 3.5 deg (its own gyro 13.3): forwards went into its border at x = 102.6."""
    L = _lad()
    for h in (3.5, 13.3):
        assert L.crawl_sign("Center", (93.36, 72.59), h, LD.KNOWN_SPREAD_DEG) == -1
    # the old rule's forward room: the tip at 18.6 cm from x = 122, inside the robot's own border (19.5 cm)
    assert 122.0 - (93.36 + 10 * math.cos(math.radians(13.3))) < LD.ROBOT_BORDER_IN_CM


def test_left_2215_hint_backs_first():
    """Left at (95.6, 27.8): its heading -28.3 deg (drive 17's camera line) was dropped by the find step; forwards
    met its border at (102.5, 23.8)."""
    L = _lad()
    assert L.crawl_sign("Left", (95.59, 27.75), -28.28, LD.HINT_SPREAD_DEG) == -1


def test_right_2216_eight_cm_from_the_edge_backs_away():
    """Right 8 cm from x = 0 facing about -150 deg (towards the edge): it backs away from it."""
    L = _lad()
    assert L.crawl_sign("Right", (8.13, 38.23), -150.0, LD.HINT_SPREAD_DEG) == -1


def test_parallel_to_the_border_with_a_hint_turns_to_the_middle_first():
    L = _lad()
    sent = []
    L.bench = types.SimpleNamespace(leg=lambda n, t, to: sent.append(t) or ("done", None, None, None, ""))
    # 21 cm from y = 0 facing along it: +-30 deg either way could close 5 cm on the edge
    assert L.crawl_sign("Left", (60.0, 21.0), 0.0, LD.HINT_SPREAD_DEG) is None
    sg, h = L.crawl_way("Left", (60.0, 21.0), 0.0, LD.HINT_SPREAD_DEG)
    assert sent and sent[0].startswith("turn ") and sg == 1
    assert abs(LD.wrap180(h - math.degrees(math.atan2(61.0 - 21.0, 61.0 - 60.0)))) < 0.01
    # inside the turn check the turn is not allowed (it would be measured as part of the check's turn)
    sent.clear()
    with pytest.raises(LD.LadderStop):
        L.crawl_way("Left", (60.0, 21.0), 0.0, LD.HINT_SPREAD_DEG, may_turn=False)
    assert not sent


def test_middle_of_the_board_goes_forwards():
    L = _lad()
    for h in range(-180, 180, 30):
        assert L.crawl_sign("Left", (61.0, 61.0), h, LD.HINT_SPREAD_DEG) == 1


def test_turn_check_spot_25_cm_in_parallel_still_crawls():
    """The turn check allows 25 cm from an edge: a known heading along it must still crawl (forwards)."""
    L = _lad()
    assert L.crawl_sign("Left", (60.0, 25.0), 0.0, LD.KNOWN_SPREAD_DEG) == 1


# ------------------------------------------------------------------ 2. the crawl itself

class _Eye:
    def __init__(self, stills, hist):
        self.it = iter(stills)
        self.trk = types.SimpleNamespace(history=lambda n, a, b: hist)

    def still(self, n):
        return next(self.it)

    def sources(self, n, a, b):
        return {"both": 30}


def test_backing_first_sends_the_heading_with_the_fix(monkeypatch):
    monkeypatch.setattr(LD.time, "sleep", lambda s: None)
    L = _lad()
    sent, legs = [], []
    L.bench = types.SimpleNamespace(leg=lambda n, t, to: legs.append(t) or ("done", 0, 0, 0.0, ""),
                                    send=lambda n, c: sent.append(c))
    # backs from (93, 72) facing 3.5 deg: to (83, 72) and back; the camera line points at -176.5 (travel), so 3.5
    hist = [(i, 93.0 - i * 10.0 / 7, 72.0 - i * 0.087) for i in range(8)]
    L.eye = _Eye([(83.0, 71.4), (93.0, 72.0)], hist)
    h, d = L.crawl("Center", (93.0, 72.0), None, hint=3.5)
    assert sent[0].startswith("fix 93.00 72.00 3.50 ") and legs[0] == "go -10.0" and legs[1] == "go 10.0"
    assert abs(LD.wrap180(h - 3.5)) < 1.0


def test_back_leg_refused_without_a_pose_gets_one_from_the_forward_leg(monkeypatch):
    """29 Oct 22:11-22:16: Right had no pose (its start-up had failed), every position-only fix was refused, so every
    back leg after the first was refused and three crawls left it 30 cm on, at the x = 0 edge."""
    monkeypatch.setattr(LD.time, "sleep", lambda s: None)
    L = _lad()
    sent, legs = [], []
    answers = iter([("done", 0, 0, 0.0, ""),
                    ("refused", 0, 0, 0.0, "backing blind: more than e02_back_max_cm since the last camera fix"),
                    ("done", 0, 0, 0.0, "")])
    L.bench = types.SimpleNamespace(leg=lambda n, t, to: legs.append(t) or next(answers),
                                    send=lambda n, c: sent.append(c))
    hist = [(i, 50.0 + i * 10.0 / 7, 50.0) for i in range(8)]
    L.eye = _Eye([(60.0, 50.0), (50.0, 50.0)], hist)
    h, d = L.crawl("Right", (50.0, 50.0), None)          # no heading at all: forwards first, as before
    assert legs == ["go 10.0", "go -10.0", "go -10.0"]
    assert sent[0].startswith("fix 50.00 50.00 - ") and sent[1].startswith("fix 60.00 50.00 0.00 ")
    assert abs(h) < 1e-6 and d["back_off_cm"] == 0.0


# ------------------------------------------------------------------ 3. hints

def test_hint_kept_when_the_heading_is_dropped_and_only_near_its_spot():
    L = _lad(robots={"Left": {"x": 95.7, "y": 26.0, "h": -28.28}})
    L.forget_heading("Left", "test")
    r = L.st["robots"]["Left"]
    assert "h" not in r and r["h_hint"] == -28.28
    assert L.heading_hint("Left", (95.59, 27.75)) == -28.28              # 1.8 cm away
    assert L.heading_hint("Left", (95.7 + LD.HINT_NEAR_CM + 1, 26.0)) is None
    r["h"] = 10.0
    assert L.heading_hint("Left", (0.0, 0.0)) == 10.0                    # a known heading always


def test_doe_hold_keeps_a_hint():
    L = DL.DoeLoop.__new__(DL.DoeLoop)
    L.st = {"robots": {"Right": {"x": 30.0, "y": 50.0, "h": -140.0}}, "doe": {}, "kept_out": {}}
    L.save = lambda: None
    L.row = lambda *a, **k: None
    L.frames = None
    L.hold("Right", "test", "side")
    assert "h" not in L.st["robots"]["Right"] and L.st["robots"]["Right"]["h_hint"] == -140.0


# ------------------------------------------------------------------ 4. the link: a resent leg's refusal

class _ResendBench(LD.Bench):
    """The first `go`'s queued/started answers are lost; the copy is refused (a leg is running); then the first one
    ends."""

    def __init__(self):
        import threading
        from collections import deque
        self.lock = threading.Lock()
        self.legs, self.state, self.fw, self.msgs = {}, {}, {}, deque()
        self.sent = []

    def send(self, name, cmd):
        self.sent.append(cmd)
        if cmd.startswith("go") and len([c for c in self.sent if c.startswith("go")]) == 2:
            t = time.time()
            self.legs.setdefault(name, []).append((t, "go", 0.0, "refused", 0, 0, 0, "a leg is running"))

            def end():
                time.sleep(0.4)
                with self.lock:
                    self.legs[name].append((time.time(), "go", 10.0, "done", 1.0, 2.0, 3.0, "front=50"))
            import threading
            threading.Thread(target=end, daemon=True).start()


def test_a_resent_legs_refusal_is_not_its_end():
    b = _ResendBench()
    r = b.leg("Right", "go 10", 5)
    assert r[0] == "done" and r[1] == 1.0
    assert [c for c in b.sent if c.startswith("go")] == ["go 10", "go 10"]


# ------------------------------------------------------------------ 5. the drive's park spot

@pytest.mark.parametrize("p", [(32.3, 24.0), (35.4, 27.4), (94.5, 27.3), (96.2, 26.7), (30.6, 49.3), (27.8, 56.5)])
def test_park_spot_a_keeps_off_the_edges(p):
    """The night's start spots near an edge (Left's two corners, Right near x = 0): A is now 30 cm in."""
    for length in DL.LENGTHS:
        A, B = DL.pick_segment(RECT, p, length, [])
        assert DL.edge_dist(RECT, *A) >= DL.CLEAR_CM + DL.A_INWARD_CM - 1e-6
        assert DL.edge_dist(RECT, *B) >= DL.CLEAR_CM - 1e-6
        assert abs(math.dist(A, B) - length) < 0.01


def test_park_spot_falls_back_when_nothing_fits_30_cm_in():
    rect = (0.0, 0.0, 122.0, 61.0)            # Monday's table: 30 cm in leaves a 1 cm strip
    seg = DL.pick_segment(rect, (40.0, 30.0), 40.0, [])
    assert seg is not None and DL.edge_dist(rect, *seg[0]) >= DL.CLEAR_CM - 1e-6


# ------------------------------------------------------------------ 6. the side camera's learned offset

def test_learned_offset_is_kept_and_seeds_the_next_tracker():
    import e02_twocam as TW
    L = _lad()
    eye = LD.Eye.__new__(LD.Eye)
    eye.side_cam = object()
    eye.side_bias = (0.0, 0.0)
    eye.trk = types.SimpleNamespace(bias=[-4.4, 3.7], bias_n=TW.BIAS_MIN_N)
    L.eye = eye
    assert L.note_bias() == (-4.4, 3.7)
    assert L.kept_bias() == (-4.4, 3.7) and eye.side_bias == (-4.4, 3.7)
    # not learned yet (under BIAS_MIN_N pictures): the kept one stays
    eye.trk = types.SimpleNamespace(bias=[0.0, 0.0], bias_n=3)
    assert L.note_bias() is None and L.kept_bias() == (-4.4, 3.7)


def test_bridge_starts_from_the_side_bias_it_is_given():
    import cam_bridge as CB
    side = ("cam", 0.02, (0.0, 0.0))
    new, words = CB.with_side_bias(side, "-4.40,3.70")
    assert new == ("cam", 0.02, (-4.4, 3.7)) and "learned this session" in words
    for bad in ("", "x,y", "1", "nan,2"):
        assert CB.with_side_bias(side, bad)[0] == side


# ------------------------------------------------------------------ 7. a start-up that sent nothing keeps the heading

def _drive_stub(tmp_path, summary):
    L = DL.DoeLoop.__new__(DL.DoeLoop)
    L.a = types.SimpleNamespace(keep_going=True)
    L.st = {"kept_out": {}, "doe": {}, "robots": {"Right": {"x": 32.0, "y": 53.0, "h": 178.0}}}
    L.rect = RECT
    L.bench, L.frames, L.files, L.row_err = None, None, {}, False
    L.dir = str(tmp_path)
    L.save = lambda: None
    L.frames_on = L.frames_off = lambda *a: None
    L.row = lambda *a, **k: None
    sess = tmp_path / "drive13-Right-5cms-40cm"
    (sess / "bridge").mkdir(parents=True, exist_ok=True)
    L.run_planned = lambda tag, m, at, sub: dict(code=1, summary=summary, check={"robots": {}}, sess=str(sess))
    # the bridge saw it (side camera alone) 5.7 cm off and no camera line: before, the heading was forgotten here
    L.end_poses = lambda sess, names: {"Right": dict(x=28.0, y=56.8, h=None, how="none")}
    return L


def test_start_up_failed_with_nothing_sent_keeps_the_heading(tmp_path, monkeypatch):
    monkeypatch.setattr(DL, "miss_pattern", lambda *a, **k: dict(n=0))
    monkeypatch.setattr(DL, "sensor_runs", lambda *a, **k: (dict(readings=0, low_readings=0, low_runs=0, single=0,
                                                                  stops=0, stops_avoided_by_2=0, stops_avoided_by_3=0,
                                                                  stop_detail=[]), []))
    summ = {"result": "start-up failed", "steps_sent": {"Right": 0},
            "start_checks": {"Right": {"place_cm": 5.69, "heading_deg": 0.0, "heading_allowed_deg": 10.0}}}
    L = _drive_stub(tmp_path, summ)
    assert L.drive(str(tmp_path), "Right", 12, 29) == "fail"
    assert L.st["robots"]["Right"]["h"] == 178.0 and "Right" not in L.st["kept_out"]

    # a start-up that sent steps (it moved), or failed on its heading: the heading goes, as before
    summ2 = dict(summ, steps_sent={"Right": 2})
    L = _drive_stub(tmp_path, summ2)
    L.drive(str(tmp_path), "Right", 12, 29)
    assert "h" not in L.st["robots"]["Right"] and L.st["robots"]["Right"]["h_hint"] == 178.0
    summ3 = dict(summ, start_checks={"Right": {"place_cm": 1.0, "heading_deg": 25.0, "heading_allowed_deg": 10.0}})
    L = _drive_stub(tmp_path, summ3)
    L.drive(str(tmp_path), "Right", 12, 29)
    assert "h" not in L.st["robots"]["Right"]
    # run_plan calibrated it first (a restart): the roll moved it, the heading goes (dry run, 30 Oct 07:19)
    summ4 = dict(summ, start_checks={"Right": {"restarted": True}})
    L = _drive_stub(tmp_path, summ4)
    L.drive(str(tmp_path), "Right", 12, 29)
    assert "h" not in L.st["robots"]["Right"]


def test_the_bridge_command_line_takes_a_negative_offset(tmp_path):
    """First dry run, 30 Oct: ["--side-bias", "-4.40,3.70"] stopped the bridge (argparse took the minus for an option)."""
    import cam_bridge as CB
    args = ["--calib", str(tmp_path / "none.json"), "--wait-calib-s", "0", "--side-calib", str(tmp_path / "s.json")] + \
        LD.side_bias_args((-4.4, 3.7))
    with pytest.raises((FileNotFoundError, OSError)):      # past the arguments, at the (missing) calibration
        CB.main(args)
