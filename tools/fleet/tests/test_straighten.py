"""Start straightening, robots facing an edge, driving blind, the bridge's "moved" rule, Left's stop limit
(build e02-oct29-start-straighten-blind-strip.md, 2026-10-29)."""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
sys.path.insert(0, FLEET)

import straighten as ST     # noqa: E402
import run_plan as RP       # noqa: E402
import engine_checks as EC  # noqa: E402

RECT = (0.0, 0.0, 122.0, 61.0)


# ---------------------------------------------------------------------------- 1. the correction's steps

def test_on_its_spot_no_correction():
    r = ST.plan_one("Left", (17.4, 16.8, 92.0), (17, 17, 90), RECT)
    assert not r["need"] and r["steps"] == []


@pytest.mark.parametrize("h", [110.0, 70.0, 105.0, -90.0, 270.0 - 360.0, 180.0])
def test_any_heading_off_is_one_turn_on_the_spot(h):
    r = ST.plan_one("Left", (17, 17, h), (17, 17, 90), RECT)
    assert [s["type"] for s in r["steps"]] == ["turn"]
    assert r["steps"][0]["to"][2] == 90 and r["steps"][0]["to"][:2] == [17, 17]


def test_backwards_says_so_in_plain_words():
    r = ST.plan_one("Left", (17, 17, -90), (17, 17, 90), RECT)
    assert "backwards" in r["words"][-1] and "the y = 61 side" in r["words"][-1]
    assert ST.done_words("Left", r, RECT, 90) == "Left was 180 deg off (backwards): turned to face the y = 61 side"


def test_off_its_spot_turn_drive_forwards_turn():
    r = ST.plan_one("Left", (25, 17, 90), (17, 17, 90), RECT)
    assert [s["type"] for s in r["steps"]] == ["turn", "drive", "turn"]
    d = r["steps"][1]
    assert not d["reverse"] and d["to"][:2] == [17, 17] and d["speed"] == ST.DRIVE_CMS
    assert RP.step_command(5, d).startswith("fs 5 D 25.00 17.00 17.00 17.00 180.00 7.50 0 ")


def test_side_camera_alone_allows_5_cm():
    assert not ST.plan_one("Left", (21.5, 17, 90), (17, 17, 90), RECT, side_only=True)["need"]
    assert ST.plan_one("Left", (21.5, 17, 90), (17, 17, 90), RECT)["need"]
    assert ST.plan_one("Left", (22.5, 17, 90), (17, 17, 90), RECT, side_only=True)["need"]


def test_facing_an_edge_under_12_cm_turns_first_never_backs():
    r = ST.plan_one("Left", (17, 9, -90), (17, 17, 90), RECT)       # nose 4 cm from the y = 0 edge
    assert r["steps"][0]["type"] == "turn" and r["ahead_cm"] < ST.EDGE_AHEAD_CM
    assert "turns on the spot first (no backing)" in " ".join(r["words"])
    assert all(not s.get("reverse") for s in r["steps"])


def test_too_near_an_edge_to_spin_stops_with_words():
    with pytest.raises(ST.Refuse, match="cannot turn"):
        ST.plan_one("Left", (17, 6.0, 90), (17, 17, 90), RECT)


def test_two_robots_whose_ways_cross_go_one_after_the_other():
    p = ST.plan_all({"Left": (50, 30, 90), "Right": (30, 45, 0)}, {"Left": (17, 17, 90), "Right": (80, 45, 90)}, RECT)
    assert p["after"] == {"Right": "Left"}
    assert any("waits until Left" in w for w in p["words"])


def test_recheck_allowance():
    assert ST.recheck("Left", (17.5, 17, 95), (17, 17, 90))[0]
    assert not ST.recheck("Left", (17.5, 17, 100), (17, 17, 90))[0]
    assert not ST.recheck("Left", (21, 17, 90), (17, 17, 90))[0]
    assert ST.recheck("Left", (21, 17, 90), (17, 17, 90), side_only=True)[0]


# ---------------------------------------------------------------------------- 2. driving blind

class _Src:
    live = True


def _coord():
    class A:
        pass
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.args, c.source, c.t_run = A(), _Src(), RP.now() - 5
    c.m = {"floor": {"x_min": 0.0, "y_min": 0.0, "x_max": 122.0, "y_max": 61.0}}
    b = RP.Bot("Left")
    c.bots, c.names = {"Left": b}, ["Left"]
    c.logged, c.said, c.sent = [], [], []
    c.log = lambda ev, **k: c.logged.append((ev, k))
    c.say = c.said.append
    c.send = lambda n, t: c.sent.append((n, t))
    c.halt_all = lambda: c.sent.append(("all", "halt"))
    c.abort = None
    return c, b


def _drive(c, b, xs, y, t0, seen_until):
    """The robot drives along y at x in xs, 0.1 s apart; the camera sees it until seen_until."""
    for k, x in enumerate(xs):
        t = t0 + 0.1 * k
        b.pose, b.pose_t = (x, y, 0.0), t
        b.pose_hist = getattr(b, "pose_hist", []) + [(t, x, y, 0.0)]
        if t <= seen_until:
            b.cam_t = t
            b.cam_hist = getattr(b, "cam_hist", []) + [(t, t, x, y, "both")]
        RP.now_saved = RP.now
        RP.now = lambda tt=t: tt
        try:
            c.blind_watch()
        finally:
            RP.now = RP.now_saved
        if c.abort:
            return k
    return None


def test_blind_drive_is_allowed_up_to_the_cap_then_stops_everyone():
    c, b = _coord()
    t0 = RP.now()
    k = _drive(c, b, [30 + 1.0 * i for i in range(40)], 30.0, t0, t0 + 0.2)   # 10 cm/s, blind after 0.2 s
    assert k is not None and c.abort and "no camera seeing it" in c.abort
    blind_cm = 30 + k - 32
    assert c.BLIND_MAX_CM <= blind_cm + 1.5 <= c.BLIND_MAX_CM + 3
    assert ("all", "halt") in c.sent


def test_blind_near_an_edge_stops_sooner():
    c, b = _coord()
    t0 = RP.now()
    k = _drive(c, b, [30 + 1.0 * i for i in range(40)], 61.0 - 4.75 - 8.0, t0, t0 + 0.2)   # body 8 cm from y = 61
    assert c.abort and "may be" in c.abort and k < 2 + c.BLIND_MAX_CM - 5   # well before the cap


def test_seen_again_after_blind_its_pose_is_reset_when_off():
    c, b = _coord()
    t0 = RP.now()
    assert _drive(c, b, [30 + 1.0 * i for i in range(12)], 30.0, t0, t0 + 0.2) is None
    assert b.blind is not None and b.blind["dist"] > 5
    # the camera sees it again 3 cm off its own pose
    t = t0 + 1.25
    b.pose, b.pose_t = (42.5, 30.0, 0.0), t
    b.pose_hist.append((t, 42.5, 30.0, 0.0))
    b.cam_t = t
    b.cam_hist.append((t, t, 45.5, 30.0, "both"))
    RP.now_saved = RP.now
    RP.now = lambda: t
    try:
        c.blind_watch()
    finally:
        RP.now = RP.now_saved
    assert b.blind is None and not c.abort
    assert any(x[1].startswith("fpose 45.50 30.00") for x in c.sent)
    ev = [k for e, k in c.logged if e == "blind_end"][-1]
    assert ev["err_cm"] == pytest.approx(3.0, abs=0.01) and ev["reset"] == [45.5, 30.0]


# ---------------------------------------------------------------------------- 3. Left's stop limit

def test_left_alone_gets_its_10_cm_stop_limit():
    assert EC.robot_settings_cmds("Left") == ["set follow_look_cm 10"]
    assert EC.robot_settings_cmds("Right") == [] and EC.robot_settings_cmds("Center") == []
    assert EC.parse_setting("set follow_look_cm 10") == ("follow_look_cm", 10.0)


# ---------------------------------------------------------------------------- 4. the bridge: moved = own motion

def test_bridge_counts_moved_only_by_its_own_motion():
    import cam_bridge as CB

    class Br:
        pass
    br = Br()
    br.starts = {"Left": (17.0, 17.0, 90.0)}
    br.has_pose = {"Left": False}
    br.moved = {"Left": False}
    br.events = []
    br.event = lambda kind, **k: br.events.append(kind)

    class T:
        def own_pose(self, *a):
            pass
    br.trk = T()
    on_pose = CB.Bridge.on_pose
    on_pose(br, {"robot": "Left", "x": 20.0, "y": 17.0, "th": 90.0, "t": 1.0, "state": "idle"})
    assert br.has_pose["Left"] and not br.moved["Left"]          # 3 cm off its spot, standing: not moved
    on_pose(br, {"robot": "Left", "x": 20.4, "y": 17.0, "th": 90.0, "t": 1.1, "state": "idle"})
    assert not br.moved["Left"]
    on_pose(br, {"robot": "Left", "x": 20.4, "y": 17.0, "th": 90.0, "t": 1.2, "state": "turn"})
    assert br.moved["Left"]                                       # a step running: moved


# ---------------------------------------------------------------------------- 5. ladder / DOE loop: turn from the edge

def test_ladder_turns_away_from_an_edge_before_its_crawl():
    import ladder as LD
    lad = LD.Ladder.__new__(LD.Ladder)
    lad.rect = RECT
    lad.st = {"robots": {"Right": {"x": 110.0, "y": 9.0, "facing": -90.0, "facing_sure": True,
                                   "facing_at": [110.0, 9.0]}}}
    sent = []

    class Bench:
        def leg(self, n, text, timeout):
            sent.append((n, text))
            return ("done", None, None, None, "")
    lad.bench = Bench()
    lad.log_event = lambda *a, **k: None
    h = lad.turn_from_edge("Right", (110.0, 9.0))
    assert sent and sent[0][1].startswith("turn ")
    turn = float(sent[0][1].split()[1])
    assert abs(LD.wrap180(-90.0 + turn - h)) < 0.06
    assert ST.ahead_to_edge(RECT, 110.0, 9.0, h) - ST.BODY_HALF_CM >= ST.EDGE_AHEAD_CM
    # measured elsewhere, or not sure: nothing
    sent.clear()
    lad.st["robots"]["Right"].update(facing=-90.0, facing_sure=True, facing_at=[100.0, 9.0])
    assert lad.turn_from_edge("Right", (110.0, 9.0)) is None and not sent
    # facing open space: nothing
    lad.st["robots"]["Right"].update(facing=90.0, facing_sure=True, facing_at=[110.0, 9.0])
    assert lad.turn_from_edge("Right", (110.0, 9.0)) is None and not sent


def test_straighten_selftest():
    ST._selftest()
