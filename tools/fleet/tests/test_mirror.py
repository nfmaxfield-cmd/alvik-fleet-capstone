"""Mirroring by speed matching (Nick, 2026-10-29 20:06-20:08), and the same build's smaller changes: Left's stop limit
never under 9 cm, a second stop confirms only if the robot was moving at one of the two, a held robot keeps the 8 cm
gap. Simulator runs: docs/build-reports/e02-oct29-mirroring.md."""
import math
import os
import sys

import pytest

FLEET = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, FLEET)

import missions as MS        # noqa: E402
import planner as P          # noqa: E402
import run_plan as RP        # noqa: E402
import front_stop as FSTOP   # noqa: E402
import workspace as WS       # noqa: E402

SIZES = [(122, 61), (130, 70), (137, 76)]


def _monday(W, H):
    m = WS.blank_map((0, 0, W, H), [("Left", [20, 20, 90]), ("Center", [60, 20, 90]), ("Right", [100, 20, 90])])
    return MS.build(m, MS.monday_program(("Left", "Right")))


# ---------------------------------------------------------------------------- the mirrored plan

@pytest.mark.parametrize("size", SIZES)
def test_lanes_are_symmetric_and_about_12_cm_apart(size):
    W, H = size
    mm = _monday(W, H)
    lanes = MS.lanes_x(mm, mm["mission"]["settings"], mirror=True)[0]
    c = W / 2.0
    assert all(abs((a - c) + (b - c)) < 0.02 for a, b in zip(lanes, reversed(lanes)))
    gaps = [b - a for a, b in zip(lanes, lanes[1:])]
    inner = len(lanes) // 2 - 1
    assert abs(gaps[inner] - MS.mirror_centre_gap(mm)) < 0.02
    assert all(g <= MS.SETTINGS["mow_lane_cm"] + MS.MIRROR_LANE_STRETCH_CM + 1e-6 for k, g in enumerate(gaps) if k != inner)
    assert lanes[0] == pytest.approx(MS.SETTINGS["mow_inset_cm"])


@pytest.mark.parametrize("size", SIZES)
def test_every_mirrored_point_has_its_mirror_image(size):
    W, H = size
    mm = _monday(W, H)
    c = mm["mission"]["mirror"]["at_cm"]
    pts = {}
    for n in ("Left", "Right"):
        for v in mm["tasks"][n]["visit"]:
            if v.get("mirror"):
                assert v["cruise_cms"] == MS.SETTINGS["mirror_cruise_cms"]
                pts.setdefault(v["mirror"], {})[n] = v["goto"][:2]
    assert len(pts) >= 9
    for k, v in pts.items():
        assert set(v) == {"Left", "Right"}, k
        (xl, yl), (xr, yr) = v["Left"], v["Right"]
        assert abs(xl - (2 * c - xr)) < 0.05 and abs(yl - yr) < 0.05, k


def test_a_program_without_mirror_has_no_keys():
    m = WS.blank_map((0, 0, 122, 61), [("Left", [20, 20, 90]), ("Center", [60, 20, 90]), ("Right", [100, 20, 90])])
    prog = MS.monday_program(("Left", "Right"))
    prog["mirror"] = []           # explicitly not mirrored (a program without the key is Monday's saved default)
    mm = MS.build(m, prog)
    assert "mirror" not in mm["mission"]
    assert not any(v.get("mirror") or v.get("cruise_cms") for n in ("Left", "Right") for v in mm["tasks"][n]["visit"])


def test_plan_steps_carry_the_keys_and_the_slower_speed():
    mm = _monday(122, 61)
    plan = P.make_plan(mm, over={"separation_cm": 8.0}, t_limit_s=60)
    for n in ("Left", "Right"):
        steps = plan["robots"][n]["steps"]
        keyed = [st for st in steps if st.get("mirror")]
        assert len(keyed) >= 9
        legs = [st for st in steps if st.get("mirror_leg") and st["type"] == "drive" and st["phase"] != "confirm"]
        assert legs and all(abs(st["speed"] - 8.0) < 1e-6 for st in legs)
        chick = [st for st in steps if str(st.get("label", "")).startswith("chicken") and st["type"] == "drive"
                 and st["phase"] != "confirm"]
        assert chick and all(not st.get("mirror_leg") for st in chick)     # Chicken keeps its own speed (10)
    # a step turned back into a prim (re-plans keep what is under way) keeps its keys
    st = next(s for s in plan["robots"]["Left"]["steps"] if s.get("mirror"))
    p = P.step_to_prim(st)
    assert p["mirror"] == st["mirror"] and p["mirror_leg"] == st["mirror_leg"]


# ---------------------------------------------------------------------------- the mirror sync (run_plan)

class _Src:
    live = False


def _step(i, typ, frm, to, t0, t1, key=None, leg=None, speed=None, phase="travel"):
    st = {"i": i, "type": typ, "phase": phase, "from": list(frm), "to": list(to), "t_start": t0, "t_end": t1,
          "move_s": (t1 - t0 - 0.2) if typ == "drive" else (t1 - t0 - 0.3), "speed": speed, "after": []}
    if typ == "turn":
        st["turn_deg"] = 90.0
    if key:
        st["mirror"] = key
    if leg:
        st["mirror_leg"] = leg
    return st


def _coord(t, left_done=0, right_done=0, left_started_at=None):
    """Two mirrored robots, each: drive 16 cm to P1 (key k1), turn, drive 16 cm to P2 (key k2), all at 8 cm/s."""
    class A:
        no_mirror = False
        turn_pause = False
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.args, c.source = A(), _Src()
    c.t_run = c.plan_start = t - 10.0
    c.plan_no = 0
    c.m = {"floor": {"x_min": 0.0, "y_min": 0.0, "x_max": 122.0, "y_max": 61.0},
           "robot": {"width_cm": 9.5, "length_cm": 9.5, "front_offset_cm": 4.75},
           "mission": {"mirror": {"robots": ["Left", "Right"], "axis": "x", "at_cm": 61.0, "min_cms": 5.0,
                                  "max_cms": 10.0, "cruise_cms": 8.0, "tol_s": 0.5, "tol_cm": 3.0, "wait_max_s": 8.0}}}

    def steps(x0, sgn):
        return [_step(0, "drive", (x0, 17, 90), (x0, 33, 90), 0.0, 2.2, key="k1", leg="k1", speed=8.0),
                _step(1, "turn", (x0, 33, 90), (x0, 33, 90 - 90 * sgn), 2.2, 4.0, leg="k2"),
                _step(2, "drive", (x0, 33, 0), (x0 + 16 * sgn, 33, 0), 4.0, 6.2, key="k2", leg="k2", speed=8.0)]
    c.plan = {"robots": {"Left": {"steps": steps(17.0, 1)}, "Right": {"steps": steps(105.0, -1)}},
              "settings": {"turn_dps": 60.0}}
    c.names = ["Left", "Right"]
    c.bots = {n: RP.Bot(n) for n in c.names}
    for n, k in (("Left", left_done), ("Right", right_done)):
        b = c.bots[n]
        b.stat = [dict(status="done" if j < k else "unsent", t_done=t - 1.0, info="") for j in range(3)]
        b.pose = tuple(c.plan["robots"][n]["steps"][max(0, k - 1)]["to"]) if k else (17.0 if n == "Left" else 105.0,
                                                                                       17.0, 90.0)
    c.logged, c.said = [], []
    c.log = lambda ev, **kw: c.logged.append((ev, kw))
    c.say = c.said.append
    return c


def test_in_step_both_go_at_the_planned_speed():
    t = RP.now()
    c = _coord(t)
    ok, v = c.mirror_gate(c.bots["Left"], 0, t)
    assert ok and v == pytest.approx(8.0, abs=0.05)


def test_the_late_robot_speeds_up_and_the_leader_slows_down():
    t = RP.now()
    c = _coord(t)
    # Right already 1 s into its first drive (sent at 8 cm/s, 8 cm left): Left is about 1 s late
    rb = c.bots["Right"]
    rb.stat[0].update(status="started", t_started=t - 1.0, speed_sent=8.0)
    rb.pose = (105.0, 25.0, 90.0)
    ok, vL = c.mirror_gate(c.bots["Left"], 0, t)
    assert ok and 8.0 < vL <= 10.0
    # Right's next drive (after its turn) is slowed if Left is still behind
    c2 = _coord(t, left_done=0, right_done=2)
    ok, vR = c2.mirror_gate(c2.bots["Right"], 2, t)
    assert ok is False or (vR is not None and vR < 8.0)


def test_the_leader_waits_when_speed_cannot_close_the_gap():
    t = RP.now()
    c = _coord(t, left_done=0, right_done=1)      # Right already at P1; Left has 16 + turn + 16 to go,
    c.bots["Left"].stat[0].update(status="started", t_started=t, speed_sent=5.0)   # its first 16 at 5 cm/s
    ok, v = c.mirror_gate(c.bots["Right"], 1, t)  # Right's turn: the first step of its leg to P2
    assert ok is False
    assert any(ev == "mirror_hold" for ev, _ in c.logged)
    assert any("waits where it is" in s for s in c.said)
    # ... but not for ever
    c.mirror_state[(0, "Right", "k2")]["t0"] = t - 9.0
    ok, v = c.mirror_gate(c.bots["Right"], 1, t)
    assert ok is True and any(ev == "mirror_give_up" for ev, _ in c.logged)


def test_no_mirror_sends_as_planned():
    t = RP.now()
    c = _coord(t, left_done=0, right_done=1)
    c.args.no_mirror = True
    assert c.mirror_gate(c.bots["Right"], 1, t) == (True, None)


def test_a_held_partner_switches_mirroring_off():
    t = RP.now()
    c = _coord(t, left_done=0, right_done=1)
    c.bots["Left"].out = "silent"
    assert c.mirror_gate(c.bots["Right"], 1, t) == (True, None)


def test_matching_points_are_measured():
    t = RP.now()
    c = _coord(t)
    for n, x, dt in (("Left", 17.4, 0.0), ("Right", 105.0, 0.3)):
        b = c.bots[n]
        b.pose = (x, 33.0, 90.0)
        c.mirror_arrived(b, 0, t + dt)
    pts = c.mirror_points
    assert len(pts) == 1 and pts[0]["time_err_s"] == pytest.approx(0.3, abs=1e-3)
    assert pts[0]["pos_err_cm"] == pytest.approx(0.4, abs=1e-3) and pts[0]["ok"]
    sm = c.mirror_summary()
    assert sm["on"] and sm["all_points"]["time_s"]["max"] == pytest.approx(0.3, abs=1e-3)


# ---------------------------------------------------------------------------- the build's smaller changes

def test_left_never_below_9_cm():
    assert FSTOP.robot_min_cm("Left") == 9.0 and FSTOP.robot_min_cm("Right") == FSTOP.FLOOR_MIN_CM
    lim, rule = FSTOP.stop_limit(9.4, 10.0, lo_cm=FSTOP.robot_min_cm("Left"))     # floor 9.4 - 1 = 8.4 -> 9
    assert (lim, rule) == (9.0, "minimum")
    lim, rule = FSTOP.stop_limit(10.6, 10.0, lo_cm=FSTOP.robot_min_cm("Left"))
    assert lim == 9.6 and rule == "floor"
    lim, _ = FSTOP.stop_limit(9.4, 12.0, lo_cm=FSTOP.robot_min_cm("Right"))
    assert lim == 8.4


def test_a_second_stop_confirms_only_if_moving_at_one_of_them():
    clock = [0.0]
    u = FSTOP.Unconfirmed(lambda: clock[0])
    pts = [[50.0, 30.0], [52.0, 30.0], [51.0, 33.0]]
    u.add({"id": "a", "points": pts}, "Left", moving=False)
    assert u.near(pts, moving=False) == []              # standing both times: not confirmed
    assert u.near(pts, moving=True) == ["a"]            # moving now
    u.add({"id": "b", "points": [[80.0, 30.0]]}, "Left", moving=True)
    assert u.near([[80.5, 30.0]], moving=False) == ["b"]   # moving the first time
    assert u.near(pts) == ["a"]                         # unknown: as before


def test_was_moving_by_how_far_the_step_took_it():
    t = RP.now()
    c = _coord(t)
    b = c.bots["Left"]
    b.stat[0].update(status="started", pose_started=(17.0, 17.0, 90.0))
    assert c.was_moving(b, 0, (17.0, 18.0, 90.0)) is False
    assert c.was_moving(b, 0, (17.0, 21.0, 90.0)) is True
    assert c.was_moving(b, 1, (17.0, 33.0, 90.0)) is False    # a turn


def test_a_held_robot_keeps_the_8_cm_gap():
    t = RP.now()
    c = _coord(t)
    c.args.min_gap_cm = 8.0
    c.plan["settings"]["safety_margin_cm"] = 1.5
    c.plan["robots"]["Left"]["start"] = [17.0, 17.0, 90.0]
    b = c.bots["Left"]
    b.pose = (40.0, 30.0, 0.0)
    b.stat = [dict(status="done") for _ in range(3)]
    ob = c.out_obstacle(b)
    assert ob["gap_cm"] == pytest.approx(6.5)
    xs = [p[0] for p in ob["points"]]
    # the body reaches 4.75 cm either side; the outline 6.5 cm more (corners: a little more)
    assert max(xs) - 40.0 >= 4.75 + 6.5 - 0.5


def test_mondays_default_saved_before_mirroring_still_mirrors():
    """Mission Control's program.json saved before 2026-10-29 20:06 has no "mirror"; the unedited default mirrors."""
    m = WS.blank_map((0, 0, 122, 61), [("Left", [20, 20, 90]), ("Center", [60, 20, 90]), ("Right", [100, 20, 90])])
    prog = MS.monday_program(("Left", "Right"))
    prog.pop("mirror")
    assert MS.build(m, prog)["mission"]["mirror"]["robots"] == ["Left", "Right"]
    prog["blocks"][0]["to"] = 0.3                 # edited: not mirrored unless it says so
    assert "mirror" not in MS.build(m, prog)["mission"]
