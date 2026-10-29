"""Calibration room (2026-10-29): spin room, the per-heading roll caps sent as `set calib_dir_limits`, and keep-or-
calibrate from today's record.

    python3 -m pytest tools/fleet/tests/test_calib_room.py -q          # about 2 s
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, TOOLS):
    if p not in sys.path:
        sys.path.append(p)

import calib_room as CR      # noqa: E402
import engine_checks as EC   # noqa: E402

RECT = (0.0, 0.0, 122.0, 122.0)


def cap_at(cp, deg):
    return dict(cp)[deg]


def test_room_numbers_follow_from_the_body():
    assert abs(CR.SWEEP_R_CM - 6.72) < 0.01                      # half the 9.5 cm body's diagonal
    assert CR.ROOM_EDGE_CM == 12.0                               # 6.72 + 2 (camera) + 3 (margin), rounded up
    assert CR.ROOM_ROBOT_CM == 21.0                              # 2 x 6.72 + 2 x 2 + 3, rounded up


def test_spin_room_this_mornings_right_and_the_test_spots():
    # 07:43 29 Oct: Right kept out at 27 cm from the x = 122 edge (the old 30 cm rule): it has room now
    assert CR.spin_room((95.0, 61.0), RECT, {})[0]
    assert CR.spin_room((107.0, 61.0), RECT, {})[0]              # 15 cm
    assert CR.spin_room((102.0, 61.0), RECT, {"Left": (102.0, 33.0)})[0]   # 20 cm from an edge, 28 cm from a robot
    ok, why = CR.spin_room((112.0, 61.0), RECT, {})              # 10 cm: its corners' circle + margins do not fit
    assert not ok and "10 cm" in why
    ok, why = CR.spin_room((60.0, 60.0), RECT, {"Center": (60.0, 80.0)})
    assert not ok and "Center is 20 cm away" in why


def test_caps_cover_every_heading_with_eight_pairs():
    cp = CR.caps((61.0, 61.0), 0.0, RECT)
    assert [d for d, _ in cp] == [0, 45, 90, 135, 180, -135, -90, -45]
    assert all(cm == CR.ROLL_CM for _, cm in cp)                 # the middle of the board: the full roll every way
    cmd = CR.caps_cmd(cp)
    assert cmd.startswith("set calib_dir_limits 0 15.0 45 15.0") and len(cmd.split()) == 2 + 16


def test_facing_the_edge_15_cm_away_it_rolls_away_from_it():
    # x = 107, facing +x (towards the x = 122 edge): no room ahead, the full roll behind it
    cp = CR.caps((107.0, 61.0), 0.0, RECT)
    assert cap_at(cp, 0) < CR.ROLL_MIN_CM                        # straight ahead: skipped
    assert cap_at(cp, 180) == CR.ROLL_CM                         # away from the edge: the full 15 cm
    assert cap_at(cp, 90) < CR.ROLL_CM and cap_at(cp, -90) < CR.ROLL_CM   # along the edge: the window reaches it
    # whatever it picks inside a cap's window, the swept body stays 5 cm (margin + camera) inside the board
    for d, cm in cp:
        for a in range(-22, 23, 2):
            assert CR.roll_fits((107.0, 61.0), d + a, cm, RECT)


def test_caps_point_the_roll_at_open_space_in_world_terms():
    # the same spot, the robot facing the other way: now straight ahead is open
    cp = CR.caps((107.0, 61.0), 180.0, RECT)
    assert cap_at(cp, 0) == CR.ROLL_CM and cap_at(cp, 180) < CR.ROLL_MIN_CM
    d, cm = CR.best_world_heading((107.0, 61.0), 180.0, RECT)
    assert d == 0 and cm == CR.ROLL_CM
    assert CR.compass_words(180.0 + d) == "towards -x"


def test_unknown_heading_gets_the_roll_safe_all_round():
    for dist, rolls in ((15.0, False), (20.0, True), (27.0, True)):
        cp = CR.caps((122.0 - dist, 61.0), None, RECT)
        assert len({cm for _, cm in cp}) == 1                   # the same cap every way
        assert (cp[0][1] >= CR.ROLL_MIN_CM) == rolls, (dist, cp)
        for a in range(0, 360, 5):                              # safe whichever way it faces
            assert CR.roll_fits((122.0 - dist, 61.0), a, cp[0][1], RECT)
    assert "skip the roll" in CR.caps_words(CR.caps((107.0, 61.0), None, RECT), None, 180)


def test_a_robot_28_cm_away_caps_the_roll_towards_it():
    p, q = (61.0, 40.0), (61.0, 68.0)          # 28 cm apart, the robot faces +y (towards the other)
    cp = CR.caps(p, 90.0, RECT, fixed=[q])
    assert cap_at(cp, 0) < CR.ROLL_CM                            # towards the other robot: cut short
    assert cap_at(cp, 180) == CR.ROLL_CM                         # away from it: the full roll
    need = CR.SWEEP_R_CM + CR.MARGIN_CM + 2 * CR.CAM_ERR_CM
    for d, cm in cp:
        for a in range(-22, 23, 2):
            ang = math.radians(90.0 + d + a)
            tip = (p[0] + (cm + CR.BODY_HALF_CM) * math.cos(ang), p[1] + (cm + CR.BODY_HALF_CM) * math.sin(ang))
            assert math.dist(tip, q) >= need - 0.2


def test_two_robots_calibrating_together_keep_to_their_own_halves():
    p, q = (61.0, 40.0), (61.0, 68.0)
    a = CR.caps(p, 90.0, RECT, moving=[q])
    b = CR.caps(q, -90.0, RECT, moving=[p])
    # each one's furthest reach towards the other stays short of the line half way (54 cm)
    reach_a = 40.0 + cap_at(a, 0) + CR.BODY_HALF_CM
    reach_b = 68.0 - cap_at(b, 0) - CR.BODY_HALF_CM
    assert reach_a < 54.0 < reach_b


def test_heading_uncertainty_only_ever_shortens_a_cap():
    for h in (0.0, 30.0, 135.0):
        tight = CR.caps((104.0, 20.0), h, RECT, unc=5.0)
        loose = CR.caps((104.0, 20.0), h, RECT, unc=45.0)
        assert all(l <= t + 1e-9 for (_, t), (_, l) in zip(tight, loose))


def test_keep_or_calibrate(tmp_path):
    day = EC.DayState(str(tmp_path / "day.json"))
    how, words = CR.decide("Right", "C12D", day)
    assert how == "full" and "not calibrated yet today" in words            # a new day (no record)
    day.note_calibrated("Right", "C12D", t=1761738180.0)
    day.save()
    day = EC.DayState(str(tmp_path / "day.json"))
    how, words = CR.decide("Right", "C12D", day)
    assert how == "keep" and "boot C12D" in words and "no 65 s calibration" in words
    how, words = CR.decide("Right", "77A0", day)
    assert how == "full" and "restarted" in words and "C12D" in words and "77A0" in words
    assert CR.decide("Right", "C12D", day, force=True)[0] == "full"
    assert CR.decide("Right", None, day)[0] == "full"                      # boot not heard: never assume


def test_dry_runs_keep_their_own_record(tmp_path, monkeypatch):
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    d = CR.day_state(dry=True)
    assert os.path.join(str(tmp_path), "day-dry") in d.path
    assert CR.day_state(dry=False).path == EC.DayState().path


def test_open_spot_for_a_robot_without_room():
    """2026-10-29: a robot without room to spin is driven to the nearest open spot first (ladder.open_spot): 32 cm from
    every edge and 38 cm from the others' middles."""
    import ladder as LD
    L = LD.Ladder.__new__(LD.Ladder)
    L.rect = RECT
    L.st = {"robots": {"Right": {"x": 113.7, "y": 61.2}, "Left": {"x": 39.7, "y": 20.5},
                       "Center": {"x": 39.8, "y": 37.9}}}
    q = L.open_spot("Right", (113.7, 61.2))
    assert q == (90.0, 62.0)                                     # straight in from the x = 122 edge (dry run: 90, 62)
    q = L.open_spot("Left", (39.7, 20.5))
    assert CR.edge_dist(RECT, *q) >= L.OPEN_EDGE_CM
    assert all(math.dist(q, o) >= L.OPEN_ROBOT_CM for o in ((113.7, 61.2), (39.8, 37.9)))
    L.st["robots"] = {"Right": {"x": 61.0, "y": 61.0}}
    L.rect = (0.0, 0.0, 61.0, 61.0)                              # a small board with a robot in its middle: nowhere
    assert L.open_spot("Left", (10.0, 10.0)) is None
