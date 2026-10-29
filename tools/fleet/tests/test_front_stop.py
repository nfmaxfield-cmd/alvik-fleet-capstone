"""The distance-sensor stop, laptop side (2026-10-29 16:2x): 3 readings in a row, the floor check, unconfirmed
obstacles, a silent robot held (run_plan), per-camera lighting labels are in test_lighting.py.

    python3 -m pytest tools/fleet/tests/test_front_stop.py -q
"""
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
if FLEET not in sys.path:
    sys.path.insert(0, FLEET)

import front_stop as FS  # noqa: E402

FAR = 120.0


def floor_rows(cr, n=40, dips=()):
    """n new readings: the centre-right zone reads the floor at cr (alternating +-0.1 so each is new), the others far;
    dips: {index: value} for that zone."""
    rows = []
    for i in range(n):
        v = dict(dips).get(i, cr + (0.1 if i % 2 else -0.1))
        rows.append((FAR, FAR, FAR, v, FAR))
    return rows


def test_three_readings_is_the_default_everywhere():
    """run_plan sends obs_n 3 to every robot (the ladder, the DOE loop and Monday's mission all start run_plan)."""
    import run_plan as RP
    assert FS.STOP_READINGS == 3 and RP.OBSTACLE_READINGS == 3
    src = open(RP.__file__).read()
    assert 'add_argument("--obstacle-readings", type=int, default=OBSTACLE_READINGS' in src


def test_in_path_is_the_firmware_projection():
    # 11 deg zone at 11.4 cm: 2.2 cm to the side (in the 5.25 cm path), 11.19 cm ahead
    assert abs(FS.in_path((FAR, FAR, FAR, 11.4, FAR)) - 11.4 * math.cos(math.radians(11))) < 1e-6
    # 22 deg zone at 15 cm: 5.6 cm to the side, outside the path; at 13 cm, 4.9 cm: inside
    assert FS.in_path((15.0, 0, 0, 0, 0)) == 0.0
    assert abs(FS.in_path((13.0, 0, 0, 0, 0)) - 13.0 * math.cos(math.radians(22))) < 1e-6


def test_floor_level_ignores_dips_shorter_than_three_readings():
    base = FS.floor_level(floor_rows(11.4))
    assert abs(base["floor_cm"] - 11.5 * math.cos(math.radians(11))) < 0.01     # every 3 in a row hold an 11.5
    two = FS.floor_level(floor_rows(11.4, dips={10: 9.0, 11: 9.05}))
    assert two["floor_cm"] > 11.0             # two low readings (9 cm) would not stop it with 3: they do not set it
    three = FS.floor_level(floor_rows(11.4, dips={10: 9.0, 11: 9.05, 12: 9.1}))
    assert abs(three["floor_cm"] - 9.1 * math.cos(math.radians(11))) < 0.01


def test_floor_level_held_repeats_are_one_reading_and_far_floor_is_none():
    rows = []
    for v in floor_rows(11.4, n=20):
        rows += [v, v, v]                                          # the sensor holds its value between readings
    fl = FS.floor_level(rows)
    assert fl["n"] == 20 and fl["floor_cm"] is not None
    assert FS.floor_level([(FAR,) * 5, (FAR - 1,) * 5] * 10)["floor_cm"] is None
    assert "only" in FS.floor_level(floor_rows(11.4, n=5))["why"]


def test_stop_limit_one_rule_the_lowest_wins_never_under_8():
    assert FS.stop_limit(16.0) == (12.0, "cap")                    # Right, floor far below the stop: the usual 12
    assert FS.stop_limit(16.0, ceiling_cm=10.0) == (10.0, "ceiling")    # Left at 07:43 (floor 15-16): its own 10
    assert FS.stop_limit(10.6, ceiling_cm=10.0) == (9.6, "floor")       # Left with its floor at 10.6: 9.6
    assert FS.stop_limit(11.8) == (10.8, "floor")                  # Center, 28 Oct 19:17
    assert FS.stop_limit(8.5) == (8.0, "minimum")                  # never under 8
    assert FS.stop_limit(None, ceiling_cm=10.0) == (10.0, "ceiling")    # no floor seen: as decided for it
    assert FS.stop_limit(None) == (12.0, "cap")
    w = FS.limit_words("Left", {"floor_cm": 8.5}, 8.0, "minimum")
    assert "lowest allowed" in w


def test_unconfirmed_registry_expires_confirms_and_counts():
    t = [100.0]
    u = FS.Unconfirmed(lambda: t[0], expire_s=15.0)
    sq = [[0, 0], [4, 0], [4, 4], [0, 4]]
    u.add({"id": "a", "points": sq}, "Left")
    assert u.near([[6, 2]]) == ["a"] and u.near([[20, 20]]) == []
    t[0] = 114.9
    assert u.expired() == []
    t[0] = 115.0
    assert u.expired() == ["a"]
    u.drop("a", "expired")
    assert u.items == {} and u.near([[6, 2]]) == ["a"]        # a second stop there still confirms it
    u.add({"id": "b", "points": sq}, "Right")
    u.confirm("b", "a second stop")
    assert u.summary() == {"added": 2, "expired": 1, "confirmed": 1}


class FakeCoord:
    """Just enough of run_plan.Coordinator for its unconfirmed-obstacle methods."""

    def __init__(self, unconfirmed_s=15.0):
        import run_plan as RP
        self.RP = RP
        for k in ("unconf", "mark_unconfirmed", "drop_obstacles", "unconfirmed_watch", "no_plan_drop",
                  "live_obstacles"):
            setattr(self, k, types.MethodType(getattr(RP.Coordinator, k), self))
        self.args = types.SimpleNamespace(unconfirmed_s=unconfirmed_s)
        self.m = {"obstacles": [], "floor": {"x_min": 0, "y_min": 0, "x_max": 122, "y_max": 61}, "robots": [],
                  "boards": []}
        self.plan = {}
        self.planning = None
        self.improve_at = None
        self.abort = None
        self.said, self.logged, self.replans = [], [], []
        self.sensor_stops = []

    def say(self, m):
        self.said.append(m)

    def log(self, ev, **kw):
        self.logged.append((ev, kw))

    def handle_obstacle(self, ob, cause=None, **kw):
        self.replans.append((ob, cause))


def _ob(i, x=50.0, y=30.0):
    return {"id": "seen-by-Left-%d" % i, "shape": "polygon", "source": "distance_sensor",
            "points": [[x, y], [x + 4, y], [x + 4, y + 4], [x, y + 4]]}


def test_sensor_only_obstacle_is_unconfirmed_and_expires(monkeypatch):
    import run_plan as RP
    t = [10.0]
    monkeypatch.setattr(RP, "now", lambda: t[0])
    c = FakeCoord()
    b = types.SimpleNamespace(name="Left")
    ob = _ob(1)
    c.sensor_stops.append({"camera": None})
    c.mark_unconfirmed(b, ob)
    assert ob["unconfirmed"] and "seen-by-Left-1" in c.unconf().items
    c.m["obstacles"].append(dict(ob))
    assert c.live_obstacles()[0]["unconfirmed"] and c.live_obstacles()[0]["expires_in_s"] == 15.0
    t[0] = 25.5
    c.unconfirmed_watch()
    assert c.m["obstacles"] == [] and c.unconf().items == {}
    assert any("taken off the map" in s for s in c.said)


def test_a_second_stop_or_the_camera_confirms(monkeypatch):
    import run_plan as RP
    monkeypatch.setattr(RP, "now", lambda: 10.0)
    c = FakeCoord()
    b = types.SimpleNamespace(name="Left")
    c.sensor_stops.append({"camera": None})
    c.mark_unconfirmed(b, _ob(1))
    second = _ob(2, x=53.0)                      # 3 cm on: the same thing
    c.mark_unconfirmed(b, second)
    assert "unconfirmed" not in second and second["confirmed"] == "a second stop"
    assert c.unconf().items == {}
    third = _ob(3, x=90.0)
    c.sensor_stops.append({"camera": "the camera's obstacle brick-1"})
    c.mark_unconfirmed(b, third)
    assert third["confirmed"].startswith("the camera") and c.unconf().items == {}


def test_no_plan_drops_unconfirmed_and_plans_again(monkeypatch):
    import run_plan as RP
    monkeypatch.setattr(RP, "now", lambda: 10.0)
    c = FakeCoord()
    b = types.SimpleNamespace(name="Left")
    c.sensor_stops.append({"camera": None})
    old = _ob(1, x=20.0)
    c.mark_unconfirmed(b, old)
    c.m["obstacles"].append(dict(old))
    new = _ob(2, x=80.0)
    c.mark_unconfirmed(b, new)
    assert c.no_plan_drop(new, "Left", "park spot cannot be reached") is True
    assert c.m["obstacles"] == [] and c.replans == [(None, "Left")] and c.no_plan_drops == 1
    # nothing unconfirmed left: the old rule (stop everyone) applies
    assert c.no_plan_drop(None, "Left", "no route") is False
    # --unconfirmed-s 0: every sensor obstacle stays, as before
    c0 = FakeCoord(unconfirmed_s=0)
    ob = _ob(5)
    c0.mark_unconfirmed(b, ob)
    assert "unconfirmed" not in ob and c0.no_plan_drop(ob, "Left", "x") is False


def test_floor_check_sets_each_robots_limit_and_reads_it_back(monkeypatch, tmp_path):
    import run_plan as RP
    import engine_checks as EC
    monkeypatch.setattr(FS, "FLOOR_SAMPLE_S", 0.05)
    monkeypatch.setattr(EC, "ROBOT_SETTINGS", {"Left": {"follow_look_cm": 10}}, raising=False)
    cols = ["run", "ms", "step", "v_cms", "w_dps", "dL_cm", "dCL_cm", "dC_cm", "dCR_cm", "dR_cm"]
    feeds = {"Left": floor_rows(15.5), "Right": floor_rows(11.0), "Center": floor_rows(8.6)}
    bots = {n: types.SimpleNamespace(name=n, params={}, pose=None, last_cam=None) for n in feeds}
    on_robot = {n: {"follow_look_cm": 10.0 if n == "Left" else 12.0} for n in feeds}
    sent = []

    class Link:
        session = types.SimpleNamespace(headers={"RAW": cols})

        def send_name(self, n, c):
            sent.append((n, c))
            if c.startswith("set follow_look_cm"):
                on_robot[n]["follow_look_cm"] = float(c.split()[-1])
            if c.startswith("get follow_look_cm"):
                bots[n].params["follow_look_cm"] = on_robot[n]["follow_look_cm"]

    coord = types.SimpleNamespace(bots=bots, plan={"robots": {n: {"start": [20 + 40 * i, 20, 90]}
                                                              for i, n in enumerate(feeds)}},
                                  m={"obstacles": []}, sent_settings={"Left": {"follow_look_cm": 10.0}},
                                  outdir=str(tmp_path), start_checks={n: {} for n in feeds}, logged=[])
    coord.log = lambda ev, **kw: coord.logged.append((ev, kw))
    coord.say = lambda m: None
    coord.link = Link()
    coord.floor_feed = types.MethodType(RP.Coordinator.floor_feed, coord)

    def drain():
        if getattr(coord, "floor_raw", None) is None:
            return
        for n, rows in feeds.items():
            for z in rows:
                coord.floor_feed(bots[n], ["1-1", "0", "0", "0.0", "0.0"] + ["%.2f" % v for v in z])
            feeds[n] = []
    coord.drain = drain
    out = RP.floor_check(coord, coord.link, types.SimpleNamespace(floor_check="on", floor_margin_cm=1.0),
                         list(feeds))
    assert out["Left"]["limit_cm"] == 10.0 and out["Left"]["rule"] == "ceiling"      # floor 15: its own 10 stays
    assert out["Right"]["rule"] == "floor" and abs(out["Right"]["limit_cm"] - 9.9) < 0.11    # floor 10.9 -> 9.9
    assert out["Center"]["limit_cm"] == 8.0 and out["Center"]["rule"] == "minimum"
    assert ("Left", "set follow_look_cm 10.0") not in sent                           # already 10: not sent again
    assert any(c.startswith("set follow_look_cm 9.") for n, c in sent if n == "Right")
    assert bots["Right"].stop_cm == out["Right"]["limit_cm"]
    assert os.path.exists(os.path.join(str(tmp_path), "floor_check.json"))
    assert coord.start_checks["Center"]["stop_limit"]["rule"] == "minimum"


def test_floor_check_skips_a_robot_with_another_in_front():
    import run_plan as RP
    bots = {"Left": types.SimpleNamespace(name="Left", pose=(20.0, 20.0, 0.0), last_cam=None),
            "Right": types.SimpleNamespace(name="Right", pose=(40.0, 22.0, 180.0), last_cam=None)}
    coord = types.SimpleNamespace(bots=bots, plan={"robots": {}}, m={"obstacles": []})
    nm, d = RP.floor_blocked(coord, "Left")
    assert nm == "Right" and abs(d - math.hypot(20, 2)) < 1e-6
    bots["Right"].pose = (20.0, 45.0, 0.0)                         # beside it, 25 cm away: not in front
    assert RP.floor_blocked(coord, "Left") is None


def test_a_dropped_obstacle_met_again_is_confirmed(monkeypatch):
    """A real brick no camera sees: the first stop's obstacle is dropped (no plan), the robot meets the brick again:
    the second stop confirms it, and it stays (a later no plan stops everyone, as before)."""
    import run_plan as RP
    monkeypatch.setattr(RP, "now", lambda: 10.0)
    c = FakeCoord()
    b = types.SimpleNamespace(name="Left")
    c.sensor_stops.append({"camera": None})
    first = _ob(1)
    c.mark_unconfirmed(b, first)
    assert c.no_plan_drop(first, "Left", "no route") is True
    again = _ob(2, x=51.0)
    c.mark_unconfirmed(b, again)
    assert again.get("confirmed") == "a second stop" and "unconfirmed" not in again
    assert [o["id"] for o in c.m["obstacles"]] == ["seen-by-Left-1"]          # the dropped one is back, confirmed
    c.m["obstacles"].append(dict(again))
    assert c.no_plan_drop(None, "Left", "no route") is False
