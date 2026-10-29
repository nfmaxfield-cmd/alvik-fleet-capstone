"""The remote ladder's pure parts (2026-10-27): layouts round parked robots, the camera's line-fit heading, the maps
it hands the planner, and the simulator's lights / start spots / turn error.

    python3 -m pytest tools/fleet/tests/test_ladder.py -q          # about 10 s
The ladder's dry runs themselves (each step against the simulated board, ~25 min): see
docs/session-plans/2026-10-27.md, "Remote ladder".
"""
import math
import time
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, os.path.join(TOOLS, "sim"), TOOLS):
    if p not in sys.path:
        sys.path.append(p)

import ladder as LD         # noqa: E402
import fleetmap as FM       # noqa: E402
import planner as P         # noqa: E402

RECT = (0.0, 0.0, 122.0, 122.0)


def test_line_fit_heading_direction_and_sign():
    pts = [(i * 0.1, 50 + i * 0.5 * math.cos(math.radians(30)), 40 + i * 0.5 * math.sin(math.radians(30)))
           for i in range(30)]
    h = LD.line_fit_heading(pts, pts[0][1:], pts[-1][1:])
    assert h == pytest.approx(30.0, abs=0.01)
    h2 = LD.line_fit_heading(pts, pts[-1][1:], pts[0][1:])
    assert LD.wrap180(h2 - 210.0) == pytest.approx(0.0, abs=0.01)


def test_square_fits_empty_board_like_phase0():
    lays = LD.square_layouts(RECT, [], (35, 35))
    assert lays and lays[0]["side"] == 62.0 and lays[0]["margin"] == 30.0
    c = lays[0]["corners"][lays[0]["start"]]
    assert math.dist(c, (30, 30)) < 1e-6         # the corner nearest the robot


def test_square_keeps_parked_robots_clear():
    parked = [(40.0, 40.0), (85.0, 80.0)]
    lays = LD.square_layouts(RECT, parked, (60, 60))
    assert lays and lays[0]["side"] == 32.0       # only the smallest fits round two parked robots
    assert lays
    lay = lays[0]
    c = lay["corners"]
    segs = [(c[k], c[(k + 1) % 4]) for k in range(4)]
    assert LD.segs_clear(segs, parked, 0) >= LD.LINE_CLEAR_CM
    assert all(LD.edge_dist(RECT, *q) >= lay["margin"] - 1e-6 for q in c)


def test_square_none_when_boxed_in():
    parked = [(x, y) for x in range(30, 100, 15) for y in range(30, 100, 15)]
    assert LD.square_layouts(RECT, parked, (60, 60)) == []


def test_cross_layout_and_map_plan():
    parked = [(90.0, 40.0)]
    lays = LD.cross_layouts(RECT, parked, [("Left", (50, 80)), ("Right", (40, 45))])
    assert lays
    lay = lays[0]
    segs = [(lay["A"], lay["D"]), (lay["B"], lay["C"])]
    assert LD.segs_clear(segs, parked, 0) >= LD.LINE_CLEAR_CM
    m = LD.cross_map(RECT, lay, {"Center": parked[0]}, LD.SLOW_CMS, LD.WIDE_SEP_CM)
    plan = P.make_plan(m)
    assert set(plan["robots"]) == {"Left", "Right"}
    assert FM.normalize(m)["obstacles"][0]["radius"] == LD.OBSTACLE_R_CM


def test_square_map_plans_round_parked_robot():
    parked = {"Center": (61.0, 61.0)}
    lay = LD.square_layouts(RECT, list(parked.values()), (35, 35))[0]
    m = LD.square_map(RECT, lay, "Left", parked, LD.SLOW_CMS)
    plan = P.make_plan(m)
    steps = plan["robots"]["Left"]["steps"]
    assert any(s["type"] == "drive" for s in steps)
    assert all(abs(s.get("speed", LD.SLOW_CMS)) <= LD.SLOW_CMS + 1e-6 for s in steps if s["type"] == "drive")


def test_drive_to_map_parks_at_target_heading():
    m = LD.base_map(RECT, "drive", {"Center": (90.0, 40.0)}, LD.SLOW_CMS)
    m["robots"] = [{"name": "Left", "start": [50, 85, -96], "park": [30, 30, 90]}]
    m["tasks"] = {"Left": {"visit": [], "park": True}}
    plan = P.make_plan(m)
    last = plan["robots"]["Left"]["steps"][-1]
    assert last["to"][:2] == pytest.approx([30, 30], abs=0.5)
    assert LD.wrap180(last["to"][2] - 90) == pytest.approx(0, abs=1.0)


def test_holding_spot_makes_room_for_the_full_square():
    lay = LD.square_layouts(RECT, [], (35, 35))[0]
    segs = LD.square_segs(lay)
    hs = LD.holding_spots(RECT, segs, {"Right": (40.0, 44.0)}, [(89.0, 39.0), (52.0, 85.0)])
    assert hs is not None
    q = hs["Right"]
    assert LD.segs_clear(segs, [q], 0) >= LD.LINE_CLEAR_CM and LD.edge_dist(RECT, *q) >= LD.EDGE_REFUSE_CM


def test_sim_lights_starts_and_turn_error(tmp_path):
    import json
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.2", 5006))
        s.close()
    except OSError:
        pytest.skip("a simulated board is running on these ports")
    import fleet_sim as FS
    mp = tmp_path / "m.json"
    m = LD.base_map(RECT, "t", {}, 10.0)
    m["robots"] = [{"name": "Left", "start": [30, 30, 0]}, {"name": "Center", "start": [90, 90, 0]}]
    mp.write_text(json.dumps(m))
    sp = tmp_path / "s.json"
    sp.write_text(json.dumps({"Left": [50, 60, 45]}))
    args = FS.main.__code__  # noqa: F841 (parser lives in main; build the namespace by hand)
    import argparse
    a = argparse.Namespace(map=str(mp), plan=None, out=str(tmp_path), config=None, seed=1, laptop="127.0.0.1",
                           fixes_to="127.0.0.1:59999", fix_hz=0, fix_sigma=0.3, fix_sigma_deg=1, fix_delay=0.1,
                           start_noise=0.0, uncalibrated=False, tof_robots=1, brick_camera=False, cam_gap=[],
                           true_sides=None, true_objects=[], brick=None, brick_ahead_s=3, cam_delay=0.5, wifi_drop=[],
                           serve_live=0, live_fps=15, live_px_per_cm=2.34, live_calib_out=None, live_brick=False,
                           max_s=1, quiet=True, starts=str(sp), turn_err=["Center=18"], persist=True)
    sim = FS.FleetSim(a)
    try:
        L, C = sim.robots["Left"], sim.robots["Center"]
        assert (L.x, L.y) == pytest.approx((50, 60)) and math.degrees(L.th) == pytest.approx(45)
        assert (1 + C.gyro_scale) * 1.18 == pytest.approx(1.0)
        L.handle_command("led 1 1 1 2000", False)
        assert L.lights()[0] == (1.0, 1.0, 1.0)
    finally:
        for rb in sim.robots.values():
            rb.radio.sock.close()


def test_crawl_rechecks_once_before_failing(monkeypatch):
    """2026-10-28: a first crawl that reads 15 cm for 10 (the start spot's floor was filled in: a piece of the robot,
    off its middle) is crawled again; the second reading (10.4 cm) counts. Two bad readings raise, naming the
    camera(s) that saw it."""
    import types
    monkeypatch.setattr(LD.time, "sleep", lambda s: None)
    L = LD.Ladder.__new__(LD.Ladder)
    L.rect = (0.0, 0.0, 122.0, 122.0)
    L.others_of = lambda n: {}
    sent = []
    L.bench = types.SimpleNamespace(leg=lambda n, t, to: ("done", 0, 0, 0.0, ""), send=lambda n, c: sent.append(c))

    def eye_for(stills):
        it = iter(stills)
        return types.SimpleNamespace(
            still=lambda n: next(it),
            trk=types.SimpleNamespace(history=lambda n, a, b: [(i, 50.0 + i, 50.0) for i in range(8)]),
            sources=lambda n, a, b: {"side": 30, "both": 2})
    L.eye = eye_for([(65.0, 50.0), (50.0, 50.0), (60.4, 50.0), (50.0, 50.0)])
    h, d = L.crawl("Left", (50.0, 50.0), None)
    assert d["crawl_cm"] == pytest.approx(10.4) and d["evidence"] == "side" and abs(h) < 1e-6
    L.eye = eye_for([(65.0, 50.0), (50.0, 50.0), (65.0, 50.0), (50.0, 50.0)])
    with pytest.raises(LD.LadderStop) as e:
        L.crawl("Left", (50.0, 50.0), None)
    assert e.value.evidence == "side" and "15.0 cm" in str(e.value)


def test_misread_heading_pattern():
    rows = lambda es: [dict(error_deg=e) for e in es]
    assert LD.misread_heading(rows([3.9, -1.8, 7.9, -5.6]), 4.3) == 3          # Center, 28 Oct 16:38
    assert LD.misread_heading(rows([-10.6, 6.0, -0.0, 2.1]), -2.6) == 1        # Right, 27 Oct
    assert LD.misread_heading(rows([7.9, 7.5, 6.8, 8.0]), 30.2) == 0           # really turns wrong
    assert LD.misread_heading(rows([1.0, -0.5, 0.3, 9.0]), 9.8) == 0           # total over 8: a real FAIL


def _drive_session(tmp, travel_deg, plan_deg=45.0, cm=12.0):
    """One straight drive step planned along plan_deg; the camera sees the robot travel along travel_deg."""
    import json
    sess = tmp / "s"
    os.makedirs(sess / "bridge")
    x0, y0 = 50.0, 50.0
    x1, y1 = x0 + cm * math.cos(math.radians(plan_deg)), y0 + cm * math.sin(math.radians(plan_deg))
    with open(sess / "coordinator.jsonl", "w") as f:
        f.write(json.dumps(dict(t=100.0, event="step_sent", robot="C", id=1, text="fs 1 D %.2f %.2f %.2f %.2f %.2f 7.5 "
                                                                                   "0 -" % (x0, y0, x1, y1, plan_deg))) + "\n")
        f.write(json.dumps(dict(t=100.1, event="step_report", robot="C", id=1, state="started", th=plan_deg)) + "\n")
        f.write(json.dumps(dict(t=102.0, event="step_report", robot="C", id=1, state="done", th=plan_deg)) + "\n")
    with open(sess / "bridge" / "track.csv", "w") as f:
        f.write("t,robot,x,y,status,sent_th,why,src\n")
        for k in range(20):
            d = cm * k / 19.0
            f.write("%.3f,C,%.2f,%.2f,seen,,,both\n" % (100.1 + 0.1 * k, x0 + d * math.cos(math.radians(travel_deg)),
                                                        y0 + d * math.sin(math.radians(travel_deg))))
    with open(sess / "follow.csv", "w") as f:
        f.write("robot,t_recv,th_deg\nC,101.0,%.1f\n" % plan_deg)
    return str(sess)


def test_end_poses_heading_is_the_way_the_camera_saw_it_go(tmp_path):
    """28 Oct 19:29: Center believed 45 deg and drove 216 deg (its heading 190 deg off); the line fit was pointed the
    plan's way and gave 34 deg, so the wrong heading was kept for two more drives. Now: the camera's own travel."""
    e = LD.Ladder.end_poses(None, _drive_session(tmp_path, travel_deg=216.0), ["C"])["C"]
    assert abs(LD.wrap180(e["h"] - 216.0)) < 1.0 and e["opposite"] and "OPPOSITE" in e["how"]
    e = LD.Ladder.end_poses(None, _drive_session(tmp_path / "b", travel_deg=47.0), ["C"])["C"]
    assert abs(e["h"] - 47.0) < 1.0 and not e["opposite"]


def test_still_spot_by_camera_source():
    """28 Oct 19:43: Right standing still hopped 1.4 cm between the two cameras' mix and the side camera alone; the
    1.2 cm rule failed for 6 s and Right was parked as "lost". Each source alone is still."""
    h = [(0.1 * k, 51.6, 85.6, "both") if k % 3 else (0.1 * k, 50.4, 84.7, "side") for k in range(9)]
    p = LD.still_spot(h, 0.8, 1.2)
    assert p is not None and math.dist(p, (51.6, 85.6)) < 0.1
    moving = [(0.1 * k, 50.0 + k, 85.0, "both") for k in range(9)]
    assert LD.still_spot(moving, 0.8, 1.2) is None


class _LegBench(LD.Bench):
    """Bench.leg against a robot whose leg end record is lost: it answers `status` idle."""

    def __init__(self, lose_end, idle=True):
        import threading
        from collections import deque
        self.lock = threading.Lock()
        self.legs, self.state, self.fw, self.msgs = {}, {}, {}, deque()
        self.lose_end, self.idle, self.sent = lose_end, idle, []

    def send(self, name, cmd):
        self.sent.append(cmd)
        t = time.time()
        if cmd.startswith("go"):
            self.legs.setdefault(name, []).append((t, "go", 10.0, "started", 0, 0, 0, ""))
            if not self.lose_end:
                self.legs[name].append((t, "go", 10.0, "done", 1.0, 2.0, 3.0, "front=50"))
        elif cmd == "status":
            self.state[name] = (time.time() + 0.01, "idle" if self.idle else "running")


def test_leg_end_record_lost_is_asked_for(monkeypatch):
    """28 Oct 19:22: Left's `go 10` ended, its `done` was lost on the WiFi, the turn check timed out after 20 s and
    the crawl's back leg was then refused. The robot is now asked: idle means the leg ended."""
    import time as _t
    monkeypatch.setattr(LD, "LEG_QUIET_S", 0.2)
    b = _LegBench(lose_end=True)
    t0 = _t.time()
    r = b.leg("Left", "go 10", 5)
    assert r[0] == "done" and r[1] is None and "not heard" in r[4] and _t.time() - t0 < 3.0 and "status" in b.sent
    b = _LegBench(lose_end=True, idle=False)
    r = b.leg("Left", "go 10", 1.0)
    assert r[0] == "timeout"
    b = _LegBench(lose_end=False)
    r = b.leg("Left", "go 10", 5)
    assert r[0] == "done" and r[1] == 1.0 and "status" not in b.sent


def test_crawl_fixes_position_first_and_never_backs_after_an_unknown_leg(monkeypatch):
    import types
    monkeypatch.setattr(LD.time, "sleep", lambda s: None)
    L = LD.Ladder.__new__(LD.Ladder)
    L.rect = (0.0, 0.0, 122.0, 122.0)
    L.others_of = lambda n: {}
    sent, legs = [], []

    def leg(n, t, to):
        legs.append(t)
        return ("timeout", None, None, None, "no end of the leg in 20 s")
    L.bench = types.SimpleNamespace(leg=leg, send=lambda n, c: sent.append(c))
    L.eye = types.SimpleNamespace(still=lambda n: (50.0, 50.0))
    with pytest.raises(LD.LadderStop):
        L.crawl("Left", (50.0, 50.0), None)
    assert sent[0].startswith("fix 50.00 50.00 - ") and legs == ["go 10.0"] and "stop" in sent


def test_crawl_with_no_room_either_way_is_refused():
    L = LD.Ladder.__new__(LD.Ladder)
    L.rect = (0.0, 0.0, 122.0, 122.0)
    L.others_of = lambda n: {"Right": (14.0, 20.0)}
    L.bench = L.eye = None
    # 8 cm from x = 0, facing the edge; backing would come within 12 cm of Right's middle
    with pytest.raises(LD.LadderStop) as e:
        L.crawl("Center", (8.0, 20.0), 180.0)
    assert "no room to crawl" in str(e.value)
