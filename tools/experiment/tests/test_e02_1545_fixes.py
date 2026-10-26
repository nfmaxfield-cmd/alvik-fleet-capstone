"""The 2026-10-25 15:45 session fixes (docs/build-reports/e02-oct25-1545-fixes.md): turn health (fix 1), the heading
crawl (2), cut and refused turns (3), edge touches and back-outs (4), caution around turns (5), no fix from a held
spot (6), routes and set-aside robots (7), and the live page's sounds."""
import math
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "recognition"))

import e02 as E  # noqa: E402
import e02_plan as PL  # noqa: E402
import e02_steer as ST  # noqa: E402
import e02_turnwatch as TW  # noqa: E402


# --- fix 1: turn health ---------------------------------------------------------------------------------------
def test_two_timed_out_turns_in_a_row_trip_and_a_good_turn_clears():
    tw = TW.TurnWatch()
    # 15:58:58.8, Right: its first turn after the compass check (+40) ran to the timeout
    assert tw.feed_turn("Right", 40.0, "failed", "turn timed out", -129.0) == ""
    why = tw.feed_turn("Right", -106.3, "failed", "turn timed out", 70.0)
    assert "last 2 turns failed" in why and "timed out" in why
    assert "Right" in tw.tripped and tw.feed_turn("Right", 10.0, "failed", "turn timed out", 0.0) == ""   # once
    tw2 = TW.TurnWatch()
    tw2.feed_turn("Left", 90.0, "failed", "turn timed out", 50.0)
    tw2.feed_turn("Left", 90.0, "done", "front=40", 89.0)           # a good one in between
    assert tw2.feed_turn("Left", 90.0, "failed", "turn timed out", 50.0) == ""


def test_a_turn_the_wrong_way_by_its_own_heading_is_a_fault_but_small_or_refused_ones_are_not():
    assert "other way" in TW.turn_fault(40.0, "done", "", -35.0)
    assert TW.turn_fault(15.0, "done", "", -12.0) == ""             # under 20 deg: too small to judge
    assert TW.turn_fault(40.0, "done", "", -5.0) == ""              # 5 deg the wrong way: gyro noise
    assert TW.turn_fault(40.0, "failed", "stop command", 25.0) == ""  # cut short, the right way: not a fault
    tw = TW.TurnWatch()
    tw.feed_turn("Right", 40.0, "refused", "virtual border", None)
    assert tw.feed_turn("Right", 40.0, "refused", "a leg is running", None) == ""   # refused: not tried


def test_big_heading_corrections_after_its_own_turns_trip():
    tw = TW.TurnWatch()
    assert tw.correction("Right", 120.0, after_turn=False) == ""     # a placeholder heading: not counted
    assert tw.correction("Right", 120.0, after_turn=True) == ""
    tw.feed_go("Right")
    assert "over 45 deg" in tw.correction("Right", -60.0, after_turn=True)
    tw2 = TW.TurnWatch()
    tw2.correction("Center", 50.0, True)
    for _ in range(6):
        tw2.feed_go("Center")
    assert tw2.correction("Center", 50.0, True) == ""               # outside the 5-leg window


class _Lis:
    """LEG records: leg id -> (start th, final th)."""

    def __init__(self):
        self.th = {}

    def leg_start(self, robot, run, lid):
        return None if lid not in self.th else dict(th_deg=str(self.th[lid][0]))

    def leg_final(self, robot, run, lid):
        return None if lid not in self.th else dict(th_deg=str(self.th[lid][1]), state="failed")


def _health_stub():
    r = types.SimpleNamespace(listener=_Lis(), turnwatch=TW.TurnWatch(), st={}, said=[], held_out=set(),
                              unavailable=set(), moves={"Right": {"run": "-"}}, sent=[])
    r.link = types.SimpleNamespace(send_name=lambda n, c: r.sent.append((n, c)))
    r.say = lambda m, quiet=False: r.said.append(m)
    r.mark = lambda t: None
    r.st = types.SimpleNamespace(d={}, setdefault=lambda k, v: r.st.d.setdefault(k, v), save=lambda: None,
                                 get=lambda k, d=None: r.st.d.get(k, d))
    r.alert = types.MethodType(E.Runner.alert, r)
    r.turn_fault_found = types.MethodType(E.Runner.turn_fault_found, r)

    def leave_out(robot):
        r.held_out.add(robot)
        r.unavailable.add(robot)
        return True
    r.leave_out = leave_out
    r.hold_for_nick = lambda *a, **k: "o"
    return r


def test_right_is_stopped_named_and_left_out_with_a_sound_event():
    r = _health_stub()
    r.listener.th = {1: (-90.0, 141.0), 2: (141.0, -95.0)}     # its own heading moved the wrong way both times
    E.Runner.leg_health(r, "Right", "turn 40.0", r.moves["Right"], 1, ("failed", "turn timed out"))
    assert "Right" not in r.held_out and r.turn_done["Right"] == -129.0
    E.Runner.leg_health(r, "Right", "turn -106.3", r.moves["Right"], 2, ("failed", "turn timed out"))
    assert "Right" in r.held_out and "Right" in r.unavailable
    assert ("Right", "stop") in r.sent
    assert r.st.d["turn_faults"][0][1] == "Right"
    al = r.st.d["alerts"][-1]
    assert al["kind"] == "turn_fault" and al["robot"] == "Right" and "turns are not working" in al["words"]
    assert any("TURNS" in m for m in r.said)


def test_an_edge_stop_is_remembered_for_the_back_out():
    r = _health_stub()
    E.Runner.leg_health(r, "Left", "go 4.0", {"run": "-"}, 9, ("failed", "plywood edge (bottom)"))
    assert abs(r.edge_stop_t["Left"] - time.time()) < 1.0


# --- fix 3: what a cut turn did ----------------------------------------------------------------------------------
def test_turn_done_and_rest():
    assert TW.turn_done_deg(-126.0, -8.0) == 118.0           # 16:40:17, Left: 118 of its 154 deg turn
    assert abs(TW.turn_rest(154.2, 118.0) - 36.2) < 1e-6     # the rest, not 154 again
    assert TW.turn_rest(40.0, 45.0) == 0.0
    assert TW.turn_done_deg(170.0, -170.0) == 20.0           # across +-180
    assert TW.turn_done_deg(None, 3.0) is None


class _DriveStub:
    """Enough of a Runner for drive_to: the camera position is a fixed point, leg() is scripted."""

    def __init__(self, p, heading, script, done=None):
        self.board = PL.Board(xboard_side=1, yboard_side=1)
        self.heads = {"Left": [heading, True]}
        self.watch_count = {"Left": 0}
        self.disagree = {"Left": 0}
        self.front_bad = {}
        self.held_out = set()
        self.turnwatch = TW.TurnWatch()
        self.st = {"positions": {}}
        self.p = p
        self.script = list(script)
        self.sent = []
        self.done = done
        self.watch_stop = None
        self.said = []
        self.turn_done = {}
        self.exploring = False

    def settle(self, **k):
        return self.p

    def safe_path(self, robot, p, target):
        return [target]

    def say(self, m, quiet=False):
        self.said.append(m)

    def camera_lag_s(self):
        return 0.2

    def heading_push(self, robot, h):
        return False

    def leg(self, robot, cmd, run=None, backout=False):
        self.sent.append(cmd)
        out = self.script.pop(0) if self.script else ("done", None)
        words, newp = out
        if cmd[0] == "turn" and self.done is not None and not words.startswith("done"):
            self.turn_done[robot] = self.done
        p0 = self.p
        if newp is not None:
            self.p = newp
        return p0, self.p, words


def _bind(stub):
    for name in ("drive_to", "signed_room", "edge_back_out", "heading_crawl", "open_room_now", "track_heading", "add_wheels"):
        setattr(stub, name, types.MethodType(getattr(E.Runner, name), stub))
    stub.away_leg = E.Runner.away_leg
    stub.wheels_of = E.Runner.wheels_of
    stub.LOCKED_SOURCES = E.Runner.LOCKED_SOURCES
    stub.refind = lambda robot, why: None
    stub.tracker = None
    return stub


def test_a_cut_turn_sends_only_its_rest():
    # Left 30 cm in from the bottom edge facing -126; the target straight in: a +126 turn, cut at 90 by a lost stop
    s = _bind(_DriveStub((30.0, 60.0), -126.0, [("watchdog (lost while driving)", None), ("done", None),
                                                 ("done", (30.0 + 20, 60.0))], done=90.0))
    s.watch_stop = "lost while driving"
    ok, why = s.drive_to("Left", (50.0, 60.0), max_legs=6)
    turns = [c for c in s.sent if c[0] == "turn"]
    assert abs(turns[0][1] - 126.0) < 0.5
    assert abs(turns[1][1] - 36.0) < 1.0, s.sent            # only the rest (the old code sent 126 again)


def test_a_refused_turn_never_counts_as_done():
    s = _bind(_DriveStub((30.0, 60.0), -126.0, [("refused (virtual border)", None)], done=0.0))
    ok, why = s.drive_to("Left", (50.0, 60.0), max_legs=6)
    assert not ok and "refused" in why
    assert s.heads["Left"] == [-126.0, False]                 # not -126 + 126


# --- fix 5: caution around turns -----------------------------------------------------------------------------------
def test_no_turn_on_the_spot_near_an_edge_it_moves_away_first():
    # 7 cm from the bottom edge (x = 0), facing 150 deg (towards the edge): back away first, then turn
    s = _bind(_DriveStub((7.0, 60.0), 150.0, [("done", (7.0 + 3.6, 60.0 - 2.1)), ("done", None), ("done", None)]))
    s.drive_to("Left", (40.0, 60.0), max_legs=3)
    assert s.sent[0][0] == "go" and s.sent[0][1] < 0          # backwards: it faces the edge
    assert s.sent[1][0] == "turn"
    assert s.st["edge_aways"][0][1] == "Left"


def test_away_leg_geometry():
    room = (7.0, (1.0, 0.0))                                  # 7 cm in from an x = low edge
    assert abs(E.Runner.away_leg(180.0, room) + 3.5) < 1e-6   # facing out: back 3.5 cm to reach 10.5
    assert abs(E.Runner.away_leg(0.0, room) - 3.5) < 1e-6     # facing in: forward
    assert E.Runner.away_leg(90.0, room) is None              # along the edge: driving would not help
    assert E.Runner.away_leg(0.0, (20.0, (1.0, 0.0))) is None  # far enough already
    assert E.Runner.away_leg(180.0, (-5.0, (1.0, 0.0))) == -12.0   # capped (centre past the line)


def test_after_a_turn_near_an_edge_the_first_drive_is_a_short_check():
    s = _bind(_DriveStub((16.0, 60.0), 90.0, [("done", None), ("done", (20.0, 60.0)), ("done", (40.0, 60.0))]))
    s.drive_to("Left", (60.0, 60.0), max_legs=3)
    assert s.sent[0][0] == "turn" and s.sent[1] == ("go", E.HEADING_CHECK_CM)


def test_after_a_turn_in_the_open_the_drive_is_full_length():
    s = _bind(_DriveStub((60.0, 60.0), 90.0, [("done", None), ("done", (70.0, 60.0))]))
    s.drive_to("Left", (90.0, 60.0), max_legs=2)
    assert s.sent[1][0] == "go" and s.sent[1][1] > E.HEADING_CHECK_CM


def _border_stub():
    r = types.SimpleNamespace()
    r.board = PL.Board(xboard_side=1, yboard_side=1)
    r.exploring = False
    r.seek, r.edge_touch, r.guard, r.cam, r.moves = set(), None, None, None, {}
    r.leg_live, r.leg_kind = {"Right": 0.0}, {"Right": "turn"}
    r.border_frame = types.MethodType(E.Runner.border_frame, r)
    r.border_cm = types.MethodType(E.Runner.border_cm, r)
    r.spin_check = types.MethodType(E.Runner.spin_check, r)
    return r


def _track(x0, y0, x1, y1, n=6, dt=0.1):
    return [(k * dt, x0 + (x1 - x0) * k / (n - 1), y0 + (y1 - y0) * k / (n - 1)) for k in range(n)]


def test_a_spin_that_carries_the_robot_into_the_band_holds():
    r = _border_stub()
    r.turn_start_xy = {"Right": (8.6, 69.0)}                  # 16:12:54: a 5.4 s spin from 8.6 cm
    why = E.Runner.border_check(r, "Right", _track(7.5, 69.0, 4.8, 69.0))
    assert "while it turned on its spot" in why
    assert E.Runner.border_check(r, "Right", _track(8.4, 69.0, 7.4, 69.0)) == ""   # a 1.2 cm swing: no
    r.turn_start_xy = {"Right": (40.0, 60.0)}
    assert E.Runner.border_check(r, "Right", _track(40.0, 60.0, 35.0, 60.0)) == ""  # outside the band


# --- fix 4: touches and back-outs ------------------------------------------------------------------------------
def test_an_edge_touch_with_an_unmeasured_heading_is_not_logged():
    called = []
    r = types.SimpleNamespace(board=PL.Board(xboard_side=1, yboard_side=1), heads={"Left": [41.0, False]},
                              st={}, learn_open_edges=lambda: called.append(1))
    E.Runner.edge_touch_log(r, "Left", (7.0, 95.0), "failed (plywood edge (bottom))", "go")
    assert "edge_touches" not in r.st and r.st["edge_touches_skipped"][0][1] == "Left" and not called
    r.heads["Left"] = [180.0, True]
    E.Runner.edge_touch_log(r, "Left", (7.0, 95.0), "failed (plywood edge (bottom))", "go")
    assert r.st["edge_touches"] and called


def test_still_going_needs_real_outward_motion():
    bd = PL.Board(xboard_side=1, yboard_side=1)
    jitter = [(k * 0.1, -3.8 - (0.6 if k > 6 else 0.0), 96.5) for k in range(12)]     # 16:40:36: 0.6 cm jitter
    assert ST.hazard(jitter, bd, {}) is None
    out = [(k * 0.1, -3.0 - 0.25 * k, 96.5) for k in range(12)]                          # really going out 2.5 cm/s
    assert "still going" in ST.hazard(out, bd, {})
    towards = [(k * 0.1, 12.0 - 0.6 * k, 60.0) for k in range(12)]                       # approaching: unchanged
    assert "heading for it" in ST.hazard(towards, bd, {})


def test_a_back_out_after_its_own_edge_stop_is_allowed():
    r = types.SimpleNamespace(edge_stop_t={"Left": 100.0}, leg_live={"Left": 101.0}, leg_kind={"Left": "go"},
                              leg_val={"Left": -6.0})
    assert E.Runner.backing_out(r, "Left", now=102.0)
    r.leg_val["Left"] = 4.0
    assert not E.Runner.backing_out(r, "Left", now=102.0)     # forwards: no
    r.leg_val["Left"] = -6.0
    # 2026-10-26 runner fixes, item 2: "right after" = nothing else has run since, however long a hold took (21:14:
    # Left's second back-out came 27 s after its edge stop, after a 104 s hold); a turn or forward drive that ran
    # clears the edge stop (leg_health), and then a back-out is an ordinary backwards drive
    assert E.Runner.backing_out(r, "Left", now=100.0 + E.EDGE_BACKOUT_S + 300)
    r.edge_stop_t.pop("Left")
    assert not E.Runner.backing_out(r, "Left", now=102.0)


# --- fix 2: the heading crawl -------------------------------------------------------------------------------------
class _CrawlStub(_DriveStub):
    def __init__(self, p, script):
        super().__init__(p, None, script)
        self.heads = {"Left": [None, False]}
        self.speed_now = {}
        self.seek = set()
        self.seek_end = {}
        self.backouts = []

    def explore_speed(self, n, cms):
        self.speed_now[n] = cms

    def fw_seek(self, robot, on):
        pass

    def leg(self, robot, cmd, run=None, backout=False):
        if backout:
            self.backouts.append(cmd)
        return super().leg(robot, cmd, run, backout)


def test_crawl_at_the_edge_backs_out_and_measures_the_heading():
    # Left 11 cm from the bottom edge facing 153 deg: the crawl's edge stop, then straight back 6 cm
    s = _bind(_CrawlStub((11.0, 96.0), [("failed (plywood edge (bottom))", (10.0, 96.5)),
                                        ("done", (10.0 + 5.3, 96.5 - 2.7))]))
    q = s.heading_crawl("Left")
    assert s.sent[0] == ("go", E.HEADING_CRAWL_CM) and s.backouts == [("go", -E.HEADING_CRAWL_BACK_CM)]
    assert abs(ST.wrap180(s.heads["Left"][0] - 153.0)) < 3.0 and s.heads["Left"][1]
    assert q == (15.3, 93.8)
    assert s.st["heading_crawls"][0]["edge"] and s.speed_now["Left"] == E.EXPLORE_FAST_CMS


def test_crawl_inward_measures_the_heading_without_backing():
    s = _bind(_CrawlStub((11.0, 96.0), [("done", (12.5, 96.0)), ("done", (14.0, 96.0))]))
    s.heading_crawl("Left")
    assert len(s.sent) == 2 and not s.backouts and abs(s.heads["Left"][0]) < 1.0


def test_no_crawl_with_the_sensors_maybe_over_the_edge():
    s = _bind(_CrawlStub((4.0, 96.0), []))
    assert s.heading_crawl("Left") is None and not s.sent


def test_drive_to_crawls_instead_of_a_blind_probe_near_an_edge():
    s = _bind(_CrawlStub((11.0, 96.0), [("done", (12.5, 96.0)), ("done", (14.0, 96.0)), ("done", None)]))
    s.drive_to("Left", (40.0, 96.0), max_legs=3)
    assert s.sent[0] == ("go", E.HEADING_CRAWL_CM)              # not ("probe", 4.0)


# --- fix 6: no fix from a held spot right after a drive -----------------------------------------------------------
def test_no_fix_from_a_held_spot_right_after_a_drive(monkeypatch):
    monkeypatch.setattr(E, "FIX_HELD_WAIT_S", 0.2)
    r = types.SimpleNamespace(go_end={"Left": time.time() - 0.5}, st={}, REAL_SOURCES=E.Runner.REAL_SOURCES,
                              say=lambda *a, **k: None)
    r.src_log = [(time.time() - 0.2, "held"), (time.time() - 0.1, "held")]
    assert E.Runner.seen_since_drive(r, "Left") is False and r.st["fix_skipped_held"]
    t_ = time.time() - 0.1
    r.src_log.append((t_, "lock"))
    assert E.Runner.seen_since_drive(r, "Left") == t_
    r.go_end = {"Left": time.time() - 30.0}                     # long after the drive: any picture
    assert E.Runner.seen_since_drive(r, "Left") is None


# --- fix 7: routes avoid set-aside robots -----------------------------------------------------------------------
def test_a_route_through_a_set_aside_robot_is_not_run():
    bd = PL.Board(xboard_side=1, yboard_side=1)
    out, back = bd.route_floor(3, bd.nominal_start()[:2])
    mid = ((out[0][0] + out[1][0]) / 2.0, (out[0][1] + out[1][1]) / 2.0)   # on the square's second side
    r = types.SimpleNamespace(board=bd, unavailable={"Right"}, st={"positions": {"Right": [mid[0], mid[1], None]}})
    blk = E.Runner.route_blocked_by_aside(r, 3)
    assert blk[0] == "Right" and blk[1] < 1.0
    r.st["positions"]["Right"] = [5.0, 5.0, None]               # far from every route
    assert E.Runner.route_blocked_by_aside(r, 3) is None
    r.unavailable = set()
    assert E.Runner.route_blocked_by_aside(r, 3) is None


# --- the live page's sounds ---------------------------------------------------------------------------------------
def test_live_page_serves_alerts_and_has_the_sound_control():
    import e02_view as V
    st = {"alerts": [dict(id=1, t=1.0, kind="hold", robot="Left", words="lost"),
                     dict(id=2, t=2.0, kind="turn_fault", robot="Right", words="Right's turns are not working")]}
    r = types.SimpleNamespace(st=st, help_waiting="Left", hold=None)
    v = V.LiveView(r)
    a = v.alerts()
    assert [x["id"] for x in a["items"]] == [1, 2] and a["open"] and a["open_robot"] == "Left"
    assert a["repeat_s"] == E.ALERT_REPEAT_S
    html = v.page("main")
    assert 'id="sndb"' in html and "Turn on sounds" in html and "AudioContext" in html and "sounds(s.alerts)" in html
    r.help_waiting = None
    assert not v.alerts()["open"]


def test_live_page_sound_logic_in_node(tmp_path):
    """The page's sounds() in node: alerts there when the page opened stay silent, a new one plays once (a robot left
    out plays its own tune), a waiting hold repeats after repeat_s and not sooner."""
    import re
    import shutil
    import subprocess
    import pytest
    import e02_view as V
    node = shutil.which("node")
    if node is None:
        pytest.skip("no node")
    js = re.findall(r"<script>(.*?)</script>", V.LiveView(types.SimpleNamespace(st={})).page("main"), re.S)[-1]
    seg = js[js.index("let AC=null,heard=null"):js.index("function tick()")]
    seg = seg.replace("$('sndb').onclick", "var _x").replace("let AC=null", "var AC=null")
    test = ("let rings=[];const $=()=>({});\n" + seg + """
ring=function(k){rings.push(k);lastRing=Date.now()};AC={};
const A=[{id:1,kind:'hold'}],B=A.concat([{id:2,kind:'turn_fault'}]);
sounds({items:A,open:true,repeat_s:30});sounds({items:B,open:false,repeat_s:30});sounds({items:B,open:false,repeat_s:30});
lastRing=Date.now()-31000;sounds({items:B,open:true,repeat_s:30});sounds({items:B,open:true,repeat_s:30});
console.log(JSON.stringify(rings));""")
    f = tmp_path / "t.js"
    f.write_text(test)
    out = subprocess.run([node, str(f)], capture_output=True, text=True, timeout=30)
    assert out.stdout.strip() == '["turn_fault","hold"]', out.stderr


def test_the_watchdog_judges_only_a_robot_that_may_be_moving():
    r = types.SimpleNamespace(leg_live={}, seek=set(), moves={}, find_live={}, leg_end={"Left": 100.0},
                              STILL_GRACE_S=E.Runner.STILL_GRACE_S,
                              watch=types.SimpleNamespace(started={}, ended={}))
    assert not E.Runner.may_move(r, "Left", now=110.0)                # standing: the tracker settling is not it
    assert E.Runner.may_move(r, "Left", now=101.0)                    # just after a leg
    r.watch.started["Left"] = ("r7", "e02")
    assert E.Runner.may_move(r, "Left", now=110.0)                    # a firmware run going
    r.watch.ended["r7"] = dict(result="done")
    assert not E.Runner.may_move(r, "Left", now=110.0)
    r.leg_live["Left"] = 109.0
    assert E.Runner.may_move(r, "Left", now=110.0)


def test_a_crawl_track_the_camera_did_not_follow_gives_no_heading():
    """19:18 dry run: the lock lost at the bottom edge, the detector's positions jumping: no heading from that, and
    none when the camera saw under 40 % of what the wheels drove."""
    t0 = 1000.0
    smp = [(t0 + 0.1 * k, 9.0 + (1.5 if k % 2 else 0.0), 94.0 - 0.1 * k) for k in range(20)]     # jumping
    r = types.SimpleNamespace(history=types.SimpleNamespace(since=lambda t: smp),
                              src_log=[(q[0], "detector") for q in smp], LOCKED_SOURCES=E.Runner.LOCKED_SOURCES)
    assert E.Runner.track_heading(r, t0, (11.0, 94.6), (9.4, 94.3))[0] is None
    good = [(t0 + 0.1 * k, 9.0 - 0.12 * k, 94.0 - 0.16 * k) for k in range(20)]                  # 3.8 cm at -127
    r.history = types.SimpleNamespace(since=lambda t: good)
    r.src_log = [(q[0], "both") for q in good]
    h, moved = E.Runner.track_heading(r, t0, good[0][1:], good[-1][1:], wheels_cm=4.9)
    assert abs(h + 127.0) < 2.0
    assert E.Runner.track_heading(r, t0, good[0][1:], good[-1][1:], wheels_cm=20.0)[0] is None   # saw 19 %
    r.src_log = [(q[0], "both" if k % 2 else "detector") for k, q in enumerate(good)]            # locked half the time
    assert E.Runner.track_heading(r, t0, good[0][1:], good[-1][1:], wheels_cm=4.9)[0] is None


# --- fix 1, root cause: the compass check ------------------------------------------------------------------------
class _CompassStub:
    """A robot whose compass check turn is scripted: each entry (state, info, own heading change)."""

    def __init__(self, script, mend_after=None):
        self.script = list(script)
        self.calibrated = 0
        self.mend_after = mend_after
        self.listener = _Lis()
        self.link = types.SimpleNamespace(send_name=lambda n, c: None)
        self.moves = {}
        self.leg_ids = {}
        self.last_lid = {}
        self.turn_speed_now = {}
        self.held_out, self.unavailable = set(), set()
        class _St(dict):
            def save(self):
                pass
        self.st = _St()
        self.said = []
        self.watch = object()
        self.turnwatch = TW.TurnWatch()
        for n in ("compass_verify", "compass_turn_check", "next_id", "turn_fault_found", "alert"):
            setattr(self, n, types.MethodType(getattr(E.Runner, n), self))
        for k in ("COMPASS_CHECK_DEG", "COMPASS_CHECK_TIMEOUT_MS", "COMPASS_TRIES"):
            setattr(self, k, getattr(E.Runner, k))

    def say(self, m, quiet=False):
        self.said.append(m)

    def mark(self, t):
        pass

    def _calibrate(self, robots, verify=True):
        self.calibrated += 1

    def _send_leg_once(self, robot, text, timeout, m, lid, following):
        if not text.startswith("turn 15"):
            return "done", ""
        state, info, dth = self.script.pop(0)
        self.listener.th[lid] = (0.0, dth)
        return state, info

    def leave_out(self, robot):
        self.held_out.add(robot)
        return True

    def hold_for_nick(self, *a, **k):
        return "o"


def test_a_compass_check_with_the_wrong_sign_is_redone_and_then_passes():
    # 15:58:41, Right: its check learned the wrong sign; the check turn runs the other way to its capped timeout
    s = _CompassStub([("failed", "turn timed out", -80.0), ("done", "", 15.2)])
    assert s.compass_verify("Right")
    assert s.calibrated == 1 and "Right" not in s.held_out
    assert [c[3] for c in s.st["compass_checks"]] == [False, True]


def test_a_compass_check_that_keeps_failing_leaves_the_robot_out():
    s = _CompassStub([("failed", "turn timed out", -80.0)] * 3)
    assert not s.compass_verify("Right")
    assert s.calibrated == 2 and "Right" in s.held_out and s.st["alerts"][-1]["kind"] == "turn_fault"


def test_a_turn_fault_first_gets_a_new_compass_check_that_can_mend_it():
    s = _CompassStub([("done", "", 14.8)])
    s.moves["Right"] = {"run": "-"}
    E.Runner.turn_fault_found(s, "Right", "its last 2 turns failed")
    assert "Right" not in s.held_out and s.calibrated == 1 and s.moves["Right"] == {"run": "-"}
    assert s.st["alerts"][-1]["kind"] == "compass"
    # a second fault later: no second repair, it is left out
    E.Runner.turn_fault_found(s, "Right", "its last 2 turns failed")
    assert "Right" in s.held_out
