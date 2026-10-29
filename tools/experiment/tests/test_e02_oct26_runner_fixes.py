"""2026-10-26 runner fixes (MIRLT): commands that arrive, back-outs, no turns at an edge, tilt / stuck stops, parking,
the simulator's lost commands, rescues instead of ending, finding a robot by its lights before asking.
docs/build-reports/e02-oct26-runner-fixes.md."""
import math
import os
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "sim"))

import e02 as E  # noqa: E402
import e02_link as LNK  # noqa: E402
import e02_plan as PL  # noqa: E402
import e02_steer as ST  # noqa: E402


# =============================================================================== item 1: the link layer
class FakeSession:
    def __init__(self):
        self.listeners = []
        self.lock = threading.Lock()
        self.clock_sent = {}

    def line(self, name, kind, rtype, vals):
        for fn in list(self.listeners):
            fn(name, kind, rtype, vals)


class FakeLink:
    """The console's Link, with a robot that answers (or not) like firmware 2026-10-26a."""

    def __init__(self, drop=(), answer_set=True, fw_numbered=True):
        self.session = FakeSession()
        self.sent = []
        self.drop = list(drop)        # packets (by order of sending) the robot never gets
        self.answer_set = answer_set
        self.seen = set()
        self.ran = []
        self.params = {}
        self.fw_numbered = fw_numbered

    def robots(self):
        return ["Left"]

    def send_name(self, name, cmd):
        self.sent.append((time.time(), name, cmd))
        k = len(self.sent) - 1
        if k in self.drop:
            return True
        threading.Thread(target=self._robot, args=(name, cmd), daemon=True).start()
        return True

    def send_ip(self, ip, cmd):
        pass

    def send_all(self, cmd):
        pass

    def close(self):
        pass

    def summary(self):
        return []

    def _robot(self, name, cmd):
        time.sleep(0.01)
        if cmd.startswith("#"):
            head, _, cmd = cmd.partition(" ")
            n = int(head[1:])
            dup = n in self.seen
            self.session.line(name, "D", "ACK", [str(n), "dup" if dup else "new", "0"])
            if dup:
                return
            self.seen.add(n)
        self.ran.append(cmd)
        parts = cmd.split()
        if parts[0] == "set" and len(parts) == 3:
            if self.answer_set:
                self.params[parts[1]] = float(parts[2])
                self.session.line(name, "M", "", "%s = %.3f" % (parts[1], float(parts[2])))
        elif parts[0] == "get" and len(parts) == 2:
            self.session.line(name, "M", "", "%s = %.3f  (help)" % (parts[1], self.params.get(parts[1], 0.0)))


def _rl(fake, numbered=True, waits=(0.15, 0.15, 0.2, 0.2, 0.2)):
    fw = {"Left": "2026-10-26a" if numbered else "2026-10-25c"}
    return LNK.ReliableLink(fake, firmware=fw, waits=waits)


def _wait(cond, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_a_numbered_command_is_acknowledged_and_not_resent():
    fake = FakeLink()
    rl = _rl(fake)
    try:
        rl.send_name("Left", "go 12.00 5")
        assert fake.sent[0][2].startswith("#") and fake.sent[0][2].endswith(" go 12.00 5")
        assert _wait(lambda: not rl.pending)
        time.sleep(0.4)
        assert len(fake.sent) == 1 and fake.ran == ["go 12.00 5"]
        assert rl.counts["first_try"] == 1 and rl.gauge.share(min_n=1)[0] == 0.0
        assert rl.ack_time("Left", "go 12.00 5") is not None
    finally:
        rl.quit.set()


def test_a_lost_command_is_resent_with_the_same_number_and_runs_once():
    fake = FakeLink(drop=[0])                  # the first send never arrives (21:14: 1 in 10)
    rl = _rl(fake)
    try:
        rl.send_name("Left", "go 12.00 5")
        assert _wait(lambda: not rl.pending)
        nums = {c.split()[0] for _, _, c in fake.sent}
        assert len(fake.sent) == 2 and len(nums) == 1      # resent once, same number
        assert fake.ran == ["go 12.00 5"]
        assert rl.counts["resends"] == 1 and [e[1] for e in rl.gauge.ev] == [False]
    finally:
        rl.quit.set()


def test_an_answer_that_is_lost_makes_a_resend_the_robot_answers_dup_and_does_not_run_again():
    fake = FakeLink()
    rl = _rl(fake)
    orig = fake.session.line

    def lose_first_ack(name, kind, rtype, vals, box=[]):
        if rtype == "ACK" and not box:
            box.append(1)
            fake.seen.add(int(vals[0]))
            return                              # the robot got it (and ran it) but its answer was lost
        orig(name, kind, rtype, vals)
    fake.session.line = lose_first_ack
    try:
        rl.send_name("Left", "turn 90.00 6")
        assert _wait(lambda: not rl.pending)
        assert len(fake.sent) == 2
        assert fake.ran == ["turn 90.00 6"]                 # once: the resend was answered dup
    finally:
        rl.quit.set()


def test_a_command_never_answered_is_given_up_and_counted():
    fake = FakeLink(drop=list(range(20)))
    rl = _rl(fake, waits=(0.05, 0.05, 0.05, 0.05, 0.05))
    try:
        rl.send_name("Left", "go 5.00 9")
        assert _wait(lambda: not rl.pending)
        assert len(fake.sent) == 6 and rl.counts["unacked"] == 1 and list(rl.unacked)[-1][2] == "go 5.00 9"
    finally:
        rl.quit.set()


def test_stop_goes_out_three_times_under_one_number():
    fake = FakeLink()
    rl = _rl(fake)
    try:
        rl.send_name("Left", "stop")
        assert _wait(lambda: len(fake.sent) >= 3)
        assert len({c for _, _, c in fake.sent}) == 1 and fake.ran == ["stop"]
    finally:
        rl.quit.set()


def test_older_firmware_gets_plain_commands():
    fake = FakeLink()
    rl = _rl(fake, numbered=False)
    try:
        rl.send_name("Left", "go 5.00 9")
        assert fake.sent[0][2] == "go 5.00 9" and not rl.pending
        rl.firmware["Left"] = "2026-10-26a"
        rl.send_name("Left", "go 5.00 10")
        assert fake.sent[-1][2].startswith("#")
    finally:
        rl.quit.set()


def test_a_setting_is_confirmed_by_the_robots_own_words():
    fake = FakeLink()
    rl = _rl(fake)
    try:
        rl.send_name("Left", "set e02_cms 4")
        assert rl.wait_settings("Left", timeout=2.0) == []
        assert rl.counts["set_resends"] == 0
    finally:
        rl.quit.set()


def test_a_lost_setting_is_read_back_and_sent_again_until_it_matches():
    """21:14: Center's `set e02_open_x_cm 0` never arrived and was never sent again (its own border off all session).
    On plain (older) firmware too: the read-back needs no numbers."""
    for numbered in (True, False):
        fake = FakeLink(answer_set=False)       # the robot never takes the setting ...
        rl = _rl(fake, numbered=numbered)
        LNK_CHECK = LNK.SET_CHECK_S
        LNK.SET_CHECK_S = 0.1
        try:
            fake.params["e02_open_x_cm"] = -999.0
            rl.send_name("Left", "set e02_open_x_cm 0")
            assert _wait(lambda: any(c.endswith("get e02_open_x_cm") for _, _, c in fake.sent), 2.0)
            fake.answer_set = True              # ... until a later send arrives
            assert rl.wait_settings("Left", timeout=5.0) == [] and rl.failed_settings() == []
            assert rl.counts["set_resends"] >= 1
            assert any(c.endswith("get e02_open_x_cm") for _, _, c in fake.sent)
            assert fake.params["e02_open_x_cm"] == 0.0
        finally:
            LNK.SET_CHECK_S = LNK_CHECK
            rl.quit.set()


def test_a_setting_that_never_takes_is_given_up_and_reported():
    fake = FakeLink(answer_set=False)
    rl = _rl(fake, waits=(0.03, 0.03, 0.03, 0.03, 0.03))
    LNK_CHECK = LNK.SET_CHECK_S
    LNK.SET_CHECK_S = 0.05
    try:
        rl.send_name("Left", "set safe_front_cm 7")
        left = rl.wait_settings("Left", timeout=6.0)
        assert left == [] and rl.failed_settings("Left") == [("Left", "safe_front_cm", 7.0)]
        assert rl.status()["set_failed"] == [["Left", "safe_front_cm", 7.0]]
    finally:
        LNK.SET_CHECK_S = LNK_CHECK
        rl.quit.set()


def test_link_gauge_levels_from_the_analysis():
    g = LNK.Gauge()
    now = 1000.0
    for k in range(100):
        g.add(k % 50 != 0, "Left", now - 100 + k)          # 2 % lost: a normal session
    assert g.level(now) == "ok"
    g2 = LNK.Gauge()
    for k in range(100):
        g2.add(k % 10 != 0, "Left", now - 100 + k)          # 10 %: 21:14
    assert g2.level(now) == "warn"
    g3 = LNK.Gauge()
    for k in range(250):
        g3.add(k % 4 != 0, "Left", now - 260 + k)           # 25 %, clearly above 15 %
    assert g3.level(now) == "pause"
    g5 = LNK.Gauge()
    for k in range(300):
        g5.add(k % 20 not in (0, 7, 13), "Left", now - 290 + k)   # 15 % exactly: not clearly above: warn, no pause
    assert g5.level(now) == "warn"
    assert LNK.LINK_WARN == 0.05 and LNK.LINK_PAUSE == 0.15 and LNK.RESEND_WAITS[0] == 1.0
    g4 = LNK.Gauge()
    for k in range(20):
        g4.add(True, "Left", now - 50 + k)
    assert g4.recovered(now) and g4.level(now) is None      # too few for a level, enough for "recovered"


def test_unanswered_pings_count_in_the_gauge():
    fake = FakeLink()
    rl = _rl(fake)
    try:
        t = time.time()
        with fake.session.lock:
            fake.session.clock_sent.update({"c1": ("Left", t - 5.0), "c2": ("Left", t - 0.1)})
        rl._pings()
        with fake.session.lock:
            fake.session.clock_sent.pop("c2")              # answered (the console pops it and the answer arrives)
        fake.session.line("Left", "D", "CLOCK", ["c2", "12345", "F3AF"])
        fake.session.line("Left", "D", "CLOCK", ["c3", "12346", "F3AF"])   # answered between two looks: still counted
        rl._pings()
        s, n = rl.gauge.share(min_n=1)
        assert n == 3 and abs(s - 1 / 3.0) < 1e-9
    finally:
        rl.quit.set()


def test_the_runner_pauses_moves_on_a_bad_link_and_goes_on_when_it_recovers():
    r = E.Runner.__new__(E.Runner)
    r.st = {}
    r.said, r.marks, r.alerts_ = [], [], []
    r.say = lambda m, quiet=False: r.said.append(m)
    r.mark = lambda t: r.marks.append(t)
    r.alert = lambda k, rb, w: r.alerts_.append((k, w))
    r.link_paused = None
    r.link_level = None
    g = LNK.Gauge()
    r.link = types.SimpleNamespace(gauge=g, status=lambda now=None: dict(level=g.level(now), share=g.share(now=now)[0]))
    now = time.time()
    for k in range(240):
        g.add(k % 4 != 0, "Left", now - 250 + k)            # 25 %
    E.Runner.link_tick(r, now)
    assert r.link_paused == now and r.alerts_[-1][0] == "link" and "e02 link pause" in r.marks
    g.ev.clear()
    for k in range(30):
        g.add(k % 10 != 0, "Left", now + 1 + 3 * k)          # 10 % over the next 2 minutes: under the pause level
    E.Runner.link_tick(r, now + 92)
    assert r.link_paused is None and r.st["link_events"][-1][1] == "resume"


# =============================================================================== the firmware model (simulator)
def test_the_simulated_robot_answers_numbers_and_never_runs_a_repeat():
    import fleet_sim as FS
    radio = FS.Radio.__new__(FS.Radio)
    radio.lock = threading.Lock()
    from collections import deque
    radio.inbox = deque()
    radio.got_cmd = False
    out = []
    rb = types.SimpleNamespace(fw_version="2026-10-26a", t_at=lambda ms: 1e18, ms=0, send=out.append,
                               millis=lambda: 1234)
    for c in ("#5 set e02_cms 4", "#5 set e02_cms 4", "go 3 1", "#6 stop"):
        radio.inbox.append((0.0, c))
    got = [radio.poll(rb) for _ in range(4)]
    assert got == ["set e02_cms 4", None, "go 3 1", "stop"]
    assert [o for o in out if o.startswith("D,ACK")] == ["D,ACK,5,new,1234", "D,ACK,5,dup,1234", "D,ACK,6,new,1234"]


def test_a_resent_leg_id_is_dup_before_any_refusal():
    """Firmware 26a (and the simulator's copy): a leg id seen before answers dup first; it used to be refused "a leg
    is running" while the first copy ran, and the laptop took that for the leg's own answer."""
    src = open(os.path.join(os.path.dirname(os.path.dirname(HERE)), "..", "firmware", "home_lab", "e02.h")).read()
    i_dup = src.index('legRecord(id, kind, arg, "dup", "")')
    i_running = src.index('"a leg is running"')
    assert i_dup < i_running
    import fw_e02
    fsrc = open(fw_e02.__file__).read()
    assert fsrc.index('"dup", ""') < fsrc.index('"a leg is running"')


def test_firmware_version_and_the_tilt_parameters():
    fw = os.path.join(os.path.dirname(os.path.dirname(HERE)), "..", "firmware", "home_lab")
    ino = open(os.path.join(fw, "home_lab.ino")).read()
    assert '#define FW_VERSION "2026-10-26a"' in ino or '#define FW_VERSION "2026-10-27a"' in ino \
        or '#define FW_VERSION "2026-10-28a"' in ino \
        or '#define FW_VERSION "2026-10-29a"' in ino   # 27a: school build; 28a: one system
    p = open(os.path.join(fw, "params.h")).read()
    assert '{"tilt_stop_deg",    5,' in p and '{"tilt_ms",        300,' in p
    n = open(os.path.join(fw, "net.h")).read()
    assert "D,ACK,%lu,%s,%lu" in n and "ACK_KEEP = 64" in n


# =============================================================================== item 1c: lost counted from 'started'
def _lost_stub(t_send, started_at=None, ack_at=None, numbered=True):
    recs = {"LEG": [(started_at, "Right", {"id": "9", "state": "started"})] if started_at else []}
    link = types.SimpleNamespace(ack_time=lambda rb, c: ack_at, numbered=lambda rb: numbered)
    return types.SimpleNamespace(leg_live={"Right": t_send}, leg_kind={"Right": "go"}, leg_val={"Right": -11.79},
                                 refinding=set(), seek=set(), speed_now={"Right": 4.0}, last_lid={"Right": 9},
                                 leg_text={"Right": "go -11.79 9"}, link=link,
                                 listener=types.SimpleNamespace(cv=threading.Condition(), recs=recs),
                                 history=types.SimpleNamespace(last_t=lambda: 1000.0,
                                                               since=lambda t: [(t, 50.0, 7.0), (1000.0, 50.0, 7.0)]))


def test_a_back_out_that_never_reached_the_robot_is_not_a_lost_robot():
    """21:30:14: Right's `go -11.79` never arrived; 4 s later the lost check said "wheels ~10 cm, camera 1 cm"."""
    r = _lost_stub(t_send=995.5)                          # no ack, no 'started': it is not driving
    assert E.Runner.lost_why(r, "Right", 1000.0) == ""
    r = _lost_stub(t_send=995.5, ack_at=999.0)             # answered late (resent): counts from then
    assert E.Runner.lost_why(r, "Right", 1000.0) == ""
    r = _lost_stub(t_send=996.3, started_at=996.4)         # really started 3.6 s ago, camera saw nothing move
    assert "wheels should have taken it" in E.Runner.lost_why(r, "Right", 1000.0)
    r = _lost_stub(t_send=990.0, started_at=990.2)         # 10 s ago: an 11.8 cm leg is over; its end record was lost
    assert E.Runner.lost_why(r, "Right", 1000.0) == ""


# =============================================================================== item 2: back-outs and the wobble
def _border_stub():
    r = types.SimpleNamespace(board=PL.Board(xboard_side=1, yboard_side=1), exploring=False, seek=set(),
                              edge_touch=None, guard=None, cam=None, leg_live={"Left": 0.0}, leg_kind={"Left": "go"},
                              leg_val={"Left": -9.5}, moves={})
    r.border_frame = types.MethodType(E.Runner.border_frame, r)
    r.border_cm = types.MethodType(E.Runner.border_cm, r)
    return r


def _track(x0, y0, x1, y1, n=6, dt=0.1):
    return [(k * dt, x0 + (x1 - x0) * k / (n - 1), y0 + (y1 - y0) * k / (n - 1)) for k in range(n)]


def test_a_back_out_after_its_own_edge_stop_is_never_held_by_the_border():
    """21:42:47: Left's back-out held 0.1 s in, "6 cm past the line, moving further out" (pictures of the crawl)."""
    r = _border_stub()
    crawl_pictures = _track(-2.0, 60.0, -6.0, 60.0)        # the camera still shows the forward crawl
    assert "past an open edge" in E.Runner.border_check(r, "Left", crawl_pictures)
    r.edge_stop_t = {"Left": -500.0}                        # its own edge stop, long ago (a hold in between)
    assert E.Runner.border_check(r, "Left", crawl_pictures) == ""


def test_the_border_ignores_the_cameras_wobble():
    """21:43:14 (0.8 cm) and 22:55:06 (8.0 -> 7.0 cm in 0.5 s, the overnight hold) were wobble."""
    r = _border_stub()
    r.leg_val["Left"] = 3.0
    wobble = [(0.0, 8.0, 60.0), (0.1, 7.6, 60.0), (0.2, 8.1, 60.0), (0.3, 7.3, 60.0), (0.4, 7.9, 60.0),
              (0.5, 7.0, 60.0)]
    assert E.Runner.border_check(r, "Left", wobble) == ""
    assert "moving towards it" in E.Runner.border_check(r, "Left", _track(9.0, 60.0, 7.0, 60.0))   # 4 cm/s: real
    assert E.BORDER_JITTER_CM == 1.5


def test_line_ends_fit_a_straight_line():
    (x0, y0), (x1, y1) = E.line_ends([(0.0, 1.0, 5.0), (1.0, 2.0, 5.0), (2.0, 3.0, 5.0)])
    assert abs(x0 - 1.0) < 1e-9 and abs(x1 - 3.0) < 1e-9 and abs(y1 - 5.0) < 1e-9


# =============================================================================== item 3: no turn at an edge
class _ExploreStub:
    """Enough of a runner for edge_clear_before_turn / at_edge_why."""

    def __init__(self, room):
        self.room = [room]
        self.edge_stop_t, self.backed_since_edge, self.last_go = {}, {}, {}
        self.legs = []
        self.st = {}
        self.said = []
        self.exploring = True
        self.listener = None

    def say(self, m, quiet=False):
        self.said.append(m)

    def camera_edge_room(self, robot):
        return self.room[0]

    def at_edge_why(self, robot):
        return E.Runner.at_edge_why(self, robot)

    def _explore_leg_once(self, robot, kind, val, timeout=None):
        self.legs.append((kind, round(val, 2)))
        if kind == "go":
            self.room[0] += -val if self.room[0] is not None and val < 0 else 0.0
            if robot in self.edge_stop_t and val < 0:
                self.backed_since_edge[robot] = self.backed_since_edge.get(robot, 0.0) - val
        return "done", "", 1

    def explore_stop(self, *a, **k):
        return None


def test_no_turn_after_its_own_edge_stop_until_it_has_backed_out():
    """21:44:58: `turn 88.6` ran with Left's nose over the edge and swung its body over the corner."""
    r = _ExploreStub(room=-0.5)
    r.edge_stop_t["Left"] = time.time()
    r.backed_since_edge["Left"] = 0.0
    assert E.Runner.edge_clear_before_turn(r, "Left")
    assert r.legs[0] == ("go", -E.EXPLORE_EDGE_BACK_CM)    # straight back first
    assert r.st["edge_clear_moves"][0][1] == "Left"


def test_a_turn_is_refused_when_it_cannot_get_clear():
    r = _ExploreStub(room=3.0)                              # the camera: 3 cm from the edge, no edge stop, no last drive
    assert not E.Runner.edge_clear_before_turn(r, "Left")
    assert r.legs == [] and r.st["edge_turns_refused"]


def test_near_the_edge_by_the_camera_it_moves_the_way_its_last_drive_did_not_bring_it():
    r = _ExploreStub(room=3.0)
    r.last_go["Left"] = (-6.0, 9.0, 3.0)                    # backing brought it nearer: go forward
    E.Runner.edge_clear_before_turn(r, "Left")
    assert r.legs[0][0] == "go" and r.legs[0][1] > 0


def test_driving_along_an_edge_it_does_not_rock_back_and_forth():
    """Dry run D3: Left 7 cm from the bottom edge, driving along it: straight moves never got it clear."""
    r = _ExploreStub(room=7.0)
    assert E.Runner.at_edge_why(r, "Left") == ""            # 7 cm: its body does not reach the line (explore: 5 cm)
    r = _ExploreStub(room=3.0)
    r.last_go["Left"] = (-9.5, 3.2, 3.0)                    # a 9.5 cm drive changed its room 0.2 cm: along the edge
    assert not E.Runner.edge_clear_before_turn(r, "Left") and r.legs == []
    assert E.EXPLORE_TURN_ROOM_CM == 5.0


# =============================================================================== item 4: tilt and stuck
def test_tilt_from_raw_rows_against_its_own_level():
    import alvik_console  # noqa: F401  (the listener's session type is not needed)
    lis = E.Listener(types.SimpleNamespace(headers={"RAW": ["pitch_deg", "roll_deg", "run", "ms"]}))
    now = time.time()
    for k in range(200):
        lis("Left", "D", "RAW", ["0.5", "0.2", "r", str(k)])
    with lis.cv:                                            # (spread the rows over the last 10 s, 20 a second)
        lis.tilt["Left"] = E.deque([(now - 10.0 + 0.05 * k, p, r_) for k, (_, p, r_) in
                                    enumerate(lis.tilt["Left"])], maxlen=E.TILT_KEEP)
    assert lis.tilt_dev("Left", 0.3, now) is not None and lis.tilt_dev("Left", 0.3, now) < 1.0
    with lis.cv:
        lis.tilt["Left"].extend((now + 0.1 + 0.05 * k, -9.7, -1.4) for k in range(9))    # 21:45:49, Left hanging
    now += 0.5
    assert lis.tilt_dev("Left", 0.3, now) > 9.0
    r = types.SimpleNamespace(listener=lis)
    assert "10.2 deg off level" in E.Runner.tilt_why(r, "Left", now)
    assert E.TILT_STOP_DEG == 5.0 and E.TILT_MS == 300.0


def test_wheels_turning_while_the_camera_sees_it_still_is_stuck():
    """21:45:05-35: ray 8's wheels counted 104.85 cm, the camera saw 1 px."""
    now = 1000.0
    still = [(now - 3.0 + 0.1 * k, 4.0 + (0.3 if k % 2 else -0.3), -6.2) for k in range(31)]
    recs = {"LEG": [(now - 10.0, "Left", {"id": "4", "state": "started"})]}
    r = types.SimpleNamespace(leg_live={"Left": now - 10.0}, leg_kind={"Left": "go"}, leg_val={"Left": 105.0},
                              speed_now={"Left": 4.0}, last_lid={"Left": 4},
                              listener=types.SimpleNamespace(cv=threading.Condition(), recs=recs),
                              history=types.SimpleNamespace(last_t=lambda: now, since=lambda t: still))
    assert "camera saw it move" in E.Runner.stuck_why(r, "Left", now)
    r.leg_val["Left"] = 12.0                                # D4: a 12 cm crawl at 4 cm/s, started 10 s ago: over
    assert E.Runner.stuck_why(r, "Left", now) == ""
    r.leg_val["Left"] = 105.0
    r.leg_val["Left"] = 6.42                                # D2: a 6.4 cm back-out whose 'done' record was lost
    assert E.Runner.stuck_why(r, "Left", now) == ""
    r.leg_val["Left"] = 105.0
    moving = [(now - 3.0 + 0.1 * k, 4.0 + 0.4 * k, -6.2) for k in range(31)]
    r.history = types.SimpleNamespace(last_t=lambda: now, since=lambda t: moving)
    assert E.Runner.stuck_why(r, "Left", now) == ""


# =============================================================================== item 5: parking
def test_no_back_and_forth_when_the_target_is_nearer_than_the_check_drive():
    """22:52:54-22:53:48: Center went to and fro 5 times between 17 and 23 cm from the edge."""
    r = types.SimpleNamespace(st={}, said=[])
    r.say = lambda m, quiet=False: r.said.append(m)
    steer = ST.Steer([(50.0, 20.0)], heading=90.0, measured=False)
    steer.i = 0
    p = (50.0, 22.5)                                        # 2.5 cm past it, facing away (heading +90: up)
    cmd = E.Runner.near_target_leg(r, "Center", steer, p, ("turn", -175.0))
    assert cmd == ("go", -2.5)                              # rolls back instead of a half turn
    p = (50.0, 17.5)
    cmd = E.Runner.near_target_leg(r, "Center", steer, p, ("turn", 178.0))
    assert cmd[0] == "go" and cmd[1] > 0                    # first change of direction
    cmd = E.Runner.near_target_leg(r, "Center", steer, (50.0, 22.0), ("turn", -170.0))
    assert cmd == ("done", 0.0) and r.st["park_close_enough"]
    far = E.Runner.near_target_leg(r, "Center", steer, (50.0, 40.0), ("turn", 90.0))
    assert far is None                                      # far away: the plan's own


# =============================================================================== item 7: rescues
def test_a_tilted_robot_is_not_driven_by_its_rescue():
    r = E.Runner.__new__(E.Runner)
    r.st = types.SimpleNamespace(save=lambda: None)
    store = {}
    r.st = type("S", (dict,), {"save": lambda self: None})()
    r.said, r.marks = [], []
    r.say = lambda m, quiet=False: r.said.append(m)
    r.mark = lambda t: r.marks.append(t)
    r.alert = lambda *a: None
    r.leg_live = {}
    assert E.Runner.self_rescue(r, "Left", "tilt", "tilted 9.7 deg") is False
    assert r.st["rescues"][-1][4] is False and "not driven" in r.st["rescues"][-1][5][0]
    assert store == {}


def test_a_border_holds_rescue_backs_the_robot_out():
    r = E.Runner.__new__(E.Runner)
    r.st = type("S", (dict,), {"save": lambda self: None})()
    r.said, r.marks = [], []
    r.say = lambda m, quiet=False: r.said.append(m)
    r.mark = lambda t: r.marks.append(t)
    r.alert = lambda *a: None
    r.backed = []
    r.back_out = lambda robot, within_s=60.0, inward_ok=False: r.backed.append((robot, inward_ok)) or True
    r.camera_edge_room = lambda robot: 12.0
    r.edge_stop_t, r.last_go = {}, {}
    r.tracker = None
    r.exploring = False
    assert E.Runner.self_rescue(r, "Right", "border", "7 cm from an open edge, moving towards it") is True
    assert r.backed == [("Right", True)] and r.st["rescues"][-1][4] is True   # 2026-10-27 fix 4: inward allowed


# =============================================================================== coordinator: found by its lights
def test_a_lost_robot_found_by_its_lights_is_not_put_to_nick():
    r = E.Runner.__new__(E.Runner)
    r.st = type("S", (dict,), {"save": lambda self: None})()
    r.st["robots"] = ["Left"]
    r.hold, r.hold_lock = threading.Event(), threading.Lock()
    r.held_out, r.stop_requested, r.hold_auto_refind = set(), False, False
    r.hold_owner, r.last_hold_end, r.order = None, 0.0, "explore"
    r.link = types.SimpleNamespace(send_name=lambda n, c: True, robots=lambda: ["Left"])
    r.found = []
    r.auto_find = lambda robot, why: r.found.append(why) or (40.0, 30.0)
    asked = []
    r.hold_ask = lambda *a, **k: asked.append(a) or "c"
    assert E.Runner.hold_for_nick(r, "Left", "lost", "wheels 11 cm, camera 3 cm") == "c"
    assert r.found == ["wheels 11 cm, camera 3 cm"] and asked == []
    assert r.st["holds"][-1][5] == "found" and not r.hold.is_set()


# =============================================================================== the simulator's hang and drops
def test_the_simulated_robot_hangs_over_a_corner_and_tilts():
    import config
    import robot as R
    cfg = config.load_config(None)
    cfg.session.board_cm = [0.0, 0.0, 122.0, 122.0]
    cfg.session.open_edges = ["x0", "y0"]
    import random
    rb = R.Robot("Left", cfg, random.Random(1), config.firmware_params(), config.firmware_version(), 0.0, 0x1234,
                 "127.0.0.9")
    rb.hang_cm = 0.0
    rb.x, rb.y, rb.th = 60.0, 60.0, 0.0                      # mid-board: drives as before, never hangs
    rb.set_wheels_speed(20, 20)
    rb.advance(1000)
    assert not getattr(rb, "hung", False) and rb.x > 60.5
    rb.x, rb.y, rb.th = 7.4, 1.0, math.radians(-90.0)       # nose at the right edge (y = 0)
    rb.set_wheels_speed(20, 20)
    rb.advance(3000)
    assert rb.hung and rb.past_open_edge(rb.x, rb.y) < 3.0
    x_hung = rb.x
    rb.advance(2000)
    assert rb.x == x_hung                                    # wheels turn, body still
    roll, pitch = rb.tilt_now()
    assert pitch < -8.0


def test_simulator_drop_options():
    import e02_sim
    src = open(e02_sim.__file__).read()
    for opt in ("--drop-in-pct", "--drop-out-pct", "--low-contrast", "--hang-cm", '"s2114"'):
        assert opt in src
    esrc = open(E.__file__).read()
    for opt in ("--dry-drop-pct", "--dry-low-contrast", "--plain-commands"):
        assert opt in esrc


# =============================================================================== coordinator 10:45: board sides from explore
def test_explores_own_board_sides_skip_the_questions():
    r = types.SimpleNamespace(st={"setup": {"explore_board_words": dict(x="top", y="left")}})
    xs_words, ys_words = {0: "bottom", 1: "top"}, {0: "right", 1: "left"}
    assert E.Runner.explore_board_sides(r, xs_words, ys_words) == (1, 1)
    r.st["setup"]["explore_board_words"] = dict(x="left", y="top")          # the axes swapped in this calibration
    assert E.Runner.explore_board_sides(r, {0: "right", 1: "left"}, {0: "bottom", 1: "top"}) == (1, 1)
    r.st["setup"]["explore_board_words"] = dict(x="top", y="top")           # nonsense: ask
    assert E.Runner.explore_board_sides(r, xs_words, ys_words) is None
    r.st["setup"] = {}
    assert E.Runner.explore_board_sides(r, xs_words, ys_words) is None     # explore did not find them: ask


def test_board_sides_from_explores_contacts_when_its_fit_failed():
    import numpy as np
    contacts = [["Left", 320, 10, "board"], ["Right", 200, 12, "board"], ["Center", 8, 240, "board"],
                ["Left", 6, 300, "board"], ["Right", 630, 250, "edge"], ["Center", 320, 470, "edge"],
                ["Left", 400, 8, "edge"]]
    r = types.SimpleNamespace(st={"setup": {"explore_live": {"contacts": contacts}}},
                              feed=types.SimpleNamespace(last_img=np.zeros((480, 640), np.uint8)))
    w = E.Runner.board_words_from_contacts(r)
    assert {w["x"], w["y"]} == {"top", "left"}
    assert E.Runner.explore_board_sides(r, {0: "bottom", 1: "top"}, {0: "right", 1: "left"}) == (1, 1)
    r.st["setup"]["explore_live"]["contacts"] = contacts[:1]
    assert E.Runner.board_words_from_contacts(r) is None


def test_a_silent_robot_is_not_a_bad_link():
    """Review: a robot switched off, asleep or flat answers nothing; that must not pause the others."""
    g = LNK.Gauge()
    now = 1000.0
    for k in range(240):
        g.add(True, "Left", now - 250 + k)
        g.add(True, "Center", now - 250 + k)
        g.add(k < 100, "Right", now - 250 + k)               # Right went silent 150 s ago
    assert g.silent(now) == {"Right"} and g.level(now) == "ok"


# =============================================================================== the 09:55 session analysis's fixes
def test_reporting_fixes():
    import e02_cpulog as CL
    import e02_framesaver as FS
    assert CL.parse_arm_clock("frequency(48)=1500398464") == 1500 and CL.parse_arm_clock("") is None
    assert FS.kind_of_mark("e02 near-robot stop Left") == "near-robot stop"
    assert FS.kind_of_mark("e02 watchdog stop Left") == "edge stop"
    r = types.SimpleNamespace()
    E.Runner.keep_tracker_stats(r, types.SimpleNamespace(stats={"both": 10, "none": 2}))
    E.Runner.keep_tracker_stats(r, types.SimpleNamespace(stats={"both": 5, "lock": 1}))
    assert r.tracker_stats_before == {"both": 15, "none": 2, "lock": 1}


def test_a_put_aside_robot_in_the_find_corner_moves_the_finds_out():
    r = types.SimpleNamespace(board=PL.Board(xboard_side=1, yboard_side=1), st={}, said=[], marks=[])
    r.say = lambda m, quiet=False: r.said.append(m)
    r.mark = lambda t: r.marks.append(t)
    fx, fy = r.board.find_stage()
    assert E.Runner.free_find_corner(r, (fx, fy)) is True             # the robot sits at the usual staging point
    assert r.board.find_stage_cm == 50.0 and r.st["find_stage_cm"] == 50.0
    nx, ny = r.board.find_stage()
    assert math.hypot(nx - fx, ny - fy) > 30.0
    assert "set e02_find_stage_cm 50" in E.Runner.board_params(r)
    r2 = types.SimpleNamespace(board=PL.Board(xboard_side=1, yboard_side=1), st={}, say=lambda *a, **k: None,
                               mark=lambda t: None)
    nx2, ny2 = r2.board.uv(50.0, 50.0)
    assert E.Runner.free_find_corner(r2, (nx2, ny2)) is False        # it sits at the other one too: no
