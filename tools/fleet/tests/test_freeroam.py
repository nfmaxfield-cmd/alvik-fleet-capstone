"""Home free roam (2026-11-05, docs/build-reports/e04-nov05-home-free-roam.md): the controller (freeroam_core.py) and
its stress simulator (freeroam_fastsim.py), the firmware 2026-11-05b commands in the simulator, the launcher.

    python3 -m pytest tools/fleet/tests/test_freeroam.py -q
"""
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(FLEET))
for p in (FLEET, os.path.join(REPO, "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)

import freeroam_core as FC       # noqa: E402
import freeroam_fastsim as FS    # noqa: E402

BOARD = (0.0, 0.0, 122.24, 121.92)


def ctrl(names=("Left", "Center", "Right"), **kw):
    sent, rows = [], []
    c = FC.Controller(list(names), BOARD, lambda n, t: sent.append((n, t)), lambda tb, r: rows.append((tb, r)),
                      seed=3, **kw)
    return c, sent, rows


def place(c, name, x, y, th, t=0.0, fw="2026-11-05b"):
    c.t = t
    c.on_fw(name, fw)
    c.on_heard(name, t)
    c.on_battery(name, 100, t)
    c.on_follow(name, t, "idle", x, y, th, 0)
    c.on_fix(name, t, x, y, th)
    c.begin(name)


# -------------------------------------------------------------------------------------------- reservations
def test_crossing_reservations_respect_time():
    """Two moves through the same spot conflict only when their time windows overlap (+ slack)."""
    a = FC.Traj.move(0.0, (20, 60), 0.0, 0.0, 80, 10.0)          # along y = 60, x 20 -> 100, ~8.8 s
    b_same = FC.Traj.move(0.0, (60, 20), 90.0, 0.0, 80, 10.0)    # along x = 60 at the same time: meet at (60, 60)
    b_late = FC.Traj.move(30.0, (60, 20), 90.0, 0.0, 80, 10.0)   # 30 s later: the first one stands at (100, 60)
    assert FC.clearance(a, b_same) < FC.PLAN_CC
    assert FC.clearance(a, b_late) >= FC.PLAN_CC


def test_standing_end_is_reserved_for_ever():
    a = FC.Traj.move(0.0, (20, 60), 0.0, 0.0, 40, 10.0)          # ends standing at (60, 60)
    b = FC.Traj.move(500.0, (60, 20), 90.0, 0.0, 80, 10.0)       # long after: still through (60, 60)
    assert FC.clearance(a, b) < FC.PLAN_CC


# -------------------------------------------------------------------------------------------- the menu
def test_menu_least_used_first_and_seeded():
    import random
    m1, m2 = FC.Menu(random.Random(5)), FC.Menu(random.Random(5))
    assert m1.order("Left") == m2.order("Left")                  # seeded: the same run twice is the same
    c0 = m1.order("Left")[0]
    m1.use(c0, "Left")
    assert c0 not in m1.order("Left")[:len(m1.cells) - 1]        # a used cell goes to the back
    assert len(m1.cells) == len(FC.TURN_BINS) * len(FC.LEN_BINS) * len(FC.SPEED_BINS)


def test_moves_keep_the_edge_and_cover_the_menu():
    c, sent, rows = ctrl(names=("Left",))
    place(c, "Left", 61, 61, 0.0)
    for k in range(60):
        c.tick(k * 0.05)
        r = c.R["Left"]
        assert r.move is not None
        e = r.move["end"]
        assert FC.edge_dist(BOARD, *e) >= FC.EDGE_CC - 1e-6
        h = r.move["h1"]
        c.end_move(r)                                            # pretend it got there
        c.on_follow("Left", c.t, "idle", e[0], e[1], h, 0)     # (its own pose first: the camera sees it there after)
        c.on_fix("Left", c.t, e[0], e[1], None)
        r.busy_until = 0.0
    used = sum(1 for v in c.menu.n.values() if v)
    assert used >= 40                                            # 60 moves spread over the cells, not repeated


def test_fallback_never_waits_and_holding_needs_room():
    """Boxed in by two robots standing close: no menu move; the fallback is a move (escape or yield) or a holding
    turn, never a wait (Nick 11:23); a holding turn only with its own circle clear."""
    c, sent, rows = ctrl()
    place(c, "Left", 20, 20, 45.0)
    place(c, "Center", 41, 20, 180.0)
    place(c, "Right", 20, 41, -90.0)
    res = c.pick(c.R["Left"])
    assert res is not None and res[0] in ("off-menu", "escape", "holding", "yield", "menu", "free")
    fb = [r for t, r in rows if t == "fallbacks"]
    assert all("wait" not in f["kind"] for f in fb)


# -------------------------------------------------------------------------------------------- the reflex
def _driving(c, name, x, y, th, end, v=10.0):
    r = c.R[name]
    c.on_follow(name, c.t, "drive", x, y, th, 1)
    r.move = dict(kind="menu", cell=None, turn=0.0, L=math.dist((x, y), end), v=v, t0=c.t, p0=(x, y), h0=th, h1=th,
                  end=end, phase="drive", n=1, drive_id=1, turn_id=None, t_drive=c.t, adj_scale=1.0)
    r.mode = "move"
    r.steps[1] = dict(text="fs 1 D", status="started", t_sent=c.t)
    r.next_id = 2
    r.traj = FC.Traj.move(c.t, (x, y), th, 0.0, math.dist((x, y), end), v)


def test_reflex_speed_change_against_a_crossing_robot():
    c, sent, rows = ctrl(names=("Left", "Right"))
    place(c, "Left", 30, 60, 0.0)
    place(c, "Right", 60, 30, 90.0)
    c.t = 1.0
    for n in ("Left", "Right"):
        c.on_heard(n, 1.0)
    c.on_fix("Left", 1.0, 30, 60, 0.0)
    c.on_fix("Right", 1.0, 60, 30, 90.0)
    _driving(c, "Left", 30, 60, 0.0, (100, 60))
    _driving(c, "Right", 60, 30, 90.0, (60, 100))
    c.reflex()
    ev = [r for r in c.R.values() if r.adj is not None]
    assert ev, "a crossing at the same time must make the reflex act"
    a = ev[0]
    assert a.name == "Right"                                    # the lower-priority mover acts
    assert a.adj["kind"] in ("speed", "steer")
    assert any(t.startswith(("fspd", "fsteer")) for n, t in sent if n == "Right")


def test_reflex_ignores_robots_moving_apart():
    c, sent, rows = ctrl(names=("Left", "Right"))
    place(c, "Left", 50, 60, 180.0)
    place(c, "Right", 64, 60, 0.0)                              # 14 cm apart, driving away from each other
    c.t = 1.0
    c.on_fix("Left", 1.0, 50, 60, 180.0)
    c.on_fix("Right", 1.0, 64, 60, 0.0)
    _driving(c, "Left", 50, 60, 180.0, (20, 60))
    _driving(c, "Right", 64, 60, 0.0, (100, 60))
    c.reflex()
    assert all(r.adj is None for r in c.R.values())
    assert c.counts["breaks"] == 0


def test_reflex_never_slows_into_a_standing_robot():
    """Slowing towards a robot that stands only delays the meeting: steer or a new move instead."""
    c, sent, rows = ctrl(names=("Left", "Right"))
    place(c, "Left", 30, 60, 0.0)
    place(c, "Right", 52, 60, 0.0)
    c.t = 1.0
    c.on_fix("Left", 1.0, 30, 60, 0.0)
    c.on_fix("Right", 1.0, 52, 60, 0.0)
    c.on_follow("Right", 1.0, "idle", 52, 60, 0.0, 0)
    _driving(c, "Left", 30, 60, 0.0, (100, 60))
    c.reflex()
    kinds = [r["kind"] for t, r in rows if t == "reflex"] + [r.adj["kind"] for r in c.R.values() if r.adj]
    assert kinds and "speed" not in kinds


def test_old_firmware_uses_halt_and_resend():
    c, sent, rows = ctrl(names=("Left", "Right"))
    place(c, "Left", 30, 60, 0.0, fw="2026-10-29b")
    place(c, "Right", 60, 30, 90.0, fw="2026-10-29b")
    c.t = 1.0
    c.on_fix("Left", 1.0, 30, 60, 0.0)
    c.on_fix("Right", 1.0, 60, 30, 90.0)
    _driving(c, "Left", 30, 60, 0.0, (100, 60))
    _driving(c, "Right", 60, 30, 90.0, (60, 100))
    c.reflex()
    assert not any(t.startswith(("fspd", "fsteer")) for n, t in sent)
    assert any(t == "halt" for n, t in sent)


# -------------------------------------------------------------------------------------------- lost by the camera
def test_dead_reckoning_bubble_grows_and_jump_is_logged(monkeypatch):
    # (e21) this tests the e20 mechanism on its own numbers; the e21 way: test_e21_error_shape.py
    monkeypatch.setattr(FC, "SHAPE", False)
    c, sent, rows = ctrl(names=("Left",))
    place(c, "Left", 60, 60, 0.0)
    c.t = 2.0
    c.tick(2.0)                                                 # no fix for 2 s: lost
    r = c.R["Left"]
    assert r.lost is not None
    for k in range(1, 41):                                       # drives 40 cm on its own pose
        c.on_follow("Left", 2.0 + k * 0.1, "drive", 60 + k, 60, 0.0, 1)
    assert abs(r.bubble - (FC.BLIND_BASE + FC.BLIND_PER_CM * 40)) < 0.2
    c.t = 7.0
    c.on_fix("Left", 7.0, 103.0, 60.5, None)                     # camera: 3 cm further than its odometry
    assert r.lost is None
    lost = [x for t, x in rows if t == "lost"]
    assert lost and abs(lost[0]["jump_cm"] - math.hypot(3.0, 0.5)) < 0.05
    assert any(t.startswith("fpose") for n, t in sent)          # over REACQUIRE_CM: the pose set to the camera's


def test_hold_mode_still_available():
    c, sent, rows = ctrl(names=("Left",), lost_mode="hold")
    place(c, "Left", 60, 60, 0.0)
    _driving(c, "Left", 60, 60, 0.0, (100, 60))
    c.t = 2.0
    c.tick(2.0)
    assert any(t == "halt" for n, t in sent)
    assert c.counts["breaks"] == 1


# -------------------------------------------------------------------------------------------- battery
def test_battery_floor_parks_and_sleeps():
    c, sent, rows = ctrl(names=("Left",))
    place(c, "Left", 60, 60, 0.0)
    c.on_battery("Left", 10, 0.0)
    c.tick(0.1)
    r = c.R["Left"]
    assert r.mode == "parking"
    c.on_step("Left", r.move["turn_id"] or r.move["drive_id"], "queued")
    sid = r.move["drive_id"]
    c.on_step("Left", sid, "done")
    assert r.mode == "out"
    assert ("Left", "stop") in sent and ("Left", "sleep 0") in sent


# -------------------------------------------------------------------------------------------- stress simulator
def test_fastsim_quarter_hour_no_contact():
    s = FS.Sim(4, 0.25)
    res = s.run()
    assert res["contacts"] == 0
    assert res["moves"] > 80
    assert res["off_board"] == 0
    assert res["motion"]["any_ge2"] > 60.0


def test_fastsim_pi_freeze_stops_robots_by_watchdog():
    s = FS.Sim(5, 0.2, dead_robot=False)
    s.freeze_at = 120.0
    res = s.run()
    assert res["watchdog_stops"] >= 1
    assert res["contacts"] == 0


# -------------------------------------------------------------------------------------------- firmware (simulated)
def test_sim_firmware_fspd_fsteer():
    """The simulated firmware (tools/sim/fw_follow.py) runs fspd / fsteer on 2026-11-05b and treats them as unknown
    on older firmware; the real firmware passes the syntax check."""
    src = open(os.path.join(REPO, "tools", "sim", "fw_follow.py")).read()
    assert '"fspd", "fsteer"' in src and "follow_wd_ms" in src
    ino = open(os.path.join(REPO, "firmware", "home_lab", "home_lab.ino")).read()
    assert '#define FW_VERSION "2026-11-' in ino   # 11-05b or newer (11-05c: curves keep fspd / fsteer)
    follow = open(os.path.join(REPO, "firmware", "home_lab", "follow.h")).read()
    assert 'verb == "fspd"' in follow and '"watchdog"' in follow
    r = subprocess.run(["sh", os.path.join(REPO, "tools", "fwcheck", "check.sh")], capture_output=True, text=True)
    assert r.returncode == 0 and not r.stdout.strip(), r.stdout


def test_launcher_has_freeroam_and_stopall_knows_it():
    sh = open(os.path.join(REPO, "run-fleet-pi.sh")).read()
    for w in ("freeroam)", "freeroam-run)", "freeroam-finish)", "freeroam-status)", "freeroam-stop)"):
        assert w in sh
    import stop_all
    assert "freeroam" in stop_all.TMUX_RUNS
    assert subprocess.run(["bash", "-n", os.path.join(REPO, "run-fleet-pi.sh")]).returncode == 0


def test_old_roam_still_parses():
    import roam
    assert hasattr(roam, "Roamer") and callable(roam.main)


def test_lost_step_report_seen_in_follow():
    """A WiFi drop: the watchdog failed the step and its report was lost. FOLLOW's last_failed ends the move after
    0.5 s instead of the slow-move timeout (rehearsal 5 Nov: 37 and 47 s standing)."""
    c, sent, rows = ctrl(names=("Left",))
    place(c, "Left", 60, 60, 0.0)
    c.tick(0.1)
    r = c.R["Left"]
    sid = r.move["drive_id"] or r.move["turn_id"]
    c.on_step("Left", sid, "queued")
    c.on_step("Left", sid, "started")
    c.on_follow("Left", 1.0, "idle", 60, 60, 0.0, 0, 0, sid)
    assert r.move is not None                                   # not yet: its own report may still come
    c.on_follow("Left", 1.6, "idle", 60, 60, 0.0, 0, 0, sid)
    assert r.move is None
    assert any(x["kind"] == "lost report" for t, x in rows if t == "events")


def test_reflex_drill_refused_on_the_robots():
    import freeroam
    assert freeroam.main(["run", "--reflex-drill", "0.3", "--robot", "192.168.1.250"]) == 2


def test_report_counts_stand_stills():
    import freeroam_report as FR
    mot = []
    for k in range(40):
        t = "%.1f" % (k * 0.5)
        still = 10 <= k < 20                                     # 5 s without motion
        mot.append(dict(t=t, robot="Left", any=0 if still else 1, translating=0 if still else 1, mode="move"))
    ms = FR.motion_stats(mot)
    assert ms["robot"]["Left"]["still_over_3s"] == 1 and abs(ms["robot"]["Left"]["longest_still_s"] - 5.0) < 0.01


# -------------------------------------------------------------------------------------------- the first real run (11-05c)
# 5 Nov 14:35, home Pi, firmware 2026-10-28a, 17 min (release freeroam-20261105-143553-freeroam): fixes arrived ~0.45 s
# old so every short gap read as "lost"; a stale camera spot was sent as fpose; the camera put Center 45 cm off (on
# Right's spot) and the controller believed it; Left's own front sensor stopped 515 moves in a row (each turn step
# refused: something 2.9 cm ahead); Right stood 3 min 7 cm from an edge with no turn room ("stand" 186 + 182 times).
def _moving_robot(c, name, t0, t1, v=10.0, delay=1.0, clock=0.7, y=60.0, x0=30.0):
    """Own poses every 0.1 s (driving along +x, speed changing between 0 and 2v every 4 s); camera fixes every 0.08 s,
    stamped on a clock `clock` ahead, arriving `delay` s after the picture. Returns the true x at t1."""
    def X(tt):
        return x0 + v * (tt - t0) - v * 4.0 / (2 * math.pi) * math.sin(2 * math.pi * (tt - t0) / 4.0)
    t = t0
    nxt_fix = t0
    pend = []
    while t <= t1 + 1e-9:
        c.on_heard(name, t)
        c.on_follow(name, t, "drive", X(t), y, 0.0, 1)
        if t >= nxt_fix:
            pend.append((t + delay, t, X(t)))
            nxt_fix = t + 0.08
        for due, tp, xp in [q for q in pend if q[0] <= t]:
            c.on_fix(name, tp + clock, xp, y, None, arrival=t)
        pend = [q for q in pend if q[0] > t]
        c.tick(t)
        t = round(t + 0.05, 4)
    return X(t1)


def test_camera_delay_and_clock_measured_no_false_lost():
    """Real failure: fixes ~0.45 s old (here 1.0 s, on a clock 0.7 s ahead) made the controller call every robot
    lost several times a second and compare stale fixes with its pose NOW. Now: the delay and the clock are measured
    online, no false 'lost', and the position used is the camera's moved on by its own motion (within 1.5 cm)."""
    c, sent, rows = ctrl(names=("Left",))
    place(c, "Left", 30, 60, 0.0)
    r = c.R["Left"]
    r.mode = "out"                                   # no planning: only the timing is under test
    xt = _moving_robot(c, "Left", 0.05, 32.0)
    ev = [x for t, x in rows if t == "events"]
    assert abs(c.clock - 0.7) < 0.1, c.clock
    assert abs(c.cam_delay - 1.0) < 0.1, c.cam_delay
    late = [e for e in ev if e["kind"] == "lost" and e["t"] > 8.0]
    assert not late, late[:3]
    p = c.pos(r)
    assert abs(p[0] - xt) < 1.5 and abs(p[1] - 60.0) < 0.5, (p, xt)


def test_reacquire_compares_at_the_pictures_time_and_projects_fpose():
    """Real failure: 'seen again' compared a 0.45 s old fix with the pose now (jumps up to 45 cm) and sent that stale
    spot as fpose. Now the jump is measured at the picture's time and the fpose is moved on to now."""
    c, sent, rows = ctrl(names=("Left",))
    place(c, "Left", 30, 60, 0.0)
    r = c.R["Left"]
    r.mode = "out"
    _moving_robot(c, "Left", 0.05, 32.0)            # delay and clock learned
    assert abs(c.clock - 0.7) < 0.1
    c.start_lost(r)
    # 3 s blind from x 130 on its own pose; its own pose drifts 4 cm sideways while it drives on at 10 cm/s
    for k in range(1, 31):
        tt = 40.0 + k * 0.1
        c.on_follow("Left", tt, "drive", 130 + k, 60 + 4.0 * k / 30, 0.0, 1)
        c.t = tt
    # a fix of the picture taken at 42.0 (camera clock +0.7), arriving at 43.0: truth at 42.0 was x 150, y 60
    c.t = 43.0
    c.on_fix("Left", 42.0 + 0.7, 150.0, 60.0, None, arrival=43.0)
    lost = [x for t, x in rows if t == "lost"][-1]
    assert abs(lost["jump_cm"] - 2.67) < 0.4, lost            # own at 12.0 was (150, 62.67): 2.67 cm, not ~10
    fp = [t for n, t in sent if t.startswith("fpose")][-1].split()
    assert abs(float(fp[1]) - 160.0) < 0.6 and abs(float(fp[2]) - 61.33) < 0.6, fp   # moved on to now


def test_camera_on_another_robot_is_not_believed():
    """Real failure: the camera put Center on Right's spot, 45 cm from where Center's wheels had it; three agreeing
    fixes were enough and Center's pose was set there (it then stood 'near the edge' for the rest of the run, and
    Left drove into the real Center). Now: a far spot must move as the wheels do, and a spot where another robot
    stands is refused."""
    c, sent, rows = ctrl(names=("Center", "Right"))
    place(c, "Center", 30, 60, 90.0)
    place(c, "Right", 27, 115, 90.0)
    c.R["Center"].mode = c.R["Right"].mode = "out"
    c.start_lost(c.R["Center"])
    for k in range(1, 41):                            # Center drives on +y at 6.5 cm/s; the camera says Right's spot
        tt = k * 0.1
        c.t = tt
        c.on_follow("Center", tt, "drive", 30, 60 + 0.65 * k, 90.0, 1)
        c.on_follow("Right", tt, "idle", 27, 115, 90.0, 0)
        c.on_fix("Right", tt, 27.0, 115.0, None)
        c.on_fix("Center", tt, 26.9 + 0.1 * (k % 2), 115.5, None)
    assert c.R["Center"].lost is not None
    assert not any(n == "Center" and t.startswith("fpose") for n, t in sent)
    assert any(x["kind"] == "camera disagrees" for t, x in rows if t == "events")


def test_front_stop_backs_off_and_discovers_a_keepout():
    """Real failure: Left's own front sensor stopped 515 moves in a row (2.9 cm ahead; turn steps refused), the
    planner retrying from the same spot. Now: the next move backs straight away (a reverse drive, no front check),
    the felt point is a keep-out the planner respects, and no move after it heads into it."""
    c, sent, rows = ctrl(names=("Left",))
    place(c, "Left", 18.7, 62.8, 92.6, fw="2026-10-28a")
    c.t = 0.3
    c.tick(0.3)
    r = c.R["Left"]
    sid = r.move["turn_id"] or r.move["drive_id"]
    c.on_step("Left", sid, "failed", "obstacle 2.9 cm ahead; zones 4.0 2.9 2.8 2.7 3.7")
    ko = c.keepouts
    assert len(ko) == 1 and abs(ko[0]["x"] - 18.4) < 0.6 and abs(ko[0]["y"] - 70.4) < 0.6, ko
    c.t = 0.6
    c.on_fix("Left", 0.6, 18.7, 62.8, 92.6)     # (11-05h) a reverse only with a fresh camera position (protection 4)
    c.tick(0.6)
    assert r.move["kind"] == "back off"
    d = [t for n, t in sent if t.startswith("fs ") and " D " in t][-1].split()
    assert d[-2] == "1", d                                        # rev 1: backwards
    assert float(d[6]) < 62.8 - 5.0                               # ends further from the thing it felt
    for k in range(40):                                           # every later plan keeps out of it
        c.end_move(r)
        c.t = 1.0 + k
        c.on_follow("Left", c.t, "idle", float(d[5]), float(d[6]), 92.6, 0)
        c.on_fix("Left", c.t, float(d[5]), float(d[6]), None)
        r.busy_until = 0
        c.tick(c.t)
        if r.move is None:
            continue
        assert c.keepout_ok(r, r.traj)


def test_edge_trap_reverses_out():
    """Real failure: Right 6.6 cm from the y_max edge facing out: no turn fits, so it stood ('stand') for the rest of
    the run. Now it reverses inward, then moves on."""
    c, sent, rows = ctrl(names=("Right",))
    place(c, "Right", 27.0, 115.4, 87.0, fw="2026-10-28a")
    c.t = 0.3
    c.tick(0.3)
    r = c.R["Right"]
    assert r.move is not None and r.move["kind"] == "reverse", [x for t, x in rows if t == "fallbacks"]
    d = [t for n, t in sent if " D " in t][-1].split()
    assert d[-2] == "1" and float(d[6]) < 115.4 - 5.0


def test_fastsim_real_run_conditions_no_loops_no_contact():
    """The 5 Nov conditions in the stress simulator: camera 1 s late on a clock 0.7 s ahead, a wall 5 cm inside the
    believed x_min edge, Right starting trapped 7 cm from an edge facing out, firmware 2026-10-28a."""
    s = FS.Sim(2, 0.2, dead_robot=False, pi_freeze=False, cam_delay=1.0, cam_clock=0.7, wall_in=5.0, trap=True,
               fw="2026-10-28a")
    res = s.run()
    assert res["contacts"] == 0
    assert res["front_loops"] == 0
    assert res["wall_hits"] == 0
    assert max(res["longest_still_s"].values()) < 30.0, res["longest_still_s"]
    assert abs(res["clock_s"] - 0.7) < 0.1
