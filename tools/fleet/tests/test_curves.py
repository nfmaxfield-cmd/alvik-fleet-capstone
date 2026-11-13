"""Build e05 (5 Nov 2026, docs/build-reports/e05-nov05-curves.md): continuous curves (firmware 2026-11-05c follow step
R, farc, coast, dodge), the free roam's curve mode and its never-stops accounting, the motion choice in the planner /
missions / Mission Control, the curves' calibration found in the run, and the calibration's lost-tag recovery.

    python3 -m pytest tools/fleet/tests/test_curves.py -q
"""
import math
import os
import subprocess
import sys
import types

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


def place(c, name, x, y, th, t=0.0, fw="2026-11-05c"):
    c.t = t
    c.on_fw(name, fw)
    c.on_heard(name, t)
    c.on_battery(name, 100, t)
    c.on_follow(name, t, "idle", x, y, th, 0)
    c.on_fix(name, t, x, y, th)
    c.begin(name)


# -------------------------------------------------------------------------------------------- geometry, menu
def test_arc_geometry_and_loop_disc_reserved():
    x, y, h = FC.arc_at(50.0, 50.0, 0.0, 1 / 10.0, math.pi * 10 / 2)       # a quarter circle to the left
    assert abs(float(x) - 60.0) < 1e-6 and abs(float(y) - 60.0) < 1e-6 and abs(float(h) - 90.0) < 1e-6
    tr = FC.curve_traj(0.0, [(50.0, 50.0, 0.0, 0.0, 20.0, 10.0)], 1)
    assert tr.hi[-1] == FC.INF and tr.m[-1] >= FC.LOOP_R            # the loop-away disc, for ever
    assert abs(tr.loop_c[0] - 70.0) < 1e-6 and abs(tr.loop_c[1] - (50.0 + FC.LOOP_R)) < 1e-6
    far = FC.Traj.stand(500.0, 70.0, 50.0 + FC.LOOP_R)               # someone standing in that disc much later
    assert FC.clearance(tr, far) < FC.PLAN_CC


def test_curve_menu_balanced_and_wheel_capped():
    import random
    m = FC.CurveMenu(random.Random(4))
    assert len(m.cells) == 7 * 3 * 4
    for cell in m.cells:
        k, L, v = m.draw(cell)
        assert v * (1 + FC.HALF_TRACK * abs(k)) <= FC.WHEEL_TOP + 1e-6      # the outer wheel within its top speed
    seen = set()
    for _ in range(84):
        cell = m.order("Left")[0]
        m.use(cell, "Left")
        seen.add(cell)
    assert len(seen) == 84                                             # least-used first: every cell once


# -------------------------------------------------------------------------------------------- the controller
def test_curve_robot_drives_R_pieces_queued_ahead_old_robot_stop_turn():
    c, sent, rows = ctrl()
    place(c, "Left", 40, 40, 0.0)
    place(c, "Center", 85, 85, 180.0)
    place(c, "Right", 40, 90, -90.0, fw="2026-10-28a")
    c.tick(0.05)
    assert c.R["Left"].curve and c.R["Center"].curve and not c.R["Right"].curve
    lt = [t for n, t in sent if n == "Left" and t.startswith("fs ")]
    assert lt and lt[0].split()[2] == "R"
    rt = [t for n, t in sent if n == "Right" and t.startswith("fs ")]
    assert rt and rt[0].split()[2] in ("T", "D")
    ev = [r for tb, r in rows if tb == "events" and r["kind"] == "motion"]
    assert any("older than" in e["text"] for e in ev if e["robot"] == "Right")
    # the piece starts: the next one is planned and queued at once (they run into each other)
    sid = c.R["Left"].move["drive_id"]
    c.on_step("Left", sid, "queued")
    c.on_step("Left", sid, "started")
    c.tick(0.1)
    assert len(c.R["Left"].queue) == 1
    nxt = c.R["Left"].queue[0]
    assert math.dist((nxt["x"], nxt["y"]), c.R["Left"].move["end"]) < 1e-6     # starts where this one ends


def test_stopturn_style_keeps_the_old_moves():
    c, sent, _ = ctrl(motion="stopturn")
    place(c, "Left", 40, 40, 0.0)
    c.tick(0.05)
    assert not c.R["Left"].curve
    assert all(t.split()[2] in ("T", "D") for n, t in sent if t.startswith("fs "))


def test_dodge_is_not_a_stop_and_lost_report_of_a_piece_is_recovered():
    c, sent, rows = ctrl(("Left",))
    place(c, "Left", 60, 60, 0.0)
    c.tick(0.05)
    r = c.R["Left"]
    sid = r.move["drive_id"]
    c.on_step("Left", sid, "queued")
    c.on_step("Left", sid, "started")
    c.tick(0.1)
    nid = r.queue[0]["drive_id"]
    c.on_step("Left", nid, "queued")
    c.on_step("Left", nid, "started")            # the next piece started; the first one's "done" never came
    assert r.move["drive_id"] == nid and not r.queue
    c.on_step("Left", nid, "failed", "dodged obstacle 6.0 cm ahead; curving left at 4.0 cm/s")
    assert r.coast and r.coast["dodge"] and r.coast["k"] > 0
    c.tick(0.2)
    assert r.mode == "move" and r.move["drive_id"] > nid                      # a new piece at once, no stand
    assert c.counts["dodges"] == 1


def test_fastsim_curves_short_run_no_contact_and_moving():
    s = FS.Sim(3, 0.1, fw="2026-11-05c", dead_robot=False, pi_freeze=False)
    res = s.run()
    assert res["contacts"] == 0 and res["off_board"] == 0
    sp = res["speed"]
    assert all(v["style"] == "curve" for v in sp["robot"].values())
    assert all(v["moving_pct"] > 90.0 for v in sp["robot"].values()), sp
    assert sp["ge2_pct"] > 90.0
    assert res["pieces"] > 50


def test_fastsim_firmware_mix_counts_styles_apart():
    s = FS.Sim(2, 0.05, fw="2026-11-05c", fws={"Right": "2026-10-28a"}, dead_robot=False, pi_freeze=False)
    res = s.run()
    st = {n: v["style"] for n, v in res["speed"]["robot"].items()}
    assert st == {"Left": "curve", "Center": "curve", "Right": "stopturn"}
    assert res["contacts"] == 0
    # the stop-turn robot stops at every turn: many more speed-zero moments than the curve robots
    z = {n: v["zeros"] for n, v in res["speed"]["robot"].items()}
    assert z["Right"] > max(z["Left"], z["Center"])


def test_arc_fit_finds_a_curvature_error(monkeypatch):
    c, _, rows = ctrl(("Left",))
    place(c, "Left", 60, 60, 0.0)
    r = c.R["Left"]
    for j in range(16):
        k, L = 1 / 20.0, 30.0
        mv = dict(k=k, L=L, v=7.5, kind="menu")
        R = 20.0 / 1.05                             # it really turns 5 % tighter
        r.arc_pts = [(60 + R * math.sin(a), 60 + R - R * math.cos(a)) for a in [i * (L / 20.0) / 20 * 1.05
                                                                                for i in range(21)]]
        c.arc_fit(r, mv)
    kr, kh, dr, dh, n = c.arc_result(r)
    assert n == 16 and abs(kr - 1.05) < 0.01 and abs(dr - 1.0) < 0.06
    sent = []
    c.out_fn = lambda n_, t: sent.append(t)
    r.curve = True
    c.arc_save(r)          # (e18) curve scales are held until Nick decides (MIRLT_CURVE_SCALE_SAVE=0): logged only
    assert not any(t.startswith("set arc_k_scale") for t in sent)
    import freeroam_core as FC_
    monkeypatch.setattr(FC_, "CURVE_SCALE_SAVE", True)     # the e17 way, behind the switch
    c.arc_save(r)
    assert any(t.startswith("set arc_k_scale 0.95") for t in sent) and "save" in sent


# -------------------------------------------------------------------------------------------- firmware
def test_firmware_11_05c_curves_syntax_and_mirror():
    ino = open(os.path.join(REPO, "firmware", "home_lab", "home_lab.ino")).read()
    import re as _re
    assert _re.search(r'#define FW_VERSION "(2026-11-(?:05[c-z]|1[0-9][a-z0-9]*))"', ino) and 'verb == "farc"' in ino   # 11-05c or newer
    fol = open(os.path.join(REPO, "firmware", "home_lab", "follow.h")).read()
    assert "s.type == 'R'" in fol and "coast ended" in fol and "dodged obstacle" in fol
    pose = open(os.path.join(REPO, "firmware", "home_lab", "pose.h")).read()
    assert "int driveCurve(" in pose and "arc_dodge_r" in pose
    corr = open(os.path.join(REPO, "firmware", "home_lab", "corr.h")).read()
    assert '"arc_k_scale", "arc_d_scale"' in corr and "const int N = 5;" in corr
    out = subprocess.run(["sh", os.path.join(REPO, "tools", "fwcheck", "check.sh")], capture_output=True, text=True)
    assert out.stdout.strip() == "" and out.returncode == 0, out.stdout
    sim = open(os.path.join(REPO, "tools", "sim", "fw_follow.py")).read()
    assert "def drive_curve(" in sim and '"farc"' in sim


# -------------------------------------------------------------------------------------------- the planner's choice
def test_planner_curves_round_corners_only_with_room():
    import json
    import planner as P
    import fleetmap as FM
    m = FM.load(os.path.join(FLEET, "examples", "demo_map.json"))
    m = FM.normalize(dict(m, obstacles=[{"id": "post", "shape": "circle", "center": [60, 74], "radius": 6,
                                         "source": "user"}]))
    pl = P.Planner(m)
    a = [15.0, 15.0, 0.0]
    st = P._travel(a, (60, 95), pl.grid, P.settings({"motion": "stopturn"}), 0, "x")
    cv = P._travel(a, (60, 95), pl.grid, P.settings({"motion": "curves"}), 0, "x")
    assert sum(p["type"] == "turn" for p in st) == 2 and not any(p["type"] == "arc" for p in st)
    assert any(p["type"] == "arc" for p in cv) and sum(p["type"] == "turn" for p in cv) == 1
    for p in cv:
        if p["type"] == "arc":
            assert min(pl.grid.point_clear(q[:2]) for q in P.prim_poses(p, pl.s)) >= pl.grid.turn_thr - 0.05
    assert P.settings()["motion"] == "stopturn"
    try:
        P.settings({"motion": "zigzag"})
        assert False
    except P.PlanError:
        pass
    html = open(os.path.join(FLEET, "mission_control.html")).read()
    assert 'id="motion"' in html and "Curves at the corners" in html
    mr = open(os.path.join(FLEET, "mission_run.py")).read()
    # (e11) --motion's default is resolved after parsing (free roam's own default is curves): the planner's stays stopturn
    assert '"--motion", choices=("stopturn", "curves"), default=None' in mr and \
        'args.motion = args.motion or "stopturn"' in mr


def test_launcher_motion_words():
    sh = open(os.path.join(REPO, "run-fleet-pi.sh")).read()
    assert 'stopturn) LOST="$LOST --motion stopturn"' in sh
    fr = open(os.path.join(FLEET, "freeroam.py")).read()
    assert '"--motion", choices=("curves", "stopturn"), default="curves"' in fr


# -------------------------------------------------------------------------------------------- calibration recovery
def test_calib_recovery_steps_towards_a_reliable_spot_and_logs_the_jump():
    import calib_run as CRN
    cls = [v for v in vars(CRN).values() if isinstance(v, type) and hasattr(v, "recover")][0]
    run = cls.__new__(cls)
    legs = []
    seen_after = [2]

    class RB:
        def set(self, *a):
            pass

        def leg(self, kind, value, on_tick=None, timeout=0):
            legs.append((kind, value))
            x = sum(v for k, v in legs if k == "go")
            return dict(result="done", info="", start=(0, 0, 0, 0), end=((value, 0, 0, 0) if kind == "go" else
                                                                         (0, 0, value, 0)), id=len(legs))

    class Eye:
        def age(self, tid):
            return 0.1 if len([l for l in legs if l[0] == "go"]) >= seen_after[0] else 99.0

        def last(self, tid):
            return (0, 52.0, 60.0, 180.0)

    run.rb, run.eye, run.tag = RB(), Eye(), 2
    run.a = types.SimpleNamespace(lost_policy="recover")
    run.S = types.SimpleNamespace(problems=lambda a, b, buf=0, edge_buf=None: [])
    run.stop_flag = types.SimpleNamespace(is_set=lambda: False)
    run.reliable = [(40.0, 60.0), (90.0, 90.0)]
    run.recoveries, run._recovered = [], False
    run.D = None
    run.cam_pose = lambda dur=0: (52.5, 60.0, 180.0)
    run.ev = lambda *a, **k: None
    run.set_status = lambda **k: None
    run.fit = None
    run.baseline = "defaults"
    run.recover((70.0, 60.0, 0.0), 10.0, 0.0, "test")
    assert legs[0][0] == "turn" and abs(legs[0][1]) <= 170.0          # turns towards (40, 60) first
    assert [l for l in legs if l[0] == "go"] and all(v <= CRN.RECOVER_STEP_CM + 1e-9 for k, v in legs if k == "go")
    rec = run.recoveries[-1]
    assert rec["outcome"] == "seen again" and "jump_cm" in rec and run._recovered


def test_calib_bubble_bigger_uncalibrated_and_stops_at_a_margin():
    import calib_run as CRN
    cls = [v for v in vars(CRN).values() if isinstance(v, type) and hasattr(v, "recover")][0]
    run = cls.__new__(cls)
    run.fit, run.baseline = None, "defaults"
    b_uncal = run.bubble(30.0, 90.0)
    run.baseline = "saved"
    b_cal = run.bubble(30.0, 90.0)
    assert b_uncal > b_cal > CRN.BLIND_BASE_CM
    # a margin straight ahead: no step, a wiggle, then the old hold (made instant here)
    legs = []

    class RB:
        def set(self, *a):
            pass

        def leg(self, kind, value, on_tick=None, timeout=0):
            legs.append((kind, value))
            return dict(result="done", info="", start=(0, 0, 0, 0), end=(0, 0, value, 0), id=len(legs))
    run.rb, run.tag = RB(), 2
    run.eye = types.SimpleNamespace(age=lambda tid: 99.0)
    run.a = types.SimpleNamespace(lost_policy="recover")
    run.S = types.SimpleNamespace(problems=lambda a, b, buf=0, edge_buf=None: ["the edge margin"] if a != b else [])
    run.stop_flag = types.SimpleNamespace(is_set=lambda: False)
    run.reliable = [(40.0, 60.0)]
    run.recoveries, run._recovered = [], False
    run.D = None
    run.ev = lambda *a, **k: None
    run.set_status = lambda **k: None
    run.hold_until_seen = lambda: 1.5
    run.recover((70.0, 60.0, 180.0), 5.0, 0.0, "test")
    rec = run.recoveries[-1]
    assert rec["outcome"].startswith("stopped") and rec.get("wiggle") and rec["held_s"] == 1.5
    assert not [l for l in legs if l[0] == "go"]


def test_calib_lost_policy_default_recover_hold_kept():
    src = open(os.path.join(FLEET, "calib_run.py")).read()
    assert 'choices=["recover", "hold", "keep_going"], default=None' in src
    assert 'cal_conf.get("lost_policy") or "recover"' in src
    assert "def hold_until_seen" in src                 # the old hold: kept (switch and last resort)


def test_farc_answer_stops_its_resending():
    """Found in the stress runs: a farc was resent every 0.3 s for ever (its FSPD answer did not clear it), bending
    every later piece and dropping the queue (the robots then crowded and touched)."""
    c, sent, _ = ctrl(("Left",))
    place(c, "Left", 60, 60, 0.0)
    r = c.R["Left"]
    c.control(r, "farc 0.05000 25.0", "farc")
    c.on_fspd("Left")
    assert not [x for x in r.controls if x["kind"] == "farc"]
    c.control(r, "farc 0.05000 25.0", "farc")
    c.end_move(r)
    assert not [x for x in r.controls if x["kind"] == "farc"]
