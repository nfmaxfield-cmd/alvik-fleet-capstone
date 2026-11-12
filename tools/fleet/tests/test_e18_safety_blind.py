"""Build e18 (12 Nov, Nick's decisions 1-7): the out-and-back identity drive, never still everywhere (missions and
calibration too) with every safety switch in ONE Safety group, a doubtful heading beside a known / unknown neighbour,
lost robots first, each robot's odometry trust and the blind-drive allowance, every calibration correction saved at any
run end. Simulator and saved logs only: no robot, no Pi.

    python3 -m pytest tools/fleet/tests/test_e18_safety_blind.py -q
"""
import json
import math
import os
import re
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)

import e17_harness as H          # noqa: E402
import freeroam_core as FC       # noqa: E402
import odo_trust as OT           # noqa: E402
import proven_workspace as PW    # noqa: E402
import safety_group as SG        # noqa: E402
import tracking_layer as TL      # noqa: E402

DARK = (70.0, 45.0, 100.0, 80.0)


# ---------------------------------------------------------------------------------------------------- 2: the group
def test_every_safety_switch_the_safety_code_reads_is_in_the_one_group():
    """The Safety group is the one place: every switch read through SG in the safety code is listed, and none of the
    listed switches is read past it (a second default could drift apart)."""
    files = ["freeroam_core.py", "safety_rules.py", "tracking_layer.py", "run_plan.py", "mission_run.py",
             "calib_run.py", "freeroam.py"]
    used = set()
    for f in files:
        txt = open(os.path.join(FLEET, f)).read()
        used |= set(re.findall(r'SG_?\.(?:flag|value)\("(MIRLT_[A-Z0-9_]+)"', txt))
        for env in SG.DEFAULT:
            bad = re.findall(r'os\.environ\.get\("%s"' % env, txt)
            if f in ("freeroam_core.py", "safety_rules.py", "tracking_layer.py"):
                assert not bad, "%s reads %s past the Safety group" % (f, env)
    assert used and used <= set(SG.DEFAULT), used - set(SG.DEFAULT)
    for env in ("MIRLT_NEVER_STILL", "MIRLT_OUTBACK_ID", "MIRLT_DOUBT_NEIGHBOUR", "MIRLT_LOST_PRIORITY",
                "MIRLT_BLIND_ALLOW", "MIRLT_SAVE_CAL_AT_END", "MIRLT_REJOIN_EVIDENCE", "MIRLT_EDGE_BACKOUT",
                "MIRLT_BLIND_RULE"):
        assert env in SG.DEFAULT
    assert "MIRLT_NEVER_STILL" in SG.doc() and "| Switch |" in SG.doc()


def test_the_group_is_automatic_by_default_and_the_old_way_is_one_switch(monkeypatch, tmp_path):
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    monkeypatch.setattr(SG, "_FILE", None)
    for env in SG.DEFAULT:
        monkeypatch.delenv(env, raising=False)
    assert SG.changed() == [] and "automatic default" in SG.start_words()
    monkeypatch.setenv("MIRLT_NEVER_STILL", "0")
    assert SG.flag("MIRLT_NEVER_STILL") is False and SG.changed() == [("MIRLT_NEVER_STILL", "0", "1")]
    monkeypatch.delenv("MIRLT_NEVER_STILL")
    # Mission Control's Safety box: written to the Pi's safety.json, read by every run there; "auto" writes nothing
    assert SG.write_file(SG.settings_defaults()) is None
    st = dict(SG.settings_defaults(), MIRLT_OUTBACK_ID="0")
    p = SG.write_file(st)
    assert p and json.load(open(p))["switches"] == {"MIRLT_OUTBACK_ID": "0"}
    assert SG.flag("MIRLT_OUTBACK_ID") is False and SG.flag("MIRLT_NEVER_STILL") is True
    monkeypatch.setenv("MIRLT_OUTBACK_ID", "1")              # the environment wins for one run
    assert SG.flag("MIRLT_OUTBACK_ID") is True
    assert SG.env_from_settings(st) == {"MIRLT_OUTBACK_ID": "0"}


def test_mission_control_has_one_safety_box_and_passes_it_to_the_runs(tmp_path, monkeypatch):
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    import mission_control as MC
    st = MC.Session(str(tmp_path))
    s = st.state()
    assert set(s["defaults"]["safety"]) == set(SG.DEFAULT) and all(v == "auto" for v in s["settings"]["safety"].values())
    assert any(q["env"] == "MIRLT_NEVER_STILL" for q in s["safety_info"])
    assert st.safety_env() is None                          # every switch on auto: the run gets the Pi's own environment
    st.settings["safety"]["MIRLT_NEVER_STILL"] = "0"
    assert st.safety_env()["MIRLT_NEVER_STILL"] == "0"
    html = open(os.path.join(FLEET, "mission_control.html")).read()
    assert 'settingsTable("safety")' in html and "safetyNote()" in html


def test_core_layout_b_is_still_the_default_and_documented_as_nicks_choice():
    sys.path.insert(0, os.path.join(TOOLS, "pi"))
    import core_layout as CL
    assert CL.DEFAULT == "B" and "Nick chose B" in open(CL.__file__).read()
    txt = open(os.path.join(REPO, "docs", "how-to", "pi-bench.md")).read()
    assert "Nick chose B" in txt


# ------------------------------------------------------------------------------- 1: the out-and-back identity drive
TWO = {"Left": (50.0, 60.0, 0.0), "Center": (95.0, 100.0, 90.0), "Right": (50.0, 80.0, 180.0)}
OFFS = {"Left": (30.0, -10.0, 0.0), "Right": (30.0, 10.0, 0.0)}


def _two_unaccounted(truth, offs=OFFS, secs=30.0, **kw):
    R = H.Rig(truth, own_off=offs, lights_off_cm=0.0, spin_flag=False, seed=3, **kw)   # the home lights: no turn seen
    R.run(0.1)
    for n in ("Left", "Right"):
        R.c.start_lost(R.c.R[n], "test: the camera lost it")
    R.run(secs)
    return R


def test_two_robots_unaccounted_standing_near_each_other_are_told_apart_by_the_out_and_back_without_a_swap():
    R = _two_unaccounted(TWO)
    assert R.first("identity wiggle", "Left") is not None                  # the wiggle first ...
    assert any(l.get("what") == "wiggle showed nothing" for l in R.logs)     # ... the lights cannot see a turn
    assert R.first("identity drive", "Left") is not None                   # ... then the out-and-back
    assert R.layer.counts["outback_named"] == 1
    assert R.gap("Left") < 3.0 and R.gap("Right") < 3.0                     # each named on its own spot: no swap
    assert "correction revoked" not in R.kinds()
    out = [s for s in R.sent if s[1] == "Left" and s[2].startswith("fs ") and " D " in s[2]]
    assert len(out) >= 2                                                     # out, and back (reverse)
    L = math.hypot(float(out[0][2].split()[5]) - float(out[0][2].split()[3]),
                   float(out[0][2].split()[6]) - float(out[0][2].split()[4]))
    assert 2.0 - 1e-6 <= L <= 4.0 + 1e-6                                     # Nick: 2-4 cm


def test_the_out_and_back_length_follows_the_measured_camera_noise():
    L = TL.TrackingLayer(["A", "B"], ("lights",))
    small = L._outback_size([dict(sd0=0.2), dict(sd0=0.3)])
    big = L._outback_size([dict(sd0=0.9), dict(sd0=0.3)])
    huge = L._outback_size([dict(sd0=3.0)])
    assert small["L"] == 2.0 and 2.0 < big["L"] <= 4.0 and huge["L"] == 4.0
    assert 2.0 <= small["v"] <= 5.0 and small["thr"] <= small["L"]


def test_too_close_for_any_identity_move_nothing_is_guessed():
    near = {"Left": (50.0, 60.0, 0.0), "Center": (95.0, 100.0, 90.0), "Right": (50.0, 74.5, 180.0)}   # 14.5 cm
    R = _two_unaccounted(near, secs=20.0)
    assert R.first("CORRECTED") is None
    assert R.layer.counts["outback_named"] == 0
    assert any("refused" in str(l.get("what")) or "unsure" in str(l.get("what")) for l in R.logs)


def test_the_switch_keeps_e17s_wiggles_only(monkeypatch):
    R = _two_unaccounted(TWO, secs=20.0, layer_kw=dict(outback=False))
    assert R.first("identity drive") is None and R.layer.counts["outback_named"] == 0
    assert R.first("CORRECTED") is None                                     # the wiggle alone cannot tell them


# ------------------------------------------------------------------------------- 2: never still in missions too
def _held_coord():
    import run_plan as RP
    import test_keep_going as KG
    mm, plan = KG.production_plan()
    c = KG.coord(mm, plan)
    b = c.bots["Alpha"]
    b.pose = (80.0, 120.0, 0.0)
    c.bots["Bravo"].pose = (320.0, 120.0, 0.0)
    c.retire(b, "test: a blind stop", kind="restarted")
    b.last_heard = RP.now()
    c.blind_after = lambda b_, base=None: 0.6
    return RP, c, b


def test_a_held_robot_in_a_mission_creeps_and_never_stands(monkeypatch):
    RP, c, b = _held_coord()
    monkeypatch.setattr(RP, "NEVER_STILL", True)
    t = [RP.now()]
    monkeypatch.setattr(RP, "now", lambda: t[0])
    sends = []
    for k in range(60):                                    # 30 s at 2 ticks a second; it answers, its pose follows
        t[0] += 0.5
        b.last_heard = t[0]
        n0 = len(c.sent)
        c.held_creep_watch()
        for _, txt in c.sent[n0:]:
            if txt.startswith("fs ") and " D " in txt:
                q = txt.split()
                b.pose = (float(q[5]), float(q[6]), float(q[7]))
                sends.append((t[0], b.pose))
    assert len(sends) >= 8                                 # a creep every few seconds, never an indefinite stop
    gaps = [b_[0] - a_[0] for a_, b_ in zip(sends, sends[1:])]
    assert max(gaps) <= 0.6 + 0.5 + 8.0 / 2.0 + 1.0 + 0.6   # the derived pause + one creep's own time
    assert all(math.dist(p[:2], b.hold_at) <= RP.CREEP_REACH_CM + 1e-6 for _, p in sends)   # inside its held outline
    ob = c.out_obstacle(b)
    assert ob["gap_cm"] >= RP.CREEP_REACH_CM               # the others plan round its whole reach
    assert any(e[0] == "held_creep" for e in c.logged)


def test_the_switch_keeps_the_mission_hold_still(monkeypatch):
    RP, c, b = _held_coord()
    monkeypatch.setattr(RP, "NEVER_STILL", False)
    for _ in range(10):
        c.held_creep_watch()
    assert not any(txt.startswith("fs ") for _, txt in c.sent if _ == "Alpha")


def test_calibration_holds_creep_too():
    txt = open(os.path.join(FLEET, "calib_run.py")).read()
    assert "def hold_creep" in txt and 'SG_.flag("MIRLT_NEVER_STILL")' in txt


# -------------------------------------------------------------------- 3: a doubtful heading beside another robot
def _doubt_rig(neigh_lost=False, gap=20.0):
    """Left's heading in doubt (a heading check pending after a large correction), Center `gap` cm east of it, standing
    (its own never-still creeps held off by the test, so the case stays as set)."""
    dark = [(60.0 + gap - 8.0, 50.0, 60.0 + gap + 8.0, 70.0)] if neigh_lost else []     # the camera cannot see Center
    R = H.Rig({"Left": (60.0, 60.0, 0.0), "Center": (60.0 + gap, 60.0, 180.0), "Right": (20.0, 110.0, 0.0)},
              never_still=True, dark=dark)
    R.run(0.05)
    r = R.c.R["Left"]
    r.hd_confirm = dict(t=R.t, why="test: a large correction", tries=0, stage=None)
    return R, r


def _hold_others(R):
    for n in ("Center", "Right"):
        R.c.R[n].still_since = R.t
        if R.c.R[n].lost is not None:
            R.c.R[n].lost["t0"] = min(R.c.R[n].lost["t0"], R.t)


def test_heading_in_doubt_beside_a_neighbour_the_camera_has_waits_for_room():
    R, r = _doubt_rig(gap=16.0)
    R.run(12.0, hook=_hold_others)
    assert R.c.counts.get("doubt_waits", 0) >= 1 and not R.c.counts.get("doubt_cautious")
    assert math.dist(R.truth["Left"][:2], R.truth["Center"][:2]) >= 16.0 - 0.5       # never closer
    assert R.first("waits for room", "Left") is not None


def test_heading_in_doubt_with_room_creeps_straight_away_from_a_known_neighbour():
    R, r = _doubt_rig(gap=24.0)
    R.run(10.0, hook=_hold_others)
    assert R.c.counts.get("doubt_away", 0) >= 1
    assert math.dist(R.truth["Left"][:2], R.truth["Center"][:2]) > 24.0 - 0.5


def test_heading_in_doubt_beside_a_neighbour_it_does_not_have_creeps_cautiously_away():
    R, r = _doubt_rig(neigh_lost=True, gap=18.0)
    R.run(10.0, hook=_hold_others)
    assert R.c.counts.get("doubt_cautious", 0) >= 1
    rows = [l for l in R.logs if l.get("table") == "creep" and l["robot"] == "Left" and l.get("doubt") == "cautious"]
    assert rows and all(q["L_cm"] <= 0.5 * FC.CREEP_MAX_CM + 1e-6 and q["v"] <= FC.CREEP_V_MIN + 1e-6 for q in rows)


def test_the_switch_keeps_e17s_creep(monkeypatch):
    monkeypatch.setattr(FC, "DOUBT_NEIGHBOUR", False)
    R, r = _doubt_rig(gap=16.0)
    R.run(8.0, hook=_hold_others)
    assert not R.c.counts.get("doubt_waits") and not R.c.counts.get("doubt_cautious")


# ------------------------------------------------------------------------------------------ 4: lost robots first
def _readable_ws(limit=70.0):
    ws = PW.ProvenWorkspace(H.BOARD)
    ws.see(H.BOARD)
    ws.limit_to(lambda x, y: x < limit, "readable")
    return ws


def test_a_lost_robot_goes_first_and_a_standing_robot_in_its_way_makes_room():
    R = H.Rig({"Left": (95.0, 60.0, 180.0), "Center": (62.0, 60.0, 0.0), "Right": (20.0, 110.0, 0.0)},
              never_still=True)
    R.c.ws = R.c.rect = _readable_ws()
    R.run(1.0)
    left, center = R.c.R["Left"], R.c.R["Center"]
    assert R.c.recovering(left) and R.c.prio(left) < R.c.prio(center)       # it outranks the others in the reflex
    R.run(40.0)
    assert R.c.counts.get("room_asked", 0) >= 1 and R.c.counts.get("room_made", 0) >= 1
    assert R.truth["Left"][0] < 70.0                                         # back where the camera reads
    assert R.first("makes room", "Center") is not None


def test_the_readable_point_it_heads_for_is_one_nobody_else_is_using():
    R = H.Rig({"Left": (95.0, 60.0, 180.0), "Center": (64.0, 60.0, 0.0), "Right": (20.0, 110.0, 0.0)})
    R.c.ws = R.c.rect = _readable_ws()
    R.run(0.5)
    left = R.c.R["Left"]
    left.last_in = (64.0, 60.0)                                               # where Center stands now
    tgt, _ = R.c.creep_target(left, R.c.pos(left))
    assert math.dist(tgt, R.c.pos(R.c.R["Center"])) >= FC.PLAN_CC


def test_missions_give_a_robot_with_no_readable_points_the_right_of_way():
    import mission_run as MR
    pts = {"P01": (20.0, 20.0), "P02": (40.0, 20.0), "P03": (200.0, 20.0)}
    orders = {"Alpha": ["P01", "P02"], "Bravo": []}
    out = MR.to_readable_when_empty(orders, pts, {"Alpha": (20, 20), "Bravo": (190, 20)},
                                    lambda x, y: (x < 100, ""), ["Alpha", "Bravo"])
    assert out["Bravo"] == ["P02"] and MR.RESCUED == ["Bravo"]
    txt = open(os.path.join(FLEET, "mission_run.py")).read()
    assert 'over["priority_first"] = RESCUED[0]' in txt


# ------------------------------------------------------------------------- 5: the odometry trust and blind legs
def test_the_trust_model_starts_at_the_replays_numbers_and_follows_a_robots_own_record():
    t = OT.Trust("A")
    assert abs(t.radius(40.0) - (2.3 + 0.28 * 40)) < 1.0                     # the measured prior (the replay)
    good, poor = OT.Trust("G"), OT.Trust("P")
    for k in range(60):
        L = (10.0, 20.0, 40.0)[k % 3]
        good.add_sample(L, 1.2 + 0.02 * L)
        poor.add_sample(L, 2.0 + 0.40 * L)
    assert good.radius(40.0) < t.radius(40.0) < poor.radius(40.0)
    assert poor.radius(40.0) > 10.0
    # turns: a robot whose heading slips on turns gets a bigger error after a turn than before it
    turny = OT.Trust("T")
    for k in range(60):
        L = (10.0, 20.0, 40.0)[k % 3]
        turny.add_sample(L, 1.5 + 0.03 * L)
        turny.add_sample(L, 1.5 + 0.03 * L + 0.10 * 90.0 * L * math.pi / 180.0, ts=90.0, ws=90.0 * L)
    assert turny.radius(40.0, 90.0, 0.0, 90.0 * 40.0, 0.0) > turny.radius(40.0) + 3.0


def test_the_model_checks_itself_and_corrects_its_own_scale():
    t = OT.Trust("A")
    for k in range(25):
        t.check(t.radius(20.0), 2.0 * t.radius_raw(20.0), "seen again")      # it under-predicted by 2x
    assert t.k_cal > 1.8 and t.cal[0]["within"] == 0 and t.cal[-1]["within"] == 1   # caught, then corrected
    t2 = OT.Trust("B")
    for k in range(25):
        t2.check(t2.radius(20.0), 0.3 * t2.radius_raw(20.0), "seen again")    # far over: shrinks, never under K_MIN
    assert t2.k_cal == pytest.approx(OT.K_MIN)


def test_records_are_saved_and_loaded(tmp_path):
    t = OT.Trust("Left")
    for k in range(10):
        t.add_sample(20.0, 3.0)
    p = OT.save({"Left": t}, str(tmp_path / "odo_trust.json"), why="test")
    back = OT.load(["Left", "New"], p)
    assert len(back["Left"].samples) == 10 and back["New"].samples == []


def _dark_rig(odo, seed=1, roam=150.0):
    ws = PW.ProvenWorkspace(H.BOARD)
    ws.see(H.BOARD, None, [DARK])
    R = H.Rig({"Left": (40.0, 60.0, 0.0), "Center": (100.0, 105.0, 90.0), "Right": (20.0, 15.0, 180.0)},
              odo={"Left": odo}, dark=[DARK], workspace=ws, raw=True, seed=seed, fuse_then=True)
    R.c.motion_style = "stopturn"
    R.run(0.5)
    r = R.c.R["Left"]
    r.busy_until = 0.0
    R.run(roam)
    r.lost_hold = True
    for _ in range(40):
        if r.mode == "idle" and r.move is None:
            break
        R.run(0.5)
    r.lost_hold = False
    r.busy_until = 1e9
    R.run(2.0)
    return R, r


def test_a_blind_excursion_through_a_dark_patch_good_record_allowed_poor_record_shortened_or_refused():
    Rg, rg = _dark_rig((0.003, 0.0, 0.005))
    Rp, rp = _dark_rig((0.05, 0.03, 0.05))
    assert Rg.c.trust["Left"].radius(40.0) < Rp.c.trust["Left"].radius(40.0)
    res_g = Rg.c.blind_excursion(rg, (85.0, 62.0))
    res_p = Rp.c.blind_excursion(rp, (85.0, 62.0))
    assert res_g["ok"] and res_g["depth_cm"] == pytest.approx(res_g["want_cm"])
    deep_g = res_g["depth_cm"] - (math.dist(Rg.c.pos(rg), (85.0, 62.0)) - 15.0)
    assert (not res_p["ok"]) or res_p["depth_cm"] < res_p["want_cm"] - 1.0      # shortened or refused
    Rg.run(30.0)
    chk = [q for q in Rg.c.trust["Left"].cal if q["kind"] == "excursion"]
    assert chk and chk[-1]["within"] == 1                                      # the error found inside the prediction
    rows = [l for l in Rg.logs if l.get("table") == "blind_checks"]
    assert rows and rows[-1]["excursion"] == 1
    assert deep_g > 0


def test_the_switch_keeps_the_fixed_blind_numbers(monkeypatch):
    c = FC.Controller(["A"], H.BOARD, lambda n, s: None, lambda *a: None)
    c.blind_allow = False
    c.on_follow("A", 0.0, "idle", 50.0, 50.0, 0.0)
    c.on_fix("A", 0.0, 50.0, 50.0)
    c.start_lost(c.R["A"], "test")
    c.on_follow("A", 1.0, "drive", 60.0, 50.0, 0.0)
    assert c.R["A"].bubble == pytest.approx(FC.BLIND_BASE + FC.BLIND_PER_CM * 10.0)


def test_missions_keep_an_unreadable_point_a_blind_leg_reaches():
    import run_plan as RP
    import test_keep_going as KG
    mm, plan = KG.production_plan()
    c = KG.coord(mm, plan)

    class RM:
        nx, ny, cell = 40, 17, 10.0

        def centre(self, i, j):
            return (5.0 + 10.0 * i, 5.0 + 10.0 * j)

        def readable(self, x, y):
            return (x < 300.0, "test")
    c.readmap = RM()
    near = c.blind_leg_allowed("Alpha", (310.0, 80.0), c.readmap.readable)
    far = c.blind_leg_allowed("Alpha", (390.0, 80.0), c.readmap.readable)
    assert near["ok"] and not far["ok"]


# --------------------------------------------------------------- 6: every calibration correction saved at any end
def test_every_correction_that_agrees_is_saved_at_the_end_and_one_that_does_not_is_said():
    sent = []
    c = FC.Controller(["A", "B"], H.BOARD, lambda n, s: sent.append((n, s)), lambda *a: None)
    for n in ("A", "B"):
        c.R[n].mode = "idle"
        c.R[n].curve = False
    c.params = {"A": {"wheel_scale": 0.98, "gyro_scale": 1.07}, "B": {"gyro_scale": 1.07}}
    for k in range(30):
        c.ratio_fit(c.R["A"], "drive", 1.02 + 0.001 * (k % 5))           # consistent: saved
        c.ratio_fit(c.R["B"], "drive", 1.02 + 0.001 * (k % 5))           # its wheel_scale was not heard: said, not saved
    done = c.save_corrections("the time cap")
    assert ("A", "set wheel_scale %.4f" % (0.98 * 1.022)) in sent or any(
        s.startswith("set wheel_scale 1.00") for n, s in sent if n == "A")
    assert ("A", "save") in sent and not any(s.startswith("set wheel_scale") for n, s in sent if n == "B")
    assert {d["robot"]: d["ok"] for d in done} == {"A": True, "B": False}


def test_free_roam_saves_at_every_ending_including_stop_and_errors():
    txt = open(os.path.join(FLEET, "freeroam.py")).read()
    fin = txt[txt.index("    finally:\n        if cpu is not None:"):]
    assert "runner.end_save(why)" in fin.split("runner.write_state(why)")[0]     # Ctrl+C / SIGTERM / an error
    assert "self.end_save(why)" in txt                                           # the time cap, all out
    assert "FC.CHECKPOINT_S" in txt                                              # a crash loses at most a minute
    rp = open(os.path.join(FLEET, "run_plan.py")).read()
    assert "coord.trust_save(" in rp[rp.index("    finally:\n        # sends stop"):]


# ------------------------------------------------- the coordinator's replay of all 10 home runs (12 Nov), items 1-4
def _elim_scene(guard):
    """5 Nov 14:55 (the replay): Left and Center in an unresolved close pass, all three unaccounted, a steady spot at
    Left's OWN pose (0.1 cm off) and Right standing 56 cm away. Before: the elimination gave that spot to Right."""
    import random
    rng = random.Random(1)
    L = TL.TrackingLayer(["Left", "Center", "Right"], ("motion", "lights"), board=(0, 0, 122.24, 121.92),
                         pair_guard=guard, camera_watch=False)
    for n, p in (("Left", (60.0, 40.0)), ("Center", (50.0, 116.0)), ("Right", (27.0, 116.0))):
        L.seed(n, 0.0, p[0], p[1])
    named, t = [], 0.0
    while t < 6.0:
        t = round(t + 0.25, 3)
        L.own("Left", t, 17.1, 60.0, 0.0, True)
        L.own("Center", t, 50.0, 116.0, 90.0, True)
        L.own("Right", t, 27.0, 116.0, 90.0, True)
        if t > 1.0:
            for n in ("Left", "Center", "Right"):
                L.set_unaccounted(n, "lost")
            L.pairs[frozenset(("Left", "Center"))] = dict(t0=0.5, parted_t=None, state="close")
        for f in L.feed(t, [TL.Det("lights", t, 17.0 + rng.gauss(0, 0.3), 60.1 + rng.gauss(0, 0.3))]):
            if f.get("how"):
                named.append((f["robot"], f["how"]))
    return named, L.counts.get("elim_refused_guard", 0)


def test_the_elimination_never_gives_a_robot_a_spot_on_another_robots_own_pose():
    old, _ = _elim_scene(False)
    assert ("Right", "elimination") in old                     # the replay's wrong elimination, reproduced
    new, refused = _elim_scene(True)
    assert not any(n == "Right" for n, _ in new) and refused > 0
    assert SG.DEFAULT["MIRLT_ELIM_PAIR_GUARD"] == "1" and TL.ELIM_PAIR_GUARD


def _far_rig(on, monkeypatch):
    monkeypatch.setattr(FC, "FAR_NAMED_CORRECT", on)
    R = H.Rig({"Left": (30.0, 30.0, 0.0), "Center": (60.0, 90.0, 90.0), "Right": (95.0, 30.0, 180.0)}, seed=2)
    R.run(3.0)
    R.off["Center"] = [31.0 * 0.6, -31.0 * 0.8, 0.0]       # its own pose 31 cm off, the robot standing (11 Nov 08:26)
    R.run(4.0)
    return R, [e for e in R.events if e[1] == "Center" and e[2] == "CORRECTED" and "named it all along" in e[3]]


def test_a_robot_named_all_along_beyond_its_fix_gate_is_corrected_from_the_camera(monkeypatch):
    R, ev = _far_rig(True, monkeypatch)
    assert ev and ev[0][0] - 3.0 < 3.0 and R.own_gap("Center") < 3.0
    assert any(s[1] == "Center" and s[2].startswith("fpose") for s in R.sent)
    _, ev_off = _far_rig(False, monkeypatch)
    assert not ev_off                                          # MIRLT_FAR_NAMED_CORRECT=0: the e17 way


def test_far_named_needs_a_steady_offset_and_nobody_else_on_the_spot():
    c = FC.Controller(["A", "B"], H.BOARD, lambda *a: None, lambda *a: None)
    a, b = c.R["A"], c.R["B"]
    for k in range(30):
        c.t = k * 0.1
        c.on_follow("A", c.t, "idle", 20.0, 20.0, 0.0)
        c.on_follow("B", c.t, "idle", 50.0, 20.0, 0.0)
    oa = (20.0, 20.0, 0.0)
    assert not any(c.far_named(a, 3.0 + 0.2 * k, 50.0, 21.0, oa, "lights") for k in range(8))   # B's own pose
    assert not any(c.far_named(a, 5.0 + 0.2 * k, 30.0 + 4.0 * k, 20.0, oa, "lights") for k in range(8))  # not steady
    assert any(c.far_named(a, 7.0 + 0.2 * k, 35.0, 30.0, oa, "lights") for k in range(8))


def _reseed(on, monkeypatch, turned=0.0, src="tags"):
    monkeypatch.setattr(FC, "RESEED_HEADING_KEEP", on)
    R = H.Rig({"Left": (30.0, 30.0, 0.0), "Center": (60.0, 90.0, 90.0), "Right": (95.0, 30.0, 180.0)}, seed=2,
              wiggle=False)
    R.run(3.0)
    r = R.c.R["Center"]
    R.c.camera_proven = True
    r.turn_since_head = turned
    r.pfuse = types.SimpleNamespace(disagree_cm=6.0, feed=lambda *a: dict(d_cm=6.0, for_s=2.0), unc_extra=lambda: 0.0)
    R.c.pose_fuse(r, (60.0, 90.0), (64.0, 90.0), 130.0, t_pic=R.t - 0.3, src_=src)   # the camera says 130 deg
    fp = [s[2] for s in R.sent if s[1] == "Center" and s[2].startswith("fpose")]
    return float(fp[-1].split()[3])


def test_a_standing_reseed_keeps_its_own_heading_unless_the_camera_knows_better(monkeypatch):
    assert _reseed(True, monkeypatch) == pytest.approx(90.0)                 # its own heading: fresh, 2 deg doubt
    assert _reseed(True, monkeypatch, src="lights") == pytest.approx(90.0)   # an outline's heading never wins
    assert _reseed(True, monkeypatch, turned=900.0) == pytest.approx(130.0)  # ten turns since: the roof tag's is better
    assert _reseed(False, monkeypatch) == pytest.approx(130.0)               # MIRLT_RESEED_HEADING_KEEP=0: e17


def test_the_trust_record_is_the_fleets_until_a_robot_differs(monkeypatch):
    monkeypatch.setattr(OT, "FLEET", True)
    t = OT.Trust("A")
    assert abs(t.radius(40.0) - (2.3 + 0.28 * 40)) < 1.0                    # the replay's prior (90th pct slope)
    import random
    rng = random.Random(3)
    fl = OT.link({n: OT.Trust(n) for n in ("Left", "Center", "Right")})
    for k in range(30):
        L = (10.0, 20.0, 40.0)[k % 3]
        fl["Left"].add_sample(L, (2.3 + 0.06 * L) * rng.uniform(0.5, 1.2))      # the same drift, with scatter
        fl["Center"].add_sample(L, (2.3 + 0.06 * L) * rng.uniform(0.5, 1.2))
    # Right has no samples of its own yet: the fleet's record, not the prior
    assert fl["Right"].radius(40.0) < 7.0 and fl["Right"].summary()["record"] == "fleet"
    for k in range(30):
        L = (10.0, 20.0, 40.0)[k % 3]
        fl["Right"].add_sample(L, (2.0 + 0.30 * L) * rng.uniform(0.5, 1.2))  # Right drifts five times as much
    assert fl["Right"].summary()["record"] == "own" and fl["Right"].radius(40.0) > 12.0
    assert fl["Left"].summary()["record"] == "fleet" and fl["Left"].radius(40.0) < 7.0
    monkeypatch.setattr(OT, "FLEET", False)                                  # MIRLT_TRUST_FLEET=0: each its own
    fl["Right"]._fit = None
    assert OT.Trust("X").summary()["record"] == "own"


def test_curve_samples_count_only_in_their_own_session(monkeypatch):
    monkeypatch.setattr(OT, "FLEET", True)
    t = OT.Trust("A")
    for k in range(40):
        L = (10.0, 20.0, 40.0)[k % 3]
        t.add_sample(L, 2.0 + 0.05 * L)
        t.add_sample(L, 2.0 + 0.05 * L + 0.2 * 90.0 * L * math.pi / 180.0, tc=90.0, wc=90.0 * L, run="an earlier run")
    curve_old = t.radius(40.0, 0.0, 90.0, 0.0, 90.0 * 40.0)
    assert curve_old < t.radius(40.0) + 3.0                 # last session's bad curves do not count in this one
    monkeypatch.setattr(OT, "FLEET", False)
    t._fit = None
    assert t.radius(40.0, 0.0, 90.0, 0.0, 90.0 * 40.0) > t.radius(40.0) + 5.0   # the first e18 way: they did


def test_curve_scales_are_logged_not_saved_until_nick_decides(monkeypatch):
    c = FC.Controller(["Left"], H.BOARD, lambda *a: None, lambda *a: None)
    sent, ev = [], []
    c.out = lambda n, txt: sent.append(txt)
    c.event = lambda r, k, txt: ev.append(txt)
    r = c.R["Left"]
    r.curve, r.arc_scales = True, (1.0736, 1.0)
    monkeypatch.setattr(c, "arc_result", lambda r_: (0.95, 0.005, 1.02, 0.005, 40))
    monkeypatch.setattr(FC, "CURVE_SCALE_SAVE", False)
    c.arc_save(r)
    assert not any("arc_k_scale" in s or s == "save" for s in sent) and "NOT saved" in ev[-1]
    monkeypatch.setattr(FC, "CURVE_SCALE_SAVE", True)
    c.arc_save(r)
    assert any(s.startswith("set arc_k_scale") for s in sent)
    assert SG.DEFAULT["MIRLT_CURVE_SCALE_SAVE"] == "0"
