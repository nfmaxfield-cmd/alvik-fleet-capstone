"""e21 (Nick 12 Nov): one error shape per robot, the four stages, no overlapping shapes, the ratchet broken (Pi and
firmware), the simulator's firmware filter, per-camera verdicts and the bridge's per-camera sightings. Offline only.

Changelog: e21-2026-11-12a first version.
"""
import math
import os
import random
import shutil
import subprocess
import sys

import pytest

FLEET = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(FLEET))
sys.path.insert(0, FLEET)

import camera_trust as CT        # noqa: E402
import error_shape as ES         # noqa: E402
import freeroam_core as FC       # noqa: E402
import odo_trust as OT           # noqa: E402
import safety_group as SG        # noqa: E402
import safety_rules as SR        # noqa: E402
import tracking_layer as TL      # noqa: E402

BOARD = (0.0, 0.0, 122.0, 122.0)


def _shape(s_cm=0.0, base=2.45, h=0.0, x0=60.0, y0=60.0):
    sh = ES.Shape("A", OT.Trust("A"))
    sh.anchor(0.0, x0, y0, h, base)
    ux, uy = math.cos(math.radians(h)), math.sin(math.radians(h))
    for k in range(int(s_cm)):
        sh.drive(x0 + (k + 1) * ux, y0 + (k + 1) * uy, h, 1.0, 0.0, False)
    return sh


# -- 1. the shape ---------------------------------------------------------------------------------------------------
def test_the_prior_is_the_measured_cone_one_sided_along_the_track():
    assert ES.PRIOR["b_back"] > 15.0 > ES.PRIOR["b_fwd"]      # stuck wheels leave it behind, not ahead
    assert 0.2 < ES.PRIOR["a_across"] < 0.3                   # sideways 0.232 cm per cm (95 %)
    sh = _shape(40.0)
    f, b, c = sh.axes()
    assert b > f and c > f                                     # back along its track and sideways, little ahead
    i, a, bb, ang, off = sh.ellipse()
    assert off < 0 and abs(ang) < 1e-6                        # its centre sits back along its track
    assert sh.extent() == pytest.approx(i + max(a + abs(off), bb))


def test_a_robot_that_has_not_driven_has_no_cone_only_what_was_seen():
    sh = _shape(0.0, base=3.0)
    assert sh.extent() == pytest.approx(3.0)


def test_reach_toward_each_edge_differs_with_the_direction_of_its_track():
    """Driving +x along the board: ahead (toward x1) it reaches little, behind (toward x0) far, sideways the cone."""
    sh = _shape(40.0, x0=40.0)
    ell = sh.ellipse()
    ahead = sh.support(1.0, 0.0, ell)
    behind = sh.support(-1.0, 0.0, ell)
    side = sh.support(0.0, 1.0, ell)
    assert behind > side > ahead
    # rect_margin is the least over the four sides of distance less reach
    x, y = sh.x, sh.y
    m = ES.rect_margin(BOARD, x, y, *ell)
    assert m == pytest.approx(min(x - behind, 122 - x - ahead, y - side, 122 - y - side), abs=1e-6)


def test_it_shrinks_only_when_a_fix_is_accepted_and_events_widen_it_at_once():
    sh = _shape(30.0)
    e0 = sh.extent()
    add = sh.event(1.0, "front stop")
    assert add == pytest.approx(ES.EVENT_CM["front stop"])
    assert sh.extent() == pytest.approx(e0 + add)
    sh.event(2.0, "front stop")                      # independent surprises add in quadrature
    assert sh.iso() == pytest.approx(sh.base + math.sqrt(2) * ES.EVENT_CM["front stop"])
    assert sh.event(3.0, "lost report") == 0.0       # not an event that moves it
    sh.drive(sh.x + 1, sh.y, 0.0, 1.0, 0.0, False)
    assert sh.extent() > e0                          # driving never shrinks it
    sh.anchor(4.0, sh.x, sh.y, 0.0, 2.45, "fix")
    assert sh.extent() == pytest.approx(2.45)        # an accepted fix does


def test_a_heading_unknown_makes_the_cone_a_disc_of_the_distance():
    sh = _shape(0.0)
    sh.hd_deg = 180.0
    f, b, c = sh.axes_at(10.0, 0.0)
    assert c >= 10.0 - 1e-6 and b >= 20.0 - 1e-6     # anywhere within 10 cm of where it started


def test_the_record_fits_along_and_across_from_odo_trust_samples():
    tr = OT.Trust("A")
    rng = random.Random(3)
    for k in range(80):
        s = rng.choice((10.0, 20.0, 40.0))
        tr.add_sample(s, 0.0, al=-rng.uniform(0, 0.5), ac=rng.gauss(0, 0.05 * s))
    f = ES.AxesFit(tr).get()
    assert f["source"] == "record"
    assert f["b_back"] < ES.PRIOR["b_back"]          # this robot does not get stuck: its own record says so
    assert f["a_across"] < ES.PRIOR["a_across"]


# -- 2. no overlapping shapes: the cheapest bound that is still safe -----------------------------------------------------
def test_the_pair_gap_along_the_centre_line_never_says_apart_when_they_overlap():
    rng = random.Random(7)
    for _ in range(300):
        sa = (rng.uniform(0, 4), rng.uniform(0, 15), rng.uniform(0, 15), rng.uniform(-180, 180), rng.uniform(-8, 0))
        sb = (rng.uniform(0, 4), rng.uniform(0, 15), rng.uniform(0, 15), rng.uniform(-180, 180), rng.uniform(-8, 0))
        p, q = (0.0, 0.0), (rng.uniform(-50, 50), rng.uniform(-50, 50))
        g = ES.pair_gap(p, sa, q, sb)
        if g > 0:
            assert ES.true_gap(p, sa, q, sb) >= g - 1e-6      # a gap it reports is a real gap


def test_the_ellipse_bound_lets_two_side_by_side_tracks_pass_where_circles_would_not():
    # two lost robots driving parallel, 30 cm apart: their cones point along the tracks
    sa = (2.0, 16.0, 6.0, 0.0, 0.0)
    sb = (2.0, 16.0, 6.0, 0.0, 0.0)
    circle = 30.0 - 2 * (2.0 + 16.0)
    assert circle < 0 < ES.pair_gap((0, 0), sa, (0, 30), sb)


def test_reservations_carry_the_shape_and_clearance_reads_it():
    A = FC.Traj.move(0.0, (20.0, 20.0), 0.0, 0.0, 30.0, 5.0)
    B = FC.Traj.move(0.0, (20.0, 45.0), 0.0, 0.0, 30.0, 5.0)
    c0 = FC.clearance(A, B)
    A.set_shape([(0.0, 16.0, 4.0, 0.0)] * len(A.x))
    c1 = FC.clearance(A, B)
    assert 4.0 - 0.2 <= c0 - c1 <= 6.0                        # across the tracks: about its 4 cm across, not 16
    C = FC.Traj.move(0.0, (90.0, 20.0), 0.0, 180.0, 1.0, 5.0)   # ahead of A, on its line
    cc0 = FC.clearance(FC.Traj.move(0.0, (20.0, 20.0), 0.0, 0.0, 30.0, 5.0), C)
    assert cc0 - FC.clearance(A, C) == pytest.approx(16.0, abs=0.5)   # along its track: the full 16


# -- 3. the controller: one shape read by all, the four stages ---------------------------------------------------------
def _ctrl(names=("A", "B")):
    out, logs = [], []
    c = FC.Controller(list(names), BOARD, lambda n, t: out.append((n, t)), lambda tb, row: logs.append((tb, row)))
    c.t = 1.0
    return c, out, logs


def _place(c, n, x, y, th, t=1.0, lost=False):
    r = c.R[n]
    c.on_follow(n, t - 0.2, "idle", x, y, th)
    c.on_follow(n, t, "idle", x, y, th)
    r.cam = (t, x, y, None)                  # lights: no heading from the camera
    r.cam_arr = t
    r.mode = "idle"
    if lost:
        c.t = t
        c.start_lost(r, "test")
    return r


def test_the_shape_is_the_controllers_one_doubt_and_the_bubble_reads_it():
    c, out, logs = _ctrl()
    r = _place(c, "A", 60.0, 60.0, 0.0, lost=True)
    for k in range(20):
        c.t += 0.2
        c.on_follow("A", c.t, "drive", 60.0 + k + 1, 60.0, 0.0)
    assert r.bubble == pytest.approx(r.shape.extent())
    assert c.unc(r) == pytest.approx(FC.PLAN_M0 + r.shape.extent())
    c.event(r, "did not move", "test")
    assert r.bubble == pytest.approx(r.shape.extent())        # widened at once


def test_veer_when_the_shape_would_reach_an_edge_within_the_next_moves(monkeypatch):
    monkeypatch.setattr(FC, "STAGES", True)
    c, out, logs = _ctrl()
    r = _place(c, "A", 60.0, 60.0, 0.0, lost=True)
    assert c.stage_of(r) == "run"
    r2 = _place(c, "B", 30.0, 60.0, 180.0, lost=True)
    for k in range(8):
        c.t += 0.2
        c.on_follow("B", c.t, "drive", 30.0 - k - 1, 60.0, 180.0)
    assert c.stage_of(r2) == "veer"


def test_no_move_gains_room_with_the_heading_unknown_but_one_does_when_it_is_known(monkeypatch):
    monkeypatch.setattr(FC, "STAGES", True)
    c, out, logs = _ctrl()
    r = _place(c, "A", 14.0, 60.0, 180.0, lost=True)
    for k in range(6):
        c.t += 0.2
        c.on_follow("A", c.t, "drive", 14.0 - 0.5 * (k + 1), 60.0, 180.0)
    p, h = c.pos(r), c.heading(r)
    g = c.gain_move(r, p, h)
    mot = g["h1"] + (180.0 if g["back"] else 0.0) if g is not None else None
    assert g is not None and math.cos(math.radians(mot)) > 0.5         # inward (+x) gains room
    r.head_unknown = 1.0
    assert c.gain_move(r, p, h) is None                               # Nick: in practice, heading unknown


def test_pause_and_search_turns_focuses_the_cameras_holds_only_who_must_then_asks(monkeypatch):
    monkeypatch.setattr(FC, "STAGES", True)
    c, out, logs = _ctrl(("A", "B", "C"))
    r = _place(c, "A", 30.0, 60.0, 0.0, lost=True)
    _place(c, "B", 30.0, 75.0, 0.0)          # within the camera's close-pass reach of its shape: must hold
    _place(c, "C", 100.0, 100.0, 0.0)        # far: carries on
    asked = []
    c.place_fn = lambda name, *a, **k: asked.append(name)
    c.search_step(r)
    assert c.stage_of(r) == "pause"
    assert "A" in c.search_focus
    assert c.R["B"].__dict__.get("search_hold_until") and not c.R["C"].__dict__.get("search_hold_until")
    assert r.move is not None and r.move["kind"] == "search turn"       # it turns on the spot
    c.t += c.search_window(r) + 0.1
    r.move, r.mode = None, "idle"
    c.search_step(r)
    assert r.search["asked"]
    c.search_end(r, "found")
    assert "A" not in c.search_focus and not c.R["B"].search_hold_until


def test_no_creep_without_the_edge_rule(monkeypatch):
    monkeypatch.setattr(FC, "STAGES", True)
    monkeypatch.setattr(FC, "LOST_CREEP", False)
    c, out, logs = _ctrl()
    r = _place(c, "A", 20.0, 60.0, 0.0)
    assert c.creep_edge_ok(r, (20.0, 60.0), (26.0, 60.0))      # inward
    assert not c.creep_edge_ok(r, (12.0, 60.0), (8.0, 60.0))   # outward, already inside the clearance
    monkeypatch.setattr(FC, "LOST_CREEP", True)
    assert c.creep_edge_ok(r, (12.0, 60.0), (8.0, 60.0))       # the e17 creep behind its switch


# -- 4. the ratchet broken on the Pi ------------------------------------------------------------------------------------
def test_consistent_sightings_beyond_its_gate_correct_its_own_pose_and_ask_its_heading(monkeypatch):
    monkeypatch.setattr(FC, "CONSISTENT_ESCAPE", True)
    c, out, logs = _ctrl()
    r = _place(c, "A", 71.0, 50.0, 14.0, t=1.0)
    _place(c, "B", 20.0, 100.0, 0.0, t=1.0)
    c.R["A"].fpose_t = -100.0
    fired = None
    for k in range(12):
        t = 2.0 + 0.6 * k
        c.t = t
        c.on_follow("A", t, "idle", 71.0, 50.0, 14.0)
        if c.consistent_escape(r, t, 46.0 + 0.3 * ((k % 3) - 1), 50.0, (71.0, 50.0, 14.0), 1.0, "cam_bridge:both", None):
            fired = k + 1
            break
    assert fired is not None and fired >= FC.ESCAPE_MIN_N
    c.correct_from_camera(r, c.t, 46.0, 50.0, None, c.t, "consistent", "cam_bridge:both")
    assert r.__dict__.get("head_unknown") is not None and r.hd_confirm is not None
    assert any("fpose 46" in t_ for n, t_ in out if n == "A")


def test_sightings_that_jump_about_or_lie_on_another_robot_never_escape(monkeypatch):
    c, out, logs = _ctrl()
    r = _place(c, "A", 71.0, 50.0, 14.0)
    _place(c, "B", 46.0, 50.0, 0.0)            # B stands where the far sighting is
    r.fpose_t = -100.0
    assert not any(c.consistent_escape(r, 2.0 + 0.6 * k, 46.0, 50.0, (71.0, 50.0, 14.0), 1.0, "x", None)
                   for k in range(12))
    c2, _, _ = _ctrl()
    r2 = _place(c2, "A", 71.0, 50.0, 14.0)
    r2.fpose_t = -100.0
    assert not any(c2.consistent_escape(r2, 2.0 + 0.6 * k, 46.0 if k % 2 else 60.0, 50.0 if k % 2 else 30.0,
                                        (71.0, 50.0, 14.0), 1.0, "x", None) for k in range(12))


def test_inward_by_both_uses_each_positions_own_heading():
    # own pose says it faces +x (inward from x = 0); the camera saw it drive -x (a 180 deg flip)
    assert SR.inward_agreed(BOARD, (10, 60), (11, 60), 0.0)                       # e08: one heading, fooled
    assert not SR.inward_agreed(BOARD, (10, 60), (11, 60), 0.0, heading_cam=180.0)  # e21: the flip is caught
    assert not SR.inward_agreed(BOARD, (10, 60), (11, 60), 0.0, heading_cam=None)   # unknown: never guessed
    assert SR.inward_agreed(BOARD, (10, 60), (11, 60), 0.0, heading_cam=5.0)


def test_a_corrected_robots_heading_is_not_trusted_for_inward_until_the_camera_sees_it_move(monkeypatch):
    monkeypatch.setattr(FC, "INWARD_TWO", True)
    c, out, logs = _ctrl()
    r = _place(c, "A", 10.0, 60.0, 0.0)
    assert c.inward_ok((10, 60), (11, 60), 0.0, False, r)
    r.head_unknown = 1.0
    assert not c.inward_ok((10, 60), (11, 60), 0.0, False, r)


# -- 5. the simulator shows the ratchet; bumps and slips ---------------------------------------------------------------
def _sim_bot(F, params):
    import freeroam_fastsim as FS
    s = FS.Sim.__new__(FS.Sim)
    s.t, s.stats, s.own_off = 10.0, {}, []
    b = FS.Bot("A", 50.0, 50.0, 0.0, random.Random(1), True)
    b.fwp = dict(params)
    b.var, b.var_th, b.vlast = 1.0, 4.0, (50.0, 50.0, 0.0)
    b.hist = [(7.0 + 0.1 * k, 50.0, 50.0) for k in range(31)]
    return s, b


def test_the_simulated_robot_ratchets_as_the_firmware_does_and_the_e21_firmware_escapes():
    import freeroam_fastsim as FS
    s, b = _sim_bot(FS, FS.FW_PARAMS_OLD)
    for k in range(20):
        s.t += 0.6
        b.hist.append((s.t, b.ox, b.oy))
        s.fuse_fw(b, 60.0, 50.0, None, s.t - 0.3, 1.0, 3.0)
    assert b.ox == pytest.approx(50.0) and b.rej.get("far") == 20         # refused for good
    s, b = _sim_bot(FS, FS.FW_PARAMS_E21)
    for k in range(20):
        s.t += 0.6
        b.hist.append((s.t, b.ox, b.oy))
        s.fuse_fw(b, 60.0, 50.0, None, s.t - 0.3, 1.0, 3.0)
    assert b.ox == pytest.approx(60.0, abs=0.5) and s.stats.get("fw_escapes") == 1


def test_the_simulated_firmware_refuses_old_fixes_by_its_age_limit():
    import freeroam_fastsim as FS
    s, b = _sim_bot(FS, FS.FW_PARAMS_OLD)
    s.t += 0.1
    b.hist.append((s.t, 50.0, 50.0))
    assert s.fuse_fw(b, 50.5, 50.0, None, s.t - 1.2, 1.0, 3.0) is False and b.rej.get("old") == 1
    s, b = _sim_bot(FS, FS.FW_PARAMS_E21)
    s.t += 0.1
    b.hist.append((s.t, 50.0, 50.0))
    assert s.fuse_fw(b, 50.5, 50.0, None, s.t - 1.2, 1.0, 3.0) is True


def test_harsh_has_bumps_and_slips():
    import fastsim_realism as R
    assert "bumps" in R.HARSH_FAULTS and "slips" in R.HARSH_FAULTS


@pytest.mark.skipif(shutil.which("g++") is None, reason="needs g++")
def test_firmware_pose_filter_host_test_passes():
    r = subprocess.run(["sh", os.path.join(REPO, "tools", "fwcheck", "pose_test.sh")], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert r.stdout.count("ok ") == 15


def test_firmware_version_and_old_behaviour_behind_params():
    ino = open(os.path.join(REPO, "firmware", "home_lab", "home_lab.ino")).read()
    assert '#define FW_VERSION "2026-11-12e21"' in ino
    par = open(os.path.join(REPO, "firmware", "home_lab", "params.h")).read()
    for k in ("pose_sd_per_cm", "pose_hist_ms", "fix_escape_n", "fix_max_age_ms"):
        assert '"%s"' % k in par


def test_the_pi_reads_follow_rows_from_old_and_new_firmware():
    c, out, logs = _ctrl()
    c.on_fw_state("A", 1.5, None)                                   # old firmware: its sd only
    assert c.fw_gate_cm(c.R["A"]) == pytest.approx(6.0)
    c.on_fw_state("A", 2.0, dict(old=3, far=7, hist=0, future=0, escapes=1))     # e21 firmware: reasons too
    assert c.R["A"].fw_rej["far"] == 7 and c.fw_gate_cm(c.R["A"]) == pytest.approx(8.0)
    assert any(tb == "fw_refusals" for tb, row in logs)


# -- 6. per-camera verdicts; the bridge's sightings per camera --------------------------------------------------------
def test_a_mirrored_camera_is_refused_alone_and_the_good_cameras_sightings_are_used(monkeypatch):
    monkeypatch.setattr(TL, "PER_CAMERA", True)
    book = CT.TrustBook("t", board=(0, 0, 122, 122))
    L = TL.TrackingLayer(["A", "B", "C"], sources=("motion",), book=book, board=(0, 0, 122, 122))
    L._one_transform = staticmethod(lambda ux: True)
    xs = (20.0, 60.0, 100.0)
    for n, x in zip("ABC", xs):
        L.seed(n, 0.0, x, 40.0)
    got = []
    for k in range(40):
        t = 0.3 * k
        for n, x in zip("ABC", xs):
            L.own(n, t, x, 40.0 + 1.0 * k, 90.0, False)
        dets = [TL.Det("motion", t, x, 40.0 + 1.0 * k, cam="good") for x in xs] + \
            [TL.Det("motion", t, x + 0.5, 80.0 - 1.0 * k, cam="mirror") for x in xs]
        got = L.feed(t, dets)
    assert "mirror" in L.cam_bad and "good" not in L.cam_bad
    assert len(got) == 3                                        # the good camera still fixes all three


def test_bridge_per_camera_switch_and_twocam_state_carry_each_camera():
    assert SG.value("MIRLT_BRIDGE_PER_CAMERA") in ("1", 1, True) or SG.flag("MIRLT_BRIDGE_PER_CAMERA")
    src = open(os.path.join(REPO, "tools", "experiment", "e02_twocam.py")).read()
    assert "per_cam=getattr(tr, \"per_cam\", None)" in src
    br = open(os.path.join(FLEET, "cam_bridge.py")).read()
    assert '"src": "cam_bridge:%s" % cam_' in br


# -- 7. the Safety group, Mission Control ------------------------------------------------------------------------------
def test_every_e21_switch_is_in_the_safety_group_on_its_automatic_default():
    want = {"MIRLT_ERROR_SHAPE": "cone", "MIRLT_STAGES": "four", "MIRLT_LOST_CREEP": "0", "MIRLT_SHAPE_PLAN": "ellipse",
            "MIRLT_CONSISTENT_ESCAPE": "1", "MIRLT_INWARD_TWO_HEADINGS": "1", "MIRLT_FW_FILTER_PARAMS": "derived",
            "MIRLT_CAMERA_VERDICT": "per_camera", "MIRLT_BRIDGE_PER_CAMERA": "1"}
    for k, v in want.items():
        assert SG.DEFAULT[k] == v, k


def test_mission_control_shows_the_shape_not_the_firmwares_own_sd(tmp_path):
    import json
    import live_view as LV
    d = tmp_path
    (d / "state.json").write_text(json.dumps({"robots": {"Left": {"mode": "move", "shape": {
        "iso": 2.4, "forward": 3.0, "back": 12.0, "across": 6.0, "extent": 16.5}, "stage": "veer"}}}))
    (d / "follow.csv").write_text("robot,t_recv,run,ms,step_id,state,queued,last_done,last_failed,x_cm,y_cm,th_deg,"
                                  "pos_sd_cm,th_sd_deg,fixes_used,fixes_rejected,front_cm,travel_cm,quiet_ms\n"
                                  "Left,%.2f,r,1,0,idle,0,0,0,50,50,0,0.5,1,1,0,10,0,0\n" % 1e9)
    rows = LV.from_freeroam(str(d), {"Left": "#f00"}, {}, now=1e9 + 0.5)
    left = [r for r in rows if r["name"] == "Left"][0]
    assert left["pm_cm"] == pytest.approx(16.5) and left["stage"] == "veer"
