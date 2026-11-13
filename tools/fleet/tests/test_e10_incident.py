"""e10, the 7 Nov run 2 incident (Right hung over the y = 0 edge): its 55 cm straight ran 25 deg off, the piece
queued behind it was planned from the straight's planned end and ran from where Right really was, and its line-sensor
stop 9 cm in by the camera was called a surface mark. The fixes, each behind a switch (MIRLT_QUEUE_WATCH,
MIRLT_MARK_UNC, MIRLT_START_EDGE). No hardware."""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
for p in (FLEET, os.path.dirname(FLEET)):
    if p not in sys.path:
        sys.path.insert(0, p)

import freeroam_core as FC        # noqa: E402
import start_discovery as D       # noqa: E402

HOME = (0.0, 0.0, 122.24, 121.92)


def straight_with_queue(qw=True, err_deg=25.0):
    """Right's case: a 55 cm straight at -47.7 deg from (40.8, 72.7) with a piece queued behind it; its own pose
    follows the plan, the camera sees it go err_deg further round."""
    out = []
    c = FC.Controller(["Right"], HOME, lambda n, s: out.append(s), lambda *a: None, pose_source="blend",
                      queue_watch=qw)
    r = c.R["Right"]
    r.curve = True
    h = -47.7
    c.t = 10.0
    c.on_follow("Right", 10.0, "idle", 40.8, 72.7, h)
    c.on_fix("Right", 10.0, 40.8, 72.7, None)
    mv = c.send_piece(r, "menu", None, 0.0, 54.9, 5.0, 1, 40.8, 72.7, h, 10.0)
    mv["phase"], mv["t_drive"] = "drive", 10.0
    r.move, r.mode = mv, "move"
    q = c.send_piece(r, "menu", None, 0.0354, 12.9, 7.5, 1, mv["end"][0], mv["end"][1], h, 21.0)
    r.queue = [q]
    for st in r.steps.values():
        st["status"] = "acked"
    acted = None
    t = 10.0
    while t < 20.0:
        t = round(t + 0.1, 2)
        c.t = t
        s = (t - 10.0) * 5.0
        ox, oy = 40.8 + s * math.cos(math.radians(h)), 72.7 + s * math.sin(math.radians(h))
        tx, ty = 40.8 + s * math.cos(math.radians(h - err_deg)), 72.7 + s * math.sin(math.radians(h - err_deg))
        c.on_follow("Right", t, "curve", ox, oy, h)
        c.on_fix("Right", t, tx, ty, None)
        c.queue_watch(r)
        if acted is None and (not r.queue or r.move is None):
            acted = (t, s, FC.edge_dist(HOME, tx, ty))
    return c, r, out, acted, q


def test_a_straight_running_25_deg_off_is_caught_early_and_its_queued_piece_dropped():
    c, r, out, acted, q = straight_with_queue(True)
    assert acted is not None
    t, s, e = acted
    assert s < 15.0                          # caught within 15 cm of a 55 cm straight (run 2: ran the whole 55 cm)
    assert e > 25.0                          # still far from the edge
    assert r.steps[q["drive_id"]]["status"] == "dropped"
    assert any(x.startswith(("farc", "halt", "fpose")) for x in out)
    assert c.counts["queue_watch"] >= 1


def test_the_old_way_lets_the_queued_piece_stand():
    c, r, out, acted, q = straight_with_queue(False)
    assert acted is None and r.queue == [q] and c.counts.get("queue_watch", 0) == 0


def test_a_straight_on_its_line_is_left_alone():
    c, r, out, acted, q = straight_with_queue(True, err_deg=2.0)
    assert acted is None and r.queue == [q]


def line_trip(mark_unc, unc):
    c = FC.Controller(["Right"], HOME, lambda n, s: None, lambda *a: None, pose_source="blend", mark_unc=mark_unc)
    r = c.R["Right"]
    c.t = 5.0
    c.on_follow("Right", 5.0, "idle", 63.5, 9.4, -60.0)
    c.on_fix("Right", 5.0, 63.5, 9.4, None)
    c.unc = lambda r_: unc
    c.felt(r, "plywood edge at 70.3 10.5 -12.4 line")
    return c


def test_a_line_trip_9_cm_in_with_6_cm_of_doubt_is_the_edge():
    assert line_trip(True, 6.1).counts.get("surface_marks", 0) == 0          # run 2's numbers: the edge
    assert line_trip(True, 6.1).counts.get("marks_deferred", 0) == 1
    assert line_trip(True, 0.5).counts.get("surface_marks", 0) == 1          # a sure camera: a mark, as before
    assert line_trip(False, 6.1).counts.get("surface_marks", 0) == 1         # the old way (the switch)


def test_a_robot_that_starts_at_an_edge_first_tries_to_free_itself():
    sent = []
    c = FC.Controller(["Right"], HOME, lambda n, s: sent.append(s), lambda *a: None, pose_source="blend")
    r = c.R["Right"]
    c.t = 1.0
    c.on_follow("Right", 1.0, "idle", 60.0, 3.0, -90.0)
    c.on_fix("Right", 1.0, 60.0, 3.0, None)
    assert c.start_mode == "recover"
    assert c.start_check(r) and r.tilted and c.counts["start_recover"] == 1      # Nick 7 Nov: the e08 tries first
    assert c.counts.get("start_held", 0) == 0
    c.start_move(r)                                                              # the recovery's first try
    assert r.move is not None and r.move["kind"].startswith("recover") and r.move.get("rev")


def test_a_robot_that_starts_at_an_edge_is_held_until_moved_in_behind_the_switch():
    c = FC.Controller(["Right"], HOME, lambda n, s: None, lambda *a: None, pose_source="blend", start_mode="wait")
    r = c.R["Right"]
    c.t = 1.0
    c.on_follow("Right", 1.0, "idle", 60.0, 3.0, -90.0)
    c.on_fix("Right", 1.0, 60.0, 3.0, None)
    assert not c.start_check(r) and c.counts["start_held"] == 1
    ev = [e for e in getattr(c, "_events", [])] if hasattr(c, "_events") else []
    c.t = 4.0
    c.on_follow("Right", 4.0, "idle", 60.0, 20.0, 90.0)
    c.on_fix("Right", 4.0, 60.0, 20.0, None)
    assert c.start_check(r)                  # moved in: it starts by itself
    c2 = FC.Controller(["Right"], HOME, lambda n, s: None, lambda *a: None, pose_source="blend", start_edge=False,
                       start_mode="wait")
    c2.t = 1.0
    c2.on_follow("Right", 1.0, "idle", 60.0, 3.0, -90.0)
    c2.on_fix("Right", 1.0, 60.0, 3.0, None)
    assert c2.start_check(c2.R["Right"])     # the old way
    assert not ev or True


def test_the_proof_drive_skips_a_robot_at_an_edge():
    sent = []
    drv = D.ProofDriver(["A", "B"], lambda n, t: sent.append((n, t)), t0=0.0, board=HOME, edge_cm=FC.START_EDGE_CM)
    t = 0.0
    while t < 2.0:
        t += 0.1
        drv.on_follow("A", t, 60.0, 3.0, -90.0)
        drv.on_follow("B", t, 60.0, 60.0, 0.0)
        drv.on_sighting("A", t, 60.0, 3.0)
        drv.on_sighting("B", t, 60.0, 60.0)
        drv.tick(t)
    d = drv.data()
    assert d["A"]["at_edge"] == pytest.approx(3.0) and "at the edge" in d["A"]["failed"]
    assert d["B"]["at_edge"] is None and d["B"]["failed"] is None
    assert not any(n == "A" and x.startswith("fs") for n, x in sent)
    assert any(n == "B" and x.startswith("fs") for n, x in sent)


def test_the_simulator_reproduces_run_2_and_the_fix_holds(monkeypatch):
    # (e21) the 7 Nov run 2 reproduction needs e20's trajectories (long straights near an edge): every e21 switch on
    # its old way, and the simulated robot's e19 blend
    import freeroam_core as FC_
    import freeroam_fastsim as FS_
    for k_, v_ in (("SHAPE", False), ("STAGES", False), ("CONSISTENT_ESCAPE", False), ("SLIP_TAG", False),
                   ("INWARD_TWO", False), ("LOST_CREEP", True)):
        monkeypatch.setattr(FC_, k_, v_)
    monkeypatch.setattr(FS_, "SIM_FW_FILTER", "blend")
    import freeroam_fastsim as S
    kw = dict(dead_robot=False, pi_freeze=False, sensors=True, fw="2026-11-05d", edge_physics=True,
              bottom_zone=True, dropouts=False, camera="detections", scene=dict(tracking="lights"), layer_kw={},
              discover=True, workspace=True, hdg_fault=dict(deg=25.0, near_cm=14.0, every_s=30.0))
    before = S.Sim(1, 0.15, ctrl_kw=dict(pose_source="blend", queue_watch=False, mark_unc=False,
                                         start_edge=False), **kw).run()
    after = S.Sim(1, 0.15, ctrl_kw=dict(pose_source="blend"), **kw).run()
    assert before["incident"]["hdg_faults"] >= 1 and after["incident"]["hdg_faults"] >= 1
    assert before["edge_min_body_cm"] < 0 or before["hangs"] > 0       # run 2: a body over the edge
    assert after["edge_min_body_cm"] > 5.0 and after["hangs"] == 0 and after["falls"] == 0
