"""e12 (10 Nov 2026): the calibration's curve phase, the camera self-check and blind areas, no pictures from a camera,
one robot at an edge never holds up the others, the loud edge message, the shutdown abort and freeroam-status crash,
jay-bundle's latest, the card floor per site, new code noticed by Mission Control, the USB stick."""
import json
import math
import os
import subprocess
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
sys.path[:0] = [FLEET, TOOLS, os.path.join(TOOLS, "sim")]

import freeroam_core as FC  # noqa: E402


# -------------------------------------------------------------------------------------------- 1. the curve phase
def _arc_pts(R, sweep_deg, kr=1.0, n=40):
    Rt = R / kr
    return [(Rt * math.sin(a), Rt - Rt * math.cos(a)) for a in
            [math.radians(sweep_deg) * kr * i / n for i in range(n + 1)]]


def test_fit_arc_points_is_free_roams_fit_and_reaches_radius_50_with_its_own_floor():
    pts = _arc_pts(50.0, 60.0, kr=1.06)
    k, L = 1 / 50.0, math.radians(60.0) * 50.0
    assert FC.fit_arc_points(pts, k, L) is None                  # free roam: radius 40 and tighter only
    kr, dr, R = FC.fit_arc_points(pts, k, L, min_k=1 / 60.0)
    assert abs(kr - 1.06) < 0.01 and abs(dr - 1.0) < 0.01 and abs(R - 50 / 1.06) < 0.5
    res = FC.arc_summary([(1.06, 0.943)] * 16)
    ok, ks, ds = FC.arc_agree(res, (1.0, 1.0))
    assert ok and abs(ks - 1 / 1.06) < 1e-3 and abs(ds - 1 / 0.943) < 1e-3
    ok2, _, _ = FC.arc_agree(FC.arc_summary([(1.06, 0.943)] * 10), (1.0, 1.0))
    assert not ok2                                               # under free roam's 15 fits: not saved


def _calib():
    import calib_run as CR
    return CR


def test_arc_path_geometry_left_and_right():
    CR = _calib()
    p = CR.CalibRun.arc_path(0.0, 0.0, 0.0, 1 / 10.0, math.pi * 10 / 2)       # a quarter circle to the left
    assert abs(p[-1][0] - 10.0) < 0.05 and abs(p[-1][1] - 10.0) < 0.05
    q = CR.CalibRun.arc_path(0.0, 0.0, 90.0, -1 / 10.0, math.pi * 10 / 2)     # heading +y, curving right
    assert abs(q[-1][0] - 10.0) < 0.05 and abs(q[-1][1] - 10.0) < 0.05


def test_curve_phase_switch_auto_on_off_and_firmware(tmp_path):
    CR = _calib()
    mc = tmp_path / "mc"
    mc.mkdir()
    site = {"name": "school-conference", "mc_dir": str(mc)}

    def run(mode, fw, motion=""):
        o = types.SimpleNamespace(a=types.SimpleNamespace(arc_phase=mode, roam_motion=motion), site=site,
                                  rb=types.SimpleNamespace(name="AL-1DC0"), parent=None,
                                  fw={"AL-1DC0": fw})
        return CR.CalibRun.arc_wanted(o)
    assert run("auto", "2026-11-05d")[0]                          # nothing said anywhere: free roam's default, curves
    assert not run("auto", "2026-11-05d", "stopturn")[0]
    assert not run("auto", "2026-10-29b")[0]                      # old firmware: never
    assert not run("on", "2026-10-29b")[0]
    assert run("on", "2026-11-05c")[0] and not run("off", "2026-11-05d")[0]
    (mc / "advanced.json").write_text(json.dumps({"roam_motion": "stopturn"}))     # Nick 9 Nov at school: straight
    on, why = run("auto", "2026-11-05d")
    assert not on and "Advanced" in why
    assert CR.roam_motion_for(types.SimpleNamespace(roam_motion=""), {"roam_motion": "curves"}) == (
        "curves", "the site file's roam_motion")


def test_start_screen_passes_the_curve_switch_to_the_calibration():
    import start_job as SJ
    assert SJ.ADV_DEFAULTS["calib_curves"] == "auto"
    src = open(os.path.join(FLEET, "start_job.py")).read()
    assert '"--arc-phase", str(self.adv.get("calib_curves")' in src and '"--roam-motion"' in src
    page = open(os.path.join(FLEET, "mission_control.html")).read()
    assert 'advSel("calib_curves", a)' in page


def test_simulator_arc_error_tightens_true_curves_only():
    import robot as RB
    assert 0 < RB.ARC_ERR_MIN_K < 1 / 50.0
    src = open(os.path.join(TOOLS, "sim", "robot.py")).read()
    assert "ds /= (1.0 + ae)" in src and "not opposite" in src
    fs = open(os.path.join(TOOLS, "sim", "fleet_sim.py")).read()
    assert '"--arc-err"' in fs and 'n_ in (name, "*")' in fs


# -------------------------------------------------------------------------------------------- 2. camera self-check, blind areas
def test_blind_cells_and_the_readability_seed_carried_per_site(tmp_path, monkeypatch):
    monkeypatch.setenv("MIRLT_READMAP_STORE", "0")      # (e14) e12's readmap.json carry-over, kept on its switch
    import observe as OB
    import calib_run as CR
    rm = OB.ReadMap(200.0, 100.0)
    rm.seed(lambda x, y: 0.9, "test")
    for _ in range(6):
        rm.add(150.0, 50.0, False)
    bl = OB.blind_cells(rm)
    assert len(bl) == 1 and bl[0][0] <= 150 <= bl[0][2]
    kept = CR.save_readmap(rm, str(tmp_path / "run"), {"conf_dir": str(tmp_path / "site")})
    assert os.path.exists(tmp_path / "site" / "readmap.json") and os.path.exists(tmp_path / "site" / "blind_areas.json")
    rm2 = OB.ReadMap(200.0, 100.0)
    rm2.seed(lambda x, y: 0.9, "taken as readable")              # calib_run.readmap()'s first guess
    w = CR.seed_readmap(rm2, {"conf_dir": str(tmp_path / "site"), "tags": {}}, dry=False)
    assert "earlier runs" in w and not rm2.readable(150.0, 50.0)[0] and rm2.readable(20.0, 20.0)[0]
    assert len(kept) == 2


def test_camera_selfcheck_compare_verdict_and_off(monkeypatch):
    import numpy as np
    import camera_selfcheck as CSC
    e = {"overhead": {"20": np.array([[0, 0], [10, 0], [10, 10], [0, 10]], float)}}
    near = {"overhead": {"20": [[1, 0], [11, 0], [11, 10], [1, 10]]}}
    far = {"overhead": {"20": [[6, 0], [16, 0], [16, 10], [6, 10]]}}
    assert CSC.verdict(CSC.compare(e, near), 3.0)[0] == "ok"
    assert CSC.verdict(CSC.compare(e, far), 3.0)[0] == "refit"
    assert CSC.verdict(CSC.compare(e, {}), 3.0)[0] == "unknown"
    monkeypatch.setenv("MIRLT_CAMERA_SELFCHECK", "0")
    assert CSC.run({"name": "x"}, log=None)["result"] == "off"
    monkeypatch.setenv("MIRLT_CAMERA_SELFCHECK", "1")
    r = CSC.run({"name": "x", "conf_dir": "/nonexistent", "ref_tags": {"use": True}}, log=None)
    assert r["result"] == "unknown" and "Measure" in r["words"]


def test_calibration_uses_the_proven_workspace_and_refuses_unknown_points(tmp_path, monkeypatch):
    monkeypatch.setenv("MIRLT_SITE", os.environ.get("MIRLT_SITE", "home"))   # calib_run.main sets it: restored after
    import calib_run as CR
    import proven_workspace as PW
    import sites as SI
    site = SI.load("school-conference")
    S = CR.Site(site)
    before = S.edge_dist((30.0, 80.0))
    ws = PW.ProvenWorkspace((0.0, 0.0, S.L, S.W))
    ws.see()
    ws.drop_at(10.0, 80.0, 180.0, reach_cm=0.0, width_cm=40.0, depth_cm=5.0)    # a drop felt 10 cm in from x = 0
    p = PW.site_path(str(tmp_path), site["name"])
    ws.save(p)
    w = S.use_workspace(dict(site, conf_dir=str(tmp_path)))
    assert "added" in w and S.edge_dist((30.0, 80.0)) < before - 5.0
    try:
        CR.main(["--site", "school-conference", "--dry", "--visit-order", "P01,P99", "--session",
                 str(tmp_path / "s")])
        assert False, "should refuse"
    except SystemExit as e:
        assert "P99" in str(e)


# -------------------------------------------------------------------------------------------- 3. no pictures
def test_frame_watch_says_no_pictures_once_repeats_and_pictures_back():
    import frame_watch as FW
    clk = [100.0]
    sent, said, evs = [], [], []
    w = FW.FrameWatch(["overhead"], send=sent.append, event=lambda k, **kw: evs.append(k), say=said.append,
                      clock=lambda: clk[0], thread=False)
    clk[0] += 2.0
    assert w.check() == []
    clk[0] += 1.5
    assert w.check() == ["overhead"] and sent[-1]["state"] == "no_pictures" and "NO NEW PICTURE" in said[-1]
    clk[0] += 2.0
    assert w.check() == []                                        # not said again before REPEAT_S
    clk[0] += FW.REPEAT_S
    assert w.check() == ["overhead"] and "STILL NONE" in said[-1]
    w.saw("overhead")
    assert sent[-1]["state"] == "pictures" and sent[-1]["gap_s"] > 10 and evs[-1] == "pictures_back"
    assert FW.camera_words({"type": "camera", "words": "x"}) == "x" and FW.camera_words({"type": "fix"}) is None


def test_runs_take_the_camera_message():
    import socket
    import run_plan as RP
    src = RP.UdpSource(0)
    port = src.sock.getsockname()[1]
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.sendto(json.dumps({"type": "camera", "state": "no_pictures", "cam": "overhead"}).encode(), ("127.0.0.1", port))
    time.sleep(0.2)
    got = src.poll()
    src.close()
    assert got and got[0]["type"] == "camera"
    for f in ("freeroam.py", "calib_run.py", "run_plan.py"):
        assert 'get("type") == "camera"' in open(os.path.join(FLEET, f)).read(), f
    for f in ("tag_bridge.py", "cam_bridge.py"):
        assert "FrameWatch" in open(os.path.join(FLEET, f)).read(), f


# -------------------------------------------------------------------------------------------- 4 + 5. one robot at an edge
def _hung_sim(park=True):
    import freeroam_fastsim as S
    old = FC.EDGE_PARK
    FC.EDGE_PARK = park
    try:
        sim = S.Sim(1, 0.06, dead_robot=False, pi_freeze=False, edge_physics=True,
                    starts={"Center": (3.5, 18.6, 117.0)}, ctrl_kw=dict(start_mode="wait"))
        r = sim.run()
    finally:
        FC.EDGE_PARK = old
    return sim.ctrl, r


def test_a_robot_held_at_an_edge_is_parked_and_the_others_roam_on():
    c, r = _hung_sim(True)
    assert c.R["Center"].mode == "out" and "held at an edge" in c.R["Center"].out_why
    assert c.edge_parked and "SOMEONE MUST MOVE IT" in c.edge_parked[0]["words"] and "Center" in c.edge_parked[0]["words"]
    assert r["moving_pct"]["Left"] > 50 and r["moving_pct"]["Right"] > 50
    c2, _ = _hung_sim(False)                                      # the switch: held the whole run (before e12)
    assert c2.R["Center"].mode != "out" and not getattr(c2, "edge_parked", None)


def test_edge_parked_shown_in_status_and_on_mission_control(tmp_path, capsys):
    import freeroam as FR
    import start_job as SJ
    d = tmp_path / "run" / "fr"
    d.mkdir(parents=True)
    FR.write_edge_parked(str(d), [{"robot": "Center", "words": "Center IS PARKED AT THE BOARD'S EDGE at 3, 20"}])
    (d / "state.json").write_text(json.dumps({"t0": 1, "now": 61, "robots": {"Center": {"mode": "out", "battery": 50,
                                                                                       "moves": 3}},
                                              "edge_parked": ["Center IS PARKED AT THE BOARD'S EDGE at 3, 20"]}))
    assert FR.status(str(d)) == 0
    assert "PARKED AT THE BOARD'S EDGE" in capsys.readouterr().out
    st = SJ.Starter.__new__(SJ.Starter)
    assert st.edge_parked_words(str(tmp_path / "run"))[0].startswith("Center IS PARKED")
    page = open(os.path.join(FLEET, "mission_control.html")).read()
    assert "st.edge_parked" in page and "someone must move it before the next run" in page


# -------------------------------------------------------------------------------------------- 6. crashes
def test_freeroam_status_reads_the_start_ups_ladder_state_without_keyerror(tmp_path, capsys):
    import freeroam as FR
    (tmp_path / "state.json").write_text(json.dumps({"robots": {"Left": {"x": 1, "y": 2}, "Right": {}},
                                                      "passed": {"L0:Left": True}}))
    assert FR.status(str(tmp_path)) == 0
    assert "starting" in capsys.readouterr().out
    (tmp_path / "state.json").write_text(json.dumps({"t0": 0, "now": 120, "robots": {"Left": {"mode": "idle"}}}))
    assert FR.status(str(tmp_path)) == 0


def test_orderly_exit_ends_with_a_thread_still_running(tmp_path):
    code = ("import sys, threading, time; sys.path.insert(0, %r)\n"
            "threading.Thread(target=lambda: time.sleep(60), daemon=True).start()\n"
            "import orderly_exit as OX; OX.finish(3, wait_s=0.3)\n" % FLEET)
    t0 = time.time()
    p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=30)
    assert p.returncode == 3 and time.time() - t0 < 10
    for f in ("tag_bridge.py", "cam_bridge.py", "freeroam.py"):
        assert "OX.finish(_code)" in open(os.path.join(FLEET, f)).read(), f
    assert "_orderly_exit(" in open(os.path.join(TOOLS, "pi", "camrec.py")).read()
    assert "a._ladder.drop_eye()" in open(os.path.join(FLEET, "freeroam.py")).read()


# -------------------------------------------------------------------------------------------- 7. jay-bundle latest
def test_latest_run_finds_command_line_and_mission_control_runs(tmp_path, monkeypatch):
    import run_exports as RE
    mc = tmp_path / "logs" / "mission-control-no-such-site" / "runs" / "20261110-090000-visit-robots"
    cl = tmp_path / "logs" / "20261110-100000-mission"
    for d in (mc, cl):
        (d / "run").mkdir(parents=True)
        (d / "run" / "stops.csv").write_text("robot,point\nAlpha,P01\n")
    os.utime(mc, (time.time() - 100, time.time() - 100))
    monkeypatch.setenv("MIRLT_SITE", "no-such-site")
    d, where = RE.latest_run(site="no-such-site", repo=str(tmp_path))
    assert d == str(cl) and where == "command line"
    os.utime(mc, (time.time() + 100, time.time() + 100))
    assert RE.latest_run(site="no-such-site", repo=str(tmp_path))[1] == "Mission Control"


# -------------------------------------------------------------------------------------------- 8. housekeeping
def test_card_floor_per_site_env_overrides(monkeypatch):
    import diskguard as DG
    monkeypatch.delenv("MIRLT_CARD_STOP_GB", raising=False)
    assert DG.card_stop_gb({"name": "school-conference", "card_stop_gb": 0.55})[0] == 0.55
    assert DG.card_stop_gb({"name": "home"})[0] == 1.0
    monkeypatch.setenv("MIRLT_CARD_STOP_GB", "0.8")
    assert DG.card_stop_gb({"card_stop_gb": 0.55}) == (0.8, "MIRLT_CARD_STOP_GB")
    monkeypatch.setenv("MIRLT_CARD_STOP_GB", "0.1")
    assert DG.card_stop_gb({})[0] == DG.CARD_HARD_GB
    import sites as SI
    assert SI.load("school-conference")["card_stop_gb"] == 0.55


def test_mission_control_notices_new_code_and_never_restarts_mid_run(monkeypatch):
    import mission_control as MC
    stamps = [("aaaa1111", 1.0)]
    monkeypatch.setattr(MC, "code_stamp", lambda repo=None: stamps[-1])
    cw = MC.CodeWatch()
    assert not cw.status(False)["changed"]
    stamps.append(("bbbb2222", 2.0))
    cw.t = 0.0
    s = cw.status(False)
    assert s["changed"] and "Restart Mission Control now" in s["words"]
    cw.t = 0.0
    assert "never mid-run" in cw.status(True)["words"]
    busy = types.SimpleNamespace(start=types.SimpleNamespace(busy=lambda: True), run=None,
                                 calib=types.SimpleNamespace(running=lambda: False))
    assert MC.run_going(busy)
    src = open(os.path.join(FLEET, "mission_control.py")).read()
    assert 'if path == "/api/restart":' in src and "if run_going(S):" in src


def test_usb_stick_script_checks_without_sudo_and_launcher_warns(tmp_path):
    sh = os.path.join(TOOLS, "pi", "mount_fleet.sh")
    p = subprocess.run(["bash", "-n", sh], capture_output=True, text=True)
    assert p.returncode == 0
    p = subprocess.run(["bash", sh, "--check"], capture_output=True, text=True,
                       env=dict(os.environ, FLEET_LABEL="NO-SUCH-LABEL-E12"))
    assert p.returncode == 1 and "NOT mounted" in p.stdout
    sh2 = open(os.path.join(REPO, "run-fleet-pi.sh")).read()
    assert "mount_fleet.sh --check" in sh2 and "USB stick FLEET is NOT mounted" in sh2
    doc = open(os.path.join(REPO, "docs", "how-to", "usb-stick.md")).read()
    assert "LABEL=FLEET" in doc and "nofail" in doc


def test_run_evidence_overturns_the_cautious_seed_next_run(tmp_path):
    """9 Nov school: the seed flagged P12 (about 150, 84 cm) unreadable; every stop there had camera fixes. The run's
    stops and camera fixes must make it readable in the next run's map (seed + this site's history at half weight)."""
    import observe as OB
    run = tmp_path / "run" / "run"
    run.mkdir(parents=True)
    with open(run / "stops.csv", "w") as f:
        f.write("robot,n,point,cam_x_cm,cam_y_cm,n_fixes\n")
        for k in range(7):
            f.write("Alpha,%d,P12,%.1f,84.0,40\n" % (k, 150.0 + 0.3 * k))
    with open(run / "camera_track.csv", "w") as f:
        f.write("t,robot,x_cm,y_cm\n")
        for k in range(200):
            f.write("%d,Alpha,%.1f,%.1f\n" % (k, 120 + k * 0.3, 84.0))
    seed = lambda x, y: 0.1                                       # noqa: E731  (the cautious camera-model seed)
    rm = OB.ReadMap(396.0, 167.0)
    rm.seed(seed, "too cautious")
    assert not rm.readable(150.0, 84.0)[0]
    nf, ns = OB.learn_from_logs(rm, str(tmp_path / "run"))
    assert ns == 7 and nf > 0 and rm.readable(150.0, 84.0)[0]
    nxt = OB.ReadMap(396.0, 167.0)
    nxt.seed(seed, "too cautious")
    assert nxt.merge_history(OB.ReadMap.from_json(rm.to_json()))
    assert nxt.readable(150.0, 84.0)[0], nxt.readable(150.0, 84.0)
    assert not nxt.readable(300.0, 150.0)[0]                      # nothing seen there: the seed still stands
    assert "learn_readmap(args, out, sess)" in open(os.path.join(FLEET, "mission_run.py")).read()


# -------------------------------------------------------------------------------------------- 9. the P15 lock (9 Nov)
def _coord():
    import run_plan as RP
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.args = types.SimpleNamespace(cam_guard_cm=6.0)
    c.source = types.SimpleNamespace(live=True)
    said, logs, sent, ctl = [], [], [], []
    c.say, c.log = said.append, (lambda ev, **kw: logs.append((ev, kw)))
    c.send = lambda n, t: sent.append(t)
    c.control = lambda b, t: ctl.append(t)
    c.holds_one = lambda: True
    c.keep_going = lambda: False
    c.pose_at = lambda b, t: None
    b = RP.Bot.__new__(RP.Bot)
    b.name, b.out, b.own_until, b.backoff, b.pose_t = "Alpha", False, 0.0, None, 0.0
    c.bots = {"Alpha": b}
    return RP, c, b, said, logs, sent


def _fix(c, b, x, y, src, th=None):
    c.cam_seen(b, {"x": x, "y": y, "th": th, "src": src})


def test_a_lock_only_fix_never_resets_the_pose_but_a_tag_read_does(monkeypatch):
    RP, c, b, said, logs, sent = _coord()
    t0 = time.time()
    # the robot stands at (180.4, 151.9) by its own pose and by a tag read; the lock then says (186.2, 154.4)
    b.pose, b.pose_t = (180.4, 151.9, 90.0), t0
    b.last_tag = (t0 - 5.0, 180.4, 151.9)                  # the last tag read is old (weak reads near P15)
    b.last_cam, b.last_cam_src, b.cam_t = [186.16, 154.41, None], "tag_bridge:lock", t0
    b.guard_since = t0 - 1.5
    c.cam_guard()
    assert b.pose[:2] == (180.4, 151.9) and not any(s.startswith("fpose") for s in sent)
    assert logs and logs[-1][0] == "cam_guard_lock_declined"
    # a roof-tag read 7 cm off is believed as before (the guard's own job)
    b.last_cam, b.last_cam_src, b.cam_t, b.guard_since = [186.16, 154.41, 90.0], "tag_bridge:overhead", time.time(), \
        time.time() - 1.5
    b.pose_t = time.time()
    c.cam_guard()
    assert b.pose[0] == 186.16 and any(s.startswith("fpose") for s in sent)
    # the switch: the old way believes the lock too
    monkeypatch.setattr(RP, "LOCK_GUARD", False)
    b.pose, b.pose_t, b.guard_resets = (180.4, 151.9, 90.0), time.time(), []
    b.last_cam, b.last_cam_src, b.cam_t, b.guard_since = [186.16, 154.41, None], "tag_bridge:lock", time.time(), \
        time.time() - 1.5
    c.cam_guard()
    assert b.pose[0] == 186.16


def test_a_lock_agreeing_with_a_recent_tag_read_is_trusted():
    RP, c, b, *_ = _coord()
    t = time.time()
    b.last_cam_src, b.last_tag = "tag_bridge:lock", (t - 0.5, 180.0, 151.0)
    assert c.lock_trusted(b, [181.0, 152.0, None], t) and not c.lock_trusted(b, [186.2, 154.4, None], t)


def test_an_interrupted_dwell_where_the_robot_stayed_is_one_full_stop(tmp_path):
    RP, c, b, said, logs, sent = _coord()
    rec = []
    c.record_point = lambda b_, p, t0, t1, interrupted=False: rec.append((p, round(t1 - t0, 1), interrupted))
    b.pose, b.last_tag = (180.4, 151.9, 90.0), (time.time(), 180.4, 151.9)
    stp = {"label": "at P15", "type": "wait"}
    t0 = time.time()
    c.point_step(b, stp, "started", t0)
    b.last_tag = (time.time(), 180.9, 152.3)                 # it stayed within 2 cm (tag reads)
    c.point_step(b, stp, "started", t0 + 4.0)               # the halt and the replanned W 4000
    c.point_step(b, stp, "done", t0 + 8.0)
    assert rec == [("P15", 8.0, True)]
    # moved 7 cm (a real move): a new dwell
    rec.clear()
    c.point_step(b, stp, "started", t0)
    b.last_tag = (time.time(), 187.4, 151.9)
    c.point_step(b, stp, "started", t0 + 4.0)
    c.point_step(b, stp, "done", t0 + 8.0)
    assert rec == [("P15", 4.0, False)]
    import run_exports as RE
    assert RE.STOP_COLS[-1] == "interrupted"


def test_simulator_lock_latches_on_a_fixed_spot():
    import fleet_sim as FS
    import random
    sim = FS.FleetSim.__new__(FS.FleetSim) if hasattr(FS, "FleetSim") else None
    src = open(os.path.join(TOOLS, "sim", "fleet_sim.py")).read()
    assert "def lock_latch(self, rb, rng)" in src and '"--lock-latch"' in src
    if sim is None:
        return
    sim.args = types.SimpleNamespace(lock_latch=["186.2,154.4,15,1.0"])
    near = types.SimpleNamespace(x=180.0, y=151.0)
    far = types.SimpleNamespace(x=100.0, y=100.0)
    rng = random.Random(1)
    assert sim.lock_latch(near, rng) == (186.2, 154.4) and sim.lock_latch(far, rng) is None
