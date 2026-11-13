"""Build e19 (12 Nov): the Fable audit's fixes (docs/audits/2026-11-12-fable-audit.md), safety first. Each test fails on
the code before its fix (main at c79c1df). Simulator, test rig and saved logs only: no robot, no Pi, no camera.

    python3 -m pytest tools/fleet/tests/test_e19_audit_fixes.py -q
"""
import json
import math
import os
import shutil
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)

ROBOTS = ("Left", "Center", "Right")


def _unshare_ok():
    if not shutil.which("unshare"):
        return False
    try:
        return subprocess.run(["unshare", "-rn", "true"], capture_output=True, timeout=10).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


needs_ns = pytest.mark.skipif(not _unshare_ok(), reason="needs `unshare -rn` (its own network for the robots' ports)")


def dry_end(tmp_path, mode, secs, home=None):
    """One simulated free roam through e19_end_harness.py, with its own HOME (this machine's ~/.config/mirlt is never
    touched). Returns the harness's record."""
    home = home or str(tmp_path / "home")
    os.makedirs(home, exist_ok=True)
    env = dict(os.environ, HOME=home, MPLCONFIGDIR=str(tmp_path / "mpl"))
    env.pop("MIRLT_CONF_DIR", None)
    out = str(tmp_path / ("end-%s.json" % mode))
    p = subprocess.run(["unshare", "-rn", sys.executable, os.path.join(HERE, "e19_end_harness.py"), out,
                        str(tmp_path / ("run-" + mode)), mode, str(secs)], env=env, capture_output=True, text=True,
                       timeout=240)
    assert os.path.exists(out), p.stdout[-3000:] + p.stderr[-3000:]
    rec = json.load(open(out))
    rec["stdout"] = p.stdout
    return rec


# ------------------------------------------------------------------------------- 1 (audit H1): stop first at every end
@needs_ns
def test_sigterm_with_a_full_disk_still_sends_stop_to_every_robot_before_saving(tmp_path):
    """Audit H1: SIGTERM, then every end-of-run write fails with "No space left on device". Before e19 the state write
    raised first and `stop` was never sent. Now `stop` goes to every robot first, before any save is tried."""
    rec = dry_end(tmp_path, "sigterm-diskfull", 3)
    t_sig = rec["T"]["signal"]
    assert "roam" in rec["T"] and "end_save" in rec["T"]       # stopped mid-roam: the corrections' save was tried
    stops = [(t, n) for t, n, c in rec["sent"] if c == "stop" and t >= t_sig]
    assert {n for _, n in stops} == set(ROBOTS), rec["stdout"][-2000:]
    first = min(t for t, _ in stops)
    for step in ("end_save", "write_state", "save_workspace"):
        if step in rec["T"]:
            assert first <= rec["T"][step], step
    assert first - t_sig < 1.0
    assert "close" in rec["T"]                 # the later steps still ran after the failing ones


@needs_ns
def test_the_time_cap_sends_stop_before_the_corrections_are_saved(tmp_path):
    """Audit H1 (0.37 s on a normal end): the run's own end now sends stop before end_save's commands."""
    rec = dry_end(tmp_path, "timecap", 22)
    assert "end_save" in rec["T"], rec["stdout"][-2000:]
    stops = [t for t, n, c in rec["sent"] if c == "stop" and t >= rec["T"]["roam"]]
    assert stops and min(stops) <= rec["T"]["end_save"]
    later = [c for t, n, c in rec["sent"] if t < min(stops) and t >= rec["T"]["end_save"]]
    assert not later


CALIB_ERR = r'''
import json, os, sys, types
sys.path.insert(0, sys.argv[1] + "/tools/fleet"); sys.path.insert(0, sys.argv[1] + "/tools")
import calib_run as CR
sent = []
class L:
    def send(self, ip, c): sent.append((ip, c))
def run(self, eye_factory):
    self.robots = [types.SimpleNamespace(link=L(), ip="127.0.0.%d" % k, name="R%d" % k) for k in (1, 2)]
    raise RuntimeError("boom (an unexpected error mid-calibration)")
CR.CalibRun.run = run
try:
    CR.main(["--dry", "--session", sys.argv[2]])
except BaseException as e:
    print("raised", repr(e))
json.dump(sent, open(sys.argv[3], "w"))
'''


@needs_ns
def test_calibration_ending_in_an_unexpected_error_sends_stop_to_every_robot(tmp_path):
    """Audit H1, calibration: only Ctrl+C / SIGTERM sent stop before e19; an unexpected error closed the link without
    one (the robots drove on until their watchdogs). Now stop goes first at every ending."""
    h = tmp_path / "h.py"
    h.write_text(CALIB_ERR)
    out = tmp_path / "sent.json"
    env = dict(os.environ, HOME=str(tmp_path / "home"), MPLCONFIGDIR=str(tmp_path / "mpl"))
    env.pop("MIRLT_CONF_DIR", None)
    p = subprocess.run(["unshare", "-rn", sys.executable, str(h), REPO, str(tmp_path / "s"), str(out)], env=env,
                       capture_output=True, text=True, timeout=180)
    sent = json.load(open(out))
    assert ["127.0.0.1", "stop"] in sent and ["127.0.0.2", "stop"] in sent, p.stdout[-1500:] + p.stderr[-1500:]


def test_a_mission_ends_the_ctrl_c_way_on_sigterm():
    """Audit 17 (missions): run_plan had no SIGTERM handler, so `timeout` / systemd / a Stop by signal killed it
    without its finally (no stop to the robots). Now SIGTERM raises KeyboardInterrupt like Ctrl+C and SIGHUP."""
    import signal
    import types
    import run_plan as RP
    old = {s: signal.getsignal(s) for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGUSR1, signal.SIGUSR2)}
    try:
        RP.install_signals(types.SimpleNamespace())
        with pytest.raises(KeyboardInterrupt):
            os.kill(os.getpid(), signal.SIGTERM)
            for _ in range(100):
                pass
    finally:
        for s, h in old.items():
            signal.signal(s, h)


# ------------------------------------------------------- 2 (audit H2): dry runs never write the real robots' settings
def _tree(d):
    return sorted(os.path.relpath(os.path.join(a, f), d) for a, _, fs in os.walk(d) for f in fs) if os.path.isdir(d) \
        else []


@needs_ns
def test_a_dry_free_roam_leaves_the_real_settings_folder_unchanged(tmp_path):
    """Audit H2: three dry runs added rows to ~/.config/mirlt/robots/Left.json, rewrote workspace/home.json, and (found
    here) put the simulated robots' 127.0.0.x addresses into heard_robots.json, the emergency stop's list. Now a dry run
    writes into its own folder (~/.config/mirlt-dry) and never remembers a loopback address."""
    home = tmp_path / "home"
    real = home / ".config" / "mirlt"
    (real / "robots").mkdir(parents=True)
    (real / "robots" / "Left.json").write_text(json.dumps({"robot": "Left", "runs": [
        {"wheel": 1.004, "wheel_se": 0.004, "gyro": 0.998, "gyro_se": 0.004, "t": "2026-11-11 08:58:00",
         "source": "real", "run": "20261111-085800-freeroam"}]}))
    (real / "heard_robots.json").write_text(json.dumps({"Left": {"ip": "192.168.1.51", "t": 1.0}}))
    before = {p: open(os.path.join(real, p)).read() for p in _tree(str(real))}
    rec = dry_end(tmp_path, "plain", 4, home=str(home))
    assert "roam" in rec["T"], rec["stdout"][-2000:]
    after = {p: open(os.path.join(real, p)).read() for p in _tree(str(real))}
    assert after == before, sorted(set(after) ^ set(before)) or [p for p in after if after[p] != before.get(p)]
    dry = home / ".config" / "mirlt-dry"
    assert any(p.startswith("workspace") for p in _tree(str(dry))), _tree(str(dry))


def test_the_check_finds_untagged_dry_rows_and_the_repair_backs_up_and_never_deletes(tmp_path):
    """Audit H2's repair. Rows written before e19 carry no source: they are judged by the run folders (a dry run's
    discovery.json holds the same fit; else the dry run's time window). The repair copies each file first, moves dry rows
    into runs_quarantined_dry (kept), pools again from the real rows, sets the dry-touched workspace aside and drops
    loopback addresses from the emergency stop's list into quarantined_dry."""
    import dry_check as DC
    conf, logs = tmp_path / "conf", tmp_path / "logs"
    (conf / "robots").mkdir(parents=True)
    (conf / "workspace").mkdir()
    real_fit = {"wheel": 1.006, "wheel_se": 0.003, "gyro": 1.001, "gyro_se": 0.003}
    dry_fit = {"wheel": 0.951, "wheel_se": 0.002, "gyro": 0.990, "gyro_se": 0.002}
    timed_fit = {"wheel": 0.960, "wheel_se": 0.002}
    for name, fit, dry, t0 in (("20261111-085800-freeroam", real_fit, False, "2026-11-11T08:58:00"),
                               ("20261112-101500-freeroam-dryrun", dry_fit, True, "2026-11-12T10:15:00"),
                               ("20261112-110000-freeroam-dryrun", {}, True, "2026-11-12T11:00:00")):
        (logs / name).mkdir(parents=True)
        (logs / name / "run.json").write_text(json.dumps({"dry": dry, "started": t0, "finished": t0[:-5] + "30:00"}))
        (logs / name / "discovery.json").write_text(json.dumps({"fits": {"Left": fit} if fit else {}}))
        if dry:
            (logs / name / "workspace.json").write_text("{}")
    rows = [dict(real_fit, t="2026-11-11 09:00:01"), dict(dry_fit, t="2026-11-12 10:16:00"),
            dict(timed_fit, t="2026-11-12 11:05:00"), {"wheel": 1.02, "wheel_se": 0.01, "t": "2026-10-01 09:00:00"}]
    (conf / "robots" / "Left.json").write_text(json.dumps({"robot": "Left", "runs": rows, "pooled": {"x": 1}}))
    (conf / "workspace" / "home.json").write_text(json.dumps({"board": [0, 0, 1, 1], "rle": []}))
    (conf / "heard_robots.json").write_text(json.dumps({"Left": {"ip": "127.0.0.2", "t": 2.0},
                                                         "Right": {"ip": "192.168.1.53", "t": 1.0}}))
    rep = DC.check(str(conf), str(logs))
    kinds = [q["kind"] for q in rep["robots"]["Left"]["rows"]]
    assert kinds == ["real", "dry", "dry", "unknown"]
    assert rep["contaminated"] and rep["workspace"]["home.json"]["dry"] and rep["heard"]["loopback"] == ["Left"]
    n_files = len(_tree(str(conf)))
    rep2, lines = DC.repair(str(conf), str(logs), stamp="T")
    d = json.load(open(conf / "robots" / "Left.json"))
    assert len(d["runs"]) == 2 and len(d["runs_quarantined_dry"]) == 2          # moved, not deleted
    assert d["pooled"]["wheel"]["runs"] == 2 and abs(d["pooled"]["wheel"]["value"] - 1.0) < 0.02
    bk = conf / "backup-dry-check-T"
    assert json.load(open(bk / "robots" / "Left.json"))["runs"] == rows       # the original, byte for byte in content
    assert (bk / "workspace" / "home.json").exists() and (conf / "workspace" / "home.json.dry-quarantined").exists()
    h = json.load(open(conf / "heard_robots.json"))
    assert "Left" not in h and h["quarantined_dry"]["Left"]["ip"] == "127.0.0.2" and h["Right"]["ip"] == "192.168.1.53"
    assert len(_tree(str(conf))) >= n_files                                      # nothing deleted
    assert not DC.check(str(conf), str(logs))["contaminated"]


def test_pool_rows_and_workspace_saves_record_their_source(tmp_path):
    import proven_workspace as PW
    import start_discovery as D
    D.pool_fit(str(tmp_path), "Left", {"wheel": 1.01, "wheel_se": 0.004}, source="dry", run="R1")
    row = json.load(open(D.cal_path(str(tmp_path), "Left")))["runs"][-1]
    assert row["source"] == "dry" and row["run"] == "R1"
    ws = PW.ProvenWorkspace((0.0, 0.0, 100.0, 100.0))
    p = PW.site_path(str(tmp_path), "home")
    ws.save(p, source=dict(run="R1", dry=True))
    ws.save(p, source=dict(run="R2", dry=False))
    assert [(s["run"], s["dry"]) for s in json.load(open(p))["saved_by"]] == [("R1", True), ("R2", False)]


# ------------------------------------------------- 5 (audit H5): a hung robot never holds the creep token for good
BOARD = (0.0, 0.0, 122.24, 121.92)


def _two_unaccounted():
    import freeroam_core as FC
    sent = []
    c = FC.Controller(["Left", "Right"], BOARD, lambda n, s: sent.append((n, s)), lambda *a: None)
    c.never_still = True
    c.t = 100.0
    for n, (x, y) in (("Left", (6.0, 60.0)), ("Right", (110.0, 15.0))):
        r = c.R[n]
        r.mode = "idle"
        r.own = (c.t, x, y, 0.0)
        r.own_hist = [(c.t - 1.0, x, y, 0.0), (c.t, x, y, 0.0)]
        c.start_lost(r, "test")
        r.bubble = 3.0
        r.id_pause_from = c.t - 100.0           # (item 8's identification pause) over: the token rules are tested here
    return c


def test_a_free_robot_creeps_while_a_hung_robot_holds_the_token():
    """Audit H5: with two robots unaccounted, hung Left re-crept in vain before its token ran out and so held it for
    good; free Right was refused 5,794 of 5,912 ticks (311 s). Now a stuck holder (tilted, recovering, or its last
    moves did not move) releases the token at once."""
    c = _two_unaccounted()
    L, R = c.R["Left"], c.R["Right"]
    L.tilted = True                                           # hung at the x = 0 edge
    L.move = dict(kind="creep back", L=2.0, v=2.0, turn=0.0)  # its last creep, sent and going nowhere
    c.creep_token = ("Left", c.t + 600.0)
    R.still_since = c.t - 3.0 * c.still_pause(R) + 0.01        # inside its third pause
    called = []
    c.creep_step = lambda r, why: called.append(r.name) or False
    c.never_still_watch(R)
    assert called == ["Right"]


def test_a_robot_whose_moves_do_not_move_is_stuck_and_never_takes_the_token():
    c = _two_unaccounted()
    L, R = c.R["Left"], c.R["Right"]
    L.nomove_n = c.STUCK_N                                    # its last moves "did not move" (fast_done)
    L.move = dict(kind="creep", L=2.0, v=2.0, turn=0.0)
    c.creep_token = ("Left", c.t + 600.0)
    R.still_since = c.t - c.still_pause(R) - 0.01
    called = []
    c.creep_step = lambda r, why: called.append(r.name) or False
    c.never_still_watch(R)
    assert called == ["Right"]
    # the stuck robot's own creep does not take the token
    L.move = None
    L.still_since = c.t - 10.0
    c.creep_token = None

    def step(r, why):
        r.move = dict(kind="creep", L=2.0, v=2.0, turn=0.0)
        return True
    c.creep_step = step
    c.never_still_watch(L)
    assert c.__dict__.get("creep_token") is None


def test_a_live_creep_still_holds_the_token_for_its_length():
    """The token's purpose stays: one unaccounted robot creeps at a time while its creep is really under way."""
    c = _two_unaccounted()
    L, R = c.R["Left"], c.R["Right"]
    L.move = dict(kind="creep", L=2.0, v=2.0, turn=0.0)
    c.creep_token = ("Left", c.t + 3.0)
    R.still_since = c.t - c.still_pause(R) - 0.01
    called = []
    c.creep_step = lambda r, why: called.append(r.name) or False
    c.never_still_watch(R)
    assert called == []


# ------------------------------------------------- 6 (audit H6): a hung robot is never given up on
def test_a_failed_recovery_try_is_forgotten_at_the_next_round():
    """Audit H6a: a failure was remembered for ever at that spot, so every 20 s retry skipped all its tries (skew seed
    2: "pivot_right: skipped" 54 times). Now a round counts only its own failures."""
    import observe as OB
    fm = OB.FailMemory()
    fm.add(10.0, "recover:pivot_right", 1, 15.0, 3.0, 60.0)
    assert fm.same("recover:pivot_right", 1, 15.0, 3.0, 60.0)                      # inside its round
    assert not fm.same("recover:pivot_right", 1, 15.0, 3.0, 60.0, since=30.0)      # a later round tries it again


def test_a_robot_that_backed_over_tries_forward_first_and_rounds_grow():
    """Nick 12 Nov: "back itself up or do the opposite of whatever got it to tilt". Backed over (nose inward): forward
    tries first; turned over: the pivot the other way first; each round longer (recover_scale)."""
    import safety_rules as SR
    t = SR.tilt_recovery_tries_e19(6.0, 0.0, False, cause="reverse")
    assert t[0] == "forward" and "hold" == t[-1] and not any(x.startswith("reverse") for x in t)
    t2 = SR.tilt_recovery_tries_e19(180.0, 0.0, True, cause="turn_left")
    assert t2[0] == "pivot_right"
    assert SR.recover_scale(1) == 1.0 < SR.recover_scale(2) < SR.recover_scale(3) <= SR.RECOVER_GROW_MAX


def _hang_sim(seed):
    import freeroam_fastsim as F
    s = F.Sim(seed, 0.04, edge_physics=True, dead_robot=False, pi_freeze=False,
              starts={"Center": (0.4, 60.0, 6.0), "Left": (90.0, 80.0, 40.0), "Right": (70.0, 30.0, -70.0)},
              ctrl_kw=dict(start_edge=False))
    s.run()
    return s


@pytest.mark.parametrize("seed", [1, 2])
def test_a_robot_hung_after_backing_over_the_edge_is_freed_in_the_simulator(seed):
    """Audit H6b + the simulator's fairness: a robot hung with its nose 6 deg from straight inward (it backed over) had
    no way out (no try drove forward, and the simulator freed only reverses): hung all 144 s on c79c1df. Now its forward
    tries (tilt stop raised for the step) and the symmetric hang model free it, with no fall."""
    s = _hang_sim(seed)
    b = s.bots["Center"]
    assert not getattr(b, "hung", False) and s.stats["falls"] == 0
    assert s.ctrl.counts.get("recovered", 0) >= 1


# ------------------------------------------- 3 (audit H4): no far jump without motion proof, from the robot's own doubt
def _far_rig19(proof, monkeypatch, secs=12.0):
    import e17_harness as H
    import freeroam_core as FC
    monkeypatch.setattr(FC, "MOTION_PROOF", proof)
    R = H.Rig({"Left": (30.0, 30.0, 0.0), "Center": (60.0, 90.0, 90.0), "Right": (95.0, 30.0, 180.0)}, seed=2)
    R.run(3.0)
    R.off["Center"] = [31.0 * 0.6, -31.0 * 0.8, 0.0]       # its own pose 31 cm off, the robot standing (11 Nov 08:26)
    R.run(secs)
    return R


def test_named_far_beyond_its_doubt_is_proven_by_an_out_and_back_before_any_correction(monkeypatch):
    """Audit H4: "named far" had no upper limit and no motion proof (3 of the 7 falls: 24-91 cm wrong). Now, beyond the
    robot's stated doubt (5 cm here), it drives a short out-and-back first; the 11 Nov 08:26 case still ends corrected."""
    # (e21) this tests the e20 mechanism on its own numbers; the e21 way: test_e21_error_shape.py
    import freeroam_core as FC_
    monkeypatch.setattr(FC_, "CONSISTENT_ESCAPE", False)
    R = _far_rig19(True, monkeypatch)
    ev = [e for e in R.events if e[1] == "Center"]
    assert not any(e[2] == "CORRECTED" and "named it all along" in e[3] for e in ev)      # never at once
    asked = [e for e in ev if e[2] == "identity drive" and "motion proof" in e[3]]
    assert asked and R.c.counts.get("proofs_asked", 0) >= 1
    assert R.own_gap("Center") < 3.5


def _proof_ctrl():
    import freeroam_core as FC
    c = FC.Controller(["A", "B"], BOARD, lambda n, s: None, lambda *a: None)
    r = c.R["A"]
    c.t = 10.0
    r.own_hist = [(10.0 + 0.1 * k, 40.0 + min(k, 30 - k) * 0.2, 60.0, 0.0) for k in range(31)]   # 3 cm out and back
    r.proof = dict(t0=10.0, x=80.0, y=60.0, sig=0.8, L=3.0, win=3.0, pts=[])
    return c, r


def test_the_proof_passes_only_when_the_named_sighting_goes_out_and_back_with_the_drive():
    c, r = _proof_ctrl()
    done = []
    c.correct_from_camera = lambda *a, **k: done.append(a)
    for k in range(31):                               # the named sighting moves with the robot's own drive
        c.t = 10.0 + 0.1 * k
        c.proof_watch(r, c.t, 80.0 + min(k, 30 - k) * 0.2, 60.0)
    c.t = 13.1
    c.proof_watch(r, 13.1, 80.0, 60.0)
    assert done and "proven" in done[0][-1]
    c, r = _proof_ctrl()                              # a sighting that stays put (another object, a mirrored spot)
    done = []
    c.correct_from_camera = lambda *a, **k: done.append(a)
    for k in range(32):
        c.t = 10.0 + 0.1 * k
        c.proof_watch(r, c.t, 80.0, 60.0)
    assert not done and c.counts.get("proofs_failed") == 1


def test_an_unconfirmed_correction_permits_no_reverse_creep_toward_an_edge(monkeypatch):
    """Audit H4: 5 of the 7 falls were a "creep back" after a wrong correction (no rear sensor). Until the robot's own
    move confirms the corrected pose, a creep never reverses toward an edge."""
    # (e21) this tests the e20 mechanism on its own numbers; the e21 way: test_e21_error_shape.py
    import freeroam_core as FC_
    monkeypatch.setattr(FC_, "LOST_CREEP", True)          # e17's creep (no edge rule), behind its switch
    import freeroam_core as FC
    sent = []
    c = FC.Controller(["A", "B"], BOARD, lambda n, s: sent.append((n, s)), lambda *a: None)
    c.t = 50.0
    r = c.R["A"]
    r.mode = "idle"
    r.own = (c.t, 14.0, 60.0, 0.0)                 # nose +x, its back to the x = 0 edge 14 cm away
    r.own_hist = [(c.t - 1.0, 14.0, 60.0, 0.0), (c.t, 14.0, 60.0, 0.0)]
    c.R["B"].mode = "gone"
    c.creep_target = lambda r_, p: ((4.0, 60.0), "behind it")
    r.corr_unconf = dict(t=c.t, how="named far", d=40.0, drift=None)
    c.creep_step(r, "test")
    assert r.move is not None and not r.move["rev"]                 # it turns and drives forward instead
    assert c.counts.get("unconf_back_refused", 0) >= 1
    r.corr_unconf = None
    r.move = None
    c.creep_step(r, "test")
    assert r.move is not None and r.move["rev"]                     # confirmed: it may reverse (as before)


def test_an_elimination_beyond_the_robots_doubt_asks_for_motion_proof_in_the_layer():
    """Audit H4 (elimination, 1 of the 7 falls: 87 cm wrong): the layer names by elimination alone only within the
    robot's stated doubt; beyond it the same wiggle / out-and-back as an ambiguous case decides."""
    import tracking_layer as TL
    asked = []
    L = TL.TrackingLayer(["Left", "Right"], ("motion", "lights"), board=BOARD, camera_watch=False,
                         wiggle=lambda n, why: asked.append(n) or True)
    L.proof_fn = lambda n, sg: 5.0
    named = []
    for k in range(80):
        t = 0.2 * k
        L.own("Left", t, 30.0, 30.0, 0.0, True)
        L.own("Right", t, 100.0, 100.0, 0.0, True)
        L.set_unaccounted("Left", "lost")
        dets = [TL.Det("lights", t, 100.0 + 0.1 * (k % 3), 100.0, None, None, 0.4),
                TL.Det("lights", t, 60.0 + 0.1 * (k % 2), 70.0, None, None, 0.4)]
        out = L.feed(t, dets)
        named += [(n, h) for n, (d, h) in (out or {}).items() if h] if isinstance(out, dict) else []
    assert asked and not any(h == "elimination" for n, h in named)
    assert L.counts.get("elim_needs_proof", 0) >= 1


# ------------------------------------------- 7 (audit 11): after a restart, its pose is sent and a heading check asked
def test_after_a_restart_its_pose_is_sent_and_a_heading_check_is_asked():
    """Audit finding 11 / hard simulator #8: on_boot set the safety layers and the follow run again, sent no pose and
    asked no heading check; e18's "keep its own heading" then rated the boot heading as good evidence."""
    import freeroam_core as FC
    sent = []
    c = FC.Controller(["A", "B"], BOARD, lambda n, s: sent.append((n, s)), lambda *a: None)
    c.t = 20.0
    r = c.R["A"]
    r.mode = "idle"
    r.own = (c.t, 40.0, 50.0, 30.0)
    r.own_hist = [(c.t, 40.0, 50.0, 30.0)]
    c.on_restart(r)
    assert c.heading_doubt(r) == FC.DOUBT_DEG_UNKNOWN and c.head_sd(r) > 10.0   # its heading counts as unknown
    c.follow_again(r)
    c.t += 1.5
    c.boot_watch(r)
    fp = [s for n, s in sent if s.startswith("fpose")]
    assert fp and fp[-1].split()[1:4] == ["40.00", "50.00", "30.00"]


# ------------------------------------------- 6 (audit 10, 14): a wrong or bumped camera; the side camera never the overhead
def _drive_scene(mirror_from, mirror_to, secs=20.0):
    import tracking_layer as TL
    said = []
    L = TL.TrackingLayer(["Left", "Center", "Right"], ("motion", "lights"), board=BOARD, camera_watch=False,
                         eliminate=False)
    L.say_fn = said.append
    L.proof_fn = lambda n, sg: 4.0
    W = BOARD[2]
    starts = {"Left": (20.0, 30.0), "Center": (30.0, 70.0), "Right": (25.0, 100.0)}
    out_n = {}
    for k in range(int(secs / 0.5)):
        t = 0.5 * k
        dets = []
        for n, (x0, y0) in starts.items():
            x = x0 + 3.0 * (t % 20.0)                       # each drives +x at 3 cm/s
            L.own(n, t, x, y0, 0.0, False)
            sx = (W - x) if mirror_from <= t < mirror_to else x
            dets.append(TL.Det("lights", t, sx, y0, None, None, 0.5))
        fixes = L.feed(t, dets)
        out_n[t] = len(fixes)
    return L, said, out_n


def test_a_mirrored_camera_is_not_trusted_as_a_whole_and_trusted_again_when_it_agrees():
    """Audit finding 10: the camera watch voted only on sightings already named to a robot; mirrored sightings fell
    outside every gate, so a wrong camera mid-run was never concluded. Now: every robot that drives sees its sighting
    move the wrong way -> CAMERA NOT TRUSTED (said, logged), no fixes from it; trusted again once they agree."""
    L, said, out_n = _drive_scene(4.0, 12.0, secs=19.5)
    assert any("CAMERA NOT TRUSTED" in w for w in said), said
    t_dis = min(t for t, n in out_n.items() if t > 4.0 and n == 0)
    assert t_dis <= 10.0                                     # within the window after the mirror starts
    assert any("trusted again" in w for w in said)
    assert out_n[max(out_n)] > 0                              # fixes again at the end


def test_robots_whose_own_poses_alone_are_off_do_not_make_the_camera_untrusted():
    """The false alarm the watch must not raise: one or two robots' own poses far off (what the elimination is for) while
    the camera is right. Their motion still matches their sightings."""
    import tracking_layer as TL
    L = TL.TrackingLayer(["Left", "Center", "Right"], ("motion", "lights"), board=BOARD, camera_watch=False,
                         eliminate=False)
    L.proof_fn = lambda n, sg: 4.0
    for k in range(60):
        t = 0.5 * k
        dets = []
        for n, (x0, y0), off in (("Left", (20.0, 30.0), 30.0), ("Center", (30.0, 70.0), 25.0),
                                 ("Right", (25.0, 100.0), 0.0)):
            x = x0 + 2.0 * (t % 20.0)
            L.own(n, t, x + off, y0, 0.0, False)              # its own pose 25-30 cm off, moving the right way
            dets.append(TL.Det("lights", t, x, y0, None, None, 0.5))
        L.feed(t, dets)
    assert L.cam_distrust is None and not L.counts.get("whole_camera_distrust")


def test_with_the_overhead_unplugged_the_side_camera_is_not_taken_as_the_overhead():
    """Audit finding 14: one camera plugged in became the overhead whatever its name, even the side camera's."""
    import observe as OB
    m, w = OB.pick_cameras([("/dev/video2", "EMEET SmartCam C950")], {"overhead": "USB 2.0 Camera", "side": "EMEET"})
    assert "overhead" not in m and m.get("side") == "/dev/video2" and any("NOT used as the overhead" in x for x in w)
    m2, _ = OB.pick_cameras([("/dev/video0", "Some Camera"), ("/dev/video2", "EMEET SmartCam C950")],
                            {"overhead": "USB 2.0 Camera", "side": "EMEET"})
    assert m2["overhead"] == "/dev/video0" and m2["side"] == "/dev/video2"


# ------------------------------------------- 8 (audit H3 + Nick): identity: a sound out-and-back, inward-only, pause
def _ob_trial(seed, sd, hz, src, moves=True, dur=90.0, latency=0.4):
    """The audit's layer-alone trial (ob_real.py / ob_false.py): two unaccounted robots 30 cm apart, own poses far off;
    the wiggle shows nothing, then the out-and-back. moves=False: the command is lost (nobody moves)."""
    import random
    import tracking_layer as TL
    rng = random.Random(seed)
    st = dict(move=None, t=0.0)
    truth = {"A": [40.0, 60.0], "B": [70.0, 60.0]}
    hd = {"A": rng.uniform(0, 360), "B": rng.uniform(0, 360)}
    lay = []

    def wig(name, why):
        req = lay[0].wiggle_req
        if req["kind"] == "wiggle":
            return False
        if moves:
            st["move"] = dict(who=name, t0=st["t"] + latency, L=req["L"], v=req["v"])
        return True
    L = TL.TrackingLayer(["A", "B"], sources=(src,), wiggle=wig, board=(0, 0, 122, 122), brick_watch=False,
                         parked_watch=False, camera_watch=False)
    lay.append(L)
    own = {"A": (20.0, 20.0), "B": (100.0, 100.0)}
    L.set_unaccounted("A", "lost")
    L.set_unaccounted("B", "lost")
    t, res = 0.0, []
    while t < dur and not res:
        t += 1.0 / hz
        st["t"] = t
        pos = {}
        for n in truth:
            m = st["move"]
            off = 0.0
            if m and m["who"] == n and t >= m["t0"]:
                s_ = (t - m["t0"]) * m["v"]
                off = s_ if s_ <= m["L"] else max(0.0, 2 * m["L"] - s_)
            c_, s2 = math.cos(math.radians(hd[n])), math.sin(math.radians(hd[n]))
            pos[n] = (truth[n][0] + off * c_, truth[n][1] + off * s2)
            L.own(n, t, own[n][0] + off * c_, own[n][1] + off * s2, hd[n], off == 0.0)
        dets = [TL.Det(src, t, pos[n][0] + rng.gauss(0, sd), pos[n][1] + rng.gauss(0, sd), sigma=sd) for n in truth]
        for f in L.feed(t, dets):
            if f.get("how"):
                res.append(math.hypot(f["x"] - truth[f["robot"]][0], f["y"] - truth[f["robot"]][1]) < 10)
    return res


def test_the_out_and_back_names_right_at_detector_noise():
    """Audit H3: capped at 4 cm and judged on the 3 farthest sightings, at the detector's noise (1.6 cm, 2 pictures a
    second) the first name was WRONG in 33 of 200 (right 94). Now: as long as the noise needs, judged on medians at the
    time the robot's own track was out. (The full 200-trial table is in the build report.)"""
    res = [_ob_trial(s, 1.6, 2.0, "ml") for s in range(60)]
    right = sum(1 for r in res if r and r[0])
    wrong = sum(1 for r in res if r and not r[0])
    assert right >= 50 and wrong <= 4, (right, wrong)


def test_nobody_is_named_when_the_out_and_back_command_is_lost():
    """Audit H3: nobody moved (a lost command), noise 1.5 cm at 2 pictures a second: a robot was named in 81 of 200."""
    named = sum(1 for s in range(40) if _ob_trial(s, 1.5, 2.0, "lights", moves=False, dur=60.0))
    assert named == 0


def test_a_robot_half_over_an_edge_gets_an_inward_only_out_and_back():
    """Nick 12 Nov ("yes"): a robot near or half over an edge is not refused: it drives straight in and back (forward or
    reverse, whichever takes every candidate spot further in), never toward the edge."""
    import freeroam_core as FC
    sent = []
    c = FC.Controller(["A", "B"], BOARD, lambda n, s: sent.append((n, s)), lambda *a: None)
    c.t = 10.0
    r = c.R["A"]
    r.mode = "idle"
    r.own = (c.t, 3.0, 60.0, 180.0)                 # half over the x = 0 edge, its nose OUT (its back inward)
    r.own_hist = [(c.t, 3.0, 60.0, 180.0)]
    c.R["B"].mode = "gone"
    ok = c.outback(r, "test", dict(kind="outback", L=4.0, v=2.0, spots=[(3.0, 60.0)], sig=0.5, win=8.0, L_need=4.0))
    assert ok and r.move["rev"]                     # out by reversing (inward), back forward
    d = [s.split() for n, s in sent if " D " in s]
    assert float(d[0][5]) > 3.0                     # the first leg ends further in (x grows)


def test_an_unnamed_robot_pauses_while_it_is_identified_then_creeps_again():
    """MIRLT_ID_PAUSE (on by default; Nick to confirm): with two robots unaccounted an unnamed robot does not creep for
    the identification's derived pause, then never still resumes."""
    import freeroam_core as FC
    c = _two_unaccounted()
    R = c.R["Right"]
    R.id_pause_from = None
    R.still_since = c.t - 10.0
    called = []
    c.creep_step = lambda r, why: called.append(c.t) or False
    c.never_still_watch(R)
    assert not called and c.counts.get("id_pauses")
    c.t += c.id_pause_s(R) + 0.1
    R.still_since = c.t - 10.0
    c.never_still_watch(R)
    assert called


# ------------------------------------------- 9 (audit 9 + Nick): tag-free hands a heading on only past noise and doubt
def test_a_correct_own_heading_is_not_overridden_by_detector_noise():
    """Audit finding 9: own heading right, detector noise 15 deg at 2 pictures a second: a heading was handed on in 20 %
    of 15 s stands (the 2-sigma test repeated on every picture). Now the disagreement must pass the detector's measured
    95 % error (26 deg) and the robot's own doubt together."""
    import random
    import tagfree as TF
    handed = 0
    for s in range(200):
        rng = random.Random(s)
        f = TF.HeadingFuser()
        for k in range(30):
            f.update("A", 0.5 * (k + 1), 40.0, 40.0 + rng.gauss(0, 15.0))
            if f.advice("A") == "hand":
                handed += 1
                break
    assert handed <= 2


def test_the_hand_on_limit_comes_from_the_robots_room():
    """Nick: no fixed 45 deg. A 35 deg disagreement (past noise) is handed on in open ground, but near an edge (2 cm of
    room over the 20 cm horizon allows ~6 deg) the camera-measured heading check is asked instead."""
    import tagfree as TF
    f = TF.HeadingFuser()
    for k in range(20):
        f.update("A", 0.5 * (k + 1), 40.0, 5.0)                      # the detector says 35 deg less, steadily
    assert f.advice("A", own95=4.0, clear_cm=None) == "hand"
    assert f.advice("A", own95=4.0, clear_cm=2.0) == "check"
    assert f.advice("A", own95=40.0, clear_cm=None) is None           # its own doubt explains it: kept


@needs_ns
def test_mission_controls_stop_path_and_stop_first_work_together(tmp_path):
    """Mission Control's STOP (c5f6d2e): Ctrl+C to the run's process group, then SIGTERM while it is still ending. The
    run sends stop to every robot first, says and ignores the second signal, and still saves its corrections and state
    (Mission Control's own 25 s force-stop and its robot confirmations come after)."""
    rec = dry_end(tmp_path, "mcstop", 3)
    t_sig = rec["T"]["signal"]
    stops = [t for t, n, c in rec["sent"] if c == "stop" and t >= t_sig]
    assert len({n for t, n, c in rec["sent"] if c == "stop" and t >= t_sig}) == 3
    assert min(stops) - t_sig < 1.0 and "end_save" in rec["T"] and "close" in rec["T"]
    assert "already ending" in rec["stdout"]
    assert json.load(open(tmp_path / "run-mcstop" / "state.json"))["ended"].startswith("stopped by hand")


# ------------------------------------------- 10 (audit 7 + Nick): one wheel_scale owner; curves live from the fleet
def test_a_correction_inside_its_own_error_bar_is_never_saved(tmp_path):
    """Audit finding 7: "wheel_scale 1.0000 -> 1.0068 (+-0.013)": three corrections inside their own error bar, all
    saved. Now the pooled value is saved only when it differs from the value in use past its 95 % error bar."""
    import start_discovery as D
    pooled = {"wheel": dict(value=1.0068, se=0.0066, runs=3, sure=True)}
    assert D.corr_commands(pooled, {"wheel_scale": 1.0}, owner="pool") == []
    big = {"wheel": dict(value=1.0300, se=0.0040, runs=3, sure=True)}
    assert D.corr_commands(big, {"wheel_scale": 1.0}, owner="pool") == ["corr save wheel_scale 1.0300"]
    assert D.corr_commands(big, {}, owner="pool") == []                    # its value in use not heard: not saved


def test_the_pool_is_absolute_so_a_relative_fit_is_saved_right(tmp_path):
    """Audit finding 7: discovery's corr_now was never set, so a fit relative to 0.98 in use was saved as if relative to
    1. Now each row keeps the value in use; the pool and the save are absolute."""
    import start_discovery as D
    for k in range(3):
        pooled = D.pool_fit(str(tmp_path), "Left", {"wheel": 1.02, "wheel_se": 0.002}, in_use={"wheel_scale": 0.98})
    assert abs(pooled["wheel"]["value"] - 0.9996) < 1e-3
    assert D.corr_commands(pooled, {"wheel_scale": 0.98}, owner="pool") == ["corr save wheel_scale 0.9996"]


def test_the_end_of_run_measurement_goes_to_the_pool_not_to_the_robot():
    import freeroam_core as FC
    sent = []
    c = FC.Controller(["A", "B"], BOARD, lambda n, s: sent.append((n, s)), lambda *a: None)
    for n in ("A", "B"):
        c.R[n].mode, c.R[n].curve = "idle", False
    c.params = {"A": {"wheel_scale": 0.98}}
    for k in range(30):
        c.ratio_fit(c.R["A"], "drive", 1.02 + 0.001 * (k % 5))
    done = c.save_corrections("the time cap")
    assert not any(s.startswith("set wheel_scale") or s == "save" for n, s in sent)
    assert [d["pool"] for d in done if d["robot"] == "A"] == [True]


def _curve_ctrl(resp):
    import freeroam_core as FC
    sent = []
    c = FC.Controller(list(resp), BOARD, lambda n, s: sent.append((n, s)), lambda *a: None)
    for n in resp:
        c.R[n].mode, c.R[n].curve, c.R[n].arc_scales = "idle", True, (1.0, 1.0)
    import random
    rng = random.Random(3)
    for k in range(60):
        n = list(resp)[k % len(resp)]
        use = c.R[n].__dict__.get("arc_live") or (1.0, 1.0)        # the robot drives on the scale set in use
        c.curve_live(c.R[n], (resp[n] + rng.gauss(0, 0.01)) * use[0], (1.0 + rng.gauss(0, 0.01)) * use[1])
    return c, sent


def test_curves_use_the_fleets_live_estimate_and_a_robot_its_own_only_when_it_differs():
    """Nick 12 Nov: a live fleet-wide per-session curve estimate for all robots; a robot its own only when its data
    differ significantly (Mann-Whitney, fine print in the report)."""
    c, sent = _curve_ctrl({"Left": 1.06, "Center": 1.06, "Right": 1.06})
    ks = {n: float(s.split()[2]) for n, s in sent if s.startswith("set arc_k_scale")}
    assert set(ks) == {"Left", "Center", "Right"} and all(abs(v - 1 / 1.06) < 0.01 for v in ks.values())
    assert not any(s == "save" for n, s in sent)                            # live, never saved
    c, sent = _curve_ctrl({"Left": 1.15, "Center": 1.03, "Right": 1.03})
    last = {}
    for n, s in sent:
        if s.startswith("set arc_k_scale"):
            last[n] = float(s.split()[2])
    assert abs(last["Left"] - 1 / 1.15) < 0.01 and abs(last["Center"] - 1 / 1.03) < 0.015


# ------------------------------------------- 12: the side camera's floor height: its effect measured, not assumed
def test_the_side_cameras_offset_from_the_overhead_is_measured_and_removed():
    """Item 12: the assumed 2 cm floor height cannot be measured from the uploaded logs (the 3-D side model stays on the
    Pi). Its effect is measured live: side-and-overhead pairs of one robot give the side camera's offset, removed from
    side-only detections once 20 pairs exist."""
    import random
    import tagfree as TF
    import tracking_layer as TL
    src = TF.DetectorSource.__new__(TF.DetectorSource)
    assert src.side_offset() is None
    rng = random.Random(2)
    for k in range(25):
        x, y = rng.uniform(10, 110), rng.uniform(10, 110)
        over = [TL.Det("ml", 0.0, x, y)]
        side = [TL.Det("ml", 0.0, x + 2.5 + rng.gauss(0, 1.0), y - 1.0 + rng.gauss(0, 1.0))]
        src._side_learn(over, side)
    off = src.side_offset()
    assert off is not None and abs(off[0] - 2.5) < 0.8 and abs(off[1] + 1.0) < 0.8


def test_a_stuck_robot_never_reverses_toward_an_edge_on_its_unconfirmed_track():
    """Found by e19's own e17-comparison re-run (seed 1, fixes 1 s late): Left's moves did not move (hung), a re-seed
    shifted its track, then a 12 cm back-out along that track took it over the edge. A reverse toward an edge is now
    refused while its pose is unconfirmed or it is stuck; the run then holds or tries forward / turning moves."""
    import freeroam_fastsim as F
    s = F.Sim(1, 0.5, dead_robot=False, pi_freeze=True, edge_physics=True, camera="detections", sensors=True,
              cam_delay=1.0, hdg_fault={"deg": 25, "near_cm": 14, "every_s": 60})
    s.run()
    assert s.stats.get("falls", 0) == 0


# ------------------------------------------- 13 (audit 8, 12, the text-only tests)
def test_a_typed_safety_value_is_one_of_its_choices_or_refused(monkeypatch, tmp_path):
    """Audit finding 8: MIRLT_CURVE_SCALE_SAVE="Auto" read as ON (a default-off switch turned on); "Fixed" reported as
    set but not "fixed". Now "auto" in any case is the default, a choice matches in any case, anything else is refused
    (the default used, said at the start) and never written to safety.json."""
    import importlib
    import safety_group as SG
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    SG._FILE = None
    SG._REFUSED.clear()
    monkeypatch.setenv("MIRLT_CURVE_SCALE_SAVE", "Auto")
    assert SG.flag("MIRLT_CURVE_SCALE_SAVE") is False
    monkeypatch.setenv("MIRLT_CURVE_SCALE_SAVE", "auto ")
    assert SG.flag("MIRLT_CURVE_SCALE_SAVE") is False
    monkeypatch.setenv("MIRLT_EDGE_RULE", "Fixed")
    assert SG.value("MIRLT_EDGE_RULE") == "fixed"
    monkeypatch.setenv("MIRLT_EDGE_RULE", "sideways")
    assert SG.value("MIRLT_EDGE_RULE") == "derived" and "MIRLT_EDGE_RULE" in SG.refused()
    assert "REFUSED" in SG.start_words()
    assert SG.env_from_settings({"MIRLT_EDGE_PARK": "yes please", "MIRLT_NEVER_STILL": "0"}) == {"MIRLT_NEVER_STILL": "0"}


def test_missions_and_calibration_read_the_safety_box(monkeypatch, tmp_path):
    """Audit finding 8: six switches were read straight from the environment in missions and calibration, so a value
    saved in the Safety box (safety.json) was ignored there."""
    import importlib
    import safety_group as SG
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    monkeypatch.delenv("MIRLT_FRAME_WATCH", raising=False)
    SG._FILE = None
    SG.write_file({"MIRLT_FRAME_WATCH": "0", "MIRLT_CALIB_GIVEUP": "1"}, str(tmp_path / "safety.json"))
    SG._FILE = None
    import frame_watch as FW
    assert FW.enabled() is False
    assert SG.flag("MIRLT_CALIB_GIVEUP") is True
    SG._FILE = None


def test_the_trust_refit_runs_off_the_control_loop(monkeypatch):
    """Audit finding 12: every 5 new samples the quantile fits ran inside the 50 ms loop (52 ms median, 604 ms worst
    here). With ASYNC (set by the runners) the loop gets the last fit at once and a worker swaps the new one in."""
    import random
    import time as _t
    import odo_trust as OT
    rng = random.Random(1)
    fl = {n: OT.Trust(n) for n in ("A", "B", "C")}
    OT.link(fl)
    for k in range(120):
        for t in fl.values():
            t.add_sample(rng.uniform(10, 40), abs(rng.gauss(0, 2.0)) + 0.1 * rng.uniform(10, 40), t=k)
    first = fl["A"].fit()
    for k in range(40):
        fl["A"].add_sample(40.0, 30.0, t=200 + k)                 # A gets much worse: the fit must follow
    monkeypatch.setattr(OT, "ASYNC", True)
    t0 = _t.perf_counter()
    stale = fl["A"].fit()
    ms = 1000 * (_t.perf_counter() - t0)
    assert stale == first and ms < 20.0
    fl["A"]._fit_thread.join(10)
    assert fl["A"].fit() != first
