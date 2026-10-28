"""Engine checks (one system, build e02-oct28-one-system.md): restarts, battery, settings read back, robots that go
silent, the 8 cm safety net, run-ins kept through re-plans, scaled heading checks, misses in the line check."""
import json
import math
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(FLEET))
sys.path.insert(0, FLEET)

import engine_checks as EC  # noqa: E402
import fleetmap as FM       # noqa: E402
import line_check as LC     # noqa: E402
import planner as P         # noqa: E402
import workspace as WS      # noqa: E402


def test_parse_status_old_and_new_firmware():
    a = EC.parse_status("status Left (firmware 2026-10-27a): idle, selected turn, battery 43%, WiFi -51 dBm")
    assert a == {"name": "Left", "firmware": "2026-10-27a", "boot": None, "state": "idle", "battery": 43}
    b = EC.parse_status("status Center (firmware 2026-10-28a, boot 4305): running, selected turn, battery 23%, "
                        "WiFi -60 dBm")
    assert b["boot"] == "4305" and b["battery"] == 23 and b["state"] == "running"
    assert EC.boot_of_run("C12D-3") == "C12D" and EC.boot_of_run("nonsense") is None


def test_day_state_restart_and_warmup(tmp_path):
    d = EC.DayState(str(tmp_path / "d.json"))
    assert d.restarted("Left", None) is None
    assert d.restarted("Left", "AAAA") is True           # never calibrated today by the engine
    d.note_calibrated("Left", "AAAA")
    d.note_warmup("Left", wheel_ratio=1.02, gyro_ratio=1.09)
    d.save()
    d2 = EC.DayState(str(tmp_path / "d.json"))
    assert d2.restarted("Left", "AAAA") is False
    assert d2.restarted("Left", "BBBB") is True           # a sleep check-in / power cycle: a new boot ID
    notes = []
    c = EC.corrections_for("Left", d2, notes)
    base = EC.base_corrections("Left")
    assert c["wheel_scale"] == pytest.approx(base["wheel_scale"] * 1.02, abs=1e-4)
    assert c["gyro_scale"] == pytest.approx(base["gyro_scale"])     # 9 % is over WARMUP_SCALE_MAX: not applied
    assert any("NOT applied" in x for x in notes)


def _two_robot_map(a, b, sep=8.0):
    m = WS.blank_map((0, 0, 122, 61), [("Left", a), ("Right", b)])
    m["tasks"] = {}
    m["mission"] = {"planner": {"separation_cm": sep}}
    return FM.normalize(m)


def test_plan_gap_refuses_under_the_net_but_not_robots_already_close():
    # two robots crossing: planned with separation 5, the net (8) refuses it if they come under 8
    m = _two_robot_map([20, 18, 0], [100, 18, 180], sep=5.0)
    m["tasks"] = {"Left": {"visit": [{"goto": [100, 43]}], "park": False},
                  "Right": {"visit": [{"goto": [20, 43]}], "park": False}}
    plan = P.make_plan(m, over={"separation_cm": 5.0})
    gap = EC.plan_gap(plan, m, gate_cm=8.0)
    if gap["closest_cm"] < 8.0:
        assert EC.gate_words(gap, 8.0)
    plan8 = P.make_plan(m, over={"separation_cm": 8.0})
    assert EC.gate_words(EC.plan_gap(plan8, m, gate_cm=8.0), 8.0) is None
    # two robots stopped 7 cm apart (a front-sensor stop) that are planned apart: not refused
    m2 = _two_robot_map([40, 30, 180], [56.5, 30, 0], sep=5.0)
    m2["tasks"] = {"Left": {"visit": [{"goto": [20, 30]}], "park": False},
                   "Right": {"visit": [{"goto": [100, 30]}], "park": False}}
    p2 = P.make_plan(m2)
    g2 = EC.plan_gap(p2, m2, gate_cm=8.0)
    assert g2["start_cm"] < 8.0 and EC.gate_words(g2, 8.0) is None


def test_allowed_heading_scales_with_distance_and_risk():
    short = EC.allowed_heading_deg(5.0, EC.OPEN_DRIFT_CM, 15.0)
    longer = EC.allowed_heading_deg(40.0, EC.OPEN_DRIFT_CM, 15.0)     # capped at the next fix (10 cm)
    risky = EC.allowed_heading_deg(40.0, EC.RISKY_DRIFT_CM, 15.0)
    assert short > longer > risky
    assert longer == pytest.approx(math.degrees(math.atan2(3, 10)) - math.degrees(math.atan2(1, 15)))
    # a heading read off an 8 cm line (14:30 L4) cannot judge a risky leg: confirm first
    assert EC.allowed_heading_deg(40.0, EC.RISKY_DRIFT_CM, 8.0) < 2.0


def test_run_in_is_kept_by_a_replan():
    """14:30 L4: a re-plan dropped the 15 cm lead-in. A goto with run_in_cm ends with a straight confirm drive along its
    heading from wherever the robot is, and a re-plan from mid run-in drives the rest of the line."""
    m = _two_robot_map([30, 30, 0], [100, 45, 180])
    m["tasks"] = {"Left": {"visit": [{"goto": [70, 30, 0.0], "run_in_cm": 15.0}], "park": False},
                  "Right": {"visit": [], "park": False}}
    m = FM.normalize(m)
    plan = P.make_plan(m)
    st = plan["robots"]["Left"]["steps"]
    conf = [x for x in st if x["phase"] == "confirm"]
    assert conf and conf[-1]["to"][:2] == pytest.approx([70, 30], abs=0.01)
    L = sum(math.dist(x["from"][:2], x["to"][:2]) for x in conf if x["type"] == "drive")
    assert L == pytest.approx(15.0, abs=0.05)
    assert all(x["run_in"] for x in conf) and conf[0]["speed"] == P.DEFAULT_SETTINGS["confirm_cms"]
    # from 10 cm off the line, facing elsewhere (a sensor stop and a re-plan): the run-in comes back whole
    job = P.Job("Left", [62.0, 40.0, math.radians(90)], m["tasks"]["Left"]["visit"], False, [62, 40, 0])
    pl = P.Planner(m)
    pr = P.build_prims(job, [0], pl.grid, pl.world, pl.docks, pl.s, pl.body, [])
    ri = [p for p in pr if p.get("run_in") and p["type"] == "drive"]
    assert sum(math.dist(p["from"][:2], p["to"][:2]) for p in ri) == pytest.approx(15.0, abs=0.05)
    # already on the run-in line, 6 cm before the end: it drives the 6 cm, no detour
    job2 = P.Job("Left", [64.0, 30.2, 0.0], m["tasks"]["Left"]["visit"], False, [64, 30, 0])
    pr2 = P.build_prims(job2, [0], pl.grid, pl.world, pl.docks, pl.s, pl.body, [])
    assert [p["type"] for p in pr2] == ["drive"] and pr2[0]["phase"] == "confirm"


def test_line_check_misses_singles_and_runs():
    d = [1, 4, 1, 4, 4, 4, 1, 1, 5]
    ts = [0.1 * k for k in range(len(d))]
    ms = LC.misses(d, ts, 3.0)
    assert ms == {"misses": 5, "single": 2, "runs": 1, "in_runs": 3, "longest": 3, "longest_s": 0.2}
    # two misses with a long gap between them (no pictures) are two one-offs
    assert LC.misses([4, 4], [0.0, 1.0], 3.0)["single"] == 2


def test_setting_readback_compare():
    assert EC.parse_setting("set wheel_scale 0.979") == ("wheel_scale", 0.979)
    assert EC.parse_setting("set calib_dir_limits 10 20") is None
    assert EC.setting_matches(0.979, 0.9790)
    assert not EC.setting_matches(0.979, 1.0)          # Center at 14:12: sent 0.979, PARAM said 1.0000
    assert not EC.setting_matches(3, None)


def _dry(tmp_path, extra, cond=None, name="r"):
    """run_plan --dry-run on the demo map in its own network namespace (sweeps/rp_shim.py)."""
    sess = str(tmp_path / name)
    cmd = ["unshare", "-n", sys.executable, os.path.join(FLEET, "sweeps", "rp_shim.py"), "--map",
           os.path.join(FLEET, "examples", "demo_map.json"), "--dry-run", "--yes", "--session", sess] + extra
    if cond:
        cp = str(tmp_path / (name + "_cond.json"))
        with open(cp, "w") as f:
            json.dump(cond, f)
        cmd.append("--sim-arg=--sweep-cond=%s" % cp)
    out = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=600)
    with open(os.path.join(sess, "run_plan_summary.json")) as f:
        return json.load(f), out.stdout


@pytest.mark.skipif(not os.environ.get("FLEET_DRYRUN"), reason="about 2 min: set FLEET_DRYRUN=1 to run")
def test_dry_dead_robot_others_carry_on(tmp_path):
    """Monday sweep: a robot that died left the others waiting for ever. Now it is a fixed obstacle and they finish."""
    s, out = _dry(tmp_path, [], {"robot_out": {"name": "Right", "t": 20, "mode": "dead"}})
    assert s["result"] == "done", out[-2000:]
    assert [r["robot"] for r in s["robots_out"]] == ["Right"]
    if "simulator" in s:          # (the simulator waits for the dead robot's run to end: often no report)
        assert s["simulator"]["collision"] is False


@pytest.mark.skipif(not os.environ.get("FLEET_DRYRUN"), reason="about 3 min: set FLEET_DRYRUN=1 to run")
def test_dry_uncalibrated_robots_are_calibrated_and_settings_read_back(tmp_path):
    s, out = _dry(tmp_path, ["--sim-arg=--uncalibrated", "--send", "set safe_edge 3", "--send", "set edge_thr 150"])
    assert s["result"] == "done", out[-2000:]
    assert "Calibrating" in out and "settings read back: Left 2 ok" in out
    day = json.load(open(str(tmp_path / "r" / "day_state.json")))
    assert all(day["robots"][n]["calibrated_boot"] == day["robots"][n]["boot"] for n in ("Left", "Center", "Right"))


@pytest.mark.skipif(not os.environ.get("FLEET_DRYRUN"), reason="about 30 s: set FLEET_DRYRUN=1 to run")
def test_dry_battery_floor_refuses(tmp_path):
    s, out = _dry(tmp_path, ["--battery-floor", "99"])
    assert s["result"] != "done" and "Battery under the 99 % floor" in out
