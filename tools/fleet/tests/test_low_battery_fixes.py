"""The 2 Nov low-battery fixes (docs/build-reports/e02-nov02-low-battery-fixes.md):

  1. one battery floor end to end: `low-battery` lets a robot drive down to 5 % (the loop's check AND every run_plan),
     the default stays 25 %, and the floor in force is printed at the start and in every refusal;
  2. the holding loop: with every robot silent the loop backs off (10, 20, 40, then 60 s), logs each robot's change
     once with one clip, a summary every 5 min, and an alert after 10 min with nobody answering (screen, doe-status,
     events.csv); a robot that answers ends the wait and clears the alert;
  3. (Pi side of) the firmware's power telemetry: the status tail and the "motor board not answering" line.

    python3 -m pytest tools/fleet/tests/test_low_battery_fixes.py -q        # about 2 min (two simulator runs)
"""
import csv
import json
import os
import subprocess
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (HERE, FLEET, os.path.join(TOOLS, "sim"), TOOLS, os.path.join(TOOLS, "pi")):
    if p not in sys.path:
        sys.path.append(p)

import doe_loop as D          # noqa: E402
import engine_checks as EC    # noqa: E402
import ladder as LD           # noqa: E402
from test_doe_loop import FakeBench, _good, _loop   # noqa: E402

LOW = {"MIRLT_MIN_BATTERY": "5"}


# ---------------------------------------------------------------------------------------------- 1. the floor
def test_one_floor_default_25_low_battery_5_never_under_5():
    assert EC.battery_floor({}) == 25 and EC.battery_floor({"MIRLT_MIN_BATTERY": ""}) == 25
    assert EC.battery_floor(LOW) == 5
    assert EC.battery_floor({"MIRLT_MIN_BATTERY": "2"}) == 5          # clamped at 5 (was 10 in dfb3396)
    assert EC.battery_floor({"MIRLT_MIN_BATTERY": "10"}) == 10
    assert EC.battery_floor({"MIRLT_MIN_BATTERY": "ten"}) == 25       # nonsense: the default, never "off"
    assert EC.battery_floor_words(env={}) == "25 % (the default)"
    assert EC.battery_floor_words(env=LOW) == "5 % (low-battery run: MIRLT_MIN_BATTERY=5)"
    assert "never under 5" in EC.battery_floor_words(env={"MIRLT_MIN_BATTERY": "2"})


def test_low_battery_word_sets_5_in_the_launcher():
    sh = open(os.path.join(REPO, "run-fleet-pi.sh")).read()
    assert "low-battery) MINB=5 ;;" in sh and "export MIRLT_MIN_BATTERY=$MINB" in sh
    assert "CHARGE THE ROBOTS STRAIGHT AFTER A LOW-BATTERY RUN" in sh


def test_every_run_plan_inherits_the_floor_from_the_environment():
    """mission_run, warm-up, straightening and calibration all run inside run_plan: its --battery-floor default is
    the same env var, so nothing on the drive path keeps a 25 % floor of its own."""
    for env, want in ((LOW, "default 5"), ({}, "default 25")):
        e = dict(os.environ)
        e.pop("MIRLT_MIN_BATTERY", None)
        e.update(env)
        out = subprocess.run([sys.executable, os.path.join(FLEET, "run_plan.py"), "--help"], capture_output=True,
                             text=True, env=e, timeout=60).stdout
        assert want in " ".join(out.split()), out[-800:]


def test_the_loop_passes_its_floor_to_every_drive(tmp_path, monkeypatch):
    """dfb3396 lowered the loop's line only; run_plan kept 25 and refused all 8 drives under 25 % on 2 Nov."""
    calls = []
    monkeypatch.setattr(LD, "MIN_BATTERY", 5)
    monkeypatch.setattr(LD.subprocess, "Popen", lambda *a, **k: types.SimpleNamespace(
        send_signal=lambda s: None, wait=lambda timeout=None: 0, kill=lambda: None))
    monkeypatch.setattr(LD.subprocess, "call", lambda cmd, **k: calls.append(cmd) or 0)
    monkeypatch.setattr(LD.time, "sleep", lambda s: None)
    L = LD.Ladder.__new__(LD.Ladder)
    L.drop_bench = L.drop_eye = lambda: None
    L.calib, L.stream, L.side_calib, L.ips = "c.json", "127.0.0.1:8000", "", ["192.168.1.250"]
    L.a = types.SimpleNamespace(no_lights=False, dry=False)
    L.run_planned("drive01", {"notes": "x", "mission": {"planner": {"cruise_cms": 10}}}, {"Left": (1, 2, 3)},
                  str(tmp_path))
    cmd = calls[0]
    assert cmd[cmd.index("--battery-floor") + 1] == "5"


def test_a_robot_at_15pct_passes_the_loops_check_on_a_low_battery_run(tmp_path, monkeypatch):
    monkeypatch.setattr(LD, "corrections_for", lambda n: dict(gyro_scale=1.075, wheel_scale=0.979, gyro_drift_dps=0.0))
    monkeypatch.setattr(D.time, "sleep", lambda s: None)
    monkeypatch.setattr(LD, "MIN_BATTERY", EC.battery_floor(LOW))
    L = _loop(tmp_path, {"Left": _good(battery=15)})
    assert L.check(str(tmp_path), ["Left"], "loop 3") == ["Left"]
    # the default floor (25) sets it aside, and says which floor
    monkeypatch.setattr(LD, "MIN_BATTERY", EC.battery_floor({}))
    assert L.check(str(tmp_path), ["Left"], "loop 4") == []
    assert "battery 15% (under the 25 % (the default) floor)" in L.st["kept_out"]["Left"]
    row = list(csv.DictReader(open(os.path.join(str(tmp_path), "battery.csv"))))[0]
    assert row["floor_pct"] == "5"


def test_a_drive_refused_on_the_battery_is_a_hold_with_no_p95(tmp_path, monkeypatch):
    """Between the loop's check and run_plan's own read the % can fall under the floor: logged as a battery park
    (tried again next loop), not "start-up failed" with the p95 of a robot that never moved."""
    sys.path.insert(0, HERE)
    import test_night_oct30 as N
    monkeypatch.setattr(D, "miss_pattern", lambda *a, **k: dict(n=40, over=0))
    monkeypatch.setattr(D, "sensor_runs", lambda *a, **k: (dict(readings=0, low_readings=0, low_runs=0, single=0,
                                                                 stops=0, stops_avoided_by_2=0, stops_avoided_by_3=0,
                                                                 stop_detail=[]), []))
    monkeypatch.setattr(LD, "MIN_BATTERY", 25)
    summ = {"result": "start-up failed", "steps_sent": {"Right": 0},
            "start_checks": {"Right": {"battery": 24, "heading_deg": 0.0, "heading_allowed_deg": 10.0}}}
    L = N._drive_stub(tmp_path, summ)
    rows = []
    L.row = lambda name, d: rows.append((name, d))
    L.run_planned = lambda tag, m, at, sub: dict(code=1, summary=summ, check={"robots": {"Right": {"camera": {
        "p95_cm": 0.38, "median_cm": 0.2, "max_cm": 0.5}}}}, sess=str(tmp_path / "drive13-Right-5cms-40cm"))
    assert L.drive(str(tmp_path), "Right", 12, 29) == "held"
    d = [q for n, q in rows if n == "drives"][0]
    assert d["p95_cm"] is None and d["result"] == "start-up failed"
    assert "under the 25 % (the default) floor" in L.st["kept_out"]["Right"]


def _dry(tmp_path, env, name):
    sess = str(tmp_path / name)
    cfg = str(tmp_path / "battery15.json")
    with open(cfg, "w") as f:
        json.dump({"robot": {"battery_start_pct": [15.0, 15.0]}}, f)       # every simulated robot starts at 15 %
    e = dict(os.environ)
    e.pop("MIRLT_MIN_BATTERY", None)
    e.update(env)
    cmd = ["unshare", "-n", sys.executable, os.path.join(FLEET, "sweeps", "rp_shim.py"), "--map",
           os.path.join(FLEET, "examples", "demo_map.json"), "--dry-run", "--yes", "--session", sess,
           "--sim-arg=--config=%s" % cfg]
    out = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=600, env=e)
    with open(os.path.join(sess, "run_plan_summary.json")) as f:
        return json.load(f), out.stdout


def test_dry_robots_at_15pct_drive_on_a_low_battery_run_and_are_refused_by_default(tmp_path):
    """The simulator, three robots at 15 %: with MIRLT_MIN_BATTERY=5 (what `low-battery` sets) the plan runs to the
    end; without it the default 25 % floor refuses the start and says so (nothing moves)."""
    s, out = _dry(tmp_path, LOW, "low")
    assert s["result"] == "done", out[-2000:]
    assert "[battery floor: 5 % (low-battery run: MIRLT_MIN_BATTERY=5)]" in out
    assert s["steps_sent"] and all(int(v) > 0 for v in s["steps_sent"].values())
    s, out = _dry(tmp_path, {}, "default")
    assert s["result"] == "start-up failed"
    assert "Battery under the 25 % floor" in out and "[floor: 25 % (the default)]" in out


# ---------------------------------------------------------------------------------------------- 2. the holding loop
class Clock:
    def __init__(self):
        self.t = 1_000_000.0

    def time(self):
        return self.t

    def sleep(self, s):
        self.t += max(0.0, s)


class SilentBench(FakeBench):
    """FakeBench whose waits take time on the fake clock (a silent robot makes every answer wait run out)."""

    def __init__(self, loop, robots, clock):
        super().__init__(loop, robots)
        self.clock = clock

    def wait(self, cond, s):
        if cond():
            return True
        self.clock.sleep(s)
        return cond()


def _silent_loop(tmp_path, monkeypatch, hours):
    clock = Clock()
    monkeypatch.setattr(D.time, "time", clock.time)
    monkeypatch.setattr(D.time, "sleep", clock.sleep)
    monkeypatch.setattr(LD, "corrections_for", lambda n: dict(gyro_scale=1.075, wheel_scale=0.979, gyro_drift_dps=0.0))
    robots = {n: dict(_good(boot=b), silent=True) for n, b in (("Left", "A1B2"), ("Right", "C0DE"), ("Center", "4305"))}
    L = _loop(tmp_path, robots)
    L.bench = SilentBench(L, robots, clock)
    L.a = types.SimpleNamespace(dry=False, hours=hours, keep_going=True)
    L.t_start = clock.t
    L.st["passed"]["L0"] = 1
    clips = []
    L.frames = types.SimpleNamespace(event=lambda kind, text, **k: clips.append((k.get("robot"), text)),
                                     active=lambda *a, **k: None)
    L.start_logs = lambda: None
    L.guard = types.SimpleNamespace(state=lambda: {"stop": False})
    L.active = lambda step=None: [n for n in robots if n not in L.st["kept_out"]]
    L.sub = lambda name: str(tmp_path / name)
    L.space_check = lambda k: False
    return L, robots, clock, clips


def test_three_silent_robots_for_15_min_back_off_log_once_and_alert(tmp_path, monkeypatch, capsys):
    """2 Nov 15:34-15:56: one loop every 9.4 s, 408 "held" lines with a clip each, nobody told. Simulated here: the
    three robots silent for the loop's whole 15 min cap."""
    L, robots, clock, clips = _silent_loop(tmp_path, monkeypatch, hours=0.25)
    waits = []
    real_idle = L.idle_wait

    def idle(names):
        t0 = clock.t
        out = real_idle(names)
        waits.append(round(clock.t - t0))
        return out
    L.idle_wait = idle
    L.run()
    screen = capsys.readouterr().out
    # backs off: 10, 20, 40, then every 60 s (before: every 9.4 s, about 96 loops in 15 min)
    assert waits[:5] == [10, 20, 40, 60, 60] and set(waits[3:]) == {60}
    assert L.st["doe"]["loops"] <= 16 and L.st["doe"]["held_loops"] == L.st["doe"]["loops"]
    # each robot's change logged once, with ONE clip each; then a summary every 5 min
    ev = list(csv.DictReader(open(os.path.join(str(tmp_path), "events.csv"))))
    held = [r for r in ev if r["kind"] == "held"]
    assert sorted(r["robot"] for r in held) == ["Center", "Left", "Right"]
    assert sorted(r for r, _ in clips) == ["Center", "Left", "Right"]
    assert 2 <= sum(1 for r in ev if r["kind"] == "still not answering") <= 3
    # the alert after 10 min with nobody answering: once, on the screen, in events.csv and in state.json (doe-status)
    alerts = [r for r in ev if r["kind"] == "ALERT"]
    assert len(alerts) == 1 and "NO ROBOT HAS ANSWERED FOR" in alerts[0]["text"]
    assert 600 <= float(alerts[0]["t"]) - L.st["doe"]["silent_t"]["Left"] < 603     # at 10 min, not after the wait
    assert "ALERT: NO ROBOT HAS ANSWERED" in screen and "Battery floor: 25 % (the default)" in screen
    assert "next check in 60 s" in screen and screen.count("next check in") == 4     # said while the wait grows
    assert screen.count("DOE loop 1 ") == 0                                         # no banner per held loop
    capsys.readouterr()
    assert D.status(str(tmp_path)) == 0
    st = capsys.readouterr().out
    assert st.startswith("!" * 100) and "ALERT" in st.splitlines()[1] and "not answering: Center since" in st
    # while waiting, every silent robot got `wake` + `status` every 2 s (a robot waking for its check-in is caught)
    assert sum(1 for n, c in L.bench.sent if n == "Left" and c == "wake") > 100


def test_a_robot_that_answers_ends_the_wait_and_clears_the_alert(tmp_path, monkeypatch):
    L, robots, clock, clips = _silent_loop(tmp_path, monkeypatch, hours=1)
    names = list(robots)
    for _ in range(13):                     # about 11 min of checks and waits
        assert L.check(str(tmp_path), names, "loop") == []
        L.idle_wait(names)
    assert L.st["doe"]["alert"] and L.backoff_i >= 4
    robots["Left"]["silent"] = False        # Left is plugged in and answers during the next wait
    t0 = clock.t
    assert L.idle_wait(names) == ["Left"] and clock.t - t0 < 3.0 and L.backoff_i == 0
    assert L.check(str(tmp_path), names, "loop") == ["Left"]
    ev = open(os.path.join(str(tmp_path), "events.csv")).read()
    assert "answering again" in ev and "alert cleared" in ev and L.st["doe"]["alert"] is None
    assert "Left" not in L.st["doe"]["silent"]


def test_a_robot_silent_again_later_gets_a_new_line_and_clip(tmp_path, monkeypatch):
    L, robots, clock, clips = _silent_loop(tmp_path, monkeypatch, hours=1)
    robots["Right"]["silent"] = robots["Center"]["silent"] = False
    for silent in (True, True, True, False, True):
        robots["Left"]["silent"] = silent
        L.check(str(tmp_path), ["Left"], "loop")
    ev = [r for r in csv.DictReader(open(os.path.join(str(tmp_path), "events.csv"))) if r["robot"] == "Left"]
    assert [r["kind"] for r in ev] == ["held", "answering again", "held"]
    assert len([c for c in clips if c[0] == "Left" and c[1].startswith("held")]) == 2


def test_battery_parks_repeat_without_new_lines(tmp_path, monkeypatch):
    """A robot under the floor is retried every loop (Nick 30 Oct: nobody is out for good); its park and retry are
    said once while the reason stays the same."""
    monkeypatch.setattr(LD, "corrections_for", lambda n: dict(gyro_scale=1.075, wheel_scale=0.979, gyro_drift_dps=0.0))
    monkeypatch.setattr(D.time, "sleep", lambda s: None)
    monkeypatch.setattr(LD, "MIN_BATTERY", 5)
    robots = {"Left": _good(battery=4)}
    L = _loop(tmp_path, robots)
    for k in range(4):
        L.retry_outs(k)
        L.check(str(tmp_path), ["Left"], "loop %d" % k)
        robots["Left"]["battery"] -= 1 if k == 1 else 0
    ev = [r["kind"] for r in csv.DictReader(open(os.path.join(str(tmp_path), "events.csv")))]
    assert ev.count("parked") == 1 and ev.count("retried") == 1


# ---------------------------------------------------------------------------------------------- 3. power telemetry
def test_status_tail_and_motor_board_line(tmp_path, monkeypatch):
    assert D.parse_power("M,status Left (firmware 2026-10-29b, boot 4305): idle, selected turn, battery 21%, WiFi -62 "
                         "dBm, charging 0, motor board on, sensors 12 ms old") == dict(charging=0, motor_board="on",
                                                                                         sensor_age_ms=12)
    assert D.parse_power("M,status Left (firmware 2026-10-29a, boot 4305): idle, selected turn, battery 21%, "
                         "WiFi -62 dBm") is None
    text = ("M,status Left (firmware 2026-10-29b, boot 4305): idle, selected turn, battery 21%, WiFi -62 dBm, "
            "charging 1, motor board off, sensors 5012 ms old")
    p = EC.parse_status(text)
    assert p["battery"] == 21 and p["firmware"] == "2026-10-29b" and p["boot"] == "4305"
    L = _loop(tmp_path, {"Left": _good()})
    L._listen("Left", "M", "", text)
    L._listen("Left", "M", "", "Left: motor board not answering (no new sensor reading for 2100 ms; motor board "
                               "power on; battery 21% and charging 0 are its last readings)")
    assert L.got["Left"]["power"]["motor_board"] == "off"
    ev = list(csv.DictReader(open(os.path.join(str(tmp_path), "events.csv"))))
    assert ev[-1]["kind"] == "motor board" and "not answering" in ev[-1]["text"]


def test_firmware_version_bumped_and_the_runner_accepts_it():
    ino = open(os.path.join(REPO, "firmware", "home_lab", "home_lab.ino")).read()
    assert ('#define FW_VERSION "2026-11-' in ino and "2026-10-29b (2 Nov" in ino) and "power::tick()" in ino and "board %s, sensors %lu ms old" in ino
    sys.path.insert(0, os.path.join(TOOLS, "experiment"))
    src = open(os.path.join(TOOLS, "experiment", "e02.py")).read()
    for name in ("OK_FW", "BORDER_FW", "BACK_FW", "TILT_FW"):
        line = [ln for ln in src.splitlines() if ln.startswith(name + " = (")][0]
        assert '"2026-10-29b"' in line
