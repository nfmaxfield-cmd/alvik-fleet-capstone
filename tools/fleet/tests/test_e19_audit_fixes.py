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
