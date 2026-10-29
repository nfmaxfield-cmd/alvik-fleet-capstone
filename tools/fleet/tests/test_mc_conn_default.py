"""Mission Control's Connect screen opens on "Robots" on the Pi with real cameras, else the simulator (2026-10-29)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
sys.path.insert(0, FLEET)

import mission_control as MC  # noqa: E402


def test_robots_when_the_real_cameras_answer(monkeypatch):
    monkeypatch.delenv("FLEET_DRY", raising=False)
    d = MC.conn_default(probe=lambda p: p in (8000, 8001))
    assert d["mode"] == "robots" and "overhead and side" in d["why"]
    assert MC.conn_default(probe=lambda p: p == 8000)["mode"] == "robots"


def test_simulator_off_the_pi_and_for_dry_runs(monkeypatch):
    monkeypatch.delenv("FLEET_DRY", raising=False)
    assert MC.conn_default(probe=lambda p: False)["mode"] == "sim"            # no camera: off the Pi
    monkeypatch.setenv("FLEET_DRY", "1")
    d = MC.conn_default(probe=lambda p: True)                                 # cameras up, but a dry run
    assert d["mode"] == "sim" and "FLEET_DRY" in d["why"]
    assert MC.conn_default(dry=True, probe=lambda p: True)["mode"] == "sim"   # mission_control.py --dry


def test_state_carries_it_and_the_page_follows_it_until_picked(tmp_path, monkeypatch):
    monkeypatch.setenv("FLEET_DRY", "1")
    S = MC.Session(str(tmp_path / "mc"))
    assert S.state()["conn_default"]["mode"] == "sim"
    html = open(os.path.join(FLEET, "mission_control.html")).read()
    assert "if (!connPicked && S.conn_default) conn = S.conn_default.mode" in html
    assert 'id="connmode"' in html and "connPicked = true" in html
