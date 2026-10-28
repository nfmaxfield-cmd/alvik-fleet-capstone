"""Practice day 2026-10-28 (docs/session-plans/2026-10-28.md): one block at a time (practice programs and
`run-fleet-pi.sh practice`), the stop-everything command, the bench settings read-back, and the camera setup's
surface (the home plywood is bigger than Monday's table)."""
import json
import os
import socket
import subprocess
import sys
import threading

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(FLEET))
sys.path.insert(0, FLEET)
import fleetmap as FM          # noqa: E402
import mission_control as MC   # noqa: E402
import missions as MS          # noqa: E402
import robot_cmd as RC         # noqa: E402
import stop_all as SA          # noqa: E402
import workspace as WS         # noqa: E402

PRACTICE = os.path.join(FLEET, "examples", "practice")


@pytest.mark.parametrize("name", ["mow", "circle", "chicken", "dance"])
@pytest.mark.parametrize("table", [(122, 61), (130, 70), (137, 76)])
def test_practice_blocks_plan_on_every_monday_table(name, table):
    with open(os.path.join(PRACTICE, name + ".json")) as f:
        prog = json.load(f)
    m = WS.blank_map((0, 0) + table, [("Left", [20, 15, 90]), ("Center", [50, 15, 90]), ("Right", [80, 15, 90])])
    mm = MS.build(m, prog)
    plan = MS.plan_synced(mm, t_limit_s=30)
    assert sorted(plan["robots"]) == sorted(prog["robots"])
    kinds = {st["type"] for r in plan["robots"].values() for st in r["steps"]}
    if name == "circle":
        assert "arc" in kinds
    if name == "dance":
        assert "dance" in kinds
        # a practice dance ends by itself (Monday's own dance goes on until switched off)
        assert all(st.get("dance_ms") == 10000 for r in plan["robots"].values() for st in r["steps"]
                   if st["type"] == "dance")
    f = mm["floor"]
    for r in plan["robots"].values():                  # nothing planned outside the table
        for st in r["steps"]:
            x, y = st["to"][:2]
            assert f["x_min"] < x < f["x_max"] and f["y_min"] < y < f["y_max"]


def test_launcher_practice_needs_a_known_block():
    env = dict(os.environ, FLEET_DRY="1")
    r = subprocess.run(["bash", os.path.join(REPO, "run-fleet-pi.sh"), "practice", "nope"], capture_output=True,
                       text=True, env=env, cwd=REPO)
    assert r.returncode == 1 and "mow|circle|chicken|dance" in r.stdout


def test_stop_all_sends_stop_to_every_robot_without_the_log_port():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(2.0)
    port = rx.getsockname()[1]
    got = []

    def recv():
        try:
            while len(got) < SA.STOP_REPEATS:
                got.append(rx.recv(64))
        except socket.timeout:
            pass
    t = threading.Thread(target=recv)
    t.start()
    rc = SA.main(["--robot", "127.0.0.1", "--port", str(port), "--no-tmux", "--mc-url", "http://127.0.0.1:9/x"])
    t.join()
    rx.close()
    assert rc == 0 and got == [b"stop"] * SA.STOP_REPEATS


def test_stop_all_tmux_only_touches_running_fleet_sessions():
    calls = []

    class R:
        def __init__(self, rc):
            self.returncode = rc

    def fake_run(cmd, capture_output=True):
        calls.append(cmd)
        return R(0 if cmd[1] == "has-session" and cmd[3] in ("practice",) else 1)
    assert SA.tmux_ctrl_c(run=fake_run) == ["practice"]
    sent = [c for c in calls if c[1] == "send-keys"]
    assert sent == [["tmux", "send-keys", "-t", "practice", "C-c"]]


def test_robot_cmd_then_sends_each_command_in_order(monkeypatch, tmp_path):
    sent = []

    class FakeLink:
        def __init__(self, out, ips):
            self.session = type("S", (), {"listeners": []})()

        def robots(self):
            return ["Left", "Right"]

        def send_name(self, n, c):
            sent.append((n, c))

        def close(self):
            sent.append(("all", "stop"))
    monkeypatch.setattr(RC.AC, "Link", FakeLink)
    monkeypatch.setattr(RC.time, "sleep", lambda s: None)
    RC.main(["--to", "Left", "--wait-s", "0", "--find-s", "0", "--out", str(tmp_path), "set gyro_scale 1.075",
             "--then", "set wheel_scale 0.979", "--then", "get gyro_scale", "--then", "get wheel_scale"])
    assert sent == [("Left", "set gyro_scale 1.075"), ("Left", "set wheel_scale 0.979"), ("Left", "get gyro_scale"),
                    ("Left", "get wheel_scale"), ("Left", "status"), ("all", "stop")]


def test_setup_uses_the_surface_not_the_mission_table(tmp_path, monkeypatch):
    s = MC.Session(str(tmp_path / "mc"))
    s.set_map({"floor": {"x_min": 0, "y_min": 0, "x_max": 122, "y_max": 61}})
    assert s.surface() == "122x61"                      # nothing said: the table's size (the school case)
    seen = {}
    monkeypatch.setattr(MC.threading, "Thread",
                        lambda target, args, daemon: type("T", (), {"start": lambda self: seen.update(table=args[2])})())
    s.setup_start({"conn": "robots", "surface": "122 x 122"})
    assert seen["table"] == "122x122"
    assert s.state()["surface"] == "122x122"
    # kept in the map for next time, and the mission's table is unchanged
    m = FM.load(os.path.join(str(tmp_path / "mc"), "map.json"))
    assert m["surface_cm"] == [122.0, 122.0]
    assert (m["floor"]["x_max"], m["floor"]["y_max"]) == (122.0, 61.0)
    s2 = MC.Session(str(tmp_path / "mc"))
    s2.setup_start({"conn": "robots"})
    assert seen["table"] == "122x122"
    with pytest.raises(ValueError):
        s.surface("0x122")
