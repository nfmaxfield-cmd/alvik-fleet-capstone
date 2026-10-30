"""Robots put down anywhere on the board (build e02-oct30-robots-anywhere.md, 2026-10-30): the camera bridge is seeded
from the facing check's lights, position-only fixes are held from a robot with no pose, the calibration's roll caps
come from where the lights are, and a skipped roll is explained from the robot's own numbers.

    python3 -m pytest tools/fleet/tests/test_robots_anywhere.py -q
    FLEET_DRYRUN=1 python3 -m pytest tools/fleet/tests/test_robots_anywhere.py -q -k dry     # + 2 dry runs (~4 min)
"""
import json
import math
import os
import subprocess
import sys
from collections import deque

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "experiment"), TOOLS):
    if p not in sys.path:
        sys.path.append(p)

import cam_bridge as CB     # noqa: E402
import run_plan as RP       # noqa: E402


class _Track:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _Trk:
    def __init__(self):
        self.tracks = {"Left": _Track(17.0, 17.0)}
        self.calls = []
        self.side = None
        self.frame_t = 5.0

    def add(self, n, x, y, heading=None, t=None):
        self.tracks[n] = _Track(x, y)
        self.calls.append(("add", n, x, y, heading))

    def robot_at(self, x, y, scale=1.2):
        self.calls.append(("robot_at", x, y))

    def refresh_spot(self, x, y, scale=1.0):
        self.calls.append(("refresh", x, y))


def _bridge():
    br = CB.Bridge.__new__(CB.Bridge)

    class A:
        farm = False
    br.args = A()
    br.starts = {"Left": (17.0, 17.0, 90.0), "Right": (105.0, 17.0, 90.0)}
    br.has_pose = {"Left": False, "Right": False}
    br.moved = {"Left": False, "Right": False}
    br.last_obs = {"Left": 3.0, "Right": None}
    br.start_heads = {"Left": deque([1.0]), "Right": deque()}
    br.relearned = {"Left": [], "Right": []}
    br.trk = _Trk()
    br.events = []
    br.event = lambda kind, **k: br.events.append((kind, k))
    br.say = lambda s: None
    return br


def test_seed_moves_the_track_and_relearns_both_spots():
    br = _bridge()
    br.on_seed({"type": "seed", "robot": "Left", "x": 45.0, "y": 40.0, "th": -89.0})
    assert br.starts["Left"] == (45.0, 40.0, -89.0)              # its first fixes carry the lights' heading
    assert ("add", "Left", 45.0, 40.0, -89.0) in br.trk.calls
    assert ("robot_at", 45.0, 40.0) in br.trk.calls               # its body was in the floor memory: forgotten
    assert ("refresh", 17.0, 17.0) in br.trk.calls                # its start spot, empty: learned again
    assert br.last_obs["Left"] is None and not br.start_heads["Left"]
    n = len(br.trk.calls)
    br.on_seed({"type": "seed", "robot": "Left", "x": 45.3, "y": 40.1, "th": -88.0})   # the same seed resent
    assert len(br.trk.calls) == n


def test_seed_ignored_once_it_has_its_own_pose():
    br = _bridge()
    br.has_pose["Left"] = True
    br.on_seed({"type": "seed", "robot": "Left", "x": 45.0, "y": 40.0, "th": -89.0})
    assert br.starts["Left"] == (17.0, 17.0, 90.0) and not br.trk.calls


def test_seed_does_not_relearn_a_spot_another_robot_stands_near():
    br = _bridge()
    br.starts["Right"] = (25.0, 20.0, 90.0)                       # Right stands 8.5 cm from Left's old spot
    br.on_seed({"type": "seed", "robot": "Left", "x": 45.0, "y": 40.0, "th": 0.0})
    assert not [c for c in br.trk.calls if c[0] == "refresh"]


def test_position_only_fix_is_held_from_a_robot_with_no_pose():
    c = RP.Coordinator.__new__(RP.Coordinator)

    class A:
        fix_hz = 10.0
        straighten = True

    class Src:
        live = True

        def poll(self):
            return []
    c.args, c.source = A(), Src()
    b = RP.Bot("Right")
    b.offset, b.fix_sent_t = 0.0, 0.0
    b.fix = {"robot": "Right", "t": RP.now(), "x": 80.0, "y": 45.0, "th": 180.0}
    c.bots, sent = {"Right": b}, []
    c.send = lambda n, t: sent.append(t)
    c.fixes()
    assert not sent and b.fix_held == 1                           # no pose: it would refuse it (09:52, 76 of 77)
    b.pose, b.fposed_t = (80.0, 45.0, 180.0), RP.now() - 5
    b.fix = {"robot": "Right", "t": RP.now(), "x": 80.0, "y": 45.0, "th": 180.0}
    c.fixes()
    assert len(sent) == 1 and sent[0].startswith("fix ")


def test_facing_poses_take_the_middle_behind_the_lights():
    fres = {"robots": {"Left": {"meas": {"mid": [45.0, 40.0], "heading": 0.0, "sure": True}},
                       "Right": {"meas": {"heading": None}}},
            "headings": {"Left": 0.0}}
    fp = RP.facing_poses(fres, ["Left", "Right"])
    assert set(fp) == {"Left"}
    x, y, h, how, hh = fp["Left"]
    assert x == pytest.approx(45.0 - RP.LIGHTS_AHEAD_CM) and y == pytest.approx(40.0) and h == 0.0


def test_calib_caps_use_the_lights_not_the_plan_start():
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.names = ["Left"]
    c.plan = {"robots": {"Left": {"start": [17.0, 17.0, 90.0]}}}
    c.facing_at = {"Left": (45.0, 40.0, -89.0, "lights (facing check)", -89.0)}
    poses = RP.calib_poses(c, ["Left"], source=None)
    assert poses["Left"] == (45.0, 40.0, -89.0, "lights (facing check)")


def test_skipped_roll_is_explained_from_the_robots_numbers():
    w = RP.calib_roll_words("Left", {"room8": "12.1;13.0;11.5;14.2;12.0;12.8;13.3;11.9", "live_cm": "12.0",
                                     "caps": "0:15.0;45:14.0"})
    assert "reading the floor" in w and "12/13/12/14/12/13/13/12" in w
    w = RP.calib_roll_words("Right", {"room8": "999.0;8.0;999.0;999.0;999.0;999.0;999.0;999.0"})
    assert "floor" not in w and "clear" in w
    assert RP.calib_roll_words("Right", {}) == ""


@pytest.mark.skipif(os.environ.get("FLEET_DRYRUN") != "1", reason="end-to-end dry run: set FLEET_DRYRUN=1")
@pytest.mark.parametrize("table,starts", [("126x126", {"Left": [45, 40, -90], "Right": [85, 45, 180]}),
                                          ("61x122", {"Left": [40, 45, 180], "Right": [80, 30, -90]})])
def test_dry_monday_robots_put_down_far_off_backwards_and_sideways(tmp_path, table, starts):
    sp = tmp_path / "starts.json"
    sp.write_text(json.dumps(starts))
    r = subprocess.run([sys.executable, os.path.join(FLEET, "mission_run.py"), "--dry", "--table", table,
                        "--session", str(tmp_path / "s"), "--sim-arg=--starts=%s" % sp, "--sim-arg=--uncalibrated",
                        "--sim-arg=--old-led", "--no-frames"], capture_output=True, text=True, timeout=600, cwd=REPO)
    out = r.stdout + r.stderr
    assert "Result: done" in out, out[-3000:]
    assert "used/ignored on the robots {'Left': (" in out
    summ = json.load(open(tmp_path / "s" / "run" / "run_plan_summary.json"))
    for n, (used, rej) in summ["fixes_used_rejected"].items():
        assert int(rej) <= 2, (n, used, rej)
    for n in starts:
        assert "%s: ready at" % n in out
