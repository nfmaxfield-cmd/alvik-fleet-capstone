"""The facing check (facing.py, 2026-10-29): which way each robot faces, from its lights, before anything moves."""
import math
import os
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(FLEET))
sys.path.insert(0, FLEET)
sys.path.insert(0, os.path.join(REPO, "tools", "sim"))

import facing as F  # noqa: E402

cv2 = pytest.importorskip("cv2")


class Cam:
    """A camera straight down: 1 px = 0.5 cm."""

    def to_floor(self, u, v, z):
        return u * 0.5, v * 0.5


class World:
    """Robots (x, y, heading) whose two lights sit 3.4 cm either side of the middle, and a `led` like the firmware's."""

    def __init__(self, robots, new_fw=True, idle_left=True, lose=()):
        self.r = {n: list(p) for n, p in robots.items()}
        self.state = {n: ("idle", 0.0) for n in robots}      # (what, until)
        self.new_fw, self.idle_left = new_fw, idle_left
        self.said = []
        self.lose = list(lose)                               # commands lost once each (WiFi)
        self.rng = np.random.default_rng(2)
        self.base = (150 + self.rng.normal(0, 3, (300, 300))).astype(np.float32)
        self.frames = []

    def send(self, n, c):
        if c in self.lose:
            self.lose.remove(c)
            return
        w = c.split()
        ms = float(w[4]) / 1000.0
        side = w[5] if len(w) > 5 and self.new_fw else ""
        on = any(int(v) for v in w[1:4])
        until = time.time() + (ms if ms >= 0.1 else 0.0)       # the check waits out a short `led 0 0 0 60` (settle)
        self.state[n] = (("L" if side == "L" else "R" if side == "R" else "both") if on else "dark", until)
        self.said.append((time.time(), n, "%s: led %s for %d ms" % (n, " ".join(w[1:4]), ms * 1000)))

    def acked(self, n, t, words, timeout=0.3):
        for tt, nn, v in self.said:
            if nn == n and tt >= t - 0.01 and words in v:
                return tt
        return None

    def lights(self, n):
        what, until = self.state[n]
        if time.time() > until:
            what = "idle"
        x, y, h = self.r[n]
        r = math.radians(h)
        L = (x - 3.4 * math.sin(r), y + 3.4 * math.cos(r))
        R = (x + 3.4 * math.sin(r), y - 3.4 * math.cos(r))
        return {"both": [L, R], "L": [L], "R": [R], "dark": [], "idle": [L] if self.idle_left else []}[what]

    def picture(self):
        im = self.base + self.rng.normal(0, 2, self.base.shape)
        for n in self.r:
            for x, y in self.lights(n):
                cv2.circle(im, (int(round(x / 0.5)), int(round(y / 0.5))), 2, 255, -1)
        return np.clip(im, 0, 255).astype(np.uint8)


class FakeGrab:
    def __init__(self, world):
        self.w = world

    def between(self, name, t0, t1, wait_s=0):
        return [self.w.picture() for _ in range(4)]


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(F, "SETTLE_S", 0.0)
    monkeypatch.setattr(F, "COLLECT_S", 0.0)


def run(world, assumed, fw="2026-10-29a", tol=F.TOL_DEG):
    said = []
    res = F.check(list(world.r), world.send, world.acked, FakeGrab(world), {"overhead": Cam()}, assumed=assumed,
                  firmware={n: fw for n in world.r}, rect=(0, 0, 140, 140), tol=tol, say=said.append)
    return res, said


@pytest.mark.parametrize("h", list(range(-180, 180, 30)))
def test_one_side_lights_give_the_heading_every_way_round(h):
    w = World({"Left": (40, 50, h)})
    res, _ = run(w, {"Left": h})
    m = res["robots"]["Left"]["meas"]
    assert m["heading"] is not None and abs(F.wrap180(m["heading"] - h)) < 5
    assert res["ok"]


def test_backwards_and_sideways_stop_everything_in_plain_words():
    w = World({"Left": (30, 30, 270), "Right": (100, 30, 0), "Center": (60, 80, 92)})
    res, said = run(w, {"Left": 90, "Right": 90, "Center": 90})
    assert not res["ok"]
    words = " ".join(said)
    assert "Left is facing the wrong way (about 180 deg off): turn it round so its front (the distance sensor)" in words
    assert "Right is turned about 90 deg to its right (sideways)" in words
    assert "Center faces its start direction" in words
    assert res["robots"]["Center"]["verdict"]["ok"]


def test_turned_a_little_more_than_the_start_allows_stops_it():
    w = World({"Left": (30, 30, 108)})
    res, said = run(w, {"Left": 90})
    assert not res["ok"] and "turned about 18 deg to its left (its start allows 15)" in said[0]


def test_old_firmware_uses_the_idle_left_light_for_the_front():
    w = World({"Left": (30, 30, 270)}, new_fw=False)
    res, said = run(w, {"Left": 90}, fw="2026-10-28a")
    assert res["robots"]["Left"]["meas"]["heading"] is not None
    assert "facing the wrong way" in said[0]


def test_old_firmware_with_its_idle_light_off_still_catches_sideways():
    w = World({"Left": (30, 30, 0), "Right": (100, 30, 270)}, new_fw=False, idle_left=False)
    res, said = run(w, {"Left": 90, "Right": 90}, fw="2026-10-28a")
    words = " ".join(said)
    assert "Left is turned sideways" in words
    assert "Right: not sideways, but its front and back could not be told apart" in words
    assert not res["ok"]                     # Left sideways stops the start; Right alone would not


def test_a_lost_led_command_is_sent_again():
    w = World({"Left": (30, 30, 90)}, lose=["led 1 1 1 1500 L"])
    res, _ = run(w, {"Left": 90})
    assert res["ok"] and abs(res["robots"]["Left"]["meas"]["heading"] - 90) < 5


def test_another_robots_light_left_on_is_not_taken():
    w = World({"Left": (30, 30, 90), "Right": (100, 30, 90)})
    real = w.send

    def keep_left_lit(n, c):             # Left's "dark again" lost every time: its right light stays on
        if n == "Left" and c.startswith("led 0 0 0") and w.state["Left"][0] == "R":
            w.said.append((time.time(), n, "Left: led 0 0 0 for 15000 ms"))
            return
        real(n, c)
    w.send = keep_left_lit
    res, _ = run(w, {"Left": 90, "Right": 90})
    assert abs(res["robots"]["Right"]["meas"]["heading"] - 90) < 5


def test_two_cameras_that_disagree_are_not_sure():
    a = dict(ok=True, heading=90.0, mid=(10, 10))
    b = dict(ok=True, heading=150.0, mid=(10, 10))
    m = F.combine({"overhead": a, "side": b})
    assert not m["sure"]
    v = F.verdict("Left", m, 90.0)
    assert not v["ok"] and "the two cameras disagree" in v["words"]
    m = F.combine({"overhead": dict(a, heading=100.0), "side": dict(b, heading=92.0)})
    assert m["sure"] and 92 <= m["heading"] < 94          # the side camera weighs more


def test_swapped_robots_are_named():
    res = {"Left": {"meas": {"mid": (100, 18)}}, "Right": {"meas": {"mid": (17, 17)}}}
    w = F.identity_words(res, {"Left": (17, 17), "Right": (105, 17)})
    assert any("Left's lights are on Right's start spot" in x for x in w)


def test_towards_words():
    assert F.towards_words(90, (0, 0, 122, 61)) == "towards the y = 61 side (90 deg)"
    assert F.towards_words(180, (0, 0, 122, 61)) == "towards the x = 0 end (180 deg)"


def test_firmware_lights_one_side_and_says_so():
    src = open(os.path.join(REPO, "firmware", "home_lab", "home_lab.ino")).read()
    assert '#define FW_VERSION "2026-10-29a"' in src
    assert 'sscanf(raw.c_str(), "%*s %d %d %d %ld %3s", &r, &g, &bl, &ms, side);' in src
    assert 'onlyL ? ", left only" : (onlyR ? ", right only" : "")' in src


def test_simulated_robot_lights_one_side():
    import fleet_sim as FS

    class Rb(FS.LiveRobot):
        def __init__(self):
            self.name, self.sent, self.leds = "Left", [], []

        def millis(self):
            return 0

        def send(self, text):
            self.sent.append(text)
    rb = Rb()
    rb.handle_command("led 1 1 1 1500 L", False)
    assert rb.lights() == ((1.0, 1.0, 1.0), (0.0, 0.0, 0.0)) and rb.sent[-1].endswith(", left only")
    rb.handle_command("led 1 1 1 1500 R", False)
    assert rb.lights() == ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    rb.handle_command("led 0 0 0 3000", False)
    assert rb.lights() == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))     # dark for 3 s, as the firmware
    rb.one_side_led = False
    rb.handle_command("led 1 1 1 1500 L", False)
    assert rb.lights() == ((1.0, 1.0, 1.0), (1.0, 1.0, 1.0))


def test_ladder_drops_a_kept_heading_the_lights_contradict(tmp_path):
    import ladder as LD

    class L:
        st = {"robots": {"Center": {}}}
        dir = str(tmp_path)
        facing_heading = LD.Ladder.facing_heading
        log_event = LD.Ladder.log_event
    lad = L()
    side = {"side": {"heading": -150.0}}
    h, how = lad.facing_heading("Center", 41.0, dict(heading=-148.0, sure=True, how="both", cameras=side))
    assert h is None and "its lights said -148" in how                       # 28 Oct: Center kept 41, faced ~210
    h, how = lad.facing_heading("Center", -146.0, dict(heading=-148.0, sure=True, how="both", cameras=side))
    assert h == -146.0 and how is None                                      # agrees: kept
    h, how = lad.facing_heading("Center", None, dict(heading=-148.0, sure=True, how="both", cameras=side))
    assert h == -148.0 and how.startswith("its lights")                    # unknown: the lights' heading
    h, how = lad.facing_heading("Center", None, dict(heading=-148.0, sure=True, how="overhead alone",
                                                     cameras={"overhead": {"heading": -148.0}}))
    assert h is None                                                       # overhead alone: not enough to drive by
