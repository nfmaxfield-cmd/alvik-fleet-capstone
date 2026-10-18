"""Tests for the live link (tools/live). About 40 s: three short synthetic
scenes are drawn (tools/blink/tests/synthworld.py) and tracked frame by frame.

    python -m pytest tools/live/tests -q
"""
import json
import math
import os
import socket
import sys
import threading
import time
from http.server import ThreadingHTTPServer

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LIVE = os.path.dirname(HERE)
TOOLS = os.path.dirname(LIVE)
for p in (HERE, LIVE, TOOLS, os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "blink"),
          os.path.join(TOOLS, "blink", "tests"), os.path.join(TOOLS, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)

import scenes as SC  # noqa: E402
import floormap as FMAP  # noqa: E402
import livecore as LC  # noqa: E402
import sources as S  # noqa: E402
import track_live as TL  # noqa: E402
import capture_delay as CD  # noqa: E402
import synthworld as SW  # noqa: E402


@pytest.fixture(scope="module")
def crossing(tmp_path_factory):
    out = str(tmp_path_factory.mktemp("crossing"))
    sc, robots = SC.render(out, "crossing", 2)     # seed 2: a flash that both robots could have had
    tr, msgs = SC.track(out)
    return out, sc, robots, tr, msgs


def test_crossing_right_names_and_accuracy(crossing):
    """Two robots crossing twice and passing head-on 11 cm apart, phases 0.12 s
    apart, among distracting lamps: every fix has the right name, and the pose is
    accurate."""
    out, sc, robots, tr, msgs = crossing
    r = SC.score(sc, robots, msgs)
    assert r["wrong"] == 0 and r["far"] == 0
    assert r["per"].get("Left", 0) >= 40 and r["per"].get("Center", 0) >= 15
    e, h = np.array(r["err"]), np.array(r["herr"])
    assert np.percentile(e, 95) < 1.0 and e.max() < 2.5
    assert np.percentile(h, 95) < 2.0 and h.max() < 6.0
    assert tr.stats["fast"] > tr.stats["late"]          # most fixes come straight from the frame


def test_fix_messages_have_run_plan_fields(crossing):
    _, _, _, _, msgs = crossing
    fixes = [m for m in msgs if m["type"] == "fix"]
    for m in fixes:
        for k in ("robot", "t_pi", "x", "y", "th", "sigma", "sigma_th", "id_p", "path"):
            assert k in m
        assert m["id_p"] >= 0.9 and m["path"] in ("fast", "late")
    # never two fixes for the same robot and frame
    keys = [(m["robot"], m["frame"]) for m in fixes]
    assert len(keys) == len(set(keys))


def test_distractors_give_no_fixes(tmp_path):
    """Flickering lamps, a 4 Hz flicker, random 200 ms pulses, a moving white
    object, idle robots with steady LEDs: nothing is sent."""
    out = str(tmp_path / "d")
    sc, robots = SC.render(out, "distractors", 2, seconds=10)
    tr, msgs = SC.track(out)
    assert [m for m in msgs if m["type"] == "fix"] == []


def test_two_robots_same_code_are_held_back(tmp_path):
    """Two robots blinking the same code (a set-up mistake): once both are seen,
    neither gets fixes under that name."""
    cam = SW.Cam()
    T = 9.0
    a = SW.Robot("Left", 10, 10, 0, code=1, t_end=T + 1)
    a.drive(T, 0)
    a.add_blink(0.3, T, phase=0.3)
    b = SW.Robot("Left2", 60, 30, 90, code=1, t_end=T + 1)
    b.drive(T, 0)
    b.add_blink(0.3, T, phase=0.42)
    sc = SW.Scene(cam, [a, b], [], T, seed=3, clutter=False)
    out = str(tmp_path / "same")
    sc.render(out)
    tr, msgs = SC.track(out)
    late = [m for m in msgs if m["type"] == "fix" and m["t_pi"] > 4.5]
    assert late == []
    assert tr.stats["contested"] > 0


def test_floormap_matches_simulator_camera(tmp_path):
    """Pixels of a point 4 cm up -> the floor point below it, for the real Oct 18
    camera set-up (simulator geometry), within 0.02 cm."""
    import config as SCFG
    import camera as SCAM
    cfg = SCFG.load_config(os.path.join(TOOLS, "sim", "configs", "real_oct18.json"))
    g = SCAM.CameraGeometry(cfg.camera)
    path = tmp_path / "floor_calibration.json"
    path.write_text(json.dumps(g.calibration()))
    fm = FMAP.FloorMap(str(path))
    assert 3.3 < fm.px_per_cm() < 3.6
    for x, y in ((10, 10), (61, 61), (110, 20), (5, 115)):
        u, v = g.project(x, y, 4.0)
        fx, fy = fm.to_floor(float(u), float(v))
        assert math.hypot(fx - x, fy - y) < 0.02
        u2, v2 = fm.to_px(x, y)
        assert math.hypot(u2 - float(u), v2 - float(v)) < 0.01


def test_clock_envelope_takes_the_fastest_trip():
    rng = np.random.default_rng(0)
    c = S.ClockEnvelope(window_s=30)
    true_off = 12.345                      # laptop - Pi
    for k in range(400):
        t_pi = 1000.0 + k * 0.04
        trip = 0.002 + abs(rng.normal(0, 0.01)) + (0.2 if k % 50 == 7 else 0.0)
        c.add(t_pi + true_off + trip, t_pi)
    assert 0.0 <= c.offset - true_off < 0.004


def test_replay_keeps_real_time(crossing):
    out = crossing[0]
    now = [100.0]
    slept = []

    def clock():
        return now[0]

    def sleep(s):
        slept.append(s)
        now[0] += s
    src = S.ReplaySource(out, speed=1.0, max_s=1.0, clock=clock, sleep=sleep)
    frames = list(src)
    assert len(frames) >= 25
    for f in frames:
        assert abs((f.t_arrived - frames[0].t_arrived) - (f.t_pi - frames[0].t_pi)) < 1e-6
    assert sum(slept) == pytest.approx(frames[-1].t_pi - frames[0].t_pi, abs=1e-6)


def test_camrec_live_stream_end_to_end(crossing):
    """camrec.py's /live endpoint -> StreamSource: frame numbers, Pi times and
    pictures come through; the clock offset is found."""
    import camrec
    import cv2
    out = crossing[0]
    cap = cv2.VideoCapture(os.path.join(out, "video.avi"))
    imgs = [cap.read()[1] for _ in range(6)]
    cap.release()
    live = camrec.LiveFrames()
    server = ThreadingHTTPServer(("127.0.0.1", 0), camrec.make_handler(camrec.Latest(), live))
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    stop = threading.Event()

    def feed():
        while not live.clients and not stop.is_set():
            time.sleep(0.01)
        for n in range(200):
            if stop.is_set():
                break
            live.put(imgs[n % len(imgs)], n, 5000.0 + n * 0.04)
            time.sleep(0.04)
    threading.Thread(target=feed, daemon=True).start()
    src = S.StreamSource("127.0.0.1:%d" % port, log=lambda *a: None)
    got = []
    for fr in src:
        got.append(fr)
        if len(got) >= 5:
            break
    stop.set()
    src.close()
    server.shutdown()
    assert [f.pi_frame for f in got] == list(range(got[0].pi_frame, got[0].pi_frame + 5))
    assert got[0].img.shape == (480, 640) and got[0].img.dtype == np.uint8
    assert abs(got[1].t_pi - got[0].t_pi - 0.04) < 1e-3
    gray = cv2.cvtColor(imgs[got[0].pi_frame % len(imgs)], cv2.COLOR_BGR2GRAY)
    assert np.abs(got[0].img.astype(int) - gray).mean() < 3      # JPEG quality 90


def test_messages_are_read_by_run_plan():
    """The UDP JSON lines go straight into run_plan.py's own position source."""
    import run_plan as RP
    src = RP.UdpSource(0, host="127.0.0.1")
    port = src.sock.getsockname()[1]
    snd = TL.Sender("127.0.0.1:%d" % port)
    msg = dict(type="fix", robot="Left", t=time.time(), x=10.0, y=20.0, th=90.0, sigma=0.5, sigma_th=1.5,
               id_p=0.99, path="fast", t_pi=1.0, frame=3, track=1)
    snd.send(msg)
    snd.send(dict(type="sighting", x=1, y=2))
    got = []
    t0 = time.time()
    while time.time() - t0 < 2 and not got:
        got = src.poll()
        time.sleep(0.01)
    snd.close()
    src.close()
    assert len(got) == 1 and got[0]["robot"] == "Left" and got[0]["th"] == 90.0     # the sighting is ignored


def test_heading_needs_resolving_before_fixes():
    """A track whose front/back isn't clear yet gets no fixes."""
    T = LC.LiveTrack(1)
    T.h, T.votes = 10.0, 1.0
    assert not T.resolved
    T.votes = 2.5
    assert T.resolved


def test_capture_delay_formula():
    # video t_wall - laptop time of the flash = (Pi - laptop) + delay; clock.csv gives laptop - Pi
    assert CD.capture_delay(0.04 + 0.04, -0.04) == pytest.approx(0.04)
