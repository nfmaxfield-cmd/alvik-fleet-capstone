"""tools/pi/camrec.py with two (fake) cameras: folder layout, session.json,
shared clock, meta.json, live streams on port and port + 1. No hardware."""
import csv
import importlib.util
import json
import os
import socket
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np
import pytest

PI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.dirname(PI)
if PI not in sys.path:
    sys.path.insert(0, PI)

import camrec  # noqa: E402


class FakeCapture:
    """A webcam: a frame every 1/fps s on the system clock, with the frame number
    drawn in, and a driver timestamp (POS_MSEC). drop_every: skip a frame now and then."""

    def __init__(self, fps=30, size=(64, 48), shade=100, drop_every=0):
        self.fps, self.size, self.shade, self.drop_every = fps, size, shade, drop_every
        self.k = 0
        self.t0 = None
        self.released = False
        self.last_ms = 0.0

    def read(self):
        if self.released:
            return False, None
        if self.t0 is None:
            self.t0 = time.monotonic()
        self.k += 1
        if self.drop_every and self.k % self.drop_every == 0:
            self.k += 1                                  # this one was lost on the bus
        due = self.t0 + self.k / self.fps
        wait = due - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        self.last_ms = (due - self.t0) * 1000.0
        img = np.full((self.size[1], self.size[0], 3), self.shade, np.uint8)
        cv2.putText(img, str(self.k % 100), (2, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        return True, img

    def get(self, prop):
        if prop == cv2.CAP_PROP_FPS:
            return float(self.fps)
        if prop == cv2.CAP_PROP_POS_MSEC:
            return self.last_ms
        if prop == cv2.CAP_PROP_FOURCC:
            return float(cv2.VideoWriter_fourcc(*"MJPG"))
        return -1.0

    def release(self):
        self.released = True


def read_frames(folder):
    with open(os.path.join(folder, "frames.csv"), newline="") as f:
        return list(csv.DictReader(f))


def record_two(tmp_path, seconds=2.0, port=0, caps=None, **kw):
    caps = caps or [FakeCapture(30, shade=80), FakeCapture(25, shade=160)]
    lines = []
    folder, cams = camrec.run_session(caps, ["0", "2"], names=["overhead", "side"], out=str(tmp_path),
                                      label="T1", seconds=seconds, fps=30, port=port, notes_port=0,
                                      stdin_notes=False, replays=[False, False], log=lines.append,
                                      disk_check_s=0, **kw)
    return folder, cams, lines


def test_two_cameras_layout_session_and_shared_clock(tmp_path):
    t_before = time.monotonic()
    folder, cams, lines = record_two(tmp_path)
    t_after = time.monotonic()
    assert os.path.basename(folder).endswith("_T1")
    assert sorted(os.listdir(folder)) == ["cam0", "cam1", "session.json"]
    for sub in ("cam0", "cam1"):
        assert sorted(os.listdir(os.path.join(folder, sub))) == ["events.csv", "frames.csv", "meta.json", "video.avi"]

    s = json.load(open(os.path.join(folder, "session.json")))
    assert s["clock"]["t_mono_shared"] is True and s["capture"] == "threads in one process"
    assert [c["folder"] for c in s["cameras"]] == ["cam0", "cam1"]
    assert [c["device"] for c in s["cameras"]] == ["0", "2"]
    assert [c["role"] for c in s["cameras"]] == ["overhead", "side"]
    assert all(c["frames"] > 30 for c in s["cameras"]) and "end" in s
    t0 = s["clock"]["t0_monotonic"]
    assert t_before <= t0 <= t_after

    rows = [read_frames(os.path.join(folder, sub)) for sub in ("cam0", "cam1")]
    for i, (r, fps) in enumerate(zip(rows, (30, 25))):
        meta = json.load(open(os.path.join(folder, f"cam{i}", "meta.json")))
        assert meta["fps_reported"] == fps and meta["width"] == 64 and meta["height"] == 48
        assert meta["t0_monotonic"] == t0 and meta["role"] == ["overhead", "side"][i] and meta["camera_index"] == i
        assert meta["frame_check"]["missed_estimate"] == 0 and meta["frame_check"]["ok"]
        tm = [float(x["t_mono"]) for x in r]
        assert [int(x["frame"]) for x in r] == list(range(len(r)))
        assert all(b > a for a, b in zip(tm, tm[1:])), "t_mono must increase"
        assert 0.3 < tm[0] < 0.8                     # after the 0.5 s buffer flush
        assert 1.9 < tm[-1] < 2.2                    # --seconds 2 on the shared clock
        assert abs(len(r) / (tm[-1] - tm[0]) - fps) < 0.15 * fps
        # t_mono and t_wall come from clocks running together: their difference is constant
        d = [float(x["t_wall"]) - float(x["t_mono"]) for x in r]
        assert max(d) - min(d) < 0.02
    # shared clock: both cameras started and stopped together on the same t_mono scale,
    # and the same instant has the same t_mono - t_wall offset in both
    t_first = [float(r[0]["t_mono"]) for r in rows]
    t_last = [float(r[-1]["t_mono"]) for r in rows]
    assert abs(t_first[0] - t_first[1]) < 0.1 and abs(t_last[0] - t_last[1]) < 0.1
    off = [float(r[0]["t_wall"]) - float(r[0]["t_mono"]) for r in rows]
    assert abs(off[0] - off[1]) < 0.01


def test_label_tool_opens_each_camera_folder(tmp_path):
    folder, cams, _ = record_two(tmp_path, seconds=1.0)
    try:
        spec = importlib.util.spec_from_file_location("label_app", os.path.join(TOOLS, "label", "app.py"))
        app = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(app)
        Recording = app.Recording
    except Exception as e:      # tools/label is edited separately; this checks the format, not that tool
        pytest.skip(f"tools/label/app.py did not import: {e}")
    for i, fps in enumerate((30, 25)):
        rec = Recording(os.path.join(folder, f"cam{i}"))
        assert rec.fps == fps and rec.width == 64 and rec.height == 48
        assert rec.n == cams[i].n and rec.times[0] > 0.3
    cap = cv2.VideoCapture(os.path.join(folder, "cam1", "video.avi"))
    assert int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == cams[1].n


def test_notes_go_to_both_cameras(tmp_path):
    caps = [FakeCapture(30), FakeCapture(30)]
    out = {}

    def note_later():
        time.sleep(1.0)
        for c in out["cams"]:
            c.mark("trial 1 start", time.time())
    # mark through the cameras the session creates: patch CamRec.loop's caller by grabbing them
    orig = camrec.CamRec.open_outputs

    def grab(self, folder, meta):
        orig(self, folder, meta)
        out.setdefault("cams", []).append(self)
        if len(out["cams"]) == 2:
            threading.Thread(target=note_later, daemon=True).start()
    camrec.CamRec.open_outputs = grab
    try:
        folder, cams, _ = record_two(tmp_path, seconds=1.5, caps=caps)
    finally:
        camrec.CamRec.open_outputs = orig
    for i in range(2):
        with open(os.path.join(folder, f"cam{i}", "events.csv")) as f:
            ev = list(csv.DictReader(f))
        assert [e["text"] for e in ev] == ["trial 1 start"] and 10 < int(ev[0]["frame"]) < 45


def test_lost_frames_are_warned(tmp_path):
    camrec.CamRec.WINDOW_S = 1.0
    try:
        caps = [FakeCapture(30), FakeCapture(30, drop_every=3)]   # a third of cam1's frames lost
        folder, cams, lines = record_two(tmp_path, seconds=2.5, caps=caps)
    finally:
        camrec.CamRec.WINDOW_S = 5.0
    assert cams[0].frame_check()["ok"] and cams[0].missed == 0
    fc = cams[1].frame_check()
    assert not fc["ok"] and fc["missed_estimate"] > 10
    warn = [ln for ln in lines if ln.startswith("!")]
    assert any("cam1 (side)" in ln and "fps" in ln for ln in warn)
    assert not any("cam0" in ln for ln in warn)
    s = json.load(open(os.path.join(folder, "session.json")))
    assert s["cameras"][1]["frame_check"]["ok"] is False


def free_port_pair():
    for _ in range(50):
        s1, s2 = socket.socket(), socket.socket()
        try:
            s1.bind(("", 0))
            p = s1.getsockname()[1]
            s2.bind(("", p + 1))
            return p
        except OSError:
            continue
        finally:
            s1.close()
            s2.close()
    pytest.skip("no two free ports in a row")


def first_live_part(port):
    r = urllib.request.urlopen(f"http://127.0.0.1:{port}/live", timeout=5)
    head = b""
    while b"\r\n\r\n" not in head:
        head += r.read(1)
    r.close()
    return head.decode()


def test_each_camera_has_its_own_live_stream(tmp_path):
    port = free_port_pair()
    got = {}

    def fetch():
        time.sleep(0.8)
        for p in (port, port + 1):
            got[p] = first_live_part(p)
    t = threading.Thread(target=fetch)
    t.start()
    folder, cams, lines = record_two(tmp_path, seconds=2.0, port=port)
    t.join()
    assert "X-Frame" in got[port] and "X-Time" in got[port + 1]
    assert any("--stream" in ln and f":{port + 1})" in ln for ln in lines)
    s = json.load(open(os.path.join(folder, "session.json")))
    assert [c["port"] for c in s["cameras"]] == [port, port + 1]


def test_one_camera_layout_unchanged(tmp_path):
    lines = []
    folder, cams = camrec.run_session([FakeCapture(30)], ["0"], out=str(tmp_path), label="E1", seconds=1.0,
                                      fps=30, port=0, notes_port=0, stdin_notes=False, replays=[False],
                                      log=lines.append, disk_check_s=0)
    assert sorted(os.listdir(folder)) == ["events.csv", "frames.csv", "meta.json", "video.avi"]
    meta = json.load(open(os.path.join(folder, "meta.json")))
    assert "session" not in meta and meta["device"] == "0"
    r = read_frames(folder)
    assert list(r[0]) == ["frame", "t_wall", "t_mono", "cam_ms"]
    assert any(ln.startswith("Recording 64x48 to") for ln in lines)
    assert not any(ln.startswith("!") for ln in lines)


def test_disk_precheck_warns(tmp_path):
    lines = []
    camrec.disk_precheck(str(tmp_path), 2, 0, log=lines.append, free=3.0)    # 2 cams x 10 min want 4 GB
    assert lines[0].startswith("! LOW DISK SPACE") and "8 min fit" in lines[0]
    lines = []
    camrec.disk_precheck(str(tmp_path), 1, 0, log=lines.append, free=3.0)
    assert not lines[0].startswith("!")


def test_disk_watch_stops_recording(tmp_path):
    stop = threading.Event()
    lines = []
    levels = iter([5.0, 0.8, 0.8, 0.2])
    camrec.watch_disk(str(tmp_path), stop, 0.01, lines.append, free=lambda _: next(levels))
    assert stop.is_set()
    assert sum("LOW DISK SPACE" in ln for ln in lines) == 1 and any("stopping and saving" in ln for ln in lines)


def test_list_cameras_marks_the_ones_that_send_pictures():
    class Dead(FakeCapture):
        def read(self):
            return False, None
    lines = []
    found = camrec.list_cameras(run=lambda *a, **k: (_ for _ in ()).throw(OSError()), devices=[0, 1, 2, 3],
                                opener=lambda n: FakeCapture(30) if n in (0, 2) else (Dead() if n == 1 else None),
                                log=lines.append, seconds=0.2)
    assert [(n, d) for n, _, d, _ in found] == [(0, True), (1, False), (2, True), (3, False)]
    assert "no picture" in found[1][3] and "could not open" in found[3][3]
    assert any(ln.startswith("  USE ->") and " 2 " in ln for ln in lines)
    assert any("--device 0,2" in ln for ln in lines)


def test_tag_line():
    assert camrec.tag_line("  ! exposure: x", "cam1 (side): ") == "  ! cam1 (side): exposure: x"
    assert camrec.tag_line("Camera: exposure 80", "cam0: ") == "cam0: Camera: exposure 80"
    assert camrec.tag_line("!" * 10, "cam0: ") == "!" * 10
