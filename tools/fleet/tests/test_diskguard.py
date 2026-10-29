"""Free space (2026-10-28, after the 21:34 DOE loop filled the Pi's card at 22:03): where frames go, the guard's floors,
the DOE loop ending cleanly at the card's floor, frame saving stopping at the frames' floor, run_plan not crashing on a
full disk. The free space is faked (diskguard.Guard(fake=...)); nothing here fills a disk.

    python3 -m pytest tools/fleet/tests/test_diskguard.py -q
"""
import csv
import os
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, os.path.join(TOOLS, "sim"), TOOLS, os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.append(p)

import diskguard as DG      # noqa: E402
import doe_loop as D        # noqa: E402


def test_floors_frames_first_then_the_card(tmp_path):
    drive = tmp_path / "drive"
    drive.mkdir()
    g = DG.Guard(card=str(tmp_path), frames_dir=str(drive / "f"), fake=dict(card=10.0, drive=50.0))
    g.frames_on_card = False                      # (tmp_path and "drive" are one filesystem here)
    s = g.state()
    assert s["frames_ok"] and not s["stop"]
    g.fake["drive"] = 4.9                         # under the drive's 5 GB: frames stop, the run goes on
    s = g.state()
    assert not s["frames_ok"] and not s["stop"] and "USB drive" in s["why_frames"]
    g.fake["drive"] = 6.0                         # 6 GB free but 1.5 GB of frames to copy for the upload: 4.5 left
    assert not g.state(frames_bytes=1.5e9)["frames_ok"]
    assert g.state(frames_bytes=0.5e9)["frames_ok"]
    g.fake["card"] = 0.9                          # under the card's 1 GB: the run ends
    s = g.state()
    assert s["stop"] and "0.90 GB" in s["why"]


def test_frames_on_the_card_stop_at_2gb(tmp_path):
    g = DG.Guard(card=str(tmp_path), frames_dir=str(tmp_path / "s" / "frames"), fake=dict(card=1.9))
    s = g.state()
    assert g.frames_on_card and not s["frames_ok"] and not s["stop"]
    assert "ON THE PI'S CARD" in g.words()


def test_frames_go_to_the_drive_and_upload_finds_them(tmp_path, monkeypatch):
    drive = tmp_path / "usb"
    drive.mkdir()
    sess = tmp_path / "logs" / "20261028-090000-doe"
    monkeypatch.setenv("MIRLT_DRIVE", str(drive))
    fd, on_drive = DG.frames_dir_for(str(sess))
    assert on_drive and fd == str(drive / "fleet-frames" / sess.name / "frames") and os.path.isdir(fd)
    assert DG.where_frames(str(sess)) == fd       # doe-finish: diskguard.py where <session>
    assert os.path.basename(fd) == "frames"       # the upload's asset stays rec-frames-small.zip
    monkeypatch.setenv("MIRLT_DRIVE", str(tmp_path / "not-there"))
    sess2 = tmp_path / "logs" / "20261028-100000-doe"
    fd2, on2 = DG.frames_dir_for(str(sess2))
    assert not on2 and fd2 == str(sess2 / "frames") and DG.where_frames(str(sess2)) == fd2


def test_frames_stop_saving_under_the_floor_and_log_free_space(tmp_path):
    g = DG.Guard(card=str(tmp_path), frames_dir=str(tmp_path / "frames"), fake=dict(card=3.0))
    fr = D.Frames(str(tmp_path), [], frames_dir=str(tmp_path / "frames"), guard=g)
    cam = types.SimpleNamespace(stopped=False, saved_bytes=lambda: 0.0, cam="overhead", n_in=0, n_kept=0, mb=0.0,
                                saver=types.SimpleNamespace(summary=lambda: {}), close=lambda: None,
                                perf_row=lambda now: (0.0, None, None))
    cam.stop_saving = lambda: setattr(cam, "stopped", True)
    fr.cams = [cam]
    assert fr.check_space()["frames_ok"] and fr.stopped is None
    g.fake["card"] = 1.5                          # frames on the card: under 2 GB
    st = fr.check_space()
    assert not st["frames_ok"] and cam.stopped and fr.stopped
    fr.event("hold", "x")                         # no clips after the stop (no saver touched)
    assert "frame saving stopped" in fr.words()
    fr.close()
    head = open(tmp_path / "perf.csv").readline().strip().split(",")
    assert head[-2:] == ["card_free_gb", "frames_free_gb"] and head[:5] == ["t", "fps_handled", "age_mean_s",
                                                                             "age_max_s", "side_fps"]


def _bare_loop(tmp_path, sim_free):
    L = D.DoeLoop.__new__(D.DoeLoop)
    L.a = types.SimpleNamespace(dry=True, hours=1.0, no_frames=True, sim_free=sim_free, robots="")
    L.dir = str(tmp_path)
    L.state_path = os.path.join(L.dir, "state.json")
    L.st = {"passed": {"L0": 1, "L0:Left": 1, "L1:Left": 1, "L0:Right": 1, "L1:Right": 1}, "results": {},
            "robots": {"Left": {"x": 60, "y": 60, "h": 0}, "Right": {"x": 30, "y": 30, "h": 0}}, "kept_out": {},
            "doe": {"i": {}, "boot": {}, "uptime": {}, "loops": 0}}
    L.got, L.files, L.frames, L.keeper, L.eye, L.bench, L.cpu = {}, {}, None, None, None, None, None
    L.stream, L.side_stream = "", None
    L.sim_events, L.t_start = [], __import__("time").time()
    L.guard, L.frames_dir, L.sim_fall, L.ended, L.row_err = None, None, 0.0, None, False
    L.moved = []
    L.check = lambda sub, names, why: names
    L.need_bench = lambda sub: None
    L.find_all = lambda sub: None
    L.drive = lambda *a: L.moved.append(a) or "done"
    L.turn = lambda *a: L.moved.append(a) or True
    L.ensure_room = lambda *a: True        # (room making: tests in test_doe_loop.py)
    L.start_logs_real = L.start_logs
    return L


def test_doe_loop_ends_cleanly_at_the_card_floor(tmp_path):
    # 1.9 GB, less 0.5 before each loop: loop 0 runs (1.4 GB), loop 1 would start at 0.9 GB: the loop ends there
    L = _bare_loop(tmp_path, "card=1.9,drive=50,fall=0.5")
    assert L.run() is True
    assert L.ended == "disk" and len(L.moved) == 2          # loop 0's turn check and drive only
    L.finish()
    ev = list(csv.DictReader(open(tmp_path / "events.csv")))
    assert ev[-1]["kind"] == "disk" and "ends before loop 1" in ev[-1]["text"]
    assert "disk Pi" in open(tmp_path / "summary.md").read()


def test_doe_loop_not_started_on_a_full_card(tmp_path):
    L = _bare_loop(tmp_path, "card=0.4,drive=50")
    assert L.run() is False
    assert L.ended == "disk" and not L.moved


def test_run_plan_log_on_a_full_disk_ends_the_run_instead_of_crashing():
    import run_plan as RP

    class Full:
        def write(self, s):
            raise OSError(28, "No space left on device")

        def flush(self):
            pass
    c = types.SimpleNamespace(log_f=Full())
    RP.Coordinator.log(c, "step_report", robot="Left")      # (22:03: this raised and killed run_plan mid-drive)
    assert "disk is full" in c.abort


def test_space_and_preflight_print(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("MIRLT_DRIVE", str(tmp_path))
    (tmp_path / "fleet-frames" / "20261027-213425-doe" / "frames").mkdir(parents=True)
    (tmp_path / "fleet-frames" / "20261027-213425-doe" / "frames" / "a.avi").write_bytes(b"x" * 1000)
    assert DG.space(k=3) == 0
    out = capsys.readouterr().out
    assert "The Pi's card" in out and "20261027-213425-doe" in out
    monkeypatch.setenv("MIRLT_FAKE_FREE", "card=0.5,drive=50")
    assert DG.preflight() == 1
    assert "NOT STARTED" in capsys.readouterr().out
    monkeypatch.setenv("MIRLT_FAKE_FREE", "card=20,drive=50")
    assert DG.preflight() == 0
    assert "Frames go to" in capsys.readouterr().out


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
