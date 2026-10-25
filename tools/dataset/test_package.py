"""tools/dataset/package.py: copies, scrubs and describes a session."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import package as P  # noqa: E402


def _session(tmp):
    s = tmp / "e02-20261024-193526"
    (s / "robot-logs-1").mkdir(parents=True)
    (s / "e02_log.txt").write_text("[19:35] folder /home/st1maxfini/x, robot 192.168.1.250 and 192.168.1.253\n")
    (s / "track.csv").write_text("t,robot,x_cm,y_cm\n1.0,Right,20.3,77.8\n2.0,Right,20.4,77.9\n")
    (s / "robot-logs-1" / "leg.csv").write_text("robot,t_recv,run\nRight,1.0,3\n")
    (s / "sensor_trust.json").write_text("{}")
    rec = tmp / "rec"
    rec.mkdir()
    (rec / "frames.csv").write_text("frame,t_wall\n0,1.0\n")
    (rec / "meta.json").write_text('{"host": "st1maxfini"}')
    (rec / "video.avi").write_bytes(b"x" * 10)
    return s, rec


def test_package_copies_scrubs_and_describes(tmp_path):
    s, rec = _session(tmp_path)
    out = tmp_path / "out"
    assert P.main([str(s), "--out", str(out), "--recording", str(rec)]) == 0
    log = (out / s.name / "e02_log.txt").read_text()
    assert "/home/st1maxfini" not in log and "~/x" in log
    assert "192.168" not in log and "robot-1" in log and "robot-2" in log
    assert "st1maxfini" not in (out / s.name / "recording" / "meta.json").read_text()
    assert not (out / s.name / "sensor_trust.json").exists()             # learned state is left out
    assert not (out / s.name / "recording" / "video.avi").exists()       # no video unless asked
    assert (out / s.name / "recording" / "frames.csv").exists()
    m = json.loads((out / "manifest.json").read_text())
    tr = next(f for f in m["files"] if f["path"].endswith("track.csv"))
    assert tr["rows"] == 2 and tr["columns"] == ["t", "robot", "x_cm", "y_cm"] and len(tr["sha256"]) == 64
    card = (out / "DATASET.md").read_text()
    assert "track.csv" in card and "`x_cm`" in card
    # the session folder itself is untouched
    assert "/home/st1maxfini" in (s / "e02_log.txt").read_text()


def test_video_only_when_asked(tmp_path):
    s, rec = _session(tmp_path)
    out = tmp_path / "out"
    P.main([str(s), "--out", str(out), "--recording", str(rec), "--video"])
    assert (out / s.name / "recording" / "video.avi").exists()
