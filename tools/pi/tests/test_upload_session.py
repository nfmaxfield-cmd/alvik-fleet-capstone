"""upload_session: what goes up, split into parts, skipping parts already up (no network: api is replaced)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import upload_session as U  # noqa: E402


def test_parts_and_skips(tmp_path, monkeypatch):
    sess = tmp_path / "logs" / "e02-20261025-090000"
    sess.mkdir(parents=True)
    (sess / "e02_log.txt").write_text("hello")
    rec = tmp_path / "rec" / "20261025-090010_E02" / "cam0"
    rec.mkdir(parents=True)
    (rec / "frames.csv").write_text("frame,t\n")
    big = rec / "video.avi"
    big.write_bytes(os.urandom(3000))
    monkeypatch.setattr(U, "PART_BYTES", 1000)
    monkeypatch.setattr(U, "SMALL_BYTES", 500)
    monkeypatch.setattr(U, "token", lambda: "t")
    sent = []

    def fake_api(method, url, tok, data=None, ctype=None, length=None):
        if method == "GET":
            return dict(upload_url="https://up/x{?name}", assets=[dict(name="cam0_video.avi.part-ab")])
        if data is not None and hasattr(data, "read"):
            b = b""
            while True:
                c = data.read(400)
                if not c:
                    break
                b += c
            assert len(b) == length
            sent.append((url.split("name=")[1], len(b)))
        return {}

    monkeypatch.setattr(U, "api", fake_api)
    monkeypatch.setattr(sys, "argv", ["x", "--recordings", str(tmp_path / "rec"), "--session", str(sess)])
    U.main()
    names = [n for n, _ in sent]
    assert names[0] == "logs-e02-20261025-090000.zip"
    assert names[1].startswith("rec-20261025-090010_E02-small")
    assert ("cam0_video.avi.part-aa", 1000) in sent and ("cam0_video.avi.part-ac", 1000) in sent
    assert not any(n == "cam0_video.avi.part-ab" for n in names)       # already up: skipped
