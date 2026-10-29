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


# ---------------------------------------------------------------------------- 2026-10-29: zips over 1.9 GB, retries

def _frames_case(tmp_path, monkeypatch, assets, fail_first=None):
    """A DOE loop's logs and frames folder; the frames zip is bigger than PART_BYTES. Returns (sent, deleted, zsize)."""
    import urllib.error
    sess = tmp_path / "logs" / "20261029-074342-doe"
    sess.mkdir(parents=True)
    (sess / "drives.csv").write_text("robot\n")
    fr = tmp_path / "usb" / "fleet-frames" / "20261029-074342-doe" / "frames"
    fr.mkdir(parents=True)
    for k in range(6):
        (fr / ("f%03d.jpg" % k)).write_bytes(os.urandom(700))
    monkeypatch.setattr(U, "PART_BYTES", 1500)
    monkeypatch.setattr(U, "SMALL_BYTES", 10000)
    monkeypatch.setattr(U, "token", lambda: "t")
    monkeypatch.setattr(U.time, "sleep", lambda s: None)
    monkeypatch.setattr(U, "drop_copy", lambda z: None)          # keep the zip to measure it
    sent, deleted, posts = [], [], []
    state = {"assets": list(assets)}

    def fake_api(method, url, tok, data=None, ctype=None, length=None):
        if method == "GET":
            return dict(url="https://api/rel/1", upload_url="https://up/x{?name}", assets=state["assets"])
        if method == "DELETE":
            deleted.append(url)
            state["assets"] = [a for a in state["assets"] if a.get("url") != url]
            return {}
        b = b""
        while True:
            c = data.read(400)
            if not c:
                break
            b += c
        name = url.split("name=")[1]
        posts.append(name)
        if fail_first and posts.count(name) == 1 and name == fail_first:
            state["assets"].append(dict(name=name, state="uploaded", size=len(b), url="https://api/a/" + name))
            raise urllib.error.HTTPError(url, 422, "Validation Failed", {}, None)   # the answer lost, the part up
        sent.append((name, len(b)))
        return {}

    monkeypatch.setattr(U, "api", fake_api)
    monkeypatch.setattr(sys, "argv", ["x", "--recordings", str(tmp_path / "usb" / "recordings"), "--session", str(sess),
                                      "--tag", "doe-20261029-074342-doe", "--no-logs", "--recording-dir", str(fr)])
    (tmp_path / "usb" / "recordings").mkdir(parents=True)
    U.main()
    z = tmp_path / "usb" / ".upload-tmp" / "rec-frames-small.zip"
    return sent, deleted, posts, os.path.getsize(z)


def test_a_zip_over_the_part_size_goes_up_in_parts(tmp_path, monkeypatch):
    sent, deleted, posts, zs = _frames_case(tmp_path, monkeypatch, [])
    names = [n for n, _ in sent]
    assert "rec-frames-small.zip" not in names                              # never the whole (GitHub: HTTP 422)
    assert names[0] == "rec-frames-small.zip.part-aa" and len(names) == -(-zs // 1500)
    assert sum(k for _, k in sent) == zs and all(k <= 1500 for _, k in sent)


def test_a_retry_skips_parts_already_up_and_resends_a_broken_one(tmp_path, monkeypatch):
    # the first try got part-aa up whole; part-ab was cut off half-way (GitHub keeps it as 'starter')
    _, _, _, zs = _frames_case(tmp_path / "first", monkeypatch, [])
    assets = [dict(name="rec-frames-small.zip.part-aa", state="uploaded", size=1500, url="https://api/a/aa"),
              dict(name="rec-frames-small.zip.part-ab", state="starter", size=0, url="https://api/a/ab")]
    sent, deleted, posts, zs2 = _frames_case(tmp_path / "again", monkeypatch, assets)
    names = [n for n, _ in sent]
    assert "rec-frames-small.zip.part-aa" not in names                      # whole and the right size: not resent
    assert deleted == ["https://api/a/ab"] and "rec-frames-small.zip.part-ab" in names
    assert sum(k for _, k in sent) == zs2 - 1500


def test_a_422_after_the_part_got_there_is_not_sent_again(tmp_path, monkeypatch):
    sent, deleted, posts, zs = _frames_case(tmp_path, monkeypatch, [], fail_first="rec-frames-small.zip.part-aa")
    assert posts.count("rec-frames-small.zip.part-aa") == 1 and not deleted
    assert "rec-frames-small.zip.part-ab" in [n for n, _ in sent]


def test_clean_uploaded_counts_a_zip_sent_in_parts():
    import clean_uploaded as CU
    a = {"rec-frames-small.zip.part-aa": 1500, "rec-frames-small.zip.part-ab": 700, "logs-x.zip": 10}
    assert CU.asset_size(a, "rec-frames-small.zip") == 2200
    assert CU.asset_size(a, "logs-x.zip") == 10 and CU.asset_size(a, "nothing.zip") is None
