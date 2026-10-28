"""clean_uploaded.py (2026-10-28): deletes only what GitHub confirms (exact sizes), never anything else.

GitHub is faked: the "release" holds the sizes of zips made with upload_session's own zip_folder, as the upload does.
    python3 -m pytest tools/pi/tests/test_clean_uploaded.py -q
"""
import os
import sys
import urllib.error

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
TOOLS = os.path.dirname(PI)
for p in (PI, os.path.join(TOOLS, "fleet")):
    if p not in sys.path:
        sys.path.insert(0, p)

import clean_uploaded as C   # noqa: E402
import upload_session as US  # noqa: E402
import diskguard as DG       # noqa: E402

NAME = "20261027-213425-doe"


def _session(root, name=NAME, frames_on_drive=None):
    s = root / "logs" / name
    (s / "213808-loop000").mkdir(parents=True)
    (s / "drives.csv").write_text("t,robot\n1,Left\n" * 50)
    (s / "213808-loop000" / "coordinator.jsonl").write_text('{"event": "x"}\n' * 200)
    fd = (frames_on_drive / "fleet-frames" / name / "frames") if frames_on_drive else (s / "frames")
    (fd / "overhead" / "track").mkdir(parents=True)
    (fd / "overhead" / "track" / "214000_drive01.avi").write_bytes(os.urandom(20000))
    (fd / "overhead" / "events.csv").write_text("t,kind\n")
    if frames_on_drive:
        (s / "frames_where.txt").write_text(str(fd) + "\n")
    return s, fd


def _asset_sizes(tmp, sess, fd):
    """What the upload put on GitHub: logs zip (frames left out) and the frames' small-files zip."""
    lz = US.zip_folder(str(sess), str(tmp / "l.zip"), exclude=("frames",))
    rz = US.zip_folder(str(fd), str(tmp / "r.zip"), small_only=True)
    return {"logs-%s.zip" % sess.name: os.path.getsize(lz), "rec-frames-small.zip": os.path.getsize(rz)}


@pytest.fixture
def env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setenv("MIRLT_DRIVE", str(tmp_path / "no-drive"))
    monkeypatch.setattr(US, "STATE_DIR", str(tmp_path / "state"))
    monkeypatch.setattr(DG, "rec_out", lambda: str(tmp_path / "home" / "recordings"))
    monkeypatch.setattr(C, "running", lambda: [])
    monkeypatch.setattr(US, "token", lambda: None)
    return tmp_path


def _gh(rels):
    def get(url, tok):
        return [dict(tag_name=t, assets=[dict(name=n, size=s, state="uploaded") for n, s in a.items()])
                for t, a in rels.items()]
    return get


def _run(env, rels, *args):
    out = []
    code = C.main(["--logs", str(env / "logs")] + list(args), get=_gh(rels), say=out.append)
    return code, "\n".join(out)


def test_confirmed_session_listed_first_deleted_only_with_yes(env):
    sess, fd = _session(env)
    rels = {"doe-" + NAME: _asset_sizes(env, sess, fd)}
    code, out = _run(env, rels)
    assert code == 0 and "DELETE" in out and "Nothing was deleted" in out and sess.exists()
    code, out = _run(env, rels, "--yes")
    assert code == 0 and not sess.exists() and "deleted" in out


def test_unconfirmed_folders_are_never_deleted(env):
    sess, fd = _session(env)
    other, ofd = _session(env, "20261028-010000-doe")
    dry, _ = _session(env, "20261028-020000-doe-dryrun")
    good = _asset_sizes(env, sess, fd)
    rels = {"doe-" + NAME: good,
            # the other session's release has a frames zip of the wrong size (an upload cut short)
            "doe-20261028-010000-doe": {"logs-20261028-010000-doe.zip": _asset_sizes(env, other, ofd)[
                "logs-20261028-010000-doe.zip"], "rec-frames-small.zip": good["rec-frames-small.zip"] - 7}}
    code, out = _run(env, rels, "--yes")
    assert not sess.exists()
    assert other.exists() and ofd.exists() and dry.exists()
    assert "20261028-010000-doe: " in out and "rec-frames-small.zip" in out and "a dry run" in out


def test_no_release_or_changed_since_upload_is_kept(env):
    sess, fd = _session(env)
    rels = {"doe-" + NAME: _asset_sizes(env, sess, fd)}
    (sess / "notes-after-upload.txt").write_text("added after the upload")      # the folder changed
    code, out = _run(env, rels, "--yes")
    assert sess.exists() and "changed since the upload" in out
    code, out = _run(env, {}, "--yes")
    assert sess.exists() and "no release on GitHub holds it" in out


def test_frames_on_the_usb_drive_and_frames_only(env, monkeypatch):
    drive = env / "usb"
    drive.mkdir()
    monkeypatch.setenv("MIRLT_DRIVE", str(drive))
    sess, fd = _session(env, frames_on_drive=drive)
    rels = {"doe-%s-2" % NAME: _asset_sizes(env, sess, fd)}                      # --unique-tag's second try
    code, out = _run(env, rels, "--yes", "--frames-only")
    assert sess.exists() and not fd.exists() and "kept because of --frames-only" in out
    sess2, fd2 = _session(env, "20261028-030000-doe", frames_on_drive=drive)
    rels = {"doe-20261028-030000-doe": _asset_sizes(env, sess2, fd2)}
    code, out = _run(env, rels, "--yes")
    assert not sess2.exists() and not fd2.exists()


def test_github_unreachable_deletes_nothing(env):
    sess, fd = _session(env)

    def down(url, tok):
        raise urllib.error.URLError("no WiFi")
    out = []
    code = C.main(["--logs", str(env / "logs"), "--yes"], get=down, say=out.append)
    assert code == 2 and sess.exists() and "nothing is deleted" in "\n".join(out)


def test_a_run_going_deletes_nothing(env, monkeypatch):
    sess, fd = _session(env)
    rels = {"doe-" + NAME: _asset_sizes(env, sess, fd)}
    monkeypatch.setattr(C, "running", lambda: ["doe"])
    code, out = _run(env, rels, "--yes")
    assert code == 1 and sess.exists() and "a run is going" in out


def test_upload_copies_deleted_only_when_github_has_that_exact_size(env):
    tmpd = env / "home" / ".upload-tmp"
    tmpd.mkdir(parents=True)
    (tmpd / "rec-frames-small.zip").write_bytes(b"z" * 1000)
    (tmpd / "logs-x.zip").write_bytes(b"z" * 500)
    rels = {"doe-x": {"rec-frames-small.zip": 1000, "logs-x.zip": 499}}
    code, out = _run(env, rels, "--yes")
    assert not (tmpd / "rec-frames-small.zip").exists() and (tmpd / "logs-x.zip").exists()


def test_zip_size_in_memory_matches_the_real_zip(env):
    sess, fd = _session(env)
    real = US.zip_folder(str(sess), str(env / "real.zip"), exclude=("frames",))
    assert C.zip_size(str(sess), exclude=("frames",)) == os.path.getsize(real)


def test_tags_for_names():
    rels = {"doe-%s" % NAME: {}, "doe-%s-2" % NAME: {}, "mission-20261028-100000-mission": {},
            "e02-session-20261025-2114": {}, "doe-20261027-213425-doe-dryrun": {}}
    assert C.tags_for(NAME, rels) == ["doe-%s" % NAME, "doe-%s-2" % NAME]
    assert C.tags_for("e02-20261025-2114", rels) == ["e02-session-20261025-2114"]
