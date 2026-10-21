"""Stage 2, interface (lane U): the tracker's flags first in "Most useful next", Investigate's
preview and redo, "Reject here" (one frame, the spot remembered), accept-without-looking by frame
list (for Z / Shift+Z), the clip on key C, the page's keys and messages, and the routes."""
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LABEL = os.path.dirname(HERE)
sys.path.insert(0, LABEL)
if HERE not in sys.path:
    sys.path.append(HERE)

import app          # noqa: E402
from test_handoff_label import still_recording, fresh, open_with_frames, rows, N  # noqa: E402

PAGE = open(os.path.join(LABEL, "page.html"), encoding="utf-8").read()
SCRIPT = PAGE[PAGE.index("<script>") + 8:PAGE.rindex("</script>")]


@pytest.fixture(scope="module")
def still(tmp_path_factory):
    d = still_recording(str(tmp_path_factory.mktemp("still_u") / "rec"))
    app.Session(d, log=lambda *a: None, classes=["box", "unknown"])
    return d


def guess(obj, x, y, cls="box", score=0.8, **flags):
    g = dict(obj=obj, cls=cls, x=float(x), y=float(y), heading_deg=None, heading_known=False, heading_guess=None,
             source="motion", score=score)
    g.update(flags)
    return g


def _label(s, f, x, y, cls="box", status="corrected"):
    return s.save_frame(f, [dict(obj=None, cls=cls, x=x, y=y, source="user", status=status)], "reviewed", "spot")


# ---------------------------------------------------------------- 4. the tracker's flags first
def test_flagged_frames_come_first_with_a_plain_reason(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20, 30, 40, 50])
    s.guesses = {10: [guess(1, 50, 50, score=0.2)], 20: [guess(2, 60, 60, score=0.9)],
                 30: [guess(3, 70, 70, score=0.95, chain_broken=True)],
                 40: [guess(4, 80, 80, score=0.99, split_unresolved=True), guess(5, 90, 80, score=0.99)],
                 50: [guess(6, 90, 90, score=0.5)]}
    s._update_conf()
    q = s.queue("next")
    assert [x["f"] for x in q] == [40, 30, 10, 50, 20]            # unresolved split, broken chain, then least confident
    assert q[0]["flag"] == "split_unresolved" and q[1]["flag"] == "chain_broken" and "flag" not in q[2]
    v = s.frame_view(40)
    assert v["flags"] == [dict(key="split_unresolved", text="two lookalikes crossed here: which is which?")]
    assert "still there" in s.frame_view(30)["flags"][0]["text"]
    s.guesses[50][0]["confirm_due"] = True                         # the chain asks for a fresh confirmation
    assert [x["f"] for x in s.queue("next")][:3] == [40, 30, 50] and s.frame_view(50)["flags"][0]["key"] == "confirm_due"
    s.guesses[50][0]["confirm_due"] = False
    assert s.stats()["flagged_left"] == 2
    # a finished frame is not "to look at first" any more
    s.save_frame(40, s.frame_view(40)["objects"], "reviewed", "spot")
    assert s.stats()["flagged_left"] == 1 and s.queue("next")[0]["f"] == 30
    # the summary written to review.json keeps its old keys
    assert "flagged_left" not in json.load(open(s.review_path))["summary"]


def test_flags_from_the_trackers_own_function(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20, 30])
    s.guesses = {10: [guess(1, 50, 50, score=0.2)], 20: [guess(2, 50, 50, score=0.9)], 30: [guess(3, 50, 50, score=0.9)]}
    s._update_conf()
    s.run.frame_flags = lambda f: ["chain_broken"] if f == 30 else (["check_due"] if f == 20 else [])
    q = s.queue("next")
    assert [x["f"] for x in q] == [30, 20, 10]                    # known flags before other flags
    assert s.frame_view(20)["flags"][0]["text"] == app.OTHER_FLAG_TEXT

    def broken(f):
        raise RuntimeError("tracker trouble")
    s.run.frame_flags = broken
    assert [x["f"] for x in s.queue("next")] == [10, 20, 30]       # never stops the list


# ---------------------------------------------------------------- 2. change all: preview, undo, redo
def test_change_all_preview_changes_nothing_and_undo_gives_a_redo(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, range(0, N, 10))
    for f in (20, 40, 60):
        _label(s, f, 101.0, 99.0)
    items = [dict(frame=l["frame"], obj=l["obj"]) for l in s.labels_at(100, 100, s.spot_radius())]
    before = open(s.labels_path).read()
    p = s.change_at(items, "reject", preview=True)
    assert p["preview"] and p["changed"] == 3 and p["frames"] == [20, 40, 60]
    assert p["items"][0]["status"] == "corrected" and p["items"][0]["new_status"] == "rejected"
    assert open(s.labels_path).read() == before and not s.bulk_undo           # nothing changed, nothing to undo
    assert s.change_at(items, "type", "box", preview=True)["changed"] == 0     # already boxes
    r = s.change_at(items, "reject")
    after = open(s.labels_path).read()
    u = s.undo_change(r["token"])
    assert u["undone"] == 3 and u["token"]
    assert open(s.labels_path).read().split("\n")[1:] == before.split("\n")[1:]
    redo = s.undo_change(u["token"])                                          # Shift+Z
    assert redo["undone"] == 3 and redo["token"]
    assert [x["status"] for x in rows(s) if x["frame"] == "40" and x["cls"] != "none"] == ["rejected"]
    assert open(s.labels_path).read().split("\n")[1:] == after.split("\n")[1:]
    s.undo_change(redo["token"])                                              # Z again
    assert open(s.labels_path).read().split("\n")[1:] == before.split("\n")[1:]


# ---------------------------------------------------------------- accept without looking, by frame list
def test_auto_accept_and_take_back_by_frame_list(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20, 30, 40])
    s.guesses = {f: [guess(f, 50, 50, score=0.95)] for f in (10, 20, 30, 40)}
    s._update_conf()
    got = s.auto_accept_frames()
    assert got == [10, 20, 30, 40]
    assert s.undo_auto_frames([20, 30]) == [20, 30]                # Z takes back this accept only
    assert s.frame_state(20) == "pending" and s.frame_state(10) == "auto"
    assert s.auto_accept_frames([20, 30]) == [20, 30]              # Shift+Z puts them back
    assert s.undo_auto() == 4                                       # the button still takes back all of them


# ---------------------------------------------------------------- 3. Reject here
class FakeMemory:
    def __init__(self):
        self.seen, self.gone = [], []

    def remember(self, **kw):
        self.seen.append(kw)

    def forget(self, **kw):
        self.gone.append(kw)


def test_reject_here_on_one_frame_and_the_spot_is_remembered(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20, 30])
    mem = FakeMemory()
    s.run.remember_rejected, s.run.forget_rejected = mem.remember, mem.forget
    stain = [guess(7, 100, 100, score=0.6), guess(8, 250, 200, score=0.9)]
    s.guesses = {10: [dict(g) for g in stain], 20: [dict(g) for g in stain], 30: [dict(g) for g in stain]}
    s._update_conf()
    p = s.reject_here(20, 101, 99, preview=True)
    assert [i["obj"] for i in p["items"]] == [7] and 20 not in s.frames and not s.rejected_spots()
    r = s.reject_here(20, 101, 99)
    assert r["rejected"] == [7] and r["tracker_told"] and r["token"]
    st = s.frames[20]
    assert st["state"] == "edited" and {o["obj"]: o["status"] for o in st["objects"]} == {7: "rejected", 8: "pending"}
    assert 10 not in s.frames and 30 not in s.frames                         # never other frames
    assert mem.seen == [dict(frame=20, x=101.0, y=99.0, r=s.spot_radius())]
    saved = json.load(open(s.review_path))
    assert saved["rejected_spots"][0]["frame"] == 20 and "Reject here" in saved["help"]
    # Z: the frame and the tracker's memory as before
    u = s.undo_change(r["token"])
    assert 20 not in s.frames and s.rejected_spots() == [] and len(mem.gone) == 1
    # Shift+Z: both again
    again = s.undo_change(u["token"])
    assert s.frames[20]["objects"][0]["status"] == "rejected" and len(s.rejected_spots()) == 1 and len(mem.seen) == 2
    s.undo_change(again["token"])
    assert s.rejected_spots() == []


def test_reject_here_keeps_your_own_object_and_finished_frames_stay_finished(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20])
    s.save_frame(10, [dict(obj=3, cls="box", x=100, y=100, source="motion", status="confirmed", score=0.7),
                      dict(obj=None, cls="box", x=104, y=100, source="user", status="corrected")], "reviewed", "spot")
    r = s.reject_here(10, 100, 100)
    assert r["rejected"] == [3] and len(r["yours"]) == 1
    st = s.frames[10]
    assert st["state"] == "reviewed" and [o["status"] for o in st["objects"]] == ["rejected", "corrected"]
    # nothing guessed at the spot: no label changes, the empty spot is still remembered
    r2 = s.reject_here(20, 300, 30)
    assert r2["rejected"] == [] and 20 not in s.frames and len(s.rejected_spots()) == 2
    with pytest.raises(ValueError):
        s.reject_here(11, 100, 100)                                          # not a picked frame


def test_reject_here_on_an_auto_frame_makes_it_a_persons_change(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20])
    s.guesses = {10: [guess(1, 100, 100, score=0.95)], 20: [guess(2, 100, 100, score=0.95)]}
    s._update_conf()
    s.auto_accept_frames()
    s.reject_here(10, 100, 100)
    assert s.frames[10]["state"] == "reviewed" and s.frames[10]["how"] == "change_all" and s.frames[10]["empty"]
    assert s.undo_auto() == 1 and 10 in s.frames                          # undo auto-accept leaves it alone


def test_rejected_spots_are_given_to_the_tracker_when_it_opens_again(tmp_path, still, monkeypatch):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20])
    s.reject_here(10, 120, 80)
    seen = []
    monkeypatch.setattr(app.tracker.TrackRun, "remember_rejected", lambda self, **kw: seen.append(kw), raising=False)
    app.Session(d, log=lambda *a: None, classes=["box", "unknown"])
    assert seen and seen[0]["frame"] == 10 and seen[0]["x"] == 120.0


def test_without_the_trackers_memory_reject_here_still_works(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10])
    s.run.remember_rejected = None                  # a tracker without the memory
    s.guesses = {10: [guess(1, 100, 100)]}
    r = s.reject_here(10, 100, 100)
    assert r["rejected"] == [1] and not r["tracker_told"]


# ---------------------------------------------------------------- 5. the clip
def test_clip_about_a_second_either_side(tmp_path, still):
    s = app.Session(still, log=lambda *a: None, classes=["box", "unknown"])
    c = s.clip(100)
    dts = [x["dt"] for x in c["frames"]]
    assert 0.0 in dts and min(dts) == -1.0 and max(dts) == 1.0              # 10 frames per second
    assert len(dts) == 21 and c["interval_ms"] == 100 and c["before_s"] == 1.0
    assert all(x["picture"].startswith("data:image/jpeg;base64,") for x in c["frames"])
    first = s.clip(0)
    assert first["frames"][0]["frame"] == 0 and first["before_s"] == 0.0     # the start of the recording
    few = s.clip(100, max_frames=7)
    assert len(few["frames"]) <= 8 and 100 in [x["frame"] for x in few["frames"]]
    with pytest.raises(ValueError):
        s.clip(N + 5)


# ---------------------------------------------------------------- the routes
def _get(url):
    with urllib.request.urlopen(url, timeout=60) as r:
        return json.loads(r.read())


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read())


def test_routes(tmp_path, still):
    d = fresh(still, str(tmp_path / "rec"))
    s = open_with_frames(d, [10, 20, 30])
    s.guesses = {f: [guess(f, 100, 100, score=0.95)] for f in (10, 20, 30)}
    s._update_conf()
    server, url = app.serve(s, 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        assert len(_get(url + "api/clip?f=20")["frames"]) == 21
        a = _post(url + "api/auto_accept", {})
        assert a["accepted"] == 3 and a["frames"] == [10, 20, 30]
        u = _post(url + "api/auto_undo", dict(frames=[30]))
        assert u["frames"] == [30] and s.frame_state(30) == "pending"
        p = _post(url + "api/investigate/reject_here", dict(frame=30, x=100, y=100, preview=True))
        assert p["preview"] and [i["obj"] for i in p["items"]] == [30] and 30 not in s.frames
        r = _post(url + "api/investigate/reject_here", dict(frame=30, x=100, y=100))
        assert r["rejected"] == [30]
        back = _post(url + "api/investigate/undo", dict(token=r["token"]))
        assert back["token"] and 30 not in s.frames
        q = _get(url + "api/queue?mode=next")
        assert "flagged_left" in q["stats"]
        assert "flags" in _get(url + "api/frame?f=10")
    finally:
        server.shutdown()
        server.server_close()


# ---------------------------------------------------------------- the page
def test_page_keys_and_parts():
    keys = re.findall(r"<kbd>(.*?)</kbd>", PAGE)
    for k in ("C", "Q", "Z", "Shift+Z"):
        assert k in keys, k
    assert 'id="invignore"' not in PAGE and "ignoreSpot" not in PAGE and "/api/studio/ignore_spot" not in PAGE
    for word in ("1. What we found", "2. Look around", "3. Change things", "Reject here", "Q brings you back here",
                 "Yes, change it", "Cancel: change nothing", "Old ignore spots"):
        assert word in PAGE, word
    assert 'id="flagnote"' in PAGE and "to look at first" in PAGE
    assert "in real life" not in PAGE.lower() and "alvik" not in PAGE.lower()


def test_every_message_says_how_to_take_it_back():
    """The message after each action that changes saved work names the key or button that takes it back."""
    for marker, back in (("Accepted frame ", "Z takes the accept back"), ("Marked frame ", "Z takes it back"),
                         ("is back to the guesses", "Z brings them back"), ("Accepted the guesses on ", "Z or Undo auto-accept"),
                         ("Work area saved", "Z puts the old one back"), ("Jumped to frame", "press Q to go back"),
                         ("Undid: ", "Shift+Z puts it back"), ("Redid: ", "Z takes it back again")):
        i = SCRIPT.index(marker)
        assert back in SCRIPT[max(0, i - 900):i + 900], marker
    # change() always ends its message with the take-back part
    assert '(back || "Z takes it back.")' in SCRIPT
    # the old messages without a take-back part are gone
    for old in ('toast("Undone (frame', 'toast("Object types saved.")', 'toast("Training started.")',
                'toast("Ignore spot removed.")', 'toast("No longer unsure.")'):
        assert old not in SCRIPT, old


def test_page_script_parses():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    p = os.path.join(HERE, "__pycache__")
    os.makedirs(p, exist_ok=True)
    f = os.path.join(p, "page_check_u.js")
    with open(f, "w", encoding="utf-8") as fh:
        fh.write(SCRIPT)
    r = subprocess.run([node, "--check", f], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
