"""The tracker process build (2026-10-25 evening, MIRLT): the faster matcher and floor update give the same answers,
every parked robot is watched, the shared picture ring, the frame saver, the CPU log, the process end to end, and the
fusion-steering rule. Real footage (if present): E02_TRACKER_CLIP=<folder with clip.bin + clip_index.csv>."""
import csv
import math
import os
import tempfile
import time

import cv2
import numpy as np
import pytest

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02_alltrack as AT
import e02_camera as CAM
import e02_cpulog as CL
import e02_framesaver as FS
import e02_fusesteer as FU
import e02_lock as LK
import e02_shmframes as SF


# =============================================================================== a small synthetic world
class Robot:
    def __init__(self, x, y, th=0.0, name=""):
        self.x, self.y, self.th, self.name = x, y, th, name
        self.lights = (0.0, 0.0)


class World:
    def __init__(self):
        import e02_plan as PL
        self.board = PL.Board(xboard_side=1, yboard_side=1)
        self.robots = []

    @staticmethod
    def view(r):
        return r.x, r.y, r.th, 0.0


@pytest.fixture(scope="module")
def scene():
    import argparse
    import e02_sim
    w = World()
    sc = e02_sim.SimCamera(w, argparse.Namespace(px_per_cm=2.34, tilt=3.0, yaw=1.5, seed=3, fps=15, pi_offset=0.0))
    d = tempfile.mkdtemp()
    cam = CAM.load_camera(sc.true_camera_model(os.path.join(d, "true.json")))
    return w, sc, cam


def grey(sc):
    img = sc.render()
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img


# =============================================================================== the faster lock-on matcher
def test_batch_match_equals_opencv_on_random_and_flat_windows():
    rng = np.random.default_rng(1)
    for n, w in ((33, 61), (21, 45), (41, 71)):
        win = (rng.random((w, w)) * 200).astype(np.float32)
        win[: w // 3, : w // 3] = 50.0                         # a flat part (no contrast)
        tps = [(rng.random((n, n)) * 50).astype(np.float32) for _ in range(5)]
        got = AT.ccoeff_normed_batch(win, tps)
        for j, tp in enumerate(tps):
            ref = np.nan_to_num(cv2.matchTemplate(win, tp, cv2.TM_CCOEFF_NORMED), nan=0.0, posinf=0.0, neginf=0.0)
            assert got[j].shape == ref.shape
            assert np.abs(got[j] - ref).max() < 1e-4


def test_batch_match_flat_window_is_zero_as_opencv():
    win = np.full((40, 40), 80.0, np.float32)
    tp = np.random.default_rng(2).random((15, 15)).astype(np.float32)
    ref = np.nan_to_num(cv2.matchTemplate(win, tp, cv2.TM_CCOEFF_NORMED), nan=0.0)
    got = AT.ccoeff_normed_batch(win, [tp])[0]
    assert np.allclose(got, ref, atol=1e-5)


def test_fast_lock_best_equals_the_old_one(scene):
    w, sc, cam = scene
    w.robots[:] = [Robot(50.0, 60.0, 20.0, "A")]
    img = grey(sc)
    rpx = CAM.ROBOT_CM * cam.px_per_cm(61, 61, CAM.ROBOT_TOP_CM)
    old = LK.LockTracker(cam, rpx, CAM.ROBOT_TOP_CM)
    new = AT.FastLockTracker(cam, rpx, CAM.ROBOT_TOP_CM)
    assert old.start(img, 50.0, 60.0, 0.0) and new.start(img, 50.0, 60.0, 0.0)
    feat = old.features(img)
    u, v = old._px(50.0, 60.0)
    for du, dv, near in ((0, 0, None), (3, -2, 0.0), (-4, 5, 170.0), (10, 10, 90.0)):
        a = old._best(feat, u + du, v + dv, 14, old.tpls, old.mask, near_angle=near)
        b = new._best(feat, u + du, v + dv, 14, new.tpls, new.mask, near_angle=near)
        assert a[1:] == b[1:] and abs(a[0] - b[0]) < 1e-4


# =============================================================================== same answers, every robot watched
def run_both(scene, steps, watch=True):
    """The same pictures through MotionTracker and AllTracker; steps: list of (robots' (x, y) dict, active name).
    The pictures are drawn once (the renderer adds fresh noise to every picture)."""
    w, sc, cam = scene
    w.robots[:] = []
    bgs = [grey(sc) for _ in range(3)]
    rs = {n: Robot(x, y, 0.0, n) for n, (x, y) in steps[0][0].items()}
    w.robots[:] = list(rs.values())
    pics = []
    for pos, a in steps:
        for n, (x, y) in pos.items():
            rs[n].x, rs[n].y = x, y
        pics.append((grey(sc), dict(pos), a))
    out = []
    for cls in (CAM.MotionTracker, AT.AllTracker):
        tr = cls(cam, (61.0, 61.0))
        tr.set_area((0.0, 0.0, 122.15, 121.9))
        tr.set_background(bgs)
        if cls is AT.AllTracker:
            tr.WATCH_PARKED = watch
        for n, (x, y) in steps[0][0].items():
            tr.robot_at(x, y)
            tr.park(n, x, y)
        res = []
        act = None
        for k, (img, pos, a) in enumerate(pics):
            if a != act and a is not None:
                tr.follow(a, pos[a][0], pos[a][1], t=k * 0.04)
                act = a
            res.append(tr.push(img, k * 0.04 + 0.04))
        out.append((res, tr))
    return out


def straight_steps(n=60):
    steps = []
    for k in range(n):
        steps.append(({"A": (40.0 + 0.4 * k, 50.0), "B": (95.0, 30.0), "C": (30.0, 100.0)}, "A"))
    return steps


def test_alltracker_follows_exactly_as_motiontracker(scene):
    (r1, t1), (r2, t2) = run_both(scene, straight_steps())
    assert len(r1) == len(r2)
    for a, b in zip(r1, r2):
        assert (a is None) == (b is None)
        if a is not None:
            assert a[1] == pytest.approx(b[1], abs=1e-9) and a[2] == pytest.approx(b[2], abs=1e-9)
    assert np.array_equal(t1.bg, t2.bg) and np.array_equal(t1.known, t2.known)
    assert np.array_equal(t1.still_new, t2.still_new)
    assert t1.stats == t2.stats


@pytest.fixture
def every_picture(monkeypatch):
    """The parked watch on every picture (as built on 2026-10-25): these tests check its logic picture by picture.
    Since 2026-10-26 it runs on every PARK_EVERY_N-th picture (test_parked_watch_every_5th_picture)."""
    monkeypatch.setattr(AT.AllTracker, "PARK_EVERY_N", 1)


def test_parked_watch_every_5th_picture(scene):
    """2026-10-26 09:55 session: the watch on every 5th picture (its cost) still sees parked robots and a push."""
    assert AT.AllTracker.PARK_EVERY_N == 5 and AT.park_moved_n(5) == 3
    steps = straight_steps(60)
    for k in range(80):                     # B pushed 4 cm along y over 0.4 s, then stands (3.2 s)
        y = 30.0 + min(4.0, 0.4 * k)
        steps.append(({"A": (64.0, 50.0), "B": (95.0, y), "C": (30.0, 100.0)}, "A"))
    (_, _), (_, tr) = run_both(scene, steps)
    ev = [e for e in tr.take_park_events() if e[2] == "moved"]
    assert len(ev) == 1 and ev[0][1] == "B" and ev[0][3]["cm"] == pytest.approx(4.0, abs=1.0)
    ps = tr.parked_state()
    assert ps["C"]["status"] == "seen" and ps["C"]["moves"] == 0 and ps["C"]["seen_age_s"] < 0.25


def test_parked_robots_are_seen_in_every_picture(scene, every_picture):
    (_, _), (_, tr) = run_both(scene, straight_steps(40))
    ps = tr.parked_state()
    assert set(ps) == {"B", "C"}
    for r in ("B", "C"):
        assert ps[r]["status"] == "seen" and ps[r]["seen_age_s"] < 0.1 and ps[r]["moves"] == 0


def test_a_pushed_parked_robot_is_a_move_and_its_spot_follows(scene, every_picture):
    steps = straight_steps(30)
    for k in range(40):                     # B pushed 4 cm along y over 0.4 s, then stands
        y = 30.0 + min(4.0, 0.4 * k)
        steps.append(({"A": (52.0, 50.0), "B": (95.0, y), "C": (30.0, 100.0)}, "A"))
    (_, _), (_, tr) = run_both(scene, steps)
    ev = [e for e in tr.take_park_events() if e[2] == "moved"]
    assert len(ev) == 1 and ev[0][1] == "B"
    assert ev[0][3]["cm"] == pytest.approx(4.0, abs=1.0)
    assert tr.protect["B"][1] == pytest.approx(34.0, abs=1.0)
    assert tr.parked_state()["C"]["moves"] == 0


def test_a_parked_robot_taken_away_is_unseen_after_the_wait(scene, every_picture):
    w, sc, cam = scene
    w.robots[:] = []
    tr = AT.AllTracker(cam, (61.0, 61.0))
    tr.set_area((0.0, 0.0, 122.15, 121.9))
    tr.set_background([grey(sc) for _ in range(3)])
    b = Robot(95.0, 30.0, 0.0, "B")
    w.robots[:] = [b]
    tr.robot_at(b.x, b.y)
    tr.park("B", b.x, b.y)
    t = 0.0
    for _ in range(20):
        t += 0.04
        tr.push(grey(sc), t)
    assert tr.parked_state()["B"]["status"] == "seen"
    w.robots[:] = []
    seen_unseen = None
    for _ in range(int((AT.PARK_UNSEEN_S + 1.0) / 0.04)):
        t += 0.04
        tr.push(grey(sc), t)
        if seen_unseen is None and tr.parked_state()["B"]["status"] == "unseen":
            seen_unseen = t
    assert seen_unseen is not None
    assert any(e[2] == "unseen" for e in tr.take_park_events())
    # something robot-shaped comes back 5 cm away: "seen again", never a move across the gap
    b.y = 35.0
    w.robots[:] = [b]
    for _ in range(60):
        t += 0.04
        tr.push(grey(sc), t)
    ev = tr.take_park_events()
    assert any(e[2] == "seen again" for e in ev) and not any(e[2] == "moved" for e in ev)
    assert tr.parked["B"]["y"] == pytest.approx(30.0)
    assert "unconfirmed" in tr.parked_state()["B"]["status"]


def test_an_outline_that_shifts_while_the_robot_stands_is_not_a_move(scene, every_picture):
    """15:45 footage, Left by the bottom edge: its outline's centre jumped though it never moved. The picture where
    it stands is unchanged, so its spot must stay."""
    w, sc, cam = scene
    w.robots[:] = []
    tr = AT.AllTracker(cam, (61.0, 61.0))
    tr.set_area((0.0, 0.0, 122.15, 121.9))
    tr.set_background([grey(sc) for _ in range(3)])
    b = Robot(95.0, 30.0, 0.0, "B")
    w.robots[:] = [b]
    tr.robot_at(b.x, b.y)
    tr.park("B", b.x, b.y)
    t = 0.0
    for _ in range(25):
        t += 0.04
        tr.push(grey(sc), t)
    assert tr.parked["B"]["ref"] is not None
    # the floor memory beside it goes wrong (as by a board's edge): a robot-wide strip next to it now differs
    u, v = tr._px(95.0 + 6.0, 30.0)
    r = int(tr.rpx * 0.5)
    tr.frozen = True                                    # keep the floor memory from healing during the check
    tr.bg[int(v) - r:int(v) + r, int(u) - r // 2:int(u) + r] -= 60.0
    for _ in range(40):
        t += 0.04
        tr.push(grey(sc), t)
    ev = tr.take_park_events()
    assert not any(e[2] == "moved" for e in ev)
    assert any(e[2] == "outline shifted" for e in ev)      # the outline did move: the check had work to do
    assert tr.parked["B"]["x"] == pytest.approx(95.0) and tr.parked["B"]["moves"] == 0


def test_proc_tracker_takes_no_position_from_before_its_latest_command():
    import e02_trackproc as TP

    class FakeFeed:
        def __init__(self):
            self.n = 0

        def command(self, tr, op, *a, **k):
            self.n += 1
            tr.__dict__["_sent"] = self.n
            return self.n

    class Cam:
        @staticmethod
        def px_per_cm(x, y, z):
            return 2.34

    f = FakeFeed()
    tr = TP.ProcTracker(f, Cam(), (61.0, 61.0))
    tr.follow("A", 10.0, 20.0, t=5.0)
    assert tr.active == "A" and tr.where() == (5.0, 10.0, 20.0)
    base = dict(active=None, pos=None, last_seen=0.0, misses=0, source=None, frame_t=1.0, protect={}, hand_moved=[],
                moved_by_hand=0, conf={}, stats={}, stats_hold={}, in_hold=False, last_cands=[], hint=None,
                offset=0.0, thr=16.0, lock_off=(0.0, 0.0), use_lock=True, tn=1)
    assert tr._update(dict(base, cmd=0)) is False        # from before follow(): not taken
    assert tr.active == "A" and tr.where() == (5.0, 10.0, 20.0)
    assert tr._update(dict(base, cmd=1, active="A", pos=(6.0, 11.0, 20.0))) is True
    assert tr.where() == (6.0, 11.0, 20.0)
    tr.hand_moved.add("A")
    tr.hand_moved.discard("A")
    assert f.n == 2                                      # the discard went to the process


def test_parked_watch_off_changes_nothing(scene):
    (r1, t1), (r2, t2) = run_both(scene, straight_steps(30), watch=False)
    assert t2.parked_state()["B"]["status"] == "settling"
    assert all((a is None) == (b is None) for a, b in zip(r1, r2))


# =============================================================================== the shared picture ring
def test_ring_write_read_and_overwrite():
    ring = SF.FrameRing.create(slots=4, height=8, width=10)
    try:
        other = SF.FrameRing.attach(ring.name)
        nums = [ring.write(np.full((8, 10), k, np.uint8), t=100.0 + k, t_pi=k, pi_frame=7 + k) for k in range(6)]
        assert nums == list(range(6)) and other.newest() == 5
        img, meta = other.read(5)
        assert int(img[0, 0]) == 5 and meta[0] == 105.0 and meta[2] == 12
        assert other.read(1) is None                          # overwritten (4 slots)
        assert other.since(103.0) == [3, 4, 5]
        with pytest.raises(ValueError):
            ring.write(np.zeros((9, 10), np.uint8), t=0.0)
        other.close()
    finally:
        ring.close()


# =============================================================================== frame saving
def jpegs(n, start=0):
    return [cv2.imencode(".jpg", np.full((48, 64), (k * 7) % 255, np.uint8))[1].tobytes() for k in range(start, start + n)]


def test_frame_saver_writes_still_clip_and_row():
    d = tempfile.mkdtemp()
    fs = FS.FrameSaver(d, before_s=2.0, after_s=1.0, thumb_every_s=1.0, log=lambda m: None)
    js = jpegs(100)
    for k, j in enumerate(js):
        t = 1000.0 + k * 0.04
        fs.add(j, t, pi_frame=k)
        if k == 60:
            fs.event("hold", "e02 hold Right", t=t, robot="Right")
        if k == 70:
            fs.event("failed turn", "e02 turn fault Right", t=t, robot="Right")   # inside the same clip: joins it
    fs.close()
    rows = list(csv.DictReader(open(os.path.join(d, "events.csv"))))
    assert [r["kind"] for r in rows] == ["hold", "failed turn"]
    assert rows[0]["clip"] == rows[1]["clip"] and rows[0]["clip"].endswith(".avi")
    cap = cv2.VideoCapture(os.path.join(d, rows[0]["clip"]))
    n = 0
    while cap.read()[0]:
        n += 1
    assert 95 >= n >= 2.0 / 0.04 + 1.0 / 0.04          # 2 s before the first event to 1 s after the second
    assert os.path.exists(os.path.join(d, rows[0]["still"]))
    assert len(os.listdir(os.path.join(d, "thumbs"))) >= 3
    assert fs.summary()["clips"] == 1


def test_frame_saver_cap_stops_clips_not_stills():
    d = tempfile.mkdtemp()
    fs = FS.FrameSaver(d, before_s=0.5, after_s=0.1, thumb_every_s=0, max_mb=0.0, log=lambda m: None)
    for k, j in enumerate(jpegs(40)):
        fs.add(j, 10.0 + k * 0.04, k)
        if k == 20:
            fs.event("note", "Note 3", t=10.0 + k * 0.04)
    fs.close()
    rows = list(csv.DictReader(open(os.path.join(d, "events.csv"))))
    assert rows[0]["clip"] == "" and rows[0]["still"] != ""


@pytest.mark.parametrize("text,kind", [("e02 hold Right lost", "hold"), ("e02 hold answer a", None),
                                       ("e02 help Left", "help call"), ("e02 watchdog stop Left", "edge stop"), ("e02 hold border Right: crossed", "border stop"),
                                       ("e02 edge risk Right", "edge stop"), ("e02 turn fault Right", "failed turn"),
                                       ("e02 run 3 start", None)])
def test_marks_to_frame_saving_kinds(text, kind):
    assert FS.kind_of_mark(text) == kind


# =============================================================================== the CPU log
def test_cpu_log_writes_rows_with_cores_and_our_process():
    d = tempfile.mkdtemp()
    fake = lambda args, **k: type("R", (), {"stdout": "throttled=0x50005" if "get_throttled" in args else
                                            "temp=51.2'C"})()   # noqa: E731
    w = CL.CpuLog(d, procs={"runner": os.getpid(), "tracker": None}, find={"recorder": "no-such-program-xyz"},
                  every_s=0.05, run=fake)
    w.start()
    end = time.time() + 3
    while w.rows < 3 and time.time() < end:
        time.sleep(0.02)
    w.stop()
    w.join(2)
    rows = list(csv.DictReader(open(os.path.join(d, "cpu.csv"))))
    assert len(rows) >= 3
    assert "core0" in rows[0] and rows[-1]["runner_rss_mb"] != ""
    assert rows[-1]["temp_c"] != ""


def test_throttle_words():
    assert CL.parse_throttled("throttled=0x50005") == 0x50005
    assert CL.parse_throttled("0") == 0
    assert "under-voltage" in CL.throttle_words(0x1) and CL.throttle_words(0) == ""


# =============================================================================== fusion steering (off by default)
def test_fusion_choice():
    now = 100.0
    f = dict(t=99.95, x=10.0, y=10.0, sd_cm=1.0, heading_known=True)
    assert FU.choose((99.9, 10.5, 10.0), f, None, now)[3] == "camera"
    assert FU.choose((99.9, 16.0, 10.0), f, 8.0, now)[3] == "fused (camera near a board)"
    assert FU.choose((99.9, 16.0, 10.0), f, 40.0, now)[3] == "camera"
    assert FU.choose((98.0, 16.0, 10.0), f, 40.0, now)[3] == "fused (camera gap)"
    assert FU.choose(None, dict(f, heading_known=False), None, now) is None
    assert FU.choose(None, dict(f, sd_cm=5.0), None, now) is None
    assert FU.choose(None, dict(f, t=99.0), None, now) is None


# =============================================================================== the process, end to end
def write_clip(folder, imgs, t0=2000.0, fps=25.0):
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "clip.bin"), "wb") as b, open(os.path.join(folder, "clip_index.csv"), "w") as f:
        f.write("frame,off,size,t_wall\n")
        off = 0
        for k, img in enumerate(imgs):
            j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
            b.write(j)
            f.write("%d,%d,%d,%.4f\n" % (k, off, len(j), t0 + k / fps))
            off += len(j)


def test_process_end_to_end_matches_in_runner_tracker(scene):
    import e02_trackproc as TP
    w, sc, cam = scene
    w.robots[:] = []
    imgs = [grey(sc) for _ in range(6)]                       # empty board: the floor memory
    a, bb = Robot(40.0, 50.0, 0.0, "A"), Robot(95.0, 30.0, 0.0, "B")
    w.robots[:] = [a, bb]
    for k in range(70):
        a.x = 40.0 + 0.4 * max(0, k - 10)
        imgs.append(grey(sc))
    d = tempfile.mkdtemp()
    write_clip(os.path.join(d, "clip"), imgs)
    hist = CAM.History()
    got_hook = []
    f = TP.ProcFeed("clip:%s@0" % os.path.join(d, "clip"), hist, session_dir=os.path.join(d, "s"), lockstep=True,
                    save_frames=True, log=lambda m: None)
    f.keep_s = 30.0
    f.start()
    try:
        tr = f.make_tracker(cam, (61.0, 61.0))
        tr.set_area((0.0, 0.0, 122.15, 121.9))
        f.hooks.append(lambda img, t: got_hook.append(t))
        bgf = f.wait_frames(3, timeout=10)
        assert len(bgf) == 3 and f.numbers_of(bgf) is not None        # by number, not through the pipe
        tr.set_background(bgf)
        tr.robot_at(95.0, 30.0)
        tr.park("B", 95.0, 30.0)
        tr.follow("A", 40.0, 50.0)
        assert tr.where()[1:] == (40.0, 50.0) and tr.active == "A"   # at once, in the runner
        end = time.time() + 60
        while f.is_alive() and time.time() < end:
            time.sleep(0.05)
        p = tr.where()
        assert p is not None and p[1] == pytest.approx(40.0 + 0.4 * 59, abs=1.5)
        assert len(hist.since(0)) > 20
        assert tr.parked_state()["B"]["status"] in ("seen", "settling")
        assert len(got_hook) > 20
    finally:
        f.close()


def test_process_kept_pictures_and_event_files(scene):
    import e02_trackproc as TP
    w, sc, cam = scene
    w.robots[:] = [Robot(60.0, 60.0, 0.0, "A")]
    imgs = [grey(sc) for _ in range(120)]
    d = tempfile.mkdtemp()
    write_clip(os.path.join(d, "clip"), imgs)
    f = TP.ProcFeed("clip:%s@1" % os.path.join(d, "clip"), CAM.History(), session_dir=os.path.join(d, "s"),
                    save_frames=True, log=lambda m: None)
    f.start()
    try:
        f.keep_s = 30.0
        time.sleep(1.5)
        f.event("hold", "e02 hold A", robot="A")
        k = f.kept(time.time() - 1.0)
        assert k and len(k[-1]) == 4 and k[-1][1][:2] == b"\xff\xd8"
        assert f.frame_at(k[-1][0]) is not None
        assert f.last_img is not None and f.last_img.shape == imgs[0].shape
        end = time.time() + 20
        while f.is_alive() and time.time() < end:
            time.sleep(0.1)
    finally:
        f.close()
    rows = list(csv.DictReader(open(os.path.join(d, "s", "frames", "events.csv"))))
    assert rows and rows[0]["kind"] == "hold" and rows[0]["clip"].endswith(".avi")


def test_process_tracker_none_pauses_and_returns(scene):
    import e02_trackproc as TP
    w, sc, cam = scene
    w.robots[:] = []
    imgs = [grey(sc) for _ in range(500)]
    d = tempfile.mkdtemp()
    write_clip(os.path.join(d, "clip"), imgs)
    f = TP.ProcFeed("clip:%s@1" % os.path.join(d, "clip"), CAM.History(), log=lambda m: None)
    f.start()

    def grows(timeout=6.0):
        n0, end = f.n_trk, time.time() + timeout
        while time.time() < end:
            if f.n_trk > n0 + 2:
                return True
            time.sleep(0.05)
        return False
    try:
        tr = f.make_tracker(cam, (61.0, 61.0))
        assert grows()
        f.tracker = None
        assert f.tracker is None
        time.sleep(0.5)                      # pictures already on their way
        n1 = f.n_trk
        time.sleep(1.0)
        assert f.n_trk - n1 <= 1
        f.tracker = tr
        assert grows()
        with pytest.raises(ValueError):
            f.tracker = CAM.MotionTracker(cam)
    finally:
        f.close()


# =============================================================================== real footage (optional)
REAL = os.environ.get("E02_TRACKER_CLIP")


@pytest.mark.skipif(not REAL, reason="set E02_TRACKER_CLIP to a folder with clip.bin + clip_index.csv")
def test_real_footage_same_positions():
    import e02_plan as PL
    rows = list(csv.DictReader(open(os.path.join(REAL, "clip_index.csv"))))[:300]
    blob = open(os.path.join(REAL, "clip.bin"), "rb").read()
    imgs = [cv2.imdecode(np.frombuffer(blob[int(r["off"]):int(r["off"]) + int(r["size"])], np.uint8), 0)
            for r in rows]
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cam = CAM.load_camera(os.path.join(here, "data", "overhead_floorcal_roam1.json"))
    b = PL.Board()
    res = []
    for cls in (CAM.MotionTracker, AT.AllTracker):
        tr = cls(cam, (61.0, 61.0))
        tr.set_area((b.x0, b.y0, b.x1, b.y1))
        tr.set_background(imgs[:5], robots=[(18.34, 104.93), (111.09, 17.1), (33.66, 83.43)])
        if cls is AT.AllTracker:
            tr.MOVE_SPOT = False
        tr.park("Left", 18.34, 104.93)
        tr.park("Right", 33.66, 83.43)
        tr.follow("Center", 111.09, 17.1, t=float(rows[0]["t_wall"]))
        res.append([tr.push(im, float(r["t_wall"])) for im, r in zip(imgs, rows)])
    assert res[0] == res[1]


# =============================================================================== the runner's wiring (e02_trackwire)
class FakeRun:
    def __init__(self, d):
        import e02_plan as PL
        self.dir = d
        self.st = {"robots": ["Left", "Center", "Right"], "positions": {"Right": [30.0, 80.0, None]}}
        self.lines = []
        self.board = PL.Board(xboard_side=1, yboard_side=1)
        self.lab = None

    def say(self, m, quiet=False):
        self.lines.append(m)


def test_wiring_parked_move_updates_the_stored_place_and_logs(tmp_path):
    import e02_trackwire as TKW
    run = FakeRun(str(tmp_path))
    run.feed = None
    TKW.on_park_event(run, 100.0, "Right", "moved", dict(x0=30.0, y0=80.0, x=30.5, y=84.0, cm=4.0))
    assert run.st["positions"]["Right"][:2] == [30.5, 84.0]
    rows = list(csv.DictReader(open(os.path.join(str(tmp_path), "parked.csv"))))
    assert rows[0]["event"] == "moved" and rows[0]["robot"] == "Right"
    assert any("moved 4.0 cm" in ln for ln in run.lines)


def test_wiring_marks_become_events_only_with_the_process(tmp_path):
    import e02_trackwire as TKW
    import e02_trackproc as TP
    run = FakeRun(str(tmp_path))
    sent = []
    f = TP.ProcFeed.__new__(TP.ProcFeed)
    f.event = lambda *a: sent.append(a)
    run.feed = f
    TKW.on_mark(run, "e02 hold lost Right: no camera position")
    TKW.on_mark(run, "e02 run 3 start")
    TKW.on_note(run, {"note": 4, "text": "two robots touching"})
    assert [s[0] for s in sent] == ["hold", "note"] and sent[0][3] == "Right"


def test_wiring_fusion_steer_uses_the_camera_when_it_is_fresh(tmp_path):
    import e02_trackwire as TKW
    run = FakeRun(str(tmp_path))
    now = time.time()
    assert TKW.steer_pos(run, "Left", (now - 0.1, 50.0, 50.0), 1.5)[1:] == (50.0, 50.0)
    assert TKW.steer_pos(run, "Left", (now - 1.0, 50.0, 50.0), 1.5)[1:] == (50.0, 50.0)   # no lab: the old rule
    assert TKW.steer_pos(run, "Left", (now - 2.0, 50.0, 50.0), 1.5) is None


def test_device_source_keeps_the_cameras_own_jpeg_bytes():
    """Production: the tracker process opens the camera. With CONVERT_RGB off, OpenCV hands over the MJPG bytes as
    one row; they are kept as they are (no decode and encode again). A camera that decodes anyway is handled too."""
    import e02_trackproc as TP
    img = np.full((48, 64), 90, np.uint8)
    jpg = cv2.imencode(".jpg", img)[1]

    class Cap:
        def __init__(self, raw):
            self.raw, self.n = raw, 0

        def set(self, prop, v):
            return self.raw

        def read(self):
            self.n += 1
            return True, (jpg.reshape(1, -1) if self.raw else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))

        def release(self):
            pass
    for raw in (True, False):
        src = TP.DevicePics(opener=lambda *a, raw=raw: Cap(raw), log=lambda m: None)
        it = iter(src)
        p = next(it)
        assert p.jpeg[:2] == b"\xff\xd8"
        assert (p.img is None) == raw
        if raw:
            assert p.jpeg == jpg.tobytes()
        src.close()
