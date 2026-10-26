"""Explore reliability (2026-10-26, docs/build-reports/e02-oct26-explore-reliability.md): legs whose wheels turned
while the body did not follow are repaired from the camera before the fit is trusted; a creep step that the camera
saw stand still is a contact at the bumper; robots left out of the fit still give their contacts through the fitted
camera; and the shared picture ring survives being closed while another thread reads it (the shutdown crash, exit
139)."""
import math
import os
import subprocess
import sys
import textwrap
import types

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
TOOLS = os.path.dirname(EXP)
for p in (EXP, TOOLS, os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import e02_camera as CAM  # noqa: E402
import e02_explore as X  # noqa: E402
import e02_shmframes as SF  # noqa: E402

CALIB = os.path.join(EXP, "data", "overhead_floorcal_roam1.json")
W, H = 122.15, 121.9
# start (x, y, heading), wheel scale (true cm per counted cm), gyro scale (true turn per counted)
TRUTH = {"Left": ((25, 100, 0), 1.00, 1.07), "Center": ((100, 25, 90), 0.99, 1.08), "Right": ((30, 60, -30), 1.01, 1.07)}


def hit(x, y, th):
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    best = None
    for k, (a, v) in {"x0": ("x", 0), "x1": ("x", W), "y0": ("y", 0), "y1": ("y", H)}.items():
        den = c if a == "x" else s
        if abs(den) < 1e-9:
            continue
        d = (v - (x if a == "x" else y)) / den
        if d > 0 and (best is None or d < best[0]):
            best = (d, k)
    return best


def session(cam, seed=3, phantom=None):
    """Leg-format stops (d, dth: the leg's own record), as the runner records them. phantom: {robot: [ray index]}:
    on those rays the robot meets the side, then creeps 4 times 2.9 cm (counted) that move it 0.3 cm (a slanted
    bumper against a board), or a short ray to an edge counts 10 cm that never happened (a hang at an edge)."""
    rng = np.random.default_rng(seed)
    robots, truth = {}, {}
    for n, ((x, y, th), s_t, g_t) in TRUTH.items():
        st = dict(tx=x, ty=y, tth=th)
        stops, tp = [], []

        def stop(d, dth, contact=None):
            u, v = cam.project((st["tx"], st["ty"], X.ROBOT_TOP_CM))
            q = dict(d=d, dth=dth, u=u + rng.normal(0, 0.7), v=v + rng.normal(0, 0.7), t=1000.0 + len(stops))
            if contact:
                q["contact"] = contact
            stops.append(q)
            tp.append((st["tx"], st["ty"], st["tth"]))

        def drive(true_cm, counted_cm):
            c, s_ = math.cos(math.radians(st["tth"])), math.sin(math.radians(st["tth"]))
            st["tx"] += true_cm * c
            st["ty"] += true_cm * s_
            return counted_cm

        def turn(deg):
            st["tth"] += deg
            stop(0.0, deg / g_t * (1 + rng.normal(0, 0.003)))

        stop(0.0, 0.0)
        for _ in range(4):
            stop(drive(15.0, 15.0 / s_t), 0.0)
            turn(90.0)
        for k in range(6):
            turn(60.0)
            d, side = hit(st["tx"], st["ty"], st["tth"])
            kd = "edge" if side in ("x0", "y0") else "board"
            out = d - (3.5 if kd == "edge" else 11.5)
            if out <= 4:
                continue
            # a hang at an edge after a short ray (on a long one 10 cm is within the camera's own size errors, and
            # the repair leaves such legs alone: after-batch run r07)
            extra = 10.0 if phantom and k in phantom.get(n, []) and kd == "edge" and out < 60 else 0.0
            stop(drive(out, out / s_t + extra), 0.0, kd)
            if phantom and k in phantom.get(n, []) and kd == "board":
                for _ in range(4):
                    stop(drive(0.3, 2.9), 0.0)
            turn(180.0)
            stop(drive(out, out / s_t), 0.0)
        robots[n], truth[n] = stops, tp
    return robots, truth


def camera_scale(cam, sol):
    """The fitted camera's cm per true cm over the board (a similarity from its world to the truth)."""
    G = np.array([(x, y) for x in np.arange(10, 115, 15.0) for y in np.arange(10, 115, 15.0)])
    A = np.array([X.world_of(sol, *cam.project((x, y, X.ROBOT_TOP_CM))) for x, y in G])
    best = None
    for mir in (1.0, -1.0):
        Am = A * np.array([1.0, mir])
        ma, mb = Am.mean(0), G.mean(0)
        U, S, Vt = np.linalg.svd((G - mb).T @ (Am - ma))
        D = np.eye(2)
        if np.linalg.det(U @ Vt) < 0:
            D[1, 1] = -1
        R = U @ D @ Vt
        s = float(np.trace(np.diag(S) @ D) / ((Am - ma) ** 2).sum())
        res = np.linalg.norm((s * (R @ (Am - ma).T)).T + mb - G, axis=1)
        if best is None or np.median(res) < best[1]:
            best = (1.0 / s, float(np.median(res)))
    return best


@pytest.fixture(scope="module")
def cam():
    if not os.path.exists(CALIB):
        pytest.skip("no calibration file")
    return CAM.load_camera(CALIB)


@pytest.fixture
def repair_on():
    old = X.REPAIR_LEGS
    yield
    X.REPAIR_LEGS = old


def test_clean_session_needs_no_repair(cam, repair_on):
    robots, _ = session(cam)
    X.REPAIR_LEGS = True
    sol = X.fit(robots, (640, 480))
    assert sol["repaired"] == 0 and sol["rms_px"] < 3.0
    assert abs(camera_scale(cam, sol)[0] - 1.0) < 0.03


def test_overcounted_legs_are_repaired_and_the_scale_holds(cam, repair_on):
    robots, _ = session(cam, phantom={"Left": [0, 1, 2, 3, 4, 5]})
    X.REPAIR_LEGS = False
    bad = X.fit(robots, (640, 480))
    X.REPAIR_LEGS = True
    good = X.fit(robots, (640, 480))
    assert good["repaired"] >= 4
    assert sum(len(v) for v in good["repairs"].values()) == good["repaired"]
    assert all(n == "Left" for n in good["repairs"])          # no other robot's legs touched
    assert good["rms_px"] < bad["rms_px"]
    assert abs(good["s"]["Left"] - TRUTH["Left"][1]) < abs(bad["s"]["Left"] - TRUTH["Left"][1])
    assert abs(good["s"]["Left"] - TRUTH["Left"][1]) < 0.03
    assert abs(camera_scale(cam, good)[0] - 1.0) < 0.03
    # the runner's own stops get the same repairs (contacts and end places are built on them)
    rep = X.apply_repairs(robots, good)
    i, d = good["repairs"]["Left"][0][:2]
    assert rep["Left"][i]["d"] == d and rep["Left"][i]["d_own"] == robots["Left"][i]["d"]
    assert "d_own" not in robots["Left"][i]                   # the input is not changed


def test_lost_records_are_fitted(cam, repair_on):
    """A leg whose record never arrived is taken as told by the runner and marked lost: the fit takes its length or
    turn as unknown (a told 60 deg turn the gyro would have counted as 56; a drive whose camera estimate is 20 % long)."""
    robots, _ = session(cam)
    X.REPAIR_LEGS = True
    v = robots["Center"]
    ti = next(i for i, q in enumerate(v) if i > 12 and abs(q["dth"] - 60 / 1.08) < 1.0)
    gi = next(i for i, q in enumerate(v) if i > ti and q["d"] > 20)
    true_dth, true_d = v[ti]["dth"], v[gi]["d"]
    v[ti] = dict(v[ti], dth=60.0, lost=dict(kind="turn", val=60.0))
    v[gi] = dict(v[gi], d=true_d * 1.2, lost=dict(kind="go", val=105.0))
    sol = X.fit(robots, (640, 480))
    rows = {r[0]: r for r in sol["repairs"]["Center"]}
    assert abs(rows[ti][2] - true_dth) < 1.5
    assert abs(rows[gi][1] - true_d) < 2.0
    assert sol["rms_px"] < 3.0


def test_left_out_robot_contacts_are_placed_by_the_camera(cam):
    robots, truth = session(cam)
    used = {n: v for n, v in robots.items() if n != "Right"}
    sol = X.fit(used, (640, 480))
    seen = X.contacts_seen({"Right": robots["Right"]}, sol)
    n_contacts = sum(1 for q in robots["Right"] if q.get("contact"))
    assert len(seen) == n_contacts > 0
    # each lands on a true side once the fitted world is put on the truth (similarity from the used robots' stops)
    P = np.array([X.world_of(sol, q["u"], q["v"]) for n in used for q in used[n]])
    T = np.array([p[:2] for n in used for p in truth[n]])
    mp, mt = P.mean(0), T.mean(0)
    best = None
    for mir in (1.0, -1.0):
        Pm = P * np.array([1.0, mir])
        U, S, Vt = np.linalg.svd((T - mt).T @ (Pm - Pm.mean(0)))
        D = np.eye(2)
        if np.linalg.det(U @ Vt) < 0:
            D[1, 1] = -1
        R = U @ D @ Vt
        s = float(np.trace(np.diag(S) @ D) / ((Pm - Pm.mean(0)) ** 2).sum())
        r = np.linalg.norm((s * (R @ (Pm - Pm.mean(0)).T)).T + mt - T, axis=1)
        if best is None or np.median(r) < best[0]:
            best = (np.median(r), mir, R, s, Pm.mean(0))
    _, mir, R, s, mpm = best
    for c in seen:
        x, y = s * (R @ (np.array([c[0], mir * c[1]]) - mpm)) + mt
        assert min(abs(x), abs(y), abs(x - W), abs(y - H)) < 5.0, (x, y, c[2])


def test_left_out_ray_whose_camera_length_disagrees_is_not_used(cam):
    robots, _ = session(cam)
    used = {n: v for n, v in robots.items() if n != "Right"}
    sol = X.fit(used, (640, 480))
    right = [dict(q) for q in robots["Right"]]
    for q in right:
        if q.get("contact"):
            q["d"] *= 1.6                                     # the spot did not follow the robot on these rays
    assert X.contacts_seen({"Right": right}, sol) == []


def test_creep_stall_is_seen_by_the_camera():
    import e02 as E
    # a robot's straight legs: 20 px per 10 cm (2 px per cm)
    obs = {"Left": [dict(u=100.0, v=100.0, d=0.0, dth=0.0)]}
    for k in range(4):
        obs["Left"].append(dict(u=100.0 + 20.0 * (k + 1), v=100.0, d=10.0, dth=0.0))
    me = types.SimpleNamespace(explore_px_per_cm=lambda n, o: E.Runner.explore_px_per_cm(None, n, o))
    assert abs(E.Runner.explore_px_per_cm(None, "Left", obs) - 2.0) < 1e-9
    prev = obs["Left"][-1]
    stalled = dict(u=prev["u"] + 0.6, v=100.0, d=2.9, dth=0.0)       # 0.3 cm seen for 2.9 counted
    free = dict(u=prev["u"] + 5.6, v=100.0, d=2.9, dth=0.0)          # 2.8 cm seen
    got = E.Runner.explore_creep_stalled(me, "Left", obs, prev, stalled, 2.9)
    assert got is not None and abs(got - 0.3) < 0.01
    assert E.Runner.explore_creep_stalled(me, "Left", obs, prev, free, 2.9) is None
    # too few legs to know the robot's pixels per cm: no call either way
    assert E.Runner.explore_creep_stalled(me, "Left", {"Left": obs["Left"][:2]}, prev, stalled, 2.9) is None


def test_ring_read_after_close_is_none():
    ring = SF.FrameRing.create(slots=4, height=8, width=10)
    other = SF.FrameRing.attach(ring.name)
    n = ring.write(np.full((8, 10), 3, np.uint8), t=1.0)
    assert other.read(n) is not None
    other.close()
    assert other.read(n) is None and other.newest() == -1 and other.meta_of(n) is None and other.since(0.0) == []
    other.close()                                            # twice is fine
    ring.close()
    assert ring.write(np.zeros((8, 10), np.uint8), t=2.0) == -1


def test_ring_closed_while_a_thread_reads_does_not_crash():
    """The shutdown crash: before the lock, this ended with a segmentation fault (exit 139) every time."""
    code = textwrap.dedent("""
        import sys, threading, time
        sys.path.insert(0, %r)
        import numpy as np
        import e02_shmframes as SF
        w = SF.FrameRing.create(slots=8, height=480, width=640)
        for i in range(8):
            w.write(np.zeros((480, 640), np.uint8), time.time())
        r = SF.FrameRing(w.shm, owner=False)                 # a second view of the same block (no resource tracker)
        stop = []
        def loop():
            while not stop:
                r.read(max(0, r.newest()))
        th = threading.Thread(target=loop, daemon=True)
        th.start()
        time.sleep(0.2)
        r.close()
        time.sleep(0.3)
        stop.append(1)
        th.join(1.0)
        w.close()
        print("survived")
    """ % EXP)
    for _ in range(3):
        p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=60)
        assert p.returncode == 0 and "survived" in p.stdout, (p.returncode, p.stderr[-500:])


def test_a_done_record_wins_over_a_later_dup():
    """A leg resent with the same id is answered `dup`; arriving after the leg's own `done` it hid it, and explore
    recorded the move as nothing (after-batch run r02, first attempt: `go 12.00 438086` sent twice)."""
    import collections
    import threading
    import e02 as E
    L = E.Listener.__new__(E.Listener)
    L.cv = threading.Condition()
    L.recs = collections.defaultdict(list)
    done = {"id": "7", "run": "-", "state": "done", "x_cm": "11.6", "y_cm": "0", "th_deg": "0"}
    dup = {"id": "7", "run": "-", "state": "dup", "x_cm": "0", "y_cm": "0", "th_deg": "0"}
    L.recs["LEG"] = [(1.0, "Left", done), (2.0, "Left", dup)]
    assert L.leg_final("Left", "-", 7)["state"] == "done"
    L.recs["LEG"] = [(2.0, "Left", dup)]
    assert L.leg_final("Left", "-", 7)["state"] == "dup"       # nothing better: the dup (explore takes it as told)
    assert L.leg_final("Left", "-", 8) is None
