"""Synthetic E02 data with known truth, for testing e02_analysis.py (and dry runs of the report).

  fake_runs(rng, n, spec)     run rows (no video) with errors drawn from known distributions per condition:
                              check that the statistics recover them.
  make_session(folder, specs) a tiny overhead video (320x240, 25 fps) plus frames.csv, events.csv, runs.csv,
                              video_clock.json and logs/{bump,find,e02,clock_sync}.csv for a few runs, each
                              with a find (two bumps), a route with a pause at the far point, and a return;
                              returns the truth so a test can compare. Camera: overhead, 2 px/cm.
"""
import csv
import json
import math
import os

import cv2
import numpy as np

import e02_analysis as ea

T0 = 1_760_000_000.0
FPS = 25.0
SIZE = (320, 240)
CAM_C = (60.0, 90.0)
CAM_H = 200.0
SCALE = 2.0                       # px per cm at the robot's top
BX, BY = 100.0, 100.0             # board lines the robot bumps into (x board, y board)
CLOCK_ROBOT_OFF = T0 - 5.0        # laptop time = robot_ms / 1000 + this
VIDEO_OFF = 0.3                   # video time = laptop time + this
ROUTES_SMALL = {"straight": (40.0, 0.0, 0.0), "L": (20.0, 20.0, 90.0), "square": (20.0, 20.0, 90.0)}
DECOYS = [(15.0, 55.0, 30.0), (20.0, 125.0, 200.0), (115.0, 50.0, 100.0)]    # parked robots


def make_camera():
    from cameras import CameraModel
    f = SCALE * (CAM_H - ea.TOP_CM)
    K = [[f, 0, SIZE[0] / 2], [0, f, SIZE[1] / 2], [0, 0, 1]]
    return CameraModel(K, None, [math.pi, 0, 0], [-CAM_C[0], CAM_C[1], CAM_H], SIZE)


def fake_runs(rng, n, spec, routes=("straight", "L", "square"), robots=("Left", "Center", "Right")):
    """spec: {condition: dict(end_cm=(mean, sd), end_deg=(mean, sd))} -> run dicts like runs.csv rows. The end
    position error is drawn as |N(mean, sd)| (a distance), the signed heading error as N(mean, sd)."""
    rows = []
    for c, sp in spec.items():
        for i in range(n):
            h = rng.normal(*sp["end_deg"])
            rows.append(dict(run_id="%s%d" % (c, i), robot=robots[i % len(robots)], condition=c,
                             route=routes[i % len(routes)], lost=False,
                             end_err_cm=float(abs(rng.normal(*sp["end_cm"]))), end_err_deg=float(h),
                             end_head_abs_deg=float(abs(h))))
    return rows


# ------------------------------------------------------------------ the world
def _lerp(a, b, u):
    return tuple(a[i] + (b[i] - a[i]) * u for i in range(3))


def run_timeline(spec):
    """Piecewise pose(t) for one run (t from the run's first moment) and its landmarks (truth)."""
    put = (BX - 5.0 - 24.0, BY - 5.0 - 24.0, 0.0)            # put-down pose (the robot's own frame origin)
    c1 = (BX - 5.0, put[1], 0.0)                            # touching the x board
    b1 = (BX - 10.0, put[1], 0.0)
    t1 = (BX - 10.0, put[1], 90.0)
    c2 = (BX - 10.0, BY - 5.0, 90.0)                        # touching the y board
    start = (BX - 10.0, BY - 10.0, 90.0)
    rel = spec.get("routes", ROUTES_SMALL)[spec["route"]]
    far_i = ea.compose(start, rel)
    ex, ey, eth = spec["end_err"]
    fx, fy, fth = spec.get("far_err", (0.0, 0.0, 0.0))
    far = (far_i[0] + fx, far_i[1] + fy, far_i[2] + fth)
    end = (start[0] + ex, start[1] + ey, start[2] + eth)
    seg = []                                                # (duration, from, to, name)
    seg += [(0.5, put, put), (2.0, put, c1), (0.8, c1, c1), (0.6, c1, b1), (1.0, b1, t1)]
    seg += [(2.0, t1, c2), (0.8, c2, c2), (0.6, c2, start), (1.0, start, start)]
    t_find_end = sum(s[0] for s in seg)
    seg += [(2.0, start, far), (1.0, far, far), (2.0, far, end), (3.0, end, end)]
    times, tt = [], 0.0
    for d, a, b in seg:
        times.append((tt, tt + d, a, b))
        tt += d
    marks = dict(c1=0.5 + 2.0, c2=0.5 + 2.0 + 0.8 + 0.6 + 1.0 + 2.0, start=t_find_end, far=t_find_end + 2.0,
                 end=t_find_end + 2.0 + 1.0 + 2.0, total=tt)
    return dict(times=times, put=put, c1=c1, c2=c2, start=start, far=far, end=end, marks=marks, rel=rel)


def pose_at(tl, t):
    for t0, t1, a, b in tl["times"]:
        if t <= t1:
            return _lerp(a, b, 0.0 if t1 == t0 else min(1.0, max(0.0, (t - t0) / (t1 - t0))))
    return tl["times"][-1][3]


_FLOOR = []


def _floor():
    if not _FLOOR:
        yy, xx = np.mgrid[0:SIZE[1], 0:SIZE[0]]
        _FLOOR.append((150 + 12 * np.sin(xx / 37.0) + 8 * np.cos(yy / 29.0)).astype(np.float32))
    return _FLOOR[0].copy()


def render(cam, pose, rng, noise=2.0, ss=4):
    """Grey picture: floor texture, parked decoy robots and the robot at pose (dark rectangles, each drawn at ss
    times the size in a small patch and averaged down, so edges are exact to a fraction of a pixel)."""
    img = _floor()
    for dx, dy, dth in DECOYS:
        _stamp(img, _outline(cam, (dx, dy, dth)), 105.0, ss)
    _stamp(img, _outline(cam, pose), 115.0, ss)
    img += rng.normal(0, noise, img.shape).astype(np.float32)
    return np.clip(img, 0, 255).astype(np.uint8)


def _stamp(img, poly, value, ss):
    u0, v0 = int(np.floor(poly[:, 0].min())) - 2, int(np.floor(poly[:, 1].min())) - 2
    u1, v1 = int(np.ceil(poly[:, 0].max())) + 3, int(np.ceil(poly[:, 1].max())) + 3
    u0, v0, u1, v1 = max(u0, 0), max(v0, 0), min(u1, SIZE[0]), min(v1, SIZE[1])
    if u1 <= u0 or v1 <= v0:
        return
    patch = np.zeros(((v1 - v0) * ss, (u1 - u0) * ss), np.float32)
    p = (poly.astype(np.float64) - (u0, v0) + 0.5) * ss - 0.5
    cv2.fillPoly(patch, [np.round(p * 8).astype(np.int32)], value, lineType=cv2.LINE_8, shift=3)
    img[v0:v1, u0:u1] -= cv2.resize(patch, (u1 - u0, v1 - v0), interpolation=cv2.INTER_AREA)


def _outline(cam, pose):
    x, y, th = pose
    hw, hl = ea.ROBOT_W_CM / 2, ea.ROBOT_L_CM / 2
    pts = []
    for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):          # ahead, left
        px, py, _ = ea.compose((x, y, th), (a, b, 0.0))
        u, v = cam.project((px, py, ea.TOP_CM))
        pts.append((u, v))
    return np.array(pts, np.float32)


def make_session(folder, specs, seed=1, gap_s=1.5):
    """Write a recording + logs for the runs in specs (dicts: run_id, robot, condition, route, end_err
    (dx, dy, dth) truth, optional far_err, bel_bias (dx, dy, dth) of the robot's belief). Returns truth."""
    rng = np.random.default_rng(seed)
    cam = make_camera()
    rec, logs = os.path.join(folder, "rec"), os.path.join(folder, "logs")
    os.makedirs(rec, exist_ok=True)
    os.makedirs(logs, exist_ok=True)
    vw = cv2.VideoWriter(os.path.join(rec, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), FPS, SIZE)
    frames, events, runs, bump, find, e02, truth = [], [], [], [], [], [], {}
    n = 0
    for k, sp in enumerate(specs):
        tl = run_timeline(sp)
        tr0 = T0 + 1.0 + n / FPS
        for j in range(int(tl["marks"]["total"] * FPS) + 1):
            pose = pose_at(tl, j / FPS)
            vw.write(cv2.cvtColor(render(cam, pose, rng), cv2.COLOR_GRAY2BGR))
            frames.append((n, T0 + 1.0 + n / FPS))
            n += 1
        m = tl["marks"]
        rid, rb, rr = str(sp["run_id"]), sp["robot"], k + 1
        runs.append(dict(run_id=rid, robot=rb, condition=sp["condition"], route=sp["route"], block=1 + k // 4,
                         robot_run=rr, lost=int(sp.get("lost", 0)), t_find=tr0, t_start=tr0 + m["start"],
                         t_far=tr0 + m["far"], t_end=tr0 + m["end"]))
        for ph, key in (("find", None), ("start", "start"), ("far", "far"), ("end", "end")):
            tm = tr0 if key is None else tr0 + m[key]
            events.append((tm, int(round((tm - T0 - 1.0) * FPS)), "E02 run %s %s" % (rid, ph)))

        def robot_ms(tv):
            return (tv - VIDEO_OFF - CLOCK_ROBOT_OFF) * 1000.0

        put = tl["put"]
        for board, key, tc in (("x", "c1", m["c1"]), ("y", "c2", m["c2"])):
            own = ea.relative(tl[key], put)
            noise = rng.normal(0, 0.3, 3) * (1, 1, 2)
            bump.append(dict(robot=rb, run=rr, board=board, x_cm=own[0] + noise[0], y_cm=own[1] + noise[1],
                             th_deg=own[2] + noise[2], jolt=1.5 + rng.normal(0, 0.1), delay_ms=80,
                             overshoot_cm=0.3, ms=robot_ms(tr0 + tc + 0.08)))
        bias = sp.get("bel_bias", (0.0, 0.0, 0.0))
        b0 = tuple(np.array(tl["start"]) + rng.normal(0, 0.4, 3) * (1, 1, 2))
        rel_true_far = ea.relative(tl["far"], tl["start"])
        rel_true_end = ea.relative(tl["end"], tl["start"])
        bfar = ea.compose(b0, tuple(rel_true_far[i] + bias[i] for i in range(3)))
        bend = ea.compose(b0, tuple(rel_true_end[i] + bias[i] for i in range(3)))
        find.append(dict(robot=rb, run=rr, result="done", after_x_cm=b0[0], after_y_cm=b0[1], after_th_deg=b0[2]))
        e02.append(dict(robot=rb, run=rr, route=sp["route"], mode=0, start_x_cm=b0[0], start_y_cm=b0[1],
                        start_th_deg=b0[2], far_x_cm=bfar[0], far_y_cm=bfar[1], far_th_deg=bfar[2],
                        end_x_cm=bend[0], end_y_cm=bend[1], end_th_deg=bend[2], ms_start=robot_ms(tr0)))
        truth[rid] = dict(start=tl["start"], far=tl["far"], end=tl["end"], c1=tl["c1"], c2=tl["c2"],
                          t_c1=tr0 + m["c1"], t_c2=tr0 + m["c2"], put=put, t_start=tr0 + m["start"],
                          bel_bias=bias, b0=b0)
        nxt = run_timeline(specs[k + 1])["put"] if k + 1 < len(specs) else tl["end"]    # carried to the next put-down
        for _ in range(int(gap_s * FPS)):
            vw.write(cv2.cvtColor(render(cam, nxt, rng), cv2.COLOR_GRAY2BGR))
            frames.append((n, T0 + 1.0 + n / FPS))
            n += 1
    vw.release()
    _write(os.path.join(rec, "frames.csv"), ["frame", "t_wall"], [dict(frame=a, t_wall="%.4f" % b) for a, b in frames])
    _write(os.path.join(rec, "events.csv"), ["t_wall", "frame", "text"],
           [dict(t_wall="%.4f" % a, frame=b, text=c) for a, b, c in events])
    _write(os.path.join(folder, "runs.csv"), list(runs[0]), runs)
    _write(os.path.join(logs, "bump.csv"), list(bump[0]), bump)
    _write(os.path.join(logs, "find.csv"), list(find[0]), find)
    _write(os.path.join(logs, "e02.csv"), list(e02[0]), e02)
    _write(os.path.join(logs, "clock_sync.csv"), ["robot", "boot", "token", "t_send", "t_recv", "robot_ms", "rtt_ms", "offset_s"],
           [dict(robot=r, boot=1, token=i, t_send=0, t_recv=0, robot_ms=0, rtt_ms=10 + i, offset_s=CLOCK_ROBOT_OFF)
            for r in sorted({s["robot"] for s in specs}) for i in range(6)])
    with open(os.path.join(rec, "video_clock.json"), "w") as f:
        json.dump(dict(offset_s=VIDEO_OFF, drift_ppm=0.0, use_drift=False), f)
    cam.save(os.path.join(folder, "camera.json"))
    return dict(truth=truth, folder=folder, recording=rec, logs=logs, runs=os.path.join(folder, "runs.csv"),
                calib=os.path.join(folder, "camera.json"), cam=cam)


def _write(path, cols, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
