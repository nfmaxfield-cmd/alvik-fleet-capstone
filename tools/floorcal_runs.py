"""Calibration from the runs themselves (no clicking), for tools/calibrate_floor.py.

Two ways, both pairing what the robot logs with where the camera saw it:

  still points  (--from-runs) Each tape-measured run gives two floor points:
                the start pose (xf0..yr0) while the robot sits still before
                it first moves, and the end pose (xf..yr) after it stops.
                Paired with the median detection in those still windows.
  survey        (--survey) While the robot drives, its own wheel distance +
                gyro heading path (tools/fusion EKF, the replay.py setup
                "wheel distance + gyro heading"), started at the tape start
                pose (the one anchor), is paired with the camera track at the
                same moments: hundreds of pairs, weighted by the path's own
                uncertainty (it grows as odometry drifts), RANSAC against wrong
                detections. Recalibrates a camera that was moved, with no
                clicking and no tape beyond the start marks.

Circularity: a calibration made from a run's tape marks (or its odometry path)
must not be used to score the camera against that same run. calibrate_floor.py
therefore also writes one calibration per run made WITHOUT that run
(leave-one-run-out, or k-fold by run) and floor_calibration_loro.csv saying
which file to use to score which run.

Survey goes further: the odometry path carries the robot's own wheel-size and
gyro errors into the camera fit, so a calibration made from Left's other runs
shares Left's wheel error with the run being scored (camera and wheels would
then agree for the wrong reason). --holdout robot leaves each robot out
entirely (floor_calibration_loro_robot.csv); use that, or a click / tape-point
calibration, for headline camera-vs-wheels numbers (T2/T3).
"""
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (HERE, os.path.join(HERE, "fusion")):
    if p not in sys.path:
        sys.path.insert(0, p)

import runs as rn          # noqa: E402  (tools/fusion/runs.py)

ODO_CONFIG = dict(name="wheel distance + gyro heading", sensors=["wheel_v", "gyro"])   # as in tools/fusion/replay.py


def fnum(x, default=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


# ------------------------------------------------------------------ detections
def read_detections(path):
    """Camera sightings as dicts t_wall, frame, x, y, robot (None if anonymous), kind.
    Accepts a detector CSV (frame,t_wall,x_px,y_px,score[,cls,...]), the simulator's
    camera_detections.csv, labels.csv (confirmed/corrected/auto alvik rows), or
    tools/blink's blink_detections.csv (robot known, but the point is the LEDs'
    midpoint, ahead of the robot's centre: kind 'led')."""
    out = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        rd = csv.DictReader(f)
        blink = "code" in (rd.fieldnames or []) and "lit" in (rd.fieldnames or [])
        for r in rd:
            if blink:
                if fnum(r.get("n_spots"), 1) == 0:      # interpolated between pulses
                    continue
            else:
                if r.get("cls") not in (None, "", "alvik"):
                    continue
                if "status" in r and r.get("status") not in ("confirmed", "corrected", "auto"):
                    continue
            t, x, y = fnum(r.get("t_wall")), fnum(r.get("x_px")), fnum(r.get("y_px"))
            if any(math.isnan(v) for v in (t, x, y)):
                continue
            out.append(dict(t_wall=t, frame=int(fnum(r.get("frame"), -1)), x=x, y=y,
                            robot=(r.get("robot") or None) if blink else None, kind="led" if blink else "centre"))
    out.sort(key=lambda d: d["t_wall"])
    return out


def in_window(dets, t0, t1, offset):
    """Detections whose laptop time (t_wall - offset) is in [t0, t1]."""
    ts = [d["t_wall"] - offset for d in dets]
    import bisect
    i, j = bisect.bisect_left(ts, t0), bisect.bisect_right(ts, t1)
    return dets[i:j]


# ------------------------------------------------------------------ still windows
def still_windows(run, min_s=0.3, margin_s=0.15):
    """(which, t0, t1) laptop times when the robot sits still at its taped start
    pose (before its wheels first turn) and end pose (after they last turn)."""
    raw = [r for r in run.raw if fnum(r.get("lifted"), 0) < 0.5]
    if len(raw) < 3:
        return []
    moving = []
    for a, b in zip(raw, raw[1:]):
        dw = abs(fnum(b.get("wl_deg"), 0) - fnum(a.get("wl_deg"), 0)) + abs(fnum(b.get("wr_deg"), 0) - fnum(a.get("wr_deg"), 0))
        rpm = abs(fnum(b.get("wl_rpm"), 0)) + abs(fnum(b.get("wr_rpm"), 0))
        moving.append(dw > 0.7 or rpm > 6.0)    # rpm reads up to ~1 at rest; encoder steps are 0.36 deg
    if not any(moving):
        return []
    first = raw[moving.index(True)]["t"]
    last = raw[len(moving) - moving[::-1].index(True)]["t"]
    out = []
    if first - margin_s - raw[0]["t"] >= min_s:
        out.append(("start", raw[0]["t"], first - margin_s))
    if raw[-1]["t"] - (last + 2 * margin_s) >= min_s:
        out.append(("end", last + 2 * margin_s, raw[-1]["t"]))
    return out, first, last


def clusters(dets, radius=5.0):
    """Greedy clusters of image points: [(median xy, members)] biggest first."""
    pts = np.array([[d["x"], d["y"]] for d in dets], float)
    left = np.arange(len(pts))
    out = []
    while len(left):
        p = pts[left]
        d = np.linalg.norm(p[:, None, :] - p[None, :, :], axis=2)
        k = int(np.argmax((d <= radius).sum(1)))
        m = np.median(p[d[k] <= radius], 0)
        mem = left[np.linalg.norm(p - m, axis=1) <= radius]
        m = np.median(pts[mem], 0)
        out.append((m, [dets[i] for i in mem]))
        left = np.setdiff1d(left, mem)
    return out


def still_points(runs, dets, offset, robot_top_cm=4.0, led_offset_cm=None, log=print):
    """One point per still window of every tape-measured run: the tape pose (cm)
    paired with the robot's median detection (px). Returns (points, notes)."""
    points, notes = [], []
    for run in runs:
        if run.excluded:
            continue
        sw = still_windows(run)
        if not sw:
            notes.append(f"{run.run_id}: robot never moved or no RAW; skipped")
            continue
        wins, t_first, t_last = sw
        for which, t0, t1 in wins:
            pose = run.tape_start if which == "start" else run.tape_end
            if not pose:
                continue
            win = [d for d in in_window(dets, t0, t1, offset) if d["robot"] in (None, run.robot)]
            n_frames = max(1, len({d["frame"] for d in win}))
            cands = [(m, mem) for m, mem in clusters(win) if len({d["frame"] for d in mem}) >= 0.5 * n_frames] if win else []
            if not cands:
                notes.append(f"{run.run_id} {which}: no steady detection while the robot sat still; skipped")
                continue
            if len(cands) > 1:
                # the robot's own cluster is the one that empties when it moves (start) / was empty before it stopped (end)
                ta, tb = (t_first, t_first + 2.0) if which == "start" else (t_last - 2.0, t_last)
                after = in_window(dets, ta, tb, offset)
                nf = max(1, len({d["frame"] for d in after}))
                occ = [len({d["frame"] for d in after if math.hypot(d["x"] - m[0], d["y"] - m[1]) <= 4}) / nf
                       for m, _ in cands]
                order = np.argsort(occ)
                if occ[order[0]] > 0.5 or occ[order[1]] - occ[order[0]] < 0.3:
                    notes.append(f"{run.run_id} {which}: {len(cands)} still objects in view and none clearly "
                                 "left/arrived with the robot; skipped")
                    continue
                cands = [cands[order[0]]]
            m, mem = cands[0]
            x, y, h = pose
            kind = mem[0]["kind"]
            if kind == "led":
                if led_offset_cm:
                    a = math.radians(h)
                    fx, fy = led_offset_cm
                    x, y = x + fx * math.cos(a) - fy * math.sin(a), y + fx * math.sin(a) + fy * math.cos(a)
            spread = float(np.sqrt(np.mean([(d["x"] - m[0]) ** 2 + (d["y"] - m[1]) ** 2 for d in mem])))
            points.append(dict(kind="run", u=round(float(m[0]), 2), v=round(float(m[1]), 2), x=round(x, 3),
                               y=round(y, 3), z=robot_top_cm, frame=int(np.median([d["frame"] for d in mem])),
                               group=run.run_id, spot=f"{run.run_id}-{which}", enabled=True,
                               label=f"run {run.run_id} {which} ({run.robot}{', LED midpoint' if kind == 'led' else ''}; "
                                     f"{len(mem)} detections, spread {spread:.1f} px)"))
    if any("LED midpoint" in p["label"] for p in points) and not led_offset_cm:
        notes.append("WARNING: only blink (LED) detections were available and no --led-offset-cm was given: the LED "
                     "midpoint sits a few cm ahead of the robot's centre, so these points are biased along each "
                     "robot's heading. Give --led-offset-cm (tools/blink/calib.py measures it) or use detector output.")
    return points, notes


# ------------------------------------------------------------------ survey
def odometry_paths(runs, start_from_tape=True):
    """Wheel distance + gyro heading path for each run, started at its tape start
    pose (or at 0,0,0 facing +x if the run has none and start_from_tape is False).
    Returns run_id -> [(t, x, y, th_rad, sigma_cm)]."""
    import engine as eng       # tools/fusion/engine.py (read only)
    use = []
    for r in runs:
        if r.excluded:
            continue
        if not r.tape_start:
            if start_from_tape:
                continue
            r.tape_start = (0.0, 0.0, 0.0)
        use.append(r)
    if not use:
        return {}
    cfg = eng.config(**dict(ODO_CONFIG, start_from="tape"))
    tracked, _ = eng.run_engine(use, None, cfg)
    out = {}
    for rid, tr in tracked.items():
        out[rid] = [(t, x, y, th, math.hypot(sx, sy) / math.sqrt(2)) for t, x, y, th, sx, sy in tr.trajectory()]
    return out


SIGMA_CAM_CM = 0.5      # camera point noise on the floor (~1.5 px at ~3 px/cm)
TAPE_XY_CM = 0.3        # tape start pose: position error (midpoint of two marks measured to ~0.2 cm)
TAPE_HEADING_DEG = 1.5  # ... and heading error (two marks ~9 cm apart)
CLOCK_PRIOR_S = 0.1     # video clock offset (runs.video_offset) is refined within about this much
PAIR_COLS = ["u", "v", "x_cm", "y_cm", "vx_cms", "vy_cms", "sigma_cm", "run", "t", "frame", "kind",
             "anchor_x_cm", "anchor_y_cm", "session", "robot"]


def survey_pairs(runs, dets, offset, paths, axle_to_centre_cm=0.0, led_offset_cm=None, max_sigma_cm=3.0,
                 max_pairs_per_run=400, session=""):
    """Candidate pairs, one per detection while a run drives: dict with u, v (px),
    x_cm, y_cm (odometry path point the camera should see, floor cm), vx/vy (path
    velocity, cm/s, for refining the clock offset), sigma_cm (path uncertainty),
    run, t, frame, kind, anchor (the tape start the path hangs from), session.
    Several detections in one frame all become candidates; the fit keeps the right one."""
    out = []
    for run in runs:
        path = paths.get(run.run_id)
        if not path or not run.tape_start:
            continue
        lift = next((r["t"] for r in run.raw if fnum(r.get("lifted"), 0) > 0.5), None)
        t_end = min(path[-1][0], lift - 0.2) if lift else path[-1][0]
        P = np.array(path)
        ts = P[:, 0]
        th_u = np.unwrap(P[:, 3])
        win = [d for d in in_window(dets, path[0][0], t_end, offset) if d["robot"] in (None, run.robot)]
        rows = []
        for d in win:
            t = d["t_wall"] - offset
            sg = float(np.interp(t, ts, P[:, 4]))
            if sg > max_sigma_cm:
                continue
            x, y, th = (float(np.interp(t, ts, a)) for a in (P[:, 1], P[:, 2], th_u))
            fx, fy = axle_to_centre_cm, 0.0
            if d["kind"] == "led" and led_offset_cm:
                fx, fy = fx + led_offset_cm[0], led_offset_cm[1]
            x, y = x + fx * math.cos(th) - fy * math.sin(th), y + fx * math.sin(th) + fy * math.cos(th)
            dt = 0.04
            x2, y2 = (float(np.interp(t + dt, ts, a)) for a in (P[:, 1], P[:, 2]))
            x1, y1 = (float(np.interp(t - dt, ts, a)) for a in (P[:, 1], P[:, 2]))
            rows.append(dict(u=d["x"], v=d["y"], x_cm=x, y_cm=y, vx_cms=(x2 - x1) / (2 * dt), vy_cms=(y2 - y1) / (2 * dt),
                             sigma_cm=sg, run=run.run_id, t=t, frame=d["frame"], kind=d["kind"],
                             anchor_x_cm=run.tape_start[0], anchor_y_cm=run.tape_start[1], session=session,
                             robot=run.robot))
        if len(rows) > max_pairs_per_run:      # thin out evenly in time, keeping all detections of a kept frame
            frames = sorted({r["frame"] for r in rows})
            keep = set(frames[::max(1, int(math.ceil(len(frames) / max_pairs_per_run)))])
            rows = [r for r in rows if r["frame"] in keep]
        out.extend(rows)
    return out


def end_mark_pairs(runs, dets, offset, robot_top_cm=4.0, led_offset_cm=None, session=""):
    """Survey pairs from the runs' tape END marks (robot sitting still after the
    run, median detection): absolute floor points (kind 'tape_end'), not moved by
    the start-pose / clock refinement. Grouped by run like the path pairs, so in
    the held-out calibrations a run's own end marks never calibrate the camera
    that scores it."""
    pts, notes = still_points(runs, dets, offset, robot_top_cm, led_offset_cm)
    robot_of = {r.run_id: r.robot for r in runs}
    out = []
    for p in pts:
        if not p["spot"].endswith("-end"):
            continue
        out.append(dict(u=p["u"], v=p["v"], x_cm=p["x"], y_cm=p["y"], vx_cms=0.0, vy_cms=0.0, sigma_cm=TAPE_XY_CM,
                        run=p["group"], t=0.0, frame=p["frame"], kind="tape_end", anchor_x_cm=p["x"],
                        anchor_y_cm=p["y"], session=session, robot=robot_of.get(p["group"], "")))
    return out, notes


def write_pairs(path, pairs):
    """Accumulate survey pairs across sessions: rows of runs in `pairs` replace earlier rows of those runs."""
    old = []
    if os.path.isfile(path):
        new_runs = {p["run"] for p in pairs}
        with open(path, newline="", encoding="utf-8-sig") as f:
            old = [r for r in csv.DictReader(f) if r["run"] not in new_runs]
    with open(path, "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=PAIR_COLS, extrasaction="ignore")
        wr.writeheader()
        wr.writerows(old)
        for p in pairs:
            wr.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in p.items()})


def read_pairs(path):
    out = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            d = dict(r)
            d["robot"] = (d.get("robot") or "").strip()     # files before 2026-10-18 have no robot column
            for k in ("u", "v", "x_cm", "y_cm", "vx_cms", "vy_cms", "sigma_cm", "t", "anchor_x_cm", "anchor_y_cm"):
                d[k] = float(d[k])
            d["frame"] = int(float(d["frame"]))
            out.append(d)
    return out


def _norm(p):
    m = p.mean(0)
    s = math.sqrt(2) / max(np.mean(np.linalg.norm(p - m, axis=1)), 1e-9)
    return np.array([[s, 0, -s * m[0]], [0, s, -s * m[1]], [0, 0, 1.0]])


def fit_g_weighted(px, fl, w):
    """Image px -> floor cm homography by weighted DLT (normalised coordinates)."""
    T1, T2 = _norm(px), _norm(fl)
    a = (np.c_[px, np.ones(len(px))] @ T1.T)[:, :2]
    b = (np.c_[fl, np.ones(len(fl))] @ T2.T)[:, :2]
    sw = np.sqrt(w)[:, None]
    z = np.zeros((len(a), 3))
    A1 = np.c_[a, np.ones(len(a)), z, -b[:, :1] * a, -b[:, :1]]
    A2 = np.c_[z, a, np.ones(len(a)), -b[:, 1:] * a, -b[:, 1:]]
    _, _, Vt = np.linalg.svd(np.vstack([A1 * sw, A2 * sw]), full_matrices=False)
    G = np.linalg.inv(T2) @ Vt[-1].reshape(3, 3) @ T1
    return G / G[2, 2]


def _lm(fun, p0, iters=30):
    """Levenberg-Marquardt with a numerical Jacobian; fun(p) -> residual vector."""
    p = np.array(p0, float)
    r = fun(p)
    lam = 1e-3
    for _ in range(iters):
        J = np.empty((len(r), len(p)))
        for k in range(len(p)):
            dp = np.zeros(len(p))
            dp[k] = 1e-6
            J[:, k] = (fun(p + dp) - r) / 1e-6
        A, g = J.T @ J, J.T @ r
        improved = False
        for _ in range(8):
            step = -np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), g)
            r2 = fun(p + step)
            if r2 @ r2 < r @ r:
                p, r, lam, improved = p + step, r2, lam / 3, True
                break
            lam *= 5
        if not improved or np.max(np.abs(step)) < 1e-9:
            break
    return p, r


def fit_survey(pairs, size, height_cm=None, nadir=None, f_px=None, k1_mode="off", area=None, robot_top_cm=4.0,
               ransac_cm=3.0, reject_sigma=3.5, refine=True, exclude_runs=()):
    """Fit the floor <-> image mapping to survey pairs. Returns a result dict like
    calibrate_floor.fit (points summarised per run under 'survey').

    After a first weighted fit (RANSAC against wrong detections, one detection per
    frame), a joint refinement also lets each run's tape start pose move within the
    tape's accuracy (TAPE_XY_CM, TAPE_HEADING_DEG: the path hangs from it, so a 1.5 deg
    heading error swings the far end) and the video clock offset move (CLOCK_PRIOR_S;
    the flash match is good to about a frame), one per session.

    height_cm: camera height (tape). Without it but with f_px (focal length from an
    earlier calibration of this webcam; it doesn't change when the camera is moved)
    the height is estimated as f_px / (px per cm on the robot-top plane) + robot top.
    With neither, the homography maps the robot-top plane (plane_height_cm)."""
    import cv2
    import calibrate_floor as cf
    pairs = [p for p in pairs if p["run"] not in set(exclude_runs)]
    if len(pairs) < 12:
        raise cf.FitError(f"only {len(pairs)} survey pairs (camera track while driving); need dozens")
    W, Hh = size
    c = np.array([W / 2.0, Hh / 2.0])
    px = np.array([[p["u"], p["v"]] for p in pairs], float)
    fl0 = np.array([[p["x_cm"], p["y_cm"]] for p in pairs], float)
    vel = np.array([[p["vx_cms"], p["vy_cms"]] for p in pairs], float)
    anc = np.array([[p["anchor_x_cm"], p["anchor_y_cm"]] for p in pairs], float)
    sig = np.sqrt(np.array([p["sigma_cm"] for p in pairs], float) ** 2 + SIGMA_CAM_CM ** 2)
    w = 1 / sig ** 2
    run_ids = np.array([p["run"] for p in pairs])
    runs_u = sorted(set(run_ids))
    ri = np.array([runs_u.index(r) for r in run_ids])
    sess = np.array([str(p.get("session", "")) for p in pairs])
    sess_u = sorted(set(sess))
    si = np.array([sess_u.index(s) for s in sess])
    absolute = np.array([p.get("kind") == "tape_end" for p in pairs])   # tape end marks: not moved by refinement
    frame_key = np.array([f"{p['run']}:{p['frame']}" + (":end" if a else "") for p, a in zip(pairs, absolute)])
    z = robot_top_cm
    warnings = []
    hc, hc_src = height_cm, ("tape (given)" if height_cm else None)
    nad = np.asarray(nadir, float) if nadir is not None else None
    nad_src = "plumb line (given)" if nadir is not None else None
    k1 = float(k1_mode) if k1_mode not in ("auto", "off", "fit", None, "") else 0.0
    f = f_px or float(W)
    corr = np.zeros(3 * len(runs_u) + len(sess_u))   # per run dx, dy, dth (rad); per session dt (s)

    def targets(corr):
        d = corr[:3 * len(runs_u)].reshape(-1, 3)[ri] * (~absolute)[:, None]
        dt = corr[3 * len(runs_u):][si] * (~absolute)
        q = fl0 + vel * dt[:, None] - anc
        cs, sn = np.cos(d[:, 2]), np.sin(d[:, 2])
        return anc + np.c_[cs * q[:, 0] - sn * q[:, 1], sn * q[:, 0] + cs * q[:, 1]] + d[:, :2]

    def eqs(tg, hc, nad):
        return cf.raise_to_floor_equiv(tg, z, hc, nad) if (hc and nad is not None) else tg

    def one_per_frame(r, ok):
        best = {}
        for i in np.where(ok)[0]:
            if frame_key[i] not in best or r[i] < r[best[frame_key[i]]]:
                best[frame_key[i]] = i
        m = np.zeros(len(r), bool)
        m[list(best.values())] = True
        return m

    def initial(k1, hc, nad, corr):
        und = cf.undistort(px, k1, f, c)
        eq = eqs(targets(corr), hc, nad)
        cv2.setRNGSeed(7)
        _, m = cv2.findHomography(und, eq, cv2.RANSAC, ransac_cm, maxIters=5000, confidence=0.999)
        if m is None or m.sum() < 12:
            raise cf.FitError("RANSAC found no consistent mapping: is the video offset right? Was the robot seen?")
        mask = m.ravel().astype(bool)
        for _ in range(4):
            G = fit_g_weighted(und[mask], eq[mask], w[mask])
            r = np.linalg.norm(cf.apply_h(G, und) - eq, axis=1)
            new = one_per_frame(r, r / sig <= reject_sigma)
            if new.sum() < 12:
                raise cf.FitError("too few survey pairs agree with each other")
            if np.array_equal(new, mask):
                break
            mask = new
        return fit_g_weighted(und[mask], eq[mask], w[mask]), mask

    G, mask = initial(k1, None, None, corr)
    if k1_mode in ("auto", "fit"):
        def cost(k):
            und = cf.undistort(px[mask], k, f, c)
            Gk = fit_g_weighted(und, fl0[mask], w[mask])
            r = np.linalg.norm(cf.apply_h(Gk, und) - fl0[mask], axis=1)
            return float(np.sum(w[mask] * r ** 2) / np.sum(w[mask]))
        ks = np.round(np.arange(-0.3, 0.1501, 0.01), 3)
        cs_ = [cost(k) for k in ks]
        kb = float(ks[int(np.argmin(cs_))])
        if k1_mode == "fit" or min(cs_) < 0.85 * cost(0.0):
            k1 = kb
        G, mask = initial(k1, None, None, corr)
    T1 = _norm(px[mask])
    for outer in range(4):
        H = np.linalg.inv(G)
        H /= H[2, 2]
        if not height_cm and f_px:
            s = cf.scale_at(H, c)
            hc = f_px / s + (z if hc is None else 0.0)    # first pass: H maps the robot-top plane
            hc_src = "estimated from the webcam's focal length (f_px) and the fit"
        if hc and nadir is None:
            nad, nad_src = cf.estimate_nadir(H, size, hc)
            nad_src = "estimated: " + nad_src
        und = cf.undistort(px, k1, f, c)
        if refine:
            G0 = G.copy()
            T1i = np.linalg.inv(T1)

            def G_of(p):
                D = np.r_[p[:8], 0.0].reshape(3, 3)
                Gp = G0 @ T1i @ (np.eye(3) + D) @ T1
                return Gp / Gp[2, 2]

            prior = np.r_[np.tile([TAPE_XY_CM, TAPE_XY_CM, math.radians(TAPE_HEADING_DEG)], len(runs_u)),
                          np.full(len(sess_u), CLOCK_PRIOR_S)]
            m_idx = np.where(mask)[0]
            sw = np.sqrt(w[m_idx])

            def resid(p):
                tg = eqs(targets(p[8:])[m_idx], hc, nad)
                r = (cf.apply_h(G_of(p), und[m_idx]) - tg) * sw[:, None]
                return np.r_[r.ravel(), p[8:] / prior]

            p, _ = _lm(resid, np.r_[np.zeros(8), corr])
            G, corr = G_of(p), p[8:]
        eq = eqs(targets(corr), hc, nad)
        r = np.linalg.norm(cf.apply_h(G, und) - eq, axis=1)
        new = one_per_frame(r, r / sig <= reject_sigma)
        if new.sum() < 12:
            raise cf.FitError("too few survey pairs agree with each other")
        if not refine:
            G = fit_g_weighted(und[new], eq[new], w[new])
        if np.array_equal(new, mask) and outer >= 1 and (height_cm or not f_px):
            mask = new
            break
        mask = new
    H = np.linalg.inv(G)
    H /= H[2, 2]
    if not hc:
        warnings.append(f"no camera height (and no f_px to estimate it): this homography maps the plane "
                        f"{z:g} cm up (the robot's top), so robot positions are right but floor marks are "
                        "about 3.5% too far from the image centre.")
    tg = targets(corr)
    seen = cf.apply_h(G, cf.undistort(px, k1, f, c))
    if hc:
        seen = cf.lower_from_floor_equiv(seen, z, hc, nad)
    err = np.linalg.norm(seen - tg, axis=1)
    per_run = {}
    cr = corr[:3 * len(runs_u)].reshape(-1, 3)
    for k, rid in enumerate(runs_u):
        m = mask & (run_ids == rid)
        per_run[str(rid)] = dict(pairs=int((run_ids == rid).sum()), used=int(m.sum()),
                                 rms_cm=round(cf._rms(err[m]), 3) if m.any() else None,
                                 start_shift_cm=[round(float(cr[k, 0]), 2), round(float(cr[k, 1]), 2)],
                                 start_turn_deg=round(math.degrees(cr[k, 2]), 2),
                                 end_mark_err_cm=(round(float(err[(run_ids == rid) & absolute][0]), 3)
                                                  if ((run_ids == rid) & absolute).any() else None))
    path = mask & ~absolute
    s_used = sig[path] if path.any() else sig[mask]
    early = path & (sig <= np.median(s_used))
    late = path & (sig > np.median(s_used))
    area = list(area or cf.default_area())
    cov, far = cf.coverage(tg[mask], area)
    if cov < 0.5:
        warnings.append(f"the survey path covers only {cov * 100:.0f}% of the area {area}: the mapping is "
                        "extrapolated elsewhere. Drive a pattern that crosses the area.")
    if mask.mean() < 0.5:
        warnings.append(f"only {mask.mean() * 100:.0f}% of the candidate pairs fit: check the video offset, "
                        "and whether other robots or hands were in view")
    if len(runs_u) < 3:
        warnings.append(f"only {len(runs_u)} run(s): the whole mapping hangs on that tape start's heading "
                        f"(about +/-{TAPE_HEADING_DEG} deg, i.e. {math.radians(TAPE_HEADING_DEG) * 100:.1f} cm at 1 m). "
                        "More runs from separately measured starts average it out.")
    return dict(H=H, k1=float(k1), f_px=float(f), cx=float(c[0]), cy=float(c[1]), size=[int(W), int(Hh)],
                height_cm=hc, height_source=hc_src, plane_height_cm=0.0 if hc else z,
                nadir=None if nad is None else [float(nad[0]), float(nad[1])], nadir_source=nad_src,
                points=[], n_points=len(pairs), n_inliers=int(mask.sum()),
                rms_cm=cf._rms(err[mask]), max_cm=float(err[mask].max()), loo_rms_cm=float("nan"),
                coverage=cov, outside_cm=far, area=area, warnings=warnings, k1_info={}, k1_ignored_max_cm=None,
                survey=dict(runs=per_run, sessions={s: round(float(corr[3 * len(runs_u) + k]) * 1000, 1)
                                                    for k, s in enumerate(sess_u)},
                            weighted_rms_cm=round(float(np.sqrt(np.sum(w[mask] * err[mask] ** 2) / np.sum(w[mask]))), 3),
                            rms_early_cm=round(cf._rms(err[early]), 3), rms_late_cm=round(cf._rms(err[late]), 3),
                            path_sigma_cm_median=round(float(np.median(s_used)), 3), refined=bool(refine),
                            end_marks_used=int((mask & absolute).sum())))


def survey_error(res, pairs, run, kind=None):
    """A run's pairs under a calibration that did not use them (held-out check):
    median and RMS floor error (cm) of its path pairs (or kind='tape_end': its end
    mark) after one-per-frame, ignoring > 10 cm."""
    import calibrate_floor as cf
    cal = cf.to_calibration(res)
    e = {}
    for p in pairs:
        if p["run"] != run or (p.get("kind") == "tape_end") != (kind == "tape_end"):
            continue
        x, y = cf.to_floor(cal, p["u"], p["v"], 4.0 if res.get("height_cm") else 0.0)
        d = math.hypot(x - p["x_cm"], y - p["y_cm"])
        if p["frame"] not in e or d < e[p["frame"]]:
            e[p["frame"]] = d
    a = np.array([v for v in e.values() if v < 10])
    if not len(a):
        return None, None
    return float(np.median(a)), float(np.sqrt(np.mean(a ** 2)))
