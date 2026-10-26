"""Explore (Nick, 2026-10-23): the robots calibrate the camera and map the board together, from scratch.

What the system knows at the start: the floor is flat; each robot's own size and sensor positions; a camera is
somewhere above, looking down. Not the board's size, its edges, where the boards or the camera are, nor each robot's
own corrections (wheel scale, gyro scale).

How it is solved (the camera and the robots calibrate each other):
  - every stop of every robot is a pair: where the camera sees it (pixels) and where its own wheels and gyro say it
    is (in its own frame, from wherever it was switched on);
  - a robot's path is rebuilt from its legs with two unknown corrections: its wheel scale s (distances) and its
    gyro scale g (turns). A camera keeps straight lines straight, so a "straight" path that bends, or a square that
    does not close in the picture, shows g; the shared distance scale is the robots' wheels averaged (sum of
    log s = 0), cross-checked by the robots' known size in the picture;
  - each robot's own frame sits somewhere on the floor (unknown x, y, turn), except the first robot's, which is the
    world frame while exploring;
  - the camera is a pinhole (focal length, one lens-bend term, where it is and how it is turned), the principal
    point at the picture's centre;
  - all of it is fitted at once to the pixels (least squares, robust to a few bad stops), started from one
    homography per robot (its own path against its pixels) put together.
Then the border: each ray's end (a board met by the front sensor, or an open edge by the edge sensors) is a point
in the world frame; four straight sides are fitted to them (a rectangle), each side's kind by its points' majority,
and the frame is turned so the corner of the two open edges is (0, 0) and the boards are at x = W and y = H (the
runner's board convention). The camera model is re-expressed in that frame.

Pure functions; the driving is in e02.py (Runner.explore). Unit-tested in tests/test_e02_explore.py.
"""
import math

import cv2
import numpy as np
from scipy.optimize import least_squares

ROBOT_TOP_CM = 3.7          # the height the camera sees a robot at (its top)
ROBOT_CM = 9.5              # a robot's width (its known size: the scale cross-check)
F_PRIOR_HFOV_DEG = 60.0     # a typical webcam's horizontal field of view (a weak prior on the focal length)
F_PRIOR_SD_LOG = 0.5        # ... loose: about x/1.6
K1_PRIOR_SD = 0.15          # the lens-bend term near 0
SIDE_TOL_CM = 3.0           # a contact point within this of a side's line belongs to it


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------------------------------------- robot paths
def segments(stops):
    """From a robot's stops (each with its own pose x, y, th in its own frame), the legs between them: (signed
    distance along the heading, heading change in deg, the heading at the leg's start, deg)."""
    out = []
    for a, b in zip(stops[:-1], stops[1:]):
        if "dth" in b:
            # the leg's own record (firmware idle legs report their pose from the leg's start: x forward, th turned)
            out.append((b["d"], b["dth"]))
            continue
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        dth = wrap(b["th"] - a["th"])
        mid = math.radians(a["th"] + dth / 2.0)
        d = dx * math.cos(mid) + dy * math.sin(mid)       # signed: a backwards drive is negative
        out.append((d, dth))
    return out


def rebuild(stops, s=1.0, g=1.0, dmap=None, thmap=None):
    """The path (x, y, th) of the stops in the robot's own frame, its distances x s and turns x g. dmap / thmap:
    {stop index: distance / turn} replacing those legs' own (the fit's free legs: repair_legs, lost records)."""
    x, y, th = (0.0, 0.0, 0.0) if "dth" in stops[0] else (stops[0]["x"], stops[0]["y"], stops[0]["th"])
    out = [(x, y, th)]
    for k, (d, dth) in enumerate(segments(stops)):
        if dmap and (k + 1) in dmap:
            d = dmap[k + 1]
        if thmap and (k + 1) in thmap:
            dth = thmap[k + 1]
        mid = math.radians(th + g * dth / 2.0)
        x += s * d * math.cos(mid)
        y += s * d * math.sin(mid)
        th = th + g * dth
        out.append((x, y, th))
    return out


def to_world(path, frame):
    """A path in a robot's own frame into the world frame: frame = (x0, y0, turn deg)."""
    c, sn = math.cos(math.radians(frame[2])), math.sin(math.radians(frame[2]))
    return [(frame[0] + c * x - sn * y, frame[1] + sn * x + c * y, th + frame[2]) for x, y, th in path]


# ------------------------------------------------------------------------------------------------- the camera
def K_of(f, size):
    return np.array([[f, 0, size[0] / 2.0], [0, f, size[1] / 2.0], [0, 0, 1.0]])


def project(pts_xyz, f, k1, rvec, tvec, size):
    uv, _ = cv2.projectPoints(np.asarray(pts_xyz, float).reshape(-1, 1, 3), np.asarray(rvec, float),
                              np.asarray(tvec, float), K_of(f, size), np.array([k1, 0, 0, 0, 0], float))
    return uv.reshape(-1, 2)


def _init(robots, size):
    """Start values: each robot's path (s = g = 1) against its pixels by a homography; the robots' frames put
    together through the first robot's homography (a rigid fit with scale); the camera from the first robot's
    points by solvePnP over a range of focal lengths."""
    names = list(robots)
    H, G, spread = {}, {}, {}
    for n in names:
        st = robots[n]
        uv = np.array([(q["u"], q["v"]) for q in st], float)
        if len(st) < 4:
            continue
        # the gyro scale first, by a search: the path that a homography fits best (2026-10-23 dry run: half turns
        # 8% off threw every later stop off and the joint fit started too far away)
        best = None
        for g in np.arange(0.80, 1.25, 0.005):
            p = np.array([(q[0], q[1]) for q in rebuild(st, 1.0, g)], float)
            ok = np.array([not q.get("skip") for q in st])
            if ok.sum() < 4:
                continue
            Hn, _ = cv2.findHomography(p[ok], uv[ok], cv2.RANSAC, 8.0)
            if Hn is None:
                continue
            e = cv2.perspectiveTransform(p[ok].reshape(-1, 1, 2), Hn).reshape(-1, 2) - uv[ok]
            r = float(np.median(np.sum(e ** 2, 1)))
            if best is None or r < best[0]:
                best = (r, g, Hn)
        if best is not None:
            H[n], G[n] = best[2], best[1]
            spread[n] = (int(ok.sum()), -best[0])
    if not H:
        raise ValueError("no robot has 4 stops the camera saw")
    # the reference: the robot with the most stops the camera saw (2026-10-23: Left, whose tracker had stayed
    # behind, made a poor reference for the others)
    ref = max(H, key=lambda n: spread[n])
    Hinv = np.linalg.inv(H[ref])
    frames, scales = {ref: (0.0, 0.0, 0.0)}, {ref: 1.0}
    for n in names:
        if n == ref:
            continue
        st = robots[n]
        p = np.array([(q[0], q[1]) for q in rebuild(st, 1.0, G.get(n, 1.0))], float)
        uv = np.array([(q["u"], q["v"]) for q in st], float)
        ok = np.array([not q.get("skip") for q in st])
        p, uv = p[ok], uv[ok]
        w = cv2.perspectiveTransform(uv.reshape(-1, 1, 2), Hinv).reshape(-1, 2)   # its pixels in the ref frame
        # w = s R p + T (Umeyama)
        mp, mw = p.mean(0), w.mean(0)
        P, W = p - mp, w - mw
        U, S, Vt = np.linalg.svd(W.T @ P)
        D = np.eye(2)
        if np.linalg.det(U @ Vt) < 0:
            D[1, 1] = -1
        R = U @ D @ Vt
        sc = float(np.trace(np.diag(S) @ D) / max(1e-9, (P ** 2).sum()))
        T = mw - sc * R @ mp
        frames[n] = (float(T[0]), float(T[1]), math.degrees(math.atan2(R[1, 0], R[0, 0])))
        scales[n] = sc
    return _pnp(robots, frames, scales, G, size)


def _pnp(robots, frames, scales, G, size):
    """From the robots' frames and wheel scales (any shared size): the shared scale set by the wheels' average,
    then the camera by solvePnP over a range of focal lengths."""
    names = list(robots)
    # the shared distance scale: the wheels averaged
    gm = math.exp(np.mean([math.log(max(1e-6, v)) for v in scales.values()]))
    scales = {n: v / gm for n, v in scales.items()}
    frames = {n: (fr[0] / gm, fr[1] / gm, fr[2]) for n, fr in frames.items()}
    # the camera: world points of all robots at robot-top height
    obj, img = [], []
    for n in names:
        for (x, y, _), q in zip(to_world(rebuild(robots[n], scales[n], G.get(n, 1.0)), frames[n]), robots[n]):
            if q.get("skip"):
                continue
            obj.append((x / 1.0, y / 1.0, ROBOT_TOP_CM))
            img.append((q["u"], q["v"]))
    obj, img = np.array(obj, float), np.array(img, float)
    best = None
    for f in np.geomspace(0.4 * size[0], 3.0 * size[0], 25):
        for flip in (False, True):
            o = obj.copy()
            if flip:
                continue
            ok, rv, tv, _inl = cv2.solvePnPRansac(o, img, K_of(f, size), None, reprojectionError=8.0)
            if not ok:
                continue
            uv = project(o, f, 0.0, rv.ravel(), tv.ravel(), size)
            err = float(np.sqrt(np.mean(np.sum((uv - img) ** 2, 1))))
            R = cv2.Rodrigues(rv)[0]
            cz = (-R.T @ tv.ravel())[2]
            if cz <= ROBOT_TOP_CM:
                continue                                  # the camera must be above the floor
            if best is None or err < best[0]:
                best = (err, f, rv.ravel(), tv.ravel())
    if best is None:
        raise ValueError("no camera pose puts the camera above the floor")
    return frames, scales, best, G


def _init_prov(robots, size, h_cm=260.0, f_px=None):
    """Start values without any one robot's homography (2026-10-23 real board: the reference robot's few stops,
    nearly in a line, extrapolated badly to the others): the pixels laid on the floor as a camera straight overhead
    would see them (a near-similarity of the truth for a camera looking roughly down), each robot's path (its
    gyro scale from robot_check) fitted to that by a rigid fit with scale. Tries both handednesses."""
    f_px = f_px or float(size[0])
    names = list(robots)
    best = None
    for flip in (1.0, -1.0):
        frames, scales, G, err = {}, {}, {}, 0.0
        for n in names:
            st = implausible(robots[n])
            ok = np.array([not q.get("skip") for q in st])
            if ok.sum() < 4:
                continue
            g = robot_check(robots[n]).get("g") or 1.0
            p = np.array([(q[0], q[1]) for q in rebuild(st, 1.0, g)], float)[ok]
            uv = np.array([(q["u"], q["v"]) for q in st], float)[ok]
            w = np.column_stack([(uv[:, 0] - size[0] / 2) * h_cm / f_px, flip * (uv[:, 1] - size[1] / 2) * h_cm / f_px])
            mp, mw = p.mean(0), w.mean(0)
            P, Wc = p - mp, w - mw
            if (P ** 2).sum() < 1e-6:
                continue
            U, S, Vt = np.linalg.svd(Wc.T @ P)
            D = np.eye(2)
            if np.linalg.det(U @ Vt) < 0:
                D[1, 1] = -1
            R = U @ D @ Vt
            sc = float(np.trace(np.diag(S) @ D) / (P ** 2).sum())
            T = mw - sc * R @ mp
            err += float(np.sum(np.linalg.norm((sc * (R @ P.T)).T + mw - w, axis=1) ** 2))
            frames[n] = (float(T[0]), float(T[1]), math.degrees(math.atan2(R[1, 0], R[0, 0])))
            scales[n], G[n] = sc, g
        if len(frames) == len(names) and (best is None or err < best[0]):
            best = (err, frames, scales, G)
    if best is None:
        raise ValueError("too few stops for a start")
    _, frames, scales, G = best
    ref = names[0]
    # the first robot's frame as the world frame: move everything into it
    x0, y0, a0 = frames[ref]
    s0 = scales[ref]
    c, sn = math.cos(math.radians(-a0)), math.sin(math.radians(-a0))
    fr2 = {}
    for n, (x, y, a) in frames.items():
        dx, dy = (x - x0) / s0, (y - y0) / s0
        fr2[n] = (c * dx - sn * dy, sn * dx + c * dy, a - a0) if n != ref else (0.0, 0.0, 0.0)
    sc2 = {n: v / s0 for n, v in scales.items()}
    return _pnp(robots, fr2, sc2, G, size)


MIN_PX_PER_CM = 0.4      # chosen: a camera 6x higher than any we would use still shows more than this
STILL_JUMP_PX = 25.0     # chosen: a robot that only turned on its spot moves its top centre a few pixels


def implausible(stops):
    """A copy of stops with skip set where the camera's spot cannot be the robot: against the last stop still
    trusted, the robot's own wheels and gyro moved it 8 cm or more and the spot moved under MIN_PX_PER_CM per cm
    (the tracker stayed behind: 2026-10-23 dry run, Left for 19 stops), or they moved it under 2 cm and the spot
    jumped more than STILL_JUMP_PX (the tracker went to something else)."""
    path = rebuild(stops, 1.0, 1.0)
    out, last = [], None
    for i, q in enumerate(stops):
        q2 = dict(q)
        if q.get("skip"):
            out.append(q2)
            continue
        if last is not None:
            own = math.hypot(path[i][0] - path[last][0], path[i][1] - path[last][1])
            moved = math.hypot(q["u"] - stops[last]["u"], q["v"] - stops[last]["v"])
            if (own >= 8.0 and moved < MIN_PX_PER_CM * own) or (own < 2.0 and moved > STILL_JUMP_PX):
                q2["skip"] = True
                out.append(q2)
                continue
        last = i
        out.append(q2)
    return out


def fit(robots, size, g_fixed=None, outlier_px=10.0):
    """fit_once, then once more without the stops the camera clearly got wrong (the tracker on something else):
    a residual over max(outlier_px, 4 x the median). Those stops are marked skip (their legs still count for the
    path).

    First, before any fit: stops whose pixels cannot be the robot given its own wheels and gyro (implausible())."""
    robots = {n: implausible(v) for n, v in robots.items()}
    pre = sum(1 for v in robots.values() for q in v if q.get("skip"))
    sol = best_fit(robots, size, g_fixed)
    rep = 0
    if REPAIR_LEGS:
        for _ in range(REPAIR_ROUNDS):
            robots, k = repair_legs(robots, sol)
            if not k:
                break
            rep += k
            sol = best_fit(robots, size, g_fixed)
            robots = _take_free_d(robots, sol)
    sol["implausible"] = pre
    sol["repaired"] = rep
    res = sol["resid"]
    allr = np.concatenate([res[n] for n in res]) if res else np.array([])
    if not len(allr):
        return sol
    lim = max(outlier_px, 4.0 * float(np.median(allr)))
    n_out = 0
    robots2 = {}
    for n, stops in robots.items():
        st2 = []
        for q, r in zip(stops, res[n]):
            q2 = dict(q)
            if r > lim and not q.get("skip"):
                q2["skip"] = True
                n_out += 1
            st2.append(q2)
        robots2[n] = st2
    sol["repairs"] = repairs_of(_take_free_d(robots, sol))
    if not n_out:
        sol["outliers"] = pre
        sol["skipped"] = {n: [i for i, q in enumerate(v) if q.get("skip")] for n, v in robots.items()}
        return sol
    sol2 = best_fit(robots2, size, g_fixed)
    sol2["implausible"] = pre
    sol2["repaired"] = rep
    sol2["outliers"] = n_out + pre
    sol2["repairs"] = repairs_of(_take_free_d(robots2, sol2))
    sol2["skipped"] = {n: [i for i, q in enumerate(v) if q.get("skip")] for n, v in robots2.items()}
    return sol2


# Legs whose wheels turned while the body did not follow (explore reliability, 2026-10-26). In the simulator batch a
# creep into a board at a slant (the bumper's corner touches first; the wheels keep turning: 2.9 cm counted, 0.4 cm
# moved, 4 times in a row) and a robot hanging at an open edge (26.4 cm counted, 17.3 moved) put up to 20 cm of
# distance into one robot's path that never happened. One such robot pushed its wheel scale to the fit's bound and the
# camera's scale 12 % off (seed 49). The fitted camera measures each straight leg between two trusted stops; a leg the
# camera saw much shorter than its wheels is given the camera's length, and the fit is run again.
REPAIR_LEGS = True
REPAIR_MIN_CM = 1.5         # chosen: more than the camera's own noise between two stops (about 1 px, 0.4-0.5 cm, each).
                            # DOE note: yes, simulator: 1 / 1.5 / 2.5 cm against the true leg errors (analysis
                            # tools/experiment/analyses_oct26/explore_reliability/legcheck.py), count false repairs
REPAIR_MAX_TURN_DEG = 10.0  # chosen: a straight leg that a hang twisted (seed 49: 3.6 deg) is still a straight leg; a
                            # turn leg is 45 deg or more. No DOE: anything between the two works
REPAIR_ABS_CM = 8.0         # chosen: ... or this many cm short and under REPAIR_ABS_SHARE of it. 0.93 caught a hang's
REPAIR_ABS_SHARE = 0.85     # 10 cm on a 113 cm ray in the synthetic test, but on the after batch (r07) it also took two
                            # good 105 cm rays the first camera fit saw 5-10 % short, and the fit then shrank them to
                            # 45 cm. DOE note: yes, the same simulator row as REPAIR_MIN_CM: false repairs of good long
                            # legs against overcounts caught, over the batch's recorded stops
REPAIR_ROUNDS = 2           # chosen: a loop guard (a repaired leg lets the next fit see the next one); no DOE
REPAIR_SHARE = 0.75         # chosen: the camera saw less than this share of the leg (a 2.9 cm creep that moved 0.4 cm,
                            # a hang 65 %); real wheel and camera scales agree within about 5 %. Same DOE row


REPAIR_NEXT_LONG = 1.25    # chosen: the next straight leg seen outside REPAIR_SHARE..this of its wheels' count: the
                           # spot, not the wheels (same DOE row as REPAIR_MIN_CM)
REPAIR_RAY_CM = 100.0       # a straight leg this long ended by its length, not by a side: a ray that met nothing (105 cm)


def suspect_leg(q):
    """Could this leg's wheels have turned without the body following? A leg that ended against something (a
    contact: a hang at an edge, a push at a board), a ray that drove its whole length and met nothing (a slanted
    bumper against a board or a robot, the front sensor seeing past it), a creep step (3 cm or less), or a leg whose
    record was lost. Returns and plain drives over ground just crossed are not (after-batch run r07: the camera's spot
    left behind at two ray ends made 105 cm rays look 45 cm, and repairing them put the camera 12 % small; a wrong
    spot is the outlier pass's job, not this one's)."""
    d = abs(q.get("d") or 0.0)
    return bool(q.get("contact") or q.get("ray") or q.get("lost") or q.get("stalled") or d >= REPAIR_RAY_CM or d <= 3.5)


def repair_legs(robots, sol, min_cm=None, share=None):
    """A copy of robots with each straight leg (turned under REPAIR_MAX_TURN_DEG, 2 cm or more) that the fitted camera saw clearly
    shorter than the wheels counted given the camera's length (d_own keeps the robot's own). "Shorter" is against
    the robot's own other legs (the median of camera length / wheel length over its straight legs of 5 cm or more),
    not the fit's wheel scale: a robot with overcounted legs has its fitted scale pulled by them (seed 49: 0.819, at
    the bound), and a camera fit pulled by them is off in size, which would hide the very legs that pulled it.
    Returns (robots, how many legs)."""
    min_cm = REPAIR_MIN_CM if min_cm is None else min_cm
    share = REPAIR_SHARE if share is None else share
    out, k = {}, 0
    for n, stops in robots.items():
        st2 = [dict(q) for q in stops]
        legs = []
        for i in range(1, len(st2)):
            a, b = st2[i - 1], st2[i]
            if a.get("skip") or b.get("skip") or "dth" not in b or b.get("u") is None or a.get("u") is None:
                continue
            d = abs(b.get("d") or 0.0)
            if (d < 2.0 and "d_own" not in b) or abs(b["dth"]) > REPAIR_MAX_TURN_DEG:
                continue
            pa, pb = world_of(sol, a["u"], a["v"]), world_of(sol, b["u"], b["v"])
            cam = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
            if np.isfinite(cam):
                legs.append((i, d, cam))
        long_ = [c / d for i, d, c in legs if d >= 5.0 and "d_own" not in st2[i]]
        m = float(np.median(long_)) if len(long_) >= 3 else 1.0
        for k_, (i, d, cam) in enumerate(legs):
            if "d_own" in st2[i]:
                continue                      # already free in the fit (an earlier round)
            if not suspect_leg(st2[i]):
                continue
            # a wrong camera spot at this stop (not the wheels) makes the next straight leg wrong too: long when the
            # robot drives on, short when it comes back the way it came (after-batch r07: two 105 cm rays, out and
            # back, both seen as 45 cm: the spot stayed 60 cm short of the ray's end). Wheels that overcounted leave
            # the next leg as the wheels say. Creep steps are exempt (a stalled creep is followed by more)
            nxt = None
            for j in range(i + 1, len(st2)):
                qj, pj = st2[j], st2[j - 1]
                if abs(qj.get("d") or 0.0) >= 5.0 and abs(qj.get("dth") or 0.0) <= REPAIR_MAX_TURN_DEG and \
                        qj.get("u") is not None and pj.get("u") is not None:
                    p1, p2 = world_of(sol, pj["u"], pj["v"]), world_of(sol, qj["u"], qj["v"])
                    nxt = (abs(qj["d"]), math.hypot(p2[0] - p1[0], p2[1] - p1[1]))
                    break
            if d > 3.5 and nxt is not None:
                rel, rel2 = cam / (m * d), nxt[1] / (m * nxt[0])
                if rel2 > REPAIR_NEXT_LONG or rel2 < 1.0 - 0.5 * (1.0 - rel):
                    continue                  # the next leg is off too (half as much or more): the spot or the camera
            short = m * d - cam
            if (short > min_cm and cam < share * m * d) or (short >= REPAIR_ABS_CM and cam < REPAIR_ABS_SHARE * m * d):
                b = st2[i]
                b.setdefault("d_own", b["d"])
                b["d"] = math.copysign(cam / m, b["d"])
                k += 1
        out[n] = st2
    return out, k


def _take_free_d(robots, sol):
    """robots with the fitted lengths and turns of their free legs (sol['free_d']) as their d / dth (d_own kept)."""
    fd = sol.get("free_d") or {}
    out = {}
    for n, v in robots.items():
        v2 = [dict(q) for q in v]
        dm, tm = fd.get(n) or ({}, {})
        for i, d in dm.items():
            v2[i]["d"] = d
        for i, t in tm.items():
            v2[i]["dth"] = t
        out[n] = v2
    return out


def repairs_of(robots):
    """{name: [[stop index, distance, turn], ...]} of the legs the fit set (repaired, or lost records)."""
    out = {}
    for n, v in robots.items():
        rows = [[i, q["d"], q["dth"]] for i, q in enumerate(v) if "d_own" in q or q.get("lost")]
        if rows:
            out[n] = rows
    return out


def apply_repairs(robots, sol):
    """The runner's stops with the fit's leg repairs and lost-record legs applied (for the contacts and positions
    built on the fit)."""
    rp = sol.get("repairs") or {}
    out = {}
    for n, v in robots.items():
        v2 = [dict(q) for q in v]
        for row in rp.get(n, []):
            i, d = row[0], row[1]
            if i < len(v2):
                if v2[i].get("d") != d and "d_own" not in v2[i]:
                    v2[i]["d_own"] = v2[i].get("d")
                v2[i]["d"] = d
                if len(row) > 2:
                    v2[i]["dth"] = row[2]
        out[n] = v2
    return out


def best_fit(robots, size, g_fixed=None):
    """fit_once from both starts (a homography through the best-seen robot; the pixels laid flat as from overhead):
    the better fit."""
    sols = []
    for how in ("homography", "overhead"):
        try:
            init = None if how == "homography" else _init_prov(robots, size)
            sol = fit_once(robots, size, g_fixed, init=init)
            sol["init"] = how
            sols.append(sol)
        except (ValueError, np.linalg.LinAlgError, cv2.error):
            continue
    if not sols:
        raise ValueError("no start value worked")
    return min(sols, key=lambda q: q["rms_px"] if np.isfinite(q["rms_px"]) else 1e9)


# Priors tried in the explore reliability study (2026-10-26; offline on the simulator batch's recorded stops,
# analyses_oct26/explore_reliability/evalfits.py):
# - each robot's wheel scale near 1 (sd 4 % in log): ON. With legs freed by repair_legs, the camera's perspective and
#   the robots' separate wheel scales trade off (seed 49: wheel scales 0.91 / 1.06 / 1.04 against true 1.03 / 1.00 /
#   1.00; batch run r00: the camera 47 % too big with the repairs alone, 0.6 % with this prior). Real wheel scales
#   from runs against the camera: 0.96-0.98 (10-24, 10-25). DOE note: yes, simulator: sd 2 / 4 / 8 %, weight 1 / 3 / 10,
#   over the batch's stops (camera scale error and pass rate), then the real sessions' stops against roam1
S_PRIOR_SD_LOG = 0.04
S_PRIOR_W = 3.0
# - the camera looks roughly straight down (tilt sd 10 deg, weight 10): OFF. It fixes the real 09:55 Center-only fit
#   (camera 12.5 % small -> 0.3 %) but that fit is refused anyway (EXPLORE_MIN_ROBOTS), and on the batch it was mixed.
#   DOE note: yes, the same simulator row, at the simulator's 3 deg and the real camera's 8.8 deg tilt (--tilt)
TILT_PRIOR_SD_DEG = None
TILT_PRIOR_W = 3.0
# - the webcam's focal length known (606 px at 640 wide): OFF, it did not fix the scale on its own (09:55: 12.8 %).
#   No DOE: kept only for the study
F_PRIOR_PX = None
F_PRIOR_SD_LOG_KNOWN = 0.05


def _tilt_deg(rvec):
    """The camera's look direction from straight down, deg (0 = looking straight down at the floor)."""
    R = cv2.Rodrigues(np.asarray(rvec, float))[0]
    ax = R.T @ np.array([0.0, 0.0, 1.0])
    return math.degrees(math.acos(min(1.0, abs(float(ax[2])))))


def fit_once(robots, size, g_fixed=None, init=None):
    """robots: {name: [stop dict(x, y, th (its own pose), u, v (pixels of its top))]}, in time order; the first
    robot's own frame is the world frame (the one the camera saw at most stops). Returns dict(f, k1, rvec, tvec, frames {name: (x0, y0, turn)},
    s {name}, g {name}, rms_px, n, per_robot_rms_px)."""
    frames0, scales0, (_, f0, rv0, tv0), G0 = init if init is not None else _init(robots, size)
    ref = next(n for n, fr in frames0.items() if fr == (0.0, 0.0, 0.0))
    names = [ref] + [n for n in robots if n != ref]      # the reference robot's frame is the world frame
    nr = len(names)
    # parameters: logf, k1, rvec(3), tvec(3), then per robot (except the first): x0, y0, turn; then per robot:
    # log s (all but the last: the sum is 0), log g
    x0 = [math.log(f0), 0.0] + list(rv0) + list(tv0)
    for n in names[1:]:
        x0 += list(frames0[n])
    ls = [math.log(scales0[n]) for n in names]
    x0 += ls[:-1]
    x0 += [math.log(G0.get(n, 1.0)) for n in names]
    # legs the camera saw much shorter than the wheels counted (repair_legs: d_own kept): their length is fitted too,
    # between 0 and what the wheels counted (a body never goes further than its wheels, give or take slip)
    # A leg whose record never arrived ("lost": taken as told by the runner) is free the same way: a drive between 0
    # and what it was told, a turn between 0 and 1.2 x what it was told (the gyro counts 91-95 % of a turn)
    free = []
    for n in names:
        for i, q in enumerate(robots[n]):
            if i == 0:
                continue
            lost = q.get("lost") or {}
            if "d_own" in q:
                free.append((n, i, "d", 1.05 * abs(q["d_own"]) + 0.1, math.copysign(1.0, q["d_own"] or 1.0)))
            elif lost.get("kind") == "go":
                free.append((n, i, "d", 1.05 * abs(lost["val"]) + 0.1, math.copysign(1.0, lost["val"] or 1.0)))
            if lost.get("kind") == "turn":
                free.append((n, i, "th", 1.2 * abs(lost["val"]) + 1.0, math.copysign(1.0, lost["val"] or 1.0)))
    n_fixed = len(x0)
    x0 += [min(abs(robots[n][i].get("d" if k == "d" else "dth") or 0.0), hi_ - 1e-3) for n, i, k, hi_, _ in free]
    f_prior = math.log(F_PRIOR_PX * size[0] / 640.0) if F_PRIOR_PX else \
        math.log(size[0] / (2 * math.tan(math.radians(F_PRIOR_HFOV_DEG) / 2)))
    f_sd = F_PRIOR_SD_LOG_KNOWN if F_PRIOR_PX else F_PRIOR_SD_LOG
    uv_all = {n: np.array([(q["u"], q["v"]) for q in robots[n]], float) for n in names}
    use = {n: np.array([0.0 if q.get("skip") else 1.0 for q in robots[n]], float)[:, None] for n in names}

    def unpack(x):
        i = 0
        lf, k1 = x[0], x[1]
        rv, tv = x[2:5], x[5:8]
        i = 8
        fr = {names[0]: (0.0, 0.0, 0.0)}
        for n in names[1:]:
            fr[n] = tuple(x[i:i + 3])
            i += 3
        lsv = list(x[i:i + nr - 1])
        i += nr - 1
        lsv.append(-sum(lsv))
        lg = list(x[i:i + nr])
        return lf, k1, rv, tv, fr, dict(zip(names, lsv)), dict(zip(names, lg))

    def dmaps(x):
        out = {}
        for (n, i, k, _, sg), v in zip(free, x[n_fixed:]):
            out.setdefault(n, ({}, {}))[0 if k == "d" else 1][i] = sg * float(v)
        return out

    def resid(x):
        lf, k1, rv, tv, fr, lsd, lgd = unpack(x)
        dm = dmaps(x)
        out = []
        for n in names:
            g = g_fixed[n] if g_fixed and n in g_fixed else math.exp(lgd[n])
            w = to_world(rebuild(robots[n], math.exp(lsd[n]), g, *dm.get(n, (None, None))), fr[n])
            obj = [(p[0], p[1], ROBOT_TOP_CM) for p in w]
            uv = project(obj, math.exp(lf), k1, rv, tv, size)
            out.append(((uv - uv_all[n]) * use[n]).ravel())
        pri = [(lf - f_prior) / f_sd * 3.0, k1 / K1_PRIOR_SD * 3.0]
        if TILT_PRIOR_SD_DEG:
            pri.append(_tilt_deg(rv) / TILT_PRIOR_SD_DEG * TILT_PRIOR_W)
        if S_PRIOR_SD_LOG:
            pri += [v / S_PRIOR_SD_LOG * S_PRIOR_W for v in lsd.values()]
        out.append(np.array(pri))
        return np.concatenate(out)

    lo = [-np.inf] * len(x0)
    hi = [np.inf] * len(x0)
    lo[0], hi[0] = math.log(0.2 * size[0]), math.log(5.0 * size[0])
    lo[1], hi[1] = -0.6, 0.6
    j = 8 + 3 * (nr - 1)
    for k in range(j, j + nr - 1):                       # log wheel scales: within +-20%
        lo[k], hi[k] = -0.2, 0.2
    for k in range(j + nr - 1, j + 2 * nr - 1):          # log gyro scales: within +-30%
        lo[k], hi[k] = -0.3, 0.3
    for k, (n, i, _, hi_, _s) in enumerate(free):       # a free leg: 0 to its wheels' count (+5 % slip) or its told value
        lo[n_fixed + k], hi[n_fixed + k] = 0.0, hi_
    x0 = np.clip(np.array(x0, float), np.array(lo) + 1e-9, np.array(hi) - 1e-9)
    sol = least_squares(resid, x0, loss="huber", f_scale=3.0, max_nfev=4000, bounds=(lo, hi), method="trf")
    lf, k1, rv, tv, fr, lsd, lgd = unpack(sol.x)
    per, i, res_n = {}, 0, {}
    lf, k1, rv, tv, fr, lsd, lgd = unpack(sol.x)
    dm = dmaps(sol.x)
    for n in names:
        k = len(robots[n])
        g = g_fixed[n] if g_fixed and n in g_fixed else math.exp(lgd[n])
        w = to_world(rebuild(robots[n], math.exp(lsd[n]), g, *dm.get(n, (None, None))), fr[n])
        uv = project([(p[0], p[1], ROBOT_TOP_CM) for p in w], math.exp(lf), k1, rv, tv, size)
        res_n[n] = np.hypot(*(uv - uv_all[n]).T)
        m = use[n].ravel() > 0
        per[n] = float(np.sqrt(np.mean(res_n[n][m] ** 2))) if m.any() else float("nan")
        i += k
    r = np.concatenate([res_n[n][use[n].ravel() > 0] for n in names])[:, None]
    return dict(f=math.exp(lf), k1=float(k1), rvec=[float(v) for v in rv], tvec=[float(v) for v in tv],
                frames={n: tuple(float(v) for v in fr[n]) for n in names},
                s={n: math.exp(lsd[n]) for n in names},
                g={n: (g_fixed[n] if g_fixed and n in g_fixed else math.exp(lgd[n])) for n in names},
                rms_px=float(np.sqrt(np.mean(np.sum(r ** 2, 1)))), per_robot_rms_px=per, resid=res_n, free_d=dm,
                n=int(sum(len(v) for v in robots.values())), size=list(size))


def robot_px(sol, x, y):
    """A robot's width in pixels at world (x, y) with the fitted camera (the known-size cross-check)."""
    a = project([(x - ROBOT_CM / 2, y, ROBOT_TOP_CM), (x + ROBOT_CM / 2, y, ROBOT_TOP_CM)], sol["f"], sol["k1"],
                sol["rvec"], sol["tvec"], sol["size"])
    return float(np.hypot(*(a[1] - a[0])))


def world_of(sol, u, v, z=ROBOT_TOP_CM):
    """The floor point (world frame) at height z seen at pixel (u, v)."""
    K = K_of(sol["f"], sol["size"])
    p = cv2.undistortPoints(np.array([[[u, v]]], float), K, np.array([sol["k1"], 0, 0, 0, 0], float)).reshape(2)
    R = cv2.Rodrigues(np.asarray(sol["rvec"], float))[0]
    t = np.asarray(sol["tvec"], float)
    c = -R.T @ t
    d = R.T @ np.array([p[0], p[1], 1.0])
    k = (z - c[2]) / d[2]
    q = c + k * d
    return float(q[0]), float(q[1])


# ------------------------------------------------------------------------------------------------- the border
def _line_fit(pts):
    p = np.asarray(pts, float)
    m = p.mean(0)
    _, _, vt = np.linalg.svd(p - m)
    return m, vt[0]                                       # a point on it, its direction


def sides(points, n_max=4, tol=SIDE_TOL_CM, min_pts=2, iters=300, seed=1):
    """Straight sides from contact points [(x, y, kind)] (world frame): repeatedly the line with the most points
    within tol (RANSAC), its points removed. Returns [dict(point, dir, n, kind, kinds, spread, pts)]."""
    rng = np.random.default_rng(seed)
    left = [p for p in points]
    out = []
    while len(left) >= min_pts and len(out) < n_max:
        xy = np.array([(p[0], p[1]) for p in left], float)
        best = None
        for _ in range(iters):
            i, j = rng.choice(len(left), 2, replace=False) if len(left) > 2 else (0, 1)
            d = xy[j] - xy[i]
            L = np.hypot(*d)
            if L < 1e-6:
                continue
            nrm = np.array([-d[1], d[0]]) / L
            dist = np.abs((xy - xy[i]) @ nrm)
            inl = np.where(dist <= tol)[0]
            if best is None or len(inl) > len(best):
                best = inl
        if best is None or len(best) < min_pts:
            break
        pts = [left[k] for k in best]
        m, dvec = _line_fit([(p[0], p[1]) for p in pts])
        nrm = np.array([-dvec[1], dvec[0]])
        spread = float(np.ptp([(np.array(p[:2]) - m) @ nrm for p in pts])) if len(pts) > 1 else 0.0
        kinds = [p[2] for p in pts]
        out.append(dict(point=m, dir=dvec, n=len(pts), kind=max(set(kinds), key=kinds.count), kinds=kinds,
                        spread=spread, pts=pts))
        left = [p for k, p in enumerate(left) if k not in set(best.tolist())]
    return out


def rectangle(sd):
    """From fitted sides, the board as a rectangle: its direction (the sides' common direction, mod 90 deg) and,
    in that turned frame, the four side lines. Returns dict(angle_deg, lines {'x0','x1','y0','y1': (value, side)})
    or None when not all four sides are there."""
    if len(sd) < 2:
        return None
    angs = [math.degrees(math.atan2(s["dir"][1], s["dir"][0])) % 90.0 for s in sd]
    w = [s["n"] for s in sd]
    # circular mean on the 90-deg circle
    c = sum(wi * math.cos(math.radians(4 * a)) for a, wi in zip(angs, w))
    s_ = sum(wi * math.sin(math.radians(4 * a)) for a, wi in zip(angs, w))
    alpha = (math.degrees(math.atan2(s_, c)) / 4.0) % 90.0
    ca, sa = math.cos(math.radians(-alpha)), math.sin(math.radians(-alpha))
    lines = {}
    for s in sd:
        pts = np.array([(p[0], p[1]) for p in s["pts"]], float)
        rp = np.column_stack([ca * pts[:, 0] - sa * pts[:, 1], sa * pts[:, 0] + ca * pts[:, 1]])
        a = math.degrees(math.atan2(s["dir"][1], s["dir"][0])) - alpha
        horizontal = abs(math.sin(math.radians(a))) < abs(math.cos(math.radians(a)))
        lines.setdefault("h" if horizontal else "v", []).append((float(np.median(rp[:, 1] if horizontal else rp[:, 0])), s))
    h, v = sorted(lines.get("h", [])), sorted(lines.get("v", []))
    if len(h) < 2 or len(v) < 2:
        return None
    return dict(angle_deg=alpha, lines={"y0": h[0], "y1": h[-1], "x0": v[0], "x1": v[-1]})


def board_frame(rect):
    """The transform from the world (exploring) frame to the runner's board frame: the corner of the two open edges
    at (0, 0), the boards at x = W and y = H. Returns dict(R (2x2), t, W, H, open, boards, kinds) or None when the
    sides' kinds are not two open edges meeting at a corner and two boards."""
    L = rect["lines"]
    kinds = {k: L[k][1]["kind"] for k in L}
    opens = [k for k in kinds if kinds[k] == "edge"]
    boards = [k for k in kinds if kinds[k] == "board"]
    if len(opens) != 2 or len(boards) != 2 or opens[0][0] == opens[1][0]:
        return None
    alpha = math.radians(rect["angle_deg"])
    # in the turned frame (x', y'): the open x-side and open y-side
    ox = next(k for k in opens if k[0] == "x")
    oy = next(k for k in opens if k[0] == "y")
    bx = "x1" if ox == "x0" else "x0"
    by = "y1" if oy == "y0" else "y0"
    W = abs(L[bx][0] - L[ox][0])
    Hh = abs(L[by][0] - L[oy][0])
    # board frame axes in the turned frame: x along from the open y-side line... origin at (L[ox], L[oy])
    sx = 1.0 if L[bx][0] > L[ox][0] else -1.0
    sy = 1.0 if L[by][0] > L[oy][0] else -1.0
    # turned -> board: x_b = sx (x' - ox), y_b = sy (y' - oy); a reflection when sx * sy < 0: then swap the axes
    # (a right-handed board frame: x_b and y_b exchanged keeps it a rotation)
    ca, sa = math.cos(-alpha), math.sin(-alpha)
    Rt = np.array([[ca, -sa], [sa, ca]])                  # world -> turned
    if sx * sy > 0:
        A = np.diag([sx, sy])
        W_, H_ = W, Hh
        xb_side, yb_side = by, bx                         # x-board: the board along x (a y = const line)
    else:
        A = np.array([[0.0, sy], [sx, 0.0]])              # x_b = sy (y' - oy), y_b = sx (x' - ox)
        W_, H_ = Hh, W
        xb_side, yb_side = bx, by
    R = A @ Rt
    o_turned = np.array([L[ox][0], L[oy][0]])
    t = -A @ o_turned
    return dict(R=R, t=t, W=W_, H=H_, kinds=kinds, sides={k: L[k][0] for k in L}, xboard=xb_side, yboard=yb_side,
                spreads={k: L[k][1]["spread"] for k in L}, n={k: L[k][1]["n"] for k in L})


def camera_in_board(sol, bf):
    """The fitted camera (world frame) re-expressed in the board frame: p_cam = R p_world + T, p_board = Rb p_world
    + tb -> p_cam = (R Rb^T) p_board + (T - R Rb^T tb)."""
    R = cv2.Rodrigues(np.asarray(sol["rvec"], float))[0]
    T = np.asarray(sol["tvec"], float)
    Rb = np.eye(3)
    Rb[:2, :2] = bf["R"]
    tb = np.array([bf["t"][0], bf["t"][1], 0.0])
    R2 = R @ Rb.T
    T2 = T - R2 @ tb
    return dict(K=K_of(sol["f"], sol["size"]).tolist(), dist=[sol["k1"], 0, 0, 0, 0],
                rvec=cv2.Rodrigues(R2)[0].ravel().tolist(), tvec=T2.tolist(), image_size=list(sol["size"]))


def to_board(bf, x, y):
    p = bf["R"] @ np.array([x, y]) + bf["t"]
    return float(p[0]), float(p[1])


def rectangle_from_rays(points, step_deg=0.25, cap_cm=10.0):
    """The board as a rectangle straight from the contact points and the headings of the rays that met them:
    points [(x, y, kind, heading deg)] (world frame). For each turn of the frame (0-90 deg), a ray heading mostly
    along +x' met the x1 side, and so on; the turn whose sides are straightest (least spread across each side) is the
    board's. Robust to a side with few points, unlike fitting lines one by one (two stray points make a line).
    Returns the same dict as rectangle() (angle_deg, lines {side: (value, side dict)}) or None."""
    if len(points) < 4:
        return None
    P = np.array([(p[0], p[1]) for p in points], float)
    H = np.radians([p[3] for p in points])
    best = None
    for a in np.arange(0.0, 90.0, step_deg):
        ca, sa = math.cos(math.radians(-a)), math.sin(math.radians(-a))
        X = ca * P[:, 0] - sa * P[:, 1]
        Y = sa * P[:, 0] + ca * P[:, 1]
        hx, hy = np.cos(H - math.radians(a)), np.sin(H - math.radians(a))
        side = np.where(np.abs(hx) >= np.abs(hy), np.where(hx > 0, 1, 0), np.where(hy > 0, 3, 2))  # x0 x1 y0 y1
        score, ok = 0.0, True
        for k in range(4):
            m = side == k
            if not m.any():
                ok = False
                continue
            c = X[m] if k < 2 else Y[m]
            score += float(np.minimum(np.abs(c - np.median(c)), cap_cm).sum())
        if not ok:
            score += 1000.0
        if best is None or score < best[0]:
            best = (score, a, X, Y, side)
    score, a, X, Y, side = best
    if score >= 1000.0:
        return None
    vals = {}
    for k in range(4):
        m = side == k
        vals[k] = float(np.median(X[m] if k < 2 else Y[m]))
    # second pass: each point to the side its coordinate is nearest to (corner rays)
    res = np.column_stack([np.abs(X - vals[0]), np.abs(X - vals[1]), np.abs(Y - vals[2]), np.abs(Y - vals[3])])
    side = np.argmin(res, axis=1)
    lines = {}
    for k, name in enumerate(("x0", "x1", "y0", "y1")):
        m = np.where(side == k)[0]
        if not len(m):
            return None
        c = (X if k < 2 else Y)[m]
        v = float(np.median(c))
        kinds = [points[i][2] for i in m]
        lines[name] = (v, dict(kind=max(set(kinds), key=kinds.count), kinds=kinds, n=len(m),
                               spread=float(np.ptp(c)) if len(m) > 1 else 0.0,
                               pts=[points[i] for i in m]))
    if lines["x1"][0] <= lines["x0"][0] or lines["y1"][0] <= lines["y0"][0]:
        return None
    return dict(angle_deg=float(a), lines=lines, score=score)


def refine_edges(rect, nominal_ahead_cm, lo_cm=0.0, hi_cm=16.0, min_spread=0.3):
    """How far ahead of the axle the edge guard really sees an open edge, learned from the rays themselves
    (2026-10-23 dry run: most edge stops were the distance sensor's bottom zone, which sees the drop well before the
    line sensors reach it, so the felt edges sat ~4 cm inside the plywood). A ray that meets an edge square on stops
    `ahead` short of it; one that meets it slanting stops ahead * cos(slant) short. So across one side the axle's
    coordinate is v - ahead * (the heading's part across the side): a straight line in that part, with the edge
    (v) and ahead shared by every ray. One ahead for every edge side, one v per side; least squares.
    rect: from rectangle_from_rays (its points carry the nominal contact, kind and heading). Returns (rect with the
    edge sides' values replaced, ahead_cm) or (rect, None) when the rays' slants are too alike to tell."""
    a = math.radians(rect["angle_deg"])
    ca, sa = math.cos(-a), math.sin(-a)
    rows, rhs, names = [], [], []
    for name, (v, sd) in rect["lines"].items():
        if sd["kind"] != "edge":
            continue
        for p in sd["pts"]:
            x, y, kind, hd = p[:4]
            if kind != "edge" or (len(p) > 4 and p[4] == "line"):
                continue                   # the line sensors' stops are where e02_map says; the fit is for the rest
            h = math.radians(hd) - a
            hx, hy = math.cos(h), math.sin(h)
            # the axle (the contact minus the nominal reach), turned into the board's axes
            ax = x - nominal_ahead_cm * math.cos(math.radians(hd))
            ay = y - nominal_ahead_cm * math.sin(math.radians(hd))
            X, Y = ca * ax - sa * ay, sa * ax + ca * ay
            coord, part = (X, hx) if name in ("x0", "x1") else (Y, hy)
            if name not in names:
                names.append(name)
            rows.append((name, part))
            rhs.append(coord)
    if len(rows) < 4 or not names:
        return rect, None
    parts = np.array([abs(p) for _, p in rows])
    if parts.max() - parts.min() < min_spread:
        return rect, None                  # the slants too alike: reach and edge cannot be told apart
    A = np.zeros((len(rows), len(names) + 1))
    for i, (n, p) in enumerate(rows):
        A[i, names.index(n)] = 1.0
        A[i, -1] = -p                      # coord = v - ahead * part
    sol, *_ = np.linalg.lstsq(A, np.array(rhs), rcond=None)
    ahead = float(sol[-1])
    if not (lo_cm <= ahead <= hi_cm):
        return rect, None
    out = dict(rect, lines=dict(rect["lines"]))
    for k, n in enumerate(names):
        v, sd = out["lines"][n]
        res = np.array(rhs)[[i for i, (m, _) in enumerate(rows) if m == n]] - (
            sol[k] - ahead * np.array([p for m, p in rows if m == n]))
        out["lines"][n] = (float(sol[k]), dict(sd, spread=float(np.ptp(res)) if len(res) > 1 else 0.0,
                                               nominal=v))
    if out["lines"]["x1"][0] <= out["lines"]["x0"][0] or out["lines"]["y1"][0] <= out["lines"]["y0"][0]:
        return rect, None
    out["edge_ahead_cm"] = ahead
    return out, ahead


SWERVE_DEG = 3.0     # chosen: a straight ray that turned more than this by the time it stopped


def contacts(robots, sol, robot_clear_cm=22.0, ahead=None, others=None):
    """The rays' contact points in the world frame: [(x, y, kind, heading deg)]. A ray stopped by another robot is
    left out: where that robot stood at that moment (its last stop before then, or its first one if it had not
    moved yet: 2026-10-23 dry run, Center's ray met Right, which had not explored yet, and only robots' end places
    were checked). ahead: {kind: cm} to override e02_map's reach. others: {name: stops} of robots left out of the
    fit (2026-10-24 dry run: Center was left out, and rays that met it were taken for the board, scattering that
    side over 28 cm): they still stand on the board, so where the fitted camera sees them is checked too."""
    paths = {n: to_world(rebuild(st, sol["s"][n], sol["g"][n]), sol["frames"][n]) for n, st in robots.items()}

    def where(m, t):
        st = robots[m]
        k = 0
        for i, q in enumerate(st):
            if q.get("t") is not None and t is not None and q["t"] <= t:
                k = i
        return paths[m][k]

    def seen(m, t):
        """A left-out robot's place at time t: its stop then, where the fitted camera sees it."""
        st = (others or {}).get(m) or []
        q0 = None
        for q in st:
            if q.get("u") is None:
                continue
            if q0 is None or (q.get("t") is not None and t is not None and q["t"] <= t):
                q0 = q
        if q0 is None:
            return None
        w = world_of(sol, q0["u"], q0["v"])
        return w if w is not None and np.all(np.isfinite(w)) else None

    out = []
    for n, stops in robots.items():
        for (x, y, th), q in zip(paths[n], stops):
            if not q.get("contact"):
                continue
            kind = q["contact"]
            if abs(q.get("dth", 0.0) or 0.0) > SWERVE_DEG and q.get("d") is not None:
                continue                  # it turned while stopping (2026-10-23: -6.9 deg): where it met is unsure
            if q.get("reach_cm") is not None:
                # measured at the stop (explore's creep: its own front reading, 2026-10-25)
                c = (x + q["reach_cm"] * math.cos(math.radians(th)), y + q["reach_cm"] * math.sin(math.radians(th)))
            elif ahead and kind in ahead:
                c = (x + ahead[kind] * math.cos(math.radians(th)), y + ahead[kind] * math.sin(math.radians(th)))
            else:
                import e02_map as MAP
                c = MAP.contact_point(x, y, th, kind, q.get("cue", ""))
            if any(math.hypot(c[0] - o[0], c[1] - o[1]) < robot_clear_cm
                   for m in robots if m != n for o in [where(m, q.get("t"))]):
                continue
            if any(math.hypot(c[0] - o[0], c[1] - o[1]) < robot_clear_cm
                   for m in (others or {}) for o in [seen(m, q.get("t"))] if o is not None):
                continue
            out.append((c[0], c[1], kind, th, q.get("cue", "")))
    return out


SEEN_RAY_MIN_CM = 8.0     # chosen: a left-out robot's ray must be at least this long for its heading to come from the
                          # camera (2 px noise at each end is about 6 deg on 8 cm). No DOE: below it the heading is noise
SEEN_RAY_AGREE = 0.25     # chosen: the camera's length of that ray within 25 % of the wheels' count (a camera spot that
                          # followed the robot on this ray, whatever went wrong elsewhere). DOE note: yes, simulator,
                          # with the leg-repair row (count wrong sides against contacts gained)


def contacts_seen(others, sol, used_paths=None, robot_clear_cm=22.0):
    """Contacts of robots left out of the camera fit, placed by the fitted camera alone (09:55 session: Left's and
    Right's gap rays met the boards 8 times, but their paths were out of the fit, so none counted; the sides were
    never closed). For each contact stop: where the camera sees it; its heading from the camera's line between the
    ray's start and end (the ray is straight); only when the camera's length of the ray agrees with the wheels'
    count (the spot followed the robot on that ray). Same format as contacts(); a contact near another robot is left
    out as there. used_paths: {name: [(t, x, y)]} of the robots in the fit (their places in the world frame)."""
    import e02_map as MAP
    out = []
    for n, stops in (others or {}).items():
        imp = implausible(stops)
        for i, q in enumerate(imp):
            if not q.get("contact") or q.get("skip") or q.get("u") is None:
                continue
            # the ray: back to the stop before the out drive (creep stops ride on the ray before them)
            j, d_tot = i, 0.0
            while j > 0 and abs(imp[j].get("dth", 0.0) or 0.0) < 3.0 and (imp[j].get("d") or 0.0) > 0:
                d_tot += imp[j]["d"]
                j -= 1
            a = imp[j]
            if d_tot < SEEN_RAY_MIN_CM or a.get("skip") or a.get("u") is None:
                continue
            pa, pb = np.array(world_of(sol, a["u"], a["v"])), np.array(world_of(sol, q["u"], q["v"]))
            cam = float(np.linalg.norm(pb - pa))
            if not np.isfinite(cam) or abs(cam / d_tot - 1.0) > SEEN_RAY_AGREE:
                continue
            th = math.degrees(math.atan2(pb[1] - pa[1], pb[0] - pa[0]))
            kind = q["contact"]
            if q.get("reach_cm") is not None:
                r = q["reach_cm"]
                c = (pb[0] + r * math.cos(math.radians(th)), pb[1] + r * math.sin(math.radians(th)))
            else:
                c = MAP.contact_point(pb[0], pb[1], th, kind, q.get("cue", ""))
            near = False
            for m, path in (used_paths or {}).items():
                if m == n or not path:
                    continue
                o = min(path, key=lambda r_: abs((r_[0] or 0) - (q.get("t") or 0)))
                if math.hypot(c[0] - o[1], c[1] - o[2]) < robot_clear_cm:
                    near = True
            if not near:
                out.append((c[0], c[1], kind, th, q.get("cue", "")))
    return out


ROBOT_LENGTH_CM = 9.5     # the Alvik's length (e02_plan.ROBOT_HALF_CM x 2): its outline's long side from above


def size_scale(robots, sol, size):
    """The robots' own size as a check on explore's scale (Nick, 2026-10-23: "robot size for now"): at each stop
    with an outline measured (size_px, the long side), the fitted camera says how many cm that is at robot-top
    height. Returns dict(ratio (measured / the real length: 1.02 = explore's distances 2% short, or the outline
    padded by its shadow), n, sd) or None. Reported, not applied: the detector's outline includes some shadow, which
    has not been measured on the real board yet."""
    vals = []
    for n, stops in robots.items():
        path = to_world(rebuild(stops, sol["s"][n], sol["g"][n]), sol["frames"][n])
        for (x, y, _), q in zip(path, stops):
            if q.get("skip") or not q.get("size_px"):
                continue
            p0 = project([(x, y, ROBOT_TOP_CM)], sol["f"], sol["k1"], np.array(sol["rvec"]), np.array(sol["tvec"]), size)[0]
            p1 = project([(x + 1.0, y, ROBOT_TOP_CM)], sol["f"], sol["k1"], np.array(sol["rvec"]), np.array(sol["tvec"]), size)[0]
            p2 = project([(x, y + 1.0, ROBOT_TOP_CM)], sol["f"], sol["k1"], np.array(sol["rvec"]), np.array(sol["tvec"]), size)[0]
            ppc = 0.5 * (math.hypot(*(p1 - p0)) + math.hypot(*(p2 - p0)))
            if ppc > 0:
                vals.append(q["size_px"] / ppc / ROBOT_LENGTH_CM)
    if len(vals) < 5:
        return None
    v = np.array(vals)
    return dict(ratio=round(float(np.median(v)), 4), n=len(v), sd=round(float(np.std(v)), 4))


# ---- check before trust (Nick, 2026-10-23 evening: one robot's bad data must not spoil the calibration) ----
ROBOT_RMS_MAX_PX = 20.0     # chosen: a robot's own path fits its pixels within this (simulator 0.5-7 px, the most
                            # from its own drift over ~40 stops; a lens bend adds little: the real camera against a
                            # flat-floor mapping, 0 px); a sanity bound against nonsense (2026-10-23: 136 px), not a
                            # quality target, so a DOE would not help
ROBOT_INLIERS_MIN = 0.7     # chosen: at least 70% of its stops within 3 x that
GYRO_RANGE = (0.85, 1.25)   # chosen: real gyros read 91-95% of a turn (scale 1.05-1.10); outside this, something
                            # other than the gyro is wrong (a wheel off the board, the tracker on something else)
FIT_RMS_MAX_PX = 8.0        # chosen: the joint camera fit (simulator 1.5-4.5 px); a sanity bound like the above
FIT_ROBOT_MAX_PX = 12.0     # chosen: one robot's stops against the joint fit (the robot check above is cruder: a
                            # flat mapping cannot follow a robot's own drift over 50 stops, simulator up to 13 px)


def repair_legs_px(stops, min_px=None, share=None):
    """repair_legs without a camera fit, for the robot check: each straight leg's pixel length against the robot's
    own pixels per cm (the median over its straight legs of 5 cm or more; the camera's scale changes only a few % over
    the board when it looks down). Returns (stops copy, how many legs)."""
    share = REPAIR_SHARE if share is None else share
    st2 = [dict(q) for q in stops]
    legs = []
    for i in range(1, len(st2)):
        a, b = st2[i - 1], st2[i]
        if a.get("skip") or b.get("skip") or "dth" not in b or b.get("u") is None or a.get("u") is None:
            continue
        d = abs(b.get("d") or 0.0)
        if d < 2.0 or abs(b["dth"]) > REPAIR_MAX_TURN_DEG:
            continue
        legs.append((i, d, math.hypot(b["u"] - a["u"], b["v"] - a["v"])))
    r = [c / d for _, d, c in legs if d >= 5.0]
    if len(r) < 3:
        return st2, 0
    m = float(np.median(r))
    k = 0
    for i, d, c in legs:
        short = d - c / m
        if (short > REPAIR_MIN_CM and c < share * m * d) or (short >= REPAIR_ABS_CM and c < REPAIR_ABS_SHARE * m * d):
            st2[i]["d"] = math.copysign(c / m, st2[i]["d"])
            k += 1
    return st2, k


def robot_check(stops):
    """One robot judged on its own: the gyro scale whose path a flat-floor mapping (homography) fits best, how well
    it fits (robust rms, px, over the stops within 3 x ROBOT_RMS_MAX_PX), and the share of stops that fit. Returns
    dict(ok, why, rms_px, g, inliers, n). 2026-10-26 explore reliability: a robot that fails is judged once more
    with the legs its own pixels say its wheels overcounted taken at the pixels' length (repair_legs_px; the camera
    fit then fits those legs itself): one creep against a board or a hang no longer leaves a robot out."""
    out = _robot_check(stops)
    if out["ok"] or not REPAIR_LEGS:
        return out
    st2, k = repair_legs_px(stops)
    if not k:
        return out
    out2 = _robot_check(st2)
    if out2["ok"]:
        out2["repaired_px"] = k
        return out2
    return out


def _robot_check(stops):
    st = implausible(stops)
    ok_m = np.array([not q.get("skip") for q in st])
    n = int(ok_m.sum())
    if n < 6:
        return dict(ok=False, why="only %d stops the camera saw" % n, rms_px=None, g=None, inliers=0.0, n=n)
    uv = np.array([(q["u"], q["v"]) for q in st], float)[ok_m]
    best = None
    for g in np.arange(0.75, 1.35, 0.005):
        p = np.array([(q[0], q[1]) for q in rebuild(st, 1.0, g)], float)[ok_m]
        if np.ptp(p[:, 0]) < 1e-6 and np.ptp(p[:, 1]) < 1e-6:
            continue
        H, _ = cv2.findHomography(p, uv, cv2.RANSAC, 3.0 * ROBOT_RMS_MAX_PX)
        if H is None:
            continue
        e = np.linalg.norm(cv2.perspectiveTransform(p.reshape(-1, 1, 2), H).reshape(-1, 2) - uv, axis=1)
        inl = e <= 3.0 * ROBOT_RMS_MAX_PX
        if inl.sum() < 4:
            continue
        rms = float(np.sqrt(np.mean(e[inl] ** 2)))
        key = (-inl.mean(), rms)
        if best is None or key < best[0]:
            best = (key, float(g), rms, float(inl.mean()))
    if best is None:
        return dict(ok=False, why="its path and its pixels have nothing in common", rms_px=None, g=None,
                    inliers=0.0, n=n)
    _, g, rms, share = best
    why = []
    if share < ROBOT_INLIERS_MIN:
        why.append("only %.0f%% of its stops fit its own path" % (100 * share))
    if rms > ROBOT_RMS_MAX_PX:
        why.append("its stops fit its path only within %.1f px" % rms)
    if not (GYRO_RANGE[0] <= g <= GYRO_RANGE[1]):
        why.append("its gyro would have to be %.0f%% off" % (100 * abs(g - 1)))
    return dict(ok=not why, why="; ".join(why), rms_px=round(rms, 2), g=round(g, 3), inliers=round(share, 3), n=n)


def fit_check(sol, lo_s=-0.2, hi_s=0.2, lo_g=-0.3, hi_g=0.3):
    """The joint fit, checked: its rms and no scale pinned at a bound (2026-10-23: wheel scales of 0.819 and 1.492
    were the bounds, i.e. the fit had nowhere sensible to go). Returns '' or why not."""
    why = []
    if not np.isfinite(sol["rms_px"]) or sol["rms_px"] > FIT_RMS_MAX_PX:
        why.append("the camera fit misses by %.1f px (limit %.0f)" % (sol["rms_px"], FIT_RMS_MAX_PX))
    for n, v in sol["s"].items():
        if not (lo_s + 0.01 < math.log(v) < hi_s - 0.01):
            why.append("%s's wheel scale ran to its limit (%.3f)" % (n, v))
    for n, v in sol["g"].items():
        if not (lo_g + 0.01 < math.log(v) < hi_g - 0.01):
            why.append("%s's gyro scale ran to its limit (%.3f)" % (n, v))
    return "; ".join(why)


SIDE_MIN_POINTS = 2         # chosen: a side from one ray end is a guess (2026-10-23 dry run: a robot taken for a board)
SIDE_NEAR_CM = 6.0          # chosen: a ray end this near its side's median agrees with it (simulator spread 0-8 cm)
SIDE_KIND_SHARE = 0.66      # chosen: two thirds of the agreeing ray ends agree on what it is (board or edge)


def rect_check(rect):
    """The board's four sides, checked before they are trusted: at least SIDE_MIN_POINTS ray ends near each side's
    line, at least half of the side's ray ends among them (one stray end, e.g. a robot met, is allowed), and two
    thirds of those agreeing on what the side is. Returns '' or why not."""
    a = math.radians(rect["angle_deg"])
    ca, sa = math.cos(-a), math.sin(-a)
    why = []
    for name, (v, sd) in rect["lines"].items():
        pts = sd.get("pts") or []
        if len(pts) < SIDE_MIN_POINTS:
            why.append("side %s met by %d ray" % (name, len(pts)))
            continue
        coords = [(ca * p[0] - sa * p[1]) if name in ("x0", "x1") else (sa * p[0] + ca * p[1]) for p in pts]
        if sd.get("kind") == "board":
            # a front stop well inside the board met something standing on it (a robot, parked or left out of the
            # fit), not the board: not counted against the side (2026-10-24 dry run: three such stops, 25-60 cm short
            # of the board, failed a side that four stops agreed on)
            inward = 1.0 if name.endswith("0") else -1.0
            keep = [(p, c) for p, c in zip(pts, coords) if (c - v) * inward <= SIDE_NEAR_CM]
            pts, coords = [k[0] for k in keep], [k[1] for k in keep]
        near = [p for p, c in zip(pts, coords) if abs(c - v) <= SIDE_NEAR_CM]
        if len(near) < SIDE_MIN_POINTS or len(near) < 0.5 * len(pts):
            why.append("side %s: only %d of %d ray ends agree where it is" % (name, len(near), len(pts)))
            continue
        kinds = [p[2] for p in near]
        if kinds.count(sd["kind"]) / float(len(kinds)) < SIDE_KIND_SHARE:
            why.append("side %s is %s in only %d of %d rays" % (name, sd["kind"], kinds.count(sd["kind"]), len(kinds)))
    return "; ".join(why)


def own_to_floor_map(own_xy, floor_xy):
    """A 2x2 linear map with a shift (a turn, a size and, if the camera's view needs it, a mirror) from a robot's own
    frame to the provisional floor frame, from matched points: (A, t) with floor = A @ own + t, or None. Used while
    exploring to know which way a robot's ray points on the floor (to keep rays off the other robots)."""
    P = np.asarray(own_xy, float)
    W = np.asarray(floor_xy, float)
    if len(P) < 4 or np.ptp(P[:, 0]) + np.ptp(P[:, 1]) < 10.0:
        return None
    best = None
    for flip in (1.0, -1.0):
        Pf = P * np.array([1.0, flip])
        mp, mw = Pf.mean(0), W.mean(0)
        A_, B_ = Pf - mp, W - mw
        U, S, Vt = np.linalg.svd(B_.T @ A_)
        D = np.eye(2)
        if np.linalg.det(U @ Vt) < 0:
            D[1, 1] = -1
        R = U @ D @ Vt
        s = float(np.trace(np.diag(S) @ D) / max(1e-9, (A_ ** 2).sum()))
        err = float(np.sum(np.linalg.norm((s * (R @ A_.T)).T - B_, axis=1) ** 2))
        if best is None or err < best[0]:
            A = s * R @ np.diag([1.0, flip])
            best = (err, A, mw - A @ P.mean(0))
    return best[1], best[2]
