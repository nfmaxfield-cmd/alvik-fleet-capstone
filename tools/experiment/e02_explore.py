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


def rebuild(stops, s=1.0, g=1.0):
    """The path (x, y, th) of the stops in the robot's own frame, its distances x s and turns x g."""
    x, y, th = (0.0, 0.0, 0.0) if "dth" in stops[0] else (stops[0]["x"], stops[0]["y"], stops[0]["th"])
    out = [(x, y, th)]
    for d, dth in segments(stops):
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
    sol = fit_once(robots, size, g_fixed)
    sol["implausible"] = pre
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
    if not n_out:
        sol["outliers"] = pre
        sol["skipped"] = {n: [i for i, q in enumerate(v) if q.get("skip")] for n, v in robots.items()}
        return sol
    sol2 = fit_once(robots2, size, g_fixed)
    sol2["implausible"] = pre
    sol2["outliers"] = n_out + pre
    sol2["skipped"] = {n: [i for i, q in enumerate(v) if q.get("skip")] for n, v in robots2.items()}
    return sol2


def fit_once(robots, size, g_fixed=None):
    """robots: {name: [stop dict(x, y, th (its own pose), u, v (pixels of its top))]}, in time order; the first
    robot's own frame is the world frame (the one the camera saw at most stops). Returns dict(f, k1, rvec, tvec, frames {name: (x0, y0, turn)},
    s {name}, g {name}, rms_px, n, per_robot_rms_px)."""
    frames0, scales0, (_, f0, rv0, tv0), G0 = _init(robots, size)
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
    f_prior = math.log(size[0] / (2 * math.tan(math.radians(F_PRIOR_HFOV_DEG) / 2)))
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

    def resid(x):
        lf, k1, rv, tv, fr, lsd, lgd = unpack(x)
        out = []
        for n in names:
            g = g_fixed[n] if g_fixed and n in g_fixed else math.exp(lgd[n])
            w = to_world(rebuild(robots[n], math.exp(lsd[n]), g), fr[n])
            obj = [(p[0], p[1], ROBOT_TOP_CM) for p in w]
            uv = project(obj, math.exp(lf), k1, rv, tv, size)
            out.append(((uv - uv_all[n]) * use[n]).ravel())
        out.append(np.array([(lf - f_prior) / F_PRIOR_SD_LOG * 3.0, k1 / K1_PRIOR_SD * 3.0]))
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
    x0 = np.clip(np.array(x0, float), np.array(lo) + 1e-9, np.array(hi) - 1e-9)
    sol = least_squares(resid, x0, loss="huber", f_scale=3.0, max_nfev=4000, bounds=(lo, hi), method="trf")
    lf, k1, rv, tv, fr, lsd, lgd = unpack(sol.x)
    r = resid(sol.x)[:-2].reshape(-1, 2)
    per, i, res_n = {}, 0, {}
    lf, k1, rv, tv, fr, lsd, lgd = unpack(sol.x)
    for n in names:
        k = len(robots[n])
        g = g_fixed[n] if g_fixed and n in g_fixed else math.exp(lgd[n])
        w = to_world(rebuild(robots[n], math.exp(lsd[n]), g), fr[n])
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
                rms_px=float(np.sqrt(np.mean(np.sum(r ** 2, 1)))), per_robot_rms_px=per, resid=res_n,
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


def contacts(robots, sol, robot_clear_cm=16.0, ahead=None):
    """The rays' contact points in the world frame: [(x, y, kind, heading deg)]. A ray stopped by another robot is
    left out: where that robot stood at that moment (its last stop before then, or its first one if it had not
    moved yet: 2026-10-23 dry run, Center's ray met Right, which had not explored yet, and only robots' end places
    were checked). ahead: {kind: cm} to override e02_map's reach."""
    paths = {n: to_world(rebuild(st, sol["s"][n], sol["g"][n]), sol["frames"][n]) for n, st in robots.items()}

    def where(m, t):
        st = robots[m]
        k = 0
        for i, q in enumerate(st):
            if q.get("t") is not None and t is not None and q["t"] <= t:
                k = i
        return paths[m][k]

    out = []
    for n, stops in robots.items():
        for (x, y, th), q in zip(paths[n], stops):
            if not q.get("contact"):
                continue
            kind = q["contact"]
            if abs(q.get("dth", 0.0) or 0.0) > SWERVE_DEG and q.get("d") is not None:
                continue                  # it turned while stopping (2026-10-23: -6.9 deg): where it met is unsure
            if ahead and kind in ahead:
                c = (x + ahead[kind] * math.cos(math.radians(th)), y + ahead[kind] * math.sin(math.radians(th)))
            else:
                import e02_map as MAP
                c = MAP.contact_point(x, y, th, kind, q.get("cue", ""))
            if any(math.hypot(c[0] - o[0], c[1] - o[1]) < robot_clear_cm
                   for m in robots if m != n for o in [where(m, q.get("t"))]):
                continue
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
