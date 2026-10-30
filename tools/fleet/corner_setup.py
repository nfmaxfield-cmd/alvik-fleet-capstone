#!/usr/bin/env python3
"""Corners-only setup (Nick, 2026-10-29 20:20-20:27): the user does nothing but click the table's 4 corners, 1-2-3-4,
in the overhead picture and the same 1-4 in the side picture. Corner 1 is 0,0; the 1 -> 2 side is x; y runs 1 -> 4.

Everything else is MEASURED, with the robots allowed to move:

  1. Each camera, from its 4 corners and its lens model (K, dist from the calibration it already has), gives its pose
     and the table's shape (length over width) up to one unknown: the scale. (A rectangle seen by a camera whose lens
     is known fixes everything but its size; the right angle at the corners is one spare check, `square_deg`.)
  2. The scale comes from things of known size seen by the cameras:
       lights  each Alvik's two built-in lights (left_led, right_led) sit LIGHT_SPACING_CM apart (a fixed hardware
               spacing, measured from 28 Oct's pictures of the known 122 x 122 plywood: see LIGHT_SPACING_CM). Primary.
       roll    a robot rolls ROLL_CM on its wheels (`go`), its lights flashed before and after: the wheels' own
               distance (with the stored wheel_scale 0.979) against the cameras'. Second check.
       body    the robot's 9.5 cm body, its outline in the overhead picture where it stood before the roll. Third check.
     Each gives the scale on its own (the three are compared and a disagreement is said in plain words); one joint fit
     of both cameras uses all of them together (`solve`).
  3. The two cameras are lined up by the robots' lights, which both cameras see (the job the chessboard corner piece
     did): the joint fit puts each light at ONE floor spot for both cameras, and how far apart the cameras' spots are
     after the fit says how well they line up (`cams_agree_cm`). A side picture numbered differently from the
     overhead's is caught here (and from the table's shape), before anything is saved.
  4. Out: each camera's model (the studio form e02_camera.load_camera reads), its height and tilt, the table's measured
     length x width ("your table measures 76 x 137 cm"), and the checks.

The math in one line: for a camera whose position is known up to the scale s (cm per unit), two pixels at the same
height h above the table that are D cm apart give s = (D / |b1 - b2| + h) / C_z, where b = the ray's direction
divided by its downward part and C_z the camera's height in units. Lights, roll and body all reduce to that.

    python3 tools/fleet/corner_setup.py --selftest            # the math on drawn cameras (no pictures, no robots)

Used by mission_control.py (Setup screen, "Measure") through `run_live` and by the tests (tools/fleet/tests/
test_corner_setup.py, the simulator's two cameras on Monday's three tables and the 122 x 122 board).
"""
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, TOOLS, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np    # noqa: E402

# ------------------------------------------------------------------------------------------------- the numbers
LIGHT_SPACING_CM = 7.08   # MEASURED, not from a datasheet (Arduino's AKX00066 datasheet names the two top LEDs DL1 /
                          # DL2 but gives no positions). From 28 Oct 19:17's 22 find steps (42 light pairs seen by both
                          # cameras) with the 27 Oct 12:58 corner clicks, as the spacing that makes this setup measure
                          # the known 122.15 x 121.9 plywood: 7.08 cm. Each half of the steps alone: 6.95 and 7.22;
                          # each light triangulated by the two plywood-calibrated cameras: 7.01 (median). How sure:
                          # +-0.15 cm (2 %). Report appendix A. No DOE: a ruler across one robot's two lights (lit, seen
                          # from straight above) settles it to 0.5 mm
LIGHT_SPACING_SD_CM = 0.15  # the spacing's own uncertainty (above). Its weight in the joint fit
LIGHT_Z_CM = 3.7          # the lights' height above the table: the robot's top (e02_camera.ROBOT_TOP_CM). The overhead's
                          # scale barely depends on it (a 1 cm error moves the scale 0.4 % at 260 cm); the side's more.
                          # Appendix A also gives the triangulated height. No DOE: a ruler
BODY_CM = 9.5             # the body's length and width (e02_camera.ROBOT_CM). The third, weakest check: the outline
                          # also holds the wheels, a shadow and the walls the overhead sees near the picture's edge
BODY_SD_CM = 0.6          # chosen: about 6 % for the outline's own error (above). DOE note: the body check against the
                          # lights on 5 still robots across the board, its spread sets this
ROLL_CM = 10.0            # each robot's roll (`go 10`, then `go -10` back to its spot): 10 is the firmware's longest
                          # backward leg (e02_back_max_cm), so either order brings it back. DOE note: longer rolls measure
                          # better (the cameras' error is a fixed ~0.5 px per end), but need room; 10 / 15 / 20 cm on the
                          # board against the roll check's spread would choose it
ROLL_SD_FRAC = 0.015      # chosen: the wheels' distance error, 1.5 % (wheel_scale 0.979 is itself a fit; the
                          # simulator's slip is 0.2 %). DOE note: yes, the roll check against the lights over a week of
                          # setups gives the wheels' real spread
ROLL_SD_CM = 0.2          # plus a fixed 0.2 cm (where the leg ends). Chosen with ROLL_SD_FRAC
CLICK_SD_PX = 2.0         # chosen: a person's click on a table corner (the corner is a few px of blur). DOE note: Nick
                          # clicks the same 4 corners 5 times; their spread sets it
SPOT_SD_PX = 0.5          # chosen: a light's spot centre (weighted, 5-30 px area). DOE note: the same light over a
                          # still robot's pictures, its spread
EDGE_ROOM_CM = 5.0        # the roll keeps the body this far from an edge (plus its half, 4.75). Chosen like the
                          # calibration room's 3 cm margin + 2 cm camera; no DOE: a safety room
AGREE_FRAC = 0.02         # a check further than 2 % from the primary (the lights) is said as a disagreement and left
                          # out of the result. Chosen: 2 % of a 137 cm table is 2.7 cm, about the tracking error Monday
                          # tolerates, and about each check's own systematic error (lights 2 %, wheels 1.5 %), so a
                          # tighter line would flag good setups. DOE note: yes, from the checks' spread over many real
                          # setups (each setup logs all three)
CAMS_AGREE_CM = 2.0       # the two cameras further apart than this on the same light (after the fit): said as not
                          # lined up. Chosen: the side camera alone is good to about 2-3 cm (29 Oct). No DOE: a warning
SQUARE_WARN_DEG = 4.0     # a camera's 4 corners that need more than this off a right angle (with its lens) are said as
                          # "check the clicks or the lens". Chosen; no DOE: a warning


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------------------------------------ geometry helpers
def undistort(px, K, dist):
    """Pixels (n, 2) -> normalised image coordinates (n, 2) (the lens model's distortion removed)."""
    import cv2
    p = np.asarray(px, np.float64).reshape(-1, 1, 2)
    if dist is not None and np.any(np.asarray(dist, float)):
        crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-12)
        return cv2.undistortPointsIter(p, np.asarray(K, float), np.asarray(dist, float), None, None, crit).reshape(-1, 2)
    K = np.asarray(K, float)
    return (p.reshape(-1, 2) - K[:2, 2]) @ np.linalg.inv(K[:2, :2]).T


def homography(src, dst):
    """3 x 3 H with dst ~ H src (4+ points, float64 DLT, normalised)."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)

    def norm(p):
        c = p.mean(0)
        s = math.sqrt(2) / max(1e-12, np.mean(np.linalg.norm(p - c, axis=1)))
        T = np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1.0]])
        return T, (p - c) * s
    Ts, a = norm(src)
    Td, b = norm(dst)
    A = []
    for (x, y), (u, v) in zip(a, b):
        A.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        A.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, Vt = np.linalg.svd(np.array(A))
    H = Vt[-1].reshape(3, 3)
    H = np.linalg.inv(Td) @ H @ Ts
    return H / H[2, 2]


class Pose:
    """A camera's pose in the table frame measured in units of the table's WIDTH (W = 1, x along 1 -> 2, y along 1 -> 4,
    z up towards the camera): R (world -> camera), t, so p_cam = R p + t. Times the scale s (cm per unit) it is the real
    pose. aspect = length / width; square_deg: how far the corners are from a right angle with this lens (0 for a
    perfect rectangle and lens); mirrored: the clicks go round clockwise seen from above."""

    def __init__(self, R, t, aspect, square_deg, mirrored, K, dist, size):
        self.R, self.t = np.asarray(R, float), np.asarray(t, float)
        self.aspect, self.square_deg, self.mirrored = float(aspect), float(square_deg), bool(mirrored)
        self.K, self.dist, self.size = np.asarray(K, float), np.asarray(dist, float).ravel(), tuple(size)

    @property
    def C(self):
        return -self.R.T @ self.t

    def b(self, px):
        """For pixels (n, 2): the ray direction over its downward part, (n, 2) (x, y), so a point h units below the
        camera on that ray is C_xy + h * b... see scale_from."""
        n = undistort(px, self.K, self.dist)
        d = np.column_stack([n, np.ones(len(n))]) @ self.R        # R.T @ [xn, yn, 1] per row: world directions
        return d[:, :2] / (-d[:, 2:3])                            # z is up, the rays go down: -d_z > 0

    def floor(self, px, z_units):
        """Pixels -> (n, 2) table points (units) on the plane z = z_units."""
        C = self.C
        return C[:2] + (C[2] - z_units) * self.b(px)


def rect_pose(corners_px, K, dist, size):
    """A camera's pose up to scale from the table's 4 corners clicked as 1, 2, 3, 4 (1 = 0,0; 2 along x; 4 along y).
    Returns Pose (W = 1 unit)."""
    n = undistort(corners_px, K, dist)
    H = homography([[0, 0], [1, 0], [1, 1], [0, 1]], n)          # unit square -> normalised image
    h1, h2, h3 = H[:, 0], H[:, 1], H[:, 2]
    if h3[2] < 0:                                                # corner 1 must be in front of the camera
        h1, h2, h3 = -h1, -h2, -h3
    a, b = np.linalg.norm(h1), np.linalg.norm(h2)
    cosang = float(np.dot(h1, h2) / (a * b))
    square_deg = abs(90.0 - math.degrees(math.acos(max(-1.0, min(1.0, cosang)))))
    r1, r2 = h1 / a, h2 / b
    r3 = np.cross(r1, r2)
    U, _, Vt = np.linalg.svd(np.column_stack([r1, r2, r3]))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R = U @ np.diag([1, 1, -1]) @ Vt
    t = h3 / b                                                   # W = 1 unit
    aspect = a / b
    C = -R.T @ t
    mirrored = C[2] < 0                                          # the camera below the table: y runs the other way
    return Pose(R, t, aspect, square_deg, mirrored, K, dist, size)


def scale_from(pose, pairs_px, length_cm, z_cm):
    """The scale s (cm per unit) from segments of known length at a known height. pairs_px: [(px_a, px_b)]. One s per
    pair: s = (D / |b_a - b_b| + z) / C_z (module doc)."""
    out = []
    Cz = pose.C[2]
    for a, b in pairs_px:
        ba, bb = pose.b(np.array([a, b], float))
        d = float(np.linalg.norm(ba - bb))
        if d <= 1e-9:
            continue
        L = length_cm if np.ndim(length_cm) == 0 else None
        out.append((L / d + z_cm) / Cz)
    return out


def scale_from_lengths(pose, pairs_px, lengths_cm, z_cm):
    return [s for (a, b), L in zip(pairs_px, lengths_cm) for s in scale_from(pose, [(a, b)], L, z_cm)]


def robust_mean(v):
    """(mean, sd of the mean, n) after dropping values over 3 MADs from the median."""
    v = np.asarray([x for x in v if x is not None and np.isfinite(x)], float)
    if len(v) == 0:
        return None, None, 0
    med = np.median(v)
    mad = np.median(np.abs(v - med)) * 1.4826
    keep = v[np.abs(v - med) <= 3 * mad + 1e-9] if mad > 0 else v
    sd = float(np.std(keep, ddof=1)) if len(keep) > 1 else float("nan")
    return float(np.mean(keep)), (sd / math.sqrt(len(keep)) if len(keep) > 1 else float("nan")), int(len(keep))


# --------------------------------------------------------------------------------------------- numbering checks
def corner_words(poses):
    """Plain-words problems with the clicks, before anything moves: a numbering that goes round clockwise (the robots
    would turn the wrong way in that frame), the two pictures numbered differently, corners far off a right angle."""
    out = []
    names = list(poses)
    for c, p in poses.items():
        if p.mirrored:
            out.append("The %s picture's corners go round the table the other way (clockwise seen from above). Keep "
                       "corner 1 where it is and swap 2 and 4: 1 -> 2 runs along the side you want as x, and 3 is "
                       "the corner opposite 1." % c)
        elif p.square_deg > SQUARE_WARN_DEG:
            out.append("The %s picture's 4 corners are %.0f deg off a right angle with this camera's lens: check the "
                       "clicks (each on the table's very corner)." % (c, p.square_deg))
    if len(names) == 2 and not any(poses[c].mirrored for c in names):
        a, b = poses[names[0]].aspect, poses[names[1]].aspect
        if abs(math.log(a / b)) > abs(math.log(a * b)) + 0.05 and abs(math.log(a)) > 0.15:
            out.append("The two pictures are numbered differently: in the %s picture 1 -> 2 is the %s side, in the %s "
                       "picture the %s side. Click the same table corner as 1 in both, then 2 the same, and so on."
                       % (names[0], "long" if a > 1 else "short", names[1], "long" if b > 1 else "short"))
        if poses[names[0]].mirrored != poses[names[1]].mirrored:
            pass
    return out


# ------------------------------------------------------------------------------------------------- the joint fit
def _rvec(R):
    import cv2
    return cv2.Rodrigues(np.asarray(R, float))[0].ravel()


def _R(rv):
    import cv2
    return cv2.Rodrigues(np.asarray(rv, float).reshape(3, 1))[0]


def project(K, dist, rv, tv, pts):
    import cv2
    uv, _ = cv2.projectPoints(np.asarray(pts, float).reshape(-1, 1, 3), np.asarray(rv, float), np.asarray(tv, float),
                              np.asarray(K, float), np.asarray(dist, float))
    return uv.reshape(-1, 2)


def solve(obs, lights=None, body_cm=BODY_CM):
    """The whole measure. obs:
        cameras: {name: {K, dist, size, corners_px: [[u, v] x 4] as 1, 2, 3, 4}}
        spots:   {id: {name: [u, v]}}: each light seen (an id per light per moment: "Left/start/L"), by camera
        pairs:   [(id_a, id_b)]: the two lights of one robot at one moment (LIGHT_SPACING_CM apart)
        rolls:   [(id_a, id_b, cm)]: the same light before and after a roll the wheels measured as cm
        bodies:  [{camera, corners_px: 4 x [u, v] (its outline's rectangle, going round), z_cm}]
    lights: {"spacing_cm", "z_cm", "spacing_sd_cm"} (default: the Alvik's).
    Returns dict: ok, words (problems), size_cm (length, width), checks {lights, roll, body: their size and n},
    agree (words), cameras {name: {model (studio dict), height_cm, tilt_deg, corners_err_px}}, cams_agree_cm,
    points {id: (x, y)}."""
    from scipy.optimize import least_squares
    L = dict(spacing_cm=LIGHT_SPACING_CM, z_cm=LIGHT_Z_CM, spacing_sd_cm=LIGHT_SPACING_SD_CM)
    L.update(lights or {})
    D, zl = float(L["spacing_cm"]), float(L["z_cm"])
    cams = obs["cameras"]
    names = list(cams)
    poses = {c: rect_pose(cams[c]["corners_px"], cams[c]["K"], cams[c].get("dist") or [0] * 5, cams[c]["size"])
             for c in names}
    words = corner_words(poses)
    if any(p.mirrored for p in poses.values()) or any("numbered differently" in w for w in words):
        return dict(ok=False, words=words, poses={c: _pose_info(p) for c, p in poses.items()})
    spots, pairs = obs.get("spots") or {}, obs.get("pairs") or []
    rolls, bodies = obs.get("rolls") or [], obs.get("bodies") or []

    # 1. each check alone, per camera (closed form): the width in cm each one gives
    per = {"lights": {}, "roll": {}, "body": {}}
    for c in names:
        p = poses[c]
        pp = [(spots[a][c], spots[b][c]) for a, b in pairs if c in spots.get(a, {}) and c in spots.get(b, {})]
        if pp:
            per["lights"][c] = scale_from(p, pp, D, zl)
        rr = [(spots[a][c], spots[b][c], cm) for a, b, cm in rolls if c in spots.get(a, {}) and c in spots.get(b, {})]
        if rr:
            per["roll"][c] = scale_from_lengths(p, [(a, b) for a, b, _ in rr], [cm for _, _, cm in rr], zl)
        bb = [q for q in bodies if q.get("camera") == c]
        if bb:
            vals = []
            for q in bb:
                cp = q["corners_px"]
                sides = [(cp[k], cp[(k + 1) % 4]) for k in range(4)]
                vals += scale_from(p, sides, body_cm, float(q.get("z_cm", zl)))
            per["body"][c] = vals
    checks = {}
    cam_disagree = []
    for k, d in per.items():
        allv, cam_sizes = [], {}
        for c, v in d.items():
            m, sd, n = robust_mean(v)
            if m is None:
                continue
            allv += list(v)
            cam_sizes[c] = dict(width=round(m, 2), length=round(m * poses[c].aspect, 2), n=n,
                                sd=None if sd is None or not np.isfinite(sd) else round(sd, 3))
        if cam_sizes:
            # the check's width: its cameras' widths, each weighted by its own spread; when two cameras disagree by more
            # than AGREE_FRAC, the one looking most straight down alone (its scale hardly depends on its lens model,
            # an oblique camera's does: 28 Oct's side camera read 6.5 % large)
            vals = sorted(cam_sizes.items(), key=lambda kv: tilt_of(poses[kv[0]]))
            spread = (max(v["width"] for _, v in vals) - min(v["width"] for _, v in vals)) / vals[0][1]["width"]
            if len(vals) > 1 and spread > AGREE_FRAC:
                cam_disagree.append((k, vals[0][0], vals[-1][0], vals[-1][1]["width"] / vals[0][1]["width"] - 1))
                vals = vals[:1]
            ws = [(v["width"], (v["sd"] or v["width"] * 0.02) ** -2) for _, v in vals]
            w = sum(a * b for a, b in ws) / sum(b for _, b in ws)
            sd = sum(b for _, b in ws) ** -0.5
            asp = np.mean([poses[c].aspect for c in cam_sizes])
            sys_frac = (float(L.get("spacing_sd_cm", LIGHT_SPACING_SD_CM)) / D) if k == "lights" else SYS_FRAC[k]
            sd = math.hypot(sd, sys_frac * w)              # plus the check's own systematic part (its known length)
            checks[k] = dict(width=round(w, 2), length=round(w * asp, 2), sd_width=round(sd, 3), cameras=cam_sizes)
    if "lights" not in checks and "roll" not in checks:
        return dict(ok=False, words=words + ["No robot's lights were seen by the cameras, so nothing gives the scale: "
                                             "are the robots on the table and switched on?"],
                    checks=checks, poses={c: _pose_info(p) for c, p in poses.items()})
    # which checks go into the fit: the primary (the lights; the rolls when no light pair was seen) always; another
    # only when it agrees with the primary (AGREE_FRAC and 2.5 sd); one that disagrees is said and left out
    prim = "lights" if "lights" in checks else "roll"
    use = {prim}
    left_out = []
    for k in ("lights", "roll", "body"):
        if k == prim or k not in checks:
            continue
        a, b = checks[prim], checks[k]
        diff = abs(a["width"] - b["width"]) / a["width"]
        if diff > AGREE_FRAC:
            left_out.append(k)
        else:
            use.add(k)
    w0 = checks[prim]["width"]

    # 2. the fit, in two stages. (a) The size: the camera looking most straight down, with every light it saw, the
    # lights' spacing, the rolls and the bodies; its scale hardly depends on its lens model. (b) Each other camera lined
    # up on it: its pose (and, tilted, its focal length) from its own 4 corners at that size AND the same lights at the
    # floor spots (a) found, so both cameras put every robot in one place. (One joint fit of both was tried first: on 28
    # Oct's real pictures the side camera's lens model, good on the floor, read things 4 cm up 6.5 % large and pulled
    # the size 5 % off; appendix B of the report.)
    from cameras import CameraModel
    order = sorted(names, key=lambda c: tilt_of(poses[c]))
    main_c = order[0]
    sdl = float(L.get("spacing_sd_cm", LIGHT_SPACING_SD_CM))
    ids = sorted(i for i in spots if main_c in spots[i])
    x0 = [w0 * poses[main_c].aspect, w0] + list(_rvec(poses[main_c].R)) + list(poses[main_c].t * w0)
    for i in ids:
        x0 += list(poses[main_c].floor(np.array([spots[i][main_c]]), zl / w0)[0] * w0)
    body_q = [q for q in (bodies if "body" in use else []) if q.get("camera") == main_c]
    for q in body_q:
        x0 += list((poses[main_c].floor(np.array(q["corners_px"], float), float(q.get("z_cm", zl)) / w0) * w0).ravel())
    x0 = np.array(x0, float)
    idx = {i: 8 + 2 * k for k, i in enumerate(ids)}
    bstart = 8 + 2 * len(ids)
    cc = cams[main_c]
    dist_m = cc.get("dist") or [0] * 5

    def resid_a(x):
        Lc, Wc = x[0], x[1]
        rv, tv = x[2:5], x[5:8]
        r = list(((project(cc["K"], dist_m, rv, tv, [[0, 0, 0], [Lc, 0, 0], [Lc, Wc, 0], [0, Wc, 0]])
                   - np.asarray(cc["corners_px"], float)) / CLICK_SD_PX).ravel())
        if ids:
            P = np.array([[x[idx[i]], x[idx[i] + 1], zl] for i in ids])
            r += list(((project(cc["K"], dist_m, rv, tv, P) - np.array([spots[i][main_c] for i in ids], float))
                       / SPOT_SD_PX).ravel())
        for a, b in (pairs if "lights" in use else []):
            if a in idx and b in idx:
                r.append((math.hypot(x[idx[a]] - x[idx[b]], x[idx[a] + 1] - x[idx[b] + 1]) - D) / sdl)
        for a, b, cm in (rolls if "roll" in use else []):
            if a in idx and b in idx:
                r.append((math.hypot(x[idx[a]] - x[idx[b]], x[idx[a] + 1] - x[idx[b] + 1]) - cm)
                         / (ROLL_SD_FRAC * cm + ROLL_SD_CM))
        j = bstart
        for q in body_q:
            P = x[j: j + 8].reshape(4, 2)
            zb = float(q.get("z_cm", zl))
            r += list(((project(cc["K"], dist_m, rv, tv, np.column_stack([P, np.full(4, zb)]))
                        - np.asarray(q["corners_px"], float)) / 1.5).ravel())
            for kk in range(4):
                r.append((math.hypot(*(P[kk] - P[(kk + 1) % 4])) - body_cm) / BODY_SD_CM)
            j += 8
        return np.array(r)

    sol = least_squares(resid_a, x0, x_scale="jac", max_nfev=4000)
    x = sol.x
    Lc, Wc = float(x[0]), float(x[1])
    try:
        cov = np.linalg.pinv(sol.jac.T @ sol.jac)
        sdL, sdW = float(math.sqrt(max(0, cov[0, 0]))), float(math.sqrt(max(0, cov[1, 1])))
    except Exception:     # noqa: BLE001
        sdL = sdW = float("nan")
    pts = {i: (float(x[idx[i]]), float(x[idx[i] + 1])) for i in ids}
    fitted = {main_c: (np.array(cc["K"], float), x[2:5].copy(), x[5:8].copy(), False)}
    corners3 = np.array([[0, 0, 0], [Lc, 0, 0], [Lc, Wc, 0], [0, Wc, 0]], float)

    # (b) the other cameras, lined up on the lights
    for c in order[1:]:
        cc2 = cams[c]
        d2 = cc2.get("dist") or [0] * 5
        K0 = np.array(cc2["K"], float)
        shared = [i for i in pts if c in spots[i]]
        free = tilt_of(poses[c]) > FREE_F_TILT_DEG and len(shared) >= 6
        p0 = poses[c]
        y0 = np.array(list(_rvec(p0.R)) + list(p0.t * Wc) + ([0.0] if free else []), float)

        def Kf(y):
            if not free:
                return K0
            K = K0.copy()
            K[0, 0] *= math.exp(y[6])
            K[1, 1] *= math.exp(y[6])
            return K

        def resid_b(y):
            K = Kf(y)
            r = list(((project(K, d2, y[:3], y[3:6], corners3) - np.asarray(cc2["corners_px"], float))
                      / CLICK_SD_PX).ravel())
            if shared:
                P = np.array([[pts[i][0], pts[i][1], zl] for i in shared])
                r += list(((project(K, d2, y[:3], y[3:6], P) - np.array([spots[i][c] for i in shared], float))
                           / LINEUP_SD_PX).ravel())
            return np.array(r)
        sb = least_squares(resid_b, y0, x_scale="jac", max_nfev=2000)
        fitted[c] = (Kf(sb.x), sb.x[:3].copy(), sb.x[3:6].copy(), free)

    out_cams = {}
    models = {}
    for c in names:
        K, rv, tv, free = fitted[c]
        cc2 = cams[c]
        d2 = cc2.get("dist") or [0] * 5
        m = CameraModel(K, d2, rv, tv, cc2["size"], name=c)
        e = np.linalg.norm(project(K, d2, rv, tv, corners3) - np.asarray(cc2["corners_px"], float), axis=1)
        ps = m.pose_summary()
        m.fit = dict(method="corners-only setup (tools/fleet/corner_setup.py): 4 clicked corners; the scale from the "
                            "robots' lights, rolls and bodies seen by the %s camera; %s" % (
                                main_c, "its size" if c == main_c else "lined up on the same lights"),
                     corners_err_px=[round(float(v), 2) for v in e])
        models[c] = m
        d = m.to_dict()
        d["name"] = c
        out_cams[c] = dict(model=d, height_cm=round(ps["camera_position_cm"][2], 1),
                           position_cm=ps["camera_position_cm"], tilt_deg=ps["tilt_from_vertical_deg"],
                           corners_err_px=round(float(e.max()), 2), square_deg=round(poses[c].square_deg, 2),
                           f_px=round(float(K[0, 0]), 1), f_given_px=round(float(np.array(cc2["K"], float)[0, 0]), 1),
                           f_refit=bool(free), role="size" if c == main_c else "lined up")
    # the cameras lined up: each light through each camera's own new model, the two floor spots compared
    disagree = []
    for i in spots:
        seen = [c for c in spots[i] if c in models]
        if len(seen) == 2:
            a = models[seen[0]].to_floor(*spots[i][seen[0]], z=zl)
            b = models[seen[1]].to_floor(*spots[i][seen[1]], z=zl)
            disagree.append(math.hypot(a[0] - b[0], a[1] - b[1]))
    cams_agree = round(float(np.median(disagree)), 2) if disagree else None
    cams_worst = round(float(np.max(disagree)), 2) if disagree else None
    for k in checks:                      # each check's length: its width times the fit's shape
        checks[k]["length"] = round(checks[k]["width"] * Lc / Wc, 2)
    agree = agreement_words(checks, Lc, Wc, prim, left_out)
    for k, good, bad, frac in cam_disagree:
        cf = out_cams.get(bad, {})
        agree.append("Alone, the %s camera reads the %s %.0f %% %s than the %s camera (its lens model is less sure off "
                     "the floor%s). The size comes from the %s camera, which looks most straight down." % (
                         bad, NAMES[k], abs(100 * frac), "larger" if frac > 0 else "smaller", good,
                         "; its focal length was refitted from the lights, %.0f -> %.0f px" % (cf["f_given_px"], cf["f_px"])
                         if cf.get("f_refit") else "", good))
    if cams_agree is not None and cams_agree > CAMS_AGREE_CM:
        agree.append("The two cameras put the same light %.1f cm apart (median): they are not lined up well. Check "
                     "that corner 1 is the same table corner in both pictures, and each click is on the very corner."
                     % cams_agree)
    return dict(ok=True, words=words, agree=agree, used=sorted(use), left_out=left_out, size_cm=(round(Lc, 1), round(Wc, 1)),
                size_sd_cm=(round(sdL, 2), round(sdW, 2)), checks=checks, cameras=out_cams, cams_agree_cm=cams_agree,
                cams_worst_cm=cams_worst, points={i: (round(p[0], 2), round(p[1], 2)) for i, p in pts.items()},
                lights=L, size_from=main_c, fit_cost=round(float(sol.cost), 2),
                poses={c: _pose_info(p) for c, p in poses.items()})


LINEUP_SD_PX = 1.5        # chosen: a light's spot in the second camera against the floor spot the first camera gave it
                          # (both spots' errors and the light's height). Its weight against the corners (2 px). DOE note:
                          # the cams-agree distance over many setups at 1 / 1.5 / 3 px


FREE_F_TILT_DEG = 15.0    # a camera tilted more than this from straight down gets its focal length refitted in the joint
                          # fit (when both cameras see 6+ lights). No DOE: below it the focal length and the height
                          # cannot be told apart (the straight-down overhead), above it they can


def tilt_of(p):
    """Degrees from straight down of a Pose's view axis."""
    axis = p.R.T @ np.array([0.0, 0.0, 1.0])
    return math.degrees(math.acos(max(-1.0, min(1.0, -axis[2]))))


def _pose_info(p):
    return dict(aspect=round(p.aspect, 4), square_deg=round(p.square_deg, 2), mirrored=p.mirrored)


def agreement_words(checks, Lc, Wc, prim="lights", left_out=()):
    """Each check against the primary in plain words: one that disagrees (over AGREE_FRAC and 2.5 sd) was left out of
    the result, and the words say by how much and what to do."""
    out = []
    a = checks.get(prim)
    for k in left_out:
        b = checks[k]
        diff = (b["width"] - a["width"]) / a["width"]
        out.append("The %s say %.0f x %.0f cm, %.0f %% %s than the %s (%.0f x %.0f): left out of the result. %s"
                   % (NAMES[k], b["length"], b["width"], abs(100 * diff), "larger" if diff > 0 else "smaller",
                      NAMES[prim], a["length"], a["width"], ADVICE[k]))
    return out


ADVICE = {"roll": "A robot's wheels may have slipped, or its stored wheel_scale is off: check one robot's 10 cm roll "
                  "with a ruler.",
          "body": "The body is the weakest check (its outline also holds wheels and shadow); if the lights and the "
                  "rolls agree, nothing to do.",
          "lights": "Check that every robot's two lights are clear and on top."}
SYS_FRAC = {"lights": LIGHT_SPACING_SD_CM / LIGHT_SPACING_CM, "roll": ROLL_SD_FRAC, "body": BODY_SD_CM / BODY_CM}


NAMES = {"lights": "robots' lights", "roll": "rolls on the wheels", "body": "robots' bodies"}


def result_line(res):
    """The one line the Setup screen shows."""
    if not res.get("ok"):
        return "Not measured: " + " ".join(res.get("words") or [])
    Lc, Wc = res["size_cm"]
    ch = res["checks"]
    parts = []
    for k in ("lights", "roll", "body"):
        if k in ch:
            parts.append("%s %.1f x %.1f" % (k, ch[k]["length"], ch[k]["width"]))
    cams = ", ".join("%s %.0f cm up, tilted %.0f deg" % (c, v["height_cm"], v["tilt_deg"]) for c, v in res["cameras"].items())
    s = "Your table measures %.0f x %.0f cm (checks: %s). Cameras: %s" % (Lc, Wc, "; ".join(parts), cams)
    if res.get("cams_agree_cm") is not None:
        s += "; they agree on the robots within %.1f cm" % res["cams_agree_cm"]
    s += "."
    if res.get("agree"):
        s += " WARNING: " + " ".join(res["agree"])
    return s


# ------------------------------------------------------------------------------------------- pictures -> spots
def light_spots(dark, lit, mask=None, thr=30.0, max_n=4):
    """Spots that came on between two sets of grey pictures (their medians): [(strength, u, v, area)], strongest
    first. mask: where to look (uint8, 255 = inside), e.g. the table's outline grown a little."""
    import cv2
    b = np.median(np.stack([f.astype(np.float32) for f in dark]), axis=0)
    a = np.median(np.stack([f.astype(np.float32) for f in lit]), axis=0)
    d = a - b
    d -= float(np.median(d[::4, ::4]))
    m = (d > thr).astype(np.uint8)
    if mask is not None:
        m &= (mask > 0).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    out = []
    for i in range(1, n):
        area = int(st[i, cv2.CC_STAT_AREA])
        if area < 2 or area > 400:
            continue
        x0, y0, w, h = st[i, 0], st[i, 1], st[i, 2], st[i, 3]
        sub = lab[y0:y0 + h, x0:x0 + w] == i
        vv, uu = np.nonzero(sub)
        wv = d[y0:y0 + h, x0:x0 + w][vv, uu]
        s = float(wv.sum())
        out.append((s, float((uu * wv).sum() / s) + x0, float((vv * wv).sum() / s) + y0, area))
    out.sort(key=lambda q: -q[0])
    return out[:max_n]


SNAP_PX = 8.0             # run_live(snap=True) moves a click onto the corner feature found within this many px. OFF
                          # by default: on the real 27 / 28 Oct pictures the same unmoved plywood corner snapped to spots
                          # up to 9 px apart in two pictures (plywood grain and floor texture beat the soft corner), worse
                          # than a click. DOE note: yes, before switching it on: Nick clicks each corner 5 times on real
                          # pictures; snap on / off against the clicks' spread and the measured size


def refine_corner(img, uv, r=SNAP_PX):
    """The table's corner nearest a click: the strongest corner feature within r px (Shi-Tomasi), to sub-pixel. Returns
    ((u, v), moved px) or (the click, 0.0) when none is found."""
    import cv2
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    u, v = float(uv[0]), float(uv[1])
    H, W = g.shape
    R = int(r) + 6
    x0, y0 = max(0, int(round(u)) - R), max(0, int(round(v)) - R)
    x1, y1 = min(W, int(round(u)) + R + 1), min(H, int(round(v)) + R + 1)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return (u, v), 0.0
    win = cv2.GaussianBlur(g[y0:y1, x0:x1], (3, 3), 0)
    pts = cv2.goodFeaturesToTrack(win, maxCorners=4, qualityLevel=0.2, minDistance=3, blockSize=5)
    if pts is None:
        return (u, v), 0.0
    pts = pts.reshape(-1, 2) + [x0, y0]
    d = np.hypot(pts[:, 0] - u, pts[:, 1] - v)
    k = int(np.argmin(d))
    if d[k] > r:
        return (u, v), 0.0
    q = pts[k:k + 1].astype(np.float32).reshape(-1, 1, 2)
    cv2.cornerSubPix(g, q, (4, 4), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 40, 0.01))
    qu, qv = float(q[0, 0, 0]), float(q[0, 0, 1])
    m = math.hypot(qu - u, qv - v)
    if m > r:
        return (u, v), 0.0
    return (qu, qv), m


def table_mask(shape, corners_px, grow_px=12):
    import cv2
    m = np.zeros(shape[:2], np.uint8)
    q = np.asarray(corners_px, float)
    c = q.mean(0)
    q = c + (q - c) * (1 + grow_px / max(1.0, np.mean(np.linalg.norm(q - c, axis=1))))
    cv2.fillPoly(m, [np.round(q).astype(np.int32)], 255)
    return m


def body_outline(before, after, at_px, away_px, mask=None, thr=18.0, max_r_px=None):
    """The robot's outline where it stood (at_px) from a picture with it there (before) and one after it rolled away
    (after, it is at away_px): the pixels that changed, on at_px's side of the line half way between. Returns the
    outline's rotated rectangle as 4 corners [[u, v]] going round, or None."""
    import cv2
    d = np.abs(before.astype(np.float32) - after.astype(np.float32))
    d = cv2.GaussianBlur(d, (3, 3), 0)
    m = (d > thr).astype(np.uint8)
    H, W = m.shape
    vv, uu = np.mgrid[0:H, 0:W]
    a, b = np.asarray(at_px, float), np.asarray(away_px, float)
    mid, n = (a + b) / 2, (a - b) / max(1e-9, np.linalg.norm(a - b))
    m &= (((uu - mid[0]) * n[0] + (vv - mid[1]) * n[1]) > 0).astype(np.uint8)
    if max_r_px:
        m &= ((uu - a[0]) ** 2 + (vv - a[1]) ** 2 < max_r_px ** 2).astype(np.uint8)
    if mask is not None:
        m &= (mask > 0).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    nlab, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    if nlab < 2:
        return None
    # the component nearest at_px
    best, bd = None, 1e9
    for i in range(1, nlab):
        if st[i, cv2.CC_STAT_AREA] < 20:
            continue
        ys, xs = np.nonzero(lab == i)
        dd = math.hypot(xs.mean() - a[0], ys.mean() - a[1])
        if dd < bd:
            best, bd = i, dd
    if best is None:
        return None
    ys, xs = np.nonzero(lab == best)
    rect = cv2.minAreaRect(np.column_stack([xs, ys]).astype(np.float32))
    return [[float(u), float(v)] for u, v in cv2.boxPoints(rect)]


# ------------------------------------------------------------------------------------------- the live measure
SIM_LIGHTS = dict(spacing_cm=5.1, z_cm=4.0, spacing_sd_cm=0.05)   # the simulator's Alvik (tools/sim/camera.py: lights
                        # 2.55 cm either side of the middle, on the 4.0 cm top; table_cam.SideCamera the same): the
                        # simulator's own "hardware", so a dry run measures its table with its robots
SETTLE_S, COLLECT_S = 0.35, 0.45   # as facing.py (a light's command -> seen; pictures averaged per state). DOE notes there
DARK_MS = 20000                    # every robot dark this long while the others flash (renewed per robot)


def _still(grab, c, t0, t1):
    fr = grab.between(c, t0, t1)
    return fr


def _median(frames):
    return np.median(np.stack([f.astype(np.float32) for f in frames]), axis=0).astype(np.uint8)


def run_live(streams, lens, corners, bench, names, out_dir, say=print, lights=None, roll=True, dry=False,
             firmware=None, snap=False):
    """The whole setup with the robots: flash each robot's left and right light (both cameras), roll each one ROLL_CM
    out and back on its wheels (lights flashed at the far end and back home), its body outline from before / after,
    then solve(). streams: {camera: "host:port"}; lens: {camera: CameraModel or studio dict with K, dist, image_size};
    corners: {camera: [[u, v] x 4] as 1, 2, 3, 4}; bench: ladder.Bench (robots' link); names: robots on the table.
    Writes observations.json, result.json, both cameras' models and pictures into out_dir. Returns the result (solve's
    dict, plus "line", "obs", "seconds")."""
    import cv2
    import facing as F
    os.makedirs(out_dir, exist_ok=True)
    t_start = time.time()
    L = dict(spacing_cm=LIGHT_SPACING_CM, z_cm=LIGHT_Z_CM, spacing_sd_cm=LIGHT_SPACING_SD_CM)
    L.update(lights or {})
    firmware = firmware or {}
    cams = {}
    for c, m in lens.items():
        d = m if isinstance(m, dict) else m.to_dict()
        cams[c] = dict(K=d["K"], dist=list(np.ravel(d.get("dist") or [0] * 5)), size=list(d["image_size"]),
                       corners_px=[list(map(float, q)) for q in corners[c]])
    poses = {c: rect_pose(cams[c]["corners_px"], cams[c]["K"], cams[c]["dist"], cams[c]["size"]) for c in cams}
    words = corner_words(poses)
    if any(p.mirrored for p in poses.values()) or any("numbered differently" in w for w in words):
        res = dict(ok=False, words=words, seconds=round(time.time() - t_start, 1))
        res["line"] = result_line(res)
        say(res["line"])
        return res
    import ladder as LD
    for n in names:                       # the robots' edge guard (not kept over a power-off) and stored corrections
        for c in LD.SAFE + ([] if dry else LD.corrections_cmds(n)):
            bench.send(n, c)
            time.sleep(0.02)
    grab = F.Grab(streams)
    obs = dict(cameras=cams, spots={}, pairs=[], rolls=[], bodies=[], lights=L, robots=list(names))
    try:
        if not grab.wait_first(8.0):
            missing = [c for c, b in grab.buf.items() if not b]
            raise RuntimeError("no pictures from the %s camera (is the recorder running?)" % ", ".join(missing))
        masks = {}
        for c in cams:
            f0 = _median([f for _, f, _ in list(grab.buf[c])[-8:]])
            cv2.imwrite(os.path.join(out_dir, "%s.png" % c), f0)
            if snap:
                # each click moved onto the table's corner it is on (sub-pixel), when one is found close by
                new, moved = [], []
                for q in cams[c]["corners_px"]:
                    p_, m_ = refine_corner(f0, q)
                    new.append([round(p_[0], 2), round(p_[1], 2)])
                    moved.append(round(m_, 1))
                obs.setdefault("clicked", {})[c] = cams[c]["corners_px"]
                cams[c]["corners_px"] = new
                say("  %s corners: each click moved onto the corner it is on by %s px" % (c, ", ".join("%.1f" % m_ for m_ in moved)))
            masks[c] = table_mask(f0.shape, cams[c]["corners_px"], grow_px=12)
        poses = {c: rect_pose(cams[c]["corners_px"], cams[c]["K"], cams[c]["dist"], cams[c]["size"]) for c in cams}

        def cmd(n, c, words_, tries=2):
            for _ in range(tries):
                t = time.time()
                bench.send(n, c)
                if bench.wait(lambda: any(tt >= t - 0.01 and nn == n and words_ in v for tt, nn, v in list(bench.msgs)[-30:]), 0.8):
                    return time.time()
            return None

        def pics(t0):
            time.sleep(max(0.0, t0 + SETTLE_S + COLLECT_S - time.time()))
            return {c: grab.between(c, t0 + SETTLE_S, t0 + SETTLE_S + COLLECT_S) for c in cams}

        def flash(n, when):
            """n's left light alone, then its right alone, against the dark pictures: spots ids n/when/L, n/when/R."""
            for o in names:
                cmd(o, "led 0 0 0 %d" % DARK_MS, "led 0 0 0")
            td = time.time()
            dark = pics(td)
            new = F.new_firmware(firmware.get(n)) or dry
            got = {}
            sides = (("L", "led 1 1 1 1500 L"), ("R", "led 1 1 1 1500 R")) if new else (("B", "led 1 1 1 1500"),)
            for s, c_ in sides:
                t1 = cmd(n, c_, "led 1 1 1")
                if t1 is None:
                    say("  %s did not answer `led`" % n)
                    return None
                lit = pics(t1)
                cmd(n, "led 0 0 0 %d" % DARK_MS, "led 0 0 0")
                for c in cams:
                    if len(dark[c]) < 2 or len(lit[c]) < 2:
                        continue
                    sp = light_spots(dark[c], lit[c], masks[c], max_n=3)
                    got.setdefault(c, {})[s] = sp
            ids = []
            for c, d in got.items():
                if new:
                    if d.get("L") and d.get("R"):
                        for s in ("L", "R"):
                            obs["spots"].setdefault("%s/%s/%s" % (n, when, s), {})[c] = [d[s][0][1], d[s][0][2]]
                elif d.get("B") and len(d["B"]) >= 2:
                    a, b = d["B"][0], d["B"][1]
                    obs["spots"].setdefault("%s/%s/A" % (n, when), {})[c] = [a[1], a[2]]
                    obs["spots"].setdefault("%s/%s/B" % (n, when), {})[c] = [b[1], b[2]]
            a, b = ("L", "R") if new else ("A", "B")
            ia, ib = "%s/%s/%s" % (n, when, a), "%s/%s/%s" % (n, when, b)
            if ia in obs["spots"] and ib in obs["spots"]:
                if not new:
                    _match_unordered(obs, ia, ib, poses, L)
                obs["pairs"].append((ia, ib))
                ids = [ia, ib]
            return ids

        say("Lights: each robot's left light, then its right, both cameras (nothing moves) ...")
        home = {}
        for n in names:
            ids = flash(n, "start")
            if ids:
                home[n] = ids
                say("  %s: both lights seen by %s" % (n, " and ".join(sorted(set(obs["spots"][ids[0]]) & set(obs["spots"][ids[1]])))))
            else:
                say("  %s: its lights were not seen in both pictures" % n)
        if not home:
            raise RuntimeError("no robot's lights were seen: are the robots on the table and switched on?")
        # a first scale from the lights alone: where each robot stands and faces, for the rolls' room
        prov = solve(dict(obs, rolls=[], bodies=[]), lights=L)
        if roll and prov.get("ok"):
            Lc, Wc = prov["size_cm"]
            rect = (0.0, 0.0, Lc, Wc)
            pos = {}
            for n, (ia, ib) in home.items():
                pa, pb = np.array(prov["points"][ia]), np.array(prov["points"][ib])
                mid = (pa + pb) / 2
                # heading: from the left light to the right, turned 90 deg clockwise (facing.py); unordered: unknown
                hd = math.degrees(math.atan2(pb[1] - pa[1], pb[0] - pa[0])) + 90.0 if ia.endswith("/L") else None
                pos[n] = (mid, hd)
            say("Rolls: each robot %.0f cm on its wheels and back, the cameras watching (first scale from the lights: "
                "%.0f x %.0f cm) ..." % (ROLL_CM, Lc, Wc))
            for n, (mid, hd) in pos.items():
                if hd is None:
                    say("  %s: front and back not told apart (old firmware): no roll" % n)
                    continue
                order = roll_order(mid, hd, rect, {o: p for o, (p, _) in pos.items() if o != n})
                if order is None:
                    say("  %s: no room for a %.0f cm roll either way (%s): skipped" % (n, ROLL_CM, edge_words(mid, rect)))
                    continue
                o_still = {c: _median(grab.between(c, time.time() - 0.6, time.time())) for c in cams}
                r1 = _leg(bench, n, "go %g" % (order * ROLL_CM), say)
                if r1 is None:
                    continue
                far = flash(n, "far")
                f_still = {c: _median(grab.between(c, time.time() - 0.6, time.time())) for c in cams}
                r2 = _leg(bench, n, "go %g" % (-order * ROLL_CM), say)
                back = flash(n, "back") if r2 is not None else []
                # each leg's own pose starts at 0, 0 (the firmware resets it when the leg starts): its end is the
                # distance its wheels measured (with the stored wheel_scale)
                d1 = math.hypot(r1[1], r1[2]) if r1[1] is not None else None
                d2 = math.hypot(r2[1], r2[2]) if r2 is not None and r2[1] is not None else None
                if d1 is None:
                    say("  %s: its roll's end was not heard: not used" % n)
                    continue
                say("  %s: rolled %s %.1f cm and back %s (its wheels)" % (
                    n, "forward" if order > 0 else "backward", d1, "%.1f cm" % d2 if d2 is not None else "(not measured)"))
                for a, b in zip(home[n], far or []):
                    obs["rolls"].append((a, b, round(d1, 2)))
                if d2 is not None:
                    for a, b in zip(far or [], back or []):
                        obs["rolls"].append((a, b, round(d2, 2)))
                # the body: where it stood, from the picture before the roll and the one at the far end
                if "overhead" in cams and far:
                    sa = [obs["spots"][i].get("overhead") for i in home[n]]
                    sb = [obs["spots"][i].get("overhead") for i in far]
                    if all(sa) and all(sb):
                        at, away = np.mean(sa, 0), np.mean(sb, 0)
                        ppc = np.linalg.norm(np.subtract(sa[0], sa[1])) / L["spacing_cm"]
                        box = body_outline(o_still["overhead"], f_still["overhead"], at, away,
                                           masks["overhead"], max_r_px=9.0 * ppc)
                        if box is not None:
                            obs["bodies"].append(dict(robot=n, camera="overhead", corners_px=box, z_cm=L["z_cm"]))
                            cv2.imwrite(os.path.join(out_dir, "body_%s_before.png" % n), o_still["overhead"])
        elif roll:
            say("  no rolls: %s" % " ".join(prov.get("words") or []))
    finally:
        grab.close()
        for n in names:
            try:
                bench.send(n, "led 0 0 0 60")
            except Exception:     # noqa: BLE001
                pass
    res = solve(obs, lights=L)
    res["line"] = result_line(res)
    res["seconds"] = round(time.time() - t_start, 1)
    with open(os.path.join(out_dir, "observations.json"), "w") as f:
        json.dump(obs, f, indent=1, default=float)
    if res.get("ok"):
        for c, v in res["cameras"].items():
            with open(os.path.join(out_dir, "%s_corners_setup.json" % c), "w") as f:
                json.dump(v["model"], f, indent=1)
    with open(os.path.join(out_dir, "result.json"), "w") as f:
        json.dump({k: v for k, v in res.items() if k != "obs"}, f, indent=1, default=float)
    say(res["line"])
    return res


def _match_unordered(obs, ia, ib, poses, L):
    """Old firmware lights both lights at once: name them A and B the same way in both cameras (the pairing whose
    floor spots, each camera on its own first scale, lie nearer each other)."""
    cs = [c for c in obs["spots"][ia] if c in obs["spots"][ib]]
    if len(cs) < 2:
        return
    c0, c1 = cs[0], cs[1]
    fl = {}
    for c in (c0, c1):
        p = poses[c]
        s = np.mean(scale_from(p, [(obs["spots"][ia][c], obs["spots"][ib][c])], L["spacing_cm"], L["z_cm"]))
        fl[c] = [p.floor(np.array([obs["spots"][i][c]]), L["z_cm"] / s)[0] * s for i in (ia, ib)]
    same = np.linalg.norm(fl[c0][0] - fl[c1][0]) + np.linalg.norm(fl[c0][1] - fl[c1][1])
    swap = np.linalg.norm(fl[c0][0] - fl[c1][1]) + np.linalg.norm(fl[c0][1] - fl[c1][0])
    if swap < same:
        obs["spots"][ia][c1], obs["spots"][ib][c1] = obs["spots"][ib][c1], obs["spots"][ia][c1]


def roll_order(mid, hd, rect, others, d=ROLL_CM):
    """+1 (forward first), -1 (backward first) or None: the body keeps EDGE_ROOM_CM from every edge and 21 cm from the
    other robots' middles at the far end. Forward preferred (the front sensor watches it)."""
    import calib_room as CR
    for sgn in (1, -1):
        h = math.radians(hd)
        end = (mid[0] + sgn * d * math.cos(h), mid[1] + sgn * d * math.sin(h))
        ok = CR.edge_dist(rect, *end) >= CR.BODY_HALF_CM + EDGE_ROOM_CM
        ok = ok and all(math.dist(end, p) >= CR.ROOM_ROBOT_CM for p in others.values())
        if ok:
            return sgn
    return None


def edge_words(p, rect):
    d = min(p[0] - rect[0], rect[2] - p[0], p[1] - rect[1], rect[3] - p[1])
    return "%.0f cm from the nearest edge" % d


def _leg(bench, n, text, say, timeout=20.0):
    """One idle `go` leg. A robot with no compass check since power-on refuses it: it gets the compass check alone (a
    few degrees' wiggle on the spot, no spin, no roll; ladder.Ladder.compass_only) and the leg is sent again.
    Returns (result, x, y) or None."""
    r = bench.leg(n, text, timeout)
    if r[0] == "refused" and "compass" in (r[4] or ""):
        import ladder as LD
        say("  %s: no compass check since it was switched on: the compass check alone first (a small wiggle)" % n)
        for c in LD.Ladder.COMPASS_ONLY:
            bench.send(n, c)
            time.sleep(0.05)
        bench.runend.pop(n, None)
        bench.send(n, "calibrate")
        bench.wait(lambda: n in bench.runend and bench.runend[n][0] == "calib", 60)
        for c in LD.Ladder.COMPASS_BACK:
            bench.send(n, c)
            time.sleep(0.05)
        r = bench.leg(n, text, timeout)
    if r[0] != "done":
        say("  %s: its `%s` ended %s (%s): no roll" % (n, text, r[0], r[4]))
        return None
    return (r[0], r[1], r[2])


# --------------------------------------------------------------------------------------- dry run: the simulator
def sim_board(table, out, seed=3, robots=None, side=True, ports=(8775, 8776), extra=()):
    """A simulated board for a dry setup: fleet_sim with both cameras serving /live, robots on Monday's crosses (17 cm in;
    Center half way). Returns (process, starts, true overhead model path, true side model path, robot ips)."""
    import subprocess
    import workspace as WS
    L_, W_ = table
    starts = robots or {"Left": [17.0, 17.0, 90.0], "Center": [L_ / 2.0, 17.0, 90.0], "Right": [L_ - 17.0, 17.0, 90.0]}
    m = WS.blank_map((0, 0, L_, W_), [(n, s) for n, s in starts.items()])
    os.makedirs(out, exist_ok=True)
    mp, sp = os.path.join(out, "board_map.json"), os.path.join(out, "starts.json")
    with open(mp, "w") as f:
        json.dump(m, f, indent=1)
    with open(sp, "w") as f:
        json.dump(starts, f, indent=1)
    oc, scal = os.path.join(out, "table_camera.json"), os.path.join(out, "side_camera.json")
    for p in (oc, scal):
        if os.path.exists(p):
            os.remove(p)
    cmd = [sys.executable, os.path.join(TOOLS, "sim", "fleet_sim.py"), "--map", mp, "--starts", sp, "--out", out,
           "--seed", str(seed), "--fix-hz", "0", "--serve-live", str(ports[0]), "--live-calib-out", oc, "--persist",
           "--max-s", "900", "--quiet", "--start-noise", "0"] + list(extra)
    if side:
        cmd += ["--live-side", str(ports[1]), "--live-side-calib-out", scal]
    p = subprocess.Popen(cmd, stdout=open(os.path.join(out, "sim_stdout.txt"), "a"), stderr=subprocess.STDOUT,
                         start_new_session=True)
    t0 = time.time()
    while not os.path.exists(oc) and time.time() - t0 < 40:
        time.sleep(0.3)
    time.sleep(1.0)
    return p, starts, oc, scal if side else None, ["127.0.0.%d" % (2 + k) for k in range(len(starts))]


def sim_clicks(model_path, table, sd_px=1.5, seed=0):
    """Where a person would click the table's 4 corners (1, 2, 3, 4) in a simulated camera: the true corners plus a
    click's error (CLICK_SD_PX-like, sd_px)."""
    import e02_camera as CAM
    m = CAM.load_camera(model_path)
    L_, W_ = table
    u, v = m.project(np.array([[0, 0, 0], [L_, 0, 0], [L_, W_, 0], [0, W_, 0]], float))
    rng = np.random.default_rng(seed)
    return (np.column_stack([u, v]) + rng.normal(0, sd_px, (4, 2))).tolist()


def truth_report(res, table, true_paths, starts, lights=SIM_LIGHTS):
    """A dry run's result against the simulator's truth: size error, each camera's pose error (lens position cm, tilt
    deg) and floor error over the table (cm, a 5 cm grid), and the robots' positions through the new models."""
    import e02_camera as CAM
    out = dict(true_size=list(table), size=list(res["size_cm"]),
               size_err=[round(res["size_cm"][0] - table[0], 2), round(res["size_cm"][1] - table[1], 2)], cameras={})
    xs, ys = np.meshgrid(np.arange(0, table[0] + 0.1, 5.0), np.arange(0, table[1] + 0.1, 5.0))
    g = np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)])
    from cameras import CameraModel
    for c, path in true_paths.items():
        if not path or c not in res["cameras"]:
            continue
        t = CAM.load_camera(path)
        m = CameraModel.from_dict(res["cameras"][c]["model"])
        u, v = t.project(g)
        ok = t.in_image(u, v)
        x, y = m.to_floor(u[ok], v[ok], 0.0)
        e = np.hypot(x - g[ok, 0], y - g[ok, 1])
        tp, mp = t.pose_summary(), m.pose_summary()
        out["cameras"][c] = dict(lens_err_cm=round(float(np.linalg.norm(np.subtract(tp["camera_position_cm"], mp["camera_position_cm"]))), 2),
                                 height_true=tp["camera_position_cm"][2], height=mp["camera_position_cm"][2],
                                 tilt_err_deg=round(mp["tilt_from_vertical_deg"] - tp["tilt_from_vertical_deg"], 2),
                                 floor_err_cm_median=round(float(np.median(e)), 2), floor_err_cm_max=round(float(e.max()), 2))
    # the robots: their lights' middle at the start through each new model against the truth (the lights' middle is
    # 2.75 cm ahead of the robot's middle in the simulator: camera.py fwd = length / 2 - 2.0)
    rob = {}
    for n, s in starts.items():
        h = math.radians(s[2])
        tx, ty = s[0] + 2.75 * math.cos(h), s[1] + 2.75 * math.sin(h)
        ia, ib = "%s/start/L" % n, "%s/start/R" % n
        per = {}
        for c in res["cameras"]:
            m = CameraModel.from_dict(res["cameras"][c]["model"])
            sp = res.get("_spots") or {}
            if ia in sp and ib in sp and c in sp[ia] and c in sp[ib]:
                a = m.to_floor(*sp[ia][c], z=lights["z_cm"])
                b = m.to_floor(*sp[ib][c], z=lights["z_cm"])
                per[c] = round(math.hypot((a[0] + b[0]) / 2 - tx, (a[1] + b[1]) / 2 - ty), 2)
        rob[n] = per
    out["robots_err_cm"] = rob
    return out


def dry_run(table, out, seed=3, say=print, click_sd=1.5, sim_extra=(), ports=(8775, 8776)):
    """The whole setup on the simulator (both cameras, three robots on Monday's crosses), against its truth."""
    import ladder as LD
    p, starts, oc, scal, ips = sim_board(table, os.path.join(out, "sim"), seed=seed, extra=sim_extra, ports=ports)
    bench = None
    try:
        bench = LD.Bench(ips, os.path.join(out, "link"))
        bench.wait(lambda: len(bench.robots()) >= len(ips), 10)
        names = sorted(bench.robots(), key=lambda n: ["Left", "Center", "Right"].index(n) if n in ("Left", "Center", "Right") else 9)
        import e02_camera as CAM
        lens = {"overhead": CAM.load_camera(oc)}
        corners = {"overhead": sim_clicks(oc, table, click_sd, seed)}
        streams = {"overhead": "127.0.0.1:%d" % ports[0]}
        if scal:
            lens["side"] = CAM.load_camera(scal)
            corners["side"] = sim_clicks(scal, table, click_sd, seed + 1)
            streams["side"] = "127.0.0.1:%d" % ports[1]
        res = run_live(streams, lens, corners, bench, names, out, say=say, lights=SIM_LIGHTS, dry=True)
        if res.get("ok"):
            obs = json.load(open(os.path.join(out, "observations.json")))
            res["_spots"] = obs["spots"]
            res["truth"] = truth_report(res, table, {"overhead": oc, "side": scal}, starts)
            res.pop("_spots", None)
            with open(os.path.join(out, "result.json"), "w") as f:
                json.dump(res, f, indent=1, default=float)
        return res
    finally:
        if bench is not None:
            bench.close()
        import signal
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except OSError:
            pass
        p.wait(timeout=10)


# ----------------------------------------------------------------------------------------------- self test
def _selftest():
    """Two drawn cameras over a 76 x 137 table; lights and rolls projected exactly: the size must come back."""
    from cameras import CameraModel
    rng = np.random.default_rng(1)
    Lc, Wc = 137.0, 76.0

    def cam(C, look, f, size=(640, 480)):
        z = np.asarray(look, float) - C
        z /= np.linalg.norm(z)
        x = np.cross(z, [0, 0, 1.0])
        if np.linalg.norm(x) < 1e-6:
            x = np.array([1.0, 0, 0])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        R = np.vstack([x, y, z])
        t = -R @ np.asarray(C, float)
        K = np.array([[f, 0, size[0] / 2], [0, f, size[1] / 2], [0, 0, 1.0]])
        return CameraModel(K, [0] * 5, _rvec(R), t, size)
    over = cam([70, 40, 250], [68, 38, 0], 600)
    side = cam([40, -55, 170], [68, 34, 0], 520)
    true = {"overhead": over, "side": side}
    corners = [[0, 0, 0], [Lc, 0, 0], [Lc, Wc, 0], [0, Wc, 0]]
    obs = {"cameras": {}, "spots": {}, "pairs": [], "rolls": []}
    for c, m in true.items():
        u, v = m.project(np.array(corners, float))
        obs["cameras"][c] = dict(K=m.K.tolist(), dist=[0] * 5, size=list(m.image_size),
                                 corners_px=(np.column_stack([u, v]) + rng.normal(0, 1.0, (4, 2))).tolist())
    for n, (x, y, h) in {"Left": (17, 17, 90), "Right": (120, 17, 90), "Center": (68, 40, 30)}.items():
        for when, dx in (("start", 0.0), ("far", ROLL_CM)):
            th = math.radians(h)
            cx, cy = x + dx * math.cos(th), y + dx * math.sin(th)
            for side_, s in (("L", 1), ("R", -1)):
                p = (cx - s * math.sin(th) * LIGHT_SPACING_CM / 2, cy + s * math.cos(th) * LIGHT_SPACING_CM / 2, LIGHT_Z_CM)
                i = "%s/%s/%s" % (n, when, side_)
                obs["spots"][i] = {c: (np.array(m.project(p)) + rng.normal(0, 0.3, 2)).tolist() for c, m in true.items()}
            obs["pairs"].append(("%s/%s/L" % (n, when), "%s/%s/R" % (n, when)))
        for s in ("L", "R"):
            obs["rolls"].append(("%s/start/%s" % (n, s), "%s/far/%s" % (n, s), ROLL_CM))
    res = solve(obs)
    print(result_line(res))
    for c, v in res["cameras"].items():
        print("  %s: height %.1f cm (true %.1f), tilt %.1f deg (true %.1f)" % (
            c, v["height_cm"], true[c].center[2], v["tilt_deg"], true[c].pose_summary()["tilt_from_vertical_deg"]))
    ok = abs(res["size_cm"][0] - Lc) < 1.5 and abs(res["size_cm"][1] - Wc) < 1.0
    print("selftest", "OK" if ok else "FAILED")
    return 0 if ok else 1


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--dry", metavar="LxW", help="a dry setup on the simulator's table of this size, e.g. 137x76")
    ap.add_argument("--out", default=None)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--click-sd", type=float, default=1.5, help="dry: the simulated clicks' error, px")
    a = ap.parse_args(argv)
    if a.selftest:
        return _selftest()
    if a.dry:
        L_, W_ = (float(v) for v in a.dry.lower().split("x"))
        out = a.out or os.path.join(os.path.dirname(TOOLS), "logs", time.strftime("%Y%m%d-%H%M%S") + "-corner-setup-dry")
        res = dry_run((L_, W_), out, seed=a.seed, click_sd=a.click_sd)
        print(json.dumps(res.get("truth"), indent=1))
        return 0 if res.get("ok") else 1
    print(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main())
