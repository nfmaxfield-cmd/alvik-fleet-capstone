#!/usr/bin/env python3
"""Floor calibration without printed markers: makes floor_calibration.json
(floor cm -> image px) for a camera recording.

    python tools/calibrate_floor.py <recording folder>                  # click mode
    python tools/calibrate_floor.py <recording folder> --height-cm 115 --nadir 60,10 \\
        --sheet docs/experiments/E00-measurement-check/data/measurements.csv   # + robot mode
    python tools/calibrate_floor.py <folder> --camera 0                 # live grab (camera on this computer)
    python tools/calibrate_floor.py <recording folder> --fit-only       # refit the saved points, no window

A page opens in your browser (served only on this computer). Two kinds of
point pairs, usable together:

  Floor points  Click a spot whose floor position you know (the origin, the
                10 cm marks on the taped axes, a pencil dot measured from both
                axes) and type its x,y in cm. Enter accepts the suggested next
                mark (the last step repeated).
  Robot points  With --sheet (the E00 tape sheet: front and rear marks per
                placement), pick a placement in the list, find a frame where
                the robot sits there, click the centre of its top. The robot's
                position is the midpoint of the front and rear marks. Its top is
                --robot-top-cm (4) above the floor, which is allowed for with the
                camera height. With --detections, a click near a detection snaps
                to the detector's median position around that frame.

No clicking (needs the console log folder --session, the runs' tape sheet
--sheet and camera detections --detections; see floorcal_runs.py):

  --from-runs   each taped run's start and end pose while the robot sits still,
                paired with its median detection. E01 runs all start at the
                origin, so these points alone lie on a line: combine with clicks.
  --survey      the robots' own wheel distance + gyro heading paths while they
                drive, hung from each run's tape start pose (the only tape
                needed), paired with the camera track: for recalibrating after
                the camera was moved. Also refines each start pose within tape
                accuracy and the video clock offset.

    python tools/calibrate_floor.py <recording> --survey --session logs/<x> \
        --sheet <sheet> --detections <recording>/detections.csv --height-cm 140

Survey also uses each run's tape END marks (robot sitting still after the run)
as absolute points when the sheet has them (--no-end-marks: start marks only).

Circularity: a calibration made from runs must not be used to score the camera
against those same runs. Whenever points or pairs come from two or more runs
(or E00 placement spots: both passes at a spot are one group, left out
together), floor_calibration_loro/<run>.json (made WITHOUT that run;
--folds k for k-fold) and floor_calibration_loro.csv (which file scores which
run, with that run's held-out error) are written too; tools/fusion/runs.py
--calib-per-run uses them.

Every change refits and shows, for every point, where the fit puts its typed
cm position (circle) against where you clicked (cross), with the error in cm.
Points are saved after every change to floor_points.json (next start resumes).
"Save calibration" writes floor_calibration.json and floor_calibration_check.jpg
(the frame with a 10 cm grid from the fit drawn on it: the grid should sit on
the tape) beside the recording.

The fit: homography with RANSAC (from 6 points), refined by least squares on
the inliers. Each point's error is reported in cm, both against the fit and
left-one-out (the fit made without that point, which shows a bad click that
pulled the fit towards itself). Optional single radial lens term k1 (--k1 auto
keeps it only if left-one-out errors drop clearly).

Camera height (tape, lens to floor) and the nadir (the floor point straight
below the lens, plumb line) are optional; without the nadir it is estimated
from the fit, which is fine for the robot-top correction (a 3 cm nadir error
moves a robot-top point by 0.1 cm) and is labelled as an estimate in the file.

Needs numpy and opencv-python. Stop with Ctrl+C.
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import re
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("OpenCV is missing. Install it with: pip install opencv-python")

DEFAULT_AREA = [0.0, 0.0, 122.0, 122.0]      # the plywood floor, about 122 x 122 cm (until measured)
FLEET_MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fleet", "map.json")


def default_area(map_path=FLEET_MAP):
    """The floor the robots use, cm [x0, y0, x1, y1]: tools/fleet/map.json's floor
    bounds if that file exists, else DEFAULT_AREA."""
    try:
        with open(map_path) as f:
            fl = json.load(f)["floor"]
        a = [float(fl["x_min"]), float(fl["y_min"]), float(fl["x_max"]), float(fl["y_max"])]
        if a[2] > a[0] and a[3] > a[1]:
            return a
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return list(DEFAULT_AREA)
ROBOT_TOP_CM = 4.0
RANSAC_PX = 6.0           # a click this far (px) from the fit counts as an outlier (about 1.5 cm)
MIN_RANSAC_POINTS = 6     # fewer points than this: no RANSAC (a bad click can't be told apart)
K1_GRID = np.round(np.arange(-0.40, 0.2001, 0.02), 4)
K1_KEEP_RATIO = 0.85      # --k1 auto keeps k1 only if left-one-out RMS drops below this share of the k1 = 0 one
K1_MIN_POINTS = 8
TRUTHY = {"1", "yes", "y", "x", "true"}


class FitError(Exception):
    pass


# ------------------------------------------------------------------ geometry
def apply_h(H, xy):
    """Apply a 3x3 homography to an (n, 2) array."""
    xy = np.asarray(xy, float).reshape(-1, 2)
    p = np.hstack([xy, np.ones((len(xy), 1))]) @ np.asarray(H, float).T
    return p[:, :2] / p[:, 2:3]


def distort(uv, k1, f, c):
    """Ideal (homography) pixels -> pixels with radial distortion k1 (same model as tools/sim/camera.py)."""
    uv = np.asarray(uv, float).reshape(-1, 2)
    if not k1:
        return uv.copy()
    n = (uv - c) / f
    fac = 1 + k1 * (n ** 2).sum(1, keepdims=True)
    return c + f * n * fac


def undistort(uv, k1, f, c, iters=20):
    """Inverse of distort(), by fixed-point iteration (fine for |k1| < 0.5 inside the image)."""
    uv = np.asarray(uv, float).reshape(-1, 2)
    if not k1:
        return uv.copy()
    d = (uv - c) / f
    n = d.copy()
    for _ in range(iters):
        n = d / (1 + k1 * (n ** 2).sum(1, keepdims=True))
    return c + f * n


def raise_to_floor_equiv(xy, z, height_cm, nadir):
    """Floor point that a camera at height_cm above `nadir` sees in the same place
    as the point (x, y) lifted z cm (the robot's top): pushed away from the nadir."""
    xy = np.asarray(xy, float).reshape(-1, 2)
    z = np.broadcast_to(np.asarray(z, float), (len(xy),))
    if not height_cm or nadir is None:
        return xy.copy()
    n = np.asarray(nadir, float)
    return n + (xy - n) * (height_cm / (height_cm - z))[:, None]


def lower_from_floor_equiv(xy, z, height_cm, nadir):
    """Inverse of raise_to_floor_equiv (same as tools/fusion/runs.py floor_from_top)."""
    xy = np.asarray(xy, float).reshape(-1, 2)
    z = np.broadcast_to(np.asarray(z, float), (len(xy),))
    if not height_cm or nadir is None:
        return xy.copy()
    n = np.asarray(nadir, float)
    return n + (xy - n) * ((height_cm - z) / height_cm)[:, None]


def scale_at(H, uv):
    """Pixels per cm of the homography around image point uv (floor plane)."""
    Hinv = np.linalg.inv(H)
    p = apply_h(Hinv, uv)[0]
    q = apply_h(H, [p, p + [1, 0], p + [0, 1]])
    return 0.5 * (np.linalg.norm(q[1] - q[0]) + np.linalg.norm(q[2] - q[0]))


def estimate_nadir(H, size, height_cm=None):
    """Floor point below the lens, assuming the principal point is the image centre.
    With a camera height: the vertical vanishing point from the homography (allows
    for a slight tilt; focal length = px per cm at the centre x height). Without:
    the floor point at the image centre (exact only for a camera looking straight down)."""
    c = np.array(size, float) / 2.0
    Hinv = np.linalg.inv(H)
    centre = apply_h(Hinv, c)[0]
    if not height_cm:
        return centre, "floor point at the image centre"
    f = scale_at(H, c) * height_cm
    K = np.array([[f, 0, c[0]], [0, f, c[1]], [0, 0, 1.0]])
    A = np.linalg.inv(K) @ np.asarray(H, float)
    d = np.cross(A[:, 0], A[:, 1])
    if d[2] < 0:
        d = -d
    p = K @ d
    nad = apply_h(Hinv, p[:2] / p[2])[0]
    if not np.all(np.isfinite(nad)) or np.linalg.norm(nad - centre) > 0.5 * height_cm:   # > ~27 deg tilt: distrust
        return centre, "floor point at the image centre"
    return nad, "vertical vanishing point of the fit"


# ------------------------------------------------------------------ fitting
def _check_spread(xy):
    xy = np.asarray(xy, float)
    s = np.linalg.svd(xy - xy.mean(0), compute_uv=False)
    if len(xy) < 4 or s[-1] < 1e-6 * max(s[0], 1e-9) or s[-1] / math.sqrt(len(xy)) < 1.0:
        raise FitError("the points lie (nearly) on one line: add points off that line (the other axis, or a measured dot)")


def fit_h(floor_xy, px, ransac_px=None):
    """floor cm -> px homography. RANSAC (if ransac_px and >= MIN_RANSAC_POINTS),
    then least squares on the inliers. Returns (H, inlier mask)."""
    src = np.asarray(floor_xy, np.float64).reshape(-1, 2)
    dst = np.asarray(px, np.float64).reshape(-1, 2)
    n = len(src)
    if n < 4:
        raise FitError(f"need at least 4 points, have {n}")
    mask = np.ones(n, bool)
    if ransac_px and n >= MIN_RANSAC_POINTS:
        cv2.setRNGSeed(12345)
        H, m = cv2.findHomography(src, dst, cv2.RANSAC, ransac_px, maxIters=4000, confidence=0.999)
        if H is not None and m is not None and int(m.sum()) >= 4:
            mask = m.ravel().astype(bool)
    _check_spread(src[mask])
    H, _ = cv2.findHomography(src[mask], dst[mask], 0)
    if H is None or not np.all(np.isfinite(H)) or abs(np.linalg.det(H)) < 1e-12:
        raise FitError("no homography fits these points (are some typed coordinates swapped?)")
    return H / H[2, 2], mask


def _floor_err(H, eq, und):
    """cm distance between the typed (floor-equivalent) points and where the fit maps the clicks."""
    return np.linalg.norm(apply_h(np.linalg.inv(H), und) - eq, axis=1)


def _fit_k1(eq, px, f, c, grid=K1_GRID, golden=True):
    """Best k1 (floor-cm RMS) by grid then golden-section search. Returns (k1, H)."""
    def cost(k):
        try:
            H, _ = fit_h(eq, undistort(px, k, f, c))
        except FitError:
            return float("inf"), None
        return float(np.sqrt(np.mean(_floor_err(H, eq, undistort(px, k, f, c)) ** 2))), H
    costs = [cost(k)[0] for k in grid]
    i = int(np.argmin(costs))
    if not golden:
        return float(grid[i]), cost(grid[i])[1]
    a, b = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
    g = (math.sqrt(5) - 1) / 2
    for _ in range(25):
        m1, m2 = b - g * (b - a), a + g * (b - a)
        if cost(m1)[0] < cost(m2)[0]:
            b = m2
        else:
            a = m1
    k = (a + b) / 2
    return k, cost(k)[1]


def _loo(eq, px, k1, f, c, refit_k1=False, grid=None):
    """Left-one-out floor error (cm) per point."""
    n = len(eq)
    out = np.full(n, np.nan)
    if n < 5:
        return out
    for i in range(n):
        keep = np.arange(n) != i
        try:
            if refit_k1:
                k, H = _fit_k1(eq[keep], px[keep], f, c, grid if grid is not None else K1_GRID, golden=False)
            else:
                k = k1
                H, _ = fit_h(eq[keep], undistort(px[keep], k, f, c))
        except FitError:
            continue
        if H is None:
            continue
        out[i] = _floor_err(H, eq[i:i + 1], undistort(px[i:i + 1], k, f, c))[0]
    return out


def _rms(a):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a ** 2))) if len(a) else float("nan")


def coverage(points_xy, area, step=2.0):
    """Share of the area rectangle inside the convex hull of the points, and the
    farthest area point from that hull (cm)."""
    x0, y0, x1, y1 = area
    xs, ys = np.meshgrid(np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step))
    grid = np.stack([xs.ravel(), ys.ravel()], 1)
    pts = np.asarray(points_xy, np.float32).reshape(-1, 2)
    if len(pts) < 3:
        return 0.0, float("nan")
    hull = cv2.convexHull(pts)
    d = np.array([cv2.pointPolygonTest(hull, (float(x), float(y)), True) for x, y in grid])
    return float(np.mean(d >= -1e-6)), float(max(0.0, -d.min()))


def fit(points, size, height_cm=None, nadir=None, k1_mode="auto", area=None, ransac_px=RANSAC_PX):
    """Fit a calibration to the enabled points. points: dicts with u, v (px),
    x, y (cm) and z (height above floor, cm; 0 = floor mark). Returns a dict
    (see to_calibration for the file). Raises FitError if it can't."""
    pts = [p for p in points if p.get("enabled", True)]
    if len(pts) < 4:
        raise FitError(f"need at least 4 points, have {len(pts)}")
    W, Hh = size
    c = np.array([W / 2.0, Hh / 2.0])
    px = np.array([[p["u"], p["v"]] for p in pts], float)
    xy = np.array([[p["x"], p["y"]] for p in pts], float)
    z = np.array([float(p.get("z") or 0.0) for p in pts])
    lifted = bool(np.any(z > 0))
    warnings = []
    if lifted and not height_cm:
        warnings.append("robot-top points used without a camera height: they are treated as floor points, "
                        "which puts them about 3.5% too far from the image centre (up to ~2 cm). "
                        "Measure the camera height and enter it.")
    nadir_given = nadir is not None
    nad = np.asarray(nadir, float) if nadir_given else None
    nadir_src = "plumb line (given)" if nadir_given else None
    if not nadir_given and lifted and height_cm:
        floor_only = z <= 0
        try:   # start from the floor-only fit's estimate if there are enough floor points
            H0, _ = fit_h(xy[floor_only], px[floor_only], ransac_px) if floor_only.sum() >= 4 else fit_h(xy, px, ransac_px)
            nad, nadir_src = estimate_nadir(H0, size, height_cm)
        except FitError:
            nad, nadir_src = None, None
    k1_info = {}
    iters = 3 if (lifted and height_cm and not nadir_given) else 1
    for _ in range(iters):
        eq = raise_to_floor_equiv(xy, z, height_cm, nad) if (height_cm and nad is not None) else xy.copy()
        H0, mask = fit_h(eq, px, ransac_px)
        f = scale_at(H0, c) * height_cm if height_cm else float(W)
        k1 = 0.0
        if k1_mode not in ("auto", "off", "fit", None, ""):
            k1 = float(k1_mode)
        elif k1_mode in ("auto", "fit") and mask.sum() >= (K1_MIN_POINTS if k1_mode == "auto" else 5):
            kf, _ = _fit_k1(eq[mask], px[mask], f, c)
            # inliers again with that k1 (a strongly distorted edge point can look like an outlier at k1 = 0)
            _, mask2 = fit_h(eq, undistort(px, kf, f, c), ransac_px)
            if mask2.sum() >= mask.sum():
                mask = mask2
                kf, _ = _fit_k1(eq[mask], px[mask], f, c)
            if k1_mode == "fit":
                k1 = kf
            else:
                loo0 = _rms(_loo(eq[mask], px[mask], 0.0, f, c))
                loo1 = _rms(_loo(eq[mask], px[mask], kf, f, c, refit_k1=True, grid=K1_GRID[::2]))
                keep = loo1 < K1_KEEP_RATIO * loo0
                k1 = kf if keep else 0.0
                k1_info = dict(k1_tried=round(kf, 4), loo_rms_cm_without_k1=round(loo0, 3),
                               loo_rms_cm_with_k1=round(loo1, 3), k1_kept=bool(keep))
        und = undistort(px, k1, f, c)
        H, _ = fit_h(eq[mask], und[mask])
        if height_cm and not nadir_given:
            nad, nadir_src = estimate_nadir(H, size, height_cm)
    if not nadir_given and nad is None:
        nad, nadir_src = estimate_nadir(H, size, height_cm)
    # per-point errors, in floor cm at the point's own height
    eq = raise_to_floor_equiv(xy, z, height_cm, nad) if height_cm else xy
    und = undistort(px, k1, f, c)
    seen = lower_from_floor_equiv(apply_h(np.linalg.inv(H), und), z, height_cm, nad) if height_cm else \
        apply_h(np.linalg.inv(H), und)
    err = np.linalg.norm(seen - xy, axis=1)
    pred_px = distort(apply_h(H, eq), k1, f, c)
    err_px = np.linalg.norm(pred_px - px, axis=1)
    loo = np.full(len(pts), np.nan)
    loo[mask] = _loo(eq[mask], px[mask], k1, f, c)
    if not np.all(mask):   # outliers: their error against the fit is already "left out"
        loo[~mask] = err[~mask]
    med_res = float(np.median(err[mask]))
    med_loo = float(np.nanmedian(loo)) if np.any(np.isfinite(loo)) else float("nan")
    rows = []
    for i, p in enumerate(pts):
        why = []
        if not mask[i]:
            why.append("RANSAC outlier")
        if np.isfinite(loo[i]) and np.isfinite(med_loo) and loo[i] > max(1.5, 3 * med_loo):
            why.append("left-one-out error large")
        if mask[i] and err[i] > max(1.0, 3 * med_res):
            why.append("error large")
        rows.append(dict(id=p.get("id", i + 1), kind=p.get("kind", "floor"), label=p.get("label", ""),
                         x=float(xy[i, 0]), y=float(xy[i, 1]), z=float(z[i]), u=float(px[i, 0]), v=float(px[i, 1]),
                         err_cm=round(float(err[i]), 3), err_px=round(float(err_px[i]), 2),
                         loo_cm=None if not np.isfinite(loo[i]) else round(float(loo[i]), 3),
                         dx_cm=round(float(seen[i, 0] - xy[i, 0]), 3), dy_cm=round(float(seen[i, 1] - xy[i, 1]), 3),
                         pred_u=round(float(pred_px[i, 0]), 2), pred_v=round(float(pred_px[i, 1]), 2),
                         inlier=bool(mask[i]), flagged=bool(why), why=", ".join(why)))
    # edge vs centre (are the edges clearly worse? -> lens distortion)
    r = np.linalg.norm(px - c, axis=1) / np.linalg.norm(c)
    edge, mid = mask & (r > 0.6), mask & (r <= 0.6)
    if k1 == 0 and edge.sum() >= 3 and mid.sum() >= 3 and _rms(err[edge]) > 2 * _rms(err[mid]) and _rms(err[edge]) > 0.5:
        warnings.append(f"points near the image edges fit worse ({_rms(err[edge]):.2f} cm RMS) than near the centre "
                        f"({_rms(err[mid]):.2f} cm): lens distortion? Try --k1 fit (or more edge points with --k1 auto).")
    area = list(area or default_area())
    cov, far = coverage(xy[mask], area)
    if cov < 0.6:
        warnings.append(f"the points cover only {cov * 100:.0f}% of the robots' area x {area[0]:g}..{area[2]:g}, "
                        f"y {area[1]:g}..{area[3]:g} cm (up to {far:.0f} cm outside them): the fit is extrapolated "
                        "there. Add points towards the area's far corners.")
    corners = np.array([[area[0], area[1]], [area[2], area[1]], [area[2], area[3]], [area[0], area[3]]])
    cpx = distort(apply_h(H, corners), k1, f, c)
    outside = [f"({x:g}, {y:g})" for (x, y), (u, v) in zip(corners, cpx) if not (0 <= u < W and 0 <= v < Hh)]
    if outside:
        warnings.append("area corners outside the camera's view: " + ", ".join(outside))
    if len(pts) < MIN_RANSAC_POINTS:
        warnings.append(f"only {len(pts)} points: a bad click can't be spotted. Use 8 or more.")
    if not mask.all():
        warnings.append(f"{int((~mask).sum())} point(s) left out of the fit as outliers (RANSAC): check them")
    gap = None
    if k1:   # what a reader that ignores k1 gets wrong
        xs, ys = np.meshgrid(np.linspace(area[0], area[2], 15), np.linspace(area[1], area[3], 11))
        g = np.stack([xs.ravel(), ys.ravel()], 1)
        gpx = distort(apply_h(H, g), k1, f, c)
        gap = float(np.max(np.linalg.norm(apply_h(np.linalg.inv(H), gpx) - g, axis=1)))
    if nadir_src and not nadir_given:
        nadir_src = "estimated: " + nadir_src
    return dict(H=H, k1=float(k1), f_px=float(f), cx=float(c[0]), cy=float(c[1]), size=[int(W), int(Hh)],
                height_cm=height_cm, nadir=None if nad is None else [float(nad[0]), float(nad[1])],
                nadir_source=nadir_src, points=rows, n_points=len(pts), n_inliers=int(mask.sum()),
                rms_cm=_rms(err[mask]), max_cm=float(err[mask].max()),
                loo_rms_cm=_rms(loo[mask]), coverage=cov, outside_cm=far, area=area, warnings=warnings,
                k1_info=k1_info, k1_ignored_max_cm=gap, mode="points",
                used_runs=sorted({str(p["group"]) for p in pts if p.get("group")}))


def view_polygons(res, n=16):
    """The camera's view mapped to the floor: polygons (floor cm) of the floor area
    seen, and (with a camera height) of where the robot's top centre is in view.
    Returns (floor polygon, robot polygon or None)."""
    W, Hh = res["size"]
    s = np.linspace(0, 1, n, endpoint=False)
    e = [(-0.5 + (W) * t, -0.5) for t in s] + [(W - 0.5, -0.5 + Hh * t) for t in s] + \
        [(W - 0.5 - W * t, Hh - 0.5) for t in s] + [(-0.5, Hh - 0.5 - Hh * t) for t in s]
    und = undistort(np.array(e), res["k1"], res["f_px"], np.array([res["cx"], res["cy"]]))
    G = np.linalg.inv(np.asarray(res["H"], float))
    p = np.hstack([und, np.ones((len(und), 1))]) @ G.T
    ok = p[:, 2] > 0 if np.median(p[:, 2]) > 0 else p[:, 2] < 0
    if not ok.all():     # part of the image border is at or above the horizon: not a floor point
        res.setdefault("warnings", []).append("part of the image shows no floor (horizon): view polygon cut")
    fl = p[ok, :2] / p[ok, 2:3]
    rob = None
    if res.get("height_cm") and res.get("nadir") is not None:
        rob = lower_from_floor_equiv(fl, ROBOT_TOP_CM, res["height_cm"], res["nadir"])
    r2 = lambda a: [[round(float(x), 1), round(float(y), 1)] for x, y in a]
    return r2(fl), (r2(rob) if rob is not None else None)


def to_calibration(res, source=None, extra_note=""):
    """The floor_calibration.json content (docs/data-formats.md "Floor calibration")."""
    cam = {}
    if res.get("height_cm"):
        cam["height_cm"] = round(float(res["height_cm"]), 2)
        if res.get("height_source"):
            cam["height_source"] = res["height_source"]
    if res.get("nadir") is not None:
        cam["nadir_floor_cm"] = [round(v, 2) for v in res["nadir"]]
        cam["nadir_source"] = res.get("nadir_source")
    out = dict(
        note="floor cm -> image px. [u, v, w] = H_floor_to_px @ [x_cm, y_cm, 1]; pixel = (u/w, v/w), origin "
             "top-left, pixel centres on whole numbers. If k1 != 0, the lens bends that pixel: xn = (u - cx)/f_px, "
             "yn = (v - cy)/f_px, factor = 1 + k1 (xn^2 + yn^2), pixel = (cx + f_px xn factor, cy + f_px yn factor) "
             "(same model as the simulator). Written by tools/calibrate_floor.py. " + extra_note,
        H_floor_to_px=[[float(v) for v in row] for row in np.asarray(res["H"])],
        image_size=res["size"],
        k1=round(float(res["k1"]), 5), cx=res["cx"], cy=res["cy"], f_px=round(float(res["f_px"]), 2),
        camera=cam,
        fit=dict(mode=res.get("mode", "points"), used_runs=res.get("used_runs") or [],
                 survey=res.get("survey"),n_points=res["n_points"], n_inliers=res["n_inliers"], rms_cm=round(res["rms_cm"], 3),
                 max_cm=round(res["max_cm"], 3), loo_rms_cm=round(res["loo_rms_cm"], 3),
                 coverage_of_area=round(res["coverage"], 3), area_cm=res["area"], warnings=res["warnings"],
                 k1=res.get("k1_info") or None,
                 k1_ignored_max_cm=None if res.get("k1_ignored_max_cm") is None else round(res["k1_ignored_max_cm"], 2),
                 points=[{k: p[k] for k in ("id", "kind", "label", "x", "y", "z", "u", "v", "err_cm", "loo_cm",
                                            "inlier", "why")} for p in res["points"]]),
        created=dt.datetime.now().isoformat(timespec="seconds"),
        source=source)
    if res.get("plane_height_cm"):
        out["plane_height_cm"] = res["plane_height_cm"]
        out["note"] += (f" NO CAMERA HEIGHT: this homography maps the plane {res['plane_height_cm']:g} cm above the "
                        "floor (the robot's top), not the floor itself.")
    floor_poly, robot_poly = view_polygons(res)
    out["view_floor_polygon_cm"] = floor_poly
    if robot_poly:
        out["view_robot_polygon_cm"] = robot_poly
    if res.get("used_runs"):
        out["note"] += (" The runs in fit.used_runs helped make this calibration: to score the camera against those "
                        "runs' tape marks (or odometry), use the calibration made without them, listed in "
                        "floor_calibration_loro.csv.")
    return out


def project(calib, x, y, z=0.0):
    """Floor point (cm, z cm above the floor) -> pixel, using a calibration dict."""
    cam = calib.get("camera") or {}
    eq = raise_to_floor_equiv([[x, y]], z, cam.get("height_cm"), cam.get("nadir_floor_cm"))
    uv = apply_h(calib["H_floor_to_px"], eq)
    return tuple(distort(uv, calib.get("k1") or 0.0, calib.get("f_px") or 1.0,
                         np.array([calib.get("cx") or 0.0, calib.get("cy") or 0.0]))[0])


def to_floor(calib, u, v, z=0.0):
    """Pixel -> floor point (cm) for a point z cm above the floor."""
    cam = calib.get("camera") or {}
    und = undistort([[u, v]], calib.get("k1") or 0.0, calib.get("f_px") or 1.0,
                    np.array([calib.get("cx") or 0.0, calib.get("cy") or 0.0]))
    eq = apply_h(np.linalg.inv(np.asarray(calib["H_floor_to_px"], float)), und)
    return tuple(lower_from_floor_equiv(eq, z, cam.get("height_cm"), cam.get("nadir_floor_cm"))[0])


def grid_lines(res, step=10.0):
    """Polylines (px) of a floor grid over the area, from a fit result, for drawing."""
    x0, y0, x1, y1 = res["area"]
    c = np.array([res["cx"], res["cy"]])
    lines = []
    for x in np.arange(math.ceil(x0 / step) * step, x1 + 1e-9, step):
        ys = np.linspace(y0, y1, 25)
        lines.append(dict(axis=bool(abs(x) < 1e-9), pts=distort(apply_h(res["H"], np.stack([np.full_like(ys, x), ys], 1)),
                                                        res["k1"], res["f_px"], c).round(1).tolist()))
    for y in np.arange(math.ceil(y0 / step) * step, y1 + 1e-9, step):
        xs = np.linspace(x0, x1, 25)
        lines.append(dict(axis=bool(abs(y) < 1e-9), pts=distort(apply_h(res["H"], np.stack([xs, np.full_like(xs, y)], 1)),
                                                        res["k1"], res["f_px"], c).round(1).tolist()))
    return lines


def draw_check(img, res):
    """The frame with the fitted grid, the axes, the clicks and the fit's positions for them."""
    out = img.copy()
    for ln in grid_lines(res):
        pts = np.round(np.array(ln["pts"]) * 16).astype(np.int32)
        cv2.polylines(out, [pts], False, (0, 0, 255) if ln["axis"] else (0, 200, 255), 2 if ln["axis"] else 1,
                      cv2.LINE_AA, 4)
    for p in res["points"]:
        col = (0, 0, 255) if p["flagged"] else (0, 200, 0)
        u, v = p["u"], p["v"]
        cv2.drawMarker(out, (int(round(u)), int(round(v))), col, cv2.MARKER_CROSS, 9, 1, cv2.LINE_AA)
        cv2.circle(out, (int(round(p["pred_u"] * 16)), int(round(p["pred_v"] * 16))), 4 * 16, (255, 120, 0), 1,
                   cv2.LINE_AA, 4)
        cv2.putText(out, f"{p['id']}: {p['err_cm']:.1f}", (int(u) + 5, int(v) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    col, 1, cv2.LINE_AA)
    txt = f"RMS {res['rms_cm']:.2f} cm  max {res['max_cm']:.2f} cm  n={res['n_inliers']}/{res['n_points']}  k1={res['k1']:.3f}"
    cv2.putText(out, txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(out, txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return out


# ------------------------------------------------------------------ leave-one-run-out
def write_loro(out_path, groups, fit_fn, heldout_fn, folds=0, source=None, log=print):
    """One calibration per run (or per fold of runs) made WITHOUT that run, so the
    camera can be scored against a run's tape marks without circularity.
    Writes <out dir>/floor_calibration_loro/<run or fold>.json and
    <out dir>/floor_calibration_loro.csv (run_id -> the file to score it with).
    fit_fn(excluded runs) -> fit result; heldout_fn(result, run) -> dict of numbers."""
    groups = sorted(set(groups))
    if len(groups) < 2:
        return None
    base = os.path.dirname(os.path.abspath(out_path))
    d = os.path.join(base, "floor_calibration_loro")
    os.makedirs(d, exist_ok=True)
    if folds and folds < len(groups):
        fold_sets = [groups[k::folds] for k in range(folds)]
        names = [f"fold{k + 1}" for k in range(folds)]
    else:
        fold_sets = [[g] for g in groups]
        names = groups
    rows = []
    for name, held in zip(names, fold_sets):
        fname = os.path.join(d, re.sub(r"[^A-Za-z0-9_.-]", "_", name) + ".json")
        try:
            res = fit_fn(set(held))
        except FitError as e:
            for g in held:
                rows.append(dict(run_id=g, calibration="", left_out=";".join(held), n_used="", fit_rms_cm="",
                                 note=f"no fit without it: {e}"))
            continue
        cal = to_calibration(res, source=source)
        cal["held_out_runs"] = list(held)
        cal["use_for"] = ("score the camera against the tape marks / odometry of the held_out_runs only; "
                          "these runs did not help make this calibration")
        with open(fname, "w") as f:
            json.dump(cal, f, indent=2, default=_jsonable)
        for g in held:
            row = dict(run_id=g, calibration=os.path.relpath(fname, base), left_out=";".join(held),
                       n_used=res["n_inliers"], fit_rms_cm=round(res["rms_cm"], 3), note="")
            row.update(heldout_fn(res, g) or {})
            rows.append(row)
    cols = []
    for r in rows:
        cols += [k for k in r if k not in cols]
    man = os.path.join(base, "floor_calibration_loro.csv")
    with open(man, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    log(f"Wrote {len(fold_sets)} held-out calibrations to {d}\n  and {man} (which file scores which run)")
    return man, rows


def points_heldout(points):
    """For write_loro: a run's own points' errors (cm) under a fit made without them."""
    def fn(res, g):
        cal = to_calibration(res)
        out = {}
        for p in points:
            if str(p.get("group")) == str(g) and p.get("enabled", True):
                x, y = to_floor(cal, p["u"], p["v"], float(p.get("z") or 0))
                key = "heldout_" + (p.get("spot", "").rsplit("-", 1)[-1] or "point") + "_err_cm"
                out[key] = round(math.hypot(x - p["x"], y - p["y"]), 3)
        return out
    return fn


def summary_text(res):
    loo = res.get("loo_rms_cm")
    lines = [f"{res['n_points']} {'survey pairs' if res.get('survey') else 'points'}, {res['n_inliers']} used. "
             f"Error RMS {res['rms_cm']:.2f} cm, max {res['max_cm']:.2f} cm"
             + (f"; left-one-out RMS {loo:.2f} cm." if loo is not None and np.isfinite(loo) else ".")]
    if res["k1"]:
        lines.append(f"Lens term k1 = {res['k1']:.4f} (f_px {res['f_px']:.0f}). Tools that ignore k1 "
                     f"(tools/blink) are off by up to {res['k1_ignored_max_cm']:.1f} cm in the area.")
    elif res.get("k1_info"):
        i = res["k1_info"]
        lines.append(f"Lens term not kept: k1 = {i['k1_tried']} gave left-one-out {i['loo_rms_cm_with_k1']} cm "
                     f"vs {i['loo_rms_cm_without_k1']} cm without.")
    if res.get("nadir") is not None:
        lines.append(f"Nadir {res['nadir'][0]:.1f}, {res['nadir'][1]:.1f} cm ({res['nadir_source']}); "
                     f"camera height {res['height_cm'] or 'not given'}.")
    if res.get("height_source") and res.get("height_cm"):
        lines.append(f"Camera height {res['height_cm']:.1f} cm ({res['height_source']}).")
    lines.append(f"Points cover {res['coverage'] * 100:.0f}% of the area {res['area']}.")
    sv = res.get("survey")
    if sv:
        if sv.get("end_marks_used"):
            lines.append(f"Survey used {sv['end_marks_used']} tape end marks (absolute points).")
        lines.append(f"Survey: path error weighted RMS {sv['weighted_rms_cm']} cm; early half of the paths "
                     f"{sv['rms_early_cm']} cm, later half {sv['rms_late_cm']} cm. Video clock refined by "
                     + ", ".join(f"{v:+.0f} ms" for v in sv["sessions"].values()) + ".")
        lines.append("  run          pairs used  RMS cm  start moved cm / deg")
        for rid, r in sv["runs"].items():
            lines.append(f"  {rid:12} {r['pairs']:5} {r['used']:4} {r['rms_cm'] if r['rms_cm'] is not None else '-':>7}  "
                         f"{r['start_shift_cm']} / {r['start_turn_deg']}")
    if res["points"]:
        lines.append(" id kind    x cm    y cm   err cm  l-1-out  flag")
    for p in res["points"]:
        loo = "   -   " if p["loo_cm"] is None else f"{p['loo_cm']:7.2f}"
        lines.append(f"{p['id']:3} {p['kind']:5} {p['x']:7.1f} {p['y']:7.1f} {p['err_cm']:8.2f} {loo}  "
                     f"{'<-- ' + p['why'] if p['flagged'] else ''}")
    for w in res["warnings"]:
        lines.append("WARNING: " + w)
    return "\n".join(lines)


# ------------------------------------------------------------------ inputs
def read_sheet_spots(path):
    """Robot positions from a tape sheet (docs/data-formats.md): midpoint of the
    front and rear marks, for the end marks and (if filled) the start marks."""
    spots = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if str(r.get("excluded", "")).strip().lower() in TRUTHY:
                continue
            for which, keys in (("start", ("xf0_cm", "yf0_cm", "xr0_cm", "yr0_cm")),
                                ("end", ("xf_cm", "yf_cm", "xr_cm", "yr_cm"))):
                try:
                    xf, yf, xr, yr = (float(r[k]) for k in keys)
                except (KeyError, TypeError, ValueError):
                    continue
                spots.append(dict(key=f"{r.get('order')}-{which}", order=r.get("order"), which=which,
                                  robot=r.get("robot", ""), level=r.get("level", ""),
                                  x=round((xf + xr) / 2, 3), y=round((yf + yr) / 2, 3),
                                  heading=round(math.degrees(math.atan2(yf - yr, xf - xr)), 1)))
    return spots


def read_events(folder):
    p = os.path.join(folder, "events.csv") if folder else None
    if not p or not os.path.isfile(p):
        return []
    out = []
    with open(p, newline="") as f:
        for r in csv.DictReader(f):
            try:
                out.append(dict(frame=int(float(r["frame"])), text=r.get("text", "")))
            except (KeyError, ValueError):
                pass
    return out


def spot_frames(spots, events):
    """Guess a frame for each spot from run_sheet.py's notes ("trial N ... start" / "trial N run X end")."""
    for s in spots:
        start = end = None
        for e in events:
            m = re.match(r"\s*trial\s+(\d+)\b", e["text"], re.I)
            if m and m.group(1) == str(s["order"]):
                if re.search(r"\bend\b", e["text"], re.I):
                    end = e["frame"]
                elif start is None:
                    start = e["frame"]
        s["frame"] = (end if end is not None else start) if s["which"] == "end" else start
    return spots


def read_detections(path):
    """frame -> list of (x_px, y_px) from a detections CSV or labels.csv (alvik rows)."""
    dets = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r.get("cls") not in (None, "", "alvik"):
                continue
            if r.get("status") in ("rejected", "pending"):
                continue
            try:
                fr, x, y = int(float(r["frame"])), float(r["x_px"]), float(r["y_px"])
            except (KeyError, ValueError):
                continue
            dets.setdefault(fr, []).append((x, y))
    return dets


def snap_to_detection(dets, frame, u, v, radius=15.0, window=15):
    """Median detector position near (u, v) over frames frame-window..frame+window
    (the robot sits still there). Returns (u, v, n, spread_px) or None."""
    near = [(x, y) for fr in range(frame - window, frame + window + 1) for x, y in dets.get(fr, [])
            if math.hypot(x - u, y - v) <= radius]
    if not near:
        return None
    a = np.array(near)
    m = np.median(a, 0)
    a = a[np.linalg.norm(a - m, axis=1) <= 4.0]
    if not len(a):
        return None
    m = np.median(a, 0)
    return float(m[0]), float(m[1]), int(len(a)), float(np.sqrt(np.mean(np.sum((a - m) ** 2, 1))))


class Source:
    """Frames from a recording folder, a video file, an image, or a camera grab."""

    def __init__(self, path=None, camera=None, folder=None):
        self.lock = threading.Lock()
        self.cap = None
        self.still = None
        self.n_frames, self.fps = 1, 30.0
        if camera is not None:
            self.still = grab_camera(camera)
            os.makedirs(folder, exist_ok=True)
            self.desc = f"camera {camera}, grabbed {dt.datetime.now().isoformat(timespec='seconds')}"
            cv2.imwrite(os.path.join(folder, "calib_frame.jpg"), self.still)
            print(f"Grabbed a frame from camera {camera}: {os.path.join(folder, 'calib_frame.jpg')}")
        elif os.path.isdir(path):
            vid = os.path.join(path, "video.avi")
            if not os.path.isfile(vid):
                stills = [p for p in ("calib_frame.jpg", "calib_frame.png") if os.path.isfile(os.path.join(path, p))]
                if not stills:
                    raise SystemExit(f"{path}: no video.avi (or calib_frame.jpg) in it")
                self._open_image(os.path.join(path, stills[0]))
            else:
                self._open_video(vid)
        elif path.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
            self._open_image(path)
        else:
            self._open_video(path)
        img = self.frame(0)
        self.size = [int(img.shape[1]), int(img.shape[0])]
        self.last = (0, img)

    def _open_image(self, p):
        self.still = cv2.imread(p)
        if self.still is None:
            raise SystemExit(f"Can't read the image {p}")
        self.desc = os.path.abspath(p)

    def _open_video(self, p):
        self.cap = cv2.VideoCapture(p)
        if not self.cap.isOpened():
            raise SystemExit(f"Can't open the video {p}")
        self.n_frames = max(1, int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT) or 1))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.desc = os.path.abspath(p)

    def frame(self, i):
        if self.still is not None:
            return self.still
        i = int(min(max(i, 0), self.n_frames - 1))
        with self.lock:
            if getattr(self, "last", None) and self.last[0] == i:
                return self.last[1]
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, img = self.cap.read()
            if not ok:
                raise ValueError(f"frame {i} could not be read")
            self.last = (i, img)
            return img


def grab_camera(index, warmup_s=1.5):
    backends = [cv2.CAP_DSHOW, cv2.CAP_ANY] if os.name == "nt" else [cv2.CAP_ANY]
    for be in backends:
        cap = cv2.VideoCapture(int(index), be)
        if cap.isOpened():
            break
    else:
        raise SystemExit(f"Can't open camera {index}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    t0, img = time.time(), None
    while time.time() - t0 < warmup_s or img is None:   # let exposure settle
        ok, im = cap.read()
        if ok:
            img = im
        elif time.time() - t0 > 5:
            break
    cap.release()
    if img is None:
        raise SystemExit(f"Camera {index} opened but sent no picture")
    return img


# ------------------------------------------------------------------ points file
def load_points(path):
    if path and os.path.isfile(path):
        with open(path) as f:
            d = json.load(f)
        return d.get("points", []), d.get("settings", {})
    return [], {}


def save_points(path, points, settings, source, size):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(dict(version=1, source=source, image_size=size, saved=dt.datetime.now().isoformat(timespec="seconds"),
                       settings=settings, points=points), f, indent=1, default=_jsonable)
    os.replace(tmp, path)


def parse_k1(s):
    s = str(s if s is not None else "auto").strip().lower()
    if s in ("auto", "off", "fit"):
        return s
    if s in ("0", "none", ""):
        return "off"
    return float(s)


class App:
    def __init__(self, src, points_path, out_path, settings, spots, events, dets):
        self.src, self.points_path, self.out_path = src, points_path, out_path
        self.settings, self.spots, self.events, self.dets = settings, spots, events, dets
        self.points, saved = load_points(points_path)
        for k, v in saved.items():           # saved settings, unless given on the command line
            if self.settings.get(k) is None:
                self.settings[k] = v
        self.lock = threading.Lock()
        self.folds = 0

    def fit(self):
        s = self.settings
        try:
            res = fit(self.points, self.src.size, s.get("height_cm"), s.get("nadir_floor_cm"),
                      parse_k1(s.get("k1", "auto")), s.get("area") or default_area())
        except FitError as e:
            return None, str(e)
        return res, None

    def fit_json(self):
        res, err = self.fit()
        if res is None:
            return dict(ok=False, error=err)
        keep = {k: v for k, v in res.items() if k != "H"}
        keep["H"] = np.asarray(res["H"]).tolist()
        keep["grid"] = grid_lines(res)
        keep["summary"] = summary_text(res)
        return dict(ok=True, fit=keep)

    def save_calibration(self):
        res, err = self.fit()
        if res is None:
            return dict(ok=False, error=err)
        cal = to_calibration(res, source=dict(video=self.src.desc, points_file=os.path.abspath(self.points_path)))
        with open(self.out_path, "w") as f:
            json.dump(cal, f, indent=2, default=_jsonable)
        check = os.path.join(os.path.dirname(os.path.abspath(self.out_path)), "floor_calibration_check.jpg")
        fr = int(self.points[-1].get("frame", 0) or 0) if self.points else 0
        cv2.imwrite(check, draw_check(self.src.frame(fr), res))
        groups = [str(p["group"]) for p in self.points if p.get("group") and p.get("enabled", True)]
        msg = []
        if len(set(groups)) >= 2:
            s = self.settings

            def fit_without(excl):
                return fit([p for p in self.points if str(p.get("group")) not in excl], self.src.size,
                           s.get("height_cm"), s.get("nadir_floor_cm"), parse_k1(s.get("k1", "auto")),
                           s.get("area") or default_area())
            write_loro(self.out_path, groups, fit_without, points_heldout(self.points), self.folds,
                       source=cal["source"], log=msg.append)
        return dict(ok=True, path=os.path.abspath(self.out_path), check=check,
                    summary=summary_text(res) + ("\n" + "\n".join(msg) if msg else ""))


def _jsonable(o):
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"{type(o).__name__} is not JSON serializable")


def make_handler(app):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json"):
            if isinstance(body, (dict, list)):
                body = json.dumps(body, default=_jsonable).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            try:
                if u.path in ("/", "/index.html"):
                    return self._send(200, PAGE, "text/html; charset=utf-8")
                if u.path == "/frame":
                    ok, jpg = cv2.imencode(".jpg", app.src.frame(int(q.get("i", 0))), [cv2.IMWRITE_JPEG_QUALITY, 92])
                    return self._send(200, jpg.tobytes(), "image/jpeg")
                if u.path == "/api/state":
                    with app.lock:
                        return self._send(200, dict(size=app.src.size, n_frames=app.src.n_frames, fps=app.src.fps,
                                                    source=app.src.desc, events=app.events, spots=app.spots,
                                                    points=app.points, settings=app.settings,
                                                    has_dets=bool(app.dets), out=os.path.abspath(app.out_path),
                                                    **app.fit_json()))
                if u.path == "/api/dets":
                    fr = int(q.get("frame", 0))
                    return self._send(200, dict(dets=(app.dets or {}).get(fr, [])))
                if u.path == "/api/snap":
                    s = snap_to_detection(app.dets or {}, int(q["frame"]), float(q["u"]), float(q["v"]))
                    return self._send(200, dict(snap=s))
                return self._send(404, dict(error="not found"))
            except Exception as e:     # keep the page alive; show the problem
                return self._send(500, dict(ok=False, error=f"{type(e).__name__}: {e}"))

        def do_POST(self):
            u = urlparse(self.path)
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
                with app.lock:
                    if u.path == "/api/points":
                        app.points = body.get("points", app.points)
                        app.settings.update(body.get("settings") or {})
                        save_points(app.points_path, app.points, app.settings, app.src.desc, app.src.size)
                        return self._send(200, app.fit_json())
                    if u.path == "/api/save":
                        r = app.save_calibration()
                        if r.get("ok"):
                            print("\n" + r["summary"] + f"\nSaved {r['path']}\n      {r['check']}")
                        return self._send(200, r)
                return self._send(404, dict(error="not found"))
            except Exception as e:
                return self._send(500, dict(ok=False, error=f"{type(e).__name__}: {e}"))

    return H


def parse_pair(s):
    if s is None:
        return None
    a = [float(v) for v in re.split(r"[,\s]+", s.strip()) if v]
    if len(a) != 2:
        raise argparse.ArgumentTypeError("give two numbers, e.g. 60,10")
    return a


def run_inputs(args, folder):
    """Runs (laptop clock), video offset and detections for --from-runs / --survey."""
    import floorcal_runs as fr
    if not args.session or not args.detections:
        raise SystemExit("--from-runs / --survey need --session (console log folder) and --detections")
    runs, fits = fr.rn.load_runs(args.session, args.sheet)
    off = args.video_offset
    if off is None:
        print("Finding the light flashes to line up the video clock (a few minutes for a long video; "
              "--video-offset skips this)...")
        vo = fr.rn.video_offset(args.session, folder, fits)
        if vo is None:
            raise SystemExit("No light flashes matched between video and robot logs; give --video-offset")
        off = vo[0]
        print(f"video clock = laptop clock {off * 1000:+.0f} ms (from {vo[2]} flashes)")
    dets = fr.read_detections(args.detections)
    kinds = {d["kind"] for d in dets}
    if kinds == {"led"}:
        print("NOTE: only blink (LED) detections given: the LED midpoint is ahead of the robot's centre"
              + (f"; shifted by --led-offset-cm {args.led_offset_cm}" if args.led_offset_cm else
                 "; NOT corrected (give --led-offset-cm fwd,left), so points are biased along the heading"))
    return fr, runs, off, dets


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", nargs="?", default=".",
                    help="recording folder (with video.avi), a video file or an image; with --camera, the folder to save in")
    ap.add_argument("--camera", type=int, help="grab a frame from this camera on this computer instead")
    ap.add_argument("--sheet", help="tape measurement sheet: E00's for robot points; the runs' sheet for --from-runs/--survey")
    ap.add_argument("--detections", help="detections CSV, labels.csv or blink_detections.csv")
    ap.add_argument("--height-cm", type=float, help="camera lens height above the floor, cm (tape)")
    ap.add_argument("--nadir", type=parse_pair, help="floor x,y (cm) straight below the lens (plumb line)")
    ap.add_argument("--robot-top-cm", type=float, help=f"height of the robot's top, cm (default {ROBOT_TOP_CM})")
    ap.add_argument("--k1", help="lens term: auto (default; kept only if it clearly helps), off, fit, or a number")
    ap.add_argument("--area", help="x0,y0,x1,y1 cm: the floor the robots use (default: tools/fleet/map.json's floor "
                                   "bounds if it exists, else 0,0,122,122)")
    ap.add_argument("--points", help="points file (default floor_points.json beside the output); share one "
                                     "across sessions to accumulate (only while the camera stays put)")
    ap.add_argument("--out", help="output (default floor_calibration.json in the recording folder)")
    ap.add_argument("--port", type=int, default=8771)
    ap.add_argument("--no-browser", action="store_true", help="don't open the browser")
    ap.add_argument("--fit-only", action="store_true", help="fit the saved points and write the calibration, no page")
    g = ap.add_argument_group("from the runs themselves (no clicking)")
    g.add_argument("--session", help="console log folder of the runs (logs/<date-time>)")
    g.add_argument("--from-runs", action="store_true",
                   help="add each taped run's start and end pose (robot sitting still) as points")
    g.add_argument("--survey", action="store_true",
                   help="fit from the robots' wheel+gyro paths while driving (tape start pose = the anchor)")
    g.add_argument("--video-offset", type=float, help="video clock minus laptop clock, s (skips the flash search)")
    g.add_argument("--folds", type=int, default=0, help="held-out calibrations: 0 = leave one run out, k = k folds by run")
    g.add_argument("--led-offset-cm", type=parse_pair, help="blink detections: LED midpoint ahead,left of the centre, cm")
    g.add_argument("--axle-to-centre-cm", type=float, default=0.0,
                   help="survey: robot centre this far ahead of the wheel axle (odometry tracks the axle)")
    g.add_argument("--intrinsics", help="survey without a camera height: an earlier floor_calibration.json of this "
                                        "webcam; its f_px (and k1) give the height")
    g.add_argument("--no-end-marks", action="store_true",
                   help="survey: don't use the runs' tape end marks (by default they are used, held out per run)")
    g.add_argument("--max-path-sigma-cm", type=float, default=3.0,
                   help="survey: stop using a path once its uncertainty passes this")
    args = ap.parse_args(argv)

    if args.camera is not None or os.path.isdir(args.source):
        folder = args.source
    else:
        folder = os.path.dirname(os.path.abspath(args.source))
    out = args.out or os.path.join(folder, "floor_calibration.json")
    pts_path = args.points or os.path.join(os.path.dirname(os.path.abspath(out)), "floor_points.json")
    settings = dict(height_cm=args.height_cm, nadir_floor_cm=args.nadir, robot_top_cm=args.robot_top_cm,
                    k1=args.k1, area=[float(v) for v in args.area.split(",")] if args.area else None)
    src = Source(None if args.camera is not None else args.source, camera=args.camera, folder=folder)
    events = read_events(folder)
    auto = args.survey or args.from_runs
    spots = spot_frames(read_sheet_spots(args.sheet), events) if (args.sheet and not auto) else []
    dets = read_detections(args.detections) if (args.detections and not auto) else None
    app = App(src, pts_path, out, settings, spots, events, dets)
    app.folds = args.folds
    for k, v in (("robot_top_cm", ROBOT_TOP_CM), ("k1", "auto"), ("area", default_area())):
        if app.settings.get(k) is None:
            app.settings[k] = v
    if spots:
        print(f"{len(spots)} robot placements from {args.sheet}")

    if args.survey:
        return survey_main(args, app, folder, out)
    if args.from_runs:
        fr, runs, off, rdets = run_inputs(args, folder)
        new, notes = fr.still_points(runs, rdets, off, app.settings["robot_top_cm"], args.led_offset_cm)
        for n in notes:
            print("  " + n)
        keys = {p["spot"] for p in new}
        kept = [p for p in app.points if p.get("spot") not in keys]
        nid = max([p.get("id", 0) for p in kept] + [0])
        for p in new:
            nid += 1
            p["id"] = nid
            p["session"] = os.path.basename(os.path.normpath(args.session))
        app.points = kept + new
        save_points(pts_path, app.points, app.settings, src.desc, src.size)
        print(f"{len(new)} run points added ({len(app.points)} points in {pts_path})")
    if args.fit_only:
        r = app.save_calibration()
        if not r["ok"]:
            sys.exit("Can't fit: " + r["error"])
        print(r["summary"] + f"\nSaved {r['path']}\n      {r['check']}")
        return
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(app))
    url = f"http://127.0.0.1:{args.port}/"
    print(f"Calibration page: {url}  (points save to {pts_path}; Ctrl+C stops)")
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped. Points are in", pts_path)


def survey_main(args, app, folder, out):
    fr, runs, off, rdets = run_inputs(args, folder)
    s = app.settings
    if not any(r.tape_start for r in runs if not r.excluded):
        raise SystemExit("--survey needs at least one run with tape start marks on --sheet (the anchor)")
    paths = fr.odometry_paths(runs)
    session = os.path.basename(os.path.normpath(args.session))
    new = fr.survey_pairs(runs, rdets, off, paths, args.axle_to_centre_cm, args.led_offset_cm,
                          args.max_path_sigma_cm, session=session)
    if not args.no_end_marks:
        ends, notes = fr.end_mark_pairs(runs, rdets, off, s["robot_top_cm"], args.led_offset_cm, session=session)
        for n in notes:
            if "end" in n or "WARNING" in n:
                print("  " + n)
        new += ends
        print(f"{len(ends)} tape end marks added (each held out with its own run in the per-run calibrations)")
    pairs_path = os.path.join(os.path.dirname(os.path.abspath(out)), "survey_pairs.csv")
    fr.write_pairs(pairs_path, new)
    pairs = fr.read_pairs(pairs_path)
    print(f"{len(new)} survey pairs from {len({p['run'] for p in new})} runs ({len(pairs)} in {pairs_path})")
    f_px, k1 = None, parse_k1(s.get("k1", "auto"))
    if args.intrinsics:
        with open(args.intrinsics) as f:
            intr = json.load(f)
        f_px = intr.get("f_px")
        if intr.get("k1") and args.k1 is None:
            k1 = float(intr["k1"])
        print(f"webcam focal length {f_px} px, k1 {k1} from {args.intrinsics}")

    def fit_without(excl):
        res = fr.fit_survey(pairs, app.src.size, s.get("height_cm"), s.get("nadir_floor_cm"), f_px, k1,
                            s.get("area"), s["robot_top_cm"], exclude_runs=excl)
        res["mode"] = "survey"
        res["used_runs"] = sorted({p["run"] for p in pairs if p["run"] not in excl})
        return res
    try:
        res = fit_without(set())
    except FitError as e:
        sys.exit("Can't fit: " + str(e))
    cal = to_calibration(res, source=dict(video=app.src.desc, session=os.path.abspath(args.session),
                                          pairs_file=pairs_path, video_offset_s=off))
    with open(out, "w") as f:
        json.dump(cal, f, indent=2, default=_jsonable)
    check = os.path.join(os.path.dirname(os.path.abspath(out)), "floor_calibration_check.jpg")
    cv2.imwrite(check, draw_check(app.src.frame(new[len(new) // 2]["frame"] if new else 0), res))
    print(summary_text(res) + f"\nSaved {os.path.abspath(out)}\n      {check}")

    def heldout(r, g):
        med, rms = fr.survey_error(r, pairs, g)
        end, _ = fr.survey_error(r, pairs, g, kind="tape_end")
        return dict(heldout_path_median_cm=None if med is None else round(med, 3),
                    heldout_path_rms_cm=None if rms is None else round(rms, 3),
                    heldout_end_mark_err_cm=None if end is None else round(end, 3))
    write_loro(out, [p["run"] for p in pairs], fit_without, heldout, args.folds, source=cal["source"])


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Floor calibration</title>
<style>
:root{--bg:#16181b;--panel:#202328;--line:#363a41;--text:#e6e6e6;--dim:#9aa0a6;--ok:#3ccf6e;--bad:#ff5a5a;--acc:#ffb020;--pred:#4aa3ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:13px system-ui,Segoe UI,sans-serif;display:flex;height:100vh}
#left{flex:1;display:flex;flex-direction:column;min-width:0;padding:8px}
#wrap{position:relative;flex:1;overflow:auto}
canvas#cv{cursor:crosshair;display:block}
#loupe{position:fixed;width:180px;height:180px;border:2px solid var(--acc);pointer-events:none;display:none;background:#000;z-index:5}
#right{width:430px;background:var(--panel);border-left:1px solid var(--line);overflow:auto;padding:10px}
h3{margin:12px 0 6px;font-size:13px;color:var(--acc)}
input,select,button{background:#2b2f35;color:var(--text);border:1px solid var(--line);border-radius:4px;padding:3px 6px;font:inherit}
button{cursor:pointer}button:hover{border-color:var(--acc)}
.row{display:flex;gap:6px;align-items:center;flex-wrap:wrap;margin:4px 0}
table{border-collapse:collapse;width:100%;font-size:12px}td,th{padding:2px 4px;border-bottom:1px solid var(--line);text-align:right}
th{color:var(--dim);font-weight:normal}td.l,th.l{text-align:left}
tr.flag td{color:var(--bad)}tr.off td{color:#666;text-decoration:line-through}tr.sel{background:#33302a}
#entry{display:none;background:#3a2f12;border:1px solid var(--acc);padding:6px;border-radius:4px;margin-bottom:6px}
#msg{white-space:pre-wrap;font-family:Consolas,monospace;font-size:11px;color:var(--dim)}
.warn{color:var(--acc);margin:3px 0}.err{color:var(--bad)}
.spot{cursor:pointer;padding:2px 4px;border-radius:3px}.spot.sel{background:#3a2f12}.spot.done{color:var(--ok)}
.tabs button.on{border-color:var(--acc);color:var(--acc)}
.dim{color:var(--dim)}
</style></head><body>
<div id="left">
  <div id="entry"><span id="entryLbl"></span> x, y (cm): <input id="entryIn" size="12" placeholder="e.g. 10, 0">
    <button onclick="acceptEntry()">Add (Enter)</button> <button onclick="cancelEntry()">Cancel (Esc)</button></div>
  <div class="row">
    <button onclick="step(-30)">&laquo; 1 s</button><button onclick="step(-1)">&lsaquo;</button>
    <input id="frameIn" size="6" onchange="goFrame(+this.value)"><button onclick="step(1)">&rsaquo;</button>
    <button onclick="step(30)">1 s &raquo;</button>
    <input id="slider" type="range" min="0" value="0" style="flex:1" oninput="goFrame(+this.value)">
    <select id="events" onchange="if(this.value!=='')goFrame(+this.value)"></select>
    <label><input type="checkbox" id="showGrid" checked onchange="draw()"> grid (G)</label>
    <label>zoom <select id="zoom" onchange="layout()"><option>1</option><option selected>1.5</option><option>2</option><option>3</option></select></label>
  </div>
  <div id="wrap"><canvas id="cv"></canvas></div>
  <div class="dim">Click a point you know. Cross = your click; blue circle = where the fit puts the cm you typed; the line between them is the error.
  Ctrl+Z undo, Delete removes the selected point, G grid on/off.</div>
</div>
<canvas id="loupe" width="180" height="180"></canvas>
<div id="right">
  <div class="row tabs"><b>Mode:</b> <button id="mFloor" onclick="setMode('floor')">Floor points</button>
    <button id="mRobot" onclick="setMode('robot')">Robot points</button></div>
  <div id="robotBox" style="display:none">
    <div class="dim">Pick a placement, find a frame where the robot sits there, click the centre of its top.</div>
    <div id="spots" style="max-height:180px;overflow:auto;border:1px solid var(--line);margin-top:4px"></div>
  </div>
  <h3>Camera</h3>
  <div class="row">height <input id="sH" size="5"> cm &nbsp; nadir x,y <input id="sN" size="9"> cm</div>
  <div class="row">robot top <input id="sZ" size="4"> cm &nbsp; lens k1 <input id="sK" size="5" title="auto, off, fit or a number"></div>
  <div class="row">area x0,y0,x1,y1 <input id="sA" size="16"> <button onclick="applySettings()">Apply</button></div>
  <h3>Fit</h3>
  <div id="fitBox"></div>
  <div class="row"><button onclick="undo()">Undo (Ctrl+Z)</button> <button onclick="save()" style="border-color:var(--ok)">Save calibration</button></div>
  <div id="saved" class="dim"></div>
  <h3>Points</h3>
  <table id="tbl"></table>
  <h3>Details</h3><div id="msg"></div>
</div>
<script>
let S={points:[],settings:{},spots:[],events:[],size:[640,480],n_frames:1},fit=null,hist=[],frame=0,mode='floor',
    sel=null,selSpot=null,pending=null,img=new Image(),dets=[],Z=1.5;
const cv=document.getElementById('cv'),ctx=cv.getContext('2d'),lp=document.getElementById('loupe'),lctx=lp.getContext('2d');
async function j(url,body){const r=await fetch(url,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});return r.json()}
async function init(){S=await j('/api/state');fit=S.ok?S.fit:null;
  document.getElementById('slider').max=S.n_frames-1;
  const ev=document.getElementById('events');ev.innerHTML='<option value="">notes...</option>'+S.events.map(e=>`<option value="${e.frame}">${e.frame}: ${esc(e.text)}</option>`).join('');
  fillSettings();setMode(S.spots.length?'robot':'floor');
  const last=S.points.length?S.points[S.points.length-1].frame||0:0;goFrame(last);showFit(S);}
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
function fillSettings(){const s=S.settings;sH.value=s.height_cm??'';sN.value=s.nadir_floor_cm?s.nadir_floor_cm.join(', '):'';
  sZ.value=s.robot_top_cm??4;sK.value=s.k1??'auto';sA.value=(s.area||[]).join(', ');}
function readSettings(){const num=v=>v.trim()===''?null:+v;const pair=v=>{const a=v.split(/[ ,]+/).filter(x=>x).map(Number);return a.length==2&&a.every(isFinite)?a:null};
  const a=sA.value.split(/[ ,]+/).filter(x=>x).map(Number);
  return {height_cm:num(sH.value),nadir_floor_cm:pair(sN.value),robot_top_cm:num(sZ.value)??4,k1:sK.value.trim()||'auto',area:a.length==4?a:null}}
async function applySettings(){S.settings=readSettings();await push(false)}
function setMode(m){mode=m;mFloor.classList.toggle('on',m=='floor');mRobot.classList.toggle('on',m=='robot');
  robotBox.style.display=m=='robot'?'':'none';if(m=='robot'&&!S.spots.length){robotBox.innerHTML='<div class="err">No sheet given: start the tool with --sheet docs/experiments/E00-measurement-check/data/measurements.csv</div>'}
  drawSpots();}
function drawSpots(){const box=document.getElementById('spots');if(!box)return;
  box.innerHTML=S.spots.map(s=>{const p=S.points.find(p=>p.spot==s.key);
   return `<div class="spot ${selSpot==s.key?'sel':''} ${p?'done':''}" onclick="pickSpot('${s.key}')">#${s.order} ${s.which} ${esc(s.level)} ${esc(s.robot)} (${s.x.toFixed(1)}, ${s.y.toFixed(1)})${s.frame!=null?' frame '+s.frame:''}${p?' &#10003; point '+p.id:''}</div>`}).join('')}
function pickSpot(k){selSpot=k;const s=S.spots.find(s=>s.key==k);if(s&&s.frame!=null)goFrame(s.frame);drawSpots()}
function goFrame(i){frame=Math.max(0,Math.min(S.n_frames-1,i|0));frameIn.value=frame;slider.value=frame;
  img.onload=()=>{layout()};img.src='/frame?i='+frame;if(S.has_dets)j('/api/dets?frame='+frame).then(r=>{dets=r.dets;draw()})}
function step(d){goFrame(frame+d)}
function layout(){Z=+zoom.value;cv.width=S.size[0]*Z;cv.height=S.size[1]*Z;draw()}
function P(u,v){return[(u+0.5)*Z,(v+0.5)*Z]}   // pixel centres on whole numbers
function draw(){ctx.setTransform(1,0,0,1,0,0);ctx.imageSmoothingEnabled=Z<2;ctx.drawImage(img,0,0,cv.width,cv.height);
  if(fit&&showGrid.checked){for(const l of fit.grid){ctx.beginPath();l.pts.forEach((p,i)=>{const[x,y]=P(p[0],p[1]);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});
    ctx.strokeStyle=l.axis?'rgba(255,60,60,.8)':'rgba(255,190,0,.45)';ctx.lineWidth=1;ctx.stroke()}}
  for(const d of dets){const[x,y]=P(d[0],d[1]);ctx.strokeStyle='#c0f';ctx.beginPath();ctx.arc(x,y,7*Z,0,7);ctx.stroke()}
  const rows=fit?Object.fromEntries(fit.points.map(r=>[r.id,r])):{};
  for(const p of S.points){const r=rows[p.id];const[x,y]=P(p.u,p.v);
    const col=p.enabled===false?'#777':(r&&r.flagged?'#ff5a5a':'#3ccf6e');
    if(r&&p.enabled!==false){const[a,b]=P(r.pred_u,r.pred_v);ctx.strokeStyle='#4aa3ff';ctx.lineWidth=1;ctx.beginPath();ctx.arc(a,b,4*Z,0,7);ctx.stroke();
      ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(a,b);ctx.stroke()}
    ctx.strokeStyle=col;ctx.lineWidth=p.id===sel?3:1.5;const k=6;ctx.beginPath();ctx.moveTo(x-k,y);ctx.lineTo(x+k,y);ctx.moveTo(x,y-k);ctx.lineTo(x,y+k);ctx.stroke();
    ctx.fillStyle=col;ctx.font='11px sans-serif';ctx.fillText(p.id+(r?' '+r.err_cm.toFixed(1):''),x+5,y-5)}
  if(pending){const[x,y]=P(pending.u,pending.v);ctx.strokeStyle='#ffb020';ctx.lineWidth=2;ctx.beginPath();ctx.arc(x,y,6,0,7);ctx.stroke()}}
function evPx(e){const r=cv.getBoundingClientRect();return[(e.clientX-r.left)/Z-0.5,(e.clientY-r.top)/Z-0.5]}
cv.addEventListener('mousemove',e=>{const[u,v]=evPx(e);lp.style.display='block';lp.style.left=(e.clientX+24)+'px';lp.style.top=(e.clientY+24)+'px';
  const M=6,w=180/M;lctx.imageSmoothingEnabled=false;lctx.fillStyle='#000';lctx.fillRect(0,0,180,180);
  lctx.drawImage(img,u+0.5-w/2,v+0.5-w/2,w,w,0,0,180,180);lctx.strokeStyle='#ffb020';lctx.beginPath();lctx.moveTo(90,0);lctx.lineTo(90,180);lctx.moveTo(0,90);lctx.lineTo(180,90);lctx.stroke();
  lctx.fillStyle='#ffb020';lctx.fillText(u.toFixed(1)+', '+v.toFixed(1),4,176)});
cv.addEventListener('mouseleave',()=>lp.style.display='none');
cv.addEventListener('click',async e=>{const[u,v]=evPx(e);const uv={u:+u.toFixed(2),v:+v.toFixed(2)};
  if(mode=='floor'){pending=uv;entry.style.display='block';entryLbl.textContent=`Point at pixel (${uv.u.toFixed(1)}, ${uv.v.toFixed(1)}):`;
    entryIn.value=suggest();entryIn.focus();entryIn.select();draw();return}
  const s=S.spots.find(s=>s.key==selSpot);if(!s){alert('Pick a placement in the list first.');return}
  let note='';if(S.has_dets){const r=await j(`/api/snap?frame=${frame}&u=${uv.u}&v=${uv.v}`);if(r.snap&&!e.shiftKey){uv.u=+r.snap[0].toFixed(2);uv.v=+r.snap[1].toFixed(2);note=` (detector median of ${r.snap[2]}, spread ${r.snap[3].toFixed(1)} px)`}}
  const pts=S.points.filter(p=>p.spot!=s.key);   // re-clicking a placement replaces its point
  pts.push({id:nextId(),kind:'robot',u:uv.u,v:uv.v,x:s.x,y:s.y,z:readSettings().robot_top_cm,frame:frame,spot:s.key,group:s.level?'spot '+s.level:'row '+s.order,label:`#${s.order} ${s.which} ${s.level}`+note,enabled:true});
  await change(pts);const i=S.spots.findIndex(t=>t.key==s.key);const nx=S.spots.slice(i+1).find(t=>!S.points.some(p=>p.spot==t.key));if(nx)pickSpot(nx.key)});
function nextId(){return S.points.reduce((m,p)=>Math.max(m,p.id),0)+1}
function suggest(){const f=S.points.filter(p=>p.kind=='floor');if(f.length>=2){const a=f[f.length-2],b=f[f.length-1];return `${+(2*b.x-a.x).toFixed(2)}, ${+(2*b.y-a.y).toFixed(2)}`}return ''}
entryIn.addEventListener('keydown',e=>{if(e.key=='Enter')acceptEntry();if(e.key=='Escape')cancelEntry();e.stopPropagation()});
async function acceptEntry(){const a=entryIn.value.split(/[ ,;]+/).filter(x=>x).map(Number);
  if(a.length!=2||!a.every(isFinite)){entryIn.style.borderColor='red';return}entryIn.style.borderColor='';
  const pts=S.points.concat([{id:nextId(),kind:'floor',u:pending.u,v:pending.v,x:a[0],y:a[1],z:0,frame:frame,label:'',enabled:true}]);
  cancelEntry();await change(pts)}
function cancelEntry(){pending=null;entry.style.display='none';draw()}
async function change(pts){hist.push(JSON.stringify(S.points));S.points=pts;await push(true)}
let seq=0;async function push(){const my=++seq;fitBox.style.opacity=.5;const r=await j('/api/points',{points:S.points,settings:S.settings});if(my!==seq)return;fitBox.style.opacity=1;showFit(r)}
function showFit(r){fit=r.ok?r.fit:null;let h='';
  if(!r.ok)h=`<div class="err">${esc(r.error||'')}</div>`;
  else{const f=r.fit;h=`<div><b>${f.n_inliers}/${f.n_points}</b> points used &middot; RMS <b>${f.rms_cm.toFixed(2)} cm</b> &middot; max <b>${f.max_cm.toFixed(2)} cm</b> &middot; left-one-out RMS ${isFinite(f.loo_rms_cm)?f.loo_rms_cm.toFixed(2):'-'} cm</div>
   <div class="dim">k1 ${f.k1.toFixed(4)} &middot; covers ${(f.coverage*100).toFixed(0)}% of the area${f.nadir?` &middot; nadir ${f.nadir[0].toFixed(1)}, ${f.nadir[1].toFixed(1)} cm (${esc(f.nadir_source)})`:''}</div>`+
   f.warnings.map(w=>`<div class="warn">&#9888; ${esc(w)}</div>`).join('');msg.textContent=f.summary}
  fitBox.innerHTML=h;table();drawSpots();draw()}
function table(){const rows=fit?Object.fromEntries(fit.points.map(r=>[r.id,r])):{};
  tbl.innerHTML='<tr><th>use</th><th>id</th><th class="l">kind</th><th>x</th><th>y</th><th>err cm</th><th>1-out</th><th class="l">flag</th><th></th></tr>'+
  S.points.map(p=>{const r=rows[p.id];return `<tr class="${p.enabled===false?'off':''} ${r&&r.flagged?'flag':''} ${p.id===sel?'sel':''}" onclick="selPt(${p.id})">
   <td><input type="checkbox" ${p.enabled===false?'':'checked'} onclick="event.stopPropagation();togglePt(${p.id})"></td><td>${p.id}</td><td class="l" title="${esc(p.label||'')}">${p.kind}${p.z?' +'+p.z:''}</td>
   <td>${p.x}</td><td>${p.y}</td><td>${r?r.err_cm.toFixed(2):''}</td><td>${r&&r.loo_cm!=null?r.loo_cm.toFixed(2):''}</td><td class="l">${r?esc(r.why):''}</td>
   <td><button onclick="event.stopPropagation();delPt(${p.id})">&times;</button></td></tr>`}).join('')}
function selPt(id){sel=id;const p=S.points.find(p=>p.id==id);if(p&&p.frame!=null&&p.frame!=frame)goFrame(p.frame);else draw();table()}
function delPt(id){change(S.points.filter(p=>p.id!=id))}
function togglePt(id){change(S.points.map(p=>p.id==id?Object.assign({},p,{enabled:p.enabled===false}):p))}
async function undo(){if(!hist.length)return;S.points=JSON.parse(hist.pop());await push()}
async function save(){const r=await j('/api/save',{});saved.innerHTML=r.ok?`Saved ${esc(r.path)}<br>and ${esc(r.check)}`:`<span class="err">${esc(r.error)}</span>`}
document.addEventListener('keydown',e=>{if(e.target.tagName=='INPUT')return;
  if((e.ctrlKey||e.metaKey)&&e.key=='z'){e.preventDefault();undo()}
  else if(e.key=='Delete'&&sel!=null)delPt(sel);else if(e.key=='g'||e.key=='G'){showGrid.checked=!showGrid.checked;draw()}
  else if(e.key=='ArrowRight')step(e.shiftKey?30:1);else if(e.key=='ArrowLeft')step(e.shiftKey?-30:-1)});
init();
</script></body></html>
"""

if __name__ == "__main__":
    main()
