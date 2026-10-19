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
                paired with its median detection. With only one or two start
                spots (and straight runs) the points can lie near a line:
                combine with clicks.
  --survey      the robots' own wheel distance + gyro heading paths while they
                drive, hung from each run's tape start pose (the only tape
                needed), paired with the camera track: for recalibrating after
                the camera was moved. Also refines each start pose within tape
                accuracy and the video clock offset.

    python tools/calibrate_floor.py <recording> --survey --session logs/<x> \
        --sheet <sheet> --detections <recording>/detections.csv --height-cm 140

Any logged run works (E1 routes, demo, straight), but the robots also have a
drive made for this (firmware 2026-10-18b+): put a robot on a pencil-marked
spot near the middle of the view, note its front/rear marks in a sheet row,
start the recording and send `run survey` (a square spiral out to about 1 m x
1 m, then back to the start; docs/home-experiments.md "Survey drive"). Do 3-4
from separately measured spots, then run this with --survey. The taped start
heading is what limits a survey calibration (simulated: 2 surveys mean 0.8 to
2.2 cm over the view depending on the tape errors, 4 surveys 0.6 cm, the 36 E1
runs 0.2 cm), so more separately measured starts help most.

Survey also uses each run's tape END marks (robot sitting still after the run)
as absolute points when the sheet has them (--no-end-marks: start marks only).

Circularity: a calibration made from runs must not be used to score the camera
against those same runs. Whenever points or pairs come from two or more runs
(or E00 placement spots: both passes at a spot are one group, left out
together), floor_calibration_loro/<run>.json (made WITHOUT that run;
--folds k for k-fold) and floor_calibration_loro.csv (which file scores which
run, with that run's held-out error) are written too; tools/fusion/replay.py
and runs.py --calib-per-run use them (and refuse a run its calibration used).

Survey and wheel odometry: the survey hangs the camera fit on the robots' own
wheel + gyro paths, so a robot's wheel-size error goes into the calibration.
Leaving out one run doesn't remove it (the robot's other runs carry the same
error), and the camera then agrees with that robot's wheels for the wrong
reason. Headline T2/T3 results (camera vs wheels) should use a click /
tape-point calibration, or a leave-one-robot-out survey:
--survey --holdout robot (floor_calibration_loro_robot.csv, used the same way).

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

Landmarks (boards and blocks the robots' distance sensor can range, for
tools/fusion --map): the page's Landmarks mode, after a calibration is saved.
Pick a frame where the boards show; for a board click the two ends of its FOOT
(where it meets the floor: its top edge is higher and looks shifted in the
picture, so it must not be used); for a brick or cinderblock click 2-4 corners
of its footprint, or of its top face with its height typed in. Positions show
in cm and inches; drag a click to adjust it, or type a tape-measured position
(inches) for any end. "Save landmarks" writes landmarks.csv beside the
calibration (docs/data-formats.md "Landmarks", source user-picked) and, if
--map was given, puts them into that map.json. Clicks are kept in
landmarks_picked.json. Which face of a board the robot sees is left to the
fusion: all faces are written. No page (tests, scripts):

    python tools/calibrate_floor.py <recording folder> --landmarks-from picks.json [--map map.json]

(picks.json in the landmarks_picked.json format, tools/landmark_pick.py.)

Needs numpy and opencv-python. Stop with Ctrl+C.
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import re
import socket
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
MAP_PATH = None       # the fleet map whose floor bounds are the default area (set by main: --map or found)


def map_candidates():
    """Where a fleet map may be: ./map.json (tools/fleet/app.py's default, saved in the
    folder it was started from) and tools/fleet/map.json."""
    out = []
    for c in (os.path.abspath("map.json"), FLEET_MAP):
        if c not in out:
            out.append(c)
    return out


def read_map_area(map_path):
    """[x0, y0, x1, y1] cm from a fleet map's floor bounds, or None if missing/broken."""
    try:
        with open(map_path, encoding="utf-8-sig") as f:
            fl = json.load(f)["floor"]
        a = [float(fl["x_min"]), float(fl["y_min"]), float(fl["x_max"]), float(fl["y_max"])]
        if a[2] > a[0] and a[3] > a[1]:
            return a
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def find_map(arg=None, log=print):
    """The fleet map to take the default area from: --map if given, else the first of
    map_candidates() that exists. Says which one it used (and if two disagree)."""
    if arg:
        if not os.path.isfile(arg):
            raise SystemExit(f"--map {arg}: no such file")
        log(f"floor area from the fleet map {os.path.abspath(arg)} (--map)")
        return os.path.abspath(arg)
    found = [c for c in map_candidates() if os.path.isfile(c)]
    if not found:
        log("no fleet map (looked for " + " and ".join(map_candidates()) + f"): default area {DEFAULT_AREA}")
        return None
    log(f"floor area from the fleet map {found[0]}" + (" (--map picks another)" if len(found) > 1 else ""))
    if len(found) > 1 and read_map_area(found[0]) != read_map_area(found[1]):
        log(f"  note: {found[1]} has a different floor ({read_map_area(found[1])} vs {read_map_area(found[0])})")
    return found[0]


def default_area(map_path=None):
    """The floor the robots use, cm [x0, y0, x1, y1]: the fleet map's floor bounds
    (map_path, else MAP_PATH, else the first of map_candidates() that exists) if
    there is one, else DEFAULT_AREA."""
    if map_path is None:
        map_path = MAP_PATH or next((c for c in map_candidates() if os.path.isfile(c)), None)
    a = read_map_area(map_path) if map_path else None
    return a if a else list(DEFAULT_AREA)


ROBOT_TOP_CM = 4.0
RANSAC_PX = 6.0           # a click this far (px) from the fit counts as an outlier (about 1.5 cm)
MIN_RANSAC_POINTS = 6     # fewer points (or separate spots) than this: no RANSAC (a bad click can't be told apart)
SPOT_TOL_CM = 3.0         # points closer than this count as one spot (tape repeat of the same mark)
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


def distinct_spots(floor_xy, tol_cm=SPOT_TOL_CM):
    """How many separate floor spots the points are on (points within tol_cm of an earlier spot
    count once: e.g. a run's start and end on the same mark, or repeats of one start spot)."""
    spots = []
    for p in np.asarray(floor_xy, float).reshape(-1, 2):
        if not any(np.hypot(*(p - q)) < tol_cm for q in spots):
            spots.append(p)
    return len(spots)


def fit_h(floor_xy, px, ransac_px=None):
    """floor cm -> px homography. RANSAC (if ransac_px and the points are on >= MIN_RANSAC_POINTS
    separate spots), then least squares on the inliers. Returns (H, inlier mask)."""
    src = np.asarray(floor_xy, np.float64).reshape(-1, 2)
    dst = np.asarray(px, np.float64).reshape(-1, 2)
    n = len(src)
    if n < 4:
        raise FitError(f"need at least 4 points, have {n}")
    mask = np.ones(n, bool)
    # RANSAC needs separate spots, not just many points: with points on only 4-5 spots (repeated
    # start marks) any 4-point sample fits exactly and a good spot gets thrown out as an "outlier"
    if ransac_px and n >= MIN_RANSAC_POINTS and distinct_spots(src) >= MIN_RANSAC_POINTS:
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
    n_spots = distinct_spots(xy)
    if len(pts) < MIN_RANSAC_POINTS:
        warnings.append(f"only {len(pts)} points: a bad click can't be spotted. Use 8 or more.")
    elif n_spots < MIN_RANSAC_POINTS:
        warnings.append(f"the {len(pts)} points are on only {n_spots} separate spots: a bad point can't be spotted "
                        "and the fit between and beyond them is poorly pinned. Add points on other spots "
                        "(axis clicks or measured dots).")
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
                 **({"used_robots": res["used_robots"]} if res.get("used_robots") is not None else {}),
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
def write_loro(out_path, groups, fit_fn, heldout_fn, folds=0, source=None, log=print, members=None, holdout="run"):
    """One calibration per run (or per fold of runs) made WITHOUT that run, so the
    camera can be scored against a run's tape marks without circularity.
    Writes <out dir>/floor_calibration_loro/<run or fold>.json and
    <out dir>/floor_calibration_loro.csv (run_id -> the file to score it with).
    fit_fn(excluded runs) -> fit result; heldout_fn(result, run) -> dict of numbers.

    members: {group: [run ids]} to leave out whole groups of runs at a time
    (holdout="robot": a group is a robot and all its runs; files go to
    floor_calibration_loro_robot/ and floor_calibration_loro_robot.csv, one row
    per run, so they are used exactly like the per-run ones)."""
    groups = sorted(set(groups))
    if len(groups) < 2:
        return None
    base = os.path.dirname(os.path.abspath(out_path))
    stem = "floor_calibration_loro" + ("" if holdout == "run" else "_" + holdout)
    d = os.path.join(base, stem)
    os.makedirs(d, exist_ok=True)
    if folds and folds < len(groups):
        fold_sets = [groups[k::folds] for k in range(folds)]
        names = [f"fold{k + 1}" for k in range(folds)]
    else:
        fold_sets = [[g] for g in groups]
        names = groups
    runs_of = (lambda g: sorted(members.get(g, []))) if members else (lambda g: [g])
    rows = []
    for name, held in zip(names, fold_sets):
        fname = os.path.join(d, re.sub(r"[^A-Za-z0-9_.-]", "_", name) + ".json")
        held_runs = [r for g in held for r in runs_of(g)]
        try:
            res = fit_fn(set(held_runs))
        except FitError as e:
            for r in held_runs:
                rows.append(dict(run_id=r, calibration="", left_out=";".join(held), n_used="", fit_rms_cm="",
                                 note=f"no fit without it: {e}", holdout=holdout))
            continue
        cal = to_calibration(res, source=source)
        cal["held_out_runs"] = list(held_runs)
        if holdout != "run":
            cal["held_out_" + holdout + "s"] = list(held)
        cal["use_for"] = ("score the camera against the tape marks / odometry of the held_out_runs only; "
                          "these runs did not help make this calibration")
        with open(fname, "w") as f:
            json.dump(cal, f, indent=2, default=_jsonable)
        for r in held_runs:
            row = dict(run_id=r, calibration=os.path.relpath(fname, base), left_out=";".join(held),
                       n_used=res["n_inliers"], fit_rms_cm=round(res["rms_cm"], 3), note="", holdout=holdout)
            row.update(heldout_fn(res, r) or {})
            rows.append(row)
    cols = []
    for r in rows:
        cols += [k for k in r if k not in cols]
    man = os.path.join(base, stem + ".csv")
    with open(man, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    log(f"Wrote {len(fold_sets)} held-out calibrations (leave one {holdout if not folds else 'fold'} out) to {d}\n"
        f"  and {man} (which file scores which run)")
    return man, rows


def points_heldout(points):
    """For write_loro: a run's own points' errors (cm) under a fit made without them.
    Run points (--from-runs: one start and one end per run) give heldout_start_err_cm /
    heldout_end_err_cm. E00 placement spots (several sheet rows at the same spot, left
    out together) give one column per sheet row: heldout_<order>_<start|end>_err_cm."""
    def fn(res, g):
        cal = to_calibration(res)
        out = {}
        for p in points:
            if str(p.get("group")) == str(g) and p.get("enabled", True):
                x, y = to_floor(cal, p["u"], p["v"], float(p.get("z") or 0))
                spot = str(p.get("spot") or "")
                row, _, which = spot.rpartition("-")
                if not spot:
                    key = f"heldout_point{p.get('id', '')}_err_cm"
                elif row == str(g):           # a run's own start / end
                    key = f"heldout_{which}_err_cm"
                else:                         # sheet row <order>: both passes at a spot stay apart
                    key = "heldout_" + re.sub(r"[^A-Za-z0-9]+", "_", spot) + "_err_cm"
                k, n = key, 2
                while k in out:               # never overwrite another point's error
                    k, n = key.replace("_err_cm", f"_{n}_err_cm"), n + 1
                out[k] = round(math.hypot(x - p["x"], y - p["y"]), 3)
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
    with open(path, newline="", encoding="utf-8-sig") as f:   # -sig: Excel's "CSV UTF-8" starts with a BOM
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
    with open(p, newline="", encoding="utf-8-sig") as f:
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
    with open(path, newline="", encoding="utf-8-sig") as f:
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
        base = os.path.dirname(os.path.abspath(out_path))
        self.lm_path = os.path.join(base, "landmarks_picked.json")     # landmark clicks (resume file)
        self.lm_csv = os.path.join(base, "landmarks.csv")
        self.lm_map = None                                              # map.json to update (--map given)
        self.lm_picks = None

    def landmarks_json(self, picks=None):
        import landmark_pick as lp
        if picks is not None:
            self.lm_picks = picks
            lp.save_picks(self.lm_path, picks)
        if self.lm_picks is None:
            self.lm_picks = lp.load_picks(self.lm_path)
        cal, err = lp.load_calibration(self.out_path)
        warn = []
        if cal is not None and list(cal.get("image_size") or self.src.size) != list(self.src.size):
            warn.append(f"the calibration was made on {cal.get('image_size')} px pictures, this video is "
                        f"{self.src.size}: it doesn't fit this camera setup")
        area = self.settings.get("area") or default_area()
        return dict(ok=True, picks=self.lm_picks, results=lp.compute_all(cal, self.lm_picks, area),
                    cal_ok=cal is not None, cal_error=err, cal_warnings=warn,
                    cam_height_cm=((cal or {}).get("camera") or {}).get("height_cm"),
                    calibration=os.path.abspath(self.out_path), csv=os.path.abspath(self.lm_csv),
                    map=self.lm_map, picks_file=os.path.abspath(self.lm_path))

    def save_landmarks(self):
        import landmark_pick as lp
        if self.lm_picks is None:
            self.lm_picks = lp.load_picks(self.lm_path)
        r = lp.save_all(self.lm_picks, self.out_path, self.lm_csv, self.lm_map,
                        self.settings.get("area") or default_area())
        r.pop("results", None)
        return r

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


class LocalServer(ThreadingHTTPServer):
    """The page server. On Windows, SO_REUSEADDR lets a second copy bind the same port
    without an error (the browser then talks to either copy), so it is off there and
    the port is claimed exclusively."""
    allow_reuse_address = os.name != "nt"
    daemon_threads = True

    def server_bind(self):
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def make_server(port, handler):
    try:
        return LocalServer(("127.0.0.1", port), handler)
    except OSError as e:
        raise SystemExit(f"Port {port} is in use ({e.strerror or e}): is calibrate_floor.py already running in "
                         f"another window? Close it, or start with --port {port + 1}.")


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
                if u.path == "/api/landmarks":
                    with app.lock:
                        return self._send(200, app.landmarks_json())
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
                    if u.path == "/api/landmarks":
                        return self._send(200, app.landmarks_json(dict(version=1, landmarks=body.get("landmarks", []))))
                    if u.path == "/api/landmarks/save":
                        r = app.save_landmarks()
                        if r.get("ok"):
                            print("\n".join(["Landmarks:"] + r["messages"]))
                        return self._send(200, r)
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
    ap.add_argument("--area", help="x0,y0,x1,y1 cm: the floor the robots use (default: the fleet map's floor "
                                   "bounds, else 0,0,122,122)")
    ap.add_argument("--map", help="fleet map.json for the default area (default: ./map.json, where tools/fleet/app.py "
                                  "saves it, else tools/fleet/map.json). Given here, saving landmarks also "
                                  "writes them into it (old copy kept as .bak)")
    ap.add_argument("--landmarks-from", help="no page: landmark clicks (landmarks_picked.json format) -> landmarks.csv "
                                             "(and --map), using the saved floor calibration (--out)")
    ap.add_argument("--landmarks-out", help="landmarks.csv to write (default: beside the floor calibration)")
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
    g.add_argument("--holdout", choices=("run", "robot"), default="run",
                   help="survey: held-out calibrations leave out one run (default) or one ROBOT with all its runs "
                        "(floor_calibration_loro_robot.csv). A survey fit carries each robot's wheel-size error, so "
                        "headline T2/T3 camera-vs-wheels results should use --holdout robot or a click / tape-point "
                        "calibration")
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
    global MAP_PATH
    if args.area:
        print(f"area {args.area} cm (--area)")
    else:
        MAP_PATH = find_map(args.map)

    if args.camera is not None or os.path.isdir(args.source):
        folder = args.source
    else:
        folder = os.path.dirname(os.path.abspath(args.source))
    out = args.out or os.path.join(folder, "floor_calibration.json")
    lm_csv = args.landmarks_out or os.path.join(os.path.dirname(os.path.abspath(out)), "landmarks.csv")
    if args.landmarks_from:
        import landmark_pick as lp
        if not os.path.isfile(args.landmarks_from):
            raise SystemExit(f"--landmarks-from {args.landmarks_from}: no such file")
        r = lp.save_all(lp.load_picks(args.landmarks_from), out, lm_csv, os.path.abspath(args.map) if args.map else None,
                        [float(v) for v in args.area.split(",")] if args.area else default_area())
        if not r["ok"]:
            sys.exit("Landmarks not saved: " + r["error"])
        print("\n".join(r["messages"]))
        return r
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
    app.lm_csv = lm_csv
    app.lm_map = os.path.abspath(args.map) if args.map else None
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
    server = make_server(args.port, make_handler(app))
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

    robot_of = {p["run"]: p["robot"] for p in pairs if p.get("robot")}
    robot_of.update({r.run_id: r.robot for r in runs})
    no_robot = sorted({p["run"] for p in pairs} - set(robot_of))

    def fit_without(excl):
        res = fr.fit_survey(pairs, app.src.size, s.get("height_cm"), s.get("nadir_floor_cm"), f_px, k1,
                            s.get("area"), s["robot_top_cm"], exclude_runs=excl)
        res["mode"] = "survey"
        res["used_runs"] = sorted({p["run"] for p in pairs if p["run"] not in excl})
        res["used_robots"] = sorted({robot_of[r] for r in res["used_runs"] if r in robot_of})
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
    if args.holdout == "robot":
        members = {}
        for rid, rb in robot_of.items():
            if any(p["run"] == rid for p in pairs):
                members.setdefault(rb, []).append(rid)
        if no_robot:
            print(f"WARNING: {len(no_robot)} run(s) in {pairs_path} have no robot name (pairs written before "
                  f"2026-10-18): {', '.join(no_robot[:6])}{' ...' if len(no_robot) > 6 else ''}. They stay in every "
                  "leave-one-robot-out calibration; re-run --survey on their sessions to fix that.")
        if write_loro(out, list(members), fit_without, heldout, args.folds, source=cal["source"], members=members,
                      holdout="robot") is None:
            print(f"Only one robot ({', '.join(members) or '?'}) in the survey: no leave-one-robot-out calibration "
                  "possible. For camera-vs-wheels numbers use a click / tape-point calibration instead.")
    else:
        write_loro(out, [p["run"] for p in pairs], fit_without, heldout, args.folds, source=cal["source"])
        print("Note: these leave-one-RUN-out survey calibrations still use the scored robot's other runs, so its "
              "wheel-size error is in the camera fit. For headline T2/T3 numbers use --holdout robot or a "
              "click / tape-point calibration.")


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
.lm{border:1px solid var(--line);border-radius:4px;padding:5px;margin:6px 0}.lm.sel{border-color:var(--ok)}
.lm table td,.lm table th{text-align:left}.lm input.tape{width:120px}
#lmStatus{background:#3a2f12;border:1px solid var(--acc);border-radius:4px;padding:6px;margin:6px 0;font-size:14px}
details summary{cursor:pointer;color:var(--acc)}.big{font-size:14px}
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
  <div class="dim" id="hint">Click a point you know. Cross = your click; blue circle = where the fit puts the cm you typed; the line between them is the error.
  Ctrl+Z undo, Delete removes the selected point, G grid on/off.</div>
</div>
<canvas id="loupe" width="180" height="180"></canvas>
<div id="right">
  <div class="row tabs"><b>Mode:</b> <button id="mFloor" onclick="setMode('floor')">Floor points</button>
    <button id="mRobot" onclick="setMode('robot')">Robot points</button>
    <button id="mLm" onclick="setMode('lm')">Landmarks</button></div>
  <div id="lmBox" style="display:none">
    <div id="lmCal"></div>
    <div id="lmStatus"></div>
    <div class="row"><button class="big" onclick="lmNew('board')">+ New board</button>
      <button class="big" onclick="lmNew('block')">+ New brick / block</button>
      <button id="lmFinishBtn" style="display:none" onclick="lmFinish()">Finish this block</button></div>
    <div class="row">Thickness of new boards <input id="lmT" size="4" value="2"> cm <span class="dim">(tape it once; you can change it per board)</span></div>
    <details id="lmHelp"><summary>How to click a board, and why (read once)</summary>
      <div class="row" style="align-items:flex-start">
      <svg width="150" height="92" viewBox="0 0 150 92" style="flex:none;background:#111;border-radius:4px">
        <rect x="8" y="4" width="16" height="9" fill="#9aa0a6"/><text x="27" y="12" fill="#9aa0a6" font-size="9">camera</text>
        <line x1="0" y1="80" x2="150" y2="80" stroke="#777"/>
        <rect x="70" y="48" width="5" height="32" fill="#c8a165"/>
        <line x1="16" y1="13" x2="70" y2="48" stroke="#ff5a5a" stroke-dasharray="3,2"/>
        <line x1="70" y1="48" x2="108" y2="73" stroke="#ff5a5a" stroke-dasharray="3,2"/>
        <path d="M104 76l8 8M112 76l-8 8" stroke="#ff5a5a" stroke-width="2"/>
        <circle cx="70" cy="80" r="4" fill="#3ccf6e"/>
        <text x="30" y="90" fill="#3ccf6e" font-size="9">click here</text>
        <text x="96" y="70" fill="#ff5a5a" font-size="9">not here</text>
      </svg>
      <div>The camera looks down from about <span id="lmCamH">262</span> cm. A board's top edge is above the floor,
      so in the picture it looks pushed outward, away from the middle of the picture (red). Only the line where the
      board <b>meets the floor</b> (its foot, green) shows where it really is.<br>
      So: click the two <b>ends of the foot</b>, on the side of the board you can see. The hidden back side doesn't
      matter; the tools work out which side the robot sees.</div></div>
      <div style="margin-top:4px"><b>Brick or cinderblock:</b> click its corners one after another, going around it.
      3 corners are enough for a box (the 4th is filled in); press Finish after 2 or 3. 2 opposite corners only if it
      lines up with the taped axes. A tall block hides its far floor corners: then pick "corners of its top", type its
      height (tape, floor to top) and click the top corners instead.</div>
      <div style="margin-top:4px"><b>Adjust:</b> drag a circle. <b>Tape:</b> if you measure an end with the tape, type
      it in the end's tape box (x, y in inches from the taped origin, comma between, e.g. <i>36 3/32, 12 1/2</i>); it
      then replaces your click. Ctrl+Z undoes, Esc stops clicking.</div>
    </details>
    <div class="row"><button class="big" onclick="lmSave()" style="border-color:var(--ok)">Save landmarks</button>
      <span class="dim">writes landmarks.csv (and the map, if you started with --map)</span></div>
    <div id="lmSaved" class="dim" style="white-space:pre-wrap"></div>
    <div id="lmList"></div>
  </div>
  <div id="calBox">
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
  mLm.classList.toggle('on',m=='lm');lmBox.style.display=m=='lm'?'':'none';calBox.style.display=m=='lm'?'none':'';
  hint.textContent=m=='lm'?'Landmarks: find a frame where the boards show (slider above), then use the buttons on the right. Drag a circle to adjust it.':
   'Click a point you know. Cross = your click; blue circle = where the fit puts the cm you typed; the line between them is the error. Ctrl+Z undo, Delete removes the selected point, G grid on/off.';
  if(m=='lm')lmLoad();
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
  if(mode=='lm'){drawLm();return}
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
  if(mode=='lm'){if(dragged){dragged=false;return}lmClick(uv);return}
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
  if((e.ctrlKey||e.metaKey)&&e.key=='z'){e.preventDefault();mode=='lm'?lmUndo():undo()}
  else if(mode=='lm'&&e.key=='Escape'){lmActive=null;lmRender()}
  else if(e.key=='Delete'&&sel!=null&&mode!='lm')delPt(sel);else if(e.key=='g'||e.key=='G'){showGrid.checked=!showGrid.checked;draw()}
  else if(e.key=='ArrowRight')step(e.shiftKey?30:1);else if(e.key=='ArrowLeft')step(e.shiftKey?-30:-1)});

// ---------------------------------------------------------------- landmarks
let LM={picks:{landmarks:[]},results:[],cal_ok:false},lmActive=null,lmSel=null,lmHist=[],drag=null,dragged=false,lmSeq=0;
function L(){return LM.picks.landmarks}
async function lmLoad(){const r=await j('/api/landmarks');LM=r;lmRender()}
async function lmPush(){const my=++lmSeq;const r=await j('/api/landmarks',{landmarks:L()});if(my!==lmSeq)return;LM=r;lmRender()}
function lmChange(fn){lmHist.push(JSON.stringify(L()));fn();lmRender();return lmPush()}
async function lmUndo(){if(!lmHist.length)return;LM.picks.landmarks=JSON.parse(lmHist.pop());lmActive=null;lmRender();await lmPush()}
function defName(kind){const used=new Set(L().map(l=>String(l.name).toLowerCase()));
  if(kind=='board'){for(const c of 'ABCDEFGHIJKLMNOPQRSTUVWXYZ')if(!used.has(c.toLowerCase()))return c}
  for(let i=1;;i++){const n=(kind=='board'?'board ':'block ')+i;if(!used.has(n))return n}}
function lmNew(kind){if(lmActive!=null&&L()[lmActive]&&!L()[lmActive].clicks.length)L().splice(lmActive,1);
  const t=parseFloat(lmT.value);
  lmChange(()=>{L().push({name:defName(kind),kind:kind,clicks:[],fix:[],frame:frame,thickness_cm:kind=='board'?(t>0?t:2):null,
    height_cm:null,clicked_at:'floor'})});lmActive=lmSel=L().length-1;lmRender()}
function lmClick(uv){
  if(lmActive==null||!L()[lmActive]){flash('First press "+ New board" or "+ New brick / block" (on the right), then click.');return}
  const i=lmActive,lm=L()[i];
  lmChange(()=>{lm.clicks.push([uv.u,uv.v]);lm.fix.push(null);lm.frame=frame;
    if((lm.kind=='board'&&lm.clicks.length>=2)||lm.clicks.length>=4)lmActive=null});}
function lmFinish(){const lm=L()[lmActive];if(lm&&lm.clicks.length>=2){lmActive=null;lmRender()}}
function lmRepick(i){lmChange(()=>{L()[i].clicks=[];L()[i].fix=[]});lmActive=lmSel=i;lmRender()}
function lmDel(i){if(!confirm('Delete "'+L()[i].name+'"?'))return;lmChange(()=>{L().splice(i,1)});
  if(lmActive===i)lmActive=null;else if(lmActive>i)lmActive--;lmSel=null;lmRender()}
function lmSet(i,k,v){lmChange(()=>{const lm=L()[i];
  if(k=='name')lm.name=v.trim();else if(k=='clicked_at')lm.clicked_at=v;
  else{const x=v.trim()===''?null:parseFloat(v);lm[k]=(x===null||isFinite(x))?x:lm[k]}})}
function lmFix(i,k,v){lmChange(()=>{const lm=L()[i];while(lm.fix.length<lm.clicks.length)lm.fix.push(null);lm.fix[k]=v.trim()||null})}
function lmSelect(i){lmSel=i;const lm=L()[i];lmRender();if(lm&&lm.frame!=null&&lm.frame!=frame)goFrame(lm.frame)}
let flashT=null;function flash(t){lmStatus.innerHTML='<span class="err">'+esc(t)+'</span>';clearTimeout(flashT);flashT=setTimeout(lmRender,3500)}
function f1(v){return v==null?'-':(+v).toFixed(1)}
function lmRender(){const act=document.activeElement,aid=act&&act.id,pos=act&&act.selectionStart;
  lmCal.innerHTML=LM.cal_ok?(LM.cal_warnings||[]).map(w=>`<div class="warn">&#9888; ${esc(w)}</div>`).join(''):
    `<div class="err big">${esc(LM.cal_error||'No floor calibration.')}</div>`;
  if(LM.cam_height_cm)lmCamH.textContent=Math.round(LM.cam_height_cm);
  const a=lmActive!=null?L()[lmActive]:null;let st;
  if(!a)st=L().length?'Done clicking. Check the positions below, then press <b>Save landmarks</b>. To add another: <b>+ New board</b> or <b>+ New brick / block</b>.':
    'Move the slider to a frame where the boards show, then press <b>+ New board</b> (or <b>+ New brick / block</b>).';
  else if(a.kind=='board')st=a.clicks.length==0?`Board <b>${esc(a.name)}</b>: click one <b>end of its foot</b> (where the board meets the floor, not its top edge).`:
    `Board <b>${esc(a.name)}</b>: now click the <b>other end</b> of its foot.`;
  else st=`Block <b>${esc(a.name)}</b>: click corner ${a.clicks.length+1} of its ${a.clicked_at=='top'?'<b>top</b>':'footprint <b>on the floor</b>'}, going around it.`+
    (a.clicks.length>=2?' Or press <b>Finish this block</b>'+(a.clicks.length==2?' (2 opposite corners: only if it lines up with the taped axes).':' (the 4th corner is filled in).'):'');
  lmStatus.innerHTML=st;lmFinishBtn.style.display=a&&a.kind=='block'&&a.clicks.length>=2?'':'none';
  lmList.innerHTML=L().map((lm,i)=>{const r=LM.results[i]||{ends:[],errors:[],warnings:[]};const board=lm.kind=='board';
    const ends=lm.clicks.map((c,k)=>{const e=r.ends[k];return `<tr><td>${board?'end':'corner'} ${k+1}</td>
      <td>${e?f1(e.x)+', '+f1(e.y):'-'}</td><td>${e?esc(e.x_in)+', '+esc(e.y_in):'-'}</td>
      <td><input class="tape" id="lm-f-${i}-${k}" value="${esc(lm.fix[k]||'')}" placeholder="tape, inches: x, y" title="${board?'Tape the same spot you clicked: this end of the foot, on the side the camera sees.':'Tape this corner where it meets the floor.'} x, y from the taped origin, comma between, e.g. 36 3/32, 12 1/2" onchange="lmFix(${i},${k},this.value)"></td></tr>`+
      (e&&e.tape&&!e.tape_error?`<tr><td></td><td colspan="3" class="dim">using the tape; your click said ${f1(e.click_x)}, ${f1(e.click_y)} cm (${esc(e.click_x_in)}, ${esc(e.click_y_in)}), ${f1(e.diff_cm)} cm away</td></tr>`:'')}).join('');
    const size=r.length_cm!=null?`Length ${f1(r.length_cm)} cm (${esc(r.length_in)})`:(r.sides_cm?`Sides ${r.sides_cm.map(f1).join(', ')} cm (${r.sides_in.map(esc).join(', ')})`:'');
    return `<div class="lm ${i===lmSel?'sel':''}" onclick="if(event.target.tagName!='INPUT'&&event.target.tagName!='SELECT'&&event.target.tagName!='BUTTON')lmSelect(${i})">
     <div class="row"><b>${board?'Board':'Block'}</b> name <input id="lm-n-${i}" size="10" value="${esc(lm.name)}" onchange="lmSet(${i},'name',this.value)">
      <span class="dim">frame ${lm.frame??'-'}</span> <button onclick="lmRepick(${i})">Click again</button> <button onclick="lmDel(${i})">Delete</button></div>
     <div class="row">${board?`thickness <input id="lm-t-${i}" size="4" value="${lm.thickness_cm??''}" onchange="lmSet(${i},'thickness_cm',this.value)"> cm &nbsp;`:
       `I clicked the <select id="lm-a-${i}" onchange="lmSet(${i},'clicked_at',this.value)"><option value="floor" ${lm.clicked_at!='top'?'selected':''}>corners on the floor</option><option value="top" ${lm.clicked_at=='top'?'selected':''}>corners of its top</option></select>`}
      height <input id="lm-h-${i}" size="4" value="${lm.height_cm??''}" onchange="lmSet(${i},'height_cm',this.value)"> cm <span class="dim">${board||lm.clicked_at!='top'?'(optional: draws its top, to check)':'(needed)'}</span></div>
     ${lm.clicks.length?`<table><tr><th></th><th>floor x, y (cm)</th><th>inches</th><th>tape (inches)</th></tr>${ends}</table>`:''}
     <div class="dim">${size}</div>
     ${r.errors.filter(x=>x!='no floor calibration').map(x=>`<div class="err">${esc(x)}</div>`).join('')}${r.warnings.map(x=>`<div class="warn">&#9888; ${esc(x)}</div>`).join('')}</div>`}).join('');
  if(aid){const el=document.getElementById(aid);if(el&&el!==act){el.focus();try{el.setSelectionRange(pos,pos)}catch(_){}}}
  draw()}
function poly(pts,col,w,dash){ctx.strokeStyle=col;ctx.lineWidth=w;ctx.setLineDash(dash);ctx.beginPath();
  pts.forEach((p,i)=>{const[x,y]=P(p[0],p[1]);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.closePath();ctx.stroke();ctx.setLineDash([])}
function drawLm(){L().forEach((lm,i)=>{const r=LM.results[i];const hi=i===lmSel||i===lmActive;
  if(r&&r.outline)poly(r.outline,hi?'#3ccf6e':'rgba(60,207,110,.65)',hi?2:1.2,[]);
  if(r&&r.top_outline)poly(r.top_outline,'rgba(255,176,32,.85)',1,[4,3]);
  if(lm.kind=='board'&&lm.clicks.length==2){const[a,b]=lm.clicks.map(c=>P(c[0],c[1]));ctx.strokeStyle='#ffe040';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke()}
  lm.clicks.forEach((c,k)=>{const[x,y]=P(c[0],c[1]);ctx.strokeStyle=i===lmActive?'#ffb020':(hi?'#fff':'#ddd');ctx.lineWidth=2;
    ctx.beginPath();ctx.arc(x,y,5,0,7);ctx.stroke();ctx.fillStyle='#fff';ctx.font='11px sans-serif';ctx.fillText((k==0?lm.name+' ':'')+(k+1),x+6,y-6);
    const e=r&&r.ends[k];if(e&&e.tape_px){const[a,b]=P(e.tape_px[0],e.tape_px[1]);ctx.strokeStyle='#4aa3ff';ctx.setLineDash([3,2]);ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(a,b);ctx.stroke();ctx.setLineDash([]);
      ctx.beginPath();ctx.moveTo(a,b-5);ctx.lineTo(a+5,b);ctx.lineTo(a,b+5);ctx.lineTo(a-5,b);ctx.closePath();ctx.stroke()}})})}
cv.addEventListener('mousedown',e=>{if(mode!='lm')return;const r=cv.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;let best=null;
  L().forEach((lm,i)=>lm.clicks.forEach((c,k)=>{const[x,y]=P(c[0],c[1]);const d=Math.hypot(x-mx,y-my);if(d<9&&(!best||d<best.d))best={i,k,d}}));
  if(best){drag={i:best.i,k:best.k,moved:false,before:JSON.stringify(L())};e.preventDefault()}});
cv.addEventListener('mousemove',e=>{if(!drag)return;const[u,v]=evPx(e);L()[drag.i].clicks[drag.k]=[+u.toFixed(2),+v.toFixed(2)];drag.moved=true;draw()});
window.addEventListener('mouseup',()=>{if(!drag)return;const d=drag;drag=null;if(d.moved){dragged=true;lmHist.push(d.before);lmSel=d.i;lmPush();setTimeout(()=>{dragged=false},50)}});
async function lmSave(){lmSaved.textContent='saving...';await lmPush();const r=await j('/api/landmarks/save',{});
  lmSaved.innerHTML=r.ok?esc(r.messages.join('\n')):`<span class="err">${esc(r.error)}</span>`}
init();
</script></body></html>
"""

if __name__ == "__main__":
    main()
