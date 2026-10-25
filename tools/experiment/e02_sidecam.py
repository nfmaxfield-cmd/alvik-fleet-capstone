"""Side camera (EMEET C950, steep view) offline: calibrate it to the floor from robots seen by both cameras, cross-check
it against the overhead at moments a robot stands still, and count where it sees robots the overhead does not.

Built 2026-10-24 for steps 2-3 of docs/build-plans/second-camera-plan.md, on roam1 (2026-10-20). What roam1 has for the
side camera: the studio's motion detections (labels/<rec>_side/cache/motion_detections.csv, side pixels, 27,928 frames);
NOT its video or frames.csv (they stayed on Nick's laptop). So:
  - time: side frame i is taken at t = offset + i * dt (dt = 960.02 s / 27,928 frames, the cache's own figures), and
    the offset is found by how well the two cameras' robots line up (find_offset);
  - calibration: floor positions from the overhead (its roam1 calibration) paired with side pixels at the same
    moment, a RANSAC plane homography to throw out pairs of different robots, then CameraModel.from_points on
    per-cell medians (fit_from_pairs). The board's corners are then predicted in the side picture;
    order_corners_like (e02_camera) puts corners found in a real side picture into floorcal's order, once one exists.
The side positions are on the plane SIDE_Z_CM up: the side camera's outline of a robot is its top plus its near side
face, whose centre is about half way up.

Run on roam1 (paths are arguments; nothing is downloaded here):
    python tools/experiment/e02_sidecam.py --overhead-dets <...overhead/cache/motion_detections.csv>
        --side-dets <...side/cache/motion_detections.csv> --overhead-frames <frames.csv> --out <folder>
"""
import argparse
import collections
import csv
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import e02_camera as CAM  # noqa: E402
from cameras import CameraModel  # noqa: E402

SIDE_Z_CM = 0.5 * CAM.ROBOT_TOP_CM   # chosen: the side outline's centre is about half way up the robot (DOE: no, measure it)
PAIR_TOL_S = 0.02                    # chosen: an overhead frame within this of a side frame is "the same moment"
RANSAC_PX = 4.0                      # chosen: homography inlier limit in side pixels (DOE: no, a sweep is enough)
SLOW_CM = 1.5                        # chosen: a pair is used only if the overhead robot moved under this in +-0.2 s
CELL_CM = 10.0                       # chosen: pairs are reduced to one median point per floor cell this big
NEAR_CM = 10.0                       # chosen: an overhead sighting this near a side one is the same robot


# =============================================================================== reading the studio caches
def read_dets(path, cls=("alvik",)):
    """{frame: [(u, v, score), ...]} from a studio motion_detections.csv."""
    out = collections.defaultdict(list)
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["cls"] in cls:
                out[int(r["frame"])].append((float(r["x"]), float(r["y"]), float(r["score"] or 0.0)))
    return dict(out)


def read_frame_times(path):
    """{frame: t_mono} from a camrec frames.csv."""
    with open(path) as f:
        return {int(r["frame"]): float(r["t_mono"]) for r in csv.DictReader(f)}


def overhead_floor(cam, dets, times, z=CAM.ROBOT_TOP_CM):
    """(t array sorted, [list of (x, y)] per frame) from the overhead's detections."""
    fr = sorted(f for f in dets if f in times)
    ts = np.array([times[f] for f in fr])
    xy = [[cam.to_floor(u, v, z) for u, v, _ in dets[f]] for f in fr]
    return ts, xy


def side_time(frame, offset, dt):
    return offset + frame * dt


def _nearest(ts, t, tol):
    i = int(np.searchsorted(ts, t))
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(ts) and abs(ts[j] - t) <= tol and (best is None or abs(ts[j] - t) < abs(ts[best] - t)):
            best = j
    return best


# =============================================================================== pairing and the fit
def pairs(side, ots, oxy, offset, dt, tol=PAIR_TOL_S, slow_cm=None):
    """Every (x, y, u, v, t) of an overhead sighting and a side sighting at the same moment (all combinations: the
    homography's RANSAC throws out the pairs of different robots). slow_cm: keep only overhead sightings that have a
    sighting within slow_cm 0.2 s before and after (a slow robot, so a small clock error moves it little)."""
    out = []
    for f, l in side.items():
        t = side_time(f, offset, dt)
        j = _nearest(ots, t, tol)
        if j is None:
            continue
        cand = oxy[j]
        if slow_cm is not None:
            keep = []
            for x, y in cand:
                ok = True
                for dtt in (-0.2, 0.2):
                    k = _nearest(ots, ots[j] + dtt, 0.05)
                    if k is None or not any(math.hypot(x - a, y - b) < slow_cm for a, b in oxy[k]):
                        ok = False
                        break
                if ok:
                    keep.append((x, y))
            cand = keep
        for u, v, _ in l:
            for x, y in cand:
                out.append((x, y, u, v, t))
    return np.array(out, float).reshape(-1, 5)


def homography_inliers(P, px=RANSAC_PX, seed=0):
    """(H floor->side px, inlier mask) by RANSAC; (None, None) with under 8 pairs."""
    if len(P) < 8:
        return None, None
    cv2.setRNGSeed(seed)
    H, m = cv2.findHomography(P[:, :2].astype(np.float32), P[:, 2:4].astype(np.float32), cv2.RANSAC, px,
                              maxIters=5000, confidence=0.999)
    return H, (m.ravel() > 0) if m is not None else None


def find_offset(side, ots, oxy, dt, lo=-3.0, hi=3.0, step=0.05):
    """The side clock's offset (s) that lines the two cameras' robots up best: [(offset, inliers)] and the best."""
    scores = []
    for a in np.arange(lo, hi + 1e-9, step):
        P = pairs(side, ots, oxy, a, dt)
        _, m = homography_inliers(P)
        scores.append((round(float(a), 3), int(m.sum()) if m is not None else 0))
    return max(scores, key=lambda s: s[1])[0], scores


def cell_points(P, cell=CELL_CM, min_n=3):
    """One (x, y, u, v, n) per floor cell: the median of the pairs in it (balances the board; drops strays)."""
    key = collections.defaultdict(list)
    for p in P:
        key[(int(p[0] // cell), int(p[1] // cell))].append(p)
    out = []
    for k, l in sorted(key.items()):
        if len(l) >= min_n:
            a = np.array(l)
            out.append((*np.median(a[:, :4], axis=0), len(l)))
    return np.array(out, float).reshape(-1, 5)


def fit_from_pairs(P, image_size=(640, 480), z=SIDE_Z_CM, name="side", rounds=2, px=RANSAC_PX):
    """Side CameraModel from (x, y, u, v, t) pairs: RANSAC homography -> per-cell medians -> from_points; then pairs
    are picked again by the camera model itself (within px) and it is fitted again. Returns (cam, info)."""
    H, m = homography_inliers(P, px)
    if H is None or m is None or m.sum() < 8:
        raise ValueError("too few pairs line up (%d)" % (0 if m is None else int(m.sum())))
    use = P[m]
    cam = None
    for _ in range(rounds):
        C = cell_points(use)
        if len(C) < 6:
            raise ValueError("pairs cover only %d floor cells" % len(C))
        cam = CameraModel.from_points(C[:, 2:4], np.column_stack([C[:, :2], np.full(len(C), z)]), image_size,
                                      k1="off", mirror="auto", name=name)
        u, v = cam.project(np.column_stack([P[:, :2], np.full(len(P), z)]))
        r = np.hypot(u - P[:, 2], v - P[:, 3])
        use = P[np.isfinite(r) & (r < px)]
    info = dict(pairs=int(len(P)), homography_inliers=int(m.sum()), model_inliers=int(len(use)),
                cells=int(len(C)), z_cm=z, rms_px=cam.fit.get("rms_px"), loo_rms_cm=cam.fit.get("loo_rms_cm"),
                loo_max_cm=cam.fit.get("loo_max_cm"))
    return cam, info


def save_side_calibration(cam, path, info, board=(CAM.BOARD_W_CM, CAM.BOARD_L_CM), note=""):
    """Write the side camera file (CameraModel.to_dict + how it was made + where the board's corners fall)."""
    d = cam.to_dict()
    d["name"] = cam.name or "side"
    W, L = board
    corners = [[0.0, 0.0], [W, 0.0], [W, L], [0.0, L]]
    d["board_corners_px"] = [[round(c, 1) for c in cam.project((x, y, 0.0))] for x, y in corners]
    d["robot_match_calibration"] = dict(info, note=note)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        json.dump(d, f, indent=1)
    return path


# =============================================================================== still moments and the cross-check
def overhead_tracks(ots, oxy, join_cm=3.0, gap_s=0.3):
    """Simple nearest-neighbour tracks of overhead sightings: [[(t, x, y), ...], ...]."""
    live, done = [], []
    for t, l in zip(ots, oxy):
        used = set()
        for x, y in l:
            best, bd = None, join_cm
            for k, tr in enumerate(live):
                if k in used or t - tr[-1][0] > gap_s:
                    continue
                d = math.hypot(x - tr[-1][1], y - tr[-1][2])
                if d < bd:
                    best, bd = k, d
            if best is None:
                live.append([(t, x, y)])
                used.add(len(live) - 1)
            else:
                live[best].append((t, x, y))
                used.add(best)
        keep = []
        for tr in live:
            (keep if t - tr[-1][0] <= gap_s else done).append(tr)
        live = keep
    return done + live


def still_moments(tracks, min_s=0.4, spread_cm=0.4, every_s=1.0):
    """(t, x, y) of a robot standing still (e02_camera.still_position's rule), at most one per every_s per track."""
    out = []
    for tr in tracks:
        last = -1e9
        for i in range(len(tr)):
            if tr[i][0] - last < every_s:
                continue
            j0 = i
            while j0 > 0 and tr[i][0] - tr[j0 - 1][0] <= min_s:
                j0 -= 1
            p = CAM.still_position(tr[j0:i + 1], min_s, spread_cm)
            if p is not None:
                out.append((tr[i][0], p[0], p[1]))
                last = tr[i][0]
    return out


def side_at(cam, side, t, x, y, offset, dt, z=SIDE_Z_CM, win_s=0.2, near_px=None):
    """Median side floor position near (x, y) over t +- win_s, or None. near_px: search radius in side pixels around
    where the side camera should see (x, y) (default: 1.5 robot widths there)."""
    u0, v0 = cam.project((x, y, z))
    if not np.isfinite(u0):
        return None
    if near_px is None:
        near_px = 1.5 * CAM.ROBOT_CM * cam.px_per_cm(x, y, z)
    f0 = int(math.floor((t - win_s - offset) / dt))
    f1 = int(math.ceil((t + win_s - offset) / dt))
    got = []
    for f in range(f0, f1 + 1):
        for u, v, _ in side.get(f, ()):
            if math.hypot(u - u0, v - v0) < near_px:
                p = cam.to_floor(u, v, z)
                if np.isfinite(p[0]) and np.isfinite(p[1]):
                    got.append(p)
    if len(got) < 2:
        return None
    a = np.array(got)
    return float(np.median(a[:, 0])), float(np.median(a[:, 1])), len(got)


def lean(cam, x, y, dx, dy):
    """(component of the side-minus-overhead difference toward the side camera's floor point, distance to it, cm)."""
    C = cam.center
    tx, ty = C[0] - x, C[1] - y
    d = math.hypot(tx, ty)
    return (dx * tx + dy * ty) / max(d, 1e-9), d


# =============================================================================== where each camera sees
def region(x, y, W=CAM.BOARD_W_CM, L=CAM.BOARD_L_CM, edge_cm=12.0, corner_cm=25.0, blocked=("x+", "y+")):
    """Board region of a floor point: 'off', 'blocked corner', 'along a board', 'open edge', 'middle'. blocked: which
    edges carry a board ('x+' = the edge at x = W, 'y+' = at y = L; the simulator's default layout)."""
    if x < -8 or y < -8 or x > W + 8 or y > L + 8:
        return "off"
    near = dict(xm=x < edge_cm, xp=x > W - edge_cm, ym=y < edge_cm, yp=y > L - edge_cm)
    bx = "x+" in blocked
    by = "y+" in blocked
    if (x > W - corner_cm if bx else x < corner_cm) and (y > L - corner_cm if by else y < corner_cm):
        return "blocked corner"
    if (near["xp"] if bx else near["xm"]) or (near["yp"] if by else near["ym"]):
        return "along a board"
    if any(near.values()):
        return "open edge"
    return "middle"


def region_split(x, y, W=CAM.BOARD_W_CM):
    """region() with the boards and open edges named one by one (for the report's table)."""
    r = region(x, y)
    if r == "along a board":
        return "board at x=122" if x > W - 12.0 else "board at y=122"
    if r == "open edge":
        return "open edge x=0" if x < 12.0 else "open edge y=0"
    return r


def coverage(cam, side, ots, oxy, offset, dt, z=SIDE_Z_CM, near_cm=NEAR_CM, tol=0.04, **reg):
    """For every side robot sighting on the board (at a moment the overhead has a picture): whether the overhead also
    saw a robot within near_cm. {region: dict(side=n, overhead_too=n, side_only=n)}."""
    out = collections.defaultdict(lambda: dict(side=0, overhead_too=0, side_only=0))
    for f, l in side.items():
        t = side_time(f, offset, dt)
        j = _nearest(ots, t, tol)
        for u, v, _ in l:
            x, y = cam.to_floor(u, v, z)
            if not (np.isfinite(x) and np.isfinite(y)):
                continue
            r = region(x, y, **reg)
            if r == "off":
                continue
            c = out[r]
            c["side"] += 1
            if j is not None and any(math.hypot(x - a, y - b) < near_cm for a, b in oxy[j]):
                c["overhead_too"] += 1
            else:
                c["side_only"] += 1
    return dict(out)


def overhead_coverage(cam, side, ots, oxy, offset, dt, z=SIDE_Z_CM, near_cm=NEAR_CM, side_live_s=1.0, **reg):
    """The other way round: every overhead sighting on the board, at a time the side camera was producing sightings
    (one within side_live_s), and whether the side saw a robot within near_cm. {region: dict(overhead, side_too)}."""
    sframes = np.array(sorted(side))
    stimes = side_time(sframes, offset, dt)
    out = collections.defaultdict(lambda: dict(overhead=0, side_too=0))
    for t, l in zip(ots, oxy):
        i = int(np.searchsorted(stimes, t))
        near = [k for k in (i - 1, i) if 0 <= k < len(stimes) and abs(stimes[k] - t) <= side_live_s]
        if not near:
            continue
        k = min(near, key=lambda k: abs(stimes[k] - t))
        sxy = [cam.to_floor(u, v, z) for u, v, _ in side[int(sframes[k])]] if abs(stimes[k] - t) < 0.04 else []
        for x, y in l:
            r = region(x, y, **reg)
            if r == "off":
                continue
            out[r]["overhead"] += 1
            if any(math.hypot(x - a, y - b) < near_cm for a, b in sxy):
                out[r]["side_too"] += 1
    return dict(out)


def presence(cam, side, ots, oxy, offset, dt, z=SIDE_Z_CM, bin_s=0.5, near_cm=NEAR_CM, min_n=2, t_range=None,
             region_fn=None):
    """Robot presences per time bin, from both cameras' sightings together: in every bin_s, all sightings (floor
    positions) are grouped (within near_cm of a group's first sighting); a camera sees a group when it has at least
    min_n sightings in it. Returns {region: dict(both, overhead_only, side_only)} counted in bins (time = bins *
    bin_s). region_fn(x, y) names the region (default region())."""
    region_fn = region_fn or region
    sf = np.array(sorted(side))
    st = side_time(sf, offset, dt)
    lo = max(float(ots[0]), float(st[0])) if t_range is None else t_range[0]
    hi = min(float(ots[-1]), float(st[-1])) if t_range is None else t_range[1]
    out = collections.defaultdict(lambda: dict(both=0, overhead_only=0, side_only=0))
    t = lo
    while t < hi:
        pts = []
        for i in range(int(np.searchsorted(ots, t)), int(np.searchsorted(ots, t + bin_s))):
            pts += [(x, y, 0) for x, y in oxy[i]]
        for i in range(int(np.searchsorted(st, t)), int(np.searchsorted(st, t + bin_s))):
            for u, v, _ in side[int(sf[i])]:
                x, y = cam.to_floor(u, v, z)
                if np.isfinite(x) and np.isfinite(y):
                    pts.append((x, y, 1))
        groups = []
        for x, y, c in pts:
            for g in groups:
                if math.hypot(x - g[0], y - g[1]) < near_cm:
                    g[2][c] += 1
                    break
            else:
                groups.append([x, y, [0, 0]])
                groups[-1][2][c] += 1
        for x, y, n in groups:
            r = region_fn(x, y)
            if r == "off":
                continue
            o, s = n[0] >= min_n, n[1] >= min_n
            if o and s:
                out[r]["both"] += 1
            elif o:
                out[r]["overhead_only"] += 1
            elif s:
                out[r]["side_only"] += 1
        t += bin_s
    return dict(out)


def check_picture(cam, side, path, size=(640, 480), board=None, every=25):
    """draw_check on a plain grey canvas (no side picture exists for roam1): the calibrated board outline, the
    predicted corners, and every every-th side robot sighting as a dot."""
    W, H = size
    img = np.full((H, W), 60, np.uint8)
    out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for k, f in enumerate(sorted(side)):
        if k % every:
            continue
        for u, v, _ in side[f]:
            cv2.circle(out, (int(round(u)), int(round(v))), 1, (180, 180, 180), -1)
    import e02_plan as PL
    b = board or PL.Board(xboard_side=1, yboard_side=1)
    corners = [cam.project((x, y, 0.0)) for x, y in ((b.x0, b.y0), (b.x1, b.y0), (b.x1, b.y1), (b.x0, b.y1))]
    return CAM.draw_check(out, cam, path, board=b, corners=corners)


# =============================================================================== main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--overhead-dets", required=True)
    ap.add_argument("--side-dets", required=True)
    ap.add_argument("--overhead-frames", required=True)
    ap.add_argument("--overhead-cal", default=os.path.join(HERE, "data", "overhead_floorcal_roam1.json"))
    ap.add_argument("--side-frames", type=int, default=27928)
    ap.add_argument("--side-seconds", type=float, default=960.02)
    ap.add_argument("--out", required=True, help="folder for results.json and the check picture")
    ap.add_argument("--cal-out", default=os.path.join(HERE, "data", "side_floorcal_roam1.json"))
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    ocam = CAM.load_camera(a.overhead_cal)
    ots, oxy = overhead_floor(ocam, read_dets(a.overhead_dets), read_frame_times(a.overhead_frames))
    side = read_dets(a.side_dets)
    dt = a.side_seconds / a.side_frames
    res = dict(dt=dt)

    off, scores = find_offset(side, ots, oxy, dt, -3.0, 3.0, 0.25)
    off, fine = find_offset(side, ots, oxy, dt, off - 0.3, off + 0.3, 0.02)
    res.update(offset_s=off, offset_scan=scores, offset_fine=fine)
    print("side clock offset %.2f s" % off)

    # 2-fold by alternating 60 s blocks: fit on one, cross-check the other's still moments
    P = pairs(side, ots, oxy, off, dt, slow_cm=SLOW_CM)
    blk = (P[:, 4] // 60).astype(int) % 2
    tracks = overhead_tracks(ots, oxy)
    still = still_moments(tracks)
    rows = []
    for fold in (0, 1):
        cam_f, inf = fit_from_pairs(P[blk == fold])
        for t, x, y in still:
            if int(t // 60) % 2 == fold:
                continue
            s = side_at(cam_f, side, t, x, y, off, dt)
            if s is None:
                continue
            dx, dy = s[0] - x, s[1] - y
            ln, dist = lean(cam_f, x, y, dx, dy)
            rows.append(dict(fold=fold, t=round(t, 2), x=round(x, 1), y=round(y, 1), sx=round(s[0], 1),
                             sy=round(s[1], 1), d_cm=round(math.hypot(dx, dy), 2), lean_cm=round(ln, 2),
                             dist_cm=round(dist, 1), region=region(x, y), n_side=s[2]))
        res["fold%d" % fold] = inf
    res["cross_check"] = rows

    cam, info = fit_from_pairs(P)
    save_side_calibration(cam, a.cal_out, info, note=(
        "roam1 2026-10-20, from robots seen by both cameras (no side picture was uploaded): overhead floor positions "
        "(overhead_floorcal_roam1.json) paired with side motion detections, side clock offset %.2f s, side z %.2f cm. "
        "Made by tools/experiment/e02_sidecam.py." % (off, SIDE_Z_CM)))
    res["calibration"] = dict(info, path=a.cal_out, pose=cam.pose_summary() if hasattr(cam, "pose_summary") else None)
    check_picture(cam, side, os.path.join(a.out, "side_check_roam1.png"))
    res["coverage_side"] = coverage(cam, side, ots, oxy, off, dt)
    res["coverage_overhead"] = overhead_coverage(cam, side, ots, oxy, off, dt)
    # robot-time per region and camera, by roam1's phases (events.csv: calibration 170 s, roaming 480 s, objects 730 s)
    res["presence"] = {name: presence(cam, side, ots, oxy, off, dt, t_range=rng, region_fn=region_split)
                       for name, rng in (("calibration+roaming 170-730 s", (170.0, 730.0)),
                                         ("objects 730-940 s", (730.0, 940.0)))}
    with open(os.path.join(a.out, "results.json"), "w") as f:
        json.dump(res, f, indent=1, default=float)
    print(json.dumps({k: v for k, v in res.items() if k not in ("offset_scan", "offset_fine", "cross_check")},
                     indent=1, default=float))
    return res


if __name__ == "__main__":
    main()
