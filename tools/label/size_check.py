"""Camera scale check from things of known size in the picture (Nick, 2026-10-25: "Can't we use the robot's size?
And later the dock's size?").

The floor calibration turns a pixel into floor cm. If it is right, the robot's top, measured through it, comes out
at the robot's real size everywhere on the board. If the calibration's scale is off (camera height or focal length
wrong), every robot comes out too big or too small; if its tilt is off, robots come out bigger on one side of the
board than the other. So: fit the outline of a known object in a few pictures spread over the board, measure it in
floor cm, and compare with the real size, region by region.

Two ways to get the outline:
  - by hand (the mark-the-robot page, "Size check"): four corner handles start on the calibration's own prediction
    of the object (or on the automatic outline when one is found); you drag them onto the object's top corners;
  - automatically (auto_outline): the bright or dark blob around a known position (a mark, or the tracker's
    position), its smallest enclosing rectangle. Cheap, every picture, but the blob can take in the robot's shadow
    or the glow its light throws on the floor, which makes it look bigger. Pictures with both a hand fit and an
    automatic one measure that padding (auto_padding), so the automatic check can be corrected and then run on
    every picture of every session without anyone.

Known sizes (KNOWN): the Alvik's top outline and, once printed, the dock's marker strip.
"""
import math

import numpy as np

# The Alvik's top, as the overhead camera sees it: the shell's top edge (the widest part: its sides lean out going up).
# hardware/dock/dock_params.py: ROBOT_WIDTH_MM 96.0 ([STEP] across the shell's top edge; [TS] 96); length at the top
# edge 96.0 ([STEP]; the tape on Nick's robot read 95.25 = 3 3/4 in, to the tape's 1/16 in); ROBOT_TOP_Z_MM 37.2.
ROBOT_TOP_W_CM = 9.60
ROBOT_TOP_L_CM = 9.60
ROBOT_TOP_Z_CM = 3.72
# The dock's marker strip (hardware/dock/build_dock_v5.py STRIP_LEN_MM x STRIP_DEP_MM, printed size), lying in the
# base's recess, its top at the base's height (WALL_H_MM 22). Measure the print with calipers and pass the measured
# length (mark_robot.py --strip-mm): prints come out 0.1-0.3 % off.
STRIP_LONG_CM = 9.00
STRIP_SHORT_CM = 3.00
STRIP_Z_CM = 2.20

KNOWN = {
    "robot": dict(label="the robot's top", long_cm=ROBOT_TOP_L_CM, short_cm=ROBOT_TOP_W_CM, z_cm=ROBOT_TOP_Z_CM,
                  source="hardware/dock/dock_params.py (Arduino's CAD: 96.0 x 96.0 mm at the top edge, 37.2 mm up)"),
    "dock_strip": dict(label="the dock's marker strip", long_cm=STRIP_LONG_CM, short_cm=STRIP_SHORT_CM, z_cm=STRIP_Z_CM,
                       source="hardware/dock/build_dock_v5.py (printed 90.00 x 30.00 mm, 22 mm up)"),
}

LIMIT_PCT = 2.0     # chosen: a scale error this size moves a robot 61 cm from the board's middle by 1.2 cm, a quarter
                    # of the tracker's 5 cm "wrong" limit. A tolerance taken from that budget, not a number a DOE sets
                    # (the camera error map, e02_camerr, is where the budget itself would be measured)
AXIS_COS = math.cos(math.radians(30))   # a side within 30 deg of a board axis counts for that axis's scale
AUTO_AREA = (0.5, 2.0)  # chosen: an automatic blob this many times the predicted area (or outside) is not the object
AUTO_WIN = 1.7          # the automatic outline looks in a square this many object sizes across


def known(obj, strip_mm=None):
    k = dict(KNOWN[obj])
    if obj == "dock_strip" and strip_mm:
        k["long_cm"] = float(strip_mm) / 10.0
        k["source"] += "; long side measured %.2f mm" % float(strip_mm)
    return k


def board_regions(board=None):
    """The board split 3 x 3 (x0, y0, x1, y1 in floor cm)."""
    x0, y0, x1, y1 = (board.x0, board.y0, board.x1, board.y1) if board is not None else (0.0, 0.0, 122.15, 121.9)
    return x0, y0, x1, y1


def region_of(x, y, board=None):
    """(column, row) 0..2 of a floor point on the 3 x 3 grid; points off the board go to the nearest cell."""
    x0, y0, x1, y1 = board_regions(board)
    c = int(min(2, max(0, math.floor(3 * (x - x0) / (x1 - x0)))))
    r = int(min(2, max(0, math.floor(3 * (y - y0) / (y1 - y0)))))
    return c, r


def to_floor(cam, u, v, z):
    x, y = cam.to_floor(float(u), float(v), float(z))[:2]
    return (float(x), float(y)) if np.isfinite(x) and np.isfinite(y) else None


def order_corners(pts):
    """Four points in order around their centre (so sides 0-1, 1-2, 2-3, 3-0 are the outline's sides)."""
    p = np.asarray(pts, float).reshape(4, 2)
    c = p.mean(axis=0)
    a = np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0])
    return p[np.argsort(a)]


def predicted_corners(cam, x, y, obj="robot", heading_deg=0.0, strip_mm=None):
    """Where the calibration says the object's top corners are in the picture, if it stands at floor (x, y) turned
    heading_deg: the start of a hand fit (a perfect calibration: the handles already sit on the corners)."""
    k = known(obj, strip_mm)
    h = math.radians(heading_deg)
    ca, sa = math.cos(h), math.sin(h)
    out = []
    for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        px, py = dx * k["long_cm"] / 2, dy * k["short_cm"] / 2
        u, v = cam.project((x + px * ca - py * sa, y + px * sa + py * ca, k["z_cm"]))[:2]
        out.append([float(u), float(v)])
    return out


def measure(cam, corners_px, obj="robot", strip_mm=None, board=None):
    """The object's outline (four corners in pixels) measured in floor cm through the calibration, against its real
    size. Returns dict(x, y (its centre, cm), sides_cm, long_cm, short_cm, scale_pct (all four sides together: +1.0 =
    1 % bigger than real), scale_x_pct / scale_y_pct (from the sides that run along the board's x / y; None when no
    side does), corner_dev_deg (how far the corners are from square: a sloppy fit or a wrong tilt), region (col,
    row)) or None when a corner falls outside the calibration."""
    k = known(obj, strip_mm)
    p = order_corners(corners_px)
    f = [to_floor(cam, u, v, k["z_cm"]) for u, v in p]
    if any(q is None for q in f):
        return None
    f = np.array(f)
    sides = [float(np.linalg.norm(f[(i + 1) % 4] - f[i])) for i in range(4)]
    pair_a, pair_b = (sides[0] + sides[2]) / 2, (sides[1] + sides[3]) / 2
    long_pair = 0 if pair_a >= pair_b else 1
    real = [k["long_cm"] if (i % 2) == long_pair else k["short_cm"] for i in range(4)]
    ratios = [s / r for s, r in zip(sides, real)]
    ax = {"x": [], "y": []}
    for i in range(4):
        d = f[(i + 1) % 4] - f[i]
        n = float(np.linalg.norm(d))
        if n <= 0:
            continue
        cx, cy = abs(d[0]) / n, abs(d[1]) / n
        if cx >= AXIS_COS:
            ax["x"].append(ratios[i])
        elif cy >= AXIS_COS:
            ax["y"].append(ratios[i])
    devs = []
    for i in range(4):
        a, b = f[i - 1] - f[i], f[(i + 1) % 4] - f[i]
        cosang = float(np.dot(a, b) / max(1e-9, np.linalg.norm(a) * np.linalg.norm(b)))
        devs.append(abs(math.degrees(math.acos(max(-1.0, min(1.0, cosang)))) - 90.0))
    cx, cy = float(f[:, 0].mean()), float(f[:, 1].mean())

    def pct(v):
        return None if v is None else round(100.0 * (v - 1.0), 2)
    return dict(obj=obj, x=round(cx, 2), y=round(cy, 2), sides_cm=[round(s, 3) for s in sides],
                long_cm=round(max(pair_a, pair_b), 3), short_cm=round(min(pair_a, pair_b), 3),
                real_long_cm=k["long_cm"], real_short_cm=k["short_cm"],
                scale_pct=pct(float(np.mean(ratios))),
                scale_x_pct=pct(float(np.mean(ax["x"]))) if ax["x"] else None,
                scale_y_pct=pct(float(np.mean(ax["y"]))) if ax["y"] else None,
                corner_dev_deg=round(float(np.mean(devs)), 2), region=list(region_of(cx, cy, board)),
                corners_px=[[round(float(u), 2), round(float(v), 2)] for u, v in p])


def auto_outline(gray, cam, x, y, obj="robot", strip_mm=None, board=None):
    """The object's outline found by itself near floor (x, y): in a window AUTO_WIN object sizes across, the picture
    split in two by brightness (Otsu), the part under the centre kept (bright or dark, whichever the object is),
    and the rectangle with that blob's area and spread (sub-pixel, from its brightness-weighted moments). A square
    object's heading is not defined by its moments: the rectangle is then turned arbitrarily, which changes
    nothing in its size. Returns measure()'s dict (plus area_ratio: the blob's area against the
    predicted one) or None (nothing object-sized there)."""
    import cv2
    k = known(obj, strip_mm)
    if gray is None:
        return None
    u, v = cam.project((float(x), float(y), k["z_cm"]))[:2]
    if not (np.isfinite(u) and np.isfinite(v)):
        return None
    ppc = float(cam.px_per_cm(float(x), float(y), k["z_cm"]))
    if not np.isfinite(ppc) or ppc <= 0:
        return None
    half = int(math.ceil(AUTO_WIN * k["long_cm"] * ppc / 2)) + 2
    H, W = gray.shape[:2]
    u0, v0 = int(round(u)) - half, int(round(v)) - half
    a0, b0 = max(0, u0), max(0, v0)
    a1, b1 = min(W, u0 + 2 * half + 1), min(H, v0 + 2 * half + 1)
    if a1 - a0 < 8 or b1 - b0 < 8:
        return None
    raw = gray[b0:b1, a0:a1].astype(float)
    crop = cv2.GaussianBlur(gray[b0:b1, a0:a1], (3, 3), 0)     # blurred for the split only (blur adds spread)
    _, th = cv2.threshold(crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cu, cv_ = int(round(u)) - a0, int(round(v)) - b0
    cu, cv_ = min(max(cu, 0), crop.shape[1] - 1), min(max(cv_, 0), crop.shape[0] - 1)
    mask = th if th[cv_, cu] > 0 else 255 - th
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab = cv2.connectedComponents(mask)
    c = lab[cv_, cu]
    if c == 0:
        return None
    comp = (lab == c).astype(np.uint8)
    if comp.sum() < 10:
        return None
    # Sub-pixel outline from the blob's moments (a pixel's minimum rectangle is padded by the staircase edge):
    # each pixel weighted by how far its brightness is from the floor's towards the object's, near the blob only;
    # a uniform rectangle's spread along its axes is length^2 / 12.
    near = cv2.dilate(comp, np.ones((5, 5), np.uint8)).astype(bool)
    obj_lvl = float(np.median(raw[cv2.erode(comp, np.ones((3, 3), np.uint8)).astype(bool) | (comp.sum() < 30
                                                                                         and comp.astype(bool))]))
    floor_px = raw[~near]
    if floor_px.size < 10:
        return None
    floor_lvl = float(np.median(floor_px))
    if abs(obj_lvl - floor_lvl) < 8:
        return None
    w = np.clip((raw - floor_lvl) / (obj_lvl - floor_lvl), -1.0, 2.0) * near   # not cut at 0: floor noise cancels
    m = w.sum()
    if m < 10:
        return None
    yy, xx = np.mgrid[0:crop.shape[0], 0:crop.shape[1]]
    mx, my = float((w * xx).sum() / m), float((w * yy).sum() / m)
    cxx = float((w * (xx - mx) ** 2).sum() / m)
    cyy = float((w * (yy - my) ** 2).sum() / m)
    cxy = float((w * (xx - mx) * (yy - my)).sum() / m)
    ev, evec = np.linalg.eigh(np.array([[cxx, cxy], [cxy, cyy]]))
    L1, L2 = math.sqrt(12 * max(ev[1], 0.0)), math.sqrt(12 * max(ev[0], 0.0))
    pred_area = k["long_cm"] * k["short_cm"] * ppc * ppc
    ratio = float(m) / pred_area if pred_area > 0 else 0.0
    if not (AUTO_AREA[0] <= ratio <= AUTO_AREA[1]):
        return None
    e1, e2 = evec[:, 1], evec[:, 0]
    cen = np.array([mx + a0, my + b0])
    box = np.array([cen + sa * L1 / 2 * e1 + sb * L2 / 2 * e2 for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    out = measure(cam, box.tolist(), obj, strip_mm, board)
    if out is not None:
        out["area_ratio"] = round(ratio, 3)
    return out


def _stats(vals):
    v = np.array([q for q in vals if q is not None], float)
    if len(v) == 0:
        return None
    out = dict(n=int(len(v)), median_pct=round(float(np.median(v)), 2), mean_pct=round(float(v.mean()), 2))
    if len(v) >= 2:
        sd = float(v.std(ddof=1))
        from math import sqrt
        try:
            from scipy import stats as st
            tq = float(st.t.ppf(0.975, len(v) - 1))
        except Exception:       # scipy is not on the laptop: the t value from a small table
            tq = {1: 12.71, 2: 4.30, 3: 3.18, 4: 2.78, 5: 2.57, 6: 2.45, 7: 2.36, 8: 2.31, 9: 2.26}.get(len(v) - 1, 2.0 + 2.4 / len(v))
        half = tq * sd / sqrt(len(v))
        out.update(sd_pct=round(sd, 2), ci95_pct=[round(float(v.mean()) - half, 2), round(float(v.mean()) + half, 2)])
    return out


def verdict(st, limit=LIMIT_PCT):
    """'ok' (the 95% interval of the mean is inside +-limit), 'off' (entirely outside), else 'unsure'."""
    if not st or "ci95_pct" not in st:
        return "unsure"
    lo, hi = st["ci95_pct"]
    if -limit <= lo and hi <= limit:
        return "ok"
    if lo > limit or hi < -limit:
        return "off"
    return "unsure"


def summarize(fits, board=None, limit=LIMIT_PCT):
    """What a set of fits (measure()'s dicts) says about the calibration: overall, per 3 x 3 region, per axis, and
    the trend across the board (a scale error that grows from one side to the other: the tilt is off)."""
    fits = [f for f in fits if f and f.get("scale_pct") is not None]
    out = dict(n=len(fits), limit_pct=limit, overall=_stats([f["scale_pct"] for f in fits]))
    out["verdict"] = verdict(out["overall"], limit)
    regions = []
    for r in range(3):
        for c in range(3):
            st = _stats([f["scale_pct"] for f in fits if list(f["region"]) == [c, r]])
            regions.append(dict(col=c, row=r, **(st or dict(n=0))))
    out["regions"] = regions
    out["axis_x"] = _stats([f.get("scale_x_pct") for f in fits])
    out["axis_y"] = _stats([f.get("scale_y_pct") for f in fits])
    out["trend"] = None
    if len(fits) >= 6:
        x0, y0, x1, y1 = board_regions(board)
        X = np.array([[1.0, (f["x"] - (x0 + x1) / 2) / (x1 - x0), (f["y"] - (y0 + y1) / 2) / (y1 - y0)] for f in fits])
        yv = np.array([f["scale_pct"] for f in fits])
        if np.linalg.matrix_rank(X) == 3 and np.ptp(X[:, 1]) > 0.3 and np.ptp(X[:, 2]) > 0.3:
            b = np.linalg.lstsq(X, yv, rcond=None)[0]
            out["trend"] = dict(across_x_pct=round(float(b[1]), 2), across_y_pct=round(float(b[2]), 2),
                                note="how much the scale error changes from one side of the board to the other")
    out["spread"] = len(set(tuple(f["region"]) for f in fits))      # regions with at least one fit
    return out


def auto_padding(pairs):
    """Pictures with both a hand fit and an automatic one: [(hand, auto)]. The median of auto - hand scale (pct):
    how much bigger the automatic outline is (shadow, glow). None with fewer than 3 pairs."""
    d = [a["scale_pct"] - h["scale_pct"] for h, a in pairs if h and a]
    if len(d) < 3:
        return None
    return dict(n=len(d), median_pct=round(float(np.median(d)), 2), sd_pct=round(float(np.std(d)), 2))


def plain(summary, what="the object"):
    """One plain sentence about a summary, for the page and the report (what: the thing measured)."""
    s = summary
    if not s["n"]:
        return "No size fits yet."
    o = s["overall"]
    lo_hi = (" (95%% range %+.1f to %+.1f %%)" % tuple(o["ci95_pct"])) if "ci95_pct" in o else ""
    head = "Through the calibration %s measures %+.1f %% against its real size over %d fit%s%s" % (
        what, o["mean_pct"], s["n"], "" if s["n"] == 1 else "s", lo_hi)
    tail = {"ok": ": the scale is right to within %.0f %%." % s["limit_pct"],
            "off": ": the scale is off by more than %.0f %%." % s["limit_pct"],
            "unsure": ": not enough fits yet to say whether it is within %.0f %%." % s["limit_pct"]}[s["verdict"]]
    return head + tail
