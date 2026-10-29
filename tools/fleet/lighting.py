#!/usr/bin/env python3
"""Lighting check before a run (MIRLT, 2026-10-28): does each camera see the robots against the board?

Why. At 16:29 on 28 Oct the overhead camera found the robots' bodies only 3 times in 29 (daylight from the window);
the robots' tops stood 20-25 grey levels from the board (31-34 the night before, under the lamp), and every robot
standing where it was when tracking began, on floor the tracker had to fill in (margin 13-14), was seen only in
pieces. Nick: say so in plain words BEFORE anything moves, rather than failing later.

What it measures, per camera, from one picture (no tracking needed):
  - the board: its median grey and the share of it that clips (250+): a clipping board is washed out;
  - each robot's contrast: the median difference between the robot and the board right round it (the board there is
    the picture median-filtered over 2.5 robot widths, so a robot does not count as board), on the robot's middle
    0.45 of a width. Where the robots are: given (L0: their lights), or found as robot-sized patches that stand out
    (Mission Control's Setup screen, before L0);
  - found: a robot counts as found when at least FOUND_SHARE of its middle stands out by more than the tracker's
    threshold (16): enough for an outline.
Verdicts: "ok", "low" (it works, with little margin) or "washed out" (said in plain words, with what to do).

    python3 tools/fleet/lighting.py <picture.jpg> <camera calibration .json> [--spot x,y ...]
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (os.path.join(TOOLS, "experiment"),):
    if p not in sys.path:
        sys.path.insert(0, p)

import e02_camera as CAM  # noqa: E402

THR = 16.0              # the tracker's threshold (MotionTracker thr_min): a pixel this far from the floor counts
FOUND_SHARE = 0.3       # chosen: a robot is "found" when 30 % of its middle passes THR (a robot's outline must cover at
                        # least 0.3 of its area to be robot-shaped, MotionTracker._shape_ok). No DOE: follows from it
WASHED_CONTRAST = 18.0  # chosen from the 27-28 Oct replays, this module's measure on the overhead (median per robot)
LOW_CONTRAST = 26.0     # against how often the overhead saw that robot's body: 28 Oct daylight: Center 12 -> 60 %,
                        # Right 19 -> 71 %, Left 28 -> 99.7 %; 27 Oct lamp: Center 23.5 -> 84 %, Right 28 -> 97 %, Left
                        # 37 -> 99.6 %. Under WASHED_CONTRAST: "washed out"; under LOW_CONTRAST: "low" (27 Oct's Center
                        # was already there). DOE note: on the board, lamp / daylight / both, contrast against the
                        # overhead's body seen-rate at L0 (the check's own log, lighting.json, is the data)
CLIP_SHARE = 0.02       # a board with more than 2 % at 250+ is washed out (camrec.WashoutGuard.SAT_SHARE)

ADVICE = {"overhead": "close the blinds or turn off the lamp above the table",
          "side": "close the blinds, or move the lamp so it does not shine across the table"}


def board_poly(cam, rect, z=0.0, n=8):
    x0, y0, x1, y1 = rect
    pts = []
    for k in range(n):
        pts.append((x0 + (x1 - x0) * k / n, y0))
    for k in range(n):
        pts.append((x1, y0 + (y1 - y0) * k / n))
    for k in range(n):
        pts.append((x1 - (x1 - x0) * k / n, y1))
    for k in range(n):
        pts.append((x0, y1 - (y1 - y0) * k / n))
    return np.array([cam.project((x, y, z)) for x, y in pts], np.float32)


def measure(img, cam, rect=(0.0, 0.0, CAM.BOARD_W_CM, CAM.BOARD_L_CM), spots=None, z=None, name="overhead",
            n_robots=None):
    """dict(camera, board, clipped, robots=[dict(x, y, contrast, found)], found, expected, min_contrast,
    verdict, words). spots: [(x, y)] floor cm where the robots are (their middles), or None: robot-sized patches
    that stand out are searched for (n_robots: how many there should be)."""
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    g = g.astype(np.float32)
    H, W = g.shape
    z = CAM.ROBOT_TOP_CM if z is None else z
    cx, cy = (rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0
    rpx = CAM.ROBOT_CM * cam.px_per_cm(cx, cy, z)
    mask = np.zeros((H, W), np.uint8)
    poly = board_poly(cam, rect)
    if np.all(np.isfinite(poly)):
        cv2.fillPoly(mask, [np.round(poly).astype(np.int32)], 1)
    if mask.sum() < 100:
        mask[:] = 1
    board_px = g[mask > 0]
    # the board round each spot: a median over 2.5 robot widths (on a half-size picture: fast on the Pi)
    k = int(2.5 * rpx / 2) | 1
    small = cv2.resize(g.astype(np.uint8), (W // 2, H // 2), interpolation=cv2.INTER_AREA)
    local = cv2.resize(cv2.medianBlur(small, max(3, min(k, 255))), (W, H), interpolation=cv2.INTER_LINEAR)
    d = np.abs(g - local.astype(np.float32)) * (mask > 0)
    robots = []
    if spots is None:
        m = ((d > THR) * 255).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        n, lab, st, cents = cv2.connectedComponentsWithStats(m, connectivity=8)
        cand = []
        for i in range(1, n):
            a = st[i, cv2.CC_STAT_AREA]
            w, h = st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT]
            if 0.3 * rpx * rpx <= a <= 2.6 * rpx * rpx and max(w, h) <= 2.0 * rpx:
                x, y = cam.to_floor(cents[i][0], cents[i][1], z)
                cand.append((a, float(x), float(y)))
        cand.sort(reverse=True)
        spots = [(x, y) for _, x, y in cand[:n_robots or 3]]
    yy, xx = np.mgrid[:H, :W]
    for x, y in spots:
        u, v = cam.project((x, y, z))
        if not (np.isfinite(u) and np.isfinite(v)):
            continue
        r = 0.45 * CAM.ROBOT_CM * cam.px_per_cm(x, y, z)
        x0, x1 = int(max(0, u - r - 1)), int(min(W, u + r + 2))
        y0, y1 = int(max(0, v - r - 1)), int(min(H, v + r + 2))
        if x1 <= x0 or y1 <= y0:
            continue
        disk = np.hypot(xx[y0:y1, x0:x1] - u, yy[y0:y1, x0:x1] - v) <= r
        dd = d[y0:y1, x0:x1][disk]
        if dd.size == 0:
            continue
        robots.append(dict(x=round(x, 1), y=round(y, 1), contrast=round(float(np.median(dd)), 1),
                           found=bool((dd > THR).mean() >= FOUND_SHARE)))
    expected = n_robots if n_robots is not None else len(spots)
    found = sum(r["found"] for r in robots)
    clipped = float((board_px >= 250).mean())
    minc = min([r["contrast"] for r in robots], default=None)
    if clipped > CLIP_SHARE:
        verdict = "washed out"
        words = "%s camera washed out: %.0f %% of the board is white (clipped): %s" % (
            name, 100 * clipped, ADVICE.get(name, ADVICE["overhead"]))
    elif expected and (minc is None or minc < WASHED_CONTRAST):
        # the label follows the stated thresholds (29 Oct): washed out only under WASHED_CONTRAST (or nothing stands
        # out at all). Before, a robot not found also said "washed out", so 22 grey levels was labelled washed out
        verdict = "washed out"
        words = ("%s camera washed out: no robot stands out from the board (%d of %d found): %s" % (
            name, found, expected, ADVICE.get(name, ADVICE["overhead"]))) if minc is None else (
            "%s camera washed out: the robots stand only %.0f grey levels from the board (it needs %.0f+), %d of %d "
            "found: %s" % (name, minc, WASHED_CONTRAST, found, expected, ADVICE.get(name, ADVICE["overhead"])))
    elif minc is not None and (minc < LOW_CONTRAST or found < expected):
        verdict = "low"
        words = "%s camera: the robots stand %.0f grey levels from the board (fine from %.0f, washed out under %.0f)" \
                % (name, minc, LOW_CONTRAST, WASHED_CONTRAST)
        if found < expected:
            words += ", but only %d of %d robots found: check every robot is in this camera's picture" % (found,
                                                                                                         expected)
        if minc < LOW_CONTRAST:
            words += ": it works, with little margin; less light on the board would help"
    else:
        verdict = "ok"
        words = "%s camera: fine (robots %s grey levels from the board, %d of %d found)" % (
            name, "?" if minc is None else "%.0f" % minc, found, expected)
    return dict(camera=name, board=round(float(np.median(board_px)), 1), clipped=round(clipped, 4), robots=robots,
                found=found, expected=expected, min_contrast=minc, verdict=verdict, words=words)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("picture")
    ap.add_argument("calib")
    ap.add_argument("--name", default="overhead")
    ap.add_argument("--spot", action="append", default=[])
    ap.add_argument("--robots", type=int, default=None)
    a = ap.parse_args()
    cam = CAM.load_camera(a.calib)
    spots = [tuple(float(v) for v in s.split(",")) for s in a.spot] or None
    z = CAM.ROBOT_TOP_CM if a.name == "overhead" else 0.5 * CAM.ROBOT_TOP_CM
    r = measure(cv2.imread(a.picture), cam, spots=spots, z=z, name=a.name, n_robots=a.robots)
    print(r["words"])
    for q in r["robots"]:
        print("  robot at (%.1f, %.1f): %.1f grey levels, %s" % (q["x"], q["y"], q["contrast"],
                                                                  "found" if q["found"] else "NOT found"))


if __name__ == "__main__":
    main()
