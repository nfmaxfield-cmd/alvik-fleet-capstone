#!/usr/bin/env python3
"""The checkerboard corner piece (Nick, 2026-10-27 09:56-09:59: "a small checkerboard piece ... just one in the corner so
the two cameras know how to align the image"; one system build 2026-10-28, item 5).

One printed ChArUco board (letter size, squares with markers, so it has a direction) lies flat in the table's 0,0
corner, its edges along the table's sides, while both cameras take a picture. It is the shared anchor: 0,0, the
axes and the handedness, the same in both cameras. With each camera's lens (its intrinsics, from the calibration
it already has) its pose follows from the board's corners (solvePnP), and the table's 4 corners (known table size)
add points across the whole table, so the far end is not an extrapolation. Out: each camera's pose and lens height,
a floor calibration file per camera (the studio form e02_camera.load_camera reads), and how well the two cameras
agree on the table's corners.

    python3 tools/pi/charuco_calib.py print --out board.png          # the board to print (letter, 300 dpi)
    python3 tools/pi/charuco_calib.py solve --image over.png --intrinsics overhead_floorcal.json \\
        --table 122x61 [--corners u,v;u,v;u,v;u,v] --out overhead_charuco.json

The camera setup page (camera_setup.py) runs solve() on both cameras' pictures (part 3, "checkerboard").
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2  # noqa: E402

SQUARES = (5, 7)        # chosen: 5 x 7 squares on a letter sheet; Nick's rule of thumb: the far camera sees about 5 x 5
                        # squares clearly. DOE note (Nick's own, brief 09:59): 3 board sizes against pose accuracy (the
                        # two cameras' disagreement on the table's corners)
SQUARE_CM = 3.5         # 5 x 3.5 = 17.5 cm by 7 x 3.5 = 24.5 cm: fits 21.6 x 27.9 with a margin. Same DOE as SQUARES
MARKER_CM = 2.6         # the marker inside each white square (about 3/4 of it). No DOE: ArUco's usual ratio
DICT = cv2.aruco.DICT_4X4_50
BOARD_OFFSET_CM = (0.0, 0.0)   # the print's bottom-left corner at the table's 0,0 corner, its short edge along the
                               # table's x side. Nick lines it up by hand: the offset is measured if it cannot sit in the corner
# The far camera's pictures are small (640 x 480 over ~190 cm of floor: ~3.4 px/cm), so a marker's bits are 2-3 px
# across on a letter sheet: ArUco markers do not read at that size (simulated overhead at 4.2 px/cm: none found). A
# plain chessboard's corners do (findChessboardCornersSB works down to ~8 px squares). So the corner piece is a plain
# chessboard; its direction comes from where it lies: it marks the table's 0,0 corner, and the table's long side is x.
CHECK_SQUARES = (5, 7)  # chosen: 5 x 7 squares (4 x 6 inner corners, letter size at 3.8 cm). DOE note: as SQUARES
CHECK_CM = 3.8          # 5 x 3.8 = 19 cm, 7 x 3.8 = 26.6 cm on a 21.6 x 27.9 sheet
MIN_CORNERS = 6         # a picture with fewer board corners found is not used. No DOE: solvePnP needs a few points


def board(squares=SQUARES, square_cm=SQUARE_CM, marker_cm=MARKER_CM):
    d = cv2.aruco.getPredefinedDictionary(DICT)
    return cv2.aruco.CharucoBoard(squares, square_cm, marker_cm, d)


def print_png(path, dpi=300, margin_cm=1.5):
    b = board()
    w_cm, h_cm = SQUARES[0] * SQUARE_CM + 2 * margin_cm, SQUARES[1] * SQUARE_CM + 2 * margin_cm
    px = lambda cm: int(round(cm / 2.54 * dpi))      # noqa: E731
    img = b.generateImage((px(w_cm), px(h_cm)), marginSize=px(margin_cm))
    cv2.putText(img, "MIRLT 0,0 corner: put THIS corner in the table's 0,0 corner  (%dx%d, %.1f cm squares)"
                % (SQUARES[0], SQUARES[1], SQUARE_CM), (px(0.3), px(0.8)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
    cv2.imwrite(path, img)
    return path


def floor_of(ids, b=None, offset=BOARD_OFFSET_CM):
    """Floor (x, y, 0) of each ChArUco corner id: the board's own corner positions (cm, its origin at its outer
    corner) moved to the table's 0,0 corner."""
    b = b or board()
    pts = b.getChessboardCorners()          # (N, 3), board units (cm): x across the print, y DOWN the print
    h = SQUARES[1] * SQUARE_CM
    # lying face up with the print's bottom-left corner in the table's 0,0 corner and its bottom edge along x: the
    # print's "down" is the floor's -y (seen from above, x right and y up, as the map)
    return np.array([[pts[i][0] + offset[0], h - pts[i][1] + offset[1], 0.0] for i in np.ravel(ids)], np.float64)


def checker_png(path, dpi=300, margin_cm=0.5):
    px = lambda cm: int(round(cm / 2.54 * dpi))      # noqa: E731
    nx, ny = CHECK_SQUARES
    img = np.full((px(ny * CHECK_CM + 2 * margin_cm), px(nx * CHECK_CM + 2 * margin_cm)), 255, np.uint8)
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                x0, y0 = px(margin_cm + i * CHECK_CM), px(margin_cm + j * CHECK_CM)
                img[y0:px(margin_cm + (j + 1) * CHECK_CM), x0:px(margin_cm + (i + 1) * CHECK_CM)] = 0
    cv2.imwrite(path, img)
    return path


def render_checker(model, size=None, base=None, px_per_cm=20):
    """The chessboard lying in the 0,0 corner as camera `model` sees it (tests, fake cameras)."""
    nx, ny = CHECK_SQUARES
    tex = np.full((int(ny * CHECK_CM * px_per_cm), int(nx * CHECK_CM * px_per_cm)), 255, np.uint8)
    q = CHECK_CM * px_per_cm
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                tex[int(j * q):int((j + 1) * q), int(i * q):int((i + 1) * q)] = 0
    w_cm, h_cm = nx * CHECK_CM, ny * CHECK_CM
    src = np.float32([[0, 0], [tex.shape[1], 0], [tex.shape[1], tex.shape[0]], [0, tex.shape[0]]])
    floor = [(0, h_cm), (w_cm, h_cm), (w_cm, 0), (0, 0)]
    dst = np.float32([model.project((x, y, 0.0)) for x, y in floor])
    Hm = cv2.getPerspectiveTransform(src, dst)
    W, H = size or model.image_size
    out = base.copy() if base is not None else np.full((H, W), 170, np.uint8)
    warped = cv2.warpPerspective(tex, Hm, (W, H), flags=cv2.INTER_AREA, borderValue=0)
    mask = cv2.warpPerspective(np.full(tex.shape, 255, np.uint8), Hm, (W, H), flags=cv2.INTER_NEAREST)
    out[mask > 0] = warped[mask > 0]
    return out


def detect_checker(img):
    """Inner corners (N, 2) of the chessboard (4 x 6), sub-pixel, in OpenCV's order; None if not found."""
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    pat = (CHECK_SQUARES[0] - 1, CHECK_SQUARES[1] - 1)
    for p in (pat, pat[::-1]):
        ok, c = cv2.findChessboardCornersSB(g, p, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            return c.reshape(-1, 2).astype(np.float64), p
    return None, None


def solve_checker(img, K, dist, image_size, table, table_corners_px, name="overhead"):
    """The camera's pose from the chessboard in the 0,0 corner plus the table's 4 corners (in the picture, in loop
    order, any start, any direction). Finds which corner is 0,0 (the one under the board) and which way x runs (the
    table's long side: the labelling whose floor-to-picture mapping puts the board's corners on CHECK_CM squares).
    Returns as solve(), plus "corners_px": the table's corners in floor order (0,0 / L,0 / L,W / 0,W)."""
    from cameras import CameraModel
    L, W = table
    c, pat = detect_checker(img)
    if c is None:
        return {"ok": False, "why": "the chessboard was not found: is all of it in the picture, flat, evenly lit?"}
    loop = [tuple(map(float, q)) for q in table_corners_px]
    cen = c.mean(0)
    k0 = int(np.argmin([math.hypot(u - cen[0], v - cen[1]) for u, v in loop]))
    best = None
    for d in (1, -1):
        order = [loop[(k0 + d * k) % 4] for k in range(4)]
        Hm = cv2.getPerspectiveTransform(np.float32([[0, 0], [L, 0], [L, W], [0, W]]), np.float32(order))
        fl = cv2.perspectiveTransform(c.reshape(-1, 1, 2), np.linalg.inv(Hm)).reshape(-1, 2)
        # the board's corners on the floor under this labelling: on a grid of CHECK_CM from the corner?
        g = fl / CHECK_CM
        gi = np.round(g)
        # the board's short side lies along x: inner corners 1..4 along x and 1..6 along y (this also settles a
        # square table, where the long-side rule cannot)
        shape_ok = gi[:, 0].min() == 1 and gi[:, 0].max() == CHECK_SQUARES[0] - 1 and gi[:, 1].min() == 1 and \
            gi[:, 1].max() == CHECK_SQUARES[1] - 1
        err = float(np.median(np.abs(g - gi))) + (0.0 if shape_ok else 1.0)
        if best is None or err < best[0]:
            best = (err, order, fl)
    err, order, fl = best
    grid = np.round(fl / CHECK_CM)
    obj = np.column_stack([grid * CHECK_CM, np.zeros(len(grid))])
    if err > 0.2:
        return {"ok": False, "why": "the chessboard is not lying in a table corner square to its sides (grid error %.2f)"
                % err}
    tc = np.array([[0, 0, 0], [L, 0, 0], [L, W, 0], [0, W, 0]], np.float64)
    objp = np.vstack([obj, tc])
    imgp = np.vstack([c, np.array(order, np.float64)])
    ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return {"ok": False, "why": "no camera pose fits these points"}
    rvec, tvec = cv2.solvePnPRefineLM(objp, imgp, K, dist, rvec, tvec)
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    e = np.linalg.norm(proj.reshape(-1, 2) - imgp, axis=1)
    R, _ = cv2.Rodrigues(rvec)
    C = (-R.T @ tvec).ravel()
    model = CameraModel(K, dist, rvec.ravel(), tvec.ravel(), image_size, name=name)
    n = len(c)
    return {"ok": True, "model": model.to_dict(), "height_cm": round(float(C[2]), 1),
            "nadir_cm": [round(float(C[0]), 1), round(float(C[1]), 1)], "board_corners": n,
            "board_err_px": round(float(np.median(e[:n])), 2), "table_err_px": round(float(np.max(e[n:])), 2),
            "corners_px": [list(map(float, q)) for q in order]}


def detect(img, b=None):
    """(ids (N,), image points (N, 2)) of the ChArUco corners found, or (None, None)."""
    b = b or board()
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    det = cv2.aruco.CharucoDetector(b)
    cc, ids, _, _ = det.detectBoard(g)
    if ids is None or len(ids) < MIN_CORNERS:
        return None, None
    return np.ravel(ids).astype(int), cc.reshape(-1, 2).astype(np.float64)


def intrinsics(path):
    """K, dist, image size from a camera file the camera already has (studio form or floor calibration)."""
    import e02_camera as CAM
    cam = CAM.load_camera(path)
    return np.array(cam.K, np.float64), np.array(cam.dist, np.float64).ravel(), tuple(cam.image_size)


def solve(img, K, dist, image_size, table=None, table_corners_px=None, b=None, name="overhead"):
    """The camera's pose from the board's corners (and the table's 4 corners when given: table = (L, W) cm,
    table_corners_px = [(u, v)] for floor (0,0), (L,0), (L,W), (0,W)). Returns a dict: model (studio camera dict),
    height_cm, nadir_cm, reprojection error (px) on the board and on the table's corners, points used."""
    from cameras import CameraModel
    ids, px = detect(img, b)
    if ids is None:
        return {"ok": False, "why": "the board was not found (fewer than %d corners): is it in the picture, flat, lit?"
                % MIN_CORNERS}
    obj = floor_of(ids, b)
    imgp = px.copy()
    if table is not None and table_corners_px is not None:
        L, W = table
        tc = np.array([[0, 0, 0], [L, 0, 0], [L, W, 0], [0, W, 0]], np.float64)
        obj = np.vstack([obj, tc])
        imgp = np.vstack([imgp, np.array(table_corners_px, np.float64)])
    ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return {"ok": False, "why": "no camera pose fits these points"}
    rvec, tvec = cv2.solvePnPRefineLM(obj, imgp, K, dist, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - imgp, axis=1)
    nb = len(ids)
    R, _ = cv2.Rodrigues(rvec)
    C = (-R.T @ tvec).ravel()
    model = CameraModel(K, dist, rvec.ravel(), tvec.ravel(), image_size, name=name)
    return {"ok": True, "model": model.to_dict(), "height_cm": round(float(C[2]), 1),
            "nadir_cm": [round(float(C[0]), 1), round(float(C[1]), 1)], "board_corners": nb,
            "board_err_px": round(float(np.median(err[:nb])), 2),
            "table_err_px": round(float(np.max(err[nb:])), 2) if len(err) > nb else None}


def render_board(model, b=None, size=None, base=None, offset=BOARD_OFFSET_CM, px_per_cm=20):
    """A picture of the board lying in the 0,0 corner as camera `model` (CameraModel) sees it (tests, the setup page's
    fake cameras)."""
    b = b or board()
    w_cm, h_cm = SQUARES[0] * SQUARE_CM, SQUARES[1] * SQUARE_CM
    tex = b.generateImage((int(w_cm * px_per_cm), int(h_cm * px_per_cm)), marginSize=0)
    src = np.float32([[0, 0], [tex.shape[1], 0], [tex.shape[1], tex.shape[0]], [0, tex.shape[0]]])
    floor = [(offset[0], offset[1] + h_cm), (offset[0] + w_cm, offset[1] + h_cm), (offset[0] + w_cm, offset[1]),
             (offset[0], offset[1])]           # the print's top-left, top-right, bottom-right, bottom-left
    dst = np.float32([model.project((x, y, 0.0)) for x, y in floor])
    Hm = cv2.getPerspectiveTransform(src, dst)
    W, H = size or model.image_size
    out = base.copy() if base is not None else np.full((H, W), 170, np.uint8)
    warped = cv2.warpPerspective(tex, Hm, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)
    mask = cv2.warpPerspective(np.full(tex.shape, 255, np.uint8), Hm, (W, H), flags=cv2.INTER_NEAREST)
    out[mask > 0] = warped[mask > 0]
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["print", "solve"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--image")
    ap.add_argument("--intrinsics")
    ap.add_argument("--table", help="LxW cm")
    ap.add_argument("--corners", help="the table's 4 corners in the picture: u,v;u,v;u,v;u,v (0,0 / L,0 / L,W / 0,W)")
    ap.add_argument("--name", default="overhead")
    a = ap.parse_args(argv)
    if a.what == "print":
        print(print_png(a.out))
        return 0
    img = cv2.imread(a.image, cv2.IMREAD_GRAYSCALE)
    K, dist, size = intrinsics(a.intrinsics)
    table = tuple(float(v) for v in a.table.split("x")) if a.table else None
    corners = [tuple(float(v) for v in c.split(",")) for c in a.corners.split(";")] if a.corners else None
    r = solve(img, K, dist, size, table, corners, name=a.name)
    with open(a.out, "w") as f:
        json.dump(r.get("model") or r, f, indent=1)
    print(json.dumps({k: v for k, v in r.items() if k != "model"}, indent=1))
    return 0 if r.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
