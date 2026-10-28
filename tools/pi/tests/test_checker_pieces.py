"""Checker pieces (2026-10-28): the PDF's pages and sizes, the detector's 0,0 corner on every piece and turn, the setup
page's solve with the white margin and the dot, and the dock piece against dock v6's pocket and the bump check."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(PI))
for p in (PI, os.path.join(REPO, "tools", "experiment"), os.path.join(REPO, "tools", "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2  # noqa: E402
import checker_pieces as CP  # noqa: E402


@pytest.fixture(scope="module")
def pdf(tmp_path_factory):
    path = str(tmp_path_factory.mktemp("pieces") / "pieces.pdf")
    CP.make_pdf(path)
    return path


def test_pdf_is_three_letter_pages_and_every_piece_inside_the_printable_area(pdf):
    import subprocess
    info = subprocess.run(["pdfinfo", pdf], capture_output=True, text=True).stdout
    import re
    assert re.search(r"Pages:\s+3\n", info) and "612 x 792 pts (letter)" in info
    for p in CP.PIECES:
        w, h = CP.cut_mm(p)
        x0, y0 = p["at"]
        assert x0 >= CP.SAFE_MM and y0 >= CP.SAFE_MM
        assert x0 + w <= CP.PAGE_MM[0] - CP.SAFE_MM + 1e-6 and y0 + h <= CP.PAGE_MM[1] - CP.SAFE_MM + 1e-6, p["key"]
    # no two pieces on a page overlap
    for a in CP.PIECES:
        for b in CP.PIECES:
            if a is b or a["page"] != b["page"]:
                continue
            (ax, ay), (bx, by) = a["at"], b["at"]
            (aw, ah), (bw, bh) = CP.cut_mm(a), CP.cut_mm(b)
            assert ax + aw <= bx or bx + bw <= ax or ay + ah <= by or by + bh <= ay, (a["key"], b["key"])


def test_printed_squares_are_their_size_and_found_flat(pdf):
    """Each calibration piece straight from the PDF raster (10 px a mm): found, 0,0 read, squares the right size."""
    tex, ppm = CP.pdf_pieces(pdf)
    for key in "ABCD":
        p = CP.BY_KEY[key]
        img = cv2.resize(tex[key], None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)   # 2.5 px a mm
        img = cv2.copyMakeBorder(img, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=150)      # lying on a grey table
        r = CP.detect_piece(img, p)
        assert r["found"] and r["oriented"], key
        assert abs(r["px_per_square"] - p["sq"] * 2.5) < 0.6, (key, r["px_per_square"])
        # the first inner corner is one square in from the 0,0 corner: bottom-left of the picture
        H = img.shape[0] - 60                      # the piece's own height in the picture
        x, y = r["corners"][0] - 30
        assert abs(x - (p["margin"] + p["sq"]) * 2.5) < 1.5 and abs((H - y) - (p["margin"] + p["sq"]) * 2.5) < 1.5, key


@pytest.mark.parametrize("key", list("ABCD"))
def test_printed_pieces_found_from_6_to_100_px_a_square(pdf, key):
    """The finder misses some sizes of the minimal 4 x 4 board with one flag set: detect_piece tries several."""
    tex, ppm = CP.pdf_pieces(pdf)
    p = CP.BY_KEY[key]
    for spx in (6, 8, 10, 14, 20, 30, 40, 50, 67, 80, 100):
        f = spx / (p["sq"] * ppm)
        img = cv2.resize(tex[key], None, fx=f, fy=f, interpolation=cv2.INTER_AREA)
        img = cv2.copyMakeBorder(img, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=150)
        r = CP.detect_piece(img, p)
        assert r["found"] and r["oriented"], (key, spx)
        k = ppm * f                                # px a mm
        x, y = 30 + (p["margin"] + p["sq"]) * k, img.shape[0] - 30 - (p["margin"] + p["sq"]) * k
        assert np.hypot(r["corners"][0][0] - x, r["corners"][0][1] - y) < max(1.5, 0.1 * spx), (key, spx)


@pytest.mark.parametrize("key", list("ABCD"))
@pytest.mark.parametrize("turn", [0, 90, 180, 270])
def test_detector_reads_the_dot_corner_whatever_the_turn(key, turn):
    p = CP.BY_KEY[key]
    tex = CP.piece_texture(p, 0.8)            # 8 px a cm
    pad = 40
    img = np.full((tex.shape[0] + 2 * pad, tex.shape[1] + 2 * pad), 150, np.uint8)
    img[pad:-pad, pad:-pad] = tex
    rot = {0: None, 90: cv2.ROTATE_90_COUNTERCLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_CLOCKWISE}[turn]
    H0, W0 = img.shape
    if rot is not None:
        img = cv2.rotate(img, rot)
    r = CP.detect_piece(img, p)
    assert r["found"] and r["oriented"] and not r["mirrored"]
    # where the dot square's first inner corner should be, turned the same way
    x, y = pad + (p["margin"] + p["sq"]) * 0.8, H0 - pad - (p["margin"] + p["sq"]) * 0.8
    if turn == 90:
        x, y = y, W0 - x
    elif turn == 180:
        x, y = W0 - x, H0 - y
    elif turn == 270:
        x, y = H0 - y, x
    assert np.hypot(r["corners"][0][0] - x, r["corners"][0][1] - y) < 1.5, (key, turn, r["corners"][0], (x, y))
    # x runs along the piece's first count: corner 1 is one square from corner 0, corner nx-1 starts row 2
    nx = p["n"][0] - 1
    step = np.linalg.norm(r["corners"][1] - r["corners"][0])
    assert abs(step - p["sq"] * 0.8) < 1.0
    assert abs(np.linalg.norm(r["corners"][nx] - r["corners"][0]) - p["sq"] * 0.8) < 1.0


def test_mirrored_rectangle_is_flagged():
    """A print seen mirrored (a camera picture flipped by its driver, say) is flagged on a rectangle. (A square piece
    with one dot looks the same mirrored as turned, so there it cannot be told.)"""
    p = CP.BY_KEY["C"]
    tex = cv2.flip(CP.piece_texture(p, 0.8), 1)
    img = np.full((tex.shape[0] + 80, tex.shape[1] + 80), 150, np.uint8)
    img[40:-40, 40:-40] = tex
    r = CP.detect_piece(img, p)
    assert r["found"] and r["mirrored"] and not r["oriented"]


def test_dock_piece_is_the_strip_and_fits_the_v6_pocket():
    sys.path.insert(0, os.path.join(REPO, "hardware", "dock"))
    src = open(os.path.join(REPO, "hardware", "dock", "build_dock_v6.py")).read()
    import build_dock_v5 as V5
    w, h = CP.cut_mm(CP.DOCK)
    assert (w, h) == (V5.STRIP_LEN_MM, V5.STRIP_DEP_MM) == (90.0, 30.0)
    assert CP.DOCK["sq"] == V5.STRIP_CELL_MM and CP.DOCK["n"] == V5.STRIP_N
    assert "POCKET_DEPTH_CARD_MM = 0.6" in src
    r = CP.detect_piece(CP.piece_texture(CP.DOCK, 1.0), CP.DOCK)
    assert not r["found"] and "4 x 4" in r["why"]


def test_setup_solve_with_margin_and_dot():
    """The corner piece in the 0,0 corner: the setup page's solve puts the table where the true camera does; the same
    piece turned half way round in the corner is refused with the reason."""
    import camera_setup as CS
    import charuco_calib as CC
    w = CS.FakeWorld()
    src = CS.FakeSource(w, "overhead", fps=1000, seed=2)
    true = src.cam
    base = np.clip(src.base, 0, 255).astype(np.uint8)
    corners = CS.board_corners_px(true).tolist()
    K, dist = np.array(true.K, float), np.array(true.dist, float).ravel()
    table = (CS.CAM.BOARD_W_CM, CS.CAM.BOARD_L_CM)
    tex, ppm = CP.piece_texture(CP.CORNER, 10.0), 10.0
    img, _ = CP.render_on_floor(tex, ppm, true, (0.0, 0.0), 0.0, base, seed=3)
    r = CC.solve_checker(img, K, dist, tuple(true.image_size), table, corners)
    assert r["ok"] and r["dot_agrees"] is True, r
    cam = CS.CAM.CameraModel.from_dict(r["model"])
    for x, y in ((0, 0), (17.5, 24.5), (61, 61), (118, 118)):
        fx, fy = cam.to_floor(*true.project((x, y, 0)))
        assert math.hypot(fx - x, fy - y) < 0.6, (x, y, fx, fy)
    wc, hc = CP.cut_mm(CP.CORNER)
    img2, _ = CP.render_on_floor(tex, ppm, true, (wc / 10.0, hc / 10.0), 180.0, base, seed=3)
    r2 = CC.solve_checker(img2, K, dist, tuple(true.image_size), table, corners)
    assert not r2["ok"] and "turned" in r2["why"]


def test_dock_piece_bump_check_sees_half_a_pixel():
    for r in CP.dock_tests():
        for (dx, dy), (mx, my), cc in r["shifts"]:
            assert cc > 0.95 and abs(mx - dx) < 0.1 and abs(my - dy) < 0.1, (r["camera"], dx, dy, mx, my)
