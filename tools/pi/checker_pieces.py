#!/usr/bin/env python3
"""Checker pieces: one printable letter-size PDF of chessboard pieces for the cameras, and the detector that reads them
(Nick, 2026-10-27 22:03: "a file with a few checker pieces in it, squares and rectangles ... one that will fit on the
new dock").

    python3 tools/pi/checker_pieces.py pdf  [--out docs/hardware/checker-pieces-letter.pdf]
    python3 tools/pi/checker_pieces.py test [--pdf ...] [--shots docs/screenshots/2026-10-28]   # synthetic detection

Every calibration piece is a plain chessboard (the setup page's detector: OpenCV findChessboardCornersSB, which needs at
least 4 x 4 squares, i.e. 3 x 3 inner corners) with a WHITE DOT in its 0,0 corner square. The dot gives the piece a
direction even where the board itself is symmetric (an odd x odd board looks the same turned half way round; a square
board the same turned a quarter): this corner is 0,0, x runs along the piece's first count (the label's "5 x 7": 5
along x), y up the print towards the label strip, as the map (x right, y up, seen from above). The white margin round
the squares is part of each piece and is a known offset: the paper's corner is (0, 0), the squares start MARGIN in.

The dock piece is different: 2 x 6 squares of 15 mm (90 x 30 mm), the same pattern and size as dock v5/v6's marker
strip, cut on its own edge and glued (on stiff card) into the pocket on top of the dock's base. The overhead camera
does not look for its corners: the bump check (tools/session_checks.py PatternWatch) follows it as a picture, and the
size check (tools/label/size_check.py) measures its 90 x 30 mm outline. Two rows are below the chessboard finder's
4 x 4 minimum, and its 15 mm squares are 3.5 px across in the overhead picture, so the finder is not meant to find it.
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
PDF_DEFAULT = os.path.join(REPO, "docs", "hardware", "checker-pieces-letter.pdf")

PAGE_MM = (215.9, 279.4)        # US letter
SAFE_MM = 6.35                  # most printers do not print the outer 1/4 in: everything stays inside this
LABEL_MM = 12.0                 # the label strip inside the cut line, above the squares: 11 mm + 1 mm, after a gap of
GAP_MM = 3.0                    # 3 mm (each piece may ask for more: B's 4 x 4 board, the smallest pattern the finder takes,
                                # was missed at 67+ px a square with the strip 3 mm away; 10 mm fixed it, test table)
DOT_FRAC = 0.5                  # the white dot's diameter, as a share of a square. Chosen: big enough to read at ~8 px a
                                # square (4 px dot), small enough to leave the square's corner alone for the finder.
                                # DOE note: dot size x squares' pixels against the orientation read rate (test table)

# key, name, squares (x, y), square mm, white margin mm, page, cut rectangle's lower-left on the page (mm), purpose
PIECES = [
    dict(key="A", name="Corner piece", n=(5, 7), sq=35.0, margin=5.0, page=1, at=(15.45, 7.7),
         purpose="The setup page's anchor: white-dot corner in the table's 0,0 corner, the 5-square side along x, "
                 "while both cameras take a picture."),
    dict(key="B", name="Large square", n=(4, 4), sq=45.0, margin=8.0, gap=10.0, page=2, at=(9.95, 63.0),
         purpose="Far-end check: where squares come out smallest (the side camera's far half), to see each "
                 "camera's calibration holds at the far end."),
    dict(key="E", name="Dock piece", n=(6, 2), sq=15.0, margin=0.0, page=2, at=(20.0, 24.0), dock=True,
         purpose="Glued on stiff card into the pocket on top of dock v6's base: the overhead camera's bump check "
                 "and the system's known length (90 x 30 mm)."),
    dict(key="C", name="Small rectangle", n=(6, 4), sq=25.0, margin=6.0, page=3, at=(26.95, 150.4),
         purpose="Spot checks anywhere on the table (fits beside a robot); the side camera's near half."),
    dict(key="D", name="Small square", n=(5, 5), sq=20.0, margin=6.0, page=3, at=(51.95, 26.4),
         purpose="Close cameras: a laptop, phone or Pi camera on the desk (squares 20 mm), and the near edge of the "
                 "side camera."),
]
BY_KEY = {p["key"]: p for p in PIECES}
CORNER = BY_KEY["A"]
DOCK = BY_KEY["E"]


def board_mm(p):
    return p["n"][0] * p["sq"], p["n"][1] * p["sq"]


def cut_mm(p):
    """The piece's cut size (w, h) mm: squares, margin all round, and the label strip on top (none on the dock piece)."""
    w, h = board_mm(p)
    if p.get("dock"):
        return w, h
    return w + 2 * p["margin"], h + p["margin"] + p.get("gap", GAP_MM) + LABEL_MM


def black(i, k):
    """Square (i along x, k up from the 0,0 corner) is black: the 0,0 square is always black."""
    return (i + k) % 2 == 0


def inner_floor_mm(p):
    """The inner corners (i, j) -> (x, y) mm from the piece's own 0,0 (the paper's corner), x fastest."""
    nx, ny = p["n"]
    m, s = p["margin"], p["sq"]
    return np.array([[m + (i + 1) * s, m + (j + 1) * s] for j in range(ny - 1) for i in range(nx - 1)], np.float64)


# ===================================================================================================== the PDF
def make_pdf(path=PDF_DEFAULT):
    from reportlab.lib.colors import Color, black as BK, white as WH
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    from reportlab.pdfbase.pdfmetrics import stringWidth
    grey = Color(0.55, 0.55, 0.55)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    c = canvas.Canvas(path, pagesize=(PAGE_MM[0] * mm, PAGE_MM[1] * mm))
    c.setTitle("MIRLT checker pieces (letter, print at actual size)")
    c.setAuthor("MIRLT capstone")

    def text(x, y, s, size=7.0, bold=False, col=BK, anchor="l"):
        f = "Helvetica-Bold" if bold else "Helvetica"
        c.setFont(f, size)
        c.setFillColor(col)
        w = stringWidth(s, f, size) / mm
        x0 = x - w if anchor == "r" else (x - w / 2 if anchor == "c" else x)
        c.drawString(x0 * mm, y * mm, s)
        return w

    def ruler(x, y):
        """A 5 cm bar at (x, y) mm (its lower-left), 1 cm segments, labels under it."""
        for k in range(5):
            c.setFillColor(BK if k % 2 == 0 else WH)
            c.setStrokeColor(BK)
            c.setLineWidth(0.35)
            c.rect((x + 10 * k) * mm, y * mm, 10 * mm, 2.4 * mm, stroke=1, fill=1)
        for k in range(6):
            text(x + 10 * k, y - 2.3, "%d" % k, 5.0, anchor="c")
        text(x + 50 + 1.2, y - 2.3, "cm", 5.0)
        text(x + 25, y + 3.2, "5 cm: check with a ruler", 5.0, anchor="c")

    def squares(p, x0, y0):
        nx, ny = p["n"]
        s = p["sq"]
        c.setFillColor(BK)
        for k in range(ny):
            for i in range(nx):
                if black(i, k):
                    c.rect((x0 + i * s) * mm, (y0 + k * s) * mm, s * mm, s * mm, stroke=0, fill=1)

    def fits(x0, y0, w, h, what):
        assert x0 >= SAFE_MM - 1e-6 and y0 >= SAFE_MM - 1e-6 and x0 + w <= PAGE_MM[0] - SAFE_MM + 1e-6 and \
            y0 + h <= PAGE_MM[1] - SAFE_MM + 1e-6, "%s does not fit the printable area" % what

    for page in sorted({p["page"] for p in PIECES}):
        for p in [q for q in PIECES if q["page"] == page]:
            x0, y0 = p["at"]
            w, h = cut_mm(p)
            bw, bh = board_mm(p)
            fits(x0, y0, w, h, p["key"])
            m = p["margin"]
            bx, by = x0 + m, y0 + m
            if p.get("dock"):
                squares(p, bx, by)
                # cut ON the pattern's own edge: a hairline there, and crop marks outside the corners
                c.setStrokeColor(grey)
                c.setLineWidth(0.2)
                c.rect(bx * mm, by * mm, bw * mm, bh * mm, stroke=1, fill=0)
                c.setLineWidth(0.3)
                for cx, cy, dx, dy in ((bx, by, -1, -1), (bx + bw, by, 1, -1), (bx, by + bh, -1, 1),
                                       (bx + bw, by + bh, 1, 1)):
                    c.line((cx + dx * 2) * mm, cy * mm, (cx + dx * 6) * mm, cy * mm)
                    c.line(cx * mm, (cy + dy * 2) * mm, cx * mm, (cy + dy * 6) * mm)
                text(bx, by + bh + 4.0, "%s  %s  -  %d x %d squares of %.0f mm (%.0f x %.0f mm)"
                     % (p["key"], p["name"], p["n"][0], p["n"][1], p["sq"], bw, bh), 8.0, True)
                text(bx, by + bh + 1.0, "Cut ON the thin grey edge line (no white border). Either way round.", 6.0)
                tx = bx + bw + 12
                ruler(tx, by + 22)
                for k, s in enumerate(("Glue it flat on stiff card (cereal-box card),", "cut the card to the same edge,",
                                       "press it into the pocket on top of the dock's", "base (the bottom of the U).",
                                       "Long side across the dock.")):
                    text(tx, by + 14 - 3.0 * k, s, 6.0)
                continue
            # cut line (dashed) round the whole piece
            c.setStrokeColor(grey)
            c.setLineWidth(0.3)
            c.setDash(2 * mm, 1.5 * mm)
            c.rect(x0 * mm, y0 * mm, w * mm, h * mm, stroke=1, fill=0)
            c.setDash()
            text(x0 + w - 1.0, y0 - 3.2 if y0 > SAFE_MM + 4 else y0 + h + 1.0, "cut on the dashed line", 5.0, col=grey,
                 anchor="r")
            squares(p, bx, by)
            # the white dot in the 0,0 square (bottom-left)
            c.setFillColor(WH)
            c.circle((bx + p["sq"] / 2) * mm, (by + p["sq"] / 2) * mm, DOT_FRAC * p["sq"] / 2 * mm, stroke=0, fill=1)
            # label strip
            ly = by + bh + p.get("gap", GAP_MM)
            ruler(x0 + w - 52 - 1.5, ly + 4.2)
            room = w - 52 - 2 * 1.5 - 3
            title = "%s  %s  -  %d x %d squares of %.0f mm" % (p["key"], p["name"], p["n"][0], p["n"][1], p["sq"])
            mark = "0,0 = the corner square with the white dot (bottom left). This way up."
            for size in (8.0, 7.0, 6.0):
                if stringWidth(title, "Helvetica-Bold", size) / mm <= room:
                    break
            text(x0 + 1.5, ly + 7.6, title, size, True)
            text(x0 + 1.5, ly + 4.4, mark if stringWidth(mark, "Helvetica", 5.5) / mm <= room else
                 "0,0 = the white-dot corner. This way up.", 5.5)
            pur = p["purpose"]
            if stringWidth(pur, "Helvetica", 5.0) / mm <= room:
                text(x0 + 1.5, ly + 1.4, pur, 5.0)
            else:
                # two halves on one line would not fit: the first sentence only; the full line is on the sheet
                text(x0 + 1.5, ly + 1.4, pur.split(":")[0] + ".", 5.0)
        # sheet notes (outside the pieces)
        if page == 2:
            text(SAFE_MM + 2, 13.0, "Print at ACTUAL SIZE (100 %), not 'Fit to page'. Check each 5 cm bar with a ruler "
                 "before cutting.", 7.0, True)
            text(SAFE_MM + 2, 9.8, "B: " + BY_KEY["B"]["purpose"], 5.6)
            text(SAFE_MM + 2, 7.2, "E: the pocket is 90.8 x 30.8 mm, 0.6 mm deep (dock v6, build_dock_v6.py).", 5.6)
        if page == 3:
            text(SAFE_MM + 2, 18.5, "Print at ACTUAL SIZE (100 %), not 'Fit to page'. Check each 5 cm bar.", 7.0,
                 True)
            text(SAFE_MM + 2, 15.0, "C: " + BY_KEY["C"]["purpose"], 5.6)
            text(SAFE_MM + 2, 11.5, "D: " + BY_KEY["D"]["purpose"], 5.6)
            text(SAFE_MM + 2, 8.0, "Glue a piece flat on stiff card: a wavy piece reads wrong. Pieces made by "
                 "tools/pi/checker_pieces.py.", 5.6)
        c.showPage()
    c.save()
    return path


def piece_rect_px(p, dpi):
    """The piece's cut rectangle in a page raster at dpi: (x0, y0, x1, y1) px, top-left origin."""
    k = dpi / 25.4
    w, h = cut_mm(p)
    x0, y0 = p["at"]
    return (int(round(x0 * k)), int(round((PAGE_MM[1] - y0 - h) * k)), int(round((x0 + w) * k)),
            int(round((PAGE_MM[1] - y0) * k)))


def pdf_pieces(pdf=PDF_DEFAULT, dpi=254):
    """{key: grey picture of the cut piece, rasterised from the PDF itself (pdftoppm)}: 10 px a mm at 254 dpi."""
    import cv2
    out = {}
    with tempfile.TemporaryDirectory() as td:
        subprocess.run(["pdftoppm", "-r", str(dpi), "-gray", "-png", pdf, os.path.join(td, "p")], check=True)
        pages = sorted(f for f in os.listdir(td) if f.endswith(".png"))
        imgs = [cv2.imread(os.path.join(td, f), cv2.IMREAD_GRAYSCALE) for f in pages]
    for p in PIECES:
        x0, y0, x1, y1 = piece_rect_px(p, dpi)
        out[p["key"]] = imgs[p["page"] - 1][y0:y1, x0:x1].copy()
    return out, dpi / 25.4


def piece_texture(p, px_per_mm=10.0):
    """The piece drawn directly (no labels): white margin, squares, dot. For the fake cameras and quick tests."""
    w, h = cut_mm(p)
    W, H = int(round(w * px_per_mm)), int(round(h * px_per_mm))
    img = np.full((H, W), 255, np.uint8)
    nx, ny = p["n"]
    s, m = p["sq"], p["margin"]
    for k in range(ny):
        for i in range(nx):
            if black(i, k):
                x0, x1 = int(round((m + i * s) * px_per_mm)), int(round((m + (i + 1) * s) * px_per_mm))
                y1, y0 = H - int(round((m + k * s) * px_per_mm)), H - int(round((m + (k + 1) * s) * px_per_mm))
                img[y0:y1, x0:x1] = 0
    if not p.get("dock"):
        import cv2
        cv2.circle(img, (int(round((m + s / 2) * px_per_mm)), H - int(round((m + s / 2) * px_per_mm))),
                   int(round(DOT_FRAC * s / 2 * px_per_mm)), 255, -1, lineType=cv2.LINE_AA)
    return img


# ===================================================================================================== detection
def _sample(g, pts):
    import cv2
    pts = np.asarray(pts, np.float32).reshape(-1, 1, 2)
    v = cv2.remap(g.astype(np.float32), pts[:, :, 0], pts[:, :, 1], cv2.INTER_LINEAR, borderValue=float("nan"))
    v = v[np.isfinite(v)]
    return float(v.mean()) if v.size else float("nan")


def detect_piece(img, p=CORNER):
    """Find piece p in a picture. Returns a dict:
    found, corners (N, 2) px in the piece's order (x fastest, from the 0,0 corner), floor_mm (N, 2) the same corners
    in the piece's own frame (mm from the paper's 0,0 corner), oriented (the white dot was read), dot_contrast (grey
    levels: the dot square's centre against its own black), px_per_square, raw (OpenCV's own order), pattern."""
    import cv2
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    nx, ny = p["n"]
    if min(nx, ny) < 4:
        return {"found": False, "why": "%d x %d squares: the chessboard finder needs at least 4 x 4" % (nx, ny)}
    pat = (nx - 1, ny - 1)
    first = None
    # OpenCV's finder is not equally sure with every flag set: on the 4 x 4 board (3 x 3 corners, its minimum) the
    # accurate exhaustive search missed some sizes the plain one found. So several tries; the first grid whose white
    # dot reads is taken (the dot also rejects a grid shifted onto the paper's edge, see the report), else the first
    # grid found, flagged not oriented
    flag_sets = (cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY, cv2.CALIB_CB_ACCURACY, 0, cv2.CALIB_CB_EXHAUSTIVE)
    seen = []
    for fl in flag_sets:
        for q in ((pat, pat[::-1]) if pat[0] != pat[1] else (pat,)):
            ok, cc = cv2.findChessboardCornersSB(g, q, flags=fl)
            if not ok:
                continue
            raw = cc.reshape(-1, 2).astype(np.float64)
            if any(np.allclose(raw, r0, atol=0.5) for r0 in seen):
                continue
            seen.append(raw)
            r = _orient(g, p, raw, q)
            if not r["found"]:
                continue
            if r["oriented"]:
                return r
            first = first or r
            break
    return first or {"found": False, "why": "not found"}


def _orient(g, p, raw, used):
    """The piece's order for a grid the finder returned: which corner square has the dot, which way x runs."""
    import cv2
    nx, ny = p["n"]
    w, h = used
    grid = np.array([[k % w, k // w] for k in range(w * h)], np.float64)
    Hm, _ = cv2.findHomography(grid, raw)

    def to_px(pts):
        return cv2.perspectiveTransform(np.asarray(pts, np.float64).reshape(-1, 1, 2), Hm).reshape(-1, 2)

    pxsq = float(np.median(np.linalg.norm(np.diff(raw.reshape(h, w, 2), axis=1), axis=2)))
    # is it the board? every one of its squares (inner corners' grid extended by half a square all round), sampled at
    # four points away from the dot, must fall into two groups by parity, every dark one darker than every light one
    vals = np.zeros((h + 1, w + 1))
    for b in range(h + 1):
        for a in range(w + 1):
            vals[b, a] = _sample(g, to_px([(a - 0.5 + dx, b - 0.5 + dy) for dx in (-0.3, 0.3) for dy in (-0.3, 0.3)]))
    par = (np.add.outer(np.arange(h + 1), np.arange(w + 1)) % 2).astype(bool)
    if not np.all(np.isfinite(vals)):
        return {"found": False, "why": "part of the board is outside the picture"}
    g0, g1 = vals[~par], vals[par]
    dark_is_1 = g1.mean() < g0.mean()
    dark, light = (g1, g0) if dark_is_1 else (g0, g1)
    # chosen: 10 grey levels between the lightest dark square and the darkest light one (paper under room light gives
    # 150+; a grid that strays off the board mixes the two groups). No DOE: the test table's pictures all pass by far
    if float(light.min() - dark.max()) < 10.0:
        return {"found": False, "why": "a grid was found but its squares do not alternate (not this piece)"}
    # the four corner squares' centres in grid units, and which one has the dot (white middle in a black square)
    cands = [(-0.5, -0.5), (w - 0.5, -0.5), (-0.5, h - 0.5), (w - 0.5, h - 0.5)]
    ring = [(dx, dy) for dx in (-0.36, 0.0, 0.36) for dy in (-0.36, 0.0, 0.36) if (dx, dy) != (0.0, 0.0)]
    mid = [(dx, dy) for dx in (-0.08, 0.0, 0.08) for dy in (-0.08, 0.0, 0.08)]
    scores = []
    for cx, cy in cands:
        cm = _sample(g, to_px([(cx + dx, cy + dy) for dx, dy in mid]))
        cr = _sample(g, to_px([(cx + dx, cy + dy) for dx, dy in ring]))
        sc = cm - cr
        a, b = int(cx + 0.5), int(cy + 0.5)
        is_dark = par[b, a] == dark_is_1                      # the 0,0 square is black: only a dark one can hold the dot
        scores.append(sc if np.isfinite(sc) and is_dark else -999.0)
    order = np.argsort(scores)[::-1]
    best, second = scores[order[0]], scores[order[1]]
    # chosen: the dot square's middle at least 20 grey levels above its own ring and 15 above any other corner square.
    # No DOE yet: the test table's dot contrast (all pictures) is the data to set it from
    oriented = bool(best >= 20.0 and best - second >= 15.0)
    ox, oy = cands[order[0]] if oriented else cands[0]
    sx, sy = (1.0 if ox < 0 else -1.0), (1.0 if oy < 0 else -1.0)
    gx0, gy0 = (0.0 if ox < 0 else w - 1.0), (0.0 if oy < 0 else h - 1.0)
    # the two grid axes from the dot corner, in the picture
    o = to_px([(gx0, gy0)])[0]
    ea = to_px([(gx0 + sx, gy0)])[0] - o
    eb = to_px([(gx0, gy0 + sy)])[0] - o
    cross = ea[0] * eb[1] - ea[1] * eb[0]
    # x along the piece's first count; on a square piece, x is the axis that makes (x, y) right-handed seen from above
    # (picture v runs down, so a right-handed floor pair has a negative cross product in the picture)
    x_is_a = (w == nx - 1) if nx != ny else cross < 0
    mirrored = (cross < 0) != x_is_a
    out = np.zeros_like(raw)
    for k in range(w * h):
        ga, gb = grid[k]
        ia, ib = int(round((ga - gx0) * sx)), int(round((gb - gy0) * sy))
        i, j = (ia, ib) if x_is_a else (ib, ia)
        out[j * (nx - 1) + i] = raw[k]
    return {"found": True, "corners": out, "floor_mm": inner_floor_mm(p), "oriented": oriented and not mirrored,
            "dot_contrast": round(float(best), 1), "dot_margin": round(float(best - second), 1),
            "px_per_square": round(pxsq, 1), "raw": raw, "pattern": used, "mirrored": bool(mirrored)}


# ===================================================================================================== synthetic camera
def render_on_floor(tex, px_per_mm, cam, at_cm, angle_deg, base, z_cm=0.0, blur=0.7, noise=2.0, seed=0):
    """Picture `base` (grey) with the piece texture lying on the floor: its 0,0 corner (the paper's lower-left) at
    at_cm, turned angle_deg (x axis from the floor's x). Returns (picture, floor_to_picture(points_mm) function)."""
    import cv2
    th = math.radians(angle_deg)
    ca, sa = math.cos(th), math.sin(th)
    H, W = tex.shape

    def floor_of(xy_mm):
        xy = np.asarray(xy_mm, np.float64).reshape(-1, 2) / 10.0
        return np.column_stack([at_cm[0] + ca * xy[:, 0] - sa * xy[:, 1], at_cm[1] + sa * xy[:, 0] + ca * xy[:, 1]])

    def to_px(xy_mm):
        return np.array([cam.project((x, y, z_cm)) for x, y in floor_of(xy_mm)], np.float64)

    wmm, hmm = W / px_per_mm, H / px_per_mm
    src = np.float32([[0, 0], [W, 0], [W, H], [0, H]])
    dst = np.float32(to_px([(0, hmm), (wmm, hmm), (wmm, 0), (0, 0)]))
    Hm = cv2.getPerspectiveTransform(src, dst)
    Wi, Hi = cam.image_size
    # paper is not quite white or black under room light: 235 / 30 (chosen, the fake camera's board is 165)
    shade = (tex.astype(np.float32) / 255.0) * 205.0 + 30.0
    warped = cv2.warpPerspective(shade, Hm, (Wi, Hi), flags=cv2.INTER_AREA, borderValue=0)
    mask = cv2.warpPerspective(np.full(tex.shape, 1.0, np.float32), Hm, (Wi, Hi), flags=cv2.INTER_AREA)
    out = base.astype(np.float32) * (1 - mask) + warped * mask
    if blur:
        out = cv2.GaussianBlur(out, (0, 0), blur)
    rng = np.random.default_rng(seed)
    out = out + rng.normal(0, noise, out.shape)
    return np.clip(out, 0, 255).astype(np.uint8), to_px


def _cameras():
    """(name, CameraModel, base picture) for the overhead (roam1's real lens and pose, slightly moved: the setup page's
    fake overhead) and the side camera (the simulator's angled camera)."""
    sys.path.insert(0, HERE)
    sys.path.insert(0, os.path.join(REPO, "tools", "experiment"))
    import camera_setup as CS
    w = CS.FakeWorld()
    out = []
    for which in ("overhead", "side"):
        src = CS.FakeSource(w, which, fps=1000, seed=2)
        out.append((which, src.cam, np.clip(src.base, 0, 255).astype(np.uint8)))
    return out


# where each piece is laid in the test (floor cm, turn deg): its intended spot first, then two more
SPOTS = {
    "A": [((0.0, 0.0), 0.0, "table 0,0 corner"), ((40.0, 50.0), 20.0, "middle, turned 20"),
          ((112.0, 78.0), 90.0, "far side, turned 90")],
    "B": [((98.0, 98.0), 0.0, "far corner"), ((50.0, 40.0), -30.0, "middle, turned -30"),
          ((30.0, 100.0), 180.0, "left side, turned 180")],
    "C": [((15.0, 15.0), 0.0, "near corner"), ((55.0, 60.0), 35.0, "middle, turned 35"),
          ((90.0, 95.0), -90.0, "far corner, turned -90")],
    "D": [((15.0, 15.0), 0.0, "near corner"), ((60.0, 55.0), 45.0, "middle, turned 45"),
          ((100.0, 100.0), 180.0, "far corner, turned 180")],
}


def close_cams():
    """Two desk cameras for the small square D (its purpose: a camera held close): 640 x 480, about 63 deg across
    (f = 520 px), 45 cm from the piece, straight down and tilted 35 deg. Chosen to stand for a webcam or the Pi
    camera on the desk, not measured on one."""
    sys.path.insert(0, os.path.join(REPO, "tools", "recognition"))
    from cameras import CameraModel
    import cv2
    K = [[520.0, 0, 320.0], [0, 520.0, 240.0], [0, 0, 1]]
    out = []
    for name, tilt in (("close, straight", 0.0), ("close, tilted 35", 35.0)):
        t = math.radians(tilt)
        # the camera above the piece's middle (6, 6.6 cm), backed off along -y by the tilt, looking at the middle
        C = np.array([6.0, 6.6 - 45.0 * math.sin(t), 45.0 * math.cos(t)])
        z = np.array([6.0, 6.6, 0.0]) - C
        z /= np.linalg.norm(z)
        x = np.array([1.0, 0.0, 0.0])
        y = np.cross(z, x)
        R = np.vstack([x, y, z])
        rvec = cv2.Rodrigues(R)[0].ravel()
        out.append((name, CameraModel(K, np.zeros(5), rvec, -R @ C, (640, 480), name=name),
                    np.full((480, 640), 150, np.uint8)))
    return out


def run_tests(pdf=PDF_DEFAULT, shots=None):
    """Every calibration piece, rasterised from the PDF, laid at 3 spots, in both cameras' pictures: found?, the 0,0
    corner read right?, corner error against the truth (px), pixels a square. Returns rows (dicts)."""
    import cv2
    tex, ppm = pdf_pieces(pdf)
    rows = []
    pics = {}
    jobs = [(c, b, cb, "ABCD", None) for c, b, cb in _cameras()]
    jobs += [(c, b, cb, "D", [((0.0, 0.0), 0.0, "on the desk"), ((1.0, 0.5), 20.0, "on the desk, turned 20"),
                               ((11.0, 13.0), 180.0, "on the desk, turned 180")]) for c, b, cb in close_cams()]
    for cam_name, cam, base, keys, spots in jobs:
        for key in keys:
            p = BY_KEY[key]
            for n, (at, ang, where) in enumerate(spots or SPOTS[key]):
                img, to_px = render_on_floor(tex[key], ppm, cam, at, ang, base, seed=n)
                truth = to_px(inner_floor_mm(p))
                pxsq = float(np.median(np.linalg.norm(np.diff(to_px([(0, 0), (p["sq"], 0)]), axis=0), axis=1)))
                inside = bool(np.all((truth[:, 0] > 2) & (truth[:, 0] < cam.image_size[0] - 2) &
                                     (truth[:, 1] > 2) & (truth[:, 1] < cam.image_size[1] - 2)))
                r = detect_piece(img, p)
                row = dict(piece=key, camera=cam_name, where=where, px_sq=round(pxsq, 1), in_view=inside,
                           found=r["found"], oriented=r.get("oriented"), dot=r.get("dot_contrast"))
                if r["found"]:
                    err = np.linalg.norm(r["corners"] - truth, axis=1)
                    row["err_px"] = round(float(np.median(err)), 2)
                    row["right_way"] = bool(np.max(err) < 0.5 * pxsq)
                rows.append(row)
                if n == 0:
                    pics[(key, cam_name)] = (img, r, truth)
    if shots:
        _test_sheet(pics, rows, os.path.join(shots, "checker-pieces-feature-detected-overhead-and-angled.png"))
    return rows


def dock_tests(shots=None):
    """The dock piece on the dock's base (overhead): the bump check's PatternWatch follows a 0.5 px camera bump; the
    chessboard finder does not look for it (2 rows)."""
    import cv2
    sys.path.insert(0, os.path.join(REPO, "tools"))
    import session_checks as SCK
    (_, cam, base), (_, side, sbase) = _cameras()
    tex = piece_texture(DOCK, 10.0)
    # the dock's base top (black PLA, 36 x 125 mm) with the pocket; the piece in it. Dock near the table's middle
    # edge, mouth towards +x (the base at the low-x end): base top spans x 0..36 mm, y -62.6..62.6 mm
    at = (8.0, 61.0)
    res = []
    for name, c, b in (("overhead", cam, base), ("side", side, sbase)):
        dock_tex = np.full((1252, 360), 40, np.uint8)            # 10 px a mm: 36 x 125.2 mm of black PLA
        x0, y0 = int(round((3.0 + 0.4) * 10)), int(round((62.6 - 45.0) * 10))
        piece_rot = cv2.rotate(tex, cv2.ROTATE_90_CLOCKWISE)     # long side across the dock (floor y)
        dock_tex[y0:y0 + piece_rot.shape[0], x0:x0 + piece_rot.shape[1]] = np.maximum(piece_rot, 30)
        img0, to_px = render_on_floor(dock_tex, 10.0, c, (at[0], at[1] - 6.26), 0.0, b, z_cm=2.2, seed=1)
        ctr = to_px([(18.4, 62.6)])[0]
        corners = to_px([(3.4, 17.6), (33.4, 17.6), (33.4, 107.6), (3.4, 107.6)])
        hw = int(round(np.ptp(corners[:, 0]) / 2 + 1.2))
        hh = int(round(np.ptp(corners[:, 1]) / 2 + 1.2))
        pw = SCK.PatternWatch(ctr, half_px=(hw, hh), margin_px=4)
        pw.update(img0.astype(np.float32))
        shifts = []
        for k, (dx, dy) in enumerate(((0.0, 0.0), (0.5, 0.0), (0.0, 0.5), (1.0, 1.0))):
            M = np.float32([[1, 0, dx], [0, 1, dy]])
            moved = cv2.warpAffine(img0, M, (img0.shape[1], img0.shape[0]), flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_REPLICATE)
            moved = np.clip(moved + np.random.default_rng(10 + k).normal(0, 2.0, moved.shape), 0, 255).astype(np.uint8)
            mdx, mdy, cc, _ = pw.update(moved.astype(np.float32))
            shifts.append(((dx, dy), (round(mdx, 2), round(mdy, 2)), round(cc, 3)))
        r = detect_piece(img0, DOCK)
        pxsq = float(np.linalg.norm(np.diff(to_px([(0, 0), (15.0, 0)]), axis=0)))
        res.append(dict(camera=name, px_sq=round(pxsq, 1), shifts=shifts, finder=r.get("why", "found"),
                        img=img0, ctr=ctr, corners=corners))
    if shots:
        _dock_sheet(res, os.path.join(shots, "checker-pieces-feature-dock-piece-bump-check.png"))
    return res


# ===================================================================================================== pictures
def _test_sheet(pics, rows, path):
    import cv2
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 4, figsize=(16, 7.6))
    for ci, cam_name in enumerate(("overhead", "side")):
        for ki, key in enumerate("ABCD"):
            ax = axs[ci][ki]
            img, r, truth = pics[(key, cam_name)]
            p = BY_KEY[key]
            c = r.get("corners") if r["found"] else None
            pts = c if c is not None else truth
            pad = 40
            u0, v0 = max(0, int(pts[:, 0].min()) - pad), max(0, int(pts[:, 1].min()) - pad)
            u1, v1 = min(img.shape[1], int(pts[:, 0].max()) + pad), min(img.shape[0], int(pts[:, 1].max()) + pad)
            ax.imshow(img[v0:v1, u0:u1], cmap="gray", vmin=0, vmax=255, extent=(u0, u1, v1, v0))
            if c is not None:
                ax.plot(c[:, 0], c[:, 1], ".", ms=4, color="#e8590c")
                ax.plot(c[0, 0], c[0, 1], "o", ms=9, mfc="none", mec="#1971c2", mew=2)
                nxm = p["n"][0] - 1
                ax.annotate("", xy=c[nxm - 1], xytext=c[0], arrowprops=dict(arrowstyle="->", color="#1971c2", lw=1.6))
            row = next(rw for rw in rows if rw["piece"] == key and rw["camera"] == cam_name)
            ax.set_title("%s %s, %s camera\n%s: %s, %.1f px a square" % (
                key, p["name"], cam_name, row["where"],
                ("found, 0,0 read right" if row.get("right_way") and row.get("oriented") else
                 "found" if row["found"] else "NOT found"), row["px_sq"]), fontsize=9)
            ax.set_xticks([])
            ax.set_yticks([])
    fig.suptitle("Checker pieces A-D, rasterised from the PDF, in synthetic pictures from the setup page's two simulated "
                 "cameras (overhead: roam1's lens; side: the simulator's angled camera)", fontsize=11)
    fig.text(0.5, 0.015, "Orange dots: the inner corners the detector found. Blue ring: the corner it read as 0,0 (next "
             "to the white dot); blue arrow: the piece's x direction.", ha="center", fontsize=10)
    fig.tight_layout(rect=(0, 0.03, 1, 0.94))
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _dock_sheet(res, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax, r in zip(axs, res):
        img, cs = r["img"], r["corners"]
        pad = 25
        u0, v0 = max(0, int(cs[:, 0].min()) - pad), max(0, int(cs[:, 1].min()) - pad)
        u1, v1 = min(img.shape[1], int(cs[:, 0].max()) + pad), min(img.shape[0], int(cs[:, 1].max()) + pad)
        ax.imshow(img[v0:v1, u0:u1], cmap="gray", vmin=0, vmax=255, extent=(u0, u1, v1, v0),
                  interpolation="nearest")
        poly = np.vstack([cs, cs[:1]])
        ax.plot(poly[:, 0], poly[:, 1], "-", color="#e8590c", lw=1)
        s = r["shifts"]
        ax.set_title("Dock piece E, %s camera: %.1f px a square\nbump 0.5 px right -> measured %.2f px; 1 px diagonal -> "
                     "%.2f, %.2f" % (r["camera"], r["px_sq"], s[1][1][0], s[3][1][0], s[3][1][1]), fontsize=9)
        ax.set_xticks([])
        ax.set_yticks([])
    fig.text(0.5, 0.02, "The dock piece on the dock's base in synthetic pictures; orange: the 90 x 30 mm outline the bump "
             "check watches (tools/session_checks.py PatternWatch).", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(path, dpi=110)
    plt.close(fig)


def pdf_preview(pdf, path, dpi=60):
    """All pages side by side, one picture."""
    import cv2
    with tempfile.TemporaryDirectory() as td:
        subprocess.run(["pdftoppm", "-r", str(dpi), "-png", pdf, os.path.join(td, "p")], check=True)
        pages = [cv2.imread(os.path.join(td, f)) for f in sorted(os.listdir(td)) if f.endswith(".png")]
    gap = np.full((pages[0].shape[0], 16, 3), 200, np.uint8)
    row = []
    for pg in pages:
        pg = pg.copy()
        cv2.rectangle(pg, (0, 0), (pg.shape[1] - 1, pg.shape[0] - 1), (120, 120, 120), 1)
        row += [pg, gap]
    cv2.imwrite(path, np.hstack(row[:-1]))
    return path


def table_md(rows):
    lines = ["| Piece | Camera | Spots found | Pixels a square | 0,0 read right | Corner error, median (px) |",
             "|---|---|---|---|---|---|"]
    for key in "ABCD":
        for cam in ("overhead", "side") + (("close, straight", "close, tilted 35") if key == "D" else ()):
            rs = [r for r in rows if r["piece"] == key and r["camera"] == cam]
            found = [r for r in rs if r["found"]]
            right = [r for r in found if r.get("right_way") and r.get("oriented")]
            errs = [r["err_px"] for r in found]
            lines.append("| %s %s %d x %d, %.0f mm | %s | %d of %d | %s | %d of %d | %s |" % (
                key, BY_KEY[key]["name"], BY_KEY[key]["n"][0], BY_KEY[key]["n"][1], BY_KEY[key]["sq"], cam,
                len(found), len(rs), "%.1f-%.1f" % (min(r["px_sq"] for r in rs), max(r["px_sq"] for r in rs)),
                len(right), len(found), "%.2f-%.2f" % (min(errs), max(errs)) if errs else "-"))
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["pdf", "test"])
    ap.add_argument("--out", default=PDF_DEFAULT)
    ap.add_argument("--pdf", default=PDF_DEFAULT)
    ap.add_argument("--shots", help="folder for the test pictures")
    a = ap.parse_args(argv)
    if a.what == "pdf":
        print(make_pdf(a.out))
        for p in PIECES:
            w, h = cut_mm(p)
            print("  %s %-16s %d x %d squares of %4.1f mm   squares %5.1f x %5.1f mm   cut %5.1f x %5.1f mm   page %d"
                  % (p["key"], p["name"], p["n"][0], p["n"][1], p["sq"], *board_mm(p), w, h, p["page"]))
        print("print at Actual size (100 %), not 'Fit to page'; each 5 cm bar must measure 5 cm")
        return 0
    if a.shots:
        os.makedirs(a.shots, exist_ok=True)
        pdf_preview(a.pdf, os.path.join(a.shots, "checker-pieces-feature-pdf-pages.png"))
    rows = run_tests(a.pdf, a.shots)
    for r in rows:
        print(r)
    print(table_md(rows))
    for r in dock_tests(a.shots):
        print("dock piece, %s: %.1f px a square; chessboard finder: %s; bump (true -> measured, match): %s"
              % (r["camera"], r["px_sq"], r["finder"], r["shifts"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
