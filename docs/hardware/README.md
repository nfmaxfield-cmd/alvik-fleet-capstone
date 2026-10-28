# Printouts for the cameras

## Checker pieces (2026-10-28): `checker-pieces-letter.pdf`

Three letter pages, five pieces. Made by `python3 tools/pi/checker_pieces.py pdf`; the detector is
`checker_pieces.detect_piece` (the setup page's chessboard finder, OpenCV `findChessboardCornersSB`, plus a check
that the squares alternate and a read of the white dot).

| Piece | Squares | Piece size (cut) | Page | What it is for |
|---|---|---|---|---|
| A Corner piece (rectangle) | 5 x 7 of 35 mm | 185 x 265 mm | 1 | The setup page's anchor: white-dot corner in the table's 0,0 corner, the 5-square side along x, while both cameras take a picture |
| B Large square | 4 x 4 of 45 mm | 196 x 210 mm | 2 | Far-end check: where squares come out smallest, to see a camera's calibration holds at the far end of the table |
| C Small rectangle | 6 x 4 of 25 mm | 162 x 121 mm | 3 | Spot checks anywhere on the table (fits beside a robot); the side camera's near half |
| D Small square | 5 x 5 of 20 mm | 112 x 121 mm | 3 | A camera held close: a laptop, phone or Pi camera on the desk (too small for the table cameras' far half) |
| E Dock piece | 2 x 6 of 15 mm | 90 x 30 mm | 2 | Glued on stiff card into the pocket on top of dock v6's base: the overhead camera's bump check and the known length |

**Which way round.** On A-D, the corner square with the **white dot** is 0,0; the label strip is the top ("this way
up"); x runs along the first number of "5 x 7" (5 squares on A). The squares start 5 mm in from the paper's edge on
A (the setup page counts that offset), so the paper's corner goes in the table's corner. E is not read as a
chessboard (2 rows are below the finder's 4 x 4 minimum, and its squares are about 3.5 px in the overhead picture):
the bump check follows it as a picture, either way round.

**Printing.** Print at **Actual size / 100 %** (not "Fit to page"). Check each piece's **5 cm bar** with a ruler
before cutting: it must measure 5 cm (a square on A must measure 3.5 cm). Cut on the dashed lines (E: on its thin
grey edge line, no white border), glue each piece flat on stiff card (cereal-box card); a wavy piece reads wrong.

**Tested** (synthetic pictures, the PDF itself rasterised and laid on the table in the setup page's two simulated
cameras; report: `docs/build-reports/e02-oct28-checker-pieces.md`): A, B and C found with the 0,0 corner right in
every spot in both cameras (5.5-11.9 px a square); D in 4 of 6 table spots (only the 4.5 px ones missed) and in all 6
close-camera pictures; E's bump check reads a 0.5 px camera bump as 0.45-0.51 px.

`chessboard-corner-piece-5x7-3.5cm.pdf` (the one-system build's corner piece: same squares and the same 5 mm margin,
no dot, no labels) still works on the setup page; the new PDF replaces it.
