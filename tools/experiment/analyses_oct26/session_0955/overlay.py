"""overlay.py IN OUT [x0 x1 y0 y1 zoom] : a frame with the stored calibration's lines drawn at floor level: the open
edges' lines (x = 0, y = 0, red), 4 and 8 cm in (yellow, green), 4 cm out (blue); crop and zoom."""
import sys, cv2, numpy as np
import pose
im = cv2.imread(sys.argv[1])
def line(pts, col):
    p = [tuple(int(round(c)) for c in pose.board_px(x, y)) for x, y in pts]
    for a, b in zip(p, p[1:]):
        cv2.line(im, a, b, col, 1, cv2.LINE_AA)
for off, col in ((0, (0, 0, 255)), (4, (0, 255, 255)), (8, (0, 200, 0)), (-4, (255, 120, 0))):
    line([(off, y) for y in np.linspace(-10, 125, 30)], col)
    line([(x, off) for x in np.linspace(-10, 125, 30)], col)
if len(sys.argv) > 3:
    x0, x1, y0, y1, z = (int(v) for v in sys.argv[3:8])
    im = cv2.resize(im[y0:y1, x0:x1], None, fx=z, fy=z, interpolation=cv2.INTER_CUBIC)
cv2.imwrite(sys.argv[2], im)
