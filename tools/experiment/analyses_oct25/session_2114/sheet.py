"""sheet.py DIR OUT x0 x1 y0 y1 zoom cols TIME... : a contact sheet of crops (calibration lines drawn as overlay.py)
nearest each local time, labelled with the time."""
import sys, os, bisect, cv2, numpy as np
import pose
from common import ep, lt
d, out = sys.argv[1], sys.argv[2]
x0, x1, y0, y1, z, cols = (int(v) for v in sys.argv[3:9])
times = sys.argv[9:]
fr = [l.strip().split(',') for l in open(os.path.join(d, 'times.csv'))]
fr = sorted((float(t), int(k)) for k, t in fr)
ts = [f[0] for f in fr]
tiles = []
for s in times:
    t = ep(s)
    i = min(range(max(0, bisect.bisect_left(ts, t) - 1), min(len(ts), bisect.bisect_left(ts, t) + 1)),
            key=lambda j: abs(ts[j] - t))
    im = cv2.imread(os.path.join(d, '%06d.jpg' % fr[i][1]))
    def line(pts, col):
        p = [tuple(int(round(c)) for c in pose.board_px(x, y)) for x, y in pts]
        for a, b in zip(p, p[1:]):
            cv2.line(im, a, b, col, 1, cv2.LINE_AA)
    for off, col in ((0, (0, 0, 255)), (8, (0, 200, 0)), (-4, (255, 120, 0))):
        line([(off, y) for y in np.linspace(-10, 125, 30)], col)
        line([(x, off) for x in np.linspace(-10, 125, 30)], col)
    c = cv2.resize(im[y0:y1, x0:x1], None, fx=z, fy=z, interpolation=cv2.INTER_CUBIC)
    cv2.putText(c, lt(ts[i])[:10], (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(c, lt(ts[i])[:10], (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    tiles.append(c)
while len(tiles) % cols:
    tiles.append(np.zeros_like(tiles[0]))
rows_ = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
cv2.imwrite(out, np.vstack(rows_))
