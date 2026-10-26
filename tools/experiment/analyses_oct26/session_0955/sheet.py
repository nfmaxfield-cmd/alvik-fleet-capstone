"""sheet.py OUT X Y HALF ZOOM HH:MM:SS.s ... : overhead video crops around board point (X, Y) cm (roam1) at the given
times, with the stored open-edge lines (red, x = 0 and y = 0) and 4 cm outside them (blue), labelled, in one row.
Frames from getframes.py in W/fr."""
import sys, os, bisect, cv2, numpy as np
import pose
from common import W, ep, lt
FR = W + '/fr'
fr = sorted((float(t), int(k)) for k, t in (l.strip().split(',') for l in open(os.path.join(FR, 'times.csv'))))
ts = [f[0] for f in fr]
out, X, Y, H, Z = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), int(sys.argv[4]), float(sys.argv[5])
u0, v0 = pose.board_px(X, Y, 3.7)
tiles = []
for s in sys.argv[6:]:
    i = min(range(len(ts)), key=lambda j: abs(ts[j] - ep(s)))
    im = cv2.imread(os.path.join(FR, '%06d.jpg' % fr[i][1]))
    for off, col in ((0, (0, 0, 255)), (-4, (255, 120, 0))):
        for pts in ([(off, y) for y in np.linspace(-10, 125, 40)], [(x, off) for x in np.linspace(-10, 125, 40)]):
            p = [tuple(int(round(c)) for c in pose.board_px(a, b)) for a, b in pts]
            for a, b in zip(p, p[1:]):
                cv2.line(im, a, b, col, 1, cv2.LINE_AA)
    x0, y0 = int(u0 - H), int(v0 - H)
    pad = cv2.copyMakeBorder(im, H, H, H, H, cv2.BORDER_CONSTANT)
    c = pad[y0 + H:y0 + 3 * H, x0 + H:x0 + 3 * H]
    c = cv2.resize(c, None, fx=Z, fy=Z, interpolation=cv2.INTER_CUBIC)
    cv2.rectangle(c, (0, 0), (c.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(c, lt(ts[i])[:10], (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    tiles.append(c)
cv2.imwrite(out, np.hstack(tiles))
print(out)
