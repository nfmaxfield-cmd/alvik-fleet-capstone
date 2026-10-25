"""edgecrop.py X Y HALFPX SCALE OUT FRAME...: raw frames around (X, Y) with the real bottom edge (x = 4.7, red) and
lines every 5 cm (x = -5, 0 green dashed = stored line, 5, 10, 15) at floor level; no runner marks."""
import sys, os, json
import numpy as np, cv2
S = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
cal = json.load(open('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json'))
K = np.array(cal['K']); rv = np.array(cal['rvec']); tv = np.array(cal['tvec'])
def px(x, y):
    u, _ = cv2.projectPoints(np.array([[x, y, 0.0]]), rv, tv, K, np.zeros(5)); return u[0, 0]
X, Y, h, sc, out = float(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
times = dict((int(a), float(b)) for a, b in (l.strip().split(',') for l in open(os.path.join(S, 'frames', 'times.csv'))))
c = px(X, Y); tiles = []
for k in map(int, sys.argv[6:]):
    img = cv2.imread(os.path.join(S, 'frames', '%06d.jpg' % k))
    x0, y0 = int(c[0]) - h, int(c[1]) - h
    crop = cv2.resize(img[y0:y0 + 2 * h, x0:x0 + 2 * h], None, fx=sc, fy=sc, interpolation=cv2.INTER_CUBIC)
    for xv, col in ((4.7, (0, 0, 255)), (0, (0, 200, 0)), (10, (160, 160, 160)), (15, (160, 160, 160)), (8, (0, 200, 255))):
        a = (px(xv, Y - 30) - [x0, y0]) * sc; b = (px(xv, Y + 30) - [x0, y0]) * sc
        cv2.line(crop, tuple(int(v) for v in a), tuple(int(v) for v in b), col, 1)
    import time
    cv2.putText(crop, time.strftime('%H:%M:%S', time.gmtime(times[k] - 14400)) + '.%d' % int(times[k] % 1 * 10), (4, 14), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), 1)
    tiles.append(crop)
cv2.imwrite(out, np.hstack(tiles))
