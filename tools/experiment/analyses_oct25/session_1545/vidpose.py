"""vidpose.py REF_FRAME X0,X1,Y0,Y1 FRAME_FROM FRAME_TO : the robot's centre measured from the video: difference
against a clean reference frame inside a board-cm box, largest blob, its centre projected to the robot-top plane.
Prints frame, time, x, y (stored-calibration cm), blob area px."""
import sys, os, json
import numpy as np, cv2
S = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
cal = json.load(open('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json'))
K = np.array(cal['K']); rv = np.array(cal['rvec']); tv = np.array(cal['tvec']).reshape(3)
R, _ = cv2.Rodrigues(rv); Ki = np.linalg.inv(K)
def to_floor(u, v, z=3.7):
    d = R.T @ (Ki @ np.array([u, v, 1.0])); c = -R.T @ tv
    s = (z - c[2]) / d[2]; p = c + s * d; return p[0], p[1]
def px(x, y, z=0.0):
    u, _ = cv2.projectPoints(np.array([[x, y, z]], dtype=np.float64), rv, tv, K, np.zeros(5)); return u[0, 0]
times = dict((int(a), float(b)) for a, b in (l.strip().split(',') for l in open(os.path.join(S, 'frames', 'times.csv'))))
ref = cv2.GaussianBlur(cv2.imread(os.path.join(S, 'frames', '%06d.jpg' % int(sys.argv[1])), 0), (5, 5), 0).astype(np.float32)
x0, x1, y0, y1 = map(float, sys.argv[2].split(','))
poly = np.array([px(x0, y0), px(x1, y0), px(x1, y1), px(x0, y1)], dtype=np.int32)
mask = np.zeros(ref.shape, np.uint8); cv2.fillPoly(mask, [poly], 255)
a, b = int(sys.argv[3]), int(sys.argv[4])
for k in sorted(times):
    if not a <= k <= b: continue
    g = cv2.GaussianBlur(cv2.imread(os.path.join(S, 'frames', '%06d.jpg' % k), 0), (5, 5), 0).astype(np.float32)
    g = g - (np.median(g[mask > 0]) - np.median(ref[mask > 0]))       # overall brightness change
    d = (np.abs(g - ref) > 28).astype(np.uint8) * (mask > 0)
    d = cv2.morphologyEx(d, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    d = cv2.morphologyEx(d, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(d)
    if n <= 1: print(k, '%.2f' % times[k], 'none'); continue
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    x, y = to_floor(*cen[i])
    print(k, '%.2f' % times[k], '%.1f %.1f' % (x, y), st[i, cv2.CC_STAT_AREA])
