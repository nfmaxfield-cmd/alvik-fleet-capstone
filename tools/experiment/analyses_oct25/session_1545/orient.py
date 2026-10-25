"""orient.py REF_FRAME X0,X1,Y0,Y1 FROM TO: orientation of the robot's dark body (second moments of the darkest
pixels inside the box) in BOARD degrees (mod 180), frame by frame, plus the unwrapped change."""
import sys, os, json, math
import numpy as np, cv2
S = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
cal = json.load(open('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json'))
K = np.array(cal['K']); rv = np.array(cal['rvec']); tv = np.array(cal['tvec']).reshape(3)
R, _ = cv2.Rodrigues(rv); Ki = np.linalg.inv(K)
def to_floor(u, v, z=3.7):
    d = R.T @ (Ki @ np.array([u, v, 1.0])); c = -R.T @ tv; s = (z - c[2]) / d[2]; p = c + s * d; return p[0], p[1]
def px(x, y):
    u, _ = cv2.projectPoints(np.array([[x, y, 0.0]]), rv, tv, K, np.zeros(5)); return u[0, 0]
times = dict((int(a), float(b)) for a, b in (l.strip().split(',') for l in open(os.path.join(S, 'frames', 'times.csv'))))
ref = cv2.GaussianBlur(cv2.imread(os.path.join(S, 'frames', '%06d.jpg' % int(sys.argv[1])), 0), (5, 5), 0).astype(np.float32)
x0, x1, y0, y1 = map(float, sys.argv[2].split(','))
poly = np.array([px(x0, y0), px(x1, y0), px(x1, y1), px(x0, y1)], dtype=np.int32)
mask = np.zeros(ref.shape, np.uint8); cv2.fillPoly(mask, [poly], 255)
prev = None; tot = 0.0
for k in sorted(times):
    if not int(sys.argv[3]) <= k <= int(sys.argv[4]): continue
    g = cv2.GaussianBlur(cv2.imread(os.path.join(S, 'frames', '%06d.jpg' % k), 0), (5, 5), 0).astype(np.float32)
    d = ((ref - g) > 35).astype(np.uint8) * (mask > 0)          # darker than the floor: the body
    ys, xs = np.nonzero(d)
    if len(xs) < 30: print(k, 'none'); continue
    # to board cm, then moments
    P = np.array([to_floor(u, v) for u, v in zip(xs, ys)])
    c = P.mean(0); Q = P - c; cov = Q.T @ Q / len(Q)
    ang = 0.5 * math.degrees(math.atan2(2 * cov[0, 1], cov[0, 0] - cov[1, 1]))
    if prev is not None:
        dd = ((ang - prev + 90) % 180) - 90; tot += dd
    prev = ang
    print(k, '%.2f' % times[k], 'centre %.1f %.1f' % tuple(c), 'axis %6.1f' % ang, 'turned %+6.1f' % tot, len(xs))
