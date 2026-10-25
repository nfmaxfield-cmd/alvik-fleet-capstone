"""Brightness across an open edge on one frame, measured along the floor in stored-calibration cm."""
import sys, os, json
import numpy as np, cv2
S = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
cal = json.load(open('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json'))
K = np.array(cal['K']); rv = np.array(cal['rvec']); tv = np.array(cal['tvec'])
img = cv2.imread(os.path.join(S, 'frames', '%06d.jpg' % int(sys.argv[1])), cv2.IMREAD_GRAYSCALE).astype(np.float32)
img = cv2.GaussianBlur(img, (3, 3), 0)
axis = sys.argv[2]   # 'x' : profile along x (bottom edge) at fixed y values; 'y': along y (right edge) at fixed x
for c in map(float, sys.argv[3].split(',')):
    vs = np.arange(-12, 20.01, 0.5)
    P = np.array([[v, c, 0] if axis == 'x' else [c, v, 0] for v in vs], dtype=np.float64)
    u, _ = cv2.projectPoints(P, rv, tv, K, np.zeros(5)); u = u[:, 0, :]
    b = np.array([cv2.getRectSubPix(img, (1, 1), (float(a), float(bb)))[0, 0] for a, bb in u])
    d = np.diff(b)  # brightness increase going inward
    i = int(np.argmax(d))
    # half-way crossing between dark and bright
    lo, hi = np.percentile(b[vs < 0], 50), np.percentile(b[vs > 12], 50)
    j = np.where(b > (lo + hi) / 2)[0]
    print('%s=%5.1f strongest step at %5.1f cm (%+.0f); half-way %s; dark %.0f bright %.0f' % (
        'y' if axis == 'x' else 'x', c, (vs[i] + vs[i + 1]) / 2, d[i], ('%.1f' % vs[j[0]]) if len(j) else '-', lo, hi))
    if len(sys.argv) > 4: print('   ', ' '.join('%d' % v for v in b[::2]))
