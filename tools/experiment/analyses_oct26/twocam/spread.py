"""spread.py : how many robot spots, and how spread out, the robot-made calibration needs. Still robots' pairs
reduced to one median point per 10 cm cell ("spots"); random sets of N spots fit a plain homography; scored on
every pair of the pooled set (median floor disagreement with the overhead, cm)."""
import sys, os, pickle, json, collections; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, cv2
from scipy.spatial import ConvexHull
d = pickle.load(open('../work/pooled.pkl', 'rb')); P = d['P']; m = d['m']
still = [p for p in P if p['sp'] < 3.0]
cells = collections.defaultdict(list)
for p in still: cells[(int(p['x'] // 10), int(p['y'] // 10))].append(p)
spots = []
for k, v in cells.items():
    if len(v) >= 3:
        spots.append((np.median([p['x'] for p in v]), np.median([p['y'] for p in v]), np.median([p['u'] for p in v]), np.median([p['v'] for p in v])))
spots = np.array(spots); print('still pairs', len(still), 'spots', len(spots))
ev = np.array([(p['x'], p['y'], p['u'], p['v']) for p in P])
ev = ev[np.random.default_rng(0).choice(len(ev), 2000, replace=False)]
BOARD = 122.15 * 121.9
rng = np.random.default_rng(1); res = []
for N in [4, 5, 6, 8, 10, 15, 20, 30]:
    for _ in range(300):
        S = spots[rng.choice(len(spots), N, replace=False)]
        try: area = ConvexHull(S[:, :2]).volume / BOARD
        except Exception: continue
        H, _ = cv2.findHomography(S[:, :2], S[:, 2:], 0)
        if H is None: continue
        Hi = np.linalg.inv(H)
        q = cv2.perspectiveTransform(ev[:, 2:].reshape(-1, 1, 2), Hi).reshape(-1, 2)
        e = np.hypot(*(q - ev[:, :2]).T)
        res.append((N, area, float(np.median(e)), float(np.percentile(e, 95))))
res = np.array(res); pickle.dump(dict(res=res, spots=spots), open('../work/spread.pkl', 'wb'))
# reference: the full model on the same pairs
q = m.inv(ev[:, 2:]); e = np.hypot(*(q - ev[:, :2]).T); print('full model on these pairs: median %.2f p95 %.2f' % (np.median(e), np.percentile(e, 95)))
for N in [4, 5, 6, 8, 10, 15, 20, 30]:
    r = res[res[:, 0] == N]
    for lo, hi in [(0, .25), (.25, .5), (.5, 1.01)]:
        k = (r[:, 1] >= lo) & (r[:, 1] < hi)
        if k.sum() >= 5:
            print('N %2d spread %.2f-%.2f: draws %3d median-of-medians %.2f cm, 90th pct %.2f' % (N, lo, hi, k.sum(), np.median(r[k, 2]), np.percentile(r[k, 2], 90)))
