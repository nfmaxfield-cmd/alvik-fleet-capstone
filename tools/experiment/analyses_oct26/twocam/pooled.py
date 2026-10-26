"""pooled.py : one side-camera model from all four sessions' robots (camera unmoved), each session at its own lag;
two-fold cross-check by alternating minutes; floor differences by region, split along / across the line of sight."""
import sys, os, pickle, json; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, pairs, calib
from model import Side, fit
LAG = {s: pickle.load(open('../work/cal_%s.pkl' % s, 'rb'))['delta'] for s in ['1510', '1545', '2114', '0955']}
CAMXY = np.array([45.4, -54.2])   # side camera's floor spot (roam1 grain calibration; used only for directions)
B = {s: pairs.load_blobs(s) for s in LAG}
def pairs_for(m, gate, fold=None, maxspeed=15.0):
    out = []
    for s in LAG:
        ok = None if fold is None else (lambda s_, f, fold=fold: int(B[s_][f]['t'] // 60) % 2 == fold)
        out += calib.assign([(s, B[s])], m, LAG[s], gate, maxspeed=maxspeed, frames_ok=ok)
    return out
def refit(m, fold=None, gates=(30, 20, 15, 12, 12)):
    for g in gates:
        P = pairs_for(m, g, fold); m = fit([(p['x'], p['y']) for p in P], [(p['u'], p['v']) for p in P], m)
    return m, P
def region(x, y):
    a = 'near' if y < 40 else ('middle' if y < 80 else 'far')
    b = 'left' if x < 40 else ('centre' if x < 80 else 'right')
    return a + ' ' + b
def edge(x, y, W=122.15, L=121.9, band=12.0):
    return min(x, y, W - x, L - y) < band
def floor_diff(m, P):
    uv = np.array([(p['u'], p['v']) for p in P]); xy = np.array([(p['x'], p['y']) for p in P])
    sxy = m.inv(uv); d = sxy - xy
    los = xy - CAMXY; los /= np.linalg.norm(los, axis=1)[:, None]
    along = (d * los).sum(1); across = d[:, 0] * (-los[:, 1]) + d[:, 1] * los[:, 0]
    return d, along, across
if __name__ == '__main__':
    m0 = pickle.load(open('../work/cal_2114.pkl', 'rb'))['m']
    m, P = refit(m0)
    e = np.linalg.norm(m.fwd([(p['x'], p['y']) for p in P]) - np.array([(p['u'], p['v']) for p in P]), axis=1)
    print('pooled: %d pairs, px residual median %.2f rms %.2f, kappa %.3g' % (len(P), np.median(e), np.sqrt(np.mean(e ** 2)), m.kap))
    mA, _ = refit(m, 0); mB, _ = refit(m, 1)
    T = []
    for mm, fold in [(mA, 1), (mB, 0)]:
        Q = pairs_for(mm, 1e9, fold)            # every pair in the other fold whose robot had a blob (nearest, no gate)
        d, al, ac = floor_diff(mm, Q)
        for q, dd, a1, a2 in zip(Q, d, al, ac):
            q.update(dx=dd[0], dy=dd[1], dist=float(np.hypot(*dd)), along=float(a1), across=float(a2), reg=region(q['x'], q['y']), edge=edge(q['x'], q['y']))
        T += Q
    pickle.dump(dict(m=m, P=P, T=T, mA=mA, mB=mB, lag=LAG), open('../work/pooled.pkl', 'wb'))
    dist = np.array([t['dist'] for t in T])
    print('held-out pairs %d; within 10 cm %.1f%%' % (len(T), 100 * np.mean(dist < 10)))
    k = dist < 10
    print('agreeing (<10 cm): median %.2f cm, 95%% %.2f cm' % (np.median(dist[k]), np.percentile(dist[k], 95)))
    al = np.array([t['along'] for t in T])[k]; ac = np.array([t['across'] for t in T])[k]
    print('along sight SD-like (IQR/1.35) %.2f, across %.2f' % (np.subtract(*np.percentile(al, [75, 25])) / 1.35, np.subtract(*np.percentile(ac, [75, 25])) / 1.35))
