"""indep.py SESSION : the overhead tracker against the side camera calibrated independently (roam1 plywood-grain
calibration, camera checked unmoved). Lag scan on moving robots, offsets of still robots."""
import sys, os, pickle; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common2 import *
import pairs
R1 = CAM.load_camera(REPO + '/tools/experiment/data/side_floorcal_roam1.json')
ZC = 1.85
def side_floor(b):
    out = {}
    for f, e in b.items():
        out[f] = [(R1.to_floor(q[0], q[1], ZC), q) for q in e['b'] if q[2] >= 200]
    return out
def match(s, b, sf, d, gate=12.0, minspeed=None, maxspeed=None):
    tk = pairs.track_r1(s); rows = []
    for f, e in b.items():
        if not sf[f]: continue
        S = np.array([p for p, q in sf[f]])
        for r, a in tk.items():
            q = pairs.interp(a, e['t'] + d)
            if q is None: continue
            (x, y), sp = q
            if minspeed is not None and not sp >= minspeed: continue
            if maxspeed is not None and not sp <= maxspeed: continue
            dd = np.hypot(S[:, 0] - x, S[:, 1] - y); k = int(np.argmin(dd))
            if dd[k] <= gate:
                rows.append(dict(s=s, f=f, r=r, t=e['t'], x=x, y=y, sp=sp, sx=S[k, 0], sy=S[k, 1], u=sf[f][k][1][0], v=sf[f][k][1][1], a=sf[f][k][1][2], nb=len(S)))
    return rows
if __name__ == '__main__':
    s = sys.argv[1]; b = pairs.load_blobs(s); sf = side_floor(b)
    best = None
    for d in np.arange(-0.8, 0.31, 0.05):
        R = match(s, b, sf, d, minspeed=6)
        e = np.array([np.hypot(r['x'] - r['sx'], r['y'] - r['sy']) for r in R])
        print('lag %+.2f moving n %d median %.2f cm' % (d, len(e), np.median(e)))
        if best is None or np.median(e) < best[1]: best = (d, np.median(e))
    d = best[0]
    R = match(s, b, sf, d)
    pickle.dump(dict(lag=d, rows=R), open('../work/indep_%s.pkl' % s, 'wb'))
    st = [r for r in R if r['sp'] < 1.5]
    dx = np.array([r['x'] - r['sx'] for r in st]); dy = np.array([r['y'] - r['sy'] for r in st])
    print('DONE lag %.2f; still pairs %d: overhead minus side median dx %.2f dy %.2f, dist median %.2f' % (d, len(st), np.median(dx), np.median(dy), np.median(np.hypot(dx, dy))))
