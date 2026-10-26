"""calib.py SESSION... : robots calibrate the side camera. RANSAC start, then nearest-blob assignment and a robust
fit of homography + lens bend, repeated with a shrinking gate."""
import sys, os, json, pickle; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, cv2, sess, pairs
from model import Side, fit
from fit import ransac
def assign(sessions, m, delta, gate_px, maxspeed=None, minspeed=None, frames_ok=None):
    """one pair per (frame, robot): its projected spot's nearest blob within gate_px"""
    out = []
    for s, b in sessions:
        tk = pairs.track_r1(s)
        for f, e in b.items():
            if frames_ok is not None and not frames_ok(s, f): continue
            if not e['b']: continue
            B = np.array([q[:2] for q in e['b']])
            for r, a in tk.items():
                q = pairs.interp(a, e['t'] + delta)
                if q is None: continue
                (x, y), sp = q
                if maxspeed is not None and not sp <= maxspeed: continue
                if minspeed is not None and not sp >= minspeed: continue
                p = m.fwd([(x, y)])[0]; d = np.linalg.norm(B - p, axis=1); k = int(np.argmin(d))
                if d[k] <= gate_px:
                    out.append(dict(s=s, f=f, r=r, t=e['t'], x=x, y=y, sp=sp, u=B[k, 0], v=B[k, 1], a=e['b'][k][2], n=len(B)))
    return out
def calibrate(sessions, delta=0.0, maxspeed=15.0, log=print, frames_ok=None):
    rows = []
    for s, b in sessions:
        rows += [r for r in pairs.candidates(s, {f: e for f, e in b.items() if frames_ok is None or frames_ok(s, f)}, delta, maxspeed=maxspeed)]
    H, msk = ransac(rows, 12.0)
    m = Side(H, 0.0); log('ransac: %d candidates, %d inliers' % (len(rows), msk.sum()))
    for gate in [40, 30, 20, 15, 12, 12]:
        P = assign(sessions, m, delta, gate, maxspeed=maxspeed, frames_ok=frames_ok)
        xy = [(p['x'], p['y']) for p in P]; uv = [(p['u'], p['v']) for p in P]
        m = fit(xy, uv, m)
        e = np.linalg.norm(m.fwd(xy) - np.array(uv), axis=1)
        log('gate %d px: %d pairs, residual median %.2f px, rms(<gate) %.2f, kappa %.3g' % (gate, len(P), np.median(e), np.sqrt(np.mean(e[e < gate] ** 2)), m.kap))
    return m, P
if __name__ == '__main__':
    ss = [(s, pairs.load_blobs(s)) for s in sys.argv[1:]]
    m, P = calibrate(ss)
    pickle.dump(m, open('../work/side_%s.pkl' % '_'.join(sys.argv[1:]), 'wb'))
