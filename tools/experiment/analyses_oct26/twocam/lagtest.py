"""lagtest.py : camera-to-camera picture lag with no tracker. Dense frames from both cameras over stretches where a
robot drives (track speed > 6 cm/s for 4 s); one blob per picture near the robot, to floor with each camera's own
calibration (roam1 overhead, roam1 side grain); lag = shift that best lines up the two paths (a constant offset
allowed)."""
import sys, os, pickle; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common2 import *
import sess, vid, det, pairs
R1 = CAM.load_camera(REPO + '/tools/experiment/data/side_floorcal_roam1.json')
def stretches(s, vmin=6.0, n=6):
    tk = pairs.track_r1(s); out = []
    for r, a in tk.items():
        for i in range(0, len(a) - 50, 10):
            seg = a[i:i + 50]; T = seg[-1, 0] - seg[0, 0]
            if not 3.5 < T < 6: continue
            v = np.hypot(*(seg[-1, 1:] - seg[0, 1:])) / T
            if v > vmin: out.append((v, r, seg[0, 0], seg[-1, 0]))
    out.sort(reverse=True); sel = []
    for o in out:
        if all(abs(o[2] - x[2]) > 30 for x in sel): sel.append(o)
    return sel[:n]
def path(s, cam, t0, t1, model, z, robot=None):
    tag = sess.S[s][0]; fts = sess.ftimes(s, cam)
    ts = np.array([x[1] for x in fts]); fn = np.array([x[0] for x in fts])
    dense = fn[(ts >= t0) & (ts <= t1)]
    bgf = fn[(ts >= t0 - 30) & (ts <= t1 + 30)][::max(1, int(len(fn[(ts >= t0 - 30) & (ts <= t1 + 30)]) / 40))]
    fr = vid.frames(tag, cam, list(dense) + list(bgf))
    im = {f: dec(v) for f, v in fr.items()}; im = {f: i for f, i in im.items() if i is not None and i.shape == (480, 640, 3)}
    if not [f for f in bgf if f in im]: return np.zeros((0, 3))
    bg = det.background([im[f] for f in bgf if f in im])
    arr = pairs.track_r1(s)[robot]
    tmap = dict(zip(fn, ts)); out = []
    for f in dense:
        if f not in im: continue
        b, _ = det.blobs(im[f], bg)
        b = [q for q in b if q['a'] >= 200]
        if not b: continue
        g = pairs.interp(arr, tmap[f], 1.0)
        if g is None: continue
        P = [np.array(model(q['u'], q['v'], z)) for q in b]
        dd = [np.hypot(*(p - g[0])) for p in P]; k = int(np.argmin(dd))
        if dd[k] < 15: out.append((tmap[f],) + tuple(P[k]))
    return np.array(out)
if __name__ == '__main__':
    res = []
    for s in sys.argv[1:]:
        for v, r, t0, t1 in stretches(s):
            o = path(s, 'cam0', t0, t1, OCAM.to_floor, CAM.ROBOT_TOP_CM, r); sd = path(s, 'cam1', t0, t1, R1.to_floor, 1.85, r)
            if len(o) < 30 or len(sd) < 20: continue
            best = None
            for d in np.arange(-0.8, 0.81, 0.02):
                tt = o[:, 0] + d; ok = (tt > sd[0, 0]) & (tt < sd[-1, 0])
                if ok.sum() < 20: continue
                sx = np.interp(tt[ok], sd[:, 0], sd[:, 1]); sy = np.interp(tt[ok], sd[:, 0], sd[:, 2])
                ex = o[ok, 1] - sx; ey = o[ok, 2] - sy
                rms = np.sqrt(np.mean((ex - np.median(ex)) ** 2 + (ey - np.median(ey)) ** 2))
                if best is None or rms < best[1]: best = (d, rms, np.median(ex), np.median(ey))
            print(s, r, sess.lt(t0), 'speed %.1f' % v, 'side minus overhead lag %+.2f s, rms %.2f cm, offset %.1f,%.1f' % best, flush=True)
            res.append((s, r, t0, v) + best)
    pickle.dump(res, open('../work/lagtest_%s.pkl' % '_'.join(sys.argv[1:]), 'wb'))
