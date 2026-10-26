"""ohcheck.py SESSION NWIN : the overhead tracker against its own pictures. Robot blobs in cam0 (window median
background), centroid to floor on the robot-top plane with the roam1 calibration, paired with track.csv; lag scan."""
import sys, os, pickle; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common2 import *
import sess, vid, det, pairs
s = sys.argv[1]; nw = int(sys.argv[2]); tag = sess.S[s][0]
tk = pairs.track_r1(s)
tt = np.sort(np.concatenate([a[:, 0] for a in tk.values()]))
wins = sorted(set((tt // 60).astype(int)))
cnt = {w: np.sum((tt // 60).astype(int) == w) for w in wins}
wins = sorted(wins, key=lambda w: -cnt[w])[:nw]
recs = []
for wi in wins:
    ts = np.arange(wi * 60, wi * 60 + 60, 0.4)
    fs = sorted(set(sess.frame_at(s, 'cam0', t)[0] for t in ts))
    fr = vid.frames(tag, 'cam0', fs)
    im = {f: dec(fr[f]) for f in fs if f in fr}; im = {f: i for f, i in im.items() if i is not None and i.shape == (480, 640, 3)}
    bg = det.background([im[f] for f in sorted(im)])
    f0 = {x[0]: x for x in sess.ftimes(s, 'cam0')}
    for f in sorted(im):
        b, _ = det.blobs(im[f], bg)
        for q in b:
            if q['a'] < 150: continue
            x, y = OCAM.to_floor(q['u'], q['v'], CAM.ROBOT_TOP_CM)
            recs.append((f0[f][1], x, y, q['a']))
pickle.dump(recs, open('../work/oh_%s.pkl' % s, 'wb'))
def match(d, minspeed=None):
    e = []
    for t, x, y, a in recs:
        best = None
        for r, arr in tk.items():
            q = pairs.interp(arr, t + d)
            if q is None: continue
            (X, Y), sp = q
            if minspeed is not None and not sp >= minspeed: continue
            dd = np.hypot(X - x, Y - y)
            if dd < 12 and (best is None or dd < best[0]): best = (dd, X - x, Y - y)
        if best: e.append(best)
    return np.array(e)
for d in np.arange(-0.8, 0.21, 0.05):
    e = match(d, 6)
    if len(e): print('lag %+.1f moving n %d median dist %.2f cm' % (d, len(e), np.median(e[:, 0])))
e = match(0.0, None); e2 = match(-0.4, None)
print('all, lag 0: n %d median dist %.2f, mean offset track-blob x %.2f y %.2f' % (len(e), np.median(e[:, 0]), np.median(e[:, 1]), np.median(e[:, 2])))
