"""extract.py SESSION : side-view blobs every 0.4 s over every minute the overhead tracked a robot -> ../work/blobs_SESSION.csv"""
import sys, os, time, csv; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common2 import *
import sess, vid, det
s = sys.argv[1]; tag = sess.S[s][0]
END = {'2114': sess.ep('2114', '23:00:00')}
os.makedirs('../work', exist_ok=True)
nidx = len(vid.index(tag, 'cam1'))
f1 = {x[0]: x for x in sess.ftimes(s, 'cam1')}
tt = sorted(x[0] for v in sess.track(s).values() for x in v if x[3] != 'held')
tt = [t for t in tt if t < END.get(s, 1e12)]
wins = sorted(set(int(t // 60) for t in tt))
P = '../work/blobs_%s.csv' % s
last = -1
if os.path.exists(P):
    for line in open(P):
        try: last = max(last, int(line.split(',')[0]))
        except ValueError: pass
out = open(P, 'a'); w = csv.writer(out)
if last < 0: w.writerow(['frame', 't_wall', 't_mono', 'k', 'u', 'v', 'a', 'x', 'y', 'w', 'h'])
tproc = []
for wi in wins:
    ts = np.arange(wi * 60, wi * 60 + 60, 0.4)
    have = np.searchsorted(tt, ts)
    near = [min(abs(tt[min(i, len(tt) - 1)] - t), abs(tt[max(i - 1, 0)] - t)) for i, t in zip(have, ts)]
    if np.mean(np.array(near) < 0.3) < 0.2: continue
    fs = sorted(set(sess.frame_at(s, 'cam1', t)[0] for t in ts))
    fs = [f for f in fs if f < nidx]
    if fs and fs[-1] <= last: continue
    if len(fs) < 50: continue
    fr = vid.frames(tag, 'cam1', fs)
    imgs = {f: dec(fr[f]) for f in fs if f in fr}
    imgs = {f: i for f, i in imgs.items() if i is not None and i.shape == (480, 640, 3)}
    fs = sorted(imgs)
    bg = det.background([imgs[f] for f in fs])
    for f in fs:
        c = time.perf_counter(); b, _ = det.blobs(imgs[f], bg); tproc.append(time.perf_counter() - c)
        r = f1[f]
        for k, q in enumerate(b):
            w.writerow([f, '%.3f' % r[1], '%.4f' % r[2], k, '%.1f' % q['u'], '%.1f' % q['v'], q['a'], q['x'], q['y'], q['w'], q['h']])
        if not b: w.writerow([f, '%.3f' % r[1], '%.4f' % r[2], -1, '', '', '', '', '', '', ''])
    out.flush()
    print(s, sess.lt(wi * 60), len(fs), 'blob ms %.1f' % (1000 * np.median(tproc)), flush=True)
print('DONE', s, len(tproc), 'frames, blob step median ms %.2f' % (1000 * np.median(tproc)))
