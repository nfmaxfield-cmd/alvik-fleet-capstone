"""bench.py : processing cost per picture on this machine. (1) the runner's MotionTracker on overhead pictures, idle and
following one robot, to scale against the Pi's own measured cost (0955 report: 26-29 ms idle, 73 ms following);
(2) side-picture confirmation: full-picture decode + blob step, and a cheap variant (half-size decode + a crop
around the expected spot compared with a reference)."""
import sys, os, time, json, pickle; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common2 import *
import sess, vid, det
s = '0955'; tag = sess.S[s][0]
t0 = sess.ep(s, '10:06:20')
f0 = [sess.frame_at(s, 'cam0', t0 + k * 0.04)[0] for k in range(250)]
f1 = [sess.frame_at(s, 'cam1', t0 + k * 0.0667)[0] for k in range(150)]
J0 = vid.frames(tag, 'cam0', f0); J1 = vid.frames(tag, 'cam1', f1)
J0 = [J0[f] for f in sorted(set(f0))]; J1 = [J1[f] for f in sorted(set(f1))]
def tm(fn, items, rep=1):
    a = time.perf_counter()
    for _ in range(rep):
        for x in items: fn(x)
    return 1000 * (time.perf_counter() - a) / (len(items) * rep)
res = {}
I0 = [cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_GRAYSCALE) for j in J0]
for follow in [False, True]:
    tr = CAM.MotionTracker(OCAM)
    if follow: tr.follow('Center', 100.0, 60.0, t0)
    k = [0]
    def push(img):
        tr.push(img, t0 + k[0] * 0.04); k[0] += 1
    for img in I0[:30]: push(img)
    res['tracker_follow' if follow else 'tracker_idle'] = tm(push, I0[30:])
res['decode_full'] = tm(dec, J1)
bg = det.background([dec(j) for j in J1])
res['decode+blobs_full'] = tm(lambda j: det.blobs(dec(j), bg), J1)
bgh = cv2.resize(bg, (320, 240)); G = cv2.cvtColor(bgh, cv2.COLOR_BGR2GRAY)
def cheap(j, cu=200, cv_=150, w=40, h=30):
    im = cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_REDUCED_GRAYSCALE_2)
    c = im[cv_ - h:cv_ + h, cu - w:cu + w]; r = G[cv_ - h:cv_ + h, cu - w:cu + w]
    m = (cv2.absdiff(c, r) > 28).astype(np.uint8); n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    return cen
res['cheap_confirm'] = tm(cheap, J1, rep=3)
print(json.dumps({k: round(v, 2) for k, v in res.items()}))
json.dump(res, open('../work/bench.json', 'w'))
