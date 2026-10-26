"""Did either camera move between sessions? Median of 15 pictures from each session's first minutes, matched by ORB
features (robots excluded by the median and RANSAC); shift = median feature displacement and a fitted homography."""
import sys, os, json; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common2 import *
import sess, vid
med = {}
for s in sess.S:
    for cam in ['cam0', 'cam1']:
        n = len(vid.index(sess.S[s][0], cam)); step = 250 if cam == 'cam1' else 400
        fs = list(range(30, min(n, 30 + 15 * step), step))
        fr = vid.frames(sess.S[s][0], cam, fs, gap=1e5)
        med[(s, cam)] = np.median(np.stack([i for i in (dec(fr[f]) for f in fs) if i is not None and i.shape == (480, 640, 3)]), axis=0).astype(np.uint8)
        cv2.imwrite('../work/med_%s_%s.jpg' % (s, cam), med[(s, cam)])
orb = cv2.ORB_create(3000)
ref = '0955'
res = {}
for cam in ['cam0', 'cam1']:
    g0 = cv2.cvtColor(med[(ref, cam)], cv2.COLOR_BGR2GRAY); k0, d0 = orb.detectAndCompute(g0, None)
    for s in sess.S:
        if s == ref: continue
        g = cv2.cvtColor(med[(s, cam)], cv2.COLOR_BGR2GRAY); k, d = orb.detectAndCompute(g, None)
        m = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(d0, d)
        a = np.float32([k0[x.queryIdx].pt for x in m]); b = np.float32([k[x.trainIdx].pt for x in m])
        H, inl = cv2.findHomography(a, b, cv2.RANSAC, 2.0); inl = inl.ravel() > 0
        disp = np.linalg.norm(b[inl] - a[inl], axis=1)
        grid = np.float32([[u, v] for u in range(40, 640, 80) for v in range(40, 480, 80)]).reshape(-1, 1, 2)
        gm = np.linalg.norm(cv2.perspectiveTransform(grid, H) - grid, axis=2).ravel()
        res['%s %s' % (cam, s)] = dict(inliers=int(inl.sum()), feat_disp_med=float(np.median(disp)), feat_disp_p90=float(np.percentile(disp, 90)), grid_shift_max=float(gm.max()), grid_shift_med=float(np.median(gm)))
        print(cam, s, 'vs', ref, {k: round(v, 2) for k, v in res['%s %s' % (cam, s)].items()})
json.dump(res, open('../work/moved.json', 'w'), indent=1)
