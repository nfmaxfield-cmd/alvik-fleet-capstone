"""fit.py : side camera floor homography from the robots (overhead positions paired with side blobs)."""
import sys, os, json, collections; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, cv2, sess, pairs
W = os.path.join(os.path.dirname(sess.DL), 'work')
def ransac(rows, thr=8.0):
    if len(rows) < 8: return None, None
    A = np.array([(r[2], r[3]) for r in rows], np.float64); B = np.array([(r[5], r[6]) for r in rows], np.float64)
    H, m = cv2.findHomography(A, B, cv2.RANSAC, thr, maxIters=20000, confidence=0.999)
    return H, (m.ravel() > 0 if m is not None else None)
def proj(H, xy):
    p = cv2.perspectiveTransform(np.asarray(xy, np.float64).reshape(-1, 1, 2), H).reshape(-1, 2); return p
def best_delta(s, blobs, grid=np.arange(-0.6, 0.61, 0.05)):
    out = []
    for d in grid:
        rows = pairs.candidates(s, blobs, d, maxspeed=40)
        mov = [r for r in rows if r[4] > 3]   # moving robots carry the timing information
        H, m = ransac(rows)
        if H is None: continue
        A = np.array([(r[2], r[3]) for r in mov]); B = np.array([(r[5], r[6]) for r in mov])
        e = np.linalg.norm(proj(H, A) - B, axis=1)
        out.append((d, int(m.sum()), float(np.median(e[e < 8])), int((e < 8).sum())))
    return out
if __name__ == '__main__':
    for s in sys.argv[1:]:
        b = pairs.load_blobs(s)
        print(s, len(b), 'frames')
        for d, n, med, nm in best_delta(s, b):
            print('  delta %+.2f inliers %5d moving-inlier median px %.2f n %d' % (d, n, med, nm))
