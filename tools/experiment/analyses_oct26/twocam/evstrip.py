"""evstrip.py SESSION ROBOT OUT HH:MM:SS ... : side crops around the overhead's position (pooled robot-made model)
and overhead crops, one column per moment."""
import sys, os, pickle; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common2 import *
import sess, vid, pairs
s, robot, outp = sys.argv[1:4]; hms = sys.argv[4:]
d = pickle.load(open('../work/pooled.pkl', 'rb')); m = d['m']; lag = d['lag'][s]
a = pairs.track_r1(s)[robot]; cols = []
for h in hms:
    t = sess.ep(s, h)
    f0, _ = sess.frame_at(s, 'cam0', t); f1, t1 = sess.frame_at(s, 'cam1', t)
    o = dec(vid.frames(sess.S[s][0], 'cam0', [f0])[f0]); sd = dec(vid.frames(sess.S[s][0], 'cam1', [f1])[f1])
    q = pairs.interp(a, t1 + lag, 2.0)
    if q is None:
        i = np.argmin(abs(a[:, 0] - t)); x, y = a[i, 1:]
    else: x, y = q[0]
    u, v = m.fwd([(x, y)])[0]; ou, ov = OCAM.project([x, y, CAM.ROBOT_TOP_CM])
    cv2.circle(sd, (int(u), int(v)), 16, (0, 0, 255), 2); cv2.circle(o, (int(ou), int(ov)), 9, (0, 0, 255), 2)
    def crop(img, cu, cv_, w, hh, sc):
        img = cv2.copyMakeBorder(img, 200, 200, 200, 200, cv2.BORDER_CONSTANT)
        c = img[int(cv_) + 200 - hh:int(cv_) + 200 + hh, int(cu) + 200 - w:int(cu) + 200 + w]; return cv2.resize(c, None, fx=sc, fy=sc)
    A = crop(sd, u, v, 90, 70, 1.5); Bc = crop(o, ou, ov, 45, 35, 3.0)
    A = cv2.resize(A, (270, 210)); Bc = cv2.resize(Bc, (270, 210))
    col = np.vstack([A, Bc]); cv2.putText(col, h, (5, 18), 0, 0.55, (0, 255, 255), 2)
    cols.append(col); print(h, 'track', round(x, 1), round(y, 1))
cv2.imwrite(outp, np.hstack(cols))
