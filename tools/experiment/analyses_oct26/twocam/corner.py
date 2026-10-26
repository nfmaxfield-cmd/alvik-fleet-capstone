import sys, os, pickle, csv; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common2 import *
import sess, vid, pairs
d = pickle.load(open('../work/pooled.pkl', 'rb')); m = d['m']
R1 = CAM.load_camera(REPO + '/tools/experiment/data/side_floorcal_roam1.json')
rows = {round(float(r['t']), 1): r for r in csv.DictReader(open('../work/ev_2114left.csv'))}
cols = []
for h in ['21:42:30', '21:42:47', '21:45:05', '21:45:40', '21:46:40']:
    t = sess.ep('2114', h); f1, t1 = sess.frame_at('2114', 'cam1', t)
    img = dec(vid.frames(sess.S['2114'][0], 'cam1', [f1])[f1])
    for a_, b_ in [((0, -10, 0), (0, 60, 0)), ((-10, 0, 0), (60, 0, 0))]:
        p = [R1.project([a_[0] + (b_[0] - a_[0]) * s, a_[1] + (b_[1] - a_[1]) * s, 0]) for s in np.linspace(0, 1, 30)]
        cv2.polylines(img, [np.array(p, np.int32)], False, (0, 220, 255), 1)
    r = min(rows.values(), key=lambda r: abs(float(r['t']) - t))
    u, v = m.fwd([(float(r['track_x']), float(r['track_y']))])[0]
    cv2.circle(img, (int(u), int(v)), 14, (0, 0, 255), 2)
    cu, cv_ = R1.project([5, 0, 0])
    c = cv2.copyMakeBorder(img, 150, 150, 150, 150, cv2.BORDER_CONSTANT)[int(cv_) + 150 - 70:int(cv_) + 150 + 70, int(cu) + 150 - 90:int(cu) + 150 + 90]
    c = cv2.resize(c, None, fx=1.6, fy=1.6); cv2.putText(c, h, (5, 18), 0, 0.55, (0, 255, 255), 2)
    cols.append(c)
cv2.imwrite('../figs/twocam-2114-corner-crops.jpg', np.hstack(cols))
