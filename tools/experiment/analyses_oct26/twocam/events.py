"""events.py SESSION ROBOT T0 T1 STEP BG0 BG1 NAME : the side view over a moment the overhead went wrong. Background =
median of side pictures between BG0 and BG1 (a stretch when the robot's spot was empty); each picture's blobs to floor
with the pooled robot-made model; the blob nearest the robot's last side position (seeded from the overhead) is kept."""
import sys, os, pickle, csv; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common2 import *
import sess, vid, det, pairs
s, robot, T0, T1, step, BG0, BG1, name = sys.argv[1:9]; step = float(step)
d = pickle.load(open('../work/pooled.pkl', 'rb')); m = d['m']; lag = d['lag'][s]
R1 = CAM.load_camera(REPO + '/tools/experiment/data/side_floorcal_roam1.json')
tag = sess.S[s][0]
bgf = sorted(set(sess.frame_at(s, 'cam1', t)[0] for t in np.linspace(sess.ep(s, BG0), sess.ep(s, BG1), 40)))
fr = vid.frames(tag, 'cam1', bgf); bg = det.background([i for i in (dec(v) for v in fr.values()) if i is not None])
cv2.imwrite('../work/bg_%s.jpg' % name, bg)
ts = np.arange(sess.ep(s, T0), sess.ep(s, T1), step)
fs = [sess.frame_at(s, 'cam1', t) for t in ts]
fr = vid.frames(tag, 'cam1', [f for f, _ in fs])
a = pairs.track_r1(s)[robot]; last = None; rows = []
for f, t in fs:
    img = dec(fr[f]) if f in fr else None
    if img is None: continue
    b, _ = det.blobs(img, bg); b = [q for q in b if q['a'] >= 250]
    i = np.searchsorted(a[:, 0], t + lag); i = min(max(i, 1), len(a) - 1)
    tr = a[i - 1, 1:] if abs(a[i - 1, 0] - t - lag) < abs(a[i, 0] - t - lag) else a[i, 1:]
    tr_age = float(min(abs(a[i - 1, 0] - t - lag), abs(a[i, 0] - t - lag)))
    ref = last if last is not None else tr
    best = None
    for q in b:
        p = m.inv([(q['u'], q['v'])])[0]; dd = np.hypot(*(p - ref))
        if dd < 25 and (best is None or dd < best[0]): best = (dd, p, q)
    if best:
        last = best[1]; q = best[2]; pr = R1.to_floor(q['u'], q['v'], 1.85)
        rows.append([t, f, tr[0], tr[1], tr_age, best[1][0], best[1][1], pr[0], pr[1], float(np.hypot(*(best[1] - tr))), q['a']])
    else:
        rows.append([t, f, tr[0], tr[1], tr_age, '', '', '', '', '', ''])
w = csv.writer(open('../work/ev_%s.csv' % name, 'w'))
w.writerow(['t', 'frame', 'track_x', 'track_y', 'track_age_s', 'side_x', 'side_y', 'side_indep_x', 'side_indep_y', 'dist_cm', 'blob_px'])
w.writerows(rows)
seen = [r for r in rows if r[9] != '']
print(name, len(rows), 'pictures, robot found in', len(seen))
for r in rows[::max(1, len(rows) // 25)]:
    print(sess.lt(r[0]), 'track %.1f,%.1f age %.1f' % (r[2], r[3], r[4]), ('side %.1f,%.1f dist %.1f' % (r[5], r[6], r[9])) if r[9] != '' else 'side -')
