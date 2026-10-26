"""blob.py DIR TIME... : the robot's body near the bottom-right corner by difference from the empty board (21:15:00
frame 1801): the extent of the changed pixels (dark body, not the bright glow of its lights) in board cm at floor level
and at the top's height."""
import sys, os, bisect, cv2, numpy as np
import pose, e02_explore as EX
from common import ep, lt
d = sys.argv[1]
fr = sorted((float(t), int(k)) for k, t in (l.strip().split(',') for l in open(os.path.join(d, 'times.csv'))))
ts = [f[0] for f in fr]
bg = cv2.cvtColor(cv2.imread(os.path.join(d, '001801.jpg')), cv2.COLOR_BGR2GRAY).astype(float)
x0, x1, y0, y1 = 360, 430, 290, 360
for s in sys.argv[2:]:
    t = ep(s)
    i = min(range(len(ts)), key=lambda j: abs(ts[j] - t))
    im = cv2.cvtColor(cv2.imread(os.path.join(d, '%06d.jpg' % fr[i][1])), cv2.COLOR_BGR2GRAY).astype(float)
    diff = (bg - im)[y0:y1, x0:x1]          # darker than the empty board: the body
    m = (diff > 25).astype(np.uint8)
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    if n < 2:
        print(s, 'nothing'); continue
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(lab == k)
    pts = [pose.ROAM.to_floor(float(u + x0), float(v + y0), EX.ROBOT_TOP_CM) for u, v in zip(xs, ys)]
    bx = np.array([float(p[0]) for p in pts]); by = np.array([float(p[1]) for p in pts])
    print('%s area %d px; top-height board x %.1f..%.1f (centre %.1f), y %.1f..%.1f (centre %.1f)' % (
        lt(ts[i]), len(xs), np.percentile(bx, 3), np.percentile(bx, 97), np.median(bx), np.percentile(by, 3),
        np.percentile(by, 97), np.median(by)))
