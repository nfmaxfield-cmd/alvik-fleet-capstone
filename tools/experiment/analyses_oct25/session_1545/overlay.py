"""overlay.py FRAME_INDEX OUT [robot] [crop x0,y0,x1,y1 px] [scale]: frame with the stored calibration's outline (floor),
the 8 cm border, a 5 cm grid near the open edges, and the runner's tracked spot for that time."""
import sys, os, json, csv, bisect, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, cv2
from common import *
S = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
cal = json.load(open('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json'))
K = np.array(cal['K']); dist = np.array(cal['dist']); rv = np.array(cal['rvec']); tv = np.array(cal['tvec'])
mir = cal.get('mirror_y', False)
def px(x, y, z=0.0):
    p = np.array([[x, -y if mir else y, z]], dtype=np.float64)
    u, _ = cv2.projectPoints(p, rv, tv, K, dist); return tuple(int(round(v)) for v in u[0, 0])
TOP = 3.7
k = int(sys.argv[1]); outp = sys.argv[2]
rob = sys.argv[3] if len(sys.argv) > 3 else None
img = cv2.imread(os.path.join(S, 'frames', '%06d.jpg' % k))
times = dict(tuple(map(float, l.split(','))) for l in open(os.path.join(S, 'frames', 'times.csv')))
t = times[k]
# outline (stored), border 8 cm on open edges x=0, y=0
X1, Y1 = 122.15, 121.9
def line(a, b, col, th=1, z=0.0, n=20):
    pts = [px(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n, z) for i in range(n + 1)]
    for p, q in zip(pts, pts[1:]): cv2.line(img, p, q, col, th)
line((0, 0), (X1, 0), (0, 255, 0)); line((0, 0), (0, Y1), (0, 255, 0)); line((X1, 0), (X1, Y1), (0, 160, 0)); line((0, Y1), (X1, Y1), (0, 160, 0))
line((8, 0), (8, Y1), (0, 200, 255)); line((0, 8), (X1, 8), (0, 200, 255))
for v in (-5, 5, 10, 15, 20):
    line((v, 0), (v, Y1), (90, 90, 90)); line((0, v), (X1, v), (90, 90, 90))
for v in range(0, 121, 10):
    cv2.putText(img, str(v), px(-3, v), cv2.FONT_HERSHEY_PLAIN, 0.6, (0, 255, 0), 1)
    cv2.putText(img, str(v), px(v, -3), cv2.FONT_HERSHEY_PLAIN, 0.6, (0, 255, 0), 1)
if rob:
    T = [r for r in rows('track.csv') if r['robot'] == rob]
    ts = [float(r['t']) for r in T]
    i = bisect.bisect_left(ts, t)
    for j in (i - 1, i):
        if 0 <= j < len(T) and abs(ts[j] - t) < 0.6:
            x, y = float(T[j]['x_cm']), float(T[j]['y_cm'])
            u = px(x, y, TOP); cv2.drawMarker(img, u, (255, 0, 255), cv2.MARKER_CROSS, 12, 1)
            break
cv2.putText(img, '%s f%d' % (lt(t), k), (5, 15), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), 1)
if len(sys.argv) > 4:
    x0, y0, x1, y1 = map(int, sys.argv[4].split(','))
    img = img[y0:y1, x0:x1]
    sc = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0
    img = cv2.resize(img, None, fx=sc, fy=sc, interpolation=cv2.INTER_CUBIC)
cv2.imwrite(outp, img)
print(lt(t), 'x0 line px', px(0, 0), px(0, 60), px(0, 121.9), 'y0 line', px(60, 0), px(122.15, 0))
