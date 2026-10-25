"""crop.py ROBOT X Y HALF SCALE OUT FRAME...: crops around board point (X, Y) of several frames, side by side."""
import sys, os, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, cv2, json
cal = json.load(open('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json'))
K = np.array(cal['K']); rv = np.array(cal['rvec']); tv = np.array(cal['tvec'])
u, _ = cv2.projectPoints(np.array([[float(sys.argv[2]), float(sys.argv[3]), 0.0]]), rv, tv, K, np.zeros(5))
cu, cv_ = int(u[0, 0, 0]), int(u[0, 0, 1]); h = int(sys.argv[4]); sc = sys.argv[5]; out = sys.argv[6]
S = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
tiles = []
for k in sys.argv[7:]:
    p = os.path.join(S, 'view', '_t%s.png' % k)
    subprocess.run([sys.executable, '-I', os.path.join(S, 'work', 'overlay.py'), k, p, sys.argv[1],
                    '%d,%d,%d,%d' % (max(0, cu - h), max(0, cv_ - h), cu + h, cv_ + h), sc], check=True, capture_output=True)
    tiles.append(cv2.imread(p)); os.remove(p)
hmax = max(t.shape[0] for t in tiles)
tiles = [cv2.copyMakeBorder(t, 0, hmax - t.shape[0], 0, 4, cv2.BORDER_CONSTANT, value=(0, 0, 0)) for t in tiles]
rows_ = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
w = max(r.shape[1] for r in rows_)
rows_ = [cv2.copyMakeBorder(r, 0, 4, 0, w - r.shape[1], cv2.BORDER_CONSTANT, value=(0, 0, 0)) for r in rows_]
cv2.imwrite(out, np.vstack(rows_))
