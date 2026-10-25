import sys, math
sys.path[:0] = ['/home/claude/alvik-fleet-capstone/tools/recognition']
import numpy as np, cv2
import cameras as CMS
cam = CMS.CameraModel.load('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json')
img = cv2.imread('/tmp/claude-0/s1025/fr/f_153200.png')
g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(float), (5, 5), 0)
W, H = 122.15, 121.9
C = [(0, 0), (W, 0), (W, H), (0, H)]
names = {0: 'y=0 (open)', 1: 'x=W (board)', 2: 'y=H (board)', 3: 'x=0 (open)'}
P = lambda x, y: np.array(cam.project((x, y, 0.0)), float).ravel()
out = img.copy()
for k in range(4):
    a, b = C[k], C[(k + 1) % 4]
    cv2.line(out, tuple(int(v) for v in P(*a)), tuple(int(v) for v in P(*b)), (0, 255, 0), 1)
print('corners px', [tuple(np.round(P(*c), 1)) for c in C])
cx, cy = W / 2, H / 2
res = {}
for k in range(4):
    a, b = np.array(C[k], float), np.array(C[(k + 1) % 4], float)
    offs = []
    for f in np.linspace(0.1, 0.9, 9):
        p = a + f * (b - a)
        inward = np.array([cx, cy]) - p
        # unit inward normal in floor
        d = (b - a) / np.linalg.norm(b - a); n = np.array([-d[1], d[0]])
        if n @ inward < 0: n = -n
        # sample brightness along the normal from -20 cm (outside) to +20 cm (inside)
        ss = np.arange(-20, 20.01, 0.25)
        vals = []
        for s in ss:
            u, v = P(*(p + s * n))
            if 0 <= u < 639 and 0 <= v < 479: vals.append(g[int(round(v)), int(round(u))])
            else: vals.append(np.nan)
        vals = np.array(vals)
        if np.isnan(vals).any(): offs.append(None); continue
        grad = np.gradient(vals)
        i = int(np.nanargmax(grad))            # dark (outside) -> bright (plywood) inward
        offs.append(round(float(ss[i]), 2))
        cv2.circle(out, tuple(int(v) for v in P(*(p + ss[i] * n))), 2, (0, 0, 255), -1)
    res[names[k]] = offs
    print(names[k], 'plywood edge found at (cm inward of the calibration line; negative = outside it):', offs)
cv2.imwrite('/tmp/claude-0/s1025/fr/calcheck.png', out)
big = cv2.resize(out, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
cv2.imwrite('/tmp/claude-0/s1025/fr/calcheck2x.png', big)
