"""How well the camera sees a robot at each place on the board (session 1 morning footage).
For sampled frames (one every 2 s) and each robot the tracker had a position for within 0.2 s: project the
robot's top into the picture (the hand-clicked calibration session 1 used) and measure there: the robot's
contrast against a floor ring around it, the share of washed-out pixels (>= 250) on the robot, and the tracker's
source at that moment. Also the floor alone: a median of frames, brightness and washed-out share by cell.
Cells of 15 cm on the board."""
import sys, csv, bisect, json
for p in ('/home/claude/alvik-fleet-capstone/tools/experiment', '/home/claude/alvik-fleet-capstone/tools',
          '/home/claude/alvik-fleet-capstone/tools/recognition'):
    sys.path.insert(0, p)
import cv2, numpy as np
import e02_camera as CAM
cam = CAM.load_camera('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json')
REC = '/tmp/claude-0/oct23/rec'; S = '/tmp/claude-0/oct23/e02-20261023-085107'
fr = [(int(r['frame']), float(r['t_wall'])) for r in csv.DictReader(open(REC + '/frames.csv'))]
fts = [t for _, t in fr]
tr = {}
for r in csv.DictReader(open(S + '/track.csv')):
    if r['x_cm']:
        tr.setdefault(r['robot'], []).append((float(r['t']), float(r['x_cm']), float(r['y_cm']), r['source']))
TS = {}
for n, v in tr.items():
    v.sort(); TS[n] = [a[0] for a in v]
cap = cv2.VideoCapture(REC + '/video.avi')
yy, xx = np.mgrid[-24:25, -24:25]; rr = np.hypot(xx, yy)
inner, ring = rr <= 6, (rr >= 13) & (rr <= 20)
rows, floors = [], []
t0 = max(fts[0], min(v[0] for v in TS.values())); t1 = min(fts[-1], max(v[-1] for v in TS.values()))
for t in np.arange(t0, t1, 4.0):
    i = min(bisect.bisect(fts, t), len(fr) - 1)
    cap.set(cv2.CAP_PROP_POS_FRAMES, fr[i][0]); ok, img = cap.read()
    if not ok:
        continue
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    if len(floors) < 300 and len(rows) % 10 == 0:
        floors.append(g)
    tf = fr[i][1]
    for n, v in tr.items():
        k = bisect.bisect(TS[n], tf)
        best = min([k - 1, k], key=lambda j: abs(TS[n][j] - tf) if 0 <= j < len(v) else 1e9)
        if not (0 <= best < len(v)) or abs(TS[n][best] - tf) > 0.2:
            continue
        _, x, y, src = v[best]
        u, w = cam.project((x, y, CAM.ROBOT_TOP_CM))
        u, w = int(round(u)), int(round(w))
        if not (24 <= u < 640 - 24 and 24 <= w < 480 - 24):
            continue
        patch = g[w - 24:w + 25, u - 24:u + 25]
        rows.append(dict(t=round(tf, 1), robot=n, x=x, y=y, src=src, robot_mean=float(patch[inner].mean()),
                         floor_mean=float(patch[ring].mean()), floor_sd=float(patch[ring].std()),
                         contrast=float(patch[inner].mean() - patch[ring].mean()), robot_sd=float(patch[rr <= 9].std()), robot_max=float(patch[inner].max()),
                         washed=float((patch[inner] >= 250).mean()), floor_washed=float((patch[ring] >= 250).mean())))
json.dump(rows, open('/tmp/claude-0/an/quality_rows2.json', 'w'))
med = np.median(np.array(floors), axis=0)
cv2.imwrite('/tmp/claude-0/an/floor_median.png', med.astype(np.uint8))
print(len(rows), 'robot samples;', len(floors), 'floor frames')
