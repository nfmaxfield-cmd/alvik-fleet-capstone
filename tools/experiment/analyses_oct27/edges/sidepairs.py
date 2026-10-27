"""sidepairs.py OUT.csv : side camera (cam1) pictures every 0.4 s over every minute the overhead tracked a robot (both
stages); moving outlines against that minute's median picture (analyses_oct26/twocam/det.py, unchanged); each tracked
robot's track.csv spot (not 'held') put into the side picture by the robot-made side model
(data/side_floorcal_robots_20261026.json, lag 0.05 s chosen from the earlier sessions' 0-0.08 s); a pair = the nearest
outline within 20 px of that spot (20 px: about 5-8 cm on the floor; chosen). Rows: stage, t, robot, x, y, u, v, miss_px."""
import sys, os, csv, json, bisect
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(HERE, '..', '..', 'analyses_oct26', 'twocam'))   # appended: its vid.py and remote.py must not shadow this folder's
import numpy as np, cv2
import det, model
import vid, frametimes as FT
from common import TAG, track, REPO, lt
j = json.load(open(os.path.join(REPO, 'tools/experiment/data/side_floorcal_robots_20261026.json')))
SIDE = model.Side(np.array(j['H']), j['kappa'])
LAG, GATE = 0.05, 20.0
done = set()
if os.path.exists(sys.argv[1]):          # resume: minutes already written are skipped
    for r in csv.DictReader(open(sys.argv[1])):
        done.add((r['stage'], int(float(r['t']) // 60)))
    done = {d for d in done if d != max(done, key=lambda z: z[1])}   # the last minute may be partial: redo it
    keep = [r for r in csv.DictReader(open(sys.argv[1])) if (r['stage'], int(float(r['t']) // 60)) in done]
    f = open(sys.argv[1], 'w', newline=''); w0 = csv.DictWriter(f, fieldnames=list(keep[0]) if keep else ['stage']); w0.writeheader(); w0.writerows(keep); f.close()
    out = csv.writer(open(sys.argv[1], 'a', newline=''))
else:
    out = csv.writer(open(sys.argv[1], 'w', newline='')); out.writerow(['stage', 't', 'robot', 'x', 'y', 'u', 'v', 'miss_px', 'n_blobs'])
tw = FT.ftimes('cam1')
for st in ('E1', 'E2'):
    tk = {r: [q for q in a if q[3] != 'held'] for r, a in track(st).items()}
    allt = sorted(q[0] for a in tk.values() for q in a)
    mins = sorted(set(int(t // 60) for t in allt))
    for m in mins:
        if (st, m) in done: continue
        ts = np.arange(m * 60, m * 60 + 60, 0.4)
        fs = sorted(set(FT.frame_at('cam1', t) for t in ts))
        fr = vid.frames(TAG, 'cam1', fs)
        imgs = {f: cv2.imdecode(np.frombuffer(fr[f], np.uint8), cv2.IMREAD_COLOR) for f in fs if f in fr}
        imgs = {f: i for f, i in imgs.items() if i is not None and i.shape == (480, 640, 3)}
        if len(imgs) < 50: continue
        bg = det.background([imgs[f] for f in sorted(imgs)])
        n = 0
        for f in sorted(imgs):
            t = tw[f] + LAG
            b, _ = det.blobs(imgs[f], bg)
            for r, a in tk.items():
                ts_ = [q[0] for q in a]; i = bisect.bisect_left(ts_, t)
                c = [k for k in (i - 1, i) if 0 <= k < len(a)]
                if not c: continue
                k = min(c, key=lambda k: abs(ts_[k] - t))
                if abs(ts_[k] - t) > 0.15: continue
                x, y = a[k][1], a[k][2]
                u, v = SIDE.fwd([(x, y)])[0]
                best = min(((np.hypot(q['u'] - u, q['v'] - v), q) for q in b), key=lambda z: z[0], default=None)
                miss = best[0] if best else None
                out.writerow([st, '%.2f' % tw[f], r, '%.1f' % x, '%.1f' % y, '%.1f' % u, '%.1f' % v,
                              '%.1f' % miss if miss is not None else '', len(b)])
                n += miss is not None and miss < GATE
        print(st, lt(m * 60), len(imgs), 'pairs', n, flush=True)
