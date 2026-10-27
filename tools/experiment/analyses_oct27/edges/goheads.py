"""goheads.py : for every drive leg of 8 cm or more, the heading the camera saw (start -> end of the leg, track.csv)
against the robot's own heading; their difference drifts when a robot's turns are not what its gyro says."""
import csv, math, bisect, os, sys
from common import STAGE, track, lt
out = []
for st in ('E1', 'E2'):
    tk = track(st)
    legs = {}
    for r in csv.DictReader(open(os.path.join(STAGE[st], 'robot-logs-1', 'leg.csv'))):
        if r['kind'] != 'go': continue
        k = (r['robot'], r['id'])
        if r['result'] == 'started': legs[k] = [float(r['t_recv']), float(r['th_deg']), None, float(r['value'])]
        elif r['result'] in ('done', 'failed') and k in legs and legs[k][2] is None: legs[k][2] = float(r['t_recv'])
    for (robot, i), (t0, th, t1, v) in sorted(legs.items(), key=lambda kv: kv[1][0]):
        if t1 is None or abs(v) < 8: continue
        a = [q for q in tk.get(robot, []) if q[3] != 'held']; ts = [q[0] for q in a]
        def at(t):
            j = bisect.bisect_left(ts, t); c = [x for x in (j - 1, j) if 0 <= x < len(a)]
            if not c: return None
            j = min(c, key=lambda x: abs(ts[x] - t)); return a[j] if abs(ts[j] - t) < 0.5 else None
        p, q = at(t0 + 0.2), at(t1 + 0.3)
        if not p or not q: continue
        dx, dy = q[1] - p[1], q[2] - p[2]; d = math.hypot(dx, dy)
        if d < 6: continue
        ch = math.degrees(math.atan2(dy, dx)) + (180 if v < 0 else 0)
        diff = (ch - th + 180) % 360 - 180
        out.append((st, robot, lt(t0), v, round(d, 1), round(th, 1), round(ch, 1), round(diff, 1)))
for o in out: print(*o)
