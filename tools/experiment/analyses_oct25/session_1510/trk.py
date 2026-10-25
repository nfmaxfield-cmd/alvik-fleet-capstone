import sys, csv, math, time, json
sys.path[:0] = ['/home/claude/alvik-fleet-capstone/tools/experiment', '/home/claude/alvik-fleet-capstone/tools/recognition']
import numpy as np
import cameras as CMS
L = '/tmp/claude-0/s1025/logs/e02-20261025-151004/'
prov = CMS.CameraModel([[600, 0, 320], [0, 600, 240], [0, 0, 1]], [0]*5, [math.pi, 0, 0], [0, 0, 260.0], (640, 480))
guard = CMS.CameraModel.load('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json') if hasattr(CMS.CameraModel,'load') else None
def T(t): return time.strftime('%H:%M:%S', time.gmtime(t-4*3600)) + '.%d' % int((t % 1)*10)
rows = []
for r in csv.DictReader(open(L + 'track.csv')):
    t = float(r['t']); x = float(r['x_cm']); y = float(r['y_cm'])
    u, v = prov.project((x, y, 3.7))
    gx, gy = guard.to_floor(float(u), float(v), 3.7)
    rows.append(dict(t=t, r=r['robot'], x=x, y=y, u=float(u), v=float(v), gx=gx, gy=gy, e=min(gx, gy), src=r['source'], ph=r['phase'], ls=r['lock_score']))
if __name__ == '__main__':
    a, b, who = sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None
    def P(s):
        h, m, sec = s.split(':'); base = 1792955404.0 - (15*3600+10*60+4)
        return base + int(h)*3600 + int(m)*60 + float(sec)
    t0, t1 = P(a), P(b)
    step = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    last = {}
    for q in rows:
        if t0 <= q['t'] <= t1 and (who is None or q['r'] == who):
            if q['t'] - last.get(q['r'], 0) < step: continue
            last[q['r']] = q['t']
            print(T(q['t']), q['r'], 'prov(%.1f,%.1f) px(%.0f,%.0f) roam(%.1f,%.1f) edge %.1f' % (q['x'], q['y'], q['u'], q['v'], q['gx'], q['gy'], q['e']), q['src'], q['ph'], q['ls'])
