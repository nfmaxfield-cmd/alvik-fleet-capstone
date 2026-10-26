"""timeline.py ROBOT HH:MM:SS HH:MM:SS [step_s] : per step, what the runner sent (e02_log.txt), the robot's own leg
records (leg.csv), its edge events (edge.csv), the camera position on the board (pose.py: roam1 cm, d = the centre's
signed distance to the nearest open edge line, + inside), and the camera's pictures a second and picture age
(perf.csv). Markdown rows."""
import re, sys, csv, os, bisect
import pose
from common import D, rows, ep, lt
robot, a, b = sys.argv[1], ep(sys.argv[2]), ep(sys.argv[3])
step = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
ev = []
for line in open(D + '/e02_log.txt'):
    m = re.match(r'\[(\S+)\]\s+(.*)$', line.rstrip())
    if not m:
        continue
    t = ep(m.group(1))
    txt = m.group(2)
    if a - 1 <= t <= b + 1:
        mm = re.match(r'-> (\w+): (go|turn|stop) ?(\S*)', txt)
        if mm and mm.group(1) == robot:
            ev.append((t, 'sent ' + mm.group(2) + (' ' + mm.group(3) if mm.group(3) else '')))
        elif robot in txt and ('HOLD' in txt or 'STOPPED' in txt or 'answer' in txt or 'SPEED GUARD' in txt):
            ev.append((t, txt[:60]))
        elif "Nick's answer" in txt:
            ev.append((t, txt[:40]))
legs = [(float(r['t_recv']), r) for r in rows('robot-logs-1/leg.csv') if r['robot'] == robot
        and a - 1 <= float(r['t_recv']) <= b + 1 and r['result'] not in ('started',)]
edges = [(float(r['t_recv']), r) for r in rows('robot-logs-1/edge.csv') if r['robot'] == robot
         and a - 1 <= float(r['t_recv']) <= b + 1 and r['event'] != 'base']
tr = pose.track(robot, a - 1, b + 1)
tt = [q['t'] for q in tr]
perf = [(float(r['t']), r) for r in rows('perf.csv') if a - 2 <= float(r['t']) <= b + 2]
pt = [p[0] for p in perf]
print('| Time | Runner sent / said | Robot (own record) | Edge sensors | Camera on board x, y (d) | Pictures/s, age |')
print('|---|---|---|---|---|---|')
t = a
while t < b:
    e = '; '.join(x[1] for x in ev if t <= x[0] < t + step)
    lg = '; '.join('%s %s %s%s' % (r['kind'], r['value'], r['result'], (' (' + r['info'] + ')') if r['info'] and
                                   not r['info'].startswith('front=') else '') for x, r in legs if t <= x < t + step)
    ed = '; '.join('%s %s/%s/%s bot %s' % (r['info'] or r['event'], r['lineL'], r['lineC'], r['lineR'], r['dbot_cm'])
                   for x, r in edges if t <= x < t + step)
    i = bisect.bisect_left(tt, t + step) - 1
    cam = ''
    if i >= 0 and tr[i]['t'] >= t - 0.5:
        q = tr[i]
        cam = '%.1f, %.1f (%.1f) %s' % (q['bx'], q['by'], min(q['bx'], q['by']), q['src'])
    j = bisect.bisect_left(pt, t + step) - 1
    pf = '%s, %.2f s' % (perf[j][1]['fps_handled'], float(perf[j][1]['age_mean_s'])) if j >= 0 else ''
    print('| %s | %s | %s | %s | %s | %s |' % (lt(t)[:8], e, lg, ed, cam, pf))
    t += step
