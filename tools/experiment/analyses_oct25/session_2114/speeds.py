"""speeds.py : each go leg's average speed (its own distance over queued->done/failed time) against the speed the
runner last set (set e02_cms in e02_log.txt): a 'set' the robot never got shows as a leg at the old speed."""
import re
from collections import defaultdict
from common import D, rows, ep, lt
setv = defaultdict(list)
for line in open(D + '/e02_log.txt'):
    m = re.match(r'\[(\S+)\]\s+-> (\w+): set e02_cms (\S+)', line)
    if m:
        setv[m.group(2)].append((ep(m.group(1), 26 if m.group(1) < '12' else 25), float(m.group(3))))
q = {}
out = []
for r in rows('robot-logs-1/leg.csv'):
    k = (r['robot'], r['id'])
    if r['kind'] != 'go':
        continue
    if r['result'] == 'queued':
        q[k] = float(r['t_recv'])
    elif k in q and r['result'] in ('done',) and abs(float(r['x_cm'])) > 2.5:
        t0, t1 = q.pop(k), float(r['t_recv'])
        s = [v for t, v in setv[r['robot']] if t <= t0 + 0.2]
        want = s[-1] if s else 10.0
        out.append((t0, r['robot'], float(r['value']), float(r['x_cm']), t1 - t0, want))
for t0, rb, val, d, dt, want in out:
    cms = abs(d) / max(0.1, dt - 0.5)       # minus about 0.5 s of start and stop
    flag = ' <-- faster than set' if cms > 1.6 * want else ''
    if flag or want < 10:
        print('%s %-6s go %7.2f moved %7.2f in %.1f s: %.1f cm/s (set %g)%s' % (lt(t0), rb, val, d, dt, cms, want, flag))
