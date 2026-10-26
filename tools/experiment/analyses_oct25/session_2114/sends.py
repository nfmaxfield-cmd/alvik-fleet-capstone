"""sends.py : every go/turn the runner sent (e02_log.txt) against the robot's own leg records (leg.csv): sends that
got no 'queued' record before the next send of the same id (lost on the way to the robot, or its answer lost)."""
import re, sys
from collections import defaultdict
from common import D, rows, lt, ep
L = rows('robot-logs-1/leg.csv')
rec = defaultdict(list)
for r in L:
    rec[(r['robot'], r['id'])].append((float(r['t_recv']), r['result'], r))
sent = defaultdict(list)
for line in open(D + '/e02_log.txt'):
    m = re.match(r'\[(\S+)\]\s+-> (\w+): (go|turn) (\S+) (\d+)', line)
    if m:
        t = ep(m.group(1), 26 if m.group(1) < '12' else 25)
        sent[(m.group(2), m.group(5))].append((t, m.group(3), m.group(4)))
n_send = sum(len(v) for v in sent.values())
lost = []
for k, ss in sent.items():
    for i, (t, kind, val) in enumerate(ss):
        nxt = ss[i + 1][0] if i + 1 < len(ss) else t + 60
        got = [x for x in rec.get(k, []) if t - 1.5 <= x[0] < nxt - 0.5]
        if not got:
            lost.append((t, k[0], k[1], kind, val, i + 1, len(ss)))
print('sends', n_send, 'legs', len(sent), 'sends with no record from the robot', len(lost))
for t, rb, lid, kind, val, i, n in sorted(lost):
    after = [x for x in rec.get((rb, lid), []) if x[0] > t]
    print(lt(t), rb, lid, kind, val, 'send %d of %d' % (i, n), '| later records:',
          ', '.join('%s %s' % (lt(x[0]), x[1]) for x in after[:3]))
