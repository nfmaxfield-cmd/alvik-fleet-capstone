"""sends_prev.py : loss of a go/turn send by what was sent to the same robot just before it (same log second)."""
import re
from collections import defaultdict, Counter
from common import D, rows, ep
L = rows('robot-logs-1/leg.csv')
rec = defaultdict(list)
for r in L:
    rec[(r['robot'], r['id'])].append(float(r['t_recv']))
lines = []
for line in open(D + '/e02_log.txt'):
    m = re.match(r'\[(\S+)\]\s+-> (\w+): (.*)$', line.rstrip())
    if m:
        lines.append((ep(m.group(1), 26 if m.group(1) < '12' else 25), m.group(2), m.group(3)))
sends = defaultdict(list)
for i, (t, rb, cmd) in enumerate(lines):
    m = re.match(r'(go|turn) (\S+) (\d+)$', cmd)
    if m:
        sends[(rb, m.group(3))].append(i)
tab = Counter()
for k, idxs in sends.items():
    for j, i in enumerate(idxs):
        t, rb, cmd = lines[i]
        nxt = lines[idxs[j + 1]][0] if j + 1 < len(idxs) else t + 60
        got = any(t - 1.5 <= x < nxt - 0.5 for x in rec.get(k, []))
        prev = [q for q in lines[:i] if q[1] == rb]
        p = prev[-1] if prev else None
        cls = 'none'
        if p and t - p[0] <= 1.0:
            cls = p[2].split()[0] + ((' ' + p[2].split()[1]) if p[2].startswith('set') else '')
        elif p:
            cls = 'gap>1s'
        tab[(cls, got)] += 1
for cls in sorted(set(c for c, _ in tab)):
    a, b = tab[(cls, True)], tab[(cls, False)]
    print('%-26s sends %3d  no record %2d  (%.0f%%)' % (cls, a + b, b, 100.0 * b / (a + b)))
