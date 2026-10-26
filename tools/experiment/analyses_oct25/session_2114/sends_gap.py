"""sends_gap.py : for each go/turn send, seconds since that robot's previous leg ended (its done/failed record), and
whether the send got a record; also by robot and by the hour."""
import re, sys, bisect
from collections import defaultdict, Counter
import common
from common import rows, lt, ep
D = common.D
L = rows('robot-logs-1/leg.csv')
rec = defaultdict(list)
ends = defaultdict(list)
for r in L:
    rec[(r['robot'], r['id'])].append(float(r['t_recv']))
    if r['result'] in ('done', 'failed') or r['result'].startswith('failed') or r['result'].startswith('refused'):
        ends[r['robot']].append(float(r['t_recv']))
for k in ends:
    ends[k].sort()
sends = defaultdict(list)
for line in open(D + '/e02_log.txt'):
    m = re.match(r'\[(\S+)\]\s+-> (\w+): (go|turn) (\S+) (\d+)', line)
    if m:
        sends[(m.group(2), m.group(5))].append(ep(m.group(1), 26 if m.group(1) < '12' else 25) + 0.5)
tab, byr = Counter(), Counter()
for k, ts in sends.items():
    for j, t in enumerate(ts):
        nxt = ts[j + 1] if j + 1 < len(ts) else t + 60
        got = [x for x in rec.get(k, []) if t - 2.0 <= x < nxt - 1.0]
        ok = bool(got)
        e = ends[k[0]]
        i = bisect.bisect_left(e, (got[0] if got else t) - 0.05) - 1
        gap = (got[0] if got else t) - e[i] if i >= 0 else 99
        b = '<1 s' if gap < 1 else '1-3 s' if gap < 3 else '3-10 s' if gap < 10 else '>10 s'
        tab[(b, ok)] += 1
        byr[(k[0], ok)] += 1
for b in ('<1 s', '1-3 s', '3-10 s', '>10 s'):
    a, n = tab[(b, True)], tab[(b, False)]
    print('since its last leg ended %-7s sends %3d, no record %2d' % (b, a + n, n))
for r in ('Left', 'Center', 'Right'):
    print(r, 'sends', byr[(r, True)] + byr[(r, False)], 'no record', byr[(r, False)])
