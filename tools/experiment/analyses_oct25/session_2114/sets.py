"""sets.py : every 'set NAME VALUE' the runner sent (e02_log.txt) checked against the robot's own parameter dump at its
next firmware run start (param.csv): the last value set before the dump should be the value dumped. A mismatch is a
'set' the robot never got (a set has no answer, so nothing resends it)."""
import re
from collections import defaultdict
from common import D, rows, ep, lt
sets = []
for line in open(D + '/e02_log.txt'):
    m = re.match(r'\[(\S+)\]\s+-> (\w+): set (\S+) (\S+)', line)
    if m:
        sets.append((ep(m.group(1), 26 if m.group(1) < '12' else 25), m.group(2), m.group(3), float(m.group(4))))
dumps = defaultdict(dict)
dt = {}
for r in rows('robot-logs-1/param.csv'):
    dumps[(r['robot'], r['run'])][r['name']] = float(r['value'])
    dt[(r['robot'], r['run'])] = min(dt.get((r['robot'], r['run']), 9e9), float(r['t_recv']))
checked, bad = 0, []
for (rb, run), vals in sorted(dumps.items(), key=lambda kv: dt[kv[0]]):
    t = dt[(rb, run)]
    last = {}
    for ts, r2, name, v in sets:
        if r2 == rb and ts <= t - 1.0:          # sent at least a second before the dump (log times are whole seconds)
            last[name] = (ts, v)
    for name, (ts, v) in last.items():
        if name not in vals:
            continue
        checked += 1
        if abs(vals[name] - v) > 1e-3:
            bad.append((rb, run, lt(t), name, v, vals[name], lt(ts)))
print('sets checked against a later dump:', checked, 'mismatches:', len(bad))
for b in bad:
    print('%s %s (dump %s): %s set to %g at %s, robot has %g' % (b[0], b[1], b[2], b[3], b[4], b[6], b[5]))
