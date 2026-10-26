"""trackmix.py : per minute, track.csv rows a second per robot and their sources (what the tracker was doing)."""
import collections
from common import rows, lt
m = collections.defaultdict(collections.Counter)
for r in rows('track.csv'):
    k = lt(r['t'])[:5]
    m[k][r['robot'] + ':' + r['source']] += 1
for k in sorted(m):
    print(k, ' '.join('%s=%.1f' % (a, b / 60) for a, b in sorted(m[k].items())))
