"""clockloss.py [session dir] : the clock pings (clock_sync.csv). The runner numbers its pings with one counter for all
robots (cN), so a number that no robot answered is a ping or its answer lost. Loss per 15 minutes."""
import sys, os, csv
from collections import defaultdict
import common
d = sys.argv[1] if len(sys.argv) > 1 else common.D
rows = list(csv.DictReader(open(os.path.join(d, 'robot-logs-1', 'clock_sync.csv'))))
tok = {}
for r in rows:
    try:
        tok[int(r['token'].lstrip('c'))] = float(r['t_send'])
    except ValueError:
        pass
ks = sorted(tok)
lo, hi = ks[0], ks[-1]
print('%s: pings %d..%d, answered %d, lost %d (%.1f%%)' % (os.path.basename(d), lo, hi, len(ks), hi - lo + 1 - len(ks),
                                                         100.0 * (hi - lo + 1 - len(ks)) / (hi - lo + 1)))
# per 15 min: answered vs numbers spanned
buck = defaultdict(lambda: [10 ** 9, -1, 0])
for k in ks:
    b = int((tok[k] - tok[ks[0]]) // 900)
    q = buck[b]
    q[0], q[1], q[2] = min(q[0], k), max(q[1], k), q[2] + 1
for b in sorted(buck):
    q = buck[b]
    n = q[1] - q[0] + 1
    print('  %s +%3d min: %5d pings, lost %.1f%%' % (common.lt(tok[ks[0]] + 900 * b)[:5], 15 * b, n, 100.0 * (n - q[2]) / n))
