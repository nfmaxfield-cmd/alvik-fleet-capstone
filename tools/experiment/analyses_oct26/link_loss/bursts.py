"""bursts.py <sessions folder> : are lost pings independent? For each session, P(lost) and P(lost | the ping before it
to the same robot was lost), from clock_sync.csv tokens (pings to one robot come in bursts of 3, 50 ms apart, and the
runner's counter is shared: a missing number between two answered ones of the same robot's burst is that robot's).
Also holds and lost sends per hour of active session in windows grouped by ping loss (linkloss.json)."""
import csv, glob, json, os, sys
from collections import defaultdict
root = sys.argv[1]
for f in sorted(glob.glob(os.path.join(root, '*', 'e02-2026102[456]-*', 'robot-logs-1', 'clock_sync.csv'))):
    rows = list(csv.DictReader(open(f)))
    t = sorted((int(r['token'][1:]), float(r['t_send'])) for r in rows if r['token'][1:].isdigit())
    if f.find('211419') > 0:   # the active part only (before the robots went flat overnight)
        t = [x for x in t if x[1] < t[0][1] + 6000]
    got = {k for k, _ in t}
    lo, hi = t[0][0], t[-1][0]
    seq = [k in got for k in range(lo, hi + 1)]
    n1 = sum(1 for i in range(1, len(seq)) if not seq[i - 1])
    n11 = sum(1 for i in range(1, len(seq)) if not seq[i - 1] and not seq[i])
    p = 1 - sum(seq) / len(seq)
    print('%s  P(lost) %.3f  P(lost | previous number lost) %.3f  (n %d)' % (f.split('/')[-3], p, n11 / max(1, n1), n1))
R = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'linkloss.json')))
g = defaultdict(lambda: [0, 0, 0, 0, 0])
for r in R:
    if r['session'] < 'e02-20261024':
        continue
    for w in r['windows']:
        if w['sends'] == 0:
            continue
        b = '<2%' if w['loss'] < .02 else '2-5%' if w['loss'] < .05 else '5-15%' if w['loss'] < .15 else '>=15%'
        q = g[b]
        q[0] += 1; q[1] += w['holds']; q[2] += w['lost_sends']; q[3] += w['sends']; q[4] += w['failed_legs']
for b in ('<2%', '2-5%', '5-15%', '>=15%'):
    q = g[b]
    if q[0]:
        print('%-6s windows %3d (%.1f h)  holds %2d (%.1f/h)  lost sends %3d of %4d (%.1f%%)  failed legs %3d' % (
            b, q[0], q[0] / 12, q[1], q[1] * 12 / q[0], q[2], q[3], 100 * q[2] / max(1, q[3]), q[4]))
