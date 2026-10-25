import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *
import json, bisect, statistics as S, collections
st = json.load(open(os.path.join(D, 'e02_state.json')))
P = rows('perf.csv'); pt = [float(r['t']) for r in P]
T = rows('track.csv')
for rob in ('Center', 'Left'):
    fd = st['fit_data'][rob]
    ws = st['fits'][rob]['values']['wheel_scale']
    print(rob, 'stretches', len(fd), 'final wheel scale', ws, 'keys', list(fd[0].keys()))
    tt = [(float(r['t']), r['source'], float(r['x_cm']), float(r['y_cm'])) for r in T if r['robot'] == rob]
    ts = [q[0] for q in tt]
    res = []
    for s in fd:
        r = s['cam_len'] - ws * s['bel_len']
        i = bisect.bisect_left(pt, s['tm']); p = P[min(i, len(P) - 1)]
        j = bisect.bisect_left(ts, s['tm'])
        src = collections.Counter(q[1] for q in tt[max(0, j - 5):j + 5]).most_common(1)
        res.append((r, float(p['fps_handled']), float(p['age_mean_s']), src[0][0] if src else '-', s['run'], s['cam_len'], s['forward']))
    rms = lambda v: (sum(x * x for x in v) / len(v)) ** 0.5 if v else float('nan')
    print('  all: rms %.2f cm n=%d' % (rms([x[0] for x in res]), len(res)))
    for lab, f in (('fps>=11', lambda x: x[1] >= 11), ('fps<11', lambda x: x[1] < 11),
                   ('src both', lambda x: x[3] == 'both'), ('src not both', lambda x: x[3] != 'both'),
                   ('forward', lambda x: x[6]), ('backward', lambda x: not x[6]),
                   ('len<20', lambda x: x[5] < 20), ('len>=20', lambda x: x[5] >= 20)):
        v = [x[0] for x in res if f(x)]
        print('  %-13s rms %.2f n=%d' % (lab, rms(v), len(v)))
    by = collections.defaultdict(list)
    for x in res: by[x[4]].append(x[0])
    print('  by run:', {k: '%.2f/%d' % (rms(v), len(v)) for k, v in by.items()})
    print('  worst:', sorted(((round(x[0], 1), x[1], x[3], round(x[5]), x[6]) for x in res), key=lambda z: -abs(z[0]))[:8])
