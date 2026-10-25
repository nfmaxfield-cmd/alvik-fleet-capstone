"""timeline.py ROBOT FROM TO VIDFILE: second-by-second table for a border crossing."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *
import bisect, statistics as S, math, json
rob, a, b, vf = sys.argv[1], ep(sys.argv[2]), ep(sys.argv[3]), sys.argv[4]
REAL_EDGE = 0.0     # the physical drop: at the stored bottom line within ~1-2 cm (see report); the bright plywood edge is at 4.7
st = json.load(open(os.path.join(D, 'e02_state.json')))
x0_moved = [u['t'] for u in st.get('open_edge_updates', []) if u['edge'] == 'x0']
legs = {}
for r in rows('robot-logs-1/leg.csv'):
    if r['robot'] != rob: continue
    k = r['id']; t = float(r['t_recv'])
    if r['result'] == 'started': legs[k] = dict(t0=t, kind=r['kind'], val=float(r['value']), p0=(float(r['x_cm']), float(r['y_cm']), float(r['th_deg'])))
    elif r['result'] in ('done', 'failed') and k in legs:
        legs[k].update(t1=t, res=(r['info'] or r['result'])[:22], p1=(float(r['x_cm']), float(r['y_cm']), float(r['th_deg'])))
L = sorted(legs.values(), key=lambda q: q['t0'])
T = [(float(r['t']), float(r['x_cm']), float(r['y_cm']), r['source']) for r in rows('track.csv') if r['robot'] == rob]
P = [(float(r['t']), float(r['fps_handled']), float(r['age_mean_s'])) for r in rows('perf.csv')]
V = []
for l in open(vf):
    p = l.split()
    if len(p) >= 4 and p[2] != 'none': V.append((float(p[1]), float(p[2]), float(p[3])))
print('| Time | Command [robot result] | Robot own x, y, heading | Tracker x, y (source) | Pictures/s, age | Border 8 cm: d, towards/speed cm/s | Watchdog 9 cm: d now -> in 0.3 s |'); print('|---|---|---|---|---|---|---|')
t = a
while t < b:
    lg = [q for q in L if q['t0'] <= t + 1 and q.get('t1', 1e18) >= t]
    cmd = '; '.join('%s %.1f%s' % (q['kind'], q['val'], ' [' + q['res'] + ']' if q.get('t1', 1e18) <= t + 1 else '') for q in lg) or 'standing'
    own = None
    for q in L:
        if q['t0'] <= t + 1: own = q['p1'] if q.get('t1', 1e18) <= t + 1 else q['p0']
    tr = [q for q in T if t <= q[0] < t + 1]
    vv = [q for q in V if t <= q[0] < t + 1]
    pp = [q for q in P if t <= q[0] < t + 1.0]
    x0 = 8.0 if any(t + 1 > u for u in x0_moved) else 0.0
    # border (as run): last 0.6 s of samples at the end of the second
    smp = [q for q in T if t + 0.4 <= q[0] < t + 1]
    bd = ''
    if len(smp) >= 3:
        d = abs(smp[-1][1] - x0); dt = max(1e-3, smp[-1][0] - smp[0][0])
        tow = (abs(smp[0][1] - x0) - abs(smp[-1][1] - x0)) / dt
        sp = math.hypot(smp[-1][1] - smp[0][1], smp[-1][2] - smp[0][2]) / dt
        live = [q for q in L if q['t0'] - 0.3 <= t + 1 and q.get('t1', 1e18) >= t + 0.5]
        ex = 'not driving' if not live else ('turn exempt' if live[-1]['kind'] == 'turn' else '')
        trip = d < 8 and 2 < tow < 50 and tow >= 0.5 * sp
        bd = '%.1f %+.1f/%.1f %s' % (d, tow, sp, ('HOLD' if trip and not ex else ('(' + ex + ')' if ex else ('-' if d >= 8 else 'along/slow'))))
    wd = ''
    w = [q for q in T if t + 0.5 <= q[0] < t + 1]
    if len(w) >= 4:
        k = max(2, len(w) // 4)
        xa = S.median(q[1] for q in w[:k]); xb = S.median(q[1] for q in w[-k:])
        dtw = max(1e-3, S.median(q[0] for q in w[-k:]) - S.median(q[0] for q in w[:k]))
        vx = (xb - xa) / dtw
        d0 = xb - x0; d1 = d0 + vx * 0.3
        wd = '%.1f->%.1f%s' % (d0, d1, ' STOP' if d1 < 9 and d1 < d0 - 0.5 else '')
    f = lambda v, i: S.median(q[i] for q in v)
    print('| %s | %s | %s | %s | %s | %s | %s |' % (
        lt(t)[:8], cmd,
        '%.1f, %.1f, %.0f' % own if own else '-',
        ('%.1f, %.1f (%s)' % (f(tr, 1), f(tr, 2), max(set(q[3] for q in tr), key=[q[3] for q in tr].count))) if tr else 'no picture',
        ('%.0f, %.2f s' % (pp[0][1], pp[0][2])) if pp else '-', bd or '-', wd or '-'))
    t += 1
