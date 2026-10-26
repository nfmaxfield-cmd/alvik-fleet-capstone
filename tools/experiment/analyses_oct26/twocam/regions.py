import sys, os, pickle, json, collections; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
d = pickle.load(open('../work/pooled.pkl', 'rb')); T = d['T']; m = d['m']
def iqr(a): return np.subtract(*np.percentile(a, [75, 25])) / 1.35
rows = []
groups = collections.defaultdict(list)
for t in T:
    groups[t['reg']].append(t)
    groups['edge band (12 cm)' if t['edge'] else 'inside'].append(t)
    groups['all'].append(t)
order = ['far left', 'far centre', 'far right', 'middle left', 'middle centre', 'middle right', 'near left', 'near centre', 'near right', 'edge band (12 cm)', 'inside', 'all']
out = {}
for g in order:
    v = groups.get(g, [])
    if not v: continue
    dist = np.array([t['dist'] for t in v]); k = dist < 10
    al = np.array([t['along'] for t in v])[k]; ac = np.array([t['across'] for t in v])[k]
    # side camera's own precision shape at the region's middle: floor cm per px along / across sight
    xs = np.median([t['x'] for t in v]); ys = np.median([t['y'] for t in v])
    J = m.jac(xs, ys); Ci = np.linalg.inv(J.T @ J); ev = np.sqrt(np.linalg.eigvalsh(Ci))
    out[g] = dict(n=len(v), agree=float(np.mean(k)), med=float(np.median(dist[k])), p95=float(np.percentile(dist[k], 95)),
                  along=float(iqr(al)), across=float(iqr(ac)), bias_along=float(np.median(al)), bias_across=float(np.median(ac)),
                  cm_per_px_major=float(ev[1]), cm_per_px_minor=float(ev[0]))
    o = out[g]
    print('%-18s n %5d agree %.1f%% med %.2f p95 %.2f | spread along %.2f across %.2f | bias along %+.2f across %+.2f | side cm/px %.2f x %.2f' % (g, o['n'], 100 * o['agree'], o['med'], o['p95'], o['along'], o['across'], o['bias_along'], o['bias_across'], o['cm_per_px_major'], o['cm_per_px_minor']))
json.dump(out, open('../work/regions.json', 'w'), indent=1)
# visibility: tracked moments (moving > 1 cm/s) with a side blob within 10 cm, per session
import pairs, calib
for s, lag in d['lag'].items():
    b = pairs.load_blobs(s); tk = pairs.track_r1(s); n = hit = 0
    for f, e in b.items():
        for r, a in tk.items():
            q = pairs.interp(a, e['t'] + lag)
            if q is None or not q[1] > 1.0: continue
            n += 1
            if e['b']:
                S = m.inv([bb[:2] for bb in e['b']]); hit += np.min(np.hypot(*(S - q[0]).T)) < 10
    print(s, 'lag %.2f moving tracked moments %d, side blob within 10 cm %.1f%%' % (lag, n, 100 * hit / max(n, 1)))
