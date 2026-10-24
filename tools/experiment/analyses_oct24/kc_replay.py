"""KC replay: what end error would each KC run have had with a different camera-fix gain?
Model (position only): belief error e = belief - true. A camera fix sees true + b (camera error at that spot).
Robot drives to absolute waypoints in its belief, ending at mStart, so end error E = e_mark - e_final.
eps = e - e_mark.  At a stop: innovation = m - eps, with m = b - e_mark (camera error relative to the start).
Between stops eps grows by odometry drift delta (unknown; total fixed by the measured E, spread by distance)."""
import csv, glob, math, sys, json
import numpy as np
S = sys.argv[1]
runs = {r['e02_run']: r for r in csv.DictReader(open(S + '/runs.csv'))}
e02 = {}
for f in glob.glob(S + '/robot-logs-*/e02.csv'):
    for r in csv.DictReader(open(f)):
        e02[r['run']] = r
fixes = {}
for f in glob.glob(S + '/robot-logs-*/fix.csv'):
    for r in csv.DictReader(open(f)):
        fixes.setdefault(r['run'], []).append(r)

def stops_of(run):
    r = e02[run]; m0, m1 = float(r['ms_start']), float(r['ms_end'])
    fx = sorted([f for f in fixes.get(run, []) if f['result'] == 'blended' and m0 <= float(f['ms']) <= m1],
                key=lambda f: float(f['ms']))
    stops = []
    for f in fx:
        b = np.array([float(f['before_x']), float(f['before_y'])])
        a = np.array([float(f['after_x']), float(f['after_y'])])
        c = np.array([float(f['cam_x']), float(f['cam_y'])])
        if stops and np.linalg.norm(b - stops[-1]['after']) < 1.0:
            s = stops[-1]; s['cams'].append(c); s['after'] = a; s['n'] += 1
        else:
            stops.append(dict(before=b, after=a, cams=[c], n=1))
    return stops

def replay(stops, E, gain_fn, start):
    """Returns the end error vector for a gain policy. Drift spread by belief distance between stops."""
    # actual: reconstruct m_j and drift
    pts = [start] + [s['before'] for s in stops]
    seg = [np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(stops))]
    # the last leg from the last stop back to the start
    tail = np.linalg.norm(start - stops[-1]['before']) if stops else 0.0
    corr = sum((s['after'] - s['before'] for s in stops), np.zeros(2))
    total_drift = -E - corr
    L = sum(seg) + tail
    dr = [total_drift * (d / L) for d in seg] + [total_drift * (tail / L)]
    eps = np.zeros(2); ms = []
    for j, s in enumerate(stops):
        eps = eps + dr[j]
        ms.append(np.mean(s['cams'], axis=0) - s['before'] + eps)
        eps = eps + (s['after'] - s['before'])
    # counterfactual
    eps = np.zeros(2); dist = 0.0
    for j, s in enumerate(stops):
        eps = eps + dr[j]; dist += seg[j]
        g = gain_fn(s, dist)
        eps = eps + g * (ms[j] - eps)
        if g > 0: dist = 0.0
    eps = eps + dr[-1]
    return -eps, ms

def actual_gain(s, dist):
    return 1 - 0.5 ** s['n']

out = []
for run, r in runs.items():
    if r['cond'] != 'KC' or not r['cam_end_x'] or run not in e02:
        continue
    E = np.array([float(r['cam_end_x']) - float(r['cam_start_x']), float(r['cam_end_y']) - float(r['cam_start_y'])])
    st = stops_of(run)
    start = np.array([float(e02[run]['start_x']), float(e02[run]['start_y'])])
    row = dict(run=run, robot=r['robot'], route=r['route'], E=round(float(np.linalg.norm(E)), 2), stops=len(st),
               blends=sum(s['n'] for s in st))
    for name, fn in [('none', lambda s, d: 0.0), ('actual', actual_gain), ('once_0.5', lambda s, d: 0.5),
                     ('0.1', lambda s, d: 0.1), ('0.25', lambda s, d: 0.25), ('replace', lambda s, d: 1.0)]:
        v, ms = replay(st, E, fn, start)
        row[name] = round(float(np.linalg.norm(v)), 2)
    for sc in (2.0, 3.0, 4.0):
        def w(s, d, sc=sc):
            sr = 0.5 + 0.01 * d + d * math.radians(0.5)  # turned share left out: position-only stops
            return sr ** 2 / (sr ** 2 + sc ** 2)
        v, ms = replay(st, E, w, start)
        row['weighted_cam%.0f' % sc] = round(float(np.linalg.norm(v)), 2)
    row['m_cm'] = [round(float(np.linalg.norm(m)), 1) for m in ms]
    out.append(row)
keys = ['none', 'actual', 'once_0.5', '0.1', '0.25', 'replace', 'weighted_cam2', 'weighted_cam3', 'weighted_cam4']
for o in out:
    print(o['run'], o['robot'], o['route'], 'E', o['E'], 'stops', o['stops'], 'blends', o['blends'],
          ' '.join('%s=%.2f' % (k, o[k]) for k in keys), 'camera-vs-start', o['m_cm'])
print('MEDIANS', {k: round(float(np.median([o[k] for o in out])), 2) for k in ['E'] + keys}, 'n', len(out))
json.dump(out, open('/tmp/claude-0/replay/kc_replay_%s.json' % S.rstrip('/').split('/')[-1], 'w'), indent=1)
