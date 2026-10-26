"""Pair the overhead tracker's positions (track.csv, roam1 floor cm) with side-view blobs at the same moment."""
import sys, os, csv, collections; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, cv2, sess
def load_blobs(s):
    fr = collections.OrderedDict()
    for r in csv.DictReader(open(os.path.join(os.path.dirname(sess.DL), 'work', 'blobs_%s.csv' % s))):
        e = fr.setdefault(int(r['frame']), dict(t=float(r['t_wall']), b=[]))
        if r['k'] != '-1':
            e['b'].append((float(r['u']), float(r['v']), int(r['a']), int(r['x']), int(r['y']), int(r['w']), int(r['h'])))
    return fr
_tk = {}
def track_r1(s):
    """{robot: array (t, x, y)} in roam1 floor cm, 'held' rows dropped"""
    if s not in _tk:
        out = {}
        for r, v in sess.track(s).items():
            a = [(x[0],) + tuple(sess.to_roam1(s, x[0], x[1], x[2])) for x in v if x[3] != 'held']
            out[r] = np.array(a)
        _tk[s] = out
    return _tk[s]
def interp(a, t, maxgap=0.25):
    """position and speed at t from track rows bracketing t within maxgap, else None"""
    i = np.searchsorted(a[:, 0], t)
    if i == 0 or i >= len(a): return None
    t0, t1 = a[i - 1, 0], a[i, 0]
    if t1 - t0 > maxgap: return None
    w = (t - t0) / max(t1 - t0, 1e-6)
    p = a[i - 1, 1:] * (1 - w) + a[i, 1:] * w
    j0, j1 = max(i - 3, 0), min(i + 2, len(a) - 1)
    dt = a[j1, 0] - a[j0, 0]
    sp = np.hypot(*(a[j1, 1:] - a[j0, 1:])) / dt if dt > 0.05 else np.nan
    return p, sp
def candidates(s, blobs, delta, maxspeed=None):
    """rows: frame, robot, x, y, speed, u, v, area, nblobs"""
    tk = track_r1(s); rows = []
    for f, e in blobs.items():
        for r, a in tk.items():
            q = interp(a, e['t'] + delta)
            if q is None: continue
            (x, y), sp = q
            if maxspeed is not None and not (sp <= maxspeed): continue
            for b in e['b']:
                rows.append((f, r, x, y, sp, b[0], b[1], b[2], len(e['b'])))
    return rows
