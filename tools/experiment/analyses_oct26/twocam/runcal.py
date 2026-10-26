"""runcal.py SESSION : calibrate at delta 0, find the track's time lag on moving robots, recalibrate there, rescan."""
import sys, os, pickle, json; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, pairs, calib
def scan(sb, m, grid):
    out = []
    for d in grid:
        P = calib.assign(sb, m, d, 25, minspeed=6)
        e = np.linalg.norm(m.fwd([(p['x'], p['y']) for p in P]) - np.array([(p['u'], p['v']) for p in P]), axis=1)
        out.append((float(d), len(P), float(np.median(e))))
    return out
if __name__ == '__main__':
    s = sys.argv[1]; sb = [(s, pairs.load_blobs(s))]
    m, P = calib.calibrate(sb, 0.0)
    sc = scan(sb, m, np.arange(-0.8, 0.31, 0.05)); d = min(sc, key=lambda z: z[2])[0]
    print('first lag', d)
    m, P = calib.calibrate(sb, d)
    sc = scan(sb, m, np.arange(d - 0.2, d + 0.21, 0.02)); d2 = min(sc, key=lambda z: z[2])[0]
    for z in sc: print('  %+.2f n %d med %.2f' % z)
    m, P = calib.calibrate(sb, d2)
    pickle.dump(dict(m=m, delta=d2, scan=sc, P=P), open('../work/cal_%s.pkl' % s, 'wb'))
    print('DONE', s, 'lag', d2)
