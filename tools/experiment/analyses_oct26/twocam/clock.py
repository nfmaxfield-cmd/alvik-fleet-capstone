import sys, os, json; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, sess
res = {}
for s in sess.S:
    t0 = json.load(open(os.path.join(sess.DL, sess.S[s][1], 'session.json')))['clock']['t0_monotonic']
    o = {}
    for cam in ['cam0', 'cam1']:
        a = np.array(sess.ftimes(s, cam))
        delay = (a[:, 2] + t0) * 1000 - a[:, 3]
        dt = np.diff(a[:, 2]) * 1000
        wm = a[:, 1] - a[:, 2]
        o[cam] = dict(delay_med=np.median(delay), delay_p5=np.percentile(delay, 5), delay_p95=np.percentile(delay, 95), delay_p99=np.percentile(delay, 99),
                      dt_med=np.median(dt), dt_p99=np.percentile(dt, 99), wall_minus_mono_spread_ms=1000 * (np.percentile(wm, 99.5) - np.percentile(wm, 0.5)))
    res[s] = o
    print(s, {c: {k: round(v, 1) for k, v in d.items()} for c, d in o.items()})
    print('   delay difference side - overhead (median) ms: %.1f' % (o['cam1']['delay_med'] - o['cam0']['delay_med']))
# robot speed from track
for s in sess.S:
    sp = []
    for r, v in sess.track(s).items():
        a = np.array([(x[0], x[1], x[2]) for x in v])
        d = np.diff(a, axis=0); ok = (d[:, 0] > 0.03) & (d[:, 0] < 0.2)
        sp += list(np.hypot(d[ok, 1], d[ok, 2]) / d[ok, 0])
    sp = np.array(sp); print(s, 'track speed cm/s p50 %.1f p90 %.1f p99 %.1f' % tuple(np.percentile(sp, [50, 90, 99])))
