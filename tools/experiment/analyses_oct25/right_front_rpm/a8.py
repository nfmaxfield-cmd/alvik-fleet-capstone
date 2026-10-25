import pandas as pd, numpy as np, os, glob
H = os.environ.get('RF_WORK', '/tmp/claude-0/rf_work')
Z = ['dL_cm','dCL_cm','dC_cm','dCR_cm','dR_cm','dtop_cm','dbot_cm']
A = pd.read_pickle(H + '/ahead.pkl')
# update rate: time between changes of the zone value (RAW is 50 Hz)
print('time between value changes (ms), median / p90, and longest unchanged run while driving forward (s)')
for rob, g in A.groupby('robot'):
    out = []
    for z in ['dCL_cm','dC_cm','dCR_cm','dbot_cm']:
        iv, longest = [], []
        for run, r in g.groupby('run'):
            v = r[z].values; t = r.t_recv.values
            ch = np.r_[True, np.diff(v) != 0]
            tc = t[ch]; iv.append(np.diff(tc))
            # longest stretch with no change while fwd
            f = r.fwd.values
            idx = np.cumsum(ch)
            s = pd.Series(t).groupby(idx).agg(['min','max']); fw = pd.Series(f).groupby(idx).mean()
            longest.append(((s['max']-s['min'])[fw > 0.9]).max() if (fw > 0.9).any() else 0)
        iv = np.concatenate(iv)
        out.append('%s %4.0f/%4.0f  stuck-max %.2f' % (z, 1000*np.median(iv), 1000*np.percentile(iv, 90), np.nanmax(longest)))
    print(rob, ' | '.join(out))
# lag: zone vs map distance during forward drives straight at a board
print('lag (s) that best lines the zone up with the map distance (+ = zone late)')
for rob, g in A.groupby('robot'):
    res = {}
    for z in ['dCL_cm','dC_cm','dCR_cm']:
        best = []
        for run, r in g.groupby('run'):
            k = r.fwd.values & (r.exp.values > 8) & (r.exp.values < 50) & (r.cos.values > 0.9)
            if k.sum() < 50: continue
            t = r.t_recv.values; e = r.exp.values; y = r[z].values
            sds = []
            taus = np.arange(-0.4, 0.61, 0.02)
            for tau in taus:
                ee = np.interp(t[k] - tau, t, e)
                m = y[k] - ee; sds.append(np.median(np.abs(m - np.median(m))))
            best.append(taus[int(np.argmin(sds))])
        res[z] = (np.median(best), len(best)) if best else None
    print(rob, {k: (round(v[0], 2), v[1]) for k, v in res.items() if v})
