# Join each front check (reading at a leg end, board square on, <= 50 cm) with the RAW row nearest in time: all zones
import glob, pandas as pd, numpy as np, os
H = os.environ.get('RF_WORK', '/tmp/claude-0/rf_work')
Z = ['dL_cm','dCL_cm','dC_cm','dCR_cm','dR_cm','dtop_cm','dbot_cm']
fc = pd.read_pickle(H+'/fc.pkl')
fc = fc[fc.what.astype(str).str.contains('board') & (fc.expected_cm <= 50)].copy()
out = []
for sid, g in fc.groupby('sess'):
    p = f'{H}/cache/{sid}.pkl'
    if not os.path.exists(p): continue
    raw = pd.read_pickle(p)
    for rob, gg in g.groupby('robot'):
        r = raw[raw.robot == rob].sort_values('t_recv')
        if r.empty: continue
        t = r.t_recv.values
        for _, row in gg.iterrows():
            i = np.searchsorted(t, row.t)
            cand = [j for j in (i-1, i) if 0 <= j < len(t)]
            j = min(cand, key=lambda j: abs(t[j]-row.t))
            dt = t[j] - row.t
            if abs(dt) > 0.5: continue
            # rows within +-0.3 s: median per zone
            k = (t > row.t - 0.4) & (t < row.t + 0.1)
            d = {z: r[z].values[k].mean() for z in Z}
            d.update(sess=sid, robot=rob, expected=row.expected_cm, front=row.front_cm, dt=dt, hm=row.heading_measured)
            out.append(d)
J = pd.DataFrame(out); J.to_pickle(H+'/join.pkl')
print('joined', len(J), J.groupby('robot').size().to_dict(), 'dt med', J.dt.abs().median())
print('front_cm vs dC_cm at join (should agree):', (J.front - J.dC_cm).abs().median())
pd.set_option('display.width', 250)
for rob, g in J.groupby('robot'):
    print(rob, 'n', len(g))
    for z in Z:
        m = g[z] - g.expected
        sl = np.polyfit(g.expected, g[z], 1)
        print('  %-8s median(reading-map) %6.1f  spread %5.1f  slope %5.2f  r %5.2f  median level %6.1f' % (
            z, m.median(), (m-m.median()).abs().median()*1.4826, sl[0], np.corrcoef(g.expected, g[z])[0,1], g[z].median()))
