import pandas as pd, numpy as np, os
H = os.environ.get('RF_WORK', '/tmp/claude-0/rf_work')
out = []
for sid in ['e02-20261023-085107','e02-20261023-191347','e02-20261024-161612']:
    df = pd.read_pickle(f'{H}/cache/{sid}.pkl')
    for (rob, run), r in df.groupby(['robot','run']):
        r = r.sort_values('ms')
        if len(r) < 500: continue
        ms = r.ms.values.astype(float); a = r.wl_deg.values; b = r.wr_deg.values; rn = r.raw_n.values
        # consecutive rows (no lost row), both wheels turning well (> 0.5 deg per row) -> count rows with no update of either wheel
        d_ok = (np.diff(rn) == 1)
        moving = (np.abs(np.diff(a)) > 0) | (np.abs(np.diff(b)) > 0)
        # long moving blocks: 1 s windows where the angle advanced > 60 deg
        t = ms / 1000; i1 = np.clip(np.searchsorted(t, t + 1.0), 0, len(t)-1)
        fast = (np.abs(a[i1] - a) > 60) & (np.abs(b[i1] - b) > 60)
        k = fast[:-1] & d_ok
        # updates seen = rows where the angle changed; elapsed robot time over those rows
        ch = (np.diff(a) != 0)[k]
        el = np.diff(ms)[k]
        out.append(dict(sess=sid, robot=rob, rows=int(k.sum()), T_ms=el.sum() / max(1, ch.sum()), raw_dt=el.mean()))
R = pd.DataFrame(out)
R = R[R.rows > 500]
print(R.groupby('robot')[['T_ms','raw_dt']].median().round(2)); print(R.groupby(['sess','robot'])[['T_ms']].median().round(2))
