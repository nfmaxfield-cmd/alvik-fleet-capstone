# Every RAW row with a camera position: map distance ahead (heading = gyro yaw + per-run offset fitted to the camera's
# direction of forward travel), compared with each zone.
import sys, glob, pandas as pd, numpy as np, os, json
sys.path.insert(0, '/home/claude/alvik-fleet-capstone/tools/discovery'); import load
H = os.environ.get('RF_WORK', '/tmp/claude-0/rf_work')
Z = ['dL_cm','dCL_cm','dC_cm','dCR_cm','dR_cm','dtop_cm','dbot_cm']
RICH = ['e02-20261022-095125','e02-20261022-200532','e02-20261023-085107','e02-20261023-114620','e02-20261023-191347','e02-20261024-153612','e02-20261024-161612']
S = load.find_sessions()
HALF = 4.75
def wrap(a): return (a + 180) % 360 - 180
res = []
for sid in RICH:
    sdir = S[sid]
    b = json.load(open(sdir + '/e02_state.json')).get('board') or {}
    if not b.get('xboard_cm'): print('no board', sid); continue
    XB, YB = float(b['xboard_cm']), float(b['yboard_cm'])     # x-board at y = XB; y-board at x = YB
    tr = pd.read_csv(sdir + '/track.csv', usecols=['t','robot','x_cm','y_cm'])
    raw = pd.read_pickle(f'{H}/cache/{sid}.pkl')
    for (rob, run), r in raw.groupby(['robot','run']):
        r = r.sort_values('t_recv').copy()
        c = tr[tr.robot == rob].sort_values('t')
        c = c[(c.t >= r.t_recv.min() - 1) & (c.t <= r.t_recv.max() + 1)]
        if len(c) < 50 or len(r) < 100: continue
        ct = c.t.values
        i = np.clip(np.searchsorted(ct, r.t_recv.values), 1, len(ct) - 1)
        j = np.where(np.abs(ct[i] - r.t_recv.values) < np.abs(ct[i-1] - r.t_recv.values), i, i - 1)
        gap = np.abs(ct[j] - r.t_recv.values)
        r['cx'] = np.interp(r.t_recv.values, ct, c.x_cm.values); r['cy'] = np.interp(r.t_recv.values, ct, c.y_cm.values)
        r['cgap'] = gap
        # forward travel direction from the camera over 0.6 s, where wheels go forward (both) and travel > 2.5 cm
        t = r.t_recv.values
        k2 = np.searchsorted(t, t + 0.6)
        k2 = np.clip(k2, 0, len(t) - 1)
        dx, dy = r.cx.values[k2] - r.cx.values, r.cy.values[k2] - r.cy.values
        dwl, dwr = r.wl_deg.values[k2] - r.wl_deg.values, r.wr_deg.values[k2] - r.wr_deg.values
        fwd = (dwl > 60) & (dwr > 60) & (np.abs(dwl - dwr) < 0.15 * (dwl + dwr)) & (np.hypot(dx, dy) > 2.5) & (gap < 0.15)
        if fwd.sum() < 20: continue
        trav = np.degrees(np.arctan2(dy, dx))
        off = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(trav[fwd] - r.yaw_deg.values[fwd])))))
        resid = wrap(trav[fwd] - r.yaw_deg.values[fwd] - off)
        if np.median(np.abs(resid)) > 8: continue     # gyro and camera disagree: skip the run
        hd = np.radians(r.yaw_deg.values + off)
        ux, uy = np.cos(hd), np.sin(hd)
        with np.errstate(divide='ignore', invalid='ignore'):
            dyb = np.where(np.abs(ux) > 1e-6, (YB - r.cx.values) / ux, -1)
            dxb = np.where(np.abs(uy) > 1e-6, (XB - r.cy.values) / uy, -1)
        dyb[dyb <= 0] = np.inf; dxb[dxb <= 0] = np.inf
        exp_ = np.minimum(dyb, dxb) - HALF
        cos_ = np.where(dyb < dxb, np.abs(ux), np.abs(uy))
        r['exp'] = exp_; r['cos'] = cos_; r['fwd'] = fwd; r['sess'] = sid
        r['moving'] = (np.abs(dwl) + np.abs(dwr)) > 30
        res.append(r[['sess','robot','run','t_recv','exp','cos','cgap','fwd','moving','wl_deg','wr_deg'] + Z])
        print(sid, rob, run, 'rows', len(r), 'fwd', int(fwd.sum()), 'off resid', round(float(np.median(np.abs(resid))),1), flush=True)
A = pd.concat(res); A.to_pickle(H + '/ahead.pkl')
print(len(A))
