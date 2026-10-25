# wl_rpm against the wheel angle's rate; the motor board's update period from how often the angle changes; gyro checks
import glob, pandas as pd, numpy as np, os
H = os.environ.get('RF_WORK', '/tmp/claude-0/rf_work')
RICH = ['e02-20261022-095125','e02-20261022-200532','e02-20261023-085107','e02-20261023-114620','e02-20261023-191347','e02-20261024-153612','e02-20261024-161612']
rows = []; pts = []
for sid in RICH:
    df = pd.read_pickle(f'{H}/cache/{sid}.pkl')
    for (rob, run), r in df.groupby(['robot', 'run']):
        r = r.sort_values('ms')
        if len(r) < 500: continue
        ms = r.ms.values.astype(float); t = ms / 1000.0
        out = dict(sess=sid, robot=rob, run=run, n=len(r))
        for side in ('l', 'r'):
            deg = r[f'w{side}_deg'].values; rpm = r[f'w{side}_rpm'].values
            # rate over +-0.25 s centred windows (robot clock)
            i0 = np.searchsorted(t, t - 0.25); i1 = np.clip(np.searchsorted(t, t + 0.25), 0, len(t) - 1)
            dt = t[i1] - t[i0]
            ok = dt > 0.4
            rate = np.full(len(t), np.nan); rate[ok] = (deg[i1] - deg[i0])[ok] / dt[ok] / 6.0   # rpm from angle
            # steady: rpm mean over the same window, |rate| > 5 rpm, and rate steady (change < 10% across the window)
            cs = np.r_[0, np.cumsum(rpm)]
            rpm_m = np.full(len(t), np.nan); rpm_m[ok] = (cs[i1 + 1] - cs[i0])[ok] / (i1 - i0 + 1)[ok]
            st = ok & (np.abs(rate) > 5)
            if st.sum() < 50: continue
            ratio = rpm_m[st] / rate[st]
            out[f'ratio_{side}'] = np.median(ratio)
            out[f'nst_{side}'] = int(st.sum())
            # lag of rpm behind the angle's rate: shift rpm by tau (ms) and fit
            if side == 'l':
                sel = np.where(st)[0][::5]
                pts.append(pd.DataFrame(dict(robot=rob, sess=sid, rate=rate[sel], rpm=rpm_m[sel])))
            # motor board update period: distinct angle values per second while the wheel turns > 5 rpm
            inst = np.r_[np.nan, np.diff(deg) / np.diff(t) / 6.0]
            mv = np.abs(rate) > 5
            changes = np.r_[False, np.diff(deg) != 0]
            dms = np.r_[np.nan, np.diff(ms)]
            k = mv & (dms > 15) & (dms < 25)
            out[f'chg_frac_{side}'] = changes[k].mean() if k.sum() > 50 else np.nan
            out[f'raw_dt_{side}'] = np.nanmedian(dms[k]) if k.sum() > 50 else np.nan
            # per-update angle step: degrees per change vs rpm -> period T = step / (rpm*6) 
            stp = np.abs(np.diff(deg)); nz = stp > 0
            kk = nz & mv[1:] & (np.abs(rpm[1:]) > 5)
            if kk.sum() > 50:
                out[f'T_ms_{side}'] = 1000 * np.median(stp[kk] / (np.abs(rate[1:][kk]) * 6.0))
        # turn rate: w_dps (from the board's rpm) vs gyro gz, and vs the wheel angles' difference
        gz = r.gz.values; w = r.w_dps.values
        i0 = np.searchsorted(t, t - 0.25); i1 = np.clip(np.searchsorted(t, t + 0.25), 0, len(t) - 1); dt = t[i1] - t[i0]; ok = dt > 0.4
        dl = (r.wl_deg.values[i1] - r.wl_deg.values[i0]); dr = (r.wr_deg.values[i1] - r.wr_deg.values[i0])
        wheel_turn = np.full(len(t), np.nan); wheel_turn[ok] = ((dr - dl) * np.pi / 180 * 1.7 / 8.9 * 180 / np.pi)[ok] / dt[ok]
        cg = np.r_[0, np.cumsum(gz)]; cw = np.r_[0, np.cumsum(w)]
        gzm = np.full(len(t), np.nan); wm = np.full(len(t), np.nan)
        gzm[ok] = (cg[i1 + 1] - cg[i0])[ok] / (i1 - i0 + 1)[ok]; wm[ok] = (cw[i1 + 1] - cw[i0])[ok] / (i1 - i0 + 1)[ok]
        spin = ok & (np.abs(gzm) > 15)
        if spin.sum() > 50:
            out['w_over_gz'] = np.median(wm[spin] / gzm[spin])
            out['wheelturn_over_gz'] = np.median(wheel_turn[spin] / gzm[spin])
            out['nspin'] = int(spin.sum())
        # drive speed: v_cms vs the angle rate * r
        v = r.v_cms.values; cv = np.r_[0, np.cumsum(v)]; vm = np.full(len(t), np.nan); vm[ok] = (cv[i1 + 1] - cv[i0])[ok] / (i1 - i0 + 1)[ok]
        vang = np.full(len(t), np.nan); vang[ok] = (0.5 * (dl + dr) * np.pi / 180 * 1.7)[ok] / dt[ok]
        dr_ = ok & (np.abs(vang) > 2)
        if dr_.sum() > 50: out['v_over_angle'] = np.median(vm[dr_] / vang[dr_])
        rows.append(out)
R = pd.DataFrame(rows); R.to_pickle(H + '/rpm_runs.pkl'); pd.concat(pts).to_pickle(H + '/rpm_pts.pkl')
pd.set_option('display.width', 250); pd.set_option('display.max_columns', 30)
cols = ['ratio_l','ratio_r','chg_frac_l','chg_frac_r','T_ms_l','T_ms_r','raw_dt_l','w_over_gz','wheelturn_over_gz','v_over_angle']
print(R.groupby('robot')[cols].median().round(3).T)
print(R.groupby(['sess','robot'])[['ratio_l','ratio_r','T_ms_l','w_over_gz','wheelturn_over_gz']].median().round(3))
print('ratio spread across runs', R[['ratio_l','ratio_r']].describe().round(3))
