"""regress.py : ordinary least squares of wheel error against speed, per robot and pooled, for dataset A
(stretches.csv: the sensor-trust check's stretches) and dataset B (legs.csv: every go leg). Standard errors are
heteroscedasticity-robust (HC3), confidence intervals 95 % (t). Writes OUT/regress.json and prints the tables."""
import os, json, sys
import numpy as np
import pandas as pd
from scipy import stats
from common import OUT

ROBOTS = ['Left', 'Center', 'Right']


def ols(y, X, names):
    """beta, HC3 se, t-based 95 % CI, p, R^2, n. X includes the intercept column."""
    y = np.asarray(y, float)
    X = np.asarray(X, float)
    keep = np.isfinite(y) & np.all(np.isfinite(X), 1)
    y, X = y[keep], X[keep]
    # drop all-zero / duplicate columns (a session with no rows for this robot)
    cols = [j for j in range(X.shape[1]) if np.ptp(X[:, j]) > 0 or j == 0]
    X = X[:, cols]
    names = [names[j] for j in cols]
    n, p = X.shape
    XtXi = np.linalg.pinv(X.T @ X)
    b = XtXi @ X.T @ y
    e = y - X @ b
    h = np.einsum('ij,jk,ik->i', X, XtXi, X)
    u = e / np.clip(1 - h, 1e-6, None)
    cov = XtXi @ (X.T * u ** 2) @ X @ XtXi
    se = np.sqrt(np.clip(np.diag(cov), 0, None))
    dof = max(1, n - p)
    tq = stats.t.ppf(0.975, dof)
    r2 = 1 - (e @ e) / (((y - y.mean()) ** 2).sum()) if n > 1 else float('nan')
    out = {nm: dict(b=float(b[j]), se=float(se[j]), lo=float(b[j] - tq * se[j]), hi=float(b[j] + tq * se[j]),
                    p=float(2 * stats.t.sf(abs(b[j] / se[j]), dof)) if se[j] > 0 else float('nan'))
           for j, nm in enumerate(names)}
    return dict(coef=out, r2=float(r2), n=int(n), p=int(p), cov=cov, names=names, b=b, resid=e, fitted=X @ b,
                keep=keep)


def design(d, speed, covars, robot_inter=False):
    cols, names = [np.ones(len(d))], ['const']
    if robot_inter:
        for r in ('Center', 'Right'):
            cols.append((d.robot == r).astype(float).values); names.append('robot=' + r)
        cols.append(d[speed].values); names.append(speed)
        for r in ('Center', 'Right'):
            cols.append(((d.robot == r) * d[speed]).values); names.append(speed + ':' + r)
    else:
        cols.append(d[speed].values); names.append(speed)
    for c in covars:
        if c in ('kind', 'session'):
            lv = sorted(d[c].dropna().unique())
            for v in lv[1:]:
                cols.append((d[c] == v).astype(float).values); names.append('%s=%s' % (c, v))
        else:
            cols.append(d[c].values.astype(float)); names.append(c)
    return np.column_stack(cols), names


def wald(fit, names):
    idx = [fit['names'].index(n) for n in names if n in fit['names']]
    if not idx:
        return None
    b = fit['b'][idx]
    V = fit['cov'][np.ix_(idx, idx)]
    W = float(b @ np.linalg.pinv(V) @ b)
    q = len(idx)
    F = W / q
    p = float(stats.f.sf(F, q, max(1, fit['n'] - fit['p'])))
    return dict(F=F, df1=q, df2=fit['n'] - fit['p'], p=p)


def prep_a():
    d = pd.read_csv(os.path.join(OUT, 'stretches.csv'))
    d = d[d.experiment.isin(['find', 'e02'])].copy()     # the runs learn() uses (calib: none paired)
    d['abs_err'] = d.err_cm.abs()
    d['length'] = d.wheel_cm
    d['battery'] = d.battery.fillna(d.groupby('session').battery.transform('mean'))
    return d


def prep_b():
    d = pd.read_csv(os.path.join(OUT, 'legs.csv'))
    d = d[d.still & ~d.fix & d.err_cm.notna() & d.cms.notna()].copy()
    d['abs_err'] = d.err_cm.abs()
    d['length'] = d.wheel_cm
    d['battery'] = d.battery.fillna(d.groupby('session').battery.transform('mean'))
    return d


def summarize(tag, d, resp, speed, covars):
    res = {}
    for r in ROBOTS + ['all']:
        dd = d if r == 'all' else d[d.robot == r]
        if len(dd) < 8:
            continue
        cv = covars + (['robot_c', 'robot_r'] if r == 'all' else [])
        if r == 'all':
            dd = dd.assign(robot_c=(dd.robot == 'Center').astype(float), robot_r=(dd.robot == 'Right').astype(float))
        X, nm = design(dd, speed, cv)
        f = ols(dd[resp].values, X, nm)
        X0, nm0 = design(dd, speed, [])
        f0 = ols(dd[resp].values, X0, nm0)
        Xc, nmc = design(dd, speed, cv)
        # R^2 of the covariates alone (speed taken out): what speed adds
        Xn = np.delete(Xc, nmc.index(speed), 1)
        fn = ols(dd[resp].values, Xn, [n for n in nmc if n != speed])
        res[r] = dict(n=f['n'], slope=f['coef'][speed], r2_full=f['r2'], r2_speed_only=f0['r2'],
                      r2_without_speed=fn['r2'], slope_alone=f0['coef'][speed])
    # does Center's slope differ? pooled model with robot x speed terms (Left the reference)
    X, nm = design(d, speed, covars, robot_inter=True)
    fi = ols(d[resp].values, X, nm)
    res['interaction'] = dict(center_minus_left=fi['coef'].get(speed + ':Center'),
                              right_minus_left=fi['coef'].get(speed + ':Right'),
                              both=wald(fi, [speed + ':Center', speed + ':Right']),
                              center_level=fi['coef'].get('robot=Center'))
    # Center against Left and Right together (one contrast)
    d2 = d.assign(center=(d.robot == 'Center').astype(float))
    d2 = d2.assign(cxs=d2.center * d2[speed])
    X2, nm2 = design(d2, speed, covars + ['center', 'cxs'])
    f2 = ols(d2[resp].values, X2, nm2)
    res['center_vs_others'] = dict(slope_diff=f2['coef'].get('cxs'), level_diff=f2['coef'].get('center'))
    return res


if __name__ == '__main__':
    A, B = prep_a(), prep_b()
    out = {}
    specs = [('A', A, 'err_cm', 'cam_speed', ['length', 'kind', 'session', 'battery']),
             ('A', A, 'err_cm', 'wheel_speed', ['length', 'kind', 'session', 'battery']),
             ('A', A, 'abs_err', 'cam_speed', ['length', 'kind', 'session', 'battery']),
             ('A', A, 'abs_err', 'wheel_speed', ['length', 'kind', 'session', 'battery']),
             ('B', B, 'err_cm', 'cms', ['length', 'kind', 'session', 'battery']),
             ('B', B, 'abs_err', 'cms', ['length', 'kind', 'session', 'battery']),
             ('B', B, 'err_cm', 'cam_speed', ['length', 'kind', 'session', 'battery']),
             ('B', B, 'abs_err', 'cam_speed', ['length', 'kind', 'session', 'battery']),
             ('B', B, 'err_cm', 'wheel_speed', ['length', 'kind', 'session', 'battery'])]
    for ds, d, resp, sp, cv in specs:
        key = '%s|%s|%s' % (ds, resp, sp)
        out[key] = summarize(key, d, resp, sp, cv)
        print('==', key, 'n', len(d))
        for r in ROBOTS + ['all']:
            if r in out[key]:
                q = out[key][r]
                s = q['slope']
                print('  %-6s n=%3d slope %+.3f [%+.3f, %+.3f] p=%.3g | alone %+.3f p=%.3g | R2 full %.2f, speed only %.3f, without speed %.2f' % (
                    r, q['n'], s['b'], s['lo'], s['hi'], s['p'], q['slope_alone']['b'], q['slope_alone']['p'],
                    q['r2_full'], q['r2_speed_only'], q['r2_without_speed']))
        it = out[key]['interaction']
        cv_ = out[key]['center_vs_others']
        print('  robot x speed: both F=%.2f p=%.3g; Center-Left slope %+.3f [%+.3f,%+.3f]; Center vs L+R slope %+.3f [%+.3f,%+.3f] p=%.3g, level %+.3f [%+.3f,%+.3f] p=%.3g' % (
            it['both']['F'], it['both']['p'], it['center_minus_left']['b'], it['center_minus_left']['lo'],
            it['center_minus_left']['hi'], cv_['slope_diff']['b'], cv_['slope_diff']['lo'], cv_['slope_diff']['hi'],
            cv_['slope_diff']['p'], cv_['level_diff']['b'], cv_['level_diff']['lo'], cv_['level_diff']['hi'],
            cv_['level_diff']['p']))
    json.dump(out, open(os.path.join(OUT, 'regress.json'), 'w'), indent=1, default=float)
