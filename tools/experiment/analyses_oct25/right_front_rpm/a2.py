import glob, os, pandas as pd, numpy as np
best = {}
for p in glob.glob('/tmp/claude-0/**/front_checks.csv', recursive=True):
    if '/-home-claude' in p: continue
    sid = os.path.basename(os.path.dirname(p))
    n = sum(1 for _ in open(p))
    if sid not in best or n > best[sid][0]: best[sid] = (n, p)
rows = []
for sid,(n,p) in sorted(best.items()):
    d = pd.read_csv(p); d['sess'] = sid; rows.append(d)
fc = pd.concat(rows)
fc['expected_cm'] = pd.to_numeric(fc.expected_cm, errors='coerce')
fc['miss'] = fc.front_cm - fc.expected_cm
fc.to_pickle(os.environ.get('RF_WORK', '/tmp/claude-0/rf_work') + '/fc.pkl')
pd.set_option('display.width', 250)
# square-on board, within 50 cm, heading measured
ok = fc.what.astype(str).str.contains('board') & (fc.expected_cm <= 50)
print('all rows', len(fc), 'board<=50', ok.sum())
g = fc[ok].groupby(['robot'])
def summ(s):
    m = s.miss; return pd.Series(dict(n=len(m), med=m.median(), mad=(m-m.median()).abs().median()*1.4826,
        within6=(m.abs()<=6).mean(), slope=np.polyfit(s.expected_cm, s.front_cm,1)[0] if len(m)>3 else np.nan,
        r=np.corrcoef(s.expected_cm, s.front_cm)[0,1] if len(m)>3 else np.nan))
print(fc[ok].groupby('robot').apply(summ).round(2))
print(fc[ok].groupby(['sess','robot']).apply(summ).round(2))
print(fc[ok & (fc.heading_measured==1)].groupby('robot').apply(summ).round(2))
