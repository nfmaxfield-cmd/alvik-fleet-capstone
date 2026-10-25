import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *
import collections, statistics as S
P=rows('perf.csv')
T=rows('track.csv')
src=collections.defaultdict(collections.Counter)
for r in T: src[int(float(r['t'])//60)][r['source']]+=1
by=collections.defaultdict(list)
for r in P: by[int(float(r['t'])//60)].append(r)
print('min  fps  ms_mean ms_max age  drop phase | track sources')
for m in sorted(by):
    v=by[m]
    f=S.median(float(r['fps_handled']) for r in v); ms=S.median(float(r['ms_mean']) for r in v)
    mx=max(float(r['ms_max']) for r in v); age=S.median(float(r['age_mean_s']) for r in v)
    dr=sum(int(float(r['dropped'] or 0)) for r in v)
    ph=collections.Counter(r['phase'] for r in v).most_common(1)[0][0]
    sc=collections.Counter(r['source'] for r in v).most_common(2)
    print(lt(m*60)[:5], '%5.1f %6.1f %6.0f %5.2f %4d %-9s %s | %s'%(f,ms,mx,age,dr,ph,sc,dict(src[m])))
