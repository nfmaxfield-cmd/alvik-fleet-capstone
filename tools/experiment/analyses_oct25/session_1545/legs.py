import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *
rob,a,b=sys.argv[1],ep(sys.argv[2]),ep(sys.argv[3])
for f in ['robot-logs-1/leg.csv','robot-logs-1/edge.csv','robot-logs-1/fix.csv','robot-logs-1/bump.csv']:
    print('==',f)
    for r in rows(f):
        t=float(r['t_recv'])
        if r['robot']==rob and a<=t<=b:
            r.pop('robot'); r['t_recv']=lt(t); print(','.join(str(v) for v in r.values()))
