import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *
import collections
st={}
out=collections.defaultdict(list)
for r in rows('robot-logs-1/leg.csv'):
    if r['kind']!='turn': continue
    k=(r['robot'],r['id'])
    if r['result']=='started': st[k]=(float(r['th_deg']), float(r['t_recv']))
    elif r['result'] in ('done','failed') and k in st:
        th0,t0=st.pop(k); d=((float(r['th_deg'])-th0+180)%360)-180
        cmd=float(r['value'])
        out[r['robot']].append((lt(t0),cmd,d,r['result'],r['info'],float(r['t_recv'])-t0))
for rob,v in out.items():
    res=collections.Counter((x[3]+':'+x[4].split('(')[0][:20]) for x in v)
    big=[x for x in v if abs(x[1])>=20]
    wrong=[x for x in big if x[1]*x[2]<0]
    err=[abs(((x[2]-x[1]+180)%360)-180) for x in big if x[3]=='done']
    print(rob,len(v),dict(res),'| >=20deg:',len(big),'wrong sign:',len(wrong), 'done median err', sorted(err)[len(err)//2] if err else None)
if len(sys.argv)>1:
    for x in out[sys.argv[1]]:
        if x[0]>=sys.argv[2] and x[0]<=sys.argv[3]: print(x)
