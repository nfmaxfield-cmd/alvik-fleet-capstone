import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sys; from common import *
rob,a,b=sys.argv[1],ep(sys.argv[2]),ep(sys.argv[3])
for r in rows('track.csv'):
    t=float(r['t'])
    if r['robot']==rob and a<=t<=b: print(lt(t), r['x_cm'], r['y_cm'], r['phase'], r['source'], r['lock_score'])
