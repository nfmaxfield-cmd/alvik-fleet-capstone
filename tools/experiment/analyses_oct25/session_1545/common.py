import csv, time, os
D='/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/s1545/dl/logs/e02-20261025-154522'
R='/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/s1545/dl/rec/20261025-154514_E02'
def lt(t):
    t=float(t); s=time.strftime('%H:%M:%S', time.gmtime(t-4*3600)); return s+'.%d'%int((t%1)*10)
def ep(s):
    h,m,sec=s.split(':'); 
    import calendar
    return calendar.timegm((2026,10,25,int(h)+4,int(m),0))+float(sec)
def rows(f): return list(csv.DictReader(open(os.path.join(D,f))))
