import csv,sys,datetime,matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
R,O=sys.argv[1],sys.argv[2]
br=[r for r in csv.DictReader(open(R+'/bridge/track.csv')) if r['status']=='seen']
fo=list(csv.DictReader(open(R+'/follow.csv')))
a,b=1794404903,1794404926
fig,ax=plt.subplots(figsize=(6.4,5.4))
for n,c in (('Center','#1f6fb4'),('Right','#c2410c')):
    p=[(float(r['x']),float(r['y'])) for r in br if r['robot']==n and a<float(r['t'])<b]
    ax.plot([q[0] for q in p],[q[1] for q in p],'-',color=c,lw=2,label=n+': camera')
    ax.add_patch(plt.Circle(p[-1],5,fill=False,ec=c))
q=[(float(r['x_cm']),float(r['y_cm'])) for r in fo if r['robot']=='Center' and a<float(r['t_recv'])<b]
ax.plot([v[0] for v in q],[v[1] for v in q],'--',color='#1f6fb4',lw=1.5,label="Center: where the run thought it was")
ax.axhline(122,color='k',lw=1); ax.text(52,122.6,'board edge (y = 122 cm)',fontsize=8)
ax.set_xlim(50,100); ax.set_ylim(85,132); ax.set_aspect('equal')
ax.set_xlabel('x (cm)'); ax.set_ylabel('y (cm)'); ax.legend(fontsize=8,loc='lower left')
ax.set_title('08:48:23-08:48:46: Center drives into Right',fontsize=10)
fig.text(0.01,0.005,'Source: bridge/track.csv (camera), follow.csv (Center\'s own pose), run 20261111-082651-freeroam. Circles: final positions, 10 cm across (about a robot\'s width).',fontsize=6.5)
fig.tight_layout(rect=(0,0.03,1,1)); fig.savefig(O+'/01-push.png',dpi=150)
c=list(csv.DictReader(open(R+'/cpu.csv')))
t=[(float(r['t'])-float(c[0]['t']))/60 for r in c]
fig,ax=plt.subplots(figsize=(7,3.4))
for k,col,lab in (('core0','#94a3b8','core 0 (recorder)'),('core1','#64748b','core 1 (recorder)'),('core2','#dc2626','core 2 (tag reader + planner)'),('core3','#f97316','core 3 (tag reader + planner)')):
    ax.plot(t,[float(r[k]) for r in c],color=col,lw=1,label=lab)
ax.set_ylim(50,101); ax.set_xlabel('minutes into the run'); ax.set_ylabel('core busy (%)'); ax.legend(fontsize=7,ncol=2,loc='lower right')
fig.text(0.01,0.01,'Source: cpu.csv, home Pi, run 20261111-082651-freeroam (code 4732a48, 2 cameras, video on).',fontsize=6.5)
fig.tight_layout(rect=(0,0.04,1,1)); fig.savefig(O+'/02-cores.png',dpi=150)
