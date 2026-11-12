import csv,sys,bisect,math,matplotlib,datetime
matplotlib.use('Agg'); import matplotlib.pyplot as plt, matplotlib.dates as md
R,O=sys.argv[1],sys.argv[2]
br=[r for r in csv.DictReader(open(R+'/bridge/track.csv')) if r['status']=='seen']
fo=list(csv.DictReader(open(R+'/follow.csv')))
fig,ax=plt.subplots(figsize=(8,3.6)); out={}
for n,c in (('Left','#15803d'),('Center','#1f6fb4'),('Right','#c2410c')):
    B=[(float(r['t']),float(r['x']),float(r['y'])) for r in br if r['robot']==n]; bt=[b[0] for b in B]
    T,G=[],[]; last=0
    for r in fo:
        if r['robot']!=n or r['state']=='nopose': continue
        t=float(r['t_recv'])
        if t-last<5: continue
        i=bisect.bisect(bt,t)-1
        if i<0 or t-bt[i]>1: continue
        last=t; g=math.hypot(B[i][1]-float(r['x_cm']),B[i][2]-float(r['y_cm']))
        T.append(datetime.datetime.fromtimestamp(t)); G.append(g)
    ax.plot(T,G,'.',ms=2,color=c,label=n); out[n]=G
ax.set_ylabel('gap (cm)'); ax.set_yscale('log'); ax.set_ylim(0.3,100); ax.legend(fontsize=8,markerscale=5)
ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
fig.text(0.01,0.01,'Gap = distance between where the camera saw each robot (bridge/track.csv) and where the robot itself said it was (follow.csv), every 5 s. Run 20261111-085837-freeroam.',fontsize=6.3)
fig.tight_layout(rect=(0,0.04,1,1)); fig.savefig(O+'/03-gap-0858.png',dpi=150)
for n,G in out.items():
    G=sorted(G); print(n,len(G),'median',round(G[len(G)//2],1),'p90',round(G[int(.9*len(G))],1),'over10',round(100*sum(g>10 for g in G)/len(G),1))
