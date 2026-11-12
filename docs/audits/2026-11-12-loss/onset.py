"""How does a robot's own pose come apart from the camera? For every divergence episode (own vs camera gap over 8 cm
for 3 s or more), find its onset (the last moment the gap was under 3 cm) and what happened in the 4 s around it."""
import csv, sys, math, bisect, json, collections, os
SC='/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/'
RUNS=[SC+'rall/un/fr1105-143553/20261105-143553-freeroam', SC+'rall/un/fr1105-155001/20261105-155001-freeroam',
      SC+'rall/un/fr1107-122722/20261107-122722-freeroam', SC+'rall/un/fr1107-133604/20261107-133604-freeroam',
      SC+'rall/un/fr1107-144400/20261107-144400-freeroam', SC+'rall/un/fr1109-123542/20261109-123542-freeroam',
      SC+'home1111/run/20261111-082651-freeroam', SC+'home0858/run/20261111-085837-freeroam']
out=[]
for R in RUNS:
    run=os.path.basename(R)
    fo=collections.defaultdict(list)
    for r in csv.DictReader(open(R+'/follow.csv')):
        if r['state']=='nopose': continue
        try: fo[r['robot']].append((float(r['t_recv']),float(r['x_cm']),float(r['y_cm']),float(r['th_deg']),r['run']))
        except ValueError: pass
    cam=collections.defaultdict(list)
    allcam=[]
    for r in csv.DictReader(open(R+'/bridge/track.csv')):
        if r['status']!='seen' or not r['x']: continue
        q=(float(r['t']),float(r['x']),float(r['y']),r.get('src',''),r.get('why',''))
        cam[r['robot']].append(q); allcam.append((q[0],r['robot'],q[1],q[2]))
    ev=collections.defaultdict(list)
    try:
        for e in csv.DictReader(open(R+'/events.csv')): ev[e['robot']].append((float(e['t']),e['kind'],e['text']))
    except FileNotFoundError: pass
    cp=[]
    try:
        for e in csv.DictReader(open(R+'/tracking.csv')): cp.append((float(e['t']),e['robot'],e['what']))
    except FileNotFoundError: pass
    for rb,C in cam.items():
        F=fo.get(rb)
        if not F or len(C)<50: continue
        ft=[f[0] for f in F]
        def own(t):
            i=bisect.bisect(ft,t)
            if i<=0 or i>=len(F): return None
            a,b=F[i-1],F[i]
            if b[0]-a[0]>1.0: return None
            k=(t-a[0])/(b[0]-a[0]); return (a[1]+k*(b[1]-a[1]),a[2]+k*(b[2]-a[2]),a[3],b[4])
        G=[]
        for c in C:
            o=own(c[0])
            if o: G.append((c[0],math.hypot(c[1]-o[0],c[2]-o[1]),c,o))
        i=0
        while i<len(G):
            if G[i][1]>8:
                j=i
                while j<len(G) and G[j][1]>6: j+=1
                if G[j-1][0]-G[i][0]>=3.0:
                    k=i
                    while k>0 and G[k][1]>=3: k-=1
                    t_on=G[k][0]; t_big=G[i][0]
                    win=(t_on-3,t_big+1)
                    kinds=sorted(set(e[1] for e in ev.get(rb,[]) if win[0]<=e[0]<=win[1]))
                    cps=[x for x in cp if win[0]<=x[0]<=win[1] and rb in x[1]]
                    whys=sorted(set(g[2][4][:30] for g in G[k:i+1] if g[2][4]))
                    # jumps: own pose jump (reseed applied), camera spot jump
                    ownj=max([math.hypot(F[m][1]-F[m-1][1],F[m][2]-F[m-1][2]) for m in range(1,len(F)) if win[0]<=F[m][0]<=win[1] and F[m][0]-F[m-1][0]<0.3] or [0])
                    cj=[math.hypot(C2[1]-C1[1],C2[2]-C1[2])/max(0.05,C2[0]-C1[0]) for C1,C2 in zip(C,C[1:]) if win[0]<=C2[0]<=win[1]]
                    camv=max(cj or [0])
                    # nearest other robot (camera) at onset
                    near=min([math.hypot(a[2]-G[i][2][1],a[3]-G[i][2][2]) for a in allcam if a[1]!=rb and abs(a[0]-t_big)<0.5] or [999])
                    # own heading rate around onset
                    hs=[(F[m][0],F[m][3]) for m in range(len(F)) if win[0]<=F[m][0]<=win[1]]
                    turn=max([abs((b[1]-a[1]+180)%360-180)/max(0.05,b[0]-a[0]) for a,b in zip(hs,hs[1:])] or [0])
                    rstart=len(set(F[m][4] for m in range(len(F)) if win[0]<=F[m][0]<=win[1]))>1
                    # own travel and camera travel between onset and big
                    o0=own(t_on); o1=own(t_big)
                    ownmv=math.hypot(o1[0]-o0[0],o1[1]-o0[1]) if o0 and o1 else None
                    c0=G[k][2]; c1=G[i][2]; cammv=math.hypot(c1[1]-c0[1],c1[2]-c0[2])
                    out.append(dict(run=run,robot=rb,t_on=round(t_on,1),build_s=round(t_big-t_on,2),dur_s=round(G[j-1][0]-G[i][0],1),
                        peak=round(max(g[1] for g in G[i:j]),1),kinds=kinds,closepass=len(cps),bridge_why=whys,own_jump=round(ownj,1),
                        cam_speed_max=round(camv,1),near_other_cm=round(near,1),turn_rate=round(turn,1),restart=rstart,
                        own_moved=None if ownmv is None else round(ownmv,1),cam_moved=round(cammv,1)))
                i=j
            else: i+=1
json.dump(out,open(SC+'lossaudit/onsets.json','w'),indent=1)
print(len(out))
