import json,csv,math,collections,matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
SC='/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/'
OUT=SC+'peek/docs/screenshots/2026-11-12/'
O=json.load(open(SC+'lossaudit/onsets.json'))
# 1: how divergences start, by era
cats=sorted(set(o['cat'] for o in O))
eras=[('5 Nov (2 runs)',lambda r:r.startswith('20261105')),('7-9 Nov (4 runs)',lambda r:r[6:8] in('07','09')),('11 Nov (2 runs)',lambda r:r.startswith('20261111'))]
fig,ax=plt.subplots(figsize=(9.6,4.4))
cols=['#9aa5b1','#d97706','#1f6fb4']
import numpy as np
y=np.arange(len(cats))
for k,(nm,f) in enumerate(eras):
    v=[sum(1 for o in O if f(o['run']) and o['cat']==c) for c in cats]
    ax.barh(y+(k-1)*0.26,v,0.26,color=cols[k],label=nm)
ax.set_yticks(y); ax.set_yticklabels([c[2:] for c in cats],fontsize=8); ax.invert_yaxis()
ax.set_xlabel('episodes where a robot\'s own pose came 8 cm or more off the camera for 3 s or longer'); ax.legend(fontsize=8)
ax.set_title('How a pinned-down robot comes apart from the camera (147 episodes, 8 home runs)',fontsize=10)
fig.text(0.01,0.01,'Data: follow.csv (own pose) against bridge/track.csv (camera), events.csv and tracking.csv, home free roams 5-11 Nov. Each episode classified by what happened in the 3 s before it began.\n'
 'On 5 Nov almost every episode was slow drift while the robot refused every fix; by 11 Nov it was mostly physical events: wheels turning without moving, bumps, and camera mix-ups.',fontsize=6.6)
fig.tight_layout(rect=(0,0.07,1,1)); fig.savefig(OUT+'loss-audit-problem-how-divergences-start-chart.png',dpi=130); plt.close(fig)
# 2: the gate against measured error
d=np.linspace(0,100,201)
gate=np.maximum(3.0,4*np.sqrt(0.01*d+1.0))
med=2.3+0.06*d; p90=2.3+0.28*d
fig,ax=plt.subplots(figsize=(8,4.2))
ax.plot(d,gate,color='#b91c1c',lw=2.2,label='robot accepts a camera fix only inside this (firmware fix gate: 4 sd, sd from pose_q_cm 0.01)')
ax.plot(d,med,color='#1f6fb4',lw=1.8,label='real error, typical (fleet Theil-Sen fit: 2.3 cm + 0.06 cm per cm)')
ax.plot(d,p90,color='#1f6fb4',lw=1.2,ls='--',label='real error, 1 drive in 10 is worse than this (0.28 cm per cm)')
ax.fill_between(d,gate,p90,where=p90>gate,color='#fecaca',alpha=.6)
ax.set_xlabel('cm driven since its own pose last took a camera fix'); ax.set_ylabel('cm'); ax.set_ylim(0,32); ax.legend(fontsize=7,loc='upper left')
ax.set_title('The ratchet: past about 40 cm, the robot refuses even an honest camera fix',fontsize=10)
fig.text(0.01,0.01,'Data: firmware/home_lab/pose.h and params.h (gate); real error from docs/results/replay-all-nov12 (559 blind stretches, 10 home runs). The camera\'s own sd taken as 1 cm.\n'
 'In the pink area the camera is right and the robot is wrong, but the robot throws the fix away, so the error stays until the Pi forces a new pose.',fontsize=6.6)
fig.tight_layout(rect=(0,0.07,1,1)); fig.savefig(OUT+'loss-audit-problem-fix-gate-ratchet-chart.png',dpi=130); plt.close(fig)
# 3: Right 155 s
R=SC+'home0858/run/20261111-085837-freeroam'
L=[l for l in csv.DictReader(open(R+'/lost.csv')) if l['robot']=='Right' and l['dur_s']=='155.72'][0]
t0,t1=float(L['t0']),float(L['t1'])
C=[(float(r['t'])-t0,float(r['x']),float(r['y'])) for r in csv.DictReader(open(R+'/bridge/track.csv')) if r['robot']=='Right' and r['status']=='seen' and r['x'] and t0-20<=float(r['t'])<=t1+10]
F=[(float(r['t_recv'])-t0,float(r['x_cm']),float(r['y_cm'])) for r in csv.DictReader(open(R+'/follow.csv')) if r['robot']=='Right' and t0-20<=float(r['t_recv'])<=t1+10]
fig,ax=plt.subplots(figsize=(8,3.8))
ax.plot([c[0] for c in C],[c[1] for c in C],'.',ms=3,color='#1f6fb4',label='camera: x of Right (both cameras agree)')
ax.plot([f[0] for f in F],[f[1] for f in F],'-',lw=1.6,color='#b91c1c',label='Right\'s own pose: x')
ax.axvspan(0,t1-t0,color='#fde68a',alpha=.5,label='the run called it LOST (and asked for a hand)')
ax.set_xlabel('seconds from the start of the "lost" spell'); ax.set_ylabel('x (cm)'); ax.legend(fontsize=7,loc='center right')
ax.set_title('Right was "lost" for 156 s while the cameras saw it standing still, 27 cm from where it thought it was',fontsize=9.5)
fig.text(0.01,0.01,'Data: home run 11 Nov 08:58 (20261111-085837-freeroam), bridge/track.csv and follow.csv. The robot did not drive (0.0 cm) during the spell.\n'
 'The camera was right all along; the software would not believe a spot that far from the robot\'s own pose while it stood still (old code: e17-e19 now correct this case).',fontsize=6.6)
fig.tight_layout(rect=(0,0.08,1,1)); fig.savefig(OUT+'loss-audit-problem-lost-but-seen-right-155s-chart.png',dpi=130); plt.close(fig)
print('ok')
