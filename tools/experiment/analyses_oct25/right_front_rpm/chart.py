import pandas as pd, numpy as np, os, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
H = os.environ.get('RF_WORK', '/tmp/claude-0/rf_work')
OUT = '/home/claude/alvik-fleet-capstone/docs/build-reports/e02-oct25-right-front-and-rpm.png'
COL = {'Left': '#2a78d6', 'Center': '#eb6834', 'Right': '#1baf7a'}; MK = {'Left': 'o', 'Center': 's', 'Right': '^'}
A = pd.read_pickle(H + '/ahead.pkl')
B = A[(A.exp > 3) & (A.exp < 80) & (A.cos >= 0.8) & (A.cgap < 0.15)].copy()
B['bin'] = (B.exp // 5) * 5 + 2.5
W = pd.read_pickle(H + '/windows.pkl')
Z = ['dL_cm','dCL_cm','dC_cm','dCR_cm','dR_cm']
plt.rcParams.update({'font.size': 10, 'axes.edgecolor': '#52514e', 'axes.labelcolor': '#0b0b0b', 'xtick.color': '#52514e', 'ytick.color': '#52514e'})
fig, ax = plt.subplots(1, 2, figsize=(11, 4.4), facecolor='#fcfcfb')
a = ax[0]; a.set_facecolor('#fcfcfb')
a.plot([0, 80], [0, 80], color='#9a9993', lw=1, ls='--', label='reading = map')
for rob in ('Left', 'Center', 'Right'):
    g = B[B.robot == rob].groupby('bin').dC_cm.median()
    a.plot(g.index, g.values, color=COL[rob], lw=2, marker=MK[rob], ms=6, label=rob, markeredgecolor='#fcfcfb')
a.set_xlabel('Map distance from the robot\'s face to the board ahead (cm)'); a.set_ylabel('Front sensor centre zone dC_cm (cm), median')
a.set_xlim(0, 80); a.set_ylim(0, 80); a.grid(color='#e6e5e0', lw=0.6); a.legend(frameon=False, loc='upper left')
a.set_title('(a) Centre zone against the map, while running', loc='left', fontsize=10.5)
a = ax[1]; a.set_facecolor('#fcfcfb')
rows = []
for i, rob in enumerate(('Left', 'Center', 'Right')):
    g = W[W.robot == rob]
    for z in Z:
        q = g[z + '_sd'].quantile(0.1)
        rows.append((rob, z, g[z + '_sd'].median() / q, g.loc[g.moving, z + '_sd'].median() / q))
R = pd.DataFrame(rows, columns=['robot', 'zone', 'all', 'moving'])
for i, rob in enumerate(('Left', 'Center', 'Right')):
    r = R[R.robot == rob]
    x0 = i * 2
    a.scatter(np.full(len(r), x0) + np.linspace(-0.25, 0.25, len(r)), r['all'], color=COL[rob], marker=MK[rob], s=40, edgecolor='#fcfcfb')
    a.scatter(np.full(len(r), x0 + 0.9) + np.linspace(-0.25, 0.25, len(r)), r['moving'], color=COL[rob], marker=MK[rob], s=40, edgecolor='#fcfcfb', alpha=0.55)
a.set_yscale('log'); a.set_yticks([2, 5, 10, 20, 50]); a.set_yticklabels(['2', '5', '10', '20', '50']); a.minorticks_off(); a.set_xticks([0, 0.9, 2, 2.9, 4, 4.9]); a.set_xticklabels(['Left\nall', 'Left\nmoving', 'Center\nall', 'Center\nmoving', 'Right\nall', 'Right\nmoving'])
a.set_ylabel('Typical 1 s spread / quietest 1 s spread (x, log)'); a.grid(axis='y', color='#e6e5e0', lw=0.6)
a.set_title('(b) Replay\'s "motion response", all 1 s windows vs moving only', loc='left', fontsize=10.5)
fig.text(0.01, 0.01, 'Data: RAW logs (raw.csv) of the 7 rich sessions 2026-10-22 to 10-24, camera track (track.csv), board lines (e02_state.json). '
         '(a) heading = gyro yaw aligned to camera travel per run; boards within 45 deg of head-on; other robots not in the map. (b) one dot per zone (L, CL, C, CR, R).',
         fontsize=7.5, color='#52514e', wrap=True)
fig.tight_layout(rect=(0, 0.06, 1, 1)); fig.savefig(OUT, dpi=130, facecolor='#fcfcfb'); print(R.round(1))
