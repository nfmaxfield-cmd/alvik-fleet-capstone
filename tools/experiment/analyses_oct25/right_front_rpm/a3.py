import glob, pandas as pd, numpy as np, os
C = os.environ.get('RF_WORK', '/tmp/claude-0/rf_work') + '/cache'
Z = ['dL_cm','dCL_cm','dC_cm','dCR_cm','dR_cm','dtop_cm','dbot_cm']
RICH = ['e02-20261022-095125','e02-20261022-200532','e02-20261023-085107','e02-20261023-114620','e02-20261023-191347','e02-20261024-153612','e02-20261024-161612']
out = []
for sid in RICH:
    df = pd.read_pickle(f'{C}/{sid}.pkl')
    for (rob, run), g in df.groupby(['robot','run']):
        g = g.sort_values('ms')
        sec = (g.ms // 1000).values
        for s, w in g.groupby(sec):
            if len(w) < 20: continue
            wheel = np.abs(np.diff(w.wl_deg.values)).sum() + np.abs(np.diff(w.wr_deg.values)).sum()
            row = dict(sess=sid, robot=rob, still=wheel < 1.0, moving=wheel > 50, n=len(w))
            for z in Z:
                v = w[z].values
                row[z+'_sd'] = np.std(v)
                row[z+'_nuniq'] = len(np.unique(v))
                row[z+'_med'] = np.median(v)
            out.append(row)
W = pd.DataFrame(out)
W.to_pickle(os.environ.get('RF_WORK', '/tmp/claude-0/rf_work') + '/windows.pkl')
pd.set_option('display.width', 250); pd.set_option('display.max_columns', 30)
print('windows', W.groupby(['robot']).agg(still=('still','mean'), moving=('moving','mean'), n=('n','size')))
for lab, k in (('STILL', W.still), ('MOVING', W.moving)):
    print(lab, 'median 1 s SD (cm)')
    print(W[k].groupby('robot')[[z+'_sd' for z in Z]].median().round(2))
print('STILL median level (cm)')
print(W[W.still].groupby('robot')[[z+'_med' for z in Z]].median().round(1))
