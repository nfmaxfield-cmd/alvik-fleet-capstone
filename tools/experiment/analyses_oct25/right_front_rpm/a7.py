import pandas as pd, numpy as np, os
H = os.environ.get('RF_WORK', '/tmp/claude-0/rf_work')
Z = ['dL_cm','dCL_cm','dC_cm','dCR_cm','dR_cm','dtop_cm']
W = pd.read_pickle(H + '/windows.pkl')
pd.set_option('display.width', 250)
S = W[W.still]
L = []
for z in Z:
    for rob, g in S.groupby('robot'):
        near = g[z+'_med'] < 60
        L.append(dict(zone=z, robot=rob, still_far_frac=1 - near.mean(), sd_near=g.loc[near, z+'_sd'].median(), sd_far=g.loc[~near, z+'_sd'].median()))
print(pd.DataFrame(L).pivot_table(index='zone', columns='robot', values=['still_far_frac','sd_near','sd_far']).round(2))
# the replay's ratio, rebuilt: median 1 s window SD / 10th percentile of window SDs, over (a) all windows (b) moving only (c) still only
L = []
for rob, g in W.groupby('robot'):
    for z in Z:
        q = g[z+'_sd'].quantile(0.1)
        L.append(dict(robot=rob, zone=z, all=g[z+'_sd'].median()/q, moving=g.loc[g.moving, z+'_sd'].median()/q, still=g.loc[g.still, z+'_sd'].median()/q, q10=q))
print(pd.DataFrame(L).pivot_table(index='zone', columns='robot', values=['all','moving','still']).round(1))
print(pd.DataFrame(L).pivot_table(index='zone', columns='robot', values='q10').round(2))
