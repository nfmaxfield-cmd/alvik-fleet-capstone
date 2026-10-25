import pandas as pd, numpy as np, os
H = os.environ.get('RF_WORK', '/tmp/claude-0/rf_work')
Z = ['dL_cm','dCL_cm','dC_cm','dCR_cm','dR_cm','dtop_cm','dbot_cm']
A = pd.read_pickle(H + '/ahead.pkl')
k = (A.exp > 3) & (A.exp < 60) & (A.cos >= 0.8) & (A.cgap < 0.15)
B = A[k]
print('rows', B.groupby('robot').size().to_dict())
def fit(g, z):
    x, y = g.exp.values, g[z].values
    m = y - x
    sl = np.polyfit(x, y, 1)
    # robust: share of rows within 5 cm
    return dict(n=len(g), off=np.median(m), spread=np.median(np.abs(m - np.median(m))) * 1.4826, slope=sl[0], icpt=sl[1],
                r=np.corrcoef(x, y)[0, 1], w5=np.mean(np.abs(m - np.median(m)) < 5))
rows = []
for (rob), g in B.groupby('robot'):
    for z in Z: rows.append(dict(robot=rob, zone=z, **fit(g, z)))
R = pd.DataFrame(rows); pd.set_option('display.width', 250)
print(R.round(2).to_string())
rows = []
for (sid, rob), g in B.groupby(['sess', 'robot']):
    if len(g) < 200: continue
    for z in ['dCL_cm', 'dC_cm', 'dCR_cm', 'dL_cm', 'dR_cm']: rows.append(dict(sess=sid[4:], robot=rob, zone=z, **fit(g, z)))
R2 = pd.DataFrame(rows)
print(R2.pivot_table(index=['sess', 'robot'], columns='zone', values='slope').round(2))
print(R2.pivot_table(index=['sess', 'robot'], columns='zone', values='r').round(2))
