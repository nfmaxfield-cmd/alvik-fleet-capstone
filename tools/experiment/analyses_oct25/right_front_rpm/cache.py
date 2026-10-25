import sys, glob, os, pandas as pd, numpy as np
sys.path.insert(0, '/home/claude/alvik-fleet-capstone/tools/discovery')
import load
OUT = os.environ.get('RF_WORK', '/tmp/claude-0/rf_work') + '/cache'
os.makedirs(OUT, exist_ok=True)
COLS = ['t_recv','ms','step','wl_deg','wr_deg','wl_rpm','wr_rpm','v_cms','w_dps','x_cm','y_cm','th_deg','gz',
        'dL_cm','dCL_cm','dC_cm','dCR_cm','dR_cm','dtop_cm','dbot_cm','raw_n','battery','yaw_deg']
for sid, sdir in load.find_sessions().items():
    parts = []
    for p in sorted(glob.glob(os.path.join(sdir, 'robot-logs-*', 'raw.csv'))):
        df = pd.read_csv(p, dtype=str, on_bad_lines='skip')
        df = df[df['robot'] != 'robot']
        df['src'] = os.path.basename(os.path.dirname(p))
        parts.append(df)
    df = pd.concat(parts)
    for c in COLS:
        if c in df: df[c] = pd.to_numeric(df[c], errors='coerce')
    keep = ['robot','run','src'] + [c for c in COLS if c in df]
    df = df[keep].drop_duplicates(subset=['robot','run','ms'])
    df.to_pickle(f'{OUT}/{sid}.pkl')
    print(sid, len(df), df.robot.value_counts().to_dict())
