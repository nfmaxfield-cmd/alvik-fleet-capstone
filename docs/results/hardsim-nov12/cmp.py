import json,sys
def g(d,*k):
    for x in k:
        if d is None: return None
        d=d.get(x) if isinstance(d,dict) else None
    return d
rows=[('pics p50',('pictures_s','p50')),('delay p10',('fix_delay_s','p10')),('delay p50',('fix_delay_s','p50')),('delay p90',('fix_delay_s','p90')),('delay p99',('fix_delay_s','p99')),
('quiet p50',('quiet_s','p50')),('quiet p90',('quiet_s','p90')),('quiet p99',('quiet_s','p99')),('stalls/rh',('stalls','per_robot_h')),('stall p50',('stalls','length_s','p50')),('stall p90',('stalls','length_s','p90')),('shared%',('stalls','shared_pct')),
('wd/h',('watchdog_per_h',)),('unnamed/rh',('unnamed','per_robot_h')),('unn>5/rh',('unnamed','over_5s_per_robot_h')),('unn p50',('unnamed','length_s','p50')),('unn longest',('unnamed','longest_s')),
('gaps/rh',('cam_gaps','per_robot_h')),('gap p50',('cam_gaps','length_s','p50')),
('own-cam p50',('own_cam_gap_cm','p50')),('own-cam p90',('own_cam_gap_cm','p90')),('own-cam p99',('own_cam_gap_cm','p99')),
('hdg/rh',('heading_err_deg','per_robot_h')),('dnm/rh',('did_not_move','per_robot_h')),('hand/rh',('needs_hand','per_robot_h')),('rejoin/rh',('rejoined','per_robot_h')),('lost/rh',('lost','per_robot_h')),('lost stand/rh',('lost','standing_per_robot_h')),('lost p50',('lost','length_s','p50'))]
cols=[]
for f in sys.argv[1:]:
    d=json.load(open(f))
    if 'runs' in d: cols.append(('real',d['runs'][0]))
    else: cols.append((f.split('/')[-1][:10],d['measures']))
print('%-14s'%'',' '.join('%10s'%c for c,_ in cols))
for n,k in rows: print('%-14s'%n,' '.join('%10s'%g(m,*k) for _,m in cols))
