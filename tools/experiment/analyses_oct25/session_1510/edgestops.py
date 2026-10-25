import csv, sys
sys.argv = ['x', '15:00:00', '15:00:01']
exec(open('/tmp/claude-0/s1025/work/trk.py').read().split("if __name__")[0])
legs = list(csv.DictReader(open(L + 'robot-logs-1/leg.csv'), fieldnames=None))
for g in legs:
    if g['result'] == 'failed' and g['kind'] == 'go' and 'edge' in (g['info'] or ''):
        t = float(g['t_recv'])
        near = [q for q in rows if q['r'] == g['robot'] and t - 0.4 <= q['t'] <= t + 0.6]
        if not near: print(T(t), g['robot'], 'no track'); continue
        q = near[-1]
        print(T(t), g['robot'], g['value'], g['info'], 'px(%.0f,%.0f) roam x %.1f y %.1f' % (q['u'], q['v'], q['gx'], q['gy']))
