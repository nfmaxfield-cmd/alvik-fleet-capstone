"""edgetable.py OUT.csv : every edge event of both stages (the robots' own edge stops from edge.csv, the edge finds'
touches and barrier, border holds and camera-watchdog stops from e02_state.json) with the camera's distance from the
nearest open-edge line at the moment (median of the track.csv spots within 0.3 s, 'held' left out) and the closest it
got in the 4 s after (negative = past the line)."""
import csv, json, os, sys, bisect, statistics as stx
from common import STAGE, track, edge_d, lt
rows = []
for st in ('E1', 'E2'):
    tk = track(st)
    def cam(robot, t, w=0.3):
        a = [q for q in tk.get(robot, []) if q[3] != 'held' and t - w <= q[0] <= t + w]
        if not a: return None, None
        x, y = stx.median(q[1] for q in a), stx.median(q[2] for q in a)
        d, side = edge_d(x, y); return round(d, 1), side
    def closest(robot, t0, t1):
        a = [edge_d(q[1], q[2])[0] for q in tk.get(robot, []) if q[3] != 'held' and t0 <= q[0] <= t1]
        return round(min(a), 1) if a else None
    d = STAGE[st]
    for r in csv.DictReader(open(os.path.join(d, 'robot-logs-1', 'edge.csv'))):
        if r['event'] != 'trip': continue
        t = float(r['t_recv']); c, side = cam(r['robot'], t)
        rows.append(dict(stage=st, t=lt(t), robot=r['robot'], kind='own edge stop (%s)' % r['info'].replace('line sensors', 'line').replace('bottom zone', 'bottom'),
                         lines='%s/%s/%s' % (r['lineL'], r['lineC'], r['lineR']), base='%s/%s/%s' % (r['baseL'], r['baseC'], r['baseR']),
                         dbot=r['dbot_cm'], cam_cm=c, side=side, closest_4s=closest(r['robot'], t - 0.5, t + 4), note=''))
    s = json.load(open(os.path.join(d, 'e02_state.json')))
    for e in s.get('edge_finds', []):
        for q in e['touches']:
            if q['kind'] == 'board':
                rows.append(dict(stage=st, t=lt(e['t']), robot=e['robot'], kind='barrier felt (front stop at an open edge)',
                                 lines='', base='', dbot='', cam_cm=None, side='x=0', closest_4s=None,
                                 note='own pose (%.1f, %.1f); find not used, would have moved it %s' % (q['x'], q['y'], e.get('would_be'))))
    for w in s.get('watchdog_stops', []):
        t = w[0]; c, side = cam(w[1], t)
        rows.append(dict(stage=st, t=lt(t), robot=w[1], kind='camera stop: ' + w[2][:70], lines='', base='', dbot='',
                         cam_cm=c, side=side, closest_4s=closest(w[1], t - 0.5, t + 4), note=''))
rows.sort(key=lambda r: (r['stage'], r['t']))
with open(sys.argv[1], 'w', newline='') as f:
    wr = csv.DictWriter(f, fieldnames=list(rows[0])); wr.writeheader(); wr.writerows(rows)
for r in rows: print(r['stage'], r['t'][:8], r['robot'], r['kind'][:60], r['lines'], r['dbot'], r['cam_cm'], r['side'], r['closest_4s'], r['note'])
