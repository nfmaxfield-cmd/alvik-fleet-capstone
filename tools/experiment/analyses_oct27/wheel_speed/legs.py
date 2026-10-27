"""legs.py [session ...] : dataset B, every `go` leg a robot drove (robot-logs-1/leg.csv), including the ones no
robot run streams raw counts for: explore rays, creep steps, back-outs, identify wiggles, square checks, returns and
route legs. Wheel distance = the robot's own pose (wheel counts x its stored wheel scale, firmware pose.h) from the
leg's start record to its done/failed record. Camera distance = the median camera position while standing still
after the leg against before it. Commanded speed = the last `set e02_cms` sent to that robot before the leg's command
(e02_log.txt). Legs with a camera fix sent during them (fix.csv) are left out (the pose jumps). Writes OUT/legs.csv."""
import sys, os, csv, re
import numpy as np
from common import SESSIONS, sdir, rows, num, track, battery, at, OUT, session_day_epoch

PRE_S, POST_LAG_S, POST_S = 1.0, 0.4, 1.0     # chosen windows (s): standing before / lag after done / standing after
STILL_SD_CM = 1.2                             # chosen: a window counts as standing still under this scatter (a turn on the spot swings the centre ~1.5 cm)


def kind_of(value, runid):
    if runid != '-':
        return 'route'
    v = round(value, 2)
    if v == 105.0 or v == 104.83:
        return 'ray (crawl to edge/board)'
    if v == 3.0:
        return 'creep step'
    if v == -9.5:
        return 'back-out'
    if v == 12.0 or -12.0 < v <= -11.7:
        return 'identify wiggle'
    if v == 15.0:
        return 'square check'
    return 'return / reposition' if v > 0 else 'reverse'


def commands(s):
    """{(robot, id): (cms, wheel_scale, route wheel_scale) in effect when the leg was sent} from e02_log.txt (None:
    not sent this session)."""
    cms, ws, rws, out = {}, {}, {}, {}
    for l in open(os.path.join(sdir(s), 'e02_log.txt')):
        m = re.search(r'-> (\w+): set (e02_cms|wheel_scale|e02_route_wheel_scale) (-?[\d.]+)', l)
        if m:
            {'e02_cms': cms, 'wheel_scale': ws, 'e02_route_wheel_scale': rws}[m.group(2)][m.group(1)] = float(m.group(3))
            continue
        m = re.search(r'-> (\w+): go (-?[\d.]+) (\d+)', l)
        if m and (m.group(1), m.group(3)) not in out:
            r = m.group(1)
            out[(r, m.group(3))] = (cms.get(r), ws.get(r), rws.get(r))
    return out


def param_scale(s):
    """{robot: (t array, wheel_scale array)} from the robots' parameter dumps (param.csv)."""
    d = {}
    for r in rows(os.path.join(sdir(s), 'robot-logs-1', 'param.csv')):
        if r['name'] == 'wheel_scale':
            d.setdefault(r['robot'], []).append((float(r['t_recv']), float(r['value'])))
    return {k: (np.array(sorted(v))[:, 0], np.array(sorted(v))[:, 1]) for k, v in d.items()}


def pos_at(t, x, y, tq, both_s=0.4, near_s=0.3):
    i = int(np.searchsorted(t, tq))
    if 0 < i < len(t) and tq - t[i - 1] <= both_s and t[i] - tq <= both_s:
        f = (tq - t[i - 1]) / max(1e-6, t[i] - t[i - 1])
        return x[i - 1] + f * (x[i] - x[i - 1]), y[i - 1] + f * (y[i] - y[i - 1])
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(t) and abs(t[j] - tq) <= near_s and (best is None or abs(t[j] - tq) < abs(t[best] - tq)):
            best = j
    return (x[best], y[best]) if best is not None else None


def run(s):
    d = sdir(s)
    rl = os.path.join(d, 'robot-logs-1')
    L = rows(os.path.join(rl, 'leg.csv'))
    fixes = {}
    if os.path.exists(os.path.join(rl, 'fix.csv')):
        for r in rows(os.path.join(rl, 'fix.csv')):
            fixes.setdefault(r['robot'], []).append(float(r['t_recv']))
    cmd = commands(s)
    psc = param_scale(s)
    from stretches import stored_scale, PRIOR_W
    stored = stored_scale(s)
    cams = track(s)
    bat = battery(s)
    by = {}
    for r in L:
        by.setdefault(r['robot'], []).append(r)
    out = []
    for robot, lst in by.items():
        lst.sort(key=lambda r: float(r['t_recv']))
        legs = []                                   # (id, kind, value, run, start row, end row)
        start = {}
        for r in lst:
            if r['result'] == 'started':
                start[r['id']] = r
            elif r['result'] in ('done', 'failed') and r['id'] in start:
                legs.append((start.pop(r['id']), r))
        def tt(r):
            return float(r['t_recv'])
        # laptop - robot clock, per leg: lowest (t_recv - ms) of its records (least radio delay)
        times = []
        for a, b in legs:
            off = min(float(a['t_recv']) - float(a['ms']) / 1000, float(b['t_recv']) - float(b['ms']) / 1000)
            times.append((float(a['ms']) / 1000 + off, float(b['ms']) / 1000 + off))
        if robot not in cams:
            continue
        t, x, y = cams[robot]
        for i, ((a, b), (t0, t1)) in enumerate(zip(legs, times)):
            if a['kind'] != 'go':
                continue
            # a turn on the spot before or after counts as standing (its centre moves about 1.5 cm); a drive does not
            prev_end = max([tb for (aa, _), (_, tb) in zip(legs[:i], times[:i]) if aa['kind'] == 'go'] or [-1e18])
            next_start = min([ta for (aa, _), (ta, _) in zip(legs[i + 1:], times[i + 1:]) if aa['kind'] == 'go']
                             or [1e18])
            w0 = max(t0 - PRE_S, prev_end + POST_LAG_S)
            w1 = t1 + POST_LAG_S
            k0 = (t >= w0) & (t <= t0 + 0.1)
            k1 = (t >= w1) & (t <= min(w1 + POST_S, next_start + 0.1))
            cms, ws_, rws_ = cmd.get((robot, a['id'])) or (None, None, None)
            if a['run'] != '-' and rws_ is not None and rws_ > 0:
                scale_now = rws_                    # a route leg runs on the route's own wheel scale
            else:
                scale_now = ws_ if ws_ is not None else (at(psc.get(robot), t0) or 1.0)
            pose_cm = float(np.hypot(float(b['x_cm']) - float(a['x_cm']), float(b['y_cm']) - float(a['y_cm'])))
            w = stored.get(robot, PRIOR_W)
            rec = dict(session=s, robot=robot, id=a['id'], run=a['run'], value=float(a['value']),
                       kind=kind_of(float(a['value']), a['run']), result=b['result'], info=b['info'],
                       t0=round(t0, 3), t1=round(t1, 3), dur_s=round(t1 - t0, 3),
                       pose_cm=round(pose_cm, 3), scale_in_robot=scale_now, w=w,
                       wheel_cm=round(w * pose_cm / scale_now, 3),
                       cms=cms, battery=at(bat.get(robot), t0),
                       fix=any(t0 - 0.2 <= f <= t1 + 0.2 for f in fixes.get(robot, [])),
                       n0=int(k0.sum()), n1=int(k1.sum()))
            ok = rec['n0'] >= 3 and rec['n1'] >= 3
            if ok:
                p0 = np.array([np.median(x[k0]), np.median(y[k0])])
                p1 = np.array([np.median(x[k1]), np.median(y[k1])])
                sd0 = float(np.sqrt(np.var(x[k0]) + np.var(y[k0])))
                sd1 = float(np.sqrt(np.var(x[k1]) + np.var(y[k1])))
                cam = float(np.hypot(*(p1 - p0)))
                rec.update(cam_cm=round(cam, 3), sd0=round(sd0, 2), sd1=round(sd1, 2),
                           still=sd0 < STILL_SD_CM and sd1 < STILL_SD_CM,
                           err_cm=round(cam - rec['wheel_cm'], 3),
                           cam_speed=round(cam / rec['dur_s'], 3) if rec['dur_s'] > 0 else None,
                           wheel_speed=round(rec['wheel_cm'] / rec['dur_s'], 3) if rec['dur_s'] > 0 else None)
            else:
                rec.update(cam_cm=None, sd0=None, sd1=None, still=False, err_cm=None, cam_speed=None, wheel_speed=None)
            # fallback for slow legs (crawl 4, creep 2 cm/s) whose standing windows have too few pictures (explore: the
            # tracker stops following a robot about 0.1 s after its leg): the camera position AT the start and end
            # times, interpolated between pictures under 0.4 s either side, else the nearest picture within 0.3 s
            # (at 4 cm/s that is at most 1.2 cm off).
            rec['cam_at_cm'] = None
            if cms is not None and cms <= 4.0:
                q0, q1 = pos_at(t, x, y, t0), pos_at(t, x, y, t1)
                if q0 is not None and q1 is not None:
                    rec['cam_at_cm'] = round(float(np.hypot(q1[0] - q0[0], q1[1] - q0[1])), 3)
            out.append(rec)
    return out


if __name__ == '__main__':
    ss = sys.argv[1:] or SESSIONS
    allr = []
    for s in ss:
        r = run(s)
        good = [q for q in r if q['still'] and not q['fix']]
        print(s, len(r), 'go legs;', len(good), 'usable (still before and after, no fix)')
        allr += r
    if not allr:
        sys.exit('nothing found: no file written')
    name = os.environ.get('WS_NAME', 'legs.csv')
    keys = list(allr[0].keys())
    with open(os.path.join(OUT, name), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(allr)
    print('written', os.path.join(OUT, name), len(allr))
