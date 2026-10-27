"""stretches.py [session ...] : dataset A, what the sensor-trust check sees. Every straight stretch the camera saw a
robot drive (e02_fit.camera_stretches, as the runner's learn() finds them) inside a robot run that streamed raw wheel
counts (find, route, calib), paired with the wheels' own count over the same time (e02_fit.pair). Wheel error = camera
length - stored wheel scale x counted length (e02.stretch_rms_against's item). Writes OUT/stretches.csv."""
import sys, os, csv, json
import numpy as np
from common import SESSIONS, sdir, rows, num, track, battery, at, OUT, FIT

PRIOR_W = FIT.PRIOR['wheel_scale']


def stored_scale(s):
    """{robot: wheel scale}: the session's last pooled fit (what stretch_rms_against used), else the prior."""
    st = json.load(open(os.path.join(sdir(s), 'e02_state.json')))
    out = {}
    for r, v in (st.get('fits') or {}).items():
        for h in v.get('history', []):
            w = (h.get('fitted') or {}).get('wheel_scale') or (h.get('values') or {}).get('wheel_scale')
            if w:
                out[r] = float(w)
    return out


def run(s):
    d = sdir(s)
    rl = os.path.join(d, 'robot-logs-1')
    info = {(r['robot'], r['run']): r for r in rows(os.path.join(rl, 'runinfo.csv'))}
    raw = {}
    with open(os.path.join(rl, 'raw.csv')) as f:
        for r in csv.DictReader(f):
            q = dict(t_recv=num(r['t_recv']), ms=num(r['ms']), x_cm=num(r['x_cm']), y_cm=num(r['y_cm']),
                     yaw_deg=num(r['yaw_deg']), th_deg=num(r['th_deg']), v=num(r['v_cms']), batt=num(r['battery']))
            raw.setdefault((r['robot'], r['run']), []).append(q)
    cams = track(s)
    bat = battery(s)
    ws = stored_scale(s)
    out = []
    for (robot, runid), rr in raw.items():
        exp = (info.get((robot, runid)) or {}).get('experiment', '?')
        if exp == 'gyro':
            continue
        ys = num((info.get((robot, runid)) or {}).get('yaw_sign')) or 1
        ser = FIT.raw_series(rr, yaw_sign=1 if ys >= 0 else -1)
        if ser is None or robot not in cams:
            continue
        t, x, y = cams[robot]
        k = (t >= ser['t'][0] - 0.1) & (t <= ser['t'][-1] + 0.1)
        cam = list(zip(t[k], x[k], y[k]))
        for st_ in FIT.camera_stretches(cam):
            p = FIT.pair([st_], ser, run=runid)
            if not p:
                continue
            p = p[0]
            dur = st_['t1'] - st_['t0']
            w = ws.get(robot, PRIOR_W)
            out.append(dict(session=s, robot=robot, run=runid, experiment=exp,
                            kind=('route' if exp == 'e02' else exp) + (' (backwards)' if not p['forward'] else ''),
                            t0=round(st_['t0'], 3), t1=round(st_['t1'], 3), dur_s=round(dur, 3),
                            cam_cm=round(p['cam_len'], 3), wheel_raw_cm=round(p['bel_len'], 3), w=w,
                            wheel_cm=round(w * p['bel_len'], 3), err_cm=round(p['cam_len'] - w * p['bel_len'], 3),
                            cam_speed=round(p['cam_len'] / dur, 3), wheel_speed=round(w * p['bel_len'] / dur, 3),
                            battery=at(bat.get(robot), 0.5 * (st_['t0'] + st_['t1']))))
    return out


if __name__ == '__main__':
    ss = sys.argv[1:] or SESSIONS
    allr = []
    for s in ss:
        r = run(s)
        print(s, len(r), 'stretches', {k: sum(1 for q in r if q['robot'] == k) for k in ('Left', 'Center', 'Right')})
        allr += r
    name = os.environ.get('WS_NAME', 'stretches.csv')
    with open(os.path.join(OUT, name), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(allr[0].keys()))
        w.writeheader()
        w.writerows(allr)
    print('written', os.path.join(OUT, name), len(allr))
