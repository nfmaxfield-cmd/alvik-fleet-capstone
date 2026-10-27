"""Paths and loaders for the wheel-error-against-speed analysis (2026-10-27). The sessions' log zips are fetched from
the repo's GitHub releases into DL (scratch; change it or set WS_DL) and unzipped to DL/x/logs-<session>/<session>/."""
import csv, os, sys, re, glob, math, bisect, json, calendar, time
HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(os.path.dirname(HERE))
REPO = os.path.dirname(os.path.dirname(EXP))
sys.path.insert(0, EXP)
sys.path.insert(0, os.path.join(os.path.dirname(EXP), 'recognition'))
import numpy as np
import e02_camera as CAM
import e02_explore as EX
import e02_fit as FIT

DL = os.environ.get('WS_DL', '/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/dl')
OUT = os.environ.get('WS_OUT', os.path.join(DL, 'ws_out'))
os.makedirs(OUT, exist_ok=True)
# every real session with robot logs since 2026-10-23 (the 23rd's three releases hold video only: no robot logs)
SESSIONS = ['e02-20261024-153612', 'e02-20261024-161612', 'e02-20261024-173242', 'e02-20261024-193526',
            'e02-20261025-151004', 'e02-20261025-154522', 'e02-20261025-211419', 'e02-20261026-095541']
TONIGHT = 'e02-20261027-001136'
ROAM = CAM.load_camera(os.path.join(EXP, 'data', 'overhead_floorcal_roam1.json'))


def sdir(s):
    g = glob.glob(os.path.join(DL, 'x', 'logs-' + s, '*'))
    return g[0] if g else None


def short(s):
    """'e02-20261025-154522' -> 'Oct 25 15:45'."""
    m = re.match(r'e02-2026(\d\d)(\d\d)-(\d\d)(\d\d)', s)
    return 'Oct %s %s:%s' % (m.group(2), m.group(3), m.group(4))


def rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def num(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def local_epoch(day, hms):
    """'HH:MM:SS' EDT on 2026-10-<day> -> Unix s."""
    h, m, s = (int(v) for v in hms.split(':'))
    return calendar.timegm((2026, 10, day, h + 4, m, s))


def session_day_epoch(s, hms):
    m = re.match(r'e02-2026(\d\d)(\d\d)-(\d\d)', s)
    day = int(m.group(2))
    t = local_epoch(day, hms)
    if int(hms[:2]) < int(m.group(3)) - 1:        # past midnight
        t += 86400
    return t


def explore_end(s):
    """Unix s until which the session's camera positions are on the provisional camera (explore from scratch, until
    "EXPLORE FAILED" or the end of the log), or None when the session started on the stored calibration."""
    L = open(os.path.join(sdir(s), 'e02_log.txt')).read().splitlines()
    if not any('From scratch' in l for l in L[:40]):
        return None
    for l in L:
        if 'EXPLORE FAILED' in l or 'Explore done' in l or 'explore found' in l.lower():
            return session_day_epoch(s, l[1:9])
    return float('inf')


_prov = None


def prov():
    global _prov
    if _prov is None:
        import cameras as CMS
        f = 600.0
        _prov = CMS.CameraModel([[f, 0, 320.0], [0, f, 240.0], [0, 0, 1]], [0, 0, 0, 0, 0], [math.pi, 0.0, 0.0],
                                [0.0, 0.0, 260.0], (640, 480), name="provisional")
    return _prov


def track(s):
    """{robot: (t array, x array, y array)} on the stored roam1 calibration (board cm), 'held' rows (the tracker
    not seeing the robot) left out. Explore-time positions (provisional camera) go back to pixels and onto roam1 at
    the robot's top height, as the runner's border did."""
    te = explore_end(s)
    out = {}
    with open(os.path.join(sdir(s), 'track.csv')) as f:
        for r in csv.DictReader(f):
            if r['source'] == 'held':
                continue
            x, y = num(r['x_cm']), num(r['y_cm'])
            if x is None or y is None:
                continue
            t = float(r['t'])
            if te is not None and t < te:
                u, v = prov().project((x, y, EX.ROBOT_TOP_CM))
                gx, gy = ROAM.to_floor(float(u), float(v), EX.ROBOT_TOP_CM)
                x, y = float(gx), float(gy)
            out.setdefault(r['robot'], []).append((t, x, y))
    res = {}
    for k, v in out.items():
        v.sort()
        a = np.array(v)
        res[k] = (a[:, 0], a[:, 1], a[:, 2])
    return res


def battery(s):
    """{robot: (t array, % array)} from battery.csv and the robots' raw rows."""
    d = {}
    p = os.path.join(sdir(s), 'battery.csv')
    if os.path.exists(p):
        for r in rows(p):
            b = num(r['battery'])
            if b is not None:
                d.setdefault(r['robot'], []).append((float(r['t']), b))
    with open(os.path.join(sdir(s), 'robot-logs-1', 'raw.csv')) as f:
        for i, r in enumerate(csv.DictReader(f)):
            if i % 50:
                continue
            b = num(r['battery'])
            if b is not None and b > 0:
                d.setdefault(r['robot'], []).append((float(r['t_recv']), b))
    return {k: (np.array(sorted(v))[:, 0], np.array(sorted(v))[:, 1]) for k, v in d.items()}


def at(series, t):
    """Nearest value of (t array, v array) at t, or None."""
    if series is None or not len(series[0]):
        return None
    i = int(np.clip(np.searchsorted(series[0], t), 1, len(series[0]) - 1))
    j = i if abs(series[0][i] - t) < abs(series[0][i - 1] - t) else i - 1
    return float(series[1][j])
