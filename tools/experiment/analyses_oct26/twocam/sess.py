"""Two-camera geometry study (2026-10-26, docs/build-plans/two-camera-geometry.md). Scratch paths at analysis time:
scripts in <scratch>/tc/bin, downloads in <scratch>/tc/dl, results in <scratch>/tc/work (relative ../dl, ../work)."""
"""Session table, frame times (shared t_mono and wall), track.csv access."""
import os, csv, bisect, math, sys, glob, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
DL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'dl')
S = {
 '1510': ('e02-session-20261025-151004', '1510/20261025-150957_E02', '1510/e02-20261025-151004'),
 '1545': ('e02-session-20261025-154522', '1545/20261025-154514_E02', '1545/e02-20261025-154522'),
 '2114': ('e02-session-20261025-211419', '2114/20261025-211347_E02', '2114/e02-20261025-211419'),
 '0955': ('e02-session-20261026-095541', '0955/20261026-095534_E02', '0955/e02-20261026-095541'),
}
def lt(t):
    t = float(t); return time.strftime('%H:%M:%S', time.gmtime(t - 4 * 3600)) + '.%d' % int((t % 1) * 10)
_ft = {}
def ftimes(s, cam):
    """list of (frame, t_wall, t_mono, cam_ms) cleaned (NULs dropped)"""
    k = (s, cam)
    if k not in _ft:
        rows = []
        for line in open(os.path.join(DL, S[s][1], cam, 'frames.csv'), 'rb').read().split(b'\n')[1:]:
            p = line.replace(b'\x00', b'').strip().split(b',')
            try: rows.append((int(p[0]), float(p[1]), float(p[2]), float(p[3])))
            except (ValueError, IndexError): pass
        _ft[k] = rows
    return _ft[k]
def frame_at(s, cam, tw):
    """nearest frame to wall time tw -> (frame, t_wall)"""
    r = ftimes(s, cam); ts = [x[1] for x in r]
    i = bisect.bisect_left(ts, tw)
    c = [j for j in (i - 1, i) if 0 <= j < len(r)]
    j = min(c, key=lambda j: abs(ts[j] - tw)); return r[j][0], r[j][1]
_tr = {}
def track(s):
    if s not in _tr:
        out = {}
        for r in csv.DictReader(open(os.path.join(DL, S[s][2], 'track.csv'))):
            try: out.setdefault(r['robot'], []).append((float(r['t']), float(r['x_cm']), float(r['y_cm']), r['source'], r['phase'], float(r['lock_score'] or 'nan')))
            except ValueError: pass
        _tr[s] = out
    return _tr[s]
def pos_at(s, robot, t, maxdt=0.25):
    rr = track(s).get(robot, []); ts = [x[0] for x in rr]
    i = bisect.bisect_left(ts, t)
    c = [j for j in (i - 1, i) if 0 <= j < len(rr)]
    if not c: return None
    j = min(c, key=lambda j: abs(ts[j] - t))
    return rr[j] if abs(ts[j] - t) <= maxdt else None
# frame of track.csv positions: provisional (from-scratch explore, before the runner fell back) or roam1
SWITCH = {'1510': '15:26:34.6', '2114': '22:02:12.5', '0955': '10:22:10.5', '1545': None}
import calendar
def ep(s, hms):
    day = 26 if s == '0955' else 25
    h, m, sec = hms.split(':'); hh = int(h) + 4; d = day
    if hh >= 24: hh -= 24; d += 1
    return calendar.timegm((2026, 10, d, hh, int(m), 0)) + float(sec)
_prov = None
def to_roam1(s, t, x, y):
    """track (x, y) at time t -> roam1 floor cm on the tracker's plane (z = robot top)"""
    global _prov
    sw = SWITCH[s]
    if sw is None or t >= ep(s, sw):
        return x, y
    from common2 import OCAM, CAM
    if _prov is None:
        import e02
        _prov = e02.EX_provisional_camera(640, 480)
    u, v = _prov.project([x, y, CAM.ROBOT_TOP_CM])
    return OCAM.to_floor(u, v, CAM.ROBOT_TOP_CM)
