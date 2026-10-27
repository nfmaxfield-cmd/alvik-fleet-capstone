"""Paths and helpers for the 2026-10-27 edges session analysis (E1 00:11, E2 00:29). Scratch paths at analysis time:
set EDGES_DIR to the folder holding e1/, e2/, rec/ (the unzipped logs and the recorder's small zip)."""
import csv, time, os, calendar, glob
SCR = os.environ.get('EDGES_DIR', '/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/edges')
STAGE = {'E1': glob.glob(SCR + '/e1/e02-*')[0] if glob.glob(SCR + '/e1/e02-*') else None,
         'E2': glob.glob(SCR + '/e2/e02-*')[0] if glob.glob(SCR + '/e2/e02-*') else None}
REC = SCR + '/rec/20261027-001129_E02'
TAG = 'e02-pi-plan-20261027-0011-recording'
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
XMAX, YMAX = 122.15, 121.9      # the roam1 board's open-edge lines (e02_state.json edge finds)


def lt(t):
    t = float(t)
    return time.strftime('%H:%M:%S', time.gmtime(t - 4 * 3600)) + '.%d' % int((t % 1) * 10)


def ep(s):
    """'HH:MM:SS(.s)' local EDT on 2026-10-27 (00:xx) -> Unix seconds."""
    h, m, sec = s.split(':')
    return calendar.timegm((2026, 10, 27, int(h) + 4, int(m), 0)) + float(sec)


def rows(stage, f):
    return list(csv.DictReader(open(os.path.join(STAGE[stage], f))))


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def edge_d(x, y):
    """signed distance (cm) of a point to the nearest open-edge line (negative = past it), and which line"""
    c = [(x, 'x=0'), (XMAX - x, 'x=122'), (y, 'y=0'), (YMAX - y, 'y=122')]
    return min(c)


def track(stage):
    out = {}
    for r in rows(stage, 'track.csv'):
        x, y = num(r['x_cm']), num(r['y_cm'])
        if x is None:
            continue
        out.setdefault(r['robot'], []).append((float(r['t']), x, y, r['source'], r['phase']))
    return out
