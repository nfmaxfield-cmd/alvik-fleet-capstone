"""Paths and helpers for the 2026-10-26 09:55 session analysis (scratch paths at analysis time; change D and R)."""
import csv, time, os, calendar
SCR = '/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad'
D = SCR + '/dl/e02-20261026-095541'
R = SCR + '/dl/20261026-095534_E02'
W = SCR + '/work'
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))


def lt(t):
    """Unix seconds -> local EDT 'HH:MM:SS.d'."""
    t = float(t)
    return time.strftime('%H:%M:%S', time.gmtime(t - 4 * 3600)) + '.%d' % int((t % 1) * 10)


def ep(s, day=26):
    """'HH:MM:SS(.s)' local EDT on 2026-10-26 -> Unix seconds."""
    h, m, sec = s.split(':')
    return calendar.timegm((2026, 10, day, int(h) + 4, int(m), 0)) + float(sec)


def rows(f, base=None):
    return list(csv.DictReader(open(os.path.join(base or D, f))))


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None
