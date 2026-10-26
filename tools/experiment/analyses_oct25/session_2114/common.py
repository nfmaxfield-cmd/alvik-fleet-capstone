"""Paths and helpers for the 2026-10-25 21:14 session analysis (scratch paths at analysis time; change D and R)."""
import csv, time, os, calendar
SCR = '/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad'
D = SCR + '/dl/logs/e02-20261025-211419'
R = SCR + '/dl/rec/20261025-211347_E02'
W = SCR + '/work'


def lt(t):
    """Unix seconds -> local EDT 'HH:MM:SS.d'."""
    t = float(t)
    return time.strftime('%H:%M:%S', time.gmtime(t - 4 * 3600)) + '.%d' % int((t % 1) * 10)


def ep(s, day=25):
    """'HH:MM:SS(.s)' local EDT on 2026-10-25 -> Unix seconds."""
    h, m, sec = s.split(':')
    return calendar.timegm((2026, 10, day, int(h) + 4, int(m), 0)) + float(sec)


def rows(f, base=None):
    return list(csv.DictReader(open(os.path.join(base or D, f))))
