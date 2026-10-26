"""trust_2114.py : Center's "wheels look bad x2.9" on the live page at 22:53, replayed with the sensor-trust code on main
(e02_trust.py, unchanged since edbf86e, which this session already ran). The real sessions' evidence is chained in order
(2026-10-23 morning, evening, 2026-10-24 stages 1-2, 2026-10-25 15:10 and 15:45) as trust_fix_real_replay.py does, then a
new session (21:14) with no stretch or run evidence (explore failed; no runs). Printed: each robot's wheels as the
runner judges them (judge) and as the live page showed them (e02_view._live_trust: with no live check this session,
k = all sessions' stretch rms / the healthy prior 0.6). Then the same with the 15:45 session's live flag carried over
(raised at 17:02:44 by the code of that time, b6de33c: nothing in this session can clear it)."""
import os, sys, copy
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
os.environ.setdefault('TRUST_EXTRA_SESSIONS', ','.join(
    '%s=%s' % (k, os.path.join(os.environ.get('S2114_PREV', ''), v)) for k, v in
    (('s1510', 'e02-20261025-151004'), ('s1545', 'e02-20261025-154522'))))
import trust_fix_real_replay as RR  # noqa: E402
TR = RR.TR
chain = None
for name in RR.SESS:
    chain, flags, _, robots = RR.replay(name, chain)
    print(name, 'flags raised:', [(s, r, round(k, 2)) for s, r, w, k, _ in flags if w == 'degraded'] or 'none')
robots = ['Left', 'Center', 'Right']


def show(tr, label):
    print(label)
    for r in robots:
        j = tr.judge('wheels', r, robots)
        sd, n, how = tr.own_sd('wheels', r)
        k_view = j['k'] if j['worst'] is not None else sd / TR.HEALTHY['wheels']
        print('  %-6s judge k %.2f (checks: %s) | live page: sd %.2f cm over %d fits (%s) -> k %.2f | flagged %s' % (
            r, j['k'], [c['check'] for c in j['checks']] or 'none this session', sd, n, how, k_view,
            tr.bad('wheels', r)))


s = copy.deepcopy(chain)
s.new_session()
show(s, 'A. 21:14 with main\'s code all the way (no flag carried):')
s2 = copy.deepcopy(chain)
s2.flags['wheels/Center'] = True          # the 15:45 live flag (k 2.3 at 17:02:44)
s2.flag_session['wheels/Center'] = s2.session
s2.new_session()
s2.update_flags(robots)
show(s2, 'B. 21:14 with the 15:45 live flag carried over:')
