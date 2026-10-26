"""fit_replay.py : explore's camera fit replayed offline from the session's own stops (e02_state.json setup.explore_obs),
as e02.explore_finish does it (robot checks, joint fit, the per-robot drop, fit_check, contacts, sides). Variants: as run;
Left's stops cut where it hung at the corner (after 21:42:46, ray 6's edge stop); and each robot alone."""
import json, sys, os, time
import pose  # noqa: F401  (paths)
import e02_explore as EX
import e02_plan as PL
import e02_map as MAP
from common import D, ep, lt
st = json.load(open(D + '/e02_state.json'))
obs = st['setup']['explore_obs']
size = (640, 480)


def run(name, robots):
    robots = {n: v for n, v in robots.items() if len(v) >= 5}
    checks = {n: EX.robot_check(v) for n, v in robots.items()}
    left = [n for n in robots if not checks[n]['ok']]
    use = {n: v for n, v in robots.items() if checks[n]['ok']}
    out = dict(variant=name, checks={n: (round(c['rms_px'], 1), c['ok']) for n, c in checks.items()})
    if not use:
        out['result'] = 'no robot passes its own check'
        return out
    sol = EX.fit(use, size)
    worse = [n for n, v in sol['per_robot_rms_px'].items() if not (v <= EX.FIT_ROBOT_MAX_PX)]
    if worse and len(worse) < len(use):
        left += worse
        use = {n: v for n, v in use.items() if n not in worse}
        sol = EX.fit(use, size)
    out.update(rms_px=round(sol['rms_px'], 2), per_robot={n: round(v, 1) for n, v in sol['per_robot_rms_px'].items()},
               s={n: round(v, 3) for n, v in sol['s'].items()}, g={n: round(v, 3) for n, v in sol['g'].items()},
               left_out=left, fit_check=EX.fit_check(sol) or 'pass')
    pts = EX.contacts(use, sol, others={n: robots[n] for n in left if n in robots})
    out['contacts'] = len(pts)
    rect = EX.rectangle_from_rays(pts)
    if rect:
        rect, ahead = EX.refine_edges(rect, MAP.BOTTOM_AHEAD_CM)
        out['rect_check'] = EX.rect_check(rect) or 'pass'
        bf = EX.board_frame(rect) if not EX.rect_check(rect) else None
        if bf:
            out['board'] = '%.1f x %.1f cm, kinds %s, spreads %s' % (bf['W'], bf['H'], bf['kinds'], {
                k: round(v, 1) for k, v in bf['spreads'].items()})
    else:
        out['rect_check'] = 'no rectangle from the rays'
    return out


cut = ep('21:42:46')
variants = [('as run', dict(obs)),
            ('Left cut at 21:42:46 (before it hung)', dict(obs, Left=[s for s in obs['Left'] if s['t'] < cut])),
            ('Center alone', {'Center': obs['Center']}),
            ('Right alone', {'Right': obs['Right']}),
            ('Center + Left cut', {'Center': obs['Center'], 'Left': [s for s in obs['Left'] if s['t'] < cut]})]
for name, rb in variants:
    t0 = time.time()
    r = run(name, rb)
    print(json.dumps(r), '(%.0f s)' % (time.time() - t0))
    sys.stdout.flush()
