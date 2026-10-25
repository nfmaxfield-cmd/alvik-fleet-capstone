import sys, json, math, pickle
sys.path[:0] = ['/home/claude/alvik-fleet-capstone/tools/experiment', '/home/claude/alvik-fleet-capstone/tools/recognition']
import numpy as np
import e02_explore as EX, e02_map as MAP
d = json.load(open('/tmp/claude-0/s1025/logs/e02-20261025-151004/e02_state.json'))
obs = d['setup']['explore_obs']
size = (640, 480)
robots = {n: v for n, v in obs.items() if len(v) >= 5}
checks = {n: EX.robot_check(v) for n, v in robots.items()}
print('checks', checks)
sol = EX.fit(robots, size)
print('fit rms %.2f f %.1f k1 %.3f per %s s %s g %s init %s' % (sol['rms_px'], sol['f'], sol['k1'], sol['per_robot_rms_px'], sol['s'], sol['g'], sol.get('init')))
print('fit_check', repr(EX.fit_check(sol)))
pts = EX.contacts(robots, sol)
for p in pts: print('contact', [round(v, 1) if isinstance(v, float) else v for v in p])
rect = EX.rectangle_from_rays(pts)
print('rect', rect and {k: (round(v[0], 1), v[1]['kind'], v[1]['kinds'], v[1]['n']) for k, v in rect['lines'].items()}, rect and rect['angle_deg'])
if rect:
    r2, ah = EX.refine_edges(rect, MAP.BOTTOM_AHEAD_CM)
    print('refine ahead', ah)
    print('rect2', {k: (round(v[0], 1), v[1]['kind'], v[1]['kinds']) for k, v in r2['lines'].items()})
    print('rect_check', EX.rect_check(r2))
print('size', EX.size_scale(robots, sol, size))
sol.pop('resid', None)
pickle.dump(dict(sol=sol, pts=pts, rect=rect), open('/tmp/claude-0/s1025/work/refit.pkl', 'wb'))
