"""fit_replay.py : explore's camera fit replayed offline from the session's own stops (e02_state.json
setup.explore_obs), as e02.explore_finish does it (robot checks, joint fit, the per-robot drop, fit_check, contacts,
sides); then explore's camera against the stored roam1 calibration: the board's pixels mapped to the floor by both,
aligned by a rotation + shift (+ scale reported), and the leftover difference in cm."""
import json, sys, os, time
import numpy as np
import pose  # noqa: F401  (paths)
import e02_explore as EX
import e02_map as MAP
from common import D
st = json.load(open(D + '/e02_state.json'))
obs = st['setup']['explore_obs']
size = (640, 480)


def run(name, robots):
    robots = {n: v for n, v in robots.items() if len(v) >= 5}
    checks = {n: EX.robot_check(v) for n, v in robots.items()}
    left = [n for n in robots if not checks[n]['ok']]
    use = {n: v for n, v in robots.items() if checks[n]['ok']}
    out = dict(variant=name, checks={n: (round(c['rms_px'], 1), c['ok'], len(robots[n])) for n, c in checks.items()})
    if not use:
        out['result'] = 'no robot passes its own check'
        return out, None
    sol = EX.fit(use, size)
    worse = [n for n, v in sol['per_robot_rms_px'].items() if not (v <= EX.FIT_ROBOT_MAX_PX)]
    if worse and len(worse) < len(use):
        left += worse
        use = {n: v for n, v in use.items() if n not in worse}
        sol = EX.fit(use, size)
    out.update(rms_px=round(sol['rms_px'], 2), n=sol.get('n'), f=round(sol['f'], 1), k1=round(sol['k1'], 4),
               per_robot={n: round(v, 1) for n, v in sol['per_robot_rms_px'].items()}, left_out=left,
               fit_check=EX.fit_check(sol) or 'pass')
    pts = EX.contacts(use, sol, others={n: robots[n] for n in left if n in robots})
    out['contacts'] = len(pts)
    out['contact_kinds'] = [p[2] if len(p) > 2 else None for p in pts]
    rect = EX.rectangle_from_rays(pts)
    out['rect_check'] = (EX.rect_check(rect) or 'pass') if rect else 'no rectangle from the rays'
    return out, sol


def compare(sol):
    """explore's floor (its own robot frame) against roam1's floor for a grid of pixels inside the board."""
    corners = [pose.board_px(x, y, 0) for x, y in ((0, 0), (122.15, 0), (122.15, 121.9), (0, 121.9))]
    import cv2
    poly = np.array(corners, np.float32)
    A, B = [], []
    for u in range(0, 640, 10):
        for v in range(0, 480, 10):
            if cv2.pointPolygonTest(poly, (u, v), False) > 0:
                A.append(EX.world_of(sol, u, v, 0.0))
                B.append(pose.ROAM.to_floor(float(u), float(v), 0.0))
    A, B = np.array(A, float), np.array([[float(p[0]), float(p[1])] for p in B])
    ma, mb = A.mean(0), B.mean(0)
    H = (A - ma).T @ (B - mb)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:       # a mirror: explore's y may run the other way
        Vt[-1] *= -1
        R = Vt.T @ U.T
    s = S.sum() / ((A - ma) ** 2).sum()
    for scale, lab in ((1.0, 'rotation + shift'), (s, 'rotation + shift + scale %.3f' % s)):
        P = scale * (A - ma) @ R.T + mb
        d = np.hypot(*(P - B).T)
        print('  %s: %d points, difference median %.1f cm, 95%% %.1f cm, max %.1f cm' % (
            lab, len(d), np.median(d), np.percentile(d, 95), d.max()))
    # board corners as explore puts them, after the best rigid alignment, against roam1's
    for (u, v), name in zip(corners, ('bottom-right', 'top-right (x 122)', 'top-left', 'bottom-left')):
        a = np.array(EX.world_of(sol, u, v, 0.0))
        p = (a - ma) @ R.T + mb
        print('  corner %s: roam1 at its own, explore %.1f cm off' % (name, np.hypot(*(p - pose.ROAM.to_floor(u, v, 0.0)))))


variants = [('as run', dict(obs)), ('Center alone', {'Center': obs['Center']})]
for name, rb in variants:
    t0 = time.time()
    r, sol = run(name, rb)
    print(json.dumps(r), '(%.0f s)' % (time.time() - t0))
    if sol is not None and name == 'as run':
        compare(sol)
    sys.stdout.flush()
