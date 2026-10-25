import sys, json, pickle, math
sys.path[:0] = ['/home/claude/alvik-fleet-capstone/tools/experiment', '/home/claude/alvik-fleet-capstone/tools/recognition']
import numpy as np, cameras as CMS, e02_explore as EX
d = json.load(open('/tmp/claude-0/s1025/logs/e02-20261025-151004/e02_state.json'))
obs = d['setup']['explore_obs']; S = pickle.load(open('refit.pkl', 'rb'))['sol']
roam = CMS.CameraModel.load('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json')
A, B, own, R = [], [], [], []
for n, st in obs.items():
    st2 = EX.implausible(st)
    p_ex = EX.to_world(EX.rebuild(st, S['s'][n], S['g'][n]), S['frames'][n])
    p_own = EX.rebuild(st, 1.0, 1.0)
    for q, pe in zip(st2, p_ex):
        if q.get('skip'): continue
        gx, gy = roam.to_floor(q['u'], q['v'], 3.7)
        A.append(pe[:2]); B.append((gx, gy)); R.append(n)
A, B = np.array(A), np.array(B)
def sim(P, W, scale=True):
    mp, mw = P.mean(0), W.mean(0); U, Sg, Vt = np.linalg.svd((W - mw).T @ (P - mp))
    D = np.eye(2); D[1, 1] = np.sign(np.linalg.det(U @ Vt)); Rm = U @ D @ Vt
    s = np.trace(np.diag(Sg) @ D) / ((P - mp) ** 2).sum() if scale else 1.0
    return s, Rm, mw - s * Rm @ mp
for sc in (False, True):
    s, Rm, t = sim(A, B, sc)
    e = np.linalg.norm((s * (Rm @ A.T)).T + t - B, axis=1)
    print('explore floor vs roam1 floor, %s: scale %.3f, residual median %.1f cm, 90%% %.1f, max %.1f (n=%d)' % ('with scale' if sc else 'rigid', s, np.median(e), np.percentile(e, 90), e.max(), len(e)))
# explore's pixel scale at the middle
for nm, cam in (('explore', None),):
    p0 = EX.project([(A[:,0].mean(), A[:,1].mean(), 3.7)], S['f'], S['k1'], np.array(S['rvec']), np.array(S['tvec']), (640, 480))[0]
    p1 = EX.project([(A[:,0].mean() + 10, A[:,1].mean(), 3.7)], S['f'], S['k1'], np.array(S['rvec']), np.array(S['tvec']), (640, 480))[0]
    print('explore px per cm at the middle: %.2f' % (np.linalg.norm(p1 - p0) / 10))
c = np.array(roam.project((61, 61, 3.7))).ravel(); c2 = np.array(roam.project((71, 61, 3.7))).ravel()
print('roam1 px per cm at the middle: %.2f' % (np.linalg.norm(c2 - c) / 10))
R_ = cv2 = None
import cv2
Rc = cv2.Rodrigues(np.array(S['rvec']))[0]; cpos = -Rc.T @ np.array(S['tvec'])
print('explore camera height above floor %.0f cm, f %.0f px' % (cpos[2], S['f']))
