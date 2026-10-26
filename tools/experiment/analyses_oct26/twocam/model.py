"""Side camera model fitted from robots: a floor homography plus one radial lens-bend term about the picture centre."""
import numpy as np, cv2
from scipy.optimize import least_squares
C = np.array([320.0, 240.0])
class Side:
    def __init__(self, H, kap=0.0):
        self.H = np.asarray(H, float) / H[2, 2]; self.kap = float(kap)
    def p(self): return np.r_[self.H.ravel()[:8], self.kap * 1e6]
    @classmethod
    def from_p(cls, p): return cls(np.r_[p[:8], 1.0].reshape(3, 3), p[8] * 1e-6)
    def fwd(self, xy):
        xy = np.asarray(xy, float).reshape(-1, 2)
        q = (self.H @ np.c_[xy, np.ones(len(xy))].T).T; u = q[:, :2] / q[:, 2:3]
        d = u - C; return C + d * (1 + self.kap * (d ** 2).sum(1))[:, None]
    def undist(self, uv):
        uv = np.asarray(uv, float).reshape(-1, 2); d0 = uv - C; d = d0.copy()
        for _ in range(20): d = d0 / (1 + self.kap * (d ** 2).sum(1))[:, None]
        return C + d
    def inv(self, uv):
        u = self.undist(uv); q = (np.linalg.inv(self.H) @ np.c_[u, np.ones(len(u))].T).T
        return q[:, :2] / q[:, 2:3]
    def jac(self, x, y, e=0.05):
        """d(u,v)/d(x,y) at a floor point (pixels per cm)"""
        a = self.fwd([(x + e, y), (x - e, y), (x, y + e), (x, y - e)])
        return np.column_stack([(a[0] - a[1]) / (2 * e), (a[2] - a[3]) / (2 * e)])
def fit(xy, uv, init, fit_k=True, loss='soft_l1', f_scale=3.0):
    xy = np.asarray(xy, float); uv = np.asarray(uv, float)
    def res(p):
        m = Side.from_p(p if fit_k else np.r_[p, init.kap * 1e6]); return (m.fwd(xy) - uv).ravel()
    p0 = init.p() if fit_k else init.p()[:8]
    r = least_squares(res, p0, loss=loss, f_scale=f_scale, x_scale='jac')
    return Side.from_p(r.x if fit_k else np.r_[r.x, init.kap * 1e6])
