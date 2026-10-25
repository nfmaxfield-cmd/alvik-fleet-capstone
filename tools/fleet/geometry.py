"""Plane geometry for the fleet planner. Floor frame, cm, radians inside the
code (degrees only in files and on screen).

Everything that takes many points is vectorised with numpy, because the
planner calls these thousands of times.
"""
import math

import numpy as np


def wrap(a):
    """Angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_np(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


# ----------------------------------------------------------------------------
# Points, segments, polygons
# ----------------------------------------------------------------------------

def point_seg_dist(p, a, b):
    """Distance from points p (N,2) to segment a-b (2,) or segments a,b (N,2)."""
    p = np.asarray(p, float)
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    ab = b - a
    ap = p - a
    den = np.sum(ab * ab, axis=-1)
    t = np.where(den > 1e-12, np.sum(ap * ab, axis=-1) / np.where(den > 1e-12, den, 1.0), 0.0)
    t = np.clip(t, 0.0, 1.0)
    q = a + t[..., None] * ab
    return np.linalg.norm(p - q, axis=-1)


def points_in_polygon(p, poly):
    """Even-odd test, points p (N,2), polygon (M,2). Returns bool (N,)."""
    p = np.asarray(p, float)
    poly = np.asarray(poly, float)
    x, y = p[:, 0], p[:, 1]
    inside = np.zeros(len(p), bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cond = (y1 > y) != (y2 > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        inside ^= cond & (x < xint)
    return inside


def points_to_polygon(p, poly):
    """Distance from points to a filled polygon (0 inside)."""
    p = np.atleast_2d(np.asarray(p, float))
    poly = np.asarray(poly, float)
    d = np.full(len(p), np.inf)
    for i in range(len(poly)):
        d = np.minimum(d, point_seg_dist(p, poly[i], poly[(i + 1) % len(poly)]))
    d[points_in_polygon(p, poly)] = 0.0
    return d


def _seg_seg_dist(a, b, c, d):
    """Distance between segments a-b and c-d (all (2,))."""
    if _segs_cross(a, b, c, d):
        return 0.0
    return float(min(point_seg_dist(a[None], c, d)[0], point_seg_dist(b[None], c, d)[0],
                     point_seg_dist(c[None], a, b)[0], point_seg_dist(d[None], a, b)[0]))


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segs_cross(a, b, c, d):
    d1, d2 = _cross(c, d, a), _cross(c, d, b)
    d3, d4 = _cross(a, b, c), _cross(a, b, d)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def polygon_polygon_dist(P, Q):
    """Distance between two filled polygons (0 if they touch or overlap)."""
    P = np.asarray(P, float)
    Q = np.asarray(Q, float)
    if points_in_polygon(P[:1], Q)[0] or points_in_polygon(Q[:1], P)[0]:
        return 0.0
    Pa, Pb = P, np.roll(P, -1, axis=0)
    Qa, Qb = Q, np.roll(Q, -1, axis=0)
    # any edges crossing?
    A = Pa[:, None, :]
    B = Pb[:, None, :]
    C = Qa[None, :, :]
    D = Qb[None, :, :]

    def cr(o, a, b):
        return (a[..., 0] - o[..., 0]) * (b[..., 1] - o[..., 1]) - (a[..., 1] - o[..., 1]) * (b[..., 0] - o[..., 0])
    d1, d2, d3, d4 = cr(C, D, A), cr(C, D, B), cr(A, B, C), cr(A, B, D)
    if np.any((d1 * d2 < 0) & (d3 * d4 < 0)):
        return 0.0
    # vertex to edge, both ways
    v1 = point_seg_dist(np.repeat(P, len(Q), 0), np.tile(Qa, (len(P), 1)), np.tile(Qb, (len(P), 1)))
    v2 = point_seg_dist(np.repeat(Q, len(P), 0), np.tile(Pa, (len(Q), 1)), np.tile(Pb, (len(Q), 1)))
    return float(min(v1.min(), v2.min()))


def segment_rect(p1, p2, thickness):
    """Rectangle (4,2) around segment p1-p2 with the given total thickness."""
    p1 = np.asarray(p1, float)
    p2 = np.asarray(p2, float)
    u = p2 - p1
    L = np.linalg.norm(u)
    u = u / L if L > 1e-9 else np.array([1.0, 0.0])
    n = np.array([-u[1], u[0]]) * thickness / 2.0
    return np.array([p1 + n, p2 + n, p2 - n, p1 - n])


# ----------------------------------------------------------------------------
# Robot body (a rectangle; the tracked point is the turning centre)
# ----------------------------------------------------------------------------

class Body:
    """Robot footprint in its own frame: x forward, y left.

    front: distance from the turning centre to the front face
    length, width: outer size.
    bumper: a nose bumper (hardware/dock) sticking out this far in front of the front face: the footprint
    (length, front) includes it; `sensor` stays at the shell's face, where the front distance sensor is.
    bumper_w: the bumper's width (the printed bracket's arms run down the robot's sides): the footprint is at least
    this wide (a rectangle: a little wider than the robot behind the arms too, on the safe side).
    rear, rear_w: the same for an attachment on the robot's BACK (Nick, 2026-10-25: one attachment, switches at the
    back): the footprint reaches `rear` further behind the shell, and is at least rear_w wide."""

    def __init__(self, length=9.5, width=9.5, front=4.75, bumper=0.0, bumper_w=0.0, rear=0.0, rear_w=0.0):
        self.bumper = float(bumper)
        self.rear = float(rear)
        self.length = float(length) + self.bumper + self.rear
        self.width = max(float(width), float(bumper_w) if self.bumper else 0.0, float(rear_w) if self.rear else 0.0)
        self.front = float(front) + self.bumper
        self.sensor = float(front)           # the shell's front face (the distance sensor's readings start here)
        self.back = self.length - self.front
        hw = self.width / 2.0
        self.local = np.array([[self.front, hw], [-self.back, hw], [-self.back, -hw], [self.front, -hw]])
        self.r_turn = float(np.max(np.linalg.norm(self.local, axis=1)))   # circle swept by a turn in place
        self.r_side = hw                                                   # half width
        self.r_in = min(hw, self.front, self.back)                         # largest circle inside

    def corners(self, poses):
        """poses (N,3) -> corners (N,4,2)."""
        poses = np.atleast_2d(np.asarray(poses, float))
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        lx, ly = self.local[:, 0], self.local[:, 1]
        X = poses[:, 0:1] + c[:, None] * lx - s[:, None] * ly
        Y = poses[:, 1:2] + s[:, None] * lx + c[:, None] * ly
        return np.stack([X, Y], axis=-1)

    def drive_sweep(self, a, b, heading):
        """Polygon swept when the body drives straight from point a to b at
        a fixed heading (forwards or backwards)."""
        a = np.asarray(a, float)
        b = np.asarray(b, float)
        ca = self.corners([[a[0], a[1], heading]])[0]
        cb = self.corners([[b[0], b[1], heading]])[0]
        pts = np.vstack([ca, cb])
        return convex_hull(pts)


def convex_hull(pts):
    pts = sorted(map(tuple, np.asarray(pts, float)))
    if len(pts) <= 2:
        return np.array(pts)
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def _rect_gap_core(A, B):
    """A, B: corners (N,4,2) of convex quads. Returns gap (N,) between them,
    0 when they overlap (separating-axis test, then vertex-edge distances)."""
    N = len(A)
    overlap = np.ones(N, bool)
    for P in (A, B):
        for i in range(2):                      # rectangles: 2 axes each
            e = P[:, i + 1] - P[:, i]
            ax = np.stack([-e[:, 1], e[:, 0]], axis=-1)
            pa = np.einsum("nkd,nd->nk", A, ax)
            pb = np.einsum("nkd,nd->nk", B, ax)
            sep = (pa.max(1) < pb.min(1)) | (pb.max(1) < pa.min(1))
            overlap &= ~sep
    d = np.full(N, np.inf)
    for P, Q in ((A, B), (B, A)):
        v = P[:, :, None, :]                       # (N,4,1,2) corners
        a = Q[:, None, :, :]                       # (N,1,4,2) edge starts
        e = np.roll(Q, -1, axis=1)[:, None, :, :] - a
        den = np.sum(e * e, axis=-1)
        t = np.clip(np.sum((v - a) * e, axis=-1) / np.where(den > 1e-12, den, 1.0), 0.0, 1.0)
        q = a + t[..., None] * e
        d = np.minimum(d, np.sqrt(np.sum((v - q) ** 2, axis=-1)).reshape(N, -1).min(1))
    d[overlap] = 0.0
    return d


def body_gaps(body, pa, pb):
    """Exact gap (cm) between two robot bodies at poses pa, pb (N,3) each.
    Cheap circle bounds first; the exact test only where they are not enough."""
    pa = np.atleast_2d(np.asarray(pa, float))
    pb = np.atleast_2d(np.asarray(pb, float))
    dc = np.hypot(pa[:, 0] - pb[:, 0], pa[:, 1] - pb[:, 1])
    out = dc - 2 * body.r_turn                     # lower bound
    exact = (dc < 2 * body.r_turn + 12.0)          # only near ones need the exact number
    if np.any(exact):
        idx = np.nonzero(exact)[0]
        out[idx] = _rect_gap_core(body.corners(pa[idx]), body.corners(pb[idx]))
    return np.maximum(out, 0.0)


def rects_clash(body, pa, pb, thr):
    return bool(np.any(rects_clash_each(body, pa, pb, thr)))


def rects_clash_each(body, pa, pb, thr):
    """Fast, safe test used inside the planner's search: False means the two
    bodies are certainly at least thr apart at every pose pair. It grows each
    rectangle by thr along its own sides and checks both grown shapes against
    the other plain one with separating axes. Near corners it can say "clash"
    for gaps up to about 0.4 x thr bigger than thr (cautious, never unsafe)."""
    pa = np.atleast_2d(pa)
    pb = np.atleast_2d(pb)
    off = (body.front - body.back) / 2.0          # geometric centre ahead of the turning centre
    hx, hy = body.length / 2.0, body.width / 2.0
    ca, sa = np.cos(pa[:, 2]), np.sin(pa[:, 2])
    cb, sb = np.cos(pb[:, 2]), np.sin(pb[:, 2])
    Ax = pa[:, 0] + off * ca
    Ay = pa[:, 1] + off * sa
    Bx = pb[:, 0] + off * cb
    By = pb[:, 1] + off * sb
    dx, dy = Bx - Ax, By - Ay
    # axes: a's u, v and b's u, v
    sep_a_grown = np.zeros(len(pa), bool)
    sep_b_grown = np.zeros(len(pa), bool)
    for (nx, ny) in ((ca, sa), (-sa, ca), (cb, sb), (-sb, cb)):
        dist = np.abs(dx * nx + dy * ny)
        ra = hx * np.abs(ca * nx + sa * ny) + hy * np.abs(-sa * nx + ca * ny)
        rb = hx * np.abs(cb * nx + sb * ny) + hy * np.abs(-sb * nx + cb * ny)
        ra_g = (hx + thr) * np.abs(ca * nx + sa * ny) + (hy + thr) * np.abs(-sa * nx + ca * ny)
        rb_g = (hx + thr) * np.abs(cb * nx + sb * ny) + (hy + thr) * np.abs(-sb * nx + cb * ny)
        sep_a_grown |= dist > ra_g + rb
        sep_b_grown |= dist > ra + rb_g
    return ~sep_a_grown & ~sep_b_grown


def bodies_within(body, pa, pb, thr):
    """Per pair: do the two bodies come closer than thr? Exact: the fast
    cautious test first, then the exact gap only where it says "maybe"."""
    pa = np.atleast_2d(pa)
    pb = np.atleast_2d(pb)
    out = rects_clash_each(body, pa, pb, thr)
    idx = np.nonzero(out)[0]
    if len(idx):
        out[idx] = _rect_gap_core(body.corners(pa[idx]), body.corners(pb[idx])) < thr
    return out


def min_body_gap_below(body, pa, pb, thr):
    """True if any pair of poses comes closer than thr (fast path for the planner)."""
    pa = np.atleast_2d(pa)
    pb = np.atleast_2d(pb)
    dc = np.hypot(pa[:, 0] - pb[:, 0], pa[:, 1] - pb[:, 1])
    if np.all(dc >= 2 * body.r_turn + thr):
        return False
    if np.any(dc < 2 * body.r_in + thr):
        return True
    idx = np.nonzero(dc < 2 * body.r_turn + thr)[0]
    g = _rect_gap_core(body.corners(pa[idx]), body.corners(pb[idx]))
    return bool(np.any(g < thr))
