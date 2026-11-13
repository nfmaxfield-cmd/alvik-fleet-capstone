#!/usr/bin/env python3
"""ONE error shape per robot (e21, Nick 12 Nov, decision 1). It replaces the seven radii of the 12 Nov error and loss
audit (docs/audits/2026-11-12-error-and-loss-audit.md, section B). Plans, the edge rule, the pause, corrections, the
motion-proof limit and Mission Control all read it from here.

Nick's insight: a robot the camera has lost is still driving its own known path, so where it may really be is a CONE
along its own dead-reckoned track, widening sideways. Measured on 865 stretches without a camera fix in 4 home runs
(docs/results/e21-nov12/cone_fit.py): at 45-60 cm blind the sideways error is 3.9 cm typical, 11.9 cm 1 in 10; along
the path 1.6 and 3.7. The along-path tail is in the SHORT stretches (15-30 cm: 9.1 cm 1 in 10): surprises, not
growth.

The along-path error is ONE-SIDED (cone_fit.py, the stretches with no reported event): forward of its own pose at most
2.7 + 0.027 cm per cm (97.5 %), but BEHIND it up to 18.9 cm (97.5 %), the whole stretch in short ones: stuck or
slipping wheels leave the robot back along its own track. So the shape reaches back along its own track (never further
than where its track since the anchor began) and only a little forward.

The shape (95 % bounds, like odo_trust's record), centred on the robot's own pose:
    a disc of radius ISO = the error already seen when the shape was last anchored (the camera's 95 % noise + its own
                           pose's disagreement with the camera then), plus every reported event since (front stop, "felt a
                           robot", "did not move", hazard, re-seed, merge, restart), each by its measured jump;
    + an ellipse, axes along and across the chord of its own track since the anchor, its centre moved back along it:
        forward = k_cal x (b_fwd + a_fwd x s)                        how far past its own pose it may be;
        back    = k_cal x min(s, b_back)                             how far back along its track (stuck wheels);
        across  = k_cal x (b_across + a_across x s + lateral(turns))  the cone's sideways widening;
      the ellipse's along semi-axis is (forward + back) / 2, its centre (forward - back) / 2 along the chord;
    lateral(turns) = s x sin(heading doubt), the doubt from the degrees turned on the spot and on curves (odo_trust's
    per-degree terms), as odo_trust's own model.
s = cm driven since the anchor. The ellipse's support in a direction n is sqrt((along n.u)^2 + (across n.v)^2): its
reach that way. A shape grows with driving and events and SHRINKS ONLY when a fix is accepted (anchor()).

Fit (decision 1: "from each robot's own record and the fleet's, not fixed numbers"): odo_trust's shadow samples carry
their error along and across their own chord (e21); the along and across terms are quantile fits (95 %) to the fleet's
pooled samples, or the robot's own once it differs from the fleet (odo_trust's rule), with the cone fit as the prior
(PRIOR_N rows per stretch length until the record weighs PRIOR_UNTIL). k_cal (odo_trust's own coverage check) widens
both.

Representation for planning (decision 3: "the cheapest representation accurate enough"): per reservation sample the
disc (a scalar margin, as before) plus the ellipse (along, across, its axis angle). Two shapes are kept apart by the
separating-axis test along the line between their centres: their supports along it, which is never less safe than
the truth (a gap there is a real gap) and costs a few multiplies per sample pair. Benchmarked against a circle of the
shape's largest reach and a capsule in docs/results/e21-nov12/plan_cost.py.

MIRLT_ERROR_SHAPE: "cone" (default) | "circle" (the bubble before e21: every user its own radius, as e20).

Changelog
  e21-2026-11-12a  first version.
"""
import json
import math
import os

VERSION = "error-shape-e21-2026-11-12a"

try:
    import safety_group as _SG
    MODE = _SG.value("MIRLT_ERROR_SHAPE")
except Exception:  # noqa: BLE001
    MODE = "cone"

_HERE = os.path.dirname(os.path.abspath(__file__))
_FIT_JSON = os.path.join(_HERE, "..", "..", "docs", "results", "e21-nov12", "cone_fit.json")

# ------------------------------------------------------------------------------------------------- the prior (measured)
# docs/results/e21-nov12/cone_fit.json, the stretches with NO reported event (events widen the shape on their own), 95 %
# quantile fits: along = 11.49 + 0.000 s, across = 0.00 + 0.232 s (+ 0.000 per turned radian x s). Read from the file
# when it is there (the numbers below are the same, for a Pi without docs/).
# forward / back: 97.5 % each side (95 % together) of the signed along error; across: 95 % of its size.
PRIOR = dict(b_fwd=2.66, a_fwd=0.027, b_back=18.88, b_across=0.0, a_across=0.232)
try:
    _f = json.load(open(_FIT_JSON))["fit"]
    _q = _f["quiet_q95"]
    PRIOR.update(b_across=float(_q["b_across"]), a_across=float(_q["a_across"]))
    _a = _f["along_sides"]
    PRIOR.update(b_fwd=float(_a["b_fwd"]), a_fwd=float(_a["a_fwd"]), b_back=float(_a["b_back"]))
except Exception:  # noqa: BLE001
    pass
# Event jumps (decision 1: "events widen it at once, by measured amounts"): the own-vs-camera gap 2-3 s after each event
# less the gap the second before it, 95th percentile, the 8 Nov... 11 Nov home runs (cone_fit.json "event_jumps").
EVENT_CM = {"did not move": 7.0, "felt a robot": 8.1, "front stop": 8.1, "hazard": 9.9, "merge": 4.8,
            "pose re-seeded": 2.9}
try:
    _ej = json.load(open(_FIT_JSON))["event_jumps"]
    for _k, _key in (("did not move", "did_not_move"), ("felt a robot", "felt_robot"), ("front stop", "front_stop"),
                     ("hazard", "hazard"), ("merge", "merge"), ("pose re-seeded", "reseed")):
        if _key in _ej and _ej[_key].get("p95") is not None:
            EVENT_CM[_k] = float(_ej[_key]["p95"])
except Exception:  # noqa: BLE001
    pass
# no restart in those logs: a restart widens by the largest measured jump (CHOSEN: the cautious end of the measured
# events). DOE: no; measured from the first real runs with e19's restart handling
EVENT_CM["restart"] = max(EVENT_CM.values())
EVENT_ALIASES = {"front stop": "front stop", "felt a robot": "felt a robot", "did not move": "did not move",
                 "hazard": "hazard", "pose re-seeded": "pose re-seeded", "close pass": "merge", "merged": "merge",
                 "restart": "restart", "restarted": "restart"}
PRIOR_N = 5            # odo_trust's: the prior counts as 5 samples per stretch length ...
PRIOR_UNTIL = 20       # ... until the record weighs 20 (odo_trust.PRIOR_UNTIL)
STRETCHES_CM = (10.0, 20.0, 40.0)
FIT_Q = 0.95
SIDE_Q = 0.975         # each side of a one-sided error: 97.5 % (the two together 95 %)


def _qfit(rows, w=None):
    """95 % quantile fit e = b + a s (+ c x lateral) with every term >= 0; rows (s, lat, e). None without scipy."""
    try:
        import numpy as np
        from scipy.optimize import linprog
    except Exception:  # noqa: BLE001
        return None
    n = len(rows)
    if n < 3:
        return None
    X = np.array([[1.0, r[0], r[1]] for r in rows])
    e = np.array([r[2] for r in rows])
    W = np.ones(n) if w is None else np.asarray(w, dtype=float)
    c = np.r_[np.zeros(3), FIT_Q * W, (1 - FIT_Q) * W]
    A = np.hstack([X, np.eye(n), -np.eye(n)])
    try:
        res = linprog(c, A_eq=A, b_eq=e, bounds=[(0, None)] * (3 + 2 * n), method="highs")
    except Exception:  # noqa: BLE001
        return None
    if not res.success:
        return None
    return [float(v) for v in res.x[:3]]


class AxesFit:
    """The along / across terms for one robot from its odometry record (odo_trust.Trust, pooled with the fleet's as
    odo_trust pools), refitted after REFIT_N new samples. Samples without along / across (records saved before e21)
    are not used; the prior stands until there are enough."""
    REFIT_N = 5

    def __init__(self, trust):
        self.trust = trust
        self._fit = None
        self._key = None

    def rows(self):
        tr = self.trust
        if tr is None:
            return []
        try:
            pool = [tr] if (tr.own_record or not tr._pooled()) else list(tr.fleet.values())
            return [q for q in tr._rows_of(pool) if "al" in q and "ac" in q]
        except Exception:  # noqa: BLE001
            return []

    def _lat(self, q):
        try:
            _, _, ps, pc, _ = self.trust.fit()
        except Exception:  # noqa: BLE001
            return 0.0
        return min(q["s"], (ps * q.get("ws", 0.0) + pc * q.get("wc", 0.0)) * math.pi / 180.0)

    def get(self):
        tr = self.trust
        key = tr._fleet_n() if tr is not None else 0
        if self._fit is not None and key - (self._key or 0) < self.REFIT_N:
            return self._fit
        fs = self.rows()
        wts = [float(q.get("w", 1.0)) for q in fs]
        n_prior = PRIOR_N if sum(wts) < PRIOR_UNTIL else 0
        rf = [(q["s"], 0.0, max(0.0, q["al"])) for q in fs]
        rb = [(q["s"], 0.0, max(0.0, -q["al"])) for q in fs]
        # the across fit is of its straight-line growth; the turns' sideways reach is odo_trust's lateral term (added in
        # axes_at), so a sample's own lateral reach is taken off before fitting
        rc = [(q["s"], 0.0, max(0.0, abs(q["ac"]) - self._lat(q))) for q in fs]
        for L in STRETCHES_CM:
            for _ in range(n_prior):
                rf.append((L, 0.0, PRIOR["b_fwd"] + PRIOR["a_fwd"] * L))
                rb.append((L, 0.0, min(L, PRIOR["b_back"])))
                rc.append((L, 0.0, PRIOR["b_across"] + PRIOR["a_across"] * L))
        wa = wts + [1.0] * (len(rf) - len(wts))
        W = None if all(x == 1.0 for x in wa) else wa
        global FIT_Q
        q0 = FIT_Q
        FIT_Q = SIDE_Q                                       # each side of the along error: 97.5 %
        sf, sb = _qfit(rf, W), _qfit(rb, W)
        FIT_Q = q0
        sc = _qfit(rc, W)
        fit = dict(PRIOR, n=len(fs), source="prior" if not fs else "record")
        if sf is not None and fs:
            fit.update(b_fwd=sf[0], a_fwd=sf[1])
        if sb is not None and fs:
            fit.update(b_back=sb[0] + sb[1] * STRETCHES_CM[-1])     # its tail at the longest stretch (a cap)
        if sc is not None and fs:
            fit.update(b_across=sc[0], a_across=sc[1])
        self._fit, self._key = fit, key
        return fit


class Shape:
    """One robot's error shape. Fed by the controller: anchor() on every accepted fix (or a pose set by a person or a
    correction), drive() with its own motion, event() for what it reports. Read by everyone else."""

    def __init__(self, name, trust=None):
        self.name = name
        self.trust = trust
        self.axes_fit = AxesFit(trust)
        self.t0 = None
        self.ax = self.ay = None       # the anchor: its own pose when the shape was last anchored
        self.base = 2.45               # the error seen at the anchor (cm): camera noise (95 %) + own-vs-camera gap
        self.ev = 0.0                  # events since the anchor (cm, added to the disc)
        self.events = []               # (t, kind, cm)
        self.s = self.ts = self.tc = self.ms = self.mc = 0.0
        self.h = None                  # its own heading now (deg)
        self.x = self.y = None         # its own pose now
        self.why = "start"
        self.hd_deg = None             # its heading doubt (95 %, deg) set by the controller: unknown (180) after a
                                       # correction until the camera confirms it

    # -- feeding ---------------------------------------------------------------------------------------------------------
    def anchor(self, t, x, y, h=None, base_cm=2.45, why="fix"):
        """An accepted fix (or a pose set from evidence): the shape starts again from the error seen now."""
        self.t0, self.ax, self.ay = t, x, y
        self.x, self.y = x, y
        if h is not None:
            self.h = h
        self.base = max(0.0, float(base_cm))
        self.ev = 0.0
        self.events = []
        self.s = self.ts = self.tc = self.ms = self.mc = 0.0
        self.why = why

    def widen_base(self, cm):
        """The error already seen grows (its own pose disagrees with the camera more than at the anchor): never shrinks
        here."""
        self.base = max(self.base, float(cm))

    def drive(self, x, y, h, ds, dth, spot):
        """Its own motion: ds cm (abs), dth deg turned (abs), spot: turned on the spot."""
        if self.ax is None:
            self.anchor(0.0, x, y, h, self.base, "first pose")
        self.x, self.y, self.h = x, y, h
        self.s += abs(ds)
        a = abs(dth)
        if spot:
            self.ts += a
            self.ms += a * self.s
        else:
            self.tc += a
            self.mc += a * self.s

    def event(self, t, kind):
        """A reported event widens the disc at once by its measured jump. Returns the cm added (0: not an event)."""
        k = EVENT_ALIASES.get(kind)
        if k is None:
            return 0.0
        cm = EVENT_CM.get(k, 0.0)
        # independent surprises add in quadrature (95 % of each: the 95 % of their sum is near the root of the sum of
        # squares), so a robot that reports the same edge stop 20 times is not given 20 jumps end to end
        before = self.ev
        self.ev = math.hypot(self.ev, cm)
        self.events.append((round(t, 2), k, cm))
        return self.ev - before

    # -- reading ---------------------------------------------------------------------------------------------------------
    def k_cal(self):
        try:
            return max(1.0, float(self.trust.k_cal))
        except Exception:  # noqa: BLE001
            return 1.0

    def head_doubt_rad(self, ts=None, tc=None, ws=None, wc=None):
        """The sideways reach of its heading doubt, as odo_trust's lateral term (deg x cm -> cm)."""
        s = self.s
        ts = self.ts if ts is None else ts
        tc = self.tc if tc is None else tc
        ws = (s * ts - self.ms) if ws is None else ws
        wc = (s * tc - self.mc) if wc is None else wc
        try:
            _, _, ps, pc, _ = self.trust.fit()
        except Exception:  # noqa: BLE001
            ps = pc = 0.04
        return (ps * ws + pc * wc) * math.pi / 180.0

    def axes_at(self, s, lat):
        """(forward, back, across) after s cm with lateral reach lat (cm)."""
        f = self.axes_fit.get()
        k = self.k_cal()
        fwd = k * (f["b_fwd"] + f["a_fwd"] * s) if s > 0 else 0.0
        back = k * min(s, f["b_back"])
        across = k * (f["b_across"] + f["a_across"] * s + min(s, lat))
        if self.hd_deg:
            # a heading in doubt by hd: where s cm of driving ends is on an arc of that half-angle round the commanded
            # end: s sin(hd) across, s (1 - cos hd) back (unknown, 180: anywhere within s)
            hd = math.radians(min(180.0, self.hd_deg))
            across = max(across, s * math.sin(min(hd, math.pi / 2)))
            back = max(back, s * (1.0 - math.cos(hd)))
        return fwd, back, across

    @staticmethod
    def _ell(iso, fwd, back, across, ang):
        """(iso, along semi-axis, across, angle, centre offset along the chord: + forward)."""
        return iso, 0.5 * (fwd + back), across, ang, 0.5 * (fwd - back)

    def axes(self):
        return self.axes_at(self.s, self.head_doubt_rad())

    def iso(self):
        return self.base + self.ev

    def angle(self):
        """The direction of the chord of its track since the anchor (deg): the cone's long axis. Before it has driven
        1 cm, its heading."""
        if self.ax is not None and self.x is not None and math.hypot(self.x - self.ax, self.y - self.ay) >= 1.0:
            return math.degrees(math.atan2(self.y - self.ay, self.x - self.ax))
        return self.h if self.h is not None else 0.0

    def ellipse(self):
        """(iso, along semi-axis, across, angle deg, centre offset along the chord: negative = back along its track)."""
        f, bk, c = self.axes()
        return self._ell(self.iso(), f, bk, c, self.angle())

    def extent(self):
        """Its largest reach in any direction from its own pose (cm): the circle that holds the whole shape."""
        i, a, b, _, off = self.ellipse()
        return i + max(a + abs(off), b)

    def support(self, nx, ny, ell=None):
        i, a, b, ang, off = ell or self.ellipse()
        return i + support_ellipse(a, b, ang, nx, ny) + off * (nx * math.cos(math.radians(ang)) +
                                                               ny * math.sin(math.radians(ang)))

    def summary(self):
        i, a, b, ang, off = self.ellipse()
        f, bk, _ = self.axes()
        return dict(iso=round(i, 2), forward=round(f, 2), back=round(bk, 2), across=round(b, 2), angle=round(ang, 1),
                    s=round(self.s, 1), extent=round(self.extent(), 2), base=round(self.base, 2),
                    events=list(self.events[-5:]), why=self.why, fit=self.axes_fit.get().get("source"))

    # -- planning ahead --------------------------------------------------------------------------------------------------
    def along_path(self, pts, h0=None):
        """The shape at each point of a planned path [(x, y)] driven from its pose now (its track continues): lists
        (iso, along, across, angle, offset) per point. A path's turns add heading doubt as odo_trust's model."""
        out = []
        s, ts, tc, ms, mc = self.s, self.ts, self.tc, self.ms, self.mc
        px, py = (self.x, self.y) if self.x is not None else (pts[0] if pts else (0.0, 0.0))
        ph = self.h if h0 is None else h0
        ax, ay = (self.ax, self.ay) if self.ax is not None else (px, py)
        iso = self.iso()
        for q in pts:
            d = math.hypot(q[0] - px, q[1] - py)
            if d > 1e-6:
                hh = math.degrees(math.atan2(q[1] - py, q[0] - px))
                if ph is not None:
                    a_ = abs((hh - ph + 180.0) % 360.0 - 180.0)
                    if a_ > 30.0:            # a turn on the spot before this leg (odo_trust.path_radii's rule)
                        ts += a_
                        ms += a_ * s
                    else:
                        tc += a_
                        mc += a_ * s
                ph = hh
                s += d
                px, py = q[0], q[1]
            lat = self.head_doubt_rad(ts, tc, s * ts - ms, s * tc - mc)
            fw, bk, ac = self.axes_at(s, lat)
            ang = math.degrees(math.atan2(py - ay, px - ax)) if math.hypot(px - ax, py - ay) >= 1.0 else (ph or 0.0)
            out.append(self._ell(iso, fw, bk, ac, ang))
        return out


# ------------------------------------------------------------------------------------------------- geometry
def support_ellipse(a, b, ang_deg, nx, ny):
    """Reach of an ellipse (semi-axes a along ang, b across) in the unit direction (nx, ny)."""
    c, s = math.cos(math.radians(ang_deg)), math.sin(math.radians(ang_deg))
    pu = nx * c + ny * s
    pv = -nx * s + ny * c
    return math.sqrt((a * pu) ** 2 + (b * pv) ** 2)


def rect_margin(rect, x, y, iso, a, b, ang_deg, off=0.0):
    """How far the shape centred at (x, y) stays inside the rectangle (x0, y0, x1, y1): the least over the four sides of
    (distance to that side - the shape's reach toward it). A rect with its own edge_dist (a proven workspace): the shape's
    largest reach (the workspace has no sides to aim at)."""
    c, s = math.cos(math.radians(ang_deg)), math.sin(math.radians(ang_deg))
    x, y = x + off * c, y + off * s                  # the ellipse's own centre (back along its track)
    f = getattr(rect, "edge_dist", None)
    if f is not None:
        return f(x, y) - iso - max(a, b)
    x0, y0, x1, y1 = rect[:4]
    rx = math.sqrt((a * c) ** 2 + (b * s) ** 2)      # reach along +-x
    ry = math.sqrt((a * s) ** 2 + (b * c) ** 2)      # reach along +-y
    return min(x - x0 - rx, x1 - x - rx, y - y0 - ry, y1 - y - ry) - iso


def rect_margin_v(rect, xs, ys, iso, a, b, ang_deg, off=0.0):
    """rect_margin over arrays (numpy)."""
    import numpy as np
    c, s = np.cos(np.radians(ang_deg)), np.sin(np.radians(ang_deg))
    xs, ys = xs + off * c, ys + off * s
    f = getattr(rect, "edge_dist", None)
    if f is not None:
        return np.array([f(float(x), float(y)) for x, y in zip(xs, ys)]) - iso - np.maximum(a, b)
    x0, y0, x1, y1 = rect[:4]
    rx = np.sqrt((a * c) ** 2 + (b * s) ** 2)
    ry = np.sqrt((a * s) ** 2 + (b * c) ** 2)
    return np.minimum(np.minimum(xs - x0 - rx, x1 - xs - rx), np.minimum(ys - y0 - ry, y1 - ys - ry)) - iso


def _centre(p, s_):
    if len(s_) > 4 and s_[4]:
        c, s = math.cos(math.radians(s_[3])), math.sin(math.radians(s_[3]))
        return (p[0] + s_[4] * c, p[1] + s_[4] * s)
    return p


def pair_gap(p, sa, q, sb):
    """The gap between two shapes (cm, before bodies): centre distance less each one's reach along the line between them
    (the separating-axis test along that line: a positive gap is a real gap). sa, sb: (iso, along, across, angle[,
    offset])."""
    p, q = _centre(p, sa), _centre(q, sb)
    dx, dy = q[0] - p[0], q[1] - p[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return -(sa[0] + max(sa[1], sa[2]) + sb[0] + max(sb[1], sb[2]))
    ux, uy = dx / d, dy / d
    return d - sa[0] - support_ellipse(sa[1], sa[2], sa[3], ux, uy) - sb[0] - support_ellipse(sb[1], sb[2], sb[3], -ux,
                                                                                               -uy)


def capsule_gap(p, sa, q, sb):
    """The same gap with each ellipse bounded by a capsule (radius = its shorter semi-axis, a segment of length 2 x
    (longer - shorter) along the longer one): exact for the capsules, for plan_cost.py's comparison."""
    p, q = _centre(p, sa), _centre(q, sb)

    def seg(c, s_):
        iso, a, b, ang = s_[:4]
        if a >= b:
            r, L, th = b, a - b, ang
        else:
            r, L, th = a, b - a, ang + 90.0
        ux, uy = math.cos(math.radians(th)), math.sin(math.radians(th))
        return (c[0] - ux * L, c[1] - uy * L), (c[0] + ux * L, c[1] + uy * L), iso + r

    def segdist(p1, q1, p2, q2):
        def clamp(v):
            return max(0.0, min(1.0, v))
        d1 = (q1[0] - p1[0], q1[1] - p1[1])
        d2 = (q2[0] - p2[0], q2[1] - p2[1])
        r_ = (p1[0] - p2[0], p1[1] - p2[1])
        a = d1[0] ** 2 + d1[1] ** 2
        e = d2[0] ** 2 + d2[1] ** 2
        f = d2[0] * r_[0] + d2[1] * r_[1]
        if a <= 1e-12 and e <= 1e-12:
            return math.hypot(*r_)
        if a <= 1e-12:
            s_, t_ = 0.0, clamp(f / e)
        else:
            c_ = d1[0] * r_[0] + d1[1] * r_[1]
            if e <= 1e-12:
                t_, s_ = 0.0, clamp(-c_ / a)
            else:
                b_ = d1[0] * d2[0] + d1[1] * d2[1]
                den = a * e - b_ * b_
                s_ = clamp((b_ * f - c_ * e) / den) if den > 1e-12 else 0.0
                t_ = (b_ * s_ + f) / e
                if t_ < 0:
                    t_, s_ = 0.0, clamp(-c_ / a)
                elif t_ > 1:
                    t_, s_ = 1.0, clamp((b_ - c_) / a)
        c1 = (p1[0] + d1[0] * s_, p1[1] + d1[1] * s_)
        c2 = (p2[0] + d2[0] * t_, p2[1] + d2[1] * t_)
        return math.hypot(c1[0] - c2[0], c1[1] - c2[1])
    a1, a2, ra = seg(p, sa)
    b1, b2, rb = seg(q, sb)
    return segdist(a1, a2, b1, b2) - ra - rb


def true_gap(p, sa, q, sb, n=180):
    """The exact gap between two shapes (disc + ellipse each), by sampling directions (for tests and plan_cost.py)."""
    p, q = _centre(p, sa), _centre(q, sb)
    best = 1e9
    for k in range(n):
        th = 2 * math.pi * k / n
        ux, uy = math.cos(th), math.sin(th)
        # separation along u: projection gap
        g = (q[0] - p[0]) * ux + (q[1] - p[1]) * uy - sa[0] - support_ellipse(sa[1], sa[2], sa[3], ux, uy) - sb[0] - \
            support_ellipse(sb[1], sb[2], sb[3], -ux, -uy)
        best = min(best, -g)
    return -best          # the largest separation over all axes (convex sets: the true gap when positive)
