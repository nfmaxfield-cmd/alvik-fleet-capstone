"""Light-pattern detector: one camera frame -> pattern sightings, and a small
tracker that names them (patterns.py has the patterns).

Per frame (Detector.frame):
  1. Spots: small bright things. Top-hat (the picture minus its opening with a
     disc a bit wider than one LED image) at least t_top grey levels, and the
     picture itself at least v_min. One spot per connected blob, at its
     brightness-weighted centre. A blob stretched one way (two LEDs merged at
     low resolution) is split in two along its long axis.
  2. Floor: each spot to floor cm at the LEDs' height (FloorMap: lens term,
     homography, height correction).
  3. l3: every triple whose spacing fits the L (corner -> front about arm_cm,
     corner -> side square to it) is fitted to every holder variant (rigid fit,
     least squares); the best fit per triple must be within chi2_full. Triples
     are taken best first; a triple that shares a spot with another acceptable
     triple is AMBIGUOUS (e.g. the robot's own blink LEDs next to the front
     LED can form a second L): it gives position only, no identity evidence.
     strip: blobs within a strip's length are grouped, the brightness profile
     along their long axis is compared with every code, both ways round
     (shift and blur fitted), and with a free fit (any 8 levels) for "not a
     listed code".
Per track (Tracker.step):
  4. Sightings join tracks by position (gate grows with top speed x time);
     an l3 track that knows its pose also takes 2 of its 3 LEDs (one hidden
     or switched off by the sequence).
  5. Identity: each full sighting's fit gives a probability per holder
     variant / strip code and "none"; a track adds these up (each frame may be
     wrong with eps_frame; older frames count `forget` less per frame). With
     a side-LED sequence in use (any listed code not steady), the side LED's
     on/off over the last cycles is matched to each sequence too. A track is
     NAMED when its best listed robot has posterior >= name_p, it has had
     min_full frames, (with sequences) one whole cycle was seen, and no other
     live track elsewhere carries or may carry that name (nor an outside
     claim, e.g. a blink track). Two tracks closer than swap_cm forget their
     evidence (they could swap) and must re-earn the name from fresh frames.
"""
import math
from collections import deque
from dataclasses import dataclass

import numpy as np
import cv2

import patterns as PT


@dataclass
class Params:
    # spots
    t_top: float = 40.0          # top-hat: an LED stands at least this far above its surroundings (grey levels)
    v_min: float = 240.0         # ...and is at least this bright (a lit LED saturates; the robot's white top does not)
    max_spots: int = 120
    sigma_px: float = 0.45       # spot centre noise, px (the fit's measurement noise)
    sigma_cm: float = 0.06       # plus this (holder print, LED placement), cm
    merged_px: float = 1.0       # extra noise of a spot split from a merged blob, px
    # l3 matching
    arm_tol: float = 0.25        # corner -> front within this fraction of arm_cm
    chi2_full: float = 16.0      # a fit's sum of squared residuals / sigma^2 must be under this (3 points, 6 values)
    chi2_none: float = 18.0      # "none of the variants": as likely as a fit with this chi2
    chi2_amb: float = 9.0        # a rival triple sharing a spot makes a sighting ambiguous if it fits within this of it
    # tracking and naming
    name_p: float = 0.9
    min_full: int = 1            # full sightings before a name (1 = named from a single frame)
    eps_frame: float = 0.02      # chance one frame's identity evidence is wrong
    forget: float = 0.9          # evidence from older frames counts this much less per frame
    prior_none: float = 0.2
    gate_cm: float = 1.5         # a sighting joins a track within this of its predicted centre...
    vmax_cms: float = 40.0       # ...plus half the top speed x the time since it was last seen
    turn_gate_deg: float = 25.0  # ...and within this of its predicted heading (+ max_turn_dps x time)
    max_turn_dps: float = 200.0
    min_sep_cm: float = 8.0      # robots can't overlap: a new track this close to a robot seen now is not a robot
    track_gap_s: float = 1.0
    swap_cm: float = 12.0        # two tracks this close forget their identity evidence
    contest_cm: float = 6.0
    claim_hold_s: float = 1.0    # a name a track carried counts as its claim this long after
    # side-LED sequence
    seq_eps: float = 0.1         # one frame's side-LED reading is wrong this often
    seq_guard_s: float = 0.03    # readings this close to a slot change are left out
    seq_cycles: float = 1.0      # whole cycles seen before the sequence counts
    seq_phase_step_s: float = 0.02
    # strip
    strip_noise: float = 8.0     # profile noise, grey levels (+ 5% of the fitted brightness)
    strip_none_bic: float = 1.0  # weight of the free fit's parameter penalty
    strip_min_pitch_px: float = 2.8   # LED pitch in the picture below this: the strip can't be read (decoder off)
    strip_v_min: float = 240.0   # strip LEDs: the same brightness floor (set lights_strip_bri so a lit LED just saturates)


def logsumexp(v):
    v = list(v)
    if not v:
        return -math.inf
    m = max(v)
    if m == -math.inf:
        return m
    return m + math.log(sum(math.exp(x - m) for x in v))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rigid_fit(T, O, w=None):
    """2-D rigid fit O ~ R(th) T + c. Returns (th, c, sse)."""
    T = np.asarray(T, float)
    O = np.asarray(O, float)
    w = np.ones(len(T)) if w is None else np.asarray(w, float)
    ws = w.sum()
    tc = (T * w[:, None]).sum(0) / ws
    oc = (O * w[:, None]).sum(0) / ws
    a, b = T - tc, O - oc
    s = float((w * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])).sum())
    cc = float((w * (a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])).sum())
    th = math.atan2(s, cc)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    c = oc - R @ tc
    res = O - (T @ R.T + c)
    return th, c, float((w * (res ** 2).sum(1)).sum())


def to_world(th, c, P):
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return np.asarray(P, float) @ R.T + c


class SimpleMap:
    """Stand-in for tools/live/floormap.FloorMap without a calibration: a
    plain scale, image y down = floor y up."""

    def __init__(self, px_per_cm, size=(640, 480)):
        self.s = px_per_cm
        self.size = size

    def to_floor(self, u, v):
        return u / self.s, -v / self.s

    def to_px(self, x, y, z=None):
        return x * self.s, -y * self.s

    def px_per_cm(self, u=None, v=None):
        return self.s


class Detector:
    def __init__(self, fam, fmap, p=None):
        self.fam, self.fmap, self.p = fam, fmap, p or Params()
        self.ppc = fmap.px_per_cm()
        k = max(5, 2 * int(round(1.1 * self.ppc)) + 1)
        self.kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        self.split_px = max(1.2, 0.22 * self.ppc)
        if fam.kind == "l3":
            self.geoms = fam.geoms()
            self.T = {g: fam.leds(g) for g in self.geoms}
            self.side_lo = 0.6 * min(fam.side_cm)
            self.side_hi = 1.3 * max(fam.side_cm)
        else:
            self.geoms = fam.codes()
            self._strip_models()
        self.note = ""
        if fam.kind == "strip" and self.pitch_px < self.p.strip_min_pitch_px:
            self.note = ("strip LEDs %.1f px apart in this picture (need %.1f): the code can't be read at this scale, "
                         "strip decoding is off" % (self.pitch_px, self.p.strip_min_pitch_px))
        self.top = None

    # ------------------------------------------------------------ spots
    def spots(self, img):
        p = self.p
        if img.ndim == 3:
            img = np.min(img, axis=2) if img.shape[2] > 1 else img[:, :, 0]
        core = cv2.blur(img.astype(np.float32), (2, 2))
        top = core - cv2.morphologyEx(core, cv2.MORPH_OPEN, self.kern)
        self.top, self.core = top, core
        vmin = p.v_min if self.fam.kind == "l3" else p.strip_v_min
        mask = ((top >= p.t_top) & (core >= vmin)).astype(np.uint8)
        n, lab, st, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        out = []
        if n <= 1:
            return out
        order = np.argsort(-st[1:, cv2.CC_STAT_AREA])[:p.max_spots] + 1
        H, W = top.shape
        rw = max(2, int(round(0.6 * self.ppc)))
        for i in order:
            x0, y0, w, h, a = (int(v) for v in st[i])
            if a > 400:
                continue
            # the glow around the saturated core counts too: weights in a window around the blob
            X0, Y0, X1, Y1 = max(0, x0 - rw), max(0, y0 - rw), min(W, x0 + w + rw), min(H, y0 + h + rw)
            win = top[Y0:Y1, X0:X1]
            peak = float(top[y0:y0 + h, x0:x0 + w][lab[y0:y0 + h, x0:x0 + w] == i].max())
            wt = np.maximum(win - 0.3 * peak, 0.0)
            tot = float(wt.sum())
            if tot <= 0:
                continue
            yy, xx = np.mgrid[Y0:Y1, X0:X1]
            mu = np.array([(wt * xx).sum() / tot, (wt * yy).sum() / tot])
            dx, dy = xx - mu[0], yy - mu[1]
            C = np.array([[(wt * dx * dx).sum(), (wt * dx * dy).sum()], [(wt * dx * dy).sum(), (wt * dy * dy).sum()]]) / tot
            spot = dict(u=float(mu[0]), v=float(mu[1]), peak=peak, area=a, merged=False, comp=i, cov=C,
                        bbox=(x0, y0, w, h))
            # two LEDs merged into one blob: its saturated core (the mask) is long and narrow
            ev = None
            if self.fam.kind == "l3" and a >= 5:
                my, mx = np.nonzero(lab[y0:y0 + h, x0:x0 + w] == i)
                ev, evec = np.linalg.eigh(np.cov(np.stack([mx, my]).astype(float)))
            if ev is not None and ev[1] > 3.0 * max(ev[0], 0.25) and math.sqrt(ev[1] - ev[0]) > self.split_px:
                d = math.sqrt(ev[1] - ev[0])
                ax = evec[:, 1]
                for sgn in (1, -1):
                    out.append(dict(spot, u=float(mu[0] + sgn * d * ax[0]), v=float(mu[1] + sgn * d * ax[1]),
                                    merged=True))
            else:
                out.append(spot)
        self.lab = lab
        for s in out:
            s["xy"] = np.array(self.fmap.to_floor(s["u"], s["v"]), float)
        return out

    def sigma_of(self, sp):
        e = self.p.sigma_px + (self.p.merged_px if sp["merged"] else 0.0)
        return math.hypot(e / self.ppc, self.p.sigma_cm)

    # ------------------------------------------------------------ l3
    def _fit_triple(self, S, idx):
        O = np.array([S[i]["xy"] for i in idx])
        sig = np.array([self.sigma_of(S[i]) for i in idx])
        w = 1.0 / sig ** 2
        lls = {}
        best = None
        for g in self.geoms:
            th, c, sse = rigid_fit(self.T[g], O, w)
            lls[g] = -0.5 * sse
            if best is None or sse < best[3]:
                best = (g, th, c, sse)
        return lls, best

    def l3(self, S):
        fam, p = self.fam, self.p
        n = len(S)
        P = np.array([s["xy"] for s in S]) if n else np.zeros((0, 2))
        cands = []
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                d = P[j] - P[i]
                L = math.hypot(*d)
                if abs(L - fam.arm_cm) > p.arm_tol * fam.arm_cm:
                    continue
                a = d / L
                nrm = np.array([-a[1], a[0]])
                for k in range(n):
                    if k in (i, j):
                        continue
                    v = P[k] - P[i]
                    along, across = float(v @ a), float(v @ nrm)
                    if abs(along) > 0.35 * self.side_lo + 3 * self.sigma_of(S[k]) or not (
                            self.side_lo <= abs(across) <= self.side_hi):
                        continue
                    lls, best = self._fit_triple(S, (i, j, k))
                    if best[3] <= p.chi2_full:
                        cands.append((best[3], (i, j, k), lls, best))
        cands.sort(key=lambda c: c[0])
        used, dets = set(), []
        for sse, idx, lls, best in cands:
            if used & set(idx):
                continue
            # another acceptable triple sharing a spot (or the same spots in other roles): ambiguous
            amb = any(c[1] != idx and set(c[1]) & set(idx) and c[0] <= sse + p.chi2_amb for c in cands)
            used |= set(idx)
            g, th, c, _ = best
            dets.append(dict(kind="full", spots=[S[i] for i in idx], idx=idx, geom=g, th=th, c=c, sse=sse,
                             lls=None if amb else lls, ambiguous=amb))
        rest = [S[i] for i in range(n) if i not in used]
        return dets, rest

    def frame_post(self, lls):
        """One full sighting's probabilities over the variants and 'none' (flat prior)."""
        ll = dict(lls)
        ll["none"] = -0.5 * self.p.chi2_none
        m = max(ll.values())
        z = sum(math.exp(v - m) for v in ll.values())
        return {k: math.exp(v - m) / z for k, v in ll.items()}

    # ------------------------------------------------------------ strip
    def _strip_models(self):
        fam = self.fam
        self.pitch_px = fam.pitch_cm * self.ppc
        self.strip_len_px = (fam.n - 1) * self.pitch_px
        half = self.strip_len_px / 2 + max(3.0, 1.5 * self.pitch_px)
        self.s_grid = np.arange(-half, half + 1e-6, 0.5)
        pos = (np.arange(fam.n) - (fam.n - 1) / 2) * self.pitch_px      # LED 0 at -: the front end
        models, keys = [], []
        for code in fam.codes():
            b = np.array(fam.bits(code), float)
            for dr in (1, -1):
                for sh in np.arange(-1.5, 1.51, 0.25):
                    for sg in (0.6, 0.9, 1.3, 1.8):
                        x = dr * pos + sh
                        m = (b[None, :] * np.exp(-0.5 * ((self.s_grid[:, None] - x[None, :]) / sg) ** 2)).sum(1)
                        models.append(m)
                        keys.append((code, dr, sh, sg))
        self.models = np.array(models)
        self.mkeys = keys
        # free fit: each of the 8 LEDs its own level (same shift / blur grid), for "not a listed code"
        self.free_basis = {}
        for sh in np.arange(-1.5, 1.51, 0.25):
            for sg in (0.6, 0.9, 1.3, 1.8):
                x = pos + sh
                self.free_basis[(sh, sg)] = np.exp(-0.5 * ((self.s_grid[:, None] - x[None, :]) / sg) ** 2)

    def _profile(self, mu, ax):
        nrm = np.array([-ax[1], ax[0]])
        acc = 0
        for o in (-1.0, 0.0, 1.0):
            us = (mu[0] + self.s_grid * ax[0] + o * nrm[0]).astype(np.float32)
            vs = (mu[1] + self.s_grid * ax[1] + o * nrm[1]).astype(np.float32)
            acc = acc + cv2.remap(self.top, us[None, :], vs[None, :], cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=0)[0]
        return acc / 3.0

    def strip(self, S):
        """Strip sightings: pairs of spots a strip's length apart (both ends of every code are lit) with
        the spots between them on their line; the brightness profile along that line is fitted."""
        p = self.p
        n = len(S)
        if not n or self.note:
            return [], list(S)
        P = np.array([[s["u"], s["v"]] for s in S])
        lo, hi = 6.0 * self.pitch_px - 2.0, 7.0 * self.pitch_px + 2.0
        cands = []
        for i in range(n):
            for j in range(i + 1, n):
                d = P[j] - P[i]
                span = math.hypot(*d)
                if not (lo <= span <= hi):
                    continue
                ax = d / span
                rel = P - P[i]
                along = rel @ ax
                perp = np.abs(rel @ np.array([-ax[1], ax[0]]))
                on = [k for k in range(n) if perp[k] <= 1.5 + 0.25 * self.pitch_px and -1.0 <= along[k] <= span + 1.0]
                near = [k for k in range(n) if k not in on and perp[k] <= 3.0 + self.pitch_px
                        and -self.pitch_px - 3 <= along[k] <= span + self.pitch_px + 3]
                if near:
                    continue              # something bright right beside the line: not a clean strip
                mu = 0.5 * (P[i] + P[j])
                Hh, Ww = self.top.shape
                ends = [mu + sg * ax * (span / 2 + self.pitch_px + 2) for sg in (1, -1)]
                if any(not (2 <= e[0] <= Ww - 3 and 2 <= e[1] <= Hh - 3) for e in ends):
                    continue              # part of the strip may be out of the picture
                res = self._strip_fit(self._profile(mu, ax))
                if res is None:
                    continue
                lls, best, ll_free = res
                cands.append((-max(lls.values()), tuple(on), mu, ax, lls, best, ll_free))
        cands.sort(key=lambda c: c[0])
        used, dets = set(), []
        for negll, on, mu, ax, lls, best, ll_free in cands:
            if used & set(on):
                continue
            amb = any(c[1] != on and set(c[1]) & set(on) and c[0] <= negll + 4.5 for c in cands)
            used |= set(on)
            code, dr, sh, sg = best
            c_px = mu + sh * ax
            front_px = c_px - dr * ax * self.strip_len_px / 2         # LED 0 sits at s = -L/2 (dr = 1)
            cf = np.array(self.fmap.to_floor(*c_px), float)
            ff = np.array(self.fmap.to_floor(*front_px), float)
            th = math.atan2(ff[1] - cf[1], ff[0] - cf[0])
            cx, cy = self.fam.centre_cm
            R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            c = cf - R @ np.array([cx, cy])
            dets.append(dict(kind="full", spots=[S[k] for k in on], idx=on, geom=code, th=th, c=c, sse=negll,
                             lls=None if amb else lls, ll_none=ll_free, ambiguous=amb))
        rest = [S[k] for k in range(n) if k not in used]
        return dets, rest

    def _strip_fit(self, prof):
        p = self.p
        M = self.models
        n = len(prof)
        mm, my = M.mean(1), prof.mean()
        cov = ((M - mm[:, None]) * (prof - my)[None, :]).sum(1)
        var = ((M - mm[:, None]) ** 2).sum(1)
        A = np.maximum(cov / np.maximum(var, 1e-9), 0.0)
        B = my - A * mm
        sse = ((prof[None, :] - (A[:, None] * M + B[:, None])) ** 2).sum(1)
        k = int(np.argmin(sse))
        if A[k] < p.t_top:
            return None
        sig = p.strip_noise + 0.05 * A[k]
        corr = max(1.0, 2.0 * self.mkeys[k][3])      # samples 0.5 px apart are not independent
        ll = -0.5 * sse / (sig * sig) / corr
        lls = {}
        for (code, dr, sh, sg), v in zip(self.mkeys, ll):
            lls[code] = max(lls.get(code, -math.inf), float(v))
        best = self.mkeys[k]
        # free fit: non-negative 8 levels, best over the shift / blur grid
        best_free = math.inf
        for (sh, sg), Bm in self.free_basis.items():
            X = np.hstack([Bm, np.ones((n, 1))])
            sol, *_ = np.linalg.lstsq(X, prof, rcond=None)
            sol[:-1] = np.maximum(sol[:-1], 0)
            r = prof - X @ sol
            best_free = min(best_free, float(r @ r))
        ll_free = -0.5 * best_free / (sig * sig) / corr - p.strip_none_bic * 0.5 * 6 * math.log(n)
        return lls, best, ll_free

    # ------------------------------------------------------------ frame
    BODY = np.array([[4.75, 4.75], [4.75, -4.75], [-4.75, 4.75], [-4.75, -4.75]])   # robot top corners, cm

    def in_view(self, d, margin=2.0):
        """The whole robot top is in the picture (a robot cut by the edge can show part of its pattern
        next to other bright things)."""
        Hh, Ww = self.top.shape
        for x, y in to_world(d["th"], d["c"], self.BODY):
            u, v = self.fmap.to_px(x, y)
            if not (margin <= u <= Ww - 1 - margin and margin <= v <= Hh - 1 - margin):
                return False
        return True

    def frame(self, img):
        S = self.spots(img)
        if self.fam.kind == "l3":
            d, r = self.l3(S)
        else:
            d, r = self.strip(S)
        keep = [x for x in d if self.in_view(x)]
        r = r + [s for x in d if x not in keep for s in x["spots"]]
        return keep, r, S


# ---------------------------------------------------------------- tracker
class Track:
    def __init__(self, tid, t):
        self.id = tid
        self.t0 = t
        self.t = t
        self.x = self.y = 0.0
        self.th = 0.0
        self.vx = self.vy = self.w = 0.0
        self.E = {}                  # variant -> evidence (log), plus "none"
        self.n_full = 0
        self.geom = None             # best variant so far (pose templates for 2-LED sightings)
        self.bits = deque()          # (t, side LED lit 0/1)
        self.name, self.p_name, self.best, self.why = "", 0.0, "", "new"
        self.probs, self.p_none = {}, 1.0
        self.claim_name, self.claim_t = "", -1e9
        self.seen = False
        self.last_fp = None

    def predict(self, t):
        dt = max(0.0, min(t - self.t, 0.5))
        return self.x + self.vx * dt, self.y + self.vy * dt, self.th + self.w * dt


class Tracker:
    """Detector + tracks. step(img, t, frame) -> rows (one per track seen in
    this frame), dicts with the identity and pose (see lights.py's CSV)."""

    def __init__(self, fam, fmap, names, p=None):
        self.fam, self.fmap, self.p = fam, fmap, p or Params()
        self.det = Detector(fam, fmap, self.p)
        self.names = dict(names)                               # name -> code
        self.parsed = {n: (fam.parse(c) if fam.kind == "l3" else (str(fam.parse(c)), "1111"))
                       for n, c in self.names.items()}
        self.seq_on = fam.kind == "l3" and any(q != "1111" for _, q in self.parsed.values())
        self.tracks = []
        self.n = 0
        self.external = []                                     # (name, x_cm, y_cm): other sources' claims now
        self.stats = dict(frames=0, full=0, ambiguous=0, pairs=0, named_rows=0, contested=0)

    # -------------------------------------------------------- identity
    def _seq_ll(self, T):
        """log P(side LED readings | sequence q), phase marginalised, per q; {} if too short."""
        fam, p = self.fam, self.p
        cyc = fam.slot_s * fam.n_slots
        b = list(T.bits)
        if not b or b[-1][0] - b[0][0] < p.seq_cycles * cyc:
            return {}
        out = {}
        la, lb = math.log(1 - p.seq_eps), math.log(p.seq_eps)
        tt = np.array([x[0] for x in b])
        bb = np.array([x[1] for x in b], bool)
        for q in PT.SEQS:
            pat = np.array([c == "1" for c in q])
            lls = []
            for ph in np.arange(0.0, cyc, p.seq_phase_step_s):
                pos = (tt - ph) % cyc
                slot = (pos // fam.slot_s).astype(int) % 4
                r = pos % fam.slot_s
                ok = np.minimum(r, fam.slot_s - r) >= p.seq_guard_s
                agree = pat[slot] == bb
                lls.append(float(np.where(agree, la, lb)[ok].sum()))
            out[q] = logsumexp(lls) - math.log(len(lls))
        return out

    def _posterior(self, T):
        """{name: p}, p_unlisted_or_none, best name."""
        p = self.p
        geoms = self.det.geoms
        E = T.E
        S = self._seq_ll(T) if self.seq_on else {}
        seqs = PT.SEQS if self.seq_on else ("1111",)
        n_codes = len(geoms) * len(seqs)
        lp_code = math.log((1 - p.prior_none) / n_codes)
        listed = {(g, q) for g, q in self.parsed.values()}
        sc = {}
        for n, (g, q) in self.parsed.items():
            sc[n] = lp_code + E.get(g, 0.0) + (S.get(q, 0.0) if S else 0.0)
        unl = [lp_code + E.get(g, 0.0) + (S.get(q, 0.0) if S else 0.0)
               for g in geoms for q in seqs if (g, q) not in listed]
        sc["~none"] = logsumexp(unl + [math.log(p.prior_none) + E.get("none", 0.0)])
        z = logsumexp(sc.values())
        post = {k: math.exp(v - z) for k, v in sc.items()}
        pn = post.pop("~none")
        best = max(post, key=post.get) if post else ""
        return post, pn, best, bool(S) or not self.seq_on

    def _add_evidence(self, T, post_frame):
        p = self.p
        K = len(post_frame)
        keys = set(post_frame) | set(T.E)
        for k in keys:
            q = (1 - p.eps_frame) * post_frame.get(k, 0.0) + p.eps_frame / K
            T.E[k] = p.forget * T.E.get(k, 0.0) + math.log(q)
        m = max(T.E.values())
        for k in T.E:
            T.E[k] = max(T.E[k] - m, -30.0)

    # -------------------------------------------------------- frame step
    def step(self, img, t, frame=-1):
        p, fam = self.p, self.fam
        self.stats["frames"] += 1
        dets, rest, _ = self.det.frame(img)
        # an ambiguous sighting (its spots also make another acceptable pattern) says nothing: neither
        # who it is nor where (the rival reading may be the true one)
        self.stats["ambiguous"] += sum(1 for d in dets if d["ambiguous"])
        dets = [d for d in dets if not d["ambiguous"]]
        for T in self.tracks:
            T.seen = False
            T.last_fp = None
        # full sightings -> tracks (nearest predicted pose first)
        pairs = []
        for di, d in enumerate(dets):
            for T in self.tracks:
                px, py, pth = T.predict(t)
                dt = max(0.0, t - T.t)
                dd = math.hypot(d["c"][0] - px, d["c"][1] - py)
                if dd > p.gate_cm + 0.5 * p.vmax_cms * dt:
                    continue
                if abs(wrap(d["th"] - pth)) > math.radians(p.turn_gate_deg + p.max_turn_dps * dt):
                    continue
                pairs.append((dd, di, T))
        pairs.sort(key=lambda x: x[0])
        used_d, used_t, assign = set(), set(), {}
        for dd, di, T in pairs:
            if di in used_d or id(T) in used_t:
                continue
            used_d.add(di)
            used_t.add(id(T))
            assign[di] = T
        rows = []
        placed = [dets[j]["c"] for j in assign]
        order = list(assign) + sorted((j for j in range(len(dets)) if j not in assign), key=lambda j: dets[j]["sse"])
        for di in order:
            d = dets[di]
            T = assign.get(di)
            if T is None:
                # a robot can't sit on top of another one: a sighting overlapping a robot seen in this
                # frame (or one whose prediction it contradicts) is spots of two robots, not a third
                if any(math.hypot(d["c"][0] - o.predict(t)[0], d["c"][1] - o.predict(t)[1]) < p.min_sep_cm
                       for o in self.tracks if t - o.t <= p.track_gap_s):
                    self.stats["rejected"] = self.stats.get("rejected", 0) + 1
                    continue
                if any(math.hypot(d["c"][0] - c[0], d["c"][1] - c[1]) < p.min_sep_cm for c in placed):
                    self.stats["rejected"] = self.stats.get("rejected", 0) + 1
                    continue
                placed.append(d["c"])
                self.n += 1
                T = Track(self.n, t)
                self.tracks.append(T)
            self._update_pose(T, t, d["th"], d["c"])
            T.seen = True
            self.stats["full"] += 1
            if d["lls"] is not None:
                if fam.kind == "l3":
                    fp = self.det.frame_post(d["lls"])
                else:
                    ll = dict(d["lls"])
                    ll["none"] = d["ll_none"]
                    m = max(ll.values())
                    z = sum(math.exp(v - m) for v in ll.values())
                    fp = {k: math.exp(v - m) / z for k, v in ll.items()}
                self._add_evidence(T, fp)
                T.last_fp = fp
                T.n_full += 1
                T.geom = max((g for g in fp if g != "none"), key=lambda g: T.E.get(g, -1e9))
            if fam.kind == "l3" and self.seq_on:
                T.bits.append((t, 1))
            rows.append((T, d))
        # tracks without a full sighting: two of their three LEDs (l3)
        if fam.kind == "l3":
            rows += self._partial(t, rest)
        # tracks near each other could swap: forget identity evidence
        live = [T for T in self.tracks if T.seen]
        for i, A in enumerate(live):
            for Bt in live[i + 1:]:
                if math.hypot(A.x - Bt.x, A.y - Bt.y) < p.swap_cm:
                    for T in (A, Bt):          # keep only this frame's own evidence
                        T.E, T.n_full = {}, 0
                        T.bits.clear()
                        if T.last_fp is not None:
                            self._add_evidence(T, T.last_fp)
                            T.n_full = 1
        cyc = getattr(fam, "slot_s", 0.0) * 4
        for T in self.tracks:
            while T.bits and t - T.bits[0][0] > 2.5 * cyc:
                T.bits.popleft()
        self.tracks = [T for T in self.tracks if t - T.t <= p.track_gap_s]
        out = []
        for T, d in rows:
            self._identity(T, t)
        for T, d in rows:
            out.append(self._row(T, d, t, frame))
        return out

    def _update_pose(self, T, t, th, c):
        dt = t - T.t
        if 1e-6 < dt < 0.5:
            k = 0.5
            T.vx = (1 - k) * T.vx + k * (float(c[0]) - T.x) / dt
            T.vy = (1 - k) * T.vy + k * (float(c[1]) - T.y) / dt
            T.w = (1 - k) * T.w + k * wrap(th - T.th) / dt
        elif dt >= 0.5:
            T.vx = T.vy = T.w = 0.0
        T.x, T.y, T.th = float(c[0]), float(c[1]), th
        T.t = t

    def _partial(self, t, rest):
        """l3 tracks not seen in full: their LEDs where the pose predicts them; 2 of 3 found -> pose."""
        p, fam = self.p, self.fam
        rows = []
        claims = {}
        for T in self.tracks:
            if T.seen or T.geom is None or not rest:
                continue
            px, py, pth = T.predict(t)
            dt = max(0.0, t - T.t)
            gate = 1.0 + 0.5 * p.vmax_cms * dt
            tmpl = fam.leds(T.geom)
            pred = to_world(pth, np.array([px, py]), tmpl)
            got = {}
            for si, s in enumerate(rest):
                dd = np.hypot(*(pred - s["xy"]).T)
                k = int(np.argmin(dd))
                if dd[k] <= gate and (k not in got or dd[k] < got[k][0]):
                    got[k] = (dd[k], si)
            if len(got) < 2 or len(got) == 3:
                continue
            ks = sorted(got)
            O = np.array([rest[got[k][1]]["xy"] for k in ks])
            dl = math.hypot(*(O[1] - O[0]))
            dt_ = math.hypot(*(tmpl[ks[1]] - tmpl[ks[0]]))
            if abs(dl - dt_) > 4 * self.det.sigma_of(rest[got[ks[0]][1]]) + 0.15:
                continue
            key = tuple(sorted(got[k][1] for k in ks))
            claims.setdefault(key, []).append((T, ks, O))
        for key, who in claims.items():
            if len(who) != 1:
                continue                   # two tracks want the same spots: neither
            T, ks, O = who[0]
            th, c, _ = rigid_fit(fam.leds(T.geom)[ks], O)
            if abs(wrap(th - T.predict(t)[2])) > math.radians(p.turn_gate_deg + p.max_turn_dps * max(0.0, t - T.t)):
                continue
            self._update_pose(T, t, th, c)
            T.seen = True
            self.stats["pairs"] += 1
            if self.seq_on and ks == [0, 1]:
                # side LED dark where it should be? (a hidden LED reads as dark too: seq_eps covers it)
                sx, sy = to_world(th, c, fam.leds(T.geom)[2:3])[0]
                u, v = self.fmap.to_px(sx, sy)
                val = self._top_at(u, v)
                if val is not None and val < 0.5 * p.t_top:
                    T.bits.append((t, 0))
            rows.append((T, dict(kind="pair", spots=[], idx=key, geom=T.geom, th=th, c=c, ambiguous=False, lls=None)))
        return rows

    def _top_at(self, u, v, r=1):
        top = self.det.top
        H, W = top.shape
        ui, vi = int(round(u)), int(round(v))
        if not (r <= ui < W - r and r <= vi < H - r):
            return None
        return float(top[vi - r:vi + r + 1, ui - r:ui + r + 1].max())

    def _claim(self, T, t):
        if T.name:
            return T.name
        if T.best and T.probs.get(T.best, 0.0) >= 0.5:
            return T.best
        if T.claim_name and t - T.claim_t < self.p.claim_hold_s:
            return T.claim_name
        return ""

    def _identity(self, T, t):
        p = self.p
        post, pn, best, seq_ok = self._posterior(T)
        T.probs, T.p_none, T.best = post, pn, best
        pb = post.get(best, 0.0)
        why = ""
        if T.n_full < p.min_full:
            why = "no full sighting yet" if T.n_full == 0 else "too few full sightings"
        elif not seq_ok:
            why = "sequence: less than a cycle seen"
        elif pb < p.name_p:
            why = "unclear (p %.2f)" % pb
        T.name = best if not why else ""
        if T.name:
            for o in self.tracks:
                if o is T or t - o.t > p.track_gap_s:
                    continue
                if math.hypot(o.x - T.x, o.y - T.y) <= p.contest_cm:
                    continue
                if self._claim(o, t) == T.name:
                    why = "contested"
                    break
            if not why:
                for n, x, y in self.external:
                    if n == T.name and math.hypot(x - T.x, y - T.y) > p.contest_cm:
                        why = "contested (other source)"
                        break
            if why:
                self.stats["contested"] += 1
                T.name = ""
        T.p_name = pb
        T.why = why or "named"
        if T.name:
            T.claim_name, T.claim_t = T.name, t

    def _row(self, T, d, t, frame):
        fam = self.fam
        g = T.geom or d["geom"]
        off = fam.centre(g) if fam.kind == "l3" else fam.centre()
        cw = to_world(T.th, np.array([T.x, T.y]), np.array([off]))[0]
        u, v = self.fmap.to_px(*cw)
        u2, v2 = self.fmap.to_px(cw[0] + math.cos(T.th), cw[1] + math.sin(T.th))
        hi = math.degrees(math.atan2(-(v2 - v), u2 - u))
        if T.name:
            self.stats["named_rows"] += 1
        return dict(frame=frame, t=t, track=T.id, robot=T.name, best=T.best, score=round(T.p_name, 4),
                    probs={k: round(v, 4) for k, v in T.probs.items()}, p_none=round(T.p_none, 4),
                    x_px=float(u), y_px=float(v), heading_img_deg=hi, heading_floor_deg=math.degrees(T.th),
                    x_cm=T.x, y_cm=T.y, off_fwd_cm=float(off[0]), off_left_cm=float(off[1]),
                    n_leds=len(d["idx"]) if d["kind"] != "pair" else 2, kind=d["kind"], why=T.why,
                    code=self.names.get(T.best, "") if T.best else "", ambiguous=int(bool(d.get("ambiguous"))))
