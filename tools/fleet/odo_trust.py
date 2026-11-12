#!/usr/bin/env python3
"""Each robot's odometry trust (e18, Nick 12 Nov, decision 5: the blind-drive allowance).

Robots sometimes have to drive where the camera cannot see, reach a point and come back. How far they may go depends on
how far THIS robot's own pose (its wheels and gyro, calibrated) can be trusted, which is DISCOVERED from its own record,
never typed in (CLAUDE.md "discover, don't pre-map"; "observe, then trust").

Evidence (continuous, while the camera sees the robot): shadow dead reckoning. Every ANCHOR_EVERY_CM of its own travel a
"shadow" starts at the camera's spot (the mean of its latest fixes) with the robot's own heading, and is driven on by
the robot's own motion alone (wheel speed and yaw rate from RAW when they flow, else its own pose's steps along its
heading): exactly what its pose would do if the camera were lost there. At STRETCHES_CM of travel each shadow is compared
with the camera: one sample (cm driven, degrees turned on the spot and on curves with how far it drove after each
degree, error cm). This is the 29 Oct leave-camera-out measurement, made live for each robot.

Model (each term per robot, from its samples; the measured 29 Oct numbers as the starting guess, PRIOR_N samples' worth):
    R(path) = k_cal x ( base + per_cm x s + lateral ),   lateral = sum over the path of ds x sin(heading doubt so far)
    heading doubt so far = per_spot_deg x (degrees turned on the spot so far) + per_curve_deg x (degrees turned on
                           curves so far)
    base = 2.45 x its camera noise (a 2-D 95 % circle; the fix it starts from)
R is the radius its true position may be from its own pose after a blind leg; it grows with the distance and the turns
since the last camera fix, and tightens or loosens as the record changes.

Check of the model itself: every Nth sample is held out of the fit, and every blind stretch that ends with the camera
seeing the robot again gives the error found against what R predicted. k_cal is the 95th percentile of actual / predicted
over those (never under 1: it only widens), so the model's own coverage is measured and corrected.

Allowance: a planned blind leg is allowed only if at every point R + half the body + the stop distance keeps the edge
margin, R + the other robot's doubt + both bodies + the reflex gap keeps them apart, and the leg ends where the camera
reads (the way back). A robot with a poor record has a large R: its legs are refused or shortened.

Fleet record (e18, the coordinator's replay of all 10 home runs, 12 Nov): blind-stretch drift is about 0.06 cm per cm
for all three robots, the same between runs; one record per robot is not supported by the data. So the model is fitted
to the FLEET's pooled samples (every robot's), updated live in each run, and a robot gets its own record only once its own
samples differ from the rest (a Mann-Whitney test on error / predicted, p < DIFFER_P, both sides at least PRIOR_UNTIL
samples). Curves: the replay found the curve response moves by SESSION for all robots together, so samples with curve
turning count only in the session (run) they were measured in; earlier runs' curve samples are left out of the fit.
MIRLT_TRUST_FLEET=0: each robot's own record alone, every run's curves (the first e18 version).

Changelog
  2026-11-12e18  first version.
  2026-11-12e18b the fleet record and session-only curves (the replay); the prior from the replay's blind stretches.
"""
import json
import math
import os

VERSION = "odo-trust-2026-11-12e18b"
try:
    import safety_group as _SG
    FLEET = _SG.flag("MIRLT_TRUST_FLEET")
except Exception:  # noqa: BLE001
    FLEET = True
import time as _time
RUN = "%.0f" % _time.time()   # this run's id: samples carry it (curve samples count only in their own run)
DIFFER_P = 0.01               # CHOSEN: a robot leaves the fleet record when its own samples differ at p < 0.01 (two-sided).
                              # DOE: no (a stricter or looser test only moves when, not whether)

# measured starting guesses (e18b: the replay of all 10 home runs, 12 Nov, blind stretches of the robots' own filters:
# drift 0.06 cm per cm (Theil-Sen, 95 % 0.044-0.076) + a 2-3 cm floor; the 90th percentile 0.28 cm per cm on 20-100 cm;
# turns on the spot 1.3 deg per 90 deg median, 3.6 the 90th percentile). The model is a 95th percentile, so the prior
# takes the replay's 90th percentile slope (0.28): a cautious starting guess the fleet's own samples replace after
# PRIOR_UNTIL samples. Claude's choice of which replay number; e18 first version: 2.0 + 0.07 (the 29 Oct straight legs)
PRIOR_BASE_CM = 2.3
PRIOR_PER_CM = 0.28
PRIOR_SPOT = 0.04          # deg of heading doubt per deg turned on the spot (3.6 per 90, the replay's 90th percentile)
PRIOR_CURVE = 0.04         # ... per deg turned on a curve (until this session's own curves say: the replay)
PRIOR_N = 5                # CHOSEN: the prior counts as 5 samples per stretch length ...
PRIOR_UNTIL = 20           # ... until the robot has 20 samples of its own (then its own record alone; a 95th percentile
                           # would otherwise keep the prior as a floor for ever). DOE: no (the calibration check guards it)
FIT_Q = 0.95               # the model is fitted to the 95th percentile of the errors (quantile regression); the check
                           # (k_cal) corrects whatever is left. Not a tuning number: the coverage asked for
REFIT_N = 5                # refitted after this many new samples (CPU). Not chosen: any small number
ANCHOR_EVERY_CM = 10.0     # CHOSEN: a new shadow every 10 cm of its own travel. DOE: no (more samples, same quantity)
STRETCHES_CM = (10.0, 20.0, 40.0)   # CHOSEN: samples at these blind lengths (the 29 Oct table's range). DOE: no
MAX_SHADOWS = 6
SAMPLES_KEEP = 150         # the record: its last 150 samples (about 15 min of driving). CHOSEN; DOE: no
HOLDOUT_EVERY = 5          # every 5th sample checks the model instead of fitting it
CAL_KEEP = 60
CAL_TARGET = 0.95          # the model is meant to cover 95 % of what it predicts
K_MIN = 1.0                # the check only ever WIDENS the model (a record that improves loosens it through the fit;
                           # Claude's choice: a shrink below the fit was 2.0 against 2.19 cm in the dark-patch case).
                           # Not a tuning number
K_MIN_N = 20
STRAIGHT_DEG = 10.0        # a sample that turned under this is a straight one (per cm)
SPOT_V = 1.0               # cm/s: turning slower than this is a turn on the spot
CAM_AVG_N = 3              # fixes averaged for a shadow's start (the camera's own noise / sqrt 3)


def _qreg(rows, tau):
    """Quantile regression e = b + a s + cs ws + cc wc (all >= 0) at quantile tau, by linear programming (scipy);
    None when scipy is missing or it fails."""
    try:
        import numpy as np
        from scipy.optimize import linprog
    except Exception:  # noqa: BLE001
        return None
    n = len(rows)
    X = np.array([[1.0, r[0], r[1], r[2]] for r in rows])
    e = np.array([r[3] for r in rows])
    # variables: b a cs cc (>= 0), u+ (n), u- (n)
    c = np.r_[np.zeros(4), tau * np.ones(n), (1 - tau) * np.ones(n)]
    A = np.hstack([X, np.eye(n), -np.eye(n)])
    try:
        res = linprog(c, A_eq=A, b_eq=e, bounds=[(0, None)] * (4 + 2 * n), method="highs")
    except Exception:  # noqa: BLE001
        return None
    if not res.success:
        return None
    return tuple(float(v) for v in res.x[:4])


def _q(vals, q):
    v = sorted(vals)
    if not v:
        return None
    return v[min(len(v) - 1, int(q * len(v)))]


class Trust:
    """One robot's record and model."""

    def __init__(self, name):
        self.name = name
        self.samples = []          # dict(t, s, ts, tc, ws, wc, e, hold)
        self.cal = []              # dict(t, pred, act, k, kind): predicted (raw, without k) and actual error
        self.k_cal = 1.0
        self.cam_sd = None         # its camera noise (cm, measured standing scatter)
        self.n_seen = 0
        # live: shadows and the latest fixes
        self._shadows = []
        self._fixes = []
        self._s = 0.0              # its own travel so far (cm)
        self._last_anchor_s = -1e9
        self._own_prev = None
        self._inc = []             # (t, ds, dth, spot): its own motion steps, the last few seconds
        self._fit = None
        self._fit_at = 0
        self._n_added = 0
        self.fleet = None          # (e18b) {name: Trust}: the fleet whose pooled record it uses (link())
        self.own_record = False    # True once its own samples differ from the fleet's

    # -- evidence --------------------------------------------------------------------------------------------------
    def noise(self, sd_cm):
        """Its camera noise (standing scatter of its sightings, cm)."""
        if sd_cm is not None and sd_cm > 0:
            self.cam_sd = float(sd_cm) if self.cam_sd is None else 0.9 * self.cam_sd + 0.1 * float(sd_cm)
            self._fit = None

    def motion(self, t, ds, dth):
        """Its own motion since the last call: ds cm along its heading (negative backwards), dth deg turned."""
        dt = None if self._own_prev is None else max(1e-3, t - self._own_prev)
        self._own_prev = t
        spot = dt is None or abs(ds) / dt < SPOT_V
        self._s += abs(ds)
        self._inc.append((t, ds, dth, spot))
        if len(self._inc) > 60 and self._inc[0][0] < t - 15.0:
            del self._inc[:20]
        for sh in self._shadows:
            self._drive(sh, t, ds, dth, spot)

    def _drive(self, sh, t, ds, dth, spot):
        """One own-motion step for one shadow: it drives on its own heading (the pose with no camera)."""
        sh["th"] += dth
        sh["x"] += ds * math.cos(math.radians(sh["th"]))
        sh["y"] += ds * math.sin(math.radians(sh["th"]))
        sh["s"] += abs(ds)
        a = abs(dth)
        if spot:
            sh["ts"] += a
            sh["ms"] += a * sh["s"]
        else:
            sh["tc"] += a
            sh["mc"] += a * sh["s"]
        sh["hist"].append((t, sh["x"], sh["y"], sh["s"], sh["ts"], sh["tc"], sh["ms"], sh["mc"]))
        if len(sh["hist"]) > 120:
            del sh["hist"][:20]

    def _disp(self, ta, tb, th_b):
        """Its own motion from ta to tb (dx, dy), its heading at tb being th_b (worked backwards through the steps)."""
        steps = [q for q in self._inc if ta < q[0] <= tb]
        th = th_b
        dx = dy = 0.0
        for q in reversed(steps):
            th0 = th - q[2]                            # heading before this step's turn
            dx += q[1] * math.cos(math.radians(th))
            dy += q[1] * math.sin(math.radians(th))
            th = th0
        return dx, dy

    def dist(self, ta, tb):
        """Its own count of the distance driven between ta and tb (cm), or None when its record does not reach back."""
        if not self._inc or self._inc[0][0] > ta + 0.2:
            return None
        return sum(abs(q[1]) for q in self._inc if ta < q[0] <= tb)

    def own_pose(self, t, x, y, th, prev=None):
        """Fallback when no RAW flows: its own pose's step projected on its heading (a lateral pull from a fused fix is
        left out), and its heading's change."""
        p = getattr(self, "_pp", None)
        self._pp = (t, x, y, th)
        if p is None:
            self._own_prev = t
            return
        dx, dy = x - p[1], y - p[2]
        h = math.radians(p[3])
        ds = dx * math.cos(h) + dy * math.sin(h)
        dth = (th - p[3] + 180.0) % 360.0 - 180.0
        if math.hypot(dx, dy) > 15.0:          # a re-set of its pose (fpose), not motion
            self.reset()
            self._own_prev = t
            return
        self.motion(t, ds, dth)

    def reset(self):
        """Its pose was set from elsewhere (fpose, a click): the shadows (started in its old frame) are dropped."""
        self._shadows = []
        self._last_anchor_s = -1e9
        self._inc = []

    def fix(self, t, x, y, th_own, stood=False):
        """A camera fix (picture time t) the run accepted; th_own: its own heading now (a shadow starts with it)."""
        self._fixes.append((t, float(x), float(y)))
        del self._fixes[:-CAM_AVG_N]
        self.n_seen += 1
        for sh in list(self._shadows):
            st = self._at(sh, t)
            if st is None:
                continue
            for L in STRETCHES_CM:
                if L in sh["done"] or st[3] < L:
                    continue
                sh["done"].add(L)
                e = math.hypot(x - st[1], y - st[2])
                s, ts, tc, ms, mc = st[3], st[4], st[5], st[6], st[7]
                # (deg x cm) of heading doubt carried over the distance after each degree: sum dth_k (s - s_k)
                ws, wc = s * ts - ms, s * tc - mc
                self._sample(t, s, ts, tc, ws, wc, e)
            if len(sh["done"]) == len(STRETCHES_CM) or sh["s"] > STRETCHES_CM[-1] + 5.0:
                self._shadows.remove(sh)
        if self._s - self._last_anchor_s >= ANCHOR_EVERY_CM and len(self._fixes) >= CAM_AVG_N and \
                len(self._shadows) < MAX_SHADOWS and th_own is not None:
            # its spot at t: the last fixes each moved on to t by its own motion since (a moving robot's fixes are
            # spread along its track), averaged
            th_now = float(th_own)
            pts = []
            for q in self._fixes:
                dx, dy = self._disp(q[0], t, th_now)
                pts.append((q[1] + dx, q[2] + dy))
            mx = sum(q[0] for q in pts) / len(pts)
            my = sum(q[1] for q in pts) / len(pts)
            self._last_anchor_s = self._s
            sh = dict(x=mx, y=my, th=th_now, s=0.0, ts=0.0, tc=0.0, ms=0.0, mc=0.0, done=set(),
                      hist=[(t, mx, my, 0.0, 0.0, 0.0, 0.0, 0.0)])
            # its own motion that came in after the picture (the fix is ~0.3-1 s old by now)
            for q in [q for q in self._inc if q[0] > t]:
                self._drive(sh, q[0], q[1], q[2], q[3])
            self._shadows.append(sh)

    @staticmethod
    def _at(sh, t):
        h = sh["hist"]
        if not h or t < h[0][0] - 0.05:
            return None
        best = h[-1]
        for q in reversed(h):
            if q[0] <= t:
                best = q
                break
        return best

    def _sample(self, t, s, ts, tc, ws, wc, e):
        n = self._n_added
        self._n_added += 1
        hold = (n % HOLDOUT_EVERY) == HOLDOUT_EVERY - 1
        rec = dict(t=round(t, 2), s=round(s, 2), ts=round(ts, 1), tc=round(tc, 1), ws=round(ws, 1), wc=round(wc, 1),
                   e=round(e, 2), hold=int(hold), run=RUN)
        if hold:
            pred = self.radius_raw(s, ts, tc, ws, wc)
            self.check(pred * self.k_cal, e, kind="held-out sample", t=t, raw=pred)
        self.samples.append(rec)
        del self.samples[:-SAMPLES_KEEP]

    def add_sample(self, s, e, ts=0.0, tc=0.0, ws=0.0, wc=0.0, t=0.0, hold=False, run=None):
        """A sample from elsewhere (tests, a replay of logs)."""
        self.samples.append(dict(t=t, s=s, ts=ts, tc=tc, ws=ws, wc=wc, e=e, hold=int(hold), run=run or RUN))
        del self.samples[:-SAMPLES_KEEP]
        self._n_added += 1
        self._fit = None

    # -- the model -------------------------------------------------------------------------------------------------
    def fit(self):
        """(base, per_cm, per_spot, per_curve, n): the 95th-percentile error as a function of the distance and the turns
        (quantile regression on its samples, every coefficient at least 0, the base at least its camera noise), with
        PRIOR_N prior samples per stretch length among them. per_spot / per_curve are deg of heading doubt per deg
        turned. Refitted after REFIT_N new samples."""
        key = self._fleet_n()
        if self._fit is not None and key - self._fit_at < REFIT_N:
            return self._fit
        floor = max(1.0, 2.45 * self.cam_sd) if self.cam_sd is not None else 1.0
        self.own_record = self.differs()
        fs = self._rows_of([self] if self.own_record or not self._pooled() else
                           [t for t in self.fleet.values() if t.name not in _apart(self.fleet)])
        rows = [(q["s"], q["ws"], q["wc"], q["e"]) for q in fs]
        n_prior = PRIOR_N if len(fs) < PRIOR_UNTIL else 0
        # (e18b) each turn term keeps its prior rows until there are PRIOR_UNTIL samples WITH that kind of turning (a
        # record of straight legs alone said nothing about turns, and the fit then set the turn terms to 0)
        spot_prior = sum(1 for q in fs if q["ws"] > 0) < PRIOR_UNTIL
        curve_prior = sum(1 for q in fs if q["wc"] > 0) < PRIOR_UNTIL
        for L in STRETCHES_CM:                       # the prior, as samples (straight, and one turn at the start)
            for _ in range(n_prior):
                rows.append((L, 0.0, 0.0, PRIOR_BASE_CM + PRIOR_PER_CM * L))
            if spot_prior and n_prior:
                rows.append((L, 90.0 * L, 0.0, PRIOR_BASE_CM + PRIOR_PER_CM * L + 90.0 * L * PRIOR_SPOT * math.pi / 180))
            if curve_prior and n_prior:
                rows.append((L, 0.0, 90.0 * L, PRIOR_BASE_CM + PRIOR_PER_CM * L + 90.0 * L * PRIOR_CURVE * math.pi / 180))
        sol = _qreg(rows, FIT_Q)
        if sol is None:
            base, per, cs, cc = PRIOR_BASE_CM, PRIOR_PER_CM, PRIOR_SPOT * math.pi / 180, PRIOR_CURVE * math.pi / 180
        else:
            base, per, cs, cc = sol
        base = max(floor, base)
        if spot_prior:          # (e18b) no turning record of its own yet: at least the prior's per-degree doubt
            cs = max(cs, PRIOR_SPOT * math.pi / 180)
        if curve_prior:
            cc = max(cc, PRIOR_CURVE * math.pi / 180)
        self._fit = (base, min(1.0, per), min(1.0, cs * 180 / math.pi), min(1.0, cc * 180 / math.pi), len(fs))
        self._fit_at = key
        return self._fit

    # -- the fleet record (e18b) -----------------------------------------------------------------------------------------
    def _pooled(self):
        return FLEET and self.fleet is not None and len(self.fleet) > 1

    def _fleet_n(self):
        return sum(t._n_added for t in self.fleet.values()) if self._pooled() else self._n_added

    @staticmethod
    def _rows_of(trusts):
        """Fitting samples of these robots: not held out, over 1 cm; a sample with curve turning only from this run (the
        curve response is a session property: the replay). MIRLT_TRUST_FLEET=0: every run's curves (first e18)."""
        return [q for t in trusts for q in t.samples if not q["hold"] and q["s"] > 1.0 and
                (not FLEET or q.get("wc", 0.0) <= 0.0 or q.get("run", RUN) == RUN)]

    def differs(self):
        """True when its own samples differ from the rest of the fleet's (_apart): then it keeps its own record. Not
        pooled: True (its own record, as the first e18)."""
        if not self._pooled():
            return True
        return self.name in _apart(self.fleet)

    def radius_raw(self, s, ts=0.0, tc=0.0, ws=0.0, wc=0.0):
        base, per, ps, pc, _ = self.fit()
        lat = min(s, (ps * ws + pc * wc) * math.pi / 180.0)     # small angles: the doubt (rad) x the cm after it
        return base + per * s + lat

    def radius(self, s, ts=0.0, tc=0.0, ws=0.0, wc=0.0, base=None):
        """The predicted 95 % error radius (cm) after s cm blind with these turns (ws, wc: deg x cm, see fix())."""
        r = self.radius_raw(s, ts, tc, ws, wc)
        if base is not None:
            r += max(0.0, float(base) - self.fit()[0])
        return self.k_cal * r

    def path_radii(self, pts, base=None):
        """R at every point of a planned path [(x, y)] (a turn on the spot: two points at one place with headings
        (x, y, th)). Returns [(s, R)]."""
        s = ts = tc = ms = mc = 0.0
        out = [(0.0, self.radius(0.0, base=base))]
        prev_h = None
        for a, b in zip(pts, pts[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            d = math.hypot(dx, dy)
            h = math.degrees(math.atan2(dy, dx)) if d > 1e-6 else (b[2] if len(b) > 2 else prev_h)
            if len(b) > 2 and b[2] is not None and d <= 1e-6:
                h = b[2]
            if prev_h is not None and h is not None:
                dth = abs((h - prev_h + 180.0) % 360.0 - 180.0)
                if d <= 1e-6 or dth > 30.0:
                    ts += dth
                    ms += dth * s
                else:
                    tc += dth
                    mc += dth * s
            s += d
            prev_h = h if h is not None else prev_h
            out.append((s, self.radius(s, ts, tc, s * ts - ms, s * tc - mc, base=base)))
        return out

    # -- the model's own check -----------------------------------------------------------------------------------------
    def check(self, pred, act, kind="seen again", t=None, raw=None):
        """Predicted (with the k_cal in force) against the error found; k_cal re-set from all checks."""
        raw = (pred / self.k_cal) if raw is None else raw
        self.cal.append(dict(t=t, pred=round(pred, 2), raw=round(raw, 3), act=round(act, 2), k=round(self.k_cal, 3),
                             kind=kind, within=int(act <= pred)))
        del self.cal[:-CAL_KEEP]
        ratios = [q["act"] / max(1e-6, q["raw"]) for q in self.cal]
        if len(ratios) < K_MIN_N:
            self.k_cal = max(1.0, _q(ratios, CAL_TARGET) if len(ratios) >= 5 else max(ratios))
        else:
            self.k_cal = max(K_MIN, _q(ratios, CAL_TARGET))
        return self.cal[-1]

    def coverage(self):
        """(share of checks inside their prediction, n)."""
        if not self.cal:
            return None, 0
        return sum(q["within"] for q in self.cal) / len(self.cal), len(self.cal)

    def words(self):
        base, per, ps, pc, n = self.fit()
        who = "" if not self._pooled() else (" its own record (differs from the fleet's)" if self.own_record else
                                             " the fleet's record")
        cov, nc = self.coverage()
        return ("%s: %.2f cm per cm, %.3f deg per deg turned on the spot, %.3f on curves, base %.1f cm (%d samples%s); "
                "model x%.2f%s" % (self.name + who, per, ps, pc, base, n, "" if n >= 20 else ", mostly the prior",
                                   self.k_cal, "" if cov is None else ", %d of %d checks inside" % (
                                       round(cov * nc), nc)))

    def summary(self):
        base, per, ps, pc, n = self.fit()
        cov, nc = self.coverage()
        return dict(base_cm=round(base, 2), per_cm=round(per, 4), per_spot=round(ps, 4), per_curve=round(pc, 4),
                    samples=n, record="own" if self.own_record or not self._pooled() else "fleet", k_cal=round(self.k_cal, 3), checks=nc, coverage=None if cov is None else round(cov, 3),
                    r40=round(self.radius(40.0), 2))

    # -- the allowance -----------------------------------------------------------------------------------------------
    def allow(self, pts, edge_fn, seen_fn=None, others=(), half=6.72, stop_cm=2.4, gap=2.5, base=None):
        """A planned blind leg pts [(x, y[, th])] from where the robot is now. edge_fn(x, y): room to the nearest edge
        or unknown ground (cm); seen_fn(x, y): True where the camera reads (the leg must end there: the way back);
        others: [(x, y, doubt_cm)] the other robots. Returns dict(ok, why, r_max, blind_cm, worst_cm, first_bad)."""
        # the error grows from where the leg first leaves the camera's ground (seen ground before that: the camera keeps
        # its fix, its error stays the base); once blind it keeps growing to the end of the leg (no fix is counted on)
        k0 = next((k for k, p in enumerate(pts) if seen_fn is not None and not seen_fn(p[0], p[1])), None)
        if seen_fn is None:
            k0 = 0
        if k0 is None:
            k0 = len(pts) - 1
        k0 = max(0, k0 - 1)
        rad0 = self.path_radii(pts[k0:], base=base)
        r_base = self.radius(0.0, base=base)
        rad = [(0.0, r_base)] * k0 + [(s + 0.0, R) for s, R in rad0]
        worst, why, bad = float("inf"), "", None
        blind = 0.0
        for k, ((s, R), p) in enumerate(zip(rad, pts)):
            if k and (seen_fn is None or not seen_fn(p[0], p[1])):
                blind += math.hypot(p[0] - pts[k - 1][0], p[1] - pts[k - 1][1])
            m = edge_fn(p[0], p[1]) - (R + half + stop_cm)
            if m < worst:
                worst = m
            if m < 0 and bad is None:
                bad = k
                why = "at %.0f, %.0f its possible error %.1f cm + body + stop needs %.1f cm, the edge is %.1f cm" % (
                    p[0], p[1], R, R + half + stop_cm, edge_fn(p[0], p[1]))
            for (ox, oy, ou) in others:
                m2 = math.hypot(p[0] - ox, p[1] - oy) - (R + ou + 2 * half + gap)
                if m2 < worst:
                    worst = m2
                if m2 < 0 and bad is None:
                    bad = k
                    why = "at %.0f, %.0f its possible error %.1f cm reaches another robot" % (p[0], p[1], R)
        end_ok = seen_fn is None or seen_fn(pts[-1][0], pts[-1][1])
        if bad is None and not end_ok:
            bad = len(pts) - 1
            why = "the leg does not end where the camera reads (no way back)"
        return dict(ok=bad is None, why=why or "every point keeps its margin", r_max=round(max(R for _, R in rad), 2),
                    blind_cm=round(blind, 1), worst_cm=round(worst, 2), first_bad=bad,
                    r_end=round(rad[-1][1], 2))

    def max_depth(self, x, y, th_out, want_cm, edge_fn, seen_fn=None, others=(), half=6.72, stop_cm=2.4, gap=2.5,
                  step_cm=1.0, base=None):
        """An out-and-back excursion from (x, y) along th_out: out `want_cm`, turn round on the spot, back. The longest
        depth (cm, at most want_cm) this robot's record allows (0: none), with the allowance of that depth."""
        def leg(d):
            c, s_ = math.cos(math.radians(th_out)), math.sin(math.radians(th_out))
            n = max(1, int(d / step_cm))
            out_ = [(x + c * d * k / n, y + s_ * d * k / n, th_out) for k in range(n + 1)]
            back = [(x + c * d * k / n, y + s_ * d * k / n, th_out + 180.0) for k in range(n, -1, -1)]
            return out_ + back
        best, res = 0.0, None
        lo, hi = 0.0, float(want_cm)
        r_ = self.allow(leg(hi), edge_fn, seen_fn, others, half, stop_cm, gap, base)
        if r_["ok"]:
            return hi, r_
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            r_ = self.allow(leg(mid), edge_fn, seen_fn, others, half, stop_cm, gap, base)
            if r_["ok"]:
                best, res, lo = mid, r_, mid
            else:
                hi = mid
        return best, res

    # -- keeping it ---------------------------------------------------------------------------------------------------
    def to_dict(self):
        return dict(name=self.name, version=VERSION, samples=self.samples[-SAMPLES_KEEP:], cal=self.cal[-CAL_KEEP:],
                    k_cal=self.k_cal, cam_sd=self.cam_sd)

    @classmethod
    def from_dict(cls, d):
        t = cls(d.get("name", "?"))
        t.samples = list(d.get("samples") or [])
        t.cal = list(d.get("cal") or [])
        t.k_cal = float(d.get("k_cal") or 1.0)
        t.cam_sd = d.get("cam_sd")
        t._n_added = len(t.samples)
        return t


_APART = {}


def _p_differs(own, rest):
    """Mann-Whitney p (two-sided) of own vs rest errors over what the rest's median model predicts; None: too few."""
    if len(own) < PRIOR_UNTIL or len(rest) < PRIOR_UNTIL:
        return None
    sol = _qreg([(q["s"], q["ws"], q["wc"], q["e"]) for q in rest[-3 * SAMPLES_KEEP:]], 0.5)
    if sol is None:
        return None
    b, a, cs, cc = sol

    def ratio(q):
        return q["e"] / max(0.5, b + a * q["s"] + min(q["s"], cs * q["ws"] + cc * q["wc"]))
    try:
        from scipy.stats import mannwhitneyu
        return float(mannwhitneyu([ratio(q) for q in own], [ratio(q) for q in rest], alternative="two-sided").pvalue)
    except Exception:  # noqa: BLE001
        return None


def _apart(fleet):
    """Names whose samples differ from the rest of the fleet: one at a time, the most different first (p < DIFFER_P
    against the others still in the fleet), until none differs or two are left. Cached until a sample is added."""
    key = (id(fleet), sum(t._n_added for t in fleet.values()), len(fleet))
    hit = _APART.get(id(fleet))
    if hit is not None and hit[0] == key:
        return hit[1]
    apart = set()
    while len(fleet) - len(apart) >= 2:
        inn = [t for t in fleet.values() if t.name not in apart]
        best = None
        for t in inn:
            p = _p_differs(Trust._rows_of([t]), Trust._rows_of([o for o in inn if o is not t]))
            if p is not None and p < DIFFER_P and (best is None or p < best[0]):
                best = (p, t.name)
        if best is None:
            break
        if len(inn) == 2:           # two that differ: neither is "the fleet", each keeps its own record
            apart |= {t.name for t in inn}
            break
        apart.add(best[1])
    _APART[id(fleet)] = (key, apart)
    return apart


def link(trusts):
    """(e18b) Join these robots' records into one fleet record (MIRLT_TRUST_FLEET=0: nothing changes)."""
    for t in trusts.values():
        t.fleet = trusts
        t._fit = None
    return trusts


def store_path(conf_dir=None):
    d = conf_dir or os.path.expanduser(os.environ.get("MIRLT_CONF_DIR") or "~/.config/mirlt")
    return os.path.join(d, "odo_trust.json")


def load(names, path=None):
    """{name: Trust}: each robot's saved record (a new robot: the prior)."""
    out = {n: Trust(n) for n in names}
    p = path or store_path()
    try:
        with open(p) as f:
            d = json.load(f)
        for n in names:
            if n in (d.get("robots") or {}):
                out[n] = Trust.from_dict(d["robots"][n])
    except (OSError, ValueError, TypeError):
        pass
    return out


def save(trusts, path=None, why=""):
    """Every robot's record into the store (atomic; robots not in this run are kept). Returns the path or None."""
    p = path or store_path()
    try:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        try:
            with open(p) as f:
                d = json.load(f)
        except (OSError, ValueError):
            d = {}
        rb = d.setdefault("robots", {})
        for n, t in trusts.items():
            rb[n] = t.to_dict()
        d.update(version=VERSION, why=why)
        tmp = p + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f, indent=1)
        os.replace(tmp, p)
        return p
    except OSError:
        return None
