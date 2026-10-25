"""The robot's own position uncertainty since its last camera fix, LEARNED every session from its own camera
corrections (Nick, 2026-10-25: "the goal is to create a calibration run that establishes this on its own, without
simulators. It's okay if the calibration run takes a long time, accuracy and autonomy are the goals").

The model (e02.py unc_cm; 1 sd per axis, cm) after driving d cm and turning t degrees since the last fix:
    sd = base + per_cm x d + d x rad(deg + deg_per_deg x t)
The runner uses it for the confidence pauses (stop for a camera fix before the sd would pass CONF_PAUSE_CM) and for
KC's weights (the robot's belief against the camera's position at each stop).

What it learns from: every camera fix that replaced a pose the robot drove to on its own (the robot's own pose just
before the fix against the camera's), with the driving and turning since its previous fix, and every KC stop (the
same, inside a run). A correction is the robot's error plus the camera's own: the camera's sd at that spot
(e02_camerr, else CAM_SD_CM) is added in quadrature, so the model is the robot's part only (logs DOE 2026-10-25: with
the camera's noise left in, the base term rose to 0.63 cm; with it taken out, it dropped to about 0).

The fit: maximum likelihood. A correction's length is Rayleigh-distributed with the per-axis sd above (robot and
camera together); a share of outliers (a wrong pick-up, a heading gone 30 deg wrong: 17 % of the real chains) is
allowed as a wide second Rayleigh (OUT_SD_CM), so a few wild corrections do not inflate every term. Each term has a
floor (FLOOR): a lucky session can never make the robot more sure of itself than that. per_cm and deg both grow
with the distance alone, so this data cannot tell them apart: per_cm is held at its starting value and deg is fitted
(the logs DOE held it at 1 % too). A term the session has no evidence for (no long drives, or no drives after
large turns) is held at its starting value instead of being guessed.

Where the numbers come from, per robot (like e02_front.py):
  - before this session has SESSION_MIN corrections: earlier sessions' corrections for that robot
    (logs/unc_learned.json, the newest PRIOR_KEEP), else every robot's together, else DEFAULT (the logs DOE);
  - from SESSION_MIN on: this session's own (refitted as more come in), so a robot whose gyro or wheels changed is
    caught at once;
  - the moment this session's own fit is first used it is compared with the earlier one: a report when the two
    differ beyond chance (likelihood ratio) and by more than SHIFT_FRAC at the reference drive (50 cm after 180 deg).
At the end (save) this session's corrections join the robot's history.

Pure numpy (no scipy: the Pi). Tests: tests/test_e02_unc.py."""
import json
import math
import os

import numpy as np

# Starting values, from the logs DOE (docs/build-reports/e02-oct25-logs-doe.md row 1: 673 corrections from 10
# sessions, refit with the camera's noise taken out): base 0.5 cm (kept: the refit's 0-0.63 depends on the camera
# noise assumed), 1 % per cm (unchanged), 0.5 deg (refit 0.43), 0.014 per degree turned (was 0.005; refit
# 0.0136-0.0140). After 50 cm and 180 deg: 3.4 cm (was 2.2). DOE note: these are only where a robot with no history
# starts; each session's own corrections replace them (this module), so no board DOE is needed for the four numbers.
DEFAULT = (0.5, 0.01, 0.5, 0.014)
NAMES = ("base_cm", "per_cm", "deg", "deg_per_deg")
# Floors (chosen): no session may make the model surer than this. base 0.2 cm (the camera's fix itself is not
# better than its ~0.3 cm repeatability), 0.5 % per cm, 0.2 deg, 0.005 per degree (the old value: the logs never
# showed less). DOE note: a replay of the 10 logged sessions, fitting each from its own corrections with floors at
# half / these / double, scored by how many of the NEXT session's corrections fall inside 2.45 sd (should be 95 %).
FLOOR = (0.2, 0.005, 0.2, 0.005)
CEIL = (5.0, 0.2, 20.0, 0.5)        # the logs DOE's bounds; past these the model says nothing useful
CAM_SD_CM = 0.8             # the camera's own per-axis noise when e02_camerr has no better number (logs DOE: 0.5-1.1,
                            # healthy 0.8; DOE note: e02_camerr's tape-mark rows measure it per spot)
OUT_SD_CM = 15.0            # the outliers' spread (logs DOE: the same wide Rayleigh); not a tuning knob
EPS0 = 0.05                 # outlier share to start the fit from (fitted, 0.1 % - 40 %)
SESSION_MIN = 30            # chosen: corrections before a session's own fit replaces the earlier one. DOE note: the
                            # replay above at 15 / 30 / 60 shows how soon a changed robot is caught against false
                            # "changed" reports
DRIVE_MIN_CM = 25.0         # a correction counts as a long drive from this far (fits per_cm / deg)
TURN_MIN_DEG = 90.0         # ... as a drive after turning from this many degrees, with DRIVE_MIN_CM / 2 driven
TERM_MIN = 6                # chosen: corrections of a kind before the terms they inform are fitted, not held
SHIFT_FRAC = 0.25           # chosen: a change at the reference drive smaller than 25 % is not reported
LR_CRIT = 7.81              # chi-square, 3 terms, 95 %: the session's own fit is reported as different only past this
REF_D, REF_T = 50.0, 180.0  # the reference drive for reports: 50 cm after a half turn
PRIOR_KEEP = 400            # corrections kept per robot between sessions
MAX_DIST_CM = 400.0         # a chain longer than this is not a correction of driving (a missed reset): left out
MAX_CORR_CM = 80.0          # ... nor a correction past this (a different robot picked up)


def sd_of(p, d, t):
    """The model's sd (1 sd per axis, cm). p = (base, per_cm, deg, deg_per_deg); d, t scalars or arrays."""
    base, per_cm, deg, dpd = p
    return base + per_cm * np.asarray(d, float) + np.asarray(d, float) * np.radians(deg + dpd * np.asarray(t, float))


def heading_sd(p, t):
    """The heading's own sd (deg) after turning t degrees (KC's heading weight)."""
    return p[2] + p[3] * abs(t)


def _nll(p, eps, d, t, r, cs):
    s = np.sqrt(sd_of(p, d, t) ** 2 + cs ** 2)
    f1 = r / s ** 2 * np.exp(-r ** 2 / (2 * s ** 2))
    f2 = r / OUT_SD_CM ** 2 * np.exp(-r ** 2 / (2 * OUT_SD_CM ** 2))
    return float(-np.sum(np.log((1.0 - eps) * f1 + eps * f2 + 1e-300)))


def _nelder_mead(f, x0, step=0.5, iters=600, tol=1e-7):
    """A small Nelder-Mead minimiser (no scipy on the Pi)."""
    n = len(x0)
    pts = [np.array(x0, float)]
    for i in range(n):
        x = np.array(x0, float)
        x[i] += step
        pts.append(x)
    vals = [f(x) for x in pts]
    for _ in range(iters):
        order = np.argsort(vals)
        pts = [pts[i] for i in order]
        vals = [vals[i] for i in order]
        if abs(vals[-1] - vals[0]) < tol * (1.0 + abs(vals[0])):
            break
        c = np.mean(pts[:-1], axis=0)
        xr = c + (c - pts[-1])
        fr = f(xr)
        if fr < vals[0]:
            xe = c + 2.0 * (c - pts[-1])
            fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = c + 0.5 * (pts[-1] - c)
            fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                pts = [pts[0] + 0.5 * (x - pts[0]) for x in pts]
                vals = [f(x) for x in pts]
    i = int(np.argmin(vals))
    return pts[i], vals[i]


def fit(obs, start=DEFAULT):
    """Fit the model to corrections obs [(dist_cm, turned_deg, corr_cm, cam_sd_cm)]. Terms without evidence are held
    at `start`; per_cm is always held (it cannot be told from deg). Returns dict(p, eps, nll, free, n)."""
    obs = clean(obs)
    start = tuple(float(v) for v in start)
    if not obs:
        return dict(p=start, eps=EPS0, nll=0.0, free=(), n=0)
    a = np.array(obs, float)
    d, t, r, cs = a[:, 0], a[:, 1], np.maximum(a[:, 2], 0.01), a[:, 3]
    free = ["base_cm"]
    if np.sum(d >= DRIVE_MIN_CM) >= TERM_MIN:
        free.append("deg")
    if np.sum((t >= TURN_MIN_DEG) & (d >= DRIVE_MIN_CM / 2.0)) >= TERM_MIN:
        free.append("deg_per_deg")
    idx = [NAMES.index(k) for k in free]

    def unpack(x):
        p = list(start)
        for j, i in enumerate(idx):
            # floor + a positive amount, never past the ceiling
            p[i] = min(CEIL[i], FLOOR[i] + math.exp(min(30.0, x[j])))
        eps = 0.001 + 0.399 / (1.0 + math.exp(-x[-1]))
        return tuple(p), eps

    def f(x):
        p, eps = unpack(x)
        return _nll(p, eps, d, t, r, cs)
    x0 = [math.log(max(1e-6, max(start[i], FLOOR[i] * 1.01) - FLOOR[i])) for i in idx]
    x0.append(math.log((EPS0 - 0.001) / (0.4 - EPS0)))
    best = None
    for s0 in (0.7, 0.3):                      # two restarts: the likelihood has a flat valley along the outliers
        x, v = _nelder_mead(f, best[0] if best else x0, step=s0)
        if best is None or v < best[1]:
            best = (x, v)
    p, eps = unpack(best[0])
    return dict(p=p, eps=eps, nll=best[1], free=tuple(free), n=len(obs))


def clean(obs):
    """Corrections the fit can use: finite, something driven, inside MAX_DIST_CM / MAX_CORR_CM."""
    out = []
    for o in obs or []:
        try:
            d, t, r, cs = (float(v) for v in o[:4])
        except (TypeError, ValueError):
            continue
        if not all(math.isfinite(v) for v in (d, t, r, cs)):
            continue
        if d <= 0.0 or d > MAX_DIST_CM or r < 0 or r > MAX_CORR_CM:
            continue
        out.append((d, abs(t), r, max(0.0, cs)))
    return out


def coverage(obs, p):
    """Share of corrections within 2.45 x the model's total sd (robot + camera): 95 % if the model is right."""
    obs = clean(obs)
    if not obs:
        return None
    a = np.array(obs, float)
    s = np.sqrt(sd_of(p, a[:, 0], a[:, 1]) ** 2 + a[:, 3] ** 2)
    return float(np.mean(a[:, 2] <= 2.45 * s))


class UncModel:
    """Each robot's uncertainty model: earlier sessions', then this session's own (module docstring)."""

    def __init__(self, path=None, default=DEFAULT):
        self.path = path
        self.default = tuple(default)
        self.prior = {}            # robot -> [[d, t, corr, cam_sd], ...] from earlier sessions
        self.session = {}          # robot -> this session's
        self.kinds = {}            # robot -> this session's kinds ("fix" / "kc"), for the summary
        self.reported = set()
        self._cache = {}
        if path and os.path.exists(path):
            try:
                with open(path) as f:
                    self.prior = {k: [list(map(float, o)) for o in v] for k, v in
                                  (json.load(f).get("robots") or {}).items()}
            except (OSError, ValueError, TypeError):
                self.prior = {}

    # --- where the numbers come from ---------------------------------------------------------------------
    def _prior_fit(self, robot):
        key = ("prior", robot)
        if key not in self._cache:
            own = clean(self.prior.get(robot))
            if len(own) >= SESSION_MIN:
                r = fit(own, self.default)
                r["src"] = "earlier sessions (%d corrections)" % len(own)
            else:
                every = [o for v in self.prior.values() for o in clean(v)]
                if len(every) >= SESSION_MIN:
                    r = fit(every, self.default)
                    r["src"] = "every robot's earlier sessions (%d corrections)" % len(every)
                else:
                    r = dict(p=self.default, eps=EPS0, nll=0.0, free=(), n=0, src="the logs analysis (defaults)")
            self._cache[key] = r
        return self._cache[key]

    def _session_fit(self, robot):
        v = self.session.get(robot) or []
        n = len(clean(v))
        if n < SESSION_MIN:
            return None
        key = ("session", robot, n)
        if key not in self._cache:
            self._cache = {k: c for k, c in self._cache.items() if not (k[0] == "session" and k[1] == robot)}
            r = fit(v, self._prior_fit(robot)["p"])
            r["src"] = "this session (%d corrections)" % n
            self._cache[key] = r
        return self._cache[key]

    def params(self, robot):
        """(base, per_cm, deg, deg_per_deg) for this robot now."""
        return self.model(robot)["p"]

    def model(self, robot):
        """dict(p, src, eps, free, n) in use for this robot."""
        return self._session_fit(robot) or self._prior_fit(robot)

    def sd(self, robot, d, t):
        return float(sd_of(self.params(robot), d, t))

    def heading_sd(self, robot, t):
        return heading_sd(self.params(robot), t)

    # --- learning -----------------------------------------------------------------------------------------
    def add(self, robot, dist_cm, turned_deg, corr_cm, cam_sd_cm=None, kind="fix"):
        """One correction. Returns words when this session's own fit is first used and differs from the earlier
        one (a changed robot), else ''."""
        o = [round(float(dist_cm), 2), round(abs(float(turned_deg)), 1), round(float(corr_cm), 3),
             round(float(CAM_SD_CM if cam_sd_cm is None else cam_sd_cm), 3)]
        if not clean([o]):
            return ""
        self.session.setdefault(robot, []).append(o)
        self.kinds.setdefault(robot, {}).setdefault(kind, 0)
        self.kinds[robot][kind] += 1
        if robot in self.reported or self._session_fit(robot) is None:
            return ""
        self.reported.add(robot)
        return self.compare(robot)

    def compare(self, robot):
        """Words when this session's fit differs from the earlier one beyond chance and by more than SHIFT_FRAC at
        the reference drive; '' otherwise."""
        s, pr = self._session_fit(robot), self._prior_fit(robot)
        if s is None:
            return ""
        obs = clean(self.session.get(robot))
        a = np.array(obs, float)
        d, t, r, cs = a[:, 0], a[:, 1], np.maximum(a[:, 2], 0.01), a[:, 3]
        lr = 2.0 * (_nll(pr["p"], pr.get("eps", EPS0), d, t, r, cs) - s["nll"])
        sd_s = float(sd_of(s["p"], REF_D, REF_T))
        sd_p = float(sd_of(pr["p"], REF_D, REF_T))
        if lr > LR_CRIT and abs(sd_s - sd_p) > SHIFT_FRAC * sd_p:
            return ("%s's own position uncertainty after %.0f cm and a %.0f deg turn is %.1f cm this session, not %.1f "
                    "(%s; %d corrections): this session's model is used" % (
                        robot, REF_D, REF_T, sd_s, sd_p, pr["src"], len(obs)))
        return ""

    def summary(self, robot):
        """One line for the session summary and the state."""
        m = self.model(robot)
        obs = clean(self.session.get(robot))
        cov = coverage(obs, m["p"]) if obs else None
        return dict(robot=robot, src=m["src"], params={k: round(v, 4) for k, v in zip(NAMES, m["p"])},
                    free=list(m.get("free") or ()), outliers=round(m.get("eps", 0.0), 3), corrections=len(obs),
                    kinds=dict(self.kinds.get(robot) or {}),
                    sd_ref_cm=round(float(sd_of(m["p"], REF_D, REF_T)), 2),
                    within_2_45_sd=None if cov is None else round(cov, 3))

    def save(self):
        if not self.path:
            return
        out = {k: [list(o) for o in v] for k, v in self.prior.items()}
        for k, v in self.session.items():
            out[k] = (out.get(k, []) + [list(o) for o in v])[-PRIOR_KEEP:]
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(dict(model="sd = base + per_cm*d + d*rad(deg + deg_per_deg*t); rows [dist_cm, turned_deg, "
                                 "corr_cm, cam_sd_cm]", robots=out), f)
        os.replace(tmp, self.path)
