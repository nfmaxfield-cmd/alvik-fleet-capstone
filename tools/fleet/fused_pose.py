#!/usr/bin/env python3
"""One filter per robot on the Pi (e09, Phase B2/B3, 7 Nov): the proven sensor-fusion Kalman filter
(tools/fusion/ekf.py, the one that beat every single source on the 25 Oct sessions: 5.1 cm after 40 s with no camera,
against 19.2 cm for the robot's own pose) run live, fed by everything the robot and the cameras say.

What feeds it (all from the telemetry the Pi already logs, no firmware change):
  wheels      RAW v_cms / w_dps (tools/raw_units.py: mm/s and 1.08 high), or zero-velocity when both wheels stand
  gyro        RAW gz, the raw rate (deg/s). Its bias is a state of the filter AND is taken afresh at every standstill
              (the robot's gz while both wheels stand still: 0.40-0.76 deg/s on the 5 / 6 Nov robots, measured below)
  accelerometer RAW ax: a jolt while the wheels say "driving straight" is a bump; it widens the position a little
  front sensor RAW dC_cm: a front stop reading is logged as evidence only (no map of the room to range against)
  line sensors RAW lineL/C/R: a drop under the nose widens the position (the robot may be over an edge)
  cameras     the tracking layer's fixes (tools/fleet/tracking_layer.py): roof tags, motion + lock-on, lights; each
              with its own sigma, applied at the PICTURE's time (late fixes moved on by the filter's own motion since)

What comes out: the pose now (x, y, heading) and an uncertainty in cm that the planner and Phase A's edge rule use
instead of the old "camera sigma + staleness x speed + bubble". A disagreement between the camera and the filter
widens the uncertainty AT ONCE (the filter's position covariance is widened by the disagreement before the fix is
used: Phase A's A1, kept). Learned trust: each camera source's noise is scaled by how its fixes actually fit
(median squared Mahalanobis distance of its last fixes against what chi-square says), after MIN_LEARN fixes.

B3, heading from the raw gyro rate (GyroHeading; switch, OFF by default): the heading integrated from gz minus the bias
taken at the latest standstill, instead of the firmware's library yaw x gyro_scale. Measured on the robots' own logs
(e09, tools/analysis/fused_replay.py): the library yaw reads 0.92 of the raw rate over 3 s turning windows (IQR
0.91-0.93: the firmware's gyro_scale 1.075 puts it at about 0.99), the wheels 1.02-1.035 of it (scrub), and the library
yaw does not drift standing (the raw rate's bias is taken out inside the library). It needs no reflash: gz is in RAW.

The old way (the robot's own pose, pulled to each camera fix by its firmware's one-step blend, and the controller's
"camera spot moved on by the own pose") stays behind the switch pose_source="blend" (freeroam_core, run_plan).
"""
import math
import os
import sys
from collections import deque

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS = os.path.dirname(_HERE)
for _p in (os.path.join(_TOOLS, "fusion"), _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np   # noqa: E402

from ekf import EKF, Noise   # noqa: E402

VERSION = "fused-pose-2026-11-12e21"   # e21: a negative covariance from rounding is the floor, not a crash (math domain error in the hard simulator, harsh seed 3). Before: "fused-pose-2026-11-07d"   # d (e10, the coordinator 7 Nov): the uncertainty learns its own shortfall at the end of every camera gap (UNC_LEARN)
VERSION_C = "fused-pose-2026-11-07c"   # c (e10): late RAW rows used, 95 % uncertainty (K 8)
D2R = math.pi / 180.0

POSE_SOURCE = os.environ.get("MIRLT_POSE_SOURCE", "blend")    # "ekf": the filter in the loop (e09) | "blend": the one-step
                                                               # blend as before, the filter running beside it (shadow,
                                                               # logged). Default blend after e09's rehearsal e: with the
                                                               # filter in the loop a hung robot stood after its recovery
                                                               # and two robots touched once (see the e09 report)
GYRO_HEADING = os.environ.get("MIRLT_GYRO_HEADING", "0") == "1"   # B3: OFF by default (Nick, 7 Nov)

# ---------------------------------------------------------------------------------------------- numbers, and whence
K_SIGMA = 8.0           # MEASURED for a CHOSEN 95 % coverage (Nick, 7 Nov: 95 %, not e09's 90 %): the uncertainty handed
                        # to the edge rule = K_SIGMA x the filter's 1-sigma position error. On the 5 Nov run's first
                        # 90 min (1,377 camera gaps cut out, tools/analysis/fused_replay.py) 8 x sigma held the real
                        # error 96.1 / 95.6 / 95.0 % of the time for gaps of 10 / 20 / 40 s (6 x: 87-91 %); 5 s gaps
                        # 87.9 % (there the camera's own ~1 cm noise, the reference, dominates). DOE: the coverage target
K_TRACK = 2.45          # (e10) the 2-D 95 % factor (chi-square, 2 dof) for the camera-limited part: not chosen
SPLIT_UNC = os.environ.get("MIRLT_UNC_SPLIT", "1") != "0"   # (e10) 0: e09's single K_SIGMA x sigma
LATE_RAW_S = 0.5        # (e10) CHOSEN: a RAW row this late is still applied now (WiFi 5-40 ms + the picture's ~0.15 s)
UNC_LEARN = os.environ.get("MIRLT_UNC_LEARN", "1") != "0"   # (e10, 7 Nov) the uncertainty learns what it missed: at
                        # the first fix after every camera gap of UNC_LEARN_GAP_S or more, how far the fix fell OUTSIDE
                        # the uncertainty the filter gave just before it; the 95th percentile of the last UNC_LEARN_KEEP
                        # such misses (never below 0) is added to the uncertainty from then on. 0: off (the switch)
UNC_LEARN_GAP_S = 0.5   # CHOSEN: a gap long enough that the filter ran on its own (12 pictures a second otherwise)
UNC_LEARN_Q = float(os.environ.get("MIRLT_UNC_LEARN_Q", "99"))     # CHOSEN: the percentile kept (over 95: the
                        # misses are learned from the past, the next gap may be worse); DOE with the coverage replays
UNC_LEARN_KEEP = 60     # CHOSEN: about 10-30 min of gaps at home
UNC_LEARN_MIN = 5       # CHOSEN: misses needed before it is used
UNC_FLOOR_CM = 1.0      # never below the old camera sigma (PLAN_M0 / safety_rules.CAM_SIGMA_CM): not chosen anew
GYRO_REL = 0.014        # MEASURED: heading error per degree turned (logs DOE, 25 Oct: 0.014, not the chosen 0.005)
GYRO_BASE_DPS = 0.3     # the filter's gyro noise (fusion default; the raw rate's standing sd 0.02-0.04 deg/s here)
WHEEL_SCALE_SD = 0.03   # CHOSEN prior on the wheel distance scale (the 5 Nov 4 h run: a 4 % wheel-vs-camera gap)
TURN_SCALE_SD = 0.04    # CHOSEN prior on the wheels' turn scale (measured: wheels turn 1.02-1.035 x the gyro)
STILL_RPM = 0.5         # both wheels under this: standing (as gz_turns: the RAW wl/wr_rpm columns)
STILL_BIAS_S = 0.5      # a standstill at least this long gives a fresh gyro bias (the mean gz over it)
HIST_S = 4.0            # filter states kept for late fixes (the home camera's delay 0.44 s, 1.3 s at its worst)
JUMP_CM = 40.0          # CHOSEN: a fix this far from the filter is another place, not a correction (the controller's
                        # REACQUIRE rules then decide); DOE: no, a guard (the real runs' worst true disagreement: 36 cm)
GATE_D2 = 9.21          # a fix this far (squared Mahalanobis, 2 dof, 99 %) from the filter: a DISAGREEMENT. It widens
                        # the position covariance by the innovation at once, then is used (evidence, not dropped)
MIN_LEARN = 30          # fixes of one source before its learned trust is used (e02_unc's 30 corrections)
LEARN_KEEP = 200        # the last this many fixes of a source make its trust
TRUST_MIN, TRUST_MAX = 0.5, 9.0    # a source's noise may be learned this much smaller / larger (variance factor)
CHI2_2_MEDIAN = 1.386   # median of chi-square with 2 dof
JOLT_G = 0.35           # CHOSEN: |ax| over this while driving straight = a bump (the e02 jolt cue's order); DOE: yes
JOLT_WIDEN_CM = 1.0     # CHOSEN: a bump widens the position sd by this (the robot may have slipped)
LINE_DROP = 200         # CHOSEN: a line sensor over this reads "no floor" (the 6 Nov edge rows: 40-60 on the board,
                        # 900+ over the drop); DOE: no, a reading of the board
SRC_SIGMA = {"tags": 0.6, "motion": 1.0, "lock": 1.2, "lights": 1.5, "ml": 1.5, "camera": 1.0}
                        # CHOSEN starting sigmas (cm) per camera source, before the learned trust takes over:
                        # tags = site_fit's 0.5 + roof height; motion = cam_bridge FIX_SIGMA_CM; lock: the lock-on's
                        # 7 cm worst against fixes (22-23 Oct) is a max, not a sigma; lights: their 1 cm offset + spread


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class GyroHeading:
    """B3: heading from the raw gyro rate, the bias taken at every standstill. feed(t, gz_dps, still) each RAW row;
    heading_deg() the integrated heading (anchored by set()). The sign of gz against the robot's left-positive heading
    is learned from the wheels' own turn rate (observe, then trust): until known (SIGN_MIN_DEG of agreeing turn), None."""
    SIGN_MIN_DEG = 30.0

    def __init__(self, bias0=None, scale=1.0):
        self.scale = float(scale)    # the raw rate's own scale (1.0: as read; the replay tries others)
        self.th = None
        self.t = None
        self.bias = bias0
        self.n_bias = 0
        self.still_sum = 0.0
        self.still_n = 0
        self.still_t0 = None
        self.sign = None
        self._agree = 0.0
        self._turned = 0.0

    def set(self, th_deg):
        self.th = float(th_deg)

    def feed(self, t, gz_dps, still, wheel_dps=None):
        if self.t is not None and 0.0 < t - self.t < 0.5:
            dt = t - self.t
            if still:
                if self.still_t0 is None:
                    self.still_t0 = t
                self.still_sum += gz_dps
                self.still_n += 1
            else:
                if self.still_t0 is not None and t - self.still_t0 >= STILL_BIAS_S and self.still_n >= 3:
                    self.bias = self.still_sum / self.still_n
                    self.n_bias += 1
                self.still_t0, self.still_sum, self.still_n = None, 0.0, 0
                if wheel_dps is not None and abs(wheel_dps) > 10.0 and self.sign is None:
                    g = gz_dps - (self.bias or 0.0)
                    self._agree += (1.0 if g * wheel_dps > 0 else -1.0) * abs(wheel_dps) * dt
                    if abs(self._agree) >= self.SIGN_MIN_DEG:
                        self.sign = 1.0 if self._agree > 0 else -1.0
            if self.th is not None and self.sign is not None and not still:
                self.th = wrap180(self.th + self.sign * self.scale * (gz_dps - (self.bias or 0.0)) * dt)
        self.t = t

    def heading_deg(self):
        return self.th if (self.sign is not None and self.bias is not None) else None


class FusedPose:
    """The filter for one robot. Times in seconds on the Pi's clock; positions cm; headings degrees (left-positive,
    0 = +x), as everywhere in tools/fleet."""

    def __init__(self, name, gyro_heading=None, learn=True, k_sigma=K_SIGMA, wheel_scale=True):
        self.name = name
        self.noise = Noise(g_rel=GYRO_REL, g_base=GYRO_BASE_DPS * D2R)
        self.scale = dict(k=1.0, sk=WHEEL_SCALE_SD, kw=1.0, skw=TURN_SCALE_SD, qk=0.0005, qkw=0.0005) \
            if wheel_scale else None
        self.k = None                    # the EKF once started
        self.hist = deque()              # (t, x, y, th_rad, Pxx, Pyy)
        self.gyro_heading = GYRO_HEADING if gyro_heading is None else bool(gyro_heading)
        self.gh = GyroHeading()
        self.learn = learn
        self.k_sigma = k_sigma
        self.split_unc = SPLIT_UNC   # (e10) False: e09's single factor (the switch)
        self.p_fix = None
        self.src_d2 = {}                 # source -> deque of d2 (agreeing fixes)
        self.trust = {}                  # source -> variance factor learned
        self.n_fix = {}
        self.n_disagree = 0
        self.last_disagree_cm = 0.0
        self.disagree_t = None
        self.last_fix_t = None
        self.unc_miss = deque(maxlen=UNC_LEARN_KEEP)    # (e10) at each gap's end: the fix's distance less the uncertainty
        self.unc_add = 0.0                              # the learned addition (cm)
        self.unc_learn = UNC_LEARN
        self.last_raw_t = None
        self.gsign = None                # the raw gyro's sign (learned from the wheels, as GyroHeading)
        self._gagree = 0.0
        self.bias_still = None           # (t0, sum, n) while standing
        self.n_bumps = 0
        self.n_drops = 0
        self.use_own_turn = False       # the own heading's turn only while there is no raw gyro (replays: 6 Nov,
                                        # with it on as well the 20 s gaps went from 2.65 to 4.88 cm)
        self.evidence = deque(maxlen=50)    # (t, kind, value): front stops, bumps, drops (logged by the caller)

    # -- start / reset --------------------------------------------------------------------------------------------
    @property
    def started(self):
        return self.k is not None

    def start(self, t, x, y, th_deg, sxy=1.0, sth_deg=10.0):
        self.k = EKF(float(x), float(y), float(th_deg) * D2R, float(t), self.noise, sigma_xy=sxy,
                     sigma_th=sth_deg * D2R, scale=self.scale)
        if self.gh.bias is not None:
            self.k.s[5] = self.gh.bias * D2R * (self.gsign or 1.0)
        self.gh.set(th_deg)
        self.hist.clear()
        self.p_fix = None
        self._keep()

    def reset(self, t, x, y, th_deg=None, sxy=1.0, sth_deg=None):
        if self.k is None:
            self.start(t, x, y, th_deg if th_deg is not None else 0.0, sxy, sth_deg or 20.0)
            return
        self.k.predict(max(self.k.t, t))
        self.k.reset_position(float(x), float(y), sxy, None if th_deg is None else th_deg * D2R,
                              None if sth_deg is None else sth_deg * D2R)
        if th_deg is not None:
            self.gh.set(th_deg)
        self.hist.clear()
        self.p_fix = None
        self._keep()

    SCALE_LIM = (0.85, 1.15)   # CHOSEN: the learned wheel scales are held inside this (real: 0.98-1.08 on 5 Nov);
                               # a late or skewed camera could otherwise teach them nonsense (the fast simulator's
                               # 1 s late, 0.7 s ahead camera did, before this)

    def _clamp(self):
        k = self.k
        if k is not None and self.scale:
            for i in (6, 7):
                if not (self.SCALE_LIM[0] <= k.s[i] <= self.SCALE_LIM[1]):
                    k.s[i] = min(self.SCALE_LIM[1], max(self.SCALE_LIM[0], k.s[i]))
                    self.n_clamped = getattr(self, "n_clamped", 0) + 1

    def _keep(self):
        self._clamp()
        k = self.k
        # the filter's own MOTION since the last row (its speed along its heading), never its corrections: a late fix
        # is moved on to now by this, so a correction can never feed back into the next fix
        if self.hist:
            t0 = self.hist[-1][0]
            dt = max(0.0, k.t - t0)
            u = k.s[3] / (k.s[6] if self.scale else 1.0)
            self.odo = (self.odo[0] + u * math.cos(k.s[2]) * dt, self.odo[1] + u * math.sin(k.s[2]) * dt)
        else:
            self.odo = (0.0, 0.0)
        self.hist.append((k.t, self.odo[0], self.odo[1], k.s[2], k.P[0, 0], k.P[1, 1]))
        while self.hist and self.hist[0][0] < k.t - HIST_S:
            self.hist.popleft()

    # -- the robot's own sensors ----------------------------------------------------------------------------------
    def raw(self, t, v_cms, w_dps, gz_dps=None, still=None, ax=None, front_cm=None, line=None):
        """One RAW row in TRUE units (callers convert with raw_units.speed_cms / turn_dps). still: both wheels standing
        (None: from v and w)."""
        if still is None:
            still = abs(v_cms) < 0.2 and abs(w_dps) < 1.0
        if gz_dps is not None:
            self.gh.feed(t, gz_dps, still, w_dps)
            if self.gsign is None and not still and abs(w_dps) > 10.0 and self.gh.bias is not None:
                g = gz_dps - self.gh.bias
                dt = 0.0 if self.last_raw_t is None else min(0.2, max(0.0, t - self.last_raw_t))
                self._gagree += (1.0 if g * w_dps > 0 else -1.0) * abs(w_dps) * dt
                if abs(self._gagree) >= GyroHeading.SIGN_MIN_DEG:
                    self.gsign = 1.0 if self._gagree > 0 else -1.0
        self.last_raw_t = t
        k = self.k
        if k is None:
            return
        if t <= k.t - 1e-6:
            # (e10, C4: the hung-robot case) a LATE row: the camera's fixes move the filter's clock to "now", so a RAW
            # row stamped a few tens of ms earlier arrived "in the past" and was thrown away, the standing robot's
            # zero-speed readings with it: the speed's doubt then grew without bound and a standing robot near an edge
            # read 5-20 cm of position doubt, so no move fitted (e09 rehearsal e(b), the filter in charge). Its
            # readings still hold now: applied at the filter's time, without a predict; too old ones are dropped
            if k.t - t > LATE_RAW_S:
                self.n_late_dropped = getattr(self, "n_late_dropped", 0) + 1
                return
            self.n_late_used = getattr(self, "n_late_used", 0) + 1
        else:
            k.predict(t)
        if still:
            k.zupt()
            if gz_dps is not None and self.gsign is not None:
                # the bias at a standstill: the gyro reads only its bias (B3's bias taken at every standstill, here as a
                # measurement of the filter's bias state)
                k.gyro(self.gsign * gz_dps * D2R)
        else:
            k.wheel_v(float(v_cms))
            k.wheel_w(float(w_dps) * D2R)
            if gz_dps is not None and self.gsign is not None:
                k.gyro(self.gsign * gz_dps * D2R)
        if ax is not None and not still and abs(w_dps) < 15.0 and abs(ax) > JOLT_G:
            self.n_bumps += 1
            self._widen(JOLT_WIDEN_CM)
            self.evidence.append((t, "bump", round(float(ax), 3)))
        if line is not None:
            try:
                drop = max(float(v) for v in line) > LINE_DROP
            except (TypeError, ValueError):
                drop = False
            if drop and not getattr(self, "_in_drop", False):
                # a drop under the nose STARTS: widened once (not every row while it lasts: a hung robot reads it for
                # seconds, and the doubt grew without end before; Phase A's recovery then stood it for minutes)
                self.n_drops += 1
                self._widen(2.0)
                self.evidence.append((t, "line drop", None))
            self._in_drop = drop
        if front_cm is not None and 0 < front_cm < 6.0 and not still:
            self.evidence.append((t, "front", round(float(front_cm), 1)))
        self._keep()

    OWN_TURN_REL = 0.015      # the robot's own heading (library yaw x its fitted gyro_scale): MEASURED within ~1 % of
    OWN_TURN_BASE_DPS = 0.5   # the raw rate over 3 s windows (IQR 0.91-0.93 x 1.075); base noise CHOSEN

    def own_turn(self, t, th_deg):
        """The robot's OWN heading (FOLLOW th_deg), its change since the last record as a turn-rate reading (the
        firmware's library yaw does not drift standing: measured 5 / 6 Nov). Records across a pose re-set are skipped
        by the caller (reset_own_turn)."""
        prev = getattr(self, "_own_prev", None)
        self._own_prev = (t, float(th_deg))
        k = self.k
        if self.gsign is not None and not self.use_own_turn:
            return                  # the raw gyro is in charge (the replays: own heading added made gaps worse)
        if prev is None or k is None or t <= prev[0] or t - prev[0] > 0.5 or t < k.t - 0.3:
            return
        rate = wrap180(float(th_deg) - prev[1]) / (t - prev[0])
        k.predict(max(k.t, t))
        r = (self.OWN_TURN_BASE_DPS + self.OWN_TURN_REL * abs(rate)) * D2R
        H = np.zeros((1, k.N))
        H[0, 4] = 1.0
        k._update(np.array([rate * D2R]), np.array([k.s[4]]), H, np.array([[r * r]]))
        self._keep()

    def reset_own_turn(self):
        self._own_prev = None

    def _widen(self, cm):
        if self.k is not None:
            self.k.P[0, 0] += cm * cm
            self.k.P[1, 1] += cm * cm

    # -- the cameras ----------------------------------------------------------------------------------------------
    def _then(self, t):
        h = self.hist
        if not h or t < h[0][0] - 0.05:
            return None
        if t >= h[-1][0]:
            return h[-1]
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if h[m][0] <= t:
                lo = m
            else:
                hi = m
        a, b = h[lo], h[hi]
        f = (t - a[0]) / max(1e-6, b[0] - a[0])
        return (t, a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * math.atan2(
            math.sin(b[3] - a[3]), math.cos(b[3] - a[3])), a[4], a[5])

    def sigma_of(self, src, sigma=None):
        s = SRC_SIGMA.get(src, 1.0) if sigma is None else float(sigma)
        f = self.trust.get(src)
        return s * math.sqrt(f) if f else s

    def fix(self, t_pic, x, y, th_deg=None, sigma=None, src="camera", now=None, hold_disagree=False):
        """A camera fix of the PICTURE's time t_pic. Returns dict(used, d2, disagree_cm) or None (not placeable).
        hold_disagree: a camera not yet proven (tracking_layer.proven): a disagreeing fix widens the uncertainty but
        does not move the estimate (a wrong camera name must not pull the robots into its mirrored picture)."""
        if self.k is None:
            self.start(t_pic, x, y, th_deg if th_deg is not None else 0.0, self.sigma_of(src, sigma),
                       10.0 if th_deg is not None else 180.0)
            return dict(used=True, d2=0.0, disagree_cm=0.0, started=True)
        k = self.k
        if now is not None and now > k.t:
            k.predict(now)
            self._keep()
        if t_pic > k.t:              # a picture stamped after the filter's time (a camera clock ahead): taken as now
            t_pic = k.t
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        then = self._then(t_pic)
        if then is None:
            return None
        # the fix moved on to now by the filter's own motion since the picture (as freeroam_core.projected does)
        dx, dy = self.odo[0] - then[1], self.odo[1] - then[2]
        zx, zy = float(x) + dx, float(y) + dy
        sg = self.sigma_of(src, sigma)
        # the motion since the picture adds its own doubt (the covariance grown since then)
        grow = max(0.0, k.P[0, 0] - then[4]) + max(0.0, k.P[1, 1] - then[5])
        S = k.P[:2, :2] + np.eye(2) * (sg * sg + 0.5 * grow)
        inn = np.array([zx - k.s[0], zy - k.s[1]])
        d2 = float(inn @ np.linalg.solve(S, inn))
        dis = float(math.hypot(*inn))
        if dis > JUMP_CM:
            # over JUMP_CM from the filter: not a correction but another place (a misread, another robot, a robot
            # put down elsewhere): the position's doubt grows to cover it and the controller's own re-find rules
            # (reacquire: the spot must move as the wheels do) decide; the filter is re-set by them (reset())
            self.n_jumps = getattr(self, "n_jumps", 0) + 1
            k.P[0, 0] = min(1e4, max(k.P[0, 0], 0.25 * dis * dis))
            k.P[1, 1] = min(1e4, max(k.P[1, 1], 0.25 * dis * dis))
            self._keep()
            return dict(used=False, d2=round(d2, 2), disagree_cm=round(dis, 2), jump=True)
        if self.unc_learn and self.last_fix_t is not None and t_pic - self.last_fix_t >= UNC_LEARN_GAP_S:
            # (e10) the end of a camera gap: did the uncertainty given just before it cover where the camera puts it?
            u0 = self._unc_raw(None)
            if u0 is not None:
                self.unc_miss.append(dis - u0)
                if len(self.unc_miss) >= UNC_LEARN_MIN:
                    self.unc_add = max(0.0, float(np.percentile(np.array(self.unc_miss), UNC_LEARN_Q)))
        r_scale = 1.0
        widen = None
        if d2 > GATE_D2:
            # a disagreement (evidence, not dropped): the fix is used with its noise stretched by how far out it is (a
            # robust update: it pulls, but cannot fling the heading and wheel scales through the covariance), and the
            # position's uncertainty is widened by the disagreement AT ONCE afterwards (Phase A's A1)
            self.n_disagree += 1
            self.last_disagree_cm = dis
            self.disagree_t = k.t
            r_scale = d2 / GATE_D2
            widen = inn.copy()
            if hold_disagree:
                k.P[0, 0] = min(1e4, k.P[0, 0] + inn[0] * inn[0])
                k.P[1, 1] = min(1e4, k.P[1, 1] + inn[1] * inn[1])
                self.n_held = getattr(self, "n_held", 0) + 1
                self._keep()
                return dict(used=False, d2=round(d2, 2), disagree_cm=round(dis, 2), held=True)
        elif self.learn:
            # learned trust: the camera's own noise is what the innovations show beyond the filter's own doubt
            # (E[inn^2] = P + R per axis); robust: the median of the last LEARN_KEEP agreeing fixes
            q = self.src_d2.setdefault(src, deque(maxlen=LEARN_KEEP))
            q.append((0.5 * float(inn @ inn), 0.5 * float(k.P[0, 0] + k.P[1, 1]) + 0.5 * grow))
            if len(q) >= MIN_LEARN:
                r_hat = float(np.median([a for a, _ in q])) / (CHI2_2_MEDIAN / 2.0) - float(np.median([b for _, b in q]))
                s0 = SRC_SIGMA.get(src, 1.0) if sigma is None else float(sigma)
                self.trust[src] = min(TRUST_MAX, max(TRUST_MIN, r_hat / (s0 * s0)))
        H = np.zeros((2, k.N))
        H[0, 0] = H[1, 1] = 1.0
        # a late fix moved on to now corrects the position (and, through the covariance, the heading and the wheel
        # scale), never the speed, turn rate or gyro bias directly: those come from the wheels and the gyro (a camera
        # 1 s late on a clock 0.7 s ahead made the speed states run away in the fast simulator before this)
        k._update(np.array([zx, zy]), k.s[:2].copy(), H, np.eye(2) * (sg * sg + 0.5 * grow) * r_scale,
                  freeze=(3, 4, 5))
        # (e10) the camera-limited doubt right after this fix (before any disagreement is added on top): unc_cm
        # multiplies only what grows beyond it
        self.p_fix = (float(k.P[0, 0]), float(k.P[1, 1]))
        if widen is not None:
            rest = 0.5 * float(widen @ widen) * (1.0 - 1.0 / r_scale)     # what the stretched fix did not pull in
            k.P[0, 0] = min(1e4, k.P[0, 0] + rest)
            k.P[1, 1] = min(1e4, k.P[1, 1] + rest)
        k.last_cam = k.t
        if th_deg is not None:
            hs = 3.0 * D2R if src == "tags" else 8.0 * D2R
            th_now = th_deg * D2R + (k.s[2] - then[3])
            Hh = np.zeros((1, k.N))
            Hh[0, 2] = 1.0
            k._update(np.array([math.atan2(math.sin(th_now), math.cos(th_now))]), np.array([k.s[2]]), Hh,
                      np.array([[hs * hs]]), angle_rows=(0,), freeze=(3, 4, 5))
            if self.gh.th is not None and src == "tags":
                self.gh.set(wrap180(th_deg + (k.s[2] - then[3]) / D2R))
        if not (np.all(np.isfinite(k.s)) and np.all(np.isfinite(k.P))) or k.P[0, 0] > 1e6:
            # numerically lost (never seen in the replays; a guard): started again at this fix
            self.n_restarts = getattr(self, "n_restarts", 0) + 1
            self.start(k.t, float(x), float(y), th_deg if th_deg is not None else 0.0, sg,
                       10.0 if th_deg is not None else 180.0)
            return dict(used=True, d2=0.0, disagree_cm=0.0, started=True)
        self.n_fix[src] = self.n_fix.get(src, 0) + 1
        self.last_fix_t = t_pic
        self._keep()
        return dict(used=True, d2=round(d2, 2), disagree_cm=round(dis, 2))

    # -- out -----------------------------------------------------------------------------------------------------
    EXTRAP_S = 0.3           # the pose is never carried on past the last RAW row by more than this (no RAW: the robot
                             # is silent or stopped; its last speed carried on would drive the estimate away)

    def state(self, t=None):
        k = self.k
        if k is None:
            return None, None
        if t is None or t <= k.t:
            return k.s, k.P
        if self.last_raw_t is not None and t > self.last_raw_t + self.EXTRAP_S:
            s, P = k.predicted(max(k.t, self.last_raw_t + self.EXTRAP_S))
            P = P.copy()
            gap = t - (self.last_raw_t + self.EXTRAP_S)     # the doubt still grows (it may be moving unheard)
            P[0, 0] += (2.0 * gap) ** 2
            P[1, 1] += (2.0 * gap) ** 2
            return s, P
        return k.predicted(t)

    def pose(self, t=None):
        s, _ = self.state(t)
        if s is None:
            return None
        th = s[2] / D2R
        if self.gyro_heading and self.gh.heading_deg() is not None:
            th = self.gh.heading_deg()
        return (float(s[0]), float(s[1]), wrap180(float(th)))

    def sigma_xy(self, t=None):
        _, P = self.state(t)
        if P is None:
            return None
        return math.sqrt(max(0.0, P[0, 0], P[1, 1]))

    def unc_cm(self, t=None):
        """The uncertainty the edge rule and the planner use (cm): K_SIGMA x the 1-sigma position error, never under the
        camera's 1 cm. Staleness, blind driving and disagreements are all inside the covariance; (e10) plus what it
        learned it misses at the end of camera gaps (UNC_LEARN)."""
        u = self._unc_raw(t)
        if u is None:
            return None
        return u + (self.unc_add if self.unc_learn else 0.0)

    def _unc_raw(self, t=None):
        _, P = self.state(t)
        if P is None:
            return None
        if not self.split_unc or self.p_fix is None:
            # (e21) a covariance pushed below 0 by rounding (harsh seed 3, a fix with a tiny sigma) is the floor, not a crash
            return max(UNC_FLOOR_CM, self.k_sigma * math.sqrt(max(0.0, P[0, 0], P[1, 1])))
        # (e10, C4) two parts. The doubt the camera's last fix left (the filter's own posterior: right while the camera
        # sees the robot) x K_TRACK, and the doubt grown SINCE that fix (blind driving, staleness, disagreements) x
        # K_SIGMA, the factor measured on real camera gaps. One factor for both (e09) gave a robot the camera sees 12
        # times a second 3-5 cm of doubt against the blend's 1-2, and near an edge nothing fitted (rehearsal e(b))
        f = max(self.p_fix[0], self.p_fix[1])
        g = max(0.0, max(P[0, 0] - self.p_fix[0], P[1, 1] - self.p_fix[1]))
        return max(UNC_FLOOR_CM, K_TRACK * math.sqrt(max(0.0, f)) + self.k_sigma * math.sqrt(g))

    def heading_sd_deg(self, t=None):
        _, P = self.state(t)
        return None if P is None else math.sqrt(max(0.0, P[2, 2])) / D2R

    def summary(self):
        k = self.k
        return dict(version=VERSION, fixes=dict(self.n_fix), disagreements=self.n_disagree,
                    trust={s: round(f, 2) for s, f in self.trust.items()},
                    gyro_sign=self.gsign, gyro_bias_dps=None if self.gh.bias is None else round(self.gh.bias, 3),
                    bias_updates=self.gh.n_bias, bumps=self.n_bumps, drops=self.n_drops,
                    wheel_scale=None if (k is None or not self.scale) else round(float(k.s[6]), 4),
                    turn_scale=None if (k is None or not self.scale) else round(float(k.s[7]), 4),
                    gyro_heading=self.gyro_heading, unc_add_cm=round(self.unc_add, 2), unc_misses=len(self.unc_miss))


def raw_true(row):
    """(v_cms, w_dps) in true units from a RAW row dict (raw_units: v_cms column is mm/s, speeds 1.08 high)."""
    try:
        import raw_units as RU
    except ImportError:            # pragma: no cover
        sys.path.insert(0, _TOOLS)
        import raw_units as RU
    return RU.speed_cms(row.get("v_cms")), RU.turn_dps(row.get("w_dps"))
