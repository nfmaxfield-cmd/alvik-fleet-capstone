#!/usr/bin/env python3
"""Phase A of the one-pipeline plan (e08, Nick 7 Nov): the safety rules every run type shares, in one place, so home free
roam (freeroam_core), the missions and experiments (run_plan) and the shared checks (observe) cannot drift apart again
(the 7 Nov audit: the two pipelines were built side by side and lacked each other's safeguards).

  A1 position     PoseFuse: the robot's own pose against the camera (latency-compensated). A disagreement is information:
                  the position uncertainty widens AT ONCE by the disagreement, and plans start from the best fused
                  estimate (the camera when fresh). The old rule (wait POSE_AGREE_S = 2 s, then stop and re-seed) is kept
                  behind POSE_RULE = "wait2s".
  A2 edges        ONE derived edge clearance instead of the fixed 16 / 17 / 13 / 25 / 12-21 cm:
                      clearance = half the body's diagonal + the position uncertainty now
                                  + the stopping distance (where the move stops) + the camera's edge bias (lights)
                  applied to the whole swept path (curve bulges, turns on the spot, blind backing). The old numbers stay
                  behind EDGE_RULE = "fixed" (edge_room_cm(caller) returns them). EdgeJudge says, logged, whether the
                  software is SURE or UNSURE that a robot is near an edge; Questions is the hook that asks a person.
  A3 tilt         tilt_recovery_tries(): what a tilted robot at an edge may try, in order, never towards the drop.
  A4 tilt limits  TILT_LIMITS: re-picked from the 5 and 6 Nov home logs (runtime params, no reflash).
  A6 keep-outs    keepout_confirmed(): sensor-only keep-outs fade after UNCONFIRMED_S (front_stop's 15 s rule); confirmed
                  ones (camera-seen, felt twice, a real drop or edge trip, a tilt) stay. Used by freeroam_core, run_plan
                  and observe.trip_keepout.
  A11 battery     battery_floor(run_type): free roam 10 %, everything else 25 % (engine_checks.battery_floor reads it).

Pure functions and small classes (no sockets, no clock of their own): tests/test_e08_phase_a.py.
"""
import json
import math
import os
import time

VERSION = "2026-11-10e14"   # e14 (school 17:46, a slow tag detector in dim light): fresh_fix_s, how old a fix may be and still count as seen, from the robot's measured fix interval (MIRLT_FIX_GAP_RULE=fixed: 0.6 s as before); between_fixes_cm. e13 (10 Nov, Nick "derived, risk-based thresholds"): no_picture_limit, the time a robot may go
#                              without camera pictures, from its speed, heading, uncertainty and the edge room. 2026-11-07a: e08
VERSION_A = "2026-11-07a"

# =============================================================================================== switches
# (CLAUDE.md "keep features, switch them off") each old rule stays, behind a switch decided here by default and out of a
# new user's way. Environment variables let a run (or a test) pick the old rule without code changes.
EDGE_RULE = os.environ.get("MIRLT_EDGE_RULE", "derived")       # "derived" (e08) | "fixed" (the numbers before e08)
POSE_RULE = os.environ.get("MIRLT_POSE_RULE", "widen")         # "widen" (e08) | "wait2s" (observe.PoseAgree as before)
KEEPOUT_RULE = os.environ.get("MIRLT_KEEPOUT_RULE", "fade")    # "fade" (e08, unconfirmed ones expire) | "permanent"

# =============================================================================================== A2: the edge rule
HALF_DIAG_CM = 6.72      # the 9.5 cm square's half diagonal: a circle round the body however it turns. Not chosen
CAM_SIGMA_CM = 1.0       # the camera spot's 1-sigma error away from edges (cam_bridge FIX_SIGMA_CM). Not chosen here
TRACK_P95_CM = 1.7       # MEASURED: cross-track error at a stop-turn drive's end, 95th percentile, 2,176 drives of the
                         # 5 Nov home run (0-90 min, fixes accepted; 1.54-1.74 by speed). The planners' "uncertainty
                         # when nothing more is known" (a plan made ahead of time). DOE: no (a measurement; re-measure
                         # per site)
CAM_EDGE_BIAS_CM = {"lights": 5.0, "motion": 5.0, "tags": 0.0, "none": 0.0}
                         # MEASURED: the outline / lights tracker's spot rides onto the nose near an open edge. 25 Oct
                         # 5 cm, 27 Oct 5-9 cm; re-measured on the 5 Nov run (e08): at the 3 bottom-zone trips the camera
                         # put the middle 4.9-7.5 cm from the edge where the drop the sensor sees puts it ~8-10 cm (2-5 cm
                         # outward); Right's 16 min with its nose over y = 0: the camera 2.8 cm PAST the edge, the true
                         # middle 0-4.8 cm inside (3-7.5 cm outward). 5 cm is the middle of those. Its sign follows the
                         # nose (onto the nose: outward facing the edge, inward backing or side-on), so it is added as
                         # an error, not a correction. Roof tags read the roof's middle: 0. DOE: yes (a tape-measured
                         # edge approach at 0 / 45 / 90 deg; docs/doe-backlog.md)
CAM_BIAS_ON = os.environ.get("MIRLT_CAM_EDGE_BIAS", "on") != "off"   # switch: "off" drops the bias term
STOP_P95_CM = {5.0: 1.4, 7.5: 2.1, 10.0: 2.0, 13.0: 2.4}
                         # MEASURED: how far past its planned end a drive stops (the camera's end, its 0.44 s delay times
                         # the speed added back), 95th percentile by speed, the same 2,176 drives. The median is 0.1 cm
                         # (the calibration's stop lead, 0.3-0.5 cm, is the median's sibling). A robot's own stop lead
                         # from its calibration raises it, never lowers it. DOE: no (a measurement; per robot later)
STOP_DEFAULT_CM = 2.4    # the largest measured (13 cm/s) when the speed is not known
FIXED_EDGE_CM = {        # (EDGE_RULE "fixed") the numbers before e08, each where it came from
    "freeroam": 16.0,          # freeroam_core.EDGE_CC (chosen 5 Nov): a move's end spot
    "freeroam_path": 9.22,     # freeroam_core.LOOP_EDGE / TURN_EDGE_CC = HALF + 2.5: curve samples, turns on the spot
    "missions": 17.0,          # missions.SETTINGS edge_room_cm, production.EDGE_IN_CM (Nick 28 Oct 21:27)
    "school": 13.0,            # school-conference.json edge_in_cm (chosen so every sticker was reachable)
    "doe": 25.0,               # doe_loop.CLEAR_CM and ladder.EDGE_REFUSE_CM (Nick 27 Oct)
    "ladder": 25.0,
    "calib_room": 12.0,        # calib_room.ROOM_EDGE_CM (6.72 + 2 + 3, rounded up)
    "new_area": 17.0,          # e07: a new area's runs
}
# what each caller plans with when it has no live uncertainty of its own (a plan made ahead): the tracking it uses and
# whether its moves stop at their ends, and how far it may drive with its heading only roughly known (the DOE loop's and
# the ladder's crawls: the dead-reckoning bubble over a 10 cm crawl, BLIND_BASE 2 + 0.07 cm per cm, measured 29 Oct)
CALLER_EXTRA_CM = {"doe": 2.7, "ladder": 2.7}
VIRTUAL_BORDER_CM = 6.0  # the robots' own border (firmware border.h, run_plan.BORDER_CM): left as it is (Nick 7 Nov); its
                         # fires are counted in every run (0 in the 5 Nov 4 h run and the 6 Nov run)


def tracking_bias_cm(tracking="lights"):
    """The camera's edge error for this tracking (cm): lights / motion outline 5, roof tags 0."""
    if not CAM_BIAS_ON:
        return 0.0
    return float(CAM_EDGE_BIAS_CM.get(tracking or "lights", CAM_EDGE_BIAS_CM["lights"]))


def stop_cm(speed=None, leads=None):
    """The stopping distance at this speed (cm/s): the measured 95th percentile (interpolated), raised to a robot's own
    stop lead from its calibration when that is larger (leads {speed: cm})."""
    if speed is None:
        base = STOP_DEFAULT_CM
    else:
        ks = sorted(STOP_P95_CM)
        v = float(speed)
        if v <= ks[0]:
            base = STOP_P95_CM[ks[0]]
        elif v >= ks[-1]:
            base = STOP_P95_CM[ks[-1]]
        else:
            for a, b in zip(ks, ks[1:]):
                if a <= v <= b:
                    f = (v - a) / (b - a)
                    base = STOP_P95_CM[a] + f * (STOP_P95_CM[b] - STOP_P95_CM[a])
                    break
    if leads:
        try:
            ks = sorted(float(k) for k in leads)
            near = min(ks, key=lambda k: abs(k - (speed or ks[-1])))
            base = max(base, float(leads.get(near, leads.get("%g" % near, 0.0)) or 0.0))
        except (TypeError, ValueError):
            pass
    return base


def clearance_cm(unc_cm=CAM_SIGMA_CM, speed=None, tracking="lights", stops_here=True, leads=None, extra_cm=0.0):
    """THE derived edge clearance (cm from the robot's middle to the nearest drop): half diagonal + position uncertainty
    now + stopping distance (only where the move stops) + the camera's edge bias for this tracking (+ extra_cm, a caller's
    known blind error)."""
    return HALF_DIAG_CM + max(0.0, unc_cm) + (stop_cm(speed, leads) if stops_here else 0.0) + \
        tracking_bias_cm(tracking) + max(0.0, extra_cm)


def edge_room_cm(caller, tracking=None, speed=13.0, unc_cm=None, rule=None):
    """The edge room a caller with no live per-robot uncertainty plans with (a point, a lane, a refusal threshold).
    EDGE_RULE "fixed" (or rule="fixed") returns the caller's old number. Derived: clearance with the measured track
    error (TRACK_P95_CM) at the caller's speed. tracking default: the caller's own (school "tags", the rest "lights")."""
    rule = rule or EDGE_RULE
    if rule == "fixed":
        return FIXED_EDGE_CM.get(caller, 17.0)
    if tracking is None:
        tracking = "tags" if caller == "school" else "lights"
    u = TRACK_P95_CM if unc_cm is None else unc_cm
    stops = caller not in ("calib_room",)          # a spin on the spot does not overrun
    return round(clearance_cm(u, speed if stops else None, tracking, stops_here=stops,
                              extra_cm=CALLER_EXTRA_CM.get(caller, 0.0)), 1)


def edge_dist(rect, x, y):
    f = getattr(rect, "edge_dist", None)          # (e13) a proven workspace: its own distance
    if f is not None:
        return f(x, y)
    return min(x - rect[0], rect[2] - x, y - rect[1], rect[3] - y)


# =============================================================================================== (e13) no pictures
NOPIC_BLIND_PER_CM = 0.07   # the dead-reckoning error grows 0.07 cm per cm driven blind: MEASURED (29 Oct logs, 22,000
                            # stretches; run_plan BLIND_ERR_PER_CM, freeroam_core BLIND_PER_CM). Not chosen here
NOPIC_HEAD_UNC_DEG = 3.0    # its heading's doubt with no picture: 3 deg (the e03 rerun: 0.85-0.9 deg per turn after the
                            # fit, up to 6 on a long turn before it). CHOSEN; DOE row (docs/doe-backlog.md, e13)
NOPIC_STILL_CMS = 0.3       # under this a robot stands (freeroam's speed-zero line). Not chosen here
NOPIC_SLOW_SHARE = 0.5      # at this share of its limit a moving robot is slowed to half speed (its limit then doubles);
                            # at the whole limit it is held. CHOSEN; DOE row (e13)


def _out_dir(rect, x, y, d=1.0):
    """The unit direction towards the nearest edge (minus the gradient of edge_dist), numerically."""
    gx = edge_dist(rect, x + d, y) - edge_dist(rect, x - d, y)
    gy = edge_dist(rect, x, y + d) - edge_dist(rect, x, y - d)
    n = math.hypot(gx, gy)
    if n < 1e-9:
        return 0.0, 0.0
    return -gx / n, -gy / n


def no_picture_limit(x, y, heading_deg, speed_cms, unc_cm, rect, tracking="tags", head_unc_deg=None, leads=None):
    """(e13, Nick 10 Nov: thresholds from the actual risk, not a fixed 3 s) How long this robot may go with no camera
    picture before it could have drifted to the edge rule's clearance:
        room    = distance to the nearest edge (or unproven / unreadable ground) - clearance_cm(uncertainty now, speed,
                  tracking, its stop)
        closing = speed x (cos of its heading towards that edge, at least sin(heading doubt)) + speed x 0.07 (the
                  blind error's growth per cm driven)
        limit   = room / closing; a robot standing still: no limit (inf); room <= 0: 0 (act at once).
    The action follows the same risk: slowed to half speed at NOPIC_SLOW_SHARE of the limit, held at the limit.
    Returns dict(limit_s, slow_s, room_cm, closing_cms, edge_cm, words)."""
    hu = NOPIC_HEAD_UNC_DEG if head_unc_deg is None else float(head_unc_deg)
    v = abs(float(speed_cms or 0.0))
    e = edge_dist(rect, x, y)
    clr = clearance_cm(max(0.0, unc_cm or 0.0), v if v >= NOPIC_STILL_CMS else None, tracking, stops_here=True,
                       leads=leads)
    room = e - clr
    if v < NOPIC_STILL_CMS:
        lim = float("inf") if room > 0 else 0.0
        return dict(limit_s=lim, slow_s=lim, room_cm=round(room, 1), closing_cms=0.0, edge_cm=round(e, 1),
                    words="standing %.0f cm from the edge (%.0f cm of room): %s" % (
                        e, room, "no limit while it stands" if room > 0 else "no room: held at once"))
    ox, oy = _out_dir(rect, x, y)
    if heading_deg is None:
        towards = 1.0                     # heading not known: as if straight at the edge
    else:
        h = math.radians(heading_deg)
        towards = max(math.cos(h) * ox + math.sin(h) * oy, math.sin(math.radians(hu)))
    closing = v * towards + v * NOPIC_BLIND_PER_CM
    lim = max(0.0, room) / closing if closing > 1e-6 else float("inf")
    return dict(limit_s=round(lim, 2), slow_s=round(lim * NOPIC_SLOW_SHARE, 2), room_cm=round(room, 1),
                closing_cms=round(closing, 2), edge_cm=round(e, 1),
                words="%.0f cm/s, %s, %.0f cm from the edge (%.0f cm of room after its %.1f cm clearance): limit %.1f s" % (
                    v, "heading %.0f deg towards it" % math.degrees(math.acos(max(-1.0, min(1.0, towards))))
                    if heading_deg is not None else "heading unknown", e, room, clr, lim))


def path_margin(points, rect, unc_cm, speed=None, tracking="lights", unc_per_cm=0.0, stop_at_end=True, leads=None,
                keepouts=()):
    """The whole swept path's worst margin (cm; >= 0 = clear) against the derived clearance, and where:
    points [(x, y), ...] (turns on the spot: the body circle already covers any heading); unc grows unc_per_cm per cm
    driven (dead reckoning); the stopping distance extends the path past its last point (where it stops); keepouts (x, y, r)
    discs are drops too. Returns (margin, (x, y), need_at_worst)."""
    if not points:
        return float("inf"), None, 0.0
    worst, where, need_w, d, prev = float("inf"), None, 0.0, 0.0, points[0]
    pts = list(points)
    if stop_at_end and len(pts) >= 2:
        # its stopping distance carries the path on along its last direction (inward overruns count as inward)
        (ax, ay), (bx, by) = pts[-2], pts[-1]
        L = math.hypot(bx - ax, by - ay)
        if L > 1e-6:
            s = stop_cm(speed, leads) / L
            pts.append((bx + (bx - ax) * s, by + (by - ay) * s))
    for k, p in enumerate(pts):
        d += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
        need = clearance_cm(unc_cm + unc_per_cm * d, speed, tracking, stops_here=False, leads=leads)
        m = edge_dist(rect, p[0], p[1]) - need
        for kx, ky, kr in keepouts:
            m = min(m, math.hypot(p[0] - kx, p[1] - ky) - kr - need)
        if m < worst:
            worst, where, need_w = m, (p[0], p[1]), need
    return worst, where, need_w


def path_allowed(points, rect, unc_cm, speed=None, tracking="lights", unc_per_cm=0.0, stop_at_end=True, leads=None,
                 keepouts=()):
    """(ok, words). A robot already inside its clearance (margin < 0 at its start) may take only a move whose margin
    never gets worse and ends better (towards the interior): the shrink rule of observe.path_ok, now with the derived
    clearance."""
    if not points:
        return True, ""
    m0, _, need0 = path_margin(points[:1], rect, unc_cm, None, tracking, 0.0, False, leads, keepouts)
    if m0 >= 0:
        m, w, need = path_margin(points, rect, unc_cm, speed, tracking, unc_per_cm, stop_at_end, leads, keepouts)
        if m >= 0:
            return True, ""
        return False, "its path comes %.1f cm inside the edge clearance (%.1f cm) at %.0f, %.0f" % (-m, need, w[0], w[1])
    d, prev, best = 0.0, points[0], m0
    for k, p in enumerate(points[1:], 1):
        d += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
        m, _, _ = path_margin([p], rect, unc_cm + unc_per_cm * d, None, tracking, 0.0, False, leads, keepouts)
        if m < best - 0.05:
            return False, "it is already %.1f cm inside the edge clearance: only moves that gain room" % -m0
        best = max(best, m)
    if best <= m0 + 0.05:
        return False, "it is %.1f cm inside the edge clearance and this move does not gain room" % -m0
    return True, ""


LEAD_MIN_LEG_CM = 3.0    # (e07, run_plan) legs shorter than this are sent as planned (chosen: the lead is a few mm)
LEAD_MAX_SHARE = 0.4     # ... a lead never takes more than this share of a leg (chosen; no DOE: a guard)
LEAD_CAP_CM = 1.5        # ... nor more than this (calib_run LEAD_MAX_CM)


def stop_lead_for(leads, speed, L):
    """(e07's run_plan.with_stop_lead, shared since e08 so free roam uses it too) how much short a forward drive of L cm
    at speed is sent so it comes to rest on its planned end: the robot's stop lead {speed: cm} interpolated by speed,
    never over LEAD_MAX_SHARE of the leg or LEAD_CAP_CM; 0 for legs under LEAD_MIN_LEG_CM or no leads."""
    if not leads or L < LEAD_MIN_LEG_CM:
        return 0.0
    try:
        lv = {float(k): float(v) for k, v in leads.items()}
    except (TypeError, ValueError):
        return 0.0
    v = float(speed or 0.0)
    ks = sorted(lv)
    if len(ks) == 1 or v <= ks[0]:
        lead = lv[ks[0]] * (v / ks[0] if len(ks) > 1 and v < ks[0] and ks[0] > 0 else 1.0)
    elif v >= ks[-1]:
        lead = lv[ks[-1]]
    else:
        j = max(k for k in range(len(ks)) if ks[k] <= v)
        a, c = ks[j], ks[j + 1]
        lead = lv[a] + (lv[c] - lv[a]) * (v - a) / (c - a)
    return max(0.0, min(lead, LEAD_MAX_SHARE * L, LEAD_CAP_CM))


# =============================================================================================== A1: fused position
POSE_AGREE_CM = 5.0      # observe.POSE_AGREE_CM: over this ...
POSE_AGREE_S = 2.0       # ... for this long the camera's spot is sent to the robot as its pose (a re-seed). Before e08
                         # the robot was also STOPPED for it and nothing was widened meanwhile; now the widening is at
                         # once (the safety) and the re-seed does not stop it. DOE note as observe's (3 / 5 / 8 cm)
DISAGREE_DECAY_S = 1.0   # the widened uncertainty comes back down this fast once they agree again (a fix every 0.08 s:
                         # a dozen agreeing fixes). Chosen; DOE: yes, with POSE_AGREE_CM (false re-seeds vs edge margin)


class PoseFuse:
    """A1 for one robot. feed(t, own (x, y) at the picture's time, cam (x, y), cam_age_s) every fix. disagree_cm is the
    live own-vs-camera distance, added to the position uncertainty at once (no 2 s wait) and let down over
    DISAGREE_DECAY_S when they agree again. feed() returns {"kind": "reseed"} when the camera's spot should be sent
    as its pose (over POSE_AGREE_CM for POSE_AGREE_S; "wait2s": observe.PoseAgree, whose caller also stops it)."""

    def __init__(self, rule=None):
        self.rule = rule or POSE_RULE
        self.disagree_cm = 0.0
        self.t = None
        self.since = None        # over POSE_AGREE_CM since
        self.n_widen = 0
        self.max_cm = 0.0
        self._old = None

    def feed(self, t, own, cam, cam_age_s=0.0):
        if own is None or cam is None or cam_age_s is None or cam_age_s > 0.6:
            return None
        d = math.hypot(own[0] - cam[0], own[1] - cam[1])
        if self.t is not None and d < self.disagree_cm:
            # agreeing again: the widening comes down, not at once (one good fix does not prove the last bad ones wrong)
            dt = max(0.0, t - self.t)
            self.disagree_cm = max(d, self.disagree_cm - (self.disagree_cm - d) * min(1.0, dt / DISAGREE_DECAY_S))
        else:
            if d > self.disagree_cm + 1.0:
                self.n_widen += 1
            self.disagree_cm = d                         # A1: AT ONCE
        self.max_cm = max(self.max_cm, d)
        self.t = t
        if self.rule == "wait2s":
            if self._old is None:
                import observe as OB
                self._old = OB.PoseAgree()
            return self._old.feed(t, own, cam, cam_age_s)
        if d <= POSE_AGREE_CM:
            self.since = None
            return None
        if self.since is None:
            self.since = t
            return None
        if t - self.since >= POSE_AGREE_S:
            self.since = None
            return {"kind": "reseed", "d_cm": round(d, 2), "for_s": POSE_AGREE_S, "n": self.n_widen}
        return None

    def unc_extra(self):
        return self.disagree_cm if self.rule != "wait2s" else 0.0


# =============================================================================================== A2: sure / unsure
SURE_TOL_CM = 3.0        # own pose and camera within this of each other (3 camera sigmas) = they agree. Chosen; DOE:
                         # yes (replay the 5 / 6 Nov logs: the share of "unsure" moments at 2 / 3 / 5 cm against the
                         # incidents caught)
SURE_CAM_AGE_S = 0.6     # a camera fix older than this (beyond the measured delay) is not fresh (freeroam LOST_AFTER_S)

FIX_GAP_RULE = os.environ.get("MIRLT_FIX_GAP_RULE", "derived")   # (e14) "derived" | "fixed" (SURE_CAM_AGE_S alone)
FIX_GAP_FACTOR = 1.5     # (e14) = freeroam_core.LOST_GAPS (e13): a robot is unseen only after 1.5 x its longest recent gap
FIX_GAP_N = 40           # ... between fixes (its last 40). Not chosen here
FIX_GAP_PCT = 0.9        # (e14) the gap used: the 90th percentile of those (CHOSEN: one long loss among 40 is ignored,
                         # a detector that is slow on every picture is not). DOE row (e14)


def fresh_fix_s(arrivals, base=SURE_CAM_AGE_S):
    """(e14, school 17:46: in the dim room the tag detector took 122-421 ms a picture and a standing tag got 2.3 fixes a
    second) How old a camera fix may be and still count as "the camera sees it now": a slow detector is treated like a
    slow camera, from this robot's MEASURED fix interval: max(base, FIX_GAP_FACTOR x the 90th percentile gap between its
    last FIX_GAP_N fix arrivals). The same rule as free roam's lost_after (e13). arrivals: fix arrival times, oldest first.
    MIRLT_FIX_GAP_RULE=fixed: base alone (as before e14)."""
    if FIX_GAP_RULE == "fixed":
        return base
    a = [float(x) for x in (arrivals or [])][-(FIX_GAP_N + 1):]
    if len(a) < 10:
        return base
    gaps = sorted(b - c for c, b in zip(a, a[1:]) if b > c)
    if not gaps:
        return base
    # the 90th percentile, not the longest: one lost-tag gap (seconds) must not make every later loss count as seen
    return max(base, FIX_GAP_FACTOR * gaps[int(FIX_GAP_PCT * (len(gaps) - 1))])


def between_fixes_cm(speed_cms, interval_s):
    """(e14) The position error a robot gathers between two fixes, driving on its own wheels and gyro: speed x the
    measured interval x NOPIC_BLIND_PER_CM (the blind error's growth). 5 cm/s at 0.43 s: 0.15 cm; at 2 s: 0.7 cm."""
    return abs(float(speed_cms or 0.0)) * max(0.0, float(interval_s or 0.0)) * NOPIC_BLIND_PER_CM


def edge_judgement(margin_cm, cam_fresh, disagree_cm, edge_sensed=None, tilted=False):
    """Is the software SURE where the robot is relative to the edge? margin_cm: the derived clearance's margin at its
    spot with its full uncertainty (>= 0: clear of every edge); cam_fresh: a fresh camera fix; disagree_cm: own pose vs
    camera; edge_sensed: its own edge sensors (line or bottom zone) see a drop now (True), see floor (False), or not known
    (None); tilted: its tilt watch fired.
    -> dict(state "clear" | "sure_near" | "unsure", why). "clear": far enough whatever the error; "sure_near": near an
    edge and the sources agree (moves that gain room only, no question); "unsure": near an edge and they do not agree
    (or the camera is not fresh and nothing on the robot confirms): a person is asked when one is there, else it moves
    inward only along a direction the camera and its own pose agree on, or holds."""
    if margin_cm >= 0 and not tilted and not edge_sensed:
        return dict(state="clear", why="%.1f cm clear of the edge clearance" % margin_cm)
    if tilted or edge_sensed:
        return dict(state="sure_near", why="its own sensors feel the edge (%s)" % ("tilted" if tilted else "drop seen"))
    if not cam_fresh:
        return dict(state="unsure", why="near an edge (%.1f cm inside the clearance) and the camera has no fresh fix"
                    % -margin_cm)
    if disagree_cm is not None and disagree_cm > SURE_TOL_CM:
        return dict(state="unsure", why="near an edge and its own pose is %.1f cm from the camera's" % disagree_cm)
    return dict(state="sure_near", why="near an edge; camera fresh and own pose within %.1f cm" % (disagree_cm or 0.0))


def inward_heading(rect, x, y):
    """The direction (deg) straight away from the nearest edge (corners: away from both)."""
    x0, y0, x1, y1 = rect
    ds = [(x - x0, (1.0, 0.0)), (x1 - x, (-1.0, 0.0)), (y - y0, (0.0, 1.0)), (y1 - y, (0.0, -1.0))]
    ds.sort(key=lambda q: q[0])
    vx, vy = ds[0][1]
    if ds[1][0] < ds[0][0] + 8.0:                 # a corner: both edges count
        vx, vy = vx + ds[1][1][0], vy + ds[1][1][1]
    return math.degrees(math.atan2(vy, vx))


def inward_agreed(rect, own, cam, heading_deg, reverse=False, tol_deg=80.0):
    """A move along heading_deg (backwards when reverse) goes inward by BOTH the camera's spot and its own pose (each
    one's inward direction within tol_deg of the motion: it gains distance from the nearest edge by both). Never guess
    outward. tol_deg 80 (chosen: at least 0.17 cm inward per cm; the callers also check the end is further in)."""
    mot = heading_deg + (180.0 if reverse else 0.0)
    ok = True
    for p in (own, cam):
        if p is None:
            return False
        h = inward_heading(rect, p[0], p[1])
        if abs((mot - h + 180.0) % 360.0 - 180.0) > tol_deg:
            ok = False
    return ok


# =============================================================================================== the question hook
def questions_dir():
    return os.environ.get("MIRLT_QUESTIONS_DIR") or os.path.join(os.path.expanduser("~/.config/mirlt"), "questions")


class Questions:
    """A run asks a person (attended runs only): questions.jsonl gets {id, t, robot, text, choices, run}; Mission Control
    shows the open ones (start_job.status "question") and writes the answer to answers.jsonl ({id, choice, t}). The
    run polls answer(qid). Without Mission Control the question is said on the run's screen and logged; the run holds
    that robot and checks again as the evidence changes (the camera coming back answers it too)."""

    def __init__(self, run="", folder=None):
        self.dir = folder or questions_dir()
        self.run = run
        self.open = {}
        try:
            os.makedirs(self.dir, exist_ok=True)
        except OSError:
            pass

    def ask(self, robot, text, choices=("It is near the edge", "It is clear of the edge")):
        qid = "%s-%s-%d" % (self.run or "run", robot, int(time.time() * 1000) % 10 ** 9)
        rec = dict(id=qid, t=round(time.time(), 2), robot=robot, text=text, choices=list(choices), run=self.run)
        try:
            with open(os.path.join(self.dir, "questions.jsonl"), "a") as f:
                f.write(json.dumps(rec) + "\n")
        except OSError:
            pass
        self.open[qid] = rec
        return qid

    def answer(self, qid):
        try:
            with open(os.path.join(self.dir, "answers.jsonl")) as f:
                for ln in f:
                    try:
                        a = json.loads(ln)
                    except ValueError:
                        continue
                    if a.get("id") == qid:
                        self.open.pop(qid, None)
                        return a.get("choice")
        except OSError:
            pass
        return None

    def close(self, qid, why=""):
        """The run no longer needs the answer (the evidence settled it): said to Mission Control."""
        self.open.pop(qid, None)
        try:
            with open(os.path.join(self.dir, "answers.jsonl"), "a") as f:
                f.write(json.dumps(dict(id=qid, choice=None, closed=why, t=round(time.time(), 2))) + "\n")
        except OSError:
            pass


def open_questions(folder=None, max_age_s=3600.0):
    """The questions not answered or closed yet (newest last), for Mission Control."""
    d = folder or questions_dir()
    qs, done = [], set()
    try:
        with open(os.path.join(d, "answers.jsonl")) as f:
            for ln in f:
                try:
                    done.add(json.loads(ln).get("id"))
                except ValueError:
                    continue
    except OSError:
        pass
    try:
        with open(os.path.join(d, "questions.jsonl")) as f:
            for ln in f:
                try:
                    q = json.loads(ln)
                except ValueError:
                    continue
                if q.get("id") not in done and time.time() - float(q.get("t") or 0) <= max_age_s:
                    qs.append(q)
    except OSError:
        pass
    return qs


def write_answer(qid, choice, folder=None):
    d = folder or questions_dir()
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "answers.jsonl"), "a") as f:
        f.write(json.dumps(dict(id=qid, choice=choice, t=round(time.time(), 2))) + "\n")


# =============================================================================================== A3: tilt recovery
RECOVER_TILT_MARGIN_DEG = 2.0   # a recovery try that is not a straight reverse runs with the tilt stop raised to the tilt
                                # now + this (capped at RECOVER_TILT_CAP), so a worse tilt still stops it; restored after.
RECOVER_TILT_CAP = 9.0          # the hung-at-an-edge reading on 25 Oct was 9.7-10.2 deg. Both chosen; DOE: yes (the
                                # bench: one wheel over a 1 / 2 / 3 cm drop, which tries level it)
PIVOT_DEG = 15.0                # the short inward pivot. Chosen: a corner moves 1.7 cm. DOE: yes, with the above
ONE_SIDE_CM = 2.0               # the one-sided reverse's sideways offset (loads one wheel more). Chosen


def tilt_recovery_tries(heading_deg, inward_deg, back_inward):
    """The order a tilted robot at an edge tries to get out, never towards the drop (A3, Nick 7 Nov):
      1. "reverse"         straight back, only when backing goes inward (camera and its own trail agree), even if the
                           planner's general room check says no room;
      2. "reverse_left" /  a reverse along a line ONE_SIDE_CM to one side (the follow step steers: one wheel works
         "reverse_right"   harder; the firmware runs a reverse while tilted), each side once;
      3. "pivot"           a short turn on the spot (PIVOT_DEG, by the gyro) the way that swings the nose inward, with
                           the tilt stop raised to the tilt now + RECOVER_TILT_MARGIN_DEG for that step only; then (1)
                           again from the new heading;
      4. "hold"            nothing safe remains: it holds, said; a person is asked when one is there.
    back_inward: backing along the present heading goes inward (both sources). Returns the list of try names."""
    tries = []
    if back_inward:
        tries += ["reverse", "reverse_left", "reverse_right"]
    swing = (inward_deg - heading_deg + 180.0) % 360.0 - 180.0
    tries.append("pivot_left" if swing > 0 else "pivot_right")
    if back_inward:
        tries.append("reverse_after_pivot")
    tries.append("hold")
    return tries


# =============================================================================================== A4: tilt limits
# Re-picked from the 5 Nov (4 h, 3 robots) and 6 Nov (run 1, 2 robots) home logs, 12.1 robot-hours of RAW rows (e08 report
# appendix B). "Caught" = fired inside a real hang / tip window; "false" = fired anywhere else.
#   driving (firmware tilt stop): 5 deg / 300 ms caught 0 of 3 hazards in the logged rows; 4 deg / 300 ms caught 2 of 3,
#     0 false in 12.1 h; 3.5 deg / 300 ms 2 of 3 with 3 false (0.25 / h); 3 deg / 300 ms 35 false. -> 4 deg, 300 ms.
#   standing (observe.STAND_TILT, the Pi's watch): 2.5 deg / 2 s caught 2 of 3, 0 false (kept); 2 deg / 2 s 2 false.
#   The third hazard (Right's nose over y = 0 for 16 min, 5 Nov) never tilted past 3.5 deg: only the line sensors caught it.
TILT_LIMITS = {"tilt_stop_deg": 4.0, "tilt_ms": 300.0, "stand_tilt_deg": 2.5, "stand_tilt_s": 2.0}
TILT_LIMITS_OLD = {"tilt_stop_deg": 5.0, "tilt_ms": 300.0, "stand_tilt_deg": 2.5, "stand_tilt_s": 2.0}
TILT_RULE = os.environ.get("MIRLT_TILT_RULE", "logs")       # "logs" (e08) | "old" (5 deg)


def tilt_limits():
    return dict(TILT_LIMITS if TILT_RULE != "old" else TILT_LIMITS_OLD)


# =============================================================================================== A6: keep-outs
UNCONFIRMED_S = 15.0     # front_stop.UNCONFIRMED_S (Nick 29 Oct "about 15 s"); DOE noted there (5 / 15 / 30 s)
CONFIRM_NEAR_CM = 8.0    # front_stop.CONFIRM_NEAR_CM: a second felt point this near confirms both
CONFIRMED_SOURCES = ("edge guard", "tilted robot", "tilt/drop trip", "hazard", "camera", "drop", "tilt", "wall")


def keepout_confirmed(src, felt_before=False, camera_seen=False):
    """One rule (A6): a keep-out is CONFIRMED (stays for the run) when the camera saw the thing, it was felt twice at the
    same place, or it is a real drop / edge trip / tilt / failure hazard; a single sensor-only stop is UNCONFIRMED and
    fades after UNCONFIRMED_S."""
    s = (src or "").lower()
    return bool(camera_seen or felt_before or any(k in s for k in CONFIRMED_SOURCES))


def keepout_alive(q, now, rule=None):
    """A keep-out dict ({t, confirmed, ...}) still holds at `now`: confirmed ones always; unconfirmed ones for
    UNCONFIRMED_S (rule "permanent": every one, as before e08)."""
    if (rule or KEEPOUT_RULE) == "permanent" or q.get("confirmed", True):
        return True
    return now - float(q.get("t", now)) < UNCONFIRMED_S


# =============================================================================================== A7: held robots
CHECK_EVERY_S = 20.0     # a held robot (silent, or its heading not known) is checked again this often. Chosen; DOE: no
AGREE_FOR_S = 3.0        # ... and rejoins once its own pose and the camera agree (SURE_TOL_CM) for this long. Chosen


# =============================================================================================== A11: battery
BATTERY_FLOORS = {"freeroam": 10, "default": 25}   # free roam: Nick 5 Nov (drive down to 10 %); the rest: engine_checks'
                                                   # 25 % (the 25 Oct ladder sag at 20 %)


def battery_floor(run_type="default"):
    return int(BATTERY_FLOORS.get(run_type, BATTERY_FLOORS["default"]))
