"""Guess each anonymous stream's type from its features and relations, with a confidence and the evidence.

Never sees a column name. Order of work:
  1. passive types (text id, constant, counter, clock, slow battery-like level, step/mode index, on/off flag);
  2. computed streams: a planar position pair and its heading, copies, exact combinations of two others, and
     outputs that update in lockstep with a computed angle (all labelled "derived": not separate evidence);
  3. sensor types from behaviour: integration pairs (an angle and its rate), the turn reference (commanded turns,
     the camera's heading change between drives), sensor groups (streams that flicker together at rest), the
     drive reference (camera distance, distance to the boards).
Every type gets a soft score in [0, 1]; "unknown" has a fixed floor. Confidence = best score / sum of scores
(the design doc's "share of all scores"), shrunk when the lead over the runner-up is small.

Thresholds for the decision (design doc 3): ACCEPT 0.90 = labelled "likely X" with one-tap confirm; ASK 0.60 =
user picks from the top two or three; below = unknown, kept out of the fusion."""
import numpy as np

TYPES = ("gyro_rate", "gyro_angle", "wheel_angle", "wheel_speed", "accel", "range", "line", "switch", "battery",
         "constant", "clock", "counter", "mode", "id", "derived", "unknown")
LABEL = {"gyro_rate": "gyro rate", "gyro_angle": "gyro angle", "wheel_angle": "wheel angle",
         "wheel_speed": "wheel speed", "accel": "accelerometer axis", "range": "range sensor zone",
         "line": "line / reflectance sensor", "switch": "switch / on-off flag", "battery": "battery level",
         "constant": "constant (setting or not configured)", "clock": "clock", "counter": "row counter",
         "mode": "step / mode index", "id": "text id", "derived": "derived (computed from other streams)",
         "unknown": "unknown"}
ACCEPT, ASK = 0.90, 0.60       # chosen (design doc section 3); evaluated at other values in the replay
UNKNOWN_FLOOR = 0.05           # chosen: an "unknown" score every stream starts with
COPY_R = 0.97                  # chosen: copies (a processed copy keeps |r| above this)
# motion response (1 s spread while moving / rest noise) from "weak" to "strong". Re-anchored 2026-10-25 when the
# figure changed to moving windows only: on the development cases line sensors' groups read 0.9-1.7 and range
# zones' groups 1.6-13.6 (median 3.3). Was 2.0-6.0 on the all-window figure. Chosen on dev only (DOE row)
AOR_RAMP = (1.5, 3.0)
GRAVITY_LON = 300.0            # chosen (as has_gravity_like's 300): level / noise a gravity axis must reach
PASSIVE_TYPES = {"clock", "counter", "id", "constant", "battery", "mode", "switch"}
MOTION_GATE_S = 60.0           # seconds of motion before a motion-type label may be accepted (threshold report;
#                                chosen on dev from 0/5/60/120/160/240 s, confirmed on held-out)
CONF_CAP = (0.6, 0.4)          # chosen: confidence <= 0.6 + 0.4 x best score (DOE row)
HEADING_TIE = 0.10             # chosen: planar fits this close count as a tie between two headings (DOE row)
SHORT_RAMP = (30.0, 120.0)     # chosen: log duration (s) from "fully short" to "long"; see the threshold report


def ramp(x, lo, hi, nan=0.0):
    """0 below lo, 1 above hi, linear between (hi < lo reverses). NaN -> `nan`."""
    if x is None or not np.isfinite(x):
        return nan
    if hi == lo:
        return float(x >= hi)
    v = (x - lo) / (hi - lo)
    return float(min(1.0, max(0.0, v)))


def _fin(v, d=0.0):
    """v if it is a finite number (negative excess counts as 0), else d."""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return d
    return max(v, 0.0) if np.isfinite(v) else d


def _board(f, n_min=12):
    """Agreement of a reading with the camera's distance to the boards ahead: a range reading grows with that
    distance (positive r); needs n_min drive ends."""
    if (f.get("board_n") or 0) < n_min:
        return 0.0
    v = f.get("board_corr")
    return float(v) if v is not None and np.isfinite(v) else 0.0


def _g(f, k, d=np.nan):
    v = f.get(k, d)
    return d if v is None else v


def _twin_distance(fa, fb):
    """How alike two streams look (0 = identical statistics): levels, how often they change, how often they sit
    flat, how often they flicker at rest."""
    out = []
    # scale-free statistics only, so two channels in different units (or one rescaled) still match
    for k in ("n_unique", "change_frac", "flat_frac", "still_change_frac"):
        a, b = _g(fa, k), _g(fb, k)
        if np.isfinite(a) and np.isfinite(b) and a > 0 and b > 0:
            out.append(abs(np.log(a / b)))
    return float(np.mean(out)) if out else 9.0


def find_derived(feats, R):
    """stream -> (reason, partners, strength) for computed streams."""
    der = {}
    rel = R["rel"]
    # 1. planar position pair and its heading
    pl = R.get("planar") or []
    if pl:
        Rv, xa, ya, h, u, sg = pl[0]
        # Two angles can both steer the position pair: the pose's own heading and the IMU's fused yaw (both follow
        # the robot's turns). The fused yaw is the running sum of a rate that flickers at rest (a sensor); the
        # pose heading is not (it comes from the wheels). Among headings that fit almost as well (circular R within
        # HEADING_TIE), prefer one that is not the integral of a flickering rate (2026-10-25: the two were swapped
        # and accepted at 0.95 on synthetic batteries and on log pieces)
        def sensor_sum(hh):
            return any(a == hh and _g(feats[b], "still_change_frac", 0.0) > 0.05 for a, b, r in R["integration"])
        ties = [p for p in pl if {p[1], p[2]} == {xa, ya} and p[0] >= Rv - HEADING_TIE]
        if sensor_sum(h):
            alt = [p for p in ties if not sensor_sum(p[3])]
            if alt:
                Rv, xa, ya, h, u, sg = alt[0]
                R["planar"] = [alt[0]] + [p for p in pl if p is not alt[0]]   # later rules read [0] as the pose
        for s in (xa, ya):
            der[s] = ("position worked out from a heading and distance: the direction of its change with its "
                      "partner follows the heading stream (circular R %.3f)" % Rv, [xa, ya, h], Rv, "planar")
        der[h] = ("heading of that position pair (circular R %.3f)" % Rv, [xa, ya], Rv, "planar")
        # the turn rate that the pose heading integrates, when it has no rest flicker of its own (a sensor has
        # noise when still; a computed rate is zero)
        for a, b, r in R["integration"]:
            if a == h and b not in der and _g(feats[b], "still_change_frac", 1.0) < 0.02:
                # when the rate has a copy with fewer distinct levels, that coarser copy is the computed one
                c = rel.get(b, {}).get("copy_of")
                if c and abs(rel[b].get("copy_r", 0)) >= COPY_R and \
                        (_g(feats[c], "n_unique", 0) or 0) < (_g(feats[b], "n_unique", 0) or 0):
                    der[c] = ("turn rate of the pose: a coarser copy (r %.3f) of the rate the pose heading "
                              "integrates" % rel[b]["copy_r"], [b], abs(rel[b]["copy_r"]), "poserate")
                    continue
                der[b] = ("turn rate of the pose: the pose heading is its integral (r %.3f) and it is exactly "
                          "still at rest" % r, [h], abs(r), "poserate")
    # 2. copies (one is a processed copy of the other: the one with less rest flicker, else fewer levels)
    pairs = []
    ids = R["ids"]
    for a in ids:
        b = rel[a].get("copy_of")
        if b and abs(rel[a].get("copy_r", 0)) >= COPY_R and a < b:
            pairs.append((a, b, rel[a]["copy_r"]))
    for a, b, r in pairs:
        if a in der or b in der:
            continue
        fa, fb = feats[a], feats[b]
        if rel[a].get("accumulates") and rel[b].get("accumulates"):
            continue        # two angles from the same rate: handled as gyro angle / pose heading
        if _twin_distance(fa, fb) < 0.35:
            continue        # two look-alike channels (two wheels driving straight), not a copy
        ka = (_g(fa, "still_change_frac", 0.0), _g(fa, "n_unique", 0))
        kb = (_g(fb, "still_change_frac", 0.0), _g(fb, "n_unique", 0))
        copy, orig = (a, b) if ka < kb else (b, a)
        der[copy] = ("a processed copy of another stream (r %.3f), with less noise at rest" % r, [orig], abs(r),
                     "copy")
    # 3. exact combinations of two: in a set of three that explain each other, the odd one out is computed from
    #    the two that look alike (two wheels, two zones)
    for a, p, q, r2 in sorted(R["combos"], key=lambda c: -c[3]):
        if a in der or p is None:
            continue
        if p in der or q in der:
            continue
        trio = [a, p, q]
        cands = []
        for i in range(3):
            o = trio[i]
            pair = [x for x in trio if x != o]
            cands.append((_twin_distance(feats[pair[0]], feats[pair[1]]), o, pair))
        cands.sort(key=lambda c: c[0])
        best = cands[0]
        gof = R["group_of"]
        same_group = R.get("grouping") == "rest" and gof.get(a) is not None and gof.get(a) == gof.get(p) == gof.get(q)
        if same_group or cands[1][0] - cands[0][0] < 0.15:
            # all three look alike (three zones of one sensor): the computed one is the weighted sum of the
            # other two with both weights positive (a mean or a sum); the reverse relations need one negative
            coef = rel[a].get("combo_coef")
            best = (0.0, a if coef and coef[0] > 0 and coef[1] > 0 else None, [p, q])
        if any(a in (x, y) and x not in der and y not in der for x, y, r in R["integration"]):
            continue        # a stream with its own integral partner is a measured channel (a wheel's speed)
        own = _g(feats[a], "still_change_frac", 0) or 0
        theirs = max(_g(feats[p], "still_change_frac", 0) or 0, _g(feats[q], "still_change_frac", 0) or 0)
        if best[1] == a and not (own > 0.05 and theirs < 0.01):
            der[a] = ("an exact combination of two similar streams (R2 %.3f on held-out windows)" % r2, [p, q], r2,
                      "combo")
    # 4. outputs that update in lockstep with a computed angle (the integral of a sensor's rate). An "angle" that
    #    flickers at rest is a sensor axis that a tilt made follow a rate's integral (2026-10-25: a tilted
    #    accelerometer axis), not a computed angle, so its group-mates are not computed outputs
    angles = {a for a, b, r in R["integration"] if (_g(feats[a], "still_change_frac", 0.0) or 0.0) <= 0.05}
    gof = R["group_of"]
    for s in ids:
        if s in der or s in angles:
            continue
        gi = gof.get(s)
        mates = [m for m in R["groups"][gi] if m != s] if gi is not None else []
        # a running sum or a stream that is flat at rest is a computed output, never a sensor channel
        comp = [m for m in mates if m not in der and (m in angles or m in R.get("motion_streams", []))]
        if len(mates) + 1 > 4:
            comp = []       # a big group is a multi-zone sensor that happens to share the tick, not one output
        # and it must update no more often than the computed angle does (a faster sensor's changes contain the
        # angle's ticks without being computed with it)
        tick = max([_g(feats[m], "change_hz", 0) or 0 for m in mates] + [1e-9])
        if (_g(feats[s], "change_hz", 0) or 0) > 1.5 * tick:
            comp = []
        if comp and not rel[s].get("accumulates") and R.get("grouping") == "rest":
            der[s] = ("updates in lockstep with a computed angle (an integral of a rate), not with any sensor "
                      "group", comp, 0.8, "lockstep")
    return der


def score_stream(s, f, R, der, ctx):
    """Soft score per type for one stream, and the evidence lines."""
    sc = {t: 0.0 for t in TYPES}
    ev = []
    sc["unknown"] = UNKNOWN_FLOOR
    if f.get("text"):
        sc["id"] = 1.0
        ev.append("values are text, not numbers")
        return sc, ev
    if f.get("const"):
        sc["constant"] = 1.0
        ev.append("one value for the whole case (%s)" % f.get("level"))
        return sc, ev
    mono = _g(f, "monotone_up", 0)
    tcorr = _g(f, "time_corr_in_seg", 0)
    stepm = _g(f, "step_mode_frac", 0)
    if mono > 0.999 and tcorr > 0.99 and (_g(f, "change_frac", 0) or 0) > 0.5:
        c = ramp(stepm, 0.8, 0.98)
        if c < 0.5 and ctx.get("has_time_base") and not f.get("is_time_base"):
            # one clock per robot: another stream already is the time base, so a stream that merely rose with time
            # here (a wheel turned one way, the pose drove one way through a short log) is judged on behaviour
            # (2026-10-25: wheel angles and pose x/y were accepted as clocks at 0.95 on log pieces)
            ev.append("rises with time here (r %.4f), but another stream is the clock: judged on its behaviour"
                      % tcorr)
        else:
            sc["counter"] = c
            sc["clock"] = 1 - c
            ev.append("never falls inside a segment and rises with time (r %.4f); equal steps on %.0f%% of rows"
                      % (tcorr, 100 * stepm))
            return sc, ev
    if f.get("binary"):
        sc["switch"] = 0.3
        ev.append("two values only (a contact or flag; only a touch or the user can confirm which)")
        return sc, ev
    chz = _g(f, "change_hz", 0)
    nun = _g(f, "n_unique", 0)
    oneq = _g(f, "one_quantum_frac", 0)
    integer = bool(f.get("integer"))
    short = ctx["short"]
    if chz < 0.3 and nun <= 120 and s not in der:
        b = ramp(chz, 0.05, 0.005) * ramp(abs(_fin(_g(f, "trend_corr", 0), 0) if False else abs(_g(f, "trend_corr", 0) or 0)), 0.3, 0.7)
        m = ramp(chz, 0.005, 0.05) * (0.9 if integer else 0.6)
        sc["battery"], sc["mode"] = b * (1 - 0.6 * short), min(1.0, m) * (1 - 0.6 * short)
        ev.append("changes slowly (%.3f a second), %d levels, %.0f%% of changes are one step, trend over time r %.2f"
                  % (chz, nun, 100 * oneq, _g(f, "trend_corr", 0)))
        return sc, ev
    # from here on the evidence is about behaviour: a short case (little time, little motion) earns less trust
    sc["unknown"] = UNKNOWN_FLOOR + 0.35 * short
    if short > 0:
        ev.append("short case (%.0f s of log): weaker evidence" % ctx["duration_s"])
    if s in der:
        why, partners, strength = der[s][0], der[s][1], der[s][2]
        sc["derived"] = 0.6 + 0.4 * ramp(strength, 0.8, 0.99)
        ev.append(why)
        if der[s][3] in ("lockstep", "planar", "poserate", "copy"):
            return sc, ev
    rel = R["rel"].get(s, {})
    acc = rel.get("accumulates", False)
    integ = {a: (b, r) for a, b, r in R["integration"]}
    rate_of = {b: a for a, b, r in R["integration"]}           # rate -> its angle
    gof = R["group_of"]
    grp = R["groups"][gof[s]] if s in gof else [s]
    gsize = len(grp)
    flick = _g(f, "still_change_frac", np.nan)
    turn_int = max(_g(f, "cam_turn_int_excess", 0), 0) if np.isfinite(_g(f, "cam_turn_int_excess", 0)) else 0
    turn_ang = max(_g(f, "cam_turn_ang_excess", 0), 0) if np.isfinite(_g(f, "cam_turn_ang_excess", 0)) else 0
    tmid = abs(_g(f, "turn_mid_corr", 0)) if np.isfinite(_g(f, "turn_mid_corr", 0)) else 0
    tdel = abs(_g(f, "turn_delta_corr", 0)) if np.isfinite(_g(f, "turn_delta_corr", 0)) else 0
    gdel = abs(_g(f, "go_delta_corr", 0)) if np.isfinite(_g(f, "go_delta_corr", 0)) else 0
    gmid = abs(_g(f, "go_mid_corr", 0)) if np.isfinite(_g(f, "go_mid_corr", 0)) else 0
    camgo = _g(f, "cam_go_delta_corr", np.nan)
    camrate = _g(f, "cam_speed_corr_rate", np.nan)
    camabs = _g(f, "cam_speed_corr_abs", np.nan)
    aor = _g(f, "active_over_rest", np.nan)

    # tilt evidence (a tilt by hand): this stream's integral explains another IMU stream's change (a gyro axis
    # turning gravity between accelerometer axes), or the reverse. Direct evidence for both, so no cap
    tilt = ctx.get("tilt") or {}
    if s in tilt.get("gyro", {}) or s in tilt.get("accel", {}):
        is_g = s in tilt["gyro"]
        partner, r = (tilt["gyro"] if is_g else tilt["accel"])[s]
        sc["gyro_rate" if is_g else "accel"] = 1.0
        ev.append("tilt test: %s (r %.2f on held-out windows)" %
                  ("its integral follows the change of another IMU axis: a rotation rate turning gravity"
                   if is_g else "its change follows the integral of a rotation rate: gravity moving onto this axis",
                   r))
        return sc, ev
    # angle / rate pairs
    if s in integ or s in rate_of:
        is_angle = s in integ
        partner = integ[s][0] if is_angle else rate_of[s]
        r = integ[s][1] if is_angle else dict((b, r) for a, b, r in R["integration"])[s]
        rate_s = partner if is_angle else s
        angle_s = s if is_angle else partner
        twin = ctx["twin_pair"].get(angle_s)
        imu = ctx["imu_group"]
        sensor_rate = rate_s
        for s2 in R["ids"]:
            if R["rel"][s2].get("copy_of") == rate_s and abs(R["rel"][s2].get("copy_r", 0)) >= COPY_R:
                if imu is not None and s2 in imu["members"]:
                    sensor_rate = s2
        imu_like = imu is not None and sensor_rate in imu["members"]
        turn_ref = max(_fin(ctx["feats"][sensor_rate].get("cam_turn_int_excess")),
                       _fin(ctx["feats"][angle_s].get("cam_turn_ang_excess")))
        drive_ref = max(abs(_g(ctx["feats"][angle_s], "cam_go_delta_corr", 0) or 0),
                        abs(_g(ctx["feats"][angle_s], "go_delta_corr", 0) or 0),
                        abs(_g(ctx["feats"][angle_s], "cam_speed_corr_rate", 0) or 0))
        wheel = (0.45 if twin else 0.0) + 0.35 * ramp(drive_ref, 0.3, 0.7) + (0.2 if not imu_like else 0.0)
        gyro = (0.45 if imu_like else 0.0) + 0.35 * ramp(turn_ref, 0.1, 0.4) + (0.2 if not twin else 0.0)
        ev.append("%s of a partner stream (integral relation r %.3f)" % ("integral" if is_angle else "rate", r))
        if twin:
            ev.append("its angle has a look-alike partner (a second wheel?)")
        if imu_like:
            ev.append("the rate (or the sensor it copies) belongs to the IMU-like set")
        if turn_ref > 0:
            ev.append("follows the camera's heading change between drives (excess R %.2f)" % turn_ref)
        if is_angle:
            sc["wheel_angle"] = wheel
            sc["gyro_angle"] = gyro
        else:
            sc["wheel_speed"] = wheel
            sc["gyro_rate"] = gyro
        if s in der:
            sc["gyro_angle"] *= 0.5
            sc["gyro_rate"] *= 0.5
        return sc, ev

    # a running sum that follows the turn-rate sensor's integral roughly (a fused angle), not exactly
    imu = ctx["imu_group"]
    if imu is not None and s not in der and (acc or s in R.get("motion_streams", [])):
        r_int = abs(rel.get("ds", {}).get(imu["rate"], 0.0) or 0.0)
        if r_int >= 0.7:
            sc["gyro_angle"] = 0.5 + 0.5 * ramp(r_int, 0.7, 0.98)
            ev.append("a running sum whose changes follow the integral of the IMU's turn rate (r %.2f): an angle "
                      "fused from it" % r_int)
            return sc, ev
    # sensor groups
    if imu is not None and s in imu["members"]:
        # the split between accelerometer and gyro axes rests on one sensor's axes sharing units (a noise floor);
        # only the gravity axis and the turn-following rate have direct evidence, so the others are asked about
        direct = s in (imu["gravity"], imu["rate"])
        cap = 1.0 if direct else 0.45
        if s in imu["accel"]:
            sc["accel"] = (0.55 + 0.45 * imu["split_quality"]) * cap
            sc["gyro_rate"] = 0.1 * (1 - imu["split_quality"]) + (0.0 if direct else 0.08)
            ev.append("in the group that flickers with the turn-following rate at rest; shares the noise floor of "
                      "the axis reading a steady %s at rest (gravity)" % ("level" if s != imu["gravity"] else "large level"))
        else:
            sc["gyro_rate"] = (0.55 + 0.45 * imu["split_quality"]) * cap
            sc["accel"] = 0.1 * (1 - imu["split_quality"]) + (0.0 if direct else 0.08)
            ev.append("in the IMU-like group; shares the noise floor of the turn-following rate")
        if s in der:
            sc["derived"] *= 0.5
        return sc, ev
    if gsize >= 3 and has_gravity_like(grp, ctx["feats"]):
        sc["accel"] = 0.5
        ev.append("one of %d streams that flicker together at rest, one of which reads a large steady level "
                  "(gravity?); no turn reference found to place the gyro" % gsize)
        return sc, ev
    if gsize >= 3:
        pos = _g(f, "lo", -1) >= 0
        big = ramp(aor, *AOR_RAMP, nan=0.3)
        drive = max(ramp(-(camgo if np.isfinite(camgo) else 0), 0.2, 0.6),
                    ramp(_board(f), 0.3, 0.7))
        # the group as a whole: most members respond strongly to motion
        # (motion response: over the windows where the robot moves; NaN when it hardly moved)
        g_list = [v for v in (_g(ctx["feats"][m], "active_over_rest", np.nan) for m in grp) if np.isfinite(v)]
        g_aor = float(np.median(g_list)) if g_list else np.nan
        gbig = ramp(g_aor, *AOR_RAMP, nan=0.3)
        moving = R.get("moving_frac", np.nan)
        motion_seen = ramp(moving, 0.03, 0.15, nan=0.3)
        if R.get("grouping") != "rest":
            motion_seen *= 0.5
            ev.append("(group found from timing during motion only: weaker)")
        # the group's best agreement with the camera's distances counts for every member (zones that face away
        # from the boards have no prediction of their own)
        # (the second-best member, so one chance agreement does not carry the group)
        g_vals = sorted([max(ramp(-(_g(ctx["feats"][m], "cam_go_delta_corr", 0) or 0), 0.2, 0.6),
                             ramp(_board(ctx["feats"][m]), 0.3, 0.7)) for m in grp] + [0, 0])
        g_drive = g_vals[-2]
        drive = max(drive, 0.7 * g_drive)
        sc["range"] = (0.25 * gbig + 0.15 * big + 0.45 * drive + 0.15 * pos) * (0.3 + 0.7 * motion_seen)
        # "barely responds to motion" is weak evidence for a floor sensor; whole-number readings (raw counts) are
        # the positive cue
        sc["line"] = (0.25 * (1 - gbig) + 0.15 * (1 - big) + 0.45 * (1 if integer else 0) + 0.15 * pos) * \
                     (1 - 0.8 * drive) * (0.3 + 0.7 * motion_seen)
        if motion_seen < 0.5:
            ev.append("the robot hardly moved in this case (moving %.0f%% of the time): zones and counts look alike"
                      % (100 * moving if np.isfinite(moving) else -1))
        ev.append("one of %d streams that flicker together at rest (a multi-zone or multi-channel sensor); "
                  "motion changes it %.1f x its rest noise (group median %.1f x)" % (gsize, aor if np.isfinite(aor) else -1,
                                                                                   g_aor if np.isfinite(g_aor) else -1))
        if np.isfinite(camgo):
            ev.append("change over a drive vs camera distance driven r %.2f" % camgo)
        if np.isfinite(_g(f, "board_corr", np.nan)):
            ev.append("reading vs camera distance to the boards ahead r %.2f" % f["board_corr"])
        if s in der:
            if all(m in grp for m in der[s][1]):
                # a combination inside a sensor group keeps the group's type; the derived note stays in the evidence
                sc["derived"] = 0.0
                ev.append("(computed from two neighbours in its group; same type, not separate evidence)")
            else:
                return {t: (v if t in ("derived", "unknown") else 0.0) for t, v in sc.items()}, ev
        return sc, ev
    if s not in der:
        # a lone responsive stream with no partner: weak guesses only
        if np.isfinite(camabs) and camabs > 0.6:
            sc["wheel_speed"] = 0.25
            ev.append("its size follows the camera's speed (r %.2f) but it has no partner" % camabs)
        if turn_int > 0.2:
            sc["gyro_rate"] = 0.3
        ev.append("no partner, group or reference explains it")
    return sc, ev


def has_gravity_like(gr, feats, lon_min=300.0):
    """Does the group hold an axis that reads a large steady level compared with its own noise (gravity on an
    accelerometer axis: about 1000x on the Alvik)? A range sensor's zones or a set of reflectance counts do not:
    their readings are tens of times their noise at most."""
    return any((_g(feats[m], "level_over_noise", 0) or 0) >= lon_min and not feats[m].get("accumulates")
               and (_g(feats[m], "still_change_frac", np.nan) > 0.05 if np.isfinite(_g(feats[m], "still_change_frac", np.nan))
                    else (_g(feats[m], "change_frac", 0) or 0) > 0.3)
               for m in gr)


def build_context(feats, R, der=None):
    """Case-wide structure the per-stream scores lean on: the IMU-like group and its split, look-alike pairs."""
    ctx = {"feats": feats, "twin_pair": {}, "imu_group": None, "derived": der or {}}
    ctx["has_time_base"] = any(f.get("is_time_base") for f in feats.values())
    dur = max([_g(f, "duration_s", 0) or 0 for f in feats.values()] + [0])
    ctx["duration_s"] = dur
    ctx["short"] = 1 - ramp(dur, *SHORT_RAMP)          # chosen: under 30 s counts as fully short
    angles = [a for a, b, r in R["integration"]]
    for a in angles:
        best = None
        for b in angles:
            if b == a:
                continue
            d = _twin_distance(feats[a], feats[b])
            if d < 0.35 and (best is None or d < best[0]):
                best = (d, b)
        if best:
            ctx["twin_pair"][a] = best[1]
    # IMU-like set: the rate whose integral follows turns best, plus every other stream that flickers at rest,
    # is continuous (not integer counts), is not a running sum, and is not part of a multi-channel group that
    # looks like zones (a group of 3+ that changes a lot with motion) or counts (integer channels)
    rates = []
    for a, b, r in R["integration"]:
        if a in ctx["twin_pair"]:
            continue
        rates.append(b)
        # a rate's copy (the sensor behind a processed rate) counts too
        for s2 in R["ids"]:
            if R["rel"][s2].get("copy_of") == b and abs(R["rel"][s2].get("copy_r", 0)) >= COPY_R:
                rates.append(s2)
    # the pose heading (if the planar test found one) is a turn reference too: a turn-rate sensor's integral
    # follows it even when the robot computes its pose from other streams
    head = R["planar"][0][3] if R.get("planar") else None
    hds = R["rel"].get(head, {}).get("ds", {}) if head else {}

    def turnscore(s):
        f = feats[s]
        return _fin(_g(f, "cam_turn_int_excess", 0)) + abs(_fin(_g(f, "turn_mid_corr", 0))) + \
            (0.5 if s in rates else 0) + abs(hds.get(s, 0.0) or 0.0)
    flick = [s for s in R["ids"] if (_g(feats[s], "still_change_frac", np.nan) > 0.05
                                     if np.isfinite(_g(feats[s], "still_change_frac", np.nan))
                                     else (_g(feats[s], "change_frac", 0) or 0) > 0.5)]
    cand_rate = [s for s in flick if not R["rel"].get(s, {}).get("accumulates")]
    rate = max(cand_rate, key=turnscore) if cand_rate else None
    if rate is not None and turnscore(rate) > 0.3:
        mem = []
        for s in flick:
            if R["rel"].get(s, {}).get("accumulates") or feats[s].get("integer"):
                continue
            gi = R["group_of"].get(s)
            gr = R["groups"][gi] if gi is not None else [s]
            if len(gr) >= 3 and rate not in gr and (not has_gravity_like(gr, feats) or
                                                    np.mean([bool(feats[m].get("integer")) for m in gr]) > 0.5):
                continue
            if s in ctx.get("derived", {}):
                continue
            mem.append(s)
        if rate not in mem:
            mem.append(rate)
        lv = {m: _g(feats[m], "level_over_noise", 0) or 0 for m in mem}
        grav = max([m for m in mem if m != rate], key=lambda m: lv[m]) if len(mem) > 1 else None
        if grav is not None and lv[grav] < GRAVITY_LON:
            # gravity reads about 1000x its noise on the Alvik; a computed tilt angle reads about 100x (2026-10-25:
            # pitch_deg was taken for the gravity axis, and accepted as an accelerometer, on a log piece whose az
            # had dropped out of the set)
            grav = None
        noise = {m: max(_g(feats[m], "rest_noise", 0) or 0, _g(feats[m], "quantum", 0) or 0, 1e-12) for m in mem}
        if grav is not None:
            ng, nr = np.log(noise[grav]), np.log(noise[rate])
            acc, gyr = [], []
            for m in mem:
                x = np.log(noise[m])
                (acc if abs(x - ng) < abs(x - nr) else gyr).append(m)
            split = ramp(abs(ng - nr), 0.5, 2.0)
            ctx["imu_group"] = {"members": set(mem), "accel": set(acc), "gyro": set(gyr), "gravity": grav,
                                "rate": rate, "split_quality": split}
            # a tilt angle computed from the accelerometer: follows one accelerometer axis (|r| >= 0.6)
            # but updates more slowly than the accelerometer does (a computed output on its own tick)
            acc_hz = np.median([_g(feats[m], "change_hz", 0) or 0 for m in acc]) if acc else 0
            for m in list(mem):
                if m in acc and m == grav:
                    continue
                vv = R["rel"].get(m, {}).get("vv", {})
                hit = [(a, vv.get(a, 0)) for a in acc if a != m and abs(vv.get(a, 0)) >= 0.6]
                if hit and (_g(feats[m], "change_hz", 0) or 0) < 0.6 * acc_hz and der is not None:
                    a, r = max(hit, key=lambda h: abs(h[1]))
                    der[m] = ("a tilt angle worked out from an accelerometer axis: follows it (r %.2f) but updates "
                              "%.0f%% as often" % (r, 100 * (_g(feats[m], "change_hz", 0) or 0) / max(acc_hz, 1e-9)),
                              [a], abs(r), "lockstep")
                    for k in ("members", "accel", "gyro"):
                        ctx["imu_group"][k].discard(m)
            ctx["tilt"] = tilt_pairs(feats, R, ctx, flick, der)
    return ctx


TILT_R = 0.90       # chosen: |r| of one stream's change with another's integral, on held-out windows (DOE row)


def tilt_pairs(feats, R, ctx, flick, der):
    """Tilt evidence (added 2026-10-25 from the synthetic calibration battery, tools/discovery/battery.py; it fires
    on no real log on disk, none of which has a deliberate tilt). A hand tilt turns the robot about x or y: that
    gyro axis's integral is the tilt angle, and gravity moves onto an accelerometer axis by sin(angle), so that
    axis's change follows the same integral (it even passes the exact-integral test, like a wheel angle). Sides:
      rate side   flickers at rest, continuous, is not the turn-following rate and is not itself an integral;
      changing side flickers at rest (a gyro angle or wheel angle sits still), is not a fused yaw (follows the
                  turn rate's integral) and then is either a computed tilt angle (updates < 0.6 x as often as the
                  accelerometer) or an accelerometer axis (its rest noise sits nearer the gravity axis's than the
                  gyro's).
    Returns {"gyro": {s: (partner, r)}, "accel": {s: (partner, r)}}."""
    imu = ctx["imu_group"]
    out = {"gyro": {}, "accel": {}}
    if imu is None:
        return out
    angles = {a for a, b, r in R["integration"]}
    cands = [s for s in flick if not feats[s].get("integer") and s not in der and s != imu["rate"]]
    rates = [b for b in cands if b not in angles]
    acc_hz = np.median([_g(feats[m], "change_hz", 0) or 0 for m in imu["accel"]]) if imu["accel"] else 0
    noise = lambda m: np.log(max(_g(feats[m], "rest_noise", 0) or 0, _g(feats[m], "quantum", 0) or 0, 1e-12))
    ng, nr = noise(imu["gravity"]), noise(imu["rate"])
    for a in cands:
        ds = R["rel"].get(a, {}).get("ds", {})
        if abs(ds.get(imu["rate"], 0.0) or 0.0) >= 0.7:
            continue                            # an angle fused from the turn rate (yaw)
        gi = R["group_of"].get(a)
        gr = R["groups"][gi] if gi is not None else [a]
        if len(gr) >= 4 and not (set(gr) & imu["members"]):
            continue                            # a multi-zone sensor: tilting it changes what it sees (range zones
            #                                     look over the boards), which follows the tilt without being one
        best = None
        for b in rates:
            v = ds.get(b, 0.0) or 0.0
            if b != a and abs(v) >= TILT_R and (best is None or abs(v) > abs(best[1])):
                best = (b, v)
        if best is None:
            continue
        b, v = best
        if (_g(feats[a], "change_hz", 0) or 0) < 0.6 * acc_hz:
            der[a] = ("a tilt angle: its change follows the integral of a rotation rate (r %.2f) but it updates "
                      "less often than the accelerometer (computed)" % v, [b], abs(v), "lockstep")
            continue
        if abs(noise(a) - ng) >= min(abs(noise(a) - nr), 1.0):
            continue                            # an accelerometer axis shares the gravity axis's noise floor
            #                                     (within a factor of e): a pose heading that follows a wheel turn
            #                                     rate does not (seen on a real 20 s piece, 2026-10-25)
        out["accel"][a] = (b, v)
        if b not in out["gyro"] or abs(v) > abs(out["gyro"][b][1]):
            out["gyro"][b] = (a, v)
    for s in list(out["gyro"]):
        out["accel"].pop(s, None)
    return out


def motion_seconds(feats):
    """The discoverer's own estimate of the seconds of motion in a log: 1 s windows in which a stream that sits
    flat at rest (an encoder, the pose) changed on every row. The chosen rule with this estimate in place of the
    true seconds still met its criterion on held-out log pieces (0.14% wrong plain, 0.65% disguised;
    build-reports/e02-oct25-discovery-threshold.md). A discovery run that commands its own moves passes the
    commanded seconds instead."""
    return float(max([f.get("moving_windows", 0) or 0 for f in feats.values()] + [0]))


def classify_case(feats, R, motion_s=None):
    """{stream: result dict}: type, confidence, decision, top3, evidence, derived note. motion_s: seconds of
    motion in the log (the commanded seconds, when the system commanded the moves); default: the blind estimate
    (motion_seconds)."""
    if motion_s is None:
        motion_s = motion_seconds(feats)
    der = find_derived(feats, R)
    ctx = build_context(feats, R, der)
    out = {}
    for s, f in feats.items():
        sc, ev = score_stream(s, f, R, der, ctx)
        tot = sum(sc.values())
        ranked = sorted(sc.items(), key=lambda kv: -kv[1])
        best, second = ranked[0], ranked[1]
        conf = best[1] / tot if tot > 0 else 0.0
        # weak evidence (2026-10-25): confidence is a share of all scores, so one lone weak score (0.5 from a
        # single cue) used to read 0.9+. It is now capped by the score itself: weak evidence can be asked about,
        # never accepted (a score of 0.75 is needed to reach 0.90)
        conf = min(conf, CONF_CAP[0] + CONF_CAP[1] * best[1])
        derived, derived_of = s in der, (list(der[s][1]) if s in der else [])
        if best[0] == "gyro_angle" and not derived:
            # provenance (design doc section 2): an angle summed from a rate is derived, never separate evidence
            rate = [b for a, b, r in R["integration"] if a == s]
            imu = ctx["imu_group"]
            derived, derived_of = True, rate or ([imu["rate"]] if imu else [])
        out[s] = {"type": best[0], "confidence": float(conf), "scores": sc,
                  "top3": [(t, round(v / tot, 3)) for t, v in ranked[:3] if v > 0],
                  "evidence": ev, "derived": derived, "derived_of": derived_of,
                  "decision": decide(conf, best[0], motion_s) if best[0] != "unknown" else "unknown"}
    return out


def decide(conf, typ=None, motion_s=None, accept=ACCEPT, ask=ASK, gate=None):
    """accept / ask / unknown. The rule (2026-10-25, chosen on development log pieces and confirmed on held-out
    ones: build-reports/e02-oct25-discovery-threshold.md): accept at `accept`; but a label that needs motion to
    identify (anything except the passive types) is only asked about until the log holds MOTION_GATE_S seconds of
    motion. With typ or motion_s left out, the gate does not apply (the plain threshold)."""
    gate = MOTION_GATE_S if gate is None else gate
    if conf < ask:
        return "unknown"
    if typ is not None and motion_s is not None and typ not in PASSIVE_TYPES and motion_s < gate:
        return "ask"
    return "accept" if conf >= accept else "ask"
