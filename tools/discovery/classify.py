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
UNKNOWN_FLOOR = 0.12           # chosen: an "unknown" score every stream starts with
COPY_R = 0.97                  # chosen: copies (a processed copy keeps |r| above this)


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


def _g(f, k, d=np.nan):
    v = f.get(k, d)
    return d if v is None else v


def _twin_distance(fa, fb):
    """How alike two streams look (0 = identical statistics): resolution, levels, change rate, rest noise."""
    out = []
    for k in ("n_unique", "change_frac", "rel_quantum", "window_sd_median"):
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
        for s in (xa, ya):
            der[s] = ("position worked out from a heading and distance: the direction of its change with its "
                      "partner follows the heading stream (circular R %.3f)" % Rv, [xa, ya, h], Rv)
        der[h] = ("heading of that position pair (circular R %.3f)" % Rv, [xa, ya], Rv)
        # the turn rate that the pose heading integrates, when it has no rest flicker of its own (a sensor has
        # noise when still; a computed rate is zero)
        for a, b, r in R["integration"]:
            if a == h and b not in der and _g(feats[b], "still_change_frac", 1.0) < 0.02:
                der[b] = ("turn rate of the pose: the pose heading is its integral (r %.3f) and it is exactly "
                          "still at rest" % r, [h], abs(r))
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
        ka = (_g(fa, "still_change_frac", 0.0), _g(fa, "n_unique", 0))
        kb = (_g(fb, "still_change_frac", 0.0), _g(fb, "n_unique", 0))
        copy, orig = (a, b) if ka < kb else (b, a)
        der[copy] = ("a processed copy of another stream (r %.3f), with less noise at rest" % r, [orig], abs(r))
    # 3. exact combinations of two: in a set of three that explain each other, the odd one out is computed from
    #    the two that look alike (two wheels, two zones)
    for a, p, q, r2 in sorted(R["combos"], key=lambda c: -c[3]):
        if a in der or p is None:
            continue
        if p in der or q in der:
            continue
        trio = [a, p, q]
        best = None
        for i in range(3):
            o = trio[i]
            pair = [x for x in trio if x != o]
            dist = _twin_distance(feats[pair[0]], feats[pair[1]])
            if best is None or dist < best[0]:
                best = (dist, o, pair)
        if best[1] == a:
            der[a] = ("an exact combination of two similar streams (R2 %.3f on held-out windows)" % r2, [p, q], r2)
    # 4. outputs that update in lockstep with a computed angle (the integral of a sensor's rate)
    angles = {a for a, b, r in R["integration"]}
    gof = R["group_of"]
    for s in ids:
        if s in der or s in angles:
            continue
        gi = gof.get(s)
        mates = [m for m in R["groups"][gi] if m != s] if gi is not None else []
        comp = [m for m in mates if m in angles and m not in der]
        if comp and not rel[s].get("accumulates") and R.get("grouping") == "rest":
            der[s] = ("updates in lockstep with a computed angle (an integral of a rate), not with any sensor "
                      "group", comp, 0.8)
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
    if mono > 0.999 and tcorr > 0.99:
        c = ramp(stepm, 0.8, 0.98)
        sc["counter"] = c
        sc["clock"] = 1 - c
        ev.append("never falls inside a segment and rises with time (r %.4f); equal steps on %.0f%% of rows"
                  % (tcorr, 100 * stepm))
        return sc, ev
    if f.get("binary"):
        sc["switch"] = 0.45
        ev.append("two values only")
        return sc, ev
    chz = _g(f, "change_hz", 0)
    nun = _g(f, "n_unique", 0)
    oneq = _g(f, "one_quantum_frac", 0)
    integer = bool(f.get("integer"))
    if chz < 0.3 and nun <= 120:
        b = ramp(chz, 0.05, 0.005) * ramp(abs(_fin(_g(f, "trend_corr", 0), 0) if False else abs(_g(f, "trend_corr", 0) or 0)), 0.3, 0.7)
        m = ramp(chz, 0.005, 0.05) * (0.9 if integer else 0.6)
        sc["battery"], sc["mode"] = b, min(1.0, m)
        ev.append("changes slowly (%.3f a second), %d levels, %.0f%% of changes are one step, trend over time r %.2f"
                  % (chz, nun, 100 * oneq, _g(f, "trend_corr", 0)))
        return sc, ev
    if s in der:
        why, partners, strength = der[s]
        sc["derived"] = 0.6 + 0.4 * ramp(strength, 0.8, 0.99)
        ev.append(why)
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

    # angle / rate pairs
    if s in integ or s in rate_of:
        is_angle = s in integ
        partner = integ[s][0] if is_angle else rate_of[s]
        r = integ[s][1] if is_angle else dict((b, r) for a, b, r in R["integration"])[s]
        rate_s = partner if is_angle else s
        angle_s = s if is_angle else partner
        fr = ctx["feats"][rate_s]
        twin = ctx["twin_pair"].get(angle_s)
        rate_flicker = _g(fr, "still_change_frac", np.nan)
        rg = R["groups"][gof[rate_s]] if rate_s in gof else [rate_s]
        imu_like = len(rg) >= 3 and (rate_flicker if np.isfinite(rate_flicker) else 0) > 0.05
        turn_ref = max(ctx["feats"][rate_s].get("cam_turn_int_excess", 0) or 0,
                       ctx["feats"][angle_s].get("cam_turn_ang_excess", 0) or 0, 0)
        drive_ref = max(abs(_g(ctx["feats"][angle_s], "cam_go_delta_corr", 0) or 0),
                        abs(_g(ctx["feats"][angle_s], "go_delta_corr", 0) or 0),
                        abs(_g(ctx["feats"][angle_s], "cam_speed_corr_rate", 0) or 0))
        wheel = (0.45 if twin else 0.0) + 0.35 * ramp(drive_ref, 0.3, 0.7) + (0.2 if not imu_like else 0.0)
        gyro = (0.45 if imu_like else 0.0) + 0.35 * ramp(turn_ref, 0.1, 0.4) + (0.2 if not twin else 0.0)
        ev.append("%s of a partner stream (integral relation r %.3f)" % ("integral" if is_angle else "rate", r))
        if twin:
            ev.append("its angle has a look-alike partner (a second wheel?)")
        if imu_like:
            ev.append("the rate flickers at rest together with %d other streams (an IMU-like group)" % (len(rg) - 1))
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

    # sensor groups
    imu = ctx["imu_group"]
    if imu is not None and s in imu["members"]:
        if s in imu["accel"]:
            sc["accel"] = 0.55 + 0.45 * imu["split_quality"]
            ev.append("in the group that flickers with the turn-following rate at rest; shares the noise floor of "
                      "the axis reading a steady %s at rest (gravity)" % ("level" if s != imu["gravity"] else "large level"))
        else:
            sc["gyro_rate"] = 0.55 + 0.45 * imu["split_quality"]
            ev.append("in the IMU-like group; shares the noise floor of the turn-following rate")
        if s in der:
            sc["derived"] *= 0.5
        return sc, ev
    if gsize >= 3 and not acc:
        pos = _g(f, "lo", -1) >= 0
        big = ramp(aor, 2.0, 6.0, nan=0.3)
        drive = max(ramp(-(camgo if np.isfinite(camgo) else 0), 0.2, 0.6),
                    ramp(abs(_g(f, "board_corr", 0) or 0), 0.3, 0.7))
        # the group as a whole: most members respond strongly to motion
        g_aor = np.nanmedian([_g(ctx["feats"][m], "active_over_rest", np.nan) for m in grp])
        gbig = ramp(g_aor, 2.0, 6.0, nan=0.3)
        moving = R.get("moving_frac", np.nan)
        motion_seen = ramp(moving, 0.03, 0.15, nan=0.5)
        sc["range"] = (0.35 * gbig + 0.25 * big + 0.25 * drive + 0.15 * pos) * (0.3 + 0.7 * motion_seen)
        sc["line"] = (0.4 * (1 - gbig) + 0.3 * (1 - big) + 0.2 * (1 if integer else 0) + 0.1 * pos) * \
                     (1 - 0.6 * drive) * (0.3 + 0.7 * motion_seen)
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
            # a combination inside a sensor group keeps the group's type; the derived note stays in the evidence
            sc["derived"] = 0.0
            ev.append("(computed from two neighbours in its group; same type, not separate evidence)")
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


def build_context(feats, R, der=None):
    """Case-wide structure the per-stream scores lean on: the IMU-like group and its split, look-alike pairs."""
    ctx = {"feats": feats, "twin_pair": {}, "imu_group": None, "derived": der or {}}
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
    def turnscore(s):
        f = feats[s]
        return _fin(_g(f, "cam_turn_int_excess", 0)) + abs(_fin(_g(f, "turn_mid_corr", 0))) + \
            (0.5 if s in rates else 0)
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
            if len(gr) >= 3 and rate not in gr:
                g_aor = np.nanmedian([_g(feats[m], "active_over_rest", np.nan) for m in gr])
                ints = np.mean([bool(feats[m].get("integer")) for m in gr])
                if (np.isfinite(g_aor) and g_aor >= 3) or ints > 0.5:
                    continue
            if s in ctx.get("derived", {}):
                continue
            mem.append(s)
        if rate not in mem:
            mem.append(rate)
        lv = {m: _g(feats[m], "level_over_noise", 0) or 0 for m in mem}
        grav = max([m for m in mem if m != rate], key=lambda m: lv[m]) if len(mem) > 1 else None
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
    return ctx


def classify_case(feats, R):
    """{stream: result dict}: type, confidence, decision, top3, evidence, derived note."""
    der = find_derived(feats, R)
    ctx = build_context(feats, R, der)
    out = {}
    for s, f in feats.items():
        sc, ev = score_stream(s, f, R, der, ctx)
        tot = sum(sc.values())
        ranked = sorted(sc.items(), key=lambda kv: -kv[1])
        best, second = ranked[0], ranked[1]
        conf = best[1] / tot if tot > 0 else 0.0
        out[s] = {"type": best[0], "confidence": float(conf), "scores": sc,
                  "top3": [(t, round(v / tot, 3)) for t, v in ranked[:3] if v > 0],
                  "evidence": ev, "derived": s in der, "derived_of": der[s][1] if s in der else [],
                  "decision": decide(conf) if best[0] != "unknown" else "unknown"}
    return out


def decide(conf, accept=ACCEPT, ask=ASK):
    if conf >= accept:
        return "accept"
    if conf >= ask:
        return "ask"
    return "unknown"
